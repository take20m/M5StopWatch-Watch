// 「手首を上げたら起こす」の方式を比べる診断用ファーム（v2）。ディープスリープには入らず、起きたまま動く。
//
// 同じ動作に対して、次の 3 つの検出が反応したかを同時にログに出す。
//   WRIST : BMI270 の wrist-wear wake-up（Bosch の手首検出。remap 2 を適用）
//   ANY   : BMI270 の any-motion（動き検出。ログだけ、振動しない）
//   LOOK  : ESP32 側のソフト判定。文字盤が上を向いた姿勢（センサーの +Z が上）で一定時間止まったら反応する
//
// 振動: WRIST は長く 1 回、LOOK は短く 2 回。
// KEYA（G2）を押すとログに MARK を出す（「今のは見逃された」などの印に使う）。
#include <cmath>
#include <cstdint>

#include "bmi270.h"
#include "bmi270_tools.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "m5sw_board.hpp"
#include "m5sw_ioe1.hpp"
#include "m5sw_test_support.hpp"

namespace {
constexpr char kTag[] = "raise_diag";

// 午前の測定でいちばん反応がよかった軸の付け替え（x=-X, y=-Y, z=Z）。
constexpr bmi2_remap kRemap = {BMI2_NEG_X, BMI2_NEG_Y, BMI2_Z};

// any-motion: 100ms 続く、約 48mg 以上の変化で反応。
constexpr std::uint8_t kAnyDuration  = 5;    // LSB = 20ms
constexpr std::uint8_t kAnyThreshold = 100;  // LSB = 0.48mg

// LOOK 判定（単位 g、センサー座標。+Z は文字盤から外向き）。
constexpr float kLookEnterZ      = 0.60f;  // 文字盤が上から約 53° 以内
constexpr float kLookExitZ       = 0.35f;  // ここより下がったら「見ていない」に戻る
constexpr float kStillTolerance  = 0.15f;  // |a| が 1g ± これ以内なら止まっているとみなす
constexpr std::int64_t kLookHoldUs = 250000;  // この時間、姿勢が続いたら LOOK

m5sw::power::Ioe1Adapter s_ioe1;
bool s_ioe1_ready = false;

[[noreturn]] void fail(const char* stage, esp_err_t error)
{
    ESP_LOGE(kTag, "BOARD_FAIL stage=%s error=%s", stage, esp_err_to_name(error));
    m5sw::test_support::hold_safe_idle();
    for (;;) vTaskDelay(portMAX_DELAY);
}

void prepare_motor(i2c_bus_handle_t bus)
{
    auto result = s_ioe1.begin(bus, 100000);
    if (!result.ok()) {
        ESP_LOGW(kTag, "motor disabled: ioe1_begin error=%s", esp_err_to_name(result.error));
        return;
    }
    // 振動が確認できた手順: I2C スリープ停止 → モーターピンを出力 Low → L3B ON。
    result = s_ioe1.configure_i2c(false, 100000);
    if (!result.ok()) ESP_LOGW(kTag, "ioe1 nosleep error=%s", esp_err_to_name(result.error));
    result = s_ioe1.set_pin(m5sw::board::IoePin::motor_pwm, false);
    if (!result.ok()) ESP_LOGW(kTag, "motor pin error=%s", esp_err_to_name(result.error));
    result = s_ioe1.set_pin(m5sw::board::IoePin::l3b_enable, true);
    if (!result.ok()) ESP_LOGW(kTag, "l3b on error=%s", esp_err_to_name(result.error));
    result = s_ioe1.set_pwm_frequency(5000);
    if (!result.ok()) ESP_LOGW(kTag, "pwm freq error=%s", esp_err_to_name(result.error));
    s_ioe1_ready = true;
}

void buzz(int times, int on_ms, int off_ms = 150)
{
    if (!s_ioe1_ready) return;
    for (int i = 0; i < times; ++i) {
        const auto on = s_ioe1.set_motor_percent(100);
        if (!on.ok()) ESP_LOGW(kTag, "BUZZ_WARN motor_on error=%s", esp_err_to_name(on.error));
        vTaskDelay(pdMS_TO_TICKS(on_ms));
        const auto off = s_ioe1.set_motor_percent(0);
        if (!off.ok()) ESP_LOGW(kTag, "BUZZ_WARN motor_off error=%s", esp_err_to_name(off.error));
        if (i + 1 < times) vTaskDelay(pdMS_TO_TICKS(off_ms));
    }
}

esp_err_t read_int_status(bmi270_tools* imu, std::uint16_t* status)
{
    std::uint8_t data[2] = {};
    const esp_err_t result = i2c_bus_read_bytes(imu->device(), BMI2_INT_STATUS_0_ADDR, sizeof(data), data);
    if (result == ESP_OK) *status = static_cast<std::uint16_t>(data[0]) | (static_cast<std::uint16_t>(data[1]) << 8);
    return result;
}

}  // namespace

extern "C" void app_main(void)
{
    m5sw::test_support::TestContext context = {};
    ESP_ERROR_CHECK(m5sw::test_support::initialize(&context));
    esp_log_level_set("i2c_bus", ESP_LOG_ERROR);
    esp_log_level_set("M5IOE1_PWM", ESP_LOG_WARN);

    const esp_err_t i2c_result = m5sw::board::initialize_i2c();
    if (i2c_result != ESP_OK) fail("i2c", i2c_result);
    i2c_bus_handle_t bus = nullptr;
    const esp_err_t claim_result = m5sw::board::acquire_i2c(&bus);
    if (claim_result != ESP_OK) fail("shared_bus", claim_result);

    prepare_motor(bus);

    static bmi270_tools imu;
    esp_err_t result = imu.init(bus, nullptr, false, bmi270_tools::MODE_BASE);
    if (result != ESP_OK) fail("imu_init", result);

    // wrist_wake_l2 と同じ省電力の加速度設定（50Hz、ジャイロ停止）。
    const bmi270_tools::sensor_config_t sensor_config = {
        bmi270_tools::ACCEL_RANGE_2G,
        bmi270_tools::GYRO_RANGE_2000,
        bmi270_tools::ODR_50HZ,
        bmi270_tools::ODR_25HZ,
    };
    const bmi270_tools::sensor_performance_config_t performance_config = {
        BMI2_POWER_OPT_MODE, BMI2_ACC_OSR4_AVG1, BMI2_POWER_OPT_MODE,
        BMI2_POWER_OPT_MODE, BMI2_GYR_OSR4_MODE, BMI2_GYR_OIS_2000,
    };
    result = imu.configure_sensors(sensor_config, performance_config);
    if (result != ESP_OK) fail("imu_config", result);

    if (bmi2_set_remap_axes(&kRemap, imu.bmi2_device()) != BMI2_OK) fail("remap", ESP_FAIL);

    const bmi270_tools::sensor_type_t sensors[] = {
        bmi270_tools::SENSOR_ACCEL,
        static_cast<bmi270_tools::sensor_type_t>(BMI2_WRIST_WEAR_WAKE_UP),
        static_cast<bmi270_tools::sensor_type_t>(BMI2_ANY_MOTION),
    };
    result = imu.enable_sensors(sensors, 3U);
    if (result != ESP_OK) fail("imu_enable", result);
    result = imu.configure_any_motion({kAnyDuration, kAnyThreshold});
    if (result != ESP_OK) fail("any_config", result);
    result = imu.map_interrupt_to_pin(BMI2_WRIST_WEAR_WAKE_UP, bmi270_tools::INT_PIN_1);
    if (result != ESP_OK) fail("wrist_map", result);
    result = imu.map_interrupt_to_pin(BMI2_ANY_MOTION, bmi270_tools::INT_PIN_1);
    if (result != ESP_OK) fail("any_map", result);
    result = imu.enable_interrupt(bmi270_tools::INT_PIN_1, true, false, false);
    if (result != ESP_OK) fail("imu_int", result);

    gpio_reset_pin(m5sw::board::kKey1);
    gpio_set_direction(m5sw::board::kKey1, GPIO_MODE_INPUT);
    gpio_set_pull_mode(m5sw::board::kKey1, GPIO_PULLUP_ONLY);

    ESP_LOGI(kTag, "DIAG_READY v2: WRIST=long buzz, LOOK=2 short buzz, ANY=log only, KEYA=MARK");
    buzz(1, 300);

    std::uint32_t wrist_count = 0, look_count = 0, any_count = 0, mark_count = 0;
    bool key_was_down = false;
    std::int64_t last_print_us = 0;
    float sx = 0, sy = 0, sz = 0;
    int samples = 0;

    // LOOK 判定の状態。
    bool look_armed = true;         // 一度「見ていない」姿勢に戻るまで次の LOOK は出さない
    std::int64_t pose_since_us = -1;
    std::int64_t last_any_us = 0;   // 直近の ANY（LOOK までの経過時間を出すため）

    for (;;) {
        const std::int64_t now = esp_timer_get_time();
        bmi270_tools::sensor_data_t data = {};
        if (imu.get_sensor_data(data) == ESP_OK && data.acc_valid) {
            const float ax = data.acc_x / 9.80665f, ay = data.acc_y / 9.80665f, az = data.acc_z / 9.80665f;
            sx += ax;
            sy += ay;
            sz += az;
            ++samples;

            const float norm  = std::sqrt(ax * ax + ay * ay + az * az);
            const bool still  = std::fabs(norm - 1.0f) <= kStillTolerance;
            const bool in_pose = az >= kLookEnterZ && still;
            if (az < kLookExitZ) {
                look_armed    = true;
                pose_since_us = -1;
            } else if (in_pose) {
                if (pose_since_us < 0) pose_since_us = now;
                if (look_armed && now - pose_since_us >= kLookHoldUs) {
                    look_armed = false;
                    ++look_count;
                    ESP_LOGI(kTag, "LOOK #%lu  acc=(%+.2f %+.2f %+.2f)  since_any=%lldms",
                             static_cast<unsigned long>(look_count), ax, ay, az,
                             last_any_us ? (now - last_any_us) / 1000 : -1LL);
                    buzz(2, 100);
                }
            } else {
                pose_since_us = -1;
            }
        }

        if (now - last_print_us >= 500000 && samples > 0) {
            ESP_LOGI(kTag, "ACC x=%+.2f y=%+.2f z=%+.2f  wrist=%lu look=%lu any=%lu", sx / samples, sy / samples,
                     sz / samples, static_cast<unsigned long>(wrist_count), static_cast<unsigned long>(look_count),
                     static_cast<unsigned long>(any_count));
            sx = sy = sz = 0;
            samples = 0;
            last_print_us = now;
        }

        // INT_STATUS_0 は読むと消える。
        std::uint16_t status = 0;
        if (read_int_status(&imu, &status) == ESP_OK && status != 0) {
            if (status & BMI270_ANY_MOT_STATUS_MASK) {
                ++any_count;
                last_any_us = now;
                ESP_LOGI(kTag, "ANY #%lu", static_cast<unsigned long>(any_count));
            }
            if (status & BMI270_WRIST_WAKE_UP_STATUS_MASK) {
                ++wrist_count;
                ESP_LOGI(kTag, "WRIST #%lu (status=0x%04X)", static_cast<unsigned long>(wrist_count), status);
                buzz(1, 400);
            }
        }

        const bool key_down = gpio_get_level(m5sw::board::kKey1) == 0;
        if (key_down && !key_was_down) {
            ++mark_count;
            ESP_LOGI(kTag, "MARK #%lu", static_cast<unsigned long>(mark_count));
        }
        key_was_down = key_down;

        vTaskDelay(pdMS_TO_TICKS(20));
    }
}
