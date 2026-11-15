// 手首を上げたら起こす（BMI270 wrist-wear wake-up）を試す実験用ファーム。
//
// imu_wake_l2 をもとにしている。違いは次の 2 点。
// - BMI270 の INT1 に any-motion ではなく wrist-wear wake-up だけをつなぐ（ジャイロは止める）。
// - 起きたら短く振動してログを出し、もう一度 L2 ディープスリープに戻る。何度でも試せる。
#include <cstdint>

#include "esp_attr.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "m5sw_board.hpp"
#include "m5sw_ioe1.hpp"
#include "m5sw_power.hpp"
#include "m5sw_rtc_wakeup.hpp"
#include "m5sw_test_support.hpp"
#include "m5sw_wakeup.hpp"

namespace {
constexpr char kTag[]               = "wrist_wake_l2";
constexpr TickType_t kPrepareDelay  = pdMS_TO_TICKS(3000);
constexpr std::uint16_t kMotorHz    = 5000;
constexpr std::uint8_t kMotorPct    = 80;
constexpr TickType_t kBuzzDuration  = pdMS_TO_TICKS(300);

// ディープスリープをまたいで残る起床回数（電源ボタンでのリセットでは 0 に戻る）。
RTC_DATA_ATTR std::uint32_t s_wake_count       = 0;
RTC_DATA_ATTR std::uint32_t s_wrist_wake_count = 0;

[[noreturn]] void fail(const char* stage, esp_err_t error, int raw_error)
{
    ESP_LOGE(kTag, "BOARD_FAIL stage=%s error=%s raw=%d", stage, esp_err_to_name(error), raw_error);
    m5sw::test_support::report_final(m5sw::test_support::FinalStatus::fail, "wrist wake L2 failed");
    m5sw::test_support::hold_safe_idle();
    for (;;) vTaskDelay(portMAX_DELAY);
}

// 振動モーターを短く鳴らす。失敗しても実験は続ける（ログだけ残す）。
void buzz(i2c_bus_handle_t bus, int times)
{
    static m5sw::power::Ioe1Adapter ioe1;
    if (!ioe1.initialized()) {
        const auto begun = ioe1.begin(bus, 100000);
        if (!begun.ok()) {
            ESP_LOGW(kTag, "BUZZ_SKIP stage=ioe1_begin error=%s", esp_err_to_name(begun.error));
            return;
        }
    }
    // 前回の L2 準備で IOE1 の I2C アイドルスリープ（1 秒）が有効なまま残っている。
    // UserDemo は sleep time 0 で使っているので、振動の間はスリープを止める。
    auto result = ioe1.configure_i2c(false, 100000);
    if (!result.ok()) ESP_LOGW(kTag, "BUZZ_WARN stage=ioe1_nosleep error=%s", esp_err_to_name(result.error));
    // UserDemo と full_load は、PWM の前にモーターのピンを GPIO 出力（Low）にしている。
    result = ioe1.set_pin(m5sw::board::IoePin::motor_pwm, false);
    if (!result.ok()) ESP_LOGW(kTag, "BUZZ_WARN stage=motor_pin error=%s", esp_err_to_name(result.error));
    // モーターは L3B 系統から給電される可能性がある（UserDemo の hal_ioe.cpp は L3B_EN を High にしてから振動させる）。
    // L2 の準備で L3B は切られるので、振動させる間だけ入れる。
    result = ioe1.set_pin(m5sw::board::IoePin::l3b_enable, true);
    if (!result.ok()) {
        ESP_LOGW(kTag, "BUZZ_SKIP stage=l3b_on error=%s", esp_err_to_name(result.error));
        return;
    }
    vTaskDelay(pdMS_TO_TICKS(20));
    result = ioe1.set_pwm_frequency(kMotorHz);
    if (!result.ok()) {
        ESP_LOGW(kTag, "BUZZ_SKIP stage=pwm_freq error=%s", esp_err_to_name(result.error));
        (void)ioe1.set_pin(m5sw::board::IoePin::l3b_enable, false);
        return;
    }
    for (int i = 0; i < times; ++i) {
        result = ioe1.set_motor_percent(kMotorPct);
        if (!result.ok()) ESP_LOGW(kTag, "BUZZ_WARN stage=motor_on error=%s", esp_err_to_name(result.error));
        vTaskDelay(kBuzzDuration);
        result = ioe1.set_motor_percent(0);
        if (!result.ok()) ESP_LOGW(kTag, "BUZZ_WARN stage=motor_off error=%s", esp_err_to_name(result.error));
        if (i + 1 < times) vTaskDelay(kBuzzDuration);
    }
    result = ioe1.stop_all_pwm();
    if (!result.ok()) ESP_LOGW(kTag, "BUZZ_WARN stage=pwm_stop error=%s", esp_err_to_name(result.error));
    result = ioe1.set_pin(m5sw::board::IoePin::l3b_enable, false);
    if (!result.ok()) ESP_LOGW(kTag, "BUZZ_WARN stage=l3b_off error=%s", esp_err_to_name(result.error));
    ESP_LOGI(kTag, "BUZZ x%d done", times);
}

}  // namespace

extern "C" void app_main(void)
{
    m5sw::test_support::TestContext context = {};
    ESP_ERROR_CHECK(m5sw::test_support::initialize(&context));
    esp_log_level_set("i2c_bus", ESP_LOG_ERROR);

    const esp_err_t i2c_result = m5sw::board::initialize_i2c();
    if (i2c_result != ESP_OK) fail("i2c", i2c_result, i2c_result);

    i2c_bus_handle_t bus         = nullptr;
    const esp_err_t claim_result = m5sw::board::acquire_i2c(&bus);
    if (claim_result != ESP_OK) fail("shared_bus", claim_result, claim_result);

    static m5sw::wakeup::RtcWakeup rtc;
    auto rtc_result = rtc.begin(bus);
    if (!rtc_result.ok()) fail("rtc_begin", rtc_result.error, rtc_result.raw_error);
    m5sw::wakeup::RtcFlags retained_rtc;
    rtc_result = rtc.read_flags(&retained_rtc);
    if (!rtc_result.ok()) fail("rtc_flags", rtc_result.error, rtc_result.raw_error);

    // BMI270 の初期化で割り込みステータスが消えるので、その前に読んでおく。
    m5sw::wakeup::ImuStatus retained_imu;
    auto imu_result = m5sw::wakeup::ImuWakeup::read_retained_status(bus, &retained_imu);
    if (!imu_result.ok()) fail("imu_retained", imu_result.error, imu_result.raw_error);

    static m5sw::wakeup::WakeController wake;
    auto wake_result = wake.begin(bus);
    if (!wake_result.ok()) fail("wake_begin", wake_result.error, wake_result.raw_error);
    m5sw::wakeup::WakeSnapshot snapshot;
    wake_result = wake.capture(retained_rtc, retained_imu, &snapshot);
    if (!wake_result.ok()) fail("wake_capture", wake_result.error, wake_result.raw_error);

    if (context.wake_cause == ESP_SLEEP_WAKEUP_UNDEFINED) {
        s_wake_count       = 0;
        s_wrist_wake_count = 0;
        ESP_LOGI(kTag, "COLD_BOOT: counters reset");
        buzz(bus, 1);
    } else {
        ++s_wake_count;
        if (snapshot.imu_wrist_wear) ++s_wrist_wake_count;
        ESP_LOGI(kTag,
                 "WAKE #%lu (wrist=%lu) at %lld ms: esp=%s imu_raw=0x%04X wrist=%d any=%d "
                 "pm1_gpio_irq=0x%02X btn=0x%02X",
                 static_cast<unsigned long>(s_wake_count), static_cast<unsigned long>(s_wrist_wake_count),
                 esp_timer_get_time() / 1000, m5sw::test_support::wake_cause_name(context.wake_cause),
                 retained_imu.raw, snapshot.imu_wrist_wear, snapshot.imu_any_motion, snapshot.pm1_gpio_irq,
                 snapshot.pm1_button_irq);
        // 手首で起きたら 1 回、それ以外の理由で起きたら 2 回振動する。
        buzz(bus, snapshot.imu_wrist_wear ? 1 : 2);
    }

    wake_result = wake.clear_snapshot(snapshot);
    if (!wake_result.ok()) fail("wake_clear", wake_result.error, wake_result.raw_error);
    rtc_result = rtc.clear_flags(retained_rtc);
    if (!rtc_result.ok()) fail("rtc_clear", rtc_result.error, rtc_result.raw_error);
    rtc_result = rtc.disable_interrupts();
    if (!rtc_result.ok()) fail("rtc_disable", rtc_result.error, rtc_result.raw_error);

    static m5sw::power::PowerController power;
    auto power_result = power.begin(bus);
    if (!power_result.ok()) fail("pm1_begin", power_result.error, power_result.raw_error);
    power_result = power.prepare_l2(m5sw::power::WakeSelection::rtc_imu);
    if (!power_result.ok()) fail("l2_prepare", power_result.error, power_result.raw_error);

    wake_result = wake.select_source(m5sw::wakeup::WakeSelection::imu, m5sw::wakeup::ImuWakeProfile::wrist_wear);
    if (!wake_result.ok()) fail("imu_arm", wake_result.error, wake_result.raw_error);

    ESP_LOGI(kTag, "WAKE_ARMED source=wrist_wear path=pm1_gpio0+esp_ext0");
    ESP_ERROR_CHECK(m5sw::test_support::mark_test_ready());
    vTaskDelay(kPrepareDelay);

    power_result = power.enter_l2(m5sw::power::WakeSelection::rtc_imu);
    fail("l2_returned", power_result.error, power_result.raw_error);
}
