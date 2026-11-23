#include "m5sw_power.hpp"

#include <initializer_list>

#include "driver/gpio.h"
#include "driver/rtc_io.h"
#include "esp_attr.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "m5sw_board.hpp"
#include "soc/rtc_io_reg.h"

namespace m5sw::power {
namespace {

constexpr TickType_t kL2IdleSleepDelay     = pdMS_TO_TICKS(1000);
RTC_DATA_ATTR bool s_quick_wake_armed      = false;
constexpr TickType_t kTouchResetLowDelay   = pdMS_TO_TICKS(10);
constexpr TickType_t kTouchResetReadyDelay = pdMS_TO_TICKS(50);

AdapterProbeResult probe_pm1(i2c_bus_handle_t bus)
{
    Pm1Adapter pm1;
    DriverResult result = pm1.begin(bus, 100000);
    if (!result.ok()) return {result.error, "pm1_begin", result.raw_error};

    std::uint8_t power_config = 0;
    result                    = pm1.read_power_config(&power_config);
    if (!result.ok()) return {result.error, "pm1_power_read", result.raw_error};
    result = pm1.set_power_config(M5PM1_PWR_CFG_CHG_EN, power_config & M5PM1_PWR_CFG_CHG_EN);
    if (!result.ok()) return {result.error, "pm1_power_verify", result.raw_error};

    bool ldo_hold = false;
    result        = pm1.read_ldo_hold(&ldo_hold);
    if (!result.ok()) return {result.error, "pm1_ldo_read", result.raw_error};
    result = pm1.set_ldo_hold(ldo_hold);
    if (!result.ok()) return {result.error, "pm1_ldo_verify", result.raw_error};

    result = pm1.configure_gpio_wake(Pm1Gpio::rtc_imu);
    if (!result.ok()) return {result.error, "pm1_gpio0_wake", result.raw_error};
    result = pm1.configure_gpio_wake(Pm1Gpio::port);
    if (!result.ok()) return {result.error, "pm1_gpio4_wake", result.raw_error};
    result = pm1.restore_gpio_input(Pm1Gpio::rtc_imu);
    if (!result.ok()) return {result.error, "pm1_gpio0_restore", result.raw_error};
    result = pm1.restore_gpio_input(Pm1Gpio::port);
    if (!result.ok()) return {result.error, "pm1_gpio4_restore", result.raw_error};

    std::uint8_t wake_source = 0;
    result                   = pm1.read_wake_source(&wake_source);
    if (!result.ok()) return {result.error, "pm1_wake_read", result.raw_error};
    if (wake_source != 0) {
        result = pm1.clear_wake_source(wake_source);
        if (!result.ok()) return {result.error, "pm1_wake_clear", result.raw_error};
    }

    std::uint8_t gpio_irq = 0;
    result                = pm1.read_gpio_irq(&gpio_irq);
    if (!result.ok()) return {result.error, "pm1_irq_read", result.raw_error};
    if (gpio_irq != 0) {
        result = pm1.clear_gpio_irq(gpio_irq);
        if (!result.ok()) return {result.error, "pm1_irq_clear", result.raw_error};
    }

    result = pm1.set_i2c_sleep_time(0);
    if (!result.ok()) return {result.error, "pm1_i2c_sleep", result.raw_error};
    return {};
}

AdapterProbeResult probe_ioe1(i2c_bus_handle_t bus)
{
    Ioe1Adapter ioe1;
    Ioe1Result result = ioe1.begin(bus, 100000);
    if (!result.ok()) return {result.error, "ioe1_begin", result.raw_error};
    result = ioe1.configure_i2c(false, 100000);
    if (!result.ok()) return {result.error, "ioe1_100k", result.raw_error};

    constexpr struct {
        m5sw::board::IoePin pin;
        bool safe_level;
    } kSafePins[] = {
        {m5sw::board::IoePin::mux_select, false},  {m5sw::board::IoePin::audio_enable, false},
        {m5sw::board::IoePin::touch_reset, false}, {m5sw::board::IoePin::display_reset, false},
        {m5sw::board::IoePin::l3b_enable, false},  {m5sw::board::IoePin::speaker_enable, false},
    };
    for (const auto& safe_pin : kSafePins) {
        result = ioe1.set_pin(safe_pin.pin, safe_pin.safe_level);
        if (!result.ok()) return {result.error, "ioe1_pin", result.raw_error};
        bool actual = true;
        result      = ioe1.read_pin(safe_pin.pin, &actual);
        if (!result.ok() || actual != safe_pin.safe_level) {
            return {result.ok() ? ESP_ERR_INVALID_RESPONSE : result.error, "ioe1_pin_readback",
                    result.ok() ? M5IOE1_FAIL : result.raw_error};
        }
    }

    result = ioe1.set_motor_percent(0);
    if (!result.ok()) return {result.error, "ioe1_motor_0", result.raw_error};
    result = ioe1.set_motor_percent(50);
    if (!result.ok()) return {result.error, "ioe1_motor_50", result.raw_error};
    result = ioe1.set_motor_percent(100);
    if (!result.ok()) return {result.error, "ioe1_motor_100", result.raw_error};
    result = ioe1.set_motor_percent(0);
    if (!result.ok()) return {result.error, "ioe1_motor_restore", result.raw_error};
    result = ioe1.stop_all_pwm();
    if (!result.ok()) return {result.error, "ioe1_pwm_stop", result.raw_error};
    result = ioe1.probe_idle_sleep();
    if (!result.ok()) return {result.error, "ioe1_i2c_sleep", result.raw_error};
    return {};
}

}  // namespace

AdapterProbeResult run_adapter_probe(i2c_bus_handle_t bus)
{
    if (bus == nullptr) return {ESP_ERR_INVALID_ARG, "bus", ESP_ERR_INVALID_ARG};
    AdapterProbeResult result = probe_pm1(bus);
    if (!result.ok()) return result;
    return probe_ioe1(bus);
}

PowerResult PowerController::from_pm1(DriverResult result) const
{
    return {result.error, result.raw_error};
}

PowerResult PowerController::from_ioe1(Ioe1Result result) const
{
    return {result.error, result.raw_error};
}

PowerResult PowerController::begin(i2c_bus_handle_t bus, std::uint32_t speed_hz)
{
    if (bus == nullptr) return {ESP_ERR_INVALID_ARG, M5PM1_ERR_INVALID_ARG};
    bus_                     = bus;
    const PowerResult result = from_pm1(pm1_.begin(bus, speed_hz));
    initialized_             = result.ok();
    l1_prepared_             = false;
    l1_prepared_wake_        = WakeSelection::none;
    l2_prepared_             = false;
    l2_prepared_wake_        = WakeSelection::none;
    return result;
}

PowerResult PowerController::begin_ioe1()
{
    if (!initialized_) return {ESP_ERR_INVALID_STATE, M5PM1_ERR_NOT_INIT};
    if (ioe1_initialized_ && ioe1_.initialized()) return {};
    ioe1_initialized_  = false;
    PowerResult result = from_ioe1(ioe1_.begin(bus_, 100000));
    if (!result.ok()) return result;
    result = from_ioe1(ioe1_.configure_i2c(false, 100000));
    if (!result.ok()) return result;
    ioe1_initialized_ = true;
    return {};
}

PowerResult PowerController::prepare_l0()
{
    if (!initialized_) return {ESP_ERR_INVALID_STATE, M5PM1_ERR_NOT_INIT};
    l1_prepared_       = false;
    l2_prepared_       = false;
    PowerResult result = from_pm1(pm1_.disable_gpio_wake(Pm1Gpio::rtc_imu));
    if (!result.ok()) return result;
    result = from_pm1(pm1_.disable_gpio_wake(Pm1Gpio::port));
    if (!result.ok()) return result;
    result = from_pm1(pm1_.set_power_config(M5PM1_PWR_CFG_CHG_EN | M5PM1_PWR_CFG_BOOST_EN, 0));
    if (!result.ok()) return result;
    return from_pm1(pm1_.set_ldo_hold(false));
}

PowerResult PowerController::prepare_l1(WakeSelection wake)
{
    if (!initialized_) return {ESP_ERR_INVALID_STATE, M5PM1_ERR_NOT_INIT};
    l1_prepared_       = false;
    l2_prepared_       = false;
    PowerResult result = from_pm1(pm1_.disable_gpio_wake(Pm1Gpio::rtc_imu));
    if (!result.ok()) return result;
    result = from_pm1(pm1_.disable_gpio_wake(Pm1Gpio::port));
    if (!result.ok()) return result;
    if (wake == WakeSelection::rtc_imu) {
        result = from_pm1(pm1_.configure_gpio_wake(Pm1Gpio::rtc_imu));
    } else if (wake == WakeSelection::port) {
        result = from_pm1(pm1_.configure_gpio_wake(Pm1Gpio::port));
    }
    if (!result.ok()) return result;
    result = from_pm1(pm1_.set_power_config(M5PM1_PWR_CFG_CHG_EN | M5PM1_PWR_CFG_BOOST_EN, 0));
    if (!result.ok()) return result;
    result = from_pm1(pm1_.set_ldo_hold(true));
    if (!result.ok()) return result;
    l1_prepared_      = true;
    l1_prepared_wake_ = wake;
    return {};
}

PowerResult PowerController::configure_l2_wake(WakeSelection wake)
{
    // PM1 GPIO wake and ESP32 ext0 must be configured as one chain: PM1 owns
    // the board-level source, while GPIO12 is the processor-side wake input.
    PowerResult result = from_pm1(pm1_.configure_all_gpio_safe());
    if (!result.ok()) return result;
    result = from_pm1(pm1_.mask_non_gpio_irqs());
    if (!result.ok()) return result;
    if (wake != WakeSelection::none) {
        result = from_pm1(pm1_.configure_irq_output());
        if (!result.ok()) return result;
    }
    if (wake == WakeSelection::rtc_imu) {
        result = from_pm1(pm1_.configure_gpio_wake(Pm1Gpio::rtc_imu));
    } else if (wake == WakeSelection::port) {
        result = from_pm1(pm1_.configure_gpio_wake(Pm1Gpio::port));
    }
    if (!result.ok()) return result;

    esp_sleep_disable_wakeup_source(ESP_SLEEP_WAKEUP_ALL);
    if (wake != WakeSelection::none) {
        // Prepare GPIO12 before both the pre-sleep level check and ext0 handoff.
        esp_err_t sleep_result = gpio_reset_pin(m5sw::board::kPm1Irq);
        if (sleep_result != ESP_OK) return {sleep_result, sleep_result};
        sleep_result = rtc_gpio_pulldown_dis(m5sw::board::kPm1Irq);
        if (sleep_result != ESP_OK) return {sleep_result, sleep_result};
        sleep_result = rtc_gpio_pullup_en(m5sw::board::kPm1Irq);
        if (sleep_result != ESP_OK) return {sleep_result, sleep_result};
        sleep_result = esp_sleep_enable_ext0_wakeup(m5sw::board::kPm1Irq, 0);
        if (sleep_result != ESP_OK) return {sleep_result, sleep_result};
    }
    return {};
}

PowerResult PowerController::prepare_l2(WakeSelection wake, bool keep_charging)
{
    if (!initialized_) return {ESP_ERR_INVALID_STATE, M5PM1_ERR_NOT_INIT};
    l1_prepared_ = false;
    l2_prepared_ = false;

    // Match the reference L2 sequence: do not load the BMI270 feature
    // configuration before suspend. Loading it temporarily disables advanced
    // power save and can leave the feature engine in a different state.
    if (wake == WakeSelection::none) {
        i2c_bus_device_handle_t imu = i2c_bus_device_create(bus_, m5sw::board::kImuAddress, 100000);
        if (imu == nullptr) return {ESP_ERR_NO_MEM, ESP_ERR_NO_MEM};
        esp_err_t peripheral_result = i2c_bus_write_byte(imu, 0x7C, 0x01);
        if (peripheral_result != ESP_OK) return {peripheral_result, peripheral_result};
        vTaskDelay(pdMS_TO_TICKS(100));
        peripheral_result = i2c_bus_write_byte(imu, 0x7D, 0x00);
        if (peripheral_result != ESP_OK) return {peripheral_result, peripheral_result};
        std::uint8_t power_conf = 0;
        std::uint8_t power_ctrl = 0;
        peripheral_result       = i2c_bus_read_byte(imu, 0x7C, &power_conf);
        if (peripheral_result != ESP_OK) return {peripheral_result, peripheral_result};
        peripheral_result = i2c_bus_read_byte(imu, 0x7D, &power_ctrl);
        if (peripheral_result != ESP_OK) return {peripheral_result, peripheral_result};
        if (power_conf != 0x01U || power_ctrl != 0x00U) {
            return {ESP_ERR_INVALID_RESPONSE, ESP_ERR_INVALID_RESPONSE};
        }
    }

    PowerResult result = begin_ioe1();
    if (!result.ok()) return result;

    // A USB reset does not power-cycle the L1 domain. Wake CST820 through its
    // hardware reset before sending the sleep command again, matching the
    // reference cst820_init() sequence and making L2 preparation repeatable.
    result = from_ioe1(ioe1_.set_pin(m5sw::board::IoePin::touch_reset, false));
    if (!result.ok()) return result;
    vTaskDelay(kTouchResetLowDelay);
    result = from_ioe1(ioe1_.set_pin(m5sw::board::IoePin::touch_reset, true));
    if (!result.ok()) return result;
    vTaskDelay(kTouchResetReadyDelay);

    // Reset retained IOE1 function state before applying the L2-safe board
    // configuration. The reset invalidates the driver, so begin it again.
    result = from_ioe1(ioe1_.factory_reset());
    if (!result.ok()) return result;
    ioe1_initialized_ = false;
    result            = begin_ioe1();
    if (!result.ok()) return result;

    // Keep the factory-reset GPIO state intact. The reference L2 path leaves
    // these pins high impedance; driving reset/control lines low can sink
    // current continuously through board-level pull-ups.
    i2c_bus_device_handle_t touch = i2c_bus_device_create(bus_, m5sw::board::kTouchAddress, 100000);
    if (touch == nullptr) return {ESP_ERR_NO_MEM, ESP_ERR_NO_MEM};
    vTaskDelay(pdMS_TO_TICKS(100));
    if (!touch_wake_) {
        const esp_err_t touch_result = i2c_bus_write_byte(touch, 0xE5, 0x03);
        if (touch_result != ESP_OK) return {touch_result, touch_result};
    }

    result = from_ioe1(ioe1_.stop_all_pwm());
    if (!result.ok()) return result;
    // Match the reference L2 rail state exactly: keep DCDC/LDO alive for the
    // RTC/IMU retention domain and disable charge, BOOST/5V and LED output.
    constexpr std::uint8_t kL2PowerMask = M5PM1_PWR_CFG_CHG_EN | M5PM1_PWR_CFG_DCDC_EN | M5PM1_PWR_CFG_LDO_EN |
                                          M5PM1_PWR_CFG_BOOST_EN | M5PM1_PWR_CFG_LED_CTRL;
    constexpr std::uint8_t kL2PowerValue = M5PM1_PWR_CFG_DCDC_EN | M5PM1_PWR_CFG_LDO_EN;
    const std::uint8_t l2_power_value =
        keep_charging ? static_cast<std::uint8_t>(kL2PowerValue | M5PM1_PWR_CFG_CHG_EN) : kL2PowerValue;
    result = from_pm1(pm1_.set_power_config(kL2PowerMask, l2_power_value));
    if (!result.ok()) return result;
    result = from_pm1(pm1_.set_watchdog_timeout(0));
    if (!result.ok()) return result;
    result = from_pm1(pm1_.set_ldo_hold(true));
    if (!result.ok()) return result;
    result = configure_l2_wake(wake);
    if (!result.ok()) return result;

    rtc_gpio_isolate(m5sw::board::kKey2);
    rtc_gpio_isolate(m5sw::board::kKey1);
    rtc_gpio_isolate(m5sw::board::kTouchIrq);
    if (wake == WakeSelection::none) {
        rtc_gpio_isolate(m5sw::board::kPm1Irq);  // PM1/IOE1 IRQ
    }

    l2_prepared_      = true;
    l2_prepared_wake_ = wake;
    return {};
}

PowerResult PowerController::finalize_l2_peripherals(L2Pm1IdleSleep pm1_idle_sleep)
{
    PowerResult result = from_ioe1(ioe1_.set_idle_sleep(1));
    if (!result.ok()) return result;
    if (pm1_idle_sleep == L2Pm1IdleSleep::enabled) {
        result = from_pm1(pm1_.set_i2c_sleep_time(1));
        if (!result.ok()) return result;
    }
    return {};
}

PowerResult PowerController::prepare_l2_active()
{
    PowerResult result = prepare_l2(WakeSelection::none);
    if (!result.ok()) return result;
    return finalize_l2_peripherals(L2Pm1IdleSleep::enabled);
}

PowerResult PowerController::enter_l0()
{
    PowerResult result = prepare_l0();
    if (!result.ok()) return result;
    return from_pm1(pm1_.shutdown());
}

PowerResult PowerController::power_off_keep_charging()
{
    if (!initialized_) return {ESP_ERR_INVALID_STATE, M5PM1_ERR_NOT_INIT};
    l1_prepared_       = false;
    l2_prepared_       = false;
    PowerResult result = from_pm1(pm1_.disable_gpio_wake(Pm1Gpio::rtc_imu));
    if (!result.ok()) return result;
    result = from_pm1(pm1_.disable_gpio_wake(Pm1Gpio::port));
    if (!result.ok()) return result;
    result = from_pm1(pm1_.set_power_config(M5PM1_PWR_CFG_CHG_EN | M5PM1_PWR_CFG_BOOST_EN, M5PM1_PWR_CFG_CHG_EN));
    if (!result.ok()) return result;
    result = from_pm1(pm1_.set_ldo_hold(false));
    if (!result.ok()) return result;
    return from_pm1(pm1_.shutdown());
}

PowerResult PowerController::enter_l1(WakeSelection wake)
{
    if (!l1_prepared_ || l1_prepared_wake_ != wake) {
        const PowerResult result = prepare_l1(wake);
        if (!result.ok()) return result;
    }
    return from_pm1(pm1_.shutdown());
}

PowerResult PowerController::enter_l2(WakeSelection wake, L2Pm1IdleSleep pm1_idle_sleep)
{
    if (!l2_prepared_ || l2_prepared_wake_ != wake) {
        const PowerResult result = prepare_l2(wake);
        if (!result.ok()) return result;
    }

    // Match the validated reference sequence: idle-sleep is the final I2C
    // operation. No PM1/IOE1 transaction is allowed during the one-second
    // settling interval before the ESP32 enters Deep Sleep.
    PowerResult result = finalize_l2_peripherals(pm1_idle_sleep);
    if (!result.ok()) return result;
    vTaskDelay(kL2IdleSleepDelay);
    esp_deep_sleep_start();
    return {ESP_FAIL, ESP_FAIL};
}

PowerResult PowerController::read_battery_mv(std::uint16_t* millivolts)
{
    if (!initialized_) return {ESP_ERR_INVALID_STATE, M5PM1_ERR_NOT_INIT};
    return from_pm1(pm1_.read_battery_mv(millivolts));
}

PowerResult PowerController::read_vin_mv(std::uint16_t* millivolts)
{
    if (!initialized_) return {ESP_ERR_INVALID_STATE, M5PM1_ERR_NOT_INIT};
    return from_pm1(pm1_.read_vin_mv(millivolts));
}

namespace {
constexpr std::uint32_t kTouchLatchBit = 1u << (RTC_GPIO_STATUS_INT_S + 13);  // GPIO13 = RTC GPIO13
static_assert(m5sw::board::kTouchIrq == GPIO_NUM_13, "touch latch assumes GPIO13");

void arm_touch_latch()
{
    REG_SET_FIELD(RTC_GPIO_PIN13_REG, RTC_GPIO_PIN13_INT_TYPE, 2);  // 立ち下がり
    REG_CLR_BIT(RTC_GPIO_PIN13_REG, RTC_GPIO_PIN13_WAKEUP_ENABLE);
    REG_WRITE(RTC_GPIO_STATUS_W1TC_REG, kTouchLatchBit);
}

bool touch_latched() { return (REG_READ(RTC_GPIO_STATUS_REG) & kTouchLatchBit) != 0; }
}  // namespace

bool PowerController::quick_wake_armed() { return s_quick_wake_armed; }

bool PowerController::take_touch_latch()
{
    const bool seen = touch_latched();
    REG_SET_FIELD(RTC_GPIO_PIN13_REG, RTC_GPIO_PIN13_INT_TYPE, 0);
    REG_WRITE(RTC_GPIO_STATUS_W1TC_REG, kTouchLatchBit);
    return seen;
}

PowerResult PowerController::enter_l2_quick(WakeSelection wake, std::uint64_t timer_fallback_us,
                                            L2Pm1IdleSleep pm1_idle_sleep, std::uint64_t key_wake_mask)
{
    if (!initialized_) return {ESP_ERR_INVALID_STATE, M5PM1_ERR_NOT_INIT};
    esp_sleep_disable_wakeup_source(ESP_SLEEP_WAKEUP_ALL);
    if (wake != WakeSelection::none) {
        esp_err_t sleep_result = gpio_reset_pin(m5sw::board::kPm1Irq);
        if (sleep_result != ESP_OK) return {sleep_result, sleep_result};
        sleep_result = rtc_gpio_pulldown_dis(m5sw::board::kPm1Irq);
        if (sleep_result != ESP_OK) return {sleep_result, sleep_result};
        sleep_result = rtc_gpio_pullup_en(m5sw::board::kPm1Irq);
        if (sleep_result != ESP_OK) return {sleep_result, sleep_result};
        sleep_result = esp_sleep_enable_ext0_wakeup(m5sw::board::kPm1Irq, 0);
        if (sleep_result != ESP_OK) return {sleep_result, sleep_result};
    }
    if (timer_fallback_us > 0) {
        const esp_err_t timer_result = esp_sleep_enable_timer_wakeup(timer_fallback_us);
        if (timer_result != ESP_OK) return {timer_result, timer_result};
    }
    // Keys selected in key_wake_mask stay as pulled-up RTC inputs and wake the ESP32 through ext1
    // (any key low). The others are isolated as in prepare_l2().
    for (const gpio_num_t key : {m5sw::board::kKey2, m5sw::board::kKey1}) {
        if ((key_wake_mask & (1ULL << key)) == 0) {
            rtc_gpio_isolate(key);
            continue;
        }
        rtc_gpio_hold_dis(key);
        esp_err_t key_result = rtc_gpio_init(key);
        if (key_result == ESP_OK) key_result = rtc_gpio_set_direction(key, RTC_GPIO_MODE_INPUT_ONLY);
        if (key_result == ESP_OK) key_result = rtc_gpio_pulldown_dis(key);
        if (key_result == ESP_OK) key_result = rtc_gpio_pullup_en(key);
        if (key_result != ESP_OK) return {key_result, key_result};
    }
    std::uint64_t ext1_mask = key_wake_mask;
    if (touch_wake_) {
        // タッチの割り込み（Low のパルス）でも起きる。
        const gpio_num_t irq = m5sw::board::kTouchIrq;
        rtc_gpio_hold_dis(irq);
        esp_err_t irq_result = rtc_gpio_init(irq);
        if (irq_result == ESP_OK) irq_result = rtc_gpio_set_direction(irq, RTC_GPIO_MODE_INPUT_ONLY);
        if (irq_result == ESP_OK) irq_result = rtc_gpio_pulldown_dis(irq);
        if (irq_result == ESP_OK) irq_result = rtc_gpio_pullup_en(irq);
        if (irq_result != ESP_OK) return {irq_result, irq_result};
        ext1_mask |= 1ULL << irq;
        arm_touch_latch();
    } else {
        rtc_gpio_isolate(m5sw::board::kTouchIrq);
    }
    if (ext1_mask != 0) {
        const esp_err_t ext1_result = esp_sleep_enable_ext1_wakeup_io(ext1_mask, ESP_EXT1_WAKEUP_ANY_LOW);
        if (ext1_result != ESP_OK) return {ext1_result, ext1_result};
    }

    PowerResult result = begin_ioe1();
    if (!result.ok()) return result;
    result = finalize_l2_peripherals(pm1_idle_sleep);
    if (!result.ok()) return result;
    // 寝る前の 1 秒の待ちの間にタップされたら、すぐ起きるようにタイマーを 1ms にして寝る
    // （記録は残るので、起きた側で take_touch_latch() がタップとして扱う）。
    const TickType_t step = pdMS_TO_TICKS(20);  // 待つ間は浅い眠り（アプリ側で有効にしていれば）
    s_quick_wake_armed    = false;
    for (TickType_t waited = 0; waited < kL2IdleSleepDelay; waited += step) {
        if (touch_wake_ && touch_latched()) {
            s_quick_wake_armed = true;
            break;
        }
        vTaskDelay(step);
    }
    // タイマー起床は寝る直前に設定し直す。待ちの間の自動ライトスリープ（esp_pm）が、毎回
    // esp_sleep_enable_timer_wakeup() を次の tick までの短い時間で上書きするため、そのままだと
    // ディープスリープが数十 ms で終わる（2026-11-17: 動かさずに置くと 1.1 秒ごとに起き続けていた）。
    esp_sleep_disable_wakeup_source(ESP_SLEEP_WAKEUP_TIMER);
    if (s_quick_wake_armed) {
        esp_sleep_enable_timer_wakeup(1000);
    } else if (timer_fallback_us > 0) {
        esp_sleep_enable_timer_wakeup(timer_fallback_us);
    }
    esp_deep_sleep_start();
    return {ESP_FAIL, ESP_FAIL};
}

}  // namespace m5sw::power
