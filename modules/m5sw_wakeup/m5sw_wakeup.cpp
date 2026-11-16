#include "m5sw_wakeup.hpp"

namespace m5sw::wakeup {

bool WakeSnapshot::pm1_external() const
{
    return (pm1_wake & M5PM1_WAKE_SRC_EXT_WAKE) != 0U;
}

bool WakeSnapshot::rtc_imu_gpio_irq() const
{
    return (pm1_gpio_irq & M5PM1_IRQ_GPIO0) != 0U;
}

WakeupResult WakeController::from_pm1(m5sw::power::DriverResult result) const
{
    return {result.error, result.raw_error};
}

WakeupResult WakeController::from_imu(ImuWakeupResult result) const
{
    return {result.error, result.raw_error};
}

WakeupResult WakeController::begin(i2c_bus_handle_t bus, bool init_imu)
{
    return begin(bus, init_imu, nullptr);
}

WakeupResult WakeController::begin(i2c_bus_handle_t bus, bool init_imu, m5sw::power::Pm1Adapter* shared_pm1)
{
    if (bus == nullptr) return {ESP_ERR_INVALID_ARG, ESP_ERR_INVALID_ARG};
    bus_                = bus;
    shared_pm1_         = shared_pm1;
    WakeupResult result = shared_pm1 != nullptr ? WakeupResult{} : from_pm1(pm1_.begin(bus));
    if (!result.ok()) return result;
    if (init_imu) result = from_imu(imu_.begin(bus));
    initialized_ = result.ok();
    return result;
}

WakeupResult WakeController::begin_imu()
{
    if (!initialized_) return {ESP_ERR_INVALID_STATE, ESP_ERR_INVALID_STATE};
    if (imu_.initialized()) return {};
    return from_imu(imu_.begin(bus_));
}

WakeupResult WakeController::select_source(WakeSelection selection, ImuWakeProfile imu_profile,
                                           std::uint8_t any_duration, std::uint8_t any_threshold)
{
    if (!initialized_) return {ESP_ERR_INVALID_STATE, ESP_ERR_INVALID_STATE};

    WakeupResult result = from_pm1(pm1().disable_gpio_wake(m5sw::power::Pm1Gpio::rtc_imu));
    if (!result.ok()) return result;
    result = from_pm1(pm1().disable_gpio_wake(m5sw::power::Pm1Gpio::port));
    if (!result.ok()) return result;
    result = from_imu(imu_.disable());
    if (!result.ok()) return result;

    esp_sleep_disable_wakeup_source(ESP_SLEEP_WAKEUP_ALL);
    if (selection == WakeSelection::rtc || selection == WakeSelection::imu || selection == WakeSelection::rtc_imu) {
        result = from_pm1(pm1().configure_gpio_wake(m5sw::power::Pm1Gpio::rtc_imu));
        if (!result.ok()) return result;
    }
    if (selection == WakeSelection::imu || selection == WakeSelection::rtc_imu) {
        result = from_imu(imu_.configure_any_motion(imu_profile, any_duration, any_threshold));
        if (!result.ok()) return result;
    } else if (selection == WakeSelection::port) {
        result = from_pm1(pm1().configure_gpio_wake(m5sw::power::Pm1Gpio::port));
        if (!result.ok()) return result;
    }

    if (selection != WakeSelection::none) {
        const esp_err_t ext0 = esp_sleep_enable_ext0_wakeup(m5sw::board::kPm1Irq, 0);
        if (ext0 != ESP_OK) return {ext0, ext0};
    }
    return {};
}

WakeupResult WakeController::capture(const RtcFlags& rtc_flags, const ImuStatus& imu_status, WakeSnapshot* snapshot)
{
    if (!initialized_ || snapshot == nullptr) {
        return {snapshot == nullptr ? ESP_ERR_INVALID_ARG : ESP_ERR_INVALID_STATE,
                snapshot == nullptr ? ESP_ERR_INVALID_ARG : ESP_ERR_INVALID_STATE};
    }
    *snapshot                        = {};
    m5sw::power::DriverResult result = pm1().read_wake_source(&snapshot->pm1_wake);
    if (!result.ok()) return from_pm1(result);
    result = pm1().read_gpio_irq(&snapshot->pm1_gpio_irq);
    if (!result.ok()) return from_pm1(result);
    result = pm1().read_button_irq(&snapshot->pm1_button_irq);
    if (!result.ok()) return from_pm1(result);
    result = pm1().read_system_irq(&snapshot->pm1_system_irq);
    if (!result.ok()) return from_pm1(result);
    snapshot->rtc_events     = rtc_flags.events();
    snapshot->imu_any_motion = imu_status.any_motion;
    snapshot->imu_wrist_wear = imu_status.wrist_wear;
    // GPIO4 Port wake may be represented in either PM1's retained wake-source
    // mask or its GPIO IRQ status, depending on whether the edge was sampled
    // before or during the ESP32 wake path.
    snapshot->port = ((snapshot->pm1_wake | snapshot->pm1_gpio_irq) & (1U << 4)) != 0U;
    return {};
}

WakeupResult WakeController::read_imu_status(ImuStatus* status) const
{
    return from_imu(imu_.read_status(status));
}

WakeupResult WakeController::clear_snapshot(const WakeSnapshot& snapshot)
{
    if (!initialized_) return {ESP_ERR_INVALID_STATE, ESP_ERR_INVALID_STATE};
    // Clear only the bits captured before wake classification. This preserves
    // unrelated events for the next diagnostic cycle and avoids false PASS.
    m5sw::power::DriverResult result = pm1().clear_wake_source(snapshot.pm1_wake);
    if (!result.ok()) return from_pm1(result);
    result = pm1().clear_gpio_irq(snapshot.pm1_gpio_irq);
    if (!result.ok()) return from_pm1(result);
    result = pm1().clear_button_irq(snapshot.pm1_button_irq);
    if (!result.ok()) return from_pm1(result);
    result = pm1().clear_system_irq(snapshot.pm1_system_irq);
    if (!result.ok()) return from_pm1(result);
    // Without begin_imu(), INT_STATUS_0 was already consumed by read_retained_status() (clear-on-read).
    if ((snapshot.imu_any_motion || snapshot.imu_wrist_wear) && imu_.initialized()) {
        return from_imu(imu_.clear_status());
    }
    return {};
}

}  // namespace m5sw::wakeup
