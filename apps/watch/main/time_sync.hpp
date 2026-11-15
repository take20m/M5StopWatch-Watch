// Wi-Fi で NTP から時刻を取り、RTC（UTC）に書く。1 日 1 回程度呼ぶ想定。
#pragma once

#include <cstdint>

#include "esp_err.h"
#include "m5sw_rtc_wakeup.hpp"

namespace watch {

// wifi_secrets.h があり、SSID が設定されているか。
bool wifi_time_sync_configured();

// Wi-Fi につないで NTP の時刻を RTC に書く。終わったら Wi-Fi を止める。
// 成功すると *synced_epoch に合わせた時刻（UTC の秒）、*drift_s に「RTC − NTP」の秒を入れる。
esp_err_t wifi_time_sync(m5sw::wakeup::RtcWakeup* rtc, std::int64_t* synced_epoch, std::int64_t* drift_s);

}  // namespace watch
