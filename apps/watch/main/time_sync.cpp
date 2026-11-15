#include "time_sync.hpp"

#include <cstring>
#include <ctime>

#include "esp_event.h"
#include "esp_log.h"
#include "esp_netif.h"
#include "esp_netif_sntp.h"
#include "esp_wifi.h"
#include "freertos/FreeRTOS.h"
#include "freertos/event_groups.h"
#include "nvs_flash.h"

#if __has_include("wifi_secrets.h")
#include "wifi_secrets.h"
#endif
#ifndef WATCH_WIFI_SSID
#define WATCH_WIFI_SSID ""
#endif
#ifndef WATCH_WIFI_PASSWORD
#define WATCH_WIFI_PASSWORD ""
#endif

namespace watch {
namespace {

constexpr char kTag[]            = "time_sync";
constexpr char kNtpServer[]      = "ntp.nict.jp";
constexpr int kConnectTimeoutMs  = 10000;
constexpr int kSntpTimeoutMs     = 10000;
constexpr EventBits_t kGotIp     = BIT0;

EventGroupHandle_t s_events = nullptr;

void on_event(void*, esp_event_base_t base, int32_t id, void*)
{
    if (base == WIFI_EVENT && id == WIFI_EVENT_STA_START) {
        esp_wifi_connect();
    } else if (base == WIFI_EVENT && id == WIFI_EVENT_STA_DISCONNECTED) {
        esp_wifi_connect();  // タイムアウトまで繰り返す
    } else if (base == IP_EVENT && id == IP_EVENT_STA_GOT_IP) {
        xEventGroupSetBits(s_events, kGotIp);
    }
}

std::int64_t days_from_civil(int y, unsigned m, unsigned d)
{
    y -= m <= 2;
    const int era      = (y >= 0 ? y : y - 399) / 400;
    const unsigned yoe = static_cast<unsigned>(y - era * 400);
    const unsigned doy = (153 * (m + (m > 2 ? -3 : 9)) + 2) / 5 + d - 1;
    const unsigned doe = yoe * 365 + yoe / 4 - yoe / 100 + doy;
    return static_cast<std::int64_t>(era) * 146097 + static_cast<std::int64_t>(doe) - 719468;
}

std::int64_t epoch_from_tm(const std::tm& t)
{
    return days_from_civil(t.tm_year + 1900, static_cast<unsigned>(t.tm_mon + 1), static_cast<unsigned>(t.tm_mday)) *
               86400 +
           t.tm_hour * 3600 + t.tm_min * 60 + t.tm_sec;
}

}  // namespace

bool wifi_time_sync_configured()
{
    return std::strlen(WATCH_WIFI_SSID) > 0;
}

esp_err_t wifi_time_sync(m5sw::wakeup::RtcWakeup* rtc, std::int64_t* synced_epoch, std::int64_t* drift_s)
{
    if (!wifi_time_sync_configured()) return ESP_ERR_NOT_SUPPORTED;
    // Wi-Fi ドライバのログには接続先の SSID が出るので、警告以上だけにする。
    esp_log_level_set("wifi", ESP_LOG_WARN);
    esp_log_level_set("wifi_init", ESP_LOG_WARN);
    esp_log_level_set("esp_netif_handlers", ESP_LOG_WARN);

    esp_err_t err = nvs_flash_init();
    if (err == ESP_ERR_NVS_NO_FREE_PAGES || err == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        nvs_flash_erase();
        err = nvs_flash_init();
    }
    if (err != ESP_OK) return err;
    err = esp_netif_init();
    if (err != ESP_OK && err != ESP_ERR_INVALID_STATE) return err;
    err = esp_event_loop_create_default();
    if (err != ESP_OK && err != ESP_ERR_INVALID_STATE) return err;
    if (s_events == nullptr) s_events = xEventGroupCreate();
    xEventGroupClearBits(s_events, kGotIp);

    esp_netif_t* netif           = esp_netif_create_default_wifi_sta();
    wifi_init_config_t init      = WIFI_INIT_CONFIG_DEFAULT();
    err                          = esp_wifi_init(&init);
    esp_event_handler_instance_t wifi_handler = nullptr, ip_handler = nullptr;
    if (err == ESP_OK) {
        esp_event_handler_instance_register(WIFI_EVENT, ESP_EVENT_ANY_ID, on_event, nullptr, &wifi_handler);
        esp_event_handler_instance_register(IP_EVENT, IP_EVENT_STA_GOT_IP, on_event, nullptr, &ip_handler);
        wifi_config_t config = {};
        std::strncpy(reinterpret_cast<char*>(config.sta.ssid), WATCH_WIFI_SSID, sizeof(config.sta.ssid));
        std::strncpy(reinterpret_cast<char*>(config.sta.password), WATCH_WIFI_PASSWORD, sizeof(config.sta.password));
        esp_wifi_set_storage(WIFI_STORAGE_RAM);
        err = esp_wifi_set_mode(WIFI_MODE_STA);
        if (err == ESP_OK) err = esp_wifi_set_config(WIFI_IF_STA, &config);
        if (err == ESP_OK) err = esp_wifi_start();
    }

    if (err == ESP_OK) {
        const EventBits_t bits =
            xEventGroupWaitBits(s_events, kGotIp, pdFALSE, pdTRUE, pdMS_TO_TICKS(kConnectTimeoutMs));
        if ((bits & kGotIp) == 0) err = ESP_ERR_TIMEOUT;
        if (err != ESP_OK) ESP_LOGW(kTag, "Wi-Fi connect failed error=%s", esp_err_to_name(err));
    }

    if (err == ESP_OK) {
        esp_sntp_config_t sntp = ESP_NETIF_SNTP_DEFAULT_CONFIG(kNtpServer);
        err                    = esp_netif_sntp_init(&sntp);
        if (err == ESP_OK) err = esp_netif_sntp_sync_wait(pdMS_TO_TICKS(kSntpTimeoutMs));
        if (err == ESP_OK) {
            const std::time_t now = std::time(nullptr);
            std::tm before        = {};
            const bool had_rtc    = rtc->read_time(&before).ok();
            std::tm utc           = {};
            gmtime_r(&now, &utc);
            const auto set = rtc->set_time(utc);
            if (!set.ok()) {
                err = set.error;
            } else {
                if (synced_epoch != nullptr) *synced_epoch = static_cast<std::int64_t>(now);
                if (drift_s != nullptr) *drift_s = had_rtc ? epoch_from_tm(before) - static_cast<std::int64_t>(now) : 0;
            }
        } else {
            ESP_LOGW(kTag, "SNTP failed error=%s", esp_err_to_name(err));
        }
        esp_netif_sntp_deinit();
    }

    esp_wifi_stop();
    if (wifi_handler) esp_event_handler_instance_unregister(WIFI_EVENT, ESP_EVENT_ANY_ID, wifi_handler);
    if (ip_handler) esp_event_handler_instance_unregister(IP_EVENT, IP_EVENT_STA_GOT_IP, ip_handler);
    esp_wifi_deinit();
    if (netif != nullptr) esp_netif_destroy_default_wifi(netif);
    return err;
}

}  // namespace watch
