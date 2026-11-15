// 腕時計の MVP（手順 2）。
//
// ふだんは L2 ディープスリープで寝ていて、次のどれかで起きる。
//   - 手首を上げて時計を見る（BMI270 の any-motion → PM1 → GPIO12 の ext0。起きてから姿勢と動きを判定。
//     判定は tests/look_wake_l2 で調整したもの）
//   - KEYA / KEYB を押す（GPIO2 / GPIO1 の ext1）
//   - 保険のタイマー（1 時間。何もせずに寝直す）
// 見ていれば画面を点けて、時刻・日付・電池残量を出す。腕を下ろすか一定時間で消して、また寝る。
//
// 時刻: RX8130 には UTC を入れ、表示は日本時間（JST）にする。
// コールドブートのときに USB から "T=<UNIX 時刻>" を受け取ったら RTC を合わせる（set_watch_time.py が送る）。
#include <algorithm>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <ctime>
#include <initializer_list>

#include "driver/gpio.h"
#include "driver/rtc_io.h"
#include "driver/usb_serial_jtag.h"
#include "driver/usb_serial_jtag_vfs.h"
#include "esp_attr.h"
#include "esp_log.h"
#include "esp_rom_sys.h"
#include "esp_sleep.h"
#include "esp_system.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "freertos/task.h"
#include "i2c_bus.h"
#include "m5sw_board.hpp"
#include "time_sync.hpp"
#include "watch_fonts.h"
#include "watch_panel.hpp"
#include "m5sw_power.hpp"
#include "m5sw_rtc_wakeup.hpp"
#include "m5sw_test_support.hpp"
#include "m5sw_wakeup.hpp"

namespace {
constexpr char kTag[]              = "watch";
constexpr TickType_t kLogFlushWait = pdMS_TO_TICKS(50);  // ログを USB に流し切るための待ち

// ---- 起こし方 ----
constexpr std::uint8_t kAnyDuration      = 2;    // any-motion: 40ms 続く
constexpr std::uint8_t kAnyThreshold     = 100;  // 約 48mg 以上の変化
// 保険のタイマー。v2/v3 で「寝たまま起きなくなる」ことがあったので、調査中は 1 分にして、
// 起きたら起床経路の状態をログに出し、動き検出を設定し直す（落ち着いたら 1 時間に戻す）。
constexpr std::uint64_t kTimerFallbackUs = 3600ULL * 1000 * 1000;  // 1 時間（電池の記録も兼ねる）
constexpr std::uint64_t kKeyWakeMask     = (1ULL << m5sw::board::kKey1) | (1ULL << m5sw::board::kKey2);

// ---- 画面 ----
constexpr std::uint8_t kBrightness      = 60;        // %
constexpr std::int64_t kLookShowMaxUs   = 10000000;  // 見続けていても 10 秒で消す
constexpr std::int64_t kLookMinShowUs   = 3000000;   // 手首で点けたら、腕を下ろしてもこの時間は消さない
constexpr std::int64_t kButtonShowUs    = 5000000;   // ボタンで点けたときは 5 秒（押すたびに延長）
constexpr std::int64_t kColdBootShowUs  = 5000000;
constexpr std::int32_t kJstOffsetSec    = 9 * 3600;
constexpr int kCenterX                  = 234;  // 表示領域 468x466 の中心
constexpr int kCenterY                  = 233;
constexpr std::uint16_t kColorDim       = 0x8410;  // 灰色
constexpr std::uint16_t kColorAccent    = 0xFD20;  // 橙
constexpr std::uint16_t kColorDate      = 0xC618;  // 明るい灰色

// 文字盤の縦の配置。キャラクターがあるときは時刻を上に寄せ、下にキャラクターを置く。
#if WATCH_HAS_CHARACTER
// 2026-11-15 動画 IMG_0078 で日付と秒の端が削れていたので、PC で配置を再現して決め直した。
constexpr int kBatteryY   = 28;
constexpr int kDateY      = 80;
constexpr int kTimeY      = 168;  // 数字の中心
constexpr int kSecY       = 244;
constexpr int kCharacterY = 276;  // キャラクター画像の上端（160px）
#else
constexpr int kBatteryY   = 58;
constexpr int kDateY      = 138;
constexpr int kTimeY      = kCenterY + 8;
constexpr int kSecY       = 330;
#endif

// ---- 「時計を見ている」の判定（tests/look_wake_l2 v11 と同じ） ----
// x の条件は姿勢で変わる（昼は見たとき x≥+0.18、夜は x≈−0.17 だった）ので、ゆるめにして
// 「起きる前の姿勢から十分に動いたか」（is_raise）で、置いたままの手を外す。
// 腕を机の上に伸ばして見ると x=−0.3〜−0.46 になる（2026-11-15 動画 IMG_0054。Apple Watch は点いていた）。
// 机に置いた手と姿勢では区別できないので、x はゆるめ、持ち上げた動きか（is_raise）で分ける。
// 手首を内側に返して見ると x=−0.55〜−0.63 になる（2026-11-15 動画 IMG_0063。−0.55 で切っていて点かず、
// 手首を寝かせて戻したときに点いていた）。x の下限は実質なくし、文字盤の向き（z）と持ち上げの角度で判定する。
constexpr float kLookMinX           = -0.95f;
constexpr float kLookMaxDownY       = -0.60f;  // 腕が下を向いている（下ろす途中・コップで飲む）姿勢は外す
constexpr float kLookMinZ           = 0.50f;  // 文字盤が上向き寄り
// 座って手を膝に置いた姿勢は (+0.5 +0.7 +0.5) 前後で、見る姿勢（y≦+0.05）と y で分かれる（2026-11-15 動画）。
// 以前の「腕が立っている（y≧0.6）なら z が小さくてもよい」は、膝に置いた姿勢で点いてしまうのでやめた。
constexpr float kLookMaxY           = 0.45f;
constexpr float kStillTolerance     = 0.15f;
constexpr std::int64_t kWindowUs    = 1500000;
constexpr int kSmoothFrames         = 5;
constexpr int kLookHoldFrames       = 4;   // 80ms（v1 は 200ms、7=140ms から速さのためさらに短くした）
constexpr int kAwayHoldFrames       = 25;  // 500ms
constexpr int kViewEndFrames        = 25;  // 500ms
constexpr float kRaiseMinDeg        = 25.0f;
constexpr int kRaiseSkipFrames      = 10;
constexpr int kRaiseLookbackFrames  = 100;
constexpr float kHistStillTolerance = 0.25f;
constexpr float kLsbPerG            = 16384.0f;

// ---- BMI270 の FIFO ----
constexpr std::uint32_t kImuI2cHz      = 400000;  // FIFO を速く読む（APS を切っていれば 400kHz でも読めるか確認中）
constexpr std::uint8_t kRegFifoLength  = 0x24;
constexpr std::uint8_t kRegFifoData    = 0x26;
constexpr std::uint8_t kRegFifoConfig0 = 0x48;
constexpr std::uint8_t kRegFifoConfig1 = 0x49;
constexpr std::uint8_t kRegPwrConf     = 0x7C;  // bit0 adv_power_save
constexpr std::uint8_t kRegCmd         = 0x7E;
constexpr std::uint8_t kCmdFifoFlush   = 0xB0;
constexpr std::size_t kFifoBytes       = 2048;
constexpr std::size_t kFrameBytes      = 6;

// ディープスリープをまたいで残る値。
RTC_DATA_ATTR std::uint32_t s_wake_count   = 0;
RTC_DATA_ATTR std::uint32_t s_show_count   = 0;
RTC_DATA_ATTR std::uint64_t s_awake_ms     = 0;
RTC_DATA_ATTR std::uint32_t s_timer_count  = 0;
RTC_DATA_ATTR std::int64_t s_last_sync_epoch    = 0;  // 最後に時刻を合わせた時刻（UTC の秒）
RTC_DATA_ATTR std::int64_t s_last_wifi_try_epoch = 0;  // 最後に Wi-Fi で合わせようとした時刻

// 電池の記録（1 時間ごとの定期起床で 1 件。USB を外して使った後、つないだときに読み出す）
struct BatteryRecord {
    std::int64_t epoch;
    std::uint16_t mv;
    std::uint16_t shows;   // その時点までに画面を点けた回数
    std::uint32_t wakes;   // その時点までに起きた回数
};
constexpr int kBatteryRecords = 96;  // 4 日分
RTC_DATA_ATTR BatteryRecord s_battery[kBatteryRecords];
RTC_DATA_ATTR std::uint16_t s_battery_count = 0;
RTC_DATA_ATTR std::int64_t s_next_hourly_epoch = 0;  // 次に 1 時間ごとの用事（電池の記録・時刻合わせの確認）をする時刻
constexpr int kHistLen                     = 128;
RTC_DATA_ATTR std::int16_t s_hist[kHistLen][3];
RTC_DATA_ATTR std::uint16_t s_hist_head  = 0;
RTC_DATA_ATTR std::uint16_t s_hist_count = 0;

[[noreturn]] void fail(const char* stage, esp_err_t error, int raw_error)
{
    ESP_LOGE(kTag, "BOARD_FAIL stage=%s error=%s raw=%d -> restart", stage, esp_err_to_name(error), raw_error);
    vTaskDelay(pdMS_TO_TICKS(500));
    esp_restart();
}

// PM1 / IOE1 の I2C アイドルスリープ明けの 1 回目は失敗することがあるので、数回やり直す。
template <typename Fn>
auto with_retry(Fn fn)
{
    auto result = fn();
    for (int attempt = 1; attempt < 4 && !result.ok(); ++attempt) {
        ESP_LOGW(kTag, "I2C_RETRY attempt=%d error=%s", attempt, esp_err_to_name(result.error));
        vTaskDelay(pdMS_TO_TICKS(20));
        result = fn();
    }
    return result;
}

std::int64_t now_ms()
{
    return esp_timer_get_time() / 1000;
}

// ---------------------------------------------------------------------------
// 時刻
// ---------------------------------------------------------------------------
// UTC の暦から UNIX 時刻へ（newlib に timegm がないため。Howard Hinnant の days_from_civil）。
std::int64_t epoch_from_utc(const std::tm& t)
{
    int y            = t.tm_year + 1900;
    const unsigned m = static_cast<unsigned>(t.tm_mon + 1);
    y -= m <= 2;
    const int era      = (y >= 0 ? y : y - 399) / 400;
    const unsigned yoe = static_cast<unsigned>(y - era * 400);
    const unsigned doy = (153 * (m + (m > 2 ? -3 : 9)) + 2) / 5 + static_cast<unsigned>(t.tm_mday) - 1;
    const unsigned doe = yoe * 365 + yoe / 4 - yoe / 100 + doy;
    const std::int64_t days = static_cast<std::int64_t>(era) * 146097 + static_cast<std::int64_t>(doe) - 719468;
    return days * 86400 + t.tm_hour * 3600 + t.tm_min * 60 + t.tm_sec;
}

std::tm utc_from_epoch(std::int64_t epoch)
{
    const std::time_t value = static_cast<std::time_t>(epoch);
    std::tm out             = {};
    gmtime_r(&value, &out);
    return out;
}

// コールドブートのとき、USB から "T=<UNIX 時刻>" が来たら RTC を合わせる。来なければ何もしない。
bool sync_time_from_usb(m5sw::wakeup::RtcWakeup* rtc)
{
    usb_serial_jtag_driver_config_t config = USB_SERIAL_JTAG_DRIVER_CONFIG_DEFAULT();
    if (usb_serial_jtag_driver_install(&config) != ESP_OK) {
        ESP_LOGW(kTag, "TIME_SYNC skipped (usb driver)");
        return false;
    }
    usb_serial_jtag_vfs_use_driver();
    ESP_LOGI(kTag, "TIME_REQ (send \"T=<unix seconds>\\n\" within 3 s)");
    char line[32]            = {};
    std::size_t len          = 0;
    const std::int64_t start = now_ms();
    while (now_ms() - start < 3000) {
        char c = 0;
        if (usb_serial_jtag_read_bytes(&c, 1, pdMS_TO_TICKS(50)) != 1) continue;
        if (c == '\n' || c == '\r') {
            line[len] = '\0';
            if (len > 2 && line[0] == 'T' && line[1] == '=') {
                const std::int64_t epoch = std::strtoll(line + 2, nullptr, 10);
                const std::tm utc        = utc_from_epoch(epoch);
                const auto result        = rtc->set_time(utc);
                ESP_LOGI(kTag, "TIME_SET epoch=%lld %04d-%02d-%02d %02d:%02d:%02d UTC result=%s",
                         static_cast<long long>(epoch), utc.tm_year + 1900, utc.tm_mon + 1, utc.tm_mday, utc.tm_hour,
                         utc.tm_min, utc.tm_sec, esp_err_to_name(result.error));
                if (result.ok()) s_last_sync_epoch = epoch;
                return result.ok();
            }
            len = 0;
        } else if (len + 1 < sizeof(line)) {
            line[len++] = c;
        }
    }
    ESP_LOGI(kTag, "TIME_SYNC no time received; keeping RTC");
    return false;
}

// Wi-Fi（NTP）で時刻を合わせる。1 日 1 回。失敗したら 1 時間あけて再試行する。
constexpr std::int64_t kWifiSyncIntervalS = 24 * 3600;
constexpr std::int64_t kWifiRetryS        = 3600;

void maybe_wifi_time_sync(m5sw::wakeup::RtcWakeup* rtc, bool force)
{
    if (!watch::wifi_time_sync_configured()) {
        if (force) ESP_LOGI(kTag, "WIFI_SYNC skipped (wifi_secrets.h not set)");
        return;
    }
    std::tm utc = {};
    const bool rtc_ok = rtc->read_time(&utc).ok();
    const std::int64_t now = rtc_ok ? epoch_from_utc(utc) : 0;
    if (!force) {
        if (!rtc_ok) return;
        if (s_last_sync_epoch != 0 && now - s_last_sync_epoch < kWifiSyncIntervalS) return;
        if (s_last_wifi_try_epoch != 0 && now - s_last_wifi_try_epoch < kWifiRetryS) return;
    }
    s_last_wifi_try_epoch    = now;
    const std::int64_t start = now_ms();
    std::int64_t synced = 0, drift = 0;
    const esp_err_t err = watch::wifi_time_sync(rtc, &synced, &drift);
    if (err == ESP_OK) {
        s_last_sync_epoch     = synced;
        s_last_wifi_try_epoch = synced;
    }
    ESP_LOGI(kTag, "WIFI_SYNC result=%s drift=%llds took=%lldms", esp_err_to_name(err), static_cast<long long>(drift),
             now_ms() - start);
}

// 1 時間ごとの用事。起きた理由によらず、前回から 1 時間たっていればする（腕を動かしている間は定期起床が来ないため）。
void hourly_tasks(m5sw::wakeup::RtcWakeup* rtc, m5sw::power::PowerController* power)
{
    std::tm utc = {};
    if (!rtc->read_time(&utc).ok()) return;
    const std::int64_t now = epoch_from_utc(utc);
    if (s_next_hourly_epoch != 0 && now < s_next_hourly_epoch) return;
    s_next_hourly_epoch = now + 3600;

    std::uint16_t mv = 0;
    (void)with_retry([&] { return power->read_battery_mv(&mv); });
    const BatteryRecord rec = {now, mv, static_cast<std::uint16_t>(s_show_count), s_wake_count};
    if (s_battery_count < kBatteryRecords) {
        s_battery[s_battery_count++] = rec;
    } else {
        for (int i = 1; i < kBatteryRecords; ++i) s_battery[i - 1] = s_battery[i];
        s_battery[kBatteryRecords - 1] = rec;
    }
    for (int i = 0; i < s_battery_count; ++i) {
        ESP_LOGI(kTag, "BAT_HIST %d epoch=%lld mv=%u shows=%u wakes=%lu", i, static_cast<long long>(s_battery[i].epoch),
                 s_battery[i].mv, s_battery[i].shows, static_cast<unsigned long>(s_battery[i].wakes));
    }
    maybe_wifi_time_sync(rtc, false);
}

// ---------------------------------------------------------------------------
// 加速度の履歴と FIFO（tests/look_wake_l2 から）
// ---------------------------------------------------------------------------
struct Accel {
    float x = 0, y = 0, z = 0;
    [[nodiscard]] float norm() const
    {
        return std::sqrt(x * x + y * y + z * z);
    }
    [[nodiscard]] bool still(float tolerance = kStillTolerance) const
    {
        return std::fabs(norm() - 1.0f) <= tolerance;
    }
    [[nodiscard]] bool look() const
    {
        return still() && pose_ok();
    }
    // 見る姿勢の向きか（止まっているかは問わない）。
    [[nodiscard]] bool pose_ok() const
    {
        return x >= kLookMinX && y >= kLookMaxDownY && y <= kLookMaxY && z >= kLookMinZ;
    }
    [[nodiscard]] bool still_viewing() const
    {
        return x >= kLookMinX - 0.05f && y >= kLookMaxDownY - 0.1f && y <= kLookMaxY + 0.1f && z >= kLookMinZ - 0.1f;
    }
};

float angle_deg(const Accel& a, const Accel& b)
{
    const float denom = a.norm() * b.norm();
    if (denom <= 0.0f) return 0.0f;
    float c = (a.x * b.x + a.y * b.y + a.z * b.z) / denom;
    c       = c > 1.0f ? 1.0f : (c < -1.0f ? -1.0f : c);
    return std::acos(c) * 57.29578f;
}

void hist_reset()
{
    s_hist_head  = 0;
    s_hist_count = 0;
}

void hist_push(std::int16_t x, std::int16_t y, std::int16_t z)
{
    s_hist[s_hist_head][0] = x;
    s_hist[s_hist_head][1] = y;
    s_hist[s_hist_head][2] = z;
    s_hist_head            = static_cast<std::uint16_t>((s_hist_head + 1) % kHistLen);
    if (s_hist_count < kHistLen) ++s_hist_count;
}

bool hist_avg(int from_end, Accel* out)
{
    if (from_end < 0 || from_end + kSmoothFrames > s_hist_count) return false;
    std::int32_t sx = 0, sy = 0, sz = 0;
    for (int k = 0; k < kSmoothFrames; ++k) {
        const int idx = (s_hist_head - 1 - from_end - k + 2 * kHistLen) % kHistLen;
        sx += s_hist[idx][0];
        sy += s_hist[idx][1];
        sz += s_hist[idx][2];
    }
    out->x = sx / (kSmoothFrames * kLsbPerG);
    out->y = sy / (kSmoothFrames * kLsbPerG);
    out->z = sz / (kSmoothFrames * kLsbPerG);
    return true;
}

float raise_angle(const Accel& look, int from_end, int* compared, Accel* pre = nullptr)
{
    float best = -1.0f;
    *compared  = 0;
    for (int k = from_end + kRaiseSkipFrames; k <= from_end + kRaiseLookbackFrames; ++k) {
        Accel a;
        if (!hist_avg(k, &a)) break;
        if (!a.still(kHistStillTolerance)) continue;
        ++*compared;
        const float deg = angle_deg(a, look);
        if (deg > best) {
            best = deg;
            if (pre != nullptr) *pre = a;
        }
    }
    return best;
}

class ImuFifo {
public:
    explicit ImuFifo(i2c_bus_handle_t bus) : device_(i2c_bus_device_create(bus, m5sw::board::kImuAddress, kImuI2cHz)) {}

    esp_err_t configure()
    {
        if (device_ == nullptr) return ESP_ERR_NO_MEM;
        esp_err_t result = write(kRegFifoConfig0, 0x00);
        if (result == ESP_OK) result = write(kRegFifoConfig1, 0x40);
        if (result == ESP_OK) result = write(kRegCmd, kCmdFifoFlush);
        if (result != ESP_OK) return result;
        std::uint8_t cfg[2] = {};
        result              = i2c_bus_read_bytes(device_, kRegFifoConfig0, sizeof(cfg), cfg);
        esp_rom_delay_us(450);
        if (result != ESP_OK) return result;
        return (cfg[0] == 0x00 && cfg[1] == 0x40) ? ESP_OK : ESP_ERR_INVALID_RESPONSE;
    }

    // 起きている間は advanced power save を切る（オンのままだと FIFO を正しく読めない）。寝る前に戻す。
    esp_err_t begin_awake()
    {
        if (device_ == nullptr) return ESP_ERR_NO_MEM;
        std::uint8_t conf      = 0;
        const esp_err_t result = i2c_bus_read_byte(device_, kRegPwrConf, &conf);
        esp_rom_delay_us(450);
        if (result != ESP_OK) return result;
        saved_pwr_conf_ = conf;
        have_saved_     = true;
        if ((conf & 0x01) == 0) return ESP_OK;
        const esp_err_t write_result = write(kRegPwrConf, conf & ~0x01);
        esp_rom_delay_us(1000);
        return write_result;
    }

    // 調査用：レジスタを読む（失敗したら -1）。
    [[nodiscard]] int saved_pwr_conf() const
    {
        return have_saved_ ? saved_pwr_conf_ : -1;
    }

    int read_reg(std::uint8_t reg)
    {
        std::uint8_t value = 0;
        const esp_err_t result = i2c_bus_read_byte(device_, reg, &value);
        esp_rom_delay_us(450);
        return result == ESP_OK ? value : -1;
    }

    esp_err_t end_awake()
    {
        if (device_ == nullptr || !have_saved_ || (saved_pwr_conf_ & 0x01) == 0) return ESP_OK;
        return write(kRegPwrConf, saved_pwr_conf_);
    }

    int pull()
    {
        if (device_ == nullptr) return -1;
        std::uint8_t len_raw[2] = {};
        if (i2c_bus_read_bytes(device_, kRegFifoLength, sizeof(len_raw), len_raw) != ESP_OK) return -1;
        esp_rom_delay_us(450);
        std::size_t bytes = static_cast<std::size_t>(len_raw[0]) | (static_cast<std::size_t>(len_raw[1] & 0x3F) << 8);
        bytes             = (bytes / kFrameBytes) * kFrameBytes;
        if (bytes > kFifoBytes) bytes = (kFifoBytes / kFrameBytes) * kFrameBytes;
        if (bytes == 0) return 0;
        static std::uint8_t buf[kFifoBytes];
        if (i2c_bus_read_bytes(device_, kRegFifoData, bytes, buf) != ESP_OK) return -1;
        esp_rom_delay_us(450);
        int added = 0;
        for (std::size_t off = 0; off + kFrameBytes <= bytes; off += kFrameBytes) {
            const auto x = static_cast<std::int16_t>(buf[off] | (buf[off + 1] << 8));
            const auto y = static_cast<std::int16_t>(buf[off + 2] | (buf[off + 3] << 8));
            const auto z = static_cast<std::int16_t>(buf[off + 4] | (buf[off + 5] << 8));
            if (x == INT16_MIN && y == INT16_MIN && z == INT16_MIN) continue;
            hist_push(x, y, z);
            ++added;
        }
        return added;
    }

private:
    esp_err_t write(std::uint8_t reg, std::uint8_t value)
    {
        const esp_err_t result = i2c_bus_write_byte(device_, reg, value);
        esp_rom_delay_us(450);
        return result;
    }

    i2c_bus_device_handle_t device_;
    std::uint8_t saved_pwr_conf_ = 0;
    bool have_saved_             = false;
};

struct LookResult {
    bool looking       = false;
    const char* reason = "window";
    std::int64_t at_ms = -1;
    Accel acc;
    Accel pre;  // 起きる前の姿勢（見る姿勢との角度がいちばん大きいもの）
    float raise_deg = -1.0f;
};

// 持ち上げた動きか：見る姿勢との角度が十分あれば、向きは問わない。
// 手首を内側に返して見る動き（2026-11-15 動画 IMG_0056）は x が減るので、向きの条件は外した。
// 机の上での誤点灯は気にしない（ユーザー確認済み）。
bool is_raise(const Accel& /*pre*/, const Accel& /*look*/, float deg, int compared)
{
    if (compared == 0) return true;  // 比べる履歴がない（コールドブート直後など）
    return deg >= kRaiseMinDeg;
}

// pose_started: 「見る姿勢」に入り始めて、それまでに持ち上げた動きがあったときに 1 回だけ呼ぶ（画面の準備を先に始めるため）。
LookResult check_look(ImuFifo* fifo, void (*pose_started)())
{
    LookResult out;
    const std::int64_t start = esp_timer_get_time();
    if (fifo->pull() < 0) {
        out.reason = "fifo_error";
        out.at_ms  = now_ms();
        return out;
    }
    int pose_run = 0;
    int away_run = 0;
    while (esp_timer_get_time() - start < kWindowUs) {
        vTaskDelay(pdMS_TO_TICKS(10));
        const int added = fifo->pull();
        for (int i = added - 1; i >= 0; --i) {
            Accel a;
            if (!hist_avg(i, &a)) continue;
            out.acc         = a;
            const bool pose = a.look();
            pose_run        = pose ? pose_run + 1 : 0;
            away_run        = (!pose && a.still()) ? away_run + 1 : 0;
            if (pose_run == 1 && pose_started != nullptr) {
                int compared       = 0;
                Accel pre;
                const float early  = raise_angle(a, i, &compared, &pre);
                if (is_raise(pre, a, early, compared)) pose_started();
            }
            if (pose_run >= kLookHoldFrames) {
                int compared  = 0;
                out.raise_deg = raise_angle(a, i + kLookHoldFrames - 1, &compared, &out.pre);
                out.looking   = is_raise(out.pre, a, out.raise_deg, compared);
                out.reason    = out.looking ? "LOOK" : "no_raise";
                out.at_ms     = now_ms();
                return out;
            }
            if (away_run >= kAwayHoldFrames) {
                out.reason = "pose_away";
                out.at_ms  = now_ms();
                return out;
            }
        }
    }
    out.at_ms = now_ms();
    return out;
}

// ---------------------------------------------------------------------------
// ボタン
// ---------------------------------------------------------------------------
void keys_as_inputs()
{
    for (const gpio_num_t key : {m5sw::board::kKey1, m5sw::board::kKey2}) {
        rtc_gpio_hold_dis(key);
        rtc_gpio_deinit(key);
        gpio_reset_pin(key);
        gpio_set_direction(key, GPIO_MODE_INPUT);
        gpio_set_pull_mode(key, GPIO_PULLUP_ONLY);
    }
}

// タッチ（CST820, 0x15）を、触れたら割り込み線を Low にパルスする設定にする。
// prepare_l2 がタッチをリセットした直後に呼ぶ（CST8xx は自動で低電力に入ると I2C に応答しなくなるため）。
void configure_touch_wake(i2c_bus_handle_t bus)
{
    i2c_bus_device_handle_t touch = i2c_bus_device_create(bus, m5sw::board::kTouchAddress, 100000);
    if (touch == nullptr) {
        ESP_LOGW(kTag, "TOUCH_CFG no device");
        return;
    }
    std::uint8_t chip = 0;
    esp_err_t err     = ESP_FAIL;
    for (int i = 0; i < 5 && err != ESP_OK; ++i) {
        err = i2c_bus_read_byte(touch, 0xA7, &chip);
        if (err != ESP_OK) vTaskDelay(pdMS_TO_TICKS(10));
    }
    std::uint8_t irq_ctl = 0, before = 0;
    if (err == ESP_OK) (void)i2c_bus_read_byte(touch, 0xFA, &before);
    if (err == ESP_OK) err = i2c_bus_write_byte(touch, 0xFA, 0x60);  // IrqCtl: 触れている間と状態が変わったときにパルス
    // 自動の低電力モードは無効（0x01）。有効（0x00）だとスリープ中のタップで割り込みが来ない
    // （2026-11-15 実機: 有効 0/3、無効 3/3）。待機電流の増え方は未計測。
    if (err == ESP_OK) err = i2c_bus_write_byte(touch, 0xFE, 0x01);
    if (err == ESP_OK) (void)i2c_bus_read_byte(touch, 0xFA, &irq_ctl);
    ESP_LOGI(kTag, "TOUCH_CFG chip=0x%02X irq_ctl 0x%02X->0x%02X result=%s", chip, before, irq_ctl,
             esp_err_to_name(err));
    i2c_bus_device_delete(&touch);
}

// 起きている間（手首の判定中など）にボタンが押されたら立つ。寝ている間の押下は ext1 で起きる。
volatile bool s_key_pressed_awake = false;
volatile int s_touch_edges        = 0;
volatile bool s_touched_awake     = false;  // 起きている間にタッチの割り込みが来た

void IRAM_ATTR on_touch_edge(void*)
{
    s_touch_edges   = s_touch_edges + 1;
    s_touched_awake = true;
}

void IRAM_ATTR on_key_edge(void*)
{
    s_key_pressed_awake = true;
}

void watch_keys_while_awake()
{
    gpio_install_isr_service(0);
    for (const gpio_num_t key : {m5sw::board::kKey1, m5sw::board::kKey2}) {
        gpio_set_intr_type(key, GPIO_INTR_NEGEDGE);
        gpio_isr_handler_add(key, on_key_edge, nullptr);
    }
    // タップの振動で手首の判定（ext0）が先に起きることが多い。その間に来るタッチの割り込みも拾う。
    const gpio_num_t irq = m5sw::board::kTouchIrq;
    rtc_gpio_hold_dis(irq);
    rtc_gpio_deinit(irq);
    gpio_reset_pin(irq);
    gpio_set_direction(irq, GPIO_MODE_INPUT);
    gpio_set_pull_mode(irq, GPIO_PULLUP_ONLY);
    if (gpio_get_level(irq) == 0) s_touched_awake = true;
    gpio_set_intr_type(irq, GPIO_INTR_NEGEDGE);
    gpio_isr_handler_add(irq, on_touch_edge, nullptr);
}

bool any_key_down()
{
    return gpio_get_level(m5sw::board::kKey1) == 0 || gpio_get_level(m5sw::board::kKey2) == 0;
}

// 押したまま寝ると ext1 ですぐ起きてしまうので、離すまで待つ（最大 3 秒）。
void wait_keys_released()
{
    const std::int64_t start = now_ms();
    while (any_key_down() && now_ms() - start < 3000) vTaskDelay(pdMS_TO_TICKS(20));
    vTaskDelay(pdMS_TO_TICKS(30));  // チャタリング
}

// ---------------------------------------------------------------------------
// キャラクター（main/assets/character.bin。tools/make_character.py で作る）
// ---------------------------------------------------------------------------
#if WATCH_HAS_CHARACTER
extern const std::uint8_t character_bin_start[] asm("_binary_character_bin_start");

RTC_DATA_ATTR int s_character_index = 0;  // 今の表情。点けるたび・タップするたびに次へ

struct CharacterSet {
    int count = 0, w = 0, h = 0;
    const std::uint16_t* frames = nullptr;
};

CharacterSet character_set()
{
    CharacterSet c;
    const std::uint8_t* p = character_bin_start;
    if (std::memcmp(p, "CHR1", 4) != 0) return c;
    auto u16 = [&](int off) { return static_cast<int>(p[off] | (p[off + 1] << 8)); };
    c.count  = u16(4);
    c.w      = u16(6);
    c.h      = u16(8);
    c.frames = reinterpret_cast<const std::uint16_t*>(p + 12);
    return c;
}
#endif

// ---------------------------------------------------------------------------
// 文字盤
// ---------------------------------------------------------------------------
struct FaceData {
    bool time_valid     = false;
    std::int64_t epoch  = 0;  // UTC
    int battery_percent = -1;
    std::uint16_t battery_mv = 0;
};

int battery_percent_from_mv(std::uint16_t mv)
{
    // UserDemo と同じ直線近似（3.3V = 0%、4.2V = 100%）。
    if (mv <= 3300) return 0;
    if (mv >= 4200) return 100;
    return static_cast<int>((mv - 3300) * 100 / (4200 - 3300));
}

// 前回描いた内容。変わった部分だけ描き直して、画面へ送るデータを小さくする。
// 毎秒画面全体（約 46 万バイト）を送っていたとき、まれに表示がずれた（2026-11-15 写真 IMG_0051 / IMG_0059。
// 画面が点いて数秒後の毎秒の書き換えで、全体が横にずれて折り返していた。40MHz に下げても出た）。
// 送る途中でデータがずれていると考え、毎秒送るのは秒の部分（数千バイト）だけにする。
struct FaceState {
    bool drawn     = false;
    char date[48]  = {};
    char hm[8]     = {};
    char sec[4]    = {};
};

// 文字の背景を、決まった大きさの枠で塗ってから描く（数字の幅が変わっても前の文字が残らないように）。
// 描き直した行の範囲（パネルへ送る範囲）
struct DirtyRows {
    int y0 = 1 << 30;
    int y1 = -1;
    void add(int a, int b)
    {
        y0 = std::min(y0, a);
        y1 = std::max(y1, b);
    }
    bool empty() const { return y1 < y0; }
};

void draw_text_box(lgfx::LovyanGFX* gfx, const char* text, const char* widest, int cx, int cy, DirtyRows* dirty)
{
    const int w = gfx->textWidth(widest) + 8;
    const int h = gfx->fontHeight() + 4;
    gfx->fillRect(cx - w / 2, cy - h / 2, w, h, TFT_BLACK);
    gfx->drawString(text, cx, cy);
    dirty->add(cy - h / 2, cy - h / 2 + h - 1);
}

// 時刻（HH:MM）を、数字の幅をそろえて描く（Inter の数字は幅が少しずつ違うので、1 秒ごとに揺れないように）。
void draw_time_digits(lgfx::LovyanGFX* gfx, const char* hm, int cx, int cy, DirtyRows* dirty)
{
    gfx->loadFont(kFontTime);
    int cell = 0;
    for (char d = '0'; d <= '9'; ++d) {
        const char one[2] = {d, 0};
        cell = std::max(cell, static_cast<int>(gfx->textWidth(one)));
    }
    const int colon = gfx->textWidth(":");
    const int total = cell * 4 + colon;
    // 数字は下に伸びないので、ベースラインに合わせて描き、塗るのは数字の高さだけにする
    // （行全体の高さで塗ると、上の日付の下半分まで消えていた。2026-11-15 写真 IMG_0076）。
    // 数字の高さ（Inter のキャップハイトは約 0.73em。kFontTime は 120px なので 87px）。
    // fontHeight() は行の高さ（約 1.2em）なので、その 72% では大きすぎて、上に寄せた日付の下端を削っていた
    // （2026-11-15 動画 IMG_0078）。
    constexpr int cap  = 120 * 73 / 100;
    const int baseline = cy + cap / 2;
    gfx->fillRect(cx - total / 2 - 6, baseline - cap - 4, total + 12, cap + 10, TFT_BLACK);
    dirty->add(baseline - cap - 4, baseline + 5);
    gfx->setTextColor(TFT_WHITE);  // 背景は上で塗ってあるので、文字ごとの背景塗りはしない
    gfx->setTextDatum(lgfx::textdatum_t::baseline_center);
    int x = cx - total / 2;
    for (int i = 0; hm[i] != 0; ++i) {
        const char one[2] = {hm[i], 0};
        const int w       = hm[i] == ':' ? colon : cell;
        gfx->drawString(one, x + w / 2, hm[i] == ':' ? baseline - cap / 20 : baseline);  // コロンは少し上げる（Apple 風）
        x += w;
    }
}

#if WATCH_HAS_CHARACTER
// キャラクターを下に描く（背景の黒と混ぜ済みの RGB565 をそのまま貼る）。
void draw_character(lgfx::LovyanGFX* gfx, int index, DirtyRows* dirty)
{
    const CharacterSet c = character_set();
    if (c.count == 0) return;
    index %= c.count;
    const std::uint16_t* frame = c.frames + static_cast<std::size_t>(index) * c.w * c.h;
    gfx->pushImage(kCenterX - c.w / 2, kCharacterY, c.w, c.h, reinterpret_cast<const lgfx::rgb565_t*>(frame));
    dirty->add(kCharacterY, kCharacterY + c.h - 1);
}
#endif

void draw_face(lgfx::LovyanGFX* gfx, const FaceData& data, FaceState* state, DirtyRows* dirty)
{
    static const char* const kWeekdays[] = {"Sun", "Mon", "Tue", "Wed", "Thu", "Fri", "Sat"};
    const bool full = !state->drawn;
    if (full) {
        gfx->fillScreen(TFT_BLACK);
        state->drawn = true;
        dirty->add(0, gfx->height() - 1);
    }

    // 電池（上、iPhone 風）。角の丸い電池の形の中に残量の数字を入れる。表示中は変えないので最初の 1 回だけ描く。
    if (full && data.battery_percent >= 0) {
        const int pct = std::min(100, std::max(0, data.battery_percent));
        constexpr int kW = 46, kH = 22, kR = 7, kY = kBatteryY;
        const int left            = kCenterX - (kW + 4) / 2;
        const bool low            = pct <= 20;
        const std::uint16_t body  = 0x4A69;                       // 暗い灰色（空の部分）
        const std::uint16_t level = low ? 0xF8A6 : TFT_WHITE;    // 残量（少ないと赤）
        gfx->fillRoundRect(left, kY, kW, kH, kR, body);
        const int fill_w = kW * pct / 100;
        if (fill_w > 0) {
            // 残量の部分だけを塗る（右端は丸めない）
            gfx->setClipRect(left, kY, fill_w, kH);
            gfx->fillRoundRect(left, kY, kW, kH, kR, level);
            gfx->clearClipRect();
        }
        gfx->fillRoundRect(left + kW + 2, kY + 7, 3, kH - 14, 1, body);  // 端子
        char text[8] = {};
        std::snprintf(text, sizeof(text), "%d", pct);
        gfx->loadFont(kFontBattery);
        gfx->setTextDatum(lgfx::textdatum_t::middle_center);
        // 数字は背景を描かずに重ねる（残量の白と空の灰色にまたがるので）
        gfx->setTextColor(low ? TFT_WHITE : TFT_BLACK);
        gfx->drawString(text, left + kW / 2, kY + kH / 2 + 1);
    }

#if WATCH_HAS_CHARACTER
    if (full) draw_character(gfx, s_character_index, dirty);
#endif

    if (!data.time_valid) {
        if (full) draw_time_digits(gfx, "--:--", kCenterX, kTimeY, dirty);
        gfx->unloadFont();
        return;
    }

    std::tm local = utc_from_epoch(data.epoch + kJstOffsetSec);

    // 日付（時刻の上）
    char date[48] = {};
    std::snprintf(date, sizeof(date), "%d/%d %s", local.tm_mon + 1, local.tm_mday, kWeekdays[local.tm_wday]);
    if (full || std::strcmp(date, state->date) != 0) {
        gfx->loadFont(kFontSmall);
        // 背景色を付けて描くと、文字ごとに背景を塗る範囲が隣の文字（「月」「日」の下の部分）を削っていた
        // （2026-11-15 写真 IMG_0080）。枠は draw_text_box で先に黒く塗るので、背景色なしで描く。
        // 枠を黒く塗ってから、「11/15 」を灰色、曜日を秒と同じ橙で、全体が真ん中に来るように描く（Apple Watch 風）。
        draw_text_box(gfx, "", "12/30 Wed", kCenterX, kDateY, dirty);
        char md[16] = {};
        std::snprintf(md, sizeof(md), "%d/%d ", local.tm_mon + 1, local.tm_mday);
        const char* wd = kWeekdays[local.tm_wday];
        const int md_w = gfx->textWidth(md);
        const int left = kCenterX - (md_w + gfx->textWidth(wd)) / 2;
        gfx->setTextDatum(lgfx::textdatum_t::middle_left);
        gfx->setTextColor(kColorDate);
        gfx->drawString(md, left, kDateY);
        gfx->setTextColor(kColorAccent);
        gfx->drawString(wd, left + md_w, kDateY);
        std::strcpy(state->date, date);
    }

    // 時刻（中央）
    char hm[8] = {};
    std::snprintf(hm, sizeof(hm), "%02d:%02d", local.tm_hour, local.tm_min);
    if (full || std::strcmp(hm, state->hm) != 0) {
        draw_time_digits(gfx, hm, kCenterX, kTimeY, dirty);
        std::strcpy(state->hm, hm);
    }

    // 秒（下）
    char sec[4] = {};
    std::snprintf(sec, sizeof(sec), "%02d", local.tm_sec);
    if (full || std::strcmp(sec, state->sec) != 0) {
        gfx->loadFont(kFontSec);
        gfx->setTextColor(kColorAccent);
        gfx->setTextDatum(lgfx::textdatum_t::middle_center);
        draw_text_box(gfx, sec, "88", kCenterX, kSecY, dirty);
        std::strcpy(state->sec, sec);
    }
    gfx->unloadFont();
}

enum class ShowMode { look, button, cold_boot };

// 画面はすべて専用タスク（コア 1）で扱う。電源オン・初期化・描画・消灯を同じタスクで行う
// （v2 では初期化をコア 1、描画をコア 0 で行っていて、表示が右にずれることがあった）。
// メイン側は手首の判定をして、「表示」「中止」「消灯」を指示するだけにする。
enum class DisplayCommand : int { none, show, abort };

struct DisplayJob {
    i2c_bus_handle_t bus                = nullptr;
    m5sw::wakeup::RtcWakeup* rtc        = nullptr;
    m5sw::power::PowerController* power = nullptr;
    std::int64_t wake_ms                = 0;
    volatile DisplayCommand command     = DisplayCommand::none;
    volatile ShowMode mode              = ShowMode::look;
    volatile bool stop                  = false;  // メイン側が「腕を下ろした」と判断したら立てる
    bool started                        = false;
    SemaphoreHandle_t done              = nullptr;
};
DisplayJob g_job;

void display_task(void*)
{
    DisplayJob& job              = g_job;
    const std::int64_t t_start   = now_ms();
    static watch::WatchPanel panel;
    const esp_err_t begun        = panel.begin(job.bus);
    const std::int64_t t_display = now_ms();
    FaceData data;
    std::tm utc     = {};
    data.time_valid = with_retry([&] { return job.rtc->read_time(&utc); }).ok();
    if (data.time_valid) data.epoch = epoch_from_utc(utc);
    const std::int64_t epoch_at_ms = now_ms();
    std::uint16_t mv               = 0;
    if (with_retry([&] { return job.power->read_battery_mv(&mv); }).ok() && mv > 0) {
        data.battery_mv      = mv;
        data.battery_percent = battery_percent_from_mv(mv);
    }
    const std::int64_t t_data = now_ms();
    if (begun != ESP_OK) ESP_LOGE(kTag, "DISPLAY_FAIL begin error=%s", esp_err_to_name(begun));

    while (job.command == DisplayCommand::none) vTaskDelay(pdMS_TO_TICKS(5));

    if (begun == ESP_OK && job.command == DisplayCommand::show) {
        const ShowMode mode = job.mode;
        lgfx::LovyanGFX* gfx = panel.canvas();
        FaceData frame         = data;
        if (frame.time_valid) frame.epoch += (now_ms() - epoch_at_ms) / 1000;
        FaceState face_state;
        DirtyRows dirty;
        const std::int64_t t_draw0 = now_ms();
        draw_face(gfx, frame, &face_state, &dirty);
        const std::int64_t t_draw1 = now_ms();
        const esp_err_t pushed = panel.push_rows(dirty.y0, dirty.y1);
        const std::int64_t t_push = now_ms();
        (void)panel.set_brightness_percent(kBrightness);  // 時計を書いてから光らせる
        ESP_LOGI(kTag, "FRAME_TIMING cmd_wait_end=%lld draw=%lldms push=%lldms", t_draw0, t_draw1 - t_draw0,
                 t_push - t_draw1);
        if (pushed != ESP_OK) ESP_LOGW(kTag, "PUSH_FAIL error=%s", esp_err_to_name(pushed));
        const std::int64_t first_frame_ms = now_ms();
        ESP_LOGI(kTag, "SHOW %s wake->frame %lldms (display start=%lld ready=%lld data=%lld) time=%s bat=%umV(%d%%)",
                 mode == ShowMode::look ? "look" : (mode == ShowMode::button ? "button" : "boot"),
                 first_frame_ms - job.wake_ms, t_start, t_display, t_data, data.time_valid ? "ok" : "invalid",
                 data.battery_mv, data.battery_percent);

        const std::int64_t limit_us =
            mode == ShowMode::look ? kLookShowMaxUs : (mode == ShowMode::button ? kButtonShowUs : kColdBootShowUs);
        std::int64_t deadline_us = esp_timer_get_time() + limit_us;
        int last_sec             = static_cast<int>(frame.epoch % 60);
        const char* reason       = "timeout";
        bool key_was_down        = any_key_down();
        // 表示中にタップしたら、キャラクターの表情を次に変える（1 回のタップで割り込みが何度か来るので 400ms は無視）。
        s_touch_edges = 0;
        int touch_irqs = 0;
        std::int64_t next_tap_ms = 0;
        while (esp_timer_get_time() < deadline_us) {
            if (job.stop) {
                reason = "lowered";
                break;
            }
            if (data.time_valid) {
                frame.epoch   = data.epoch + (now_ms() - epoch_at_ms) / 1000;
                const int sec = static_cast<int>(frame.epoch % 60);
                if (sec != last_sec) {
                    last_sec = sec;
                    DirtyRows changed;
                    draw_face(gfx, frame, &face_state, &changed);
                    if (!changed.empty()) {
                        const esp_err_t e = panel.push_rows(changed.y0, changed.y1);
                        if (e != ESP_OK) ESP_LOGW(kTag, "PUSH_FAIL error=%s", esp_err_to_name(e));
                    }
                }
            }
            if (s_touch_edges != 0) {
                touch_irqs += s_touch_edges;
                s_touch_edges = 0;
                if (now_ms() >= next_tap_ms) {
                    next_tap_ms = now_ms() + 400;
                    deadline_us = std::max(deadline_us, esp_timer_get_time() + kButtonShowUs);  // 触っている間は消さない
#if WATCH_HAS_CHARACTER
                    ++s_character_index;
                    DirtyRows changed;
                    draw_character(gfx, s_character_index, &changed);
                    if (!changed.empty()) (void)panel.push_rows(changed.y0, changed.y1);
#endif
                }
            }
            // ボタンを押したら延長する。
            const bool key_down = any_key_down();
            if (key_down != key_was_down) {
                ESP_LOGI(kTag, "KEY %s (a=%d b=%d)", key_down ? "down" : "up", gpio_get_level(m5sw::board::kKey1),
                         gpio_get_level(m5sw::board::kKey2));
            }
            if (key_down && !key_was_down) deadline_us = esp_timer_get_time() + kButtonShowUs;
            key_was_down = key_down;
            vTaskDelay(pdMS_TO_TICKS(40));
        }
        const std::int64_t shown_ms = now_ms() - first_frame_ms;
        const esp_err_t off         = panel.sleep();
#if WATCH_HAS_CHARACTER
        ++s_character_index;  // 次に点けたときは別の表情
#endif
        ESP_LOGI(kTag, "HIDE after %lldms (%s) power_off=%s touch_irq_samples=%d", shown_ms, reason,
                 esp_err_to_name(off), touch_irqs);
    } else {
        // 中止、または初期化に失敗したとき。電源は必ず切る。
        const esp_err_t off = panel.sleep();
        ESP_LOGI(kTag, "PREP_ABORT power_off=%s", esp_err_to_name(off));
    }
    xSemaphoreGive(job.done);
    vTaskDelete(nullptr);
}

// 画面の準備を始める（何度呼んでも 1 回だけ）。
void display_start()
{
    if (g_job.started) return;
    g_job.started = true;
    g_job.done    = xSemaphoreCreateBinary();
    xTaskCreatePinnedToCore(display_task, "display", 16384, nullptr, 5, nullptr, 1);  // なめらかフォントの描画でスタックを使う
}

void display_wait_done()
{
    if (g_job.started) xSemaphoreTake(g_job.done, portMAX_DELAY);
}

// 準備した画面を使わずに電源を切る（手首の判定が「見ていない」だったとき）。
void display_abort()
{
    if (!g_job.started) return;
    g_job.command = DisplayCommand::abort;
    display_wait_done();
}

// 画面を点けて文字盤を出し、消えるまで待つ。手首で点けたときは、腕を下ろしたかをメイン側で FIFO から見る。
bool g_shown = false;  // この起床で画面を出したか（消すときに IOE1 を初期化するので、タッチの設定をし直す）

void show_watch(ImuFifo* fifo, ShowMode mode)
{
    g_shown = true;
    display_start();
    g_job.mode    = mode;
    g_job.command = DisplayCommand::show;
    int low_run   = 0;
    const std::int64_t shown_from_us = esp_timer_get_time();
    while (xSemaphoreTake(g_job.done, pdMS_TO_TICKS(40)) != pdTRUE) {
        if (mode != ShowMode::look || g_job.stop) continue;
        const int added = fifo->pull();
        for (int i = added - 1; i >= 0; --i) {
            Accel a;
            if (!hist_avg(i, &a)) continue;
            low_run = a.still_viewing() ? 0 : low_run + 1;
        }
        if (low_run >= kViewEndFrames && esp_timer_get_time() - shown_from_us >= kLookMinShowUs) g_job.stop = true;
    }
}

}  // namespace

extern "C" void app_main(void)
{
    const std::int64_t app_start_ms = now_ms();
    // 寝ている間〜ここまでのタップの記録（GPIO13 を設定し直す前に読む）
    const bool touch_latched = m5sw::power::PowerController::take_touch_latch();
    m5sw::test_support::TestContext context = {};
    ESP_ERROR_CHECK(m5sw::test_support::initialize(&context));
    esp_log_level_set("i2c_bus", ESP_LOG_ERROR);
    esp_log_level_set("M5IOE1_I2C", ESP_LOG_WARN);
    esp_log_level_set("M5IOE1_GPIO", ESP_LOG_WARN);
    esp_log_level_set("M5IOE1_SYS", ESP_LOG_WARN);
    esp_log_level_set("M5PM1_I2C", ESP_LOG_WARN);
    esp_log_level_set("M5PM1_SYS", ESP_LOG_WARN);

    const esp_sleep_wakeup_cause_t cause = context.wake_cause;
    // 起きた直後の PM1 IRQ 線（GPIO12）の状態。寝ている間は Low で起きる設定。
    const int pm1_irq_level = rtc_gpio_get_level(m5sw::board::kPm1Irq);
    const bool cold_boot = cause != ESP_SLEEP_WAKEUP_EXT0 && cause != ESP_SLEEP_WAKEUP_EXT1 &&
                           cause != ESP_SLEEP_WAKEUP_TIMER;
    keys_as_inputs();
    watch_keys_while_awake();
    // ボタンで起きたかの手がかり（ext1 の状態と、いま押されているか）。ext0（手首）と同時だと原因は ext0 と報告される。
    const std::uint64_t ext1_status = esp_sleep_get_ext1_wakeup_status();
    const bool key_at_boot          = any_key_down();

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

    m5sw::wakeup::ImuStatus retained_imu;
    auto imu_result = m5sw::wakeup::ImuWakeup::read_retained_status(bus, &retained_imu);
    if (!imu_result.ok()) fail("imu_retained", imu_result.error, imu_result.raw_error);

    // BMI270 を初期化するのはコールドブートのときだけ（any-motion と FIFO の設定は寝ている間も残る）。
    static m5sw::wakeup::WakeController wake;
    auto wake_result = wake.begin(bus, cold_boot);
    if (!wake_result.ok()) fail("wake_begin", wake_result.error, wake_result.raw_error);
    m5sw::wakeup::WakeSnapshot snapshot;
    wake_result = wake.capture(retained_rtc, retained_imu, &snapshot);
    if (!wake_result.ok()) fail("wake_capture", wake_result.error, wake_result.raw_error);

    static m5sw::power::PowerController power;
    auto power_result = with_retry([&] { return power.begin(bus); });
    if (!power_result.ok()) fail("pm1_begin", power_result.error, power_result.raw_error);
    power.set_touch_wake(true);  // 画面をタップしても起きる

    static ImuFifo fifo(bus);
    g_job.bus     = bus;
    g_job.rtc     = &rtc;
    g_job.power   = &power;
    g_job.wake_ms = app_start_ms;

    if (cold_boot) {
        s_wake_count = s_show_count = 0;
        s_awake_ms                  = 0;
        ESP_LOGI(kTag, "COLD_BOOT cause=%d reset_reason=%d", static_cast<int>(cause), static_cast<int>(esp_reset_reason()));
        {
            const esp_err_t mux = watch::hold_usb_mux(bus);
            if (mux != ESP_OK) ESP_LOGW(kTag, "USB_MUX hold error=%s", esp_err_to_name(mux));
        }
        (void)sync_time_from_usb(&rtc);
        show_watch(&fifo, ShowMode::cold_boot);
        maybe_wifi_time_sync(&rtc, true);  // コールドブートでは毎回試す（表示の後。Wi-Fi の動作確認を兼ねる）
    } else if ((ext1_status & (1ULL << m5sw::board::kTouchIrq)) != 0 || touch_latched) {
        // タップで起きた。手首の動き（タップの振動）で先に起きた場合や、寝る直前の待ちの間のタップも、記録から分かる。
        ++s_wake_count;
        ++s_show_count;
        ESP_LOGI(kTag, "WAKE #%lu tap cause=%d ext1=0x%llx latched=%d", static_cast<unsigned long>(s_wake_count),
                 static_cast<int>(cause), static_cast<unsigned long long>(ext1_status), touch_latched);
        show_watch(&fifo, ShowMode::button);
    } else if (cause == ESP_SLEEP_WAKEUP_EXT1 ||
               (cause == ESP_SLEEP_WAKEUP_EXT0 && (key_at_boot || (ext1_status & kKeyWakeMask) != 0))) {
        ++s_wake_count;
        ++s_show_count;
        ESP_LOGI(kTag, "WAKE #%lu button cause=%d ext1=0x%llx key=%d", static_cast<unsigned long>(s_wake_count),
                 static_cast<int>(cause), static_cast<unsigned long long>(ext1_status), key_at_boot);
        show_watch(&fifo, ShowMode::button);
    } else if (cause == ESP_SLEEP_WAKEUP_EXT0) {
        ++s_wake_count;
        const esp_err_t aps_result = fifo.begin_awake();
        if (aps_result != ESP_OK) ESP_LOGW(kTag, "APS_OFF failed error=%s", esp_err_to_name(aps_result));
        const LookResult look = check_look(&fifo, display_start);
        ESP_LOGI(kTag, "WAKE #%lu imu %s decided=%lldms acc=(%+.2f %+.2f %+.2f) pre=(%+.2f %+.2f %+.2f) raise=%.0fdeg "
                 "shows=%lu avg_awake=%llums ext1=0x%llx",
                 static_cast<unsigned long>(s_wake_count), look.reason, look.at_ms, look.acc.x, look.acc.y,
                 look.acc.z, look.pre.x, look.pre.y, look.pre.z, look.raise_deg, static_cast<unsigned long>(s_show_count),
                 s_wake_count > 1 ? s_awake_ms / (s_wake_count - 1) : 0ULL, static_cast<unsigned long long>(ext1_status));
        if (look.looking) {
            ++s_show_count;
            show_watch(&fifo, ShowMode::look);
        } else if (s_touched_awake) {
            ++s_show_count;
            ESP_LOGI(kTag, "TAP while awake");
            show_watch(&fifo, ShowMode::button);
        } else if (s_key_pressed_awake || any_key_down()) {
            ++s_show_count;
            ESP_LOGI(kTag, "BUTTON while awake");
            show_watch(&fifo, ShowMode::button);
        } else {
            display_abort();  // 姿勢に入りかけて画面の準備を始めていたら、表示せずに切る
        }
    } else {
        ++s_wake_count;
        ++s_timer_count;
        // 1 時間まったく起きなかった。起床経路の状態を出してから、下で動き検出を設定し直す。
        const int aps = fifo.begin_awake();
        ESP_LOGW(kTag,
                 "TIMER_WAKE #%lu (timer=%lu) gpio12=%d pm1_gpio_irq=0x%02X pm1_btn=0x%02X imu_status=0x%04X | "
                 "bmi: internal=0x%02X int1_io=0x%02X int1_map=0x%02X acc_conf=0x%02X pwr_conf=0x%02X pwr_ctrl=0x%02X "
                 "fifo_cfg1=0x%02X aps_off=%s",
                 static_cast<unsigned long>(s_wake_count), static_cast<unsigned long>(s_timer_count), pm1_irq_level,
                 snapshot.pm1_gpio_irq, snapshot.pm1_button_irq, retained_imu.raw, fifo.read_reg(0x21),
                 fifo.read_reg(0x53), fifo.read_reg(0x56), fifo.read_reg(0x40), fifo.saved_pwr_conf(),
                 fifo.read_reg(0x7D), fifo.read_reg(0x49), esp_err_to_name(aps));
    }

    wake_result = with_retry([&] { return wake.clear_snapshot(snapshot); });
    if (!wake_result.ok()) fail("wake_clear", wake_result.error, wake_result.raw_error);
    rtc_result = with_retry([&] { return rtc.clear_flags(retained_rtc); });
    if (!rtc_result.ok()) fail("rtc_clear", rtc_result.error, rtc_result.raw_error);
    rtc_result = with_retry([&] { return rtc.disable_interrupts(); });
    if (!rtc_result.ok()) fail("rtc_disable", rtc_result.error, rtc_result.raw_error);

    const bool tap_wake = !cold_boot && (touch_latched || (ext1_status & (1ULL << m5sw::board::kTouchIrq)) != 0);
    if (cold_boot || (cause == ESP_SLEEP_WAKEUP_TIMER && !tap_wake)) {
        // ボード全体を L2 用に整え、BMI270 の any-motion と FIFO を設定する（タイマー起床では設定し直して復旧を試みる）。
        wake_result = wake.begin_imu();
        if (!wake_result.ok()) fail("imu_begin", wake_result.error, wake_result.raw_error);
        power_result = power.prepare_l2(m5sw::power::WakeSelection::rtc_imu, /*keep_charging=*/true);
        if (!power_result.ok()) fail("l2_prepare", power_result.error, power_result.raw_error);
        watch::panel_power_lost();  // prepare_l2 が IOE1 を初期化して画面の電源を切った
        configure_touch_wake(bus);
        wake_result = wake.select_source(m5sw::wakeup::WakeSelection::imu, m5sw::wakeup::ImuWakeProfile::low_power,
                                         kAnyDuration, kAnyThreshold);
        if (!wake_result.ok()) fail("imu_arm", wake_result.error, wake_result.raw_error);
        const esp_err_t fifo_result = fifo.configure();
        if (fifo_result != ESP_OK) fail("fifo_config", fifo_result, fifo_result);
        hist_reset();
    } else {
        const esp_err_t aps_result = fifo.end_awake();
        if (aps_result != ESP_OK) ESP_LOGW(kTag, "APS_RESTORE failed error=%s", esp_err_to_name(aps_result));
    }

    hourly_tasks(&rtc, &power);
    wait_keys_released();
    ESP_LOGI(kTag, "SLEEP at %lldms", now_ms());
    vTaskDelay(kLogFlushWait);
    if (!cold_boot) s_awake_ms += now_ms() + 1000;  // enter_l2_quick 内の 1 秒待ちも足す
    power_result = power.enter_l2_quick(m5sw::power::WakeSelection::rtc_imu, kTimerFallbackUs,
                                        m5sw::power::L2Pm1IdleSleep::enabled, kKeyWakeMask);
    fail("l2_returned", power_result.error, power_result.raw_error);
}
