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
#include "esp_pm.h"
#include "esp_random.h"
#include "esp_rtc_time.h"
#include "esp_rom_sys.h"
#include "esp_sleep.h"
#include "esp_system.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "freertos/task.h"
#include "i2c_bus.h"
#include "m5sw_board.hpp"
#include "nvs.h"
#include "nvs_flash.h"
#include "time_sync.hpp"
#include "watch_audio.hpp"
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
constexpr std::uint8_t kAnyThreshold     = 150;  // 約 72mg 以上の変化（100 では着けているだけで 1 時間に 1300 回空振りした。2026-11-17）
// 保険のタイマー。v2/v3 で「寝たまま起きなくなる」ことがあったので、調査中は 1 分にして、
// 起きたら起床経路の状態をログに出し、動き検出を設定し直す（落ち着いたら 1 時間に戻す）。
constexpr std::uint64_t kTimerFallbackUs = 3600ULL * 1000 * 1000;  // 1 時間（電池の記録も兼ねる）
constexpr std::uint64_t kKeyWakeMask     = (1ULL << m5sw::board::kKey1) | (1ULL << m5sw::board::kKey2);

// ---- 画面 ----
constexpr bool kTapSound               = false;  // タップで音を鳴らす（音はユーザーが用意するまで止めておく。2026-11-15）
constexpr std::uint8_t kBrightness      = 50;        // %（60 から下げた。2026-11-16 省電力）
constexpr std::int64_t kLookShowMaxUs   = 8000000;   // 見続けていても 8 秒で消す
constexpr std::int64_t kLookMinShowUs   = 2500000;   // 手首で点けたら、腕を下ろしてもこの時間は消さない
constexpr std::int64_t kButtonShowUs    = 4000000;   // ボタンで点けたときは 4 秒（押すたびに延長）
constexpr std::int64_t kColdBootShowUs  = 5000000;
constexpr std::int32_t kJstOffsetSec    = 9 * 3600;
// 省電力（2026-11-16）
constexpr int kCpuMhzIdle               = 80;   // ふだんの CPU の上限
constexpr int kCpuMhzDraw               = 240;  // 最初の 1 枚を描く間だけ上げる（点くまでを遅らせない。160→240 2026-11-17）
constexpr int kCpuMhzWifi               = 160;  // Wi-Fi の時刻合わせ
constexpr int kCpuMhzMin                = 40;   // 何もしていないとき
constexpr int kFrameMs                  = 40;   // 表示中の更新間隔（25fps）
constexpr int kNightFromHour            = 1;    // この時間帯（日本時間）は手首の動きで起きない（タップとボタンだけ）
constexpr int kNightToHour              = 6;
constexpr std::uint16_t kUsbVinMv       = 4000; // USB の電圧がこれ以上なら「挿さっている」
// 充電中（USB 給電中）は、夜以外は暗い時刻の画面を出し続ける（置き時計。2026-11-18）。
constexpr std::uint8_t kDockBrightness  = 10;   // %
constexpr int kDockCheckMs              = 1000;   // USB が抜けたか・夜になったかを見る間隔
constexpr int kDockBatteryMs            = 60000;  // 電池の表示を読み直す間隔
constexpr int kCenterX                  = 234;  // 表示領域 468x466 の中心
constexpr int kCenterY                  = 233;
constexpr std::uint16_t kColorDim       = 0x8410;  // 灰色
constexpr std::uint16_t kColorAccent    = 0xFD20;  // 橙
constexpr std::uint16_t kColorDate      = 0xC618;  // 明るい灰色

// 文字盤の縦の配置。キャラクターがあるときは時刻を上に寄せ、下にキャラクターを置く。
#if WATCH_HAS_CHARACTER
// 2026-11-15 動画 IMG_0078 で日付と秒の端が削れていたので、PC で配置を再現して決め直した。
// 電池（高さ 32px）の下端が、日付の枠の黒塗り（y=57 から）で 3 行削れていたので 4px 上げた（2026-11-18 写真 IMG_0099）。
constexpr int kBatteryY   = 24;
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
constexpr int kLookHoldFrames       = 3;   // 60ms（v1 は 200ms、7=140ms、4=80ms から速さのためさらに短くした）
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
// 電力の内訳（2026-11-16）。コールドブートから数える。
RTC_DATA_ATTR std::uint32_t s_false_count    = 0;  // 手首の動きで起きたが点けなかった回数
RTC_DATA_ATTR std::uint64_t s_false_awake_ms = 0;  // そのときに起きていた時間の合計
RTC_DATA_ATTR std::uint64_t s_shown_ms       = 0;  // 画面を点けていた時間の合計
RTC_DATA_ATTR std::uint32_t s_timer_count  = 0;
RTC_DATA_ATTR std::int64_t s_last_sync_epoch    = 0;  // 最後に時刻を合わせた時刻（UTC の秒）
RTC_DATA_ATTR std::int64_t s_last_wifi_try_epoch = 0;  // 最後に Wi-Fi で合わせようとした時刻

// 電池の記録（15 分ごとに 1 件、フラッシュに残す。USB を外して使った後、つないだときに読み出す）。
// 回数と時間はコールドブートからの累計。
struct BatteryRecord {
    std::uint32_t epoch;
    std::uint16_t mv;
    std::uint16_t reserved;
    std::uint32_t wakes;        // 起きた回数
    std::uint32_t shows;        // 画面を点けた回数
    std::uint32_t false_wakes;  // 手首の動きで起きたが点けなかった回数
    std::uint32_t awake_cs;     // 起きていた時間（1/100 秒。寝る前の待ちは除く）
    std::uint32_t false_cs;     // そのうち点けなかった起床の分
    std::uint32_t shown_cs;     // 画面を点けていた時間
};
constexpr int kBatteryRecords        = 192;  // 48 時間分
constexpr std::int64_t kRecordEveryS = 900;
constexpr std::int64_t kReinitMinSleepMs = 20LL * 60 * 1000;  // これより短い眠りからのタイマー起床では設定し直さない
RTC_DATA_ATTR std::int64_t s_next_record_epoch = 0;
RTC_DATA_ATTR std::int64_t s_next_hourly_epoch = 0;  // 次に 1 時間ごとの用事（時刻合わせの確認）をする時刻
constexpr int kHistLen                     = 128;
RTC_DATA_ATTR std::int16_t s_hist[kHistLen][3];
RTC_DATA_ATTR std::uint16_t s_hist_head  = 0;
RTC_DATA_ATTR std::uint16_t s_hist_count = 0;

// 起きた理由の記録（2026-11-17。USB を抜いている間に何が起きたかを、つないだときに読む）。
// リセット（BOARD_FAIL の再起動）でも消えない領域に置く。電源を入れ直したときは magic で初期化する。
struct WakeLogEntry {
    std::uint32_t rc_s;      // RTC の時計（秒）
    std::uint32_t slept_ms;  // 前回寝てから起きるまで（分からなければ 0xFFFFFFFF）
    std::uint16_t awake_ms;  // 起きていた時間（寝る前の待ちは除く）
    std::uint8_t cause;      // esp_sleep_wakeup_cause_t。コールドブートは 100 + リセット理由
    std::uint8_t flags;      // kWl*
};
constexpr int kWakeLogLen             = 64;
constexpr std::uint32_t kWakeLogMagic = 0x574C4F47;
enum : std::uint8_t {
    kWlTouchLatch = 1,    // 寝ている間のタップの記録があった
    kWlExt1Touch  = 2,    // ext1 の原因がタッチの線
    kWlExt1Key    = 4,    // ext1 の原因がボタン
    kWlPm1IrqLow  = 8,    // 起きた直後の PM1 IRQ 線が Low
    kWlQuickArmed = 16,   // 寝る前の待ちでタップを見つけて 1ms で寝た
    kWlReinit     = 32,   // ボードを設定し直した（1 時間ごとの見回りの経路）
    kWlShown      = 64,   // 画面を点けた
    kWlFalse      = 128,  // 手首の動きで起きたが点けなかった
};
RTC_NOINIT_ATTR WakeLogEntry s_wlog[kWakeLogLen];
RTC_NOINIT_ATTR std::uint32_t s_wlog_magic;
RTC_NOINIT_ATTR std::uint32_t s_wlog_head;
RTC_NOINIT_ATTR std::uint32_t s_wlog_count;
RTC_NOINIT_ATTR std::int64_t s_sleep_start_rc_us;
RTC_NOINIT_ATTR char s_fail_stage[16];

void wake_log_add(const WakeLogEntry& e)
{
    if (s_wlog_magic != kWakeLogMagic) {
        s_wlog_magic = kWakeLogMagic;
        s_wlog_head = s_wlog_count = 0;
        s_fail_stage[0]            = '\0';
    }
    s_wlog[s_wlog_head % kWakeLogLen] = e;
    s_wlog_head                       = (s_wlog_head + 1) % kWakeLogLen;
    if (s_wlog_count < kWakeLogLen) ++s_wlog_count;
}

[[noreturn]] void fail(const char* stage, esp_err_t error, int raw_error)
{
    ESP_LOGE(kTag, "BOARD_FAIL stage=%s error=%s raw=%d -> restart", stage, esp_err_to_name(error), raw_error);
    std::strncpy(s_fail_stage, stage, sizeof(s_fail_stage) - 1);
    s_fail_stage[sizeof(s_fail_stage) - 1] = '\0';
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
        vTaskDelay(pdMS_TO_TICKS(10));  // PM1 は起こしてから 10ms で応える
        result = fn();
    }
    return result;
}

std::int64_t now_ms()
{
    return esp_timer_get_time() / 1000;
}

// ---------------------------------------------------------------------------
// 省電力: CPU の速さ・浅い眠り（light sleep）・ログ
// ---------------------------------------------------------------------------
// 起きてから寝るまでの各段階の時刻（USB のときにまとめて出す。どこで時間を使っているかを見る）
struct Mark {
    const char* name;
    std::int32_t ms;
};
Mark s_marks[20];
int s_mark_count = 0;
void mark(const char* name)
{
    if (s_mark_count < 20) s_marks[s_mark_count++] = {name, static_cast<std::int32_t>(now_ms())};
}
int s_pm1_wake_polls = 0;
std::int64_t s_lit_ms = -1;      // 画面を光らせた時刻（起動からの ms）
bool s_lit_predrawn  = false;  // PM1 が応答するまでに読んだ回数（計測用）

void print_marks()
{
    char line[400];
    int n = 0;
    for (int i = 0; i < s_mark_count && n < static_cast<int>(sizeof(line)) - 24; ++i) {
        n += snprintf(line + n, sizeof(line) - n, " %s=%ld", s_marks[i].name, static_cast<long>(s_marks[i].ms));
    }
    line[n] = 0;
    ESP_LOGI(kTag, "TIMELINE%s pm1_polls=%d lit=%lld predrawn=%d", line, s_pm1_wake_polls, s_lit_ms, s_lit_predrawn);
}


// PM1 は I2C の眠りから覚めるのに数〜十数 ms かかる。M5PM1 の begin は読み込みに 1 回失敗すると
// 50ms 待ってやり直すので、先にこちらで 2ms 刻みに応答を確かめておく（読むこと自体が起こす合図になる）。
void wait_pm1_awake(i2c_bus_handle_t bus)
{
    i2c_bus_device_handle_t dev = i2c_bus_device_create(bus, m5sw::board::kPm1Address, 100000);
    if (dev == nullptr) return;
    std::uint8_t value = 0;
    for (s_pm1_wake_polls = 1; s_pm1_wake_polls <= 20; ++s_pm1_wake_polls) {
        if (i2c_bus_read_byte(dev, 0x00, &value) == ESP_OK) break;  // 0x00 = DEVICE_ID
        vTaskDelay(pdMS_TO_TICKS(2));
    }
    i2c_bus_device_delete(&dev);
}

bool s_usb_power = false;  // USB が挿さっている（ログを出し、浅い眠りは使わない。USB が切れるため）

// 何もしていない間は CPU を kCpuMhzMin に落とし、light_sleep なら浅く眠らせる。
void pm_apply(int max_mhz, bool light_sleep)
{
    esp_pm_config_t cfg    = {};
    cfg.max_freq_mhz       = max_mhz;
    cfg.min_freq_mhz       = kCpuMhzMin;
    cfg.light_sleep_enable = light_sleep && !s_usb_power;
    const esp_err_t err    = esp_pm_configure(&cfg);
    if (err != ESP_OK) ESP_LOGW(kTag, "PM_CONFIG max=%d light=%d error=%s", max_mhz, light_sleep, esp_err_to_name(err));
}

// 浅い眠りの間も、使っているピンの設定を保つ（既定では眠る間すべてのピンが切り離される）。
void keep_pins_in_light_sleep()
{
    for (const gpio_num_t pin :
         {m5sw::board::kKey1, m5sw::board::kKey2, m5sw::board::kPm1Irq, m5sw::board::kTouchIrq, m5sw::board::kI2cSda,
          m5sw::board::kI2cScl, GPIO_NUM_14, GPIO_NUM_38, GPIO_NUM_39, GPIO_NUM_40, GPIO_NUM_41, GPIO_NUM_42,
          GPIO_NUM_45, GPIO_NUM_46}) {
        gpio_sleep_sel_dis(pin);
    }
}

// USB が挿さっていないときはログを出さない（UART への書き出しで起きている時間が延びる）。
void apply_log_levels(bool usb)
{
    // USB なしでは何も出さない。ログは UART と USB に出るので、1 行ごとに数 ms〜50ms 待たされる
    // （寝起き直後は USB の再接続前で 50ms の送信待ちになる）。異常停止の表示はこの設定と関係なく出る。
    esp_log_level_set("*", usb ? ESP_LOG_INFO : ESP_LOG_NONE);  // "*" は個別の設定を消すので先に呼ぶ
    if (!usb) return;
    esp_log_level_set("i2c_bus", ESP_LOG_ERROR);
    esp_log_level_set("pm", ESP_LOG_WARN);  // CPU 設定のたびの表示で、USB のときだけ計測が遅く見えるのを防ぐ
    esp_log_level_set("M5IOE1_I2C", ESP_LOG_WARN);
    esp_log_level_set("M5IOE1_GPIO", ESP_LOG_WARN);
    esp_log_level_set("M5IOE1_SYS", ESP_LOG_WARN);
    esp_log_level_set("M5PM1_I2C", ESP_LOG_WARN);
    esp_log_level_set("M5PM1_SYS", ESP_LOG_WARN);
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
    pm_apply(kCpuMhzWifi, false);  // 早く終わらせたほうが無線の電力が少ない
    const esp_err_t err = watch::wifi_time_sync(rtc, &synced, &drift);
    pm_apply(kCpuMhzIdle, true);
    if (err == ESP_OK) {
        s_last_sync_epoch     = synced;
        s_last_wifi_try_epoch = synced;
    }
    ESP_LOGI(kTag, "WIFI_SYNC result=%s drift=%llds took=%lldms", esp_err_to_name(err), static_cast<long long>(drift),
             now_ms() - start);
}

void battery_append(const BatteryRecord& rec);

// 1 時間ごとの用事。起きた理由によらず、前回から 1 時間たっていればする（腕を動かしている間は定期起床が来ないため）。
// 読んだ時刻（UNIX 秒）を返す。読めなければ 0。呼び出し側は夜の判定にも使う（RTC を 2 回読まない）。
std::int64_t hourly_tasks(m5sw::wakeup::RtcWakeup* rtc, m5sw::power::PowerController* power)
{
    std::tm utc = {};
    if (!rtc->read_time(&utc).ok()) return 0;
    const std::int64_t now = epoch_from_utc(utc);
    if (s_next_record_epoch == 0 || now >= s_next_record_epoch) {
        s_next_record_epoch = now + kRecordEveryS;
        std::uint16_t mv    = 0;
        (void)with_retry([&] { return power->read_battery_mv(&mv); });
        BatteryRecord rec = {};
        rec.epoch         = static_cast<std::uint32_t>(now);
        rec.mv            = mv;
        rec.wakes         = s_wake_count;
        rec.shows         = s_show_count;
        rec.false_wakes   = s_false_count;
        rec.awake_cs      = static_cast<std::uint32_t>(s_awake_ms / 10);
        rec.false_cs      = static_cast<std::uint32_t>(s_false_awake_ms / 10);
        rec.shown_cs      = static_cast<std::uint32_t>(s_shown_ms / 10);
        battery_append(rec);
    }
    if (s_next_hourly_epoch != 0 && now < s_next_hourly_epoch) return now;
    s_next_hourly_epoch = now + 3600;
    maybe_wifi_time_sync(rtc, false);
    return now;
}

// 電池の記録はフラッシュ（NVS）にも残す。電池が切れると RTC メモリは消えるため。
bool battery_nvs_open(nvs_handle_t* h, nvs_open_mode_t mode)
{
    esp_err_t err = nvs_flash_init();
    if (err == ESP_ERR_NVS_NO_FREE_PAGES || err == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        nvs_flash_erase();
        err = nvs_flash_init();
    }
    return err == ESP_OK && nvs_open("battery", mode, h) == ESP_OK;
}

int battery_read(nvs_handle_t h, BatteryRecord* out)
{
    size_t size = sizeof(BatteryRecord) * kBatteryRecords;
    if (nvs_get_blob(h, "hist3", out, &size) != ESP_OK) return 0;
    return static_cast<int>(size / sizeof(BatteryRecord));
}

void battery_append(const BatteryRecord& rec)
{
    nvs_handle_t h;
    if (!battery_nvs_open(&h, NVS_READWRITE)) return;
    auto* list = static_cast<BatteryRecord*>(calloc(kBatteryRecords, sizeof(BatteryRecord)));
    if (list != nullptr) {
        int n = battery_read(h, list);
        if (n >= kBatteryRecords) {
            std::memmove(list, list + 1, sizeof(BatteryRecord) * (kBatteryRecords - 1));
            n = kBatteryRecords - 1;
        }
        list[n++] = rec;
        nvs_set_blob(h, "hist3", list, sizeof(BatteryRecord) * n);
        nvs_commit(h);
        free(list);
    }
    nvs_close(h);
}

// 電池の記録を全部ログに出す（USB がつながっているときに毎回。電池の持ちを測るため）。
void dump_battery_history()
{
    nvs_handle_t h;
    if (!battery_nvs_open(&h, NVS_READONLY)) return;
    auto* list = static_cast<BatteryRecord*>(calloc(kBatteryRecords, sizeof(BatteryRecord)));
    if (list != nullptr) {
        const int n = battery_read(h, list);
        for (int i = 0; i < n; ++i) {
            const BatteryRecord& r = list[i];
            ESP_LOGI(kTag, "BAT_HIST %d epoch=%lu mv=%u wakes=%lu shows=%lu false=%lu awake_cs=%lu false_cs=%lu shown_cs=%lu",
                     i, static_cast<unsigned long>(r.epoch), r.mv, static_cast<unsigned long>(r.wakes),
                     static_cast<unsigned long>(r.shows), static_cast<unsigned long>(r.false_wakes),
                     static_cast<unsigned long>(r.awake_cs), static_cast<unsigned long>(r.false_cs),
                     static_cast<unsigned long>(r.shown_cs));
            if (i % 16 == 15) vTaskDelay(pdMS_TO_TICKS(20));  // USB に流し切る
        }
        free(list);
    }
    nvs_close(h);
    if (s_wlog_magic != kWakeLogMagic) return;
    ESP_LOGI(kTag, "WAKE_LOG count=%lu last_fail=%s", static_cast<unsigned long>(s_wlog_count),
             s_fail_stage[0] != '\0' ? s_fail_stage : "-");
    for (std::uint32_t k = 0; k < s_wlog_count; ++k) {
        const WakeLogEntry& e = s_wlog[(s_wlog_head + kWakeLogLen - s_wlog_count + k) % kWakeLogLen];
        ESP_LOGI(kTag, "WAKE_LOG %lu rc=%lu slept=%ld awake=%u cause=%u flags=0x%02X", static_cast<unsigned long>(k),
                 static_cast<unsigned long>(e.rc_s), static_cast<long>(static_cast<std::int32_t>(e.slept_ms)),
                 e.awake_ms, e.cause, e.flags);
        if (k % 16 == 15) vTaskDelay(pdMS_TO_TICKS(20));
    }
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
        vTaskDelay(pdMS_TO_TICKS(20));  // FIFO は 50Hz。待つ間は CPU を浅く眠らせる
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

RTC_DATA_ATTR int s_character_index = 0;  // 今の表情（画像の番号）

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
// ストップウォッチとタイマー（寝ている間も進む。状態は RTC メモリに置く）
// ボタン: KEYA（GPIO2）= 次の画面、KEYB（GPIO1）= スタート／ストップ、KEYB 長押し = リセット。
// タイマーの画面でタップすると時間を変える。
// ---------------------------------------------------------------------------
enum class Screen : int { clock = 0, stopwatch = 1, timer = 2 };
constexpr int kScreenCount             = 3;
constexpr int kTimerPresetsS[]         = {60, 180, 300, 600, 900, 1800, 3600};
constexpr int kTimerPresetCount        = sizeof(kTimerPresetsS) / sizeof(kTimerPresetsS[0]);
constexpr std::int64_t kAppIdleBackS   = 120;       // 止まっているアプリは 2 分さわらなければ時計に戻す
constexpr std::int64_t kAlarmUs        = 30000000;  // タイマーが鳴る長さ
constexpr std::int64_t kLongPressMs    = 700;
constexpr std::int64_t kAlarmEarlyUs   = 200000;    // この時間より残りが短ければ「時間になった」とみなす
constexpr std::int64_t kNoEpoch        = INT64_MIN;

struct AppState {
    int screen                  = 0;
    std::int64_t last_use_epoch = 0;
    bool sw_running             = false;
    std::int64_t sw_base_us     = 0;  // 止めるまでにたまった時間
    std::int64_t sw_start_rc_us = 0;
    std::int64_t sw_start_epoch = 0;
    int tm_preset               = 1;  // 3 分
    bool tm_running             = false;
    std::int64_t tm_left_us     = 0;  // 一時停止中の残り（0 なら設定した時間から）
    std::int64_t tm_end_rc_us   = 0;
    std::int64_t tm_end_epoch   = 0;
    bool tm_alarm               = false;
};
RTC_DATA_ATTR AppState s_app;

// 寝ている間も進む ESP32 の RTC タイマー（内蔵の発振器なので少しずれる）。
std::int64_t rc_now_us() { return static_cast<std::int64_t>(esp_rtc_get_time_us()); }

// RTC タイマーで測った時間を、RX8130 の秒で測った時間の ±2 秒に収める（内蔵発振器のずれ対策）。
std::int64_t checked_us(std::int64_t rc_us, std::int64_t rx_s, bool have_rx)
{
    if (!have_rx) return rc_us;
    const std::int64_t lo = std::max<std::int64_t>(0, (rx_s - 2) * 1000000);
    const std::int64_t hi = std::max<std::int64_t>(0, (rx_s + 2) * 1000000);
    return std::min(std::max(rc_us, lo), hi);
}

std::int64_t timer_total_us() { return kTimerPresetsS[s_app.tm_preset % kTimerPresetCount] * 1000000LL; }

std::int64_t stopwatch_us(std::int64_t epoch)
{
    if (!s_app.sw_running) return s_app.sw_base_us;
    return s_app.sw_base_us + checked_us(rc_now_us() - s_app.sw_start_rc_us, epoch - s_app.sw_start_epoch,
                                         epoch != kNoEpoch && s_app.sw_start_epoch != kNoEpoch);
}

std::int64_t timer_left_us(std::int64_t epoch)
{
    if (!s_app.tm_running) return s_app.tm_left_us > 0 ? s_app.tm_left_us : timer_total_us();
    const std::int64_t rc_left = s_app.tm_end_rc_us - rc_now_us();
    if (rc_left <= 0) return 0;
    return checked_us(rc_left, s_app.tm_end_epoch - epoch, epoch != kNoEpoch && s_app.tm_end_epoch != kNoEpoch);
}

bool timer_due() { return s_app.tm_running && s_app.tm_end_rc_us - rc_now_us() <= kAlarmEarlyUs; }

void stopwatch_toggle(std::int64_t epoch)
{
    if (s_app.sw_running) {
        s_app.sw_base_us = stopwatch_us(epoch);
        s_app.sw_running = false;
    } else {
        s_app.sw_start_rc_us = rc_now_us();
        s_app.sw_start_epoch = epoch;
        s_app.sw_running     = true;
    }
}

void stopwatch_reset()
{
    s_app.sw_running = false;
    s_app.sw_base_us = 0;
}

void timer_toggle(std::int64_t epoch)
{
    if (s_app.tm_running) {
        s_app.tm_left_us = std::max<std::int64_t>(1, timer_left_us(epoch));
        s_app.tm_running = false;
    } else {
        const std::int64_t left = timer_left_us(epoch);
        s_app.tm_end_rc_us      = rc_now_us() + left;
        s_app.tm_end_epoch      = epoch == kNoEpoch ? kNoEpoch : epoch + (left + 500000) / 1000000;
        s_app.tm_running        = true;
    }
}

void timer_reset()
{
    s_app.tm_running = false;
    s_app.tm_left_us = 0;
    s_app.tm_alarm   = false;
}

void timer_start_alarm()
{
    s_app.tm_running = false;
    s_app.tm_left_us = 0;
    s_app.tm_alarm   = true;
    s_app.screen     = static_cast<int>(Screen::timer);
}

// 点けたときに出す画面。止まっているアプリをしばらく使っていなければ時計に戻す。
Screen screen_for_show(std::int64_t epoch)
{
    if (s_app.tm_alarm) return Screen::timer;
    Screen screen = static_cast<Screen>(s_app.screen % kScreenCount);
    const bool running = (screen == Screen::stopwatch && s_app.sw_running) || (screen == Screen::timer && s_app.tm_running);
    if (screen != Screen::clock && !running && epoch != kNoEpoch && epoch - s_app.last_use_epoch > kAppIdleBackS) {
        screen = Screen::clock;
    }
    s_app.screen = static_cast<int>(screen);
    return screen;
}

// ---------------------------------------------------------------------------
// 文字盤
// ---------------------------------------------------------------------------
struct FaceData {
    bool time_valid     = false;
    std::int64_t epoch  = 0;  // UTC
    int battery_percent = -1;
    std::uint16_t battery_mv = 0;
    bool charging            = false;  // USB が挿さっている（PM1 からは充電中かを直接読めないので、給電の有無で見る）
    bool dock                = false;  // 充電中の常時表示（秒とキャラクターを出さない）
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
    // ストップウォッチ・タイマーの画面
    char big[24]             = {};
    char small[8]            = {};
    std::uint16_t big_color  = 0;
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
void draw_time_digits(lgfx::LovyanGFX* gfx, const char* hm, int cx, int cy, DirtyRows* dirty,
                      std::uint16_t color = TFT_WHITE)
{
    gfx->loadFont(kFontTime);
    int cell = 0;
    for (char d = '0'; d <= '9'; ++d) {
        const char one[2] = {d, 0};
        cell = std::max(cell, static_cast<int>(gfx->textWidth(one)));
    }
    const int colon = gfx->textWidth(":");
    int total = 0;  // 桁数に合わせた幅（「1:05」のような 3 桁でも真ん中に来るように）
    for (int i = 0; hm[i] != 0; ++i) total += hm[i] == ':' ? colon : cell;
    // 数字は下に伸びないので、ベースラインに合わせて描き、塗るのは数字の高さだけにする
    // （行全体の高さで塗ると、上の日付の下半分まで消えていた。2026-11-15 写真 IMG_0076）。
    // 数字の高さ（Inter のキャップハイトは約 0.73em。kFontTime は 120px なので 87px）。
    // fontHeight() は行の高さ（約 1.2em）なので、その 72% では大きすぎて、上に寄せた日付の下端を削っていた
    // （2026-11-15 動画 IMG_0078）。
    constexpr int cap  = 120 * 73 / 100;
    const int baseline = cy + cap / 2;
    // 塗るのはいつも「00:00」の幅以上にする（「1:00」のように短くなったとき、前の数字の端が残っていた。2026-11-15 写真 IMG_0084）
    const int clear_w = std::max(total, cell * 4 + colon);
    gfx->fillRect(cx - clear_w / 2 - 6, baseline - cap - 4, clear_w + 12, cap + 10, TFT_BLACK);
    dirty->add(baseline - cap - 4, baseline + 5);
    gfx->setTextColor(color);  // 背景は上で塗ってあるので、文字ごとの背景塗りはしない
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
// 時間帯ごとの表情（すもっぴの絵文字 40 個 + FIGHT の並び。番号は 0 から）。並びが違う画像なら順番に出す。
constexpr int kCharacterSetSize = 41;
constexpr int kFacesNight[]     = {32, 31, 15};                     // 0〜5 時: 寝ている
constexpr int kFacesMorning[]   = {4, 40, 35, 21};                  // 6〜9 時: キラキラ、FIGHT、応援、手を振る
constexpr int kFacesLunch[]     = {20};                             // 12 時: ごはん
constexpr int kFacesDay[]       = {0, 1, 18, 21, 30, 28, 34, 3, 6}; // 10〜17 時
constexpr int kFacesEvening[]   = {19, 38, 39, 6, 1};               // 18〜20 時: ハート
constexpr int kFacesLate[]      = {25, 31, 33, 32};                 // 21〜23 時: ため息、ごろごろ

template <std::size_t N>
int pick_from(const int (&faces)[N], int current)
{
    std::size_t i = esp_random() % N;
    if (N > 1 && faces[i] == current) i = (i + 1) % N;  // 同じ表情が続かないように
    return faces[i];
}

// 点けたときの表情を時間帯で選ぶ。
int character_for_hour(int hour, int current)
{
    const CharacterSet c = character_set();
    if (c.count != kCharacterSetSize) return c.count > 0 ? (current + 1) % c.count : 0;
    if (hour < 6) return pick_from(kFacesNight, current);
    if (hour < 10) return pick_from(kFacesMorning, current);
    if (hour == 12) return pick_from(kFacesLunch, current);
    if (hour < 18) return pick_from(kFacesDay, current);
    if (hour < 21) return pick_from(kFacesEvening, current);
    return pick_from(kFacesLate, current);
}

// タップしたときは、全部の中からほかの表情にする。
int character_any(int current)
{
    const CharacterSet c = character_set();
    if (c.count <= 1) return 0;
    return (current + 1 + static_cast<int>(esp_random() % (c.count - 1))) % c.count;
}

constexpr int kCharacterTopLimit = 262;  // これより上（秒の数字）には描かない
constexpr int kCharacterMaxRise  = 18;   // 跳ねたときに上がる最大の量

// キャラクターを下に描く（背景の黒と混ぜ済みの RGB565 をそのまま貼る）。offset_y が負なら上にずらす。
void draw_character(lgfx::LovyanGFX* gfx, int index, DirtyRows* dirty, int offset_y = 0)
{
    const CharacterSet c = character_set();
    if (c.count == 0) return;
    index %= c.count;
    const std::uint16_t* frame = c.frames + static_cast<std::size_t>(index) * c.w * c.h;
    const int band_top         = kCharacterTopLimit;
    const int band_bottom      = kCharacterY + c.h + 4;
    gfx->setClipRect(0, band_top, gfx->width(), band_bottom - band_top);
    gfx->fillRect(kCenterX - c.w / 2 - 2, band_top, c.w + 4, band_bottom - band_top, TFT_BLACK);
    gfx->pushImage(kCenterX - c.w / 2, kCharacterY + offset_y, c.w, c.h, reinterpret_cast<const lgfx::rgb565_t*>(frame));
    gfx->clearClipRect();
    dirty->add(band_top, band_bottom - 1);
}

// ゆっくり上下にゆれる（±3px、1.8 秒周期）。タップした直後は 0.36 秒で 1 回跳ねる。
int character_offset(std::int64_t t_ms, std::int64_t jump_from_ms)
{
    int offset = static_cast<int>(std::lround(3.0 * std::sin(2.0 * M_PI * static_cast<double>(t_ms % 1800) / 1800.0)));
    const std::int64_t j = t_ms - jump_from_ms;
    if (jump_from_ms > 0 && j >= 0 && j < 360) {
        offset -= static_cast<int>(std::lround(kCharacterMaxRise * std::sin(M_PI * static_cast<double>(j) / 360.0)));
    }
    return offset;
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

    // 電池（上、iPhone 風）。塗りつぶした電池の中に数字と（充電中は）稲妻を入れる。表示中は変えないので最初の 1 回だけ描く。
    // 残量の色は、充電中は緑、20% 以下は赤、それ以外は白。中の数字と稲妻は白字。ただし白い残量の上だけは読めるように黒字にする
    // （2026-11-18 iOS 26 の表示に寄せ、緑の上も白字が良いとの要望で調整）。
    if (full && data.battery_percent >= 0) {
        const int pct = std::min(100, std::max(0, data.battery_percent));
        constexpr int kW = 74, kH = 32, kR = 10, kY = kBatteryY;  // 電池の外形
        constexpr int kNubW = 4, kNubH = 12, kBoltW = 11, kBoltGap = 2;
        constexpr std::uint16_t kBody = 0x4A69;  // 空の部分と端子の灰色
        const bool low            = pct <= 20;
        const std::uint16_t level = data.charging ? 0x662C : (low ? 0xF8A6 : TFT_WHITE);
        char text[8] = {};
        std::snprintf(text, sizeof(text), "%d", pct);
        gfx->loadFont(kFontBatteryPct);
        const int text_w    = gfx->textWidth(text);
        const int content_w = text_w + (data.charging ? kBoltGap + kBoltW : 0);
        const int x         = kCenterX - (kW + 2 + kNubW) / 2;
        const int cx        = x + (kW - content_w) / 2;
        const int fill_w    = std::max(pct > 0 ? 3 : 0, kW * pct / 100);
        gfx->fillRoundRect(x, kY, kW, kH, kR, kBody);
        gfx->fillRoundRect(x + kW + 2, kY + (kH - kNubH) / 2, kNubW, kNubH, 2, kBody);  // 端子
        // 数字（Inter 26px）は基準線から上へ 19px。電池の高さの中央に来るよう基準線で置く（中央揃えの指定だと下がりすぎた）。
        constexpr int kDigitH = 19;
        const int top         = kY + (kH - kDigitH) / 2;
        gfx->setTextDatum(lgfx::textdatum_t::baseline_left);
        auto draw_content = [&](std::uint16_t color) {
            gfx->setTextColor(color);
            gfx->drawString(text, cx, top + kDigitH);
            if (data.charging) {
                // 稲妻（2 つの三角形）
                const int bx = cx + text_w + kBoltGap, t = top;  // 数字と同じ高さ
                gfx->fillTriangle(bx + 7, t, bx, t + 11, bx + 6, t + 11, color);
                gfx->fillTriangle(bx + 5, t + 8, bx + kBoltW, t + 8, bx + 3, t + kDigitH, color);
            }
        };
        if (fill_w > 0) {
            gfx->setClipRect(x, kY, fill_w, kH);
            gfx->fillRoundRect(x, kY, kW, kH, kR, level);
            draw_content(level == TFT_WHITE ? TFT_BLACK : TFT_WHITE);  // 白い残量の上だけ黒字にする
        }
        if (fill_w < kW) {
            gfx->setClipRect(x + fill_w, kY, kW - fill_w, kH);
            draw_content(TFT_WHITE);
        }
        gfx->clearClipRect();
    }

#if WATCH_HAS_CHARACTER
    if (full && !data.dock) draw_character(gfx, s_character_index, dirty);
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
    if (!data.dock && (full || std::strcmp(sec, state->sec) != 0)) {
        gfx->loadFont(kFontSec);
        gfx->setTextColor(kColorAccent);
        gfx->setTextDatum(lgfx::textdatum_t::middle_center);
        draw_text_box(gfx, sec, "88", kCenterX, kSecY, dirty);
        std::strcpy(state->sec, sec);
    }
    gfx->unloadFont();
}

// ストップウォッチ・タイマーの画面。上に名前、真ん中に大きく時間、その下に 1/100 秒（または秒）、下に操作の説明。
void draw_app(lgfx::LovyanGFX* gfx, Screen screen, std::int64_t us, bool running, bool alarm, bool alarm_phase,
              FaceState* st, DirtyRows* dirty)
{
    const bool full = !st->drawn;
    const bool sw   = screen == Screen::stopwatch;
    if (full) {
        gfx->fillScreen(TFT_BLACK);
        st->drawn = true;
        dirty->add(0, gfx->height() - 1);
        gfx->loadFont(kFontSmall);
        gfx->setTextDatum(lgfx::textdatum_t::middle_center);
        gfx->setTextColor(kColorAccent);
        gfx->drawString(sw ? "STOPWATCH" : (alarm ? "DONE" : "TIMER"), kCenterX, kDateY);
        gfx->loadFont(kFontHint);
        gfx->setTextColor(kColorDim);
        if (alarm) {
            gfx->drawString("PRESS ANY BUTTON", kCenterX, 335);
        } else if (sw) {
            gfx->drawString("B  START / STOP", kCenterX, 318);
            gfx->drawString("HOLD B  RESET", kCenterX, 352);
            gfx->drawString("A  NEXT", kCenterX, 386);
        } else {
            gfx->drawString("TAP  CHANGE TIME", kCenterX, 318);
            gfx->drawString("B  START / STOP", kCenterX, 352);
            gfx->drawString("HOLD B  RESET   A  NEXT", kCenterX, 386);
        }
    }
    // タイマーは切り上げて表示する（残り 0.4 秒は 00:01）
    const std::int64_t shown = sw ? us : (us + 999999) / 1000000 * 1000000;
    const long long s        = std::min<long long>(shown / 1000000, 99 * 3600 + 3599);  // 99:59 まで
    char big[24]             = {};
    char small[8]            = {};
    if (s >= 6000) {  // 100 分からは「時:分」と「:秒」にする（60 分のタイマーは 60:00 と出す）
        std::snprintf(big, sizeof(big), "%lld:%02lld", s / 3600, (s / 60) % 60);
        std::snprintf(small, sizeof(small), ":%02lld", s % 60);
    } else {
        std::snprintf(big, sizeof(big), "%02lld:%02lld", s / 60, s % 60);
        if (sw) std::snprintf(small, sizeof(small), ".%02lld", static_cast<long long>((us / 10000) % 100));
    }
    const std::uint16_t color = alarm ? (alarm_phase ? kColorAccent : TFT_WHITE) : (running ? TFT_WHITE : kColorDate);
    if (full || std::strcmp(big, st->big) != 0 || color != st->big_color) {
        draw_time_digits(gfx, big, kCenterX, kTimeY, dirty, color);
        std::strcpy(st->big, big);
        st->big_color = color;
    }
    if (full || std::strcmp(small, st->small) != 0) {
        gfx->loadFont(kFontSec);
        gfx->setTextColor(kColorAccent);
        gfx->setTextDatum(lgfx::textdatum_t::middle_center);
        draw_text_box(gfx, small, ":88", kCenterX, kSecY, dirty);
        std::strcpy(st->small, small);
    }
    gfx->unloadFont();
}

enum class ShowMode { look, button, cold_boot, dock };

// 夜（手首の動きで起きない時間帯。日本時間）か
bool is_night(std::int64_t epoch)
{
    const int hour = static_cast<int>(((epoch + kJstOffsetSec) / 3600) % 24);
    return hour >= kNightFromHour && hour < kNightToHour;
}

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
    volatile bool interacted            = false;  // 表示中にボタンやタップで操作した（腕を下ろしても消さない）
    bool started                        = false;
    SemaphoreHandle_t done              = nullptr;
};
DisplayJob g_job;

void display_task(void*)
{
    DisplayJob& job              = g_job;
    pm_apply(kCpuMhzDraw, false);  // 画面の電源が入っている間は浅く眠らない（ピンと転送を乱さないため）
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
    data.charging = s_usb_power;
    const std::int64_t t_data = now_ms();
    if (begun != ESP_OK) ESP_LOGE(kTag, "DISPLAY_FAIL begin error=%s", esp_err_to_name(begun));

    lgfx::LovyanGFX* gfx = panel.canvas();
    FaceData frame         = data;
    FaceState face_state;
    auto epoch_now = [&]() -> std::int64_t {
        return data.time_valid ? data.epoch + (now_ms() - epoch_at_ms) / 1000 : kNoEpoch;
    };
#if WATCH_HAS_CHARACTER
    if (begun == ESP_OK) {
        const int hour = data.time_valid ? utc_from_epoch(epoch_now() + kJstOffsetSec).tm_hour : 12;
        s_character_index = character_for_hour(hour, s_character_index);
    }
#endif
    // 見ていると決まる前に、時計の文字盤を書いて送っておく（明るさ 0 のままなので見えない）。
    // 決まったら、変わった行だけ送り直して光らせる。判定を待つ間に描画と転送（約 70ms）を済ませる。
    bool predrawn = false;
    if (begun == ESP_OK && job.command == DisplayCommand::none && !timer_due() && !s_app.tm_alarm &&
        screen_for_show(epoch_now()) == Screen::clock) {
        DirtyRows pre;
        if (frame.time_valid) frame.epoch = epoch_now();
        draw_face(gfx, frame, &face_state, &pre);
        predrawn = panel.push_rows(pre.y0, pre.y1) == ESP_OK;
        if (!predrawn) face_state = FaceState{};
    }
    const std::int64_t t_pre = now_ms();

    while (job.command == DisplayCommand::none) vTaskDelay(pdMS_TO_TICKS(5));

    if (begun == ESP_OK && job.command == DisplayCommand::show) {
        const ShowMode mode = job.mode;
        DirtyRows dirty;
        if (timer_due()) timer_start_alarm();
        Screen screen              = screen_for_show(epoch_now());
        if (predrawn && screen != Screen::clock) face_state = FaceState{};  // 別の画面になったら全体を描き直す
        bool dock = mode == ShowMode::dock;
        if (dock) {
            frame.dock = true;
            face_state = FaceState{};  // 予描きは普段の文字盤なので描き直す
        }
#if WATCH_HAS_CHARACTER
        std::int64_t jump_from_ms = 0;
        int char_offset           = 0;
#endif
        std::int64_t alarm_from_ms = now_ms();
        bool alarm_phase           = true;
        auto draw_current          = [&](DirtyRows* d) {
            if (data.time_valid) frame.epoch = epoch_now();
            if (screen == Screen::clock) {
                draw_face(gfx, frame, &face_state, d);
            } else if (screen == Screen::stopwatch) {
                draw_app(gfx, screen, stopwatch_us(epoch_now()), s_app.sw_running, false, false, &face_state, d);
            } else {
                draw_app(gfx, screen, s_app.tm_alarm ? 0 : timer_left_us(epoch_now()), s_app.tm_running, s_app.tm_alarm,
                         alarm_phase, &face_state, d);
            }
        };
        const std::int64_t t_draw0 = now_ms();
        // 予描きした文字盤があれば、描き直さずにすぐ光らせる（秒が 1 つ古くても、次のフレームで直る）。
        if (!(predrawn && screen == Screen::clock)) draw_current(&dirty);
        const std::int64_t t_draw1 = now_ms();
        const esp_err_t pushed = dirty.empty() ? ESP_OK : panel.push_rows(dirty.y0, dirty.y1);
        const std::int64_t t_push = now_ms();
        (void)panel.set_brightness_percent(dock ? kDockBrightness : kBrightness);  // 時計を書いてから光らせる
        s_lit_ms       = now_ms();
        s_lit_predrawn = predrawn;
        pm_apply(kCpuMhzIdle, false);
        ESP_LOGI(kTag, "FRAME_TIMING predrawn=%d pre_end=%lld cmd_wait_end=%lld draw=%lldms push=%lldms", predrawn,
                 t_pre, t_draw0, t_draw1 - t_draw0, t_push - t_draw1);
        if (pushed != ESP_OK) ESP_LOGW(kTag, "PUSH_FAIL error=%s", esp_err_to_name(pushed));
        const std::int64_t first_frame_ms = now_ms();
        ESP_LOGI(kTag, "SHOW %s wake->frame %lldms (display start=%lld ready=%lld data=%lld) time=%s bat=%umV(%d%%)",
                 mode == ShowMode::look ? "look" : (mode == ShowMode::button ? "button" : (dock ? "dock" : "boot")),
                 first_frame_ms - job.wake_ms, t_start, t_display, t_data, data.time_valid ? "ok" : "invalid",
                 data.battery_mv, data.battery_percent);

        const std::int64_t limit_us =
            mode == ShowMode::look ? kLookShowMaxUs
                                   : (mode == ShowMode::button || dock ? kButtonShowUs : kColdBootShowUs);
        std::int64_t deadline_us = esp_timer_get_time() + (s_app.tm_alarm ? kAlarmUs : limit_us);
        const char* reason       = "timeout";
        // キーは押した瞬間・離した瞬間・長押しを見る。点けたときに押されていたキー（起こしたキー）は、離すまで無視する。
        const gpio_num_t key_a = m5sw::board::kKey1;  // KEYA（GPIO2）
        const gpio_num_t key_b = m5sw::board::kKey2;  // KEYB（GPIO1）
        bool a_was             = gpio_get_level(key_a) == 0;
        bool b_was             = gpio_get_level(key_b) == 0;
        bool a_ignore          = a_was;
        bool b_long_done       = b_was;  // true の間は、離しても短押しとして扱わない
        std::int64_t b_down_ms = 0;
        bool vibrating         = false;
        // 表示中にタップしたら、時計ではキャラクターの表情、タイマーでは時間を変える
        // （1 回のタップで割り込みが何度か来るので 400ms は無視）。
        s_touch_edges            = 0;
        int touch_irqs           = 0;
        std::int64_t next_tap_ms = 0;
        // 充電中の常時表示。普段の表示が終わる時点で USB が挿さっていて夜でなければ、消さずに暗い画面へ移る。
        // タップやボタンで普段の明るさに戻し、操作が終わればまた暗い画面に戻る。
        std::int64_t dock_from_ms  = 0;
        std::int64_t dock_total_ms = 0;
        std::int64_t next_check_ms = 0;
        std::int64_t next_bat_ms   = 0;
        auto dock_ok = [&]() { return s_usb_power && data.time_valid && !is_night(epoch_now()); };
        auto enter_dock = [&]() {
            dock          = true;
            dock_from_ms  = now_ms();
            next_check_ms = now_ms() + kDockCheckMs;
            next_bat_ms   = now_ms() + kDockBatteryMs;
            if (screen != Screen::clock && screen_for_show(epoch_now()) == Screen::clock) screen = Screen::clock;
            frame.dock = true;
            face_state = FaceState{};
            ESP_LOGI(kTag, "DOCK on");
        };
        auto leave_dock = [&]() {
            if (!dock) return;
            dock = false;
            dock_total_ms += now_ms() - dock_from_ms;
            frame.dock = false;
            face_state = FaceState{};
            (void)panel.set_brightness_percent(kBrightness);
        };
        if (dock) {
            dock          = false;
            dock_from_ms  = now_ms();
            enter_dock();
        }
        bool dock_lit = false;  // 暗い画面を描いてから明るさを下げる（描く前に下げると古い画面が暗く見える）
        auto interacted          = [&]() {
            leave_dock();
            deadline_us = std::max(deadline_us, esp_timer_get_time() + kButtonShowUs);
            job.interacted       = true;
            s_app.last_use_epoch = epoch_now();
        };
        auto go_screen = [&](Screen next) {
            screen       = next;
            s_app.screen = static_cast<int>(next);
            face_state   = FaceState{};  // 全体を描き直す
        };
        auto stop_alarm = [&]() {
            s_app.tm_alarm = false;
            face_state     = FaceState{};
            if (vibrating) {
                (void)watch::set_vibration(job.bus, false);
                vibrating = false;
            }
        };
        auto start_alarm = [&]() {
            leave_dock();
            timer_start_alarm();
            go_screen(Screen::timer);
            alarm_from_ms = now_ms();
            deadline_us   = std::max(deadline_us, esp_timer_get_time() + kAlarmUs);
            job.interacted = true;
            ESP_LOGI(kTag, "TIMER_ALARM");
        };
        if (s_app.tm_alarm) ESP_LOGI(kTag, "TIMER_ALARM (on wake)");
        while (true) {
            if (!dock) {
                const bool lowered = job.stop && !job.interacted && !s_app.tm_alarm;
                if (lowered || esp_timer_get_time() >= deadline_us) {
                    if (!s_app.tm_alarm && dock_ok()) {
                        enter_dock();
                        dock_lit = false;
                    } else {
                        reason = lowered ? "lowered" : "timeout";
                        break;
                    }
                }
            } else if (now_ms() >= next_check_ms) {
                next_check_ms     = now_ms() + kDockCheckMs;
                std::uint16_t vin = 0;
                if (job.power->read_vin_mv(&vin).ok() && vin < kUsbVinMv) {
                    s_usb_power = false;
                    reason      = "unplugged";
                    break;
                }
                if (is_night(epoch_now())) {
                    reason = "night";
                    break;
                }
                if (now_ms() >= next_bat_ms) {
                    next_bat_ms      = now_ms() + kDockBatteryMs;
                    std::uint16_t mv = 0;
                    if (job.power->read_battery_mv(&mv).ok() && mv > 0 &&
                        battery_percent_from_mv(mv) != frame.battery_percent) {
                        frame.battery_mv      = mv;
                        frame.battery_percent = battery_percent_from_mv(mv);
                        face_state            = FaceState{};  // 電池は全体を描くときだけ描くので描き直す
                    }
                }
            }
            if (s_app.tm_running && timer_left_us(epoch_now()) == 0) start_alarm();
            if (s_app.tm_alarm) {
                // 0.4 秒ごとに振動と色を切り替える
                alarm_phase = ((now_ms() - alarm_from_ms) / 400) % 2 == 0;
                if (alarm_phase != vibrating) {
                    (void)watch::set_vibration(job.bus, alarm_phase);
                    vibrating = alarm_phase;
                }
            }
            {
                DirtyRows changed;
#if WATCH_HAS_CHARACTER
                if (!face_state.drawn) char_offset = 0;  // 全体を描き直すときは、ずらさずに描かれる
#endif
                draw_current(&changed);
#if WATCH_HAS_CHARACTER
                if (screen == Screen::clock && !dock) {
                    const int offset = character_offset(now_ms(), jump_from_ms);
                    if (offset != char_offset) {
                        draw_character(gfx, s_character_index, &changed, offset);
                        char_offset = offset;
                    }
                }
#endif
                if (!changed.empty()) {
                    const esp_err_t e = panel.push_rows(changed.y0, changed.y1);
                    if (e != ESP_OK) ESP_LOGW(kTag, "PUSH_FAIL error=%s", esp_err_to_name(e));
                }
                if (dock && !dock_lit) {
                    (void)panel.set_brightness_percent(kDockBrightness);
                    dock_lit = true;
                }
            }
            if (s_touch_edges != 0) {
                touch_irqs += s_touch_edges;
                s_touch_edges = 0;
                if (now_ms() >= next_tap_ms) {
                    next_tap_ms = now_ms() + 400;
                    interacted();
                    if (s_app.tm_alarm) {
                        stop_alarm();
                    } else if (screen == Screen::timer) {
                        if (!s_app.tm_running) {
                            s_app.tm_preset  = (s_app.tm_preset + 1) % kTimerPresetCount;
                            s_app.tm_left_us = 0;
                        }
                    } else if (screen == Screen::clock) {
#if WATCH_HAS_CHARACTER
                        s_character_index = character_any(s_character_index);
                        jump_from_ms      = now_ms();
                        char_offset       = INT32_MIN;  // 描き直す
                        if (kTapSound) watch::audio_play(job.bus, watch::Sound::tap);
#endif
                    }
                }
            }
            const bool a = gpio_get_level(key_a) == 0;
            const bool b = gpio_get_level(key_b) == 0;
            if (a != a_was || b != b_was) ESP_LOGI(kTag, "KEY a=%d b=%d screen=%d", a, b, static_cast<int>(screen));
            // KEYA: 次の画面
            if (a && !a_was && !a_ignore) {
                interacted();
                if (s_app.tm_alarm) {
                    stop_alarm();
                } else {
                    go_screen(static_cast<Screen>((static_cast<int>(screen) + 1) % kScreenCount));
                }
            }
            if (!a) a_ignore = false;
            // KEYB: 短く押す = スタート／ストップ、長押し = リセット
            if (b && !b_was) {
                b_down_ms   = now_ms();
                b_long_done = false;
            }
            if (b && !b_long_done && now_ms() - b_down_ms >= kLongPressMs) {
                b_long_done = true;
                interacted();
                if (s_app.tm_alarm) {
                    stop_alarm();
                } else if (screen == Screen::stopwatch) {
                    stopwatch_reset();
                } else if (screen == Screen::timer) {
                    timer_reset();
                }
            }
            if (!b && b_was && !b_long_done) {
                interacted();
                if (s_app.tm_alarm) {
                    stop_alarm();
                } else if (screen == Screen::stopwatch) {
                    stopwatch_toggle(epoch_now());
                } else if (screen == Screen::timer) {
                    timer_toggle(epoch_now());
                } else {
#if WATCH_HAS_CHARACTER
                    s_character_index = character_any(s_character_index);
                    jump_from_ms      = now_ms();
                    char_offset       = INT32_MIN;
                    if (kTapSound) watch::audio_play(job.bus, watch::Sound::tap);
#endif
                }
            }
            a_was = a;
            b_was = b;
            vTaskDelay(pdMS_TO_TICKS(kFrameMs));
        }
        if (vibrating) (void)watch::set_vibration(job.bus, false);
        watch::audio_shutdown();
        if (s_app.tm_alarm) s_app.tm_alarm = false;  // 鳴り終わった
        if (dock) dock_total_ms += now_ms() - dock_from_ms;
        const std::int64_t shown_ms = now_ms() - first_frame_ms;
        s_shown_ms += shown_ms - dock_total_ms;  // 充電中の常時表示は電池の内訳に数えない
        if (dock_total_ms > 0) ESP_LOGI(kTag, "DOCK total %lldms", dock_total_ms);
        const esp_err_t off         = panel.sleep();
        ESP_LOGI(kTag, "HIDE after %lldms (%s) power_off=%s touch_irq_samples=%d", shown_ms, reason,
                 esp_err_to_name(off), touch_irqs);
    } else {
        // 中止、または初期化に失敗したとき。電源は必ず切る。
        const esp_err_t off = panel.sleep();
        ESP_LOGI(kTag, "PREP_ABORT power_off=%s", esp_err_to_name(off));
    }
    pm_apply(kCpuMhzIdle, true);
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
    // もう一度 display_start できるようにする（空振りのあと、充電中の常時表示を出すとき）
    g_job.started  = false;
    g_job.command  = DisplayCommand::none;
    g_job.stop     = false;
    vSemaphoreDelete(g_job.done);
    g_job.done = nullptr;
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
        if (mode != ShowMode::look || g_job.stop || g_job.interacted) continue;
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
    bool false_wake                 = false;
    // 寝ている間〜ここまでのタップの記録（GPIO13 を設定し直す前に読む）
    const bool touch_latched = m5sw::power::PowerController::take_touch_latch();
    const bool quick_armed   = m5sw::power::PowerController::quick_wake_armed();
    mark("app");
    m5sw::test_support::TestContext context = {};
    ESP_ERROR_CHECK(m5sw::test_support::initialize(&context));
    mark("init");
    apply_log_levels(false);  // USB かどうかは PM1 を読むまで分からない
    keep_pins_in_light_sleep();

    const esp_sleep_wakeup_cause_t cause = context.wake_cause;
    // 起きた直後の PM1 IRQ 線（GPIO12）の状態。寝ている間は Low で起きる設定。
    const int pm1_irq_level = rtc_gpio_get_level(m5sw::board::kPm1Irq);
    const bool cold_boot = cause != ESP_SLEEP_WAKEUP_EXT0 && cause != ESP_SLEEP_WAKEUP_EXT1 &&
                           cause != ESP_SLEEP_WAKEUP_TIMER;
    // 前回寝てから起きるまでの時間（RTC の時計。コールドブートや記録がないときは分からない）。
    const bool slept_known = !cold_boot && s_wlog_magic == kWakeLogMagic && rc_now_us() >= s_sleep_start_rc_us;
    const std::int64_t slept_ms = slept_known ? (rc_now_us() - s_sleep_start_rc_us) / 1000 : -1;
    keys_as_inputs();
    watch_keys_while_awake();
    // ボタンで起きたかの手がかり（ext1 の状態と、いま押されているか）。ext0（手首）と同時だと原因は ext0 と報告される。
    const std::uint64_t ext1_status = esp_sleep_get_ext1_wakeup_status();
    const bool key_at_boot          = any_key_down();
    mark("keys");

    const esp_err_t i2c_result = m5sw::board::initialize_i2c();
    if (i2c_result != ESP_OK) fail("i2c", i2c_result, i2c_result);
    i2c_bus_handle_t bus         = nullptr;
    const esp_err_t claim_result = m5sw::board::acquire_i2c(&bus);
    if (claim_result != ESP_OK) fail("shared_bus", claim_result, claim_result);
    mark("i2c");

    static m5sw::wakeup::RtcWakeup rtc;
    auto rtc_result = rtc.begin(bus);
    if (!rtc_result.ok()) fail("rtc_begin", rtc_result.error, rtc_result.raw_error);
    mark("rtc");
    m5sw::wakeup::RtcFlags retained_rtc;
    rtc_result = rtc.read_flags(&retained_rtc);
    if (!rtc_result.ok()) fail("rtc_flags", rtc_result.error, rtc_result.raw_error);

    m5sw::wakeup::ImuStatus retained_imu;
    auto imu_result = m5sw::wakeup::ImuWakeup::read_retained_status(bus, &retained_imu);
    if (!imu_result.ok()) fail("imu_retained", imu_result.error, imu_result.raw_error);
    mark("rtc_imu");

    // BMI270 を初期化するのはコールドブートのときだけ（any-motion と FIFO の設定は寝ている間も残る）。
    // PM1 の初期化は 1 回だけ（電源側で初期化し、起床判定側はそれを借りる）。
    wait_pm1_awake(bus);
    mark("pwake");
    static m5sw::power::PowerController power;
    auto power_result = with_retry([&] { return power.begin(bus); });
    if (!power_result.ok()) fail("pm1_begin", power_result.error, power_result.raw_error);
    mark("pbegin");

    static m5sw::wakeup::WakeController wake;
    auto wake_result = wake.begin(bus, cold_boot, &power.pm1_adapter());
    if (!wake_result.ok()) fail("wake_begin", wake_result.error, wake_result.raw_error);
    mark("wbegin");
    m5sw::wakeup::WakeSnapshot snapshot;
    wake_result = wake.capture(retained_rtc, retained_imu, &snapshot);
    if (!wake_result.ok()) fail("wake_capture", wake_result.error, wake_result.raw_error);
    mark("wake");
    power.set_touch_wake(true);  // 画面をタップしても起きる
    {
        std::uint16_t vin = 0;
        s_usb_power       = power.read_vin_mv(&vin).ok() && vin >= kUsbVinMv;
        apply_log_levels(s_usb_power);
        pm_apply(kCpuMhzIdle, true);
    }
    mark("pm1");

    static ImuFifo fifo(bus);
    g_job.bus     = bus;
    g_job.rtc     = &rtc;
    g_job.power   = &power;
    g_job.wake_ms = app_start_ms;

    if (cold_boot) {
        s_wake_count = s_show_count = 0;
        s_awake_ms                  = 0;
        s_false_count               = 0;
        s_false_awake_ms = s_shown_ms = 0;
        s_next_record_epoch         = 0;
        s_app                       = AppState{};  // 書き込み直後などは RTC メモリに前の値が残っていることがある
        ESP_LOGI(kTag, "COLD_BOOT cause=%d reset_reason=%d", static_cast<int>(cause), static_cast<int>(esp_reset_reason()));
        {
            const esp_err_t mux = watch::hold_usb_mux(bus);
            if (mux != ESP_OK) ESP_LOGW(kTag, "USB_MUX hold error=%s", esp_err_to_name(mux));
        }
        (void)sync_time_from_usb(&rtc);
        show_watch(&fifo, ShowMode::cold_boot);
        maybe_wifi_time_sync(&rtc, true);  // コールドブートでは毎回試す（表示の後。Wi-Fi の動作確認を兼ねる）
    } else if (timer_due()) {
        // タイマーの時間になった（寝る前に、終わる時刻でタイマー起床を仕掛けてある）
        ++s_wake_count;
        ++s_show_count;
        timer_start_alarm();
        ESP_LOGI(kTag, "WAKE #%lu timer_alarm cause=%d", static_cast<unsigned long>(s_wake_count), static_cast<int>(cause));
        show_watch(&fifo, ShowMode::button);
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
            ++s_false_count;
            false_wake = true;
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

    // 充電中で画面を出さなかった起床（空振り・1 時間ごとの見回り）でも、夜でなければ常時表示に入る。
    if (s_usb_power && !g_shown && !cold_boot) {
        std::tm utc = {};
        if (with_retry([&] { return rtc.read_time(&utc); }).ok() && !is_night(epoch_from_utc(utc))) {
            ESP_LOGI(kTag, "DOCK start (usb)");
            show_watch(&fifo, ShowMode::dock);
        }
    }
    mark("decided");
    wait_pm1_awake(bus);  // 判定や表示の間に PM1 が I2C の眠りに入っている（50ms のやり直し待ちを避ける）
    wake_result = with_retry([&] { return wake.clear_snapshot(snapshot); });
    if (!wake_result.ok()) fail("wake_clear", wake_result.error, wake_result.raw_error);
    mark("wclr");
    rtc_result = with_retry([&] { return rtc.clear_flags(retained_rtc); });
    if (!rtc_result.ok()) fail("rtc_clear", rtc_result.error, rtc_result.raw_error);
    mark("rclr");
    rtc_result = with_retry([&] { return rtc.disable_interrupts(); });
    if (!rtc_result.ok()) fail("rtc_disable", rtc_result.error, rtc_result.raw_error);

    const bool tap_wake = !cold_boot && (touch_latched || (ext1_status & (1ULL << m5sw::board::kTouchIrq)) != 0);
    // 1 時間ごとの見回りのときだけ、ボードを設定し直す（タップやタイマーのアラームで起きたときはしない）。
    // 寝てすぐのタイマー起床（原因を調べ中。2026-11-17 の夜に 4.5 秒ごとに起き続けた）では、
    // 3.4 秒かかる設定し直しをしない（それ自体が次の起床を呼んでいる可能性がある）。
    const bool short_timer_wake    = slept_ms >= 0 && slept_ms < kReinitMinSleepMs;
    const bool fallback_timer_wake = cause == ESP_SLEEP_WAKEUP_TIMER && !tap_wake && !g_shown && !short_timer_wake;
    if (cold_boot || fallback_timer_wake) {
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

    mark("cleared");
    const std::int64_t now_epoch = hourly_tasks(&rtc, &power);
    mark("hourly");
    if (usb_serial_jtag_is_connected()) dump_battery_history();
    wait_keys_released();
    // 夜は手首の動きで起きない（寝返りなどで何度も起きるのを防ぐ）。朝になったらタイマーで起きて設定し直す。
    bool night                   = false;
    std::int64_t until_morning_s = 0;
    if (now_epoch != 0) {
        const std::int64_t jst = now_epoch + kJstOffsetSec;
        const int hour         = static_cast<int>((jst / 3600) % 24);
        night                  = hour >= kNightFromHour && hour < kNightToHour;
        until_morning_s        = static_cast<std::int64_t>(kNightToHour) * 3600 - jst % 86400;
    }
    mark("sleep");
    print_marks();
    ESP_LOGI(kTag, "SLEEP at %lldms usb=%d night=%d", now_ms(), s_usb_power, night);
    if (s_usb_power) {
        vTaskDelay(kLogFlushWait);
    } else {
        esp_deep_sleep_disable_rom_logging();  // 起きるたびの ROM の表示を止める
    }
    if (!cold_boot) s_awake_ms += now_ms();  // 寝る前の 1 秒待ちは浅い眠りなので数えない
    {
        WakeLogEntry e = {};
        e.rc_s         = static_cast<std::uint32_t>(rc_now_us() / 1000000);
        e.slept_ms     = slept_ms >= 0 ? static_cast<std::uint32_t>(std::min<std::int64_t>(slept_ms, 0x7FFFFFFF))
                                       : 0xFFFFFFFFu;
        e.awake_ms     = static_cast<std::uint16_t>(std::min<std::int64_t>(now_ms(), 0xFFFF));
        e.cause        = cold_boot ? static_cast<std::uint8_t>(100 + esp_reset_reason()) : static_cast<std::uint8_t>(cause);
        e.flags        = (touch_latched ? kWlTouchLatch : 0) |
                  ((ext1_status & (1ULL << m5sw::board::kTouchIrq)) != 0 ? kWlExt1Touch : 0) |
                  ((ext1_status & kKeyWakeMask) != 0 ? kWlExt1Key : 0) | (pm1_irq_level == 0 ? kWlPm1IrqLow : 0) |
                  (quick_armed ? kWlQuickArmed : 0) | (fallback_timer_wake ? kWlReinit : 0) |
                  (g_shown ? kWlShown : 0) | (false_wake ? kWlFalse : 0);
        wake_log_add(e);
        s_sleep_start_rc_us = rc_now_us();
    }
    if (false_wake) s_false_awake_ms += now_ms();
    // タイマーが動いていたら、終わる時刻に起きる（寝る前に 1 秒待つ分を引く）。
    std::int64_t sleep_us = static_cast<std::int64_t>(kTimerFallbackUs);
    if (s_app.tm_running) {
        const std::int64_t left = s_app.tm_end_rc_us - rc_now_us() - 1000000;
        sleep_us                = std::min(sleep_us, std::max<std::int64_t>(1000, left));
        ESP_LOGI(kTag, "TIMER running, wake in %lldms", sleep_us / 1000);
    }
    if (night) sleep_us = std::min(sleep_us, std::max<std::int64_t>(1000000, until_morning_s * 1000000 + 5000000));
    power_result = power.enter_l2_quick(night ? m5sw::power::WakeSelection::none : m5sw::power::WakeSelection::rtc_imu, static_cast<std::uint64_t>(sleep_us),
                                        m5sw::power::L2Pm1IdleSleep::enabled, kKeyWakeMask);
    fail("l2_returned", power_result.error, power_result.raw_error);
}
