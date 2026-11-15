// 「手首を上げて時計を見たら起こす」をディープスリープ込みで試す実験用ファーム。
//
// 方式: BMI270 の any-motion（動き検出）で ESP32 を L2 ディープスリープから起こし、
// 起きた直後に加速度を最大 1.5 秒見て「時計を見ている姿勢」かを判定する。
//   見ている姿勢: 文字盤が顔の方へ傾き（x >= 0.05g）、文字盤が上向き寄りか腕が立っていて、
//                 ほぼ止まっている状態が 200ms 続く（raise_diag v2 と look_wake_l2 の測定から決めた。詳細は定数のコメント）。
//   持ち上げた動き: その姿勢になる前の 2 秒間に、向きが kRaiseMinDeg 以上変わっている。
//                 スープを飲む・キーボードを打つなど、姿勢が近いまま小さく動くだけの場合を外すため。
// 起きる前の加速度は、BMI270 の FIFO に寝ている間も溜めておき、起きたら読み出す（RTC メモリに直近 2.5 秒を保持）。
// 見ていれば 1 回振動し（将来は画面点灯に置き換える）、腕を下ろすまで起きたまま待ってから寝る。
// 寝直しは、振動の後も含めて、ボード全体の再設定を省いた素早い経路を使う（コールドブートとタイマー起床のときだけ全部設定する）。
// 起きたときは BMI270 を初期化し直さない（約 1 秒かかるため）。any-motion と FIFO の設定は寝る前のまま残っている。
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstring>

#include "esp_attr.h"
#include "esp_log.h"
#include "esp_rom_sys.h"
#include "esp_system.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "i2c_bus.h"
#include "m5sw_board.hpp"
#include "m5sw_ioe1.hpp"
#include "m5sw_power.hpp"
#include "m5sw_rtc_wakeup.hpp"
#include "m5sw_test_support.hpp"
#include "m5sw_wakeup.hpp"

namespace {
constexpr char kTag[]              = "look_wake_l2";
constexpr TickType_t kPrepareDelay = pdMS_TO_TICKS(200);  // ログを USB に流し切るための待ち
constexpr std::uint16_t kMotorHz   = 5000;
constexpr std::uint8_t kMotorPct   = 80;  // wrist_wake_l2 で体感できた値に合わせる
constexpr TickType_t kBuzzDuration = pdMS_TO_TICKS(300);

// 起こすときの any-motion: 40ms 続く、約 48mg 以上の変化。
constexpr std::uint8_t kAnyDuration  = 2;    // LSB = 20ms
constexpr std::uint8_t kAnyThreshold = 100;  // LSB = 0.48mg

// 「見ている姿勢」の判定（単位 g、センサー座標）。測定で分かった各姿勢の重力の向き:
//   胸の前で文字盤を上に向けて見る  x=+0.1〜+0.5  y=-0.5〜-0.1  z=+0.8〜+1.0
//   顔の前に腕を立てて見る          x=+0.2〜+0.35 y=+0.9        z=0
//   机・膝に手を置く / キーボード    x=-0.45〜0    y=0           z=+0.9（文字盤は上だが顔と反対側へ傾く）
//   腕を下ろす                      y=-0.9
// x（文字盤が顔の方へ傾く向き）が、見ているときと机の上とを分ける決め手になる。
// 0.05 だと、手のひらを下にして机に置いた姿勢（x=+0.08〜+0.10）でも LOOK になった（v10 のパート 2）。
// 実際に見たときの x は +0.18 以上だったので 0.15 にする。
constexpr float kLookMinX        = 0.15f;
constexpr float kLookMinZ        = 0.50f;  // 文字盤が上向き寄り、または
constexpr float kLookMinY        = 0.60f;  // 腕が立っている
constexpr float kStillTolerance  = 0.15f;
constexpr std::int64_t kWindowUs = 1500000;

// 判定は FIFO のフレーム（50Hz = 20ms ごと）を数えて行う。各フレームは直近 5 フレーム（100ms）の平均でならす。
constexpr int kSmoothFrames   = 5;
constexpr int kLookHoldFrames = 10;  // 見ている姿勢が 200ms 続いたら LOOK
constexpr int kAwayHoldFrames = 25;  // 見ていない姿勢で 500ms 止まっていたら、窓の終わりを待たずに寝直す
constexpr int kViewEndFrames  = 15;  // 見ている姿勢（ゆるい判定）から 300ms 外れたら「見終わった」
constexpr std::int64_t kViewMaxUs = 10000000;  // 見続けていても 10 秒で寝る（画面の自動消灯の代わり）

// 「持ち上げた動き」の判定: LOOK の 0.2〜2.0 秒前の向きと、LOOK のときの向きとの角度の最大値。
constexpr float kRaiseMinDeg        = 25.0f;
constexpr int kRaiseSkipFrames      = 10;   // 直前 200ms は見る姿勢に入りかけているので比べない
constexpr int kRaiseLookbackFrames  = 100;  // 2 秒
constexpr float kHistStillTolerance = 0.25f;  // 振り回し中のフレームは向きが分からないので使わない

constexpr float kLsbPerG                 = 16384.0f;  // ±2g レンジ
constexpr std::uint32_t kImuI2cHz        = 100000;    // 400kHz では FIFO の中身が化けたので、いったん 100kHz に戻す
constexpr std::uint64_t kTimerFallbackUs = 30ULL * 1000 * 1000;  // 素早い寝直しで起きられなくなったときの保険

// BMI270 の FIFO 関連レジスタ。
constexpr std::uint8_t kRegFifoLength  = 0x24;  // 2 バイト、下位 14 ビットがバイト数
constexpr std::uint8_t kRegFifoData    = 0x26;
constexpr std::uint8_t kRegFifoConfig0 = 0x48;  // bit0 stop_on_full, bit1 time_en
constexpr std::uint8_t kRegFifoConfig1 = 0x49;  // bit4 header_en, bit6 acc_en
constexpr std::uint8_t kRegPwrConf     = 0x7C;  // bit0 adv_power_save, bit1 fifo_self_wake_up
constexpr std::uint8_t kRegCmd         = 0x7E;
constexpr std::uint8_t kCmdFifoFlush   = 0xB0;
constexpr std::size_t kFifoBytes       = 2048;
constexpr std::size_t kFrameBytes      = 6;  // ヘッダなしモードの加速度フレーム（x, y, z の 16bit リトルエンディアン）

// ディープスリープをまたいで残る値（電源ボタンでのリセットでは 0 に戻る）。
RTC_DATA_ATTR std::uint32_t s_wake_count    = 0;
RTC_DATA_ATTR std::uint32_t s_look_count    = 0;
RTC_DATA_ATTR std::uint32_t s_reject_count  = 0;  // 見ていなかったので寝直した回数
RTC_DATA_ATTR std::uint32_t s_noraise_count = 0;  // 姿勢は合っていたが、持ち上げた動きがなかったので寝直した回数
RTC_DATA_ATTR std::uint64_t s_awake_ms      = 0;  // 起きていた時間の合計（ROM 起動ぶんは含まない）
RTC_DATA_ATTR std::uint32_t s_timer_count   = 0;  // 保険のタイマーで起きた回数（多ければ素早い寝直しが壊れている）

// 加速度の履歴（FIFO から読んだ生フレーム）。寝ている間も残し、起きた後の FIFO の続きとつなげる。
constexpr int kHistLen = 128;  // 2.56 秒
RTC_DATA_ATTR std::int16_t s_hist[kHistLen][3];
RTC_DATA_ATTR std::uint16_t s_hist_head  = 0;  // 次に書く位置
RTC_DATA_ATTR std::uint16_t s_hist_count = 0;

// 失敗したら起きたまま止まらず（電池を使い切ってしまう）、再起動してコールドブートからやり直す。
[[noreturn]] void fail(const char* stage, esp_err_t error, int raw_error)
{
    ESP_LOGE(kTag, "BOARD_FAIL stage=%s error=%s raw=%d -> restart", stage, esp_err_to_name(error), raw_error);
    m5sw::test_support::report_final(m5sw::test_support::FinalStatus::fail, "look wake L2 failed");
    vTaskDelay(pdMS_TO_TICKS(500));
    esp_restart();
}

// PM1 / IOE1 は I2C のアイドルスリープから自動で起きるが、起きた直後の 1 回目の通信が失敗することがある
// （起床後に BMI270 だけを 1 秒以上読んでいる間に、PM1 が再びスリープに入ったと思われる）。数回やり直す。
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

// 振動モーターを短く鳴らす（wrist_wake_l2 で動作確認済みの手順）。失敗しても実験は続ける。
// 最後に IOE1 を工場出荷状態に戻す。prepare_l2 と同じく、L2 では IOE1 のピンを全部ハイインピーダンスにしておくため
// （モーターや L3B のピンを Low 出力のまま寝ると、基板のプルアップから電流が流れ続けるおそれがある）。
void buzz(i2c_bus_handle_t bus, int times)
{
    const std::int64_t start = esp_timer_get_time();
    static m5sw::power::Ioe1Adapter ioe1;
    if (!ioe1.initialized()) {
        const auto begun = ioe1.begin(bus, 100000);
        if (!begun.ok()) {
            ESP_LOGW(kTag, "BUZZ_SKIP stage=ioe1_begin error=%s", esp_err_to_name(begun.error));
            return;
        }
    }
    auto result = ioe1.configure_i2c(false, 100000);
    if (!result.ok()) ESP_LOGW(kTag, "BUZZ_WARN stage=ioe1_nosleep error=%s", esp_err_to_name(result.error));
    result = ioe1.set_pin(m5sw::board::IoePin::motor_pwm, false);
    if (!result.ok()) ESP_LOGW(kTag, "BUZZ_WARN stage=motor_pin error=%s", esp_err_to_name(result.error));
    result = ioe1.set_pin(m5sw::board::IoePin::l3b_enable, true);
    if (!result.ok()) {
        ESP_LOGW(kTag, "BUZZ_SKIP stage=l3b_on error=%s", esp_err_to_name(result.error));
        return;
    }
    vTaskDelay(pdMS_TO_TICKS(50));  // L3B 系統の電源が立ち上がるのを待つ
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
    result = ioe1.factory_reset();
    if (!result.ok()) ESP_LOGW(kTag, "BUZZ_WARN stage=ioe1_reset error=%s", esp_err_to_name(result.error));
    ESP_LOGI(kTag, "BUZZ x%d done (%lldms incl. ioe1 reset)", times, (esp_timer_get_time() - start) / 1000);
}

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
    // 見始めの判定（厳しめ）。
    [[nodiscard]] bool look() const
    {
        return still() && x >= kLookMinX && (z >= kLookMinZ || y >= kLookMinY);
    }
    // 見続けている判定（ゆるめ。小さな揺れで消えないように）。
    [[nodiscard]] bool still_viewing() const
    {
        return x >= kLookMinX - 0.15f && (z >= kLookMinZ - 0.1f || y >= kLookMinY - 0.1f);
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

// ---- 加速度の履歴（RTC メモリのリングバッファ） ----
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

// 新しい方から数えて from_end 番目のフレームまでの kSmoothFrames 個の平均。足りなければ false。
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

// from_end のフレームで「見ている姿勢」になったとき、その前 0.2〜2.0 秒の向きとの角度の最大値。比べられなければ -1。
float raise_angle(const Accel& look, int from_end, int* compared)
{
    float best = -1.0f;
    *compared  = 0;
    for (int k = from_end + kRaiseSkipFrames; k <= from_end + kRaiseLookbackFrames; ++k) {
        Accel a;
        if (!hist_avg(k, &a)) break;
        if (!a.still(kHistStillTolerance)) continue;
        ++*compared;
        const float deg = angle_deg(a, look);
        if (deg > best) best = deg;
    }
    return best;
}

// 判定したフレームまでの 2.4 秒を 100ms ごとにログへ出す（あとで判定のしきい値を調べ直すため）。単位は 0.01g。
void log_history(const char* label, int from_end)
{
    char line[512];
    int pos = 0;
    for (int j = 23; j >= 0 && pos < static_cast<int>(sizeof(line)) - 16; --j) {
        Accel a;
        if (!hist_avg(from_end + j * kSmoothFrames, &a)) continue;
        pos += std::snprintf(line + pos, sizeof(line) - pos, "%d,%d,%d ", static_cast<int>(std::lround(a.x * 100)),
                             static_cast<int>(std::lround(a.y * 100)), static_cast<int>(std::lround(a.z * 100)));
    }
    line[pos] = '\0';
    ESP_LOGI(kTag, "HIST %s [old->new, 100ms step, 0.01g] %s", label, line);
}

// ---- BMI270 の FIFO ----
// 寝ている間も加速度（50Hz）を FIFO に溜め続け、起きたら読み出して履歴に足す。
// 設定は BMI270 の初期化で消えるので、初期化した後（コールドブートとタイマー起床）に configure() する。
class ImuFifo {
public:
    explicit ImuFifo(i2c_bus_handle_t bus) : device_(i2c_bus_device_create(bus, m5sw::board::kImuAddress, kImuI2cHz)) {}
    ~ImuFifo()
    {
        if (device_ != nullptr) i2c_bus_device_delete(&device_);
    }

    // 加速度だけ・ヘッダなし・満杯になったら古いものから捨てる（stream モード）。中身を空にして始める。
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
        if (cfg[0] != 0x00 || cfg[1] != 0x40) {
            ESP_LOGW(kTag, "FIFO_CONFIG readback mismatch cfg0=0x%02X cfg1=0x%02X", cfg[0], cfg[1]);
            return ESP_ERR_INVALID_RESPONSE;
        }
        return ESP_OK;
    }

    // 起きている間は BMI270 の advanced power save を切る。オンのままだと FIFO を続けて読んだときに
    // 同じバイトの繰り返しが返り、読んだぶんも減らなかった（v8/v9 のログ）。寝る前に元の値へ戻す。
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

    esp_err_t end_awake()
    {
        if (device_ == nullptr || !have_saved_ || (saved_pwr_conf_ & 0x01) == 0) return ESP_OK;
        return write(kRegPwrConf, saved_pwr_conf_);
    }

    [[nodiscard]] int saved_pwr_conf() const
    {
        return have_saved_ ? saved_pwr_conf_ : -1;
    }

    // 溜まっているフレームを全部読んで履歴に足す。足したフレーム数を返す（失敗は -1）。
    int pull()
    {
        if (device_ == nullptr) return -1;
        std::uint8_t len_raw[2] = {};
        if (i2c_bus_read_bytes(device_, kRegFifoLength, sizeof(len_raw), len_raw) != ESP_OK) return -1;
        esp_rom_delay_us(450);  // 省電力モード中の BMI270 は、アクセスの間に 450us 空ける必要がある
        std::size_t bytes = static_cast<std::size_t>(len_raw[0]) | (static_cast<std::size_t>(len_raw[1] & 0x3F) << 8);
        bytes             = (bytes / kFrameBytes) * kFrameBytes;
        if (bytes > kFifoBytes) bytes = (kFifoBytes / kFrameBytes) * kFrameBytes;
        if (bytes == 0) return 0;
        static std::uint8_t buf[kFifoBytes];
        if (i2c_bus_read_bytes(device_, kRegFifoData, bytes, buf) != ESP_OK) return -1;
        esp_rom_delay_us(450);
        if (debug_left_ > 0 && bytes >= 60) {
            --debug_left_;
            ESP_LOGI(kTag, "FIFO_DBG len_raw=%02X %02X bytes=%u data=%02X%02X %02X%02X %02X%02X | %02X%02X %02X%02X %02X%02X",
                     len_raw[0], len_raw[1], static_cast<unsigned>(bytes), buf[0], buf[1], buf[2], buf[3], buf[4],
                     buf[5], buf[bytes - 6], buf[bytes - 5], buf[bytes - 4], buf[bytes - 3], buf[bytes - 2],
                     buf[bytes - 1]);
        }
        int added = 0;
        for (std::size_t off = 0; off + kFrameBytes <= bytes; off += kFrameBytes) {
            const auto x = static_cast<std::int16_t>(buf[off] | (buf[off + 1] << 8));
            const auto y = static_cast<std::int16_t>(buf[off + 2] | (buf[off + 3] << 8));
            const auto z = static_cast<std::int16_t>(buf[off + 4] | (buf[off + 5] << 8));
            if (x == INT16_MIN && y == INT16_MIN && z == INT16_MIN) continue;  // 空を読んだときの印（0x8000）
            hist_push(x, y, z);
            ++added;
        }
        total_ += added;
        return added;
    }

    [[nodiscard]] int total() const
    {
        return total_;
    }

private:
    esp_err_t write(std::uint8_t reg, std::uint8_t value)
    {
        const esp_err_t result = i2c_bus_write_byte(device_, reg, value);
        esp_rom_delay_us(450);
        return result;
    }

    i2c_bus_device_handle_t device_;
    int total_      = 0;
    std::uint8_t saved_pwr_conf_ = 0;
    bool have_saved_             = false;
    int debug_left_ = 0;   // 一度に多く読んだときだけ、最初の数回、生のバイトをログに出す（FIFO の読み方の確認用）
};

struct LookResult {
    bool looking       = false;
    const char* reason = "window";  // LOOK / pose_away / window / no_raise / fifo_error
    std::int64_t at_ms = -1;        // 起動からの時刻（判定が出た時点）
    Accel acc;                      // 判定したときの加速度（100ms 平均）
    int first_frames = 0;           // 起きて最初に読んだ FIFO のフレーム数（寝ていた間の履歴）
    int frames       = 0;           // 起きてから判定までに見たフレーム数
    float raise_deg  = -1.0f;
    int compared     = 0;
    int decided_at   = 0;  // 判定したフレームの位置（新しい方から数えた番号。ログ用）
};

// 起きた直後に FIFO を最大 kWindowUs 見て、「見ている姿勢」が kLookHoldFrames 続き、
// その前に持ち上げた動きがあったかを調べる。
LookResult check_look(ImuFifo* fifo)
{
    LookResult out;
    const std::int64_t start = esp_timer_get_time();
    const int first          = fifo->pull();
    if (first < 0) {
        out.reason = "fifo_error";
        out.at_ms  = esp_timer_get_time() / 1000;
        return out;
    }
    out.first_frames = first;
    int pose_run     = 0;
    int away_run     = 0;
    while (esp_timer_get_time() - start < kWindowUs) {
        vTaskDelay(pdMS_TO_TICKS(20));
        const int added = fifo->pull();
        if (added <= 0) continue;
        // 新しく来たフレームを古い順に見る。i は新しい方から数えた位置。
        for (int i = added - 1; i >= 0; --i) {
            Accel a;
            if (!hist_avg(i, &a)) continue;
            ++out.frames;
            out.acc         = a;
            const bool pose = a.look();
            pose_run        = pose ? pose_run + 1 : 0;
            away_run        = (!pose && a.still()) ? away_run + 1 : 0;
            if (pose_run >= kLookHoldFrames) {
                // 姿勢が続いた区間の始まりより前と比べる。
                out.raise_deg  = raise_angle(a, i + kLookHoldFrames - 1, &out.compared);
                out.decided_at = i;
                // 比べられる履歴がなければ（コールドブート直後など）、取りこぼさない方を選んで LOOK にする。
                out.looking = out.compared == 0 || out.raise_deg >= kRaiseMinDeg;
                out.reason  = out.looking ? "LOOK" : "no_raise";
                out.at_ms   = esp_timer_get_time() / 1000;
                return out;
            }
            if (away_run >= kAwayHoldFrames) {
                out.reason     = "pose_away";
                out.decided_at = i;
                out.at_ms      = esp_timer_get_time() / 1000;
                return out;
            }
        }
    }
    out.at_ms = esp_timer_get_time() / 1000;
    return out;
}

// 見ている間は起きたまま待つ（画面点灯中の代わり）。腕を下ろしたか、上限時間で戻る。戻り値は待った時間 [ms]。
std::int64_t wait_while_viewing(ImuFifo* fifo, const char** reason)
{
    const std::int64_t start = esp_timer_get_time();
    int low_run              = 0;
    *reason                  = "timeout";
    while (esp_timer_get_time() - start < kViewMaxUs) {
        vTaskDelay(pdMS_TO_TICKS(20));
        const int added = fifo->pull();
        for (int i = added - 1; i >= 0; --i) {
            Accel a;
            if (!hist_avg(i, &a)) continue;
            low_run = a.still_viewing() ? 0 : low_run + 1;
            if (low_run >= kViewEndFrames) {
                *reason = "lowered";
                return (esp_timer_get_time() - start) / 1000;
            }
        }
    }
    return (esp_timer_get_time() - start) / 1000;
}

}  // namespace

extern "C" void app_main(void)
{
    const std::int64_t app_start_ms = esp_timer_get_time() / 1000;
    m5sw::test_support::TestContext context = {};
    ESP_ERROR_CHECK(m5sw::test_support::initialize(&context));
    esp_log_level_set("i2c_bus", ESP_LOG_ERROR);
    esp_log_level_set("M5IOE1_PWM", ESP_LOG_WARN);

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

    // IMU で起きたときは BMI270 を初期化しない（寝る前の any-motion と FIFO の設定をそのまま使う）。
    const bool imu_wake = context.wake_cause == ESP_SLEEP_WAKEUP_EXT0;
    static m5sw::wakeup::WakeController wake;
    auto wake_result = wake.begin(bus, !imu_wake);
    if (!wake_result.ok()) fail("wake_begin", wake_result.error, wake_result.raw_error);
    const std::int64_t ready_ms = esp_timer_get_time() / 1000;
    m5sw::wakeup::WakeSnapshot snapshot;
    wake_result = wake.capture(retained_rtc, retained_imu, &snapshot);
    if (!wake_result.ok()) fail("wake_capture", wake_result.error, wake_result.raw_error);

    static ImuFifo fifo(bus);

    // 素早い寝直し（prepare_l2 を省く）を使えるのは IMU で起きたとき。振動の後も IOE1 を工場出荷状態に戻すので使える。
    const bool quick = imu_wake;
    if (context.wake_cause == ESP_SLEEP_WAKEUP_UNDEFINED) {
        s_wake_count = s_look_count = s_reject_count = s_noraise_count = s_timer_count = 0;
        s_awake_ms = 0;
        ESP_LOGI(kTag, "COLD_BOOT: counters reset (any dur=%u thr=%u raise>=%.0fdeg)", kAnyDuration, kAnyThreshold,
                 kRaiseMinDeg);
        buzz(bus, 1);
    } else if (context.wake_cause == ESP_SLEEP_WAKEUP_TIMER) {
        ++s_timer_count;
        ESP_LOGW(kTag, "TIMER_WAKE #%lu (fallback; IMU wake may be broken)", static_cast<unsigned long>(s_timer_count));
    } else {
        ++s_wake_count;
        const esp_err_t aps_result = fifo.begin_awake();
        if (aps_result != ESP_OK) ESP_LOGW(kTag, "APS_OFF failed error=%s", esp_err_to_name(aps_result));
        const LookResult look = check_look(&fifo);
        if (look.looking) ++s_look_count;
        else if (std::strcmp(look.reason, "no_raise") == 0) ++s_noraise_count;
        else ++s_reject_count;
        ESP_LOGI(kTag,
                 "WAKE #%lu %s  app=%lldms ready=%lldms decided=%lldms acc=(%+.2f %+.2f %+.2f) raise=%.0fdeg(n=%d) "
                 "pwr_conf=%d fifo_first=%d frames=%d any=%d  total look=%lu noraise=%lu reject=%lu timer=%lu avg_awake=%llums",
                 static_cast<unsigned long>(s_wake_count), look.reason, app_start_ms, ready_ms, look.at_ms, look.acc.x,
                 look.acc.y, look.acc.z, look.raise_deg, look.compared, fifo.saved_pwr_conf(), look.first_frames, look.frames,
                 snapshot.imu_any_motion, static_cast<unsigned long>(s_look_count),
                 static_cast<unsigned long>(s_noraise_count), static_cast<unsigned long>(s_reject_count),
                 static_cast<unsigned long>(s_timer_count), s_wake_count > 1 ? s_awake_ms / (s_wake_count - 1) : 0ULL);
        log_history(look.reason, look.decided_at);
        if (look.looking) {
            buzz(bus, 1);
            const char* reason           = "";
            const std::int64_t viewed_ms = wait_while_viewing(&fifo, &reason);
            ESP_LOGI(kTag, "VIEW_END after %lldms (%s)", viewed_ms, reason);
        }
    }

    wake_result = with_retry([&] { return wake.clear_snapshot(snapshot); });
    if (!wake_result.ok()) fail("wake_clear", wake_result.error, wake_result.raw_error);
    rtc_result = with_retry([&] { return rtc.clear_flags(retained_rtc); });
    if (!rtc_result.ok()) fail("rtc_clear", rtc_result.error, rtc_result.raw_error);
    rtc_result = with_retry([&] { return rtc.disable_interrupts(); });
    if (!rtc_result.ok()) fail("rtc_disable", rtc_result.error, rtc_result.raw_error);

    static m5sw::power::PowerController power;
    auto power_result = with_retry([&] { return power.begin(bus); });
    if (!power_result.ok()) fail("pm1_begin", power_result.error, power_result.raw_error);

    if (quick) {
        // PM1 の GPIO 起床設定と BMI270 の any-motion / FIFO 設定は前回のまま残っている。
        const esp_err_t aps_result = fifo.end_awake();
        if (aps_result != ESP_OK) ESP_LOGW(kTag, "APS_RESTORE failed error=%s", esp_err_to_name(aps_result));
        ESP_LOGI(kTag, "SLEEP quick at %lldms", esp_timer_get_time() / 1000);
    } else {
        wake_result = wake.begin_imu();
        if (!wake_result.ok()) fail("imu_begin", wake_result.error, wake_result.raw_error);
        power_result = power.prepare_l2(m5sw::power::WakeSelection::rtc_imu);
        if (!power_result.ok()) fail("l2_prepare", power_result.error, power_result.raw_error);
        wake_result = wake.select_source(m5sw::wakeup::WakeSelection::imu, m5sw::wakeup::ImuWakeProfile::low_power,
                                         kAnyDuration, kAnyThreshold);
        if (!wake_result.ok()) fail("imu_arm", wake_result.error, wake_result.raw_error);
        // BMI270 を初期化し直したので FIFO も設定し直し、履歴を捨てる。
        const esp_err_t fifo_result = fifo.configure();
        if (fifo_result != ESP_OK) fail("fifo_config", fifo_result, fifo_result);
        hist_reset();
        const esp_err_t aps_probe = fifo.begin_awake();  // 値を読むだけ（APS がオンなら切れるので、すぐ戻す）
        (void)fifo.end_awake();
        ESP_LOGI(kTag, "SLEEP full at %lldms (fifo armed, pwr_conf=%d %s)", esp_timer_get_time() / 1000,
                 fifo.saved_pwr_conf(), esp_err_to_name(aps_probe));
    }

    ESP_ERROR_CHECK(m5sw::test_support::mark_test_ready());
    vTaskDelay(kPrepareDelay);
    // 起動からスリープまでの時間を電池見積もり用に積む（ROM 起動ぶんは含まない。enter_l2 内の 1 秒待ちは足しておく）。
    if (context.wake_cause != ESP_SLEEP_WAKEUP_UNDEFINED) s_awake_ms += esp_timer_get_time() / 1000 + 1000;

    if (quick) {
        power_result = power.enter_l2_quick(m5sw::power::WakeSelection::rtc_imu, kTimerFallbackUs);
    } else {
        power_result = power.enter_l2(m5sw::power::WakeSelection::rtc_imu);
    }
    fail("l2_returned", power_result.error, power_result.raw_error);
}
