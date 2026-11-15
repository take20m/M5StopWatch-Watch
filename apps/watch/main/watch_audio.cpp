#include "watch_audio.hpp"

#include <cmath>
#include <cstdint>
#include <cstring>
#include <algorithm>

#include "driver/gpio.h"
#include "driver/i2s_std.h"
#include "esp_log.h"
#include "esp_random.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/semphr.h"
#include "freertos/task.h"
#include "m5sw_board.hpp"
#include "watch_panel.hpp"

namespace watch {
namespace {

constexpr char kTag[]             = "audio";
constexpr int kSampleRate         = 16000;  // MCLK = 256 × 16kHz = 4.096MHz
constexpr gpio_num_t kSpeakerPa   = GPIO_NUM_14;  // スピーカーアンプ（IOE1 の IO10 と両方 High で鳴る。UserDemo と同じ）
constexpr std::uint8_t kVolume    = 0xB0;  // ES8311 の DAC 音量（0xBF = 0dB）

i2c_bus_handle_t s_bus            = nullptr;
i2c_bus_device_handle_t s_codec   = nullptr;
i2s_chan_handle_t s_tx            = nullptr;
QueueHandle_t s_queue             = nullptr;
SemaphoreHandle_t s_idle          = nullptr;  // 鳴らしていない間は取れる
bool s_powered                    = false;

esp_err_t codec_write(std::uint8_t reg, std::uint8_t value) { return i2c_bus_write_byte(s_codec, reg, value); }

// ES8311 を「I2S スレーブ、16 ビット、MCLK = 256fs、DAC から鳴らす」に設定する（ESPHome の es8311 と同じ手順）。
esp_err_t codec_init()
{
    if (s_codec == nullptr) s_codec = i2c_bus_device_create(s_bus, m5sw::board::kAudioCodecAddress, 100000);
    if (s_codec == nullptr) return ESP_ERR_NO_MEM;
    const std::uint8_t seq[][2] = {
        {0x00, 0x1F}, {0x00, 0x00},                                            // リセット
        {0x01, 0x3F}, {0x02, 0x00}, {0x03, 0x10}, {0x04, 0x10}, {0x05, 0x00},  // クロック（4.096MHz / 16kHz）
        {0x06, 0x03}, {0x07, 0x00}, {0x08, 0xFF},
        {0x00, 0x00},                                                          // スレーブ
        {0x09, 0x0C}, {0x0A, 0x0C},                                            // I2S 16 ビット
        {0x14, 0x1A}, {0x16, 0x00}, {0x17, 0xC8},                              // マイク側（使わない）
        {0x32, kVolume},
        {0x0D, 0x01}, {0x0E, 0x02}, {0x12, 0x00}, {0x13, 0x10}, {0x1C, 0x6A}, {0x37, 0x08},
        {0x00, 0x80},                                                          // 動作開始
        {0x31, 0x00},                                                          // ミュート解除
    };
    for (const auto& w : seq) {
        const esp_err_t err = codec_write(w[0], w[1]);
        if (err != ESP_OK) return err;
    }
    return ESP_OK;
}

esp_err_t i2s_init()
{
    i2s_chan_config_t chan = I2S_CHANNEL_DEFAULT_CONFIG(I2S_NUM_0, I2S_ROLE_MASTER);
    chan.auto_clear        = true;
    esp_err_t err          = i2s_new_channel(&chan, &s_tx, nullptr);
    if (err != ESP_OK) return err;
    i2s_std_config_t std = {};
    std.clk_cfg          = I2S_STD_CLK_DEFAULT_CONFIG(kSampleRate);
    std.slot_cfg         = I2S_STD_PHILIPS_SLOT_DEFAULT_CONFIG(I2S_DATA_BIT_WIDTH_16BIT, I2S_SLOT_MODE_STEREO);
    std.gpio_cfg.mclk    = m5sw::board::kI2sMclk;
    std.gpio_cfg.bclk    = m5sw::board::kI2sBclk;
    std.gpio_cfg.ws      = m5sw::board::kI2sLrck;
    std.gpio_cfg.dout    = m5sw::board::kI2sDacData;
    std.gpio_cfg.din     = I2S_GPIO_UNUSED;
    err                  = i2s_channel_init_std_mode(s_tx, &std);
    if (err == ESP_OK) err = i2s_channel_enable(s_tx);
    return err;
}

esp_err_t power_up()
{
    if (s_powered) return ESP_OK;
    esp_err_t err = set_ioe_pin(s_bus, m5sw::board::IoePin::audio_enable, true);
    vTaskDelay(pdMS_TO_TICKS(10));
    if (err == ESP_OK && s_tx == nullptr) err = i2s_init();  // MCLK を先に出してから ES8311 を設定する
    if (err == ESP_OK) err = codec_init();
    if (err == ESP_OK) err = set_ioe_pin(s_bus, m5sw::board::IoePin::speaker_enable, true);
    gpio_reset_pin(kSpeakerPa);
    gpio_set_direction(kSpeakerPa, GPIO_MODE_OUTPUT);
    gpio_set_level(kSpeakerPa, 1);
    if (err != ESP_OK) ESP_LOGW(kTag, "power_up error=%s", esp_err_to_name(err));
    s_powered = err == ESP_OK;
    return err;
}

void power_down()
{
    if (!s_powered) return;
    gpio_set_level(kSpeakerPa, 0);
    (void)set_ioe_pin(s_bus, m5sw::board::IoePin::speaker_enable, false);
    (void)codec_write(0x31, 0x60);  // ミュート
    if (s_tx != nullptr) {
        i2s_channel_disable(s_tx);
        i2s_del_channel(s_tx);
        s_tx = nullptr;
    }
    (void)set_ioe_pin(s_bus, m5sw::board::IoePin::audio_enable, false);
    s_powered = false;
}

// 「ぴょん」という音（周波数を上げながら、すぐ小さくなる）。ステレオの左右に同じ値を書く。
void play_tap()
{
    constexpr int kMs = 260;
    constexpr int kN  = kSampleRate * kMs / 1000;
    static std::int16_t buf[256 * 2];
    double phase = 0;
    int filled   = 0;
    for (int i = 0; i < kN; ++i) {
        const double t    = static_cast<double>(i) / kSampleRate;
        const double x    = static_cast<double>(i) / kN;
        const double freq = 380.0 * std::pow(1100.0 / 380.0, std::min(1.0, x * 1.6)) * (1.0 + 0.03 * std::sin(2 * M_PI * 18 * t));
        phase += 2 * M_PI * freq / kSampleRate;
        const double env  = std::min(1.0, i / (0.004 * kSampleRate)) * std::exp(-4.0 * x);
        const auto v      = static_cast<std::int16_t>(9000.0 * env * (std::sin(phase) + 0.25 * std::sin(2 * phase)));
        buf[filled * 2]     = v;
        buf[filled * 2 + 1] = v;
        if (++filled == 256 || i == kN - 1) {
            std::size_t written = 0;
            i2s_channel_write(s_tx, buf, filled * 2 * sizeof(std::int16_t), &written, pdMS_TO_TICKS(200));
            filled = 0;
        }
    }
    // 最後に少し無音を流して、途中で切れた音が残らないようにする
    std::memset(buf, 0, sizeof(buf));
    for (int k = 0; k < 4; ++k) {
        std::size_t written = 0;
        i2s_channel_write(s_tx, buf, sizeof(buf), &written, pdMS_TO_TICKS(200));
    }
}

#if WATCH_HAS_VOICES
extern const std::uint8_t voices_bin_start[] asm("_binary_voices_bin_start");

// 埋め込んだ声から 1 つ選んで鳴らす（同じ声が続かないように）。声がなければ false。
bool play_voice()
{
    const std::uint8_t* p = voices_bin_start;
    if (std::memcmp(p, "SND1", 4) != 0) return false;
    const int count = p[4] | (p[5] << 8);
    const int rate  = p[6] | (p[7] << 8);
    if (count == 0 || rate != kSampleRate) return false;
    static int last = -1;
    int pick        = static_cast<int>(esp_random() % count);
    if (count > 1 && pick == last) pick = (pick + 1) % count;
    last                     = pick;
    const std::uint8_t* row  = p + 8 + pick * 8;
    const std::uint32_t off  = row[0] | (row[1] << 8) | (row[2] << 16) | (static_cast<std::uint32_t>(row[3]) << 24);
    const std::uint32_t n    = row[4] | (row[5] << 8) | (row[6] << 16) | (static_cast<std::uint32_t>(row[7]) << 24);
    const auto* pcm          = reinterpret_cast<const std::int16_t*>(p + 8 + count * 8) + off;
    static std::int16_t buf[256 * 2];
    for (std::uint32_t i = 0; i < n;) {
        const std::uint32_t m = std::min<std::uint32_t>(256, n - i);
        for (std::uint32_t k = 0; k < m; ++k) buf[k * 2] = buf[k * 2 + 1] = pcm[i + k];
        std::size_t written = 0;
        i2s_channel_write(s_tx, buf, m * 2 * sizeof(std::int16_t), &written, pdMS_TO_TICKS(200));
        i += m;
    }
    std::memset(buf, 0, sizeof(buf));
    for (int k = 0; k < 4; ++k) {
        std::size_t written = 0;
        i2s_channel_write(s_tx, buf, sizeof(buf), &written, pdMS_TO_TICKS(200));
    }
    ESP_LOGI(kTag, "VOICE %d/%d (%lums)", pick, count, static_cast<unsigned long>(n * 1000 / kSampleRate));
    return true;
}
#else
bool play_voice() { return false; }
#endif

void audio_task(void*)
{
    Sound sound;
    for (;;) {
        if (xQueueReceive(s_queue, &sound, portMAX_DELAY) != pdTRUE) continue;
        xSemaphoreTake(s_idle, portMAX_DELAY);
        if (power_up() == ESP_OK) {
            const std::int64_t t0 = esp_log_timestamp();
            if (!play_voice()) play_tap();
            ESP_LOGI(kTag, "PLAY tap %lums", static_cast<unsigned long>(esp_log_timestamp() - t0));
        }
        xSemaphoreGive(s_idle);
    }
}

}  // namespace

void audio_play(i2c_bus_handle_t bus, Sound sound)
{
    if (s_queue == nullptr) {
        s_bus   = bus;
        s_queue = xQueueCreate(2, sizeof(Sound));
        s_idle  = xSemaphoreCreateMutex();
        xTaskCreatePinnedToCore(audio_task, "audio", 4096, nullptr, 4, nullptr, 0);
    }
    (void)xQueueSend(s_queue, &sound, 0);  // いっぱいなら捨てる（連打）
}

void audio_shutdown()
{
    if (s_queue == nullptr) return;
    // 鳴らす予定の音がなくなり、今の音が終わるまで待つ（最大 1 秒）
    for (int i = 0; i < 50 && uxQueueMessagesWaiting(s_queue) > 0; ++i) vTaskDelay(pdMS_TO_TICKS(20));
    if (xSemaphoreTake(s_idle, pdMS_TO_TICKS(1000)) == pdTRUE) {
        power_down();
        xSemaphoreGive(s_idle);
    }
}

}  // namespace watch
