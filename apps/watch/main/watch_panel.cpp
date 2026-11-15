#include "watch_panel.hpp"

#include <algorithm>
#include <cstring>

#include "driver/gpio.h"
#include "hal/gpio_ll.h"
#include "soc/gpio_struct.h"
#include "driver/spi_master.h"
#include "esp_heap_caps.h"
#include "esp_lcd_panel_io.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "esp_rom_sys.h"
#include "m5sw_board.hpp"
#include "m5sw_display.hpp"
#include "m5sw_ioe1.hpp"

namespace watch {
namespace {

constexpr char kTag[] = "panel";

// ピン（modules/m5sw_display と同じ）
constexpr gpio_num_t kPanelTe    = GPIO_NUM_38;
constexpr gpio_num_t kPanelCs    = GPIO_NUM_39;
constexpr gpio_num_t kPanelClock = GPIO_NUM_40;
constexpr gpio_num_t kPanelData0 = GPIO_NUM_41;
constexpr gpio_num_t kPanelData1 = GPIO_NUM_42;
constexpr gpio_num_t kPanelData2 = GPIO_NUM_46;
constexpr gpio_num_t kPanelData3 = GPIO_NUM_45;

constexpr spi_host_device_t kHost  = SPI2_HOST;
constexpr int kPclkHz              = 40 * 1000 * 1000;
constexpr int kColumnOffset        = 6;   // パネルの列 6〜473 が表示範囲
constexpr int kBandRows            = 32;  // SPI の 1 回の転送の最大行数（esp_lcd が分割する）
constexpr std::size_t kRowBytes    = WatchPanel::kWidth * 2;
constexpr std::size_t kBandBytes   = kRowBytes * kBandRows;

// CO5300 の QSPI: コマンドは「0x02, 0x00, cmd, 0x00」を 1 線で、画素は「0x32, 0x00, 0x2C, 0x00」の後に 4 線で送る。
constexpr int cmd_word(std::uint8_t cmd) { return (0x02 << 24) | (cmd << 8); }
constexpr int kWriteMemoryWord = (0x32 << 24) | (0x2C << 8);

m5sw::display::DisplayController s_m5gfx;  // 電源投入・初期化・電源断を担当
m5sw::power::Ioe1Adapter s_ioe1;            // 電源断の後に USB 切替を固定するため
// パネルの電源を入れたまま眠らせているか。ディープスリープをまたいで保つ。
RTC_DATA_ATTR bool s_panel_asleep = false;
constexpr int kSleepOutWaitMs     = 20;  // スリープ解除から表示オンまでの待ち

esp_lcd_panel_io_handle_t as_io(void* p) { return static_cast<esp_lcd_panel_io_handle_t>(p); }

}  // namespace

esp_err_t WatchPanel::send_cmd(std::uint8_t cmd, const std::uint8_t* params, std::size_t size)
{
    return esp_lcd_panel_io_tx_param(as_io(io_), cmd_word(cmd), params, size);
}

// IOE1 を初期状態に戻すと、USB の切替（CH442E）の制御ピン IO1 が入力（浮いた状態）になり、
// USB が ESP32 から外れることがある（2026-11-15: USB が頻繁に切れていた。UserDemo は常に Low に固定している）。
// Low（ESP32 側）に固定し直す。IOE1 は ESP32 のディープスリープ中も状態を保つ。
esp_err_t hold_usb_mux(i2c_bus_handle_t bus)
{
    if (bus == nullptr) return ESP_ERR_INVALID_ARG;
    auto r = s_ioe1.begin(bus, 100000);
    if (!r.ok()) return r.error;
    r = s_ioe1.set_pin(m5sw::board::IoePin::mux_select, false);
    if (!r.ok()) return r.error;
    // タッチのリセット線も浮いた状態になるので、High（動作）に固定する（タップで起きるため）。
    r = s_ioe1.set_pin(m5sw::board::IoePin::touch_reset, true);
    return r.ok() ? ESP_OK : r.error;
}

esp_err_t set_ioe_pin(i2c_bus_handle_t bus, m5sw::board::IoePin pin, bool high)
{
    if (bus == nullptr) return ESP_ERR_INVALID_ARG;
    static bool ready = false;
    if (!ready) {
        const auto r = s_ioe1.begin(bus, 100000);
        if (!r.ok()) return r.error;
        ready = true;
    }
    const auto r = s_ioe1.set_pin(pin, high);
    return r.ok() ? ESP_OK : r.error;
}

esp_err_t set_vibration(i2c_bus_handle_t bus, bool on)
{
    return set_ioe_pin(bus, m5sw::board::IoePin::motor_pwm, on);
}

esp_err_t WatchPanel::open_bus()
{
    // SPI（4 線）と esp_lcd
    spi_bus_config_t bus_config = {};
    bus_config.data0_io_num     = kPanelData0;
    bus_config.data1_io_num     = kPanelData1;
    bus_config.sclk_io_num      = kPanelClock;
    bus_config.data2_io_num     = kPanelData2;
    bus_config.data3_io_num     = kPanelData3;
    bus_config.data4_io_num     = -1;
    bus_config.data5_io_num     = -1;
    bus_config.data6_io_num     = -1;
    bus_config.data7_io_num     = -1;
    bus_config.max_transfer_sz  = kBandBytes;
    bus_config.flags            = SPICOMMON_BUSFLAG_MASTER | SPICOMMON_BUSFLAG_QUAD;
    esp_err_t err               = spi_bus_initialize(kHost, &bus_config, SPI_DMA_CH_AUTO);
    if (err != ESP_OK) return err;
    bus_ready_ = true;
    // M5GFX と同じく、バスのピンのオープンドレインを外す（spi_bus_initialize は外さない）。
    for (gpio_num_t pin : {kPanelClock, kPanelData0, kPanelData1, kPanelData2, kPanelData3, kPanelCs}) {
        gpio_ll_od_disable(&GPIO, pin);
    }

    esp_lcd_panel_io_spi_config_t io_config = {};
    io_config.cs_gpio_num                   = kPanelCs;
    io_config.dc_gpio_num                   = -1;
    io_config.spi_mode                      = 0;
    io_config.pclk_hz                       = kPclkHz;
    io_config.trans_queue_depth             = 4;
    io_config.lcd_cmd_bits                  = 32;
    io_config.lcd_param_bits                = 8;
    io_config.flags.quad_mode               = 1;
    esp_lcd_panel_io_handle_t io            = nullptr;
    err = esp_lcd_new_panel_io_spi(static_cast<esp_lcd_spi_bus_handle_t>(kHost), &io_config, &io);
    if (err != ESP_OK) return err;
    io_ = io;

    return ESP_OK;
}

esp_err_t WatchPanel::begin(i2c_bus_handle_t bus)
{
    const int64_t t0 = esp_timer_get_time();
    bus_ = bus;
    if (s_panel_asleep) {
        // 電源は入ったまま、パネルが眠っている。起こすだけ（電源投入・リセット・初期化を省く）。
        if (canvas_.getBuffer() == nullptr) {
            canvas_.setPsram(true);
            canvas_.setColorDepth(16);
            if (canvas_.createSprite(kWidth, kHeight) == nullptr) return ESP_ERR_NO_MEM;
        }
        gpio_hold_dis(kPanelCs);
        powered_      = true;
        esp_err_t err = open_bus();
        if (err != ESP_OK) return err;
        err = send_cmd(0x11, nullptr, 0);  // スリープ解除
        if (err != ESP_OK) return err;
        vTaskDelay(pdMS_TO_TICKS(kSleepOutWaitMs));
        err = send_cmd(0x29, nullptr, 0);  // 表示オン（明るさは 0 のまま。絵を送ってから上げる）
        if (err != ESP_OK) return err;
        s_panel_asleep = false;
        canvas_.fillScreen(TFT_BLACK);
        ESP_LOGI(kTag, "BEGIN_TIMING fast=%lldms", (esp_timer_get_time() - t0) / 1000);
        return ESP_OK;
    }
    // 描画先（PSRAM）とDMA 用の帯バッファ（内部 RAM）
    if (canvas_.getBuffer() == nullptr) {
        canvas_.setPsram(true);
        canvas_.setColorDepth(16);
        if (canvas_.createSprite(kWidth, kHeight) == nullptr) return ESP_ERR_NO_MEM;
    }

    // 電源投入と初期化は M5GFX（実績あり）に任せ、終わったら SPI だけ手放してもらう。
    // 自前の初期化ではパネルが動かなかった（2026-11-15: TE が出ない。esp_lcd の命令自体は届くことを確認済み）。
    const int64_t t1 = esp_timer_get_time();
    m5sw::display::DisplayController& m5gfx = s_m5gfx;
    const auto started = m5gfx.begin(bus, 100000, false);  // フレームバッファなし・暗いまま（速くするため）
    if (!started.ok()) {
        (void)m5gfx.power_off();
        return started.error;
    }
    const int64_t t2 = esp_timer_get_time();
    m5gfx.release_bus_keep_power();
    powered_ = true;

    esp_err_t err = open_bus();
    if (err != ESP_OK) return err;
    canvas_.fillScreen(TFT_BLACK);
    const int64_t t3 = esp_timer_get_time();
    ESP_LOGI(kTag, "BEGIN_TIMING canvas=%lldms m5gfx=%lldms attach=%lldms", (t1 - t0) / 1000, (t2 - t1) / 1000,
             (t3 - t2) / 1000);
    return ESP_OK;
}

esp_err_t WatchPanel::push_rows(int y0, int y1)
{
    if (io_ == nullptr) return ESP_ERR_INVALID_STATE;
    y0 = std::max(0, y0) & ~1;               // パネルの制約: 開始行は偶数
    y1 = std::min(kHeight - 1, y1 | 1);      // 終了行は奇数（行数が偶数）
    const auto* src = static_cast<const std::uint8_t*>(canvas_.getBuffer());
    const std::uint16_t xs = kColumnOffset, xe = kColumnOffset + kWidth - 1;
    const std::uint8_t caset[4] = {static_cast<std::uint8_t>(xs >> 8), static_cast<std::uint8_t>(xs & 0xFF),
                                   static_cast<std::uint8_t>(xe >> 8), static_cast<std::uint8_t>(xe & 0xFF)};
    const std::uint8_t raset[4] = {static_cast<std::uint8_t>(y0 >> 8), static_cast<std::uint8_t>(y0 & 0xFF),
                                   static_cast<std::uint8_t>(y1 >> 8), static_cast<std::uint8_t>(y1 & 0xFF)};
    esp_err_t err = send_cmd(0x2A, caset, sizeof(caset));
    if (err != ESP_OK) return err;
    err = send_cmd(0x2B, raset, sizeof(raset));
    if (err != ESP_OK) return err;
    // esp_lcd が最大転送長ごとに分け、CS を下げたまま続けて送る。PSRAM の画像は SPI ドライバが内部 RAM に写してから送る。
    // 次の send_cmd は送り終わるのを待つ。
    return esp_lcd_panel_io_tx_color(as_io(io_), kWriteMemoryWord, src + static_cast<std::size_t>(y0) * kRowBytes,
                                     static_cast<std::size_t>(y1 - y0 + 1) * kRowBytes);
}

esp_err_t WatchPanel::set_brightness_percent(std::uint8_t percent)
{
    if (io_ == nullptr) return ESP_ERR_INVALID_STATE;
    const std::uint8_t value = static_cast<std::uint8_t>((std::min<int>(percent, 100) * 255 + 50) / 100);
    return send_cmd(0x51, &value, 1);
}

esp_err_t WatchPanel::sleep()
{
    esp_err_t first_error = ESP_OK;
    auto keep             = [&](esp_err_t e) {
        if (first_error == ESP_OK && e != ESP_OK) first_error = e;
    };
    if (io_ != nullptr) {
        const std::uint8_t zero = 0;
        keep(send_cmd(0x51, &zero, 1));
        keep(send_cmd(0x28, nullptr, 0));  // 表示オフ
        keep(send_cmd(0x10, nullptr, 0));  // スリープ（電源は切らない）
        keep(esp_lcd_panel_io_del(as_io(io_)));
        io_ = nullptr;
    }
    if (bus_ready_) {
        keep(spi_bus_free(kHost));
        bus_ready_ = false;
    }
    // データ線は入力に戻し、CS は High のままディープスリープ中も保持する（パネルが雑音を命令と取らないように）。
    gpio_config_t pins = {};
    pins.pin_bit_mask  = (1ULL << kPanelClock) | (1ULL << kPanelData0) | (1ULL << kPanelData1) | (1ULL << kPanelData2) |
                        (1ULL << kPanelData3) | (1ULL << kPanelTe);
    pins.mode         = GPIO_MODE_INPUT;
    pins.pull_up_en   = GPIO_PULLUP_DISABLE;
    pins.pull_down_en = GPIO_PULLDOWN_DISABLE;
    pins.intr_type    = GPIO_INTR_DISABLE;
    keep(gpio_config(&pins));
    gpio_reset_pin(kPanelCs);
    gpio_set_direction(kPanelCs, GPIO_MODE_OUTPUT);
    gpio_set_level(kPanelCs, 1);
    gpio_hold_en(kPanelCs);
    gpio_deep_sleep_hold_en();
    s_panel_asleep = powered_;
    powered_       = false;
    if (first_error != ESP_OK) ESP_LOGW(kTag, "sleep error=%s", esp_err_to_name(first_error));
    return first_error;
}

void panel_power_lost()
{
    s_panel_asleep = false;
    gpio_hold_dis(kPanelCs);
}

esp_err_t WatchPanel::power_off()
{
    esp_err_t first_error = ESP_OK;
    auto keep             = [&](esp_err_t e) {
        if (first_error == ESP_OK && e != ESP_OK) first_error = e;
    };
    if (io_ != nullptr) {
        const std::uint8_t zero = 0;
        keep(send_cmd(0x51, &zero, 1));
        keep(send_cmd(0x28, nullptr, 0));  // 表示オフ
        keep(send_cmd(0x10, nullptr, 0));  // スリープ
        vTaskDelay(pdMS_TO_TICKS(10));
        keep(esp_lcd_panel_io_del(as_io(io_)));
        io_ = nullptr;
    }
    if (bus_ready_) {
        keep(spi_bus_free(kHost));
        bus_ready_ = false;
    }
    if (powered_) {
        // IOE1 を初期状態に戻し（L3B オフ）、ピンを入力に戻す
        const auto r = s_m5gfx.cut_power();
        if (!r.ok()) keep(r.error);
        powered_ = false;
        keep(hold_usb_mux(bus_));
    }
    gpio_config_t pins = {};
    pins.pin_bit_mask  = (1ULL << kPanelCs) | (1ULL << kPanelClock) | (1ULL << kPanelData0) | (1ULL << kPanelData1) |
                        (1ULL << kPanelData2) | (1ULL << kPanelData3) | (1ULL << kPanelTe);
    pins.mode         = GPIO_MODE_INPUT;
    pins.pull_up_en   = GPIO_PULLUP_DISABLE;
    pins.pull_down_en = GPIO_PULLDOWN_DISABLE;
    pins.intr_type    = GPIO_INTR_DISABLE;
    keep(gpio_config(&pins));
    if (first_error != ESP_OK) ESP_LOGW(kTag, "power_off error=%s", esp_err_to_name(first_error));
    return first_error;
}

}  // namespace watch

