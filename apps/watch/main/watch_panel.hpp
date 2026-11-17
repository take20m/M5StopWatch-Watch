// 腕時計用の画面ドライバ。描画は LovyanGFX のスプライト（PSRAM 上の画像）で行い、
// パネルへの転送は ESP-IDF の esp_lcd（SPI 4 線）で行う。
//
// M5GFX の転送（Bus_SPI + フレームバッファ）では、まれに画面全体が横にずれて折り返したり、
// 色が入れ替わったりした（2026-11-15。転送の待ち合わせ・CS の幅を直しても再発）。
// 転送を実績のある esp_lcd に置き換えて、M5GFX は画像を作るだけにする。
#pragma once

#include "m5sw_board.hpp"

#include <cstdint>

#include "M5GFX.h"
#include "esp_err.h"
#include "i2c_bus.h"

namespace watch {

class WatchPanel {
public:
    static constexpr int kWidth  = 468;  // パネルの横（表示は円形 466。左右に 1 列ずつ余り）
    static constexpr int kHeight = 466;

    // 電源を入れてパネルを初期化する（M5GFX で初期化し、転送だけ esp_lcd に切り替える）。
    esp_err_t begin(i2c_bus_handle_t bus);
    // 描画先。begin が成功してから使う。
    lgfx::LGFX_Sprite* canvas() { return &canvas_; }
    // canvas の y0〜y1 行（両端を含む）をパネルへ送る。
    esp_err_t push_rows(int y0, int y1);
    esp_err_t push_all() { return push_rows(0, kHeight - 1); }
    esp_err_t set_brightness_percent(std::uint8_t percent);
    // パネルを眠らせ、電源と SPI を切り、ピンを入力に戻す。
    esp_err_t power_off();
    // パネルを眠らせるだけ（電源は入れたまま）。次の begin は起こすだけで済む。
    esp_err_t sleep();

private:
    esp_err_t open_bus();
    esp_err_t send_cmd(std::uint8_t cmd, const std::uint8_t* params, std::size_t size);

    lgfx::LGFX_Sprite canvas_;
    void* io_                 = nullptr;  // esp_lcd_panel_io_handle_t
    bool bus_ready_           = false;
    bool powered_             = false;
    // 速い起こし方では、スリープ解除の後の表示オンを、最初に明るさを上げるときまで遅らせる
    // （待ちの 20ms の間に文字盤を描けるように）。
    bool display_on_pending_  = false;
    std::int64_t sleep_out_us_ = 0;
    esp_err_t finish_display_on();
    i2c_bus_handle_t bus_     = nullptr;
};

// IOE1 を初期化して画面の電源が切れたとき（prepare_l2 の後）に呼ぶ。
void panel_power_lost();

// USB の切替（CH442E）を ESP32 側に固定する。コールドブート時と、画面の電源を切った後に呼ぶ。
esp_err_t hold_usb_mux(i2c_bus_handle_t bus);

// 振動モーター（IOE1 の IO9）を回す／止める。タイマーが鳴ったときに使う。
esp_err_t set_vibration(i2c_bus_handle_t bus, bool on);

// IOE1 のピンを High / Low にする（音声チップやスピーカーの電源など）。
esp_err_t set_ioe_pin(i2c_bus_handle_t bus, m5sw::board::IoePin pin, bool high);

}  // namespace watch
