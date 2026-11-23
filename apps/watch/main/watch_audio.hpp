// スピーカーで短い音を鳴らす（ES8311 + I2S）。鳴らすのは別タスクで、呼んだ側は待たない。
#pragma once

#include <cstdint>

#include "i2c_bus.h"

namespace watch {

enum class Sound { tap, beep };

// 音を鳴らす（最初の 1 回は音声チップの電源を入れて設定する）。
void audio_play(i2c_bus_handle_t bus, Sound sound);

// 音量（ES8311 の DAC 音量。0xBF = 0dB、1 段 0.5dB）。次に鳴らす音から効く。
void audio_set_volume(std::uint8_t reg);

// 鳴っている音が終わるのを待ってから、スピーカーと音声チップの電源を切る。画面を消すときに呼ぶ。
void audio_shutdown();

}  // namespace watch
