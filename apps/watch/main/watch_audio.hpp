// スピーカーで短い音を鳴らす（ES8311 + I2S）。鳴らすのは別タスクで、呼んだ側は待たない。
#pragma once

#include "i2c_bus.h"

namespace watch {

enum class Sound { tap };

// 音を鳴らす（最初の 1 回は音声チップの電源を入れて設定する）。
void audio_play(i2c_bus_handle_t bus, Sound sound);

// 鳴っている音が終わるのを待ってから、スピーカーと音声チップの電源を切る。画面を消すときに呼ぶ。
void audio_shutdown();

}  // namespace watch
