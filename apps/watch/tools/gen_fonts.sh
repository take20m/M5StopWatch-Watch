#!/bin/zsh
# watch_fonts.h を作り直す。使い方: gen_fonts.sh <Pillow の入った python> <Inter.ttf>
# Inter は Google Fonts の Inter[opsz,wght].ttf（OFL 1.1）。
set -e
PY=$1; T=$2
cd ${0:A:h}/../main
L='ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz'
$PY fonts/make_vlw.py watch_fonts.h \
  "kFontTime:$T:600:120:0123456789:-" \
  "kFontSec:$T:500:40:0123456789.:" \
  "kFontSmall:$T:600:34:0123456789/ $L" \
  "kFontHint:$T:500:20:${L}0123456789/:+-. ·" \
  "kFontPill:$T:600:24:$L 0123456789%-.:/…" \
  "kFontBatteryPct:$T:600:26:0123456789%" \
  "kFontQrUrl:$T:600:30:take20m.dv"
