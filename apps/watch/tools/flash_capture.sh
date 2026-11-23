#!/bin/zsh
# 使い方: tools/flash_capture.sh <ログのファイル>
# 腕時計が起きて USB に出てきたら書き込み、その後はログを取り続ける（寝て USB が切れても待ち直す）。
# ESP-IDF を有効にしたシェル（. $IDF_PATH/export.sh の後）で、apps/watch から実行する。
# ログを読むには pyserial が要る（ESP-IDF の Python 環境には入っている）。
log=${1:?ログのファイルを指定してください}
here=${0:A:h}
cd "$here/.." || exit 1
end=$(( $(date +%s) + 3600 )); ok=0
while [ $(date +%s) -lt $end ]; do
  port=$(ls /dev/cu.usbmodem* 2>/dev/null | head -1)
  if [ -n "$port" ]; then
    if idf.py -p "$port" flash >/dev/null 2>&1; then echo "flash ok at $(date +%T)"; ok=1; break; fi
  fi
  sleep 1
done
[ $ok = 1 ] || exit 1
end=$(( $(date +%s) + 72*3600 ))
while [ $(date +%s) -lt $end ]; do
  port=$(ls /dev/cu.usbmodem* 2>/dev/null | head -1)
  [ -n "$port" ] && python "$here/serial_capture.py" "$port" 3600 --no-reset --send-time >> "$log" 2>&1
  sleep 2
done
