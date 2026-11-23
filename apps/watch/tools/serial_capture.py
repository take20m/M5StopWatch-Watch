"""ESP32-S3 (USB-Serial/JTAG) をリセットして、指定秒数ぶんシリアル出力を読む。
使い方: python serial_capture.py <port> <seconds> [--no-reset] [--send-time]
--send-time: デバイスが "TIME_REQ" を出したら、PC の現在時刻を "T=<UNIX 秒>" で送り返す（apps/watch の時刻合わせ）。
"""
import sys
import time

import serial

port, seconds = sys.argv[1], float(sys.argv[2])
do_reset = "--no-reset" not in sys.argv
send_time = "--send-time" in sys.argv
deadline = time.time() + seconds


def open_port():
    while time.time() < deadline:
        try:
            return serial.Serial(port, 115200, timeout=0.2)
        except (serial.SerialException, OSError):
            time.sleep(0.05)
    return None


s = open_port()
if s and do_reset:
    # esptool の hard reset と同じ手順（USB-Serial/JTAG は RTS でチップをリセットできる）
    s.setDTR(False)
    s.setRTS(True)
    time.sleep(0.1)
    s.setRTS(False)
    s.close()
    time.sleep(0.3)
    s = open_port()

last_data = time.time()
pending = ""


def stamp():
    t = time.time()
    return time.strftime("[%H:%M:%S", time.localtime(t)) + ".%03d] " % int((t % 1) * 1000)


while time.time() < deadline:
    if s is None:
        s = open_port()
        continue
    try:
        data = s.read(4096)
        if data:
            last_data = time.time()
            # 行ごとに PC 側の時刻を付ける（デバイスの時刻は起床のたびに 0 に戻るため）
            pending += data.decode("utf-8", "replace")
            *lines, pending = pending.split("\n")
            for line in lines:
                sys.stdout.write(stamp() + line + "\n")
                if send_time and "TIME_REQ" in line:
                    s.write(("T=%d\n" % round(time.time())).encode())
                    s.flush()
                    sys.stdout.write(stamp() + "[capture] sent time\n")
            sys.stdout.flush()
        elif time.time() - last_data > 1.0:
            # macOS では USB が切れても例外にならず空読みが続くことがあるので、定期的に開き直す
            s.close()
            s = None
            last_data = time.time()
    except (serial.SerialException, OSError):
        # ディープスリープやリセットで USB が切れたら、つながり直すまで待つ
        try:
            s.close()
        except Exception:
            pass
        print("\n" + stamp() + "[capture] port disconnected", flush=True)
        s = open_port()
        if s:
            print("[capture] port reconnected", flush=True)
print("\n[capture] done", flush=True)
