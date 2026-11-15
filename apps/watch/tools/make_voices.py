"""短い音声（wav など）をまとめて、腕時計に埋め込む 1 つのバイナリにする。

使い方: python make_voices.py <出力.bin> <音声> [<音声> ...]
ffmpeg で 16kHz・モノラル・16 ビットにし、音量の最大を -1dB にそろえる。
出力: "SND1" + 個数(u16) + サンプルレート(u16)、続けて 個数 ×（開始位置 u32, サンプル数 u32）、最後に PCM（int16 LE）。
声は著作物なので、出力（main/assets/）は git に入れない。
"""
import struct
import subprocess
import sys
from array import array

RATE = 16000


def load(path):
    raw = subprocess.run(
        ["ffmpeg", "-loglevel", "error", "-i", path, "-ac", "1", "-ar", str(RATE), "-f", "s16le", "-"],
        check=True, capture_output=True,
    ).stdout
    pcm = array("h", raw)
    peak = max(1, max(abs(v) for v in pcm))
    gain = 0.89 * 32767 / peak
    return array("h", (max(-32768, min(32767, int(v * gain))) for v in pcm))


def main():
    out_path, paths = sys.argv[1], sys.argv[2:]
    clips = [load(p) for p in paths]
    header = b"SND1" + struct.pack("<2H", len(clips), RATE)
    offset, table = 0, b""
    for c in clips:
        table += struct.pack("<2I", offset, len(c))
        offset += len(c)
    body = b"".join(c.tobytes() for c in clips)
    with open(out_path, "wb") as f:
        f.write(header + table + body)
    print(f"{len(clips)} clips, {offset / RATE:.2f}s, {len(header) + len(table) + len(body)} bytes")


if __name__ == "__main__":
    main()
