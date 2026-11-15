"""キャラクターの PNG をまとめて、腕時計に埋め込む 1 つのバイナリにする。

使い方: python make_character.py <出力.bin> <大きさ(px)> <png> [<png> ...]
背景が透明な部分は黒（文字盤の背景）と混ぜてから RGB565 にする。
黒い背景だとキャラクターの暗い輪郭線が沈むので、シールのような白いふち（BORDER px）を付ける。
出力: "CHR1" + 枚数(u16) + 幅(u16) + 高さ(u16) + 予備(u16)、続けて 1 枚ずつ RGB565（リトルエンディアン）。
画像は著作物なので、出力（main/assets/）は git に入れない。
"""
import struct
import sys

from PIL import Image, ImageFilter

BORDER = 3


def convert(path, size):
    im = Image.open(path).convert("RGBA")
    inner = size - 2 * BORDER
    im.thumbnail((inner, inner), Image.LANCZOS)
    layer = Image.new("RGBA", (size, size), (0, 0, 0, 0))
    layer.alpha_composite(im, ((size - im.width) // 2, (size - im.height) // 2))
    # 白いふち: 形（不透明な部分）を BORDER px 太らせて白で塗り、少しぼかして角をなめらかにする
    shape = layer.getchannel("A").point(lambda v: 255 if v > 40 else 0)
    shape = shape.filter(ImageFilter.MaxFilter(2 * BORDER + 1)).filter(ImageFilter.GaussianBlur(0.8))
    halo = Image.new("RGBA", (size, size), (255, 255, 255, 0))
    halo.putalpha(shape)
    canvas = Image.new("RGBA", (size, size), (0, 0, 0, 255))
    canvas.alpha_composite(halo)
    canvas.alpha_composite(layer)
    out = bytearray()
    raw = canvas.tobytes()
    for i in range(0, len(raw), 4):
        r, g, b = raw[i], raw[i + 1], raw[i + 2]
        out += struct.pack("<H", ((r >> 3) << 11) | ((g >> 2) << 5) | (b >> 3))
    return bytes(out)


def main():
    out_path, size, pngs = sys.argv[1], int(sys.argv[2]), sys.argv[3:]
    data = b"CHR1" + struct.pack("<4H", len(pngs), size, size, 0)
    for p in pngs:
        data += convert(p, size)
    with open(out_path, "wb") as f:
        f.write(data)
    print(f"{len(pngs)} frames {size}px -> {len(data)} bytes")


if __name__ == "__main__":
    main()
