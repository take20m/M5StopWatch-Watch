"""丸いアイコンの画像を、腕時計に埋め込む 1 つのバイナリにする（QR コードの真ん中に出す）。

使い方: python make_icon.py <入力画像> <出力.bin> <大きさ(px)>
白い背景の外側を切り落とし、白くない部分の範囲を中心にした正方形を丸く切り抜く。丸の外は白
（QR コードの地の色）にする。
出力: "ICN1" + 幅(u16) + 高さ(u16)、続けて RGB565（リトルエンディアン）。
画像はユーザーのものなので、出力（main/assets/）は git に入れない。
"""
import struct
import sys

from PIL import Image, ImageDraw


def main():
    src, out_path, size = sys.argv[1], sys.argv[2], int(sys.argv[3])
    im = Image.open(src).convert("RGB")
    # 白くない部分の範囲
    mask = im.point(lambda v: 255 if v < 200 else 0).convert("L")
    x0, y0, x1, y1 = mask.getbbox()
    side = max(x1 - x0, y1 - y0)
    cx, cy = (x0 + x1) // 2, (y0 + y1) // 2
    sq = Image.new("RGB", (side, side), (255, 255, 255))
    sq.paste(im.crop((cx - side // 2, cy - side // 2, cx - side // 2 + side, cy - side // 2 + side)))
    sq = sq.resize((size, size), Image.LANCZOS)
    circle = Image.new("L", (size * 4, size * 4), 0)
    ImageDraw.Draw(circle).ellipse((0, 0, size * 4 - 1, size * 4 - 1), fill=255)
    circle = circle.resize((size, size), Image.LANCZOS)
    out_im = Image.new("RGB", (size, size), (255, 255, 255))
    out_im.paste(sq, (0, 0), circle)
    out = bytearray(b"ICN1" + struct.pack("<HH", size, size))
    for r, g, b in out_im.getdata():
        out += struct.pack("<H", ((r >> 3) << 11) | ((g >> 2) << 5) | (b >> 3))
    open(out_path, "wb").write(out)
    print(f"{out_path}: {size}x{size} {len(out)} bytes")


if __name__ == "__main__":
    main()
