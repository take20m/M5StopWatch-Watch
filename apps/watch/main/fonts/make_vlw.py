"""TTF から LovyanGFX の VLW（なめらかフォント）を作り、C++ ヘッダに書き出す。

使い方: python make_vlw.py <出力ヘッダ> <名前>:<ttf>:<wght>:<px>:<文字列> ...
"""
import struct
import sys

from PIL import Image, ImageDraw, ImageFont


def make_font(ttf, weight, px, chars):
    font = ImageFont.truetype(ttf, px)
    try:
        axes = font.get_variation_axes()
        values = []
        for axis in axes:
            name = axis.get("name", b"")
            name = name.decode() if isinstance(name, bytes) else str(name)
            if name.lower().startswith("weight") or name.lower() == "wght":
                values.append(weight)
            elif name.lower().startswith("optical"):
                values.append(min(max(px, axis["minimum"]), axis["maximum"]))
            else:
                values.append(axis["default"])
        font.set_variation_by_axes(values)
    except OSError:
        pass  # 可変フォントでない

    glyphs = []
    ascent, descent = font.getmetrics()
    for ch in sorted(set(chars), key=ord):
        x0, y0, x1, y1 = font.getbbox(ch, anchor="ls")
        w, h = max(0, x1 - x0), max(0, y1 - y0)
        adv = int(round(font.getlength(ch)))
        bitmap = b""
        if w and h:
            img = Image.new("L", (w, h), 0)
            ImageDraw.Draw(img).text((-x0, -y0), ch, font=font, fill=255, anchor="ls")
            bitmap = img.tobytes()
        else:
            # 空白など絵のない文字は幅も高さも 0 にする。LovyanGFX は高さ 0 でも 1 行描こうとして、
            # 幅があると中身の決まっていないメモリを点として描いてしまう（2026-11-18 写真 IMG_0105 の点々）。
            w = h = x0 = y0 = 0
        assert w < 256 and adv < 256, (ch, w, adv)
        glyphs.append((ord(ch), h, w, adv, -y0, x0, bitmap))

    out = struct.pack(">6i", len(glyphs), 11, px, 0, ascent, descent)
    for code, h, w, adv, dy, dx, _ in glyphs:
        out += struct.pack(">7i", code, h, w, adv, dy, dx, 0)
    for *_, bitmap in glyphs:
        out += bitmap
    return out


def main():
    header, specs = sys.argv[1], sys.argv[2:]
    lines = [
        "// 自動生成（make_vlw.py）。LovyanGFX の loadFont() に渡す VLW フォント。",
        "// Inter / Noto Sans JP（SIL Open Font License 1.1。fonts/ の OFL_*.txt を参照）から必要な文字だけを抜き出した。",
        "#pragma once",
        "#include <cstdint>",
        "",
    ]
    for spec in specs:
        name, ttf, weight, px, chars = spec.split(":", 4)
        data = make_font(ttf, float(weight), int(px), chars)
        lines.append(f"// {name}: {ttf.split('/')[-1]} wght={weight} {px}px 文字={chars!r} ({len(data)} bytes)")
        lines.append(f"inline constexpr std::uint8_t {name}[] = {{")
        for i in range(0, len(data), 24):
            lines.append("    " + ", ".join(f"0x{b:02X}" for b in data[i : i + 24]) + ",")
        lines.append("};")
        lines.append("")
        print(name, len(data))
    with open(header, "w", encoding="utf-8") as f:
        f.write("\n".join(lines))


if __name__ == "__main__":
    main()
