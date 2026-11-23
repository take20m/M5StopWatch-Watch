// =============================================================
//  M5Stack StopWatch  背面プレート(ベルト取付用)
//  - 背面のメス拡張バス(2.54mm 7P + 6P)にオスのピンヘッダを挿して位置決め/回転止め
//  - 外周クリップで本体のフチを抱えて、引っ張りの力はクリップで受ける
//  - 両端にバネ棒用ラグ → 市販の「Apple Watchバンド変換アダプタ」を付ける
//
//  ★ = 実機で測って入れてほしい値(初期値は仮)
//  単位: mm / 座標: プレート中心が原点、+Z が本体側、-Z が手首側
// =============================================================

/* [本体] */
body_d        = 52.0;   // 本体外径(公式 52.0)
body_clear    = 0.3;    // 本体とクリップ内側のすき間(きつければ増やす)

/* [プレート] */
plate_t       = 2.0;    // プレート厚
plate_d       = 50.0;   // プレート外径(本体より少し小さめ)

/* [ピンヘッダ(★要計測)] */
// 背面ソケットの位置。2列(7Pと6P)が平行に並んでいる想定。
// 列は Y 方向に伸び、X 方向に hdr_sep 離れている。
hdr_sep       = 37.0;   // ★(写真から推定) 2列の中心間距離(X方向)
hdr_off_x     = 0.0;    // ★ 2列の中点の、本体中心からのXずれ
hdr_off_y     = 0.0;    // ★ 同 Yずれ
hdr_rot       = 0;      // ★ ソケット列の向き(度)。ラグの向きと合わせる
row1_pins     = 7;      // 列1に挿すピン数(ソケット数以下ならOK。端だけでも可)
row2_pins     = 6;      // 列2に挿すピン数
row1_shift    = 0;      // 列1のピン位置ずらし(ピン数を減らした時の調整, ピッチ単位)
row2_shift    = 0;      // 列2
pitch         = 2.54;
hdr_body_w    = 2.54;   // ピンヘッダ樹脂部の幅
hdr_body_h    = 2.5;    // ピンヘッダ樹脂部の高さ
hdr_fit       = 0.15;   // 圧入クリアランス(緩ければ減らす)
hdr_protrude  = 0.5;    // ★ 樹脂部がプレート上面から出る量
                        //   = 本体背面とプレートのすき間。ソケットが凹んでいれば増やす

/* [筐体ネジ頭の逃がし] */
screw_x       = 21.5;   // ★ ネジ中心の X 位置(±)。写真から推定
screw_head_d  = 6.0;    // 逃がし穴の径

/* [外周クリップ] */
clip_angles   = [45, 135, 225, 315]; // ★ USB-C・ボタン・Grove を避ける角度に
clip_arc      = 20;     // クリップ1個の幅(度)
clip_t        = 1.6;    // クリップ厚
clip_h        = 4.0;    // ★ 本体側面をどこまで抱えるか(プレート上面から)
lip_depth     = 0.0;    // ★ クリップ先端の内向きツメ(本体に段や面取りがあれば 0.4〜0.8)
lip_h         = 1.0;    //   ツメの高さ

/* [ラグ] */
lug_w         = 22.0;   // バネ棒の長さ(変換アダプタの規格 20 / 22 / 24)
lug_gap_fit   = 0.3;    // ラグ間すき間の余裕
lug_ear_t     = 2.5;    // ラグ耳の厚み
lug_len       = 6.0;    // プレート外周からラグ先端までの長さ
lug_hole_from_tip = 2.2; // ラグ先端からバネ棒穴中心まで
lug_ear_h     = 4.5;    // ラグ耳の高さ(Z)
lug_z0        = -1.5;   // ラグ耳の下端Z(手首側に少し出す)
bar_hole_d    = 1.1;    // バネ棒の穴径(先端径0.8〜1.0用)

$fn = 96;

// ---------------- 本体 ----------------
module plate() {
    translate([0,0,-plate_t]) cylinder(d = plate_d, h = plate_t);
}

module clips() {
    r_in = body_d/2 + body_clear;
    for (a = clip_angles) rotate([0,0,a - clip_arc/2]) {
        // 外周リブ(プレートから本体側面に沿って立ち上がる)
        translate([0,0,-plate_t])
            rotate_extrude(angle = clip_arc)
                translate([r_in, 0]) square([clip_t, plate_t + clip_h]);
        // プレートとクリップをつなぐ板
        translate([0,0,-plate_t])
            rotate_extrude(angle = clip_arc)
                translate([plate_d/2 - 1, 0]) square([r_in - plate_d/2 + 1.01, plate_t]);
        // 先端の内向きツメ
        if (lip_depth > 0)
            translate([0,0,clip_h - lip_h])
                rotate_extrude(angle = clip_arc)
                    polygon([[r_in, 0], [r_in, lip_h], [r_in - lip_depth, lip_h]]);
    }
}

module lug_pair(side) { // side = +1 / -1 (Y方向)
    gap   = lug_w + lug_gap_fit;
    y0    = side * (plate_d/2 - 3);            // プレートにめり込ませる
    y1    = side * (body_d/2 + body_clear + lug_len);
    yh    = side * (body_d/2 + body_clear + lug_len - lug_hole_from_tip);
    difference() {
        union() {
            for (sx = [-1, 1]) {
                xc = sx * (gap/2 + lug_ear_t/2);
                hull() {
                    translate([xc, y0, lug_z0 + lug_ear_h/2])
                        cube([lug_ear_t, 0.01, lug_ear_h], center = true);
                    translate([xc, yh, lug_z0 + lug_ear_h/2])
                        rotate([0,90,0]) cylinder(d = lug_ear_h, h = lug_ear_t, center = true);
                }
            }
            // 耳どうしをつなぐ橋(プレート厚の範囲)
            hull() {
                translate([0, y0, -plate_t/2]) cube([gap + 2*lug_ear_t, 0.01, plate_t], center = true);
                translate([0, side*(body_d/2 + body_clear + 1.0), -plate_t/2])
                    cube([gap + 2*lug_ear_t, 0.01, plate_t], center = true);
            }
        }
        // バネ棒穴(貫通)
        translate([0, yh, lug_z0 + lug_ear_h/2])
            rotate([0,90,0]) cylinder(d = bar_hole_d, h = gap + 2*lug_ear_t + 2, center = true, $fn = 24);
    }
}

// ピンヘッダ樹脂部を受けるポケット(貫通。下の短いピンは切って平らにする)
module header_pockets() {
    rows = [[-hdr_sep/2, row1_pins, row1_shift], [hdr_sep/2, row2_pins, row2_shift]];
    translate([hdr_off_x, hdr_off_y, 0]) rotate([0,0,hdr_rot])
        for (r = rows) if (r[1] > 0) {
            len = r[1] * pitch;
            translate([r[0], r[2]*pitch, -plate_t/2])
                cube([hdr_body_w + hdr_fit*2, len + hdr_fit*2, 20], center = true);
        }
}

// ピンヘッダ樹脂部を hdr_protrude 分だけ持ち上げる台座
// (樹脂部 2.5mm を、プレート厚 + 出っ張り量に合わせる)
module header_bosses() {
    boss_h = hdr_body_h - hdr_protrude - plate_t; // プレート下(手首側)に出る分
    rows = [[-hdr_sep/2, row1_pins, row1_shift], [hdr_sep/2, row2_pins, row2_shift]];
    if (boss_h > 0)
        translate([hdr_off_x, hdr_off_y, 0]) rotate([0,0,hdr_rot])
            for (r = rows) if (r[1] > 0) {
                len = r[1] * pitch;
                translate([r[0], r[2]*pitch, -plate_t - boss_h/2])
                    cube([hdr_body_w + 3, len + 3, boss_h], center = true);
            }
}

difference() {
    union() {
        plate();
        clips();
        lug_pair(1);
        lug_pair(-1);
        header_bosses();
    }
    header_pockets();
    for (sx = [-1, 1]) translate([sx*screw_x, 0, -10]) cylinder(d = screw_head_d, h = 20);
}

// 確認用:本体のゴースト(F5 プレビューのみ表示)
%translate([0, 0, hdr_protrude]) cylinder(d = body_d, h = 15.5);
