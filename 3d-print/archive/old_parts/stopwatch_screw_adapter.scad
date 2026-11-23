// =============================================================
//  M5Stack StopWatch  ネジ止めバンドアダプタ(22mm バネ棒バンド用)
//  - 背面 3時/9時 の筐体ネジ(M2 六角穴付き)を長いものに替えて共締め
//  - 12時/6時 にバネ棒用ラグ
//  - 背面の凹み(12時:スピーカーの逆三角形 / 6時:下部の段)にはまる「キー」付き
//
//  背面形状は M5Stack 公式の構造データ
//  (M5_Hardware/Products/C152_StopWatch/Structures/StopWatch.stl)から読み取り:
//    - 外周リング面 = 基準(0)
//    - 中央の面        : 0.2mm 凹み
//    - 12時 逆三角形   : 0.8mm 凹み(頂点 Y≈10.2、Y=22 で幅 ±10、上端は R≈24 の円)
//    - 6時 下部の段    : 1.5〜1.7mm 凹み(Y≈-19.4 より外側、縁まで抜けている)
//    - スピーカー穴    : Y 12.8〜18.8 / X ±3.2
//    - ネジ穴 2-ø2.2   : X ±20, Y 0
//
//  座標: 原点 = 本体背面(外周リング面)の中心、+Z = 本体側、-Z = 手首側、+Y = 12時
//  印刷向き: 手首側(Z = -plate_t の平らな面)をベッドに置く。サポート不要
// =============================================================

/* [本体] */
body_d        = 52.0;

/* [筐体ネジ M2] */
screw_x       = 20.0;   // 公式: ネジ間隔 40mm
screw_y       = 0.0;
screw_hole_d  = 2.3;    // M2 通し穴
head_d        = 3.8;    // ネジ頭の径(M2キャップ 3.8 / ボタン 3.5)
head_h        = 2.0;    // ネジ頭の高さ(M2キャップ 2.0 / ボタン 1.1 / 低頭 1.3)
screw_extra   = 2.0;    // 元のネジより何mm長いネジを使うか(= ネジ頭の下の肉厚)

/* [プレート] */
bar_w         = 9.0;    // 3時-9時 の梁の幅
arm_w_center  = 14.0;   // 腕の中央付近の幅
boss_d        = 8.0;    // ネジ周りの座の径
// 手首側を平らにして印刷しやすくするため、全体をネジ座と同じ厚みにする
plate_t       = screw_extra + head_h + 0.2;   // = 4.2

/* [凹みにはまるキー] */
key_on        = true;
key_clear     = 0.25;   // キー外形を凹みより小さくする量(横方向)
key_clear_z   = 0.2;    // キー高さを凹みより低くする量
tri_depth     = 0.8;    // 12時 逆三角形の深さ
step_depth    = 1.5;    // 6時 段の深さ(浅いほうの値)
step_y        = -19.4;  // 6時 段が始まる Y(この内側は斜面)
step_r        = 25.0;   // 段キーの外周(本体縁の丸みを避ける)

/* [スピーカー穴] */
spk_y         = 15.8;   // 穴中心の Y(公式データ: 穴は Y 12.8〜18.8)
spk_d         = 9.0;    // 0 で穴なし

/* [ラグ(バネ棒)] */
band_w        = 22.0;   // バンド幅 = バネ棒長(20 / 22)
lug_gap_fit   = 0.3;
ear_t         = 2.5;
hole_y        = 29.0;   // バネ棒中心の Y(本体半径 26 の 3mm 外)
hole_z        = -plate_t/2;   // バネ棒中心の Z(板厚の中央)
ear_d         = plate_t;      // 耳先端の丸み(手首側を平らに保つ)
bar_hole_d    = 1.1;
arm_end_y     = 25.5;

$fn = 64;

gap   = band_w + lug_gap_fit;
lug_W = gap + 2*ear_t;

// ---------- 2D 外形 ----------
module bar2d()  { hull() for (sx = [-1,1]) translate([sx*screw_x, screw_y]) circle(d = bar_w); }
module boss2d() { for (sx = [-1,1]) translate([sx*screw_x, screw_y]) circle(d = boss_d); }
module arm2d(side) {
    hull() {
        circle(d = arm_w_center);
        translate([-lug_W/2, side > 0 ? arm_end_y - 0.5 : -arm_end_y]) square([lug_W, 0.5]);
    }
}
module plate2d() { union() { bar2d(); boss2d(); arm2d(1); arm2d(-1); } }

// 12時 逆三角形の凹み(公式データの半幅を Y ごとに読み取ったもの)
tri_pts = [[10.2,0.01],[10.4,1.6],[10.8,2.4],[11.2,3.2],[12.0,4.0],[13.2,4.8],
           [14.8,6.0],[16.4,6.8],[18.4,8.0],[20.4,9.2],[22.0,10.0],[23.0,11.0],[26,13]];
module tri2d() {
    intersection() {
        polygon(concat([for (p = tri_pts) [ p[1], p[0]]],
                       [for (i = [len(tri_pts)-1:-1:0]) [-tri_pts[i][1], tri_pts[i][0]]]));
        circle(r = 24.0);
    }
}
// 6時 下部の段
module step2d() {
    intersection() {
        circle(r = step_r);
        translate([-30, -30]) square([60, 30 + step_y]);
    }
}

// ---------- ラグの耳 ----------
module ears(side) {
    for (sx = [-1,1]) {
        xc = sx * (gap/2 + ear_t/2);
        hull() {
            translate([xc, side*(arm_end_y - 0.5), -plate_t/2]) cube([ear_t, 1, plate_t], center = true);
            translate([xc, side*hole_y, hole_z]) rotate([0,90,0]) cylinder(d = ear_d, h = ear_t, center = true);
        }
    }
}

// ---------- キー(本体側に出る) ----------
module keys() {
    if (key_on) {
        // 12時
        linear_extrude(tri_depth - key_clear_z)
            intersection() { offset(delta = -key_clear) tri2d(); plate2d(); }
        // 6時
        linear_extrude(step_depth - key_clear_z)
            intersection() { offset(delta = -key_clear) step2d(); plate2d(); }
    }
}

difference() {
    union() {
        translate([0,0,-plate_t]) linear_extrude(plate_t) plate2d();
        for (s = [-1,1]) ears(s);
        keys();
    }
    // ネジ通し穴 + ザグリ(手首側)
    for (sx = [-1,1]) translate([sx*screw_x, screw_y, 0]) {
        translate([0,0,-20]) cylinder(d = screw_hole_d, h = 40);
        translate([0,0,-plate_t - 1]) cylinder(d = head_d + 0.6, h = head_h + 0.2 + 1);
    }
    // スピーカー穴
    if (spk_d > 0) translate([0, spk_y, -20]) cylinder(d = spk_d, h = 40);
    // バネ棒穴
    for (s = [-1,1]) translate([0, s*hole_y, hole_z]) rotate([0,90,0])
        cylinder(d = bar_hole_d, h = lug_W + 2, center = true, $fn = 24);
    // 耳は本体側(Z>0)に出さない
    for (s = [-1,1]) translate([-lug_W/2 - 1, s > 0 ? arm_end_y : -arm_end_y - 10, 0]) cube([lug_W + 2, 10, 5]);
}

// 確認用ゴースト(F5 のみ)
%cylinder(d = body_d, h = 15.5);
