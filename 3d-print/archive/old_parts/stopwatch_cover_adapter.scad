// =============================================================
//  M5Stack StopWatch  背面カバー型バンドアダプタ(薄型・22mm バネ棒)
//  - 背面のピンソケット(7P / 6P)を覆って汗が入りにくくする
//  - 手首側は全面フラットで厚み plate_t(初期 2.0mm)だけ
//  - ネジは M2 皿ネジにして頭を板の中に沈める(厚い座をなくすため)
//  - ラグの耳は本体の外周より外で「本体側」に立ち上げ、手首側に出さない
//  - 背面の凹み(12時 逆三角形 / 6時 段)と、筐体ネジ穴の ø4 ザグリにはまるキー付き
//
//  背面形状は公式構造データ StopWatch.stl から読み取り:
//    外周リング面 = 0 / 中央の面 0.2 凹み / 12時 逆三角形 0.8 凹み / 6時 段 1.5〜1.7 凹み
//    ネジ穴 2-ø2.2 @ X±20 (周りに ø4 × 深さ2.0 のザグリ) / スピーカー穴 Y12.8〜18.8, X±3.2
//    ピンソケット: X +15.6〜+17.2, Y -6.0〜+9.2(6P) / X -17.2〜-15.6, Y -9.2〜+9.2(7P)
//
//  座標: 原点 = 背面の外周リング面の中心、+Z = 本体側、-Z = 手首側、+Y = 12時
//  印刷向き: 手首側(Z = -plate_t の平らな面)をベッドに。サポート不要
//
//  ★ ネジ: M2 皿小ネジ。長さ = 元のキャップネジの長さ + 4mm(例: M2x6 → 皿 M2x10)
//          (板 2mm + 筐体の ø4 ザグリ 2mm ぶん。先端の位置は元のネジと同じになる)
// =============================================================

/* [基本] */
body_d        = 52.0;
plate_t       = 2.0;    // 手首側に出る厚み(全体)
edge_round    = 0.8;    // 手首側の縁の丸み(R)

/* [ネジ M2 皿] */
screw_x       = 20.0;
screw_hole_d  = 2.3;
csk_d         = 4.2;    // 皿の上端径(M2 皿ネジ頭径 3.5〜4.0 + 余裕)
boss_d        = 8.0;
tube_on       = true;   // 筐体の ø4 ザグリに差し込む筒(位置決め + 締め付けの受け)
tube_od       = 3.6;
tube_h        = 1.8;    // ザグリ深さ 2.0 より少し短く

/* [カバー範囲(ピンソケットを覆う)] */
cover_x       = 19.0;   // ±X
cover_y       = 11.0;   // ±Y
cover_r       = 3.0;

/* [凹みにはまるキー] */
key_on        = true;
key_clear     = 0.25;
key_clear_z   = 0.2;
tri_depth     = 0.8;
step_depth    = 1.5;
step_y        = -19.4;
step_r        = 25.0;

/* [スピーカー穴] */
spk_y         = 15.8;
spk_d         = 9.0;

/* [ラグ] */
band_w        = 22.0;
lug_gap_fit   = 0.3;
ear_t         = 2.5;
// --- バネ棒の位置は「バンド端の太さ」から自動計算して、本体ぎりぎりまで寄せる ---
band_end_t    = 5.0;    // ★ バンドのバネ棒が通る筒部分の太さ(実測して入れる)
band_clear    = 0.2;    // バンド端と本体のすき間
body_fillet   = 2.0;    // 本体背面の縁の丸み R2(公式寸法図)
bar_hole_d    = 1.1;
ear_wall      = 1.1;    // バネ棒穴まわりの肉
// 穴の下に ear_wall の肉を残せる一番低い位置(低いほど本体の丸みの下にもぐって寄せられる)
hole_z        = -plate_t + ear_wall + bar_hole_d/2;
ear_r         = bar_hole_d/2 + ear_wall;
_rb  = band_end_t/2 + band_clear;
_R   = body_d/2;
_F   = body_fillet;
// 本体の縁の丸み(中心 (R-F, F)、半径 F)と、その上の垂直な側面の両方に当たらない最小の Y
_c1  = (_R - _F) + sqrt(max(0, pow(_rb + _F, 2) - pow(hole_z - _F, 2)));
_c2  = (hole_z + _rb > _F) ? _R + sqrt(max(0, pow(_rb, 2) - pow(_F - hole_z, 2))) : 0;
lug_pull      = 0.8;    // ★ 計算位置からさらに本体側へ寄せる量(上下とも)。0 で「付け根が当たらない」位置
hole_y        = max(_c1, _c2) - lug_pull;    // バネ棒中心 Y
arm_end_y     = hole_y - _rb;                // 腕の先端(バンド端が回る空間の手前)
// この位置で本体の縁(R2)に当たらない付け根の太さの上限
_max_t = 2*(sqrt(pow(hole_y - (_R - _F), 2) + pow(hole_z - _F, 2)) - _F - band_clear);
echo(str("バネ棒中心 Y = ", hole_y, " / 耳の先端 Y = ", hole_y + ear_r, " / 本体の縁から ", hole_y + ear_r - _R, " mm"));
echo(str("本体の縁に当たらないバンド付け根の太さ: ", _max_t, " mm 以下"));
arm_w_center  = 14.0;
bar_w         = 9.02;

$fn = 64;

gap   = band_w + lug_gap_fit;
lug_W = gap + 2*ear_t;

// ---------- 2D ----------
module cover2d() { offset(r = cover_r) square([2*(cover_x - cover_r), 2*(cover_y - cover_r)], center = true); }
module bar2d()   { hull() for (sx = [-1,1]) translate([sx*screw_x, 0]) circle(d = bar_w); }
module boss2d()  { for (sx = [-1,1]) translate([sx*screw_x, 0]) circle(d = boss_d); }
module arm2d(side) {
    hull() {
        circle(d = arm_w_center);
        translate([-lug_W/2, side > 0 ? arm_end_y - 0.5 : -arm_end_y]) square([lug_W, 0.5]);
    }
}
module plate2d() { union() { cover2d(); bar2d(); boss2d(); arm2d(1); arm2d(-1); } }

tri_pts = [[10.2,0.01],[10.4,1.6],[10.8,2.4],[11.2,3.2],[12.0,4.0],[13.2,4.8],
           [14.8,6.0],[16.4,6.8],[18.4,8.0],[20.4,9.2],[22.0,10.0],[23.0,11.0],[26,13]];
module tri2d() {
    intersection() {
        polygon(concat([for (p = tri_pts) [ p[1], p[0]]],
                       [for (i = [len(tri_pts)-1:-1:0]) [-tri_pts[i][1], tri_pts[i][0]]]));
        circle(r = 24.0);
    }
}
module step2d() { intersection() { circle(r = step_r); translate([-30,-30]) square([60, 30 + step_y]); } }

// ---------- 3D ----------
// 手首側の縁を丸めた板
// 下半分は 45° の面取り、上半分は R の丸み(ベッド側で 45° を超えるオーバーハングを作らない)
// 凸な部品ごとに断面の層を hull でつないで滑らかにし、最後に union する
module rounded_piece() {
    R = edge_round; n = 6;
    hull() {
        // ベッド面(45° の延長線)
        linear_extrude(0.02) children_inset(R * (2 - 2*sin(45))) children();
        for (k = [0:n]) {
            a = 45 + 45 * k / n;
            translate([0, 0, R - R*cos(a)])
                linear_extrude(0.02) offset(delta = -(R - R*sin(a))) children();
        }
        translate([0, 0, plate_t - 0.02]) linear_extrude(0.02) children();
    }
}
module children_inset(d) { offset(delta = -d) children(); }

module plate3d() { translate([0,0,-plate_t]) plate3d_raw(); }
module plate3d_raw() {
    rounded_piece() cover2d();
    rounded_piece() bar2d();
    for (sx = [-1,1]) rounded_piece() translate([sx*screw_x, 0]) circle(d = boss_d);
    for (s = [-1,1]) rounded_piece() arm2d(s);
}

module ears(side) {
    for (sx = [-1,1]) {
        xc = sx * (gap/2 + ear_t/2);
        hull() {
            translate([xc, side*(arm_end_y - 1), -plate_t/2]) cube([ear_t, 2, plate_t], center = true);
            translate([xc, side*hole_y, hole_z]) rotate([0,90,0]) cylinder(r = ear_r, h = ear_t, center = true);
            // 手首側は板と同じ面で止める
            translate([xc, side*hole_y, -plate_t + 0.5]) cube([ear_t, 2*ear_r, 1], center = true);
        }
    }
}

module keys() {
    if (key_on) {
        linear_extrude(tri_depth - key_clear_z)
            intersection() { offset(delta = -key_clear) tri2d(); plate2d(); }
        linear_extrude(step_depth - key_clear_z)
            intersection() { offset(delta = -key_clear) step2d(); plate2d(); }
    }
    if (tube_on)
        for (sx = [-1,1]) translate([sx*screw_x, 0, 0]) cylinder(d = tube_od, h = tube_h);
}

difference() {
    union() {
        plate3d();
        for (s = [-1,1]) ears(s);
        keys();
    }
    // 皿ネジ穴
    for (sx = [-1,1]) translate([sx*screw_x, 0, 0]) {
        translate([0,0,-20]) cylinder(d = screw_hole_d, h = 40);
        translate([0,0,-plate_t - 0.01])
            cylinder(d1 = csk_d, d2 = screw_hole_d, h = (csk_d - screw_hole_d)/2);
        translate([0,0,-plate_t - 1]) cylinder(d = csk_d, h = 1);
    }
    // スピーカー穴
    if (spk_d > 0) translate([0, spk_y, -20]) cylinder(d = spk_d, h = 40);
    // バネ棒穴
    for (s = [-1,1]) translate([0, s*hole_y, hole_z]) rotate([0,90,0])
        cylinder(d = bar_hole_d, h = lug_W + 2, center = true, $fn = 24);
    // 耳の内側(バンドの付け根が回る空間)を確実にあける
    for (s = [-1,1]) translate([-gap/2, s > 0 ? arm_end_y : -arm_end_y - 10, -10]) cube([gap, 10, 20]);
    // 手首側の面より下には何も出さない
    translate([-50,-50,-plate_t - 10]) cube([100,100,10]);
}

%cylinder(d = body_d, h = 15.5);
