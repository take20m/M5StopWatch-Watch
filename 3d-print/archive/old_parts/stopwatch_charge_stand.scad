// =============================================================
//  M5Stack StopWatch  USB-C 充電スタンド
//  - 手持ちの USB-C ケーブルのプラグを上向きに固定し、そこに StopWatch を 6時側から差し込んで立てる
//  - 本体は少し後ろに傾けて(tilt)、背面のアダプタ板を左右 2 本の柱で受ける
//  - 6時側のバンドは、2 本の柱の間を通って後ろに垂れる(バンドは外して/ほどいて置く)
//  - プラグは塔の後ろの入れ口から入れ、上のふちに押し当てて「裏ぶた」をパチッとはめて固定する
//    (裏ぶたの内側のリブがプラグを前の壁に押し付けるので、ずれない・落ちない)
//  - ケーブルは塔の後ろ側のスリットにはめ込み、台の中央の溝を通して後ろへ出す
//
//  ケーブル:Apple 純正 USB-C(樹脂部分 実測 幅10 × 厚さ4.5 × 長さ14.5)
//     om_len = 金属の付け根から、樹脂がケーブルの太さに細くなる手前まで
//
//  本体の寸法は公式データ:直径 52 / 厚さ 15.5 / USB-C は 6時の側面、背面からの高さ約 8.6
//  印刷向き:このまま(台の底がベッド)。サポート不要
//  part = "cover" で裏ぶた、"fit_test" で塔の上部だけのお試し部品を出力
// =============================================================

/* [出力する部品] */
part      = "stand";  // "stand" = スタンド本体 / "cover" = 裏ぶた / "fit_test" = 塔の上部だけのお試し部品

/* [ケーブルのプラグ:Apple 純正 USB-C(実測)] */
om_w      = 10.0;   // 樹脂部分の幅
om_t      = 4.5;    // 樹脂部分の厚み
om_len    = 14.5;   // 樹脂部分の長さ
om_clear  = 0.25;   // ポケットの左右のすき間(片側)
len_clear = 0.2;    // 長さ方向のすき間(ふちと底の間)
cable_w   = 5.0;    // ケーブルを通すスリットの幅(Apple 240W 編み込みケーブル 約4.6mm + 0.4)
// プラグの抜け止め:ポケットの上にふち(リップ)を付け、金属部分だけを通す窓を開ける
lip_t     = 0.4;    // ふちの厚み(この厚みぶん差し込みが浅くなる)
win_w     = 9.0;    // 金属部分を通す窓の幅(USB-C プラグ 8.25 + 余裕)
win_h     = 3.2;    // 窓の高さ(USB-C プラグ 2.4 + 余裕)

/* [裏ぶた(プラグを押さえる)] */
cov_gap   = 0.35;   // プラグの後ろと裏ぶたの間のすき間(リブで埋める)
rib_h     = 0.45;   // 裏ぶたの内側のリブの高さ(プラグを前の壁に押し付ける量 = rib_h - cov_gap)
cov_clear = 0.1;    // 裏ぶたと入れ口のすき間(片側)
snap_r    = 0.35;   // 裏ぶたの左右のポッチの半径(入れ口側のくぼみは +0.1)

/* [スタンド] */
tilt      = 15;     // 後ろへの傾き(度)
wall      = 2.0;    // ポケットまわりの壁
base_t    = 6.0;    // 台の厚み
base_w    = 64;     // 台の幅
base_front= 14;     // プラグより前に出す長さ
base_back = 8;      // 柱より後ろに出す長さ
base_r    = 6;      // 台の角の丸み

/* [背もたれの柱] */
post_x    = 16;     // 柱の左右位置(バンド幅 22 の外側)
pad_w     = 5;      // 当たり面の幅(左右)
pad_h     = 10;     // 当たり面の高さ
pad_y     = 4;      // 当たり面の中心(本体中心から12時方向へ)
pad_t     = 3;      // 当たり面の厚み
pad_gap   = 0.5;    // 当たり面と本体背面のすき間(挿さる深さは USB-C で決め、柱は倒れ止めだけ)

/* [本体] */
body_r    = 26.0;
body_th   = 15.5;
usb_z     = 8.6;    // USB-C の中心の高さ(本体背面から)
plate_t   = 2.0;    // 背面アダプタ板の厚み(柱はこの面に当たる)

$fn = 64;

// ---------- 本体座標 → スタンド座標 ----------
// 本体座標:X = 3時方向、Y = 12時方向、Z = 画面側(背面 = 0)
// スタンド座標:x = 左右、y = 奥(+)、z = 上
s = sin(tilt); c = cos(tilt);
// USB-C の差し込み口の中心(本体の Y = -body_r、Z = usb_z)を、スタンドの (0, 0, tower_top) に置く
tower_top = base_t + 3 + (om_len + len_clear + lip_t + 2) * c + 6;   // プラグの付け根の高さ(底にケーブルの曲がり代)
function W(p) = [ p[0],
                  s*(p[1] + body_r) - c*(p[2] - usb_z),
                  tower_top + c*(p[1] + body_r) + s*(p[2] - usb_z) ];
M = [[1, 0, 0, 0],
     [0, s, -c, s*body_r + c*usb_z],
     [0, c,  s, tower_top + c*body_r - s*usb_z],
     [0, 0, 0, 1]];
module in_watch() { multmatrix(M) children(); }

// ---------- 部品(本体座標で書く) ----------
pw     = om_w/2 + om_clear;                 // ポケットの半幅
pf     = usb_z + om_t/2 + 0.05;             // ポケットの前の面(プラグはここに押し付けられる → USB-C が中心に来る)
pb     = usb_z - om_t/2 - cov_gap;          // ポケットの後ろの面(= 裏ぶたの内側の面)
top_y  = -body_r;                           // 塔の上面(本体の側面と同じ高さ)
om_top = top_y - lip_t;                     // プラグの樹脂部分の上面(ふちの下面)
shelf  = om_top - om_len - len_clear;       // ポケットの底
module pocket_w() {   // 樹脂部分が入るポケット + 上のふちの窓 + 後ろの入れ口(裏ぶたで閉じる)
    translate([-pw, shelf, pb]) cube([2*pw, om_top - shelf, pf - pb]);
    translate([0, top_y + 1, usb_z]) rotate([90, 0, 0]) linear_extrude(lip_t + 2)
        hull() for (sx = [-1, 1]) translate([sx*(win_w/2 - win_h/2), 0]) circle(d = win_h);
    translate([-pw, shelf, -40]) cube([2*pw, om_top - shelf, pb + 40.01]);
    // 裏ぶたのポッチを受けるくぼみ
    for (sx = [-1, 1], f = [0.3, 0.7]) translate([sx*pw, shelf + f*(om_top - shelf), pb - wall/2])
        sphere(r = snap_r + 0.1, $fn = 24);
}
module tower_w() {    // ポケットを囲む塔
    translate([-pw - wall, shelf - wall, pb - wall]) cube([2*(pw + wall), top_y - (shelf - wall), (pf + wall) - (pb - wall)]);
}
module slot_w() {     // ケーブルのスリット(ポケットの底から下へ、後ろ側へ開放)
    // ケーブルはプラグの中心から真下に出るので、中心より前にもケーブルの半径ぶん広げる
    translate([-cable_w/2, shelf - 80, -60]) cube([cable_w, 80 + 0.01, usb_z + cable_w/2 + 60]);
}
module pad_w(sx) {    // 背もたれの当たり面(アダプタ板の手首側の面から pad_gap 離す)
    translate([sx*post_x - pad_w/2, pad_y - pad_h/2, -plate_t - pad_gap - pad_t]) cube([pad_w, pad_h, pad_t]);
}
// 裏ぶた:入れ口にはめて、内側のリブでプラグを前の壁に押し付ける。左右のポッチでパチッと止まる
module cover_w() {
    cw = pw - cov_clear;  h0 = shelf + cov_clear;  h1 = om_top - cov_clear;
    difference() {
        union() {
            translate([-cw, h0, pb - wall]) cube([2*cw, h1 - h0, wall]);
            // 内側のリブ(縦 3 本)
            for (x = [-3, 0, 3]) translate([x - 0.5, h0 + 1, pb - 0.01]) cube([1, h1 - h0 - 2, rib_h + 0.01]);
            // 左右のポッチ
            for (sx = [-1, 1], f = [0.3, 0.7]) translate([sx*cw, shelf + f*(om_top - shelf), pb - wall/2])
                sphere(r = snap_r, $fn = 24);
        }
        // 爪をかけて外す切り欠き(上の縁の外側を斜めに)
        translate([0, h1, pb - wall]) rotate([45, 0, 0]) cube([6, 1.2, 1.2], center = true);
        // ケーブルが通る切り欠き(下の縁の中央)
        translate([-cable_w/2, h0 - 1, pb - wall - 1]) cube([cable_w, 1 + 0.01, wall + 2]);
    }
}

// ---------- 台 ----------
tw_front = W([0, top_y, pf + wall])[1];                   // 塔の前面(上端)
pad_c    = W([post_x, pad_y, -plate_t - pad_gap - pad_t]);
base_y0  = min(tw_front, W([0, -body_r, body_th])[1]) - base_front;
base_y1  = pad_c[1] + base_back;

module base2d() {
    translate([-base_w/2, base_y0]) offset(r = base_r) offset(delta = -base_r)
        square([base_w, base_y1 - base_y0]);
}

module stand() {
    difference() {
        union() {
            linear_extrude(base_t) base2d();
            // 塔:本体座標の箱と、その真下の台をつなぐ
            hull() {
                in_watch() tower_w();
                p0 = W([0, shelf - wall, pb - wall]);
                p1 = W([0, shelf - wall, pf + wall]);
                translate([-pw - wall, min(p0[1], p1[1], tw_front) , 0])
                    cube([2*(pw + wall), max(p0[1], p1[1]) - min(p0[1], p1[1], tw_front), base_t]);
            }
            // 柱:当たり面と、その下の台をつなぐ
            for (sx = [-1, 1]) hull() {
                in_watch() pad_w(sx);
                q = W([sx*post_x, pad_y, -plate_t - pad_gap - pad_t]);
                translate([sx*post_x - pad_w/2, q[1] - 2, 0]) cube([pad_w, 8, base_t]);
            }
        }
        in_watch() pocket_w();
        in_watch() slot_w();
    }
}

// お試し部品:塔の上部(ポケット・ふちの窓・入れ口・ケーブルのスリット)だけを、
// 傾けずにまっすぐ立てて切り出す。ふちが上、プラグの差し込み方向が真上。サポート不要
module fit_test() {
    translate([0, 0, -(shelf - wall)]) rotate([90, 0, 0])
        difference() { tower_w(); pocket_w(); slot_w(); }
}
// 裏ぶた:外側の面を下にして印刷
module cover_print() { translate([0, 0, -(pb - wall)]) cover_w(); }

if (part == "fit_test") fit_test();
else if (part == "cover") cover_print();
else {
    stand();
    // 確認用:本体と裏ぶた(F5 のみ)
    %in_watch() cylinder(r = body_r, h = body_th, $fn = 120);
    %in_watch() cover_w();
}
