// =============================================================
//  M5Stack StopWatch  USB-C 充電スタンド【分割タワー版】
//  - プラグを持つ「塔」を前後 2 つに分割(つなぎ目は塔の左右の側面に来る)。プラグとケーブルを挟んでから台に挿す
//  - 塔の中:プラグのポケット/ケーブルの溝 → 後ろへ曲がって、後ろ半分の背面の出口から出る
//    出口はケーブルの反対側のコネクタ(10 × 4.5)が通る大きさ。後ろ半分にケーブルを通してから前半分を合わせる
//  - 塔はまっすぐな部品。後ろへの傾き(tilt)は台の差し込み穴の角度でつける
//  - 抜け止め:塔の差し込み部の左右に、しなる爪(スナップ)。台の差し込み部の左右の窓に、爪の段がカチッと掛かる
//    外すときは、左右の窓から爪を楊枝などで押し込みながら塔を引き上げる
//
//  ケーブル:Apple 純正 USB-C(樹脂部分 実測 幅10 × 厚さ4.5 × 長さ14.5、ケーブル約4.6)
//  本体:直径 52 / 厚さ 15.5 / USB-C は 6時の側面、背面からの高さ約 8.6
//
//  part = "stand"   台(背もたれの柱つき)。そのまま印刷、サポート不要
//         "towers"  塔の前後 2 個を、分割面を下にして並べたもの。サポート不要
//         "collar"  上のキャップ。塔の上の端にかぶせ、天板(0.4)でプラグを押さえ、輪で前後の半分が開かないように押さえる。天板を下にして印刷
//         "assembly" 組み上がりの確認用
//  見た目:キャップ・塔の見えている部分・台の差し込み部を同じ外形(角丸の四角)にそろえ、1 本の柱に見えるようにした。
//          塔の上端(キャップの中)と下端(台の中)だけ細くして、キャップと差し込み部がそこにかぶさる
//  ピンは使わない(塔は爪で台に、キャップは輪で塔に止まる)
// =============================================================

part      = "stand";

/* [プラグ(Apple 純正 USB-C 実測)] */
om_w      = 10.0;
om_t      = 4.5;
om_len    = 14.5;
clr       = 0.15;   // ポケットの左右のすき間(片側)
clr_z     = 0.2;    // ポケットの前後のすき間(片側)。分割面が前後なので、ポケットの天井(ブリッジ)のたれ分を見込む
len_clear = 0.15;   // 長さ方向のすき間
cable_d   = 4.6;    // ケーブルの太さ
grip      = 0.2;    // ケーブルの溝をケーブルより細くする量(はさんでつかむ)
lip_t     = 0;      // 塔そのもののふちは無し(細すぎて刷れないため)。プラグは上のキャップの天板で押さえる
seat      = 0.4;    // 塔の上端(= プラグの樹脂の上面)と本体の側面の基準のすき間。台・塔の位置はこれで決まる(変えない)

/* [塔] */
wall      = 2.0;
drop      = 7.0;    // ポケットの底からケーブルが後ろへ曲がり始めるまで
bend_r    = 5.0;    // ケーブルの曲げ半径
sock_d    = 12.0;   // 台に差し込む深さ(爪をしならせる長さを取るため 9 → 12)
sock_clr  = 0.12;   // 差し込みのすき間(片側)
peg_gap   = 0.3;    // 塔の下端と差し込み穴の底のすき間(塔の段が差し込み部の上の面に乗って位置が決まる)

/* [抜け止めの爪(差し込み部の左右)] */
arm_t     = 1.0;    // 爪の厚み(左右方向にしなる)
arm_gap   = 0.8;    // 爪の内側のすき間(しなる余地)
arm_root  = 1.5;    // 爪の根元(差し込み部の下端側)のつながり
arm_top_gap = 0.8;  // 爪の先と塔の段の間のすき間
bump_h    = 0.5;    // 爪の段の高さ
bump_ramp = 1.5;    // 段の下側の斜面(差し込むときにすべる)
bump_flat = 0.5;
bump_from_top = 2.6;  // 段の上の面(掛かる面)の位置:差し込み部の上の面から下へ
win_play  = 0.15;   // 窓と段の上下のあそび
// ケーブルの出口(後ろ半分の背面):反対側のコネクタが通る大きさ
exit_w    = 11.0;
exit_h    = 5.4;

/* [柱の外形(キャップ・塔・台の差し込み部で共通)] */
out_x     = 9.0;    // 半幅(左右)
out_z     = 6.2;    // 半分の奥行き(前後)。USB-C の中心から
col_r     = 1.0;    // 縦の角の丸み

/* [上の留め輪(カラー)] */
// 塔の上の端に四角い輪をはめて、前後の半分を閉じたまま押さえる
col_h     = 7.0;    // 輪の高さ(天板の下)
col_wall  = 1.6;    // 輪の壁の厚み
col_clr   = 0.1;    // 塔とのすき間(片側)。緩ければ 0.05、入らなければ 0.15
cap_t     = 0.4;    // キャップの天板の厚み(= 差し込みが浅くなる量)。0.4 で充電できることを確認済み
win_w     = 9.0;    // 天板の窓(USB-C プラグ 8.25 × 2.4 + 余裕)
win_h     = 3.2;

/* [台] */
tilt      = 25;     // 後ろへの傾き(度)。卓上時計として見やすいように 25°
base_t    = 5.0;
base_w    = 64;
base_front= 12;
base_back = 8;
base_r    = 6;
sock_wall = 2.0;    // 差し込み穴まわりの壁
floor_t   = 1.6;    // 差し込み穴の底の厚み

/* [背もたれの柱] */
post_x    = 16;
pad_w     = 5;
pad_h     = 10;
pad_y     = 4;
pad_t     = 3;
pad_gap   = 0.5;

/* [本体] */
body_r    = 26.0;
body_th   = 15.5;
usb_z     = 8.6;
plate_t   = 2.0;

$fn = 64;

// ---------- 塔の寸法(本体座標:X 3時, Y 12時, Z 画面側) ----------
pw     = om_w/2 + clr;
pz0    = usb_z - om_t/2 - clr_z; pz1 = usb_z + om_t/2 + clr_z;  // ポケットの奥行き(Z)
top_y  = -body_r - seat;                                        // 塔の上端(この上にキャップの天板が乗る)
// 上端(キャップの中):外形から輪の壁ぶん細い
tx     = out_x - col_clr - col_wall;
tz0    = usb_z - (out_z - col_clr - col_wall);  tz1 = usb_z + (out_z - col_clr - col_wall);
// 下端(台の差し込み穴の中):外形から差し込み部の壁ぶん細い
px     = out_x - sock_clr - sock_wall;
qz0    = usb_z - (out_z - sock_clr - sock_wall); qz1 = usb_z + (out_z - sock_clr - sock_wall);
oz0    = usb_z - out_z;          oz1 = usb_z + out_z;           // 外形の後ろ / 前の面
om_top = top_y - lip_t;
shelf  = om_top - om_len - len_clear;
cd     = cable_d - grip;                                        // ケーブルの溝の径
bend_y = shelf - drop;                                          // ここから後ろへ曲がる
exit_y = bend_y - bend_r;                                       // 背面から出るときの高さ
peg_top= exit_y - cable_d/2 - 2.0;                              // 台の差し込み穴の上端
peg_bot= peg_top - sock_d;

// ---------- 本体座標 → スタンド座標 ----------
s = sin(tilt); c = cos(tilt);
blk_bot_y = peg_bot - floor_t;                                  // 差し込み部の一番下
blk_z0    = oz0;
// 差し込み部の一番低い角が z = 0 に来るように塔の高さを決める
tower_top = -( c*(blk_bot_y + body_r) + s*(blk_z0 - usb_z) );
function W(p) = [ p[0], s*(p[1] + body_r) - c*(p[2] - usb_z), tower_top + c*(p[1] + body_r) + s*(p[2] - usb_z) ];
M = [[1,0,0,0],[0,s,-c,s*body_r + c*usb_z],[0,c,s,tower_top + c*body_r - s*usb_z],[0,0,0,1]];
module in_watch() { multmatrix(M) children(); }

// 柱の外形(XZ 平面の角丸の四角、USB-C の中心が原点)
module col2d(hx, hz, r = col_r) { offset(r = r) offset(delta = -r) square([2*hx, 2*hz], center = true); }
// 本体座標で、Y = y0〜y1 の柱
module col_w(hx, hz, y0, y1, r = col_r) { translate([0, y0, usb_z]) rotate([-90, 0, 0]) linear_extrude(y1 - y0) col2d(hx, hz, r); }

// ---------- 塔 ----------
module cable_path() {    // ケーブルの溝:ポケットの底 → 下へ drop → 後ろへ曲がって背面から外へ
    translate([0, shelf + 0.01, usb_z]) rotate([90, 0, 0]) cylinder(d = cd, h = drop + 0.02);
    // 曲がり:中心 (Y = bend_y, Z = usb_z - bend_r)、下向きから後ろ向きへ 90°(球の hull でつなぐ)
    for (i = [0:11]) hull() for (a = [i*90/12, (i+1)*90/12])
        translate([0, bend_y - bend_r*sin(a), usb_z - bend_r + bend_r*cos(a)]) sphere(d = cd, $fn = 24);
    translate([0, exit_y, usb_z - bend_r]) rotate([180, 0, 0]) cylinder(d = cd, h = 20);
}
// 爪の位置
arm_y0 = peg_bot + peg_gap + arm_root;                          // 爪の内側のすき間の下端(ここから下が根元)
arm_y1 = peg_top - arm_top_gap;                                 // 爪の先
yb1    = peg_top - bump_from_top;                               // 段の上の面(台の窓の上の縁に掛かる)
yb0    = yb1 - bump_flat - bump_ramp;
bz0    = qz0 + 0.5;  bz1 = qz1 - 0.5;                           // 段の前後の範囲

module tower_body() {
    difference() {
        union() {
            translate([-px, peg_bot + peg_gap, qz0]) cube([2*px, peg_top - peg_bot - peg_gap + 0.01, qz1 - qz0]);   // 差し込み部
            col_w(out_x, out_z, peg_top, top_y - col_h);                                                          // 見えている部分
            translate([-tx, top_y - col_h - 0.01, tz0]) cube([2*tx, col_h + 0.01, tz1 - tz0]);                    // キャップの中
            // 爪の段
            for (sx = [-1,1]) mirror([sx < 0 ? 1 : 0, 0, 0]) translate([0, 0, bz0]) linear_extrude(bz1 - bz0)
                polygon([[px - 0.01, yb0], [px + bump_h, yb0 + bump_ramp], [px + bump_h, yb1], [px - 0.01, yb1]]);
        }
        // ポケット
        translate([-pw, shelf, pz0]) cube([2*pw, om_top - shelf, pz1 - pz0]);
        // 金属部分の窓
        translate([0, top_y + 1, usb_z]) rotate([90, 0, 0]) linear_extrude(lip_t + 2)
            hull() for (sx = [-1,1]) translate([sx*(win_w/2 - win_h/2), 0]) circle(d = win_h);
        cable_path();
        // 爪:内側のすき間と、先のすき間(前後に貫通)
        for (sx = [-1,1]) mirror([sx < 0 ? 1 : 0, 0, 0]) {
            translate([px - arm_t - arm_gap, arm_y0, qz0 - 1]) cube([arm_gap, peg_top - arm_y0 + 0.01, qz1 - qz0 + 2]);
            translate([px - arm_t - 0.01, arm_y1, qz0 - 1]) cube([arm_t + 1, peg_top - arm_y1 + 0.01, qz1 - qz0 + 2]);
        }
    }
}
// 後ろ半分だけに開ける部分:ケーブルの曲がりを前(分割面)側へ開いた溝 + 背面の出口
module back_cut() {
    for (i = [0:11]) hull() for (a = [i*90/12, (i+1)*90/12]) {
        yy = bend_y - bend_r*sin(a);  zz = usb_z - bend_r + bend_r*cos(a);
        translate([0, yy, zz]) sphere(d = cd, $fn = 24);
        translate([0, yy, usb_z + 1]) sphere(d = cd, $fn = 24);
    }
    translate([0, exit_y, oz0 - 1]) linear_extrude(usb_z - oz0 + 2)
        offset(r = 1) offset(delta = -1) square([exit_w, exit_h], center = true);   // 角を少し丸めた四角(コネクタの角が通る)
}
module tower_half(side) {   // side = +1 (前 Z > USB 中心) / -1 (後ろ)
    difference() {
        intersection() {
            tower_body();
            translate([-50, -150, side > 0 ? usb_z : usb_z - 50]) cube([100, 200, 50]);
        }
        if (side < 0) back_cut();
    }
}

// ---------- 台 ----------
pad_c   = W([post_x, pad_y, -plate_t - pad_gap - pad_t]);
base_y0 = min(W([0, top_y, oz1])[1], W([0, peg_bot, oz1])[1], W([0, -body_r, body_th])[1]) - base_front;
base_y1 = pad_c[1] + base_back;
module base2d() { translate([-base_w/2, base_y0]) offset(r = base_r) offset(delta = -base_r) square([base_w, base_y1 - base_y0]); }
module pad_w(sx) { translate([sx*post_x - pad_w/2, pad_y - pad_h/2, -plate_t - pad_gap - pad_t]) cube([pad_w, pad_h, pad_t]); }

module socket_block_w() { col_w(out_x, out_z, blk_bot_y, peg_top); }
module socket_hole_w() {
    translate([-px - sock_clr, peg_bot, qz0 - sock_clr]) cube([2*(px + sock_clr), sock_d + 20, (qz1 - qz0) + 2*sock_clr]);
    // 爪の段が掛かる窓(左右の壁を貫通)
    for (sx = [-1,1]) mirror([sx < 0 ? 1 : 0, 0, 0])
        translate([px, yb0 - 0.3, bz0 - 0.3]) cube([out_x - px + 1, yb1 + win_play - (yb0 - 0.3), bz1 - bz0 + 0.6]);
}

module stand() {
    difference() {
        union() {
            linear_extrude(base_t) base2d();
            hull() {
                in_watch() socket_block_w();
                b0 = W([0, blk_bot_y, oz0]); b1 = W([0, blk_bot_y, oz1]);
                translate([0, (b0[1] + b1[1])/2, 0]) linear_extrude(0.01)
                    offset(r = col_r) offset(delta = -col_r) square([2*out_x, abs(b1[1] - b0[1])], center = true);
            }
            for (sx = [-1,1]) hull() {
                in_watch() pad_w(sx);
                q = W([sx*post_x, pad_y, -plate_t - pad_gap - pad_t]);
                translate([sx*post_x - pad_w/2, q[1] - 2, 0]) cube([pad_w, 8, base_t]);
            }
        }
        in_watch() socket_hole_w();
        // 台の上面のケーブル溝(差し込み部の後ろから後ろの端まで)
        cx = W([0, exit_y, oz0])[1];
        translate([-cable_d/2 - 0.3, cx, base_t - 2.5]) cube([cable_d + 0.6, 200, 3]);
    }
}

// ---------- 上のキャップ ----------
module window_2d() { hull() for (sx = [-1,1]) translate([sx*(win_w/2 - win_h/2), 0]) circle(d = win_h); }
// キャップ(印刷の向き):天板を下(z = 0〜cap_t)、輪が上に立つ。外形は柱と同じ角丸の四角
module cap_print() {
    hz = (tz1 - tz0)/2;
    difference() {
        linear_extrude(cap_t + col_h) col2d(out_x, out_z);
        translate([0, 0, cap_t]) linear_extrude(col_h + 1) square([2*(tx + col_clr), 2*(hz + col_clr)], center = true);
        translate([0, 0, -1]) linear_extrude(cap_t + 2) window_2d();
        // 輪の入り口(上の縁の内側)を面取り
        translate([0, 0, cap_t + col_h - 0.6]) linear_extrude(0.61, scale = [(2*tx + 2*col_clr + 1.2)/(2*tx + 2*col_clr), (2*hz + 2*col_clr + 1.2)/(2*hz + 2*col_clr)])
            square([2*tx + 2*col_clr, 2*hz + 2*col_clr], center = true);
    }
}
// 本体座標での取り付け位置:天板の上面が本体の側面
module collar_w() { translate([0, top_y + cap_t, usb_z]) rotate([90, 0, 0]) cap_print(); }

// ---------- 出力 ----------
module towers_print() {
    // 分割面(Z = USB の中心)を下に。前半分は +Z → 上、後ろ半分は -Z → 上
    translate([-12, 0, -usb_z]) tower_half(1);
    translate([ 12, 0,  usb_z]) rotate([0, 180, 0]) tower_half(-1);
}

if (part == "towers") towers_print();
else if (part == "collar") cap_print();
else if (part == "half_f") tower_half(1);    // 本体座標のまま(確認・図解用)
else if (part == "half_b") tower_half(-1);
else if (part == "assembly") { stand(); in_watch() { tower_half(1); tower_half(-1); collar_w(); } %in_watch() cylinder(r = body_r, h = body_th, $fn = 120); }
else { stand(); %in_watch() { tower_half(1); tower_half(-1); cylinder(r = body_r, h = body_th, $fn = 120); } }
