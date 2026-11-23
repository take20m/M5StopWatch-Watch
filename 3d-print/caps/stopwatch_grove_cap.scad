// =============================================================
//  M5Stack StopWatch  Grove(HY2.0-4P)ポートのキャップ
//  - 本体側面(5時の位置)の Grove 開口にはめ込む栓 + 外側は「本体のボタンと同じ形」のカバー
//    本体ボタン(公式データ): 高さ Z 4.4〜10.6 / 出っ張り 1.7 / 両端は R1.7 の丸み / 上下の縁 R0.5
//  - 開口の形は公式構造データ StopWatch.stl から読み取り(半径方向に見た投影):
//      中心角: 6時から 3時方向へ約 44.7°(下記 cap_angle)
//      上段(表示面側): 高さ Z 7.13〜10.38 / 幅 約 12.6
//      下段(背面側)  : 高さ Z 4.88〜 7.13 / 幅 約 10.5
//    ※ Z は本体背面からの高さ。データは簡易形状なので、合わなければ実測値で上書きしてください
//  - 保持:栓の外周の「つぶしリブ」で固定(押し込むとリブがつぶれてきつくなる)。推奨素材 TPU / PETG
//  - 印刷向き: このまま(カバーの下端がベッド)。サポート不要(栓の下面に 0.5mm の小さな張り出しあり)
// =============================================================

body_r      = 26.0;
cap_angle   = 44.7;     // 開口の中心角(6時から 3時方向へ)

// 開口(タンジェント方向 u、高さ Z)。u は開口中心からのずれ
up_u0 = -6.3;  up_u1 = 6.3;  up_z0 = 7.13;  up_z1 = 10.38;   // 上段
lo_u0 = -5.25; lo_u1 = 5.25; lo_z0 = 4.88;  lo_z1 = 7.13;    // 下段

fit_clear   = 0.10;     // 栓の本体を開口より小さくする量(すき間ばめ。保持はリブで行う)
plug_depth  = 1.5;      // 栓が開口に入る深さ(奥のコネクタに当たって浮くなら 1.0 に戻す)
// つぶしリブ:栓の外周に細いリブを付け、押し込むとリブがつぶれてきつく固定される
rib_on      = true;
rib_h       = 0.15;     // リブの高さ(開口から外へ出る量)。試し刷り 0 / 0.10 / 0.15 / 0.20 のうち 0.15 がちょうどよかった(0.25 はきつくて入らなかった)
rib_w       = 0.6;      // リブの幅
rib_lead    = 0.4;      // 先端側のリブなし区間(入れ始めのガイド)
mark_n      = 0;        // 見分け用の刻み(カバー上の縁に小さな溝を mark_n 本)
// 外側のカバー(本体ボタンにそろえる)
btn_z0      = 4.4;      // 本体ボタンと同じ高さ帯
btn_z1      = 10.6;
btn_h       = 1.7;      // 出っ張り(本体ボタンと同じ)
btn_end_r   = 1.7;      // 両端の丸み(平面で見て)
btn_edge_r  = 0.5;      // 上下の縁の丸み
btn_margin  = 0.6;      // 開口からの左右のはみ出し
$fn = 180;

// 開口の2D形状(u, z)
module opening2d(off = 0) {
    offset(delta = off) union() {
        translate([up_u0, up_z0]) square([up_u1 - up_u0, up_z1 - up_z0]);
        translate([lo_u0, lo_z0]) square([lo_u1 - lo_u0, lo_z1 - lo_z0 + 0.01]);
    }
}
// 2D形状を、開口の中心の半径方向に沿って伸ばした柱(グローバル座標)
module radial_prism(len_in = 6, len_out = 6) {
    rotate([0, 0, cap_angle])                 // 6時(-Y)から +X 側へ回す
        translate([0, -body_r, 0])
            rotate([90, 0, 0])                // (u, z) → (X, Z)、押し出しは -Y(外向き)
                translate([0, 0, -len_in]) linear_extrude(len_in + len_out) children();
}
module annulus(r0, r1) {
    difference() { cylinder(r = r1, h = 30); translate([0,0,-1]) cylinder(r = r0, h = 32); }
}

// つぶしリブ(開口の上下の辺と、上段の左右の辺に付ける)
module ribs2d() {
    o = -fit_clear;
    for (u = [-4, 0, 4]) translate([u - rib_w/2, up_z1 + o - 0.01]) square([rib_w, rib_h + 0.01]);   // 上の辺
    for (u = [-3, 3])    translate([u - rib_w/2, lo_z0 - o - rib_h]) square([rib_w, rib_h + 0.01]);  // 下の辺
    for (sx = [-1, 1]) {                                                                               // 上段の左右の辺
        uu = sx > 0 ? up_u1 + o : up_u0 - o;
        translate([sx > 0 ? uu - 0.01 : uu - rib_h, (up_z0 + up_z1)/2 - rib_w/2]) square([rib_h + 0.01, rib_w]);
    }
}

// 本体ボタンと同じ断面のカバー
// 円周方向に薄い断面(半径方向 × 高さ)を並べて hull し、本体の円筒(r < body_r)を引く
module button_cover_solid() {
    W   = max(up_u1, lo_u1) - min(up_u0, lo_u0) + 2*btn_margin;
    u0  = W/2 - btn_end_r;
    n   = 48;
    H   = btn_z1 - btn_z0;
    difference() {
        rotate([0, 0, cap_angle]) hull() {
            for (i = [0:n]) {
                u  = -W/2 + W*i/n;
                du = max(0, abs(u) - u0);
                w  = (du >= btn_end_r) ? 0.05 : sqrt(btn_end_r*btn_end_r - du*du) * btn_h / btn_end_r;
                a  = u / body_r * 180 / PI;
                e  = min(btn_edge_r, w);           // 上下の縁の丸み(出っ張りが小さい所では小さく)
                rotate([0, 0, a]) translate([0, -body_r, 0])
                    // (w, z) 断面:幅 0.01 の板を、半径方向 0〜w、高さ btn_z0〜btn_z1 の角丸長方形で
                    multmatrix([[0,0,1,0],[0,-1,0,0],[1,0,0,0]])   // 2D(高さ, 出っ張り) → (Z, 外向き)
                        linear_extrude(0.01, center = true)
                            translate([btn_z0, -0.3]) hull() {
                                square([H, 0.3]);
                                translate([e, w + 0.3 - e]) circle(r = max(e, 0.01), $fn = 16);
                                translate([H - e, w + 0.3 - e]) circle(r = max(e, 0.01), $fn = 16);
                            }
            }
        }
        translate([0,0,-1]) cylinder(r = body_r, h = 30);
    }
}

module cap() {
    // 栓(開口に入る部分)
    intersection() {
        radial_prism() opening2d(-fit_clear);
        annulus(body_r - plug_depth, body_r + 0.01);
    }
    // つぶしリブ(先端 rib_lead はリブなし)
    if (rib_on) intersection() {
        radial_prism() ribs2d();
        annulus(body_r - plug_depth + rib_lead, body_r + 0.01);
    }
    // 外側のカバー(見分け用の刻みを上の縁に)
    difference() {
        button_cover_solid();
        for (i = [0:mark_n-1]) if (mark_n > 0)
            rotate([0, 0, cap_angle + ((i - (mark_n-1)/2) * 1.4) / body_r * 180 / PI])
                translate([0, -body_r - btn_h - 0.5, btn_z1 - 0.5]) cube([0.5, 2.2, 1.2], center = true);
    }
}

cap();

// 確認用(F5 のみ):本体
%difference() { cylinder(r = body_r, h = 15.5); }
