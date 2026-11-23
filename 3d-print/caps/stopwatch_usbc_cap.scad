// =============================================================
//  M5Stack StopWatch  USB-C ポートのキャップ
//  - USB-C の差し込み口(レセプタクル)に挿す「栓」+ 本体側面の開口を覆う「ふた」
//  - 栓は真ん中の舌(タング)をよけるスリット付きの筒。USB-C 規格の寸法で作るので、
//    本体の簡易データの誤差に左右されにくい。差し込み口の左右のバネで保持される
//      USB-C 差し込み口の内寸 : 幅 8.34 / 高さ 2.56(両端は半円)
//      舌                   : 幅 6.69 / 厚さ 0.7(中央)
//  - 位置は公式構造データ StopWatch.stl から読み取り:6時の側面、本体背面から高さ Z 約 7.0〜10.25
//  - 推奨素材: TPU か PETG
//  - 印刷向き: このまま(ふたの外側の平らな面がベッド、栓が上向き)。サポート不要
//    print_orient = false にすると本体に付けた位置で表示(確認用)
// =============================================================

print_orient = true;

body_r      = 26.0;
port_z      = 8.6;      // 差し込み口の中心の高さ(本体背面から)

// USB-C 差し込み口(規格寸法)
rcpt_w      = 8.34;
rcpt_h      = 2.56;
tongue_w    = 6.69;
tongue_t    = 0.70;

// 栓
plug_clear  = 0.10;     // 栓の外周を差し込み口より小さくする量(緩ければ 0.05、入らなければ 0.15)
slot_clear  = 0.25;     // 舌をよけるスリットの余裕(片側)
plug_len    = 3.0;      // 本体の表面から奥へ入る長さ
plug_lead   = 0.3;      // 先端の面取り(入れやすく)

// ふた(開口を覆う板。外側は平ら、内側は本体の丸みに沿う)
cap_w       = 11.5;     // 横幅(開口 約 9.3 + 余裕)
cap_h       = 5.0;      // 高さ(開口 約 3.3 + 余裕)
cap_t       = 1.2;      // 中央の厚み(本体表面から外側の面まで)
cap_r       = 1.0;      // 角の丸み
nail_notch  = 0.6;      // 下の縁の面取り(爪をかけて外す用)

$fn = 96;

// ---- ローカル座標:u = 横(3時方向)、v = 高さ(本体背面から)、w = 外向き(6時方向の半径) ----
module stadium(w, h) { hull() for (s = [-1,1]) translate([s*(w/2 - h/2), 0]) circle(d = h); }

module plug_profile(off = 0) {
    difference() {
        stadium(rcpt_w - 2*plug_clear - 2*off, rcpt_h - 2*plug_clear - 2*off);
        square([tongue_w + 2*slot_clear, tongue_t + 2*slot_clear], center = true);
    }
}

module cap_local() {
    w0 = body_r - plug_len;                 // 栓の先端
    // 栓(先端は面取り)
    hull_free_lead();
    extrude_w(w0 + plug_lead, body_r + 0.2) translate([0, port_z]) plug_profile();
    // ふた
    difference() {
        hull() {
            extrude_w(body_r - 2, body_r + cap_t - nail_notch) translate([0, port_z]) rounded_rect(cap_w, cap_h, cap_r);
            extrude_w(body_r + cap_t - 0.01, body_r + cap_t)
                translate([0, port_z + nail_notch/2]) rounded_rect(cap_w, cap_h - nail_notch, cap_r);
        }
        // 内側を本体の丸みに沿ってくり抜く(本体の円筒 r < body_r)
        translate([0, 0, -50]) cylinder(r = body_r, h = 100, $fn = 360);   // ローカルの z = v が円筒の軸
    }
}
module hull_free_lead() {
    w0 = body_r - plug_len;
    hull() {
        extrude_w(w0, w0 + 0.01) translate([0, port_z]) offset(delta = -plug_lead) stadium(rcpt_w - 2*plug_clear, rcpt_h - 2*plug_clear);
        extrude_w(w0 + plug_lead - 0.01, w0 + plug_lead) translate([0, port_z]) stadium(rcpt_w - 2*plug_clear, rcpt_h - 2*plug_clear);
    }
}
module rounded_rect(w, h, r) { offset(r = r) square([w - 2*r, h - 2*r], center = true); }
// 2D(u, v) を w 方向に [a, b] の範囲で押し出す(ローカル座標:x = u, y = w, z = v)
module extrude_w(a, b) {
    multmatrix([[1,0,0,0],[0,0,1,0],[0,1,0,0]])   // (x2, y2, e) → (u = x2, w = e, v = y2)
        translate([0, 0, a]) linear_extrude(b - a) children();
}

module cap_body() {
    difference() {
        cap_local();
        // 先端の面取り部分にもスリットを通す
        extrude_w(body_r - plug_len - 1, body_r + 0.2) translate([0, port_z])
            square([tongue_w + 2*slot_clear, tongue_t + 2*slot_clear], center = true);
    }
}

// ローカル(u, w, v) → グローバル(X = u, Y = -w, Z = v)
module to_global() { multmatrix([[1,0,0,0],[0,-1,0,0],[0,0,1,0]]) children(); }

if (print_orient) {
    // ふたの外側の面(w = body_r + cap_t)をベッドに、栓を上向きに
    multmatrix([[1,0,0,0],[0,0,1,-port_z],[0,-1,0,body_r + cap_t]]) cap_body();
} else {
    to_global() cap_body();
    %cylinder(r = body_r, h = 15.5);
}
