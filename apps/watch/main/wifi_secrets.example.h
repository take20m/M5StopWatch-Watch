// Wi-Fi の時刻合わせ用の設定の見本。
// このファイルを同じフォルダに wifi_secrets.h という名前でコピーし、SSID とパスワードを書き換える。
// wifi_secrets.h は git に入らない（リポジトリの .gitignore で除外）。
// wifi_secrets.h がなければ、Wi-Fi の時刻合わせはしない（USB からの時刻合わせだけになる）。
#pragma once

// 1 つだけ登録するとき
#define WATCH_WIFI_SSID     "your-ssid"
#define WATCH_WIFI_PASSWORD "your-password"

// 複数登録するとき（自宅と会社など）は、上の 2 行の代わりにこちらを書く。
// 周りを探して、登録したものの中で一番電波の強い Wi-Fi につなぐ。SSID を隠している Wi-Fi は見つけられない。
// ESP32 は 2.4GHz にしかつなげない。
// #define WATCH_WIFI_NETWORKS \
//     {                       \
//         {"home-ssid", "home-password"}, {"office-ssid", "office-password"}, \
//     }
