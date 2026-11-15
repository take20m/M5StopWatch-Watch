// Wi-Fi の時刻合わせ用の設定の見本。
// このファイルを同じフォルダに wifi_secrets.h という名前でコピーし、SSID とパスワードを書き換える。
// wifi_secrets.h は git に入らない（リポジトリの .gitignore で除外）。
// wifi_secrets.h がなければ、Wi-Fi の時刻合わせはしない（USB からの時刻合わせだけになる）。
#pragma once

#define WATCH_WIFI_SSID     "your-ssid"
#define WATCH_WIFI_PASSWORD "your-password"
