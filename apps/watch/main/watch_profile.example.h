// 自分用の表示の設定の見本。
// このファイルを同じフォルダに watch_profile.h という名前でコピーし、書き換える。
// watch_profile.h は git に入らない（リポジトリの .gitignore で除外）。
// watch_profile.h がなければ、QR コードの画面は出さない（画面の切り替えで飛ばす）。
#pragma once

// QR コードの画面に出す URL と、その下に出す文字（ASCII）
#define WATCH_QR_URL   "https://example.com/"
#define WATCH_QR_LABEL "example.com"
