# GPSコンパスの使い方

[![Python](https://img.shields.io/badge/Python-3.8+-blue.svg)](https://www.python.org)
[![License](https://img.shields.io/badge/License-MIT-green.svg)](https://opensource.org/licenses/MIT)


## 事前準備

# 1. 事前準備

#  1-1. ハードウェア準備

  Raspberry Pi 5 (RaspbianなどのLinux環境)
  USB GPSモジュール 2台接続
  USB0 → 基準局 (Base)
  USB1 → 移動局 (Rover)
  USB GPSのシリアルデバイスパスは /dev/ttyUSB0, /dev/ttyUSB1 と想定しています。
  違う場合は app.py の BASE_DEVICE, ROVER_DEVICE を修正してください。

# 1-2. ソフトウェアインストール
  ZIPファイルを展開し、Raspberry Pi上の任意のディレクトリ（例：/home/pi/gps_compass_project）に配置してください。
  Python3とpipが入っていることを確認。
  必要なPythonライブラリをインストール：

# 2. 設定確認

  シリアルポート名、ボーレート、ログパスなど環境に合わせて調整可能。
  基線長（70cm）
  必要に応じて、app.py のデバイスパスやボーレートも調整してください。


# 3. 起動方法

# 3-1. 手動起動
  以下のスクリプトを実行して、GPS読み取り、Flaskサーバー起動を同時に行います。

  python3 app.py

Flaskサーバーはデフォルトでポート5000番で起動します。

# 3-2. Webブラウザで結果確認
  Raspberry PiのIPアドレスにブラウザでアクセス：
  http://<ラズパイのIPアドレス>:5000/
  地図が表示され、基準局（緑マーカー）、移動局（赤マーカー）、方位角、推定誤差がリアルタイムに更新されます。


   ```bash
project/
├── app.py
├── config.ini
├── templates/
│   └── index.html
├── static/
│   ├── styles.css
│   ├── leaflet/
│   │   ├── leaflet.css
│   │   └── leaflet.js
└── app.log
```



# 注意事項
依存ライブラリ: mpu6050-raspberrypi は環境依存のため、実際のハードウェアに応じて適切なライブラリを指定してください。

Leafletの配置: static/leaflet フォルダにLeaflet 1.9.4をダウンロードして配置する必要があります（例: https://leafletjs.com/）。

本番環境: 本番では gunicorn や uwsgi を使用し、app.run(debug=True) を避けてください。例: gunicorn -w 4 -b 0.0.0.0:5000 app:app

テスト: 実運用前に、ダミーモード（DummyMode = True）で動作確認を行い、ログを確認してください。

proj4.js の配置:
static/proj4.js をプロジェクトにダウンロードして配置してください（例: wget https://cdnjs.cloudflare.com/ajax/libs/proj4js/2.7.5/proj4.js -O static/proj4.js）。

