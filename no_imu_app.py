import serial
import threading
import time
import json
import logging
import logging.handlers
from collections import deque
import math
from typing import Optional, Dict, Any, List
import configparser
import os
from functools import wraps
import random
import ssl
import pynmea2 # pynmea2はNMEAパース用ですが、今回は生データを保存します

from flask import Flask, jsonify, render_template, request
from pyproj import Geod
from geopy.distance import geodesic

# --- 設定ファイルの読み込み ---
config = configparser.ConfigParser()
config_file_path = os.path.join(os.path.dirname(__file__), 'config.ini')

def load_config():
    """設定ファイルを読み込み、存在しない場合はデフォルトを作成する。"""
    if not os.path.exists(config_file_path):
        print("config.iniが見つかりません。デフォルト設定を作成します。")
        config['DEFAULT'] = {
            'API_KEY': 'your_api_key_here', # 必ず変更してください
            'GPS_BASE_PORT': '/dev/ttyUSB0', # Linux/macOS
            'GPS_ROVER_PORT': '/dev/ttyUSB1', # Linux/macOS
            # 'GPS_BASE_PORT': 'COM3', # Windowsの場合
            # 'GPS_ROVER_PORT': 'COM4', # Windowsの場合
            'BAUDRATE': '115200',
            'DUMMY_MODE': 'False',
            'LOG_LEVEL': 'INFO',
            'SERIAL_RETRY_INTERVAL': '5' # シリアルポート再接続間隔 (秒)
        }
        with open(config_file_path, 'w') as configfile:
            config.write(configfile)
    else:
        config.read(config_file_path)

    # 設定値の取得と型変換
    # 環境変数からのAPIキー優先
    return {
        'API_KEY': os.environ.get('API_KEY', config['DEFAULT'].get('API_KEY', 'your_api_key_here')),
        'GPS_BASE_PORT': config['DEFAULT'].get('GPS_BASE_PORT', '/dev/ttyUSB0'),
        'GPS_ROVER_PORT': config['DEFAULT'].get('GPS_ROVER_PORT', '/dev/ttyUSB1'),
        'BAUDRATE': int(config['DEFAULT'].get('BAUDRATE', '115200')),
        'DUMMY_MODE': config['DEFAULT'].getboolean('DUMMY_MODE', False),
        'LOG_LEVEL_STR': config['DEFAULT'].get('LOG_LEVEL', 'INFO').upper(),
        'SERIAL_RETRY_INTERVAL': int(config['DEFAULT'].get('SERIAL_RETRY_INTERVAL', '5'))
    }

APP_CONFIG = load_config()

# --- ロギング設定 ---
logging.basicConfig(level=getattr(logging, APP_CONFIG['LOG_LEVEL_STR'], logging.INFO),
                    format='%(asctime)s - %(levelname)s - %(message)s')
logger = logging.getLogger(__name__)

# --- グローバル定数 ---
# 設定ファイルから取得した値を定数として定義
API_KEY = APP_CONFIG['API_KEY']
GPS_BASE_PORT = APP_CONFIG['GPS_BASE_PORT']
GPS_ROVER_PORT = APP_CONFIG['GPS_ROVER_PORT']
BAUDRATE = APP_CONFIG['BAUDRATE']
DUMMY_MODE = APP_CONFIG['DUMMY_MODE']
SERIAL_RETRY_INTERVAL = APP_CONFIG['SERIAL_RETRY_INTERVAL']

# その他の定数
HDOP_THRESHOLD = 2.0 # HDOPの閾値
NMEA_BUFFER_SIZE = 100 # NMEA表示用バッファサイズ（行数）

# グローバルイベント
stop_event = threading.Event()

# --- データを保持するクラス ---
class SensorData:
    """GPSデータを管理し、スレッドセーフなデータアクセスを提供するクラス。"""
    def __init__(self):
        self.lock = threading.Lock()
        self.data_updated_event = threading.Event() # データ更新イベント

        # GPSデータ
        self.base_data: Dict[str, Any] = {'lat': 0.0, 'lon': 0.0, 'hdop': 99.9, 'quality': 0, 'timestamp': 0.0}
        self.rover_data: Dict[str, Any] = {'lat': 0.0, 'lon': 0.0, 'hdop': 99.9, 'quality': 0, 'timestamp': 0.0}
        self.base_connected: bool = False
        self.rover_connected: bool = False
        self.base_connection_timestamp: float = 0.0
        self.rover_connection_timestamp: float = 0.0
        self.base_port_errors: int = 0
        self.base_serial_errors: int = 0
        self.rover_port_errors: int = 0
        self.rover_serial_errors: int = 0
        self.nmea_buffer: deque = deque(maxlen=NMEA_BUFFER_SIZE) # NMEAデータを保持するバッファ

        # 融合データ (IMU削除のため、GPSのみから算出)
        self.heading_gps: float = 0.0 # GPSのみから算出された方位角 (基準局から移動局への方向)
        self.distance: float = 0.0
        self.error: float = 999.9

        # IMU削除に伴い、グラフデータは使用されないが、APIの互換性のため残す
        self.graph_azimuths: deque = deque(range(360)) # 0-359
        self.graph_values: deque = deque([0.0] * 360) # 全て0.0で初期化

    def _parse_gga(self, line: str) -> Optional[Dict[str, Any]]:
        """GPGGAセンテンスを解析し、緯度、経度、HDOP、品質を抽出します。
        pynmea2ライブラリを使用して解析を堅牢化。
        """
        try:
            msg = pynmea2.parse(line)
            # GGAメッセージであることを確認
            if not isinstance(msg, pynmea2.types.talker.GGA):
                return None
            
            lat_deg = msg.latitude
            lon_deg = msg.longitude
            quality = msg.gps_qual
            hdop = msg.hdop

            # 無効なデータ（0.0, 0.0など、またはNone/NaN）をチェック
            if abs(lat_deg) < 0.0001 and abs(lon_deg) < 0.0001:
                return None
            if hdop is None: # HDOPがNoneの場合のデフォルト値
                hdop = 99.9

            return {'lat': lat_deg, 'lon': lon_deg, 'hdop': hdop, 'quality': quality}
        except pynmea2.ParseError as e:
            logger.debug(f"NMEAパースエラー (GPGGA以外か不正フォーマット): {e} - Line: '{line}'")
            return None
        except ValueError as e: # float変換失敗など
            logger.warning(f"NMEAデータ変換エラー: {e} - Line: '{line}'")
            return None
        except Exception as e:
            logger.error(f"予期せぬNMEA解析エラー: {e} - Line: '{line}'")
            return None


    def update_gps_data(self, target_key: str, lat: float, lon: float, hdop: float, quality: int) -> None:
        """GPSデータをスレッドセーフに更新し、接続状態をセットする。"""
        with self.lock:
            data = {'lat': lat, 'lon': lon, 'hdop': hdop, 'quality': quality, 'timestamp': time.monotonic()}
            
            if target_key == 'base':
                self.base_data = data
                # 接続状態が変化した場合のみログ
                if not self.base_connected:
                    self.base_connected = True
                    self.base_connection_timestamp = time.monotonic()
                    logger.info(f"GPS Base ({GPS_BASE_PORT}) が接続されました。")
            elif target_key == 'rover':
                self.rover_data = data
                # 接続状態が変化した場合のみログ
                if not self.rover_connected:
                    self.rover_connected = True
                    self.rover_connection_timestamp = time.monotonic()
                    logger.info(f"GPS Rover ({GPS_ROVER_PORT}) が接続されました。")
            self.data_updated_event.set()

# グローバルインスタンスの作成
sensor_data = SensorData()
geod = Geod(ellps='WGS84') # WGS84楕円体を使用

# --- Flaskアプリケーションの初期化 ---
app = Flask(__name__)

# --- APIキー認証デコレータ ---
def require_api_key(view_function):
    @wraps(view_function)
    def decorated_function(*args, **kwargs):
        if request.headers.get('X-API-KEY') and request.headers.get('X-API-KEY') == API_KEY:
            return view_function(*args, **kwargs)
        else:
            logger.warning(f"認証失敗: 不正なAPIキーまたはキーなし。クライアントIP: {request.remote_addr}")
            return jsonify({"error": "Unauthorized"}), 401
    return decorated_function

# --- スレッド関数 ---
def read_gps_thread(port: str, station_type: str):
    """
    シリアルポートからGPSデータを読み取り、SensorDataを更新するスレッド。
    NMEAデータをバッファに保存します。
    """
    logger.info(f"{station_type} GPSスレッドを開始します: {port}")
    if DUMMY_MODE:
        logger.info(f"ダミーモード: {station_type} GPSデータは生成されます。")
        start_time = time.monotonic()
        while not stop_event.is_set():
            current_time = time.monotonic()
            elapsed_time = current_time - start_time
            # 少しずつ位置をずらすダミーデータ
            if station_type == 'base':
                lat = 35.681236 + math.sin(elapsed_time / 20) * 0.0001
                lon = 139.767125 + math.cos(elapsed_time / 20) * 0.0001
                hdop = 0.8 + math.sin(elapsed_time / 5) * 0.1
                quality = 4 # RTK Fix
            elif station_type == 'rover':
                lat = 35.681236 + math.sin(elapsed_time / 20 + 0.00005) * 0.0001 + 0.00001 # 少しずらす
                lon = 139.767125 + math.cos(elapsed_time / 20 + 0.00005) * 0.0001 + 0.00001 # 少しずらす
                hdop = 0.9 + math.cos(elapsed_time / 5) * 0.1
                quality = 4 # RTK Fix
            sensor_data.update_gps_data(station_type, lat, lon, hdop, quality)
            # ダミーNMEAデータを生成してバッファに追加
            dummy_nmea = f"$GPGGA,{time.strftime('%H%M%S.00', time.gmtime())},{lat:.7f},N,{lon:.7f},E,{quality},{random.randint(5,15)},{hdop:.1f},100.0,M,40.0,M,,*FF"
            with sensor_data.lock:
                sensor_data.nmea_buffer.append(dummy_nmea)
            time.sleep(0.5) # ダミーデータの更新間隔
        return

    while not stop_event.is_set():
        ser = None
        try:
            ser = serial.Serial(port, BAUDRATE, timeout=1)
            logger.info(f"シリアルポート {port} に接続しました ({station_type})")
            with sensor_data.lock:
                if station_type == 'base':
                    sensor_data.base_connected = True
                    sensor_data.base_connection_timestamp = time.monotonic()
                elif station_type == 'rover':
                    sensor_data.rover_connected = True
                    sensor_data.rover_connection_timestamp = time.monotonic()

            while not stop_event.is_set():
                line = ser.readline().decode('ascii', errors='ignore').strip()
                if line: # 空行でなければ処理
                    with sensor_data.lock:
                        sensor_data.nmea_buffer.append(line) # 全てのNMEAデータをバッファに保存
                    
                    if line.startswith('$GPGGA'):
                        parsed_data = sensor_data._parse_gga(line) # pynmea2を利用
                        if parsed_data:
                            sensor_data.update_gps_data(station_type, parsed_data['lat'], parsed_data['lon'], parsed_data['hdop'], parsed_data['quality'])
                time.sleep(00.01) # 短いインターバルでCPU使用率を抑える

        except serial.SerialException as e:
            logger.error(f"シリアルポート {port} のエラー: {e}")
            with sensor_data.lock:
                if station_type == 'base':
                    sensor_data.base_port_errors += 1
                    sensor_data.base_connected = False
                elif station_type == 'rover':
                    sensor_data.rover_port_errors += 1
                    sensor_data.rover_connected = False
        except Exception as e:
            logger.error(f"GPS読み取り中に予期せぬエラー: {e}")
            with sensor_data.lock:
                if station_type == 'base':
                    sensor_data.base_serial_errors += 1
                    sensor_data.base_connected = False
                elif station_type == 'rover':
                    sensor_data.rover_serial_errors += 1
                    sensor_data.rover_connected = False
        finally:
            if ser and ser.is_open:
                ser.close()
            with sensor_data.lock:
                # 切断状態を明示的にFalseにセット
                if station_type == 'base':
                    sensor_data.base_connected = False
                elif station_type == 'rover':
                    sensor_data.rover_connected = False
            
            if not stop_event.is_set(): # アプリケーション停止中でない場合のみ再接続を試みる
                logger.warning(f"シリアルポート {port} から切断されました。{SERIAL_RETRY_INTERVAL}秒後に再接続を試みます...")
                time.sleep(SERIAL_RETRY_INTERVAL)
    logger.info(f"GPS {station_type} 読み取りスレッドを停止します。")

def calculate_heading_and_error_thread():
    """
    GPSデータからヘディングと誤差を計算するスレッド。
    IMUがないため、GPSデータのみを使用します。
    """
    logger.info("計算スレッドを開始します。")
    while not stop_event.is_set():
        sensor_data.data_updated_event.wait(timeout=1.0) 
        sensor_data.data_updated_event.clear()

        with sensor_data.lock:
            base_lat = sensor_data.base_data['lat']
            base_lon = sensor_data.base_data['lon']
            rover_lat = sensor_data.rover_data['lat']
            rover_lon = sensor_data.rover_data['lon']
            
            # --- 基線長と誤差の計算 ---
            distance = geodesic((base_lat, base_lon), (rover_lat, rover_lon)).meters
            sensor_data.distance = distance

            # --- ヘディングの計算 (GPSのみ) ---
            if base_lat != 0.0 and base_lon != 0.0 and rover_lat != 0.0 and rover_lon != 0.0:
                fwd_azimuth, _, _ = geod.fwd(base_lon, base_lat, rover_lon, rover_lat)
                gps_heading = (fwd_azimuth + 360) % 360 # 0-360度に正規化
                sensor_data.heading_gps = gps_heading 
            else:
                sensor_data.heading_gps = 0.0 # GPSデータが不完全な場合は不定

            # --- 基線誤差の計算 ---
            if sensor_data.base_data['quality'] == 4 and sensor_data.rover_data['quality'] == 4: # RTK Fix
                sensor_data.error = max(0.02, 0.005 * (sensor_data.base_data['hdop'] + sensor_data.rover_data['hdop']))
            elif sensor_data.base_data['quality'] == 5 and sensor_data.rover_data['quality'] == 5: # RTK Float
                sensor_data.error = max(0.05, 0.02 * (sensor_data.base_data['hdop'] + sensor_data.rover_data['hdop']))
            else: # Single or unknown
                sensor_data.error = max(0.5, 0.2 * (sensor_data.base_data['hdop'] + sensor_data.rover_data['hdop']))
            
            # IMU削除に伴い、グラフデータは使われないが、API互換性のためダミーデータを更新
            # ここでは単にランダム値を設定
            for i in range(360):
                sensor_data.graph_values[i] = random.uniform(-99.0, -20.0)
            
        time.sleep(0.05) # 計算スレッドの実行間隔
    logger.info("計算スレッドを停止します。")

# --- APIエンドポイント ---
@app.route("/")
def home():
    """
    アプリケーションのルートURLにアクセスしたときに、GPS可視化ページを表示します。
    """
    return render_template('index.html')

@app.route("/api/position", methods=["GET"])
@require_api_key
def get_current_data():
    """
    現在のGPSおよび計算されたヘディング・誤差データをJSON形式で返すAPIエンドポイント。
    """
    with sensor_data.lock:
        data = {
            "lat": round(sensor_data.base_data['lat'], 7),
            "lon": round(sensor_data.base_data['lon'], 7),
            "hdop_base": round(sensor_data.base_data['hdop'], 2),
            "base_quality": sensor_data.base_data['quality'], 
            "rover_lat": round(sensor_data.rover_data['lat'], 7), 
            "rover_lon": round(sensor_data.rover_data['lon'], 7), 
            "hdop_rover": round(sensor_data.rover_data['hdop'], 2),
            "rover_quality": sensor_data.rover_data['quality'], 
            "heading": round(sensor_data.heading_gps, 2), # IMU削除によりheading_fusedは削除され、GPSのみのheading
            "heading_gps": round(sensor_data.heading_gps, 2), # このフィールドは以前のバージョンとの互換性のため残す
            "error": round(sensor_data.error, 3),
            "distance": round(sensor_data.distance, 3),
            "imu": False, # IMUは常にOFF
            # IMU関連のデータフィールドは削除
            "gyro_z_offset": 0.0, # IMU削除のため常に0.0
            "base_connected": sensor_data.base_connected,
            "rover_connected": sensor_data.rover_connected,
            "base_port_errors": sensor_data.base_port_errors,
            "base_serial_errors": sensor_data.base_serial_errors,
            "rover_port_errors": sensor_data.rover_port_errors,
            "rover_serial_errors": sensor_data.rover_serial_errors,
            "dummy_mode": DUMMY_MODE,
            "log_level": logging.getLevelName(logger.level)
        }
    return jsonify(data)

@app.route("/api/graph_data", methods=["GET"])
@require_api_key
def get_graph_data():
    """
    グラフ表示用の方位角別データをJSON形式で返すAPIエンドポイント（IMUデータがないためダミーデータ）。
    """
    with sensor_data.lock:
        data = {
            "azimuths": list(sensor_data.graph_azimuths),
            "values": list(sensor_data.graph_values)
        }
    return jsonify(data)

@app.route("/api/nmea_data", methods=["GET"])
@require_api_key
def get_nmea_data():
    """
    保存されているNMEA生データをJSON形式で返すAPIエンドポイント。
    """
    with sensor_data.lock:
        # dequeをリストに変換して返す
        return jsonify({"nmea_lines": list(sensor_data.nmea_buffer)})

# IMUキャリブレーションAPIは削除
# @app.route("/api/calibrate_imu", methods=["POST"])
# @require_api_key
# def calibrate_imu():
#     return jsonify({"error": "IMU calibration is not available as IMU functionality has been removed."}), 405 # Method Not Allowed


@app.route("/api/set_log_level", methods=["POST"])
@require_api_key
def set_log_level():
    """
    ログレベルを変更するAPIエンドポイント。
    """
    level_str = request.json.get('level', 'INFO').upper()
    try:
        level = getattr(logging, level_str)
        logger.setLevel(level)
        logging.getLogger().setLevel(level) # ルートロガーも変更
        logger.info(f"ログレベルを {level_str} に変更しました。")
        return jsonify({"status": f"Log level set to {level_str}"})
    except AttributeError:
        return jsonify({"error": f"無効なログレベル: {level_str}"}), 400

# --- アプリケーション実行 ---
def run_app():
    # スレッドの開始
    threading.Thread(target=read_gps_thread, args=(GPS_BASE_PORT, 'base'), daemon=True).start()
    threading.Thread(target=read_gps_thread, args=(GPS_ROVER_PORT, 'rover'), daemon=True).start()
    threading.Thread(target=calculate_heading_and_error_thread, daemon=True).start()

    # Flaskアプリケーションの実行 (HTTPS対応)
    cert_path = os.path.join(os.path.dirname(__file__), 'cert.pem')
    key_path = os.path.join(os.path.dirname(__file__), 'key.pem')

    if os.path.exists(cert_path) and os.path.exists(key_path):
        logger.info("SSL証明書と秘密鍵が見つかりました。HTTPSでサーバーを起動します。")
        # SSLContextの作成（推奨される方法）
        context = ssl.SSLContext(ssl.PROTOCOL_TLS_SERVER)
        try:
            context.load_cert_chain(cert_path, key_path)
            app.run(host='0.0.0.0', port=5000, debug=False, threaded=True, ssl_context=context)
        except ssl.SSLError as e:
            logger.critical(f"SSLコンテキストのロードエラー: {e}。HTTPでサーバーを起動します。証明書または鍵ファイルを確認してください。")
            app.run(host='0.0.0.0', port=5000, debug=False, threaded=True)
        except FileNotFoundError as e:
            logger.critical(f"SSL証明書または秘密鍵ファイルが見つかりません: {e}。HTTPでサーバーを起動します。")
            app.run(host='0.0.0.0', port=5000, debug=False, threaded=True)
    else:
        logger.warning("SSL証明書 (cert.pem) または秘密鍵 (key.pem) が見つかりません。HTTPでサーバーを起動します。")
        logger.warning("HTTPSを有効にするには、opensslコマンドでファイルを生成し、スクリプトと同じディレクトリに配置してください。")
        app.run(host='0.0.0.0', port=5000, debug=False, threaded=True)

if __name__ == "__main__":
    try:
        run_app()
    except KeyboardInterrupt:
        logger.info("アプリケーションをシャットダウンしています...")
        stop_event.set() # 全てのスレッドに停止を通知
        time.sleep(2) 
        logger.info("アプリケーションが終了しました。")
    except Exception as e:
        logger.critical(f"アプリケーションの予期せぬ終了: {e}")
        stop_event.set()
        time.sleep(2)
