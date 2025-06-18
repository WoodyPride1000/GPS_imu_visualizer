import serial
import threading
import time
import json
import logging
from collections import deque
import math
from typing import Optional, Dict, Any, List
import configparser
import os
from functools import wraps

from flask import Flask, jsonify, render_template, request
from geopy.distance import geodesic # geopyライブラリが必要: pip install geopy

# IMUライブラリのインポート (お使いのIMUモジュールに合わせて適宜変更してください)
# 例: MPU6050の場合 (pip install smbus2 MPU6050-raspberrypi)
try:
    from mpu6050 import mpu6050 as MPU6050
    IMU_AVAILABLE = True
except ImportError:
    print("MPU6050ライブラリが見つかりません。IMU機能は無効になります。")
    IMU_AVAILABLE = False
except Exception as e:
    print(f"IMU初期化中にエラーが発生しました: {e} IMU機能は無効になります。")
    IMU_AVAILABLE = False

# --- 設定ファイルの読み込み ---
config = configparser.ConfigParser()
config_file_path = os.path.join(os.path.dirname(__file__), 'config.ini')
if not os.path.exists(config_file_path):
    print("config.iniが見つかりません。デフォルト設定を使用します。")
    # デフォルト設定の作成
    config['DEFAULT'] = {
        'API_KEY': 'your_api_key_here', # 必ず変更してください
        'BASE_STATION_PORT': '/dev/ttyUSB0', # Linux/macOS
        'ROVER_STATION_PORT': '/dev/ttyUSB1', # Linux/macOS
        # 'BASE_STATION_PORT': 'COM3', # Windowsの場合
        # 'ROVER_STATION_PORT': 'COM4', # Windowsの場合
        'BAUDRATE': '115200',
        'DUMMY_MODE': 'False',
        'LOG_LEVEL': 'INFO'
    }
    with open(config_file_path, 'w') as configfile:
        config.write(configfile)
else:
    config.read(config_file_path)

API_KEY = os.environ.get('API_KEY', config['DEFAULT'].get('API_KEY', 'your_api_key_here'))
BASE_STATION_PORT = config['DEFAULT'].get('BASE_STATION_PORT', '/dev/ttyUSB0')
ROVER_STATION_PORT = config['DEFAULT'].get('ROVER_STATION_PORT', '/dev/ttyUSB1')
BAUDRATE = int(config['DEFAULT'].get('BAUDRATE', '115200'))
DUMMY_MODE = config['DEFAULT'].getboolean('DUMMY_MODE', False)
LOG_LEVEL_STR = config['DEFAULT'].get('LOG_LEVEL', 'INFO').upper()

# --- ロギング設定 ---
logging.basicConfig(level=getattr(logging, LOG_LEVEL_STR, logging.INFO),
                    format='%(asctime)s - %(levelname)s - %(message)s')
logger = logging.getLogger(__name__)

# --- グローバル定数とデータ構造 ---
MAX_CALIBRATION_SAMPLES = 500
IMU_GYRO_BUFFER_SIZE = 100 # ジャイロZ軸の移動平均計算用バッファサイズ
HEADING_SMOOTHING_WINDOW = 5 # 方位角の移動平均ウィンドウサイズ
HDOP_THRESHOLD = 2.0 # HDOPの閾値

class SensorData:
    """GPSとIMUデータを管理し、スレッドセーフなデータアクセスを提供するクラス。"""
    def __init__(self):
        self.base_data: Dict[str, Any] = {'lat': 0.0, 'lon': 0.0, 'hdop': 99.9, 'quality': 0}
        self.rover_data: Dict[str, Any] = {'lat': 0.0, 'lon': 0.0, 'hdop': 99.9, 'quality': 0}
        self.imu_accel_x: float = 0.0
        self.imu_accel_y: float = 0.0
        self.imu_accel_z: float = 0.0
        self.imu_gyro_x: float = 0.0
        self.imu_gyro_y: float = 0.0
        self.imu_gyro_z: float = 0.0
        self.imu_status: bool = False
        self.heading_raw: float = 0.0 # GPSのみから算出された方位角
        self.heading_fused: float = 0.0 # IMU融合後の方位角
        self.heading_history: deque = deque(maxlen=HEADING_SMOOTHING_WINDOW) # 方位角平滑化用
        self.distance: float = 0.0
        self.error: float = 999.9

        # グラフ表示用の履歴データ (Chart.js用)
        self.graph_azimuths: deque = deque(maxlen=360) # 方位角のラベル用 (0-359)
        self.graph_values: deque = deque(maxlen=360)   # グラフの値用
                                                        # ここに実際の計測値などを格納するロジックを後で追加
        # 初期のグラフデータ（例として全方位で-50のデータを格納）
        for i in range(360):
            self.graph_azimuths.append(i)
            self.graph_values.append(-50.0) # 仮の値

        self.lock = threading.Lock()
        self.last_fused_heading: float = 0.0 # 最後に更新された融合方位角
        self.last_imu_read_time: float = time.monotonic()

        self.calibration_mode: bool = False
        self.calibration_samples: List[float] = []
        self.gyro_z_offset: float = 0.0 # Z軸ジャイロのオフセット

        self.imu_device: Optional[MPU6050] = None
        self.gyro_z_buffer: deque = deque(maxlen=IMU_GYRO_BUFFER_SIZE) # 生のZ軸ジャイロ値バッファ

        self.data_updated_event = threading.Event()
        self.base_connected: bool = False
        self.rover_connected: bool = False
        self.base_connection_timestamp: float = 0.0
        self.rover_connection_timestamp: float = 0.0

        # エラーカウンター
        self.base_port_errors: int = 0
        self.base_serial_errors: int = 0
        self.rover_port_errors: int = 0
        self.rover_serial_errors: int = 0

    def _parse_gga(self, parts: List[str]) -> Dict[str, Any]:
        """GPGGAセンテンスを解析し、緯度、経度、HDOP、品質を抽出します。"""
        try:
            lat_raw = float(parts[2])
            lon_raw = float(parts[4])
            quality = int(parts[6])
            hdop = float(parts[8])

            lat_deg = int(lat_raw / 100)
            lat_min = lat_raw % 100
            lat = lat_deg + lat_min / 60
            if parts[3] == 'S':
                lat *= -1

            lon_deg = int(lon_raw / 100)
            lon_min = lon_raw % 100
            lon = lon_deg + lon_min / 60
            if parts[5] == 'W':
                lon *= -1

            return {'lat': lat, 'lon': lon, 'hdop': hdop, 'quality': quality}
        except (ValueError, IndexError) as e:
            logger.warning(f"GPGGAパースエラー: {e} - データ: {parts}")
            return {'lat': 0.0, 'lon': 0.0, 'hdop': 99.9, 'quality': 0}

    def update_gps_data(self, line: str, station_type: str):
        """GPSデータを更新します。"""
        parts = line.strip().split(',')
        if len(parts) > 0 and parts[0] == '$GPGGA':
            data = self._parse_gga(parts)
            with self.lock:
                if station_type == 'base':
                    self.base_data.update(data)
                elif station_type == 'rover':
                    self.rover_data.update(data)
                self.data_updated_event.set() # データが更新されたことを通知

    def update_imu_data(self):
        """IMUデータを読み取り、更新します。"""
        if not IMU_AVAILABLE or self.imu_device is None:
            self.imu_status = False
            return

        try:
            accel_data = self.imu_device.get_accel_data()
            gyro_data = self.imu_device.get_gyro_data()

            with self.lock:
                self.imu_accel_x = accel_data['x']
                self.imu_accel_y = accel_data['y']
                self.imu_accel_z = accel_data['z']
                self.imu_gyro_x = gyro_data['x']
                self.imu_gyro_y = gyro_data['y']
                self.imu_gyro_z = gyro_data['z'] - self.gyro_z_offset # オフセットを適用

                # 生のZ軸ジャイロ値をバッファに保存（オフセット適用前）
                self.gyro_z_buffer.append(gyro_data['z'])

                self.imu_status = True
                self.data_updated_event.set() # データが更新されたことを通知
        except Exception as e:
            logger.error(f"IMUデータの読み取りエラー: {e}")
            self.imu_status = False

sensor_data = SensorData() # SensorDataクラスのインスタンス化

# --- Flaskアプリケーションの初期化 ---
app = Flask(__name__)

# --- APIキー認証デコレータ ---
def require_api_key(view_function):
    @wraps(view_function)
    def decorated_function(*args, **kwargs):
        if request.headers.get('X-API-KEY') and request.headers.get('X-API-KEY') == API_KEY:
            return view_function(*args, **kwargs)
        else:
            return jsonify({"error": "Unauthorized"}), 401
    return decorated_function

# --- ヘルパー関数 ---
def initialize_imu_device():
    """IMUデバイスを初期化します。"""
    if IMU_AVAILABLE:
        try:
            # アドレス0x68または0x69でMPU6050を検索
            # Raspberry Piの場合、I2Cはデフォルトで有効になっている必要があります
            # i2cdetect -y 1 でデバイスアドレスを確認
            sensor_data.imu_device = MPU6050(0x68)
            logger.info("IMU (MPU6050) が初期化されました。")
            sensor_data.imu_status = True
        except Exception as e:
            logger.error(f"IMU (MPU6050) の初期化に失敗しました: {e}")
            sensor_data.imu_device = None
            sensor_data.imu_status = False

def calculate_heading(lat1, lon1, lat2, lon2):
    """
    2点間の真北からの方位角（ヘディング）を計算します。
    Args:
        lat1, lon1: 1点目の緯度、経度
        lat2, lon2: 2点目の緯度、経度
    Returns:
        float: 真北からの角度（0〜360度）
    """
    if lat1 == lat2 and lon1 == lon2:
        return 0.0 # 同じ位置の場合は0を返すか、エラーとして扱う
    
    phi1 = math.radians(lat1)
    lambda1 = math.radians(lon1)
    phi2 = math.radians(lat2)
    lambda2 = math.radians(lon2)

    delta_lambda = lambda2 - lambda1

    y = math.sin(delta_lambda) * math.cos(phi2)
    x = math.cos(phi1) * math.sin(phi2) - math.sin(phi1) * math.cos(phi2) * math.cos(delta_lambda)

    bearing = math.degrees(math.atan2(y, x))
    bearing = (bearing + 360) % 360 # 0から360度の範囲に正規化
    return bearing

def calculate_distance(lat1, lon1, lat2, lon2):
    """
    2点間の距離を計算します (メートル単位)。
    Args:
        lat1, lon1: 1点目の緯度、経度
        lat2, lon2: 2点目の緯度、経度
    Returns:
        float: 距離 (メートル)
    """
    if lat1 == 0.0 and lon1 == 0.0 and lat2 == 0.0 and lon2 == 0.0:
        return 0.0 # データがない場合は0を返す
    try:
        return geodesic((lat1, lon1), (lat2, lon2)).meters
    except ValueError:
        return 0.0

# --- スレッド関数 ---
def read_gps_thread(port: str, station_type: str):
    """
    シリアルポートからGPSデータを読み取り、SensorDataを更新するスレッド。
    """
    logger.info(f"{station_type} GPSスレッドを開始します: {port}")
    if DUMMY_MODE:
        logger.info(f"ダミーモード: {station_type} GPSデータは生成されます。")
        while True:
            time.sleep(1) # 1秒ごとにダミーデータを更新
            with sensor_data.lock:
                current_time = time.time()
                # 少しずつ位置をずらすダミーデータ
                if station_type == 'base':
                    sensor_data.base_data['lat'] = 35.681236 + math.sin(current_time / 10) * 0.0001
                    sensor_data.base_data['lon'] = 139.767125 + math.cos(current_time / 10) * 0.0001
                    sensor_data.base_data['hdop'] = 0.8 + math.sin(current_time / 5) * 0.1
                    sensor_data.base_data['quality'] = 4 # RTK Fix
                    sensor_data.base_connected = True
                elif station_type == 'rover':
                    sensor_data.rover_data['lat'] = 35.681236 + math.sin(current_time / 10 + 0.0005) * 0.0001
                    sensor_data.rover_data['lon'] = 139.767125 + math.cos(current_time / 10 + 0.0005) * 0.0001
                    sensor_data.rover_data['hdop'] = 0.9 + math.cos(current_time / 5) * 0.1
                    sensor_data.rover_data['quality'] = 4 # RTK Fix
                    sensor_data.rover_connected = True
                sensor_data.data_updated_event.set()
        return

    while True:
        try:
            with serial.Serial(port, BAUDRATE, timeout=1) as ser:
                logger.info(f"シリアルポート {port} に接続しました ({station_type})")
                with sensor_data.lock:
                    if station_type == 'base':
                        sensor_data.base_connected = True
                        sensor_data.base_connection_timestamp = time.monotonic()
                    elif station_type == 'rover':
                        sensor_data.rover_connected = True
                        sensor_data.rover_connection_timestamp = time.monotonic()

                while True:
                    line = ser.readline().decode('ascii', errors='ignore')
                    if line.startswith('$GPGGA'):
                        sensor_data.update_gps_data(line, station_type)
                    # 適度なポーリング間隔
                    time.sleep(0.01) # 短いインターバルでCPU使用率を抑える
        except serial.SerialException as e:
            with sensor_data.lock:
                if station_type == 'base':
                    sensor_data.base_port_errors += 1
                    sensor_data.base_connected = False
                elif station_type == 'rover':
                    sensor_data.rover_port_errors += 1
                    sensor_data.rover_connected = False
            logger.error(f"シリアルポート {port} のエラー: {e}")
        except Exception as e:
            with sensor_data.lock:
                if station_type == 'base':
                    sensor_data.base_serial_errors += 1
                elif station_type == 'rover':
                    sensor_data.rover_serial_errors += 1
            logger.error(f"GPS読み取り中に予期せぬエラー: {e}")
        finally:
            with sensor_data.lock:
                if station_type == 'base':
                    sensor_data.base_connected = False
                elif station_type == 'rover':
                    sensor_data.rover_connected = False
            logger.warning(f"シリアルポート {port} から切断されました。再接続を試みます...")
            time.sleep(3) # 再接続までの待機時間

def read_imu_thread():
    """IMUデータを定期的に読み取るスレッド。"""
    if not IMU_AVAILABLE:
        logger.warning("IMUが使用できないため、IMUスレッドは起動しません。")
        return

    initialize_imu_device()
    if sensor_data.imu_device is None:
        logger.error("IMUデバイスが利用できないため、IMUスレッドを終了します。")
        return

    logger.info("IMU読み取りスレッドを開始します。")
    while True:
        sensor_data.update_imu_data()
        time.sleep(0.01) # IMUポーリング間隔 (Hz)

def calculate_heading_and_error_thread():
    """
    GPSデータとIMUデータを融合してヘディングと誤差を計算するスレッド。
    """
    logger.info("計算スレッドを開始します。")
    while True:
        sensor_data.data_updated_event.wait() # データ更新イベントを待機
        sensor_data.data_updated_event.clear() # イベントをクリア

        with sensor_data.lock:
            base_lat = sensor_data.base_data['lat']
            base_lon = sensor_data.base_data['lon']
            rover_lat = sensor_data.rover_data['lat']
            rover_lon = sensor_data.rover_data['lon']
            imu_gyro_z = sensor_data.imu_gyro_z # オフセット適用済みのジャイロZ
            imu_status = sensor_data.imu_status

            current_time = time.monotonic()
            delta_time = current_time - sensor_data.last_imu_read_time
            sensor_data.last_imu_read_time = current_time

            # --- 基線長と誤差の計算 ---
            distance = calculate_distance(base_lat, base_lon, rover_lat, rover_lon)
            sensor_data.distance = distance

            # --- ヘディングの計算と融合 ---
            if base_lat != 0.0 and base_lon != 0.0 and rover_lat != 0.0 and rover_lon != 0.0:
                gps_heading = calculate_heading(base_lat, base_lon, rover_lat, rover_lon)
                sensor_data.heading_raw = gps_heading

                # 安定したGPSデータがある場合、IMUオフセットを微調整
                if sensor_data.base_data['quality'] >= 4 and sensor_data.rover_data['quality'] >= 4 and \
                   sensor_data.base_data['hdop'] < HDOP_THRESHOLD and sensor_data.rover_data['hdop'] < HDOP_THRESHOLD and \
                   distance > 0.5: # ある程度の距離がないと方位は不安定
                   # GPS方位とIMU方位の差を見て、gyro_z_offsetを調整するロジックをここに追加可能
                   pass # 現状では調整はしない

                if imu_status:
                    # ジャイロデータによる方位角の更新 (積分)
                    # 前回の融合方位角に現在の角速度(度/秒)を時間刻みで加算
                    integrated_heading_change = imu_gyro_z * delta_time
                    imu_heading = (sensor_data.last_fused_heading + integrated_heading_change + 360) % 360
                    
                    # GPS方位とIMU方位を融合
                    # シンプルな融合（GPSを優先しつつ、IMUで滑らかにする）
                    # 実際のRTK-IMU融合ではカルマンフィルタなど高度な手法が使われる
                    if sensor_data.base_data['quality'] >= 4 and sensor_data.rover_data['quality'] >= 4 and \
                       sensor_data.base_data['hdop'] < HDOP_THRESHOLD and sensor_data.rover_data['hdop'] < HDOP_THRESHOLD:
                        # GPSの品質が高い場合、GPS方位を強く反映
                        # 例えば、GPS方位とIMU方位の差が小さい場合はIMUで補間、大きい場合はGPSに近づけるなど
                        alpha = 0.7 # GPSへの信頼度 (0.0 - 1.0, 高いほどGPS優先)
                        fused_heading = (alpha * gps_heading + (1 - alpha) * imu_heading + 360) % 360
                    else:
                        # GPSの品質が低い場合、IMUのみで更新
                        fused_heading = imu_heading
                    
                    sensor_data.heading_fused = fused_heading
                else:
                    # IMUがない場合、GPS方位をそのまま使う
                    sensor_data.heading_fused = gps_heading

                sensor_data.last_fused_heading = sensor_data.heading_fused
            else:
                # GPSデータが不完全な場合
                sensor_data.heading_raw = 0.0
                if imu_status:
                    # IMUのみで更新を継続
                    integrated_heading_change = imu_gyro_z * delta_time
                    sensor_data.heading_fused = (sensor_data.last_fused_heading + integrated_heading_change + 360) % 360
                    sensor_data.last_fused_heading = sensor_data.heading_fused
                else:
                    sensor_data.heading_fused = 0.0
            
            # 方位角の平滑化 (移動平均)
            sensor_data.heading_history.append(sensor_data.heading_fused)
            if len(sensor_data.heading_history) > 0:
                sensor_data.heading_fused = sum(sensor_data.heading_history) / len(sensor_data.heading_history)
                sensor_data.heading_fused = (sensor_data.heading_fused + 360) % 360 # 再び正規化

            # --- 基線誤差の計算 (仮のロジック) ---
            # 実際にはRTKの解の状態 (Fix/Float/Single) やHDOP、RMS値などから算出
            if sensor_data.base_data['quality'] == 4 and sensor_data.rover_data['quality'] == 4:
                sensor_data.error = max(0.05, 0.01 * (sensor_data.base_data['hdop'] + sensor_data.rover_data['hdop']))
            elif sensor_data.base_data['quality'] == 5 and sensor_data.rover_data['quality'] == 5: # RTK Float
                sensor_data.error = max(0.1, 0.05 * (sensor_data.base_data['hdop'] + sensor_data.rover_data['hdop']))
            else: # Single or unknown
                sensor_data.error = max(1.0, 0.5 * (sensor_data.base_data['hdop'] + sensor_data.rover_data['hdop']))
            
            # グラフデータの更新 (例: 特定の方位角範囲のデータを更新)
            # ここではダミーとして、現在の方位角とZ軸ジャイロ値を使ってグラフデータを更新する例
            # 実際には、複数の角度でのセンサー値や品質などを計測してここに格納します。
            # 例えば、方位角を1度刻みで0-359とし、それぞれの角度で何らかの値を記録・平均化するなど。
            if 0 <= int(sensor_data.heading_fused) < 360:
                idx = int(sensor_data.heading_fused)
                # ここでは、z軸角速度をそのまま値として利用する例
                # マイナス値になる可能性があるので、表示範囲に合わせて調整する
                normalized_gyro_z = max(-99.0, min(-20.0, sensor_data.imu_gyro_z * 10 - 50.0)) # 適当な範囲に正規化
                sensor_data.graph_values[idx] = normalized_gyro_z
            
        time.sleep(0.05) # 計算スレッドの実行間隔

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
    現在のGPS、IMU、および計算されたヘディング・誤差データをJSON形式で返すAPIエンドポイント。
    index.htmlのJavaScriptが期待するフィールド名に合わせる。
    """
    with sensor_data.lock:
        data = {
            "lat": round(sensor_data.base_data['lat'], 7),
            "lon": round(sensor_data.base_data['lon'], 7),
            "hdop_base": round(sensor_data.base_data['hdop'], 2),
            "base_quality": sensor_data.base_data['quality'], # JSでは使われないがデバッグ用
            "rover_lat": round(sensor_data.rover_data['lat'], 7), # JSでは使われない
            "rover_lon": round(sensor_data.rover_data['lon'], 7), # JSでは使われない
            "hdop_rover": round(sensor_data.rover_data['hdop'], 2),
            "rover_quality": sensor_data.rover_data['quality'], # JSでは使われないがデバッグ用
            "heading": round(sensor_data.heading_fused, 2),
            "error": round(sensor_data.error, 3),
            "distance": round(sensor_data.distance, 3),
            "imu": sensor_data.imu_status,
            "imu_accel_x": round(sensor_data.imu_accel_x, 5), # JSでは使われない
            "imu_accel_y": round(sensor_data.imu_accel_y, 5), # JSでは使われない
            "imu_accel_z": round(sensor_data.imu_accel_z, 5), # JSでは使われない
            "imu_gyro_x": round(sensor_data.imu_gyro_x, 5), # JSでは使われない
            "imu_gyro_y": round(sensor_data.imu_gyro_y, 5), # JSでは使われない
            "imu_raw_gyro_z": round(sum(sensor_data.gyro_z_buffer) / len(sensor_data.gyro_z_buffer) if sensor_data.gyro_z_buffer else 0.0, 5), # 生の平均値
            "gyro_z_offset": round(sensor_data.gyro_z_offset, 5),
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
    グラフ表示用の方位角別データをJSON形式で返すAPIエンドポイント。
    """
    with sensor_data.lock:
        data = {
            "azimuths": list(sensor_data.graph_azimuths),
            "values": list(sensor_data.graph_values)
        }
    return jsonify(data)

@app.route("/api/calibrate_imu", methods=["POST"])
@require_api_key
def calibrate_imu():
    """
    IMUのジャイロZ軸オフセットをキャリブレーションするAPIエンドポイント。
    """
    action = request.json.get('action')
    with sensor_data.lock:
        if action == 'start':
            sensor_data.calibration_mode = True
            sensor_data.calibration_samples = []
            sensor_data.gyro_z_offset = 0.0 # キャリブレーション開始時にオフセットをリセット
            logger.info("IMUキャリブレーションを開始します...")
            return jsonify({"status": "Calibration started."})
        elif action == 'stop':
            sensor_data.calibration_mode = False
            if sensor_data.calibration_samples:
                # 移動平均バッファ内の生のジャイロZ値の平均をオフセットとする
                sensor_data.gyro_z_offset = sum(sensor_data.calibration_samples) / len(sensor_data.calibration_samples)
                logger.info(f"IMUキャリブレーションが完了しました。Z軸オフセット: {sensor_data.gyro_z_offset:.5f}")
            else:
                sensor_data.gyro_z_offset = 0.0
                logger.warning("IMUキャリブレーション: サンプルがありませんでした。オフセットは0に設定されました。")
            return jsonify({"status": "Calibration stopped.", "offset": sensor_data.gyro_z_offset})
        else:
            return jsonify({"error": "Invalid action."}), 400

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
    threading.Thread(target=read_gps_thread, args=(BASE_STATION_PORT, 'base'), daemon=True).start()
    threading.Thread(target=read_gps_thread, args=(ROVER_STATION_PORT, 'rover'), daemon=True).start()
    threading.Thread(target=read_imu_thread, daemon=True).start()
    threading.Thread(target=calculate_heading_and_error_thread, daemon=True).start()

    # Flaskアプリケーションの実行
    # 本番環境ではGunicornなどのWSGIサーバーを使用してください
    app.run(host='0.0.0.0', port=5000, debug=False, threaded=True) # debug=Trueは開発用

if __name__ == "__main__":
    run_app()
