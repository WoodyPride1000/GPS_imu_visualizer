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
import random # ダミーデータ生成用

from flask import Flask, jsonify, render_template, request
from geopy.distance import geodesic # geopyライブラリが必要: pip install geopy
from pyproj import Geod # 方位角計算用にGeodをインポート

# --- IMUライブラリのインポート ---
GLOBAL_IMU_MODULE_AVAILABLE = False
try:
    import smbus # MPU6050ライブラリでsmbusが必要なため
    from mpu6050 import MPU6050 # MPU6050クラスをインポート
    GLOBAL_IMU_MODULE_AVAILABLE = True
except ImportError as e:
    logging.warning(f"mpu6050ライブラリが見つからないか、依存関係エラー: {e}。IMUは無効になります。")
except Exception as e:
    logging.warning(f"IMUインポート中に予期せぬエラー: {e}。IMUは無効になります。")

# --- 設定ファイルの読み込み ---
config = configparser.ConfigParser()
config_file_path = os.path.join(os.path.dirname(__file__), 'config.ini')
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
        'MPU6050_I2C_BUS_NUM': '1', # Raspberry PiのI2Cバス番号
        'MPU6050_I2C_ADDR': '0x68', # MPU6050のI2Cアドレス (0x68 or 0x69)
        'IMU_READ_INTERVAL': '0.02', # IMUデータ読み取り間隔 (秒)
        'IMU_GYRO_OUTLIER_THRESHOLD': '3.0', # Z軸ジャイロ異常値検出閾値 (標準偏差の倍数)
        'IMU_MAX_RETRY_COUNT': '5', # IMU再初期化最大リトライ回数
        'SERIAL_RETRY_INTERVAL': '5' # シリアルポート/IMU再接続間隔 (秒)
    }
    with open(config_file_path, 'w') as configfile:
        config.write(configfile)
else:
    config.read(config_file_path)

API_KEY = os.environ.get('API_KEY', config['DEFAULT'].get('API_KEY', 'your_api_key_here'))
GPS_BASE_PORT = config['DEFAULT'].get('GPS_BASE_PORT', '/dev/ttyUSB0')
GPS_ROVER_PORT = config['DEFAULT'].get('GPS_ROVER_PORT', '/dev/ttyUSB1')
BAUDRATE = int(config['DEFAULT'].get('BAUDRATE', '115200'))
DUMMY_MODE = config['DEFAULT'].getboolean('DUMMY_MODE', False)
LOG_LEVEL_STR = config['DEFAULT'].get('LOG_LEVEL', 'INFO').upper()
MPU6050_I2C_BUS_NUM = int(config['DEFAULT'].get('MPU6050_I2C_BUS_NUM', '1'), 0) # 0xを考慮
MPU6050_I2C_ADDR = int(config['DEFAULT'].get('MPU6050_I2C_ADDR', '0x68'), 0) # 0xを考慮
IMU_READ_INTERVAL = float(config['DEFAULT'].get('IMU_READ_INTERVAL', '0.02'))
IMU_GYRO_OUTLIER_THRESHOLD = float(config['DEFAULT'].get('IMU_GYRO_OUTLIER_THRESHOLD', '3.0'))
IMU_MAX_RETRY_COUNT = int(config['DEFAULT'].get('IMU_MAX_RETRY_COUNT', '5'))
SERIAL_RETRY_INTERVAL = int(config['DEFAULT'].get('SERIAL_RETRY_INTERVAL', '5'))

# --- ロギング設定 ---
logging.basicConfig(level=getattr(logging, LOG_LEVEL_STR, logging.INFO),
                    format='%(asctime)s - %(levelname)s - %(message)s')
logger = logging.getLogger(__name__)

# --- グローバル定数とデータ構造 ---
MAX_CALIBRATION_SAMPLES = 500
IMU_GYRO_BUFFER_SIZE = 100 # ジャイロZ軸の移動平均計算用バッファサイズ
HEADING_SMOOTHING_WINDOW = 5 # 方位角の移動平均ウィンドウサイズ
HDOP_THRESHOLD = 2.0 # HDOPの閾値

# グローバルイベント
stop_event = threading.Event()

# --- データを保持するクラス ---
class SensorData:
    """GPSとIMUデータを管理し、スレッドセーフなデータアクセスを提供するクラス。"""
    def __init__(self):
        self.base_data: Dict[str, Any] = {'lat': 0.0, 'lon': 0.0, 'hdop': 99.9, 'quality': 0, 'timestamp': 0.0}
        self.rover_data: Dict[str, Any] = {'lat': 0.0, 'lon': 0.0, 'hdop': 99.9, 'quality': 0, 'timestamp': 0.0}
        self.heading_gps: float = 0.0 # GPSのみから算出された方位角
        self.heading_fused: float = 0.0 # IMU融合後の方位角
        self.heading_history: deque = deque(maxlen=HEADING_SMOOTHING_WINDOW) # 方位角平滑化用
        self.distance: float = 0.0
        self.error: float = 999.9
        self.imu_status: bool = False # IMUが正常に動作しているか
        self.imu_accel_x: float = 0.0
        self.imu_accel_y: float = 0.0
        self.imu_accel_z: float = 0.0
        self.imu_gyro_x: float = 0.0
        self.imu_gyro_y: float = 0.0
        self.imu_gyro_z: float = 0.0 # オフセット適用前の生のジャイロZ値
        self.imu_raw_gyro_z: float = 0.0 # Z軸のみ移動平均とオフセット適用後の値 (index.htmlで表示)
        
        # グラフ表示用の履歴データ (Chart.js用)
        self.graph_azimuths: deque = deque(maxlen=360) # 方位角のラベル用 (0-359)
        self.graph_values: deque = deque(maxlen=360)   # グラフの値用
        for i in range(360):
            self.graph_azimuths.append(i)
            self.graph_values.append(-50.0) # 仮の値で初期化 (index.htmlのグラフ範囲に合わせて-99から-20)

        self.lock = threading.Lock()
        self.last_fused_heading: float = 0.0
        self.last_imu_read_time: float = time.monotonic()
        self.calibration_mode: bool = False
        self.calibration_samples: List[float] = [] # キャリブレーション中の生ジャイロZ値
        self.gyro_z_offset: float = 0.0 # Z軸ジャイロのオフセット

        self.imu_device: Optional[MPU6050] = None # 型ヒントをMPU6050に変更
        self.gyro_z_buffer: deque = deque(maxlen=IMU_GYRO_BUFFER_SIZE) # オフセット適用後のバッファ

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

    def update_gps_data(self, target_key: str, lat: float, lon: float, hdop: float, quality: int) -> None:
        """GPSデータをスレッドセーフに更新し、接続状態をセットする。"""
        with self.lock:
            if target_key == 'base':
                self.base_data = {'lat': lat, 'lon': lon, 'hdop': hdop, 'quality': quality, 'timestamp': time.monotonic()}
                if not self.base_connected:
                    self.base_connected = True
                    self.base_connection_timestamp = time.monotonic()
                    logger.info(f"GPS Base ({GPS_BASE_PORT}) が接続されました。")
            else: # target_key == 'rover'
                self.rover_data = {'lat': lat, 'lon': lon, 'hdop': hdop, 'quality': quality, 'timestamp': time.monotonic()}
                if not self.rover_connected:
                    self.rover_connected = True
                    self.rover_connection_timestamp = time.monotonic()
                    logger.info(f"GPS Rover ({GPS_ROVER_PORT}) が接続されました。")
            self.data_updated_event.set()

    def update_imu_data(self, accel_x: float, accel_y: float, accel_z: float, gyro_x: float, gyro_y: float, gyro_z: float) -> None:
        """IMUの加速度とジャイロデータを更新し、ジャイロZ軸に移動平均と異常値除去を適用する。"""
        with self.lock:
            self.imu_accel_x = accel_x
            self.imu_accel_y = accel_y
            self.imu_accel_z = accel_z
            self.imu_gyro_x = gyro_x
            self.imu_gyro_y = gyro_y
            self.imu_gyro_z = gyro_z # 生のZ軸ジャイロ値を保存

            # キャリブレーションモードの場合、生の値をサンプルとして追加
            if self.calibration_mode:
                self.calibration_samples.append(gyro_z)
                return # キャリブレーション中は他の処理はスキップ

            # オフセットを適用
            corrected_z = gyro_z - self.gyro_z_offset

            # 異常値除去 (IMU_GYRO_OUTLIER_THRESHOLD を使用)
            # バッファに十分なデータがある場合のみ適用
            if len(self.gyro_z_buffer) > 1:
                # バッファの現在の平均と標準偏差を計算
                temp_buffer_list = list(self.gyro_z_buffer) + [corrected_z] # 一時的に現在の値も考慮
                mean = sum(temp_buffer_list) / len(temp_buffer_list)
                # 標準偏差は、バッファ内の既存データから計算するのが一般的
                # ここでは、簡単のためバッファと現在の値を含めて再計算
                if len(temp_buffer_list) > 1:
                    std = (sum((x - mean) ** 2 for x in temp_buffer_list) / (len(temp_buffer_list) -1)) ** 0.5 if len(temp_buffer_list) > 1 else 0
                else:
                    std = 0

                if std > 0.001 and abs(corrected_z - mean) > IMU_GYRO_OUTLIER_THRESHOLD * std:
                    logger.debug(f"IMU異常値検出 (raw: {gyro_z:.5f}, corrected: {corrected_z:.5f}, mean: {mean:.5f}, std: {std:.5f}, threshold: ±{IMU_GYRO_OUTLIER_THRESHOLD*std:.5f})。スキップします。")
                    return # 異常値はバッファに追加しない
            
            # 異常値でなければバッファに追加
            self.gyro_z_buffer.append(corrected_z)
            # 移動平均を計算してimu_raw_gyro_zに格納
            self.imu_raw_gyro_z = sum(self.gyro_z_buffer) / len(self.gyro_z_buffer) if self.gyro_z_buffer else 0.0
            
            self.imu_status = True
            self.last_imu_read_time = time.monotonic()
            self.data_updated_event.set()

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
            return jsonify({"error": "Unauthorized"}), 401
    return decorated_function

# --- IMU初期化 ---
def initialize_imu_device(sensor_data: SensorData) -> bool:
    """IMUデバイスの初期化を試みる。成功すればTrue、失敗すればFalseを返す。"""
    if DUMMY_MODE:
        with sensor_data.lock:
            sensor_data.imu_status = True
        logger.info("ダミーモードのためIMU初期化はスキップされます。")
        return True

    if not GLOBAL_IMU_MODULE_AVAILABLE:
        logger.debug("mpu6050ライブラリが利用できないため、IMU初期化をスキップします。")
        with sensor_data.lock:
            sensor_data.imu_status = False
        return False

    try:
        # MPU6050ライブラリのMPU6050クラスを使用
        # お使いのI2Cバス番号とアドレスをMPU6050_I2C_BUS_NUM, MPU6050_I2C_ADDRから取得
        sensor_data.imu_device = MPU6050(MPU6050_I2C_BUS_ADDR=MPU6050_I2C_BUS_NUM, MPU6050_I2C_ADDR=MPU6050_I2C_ADDR)
        # DMP初期化は、ファームウェアがDMPを搭載している場合のみ必要
        # sensor_data.imu_device.dmp_initialize() # コメントアウト: ライブラリのバージョンやDMPの有無に依存
        
        # センサーが応答するか簡単な読み取りで確認
        accel_test = sensor_data.imu_device.get_accel_data()
        if accel_test['x'] is None: raise ValueError("IMUがデータを返しませんでした。")

        with sensor_data.lock:
            sensor_data.imu_status = True
        logger.info("IMUが正常に初期化されました。")
        return True
    except Exception as e:
        logger.error(f"IMU初期化エラー: {e}。IMUは無効になります。")
        with sensor_data.lock:
            sensor_data.imu_status = False
        return False

# --- スレッド関数 ---
def read_gps_thread(port: str, station_type: str):
    """
    シリアルポートからGPSデータを読み取り、SensorDataを更新するスレッド。
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
            time.sleep(0.5) # ダミーデータの更新間隔
        return

    while not stop_event.is_set():
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

                while not stop_event.is_set():
                    line = ser.readline().decode('ascii', errors='ignore').strip()
                    if line.startswith('$GPGGA'):
                        parts = line.split(',')
                        if len(parts) >= 9: # 必要なフィールドがあるか確認
                            parsed_data = sensor_data._parse_gga(parts)
                            if parsed_data['lat'] != 0.0 or parsed_data['lon'] != 0.0: # 無効なデータでなければ更新
                                sensor_data.update_gps_data(station_type, parsed_data['lat'], parsed_data['lon'], parsed_data['hdop'], parsed_data['quality'])
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
            if not stop_event.is_set(): # アプリケーション停止中でない場合のみ再接続を試みる
                logger.warning(f"シリアルポート {port} から切断されました。{SERIAL_RETRY_INTERVAL}秒後に再接続を試みます...")
                time.sleep(SERIAL_RETRY_INTERVAL)
    logger.info(f"GPS {station_type} 読み取りスレッドを停止します。")

def read_imu_thread(sensor_data: SensorData):
    """IMUセンサーからデータを読み取り、SensorDataオブジェクトを更新するスレッド。
    IMUの再初期化ロジックと連続エラーログ抑制、リトライ上限を含む。
    """
    last_imu_reinit_error_log_time = 0
    retry_count = 0

    while not stop_event.is_set():
        if DUMMY_MODE:
            # ダミーの加速度とジャイロデータを生成
            dummy_accel_x = random.uniform(-1.0, 1.0)
            dummy_accel_y = random.uniform(-1.0, 1.0)
            dummy_accel_z = random.uniform(9.0, 10.0) # Z軸は重力方向を模倣
            dummy_gyro_x = random.uniform(-5.0, 5.0)
            dummy_gyro_y = random.uniform(-5.0, 5.0)
            dummy_gyro_z = random.uniform(-2.0, 2.0)
            sensor_data.update_imu_data(dummy_accel_x, dummy_accel_y, dummy_accel_z, dummy_gyro_x, dummy_gyro_y, dummy_gyro_z)
            time.sleep(IMU_READ_INTERVAL)
            continue

        with sensor_data.lock:
            imu_available = sensor_data.imu_device is not None and sensor_data.imu_status

        if not imu_available:
            if retry_count < IMU_MAX_RETRY_COUNT:
                if initialize_imu_device(sensor_data):
                    logger.info("IMUの再初期化に成功しました。")
                    retry_count = 0
                else:
                    retry_count += 1
                    current_log_time = time.monotonic()
                    if current_log_time - last_imu_reinit_error_log_time > SERIAL_RETRY_INTERVAL:
                        logger.warning(f"IMU再初期化失敗 ({retry_count}/{IMU_MAX_RETRY_COUNT})。{SERIAL_RETRY_INTERVAL}秒後に再試行します。")
                        last_imu_reinit_error_log_time = current_log_time
                    time.sleep(SERIAL_RETRY_INTERVAL)
            else:
                logger.error(f"IMU再初期化の上限回数 ({IMU_MAX_RETRY_COUNT}) に達しました。IMUスレッドを終了します。")
                with sensor_data.lock:
                    sensor_data.imu_status = False
                break # スレッドを終了
            continue # IMUが利用できない間は次のループへ

        try:
            # 加速度データの取得
            accel_data = sensor_data.imu_device.get_acceleration()
            # 角速度データの取得
            gyro_data = sensor_data.imu_device.get_rotation()
            
            # 取得したデータをSensorDataオブジェクトに更新
            sensor_data.update_imu_data(accel_data.x, accel_data.y, accel_data.z, gyro_data.x, gyro_data.y, gyro_data.z)
            retry_count = 0 # 成功したらリトライカウントをリセット
        except Exception as e:
            with sensor_data.lock:
                sensor_data.imu_status = False
            logger.error(f"IMU読み取りエラー: {e}。IMUステータスをFalseに設定しました。再接続を試みます。")
            retry_count += 1
            if not stop_event.is_set():
                time.sleep(SERIAL_RETRY_INTERVAL) # エラー時の待機

        time.sleep(IMU_READ_INTERVAL) # 正常時またはダミー時のインターバル
    logger.info("IMU読み取りスレッドを停止します。")


def calculate_heading_and_error_thread():
    """
    GPSデータとIMUデータを融合してヘディングと誤差を計算するスレッド。
    """
    logger.info("計算スレッドを開始します。")
    while not stop_event.is_set():
        sensor_data.data_updated_event.wait(timeout=1.0) # データ更新イベントを最大1秒待機
        sensor_data.data_updated_event.clear() # イベントをクリア

        with sensor_data.lock:
            base_lat = sensor_data.base_data['lat']
            base_lon = sensor_data.base_data['lon']
            rover_lat = sensor_data.rover_data['lat']
            rover_lon = sensor_data.rover_data['lon']
            imu_raw_gyro_z_avg = sensor_data.imu_raw_gyro_z # オフセット適用・移動平均後のジャイロZ
            imu_status = sensor_data.imu_status

            current_time = time.monotonic()
            # delta_time の計算はupdate_imu_dataと連動させる方が良い
            # ここでは便宜上、計算スレッドの更新間隔と同期
            delta_time = current_time - sensor_data.last_imu_read_time # IMUデータの最終更新時刻から計算
            if delta_time <= 0: delta_time = IMU_READ_INTERVAL # 極端に小さい場合はデフォルト値に

            # --- 基線長と誤差の計算 ---
            distance = geodesic((base_lat, base_lon), (rover_lat, rover_lon)).meters
            sensor_data.distance = distance

            # --- ヘディングの計算と融合 ---
            if base_lat != 0.0 and base_lon != 0.0 and rover_lat != 0.0 and rover_lon != 0.0:
                # GPSからの方位角計算
                fwd_azimuth, _, _ = geod.fwd(base_lon, base_lat, rover_lon, rover_lat)
                gps_heading = (fwd_azimuth + 360) % 360 # 0-360度に正規化
                sensor_data.heading_gps = gps_heading

                # 融合方位角の計算
                if imu_status and distance > 0.1: # IMUが有効で、かつ基線長がある程度ある場合
                    # ジャイロ積分による方位角の変化量
                    integrated_heading_change = imu_raw_gyro_z_avg * delta_time
                    imu_predicted_heading = (sensor_data.last_fused_heading + integrated_heading_change + 360) % 360

                    # GPS方位とIMU方位の融合 (例: 簡易的なカルマンフィルタ風)
                    # GPS品質が良い場合はGPS方位を強く反映
                    if sensor_data.base_data['quality'] >= 4 and sensor_data.rover_data['quality'] >= 4 and \
                       sensor_data.base_data['hdop'] < HDOP_THRESHOLD and sensor_data.rover_data['hdop'] < HDOP_THRESHOLD:
                        alpha = 0.8 # GPS信頼度 (0.0-1.0, 1.0に近いほどGPSを強く信用)
                        fused_heading = (alpha * gps_heading + (1 - alpha) * imu_predicted_heading + 360) % 360
                    else:
                        # GPS品質が低い場合はIMU予測値を重視
                        fused_heading = imu_predicted_heading
                    
                    sensor_data.heading_fused = fused_heading
                else:
                    # IMUが使えない、または基線長が短すぎる場合はGPS方位のみ
                    sensor_data.heading_fused = gps_heading
                
                sensor_data.last_fused_heading = sensor_data.heading_fused

            else:
                # GPSデータが不完全な場合
                sensor_data.heading_gps = 0.0
                if imu_status:
                    # IMUのみで更新を継続
                    integrated_heading_change = imu_raw_gyro_z_avg * delta_time
                    sensor_data.heading_fused = (sensor_data.last_fused_heading + integrated_heading_change + 360) % 360
                    sensor_data.last_fused_heading = sensor_data.heading_fused
                else:
                    sensor_data.heading_fused = 0.0
            
            # 方位角の平滑化 (移動平均)
            # heading_history に追加する前に 0-360 に正規化
            smoothed_heading = sensor_data.heading_fused
            if len(sensor_data.heading_history) == sensor_data.heading_history.maxlen and sensor_data.heading_history.maxlen > 0:
                # バッファが一杯の場合、古いデータを削除して新しいデータを追加
                sensor_data.heading_history.popleft()
            sensor_data.heading_history.append(smoothed_heading) # 融合済み・平滑化前の値を履歴に追加

            if len(sensor_data.heading_history) > 0:
                sensor_data.heading_fused = sum(sensor_data.heading_history) / len(sensor_data.heading_history)
                sensor_data.heading_fused = (sensor_data.heading_fused + 360) % 360 # 再び正規化

            # --- 基線誤差の計算 (仮のロジック) ---
            if sensor_data.base_data['quality'] == 4 and sensor_data.rover_data['quality'] == 4: # RTK Fix
                sensor_data.error = max(0.02, 0.005 * (sensor_data.base_data['hdop'] + sensor_data.rover_data['hdop']))
            elif sensor_data.base_data['quality'] == 5 and sensor_data.rover_data['quality'] == 5: # RTK Float
                sensor_data.error = max(0.05, 0.02 * (sensor_data.base_data['hdop'] + sensor_data.rover_data['hdop']))
            else: # Single or unknown
                sensor_data.error = max(0.5, 0.2 * (sensor_data.base_data['hdop'] + sensor_data.rover_data['hdop']))
            
            # グラフデータの更新 (例: 特定の方位角範囲のデータを更新)
            # ここではダミーとして、現在の方位角とimu_raw_gyro_z_avgを使ってグラフデータを更新する例
            # 実際には、複数の角度でのセンサー値や品質などを計測してここに格納します。
            # 例えば、方位角を1度刻みで0-359とし、それぞれの角度で何らかの値を記録・平均化するなど。
            if 0 <= int(sensor_data.heading_fused) < 360:
                idx = int(sensor_data.heading_fused)
                # Z軸角速度をグラフの表示範囲 (-99〜-20) に正規化
                # 例えば、-5度/秒を-20、+5度/秒を-99として線形変換
                # 実際の範囲に合わせて調整
                normalized_gyro_z_for_graph = max(-99.0, min(-20.0, -imu_raw_gyro_z_avg * 10 + (-50.0))) 
                sensor_data.graph_values[idx] = normalized_gyro_z_for_graph
            
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
            "imu_raw_gyro_z": round(sensor_data.imu_raw_gyro_z, 5), # オフセット適用・移動平均後のZ軸ジャイロ
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
                # キャリブレーション中に収集した生のジャイロZ値の平均をオフセットとする
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
    threading.Thread(target=read_gps_thread, args=(GPS_BASE_PORT, 'base'), daemon=True).start()
    threading.Thread(target=read_gps_thread, args=(GPS_ROVER_PORT, 'rover'), daemon=True).start()
    threading.Thread(target=read_imu_thread, args=(sensor_data,), daemon=True).start() # sensor_dataを引数で渡す
    threading.Thread(target=calculate_heading_and_error_thread, daemon=True).start()

    # Flaskアプリケーションの実行
    app.run(host='0.0.0.0', port=5000, debug=False, threaded=True)

if __name__ == "__main__":
    try:
        run_app()
    except KeyboardInterrupt:
        logger.info("アプリケーションをシャットダウンしています...")
        stop_event.set() # 全てのスレッドに停止を通知
        time.sleep(2) # スレッドが終了するのを待つ
        logger.info("アプリケーションが終了しました。")
