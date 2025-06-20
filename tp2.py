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

# --- IMUライブラリのインポート ---
GLOBAL_IMU_MODULE_AVAILABLE = False
try:
    import smbus # MPU6050ライブラリでsmbusが必要なため
    from mpu6050 import MPU6050 # MPU6050クラスをインポート
    GLOBAL_IMU_MODULE_AVAILABLE = True
except ImportError as e:
    # ロギング設定前に警告を出力し、その後ロガーを使用
    print(f"WARNING: mpu6050ライブラリが見つからないか、依存関係エラー: {e}。IMUは無効になります。")
except Exception as e:
    print(f"WARNING: IMUインポート中に予期せぬエラー: {e}。IMUは無効になります。")

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

    # 設定値の取得と型変換
    # 環境変数からのAPIキー優先
    return {
        'API_KEY': os.environ.get('API_KEY', config['DEFAULT'].get('API_KEY', 'your_api_key_here')),
        'GPS_BASE_PORT': config['DEFAULT'].get('GPS_BASE_PORT', '/dev/ttyUSB0'),
        'GPS_ROVER_PORT': config['DEFAULT'].get('GPS_ROVER_PORT', '/dev/ttyUSB1'),
        'BAUDRATE': int(config['DEFAULT'].get('BAUDRATE', '115200')),
        'DUMMY_MODE': config['DEFAULT'].getboolean('DUMMY_MODE', False),
        'LOG_LEVEL_STR': config['DEFAULT'].get('LOG_LEVEL', 'INFO').upper(),
        'MPU6050_I2C_BUS_NUM': int(config['DEFAULT'].get('MPU6050_I2C_BUS_NUM', '1'), 0),
        'MPU6050_I2C_ADDR': int(config['DEFAULT'].get('MPU6050_I2C_ADDR', '0x68'), 0),
        'IMU_READ_INTERVAL': float(config['DEFAULT'].get('IMU_READ_INTERVAL', '0.02')),
        'IMU_GYRO_OUTLIER_THRESHOLD': float(config['DEFAULT'].get('IMU_GYRO_OUTLIER_THRESHOLD', '3.0')),
        'IMU_MAX_RETRY_COUNT': int(config['DEFAULT'].get('IMU_MAX_RETRY_COUNT', '5')),
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
MPU6050_I2C_BUS_NUM = APP_CONFIG['MPU6050_I2C_BUS_NUM']
MPU6050_I2C_ADDR = APP_CONFIG['MPU6050_I2C_ADDR']
IMU_READ_INTERVAL = APP_CONFIG['IMU_READ_INTERVAL']
IMU_GYRO_OUTLIER_THRESHOLD = APP_CONFIG['IMU_GYRO_OUTLIER_THRESHOLD']
IMU_MAX_RETRY_COUNT = APP_CONFIG['IMU_MAX_RETRY_COUNT']
SERIAL_RETRY_INTERVAL = APP_CONFIG['SERIAL_RETRY_INTERVAL']

# その他の定数
MAX_CALIBRATION_SAMPLES = 500 # キャリブレーションサンプル数の上限
IMU_GYRO_BUFFER_SIZE = 100 # ジャイロZ軸の移動平均計算用バッファサイズ
HEADING_SMOOTHING_WINDOW = 5 # 方位角の移動平均ウィンドウサイズ
HDOP_THRESHOLD = 2.0 # HDOPの閾値
NMEA_BUFFER_SIZE = 100 # NMEA表示用バッファサイズ（行数）

# グローバルイベント
stop_event = threading.Event()

# --- データを保持するクラス ---
class SensorData:
    """GPSとIMUデータを管理し、スレッドセーフなデータアクセスを提供するクラス。"""
    def __init__(self):
        self.lock = threading.Lock()
        self.data_updated_event = threading.Event()

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

        # IMUデータ
        self.imu_status: bool = False
        self.imu_accel_x: float = 0.0
        self.imu_accel_y: float = 0.0
        self.imu_accel_z: float = 0.0
        self.imu_gyro_x: float = 0.0
        self.imu_gyro_y: float = 0.0
        self.imu_gyro_z: float = 0.0 # オフセット適用前の生のジャイロZ値
        self.imu_raw_gyro_z: float = 0.0 # Z軸のみ移動平均とオフセット適用後の値 (index.htmlで表示)
        self.gyro_z_buffer: deque = deque(maxlen=IMU_GYRO_BUFFER_SIZE) # オフセット適用後のバッファ
        self.gyro_z_offset: float = 0.0 # Z軸ジャイロのオフセット
        self.last_imu_read_time: float = time.monotonic()
        self.imu_device: Optional[MPU6050] = None

        # キャリブレーション
        self.calibration_mode: bool = False
        self.calibration_samples: List[float] = [] # キャリブレーション中の生ジャイロZ値

        # 融合データ
        self.heading_gps: float = 0.0 # GPSのみから算出された方位角 (基準局から移動局への方向)
        self.heading_fused: float = 0.0 # IMU融合後の方位角
        self.heading_history: deque = deque(maxlen=HEADING_SMOOTHING_WINDOW) # 方位角平滑化用
        self.distance: float = 0.0
        self.error: float = 999.9
        self.last_fused_heading: float = 0.0 # 前回の融合方位角

        # グラフ表示用の履歴データ (Chart.js用)
        self.graph_azimuths: deque = deque(range(360)) # 0-359
        self.graph_values: deque = deque([-50.0] * 360) # 仮の値で初期化 (index.htmlのグラフ範囲に合わせて-99から-20)

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

    def update_imu_data(self, accel_x: float, accel_y: float, accel_z: float, gyro_x: float, gyro_y: float, gyro_z: float) -> None:
        """IMUの加速度とジャイロデータを更新し、ジャイロZ軸に移動平均と異常値除去を適用する。"""
        with self.lock:
            self.imu_accel_x = accel_x
            self.imu_accel_y = accel_y
            self.imu_accel_z = accel_z
            self.imu_gyro_x = gyro_x
            self.imu_gyro_y = gyro_y
            self.imu_gyro_z = gyro_z # 生のZ軸ジャイロ値を保存
            self.last_imu_read_time = time.monotonic()

            if self.calibration_mode:
                self.calibration_samples.append(gyro_z)
                return # キャリブレーション中は他の処理はスキップ

            corrected_z = gyro_z - self.gyro_z_offset

            # 異常値除去 (IMU_GYRO_OUTLIER_THRESHOLD を使用)
            # バッファに十分なデータがあり、標準偏差が計算可能な場合のみ適用
            if len(self.gyro_z_buffer) >= 5: # 少なくとも5サンプルあれば標準偏差がより意味を持つ
                # バッファの現在の平均と標準偏差を計算（現在の値は含めない）
                mean = sum(self.gyro_z_buffer) / len(self.gyro_z_buffer)
                std_dev = (sum((x - mean) ** 2 for x in self.gyro_z_buffer) / (len(self.gyro_z_buffer) -1)) ** 0.5 if len(self.gyro_z_buffer) > 1 else 0

                if std_dev > 0.001 and abs(corrected_z - mean) > IMU_GYRO_OUTLIER_THRESHOLD * std_dev:
                    logger.debug(f"IMU異常値検出 (生: {gyro_z:.5f}, 補正後: {corrected_z:.5f}, 平均: {mean:.5f}, 標準偏差: {std_dev:.5f}, 閾値: ±{IMU_GYRO_OUTLIER_THRESHOLD*std_dev:.5f})。スキップします。")
                    return # 異常値はバッファに追加しない
            
            # 異常値でなければバッファに追加
            self.gyro_z_buffer.append(corrected_z)
            self.imu_raw_gyro_z = sum(self.gyro_z_buffer) / len(self.gyro_z_buffer) if self.gyro_z_buffer else 0.0
            
            self.imu_status = True # データが更新されればIMUは動作中と見なす
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
        sensor_data.imu_device = MPU6050(MPU6050_I2C_BUS_ADDR=MPU6050_I2C_BUS_NUM, MPU6050_I2C_ADDR=MPU6050_I2C_ADDR)
        
        # センサーが応答するか簡単な読み取りで確認
        accel_test = sensor_data.imu_device.get_accel_data()
        # get_accel_dataは辞書を返すため、'x'キーが存在し、かつ数値であることを確認
        if not isinstance(accel_test, dict) or 'x' not in accel_test or not isinstance(accel_test['x'], (int, float)):
            raise ValueError("IMUが有効な加速度データを返しませんでした。")

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
    GPSデータとIMUデータを融合してヘディングと誤差を計算するスレッド。
    """
    logger.info("計算スレッドを開始します。")
    while not stop_event.is_set():
        # データ更新イベントを最大1秒待機。IMUやGPSデータが更新されるまで待つ
        # これにより、新しいデータが利用可能になったときに計算がトリガーされる
        sensor_data.data_updated_event.wait(timeout=1.0) 
        sensor_data.data_updated_event.clear() # イベントをクリア

        with sensor_data.lock:
            base_lat = sensor_data.base_data['lat']
            base_lon = sensor_data.base_data['lon']
            rover_lat = sensor_data.rover_data['lat']
            rover_lon = sensor_data.rover_data['lon']
            imu_raw_gyro_z_avg = sensor_data.imu_raw_gyro_z # オフセット適用・移動平均後のジャイロZ
            imu_status = sensor_data.imu_status
            
            # IMUデータの最終読み取り時刻を基準にdelta_timeを計算
            current_time_calc = time.monotonic()
            delta_time = current_time_calc - sensor_data.last_imu_read_time
            # 初回や時間が進んでいない場合、デフォルトのIMU読み取り間隔を使用
            if delta_time <= 0:
                delta_time = IMU_READ_INTERVAL 

            # --- 基線長と誤差の計算 ---
            distance = geodesic((base_lat, base_lon), (rover_lat, rover_lon)).meters
            sensor_data.distance = distance

            # --- ヘディングの計算と融合 ---
            if base_lat != 0.0 and base_lon != 0.0 and rover_lat != 0.0 and rover_lon != 0.0:
                # GPSからの方位角計算
                fwd_azimuth, _, _ = geod.fwd(base_lon, base_lat, rover_lon, rover_lat)
                gps_heading = (fwd_azimuth + 360) % 360 # 0-360度に正規化
                sensor_data.heading_gps = gps_heading # ここで更新

                # 融合方位角の計算
                # IMUが有効で、かつ基線長がある程度ある場合（かつ前回の方位角が有効な場合）
                if imu_status and distance > 0.1 and sensor_data.last_fused_heading != 0.0:
                    # ジャイロ積分による方位角の変化量 (ジャイロデータはdps: degrees per second を想定)
                    integrated_heading_change = imu_raw_gyro_z_avg * delta_time
                    imu_predicted_heading = (sensor_data.last_fused_heading + integrated_heading_change + 360) % 360

                    # GPS方位とIMU方位の融合 (簡易的なカルマンフィルタ風)
                    # GPS品質が良い場合はGPS方位を強く反映
                    if sensor_data.base_data['quality'] >= 4 and sensor_data.rover_data['quality'] >= 4 and \
                       sensor_data.base_data['hdop'] < HDOP_THRESHOLD and sensor_data.rover_data['hdop'] < HDOP_THRESHOLD:
                        # GPSとIMUの差を計算し、最短経路で補正（-180〜180度の範囲に正規化して計算）
                        diff = (gps_heading - imu_predicted_heading + 180 + 360) % 360 - 180
                        alpha = 0.8 # GPS信頼度 (0.0-1.0, 1.0に近いほどGPSを強く信用)
                        fused_heading = (imu_predicted_heading + alpha * diff + 360) % 360
                    else:
                        # GPS品質が低い場合はIMU予測値を重視
                        fused_heading = imu_predicted_heading
                    
                    sensor_data.heading_fused = fused_heading
                else:
                    # IMUが使えない、または基線長が短すぎる場合はGPS方位のみ
                    sensor_data.heading_fused = gps_heading
                
                # 方位角の平滑化のために履歴に追加
                # 角度の単純平均は、0度と359度のような値が平均すると大きくずれる問題があるため注意
                # より正確な移動平均（角度）を実装する場合は、円環平均を考慮した関数を別途定義する
                sensor_data.heading_history.append(sensor_data.heading_fused)
                if len(sensor_data.heading_history) > HEADING_SMOOTHING_WINDOW:
                    sensor_data.heading_history.popleft()
                
                # dequeの単純平均を最終的な融合方位角とする（円環平均が必要な場合は修正）
                if sensor_data.heading_history:
                    sensor_data.heading_fused = sum(sensor_data.heading_history) / len(sensor_data.heading_history)
                    sensor_data.heading_fused = (sensor_data.heading_fused + 360) % 360 # 再び0-360度に正規化

                sensor_data.last_fused_heading = sensor_data.heading_fused

            else:
                # GPSデータが不完全な場合
                sensor_data.heading_gps = 0.0 # GPS方位は不定
                if imu_status and sensor_data.last_fused_heading != 0.0: # IMUのみで更新を継続
                    integrated_heading_change = imu_raw_gyro_z_avg * delta_time
                    sensor_data.heading_fused = (sensor_data.last_fused_heading + integrated_heading_change + 360) % 360
                    sensor_data.last_fused_heading = sensor_data.heading_fused
                else:
                    sensor_data.heading_fused = 0.0 # IMUもGPSも使えない場合は不定

            # --- 基線誤差の計算 ---
            # ロジックは現状維持。実際の精度に合わせて調整してください。
            if sensor_data.base_data['quality'] == 4 and sensor_data.rover_data['quality'] == 4: # RTK Fix
                sensor_data.error = max(0.02, 0.005 * (sensor_data.base_data['hdop'] + sensor_data.rover_data['hdop']))
            elif sensor_data.base_data['quality'] == 5 and sensor_data.rover_data['quality'] == 5: # RTK Float
                sensor_data.error = max(0.05, 0.02 * (sensor_data.base_data['hdop'] + sensor_data.rover_data['hdop']))
            else: # Single or unknown
                sensor_data.error = max(0.5, 0.2 * (sensor_data.base_data['hdop'] + sensor_data.rover_data['hdop']))
            
            # グラフデータの更新
            # 現在の方位角のインデックスを更新
            idx = int(sensor_data.heading_fused) % 360
            # Z軸角速度をグラフの表示範囲 (-99〜-20) に正規化
            # ここではダミーとして、Z軸ジャイロの絶対値が小さいほどグラフの値が-20に近づく（安定している）ようにマッピング
            # 実際の表示したい内容に合わせてこのロジックを調整してください
            normalized_gyro_z_for_graph = -20.0 - abs(imu_raw_gyro_z_avg) * 5
            normalized_gyro_z_for_graph = max(-99.0, min(-20.0, normalized_gyro_z_for_graph))
            sensor_data.graph_values[idx] = normalized_gyro_z_for_graph
            
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
            "heading_gps": round(sensor_data.heading_gps, 2), # 新しく追加: GPSのみからのヘッドデータ
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

@app.route("/api/nmea_data", methods=["GET"])
@require_api_key
def get_nmea_data():
    """
    保存されているNMEA生データをJSON形式で返すAPIエンドポイント。
    """
    with sensor_data.lock:
        # dequeをリストに変換して返す
        return jsonify({"nmea_lines": list(sensor_data.nmea_buffer)})

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
                # キャリブレーション完了後、gyro_z_bufferを新しいオフセットで再初期化
                sensor_data.gyro_z_buffer.clear()
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
    threading.Thread(target=read_imu_thread, args=(sensor_data,), daemon=True).start()
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
        # IMU初期化はスレッド開始前に一度試みる (ダミーモードでない場合のみ)
        if not DUMMY_MODE and GLOBAL_IMU_MODULE_AVAILABLE:
            initialize_imu_device(sensor_data)
        
        run_app()
    except KeyboardInterrupt:
        logger.info("アプリケーションをシャットダウンしています...")
        stop_event.set() # 全てのスレッドに停止を通知
        # デーモンスレッドは自動終了するため、明示的なjoinは不要だが、
        # クリーンアップやログの完了を待つために少し待機
        time.sleep(2) 
        logger.info("アプリケーションが終了しました。")
    except Exception as e:
        logger.critical(f"アプリケーションの予期せぬ終了: {e}")
        stop_event.set()
        time.sleep(2)
