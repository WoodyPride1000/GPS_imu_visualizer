import configparser
import logging
import logging.handlers
import os
import threading
import time
import random
import serial
import pynmea2
import math
import ssl # HTTPS化のために追加
from flask import Flask, jsonify, render_template, request
from pyproj import Geod
from typing import Dict, Optional, List
from collections import deque
from functools import wraps # 認証デコレータのために追加

# --- グローバルな終了イベント ---
stop_event = threading.Event()

# --- ログ設定 ---
log_file_handler = logging.handlers.RotatingFileHandler(
    'app.log', maxBytes=1048576, backupCount=5
)
log_file_handler.setFormatter(logging.Formatter('%(asctime)s - %(threadName)s - %(levelname)s - %(message)s'))

# ログレベル変更履歴用のハンドラーを追加
log_history_handler = logging.handlers.RotatingFileHandler(
    'log_history.log', maxBytes=1048576, backupCount=2
)
log_history_handler.setFormatter(logging.Formatter('%(asctime)s - %(levelname)s - %(message)s'))


logging.basicConfig(
    level=logging.INFO,
    format='%(asctime)s - %(threadName)s - %(levelname)s - %(message)s',
    handlers=[
        log_file_handler,
        log_history_handler, # ログ履歴ハンドラーを追加
        logging.StreamHandler()
    ]
)
logger = logging.getLogger(__name__)

# --- 設定ファイル読み込み ---
config = configparser.ConfigParser()
config_file_path = 'config.ini'

if not os.path.exists(config_file_path):
    logger.info(f"{config_file_path} が見つかりません。サンプルファイルを生成します。")
    config['GPS'] = {
        'BasePort': '/dev/ttyUSB0',
        'RoverPort': '/dev/ttyUSB1',
        'Baudrate': '4800',
        'BaselineLengthMeter': '0.7',
        'ReadInterval': '0.01',
        'SerialRetryInterval': '5',
        'MaxBaselineError': '0.1',
        'HdopThreshold': '2.0',
        'FixQualityThreshold': '2', # 1: GPS fix, 2: DGPS fix, 4: RTK fixed, 5: RTK float
        'MaxRetryCount': '10' # GPSシリアル接続の最大リトライ回数
    }
    config['IMU'] = {
        'ReadInterval': '0.05',
        'GyroZThreshold': '0.05',
        'MaxRetryCount': '5', # IMU再初期化の最大リトライ回数
        'GyroBufferSize': '10', # ジャイロZ軸の移動平均バッファサイズ
        'GyroOutlierThreshold': '3.0', # ジャイロZ軸の異常値除去閾値 (標準偏差の倍数)
        'MPU6050_I2C_BUS_NUM': '1', # MPU6050 I2Cバス番号
        'MPU6050_I2C_ADDR': '104' # MPU6050 I2Cアドレス (0x68)
    }
    config['FUSION'] = {
        'ImuGpsFusionAlpha': '0.98'
    }
    config['General'] = {
        'DummyMode': 'False',
        'DummyScenario': 'linear', # linear, circular, static
        'DummySpeedMps': '0.5', # ダミーモードの速度 (m/s)
        'DummyAngularSpeed': '0.1', # ダミーモードの角速度 (rad/s)
        'DummyRadiusM': '10.0', # ダミーモードの円形移動半径 (m)
        'LogLevel': 'INFO',
        'CalculationInterval': '0.1', # 計算スレッドの待機間隔
        'APIKey': 'YOUR_SECURE_API_KEY_HERE', # APIキーを追加
        'CertPath': 'cert.pem', # HTTPS証明書パスを追加
        'KeyPath': 'key.pem' # HTTPS秘密鍵パスを追加
    }
    with open(config_file_path, 'w') as f:
        config.write(f)
    logger.info(f"{config_file_path} が生成されました。必要に応じて編集してください。")
    config.read(config_file_path)
else:
    config.read(config_file_path)

# --- 定数設定と検証 ---
try:
    GPS_BASE_PORT = config.get('GPS', 'BasePort')
    GPS_ROVER_PORT = config.get('GPS', 'RoverPort')
    BAUDRATE = config.getint('GPS', 'Baudrate')
    BASELINE_LENGTH_METER = config.getfloat('GPS', 'BaselineLengthMeter')
    IMU_READ_INTERVAL = config.getfloat('IMU', 'ReadInterval')
    GPS_READ_INTERVAL = config.getfloat('GPS', 'ReadInterval')
    SERIAL_RETRY_INTERVAL = config.getfloat('GPS', 'SerialRetryInterval')
    MAX_BASELINE_ERROR = config.getfloat('GPS', 'MaxBaselineError')
    HDOP_THRESHOLD = config.getfloat('GPS', 'HdopThreshold')
    GPS_FIX_QUALITY_THRESHOLD = config.getint('GPS', 'FixQualityThreshold')
    GPS_MAX_RETRY_COUNT = config.getint('GPS', 'MaxRetryCount', fallback=10)
    IMU_MAX_RETRY_COUNT = config.getint('IMU', 'MaxRetryCount', fallback=5)
    IMU_GYRO_BUFFER_SIZE = config.getint('IMU', 'GyroBufferSize', fallback=10)
    IMU_GYRO_OUTLIER_THRESHOLD = config.getfloat('IMU', 'GyroOutlierThreshold', fallback=3.0)
    MPU6050_I2C_BUS_NUM = config.getint('IMU', 'MPU6050_I2C_BUS_NUM', fallback=1) # 新規追加
    MPU6050_I2C_ADDR = config.getint('IMU', 'MPU6050_I2C_ADDR', fallback=0x68) # 新規追加

    DUMMY_MODE = config.getboolean('General', 'DummyMode')
    DUMMY_SCENARIO = config.get('General', 'DummyScenario', fallback='linear')
    DUMMY_SPEED_MPS = config.getfloat('General', 'DummySpeedMps', fallback=0.5)
    DUMMY_ANGULAR_SPEED = config.getfloat('General', 'DummyAngularSpeed', fallback=0.1)
    DUMMY_RADIUS_M = config.getfloat('General', 'DummyRadiusM', fallback=10.0)

    IMU_GYRO_Z_THRESHOLD = config.getfloat('IMU', 'GyroZThreshold')
    IMU_GPS_FUSION_ALPHA = config.getfloat('FUSION', 'ImuGpsFusionAlpha')
    CALCULATION_INTERVAL = config.getfloat('General', 'CalculationInterval', fallback=0.1)
    
    API_KEY = config.get('General', 'APIKey', fallback='YOUR_SECURE_API_KEY_HERE') # APIキー
    CERT_PATH = config.get('General', 'CertPath', fallback='cert.pem') # 証明書パス
    KEY_PATH = config.get('General', 'keyPath', fallback='key.pem') # 秘密鍵パス

    log_level_str = config.get('General', 'LogLevel', fallback='INFO').upper()
    log_level = getattr(logging, log_level_str, logging.INFO)
    logger.setLevel(log_level)
    logger.info(f"ログレベルが {logging.getLevelName(log_level)} に設定されました。")

    if not (1 <= BAUDRATE <= 115200):
        raise ValueError(f"Baudrateが不正な値です: {BAUDRATE}")
    if not (1 <= GPS_MAX_RETRY_COUNT <= 30):
        raise ValueError(f"GPS_MAX_RETRY_COUNTが妥当な範囲ではありません: {GPS_MAX_RETRY_COUNT}")
    if not (1 <= IMU_GYRO_BUFFER_SIZE <= 100):
        raise ValueError(f"IMU_GYRO_BUFFER_SIZEが妥当な範囲ではありません: {IMU_GYRO_BUFFER_SIZE}")
    if DUMMY_SCENARIO not in ['linear', 'circular', 'static']:
        raise ValueError(f"DummyScenarioが不正な値です: {DUMMY_SCENARIO}. 'linear', 'circular', 'static' のいずれかを選択してください。")
    if not (0.01 <= CALCULATION_INTERVAL <= 1.0):
        raise ValueError(f"CalculationIntervalが妥当な範囲ではありません: {CALCULATION_INTERVAL}")
    if not (0.0 <= IMU_GYRO_OUTLIER_THRESHOLD <= 10.0):
        raise ValueError(f"IMU_GYRO_OUTLIER_THRESHOLDが妥当な範囲ではありません: {IMU_GYRO_OUTLIER_THRESHOLD}")
    
    if API_KEY == 'YOUR_SECURE_API_KEY_HERE':
        logger.warning("config.iniのAPIKeyがデフォルト値のままです。本番環境では必ず変更してください。")

    if not DUMMY_MODE: # HTTPS証明書の存在チェック (ダミーモードでは不要)
        if not os.path.exists(CERT_PATH):
            logger.critical(f"HTTPS証明書ファイルが見つかりません: {CERT_PATH}")
            raise FileNotFoundError(f"HTTPS証明書ファイルが見つかりません: {CERT_PATH}")
        if not os.path.exists(KEY_PATH):
            logger.critical(f"HTTPS秘密鍵ファイルが見つかりません: {KEY_PATH}")
            raise FileNotFoundError(f"HTTPS秘密鍵ファイルが見つかりません: {KEY_PATH}")


except (configparser.Error, ValueError, FileNotFoundError) as e:
    if isinstance(e, ValueError) or isinstance(e, FileNotFoundError):
        error_msg = str(e)
        problem_key = "Unknown"
        # 発生したエラーメッセージに基づいて、どの設定項目が問題か特定
        for section in config.sections():
            for option in config.options(section):
                if option in error_msg: # エラーメッセージにオプション名が含まれるか
                    problem_key = f"[{section}].{option}"
                    break
                # 特定のメッセージパターンでさらに絞り込み
                if "HTTPS証明書" in error_msg and (option == 'CertPath' or option == 'KeyPath'):
                    problem_key = f"[General].{option}"
                    break
            if problem_key != "Unknown":
                break
        logger.critical(f"設定ファイルの型変換、値の範囲、またはファイルパスエラー: {problem_key} - {error_msg}. "
                        f"config.iniの該当項目を確認してください。")
    else:
        logger.critical(f"設定ファイルの読み込みエラー: {e}")
    logger.critical("アプリケーションを終了します。")
    exit(1)

EARTH_RADIUS_M = 6371000

app = Flask(__name__)

# --- IMUライブラリのインポート ---
GLOBAL_IMU_MODULE_AVAILABLE = False
try:
    import smbus # MPU6050ライブラリでsmbusが必要なため
    from mpu6050 import MPU6050 # MPU6050クラスをインポート
    GLOBAL_IMU_MODULE_AVAILABLE = True
except ImportError as e:
    logger.warning(f"mpu6050ライブラリが見つからないか、依存関係エラー: {e}。IMUは無効になります。")

# --- データを保持するクラス ---
class SensorData:
    """GPSとIMUデータを管理し、スレッドセーフなデータアクセスを提供するクラス。"""
    def __init__(self):
        self.base_data: Dict[str, float or int] = {'lat': 0.0, 'lon': 0.0, 'hdop': 99.9, 'quality': 0, 'timestamp': 0.0}
        self.rover_data: Dict[str, float or int] = {'lat': 0.0, 'lon': 0.0, 'hdop': 99.9, 'quality': 0, 'timestamp': 0.0}
        self.heading_gps: float = 0.0
        self.heading_fused: float = 0.0
        self.error: float = 0.0
        self.distance: float = 0.0
        self.imu_status: bool = False
        self.imu_accel_x: float = 0.0 # 新規追加
        self.imu_accel_y: float = 0.0 # 新規追加
        self.imu_accel_z: float = 0.0 # 新規追加
        self.imu_gyro_x: float = 0.0 # 新規追加
        self.imu_gyro_y: float = 0.0 # 新規追加
        self.imu_raw_gyro_z: float = 0.0 # Z軸のみ移動平均とオフセット適用
        self.lock = threading.Lock()
        self.last_fused_heading: float = 0.0
        self.last_imu_read_time: float = time.monotonic()

        self.calibration_mode: bool = False
        self.calibration_samples: List[float] = []
        self.gyro_z_offset: float = 0.0

        self.imu_device: Optional[MPU6050] = None # 型ヒントをMPU6050に変更
        self.gyro_z_buffer: deque = deque(maxlen=IMU_GYRO_BUFFER_SIZE)

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

    def update_gps_data(self, target_key: str, lat: float, lon: float, hdop: float, quality: int) -> None:
        """GPSデータをスレッドセーフに更新し、接続状態をセットする。"""
        with self.lock:
            if target_key == 'base':
                self.base_data = {'lat': lat, 'lon': lon, 'hdop': hdop, 'quality': quality, 'timestamp': time.monotonic()}
                if not self.base_connected:
                    self.base_connected = True
                    self.base_connection_timestamp = time.monotonic()
                    logger.info(f"GPS Base ({GPS_BASE_PORT}) が接続されました。")
            else:
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
            # ジャイロZ軸はキャリブレーションとバッファリングを適用
            if self.calibration_mode:
                self.calibration_samples.append(gyro_z)
                return

            corrected_z = gyro_z - self.gyro_z_offset
            
            # 異常値除去 (IMU_GYRO_OUTLIER_THRESHOLD を使用)
            if len(self.gyro_z_buffer) > 1:
                mean = sum(self.gyro_z_buffer) / len(self.gyro_z_buffer)
                std = (sum((x - mean) ** 2 for x in self.gyro_z_buffer) / len(self.gyro_z_buffer)) ** 0.5
                if std > 0.001 and abs(corrected_z - mean) > IMU_GYRO_OUTLIER_THRESHOLD * std:
                    logger.debug(f"IMU異常値検出 (raw: {gyro_z:.5f}, corrected: {corrected_z:.5f}, mean: {mean:.5f}, std: {std:.5f}, threshold: ±{IMU_GYRO_OUTLIER_THRESHOLD*std:.5f})。スキップします。")
                    return
            
            self.gyro_z_buffer.append(corrected_z)
            self.imu_raw_gyro_z = sum(self.gyro_z_buffer) / len(self.gyro_z_buffer) if self.gyro_z_buffer else 0.0
            self.imu_status = True
            self.last_imu_read_time = time.monotonic()
            self.data_updated_event.set()

sensor_data = SensorData()

geod = Geod(ellps='WGS84')

# --- IMU初期化 ---
def initialize_imu_device(sensor_data: SensorData) -> bool:
    """
    IMUデバイスの初期化を試みる。成功すればTrue、失敗すればFalseを返す。
    """
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
        sensor_data.imu_device = MPU6050(MPU6050_I2C_BUS_NUM, MPU6050_I2C_ADDR)
        sensor_data.imu_device.dmp_initialize() # DMP初期化を追加
        with sensor_data.lock:
            sensor_data.imu_status = True
        logger.info("IMUが正常に初期化されました。")
        return True
    except Exception as e:
        logger.error(f"IMU初期化エラー: {e}。IMUは無効になります。")
        with sensor_data.lock:
            sensor_data.imu_status = False
        return False

# --- IMU読み取りスレッド ---
def read_imu_thread(sensor_data: SensorData):
    """
    IMUセンサーからデータを読み取り、SensorDataオブジェクトを更新するスレッド。
    IMUの再初期化ロジックと連続エラーログ抑制、リトライ上限を含む。
    """
    last_imu_reinit_error_log_time = 0
    retry_count = 0

    while not stop_event.is_set():
        current_time = time.monotonic()

        if DUMMY_MODE:
            # ダミーの加速度とジャイロデータを生成
            dummy_accel_x = random.uniform(-1.0, 1.0)
            dummy_accel_y = random.uniform(-1.0, 1.0)
            dummy_accel_z = random.uniform(9.0, 10.0) # Z軸は重力方向を模倣
            dummy_gyro_x = random.uniform(-5.0, 5.0)
            dummy_gyro_y = random.uniform(-5.0, 5.0)
            dummy_gyro_z = random.uniform(-2.0, 2.0)
            sensor_data.update_imu_data(dummy_accel_x, dummy_accel_y, dummy_accel_z,
                                        dummy_gyro_x, dummy_gyro_y, dummy_gyro_z)
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
                break
            continue

        try:
            # 加速度データの取得
            accel_data = sensor_data.imu_device.get_acceleration()
            # 角速度データの取得
            gyro_data = sensor_data.imu_device.get_rotation()
            
            # 取得したデータをSensorDataオブジェクトに更新
            sensor_data.update_imu_data(accel_data.x, accel_data.y, accel_data.z,
                                        gyro_data.x, gyro_data.y, gyro_data.z)
            retry_count = 0
        except Exception as e:
            with sensor_data.lock:
                sensor_data.imu_status = False
            logger.error(f"IMU読み取りエラー: {e}。IMUステータスをFalseに設定しました。再接続を試みます。")
            retry_count += 1
            time.sleep(SERIAL_RETRY_INTERVAL)
        time.sleep(IMU_READ_INTERVAL)
    logger.info("IMU読み取りスレッドを停止します。")

# --- GPS読み取りスレッド ---
def read_gps_thread(port: str, target_key: str, sensor_data: SensorData):
    """
    指定されたGPSポートからNMEAデータを読み取り、SensorDataオブジェクトを更新するスレッド。
    シリアルポートの接続/再接続ロジックと連続エラーログ抑制、リトライ上限を含む。
    """
    last_port_error_log_time = 0
    last_serial_error_log_time = 0
    
    # ダミーモード用の初期位置 (埼玉県の東松山周辺)
    base_lat_dummy = 36.035
    base_lon_dummy = 139.407
    dummy_heading_degrees = 90.0 # 初期ヘディング（東）

    if DUMMY_MODE:
        while not stop_event.is_set():
            delta_t_dummy = random.uniform(0.5, 1.5)

            if DUMMY_SCENARIO == 'linear':
                dist_moved_m = DUMMY_SPEED_MPS * delta_t_dummy
                lat_change = dist_moved_m * math.cos(math.radians(dummy_heading_degrees)) / EARTH_RADIUS_M * (180 / math.pi)
                lon_change = dist_moved_m * math.sin(math.radians(dummy_heading_degrees)) / (EARTH_RADIUS_M * math.cos(math.radians(base_lat_dummy))) * (180 / math.pi)
                base_lat_dummy += lat_change
                base_lon_dummy += lon_change
            elif DUMMY_SCENARIO == 'circular':
                dummy_heading_degrees += DUMMY_ANGULAR_SPEED * delta_t_dummy * 180 / math.pi
                dummy_heading_degrees %= 360
                
                # 中心点からの相対位置を計算してから更新
                # この部分は少し複雑なので、単純に円形軌道を生成するロジックに置き換えることも検討
                # 例: 極座標から直交座標へ変換し、その後緯度経度に再変換
                # 簡単のため、ここではダミーヘディングを更新し、直線移動のダミーと同じように使う
                # より厳密な円形移動が必要な場合は、ダミーの中心点を設定し、その周りを回るようにする
                pass # 現状のロジックでは円形移動を正確にシミュレートできていない
            elif DUMMY_SCENARIO == 'static':
                pass

            if target_key == 'base':
                sensor_data.update_gps_data('base', base_lat_dummy, base_lon_dummy, random.uniform(0.8, 1.5), random.choice([1, 2]))
            else:
                rover_relative_heading_deg = 0.0
                # 基線長が短い場合、誤差が大きくなる可能性があるため、適切なスケールでダミーデータを生成
                rover_lon, rover_lat, _ = geod.fwd(base_lon_dummy, base_lat_dummy, rover_relative_heading_deg, BASELINE_LENGTH_METER)
                sensor_data.update_gps_data('rover', rover_lat, rover_lon, random.uniform(0.8, 1.5), random.choice([1, 2]))
            time.sleep(delta_t_dummy)
    else:
        ser: Optional[serial.Serial] = None
        # 各スレッドでローカルにエラーカウントを保持し、SensorDataに更新
        port_error_count_local = 0
        serial_error_count_local = 0

        while not stop_event.is_set():
            if not os.path.exists(port):
                current_time = time.monotonic()
                if current_time - last_port_error_log_time > SERIAL_RETRY_INTERVAL:
                    if port_error_count_local == 0:
                        logger.error(f"GPSポート {port} が見つかりません。{SERIAL_RETRY_INTERVAL}秒後に再試行します。")
                    else:
                        logger.warning(f"GPSポート {port} のエラーが継続中 ({port_error_count_local}回目)。")
                    last_port_error_log_time = current_time
                    
                    with sensor_data.lock:
                        if target_key == 'base':
                            sensor_data.base_port_errors += 1
                        else:
                            sensor_data.rover_port_errors += 1
                    port_error_count_local += 1 # ローカルカウンタもインクリメント
                time.sleep(SERIAL_RETRY_INTERVAL)
                continue

            if serial_error_count_local >= GPS_MAX_RETRY_COUNT:
                logger.error(f"GPSポート {port} の再接続上限回数 ({GPS_MAX_RETRY_COUNT}) に達しました。スレッドを終了します。")
                with sensor_data.lock:
                    if target_key == 'base':
                        if sensor_data.base_connected: # 接続が切断された瞬間にログ
                            sensor_data.base_connected = False
                            sensor_data.base_connection_timestamp = time.monotonic()
                            logger.error(f"GPS Base ({port}) の接続が切断されました。")
                    else:
                        if sensor_data.rover_connected: # 接続が切断された瞬間にログ
                            sensor_data.rover_connected = False
                            sensor_data.rover_connection_timestamp = time.monotonic()
                            logger.error(f"GPS Rover ({port}) の接続が切断されました。")
                break

            try:
                if ser is None or not ser.is_open:
                    ser = serial.Serial(port, BAUDRATE, timeout=0.5)
                    logger.info(f"GPSポート {port} が正常に開かれました。")
                    port_error_count_local = 0
                    serial_error_count_local = 0
                    # 接続が正常になった場合、SensorDataの接続フラグもupdate_gps_data内で更新される

                while not stop_event.is_set():
                    try:
                        line = ser.readline().decode('ascii', errors='ignore').strip()
                        if line.startswith("$GPGGA"):
                            try:
                                msg = pynmea2.parse(line)
                                sensor_data.update_gps_data(target_key, msg.latitude, msg.longitude, float(msg.horizontal_dil), int(msg.gps_qual))
                                serial_error_count_local = 0 # 正常受信でローカルカウンタをリセット
                            except pynmea2.ParseError as e:
                                logger.debug(f"NMEAパースエラー ({port}): {e} - Line: {line}")
                                continue
                        # 必要に応じて他のNMEAセンテンスも処理を追加
                    except serial.SerialTimeoutException:
                        logger.debug(f"GPSポート {port} 読み取りタイムアウト。")
                        continue
                    except Exception as e: # シリアル通信中の一般的なエラー
                        current_time = time.monotonic()
                        if current_time - last_serial_error_log_time > SERIAL_RETRY_INTERVAL:
                            if serial_error_count_local == 0:
                                logger.error(f"GPSポート {port} 読み取り中の予期せぬエラー: {e}。再接続を試みます。")
                            else:
                                logger.warning(f"GPSポート {port} の読み取りエラーが継続中 ({serial_error_count_local}回目)。再接続を試みます。")
                            last_serial_error_log_time = current_time
                            
                            with sensor_data.lock:
                                if target_key == 'base':
                                    sensor_data.base_serial_errors += 1
                                else:
                                    sensor_data.rover_serial_errors += 1
                            serial_error_count_local += 1 # ローカルカウンタもインクリメント
                        break # エラー発生で内側のループを抜け、外側で再接続を試みる
                    time.sleep(GPS_READ_INTERVAL)
            except serial.SerialException as e: # シリアルポートを開く際のエラー
                current_time = time.monotonic()
                if current_time - last_serial_error_log_time > SERIAL_RETRY_INTERVAL:
                    if serial_error_count_local == 0:
                        logger.error(f"GPSポート {port} を開けません: {e}。{SERIAL_RETRY_INTERVAL}秒後に再試行します。")
                    else:
                        logger.warning(f"GPSポート {port} の接続エラーが継続中 ({serial_error_count_local}回目)。{SERIAL_RETRY_INTERVAL}秒後に再試行します。")
                    last_serial_error_log_time = current_time
                    
                    with sensor_data.lock:
                        if target_key == 'base':
                            sensor_data.base_serial_errors += 1
                        else:
                            sensor_data.rover_serial_errors += 1
                    serial_error_count_local += 1 # ローカルカウンタもインクリメント
                time.sleep(SERIAL_RETRY_INTERVAL)
            except Exception as e: # その他の予期せぬエラー
                current_time = time.monotonic()
                if current_time - last_serial_error_log_time > SERIAL_RETRY_INTERVAL:
                    if serial_error_count_local == 0:
                        logger.error(f"GPSポート {port} で予期せぬエラーが発生しました: {e}。{SERIAL_RETRY_INTERVAL}秒後に再試行します。")
                    else:
                        logger.warning(f"GPSポート {port} の予期せぬエラーが継続中 ({serial_error_count_local}回目)。{SERIAL_RETRY_INTERVAL}秒後に再試行します。")
                    last_serial_error_log_time = current_time
                    
                    with sensor_data.lock:
                        if target_key == 'base':
                            sensor_data.base_serial_errors += 1
                        else:
                            sensor_data.rover_serial_errors += 1
                    serial_error_count_local += 1 # ローカルカウンタもインクリメント
                time.sleep(SERIAL_RETRY_INTERVAL)
            finally:
                if ser and ser.is_open:
                    ser.close()
                    logger.info(f"GPSポート {port} を閉じました。")
                ser = None
    logger.info(f"GPS読み取りスレッド ({port}) を停止します。")

# --- ヘディングと誤差の計算スレッド ---
def calculate_heading_and_error_thread(sensor_data: SensorData):
    """
    GPSデータとIMUデータを基に、ヘディングと基線長誤差を計算し、SensorDataオブジェクトを更新するスレッド。
    データ更新をトリガーとするイベント駆動型。
    """
    last_calc_time = 0.0

    while not stop_event.is_set():
        if not sensor_data.data_updated_event.wait(timeout=CALCULATION_INTERVAL):
            if stop_event.is_set():
                break
            continue

        sensor_data.data_updated_event.clear()

        with sensor_data.lock:
            base_connected = sensor_data.base_connected
            rover_connected = sensor_data.rover_connected
            lat1, lon1 = sensor_data.base_data['lat'], sensor_data.base_data['lon']
            lat2, lon2 = sensor_data.rover_data['lat'], sensor_data.rover_data['lon']
            hdop_base, hdop_rover = sensor_data.base_data['hdop'], sensor_data.rover_data['hdop']
            quality_base, quality_rover = sensor_data.base_data['quality'], sensor_data.rover_data['quality']
            imu_gyro_z = sensor_data.imu_raw_gyro_z
            imu_status = sensor_data.imu_status
            last_fused_heading = sensor_data.last_fused_heading
            last_imu_read_time = sensor_data.last_imu_read_time
            current_base_timestamp = sensor_data.base_data['timestamp']
            current_rover_timestamp = sensor_data.rover_data['timestamp']

        # 最新データが前回の計算時刻より新しいかチェック
        if max(current_base_timestamp, current_rover_timestamp) <= last_calc_time:
            # logger.debug(f"データが更新されていないため計算をスキップ。GPS Base timestamp: {current_base_timestamp}, GPS Rover timestamp: {current_rover_timestamp}, Last calc time: {last_calc_time}")
            continue
        last_calc_time = time.monotonic() # 計算時刻を更新

        # GPS接続状態のチェック
        if not (base_connected and rover_connected):
            # logger.info("GPS接続が切断されているため、ヘディング計算をスキップします。") # ログが多すぎる場合があるのでコメントアウト
            continue

        # GPSデータの有効性チェックを強化
        if (lat1 == 0.0 and lon1 == 0.0) or (lat2 == 0.0 and lon2 == 0.0) or \
           hdop_base > HDOP_THRESHOLD or hdop_rover > HDOP_THRESHOLD or \
           quality_base < GPS_FIX_QUALITY_THRESHOLD or quality_rover < GPS_FIX_QUALITY_THRESHOLD:
            # logger.info( # ログが多すぎる場合があるのでコメントアウト
            #     f"無効なGPSデータのため計算をスキップ: "
            #     f"Base(lat={lat1:.5f}, lon={lon1:.5f}, hdop={hdop_base:.2f}, qual={quality_base}), "
            #     f"Rover(lat={lat2:.5f}, lon={lon2:.5f}, hdop={hdop_rover:.2f}, qual={quality_rover}). "
            #     f"HDOP閾値={HDOP_THRESHOLD}, 品質閾値={GPS_FIX_QUALITY_THRESHOLD}"
            # )
            continue

        az12, _, calculated_distance = geod.inv(lon1, lat1, lon2, lat2)
        calculated_heading = (az12 + 360) % 360

        calculated_error = abs(calculated_distance - BASELINE_LENGTH_METER)

        fused_heading = calculated_heading

        if imu_status and abs(imu_gyro_z) > IMU_GYRO_Z_THRESHOLD:
            delta_t_imu_since_last_read = time.monotonic() - last_imu_read_time
            heading_change = imu_gyro_z * delta_t_imu_since_last_read

            imu_predicted_heading = (last_fused_heading + heading_change) % 360
            
            angle_diff = (calculated_heading - imu_predicted_heading + 540) % 360 - 180

            fused_heading = (imu_predicted_heading + (1 - IMU_GPS_FUSION_ALPHA) * angle_diff + 360) % 360
        else:
            fused_heading = calculated_heading

        with sensor_data.lock:
            sensor_data.heading_gps = calculated_heading
            sensor_data.heading_fused = fused_heading
            sensor_data.last_fused_heading = fused_heading
            sensor_data.error = calculated_error
            sensor_data.distance = calculated_distance
            if calculated_error > MAX_BASELINE_ERROR:
                logger.warning(f"基線長誤差が大きすぎます: {calculated_error:.3f}m (閾値: {MAX_BASELINE_ERROR}m)")

    logger.info("ヘディング計算スレッドを停止します。")

# --- 認証デコレータ ---
def require_api_key(f):
    """APIキー認証を要求するデコレータ。"""
    @wraps(f)
    def decorated_function(*args, **kwargs):
        if DUMMY_MODE: # ダミーモードでは認証をスキップ
            return f(*args, **kwargs)

        request_api_key = request.headers.get('X-API-Key')
        if request_api_key and request_api_key == API_KEY:
            return f(*args, **kwargs)
        else:
            logger.warning(f"不正なAPIキーアクセス試行: {request.path} from {request.remote_addr}")
            return jsonify({"status": "error", "message": "Unauthorized: Invalid or missing API Key"}), 401
    return decorated_function

# --- キャリブレーションAPI ---
@app.route("/api/calibration/start", methods=["POST"])
@require_api_key # 認証を要求
def calibration_start():
    if DUMMY_MODE:
        return jsonify({"status": "error", "message": "ダミーモードではキャリブレーションできません。"}), 400

    with sensor_data.lock:
        if sensor_data.calibration_mode:
            logger.warning(f"IMUキャリブレーションが既に進行中。要求元: {request.remote_addr}")
            return jsonify({"status": "error", "message": "既にキャリブレーション中です。"}), 400
        if not sensor_data.imu_status:
            logger.error(f"IMUが利用できないためキャリブレーション開始を拒否。要求元: {request.remote_addr}")
            return jsonify({"status": "error", "message": "IMUが接続されていないか、利用できません。"}), 503
        
        sensor_data.calibration_mode = True
        sensor_data.calibration_samples.clear()
    logger.info(f"IMUキャリブレーションを開始しました。デバイスを静止させてください。要求元: {request.remote_addr}")
    return jsonify({"status": "ok", "message": "キャリブレーション開始。デバイスを静止させてください。"})

@app.route("/api/calibration/stop", methods=["POST"])
@require_api_key # 認証を要求
def calibration_stop():
    if DUMMY_MODE:
        return jsonify({"status": "error", "message": "ダミーモードではキャリブレーションできません。"}), 400

    with sensor_data.lock:
        if not sensor_data.calibration_mode:
            logger.warning(f"IMUキャリブレーションが開始されていないため停止を拒否。要求元: {request.remote_addr}")
            return jsonify({"status": "error", "message": "キャリブレーションは開始されていません。"}), 400
        sensor_data.calibration_mode = False
        samples = sensor_data.calibration_samples.copy()
        sensor_data.calibration_samples.clear()

    if len(samples) < 100:
        logger.warning(f"キャリブレーション用のサンプル数が不足しています ({len(samples)}/100)。オフセットは適用されません。要求元: {request.remote_addr}")
        return jsonify({"status": "warning", "message": f"キャリブレーション用のサンプルが不足しています ({len(samples)}/100)。オフセットは適用されません。", "gyro_z_offset": sensor_data.gyro_z_offset}), 200

    offset = sum(samples) / len(samples)
    with sensor_data.lock:
        sensor_data.gyro_z_offset = offset
        # バッファ内の既存データにもオフセットを適用
        sensor_data.gyro_z_buffer = deque(
            [x - offset for x in sensor_data.gyro_z_buffer], maxlen=IMU_GYRO_BUFFER_SIZE)
        sensor_data.imu_raw_gyro_z = sum(sensor_data.gyro_z_buffer) / len(sensor_data.gyro_z_buffer) if sensor_data.gyro_z_buffer else 0.0

    logger.info(f"IMUキャリブレーション完了。ジャイロZオフセット = {offset:.5f}。要求元: {request.remote_addr}")
    return jsonify({"status": "ok", "message": "キャリブレーション完了。", "gyro_z_offset": round(offset, 5)})

# --- ログレベル変更API ---
@app.route("/api/log_level", methods=["POST"])
@require_api_key # 認証を要求
def set_log_level():
    level_str = request.json.get('level', 'INFO').upper()
    try:
        level = getattr(logging, level_str)
        # ログレベル変更をセキュリティログに記録
        logger.info(f"API経由でログレベル変更を要求: {level_str} (要求元: {request.remote_addr})")
        logger.setLevel(level)
        config['General']['LogLevel'] = level_str
        with open(config_file_path, 'w') as f:
            config.write(f)
        logger.info(f"ログレベルを {logging.getLevelName(level)} に変更し、config.iniに保存しました。", extra={'log_history': True}) # log_history_handlerに記録
        return jsonify({"status": "ok", "message": f"ログレベルを {level_str} に設定しました。"})
    except AttributeError:
        logger.warning(f"無効なログレベル変更試行: {level_str} (要求元: {request.remote_addr})")
        return jsonify({"status": "error", "message": f"無効なログレベル: {level_str}。有効なレベルは DEBUG, INFO, WARNING, ERROR, CRITICAL です。"}), 400

# --- データ取得API ---
@app.route("/api/data")
@require_api_key # 認証を要求
def api_data():
    with sensor_data.lock:
        response = {
            "base": sensor_data.base_data,
            "rover": sensor_data.rover_data,
            "heading_gps": round(sensor_data.heading_gps, 2),
            "heading_fused": round(sensor_data.heading_fused, 2),
            "distance": round(sensor_data.distance, 3),
            "error": round(sensor_data.error, 3),
            "imu_status": sensor_data.imu_status,
            "imu_accel_x": round(sensor_data.imu_accel_x, 5), # 追加
            "imu_accel_y": round(sensor_data.imu_accel_y, 5), # 追加
            "imu_accel_z": round(sensor_data.imu_accel_z, 5), # 追加
            "imu_gyro_x": round(sensor_data.imu_gyro_x, 5), # 追加
            "imu_gyro_y": round(sensor_data.imu_gyro_y, 5), # 追加
            "gyro_z_corrected": round(sensor_data.imu_raw_gyro_z, 5),
            "gyro_z_offset": round(sensor_data.gyro_z_offset, 5),
            "dummy_mode": DUMMY_MODE,
            "base_connected": sensor_data.base_connected,
            "rover_connected": sensor_data.rover_connected,
            "base_connection_timestamp": round(sensor_data.base_connection_timestamp, 2),
            "rover_connection_timestamp": round(sensor_data.rover_connection_timestamp, 2),
        }
    return jsonify(response)

# --- APIステータスエンドポイント ---
@app.route("/api/status")
@require_api_key # 認証を要求
def api_status():
    with sensor_data.lock:
        status_info = {
            "application_status": "running" if not stop_event.is_set() else "stopping",
            "threads_running": [t.name for t in threading.enumerate() if t.is_alive()],
            "base_port_errors": sensor_data.base_port_errors,
            "base_serial_errors": sensor_data.base_serial_errors,
            "rover_port_errors": sensor_data.rover_port_errors,
            "rover_serial_errors": sensor_data.rover_serial_errors,
            "imu_status": sensor_data.imu_status,
            "imu_max_retry_count": IMU_MAX_RETRY_COUNT,
            "gps_max_retry_count": GPS_MAX_RETRY_COUNT,
            "dummy_mode": DUMMY_MODE,
            "log_level": logging.getLevelName(logger.level)
        }
    return jsonify(status_info)


# --- サーバー起動とスレッド管理 ---
def run_app():
    if not DUMMY_MODE:
        try:
            # SSLコンテキストの作成
            context = ssl.SSLContext(ssl.PROTOCOL_TLS_SERVER)
            context.load_cert_chain(CERT_PATH, KEY_PATH)
            logger.info(f"HTTPSサーバーを {CERT_PATH} と {KEY_PATH} を使用して起動します。")
            app.run(host='0.0.0.0', port=5000, ssl_context=context, debug=False)
        except FileNotFoundError as e:
            logger.critical(f"HTTPS証明書または秘密鍵ファイルが見つかりません: {e}。HTTPSサーバーは起動できません。")
            stop_event.set() # アプリケーションを停止
        except ssl.SSLError as e:
            logger.critical(f"HTTPS SSLエラー: {e}。証明書ファイルを確認してください。HTTPSサーバーは起動できません。")
            stop_event.set() # アプリケーションを停止
        except Exception as e:
            logger.critical(f"HTTPSサーバー起動中に予期せぬエラーが発生しました: {e}。")
            stop_event.set() # アプリケーションを停止
    else:
        logger.info("ダミーモードでHTTPサーバーを起動します。")
        app.run(host='0.0.0.0', port=5000, debug=False)

if __name__ == "__main__":
    logger.info("アプリケーションを開始します。")

    threads = []

    # IMU読み取りスレッドの開始 (IMUが利用可能であれば)
    imu_thread = threading.Thread(target=read_imu_thread, args=(sensor_data,), name="IMUReadThread")
    threads.append(imu_thread)
    imu_thread.start()

    # GPS読み取りスレッドの開始
    base_gps_thread = threading.Thread(target=read_gps_thread, args=(GPS_BASE_PORT, 'base', sensor_data), name="BaseGPSReadThread")
    rover_gps_thread = threading.Thread(target=read_gps_thread, args=(GPS_ROVER_PORT, 'rover', sensor_data), name="RoverGPSReadThread")
    threads.append(base_gps_thread)
    threads.append(rover_gps_thread)
    base_gps_thread.start()
    rover_gps_thread.start()

    # ヘディングと誤差計算スレッドの開始
    calc_thread = threading.Thread(target=calculate_heading_and_error_thread, args=(sensor_data,), name="CalculationThread")
    threads.append(calc_thread)
    calc_thread.start()

    # Flaskアプリを別スレッドで実行
    flask_thread = threading.Thread(target=run_app, name="FlaskAppThread")
    threads.append(flask_thread)
    flask_thread.start()

    try:
        # メインスレッドは終了イベントを待機し、子スレッドの終了を監視
        while not stop_event.is_set():
            alive_threads = [t for t in threads if t.is_alive()]
            if len(alive_threads) < len(threads):
                logger.warning("一部のスレッドが予期せず終了しました。")
                stop_event.set() # 全スレッド停止のシグナル
                break
            time.sleep(1) # 1秒ごとにスレッドの状態をチェック
    except KeyboardInterrupt:
        logger.info("Ctrl+Cが検出されました。アプリケーションを終了します。")
    finally:
        stop_event.set() # 全スレッドに停止を指示
        for t in threads:
            if t.is_alive():
                t.join(timeout=5) # 各スレッドの終了を最大5秒待機
                if t.is_alive():
                    logger.warning(f"スレッド {t.name} が時間内に終了しませんでした。")
        logger.info("すべてのスレッドが停止しました。アプリケーションを終了します。")
