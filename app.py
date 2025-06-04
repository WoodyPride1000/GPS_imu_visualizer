import configparser
import logging
import os
import threading
import time
import random
import serial
import pynmea2
import math
from flask import Flask, jsonify, render_template

# --- ログ設定 ---
logging.basicConfig(
    level=logging.INFO,
    format='%(asctime)s - %(levelname)s - %(message)s',
    handlers=[
        logging.FileHandler('app.log'),
        logging.StreamHandler()
    ]
)
logger = logging.getLogger(__name__)

# --- 設定ファイル読み込み ---
config = configparser.ConfigParser()
config.read('config.ini')

# --- 定数設定 ---
GPS_BASE_PORT = config.get('GPS', 'BasePort', fallback='/dev/ttyUSB0')
GPS_ROVER_PORT = config.get('GPS', 'RoverPort', fallback='/dev/ttyUSB1')
BAUDRATE = config.getint('GPS', 'Baudrate', fallback=4800)
BASELINE_LENGTH_METER = config.getfloat('GPS', 'BaselineLengthMeter', fallback=0.7)
IMU_READ_INTERVAL = config.getfloat('IMU', 'ReadInterval', fallback=0.05)
GPS_READ_INTERVAL = config.getfloat('GPS', 'ReadInterval', fallback=0.01)
SERIAL_RETRY_INTERVAL = config.getfloat('GPS', 'SerialRetryInterval', fallback=5)
EARTH_RADIUS_M = 6371000  # 地球の平均半径（メートル）
MAX_BASELINE_ERROR = config.getfloat('GPS', 'MaxBaselineError', fallback=0.1)
HDOP_THRESHOLD = config.getfloat('GPS', 'HdopThreshold', fallback=2.0)
DUMMY_MODE = config.getboolean('General', 'DummyMode', fallback=False)

# IMUライブラリのインポート
IMU_AVAILABLE = False
try:
    from mpu6050 import mpu6050
    IMU_AVAILABLE = True
except ImportError:
    logger.warning("mpu6050ライブラリが見つかりませんでした。IMUは無効になります。")

app = Flask(__name__)

# --- データを保持するクラス ---
class SensorData:
    def __init__(self):
        self.base_data = {'lat': 0.0, 'lon': 0.0, 'hdop': 99.9}
        self.rover_data = {'lat': 0.0, 'lon': 0.0, 'hdop': 99.9}
        self.heading_gps = 0.0
        self.heading_fused = 0.0  # 融合ヘディング
        self.error = 0.0  # 基線誤差（メートル）
        self.distance = 0.0  # ベースとローバー間の距離（メートル）
        self.imu_status = False
        self.imu_raw_gyro_z = 0.0
        self.lock = threading.Lock()
        self.last_fused_heading = 0.0  # 融合ヘディングの前回値

sensor_data = SensorData()

# --- IMU初期化 ---
imu_device = None
if IMU_AVAILABLE and not DUMMY_MODE:
    def initialize_imu():
        global imu_device, IMU_AVAILABLE
        while True:
            try:
                imu_device = mpu6050(0x68)
                with sensor_data.lock:
                    sensor_data.imu_status = True
                logger.info("IMUが正常に初期化されました。")
                IMU_AVAILABLE = True
                break
            except Exception as e:
                logger.error(f"IMU初期化エラー: {e}。{SERIAL_RETRY_INTERVAL}秒後に再試行します。")
                IMU_AVAILABLE = False
                time.sleep(SERIAL_RETRY_INTERVAL)

    initialize_imu()

# --- IMU読み取りスレッド ---
def read_imu_thread():
    while True:
        if DUMMY_MODE:
            with sensor_data.lock:
                sensor_data.imu_raw_gyro_z = random.uniform(-10.0, 10.0)
                sensor_data.imu_status = True
            time.sleep(IMU_READ_INTERVAL)
        else:
            if not IMU_AVAILABLE:
                time.sleep(SERIAL_RETRY_INTERVAL)
                continue
            try:
                gyro = imu_device.get_gyro_data()
                with sensor_data.lock:
                    sensor_data.imu_raw_gyro_z = gyro['z']
                    sensor_data.imu_status = True
            except Exception as e:
                with sensor_data.lock:
                    sensor_data.imu_status = False
                logger.error(f"IMU読み取りエラー: {e}")
                time.sleep(SERIAL_RETRY_INTERVAL)
            time.sleep(IMU_READ_INTERVAL)

# --- GPS読み取りスレッド ---
def read_gps_thread(port: str, target_key: str):
    if DUMMY_MODE:
        while True:
            with sensor_data.lock:
                if target_key == 'base':
                    sensor_data.base_data['lat'] = random.uniform(35.680, 35.682)
                    sensor_data.base_data['lon'] = random.uniform(139.765, 139.768)
                    sensor_data.base_data['hdop'] = random.uniform(0.8, 1.5)
                else:
                    sensor_data.rover_data['lat'] = sensor_data.base_data['lat'] + random.uniform(-0.0001, 0.0001)
                    sensor_data.rover_data['lon'] = sensor_data.base_data['lon'] + random.uniform(-0.0001, 0.0001)
                    sensor_data.rover_data['hdop'] = random.uniform(0.8, 1.5)
            time.sleep(random.uniform(0.5, 1.5))
    else:
        while True:
            if not os.path.exists(port):
                logger.error(f"GPSポート {port} が見つかりません。{SERIAL_RETRY_INTERVAL}秒後に再試行します。")
                time.sleep(SERIAL_RETRY_INTERVAL)
                continue
            try:
                ser = serial.Serial(port, BAUDRATE, timeout=0.5)
                logger.info(f"GPSポート {port} が正常に開かれました。")
            except serial.SerialException as e:
                logger.error(f"GPSポート {port} を開けません: {e}。{SERIAL_RETRY_INTERVAL}秒後に再試行します。")
                time.sleep(SERIAL_RETRY_INTERVAL)
                continue
            try:
                while True:
                    line = ser.readline().decode('ascii', errors='ignore').strip()
                    if line.startswith("$GPGGA"):
                        try:
                            msg = pynmea2.parse(line)
                            with sensor_data.lock:
                                if target_key == 'base':
                                    sensor_data.base_data['lat'] = msg.latitude
                                    sensor_data.base_data['lon'] = msg.longitude
                                    sensor_data.base_data['hdop'] = float(msg.horizontal_dil)
                                else:
                                    sensor_data.rover_data['lat'] = msg.latitude
                                    sensor_data.rover_data['lon'] = msg.longitude
                                    sensor_data.rover_data['hdop'] = float(msg.horizontal_dil)
                        except pynmea2.ParseError:
                            continue
                    time.sleep(GPS_READ_INTERVAL)
            except serial.SerialException as e:
                logger.error(f"GPSポート {port} でシリアル通信エラー: {e}。再接続を試みます。")
                ser.close()
                time.sleep(SERIAL_RETRY_INTERVAL)
            except Exception as e:
                logger.error(f"GPSポート {port} で予期せぬエラー: {e}。再接続を試みます。")
                if 'ser' in locals() and ser.is_open:
                    ser.close()
                time.sleep(SERIAL_RETRY_INTERVAL)

# --- ヘディングと誤差の計算スレッド ---
def calculate_heading_and_error_thread():
    while True:
        with sensor_data.lock:
            lat1, lon1 = sensor_data.base_data['lat'], sensor_data.base_data['lon']
            lat2, lon2 = sensor_data.rover_data['lat'], sensor_data.rover_data['lon']
            hdop_base, hdop_rover = sensor_data.base_data['hdop'], sensor_data.rover_data['hdop']
            imu_gyro_z = sensor_data.imu_raw_gyro_z
            imu_status = sensor_data.imu_status

        if lat1 == 0.0 or lon1 == 0.0 or lat2 == 0.0 or lon2 == 0.0 or hdop_base > HDOP_THRESHOLD or hdop_rover > HDOP_THRESHOLD:
            logger.warning(f"無効なGPSデータ: lat1={lat1}, lon1={lon1}, lat2={lat2}, lon2={lon2}, hdop_base={hdop_base}, hdop_rover={hdop_rover}")
            time.sleep(1)
            continue

        # Haversine公式で距離計算
        phi1, lambda1 = math.radians(lat1), math.radians(lon1)
        phi2, lambda2 = math.radians(lat2), math.radians(lon2)
        d_phi, d_lambda = phi2 - phi1, lambda2 - lambda1
        a = math.sin(d_phi / 2)**2 + math.cos(phi1) * math.cos(phi2) * math.sin(d_lambda / 2)**2
        c = 2 * math.atan2(math.sqrt(a), math.sqrt(1 - a))
        calculated_distance = EARTH_RADIUS_M * c

        # ヘディング計算
        y = math.sin(d_lambda) * math.cos(phi2)
        x = math.cos(phi1) * math.sin(phi2) - math.sin(phi1) * math.cos(phi2) * math.cos(d_lambda)
        calculated_heading = (math.degrees(math.atan2(y, x)) + 360) % 360

        # 基線長誤差
        calculated_error = abs(calculated_distance - BASELINE_LENGTH_METER)

        # IMU-GPS融合（簡易補完フィルター）
        ALPHA = 0.98  # IMUの信頼度
        fused_heading = calculated_heading
        if imu_status and abs(imu_gyro_z) > 0.05:
            heading_change = imu_gyro_z * 0.5  # 0.5秒間隔を仮定
            fused_heading = (ALPHA * (sensor_data.last_fused_heading + heading_change) + (1 - ALPHA) * calculated_heading + 360) % 360

        with sensor_data.lock:
            sensor_data.heading_gps = calculated_heading
            sensor_data.heading_fused = fused_heading
            sensor_data.last_fused_heading = fused_heading
            sensor_data.error = calculated_error
            sensor_data.distance = calculated_distance
            if calculated_error > MAX_BASELINE_ERROR:
                logger.warning(f"基線長誤差が大きすぎます: {calculated_error}m")

        time.sleep(0.5)

# --- スレッド起動 ---
threading.Thread(target=read_gps_thread, args=(GPS_BASE_PORT, 'base'), daemon=True).start()
threading.Thread(target=read_gps_thread, args=(GPS_ROVER_PORT, 'rover'), daemon=True).start()
threading.Thread(target=calculate_heading_and_error_thread, daemon=True).start()
if IMU_AVAILABLE:
    threading.Thread(target=read_imu_thread, daemon=True).start()

# --- Flask Webアプリケーション ---
@app.route("/")
def index():
    return render_template("index.html")

@app.route("/api/position")
def api_position():
    with sensor_data.lock:
        data = {
            "lat": sensor_data.base_data['lat'],
            "lon": sensor_data.base_data['lon'],
            "heading": sensor_data.heading_fused,  # 融合ヘディングを返す
            "distance": sensor_data.distance,
            "error": sensor_data.error,
            "imu": sensor_data.imu_status,
            "imu_raw_gyro_z": sensor_data.imu_raw_gyro_z,
            "hdop_base": sensor_data.base_data['hdop'],
            "hdop_rover": sensor_data.rover_data['hdop']
        }
    return jsonify(data)

if __name__ == "__main__":
    app.run(debug=True, use_reloader=False, host="0.0.0.0")
