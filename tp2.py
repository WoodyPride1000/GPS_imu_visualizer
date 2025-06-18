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
            self.imu_gyro_z = gyro_z # gyro_zを直接保存
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
