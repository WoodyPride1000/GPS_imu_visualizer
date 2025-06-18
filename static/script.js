// static/script.js

// --- APIキーの設定 ---
// 注意: 実際のAPIキーを直接クライアントサイドに埋め込むのはセキュリティ上推奨されません。
// 本番環境では、バックエンドでAPIキーを管理し、プロキシ経由でアクセスするなどの対策が必要です。
const API_KEY = "your_api_key_here"; // config.ini の API_KEY と同じ値に設定してください！

// --- 地図の初期化 ---
// 'map' というIDを持つHTML要素に地図をレンダリング
const map = L.map('map', {
    center: [35.75, 139.75], // 初期表示の中心座標 (東京周辺の例)
    zoom: 15,                // 初期ズームレベル
    zoomControl: false       // ズームコントロール非表示
});

// OpenStreetMapのタイルレイヤーを追加
L.tileLayer('https://tile.openstreetmap.org/{z}/{x}/{y}.png', {
    maxZoom: 19,
    attribution: '&copy; <a href="http://www.openstreetmap.org/copyright">OpenStreetMap</a>'
}).addTo(map);

// GPS位置を表示するマーカーと円
const baseMarker = L.marker([0, 0]).addTo(map).bindPopup("基準局");
const roverMarker = L.marker([0, 0], { icon: L.icon({ iconUrl: 'data:image/svg+xml;base64,PHN2ZyB3aWR0aD0iMjQiIGhlaWdodD0iMjQiIHhtbG5zPSJodHRwOi8vd3d3LnczLm9yZy8yMDAwL3N2ZyI+PHBhdGggZD0iTTEyIDBMNGQ4IDEwQTMgMyAwIDEgMCAxMiAwem0wIDJhMSAxIDAgMSAwIDAgMnptMCA0YTMgMyAwIDEgMCAwIDZNMTAgMTZhMiAyIDAgMSAwIDAgNGwyIDJaIiBmaWxsPSJibHVlIi8+PC9zdmc+', iconSize: [24, 24], iconAnchor: [12, 24] }) }).addTo(map).bindPopup("移動局");
const errorCircle = L.circle([0, 0], { color: 'red', fillColor: '#f03', fillOpacity: 0.2, radius: 1 }).addTo(map);

// マーカーを繋ぐ線
const polyline = L.polyline([[0, 0], [0, 0]], { color: 'blue' }).addTo(map);

// データを表示する要素への参照
const latDisplay = document.getElementById('baseLat');
const lonDisplay = document.getElementById('baseLon');
const hdopBaseDisplay = document.getElementById('hdopBase');
const hdopRoverDisplay = document.getElementById('hdopRover');
const qualityBaseDisplay = document.getElementById('qualityBase');
const qualityRoverDisplay = document.getElementById('qualityRover');
const headingDisplay = document.getElementById('heading');
const errorDisplay = document.getElementById('error');
const distanceDisplay = document.getElementById('distance');
const imuStatusDisplay = document.getElementById('imuStatus');
const imuRawGyroZDisplay = document.getElementById('imuRawGyroZ');
const gyroZOffsetDisplay = document.getElementById('gyroZOffset');
const baseConnectedStatus = document.getElementById('baseConnected');
const roverConnectedStatus = document.getElementById('roverConnected');
const basePortErrorsDisplay = document.getElementById('basePortErrors');
const baseSerialErrorsDisplay = document.getElementById('baseSerialErrors');
const roverPortErrorsDisplay = document.getElementById('roverPortErrors');
const roverSerialErrorsDisplay = document.getElementById('roverSerialErrors');
const dummyModeDisplay = document.getElementById('dummyMode');
const logLevelDisplay = document.getElementById('logLevel');


// --- Chart.js グラフの初期化 ---
const ctx = document.getElementById('azimuthChart').getContext('2d');
let azimuthChart; // 後でChartオブジェクトを保持するための変数

// グラフデータを取得し、Chart.jsを初期化する関数
async function initAzimuthChart() {
    try {
        const response = await fetch('/api/graph_data', {
            headers: { 'X-API-KEY': API_KEY }
        });
        if (!response.ok) {
            throw new Error(`HTTP error! status: ${response.status}`);
        }
        const data = await response.json();

        azimuthChart = new Chart(ctx, {
            type: 'polarArea', // Polar Area chart
            data: {
                labels: data.azimuths.map(a => `${a}°`), // 0°から359°のラベル
                datasets: [{
                    label: '方位角別データ',
                    data: data.values,
                    backgroundColor: data.values.map(val => {
                        // 値に応じて色を変える例 (適宜調整)
                        if (val > -30) return 'rgba(75, 192, 192, 0.6)'; // 良い値
                        if (val > -60) return 'rgba(255, 206, 86, 0.6)'; // 普通
                        return 'rgba(255, 99, 132, 0.6)'; // 悪い値
                    }),
                    borderColor: data.values.map(val => {
                        if (val > -30) return 'rgba(75, 192, 192, 1)';
                        if (val > -60) return 'rgba(255, 206, 86, 1)';
                        return 'rgba(255, 99, 132, 1)';
                    }),
                    borderWidth: 1
                }]
            },
            options: {
                responsive: true,
                maintainAspectRatio: false,
                scale: {
                    r: {
                        angleLines: {
                            display: true
                        },
                        suggestedMin: -100, // グラフの最小値
                        suggestedMax: 0,  // グラフの最大値
                        pointLabels: {
                            display: true, // 方位角ラベルを表示
                            centerPointLabels: true // ラベルを中央に表示
                        },
                        ticks: {
                            stepSize: 20 // 目盛りのステップサイズ
                        }
                    }
                },
                plugins: {
                    legend: {
                        display: false // 凡例は表示しない
                    },
                    tooltip: {
                        callbacks: {
                            label: function(context) {
                                return `方位角 ${context.label}: ${context.raw.toFixed(2)}`;
                            }
                        }
                    }
                }
            }
        });
    } catch (error) {
        console.error("グラフの初期化エラー:", error);
    }
}


// --- 座標系変換のための Proj4js の設定 ---
// ここに日本の平面直角座標系などの定義を追加します
// 例: 平面直角座標系 第IX系 (JGD2011)
proj4.defs(
    "JGD2011_IX",
    "+proj=tmerc +lat_0=36 +lon_0=139.8333333333333 +k=0.9999 +x_0=0 +y_0=0 +ellps=GRS80 +towgs84=0,0,0,0,0,0,0 +units=m +no_defs"
);
// 必要に応じて他の系も追加してください

// 現在の座標系を保持する変数
let currentCoordSystem = 'wgs84'; // デフォルトは緯度経度

// 座標系セレクタの変更イベントリスナー
document.getElementById('coordSystemSelect').addEventListener('change', function(e) {
    currentCoordSystem = e.target.value;
    updateMapAndDisplay(
        currentCoordSystem,
        parseFloat(latDisplay.textContent), // 現在表示されている値を再利用
        parseFloat(lonDisplay.textContent)
    );
});

// 地図を更新し、表示を切り替える関数
function updateMapAndDisplay(coordSystem, lat, lon) {
    let displayLat = lat.toFixed(7);
    let displayLon = lon.toFixed(7);

    if (coordSystem === 'utm') {
        // UTM変換 (簡易的な例、ゾーンの決定は実際にはより複雑)
        const utmZone = Math.floor((lon + 180) / 6) + 1;
        const utmResult = proj4(`EPSG:4326`, `EPSG:326${utmZone}`, [lon, lat]); // EPSG:326XX は北半球UTM
        displayLat = utmResult[1].toFixed(3) + 'm';
        displayLon = utmResult[0].toFixed(3) + 'm';
    } else if (coordSystem === 'jgd2011') {
        // 平面直角座標系 第IX系への変換例
        // ここでは便宜上、JGD2011_IXを直接使用していますが、
        // 実際にはGPSの緯度経度 (WGS84) からJGD2011緯度経度に変換し、
        // それから平面直角座標系に変換するのが正しい手順です。
        // 簡単化のため、WGS84をJGD2011と同じとして変換します。
        try {
            const jgd2011Result = proj4(`EPSG:4326`, `JGD2011_IX`, [lon, lat]);
            displayLon = jgd2011Result[0].toFixed(3) + 'm'; // X
            displayLat = jgd2011Result[1].toFixed(3) + 'm'; // Y
        } catch (e) {
            console.error("平面直角座標系変換エラー:", e);
            displayLat = 'N/A';
            displayLon = 'N/A';
        }
    }

    latDisplay.textContent = `緯度: ${displayLat}`;
    lonDisplay.textContent = `経度: ${displayLon}`;
}


// --- APIからのデータ取得とUI更新 ---
async function fetchData() {
    try {
        const response = await fetch('/api/position', {
            headers: { 'X-API-KEY': API_KEY }
        });
        if (!response.ok) {
            throw new Error(`HTTP error! status: ${response.status}`);
        }
        const data = await response.json();

        // UIの更新
        latDisplay.textContent = `緯度: ${data.lat}`;
        lonDisplay.textContent = `経度: ${data.lon}`;
        hdopBaseDisplay.textContent = `HDOP (基準): ${data.hdop_base}`;
        hdopRoverDisplay.textContent = `HDOP (移動): ${data.hdop_rover}`;
        qualityBaseDisplay.textContent = `品質 (基準): ${data.base_quality}`;
        qualityRoverDisplay.textContent = `品質 (移動): ${data.rover_quality}`;
        headingDisplay.textContent = `方位角: ${data.heading.toFixed(2)}°`;
        errorDisplay.textContent = `誤差: ±${data.error.toFixed(3)}m`;
        distanceDisplay.textContent = `距離: ${data.distance.toFixed(3)}m`;
        imuStatusDisplay.textContent = `IMU: ${data.imu ? '動作中' : '停止'}`;
        imuRawGyroZDisplay.textContent = `ジャイロZ (平均): ${data.imu_raw_gyro_z.toFixed(5)}°/s`;
        gyroZOffsetDisplay.textContent = `ジャイロZオフセット: ${data.gyro_z_offset.toFixed(5)}°/s`;

        baseConnectedStatus.textContent = `基準局接続: ${data.base_connected ? 'はい' : 'いいえ'}`;
        baseConnectedStatus.className = data.base_connected ? 'status-ok' : 'status-ng';
        roverConnectedStatus.textContent = `移動局接続: ${data.rover_connected ? 'はい' : 'いいえ'}`;
        roverConnectedStatus.className = data.rover_connected ? 'status-ok' : 'status-ng';
        basePortErrorsDisplay.textContent = `基準局ポートエラー: ${data.base_port_errors}`;
        baseSerialErrorsDisplay.textContent = `基準局シリアルエラー: ${data.base_serial_errors}`;
        roverPortErrorsDisplay.textContent = `移動局ポートエラー: ${data.rover_port_errors}`;
        roverSerialErrorsDisplay.textContent = `移動局シリアルエラー: ${data.rover_serial_errors}`;
        dummyModeDisplay.textContent = `ダミーモード: ${data.dummy_mode ? 'はい' : 'いいえ'}`;
        logLevelDisplay.textContent = `ログレベル: ${data.log_level}`;

        // 地図とマーカーの更新
        if (data.lat !== 0.0 || data.lon !== 0.0) {
            baseMarker.setLatLng([data.lat, data.lon]);
            errorCircle.setLatLng([data.lat, data.lon]).setRadius(data.error);

            if (data.rover_lat !== 0.0 || data.rover_lon !== 0.0) {
                roverMarker.setLatLng([data.rover_lat, data.rover_lon]);
                polyline.setLatLngs([[data.lat, data.lon], [data.rover_lat, data.rover_lon]]);
                
                // 地図の中心を両方のマーカーの中間に設定し、ズームを調整
                const group = new L.featureGroup([baseMarker, roverMarker]);
                map.fitBounds(group.getBounds().pad(0.5)); // padding to show markers fully
            } else {
                roverMarker.setLatLng([data.lat, data.lon]); // 基準局と同じ位置に表示
                polyline.setLatLngs([[data.lat, data.lon], [data.lat, data.lon]]);
                map.setView([data.lat, data.lon], map.getZoom()); // 基準局を中心に
            }
            updateMapAndDisplay(currentCoordSystem, data.lat, data.lon); // 選択された座標系で表示
        } else {
            // データがまだ取得できていない場合、マーカーを非表示にするか、初期位置に戻す
            // baseMarker.setLatLng([0,0]);
            // roverMarker.setLatLng([0,0]);
            // errorCircle.setLatLng([0,0]).setRadius(1);
            // polyline.setLatLngs([[0,0],[0,0]]);
        }
        
        // グラフデータの更新
        // Chart.jsでデータセットを更新
        if (azimuthChart) {
            // 新しいグラフデータをAPIから取得し、チャートを更新
            const graphResponse = await fetch('/api/graph_data', {
                headers: { 'X-API-KEY': API_KEY }
            });
            if (graphResponse.ok) {
                const graphData = await graphResponse.json();
                azimuthChart.data.labels = graphData.azimuths.map(a => `${a}°`);
                azimuthChart.data.datasets[0].data = graphData.values;
                azimuthChart.data.datasets[0].backgroundColor = graphData.values.map(val => {
                    if (val > -30) return 'rgba(75, 192, 192, 0.6)';
                    if (val > -60) return 'rgba(255, 206, 86, 0.6)';
                    return 'rgba(255, 99, 132, 0.6)';
                });
                azimuthChart.data.datasets[0].borderColor = graphData.values.map(val => {
                    if (val > -30) return 'rgba(75, 192, 192, 1)';
                    if (val > -60) return 'rgba(255, 206, 86, 1)';
                    return 'rgba(255, 99, 132, 1)';
                });
                azimuthChart.update();
            }
        }

    } catch (error) {
        console.error("データの取得エラー:", error);
        // エラー発生時のUIフィードバック
        baseConnectedStatus.textContent = `基準局接続: エラー`;
        baseConnectedStatus.className = 'status-ng';
        roverConnectedStatus.textContent = `移動局接続: エラー`;
        roverConnectedStatus.className = 'status-ng';
        imuStatusDisplay.textContent = `IMU: エラー`;
    }
}

// --- イベントリスナーと初期化 ---

// IMUキャリブレーションボタン
document.getElementById('calibrateButton').addEventListener('click', async function() {
    const action = this.textContent === 'キャリブレーション開始' ? 'start' : 'stop';
    try {
        const response = await fetch('/api/calibrate_imu', {
            method: 'POST',
            headers: {
                'Content-Type': 'application/json',
                'X-API-KEY': API_KEY
            },
            body: JSON.stringify({ action: action })
        });
        const result = await response.json();
        alert(result.status);
        if (action === 'start') {
            this.textContent = 'キャリブレーション停止';
        } else {
            this.textContent = 'キャリブレーション開始';
            gyroZOffsetDisplay.textContent = `ジャイロZオフセット: ${result.offset.toFixed(5)}°/s`;
        }
    } catch (error) {
        console.error("IMUキャリブレーションエラー:", error);
        alert("IMUキャリブレーションに失敗しました。");
    }
});

// ログレベル設定ボタン
document.getElementById('setLogLevelButton').addEventListener('click', async function() {
    const logLevel = document.getElementById('logLevelSelect').value;
    try {
        const response = await fetch('/api/set_log_level', {
            method: 'POST',
            headers: {
                'Content-Type': 'application/json',
                'X-API-KEY': API_KEY
            },
            body: JSON.stringify({ level: logLevel })
        });
        const result = await response.json();
        alert(result.status);
        logLevelDisplay.textContent = `ログレベル: ${logLevel}`;
    } catch (error) {
        console.error("ログレベル設定エラー:", error);
        alert("ログレベル設定に失敗しました。");
    }
});


// ページ読み込み時にグラフを初期化
document.addEventListener('DOMContentLoaded', () => {
    initAzimuthChart();
    fetchData(); // 初回データ取得
    setInterval(fetchData, 500); // 0.5秒ごとにデータを更新
});
