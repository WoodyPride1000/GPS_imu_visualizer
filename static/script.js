// static/script.js

// --- APIキーの設定 ---
// 注意: 実際のAPIキーを直接クライアントサイドに埋め込むのはセキュリティ上推奨されません。
// 本番環境では、バックエンドでAPIキーを管理し、プロキシ経由でアクセスするなどの対策が必要です。
// config.ini の API_KEY と同じ値に設定してください！
const API_KEY = "your_api_key_here"; 

// --- グローバル変数と初期化 ---
let map;
let baseMarker, roverMarker, headingLine, fanLayer, calculatedHeadingLine;
let followMap = true;
let gridLayer;
let currentCoordSystem = 'wgs84'; // 初期座標系
let azimuthChart; // Chart.jsインスタンス
let showNMEA = false; // NMEA表示の状態
let isPlacingSymbol = false; // シンボル配置モードの状態
let symbolCounter = 0; // シンボルID用のカウンター
// customMarkersの要素は { id: string, name: string, lat: number, lon: number, marker: L.Marker, element: HTMLElement } となる
let customMarkers = []; // 配置されたカスタムマーカーを保存する配列

let chartData = {
    labels: Array.from({length: 360}, (_, i) => i.toString()), // 0-359
    datasets: [{
        label: 'IMU Z軸安定度', // グラフのラベル
        backgroundColor: 'rgba(75, 192, 192, 0.6)', // 初期色（PolarArea用）
        borderColor: 'rgba(75, 192, 192, 1)',
        borderWidth: 1,
        data: Array(360).fill(-50), // app.pyの初期値に合わせて-50
    }]
};

// --- ヘルパー関数 ---

// 角度の差を-180〜180度の範囲に正規化するヘルパー関数
function normalizeAngleDifference(angle1, angle2) {
    let diff = angle1 - angle2;
    if (diff > 180) diff -= 360;
    if (diff < -180) diff += 360;
    return diff;
}

// --- 地図の初期化 ---
function initMap() {
    console.log("initMap: Starting map initialization."); // 診断用ログ
    // 東京駅周辺を初期位置として設定
    const tokyoStationLat = 35.681236;
    const tokyoStationLon = 139.767125;

    console.log("initMap: Attempting to create L.map instance."); // 診断用ログ
    // `#map` 要素が存在するか確認
    const mapElement = document.getElementById('map');
    if (!mapElement) {
        console.error("initMap Error: Element with ID 'map' not found in the DOM.");
        return; // 地図の初期化を中止
    }

    map = L.map('map').setView([tokyoStationLat, tokyoStationLon], 18); // 初期位置とズームレベル
    console.log("initMap: L.map instance created."); // 診断用ログ

    L.tileLayer('https://{s}.tile.openstreetmap.org/{z}/{x}/{y}.png', {
        attribution: '&copy; <a href="https://www.openstreetmap.org/copyright">OpenStreetMap</a> contributors',
        maxZoom: 19
    }).addTo(map);
    console.log("initMap: Tile layer added."); // 診断用ログ

    // マップのサイズを再計算してタイルを適切に表示させる
    window.requestAnimationFrame(() => {
        setTimeout(() => {
            console.log("initMap: Invalidating map size."); // 診断用ログ
            map.invalidateSize();
            console.log("initMap: Map size invalidated."); // 診断用ログ
        }, 0); // 最小限の遅延 (DOMが完全に構築されてから次のマイクロタスクで実行)
    });

    // ウィンドウのリサイズイベントにリスナーを追加
    // これにより、ブラウザウィンドウのサイズが変更されたときに地図が自動的に調整されます。
    window.addEventListener('resize', () => {
        console.log("Resize event: Invalidating map size."); // 診断用ログ
        map.invalidateSize();
    });
    console.log("initMap: Event listeners added and initial setup complete."); // 診断用ログ

    // 基準局と移動局のマーカーを東京駅の初期位置に設定
    baseMarker = L.marker([tokyoStationLat, tokyoStationLon], {title: "基準局"}).addTo(map).bindPopup("基準局").openPopup();
    roverMarker = L.marker([tokyoStationLat, tokyoStationLon], {title: "移動局"}).addTo(map).bindPopup("移動局").openPopup();

    // 方位線と扇形のためのレイヤーグループ
    headingLine = L.polyline([], { color: 'blue', weight: 3, opacity: 0.7 }).addTo(map);
    fanLayer = L.polygon([], { color: 'orange', fillOpacity: 0.3, stroke: false }).addTo(map);

    // 算出した方位を示す線 (破線の緑色)
    calculatedHeadingLine = L.polyline([], { color: 'green', weight: 3, opacity: 0.7, dashArray: '5, 5' }).addTo(map);

    // グリッド線レイヤー
    gridLayer = L.layerGroup().addTo(map);
    updateGrid(map.getBounds()); // 初期グリッド表示
    map.on('moveend', function() {
        if (document.getElementById('gridCheckbox').checked) {
            updateGrid(map.getBounds());
        }
    });

    // シンボル配置モード時のマップクリックイベント
    map.on('click', function(e) {
        if (isPlacingSymbol) {
            const lat = e.latlng.lat;
            const lon = e.latlng.lng;
            symbolCounter++;
            const symbolId = `symbol-${symbolCounter}`;
            const symbolName = `地点 ${symbolCounter}`;

            // カスタムアイコンを作成
            const customIcon = L.divIcon({
                className: 'custom-symbol-icon',
                iconSize: [16, 16], // アイコンのサイズ
                iconAnchor: [8, 8], // アイコンの中心 (アイコンサイズの半分)
                popupAnchor: [0, -8] // ポップアップのアンカー位置
            });

            const marker = L.marker([lat, lon], {icon: customIcon}).addTo(map);
            marker.bindPopup(`<strong>${symbolName}</strong><br>緯度: ${lat.toFixed(7)}<br>経度: ${lon.toFixed(7)}`).openPopup();
            
            // シンボル情報表示リストに要素を追加
            const symbolList = document.getElementById('symbol-list');
            const listItem = document.createElement('li');
            listItem.id = `symbol-item-${symbolId}`;
            listItem.className = 'symbol-item';
            listItem.innerHTML = `
                <strong>${symbolName}</strong> 
                <button class="delete-symbol-button" data-symbol-id="${symbolId}">削除</button><br>
                緯度: ${lat.toFixed(7)}<br>
                経度: ${lon.toFixed(7)}<br>
                基準局からの距離: <span id="dist-${symbolId}">--</span> m<br>
                基準局からの方位角: <span id="bearing-${symbolId}">--</span> °
            `;
            symbolList.appendChild(listItem);

            // 削除ボタンにイベントリスナーを追加
            listItem.querySelector('.delete-symbol-button').addEventListener('click', (event) => {
                const idToDelete = event.target.dataset.symbolId;
                deleteSymbol(idToDelete);
            });

            customMarkers.push({
                id: symbolId,
                name: symbolName,
                lat: lat,
                lon: lon,
                marker: marker,
                element: listItem // HTML要素への参照を保存
            });
            console.log(`シンボルを配置しました: ${symbolName} (緯度 ${lat.toFixed(7)}, 経度 ${lon.toFixed(7)})`);
        }
    });
}

// --- シンボル削除機能 ---
function deleteSymbol(symbolId) {
    // customMarkers配列から該当するシンボルを探す
    const index = customMarkers.findIndex(s => s.id === symbolId);
    if (index > -1) {
        const symbolToDelete = customMarkers[index];

        // マップからマーカーを削除
        map.removeLayer(symbolToDelete.marker);

        // DOMからリストアイテムを削除
        symbolToDelete.element.remove();

        // customMarkers配列からシンボルを削除
        customMarkers.splice(index, 1);
        console.log(`シンボル ${symbolId} を削除しました。`);
    }
}

// --- グリッド線機能 ---
function updateGrid(bounds) {
    console.log("updateGrid called with bounds:", bounds.toBBoxString());
    gridLayer.clearLayers();
    if (!document.getElementById('gridCheckbox').checked) return;

    const zoom = map.getZoom();
    let intervalLat = 0.01;
    let intervalLon = 0.01;

    if (zoom >= 17) {
        intervalLat = 0.001;
        intervalLon = 0.001;
    } else if (zoom >= 15) {
        intervalLat = 0.005;
        intervalLon = 0.005;
    } else if (zoom >= 13) {
        intervalLat = 0.01;
        intervalLon = 0.01;
    } else {
        intervalLat = 0.05;
        intervalLon = 0.05;
    }

    const latMin = Math.floor(bounds.getSouthWest().lat / intervalLat) * intervalLat;
    const latMax = Math.ceil(bounds.getNorthEast().lat / intervalLat) * intervalLat;
    const lonMin = Math.floor(bounds.getSouthWest().lng / intervalLon) * intervalLon;
    const lonMax = Math.ceil(bounds.getNorthEast().lng / intervalLon) * intervalLon;

    for (let lat = latMin; lat <= latMax; lat += intervalLat) {
        L.polyline([[lat, lonMin], [lat, lonMax]], { color: '#ccc', weight: 0.5, opacity: 0.6, dashArray: '2, 4' }).addTo(gridLayer);
    }
    for (let lon = lonMin; lon <= lonMax; lon += intervalLon) {
        L.polyline([[latMin, lon], [latMax, lon]], { color: '#ccc', weight: 0.5, opacity: 0.6, dashArray: '2, 4' }).addTo(gridLayer);
    }
}

// --- 座標変換機能 ---
// UTM (Zone 54N for Tokyo)
proj4.defs("EPSG:32654", "+proj=utm +zone=54 +datum=WGS84 +units=m +no_defs");
// 平面直角座標系 (JGD2011, 系9 for Kanto)
proj4.defs("JGD2011_PLANE_9", "+proj=tmerc +lat_0=36 +lon_0=139.8333333333333 +k=0.9999 +x_0=0 +y_0=0 +ellps=GRS80 +towgs84=0,0,0,0,0,0,0 +units=m +no_defs");

function convertCoordinates(lat, lon, targetSystem) {
    if (targetSystem === 'wgs84') {
        return { lat, lon, text: `緯度: ${lat.toFixed(7)}, 経度: ${lon.toFixed(7)}` };
    } else if (targetSystem === 'utm') {
        if (typeof proj4 === 'undefined') {
            console.error("proj4.jsがロードされていません。UTM変換は利用できません。");
            return { lat, lon, text: `緯度: ${lat.toFixed(7)}, 経度: ${lon.toFixed(7)} (UTM変換エラー)` };
        }
        const utm = proj4("EPSG:4326", "EPSG:32654", [lon, lat]);
        return { x: utm[0], y: utm[1], text: `UTM-X: ${utm[0].toFixed(2)}, UTM-Y: ${utm[1].toFixed(2)}` };
    } else if (targetSystem === 'jgd2011') {
            if (typeof proj4 === 'undefined') {
            console.error("proj4.jsがロードされていません。JGD2011変換は利用できません。");
            return { lat, lon, text: `緯度: ${lat.toFixed(7)}, 経度: ${lon.toFixed(7)} (JGD2011変換エラー)` };
        }
        const jgd = proj4("EPSG:4326", "JGD2011_PLANE_9", [lon, lat]);
        return { x: jgd[0], y: jgd[1], text: `X: ${jgd[0].toFixed(2)}, Y: ${jgd[1].toFixed(2)} (系9)` };
    }
    return { lat, lon, text: `緯度: ${lat.toFixed(7)}, 経度: ${lon.toFixed(7)}` }; // Fallback
}

// --- Chart.js グラフの初期化 ---
function initChart() {
    const ctx = document.getElementById('azimuthChart').getContext('2d');
    azimuthChart = new Chart(ctx, {
        type: 'polarArea', // ご提示の script.js に合わせて Polar Area Chart
        data: chartData,
        options: {
            responsive: true,
            maintainAspectRatio: false,
            scale: {
                r: {
                    angleLines: {
                        display: true
                    },
                    suggestedMin: -100, // グラフの最小値
                    suggestedMax: 0,  // グラフの最大値
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
                title: {
                    display: true,
                    text: 'IMU Z軸安定度 (方位角別)'
                },
                tooltip: {
                    callbacks: {
                        label: function(context) {
                            return `方位角 ${context.label}: ${context.raw.toFixed(2)}`;
                        }
                    }
                }
            },
            animation: {
                duration: 0 // アニメーションなしで即時更新
            }
        }
    });
}

// --- ファン形状 (扇形) の更新 ---
function updateFan(lat, lon, heading, angleWidth) {
    // 方位角 (heading) は北を0°として時計回り (0-360)
    // angleWidth は扇形の中心から左右への角度 (例えば45°なら全体で90°)
    // Leaflet.GeometryUtil.destination を使用

    // 変更: roverMarkerから直接LatLngを取得し、地図コンテキストを確保
    const center = roverMarker.getLatLng(); // 修正点
    // 扇形の半径をメートル単位で指定 (例: 50メートル)
    const radiusMeters = 50; 

    const points = [center];
    const startAngle = (heading - angleWidth / 2 + 360) % 360;
    const endAngle = (heading + angleWidth / 2 + 360) % 360;

    const numSegments = 30; // 扇形を構成するセグメント数
    for (let i = 0; i <= numSegments; i++) {
        const angle = startAngle + (endAngle - startAngle) * i / numSegments;
        // L.GeometryUtil.destination(origin, bearing, distance)
        const point = L.GeometryUtil.destination(center, angle, radiusMeters);
        points.push(point);
    }
    points.push(center); // 閉じたポリゴンにするために中心に戻る

    fanLayer.setLatLngs(points);
}

// --- APIからのデータ取得とUI更新 (メインループ) ---
async function fetchSensorData() {
    try {
        const response = await fetch(window.location.origin + '/api/position', {
            headers: { 'X-API-KEY': API_KEY }
        });
        if (!response.ok) {
            const errorStatus = response.status;
            const errorText = await response.text();
            let errorMessage = `HTTPエラー: ${errorStatus} - ${errorText}`;
            if (errorStatus === 401) {
                errorMessage = "認証エラー: APIキーが不正です。";
            } else if (errorStatus === 404) {
                errorMessage = "APIエンドポイントが見つかりません (HTTP 404)。";
            }
            throw new Error(errorMessage);
        }
        const data = await response.json();

        // GPSデータの更新
        const baseLat = data.lat;
        const baseLon = data.lon;
        const roverLat = data.rover_lat;
        const roverLon = data.rover_lon; 

        baseMarker.setLatLng([baseLat, baseLon]);
        roverMarker.setLatLng([roverLat, roverLon]);

        // 方位線の更新 (基準局と移動局を結ぶ線 - 青色)
        const linePoints = [[baseLat, baseLon], [roverLat, roverLon]];
        headingLine.setLatLngs(linePoints);

        // 扇形の更新
        const fanValue = parseFloat(document.getElementById('fanSlider').value);
        const headingFused = data.heading; // 融合方位角
        if (typeof L.GeometryUtil !== 'undefined' && L.GeometryUtil.destination) {
            // updateFan内でroverMarker.getLatLng()を使用するよう変更したため、引数は不要に
            updateFan(roverLat, roverLon, headingFused, fanValue);
        } else {
            console.warn("L.GeometryUtil.destination が利用できないため、扇形は描画されません。");
        }

        // 算出した方位を示す線の更新 (画面サイズの30%の長さで表示 - 緑色破線)
        if (typeof L.GeometryUtil !== 'undefined' && L.GeometryUtil.destination) {
            // 変更: baseMarkerから直接LatLngを取得し、地図コンテキストを確保
            const baseLatLngForCalculatedLine = baseMarker.getLatLng(); // 修正点
            const bounds = map.getBounds();
            const visibleHeightMeters = map.distance(bounds.getNorthWest(), L.latLng(bounds.getSouthWest().lat, bounds.getNorthWest().lng));
            const visibleWidthMeters = map.distance(bounds.getNorthWest(), bounds.getNorthEast());
            const dynamicLineLengthMeters = Math.min(visibleHeightMeters, visibleWidthMeters) * 0.30;
            
            const calculatedLineDestination = L.GeometryUtil.destination(baseLatLngForCalculatedLine, headingFused, dynamicLineLengthMeters);
            calculatedHeadingLine.setLatLngs([baseLatLngForCalculatedLine, calculatedLineDestination]);
        }

        // 地図の追従
        if (document.getElementById('followMapCheckbox').checked) {
            map.setView([baseLat, baseLon], map.getZoom());
        }

        // Info Boxの更新
        const convertedBase = convertCoordinates(baseLat, baseLon, currentCoordSystem);
        document.getElementById('baseLat').textContent = convertedBase.text.split(',')[0].trim();
        document.getElementById('baseLon').textContent = convertedBase.text.split(',')[1] ? convertedBase.text.split(',')[1].trim() : '';

        document.getElementById('hdopBase').textContent = data.hdop_base;
        document.getElementById('hdopRover').textContent = data.hdop_rover;
        document.getElementById('qualityBase').textContent = data.base_quality;
        document.getElementById('qualityRover').textContent = data.rover_quality;
        document.getElementById('headingFused').textContent = data.heading.toFixed(2); // 融合方位角
        document.getElementById('headingGPS').textContent = data.heading_gps.toFixed(2); // GPS方位角
        
        // 方位角のずれ量の計算と表示
        const headingDiff = normalizeAngleDifference(data.heading, data.heading_gps);
        document.getElementById('headingDiff').textContent = headingDiff.toFixed(2);

        document.getElementById('distance').textContent = data.distance.toFixed(3);
        document.getElementById('error').textContent = data.error.toFixed(3);
        
        document.getElementById('imuStatus').textContent = data.imu ? 'ON' : 'OFF';
        document.getElementById('imuStatus').className = data.imu ? 'status-ok' : 'status-ng';

        document.getElementById('imuRawGyroZ').textContent = data.imu_raw_gyro_z.toFixed(5);
        document.getElementById('gyroZOffset').textContent = data.gyro_z_offset.toFixed(5);
        
        // baseConnectedとroverConnectedはdivにIDがあり、spanはその子要素
        document.getElementById('baseConnected').querySelector('span').className = data.base_connected ? 'status-ok' : 'status-ng';
        document.getElementById('baseConnected').querySelector('span').textContent = data.base_connected ? '接続中' : '切断';
        
        document.getElementById('roverConnected').querySelector('span').className = data.rover_connected ? 'status-ok' : 'status-ng';
        document.getElementById('roverConnected').querySelector('span').textContent = data.rover_connected ? '接続中' : '切断';
        
        document.getElementById('basePortErrors').textContent = data.base_port_errors;
        document.getElementById('baseSerialErrors').textContent = data.base_serial_errors;
        document.getElementById('roverPortErrors').textContent = data.rover_port_errors;
        document.getElementById('roverSerialErrors').textContent = data.rover_serial_errors;

        // IDがspan要素に直接付与されているため、querySelector('span')は不要
        document.getElementById('dummyMode').textContent = data.dummy_mode ? 'ON' : 'OFF';
        document.getElementById('logLevel').textContent = data.log_level;

        // ステータスメッセージをクリア
        const statusMessageElement = document.getElementById('status-message');
        if (statusMessageElement) {
             statusMessageElement.textContent = ""; 
        } else {
             console.error("fetchSensorData: Element with ID 'status-message' not found.");
        }


        // シンボルリストの情報を更新
        if (typeof L.GeometryUtil !== 'undefined' && L.GeometryUtil.distance && L.GeometryUtil.bearing) {
            // 変更: baseMarkerから直接LatLngを取得し、地図コンテキストを確保
            const baseLatLng = baseMarker.getLatLng(); // 修正点
            customMarkers.forEach(symbol => {
                const symbolLatLng = L.latLng(symbol.lat, symbol.lon);
                // 修正: L.GeometryUtil.distance と L.GeometryUtil.bearing の最初の引数に 'map' オブジェクトを追加
                const distanceToBase = L.GeometryUtil.distance(map, baseLatLng, symbolLatLng); // 修正点
                const bearingToBase = L.GeometryUtil.bearing(map, baseLatLng, symbolLatLng); // 修正点

                document.getElementById(`dist-${symbol.id}`).textContent = distanceToBase.toFixed(3);
                document.getElementById(`bearing-${symbol.id}`).textContent = bearingToBase.toFixed(2);
            });
        } else {
            console.warn("L.GeometryUtilが利用できないため、シンボル情報の距離・方位角は更新されません。");
        }

    } catch (error) {
        console.error("センサーデータの取得中にエラー:", error);
        const statusMessageElement = document.getElementById('status-message');
        let message = `エラー: ${error.message}`;
        let color = 'orange';

        if (error instanceof TypeError && error.message.includes("Failed to fetch")) {
            message = `サーバーに接続できません。バックエンドが実行中か、ネットワーク接続を確認してください。`;
            color = 'red';
        } else if (error.message.includes("HTTPエラー: 404")) {
            message = "APIエンドポイントが見つかりません (HTTP 404)。";
            color = 'red';
        } else if (error.message.includes("認証エラー") || error.message.includes("HTTPエラー: 401")) {
            message = "認証エラー (HTTP 401): APIキーが不正です。index.htmlとconfig.iniのAPI_KEYが一致しているか確認してください。";
            color = 'red';
        }
        
        if (statusMessageElement) {
            statusMessageElement.textContent = message;
            statusMessageElement.style.color = color;
        } else {
            console.error("Error handler: Element with ID 'status-message' not found.");
        }

        // エラー時は接続ステータスをNGにする
        // ここでも要素の存在チェックを追加
        const baseConnectedSpan = document.getElementById('baseConnected')?.querySelector('span');
        const roverConnectedSpan = document.getElementById('roverConnected')?.querySelector('span');
        const imuStatusElement = document.getElementById('imuStatus');

        if (baseConnectedSpan) {
            baseConnectedSpan.className = 'status-ng';
            baseConnectedSpan.textContent = '切断';
        }
        if (roverConnectedSpan) {
            roverConnectedSpan.className = 'status-ng';
            roverConnectedSpan.textContent = '切断';
        }
        if (imuStatusElement) {
            imuStatusElement.className = 'status-ng';
            imuStatusElement.textContent = 'OFF';
        }
    }
}

// --- グラフデータの取得と更新 ---
async function fetchGraphData() {
    try {
        const response = await fetch(window.location.origin + '/api/graph_data', {
            headers: { 'X-API-KEY': API_KEY }
        });
        if (!response.ok) {
            const errorStatus = response.status;
            const errorText = await response.text();
            let errorMessage = `HTTPエラー: ${errorStatus} - ${errorText}`;
            if (errorStatus === 401) {
                errorMessage = "認証エラー: APIキーが不正です。";
            } else if (errorStatus === 404) {
                errorMessage = "APIエンドポイントが見つかりません (HTTP 404)。";
            }
            throw new Error(errorMessage);
        }
        const data = await response.json();
        chartData.datasets[0].data = data.values;
        // PolarArea Chartの背景色とボーダー色をデータに応じて更新
        chartData.datasets[0].backgroundColor = data.values.map(val => {
            if (val > -30) return 'rgba(75, 192, 192, 0.6)'; // 良い値
            if (val > -60) return 'rgba(255, 206, 86, 0.6)'; // 普通
            return 'rgba(255, 99, 132, 0.6)'; // 悪い値
        });
        chartData.datasets[0].borderColor = data.values.map(val => {
            if (val > -30) return 'rgba(75, 192, 192, 1)';
            if (val > -60) return 'rgba(255, 206, 86, 1)';
            return 'rgba(255, 99, 132, 1)';
        });
        azimuthChart.update(); // グラフを更新
    }
    catch (error) {
        console.error("グラフデータの取得中にエラー:", error);
        // グラフ取得エラーの場合もステータスメッセージを更新
        const statusMessageElement = document.getElementById('status-message');
        let message = `グラフデータの取得中にエラー: ${error.message}`;
        let color = 'orange';

        if (error instanceof TypeError && error.message.includes("Failed to fetch")) {
            message = `サーバーに接続できません。バックエンドが実行中か、ネットワーク接続を確認してください。`;
            color = 'red';
        } else if (error.message.includes("HTTPエラー: 404")) {
            message = "APIエンドポイントが見つかりません (HTTP 404)。";
            color = 'red';
        } else if (error.message.includes("認証エラー") || error.message.includes("HTTPエラー: 401")) {
            message = "認証エラー (HTTP 401): APIキーが不正です。index.htmlとconfig.iniのAPI_KEYが一致しているか確認してください。";
            color = 'red';
        }

        if (statusMessageElement) {
            statusMessageElement.textContent = message;
            statusMessageElement.style.color = color;
        } else {
            console.error("fetchGraphData: Element with ID 'status-message' not found.");
        }
    }
}

// --- NMEAデータの取得と表示 ---
async function fetchNMEAData() {
    if (!showNMEA) return; // 表示がOFFなら何もしない
    try {
        const response = await fetch(window.location.origin + '/api/nmea_data', {
            headers: { 'X-API-KEY': API_KEY }
        });
        if (!response.ok) {
            const errorStatus = response.status;
            const errorText = await response.text();
            let errorMessage = `HTTPエラー: ${errorStatus} - ${errorText}`;
            if (errorStatus === 401) {
                errorMessage = "認証エラー: APIキーが不正です。";
            } else if (errorStatus === 404) {
                errorMessage = "APIエンドポイントが見つかりません (HTTP 404)。";
            }
            throw new Error(errorMessage);
        }
        const data = await response.json();
        const nmeaOutput = document.getElementById('nmea-output');
        nmeaOutput.value = data.nmea_lines.join('\n'); // 配列を行ごとに結合
        nmeaOutput.scrollTop = nmeaOutput.scrollHeight; // 最下部にスクロール
    } catch (error) {
        console.error("NMEAデータの取得中にエラー:", error);
        const statusMessageElement = document.getElementById('status-message');
        let message = `NMEAデータの取得中にエラー: ${error.message}`;
        let color = 'orange';

        if (error instanceof TypeError && error.message.includes("Failed to fetch")) {
            message = `サーバーに接続できません。バックエンドが実行中か、ネットワーク接続を確認してください。`;
            color = 'red';
        } else if (error.message.includes("HTTPエラー: 404")) {
            message = "APIエンドポイントが見つかりません (HTTP 404)。";
            color = 'red';
        } else if (error.message.includes("認証エラー") || error.message.includes("HTTP_401")) {
            message = "認証エラー (HTTP 401): APIキーが不正です。index.htmlとconfig.iniのAPI_KEYが一致しているか確認してください。";
            color = 'red';
        }

        if (statusMessageElement) {
            statusMessageElement.textContent = message;
            statusMessageElement.style.color = color;
        } else {
            console.error("fetchNMEAData: Element with ID 'status-message' not found.");
        }
    }
}


// --- イベントリスナー ---
document.addEventListener('DOMContentLoaded', () => {
    console.log("DOMContentLoaded: Event fired. Calling initMap()."); // 診断用ログ
    initMap();
    console.log("DOMContentLoaded: initMap() called. Initializing charts and intervals."); // 診断用ログ
    
    // #map要素が取得できなかった場合は、以降の処理は行わない
    if (!map) {
        console.error("Map initialization failed. Skipping further UI/data setup.");
        return;
    }

    initChart(); // Chart.jsの初期化
    setInterval(fetchSensorData, 2000); // 2秒ごとにデータ更新
    setInterval(fetchGraphData, 5000); // 5秒ごとにグラフデータ更新 (変更なし)
    setInterval(fetchNMEAData, 2000); // 2秒ごとにNMEAデータ更新

    // 地図追従チェックボックス
    document.getElementById('followMapCheckbox').addEventListener('change', (e) => {
        followMap = e.target.checked;
    });

    // グリッド線表示チェックボックス
    document.getElementById('gridCheckbox').addEventListener('change', (e) => {
        if (document.getElementById('gridCheckbox').checked) {
            updateGrid(map.getBounds());
        } else {
            gridLayer.clearLayers();
        }
    });

    // 座標系セレクタ
    document.getElementById('coordSystemSelect').addEventListener('change', (e) => {
        currentCoordSystem = e.target.value;
        fetchSensorData(); // 座標系が変更されたら表示を更新するために、すぐにデータ取得をトリガー
    });

    // IMUキャリブレーションボタン
    document.getElementById('calibrateButton').addEventListener('click', async () => {
        const button = document.getElementById('calibrateButton');
        let action = '';
        if (button.classList.contains('calibrating')) {
            action = 'stop';
            button.classList.remove('calibrating');
            button.textContent = 'IMUキャリブレーション開始';
        } else {
            action = 'start';
            button.classList.add('calibrating');
            button.textContent = 'キャリブレーション停止';
        }
        
        try {
            const response = await fetch(window.location.origin + '/api/calibrate_imu', {
                method: 'POST',
                headers: {
                    'Content-Type': 'application/json',
                    'X-API-KEY': API_KEY
                },
                body: JSON.stringify({ action: action })
            });
            if (!response.ok) {
                const errorStatus = response.status;
                const errorText = await response.text();
                let errorMessage = `HTTPエラー: ${errorStatus} - ${errorText}`;
                if (errorStatus === 401) {
                    errorMessage = "認証エラー: APIキーが不正です。";
                } else if (errorStatus === 404) {
                    errorMessage = "APIエンドポイントが見つかりません (HTTP 404)。";
                }
                throw new Error(errorMessage);
            }
            const result = await response.json();
            const statusMessageElement = document.getElementById('status-message');
            if (statusMessageElement) {
                statusMessageElement.textContent = `キャリブレーション: ${result.status}`;
                if (result.offset !== undefined) {
                    statusMessageElement.textContent += ` オフセット: ${result.offset.toFixed(5)}`;
                }
                statusMessageElement.style.color = 'blue';
            } else {
                console.error("calibrateButton: Element with ID 'status-message' not found.");
            }
        } catch (error) {
            console.error("キャリブレーションAPIエラー:", error);
            const statusMessageElement = document.getElementById('status-message');
            if (statusMessageElement) {
                statusMessageElement.textContent = `キャリブレーションエラー: ${error.message}`;
                statusMessageElement.style.color = 'red';
            } else {
                console.error("calibrateButton error handler: Element with ID 'status-message' not found.");
            }
            // エラー時は元の状態に戻す
            if (action === 'start') {
                button.classList.remove('calibrating');
                button.textContent = 'IMUキャリブレーション開始';
            } else {
                button.classList.add('calibrating');
                button.textContent = 'キャリブレーション停止';
            }
        }
    });

    // ログレベル設定ボタン
    document.getElementById('setLogLevelButton').addEventListener('click', async () => {
        const logLevel = document.getElementById('logLevelSelect').value;
        try {
            const response = await fetch(window.location.origin + '/api/set_log_level', {
                method: 'POST',
                headers: {
                    'Content-Type': 'application/json',
                    'X-API-KEY': API_KEY
                },
                body: JSON.stringify({ level: logLevel })
            });
            if (!response.ok) {
                const errorStatus = response.status;
                const errorText = await response.text();
                let errorMessage = `HTTPエラー: ${errorStatus} - ${errorText}`;
                if (errorStatus === 401) {
                    errorMessage = "認証エラー: APIキーが不正です。";
                } else if (errorStatus === 404) {
                    errorMessage = "APIエンドポイントが見つかりません (HTTP 404)。";
                }
                throw new Error(errorMessage);
            }
            const result = await response.json();
            const statusMessageElement = document.getElementById('status-message');
            if (statusMessageElement) {
                statusMessageElement.textContent = `ログレベル設定: ${result.status}`;
                statusMessageElement.style.color = 'blue';
            } else {
                console.error("setLogLevelButton: Element with ID 'status-message' not found.");
            }
        } catch (error) {
            console.error("ログレベル設定APIエラー:", error);
            const statusMessageElement = document.getElementById('status-message');
            if (statusMessageElement) {
                statusMessageElement.textContent = `ログレベル設定エラー: ${error.message}`;
                statusMessageElement.style.color = 'red';
            } else {
                console.error("setLogLevelButton error handler: Element with ID 'status-message' not found.");
            }
        }
    });

    // 角度幅スライダー
    document.getElementById('fanSlider').addEventListener('input', (e) => {
        document.getElementById('fanValue').textContent = e.target.value;
        // スライダーを動かしたときに、最新のデータを使って扇形を更新
        fetchSensorData(); // 最新のGPSデータを取得し、updateFanを呼び出す
    });

    // NMEA表示切り替えボタン
    document.getElementById('nmeaToggleButton').addEventListener('click', () => {
        showNMEA = !showNMEA;
        const nmeaContainer = document.getElementById('nmea-container');
        console.log(`NMEA Toggle: showNMEA=${showNMEA}`); // 診断用ログ
        
        // nmeaContainerが存在するか確認するログ
        if (nmeaContainer) {
            console.log("NMEA Toggle: nmeaContainer element found.");
            if (showNMEA) {
                nmeaContainer.style.display = 'flex';
                console.log("NMEA Toggle: Setting display to 'flex'."); // 診断用ログ
                fetchNMEAData(); // 表示開始時に一度データを取得
            } else {
                nmeaContainer.style.display = 'none';
                console.log("NMEA Toggle: Setting display to 'none'."); // 診断用ログ
            }
            console.log("NMEA Toggle: NMEA container display is now:", nmeaContainer.style.display); // 診断用ログ
        } else {
            console.error("NMEA Toggle Error: Element with ID 'nmea-container' not found in the DOM.");
        }
    });

    // シンボル配置ボタン
    document.getElementById('placeSymbolButton').addEventListener('click', () => {
        isPlacingSymbol = !isPlacingSymbol;
        const button = document.getElementById('placeSymbolButton');
        if (isPlacingSymbol) {
            button.classList.add('active');
            button.textContent = 'シンボル配置停止';
            const statusMessageElement = document.getElementById('status-message');
            if (statusMessageElement) {
                statusMessageElement.textContent = "シンボル配置モード: ON - 地図をクリックして配置";
                statusMessageElement.style.color = 'green';
            } else {
                console.error("placeSymbolButton: Element with ID 'status-message' not found.");
            }
            map.getContainer().style.cursor = 'crosshair'; // マップカーソルを十字に変更
        } else {
            button.classList.remove('active');
            button.textContent = 'シンボル配置';
            const statusMessageElement = document.getElementById('status-message');
            if (statusMessageElement) {
                statusMessageElement.textContent = "シンボル配置モード: OFF";
                statusMessageElement.style.color = 'blue';
            } else {
                console.error("placeSymbolButton: Element with ID 'status-message' not found.");
            }
            map.getContainer().style.cursor = 'grab'; // マップカーソルをデフォルトに戻す
        }
    });
});
