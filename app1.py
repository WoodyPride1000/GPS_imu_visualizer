# app.py の既存のインポート文の下あたりに追加
from collections import deque

# --- グラフデータを保持するキュー ---
# 方位角と、それに対応するグラフのY軸データ（-20〜-99）を保持します。
# グラフの表示数に合わせて適切な長さに設定してください。
GRAPH_DATA_MAX_POINTS = 100 # 表示するデータの最大点数

# データを保持するグローバルなdeque
graph_data_queue = deque(maxlen=GRAPH_DATA_MAX_POINTS)

# --- calculate_heading_and_error_thread 関数にグラフデータ追加ロジックを追加 ---
# calculate_heading_and_error_thread 関数内の `with sensor_data.lock:` のブロックの最後あたりに以下を追加します。
# ここでは例として、算出したheading_fusedとランダムな値をグラフデータとして追加しています。
# 「別のシリアルデータ」の具体的な内容に応じて、この部分を修正してください。
# 例: 別のシリアルポートから読み込んだデータをここに反映させる、など。
# sensor_data.heading_fused が方位角、Y軸のデータは-20から-99の範囲に加工する必要があります。
# 実際のシリアルデータに基づいて、Y軸の値を計算するロジックをここに記述します。

# 例: 仮に、imu_raw_gyro_z を Y軸データにマッピングする場合（範囲を-99から-20に調整）
# imu_raw_gyro_z の範囲が分からないため、ここでは適当な正規化とマッピングを行います。
# 実際のデータに応じて調整してください。
# ここでは、heading_fused を X軸に、-20から-99の範囲のランダムな値をY軸としています。
# 実際の「別のシリアルデータ」をここにマッピングしてください。
# 例えば、センサーデータが0〜100の範囲で、それを-20〜-99にマッピングするなら
# mapped_value = -20 - (original_value / 100) * 79 # 79は99-20
# のような計算が必要になります。
mapped_graph_y_value = random.uniform(-99.0, -20.0) # 仮のデータ。実際のシリアルデータから計算する
graph_data_queue.append({'azimuth': sensor_data.heading_fused, 'value': mapped_graph_y_value})


# --- Flask Webアプリケーションに新しいAPIエンドポイントを追加 ---
@app.route("/api/graph_data")
def api_graph_data():
    with sensor_data.lock: # sensor_data.lock を使用して graph_data_queue へのアクセスを保護
        # dequeのデータをリストに変換して返します
        # グラフライブラリが扱いやすいように、X軸データとY軸データを分離して返却することも可能です
        azimuths = [d['azimuth'] for d in graph_data_queue]
        values = [d['value'] for d in graph_data_queue]
    return jsonify({
        "azimuths": azimuths,
        "values": values
    })
