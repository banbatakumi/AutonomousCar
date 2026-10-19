# SURGE Mark.2 UART プロトコル — v0.27 → v0.28 差分 (ジャイロ z 軸の感度の補正)

**日付**: 2026-10-19
**状態**: STM32 側・Pi 側とも実装済み。既定値は v0.27 と同じ動き (補正なし)

仕様の正は `src/control/control_params.h` (表) と上位の `raspi/proto/protocol.toml`。ここは経緯だけ。

## 何が変わったか

ワイヤ形式 (パケットの長さ・並び) は変えていない。`CONFIG_SET` で受け付ける `param_id` が 1 つ増えた。版を上げたのは、
新しい Pi が古い STM32 に知らない `param_id` を送らないようにするため (上位は版が合わない STM32 を ARM しない)。

| 名前 | `param_id` | 既定 | 範囲 | 意味 |
|---|---|---|---|---|
| `imu_gyro_z_scale` | 0x00F1 | 1 | 0.9〜1.1 | ジャイロ z 軸の感度の補正 (倍率)。`imu.c` がバイアスを引いた後の z の角速度に掛ける |

MPU6050 の感度はデータシートの値 (131 LSB/(°/s)) のままで、公差は ±3%。この誤差はヨーレート・曲率・向きの積分
(推測航法・SLAM の初期値) の倍率にそのまま乗る。補正はバイアスの学習 (補正前の値で持つ) の後、姿勢推定 (AHRS) の前に
掛けるので、TELEMETRY の `yaw_rate`・TC/TV の基準・ピッチ/ロールのすべてが補正後になる。

値は上位の `config/vehicle.toml` の `[control] imu_gyro_z_scale` に手で置く（校正の試験は無い。既定 1 = 補正なし）。

## 上位の対応

`raspi/proto/protocol.toml` 0x001C、`config/vehicle.toml` の `[control] imu_gyro_z_scale`、`raspi/core/control_params.py`。
