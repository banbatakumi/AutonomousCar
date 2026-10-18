# SURGE Mark.2 UART プロトコル — v0.26 → v0.27 差分 (前輪エンコーダの校正・車輪の半径・カルマンフィルタの調整値を実行時に)

**日付**: 2026-10-18
**状態**: STM32 側・Pi 側とも実装済み。**未書き込み・実機未検証**。既定値は v0.26 と同じ動き (校正なし・半径は名目・調整値は v0.26 の固定値)

仕様の正は `src/control/control_params.h` (表) と上位の `raspi/proto/protocol.toml`。ここは経緯だけ。

## 何が変わったか

ワイヤ形式 (パケットの長さ・並び) は変えていない。`CONFIG_SET` で受け付ける `param_id` が増え、TELEMETRY の
`front_angle` の意味が変わった。版を上げたのは、古い Pi が新しい値を送らず、新しい Pi が古い STM32 に知らない
`param_id` を送らないようにするため (上位は版が合わない STM32 を ARM しない)。

### 1. 前輪エンコーダの校正 (`lib/encoder_cal`、`param_id` 0x00C1〜0x00C6 左・0x00C9〜0x00CE 右)

| 名前 | 既定 | 範囲 | 意味 |
|---|---|---|---|
| `enc_*_lo`・`enc_*_hi` | 0・1 | 0〜0.2・0.8〜1 | 角度 0・2π に当たる ADC の読み (フルスケールに対する比) |
| `enc_*_h1s`・`h1c`・`h2s`・`h2c` | 0 | ±0.1 rad | 1回転の周期誤差 e(θ) = h1s·sinθ + h1c·cosθ + h2s·sin2θ + h2c·cos2θ |

`Encoder` が ADC の読みを `EncoderCal_Angle` で真の角度へ直してから差分を取る (車速・`odom_dist`・各輪の周速・
TC の基準がすべて校正後)。校正が変わったら、その場の角度を測り直して累積に段差を作らない。値は上位のエンコーダ
校正試験 (`sysid_encoder`、解析 `tools/sysid/encoder_cal.py`) が決める。

### 2. 車輪の有効転がり半径 (0x00D1・0x00D2 前輪左右、0x00D3・0x00D4 後輪左右)

既定 0.030m、範囲 0.025〜0.035m。前輪は走行距離・車速の倍率、後輪は周速 (スリップ率の基準)。`drive.h` の
`DRIVE_*_WHEEL_RADIUS_M` は名目として残り、トルクベクタリングの式だけが使う。`odom_dist` は増分にそのときの
半径を掛けて積算する (半径を変えても累積に段差は出ない)。

### 3. 車速のカルマンフィルタの調整値 (0x00E1〜0x00E6)

`kf_sigma_s_m` (0.0008)・`kf_sigma_a_m_s2` (0.6)・`kf_sigma_a_per_m_s` (0、新設: 加速度の測定ノイズの車速による増え方)・
`kf_jerk_psd` (100)・`kf_bias_walk_psd` (0.05)・`kf_lever_arm_m` (0.15)。既定は v0.26 の `speed_kf.h` の固定値。
値は上位の `tools/sysid/kf_tune.py` (走行の記録からの最尤推定) が決める。

### 4. TELEMETRY の `front_angle`

v0.26 では「前輪エンコーダの絶対角」。v0.27 からは**校正前の生の読み** (ADC÷フルスケール×2π) で、`enc_*` に依らない。
エンコーダ校正試験を何度録り直しても同じ基準で求め直せるようにするため。

## 上位の対応

`raspi/proto/protocol.toml` 0x001B、`config/vehicle.toml` の `[control]` に上の値 (既定のまま)、`raspi/core/control_params.py`。
STM32 を書き込んだら、上位の手順は `surge_mk2/docs/development.md` の「v0.27 の実機での確かめ方」。
