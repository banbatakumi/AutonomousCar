# SURGE Mark.2 UART プロトコル — v0.28 → v0.29 差分 (制動の減速度・MD の遅れを実行時パラメータに)

**日付**: 2026-10-19
**状態**: STM32 側・Pi 側とも実装済み・ビルドとホストのテストのみ。**未書き込み・実機未検証**

仕様の正は `src/control/control_params.h` (表) と上位の `raspi/proto/protocol.toml`。ここは経緯だけ。

## 何が変わったか

ワイヤ形式は変えていない。`CONFIG_SET` で受け付ける `param_id` が 2 つ増え、`vehicle.h` の定数が 2 つ消えた。

| 名前 | `param_id` | 仮置き | 範囲 | 意味 | 消えた定数 |
|---|---|---|---|---|---|
| `brake_decel_m_s2` | 0x0101 | 0.5 | 0.5〜10 | 制動モード (ABS 込み) の減速度 [m/s²] | `VEHICLE_BRAKE_DECEL_M_S2` (2.47)・`VEHICLE_AUTO_STOP_DECEL_M_S2` |
| `md_torque_lag_s` | 0x0102 | 0 | 0〜0.1 | MD のトルクの遅れ [s] | `VEHICLE_MD_TORQUE_TAU_S` (0.0045) |

使い道は変わらない:

- 自動停止の停止距離 `d = v·(1/LIDAR_SCAN_FREQ_HZ + md_torque_lag_s) + v²/(2·brake_decel_m_s2) + マージン`
  (`AutoStopDistanceCm`。LiDAR の 1 回転は `VEHICLE_AUTO_STOP_SENSOR_DELAY_S` に残る)
- 走行中に ARM が外れたときに制動を続ける上限 `DRIVE_MAX_SPEED_M_S / brake_decel_m_s2 × VEHICLE_STOP_TIME_MARGIN`
  (`DisarmBrakeTimeoutS`。旧 `VEHICLE_DISARM_BRAKE_TIMEOUT_S`)

## なぜ

どちらも上位の校正の結果 (`config/vehicle.toml` の `[limits] abs_brake_decel_m_s2` = 校正 8、`[control.plant]
md_delay_s + md_tau_s` = 校正 6) で、校正し直すたびにファームの写しを直して書き込む二重管理になっていた。
上位が全項目を送って `CONFIG_ACK` で確かめるまで ARM を通さないので、ファームは値を持たなくてよい (仮置きはほかの項目と
同じく範囲の下限で、走行には使われない)。

## 上位の対応

`raspi/proto/protocol.toml` 0x001D、`raspi/core/control_params.py` の `DERIVED` (toml の 2 か所から作って送る。`[control]`
には書けない)・`disarm_brake_timeout_s` (io_node の異常時の制動の上限とシムも同じ式)、`sim/stm32.py`。
`tools/fwsim/tests/test_fw_constants.py` はファームに写しが戻っていないことを確かめる。
