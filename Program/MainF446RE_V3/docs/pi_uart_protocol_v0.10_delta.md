# SURGE Mark.2 UART プロトコル — v0.9 → v0.10 差分

**発信**: STM32F446RE 車両制御基板 (MainF446RE_V3) 開発側
**宛先**: Raspberry Pi 5 側 開発担当
**日付**: 2026-08-21
**状態**: STM32 側 実装済み・ビルド確認済み。**Pi 側の対応が必要 (実機での動作検証は未了)**

v0.9 (`pi_uart_protocol_v0.9_delta.md`) からの差分のみを記す。
**変わったのは `CONFIG_SET`/`CONFIG_GET` で使える `param_id` が減っただけ**で、
`COMMAND`/`TELEMETRY` を含めどのパケットも構造・LEN は変わっていない。

---

## 0. 要約

`RAS_PARAM_MAX_SPEED` (0x0001) / `RAS_PARAM_MAX_ACCEL` (0x0002) / `RAS_PARAM_MAX_STEER`
(0x0003) を廃止した。これらは上位から最大速度・最大加速度・最大舵角を実行時に変更する
仕組みだったが、実用上その必要が無いため、STM32 側の固定定数
(`DRIVE_MAX_SPEED_M_S` / `DRIVE_MAX_ACCEL_M_S2` / `Steering_GetMaxRoadWheelAngleRad()`)
に一本化した。

これに伴い、`COMMAND` の目標車速の加速度レート制限も STM32 側 (`src/control/drive.c`)
へ移した。挙動自体に変更は無く、上限値が `DRIVE_MAX_ACCEL_M_S2` (3.0 m/s²、旧
`RAS_PARAM_MAX_ACCEL` の既定値と同じ) に固定される点だけが変わる。

| 廃止した操作 | 旧の手段 |
|---|---|
| 最大速度・最大加速度・最大舵角の変更 | `CONFIG_SET` の `param_id = 0x0001/0x0002/0x0003` |
| 上記の現在値取得 | `CONFIG_GET` の同じ `param_id` |

**`protocol_version` は `0x0009` → `0x000A` に上がる。**
`CONFIG_SET`/`CONFIG_GET` は未知の `param_id` に対して `RAS_CONFIG_UNKNOWN_PARAM` を
返す設計のため、Pi 側がこの3つの `param_id` を送り続けていても通信自体は継続する
(その `param_id` だけ `UNKNOWN_PARAM` が返るようになる)。

**`COMMAND` の `accel_limit_m_s2` / `steer_rate_limit_rad_s` フィールドはそのまま残る。**
これらは「今回の指令をどれだけ緩やかに追従させるか」という毎指令ごとの値であり、
`CONFIG_SET` で変える走行上限そのものとは別物のため、今回は変更していない。
上限のクランプ先が `config.max_accel_m_s2` (上位が変更可能だった) から
`DRIVE_MAX_ACCEL_M_S2` (固定) に変わっただけで、フィールドの意味・送り方は従来通り。

---

## 1. `CONFIG_SET` (0x11) / `CONFIG_GET` (0x13) — LEN 変更なし

### 1.1 `param_id` 一覧 ★廃止

| 旧 param_id | 名称 | 廃止後の扱い |
|---|---|---|
| 0x0001 | `max_speed` | `CONFIG_SET`/`CONFIG_GET` とも `RAS_CONFIG_UNKNOWN_PARAM` を返す。`COMMAND.target_speed_m_s` は `DRIVE_MAX_SPEED_M_S` (5.0 m/s) で頭打ちになる |
| 0x0002 | `max_accel` | 同上。`COMMAND.accel_limit_m_s2` の上限は `DRIVE_MAX_ACCEL_M_S2` (3.0 m/s²) 固定 |
| 0x0003 | `max_steer` | 同上。`COMMAND.target_steer_rad` は `Steering_GetMaxRoadWheelAngleRad()` (路面舵角 ±30度、モータ機械角基準±60度にリンク比0.5を掛けた値) で頭打ちになる |

- 既に Pi 側がこれらを起動時に一度だけ送って初期値を揃える設計になっている場合、
  送信自体は無害 (`UNKNOWN_PARAM` が返るだけ) だが、GUI 等でその返り値を見て
  異常扱いしていないか確認すること。

---

## 2. Pi 側の対応チェックリスト

- [ ] `protocol_version` の期待値を `0x0009` → `0x000A` にする
- [ ] `param_id = 0x0001/0x0002/0x0003` の `CONFIG_SET`/`CONFIG_GET` を送っている場合は
      削除する (送り続けても `RAS_CONFIG_UNKNOWN_PARAM` が返るだけで通信は継続するが、
      もう意味のない操作になる)
- [ ] 最大速度・最大加速度・最大舵角を Pi 側で把握したい場合は、今後は STM32 側の
      固定定数として扱う (`DRIVE_MAX_SPEED_M_S` = 5.0 m/s, `DRIVE_MAX_ACCEL_M_S2` = 3.0 m/s²,
      路面舵角上限 = ±30度)。実行時に取得する手段は無くなったため、変更があれば
      別途このドキュメントで通知する
- [ ] `COMMAND.accel_limit_m_s2` / `steer_rate_limit_rad_s` による毎指令ごとの
      レート制限指定は従来通り使える (未指定/0 は既定値扱い)
