# SURGE Mark.2 UART プロトコル — v0.10 → v0.11 差分

**発信**: STM32F446RE 車両制御基板 (MainF446RE_V3) 開発側
**宛先**: Raspberry Pi 5 側 開発担当
**日付**: 2026-08-21
**状態**: STM32 側 実装済み・ビルド確認済み。**Pi 側の対応が必要 (実機での動作検証は未了)**

v0.10 (`pi_uart_protocol_v0.10_delta.md`) からの差分のみを記す。
**新しいパケットタイプ `LIMITS` (0x0A) / `LIMITS_REQ` (0x15) を追加しただけ**で、
`COMMAND`/`TELEMETRY`/`CONFIG_SET`/`CONFIG_GET` を含め既存パケットの構造・LEN は
変わっていない。

---

## 0. 要約

v0.10 で `RAS_PARAM_MAX_SPEED`/`MAX_ACCEL`/`MAX_STEER` (上位からの上限変更) を廃止し、
STM32 側の固定定数に一本化した。その結果、Pi 側 (ラジコンモード・自律走行モードそれぞれの
最大速度・最大加速度・最大トルク・最大舵角の設定) が実機の本当の上限を知る手段が無くなって
いたため、STM32 側からその固定上限値を**通知する**専用パケット `LIMITS` を新設した。
`VERSION`/`VERSION_REQ` と同じパターンで、起動直後に自動送信 (バースト) されるほか、
Pi 側からいつでも `LIMITS_REQ` で再取得できる。

| 追加した操作 | 手段 |
|---|---|
| 車両の固定上限値の取得 (起動時自動) | STM32 起動直後に `VERSION` と同じバーストで自動送信 (100ms間隔 ×3回) |
| 車両の固定上限値の取得 (任意タイミング) | `LIMITS_REQ` (0x15, ペイロード無し) を送ると `LIMITS` が返る |

**`protocol_version` は `0x000A` → `0x000B` に上がる。**
新しい TYPE を追加しただけなので、Pi 側が未対応でも既存の通信 (COMMAND/TELEMETRY等) には
一切影響しない。`LIMITS_REQ` を送らなければ起動直後のバーストしか届かないだけで、
通信自体は継続する。

---

## 1. `LIMITS` (0x0A) — STM32 → Pi、新設

ペイロード 16 バイト、量子化なし (f32 そのまま、リトルエンディアン)。

| offset | 型 | 名称 | 意味 |
|---|---|---|---|
| 0 | f32 | `max_speed_m_s` | 車速の上限 [m/s] (`DRIVE_MAX_SPEED_M_S`)。これを超える `COMMAND.target_speed_m_s` は STM32 側でクランプされる |
| 4 | f32 | `max_accel_m_s2` | 加速度の上限 [m/s²] (`DRIVE_MAX_ACCEL_M_S2`)。`COMMAND.accel_limit_m_s2` に指定できる上限でもある |
| 8 | f32 | `max_torque_nm` | 1輪あたりのトルク上限 [Nm] (`DRIVE_MAX_TORQUE_NM`)。駆動 (`target_torque_nm`) と制動 (`brake_torque_nm`) の両方が同じ値を共有する (STM32 側で駆動・制動トルクの上限は同一に固定してあるため) |
| 12 | f32 | `max_steer_rad` | 路面舵角の上限 [rad]、原点(直進)からの片側の振れ幅 (`Steering_GetMaxRoadWheelAngleRad()`)。現在の実機値は約 ±30度 (0.5236 rad) |

- いずれも**読み取り専用**。CONFIG_SET のような書き込み手段は無い (v0.10 で意図的に廃止した)。
- 値を変えたい場合は STM32 側のファームウェア (`src/control/drive.h` の
  `DRIVE_MAX_SPEED_M_S`/`DRIVE_MAX_ACCEL_M_S2`/`DRIVE_MAX_TORQUE_NM`、
  `src/control/steering.h` の `STEERING_MAX_ANGLE_RAD`/`STEERING_LINKAGE_RATIO`) を
  書き換えて再書き込みする必要がある。実行時に変わることは無いので、Pi 側は起動時に
  一度取得してキャッシュしておけば十分。

## 2. `LIMITS_REQ` (0x15) — Pi → STM32、新設

ペイロード無し (LEN=0)。受信すると即座に `LIMITS` を1回返す。`VERSION_REQ` (0x14) と
同じ使い方 (再接続時・タイムアウト再送などで能動的に取得したいとき用)。

---

## 3. Pi 側の対応チェックリスト

- [ ] `protocol_version` の期待値を `0x000A` → `0x000B` にする
- [ ] `LIMITS` (0x0A) パケットのパースを追加する (16バイト、f32×4、量子化無し)
- [ ] ラジコンモード・自律走行モードの最大速度・最大加速度・最大トルク・最大舵角の
      設定値は、`LIMITS` で受け取った値を超えないようにクランプする
      (超えて送っても STM32 側で頭打ちになるだけだが、Pi 側の意図した制御と実際の
      挙動がズレる原因になるため、Pi 側でも揃えておくことを推奨)
- [ ] 起動直後のバースト (最大3回、100ms間隔) を取りこぼした場合や、再接続直後に
      改めて取得したい場合は `LIMITS_REQ` (0x15, ペイロード無し) を送る
- [ ] **実機での動作検証は STM32 側でも未実施。** 特に `max_steer_rad` の元になる
      `STEERING_LINKAGE_RATIO` (モータ機械角→路面舵角の換算比) は実測前の机上値であり、
      実際の路面舵角上限とズレている可能性がある
