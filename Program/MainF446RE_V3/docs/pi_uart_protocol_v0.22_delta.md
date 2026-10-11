# SURGE Mark.2 UART プロトコル — v0.21 → v0.22 差分

**日付**: 2026-10-11
**状態**: STM32 側・Pi 側とも実装済み・ビルド確認済み。**未書き込み、実機での動作検証は未了** (MD は変更なし)

v0.21 (`pi_uart_protocol_v0.21_delta.md`) からの差分のみを記す。**ワイヤ形式 (LEN・オフセット)・`param_id` は同じ**で、
変わるのは `COMMAND.accel_limit` の扱いと `LIMITS.max_accel_m_s2` の値だけ。

---

## 0. 要約

1. **目標車速のランプの固定上限 (`DRIVE_MAX_ACCEL_M_S2` 3.0m/s²) を廃止した** (§1)。`COMMAND.accel_limit` が正なら
   その値でランプする。上限は上位が決める。
2. **`LIMITS.max_accel_m_s2` は 65.535 を返す** (§2)。ワイヤ (u16 × 0.001) で表せる最大 = 「STM32 側の上限なし」。
3. **`accel_limit = 0` のときの既定の傾きは 3.0m/s² のまま** (`DRIVE_DEFAULT_ACCEL_M_S2`)。
4. **自動停止の停止距離の式は変えていない** (§3)。減速度の定数を `VEHICLE_AUTO_STOP_DECEL_M_S2` 3.0 に分けただけ。

**`protocol_version` は `0x0015` → `0x0016` に上がる。上位と一緒に入れ替えること** (上位の io_node は版が合わない
STM32 を ARM しない)。★古い上位 (`[safety] cmd_accel_limit_m_s2` = 6.0) と組み合わせると、手動操作の目標車速が
6.0m/s² でランプする。版を上げたのはこの組み合わせを通さないため。

---

## 1. ランプの上限の廃止

### なぜ無くすか

- 出せる加速はタイヤのグリップで決まり、路面で変わる (フローリングの同定で駆動 2.8・減速 2.5m/s²、畳ではそれより高い。
  上位側 `surge_mk2/PROGRESS.md` 2026-10-11 節)。固定の 3.0 が残っていると、上位が同定し直してもファームを
  書き直すまで使えない。
- 上位は以前から `COMMAND.accel_limit` を毎回送っている (GUI は `[safety] cmd_accel_limit_m_s2`、自律走行は経路の
  加減速)。STM32 の 3.0 はその上にもう一段かぶさっていただけ。
- 滑らせない役目は TC が、暴走を止める役目はトルク上限 (`DRIVE_MAX_TORQUE_NM`) と速度上限 (`DRIVE_MAX_SPEED_M_S`)
  が持っている。トルク上限で出せる加速は 2 輪合計 0.26Nm × 車体の利得 約18〜21 ≒ 5m/s² で頭打ちになる。

### 残したもの

- **ランプそのもの** (`Drive_Update` が目標車速を `accel_limit` の傾きで動かす)。v0.21 の加速度フィードフォワードは
  この傾きに要るトルクを足すので、ランプが無いと成り立たない。
- `accel_limit` の下限 0.01m/s² (`DRIVE_MIN_ACCEL_LIMIT_M_S2`)。
- `accel_limit = 0` → 既定の 3.0m/s² (`DRIVE_DEFAULT_ACCEL_M_S2`)。「0 = 制限なし」とは読まない。

### コード

- `src/control/drive.h`: `DRIVE_MAX_ACCEL_M_S2` を削除し、`DRIVE_DEFAULT_ACCEL_M_S2` 3.0・`DRIVE_MIN_ACCEL_LIMIT_M_S2` 0.01 を新設
- `src/control/drive.c`: `Drive_SetTargetSpeed` が上限でクランプしない

## 2. `LIMITS.max_accel_m_s2`

65.535 (`RAS_ACCEL_LIMIT_WIRE_MAX_M_S2`) を返す。上位の io_node はこの値で `accel_limit` を切るので、実質何も切らない。
上位の実効の上限は `config/vehicle.toml` の `[safety] cmd_accel_limit_m_s2` (v0.22 で 6.0 → 3.0。v0.21 までの実効値と同じ)。

## 3. 自動停止

`d_stop = v・VEHICLE_AUTO_STOP_DELAY_S + v²/(2・VEHICLE_AUTO_STOP_DECEL_M_S2) + margin`。v0.21 までは減速度に
`DRIVE_MAX_ACCEL_M_S2` を流用していた。値は 3.0 のままなので動きは変わらない。実測の制動は ABS 込みで約2.5m/s²
(上位の同定、フローリング) なので少し楽観的 — 測り直したら更新する。

## 4. 上位側の対応 (実装済み)

- `raspi/proto/protocol.toml` `protocol_version` 0x0016
- `config/vehicle.toml`: `[safety] cmd_accel_limit_m_s2` 6.0 → 3.0。`[dynamics] speed_ramp_max_m_s2` を削除
  (`Vehicle.speed_ramp_max_m_s2`・シムの `VehicleSpec` は `cmd_accel_limit_m_s2` を読む)
- `sim/stm32.py`: `LIMITS.max_accel_m_s2` = 65.535
