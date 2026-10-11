# SURGE Mark.2 UART プロトコル — v0.7 → v0.8 差分

**発信**: STM32F446RE 車両制御基板 (MainF446RE_V3) 開発側
**宛先**: Raspberry Pi 5 側 開発担当
**日付**: 2026-08-19
**状態**: STM32 側 実装済み・ビルド確認済み。**Pi 側の対応が必要 (実機での動作検証は未了)**

v0.7 (`pi_uart_protocol_v0.7_delta.md`) からの差分のみを記す。
**変わったのは `CONFIG_SET`/`CONFIG_GET` で使える `param_id` が増えただけ**で、
`COMMAND`/`TELEMETRY` を含めどのパケットも構造・LEN は変わっていない。

---

## 0. 要約

トラクションコントロール (TC) とトルクベクタリング (TV) を、Pi から実行時に ON/OFF
できるようにした。手段は既存の `CONFIG_SET` (0x20) / `CONFIG_GET` (0x21) で、
以前から「予約済みだが未対応」だった `param_id` 帯を今回実装した。

| 追加した操作 | 手段 |
|---|---|
| TC の有効/無効切替 | `CONFIG_SET` の `param_id = 0x0010` |
| TV の有効/無効切替 | `CONFIG_SET` の `param_id = 0x0020` |
| 現在の TC/TV 有効状態の取得 | `CONFIG_GET` の同じ `param_id` |

**`protocol_version` は `0x0007` → `0x0008` に上がる。**
`CONFIG_SET`/`CONFIG_GET` は元々未知の `param_id` に対して `RAS_CONFIG_UNKNOWN_PARAM`
を返す設計だったため、**Pi 側が未対応でも通信自体は継続する** (この2つの `param_id` を
送らなければ従来通り TC/TV とも常時有効のまま)。

---

## 1. `CONFIG_SET` (0x20) / `CONFIG_GET` (0x21) — LEN 変更なし

### 1.1 `param_id` 一覧 ★追加

| param_id | 名称 | 型 | 値の意味 |
|---|---|---|---|
| 0x0010 | `tc_enable` | f32 (bool 扱い) | 0.0 = 無効、非0 = 有効。既定は有効 |
| 0x0020 | `tv_enable` | f32 (bool 扱い) | 0.0 = 無効、非0 = 有効。既定は有効 |

- ペイロード形式は既存の `CONFIG_SET`/`CONFIG_GET`/`CONFIG_ACK` と同じ (`param_id` u16 +
  `value`/`applied` f32)。真偽値も他のパラメータと型を揃えるため f32 で送受信する。
- `CONFIG_ACK.applied` には実際に適用された値 (`0.0` か `1.0`) が入る。`value` に
  `0.0`/`1.0` 以外を送った場合、`(value != 0.0)` で真偽判定した上で `0.0`/`1.0` に
  丸めて適用し、丸めが発生した (元の値と一致しない) ぶん `result` は
  `RAS_CONFIG_OUT_OF_RANGE` になる (`RAS_PARAM_LIDAR_FORMAT` と同じ量子化の扱い)。
- `0x0011` (TC のゲイン) と `0x0021` (TV のゲイン)、`0x0030`/`0x0031` (速度PIのゲイン) は
  **今回も未対応のまま**。Drive/TorqueVectoring 側がまだ定数でしか持っていないため、
  受信すると引き続き `RAS_CONFIG_UNKNOWN_PARAM` を返す。

### 1.2 TC/TV 無効時の挙動

- TC を無効にすると、各後輪のトルク上限を常に最大 (`DRIVE_MAX_TORQUE_NM`) に固定し、
  スリップ率によるトルク削り込みを一切行わない (空転しても削らない)。
- TV を無効にすると、左右後輪へ常に等トルクを配分する (オープンデフ相当。ヨーレート偏差
  によるトルク差付けをしない)。
- どちらも設定はマイコン起動時・COMMAND 未受信中は既定 (有効) に戻る。COMMAND が生きている
  間は毎周期、直近に `CONFIG_SET` で設定された値が適用され続ける。

---

## 2. Pi 側の対応チェックリスト

- [ ] `protocol_version` の期待値を `0x0007` → `0x0008` にする
- [ ] TC/TV を切り替えたい場合のみ `CONFIG_SET` (`param_id = 0x0010` / `0x0020`) を送る
      (送らなければ両方とも既定で有効のまま変わらない)
- [ ] 現在の設定値を確認したい場合は `CONFIG_GET` で同じ `param_id` を問い合わせる
- [ ] **実機でのTC/TV ON/OFF切替の動作検証は STM32 側でも未実施。** 挙動に疑問があれば
      切り分けのため一度 OFF にして走らせてみることを推奨する
