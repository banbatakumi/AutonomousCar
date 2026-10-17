# SURGE Mark.2 UART プロトコル — v0.23 → v0.26 差分 (車速のカルマンフィルタ)

**日付**: 2026-10-17
**状態**: STM32 側・Pi 側とも実装済み。**STM32 に書き込み済み、Pi へ反映済み (駆動電源なしで確認)。走行での検証は未了**

仕様の正は `src/comm/ras_link.h`・`ras_link.c` と上位の `raspi/proto/protocol.toml`。ここは経緯だけ。

## v0.24 (0x0018): 車速のカルマンフィルタを並走

TELEMETRY (84→) に `speed_kf`・`speed_lpf`・`speed_source` を足し、CONFIG に `SPEED_SOURCE` を足した。前輪の微分+LPF と
カルマンフィルタの車速を同時に出して比べるための版 (この版の記録だけ両方ある)。

## v0.25 (0x0019): 前輪の微分+LPF を廃止

`speed` は**走行距離と加速度計のカルマンフィルタ** (`src/control/speed_kf.{h,c}`) の車速だけ。`speed_kf`・`speed_lpf`・`speed_source`・
CONFIG の `SPEED_SOURCE` は削除。TELEMETRY は **90 バイト** (末尾に `accel_kf` i16・`accel_bias_kf` i16・`kf_gated` u16)。
車速PI・TC/ABS のスリップの基準・自動停止の前後判定がこれを使う。`Encoder_Update` の `dt` が関数の処理時間 (約9µs) ぶん短く、
前輪の車速が約 1.8% 速く出ていた既存の誤りも直した (角度を読む瞬間どうしの DWT の差にした)。

## v0.26 (0x001A): 一貫性の診断と前輪の絶対角

TELEMETRY は **102 バイト** (末尾に追加。既存の位置は変えていない):

| off | 型 | 名前 | 意味 |
|---|---|---|---|
| 90 | u16 ×0.001 | `kf_z_rms_dist` | 走行距離の残差の z の RMS (= √NIS)。前提が合っていれば 1。0 = その間に測定なし |
| 92 | u16 ×0.001 | `kf_z_rms_accel` | 加速度の残差の z の RMS |
| 94 | i16 ×0.0001 | `kf_rho_dist` | 走行距離の残差の隣り合う相関係数 (-1〜1)。前提が合っていれば 0 |
| 96 | i16 ×0.0001 | `kf_rho_accel` | 加速度の残差の相関係数 |
| 98 | u16 ×0.0001 rad ×2 | `front_angle[2]` | 前輪エンコーダの絶対角 [FL, FR] (0〜2π、取付向きの符号のまま)。周期誤差の同定用 |

z は残差 ÷ フィルタが主張する標準偏差。NIS を直接送らないのは、静止で 1e-3 まで小さく、量子化で潰れるため
(RMS と ρ なら窓ごとの NIS・自己相関に正確に戻せる)。検定は上位の `raspi/tools/kf_consistency.py`。

## ファーム内の他の変更 (ワイヤ形式に関係しない)

- `drive.c`: 制御ループが長く止まったとき、前輪の角度差が 1 回転ぶん折り返すのを、KF の予測した増分に近い側へ戻す。
  `SPEED_KF_MAX_DT_S` を 50ms→1s にし、1s 以上の停止は初期化して最初の増分を捨てる (`speed_kf.c` の `resync`)
- `lib/flash/flash.h` `Flash_ReadData`: `uint32_t*` への型替えをやめて `memcpy` にした (`-O2` で IMU の較正レコードが全部 0 に読まれ、
  較正が捨てられていた。`e3e40bf` 以降)
- IMU の DLPF を 21Hz→94Hz にした (遅れ 8.5→3ms。ジャイロは TC・TV に効く)
