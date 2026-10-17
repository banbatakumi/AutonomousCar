# CLAUDE.md

自律走行ミニカーの下位制御ファームウェア (STM32F446RE / Nucleo-F446RE)。

- **Raspberry Pi (上位)**: 経路計画・画像認識。UART で走行指令 (速度・舵角など) を送る。
- **STM32 (このリポジトリ)**: モーター制御・トラクションコントロールなどリアルタイム制御、灯火・ブザー、センサ取得 (LiDAR・超音波・エンコーダ・IMU・電流電圧)。
- **MD (BLDC ドライバ)**: `../../../BLDC/ProgramV4` (仕様は同リポジトリの `docs/SERIAL_PROTOCOL.md`)。

**情報の優先順位**: コード (特に `src/comm/ras_link.h`、`src/control/drive.h`、`src/control/control_params.h`) > `docs/` 直下 > `docs/archive/`。`docs/archive/` は過去の経緯で、現行仕様ではない (数値・仕様が古い)。通常は読まない。

## ビルド

```sh
make          # build/MainF446RE_V3.elf/.hex/.bin
make clean
make run      # make -j12 後、SWDFlash.sh (ST-Link, STM32_Programmer_CLI) で 0x08000000 へ書き込み
```

- `Makefile` は CubeMX 生成。ファイル追加時は `.ioc` 再生成か `C_SOURCES` / `C_INCLUDES` を手編集。
- 実機向けの単体テストは無い。`src/control` (Drive・TorqueVectoring・ControlParams) は `host/` でホスト向けにコンパイルでき、上位側リポジトリの `tools/ctrl_tune` が車両モデルと閉ループにして使う (`host/README.md`)。**`src/control` のロジックを変えたら、前後で上位側の `python -m tools.ctrl_tune.bench` と `python -m pytest tools/ctrl_tune/tests` を回す。**

## 現在の状態

- プロトコル: `RAS_PROTOCOL_VERSION` = 0x001A (v0.26)。上位と必ず一緒に入れ替える (上位は版が合わない STM32 を ARM しない)。変更点は `docs/pi_uart_protocol_v0.19_delta.md` 〜 `v0.26_delta.md` (v0.24〜v0.26 は車速のカルマンフィルタ。1つの文書にまとめてある)。
- STM32 は v0.26 (車速のカルマンフィルタ入り、2026-10-17) を書き込み済み (上位が `protocol_version` 0x001A を確認)。**車速は前輪の微分+LPF ではなく走行距離と加速度計のカルマンフィルタ** (`src/control/speed_kf.c`。調整値は経験値を含む。検定は上位の `raspi/tools/kf_consistency.py`、TELEMETRY の `kf_z_rms_*`・`kf_rho_*`)。**MD 側に v0.19 以降 (ARM の「止める」約束・ブレーキホールド・ローンチ・車速PIの加速度FF・ランプ上限廃止) の対応版を書き込んだかは未確認**。
- **実機未検証の機能**: トルクベクタリング (荷重比例の配分)、ステアのリンク換算 (v0.18)、自動停止 (LiDAR 主・超音波補助)、LIMITS パケット、ブレーキホールドの移行条件、ローンチ、加速度FF、省電力化 (超音波/LiDAR PWM 停止)。ABS・TC・片輪浮き対策のしきい値/ゲインは机上値 (上位側の車両モデルで最適化した初期値)。ESS ブレーキ灯のしきい値は未実測。自動停止の遅延 (`VEHICLE_AUTO_STOP_DELAY_S`) は LiDAR の1回転＋MD のトルクの立ち上がりから求めた値で、通しでは未実測。
- **作業ツリーにあって未書き込み・実機未検証**: フェイルセーフ後の ARM の立ち上がり要求 (`rearm_required`)、MD 異常のデバウンス、自動停止を止まるまで保持、brake/torque_mode 中の目標車速の追従、I2C BUSY 時の IMU 読み出しの見送り、超音波の途絶の無効化、ウォッチドッグ復帰時の演出の省略。ワイヤ形式は変えていないので `protocol_version` は 0x0017 のまま。
- **作業ツリーにあって未書き込み (制御定数の根拠の整理、ワイヤ形式は不変)**: 自動停止の減速度を手置きの 3.0 から同定した制動の減速度 (`VEHICLE_BRAKE_DECEL_M_S2` 2.47) へ (停止距離が 2m/s で約15cm 伸びる＝早めに止まる)、自動停止の遅延・センサ取付オフセット・矩形の幅・DISARM 後の制動の上限を車体寸法と同定値の式に、各輪の周速のローパスを係数ではなく時定数 (`DRIVE_LPF_TAU_*`) で持ち実際の経過時間から係数を求める形に、加速度FFの時定数を上位の指令周期 (`DRIVE_HOST_COMMAND_PERIOD_S`) に。写しの値 (車体寸法・同定値) は上位の `tools/ctrl_tune/tests/test_fw_constants.py` が `config/vehicle.toml` と突き合わせる。
- 実機確認済み: LiDAR 給電の ARM 連動、後輪の位置保持。

## 走行制御・安全の要点

- 起動直後と **Pi 未接続/DISARM の間は `DRIVE_POWER`・`LIDAR_POWER` とも OFF が既定**。ボタン1を押しながら起動したときだけステア原点較正のため `Setup()` で一時的に駆動電源を入れ、較正後に切る。ステア中心が未較正の間は駆動電源を入れず、`Steering_SetAngleRad` も何も送らない。
- 優先順位 (`src/vehicle/vehicle.c`): ①緊急停止ラッチ中 → ②COMMAND 生存中は上位指令 → ③それ以外はフェイルセーフ (`ApplyFailsafe`: 最大制動で停車保持。ホーン・パッシング・ウィンカーも解除)。**フェイルセーフを一度通ったら (緊急停止・COMMAND 途絶・起動直後)、上位が ARM を一度下ろして立て直すまで `armed` にしない** (`rearm_required`。ARM を立てたままの COMMAND で、解除・復帰した瞬間に走り出さないため。受け付けなかったら LOG で知らせる)。
- 安全層: ①`DRIVE_POWER` のハード遮断 (過電流でラッチ) → ②IWDG 500ms → ③ハートビート断 50ms で緊急停止 (ラッチ。**ハートビートが戻った状態でボタン2を押すまで解除しない**。駆動電源は切らない: 切ると MD が制動できず惰行するため。ただし停車して `VEHICLE_FAILSAFE_POWER_OFF_S` (30秒) 経ったら過熱・電池消耗の防止に切る。ARM で戻る) → ④COMMAND 途絶 100ms で自動ブレーキ。ハートビートを一度も検出していない間は ③が働かないので、`TELEMETRY.flags` の `NO_HEARTBEAT` (bit25) を立てる (上位は ARM を通さない)。
- 「止める」: `Coast()` は送信停止ではなくトルク0を送り続ける (`BldcMotor_Stop` を止める手段に使わない)。走行中に ARM が外れたら、停車 (`DRIVE_STANDSTILL_SPEED_M_S`) まで駆動電源を切らず最大制動 (`UpdateDisarmBraking`、上限 `VEHICLE_DISARM_BRAKE_TIMEOUT_S`)。3台の MD の健全性を毎周期監視し (`UpdateMdHealth`)、揃うまで `armed` にしない・走行中に `VEHICLE_MD_FAULT_DEBOUNCE_S` 続けて崩れたら両後輪を最大制動して `md_fault` をラッチ。
- ブレーキホールド: 制動して |車速| が `VEHICLE_BRAKE_HOLD_SPEED_M_S` 未満の状態が `VEHICLE_BRAKE_HOLD_DELAY_S` 続いたら後輪の位置保持へ (`UpdateBrakeHold`)。サイドブレーキ (`flags2` bit0) は速度に関わらず即座に位置保持し、brake・torque_mode・速度より優先。
- 駆動 (`src/control/drive.c`): 車速 PI (+加速度FF) → 左右配分 (等配分 + TV の差) → スリップ制限 (TC/ABS/片輪浮き対策) → MD へトルク指令。brake・torque_mode の間は目標車速を実車速に合わせ続け、車速指令へ戻ったら今の車速からランプで詰める。車速は前輪エンコーダの走行距離 (非駆動輪) と加速度計のカルマンフィルタ (`speed_kf.c`)。brake は車速PIを迂回して MD の制動モードへトルクを直接渡す。torque_mode も PI を迂回するが TC/TV は掛かる。brake と torque_mode が同時なら brake が勝つ。ABS は制動モードのみ、TC は負トルク (PI の減速) にも掛かる。詳細・定数は `drive.h` のコメント。
- 調整パラメータは `src/control/control_params.h` の `CONTROL_PARAM_TABLE` が唯一の定義 (名前・`param_id`・既定値・範囲)。Flash には保存せず、上位が接続のたびに `CONFIG_SET` で送る。
- 自動停止 (`COMMAND.flags` bit7): 動的停止距離 `d_stop = v・遅延 + v²/(2a) + margin` 以内、左右 ±15cm (半車幅+余裕。扇形だと遠方でコースの壁を拾う) の矩形に LiDAR 点が3点以上 (`Lidar_QueryCorridor`)、または超音波 (フォールバック/至近の独立トリガー) で作動。しきい値は車体先端基準 (取付オフセット加算)。静止時の前後判定は上位が指令した方向へフォールバック (デッドロック回避)。一度作動したら止まるまで解除しない (停止距離は車速で縮むので、判定だけだと解除と再作動を繰り返す)。緊急停止ラッチは LiDAR を使わない。ARM 中に LiDAR も前後の超音波も使えない状態が3秒続くと `AUTO_STOP_BLIND` (bit26)。超音波は生値の方が近ければ生値で判定 (LPF の遅れ対策)。判定は `AutoStopDistanceCm()` / `IsAutoStopObstacleAhead()`。
- 灯火: ウィンカー要求は `Vehicle` が `winker_request` として保持するだけで、`Lighting_SetWinker()` を呼ぶのは `Indicator_Update()` (`src/hmi/indicator.c`) のみ (フォールト時のハザードを優先するため調停を一箇所に集約)。ブレーキ灯は制動モードまたは実減速度で点灯、高速点滅 (ESS) は制動モードかつ急減速のとき (`ApplyBrakeLight()`)。
- 設定の適用: `steer_link_*` (ステアのリンクの換算) は `CONFIG_SET` で保持するだけで、`Vehicle` が停車して DISARM のときに適用 (`UpdateSteeringLinkage`。走行中に舵角の意味が変わらない)。ACK は保持値 (cubic が傾きの下限で引き上げられたら引き上げた値と `OUT_OF_RANGE`)。ほかの調整パラメータは即時反映。
- printf は `Setup()` の間だけブロッキング。`MainApp()` に入ったら `Log_Pump()` が空いたときに1バイトずつ送る (リングが満杯なら捨てる)。ジャイロのバイアス追従は前輪の車速が `DRIVE_STANDSTILL_SPEED_M_S` 未満のときだけ。IMU の初期化に失敗しても復旧の状態機械で一定間隔で再試行する。
- 上位の最大速度・最大舵角は固定定数 (`DRIVE_MAX_SPEED_M_S`、`Steering_GetMaxRoadWheelAngleRad()`)。最大加速度は上位が `COMMAND.accel_limit` で決める (0 なら `DRIVE_DEFAULT_ACCEL_M_S2`)。固定上限は `LIMITS` (0x0A) パケットで上位へ通知。

## ディレクトリ構成

```
Core/        STM32CubeMX 生成コード (編集は USER CODE ブロック内のみ)。main.h に GPIO 定義
Drivers/     HAL (編集しない)
src/         車両固有ロジック。依存は一方向: app → vehicle / hmi / comm → control / sensing / power / lighting → lib
  app/       コンポジションルート。インスタンスの宣言・初期化・毎周期の呼び出しと HAL 割り込みコールバックの振り分けだけを持つ (500us ループ)。制御ロジックは書かない
  vehicle/   上位指令・緊急停止・フェイルセーフの調停、舵角レート制限、ホーン・ブレーキ灯などの上位指令由来の灯火、自動停止
  hmi/       機体状態の表示 (電圧の呼吸 LED、フォールト/緊急停止のハザード)
  lighting/  前照灯・尾灯・ウィンカー/ハザード
  sensing/   計測のみ: Encoder (前輪アナログ絶対角)・Imu (MPU6050+AHRS、軸変換・Flash較正)・Lidar (LD06 を1度ビン×360点→30点セクタ)
  power/     電圧・電流・温度の計測、DRIVE_POWER/LIDAR_POWER の制御、異常監視 (PowerFault)
  control/   Motors (3 MD 通信と健全性)・Steering (原点較正・リンク換算)・Drive・TorqueVectoring・ControlParams
  comm/      RasLink (Pi との UART プロトコル)・Heartbeat (RAS_SIG 100Hz 矩形波監視)・Telemetry (各モジュールの値を TELEMETRY/STATS/LIDAR に詰める層)
host/        src/control のホスト向けビルド (`host/README.md`)。実機ファームには含まれない
lib/         特定の車両に依存しない汎用ライブラリ (多くはヘッダオンリー、Module_FunctionName 形式)
  adc_dma ahrs bldc_motor buzzer crc8 crc16 digitalinout filter flash ld06 mpu6050 mymath pid pwm_out serial timer ultrasonic watchdog
docs/        README.md を参照
```

- 新機能は担当モジュールを作り、app.c にはインスタンス・初期化・呼び出しだけを足す。各モジュールは「struct + `Module_Init()` で依存注入 + `Module_Update()`」形式でグローバル参照を避ける。
- メインループ順 (2kHz): `Motors_Receive → RasLink_Receive → Vehicle_Update → Drive_Update → Motors_Transmit → Telemetry → RasLink_Update` (MD の状態も上位の COMMAND も、指令計算の前に受信しないと1周期古いもので制御する。`RasLink_Update` は送信だけ)。
- `lib/crc8` は MD リポジトリ (`ProgramV4/lib/crc8/crc8.h`) とバイト単位で同一に保つ (多項式を変えると HD が落ちる)。`lib/crc16` は Pi 用 (CCITT-FALSE、LiDAR/テレメトリの長いフレームに必要)。
- `Serial_Write` は進行中の DMA を中断するので、Pi 向けは `Serial_WriteAsync` を使う。MD 向け 1回の送信 (6バイト=60us@1Mbps) は次の送信までに完了させる。
- Flash 書き込みは `lib/flash/flash.h` 経由 (Sector 7 ユーザー、Sector 6 MPU6050 較正)。
- `lib/watchdog` (IWDG) は一度起動すると止められないので、ブロッキング初期化が全部終わってから起動する。ウォッチドッグのリセットで起動し直したときは起動の演出を飛ばし、LOG で上位へ知らせる (`Setup()`)。
- 割り込みの中から HAL のブロッキング待ちに入らないこと。MPU6050 の DATA_RDY 割り込みは、I2C が BUSY なら読み出しを始めずに捨てる (`Mpu6050_OnDataReady`。`HAL_I2C_Mem_Read_IT` は BUSY を約17ms 空回りで待ち、5ms 周期の割り込みの中ではメインループが止まる)。
- `lib/ld06` の `LD06_Update()` は1パケットごとに返す。呼び出し側は false が返るまで繰り返す。
- LiDAR の左右鏡像補正は STM32 側では行わない (PlacePoint の角度を 360-angle にすると上位で通信エラーが増発する不具合が出た。原因未解明)。センサ基準の角度のまま送り、上位が `real_angle_deg = (360 - (sector_idx*30 + point_index)) % 360` で解釈する。1度ビンに複数点があるときはビン中心に最も近い点を採る。

## 通信

- **Pi**: USART1 1000000bps。フレームは SYNC(2)/TYPE/SEQ/LEN/PAYLOAD/CRC16。仕様は `src/comm/ras_link.h` (flags・param_id・型)、`ras_link.c` (LEN・レイアウト)、`docs/pi_uart_protocol_v0.19〜v0.23_delta.md`。ベース仕様と v0.5〜v0.18 の差分は `docs/archive/`。送信は優先度つきキュー (PONG > TELEMETRY > LIDAR > CONFIG_ACK 等 > LOG)。USART1 の NVIC と `USART1_IRQHandler` は RasLink が持つ (CubeMX で USART1 割り込みを有効にすると多重定義になる)。
- **RAS_SIG (PB12) と ECHO_FRONT (PA12) は同じ EXTI ライン12** のため外部割り込みが使えず、RAS_SIG はポーリング。
- **MD**: USART2 (ステア) / USART3 (左後輪) / UART4 (右後輪)、1000000bps。**ボーレートは MD 側と同時に変えること** (片側だけだと全フレームが化ける)。指令 6バイト (モード・指令値・トルク上限、CRC-8)、状態 11バイト (トルク上限をエコーバック)。CRC 不一致は破棄して前回値を保持。トルク上限 0 は「無制限」ではなく「動かない」。速度上限は持たない。指令は毎周期 (2kHz)、MD は状態を 500us ごとに返す。MD の通信監視 (`COMM_OK`、100ms 無応答) は Motors が持つ。

## ハードウェア (Core/Inc/main.h)

| 信号 | ピン | 用途 |
|---|---|---|
| I2C1_SCL/SDA | PB8/PB9 | MPU6050 (400kHz) |
| INT | PC5 | MPU6050 DATA_RDY (EXTI9_5) |
| CURRENT_P/S, VOLTAGE_S/P | PC0/PC1, PC2/PC3 | 電流・電圧 ADC |
| ENCODER_LEFT/RIGHT | PA5/PA4 | 前輪エンコーダ (アナログ絶対角, ADC2 DMA)。**ラベルは実配線と左右が逆**: PA5 が右前輪、PA4 が左前輪 (`src/sensing/encoder.h` の並びが正) |
| FRONT_LED/REAR_LED | PA6/PB0 | 前照灯・尾灯 |
| LW_LED/RW_LED | PA7/PB1 | 左右ウィンカー |
| TRIG/ECHO_FRONT | PA11/PA12 | 前方超音波 |
| TRIG/ECHO_REAR | PC8/PC9 | 後方超音波 |
| LIDAR_OUT / LIDAR_POWER | PA8 / PC4 | LD06 回転 PWM (TIM1 CH1) / 電源 |
| DRIVE_POWER | PB15 | MD 電源 |
| RAS_SIG | PB12 | Pi 生存監視 |
| BUTTON1/2 | PB14/PB13 | 操作ボタン |
| LED1–4 | PB4–PB7 | LED3/4 は PWM |
| BUZZER | PB10 | TIM2 CH3 |

- タイマ: TIM1 (LiDAR PWM。実周波数 15kHz で仕様の 30kHz ではない: `lidar.h`)、TIM2 32bit (ブザー)、TIM3 (灯火 PWM)、TIM4 (LED3/4)。`Buzzer_Init` に渡すクロック/プリスケーラは `Core/Src/tim.c` の設定と必ず一致させる。
- UART: USART1 Pi (1Mbps)、USART2/3・UART4 MD (1Mbps)、USART6 230400、UART5 250000。1Mbps 8N1 は 1バイト 10us。
- DMA1 の Stream0–7 は全て UART が使用済みのため、I2C に DMA は割り当てられない。MPU6050 は割り込み駆動 (`HAL_I2C_Mem_Read_IT`)。
- EXTI (優先度5): EXTI9_5 = INT(PC5) + ECHO_REAR(PC9)、EXTI15_10 = ECHO_FRONT(PA12)。`HAL_GPIO_EXTI_Callback` (app.c) で各モジュールへ振り分ける。PC5/PC9 はベクタ共有のため、IMU の DATA_RDY 停止は NVIC ではなく `EXTI->IMR` でマスクする。

## コーディング規約

- Google C++ Style Guide (C だが命名・構造を適用)。命名は `Module_FunctionName`。
- 公開 API のコメントは `.h`、実装の補足は `.c`。**コメントは「なぜ」が自明でない箇所だけ**。経緯・版数 (「v0.x で新設」「◯月に変更」) はコメントに書かない (git log と `docs/` が持つ)。
- `Core/` の HAL 生成コードは `USER CODE BEGIN/END` 内のみ編集。
- 変更点をプロトコルに影響させるときは `RAS_PROTOCOL_VERSION` を上げ、`docs/pi_uart_protocol_vX.YY_delta.md` を足し、この CLAUDE.md の「現在の状態」を更新する。
