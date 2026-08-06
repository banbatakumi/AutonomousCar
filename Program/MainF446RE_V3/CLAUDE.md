# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## プロジェクト概要

自律走行ミニカーの下位制御(低レイヤー)ファームウェア。STM32F446RE (Nucleo-F446RE) 上で動作する。

車両システム全体の構成:

- **Raspberry Pi (上位)**: 経路計画・画像認識など高レベルな自律走行判断を行い、シリアル経由で走行指令 (速度・舵角など) をこの STM32 に送る。
- **STM32 (このリポジトリ, 下位)**: Raspberry Pi からの指令を受け取り、モーター制御、トラクションコントロールなどのリアルタイム性が要求される低レイヤー制御、ライト (前照灯・尾灯・ウィンカー) 制御、ブザー、各種センサ (LiDAR・超音波・エンコーダ・電流電圧) の取得を担当する。

**重要: 本プロジェクトは `1からプログラムを書き直す` コミットにより全面書き直しの途中である。** `src/app/app.c` は現状ペリフェラルの初期化とごく簡単な動作確認 (LED・ライト点滅) のみを行うスケルトンで、走行制御・センサ処理・リモコン通信などの上位ロジックはまだ実装されていない。過去の設計 (Sensor/Remote/Drive/Imu/Mode などのモジュール分割) はこの書き直しで一旦解体されているため、コード内にそれらの名残を探しても見つからない。新規実装時はこの CLAUDE.md の「ディレクトリ構成」節の方針 (ハードウェア依存部分は `lib/`、車両固有ロジックは `src/`) に従うこと。

---

## ビルド・書き込みコマンド

```sh
make          # ビルド (build/MainF446RE_V3.elf/.hex/.bin を生成)
make clean    # build/ ディレクトリを削除
make run      # make -j12 でビルド後、SWDFlash.sh で ST-Link 経由書き込み
```

- `Makefile` は STM32CubeMX の projectgenerator が自動生成したもの。ソースファイルの追加時は `.ioc` を CubeMX で再生成するか、`C_SOURCES` / `C_INCLUDES` を手動編集する。
- `SWDFlash.sh` は `STM32_Programmer_CLI` (STM32CubeProgrammer 付属) を使い `build/MainF446RE_V3.bin` を `0x08000000` に書き込む。ST-Link 接続と `STM32_Programmer_CLI` の PATH 登録が前提。
- 単体テストの仕組みは現状ない。

---

## ディレクトリ構成と役割

```
Core/           STM32CubeMX 生成コード。main.h に GPIO ピン定義、Src/ に HAL 初期化・割り込みハンドラ
Drivers/        STM32F4xx HAL ドライバ (CubeMX 生成、編集しない)
src/app/        エントリポイント。Setup() でペリフェラル初期化、MainApp() がメインループ (app.h/app.c)
src/lighting/   前照灯・尾灯・ウィンカー/ハザードの制御 (Lighting_*)
src/sensing/    純粋な計測のみを行うセンサモジュール (Encoder_* : 車輪エンコーダの角度・角速度、
                Imu_* : MPU6050 + AHRS を束ねた姿勢 (yaw/pitch/roll) と加速度。取付方向の軸符号変換・
                Flash 永続キャリブレーション・静止時のジャイロバイアス追従もここ)
src/power/      電源の計測 (電圧・電流・温度) と電源スイッチ (DRIVE_POWER/LIDAR_POWER) の制御を担う (Power_*)。
                異常監視もここに実装し、結果は PowerFault のビットマスク (Power_GetFaults) で公開する。
                駆動系過電流は DRIVE_POWER を遮断、シグナル系過電流 (マイコン自身の電源なので切れない)
                は LIDAR_POWER と DRIVE_POWER を落として負荷を減らす。電圧低下は 8セルNiMH の放電終止
                電圧 (1.0V/cell = 8.0V) を下回った場合で、遮断はせずフォールトを立てるだけ。いずれも
                一定時間の継続で判定する (突入電流・電圧サグ対策)。電圧はモーター電流のリプルで
                閾値を跨いで上下し継続時間の計測がリセットされてしまうため LPF を通した値で判定し
                (Power_GetVoltage*Filtered)、電流は遮断が遅れないよう生値のまま使う。過電流は不可逆な処置を
                伴うためラッチしてリセットまで復帰しないが、電圧低下は警告だけなので電圧が戻れば
                自動でクリアされる (ヒステリシス付き、復帰は 1.1V/cell = 8.8V)。
                DRIVE_POWER は起動直後ではなく Setup() の最後 (起動演出の後) で投入する。
                フォールトの表示は app.c の UpdateFaultIndication() でハザード点滅として行う
src/control/    走行系の車両固有ロジック (Motors_* : 3モータ(ステアリング/左後輪/右後輪)のBLDC MD通信まとめ、
                Steering_* : ステアリング中心点キャリブレーションと相対角度指令、
                Drive_* : 車速のトルクベース閉ループ制御。後輪MDはトルク(Nm)モードで駆動し、
                車速PI → 左右等配分 → 各輪スリップ率によるTCリミッタ、という構造。車速の真値は
                非駆動輪である前輪エンコーダから取る。トルクベクタリング項の入口も用意済み)
lib/            特定の車両ロジックに依存しない汎用ライブラリ群 (単一責任、Module_FunctionName 形式)
  adc_dma/      ADC を DMA (Circular+ContinuousConvMode) で連続変換させ最新値を非ブロッキングで読む薄いラッパ
  ahrs/         6軸 (ジャイロ+加速度) Mahony 相補フィルタによる姿勢推定 (HAL 非依存)
  bldc_motor/   BLDC モータドライバ (MD) とのシリアル通信プロトコル実装 (指令送信・状態フレーム受信/パース)。
                送信のみ 1kHz (BLDC_MOTOR_TX_INTERVAL_US) に間引かれ、受信は呼ばれるたびに処理する
                (Serial のリングバッファ 64 バイトを溢れさせないため)。
                指令フレームは 6 バイト固定長で、モード・指令値に加えてトルク上限を毎回載せる
                (CANopen の RxPDO 相当。フレーム1つが常に完結した状態を表すので、取りこぼしや
                MD 単独のリセットがあっても次のフレームで復元され、設定値を別途同期する
                仕組みが要らない)。末尾は CRC-8/AUTOSAR で、フッタは置かない
                (固定値のフッタはペイロードの情報を含まないためデータ化けを検出できない)。
                トルク上限は uint8 で符号なし・切り捨て量子化・レンジ外飽和とし、初期値 0 は
                「無制限」ではなく「上限 0」= 動かない側に倒してある。
                速度上限は持たない。位置制御では速度指令が Kp × 位置偏差 で決まり舵角が
                ±60度に有界なので位置ゲイン自体がリミッタとして働き、後輪はトルクモードなので
                過速度の歯止めは Drive_Update の車速リミッタ側にある。
                MD からの状態フレーム (11バイト) も同じ方針で CRC-8 + フッタ無しとし、末尾に
                MD が適用中のトルク上限をエコーバックさせている。CRC 不一致のフレームは破棄して
                前回値を保持する (誤った角度・速度で制御するより保持する方が安全)。
                指令した制限値と一致しているかは BldcMotor_IsLimitSynced() で確認できる。
                Serial_Write は先頭で HAL_UART_AbortTransmit を呼ぶため、1回の送信
                (6バイト=240us) は必ず次の送信までに完了させること
  buzzer/       PWM ブザー制御 (パターン再生、起動メロディ)
  crc8/         CRC-8/AUTOSAR (poly=0x2F)。ヘッダオンリー。MD との通信で使用し、Raspberry Pi との
                通信プロトコルでも使う想定。**BLDC リポジトリ (ProgramV4/lib/crc8/crc8.h) と
                バイト単位で同一に保つこと** (diff で実装一致を検証できるようにするため)。
                多項式を変えると HD (ハミング距離) が落ちるので通信相手と揃えたまま触らない
  digitalinout/ GPIO 入出力の薄いラッパ (DigitalOut/DigitalIn)
  filter/       LPF (1次ローパス) / MAF (移動平均) フィルタ
  flash/        内部 Flash 読み書き (Sector 7 をユーザーデータ用、Sector 6 を MPU6050 キャリブレーション用に予約)
  ld06/         LD06 LiDAR パケットパース (Serial* 経由で受信、360度分の距離配列を保持)
  mpu6050/      MPU6050 の I2C ドライバ。DATA_RDY 割り込み起点の非同期読み出し (HAL_I2C_Mem_Read_IT) +
                ISR 側リングバッファ。姿勢計算は持たず生値→物理量までを担当
  mymath/       角度正規化・三角関数近似 (SinDeg/CosDeg/Atan2) など、HAL 非依存の数値ユーティリティ
  pid/          PID コントローラ (アンチワインドアップ付き)
  pwm_out/      TIM PWM 出力の薄いラッパ (duty 0.0–1.0 で指定)
  serial/       UART + DMA 受信によるリングバッファ通信ラッパ
  timer/        DWT サイクルカウンタベースのマイクロ秒精度タイマ
  ultrasonic/   HC-SR04 系超音波センサのトリガ送出・ECHOパルス幅からの距離計算 (ピン変化割り込み駆動)
```

`lib/` のほとんどのヘッダは `static inline` 実装のみでヘッダオンリー。今後トラクションコントロール・Raspberry Pi 通信プロトコルなどを実装する際は、ハードウェア依存部分 (ドライバ) は `lib/` に薄く切り出し、車両固有のロジック (制御アルゴリズム・状態遷移・通信プロトコルの解釈) は `src/` 側に置く。`src/sensing/` は計測専用、電源の計測+スイッチ制御のように読み書き両方を担うモジュールは `src/power/` のように役割ごとのディレクトリに分ける。

---

## ハードウェア構成 (Core/Inc/main.h)

| 信号名 | ポート/ピン | 用途 |
|--------|-----------|------|
| I2C1_SCL/SDA | PB8/PB9 | MPU6050 (400kHz) |
| INT | PC5 | MPU6050 DATA_RDY 割り込み (EXTI9_5) |
| CURRENT_P/S | PC0/PC1 | 電流センサ ADC (Primary/Secondary) |
| VOLTAGE_S/P | PC2/PC3 | バッテリー電圧 ADC |
| ENCODER_LEFT/RIGHT | PA5/PA4 | 車輪エンコーダ入力 |
| FRONT_LED / REAR_LED | PA6 / PB0 | 前照灯・尾灯 |
| LW_LED / RW_LED | PA7 / PB1 | 左右ウィンカー |
| TRIG_FRONT/ECHO_FRONT | PA11/PA12 | 前方超音波センサ |
| TRIG_REAR/ECHO_REAR | PC8/PC9 | 後方超音波センサ |
| LIDAR_OUT | PA8 | LD06 LiDAR モータ PWM 出力 |
| LIDAR_POWER | PC4 | LiDAR 電源スイッチ |
| DRIVE_POWER | PB15 | モータードライバ電源スイッチ |
| RAS_SIG | PB12 | Raspberry Pi との連携用信号線 |
| BUTTON1/2 | PB14/PB13 | 操作ボタン |
| LED1–4 | PB4–PB7 | ユーザー LED (LED3/4 は PWM) |
| BUZZER | PB10 | ブザー (TIM2 CH3) |

タイマ割り当て (`Core/Src/tim.c`):

- **TIM1**: Prescaler 0, Period 65535
- **TIM2**: Prescaler 0, Period 4294967295 (32bit) — ブザー PWM (CH3)
- **TIM3**: Prescaler 9, Period 899 — ライト系 PWM (前照灯/尾灯/左右ウィンカー)
- **TIM4**: Prescaler 9, Period 899 — LED3/LED4 PWM

UART は USART1/2/3/6, UART4/5 の 6 系統が CubeMX で設定済み (USART6 のみ 230400bps、他は 250000bps)。BLDC MD は USART2 (ステアリング) / USART3 (左後輪) / UART4 (右後輪) に割り当て済み。250000bps 8N1 は 1 バイト 40us なので、5 バイトのフレーム 1 つに 200us かかる (送信周期を決めるときはこれを基準にする)。`Buzzer_Init` に渡すクロック/プリスケーラ値は `Core/Src/tim.c` の `MX_TIMx_Init` の設定値と必ず一致させること (不一致は無音・音程ズレの原因になる)。

DMA の割り当て (`Core/Src/dma.c`): **DMA1 の Stream0–7 はすべて UART が使用済み**。STM32F446 の I2C1 は DMA1 (RX: Stream0/5、TX: Stream6/7) しか使えないため、UART の DMA を潰さない限り I2C に DMA は割り当てられない。そのため MPU6050 は割り込み駆動 I2C (`HAL_I2C_Mem_Read_IT`) で読んでいる。

EXTI (優先度 5、`.ioc` の NVIC 設定):

- **EXTI9_5**: INT (PC5, 立ち上がり = MPU6050 DATA_RDY) と ECHO_REAR (PC9, 両エッジ)
- **EXTI15_10**: ECHO_FRONT (PA12, 両エッジ)

いずれも `HAL_GPIO_EXTI_Callback` (`src/app/app.c`) でピン番号を見て各モジュールへ振り分ける。PC5 と PC9 はベクタを共有しているため、IMU 側で DATA_RDY を一時停止するときは NVIC ごと止めず `EXTI->IMR` で PC5 のラインだけをマスクしている (`src/sensing/imu.c` の `EnableDataReadyInterrupt`)。

---

## コーディング規約

- [Google C++ Style Guide](https://google.github.io/styleguide/cppguide.html) に従う (C プロジェクトだが命名・構造の規約を適用)
- 関数・型・マクロの命名: `Module_FunctionName` 形式 (例: `Buzzer_Init`, `PwmOut_Write`)
- 公開 API のコメントは `.h` に書き、実装詳細のコメントは `.c` に書く
- なぜ (Why) が自明でない箇所にのみコメントを書く。何をしているか (What) の説明は不要
- `Core/` 以下の HAL 生成コードは `/* USER CODE BEGIN/END */` ブロック内のみ編集する
- Flash 書き込みは `lib/flash/flash.h` 経由で行う (直接 HAL Flash API を呼ばない)
