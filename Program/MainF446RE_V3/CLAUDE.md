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
src/sensing/    純粋な計測のみを行うセンサモジュール (Encoder_* : 車輪エンコーダの角度・角速度)
src/power/      電源の計測 (電圧・電流・温度) と電源スイッチ (DRIVE_POWER/LIDAR_POWER) の制御を担う (Power_*)。
                駆動電流が閾値を超えると DRIVE_POWER を自動遮断する過電流保護もここに実装
src/control/    走行系の車両固有ロジック (Motors_* : 3モータ(ステアリング/左後輪/右後輪)のBLDC MD通信まとめ、
                Steering_* : ステアリング中心点キャリブレーションと相対角度指令)
lib/            特定の車両ロジックに依存しない汎用ライブラリ群 (単一責任、Module_FunctionName 形式)
  adc_dma/      ADC を DMA (Circular+ContinuousConvMode) で連続変換させ最新値を非ブロッキングで読む薄いラッパ
  bldc_motor/   BLDC モータドライバ (MD) とのシリアル通信プロトコル実装 (指令送信・状態フレーム受信/パース)
  buzzer/       PWM ブザー制御 (パターン再生、起動メロディ)
  digitalinout/ GPIO 入出力の薄いラッパ (DigitalOut/DigitalIn)
  filter/       LPF (1次ローパス) / MAF (移動平均) フィルタ
  flash/        内部 Flash 読み書き (Sector 7 をユーザーデータ用、Sector 6 を MPU6050 キャリブレーション用に予約 — MPU6050 モジュール自体は書き直しで未移植)
  ld06/         LD06 LiDAR パケットパース (Serial* 経由で受信、360度分の距離配列を保持)
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

UART は USART1/2/3/6, UART4/5 の 6 系統が CubeMX で設定済み (USART3 のみ 230400bps、USART6 のみ 1Mbps、他は 250000bps)。現状 `src/app/app.c` はどの UART も未使用で、Raspberry Pi 通信・LiDAR・モータードライバ通信などへの割り当ては未実装。`Buzzer_Init` に渡すクロック/プリスケーラ値は `Core/Src/tim.c` の `MX_TIMx_Init` の設定値と必ず一致させること (不一致は無音・音程ズレの原因になる)。

---

## コーディング規約

- [Google C++ Style Guide](https://google.github.io/styleguide/cppguide.html) に従う (C プロジェクトだが命名・構造の規約を適用)
- 関数・型・マクロの命名: `Module_FunctionName` 形式 (例: `Buzzer_Init`, `PwmOut_Write`)
- 公開 API のコメントは `.h` に書き、実装詳細のコメントは `.c` に書く
- なぜ (Why) が自明でない箇所にのみコメントを書く。何をしているか (What) の説明は不要
- `Core/` 以下の HAL 生成コードは `/* USER CODE BEGIN/END */` ブロック内のみ編集する
- Flash 書き込みは `lib/flash/flash.h` 経由で行う (直接 HAL Flash API を呼ばない)
