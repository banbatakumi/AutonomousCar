# CLAUDE.md

## コーディング規約

- [Google C++ Style Guide](https://google.github.io/styleguide/cppguide.html) に従う（C プロジェクトだが命名・構造の規約を適用）
- 関数・型・マクロの命名: `Module_FunctionName` 形式（例: `Drive_Update`, `Imu_Init`）
- 公開 API のコメントは `.h` に書き、実装詳細のコメントは `.c` に書く
- なぜ (Why) が自明でない箇所にのみコメントを書く。何をしているか (What) の説明は不要

---

## プロジェクト概要

**ターゲット**: STM32F446RE (Nucleo-F446RE)
**ビルド**: Makefile ベース (`make` でビルド、`make flash` で書き込み)
**フレームワーク**: STM32 HAL

自律走行小型車のファームウェア。STM32CubeMX で生成した HAL 初期化コード (`Core/`) の上に、`src/` と `lib/` でアプリロジックを実装する構成。

---

## ディレクトリ構成

```
Core/           STM32CubeMX 生成コード (HAL 初期化・割り込みハンドラ)
  Inc/main.h    GPIO ピン定義 (BUTTON, LED, TRIG/ECHO, BUZZER, LIDAR_MOTOR など)
  Src/main.c    エントリポイント。Setup() → MainApp() を呼ぶ

src/            アプリケーション層
  app/          初期化 (app_init.c) とメインループ (app_loop.c) の分割
  drive/        モータ・ステアリング制御
  imu/          MPU6050 ラッパ (Imu_*)
  lidar/        LD06 ユーティリティ (LidarSector, 最近傍・最開放方向探索)
  algorithm/    自律走行アルゴリズム (Algorithm_Run / Algorithm_ForwardOnly)
  mode/         動作モード管理 (STANDBY / RUN / FORWARD_ONLY)
  lighting/     ブレーキ・ウィンカー・ヘッドライト制御

lib/            デバイスドライバ・汎用ライブラリ
  mpu6050/      MPU6050 ドライバ (Mahony AHRS, 非同期 I2C)
  ld06/         LD06 LiDAR ドライバ
  buzzer/       ブザー制御
  pid/          PID コントローラ
  filter/       LPF (lpf.h) / MAF (maf.h)
  serial/       UART ラッパ
  timer/        マイクロ秒タイマ
  pwm_out/      PWM 出力ラッパ
  digitalinout/ GPIO ラッパ
  ultrasonic/   超音波センサ (HC-SR04)
  flash/        内部 Flash 読み書き (キャリブレーション永続化)
  mymath/       数値ユーティリティ
```

---

## メインループ

制御周期: `CONTROL_INTERVAL_US = 1000000 / 10000 = 100 µs` (10 kHz)

```
MainApp()
  └─ while(1)
       GetSensors()          ← LiDAR / 超音波 / ADC / IMU 更新
       CheckBatteryVoltage() ← 低電圧時は Drive_Free()
       Drive_Update()        ← モータコントローラ通信 (100 Hz 送信)
       Buzzer_Update()
       Lighting_Update()
       Mode_Update()         ← button1/button2 でモード切替
       switch(Mode_Get())
         STANDBY        → Drive_Free()
         RUN            → Algorithm_Run()
         FORWARD_ONLY   → Algorithm_ForwardOnly()
       制御周期待ち (user_led2 = HIGH の間がアイドル)
```

---

## ハードウェア情報

### GPIO ピン定義 (Core/Inc/main.h)

| 信号名 | ポート/ピン | 用途 |
|--------|-----------|------|
| BUTTON1 | PC14 | モード切替ボタン 1 |
| BUTTON2 | PC13 | モード切替ボタン 2 / IMU キャリブレーション |
| LED1–4 | PB1/PB0/PA7/PA6 | ユーザ LED (LED3/4 は PWM) |
| BUZZER | PB3 | ブザー |
| LIDAR_MOTOR | PA11 | LD06 モータ PWM |
| VOLTAGE_P/S | PC2/PC3 | バッテリー電圧 ADC |
| TRIG1–5/ECHO1–5 | 複数ポート | 超音波センサ |

### 電圧監視

- `MIN_VOLTAGE = 8.0 V`, `MAX_VOLTAGE = 11.0 V`
- ADC → 電圧変換: `adc_value * (3.3 / 4095.0) * ((10.0 + 1.0) / 1.0)`
- 8.0 V 未満で `battery_error = true` → `Drive_Free()`
- 8.5 V 以上でヒステリシス解除

---

## センサ仕様

### MPU6050 (IMU)

- I2C アドレス: `0x68` (AD0 = 0)
- **チップ実装方向: Z 軸まわり 180° 回転** → `MPU6050_Mount = {-1, -1, +1}`
- アルゴリズム: Mahony AHRS
- 非同期 I2C (`HAL_I2C_Mem_Read_IT`) 使用 → `Imu_OnI2CRxComplete()` をコールバックから呼ぶこと
- キャリブレーション: 起動時 button2 押下で実行し Flash 保存。次回起動時は Flash から復元

### LD06 LiDAR

- UART 接続、`LD06_Update()` を毎ループ呼ぶ
- `distances_360[]` に 0–359° の距離データ [mm] が格納される (0 = 無効)
- 扇形統計: `Lidar_GetSector(lidar, center_deg, half_width_deg)`
- 最近傍障害物: `Lidar_FindNearestSector()`
- 最開放方向: `Lidar_FindClearestDirection()`

### 超音波センサ (HC-SR04)

- front / right / left / back の 4 個
- `Ultrasonic_Get()` の戻り値は距離 [mm]

---

## 駆動系

### Drive モジュール

- `Drive_Set(torque, rate, steer)`: トルク指令 (ランプアップ)
- `Drive_SetVelocity(velocity, accel, steer)`: 速度 PID 制御
- `Drive_Brake(decel, steer)`: ブレーキ
- `Drive_Free()`: フリー (送信停止)
- `steer`: -1.0 (右最大) 〜 +1.0 (左最大)
- `WHEEL_BASE = 0.220 m`, `TREAD_WIDTH = 0.143 m`
- IMU データは `Drive_SetImuData()` で渡す → 摩擦推定・加速度上限に使用

---

## 動作モード

| モード | トリガ | 動作 |
|--------|--------|------|
| STANDBY | デフォルト | Drive_Free(), ヘッドライト 1% |
| RUN | button 操作 | Algorithm_Run() (LiDAR + 超音波) |
| FORWARD_ONLY | button 操作 | Algorithm_ForwardOnly() (前進特化) |

---

## 編集時の注意

- `Core/` 以下の HAL 生成コードは `/* USER CODE BEGIN/END */` ブロック内のみ編集する
- `lib/` はデバイスに依存しない汎用コードを置く。HAL への依存は最小化する
- Flash 書き込みは `lib/flash/flash.h` 経由で行う（直接 HAL Flash API を呼ばない）
- 新しいモードを追加する場合は `mode.h` の `OperationMode` 列挙型と `app_loop.c` の `switch` 文を両方更新する
