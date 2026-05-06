# MPU6050 DMP ライブラリ使用ガイド

## 概要

このライブラリはInvensenseのMPU6050ジャイロ/加速度センサをSTM32F446REでDMP（デジタルモーションプロセッサ）モードで使用するためのドライバです。

## 機能

- MPU6050の初期化
- DMP設定による高精度な角度推定（Yaw、Pitch、Roll）
- 相補フィルタを用いた加速度計とジャイロの融合
- キャリブレーション機能（ジャイロオフセット、加速度計オフセット計算）
- センサデータの読み取り

## ハードウェア接続

- **I2C1**: PB8 (SCL)、PB9 (SDA)
- **MPU6050アドレス**: 0x68（AD0 = 0）

## ソフトウェア使用方法

### 1. 初期化

```c
#include "mpu6050_dmp.h"

MPU6050_Handle mpu6050;
MPU6050_Data mpu6050_data;

// app.cのSetup関数で初期化
if (MPU6050_Init(&mpu6050, &hi2c1, MPU6050_I2C_ADDR_DEFAULT)) {
    printf("MPU6050 initialized successfully\n");
} else {
    printf("Failed to initialize MPU6050\n");
}
```

### 2. キャリブレーション（ボタン2を押しながら起動）

```c
// Setup関数内：ボタン2が押されているかチェック
if (DigitalIn_Read(&button2) == 0) {
    printf("Button2 detected - Starting calibration\n");
    // 200サンプル取得（100Hzで約2秒間）
    if (MPU6050_Calibrate(&mpu6050, 200)) {
        printf("Calibration successful\n");
    }
}
```

### 3. センサデータの取得

```c
// GetSensors関数内：毎制御周期で更新
void GetSensors() {
    // ... 他のセンサ更新 ...
    
    // MPU6050データを取得
    MPU6050_Update(&mpu6050, &mpu6050_data);
    
    // 各データへのアクセス
    float yaw = mpu6050_data.yaw;      // ヨー角（度）
    float pitch = mpu6050_data.pitch;  // ピッチ角（度）
    float roll = mpu6050_data.roll;    // ロール角（度）
    
    float gyrox = mpu6050_data.gyrox;  // X軸ジャイロ（度/秒）
    float gyroy = mpu6050_data.gyroy;  // Y軸ジャイロ（度/秒）
    float gyroz = mpu6050_data.gyroz;  // Z軸ジャイロ（度/秒）
    
    float accelx = mpu6050_data.accelx; // X軸加速度（m/s^2）
    float accely = mpu6050_data.accely; // Y軸加速度（m/s^2）
    float accelz = mpu6050_data.accelz; // Z軸加速度（m/s^2）
    
    float temp = mpu6050_data.temp;     // 温度（℃）
}
```

## キャリブレーション手順

1. **準備**: デバイスをアルゴリズムが実行されない平らな場所に設置
2. **実行**: ボタン2を押しながら電源を入れる
3. **LED確認**: LED1とLED2が点灯 → キャリブレーション実行中
4. **完了**: LEDが消灯 → キャリブレーション完了

キャリブレーション中は以下を計算します：
- **ジャイロオフセット**: 静止時の読み値の平均
- **加速度計オフセット**: 静止時の読み値の平均（重力除去）

## パラメータ設定

`mpu6050_dmp.c`の以下のマクロで調整可能：

```c
#define MPU6050_DMP_SAMPLE_RATE    100   // サンプルレート（Hz）
#define MPU6050_I2C_TIMEOUT        1000  // I2Cタイムアウト（ms）
```

相補フィルタの重み係数（`mpu6050_dmp.c`内）:
```c
float alpha = 0.98f;  // 0.0～1.0：ジャイロへの信頼度（1に近いほどジャイロを優先）
```

## API リファレンス

### MPU6050_Init
```c
bool MPU6050_Init(MPU6050_Handle* handle, I2C_HandleTypeDef* i2c_handle, uint8_t i2c_addr);
```
MPU6050を初期化します。接続確認と基本設定を行います。

**戻り値**: 成功時true、失敗時false

### MPU6050_Calibrate
```c
bool MPU6050_Calibrate(MPU6050_Handle* handle, uint16_t sample_count);
```
MPU6050をキャリブレーションします。指定数のサンプルを平均化してオフセットを計算します。

**パラメータ**:
- `sample_count`: 取得するサンプル数（推奨: 100～500）

**戻り値**: 成功時true、失敗時false

### MPU6050_Update
```c
bool MPU6050_Update(MPU6050_Handle* handle, MPU6050_Data* data);
```
最新のセンサデータを取得し、角度を計算します。

**戻り値**: 成功時true、失敗時false

### MPU6050_GetEulerAngles
```c
void MPU6050_GetEulerAngles(MPU6050_Handle* handle, float* yaw, float* pitch, float* roll);
```
現在のオイラー角を取得します。

### MPU6050_IsConnected
```c
bool MPU6050_IsConnected(MPU6050_Handle* handle);
```
MPU6050がI2Cバスに接続されているか確認します。

**戻り値**: 接続時true、未接続時false

### MPU6050_Reset
```c
bool MPU6050_Reset(MPU6050_Handle* handle);
```
MPU6050をリセットします。

**戻り値**: 成功時true、失敗時false

## トラブルシューティング

### 問題: MPU6050が接続されていないと言われる

**原因**: I2Cの接続不良、またはアドレス設定エラー

**解決策**:
1. PB8(SCL)、PB9(SDA)の配線を確認
2. プルアップ抵抗（通常4.7kΩ）が接続されているか確認
3. i2c_addrが正しいか確認（デフォルト: 0x68）

### 問題: 角度がドリフトする

**原因**: キャリブレーションが不十分、またはセンサが動いている

**解決策**:
1. キャリブレーションを再実行
2. より多くのサンプル数でキャリブレーション（500～1000）
3. 相補フィルタの係数`alpha`を調整

### 問題: 加速度計の読み値がおかしい

**原因**: センサが傾いている、またはキャリブレーションエラー

**解決策**:
1. 水平な場所でキャリブレーション
2. センサの物理的な損傷がないか確認

## ファイル構成

- `lib/mpu6050_dmp/mpu6050_dmp.h`: ヘッダファイル
- `lib/mpu6050_dmp/mpu6050_dmp.c`: 実装ファイル
- `src/app/app.c`: 初期化とキャリブレーション処理
- `src/app/app.h`: インクルード設定

## 今後の拡張

- DMP ファームウェアのロード（より高度なモーション処理）
- ステップカウンター機能
- タップ検出機能
- より高度なポーズ推定アルゴリズム

## 参考資料

- InvensenseのMPU-6000/6050 Product Specification
- コンプリメンタリフィルタの理論
- 相補フィルタを使用した姿勢推定
