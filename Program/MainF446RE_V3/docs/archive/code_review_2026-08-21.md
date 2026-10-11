# コードレビュー結果 (2026-08-21)

対象: `MainF446RE_V3` 全ソース (src/ 7 モジュール + lib/ 15 モジュール、約 7,600 行)
レビュー種別: 静的読解のみ (実機動作未確認)

## 対応状況 (2026-08-21 追記)

指摘した A/B/C 全20件を優先度順に修正済み (`make` でのビルド確認済み)。
唯一 **A-3 は根本対策 (Recover のステートマシン化) ではなく応急処置** (ハートビート基準時刻の
リセットで誤 estop だけ防ぐ) にとどめてある。IMU の I2C 復旧自体が ~190ms メインループを
ブロッキングする点は変わっていないため、COMMAND 受信の取りこぼしは依然として残る。
各項目の対応内容は見出し直下に追記した。**すべて実機での動作検証は未了。**

---

## 優先度A — 安全に直結する実バグ

### A-1. COMMAND 途絶/緊急停止からの復帰でレート制限が無効化される

**[対応済み]** `ApplyFailsafe()` でも `command_rate_timer` をリセットし、
`ApplyRasCommand()` 側にも `drive.c`/`pid.h` と同じ `dt_s` ガード
(`dt_s <= 0.0f || dt_s > 0.1f` なら 0 扱い) を追加した (vehicle.c)。

**場所**: [vehicle.c:60-61, 107-112](../src/vehicle/vehicle.c#L60-L61)

```c
float dt_s = Timer_Read(&obj->command_rate_timer);
Timer_Reset(&obj->command_rate_timer);
...
float speed_step = accel_limit * dt_s;
float steer_step = steer_rate_limit * dt_s;
```

`command_rate_timer` は `ApplyRasCommand()` の中でしかリセットされない。ところが
`Vehicle_Update()` は緊急停止ラッチ中と COMMAND 途絶中は `ApplyFailsafe()` へ分岐し、
タイマを回したまま戻ってこない。

**発生する事象**: 通信が 2 秒途絶して復帰した瞬間、`dt_s ≈ 2.0` となり
`speed_step = 3.0 × 2.0 = 6.0 m/s` — つまり**加速度制限が事実上取り払われ、
目標車速へ 1 周期でジャンプする**。舵角も同様で、`applied_steer_rad` は
フェイルセーフ中も保持されるため、復帰の瞬間に最大舵角まで一気に振れうる。
コメントが「急な指令変化でタイヤを滑らせたり据え切りでラックを痛めないため」と
書いている、まさにその事象が起きる。

さらに `Timer_Read()` は DWT ベースで **23.9 秒でラップ**するため、長時間の途絶では
`dt_s` が周期的に小さい値へ化けて挙動が非決定的になる。

**修正案**:
```c
// ApplyFailsafe() の中でもタイマをリセットする、かつ ApplyRasCommand 側でガードする
if (dt_s <= 0.0f || dt_s > 0.1f) dt_s = 0.0f;  // drive.c / pid.h と同じガード
```
`Drive_Update()` (drive.c:250) と `PID_Update()` (pid.h:51) は同じガードを持っている。
Vehicle だけが持っていない。

---

### A-2. 片輪浮き対策が「後退時に接地している側のトルクを絞る」

**[対応済み]** 修正案どおり、後輪左右速度の和の符号 (`dir`) で `WheelSpeedDiffAnomaly()` の
結果を正規化してから前進基準の符号判定を適用するよう変更した (drive.c)。

**場所**: [drive.c:288-294](../src/control/drive.c#L288-L294)

```c
float anomaly = WheelSpeedDiffAnomaly(obj);      // = rear_left - rear_right
float excess_left  = anomaly > 0.0f ? excess : -1.0f;
float excess_right = anomaly < 0.0f ? excess : -1.0f;
```

`anomaly > 0`（左が右より速い）を「左が浮いている」と解釈しているが、これは
**前進時にしか成立しない**。後退中は車輪速が負なので、浮いて空転している輪ほど
**より負に大きい**値になる。

**具体例**: 後退中に左輪が浮いて空転 →
`rear_left = -5.0`, `rear_right = -0.5` → `anomaly = -4.5 < 0` →
`excess_right` が設定され、**接地している右輪のトルクだけが絞られる**。
浮いている左輪は絞られないどころか、右が絞られたぶん相対的に強く駆動される。
片輪浮き対策が逆効果になる。

**修正案**: 進行方向で符号を正規化してから判定する。
```c
float dir = (obj->rear_speed_left_m_s + obj->rear_speed_right_m_s) >= 0.0f ? 1.0f : -1.0f;
float anomaly = WheelSpeedDiffAnomaly(obj) * dir;
```
（絶対車輪速の最終防波堤 `ApplyWheelLiftHardSpeedLimit()` は `Abs()` を使っているので
 後退でも正しく効く。壊れているのは左右差の経路だけ。）

---

### A-3. IMU の I2C 復旧処理がメインループを ~190ms 止め、緊急停止を誤発動させうる

**[部分対応]** 提案の「暫定的にHeartbeatの誤estop対策だけ先に入れる」方を採用した。
`Imu` に `recovery_ran` フラグを追加し (`Imu_ConsumeRecoveryRan()`)、`Recover()` が走った
周期に app.c の `UpdateSensors()` が `Heartbeat_ResetBaseline()` (新設) で基準時刻を
リセットすることで、復旧中のブロッキングを誤 estop の原因にしないようにした。
**Recover() 自体のブロッキング (~190ms) は解消していない** ため、その間の COMMAND
取りこぼし (2で指摘した約9倍のリングバッファ上書き) は依然として残る。根本対策
(ステートマシン化) は別途対応が必要。

**場所**: [imu.c:255-278](../src/sensing/imu.c#L255) (`Recover`) → [imu.c:68-110](../src/sensing/imu.c#L68) (`RecoverI2cBus`) → `Mpu6050_Init`

`Imu_Update()` は `MainApp()` の `UpdateSensors()` から毎周期呼ばれる。サンプルが
250ms 途絶すると `Recover()` へ入り、その中で以下のブロッキングが積み上がる:

| 処理 | 遅延 |
|---|---|
| `RecoverI2cBus`: `HAL_Delay(5)` + SCL 9 クロック (`HAL_Delay(1)`×18) + STOP (×3) + `HAL_Delay(10)` | 約 36ms |
| `Mpu6050_Init`: `HAL_Delay(100)` + `HAL_Delay(50)` (PLL ロック待ち) | 150ms |
| **合計** | **約 190ms** |

この間メインループが完全に停止する。結果:

1. **緊急停止の誤発動 (最も深刻)**: `HEARTBEAT_TIMEOUT_US = 50000` (50ms) に対し
   190ms 停止する。復帰直後の `Heartbeat_Update()` でピン電位が偶然変化していれば
   救われるが、100Hz 矩形波を 190ms 後にポーリングするので**約 50% の確率で
   同電位 → エッジ未検出 → `IsAlive()` が false → estop ラッチ**。
   ラッチは「ハートビート生存下でボタン2を押す」まで解除されないので、
   IMU の一時的な I2C 不調で走行不能になる。
2. **上位からの COMMAND 取りこぼし**: `RasLink_Update()` が 190ms 止まる間に
   250000bps で約 4.7kB 流入するのに対し受信リングは 512B。DMA が周回して
   **9 倍以上上書き**する。
3. IWDG (500ms 指定、LSI ばらつきで最短 340ms) には辛うじて間に合うが余裕は薄い。

**修正案**: `Recover()` をステートマシン化して 1 周期あたり 1 ステップだけ進める。
最小限の応急処置としては、`Recover()` の前後で `Heartbeat` の基準時刻をリセットして
誤 estop だけでも防ぐ (ただし通信取りこぼしは残る)。

---

## 優先度B — 機能が設計意図どおりに動いていない

### B-1. 超音波の LPF が実質的に無効化されている

**[対応済み]** 修正案どおり、`Ultrasonic` にシーケンス番号 (`seq`) を追加し
(`Ultrasonic_OnEchoEdge()` が新しい計測を確定するたびに++)、`RangeSensor` 側は
このシーケンスの変化を検出したときだけ `LPF_Update()` を呼ぶようにした。

**場所**: [range_sensor.c:7-21](../src/sensing/range_sensor.c#L7-L21)

`RangeSensor_Update()` はメインループ (2kHz) から毎周期呼ばれ、そのたびに
`LPF_Update(lpf, raw)` を実行する。ところが `Ultrasonic_GetDistanceCm()` が返す値は
**トリガ周期 60ms の間ずっと同じ**。つまり同一の raw 値を約 120 回通すことになり、
`0.6^120 ≈ 1e-27` — フィルタ出力は毎回 raw に完全収束する。

コメントは「トリガ間隔 60ms に対して…単発の反射角ノイズを抑えつつ」と書いているが、
**実際には一切平滑化されていない**。

これは v0.7 の自動停止（20cm 閾値・ヒステリシス無し）と直結する。単発の異常反射で
最大制動が入り、次の測定で解除される — というチャタリングが起きる素地がある。

**修正案**: 新しい測定値が到着したときだけフィルタを回す。
`Ultrasonic` 側に測定シーケンス番号（`OnEchoEdge` で ++ するカウンタ）を持たせ、
`UpdateOne()` で変化を検出したときのみ `LPF_Update()` を呼ぶ。

---

### B-2. `Imu_IsReady()` が IMU 停止後も最大 250ms のあいだ true を返す

**[対応済み]** 修正案どおり、`Imu_Update()` のタイムアウト判定側で `Recover()` の成否と
無関係に `IMU_SAMPLE_TIMEOUT_MS` 経過時点で即座に `data_valid = false` を立てるようにした。

**場所**: [imu.c:346-359](../src/sensing/imu.c#L346)

`data_valid` はサンプルが取れたとき true になるが、**サンプルが途絶しても
`Recover()` が呼ばれるまで false に戻らない**。その 250ms のあいだ:

- `Drive::EstimateYawRate()` が「実測値あり」と判断して**固まった `gyro_z` を使い続ける**
- `yaw_rate_measured = true` のままなので TV が動き続ける
- スリップ率の基準速度 (`reference_left/right`) も固まったヨーレートで計算される

**修正案**: `Imu_Update()` のタイムアウト判定側で、`IMU_SAMPLE_TIMEOUT_MS` 経過時に
`Recover()` の成否とは無関係に `data_valid = false` を立てる。

---

### B-3. `ApplyFailsafe()` が `torque_mode` を解除しない

**[対応済み]** 修正案どおり `ApplyFailsafe()` に `Drive_SetTorque(obj->drive, false, 0.0f);`
を追加した。

**場所**: [vehicle.c:165-174](../src/vehicle/vehicle.c#L165)

`ApplyFailsafe()` は `Drive_SetTargetSpeed(0)` と `Drive_SetBrake(true, MAX)` は呼ぶが
`Drive_SetTorque(obj->drive, false, 0)` を呼ばない。`Drive_Update()` 内で brake が
torque_mode より優先されるので**現状は表面化しない**が、Drive 側の優先順位を
1 行変えただけで「フェイルセーフ中に torque_mode の駆動トルクが復活する」バグに化ける。
安全層は他モジュールの内部優先順位に依存せず自己完結させるべき。

**修正案**: `ApplyFailsafe()` に `Drive_SetTorque(obj->drive, false, 0.0f);` を追加。

---

### B-4. `IsAutoStopObstacleAhead()` がレート制限前の生の指令値で方向を判断する

**[対応済み]** `Drive_GetVehicleSpeed()` (実車速) を方向判定に使うよう変更した。
`applied_speed_m_s` はブレーキ/torque_mode中に0固定されてしまい方向情報として使えない
ため、実車速の方を採用した。停車中に前方判定に倒れる挙動 (実害軽微) は変更していない。

**場所**: [vehicle.c:87-89](../src/vehicle/vehicle.c#L87)

```c
float intended_direction = torque_mode ? command->target_torque_nm : command->target_speed_m_s;
```

上位が「前進中に後退を指令」した瞬間、実車はまだ前進しているのに**後方の超音波**を
見に行く。前方に壁があっても自動停止が効かない窓ができる。
`obj->applied_speed_m_s`（レート制限後）か `Drive_GetVehicleSpeed()`（実速度）を
基準にする方が実態に合う。

また `target_speed == 0` のとき `>= 0` で前方センサを見るため、停車中に前方 20cm 以内へ
何かが来ると `auto_stop_active` が立ち続ける (実害はブレーキ灯とテレメトリのみ)。

---

### B-5. `Ultrasonic` の「無反射」が 651cm という有効値として報告される

**[対応済み]** 修正案どおり `ULTRASONIC_MAX_DISTANCE_CM` (400cm) を上限として設け、
超えたら `ULTRASONIC_NO_ECHO` を格納するようにした (ultrasonic.h)。

**場所**: [ultrasonic.h:51-62](../lib/ultrasonic/ultrasonic.h#L51)

HC-SR04 系は測定範囲外のとき ECHO を約 38ms 出す。立ち下がりは正常に検出されるので
`distance_cm = 38000 × 0.0343 / 2 ≈ 651cm` が有効値として格納され、
`ULTRASONIC_NO_ECHO` にはならない。上位には飽和値 (255 = 5.1m) が「実測」として届く。

**修正案**: 距離に上限 (例: 400cm) を設け、超えたら `ULTRASONIC_NO_ECHO` を入れる。

---

## 優先度C — 性能・保守性

### C-1. `Timer_Read/ReadMs/ReadUs` が毎回 `HAL_RCC_GetSysClockFreq()` を呼ぶ

**[対応済み]** `Micros_Init()` が求めた `g_cycles_per_us` を `Timer_GetCyclesPerUs()` で
公開し、`Timer_Read/ReadMs/ReadUs` はそちらを参照するよう変更した (timer.c/timer.h)。
`Timer_ReadMs/ReadUs` は整数除算のみになり float 演算も無くなった。

**場所**: [timer.h:20-30](../lib/timer/timer.h#L20-L30)

`HAL_RCC_GetSysClockFreq()` は RCC レジスタを読んで PLL の分周比から周波数を
**毎回計算し直す**（switch + 複数の除算を含む数百サイクル級の関数）。
これが 500us ループ中に何十回も呼ばれている。とくに:

```c
// app.c:254 — ビジーウェイトのループ条件で毎回呼ばれる
while (Timer_ReadUs(&control_interval_timer) < CONTROL_INTERVAL_US);
```

**修正案**: `Micros_Init()` で求めた `g_cycles_per_us` を公開して使うか、
`SystemCoreClock` を直接参照する。1 行の変更でループの空き時間が目に見えて増える。

### C-2. `LPF` が `double` で実装されている

**[対応済み]** `LPF` 構造体と `LPF_Init`/`LPF_Update` の型を `float` へ変更した (lpf.h)。
呼び出し側の double リテラル (`IMU_ACCEL_LPF_K` 等) にも `f` サフィックスを付けて揃えた。

**場所**: [lpf.h:4-20](../lib/filter/lpf.h#L4-L20)

Cortex-M4F の FPU は**単精度のみ**。`double` 演算はすべてソフトウェアエミュレーション
（1 演算あたり数十〜百サイクル）になる。`LPF_Update()` は毎周期 Drive で 4 回 +
Power で 4 回 + IMU で 3 回 + RangeSensor で 2 回 = 13 回呼ばれる。
`float` 化するだけで済む。

### C-3. `mymath.h` の角度定数が `double`

**[対応済み]** `PI`/`HALF_PI`/`TWO_PI`/`FOUR_PI`/`TWO_THIRDS_PI`/`DEG_TO_RAD`/`RAD_TO_DEG`
すべてに `f` サフィックスを付けた (mymath.h)。`sin_table[]` の初期化子 (SIN1〜SIN89) は
コンパイル時定数畳み込みのため実行時コストが無く、対象外とした。

`PI` / `TWO_PI` / `DEG_TO_RAD` はいずれも `f` サフィックスの無い double 定数。
`Radians(imu_data->gyro_z)` (telemetry.c:104,124,125 / drive.c:35) や
`voltage / ENCODER_VREF * TWO_PI` (encoder.c:8) が double 演算へ格上げされる。
定数に `f` を付けるだけで解消する。

### C-4. `Abs()` / `Constrain()` マクロの多重評価

**[対応済み]** 修正案どおり `static inline float` 関数へ置き換えた (mymath.h)。
プロジェクト内の全呼び出し箇所が float だったため float 固定で問題ない。

**場所**: [mymath.h:12-13](../lib/mymath/mymath.h#L12-L13)

`Constrain(amt, low, high)` は `amt` を最大 3 回評価する。実害が出ている箇所:

```c
// telemetry.c:132 — ADC 読み出し + 浮動小数演算が最大 3 回走る
telemetry.temp_c[3] = (uint8_t)Constrain(Power_GetTemperatureC(obj->power), 0.0f, 255.0f);
```
ADC は DMA で裏から更新され続けるため、3 回の呼び出しで**異なる値が返りうる**。
比較に使った値と代入される値が食い違う可能性がある（今回は温度表示だけなので実害は軽微だが、
このパターンを制御量へ適用したら本物のバグになる）。

`static inline` 関数へ置き換えるのが本筋。

### C-5. `Telemetry_Update()` が 2kHz で走るが送信は 50Hz

**[対応済み]** `Telemetry` に内部タイマを追加し、`Telemetry_Update()` 冒頭で
`RAS_TELEMETRY_INTERVAL_US` (50Hz) 未満なら即 return するようにした (telemetry.c/h)。
呼び出し側 (app.c) の契約は変えていない (毎周期呼んでよい)。

**場所**: [telemetry.c:96-156](../src/comm/telemetry.c#L96)

毎周期 `RasTelemetry`（約 120 バイト）をスタックに構築し、`RasLink_SetTelemetry()` で
まるごとコピーしている。実際に使われるのは 40 回に 1 回。MD 状態の集計・ADC 読み・
`BuildFlags()` の全モジュール問い合わせも同様。50Hz へ間引けば制御ループの余裕が増える。

### C-6. `Serial_Init()` の `malloc` に NULL チェックが無い

**[対応済み]** 修正案どおり `malloc` をやめ、呼び出し側が static バッファを渡す形
(`Serial_Init(&s, huart, buf, sizeof(buf))`) に変更した (serial.h)。app.c 側の
5箇所すべての呼び出しを static 配列渡しへ更新した。

**場所**: [serial.h:19-27](../lib/serial/serial.h#L19-L27)

```c
self->rxBuf = (uint8_t*)malloc(rxBufSize);
memset(self->rxBuf, 0, rxBufSize);   // NULL なら即 HardFault
```

現状は bss が 9KB / RAM 128KB で確保に失敗しないが、リンカスクリプトの
`_Min_Heap_Size` は 0x200 (512B) しか宣言されておらず、実際の要求は 960B。
`_sbrk` がスタック手前まで伸ばせるので通っているだけで、宣言と実態が食い違っている。

**修正案**: RAM は 119KB 余っているので、そもそも `malloc` をやめて呼び出し側が
static バッファを渡す形にする（`Serial_Init(&s, &huart1, buf, sizeof(buf))`）。
組み込みでヒープを使わない方が確実。

### C-7. `_Min_Stack_Size` が 1KB しかない

**[対応済み]** `_Min_Stack_Size` を 0x400 (1KB) から 0x1000 (4KB) へ拡大した
(STM32F446XX_FLASH.ld)。

**場所**: `STM32F446XX_FLASH.ld:59`

`ras_link.c` は 99 バイトのペイロード配列 + 262 バイトの `build_buf` 経由で
フレームを組む。割り込みハンドラも同じ MSP を共有する。RAM が 119KB 余っている
のだから 4KB 程度に増やしておく方が安全。

### C-8. `Serial_Read()` のオーバーラン検出が死んでいる

**[対応済み]** rxTop/rxBtm の差分 (mod演算) では周回数が原理的に判別できないため、
方針を変更し「バッファを満たすのに要する時間より長くポーリング間隔が空いたか」を
`Micros()` の実時間で判定する方式にした。`Serial_Available()` (制御周期ごとに必ず
呼ばれる) で判定・フラグ立てし、`Serial_Read()` がフラグを消費して読み出し位置を
最新へ読み捨てて追いつく (serial.h)。アイドル (無通信) 中に誤検知しないよう、
時刻更新はデータの有無に関わらず毎周期の `Serial_Available()` 呼び出しで行う。

**場所**: [serial.h:44-47](../lib/serial/serial.h#L44-L47)

```c
uint16_t available = (rxTop + self->rxBufSize - self->rxBtm) % self->rxBufSize;
if (available > self->rxBufSize - 1) { ... }   // 剰余の結果が size-1 を超えることはない
```

`% rxBufSize` の結果は最大 `rxBufSize - 1` なので、この条件は**決して真にならない**。
DMA が読み出し位置を追い越したことを検出する意図だったはずだが機能していない。
A-3 のような長時間ブロッキングが起きたとき、静かにデータが化ける。

### C-9. `Serial_WriteByte()` がスタック変数のアドレスを DMA へ渡す

**[対応済み]** 未使用を確認のうえ関数ごと削除した (serial.h)。

**場所**: [serial.h:54-56](../lib/serial/serial.h#L54-L56)

```c
static inline void Serial_WriteByte(Serial* self, uint8_t data) {
  HAL_UART_Transmit_DMA(self->huart, &data, 1);   // 関数を抜けた後も DMA が読む
}
```
現在このプロジェクトでは未使用だが、使った瞬間に化けたバイトが出る。削除するか
コメントで封印しておくべき。

### C-10. コメントと実装が食い違っている箇所

**[対応済み]** 3箇所とも記載どおりに修正した。app.c:252 は「アイドル待ち時間」である旨へ
コメントを訂正 (挙動自体は変更していない)、drive.h は「500us = 2kHz」へ訂正、
ras_link.h の `RasLink_HasCommand()` の doc は削除済みの `UpdateDriveTest` への言及を除き、
実際の用途 (TELEMETRY の UART_TIMEOUT フラグ判定) を記載した。

| 場所 | 内容 |
|---|---|
| [app.c:252](../src/app/app.c#L252) | 「LED2 の点灯幅がループ1周の処理時間になる」→ 実際は**アイドル待ち時間**が点灯幅。処理時間を測るなら点灯/消灯を逆にする |
| [drive.h:118-126](../src/control/drive.h#L118) | 「フィルタ係数は 1kHz 前提」「一定周期 (1kHz 想定) で呼ぶこと」→ 実際の `CONTROL_INTERVAL_US` は **500us = 2kHz**。係数から逆算した帯域は約 1.6Hz で結果的に妥当だが、記述は誤り |
| [ras_link.h:110](../src/comm/ras_link.h#L110) | `RasLink_HasCommand()` の doc に「偽の間は下位単独の動作 (走行テスト等) をしてよい」とあるが、`UpdateDriveTest` は削除済み |

### C-11. `RasLink_Log()` が定義されているが呼び出し元が無い

**[対応済み]** Power のフォールト (過電流/低電圧の4種) が新規に立った瞬間だけ
`RasLink_Log()` で通知する `LogNewFaults()` を telemetry.c に追加した
(過電流は RAS_LOG_ERROR、低電圧は RAS_LOG_WARN)。IMU復旧側は A-3 の対応で
別のフラグ消費経路 (ハートビート対策) に使ったため、ログ送信の対象からは外した。

LOG 用の送信キュー (512B) を確保しているが、`RasLink_Log()` はどこからも呼ばれていない。
デバッグ用に残すなら、少なくとも A-3 の IMU 復旧や Power のフォールト発生時に
`printf` の代わりに使うと上位側で原因が追える。

### C-12. 前後の超音波が同じタイミングでトリガされる

**[対応済み]** `Timer` に `Timer_RewindUs()` (開始時刻を過去へずらすことで経過時間を
即座に進める) を追加し、`RangeSensor_Init()` で後方センサの `trigger_timer` へ
半周期 (30ms) 分適用して位相をずらした (timer.h, range_sensor.c)。

`RangeSensor_Init()` が両センサの `trigger_timer` を同時に初期化するため、
前後が常に同時刻に発射される。前後向きなので直接の干渉は小さいが、壁際では
反射波が回り込むことがある。片方の初期位相を 30ms ずらしておくと確実。

---

## 補足: 良く出来ている点

- 安全層の 4 段構成と、それぞれの層に「なぜそうしたか」が書かれたコメント
- `Power_SetDrivePower()` が要求値とフォールト状態を分離し、Vehicle が毎周期
  `arm` を投げても過電流ラッチが再投入されない構造 (power.c:72-76)
- `LimitDiffTorque()` の「先に差を丸めてから配分する」設計 — 左右非対称な飽和を
  原理的に防いでいる (drive.c:170-183)
- CRC 不一致フレームを破棄して前回値を保持する方針の徹底 (bldc_motor.c / ras_link.c)
- `Encoder` の端数繰り越し付き整数累積 — float 累積の桁落ちを正しく回避している

---

## 推奨する着手順

1. **A-1** (vehicle.c に dt ガード + タイマリセット) — 数行、影響大
2. **A-2** (片輪浮きの後退時符号) — 数行、v0.9 の新機能なのでいま直すのが安い
3. **B-1** (超音波 LPF) — 自動停止の実機検証に入る前に必須
4. **A-3** (IMU 復旧の非ブロッキング化) — 設計変更が要るので工数は大きい。
   暫定的に Heartbeat の誤 estop 対策だけ先に入れる手もある
5. **C-1 / C-2 / C-3** (Timer と double) — 機械的な置換で制御ループの余裕が増える
6. 残り
