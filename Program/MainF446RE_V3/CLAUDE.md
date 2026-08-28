# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## プロジェクト概要

自律走行ミニカーの下位制御(低レイヤー)ファームウェア。STM32F446RE (Nucleo-F446RE) 上で動作する。

車両システム全体の構成:

- **Raspberry Pi (上位)**: 経路計画・画像認識など高レベルな自律走行判断を行い、シリアル経由で走行指令 (速度・舵角など) をこの STM32 に送る。
- **STM32 (このリポジトリ, 下位)**: Raspberry Pi からの指令を受け取り、モーター制御、トラクションコントロールなどのリアルタイム性が要求される低レイヤー制御、ライト (前照灯・尾灯・ウィンカー) 制御、ブザー、各種センサ (LiDAR・超音波・エンコーダ・電流電圧) の取得を担当する。

**重要: 本プロジェクトは `1からプログラムを書き直す` コミットによる全面書き直しの途中である。** 過去の設計 (Sensor/Remote/Drive/Imu/Mode などのモジュール分割) は一旦解体されているため、コード内にそれらの名残を探しても見つからない。新規実装時はこの CLAUDE.md の「ディレクトリ構成」節の方針 (ハードウェア依存部分は `lib/`、車両固有ロジックは `src/`) に従うこと。

現在の走行制御 (`src/vehicle/vehicle.c`) と起動シーケンス (`src/app/app.c`) の状態:

- Raspberry Pi から `COMMAND` が来ていない/途絶している間、および緊急停止中は `ApplyFailsafe()` (`src/vehicle/vehicle.c`) が最大制動トルクでブレーキを掛け続け、停車保持する (クラクション・パッシングも解除する)。ボタン1起点の単体走行テスト (`UpdateDriveTest`) は Pi 統合に伴い削除済み (Pi 未接続時でも駆動電源が入ってしまう fail-open だったため)。
- `DRIVE_POWER` は既定で OFF。ボタン1を押しながら起動してステアリング原点較正をする場合のみ、MD 通信が要るため `Setup()` 中に一時的に ON にして較正後 OFF に戻す (較正しない起動では一度も投入しない)。以降は `ApplyRasCommand()` (`src/vehicle/vehicle.c`) が上位の arm 要求 (`RAS_CMD_FLAG_ARM`) に従って ON/OFF する。つまり **Pi が未接続または DISARM の間は駆動電源が入らないのが既定**。
- LD06 (LiDAR) は Setup で給電・初期化され、120Hz でセクタを上位へ送っている。
- 安全層は 4段: ①`DRIVE_POWER` のハード遮断 (過電流でラッチ) → ②IWDG 500ms → ③ハートビート断 50ms で緊急停止 → ④`COMMAND` 途絶 100ms で自動ブレーキ。緊急停止はラッチし、**ハートビートが戻っている状態でボタン2を押すまで解除しない**。緊急停止で駆動電源を切らないのは、切るとMDが制動をかけられず惰行して停止距離が伸びるため。
- トルクベクタリング (`src/control/torque_vectoring.c`) は実装済みで既定は有効。ただしゲイン・安定係数・横加速度上限はいずれも机上値のままで、**実機での符号確認とチューニングが未了**。
- 前後超音波 (`RangeSensor`) を使った自動停止が v0.7 で追加された。上位が `COMMAND.flags` bit7 (`RAS_CMD_FLAG_AUTO_STOP`) を立てている間だけ有効になる。`brake` (bit1) が同時に立っていればそちらが優先。ラッチせず、しきい値を上回れば自動解除 (ヒステリシス無し)。判定ロジックは v0.12 で下記の通り全面刷新した。
- TC/TV の実行時 ON/OFF が v0.8 で追加された。`COMMAND.flags` は8bit全部埋まっているため `CONFIG_SET`/`CONFIG_GET` (`param_id = 0x0010` = TC, `0x0020` = TV) 経由。`RasConfig.tc_enabled`/`tv_enabled` (既定 true) を `ApplyRasCommand()` が毎周期 `Drive_SetTractionControlEnabled()`/`Drive_SetTorqueVectoringEnabled()` へ橋渡しする。**実機での動作検証は未了**。
- 片輪浮き対策 (Wheel Lift Guard) が v0.9 で追加された (`src/control/drive.c`)。既存TC (前輪基準のスリップ率) は基準速度が `DRIVE_TC_MIN_SPEED_M_S` (0.25 m/s) 未満だと無効化されるため、停止/低速からの片輪浮き急発進を捉えられない。この機構は前輪基準速度を使わず「後輪左右の速度差 (ヨーレートで期待される差を差し引いた異常成分)」で判定するため低速域でも機能する。速い方 (浮いていると推定される輪) だけトルク上限を絞り、加えて後輪周速の絶対上限による最終防波堤を持つ。TC本体とは独立したリミッタ状態を持ち、両者の小さい方を実効上限として使う。上位からの ON/OFF は TC本体と独立に `CONFIG_SET`/`CONFIG_GET` (`param_id = 0x0050`) 経由、`RasConfig.wheel_lift_guard_enabled` (既定 true) を `ApplyRasCommand()` が毎周期 `Drive_SetWheelLiftGuardEnabled()` へ橋渡しする。しきい値 (`DRIVE_WHEEL_LIFT_DIFF_THRESHOLD_M_S`, `DRIVE_WHEEL_LIFT_MAX_WHEEL_SPEED_M_S`) は実測前の机上値で、**実機での動作検証・しきい値のチューニングは未了**。
- 最大速度・最大加速度・最大舵角の上位からの実行時変更 (`RAS_PARAM_MAX_SPEED`/`MAX_ACCEL`/`MAX_STEER`, `param_id = 0x0001`〜`0x0003`) は v0.10 で廃止した。上位から変更する実用上の必要が無いため、`DRIVE_MAX_SPEED_M_S`/`DRIVE_MAX_ACCEL_M_S2` (`src/control/drive.h`) と `Steering_GetMaxRoadWheelAngleRad()` の固定値に一本化した。目標車速の加速度レート制限もこれに伴い `src/vehicle/vehicle.c` から `src/control/drive.c` (`Drive_SetTargetSpeed()` / `Drive_Update()`) へ移した。`COMMAND.accel_limit_m_s2`/`steer_rate_limit_rad_s` (毎指令ごとにこの上限より緩いレートを指定できるフィールド) はそのまま残っている。舵角のレート制限 (`steer_rate_limit_rad_s` 由来) は引き続き `vehicle.c` が持つ。
- v0.10 で上位から変更できなくなった固定上限値 (最大速度・最大加速度・最大トルク・最大舵角) を、上位が把握できるよう v0.11 で `LIMITS` (0x0A) パケットを新設した (`src/comm/ras_link.c` の `SendLimits()`)。`VERSION`/`VERSION_REQ` と同じパターンで、起動直後に自動送信 (`VERSION` と同じバーストに相乗り) されるほか `LIMITS_REQ` (0x15) でいつでも再取得できる、読み取り専用のパケット。ラジコンモード・自律走行モードそれぞれの上限設定を上位側で持つ場合、この値を超えないようにする用途を想定。**実機での動作検証は未了**。
- 自動停止 (`RAS_CMD_FLAG_AUTO_STOP`) の判定を v0.12 で全面刷新した。固定20cmの超音波単独判定から、車速 `v` (実車速の絶対値) から物理的に導く動的停止距離 `d_stop = v・VEHICLE_AUTO_STOP_DELAY_S + v²/(2・DRIVE_MAX_ACCEL_M_S2) + margin` へ変更 (`AutoStopDistanceCm()`, `src/vehicle/vehicle.c`)。`VEHICLE_AUTO_STOP_DELAY_S` (遅延見積り) と `DRIVE_MAX_ACCEL_M_S2` (制動側への流用) はいずれも**未実測プレースホルダー**。`margin` (安全マージン) のみ上位が `CONFIG_SET`/`CONFIG_GET` (`param_id = 0x0060`) でcm単位の値を直接指定できる (段階的なレベルではなく連続値、範囲 `RAS_AUTO_STOP_MARGIN_MIN_CM`〜`RAS_AUTO_STOP_MARGIN_MAX_CM` (0〜100cm)、既定 `RAS_AUTO_STOP_MARGIN_DEFAULT_CM` (15cm)。`RasConfig.auto_stop_margin_cm` を `AutoStopDistanceCm()` がそのまま使う)。判定センサはLiDARを主、超音波を補助とする構成に変更: 前方0度・後方180度±20度 (`VEHICLE_AUTO_STOP_LIDAR_HALF_WIDTH_DEG`) のセーフティゾーン内で `d_stop` 以内の点が3点以上 (`VEHICLE_AUTO_STOP_LIDAR_MIN_POINTS`) あれば確定させる空間デバウンス方式 (`Lidar_QueryRoi()`, `src/sensing/lidar.c`。`Lidar_TakeReadySector()` によるテレメトリ消費とは独立した永続配列 `point_distance_mm`/`sector_update_us` を新設)。超音波は「LiDARのセーフティゾーン窓内に直近300ms以内のデータが無いときのフォールバック」と「5cm未満 (`VEHICLE_AUTO_STOP_ULTRASONIC_NEAR_CM`) の独立トリガー」の2役 (`IsAutoStopObstacleAhead()`)。前後判定は実車速の符号を使うが、静止時 (`|speed| < VEHICLE_AUTO_STOP_DIRECTION_DEADBAND_M_S`) は実車速の符号が常に非負に評価され前方判定へ固定されてしまう (速度制御モードでは braking=true が target_speed を0にクランプし続けるため実車速も0のまま復帰できず、前方に障害物・後方が空いていても後退できないデッドロックになる) ため、その間は上位が指令した方向 (torque_mode なら target_torque、それ以外は target_speed の符号) にフォールバックする。しきい値判定は各センサの生値ではなく「車体先端(バンパー)から障害物まで」の距離基準で行う: LiDAR・前後超音波はいずれも車体先端より内側 (base_link=後輪車軸中心とした raspi 側 `vehicle.toml` の footprint・センサ座標から算出) に付いているため、しきい値側に取付オフセットを加算して補正する (`VEHICLE_AUTO_STOP_LIDAR_FRONT/REAR_OFFSET_CM`=23/14cm、`VEHICLE_AUTO_STOP_ULTRASONIC_FRONT/REAR_OFFSET_CM`=4/3cm)。**実機での動作検証は未了**。プロトコルは `docs/pi_uart_protocol_v0.12_delta.md` 参照 (`protocol_version` 0x000B→0x000C)。
- **未実装**: TC/TV/速度PI/片輪浮き対策の各ゲイン自体の実行時変更 (`param_id` 0x0011/0x0021/0x0030/0x0031/0x0051 は `RAS_CONFIG_UNKNOWN_PARAM` を返す)。緊急停止 (ハートビート起点の `estop_latched` ラッチ) は引き続きLiDARを使わない (LiDARを使うのは自動停止のみ)。
- TCが実機比較でフル加速性能を落とすことが判明したため (TC無12.2km/h→TC有10.7km/h、静止から2m地点)、v0.13でTCのスリップ判定を修正した。原因はスリップ判定に使う基準速度 (前輪LPF、`DRIVE_LPF_K_FRONT=0.995`、τ≈100ms) が駆動輪側のLPF (`DRIVE_LPF_K_REAR=0.90`、τ≈4.75ms) より大幅に遅く、フル加速のようなランプ入力で `τ×加速度` ぶん (最大加速度時で約0.3m/s) 基準速度が系統的に低く出て、実際には空転していないのに「常時空転」と誤検出していたこと。対策として `EstimateVehicleSpeedForSlip()` (`src/control/drive.c`) を新設し、スリップ判定専用に軽いLPF `DRIVE_LPF_K_FRONT_TC=0.98` (τ≈25ms) を使うよう分離、加えて `UpdateTractionLimit()` にしきい値超過が `DRIVE_TC_SLIP_DEBOUNCE_S=0.03s` 継続するまでカットを開始しないデバウンスを追加した。PID/上位報告用の `vehicle_speed_m_s` (重いフィルタのまま) には影響しない。**実機での効果検証は未了**。
- TCのゲイン (`DRIVE_TC_CUT_GAIN`/`DRIVE_TC_RECOVER_RATE`) チューニング用に、v0.13で `TELEMETRY` へ `slip[2]` (RL/RR、無次元)・`tc_limit_nm[2]` (RL/RR、TCが動的に決めているトルク上限) を追加した (`Drive_GetTcLimitLeft/Right()` を新設、`src/comm/telemetry.c`)。`TELEMETRY` の LEN は66→74。プロトコルは `docs/pi_uart_protocol_v0.13_delta.md` 参照 (`protocol_version` 0x000C→0x000D)。
- サイドブレーキ (後輪の位置制御によるパーキングロック) がv0.13で追加された。`COMMAND.flags` は8bit全て使用済みのため、新設した `flags2` (`param_id`と違いCONFIG系ではなくCOMMANDの拡張バイト。brakeと同じ「上位が送り続けている間だけ有効」な継続コマンドの性質のため) のbit0 (`RAS_CMD_FLAG2_SIDE_BRAKE`) で伝える。有効化された瞬間の後輪モータ機械角度を `BldcMotor_GetMechAngle()` でラッチし、`BldcMotor_SetPosition()` で位置制御へ切り替えて機械的に固定する (`Drive_SetSideBrake()`、`src/control/drive.c`)。後輪はダイレクトドライブのためモータ機械角=車輪角そのもので、かつ「読み取った角度をそのまま送り返すだけ」なので左右の符号反転・0/2π境界の正規化のいずれも不要。速度・通常ブレーキ・torque_modeより優先し、**速度に関わらず即座に切り替わる** (停止を待たない、ユーザーの明示判断)。MDの状態フレームが無効 (給電直後など) でラッチ角度が取れない間は幽霊値 (角度0) を避けるため通常の最大トルク制動へフォールバックし、実際に位置保持へ入っているかは `TELEMETRY.flags` の `RAS_FLAG_SIDE_BRAKE_ACTIVE` (bit17) で区別できる。`COMMAND` の LEN は14→15。プロトコルは `docs/pi_uart_protocol_v0.13_delta.md` 参照。**実機での動作検証は未了**。

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
src/app/        エントリポイント兼コンポジションルート。全モジュールのインスタンスを static で所有し、
                Setup() が初期化順序を、MainApp() がメインループ (500us) の呼び出し順を組み立てる。
                HAL の割り込みコールバック (EXTI/I2C) の各モジュールへの振り分けもここ。
                **制御ロジックそのものは書かない** (書きたくなったら該当する src/ のモジュールへ)
src/vehicle/    車両統括 (Vehicle_*)。上位の指令・緊急停止・フェイルセーフのどれを車両へ適用するかを
                毎周期決める層で、優先順位は ①緊急停止ラッチ中 → ②COMMAND 生存中は上位指令 →
                ③それ以外はフェイルセーフ。上位の目標舵角を steer_rate_limit でレート制限
                してから Steering へ渡すのもここ (急な指令変化で据え切りでラックを痛めない
                ため)。目標車速側の加速度レート制限 (accel_limit) は Drive 側
                (`Drive_SetTargetSpeed`) が持つ。ハートビート断による緊急停止の
                ラッチ・解除、クラクション、上位指令由来の灯火 (ブレーキ灯・前照灯・パッシング)
                もこの層が持つ。個々のアクチュエータ制御には踏み込まない
src/hmi/        機体の状態を人間へ見せる表示 (Indicator_*)。電源電圧を LED3/LED4 の「呼吸」周期に
                マップして脈打たせる表示 (脈が止まれば制御ループが止まったことも分かる) と、
                フォールト・緊急停止のハザード点滅。上位指令で点く灯火は src/vehicle/ の担当で、
                こちらは機体状態から導かれる表示だけを扱う。ハザードはウィンカーと灯火を
                共有するので、方向指示を実装したらここで優先度を調停すること
src/lighting/   前照灯・尾灯・ウィンカー/ハザードの制御 (Lighting_*)
src/sensing/    純粋な計測のみを行うセンサモジュール (Encoder_* : 車輪エンコーダの角度・角速度、
                Imu_* : MPU6050 + AHRS を束ねた姿勢 (yaw/pitch/roll) と加速度。取付方向の軸符号変換・
                Flash 永続キャリブレーション・静止時のジャイロバイアス追従もここ、
                Lidar_* : LD06 の生パケットを1度ビン×360点へ整形し、30点(=30度)ずつの
                セクタとして切り出す。各点のタイムスタンプ付与もここ。センサの0度が機体前方を
                向くよう取り付けてあるが、センサを裏向き (PCB面が下) に取り付けているため
                角度の増加方向が上から見て機体と逆周りになっており、実機確認の結果左右が
                鏡像になっている (前後は反転軸上のため影響を受けない)。この鏡像補正は
                あえてこのモジュールでは行わない。PlacePoint に渡す角度を 360-angle に
                置き換えて試したところ、計算コスト自体は無視できる差のはずが上位 (Raspberry Pi)
                側で通信エラーが増発する原因不明の不具合が再現したため (メカニズム未解明)。
                0度・180度がちょうどセクタ境界に乗り正確な補正には隣接2セクタをまたいだ
                バッファの組み替えが要ることもあり、実装が確実に安定している「センサ基準の
                角度をそのまま送る」側を維持し、左右の解釈は上位 (Raspberry Pi) 側で
                real_angle_deg = (360 - (sector_idx*30 + point_index)) % 360 として行う。
                1度ビンに複数点が入ったときは**ビン中心に最も近い点**を採る (最短距離を採ると
                地図生成用のデータが近距離側へ系統的に歪みスキャンマッチングの精度が落ちる)。
                回転モータPWM (PA8/TIM1 CH1) の周期を 30kHz へ張り替えるのもこのモジュール
                (CubeMX 既定の Period 65535 では 2.7kHz になり LD06 の仕様と合わない))
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
                車速PI → 左右配分 (等配分 + TVのトルク差) → 各輪スリップ率によるTCリミッタ、
                という構造。車速の真値は非駆動輪である前輪エンコーダから取る。
                ブレーキ (Drive_SetBrake) は車速PIを迂回して MD の制動モードへ指定トルクを
                直接渡す。目標車速0でPIに任せると制動力がゲイン任せになり、上位が N・m で
                指定した強さどおりに効かないため。上位への報告 (torque_left/right_nm) は
                正=駆動・負=制動で揃えてあり、制動中は制動トルクを負値で入れる。
                torque_mode (Drive_SetTorque, v0.6) も同じ理由で車速PIを迂回し、上位が
                指定した駆動トルクを左右等配分の総駆動トルクとして直接使うが、こちらは
                TC/TVは掛けたままにする (空転抑制のため、通常駆動時と同じ配分経路を通す)。
                brake と torque_mode が同時に指定されたら Drive_Update 内の優先順位で
                brake が勝つ、
                TorqueVectoring_* : 直接ヨーモーメント制御 (DYC)。規範モデル (自転車モデル +
                安定係数 + 横加速度の頭打ち) が出す目標ヨーレートと IMU の実測値の偏差を
                PI で埋め、左右後輪のトルク差として Drive へ返す。左右の総和は変えないので
                車速制御とは干渉しない。IMU が使えないとき Drive 側のヨーレートは舵角からの
                幾何計算に化けて規範モデルとほぼ同じ式になるため、その間 Drive は TV を
                呼ばない (偏差が常に0付近になり制御が成立しないため)。TC が削っている最中は
                トルク差を付ける余力が無いので、Drive の LimitDiffTorque() で丸めてから
                TorqueVectoring_ReportApplied() に返し、出せなかった分の積分を巻き戻す)
src/comm/       Raspberry Pi (上位) との UART プロトコル (RasLink_*)。USART1、250000bps。
                仕様は docs/pi_uart_protocol_v0.4_request.md と、変更点だけを書いた
                docs/pi_uart_protocol_v0.5_delta.md 〜 docs/pi_uart_protocol_v0.11_delta.md。
                フレーミング (SYNC/TYPE/SEQ/LEN/CRC16) と
                パケットの解釈・組み立てだけを担い、走行制御には関与しない。受信した COMMAND は
                RasLink_GetCommand()、送るテレメトリは RasLink_SetTelemetry() に物理量のまま渡す
                (量子化はモジュール側)。送信は優先度つきキュー (PONG > TELEMETRY > LIDAR > CONFIG_ACK等 > LOG)。
                Serial_Write は進行中の DMA を中断するため使わず Serial_WriteAsync でフレーム単位に送る。
                USART1 の NVIC はこのモジュールが有効化し、USART1_IRQHandler もここで定義している
                (CubeMX は生成していない。.ioc で USART1 割り込みを有効にすると多重定義になる)。
                Heartbeat_* : Raspberry Pi の生存を RAS_SIG (PB12) の 100Hz 矩形波で監視する。
                レベルではなく矩形波なのは、断線したときプルアップで正常側に戻ってしまい
                フェイルセーフにならないため。**RAS_SIG(PB12) と ECHO_FRONT(PA12) は同じ
                EXTI ライン12を使うため外部割り込みは使えず、ポーリングで検出している**。
                Telemetry_* : 各モジュールの観測量を TELEMETRY / STATS / LIDAR_SECTOR の
                フィールドへ詰め替える層。RasLink がフレーミングと量子化を担うのに対し、
                こちらは「どのモジュールの値をどのフィールドへ入れるか」だけを担い制御には
                関与しない。MD の状態フレームが 100ms 更新されないことを検出する通信監視
                (md_status の COMM_OK ビット) もここにある。MD が無言になっても保持値は
                最後の正常値のまま固まるため、これが無いと上位が古い値を現在値だと信じ続ける
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
  crc8/         CRC-8/AUTOSAR (poly=0x2F)。ヘッダオンリー。MD との通信で使用する。
                **BLDC リポジトリ (ProgramV4/lib/crc8/crc8.h) とバイト単位で同一に保つこと**
                (diff で実装一致を検証できるようにするため)。
                多項式を変えると HD (ハミング距離) が落ちるので通信相手と揃えたまま触らない
  crc16/        CRC-16/CCITT-FALSE (poly=0x1021, init=0xFFFF, 検査値 0x29B1)。ヘッダオンリー。
                Raspberry Pi との通信で使用。CRC-8 は 15バイト程度を超えると HD=2 に落ちるため、
                69〜99バイトある LiDAR/テレメトリのフレームには 16bit が要る
  digitalinout/ GPIO 入出力の薄いラッパ (DigitalOut/DigitalIn)
  filter/       LPF (1次ローパス) / MAF (移動平均) フィルタ
  flash/        内部 Flash 読み書き (Sector 7 をユーザーデータ用、Sector 6 を MPU6050 キャリブレーション用に予約)
  ld06/         LD06 LiDAR のパケットパース (47バイト/12点)。角度はセンサ基準のまま返し、
                取り付け方向の補正やビニングは行わない (それらは src/sensing/lidar の担当)。
                **LD06_Update() は1パケット処理するごとに返す。** points[] が次のパケットで
                上書きされるため、まとめて読み進めると溜まっていた分を取りこぼす。
                呼び出し側は false が返るまで繰り返すこと
  mpu6050/      MPU6050 の I2C ドライバ。DATA_RDY 割り込み起点の非同期読み出し (HAL_I2C_Mem_Read_IT) +
                ISR 側リングバッファ。姿勢計算は持たず生値→物理量までを担当
  mymath/       角度正規化・三角関数近似 (SinDeg/CosDeg/Atan2) など、HAL 非依存の数値ユーティリティ
  pid/          PID コントローラ (アンチワインドアップ付き)
  pwm_out/      TIM PWM 出力の薄いラッパ (duty 0.0–1.0 で指定)
  serial/       UART + DMA 受信によるリングバッファ通信ラッパ
  timer/        DWT サイクルカウンタベースのマイクロ秒精度タイマ (Timer_* は経過時間の計測用)。
                加えて Micros() が「起動からの単調な32bit us 時刻」を返す。DWT のカウンタは
                180MHz では約23.9秒でラップするため、Micros() は呼ばれるたびに経過サイクルを
                積算して 71.6分周期の時刻を作る (Setup の先頭で Micros_Init() を呼び、
                ラップ周期より短い間隔で呼び続けること)。上位との時刻同期の基準になる
  ultrasonic/   HC-SR04 系超音波センサのトリガ送出・ECHOパルス幅からの距離計算 (ピン変化割り込み駆動)
  watchdog/     独立ウォッチドッグ (IWDG) の薄いラッパ。ヘッダオンリー。HAL ではなくレジスタを
                直接叩いているのは、IWDG が .ioc で有効化されておらず HAL モジュールを足すと
                CubeMX の再生成と衝突するため。**一度起動すると停止できない**ので、
                ブロッキングする初期化がすべて終わってから起動すること。
                LSI は 17〜47kHz と幅があり、実タイムアウトは指定値の 0.68〜1.9 倍に振れる
```

`lib/` のほとんどのヘッダは `static inline` 実装のみでヘッダオンリー。今後トラクションコントロール・Raspberry Pi 通信プロトコルなどを実装する際は、ハードウェア依存部分 (ドライバ) は `lib/` に薄く切り出し、車両固有のロジック (制御アルゴリズム・状態遷移・通信プロトコルの解釈) は `src/` 側に置く。`src/sensing/` は計測専用、電源の計測+スイッチ制御のように読み書き両方を担うモジュールは `src/power/` のように役割ごとのディレクトリに分ける。

`src/` 内の依存の向きは `app → vehicle / hmi / comm → control / sensing / power / lighting → lib` の一方向で、逆向き・横向きの依存を作らないこと。各モジュールは「struct + `Module_Init()` で依存を注入 + `Module_Update()` を制御周期で呼ぶ」形に統一してある (グローバル変数を直接参照させない)。新しい機能を足すときは `src/app/app.c` に処理を書き足すのではなく、担当するモジュールを作って app.c には**インスタンスの宣言・初期化・毎周期の呼び出し**だけを増やす。app.c が持つのはこの3つと HAL 割り込みコールバックの振り分けだけで、制御ロジックは置かない。

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
