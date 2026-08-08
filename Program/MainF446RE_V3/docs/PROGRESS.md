# PROGRESS

## 2026-08-08: ステアリングストールによる駆動電圧崩壊

### 現象 (Pi側ログより)
mode=DISARM, target_steer=0 を送信中に発生。

| t[s]  | 駆動V | ST電流 | 舵角rad  | 指令rad | フラグ |
|-------|-------|--------|----------|---------|--------|
| 111.0 | 9.60  | +0.00  | -0.3782  | +0.0000 | 正常   |
| 111.9 | 2.80  | +2.83  | -0.3766  | +0.0000 | 電流発生→電圧崩壊 |
| 112.9 | 1.80  | +2.83  | -0.3766  | +0.0000 | 駆動低電圧 |
| 115.0 | 1.60  | +2.83  | -0.3766  | +0.0000 | 駆動低電圧(以降張り付き) |

- 実舵角が指令(0)から約21.6°ずれたまま動かず、ステア電流2.83Aが流れ続ける = ストール。
- `POWER_FAULT_DRIVE_OVERCURRENT` はラッチされず。
- `batt_current_drive` (STM32側バッテリー電流センサ) は終始0.00A。
- 前日の別セッションでは原点ズレが +0.392 rad (+22.4°) で符号が逆だった → 電源投入毎に原点がズレている。

### コード調査結果 (このリポジトリ側)

1. **DISARM中もステアは位置制御され続ける (未実装の穴)**
   [app.c:433-434](../src/app/app.c#L433-L434) `ApplyRasCommand()` は `armed` に関係なく毎周期
   `Steering_SetRoadWheelAngleRad()` を呼ぶ。`Drive` は `Drive_Enable`/`Drive_Disable` で
   明確にON/OFFされる ([app.c:430-431](../src/app/app.c#L430-L431)) のに対し、Steering には
   対応する無効化機構がない。位置モードのMDは目標角へ向けて連続的にトルクを掛け続けるため、
   DISARM中でも中心点(target=0)へ向けて突っ張り続ける。ストール検出・電流タイムアウトも無し。
   → **仕様ではなく未実装。要修正候補。**

2. **駆動電流センサが0Aに張り付く件**
   [power.c:180-183](../src/power/power.c#L180-L183) の `Power_GetCurrentDrive()` は
   ADC1 CURRENT_P (PC1, INA180A2 + R28=5mΩ) 1系統で DRIVE_POWER 配下の3台のMD
   (操舵+左右後輪) の合算電流を読む設計。ステアMDが2.83A流しているなら本来反映されるはず。
   - ハード側 (シャント未実装/配線/ADCチャンネル) の可能性
   - Piログの「駆動電流」がMD自身のテレメトリ (`motor_current_a[0]`) を指しており、
     STM32側センサ値と別物として比較していた可能性
   → 実機でテスターを当てて切り分けが必要。

3. **過電流ラッチ不発は (2) の帰結**
   [power.c:120-124](../src/power/power.c#L120-L124) は `Power_GetCurrentDrive() > 9.0A` を
   50ms継続で判定するが、センサ値が常に0Aなら判定に掛からない。センサが機能していないなら
   駆動系過電流保護は事実上無効。

4. **原点が起動毎に変わる件**
   [steering.c:56-68](../src/control/steering.c#L56-L68) `Steering_Init` は起動時に
   BUTTON1 が押されていればその場の角度をそのまま中心点として保存、押されていなければ
   Flashの保存値を読む仕様 ([app.c:536](../src/app/app.c#L536))。Flash値は明示的に
   再キャリブレーションしない限り変わらないため、毎回符号・大きさが変わるのは
   - 起動毎に意図せずBUTTON1がONでキャリブレーションが走っている
   - モータ〜リンク機構間のカプラがストール反力でスリップしている (今回の現象と整合的)
   のどちらかを疑う。

### 未確認・要現物確認
- ステアリングは手で動くか (機械的に噛んでいないか)
- カプラ/リンク機構のスリップ有無
- CURRENT_P (PC1) 周りのシャント実装・配線
- BUTTON1 (PB14) が起動時に意図せずONになる配線/固着がないか

### 対応候補 (未着手)
- Steering にも Drive 同様の Enable/Disable (非armed時はトルク上限0 or MD解放) を追加
- ステアのストール検出 (角度偏差が一定時間収束しない場合にトルクを下げる/フォールト化)

### 続報: 「なぜ駆動電源が切れるのか」の調査 (同日)

Pi側から「一度もarmしていないのに9.6Vから電圧が落ちた。省電力OFFの類はあるか」との質問。
DRIVE_POWER を操作する箇所を全て洗い出した結果、**タイムアウトによる自動OFFはこのファーム
には存在しない**ことを確認。

- [app.c:571](../src/app/app.c#L571) `Setup()` 末尾で無条件に ON (armの有無に無関係)。
- [app.c:403](../src/app/app.c#L403) `ApplyRasCommand()` で `arm_requested` に従い上書き。
  ただしこれは `RasLink_IsCommandAlive()` が真の間 (Piから100ms間隔でCOMMANDが届き続けて
  いる間) しか呼ばれない ([app.c:498](../src/app/app.c#L498))。
- 他にOFFにする経路は過電流ラッチのみ (今回未発生)。

**「一度もarmしていない」のに9.6Vが観測できたことの整合性**が課題として残る。2通りの
可能性を提示し、Pi側の確認待ち:

1. **COMMANDを一度も送っていなかった場合**: `has_command` が false のまま
   `UpdateDriveTest()` 分岐 ([app.c:504](../src/app/app.c#L504)) が動き続け、DRIVE_POWERは
   `Setup()` のONが保持される。この分岐はBUTTON1起点の下位単独走行テストで、カウントダウン後に
   `Steering_SetAngleRad(&steering, 0.0f)` と `Drive_SetTargetSpeed()` を実際に発行する
   ([app.c:205-207](../src/app/app.c#L205-L207))。ログの「指令rad=0.0000」がPiのCOMMANDでは
   なく**本体BUTTON1起点の走行テストの指令値**だった疑いがある。
2. **COMMANDを継続送信していた場合 (armなし)**: 初回COMMAND到達時点で即座にOFFになるはずで
   9.6V観測と矛盾する。VOLTAGE_P (PC3) がDRIVE_POWERスイッチより手前 (常時通電の生電圧) を
   測っている回路だとすれば、電圧低下は電源OFFではなく「ステアのストールによるバッテリー
   サグ」単体の現象として説明できる。要回路図確認。

Pi側に確認を依頼した項目:
- flags の mode ビット(bit0-1) と `RAS_FLAG_ARMED` (bit2) の実際の値
- BUTTON1 (LED1点灯) が押された形跡の有無
- COMMANDパケットが実際に送信されていたか (Pi側送信ログ)
- VOLTAGE_P (PC3) のタップ位置 (DRIVE_POWERスイッチの前/後)

### 結論 (同日、Pi側ログ解析で決着): バグではなく仕様どおりの挙動

Pi側の送受信ログを解析した結果、原因が確定した。

- COMMAND は 89.7Hz で継続送信されており、`arm` ビットは終始 `0`。100ms超の途絶は0回
  (`RasLink_IsCommandAlive()` は常に真)。
- 初COMMAND受信 (Pi時刻 0.001s) から **+13ms** で駆動電圧の崩壊が始まった。
- STM32は起動から約50秒間、Piが何も送ってこない間 `Setup()` のDRIVE_POWER=ONを保持していた。
  最初のCOMMANDが届いた瞬間、[app.c:403](../src/app/app.c#L403) が
  `Power_SetDrivePower(&power, arm_requested=false)` を実行し、そのままOFFになった。
- VOLTAGE_P (PC3) は **DRIVE_POWERスイッチの後ろ**を測っていると確定 (スイッチが開いた
  瞬間に電圧が落ちているため)。9.6V→1.6Vの「崩壊」はステア電流によるサグではなく、
  スイッチが切れた後のコンデンサ放電波形。

**過去の誤った推測を訂正:**
1. 「駆動バッテリー未接続」→ 誤り。常時接続されていたが、DISARM (arm=0) のCOMMANDで
   DRIVE_POWERがOFFになっていただけ。
2. 「ステアがストール」→ 誤り。ST電流は0.91Aで一定、急増なし。
3. 「電流センサが効いていない」→ 根拠不足。スイッチがOFFなら0.00Aが正しい値。

上記PROGRESSの2/3/4節の「駆動電流センサ異常」「ステアストール」に関する分析は本件の
原因ではなかったとして無効化する。ただし **1節(DISARM中もSteeringが位置制御され続ける
設計上の穴)自体はコード上の事実として引き続き有効**(未検証・未修正のまま)。

**設計上の含意:** Piが接続していてもarmしていない間、DRIVE_POWERは必ずOFFになる
(安全側の正しい挙動)。そのためベンチで駆動電源を上げたまま監視だけしたい場合、
COMMANDを送るとその瞬間に切れる。Pi側で「COMMANDを送らずに受信だけする」ことでのみ
ONを維持できる (ただしその場合 `RAS_FLAG_UART_TIMEOUT` は出ないが `has_command=false`
のままなので上位連携としては未接続扱いになる点に注意)。

**検証実験で確認済み (Pi側):** COMMANDを一切送らず60秒受信のみ→駆動電圧11.00Vが微動
だにせず維持。VOLTAGE_P (PC3) が DRIVE_POWER スイッチの後ろにあることも確定。
MD配線・comm_okも正常(通信断は電源喪失の結果であり原因ではない)。本件は完全に決着。

### 派生した設計論点: フェイルセーフの向きが逆転している

Pi側から指摘。実際のflags推移:

| 状態 | flags |
|------|-------|
| Pi沈黙中 (起動直後、COMMAND未着) | ARMED + 駆動電源 ON |
| Pi接続しDISARM送信 | 駆動電源 OFF |

**監視者(Pi)がいないときの方が「動ける」状態になっている。** 原因は
[app.c:571](../src/app/app.c#L571) の `Setup()` 末尾で無条件にDRIVE_POWERをONにする
設計 (BUTTON1単独走行テストをPi無しでも動かすための意図的な設計、CLAUDE.mdにも
明記されている過去の決定)。`UpdateDriveTest()` 自身は `Power_SetDrivePower()` を
一度も呼んでおらず、Setup()のONに乗っかっているだけ ([app.c:179-232](../src/app/app.c#L179-L232)
には呼び出し無し)。

Pi統合が主運用になった今、「監視者不在=動ける」は fail-open であり、一般的な
フェイルセーフの向き (監視者不在・信号無し=安全側) と逆。

**対応 (実施済み):** バンビの判断で、ボタン1単体走行テスト (`UpdateDriveTest` および
関連の `StopTest`/`IsButton1Pressed`/`TestState` 一式) を [app.c](../src/app/app.c) から
削除。あわせて `Setup()` 末尾の `Power_SetDrivePower(&power, 1)` 無条件ONも撤廃し、
以下の挙動に変更した:

- `Setup()` 中はステアリング原点較正 (MDとの通信が必要) のためだけに一時的に
  DRIVE_POWER をONにし、`Steering_Init()`/`Drive_Init()` の直後に明示的にOFFへ戻す。
- 以降は `ApplyRasCommand()` が上位の `RAS_CMD_FLAG_ARM` 要求に従ってON/OFFする経路のみ。
- `UpdateVehicleControl()` の「COMMAND未受信」「COMMAND途絶」の2分岐を
  `ApplyCommandTimeout()` に統合。上位と繋がっていない間は常に目標車速0で停車保持。

結果、Pi未接続時のflagsは `ARMED=False` になり、fail-openは解消。`make` でビルド確認済み
(エラー・警告なし)。CLAUDE.mdの `src/app/app.c の状態` 節も合わせて更新。

---

## 2026-08-08: GUI の LiDAR 点群が左右反転 → Pi 側で修正 (STM32 側は変更なし)

### 原因

STM32 は「センサ基準の角度をそのまま送り、左右の解釈は上位で行う」設計
([lidar.h](../src/sensing/lidar.h) のモジュール解説に経緯あり)。LD06 を裏向き
(PCB面が下) に取り付けているため角度の増加方向が車両座標と逆で、
`sector_idx * 30 + i` をそのまま度として使うと点群が左右反転する。

Pi 側 (`AutonomousCar_RasPi/surge_mk2/raspi/msgs/convert.py` の `ScanAssembler`) が
この反転を実装しておらず、生の添字のまま 360点配列へ詰めていたのが直接原因。

### 対応 (Pi 側で実施済み)

`ScanAssembler` の1箇所だけで `車両角 = (360 - センサ角) % 360` に変換する。

- `sector_t_ns` / `sector_dur_us` / `sector_seen` も車両座標のセクタ番号
  `11 - sector_idx` へ移動 (GUI の欠測セクタ表示が点群とずれないように)
- 反転でセクタ内の時刻の向きが逆になる (`t[j] = t_start + dur * (29 - j) / 29`)
- 回帰テスト追加。Pi 側テスト 259件 OK、GUI の `tsc --noEmit` も通過
- ドキュメント更新: Pi 側 `uart_protocol.md` §5.1 / `stm32_interface.md` §7.1 /
  `types.py` / `gui/src/types.ts` / `PROGRESS.md`

**ワイヤ形式・`protocol_version` (0x0004) は無変更。このリポジトリのコード変更は不要。**
STM32 側で反転しない理由 (0°/180° がセクタ境界に乗って1セクタが分裂する、
実際に試すと原因不明の通信エラーが増発する) は [lidar.h](../src/sensing/lidar.h) のまま有効。
