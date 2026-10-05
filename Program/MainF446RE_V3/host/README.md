# host/ — `src/control` をホスト (Mac/Linux) で動かす

`src/control` (Drive・TorqueVectoring・ControlParams) を**そのまま**ホストでコンパイルし、後輪＋
タイヤ＋車体のモデルと閉ループで回すためのもの。実機へ書き込むファームには含まれない
(`Makefile` は `src/` と `lib/` だけを拾う)。

使うのは上位側リポジトリ (AutonomousCar_RasPi/surge_mk2) の `tools/ctrl_tune`:

- `python -m tools.ctrl_tune.bench` — コミット済みの制御と作業ツリーの制御を同じ場面で比べる
- `python -m tools.ctrl_tune.gui` — 実機の記録で車両モデルを同定し、調整パラメータを最適化する
- `python -m pytest tools/ctrl_tune/tests` — 制御の約束 (発進で TC が働く・片輪を絞った分を
  反対輪へ載せない・定速で介入しない 等) の回帰テスト

**`src/control` のロジックを変えたら、変える前後で `bench` を回して比べること。**

## 中身

| ファイル | 役割 |
|---|---|
| `host_sim.c` | 車両モデルと閉ループの本体。`host_sim_run()` (1本まとめて)、`host_sim_create/step/free()` (続きから進める。上位の planner を 10Hz で閉ループにするとき) |
| `shim/*.h` | HAL に触るヘッダの差し替え (`timer.h`・`bldc_motor.h`・`encoder.h`・`imu.h`・`serial.h`)。`src/control` が呼ぶ分だけを持つ |

コンパイル (上位の `tools/ctrl_tune/fw.py` が自動で行う):

```sh
cc -O2 -shared -fPIC -DHOST_FW_HAS_CONTROL_PARAMS \
   -Ihost/shim -Ilib/pid -Ilib/filter -Ilib/mymath -Isrc/control \
   host/host_sim.c src/control/drive.c src/control/torque_vectoring.c src/control/control_params.c \
   -o host_sim.dylib
```

`-Ihost/shim` を先頭に置くのが肝: `drive.h` が `#include "encoder.h"` などと書いているのを、
実物 (`src/sensing/`) ではなく差し替えへ向ける。`-Ilib/timer`・`-Isrc/sensing` は付けない。
`HOST_FW_HAS_CONTROL_PARAMS` を付けなければ、`control_params.h` を持たない古いコミットの
`src/control` もコンパイルできる (上位が `git archive` で取り出して比べる)。

## 車両モデル (`host_sim.c`)

2kHz の制御周期を 20 に刻んで積分する。

- **後輪 (左右)**: 慣性×角加速度 = モータのトルク − 半径×タイヤの前後力 − 摩擦
- **タイヤの前後力**: `F = D·sin(C·atan(B·κ))`、`D = μ×荷重` (摩擦円で横力ぶんを引く)。
  荷重は加減速・旋回で移る。スリップ率 κ の分母には下限
- **モータドライバ**: 指令 → むだ時間 → 1次遅れ。制動は `-tanh(ω/20)` で抜ける
- **車体**: 前後は両輪の力 − 転がり抵抗。ヨーは「舵で決まる定常のヨーレートへ戻る減衰」＋
  左右の力の差のモーメント
- **センサ**: 前輪エンコーダ (ノイズ・倍率誤差・1回転周期の誤差・断線)、MD の角速度 (遅れ・
  ノイズ)、ジャイロ (遅れ・ノイズ・バイアス)

**表していないもの**: 後輪の横滑り (スピン)、路面の凹凸、タイヤの温度、バッテリー電圧による
トルクの頭打ち、MD の通信エラー。

パラメータの値・出どころは上位側 `tools/ctrl_tune/plant.py`。

## 変えるときの約束

- `HostPlant`・`HostConfig`・入出力の列を変えたら `HOST_SIM_ABI` を上げ、上位側
  `tools/ctrl_tune/fw.py` (`ABI`・`HostPlant`・`IN`・`OUT`) も揃える。合っていなければ上位が
  読み込み時に止まる
- `Drive` が新しく HAL 側の関数を呼ぶようになったら、`shim/` に同じ名前の差し替えを足す
- 時刻 (`g_host_now_s`) と舵角はファイル内の共有変数。シミュレーションを複数スレッドで
  同時に進めないこと (上位はプロセスを分けて並列にしている)
