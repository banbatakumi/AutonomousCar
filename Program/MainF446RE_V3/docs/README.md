# docs

現行の仕様の正はコード (`src/comm/ras_link.h`・`ras_link.c`、`src/control/drive.h`、`src/control/control_params.h`)。

## 直下 (現行・最近の変更点)

| ファイル | 内容 |
|---|---|
| `pi_uart_protocol_v0.19_delta.md` | 「止める」の約束 (ARM 解除時の停車まで制動・MD 健全性・ブレーキホールド)、`flags` bit22/23 |
| `pi_uart_protocol_v0.20_delta.md` | ローンチコントロール |
| `pi_uart_protocol_v0.21_delta.md` | 車速PIの加速度フィードフォワード |
| `pi_uart_protocol_v0.22_delta.md` | 目標車速ランプの固定上限の廃止 |
| `pi_uart_protocol_v0.23_delta.md` | 調整パラメータの既定値をファームに持たない・車速PIのゲインを調整パラメータに (現行版 0x0017) |

各 delta は直前の版からの差分だけを書く。プロトコルを変えたら `RAS_PROTOCOL_VERSION` を上げて delta を足す。

## archive/ (過去の記録。通常は読まない。古い数値・仕様を含む)

- `pi_uart_protocol_v0.4_request.md`: v0.4 のベース仕様 (フレーム・PING/PONG・LiDAR セクタなど)。COMMAND/TELEMETRY のレイアウトは以降の版で変わっている
- `pi_uart_protocol_v0.5〜v0.18_delta.md`: 各版の差分
- `PROGRESS.md`: 2026-08 の調査・作業ログ (ステア電圧崩壊、app.c 分割など)
- `code_review_2026-08-21.md`: 2026-08-21 のレビューと対応状況
- `md_protocol_update_prompt.md`: 2026-09 の MD 改修依頼 (現行は MD 側 `docs/SERIAL_PROTOCOL.md`)
- `CLAUDE_md_until_2026-10-11.md`: 整理前の CLAUDE.md (経緯・実機確認メモ込み)
