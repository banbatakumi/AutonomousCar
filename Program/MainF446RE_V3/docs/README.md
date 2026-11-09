# docs

現行の仕様の正はコード (`src/comm/ras_link.h`・`ras_link.c`、`src/control/drive.h`、`src/control/control_params.h`)。
プロトコルの数値・バイト配置の唯一の定義は Pi 側リポジトリの `raspi/proto/protocol.toml`（説明は Pi 側 `docs/uart_protocol.md`）。

プロトコルを変えたら `RAS_PROTOCOL_VERSION` を上げ、Pi 側の `protocol.toml` と一緒に入れ替える。変更の経緯はコミットメッセージに書く。

## archive/ (過去の記録。通常は読まない。古い数値・仕様を含む)

- `pi_uart_protocol_v0.4_request.md`・`v0.5〜v0.29_delta.md`: 各版の差分。後の版で変わった箇所はそのまま残っている
- `PROGRESS.md`: 2026-08 の調査・作業ログ (ステア電圧崩壊、app.c 分割など)
- `code_review_2026-08-21.md`: 2026-08-21 のレビューと対応状況
- `md_protocol_update_prompt.md`: 2026-09 の MD 改修依頼 (現行は MD 側 `docs/SERIAL_PROTOCOL.md`)
- `CLAUDE_md_until_2026-10-11.md`: 整理前の CLAUDE.md (経緯・実機確認メモ込み)
