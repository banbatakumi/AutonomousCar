# AutonomousCar

自律走行ミニカー **SURGE Mark.2** の STM32 ファームウェアと基板（回路）。

**技術解説サイト: https://banbatakumi.github.io/AutonomousCar_RasPi/**
ファームの制御（車速制御・TC・ABS・トルクベクタリング・安全層）もこのサイトで解説しています。

> **ファームウェアの最新は [`FW/MainBoard_V3`](https://github.com/banbatakumi/AutonomousCar/tree/FW/MainBoard_V3) ブランチです。**
> 既定ブランチ `main` の `Program/` は古い状態のままです。

## 構成

| ディレクトリ | 内容 |
|---|---|
| `Program/MainF446RE_V3/` | 現行ファーム（STM32F446RE）。最新は上記ブランチ。`Program/MainF446RE`・`MainF446RE_V2` は旧版 |
| `Circuit/` | 基板の設計（`MainBoard_V1`〜`V3_2`、`OtherBoard_V1`） |

## 関連リポジトリ

[`AutonomousCar_RasPi`](https://github.com/banbatakumi/AutonomousCar_RasPi) — Raspberry Pi 側のプログラム。
Pi とは UART（1 Mbps）で通信します。プロトコル（`RAS_PROTOCOL_VERSION`）は Pi 側と**セットで**入れ替えます。
