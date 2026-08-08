#ifndef CRC16_H_
#define CRC16_H_

#include <stddef.h>
#include <stdint.h>

// CRC-16/CCITT-FALSE
//
//   poly = 0x1021, init = 0xFFFF, xorout = 0x0000, 入出力のビット反転なし
//   チェック値: "123456789" (0x31〜0x39 の9バイト) → 0x29B1
//
// **なぜ MD 用の CRC-8/AUTOSAR ではなく 16bit なのか**
//   CRC-8/AUTOSAR (poly=0x2F) がハミング距離4を保てるのは 15バイト程度までで、
//   それを超えると HD=2 (2ビット化けを見逃す) に落ちる。Raspberry Pi との
//   フレームは LiDAR セクタが 76バイト、テレメトリが 73バイトあり、8bit では
//   保護が足りない。0x1021 は 240ビット (30バイト) まで HD=6、32751ビットまで
//   HD=4 を保つため、本プロトコルの全フレーム長で 3ビット化けまで確実に検出できる。
//
// **なぜテーブルを持たないか**
//   76バイトで 608 ループ、180MHz なら数μs。120Hz で回しても CPU の 0.1% 未満で、
//   512バイトの ROM を使ってまで削る場面ではない。速度が要るなら結果を変えずに
//   256エントリのテーブル駆動へ差し替えてよい。
#define CRC16_POLY 0x1021
#define CRC16_INIT 0xFFFF

// 1バイト分を畳み込む。フレームを組み立てながら計算する用途向け
static inline uint16_t Crc16Update(uint16_t crc, uint8_t data) {
  crc ^= (uint16_t)data << 8;
  for (uint8_t bit = 0; bit < 8; bit++) {
    crc = (crc & 0x8000) ? (uint16_t)((crc << 1) ^ CRC16_POLY) : (uint16_t)(crc << 1);
  }
  return crc;
}

static inline uint16_t Crc16(const uint8_t* data, size_t len) {
  uint16_t crc = CRC16_INIT;
  for (size_t i = 0; i < len; i++) {
    crc = Crc16Update(crc, data[i]);
  }
  return crc;
}

#endif  // CRC16_H_
