#ifndef __SERIAL_H__
#define __SERIAL_H__

#include <stdbool.h>
#include <string.h>

#include "timer.h"
#include "usart.h"

typedef struct {
  UART_HandleTypeDef* huart;
  uint8_t* rxBuf;
  uint16_t rxTop;
  uint16_t rxBtm;
  uint16_t rxBufSize;
  uint32_t rxLastPollUs;  // オーバーラン検出用、最後に Serial_Available を呼んだ時刻
  bool rxOverrun;         // Serial_Available が検出し、Serial_Read が消費して読み捨てに使うフラグ
} Serial;

// インスタンス生成。rxBuf には rxBufSize バイト以上の static バッファを渡すこと
// (組み込みではヒープを使わない方が確実なため、所有権は呼び出し側に置く)。
static inline void Serial_Init(Serial* self, UART_HandleTypeDef* huart, uint8_t* rxBuf, uint16_t rxBufSize) {
  self->huart = huart;
  self->rxBuf = rxBuf;
  memset(self->rxBuf, 0, rxBufSize);
  self->rxTop = 0;
  self->rxBtm = 0;
  self->rxBufSize = rxBufSize;
  self->rxLastPollUs = Micros();
  self->rxOverrun = false;
  HAL_UART_Receive_DMA(huart, self->rxBuf, rxBufSize);
}

// データ受信可否。DMAが読み出し位置を追い越した(オーバーラン)かも合わせて判定する。
// rxTop/rxBtmの差分だけを見ると mod 演算の結果は常に [0, rxBufSize-1] に収まってしまい、
// 「一度も追い越されていない」のか「何周も追い越された」のかを区別できない
// (周回数ぶんの情報が失われる)。そのため何バイト流れたかではなく、「バッファを満たすのに
// 要する時間より長くポーリング間隔が空いたか」を実時間 (Micros) で判定する。
// 呼び出し側 (RasLink_Update 等) が制御周期ごとに呼ぶ前提 (でなければこの判定自体が狂う)。
//
// 注意: メインループを止める長時間ブロッキング処理 (src/sensing/imu.c の静止キャリブレー
// ション中の HAL_Delay、lib/flash/flash.h の HAL_FLASHEx_Erase、IMU の I2C復旧処理等) の
// 直後は、実際の受信バイト数によらず (baudレートの理論上限で計算した) fill_time_us を
// 上回りやすく、保守的に「オーバーランの疑いあり」と判定されやすい。ただしこれは
// Serial_Read() 側の「最新位置まで読み捨てて追いつく」動作と対になっており、ブロッキング
// 中に溜まった (既に上書きされ得る) 古いバッファ内容を処理せず安全に読み飛ばして最新位置へ
// 復帰する設計なので、それ自体は正常な自己修復動作である。実際に受信済みで未上書きの
// フレームまで読み飛ばしてしまう (真の誤検知) 可能性はあるが、そちらはCRCチェックのある
// 上位のフレームパーサ側で1フレーム分の取りこぼしとして許容する設計になっている
static inline bool Serial_Available(Serial* self) {
  uint32_t baud = self->huart->Init.BaudRate;
  uint32_t now = Micros();
  uint32_t elapsed_us = now - self->rxLastPollUs;
  self->rxLastPollUs = now;
  if (baud != 0) {
    uint32_t fill_time_us = (uint32_t)(((uint64_t)self->rxBufSize * 10000000ULL) / baud);
    if (elapsed_us > fill_time_us) self->rxOverrun = true;
  }

  uint16_t rxTop = self->rxBufSize - self->huart->hdmarx->Instance->NDTR;
  return rxTop != self->rxBtm || self->rxOverrun;
}

// 1バイト受信
static inline uint8_t Serial_Read(Serial* self) {
  if (self->rxOverrun) {
    // 積んだままの分は既に上書きされて信頼できないので、最新位置まで読み捨てて追いつく
    // (誤った過去データで制御するより、フレームを1つ失う方が安全)
    self->rxBtm = self->rxBufSize - self->huart->hdmarx->Instance->NDTR;
    self->rxOverrun = false;
  }
  uint16_t rxTop = self->rxBufSize - self->huart->hdmarx->Instance->NDTR;
  if (rxTop == self->rxBtm) {
    return 0;
  }
  uint8_t data = self->rxBuf[self->rxBtm];
  self->rxBtm = (self->rxBtm + 1) % self->rxBufSize;
  return data;
}

// 複数バイト送信
static inline void Serial_Write(Serial* self, const uint8_t* data, uint16_t len) {
  HAL_UART_AbortTransmit(self->huart);
  HAL_UART_Transmit_DMA(self->huart, (uint8_t*)data, len);
}

// 送信中かどうか
static inline bool Serial_IsTxBusy(Serial* self) {
  return self->huart->gState != HAL_UART_STATE_READY;
}

// 送信中なら何もせず false を返す、中断を伴わない送信。
//
// Serial_Write は進行中の DMA を叩き切るため「次の送信までに必ず完了する」用途にしか使えず、
// 連続的にストリームを流す通信 (Raspberry Pi 側) では前のフレームが途中で切れてしまう。
// こちらは前の送信が終わるまで単に送らないので、呼び出し側でキューを持てば取りこぼしなく流せる。
//
// 注意: HAL は DMA 完了後に UART の TC 割り込みで gState を READY に戻すため、
// この関数を使う UART は NVIC の割り込みを有効にしておくこと (無効だと初回の1回しか送れない)。
// data の指す領域は送信完了まで有効なままにすること (DMA が直接読むため)。
static inline bool Serial_WriteAsync(Serial* self, const uint8_t* data, uint16_t len) {
  if (Serial_IsTxBusy(self)) return false;
  return HAL_UART_Transmit_DMA(self->huart, (uint8_t*)data, len) == HAL_OK;
}

static inline void Serial_Reset(Serial* self) {
  HAL_UART_AbortReceive(self->huart);
  HAL_UART_DMAStop(self->huart);
  memset(self->rxBuf, 0, self->rxBufSize);
  HAL_UART_Receive_DMA(self->huart, self->rxBuf, self->rxBufSize);
  self->rxBtm = 0;
  self->rxLastPollUs = Micros();
  self->rxOverrun = false;
}

#endif