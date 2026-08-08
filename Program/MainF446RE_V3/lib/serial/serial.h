#ifndef __SERIAL_H__
#define __SERIAL_H__

#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "usart.h"

typedef struct {
  UART_HandleTypeDef* huart;
  uint8_t* rxBuf;
  uint16_t rxTop;
  uint16_t rxBtm;
  uint16_t rxBufSize;
} Serial;

// インスタンス生成
static inline void Serial_Init(Serial* self, UART_HandleTypeDef* huart, uint16_t rxBufSize) {
  self->huart = huart;
  self->rxBuf = (uint8_t*)malloc(rxBufSize);
  memset(self->rxBuf, 0, rxBufSize);
  self->rxTop = 0;
  self->rxBtm = 0;
  self->rxBufSize = rxBufSize;
  HAL_UART_Receive_DMA(huart, self->rxBuf, rxBufSize);
}

// データ受信可否
static inline bool Serial_Available(Serial* self) {
  uint16_t rxTop = self->rxBufSize - self->huart->hdmarx->Instance->NDTR;
  return rxTop != self->rxBtm;
}

// 1バイト受信
static inline uint8_t Serial_Read(Serial* self) {
  uint16_t rxTop = self->rxBufSize - self->huart->hdmarx->Instance->NDTR;
  if (rxTop == self->rxBtm) {
    return 0;
  }
  if (((rxTop + self->rxBufSize - self->rxBtm) % self->rxBufSize) == 0) {
    return 0;
  }
  uint16_t available = (rxTop + self->rxBufSize - self->rxBtm) % self->rxBufSize;
  if (available > self->rxBufSize - 1) {
    self->rxBtm = (rxTop + self->rxBufSize - 1) % self->rxBufSize;
  }
  uint8_t data = self->rxBuf[self->rxBtm];
  self->rxBtm = (self->rxBtm + 1) % self->rxBufSize;
  return data;
}

// 1バイト送信
static inline void Serial_WriteByte(Serial* self, uint8_t data) {
  HAL_UART_Transmit_DMA(self->huart, &data, 1);
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
}

#endif