#include "main.h"

#ifdef STM32F411

#define USE_USART_DMA

#ifdef USE_USART_DMA
  // Page 170 RM0383:
  //
  // USART1: Channel_4
  // Rx USART1: DMA2_Stream2_IRQn or DMA2_Stream5_IRQn interrupt
  // Tx USART1: DMA2_Stream7_IRQn interrupt
  //
  // USART6: Channel_5
  // Rx USART6: DMA2_Stream1_IRQn or DMA2_Stream2_IRQn interrupt
  // Tx USART6: DMA2_Stream6_IRQn or DMA2_Stream7_IRQn interrupt
  //
  // USART2: Channel_6 for Stream7 and Channel_4 for Stream5 and Stream6
  // Rx USART2: DMA1_Stream5_IRQn or DMA2_Stream7_IRQn interrupt
  // Tx USART2: DMA2_Stream6_IRQn
  //
void dma_init(DMA_TypeDef* dma, DMA_Stream_TypeDef* dma_channel);

#endif

#define Fpclk2 SystemCoreClock
#define Fpclk1 SystemCoreClock/2

#define USART_SIZE_BUFFER 16

#define USART1_TX_PORT GPIOA
#define USART1_TX_PIN 9
#define USART1_RX_PORT GPIOA
#define USART1_RX_PIN 10

#define USART2_TX_PORT GPIOA
#define USART2_TX_PIN 2
#define USART2_RX_PORT GPIOA
#define USART2_RX_PIN 3

#define USART6_TX_PORT GPIOA
#define USART6_TX_PIN 11
#define USART6_RX_PORT GPIOA
#define USART6_RX_PIN 12

#define USART1_af af7
#define USART2_af af7
#define USART6_af af8

#endif
