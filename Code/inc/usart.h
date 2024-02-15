
#ifdef STM32F411

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