#include "stm32g4xx_hal.h"
#include "usart.h"

#ifdef __GNUC__
#define PUTCHAR_PROTOTYPE int __io_putchar(int ch)
#define GETCHAR_PROTOTYPE int __io_getchar(void)
#else
#define PUTCHAR_PROTOTYPE int fputc(int ch, FILE *f)
#define GETCHAR_PROTOTYPE int f getc(FILE* f)
#endif

void __io_putchar(uint8_t ch)
{
  HAL_UART_Transmit(&hlpuart1, &ch, 1, 0xFFFFFFFF);
}

int __io_getchar(void)
{
  uint8_t rxBuf;
  while(HAL_UART_Receive(&hlpuart1, &rxBuf, 1, 0xFFFFFFFF) != HAL_OK);
  return(rxBuf); 
}