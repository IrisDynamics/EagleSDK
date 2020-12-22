#include "UARTs.h"

void uart2_status_isr(void)
{
  UART2.isr();
}
void uart1_status_isr(void)
{
  UART1.isr();
}
