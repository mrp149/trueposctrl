#ifndef UART_H_
#define UART_H_

int uart_init();
int uart_tx(const char * msg, size_t ln);
int uart_rx();

#endif
