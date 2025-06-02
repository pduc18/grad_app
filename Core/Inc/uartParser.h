// uart_parser.h
#ifndef UARTPARSER_H
#define UARTPARSER_H

#include <stdint.h>

void UART_ProcessDMAData(uint16_t current_write_index);
float getX(void);
float getY(void);

#endif // UART_PARSER_H
