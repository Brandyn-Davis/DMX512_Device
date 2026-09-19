#ifndef UART0_H_
#define UART0_H_

void initUart0();
void setUart0BaudRate(uint32_t baudRate, uint32_t fcyc);
void putcUart0(char c);
void putsUart0(char* str);
char getcUart0();
void getsUart0(char* str, uint8_t maxChars);
bool kbhitUart0();

#endif
