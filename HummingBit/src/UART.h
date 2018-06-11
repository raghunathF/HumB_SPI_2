/*
 * UART.h
 *
 * Created: 12/12/2017 8:46:30 AM
 *  Author: raghu
 */ 


#ifndef UART_H_
#define UART_H_

#define MC_UART_DELAY 6

#define SERIAL_MODULE              SERCOM1
#define SERIAL_SERCOM_MUX_SETTING  USART_RX_3_TX_2_XCK_3
#define SERIAL_SERCOM_PINMUX_PAD0  PINMUX_UNUSED
#define SERIAL_SERCOM_PINMUX_PAD1  PINMUX_UNUSED
#define SERIAL_SERCOM_PINMUX_PAD2  PINMUX_PA24C_SERCOM1_PAD2
#define SERIAL_SERCOM_PINMUX_PAD3  PINMUX_PA25C_SERCOM1_PAD3


#define BAUDRATE_SERIAL_DEBUG 115200

void serial_init();
int convert(int k , uint8_t* output);
void enable_USART();
void usart_read_callback(struct usart_module *const usart_module);
struct usart_module usart_instance;


#endif /* UART_H_ */