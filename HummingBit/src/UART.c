/*
 * UART.c
 *
 * Created: 12/12/2017 8:45:17 AM
 *  Author: raghu
 */ 
#include <asf.h>
#include "UART.h"
#include "UART_control.h"

extern volatile uint8_t ring_buffer[MAX_LIMIT_RING_BUFFER];
extern volatile uint8_t tail_ring_buffer;
extern volatile uint8_t head_ring_buffer;
extern volatile bool received_data_updated;


#define MAX_RX_BUFFER_LENGTH   1
volatile uint8_t rx_buffer[MAX_RX_BUFFER_LENGTH];

void configure_usart(void)
{

	struct usart_config config_usart;
	usart_get_config_defaults(&config_usart);
	
	config_usart.baudrate    = BAUDRATE_SERIAL_DEBUG;
	config_usart.mux_setting = SERIAL_SERCOM_MUX_SETTING;
	config_usart.pinmux_pad0 = SERIAL_SERCOM_PINMUX_PAD0;
	config_usart.pinmux_pad1 = SERIAL_SERCOM_PINMUX_PAD1;
	config_usart.pinmux_pad2 = SERIAL_SERCOM_PINMUX_PAD2;
	config_usart.pinmux_pad3 = SERIAL_SERCOM_PINMUX_PAD3;
	
	while (usart_init(&usart_instance,SERIAL_MODULE, &config_usart) != STATUS_OK)
	{
	}
	usart_enable(&usart_instance);
	//uint8_t string[] = "Hello World!\r\n";
	//usart_write_buffer_wait(&usart_instance, string, sizeof(string));
}




void usart_read_callback(struct usart_module *const usart_module)
{
	volatile uint16_t received_data = 0;
	received_data_updated = true;
	received_data = (usart_instance.hw->USART.DATA.reg & SERCOM_USART_DATA_MASK);
	*(ring_buffer + head_ring_buffer) = (uint8_t)received_data;
	head_ring_buffer++;
	received_data_updated = true;
}

void configure_usart_callbacks()
{
	usart_register_callback(&usart_instance,usart_read_callback, USART_CALLBACK_BUFFER_RECEIVED);
}


void enable_USART()
{
	usart_enable_callback(&usart_instance, USART_CALLBACK_BUFFER_RECEIVED);
	usart_read_buffer_job(&usart_instance,(uint8_t *)rx_buffer, MAX_RX_BUFFER_LENGTH);
}

void serial_init()
{
	configure_usart();
	configure_usart_callbacks();
	
}


