/*
 * LED.h
 *
 * Created: 12/6/2017 5:09:04 PM
 *  Author: raghu
 */ 


#ifndef LED_H_
#define LED_H_

void LED_init();
void enable_LED();
void disable_LED();
void initializing_LED_pin_array();
void LED_transfer_temp();
void increasing_LED_sort_tag();

void transfer_temp_LED_2();

extern volatile uint8_t serial_timeout_count;
extern volatile uint8_t count_broadcast;
extern volatile bool serial_timeout;

#define MAX_SERIAL_TIMEOUT 10
#define LED_ON true
#define LED_OFF false




#endif /* LED_H_ */