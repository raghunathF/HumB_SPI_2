/*
 * LED_control.h
 *
 * Created: 12/8/2017 10:10:57 AM
 *  Author: raghu
 */ 
#include <asf.h>
void update_LEDS(uint8_t led1, uint8_t led2);
void update_LEDS_single(uint8_t port_no, uint8_t led);
void switch_off_LEDS();
extern bool update_compare_led_array;
extern uint8_t temp_compare_led_array[4];
extern uint8_t temp_compare_led_array_2[4];