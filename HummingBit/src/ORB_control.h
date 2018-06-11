/*
 * ORB_control.h
 *
 * Created: 12/8/2017 9:45:25 AM
 *  Author: Raghunath Jangam
 */ 


#ifndef ORB_CONTROL_H_
#define ORB_CONTROL_H_

void update_ORB_LED(uint8_t r1 ,uint8_t g1 ,uint8_t b1 ,uint8_t r2 ,uint8_t b2 ,uint8_t g2,uint8_t l1, uint8_t l2 );
void update_ORB_single(uint8_t port_no , uint8_t r , uint8_t g , uint8_t b);
void update_LEDS_single(uint8_t port_no, uint8_t led);
void switch_off_ORB_LED();

extern volatile uint8_t temp_compare_array[8];
extern volatile uint8_t temp_compare_array_2[8];
extern volatile bool update_compare_array;
extern volatile bool status_battery;

#endif /* ORB_CONTROL_H_ */