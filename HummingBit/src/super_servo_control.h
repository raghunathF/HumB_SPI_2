/*
 * super_servo_control.h
 *
 * Created: 12/11/2017 10:02:14 AM
 *  Author: raghu
 */ 


#ifndef SUPER_SERVO_CONTROL_H_
#define SUPER_SERVO_CONTROL_H_

void update_super_servo(uint8_t servo1 , uint8_t servo2 , uint8_t servo3, uint8_t servo4);
void update_super_servo_single(uint8_t port_no ,uint8_t super_servo);
void switch_off_servos();


extern volatile uint8_t battery_voltage;


#endif /* SUPER_SERVO_CONTROL_H_ */