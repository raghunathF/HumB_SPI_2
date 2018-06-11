/*
 * super_servo.h
 *
 * Created: 12/7/2017 11:58:21 AM
 *  Author: raghu
 */ 


#ifndef SUPER_SERVO_H_
#define SUPER_SERVO_H_

void super_servo_init();
void enable_super_servo();
void disable_super_servo();

struct tcc_module tcc_ss_instance0;


#define SERVO_1_CH 0
#define SERVO_2_CH 1
#define SERVO_3_CH 2
#define SERVO_4_CH 3


#define SERVO_1_WO 0
#define SERVO_2_WO 1
#define SERVO_3_WO 2
#define SERVO_4_WO 3

#endif /* SUPER_SERVO_H_ */