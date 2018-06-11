/*
 * pinout.h
 *
 * Created: 12/6/2017 10:52:22 AM
 *  Author: Raghunath Jangam
 */ 


#ifndef PINOUT_H_
#define PINOUT_H_

//RGB --2

//ORB--LED1
#define ORB_R1 PIN_PA15 
#define ORB_G1 PIN_PA16 
#define ORB_B1 PIN_PA17 

//ORB--LED2
#define ORB_R2 PIN_PA08 
#define ORB_G2 PIN_PA09 
#define ORB_B2 PIN_PA27


//LEDs --3
#define LED1 PIN_PA31
#define LED4 PIN_PA30

//Servo -- 4
#define SERVO_1 PIN_PA04
#define SERVO_2 PIN_PA05
#define SERVO_3 PIN_PA06
#define SERVO_4 PIN_PA07

//Sensors
#define SENSOR1 PIN_PA14
#define SENSOR2 PIN_PA11
#define SENSOR3 PIN_PA10

//UART//SERCOM-1, pad-2,pad-3
#define TX PIN_PA24
#define RX PIN_PA25

//Voltage-Monitoring
#define VOLTAGE_MTR PIN_PA02



#endif /* PINOUT_H_ */