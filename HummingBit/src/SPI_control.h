/*
 * SPI_control.h
 *
 * Created: 2/5/2018 4:47:44 PM
 *  Author: raghu
 */ 


#ifndef SPI_CONTROL_H_
#define SPI_CONTROL_H_


#define LED1_S         0
#define LED2_S		   1
#define LED3_S         2
#define LED4_S		   3

#define RGB1_S         4


#define RGB2_S         5


#define SERVO1_S       6
#define SERVO2_S       7
#define SERVO3_S       8
#define SERVO4_S       9

#define SET_ALL		   10

#define STOP_ALL	   11
#define SET_ALL_APP	   13
#define CHANGE_STATUS  16

#define MASK_RW		  0xC0
#define MASK_MODE     0x3F

#define WRITE_SPI     0xC0
#define READ_SPI      0x80

#define LED1_NO       0x31
#define LED2_NO       0x32
#define LED3_NO       0x33
#define LED4_NO       0x34

#define RGB1_NO       0x31
#define RGB2_NO       0x32



#define SERVO1_NO     0x31
#define SERVO2_NO     0x32
#define SERVO3_NO     0x33
#define SERVO4_NO     0x34

#define SPI_LENGTH    0x04



void spi_main_loop();

extern  volatile uint8_t received_value[20];
extern  volatile uint8_t transmit_value[20];

extern volatile uint8_t sensor_outputs[20];
extern volatile bool transcation_start;
extern volatile bool serial_timeout;
extern volatile uint8_t serial_timeout_count;
extern volatile bool status_battery ;
extern volatile bool init_status_battery ;

#endif /* SPI_CONTROL_H_ */