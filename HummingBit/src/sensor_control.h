/*
 * sensor_control.h
 *
 * Created: 12/7/2017 3:33:47 PM
 * Author:  Raghunath Jangam
 */ 


#ifndef SENSOR_CONTROL_H_
#define SENSOR_CONTROL_H_

extern volatile uint8_t battery_voltage;
void read_all_sensors();
uint8_t read_sensor(uint8_t sensor_no);
extern volatile uint8_t sensor_outputs[20];
extern volatile bool firmware_check ;


#define SENSOR1_AI  6
#define SENSOR2_AI  9
#define SENSOR3_AI  8
#define VOLTAGE_MTR_AI 0 

#define SENSORS_AI {SENSOR1_AI ,SENSOR2_AI, SENSOR3_AI, VOLTAGE_MTR_AI};

#define ALL_SENSORS {SENSOR1, SENSOR2 , SENSOR3 ,VOLTAGE_MTR} 
#define NO_SENSORS 4

#endif /* SENSOR_CONTROL_H_ */