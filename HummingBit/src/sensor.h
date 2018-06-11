/*
 * sensor.h
 *
 * Created: 12/7/2017 2:45:43 PM
 *  Author: Raghunath Jangam
 */ 


#ifndef SENSOR_H_
#define SENSOR_H_


uint16_t adc_start_read_result(const enum adc_positive_input analogPin);
void sensor_init();

void sensor_enable();
void sensor_disable();



#endif /* SENSOR_H_ */