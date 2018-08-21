/*
 * sensor.c
 *
 * Created:		12/7/2017 2:46:00 PM
 * Author:		Raghunath Jangam
 * Description:	Configure ADC and read ADC values
 */ 
/********************************************************************************************/
#include <asf.h>
#include "sensor.h"

/********************************************************************************************/
struct adc_module adc_instance;
uint16_t* adc_result = NULL;

/********************************************************************************************
Reference is VDDANA/1.48 
Sampling time -- 8Mhz/16
Max value of ADC -- 255 
For HUmmingBird Bit board 3.3V/1.48 = 2.29
/********************************************************************************************/
void configure_adc()
{
	struct adc_config conf_adc;
	adc_get_config_defaults(&conf_adc);
	adc_result = malloc(sizeof(uint16_t));

	conf_adc.reference			= ADC_REFCTRL_REFSEL_INTVCC0;
	conf_adc.clock_prescaler	= ADC_CLOCK_PRESCALER_DIV16;
	conf_adc.positive_input		= 6;
	conf_adc.negative_input		= ADC_NEGATIVE_INPUT_GND;
	conf_adc.resolution			= ADC_RESOLUTION_8BIT;
	conf_adc.left_adjust		= true;

	adc_init(&adc_instance, ADC, &conf_adc);
	adc_enable(&adc_instance);
}


uint16_t adc_start_read_result(const enum adc_positive_input analogPin)
{	
	uint16_t temp = 0;
	adc_set_positive_input(&adc_instance, analogPin );
	adc_start_conversion(&adc_instance);
	while((adc_get_status(&adc_instance) & ADC_STATUS_RESULT_READY) != 1);
	adc_read(&adc_instance, adc_result);
	temp = *adc_result;
 	return temp;
}

void configure_adc_inputs()
{
	
	struct port_config config_port_pin;
	port_get_config_defaults(&config_port_pin);
	config_port_pin.direction  = PORT_PIN_DIR_INPUT;
	config_port_pin.input_pull = PORT_PIN_PULL_DOWN;
	port_pin_set_config(SENSOR1, &config_port_pin);
	port_pin_set_config(SENSOR2, &config_port_pin);
	port_pin_set_config(SENSOR3, &config_port_pin);	

}

void sensor_init()
{
	configure_adc();
}