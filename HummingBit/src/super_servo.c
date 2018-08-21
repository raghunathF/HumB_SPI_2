/*
 * servo.c
 *
 * Created: 12/7/2017 11:56:55 AM
 *  Author: raghu
 */ 
/********************************************************************************************/
#include <asf.h>
#include "super_servo.h"

/********************************************************************************************/
extern volatile uint8_t sensor_outputs[20];
struct tcc_module tcc_ss_instance0;

/********************************************************************************************/
void enable_super_servo()
{
	tcc_enable(&tcc_ss_instance0); //Enable the TCC module
}

void disable_super_servo()
{
	tcc_disable(&tcc_ss_instance0); //Disable the TCC module
}

/********************************************************************************************
Period  ---  8Mhz / (4*40000) = 500Hz = 2msec
Single slope |\	  |\
			 | \  | \
			 |  \ |  \
			 |   \|   \
TCC0 module
Four channels are used
********************************************************************************************/
void super_servo_tcc_init()
{
	struct tcc_config config_tcc_ss;
	tcc_get_config_defaults(&config_tcc_ss, TCC0);
	
	
	config_tcc_ss.counter.clock_prescaler = TCC_CLOCK_PRESCALER_DIV4;
	config_tcc_ss.counter.period = 0x9C40; //40000
	config_tcc_ss.compare.wave_generation = TCC_WAVE_GENERATION_SINGLE_SLOPE_PWM;
	
	config_tcc_ss.compare.wave_polarity[SERVO_1_CH] = TCC_WAVE_POLARITY_0;
	config_tcc_ss.compare.wave_polarity[SERVO_2_CH] = TCC_WAVE_POLARITY_0;
	config_tcc_ss.compare.wave_polarity[SERVO_3_CH] = TCC_WAVE_POLARITY_0;
	config_tcc_ss.compare.wave_polarity[SERVO_4_CH] = TCC_WAVE_POLARITY_0;
	
	
	config_tcc_ss.compare.match[SERVO_1_CH] = 0;
	config_tcc_ss.pins.enable_wave_out_pin[SERVO_1_WO] = true;
	config_tcc_ss.pins.wave_out_pin[SERVO_1_WO]        = PIN_PA04F_TCC0_WO0; 
	config_tcc_ss.pins.wave_out_pin_mux[SERVO_1_WO]    = PINMUX_PA04F_TCC0_WO0; 
	
	
	config_tcc_ss.compare.match[SERVO_2_CH] = 0;
	config_tcc_ss.pins.enable_wave_out_pin[SERVO_2_WO] = true;
	config_tcc_ss.pins.wave_out_pin[SERVO_2_WO]        = PIN_PA05F_TCC0_WO1; 
	config_tcc_ss.pins.wave_out_pin_mux[SERVO_2_WO]    = PINMUX_PA05F_TCC0_WO1; 
	
	
	config_tcc_ss.compare.match[SERVO_3_CH] = 0;
	config_tcc_ss.pins.enable_wave_out_pin[SERVO_3_WO] = true;
	config_tcc_ss.pins.wave_out_pin[SERVO_3_WO]        = PIN_PA06F_TCC0_WO2; 
	config_tcc_ss.pins.wave_out_pin_mux[SERVO_3_WO]    = PINMUX_PA06F_TCC0_WO2; 
	
	
	config_tcc_ss.compare.match[SERVO_4_CH] = 0;
	config_tcc_ss.pins.enable_wave_out_pin[SERVO_4_WO] = true;
	config_tcc_ss.pins.wave_out_pin[SERVO_4_WO]        = PIN_PA07F_TCC0_WO3; 
	config_tcc_ss.pins.wave_out_pin_mux[SERVO_4_WO]    = PINMUX_PA07F_TCC0_WO3; 
	
	tcc_init(&tcc_ss_instance0, TCC0, &config_tcc_ss);
	
}

void super_servo_init()
{
	super_servo_tcc_init();
}
