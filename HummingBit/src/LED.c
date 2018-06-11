/*
 * LED.c
 *
 * Created: 12/6/2017 5:12:20 PM
 *  Author: raghu
 */ 
#include <asf.h>
#include "LED.h"

struct tc_module led_tc_instance;

#define NO_OF_LEDS 2
uint8_t compare_led_array[NO_OF_LEDS];
uint8_t compare_led_array_ID;

uint8_t pin_led_array[NO_OF_LEDS];
uint8_t pin_led_array_ID;

uint8_t temp_compare_led_array[NO_OF_LEDS];
uint8_t temp_compare_led_array_2[NO_OF_LEDS];
uint8_t temp_pin_led_array[NO_OF_LEDS];

bool update_compare_led_array = false;

static uint8_t N_valid_led_compares = NO_OF_LEDS;

#define CLEAR_LEDS 0xC0000000 //31,30

void LED_timer_init()
{
	struct tc_config led_tc_config;
	tc_get_config_defaults(&led_tc_config);
	led_tc_config.clock_prescaler = TC_CLOCK_PRESCALER_DIV256;
	led_tc_config.counter_size    = TC_COUNTER_SIZE_8BIT;
	led_tc_config.counter_8_bit.period = 0XFF;
	led_tc_config.counter_8_bit.compare_capture_channel[0] = 0;
	
	tc_init(&led_tc_instance, TC2, &led_tc_config);
	tc_enable(&led_tc_instance);
}

void set_drivestrength_LED()
{
	PortGroup *const port_base = port_get_group_from_gpio_pin(LED1);
	system_pinmux_group_set_output_strength(port_base,CLEAR_LEDS,SYSTEM_PINMUX_PIN_STRENGTH_HIGH);
}

void LEDS_off()
{
	//Switch off LEDs single statement
	PortGroup *const port_base = port_get_group_from_gpio_pin(LED1);
	port_base->OUTCLR.reg = CLEAR_LEDS ;
	
	//port_pin_set_output_level(LED3,LED_ON);
	
}


void LED_setup_pins()
{
	struct port_config config_port_pin;
	port_get_config_defaults(&config_port_pin);
	config_port_pin.direction  = PORT_PIN_DIR_OUTPUT;
	port_pin_set_config(LED1, &config_port_pin);
	port_pin_set_config(LED4, &config_port_pin);
	set_drivestrength_LED();
	LEDS_off();	
}

void enable_LED()
{
	tc_enable_callback(&led_tc_instance, TC_CALLBACK_OVERFLOW);
	tc_enable_callback(&led_tc_instance, TC_CALLBACK_CC_CHANNEL0);
}

void disable_LED()
{
	//clear all LEDS
	LEDS_off();
	tc_disable_callback(&led_tc_instance, TC_CALLBACK_OVERFLOW);
	tc_disable_callback(&led_tc_instance, TC_CALLBACK_CC_CHANNEL0);
	
}

void initializing_LED_pin_array()
{
	temp_pin_led_array[0]  = LED1;
	temp_pin_led_array[1]  = LED4;
}

void initializing_LED_compare_array()
{
	temp_compare_led_array_2[0] = 255;//LED1
	temp_compare_led_array_2[1] = 255;//LED2

}

void transfer_temp_LED_2()
{
	uint8_t i =0;
	for(i=0;i<=NO_OF_LEDS-1;i++)
	{
		temp_compare_led_array[i] = temp_compare_led_array_2[i];
	}
}
void increasing_LED_sort_tag()
{
	uint8_t i,j ,temp;
	uint8_t N=NO_OF_LEDS;
	transfer_temp_LED_2();
	for(i=0; i< N-1 ;i++)
	{
		for(j=0;j< N-i-1;j++)
		{
			if(temp_compare_led_array[j]>temp_compare_led_array[j+1])
			{
				temp = temp_compare_led_array[j];
				temp_compare_led_array[j] = temp_compare_led_array[j+1];
				temp_compare_led_array[j+1]= temp;
				
				temp = temp_pin_led_array[j];
				temp_pin_led_array[j] = temp_pin_led_array[j+1];
				temp_pin_led_array[j+1] = temp;
				
			}
		}
	}
}

void LED_transfer_temp()
{
	uint8_t i;
	N_valid_led_compares = 0;
	for(i=0;i<=NO_OF_LEDS-1;i++)
	{
		if(temp_compare_led_array[i] != 255)
		{
			N_valid_led_compares++;
		}
		compare_led_array[i] = temp_compare_led_array[i] ;
		pin_led_array[i]  = temp_pin_led_array[i];
	}
}


void LED_init_array()
{

	initializing_LED_pin_array();
	initializing_LED_compare_array();
	increasing_LED_sort_tag();
	LED_transfer_temp();

}


void tc_callback_LED_OF(struct tc_module *const module_inst)
{
	uint8_t compare_value=0;
	static bool led_disable_flag = false;
	serial_timeout_count++;
	count_broadcast++;
	if(serial_timeout_count > MAX_SERIAL_TIMEOUT)
	{
		serial_timeout = true;
		serial_timeout_count = 0;
	}
	
	LEDS_off();
	
	if(update_compare_led_array == true)
	{
		
		LED_transfer_temp();
		if(led_disable_flag == true)
		{
			tc_enable_callback(&led_tc_instance, TC_CALLBACK_CC_CHANNEL0);
			tc_clear_status(&led_tc_instance,0x00000011);
			led_disable_flag = false;
		}
		update_compare_led_array = false;
	}
	compare_led_array_ID = 0;
	pin_led_array_ID  = 0;
	
	compare_value = compare_led_array[0];
	if(compare_value != 255)
	{
		tc_set_compare_value(module_inst, TC_COMPARE_CAPTURE_CHANNEL_0, compare_value);
	}
	else
	{
		led_disable_flag = true;
		tc_disable_callback(&led_tc_instance, TC_CALLBACK_CC_CHANNEL0);
	}
	
}

void tc_callback_LED_PWM(struct tc_module *const module_inst)
{
	static uint8_t compare_value=0;
	static uint8_t compare_value_last=0;
	static uint8_t compare_value_current=0;
	static bool first_time = true;
	
	if(first_time == false)
	{
		port_pin_set_output_level(pin_led_array[pin_led_array_ID++], LED_ON);
		
		if(compare_led_array_ID < N_valid_led_compares-1 )
		{
			compare_value_last = compare_led_array[compare_led_array_ID];
			compare_value_current = compare_led_array[++compare_led_array_ID];
			while(compare_value_last == compare_value_current && compare_led_array_ID <= N_valid_led_compares - 1)
			{
				//Enable the LED
				port_pin_set_output_level(pin_led_array[pin_led_array_ID++],LED_ON);
				compare_value_last = compare_led_array[compare_led_array_ID];
				compare_value_current = compare_led_array[++compare_led_array_ID];
			}
			if(compare_value_current != 255)
			{
				tc_set_count_value(module_inst, compare_value_last);
				tc_set_compare_value(module_inst, TC_COMPARE_CAPTURE_CHANNEL_0, compare_value_current);
			}
			else
			{
				tc_set_count_value(module_inst, compare_value_last);
			}
		}
		
	}
	else
	{
		first_time = false;
	}
}


void LED_timer_callbacks_init()
{
	tc_register_callback(&led_tc_instance, tc_callback_LED_OF,TC_CALLBACK_OVERFLOW);
	tc_register_callback(&led_tc_instance, tc_callback_LED_PWM,TC_CALLBACK_CC_CHANNEL0);
	
}


void LED_init()
{
	LED_timer_init();
	LED_setup_pins();
	LED_init_array();
	//Initialize the timer callbacks
	LED_timer_callbacks_init();
}