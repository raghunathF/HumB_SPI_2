/*
 * ORB.c
 *
 * Created: 12/6/2017 11:20:25 AM
 *  Author: raghu
 */ 

#include <asf.h>
#include "ORB.h"

struct tc_module orb_tc_instance;

#define NO_OF_LEDS 8
extern volatile uint8_t temp_compare_array_2[NO_OF_LEDS];
extern volatile uint8_t temp_compare_array[NO_OF_LEDS];

extern volatile  uint8_t temp_pin_array[NO_OF_LEDS]; 
extern volatile  uint8_t temp_pin_array_2[NO_OF_LEDS]; 

extern volatile  bool update_compare_array;  //Should be declared in Main.c

extern volatile bool lock_temp_array;
uint8_t test_compare_array[] = {2,3,4,5,6,7,8,9};
volatile uint8_t  k =0;

#define PORT_CLEAR_REGISTER_ADD     0x41004414UL
#define PORT_SET_REGISTER_ADD		0x41004418UL

#define CLEAR_ORB_LEDS	            0xC8038300UL //15,16,17,8,9,27,31,30


void increasing_sort_tag()
{
	volatile uint32_t* const  PORT_SET		      = PORT_SET_REGISTER_ADD;
	volatile uint32_t* const PORT_CLEAR			  = PORT_CLEAR_REGISTER_ADD;
	uint32_t B2_RGB = 0x08000000;
	uint8_t i,j ,temp;
	uint8_t N = NO_OF_LEDS;
	uint8_t temp_temp_compare_array[N];
	uint8_t temp_temp_pin_array[N];
	for(i=0;i<NO_OF_LEDS;i++)
	{
		temp_temp_pin_array[i] = temp_pin_array_2[i] ;
		temp_temp_compare_array[i] = temp_compare_array_2[i] ;
	}
	for(i=0; i< N-1 ;i++)
	{
		for(j=0;j< N-i-1;j++)
		{
			if(temp_temp_compare_array[j]>temp_temp_compare_array[j+1])
			{
				temp = temp_temp_compare_array[j];
				temp_temp_compare_array[j] = temp_temp_compare_array[j+1];
				temp_temp_compare_array[j+1]= temp;
				
				temp = temp_temp_pin_array[j];
				temp_temp_pin_array[j]   = temp_temp_pin_array[j+1];
				temp_temp_pin_array[j+1] = temp;
			}
		}
	}
	lock_temp_array = true;
	for(i=0;i<N;i++)
	{
		
		temp_pin_array[i]     = temp_temp_pin_array[i]  ;
		temp_compare_array[i] = temp_temp_compare_array[i];
	}
	lock_temp_array = false;
}


void ORB_leds_off()
{
	//Switch off LEDs single statement
	PortGroup *const port_base = port_get_group_from_gpio_pin(ORB_R1);
	port_base->OUTCLR.reg	   = CLEAR_ORB_LEDS;
}


void transfer_temp_2()
{
	uint8_t i;
	for(i=0;i<NO_OF_LEDS;i++)
	{
		temp_compare_array[i] = temp_compare_array_2[i] ;
	}
}

void ORB_timer_init()
{
	struct tc_config orb_tc_config;
	tc_get_config_defaults(&orb_tc_config);
	//rgb_led_config.clock_source = GCLK_GENERATOR_1;
	orb_tc_config.clock_prescaler = TC_CLOCK_PRESCALER_DIV256;
	orb_tc_config.counter_size = TC_COUNTER_SIZE_8BIT;
	orb_tc_config.counter_8_bit.period = 0XFF;
	orb_tc_config.counter_8_bit.compare_capture_channel[0] = 0;
	
	tc_init(&orb_tc_instance, TC1, &orb_tc_config);
	tc_enable(&orb_tc_instance);
}

void set_drivestrength_ORB()
{
	PortGroup *const port_base = port_get_group_from_gpio_pin(ORB_R1);
	system_pinmux_group_set_output_strength(port_base,CLEAR_ORB_LEDS,SYSTEM_PINMUX_PIN_STRENGTH_HIGH);
}

void ORB_setup_pins()
{
	struct port_config config_port_pin;
	port_get_config_defaults(&config_port_pin);
	config_port_pin.direction  = PORT_PIN_DIR_OUTPUT;
	port_pin_set_config(ORB_R1, &config_port_pin);
	port_pin_set_config(ORB_G1, &config_port_pin);
	port_pin_set_config(ORB_B1, &config_port_pin);
	port_pin_set_config(ORB_R2, &config_port_pin);
	port_pin_set_config(ORB_G2, &config_port_pin);
	port_pin_set_config(ORB_B2, &config_port_pin);
	port_pin_set_config(LED1, &config_port_pin);
	port_pin_set_config(LED4, &config_port_pin);
	set_drivestrength_ORB();
	ORB_leds_off();
	
}



void tc_callback_PWM(struct tc_module *const module_inst)
{
}

void tc_callback_OF(struct tc_module *const module_inst)
{
}

void ORB_timer_callbacks_init()
{
	tc_register_callback(&orb_tc_instance, tc_callback_OF,TC_CALLBACK_OVERFLOW);
	tc_register_callback(&orb_tc_instance, tc_callback_PWM,TC_CALLBACK_CC_CHANNEL0);
}

void enable_ORB()
{
	tc_enable_callback(&orb_tc_instance, TC_CALLBACK_OVERFLOW);
	tc_enable_callback(&orb_tc_instance, TC_CALLBACK_CC_CHANNEL0);
}

void disable_ORB()
{
	//clear all RGB LEDS
	ORB_leds_off();
	tc_disable_callback(&orb_tc_instance, TC_CALLBACK_OVERFLOW);
	tc_disable_callback(&orb_tc_instance, TC_CALLBACK_CC_CHANNEL0);
}

void initializing_pin_array()
{
	temp_pin_array_2[0]  = ORB_R1;
	temp_pin_array_2[1]  = ORB_G1;
	temp_pin_array_2[2]  = ORB_B1;
	temp_pin_array_2[3]  = ORB_R2;
	temp_pin_array_2[4]  = ORB_G2;
	temp_pin_array_2[5]  = ORB_B2;
	temp_pin_array_2[6]  = LED1;
	temp_pin_array_2[7]  = LED4;
}

void initializing_compare_array()
{
	temp_compare_array_2[0] = 255;//Left  -- R
	temp_compare_array_2[1] = 255;//Left  -- G
	temp_compare_array_2[2] = 255;//Left  -- B
	
	temp_compare_array_2[3] = 255;//Right -- R
	temp_compare_array_2[4] = 255;//Right -- G
	temp_compare_array_2[5] = 255;//Right -- B
	
	temp_compare_array_2[6] =  255;//LED1
	temp_compare_array_2[7] =  255;//LED4
}


void ORB_init_array()
{
	initializing_pin_array();
	initializing_compare_array();
	increasing_sort_tag();
}

void ORB_init()
{
	//Timer Initialization
	ORB_timer_init();
	ORB_setup_pins();
	ORB_init_array();
	//Initialize the timer callbacks
	ORB_timer_callbacks_init();

}