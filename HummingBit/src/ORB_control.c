/*
 * ORB_control.c
 *
 * Created: 12/8/2017 9:44:53 AM
 *  Author: Raghunath Jangam
 */ 

#include <asf.h>
#include "ORB_control.h"
#include "ORB.h"
#define PORT_CLEAR_REGISTER_ADD     0x41004414UL
#define PORT_SET_REGISTER_ADD		0x41004418UL

void update_ORB_LED(uint8_t r1 ,uint8_t g1 ,uint8_t b1 ,uint8_t r2 ,uint8_t g2 ,uint8_t b2 ,uint8_t l1 , uint8_t l4 )
{
	
 	temp_compare_array_2[0] = 255-r1;//Left -- R
 	temp_compare_array_2[1] = 255-g1;//Left  -- G
 	temp_compare_array_2[2] = 255-b1;//Left  -- B
 	temp_compare_array_2[3] = 255-r2;//Right -- R
 	temp_compare_array_2[4] = 255-g2;//Right  -- G
 	temp_compare_array_2[5] = 255-b2;//Right -- B
	temp_compare_array_2[6] = 255-l1;//led1
	if(status_battery == false)
	{
		temp_compare_array_2[7] = 255-l4;//led4 
	}  
	 
	
	initializing_pin_array();
	increasing_sort_tag();
	
	update_compare_array = true;

}


void update_ORB_single(uint8_t port_no , uint8_t r , uint8_t g , uint8_t b)
{
	volatile uint32_t* const  PORT_SET		      = PORT_SET_REGISTER_ADD;
	volatile uint32_t* const PORT_CLEAR			  = PORT_CLEAR_REGISTER_ADD;
	uint32_t B2_RGB = 0x08000000;
	
	if(port_no == '1') //ASCII 1
	{
		temp_compare_array_2[0] = 255-r;//Left -- R
		temp_compare_array_2[1] = 255-g;//Left  -- G
		temp_compare_array_2[2] = 255-b;//Left  -- B
	}
	else if(port_no == '2')//ASCII 2
	{
		temp_compare_array_2[3] = 255-r;//Right -- R
		temp_compare_array_2[4] = 255-g;//Right  -- G
		temp_compare_array_2[5] = 255-b;//Right -- B
	}
	initializing_pin_array();
	increasing_sort_tag();

	update_compare_array = true;
}

void update_LEDS_single(uint8_t port_no, uint8_t led)
{
	volatile uint32_t* const  PORT_SET		      = PORT_SET_REGISTER_ADD;
	volatile uint32_t* const PORT_CLEAR           = PORT_CLEAR_REGISTER_ADD;
	uint32_t B2_RGB = 0x08000000;
	if(led == 0xff)
	{
		led = 0xfe;
	}
	switch (port_no)
	{
		case '1':
			temp_compare_array_2[6] = 255 - led;//Led1
			break;
		case '2':
		    if(status_battery == false)
		    {
				temp_compare_array_2[7] = 255  -led;//Led2
			}
			break;
		default:
			break;
	}
	initializing_pin_array();
	increasing_sort_tag();
	update_compare_array = true;

}


void switch_off_ORB_LED()
{
	temp_compare_array_2[0] = 255;//Left  -- R
	temp_compare_array_2[1] = 255;//Left  -- G
	temp_compare_array_2[2] = 255;//Left  -- B
	temp_compare_array_2[3] = 255;//Right -- R
	temp_compare_array_2[4] = 255;//Right -- G
	temp_compare_array_2[5] = 255;//Right -- B
	temp_compare_array_2[6] = 255;//Led1
	temp_compare_array_2[7] = 255;//Led4
	initializing_pin_array();
	increasing_sort_tag();
	update_compare_array = true;
}