/************************************************************************/
/*
*Author      : Raghunath Jangam                                         
*Date        : 
*Description :
*/
/************************************************************************/

/*
 * Include header files for all drivers that have been imported from
 * Atmel Software Framework (ASF).
 */
/*
 * Support and FAQ: visit <a href="http://www.atmel.com/design-support/">Atmel Support</a>
 */
#include <asf.h>
#include "ORB.h"
#include "super_servo.h"
#include "sensor_control.h"
#include "sensor.h"
#include "test.h"
#include "SPI_slave.h"
#include "SPI_control.h"
#include "ORB_control.h"
#define SPI_DATA_LENGTH 20

volatile bool spi_reset_1 = false;

volatile uint8_t sensor_outputs[20];

uint8_t* global_sensor_value = NULL;


volatile uint8_t transmit_value[SPI_DATA_LENGTH];
volatile uint8_t temp_receive[SPI_DATA_LENGTH];
volatile uint8_t received_value[SPI_DATA_LENGTH];
volatile bool transfer_complete_spi_slave = false;


volatile uint8_t serial_timeout_count = 0;
volatile bool	 serial_timeout = false;
volatile bool	 transcation_start = false;
volatile uint8_t count_broadcast = 0;


volatile uint8_t temp_compare_array_2[8];
volatile uint8_t temp_compare_array[8];

volatile uint8_t temp_pin_array[8]; 
volatile uint8_t temp_pin_array_2[8]; 
volatile bool	 update_compare_array = false;  

volatile bool	 lock_temp_array = false;

volatile uint8_t battery_voltage = 0;
volatile bool	 status_battery = true;
volatile bool    init_status_battery = true ;

//Should be included
volatile bool flash_status_LED = true; 
volatile bool firmware_check = false;


void load_input()
{
	read_all_sensors();	
}


void sensor_check()
{
	//if(firmware_check == false)
	//{
		read_all_sensors();
	//}
	
}


void microbit_connection()
{
	static bool status_LED_on = false;
	static bool status_LED_off = false;
	if(flash_status_LED == true)
	{
		if (status_LED_on == false)
		{
			update_LEDS_single(0x32, 0x55);
			status_LED_on = true;
		}
	}
	else
	{
		if (status_LED_off == false)
		{
			update_LEDS_single(0x32, 0x00);
			status_LED_off = true;
		}
	}
}

int main (void)
{
	system_init();
	delay_init();
	ORB_init();
	//microbit_connection();
	sensor_init();
	super_servo_init();
	enable_super_servo();
	enable_ORB();
	spi_slave_init();
	
	/* Insert application code here, after the board has been initialized. */
	while(1)
	{
		//microbit_connection();
		sensor_check();
		spi_main_loop();
		//test_servos();
	}
}
