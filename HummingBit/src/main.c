/************************************************************************/
/*
*Author      : Raghunath Jangam                                         
*Last edit   : 7/18/2018
*Description : Main program where all global variables are declared, where
			   the program is divided into two parts SPI check and sensor
			   check
*/
/************************************************************************/
#include <asf.h>
#include "ORB.h"
#include "super_servo.h"
#include "sensor_control.h"
#include "sensor.h"
#include "SPI_slave.h"
#include "SPI_control.h"
#include "ORB_control.h"
/************************************************************************/

#define SPI_DATA_LENGTH 20

/************************************************************************/
//120

//SPI related  global variables
volatile bool spi_reset_1 = false;
volatile uint8_t sensor_outputs[20];
volatile uint8_t transmit_value[SPI_DATA_LENGTH];
volatile uint8_t temp_receive[SPI_DATA_LENGTH];
volatile uint8_t received_value[SPI_DATA_LENGTH];
volatile bool transfer_complete_spi_slave = false;

//Serial timeout global variables
volatile uint8_t serial_timeout_count = 0;
volatile bool	 serial_timeout = false;
volatile bool	 transcation_start = false;

//LED data transfer global variables
volatile uint8_t temp_compare_array_2[8];
volatile uint8_t temp_compare_array[8];

volatile uint8_t temp_pin_array[8]; 
volatile uint8_t temp_pin_array_2[8]; 
volatile bool	 update_compare_array = false;  

volatile bool	 lock_temp_array = false;

//Voltage monitoring global variables

/************************************************************************/
volatile uint8_t battery_voltage = 0;
volatile bool	 status_battery = true;
volatile bool    init_status_battery = true ;


volatile bool firmware_check = false;

/************************************************************************/

//Read sensors very frequently 
void sensor_check()
{
	read_all_sensors();
}


int main (void)
{
	//Clock , external interrupts initialization
	system_init();
	delay_init();
	//RGB , LED1, LED4 initialization
	ORB_init();
	//ADC initialization
	sensor_init();
	//Servo port TCC module initialization
	super_servo_init();
	//Starting the interrupts for TCC module
	enable_super_servo();
	//Starting the interrupts for TC module
	enable_ORB();
	//SPI module initialization as slave
	spi_slave_init();
	
	while(1)
	{
		sensor_check();
		spi_main_loop();    // Interprets command and assigns it to the right function 
	}
}
