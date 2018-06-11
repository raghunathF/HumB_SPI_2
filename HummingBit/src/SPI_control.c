/*
 * SPI_control.c
 *
 * Created: 2/5/2018 4:46:53 PM
 *  Author: Raghunath Jangam
 */ 

#include <asf.h>
#include "SPI_control.h"
#include "SPI_slave.h"
#include "ORB_control.h"
#include "sensor_control.h"
#include "super_servo_control.h"

#define   APP

void check_timeout()
{
	if(transcation_start == true)
	{
		if(serial_timeout == true)
		{
			serial_timeout_count = 0;
			serial_timeout = false;
			transcation_start = false;
			spi_reset(&spi_slave_instance);
			spi_slave_init();
			spi_transceive_buffer_job(&spi_slave_instance, sensor_outputs, received_value,SPI_LENGTH);
		}
	}
	
}




void spi_main_loop()
{
	
	volatile enum status_code error_code = 0x10;
	volatile static uint16_t count_buffer = 0;

	uint8_t i    = 0;
	uint8_t rw   = 0;
	uint8_t mode = 0;
	static bool test = true;
	transmit_value[0] = 0x88;
	transmit_value[1] = 0xAA;
	transmit_value[2] = 0xBB;
	transmit_value[3] = 0xCC;
	check_timeout();
	if(transfer_complete_spi_slave == true)
	{
		rw   = temp_receive[0] & MASK_RW ;
		mode = temp_receive[0] & MASK_MODE;
		if(rw == WRITE_SPI)
		{	
			switch(mode)
			{
				case LED1_S:
					update_LEDS_single(LED1_NO,temp_receive[1]);
					break;
					
				case LED4_S:
					update_LEDS_single(LED2_NO,temp_receive[1]);
					break;
				
				case RGB1_S:
				    update_ORB_single(RGB1_NO , temp_receive[1] , temp_receive[2] , temp_receive[3]);
					break;
				
				case RGB2_S:
					update_ORB_single(RGB2_NO , temp_receive[1] , temp_receive[2] , temp_receive[3]);
					break;
					
				case SERVO1_S:
					update_super_servo_single(SERVO1_NO,temp_receive[1]);
					break;
					
				case SERVO2_S:
					update_super_servo_single(SERVO2_NO,temp_receive[1]);
					break;

				case SERVO3_S:
					update_super_servo_single(SERVO3_NO,temp_receive[1]);
					break;

				case SERVO4_S:
					update_super_servo_single(SERVO4_NO,temp_receive[1]);
					break;
					
				case SET_ALL:
					update_ORB_LED(temp_receive[3],temp_receive[4] ,temp_receive[5] ,temp_receive[6] ,temp_receive[7] ,temp_receive[8],temp_receive[1],temp_receive[2]);
					update_super_servo(temp_receive[9] , temp_receive[10] , temp_receive[11], temp_receive[12]);
					break;
				
				case STOP_ALL:
					switch_off_ORB_LED();
					switch_off_servos();
					break;
				
				/*
				case CHANGE_STATUS:
					status_battery = ~status_battery;
					init_status_battery = true;
					break;
				*/
				
				default:
					break;
			}
			
		}
		else if(rw == READ_SPI)
		{
			
		}
		check_buffer();		
	}
	
}		
	