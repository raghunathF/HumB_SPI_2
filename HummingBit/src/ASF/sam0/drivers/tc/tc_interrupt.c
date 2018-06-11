/**
 * \file
 *
 * \brief SAM TC - Timer Counter Callback Driver
 *
 * Copyright (C) 2013-2016 Atmel Corporation. All rights reserved.
 *
 * \asf_license_start
 *
 * \page License
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice,
 *    this list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 *
 * 3. The name of Atmel may not be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 * 4. This software may only be redistributed and used in connection with an
 *    Atmel microcontroller product.
 *
 * THIS SOFTWARE IS PROVIDED BY ATMEL "AS IS" AND ANY EXPRESS OR IMPLIED
 * WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
 * MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NON-INFRINGEMENT ARE
 * EXPRESSLY AND SPECIFICALLY DISCLAIMED. IN NO EVENT SHALL ATMEL BE LIABLE FOR
 * ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS
 * OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT,
 * STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 * \asf_license_stop
 *
 */

/*
 * Support and FAQ: visit <a href="http://www.atmel.com/design-support/">Atmel Support</a>
 */
#include <asf.h>
#include "tc_interrupt.h"

void *_tc_instances[TC_INST_NUM];

void _tc_interrupt_handler(uint8_t instance);

#define CLEAR_ORB					0x08038300UL //15,16,17,8,9,27
#define CLEAR_ORB_LEDS	            0x88038300UL //15,16,17,8,9,27,31

#define STATUS_REGISTER_ADD			0x4200180FUL
#define COMPARE_REGISTER_ADD        0x42001818UL
#define MASK_SYNC					0x80UL

#define COUNT_REGISTER_ADD			0x42001810UL

#define PORT_CLEAR_REGISTER_ADD     0x41004414UL
#define PORT_SET_REGISTER_ADD		0x41004418UL
#define PORT_TOGGLE_REGISTER_ADD    0x4100441CUL

#define STATUS_LED_PIN              0x40000000UL


#define NO_OF_LEDS                  8

#define MAX_SERIAL_TIMEOUT          2
#define MAX_STATUS_BLINK            65
#define MIN_BLINK_START             120
#define MIN_THRESHOLD_COUNT         106
#define BLINK_MAX                   0x00

volatile uint8_t N_valid_compares = NO_OF_LEDS;
extern volatile bool update_compare_array;
extern volatile uint8_t temp_compare_array[NO_OF_LEDS];
extern volatile uint8_t temp_pin_array[NO_OF_LEDS];
extern volatile bool lock_temp_array;

extern volatile uint8_t serial_timeout_count;
extern volatile bool serial_timeout;
extern volatile uint8_t battery_voltage;
extern volatile bool status_battery ;
extern volatile bool init_status_battery ;

/**
 * \brief Registers a callback.
 *
 * Registers a callback function which is implemented by the user.
 *
 * \note The callback must be enabled by \ref tc_enable_callback,
 * in order for the interrupt handler to call it when the conditions for the
 * callback type is met.
 *
 * \param[in]     module        Pointer to TC software instance struct
 * \param[in]     callback_func Pointer to callback function
 * \param[in]     callback_type Callback type given by an enum
 */
enum status_code tc_register_callback(
		struct tc_module *const module,
		tc_callback_t callback_func,
		const enum tc_callback callback_type)
{
	/* Sanity check arguments */
	Assert(module);
	Assert(callback_func);

	/* Register callback function */
	module->callback[callback_type] = callback_func;

	/* Set the bit corresponding to the callback_type */
	if (callback_type == TC_CALLBACK_CC_CHANNEL0) {
		module->register_callback_mask |= TC_INTFLAG_MC(1);
	}
	else if (callback_type == TC_CALLBACK_CC_CHANNEL1) {
		module->register_callback_mask |= TC_INTFLAG_MC(2);
	}
	else {
		module->register_callback_mask |= (1 << callback_type);
	}
	return STATUS_OK;
}

/**
 * \brief Unregisters a callback.
 *
 * Unregisters a callback function implemented by the user. The callback should be
 * disabled before it is unregistered.
 *
 * \param[in]     module        Pointer to TC software instance struct
 * \param[in]     callback_type Callback type given by an enum
 */
enum status_code tc_unregister_callback(
		struct tc_module *const module,
		const enum tc_callback callback_type)
{
	/* Sanity check arguments */
	Assert(module);

	/* Unregister callback function */
	module->callback[callback_type] = NULL;

	/* Clear the bit corresponding to the callback_type */
	if (callback_type == TC_CALLBACK_CC_CHANNEL0) {
		module->register_callback_mask &= ~TC_INTFLAG_MC(1);
	}
	else if (callback_type == TC_CALLBACK_CC_CHANNEL1) {
		module->register_callback_mask &= ~TC_INTFLAG_MC(2);
	}
	else {
		module->register_callback_mask &= ~(1 << callback_type);
	}
	return STATUS_OK;
}

/**
 * \internal ISR handler for TC
 *
 * Auto-generate a set of interrupt handlers for each TC in the device.
 */
#define _TC_INTERRUPT_HANDLER(n, m) \
		void TC##n##_Handler(void) \
		{ \
			_tc_interrupt_handler(m); \
		}

#if (SAML21E) || (SAML21G) || (SAMR30E) || (SAMR30G)
	_TC_INTERRUPT_HANDLER(0,0)
	_TC_INTERRUPT_HANDLER(1,1)
	_TC_INTERRUPT_HANDLER(4,2)
#else
	MRECURSION(TC_INST_NUM, _TC_INTERRUPT_HANDLER, TC_INST_MAX_ID)
#endif


/**
 * \internal Interrupt Handler for TC module
 *
 * Handles interrupts as they occur, it will run the callback functions
 * that are registered and enabled.
 *
 * \param[in]  instance  ID of the TC instance calling the interrupt
 *                       handler
 */
void _tc_interrupt_handler(
		uint8_t instance)
{
	volatile uint8_t*  const  COUNT_REGISTER      = COUNT_REGISTER_ADD;
	volatile uint32_t* const PORT_CLEAR_REGISTER  = PORT_CLEAR_REGISTER_ADD;
	volatile uint8_t* const STATUS_REGISTER       = STATUS_REGISTER_ADD;
	volatile uint8_t* const COMPARE_REGISTER	  = COMPARE_REGISTER_ADD;
	volatile uint32_t* const  PORT_SET		      = PORT_SET_REGISTER_ADD;
	volatile uint32_t* const  PORT_TOGGLE		  = PORT_TOGGLE_REGISTER_ADD;
	/* Temporary variable */
	uint8_t interrupt_and_callback_status_mask;
	uint8_t i =0;
	static bool int_enable = false;
	static bool led_disable_flag = false;
	uint32_t B2_RGB = 0x08000000;
	uint32_t G2_RGB = 0x00000200;
	static uint8_t compare_value=0;
	static uint8_t compare_value_last=0;
	static uint8_t compare_value_current=0;
	static bool first_time		  = true;
	static volatile uint8_t compare_array[NO_OF_LEDS];
	static volatile uint8_t compare_array_ID;
	static volatile uint8_t pin_array[NO_OF_LEDS];
	static volatile uint8_t pin_array_ID;
	static uint8_t status_update_count = 0;
	static uint8_t flexible_count  = MAX_STATUS_BLINK;
	static uint8_t threshold_blink = MIN_BLINK_START;
	
 



	/* Get device instance from the look-up table */
	struct tc_module *module = (struct tc_module *)_tc_instances[instance];

	/* Read and mask interrupt flag register */
	interrupt_and_callback_status_mask = module->hw->COUNT8.INTFLAG.reg & module->register_callback_mask & module->enable_callback_mask;
			
	/* Check if an Match/Capture Channel 0 interrupt has occurred */
	if (interrupt_and_callback_status_mask & TC_INTFLAG_MC(1)) {
		/* Invoke registered and enabled callback function */
		//(module->callback[TC_CALLBACK_CC_CHANNEL0])(module);
		/* Clear interrupt flag */
		//module->hw->COUNT8.INTFLAG.reg = TC_INTFLAG_OVF;
		
		if(first_time == false)
		{
			if(compare_array_ID != N_valid_compares)
			{
				//port_pin_set_output_level(LED1, RGB_ON);
				//port_pin_set_output_level(pin_array[pin_array_ID++], RGB_ON);
				*PORT_SET		 = (1UL << pin_array[pin_array_ID++] ) ;
				compare_array_ID = compare_array_ID + 1;
			}
			
			if(compare_array_ID < N_valid_compares )
			{
				compare_value_last    = compare_array[compare_array_ID - 1];
				compare_value_current = compare_array[compare_array_ID];
				while((compare_value_last == compare_value_current) && (compare_array_ID <= N_valid_compares - 1))
				{
					//Enable the LED
					//port_pin_set_output_level(LED1, RGB_OFF);
					//port_pin_set_output_level(pin_array[pin_array_ID++],RGB_ON);
					*PORT_SET		 = (1UL << pin_array[pin_array_ID++] ) ;
					compare_value_last	  = compare_array[compare_array_ID];
					compare_value_current = compare_array[++compare_array_ID];
				}
				if(compare_value_current != 255)
				{
					//check sync
					while((*STATUS_REGISTER && MASK_SYNC) == true);
					*COMPARE_REGISTER         =	compare_value_current;
					*COUNT_REGISTER           =	compare_value_last;			
					//while((*STATUS_REGISTER && MASK_SYNC) == true);
					//tc_set_count_value(module_inst, compare_value_last);
					//tc_set_compare_value(module_inst, TC_COMPARE_CAPTURE_CHANNEL_0, compare_value_current);
				}
				else
				{
					while((*STATUS_REGISTER && MASK_SYNC) == true);
					*COUNT_REGISTER         =	compare_value_last;
					//tc_set_count_value(module_inst, compare_value_last);
				}
			}
			
			
		}
		else
		{
			first_time = false;
			
		}
		module->hw->COUNT8.INTFLAG.reg = TC_INTFLAG_MC(1);
	}
	
	/* Check if an Overflow interrupt has occurred */
	if (interrupt_and_callback_status_mask & TC_INTFLAG_OVF) {
		/* Invoke registered and enabled callback function */
		//(module->callback[TC_CALLBACK_OVERFLOW])(module);
		/* Clear interrupt flag */
		serial_timeout_count++;
		
		if(status_battery == true)
		{
			status_update_count++;
			if(status_update_count > flexible_count)
			{
				status_update_count = 0;
				if(battery_voltage < threshold_blink)
				{
					flexible_count       =  MAX_STATUS_BLINK - ((threshold_blink- battery_voltage)*60/(80));
					threshold_blink      =   MIN_BLINK_START - 5;//decrease the threshold
					*PORT_TOGGLE          =  STATUS_LED_PIN;
					//Toggle Status LED
				}
				else
				{
					threshold_blink      =  MIN_BLINK_START;
					flexible_count       =  MAX_STATUS_BLINK;//
					*PORT_SET			 =  STATUS_LED_PIN;
					//Status LED on
					
				}
			}
		}
		
		
		if(serial_timeout_count > MAX_SERIAL_TIMEOUT)
		{
			serial_timeout = true;
			serial_timeout_count = 0;
		}
		
		*PORT_CLEAR_REGISTER						  = CLEAR_ORB_LEDS;
		if(update_compare_array == true)
		{
			//B2 on 
			//*PORT_SET = B2_RGB;
			if(int_enable == true)
			{
				int_enable = false;
				tc_enable_callback(module, TC_CALLBACK_CC_CHANNEL0);
				tc_clear_status(module,0x00000011);
				/*
				if(led_disable_flag == true)
				{
					
					led_disable_flag = false;
				}
				tc_enable_callback(module, TC_CALLBACK_CC_CHANNEL0);
				*/
			}
			
			//transfer_temp();
			//if(lock_temp_array == false)
			//{
				N_valid_compares = 0;
			
				for(i=0;i<NO_OF_LEDS;i++)
				{
					//N_valid_compares++;
				
					if(temp_compare_array[i] != 255)
					{
						N_valid_compares++;
						//k++;
					}
				
					compare_array[i] = temp_compare_array[i] ;
					pin_array[i]	 = temp_pin_array[i];
				}
				
			//}
			update_compare_array = false;
			//*PORT_CLEAR_REGISTER = B2_RGB;
		}
		compare_array_ID = 0;
		pin_array_ID  = 0;
		
		compare_value = compare_array[0];
		if(compare_value != 255)
		{
			
			led_disable_flag = true;
			//Check sync busy
			while((*STATUS_REGISTER && MASK_SYNC) == true);
			//Update the compare value
			*COMPARE_REGISTER  = compare_value;
			//tc_set_compare_value(module_inst, TC_COMPARE_CAPTURE_CHANNEL_0, compare_value);
		}
		else
		{
			int_enable = true;
			tc_disable_callback(module, TC_CALLBACK_CC_CHANNEL0);
		}
		while((*STATUS_REGISTER && MASK_SYNC) == true);
		*COUNT_REGISTER           =	0;
		module->hw->COUNT8.INTFLAG.reg = TC_INTFLAG_OVF;
		//module->hw->COUNT8.INTFLAG.reg = TC_INTFLAG_MC(1);
	}
	

	/* Check if an Error interrupt has occurred */
	//if (interrupt_and_callback_status_mask & TC_INTFLAG_ERR) {
		/* Invoke registered and enabled callback function */
		//(module->callback[TC_CALLBACK_ERROR])(module);
		/* Clear interrupt flag */
		//module->hw->COUNT8.INTFLAG.reg = TC_INTFLAG_ERR;
	//}

	

	/* Check if an Match/Capture Channel 1 interrupt has occurred */
	//if (interrupt_and_callback_status_mask & TC_INTFLAG_MC(2)) {
		/* Invoke registered and enabled callback function */
		//(module->callback[TC_CALLBACK_CC_CHANNEL1])(module);
		/* Clear interrupt flag */
		//module->hw->COUNT8.INTFLAG.reg = TC_INTFLAG_MC(2);
	//}
	
}
