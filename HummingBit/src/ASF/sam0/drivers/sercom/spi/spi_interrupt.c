/**
 * \file
 *
 * \brief SAM Serial Peripheral Interface Driver
 *
 * Copyright (c) 2013-2016 Atmel Corporation. All rights reserved.
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
//#include "LED_control.h"
#include "spi_interrupt.h"


#define SPI_LENGTH    0x04

/**
 * \internal
 *
 * Dummy byte to send when reading in master mode.
 */
uint16_t dummy_write;

/**
 * \internal
 * Starts transceive of buffers with a given length
 *
 * \param[in]  module   Pointer to SPI software instance struct
 * \param[in]  rx_data  Pointer to data to be received
 * \param[in]  tx_data  Pointer to data to be transmitted
 * \param[in]  length   Length of data buffer
 *
 */
#define MASK_RW_INT				0xC0
#define MASK_MODE_INT			0x3F
#define READ_SPI_INT			0x80
#define WR_SPI_INT_SET_ALL		0xCA
#define WR_SPI_INT_RECEIVE_ALL  0xCC
#define LENGTH_SET_ALL_COMMAND	13 
#define PRELOAD_LENGTH			0x02
#define INITIAL_LENGTH			0x04
#define DEVICE_VERSION			0x8C
#define DEVICE_ID_HARDWARE      0x01
#define DEVICE_ID_FIRMWARE      0x01
#define LENGTH_SET_ALL			13
#define LENGTH_SINGLE			 4

extern volatile uint8_t sensor_outputs[20];
extern volatile bool transcation_start;
extern volatile bool serial_timeout;
extern volatile uint8_t serial_timeout_count;
extern volatile bool firmware_check;
/*
extern volatile bool transfer_complete_spi_slave;
extern volatile uint8_t received_value[20];
extern volatile uint8_t sensor_outputs[4];
extern volatile uint8_t transmit_value[20];
*/

//extern volatile uint8_t* ring_buffer;
//volatile uint8_t tail_pointer=0;
//volatile uint8_t head_pointer=0;


static void _spi_transceive_buffer(
		struct spi_module *const module,
		uint8_t *tx_data,
		uint8_t *rx_data,
		uint16_t length)
{
	Assert(module);
	Assert(tx_data);

	/* Write parameters to the device instance */
	module->remaining_tx_buffer_length = length;
	module->remaining_rx_buffer_length = length;
	module->rx_buffer_ptr = rx_data;
	module->tx_buffer_ptr = tx_data;
	module->status = STATUS_BUSY;

	module->dir = SPI_DIRECTION_BOTH;

	/* Get a pointer to the hardware module instance */
	SercomSpi *const hw = &(module->hw->SPI);

	/* Enable the Data Register Empty and RX Complete Interrupt */
	hw->INTENSET.reg = (SPI_INTERRUPT_FLAG_DATA_REGISTER_EMPTY |
			SPI_INTERRUPT_FLAG_RX_COMPLETE);
	hw->INTFLAG.reg = (SPI_INTERRUPT_FLAG_DATA_REGISTER_EMPTY |
	SPI_INTERRUPT_FLAG_RX_COMPLETE | SPI_INTERRUPT_FLAG_COMBINED_ERROR);

#  if CONF_SPI_SLAVE_ENABLE == true
	if (module->mode == SPI_MODE_SLAVE) {
		/* Clear TXC flag if set */
		hw->INTFLAG.reg = SPI_INTERRUPT_FLAG_TX_COMPLETE;
		/* Enable transmit complete interrupt for slave */
		hw->INTENSET.reg = SPI_INTERRUPT_FLAG_TX_COMPLETE;
	}
#  endif
}

/**
 * \internal
 * Starts write of a buffer with a given length
 *
 * \param[in]  module   Pointer to SPI software instance struct
 * \param[in]  tx_data  Pointer to data to be transmitted
 * \param[in]  length   Length of data buffer
 *
 */
static void _spi_write_buffer(
		struct spi_module *const module,
		uint8_t *tx_data,
		uint16_t length)
{
	Assert(module);
	Assert(tx_data);
	
	/* Write parameters to the device instance */
	module->remaining_tx_buffer_length = length;
	module->remaining_dummy_buffer_length = length;
	module->tx_buffer_ptr = tx_data;
	module->status = STATUS_BUSY;

	module->dir = SPI_DIRECTION_WRITE;

	/* Get a pointer to the hardware module instance */
	SercomSpi *const hw = &(module->hw->SPI);

#  if CONF_SPI_SLAVE_ENABLE == true
	if (module->mode == SPI_MODE_SLAVE) {
		/* Clear TXC flag if set */
		hw->INTFLAG.reg = SPI_INTERRUPT_FLAG_TX_COMPLETE;
		/* Enable transmit complete interrupt for slave */
		hw->INTENSET.reg = SPI_INTERRUPT_FLAG_TX_COMPLETE;
	}
#  endif

	if (module->receiver_enabled) {
		/* Enable the Data Register Empty and RX Complete interrupt */
		//hw->INTFLAG.reg  = SPI_INTERRUPT_FLAG_RX_COMPLETE | SPI_INTERRUPT_FLAG_DATA_REGISTER_EMPTY;
		hw->INTENSET.reg = (SPI_INTERRUPT_FLAG_DATA_REGISTER_EMPTY |
				SPI_INTERRUPT_FLAG_RX_COMPLETE);
	} else {
		/* Enable the Data Register Empty interrupt */
		hw->INTENSET.reg = SPI_INTERRUPT_FLAG_DATA_REGISTER_EMPTY;
	}
}

/**
 * \internal
 * Setup SPI to read a buffer with a given length
 *
 * \param[in]  module   Pointer to SPI software instance struct
 * \param[in]  rx_data  Pointer to data to be received
 * \param[in]  length   Length of data buffer
 *
 */
static void _spi_read_buffer(
		struct spi_module *const module,
		uint8_t *rx_data,
		uint16_t length)
{
	Assert(module);
	Assert(rx_data);

	uint8_t tmp_intenset = 0;

	/* Set length for the buffer and the pointer, and let
	 * the interrupt handler do the rest */
	module->remaining_rx_buffer_length = length;
	module->remaining_dummy_buffer_length = length;
	module->rx_buffer_ptr = rx_data;
	module->status = STATUS_BUSY;

	module->dir = SPI_DIRECTION_READ;

	/* Get a pointer to the hardware module instance */
	SercomSpi *const hw = &(module->hw->SPI);

	/* Enable the RX Complete Interrupt */
	tmp_intenset = SPI_INTERRUPT_FLAG_RX_COMPLETE;

#  if CONF_SPI_MASTER_ENABLE == true
	if (module->mode == SPI_MODE_MASTER && module->dir == SPI_DIRECTION_READ) {
		/* Enable Data Register Empty interrupt for master */
		tmp_intenset |= SPI_INTERRUPT_FLAG_DATA_REGISTER_EMPTY;
	}
#  endif
#  if CONF_SPI_SLAVE_ENABLE == true
	if (module->mode == SPI_MODE_SLAVE) {
		/* Clear TXC flag if set */
		hw->INTFLAG.reg = SPI_INTERRUPT_FLAG_TX_COMPLETE;
		/* Enable transmit complete interrupt for slave */
		tmp_intenset |= SPI_INTERRUPT_FLAG_TX_COMPLETE;

		/* Workaround for SSL flag enable */
#ifdef FEATURE_SPI_SLAVE_SELECT_LOW_DETECT
		/* Clear SSL flag if set */
		hw->INTFLAG.reg = SPI_INTERRUPT_FLAG_SLAVE_SELECT_LOW;
		/* Enable Slave Select Low Interrupt for slave */
		tmp_intenset |= SPI_INTERRUPT_FLAG_SLAVE_SELECT_LOW;
#endif
	}
#  endif

	/* Enable all interrupts simultaneously */
	hw->INTENSET.reg = tmp_intenset;
}

/**
 * \brief Registers a SPI callback function
 *
 * Registers a callback function which is implemented by the user.
 *
 * \note The callback must be enabled by \ref spi_enable_callback, in order
 *       for the interrupt handler to call it when the conditions for the
 *       callback type are met.
 *
 * \param[in]  module         Pointer to USART software instance struct
 * \param[in]  callback_func  Pointer to callback function
 * \param[in]  callback_type  Callback type given by an enum
 *
 */
void spi_register_callback(
		struct spi_module *const module,
		spi_callback_t callback_func,
		enum spi_callback callback_type)
{
	/* Sanity check arguments */
	Assert(module);
	Assert(callback_func);

	/* Register callback function */
	module->callback[callback_type] = callback_func;

	/* Set the bit corresponding to the callback_type */
	module->registered_callback |= (1 << callback_type);
}

/**
 * \brief Unregisters a SPI callback function
 *
 * Unregisters a callback function which is implemented by the user.
 *
 * \param[in] module         Pointer to SPI software instance struct
 * \param[in] callback_type  Callback type given by an enum
 *
 */
void spi_unregister_callback(
		struct spi_module *const module,
		enum spi_callback callback_type)
{
	/* Sanity check arguments */
	Assert(module);

	/* Unregister callback function */
	module->callback[callback_type] = NULL;

	/* Clear the bit corresponding to the callback_type */
	module->registered_callback &= ~(1 << callback_type);
}

/**
 * \brief Asynchronous buffer write
 *
 * Sets up the driver to write to the SPI from a given buffer. If registered
 * and enabled, a callback function will be called when the write is finished.
 *
 * \param[in]  module   Pointer to SPI software instance struct
 * \param[out] tx_data  Pointer to data buffer to receive
 * \param[in]  length   Data buffer length
 *
 * \returns Status of the write request operation.
 * \retval STATUS_OK               If the operation completed successfully
 * \retval STATUS_ERR_BUSY         If the SPI was already busy with a write
 *                                 operation
 * \retval STATUS_ERR_INVALID_ARG  If requested write length was zero
 */
enum status_code spi_write_buffer_job(
		struct spi_module *const module,
		uint8_t *tx_data,
		uint16_t length)
{
	Assert(module);
	Assert(tx_data);

	if (length == 0) {
		return STATUS_ERR_INVALID_ARG;
	}

	/* Check if the SPI is busy transmitting or slave waiting for TXC*/
	if (module->status == STATUS_BUSY) {
		return STATUS_BUSY;
	}

	/* Issue internal write */
	_spi_write_buffer(module, tx_data, length);

	return STATUS_OK;
}

/**
 * \brief Asynchronous buffer read
 *
 * Sets up the driver to read from the SPI to a given buffer. If registered
 * and enabled, a callback function will be called when the read is finished.
 *
 * \note If address matching is enabled for the slave, the first character
 *       received and placed in the RX buffer will be the address.
 *
 * \param[in]  module   Pointer to SPI software instance struct
 * \param[out] rx_data  Pointer to data buffer to receive
 * \param[in]  length   Data buffer length
 * \param[in]  dummy    Dummy character to send when reading in master mode
 *
 * \returns Status of the operation.
 * \retval  STATUS_OK               If the operation completed successfully
 * \retval  STATUS_ERR_BUSY         If the SPI was already busy with a read
 *                                  operation
 * \retval  STATUS_ERR_DENIED       If the receiver is not enabled
 * \retval  STATUS_ERR_INVALID_ARG  If requested read length was zero
 */
enum status_code spi_read_buffer_job(
		struct spi_module *const module,
		uint8_t *rx_data,
		uint16_t length,
		uint16_t dummy)
{
	/* Sanity check arguments */
	Assert(module);
	Assert(rx_data);

	if (length == 0) {
		return STATUS_ERR_INVALID_ARG;
	}

	if (!(module->receiver_enabled)) {
		return STATUS_ERR_DENIED;
	}

	/* Check if the SPI is busy transmitting or slave waiting for TXC*/
	if (module->status == STATUS_BUSY) {
		return STATUS_BUSY;
	}

	dummy_write = dummy;
	/* Issue internal read */
	_spi_read_buffer(module, rx_data, length);
	return STATUS_OK;
}

/**
 * \brief Asynchronous buffer write and read
 *
 * Sets up the driver to write and read to and from given buffers. If registered
 * and enabled, a callback function will be called when the transfer is finished.
 *
 * \note If address matching is enabled for the slave, the first character
 *       received and placed in the RX buffer will be the address.
 *
 * \param[in]  module   Pointer to SPI software instance struct
 * \param[in] tx_data   Pointer to data buffer to send
 * \param[out] rx_data  Pointer to data buffer to receive
 * \param[in]  length   Data buffer length
 *
 * \returns Status of the operation.
 * \retval  STATUS_OK               If the operation completed successfully
 * \retval  STATUS_ERR_BUSY         If the SPI was already busy with a read
 *                                  operation
 * \retval  STATUS_ERR_DENIED       If the receiver is not enabled
 * \retval  STATUS_ERR_INVALID_ARG  If requested read length was zero
 */

enum status_code spi_transceive_buffer_job(
		struct spi_module *const module,
		uint8_t *tx_data,
		uint8_t *rx_data,
		uint16_t length)
{
	/* Sanity check arguments */
	Assert(module);
	Assert(rx_data);
	
	if (length == 0) {
		return STATUS_ERR_INVALID_ARG;
	}
	
	if (!(module->receiver_enabled)) {
		return STATUS_ERR_DENIED;
	}
	
	//port_pin_set_output_level(PIN_PA27, true);
	/* Check if the SPI is busy transmitting or slave waiting for TXC*/
	if (module->status == STATUS_BUSY) {
		return STATUS_BUSY;
	}


	/* Issue internal transceive */
	_spi_transceive_buffer(module, tx_data, rx_data, length);
	
	return STATUS_OK;
}
/**
 * \brief Aborts an ongoing job
 *
 * This function will abort the specified job type.
 *
 * \param[in]  module    Pointer to SPI software instance struct
 */
void spi_abort_job(
		struct spi_module *const module)
{
	/* Pointer to the hardware module instance */
	SercomSpi *const spi_hw
		= &(module->hw->SPI);

	/* Abort ongoing job */

	/* Disable interrupts */
	spi_hw->INTENCLR.reg = SPI_INTERRUPT_FLAG_RX_COMPLETE |
			SPI_INTERRUPT_FLAG_DATA_REGISTER_EMPTY |
			SPI_INTERRUPT_FLAG_TX_COMPLETE;

	module->status = STATUS_ABORTED;
	module->remaining_rx_buffer_length = 0;
	module->remaining_dummy_buffer_length = 0;
	module->remaining_tx_buffer_length = 0;

	module->dir = SPI_DIRECTION_IDLE;
}

#  if CONF_SPI_SLAVE_ENABLE == true || CONF_SPI_MASTER_ENABLE == true
/**
 * \internal
 * Writes a character from the TX buffer to the Data register.
 *
 * \param[in,out]  module  Pointer to SPI software instance struct
 */
static void _spi_write( struct spi_module *const module)
{
	/* Pointer to the hardware module instance */
	SercomSpi *const spi_hw = &(module->hw->SPI);

	/* Write value will be at least 8-bits long */
	uint16_t data_to_send = *(module->tx_buffer_ptr);
	/* Increment 8-bit pointer */
	(module->tx_buffer_ptr)++;

	if (module->character_size == SPI_CHARACTER_SIZE_9BIT) {
		data_to_send |= ((*(module->tx_buffer_ptr)) << 8);
		/* Increment 8-bit pointer */
		(module->tx_buffer_ptr)++;
	}

	/* Write the data to send*/
	spi_hw->DATA.reg = data_to_send & SERCOM_SPI_DATA_MASK;

	/* Decrement remaining buffer length */
	(module->remaining_tx_buffer_length)--;
}

static void _spi_write_2(
struct spi_module *const module)
{
	/* Pointer to the hardware module instance */
	SercomSpi *const spi_hw = &(module->hw->SPI);

	/* Write value will be at least 8-bits long */
	uint16_t data_to_send = *(module->tx_buffer_ptr);
	/* Increment 8-bit pointer */
	(module->tx_buffer_ptr)++;

	if (module->character_size == SPI_CHARACTER_SIZE_9BIT) {
		data_to_send |= ((*(module->tx_buffer_ptr)) << 8);
		/* Increment 8-bit pointer */
		(module->tx_buffer_ptr)++;
	}

	/* Write the data to send*/
	spi_hw->DATA.reg = sensor_outputs[0] & SERCOM_SPI_DATA_MASK;

	/* Decrement remaining buffer length */
	(module->remaining_tx_buffer_length)--;
}

#  endif

#  if CONF_SPI_MASTER_ENABLE == true
/**
 * \internal
 * Writes a dummy character to the Data register.
 *
 * \param[in,out]  module  Pointer to SPI software instance struct
 */
static void _spi_write_dummy(
		struct spi_module *const module)
{
	/* Pointer to the hardware module instance */
	SercomSpi *const spi_hw = &(module->hw->SPI);

	/* Write dummy byte */
	spi_hw->DATA.reg = dummy_write;

	/* Decrement remaining dummy buffer length */
	module->remaining_dummy_buffer_length--;
}
#  endif

/**
 * \internal
 * Writes a dummy character from the to the Data register.
 *
 * \param[in,out]  module  Pointer to SPI software instance struct
 */
static void _spi_read_dummy(
		struct spi_module *const module)
{
	/* Pointer to the hardware module instance */
	SercomSpi *const spi_hw = &(module->hw->SPI);
	uint16_t flush = 0;

	/* Read dummy byte */
	flush = spi_hw->DATA.reg;
	UNUSED(flush);

	/* Decrement remaining dummy buffer length */
	module->remaining_dummy_buffer_length--;
}

/**
 * \internal
 * Reads a character from the Data register to the RX buffer.
 *
 * \param[in,out]  module  Pointer to SPI software instance struct
 */
static void _spi_read(
		struct spi_module *const module)
{
	/* Pointer to the hardware module instance */
	SercomSpi *const spi_hw = &(module->hw->SPI);

	uint16_t received_data = (spi_hw->DATA.reg & SERCOM_SPI_DATA_MASK);

	/* Read value will be at least 8-bits long */
	*(module->rx_buffer_ptr) = received_data;
	/* Increment 8-bit pointer */
	module->rx_buffer_ptr += 1;

	if(module->character_size == SPI_CHARACTER_SIZE_9BIT) {
		/* 9-bit data, write next received byte to the buffer */
		*(module->rx_buffer_ptr) = (received_data >> 8);
		/* Increment 8-bit pointer */
		module->rx_buffer_ptr += 1;
	}

	/* Decrement length of the remaining buffer */
	module->remaining_rx_buffer_length--;
}

/**
 * \internal
 *
 * Handles interrupts as they occur, and it will run callback functions
 * which are registered and enabled.
 *
 * \note This function will be called by the Sercom_Handler, and should
 *       not be called directly from any application code.
 *
 * \param[in]  instance  ID of the SERCOM instance calling the interrupt
 *                       handler.
 */
#define PORT_CLEAR_REGISTER_ADD     0x41004414UL
#define PORT_SET_REGISTER_ADD		0x41004418UL

extern volatile bool spi_reset_1 ;


void _spi_interrupt_handler(uint8_t instance)
{
	static uint8_t buffer_length    = 0;
	static uint8_t buffer_length_wr = 0;
	uint16_t data_to_send  = 0;
	uint16_t received_data = 0;
	uint8_t i = 0;
	volatile uint32_t* const PORT_SET		      = PORT_SET_REGISTER_ADD;
	volatile uint32_t* const PORT_CLEAR		      = PORT_CLEAR_REGISTER_ADD;
	uint32_t B2_RGB = 0x08000000;
	uint32_t G2_RGB = 0x00000200;
	
	
	//*PORT_SET	=  G2_RGB;
	
	/* Get device instance from the look-up table */
	struct spi_module *module = (struct spi_module *)_sercom_instances[instance];

	/* Pointer to the hardware module instance */
	SercomSpi *const spi_hw = &(module->hw->SPI);

	/* Combine callback registered and enabled masks. */
	uint8_t callback_mask = module->enabled_callback & module->registered_callback;

	/* Read and mask interrupt flag register */
	uint16_t interrupt_status = spi_hw->INTFLAG.reg;
	interrupt_status &= spi_hw->INTENSET.reg;
	
	/* Data register empty interrupt */ 
	if (interrupt_status & SPI_INTERRUPT_FLAG_DATA_REGISTER_EMPTY) 
	{
		if((module->mode == SPI_MODE_SLAVE) &&(module->dir != SPI_DIRECTION_READ))
		 {
			//_spi_write(module);
			//*PORT_SET	=  B2_RGB;
			data_to_send = *(module->tx_buffer_ptr);
			(module->tx_buffer_ptr)++;
			spi_hw->DATA.reg = data_to_send & SERCOM_SPI_DATA_MASK;
			(module->remaining_tx_buffer_length)--;
		
			if (module->remaining_tx_buffer_length == 0) 
			{
				/* Disable the Data Register Empty Interrupt */
				spi_hw->INTENCLR.reg = SPI_INTERRUPT_FLAG_DATA_REGISTER_EMPTY;
			}
			//*PORT_CLEAR	=  B2_RGB;
		}
	}

	/* Receive complete interrupt*/
	if (interrupt_status & SPI_INTERRUPT_FLAG_RX_COMPLETE) {
		/* Check for overflow */
		//*PORT_SET	=  B2_RGB;
		if (spi_hw->STATUS.reg & SERCOM_SPI_STATUS_BUFOVF) 
		{
			/* Flush */
			uint16_t flush = spi_hw->DATA.reg;
			UNUSED(flush);
			/* Clear overflow flag */
			spi_hw->STATUS.reg = SERCOM_SPI_STATUS_BUFOVF;
		} 
		else 
		{
			
				/* Read data register */
				//_spi_read(module);
				
				received_data = (spi_hw->DATA.reg & SERCOM_SPI_DATA_MASK);
				*(module->rx_buffer_ptr) = received_data;
				module->rx_buffer_ptr += 1;
				module->remaining_rx_buffer_length--;
				
				buffer_length++;
				if(buffer_length == 1)
				{ 
					transcation_start = true;
					serial_timeout = false;
					serial_timeout_count = 0 ; 
					//Check if the command is set all 
					if ((*(module->rx_buffer_ptr-1) == WR_SPI_INT_SET_ALL || *(module->rx_buffer_ptr-1) == WR_SPI_INT_RECEIVE_ALL)) 
					{
						//*PORT_SET	=  G2_RGB;
						module->remaining_tx_buffer_length =  LENGTH_SET_ALL_COMMAND - (INITIAL_LENGTH - module->remaining_tx_buffer_length);
						module->remaining_rx_buffer_length =  LENGTH_SET_ALL_COMMAND - (INITIAL_LENGTH - module->remaining_rx_buffer_length);
						//*PORT_CLEAR	=  G2_RGB;
					}
					else if(*(module->rx_buffer_ptr-1) == DEVICE_VERSION)
					{
						*(module->tx_buffer_ptr)      =	DEVICE_ID_HARDWARE ;
						*(module->tx_buffer_ptr + 1)  = DEVICE_ID_FIRMWARE ;
						firmware_check				  = true;
					}
				}
				
				/* Check if the last character have been received */
				if (module->remaining_rx_buffer_length == 0) 
				{	
					//*PORT_SET	=  G2_RGB;
					buffer_length = 0;
					transcation_start = false;
					firmware_check = false;
					serial_timeout = false;
					serial_timeout_count = 0 ;
					module->status = STATUS_OK;
					/* Disable RX Complete Interrupt and set status */
					spi_hw->INTENCLR.reg = SPI_INTERRUPT_FLAG_RX_COMPLETE;
					if(module->dir == SPI_DIRECTION_BOTH) {
						if (callback_mask & (1 << SPI_CALLBACK_BUFFER_TRANSCEIVED)) {
							(module->callback[SPI_CALLBACK_BUFFER_TRANSCEIVED])(module);
							
						}
						
					} else if (module->dir == SPI_DIRECTION_READ) {
						if (callback_mask & (1 << SPI_CALLBACK_BUFFER_RECEIVED)) {
							(module->callback[SPI_CALLBACK_BUFFER_RECEIVED])(module);
						}
					}
					//*PORT_CLEAR	=  G2_RGB;
				}
			}
			//*PORT_CLEAR	=  B2_RGB;
	}

	/* Transmit complete */
	if (interrupt_status & SPI_INTERRUPT_FLAG_TX_COMPLETE) {
#  if CONF_SPI_SLAVE_ENABLE == true
		if (module->mode == SPI_MODE_SLAVE) {
			
			spi_reset_1 = true;
			if(module->dir == SPI_DIRECTION_BOTH) {
				if (callback_mask & (1 << SPI_CALLBACK_BUFFER_TRANSCEIVED)) {
					(module->callback[SPI_CALLBACK_BUFFER_TRANSCEIVED])(module);
					
				}
			}
			//*PORT_SET	=  B2_RGB;
			spi_hw->INTFLAG.reg = SPI_INTERRUPT_FLAG_TX_COMPLETE;
			//*PORT_CLEAR	=  B2_RGB;
		}
#  endif

	}

#  ifdef FEATURE_SPI_SLAVE_SELECT_LOW_DETECT
#  if CONF_SPI_SLAVE_ENABLE == true
        
		/* When a high to low transition is detected on the _SS pin in slave mode */
		if (interrupt_status & SPI_INTERRUPT_FLAG_SLAVE_SELECT_LOW) {
			if (module->mode == SPI_MODE_SLAVE) {
				//*PORT_SET	=  B2_RGB;
				/* Disable interrupts */
				spi_hw->INTENCLR.reg = SPI_INTERRUPT_FLAG_SLAVE_SELECT_LOW;
				/* Clear interrupt flag */
				spi_hw->INTFLAG.reg = SPI_INTERRUPT_FLAG_SLAVE_SELECT_LOW;

				if (callback_mask & (1 << SPI_CALLBACK_SLAVE_SELECT_LOW)) {
					(module->callback[SPI_CALLBACK_SLAVE_SELECT_LOW])(module);
				}
				//*PORT_CLEAR	=  B2_RGB;
			}
		}
		
#  endif
#  endif

#  ifdef FEATURE_SPI_ERROR_INTERRUPT
	/* When combined error happen */
	if (interrupt_status & SPI_INTERRUPT_FLAG_COMBINED_ERROR) {
		//*PORT_SET	=  B2_RGB;
		/* Disable interrupts */
		spi_hw->INTENCLR.reg = SPI_INTERRUPT_FLAG_COMBINED_ERROR;
		/* Clear interrupt flag */
		spi_hw->INTFLAG.reg = SPI_INTERRUPT_FLAG_COMBINED_ERROR;

		if (callback_mask & (1 << SPI_CALLBACK_COMBINED_ERROR)) {
			(module->callback[SPI_CALLBACK_COMBINED_ERROR])(module);
		}
		//*PORT_CLEAR	=  B2_RGB;
	}
#  endif
	
	//*PORT_CLEAR	=  G2_RGB;
  
}
