//
// Created by Felix Hübner on 2019-08-28.
//

#include "asf.h"
#include <samc21_slcan_adc.h>
#include "i2c_vitual_eeprom.h"
#include "i2c_slave.h"
#include <i2c_slave_interrupt.h>
#include <log.h>


/******** Global Variables ********/
adc_i2c_eeprom_u *eeprom_data_ptr = NULL;
struct i2c_slave_packet *data_packet_ptr = NULL;
uint8_t i2c_data_address = 0;


/******** Internal Prototypes ********/
enum status_code _i2c_receive_data(i2c_module_t *i2c_module, uint8_t length, uint8_t *ptr);
enum status_code _i2c_send_data(i2c_module_t *i2c_module, uint8_t length, uint8_t *ptr);
void i2c_callback_master_read_request(i2c_module_t *module);
void i2c_callback_slave_read_complete(i2c_module_t *module);
void i2c_callback_master_write_request(i2c_module_t *module);
void i2c_callback_slave_write_complete(i2c_module_t *module);
void i2c_callback_transfer_error(i2c_module_t *module);
void i2c_callback_error(i2c_module_t *module);


/******** Methods ********/
void configure_i2c_slave(i2c_module_t *i2c_module, uint8_t slave_address) {

	configASSERT(i2c_module);

	/* Create and initialize config_i2c_slave structure */
	struct i2c_slave_config config_i2c_slave;
	i2c_slave_get_config_defaults(&config_i2c_slave);
	/* Change address and address_mode */
	config_i2c_slave.generator_source 				= GCLK_GENERATOR_3;
	config_i2c_slave.transfer_speed 				= I2C_SLAVE_SPEED_STANDARD_AND_FAST;
	config_i2c_slave.address        				= slave_address;
	config_i2c_slave.address_mode   				= I2C_SLAVE_ADDRESS_MODE_MASK;
	config_i2c_slave.pinmux_pad0					= ADC_I2C_SLAVE_SDA_PINMUX;
	config_i2c_slave.pinmux_pad1					= ADC_I2C_SLAVE_SCK_PINMUX;
	config_i2c_slave.scl_stretch_only_after_ack_bit	= false;
	config_i2c_slave.buffer_timeout					= 1000;
	config_i2c_slave.enable_nack_on_address			= false;

	/* Initialize and enable device with config_i2c_slave */
	if (i2c_slave_init(i2c_module, ADC_I2C_SLAVE_MODULE, &config_i2c_slave) != STATUS_OK) {
		ulog_s("ERROR: while setup I2C!\r\n");
	}

	i2c_slave_register_callback(i2c_module, i2c_callback_master_write_request, I2C_SLAVE_CALLBACK_READ_REQUEST); //master wants to read, we have to write
	i2c_slave_enable_callback(i2c_module, I2C_SLAVE_CALLBACK_READ_REQUEST);
	i2c_slave_register_callback(i2c_module, i2c_callback_master_read_request, I2C_SLAVE_CALLBACK_WRITE_REQUEST); // master wants to write, we have to read
	i2c_slave_enable_callback(i2c_module, I2C_SLAVE_CALLBACK_WRITE_REQUEST);
	i2c_slave_register_callback(i2c_module, i2c_callback_slave_read_complete, I2C_SLAVE_CALLBACK_READ_COMPLETE); //slave read complete callback
	i2c_slave_enable_callback(i2c_module, I2C_SLAVE_CALLBACK_READ_COMPLETE);
	i2c_slave_register_callback(i2c_module, i2c_callback_slave_write_complete, I2C_SLAVE_CALLBACK_WRITE_COMPLETE); //slave write completed callback
	i2c_slave_enable_callback(i2c_module, I2C_SLAVE_CALLBACK_WRITE_COMPLETE);
	i2c_slave_register_callback(i2c_module, i2c_callback_transfer_error, I2C_SLAVE_CALLBACK_ERROR_LAST_TRANSFER);
	i2c_slave_enable_callback(i2c_module, I2C_SLAVE_CALLBACK_ERROR_LAST_TRANSFER);
	i2c_slave_register_callback(i2c_module, i2c_callback_error, I2C_SLAVE_CALLBACK_ERROR);
	i2c_slave_enable_callback(i2c_module, I2C_SLAVE_CALLBACK_ERROR);

	i2c_slave_enable(i2c_module);
}


/**
 * start a i2c read job
 * @param i2c_module i2c_module instance object
 * @param length length of data to be read
 * @param ptr pointer to a data region to write data to
 * @return return STATUS_OK if successful or STATUS_BUSY if the read job could not be set up
 */
enum status_code _i2c_receive_data(i2c_module_t *i2c_module, uint8_t length, uint8_t *ptr) {
	data_packet_ptr->data_length = length;
	data_packet_ptr->data = ptr;
	return i2c_slave_read_packet_job(i2c_module, data_packet_ptr);
}


/**
 * start a i2c write job
 * @param i2c_module i2c_module instance object
 * @param length length of data to be written
 * @param ptr pointer to a data region to read data from
 * @return return STATUS_OK if successful or STATUS_BUSY if the write job could not be set up
 */
enum status_code _i2c_send_data(i2c_module_t *i2c_module, uint8_t length, uint8_t *ptr) {
	data_packet_ptr->data_length = length;
	data_packet_ptr->data = ptr;
	return i2c_slave_write_packet_job(i2c_module, data_packet_ptr);
}


/**
 * request to read from master, we allow only 1 byte (data address)
 * @param module
 */
void i2c_callback_master_read_request(i2c_module_t *module) {
	port_pin_set_output_level(PIN_PA20, true);
//	c_log_s(" r-");
//	uint32_t status = i2c_slave_get_status(module);
//	c_log((uint8_t) ((0x0F & (status>>8)) + 0x30));
//	c_log((uint8_t) ((0x0F & (status>>4)) + 0x30));
//	c_log((uint8_t) ((0x0F & (status)) + 0x30));
	//start read 1 byte (command/address byte)
	enum status_code stat = _i2c_receive_data(module, 1, &i2c_data_address);
	if (stat == STATUS_BUSY) {
//		c_log_s("-E_B");
	}
	port_pin_set_output_level(PIN_PA20, false);
}


/**
 * will be called if read from master complete
 * @param module i2c_module instance object
 */
void i2c_callback_slave_read_complete(i2c_module_t *module) {
	port_pin_set_output_level(PIN_PA20, true);
//	c_log_s(" R-");
	uint32_t status = i2c_slave_get_status(module);
//	c_log((uint8_t) ((0x0F & (status>>8)) + 0x30));
//	c_log((uint8_t) ((0x0F & (status>>4)) + 0x30));
//	c_log((uint8_t) ((0x0F & (status)) + 0x30));
	if (status & I2C_SLAVE_STATUS_REPEATED_START) {
//		c_log_s("-S4");
	}
	port_pin_set_output_level(PIN_PA20, false);
}


/**
 * request to write to master, we don't know the length, so we start a job with the maximal possible length (up to the valid end of the eeprom)
 * @param module i2c_module instance object
 */
void i2c_callback_master_write_request(i2c_module_t *module) {
	port_pin_set_output_level(PIN_PA20, true);
//	c_log_s(" w-");


	uint32_t status = i2c_slave_get_status(module);
//	c_log((uint8_t) ((0x0F & (status>>8)) + 0x30));
//	c_log((uint8_t) ((0x0F & (status>>4)) + 0x30));
//	c_log((uint8_t) ((0x0F & (status)) + 0x30));
	if (status & I2C_SLAVE_STATUS_REPEATED_START) {
//		c_log_s("-S3");
	}

	uint8_t length = 0;
	//check if the data address is inside the valid eeprom space
	if (i2c_data_address < EEPROM_USED_SIZE) {
		//calculate allowed bytes to send
		length = EEPROM_USED_SIZE - i2c_data_address;
		enum status_code stat = _i2c_send_data(module, length, &(eeprom_data_ptr->array[i2c_data_address]));
		if (stat == STATUS_BUSY) {
//			c_log_s("-E_B");
		}
	} else {
		//start a single write to act like an eeprom. the single byte will be send and then the interrupts are disabled
		//because the buffer is empty, which will result in all following bytes are also read as 0xFF from master.
		uint8_t tmp = 0xff;
		enum status_code stat = _i2c_send_data(module, 1, &(tmp));
		if (stat == STATUS_BUSY) {
//			c_log_s("-E_B");
		}
	}
	port_pin_set_output_level(PIN_PA20, false);
}


/**
 * will be called if write to master is complete
 * @param module i2c_module instance object
 */
void i2c_callback_slave_write_complete(i2c_module_t *module) {
	port_pin_set_output_level(PIN_PA20, true);
//	c_log_s(" W-");
	uint32_t status = i2c_slave_get_status(module);
//	c_log((uint8_t) ((0x0F & (status>>8)) + 0x30));
//	c_log((uint8_t) ((0x0F & (status>>4)) + 0x30));
//	c_log((uint8_t) ((0x0F & (status)) + 0x30));
	if (status & I2C_SLAVE_STATUS_REPEATED_START) {
//		c_log_s("-S2");
	}
	port_pin_set_output_level(PIN_PA20, false);
}


/**
 * will be called if a transfer error occured
 * @param module i2c_module instance object
 */
void i2c_callback_transfer_error(i2c_module_t *module) {
	port_pin_set_output_level(PIN_PA20, true);
//	c_log_s(" T-");
	uint32_t status = i2c_slave_get_status(module);
//	c_log((uint8_t) ((0x0F & (status>>8)) + 0x30));
//	c_log((uint8_t) ((0x0F & (status>>4)) + 0x30));
//	c_log((uint8_t) ((0x0F & (status)) + 0x30));
	if (status & I2C_SLAVE_STATUS_REPEATED_START) {
//		c_log_s("-S5");
	}
	port_pin_set_output_level(PIN_PA20, false);
}


/**
 * will be called if a error with a callback occured
 * @param module i2c_module instance object
 */
void i2c_callback_error(i2c_module_t *module) {
	port_pin_set_output_level(PIN_PA20, true);
//	c_log('E');
	uint32_t status = i2c_slave_get_status(module);
	if (status & I2C_SLAVE_STATUS_REPEATED_START) {
//		c_log_s("S6");
	}
	port_pin_set_output_level(PIN_PA20, false);
}


/******** Getter/Setter ********/

/**
 * increment the data counter in the eeprom
 */
void inc_adc_data_counter(void) {
	if (eeprom_data_ptr) {
		eeprom_data_ptr->s.adc_counter++;
	}
}


/**
 * pass a pointer of the eeprom data object to this object
 * @param ptr pointer of the eeprom object
 */
void set_eeprom_data_pointer(adc_i2c_eeprom_u *ptr){
	eeprom_data_ptr = ptr;
}


/**
 * pass a pointer of the i2c_data packet to this object
 * @param ptr pointer of the eeprom object
 */
void set_i2c_slave_data_packet(i2c_slave_packet_t *ptr) {
	data_packet_ptr = ptr;
}