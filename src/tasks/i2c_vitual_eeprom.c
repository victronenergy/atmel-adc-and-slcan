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


enum status_code _i2c_receive_data(i2c_module_t *i2c_module, uint8_t length, uint8_t *ptr) {
	data_packet_ptr->data_length = length;
	data_packet_ptr->data = ptr;
	return i2c_slave_read_packet_job(i2c_module, data_packet_ptr);
}

enum status_code _i2c_send_data(i2c_module_t *i2c_module, uint8_t length, uint8_t *ptr) {
	data_packet_ptr->data_length = length;
	data_packet_ptr->data = ptr;
	return i2c_slave_write_packet_job(i2c_module, data_packet_ptr);
}

/**
 * request to read from master
 * @param module
 */
void i2c_callback_master_read_request(i2c_module_t *module) {
	port_pin_set_output_level(PIN_PA20, true);
	c_log_s(" r-");
	port_pin_set_output_level(PIN_PA20, false);

	uint32_t status = i2c_slave_get_status(module);
	c_log((uint8_t) ((0x0F & (status>>8)) + 0x30));
	c_log((uint8_t) ((0x0F & (status>>4)) + 0x30));
	c_log((uint8_t) ((0x0F & (status)) + 0x30));
//TODO this make no sense, write will not use a repeated start to write to device!! Fix later
	if (status & I2C_SLAVE_STATUS_REPEATED_START) {
		c_log_s("-S1");
		//calculate allowed bytes to write
		uint8_t sum = 0;
		if (sum > 0) {
			enum status_code stat = _i2c_receive_data(module, sum, &i2c_data_address); // teporarilly
			if (stat == STATUS_BUSY) {
				c_log_s("-E_B");
			}
		} else {
			//i2c_slave_enable_nack_on_address(module);
		}
	} else {
		//start read 1 byte (command/address byte)
		enum status_code stat = _i2c_receive_data(module, 1, &i2c_data_address);
		if (stat == STATUS_BUSY) {
			c_log_s("-E_B");
		}
//		i2c_slave_disable_nack_on_address(module);
	}
}


void i2c_callback_slave_read_complete(i2c_module_t *module) {
	port_pin_set_output_level(PIN_PA20, true);
	c_log_s(" R-");
	port_pin_set_output_level(PIN_PA20, false);
	uint32_t status = i2c_slave_get_status(module);
	c_log((uint8_t) ((0x0F & (status>>8)) + 0x30));
	c_log((uint8_t) ((0x0F & (status>>4)) + 0x30));
	c_log((uint8_t) ((0x0F & (status)) + 0x30));
	if (status & I2C_SLAVE_STATUS_REPEATED_START) {
		c_log_s("-S4");
	}
}


/**
 * master requests at least 1 byte
 * @param module
 */
void i2c_callback_master_write_request(i2c_module_t *module) {
	port_pin_set_output_level(PIN_PA20, true);
	c_log_s(" w-");
	port_pin_set_output_level(PIN_PA20, false);

	uint32_t status = i2c_slave_get_status(module);
	c_log((uint8_t) ((0x0F & (status>>8)) + 0x30));
	c_log((uint8_t) ((0x0F & (status>>4)) + 0x30));
	c_log((uint8_t) ((0x0F & (status)) + 0x30));
	if (status & I2C_SLAVE_STATUS_REPEATED_START) {
		c_log_s("-S3");

		//calculate allowed bytes to write
		uint8_t sum = 0;
		if (sum > 0) {
			enum status_code stat = _i2c_receive_data(module, sum, &i2c_data_address); // teporarilly
			if (stat == STATUS_BUSY) {
				c_log_s("-E_B");
			}
		} else {
//TODO why change nack/ack on address reception? this is a disable of the i2c interface on the specified address, is it?

//			i2c_slave_enable_nack_on_address(module);
		}
	}
	uint8_t length = 0;
	if (i2c_data_address < EEPROM_USED_SIZE) {
		length = EEPROM_USED_SIZE - i2c_data_address;
		enum status_code stat = _i2c_send_data(module, length, &(eeprom_data_ptr->array[i2c_data_address]));
		if (stat == STATUS_BUSY) {
			c_log_s("-E_B");
		}
	} else {
		//start a single write to act like an eeprom. the single byte will be send and then the interrupts are disabled
		//because the buffer is empty, which will result in all following bytes are also read as 0xFF from master.
		uint8_t tmp = 0xff;
		enum status_code stat = _i2c_send_data(module, 1, &(tmp));
		if (stat == STATUS_BUSY) {
			c_log_s("-E_B");
		}
	}
}

/**
 * receive complete
 * @param module
 */
void i2c_callback_slave_write_complete(i2c_module_t *module) {
	port_pin_set_output_level(PIN_PA20, true);
	c_log_s(" W-");

	uint32_t status = i2c_slave_get_status(module);
	c_log((uint8_t) ((0x0F & (status>>8)) + 0x30));
	c_log((uint8_t) ((0x0F & (status>>4)) + 0x30));
	c_log((uint8_t) ((0x0F & (status)) + 0x30));
	port_pin_set_output_level(PIN_PA20, false);
	if (status & I2C_SLAVE_STATUS_REPEATED_START) {
		c_log_s("-S2");
	}
//	i2c_slave_disable_nack_on_address(i2c_slave_instance);
}

void i2c_callback_transfer_error(i2c_module_t *module) {
	port_pin_set_output_level(PIN_PA20, true);
	c_log_s(" T-");
	port_pin_set_output_level(PIN_PA20, false);

	uint32_t status = i2c_slave_get_status(module);
	c_log((uint8_t) ((0x0F & (status>>8)) + 0x30));
	c_log((uint8_t) ((0x0F & (status>>4)) + 0x30));
	c_log((uint8_t) ((0x0F & (status)) + 0x30));
	if (status & I2C_SLAVE_STATUS_REPEATED_START) {
		c_log_s("-S5");
	}
//	i2c_slave_disable_nack_on_address(i2c_slave_instance);
}

void i2c_callback_error(i2c_module_t *module) {
	port_pin_set_output_level(PIN_PA20, true);
	c_log('E');
	port_pin_set_output_level(PIN_PA20, false);

	uint32_t status = i2c_slave_get_status(module);
	if (status & I2C_SLAVE_STATUS_REPEATED_START) {
		c_log_s("S6");
	}
//	i2c_slave_disable_nack_on_address(i2c_slave_instance);
}

/******** Getter/Setter ********/
void inc_adc_data_counter(void) {
	if (eeprom_data_ptr) {
		eeprom_data_ptr->s.adc_counter++;
	}
}

void set_eeprom_data_pointer(adc_i2c_eeprom_u *ptr){
	eeprom_data_ptr = ptr;
}

void set_i2c_slave_data_packet(i2c_slave_packet_t *ptr) {
	data_packet_ptr = ptr;
}