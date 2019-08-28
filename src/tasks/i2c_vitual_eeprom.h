//
// Created by Felix Hübner on 2019-08-28.
//

#ifndef SAMC_FREERTOS_I2C_VITUAL_EEPROM_H
#define SAMC_FREERTOS_I2C_VITUAL_EEPROM_H

#define EEPROM_USED_SIZE 				(sizeof(adc_i2c_eeprom_t))

/******** Typedef ********/
typedef struct {
	const uint8_t preamble[2];
	uint16_t config_size;
	uint8_t serial_no[16];
	const uint16_t sw_rev;
	uint16_t gain_correction;
	uint16_t offset_correction;
	uint16_t adc_tank_channel[2][4]; //double buffered, Channel 4-1
	uint16_t adc_temp_channel[2][4]; //double buffered, Channel 4-1
	uint16_t adc_counter; //counter, will be increased after each complete conversion round with buffer swap
} adc_i2c_eeprom_t;

typedef union {
	adc_i2c_eeprom_t s;
	uint8_t array[EEPROM_USED_SIZE];
} adc_i2c_eeprom_u;


typedef struct i2c_slave_module i2c_module_t;
typedef struct i2c_slave_packet i2c_slave_packet_t;

void configure_i2c_slave(i2c_module_t *i2c_module, uint8_t slave_address);
void inc_adc_data_counter(void);
void set_eeprom_data_pointer(adc_i2c_eeprom_u *ptr);
void set_i2c_slave_data_packet(i2c_slave_packet_t *ptr);

#endif //SAMC_FREERTOS_I2C_VITUAL_EEPROM_H
