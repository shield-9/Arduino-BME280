#include<BME280.hpp>

#include "Arduino.h"
#include<Wire.h>


BME280::BME280() {
	
}

/**
 * High Level API
 */
bool BME280::begin(const uint8_t address) {
	return begin(address, REG_CONFIG_TSB_1000, REG_CONFIG_FILTER_OFF,
	             REG_CTRL_OSRS_1X, REG_CTRL_OSRS_1X, REG_CTRL_OSRS_1X,
	             REG_CTRL_MODE_NORMAL);
}

bool BME280::begin(const uint8_t address, const uint8_t t_sb,
	               const uint8_t filter, const uint8_t osrs_t,
	               const uint8_t osrs_p, const uint8_t osrs_h,
	               const uint8_t mode) {
	return begin(address, t_sb, filter, 0x0, osrs_t, osrs_p, osrs_h, mode);
}

bool BME280::begin(const uint8_t address, const uint8_t t_sb,
	               const uint8_t filter, const uint8_t spi3w_en,
	               const uint8_t osrs_t, const uint8_t osrs_p,
	               const uint8_t osrs_h, const uint8_t mode) {
	_address = address;

	Wire.begin();

	uint8_t ctrl_meas = (osrs_t << 5) | (osrs_p << 2) | mode;
	uint8_t config    = (t_sb << 5) | (filter << 2) | spi3w_en;
	uint8_t ctrl_hum  = osrs_h;

	setRegister(REG_ADDR_CTRL_HUM,  ctrl_hum);
	setRegister(REG_ADDR_CTRL_MEAS, ctrl_meas);
	setRegister(REG_ADDR_CONFIG,    config);

	getCalibs();

	return true;
}

void BME280::getData(float* temperature, float* pressure, float* humidity) {
	int32_t adc_T, adc_P, adc_H;

	getRawData(&adc_T, &adc_P, &adc_H);

	_t_fine = calc_T_fine(adc_T);

	*temperature = compensate_T(adc_T) / 100.0;
	*pressure    = compensate_P(adc_P) / 256.0 / 100.0;
	*humidity    = compensate_H(adc_H) / 1024.0;
}


/**
 * Middle Level API
 */
void BME280::getCalibs() {
	uint8_t data[32];

	getRegister(REG_ADDR_CALIB[0],  data,    24);
	getRegister(REG_ADDR_CALIB[24], data+24, 1);
	getRegister(REG_ADDR_CALIB[25], data+25, 7);

	_calib.dig_T1 = (data[1] << 8) | data[0];
	_calib.dig_T2 = (data[3] << 8) | data[2];
	_calib.dig_T3 = (data[5] << 8) | data[4];

	_calib.dig_P1 = (data[7]  << 8) | data[6];
	_calib.dig_P2 = (data[9]  << 8) | data[8];
	_calib.dig_P3 = (data[11] << 8) | data[10];
	_calib.dig_P4 = (data[13] << 8) | data[12];
	_calib.dig_P5 = (data[15] << 8) | data[14];
	_calib.dig_P6 = (data[17] << 8) | data[16];
	_calib.dig_P7 = (data[19] << 8) | data[18];
	_calib.dig_P8 = (data[21] << 8) | data[20];
	_calib.dig_P9 = (data[23] << 8) | data[22];

	_calib.dig_H1 = data[24];
	_calib.dig_H2 = (data[26] << 8) | data[25];
	_calib.dig_H3 = data[27];
	_calib.dig_H4 = (0x0F & data[29]) | (data[28] << 4);
	_calib.dig_H5 = ((data[29] >> 4) & 0x0F) | (data[30] << 4);
	_calib.dig_H6 = data[31];
}

void BME280::getRawData(int32_t* adc_T, int32_t* adc_P, int32_t* adc_H) {
	uint8_t data[8];

	getRegister(REG_ADDR_PRESS_MSB, data, 8);

	*adc_P = ((int32_t)data[0] << 12) | ((int32_t)data[1] << 4) | ((int32_t)data[2] >> 4);
	*adc_T = ((int32_t)data[3] << 12) | ((int32_t)data[4] << 4) | ((int32_t)data[5] >> 4);
	*adc_H = ((int32_t)data[6] << 8) | (int32_t)data[7];
}

/* Compensate Methods are originally provided by BOSCH Sensortec */

int32_t BME280::calc_T_fine(const int32_t adc_T) {
	int32_t var1, var2;

	var1 = ((((adc_T >> 3) - ((int32_t)_calib.dig_T1 << 1))) * ((int32_t)_calib.dig_T2)) >> 11;
	var2 = (((((adc_T >> 4) - ((int32_t)_calib.dig_T1)) * ((adc_T >> 4) - ((int32_t)_calib.dig_T1))) >> 12) * ((int32_t)_calib.dig_T3)) >> 14;
	return var1 + var2;
}

// Returns temperature in DegC, resolution is 0.01 DegC. Output value of “5123” equals 51.23 DegC.
// t_fine carries fine temperature as global value
int32_t BME280::compensate_T(const int32_t adc_T) {
	return (_t_fine * 5 + 128) >> 8;
}

// Returns pressure in Pa as unsigned 32 bit integer in Q24.8 format (24 integer bits and 8 fractional bits).
// Output value of “24674867” represents 24674867/256 = 96386.2 Pa = 963.862 hPa
uint32_t BME280::compensate_P(const int32_t adc_P) {
	int64_t var1, var2, p;

	var1 = ((int64_t)_t_fine) - 128000;
	var2 = var1 * var1 * (int64_t)_calib.dig_P6;
	var2 = var2 + ((var1 * (int64_t)_calib.dig_P5) << 17);
	var2 = var2 + (((int64_t)_calib.dig_P4) << 35);
	var1 = ((var1 * var1 * (int64_t)_calib.dig_P3) >> 8) + ((var1 * (int64_t)_calib.dig_P2) << 12);
	var1 = (((((int64_t)1) << 47) + var1)) * ((int64_t)_calib.dig_P1) >> 33;
	if (var1 == 0) {
		return 0; // avoid exception caused by division by zero
	}
	p = 1048576 - adc_P;
	p = (((p << 31) - var2) * 3125) / var1;
	var1 = (((int64_t)_calib.dig_P9) * (p >> 13) * (p >> 13)) >> 25;
	var2 = (((int64_t)_calib.dig_P8) * p) >> 19;
	p = ((p + var1 + var2) >> 8) + (((int64_t)_calib.dig_P7) << 4);

	return (uint32_t)p;
}

// Returns humidity in %RH as unsigned 32 bit integer in Q22.10 format (22 integer and 10 fractional bits).
// Output value of “47445” represents 47445/1024 = 46.333 %RH
uint32_t BME280::compensate_H(const int32_t adc_H) {
	int32_t v_x1_u32r;

	v_x1_u32r = (_t_fine - ((int32_t)76800));
	v_x1_u32r = (((((adc_H << 14) - (((int32_t)_calib.dig_H4) << 20) - (((int32_t)_calib.dig_H5) * v_x1_u32r)) +
	             ((int32_t)16384)) >> 15) * (((((((v_x1_u32r * ((int32_t)_calib.dig_H6)) >> 10) * (((v_x1_u32r *
	                 ((int32_t)_calib.dig_H3)) >> 11) + ((int32_t)32768))) >> 10) + ((int32_t)2097152)) *
	                 ((int32_t)_calib.dig_H2) + 8192) >> 14));
	v_x1_u32r = (v_x1_u32r - (((((v_x1_u32r >> 15) * (v_x1_u32r >> 15)) >> 7) * ((int32_t)_calib.dig_H1)) >> 4));
	v_x1_u32r = (v_x1_u32r < 0 ? 0 : v_x1_u32r);
	v_x1_u32r = (v_x1_u32r > 419430400 ? 419430400 : v_x1_u32r);

	return (uint32_t)(v_x1_u32r >> 12);
}


/**
 * Low Level API
 */
void BME280::setRegister(const uint8_t reg, uint8_t byte) {
	Wire.beginTransmission(_address);
	Wire.write(reg);
	Wire.write(byte);
	Wire.endTransmission();
}

void BME280::getRegister(const uint8_t reg, uint8_t bytes[], const size_t bytes_len) {
	Wire.beginTransmission(_address);
	Wire.write(reg);
	Wire.endTransmission();
	Wire.requestFrom(_address, bytes_len);

	for(uint8_t i = 0; i < bytes_len; i++) {
		while(!Wire.available());

		bytes[i] = Wire.read();
	}
}

BME280 bme = BME280();
