#ifndef BME280_h
#define BME280_h

#include "Arduino.h"

typedef struct {
	uint16_t dig_T1;
	int16_t  dig_T2;
	int16_t  dig_T3;

	uint16_t dig_P1;
	int16_t  dig_P2;
	int16_t  dig_P3;
	int16_t  dig_P4;
	int16_t  dig_P5;
	int16_t  dig_P6;
	int16_t  dig_P7;
	int16_t  dig_P8;
	int16_t  dig_P9;

	uint8_t  dig_H1;
	int16_t  dig_H2;
	uint8_t  dig_H3;
	int16_t  dig_H4;
	int16_t  dig_H5;
	int8_t   dig_H6;
} BME280_calib;

class BME280 {
	public:
		BME280();

		// SoftSerial in the future...
		// BME280(uint8_t rxPin, uint8_t txPin);

		// This library doesn't support SPI at the moment.
		// TODO: Add support for SPI.
		bool begin(const uint8_t address);

		bool begin(const uint8_t address, uint8_t t_sb, uint8_t filter,
		           uint8_t osrs_t, uint8_t osrs_p, uint8_t osrs_h,
		           uint8_t mode);
		           
		bool begin(const uint8_t address, uint8_t t_sb, uint8_t filter,
		           uint8_t spi3w_en, uint8_t osrs_t, uint8_t osrs_p,
		           uint8_t osrs_h, uint8_t mode);

		void getData(float* temperature, float* pressure, float* humidity);

		void end();

	private:
		/**
		 * Register Addresses
		 */
		const uint8_t REG_ADDR_CALIB[32] = {
			0x88, 0x89, 0x8A, 0x8B, 0x8C, 0x8D, 0x8E, 0x8F, 0x90, 0x91,
			0x92, 0x93, 0x94, 0x95, 0x96, 0x97, 0x98, 0x99, 0x9A, 0x9B,
			0x9C, 0x9D, 0x9E, 0x9F, 0xA1, 0xE1, 0xE2, 0xE3, 0xE4, 0xE5, 0xE6, 0xE7
		};

		const uint8_t REG_ADDR_ID        = 0xD0;
		const uint8_t REG_ADDR_RESET     = 0xE0;
		const uint8_t REG_ADDR_STATUS    = 0xF3;

		const uint8_t REG_ADDR_CTRL_HUM  = 0xF2;
		const uint8_t REG_ADDR_CTRL_MEAS = 0xF4;
		const uint8_t REG_ADDR_CONFIG    = 0xF5;

		const uint8_t REG_ADDR_PRESS_MSB  = 0xF7;
		const uint8_t REG_ADDR_PRESS_LSB  = 0xF8;
		const uint8_t REG_ADDR_PRESS_XLSB = 0xF9;
		const uint8_t REG_ADDR_TEMP_MSB   = 0xFA;
		const uint8_t REG_ADDR_TEMP_LSB   = 0xFB;
		const uint8_t REG_ADDR_TEMP_XLSB  = 0xFC;
		const uint8_t REG_ADDR_HUM_XSB    = 0xFD;
		const uint8_t REG_ADDR_HUM_LSB    = 0xFE;

		/**
		 * Register 0xF5 "config"
		 */
		// Controls t_standby, inactive duration. (Unit: ms)
		const uint8_t REG_CONFIG_TSB_0_5  = 0x00;
		const uint8_t REG_CONFIG_TSB_62_5 = 0x01;
		const uint8_t REG_CONFIG_TSB_125  = 0x02;
		const uint8_t REG_CONFIG_TSB_250  = 0x03;
		const uint8_t REG_CONFIG_TSB_500  = 0x04;
		const uint8_t REG_CONFIG_TSB_1000 = 0x05;
		const uint8_t REG_CONFIG_TSB_10   = 0x06;
		const uint8_t REG_CONFIG_TSB_20   = 0x07;

		// Controls the time constant of the IIR filter.
		const uint8_t REG_CONFIG_FILTER_OFF = 0x00;
		const uint8_t REG_CONFIG_FILTER_2   = 0x01;
		const uint8_t REG_CONFIG_FILTER_4   = 0x02;
		const uint8_t REG_CONFIG_FILTER_8   = 0x03;
		const uint8_t REG_CONFIG_FILTER_16  = 0x04;

		/**
		 * Register 0xF2 "ctrl_hum", 0xF4 "ctrl_meas"
		 */
		// Controls oversampling of data.
		const uint8_t REG_CTRL_OSRS_SKIP = 0x00;
		const uint8_t REG_CTRL_OSRS_1X   = 0x01;
		const uint8_t REG_CTRL_OSRS_2X   = 0x02;
		const uint8_t REG_CTRL_OSRS_4X   = 0x03;
		const uint8_t REG_CTRL_OSRS_8X   = 0x04;
		const uint8_t REG_CTRL_OSRS_16X  = 0x05;

		// Controls the sensor mode of the device.
		const uint8_t REG_CTRL_MODE_SLEEP  = 0x00;
		const uint8_t REG_CTRL_MODE_FORCED = 0x01;
		const uint8_t REG_CTRL_MODE_NORMAL = 0x03;

		/**
		 * Register 0xD0 "id"
		 */
		const uint8_t REG_ID_VALUE = 0x60;

		/**
		 * Register 0xE0 "reset"
		 */
		const uint8_t REG_RESET_VALUE = 0xB6;


		// SoftSerial in the future...
		// uint8_t _rx_pin;
		// uint8_t _tx_pin;

		uint8_t _address;
		uint8_t _chip_id;

		BME280_calib _calib;
		int32_t _t_fine;

		void getCalibs();
		void getRawData(int32_t* adc_T, int32_t* adc_P, int32_t* adc_H);

		int32_t calc_T_fine(const int32_t adc_T);

		int32_t compensate_T(const int32_t adc_T);
		uint32_t compensate_P(const int32_t adc_P);
		uint32_t compensate_H(const int32_t adc_H);

		void setRegister(const uint8_t reg, const uint8_t byte);
		void getRegister(const uint8_t reg, uint8_t bytes[], const size_t bytes_len);
};

extern BME280 bme;

#endif
