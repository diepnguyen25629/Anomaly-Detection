
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "esp32/rom/ets_sys.h"
#include "ds18b20_1_wire.h"
#include "esp_timer.h"
#include "esp_rom_gpio.h"
#include "esp_log.h"

static const char *TAG = "DS18B20_1_WIRE";

uint8_t DS_GPIO;
uint8_t bitResolution = 12;
uint8_t devices = 0;

DeviceAddress ROM_NO;
uint8_t LastDiscrepancy;
uint8_t LastFamilyDiscrepancy;
bool LastDeviceFlag;

/// Sends one bit to bus
esp_err_t ds18b20_write(char bit){
	if (bit & 1) {
		gpio_set_direction(DS_GPIO, GPIO_MODE_OUTPUT);
		noInterrupts();
		gpio_set_level(DS_GPIO,0);
		ets_delay_us(6);
		gpio_set_direction(DS_GPIO, GPIO_MODE_INPUT);	// release bus
		ets_delay_us(64);
		interrupts();
	} else {
		gpio_set_direction(DS_GPIO, GPIO_MODE_OUTPUT);
		noInterrupts();
		gpio_set_level(DS_GPIO,0);
		ets_delay_us(60);
		gpio_set_direction(DS_GPIO, GPIO_MODE_INPUT);	// release bus
		ets_delay_us(10);
		interrupts();
	}
	return ESP_OK;
}

// Reads one bit from bus
unsigned char ds18b20_read(void){
	unsigned char value = 0;
	gpio_set_direction(DS_GPIO, GPIO_MODE_OUTPUT);
	noInterrupts();
	gpio_set_level(DS_GPIO, 0);
	ets_delay_us(6);
	gpio_set_direction(DS_GPIO, GPIO_MODE_INPUT);
	ets_delay_us(9);
	value = gpio_get_level(DS_GPIO);
	ets_delay_us(55);
	interrupts();
	return value;
}

// Sends one byte to bus
esp_err_t ds18b20_write_byte(char data){
  unsigned char i;
  unsigned char x;
  for(i=0; i<8; i++){
    x = data>>i;
    x &= 0x01;
    ds18b20_write(x);
  }
  ets_delay_us(100);
  return ESP_OK;
}

// Reads one byte from bus
unsigned char ds18b20_read_byte(void){
  unsigned char i;
  unsigned char data = 0;
  for (i=0;i<8;i++)
  {
    if(ds18b20_read()) data |= 0x01<<i;
    ets_delay_us(15);
  }
  return data;
}

// Sends reset pulse
unsigned char ds18b20_reset(void){
	unsigned char presence = 0;
	gpio_set_direction(DS_GPIO, GPIO_MODE_OUTPUT);
	noInterrupts();
	gpio_set_level(DS_GPIO, 0);
	ets_delay_us(480);
	gpio_set_level(DS_GPIO, 1);
	gpio_set_direction(DS_GPIO, GPIO_MODE_INPUT);
	ets_delay_us(70);
	presence = (gpio_get_level(DS_GPIO) == 0);
	ets_delay_us(410);
	interrupts();
	return presence;
}

bool ds18b20_setResolution(const DeviceAddress tempSensorAddresses[], int numAddresses, uint8_t newResolution) {
	bool success = false;
	uint8_t newValue = 0;
	ScratchPad scratchPad;
	// loop through each address
	for (int i = 0; i < numAddresses; i++){
		// we can only update the sensor if it is connected
		if (ds18b20_isConnected((DeviceAddress*) tempSensorAddresses[i], scratchPad)) {
			switch (newResolution) {
			case RESOLUTION_12_BIT:
				newValue = TEMP_12_BIT;
				break;
			case RESOLUTION_11_BIT:
				newValue = TEMP_11_BIT;
				break;
			case RESOLUTION_10_BIT:
				newValue = TEMP_10_BIT;
				break;
			case RESOLUTION_9_BIT:
				newValue = TEMP_9_BIT;
				break;
			default:
				newValue = TEMP_12_BIT;
				break;
			}
			// if it needs to be updated we write the new value
			if (scratchPad[CONFIGURATION] != newValue) {
				scratchPad[CONFIGURATION] = newValue;
				ds18b20_writeScratchPad((DeviceAddress*) tempSensorAddresses[i], scratchPad);
			}
			success = true;
		}
	}
	return success;
}

esp_err_t ds18b20_writeScratchPad(const DeviceAddress *deviceAddress, const uint8_t *scratchPad) {
	ds18b20_reset();
	ds18b20_select(deviceAddress);
	ds18b20_write_byte(WRITESCRATCH);
	ds18b20_write_byte(scratchPad[HIGH_ALARM_TEMP]); // high alarm temp
	ds18b20_write_byte(scratchPad[LOW_ALARM_TEMP]); // low alarm temp
	ds18b20_write_byte(scratchPad[CONFIGURATION]);
	ds18b20_reset();
	return ESP_OK;
}

bool ds18b20_readScratchPad(const DeviceAddress *deviceAddress, uint8_t* scratchPad) {
	// send the reset command and fail fast
	int b = ds18b20_reset();
	if (b == 0) return false;
	ds18b20_select(deviceAddress);
	ds18b20_write_byte(READSCRATCH);
	// Read all registers in a simple loop
	// byte 0: temperature LSB
	// byte 1: temperature MSB
	// byte 2: high alarm temp
	// byte 3: low alarm temp
	// byte 4: DS18B20 & DS1822: configuration register
	// byte 5: internal use & crc
	// byte 6: DS18B20 & DS1822: store for crc
	// byte 7: DS18B20 & DS1822: store for crc
	// byte 8: SCRATCHPAD_CRC
	for (uint8_t i = 0; i < 9; i++) {
		scratchPad[i] = ds18b20_read_byte();
	}
	b = ds18b20_reset();
	return (b == 1);
}

esp_err_t ds18b20_select(const DeviceAddress *address){
    uint8_t i;
    ds18b20_write_byte(SELECTDEVICE);           // Choose ROM
    for (i = 0; i < 8; i++) ds18b20_write_byte(((uint8_t *)address)[i]);
	return ESP_OK;
}

esp_err_t ds18b20_requestTemperatures(){
	ds18b20_reset();
	ds18b20_write_byte(SKIPROM);
	ds18b20_write_byte(GETTEMP);
    unsigned long start = esp_timer_get_time() / 1000ULL;
    while (!isConversionComplete() && ((esp_timer_get_time() / 1000ULL) - start < millisToWaitForConversion())) vPortYield();
	return ESP_OK;
}

bool isConversionComplete() {
	uint8_t b = ds18b20_read();
	return (b == 1);
}

uint16_t millisToWaitForConversion() {
	switch (bitResolution) {
	case RESOLUTION_9_BIT:
		return 94;
	case RESOLUTION_10_BIT:
		return 188;
	case RESOLUTION_11_BIT:
		return 375;
	default:
		return 750;
	}
}

bool ds18b20_isConnected(const DeviceAddress *deviceAddress, uint8_t *scratchPad) {
	bool b = ds18b20_readScratchPad(deviceAddress, scratchPad);
	return b && !ds18b20_isAllZeros(scratchPad) && (ds18b20_crc8(scratchPad, 8) == scratchPad[SCRATCHPAD_CRC]);
}

uint8_t ds18b20_crc8(const uint8_t *addr, uint8_t len){
	uint8_t crc = 0;
	while (len--) {
		crc = *addr++ ^ crc;  // just re-using crc as intermediate
		crc = pgm_read_byte(dscrc2x16_table + (crc & 0x0f)) ^
		pgm_read_byte(dscrc2x16_table + 16 + ((crc >> 4) & 0x0f));
	}
	return crc;
}

bool ds18b20_isAllZeros(const uint8_t * const scratchPad) {
	for (size_t i = 0; i < 9; i++) {
		if (scratchPad[i] != 0) {
			return false;
		}
	}
	return true;
}

esp_err_t ds18b20_getTempC(const DeviceAddress *deviceAddress, float *temp) {

	ScratchPad scratchPad;
	ds18b20_requestTemperatures();

	if (ds18b20_isConnected(deviceAddress, scratchPad)){
		int16_t rawTemp = calculateTemperature(deviceAddress, scratchPad);
		// C = RAW/128
		// F = (C*1.8)+32 = (RAW/128*1.8)+32 = (RAW*0.0140625)+32
		*temp =  rawTemp/128.0f;
	} else {
    	*temp = DEVICE_DISCONNECTED_C;
	}
	return ESP_OK;
}

// reads scratchpad and returns fixed-point temperature, scaling factor 2^-7
int16_t calculateTemperature(const DeviceAddress *deviceAddress, uint8_t* scratchPad) {
	int16_t fpTemperature = (((int16_t) scratchPad[TEMP_MSB]) << 11) | (((int16_t) scratchPad[TEMP_LSB]) << 3);
	return fpTemperature;
}

esp_err_t ds18b20_init(int GPIO) {
	ESP_LOGI(TAG, "Initializating DS18B20");

    esp_rom_gpio_pad_select_gpio(GPIO);
    DS_GPIO = GPIO; 
    return ESP_OK;
}

void ds18b20_reset_search() {
	devices=0;
	// reset the search state
	LastDiscrepancy = 0;
	LastDeviceFlag = false;
	LastFamilyDiscrepancy = 0;
	for (int i = 7; i >= 0; i--) {
		ROM_NO[i] = 0;
	}
}

bool ds18b20_search(uint8_t *newAddr, bool search_mode) {
	uint8_t id_bit_number;
	uint8_t last_zero, rom_byte_number;
	bool search_result;
	uint8_t id_bit, cmp_id_bit;

	unsigned char rom_byte_mask, search_direction;

	// initialize for search
	id_bit_number = 1;
	last_zero = 0;
	rom_byte_number = 0;
	rom_byte_mask = 1;
	search_result = false;

	// if the last call was not the last one
	if (!LastDeviceFlag) {
		// 1-Wire reset
		if (!ds18b20_reset()) {
			// reset the search
			LastDiscrepancy = 0;
			LastDeviceFlag = false;
			LastFamilyDiscrepancy = 0;
			return false;
		}

		// issue the search command
		if (search_mode == true) {
			ds18b20_write_byte(0xF0);   // NORMAL SEARCH
		} else {
			ds18b20_write_byte(0xEC);   // CONDITIONAL SEARCH
		}

		// loop to do the search
		do {
			// read a bit and its complement
			id_bit = ds18b20_read();
			cmp_id_bit = ds18b20_read();

			// check for no devices on 1-wire
			if ((id_bit == 1) && (cmp_id_bit == 1)) {
				break;
			} else {
				// all devices coupled have 0 or 1
				if (id_bit != cmp_id_bit) {
					search_direction = id_bit;  // bit write value for search
				} else {
					// if this discrepancy if before the Last Discrepancy
					// on a previous next then pick the same as last time
					if (id_bit_number < LastDiscrepancy) {
						search_direction = ((ROM_NO[rom_byte_number]
								& rom_byte_mask) > 0);
					} else {
						// if equal to last pick 1, if not then pick 0
						search_direction = (id_bit_number == LastDiscrepancy);
					}
					// if 0 was picked then record its position in LastZero
					if (search_direction == 0) {
						last_zero = id_bit_number;

						// check for Last discrepancy in family
						if (last_zero < 9)
							LastFamilyDiscrepancy = last_zero;
					}
				}

				// set or clear the bit in the ROM byte rom_byte_number
				// with mask rom_byte_mask
				if (search_direction == 1)
					ROM_NO[rom_byte_number] |= rom_byte_mask;
				else
					ROM_NO[rom_byte_number] &= ~rom_byte_mask;

				// serial number search direction write bit
				ds18b20_write(search_direction);

				// increment the byte counter id_bit_number
				// and shift the mask rom_byte_mask
				id_bit_number++;
				rom_byte_mask <<= 1;

				// if the mask is 0 then go to new SerialNum byte rom_byte_number and reset mask
				if (rom_byte_mask == 0) {
					rom_byte_number++;
					rom_byte_mask = 1;
				}
			}
		} while (rom_byte_number < 8);  // loop until through all ROM bytes 0-7

		// if the search was successful then
		if (!(id_bit_number < 65)) {
			// search successful so set LastDiscrepancy,LastDeviceFlag,search_result
			LastDiscrepancy = last_zero;

			// check for last device
			if (LastDiscrepancy == 0) {
				LastDeviceFlag = true;
			}
			search_result = true;
		}
	}

	// if no device found then reset counters so next 'search' will be like a first
	if (!search_result || !ROM_NO[0]) {
		devices=0;
		LastDiscrepancy = 0;
		LastDeviceFlag = false;
		LastFamilyDiscrepancy = 0;
		search_result = false;
	} else {
		for (int i = 0; i < 8; i++){
			newAddr[i] = ROM_NO[i];
		}
		devices++;
	}
	return search_result;
}