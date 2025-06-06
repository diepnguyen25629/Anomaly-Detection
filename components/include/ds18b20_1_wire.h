
#include <esp_system.h>

#ifndef DS18B20_H_  
#define DS18B20_H_

// OneWire commands
#define GETTEMP			0x44  // Tells device to take a temperature reading and put it on the scratchpad
#define SKIPROM			0xCC  // Command to address all devices on the bus
#define SELECTDEVICE	0x55  // Command to address all devices on the bus
#define COPYSCRATCH     0x48  // Copy scratchpad to EEPROM
#define READSCRATCH     0xBE  // Read from scratchpad
#define WRITESCRATCH    0x4E  // Write to scratchpad
#define RECALLSCRATCH   0xB8  // Recall from EEPROM to scratchpad
#define READPOWERSUPPLY 0xB4  // Determine if device needs parasite power
#define ALARMSEARCH     0xEC  // Query bus for devices with an alarm condition
// Scratchpad locations
#define TEMP_LSB        0
#define TEMP_MSB        1
#define HIGH_ALARM_TEMP 2
#define LOW_ALARM_TEMP  3
#define CONFIGURATION   4
#define INTERNAL_BYTE   5
#define COUNT_REMAIN    6
#define COUNT_PER_C     7
#define SCRATCHPAD_CRC  8
// DSROM FIELDS
#define DSROM_FAMILY    0
#define DSROM_CRC       7
// Device resolution
#define TEMP_9_BIT  0x1F //  9 bit
#define TEMP_10_BIT 0x3F // 10 bit
#define TEMP_11_BIT 0x5F // 11 bit
#define TEMP_12_BIT 0x7F // 12 bit

#define DEVICE_DISCONNECTED_C -127


#define pgm_read_byte(addr)   (*(const unsigned char *)(addr))

typedef uint8_t DeviceAddress[8];
typedef uint8_t ScratchPad[9];

static const uint8_t dscrc2x16_table[] = {
	0x00, 0x5E, 0xBC, 0xE2, 0x61, 0x3F, 0xDD, 0x83,
	0xC2, 0x9C, 0x7E, 0x20, 0xA3, 0xFD, 0x1F, 0x41,
	0x00, 0x9D, 0x23, 0xBE, 0x46, 0xDB, 0x65, 0xF8,
	0x8C, 0x11, 0xAF, 0x32, 0xCA, 0x57, 0xE9, 0x74
};

#define noInterrupts() portMUX_TYPE mux = portMUX_INITIALIZER_UNLOCKED;taskENTER_CRITICAL(&mux)
#define interrupts() taskEXIT_CRITICAL(&mux)

typedef enum {
	RESOLUTION_9_BIT = 0,
	RESOLUTION_10_BIT,
	RESOLUTION_11_BIT,
	RESOLUTION_12_BIT
} temp_resolution;

typedef struct {
	temp_resolution temp_resol;
	uint8_t temp_sample_rate;
} ds18b20_config_t;

#define ds18b20_send ds18b20_write
#define ds18b20_send_byte ds18b20_write_byte
#define ds18b20_RST_PULSE ds18b20_reset


esp_err_t ds18b20_init(int GPIO);

esp_err_t ds18b20_write(char bit);
unsigned char ds18b20_read(void);
esp_err_t ds18b20_write_byte(char data);
unsigned char ds18b20_read_byte(void);
unsigned char ds18b20_reset(void);

bool ds18b20_setResolution(const DeviceAddress tempSensorAddresses[], int numAddresses, uint8_t newResolution);
bool ds18b20_isConnected(const DeviceAddress *deviceAddress, uint8_t *scratchPad);
esp_err_t ds18b20_writeScratchPad(const DeviceAddress *deviceAddress, const uint8_t *scratchPad);
bool ds18b20_readScratchPad(const DeviceAddress *deviceAddress, uint8_t *scratchPad);
esp_err_t ds18b20_select(const DeviceAddress *address);
uint8_t ds18b20_crc8(const uint8_t *addr, uint8_t len);
bool ds18b20_isAllZeros(const uint8_t * const scratchPad);
bool isConversionComplete();
uint16_t millisToWaitForConversion();

esp_err_t ds18b20_requestTemperatures();
esp_err_t ds18b20_getTempC(const DeviceAddress *deviceAddress, float *temp);
int16_t calculateTemperature(const DeviceAddress *deviceAddress, uint8_t* scratchPad);

void ds18b20_reset_search();
bool ds18b20_search(uint8_t *newAddr, bool search_mode);

#endif