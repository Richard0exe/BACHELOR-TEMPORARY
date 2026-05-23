#ifndef CONFIGURATION_H
#define CONFIGURATION_H

#include <stdint.h>

//#define DEBUG_MODE
#define PRODUCTION_MODE

// Define DUAL_CHANNEL_MODE to enable collecting the data from all adc in our case 4 ADCs, 8 channels.
#define DUAL_CHANNEL_MODE
// If this is commented out, the code will run in single ADC mode.1
#define NUM_ADCS 4

// ADC Configuration
// SPI Settings
#define ADC_SPI_FREQ 1000000 // 1MHz
#define ADC_SPI_MODE 1
#define ADC_BITS_TRANSFER 8

// Voltage reference
#define VREF_VOLTAGE (3.3f) // only used by 1 function, we use the 4.5V from 3 AA batteries

// CONFIG0
#define ADC_MUX ADS1220_REG::Config0::MUX_AIN0_AIN1
#define ADC_GAIN ADS1220_REG::Config0::GAIN_64
#define ADC_PGA_BYPASS false

// CONFIG1
#define ADC_DATA_RATE ADS1220_REG::Config1::DR_1000SPS
#define ADC_MODE ADS1220_REG::Config1::MODE_TURBO
#define ADC_CONV_MODE true  // Continuous conversion
#define ADC_TEMP_MODE false // Temperature sensor off
#define ADC_BURNOUT false   // Burn-out current off

// CONFIG2
#define ADC_VREF ADS1220_REG::Config2::VREF_AVDD_AVSS
#define ADC_FIR ADS1220_REG::Config2::FIR_NONE
#define ADC_PSW false // PSW open
#define ADC_IDAC ADS1220_REG::Config2::IDAC_OFF

#if NUM_ADCS == 4
// For 4 ADCs at 2000SPS:
// Rate: 2000 combined records/sec (8000 total samples/sec)
// Record size: 20 bytes (timestamp, raw1, raw2, raw3, raw4)
// Data rate: 2000 * 20 = 40000 bytes/sec
// A 65536 byte buffer (2^16) will hold ~1.64 seconds of data.
#define BUFFER_SIZE 65536

#elif NUM_ADCS == 3
// For 3 ADCs at 2000SPS:
// 3 ADCs * 2000 SPS = 2000 combined records/sec
// 1 record = 16 bytes (timestamp, raw1, raw2, raw3)
// Data rate = 2000 * 16 = 32000 bytes/sec
// A 32768 byte buffer (2^15) will hold ~1.02 seconds of data.
#define BUFFER_SIZE 32768
#elif NUM_ADCS == 2
// For DUAL_ADC_MODE at 2000SPS:
// 2 ADCs * 2000 SPS = 2000 combined records/sec
// 1 record = 12 bytes (timestamp, raw1, raw2)
// Data rate = 2000 * 12 = 24000 bytes/sec
// A 32768 byte buffer (2^15) will hold ~1.36 seconds of data.
#define BUFFER_SIZE 32768
#else
// For single ADC mode at 2000SPS:
// 1 ADC * 2000 SPS = 2000 records/sec
// 1 record = 8 bytes (timestamp, raw_value)
// Data rate = 2000 * 8 = 16000 bytes/sec
// A 16384 byte buffer (2^14) will hold ~1.02 seconds of data.
#define BUFFER_SIZE 16384
#endif

#define CHUNK_SIZE 256
#define BASE_FILENAME "brauciens"
#define MAX_FILENAME_LEN 32

// TODO
enum SystemState
{
    STATE_IDLE = 0,
    STATE_RECORDING = 1,
    STATE_TRANSMITTING = 2
};

#pragma pack(push, 1)
struct ADCRecord_Single
{
    uint32_t timestamp_us;
    int32_t raw_value;
};

struct ADCRecord_Dual
{
    uint32_t timestamp_us;
    int32_t raw_value1;
    int32_t raw_value2;
};

struct ADCRecord_Triple
{
    uint32_t timestamp_us;
    int32_t raw_value1;
    int32_t raw_value2;
    int32_t raw_value3;
};

struct ADCRecord_Quad
{
    uint32_t timestamp_us;
    int32_t raw_value1;
    int32_t raw_value2;
    int32_t raw_value3;
    int32_t raw_value4;
};

struct ADCRecord_Octa
{
    uint32_t timestamp_us;
    int32_t raw_value1; // ADC1 AIN0/1
    int32_t raw_value2; // ADC1 AIN2/3
    int32_t raw_value3; // ADC2 AIN0/1
    int32_t raw_value4; // ADC2 AIN2/3
    int32_t raw_value5; // ADC3 AIN0/1
    int32_t raw_value6; // ADC3 AIN2/3
    int32_t raw_value7; // ADC4 AIN0/1
    int32_t raw_value8; // ADC4 AIN2/3
};

#pragma pack(pop)

#if NUM_ADCS == 4
#ifdef DUAL_CHANNEL_MODE
        typedef ADCRecord_Octa ADCRecord;
    #else
        typedef ADCRecord_Quad ADCRecord;
    #endif
#elif NUM_ADCS == 3
typedef ADCRecord_Triple ADCRecord;
#elif NUM_ADCS == 2
typedef ADCRecord_Dual ADCRecord;
#else
typedef ADCRecord_Single ADCRecord;
#endif

extern volatile bool buffer_ready;
extern uint64_t start_time;
extern char filename[MAX_FILENAME_LEN];

#endif