#ifndef __ADS1220__
#define __ADS1220__

#include <stdio.h>
#include <stdint.h>
#include "pico/stdlib.h"
#include "hardware/spi.h"
#include "ads1220_registers.h"

/* FOR MORE SPECIFIED INFO READ ADS1220 CHIP DOCUMENTATION
link : https://www.ti.com/product/ADS1220
*/

#define ADC_FULL_SCALE (8388608.0f)

class ADS1220
{
public:
    ADS1220(spi_inst_t *spi, uint cs_pin, uint drdy_pin);
    /*
    @brief Function for writing to one of 4 registers
    @param reg_addr Adress of 1 of 4 registers (0x00, 0x01, 0x02, 0x03)
    @param value 1 byte which we want to send where bits represent configuration
    */
    void writeRegister(uint8_t reg_addr, uint8_t value);
    /*
    @brief Read data from one of 4 available registers
    @param reg_addr Adress of 1 of 4 registers (0x00, 0x01, 0x02, 0x03)
    @return Returns the byte information of choosed register
    */
    uint8_t readRegister(uint8_t reg_addr);

    /*
    @brief Initialize the SPI communication
    @param baud_hz Communication bandwidth in Hz
    @param spi_mode Sets the spi_mode to 1 by default
    @param data_bits Sets the number of bits per transfer
    */
    void init(uint32_t baud_hz = 1000000, uint8_t spi_mode = 1, uint data_bits = 8); /// spi_mode is fixed to 1 (CPOL = 0, CPHA = 1)

    /*
    @brief Sends command RESET;
    Command byte: 0000 011x
    */
    void reset();
    /*
    @brief Sends command START/SYNC;
    Command byte: 0000 100x
    */
    /// Reset device
    void start();
    /*
    @brief Sends command POWERDOWN
    Command byte: 0000 001x
    */
    /// Start or restart conversions
    void powerDown(); /// Enter power-down mode

    /*
    @brief Waiting when DRDRY is LOW, then data is ready to be read
    @param timeout_ms Timeout in milliseconds (default 100)
    @return true if DRDY asserted before timeout; false on timeout
    */
    bool waitForDRDY(uint32_t timeout_ms = 100); /// Waiting when data is ready after conversion
    /*
    @brief Read 24-bit raw ADC result.
    @param wait_for_drdy If true, block until DRDY is asserted (timeout handled by waitForDRDY)
    @return Signed 24-bit sample packed into a 32-bit int (sign-extended)
    */
    int32_t readDataRaw(bool wait_for_drdy = true); /// Reading 24bit long raw data

    /*
    @brief Convert the raw data reading to voltage
    @param raw Raw value of ADC reading
    @param v_ref ADC reference voltage
    @param gain Choosed gain 1-128x
    */
    float rawToVoltage(int32_t raw, float v_ref, uint8_t gain);
    /*
    @brief Prints the values of all registers
    */
    void printRegisterValues();

    /*
    @brief Convert the GAIN bits to integer 1-128
    @param gain_bits Use the ADS1220 defined constants in Config0 namespace
    @return Returns the uint between 1 to 128x
    */
   uint8_t gainValueFromBits(uint8_t gain_bits);
   bool isDataReady(bool wait);

private:
    spi_inst_t *spi = nullptr;
    uint cs_pin = 0;
    uint drdy_pin = UINT32_MAX;

    uint32_t spi_baud_hz = 0;
    uint8_t spi_mode = 1; // Only SPI mode 1 (CPOL = 0, CPHA = 1) is supported.
    uint spi_data_bits = 8;

    // Chip select (CS) is an active-low input that selects the device for SPI communication. Set CS to LOW before sending or receiving
    inline void csSelect() { gpio_put(cs_pin, 0); }
    // Chip select (CS) is an active-low input that selects the device for SPI communication. Set CS to HIGH after sending or receiving
    inline void csDeselect() { gpio_put(cs_pin, 1); }
    void sendCommand(uint8_t cmd);
};
#endif
