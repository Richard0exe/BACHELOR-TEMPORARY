#include "ads1220.hpp"

#include "pico/stdlib.h"
#include "hardware/spi.h"

using namespace ADS1220_REG;

ADS1220::ADS1220(spi_inst_t *spi, uint cs, uint drdy)
    : spi(spi), cs_pin(cs), drdy_pin(drdy) {}

void ADS1220::writeRegister(uint8_t reg_addr, uint8_t value)
{
    uint8_t cmd = CMD_WREG_BASE | ((reg_addr & REG_MASK) << 2) | 0x00; // nn (number of bytes to be written - 1)
    uint8_t tx[2] = {cmd, value};

    csSelect();
    sleep_us(1);
    spi_write_blocking(spi, tx, 2);
    sleep_us(1);
    csDeselect();
}

uint8_t ADS1220::readRegister(uint8_t reg_addr)
{
    uint8_t cmd = CMD_RREG_BASE | ((reg_addr & REG_MASK) << 2) | 0x00;
    uint8_t rx = 0x00;

    csSelect();
    sleep_us(1);
    spi_write_blocking(spi, &cmd, 1);
    spi_read_blocking(spi, 0x00, &rx, 1);
    sleep_us(1);
    csDeselect();

    return rx;
}

bool ADS1220::waitForDRDY(uint32_t timeout_ms)
{
    if (drdy_pin == UINT32_MAX)
    {
        return true;
    }
    uint32_t start = to_ms_since_boot(get_absolute_time());

    while (gpio_get(drdy_pin) == 1)
    {
        if ((to_ms_since_boot(get_absolute_time()) - start) >= timeout_ms)
        {
            return false; // timeout
        }
        // sleep_ms(1); // small timeout to avoid busy-waiting DONT WORK WITH 2000SPS
    }
    return true; // DRDY went LOW -> data ready
}

int32_t ADS1220::readDataRaw(bool wait_for_drdy)
{
    if (wait_for_drdy && !waitForDRDY(100))
    {
        return 1;
    }
    uint8_t cmd = CMD_RDATA; // 0001 xxxx
    uint8_t rx[3] = {0};
    csSelect();
    sleep_us(1);
    spi_write_blocking(spi, &cmd, 1);
    spi_read_blocking(spi, 0x00, rx, 3);
    sleep_us(1);
    csDeselect();
    int32_t raw = ((int32_t)rx[0] << 16 | (int32_t)rx[1] << 8 | (int32_t)rx[2]); // first is MSB last MSB
    if (raw & 0x800000)
    {                      // check if negative, first bit 1
        raw |= 0xFF000000; // sign extend
    }
    return raw;
}

void ADS1220::init(uint32_t baud_hz, uint8_t mode, uint data_bits)
{
    spi_baud_hz = baud_hz;
    spi_mode = mode;
    spi_data_bits = data_bits;

    // init SPI
    spi_init(spi, baud_hz);

    // Set SPI format (Mode 1 is fixed)
    spi_set_format(spi, spi_data_bits, SPI_CPOL_0, SPI_CPHA_1, SPI_MSB_FIRST);

    // setup CS and DRDY
    gpio_init(cs_pin);
    gpio_set_dir(cs_pin, GPIO_OUT);
    gpio_put(cs_pin, 1);

    if (drdy_pin != UINT32_MAX)
    {
        gpio_init(drdy_pin);
        gpio_set_dir(drdy_pin, GPIO_IN);
    }
}

void ADS1220::start()
{
    sendCommand(CMD_START_SYNC);
}

void ADS1220::reset()
{
    sendCommand(CMD_RESET);
    // Wait at least 50 \u00b5s + 32 * t(CLK) before sending any other command.
    // t(CLK) = 1 / spi_baud_hz seconds -> in microseconds = 1e6 / spi_baud_hz
    // If spi_baud_hz is not configured, assume 1 MHz.
    uint32_t baud = (spi_baud_hz != 0) ? spi_baud_hz : 1000000u;
    uint32_t extra_us = (32u * 1000000u + baud - 1u) / baud; // ceil (32 * 1e6 / baud)
    sleep_us(50 + extra_us);
}

void ADS1220::powerDown()
{
    sendCommand(CMD_POWERDOWN);
}

void ADS1220::sendCommand(uint8_t cmd)
{
    csSelect();
    sleep_us(1);
    spi_write_blocking(spi, &cmd, 1);
    sleep_us(1);
    csDeselect();
}

void ADS1220::printRegisterValues()
{
    printf("Registers Config0-4 values in HEX:\n");
    printf("%#04x\n", readRegister(CONFIG0));
    printf("%#04x\n", readRegister(CONFIG1));
    printf("%#04x\n", readRegister(CONFIG2));
    printf("%#04x\n", readRegister(CONFIG3));
}

uint8_t ADS1220::gainValueFromBits(uint8_t gain_bits)
{
    switch (gain_bits >> Config0::GAIN_SHIFT & 0x07)
    { // shift bits to right 1 bit, to have bits from 0 position,
      // then use 0111 mask to take only 3 bits
    case 0b000:
        return 1;
    case 0b001:
        return 2;
    case 0b010:
        return 4;
    case 0b011:
        return 8;
    case 0b100:
        return 16;
    case 0b101:
        return 32;
    case 0b110:
        return 64;
    case 0b111:
        return 128;
    }
    return 1; // default to 1, avoid division by zero
}

float ADS1220::rawToVoltage(int32_t raw, float v_ref, uint8_t gain)
{
    if (gain <= 0 || gain > 128)
    {
        return 0.0f;
    }
    if (v_ref <= 0.0f)
    {
        return 0.0f;
    }
    return (static_cast<float>(raw) / ADC_FULL_SCALE * (v_ref / gain));
}

bool ADS1220::isDataReady(bool wait)
{
    if (wait) // never use the wait block, always call isDataReady(false)
    {
        uint32_t start = time_us_32();
        while (gpio_get(drdy_pin))
        {
            if (time_us_32() - start > 10000) // 10ms timeout
            {
                return false; // Timeout
            }
            tight_loop_contents();
        }
        return true;
    }
    else
    {
        return !gpio_get(drdy_pin);
    }
}
