#pragma once

// define SPI ports
#define SPI_ADC spi0
#define SPI_SD spi1

// ADS1220 pins assignment for RPi Pico
#define MISO_PIN 16
#define MOSI_PIN 19
#define SCK_PIN 18
// First ADS1220
#define CS_PIN 17
#define DRDY_PIN 20

// Second ADS1220
#define DRDY2_PIN 8
#define CS2_PIN 7

// Third ADS1220
#define DRDY3_PIN 27
#define CS3_PIN 26

// Fourth ADS1220
#define DRDY4_PIN 22
#define CS4_PIN 21

#define LED_PIN 15 // inbuilt use 25

// PINS for SD-Card module
#define MISO_PIN_SD 12
#define MOSI_PIN_SD 11
#define SCK_PIN_SD 10
#define CS_PIN_SD 13

// I2C pins
#define SDA_PIN 0
#define SCL_PIN 1

// TODO: Implement mode switching button
#define MODE_SELECT_PIN 14 // dummy button now ONLY FOR TESTING
