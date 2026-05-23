#include "pico/stdlib.h"
#include "hardware/spi.h"
#include "hardware/i2c.h"
#include "pico/multicore.h"
#include <cstring>
#include <string>
#include <vector>
#include <algorithm>

// FatFS includes
#include "ff.h"
#include "diskio.h"
#include "f_util.h"
#include "hw_config.h"
#include "sd_card.h"

// Project includes
#include "ads1220.hpp"
#include "ads1220_pins.hpp"
#include "config.hpp"
#include "i2c_commands.hpp"

// To use shorter name when using structs with ADS1220 configuration register constants
using namespace ADS1220_REG;

// Buffer management
static char buffer1[BUFFER_SIZE];
static char buffer2[BUFFER_SIZE];
static char *volatile active_buffer = buffer1;
static char *volatile write_buffer = buffer2;
static volatile int active_index = 0;
static volatile int write_size = 0;
volatile bool buffer_ready = false;

// SD card variables
static FRESULT fr;
static FATFS fs;
static FIL file;
static int counter_filename = 1;
char filename[MAX_FILENAME_LEN];
uint64_t start_time;

// Hardware objects
static ADS1220 *adc1 = nullptr;
#if NUM_ADCS >= 2
static ADS1220 *adc2 = nullptr;
#endif
#if NUM_ADCS >= 3
static ADS1220 *adc3 = nullptr;
#endif
#if NUM_ADCS >= 4
static ADS1220 *adc4 = nullptr;
#endif

static bool hardware_initialized = false;
static bool core1_launched = false;

// Ride variables
double elapsed_seconds = 0;
uint32_t elapsed_time = 0;

// System state and commands flags
volatile SystemState current_state = STATE_IDLE;
volatile bool button_pressed = false;
volatile bool cmd_start_recording = false;
volatile bool cmd_stop_recording = false;
volatile bool cmd_send_data = false;
volatile bool cmd_next_file = false;
volatile bool cmd_send_chunk = false;

// Button debouncing
volatile uint64_t last_button_press_time = 0;
const uint DEBOUNCE_DELAY_MS = 200;

// File transfer variables
static std::vector<std::string> filelist;
static size_t current_file_index = 0;
static FIL send_file;
static bool file_opened = false;

// ADS1220 MUX Channel 2 channels selected
bool current_channel_one = true;

void button_callback(uint gpio, uint32_t events)
{
    uint64_t now = time_us_64();
    if ((now - last_button_press_time) > (DEBOUNCE_DELAY_MS * 1000))
    {
        last_button_press_time = now;
        button_pressed = true;
    }
}

// I2C documentation from Pico-SDK:
// https://github.com/raspberrypi/pico-sdk/blob/master/src/rp2040/hardware_structs/include/hardware/structs/i2c.h
void i2c_irq_handler()
{
    // Get hardware registers
    uint32_t status = i2c_get_hw(I2C_SLAVE)->intr_stat;

    // Use mask to see in interrupt register if we received which we wanted
    if (status & I2C_IC_INTR_STAT_R_RX_FULL_BITS)
    {
        if (i2c_get_read_available(I2C_SLAVE))
        {
            uint8_t command = i2c_read_byte_raw(I2C_SLAVE);

            // Parse command and set flag (FAST!)
            switch (command)
            {
            case CMD_START:
                cmd_start_recording = true;
                break;
            case CMD_STOP:
                cmd_stop_recording = true;
                break;
            case CMD_SEND_DATA:
                cmd_send_data = true;
                break;
            case CMD_NEXT_FILE:
                cmd_next_file = true;
                break;
            case CMD_GET_CHUNK:
                cmd_send_chunk = true;
            }
        }
    }
    i2c_get_hw(I2C_SLAVE)->clr_intr;
}

void setup_button()
{
    gpio_init(MODE_SELECT_PIN);
    gpio_set_dir(MODE_SELECT_PIN, GPIO_IN);
    gpio_pull_up(MODE_SELECT_PIN);
    gpio_set_irq_enabled_with_callback(
        MODE_SELECT_PIN,
        GPIO_IRQ_EDGE_FALL, // Trigger on button press
        true,
        &button_callback);
}

void i2c_slave_setup()
{
    gpio_set_function(SDA_PIN, GPIO_FUNC_I2C);
    gpio_set_function(SCL_PIN, GPIO_FUNC_I2C);
    gpio_pull_up(SDA_PIN);
    gpio_pull_up(SCL_PIN);

    i2c_init(I2C_SLAVE, I2C_BAUD_RATE);
    i2c_set_slave_mode(I2C_SLAVE, true, I2C_SLAVE_ADDRESS);
    i2c_get_hw(I2C_SLAVE)->intr_mask = I2C_IC_INTR_MASK_M_RX_FULL_BITS;

    irq_set_exclusive_handler(I2C0_IRQ, i2c_irq_handler);
    irq_set_enabled(I2C0_IRQ, true);

    printf("I2C slave initialized at address 0x%02X with interrupts\n", I2C_SLAVE_ADDRESS);
}

uint32_t write_count = 0;
void core1_main()
{
    while (true)
    {
        // if buffer ready, start writing to file then mark that buffer is not ready
        if (buffer_ready)
        {
            UINT bw;

            uint32_t write_start = time_us_32();
            FRESULT res = f_write(&file, write_buffer, write_size, &bw);

            uint32_t write_duration = time_us_32() - write_start;
            write_count++;
            if (res != FR_OK || bw != write_size)
            {
                printf("Write error: %d\n", res);
            }
            // DEBUG info
            else
            {
                printf("✓ Buffer #%lu written: %d bytes in %lu us (%.2f KB/s)\n",
                       write_count,
                       bw,
                       write_duration,
                       (bw / (write_duration / 1000000.0)) / 1024.0);
            }

            // Sync the memory every 3 buffers to work without button
            if (write_count % 3 == 0)
            {
                uint32_t sync_start = time_us_32();
                f_sync(&file);
                printf("  -> FAT Table Synced in %lu us\n", time_us_32() - sync_start);
            }
            // f_sync(&file);
            buffer_ready = false; // Signal Core 0: Ready for next buffer
        }
        sleep_ms(1);
    }
}

void init_SD()
{
    spi_init(SPI_SD, 10 * 1000 * 1000); // 10 MHz
    gpio_set_function(MISO_PIN_SD, GPIO_FUNC_SPI);
    gpio_set_function(MOSI_PIN_SD, GPIO_FUNC_SPI);
    gpio_set_function(SCK_PIN_SD, GPIO_FUNC_SPI);
    gpio_init(CS_PIN_SD);
    gpio_set_dir(CS_PIN_SD, GPIO_OUT);
    gpio_put(CS_PIN_SD, 1);
}

void configure_sensor(ADS1220 &adc)
{
    uint8_t cfg0 = Config0::pack(ADC_MUX, ADC_GAIN, ADC_PGA_BYPASS);
    uint8_t cfg1 = Config1::pack(ADC_DATA_RATE, ADC_MODE, ADC_CONV_MODE, ADC_TEMP_MODE, ADC_BURNOUT);
    uint8_t cfg2 = Config2::pack(ADC_VREF, ADC_FIR, ADC_PSW, ADC_IDAC);
    // Write configurations
    adc.writeRegister(CONFIG0, cfg0);
    adc.writeRegister(CONFIG1, cfg1);
    adc.writeRegister(CONFIG2, cfg2);
}
void init_sensor(ADS1220 &adc)
{
    // Initialize ADC with SPI settings
    adc.init(ADC_SPI_FREQ, ADC_SPI_MODE, ADC_BITS_TRANSFER);
    printf("SPI Initialized for ADC\n");

    // Reset the ADC
    adc.reset();
    printf("ADC Reset\n");
}

void init_hardware()
{
    if (hardware_initialized)
    {
        printf("Hardware already initialized, skipping...\n");
    }

    printf("Initializing Hardware\n");

    // Common SPI pins for both ADCs
    gpio_set_function(MISO_PIN, GPIO_FUNC_SPI);
    gpio_set_function(MOSI_PIN, GPIO_FUNC_SPI);
    gpio_set_function(SCK_PIN, GPIO_FUNC_SPI);

    // Initialize ADC1
    printf("Initializing ADC1...\n");
    adc1 = new ADS1220(SPI_ADC, CS_PIN, DRDY_PIN);
    init_sensor(*adc1);
    sleep_ms(100);
    configure_sensor(*adc1);
    sleep_ms(100);
    adc1->printRegisterValues();
    printf("ADC1 initialized\n");

#if NUM_ADCS >= 2
    // Initialize ADC2
    printf("Initializing ADC2...\n");
    adc2 = new ADS1220(SPI_ADC, CS2_PIN, DRDY2_PIN);
    init_sensor(*adc2);
    sleep_ms(100);
    configure_sensor(*adc2);
    sleep_ms(100);
    adc2->printRegisterValues();
    printf("ADC2 initialized\n");
#endif
#if NUM_ADCS >= 3
    // Initialize ADC3
    printf("Initializing ADC3...\n");
    adc3 = new ADS1220(SPI_ADC, CS3_PIN, DRDY3_PIN);
    init_sensor(*adc3);
    sleep_ms(100);
    configure_sensor(*adc3);
    sleep_ms(100);
    adc3->printRegisterValues();
    printf("ADC3 initialized\n");
#endif
#if NUM_ADCS >= 4
    // Initialize ADC4
    printf("Initializing ADC4...\n");
    adc4 = new ADS1220(SPI_ADC, CS4_PIN, DRDY4_PIN);
    init_sensor(*adc4);
    sleep_ms(100);
    configure_sensor(*adc4);
    sleep_ms(100);
    adc4->printRegisterValues();
    printf("ADC4 initialized\n");
#endif

#ifdef PRODUCTION_MODE
    // Initialize SD
    sd_init_driver();
    sleep_ms(5000);
    printf("SD driver initialized\n");

    // Mount filesystem once
    fr = f_mount(&fs, "0:", 1);
    if (fr != FR_OK)
    {
        printf("SD mount failed: %d\n", fr);
        while (true)
        {
            gpio_put(LED_PIN, 1);
            sleep_ms(300);
            gpio_put(LED_PIN, 0);
            sleep_ms(300);
            gpio_put(LED_PIN, 1);
            sleep_ms(300);
            gpio_put(LED_PIN, 0);
            sleep_ms(300);
            gpio_put(LED_PIN, 1);
            sleep_ms(300);
            gpio_put(LED_PIN, 0);
            sleep_ms(1000);
        }
    }
    printf("SD Card mounted\n");
#endif

    printf("Initializing I2C slave...\n");
    i2c_slave_setup();

    hardware_initialized = true;
    printf("=== Hardware Ready ===\n");
}

void generate_filename(char *buffer, size_t len, const char *base_name)
{
    FILINFO fno;
    snprintf(buffer, len, "%s.BIN", base_name);

    // find the right counter for filenames
    while (f_stat(buffer, &fno) == FR_OK)
    {
        counter_filename++;
        snprintf(buffer, len, "%s_%d.BIN", base_name, counter_filename);
    }
    printf("Generated filename: %s\n", buffer);
}

void check_server_commands()
{
    // TODO: Implement server communication
    // Check for commands like START_RECORDING, START_TRANSMITTING
    if (button_pressed)
    {
        button_pressed = false;
        printf("Button pressed! Starting RECORDING!\n");
        current_state = STATE_RECORDING;
        return;
    }

    if (cmd_start_recording)
    {
        cmd_start_recording = false;
        printf("I2C command: START recording\n");
        current_state = STATE_RECORDING;
        return;
    }

    if (cmd_send_data)
    {
        cmd_send_data = false;
        printf("IC2 command: TRANSMIT data\n");
        current_state = STATE_TRANSMITTING;
        return;
    }

    printf("Checking for server commands...\n");
    sleep_ms(100);
}

void handle_idle()
{
    printf("=== IDLE STATE - Listening for commands ===\n");

    // LED pattern: slow blink = idle
    while (current_state == STATE_IDLE)
    {
        gpio_put(LED_PIN, 0);
        sleep_ms(500);
        gpio_put(LED_PIN, 1);
        sleep_ms(500);

        // TODO: Implement communication with server
        // Listen for commands to transition to other states
        check_server_commands();
    }
}

void handle_recording()
{
    printf("=== SENSOR MODE - Recording Data ===\n");

#ifdef PRODUCTION_MODE
    generate_filename(filename, sizeof(filename), BASE_FILENAME);

    // Create the file with correct name
    fr = f_open(&file, filename, FA_WRITE | FA_CREATE_ALWAYS);
    if (fr != FR_OK)
    {
        while (true)
        {
            // Fast blink - file error
            gpio_put(LED_PIN, 1);
            sleep_ms(200);
            gpio_put(LED_PIN, 0);
            sleep_ms(200);
        }
    }
    printf("Recording to: %s\n", filename);

    // Launch SD writer on Core 1 ONLY ONCE
    if (!core1_launched)
    {
        multicore_launch_core1(core1_main);
        core1_launched = true;
        printf("Core1 launched\n");
    }

#endif

    // LED solid ON = recording
    gpio_put(LED_PIN, 1);
    start_time = time_us_64();
    // START DATA CONVERSIONS
    adc1->start();

#if NUM_ADCS >= 2
    adc2->start();
#endif
#if NUM_ADCS >= 3
    adc3->start();
#endif
#if NUM_ADCS >= 4
    adc4->start();
#endif
    printf("%d ADCs started - recording at 2000 SPS\n", NUM_ADCS);

#ifdef DEBUG_MODE
    while (current_state == STATE_RECORDING)
    {

        // Check if button pressed to stop recording
        if (button_pressed)
        {
            button_pressed = false;
            current_state = STATE_IDLE;
            printf("Recording stopped by button\n");
            break;
        }

        elapsed_time = time_us_64() - start_time;
        int32_t raw = adc->readDataRaw(true);

        if (raw == 1)
        {
            printf("DRDY timeout or error!\n");
            continue;
        }
        else
        {
            // Use external 3.3 reference
            float voltage = adc->rawToVoltage(raw, VREF_VOLTAGE, ADC_GAIN);
            printf("Raw=%ld, Time=%ld ,Voltage= %.6f V (%.3f mV)\n", raw, elapsed_time, voltage, voltage * 1000.0f);
        }

        sleep_ms(1);
    }
#endif

#ifdef PRODUCTION_MODE

    while (current_state == STATE_RECORDING)
    {

        // Check if button pressed to stop recording
        if (button_pressed)
        {
            button_pressed = false;
            current_state = STATE_IDLE;
            printf("Recording stopped by button\n");
            break;
        }

        if (cmd_stop_recording)
        {
            cmd_stop_recording = false;
            current_state = STATE_IDLE;
            printf("I2C command: STOP recording\n");
            break;
        }

        elapsed_time = time_us_64() - start_time;
        elapsed_seconds = elapsed_time / 1000000.0;
#if NUM_ADCS == 4
#ifdef DUAL_CHANNEL_MODE
        // PHASE 1: Read AIN0/AIN1 (Sensors 1, 3, 5, 7)
        uint8_t cfg_mux1 = Config0::pack(Config0::MUX_AIN0_AIN1, ADC_GAIN, ADC_PGA_BYPASS);
        adc1->writeRegister(CONFIG0, cfg_mux1);
        adc2->writeRegister(CONFIG0, cfg_mux1);
        adc3->writeRegister(CONFIG0, cfg_mux1);
        adc4->writeRegister(CONFIG0, cfg_mux1);

        while (!adc1->isDataReady(false) || !adc2->isDataReady(false) || !adc3->isDataReady(false) || !adc4->isDataReady(false))
        {
            tight_loop_contents();
        }

        elapsed_time = time_us_64() - start_time;

        int32_t r1 = adc1->readDataRaw(false); // ADC1 S1
        int32_t r3 = adc2->readDataRaw(false); // ADC2 S3
        int32_t r5 = adc3->readDataRaw(false); // ADC3 S5
        int32_t r7 = adc4->readDataRaw(false); // ADC4 S7


        // PHASE 2: Read AIN2/AIN3 (Sensors 2, 4, 6, 8)
        uint8_t cfg_mux2 = Config0::pack(Config0::MUX_AIN2_AIN3, ADC_GAIN, ADC_PGA_BYPASS);
        adc1->writeRegister(CONFIG0, cfg_mux2);
        adc2->writeRegister(CONFIG0, cfg_mux2);
        adc3->writeRegister(CONFIG0, cfg_mux2);
        adc4->writeRegister(CONFIG0, cfg_mux2);

        while (!adc1->isDataReady(false) || !adc2->isDataReady(false) || !adc3->isDataReady(false) || !adc4->isDataReady(false))
        {
            tight_loop_contents();
        }

        int32_t r2 = adc1->readDataRaw(false); // ADC1 S2
        int32_t r4 = adc2->readDataRaw(false); // ADC2 S4
        int32_t r6 = adc3->readDataRaw(false); // ADC3 S6
        int32_t r8 = adc4->readDataRaw(false); // ADC4 S8

        if (r1 == 1 || r2 == 1 || r3 == 1 || r4 == 1 || r5 == 1 || r6 == 1 || r7 == 1 || r8 == 1)
            continue;

        ADCRecord rec = {
            .timestamp_us = elapsed_time,
            .raw_value1 = r1,
            .raw_value2 = r2,
            .raw_value3 = r3,
            .raw_value4 = r4,
            .raw_value5 = r5,
            .raw_value6 = r6,
            .raw_value7 = r7,
            .raw_value8 = r8};
#else
        // Wait for all four ADCs
        while (!adc1->isDataReady(false) || !adc2->isDataReady(false) || !adc3->isDataReady(false) || !adc4->isDataReady(false))
        {
            tight_loop_contents();
        }

        elapsed_time = time_us_64() - start_time;

        int32_t raw1 = adc1->readDataRaw(false);
        int32_t raw2 = adc2->readDataRaw(false);
        int32_t raw3 = adc3->readDataRaw(false);
        int32_t raw4 = adc4->readDataRaw(false);

        if (raw1 == 1 || raw2 == 1 || raw3 == 1 || raw4 == 1)
        {
            continue;
        }

        ADCRecord rec = {
            .timestamp_us = elapsed_time,
            .raw_value1 = raw1,
            .raw_value2 = raw2,
            .raw_value3 = raw3,
            .raw_value4 = raw4};
#endif

#elif NUM_ADCS == 3
        // Wait for all three ADCs
        while (!adc1->isDataReady(false) || !adc2->isDataReady(false) || !adc3->isDataReady(false))
        {
            tight_loop_contents();
        }

        elapsed_time = time_us_64() - start_time;

        int32_t raw1 = adc1->readDataRaw(false);
        int32_t raw2 = adc2->readDataRaw(false);
        int32_t raw3 = adc3->readDataRaw(false);

        if (raw1 == 1 || raw2 == 1 || raw3 == 1)
        {
            continue;
        }

        ADCRecord rec = {
            .timestamp_us = elapsed_time,
            .raw_value1 = raw1,
            .raw_value2 = raw2,
            .raw_value3 = raw3};

#elif NUM_ADCS == 2
        // Wait for both ADCs to have data ready
        while (!adc1->isDataReady(false) || !adc2->isDataReady(false))
        {
            tight_loop_contents();
        }

        elapsed_time = time_us_64() - start_time;

        int32_t raw1 = adc1->readDataRaw(false);
        int32_t raw2 = adc2->readDataRaw(false);

        if (raw1 == 1 || raw2 == 1)
        {
            continue;
        }

        // Record for ADCs
        ADCRecord rec = {
            .timestamp_us = elapsed_time,
            .raw_value1 = raw1,
            .raw_value2 = raw2};
#else
        int32_t raw = adc1->readDataRaw(true);

        // Skip errors
        if (raw == 1)
        {
            continue;
        }

        ADCRecord rec = {
            .timestamp_us = elapsed_time,
            .raw_value = raw};
#endif
        // Common buffer writing logic for both modes
        if (active_index + sizeof(ADCRecord) >= BUFFER_SIZE)
        {
            // make the SWAP
            while (buffer_ready)
            {
                sleep_us(100);
            }
            char *temp = active_buffer;
            active_buffer = write_buffer;
            write_buffer = temp;

            write_size = active_index;
            active_index = 0;
            buffer_ready = true;
        }
        memcpy(active_buffer + active_index, &rec, sizeof(ADCRecord));
        active_index += sizeof(ADCRecord);
    }
#endif

    uint32_t total_time = time_us_64() - start_time;
    printf("\n=== Recording Statistics ===\n");
    printf("Total recording time: %.3f seconds\n", total_time / 1000000.0);
    printf("File: %s\n", filename);
    printf("============================\n\n");

    // Turn the ADC off, completes last conversion, keeps config registers
    adc1->powerDown();
#if NUM_ADCS >= 2
    adc2->powerDown();
#endif
#if NUM_ADCS >= 3
    adc3->powerDown();
#endif
#if NUM_ADCS >= 4
    adc4->powerDown();
#endif

    // Clean Up when recording stops, flush into file last readings
    if (active_index > 0)
    {
        printf("Flushing final %d bytes ...\n", active_index);
        while (buffer_ready)
        {
            sleep_us(100);
        }
        UINT bw;
        f_write(&file, active_buffer, active_index, &bw);
        printf("Flushed %d bytes\n", bw);
    }

    // Wait Core1 to end
    while (buffer_ready)
    {
        sleep_ms(10);
    }

    f_sync(&file);
    f_close(&file);
    printf("File closed: %s\n", filename);

    active_index = 0;
    write_size = 0;
    buffer_ready = false;

    write_count = 0; // reset buffer writing buffer count for debugging
    printf("Ready for next record\n");
}

void build_filelist()
{
    filelist.clear();
    DIR dir;
    FILINFO fno;

    FRESULT res = f_opendir(&dir, "0:");
    if (res == FR_OK)
    {
        while (f_readdir(&dir, &fno) == FR_OK && fno.fname[0])
        {
            if (!(fno.fattrib & AM_DIR))
            {
                filelist.push_back(std::string(fno.fname));
            }
        }
        f_closedir(&dir);
    }
    current_file_index = 0;
    printf("Built filelist: %zu files\n", filelist.size());
}

const char *get_next_filename()
{
    if (current_file_index < filelist.size())
    {
        return filelist[current_file_index++].c_str();
    }
    return "END"; // No more files
}

/*
Opens file for reading
*/
bool open_send_file(const char *fname)
{
    if (file_opened)
    {
        f_close(&send_file);
        file_opened = false;
    }

    if (f_open(&send_file, fname, FA_READ) == FR_OK)
    {
        file_opened = true;
        printf("Opened file for sending: %s\n", fname);
        return true;
    }
    printf("Failed to open file: %s\n", fname);
    return false;
}

static int build_chunk_and_header(uint8_t *tx_buf, uint8_t *flags_out)
{
    UINT br = 0;
    uint8_t flags = 0;

    FRESULT res = f_read(&send_file, tx_buf + HEADER_SIZE, CHUNK_SIZE, &br);

    if (res != FR_OK)
    {
        br = 0;
        flags |= FLAG_EOF;
    }
    else if (br < CHUNK_SIZE || f_eof(&send_file))
    {
        flags |= FLAG_EOF;
    }

    // Zero-pad bytes (if file ends at 100 bytes, pad 156 zeros)
    if (br < CHUNK_SIZE)
    {
        memset(tx_buf + HEADER_SIZE + br, 0, CHUNK_SIZE - br);
    }

    // Build header: [length_low, length_high, flags]
    tx_buf[0] = (uint8_t)(br & 0xFF);
    tx_buf[1] = (uint8_t)((br >> 8) & 0xFF);
    tx_buf[2] = flags;

    *flags_out = flags;

    printf("Chunk: br=%u, fptr=%lu, fsize=%lu, flags=0x%02X\n",
           (unsigned)br, (unsigned long)f_tell(&send_file),
           (unsigned long)f_size(&send_file), flags);

    return (int)br;
}

void handle_transmitting()
{
    printf("=== TRANSMITTING STATE - Sending Data ===\n");

    // LED pattern: rapid blink = transmitting
    while (current_state == STATE_TRANSMITTING)
    {
        gpio_put(LED_PIN, 1);
        sleep_ms(100);
        gpio_put(LED_PIN, 0);
        sleep_ms(100);

        if (cmd_send_data)
        {
            cmd_send_data = false;
            printf("I2C: Building file list...\n");
            build_filelist();
        }

        if (cmd_next_file)
        {
            cmd_next_file = false;
            const char *fname = get_next_filename();
            printf("I2C: Sending filename: %s\n", fname);

            uint8_t fname_buf[MAX_FILENAME];
            memset(fname_buf, 0, MAX_FILENAME);
            strncpy((char *)fname_buf, fname, MAX_FILENAME - 1);

            i2c_write_raw_blocking(I2C_SLAVE, fname_buf, MAX_FILENAME);

            if (strcmp(fname, "END") != 0)
            {
                open_send_file(fname);
            }
            else
            {
                current_state = STATE_IDLE;
                printf("All files transmitted, returning to IDLE\n");
            }
        }

        if (cmd_send_chunk)
        {
            cmd_send_chunk = false;
            // Local buffer
            static uint8_t tx_buf[HEADER_SIZE + CHUNK_SIZE];
            if (!file_opened)
            {
                // No file => send EOF marker
                tx_buf[0] = 0;
                tx_buf[1] = 0;
                tx_buf[2] = FLAG_EOF;
                memset(tx_buf + HEADER_SIZE, 0, CHUNK_SIZE);
                i2c_write_raw_blocking(I2C_SLAVE, tx_buf, HEADER_SIZE + CHUNK_SIZE);
                continue;
            }
            uint8_t flags = 0;
            build_chunk_and_header(tx_buf, &flags);

            i2c_write_raw_blocking(I2C_SLAVE, tx_buf, HEADER_SIZE + CHUNK_SIZE);

            if (flags & FLAG_EOF)
            {
                f_close(&send_file);
                file_opened = false;
                printf("File transmission complete\n");
            }
        }
    }
}

int main()
{
    stdio_init_all();

    // Initialize LED
    gpio_init(LED_PIN);
    gpio_set_dir(LED_PIN, GPIO_OUT);
    gpio_put(LED_PIN, 0); // start OFF

    setup_button(); // mostly for debugging (REAL application will use wireless commands)
    sleep_ms(100);

    init_hardware();

    // --- TRACKSIDE AUTO-START WITH DELAY ---
    printf("Power ON. Waiting 5 seconds for hardware to settle...\n");

    // 5-Second Countdown Blinker
    // Gives the mechanic time to close the box lid before recording starts
    for (int i = 0; i < 5; i++)
    {
        gpio_put(LED_PIN, 1);
        sleep_ms(500);
        gpio_put(LED_PIN, 0);
        sleep_ms(500);
        printf("Starting in %d...\n", 5 - i);
    }

    printf("Auto-starting recording NOW!\n");

    current_state = STATE_RECORDING; // STATE_IDLE it was used with button or some signal to start

    while (true)
    {
        switch (current_state)
        {
        case STATE_IDLE:
            handle_idle();
            break;
        case STATE_RECORDING:
            handle_recording();
            break;
        case STATE_TRANSMITTING:
            handle_transmitting();
            break;
        default:
            current_state = STATE_RECORDING; // STATE_IDLE - when button was used
            break;
        }
    }

    return 0;
}
