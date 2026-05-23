#ifndef __ADS1220_REGISTERS__
#define __ADS1220_REGISTERS__

#include <stdint.h>

namespace ADS1220_REG {

// Register addresses
constexpr uint8_t REG_MASK = 0x03;
constexpr uint8_t CONFIG0 = 0x00;
constexpr uint8_t CONFIG1 = 0x01;
constexpr uint8_t CONFIG2 = 0x02;
constexpr uint8_t CONFIG3 = 0x03;

// Command opcodes
// Operands: rr = configuration register (00 to 11), nn = number of bytes – 1 (00 to 11), and x = don't care.
constexpr uint8_t CMD_RREG_BASE = 0x20;    // 0010 rrnn
constexpr uint8_t CMD_WREG_BASE = 0x40;    // 0100 rrnn

// Single-byte command opcodes (x = don't care -> choose x=0)
constexpr uint8_t CMD_POWERDOWN = 0x02;    // 0000 0010
constexpr uint8_t CMD_RESET = 0x06;        // 0000 0110
constexpr uint8_t CMD_START_SYNC = 0x08;   // 0000 1000
constexpr uint8_t CMD_RDATA = 0x10;        // 0001 0000

// CONFIG0 Register (0x00): MUX[3:0] (bits 7-4), GAIN[2:0] (bits 3-1), PGA_BYPASS (bit 0)
namespace Config0 {
    constexpr uint8_t MUX_SHIFT = 4;
    constexpr uint8_t MUX_MASK = 0xF0;      // bits 7-4
    constexpr uint8_t GAIN_SHIFT = 1;
    constexpr uint8_t GAIN_MASK = 0x0E;     // bits 3-1
    constexpr uint8_t PGA_BYPASS_BIT = 0;
    constexpr uint8_t PGA_BYPASS_MASK = 0x01;

    // MUX options
    constexpr uint8_t MUX_AIN0_AIN1 = 0b0000 << MUX_SHIFT;  // 0000
    constexpr uint8_t MUX_AIN0_AIN2 = 0b0001 << MUX_SHIFT;  // 0001
    constexpr uint8_t MUX_AIN0_AIN3 = 0b0010 << MUX_SHIFT;  // 0010
    constexpr uint8_t MUX_AIN1_AIN2 = 0b0011 << MUX_SHIFT;  // 0011
    constexpr uint8_t MUX_AIN1_AIN3 = 0b0100 << MUX_SHIFT;  // 0100
    constexpr uint8_t MUX_AIN2_AIN3 = 0b0101 << MUX_SHIFT;  // 0101
    constexpr uint8_t MUX_AIN1_AIN0 = 0b0110 << MUX_SHIFT;  // 0110
    constexpr uint8_t MUX_AIN3_AIN2 = 0b0111 << MUX_SHIFT;  // 0111
    constexpr uint8_t MUX_AIN0_AVSS = 0b1000 << MUX_SHIFT;  // 1000
    constexpr uint8_t MUX_AIN1_AVSS = 0b1001 << MUX_SHIFT;  // 1001
    constexpr uint8_t MUX_AIN2_AVSS = 0b1010 << MUX_SHIFT;  // 1010
    constexpr uint8_t MUX_AIN3_AVSS = 0b1011 << MUX_SHIFT;  // 1011
    constexpr uint8_t MUX_AIN_SPEC1 = 0b1100 << MUX_SHIFT;  // 1100
    constexpr uint8_t MUX_AIN_SPEC2 = 0b1101 << MUX_SHIFT;  // 1101
    constexpr uint8_t MUX_AIN_SPEC3 = 0b1110 << MUX_SHIFT;  // 1110

    // Gain settings (placed at bits 3-1)
    constexpr uint8_t GAIN_1 = 0b000 << GAIN_SHIFT;    // set gain to 1x
    constexpr uint8_t GAIN_2 = 0b001 << GAIN_SHIFT;    // set gain to 2x
    constexpr uint8_t GAIN_4 = 0b010 << GAIN_SHIFT;    // set gain to 4x
    constexpr uint8_t GAIN_8 = 0b011 << GAIN_SHIFT;    // set gain to 8x
    constexpr uint8_t GAIN_16 = 0b100 << GAIN_SHIFT;   // set gain to 16x
    constexpr uint8_t GAIN_32 = 0b101 << GAIN_SHIFT;   // set gain to 32x
    constexpr uint8_t GAIN_64 = 0b110 << GAIN_SHIFT;   // set gain to 64x
    constexpr uint8_t GAIN_128 = 0b111 << GAIN_SHIFT;  // set gain to 128x

    // PGA Bypass
    constexpr uint8_t PGA_BYPASS_ON = 1 << PGA_BYPASS_BIT;  // bypass PGA
    constexpr uint8_t PGA_BYPASS_OFF = 0;                   // PGA enabled

    // Function for setting configuration byte
    constexpr uint8_t pack(uint8_t mux_option, uint8_t gain_option, bool pga_bypass) {
        return (mux_option & MUX_MASK) | (gain_option & GAIN_MASK) | (pga_bypass ? PGA_BYPASS_ON : PGA_BYPASS_OFF);
    }
}

// CONFIG1 Register (0x01): DR[2:0] (bits 7-5), MODE[1:0] (bits 4-3), CM (bit 2), TS (bit 1), BCS (bit 0)
namespace Config1 {
    constexpr uint8_t DR_SHIFT = 5;
    constexpr uint8_t DR_MASK = 0xE0;       // bits 7-5
    constexpr uint8_t MODE_SHIFT = 3;
    constexpr uint8_t MODE_MASK = 0x18;     // bits 4-3
    constexpr uint8_t CM_BIT = 2;
    constexpr uint8_t TS_BIT = 1;
    constexpr uint8_t BCS_BIT = 0;

    // Data rates (normal mode) — placed at bits 7-5
    constexpr uint8_t DR_20SPS = 0b000 << DR_SHIFT;     // 000
    constexpr uint8_t DR_45SPS = 0b001 << DR_SHIFT;     // 001
    constexpr uint8_t DR_90SPS = 0b010 << DR_SHIFT;     // 010
    constexpr uint8_t DR_175SPS = 0b011 << DR_SHIFT;    // 011
    constexpr uint8_t DR_330SPS = 0b100 << DR_SHIFT;    // 100
    constexpr uint8_t DR_600SPS = 0b101 << DR_SHIFT;    // 101
    constexpr uint8_t DR_1000SPS = 0b110 << DR_SHIFT;   // 110

    // MODE values
    constexpr uint8_t MODE_NORMAL = 0b00 << MODE_SHIFT;
    constexpr uint8_t MODE_DUTYCYCLE = 0b01 << MODE_SHIFT;
    constexpr uint8_t MODE_TURBO = 0b10 << MODE_SHIFT;

    // Bits
    constexpr uint8_t CM_SINGLE = 0 << CM_BIT;
    constexpr uint8_t CM_CONTINUOUS = 1 << CM_BIT;
    constexpr uint8_t TS_OFF = 0 << TS_BIT;
    constexpr uint8_t TS_ON = 1 << TS_BIT;
    constexpr uint8_t BCS_OFF = 0 << BCS_BIT;
    constexpr uint8_t BCS_ON = 1 << BCS_BIT;

    // Function for setting configuration byte
    constexpr uint8_t pack(uint8_t dr, uint8_t mode, bool cm, bool ts, bool bcs) {
        return (dr & DR_MASK) | (mode & MODE_MASK) |
               (cm ? (1 << CM_BIT) : 0) |
               (ts ? (1 << TS_BIT) : 0) |
               (bcs ? (1 << BCS_BIT) : 0);
    }
}

// CONFIG2 Register (0x02): VREF[1:0] (bits 7-6), 50/60[1:0] (bits 5-4), PSW (bit 3), IDAC[2:0] (bits 2-0)
namespace Config2 {
    constexpr uint8_t VREF_SHIFT = 6;
    constexpr uint8_t VREF_MASK = 0xC0;     // bits 7-6
    constexpr uint8_t FIR_SHIFT = 4;
    constexpr uint8_t FIR_MASK = 0x30;      // bits 5-4
    constexpr uint8_t PSW_BIT = 3;
    constexpr uint8_t PSW_MASK = 1u << PSW_BIT;
    constexpr uint8_t IDAC_SHIFT = 0;
    constexpr uint8_t IDAC_MASK = 0x07;     // bits 2-0

    // VREF options
    constexpr uint8_t VREF_INTERNAL = 0b00 << VREF_SHIFT;
    constexpr uint8_t VREF_REFP0_REFN0 = 0b01 << VREF_SHIFT;
    constexpr uint8_t VREF_AIN0_AIN3 = 0b10 << VREF_SHIFT;
    constexpr uint8_t VREF_AVDD_AVSS = 0b11 << VREF_SHIFT;

    // FIR/50-60 options (use only with 20SPS normal / 5SPS duty-cycle)
    constexpr uint8_t FIR_NONE = 0b00 << FIR_SHIFT;
    constexpr uint8_t FIR_50_60 = 0b01 << FIR_SHIFT;
    constexpr uint8_t FIR_50ONLY = 0b10 << FIR_SHIFT;
    constexpr uint8_t FIR_60ONLY = 0b11 << FIR_SHIFT;

    // PSW options
    constexpr uint8_t PSW_OPEN = 0;
    constexpr uint8_t PSW_AUTO = PSW_MASK;

    // IDAC currents
    constexpr uint8_t IDAC_OFF = 0b000 << IDAC_SHIFT;
    constexpr uint8_t IDAC_10UA = 0b001 << IDAC_SHIFT;
    constexpr uint8_t IDAC_50UA = 0b010 << IDAC_SHIFT;
    constexpr uint8_t IDAC_100UA = 0b011 << IDAC_SHIFT;
    constexpr uint8_t IDAC_250UA = 0b100 << IDAC_SHIFT;
    constexpr uint8_t IDAC_500UA = 0b101 << IDAC_SHIFT;
    constexpr uint8_t IDAC_1000UA = 0b110 << IDAC_SHIFT;
    constexpr uint8_t IDAC_1500UA = 0b111 << IDAC_SHIFT;

    // Function for setting configuration byte
    constexpr uint8_t pack(uint8_t vref, uint8_t fir, bool psw, uint8_t idac) {
        return (vref & VREF_MASK) | (fir & FIR_MASK) |
               (psw ? PSW_MASK : 0) |
               (idac & IDAC_MASK);
    }
}

// CONFIG3 Register (0x03): I1MUX[2:0] (bits 7-5), I2MUX[2:0] (bits 4-2), DRDYM (bit 1), 0 (bit 0)
namespace Config3 {
    constexpr uint8_t I1_SHIFT = 5;
    constexpr uint8_t I1_MASK = 0xE0;       // bits 7-5
    constexpr uint8_t I2_SHIFT = 2;
    constexpr uint8_t I2_MASK = 0x1C;       // bits 4-2
    constexpr uint8_t DRDYM_BIT = 1;
    constexpr uint8_t DRDYM_MASK = 1u << DRDYM_BIT;

    // I1 routing (values shifted into place)
    constexpr uint8_t I1_DISABLED = 0b000 << I1_SHIFT;
    constexpr uint8_t I1_AIN0_REFP1 = 0b001 << I1_SHIFT;
    constexpr uint8_t I1_AIN1 = 0b010 << I1_SHIFT;
    constexpr uint8_t I1_AIN2 = 0b011 << I1_SHIFT;
    constexpr uint8_t I1_AIN3_REFN1 = 0b100 << I1_SHIFT;
    constexpr uint8_t I1_REFP0 = 0b101 << I1_SHIFT;
    constexpr uint8_t I1_REFN0 = 0b110 << I1_SHIFT;
    // 111 reserved

    // I2 routing (values shifted into place)
    constexpr uint8_t I2_DISABLED = 0b000 << I2_SHIFT;
    constexpr uint8_t I2_AIN0_REFP1 = 0b001 << I2_SHIFT;
    constexpr uint8_t I2_AIN1 = 0b010 << I2_SHIFT;
    constexpr uint8_t I2_AIN2 = 0b011 << I2_SHIFT;
    constexpr uint8_t I2_AIN3_REFN1 = 0b100 << I2_SHIFT;
    constexpr uint8_t I2_REFP0 = 0b101 << I2_SHIFT;
    constexpr uint8_t I2_REFN0 = 0b110 << I2_SHIFT;
    // 111 reserved

    // DRDYM options
    constexpr uint8_t DRDYM_OFF = 0;
    constexpr uint8_t DRDYM_ON = DRDYM_MASK;

    // Function for setting configuration byte
    constexpr uint8_t pack(uint8_t i1mux, uint8_t i2mux, bool drdym) {
        return (i1mux & I1_MASK) | (i2mux & I2_MASK) | (drdym ? DRDYM_MASK : 0);
    }
}

} // namespace ADS1220_REG

#endif
