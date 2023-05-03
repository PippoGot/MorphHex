// Interface module for the PCA9685 PWM driver module. Implements functions
// to communicate with the device.
#pragma once

// Includes
#include <I2C.h>  // custom I2C module

#include <algorithm>  // module for the min() function

#include "esp_log.h"            // logging module
#include "freertos/freertos.h"  // general purpose module
#include "freertos/task.h"      // delay module

// Main device class
class PCA9685 : public I2C_Device {
   public:
    // Constructor (inherited from super-class I2C_Device)

    using I2C_Device::I2C_Device;

    // Enums for channel number and output mode

    enum Channel : uint8_t {  // Available channels definition
        CHANNEL_00 = 0x0,
        CHANNEL_01 = 0x1,
        CHANNEL_02 = 0x2,
        CHANNEL_03 = 0x3,
        CHANNEL_04 = 0x4,
        CHANNEL_05 = 0x5,
        CHANNEL_06 = 0x6,
        CHANNEL_07 = 0x7,
        CHANNEL_08 = 0x8,
        CHANNEL_09 = 0x9,
        CHANNEL_10 = 0xA,
        CHANNEL_11 = 0xB,
        CHANNEL_12 = 0xC,
        CHANNEL_13 = 0xD,
        CHANNEL_14 = 0xE,
        CHANNEL_15 = 0xF,
    };

    enum OutputMode : bool {  // Available output modes
        TOTEM_POLE = true,
        OPEN_DRAIN = false,
    };

    // Initialization method

    void begin(float freq = 1000);

    // Mode methods

    void restart(void);
    void sleep(void);
    void wakeup(void);

    // Configuration setters

    void set_ext_clk(uint8_t prescale);
    void set_output_mode(OutputMode totempole);

    // PWM/Duty-Cycle setters and getters

    void set_PWM_freq(float freq);
    void set_PWM_toggles(Channel channel, uint16_t on, uint16_t off);

    void set_pulse_length(Channel channel, uint16_t length);
    uint16_t get_pulse_length(Channel channel);

    void set_duty_cycle(Channel channel, float duty_cycle);
    float get_duty_cycle(Channel channel);

    void write_microseconds(Channel channel, uint16_t microseconds);

    // Oscillator frequency and Prescaler setters and getters

    void set_osc_freq(uint32_t freq);
    uint32_t get_osc_freq(void);
    uint8_t read_prescale(void);

   protected:
    uint32_t _osc_freq;  // oscillator frequency in Hz

    // Enums for register addresses and bits

    enum RegisterAddress : uint8_t {  // Register definitions (table 4 in datasheet)

        MODE1 = 0x00,  // Mode Register 1
        MODE2 = 0x01,  // Mode Register 2

        SUBADR1 = 0x02,     // I2C-bus subaddress 1
        SUBADR2 = 0x03,     // I2C-bus subaddress 2
        SUBADR3 = 0x04,     // I2C-bus subaddress 3
        ALLCALLADR = 0x05,  // LED All Call I2C-bus address

        LED0_ON_L = 0x06,   // LED0 on tick, low byte
        LED0_ON_H = 0x07,   // LED0 on tick, high byte
        LED0_OFF_L = 0x08,  // LED0 off tick, low byte
        LED0_OFF_H = 0x09,  // LED0 off tick, high byte

        ALLLED_ON_L = 0xFA,   // Load all the LEDn_ON registers, low
        ALLLED_ON_H = 0xFB,   // Load all the LEDn_ON registers, high
        ALLLED_OFF_L = 0xFC,  // Load all the LEDn_OFF registers, low
        ALLLED_OFF_H = 0xFD,  // Load all the LEDn_OFF registers,high

        PRE_SCALE = 0xFE,  // Prescaler for PWM output frequency
        TESTMODE = 0xFF,   // Defines the test mode to be entered
    };

    enum ModeBit : uint8_t {  // Bits to set for the various modes of operation
        // MODE1 register bits description (table 5 in datasheet)

        MODE1_ALLCAL = 0x01,   // Respond to LED All Call I2C-bus address
        MODE1_SUB3 = 0x02,     // Respond to I2C-bus subaddress 3
        MODE1_SUB2 = 0x04,     // Respond to I2C-bus subaddress 2
        MODE1_SUB1 = 0x08,     // Respond to I2C-bus subaddress 1
        MODE1_SLEEP = 0x10,    // Low power mode, oscillator off
        MODE1_AI = 0x20,       // Auto-Increment enabled
        MODE1_EXTCLK = 0x40,   // Use EXTCLK pin clock
        MODE1_RESTART = 0x80,  // Restart enabled

        MODE1_EXTCLK_SLEEP = MODE1_EXTCLK + MODE1_SLEEP,  // Both EXTCLK and SLEEP bits

        // MODE2 register bits description (table 6 in datasheet)

        MODE2_OUTNE_0 = 0x01,  // Active LOW output enable input
        MODE2_OUTNE_1 = 0x02,  // Active LOW output enable input - high impedience
        MODE2_OUTDRV = 0x04,   // Totem pole (1) or open-drain (0) structure
        MODE2_OCH = 0x08,      // Outputs change on ACK (0) or STOP (1) command
        MODE2_INVRT = 0x10,    // Output logic state inverted
    };

    // Default values

    static const int FREQUENCY_OSCILLATOR = 25000000;  // Internal oscillator frequency (from datasheet)

    static const int PRE_SCALE_MIN = 3;    // Minimum prescale value
    static const int PRE_SCALE_MAX = 255;  // Maximum prescale value

    // Utility methods

    void turn_on(RegisterAddress mode_register, ModeBit mode_mask);
    void turn_off(RegisterAddress mode_register, ModeBit mode_mask);

    uint8_t compute_prescale(float freq);
    float compute_PWM_microseconds(uint8_t prescale);
    uint16_t compute_pulse_length(float duty_cycle);
    float compute_duty_cycle(uint16_t pulse_length);
};
