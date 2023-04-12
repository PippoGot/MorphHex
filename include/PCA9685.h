#pragma once

// Includes
#include <algorithm>

#include "I2C.h"
#include "freertos/freertos.h"
#include "freertos/task.h"

// Debug logging
#include "esp_log.h"

/*!
 *  @brief  Class to store state and functions for interacting with PCA9685 chip
 */
class PCA9685 : public I2C_Device {
   public:
    // Constructors
    using I2C_Device::I2C_Device;

    // Methods
    void begin(uint8_t prescale = 0);
    void reset(void);
    void sleep(void);
    void wakeup(void);

    void set_ext_clk(uint8_t prescale);
    void set_PWM_freq(float freq);
    void set_output_mode(bool totempole);
    void set_channel(uint8_t channel, uint16_t value, bool invert = false);

    // uint8_t get_PWM(uint8_t channel);
    void set_PWM(uint8_t channel, uint16_t on, uint16_t off);

    uint8_t read_prescale(void);
    void write_microseconds(uint8_t channel, uint16_t microseconds);

    void set_osc_freq(uint32_t freq);
    uint32_t get_osc_freq(void);

    // Channel definitions
    static const uint8_t CHANNEL_0 = 0;
    static const uint8_t CHANNEL_1 = 1;
    static const uint8_t CHANNEL_2 = 2;
    static const uint8_t CHANNEL_3 = 3;
    static const uint8_t CHANNEL_4 = 4;
    static const uint8_t CHANNEL_5 = 5;
    static const uint8_t CHANNEL_6 = 6;
    static const uint8_t CHANNEL_7 = 7;
    static const uint8_t CHANNEL_8 = 8;
    static const uint8_t CHANNEL_9 = 9;
    static const uint8_t CHANNEL_10 = 10;
    static const uint8_t CHANNEL_11 = 11;
    static const uint8_t CHANNEL_12 = 12;
    static const uint8_t CHANNEL_13 = 13;
    static const uint8_t CHANNEL_14 = 14;
    static const uint8_t CHANNEL_15 = 15;

   protected:
    // Variables
    uint32_t _osc_freq;

    // Register definitions and misc
    static const uint8_t MODE1 = 0x00;  // Mode Register 1
    static const uint8_t MODE2 = 0x01;  // Mode Register 2

    static const uint8_t SUBADR1 = 0x02;  // I2C-bus subaddress 1
    static const uint8_t SUBADR2 = 0x03;  // I2C-bus subaddress 2
    static const uint8_t SUBADR3 = 0x04;  // I2C-bus subaddress 3

    static const uint8_t ALLCALLADR = 0x05;  // LED All Call I2C-bus address

    static const uint8_t LED0_ON_L = 0x06;   // LED0 on tick, low byte
    static const uint8_t LED0_ON_H = 0x07;   // LED0 on tick, high byte
    static const uint8_t LED0_OFF_L = 0x08;  // LED0 off tick, low byte
    static const uint8_t LED0_OFF_H = 0x09;  // LED0 off tick, high byte

    static const uint8_t ALLLED_ON_L = 0xFA;   // load all the LEDn_ON registers, low
    static const uint8_t ALLLED_ON_H = 0xFB;   // load all the LEDn_ON registers, high
    static const uint8_t ALLLED_OFF_L = 0xFC;  // load all the LEDn_OFF registers, low
    static const uint8_t ALLLED_OFF_H = 0xFD;  // load all the LEDn_OFF registers,high

    static const uint8_t PRESCALE = 0xFE;  // Prescaler for PWM output frequency
    static const uint8_t TESTMODE = 0xFF;  // defines the test mode to be entered

    static const uint8_t MODE1_ALLCAL = 0x01;   // respond to LED All Call I2C-bus address
    static const uint8_t MODE1_SUB3 = 0x02;     // respond to I2C-bus subaddress 3
    static const uint8_t MODE1_SUB2 = 0x04;     // respond to I2C-bus subaddress 2
    static const uint8_t MODE1_SUB1 = 0x08;     // respond to I2C-bus subaddress 1
    static const uint8_t MODE1_SLEEP = 0x10;    // Low power mode. Oscillator off
    static const uint8_t MODE1_AI = 0x20;       // Auto-Increment enabled
    static const uint8_t MODE1_EXTCLK = 0x40;   // Use EXTCLK pin clock
    static const uint8_t MODE1_RESTART = 0x80;  // Restart enabled

    static const uint8_t MODE2_OUTNE_0 = 0x01;  // Active LOW output enable input
    static const uint8_t MODE2_OUTNE_1 = 0x02;  // Active LOW output enable input - high impedience
    static const uint8_t MODE2_OUTDRV = 0x04;   // totem pole structure vs open-drain
    static const uint8_t MODE2_OCH = 0x08;      // Outputs change on ACK vs STOP
    static const uint8_t MODE2_INVRT = 0x10;    // Output logic state inverted

    static const int FREQUENCY_OSCILLATOR = 25000000;  // Int. osc. frequency in datasheet

    static const int PRESCALE_MIN = 3;    // minimum prescale value
    static const int PRESCALE_MAX = 255;  // maximum prescale value
};
