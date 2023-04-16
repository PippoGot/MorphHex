// Module for I2C communication, implements:
// - Bus class for communication;
// - Device class to inherit from when implementing a device interface library;

#pragma once

#include "driver/i2c.h"  // base I2C library
#include "esp_err.h"     // error library
#include "esp_log.h"     // logging library

// Bus class: contains the functions to communicate over an I2C bus.
class I2C_Bus {
   public:
    // Constructor and Destructor

    I2C_Bus(i2c_port_t port);
    ~I2C_Bus();

    // Initialization and Closing methods

    esp_err_t begin(gpio_num_t sda_pin, gpio_num_t scl_pin, uint32_t clk_speed = DEFAULT_CLK_SPEED);
    esp_err_t begin(gpio_num_t sda_pin, gpio_num_t scl_pin, gpio_pullup_t sda_pullup_en, gpio_pullup_t scl_pullup_en, uint32_t clk_speed = DEFAULT_CLK_SPEED);
    esp_err_t close(void);

    // Timeout setter

    void set_timeout(uint32_t ms);

    // Writing and Reading methods

    esp_err_t write_bytes(uint8_t dev_addr, uint8_t reg_addr, const uint8_t *data, size_t length = 1, int32_t timeout = -1);
    esp_err_t read_bytes(uint8_t dev_addr, uint8_t reg_addr, uint8_t *data, size_t length = 1, int32_t timeout = -1);

    // Test and Debugging methods

    esp_err_t test_connection(uint8_t dev_addr, int32_t timeout = -1);
    void scanner(void);

   private:
    i2c_port_t _port;         // I2C port: either I2C_NUM_0 or I2C_NUM_1
    uint32_t _ticks_to_wait;  // Timeout in ticks for read and write

    static const uint32_t DEFAULT_CLK_SPEED = 400000;  // Default bus clock speed, 400 kHz
    static const uint32_t DEFAULT_TIMEOUT = 1000;      // Default timeout in milliseconds

    static const bool I2C_MASTER_ACK_EN = true;    // Enable ack check for master
    static const bool I2C_MASTER_ACK_DIS = false;  // Disable ack check for master
};

// I2C_Bus_t data type
using I2C_Bus_t = I2C_Bus;

// Available I2C buses
extern I2C_Bus_t I2C_B0;  // I2C port 0
extern I2C_Bus_t I2C_B1;  // I2C port 1

constexpr I2C_Bus_t &getI2C(i2c_port_t port) {
    return port == 0 ? I2C_B0 : I2C_B1;
}

// Device class: contains basic functions to read and write to a device.
class I2C_Device {
   public:
    // Constructor

    I2C_Device(const uint8_t addr, I2C_Bus_t &bus);

   protected:
    uint8_t _device_addr;  // I2C device address
    I2C_Bus_t *_bus;       // I2C bus to which the device is connected

    // Communication methods

    void read(uint8_t reg, uint8_t *data, size_t length = 1);
    uint8_t read(uint8_t reg);

    void write(uint8_t reg, uint8_t *data, size_t length = 1);
    void write(uint8_t reg, uint8_t data);
};