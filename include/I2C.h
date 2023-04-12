#pragma once

// Includes
#include "driver/i2c.h"
#include "esp_err.h"
#include "esp_log.h"

class I2C_Bus {
   public:
    // Constructors and destructros

    I2C_Bus(i2c_port_t port);
    ~I2C_Bus();

    // Methods
    esp_err_t begin(gpio_num_t sda_pin, gpio_num_t scl_pin, uint32_t clk_speed = DEFAULT_CLK_SPEED);
    esp_err_t begin(gpio_num_t sda_pin, gpio_num_t scl_pin, gpio_pullup_t sda_pullup_en, gpio_pullup_t scl_pullup_en, uint32_t clk_speed = DEFAULT_CLK_SPEED);

    esp_err_t close(void);

    void set_timeout(uint32_t ms);

    esp_err_t write_bit(uint8_t dev_addr, uint8_t reg_addr, uint8_t bit_num, uint8_t data, int32_t timeout = -1);
    esp_err_t write_bits(uint8_t dev_addr, uint8_t reg_addr, uint8_t bit_start, uint8_t length, uint8_t data, int32_t timeout = -1);
    esp_err_t write_byte(uint8_t dev_addr, uint8_t reg_addr, uint8_t data, int32_t timeout = -1);
    esp_err_t write_bytes(uint8_t dev_addr, uint8_t reg_addr, size_t length, const uint8_t *data, int32_t timeout = -1);

    esp_err_t read_bit(uint8_t dev_addr, uint8_t reg_addr, uint8_t bit_num, uint8_t *data, int32_t timeout = -1);
    esp_err_t read_bits(uint8_t dev_addr, uint8_t reg_addr, uint8_t bit_start, uint8_t length, uint8_t *data, int32_t timeout = -1);
    esp_err_t read_byte(uint8_t dev_addr, uint8_t reg_addr, uint8_t *data, int32_t timeout = -1);
    esp_err_t read_bytes(uint8_t dev_addr, uint8_t reg_addr, size_t length, uint8_t *data, int32_t timeout = -1);

    esp_err_t test_connection(uint8_t dev_addr, int32_t timeout = -1);
    void scanner(void);

   private:
    // Variables
    i2c_port_t _port;         // I2C port: I2C_NUM_0 or I2C_NUM_1
    uint32_t _ticks_to_wait;  // Timeout in ticks for read and write

    // Class variables
    static const uint32_t DEFAULT_CLK_SPEED = 400000;  // default bus clock speed, 400 kHz
    static const uint32_t DEFAULT_TIMEOUT = 1000;      // default timeout in milliseconds

    static const bool I2C_MASTER_ACK_EN = true;    // Enable ack check for master
    static const bool I2C_MASTER_ACK_DIS = false;  // Disable ack check for master
};

using I2C_Bus_t = I2C_Bus;

extern I2C_Bus_t I2C_B0;  // I2C port 0
extern I2C_Bus_t I2C_B1;  // I2C port 1

constexpr I2C_Bus_t &getI2C(i2c_port_t port) {
    return port == 0 ? I2C_B0 : I2C_B1;
}

class I2C_Device {
   public:
    // Constructor
    I2C_Device(const uint8_t addr, I2C_Bus_t &bus);

   protected:
    // Variables
    uint8_t _device_addr;
    I2C_Bus_t *_bus;

    // Methods
    uint8_t read8(uint8_t reg);
    void write8(uint8_t reg, uint8_t data);
};