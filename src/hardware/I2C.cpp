#include "I2C.h"

// I2C_Bus class
// Constructor and Destructor

I2C_Bus::I2C_Bus(i2c_port_t port) : _port(port), _ticks_to_wait{pdMS_TO_TICKS(DEFAULT_TIMEOUT)} {}

I2C_Bus::~I2C_Bus() { close(); }

// Methods

/*!
 * @brief  Config I2C bus and Install Driver
 *
 * @param  sda_io_num    GPIO number for SDA line
 * @param  scl_io_num    GPIO number for SCL line
 * @param  sda_pullup_en Enable internal pullup on SDA line
 * @param  scl_pullup_en Enable internal pullup on SCL line
 * @param  clk_speed     I2C clock frequency for master mode, (no higher than 1MHz for now), Default 400KHz
 *                       @see "driver/i2c.h"
 * @return               - ESP_OK   Success
 *                       - ESP_ERR_INVALID_ARG Parameter error
 *                       - ESP_FAIL Driver install error
 */
esp_err_t I2C_Bus::begin(gpio_num_t sda_io_num, gpio_num_t scl_io_num, uint32_t clk_speed) {
    return begin(sda_io_num, scl_io_num, GPIO_PULLUP_ENABLE, GPIO_PULLUP_ENABLE, clk_speed);
}

esp_err_t I2C_Bus::begin(gpio_num_t sda_io_num, gpio_num_t scl_io_num, gpio_pullup_t sda_pullup_en, gpio_pullup_t scl_pullup_en, uint32_t clk_speed) {
    i2c_config_t conf;  // create config objct
    conf.mode = I2C_MODE_MASTER;
    conf.sda_io_num = sda_io_num;
    conf.sda_pullup_en = sda_pullup_en;
    conf.scl_io_num = scl_io_num;
    conf.scl_pullup_en = scl_pullup_en;
    conf.master.clk_speed = clk_speed;
    conf.clk_flags = 0;

    esp_err_t err = i2c_param_config(_port, &conf);  // install the config
    if (!err) err = i2c_driver_install(_port, conf.mode, 0, 0, 0);

    return err;
}

/*!
 * Stop I2C bus and unninstall driver
 */
esp_err_t I2C_Bus::close() {
    return i2c_driver_delete(_port);  // delete the bus
}

/*!
 * Timeout read and write in milliseconds
 */
void I2C_Bus::set_timeout(uint32_t ms) {
    _ticks_to_wait = pdMS_TO_TICKS(ms);
}

// Writing

/*!
 * @brief  I2C commands for writing to a 8-bit slave device register.
 *         All of them returns standard esp_err_t codes, so it can be used
 *         with ESP_ERROR_CHECK();
 *
 * @param  dev_addr   I2C slave device register
 * @param  reg_addr   Register address to write to
 * @param  bit_num    Bit position number to write to (bit 7~0)
 * @param  bit_start  Start bit number when writing a bit-sequence (MSB)
 * @param  data       Value(s) to be write to the register
 * @param  length     Number of bytes to write (should be within the data buffer size)
 *                    writeBits() -> Number of bits after bitStart (including)
 * @param  timeout    Custom timeout for the particular call
 *
 * @return  - ESP_OK Success
 *          - ESP_ERR_INVALID_ARG Parameter error
 *          - ESP_FAIL Sending command error, slave doesn't ACK the transfer.
 *          - ESP_ERR_INVALID_STATE I2C driver not installed or not in master mode.
 *          - ESP_ERR_TIMEOUT Operation timeout because the bus is busy.
 */
esp_err_t I2C_Bus::write_bit(uint8_t dev_addr, uint8_t reg_addr, uint8_t bit_num, uint8_t data, int32_t timeout) {
    uint8_t buffer;
    esp_err_t err = read_byte(dev_addr, reg_addr, &buffer, timeout);

    if (err) return err;

    buffer = data ? (buffer | (1 << bit_num)) : (buffer & ~(1 << bit_num));

    return write_byte(dev_addr, reg_addr, buffer, timeout);
}

esp_err_t I2C_Bus::write_bits(uint8_t dev_addr, uint8_t reg_addr, uint8_t bit_start, uint8_t length, uint8_t data, int32_t timeout) {
    uint8_t buffer;
    esp_err_t err = read_byte(dev_addr, reg_addr, &buffer, timeout);

    if (err) return err;

    uint8_t mask = ((1 << length) - 1) << (bit_start - length + 1);
    data <<= (bit_start - length + 1);
    data &= mask;
    buffer &= ~mask;
    buffer |= data;

    return write_byte(dev_addr, reg_addr, buffer, timeout);
}

esp_err_t I2C_Bus::write_byte(uint8_t dev_addr, uint8_t reg_addr, uint8_t data, int32_t timeout) {
    return write_bytes(dev_addr, reg_addr, 1, &data, timeout);
}

esp_err_t I2C_Bus::write_bytes(uint8_t dev_addr, uint8_t reg_addr, size_t length, const uint8_t *data, int32_t timeout) {
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();

    i2c_master_start(cmd);

    i2c_master_write_byte(cmd, (dev_addr << 1) | I2C_MASTER_WRITE, I2C_MASTER_ACK_EN);
    i2c_master_write_byte(cmd, reg_addr, I2C_MASTER_ACK_EN);
    i2c_master_write(cmd, (uint8_t *)data, length, I2C_MASTER_ACK_EN);

    i2c_master_stop(cmd);

    esp_err_t err = i2c_master_cmd_begin(_port, cmd, (timeout < 0 ? _ticks_to_wait : pdMS_TO_TICKS(timeout)));

    i2c_cmd_link_delete(cmd);

    return err;
}

// Reading

/*!
 * @brief  I2C commands for reading a 8-bit slave device register.
 *         All of them returns standard esp_err_t codes, so it can be used
 *         with ESP_ERROR_CHECK();
 *
 * @param  dev_addr   I2C slave device register
 * @param  reg_addr   Register address to read from
 * @param  bit_num    Bit position number to write to (bit 7~0)
 * @param  bit_start  Start bit number when writing a bit-sequence (MSB)
 * @param  data       Buffer to store the read value(s)
 * @param  length     Number of bytes to read (should be within the data buffer size)
 * @param  timeout    Custom timeout for the particular call
 *
 * @return  - ESP_OK Success
 *          - ESP_ERR_INVALID_ARG Parameter error
 *          - ESP_FAIL Sending command error, slave doesn't ACK the transfer.
 *          - ESP_ERR_INVALID_STATE I2C driver not installed or not in master mode.
 *          - ESP_ERR_TIMEOUT Operation timeout because the bus is busy.
 */
esp_err_t I2C_Bus::read_bit(uint8_t dev_addr, uint8_t reg_addr, uint8_t bit_num, uint8_t *data, int32_t timeout) {
    return read_bits(dev_addr, reg_addr, bit_num, 1, data, timeout);
}

esp_err_t I2C_Bus::read_bits(uint8_t dev_addr, uint8_t reg_addr, uint8_t bit_start, uint8_t length, uint8_t *data, int32_t timeout) {
    uint8_t buffer;
    esp_err_t err = read_byte(dev_addr, reg_addr, &buffer, timeout);

    if (!err) {
        uint8_t mask = ((1 << length) - 1) << (bit_start - length + 1);
        buffer &= mask;
        buffer >>= (bit_start - length + 1);
        *data = buffer;
    }

    return err;
}

esp_err_t I2C_Bus::read_byte(uint8_t dev_addr, uint8_t reg_addr, uint8_t *data, int32_t timeout) {
    return read_bytes(dev_addr, reg_addr, 1, data, timeout);
}

esp_err_t I2C_Bus::read_bytes(uint8_t dev_addr, uint8_t reg_addr, size_t length, uint8_t *data, int32_t timeout) {
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();

    i2c_master_start(cmd);

    i2c_master_write_byte(cmd, (dev_addr << 1) | I2C_MASTER_WRITE, I2C_MASTER_ACK_EN);
    i2c_master_write_byte(cmd, reg_addr, I2C_MASTER_ACK_EN);

    i2c_master_start(cmd);

    i2c_master_write_byte(cmd, (dev_addr << 1) | I2C_MASTER_READ, I2C_MASTER_ACK_EN);

    i2c_master_read(cmd, data, length, I2C_MASTER_LAST_NACK);

    i2c_master_stop(cmd);

    esp_err_t err = i2c_master_cmd_begin(_port, cmd, (timeout < 0 ? _ticks_to_wait : pdMS_TO_TICKS(timeout)));

    i2c_cmd_link_delete(cmd);

    return err;
}

// Utility

/*!
 * @brief  Quick check to see if a slave device responds.

 * @param  dev_addr   I2C slave device register
 * @param  timeout    Custom timeout for the particular call
 *
 * @return  - ESP_OK Success
 *          - ESP_ERR_INVALID_ARG Parameter error
 *          - ESP_FAIL Sending command error, slave doesn't ACK the transfer.
 *          - ESP_ERR_INVALID_STATE I2C driver not installed or not in master mode.
 *          - ESP_ERR_TIMEOUT Operation timeout because the bus is busy.
 */
esp_err_t I2C_Bus::test_connection(uint8_t dev_addr, int32_t timeout) {
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();

    i2c_master_start(cmd);

    i2c_master_write_byte(cmd, (dev_addr << 1) | I2C_MASTER_WRITE, I2C_MASTER_ACK_EN);

    i2c_master_stop(cmd);

    esp_err_t err = i2c_master_cmd_begin(_port, cmd, (timeout < 0 ? _ticks_to_wait : pdMS_TO_TICKS(timeout)));

    i2c_cmd_link_delete(cmd);

    return err;
}

/*!
 * I2C scanner utility, prints out all device addresses found on this I2C bus.
 */
void I2C_Bus::scanner() {
    constexpr int32_t scan_timeout = 20;
    printf(LOG_COLOR_W "\n>> I2C scanning ..." LOG_RESET_COLOR "\n");
    uint8_t count = 0;
    for (size_t i = 0x3; i < 0x78; i++) {
        if (test_connection(i, scan_timeout) == ESP_OK) {
            printf(LOG_COLOR_W "- Device found at address 0x%X%s", i, LOG_RESET_COLOR "\n");
            count++;
        }
    }
    if (count == 0)
        printf(LOG_COLOR_E "- No I2C devices found!" LOG_RESET_COLOR "\n");
    printf("\n");
}

// Available buses

I2C_Bus_t I2C_B0 = I2C_Bus(I2C_NUM_0);
I2C_Bus_t I2C_B1 = I2C_Bus(I2C_NUM_1);

// I2C_Device class
/*!
 *  @brief  Instantiates a new I2C device with the I2C address and I2C bus
 *
 *  @param  addr The 7-bit I2C address to locate this device
 *  @param  bus  A reference to a 'I2C_Bus' object used to communicate with the device
 */
I2C_Device::I2C_Device(const uint8_t addr, I2C_Bus_t &bus) : _device_addr(addr), _bus(&bus) {}

// Low level I2C interface

/*!
 *  @brief  Reads a byte from the I2C_Device
 *
 *  @param  reg The address of the register to read
 */
uint8_t I2C_Device::read8(uint8_t reg) {
    uint8_t read_data;
    _bus->read_byte(_device_addr, reg, &read_data);
    return read_data;
}

/*!
 *  @brief  Writes a byte to the I2C_Device
 *
 *  @param  reg The address of the register to read
 *  @param  data The data to write in that register
 */
void I2C_Device::write8(uint8_t reg, uint8_t data) {
    _bus->write_byte(_device_addr, reg, data);
}
