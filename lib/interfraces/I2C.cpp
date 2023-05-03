#include <I2C.h>  // header file

// ------ I2C_Bus class

// --- Constructor and Destrutor

/*!
 * @brief  Constructor of the class.
 *
 * @param  port     The port number of the bus, wither I2C_NUM_0 or I2C_NUM_1;
 */
I2C_Bus::I2C_Bus(i2c_port_t port) :
    _port(port), _ticks_to_wait{pdMS_TO_TICKS(DEFAULT_TIMEOUT)} {}

/*!
 * @brief  Destructor of the class. Deletes the bus.
 */
I2C_Bus::~I2C_Bus() { close(); }

// --- Initialization and Closing methods

/*!
 * @brief  Configures the I2C bus and installs the driver.
 *
 * @param  sda_io_num   GPIO number for SDA line;
 * @param  scl_io_num   GPIO number for SCL line;
 * @param  clk_speed    I2C clock frequency for master mode, (not higher than 1MHz for now), default 400KHz;
 *
 * @return  The outcome of the operation, compatible with esp_err.h;
 */
esp_err_t I2C_Bus::begin(gpio_num_t sda_io_num, gpio_num_t scl_io_num, uint32_t clk_speed) {
    return begin(sda_io_num, scl_io_num, GPIO_PULLUP_ENABLE, GPIO_PULLUP_ENABLE, clk_speed);
}

/*!
 * @brief  Configures the I2C bus and installs the driver.
 *
 * @param  sda_io_num       GPIO number for SDA line;
 * @param  scl_io_num       GPIO number for SCL line;
 * @param  sda_pullup_en    Enable internal pullup on SDA line;
 * @param  scl_pullup_en    Enable internal pullup on SCL line;
 * @param  clk_speed        I2C clock frequency for master mode, (not higher than 1MHz for now), default 400KHz;
 *
 * @return  The outcome of the operation, compatible with esp_err.h;
 */
esp_err_t I2C_Bus::begin(gpio_num_t sda_io_num, gpio_num_t scl_io_num, gpio_pullup_t sda_pullup_en, gpio_pullup_t scl_pullup_en, uint32_t clk_speed) {
    // create config objct
    i2c_config_t conf;
    conf.mode = I2C_MODE_MASTER;
    conf.sda_io_num = sda_io_num;
    conf.sda_pullup_en = sda_pullup_en;
    conf.scl_io_num = scl_io_num;
    conf.scl_pullup_en = scl_pullup_en;
    conf.master.clk_speed = clk_speed;
    conf.clk_flags = 0;

    // install the config
    esp_err_t err = i2c_param_config(_port, &conf);
    if (!err) err = i2c_driver_install(_port, conf.mode, 0, 0, 0);

    return err;
}

/*!
 * @brief  Stop I2C bus and uninstall driver.
 */
esp_err_t I2C_Bus::close() {
    return i2c_driver_delete(_port);  // delete the bus
}

// --- Timeout setter

/*!
 * @brief  Timeout time setter (in milliseconds).
 */
void I2C_Bus::set_timeout(uint32_t ms) {
    _ticks_to_wait = pdMS_TO_TICKS(ms);  // change the variable
}

// --- Writing and Reading methods

/*!
 * @brief  I2C command for writing to 8-bit slave device register, multiple bytes version.
 *
 * @param  dev_addr     I2C slave device register;
 * @param  reg_addr     Register address to write to;
 * @param  data         Value(s) to be written to the register;
 * @param  length       Number of bytes to write (should be within the data buffer size);
 * @param  timeout      Custom timeout for the particular call;
 *
 * @return  The outcome of the operation, compatible with esp_err.h;
 */
esp_err_t I2C_Bus::write_bytes(uint8_t dev_addr, uint8_t reg_addr, const uint8_t *data, size_t length, int32_t timeout) {
    // begin command creation
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();

    // start condition
    i2c_master_start(cmd);

    // calls the slave device and select the register to write into
    i2c_master_write_byte(cmd, (dev_addr << 1) | I2C_MASTER_WRITE, I2C_MASTER_ACK_EN);
    i2c_master_write_byte(cmd, reg_addr, I2C_MASTER_ACK_EN);

    // write data
    i2c_master_write(cmd, (uint8_t *)data, length, I2C_MASTER_ACK_EN);

    // stop condition
    i2c_master_stop(cmd);

    // send command to the bus
    esp_err_t err = i2c_master_cmd_begin(_port, cmd, (timeout < 0 ? _ticks_to_wait : pdMS_TO_TICKS(timeout)));

    // delete the command
    i2c_cmd_link_delete(cmd);

    return err;
}

/*!
 * @brief  I2C command for reading a 8-bit slave device register, multiple bytes version.
 *
 * @param  dev_addr     I2C slave device register;
 * @param  reg_addr     Register address to read from;
 * @param  data         Buffer to store the read value(s);
 * @param  length       Number of bytes to read (should be within the data buffer size);
 * @param  timeout      Custom timeout for the particular call;
 *
 * @return  The outcome of the operation, compatible with esp_err.h;
 */
esp_err_t I2C_Bus::read_bytes(uint8_t dev_addr, uint8_t reg_addr, uint8_t *data, size_t length, int32_t timeout) {
    // begin command creation
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();

    // start condition
    i2c_master_start(cmd);

    // instructs the slave with the register to read
    i2c_master_write_byte(cmd, (dev_addr << 1) | I2C_MASTER_WRITE, I2C_MASTER_ACK_EN);
    i2c_master_write_byte(cmd, reg_addr, I2C_MASTER_ACK_EN);

    // repeat start condition
    i2c_master_start(cmd);

    // proceed with the read instruction
    i2c_master_write_byte(cmd, (dev_addr << 1) | I2C_MASTER_READ, I2C_MASTER_ACK_EN);

    // read data
    i2c_master_read(cmd, data, length, I2C_MASTER_LAST_NACK);

    // stop condition
    i2c_master_stop(cmd);

    // send command to the bus
    esp_err_t err = i2c_master_cmd_begin(_port, cmd, (timeout < 0 ? _ticks_to_wait : pdMS_TO_TICKS(timeout)));

    // delete the command
    i2c_cmd_link_delete(cmd);

    return err;
}

// --- Test and Debugging methods

/*!
 * @brief  Quick check method to see if a slave device responds.

 * @param  dev_addr     I2C slave device register
 * @param  timeout      Custom timeout for the particular call
 *
 * @return  The outcome of the operation, compatible with esp_err.h;
 */
esp_err_t I2C_Bus::test_connection(uint8_t dev_addr, int32_t timeout) {
    // begin command creation
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();

    // start condition
    i2c_master_start(cmd);

    // see if the device is connected and returns an ACK
    i2c_master_write_byte(cmd, (dev_addr << 1) | I2C_MASTER_WRITE, I2C_MASTER_ACK_EN);

    // stop condition
    i2c_master_stop(cmd);

    // send command to the bus
    esp_err_t err = i2c_master_cmd_begin(_port, cmd, (timeout < 0 ? _ticks_to_wait : pdMS_TO_TICKS(timeout)));

    // delete the command
    i2c_cmd_link_delete(cmd);

    return err;
}

/*!
 * @brief  I2C scanner utility method, prints out all device addresses found on this I2C bus.
 */
void I2C_Bus::scanner() {
    constexpr int32_t scan_timeout = 20;
    printf(LOG_COLOR_W "\n>> I2C scanning ..." LOG_RESET_COLOR "\n");

    // scan for devices and counts them
    uint8_t count = 0;
    for (size_t i = 0x3; i < 0x78; i++) {
        if (test_connection(i, scan_timeout) == ESP_OK) {
            // if a device is found print its address and increment counter
            printf(LOG_COLOR_W "- Device found at address 0x%X%s", i, LOG_RESET_COLOR "\n");
            count++;
        }
    }

    // if no devices found
    if (count == 0)
        printf(LOG_COLOR_E "- No I2C devices found!" LOG_RESET_COLOR "\n");
    printf("\n");
}

// Available I2C buses

I2C_Bus_t I2C_B0 = I2C_Bus(I2C_NUM_0);  // I2C port 0
I2C_Bus_t I2C_B1 = I2C_Bus(I2C_NUM_1);  // I2C port 1

// ------ I2C_Device class

// --- Constructor
/*!
 *  @brief  Instantiates a new I2C device with the I2C address and I2C bus.
 *
 *  @param  addr    The 7-bit I2C address to locate this device;
 *  @param  bus     A reference to a 'I2C_Bus' object used to communicate with the device;
 */
I2C_Device::I2C_Device(const uint8_t addr, I2C_Bus_t &bus) :
    _device_addr(addr), _bus(&bus) {}

// --- Communication methods

/*!
 *  @brief  Reads from the I2C_Device.
 *
 *  @param  reg     The address of the register to read;
 *  @param  data    The variable to store the read data of the register;
 *  @param  length  The number of data bytes to read;
 */
void I2C_Device::read(uint8_t reg, uint8_t *data, size_t length) {
    _bus->read_bytes(_device_addr, reg, data, length);
}

/*!
 *  @brief  Simpler version for single byte of read().
 *
 *  @param  reg     The address of the register to read;
 *
 *  @return  The value of the register;
 */
uint8_t I2C_Device::read(uint8_t reg) {
    uint8_t data;
    read(reg, &data);
    return data;
}

/*!
 *  @brief  Writes to the I2C_Device.
 *
 *  @param  reg     The address of the register to write on to;
 *  @param  data    The data to write in that register;
 *  @param  length  The number of data bytes to write;
 */
void I2C_Device::write(uint8_t reg, uint8_t *data, size_t length) {
    _bus->write_bytes(_device_addr, reg, data, length);
}

/*!
 *  @brief  Simpler version for single byte of write()
 *
 *  @param  reg     The address of the register to write on to;
 *  @param  data    The data to write in that register;
 */
void I2C_Device::write(uint8_t reg, uint8_t data) {
    write(reg, &data);
}
