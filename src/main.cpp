// FreeRTOS includes
#include "freertos/freertos.h"
#include "freertos/task.h"

// I2C driver
#include "I2C.h"
#define pdSECOND pdMS_TO_TICKS(1000)

// Error library
#include "esp_err.h"

// Device library and settings
#include "PCA9685.h"
#define PCA9685_ADDR 0x40
#define SDA_PIN GPIO_NUM_21
#define SCL_PIN GPIO_NUM_22

// Main function
extern "C" void app_main(void) {
    I2C_Bus_t& myI2C = I2C_B0;  // i2c0 and i2c1 are the default objects
    ESP_ERROR_CHECK(myI2C.begin(GPIO_NUM_21, GPIO_NUM_22));
    myI2C.scanner();

    PCA9685 device = PCA9685(PCA9685_ADDR, myI2C);
    device.begin();
    device.set_PWM_freq(50);

    while (true) {
        for (int i = 0; i < 15; i++) {
            uint16_t us = (i + 7) * 100;
            device.write_microseconds(PCA9685::CHANNEL_00, us);
            vTaskDelay(pdMS_TO_TICKS(1000));
            uint8_t ps = device.read_prescale();
            printf("Prescale value = %u \n", ps);
            float duty = device.get_duty_cycle(PCA9685::CHANNEL_00);
            printf("Duty cycle value = %.2f for channel 0\n", duty);
            device.write_microseconds(PCA9685::CHANNEL_00, 700);
            vTaskDelay(pdMS_TO_TICKS(500));
            device.set_duty_cycle(PCA9685::CHANNEL_00, duty);
            vTaskDelay(pdMS_TO_TICKS(500));
        }
    }

    myI2C.close();
    vTaskDelay(portMAX_DELAY);
}
