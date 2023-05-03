// Main class and config
#include <config.h>
#include <main.h>

// FreeRTOS includes and iostream
#include <iostream>

#include "freertos/freertos.h"
#include "freertos/task.h"

// GPIO
#include "driver/gpio.h"            // Module
#define INC_BUTTON_PIN GPIO_NUM_34  // Button pin for incrementing pulse length
#define DEC_BUTTON_PIN GPIO_NUM_33  // Button pin for decrementing pulse length
uint16_t pulse_length = 120;        // Initial pulse length

// Testing
#include <kinematics.h>

const float coxa_length = 60.5;
const float femur_length = 100.0;
const float tibia_length = 150.0;
ForwardKinematics fk = ForwardKinematics(coxa_length, femur_length, tibia_length);

// Servo tuning function
void servo_tune_setup() {
    // I2C bus and servo driver
    ESP_ERROR_CHECK(I2C_bus.begin(SDA_PIN, SCL_PIN));
    servo_driver.begin();
    servo_driver.set_PWM_freq(48.828125);

    // GPIO input
    gpio_set_direction(INC_BUTTON_PIN, GPIO_MODE_INPUT);
    gpio_set_direction(DEC_BUTTON_PIN, GPIO_MODE_INPUT);

    // set the servo to a specific value until button is pressed
    servo_driver.set_pulse_length(PCA9685::CHANNEL_00, pulse_length);
    while (gpio_get_level(INC_BUTTON_PIN) == 0) {
        vTaskDelay(pdMS_TO_TICKS(10));
    }

    vTaskDelay(pdMS_TO_TICKS(100));
}

void servo_tune_loop() {
    if (gpio_get_level(INC_BUTTON_PIN) == 1) {
        vTaskDelay(pdMS_TO_TICKS(10));  // Debounce
        pulse_length++;
        printf("pulse length = %d\n", pulse_length);
    }

    if (gpio_get_level(DEC_BUTTON_PIN) == 1) {
        vTaskDelay(pdMS_TO_TICKS(10));  // Debounce
        pulse_length--;
        printf("pulse length = %d\n", pulse_length);
    }

    servo_driver.set_pulse_length(PCA9685::CHANNEL_00, pulse_length);

    vTaskDelay(pdMS_TO_TICKS(100));
}

// Main functions
void Main::setup(void) {
    servo_tune_setup();

    Eigen::Vector3f configuration = Eigen::Vector3f(0, 30, 0);
    std::cout << fk.evaluate(configuration) << std::endl;
}

void Main::loop(void) {
    // servo_tune_loop();
    servo1.set_position(0);
    vTaskDelay(pdMS_TO_TICKS(1000));
    servo2.set_position(0);
    vTaskDelay(pdMS_TO_TICKS(1000));
    servo3.set_position(0);
    vTaskDelay(pdMS_TO_TICKS(1000));
}

extern "C" void app_main(void) {
    Main main = Main();
    main.setup();

    while (true) {
        main.loop();
        // servo3.test_sweep();
        // servo1.set_position(90);
    }
}
