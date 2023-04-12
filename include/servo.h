#pragma once

#include "I2C.h"
#include "freertos/freertos.h"

class Servo {
   public:
    // Methods
    void rotate(float angle);
    float get_position();
    void set_position(float position);
    uint16_t get_length();

    void servo_test(void);

   protected:
    // Variables
    float _current_position;
    uint16_t _current_length;

    I2C_Device _driver;
    uint8_t _channel;

    const int MAX_LENGTH;
    const int MIN_LENGTH;

    const float MAX_DUTY;
    const float MIN_DUTY;

    const float MAX_ANGLE;
    const float MIN_ANGLE;

    // Methods
    uint16_t angle_to_length(float angle);
};