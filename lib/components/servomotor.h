// Module to interface with physical servo motors through the driver module PCA9685.
// Implements basic movement functions.
#pragma once

// Includes
#include <PCA9685.h>  // Driver module

#include <algorithm>  // for min and max functions
#include <cmath>      // for round function

#include "freertos/freertos.h"  // general purpose module

// Main component class
class ServoMotor {
   public:
    // Constructor

    ServoMotor(
        std::vector<uint16_t> ref_points,
        int angle_interval,
        PCA9685 driver,
        PCA9685::Channel channel,
        float angle_shift = 0);

    // Position setters and getters

    void set_position(float position);
    float get_position(void) { return _current_position; };
    void rotate(float angle);
    uint16_t get_pulse_length(void) { return _current_pulse_length; };

    // Utility methods

    void test_sweep(void);

   protected:
    // Fundamental variables and constants

    const std::vector<uint16_t> _ref_points;  // Angle reference points for accurate position control
    const int _angle_interval;                // Angle swept in each reference interval
    const float _angle_shift;                 // Angle shift for the ranges

    PCA9685 _driver;                  // Driver device
    const PCA9685::Channel _channel;  // Driver channel

    float _current_position;         // Current position in angle units
    uint16_t _current_pulse_length;  // Current position in pulse length units

    // Auxiliary variables and constants

    const int _ref_points_num = _ref_points.size();  // Number of reference points

    const uint16_t MAX_PULSE_LENGTH = _ref_points[_ref_points_num - 1];                            // Maximum value for PWM pulse length
    const uint16_t MIN_PULSE_LENGTH = _ref_points[0];                                              // Minimum value for PWM pulse length
    const uint16_t MID_PULSE_LENGTH = (MAX_PULSE_LENGTH - MIN_PULSE_LENGTH) / 2 + _ref_points[0];  // Middle value for PWM pulse length

    const float MAX_ANGLE = _ref_points_num * _angle_interval + _angle_shift;  // Maximum angle value
    const float MIN_ANGLE = _angle_shift;                                      // Minimum angle value
    const float MID_ANGLE = (MAX_ANGLE - MIN_ANGLE) / 2 + _angle_shift;        // Middle angle value

    // Utility methods

    uint16_t angle_to_pulse_length(float angle);
};