#include <servomotor.h>  // header file

// --- Constructor

/*!
 *  @brief  Constructor of the ServoMotor class.
 *
 *  @param  ref_points      Vector of PWM pulse lengths corresponding to a specific angle;
 *  @param  angle_interval  Angle swept for each interval;
 *  @param  drivers         Driver device used to control this servo instance;
 *  @param  channel         Channel of the driver used to control this servo instance;
 *  @param  angle_shift     Angle shift for the servo centering (optional);
 */
ServoMotor::ServoMotor(
    std::vector<uint16_t> ref_points,
    int angle_interval,
    PCA9685 driver,
    PCA9685::Channel channel,
    float angle_shift) :
    _ref_points{ref_points},
    _angle_interval{angle_interval},
    _angle_shift{angle_shift},
    _driver{driver},
    _channel{channel} {}

// --- Utility methods

/*!
 *  @brief  Converts an angle to its corresponding PWM pulse length value.
 *
 *  @param  angle   Angle to convert in pulse length;
 *
 *  @return  PWM pulse length;
 */
uint16_t ServoMotor::angle_to_pulse_length(float angle) {
    if (angle > MAX_ANGLE) return MAX_PULSE_LENGTH;  // And possibly send a warning message
    if (angle < MIN_ANGLE) return MIN_PULSE_LENGTH;  // And possibly send a warning message

    int index = (angle - _angle_shift) / _angle_interval;                                 // Find the index of the interval
    float remainder_angle = (int)((angle - _angle_shift) * 10) % (_angle_interval * 10);  // find the angle inside the interval
    remainder_angle /= 10.0;                                                              // Shifts back the value

    if (0 == remainder_angle) return _ref_points[index];  // Case of exact matching

    float estimate = (_ref_points[index + 1] - _ref_points[index]);  // Case of inside matching
    estimate *= remainder_angle / _angle_interval;

    return (uint16_t)(round(estimate) + _ref_points[index]);
}

/*!
 *  @brief  Test routine, sweeps the servo passing on the reference points and the midpoints.
 */
void ServoMotor::test_sweep(void) {
    for (int i = 0; i < 2 * _ref_points_num - 1; i++) {
        set_position(i * _angle_interval / 2.0);
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
    for (int i = 2 * _ref_points_num - 1; i > 0; i--) {
        set_position(i * _angle_interval / 2.0);
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}

// --- Movement methods

/*!
 *  @brief  Move the servo to the specified value.
 *
 *  @param  position    Angle to move the servo to;
 */
void ServoMotor::set_position(float position) {
    _current_position = std::min(std::max(position, MAX_ANGLE), MIN_ANGLE);  // Clamp value
    _current_pulse_length = angle_to_pulse_length(position);

    _driver.set_pulse_length(_channel, _current_pulse_length);
}

/*!
 *  @brief  Rotates the servo by a specific value.
 *
 *  @param  angle   Angle to rotate the servo;
 */
void ServoMotor::rotate(float angle) {
    float current_position = get_position();
    set_position(angle + current_position);
}
