#include "PCA9685.h"  // header file

// --- Initialization methods

/*!
 *  @brief  Setups the hardware to a specified or default configuration.
 *
 *  @param  freq    Initial frequency of the PWM signal (optional);
 */
void PCA9685::begin(float freq) {
    restart();
    set_PWM_freq(freq);
    set_osc_freq(FREQUENCY_OSCILLATOR);  // set default internal frequency
}

// --- Mode methods

/*!
 *  @brief  Sends a restart command to the PCA9685 chip.
 */
void PCA9685::restart() {
    write(MODE1, MODE1_RESTART);
    vTaskDelay(pdMS_TO_TICKS(5));
}

/*!
 *  @brief  Puts board into sleep mode.
 */
void PCA9685::sleep() {
    turn_on(MODE1, MODE1_SLEEP);
    vTaskDelay(pdMS_TO_TICKS(5));  // wait until cycle ends for sleep to be active
}

/*!
 *  @brief  Wakes board from sleep.
 */
void PCA9685::wakeup() {
    turn_off(MODE1, MODE1_SLEEP);
    vTaskDelay(pdMS_TO_TICKS(5));  // wait until cycle ends for sleep to be active
}

// --- Configuration setters

/*!
 *  @brief  Sets EXTCLK pin to use the external clock.
 *
 *  @param  prescale    Configures the prescale value to be used by the external clock;
 */
void PCA9685::set_ext_clk(uint8_t prescale) {
    turn_on(MODE1, MODE1_SLEEP);         // set sleep bit to turn off the internal oscillator
    turn_on(MODE1, MODE1_EXTCLK_SLEEP);  // set both SLEEP and EXTCLK bits of the MODE1 register to switch to external clock

    write(PRE_SCALE, prescale);  // set the prescaler

    vTaskDelay(pdMS_TO_TICKS(5));  // wait for the chip to be available

    restart();  // clear the SLEEP bit to start
}

/*!
 *  @brief  Sets the output mode of the PCA9685 to either open drain or push pull / totempole.
 *
 *  Warning: LEDs with integrated zener diodes should only be driven in open drain mode.
 *
 *  @param  totempole   Totempole if true, open drain if false;
 */
void PCA9685::set_output_mode(OutputMode totempole) {
    totempole ? turn_on(MODE2, MODE2_OUTDRV) : turn_off(MODE2, MODE2_OUTDRV);
}

// --- PWM/Duty-Cycle setters and getters

/*!
 *  @brief  Sets the PWM frequency for the entire chip (up to 1526 Hz according to the datasheet).
 *
 *  @param  freq    Floating point frequency that we will attempt to match;
 */
void PCA9685::set_PWM_freq(float freq) {
    uint8_t prescale = compute_prescale(freq);  // compute the prescale value

    sleep();
    write(PRE_SCALE, prescale);  // set the prescaler
    wakeup();

    restart();
}

/*!
 *  @brief  Sets the PWM output of one of the PCA9685 pins.
 *
 *  @param  channel     One of the PWM output pins, from 0 to 15;
 *  @param  on          At what point in the 4096-part cycle to turn the PWM output ON;
 *  @param  off         At what point in the 4096-part cycle to turn the PWM output OFF;
 */
void PCA9685::set_PWM_toggles(Channel channel, uint16_t on, uint16_t off) {
    turn_on(MODE1, MODE1_AI);  // turn on auto increment mode

    // write data
    uint8_t data[] = {(uint8_t)on, (uint8_t)(on >> 8), (uint8_t)off, (uint8_t)(off >> 8)};
    write(LED0_ON_L + 4 * channel, data, 4);

    turn_off(MODE1, MODE1_AI);  // turn off auto increment mode
}

/*!
 *   @brief  Sets pin PWM output without having to deal with on/off tick placement.
 *   Properly handles a zero value as completely off and 4095 as completely on.
 *
 *   @param  channel    One of the PWM output pins, from 0 to 15;
 *   @param  length     The number of ticks out of 4096 to be active, should be a value from 0 to 4095 inclusive;
 */
void PCA9685::set_pulse_length(Channel channel, uint16_t length) {
    length = std::min(length, (uint16_t)4095);  // clamp value between 0 and 4095 inclusive

    switch (length) {
        case 4095:
            set_PWM_toggles(channel, 4096, 0);  // special value for signal fully on
            break;

        case 0:
            set_PWM_toggles(channel, 0, 4096);  // special value for signal fully off
            break;

        default:
            set_PWM_toggles(channel, 0, length);
            break;
    }
}

/*!
 *   @brief  Gets the pulse length of the specified channel.
 *
 *   @param  channel    One of the PWM output pins, from 0 to 15;
 */
uint16_t PCA9685::get_pulse_length(Channel channel) {
    turn_on(MODE1, MODE1_AI);  // turn on auto increment mode

    // read data
    uint8_t data8[4];
    read(LED0_ON_L + 4 * channel, data8, 4);

    turn_off(MODE1, MODE1_AI);  // turn off auto increment mode

    // convert data
    uint16_t data16[4];
    for (int i = 0; i < 4; i++) {
        data16[i] = (uint16_t)data8[i];
    }
    uint16_t on = data16[0] + (data16[1] << 8);
    uint16_t off = data16[2] + (data16[3] << 8);

    return on < off ? off - on : on - off;
}

/*!
 *   @brief  Sets pin PWM output at desired duty cycle value.
 *
 *   @param  channel    One of the PWM output pins, from 0 to 15;
 *   @param  length     The desired duty cycle value;
 */
void PCA9685::set_duty_cycle(Channel channel, float duty_cycle) {
    uint16_t pulse_length = compute_pulse_length(duty_cycle);
    set_pulse_length(channel, pulse_length);
}

/*!
 *  @brief  Gets the duty cycle of one of the PCA9685 pins.
 *
 *  @param  channel     One of the PWM output pins, from 0 to 15;
 *
 *  @return  Duty cycle value of the requested pin;
 */
float PCA9685::get_duty_cycle(Channel channel) {
    uint16_t pulse_length = get_pulse_length(channel);
    return compute_duty_cycle(pulse_length);
}

/*!
 *  @brief  Sets the PWM output of one of the PCA9685 pins based on the input microseconds, output is not precise.
 *
 *  @param  channel         One of the PWM output pins, from 0 to 15;
 *  @param  microseconds    The number of microseconds to turn the PWM output ON;
 */
void PCA9685::write_microseconds(Channel channel, uint16_t microseconds) {
    uint16_t prescale = read_prescale();                           // read prescale
    float us_per_PWM_period = compute_PWM_microseconds(prescale);  // find how many microseconds there are in one PWM period

    float duty_cycle;
    (microseconds > us_per_PWM_period) ? duty_cycle = 1.0 : duty_cycle = microseconds / us_per_PWM_period;
    duty_cycle *= 100;

    uint16_t pulse = compute_pulse_length(duty_cycle);

    set_PWM_toggles(channel, 0, pulse);
}

// --- Oscillator frequency and Prescaler setters and getters

/*!
 *  @brief  Setter for the internally tracked oscillator used for freq calculations.

 *  @param  freq The frequency the PCA9685 should use for frequency calculations;
 */
void PCA9685::set_osc_freq(uint32_t freq) {
    _osc_freq = freq;
}

/*!
 *  @brief  Getter for the internally tracked oscillator used for freq calculations.
 *
 *  @returns    The frequency the PCA9685 thinks it is running at (it cannot introspect);
 */
uint32_t PCA9685::get_osc_freq() {
    return _osc_freq;
}

/*!
 *  @brief  Reads set Prescale from PCA9685.
 *
 *  @return  Prescale value;
 */
uint8_t PCA9685::read_prescale() {
    return read(PRE_SCALE);
}

// --- Utility methods

/*!
 *  @brief  Sets a bit of the chosen register.
 *
 *  @param  mode_register   The register whose bit is to be set;
 *  @param  mode_mask       The bit (or bits) to be set;
 */
void PCA9685::turn_on(RegisterAddress mode_register, ModeBit mode_mask) {
    uint8_t old_mode = read(mode_register);   // read current mode
    uint8_t new_mode = old_mode | mode_mask;  // sets the mode bit
    write(mode_register, new_mode);           // writes the new mode
}

/*!
 *  @brief  Resets a bit of the chosen register.
 *
 *  @param  mode_register   The register whose bit is to be restart;
 *  @param  mode_mask       The bit (or bits) to be restart;
 */
void PCA9685::turn_off(RegisterAddress mode_register, ModeBit mode_mask) {
    uint8_t old_mode = read(mode_register);    // read current mode
    uint8_t new_mode = old_mode & ~mode_mask;  // restarts the mode bit
    write(mode_register, new_mode);            // writes the new mode
}

/*!
 *   @brief  Computes the prescale value based on the given frequency.
 *   The value is clamped in a specific range [3, 255].
 *
 *   @param  freq   The desired target frequency;
 *
 *   @return  The prescale value corresponding to the target frequency;
 */
uint8_t PCA9685::compute_prescale(float freq) {
    float prescale_val = ((_osc_freq / (freq * 4096.0)) + 0.5) - 1;  // datasheet 7.3.5, equation 1

    // check limits and clamp value
    if (prescale_val > PRE_SCALE_MAX) prescale_val = PRE_SCALE_MAX;
    if (prescale_val < PRE_SCALE_MIN) prescale_val = PRE_SCALE_MIN;
    // TODO maybe send out message to alert it was not possible if error > 1

    return (uint8_t)prescale_val;
}

/*!
 *   @brief  Computes the number of microseconds in a PWM period based on the given prescale.
 *
 *   @param  prescale   The current prescale value;
 *
 *   @return  The number of microseconds in a PWM period for the given prescale value;
 */
float PCA9685::compute_PWM_microseconds(uint8_t prescale) {
    float us_units = (1000000.0 * 4096.0) / _osc_freq;
    return (prescale + 1) * us_units;
}

/*!
 *   @brief  Computes pulse length based on a given duty cycle value.
 *
 *   @param  duty_cycle     The desired target duty cycle value;
 *
 *   @return  The pulse length value corresponding to the target duty cycle;
 */
uint16_t PCA9685::compute_pulse_length(float duty_cycle) {
    if (duty_cycle < 0.0) duty_cycle = 0.0;  // clamping value to min, TODO log a message

    float pulse_length = duty_cycle / 100 * 4095;
    uint16_t result = (uint16_t)pulse_length;

    if (result > 4095) result = 4095;  // clamping value to max, TODO log a message

    return result;
}

/*!
 *   @brief  Computes the duty cycle value based on the given pulse length.
 *
 *   @param  pulse_length   The desired target pulse length;
 *
 *   @return  The duty cycle value corresponding to the target pulse length;
 */
float PCA9685::compute_duty_cycle(uint16_t pulse_length) {
    return pulse_length / 4095.0 * 100;
}