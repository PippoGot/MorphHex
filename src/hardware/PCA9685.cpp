#include "PCA9685.h"

// Methods

/*!
 *  @brief  Setups the hardware to a specified or default configuration
 *
 *  @param  prescale Sets External Clock (Optional)
 */
void PCA9685::begin(uint8_t prescale) {
    reset();

    if (prescale) {
        set_ext_clk(prescale);
    } else {
        set_PWM_freq(1000);  // set to a default frequency
    }

    set_osc_freq(FREQUENCY_OSCILLATOR);  // set default internal frequency
}

/*!
 *  @brief  Sends a reset command to the PCA9685 chip over I2C
 */
void PCA9685::reset() {
    write8(MODE1, MODE1_RESTART);
    vTaskDelay(pdMS_TO_TICKS(10));
}

/*!
 *  @brief  Puts board into sleep mode
 */
void PCA9685::sleep() {
    uint8_t awake = read8(MODE1);

    uint8_t sleep = awake | MODE1_SLEEP;  // set sleep bit high
    write8(MODE1, sleep);

    vTaskDelay(pdMS_TO_TICKS(5));  // wait until cycle ends for sleep to be active
}

/*!
 *  @brief  Wakes board from sleep
 */
void PCA9685::wakeup() {
    uint8_t sleep = read8(MODE1);

    uint8_t wakeup = sleep & ~MODE1_SLEEP;  // set sleep bit low
    write8(MODE1, wakeup);
}

/*!
 *  @brief  Sets EXTCLK pin to use the external clock
 *
 *  @param  prescale Configures the prescale value to be used by the external clock
 */
void PCA9685::set_ext_clk(uint8_t prescale) {
    uint8_t oldmode = read8(MODE1);
    uint8_t newmode = (oldmode & ~MODE1_RESTART) | MODE1_SLEEP;  // sleep
    write8(MODE1, newmode);                                      // go to sleep, turn off internal oscillator

    // This sets both the SLEEP and EXTCLK bits of the MODE1 register to switch to
    // use the external clock.
    write8(MODE1, (newmode |= MODE1_EXTCLK));

    write8(PRESCALE, prescale);  // set the prescaler

    vTaskDelay(pdMS_TO_TICKS(5));

    // clear the SLEEP bit to start
    write8(MODE1, (newmode & ~MODE1_SLEEP) | MODE1_RESTART | MODE1_AI);
}

/*!
 *  @brief  Sets the PWM frequency for the entire chip (up to 1526 Hz according to the datasheet).
 *
 *  @param  freq Floating point frequency that we will attempt to match
 */
void PCA9685::set_PWM_freq(float freq) {
    // check limits
    if (freq < 1) freq = 1;
    if (freq > 3500) freq = 3500;  // Datasheet limit is 3052 = 50 MHz/(4*4096)

    float prescaleval = ((_osc_freq / (freq * 4096.0)) + 0.5) - 1;  // datasheet 7.3.5, equation 1

    if (prescaleval < PRESCALE_MIN) prescaleval = PRESCALE_MIN;
    if (prescaleval > PRESCALE_MAX) prescaleval = PRESCALE_MAX;
    uint8_t prescale = (uint8_t)prescaleval;

    uint8_t oldmode = read8(MODE1);
    uint8_t newmode = (oldmode & ~MODE1_RESTART) | MODE1_SLEEP;  // sleep
    write8(MODE1, newmode);                                      // go to sleep
    write8(PRESCALE, prescale);                                  // set the prescaler
    write8(MODE1, oldmode);

    vTaskDelay(pdMS_TO_TICKS(5));

    // This sets the MODE1 register to turn on auto increment.
    write8(MODE1, oldmode | MODE1_RESTART | MODE1_AI);
}

/*!
 *  @brief  Sets the output mode of the PCA9685 to either open drain or push pull / totempole.
 *
 *  Warning: LEDs with integrated zener diodes should only be driven in open drain mode.
 *
 *  @param  totempole Totempole if true, open drain if false.
 */
void PCA9685::set_output_mode(bool totempole) {
    uint8_t oldmode = read8(MODE2);
    uint8_t newmode;

    if (totempole) {
        newmode = oldmode | MODE2_OUTDRV;
    } else {
        newmode = oldmode & ~MODE2_OUTDRV;
    }

    write8(MODE2, newmode);
}

/*!
 *  @brief  Reads set Prescale from PCA9685
 *
 *  @return prescale value
 */
uint8_t PCA9685::read_prescale() {
    return read8(PRESCALE);
}

/*!
 *  @brief  Gets the PWM output of one of the PCA9685 pins
 *
 *  @param  channel One of the PWM output pins, from 0 to 15
 *
 *  @return requested PWM output value
 */
/* translate
uint8_t PCA9685::get_PWM(uint8_t channel) {
    _i2c->requestFrom((int)_i2caddr, LED0_ON_L + 4 * channel, (int)4);
    return _i2c->read();
}
*/

/*!
 *  @brief  Sets the PWM output of one of the PCA9685 pins
 *
 *  @param  channel One of the PWM output pins, from 0 to 15
 *  @param  on At what point in the 4096-part cycle to turn the PWM output ON
 *  @param  off At what point in the 4096-part cycle to turn the PWM output OFF
 */
void PCA9685::set_PWM(uint8_t channel, uint16_t on, uint16_t off) {
    write8(LED0_ON_L + 4 * channel, on);         // on register
    write8(LED0_ON_H + 4 * channel, on >> 8);    // on register
    write8(LED0_OFF_L + 4 * channel, off);       // off register
    write8(LED0_OFF_H + 4 * channel, off >> 8);  // off register
}

/*!
 *   @brief  Helper to set pin PWM output. Sets pin without having to deal with
 * on/off tick placement and properly handles a zero value as completely off and
 * 4095 as completely on.  Optional invert parameter supports inverting the
 * pulse for sinking to ground.
 *
 *   @param  channel One of the PWM output pins, from 0 to 15
 *   @param  value The number of ticks out of 4096 to be active, should be a value from 0 to 4095 inclusive.
 *   @param  invert If true, inverts the output, defaults to 'false'
 */
void PCA9685::set_channel(uint8_t channel, uint16_t value, bool invert) {
    // Clamp value between 0 and 4095 inclusive.
    value = std::min(value, (uint16_t)4095);
    if (invert) {
        if (value == 0) {
            // Special value for signal fully on.
            set_PWM(channel, 4096, 0);
        } else if (value == 4095) {
            // Special value for signal fully off.
            set_PWM(channel, 0, 4096);
        } else {
            set_PWM(channel, 0, 4095 - value);
        }
    } else {
        if (value == 4095) {
            // Special value for signal fully on.
            set_PWM(channel, 4096, 0);
        } else if (value == 0) {
            // Special value for signal fully off.
            set_PWM(channel, 0, 4096);
        } else {
            set_PWM(channel, 0, value);
        }
    }
}

/*!
 *  @brief  Sets the PWM output of one of the PCA9685 pins based on the input microseconds, output is not precise
 *
 *  @param  channel One of the PWM output pins, from 0 to 15
 *  @param  microseconds The number of microseconds to turn the PWM output ON
 */
void PCA9685::write_microseconds(uint8_t channel, uint16_t microseconds) {
    double pulse = microseconds;
    double pulselength;
    pulselength = 1000000;  // 1,000,000 us per second

    // Read prescale
    uint16_t prescale = read_prescale();

    // Calculate the pulse for PWM based on Equation 1 from the datasheet section
    // 7.3.5
    prescale += 1;
    pulselength *= prescale;
    pulselength /= _osc_freq;

    pulse /= pulselength;

    set_PWM(channel, 0, pulse);
}

/*!
 *  @brief  Getter for the internally tracked oscillator used for freq calculations
 *
 *  @returns The frequency the PCA9685 thinks it is running at (it cannot introspect)
 */
uint32_t PCA9685::get_osc_freq() {
    return _osc_freq;
}

/*!
 *  @brief Setter for the internally tracked oscillator used for freq calculations

 *  @param freq The frequency the PCA9685 should use for frequency calculations
 */
void PCA9685::set_osc_freq(uint32_t freq) {
    _osc_freq = freq;
}