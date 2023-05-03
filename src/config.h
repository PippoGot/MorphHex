#pragma once

// --- I2C interface
#include <I2C.h>              // Module
I2C_Bus_t& I2C_bus = I2C_B0;  // I2C bus object

const gpio_num_t SDA_PIN = GPIO_NUM_21;  // SDA pin
const gpio_num_t SCL_PIN = GPIO_NUM_22;  // SCL pin

// --- Servo driver
#include <PCA9685.h>  // Module

#define PCA9685_ADDR 0x40   // Device address
#define PWM_FREQ 48.828125  // PWM frequency

PCA9685 servo_driver = PCA9685(PCA9685_ADDR, I2C_bus);  // Device object

// --- Servo motors
#include <servomotor.h>  // Module

// Channel pins in the driver
#define SERVO_01_CH PCA9685::CHANNEL_00
#define SERVO_02_CH PCA9685::CHANNEL_01
#define SERVO_03_CH PCA9685::CHANNEL_02
#define SERVO_04_CH PCA9685::CHANNEL_00
#define SERVO_05_CH PCA9685::CHANNEL_00
#define SERVO_06_CH PCA9685::CHANNEL_00
#define SERVO_07_CH PCA9685::CHANNEL_00
#define SERVO_08_CH PCA9685::CHANNEL_00
#define SERVO_09_CH PCA9685::CHANNEL_00
#define SERVO_10_CH PCA9685::CHANNEL_00
#define SERVO_11_CH PCA9685::CHANNEL_00
#define SERVO_12_CH PCA9685::CHANNEL_00
#define SERVO_13_CH PCA9685::CHANNEL_00
#define SERVO_14_CH PCA9685::CHANNEL_00
#define SERVO_15_CH PCA9685::CHANNEL_00
#define SERVO_16_CH PCA9685::CHANNEL_00

// Servos reference points
std::vector<uint16_t> ref_points_1 = {128, 148, 165, 181, 196, 210, 225, 239, 257, 277, 296, 312, 331, 349, 367, 385, 402, 419, 438};
ServoMotor servo1 = ServoMotor(ref_points_1, 10, servo_driver, SERVO_01_CH, -90);

std::vector<uint16_t> ref_points_2 = {126, 142, 157, 172, 186, 201, 217, 236, 253, 270, 292, 311, 331, 349, 367, 384, 402, 422, 441};
ServoMotor servo2 = ServoMotor(ref_points_2, 10, servo_driver, SERVO_02_CH, -90);

std::vector<uint16_t> ref_points_3 = {137, 153, 171, 190, 211, 230, 250, 270, 290, 311, 331, 352, 370, 391, 411, 430, 450, 467, 484};
ServoMotor servo3 = ServoMotor(ref_points_3, 10, servo_driver, SERVO_03_CH, -90);
//
//  std::vector<uint16_t> ref_points_4 = {};
//  ServoMotor servo4 = ServoMotor(ref_points_4, 10, servo_driver, SERVO_04_CH);
//
//  std::vector<uint16_t> ref_points_5 = {};
//  ServoMotor servo5 = ServoMotor(ref_points_5, 10, servo_driver, SERVO_05_CH);
//
//  std::vector<uint16_t> ref_points_6 = {};
//  ServoMotor servo6 = ServoMotor(ref_points_6, 10, servo_driver, SERVO_06_CH);
//
//  std::vector<uint16_t> ref_points_7 = {};
//  ServoMotor servo7 = ServoMotor(ref_points_7, 10, servo_driver, SERVO_07_CH);
//
//  std::vector<uint16_t> ref_points_8 = {};
//  ServoMotor servo8 = ServoMotor(ref_points_8, 10, servo_driver, SERVO_08_CH);
//
//  std::vector<uint16_t> ref_points_9 = {};
//  ServoMotor servo9 = ServoMotor(ref_points_9, 10, servo_driver, SERVO_09_CH);
//
//  std::vector<uint16_t> ref_points_10 = {};
//  ServoMotor servo10 = ServoMotor(ref_points_10, 10, servo_driver, SERVO_10_CH);
//
//  std::vector<uint16_t> ref_points_11 = {};
//  ServoMotor servo11 = ServoMotor(ref_points_11, 10, servo_driver, SERVO_11_CH);
//
//  std::vector<uint16_t> ref_points_12 = {};
//  ServoMotor servo12 = ServoMotor(ref_points_12, 10, servo_driver, SERVO_12_CH);
//
//  std::vector<uint16_t> ref_points_13 = {};
//  ServoMotor servo13 = ServoMotor(ref_points_13, 10, servo_driver, SERVO_13_CH);
//
//  std::vector<uint16_t> ref_points_14 = {};
//  ServoMotor servo14 = ServoMotor(ref_points_14, 10, servo_driver, SERVO_14_CH);
//
//  std::vector<uint16_t> ref_points_15 = {};
//  ServoMotor servo15 = ServoMotor(ref_points_15, 10, servo_driver, SERVO_15_CH);
//
//  std::vector<uint16_t> ref_points_16 = {};
//  ServoMotor servo16 = ServoMotor(ref_points_16, 10, servo_driver, SERVO_16_CH);