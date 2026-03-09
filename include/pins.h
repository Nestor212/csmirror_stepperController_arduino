// include/pins.h
#pragma once
#include <Arduino.h>

// Motor A
constexpr uint8_t MOTOR_A_EN   = 2;
constexpr uint8_t MOTOR_A_DIR  = 3;
constexpr uint8_t MOTOR_A_STEP = 4;
constexpr uint8_t LIM_MIN_A = 5;  // lower
constexpr uint8_t LIM_MAX_A = 6;  // upper

// Motor B
constexpr uint8_t MOTOR_B_EN   = 8;
constexpr uint8_t MOTOR_B_DIR  = 9;
constexpr uint8_t MOTOR_B_STEP = 10;
constexpr uint8_t LIM_MIN_B = 11; // lower
constexpr uint8_t LIM_MAX_B = 12; // upper