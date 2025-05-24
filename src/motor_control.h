#pragma once

// volts - how many volts to apply to the motor (+ is CCW in shaft axis)
// returns how many volts were really applied, subject to constraints
// NOTE: doesn't actually measure the voltage at the terminals for the return value
float setMotorVoltsClamp(float volts);

float getMotorVolts();

void setMotorDesiredOmega(float omega);

float getMotorDesiredOmega();