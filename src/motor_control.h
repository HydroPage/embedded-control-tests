#pragma once

// + is counter cw, - is cw
// Sets the voltage on the motor
// Returns the actual saturated applied voltage post clamping
float setMotorVoltsClamp(float volts);

float getMotorVolts();

void setMotorDesiredOmega(float omega);

float getMotorDesiredOmega();