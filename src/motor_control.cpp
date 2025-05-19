#include <Arduino.h>
#include "motor_control.h"
#include "constants.h"

float appliedVolts = 0;
float desiredOmega = 0;

// volts - how many volts to apply to the motor
// returns how many volts were really applied, subject to constraints
// NOTE: doesn't actually measure the voltage at the terminals for the return value
float setMotorVoltsClamp(float volts)
{
    appliedVolts = CLAMP(volts, -V_S, V_S);

    int outPWM = (int)appliedVolts / V_S * 1023;
    int outPin1 = (volts > 0) ? outPWM : 0;
    int outPin2 = (volts > 0) ? 0 : -outPWM;

    analogWrite(MOTOR_PIN_1, outPin1);
    analogWrite(MOTOR_PIN_2, outPin2);

    return appliedVolts;
}

float getMotorVolts()
{
    return appliedVolts;
}

void setMotorDesiredOmega(float omega)
{
    desiredOmega = omega;
}

float getMotorDesiredOmega()
{
    return desiredOmega;
}