#include <Arduino.h>
#include "motor_control.h"
#include "constants.h"

float appliedVolts = 0;
float desiredOmega = 0;

float setMotorVoltsClamp(float volts)
{
    appliedVolts = CLAMP(volts, -V_S, V_S);

    int outPWM = appliedVolts / V_S * ANALOG_HIGH;

    // Brake mode (friendly, linear-looking)
    int outPin1 = (volts > 0) ? ANALOG_HIGH : ANALOG_HIGH + outPWM;
    int outPin2 = (volts > 0) ? ANALOG_HIGH - outPWM : ANALOG_HIGH;

    // Coast mode (nonlinear, hard to model, much more energy-efficient)
    // int outPin1 = (volts > 0) ? outPWM : 0;
    // int outPin2 = (volts > 0) ? 0 : -outPWM;

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