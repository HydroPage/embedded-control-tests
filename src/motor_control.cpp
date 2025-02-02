#include <Arduino.h>
#include "motor_control.h"
#include "constants.h"

float appliedVolts = 0;

void setMotorVolts(float volts)
{
    appliedVolts = min(V_S, max(volts, -V_S));

    int outPWM = min((int) abs(volts/V_S * 1023), 1023);
    int outPin1 = (volts > 0) ? outPWM : 0;
    int outPin2 = (volts > 0) ? 0 : -outPWM;

    analogWrite(MOTOR_PIN_1, outPin1);
    analogWrite(MOTOR_PIN_2, outPin2);
}

float getMotorVolts()
{
    return appliedVolts;
}