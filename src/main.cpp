#include <Arduino.h>
#include "constants.h"
#include "encoder_estimation.h"
#include "motor_control.h"

void setup()
{
    Serial.begin(115200);
    initMotorEncoding();
    pinMode(MOTOR_PIN_1, OUTPUT);
    pinMode(MOTOR_PIN_2, OUTPUT);
    pinMode(7, INPUT_PULLDOWN);
}

void loop()
{
    Serial.println(getMotorRadsPerSec());
    delay(50);

    if (digitalRead(7) == HIGH)
    {
        setMotorVolts(24);
    }
    else
    {
        setMotorVolts(0);
    }
}