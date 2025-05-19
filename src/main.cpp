#include <Arduino.h>
#include "constants.h"
#include "control_system.h"
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
    Serial.print(getMotorPosRads());
    Serial.print(',');
    Serial.print(getMotorRadsPerSec());
    Serial.println();
    delay(50);

    if (digitalRead(7) == HIGH)
    {
        // setMotorDesiredOmega(30 * sinf(TWO_PI*millis() / 1000 / 2));

        setMotorDesiredOmega(45);
    }
    else
    {
        setMotorDesiredOmega(0);
    }
}