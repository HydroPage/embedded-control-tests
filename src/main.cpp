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

    analogWriteResolution(ANALOG_WRITE_RES_BITS);
    analogWriteFrequency(ANALOG_WRITE_FREQ_HZ);
}

void loop()
{
    static unsigned long prevPrint = 0;
    unsigned long now = millis();
    if (now - prevPrint >= 50)
    {
        prevPrint = now;
        Serial.print(getMotorVolts());
        Serial.print(',');
        Serial.print(getMotorRadsPerSec());
        Serial.println();
    }

    if (digitalRead(7) == HIGH)
    {
        // setMotorDesiredOmega(50 * sinf(TWO_PI*millis() / 1000 / 2));

        // setMotorDesiredOmega(50);

        setMotorDesiredOmega(100.0f * (millis() % 10000) / 10000.0f - 50.0f);
    }
    else
    {
        setMotorDesiredOmega(0);
    }
}