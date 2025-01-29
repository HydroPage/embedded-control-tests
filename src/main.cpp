#include <Arduino.h>
#include <RotaryEncoder.h>

// Only 1, 2, and 9 can be interrupted (I think)
#define ENC_PIN_1 1
#define ENC_PIN_2 2
#define ENCODER_TICKS_PER_REV 1203.2f
#define THETA_0 0

double theta;
double omega;

RotaryEncoder encoder(ENC_PIN_1, ENC_PIN_2, RotaryEncoder::LatchMode::TWO03);

float wrapPlusMinusPi(float angleRads)
{
    angleRads = fmod(angleRads + PI, TWO_PI);
    if (angleRads < 0)
        angleRads += TWO_PI;
    return angleRads - PI;
}

float encoderTicksToRadians(long encoderTicks)
{
    return wrapPlusMinusPi(encoderTicks / ENCODER_TICKS_PER_REV * TWO_PI);
}

float radiansToEncoderTicks(float rads)
{
    return rads / TWO_PI * ENCODER_TICKS_PER_REV;
}

long encoderPos;

IRAM_ATTR void onEncoderPulse()
{
    encoder.tick();
    
    static long prevEncoderPos = radiansToEncoderTicks(THETA_0);
    static unsigned long prevTickMicros = 0;

    encoderPos = encoder.getPosition() - radiansToEncoderTicks(THETA_0);

    if (encoderPos != prevEncoderPos)
    {
        unsigned long now = micros();

        theta = encoderTicksToRadians(encoderPos);
        float prevTheta = encoderTicksToRadians(prevEncoderPos);

        omega = (theta - prevTheta) * 1000000.0f / (now - prevTickMicros);

        prevEncoderPos = encoderPos;
        prevTickMicros = now;
    }
}

void setup()
{
    Serial.begin(9600);

    // Encoder ticking interrupts
    attachInterrupt(digitalPinToInterrupt(ENC_PIN_1), onEncoderPulse, CHANGE);
    attachInterrupt(digitalPinToInterrupt(ENC_PIN_2), onEncoderPulse, CHANGE);
}

void loop()
{
    Serial.printf("Theta: %.2f, Omega: %.2f\n", theta, omega);
    delay(50);
}