#pragma once

#include <Arduino.h>

template <typename T> struct Range {
    union {
        T min;
        T lower;
    };
    union {
        T max;
        T upper;
    };
};

typedef float AngleDeg;
typedef int PWMTicks;

PWMTicks angleToTicks(AngleDeg angle, AngleDeg maxAngle, Range<PWMTicks> tickRange) {
    return int(map(angle, -maxAngle / 2, maxAngle / 2, tickRange.min, tickRange.max));
}

float ticksToPulseLengthMs(PWMTicks ticks, Adafruit_PWMServoDriver driver) {
    // Inverse of Equation 1 of the datsheet
    static float msPerSecond = 1000.0;
    return ticks * ((msPerSecond * (driver.readPrescale() + 1)) / driver.getOscillatorFrequency());
}
