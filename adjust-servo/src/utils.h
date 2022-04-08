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

typedef int AngleDeg;
typedef int PWMTicks;

PWMTicks angleToTicks(AngleDeg angle, AngleDeg maxAngle, Range<PWMTicks> tickRange) {
    return map(angle, -maxAngle / 2, maxAngle / 2, tickRange.min, tickRange.max);
}

float ticksToPulseLengthMs(PWMTicks ticks) {
    float pwmPeriodMs = 20.0f;
    return (ticks / 4096.0f) * pwmPeriodMs;
}
