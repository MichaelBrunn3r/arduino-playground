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
typedef int PWMTick;

PWMTick angleToTicks(AngleDeg angle, AngleDeg maxAngle, Range<PWMTick> tickRange) {
    return map(angle, -maxAngle / 2, maxAngle / 2, tickRange.min, tickRange.max);
}
