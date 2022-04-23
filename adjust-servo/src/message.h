#pragma once

#include <Arduino.h>

const char* MSG_FIND_SHORTEST_PULSE_LENGTH = R"===(
1. Move servo to the shortest PWM pulse length
2. Press OK
)===";

const char* MSG_FIND_LONGEST_PULSE_LENGTH = R"===(
1. Move servo to the longest PWM pulse length
2. Press OK
)===";

const char* MSG_ADJUST_MIDDLE = R"===(
Servo is now in the middle position.
1. Take off the servo arm
2. Put the servo arm where you want the 0˚ position to be
3. Make fine adjustments with the button controls
4. Press OK
)===";

const char* MSG_FIND_90_DEG = R"===(
1. Turn the servo by 90˚ in any direction (you can eyeball it)
2. Press OK
)===";

void printStepInstructions(int step, const char* instructions) {
    Serial.printf("--- Step %d ---%s--- Step %d ---\n", step, instructions, step);
}
