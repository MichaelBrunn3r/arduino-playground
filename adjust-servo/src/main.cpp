#include <Adafruit_PWMServoDriver.h>
#include <Arduino.h>
#include <Wire.h>

#include "utils.h"

#define SERVO_FREQ 50
#define PIN_BTN_INC GPIO_NUM_33
#define PIN_BTN_DEC GPIO_NUM_32
#define PIN_BTN_OK GPIO_NUM_25

typedef enum {
    FIND_1ST_PWM_PULSE_LENGTH_LIMIT,
    FIND_2ND_PWM_PULSE_LENGTH_LIMIT,
    ADJUST_MIDDLE,
    FIND_90_DEG,
} State;

State state;

Adafruit_PWMServoDriver servoController = Adafruit_PWMServoDriver(0x40);
PWMTicks current;
PWMTicks step;
int servo;
Range<PWMTicks> tickRange;
PWMTicks middle;
PWMTicks offset;

void initSerial() {
    Serial.begin(115200);
    while (!Serial && !Serial.available()) {
    }
    Serial.println();
}

void initButtons() {
    pinMode(PIN_BTN_INC, INPUT_PULLDOWN);
    pinMode(PIN_BTN_DEC, INPUT_PULLDOWN);
    pinMode(PIN_BTN_OK, INPUT_PULLDOWN);
}

void initServoController() {
    servoController.begin();
    servoController.setOscillatorFrequency(27000000);
    servoController.setPWMFreq(SERVO_FREQ);
}

void reset() {
    step = 1;
    current = 300;
    tickRange.min = tickRange.max = 0;
    state = State::FIND_1ST_PWM_PULSE_LENGTH_LIMIT;
    offset = 0;
    middle = 0;
    servoController.wakeup();
    Serial.printf(R"===(
--- Step 1 ---
1. Move servo to its 1st PWM pulse length limit
2. Press OK
--- Step 1 ---
)===");
}

void setup() {
    initSerial();
    initButtons();
    initServoController();

    reset();
    delay(1000);
}

void handleTickControlls() {
    if (digitalRead(PIN_BTN_INC) && digitalRead(PIN_BTN_DEC)) {
        if (step != 1)
            step = 1;
        else
            step = 10;
        Serial.printf("Set step size to: %u\n", step);
        delay(200);
    } else if (digitalRead(PIN_BTN_INC)) {
        current += step;
        Serial.println(current);
        servoController.setPWM(0, 0, current);
    } else if (digitalRead(PIN_BTN_DEC)) {
        current -= step;
        Serial.println(current);
        servoController.setPWM(0, 0, current);
    }
}

void printSummary(Range<PWMTicks> tickRange, PWMTicks middle, PWMTicks offset, AngleDeg offsetDeg,
                  AngleDeg maxRotation) {
    Serial.println("--- Summary ---");
    Serial.printf("|----------|-------|--------------|\n");
    Serial.printf("|          | Ticks | Pulse length |\n");
    Serial.printf("|----------|-------|--------------|\n");
    Serial.printf("| %8s | %5d | %9f ms |\n", "Shortest", tickRange.min,
                  ticksToPulseLengthMs(tickRange.min));
    servoController.setPWM(0, 0, tickRange.min);
    delay(2000);

    Serial.printf("|----------|-------|--------------|\n");
    Serial.printf("| %8s | %5d | %9f ms |\n", "Longest", tickRange.max,
                  ticksToPulseLengthMs(tickRange.max));
    servoController.setPWM(0, 0, tickRange.max);
    delay(2000);

    Serial.printf("|----------|-------|--------------|\n");
    Serial.printf("| %8s | %5d | %9f ms |\n", "Middle", middle, ticksToPulseLengthMs(middle));
    servoController.setPWM(0, 0, middle);
    delay(2000);

    Serial.printf("|----------|-------|--------------|\n");
    Serial.printf("| %8s | %5d | %9f ms |\n", "Offset", offset, ticksToPulseLengthMs(offset));

    PWMTicks ticks0Deg = angleToTicks(0 + offsetDeg, maxRotation, tickRange);
    Serial.printf("|----------|-------|--------------|\n");
    Serial.printf("| %9s | %5d | %9f ms |\n", "0°", ticks0Deg, ticksToPulseLengthMs(ticks0Deg));
    servoController.setPWM(0, 0, ticks0Deg);
    delay(2000);

    PWMTicks ticks90Deg = angleToTicks(90 + offsetDeg, maxRotation, tickRange);
    Serial.printf("|----------|-------|--------------|\n");
    Serial.printf("| %9s | %5d | %9f ms |\n", "90°", ticks90Deg, ticksToPulseLengthMs(ticks90Deg));
    servoController.setPWM(0, 0, ticks90Deg);
    delay(2000);

    PWMTicks ticksNeg90Deg = angleToTicks(-90 + offsetDeg, maxRotation, tickRange);
    Serial.printf("|----------|-------|--------------|\n");
    Serial.printf("| %9s | %5d | %9f ms |\n", "-90°", ticksNeg90Deg,
                  ticksToPulseLengthMs(ticksNeg90Deg));
    servoController.setPWM(0, 0, ticksNeg90Deg);
    delay(2000);

    Serial.printf("|----------|-------|--------------|\n");
    Serial.println("--- Summary ---");
}

void handleOK() {
    switch (state) {
        case State::FIND_1ST_PWM_PULSE_LENGTH_LIMIT: {
            Serial.printf(R"===(
--- Step 2 ---
1. Move servo to its 2nd PWM pulse length limit
2. Press OK
--- Step 2 ---
)===");
            state = State::FIND_2ND_PWM_PULSE_LENGTH_LIMIT;
            tickRange.min = current;
            break;
        };
        case State::FIND_2ND_PWM_PULSE_LENGTH_LIMIT: {
            tickRange.max = current;
            if (!(tickRange.min < tickRange.max)) {
                PWMTicks tmp = tickRange.min;
                tickRange.min = tickRange.max;
                tickRange.max = tmp;
            }
            middle = tickRange.min + (tickRange.max - tickRange.min) / 2;

            Serial.printf(R"===(
--- Step 3 ---
Servo is now in the middle position.
1. Take off the servo arm
2. Put the servo arm where you want the 0˚ position to be
3. Make fine adjustments with the button controls
4. Press OK
--- Step 3 ---
)===");

            state = State::ADJUST_MIDDLE;
            current = middle;
            servoController.setPWM(0, 0, middle);
            break;
        };
        case State::ADJUST_MIDDLE: {
            offset = current - middle;
            Serial.printf("Middle offset: %d ticks\n", offset);

            Serial.printf(R"===(
--- Step 4 ---
1. Turn the servo by 90˚ in any direction (you can eyeball it)
2. Press OK
--- Step 4 ---
)===");
            state = State::FIND_90_DEG;
            break;
        };
        case State::FIND_90_DEG: {
            float ticksPerDeg = 90.0f / abs(current - (middle + offset));
            AngleDeg maxRotation = abs(tickRange.min - tickRange.max) * ticksPerDeg;
            AngleDeg offsetDeg = offset * ticksPerDeg;
            printSummary(tickRange, middle, offset, offsetDeg, maxRotation);
            reset();
        };
    }
}

void loop() {
    if (digitalRead(PIN_BTN_OK)) {
        handleOK();
        delay(200);
    }

    handleTickControlls();
    delay(80);
}
