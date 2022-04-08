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
PWMTick current;
PWMTick step;
int servo;
Range<PWMTick> tickRange;
PWMTick middle;
PWMTick offset;

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

void printSummary(Range<PWMTick> tickRange, PWMTick middle, PWMTick offset, AngleDeg offsetDeg,
                  AngleDeg maxRotation) {
    Serial.println("--- Summary ---");

    Serial.printf("Shortest = %d Ticks\n", tickRange.min);
    servoController.setPWM(0, 0, tickRange.min);
    delay(2000);

    Serial.printf("Longest  = %d Ticks\n", tickRange.max);
    servoController.setPWM(0, 0, tickRange.max);
    delay(2000);

    Serial.printf("Middle   = %d Ticks\n", middle);
    servoController.setPWM(0, 0, middle);
    delay(2000);

    Serial.printf("Offset   = %d Ticks\n", offset);

    PWMTick ticks0Deg = angleToTicks(0 + offsetDeg, maxRotation, tickRange);
    Serial.printf("0°       = %d Ticks\n", ticks0Deg);
    servoController.setPWM(0, 0, ticks0Deg);
    delay(2000);

    PWMTick ticks90Deg = angleToTicks(90 + offsetDeg, maxRotation, tickRange);
    Serial.printf("90°      = %d Ticks\n", ticks90Deg);
    servoController.setPWM(0, 0, ticks90Deg);
    delay(2000);

    PWMTick ticksNeg90Deg = angleToTicks(-90 + offsetDeg, maxRotation, tickRange);
    Serial.printf("-90°     = %d Ticks\n", ticksNeg90Deg);
    servoController.setPWM(0, 0, ticksNeg90Deg);
    delay(2000);

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
                PWMTick tmp = tickRange.min;
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
