#include <Adafruit_PWMServoDriver.h>
#include <Arduino.h>
#include <Wire.h>

#include "controller.h"
#include "message.h"
#include "table.h"
#include "utils.h"

#define SERVO_FREQ 50
#define PIN_BTN_INC GPIO_NUM_33
#define PIN_BTN_DEC GPIO_NUM_32
#define PIN_BTN_OK GPIO_NUM_25

typedef enum {
    FIND_SHORTEST_PULSE_LENGTH,
    FIND_LONGEST_PULSE_LENGTH,
    ADJUST_MIDDLE,
    FIND_90_DEG,
} State;

State state;

Adafruit_PWMServoDriver servoDriver = Adafruit_PWMServoDriver(0x40);
ServoController servo(0, GPIO_NUM_4, &servoDriver);

Range<PWMTicks> tickRange;
PWMTicks middle;
PWMTicks offset;
PWMTicks minTicksGuess = -1;
PWMTicks maxTicksGuess = -1;

void reset() {
    if (minTicksGuess <= 0 || maxTicksGuess <= 0) {
        minTicksGuess = maxTicksGuess = 300;
    } else {
        minTicksGuess = tickRange.min + 20;
        maxTicksGuess = tickRange.max - 20;
    }

    tickRange.min = tickRange.max = 0;
    offset = 0;
    middle = 0;

    servo.reset();
    servo.set(minTicksGuess);

    state = State::FIND_SHORTEST_PULSE_LENGTH;
    printStepInstructions(state, MSG_FIND_SHORTEST_PULSE_LENGTH);
}

void setup() {
    // Init Serial
    Serial.begin(115200);
    while (!Serial && !Serial.available()) {
    }
    Serial.println();

    // Init buttons
    pinMode(PIN_BTN_INC, INPUT_PULLDOWN);
    pinMode(PIN_BTN_DEC, INPUT_PULLDOWN);
    pinMode(PIN_BTN_OK, INPUT_PULLDOWN);

    // Init servo driver
    servoDriver.begin();
    servoDriver.setOscillatorFrequency(27000000);
    servoDriver.setPWMFreq(SERVO_FREQ);

    // Init servo controller
    servo.init();

    reset();
    delay(500);
}

void handleTickControlls() {
    if (digitalRead(PIN_BTN_INC) && digitalRead(PIN_BTN_DEC)) {
        servo.toggleStepSize();
        Serial.printf("Set step size to: %u\n", servo.stepSize);
        delay(200);
    } else if (digitalRead(PIN_BTN_INC)) {
        servo.inc();
        Serial.printf("Ticks: %d, Feedback: %d\n", servo.ticks, servo.getPosFeedback());
    } else if (digitalRead(PIN_BTN_DEC)) {
        servo.dec();
        Serial.printf("Ticks: %d, Feedback: %d\n", servo.ticks, servo.getPosFeedback());
    }
}

void printSummary(Range<PWMTicks> tickRange, PWMTicks offset, AngleDeg offsetDeg,
                  AngleDeg maxRotation) {
    Serial.println("--- Summary ---");
    Serial.printf(R"===(
- Rotation range: %f˚
- Offset: %d Ticks = %f ms = %f˚
- PWM Period: %d Hz
- Oscillator Frequency : %f MHz
)===",
                  maxRotation, offset, ticksToPulseLengthMs(offset, servoDriver), offsetDeg,
                  SERVO_FREQ, servoDriver.getOscillatorFrequency() / 1000000.0f);

    Table<4> table({"        ", "Ticks", "Pulse length", "Feedback"},
                   "| %8s | %5d | %9f ms | %8d |\n");
    table.printHeader();

    servo.set(tickRange.min);
    delay(1000);
    table.printRow("Shortest", tickRange.min, ticksToPulseLengthMs(tickRange.min, servoDriver),
                   servo.getPosFeedback());

    servo.set(tickRange.max);
    delay(1000);
    table.printRow("Longest", tickRange.max, ticksToPulseLengthMs(tickRange.max, servoDriver),
                   servo.getPosFeedback());

    PWMTicks ticks0Deg = angleToTicks(0.0 + offsetDeg, maxRotation, tickRange);
    servo.set(ticks0Deg);
    delay(1000);
    table.printRow("0 deg", ticks0Deg, ticksToPulseLengthMs(ticks0Deg, servoDriver),
                   servo.getPosFeedback());

    PWMTicks ticks90Deg = angleToTicks(90.0 + offsetDeg, maxRotation, tickRange);
    servo.set(ticks90Deg);
    delay(1000);
    table.printRow("90 deg", ticks90Deg, ticksToPulseLengthMs(ticks90Deg, servoDriver),
                   servo.getPosFeedback());

    PWMTicks ticksNeg90Deg = angleToTicks(-90.0 + offsetDeg, maxRotation, tickRange);
    servo.set(ticksNeg90Deg);
    delay(1000);
    table.printRow("-90 deg", ticksNeg90Deg, ticksToPulseLengthMs(ticksNeg90Deg, servoDriver),
                   servo.getPosFeedback());

    Serial.println("\nmin, max, offset in degree, rotation range:");
    Serial.printf("%d, %d, %f, %f\n", tickRange.min, tickRange.max, offsetDeg, maxRotation);
    Serial.println("--- Summary ---");
}

void handleOK() {
    switch (state) {
        case State::FIND_SHORTEST_PULSE_LENGTH: {
            state = State::FIND_LONGEST_PULSE_LENGTH;
            printStepInstructions(state, MSG_FIND_LONGEST_PULSE_LENGTH);
            tickRange.min = servo.ticks;
            servo.set(maxTicksGuess);
            break;
        };
        case State::FIND_LONGEST_PULSE_LENGTH: {
            tickRange.max = servo.ticks;
            if (!(tickRange.min < tickRange.max)) {
                PWMTicks tmp = tickRange.min;
                tickRange.min = tickRange.max;
                tickRange.max = tmp;
            }
            middle = tickRange.min + (tickRange.max - tickRange.min) / 2;

            state = State::ADJUST_MIDDLE;
            printStepInstructions(state, MSG_ADJUST_MIDDLE);
            servo.set(middle);
            break;
        };
        case State::ADJUST_MIDDLE: {
            offset = servo.ticks - middle;

            state = State::FIND_90_DEG;
            printStepInstructions(state, MSG_FIND_90_DEG);
            servo.set(tickRange.min + abs((tickRange.min - tickRange.max) / 8));
            break;
        };
        case State::FIND_90_DEG: {
            float ticksPerDeg = 90.0f / abs(servo.ticks - (middle + offset));
            AngleDeg maxRotation = abs(tickRange.min - tickRange.max) * ticksPerDeg;
            AngleDeg offsetDeg = offset * ticksPerDeg;
            printSummary(tickRange, offset, offsetDeg, maxRotation);
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
