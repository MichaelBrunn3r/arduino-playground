#include <Adafruit_PWMServoDriver.h>
#include <Arduino.h>
#include <Wire.h>

#include "strings.h"
#include "table.h"
#include "utils.h"

#define SERVO_FREQ 50
#define PIN_BTN_INC GPIO_NUM_33
#define PIN_BTN_DEC GPIO_NUM_32
#define PIN_BTN_OK GPIO_NUM_25
#define PIN_POS_FEEDBACK GPIO_NUM_4

typedef enum {
    FIND_SHORTEST_PULSE_LENGTH,
    FIND_LONGEST_PULSE_LENGTH,
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
PWMTicks minTicksGuess = -1;
PWMTicks maxTicksGuess = -1;

void setServoTicks(PWMTicks ticks) {
    current = ticks;
    servoController.setPWM(0, 0, current);
}

void reset() {
    if (minTicksGuess <= 0 || maxTicksGuess <= 0) {
        minTicksGuess = maxTicksGuess = 300;
    } else {
        minTicksGuess = tickRange.min + 20;
        maxTicksGuess = tickRange.max - 20;
    }

    step = 1;
    tickRange.min = tickRange.max = 0;
    state = State::FIND_SHORTEST_PULSE_LENGTH;
    offset = 0;
    middle = 0;
    setServoTicks(minTicksGuess);
    Serial.printf(MSG_STEP_1);
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

    // Init position feedback pin
    pinMode(PIN_POS_FEEDBACK, INPUT);

    // Init servo controller
    servoController.begin();
    servoController.setOscillatorFrequency(27000000);
    servoController.setPWMFreq(SERVO_FREQ);

    reset();
    delay(500);
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
        setServoTicks(current + step);
        Serial.printf("Ticks: %d, Feedback: %d\n", current, analogRead(PIN_POS_FEEDBACK));
    } else if (digitalRead(PIN_BTN_DEC)) {
        setServoTicks(current - step);
        Serial.printf("Ticks: %d, Feedback: %d\n", current, analogRead(PIN_POS_FEEDBACK));
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
                  maxRotation, offset, ticksToPulseLengthMs(offset, servoController), offsetDeg,
                  SERVO_FREQ, servoController.getOscillatorFrequency() / 1000000.0f);

    Table<4> table({"        ", "Ticks", "Pulse length", "Feedback"},
                   "| %8s | %5d | %9f ms | %8d |\n");
    table.printHeader();

    servoController.setPWM(0, 0, tickRange.min);
    delay(1000);
    table.printRow("Shortest", tickRange.min, ticksToPulseLengthMs(tickRange.min, servoController),
                   analogRead(PIN_POS_FEEDBACK));

    servoController.setPWM(0, 0, tickRange.max);
    delay(1000);
    table.printRow("Longest", tickRange.max, ticksToPulseLengthMs(tickRange.max, servoController),
                   analogRead(PIN_POS_FEEDBACK));

    PWMTicks ticks0Deg = angleToTicks(0.0 + offsetDeg, maxRotation, tickRange);
    servoController.setPWM(0, 0, ticks0Deg);
    delay(1000);
    table.printRow("0 deg", ticks0Deg, ticksToPulseLengthMs(ticks0Deg, servoController),
                   analogRead(PIN_POS_FEEDBACK));

    PWMTicks ticks90Deg = angleToTicks(90.0 + offsetDeg, maxRotation, tickRange);
    servoController.setPWM(0, 0, ticks90Deg);
    delay(1000);
    table.printRow("90 deg", ticks90Deg, ticksToPulseLengthMs(ticks90Deg, servoController),
                   analogRead(PIN_POS_FEEDBACK));

    PWMTicks ticksNeg90Deg = angleToTicks(-90.0 + offsetDeg, maxRotation, tickRange);
    servoController.setPWM(0, 0, ticksNeg90Deg);
    delay(1000);
    table.printRow("-90 deg", ticksNeg90Deg, ticksToPulseLengthMs(ticksNeg90Deg, servoController),
                   analogRead(PIN_POS_FEEDBACK));

    Serial.println("\nmin, max, offset in degree, rotation range:");
    Serial.printf("%d, %d, %f, %f\n", tickRange.min, tickRange.max, offsetDeg, maxRotation);
    Serial.println("--- Summary ---");
}

void handleOK() {
    switch (state) {
        case State::FIND_SHORTEST_PULSE_LENGTH: {
            Serial.printf(MSG_STEP_2);
            state = State::FIND_LONGEST_PULSE_LENGTH;
            tickRange.min = current;
            setServoTicks(maxTicksGuess);
            break;
        };
        case State::FIND_LONGEST_PULSE_LENGTH: {
            tickRange.max = current;
            if (!(tickRange.min < tickRange.max)) {
                PWMTicks tmp = tickRange.min;
                tickRange.min = tickRange.max;
                tickRange.max = tmp;
            }
            middle = tickRange.min + (tickRange.max - tickRange.min) / 2;

            Serial.printf(MSG_STEP_3);

            state = State::ADJUST_MIDDLE;
            setServoTicks(middle);
            break;
        };
        case State::ADJUST_MIDDLE: {
            offset = current - middle;

            Serial.printf(MSG_STEP_4);
            state = State::FIND_90_DEG;
            setServoTicks(tickRange.min + abs((tickRange.min - tickRange.max) / 8));
            break;
        };
        case State::FIND_90_DEG: {
            float ticksPerDeg = 90.0f / abs(current - (middle + offset));
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
