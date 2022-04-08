#include <Adafruit_PWMServoDriver.h>
#include <Arduino.h>
#include <Wire.h>

#include "utils.h"

#define SERVO_FREQ 50
#define PIN_BTN_INC GPIO_NUM_33
#define PIN_BTN_DEC GPIO_NUM_32
#define PIN_BTN_OK GPIO_NUM_25

typedef enum {
    FIND_1ST_TICK_LIMIT,
    FIND_2ND_TICK_LIMIT,
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
    state = State::FIND_1ST_TICK_LIMIT;
    offset = 0;
    middle = 0;
    servoController.wakeup();
    Serial.println("Search the 1st tick limit, then press OK.");
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
        Serial.printf("Step size is now: %u\n", step);
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

void loop() {
    if (digitalRead(PIN_BTN_OK)) {
        switch (state) {
            case State::FIND_1ST_TICK_LIMIT: {
                Serial.println("Search the 2nd tick limit, then press OK.");
                state = State::FIND_2ND_TICK_LIMIT;
                tickRange.min = current;
                break;
            };
            case State::FIND_2ND_TICK_LIMIT: {
                tickRange.max = current;
                if (!(tickRange.min < tickRange.max)) {
                    int tmp = tickRange.min;
                    tickRange.min = tickRange.max;
                    tickRange.max = tmp;
                }
                middle = tickRange.min + (tickRange.max - tickRange.min) / 2;
                Serial.printf("Limits: [%d, %d], Middle: %d\n", tickRange.min, tickRange.max,
                              middle);

                Serial.println("Servo is now in the middle position.");
                Serial.println(
                    "Take the servo arm off and roughly put it where you want it to be when "
                    "the servo is in its middle position.");
                Serial.println("Adjust servo arm finely with button controls, then press OK.");

                state = State::ADJUST_MIDDLE;
                current = middle;
                servoController.setPWM(0, 0, middle);
                break;
            };
            case State::ADJUST_MIDDLE: {
                offset = current - middle;
                Serial.printf("Middle offset: %d ticks\n", offset);

                Serial.println("Now turn the servo 90 degrees, then press OK.");
                state = State::FIND_90_DEG;
                break;
            };
            case State::FIND_90_DEG: {
                int ticks0Deg = middle + offset;
                int ticks90Deg = current;
                float ticksPerDeg = 90.0f / abs(ticks90Deg - ticks0Deg);
                int range = abs(tickRange.min - tickRange.max) * ticksPerDeg;
                int offsetDeg = offset * ticksPerDeg;

                Serial.println("A final test ...");
                Serial.printf("Lower limit: %d\n", tickRange.min);
                servoController.setPWM(0, 0, tickRange.min);
                delay(2000);
                Serial.printf("Upper limit: %d\n", tickRange.max);
                servoController.setPWM(0, 0, tickRange.max);
                delay(2000);
                Serial.println("0°");
                servoController.setPWM(0, 0, angleToTicks(0 + offsetDeg, range, tickRange));
                delay(2000);
                Serial.println("90°");
                servoController.setPWM(0, 0, angleToTicks(90 + offsetDeg, range, tickRange));
                delay(2000);
                Serial.println("-90°");
                servoController.setPWM(0, 0, angleToTicks(-90 + offsetDeg, range, tickRange));
                delay(2000);

                Serial.printf("Limits: [%d, %d], Middle: %d, Offset: %d = %d°, Range: %d°\n",
                              tickRange.min, tickRange.max, middle, offset, offsetDeg, range);

                Serial.println("You can now test the next servo.");
                reset();
            };
        }
        delay(200);
    }

    handleTickControlls();
    delay(80);
}
