#include <Adafruit_PWMServoDriver.h>
#include <Arduino.h>
#include <Wire.h>

#define SERVO_FREQ 50
#define PIN_BTN_HIGHER GPIO_NUM_33
#define PIN_BTN_LOWER GPIO_NUM_32
#define PIN_BTN_ACKNOWLEDGE GPIO_NUM_25

typedef enum {
    FIND_1ST_TICK_LIMIT,
    FIND_2ND_TICK_LIMIT,
    ADJUST_MIDDLE,
    FIND_90_DEG,
} State;

State state;

Adafruit_PWMServoDriver servoController = Adafruit_PWMServoDriver(0x40);
int currentTick;
int step;
int servo;
int tickLimits[2];
int middle;
int offset;

void initSerial() {
    Serial.begin(115200);
    while (!Serial && !Serial.available()) {
    }
    Serial.println();
}

void initButtons() {
    pinMode(PIN_BTN_HIGHER, INPUT_PULLDOWN);
    pinMode(PIN_BTN_LOWER, INPUT_PULLDOWN);
    pinMode(PIN_BTN_ACKNOWLEDGE, INPUT_PULLDOWN);
}

void initServoController() {
    servoController.begin();
    servoController.setOscillatorFrequency(27000000);
    servoController.setPWMFreq(SERVO_FREQ);
}

void reset() {
    step = 1;
    currentTick = 300;
    tickLimits[0] = 0;
    tickLimits[1] = 0;
    state = State::FIND_1ST_TICK_LIMIT;
    offset = 0;
    middle = 0;
    servoController.wakeup();
    Serial.println("Search the 1st tick limit, then press ACKNOWLEDGE.");
}

void setup() {
    initSerial();
    initButtons();
    initServoController();

    reset();
    delay(1000);
}

void handleTickControlls() {
    if (digitalRead(PIN_BTN_HIGHER) && digitalRead(PIN_BTN_LOWER)) {
        if (step != 1)
            step = 1;
        else
            step = 10;
        Serial.printf("Step size is now: %u\n", step);
        delay(200);
    } else if (digitalRead(PIN_BTN_HIGHER)) {
        currentTick += step;
        Serial.println(currentTick);
        servoController.setPWM(0, 0, currentTick);
    } else if (digitalRead(PIN_BTN_LOWER)) {
        currentTick -= step;
        Serial.println(currentTick);
        servoController.setPWM(0, 0, currentTick);
    }
}

int angleToTicks(int angle, int range, int tickLimits[2]) {
    return map(angle, -range / 2, range / 2, tickLimits[0], tickLimits[1]);
}

void loop() {
    if (digitalRead(PIN_BTN_ACKNOWLEDGE)) {
        switch (state) {
            case State::FIND_1ST_TICK_LIMIT: {
                Serial.println("Search the 2nd tick limit, then press ACKNOWLEDGE.");
                state = State::FIND_2ND_TICK_LIMIT;
                tickLimits[0] = currentTick;
                break;
            };
            case State::FIND_2ND_TICK_LIMIT: {
                tickLimits[1] = currentTick;
                if (!(tickLimits[0] < tickLimits[1])) {
                    int tmp = tickLimits[0];
                    tickLimits[0] = tickLimits[1];
                    tickLimits[1] = tmp;
                }
                middle = tickLimits[0] + (tickLimits[1] - tickLimits[0]) / 2;
                Serial.printf("Limits: [%d, %d], Middle: %d\n", tickLimits[0], tickLimits[1],
                              middle);

                Serial.println("Servo is now in the middle position.");
                Serial.println(
                    "Take the servo arm off and roughly put it where you want it to be when "
                    "the servo is in its middle position.");
                Serial.println(
                    "Adjust servo arm finely with button controls, then press ACKNOWLEDGE.");

                state = State::ADJUST_MIDDLE;
                currentTick = middle;
                servoController.setPWM(0, 0, middle);
                break;
            };
            case State::ADJUST_MIDDLE: {
                offset = currentTick - middle;
                Serial.printf("Middle offset: %d ticks\n", offset);

                Serial.println("Now turn the servo 90 degrees, then press ACKNOWLEDGE.");
                state = State::FIND_90_DEG;
                break;
            };
            case State::FIND_90_DEG: {
                int ticks0Deg = middle + offset;
                int ticks90Deg = currentTick;
                float ticksPerDeg = 90.0f / abs(ticks90Deg - ticks0Deg);
                int range = abs(tickLimits[0] - tickLimits[1]) * ticksPerDeg;
                int offsetDeg = offset * ticksPerDeg;

                Serial.println("A final test ...");
                Serial.printf("Lower limit: %d\n", tickLimits[0]);
                servoController.setPWM(0, 0, tickLimits[0]);
                delay(2000);
                Serial.printf("Upper limit: %d\n", tickLimits[1]);
                servoController.setPWM(0, 0, tickLimits[1]);
                delay(2000);
                Serial.println("0°");
                servoController.setPWM(0, 0, angleToTicks(0 + offsetDeg, range, tickLimits));
                delay(2000);
                Serial.println("90°");
                servoController.setPWM(0, 0, angleToTicks(90 + offsetDeg, range, tickLimits));
                delay(2000);
                Serial.println("-90°");
                servoController.setPWM(0, 0, angleToTicks(-90 + offsetDeg, range, tickLimits));
                delay(2000);

                Serial.printf("Limits: [%d, %d], Middle: %d, Offset: %d = %d°, Range: %d°\n",
                              tickLimits[0], tickLimits[1], middle, offset, offsetDeg, range);

                Serial.println("You can now test the next servo.");
                reset();
            };
        }
        delay(200);
    }

    handleTickControlls();
    delay(80);
}
