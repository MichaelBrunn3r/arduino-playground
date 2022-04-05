#include <Adafruit_PWMServoDriver.h>
#include <Arduino.h>
#include <Wire.h>

#define SERVO_FREQ 50
#define PIN_BTN_HIGHER GPIO_NUM_33
#define PIN_BTN_LOWER GPIO_NUM_32

Adafruit_PWMServoDriver servoController = Adafruit_PWMServoDriver(0x40);
uint16_t current;
uint16_t step = 1;
uint16_t servo;

void initSerial() {
    Serial.begin(115200);
    while (!Serial && !Serial.available()) {
    }
    Serial.println();
}

void initButtons() {
    pinMode(PIN_BTN_HIGHER, INPUT_PULLDOWN);
    pinMode(PIN_BTN_LOWER, INPUT_PULLDOWN);
}

void initServoController() {
    servoController.begin();
    servoController.setOscillatorFrequency(27000000);
    servoController.setPWMFreq(SERVO_FREQ);
}

void setup() {
    initSerial();
    initButtons();
    initServoController();

    current = 200;
    servo = 0;
    servoController.setPWM(servo, 0, current);
    delay(1000);
}

void loop() {
    if (digitalRead(PIN_BTN_HIGHER) && digitalRead(PIN_BTN_LOWER)) {
        if (step != 1)
            step = 1;
        else
            step = 10;
        Serial.printf("Step size: %u\n", step);
        delay(200);
    } else if (digitalRead(PIN_BTN_HIGHER)) {
        current += step;
        Serial.println(current);
        servoController.setPWM(servo, 0, current);
    } else if (digitalRead(PIN_BTN_LOWER)) {
        current -= step;
        Serial.println(current);
        servoController.setPWM(servo, 0, current);
    }

    delay(80);
}
