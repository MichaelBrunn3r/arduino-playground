#include <Adafruit_PWMServoDriver.h>
#include <Arduino.h>
#include <Wire.h>

// PWM frequency the servo expects
#define SERVO_FREQ 50

// Min pulse length of the servo (i.e. the 0 positon)
#define MIN_PULSE_LENGTH 150

// Max pulse length of the servo (i.e. the 180 position)
#define MAX_PULSE_LENGTH 600

Adafruit_PWMServoDriver servoController = Adafruit_PWMServoDriver(0x40);

void initSerial() {
    Serial.begin(115200);
    while (!Serial && !Serial.available()) {
    }
    Serial.println();
}

void initServoController() {
    servoController.begin();
    servoController.setOscillatorFrequency(27000000);
    servoController.setPWMFreq(SERVO_FREQ);
}

void setup() {
    initSerial();
    initServoController();
    delay(100);

    // Test min pulse length
    // Decrease value until servo doen't turn any furthe and makes noise
    servoController.setPWM(0, 0, MIN_PULSE_LENGTH);

    // Test max pulse length
    // Increase until servo does not turn any further
    // servoController.setPWM(0, 0, MAX_PULSE_LENGTH);
}

void loop() {}
