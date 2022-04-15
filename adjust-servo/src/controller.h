#pragma once

#include "utils.h"
#include <Adafruit_PWMServoDriver.h>
#include <Arduino.h>

class ServoController {
  public:
    uint8_t pin;
    uint8_t posFeedbackPin;
    PWMTicks ticks;
    PWMTicks stepSize;
    Adafruit_PWMServoDriver* driver;

    ServoController(uint8_t pin, uint8_t posFeedbackPin, Adafruit_PWMServoDriver* driver) {
        this->pin = pin;
        this->posFeedbackPin = posFeedbackPin;

        this->ticks = 0;
        this->driver = driver;
        this->reset();
    }

    void init() { pinMode(this->posFeedbackPin, INPUT); }

    void set(PWMTicks ticks) {
        this->ticks = ticks;
        this->driver->setPin(0, this->ticks);
    }

    void inc() {
        this->ticks += this->stepSize;
        this->driver->setPin(this->pin, this->ticks);
    }

    void dec() {
        this->ticks -= this->stepSize;
        this->driver->setPin(this->pin, this->ticks);
    }

    void toggleStepSize() {
        if (this->stepSize != 1)
            this->stepSize = 1;
        else
            this->stepSize = 10;
    }

    uint16_t getPosFeedback() { return analogRead(this->posFeedbackPin); }

    void reset() { this->stepSize = 1; }
};
