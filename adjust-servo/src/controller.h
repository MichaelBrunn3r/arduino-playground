#pragma once

#include "utils.h"
#include <Adafruit_PWMServoDriver.h>
#include <Arduino.h>
#include <EMA.h>

class ServoController {
  public:
    uint8_t pin;
    uint8_t posFeedbackPin;
    PWMTicks ticks = 0;
    PWMTicks stepSize = 1;
    Adafruit_PWMServoDriver* driver;
    EMA<8, uint32_t> position = EMA<8, uint32_t>(0);

    ServoController(uint8_t pin, uint8_t posFeedbackPin, Adafruit_PWMServoDriver* driver) {
        this->pin = pin;
        this->posFeedbackPin = posFeedbackPin;
        this->driver = driver;
    }

    void init() { pinMode(this->posFeedbackPin, INPUT); }

    void off() { this->driver->setPin(0, 0); }

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

    uint16_t getPosFeedback() { return this->position(analogRead(this->posFeedbackPin)); }

    void reset() { this->stepSize = 1; }
};
