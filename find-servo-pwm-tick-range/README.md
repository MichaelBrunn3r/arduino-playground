<h1 align=center>Find PWM tick range of servo</h1>

A tool to find the min/max PWM pulse-length/ticks of a Servo.

## Usage

-   Press left/right button to lower/increase pulse length
-   Press both left and right button to toggle step size between 1 and 10

## Parts

### PCA9685 servo driver:

-   12 bit PWM
-   4096 (2^12) PWM ticks

### MG 996R servo:

-   50 Hz
-   20ms (1s / 50Hz) PWM Period
-   1-2ms Duty Cycle

|                               | -90˚ | 0˚    | 90˚ |
| ----------------------------- | ---- | ----- | --- |
| Duty Cycle / PWM pulse length | 1ms  | 1.5ms | 2ms |
| PWM Ticks (out of 4096)       | 204  | 307   | 409 |

## Wiring

<div align=center><img src="docs/Sketch.png"/></div>
