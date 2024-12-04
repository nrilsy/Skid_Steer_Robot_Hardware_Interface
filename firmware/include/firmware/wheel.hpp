#ifndef WHEELS
#define WHEELS

#include <vector>
#include <string>
#include <pigpiod_if2.h>

// Wheels are directly connected to the hardware and responsible for motor (and wheel)
// motions.

// Hardware interface will sent velocity values in m/s and Wheel::Write function
// supposed to convert m/s to the necessary pwm value.
// The hardware interface will calculate the velocity of the wheels by using:
// V = Vx +- W * (L/2) where L is wheel separation
// Write function will convert V to necessarry pwm (and since pwm = rpm) and rpm


// the motor has a maximum value of 250 rpm
// pwm range is 0-250 (assumed linear corellation between pwm and rpm in this case)
// when pwm = 0 -> rpm = 0 and when pwm = 250 -> rpm = 250

// normally, usage of pigpio library needs a int pi value which indicates
// the software is ready to write.

class Wheel
{
public:
    Wheel();
    Wheel(int pin1, int pin2, int pwm, float wheel_radius_, std::string side);
    ~Wheel();
    void Write(int pi, float velocity); // velocity is in m/s
    void SetModes(int pi);
    std::string GetSide();
private:
    int pin1_;
    int pin2_;
    int pin_pwm_;
    std::string wheel_side_;
    float wheel_radius_; // in m
};


#endif