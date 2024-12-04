#include "firmware/wheel.hpp"

# define M_PI           3.14159265358979323846  /* pi */


Wheel::Wheel(int pin1, int pin2, int pwm, float wheel_radius, std::string side)
: pin1_(pin1), pin2_(pin2), pin_pwm_(pwm), wheel_side_(side), wheel_radius_(wheel_radius) 
{}

Wheel::Wheel(){}


Wheel::~Wheel(){}


void Wheel::SetModes(int pi)
{
    set_mode(pi, pin1_, PI_OUTPUT);
    set_mode(pi, pin2_, PI_OUTPUT);
    set_PWM_range(pi, pin_pwm_, 250); // sets the range of pwm to 0-250
}

std::string Wheel::GetSide()
{
    return wheel_side_;
}

void Wheel::Write(int pi, float velocity)
{   
    // velocity is in m/s
    // can be converted to pwm as :

    // rpm = revolute / minute => rpm*(1/60)*2*pi*wheel_radius(m/revolution) = velocity
    // => pwm = rpm = velocity * 60 / (2*pi*wheel_radius)
    
    // the maximum motor speed is :
    // Vmax = 250*(1/60)*2*pi*0.034(wheel radius) = 0.89 (lets say 0.8 m/s)

    float pwm = velocity * 60 / (2*M_PI*wheel_radius_); 
    // gives necessary pwm for given velocity

    if (velocity > 0.0)
    {
        gpio_write(pi,pin1_, 1);  // Set direction forward
        gpio_write(pi,pin2_, 0);
        set_PWM_dutycycle(pi,pin_pwm_, pwm);  // Set speed with PWM (range 0-255)
    }
    else if (velocity < 0.0)
    {
        gpio_write(pi,pin1_, 0);  // Set direction backward
        gpio_write(pi,pin2_, 1);
        set_PWM_dutycycle(pi,pin_pwm_, pwm);  // Set speed with PWM (range 0-255)
    }
    else
    {
        gpio_write(pi,pin1_, 0);
        gpio_write(pi,pin2_, 0);
        set_PWM_dutycycle(pi,pin_pwm_, 0);  // Set speed with PWM (range 0-255)
    }
}