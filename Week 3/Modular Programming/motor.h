#ifndef MOTOR_H
#define MOTOR_H
#include <Arduino.h>

class Motor {
  public:
    Motor(int pin1, int pin2);

    void forward(int pwm);
    void backward(int pwm);
    void stop();

  private:
    int _pin1;
    int _pin2;
};

#endif