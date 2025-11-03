#include "motor.h"


Motor::Motor(int pin1, int pin2) {
  _pin1 = pin1;
  _pin2 = pin2;

  pinMode(_pin1, OUTPUT);
  pinMode(_pin2, OUTPUT);
}

void Motor::forward(int pwm) {
  analogWrite(_pin1, pwm);
  digitalWrite(_pin2, LOW);
}

void Motor::backward(int pwm) {
  digitalWrite(_pin1, LOW);
  analogWrite(_pin2, pwm);
}

void Motor::stop() {
  digitalWrite(_pin1, LOW);
  digitalWrite(_pin2, LOW);
}