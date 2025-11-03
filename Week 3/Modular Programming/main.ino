#include "motor.h"

Motor leftMotor(5, 6);
Motor rightMotor(9, 10);

void setup() {
    Serial.begin(9600); 
}

void loop() {

    leftMotor.forward(200);
    rightMotor.forward(200);
    delay(2000);

    leftMotor.stop();
    rightMotor.stop();
    delay(1000);

    leftMotor.backward(150);
    rightMotor.backward(150);
    delay(2000);

    leftMotor.stop();
    rightMotor.stop();
    delay(1000);

}