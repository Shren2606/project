/*
  Motor.h - Library for working with the Cytron SPG30E-30K.
  Created by Vinay Lanka, January 27, 2021.
*/
#ifndef Motor_h
#define Motor_h

#include "Arduino.h"

class Motor {
public:
  //Constructor - Plus and Minus are the Motor output / en_a and en_b are the encoder inputs
  Motor(int plus, int minus, int encoder_a, int encoder_b, int en_pin, int pwm_pin);
  //Spin the motor with a percentage value
  void rotate(int direct, int pwm);
  //Motor Outputs - plus is one direction and minus is the other
  int plus;
  int minus;
  //Encoder Inputs
  int encoder_a;
  int encoder_b;
  //pwm enable
  int en_pin;
  int pwm_pin;
};

#endif