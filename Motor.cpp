#include "Arduino.h"
#include "Motor.h"


Motor::Motor(int plus, int minus, int encoder_a, int encoder_b, int en_pin, int pwm_pin) {
  pinMode(plus, OUTPUT);
  pinMode(minus, OUTPUT);

  pinMode(encoder_a, INPUT_PULLUP);
  pinMode(encoder_b, INPUT_PULLUP);

  pinMode(en_pin, OUTPUT);

  pinMode(pwm_pin, OUTPUT);

  Motor::plus = plus;
  Motor::minus = minus;
  Motor::encoder_a = encoder_a;
  Motor::encoder_b = encoder_b;
  Motor::en_pin = en_pin;
  Motor::pwm_pin = pwm_pin;
}

void Motor::rotate(int direct, int pwm) {
  //digitalWrite(en_pin, HIGH);

  if (direct == 1) {
    digitalWrite(plus, HIGH);
    digitalWrite(minus, LOW);
  } else if (direct == 2) {
    digitalWrite(plus, LOW);
    digitalWrite(minus, HIGH);
  } else {
    digitalWrite(plus, LOW);
    digitalWrite(minus, LOW);
  }

  analogWrite(pwm_pin, pwm);
}