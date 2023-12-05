#include "Motor.h"
#include <PID_v1.h>

#define BRAKE 0
#define CW 1
#define CCW 2

//MOTOR 1
#define MOTOR_A1_PIN 7
#define MOTOR_B1_PIN 8
#define PWM_MOTOR_1 5
#define EN_PIN_1 A0

//MOTOR 2
#define MOTOR_A2_PIN 4
#define MOTOR_B2_PIN 9
#define PWM_MOTOR_2 6
#define EN_PIN_2 A1


#define L_encoderPinA 18  //do18
#define L_encoderPinB 19  //xanh19

#define R_encoderPinA 20  //do
#define R_encoderPinB 21  //xanh

#define echo 52
#define trig 50

#define LOOPTIME 10

#define red 48
#define yellow 46
#define green 44

#define lidar1 53
#define lidar2 51
#define lidar3 49
#define lidar4 47
#define lidar5 45
#define lidar6 43

Motor motorleft(MOTOR_A1_PIN, MOTOR_B1_PIN, L_encoderPinA, L_encoderPinB, EN_PIN_1, PWM_MOTOR_1);
Motor motorright(MOTOR_A2_PIN, MOTOR_B2_PIN, R_encoderPinA, R_encoderPinB, EN_PIN_2, PWM_MOTOR_2);

double left_kp = 0.08, left_ki = 0.0001, left_kd = 0.008;  // modify for optimal performance
double right_kp = 0.103, right_ki = 0.0001, right_kd = 0.008;

double kp = 100.0, ki = 10.0, kd = 10.0;

double right_input = 0, right_output = 0, right_setpoint = 0;
PID rightPID(&right_input, &right_output, &right_setpoint, right_kp, right_ki, right_kd, DIRECT);

double left_input = 0, left_output = 0, left_setpoint = 0;
PID leftPID(&left_input, &left_output, &left_setpoint, left_kp, left_ki, left_kd, DIRECT);

double input = 0, output = 0, setpoint = -80.0;
PID distancePID(&input, &output, &setpoint, kp, ki, kd, DIRECT);


double CounterL = 0;  // encoder 1
double CounterR = 0;  // encoder 2

unsigned long timecho = 100;
unsigned long thoigian, hientai;
int rpmL = 0;
int rpmR = 0;

float V_right = 0;
float V_left = 0;

unsigned short usMotor_Status = BRAKE;

float khoangCach;
long thoiGianDo;

float demandz = 0;
float demandx = 0;

int statusL;
int statusR;

int statusLidarL;
int statusLidarR;

bool A, B, C, D, E, F;

bool sensor = false;

void setup() {
  Serial.begin(9600);

  rightPID.SetMode(AUTOMATIC);
  rightPID.SetSampleTime(100);
  rightPID.SetOutputLimits(-255, 255);

  leftPID.SetMode(AUTOMATIC);
  leftPID.SetSampleTime(100);
  leftPID.SetOutputLimits(-255, 255);

  distancePID.SetMode(AUTOMATIC);
  distancePID.SetSampleTime(100);
  distancePID.SetOutputLimits(-11880.0, 11880.0);

  attachInterrupt(digitalPinToInterrupt(motorleft.encoder_a), change_left_a, CHANGE);
  attachInterrupt(digitalPinToInterrupt(motorleft.encoder_b), change_left_b, CHANGE);
  attachInterrupt(digitalPinToInterrupt(motorright.encoder_a), change_right_a, CHANGE);
  attachInterrupt(digitalPinToInterrupt(motorright.encoder_b), change_right_b, CHANGE);

  digitalWrite(motorleft.en_pin, HIGH);
  digitalWrite(motorright.en_pin, HIGH);

  pinMode(trig, OUTPUT);
  pinMode(echo, INPUT);
  pinMode(red, OUTPUT);
  pinMode(yellow, OUTPUT);
  pinMode(green, OUTPUT);

  pinMode(lidar1, INPUT_PULLUP);
  pinMode(lidar2, INPUT_PULLUP);
  pinMode(lidar3, INPUT_PULLUP);
  pinMode(lidar4, INPUT_PULLUP);
  pinMode(lidar5, INPUT_PULLUP);
  pinMode(lidar6, INPUT_PULLUP);
  delay(100);
}


void sub() {
  char dataSerial[100] = "";
  while (Serial.available() != 0) {
    Serial.readBytesUntil('\n', dataSerial, sizeof(dataSerial));

    int intValue = atoi(dataSerial);
    demandx = intValue;
    //demandx = intValue;
    if (demandz < 0) {
      digitalWrite(green, HIGH);
      digitalWrite(red, LOW);
    } else {
      digitalWrite(green, LOW);
      digitalWrite(red, HIGH);
    }
    CounterL = 0;
    CounterR = 0;
  }
}

void loop() {
  sub();
  Forward();

  // Serial.print("demandx: ");
  // Serial.print(demandx);
  // Serial.print(" ");
  // Serial.print("khoangCach: ");
  // Serial.print(khoangCach);
  // Serial.print(" ");
  // Serial.print("counter L ");
  // Serial.print(CounterL);
  // Serial.print(" ");
  // Serial.print("counter R ");
  // Serial.println(CounterR);

  motorleft.rotate(statusL, left_output);
  motorright.rotate(statusR, right_output);

  // Serial.print("left_input: ");
  // Serial.print(left_input);
  // Serial.print(" ");
  // Serial.print("right_input: ");
  // Serial.print(right_input);
  // Serial.print(" ");
  // Serial.print("left_setpoint: ");
  // Serial.print(left_setpoint);
  // Serial.print(" ");
  // Serial.print("right_setpoint: ");
  // Serial.print(right_setpoint);
  // Serial.print(" ");
  // Serial.print("statusLidarR: ");
  // Serial.print(statusLidarR);
  // Serial.print(" ");
  // Serial.print("statusLidarL: ");
  // Serial.print(statusLidarL);
  // Serial.print(" ");
  // Serial.print("demandx: ");
  // Serial.print(demandx);
  // Serial.print(" ");
  // Serial.print("left_output: ");
  // Serial.print(left_output);
  // Serial.print(" ");
  // Serial.print("Right_output: ");
  // Serial.print(right_output);
  // Serial.print(" ");
  // Serial.print("Setpoint: ");
  // int Setpoint = 10000;
  // Serial.println(Setpoint);
  // Serial.print("CounterL: ");
  // Serial.println(CounterL);
  // Serial.print(" ");
  // Serial.print("CounterR: ");
  // Serial.print(CounterR);

  // Serial.print(" ");
  // Serial.print("rpmL: ");
  // Serial.print(rpmL);
  // Serial.print(" ");
  // Serial.print("rpmR: ");
  // Serial.print(rpmR);
  //  Serial.println();

  //demandx = 0;
  //demandz = 0;
  // int Setpoint = 10000;
  // Serial.print(demandx);  // Gửi giá trị Setpoint
  // Serial.print(" ");       // Gửi khoảng trắng hoặc dấu phân cách để phân biệt giữa các giá trị
  // Serial.print(CounterL);
  // Serial.print(" ");       // Gửi khoảng trắng hoặc dấu phân cách để phân biệt giữa các giá trị
  // Serial.println(CounterR);
}


void Forward() {

  thoigian = millis();
  if (thoigian - hientai >= timecho) {

    hientai = thoigian;

    rpmL = (CounterL / 11880.0) * 1000 / timecho * 60;

    V_left = float(CounterL / 11880.0 * 1000 / timecho) * float(0.085 * 3.14);
    rpmL = abs(rpmL);



    rpmR = (CounterR / 11880.0) * 1000 / timecho * 60;
    V_right = float(CounterR / 11880.0 * 1000 / timecho) * float(0.085 * 3.14);
    rpmR = abs(rpmR);


    left_setpoint = demandx + demandz * 537.9 / 2;   //+ Z * 42.447 / 2;  //Setting required speed as a mul/frac of 1 m/s
    right_setpoint = demandx - demandz * 537.9 / 2;  // - Z * 42.447 / 2;

    left_input = CounterL;  //Input to PID controller is the current difference
    right_input = CounterR;

    leftPID.Compute();
    rightPID.Compute();

    lidar();

    if (left_output < 0) {
      if (statusLidarL == 1) statusL = CCW;
      else statusL = CW;
    } else {
      if (statusLidarL == 1) statusL = CW;
      else statusL = BRAKE;
    }

    if (right_output < 0) {
      if (statusLidarR == 1) statusR = CCW;
      else statusR = CW;
    } else {
      if (statusLidarR == 1) statusR = CW;
      else statusR = BRAKE;
    }

    left_output = abs(left_output);
    right_output = abs(right_output);
  }
}

void lidar() {
  A = digitalRead(lidar1);
  B = digitalRead(lidar2);
  C = digitalRead(lidar3);
  D = digitalRead(lidar4);
  E = digitalRead(lidar5);
  F = digitalRead(lidar6);
  statusLidarL = logic_functionL(A, B, C, D, E, F);  // Gọi hàm logic_function với các giá trị đầu vào
  statusLidarR = logic_functionR(A, B, C, D, E, F);
}

int logic_functionL(bool A, bool B, bool C, bool D, bool E, bool F) {

  // Công thức logic CD + B'EF + A'DE + B'DE + DEF + A'BD'
  bool result = (C && D) || (!B && E && F) || (!A && D && E) || (!B && D && E) || (D && E && F) || (!A && B && D);
  return result;
}

int logic_functionR(bool A, bool B, bool C, bool D, bool E, bool F) {


  // Công thức logic CD + DEF' + BD'E' + BD'F' + ABE' + ABC
  bool result = (C && D) || (D && E && !F) || (B && !D && !E) || (B && !D && !F) || (A && B && !E) || (A && B && C);
  return result;
}



void distance() {

  digitalWrite(trig, LOW);
  delayMicroseconds(2);
  digitalWrite(trig, HIGH);
  delayMicroseconds(10);
  digitalWrite(trig, LOW);

  thoiGianDo = pulseIn(echo, HIGH);
  khoangCach = thoiGianDo * 0.034 / 2;
  float khoangCachFloat = (float)khoangCach;
  input = -khoangCachFloat;
  distancePID.Compute();
  demandx = output;

  // Gửi dữ liệu dưới dạng chuỗi
  //Serial.println(khoangCachFloat);
  delay(10);
}

// ************** encoder 1 *********************
void change_left_a() {

  // look for a low-to-high on channel A
  if (digitalRead(motorleft.encoder_a) == HIGH) {
    // check channel B to see which way encoder is turning
    if (digitalRead(motorleft.encoder_b) == LOW) {
      CounterL = CounterL + 1;  // CW
    } else {
      CounterL = CounterL - 1;  // CCW
    }
  } else  // must be a high-to-low edge on channel A
  {
    // check channel B to see which way encoder is turning
    if (digitalRead(motorleft.encoder_b) == HIGH) {
      CounterL = CounterL + 1;  // CW
    } else {
      CounterL = CounterL - 1;  // CCW
    }
  }
}

void change_left_b() {

  // look for a low-to-high on channel B
  if (digitalRead(motorleft.encoder_b) == HIGH) {
    // check channel A to see which way encoder is turning
    if (digitalRead(motorleft.encoder_a) == HIGH) {
      CounterL = CounterL + 1;  // CW
    } else {
      CounterL = CounterL - 1;  // CCW
    }
  }
  // Look for a high-to-low on channel B
  else {
    // check channel B to see which way encoder is turning
    if (digitalRead(motorleft.encoder_a) == LOW) {
      CounterL = CounterL + 1;  // CW
    } else {
      CounterL = CounterL - 1;  // CCW
    }
  }
}

// ************** encoder 2 *********************

void change_right_a() {

  // look for a low-to-high on channel A
  if (digitalRead(motorright.encoder_a) == HIGH) {
    // check channel B to see which way encoder is turning
    if (digitalRead(motorright.encoder_b) == LOW) {
      CounterR = CounterR - 1;  // CW
    } else {
      CounterR = CounterR + 1;  // CCW
    }
  } else  // must be a high-to-low edge on channel A
  {
    // check channel B to see which way encoder is turning
    if (digitalRead(motorright.encoder_b) == HIGH) {
      CounterR = CounterR - 1;  // CW
    } else {
      CounterR = CounterR + 1;  // CCW
    }
  }
}

void change_right_b() {

  // look for a low-to-high on channel B
  if (digitalRead(motorright.encoder_b) == HIGH) {
    // check channel A to see which way encoder is turning
    if (digitalRead(motorright.encoder_a) == HIGH) {
      CounterR = CounterR - 1;  // CW
    } else {
      CounterR = CounterR + 1;  // CCW
    }
  }
  // Look for a high-to-low on channel B
  else {
    // check channel B to see which way encoder is turning
    if (digitalRead(motorright.encoder_a) == LOW) {
      CounterR = CounterR - 1;  // CW
    } else {
      CounterR = CounterR + 1;  // CCW
    }
  }
}
