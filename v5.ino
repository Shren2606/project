//AGV Machine - Vinay Lanka

//Import Motor - Cytron SPG30E-30K
#include "Motor.h"
#include <ros.h>
#include <geometry_msgs/Twist.h>
#include <PID_v1.h>
#include <geometry_msgs/Vector3Stamped.h>
#include <ros/time.h>
#include <std_msgs/String.h>


ros::NodeHandle nh;

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



void cmd_vel_cb(const geometry_msgs::Twist& twist) {
  demandx = twist.linear.x;
  demandz = twist.angular.z;
}

ros::Subscriber<geometry_msgs::Twist> sub("cmd_vel", cmd_vel_cb);
geometry_msgs::Vector3Stamped speed_msg;          //create a "speed_msg" ROS message
ros::Publisher speed_pub("Arduino/speed", &speed_msg);  //create a publisher to ROS topic "speed" using the "speed_msg" type

std_msgs::String str_msg;
ros::Publisher chatter("Arduino/chatter", &str_msg);

char hello[13] = "hello world!";

double speed_act_left = 0;   //Actual speed for left wheel in m/s
double speed_act_right = 0;  //Command speed for left wheel in m/s

void setup() {
  nh.initNode();
  nh.subscribe(sub);
  nh.advertise(speed_pub);
  nh.advertise(chatter);
  Serial.begin(9600);  //prepare to publish speed in ROS topic
                            //  Serial.begin(115200);



  rightPID.SetMode(AUTOMATIC);
  rightPID.SetSampleTime(100);
  rightPID.SetOutputLimits(-255, 255);

  leftPID.SetMode(AUTOMATIC);
  leftPID.SetSampleTime(100);
  leftPID.SetOutputLimits(-255, 255);


  attachInterrupt(digitalPinToInterrupt(motorleft.encoder_a), change_left_a, CHANGE);
  attachInterrupt(digitalPinToInterrupt(motorleft.encoder_b), change_left_b, CHANGE);
  attachInterrupt(digitalPinToInterrupt(motorright.encoder_a), change_right_a, CHANGE);
  attachInterrupt(digitalPinToInterrupt(motorright.encoder_b), change_right_b, CHANGE);

  digitalWrite(motorleft.en_pin, HIGH);
  digitalWrite(motorright.en_pin, HIGH);
}

void loop() {
  Forward();
  publishSpeed(LOOPTIME);
  str_msg.data = hello;
  chatter.publish(&str_msg);
  nh.spinOnce();
}


void Forward() {

  thoigian = millis();
  if (thoigian - hientai >= LOOPTIME) {

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



    if (left_output < 0) {
      statusL = CCW;
    } else {
      statusL = CW;
    }

    if (right_output < 0) {
      statusR = CCW;
    } else {
      statusR = CW;
    }

    left_output = abs(left_output);
    right_output = abs(right_output);
    motorleft.rotate(statusL, left_output);
    motorright.rotate(statusR, right_output);
  }
}


//Publish function for odometry, uses a vector type message to send the data (message type is not meant for that but that's easier than creating a specific message type)
void publishSpeed(double time) {
  speed_msg.header.stamp = nh.now();  //timestamp for odometry data
  speed_msg.vector.x = 1000;          //left wheel speed (in m/s)
  speed_msg.vector.y = 2000;          //right wheel speed (in m/s)
  speed_msg.vector.z = time / 1000;   //looptime, should be the same as specified in LOOPTIME (in s)
  speed_pub.publish(&speed_msg);
  //  nh.loginfo("Publishing odometry");
}


// ************** encoders interrupts **************

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
