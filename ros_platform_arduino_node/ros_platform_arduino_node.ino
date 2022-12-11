#
#include <ros.h>
#include <std_msgs/Int32.h>
#include <std_msgs/String.h>
#include <sensor_msgs/Range.h>

// -- Zmienne --//

#define pwmA 12
#define inA1 11
#define inA2 10
#define pwmB 7
#define inB1 9
#define inB2 8
#define encoderA1 2
#define encoderA2 24
#define encoderB1 3
#define encoderB2 28

int l_speed = 0, r_speed = 0, pwm_correction = 8, pos_A = 0, pos_B = 0;
String l_direction, r_direction;

// ROS inicjalizacja
ros::NodeHandle  nh;
std_msgs::Int32 posA_msg;
std_msgs::Int32 posB_msg;


void left_speedMsgCallback(const std_msgs::Int32 speed_msg)
{
  int speed = speed_msg.data;
  l_speed = abs(speed);
  if (speed > 0) {
    l_direction = "front";
  }
  else if (speed < 0) {
    l_direction = "back";
  }
  else {
    l_direction = "stop";
  }
}

void right_speedMsgCallback(const std_msgs::Int32 speed_msg)
{
  int speed = speed_msg.data;
  r_speed = abs(speed);

  if (speed > 0) {
    r_direction = "front";
  }
  else if (speed < 0) {
    r_direction = "back";
  }
  else {
    r_direction = "stop";
  }
}

ros::Subscriber<std_msgs::Int32> arduino_sub_l_speed("motor/left_speed", left_speedMsgCallback );
ros::Subscriber<std_msgs::Int32> arduino_sub_r_speed("motor/right_speed", right_speedMsgCallback );

ros::Publisher arduino_pub_wheelA("arduino_msgs/posA", &posA_msg);
ros::Publisher arduino_pub_wheelB("arduino_msgs/posB", &posB_msg);
//ros::Publisher pub_range( "/ultrasound", &sonar1_msg);

void setup()
{
  //nh.getHardware()->setPort(&Serial1);
  //nh.getHardware()->setBaud(57600);

  //Serial.begin(57600);
  pinMode(encoderA1, INPUT);
  pinMode(encoderA2, INPUT);
  pinMode(encoderB1, INPUT);
  pinMode(encoderB2, INPUT);

  pinMode(pwmA, OUTPUT);
  pinMode(inA1, OUTPUT);
  pinMode(inA2, OUTPUT);

  pinMode(pwmB, OUTPUT);
  pinMode(inB1, OUTPUT);
  pinMode(inB2, OUTPUT);

  attachInterrupt(digitalPinToInterrupt(encoderA1), readEncoderA, RISING);
  attachInterrupt(digitalPinToInterrupt(encoderB1), readEncoderB, RISING);


  nh.initNode();

  nh.advertise(arduino_pub_wheelA);
  nh.advertise(arduino_pub_wheelB);
  nh.subscribe(arduino_sub_l_speed);
  nh.subscribe(arduino_sub_r_speed);
}

void loop()
{

  moveRobot(r_direction, r_speed, l_direction, l_speed);

  posA_msg.data = (pos_A);
  posB_msg.data = (pos_B);

  //sonar1_msg.range = pos_B;
  arduino_pub_wheelA.publish(&posA_msg);
  arduino_pub_wheelB.publish(&posB_msg);

  pos_A = 0;
  pos_B = 0;
  nh.spinOnce();
  delay(10);
}


void readEncoderA()
{ int a = digitalRead(encoderA2);
  if (a > 0) {
    pos_A++;
  } else {
    pos_A--;
  }
}
void readEncoderB()
{ int b = digitalRead(encoderB2);
  if (b > 0) {
    pos_B++;
  } else {
    pos_B--;
  }
}

void moveRobot(String r_direction, int r_speed, String l_direction, int l_speed)
{

  if (r_direction == "front" && l_direction == "front")
  {
    digitalWrite(inA1, HIGH);
    digitalWrite(inA2, LOW);
    analogWrite(pwmA, r_speed);
    digitalWrite(inB1, HIGH);
    digitalWrite(inB2, LOW);
    analogWrite(pwmB, l_speed+pwm_correction);

  } else if (r_direction == "back" && l_direction == "back") {
    digitalWrite(inA1, LOW);
    digitalWrite(inA2, HIGH);
    analogWrite(pwmA, r_speed);
    digitalWrite(inB1, LOW);
    digitalWrite(inB2, HIGH);
    analogWrite(pwmB, l_speed+pwm_correction);
  } else if (r_direction == "back" && l_direction == "front") {
    digitalWrite(inA1, HIGH);
    digitalWrite(inA2, LOW);
    analogWrite(pwmA, r_speed);
    digitalWrite(inB1, LOW);
    digitalWrite(inB2, HIGH);
    analogWrite(pwmB, l_speed+pwm_correction);
  } else if (r_direction == "front" && l_direction == "back") {
    digitalWrite(inA1, LOW);
    digitalWrite(inA2, HIGH);
    analogWrite(pwmA, r_speed);
    digitalWrite(inB1, HIGH);
    digitalWrite(inB2, LOW);
    analogWrite(pwmB, l_speed+pwm_correction);

  } else {
    digitalWrite(inA1, LOW);
    digitalWrite(inA2, LOW);
    analogWrite(pwmA, 0);
    digitalWrite(inB1, LOW);
    digitalWrite(inB2, LOW);
    analogWrite(pwmB, 0);

  }
}
