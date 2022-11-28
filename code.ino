// ESP32_l298N_+2_servo_bluetooth

#include "BluetoothSerial.h"
#include "Servo.h"

#define max_angle_vert 180
#define min_angle_vert 0
#define max_angle_hori 180
#define min_angle_hori 0

#define motor1Channel 0
#define motor2Channel 1
#define vertical_mov_servo_pin 13 // servo1
#define horizontal_mov_servo_pin 23 // servo2


BluetoothSerial SerialBT;

const int freq = 20000;
const int resolution = 8; // 8 bit resolution so 256 values


int angleud = 0;
int anglelr = 0;
Servo servo_vertical;
Servo servo_horizontal;

#define motor1E 34
#define motor1A 27
#define motor1B 26
#define motor2A 18
#define motor2B 21
#define motor2E 33

char temp = 'S';
byte speed = 255;
bool stop_mode = 0; // 0 is immediate stop
byte operating_mode = 0; // 0 means Robot Movement Mode
// 1 means Gripper up Down Mode



void setup() {
  //Serial.begin(115200);
  SerialBT.begin("TEAM_NAME"); //Bluetooth device name
  //Serial.println("The device started, now you can pair it with bluetooth!");

  pinMode(motor1A, OUTPUT);
  pinMode(motor1B, OUTPUT);
  pinMode(motor2A, OUTPUT);
  pinMode(motor2B, OUTPUT);

  ledcSetup(motor1Channel, freq, resolution);
  ledcSetup(motor2Channel, freq, resolution);
  ledcAttachPin(motor1E, motor1Channel);
  ledcAttachPin(motor2E, motor2Channel);

  servo_vertical.attach(vertical_mov_servo_pin);
  servo_horizontal.attach(horizontal_mov_servo_pin);
  servo_vertical.write(0); // default to 0 degrees
  servo_horizontal.write(0); // default to 0 degrees

}

void down()
{
  if (angleud <= min_angle_vert)
  {
    angleud = min_angle_vert;
  }
  else
  {
    angleud = angleud - 1;
    servo_vertical.write(angleud);
    //  delay(5);
  }
}

void up()
{
  if (angleud >= max_angle_vert)
  {
    angleud = max_angle_vert;
  }
  else
  {
    angleud = angleud + 1;
    servo_vertical.write(angleud);
    //  delay(5);
  }
}

void open_gripper()
{
  if (anglelr <= min_angle_hori)
  {
    anglelr = min_angle_hori;
  }
  else
  {
    anglelr = anglelr - 1;
    servo_horizontal.write(anglelr);
    //  delay(5);
  }
}

void close_gripper()
{
  if (anglelr >= max_angle_hori)
  {
    anglelr = max_angle_hori;
  }
  else
  {
    anglelr = anglelr + 1;
    servo_horizontal.write(anglelr);
    //  delay(5);
  }
}

void forward()
{
  ledcWrite(motor1Channel, speed);
  ledcWrite(motor2Channel, speed);
  digitalWrite(motor1A, HIGH);
  digitalWrite(motor1B, LOW);
  digitalWrite(motor2A, HIGH);
  digitalWrite(motor2B, LOW);
}

void backward()
{
  ledcWrite(motor1Channel, speed);
  ledcWrite(motor2Channel, speed);
  digitalWrite(motor1A, LOW);
  digitalWrite(motor1B, HIGH);
  digitalWrite(motor2A, LOW);
  digitalWrite(motor2B, HIGH);
}

void rotate_left()
{
  ledcWrite(motor1Channel, speed);
  ledcWrite(motor2Channel, speed);
  digitalWrite(motor1A, LOW);
  digitalWrite(motor1B, HIGH);
  digitalWrite(motor2A, HIGH);
  digitalWrite(motor2B, LOW);
}

void rotate_right()
{
  ledcWrite(motor1Channel, speed);
  ledcWrite(motor2Channel, speed);
  digitalWrite(motor1A, HIGH);
  digitalWrite(motor1B, LOW);
  digitalWrite(motor2A, LOW);
  digitalWrite(motor2B, HIGH);
}
void forward_left_turn()
{
  if (stop_mode)
  {
    ledcWrite(motor1Channel, 0);
  }else{
    ledcWrite(motor1Channel, 255);
  }
  ledcWrite(motor2Channel, speed);
  digitalWrite(motor1A, LOW);
  digitalWrite(motor1B, LOW);
  digitalWrite(motor2A, HIGH);
  digitalWrite(motor2B, LOW);
}
void backward_left_turn()
{
  if (stop_mode)
  {
    ledcWrite(motor1Channel, 0);
  }else{
    ledcWrite(motor1Channel, 255);
  }
  ledcWrite(motor2Channel, speed);
  digitalWrite(motor1A, LOW);
  digitalWrite(motor1B, LOW);
  digitalWrite(motor2A, LOW);
  digitalWrite(motor2B, HIGH);
}
void forward_right_turn()
{
  ledcWrite(motor1Channel, speed);
  if (stop_mode)
  {
    ledcWrite(motor2Channel, 0);
  }else{
    ledcWrite(motor2Channel, 255);
  }
  digitalWrite(motor1A, HIGH);
  digitalWrite(motor1B, LOW);
  digitalWrite(motor2A, LOW);
  digitalWrite(motor2B, LOW);
}
void backward_right_turn()
{
  if (stop_mode)
  {
    ledcWrite(motor2Channel, 0);
  }else{
    ledcWrite(motor2Channel, 255);
  }
  ledcWrite(motor1Channel, speed);
  digitalWrite(motor1A, LOW);
  digitalWrite(motor1B, HIGH);
  digitalWrite(motor2A, LOW);
  digitalWrite(motor2B, LOW);
}
void stop()
{
  if (stop_mode)
  { // non immediate stop mode
    ledcWrite(motor1Channel, 0);
    ledcWrite(motor2Channel, 0);
  }else{
    ledcWrite(motor1Channel, 255);
    ledcWrite(motor2Channel, 255);
  }
  digitalWrite(motor1A, LOW);
  digitalWrite(motor1B, LOW);
  digitalWrite(motor2A, LOW);
  digitalWrite(motor2B, LOW);
}
void loop() {
  if (!SerialBT.hasClient())
  {
    digitalWrite(motor1A, LOW);
    digitalWrite(motor1B, LOW);
    digitalWrite(motor2A, LOW);
    digitalWrite(motor2B, LOW);
  }

  if (SerialBT.available()) {
    temp = SerialBT.read();

    switch (temp)
    {
      case 'W':
        operating_mode = 1;
        break;

      case 'w':
        operating_mode = 0;
        break;

      case 'F':
        if (operating_mode == 0)
          forward();
        else if (operating_mode == 1)
          up();
        break;

      case 'B':
        if (operating_mode == 0)
          backward();
        else if (operating_mode == 1)
          down();
        break;

      case 'L':
        if (operating_mode == 0)
          rotate_left();
        else if (operating_mode == 1)
          open_gripper();
        break;

      case 'R':
        if (operating_mode == 0)
          rotate_right();
        else if (operating_mode == 1)
          close_gripper();
        break;

      case 'G':
        forward_left_turn();
        break;

      case 'I':
        forward_right_turn();
        break;

      case 'H':
        backward_left_turn();
        break;

      case 'J':
        backward_right_turn();
        break;

      case 'S':
        stop();
        break;

      case '0':
        speed = (0.0 / 10) * 255;
        break;
      case '1':
        speed = (5.0 / 10) * 255;
        break;
      case '2':
        speed = (5.5 / 10) * 255;
        break;
      case '3':
        speed = (6.0 / 10) * 255;
        break;
      case '4':
        speed = (6.5 / 10) * 255;
        break;
      case '5':
        speed = (7.0 / 10) * 255;
        break;
      case '6':
        speed = (7.5 / 10) * 255;
        break;
      case '7':
        speed = (8.0 / 10) * 255;
        break;
      case '8':
        speed = (8.5 / 10) * 255;
        break;
      case '9':
        speed = (9.0 / 10) * 255;
        break;
      case 'q':

        speed = (10.0 / 10) * 255;
        break;

      case 'X':
        stop_mode = 1;
        break;
      case 'x':
        stop_mode = 0;
        break;
    }



  }
}
