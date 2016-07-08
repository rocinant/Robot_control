#include <PololuWheelEncoders2.h>
#include <Servo.h>

#include <Wire.h>
#include <Max3421e.h>
#include <Usb.h>
#include <AndroidAccessory.h>

//AndroidAccessory acc("Manufacturer", "Model", "Description", "Version", "URI", "Serial");
AndroidAccessory acc("Google, Inc.",            // Manufacturer
                     "VisualRobot",                 // Model
                     "VisualRobot Arduino Board",   // Description
                     "1.0",                     // Version
                     "http://www.android.com",  // URI
                     "0000000012345678");       // Serial

byte RcvMsg[9];
byte SntMsg[5];

Servo rightSideMotors;//for Analog Input mode
Servo leftSideMotors; //for Analog Input mode

#define STBY  22
// 电机1控制脚定义 
#define In1 23
#define In2  24
// 电机2控制脚定义 
#define In3 25
#define In4  26
const int RIGHT_MOTOR_PIN = 7;//PWMA
const int LEFT_MOTOR_PIN = 6;//PWMB
PololuWheelEncoders encoders;

#define  ReceivedMaxSpeed  100
#define  TurnSpeedFactor1 2 //to control the moving speed when you know the turing speed, should be value between 1 and 2
#define  TurnSpeedFactor2 1.5f //to control the moving speed when you know the turing speed, should be value between 1 and 2 2*TurnSpeedFactor2-TurnSpeedFactor1 should be >0
const int WHEEL_CIR = 36;  //wheel circumference in cm
const int BAUD_RATE = 9600;

Servo BaseRotate;
Servo CameraHor;
Servo CameraVert;
const int BASE_ROTATE_PIN = 3; 
const int CAM_HOR_PIN = 4;
const int CAM_VERT_PIN = 5;

const int FRONT_SONAR = 8;
const int REAR_SONAR = 9;
const int SONAR_SAMPLES = 10;//for average sonar reading
const int SONAR_TAKE_ACTION_THRESHOLD = 40;//sonar threshold to stop
bool HeedSonar = true;

unsigned long lastMilli=0;
#define count_per_rev      64                  // encoder produces 64 pulses per output shaft revolution
#define gear_ratio 50
#define LOOPTIME        10                     // PID loop time
int LeftSideSpeed = 0; //left wheel's real speed RPM
int RightSideSpeed = 0; //right wheel's real speed RPM
#define  LeftMotorMaxSpeed       200
#define  RightMotorMaxSpeed       200
int PWM_val_a = 0;                             
int PWM_val_b = 0;                              
int PWM_val_d = 0;                            
float Kp_a =   .4f;                                // PID proportional control Gain
float Kd_a =    0.4f;                                // PID Derivitave control gain
float Ki_a =    0.01f;                                // PID Integration control gain
float Kp_b =   .4;                                // PID proportional control Gain
float Kd_b =    0.4f;                                // PID Derivitave control gain
float Ki_b =    0.01f;                                // PID Integration control gain
float Kp_d =   .5f;                                // PID proportional control Gain
float Kd_d =    0.2f;                                // PID Derivitave control gain
float Ki_d =    0.01f;                                // PID Integration control gain
volatile bool _ForwardFlag = false;
volatile bool _BackwardFlag = false;
int GlobalSpeed=0;

  byte EvtFrontSonar = 1;
  byte EvtRearSonar = 2;
  
  #define CmdMoveForward 1
  #define CmdMoveBackward 2
  #define CmdSpinLeft  3
  #define CmdSpinRight  4
  #define CmdStop  5
  #define CmdMoveCameraVert  6
  #define CmdMoveCameraHor  7

void setup() {
  Serial.begin(BAUD_RATE);

  pinMode(BASE_ROTATE_PIN, OUTPUT);
  pinMode(CAM_HOR_PIN, OUTPUT);
  pinMode(CAM_VERT_PIN, OUTPUT);
  BaseRotate.attach(BASE_ROTATE_PIN);
  CameraHor.attach(CAM_HOR_PIN);
  CameraVert.attach(CAM_VERT_PIN);
  BaseRotate.write(90);
  CameraHor.write(90);
  CameraVert.write(90);

  pinMode(RIGHT_MOTOR_PIN, OUTPUT);
  pinMode(LEFT_MOTOR_PIN, OUTPUT);
  pinMode(STBY, OUTPUT);
  pinMode(In1,OUTPUT); 
  pinMode(In2,OUTPUT);
  pinMode(In3,OUTPUT);
  pinMode(In4,OUTPUT);
//  rightSideMotors.attach(RIGHT_MOTOR_PIN);
//  leftSideMotors.attach(LEFT_MOTOR_PIN);
  
  pinMode(FRONT_SONAR, INPUT);
  pinMode(REAR_SONAR, INPUT);
  // enable the internal pullups
  digitalWrite(FRONT_SONAR, HIGH);
  digitalWrite(REAR_SONAR, HIGH);
  
  encoders.init(A8, A9,A10, A11);//look into encoders' reading

  acc.powerOn();
  stop();

  lastMilli = millis();
  MoveForward(100);
//  MoveBackward(speed);
//  _ForwardFlag = true;
//  GlobalSpeed=50;      
}

void loop() {
  
    if((millis()-lastMilli) >= LOOPTIME)   {                                    // enter PID correction
    lastMilli = millis();
    //getMotorData();
    KeepStraight(GlobalSpeed);
//    if (_ForwardFlag) SetMotors(GlobalSpeed - PWM_val_d/2, GlobalSpeed + PWM_val_d/2, true);
//    else if (_BackwardFlag) SetMotors(-(GlobalSpeed - PWM_val_d/2), -(GlobalSpeed + PWM_val_d/2), true);
    Serial.print(LeftSideSpeed,DEC);
    Serial.print("\t");
    Serial.println(RightSideSpeed,DEC);
    }
  
}

void SendToAndroid(byte signal, unsigned long value) {
  SntMsg[0] = signal;
  SntMsg[1] = value >> 24;
  SntMsg[2] = value >> 16;
  SntMsg[3] = value >> 8;
  SntMsg[4] = value;
  acc.write(SntMsg, 5);
}

void ReadIncomingSignal() {
  int len = acc.read(RcvMsg, sizeof(RcvMsg), 1);

  if (len > 0) {
    switch(RcvMsg[0]){
      case CmdMoveForward:
        {
          int speed = getIntFromBytes(RcvMsg[1], RcvMsg[2], RcvMsg[3], RcvMsg[4]);
          MoveForward(speed);
          _ForwardFlag = true;
          GlobalSpeed=speed;
//          Serial.print("Forward");
//          Serial.print(speed);
//          Serial.print('\n');
          break;
        }
      case CmdMoveBackward:
        {
          int speed = getIntFromBytes(RcvMsg[1], RcvMsg[2], RcvMsg[3], RcvMsg[4]);
          MoveBackward(speed);
          _BackwardFlag = true;
          GlobalSpeed=speed;
//          Serial.print("Backward");
//          Serial.print(speed);
//          Serial.print('\n');
          break;
        }
      case CmdSpinLeft:
        {
          int speed = getIntFromBytes(RcvMsg[1], RcvMsg[2], RcvMsg[3], RcvMsg[4]);
          SpinLeft(speed);
          _ForwardFlag = false;
          _BackwardFlag = false;
//          Serial.print("Left");
//          Serial.print(speed);
//          Serial.print('\n');
          break;
        }
      case CmdSpinRight:
        {
          int speed = getIntFromBytes(RcvMsg[1], RcvMsg[2], RcvMsg[3], RcvMsg[4]);
          SpinRight(speed);
          _ForwardFlag = false;
          _BackwardFlag = false;
//          Serial.print("Right");
//          Serial.print(speed);
//          Serial.print('\n');
          break;
        }
      case CmdStop:
        {
          Stop();
          _ForwardFlag = false;
          _BackwardFlag = false;
//          Serial.print("Stop");
//          Serial.print('\n');
          break;
        }
      case CmdMoveCameraVert:
        {
          int degrees = getIntFromBytes(RcvMsg[1], RcvMsg[2], RcvMsg[3], RcvMsg[4]);
          MoveCameraVert(degrees);
//          Serial.print("CameraVert");
//          Serial.print(degrees);
//          Serial.print('\n');
          break;
        }
      case CmdMoveCameraHor:
        {
          int degrees = getIntFromBytes(RcvMsg[1], RcvMsg[2], RcvMsg[3], RcvMsg[4]);
          MoveCameraHor(degrees);
//          Serial.print("CameraHor");
//          Serial.print(degrees);
//          Serial.print('\n');
          break;
        }
    }
    }
}

int getIntFromBytes(byte b1, byte b2, byte b3, byte b4)
{
  int value = 0;
  value += b1;
  value <<= 8;
  value += b2;
  value <<= 8;
  value += b3;
  value <<= 8;
  value += b4;
  return value;
}


void ReadSonar() {
  unsigned long frontSum = 0;
  unsigned long rearSum = 0;
  for (int i = 0; i < SONAR_SAMPLES; i++) {
    unsigned long frontDuration = pulseIn(FRONT_SONAR, HIGH);
    unsigned long rearDuration = pulseIn(REAR_SONAR, HIGH);
    frontSum += frontDuration / 147.0 * 2.54;
    rearSum += rearDuration / 147.0 * 2.54;
  }
  
  unsigned long frontAvg = frontSum / SONAR_SAMPLES;
  unsigned long rearAvg = rearSum / SONAR_SAMPLES;
  
//  bool forward = LeftSideSpeed > 0 || RightSideSpeed > 0;
//  if (HeedSonar && ((forward && frontAvg <= SONAR_TAKE_ACTION_THRESHOLD) || (!forward && rearAvg <= SONAR_TAKE_ACTION_THRESHOLD))) {
//    Stop();
//  }
    SendToAndroid(EvtFrontSonar, frontAvg);
    SendToAndroid(EvtRearSonar, rearAvg);
}

void MoveForward(int speed) {
  SetMotors(speed, speed, true);//AnalogInput+Mixed
  lastMilli = millis();
  encoders.getCountsAndResetRight();
  encoders.getCountsAndResetLeft();
}

void MoveBackward(int speed) {
  SetMotors(-speed, -speed, true);//AnalogInput+Mixed
  lastMilli = millis();
  encoders.getCountsAndResetRight();
  encoders.getCountsAndResetLeft();
}

void SpinLeft(int speed) {
  int speed1=(int)(speed*(2.0f*TurnSpeedFactor2+TurnSpeedFactor1)/(2.0f*TurnSpeedFactor1*TurnSpeedFactor2));
  int speed2=(int)(speed*(2.0f*TurnSpeedFactor2-TurnSpeedFactor1)/(2.0f*TurnSpeedFactor1*TurnSpeedFactor2));
  SetMotors(speed1, speed2, false);//AnalogInput+Mixed
}

void SpinRight(int speed) {
  int speed1=(int)(speed*(2.0f*TurnSpeedFactor2-TurnSpeedFactor1)/(2.0f*TurnSpeedFactor1*TurnSpeedFactor2));
  int speed2=(int)(speed*(2.0f*TurnSpeedFactor2+TurnSpeedFactor1)/(2.0f*TurnSpeedFactor1*TurnSpeedFactor2));
  SetMotors(speed1, speed2, false);//AnalogInput+Mixed
}

void MoveCameraVert(int degrees) {
  CameraVert.write(degrees);
}

void MoveCameraHor(int degrees) {
  CameraHor.write(degrees);
}

void Stop() {
  stop();
}

void SetMotors(int FirstSpeed, int SecondSpeed, bool heedSonar) {
  digitalWrite(STBY, HIGH); //disable standby
  if (FirstSpeed>0&&SecondSpeed>0) {
    digitalWrite(In1, HIGH);
    digitalWrite(In2, LOW);
    digitalWrite(In3, HIGH);
    digitalWrite(In4, LOW);
    analogWrite(LEFT_MOTOR_PIN, constrain(map(SecondSpeed, 0, ReceivedMaxSpeed, 0, 255), 0, 255));
    analogWrite(RIGHT_MOTOR_PIN, constrain(map(FirstSpeed, 0, ReceivedMaxSpeed, 0, 255), 0, 255));
    }
   if (FirstSpeed<0&&SecondSpeed<0) {
    digitalWrite(In2, HIGH);
    digitalWrite(In1, LOW);
    digitalWrite(In4, HIGH);
    digitalWrite(In3, LOW);  
    analogWrite(RIGHT_MOTOR_PIN, constrain(map(-FirstSpeed, 0, ReceivedMaxSpeed, 0, 255), 0, 255));
    analogWrite(LEFT_MOTOR_PIN, constrain(map(-SecondSpeed, 0, ReceivedMaxSpeed, 0, 255), 0, 255));
    }
  HeedSonar = heedSonar;
}

void stop(){
//enable standby  
  digitalWrite(STBY, LOW);
  digitalWrite(In1, LOW);
  digitalWrite(In2, LOW);
  digitalWrite(In3, LOW);
  digitalWrite(In4, LOW);
}

void KeepStraight(int speed){
   getMotorData();                                                           // calculate speed
   PWM_val_a= updatePid(Kp_a, Kd_a, PWM_val_a, map(speed, 0, ReceivedMaxSpeed, 0, LeftMotorMaxSpeed), LeftSideSpeed);      // compute a PWM correction to compensate speed difference to reqire speed
   PWM_val_b= updatePid(Kp_b, Kd_b, PWM_val_b, map(speed, 0, ReceivedMaxSpeed, 0, RightMotorMaxSpeed), RightSideSpeed);     // compute a PWM correction to compensate speed difference to reqire speed
   PWM_val_d= updatePid(Kp_d, Kd_d, 0, LeftSideSpeed-RightSideSpeed);     // compute a PWM correction to compensate speed difference to two motors to keep straight
  }
  
void getMotorData()  { // calculate speed
    RightSideSpeed =(int) abs(encoders.getCountsRight()*60/gear_ratio*1000/LOOPTIME/count_per_rev);      // 64 pulses X 50 gear ratio = 3200 counts per output shaft rev
    LeftSideSpeed = (int) abs(encoders.getCountsLeft()*60/gear_ratio*1000/LOOPTIME/count_per_rev);      // 64 pulses X 50 gear ratio = 3200 counts per output shaft rev;
    encoders.getCountsAndResetRight();
    encoders.getCountsAndResetLeft();
}

int updatePid(float Kp, float Kd, int PWM_val, int targetValue, int currentValue)   {             // compute PWM value
register float pidTerm = 0;                                                            // PID correction
register int error=0;                                  
static int last_error=0;
static float integra_error=0; 
                            
error = targetValue - currentValue;
//integra_error += error*LOOPTIME;
//constrain(integra_error, 0, 255);
pidTerm = (Kp * error) + (Kd * (error - last_error));//+ (Ki_a*integra_error);                            
last_error = error;
return constrain(PWM_val + int(pidTerm), 0, ReceivedMaxSpeed);
}

int updatePid(float Kp, float Kd, int targetValue, int currentValue)   {             // compute PWM value
register float pidTerm = 0;                                                            // PID correction
register int error=0;                                  
static int last_error=0;
static float integra_error=0; 
                            
error = targetValue - currentValue;
//integra_error += error*LOOPTIME;
//constrain(integra_error, 0, 255);
pidTerm = (Kp * error) + (Kd * (error - last_error));//+ (Ki_a*integra_error);                            
last_error = error;
return int(pidTerm);
}

