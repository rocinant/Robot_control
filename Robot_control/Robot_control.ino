//#define NO_PORTA_PINCHANGES //Only use port C (Digital pin 30-37) to avoid conflict with MySoftwareSerial library
//#define NO_PORTB_PINCHANGES //Only use port C (Digital pin 30-37) to avoid conflict with MySoftwareSerial library
//#define NO_PORTD_PINCHANGES //Only use port C (Digital pin 30-37) to avoid conflict with MySoftwareSerial library
#include <PololuWheelEncoders2.h>
#include <Servo.h>
//#include <MySoftwareSerial.h>
#include <SabertoothSimplified.h>

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

const int RIGHT_MOTOR_PIN = 14;//for Analog Input mode
const int LEFT_MOTOR_PIN = 6;//for Analog Input mode
PololuWheelEncoders encoders;

#define  ReceivedMaxSpeed  100
#define  TurnSpeedFactor 1.5 //to control the moving speed when you know the turing speed, should be value between 1 and 2

//#define  SimpleSerialMotorPin      13
//SoftwareSerial SWSerial(NOT_A_PIN, SimpleSerialMotorPin); // RX on no pin (unused), TX on pin 16 (to S1).
//SabertoothSimplified ST(SWSerial); // Use SWSerial as the serial port.
SabertoothSimplified ST;

const int WHEEL_CIR = 36;  //wheel circumference in cm
const int BAUD_RATE = 115200;

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
#define LOOPTIME        100                     // PID loop time
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
float Kp_d =   .2f;                                // PID proportional control Gain
float Kd_d =    0.1f;                                // PID Derivitave control gain
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
enum MotorDriverMode1 { AnalogInput, SimplelifiedSerial };
#define  DefaultMode1  SimplelifiedSerial
enum MotorDriverMode2 { Independent, Mixed };
#define  DefaultMode2  Independent

void setup() {
  Serial.begin(BAUD_RATE);
//  SWSerial.begin(9600);

  pinMode(BASE_ROTATE_PIN, OUTPUT);
  pinMode(CAM_HOR_PIN, OUTPUT);
  pinMode(CAM_VERT_PIN, OUTPUT);
  BaseRotate.attach(BASE_ROTATE_PIN);
  CameraHor.attach(CAM_HOR_PIN);
  CameraVert.attach(CAM_VERT_PIN);
  BaseRotate.write(90);
  CameraHor.write(90);
  CameraVert.write(90);

  //digitalWrite(RIGHT_MOTOR_PIN, 1);
  //digitalWrite(LEFT_MOTOR_PIN, 1);
  pinMode(RIGHT_MOTOR_PIN, OUTPUT);
  pinMode(LEFT_MOTOR_PIN, OUTPUT);
//  rightSideMotors.attach(RIGHT_MOTOR_PIN);
//  leftSideMotors.attach(LEFT_MOTOR_PIN);
  
  pinMode(FRONT_SONAR, INPUT);
  pinMode(REAR_SONAR, INPUT);
  // enable the internal pullups
  digitalWrite(FRONT_SONAR, HIGH);
  digitalWrite(REAR_SONAR, HIGH);
  
  encoders.init(10, 11, 13, 12);//look into encoders' reading

  acc.powerOn();

//  MoveForward(50);
//  _ForwardFlag = true;
//  GlobalSpeed=50;      
}

void loop() {
  
//    if (_ForwardFlag || _BackwardFlag ) {
//    if((millis()-lastMilli) >= LOOPTIME)   {                                    // enter PID correction
//    lastMilli = millis();
//    KeepStraight(GlobalSpeed);
//    if (_ForwardFlag) SetMotors(PWM_val_a, PWM_val_b, true, DefaultMode1, Independent);
//    else if (_BackwardFlag) SetMotors(-PWM_val_a, -PWM_val_b, true, DefaultMode1, Independent);
//    }
//  }
  
  if (acc.isConnected()) {
//    Serial.print("Connected");
//    Serial.print('\n');
    ReadIncomingSignal();
    ReadSonar();
  }
  
  if (_ForwardFlag || _BackwardFlag ) {
    if((millis()-lastMilli) >= LOOPTIME)   {                                    // enter PID correction
    lastMilli = millis();
    KeepStraight(GlobalSpeed);
    if (_ForwardFlag) SetMotors(PWM_val_a, PWM_val_b, true, DefaultMode1, Independent);
    else if (_BackwardFlag) SetMotors(-PWM_val_a, -PWM_val_b, true, DefaultMode1, Independent);
    }
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
  SetMotors(speed, (DefaultMode2==Mixed) ? 0:speed, true, DefaultMode1, DefaultMode2);//AnalogInput+Mixed
  lastMilli = millis();
  encoders.getCountsAndResetRight();
  encoders.getCountsAndResetLeft();
}

void MoveBackward(int speed) {
  SetMotors(-speed, (DefaultMode2==Mixed) ? 0:-speed, true, DefaultMode1, DefaultMode2);//AnalogInput+Mixed
  lastMilli = millis();
  encoders.getCountsAndResetRight();
  encoders.getCountsAndResetLeft();
}

void SpinLeft(int speed) {
  SetMotors((DefaultMode2==Mixed) ? speed/TurnSpeedFactor:round((2-TurnSpeedFactor)/2/TurnSpeedFactor*speed), (DefaultMode2==Mixed) ? -speed:round((2+TurnSpeedFactor)/2/TurnSpeedFactor*speed), false, DefaultMode1, DefaultMode2);//AnalogInput+Mixed
}

void SpinRight(int speed) {
  SetMotors((DefaultMode2==Mixed) ? speed/TurnSpeedFactor:round((2+TurnSpeedFactor)/2/TurnSpeedFactor*speed), (DefaultMode2==Mixed) ? speed:round((2-TurnSpeedFactor)/2/TurnSpeedFactor*speed), false, DefaultMode1, DefaultMode2);//AnalogInput+Mixed
}

void MoveCameraVert(int degrees) {
  CameraVert.write(degrees);
}

void MoveCameraHor(int degrees) {
  CameraHor.write(degrees);
}

void Stop() {
  SetMotors(0, 0, true, DefaultMode1, DefaultMode2);
}

void SetMotors(int FirstSpeed, int SecondSpeed, bool heedSonar, byte mode1,  byte mode2) {
      switch(mode1){
      case AnalogInput:
        {
          switch(mode2){
        case Independent:
        {
          if (FirstSpeed>0) { analogWrite(RIGHT_MOTOR_PIN, map(FirstSpeed, 0, ReceivedMaxSpeed, 255/2, 255));} //  rightSideMotors.write( map(FirstSpeed, 0, ReceivedMaxSpeed, 180/2, 180));
          else { analogWrite(RIGHT_MOTOR_PIN, map(-FirstSpeed, 0, ReceivedMaxSpeed, 255/2, 0));}    //  rightSideMotors.write( map(-FirstSpeed, 0, ReceivedMaxSpeed, 180/2, 0));
          if (SecondSpeed>0) { analogWrite(LEFT_MOTOR_PIN, map(SecondSpeed, 0, ReceivedMaxSpeed, 255/2, 255));}
          else { analogWrite(LEFT_MOTOR_PIN, map(-SecondSpeed, 0, ReceivedMaxSpeed, 255/2, 0));}
          break;
        }
        case Mixed:
        {
          if (FirstSpeed>0) { analogWrite(RIGHT_MOTOR_PIN, map(FirstSpeed, 0, ReceivedMaxSpeed, 255/2, 255));}
          else { analogWrite(RIGHT_MOTOR_PIN, map(-FirstSpeed, 0, ReceivedMaxSpeed, 255/2, 0));}
          if (SecondSpeed>0) { analogWrite(LEFT_MOTOR_PIN, map(SecondSpeed, 0, ReceivedMaxSpeed, 255/2, 255));}
          else { analogWrite(LEFT_MOTOR_PIN, map(-SecondSpeed, 0, ReceivedMaxSpeed, 255/2, 0));}
          break;
        }
          }
          break;
        }
        case SimplelifiedSerial:
        {
          switch(mode2){
        case Independent:
        {
          if (FirstSpeed>0) { ST.motor(1,  map(FirstSpeed, 0, ReceivedMaxSpeed, 0,127)); }
          else { ST.motor(1,  map(-FirstSpeed, 0, ReceivedMaxSpeed, 0,-127));}
          if (SecondSpeed>0) { ST.motor(2,  map(SecondSpeed, 0, ReceivedMaxSpeed, 0,127)); }
          else { ST.motor(2,  map(-SecondSpeed, 0, ReceivedMaxSpeed, 0,-127));}
          break;
        }
        case Mixed:
        {
          if (FirstSpeed>0) { ST.drive(map(FirstSpeed, 0, ReceivedMaxSpeed, 0,127)); }
          else { ST.drive(map(-FirstSpeed, 0, ReceivedMaxSpeed, 0,-127));}
          if (SecondSpeed>0) { ST.turn(map(SecondSpeed, 0, ReceivedMaxSpeed, 0,127)); }
          else { ST.turn(map(-SecondSpeed, 0, ReceivedMaxSpeed, 0,-127));}
          break;
        }
      }
          break;
        }
      }     
  HeedSonar = heedSonar;
}

void KeepStraight(int speed){
   getMotorData();                                                           // calculate speed
   PWM_val_a= updatePid(Kp_a, Kd_a, PWM_val_a, map(speed, 0, ReceivedMaxSpeed, 0, LeftMotorMaxSpeed), LeftSideSpeed);      // compute a PWM correction to compensate speed difference to reqire speed
   PWM_val_b= updatePid(Kp_b, Kd_b, PWM_val_b, map(speed, 0, ReceivedMaxSpeed, 0, RightMotorMaxSpeed), RightSideSpeed);     // compute a PWM correction to compensate speed difference to reqire speed
   PWM_val_d= updatePid(Kp_d, Kd_d, 0, LeftSideSpeed-RightSideSpeed);     // compute a PWM correction to compensate speed difference to two motors to keep straight
   PWM_val_a+=PWM_val_d/2;
   PWM_val_b-=PWM_val_d/2;
  }
  
void getMotorData()  { // calculate speed
    LeftSideSpeed = (int) (abs(encoders.getCountsLeft())*(60*(1000/LOOPTIME)))/(count_per_rev*gear_ratio);      // 64 pulses X 50 gear ratio = 3200 counts per output shaft rev;
    RightSideSpeed = (int) (abs(encoders.getCountsLeft())*(60*(1000/LOOPTIME)))/(count_per_rev*gear_ratio);      // 64 pulses X 50 gear ratio = 3200 counts per output shaft rev
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

