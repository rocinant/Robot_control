#include <SPI.h>
#include <string.h>
#include <SD.h>

#include <PololuWheelEncoders2.h>
#include <Servo.h>

#include <Wire.h>
#include <Max3421e.h>
#include <Usb.h>
#include <AndroidAccessory.h>

#include <limits.h>

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

// set up variables using the SD utility library functions:
Sd2Card card;
SdVolume volume;
SdFile root;

// change this to match your SD shield or module;
// Arduino Ethernet shield: pin 4
// Adafruit SD shields and modules: pin 10
// Sparkfun SD shield: pin 8
const int chipSelect = 4;
//SD card File to store the data
File dataFile;
String filename;

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

#define  ReceivedMaxSpeed  255
#define  TurnSpeedFactor1 2 //to control the average moving speed when you know the turning speed, average moving speed=(left speed + right speed)/2=turning speed/TurnSpeedFactor1
#define  TurnSpeedFactor2 5 //to control the moving speed difference when you know the turning speed,  speed difference=|left speed - right speed|=turning speed/TurnSpeedFactor2, 2*TurnSpeedFactor2-TurnSpeedFactor1 should be >0
const int WHEEL_CIR = 47;  //wheel circumference in cm
const float WHEEL_WIDTH = 30.3f;  //wheel width in cm
const int BAUD_RATE = 9600;

//Servo BaseRotate;
//Servo CameraHor;
//Servo CameraVert;
//const int BASE_ROTATE_PIN = 3; 
//const int CAM_HOR_PIN = 4;
//const int CAM_VERT_PIN = 5;

const int FRONT_SONAR = A5;
const int SECOND_SONAR = A4;
const int SONAR_Number = 2;
unsigned int sonarReading[SONAR_Number];
const int SONAR_CHAIN_PIN = 11;
const int SONAR_SAMPLES = 20;//for average sonar reading
const int SONAR_TAKE_ACTION_THRESHOLD = 6;//sonar threshold to stop
const int DISTANCE_OFFEST_THRESHOLD = 10;
bool HeedSonar = true;
unsigned long lastMilliForSonar=0;
#define LOOPTIMESONAR        (50 * SONAR_Number)                     // PID loop time, read sonar frequency
int PWM_val_d_d = 0;
int LoopCount = 0;                   //for control broadcast sonar reading frequency
const int BROADCAST_SONAR_FREQ = 5;//fequency to send sonar reading to android
static long orginalDistance = 0;
static long orginalDistance2 = 0;
volatile bool _SonarTunning = false;                                                        // PID correction
int errorDistance=0;                                  
static int last_errorDistance=0;
#define  MotorMaxSpeed       400
long Kp_sonar =   2*0.2*WHEEL_WIDTH*1000/MotorMaxSpeed*1000/MotorMaxSpeed*60*255/WHEEL_CIR*60*255/WHEEL_CIR;                                // PID proportional control Gain about 
float Kd_sonar =  0.1f;                                // PID Derivitave control gain
float Ki_sonar =  5.0f;                                // PID Derivitave control gain
#define NUMREADINGS     10
int distance_array[NUMREADINGS];
int indiceForAverageDis = 0;

unsigned long lastMilli=0;
#define count_per_rev      64                  // encoder produces 64 pulses per output shaft revolution
#define gear_ratio 30
#define LOOPTIME        20                     // PID loop time
int LeftSideSpeed = 0; //left wheel's real speed RPM
int RightSideSpeed = 0; //right wheel's real speed RPM

int PWM_val_a = 0;                             
int PWM_val_b = 0;                              
int PWM_val_d = 0;
                            
float Kp_a =   .4f;                                // PID proportional control Gain
float Kd_a =    0.4f;                                // PID Derivitave control gain
float Ki_a =    0.01f;                                // PID Integration control gain

float Kp_b =   .4;                                // PID proportional control Gain
float Kd_b =    0.4f;                                // PID Derivitave control gain
float Ki_b =    0.01f;                                // PID Integration control gain

float Kp_d =   0.7f;                                // PID proportional control Gain
float Kd_d =    0.0f;                                // PID Derivitave control gain
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

#define  CLEANALLSDFILES 0
  
void setup() {
  Serial.begin(BAUD_RATE);

   while (!Serial) {
    ; // wait for serial port to connect. Needed for native USB port only
  }


  Serial.print("\nInitializing SD card...");

  // we'll use the initialization code from the utility libraries
  // since we're just testing if the card is working!
  if (!card.init(SPI_HALF_SPEED, chipSelect)) {
    Serial.println("initialization failed. Things to check:");
    Serial.println("* is a card inserted?");
    Serial.println("* is your wiring correct?");
    Serial.println("* did you change the chipSelect pin to match your shield or module?");
    return;
  } else {
    Serial.println("Wiring is correct and a card is present.");
  }

  // print the type of card
  Serial.print("\nCard type: ");
  switch (card.type()) {
    case SD_CARD_TYPE_SD1:
      Serial.println("SD1");
      break;
    case SD_CARD_TYPE_SD2:
      Serial.println("SD2");
      break;
    case SD_CARD_TYPE_SDHC:
      Serial.println("SDHC");
      break;
    default:
      Serial.println("Unknown");
  }

  // Now we will try to open the 'volume'/'partition' - it should be FAT16 or FAT32
  if (!volume.init(card)) {
    Serial.println("Could not find FAT16/FAT32 partition.\nMake sure you've formatted the card");
    return;
  }


  // print the type and size of the first FAT-type volume
  uint32_t volumesize;
  Serial.print("\nVolume type is FAT");
  Serial.println(volume.fatType(), DEC);
  Serial.println();

  volumesize = volume.blocksPerCluster();    // clusters are collections of blocks
  volumesize *= volume.clusterCount();       // we'll have a lot of clusters
  volumesize *= 512;                            // SD card blocks are always 512 bytes
  Serial.print("Volume size (bytes): ");
  Serial.println(volumesize);
  Serial.print("Volume size (Kbytes): ");
  volumesize /= 1024;
  Serial.println(volumesize);
  Serial.print("Volume size (Mbytes): ");
  volumesize /= 1024;
  Serial.println(volumesize);


  Serial.println("\nFiles found on the card (name, date and size in bytes): ");
  root.openRoot(volume);

  // list all files in the card with date and size
  root.ls(LS_R | LS_DATE | LS_SIZE);

  // see if the card is present and can be initialized:
  if (!SD.begin(chipSelect)) {
    Serial.println("Card failed, or not present");
    // don't do anything more:
    return;
  }
  Serial.println("card initialized.");

  File root0 = SD.open("/");
  #if CLEANALLSDFILES
  deleteAllFile(root0);
  #endif
  
  char intStr[255];
  itoa(printDirectoryAndGetMaxFileName(root0, 0)+1, intStr, 10);
  filename = String(intStr) + ".txt";
  dataFile = SD.open(filename, FILE_WRITE);
  dataFile.close();
  Serial.print("New created file name:");
  Serial.print('\t');
  Serial.println(filename);
  

//  pinMode(BASE_ROTATE_PIN, OUTPUT);
//  pinMode(CAM_HOR_PIN, OUTPUT);
//  pinMode(CAM_VERT_PIN, OUTPUT);
//  BaseRotate.attach(BASE_ROTATE_PIN);
//  CameraHor.attach(CAM_HOR_PIN);
//  CameraVert.attach(CAM_VERT_PIN);
//  BaseRotate.write(90);
//  CameraHor.write(90);
//  CameraVert.write(90);

  pinMode(RIGHT_MOTOR_PIN, OUTPUT);
  pinMode(LEFT_MOTOR_PIN, OUTPUT);
  pinMode(STBY, OUTPUT);
  pinMode(In1,OUTPUT); 
  pinMode(In2,OUTPUT);
  pinMode(In3,OUTPUT);
  pinMode(In4,OUTPUT);
//  rightSideMotors.attach(RIGHT_MOTOR_PIN);
//  leftSideMotors.attach(LEFT_MOTOR_PIN);
  
  //pinMode(FRONT_SONAR, INPUT);
  //pinMode(REAR_SONAR, INPUT);
  // enable the internal pullups
  //digitalWrite(FRONT_SONAR, HIGH);
  //digitalWrite(REAR_SONAR, HIGH);
  
  encoders.init(A8, A9, A10, A11);//look into encoders' reading

  //Start sonar chain
  pinMode(SONAR_CHAIN_PIN,OUTPUT);
  delay(250);
  digitalWrite(SONAR_CHAIN_PIN, HIGH);
  delay(1);
  digitalWrite(SONAR_CHAIN_PIN, LOW);
  // Make high-impedance input to signal round-robin ranging.
  //pinMode(SONAR_CHAIN_PIN, INPUT);
  // wait for calibration+initial ranging times.
  delay(49+49+100);
  // sonars should alternate automatically now
  
  for(int i=0; i<NUMREADINGS; i++)   {distance_array[i] = 0;} // initialize readings to 0
//  MoveForward(100);
//  GlobalSpeed = 100;
//  _ForwardFlag = true;

  acc.powerOn();
  stop();
}

void loop() {

  if (acc.isConnected()) {
    ReadIncomingSignal();
  }
  
  if (_ForwardFlag || _BackwardFlag ) {
    // make a string for assembling the data to log:
    String dataString = "";  

        if((millis() - lastMilliForSonar) >= LOOPTIMESONAR){                                    // enter PID correction for distance mantain
        lastMilliForSonar = millis();

                if (_SonarTunning == true){ //initialization finished
          // read sensors and append to the string: store the distance data in an int array use readSonar()
          dataString += "Distance: ";
          ReadSonarTwo();//populate the array with distance readings
          int distance = sonarReading[0]; //to record the minimal distance as the real value
          dataString += String(distance);
          dataString += ",";
          for (int i = 1; i < SONAR_Number; i++) {
            dataString += String(sonarReading[i]);
            distance = (sonarReading[i] < distance) ? sonarReading[i] : distance;
            if (i < SONAR_Number - 1) {
              dataString += ",";
            }
            else dataString += "; ";
          }
          errorDistance = orginalDistance - distance;
          //if (abs(errorDistance) > SONAR_TAKE_ACTION_THRESHOLD || abs(orginalDistance2 - distance) > DISTANCE_OFFEST_THRESHOLD){
            PWM_val_d_d = (Kp_sonar / (GlobalSpeed+50) * errorDistance / LOOPTIMESONAR / LOOPTIMESONAR) + Ki_sonar*(orginalDistance2 - distance);
            dataString += "Add distance Control: ";
            dataString += String(PWM_val_d_d);
          //}
          last_errorDistance = errorDistance;
          orginalDistance = distance;
      }
        
        if (_SonarTunning == false){ //for initialize orginal distance we should maintain
          ReadSonarTwo();//populate the array with distance readings
          int distance = sonarReading[0]; //to record the minimal distance as the real value
          for (int j = 1; j < SONAR_Number; j++) {
            distance = (sonarReading[j] < distance) ? sonarReading[j] : distance;
          }
          orginalDistance2 = orginalDistance = digital_smooth(distance, distance_array);
          indiceForAverageDis++;
          if (indiceForAverageDis == NUMREADINGS){
            _SonarTunning = true;
            dataFile = SD.open(filename, FILE_WRITE);
            // if the file is available, write to it:
            if (dataFile) {
              dataFile.println("Orginal distance" + String(orginalDistance2));
              dataFile.close();
            }
            // if the file isn't open, pop up an error:
            else {
              Serial.println("error opening datalog.txt");
                }
            }
        }

    }
    
    if((millis()-lastMilli) >= LOOPTIME){                                    // enter PID correction for speed difference
        lastMilli = millis();
        getMotorData();                                                           // calculate speed
        dataString += " L&R motor speed: ";
        dataString += String(LeftSideSpeed);
        dataString += ", ";
        dataString += String(RightSideSpeed);
        dataString += "; ";
        PWM_val_d = updatePid(Kp_d, Kd_d, 0, LeftSideSpeed-RightSideSpeed) + PWM_val_d_d;
        dataString += "Straight Control: ";
        dataString += String(PWM_val_d);
        if (_ForwardFlag) {SetMotors(GlobalSpeed - PWM_val_d/2, GlobalSpeed + PWM_val_d/2, true); lastMilli = millis(); encoders.getCountsAndResetRight();encoders.getCountsAndResetLeft();}
        else if (_BackwardFlag) {SetMotors(-(GlobalSpeed - PWM_val_d/2), -(GlobalSpeed + PWM_val_d/2), true); lastMilli = millis(); encoders.getCountsAndResetRight();encoders.getCountsAndResetLeft();}  
      }

    if (sizeof(dataString) > 3){
         //Serial.println(dataString);
         // open the file. note that only one file can be open at a time,
         // so you have to close this one before opening another.
         dataFile = SD.open(filename, FILE_WRITE);
         // if the file is available, write to it:
         if (dataFile) {
           dataFile.println(dataString);
           dataFile.close();
         }
         // if the file isn't open, pop up an error:
         else {
           Serial.println("error opening datalog.txt");
         } 
        }

  }
  
}

void UpdateLoopCount() {
  LoopCount = (LoopCount % 1000) + 1;
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
      //if(RcvMsg[0]==CmdMoveForward)
      case CmdMoveForward:
        {
          int speed = getIntFromBytes(RcvMsg[1], RcvMsg[2], RcvMsg[3], RcvMsg[4]);
          MoveForward(speed);
          _ForwardFlag = true;
          _BackwardFlag = false;
          GlobalSpeed=speed;
          break;
//          Serial.print("Forward");
//          Serial.print(speed);
//          Serial.print('\n');

        }
      //else if(RcvMsg[0]==CmdMoveBackward)
      case CmdMoveBackward:
        {
          int speed = getIntFromBytes(RcvMsg[1], RcvMsg[2], RcvMsg[3], RcvMsg[4]);
          MoveBackward(speed);
          _BackwardFlag = true;
          _ForwardFlag = false;
          GlobalSpeed=speed;
          break;
//          Serial.print("Backward");
//          Serial.print(speed);
//          Serial.print('\n');
        }
      //else if(RcvMsg[0]==CmdSpinLeft)
      case CmdSpinLeft:
        {
          int speed = getIntFromBytes(RcvMsg[1], RcvMsg[2], RcvMsg[3], RcvMsg[4]);
          SpinLeft(speed);
          _ForwardFlag = false;
          _BackwardFlag = false;
          break;
//          Serial.print("Left");
//          Serial.print(speed);
//          Serial.print('\n');
        }
      //else if(RcvMsg[0]==CmdSpinRight)
      case CmdSpinRight:
        {
          int speed = getIntFromBytes(RcvMsg[1], RcvMsg[2], RcvMsg[3], RcvMsg[4]);
          SpinRight(speed);
          _ForwardFlag = false;
          _BackwardFlag = false;
          break;
//          Serial.print("Right");
//          Serial.print(speed);
//          Serial.print('\n');
        }
      //else if(RcvMsg[0]==CmdStop)
      case CmdStop:
        {
          Stop();
          _ForwardFlag = false;
          _BackwardFlag = false;
          break;
//          Serial.print("Stop");
//          Serial.print('\n');
        }
      //else if(RcvMsg[0]==CmdMoveCameraVert)
      case CmdMoveCameraVert:
        {
          int degrees = getIntFromBytes(RcvMsg[1], RcvMsg[2], RcvMsg[3], RcvMsg[4]);
          MoveCameraVert(degrees);
          break;
//          Serial.print("CameraVert");
//          Serial.print(degrees);
//          Serial.print('\n');
        }
      //else if(RcvMsg[0]==CmdMoveCameraHor)
      case CmdMoveCameraHor:
        {
          int degrees = getIntFromBytes(RcvMsg[1], RcvMsg[2], RcvMsg[3], RcvMsg[4]);
          MoveCameraHor(degrees);
//          break;
//          Serial.print("CameraHor");
//          Serial.print(degrees);
//          Serial.print('\n');
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

int ReadSonar() {
  float frontSum = 0;
  //unsigned long frontSum = 0;
  //unsigned long rearSum = 0;
  //for (int i = 0; i < SONAR_SAMPLES; i++) {
    //unsigned long frontDuration = pulseIn(FRONT_SONAR, HIGH);
    unsigned long frontDuration = analogRead(FRONT_SONAR);
    //unsigned long rearDuration = pulseIn(REAR_SONAR, HIGH);
    //frontSum += frontDuration / 147.0 * 2.54; //1-inch resolution Readings can occur up to every 49mS, (20-Hz rate)
    frontSum = frontDuration * 512.0 /1023 * 2.54; //1-inch resolution Readings can occur up to every 49mS, (20-Hz rate) analogue reading mode, analog voltage with a scaling factor of (Vcc/512) per inch thus the distance is read *vcc /1023 /(vcc/512) inches
    //rearSum += rearDuration / 147.0 * 2.54;
  //}
  
  //unsigned long frontAvg = (int) (frontSum / SONAR_SAMPLES);
  unsigned int frontAvg = (int) (frontSum);
  //unsigned long rearAvg = rearSum / SONAR_SAMPLES;
  return frontAvg;
  
//  bool forward = LeftSideSpeed > 0 || RightSideSpeed > 0;
//  if (HeedSonar && ((forward && frontAvg <= SONAR_TAKE_ACTION_THRESHOLD) || (!forward && rearAvg <= SONAR_TAKE_ACTION_THRESHOLD))) {
//    Stop();
//  }
//      if (LoopCount % BROADCAST_SONAR_FREQ == 0) {
//    SendToAndroid(EvtFrontSonar, frontAvg);
//    SendToAndroid(EvtRearSonar, rearAvg);
//      }
}

void ReadSonarTwo() {
  digitalWrite(SONAR_CHAIN_PIN, HIGH);
  delay(1);
  digitalWrite(SONAR_CHAIN_PIN, LOW);
  sonarReading[1] = (int) (analogRead(SECOND_SONAR) * 512.0 /1023 * 2.54);
  sonarReading[0] = (int) (analogRead(FRONT_SONAR) * 512.0 /1023 * 2.54);
}

void MoveForward(int speed) {
  static int prespeed = speed;
  SetMotors(speed, speed, true);//AnalogInput+Mixed
  if (prespeed != speed){ 
    lastMilli = millis();
    encoders.getCountsAndResetRight();
    encoders.getCountsAndResetLeft();
    prespeed = speed;
  }
}

void MoveBackward(int speed) {
  static int prespeed = -speed;
  SetMotors(-speed, -speed, true);//AnalogInput+Mixed
  if (prespeed != -speed){ 
    lastMilli = millis();
    encoders.getCountsAndResetRight();
    encoders.getCountsAndResetLeft();
    prespeed = -speed;
  }
}

void SpinLeft(int speed) {
//  int speed1=(int)(speed*(2.0f*TurnSpeedFactor2+TurnSpeedFactor1)/(2.0f*TurnSpeedFactor1*TurnSpeedFactor2));
//  int speed2=(int)(speed*(2.0f*TurnSpeedFactor2-TurnSpeedFactor1)/(2.0f*TurnSpeedFactor1*TurnSpeedFactor2));
  int speed1=(int)(speed*0.5f + GlobalSpeed*0.9);
  int speed2=(int)(GlobalSpeed*0.9 - speed*0.5f);
  SetMotors(speed1, speed2, false);//AnalogInput+Mixed
}

void SpinRight(int speed) {
//  int speed1=(int)(speed*(2.0f*TurnSpeedFactor2-TurnSpeedFactor1)/(2.0f*TurnSpeedFactor1*TurnSpeedFactor2));
//  int speed2=(int)(speed*(2.0f*TurnSpeedFactor2+TurnSpeedFactor1)/(2.0f*TurnSpeedFactor1*TurnSpeedFactor2));
  float factorToGlobalSpeed = 0.95;
  float factorToSpeedDif;
  if (speed > factorToGlobalSpeed*GlobalSpeed) factorToSpeedDif = (255 + speed - 2 * factorToGlobalSpeed*GlobalSpeed) / (2 * speed); //factorToSpeedDif can not be 0.5
  else if ( (factorToGlobalSpeed*GlobalSpeed <= 0.666667*255) || ( (speed < 2*(255-factorToGlobalSpeed*GlobalSpeed)) && (factorToGlobalSpeed*GlobalSpeed > 0.666667*255) ) ) factorToSpeedDif = 0.5f;
  else factorToSpeedDif = (255 - factorToGlobalSpeed*GlobalSpeed) / (2 * speed); //factorToSpeedDif can not be 0.5
  int speed1=(int)(GlobalSpeed*factorToGlobalSpeed - speed*(1 - factorToSpeedDif));
  int speed2=(int)(GlobalSpeed*factorToGlobalSpeed + speed*factorToSpeedDif);
  SetMotors(speed1, speed2, false);//AnalogInput+Mixed
}

void MoveCameraVert(int degrees) {
  //CameraVert.write(degrees);
}

void MoveCameraHor(int degrees) {
  //CameraHor.write(degrees);
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
//   analogWrite(LEFT_MOTOR_PIN, map(SecondSpeed, 0, ReceivedMaxSpeed, 0, 255));  
//   analogWrite(RIGHT_MOTOR_PIN, map(FirstSpeed, 0, ReceivedMaxSpeed, 0, 255));
    analogWrite(LEFT_MOTOR_PIN, constrain(map(SecondSpeed, 0, ReceivedMaxSpeed, 0, 255), 0, 255));  
    analogWrite(RIGHT_MOTOR_PIN, constrain(map(FirstSpeed, 0, ReceivedMaxSpeed, 0, 255), 0, 255));
    }
   if (FirstSpeed<0&&SecondSpeed<0) {
    digitalWrite(In2, HIGH);
    digitalWrite(In1, LOW);
    digitalWrite(In4, HIGH);
    digitalWrite(In3, LOW);  
//    analogWrite(LEFT_MOTOR_PIN, map(-SecondSpeed, 0, ReceivedMaxSpeed, 0, 255));  
//    analogWrite(RIGHT_MOTOR_PIN, map(-FirstSpeed, 0, ReceivedMaxSpeed, 0, 255));
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
//   PWM_val_a= updatePid(Kp_a, Kd_a, PWM_val_a, map(speed, 0, ReceivedMaxSpeed, 0, LeftMotorMaxSpeed), LeftSideSpeed);      // compute a PWM correction to compensate speed difference to reqire speed
//   PWM_val_b= updatePid(Kp_b, Kd_b, PWM_val_b, map(speed, 0, ReceivedMaxSpeed, 0, RightMotorMaxSpeed), RightSideSpeed);     // compute a PWM correction to compensate speed difference to reqire speed
   PWM_val_d= updatePid(Kp_d, Kd_d, 0, LeftSideSpeed-RightSideSpeed);     // compute a PWM correction to compensate speed difference to two motors to keep straight
  }
  
void getMotorData()  { // calculate speed
    LeftSideSpeed = (int) abs(encoders.getCountsLeft()*60/gear_ratio*1000/LOOPTIME/count_per_rev);      // 64 pulses X 50 gear ratio = 3200 counts per output shaft rev;
    RightSideSpeed = (int) abs(encoders.getCountsRight()*60/gear_ratio*1000/LOOPTIME/count_per_rev);      // 64 pulses X 50 gear ratio = 3200 counts per output shaft rev
    encoders.getCountsAndResetLeft();
    encoders.getCountsAndResetRight();
}

int updatePid(float Kp, float Kd, int PWM_val, int targetValue, int currentValue)   {             // compute PWM value
register float pidTerm = 0;                                                            // PID correction
register int error=0;                                  
static int last_error=0;
static float integra_error=0; 
                            
error = targetValue - currentValue;
//integra_error += error;
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
//integra_error += error;
//constrain(integra_error, 0, 255);
pidTerm = (Kp * error); // + (Kd * (error - last_error));//+ (Ki_a*integra_error);                            
last_error = error;
return round(pidTerm);
}

int digital_smooth(int value, int *data_array)  // remove signal noise  value is the most updated value data_array is the array that stored all values needed to be averaged
{
  static int ndx=0;                                                         
  static int count=0;                          
  static int total=0;                          
 total -= data_array[ndx];               
 data_array[ndx] = value;                
 total += data_array[ndx];               
 ndx = (ndx+1) % NUMREADINGS;                                
 if(count < NUMREADINGS)      count++;
 return total/count;
}

int printDirectoryAndGetMaxFileName(File dir, int numTabs) {
  static int maxFileName = -1; 
  while (true) {
    File entry =  dir.openNextFile();
    if (! entry) {
      // no more files
      return  maxFileName;
    }
    for (uint8_t i = 0; i < numTabs; i++) {
      Serial.print('\t');
    }
    Serial.print(entry.name());
    if (entry.isDirectory()) {
      Serial.println("/");
      printDirectoryAndGetMaxFileName(entry, numTabs + 1);
    } else {
      // files have sizes, directories do not
      Serial.print("\t\t");
      Serial.println(entry.size(), DEC);
      int currentFileNumber = strtol(entry.name(), (char **)NULL, 10);
      maxFileName = (currentFileNumber > maxFileName) ? currentFileNumber:  maxFileName;
    }
    entry.close();
  }
}

void deleteAllFile(File dir) {
  while (true) {
    File entry =  dir.openNextFile();
    if (! entry) {
      // no more files
      break;
    }
    if (entry.isDirectory()) {
     deleteAllFile(entry);
    } else {
      SD.remove(entry.name());
    }
  }
}

