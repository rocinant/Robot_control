#include <SPI.h>

#include <string.h>
#include <SD.h>

#include <limits.h>

// This sketch code is based on the RPLIDAR driver library provided by RoboPeak
#include <RPLidar.h>

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

#define  CLEANALLSDFILES 0

// You need to create an driver instance 
RPLidar lidar;

#define RPLIDAR_MOTOR 3 // The PWM pin for control the speed of RPLIDAR's motor.
                        // This pin should connected with the RPLIDAR's MOTOCTRL signal 
const int BAUD_RATE = 115200;
  
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

  // bind the RPLIDAR driver to the arduino hardware serial
  lidar.begin(Serial1);//Baud Rate 115200
  
  // set pin modes
  pinMode(RPLIDAR_MOTOR, OUTPUT);
}

float minDistance = 70000;
float angleAtMinDist = 0;

void loop() {

    if (IS_OK(lidar.waitPoint())) {
       // make a string for assembling the data to log:
       String dataString = "";  
       // read sensors and append to the string: store the distance data
       float distance = lidar.getCurrentPoint().distance; //distance value in mm unit
       float angle    = lidar.getCurrentPoint().angle; //anglue value in degree
       bool  startBit = lidar.getCurrentPoint().startBit; //whether this point is belong to a new scan
       byte  quality  = lidar.getCurrentPoint().quality; //quality of the current measurement
    
      //perform data processing here... 
      dataString += "Distance (mm): ";
      dataString += String(distance);
      dataString += " Angle (degree): ";
      dataString += String(angle);
      dataString += " new scan? ";
      dataString += String(startBit);
      dataString += " quality: ";
      dataString += String(quality);
      dataString += "; ";

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

//    float distance = lidar.getCurrentPoint().distance;
//    float angle = lidar.getCurrentPoint().angle;
//    
//    if (lidar.getCurrentPoint().startBit) {
//      // a new scan, display the previous data...
//       Serial.print("Minimal Distance (mm): ");
//       Serial.print(minDistance);
//       Serial.print(" Angle (degree): ");
//       Serial.println(angleAtMinDist);
//       minDistance = 70000;
//       angleAtMinDist = 0;
//    } else {
//       if ( distance > 10.0 &&  distance < minDistance) {
//          minDistance = distance;
//          angleAtMinDist = angle;
//       }
//    }
    
  } else {
    analogWrite(RPLIDAR_MOTOR, 0); //stop the rplidar motor
    
    // try to detect RPLIDAR... 
    rplidar_response_device_info_t info;
    if (IS_OK(lidar.getDeviceInfo(info, 100))) {
       // detected...
       lidar.startScan();
       
       // start motor rotating at max allowed speed
       analogWrite(RPLIDAR_MOTOR, 255);
       delay(1000);
    }
  }
  
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

