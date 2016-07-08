#include <SPI.h>
#include <string.h>
#include <SD.h>

#include <limits.h>

String inputString = "";         // a string to hold incoming data
boolean stringComplete = false;  // whether the string is complete

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

const int BAUD_RATE = 9600;

void setup() {
Serial.begin(BAUD_RATE); // set the baud rate

   while (!Serial) {
    ; // wait for serial port to connect. Needed for native USB port only
  }
  
Serial.print("Ready"); // print "Ready" once
// reserve 200 bytes for the inputString:
inputString.reserve(18);
delay(500);

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
  //dataFile.println("Begin");
  dataFile.close();
  Serial.print("New created file name:");
  Serial.print('\t');
  Serial.println(filename);   
}

void loop() {
  serialEvent(); //call the function
  // print the string when a newline arrives:
  if (stringComplete) {
    dataFile = SD.open(filename, FILE_WRITE);
    // if the file is available, write to it:
    if (dataFile){
//      char substring[inputString.length()];
//      strncpy(substring, inputString.c_str(), inputString.length()-1);
//      substring[sizeof(substring) - 1] = '\0';
//      String processedString(substring);
//      dataFile.println(processedString);
      dataFile.println(inputString);
      //dataFile.flush();
      dataFile.close();
    }
    // if the file isn't open, pop up an error:
    else {
      Serial.println("error opening datalog.txt");
    }
    inputString = "";
    stringComplete = false;
    dataFile.close();
  }
}

/*
  SerialEvent occurs whenever a new data comes in the
 hardware serial RX.  This routine is run between each
 time loop() runs, so using delay inside loop can delay
 response.  Multiple bytes of data may be available.
 */
void serialEvent() {
  char inChar;
  while(!Serial.available()) {}
  // serial read section
  while (Serial.available()) {
//    if (Serial.available() > 0)
//    {
    // get the new byte:
    inChar = (char)Serial.read();
    // add it to the inputString:
    inputString += inChar;
//    }

//    if (inChar == 'E') {
//      stringComplete = true;
//      break;
//    }
  }
    // if the incoming character is a newline, set a flag
    // so the main loop can do something about it:
    if (inChar == '\n') {
      stringComplete = true;
    }

//    if (inputString.length() > 1) {
//      stringComplete = true;
//    }
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
