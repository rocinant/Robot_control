const int FRONT_SONAR = A5;
const int SECOND_SONAR = A4;
const int SONAR_Number = 2;
unsigned int sonarReading[SONAR_Number];
const int SONAR_CHAIN_PIN = 11;
unsigned long lastMilli=0;
void setup() {
  Serial.begin(9600);
  // put your setup code here, to run once:
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
}

void loop() {
  // put your main code here, to run repeatedly:
 if((millis()-lastMilli) >= 120){                                    // enter PID correction 20 ms
  lastMilli = millis();
  ReadSonarTwo();//populate the array with i=distance reading
        int distance = sonarReading[0]; //to record the minimal distance as the real value
        Serial.print(distance,DEC);
        Serial.print("\t");
        for (int i = 1; i < SONAR_Number; i++) {
          distance = (sonarReading[i] < distance) ? sonarReading[i] : distance;
          if (i < SONAR_Number - 1) {
            Serial.print(sonarReading[i],DEC);
            Serial.print("\t");
          }
          else  Serial.println(sonarReading[i],DEC);;
        }
 }
        
}

void ReadSonarTwo() {
  digitalWrite(SONAR_CHAIN_PIN, HIGH);
  delay(1);
  digitalWrite(SONAR_CHAIN_PIN, LOW);
  //delay(49);
  sonarReading[1] = (int) (analogRead(FRONT_SONAR) * 512.0 /1023 * 2.54);//接收到触发脉冲的sonar读数看起来有问题 A5 对应声呐看起来有问题
  //delay(49);
  sonarReading[0] =  (int) (analogRead(SECOND_SONAR) * 512.0 /1023 * 2.54);  //A4
  //return sonarReading;
}
