
// Server End Code (Fully Working)


#include <SPI.h>
#include <nRF24L01.h>
#include <RF24.h>

#include <SoftwareSerial.h>
#include <ArduinoJson.h>
SoftwareSerial s(2,3);

#define button A2
//#define jB1 2  // Joystick 1 PUSH button
//#define jB2 3  // Joystick 2 PUSH button

#define mode1 5   // MODE 1
#define mode2 6   // MODE 2
#define relayA 9   // RELAY 1
#define relayB 10   // RELAY 2
#define resetMega 4   // RESET ARDUINO MEGA

RF24 radio(7, 8); // CE, CSN  PIN OF NRF24L01 MODULE
const byte addresses[][6] = {"00001", "00002"};    // SENDING AND RECEIVING DATA PIPES

int joyposVert = 512;    // INITIALISATION
int joyposHorz = 512;

unsigned long lastReceiveTime = 0;
unsigned long currentTime = 0;

struct Data_Package {     // VARIABLES TO STORE DATA IN ORDER TO ""SEND"" TO FOREST ROVER
 
  byte j1PotX;   // JOYSTICK 1
  byte j1PotY;
  byte j1Button;
 
  byte j2PotX;   // JOYSTICK 2
  byte j2PotY;
  byte j2Button;
 
  byte pot1;     // POTENTIOMETER 1 VALUE
  byte pot2;     // POTENTIOMETER 2 VALUE
 
  byte relay1;   // WIRELESS SWITCH
  byte relay2;
 
  byte resetMEGA; // RESET FOREST RANGER MICROCONTROLLER
 
  byte Mode;      // MODE OF CONTROLLING
 
  byte buttonState;  // TESTING PURPOSE
 
  byte dir;       // FOR JOYSTICK CONTROLLING
  byte leftMotor;
  byte rightMotor;
 
};


Data_Package data; //Create a variable with the above structure

struct Data_PackageRec {     // VARIABLES TO STORE THE DATA ""RECEIVED"" FROM THE FOREST RANGER
 
  byte temperature;
  byte humidity;
 
  byte gasValue;
  byte flameValue;
 
  byte mode;
 
  byte waterValue;
 
  byte rel1;
  byte rel2;
 
  byte pirValue;
 
 };

Data_PackageRec dataRec; //Create a variable with the above structure

void setup() {
  Serial.begin(9600);
  s.begin(9600);
  pinMode(button, INPUT);

//  pinMode(jB1, INPUT);
//  pinMode(jB2, INPUT);
  pinMode(mode1, INPUT);
  pinMode(mode2, INPUT);
  pinMode(relayA, INPUT);
  pinMode(relayB, INPUT);
  //pinMode(b3, INPUT);
  pinMode(resetMega, INPUT);
 
  //myServo.attach(5);
  radio.begin();
  radio.openWritingPipe(addresses[0]); // 00001
  radio.openReadingPipe(1, addresses[1]); // 00002
  radio.setAutoAck(false);
  radio.setDataRate(RF24_250KBPS);
  radio.setPALevel(RF24_PA_LOW);
  //radio.setPALevel(RF24_PA_MIN);

  data.j1PotX = 127;
  data.j1PotY = 127;
  data.j2PotX = 127;
  data.j2PotY = 127;
  data.j1Button = 1;
  data.j2Button = 1;
  data.pot1 = 1;
  data.pot2 = 1;
//  data.tSwitch1 = 1;
//  data.tSwitch2 = 1;
  data.relay1 = 0;
  data.relay2 = 0;
  data.resetMEGA = 1;
  data.Mode = 0;
  data.buttonState = 0;
  data.dir = 0;
  data.leftMotor = 0;
  data.rightMotor = 0;
 
  resetData();
}

void loop() {

  // Read all analog inputs and map them to one Byte value
//  data.j1PotX = map(analogRead(A1), 0, 1023, 0, 255); // Convert the analog read value from 0 to 1023 into a BYTE value from 0 to 255
//  data.j1PotY = map(analogRead(A0), 0, 1023, 0, 255);
//  data.j2PotX = map(analogRead(A2), 0, 1023, 0, 255);
//  data.j2PotY = map(analogRead(A3), 0, 1023, 0, 255);

  joystickControlling();
 
  data.pot1 = map(analogRead(A4), 0, 1023, 0, 255);
  data.pot2 = map(analogRead(A5), 0, 1023, 0, 255);
  // Read all digital inputs
//  data.j1Button = digitalRead(jB1);
//  if(data.j1Button == 1)
//  data.resetMEGA = 0;
//  data.j2Button = digitalRead(jB2);
  int m = digitalRead(mode1);
  int n = digitalRead(mode2);
  if(m == 1 && n == 0)
  data.Mode = 1;
  else if(m == 0 && n == 1)
  data.Mode = 2;
  else
  data.Mode = 0;
  data.relay1 = digitalRead(relayA);
  data.relay2 = digitalRead(relayB);
  //data.button3 = digitalRead(b3);
  int r = digitalRead(resetMega);
  if(r == 1)
  data.resetMEGA = 0;
  else
  data.resetMEGA = 1;
  data.buttonState = digitalRead(button);

  delay(5);
    radio.stopListening();
    //buttonState = digitalRead(button);
    //radio.write(&buttonState, sizeof(buttonState));
    radio.write(&data, sizeof(Data_Package));
 
  delay(5);
  radio.startListening();
  
  StaticJsonBuffer<1000> jsonBuffer;
  JsonObject& root = jsonBuffer.createObject();

  if (radio.available()) {
    radio.read(&dataRec, sizeof(Data_PackageRec)); // Read the whole data and store it into the 'data' structure
    lastReceiveTime = millis(); // At this moment we have received the data
  }
  // Check whether we keep receving data, or we have a connection between the two modules
  currentTime = millis();
  if ( currentTime - lastReceiveTime > 5000 ) { // If current time is more then 5 second since we have recived the last data, that means we have lost connection
    resetData(); // If connection is lost, reset the data. It prevents unwanted behavior, for example if a drone has a throttle up and we lose connection, it can keep flying unless we reset the values
  }
  // Print the data in the Serial Monitor
  Serial.print("Temperature: ");
  Serial.println(dataRec.temperature);
  int z = dataRec.temperature;
  root["Temperature"] = z;
  if(z == 0)
  Serial.println("Reading Success");
  Serial.print("Humidity: ");
  Serial.println(dataRec.humidity);
  root["Humidity"] = dataRec.humidity;
  Serial.print("Gas Value: ");
  Serial.println(dataRec.gasValue);
  root["Gas"] = dataRec.gasValue;
  Serial.print("Flame: ");
  Serial.println(dataRec.flameValue);
  root["Flame"] = dataRec.flameValue;

  root.printTo(s);
  Serial.print("Yes");
 
    delay(5);
    radio.stopListening();
    //buttonState = digitalRead(button);
    //radio.write(&buttonState, sizeof(buttonState));
    radio.write(&data, sizeof(Data_Package));
}

void resetData() {
  // Reset the values when there is no radio connection - Set initial default values
  dataRec.temperature = 0;
  dataRec.humidity = 0;
  dataRec.gasValue = 0;
  dataRec.flameValue = 0;
  dataRec.mode = 0;
  dataRec.waterValue = 0;
  dataRec.rel1 = 0;
  dataRec.rel2 = 0;
  dataRec.pirValue = 0;
//  dataRec.tSwitch2 = 1;
//  dataRec.button1 = 1;
//  dataRec.button2 = 1;
//  dataRec.button3 = 1;
//  dataRec.button4 = 1;
}


void joystickControlling()
{
  // Print to Serial Monitor
  Serial.println("Reading motorcontrol values ");
 
  // Read the Joystick X and Y positions
  joyposVert = analogRead(A0);
  joyposHorz = analogRead(A1);

  // Determine if this is a forward or backward motion
  // Do this by reading the Verticle Value
  // Apply results to MotorSpeed and to Direction

  if (joyposVert < 460)
  {
    // This is Backward
    // Set Motors backward
    data.dir = 1;

    //Determine Motor Speeds
    // As we are going backwards we need to reverse readings
    data.leftMotor = map(joyposVert, 460, 0, 0, 255);
    data.rightMotor = map(joyposVert, 460, 0, 0, 255);

  }
  else if (joyposVert > 564)
  {
    // This is Forward
    // Set Motors forward
    data.dir = 2;

    //Determine Motor Speeds
    data.leftMotor = map(joyposVert, 564, 1023, 0, 255);
    data.rightMotor = map(joyposVert, 564, 1023, 0, 255);

  }
  else
  {
    // This is Stopped
    data.dir = 0;
    data.leftMotor = 0;
    data.rightMotor = 0;

  }
 
  // Now do the steering
  // The Horizontal position will "weigh" the motor speed
  // Values for each motor

  if (joyposHorz < 460)
  {
    // Move Left
    // As we are going left we need to reverse readings
    // Map the number to a value of 255 maximum
    joyposHorz = map(joyposHorz, 460, 0, 0, 255);

    data.leftMotor = data.leftMotor - joyposHorz;
    data.rightMotor = data.rightMotor + joyposHorz;

    // Don't exceed range of 0-255 for motor speeds
    if (data.leftMotor < 0)data.leftMotor = 0;
    if (data.rightMotor > 255)data.rightMotor = 255;

  }
  else if (joyposHorz > 564)
  {
    // Move Right
    // Map the number to a value of 255 maximum
    joyposHorz = map(joyposHorz, 564, 1023, 0, 255);
 
    data.leftMotor = data.leftMotor + joyposHorz;
   data.rightMotor = data.rightMotor - joyposHorz;

    // Don't exceed range of 0-255 for motor speeds
    if (data.leftMotor > 255)data.leftMotor = 255;
    if (data.rightMotor < 0)data.rightMotor = 0;      

  }

  // Adjust to prevent "buzzing" at very low speed
  if (data.leftMotor < 8)data.leftMotor = 0;
  if (data.rightMotor < 8)data.rightMotor = 0;

  //Display the Motor Control values in the serial monitor.
  Serial.print("Motor A: ");
  Serial.print(data.leftMotor);
  Serial.print(" - Motor B: ");
  Serial.print(data.rightMotor);
  Serial.print(" - Direction: ");
  Serial.println(data.dir);
}
