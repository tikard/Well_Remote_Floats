/*      WELL REMOTE FLOAT RECEIVER MODULE
  Uses LORA to get float msgs from a remote float sender unit
  

  REV 0.0  Initial Version   5/3/21

  5/4/22  Working version:   added debug pin to enable using forced inputs
                            enable debug pin 21 HIGH
                            then apply high +3.3 v to
                            Pin 32 Empty to simulate empty
                            Pin 33 Full to simulate full

    FULL WORKING VERSION   tested with LORA REMOTE sending and Debug

  5/17/22 Added Main Delay Loop every 10 seconds
          Added last fill time to display in seconds

*/

#include <Arduino.h>
#include "heltec.h"
#include "images.h"
#include "HardwareSerial.h"
#include "ArduinoJson-v6.18.5.h"
#include <U8x8lib.h>

#ifdef U8X8_HAVE_HW_SPI
#include <SPI.h>
#endif
#ifdef U8X8_HAVE_HW_I2C
#include <Wire.h>
#endif

const int WEB_RADIO_ID = 101;

// OLED screen setup
U8X8_SSD1306_128X64_NONAME_SW_I2C u8x8(/* clock=*/ 15, /* data=*/ 4, /* reset=*/ 16);

// HELTEC LORA 
#define BAND    915E6  //you can set band here directly,e.g. 868E6,915E6

#define debugMode true

String rssi = "RSSI --";
String packSize = "--";
String packet;

unsigned int counter = 0;

bool receiveflag = false;   // software flag for LoRa receiver, received data makes it true.

uint64_t chipid;

bool responseScreenSent = false;

//************************************ Well Stuff **************************************************  
bool debug = false;

enum valveStates { OPEN=0, CLOSED=1}; 

valveStates ValveStatus = OPEN;

bool powerDetected = false;
int powerThreshold = 3000;  // set to value where voltage reg is detected from dongel

const int valveOpenPin = 12;
const int powerDetectedPin = 13;

const int fakeEmptyPin = 32;  // used to fake empty
const int fakeFullPin  = 33;  // used to fake full

const int debugPin  = 21;  // used to fake full

int powerDetectedValue = 0;

int radioID   = 0;
int wellFULL  = 0;
int wellEMPTY = 0;


unsigned long currentMillis = 0;

const long MAININTERVAL      = 10000;   // MAIN DELAY counter
const long ONESECONDINTERVAL = 1000;   // one second counter
const long TWOSECONDINTERVAL = 3000;   // two second counter
const long DEFAULTINTERVAL   = 5000;   // Default Interval



long FillTime        = 0;   // Fill Time Counter
long FillStartTime   = 0;   // Fill Time Counter
long FillStopTime    = 0;   // Fill Time Counter

long previousMillis = 0;        // will store last time LED was updated

StaticJsonDocument<512> doc;

// Helpers to send output to right port
void debugPrint(String s){ Serial.print(s);}
void debugPrint(int s){ Serial.print(s);}
void debugPrintln(String s){ Serial.println(s);}
void debugPrintln(int s){ Serial.println(s);}

// OLED Functions
void pre(void)
{
  u8x8.setFont(u8x8_font_chroma48medium8_r); 
  u8x8.clear();
  u8x8.setCursor(0,0);
}

void draw_ascii_row(uint8_t r, int start)
{
  int a;
  uint8_t c;
  for( c = 0; c < u8x8.getCols(); c++ )
  {
    u8x8.setCursor(c,r);
    a = start + c;
    if ( a <= 255 )
      u8x8.write(a);
  }
}

void idleScreen(){
  pre();
  //u8x8.setFont(u8x8_font_px437wyse700b_2x2_r);
  u8x8.setFont(u8x8_font_chroma48medium8_r);
  u8x8.println("ID = " + String(radioID));

  u8x8.println("");
  if(wellEMPTY == 1)
    u8x8.println("EMPTY = TRUE");
  else
    u8x8.println("EMPTY = FALSE");

  if(wellFULL == 1)
    u8x8.println("FULL  = TRUE");
  else
    u8x8.println("FULL  = FALSE");

  //u8x8.println("");
  if(powerDetected)
    u8x8.println("POWER");
  else
    u8x8.println("NO POWER");

  //FillStopTime = 14400000;
  //FillStartTime = 1000;

  if(FillStopTime > FillStartTime){
    FillTime =  FillStopTime - FillStartTime;
    FillTime = (FillTime/1000)/60;
    u8x8.println("");
    u8x8.println("FT Mins  " + String(FillTime));
  }else
  {
    u8x8.println("");
    u8x8.println("FT Unknown");
  }



  //if(debug)
  //  u8x8.println("DEBUG Mode");
  //else
  //  u8x8.println("");



  //u8x8.println("");
  u8x8.setFont(u8x8_font_chroma48medium8_r);
  u8x8.println("Last " + rssi);
}

void receiveLORAData(String payload){
  Serial.println("payload from relay: " + payload);

  DeserializationError error = deserializeJson(doc, payload);
  if (error) {
    Serial.println(error.c_str()); 
    return;
  }

  // store data in globals
  radioID   = doc["RID"];
  wellFULL  = doc["WF"];
  wellEMPTY = doc["WE"];

  //Serial.println("Radio ID is " + radioID);
}


// *************************************************** LORA Functions

void onLORAReceive(int packetSize)  //LoRa receiver interrupt service
{
  packet = "";
  packSize = String(packetSize,DEC);

  while (LoRa.available()){
    packet += (char) LoRa.read();
  }

  rssi = "RSSI: " + String(LoRa.packetRssi(), DEC);    
  receiveflag = true;    
}

void valveClose(bool close ){
  if(close){
    if(ValveStatus == CLOSED){
      // ALREADY CLOSED
      return;
    }

    digitalWrite(valveOpenPin, HIGH);  // close valve to divert flow
    ValveStatus = CLOSED;
    debugPrintln("Close Valve");
    FillStartTime = millis();  // Save start time in milliseconds
    return;
    
  }
  else{

    if(ValveStatus == OPEN){
    // ALREADY OPEN
    return;
    }

    digitalWrite(valveOpenPin, LOW);  // open valve normal mode bypass
    ValveStatus = OPEN;
    FillStopTime = millis();  // Save start time in milliseconds
    debugPrintln("Open Valve");
  }    
}

// *****************************************   1 time SETUP CODE  ************************************
void setup() {
  Serial.begin(115200);
  currentMillis = millis();
  FillStartTime   = 0;
  FillStopTime = 0;

  u8x8.begin();
  pinMode(powerDetectedPin,INPUT_PULLDOWN);  // pull down to zero
  pinMode(valveOpenPin,OUTPUT);      // pull down to zero

  pinMode(fakeEmptyPin,INPUT_PULLDOWN);  // pull down to zero 
  pinMode(fakeFullPin,INPUT_PULLDOWN);  // pull down to zero 
  
  pinMode(debugPin, INPUT_PULLDOWN);

  debugPrintln("Well Remote Float Receiver - Testing");

  // Setup the LORA board for communications
  Heltec.begin(false /*DisplayEnable Enable*/, true /*LoRa Enable*/, true /*Serial Enable*/, true /*LoRa use PABOOST*/, BAND /*LoRa RF working band*/);
  idleScreen();
	delay(300);

  LoRa.onReceive(onLORAReceive);
  LoRa.receive();
  Heltec.LoRa.setTxPowerMax(20);
  //LoRa.setTxPower(20,RF_PACONFIG_PASELECT_PABOOST); //20dB output must via PABOOST

}

void loop() {
  
  currentMillis = millis();

  if(receiveflag){  // Process any data
    receiveLORAData(packet);  // got data over LORA from someone
    receiveflag = false;
    LoRa.receive();
    delay(50);
  }else{
  
  }

  // add in IO pin to detect power on
  powerDetectedValue = analogRead(powerDetectedPin);
  debugPrintln("Power Detected Value = " + String(powerDetectedValue));
  if(powerDetectedValue > powerThreshold)
    powerDetected = true;
  else
    powerDetected = false;


  debug = digitalRead(debugPin);

  if(debug){
    wellEMPTY = digitalRead(fakeEmptyPin);
    wellFULL  = digitalRead(fakeFullPin);   
    debugPrintln("Fake Well Empty = " + String(wellEMPTY));
    debugPrintln("Fake Well Full  = " + String(wellFULL));
  }

  
  // valve login
  if(powerDetected){  // only close valve if power available

    debugPrintln("Power Detected");

    if(wellEMPTY)  // empty 
      {
          valveClose(true);  // close valve
      }
    else if (!wellEMPTY && !wellFULL )  // Well full
      {
        debugPrintln("Not Empty and Not Full");
      }
    else if (wellFULL)  // Well full
      {
        valveClose(false);  // open valve backup and reset
      } 
  }else{
    // lost power
    debugPrintln("Power LOST");
    valveClose(false);  // open valve backup and reset  
  }

  // main delay and screen update
  if(long(currentMillis - previousMillis) > MAININTERVAL) {
    previousMillis = currentMillis;  
    
    u8x8.begin();  // fixes a bug in display lib
    idleScreen();

    LoRa.receive();
  }

  // main delay
  delay(ONESECONDINTERVAL);

}
