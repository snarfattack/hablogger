#include <Arduino.h>
#include <Wire.h>
#include <math.h>
#include <stdio.h>
#include <avr/dtostrf.h>
#include <MemoryFree.h>
#include <ZeroAPRS.h>                       //https://github.com/hakkican/ZeroAPRS
#include <SparkFun_Ublox_Arduino_Library.h> //https://github.com/sparkfun/SparkFun_Ublox_Arduino_Library
#include <Adafruit_BMP085.h>                //https://github.com/adafruit/Adafruit-BMP085-Library
#include <Adafruit_BME280.h>

#define BattPin       A5
#define GpsPwr        7
#define PwDwPin       A3
#define PowerHL       A4
#define PttPin        3
#define BmePwr        A1
#define SEALEVELPRESSURE_HPA (1013.25)

//macros
#define GpsON       digitalWrite(GpsPwr, LOW)
#define GpsOFF      digitalWrite(GpsPwr, HIGH)
#define PttON       digitalWrite(PttPin, HIGH)
#define PttOFF      digitalWrite(PttPin, LOW)
#define RadioON     digitalWrite(PwDwPin, HIGH)
#define RadioOFF    digitalWrite(PwDwPin, LOW)
#define RfHiPwr     digitalWrite(PowerHL, HIGH)
#define RfLowPwr    digitalWrite(PowerHL, LOW)
#define BmeON       digitalWrite(BmePwr, HIGH)
#define BmeOFF      digitalWrite(BmePwr, LOW)

//#define DEVMODE // Development mode. Uncomment to enable for debugging.

//******************************  APRS CONFIG **********************************
char    CallSign[7]="CALLSIGN"; //DO NOT FORGET TO CHANGE YOUR CALLSIGN
int8_t  CallNumber=11;//SSID http://www.aprs.org/aprs11/SSIDs.txt
char    Symbol='O'; // 'O' for balloon, '>' for car, for more info : http://www.aprs.org/symbols/symbols-new.txt
bool    alternateSymbolTable = false ; //false = '/' , true = '\'

char Frequency[9]="144.3900"; //default frequency. 144.3900 for US, 144.8000 for Europe
char FoxhuntFreq[9]="145.3000"; //frequency to start transmitting on after we descend for foxhunting
static uint16_t foxhuntAlt=1500; //altitude in ft to turn on the foxhunting beacon on descent
uint16_t flyAlt=3000; //altitude in ft we must exceed to trigger foxhunt mode on descent
bool flyAltReached = false; //did we exceed the flyAlt or not?

char    comment[50] = "LightAPRS 2.0"; // Max 50 char but shorter is better
char    StatusMessage[50] = "LightAPRS 2.0 by TA2NHP & TA2MUN"; // Status message to send after we aquire GPS lock
//*****************************************************************************

uint16_t  BeaconWait=50;  //seconds sleep for next beacon (HF or VHF). This is optimized value, do not change this if possible.
//uint16_t  BattWait=60;    //seconds sleep if super capacitors/batteries are below BattMin (important if power source is solar panel) 
//float     BattMin=3.3;    // min Volts to wake up.
//float     GpsMinVolt=4.5; //min Volts for GPS to wake up. (important if power source is solar panel) 
float     DraHighVolt=5.0;    // min Volts for radio module (DRA818V) to transmit (TX) 1 Watt, below this transmit 0.5 Watt.

//******************************  APRS SETTINGS *********************************

//do not change WIDE path settings below if you don't know what you are doing :) 
uint8_t   Wide1=1; // 1 for WIDE1-1 path
uint8_t   Wide2=1; // 1 for WIDE2-1 path

/**
Airborne stations above a few thousand feet should ideally use NO path at all, or at the maximum just WIDE2-1 alone.  
Due to their extended transmit range due to elevation, multiple digipeater hops are not required by airborne stations.  
Multi-hop paths just add needless congestion on the shared APRS channel in areas hundreds of miles away from the aircraft's own location.  
NEVER use WIDE1-1 in an airborne path, since this can potentially trigger hundreds of home stations simultaneously over a radius of 150-200 miles. 
 */
uint8_t pathSize=1; // 2 for WIDE1-N,WIDE2-N ; 1 for WIDE2-N
boolean autoPathSizeHighAlt = true; //force path to WIDE2-N only for high altitude (airborne) beaconing (over 1.000 meters (3.280 feet)) 
boolean gpsLock = false; //Keep track if we have a valid GPS lock or not
unsigned long aliveBeacon = 70000;
static uint8_t BeaconSecs = 20;  //time between status message beacons when we have no GPS lock
boolean radioSetup = false; //do not change this, temp value
static char telemetry_buff[200];// telemetry buffer
uint16_t TxCount = 1; //increase +1 after every APRS location transmission
uint16_t SxCount = 1; //increase +1 after every APRS status transmission
//uint16_t FxCount = 1; //increase +1 after every second spent transmitting foxhunt signal
float tempAltitude = 0; //store the current loop altitude for calculating foxhunting

//******************************  GPS SETTINGS   *********************************
int16_t   GpsResetTime=1800; // timeout for reset if GPS is not fixed
boolean ublox_high_alt_mode_enabled = false; //do not change this
boolean gpsSetup=false; //do not change this.

//********************************************************************************

SFE_UBLOX_GPS myGPS;
Adafruit_BMP085 bmp;
Adafruit_BME280 bme; 
int bmeAddress = 0x76;

void setup() {
  // While the energy rises slowly with the solar panel, 
  // using the analog reference low solves the analog measurement errors.
  analogReference(AR_INTERNAL1V65);
  pinMode(PttPin, OUTPUT);
  pinMode(GpsPwr, OUTPUT);
  pinMode(BattPin, INPUT);
  pinMode(PwDwPin, OUTPUT);
  pinMode(PowerHL, OUTPUT);
  pinMode(BmePwr, OUTPUT);
  
  GpsOFF;
  PttOFF;
  RadioOFF; 
  RfLowPwr;
  BmeON;

  SerialUSB.begin(115200);
  // Wait up to 5 seconds for serial to be opened, to allow catching
  // startup messages on native USB boards (that do not reset when
  // serial is opened).
  //Watchdog.reset();  
  unsigned long start = millis();
  while (millis() - start < 5000 && !SerialUSB){;}
  //Watchdog.reset(); 

  SerialUSB.println(F("Starting"));
  Serial1.begin(9600);// for DorjiDRA818V

  APRS_init();
  APRS_setCallsign(CallSign, CallNumber);
  char destination[] = "APLIGA";
  APRS_setDestination(destination, 0);
  char wide1[] = "WIDE1";
  APRS_setPath1(wide1, Wide1);
  char wide2[] = "WIDE2";
  APRS_setPath2(wide2, Wide2);
  APRS_setPathSize(2);
  APRS_useAlternateSymbolTable(alternateSymbolTable);
  APRS_setSymbol(Symbol);
  APRS_setPathSize(pathSize);
  APRS_setGain(2);

  RadioON;
  delay(2000);
  configDra818(Frequency);
  delay(2000);
  RadioOFF;

  Wire.begin();
  bmp.begin();
  bme.begin(bmeAddress);

  SerialUSB.println(F(""));
  SerialUSB.print(F("APRS (VHF) CallSign: "));
  SerialUSB.print(CallSign);
  SerialUSB.print(F("-"));
  SerialUSB.println(CallNumber);

}

void loop() {
  if(!gpsSetup) {gpsStart();}
  if(!ublox_high_alt_mode_enabled){setupUBloxDynamicModel();}

  if (myGPS.getPVT()) {
    gpsDebug();
    if ( (myGPS.getFixType() != 0) && (myGPS.getSIV() > 2) ) {
      if (!gpsLock) {
        sendStatus(StatusMessage);	
        gpsLock = true;
      }
      aliveBeacon = millis();
      updatePosition();
      updateTelemetry();
      tempAltitude = (myGPS.getAltitude() * 3.2808399)  / 1000.f;
      if(!flyAltReached && tempAltitude > flyAlt && (bme.readAltitude(SEALEVELPRESSURE_HPA) * 3.2808399) > flyAlt)
      { 
        //Make sure GPS and BME pressure agree we flew above the target altitude. If not we can trigger false flights during initial lock.
        flyAltReached = true;
      }

      if(autoPathSizeHighAlt && (tempAltitude > 3000)) {
        //force to use high altitude settings (WIDE2-n)
        APRS_setPathSize(1);
        } else {
        //use default settings  
        APRS_setPathSize(pathSize);
      }

      sendLocation();
      freeMem();
      SerialUSB.flush();
      if(flyAltReached && tempAltitude < foxhuntAlt && (bme.readAltitude(SEALEVELPRESSURE_HPA) * 3.2808399) < foxhuntAlt) {
#if defined(DEVMODE)  
        SerialUSB.println(F("Sending FoxHunt"));
#endif 
        sendFoxhunt(BeaconWait);
      }
      else {
#if defined(DEVMODE)  
        SerialUSB.println(F("Sleeping after location sent"));
#endif         
        sleepSeconds(BeaconWait);       
      }
    } else {
      noGpsLockStatus();
    }
  } else {
    noGpsLockStatus();
  }
}

void noGpsLockStatus() {
  gpsLock = false;
  float pressure = bme.readPressure();
  if(pressure<0){  // if pressure falls below 0, something is wrong, attempt to reinit the BME
#if defined(DEVMODE)  
    SerialUSB.println(F("Pressure below 0, reinit BME and try again"));
#endif 
    BmeOFF;
    delay(100);
    BmeON;
    bme.begin(bmeAddress);
    pressure = bme.readPressure();
  }  
  if (millis() - aliveBeacon > BeaconSecs * 1000) {	
    char msg[100];

    sprintf(msg, "No GPS Lock: Bat %4.2fv, Time %d-%d-%d %d:%d:%d, %4.0fM, %6.0fPa", readBatt(), myGPS.getYear(), myGPS.getMonth(), myGPS.getDay(), myGPS.getHour(), myGPS.getMinute(), myGPS.getSecond(), myGPS.getAltitude(), pressure);
    sendStatus(msg);	   	  
    aliveBeacon = millis();
  }
  if(flyAltReached && tempAltitude < foxhuntAlt && (bme.readAltitude(SEALEVELPRESSURE_HPA) * 3.2808399) < foxhuntAlt) {
    sendFoxhunt(BeaconSecs);
  }  
}

void gpsStart() {  
  bool gpsBegin=false;  
  while(!gpsBegin){
    GpsON;
    delay(1000);
    Wire.begin();
    gpsBegin=myGPS.begin();
    if(gpsBegin)break;
#if defined(DEVMODE)  
    SerialUSB.println(F("Ublox GPS not detected at default I2C address. Will try again"));
#endif 
    delay(2000);
  }
   // do not overload the buffer system from the GPS, disable UART output
  myGPS.setUART1Output(0); //Disable the UART1 port output 
  myGPS.setUART2Output(0); //Disable Set the UART2 port output
  myGPS.setI2COutput(COM_TYPE_UBX); //Set the I2C port to output UBX only (turn off NMEA noise)
  //myGPS.setNavigationFrequency(1);
  //myGPS.setAutoPVT(true, true);
  myGPS.saveConfiguration(); //Save the current settings to flash and BBR  
  gpsSetup=true;
}

void sleepSeconds(int sec) {
  PttOFF;
  RadioOFF;

  SerialUSB.flush();
  for (int i = 0; i < sec; i++) {
    delay(1000);   
  }

}

byte configDra818(char *freq)
{

  char ack[3];
  int n;
  char cmd[50];//"AT+DMOSETGROUP=0,144.8000,144.8000,0000,4,0000"
  sprintf(cmd, "AT+DMOSETGROUP=0,%s,%s,0000,4,0000", freq, freq);
  Serial1.println(cmd);
#if defined(DEVMODE)  
  SerialUSB.println("RF Config");
#endif 
  ack[2] = 0;
  unsigned long start = millis();
  while (ack[2] != 0xa)
  {
    if (Serial1.available() > 0) {
      ack[0] = ack[1];
      ack[1] = ack[2];
      ack[2] = Serial1.read();
    }
    if (millis() - start > 10000) { RadioOFF; break; } //If radio doesn't ack in 10 seconds, shut it off.
  }

  if (ack[0] == 0x30) {
      SerialUSB.print(F("Frequency updated: "));
      SerialUSB.print(freq);
      SerialUSB.println(F("MHz"));
    } else {
      SerialUSB.println(F("Frequency update error!!!"));    
    }
  return (ack[0] == 0x30) ? 1 : 0;
}

void updatePosition() {
  // Convert and set latitude NMEA string Degree Minute Hundreths of minutes ddmm.hh[S,N].
  char latStr[10];
  int temp = 0;
  double d_lat = myGPS.getLatitude() / 10000000.f;
  double dm_lat = 0.0;

  if (d_lat < 0.0) {
    temp = -(int)d_lat;
    dm_lat = temp * 100.0 - (d_lat + temp) * 60.0;
  } else {
    temp = (int)d_lat;
    dm_lat = temp * 100 + (d_lat - temp) * 60.0;
  }

  dtostrf(dm_lat, 7, 2, latStr);

  if (dm_lat < 1000) {
    latStr[0] = '0';
  }

  if (d_lat >= 0.0) {
    latStr[7] = 'N';
  } else {
    latStr[7] = 'S';
  }

  APRS_setLat(latStr);

  // Convert and set longitude NMEA string Degree Minute Hundreths of minutes ddmm.hh[E,W].
  char lonStr[10];
  double d_lon = myGPS.getLongitude() / 10000000.f;
  double dm_lon = 0.0;

  if (d_lon < 0.0) {
    temp = -(int)d_lon;
    dm_lon = temp * 100.0 - (d_lon + temp) * 60.0;
  } else {
    temp = (int)d_lon;
    dm_lon = temp * 100 + (d_lon - temp) * 60.0;
  }

  dtostrf(dm_lon, 8, 2, lonStr);

  if (dm_lon < 10000) {
    lonStr[0] = '0';
  }
  if (dm_lon < 1000) {
    lonStr[1] = '0';
  }

  if (d_lon >= 0.0) {
    lonStr[8] = 'E';
  } else {
    lonStr[8] = 'W';
  }

  APRS_setLon(lonStr);
  APRS_setTimeStamp(myGPS.getHour(), myGPS.getMinute(), myGPS.getSecond());
}

void updateTelemetry() {
  sprintf(telemetry_buff, "%03d", (int)(myGPS.getHeading() / 100000));
  telemetry_buff[3] += '/';
  sprintf(telemetry_buff + 4, "%03d", (int)(myGPS.getGroundSpeed() * 0.00194384f));
  telemetry_buff[7] = '/';
  telemetry_buff[8] = 'A';
  telemetry_buff[9] = '=';
  //fixing negative altitude values causing display bug on aprs.fi
  tempAltitude = (myGPS.getAltitude() * 3.2808399)  / 1000.f;

  if (tempAltitude > 0) {
    //for positive values
    sprintf(telemetry_buff + 10, "%06lu", (long)tempAltitude);
  } else {
    //for negative values
    sprintf(telemetry_buff + 10, "%06d", (long)tempAltitude);
  }
  telemetry_buff[16] = ' ';
  sprintf(telemetry_buff + 17, "%03d", TxCount);
  telemetry_buff[20] = 'T';
  telemetry_buff[21] = 'x';
  telemetry_buff[22] = 'C';
  telemetry_buff[23] = ' '; float tempC = bmp.readTemperature();
  dtostrf(tempC, 6, 2, telemetry_buff + 24);
  telemetry_buff[30] = 'C';
  telemetry_buff[31] = ' '; float pressure = bmp.readPressure() / 100.0; //Pa to hPa
  dtostrf(pressure, 7, 2, telemetry_buff + 32);
  telemetry_buff[39] = 'h';
  telemetry_buff[40] = 'P';
  telemetry_buff[41] = 'a';
  telemetry_buff[42] = ' ';
  dtostrf(readBatt(), 5, 2, telemetry_buff + 43);
  telemetry_buff[48] = 'V';
  telemetry_buff[49] = ' ';
  sprintf(telemetry_buff + 50, "%02d", (int)myGPS.getSIV()); //Returns number of sats used in fix
  telemetry_buff[52] = 'S';
  telemetry_buff[53] = ' ';
  tempC = bme.readTemperature();
  dtostrf(tempC, 6, 2, telemetry_buff + 54);
  telemetry_buff[60] = 'C';
  telemetry_buff[61] = ' '; 
  pressure = bme.readPressure(); // / 100.0; //Pa to hPa
  dtostrf(pressure, 8, 1, telemetry_buff + 62);
  //telemetry_buff[69] = 'h';
  telemetry_buff[70] = 'P';
  telemetry_buff[71] = 'a';
  telemetry_buff[72] = ' ';
  float humd = bme.readHumidity();
  dtostrf(humd, 5, 2, telemetry_buff + 73);
  telemetry_buff[78] = '%';
  //telemetry_buff[79] = ' '; 
  if(isnan(humd) || pressure < 0){
    BmeOFF;
    delay(100);
    BmeON;
    bme.begin(bmeAddress);
  }

  //sprintf(telemetry_buff + 80, "%s", comment);

#if defined(DEVMODE)
  SerialUSB.println(telemetry_buff);
#endif
}

void sendLocation() {

#if defined(DEVMODE)
  SerialUSB.println(F("Location sending with comment..."));
#endif
  if (readBatt() > DraHighVolt) {
    RfHiPwr; //DRA Power 1 Watt
#if defined(DEVMODE)
    SerialUSB.println(F("1 Watt"));
#endif
  } else {
    RfLowPwr; //DRA Power 0.5 Watt
#if defined(DEVMODE)
    SerialUSB.print(readBatt());
    SerialUSB.println(F(" -> .5 Watt"));
#endif
  }
  RadioON;
  delay(2000);
  PttON;
  delay(1000);  
  APRS_sendLoc(telemetry_buff);
  delay(10);
  PttOFF;
  RadioOFF;
  delay(1000);
#if defined(DEVMODE)
  SerialUSB.print(F("Location sent with comment - "));
  SerialUSB.println(TxCount);
#endif 
  TxCount++;
}

void sendStatus(char *msg) {

#if defined(DEVMODE)
  SerialUSB.println(F("Status sending..."));
#endif
  if (readBatt() > DraHighVolt) {
    RfHiPwr; //DRA Power 1 Watt
#if defined(DEVMODE)
      SerialUSB.println(F("1 Watt"));
#endif
  } else {
    RfLowPwr; //DRA Power 0.5 Watt
#if defined(DEVMODE)
    SerialUSB.print(readBatt());
    SerialUSB.println(F(" -> .5 Watt"));
#endif
  }
  char status[105];
  sprintf(status, "%dSxC ", SxCount);
  strcat(status, msg);

#if defined(DEVMODE)
    SerialUSB.println(status);
#endif
  RadioON;
  delay(2000);
  PttON;
  delay(1000); 
  APRS_sendStatus(status);
  delay(10);
  PttOFF;
  RadioOFF;
  delay(1000);
#if defined(DEVMODE)
  SerialUSB.print(F("Status sent - "));
  SerialUSB.println(SxCount);
#endif
  SxCount++;
}

void sendFoxhunt(int secDuration) {

  RfHiPwr; //DRA Power 1 Watt
#if defined(DEVMODE)
  SerialUSB.println(F("FoxHunt Sending..."));
  RfLowPwr; //DRA Power .5 Watt for testing
#endif
  unsigned long start = millis();
  
  RadioON;
  delay(2000);
  if (configDra818(FoxhuntFreq))
  {
    delay(500);
    PttON;
    delay(1000);
    while (millis() - start < secDuration * 1000) {
      APRS_sendLoc(telemetry_buff);
      delay(10);
    }
    PttOFF;
    configDra818(Frequency);
    delay(500);
  }
  else
  {
    SerialUSB.println(F("FoxHunt Failed"));
    sleepSeconds(secDuration);
  }
  RadioOFF;
  //delay(1000);
#if defined(DEVMODE)
  SerialUSB.print(F("FoxHunt Paused - "));
//  SerialUSB.println(FxCount);
#endif
//  FxCount += secDuration;
}

void gpsDebug() { 
#if defined(DEVMODE)
    byte fixType = myGPS.getFixType();
    SerialUSB.print(F("FixType: "));
    SerialUSB.print(fixType);    

    int SIV = myGPS.getSIV();
    SerialUSB.print(F(" Sats: "));
    SerialUSB.print(SIV);

    float flat = myGPS.getLatitude() / 10000000.f;    
    SerialUSB.print(F(" Lat: "));
    SerialUSB.print(flat);    

    float flong = myGPS.getLongitude() / 10000000.f;    
    SerialUSB.print(F(" Long: "));
    SerialUSB.print(flong);        

    float altitude = myGPS.getAltitude() / 1000;
    SerialUSB.print(F(" Alt: "));
    SerialUSB.print(altitude);
    SerialUSB.print(F(" (m)"));

    float speed = myGPS.getGroundSpeed();
    SerialUSB.print(F(" Speed: "));
    SerialUSB.print(speed * 0.00194384f);
    SerialUSB.print(F(" (kn/h)"));    
        
    SerialUSB.print(" Time: ");    
    SerialUSB.print(myGPS.getYear());
    SerialUSB.print("-");
    SerialUSB.print(myGPS.getMonth());
    SerialUSB.print("-");
    SerialUSB.print(myGPS.getDay());
    SerialUSB.print(" ");
    SerialUSB.print(myGPS.getHour());
    SerialUSB.print(":");
    SerialUSB.print(myGPS.getMinute());
    SerialUSB.print(":");
    SerialUSB.print(myGPS.getSecond());
    
    SerialUSB.print(" Temp: ");
    SerialUSB.print(bmp.readTemperature());
    SerialUSB.print("C");
    
    SerialUSB.print(" Press: ");    
    SerialUSB.print(bmp.readPressure() / 100.0);
    SerialUSB.print("hPa");

    float pressure = bme.readPressure();
    if(pressure<0){  // if pressure falls below 0, something is wrong, attempt to reinit the BME
#if defined(DEVMODE)  
      SerialUSB.println(F("Pressure below 0, reinit BME and try again"));
#endif 
      BmeOFF;
      delay(100);
      BmeON;
      bme.begin(bmeAddress);
      pressure = bme.readPressure();
    }

    SerialUSB.print(" BME Temp: ");
    SerialUSB.print(bme.readTemperature());
    SerialUSB.print("C");
    
    SerialUSB.print(" BME Press: ");    
    SerialUSB.print(pressure);
    SerialUSB.print("Pa");

    SerialUSB.print(" BME Humi: ");    
    SerialUSB.print(bme.readHumidity());
    SerialUSB.print("%");
    SerialUSB.println();  

#endif
}

void setupUBloxDynamicModel() {
    // If we are going to change the dynamic platform model, let's do it here.
    // Possible values are:
    // PORTABLE, STATIONARY, PEDESTRIAN, AUTOMOTIVE, SEA, AIRBORNE1g, AIRBORNE2g, AIRBORNE4g, WRIST, BIKE
    // DYN_MODEL_AIRBORNE4g model increases ublox max. altitude limit from 12.000 meters to 50.000 meters. 
    if (myGPS.setDynamicModel(DYN_MODEL_AIRBORNE4g) == false) // Set the dynamic model to DYN_MODEL_AIRBORNE4g
    {
      #if defined(DEVMODE)
        SerialUSB.println(F("***!!! Warning: setDynamicModel failed !!!***"));
      #endif 
    }
    else
    {
      ublox_high_alt_mode_enabled = true;
      #if defined(DEVMODE)
        SerialUSB.print(F("Dynamic platform model changed successfully! : "));
        SerialUSB.println(myGPS.getDynamicModel());
      #endif  
    }
  
  } 

float readBatt() {

  float R1 = 560000.0; // 560K
  float R2 = 100000.0; // 100K
  float value = 0.0f;

  do {    
    value =analogRead(BattPin);
    value +=analogRead(BattPin);
    value +=analogRead(BattPin);
    value = value / 3.0f;
    value = (value * 1.65) / 1024.0f;
    value = value / (R2/(R1+R2));
  } while (value > 20.0);
  return value ;

}

void freeMem() {
#if defined(DEVMODE)
  SerialUSB.print(F("Free RAM: ")); SerialUSB.print(freeMemory(), DEC); SerialUSB.println(F(" byte"));
#endif

}
