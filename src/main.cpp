#include <TinyGPS++.h>
#include <SoftwareSerial.h>
#include <Adafruit_FONA.h>

//Defines for the SIM800L module
#define FONA_RX 2
#define FONA_TX 3
#define FONA_RST 4

//Defines for the BN-220 module
#define RXPin 14
#define TXPin 15
#define GPSBaud 9600

#define GPSTIMEOUT 30000
char url[] = "https://servae.hopto.org:1881/bikefinder";

TinyGPSPlus gps;
Adafruit_FONA fona = Adafruit_FONA(FONA_RST);

// The serial connection to the GPS and GPRS device
SoftwareSerial gpsSoftSerial(RXPin, TXPin);
SoftwareSerial fonaSoftSerial(FONA_TX, FONA_RX);

// This custom version of delay() ensures that the gps object
// is being "fed".
void smartDelay(unsigned long ms){
  unsigned long start = millis();
  do
  {
    while (gpsSoftSerial.available())
    gps.encode(gpsSoftSerial.read());
  } while (millis() - start < ms);
}


void refreshGPSData(){
  gpsSoftSerial.begin(GPSBaud);
  short timeout = 0;
  Serial.print(F("Waiting for valid GPD data"));
  while (!gps.location.isValid() && timeout < GPSTIMEOUT) {
    smartDelay(1000);
    timeout += 1000;
    Serial.print(".");
  }
  Serial.println("");
  if(timeout >= GPSTIMEOUT) Serial.println("No GPS fix. Giving up.");
  else{
    Serial.println("GPS fix found. New data available.");
    Serial.println(gps.location.lat(), 6);
    Serial.println(gps.location.lng(), 6);
  }
  gpsSoftSerial.end();
}

void reset(){
  fona.enableGPRS(false);
}

void initGPRS(){
  while(fona.getNetworkStatus() != 1){
    Serial.println("Connecting to Network");
    delay(3000);
  }
  Serial.println("Connected to Network");
  delay(3000);
  while (!fona.enableGPRS(true)){
    Serial.println(F("Failed to turn on GPRS. Try again..."));
    delay(3000);
  }
  Serial.println("GPRS activation succesful");
}

void sendValue(){
  fonaSoftSerial.begin(4800);
  if (! fona.begin(fonaSoftSerial)) {
    Serial.println(F("Couldn't find FONA"));
    while (1);
  }

  initGPRS();

  uint16_t statuscode;
  int16_t length;
  char tmpLat[15];
  char tmpLng[15];
  sprintf(tmpLat, "%f", gps.location.lat());
  sprintf(tmpLng, "%f", gps.location.lng());
  String combinedUrl = String(url) + "?setLat=" + String(gps.location.lat(), 6) + "&setLon=" + String(gps.location.lng(), 6);
  Serial.println(combinedUrl);
  if (!fona.HTTP_GET_start(combinedUrl.c_str(), &statuscode, (uint16_t *)&length)) {
    Serial.println("Failed!");
    while (length > 0) {
      while (fona.available()) {
        char c = fona.read();
        Serial.write(c);
        length--;
        if (! length) break;
      }
    }
    fona.HTTP_GET_end();
  }
}


void setup()
{
  Serial.begin(9600);
  refreshGPSData();
  sendValue();

}

void loop()
{
  /*
  printInt(gps.satellites.value(), gps.satellites.isValid(), 5);
  printFloat(gps.hdop.hdop(), gps.hdop.isValid(), 6, 1);
  printFloat(gps.location.lat(), gps.location.isValid(), 11, 6);
  printFloat(gps.location.lng(), gps.location.isValid(), 12, 6);
  printInt(gps.location.age(), gps.location.isValid(), 5);
  printDateTime(gps.date, gps.time);
  printFloat(gps.altitude.meters(), gps.altitude.isValid(), 7, 2);
  printFloat(gps.course.deg(), gps.course.isValid(), 7, 2);
  printFloat(gps.speed.kmph(), gps.speed.isValid(), 6, 2);
  printStr(gps.course.isValid() ? TinyGPSPlus::cardinal(gps.course.deg()) : "*** ", 6);
  */
}
