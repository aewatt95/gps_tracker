#include <TinyGPS++.h>
#include <SoftwareSerial.h>
#include <Adafruit_FONA.h>
#include <LowPower.h>

//Defines for the SIM800L module
#define FONA_RX 2
#define FONA_TX 3
#define FONA_RST 4

//Defines for the BN-220 module
#define RXPin 14
#define TXPin 15
#define GPSBaud 9600

#define GPSTIMEOUT 30000

#define GSMPOWERPIN 5
#define GPSPOWERPIN 6
String rootUrl = "https://servae.hopto.org:1881/bikefinder";

TinyGPSPlus gps;
Adafruit_FONA fona = Adafruit_FONA(FONA_RST);

// The serial connection to the GPS and GPRS device
SoftwareSerial gpsSoftSerial(RXPin, TXPin);
SoftwareSerial fonaSoftSerial(FONA_TX, FONA_RX);

//Measure supply voltage
long readVcc() {
  // Read 1.1V reference against AVcc
  // set the reference to Vcc and the measurement to the internal 1.1V reference
  #if defined(__AVR_ATmega32U4__) || defined(__AVR_ATmega1280__) || defined(__AVR_ATmega2560__)
    ADMUX = _BV(REFS0) | _BV(MUX4) | _BV(MUX3) | _BV(MUX2) | _BV(MUX1);
  #elif defined (__AVR_ATtiny24__) || defined(__AVR_ATtiny44__) || defined(__AVR_ATtiny84__)
    ADMUX = _BV(MUX5) | _BV(MUX0);
  #elif defined (__AVR_ATtiny25__) || defined(__AVR_ATtiny45__) || defined(__AVR_ATtiny85__)
    ADMUX = _BV(MUX3) | _BV(MUX2);
  #else
    ADMUX = _BV(REFS0) | _BV(MUX3) | _BV(MUX2) | _BV(MUX1);
  #endif

  delay(2); // Wait for Vref to settle
  ADCSRA |= _BV(ADSC); // Start conversion
  while (bit_is_set(ADCSRA,ADSC)); // measuring

  uint8_t low  = ADCL; // must read ADCL first - it then locks ADCH
  uint8_t high = ADCH; // unlocks both

  long result = (high<<8) | low;

  result = 1125300L / result; // Calculate Vcc (in mV); 1125300 = 1.1*1023*1000
  return result; // Vcc in millivolts
}

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

enum {LOCATION, BATTERY, CHECKSTATUS};

void refreshGPSData(){
  digitalWrite(GPSPOWERPIN, HIGH);
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
  digitalWrite(GPSPOWERPIN, LOW);
}

void reset(){
  fona.enableGPRS(false);

}


void initGPRS(){
  while(fona.getNetworkStatus() != 1){
    Serial.println("Connecting to Network");
    delay(1000);
  }
  Serial.println("Connected to Network");
  delay(1000);
  while (!fona.enableGPRS(true)){
    Serial.println(F("Failed to turn on GPRS. Try again..."));
    delay(1000);
  }
  Serial.println("GPRS activation succesful");
}


String performGetRequest(byte topic){
  fonaSoftSerial.begin(4800);

  uint16_t statuscode;
  int16_t length;
  String combinedUrl = rootUrl;
  String response;


  if (! fona.begin(fonaSoftSerial)) {
    Serial.println(F("Couldn't find FONA"));
  }

  if (!fona.GPRSstate()) initGPRS();

  switch (topic) {
    case LOCATION: {
      combinedUrl += "?setLat=" + String(gps.location.lat(), 6) + "&setLon=" + String(gps.location.lng(), 6);
      break;
    }
    case CHECKSTATUS:{
      combinedUrl += "?getCheckStatus" + String("&setBatteryStatus=") + String(readVcc());
    }
  }
  Serial.println(combinedUrl);

  //Main part sending data
  if (!fona.HTTP_GET_start(combinedUrl.c_str(), &statuscode, (uint16_t *)&length)) Serial.println("Failed!");
  while (length > 0) {
            while (fona.available()) {
              char c = fona.read();
              Serial.write(c);
              response += c;
              length--;
              if (! length) break;
            }
          }
          Serial.println(F("\n****"));
          fona.HTTP_GET_end();
  Serial.println(response);
  reset();
  fonaSoftSerial.end();
  return response;
}


void setup()
{
  pinMode(GSMPOWERPIN, OUTPUT);
  pinMode(GPSPOWERPIN, OUTPUT);
  digitalWrite(GSMPOWERPIN, LOW);
  digitalWrite(GPSPOWERPIN, LOW);
  delay(5000);
}

void loop()
{
  Serial.begin(9600);
  digitalWrite(GSMPOWERPIN, HIGH);
  String response = performGetRequest(CHECKSTATUS);
  if(response == "true"){
    refreshGPSData();
    performGetRequest(LOCATION);
  }
  if(response == "battery"){
    performGetRequest(BATTERY);
  }

  //Prepare Deep Sleep;
  Serial.end();
  digitalWrite(GSMPOWERPIN, LOW);
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
  for(int i=0; i<8; i++)
  LowPower.idle(SLEEP_8S, ADC_OFF, TIMER2_OFF, TIMER1_OFF, TIMER0_OFF,
               SPI_OFF, USART0_OFF, TWI_OFF);
}
