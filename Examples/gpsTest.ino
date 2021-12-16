// Test to pull GPS from the SIM hat and write to the Serial

#include <Arduino.h>
#include <FreematicsCell.h>
#include <TimeLib.h>

#define PIN_BEE_UART_RXD 16
#define PIN_BEE_UART_TXD 17
#define PIN_BEE_PWR 27
#define PIN_LED 4

HTTPClientSIM7600 net;
int errors = 0;
String ops;
int signal = -114;
int connMode = 48;
int initRetries = 0;

typedef struct {
  String out;
  String topic;
} CEL_MSG;

bool init_net()
{
    Serial.println("----------------------");
    Serial.print("Init cellular module...");
    if (net.begin()) {
      Serial.print(net.deviceName());
      Serial.println(" OK");
    } else {
      Serial.println("NO");
      return false;
    }
    Serial.print("IMEI:");
    Serial.println(net.IMEI);

    if (net.checkSIM()) {
      Serial.println("SIM Card OK");
    } else {
      Serial.println("No SIM Card");
    }
    
    if (!net.setGPS(true)) {
      Serial.println("Failed to turn on GPS");
      return false;
    }
    return true;
}

void light(bool state) {
  if (state) {
    digitalWrite(PIN_LED, HIGH);
  } else {    
    digitalWrite(PIN_LED, LOW);
  }
}

GPS_DATA getPos() {
  GPS_DATA gpsdata;
  net.checkGPS();
  gpsdata = net.getGPS();
  return gpsdata;
}

String formattedGPS(bool reply = false) {
  String out;
  TimeElements curTime;
  time_t epoch;
  GPS_DATA gpsdata = getPos();
  if (gpsdata.sat > 0) {
    curTime.Day = (int)(gpsdata.date / 10000);
    curTime.Month = (int)((gpsdata.date - (curTime.Day * 10000)) / 100);
    curTime.Year = ((gpsdata.date % 100) + 30);
    curTime.Hour = (int)(gpsdata.time / 1000000);
    curTime.Minute = (int)((gpsdata.time - (curTime.Hour * 1000000)) / 10000);
    curTime.Second = (gpsdata.time / 100) % 100;
    epoch = makeTime(curTime);
    light(true);
    if (!reply) {
      out = "{\\\"T\\\":\\\"" + String(epoch) + "\\\",\\\"Lat\\\":\\\"" + String(gpsdata.lat, 6) + "\\\",\\\"Lon\\\":\\\"" + String(gpsdata.lng, 6) + "\\\",\\\"Spd\\\":\\\"" + String(gpsdata.speed, 1) + "\\\",\\\"A\\\":\\\"" + String(gpsdata.alt) + "\\\",\\\"H\\\":\\\"" + String(gpsdata.heading) + "\\\",\\\"Sat\\\":\\\"" + String(gpsdata.sat) + "\\\",\\\"PE\\\":\\\"" + String(gpsdata.pe, 1) + "\\\",\\\"HE\\\":\\\"" + String(gpsdata.he, 1) + "\\\",\\\"VE\\\":\\\"" + String(gpsdata.ve, 1) + "\\\"}";
    } else {
      out = "{\"T\":\"" + String(epoch) + "\",\"Lat\":\"" + String(gpsdata.lat, 6) + "\",\"Lon\":\"" + String(gpsdata.lng, 6) + "\",\"Spd\":\"" + String(gpsdata.speed, 1) + "\",\"A\":\"" + String(gpsdata.alt) + "\",\"H\":\"" + String(gpsdata.heading) + "\",\"Sat\":\"" + String(gpsdata.sat) + "\",\"GPS\":\"" + String(gpsdata.gps_sat) + "\",\"GLONASS\":\"" + String(gpsdata.glonass_sat) + "\",\"Beidou\":\"" + String(gpsdata.beidou_sat) + "\",\"PE\":\"" + String(gpsdata.pe, 1) + "\",\"HE\":\"" + String(gpsdata.he, 1) + "\",\"VE\":\"" + String(gpsdata.ve, 1) + "\"}";
    }
    Serial.println(out);
  } else {
    light(false);
    if (!reply) {
      out = "{\\\"Error\\\":\\\"No GPS Signal\\\"}";
    } else {
      out = "{\"Error\":\"No GPS Signal\"}";
    }
    Serial.println(F("No response..."));
  }
  return out;
}

void setup()
{
  pinMode(PIN_LED, OUTPUT);
  light(false);
  
  Serial.begin(115200);

  // start serial communication with cellular module
  net.xbBegin(115200, PIN_BEE_UART_RXD, PIN_BEE_UART_TXD);

  // initialize cellular module
  while (!init_net());

  Serial.println("Ready");
}


void loop()
{
  if (millis() % 1000 < 10) {
    Serial.print("Getting GPS... ");
    formattedGPS(true);
  }
}
