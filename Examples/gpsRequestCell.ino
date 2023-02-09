// Regularly send GPS info to a server (Hologram SIM in this example) and allow the server to request on the spot GPS info

#include <Arduino.h>
#include <FreematicsCell.h>
#include <TimeLib.h>

#define PIN_BEE_UART_RXD 16
#define PIN_BEE_UART_TXD 17
#define PIN_BEE_PWR 27
#define PIN_LED 4

// testing URL: https://hub.freematics.com/test
#define SERVER_HOST "cloudsocket.hologram.io"
#define SERVER_PORT 9999
#define SERVER_PATH "/api/1/devices/messages/******/********************************"
#define CELL_APN "hologram"
#define CONN_TIMEOUT 210000

#define CODE_VERSION "1.5.5.0"

typedef struct {
  String out = "";
  String topic = "";
} CEL_MSG;

HTTPClientSIM7600 net;
int errors = 0;
String ops;
int signal = -114;
int connMode = 2;
int initRetries = 0;
CEL_MSG backedUp[50] = {};

bool operator== (const CEL_MSG& prev, const CEL_MSG& next) {
  if (prev.out == next.out && prev.topic == next.topic) {
    return true;
  }
  return false;
}

void(* hardReset) (void) = 0;

bool init_net()
{
    Serial.println("----------------------");
    Serial.print("Init cellular module... ");
    if (net.begin()) {
      Serial.print(net.deviceName());
      Serial.println(" OK");
    } else {
      Serial.println("NO");
      hardReset();
      return false;
    }
    Serial.print("IMEI:");
    Serial.println(net.IMEI);

    if (net.checkSIM()) {
      Serial.println("SIM Card OK");
    } else {
      Serial.println("No SIM Card");
    }

    int stage = -1;
    int attemp = 1;
    
    while (stage != 5) {
      Serial.print("Registering on network...");
      stage = net.setup(CELL_APN, CONN_TIMEOUT, connMode);
    
      if (stage == 5) {
        Serial.println("OK");
      } else {
        Serial.print("failed, stage ");
        Serial.print(stage);
        Serial.print(", mode ");
        Serial.println(connMode);
        attemp++;
      }

      if (attemp > 5) {
        Serial.println("Re-init!");
        initRetries++;
        connMode = 2;
        return false;
      }

      if (attemp == 3 && initRetries == 0) {
        connMode = 39;
      }
    }

    String op = net.getOperatorName();
    if (op.length()) {
      Serial.print("Operator: ");
      Serial.println(op);
    } else {
      Serial.println("Could not get current operator!");
      return false;
    }

    String sysMode = net.getSystemMode();
    if (sysMode.length()) {
      Serial.print("Mode: ");
      Serial.println(sysMode);
    } else {
      Serial.println("Could not get system mode!");
    }

    ops = net.getAvailableOperators();
    if (ops.length()) {
      Serial.print("Operators: ");
      Serial.println(ops);
    } else {
      Serial.println("Could not get operators");
    }

    Serial.print("Obtaining IP address...");
    String ip = net.getIP();
    if (ip) {
      Serial.println(ip);
    } else {
      Serial.println("N/A");
      return false;
    }

    signal = net.getSignal();
    if (signal > -113 && signal != 85) {
      Serial.print("RSSI:");
      Serial.print(signal);
      Serial.println("dBm");
    } else if (signal == -113) {
      Serial.println(F("No signal... (-113dBm)"));
    } else if (signal == 85){
      Serial.println(F("Unable to get signal... (85dBm)"));
      return false;
    } else {
      Serial.println(F("No signal..."));
    }
    
    for (int i = 0; i < 3; i++) {
      Serial.print("Starting Server... ");
      if (net.startTCP(4010, 0)) {
        Serial.println(" OK");
        break;
      } else {
        Serial.println(" failed starting, try #" + String(i));
        char* checkServer = net.checkServer();
        Serial.println(checkServer);
        if (checkServer != 0) {
          break;
        }
      }
    }
      
    net.setGPS(true);
    return true;
}

bool reconnect_net() {
  Serial.println("----------------------");
  
  int stage = -1;
  while (stage != 5) {
    Serial.print("Registering on network...");
    stage = net.setup(CELL_APN, CONN_TIMEOUT, connMode);
  
    if (stage == 5) {
      Serial.println("OK");
    } else {
      Serial.print("failed, stage ");
      Serial.print(stage);
      Serial.print(", mode ");
      Serial.println(connMode);
    }
  }

  String op = net.getOperatorName();
  if (op.length()) {
    Serial.print("Operator: ");
    Serial.println(op);
  } else {
    Serial.println("Could not get current operator!");
    return false;
  }

  String sysMode = net.getSystemMode();
  if (sysMode.length()) {
    Serial.print("Mode: ");
    Serial.println(sysMode);
  } else {
    Serial.println("Could not get system mode!");
  }
  
  ops = net.getAvailableOperators();
  if (ops.length()) {
    Serial.print("Operators: ");
    Serial.println(ops);
  } else {
    Serial.println("Could not get operators");
  }

  Serial.print("Obtaining IP address...");
  String ip1 = net.getIP();
  if (ip1 && ip1 != "") {
    Serial.println(ip1);
  } else {
    Serial.println("N/A");
    return false;
  }

  signal = net.getSignal();
  if (signal > -113 && signal != 85) {
    Serial.print("RSSI:");
    Serial.print(signal);
    Serial.println("dBm");
  } else if (signal == -113) {
    Serial.println(F("No signal... (-113dBm)"));
  } else if (signal == 85){
    Serial.println(F("Unable to get signal... (85dBm)"));
    return false;
  } else if (signal == 6565){
    Serial.println(F("Unable to get signal... (6565dBm)"));
    return false;
  } else {
    Serial.println(F("No signal..."));
  }
  
  for (int i = 0; i < 3; i++) {
    Serial.print("Starting Server... ");
    if (net.startTCP(4010, 0)) {
      Serial.println(" OK");
      break;
    } else {
      Serial.println(" failed starting");
      char* checkServer = net.checkServer();
      Serial.println(checkServer);
      if (checkServer != 0) {
        break;
      }
    }
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

GPS_DATA getPosGSM() {
  GPS_DATA gpsdata;
  gpsdata = net.getGSM();
  return gpsdata;
}

String formattedGPS(bool reply = false, bool gsm = false) {
  String out;
  TimeElements curTime;
  time_t epoch;
  if (!gsm) {
    GPS_DATA gpsdata = getPos();
    if (gpsdata.sat > 0) {
      curTime.Day = (int)(gpsdata.date / 10000);
      curTime.Month = (int)((gpsdata.date - (curTime.Day * 10000)) / 100);
      curTime.Year = ((gpsdata.date % 100) + 30);
      curTime.Hour = (int)(gpsdata.time / 1000000);
      curTime.Minute = (int)((gpsdata.time - (curTime.Hour * 1000000)) / 10000);
      curTime.Second = (gpsdata.time / 100) % 100;
      epoch = makeTime(curTime);
      if (!reply) {
        out = "{\\\"T\\\":\\\"" + String(epoch) + "\\\",\\\"Lat\\\":\\\"" + String(gpsdata.lat, 6) + "\\\",\\\"Lon\\\":\\\"" + String(gpsdata.lng, 6) + "\\\",\\\"Spd\\\":\\\"" + String(gpsdata.speed, 1) + "\\\",\\\"A\\\":\\\"" + String(gpsdata.alt) + "\\\",\\\"H\\\":\\\"" + String(gpsdata.heading) + "\\\",\\\"Sat\\\":\\\"" + String(gpsdata.sat) + "\\\",\\\"PE\\\":\\\"" + String(gpsdata.pe, 1) + "\\\",\\\"HE\\\":\\\"" + String(gpsdata.he, 1) + "\\\",\\\"VE\\\":\\\"" + String(gpsdata.ve, 1) + "\\\"}";
      } else {
        out = "{\"T\":\"" + String(epoch) + "\",\"Lat\":\"" + String(gpsdata.lat, 6) + "\",\"Lon\":\"" + String(gpsdata.lng, 6) + "\",\"Spd\":\"" + String(gpsdata.speed, 1) + "\",\"A\":\"" + String(gpsdata.alt) + "\",\"H\":\"" + String(gpsdata.heading) + "\",\"Sat\":\"" + String(gpsdata.sat) + "\",\"PE\":\"" + String(gpsdata.pe, 1) + "\",\"HE\":\"" + String(gpsdata.he, 1) + "\",\"VE\":\"" + String(gpsdata.ve, 1) + "\"}";
      }
      Serial.println(out);
    } else {
      if (!reply) {
        out = "{\\\"Error\\\":\\\"No GPS Signal\\\"}";
      } else {
        out = "{\"Error\":\"No GPS Signal\"}";
      }
      Serial.println(F("No response... trying GSM"));
      out = formattedGPS(reply, true);
    }
  } else {
    GPS_DATA gpsdata = getPosGSM();
    if (gpsdata.stat == 0) {
      curTime.Day = (int)(gpsdata.date / 10000);
      curTime.Month = (int)((gpsdata.date - (curTime.Day * 10000)) / 100);
      curTime.Year = ((gpsdata.date % 100) + 30);
      curTime.Hour = (int)(gpsdata.time / 1000000);
      curTime.Minute = (int)((gpsdata.time - (curTime.Hour * 1000000)) / 10000);
      curTime.Second = (gpsdata.time / 100) % 100;
      epoch = makeTime(curTime);
      if (!reply) {
        out = "{\\\"T\\\":\\\"" + String(epoch) + "\\\",\\\"Lat\\\":\\\"" + String(gpsdata.lat, 6) + "\\\",\\\"Lon\\\":\\\"" + String(gpsdata.lng, 6) + "\\\",\\\"Spd\\\":\\\"" + String(gpsdata.speed, 1) + "\\\",\\\"A\\\":\\\"" + String(gpsdata.alt) + "\\\",\\\"H\\\":\\\"" + String(gpsdata.heading) + "\\\",\\\"Sat\\\":\\\"" + String(gpsdata.sat) + "\\\",\\\"PE\\\":\\\"" + String(gpsdata.pe, 1) + "\\\",\\\"HE\\\":\\\"" + String(gpsdata.he, 1) + "\\\",\\\"VE\\\":\\\"" + String(gpsdata.ve, 1) + "\\\"}";
      } else {
        out = "{\"T\":\"" + String(epoch) + "\",\"Lat\":\"" + String(gpsdata.lat, 6) + "\",\"Lon\":\"" + String(gpsdata.lng, 6) + "\",\"Spd\":\"" + String(gpsdata.speed, 1) + "\",\"A\":\"" + String(gpsdata.alt) + "\",\"H\":\"" + String(gpsdata.heading) + "\",\"Sat\":\"" + String(gpsdata.sat) + "\",\"PE\":\"" + String(gpsdata.pe, 1) + "\",\"HE\":\"" + String(gpsdata.he, 1) + "\",\"VE\":\"" + String(gpsdata.ve, 1) + "\"}";
      }
      Serial.println(out);
    } else {
      if (!reply) {
        out = "{\\\"Error\\\":\\\"No GPS Signal\\\"}";
      } else {
        out = "{\"Error\":\"No GPS Signal\"}";
      }
      Serial.println(("GSM error, stat ") + gpsdata.stat);
    }
  }
  
  return out;
}

bool reply(String out) {
  char* checkTCP = net.checkTCP();
  char outbound[out.length()] = {0};
  int len = sprintf(outbound, "%s", out.c_str());
  int attm = 1;
  char* probChar = net.findOpenTCP();
  int prob = (int)(probChar[0] - '0');
  Serial.println(prob);
  while (!net.sendTCP(outbound, len, prob) && attm < 5) {
    attm++;
  }
  if (attm < 2) {
    Serial.print("Sent: ");
    Serial.println(outbound);
    Serial.print("Closing connection...");
    if (net.closeTCP(prob)) {
      Serial.println("OK");
      return true;
    } else {
      Serial.println("Failed!");
      return false;
    }
  } else {
    Serial.print("Failed to send ");
    Serial.print(len);
    Serial.print(" charachters: ");
    Serial.println(outbound);
    return false;
  }
}

bool sendNew(String out, String topic) {
  Serial.print("Checking to see if link 0 is in use...");
  int linkNum = 0;
  char* probChar;
  probChar = net.findOpenTCP();
  if (probChar != 0) {
    int prob = (int)(probChar[0] - '0');
    if (prob == 0) {
      linkNum = 9;
      Serial.println(" it is, switching to link 9");
    }
  }
  if (linkNum == 0) {
    Serial.println(" it is not");
  }
  
  char outbound[out.length() + topic.length() + 31] = {0};
  int len = sprintf(outbound, "{\"k\":\"********\",\"d\":\"%s\",\"t\":\"%s\"}", out.c_str(), topic.c_str());
      
  Serial.print("Opening connection...");
  if (!net.openTCP(SERVER_HOST, SERVER_PORT, linkNum)) {
    Serial.println("failed! Checking open TCP connections...");
  
    char* checkTCP = net.checkTCP();
    Serial.println(checkTCP);
    return false;
  } else {
    Serial.println("OK");
    
    int attm = 1;
    while (!net.sendTCP(outbound, len, linkNum) && attm < 5) {
      attm++;
    }
    if (attm < 2) {
      Serial.print("Sent: ");
      Serial.println(outbound);
      Serial.print("Closing connection...");
      if (net.closeTCP(linkNum)) {
        Serial.println("OK");
        return true;
      } else {
        Serial.println("Failed!");
        return false;
      }
    } else {
      Serial.print("Failed to send ");
      Serial.print(len);
      Serial.print(" charachters: ");
      Serial.print(outbound);
      CEL_MSG tmpmsg;
      CEL_MSG msg;
      msg.out = out;
      msg.topic = topic;
      bool flag = false;
      for (int i = 0; i < sizeof(backedUp); i++) {
        if (backedUp[i] == tmpmsg) {
          Serial.print(" -- Storing in position ");
          Serial.println(i);
          backedUp[i] = msg;
          flag = true;
          break;
        }
      }
      if (!flag) {
        Serial.println("Storage is full!");
      }
      return false;
    }
  }
}


void setup()
{
  pinMode(PIN_LED, OUTPUT);
  light(false);
  
  Serial.begin(115200);
  Serial.println("Code version " + String(CODE_VERSION));

  // start serial communication with cellular module
  net.xbBegin(115200, PIN_BEE_UART_RXD, PIN_BEE_UART_TXD);

  // initialize cellular module
  while (!init_net());

  light(true);
  Serial.println("Waiting for incoming messages");
}


void loop()
{
  if (millis() % 20000 < 990) {
    //Serial.print("Checking connection...");
    String ip = net.getIP();
    if (ip && ip != "") {
      //Serial.println("Connection good, " + ip);
    } else {
      light(false);
      Serial.println("Connection lost! Re-connecting...");
      while (!reconnect_net());
      light(true);
    }
    
    signal = net.getSignal();
    if (signal > -113 && signal != 85) {
      Serial.print("RSSI:");
      Serial.print(signal);
      Serial.println("dBm");
    } else if (signal == -113) {
      light(false);
      Serial.println(F("No signal... (-113dBm)"));
      Serial.println("Connection lost! Re-connecting...");
      while (!reconnect_net());
      light(true);
    } else if (signal == 85){
      light(false);
      Serial.println(F("Unable to get signal... (85dBm)"));
      Serial.println("Connection lost! Re-connecting...");
      while (!reconnect_net());
      light(true);
    } else {
      Serial.println("No signal... " + ((signal + 113) / 2));
    }
  }

  if (millis() % 300000 < 1000) {
    Serial.println("Sending routine GPS...");

    Serial.print("Getting GPS data...");
    String content;
    content = formattedGPS();

    bool sent = sendNew(content, "GPS");
  }
  
  /*if (Serial.available() > 0) {
    char incomingCommandSerial[64];
    int indexInc = 0;
    while(Serial.available()) {
      incomingCommandSerial[indexInc] = Serial.read();
      indexInc++;
    }
    incomingCommandSerial[indexInc] = '\0';
    Serial.print("Incoming command: ");
    Serial.println(incomingCommandSerial);
    char* reply = net.sendCommandRaw(incomingCommandSerial);
    Serial.println("----------------");
    Serial.println(reply);
    Serial.println("----------------");
  }*/
  
  char *response;
  int bytes;
  response = net.receiveTCP(&bytes, 1000);
  if (response) {
    Serial.println("Incoming Webhook");
    Serial.println("-----TCP INCOMING-----");
    Serial.println(response);
    Serial.println("----------------------");

    if (!strstr(response,"ok")) {
      if (strstr(response,"gps")) {
    
        String content;
        content = formattedGPS(true);
    
        bool sent = reply(content);
        
      } else if (strstr(response,"ping")) {

        Serial.println("Replying 'pong!'...");
        String content = "pong!";

        bool sent = reply(content);
      }
    } else {
      Serial.println("Got a message, but filtering...");
    }
  } else {}
}