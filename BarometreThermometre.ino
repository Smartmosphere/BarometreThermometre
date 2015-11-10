// Barometer
#include <SFE_BMP180.h>
#include <Wire.h>

#include "U8glib.h"
#include <SoftwareSerial.h>
#include <Time.h>

#define SSID "YOUR_WIFI_SSID"
#define PASS "YOUR_WIFI_PASSWORD"
#define THINGSPEAK_IP "184.106.153.149"    // thingspeak.com
#define MAX_FAILS 3

SoftwareSerial espSerial(2, 3);          // RX, TX

#define TIME_HEADER  "T"   // Header tag for serial time sync message
#define TIME_REQUEST  7    // ASCII bell character requests a time sync message 

// Barometer
SFE_BMP180 pressure;
double T, P, p0, a;

unsigned long sample_interval = 60000;
unsigned long last_time;
int fails = 0;

U8GLIB_SSD1306_128X64 u8g(U8G_I2C_OPT_NONE);

void setup() {
  pinMode(2, INPUT);    // softserial RX
  pinMode(3, OUTPUT);   // softserial TX
  pinMode(4, OUTPUT);    // esp reset

  delay(3000);           // to enable uploading??

  Serial.begin(115200);
  espSerial.begin(115200);
  espSerial.setTimeout(2000);

  
  // Barometer
  if (!pressure.begin()) {
    Serial.println("Could not find a valid BMP085 sensor, check wiring!");
  }

  last_time = millis();  // get the current time;

  if (!resetESP()) return;
  if (!connectWiFi()) return;

  setSyncProvider( requestSync);  //set function to call when sync required
}

void echoAll() {
  while (espSerial.available()) {
    char c = espSerial.read();
    Serial.write(c);
    if (c == '\r') Serial.print('\n');
  }
}

boolean resetESP() {
  // - test if module ready
  Serial.print(F("reset ESP8266..."));

  // physically reset EPS module
  digitalWrite(4, LOW);
  delay(100);
  digitalWrite(4, HIGH);
  delay(500);

  if (!send("AT+RST", "ready", F("%% module no response"))) return false;

  Serial.print(F("module ready..."));
  return true;
}

boolean connectWiFi() {
  int tries = 5;

  while (tries-- > 0 && !tryConnectWiFi());

  if (tries <= 0) {
    Serial.println(F("%% tried X times to connect, please reset"));
    return false;
  }

  delay(500); // TOOD: send and wait for correct response?

  // set the single connection mode
  espSerial.println("AT+CIPMUX=0");

  delay(500); // TOOD: send and wait for correct response?
  // TODO: listen?

  return true;
}

boolean tryConnectWiFi() {
  espSerial.println("AT+CWMODE=1");
  delay(2000); // TOOD: send and wait for correct response?

  String cmd = "AT+CWJAP=\"";
  cmd += SSID;
  cmd += "\",\"";
  cmd += PASS;
  cmd += "\"";

  if (!send(cmd, "OK", F("%% cannot connect to wifi..."))) return false;

  Serial.println(F("WiFi OK..."));
  return true;
}

boolean send(String cmd, char* waitFor, String errMsg) {
  espSerial.println(cmd);
  if (!espSerial.find(waitFor)) {
    Serial.print(errMsg);
    return false;
  }
  return true;
}

boolean connect(char* ip) {
  String cmd;
  cmd = "AT+CIPSTART=\"TCP\",\"";
  cmd += ip;
  cmd += "\",80";
  espSerial.println(cmd);
  // TODO: this is strange, we wait for ERROR
  // so normal is to timeout (2s) then continue
  if (espSerial.find("Error")) return false;
  return true;
}

boolean sendGET(String path) {
  String cmd = "GET ";
  cmd += path;

  // Part 1: send info about data to send
  String xx = "AT+CIPSEND=";
  xx += cmd.length();
  if (!send(xx, ">", F("%% connect timeout"))) return false;
  Serial.print(">");

  // Part 2: send actual data
  if (!send(cmd, "SEND OK", F("%% no response"))) return false;

  return true;
}

void loop() {

  if (Serial.available()) {
    processSyncMessage();
  }
  if (timeStatus() != timeNotSet) {
    digitalClockDisplay();
  }

  if (millis() - last_time < sample_interval) return;

  // Barometer
  char status;

  status = pressure.startTemperature();
  if (status != 0)
  {
    // Wait for the measurement to complete:
    delay(status);
    // Retrieve the completed temperature measurement:
    // Note that the measurement is stored in the variable T.
    // Function returns 1 if successful, 0 if failure.
    status = pressure.getTemperature(T);
    if (status != 0)
    {
      // Print out the measurement:
      /*
      Serial.print("temperature: ");
      Serial.print(T,2);
      Serial.print(" deg C, ");
      Serial.print((9.0/5.0)*T+32.0,2);
      Serial.println(" deg F");
      */

      // Start a pressure measurement:
      // The parameter is the oversampling setting, from 0 to 3 (highest res, longest wait).
      // If request is successful, the number of ms to wait is returned.
      // If request is unsuccessful, 0 is returned.
      status = pressure.startPressure(3);
      if (status != 0)
      {
        // Wait for the measurement to complete:
        delay(status);
        // Retrieve the completed pressure measurement:
        // Note that the measurement is stored in the variable P.
        // Note also that the function requires the previous temperature measurement (T).
        // (If temperature is stable, you can do one temperature measurement for a number of pressure measurements.)
        // Function returns 1 if successful, 0 if failure.
        status = pressure.getPressure(P, T);
        if (status != 0)
        {
          // Print out the measurement:
          /*
          Serial.print("absolute pressure: ");
          Serial.print(P,2);
          Serial.print(" mb, ");
          Serial.print(P*0.0295333727,2);
          Serial.println(" inHg");
          */
        }
      }
    }
  }

  last_time = millis();

  /*
  Serial.print(last_time);
  Serial.print(": ");
  Serial.print(t, 2);
  Serial.print("C");
  Serial.print(" ");
  Serial.print(h, 2);
  Serial.print("% ");
  */

  if (!sendDataThingSpeak(P, T)) {
    Serial.println(F("%% failed sending data"));
    // we failed X times, at MAX_FAILS reconnect till it works
    if (fails++ > MAX_FAILS) {
      if (!resetESP()) return;
      if (!connectWiFi()) return;
    }
  } else {
    fails = 0;
  }

  Serial.println();

  u8g.firstPage();  //OLED Display
  do {
    OLED_display(P, T);
    u8g.setColorIndex(1);
  } while ( u8g.nextPage() );
}

void OLED_display(float temp, float hum)
{
  if (isnan(P) || isnan(T)) {
    u8g.setFont(u8g_font_7x14);
    u8g.drawStr(10, 10, "Failed to read");
    u8g.drawStr(10, 25, "from BMP180");
  } else {
    u8g.setFont(u8g_font_7x14);
    // Barometer
    u8g.setPrintPos(0, 10);
    u8g.print("P:");
    u8g.print(P, 1);
    u8g.print("hPa");
    delay(1);
    u8g.setPrintPos(0, 20);
    u8g.print("T:");
    u8g.print(T, 1);
    u8g.print("C");
    delay(1);
  }
  u8g.setPrintPos(8, 40);
  u8g.print("smartmosphere.com");
  char strOut[3];
  u8g.setPrintPos(40, 54);
  formatTimeDigits(strOut, hour());
  u8g.print(strOut);
  u8g.print(":");
  formatTimeDigits(strOut, minute());
  u8g.print(strOut);
  u8g.print(":");
  formatTimeDigits(strOut, second());
  u8g.print(strOut);
  delay(1);
  
}

void formatTimeDigits(char strOut[3], int num)
{
  strOut[0] = '0' + (num / 10);
  strOut[1] = '0' + (num % 10);
  strOut[2] = '\0';
}

boolean sendDataThingSpeak(float temp, float hum) {
  if (!connect(THINGSPEAK_IP)) return false;

  String path = "/update?key=YOUR_THINGSPEAK_API_KEY&field1=";
  path += temp;
  path += "&field2=";
  path += hum;
  path += "\r\n";
  if (!sendGET(path)) return false;

  Serial.print(F(" thingspeak.com OK"));
  return true;
}


void processSyncMessage() {
  unsigned long pctime;
  const unsigned long DEFAULT_TIME = 1357041600; // Jan 1 2013

  if (Serial.find(TIME_HEADER)) {
    pctime = Serial.parseInt();
    if ( pctime >= DEFAULT_TIME) { // check the integer is a valid time (greater than Jan 1 2013)
      setTime(pctime); // Sync Arduino clock to the time received on the serial port
    }
  }
}

void digitalClockDisplay() {
  // digital clock display of the time
  Serial.print(hour());
  printDigits(minute());
  printDigits(second());
  Serial.print(" ");
  Serial.print(day());
  Serial.print(" ");
  Serial.print(month());
  Serial.print(" ");
  Serial.print(year());
  Serial.println();
}

void printDigits(int digits) {
  // utility function for digital clock display: prints preceding colon and leading 0
  Serial.print(":");
  if (digits < 10)
    Serial.print('0');
  Serial.print(digits);
}

time_t requestSync()
{
  Serial.write(TIME_REQUEST);
  return 0; // the time will be sent later in response to serial mesg
}
