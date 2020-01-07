#include <SPI.h>
#include <WiFiNINA.h>
#include <NewPing.h>

// Store WiFi credentials in secrets file. Not checked into version control.
#include "arduino_secrets.h"

// Print out extra commands to the serial line if true
#define DEBUG true

// Get the WiFi credentials from the secrets file
char ssid[] = SECRET_SSID; // WiFi Name
char pass[] = SECRET_PASS; // WiFi Password
int keyIndex = 0; // Only needed for WEP. Network index number.

// Hardcode an IP address
IPAddress ip(192, 168, 7, 132);

// Current status for the network
int status = WL_IDLE_STATUS;

// Pin number for the LED
int ledPin = 6;

// Sonar sensors
NewPing sonar1(0, 1); // trigger_pin, echo_pin [, max_cm_distance]
NewPing sonar2(2, 3);
NewPing sonar3(4, 5);
NewPing sonarSensors[3] = {sonar1, sonar2, sonar3};
#define numSensors (sizeof(sonarSensors)/sizeof(NewPing))

// WiFi Server
WiFiServer server(80);

void setup() {
  // Initialize serial and wait for port to open
  Serial.begin(9600);

  // check for the WiFi module:
  if (WiFi.status() == WL_NO_MODULE) {
    Serial.println("Communication with WiFi module failed!");
    didFailSetup();
  }

  String fv = WiFi.firmwareVersion();
  if (fv < WIFI_FIRMWARE_LATEST_VERSION) {
    Serial.println("Please upgrade the firmware");
  }

  WiFi.config(ip);

  // Don't continue until we can connnect to WiFi
  // or reach a timeout
  int attempts = 0;
  while (status != WL_CONNECTED && attempts < 10) {
    status = WiFi.begin(ssid, pass);
    delay(3000 * attempts);
    attempts = attempts + 1;
  }

  if (status == WL_CONNECTED) {
    server.begin();

    pinMode(ledPin, OUTPUT);

    // Connected to wifi - turn the light on!
    digitalWrite(ledPin, HIGH);
    WiFi.lowPowerMode();
    printWifiStatus();
  } else {

    didFailSetup();
  }
}

void didFailSetup() {
  // Indefinitely flash the LED to show we failed during setup
  while (true) {
    digitalWrite(ledPin, HIGH);
    delay(500);
    digitalWrite(ledPin, LOW);
    delay(500);
  }
}

void loop() {
  // Listen for incoming HTTP requests
  WiFiClient client = server.available();
  if (client) {
    // An http request ends with a blank line
    boolean currentLineIsBlank = true;
    while (client.connected()) {
      if (client.available()) {
        char c = client.read();
        Serial.write(c);
        if (c == '\n' && currentLineIsBlank) {

          // Send a standard http response header
          client.println("HTTP/1.1 200 OK");
          client.println("Content-Type: text/json");
          client.println("Connection: close");  // the connection will be closed after completion of the response
          client.println();

          // Build a response with the just-read readings
          String response = "{\"readings\":[";
          for (int i = 0; i < numSensors; i++) {

            const int totalReadingsPerSensor = 5;
            int total = 0;
            for (int k = 0; k < totalReadingsPerSensor; k++) {
              int reading = sonarSensors[i].ping_in();
              total = total + reading;

              if (DEBUG) {
                Serial.print("Reading ");
                Serial.print(k);
                Serial.print(" from sensor ");
                Serial.print(i);
                Serial.print(": ");
                Serial.println(reading);
              }

              delay(60);
            }
            Serial.println("");

            int average = total / totalReadingsPerSensor;
            response = response + average;

            if (i < numSensors - 1) {
              response = response + ",";
            }
          }

          response = response + "]}";
          client.println(response);
          break;
        }
        if (c == '\n') {
          // you're starting a new line
          currentLineIsBlank = true;
        } else if (c != '\r') {
          // you've gotten a character on the current line
          currentLineIsBlank = false;
        }
      }
    }
    // give the web browser time to receive the data
    delay(1);

    // close the connection:
    client.stop();
  }
}


void printWifiStatus() {
  // print the SSID of the network you're attached to:
  Serial.print("SSID: ");
  Serial.println(WiFi.SSID());

  // print your board's IP address:
  IPAddress ip = WiFi.localIP();
  Serial.print("IP Address: ");
  Serial.println(ip);

  // print the received signal strength:
  long rssi = WiFi.RSSI();
  Serial.print("signal strength (RSSI):");
  Serial.print(rssi);
  Serial.println(" dBm");
}
