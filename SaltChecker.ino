#include <SPI.h>
#include <WiFiNINA.h>
#include <NewPing.h>

// Store WiFi credentials in secrets file. Not checked into version control.
#include "arduino_secrets.h"

// Delay between pulses in milliseconds
const int delayBetweenPulses = 60;

// Highest standard deviation allowed before redoing the sensor reading
const int highestStdDevAllowed = 10;

// Total readings per sensor
const int totalReadingsPerSensor = 8;

// How many times should you redo the sensor readings
// if the standard deviation check fails?
const int totalRedoCountForStdDev = 20;

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
    attempts++;
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

  // If a client connected, we need to read the sensors and form a response
  if (client) {
    // Wait until the current line is blank so we know the request has finished
    boolean currentLineIsBlank = true;
    while (client.connected()) {
      if (client.available()) {
        // Read all of the available client data until we hit a blank line
        char c = client.read();
        Serial.write(c);

        // We reached a blank line, now we can start on the response!
        if (c == '\n' && currentLineIsBlank) {

          // Send a standard http response header
          client.println("HTTP/1.1 200 OK");
          client.println("Content-Type: text/json");
          client.println("Connection: close");  // the connection will be closed after completion of the response
          client.println();

          // Read the sensors to form the response body
          // Populate this response in a string and concatenate to it.
          //
          // Responses are in the format:
          //
          //
          String response = "{\"readings\":[";
          for (int i = 0; i < numSensors; i++) {

            // Collect data for this sensor.
            // If the standard deviation is too high, re-collect it.
            int sensorReadAttempts = 0;
            float average = 0.0;
            float stDev = 0.0;

            do {
              int total = 0;
              int rValues[totalReadingsPerSensor];
              Serial.print("Attempt #");
              Serial.print(sensorReadAttempts+1);
              Serial.print(" for sensor #");
              Serial.print(i);
              Serial.print(": [ ");

              for (int k = 0; k < totalReadingsPerSensor; k++) {
                int reading = sonarSensors[i].ping_in();
                int readAttempts = 1;
                while (reading == 0 && readAttempts < 10) {
                  // We want to discard readings of 0 as they're likely erroneous
                  reading = sonarSensors[i].ping_in();
                  readAttempts++;
                  delay(delayBetweenPulses);
                }

                rValues[k] = reading;
                total = total + reading;

                Serial.print(reading);
                Serial.print(" ");

                delay(delayBetweenPulses);
              }
              Serial.println("]");

              // Get the average
              average = float(total) / float(totalReadingsPerSensor);

              Serial.print("Average: ");
              Serial.print(average);
              Serial.println("");

              // Get the std deviation
              float sqDevSum = 0.0;
              for (int v = 0; v < totalReadingsPerSensor; v++) {
                // pow(x, 2) is x squared.
                sqDevSum += pow((average - float(rValues[v])), 2);
              }

              // Calculate the standard deviation so we know if any readings were potentially bogus
              stDev = sqrt(sqDevSum / float(totalReadingsPerSensor));
              Serial.print("Std Dev: ");
              Serial.println(stDev);
              Serial.println("");

              sensorReadAttempts++;

            } while (stDev > highestStdDevAllowed && sensorReadAttempts < totalRedoCountForStdDev);
            // Keep reading until the Std Dev is appropriate or until we've reached out maximum attempts

            // Add the average to the response if we passed the standard deviation test (or reached max attempts)
            // We don't care about float precision so we round to an integer.
            response = response + int(average);

            // If this isn't the last reading, append a "," to save in JSON form
            if (i < numSensors - 1) {
              response = response + ",";
            }
          }

          // End the JSON-formatted response
          response = response + "]}";

          // Send the response
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
