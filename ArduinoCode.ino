#include <WiFi.h>
#include <WiFiClient.h>
#include <WebServer.h>
#include <PubSubClient.h>
#include "time.h"
#include "HTTPClient.h"
#include <ThingSpeak.h>
#include <Servo.h>
#include <ArduinoJson.h>
#include "DHT.h"
#include <Adafruit_MQTT.h>
#include <Adafruit_MQTT_Client.h>
#define DHTPIN 25
#define DHTTYPE DHT11
#define BUZZER_PIN  4
#define POWER_PIN   14
#define SIGNAL_PIN  35

Servo myservo1;
Servo myservo2_website;
DHT dht(DHTPIN, DHTTYPE);

const char* ssid = "Chinni";
const char* password = "silence2004";
int pos = 0;
int pos1 = 0;
const int trigPin = 12;
const int echoPin = 26;
long duration;
int distance;
int SENSOR_MIN = 0;
int SENSOR_MAX = 255;
int required = 15;
float value = 0;
int touchState;

WiFiServer server(80);
String cse_ip = "192.168.153.147";
String cse_port = "8080";
String serverom2m = "http://192.168.153.147:8080//in-cse/in-name/"; //+ cse_ip + ":" + cse_port + "//in-cse/in-name/";
String ae1 = "DHT11";
String ae2 = "Ultrasonic-sensor";
String ae3 = "WaterlevelSensor";
String cnt1 = "Temperature";
String cnt2 = "Humidity";
String cnt3 = "Distance";
String cnt4 = "Waterlevel";
String header;

// Auxiliar variables to store the current output state
String output26State = "off"; //orange led
String output27State = "off"; //blue led
String output28State = "off"; // motor

// Assign output variables to GPIO pins
const int output26 = 2;  // Orange LED
const int output27 = 18;  // blue LED

// Current time
unsigned long currentTime = millis();
// Previous time
unsigned long previousTime = 0;
// Define timeout time in milliseconds (example: 2000ms = 2s)
const long timeoutTime = 2000;
const char* server_mqtt = "mqtt3.thingspeak.com";
const char* clientID = "OSk2MTIgCTwNKCU4FCkxND0";
const char* mqttUserName = "OSk2MTIgCTwNKCU4FCkxND0";
const char* mqttPass = "Jo8fuK7q1Q7oj9FnRbdcLZlD";
int writeChannelID = 1757885;
char* writeAPIKey = "P13FYBKP9JQWTVDF";

WiFiClient client;
PubSubClient mqttClient(server_mqtt, 1883, client);

void createCI(String val, String ae, String cnt)
{
  HTTPClient http;
  http.begin(serverom2m + ae + "/" + cnt + "/");

  http.addHeader("X-M2M-Origin", "admin:admin");
  http.addHeader("Content-Type", "application/json;ty=4");

  int code = http.POST("{\"m2m:cin\": {\"cnf\":\"application/json\",\"con\": \"" + String(val) + "\"}}");

  if (code == -1) {
    Serial.println("UNABLE TO CONNECT TO THE SERVER");
  }
  http.end();
}

void setup() {
  Serial.begin(115200);
  myservo1.attach(21);                 // to control using ultrasonic sensor
  myservo2_website.attach(33);        //to control using website
  pinMode(output26, OUTPUT);
  pinMode(output27, OUTPUT);
  pinMode(trigPin, OUTPUT);
  pinMode(echoPin, INPUT);
  pinMode(BUZZER_PIN, OUTPUT);
  pinMode(POWER_PIN, OUTPUT);
  digitalWrite(POWER_PIN, LOW);
  digitalWrite(BUZZER_PIN,   HIGH);
  digitalWrite(output26, LOW);
  digitalWrite(output27, LOW);

  // Connect to Wi-Fi network with SSID and password
  Serial.print("Connecting to ");
  Serial.println(ssid);
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  // Print local IP address and start web server
  Serial.println("");
  Serial.println("WiFi connected.");
  Serial.println("IP address: ");
  Serial.println(WiFi.localIP());
  mqttClient.setServer (server_mqtt, 1883 );
  dht.begin();
  server.begin();
}

void loop() {
  if (mqttClient.connected() == NULL)
  {
    if (mqttClient.connect(clientID, mqttUserName, mqttPass))
    {
      Serial.println("connected");
    }
    else {

      Serial.println(mqttClient.state());
    }
  }
  float h = dht.readHumidity();
  float t = dht.readTemperature();

  if (isnan(h) || isnan(t)) {
    Serial.println("Failed to read from DHT sensor!");
    return;
  }
  Serial.println("Data collected using DHT11 Sensor");
  Serial.print("Temperature: ");
  Serial.println(t);

  Serial.print("Humidity: ");
  Serial.println(h);
  int fieldsToPublish[8] = {1, 2, 0, 0, 0, 0, 0, 0};

  String dataString1 = "field1=" + String(h);
  String dataString2 = "field2=" + String(t);

  String topicString = "channels/" + String( writeChannelID ) + "/publish";

  //Serial.println(topicString);
  mqttClient.publish(topicString.c_str(), dataString1.c_str());
  mqttClient.publish(topicString.c_str(), dataString2.c_str());
  Serial.print("Published to MQTT Server : ");
  mqttClient.loop();
  WiFiClient client = server.available();   // Listen for incoming clients

  if (client) {                             // If a new client connects,
    currentTime = millis();
    previousTime = currentTime;
    Serial.println("New Client.");          // print a message out in the serial port
    String currentLine = "";                // make a String to hold incoming data from the client
    while (client.connected() && currentTime - previousTime <= timeoutTime) {  // loop while the client's connected
      currentTime = millis();
      if (client.available()) {             // if there's bytes to read from the client,
        char c = client.read();             // read a byte, then
        Serial.write(c);                    // print it out the serial monitor
        header += c;
        if (c == '\n') {                    // if the byte is a newline character
          // if the current line is blank, you got two newline characters in a row.
          // that's the end of the client HTTP request, so send a response:
          if (currentLine.length() == 0) {
            // HTTP headers always start with a response code (e.g. HTTP/1.1 200 OK)
            // and a content-type so the client knows what's coming, then a blank line:
            client.println("HTTP/1.1 200 OK");
            client.println("Content-type:text/html");
            client.println("Connection: close");
            client.println();

            // turns the GPIOs on and off
            if (header.indexOf("GET /26/on") >= 0) {
              Serial.println("ORANGE LED on");
              output26State = "on";
              digitalWrite(output26, HIGH);
            } else if (header.indexOf("GET /26/off") >= 0) {
              Serial.println("ORANGE LED off");
              output26State = "off";
              digitalWrite(output26, LOW);
            } else if (header.indexOf("GET /27/on") >= 0) {
              Serial.println("BLUE LED on");
              output27State = "on";
              digitalWrite(output27, HIGH);
            } else if (header.indexOf("GET /27/off") >= 0) {
              Serial.println("BLUE LED  off");
              output27State = "off";
              digitalWrite(output27, LOW);
            }  else if (header.indexOf("GET /28/on") >= 0) {
              Serial.println("opening the gate");
              output28State = "on";
              for (pos = 0; pos <= 180; pos += 1)
              {
                myservo2_website.write(pos);
                //Serial.println(pos);
                delay(100);
              }
            } else if (header.indexOf("GET /28/off") >= 0) {
              Serial.println("closing the gate");
              output28State = "off";
              for (pos = 180; pos >= 0; pos -= 1)
              {
                myservo2_website.write(pos);
                //Serial.println(pos);
                delay(100);
              }
            }

            // Display the HTML web page
            client.println("<!DOCTYPE html><html>");
            client.println("<head><meta name=\"viewport\" content=\"width=device-width, initial-scale=1\">");
            client.println("<link rel=\"icon\" href=\"data:,\">");
            // CSS to style the on/off buttons
            // Feel free to change the background-color and font-size attributes to fit your preferences
            client.println("<style>html { font-family: Helvetica; display: inline-block; margin: 0px auto; text-align: center;}");
            client.println(".button { background-color: #4CAF50; border: none; color: white; padding: 16px 40px;");
            client.println("text-decoration: none; font-size: 30px; margin: 2px; cursor: pointer;}");
            client.println(".button2 {background-color: #555555;}</style></head>");

            // Web Page Heading
            client.println("<body><h1>ESP32 Web Server</h1>");

            // Display current state, and ON/OFF buttons for GPIO 26
            client.println("<p>ORANGE LED - State " + output26State + "</p>");
            // If the output26State is off, it displays the ON button
            if (output26State == "off") {
              client.println("<p><a href=\"/26/on\"><button class=\"button\">ON</button></a></p>");
            } else {
              client.println("<p><a href=\"/26/off\"><button class=\"button button2\">OFF</button></a></p>");
            }

            // Display current state, and ON/OFF buttons
            client.println("<p>BLUE LED - State " + output27State + "</p>");
            // If the output27State is off, it displays the ON button
            if (output27State == "off") {
              client.println("<p><a href=\"/27/on\"><button class=\"button\">ON</button></a></p>");
            } else {
              client.println("<p><a href=\"/27/off\"><button class=\"button button2\">OFF</button></a></p>");
            }

            client.println("<p> Shutter Controls" + output28State + "</p>");
            // If the output27State is off, it displays the ON button
            if (output28State == "off") {
              client.println("<p><a href=\"/28/on\"><button class=\"button\">ON</button></a></p>");
            } else {
              client.println("<p><a href=\"/28/off\"><button class=\"button button2\">OFF</button></a></p>");
            }
            client.println("</body></html>");

            // The HTTP response ends with another blank line
            client.println();
            // Break out of the while loop
            break;
          } else { // if you got a newline, then clear currentLine
            currentLine = "";
          }
        } else if (c != '\r') {  // if you got anything else but a carriage return character,
          currentLine += c;      // add it to the end of the currentLine
        }
      }
    }
    // Clear the header variable
    header = "";
    // Close the connection
    client.stop();
    Serial.println("Client disconnected.");
    Serial.println("");
  }
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);
  duration = pulseIn(echoPin, HIGH);
  distance = duration * 0.034 / 2;
  Serial.println("Data collected using Ultrasonic Sensor");
  Serial.print("Distance in cm: ");
  Serial.println(distance);

 if (distance < 15)
 {
    for (pos1 = 0; pos1 <= 95; pos1 += 1)
    {
      myservo1.write(pos1);
      delay(100);
    }
    delay(1500);
 }
  digitalWrite(POWER_PIN, HIGH);  // turn the sensor ON
  delay(10);                      // wait 10 milliseconds
  value = analogRead(SIGNAL_PIN);// read the analog value from sensor
  Serial.println("Data collected using Waterlevel Sensor");
  Serial.print("Water level :");
  Serial.println(value);
  digitalWrite(POWER_PIN, LOW);   // turn the sensor OFF
  float level = map(value, SENSOR_MIN, SENSOR_MAX, 0, 4); // 4 levels
  Serial.print("Water height: ");
  Serial.println(level);

  if (level < required)
  {
    Serial.println("Water is low");

    digitalWrite(BUZZER_PIN, HIGH);
  }
  else if (level > required)
  {
    Serial.println("Water is High");

    digitalWrite(BUZZER_PIN, LOW);
  }
  String temperature = String(t);
  createCI(temperature, ae1, cnt1);
  String humidity = String(h);
  createCI(humidity, ae1, cnt2);
  String d = String(distance);
  createCI(d, ae2, cnt3);
  String l = String(level);
  createCI(l, ae3, cnt4);
}
