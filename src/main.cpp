#include <Arduino.h>
#include <ESP8266WiFi.h>          // brings in wifi tools
#include <WiFiUdp.h>              // used for NTP UDP packets
#include <NTPClient.h>            // NTP client
#include <DNSServer.h>            // Used for the WiFiManager
#include <ESP8266WebServer.h>     // Enables web server for the WiFiManager
#include <WiFiManager.h>          // https://github.com/tzapu/WiFiManager WiFi Configuration Magic
#include <FS.h>                   // SPI File system support for saving variables
#include <Ticker.h>               // time scheduling for function intervals
#include <Adafruit_Sensor.h>      // adafruit unified sensor driver
#include <Adafruit_BME280.h>      // BME280 sensor driver (temperature, humidity, pressure)
#include <SPI.h>                  // used for the SPIFFS filesystem driver
#include <PubSubClient.h>         // MQTT publish-subscribe

ADC_MODE(ADC_VCC);                // set the internal ADC mux to read system voltage
                                  // TODO: check to see if this is actually working properly!

#define sensorReadSeconds 10      // interval between sensor readings/publish

Adafruit_BME280 bme;
WiFiUDP ntpUDP;
WiFiManager wifi;
Ticker ticker;
NTPClient timeClient(ntpUDP);
WiFiClient espClient;
PubSubClient client(espClient);

char node_name[40];                       // stores our node-name, used for MQTT topic creation
char mqtt_server[40] = "192.168.1.100";   // IP Address of our MQTT server
char mqtt_user[40];                       // username for our MQTT server
char mqtt_pass[40];                       // password for our MQTT server

bool shouldSaveConfig = false;
bool ledState;
float tempInF;
float tempInC;
float humidity;
float pressure;
float voltage;
unsigned long lastSensorRead;             // millis() of our last sensor reading

String mqtt_topic_sensor_midfix = "/sensor/";   // middle portion of the MQTT topic for sensors
String mqtt_topic_info_midfix = "/info/";       // middle portion of the MQTT topic for other info
String mqtt_topic_prefix;                       // beginning portion of the MQTT topic. set to node-name after config

void tick() {
  if (ledState) {
    digitalWrite(LED_BUILTIN, HIGH);
    ledState = false;
  } else {
    digitalWrite(LED_BUILTIN, LOW);
    ledState = true;
  }
}

void ledBlink() { // blink the built in LED really quickly
  digitalWrite(LED_BUILTIN, LOW);
  delay(10);
  digitalWrite(LED_BUILTIN, HIGH);
}

void saveConfigCallback() { // tells the system to save our configuration values
  Serial.println("Saving config");
  shouldSaveConfig = true;
}

void configModeCallback(WiFiManager *myWiFiManager) { // called when the node enters config mode
  Serial.println("Entered config mode");
  Serial.print("Please connect to:");
  Serial.println(myWiFiManager->getConfigPortalSSID());
  ticker.attach(0.2, tick); // blinks the LED quickly
}

void reconnect() { // reconnects to the MQTT server if disconnected
  ticker.detach();
  while (!client.connected()) {
    Serial.print("Attempting MQTT connection...");
    if (client.connect(node_name, mqtt_user, mqtt_pass)) {
      Serial.println("Connected!");
    } else {
      Serial.print("failed, rc=");
      Serial.print(client.state());
      Serial.println(" try again in 5 seconds");
      delay(5000);
    }
  }
}

void updateMQTT() { // publishes sensor readings to MQTT
  String sensorTopicPrefix = mqtt_topic_prefix + mqtt_topic_sensor_midfix;
  String infoTopicPrefix = mqtt_topic_prefix + mqtt_topic_info_midfix;

  client.publish(String(sensorTopicPrefix + "temperature").c_str(), String(tempInF).c_str());
  client.publish(String(sensorTopicPrefix + "temperatureInC").c_str(), String(tempInC).c_str());
  client.publish(String(sensorTopicPrefix + "humidity").c_str(), String(humidity).c_str());
  client.publish(String(sensorTopicPrefix + "pressure").c_str(), String(pressure).c_str());
  client.publish(String(sensorTopicPrefix + "voltage").c_str(), String(voltage).c_str());
  client.publish(String(infoTopicPrefix + "lastContact").c_str(), timeClient.getFormattedTime().c_str());
}

void updateSensor() { // grabs readings from the sensors
  if (millis() - lastSensorRead >= sensorReadSeconds * 1000) { // check to see if we are more than sensorReadSeconds from last reading
    lastSensorRead = millis();
    tempInC = bme.readTemperature();
    tempInF = (tempInC * 1.8) + 32;
    pressure = bme.readPressure();
    humidity = bme.readHumidity();
    voltage = ESP.getVcc();

    updateMQTT(); // publish values to MQTT
  }
}

void setup() { // run configuration if needed, connect to wifi, etc
  tempInC = 0;
  tempInF = 0;
  humidity = 0;
  pressure = 0;
  voltage = 0;
  lastSensorRead = 0;

  Serial.begin(115200);
  Serial.println();
  Serial.println();

  if (!bme.begin(0x76)) { // attempt to initialize BME280 sensor
    Serial.println("Could not initialize sensor!");
    ESP.reset();
  }

  pinMode(LED_BUILTIN, OUTPUT);
  pinMode(0, INPUT);            // FLASH button as input for resetting wifi config

  ticker.attach(0.5, tick);     // blink led slowly

  // TODO: Make the following nicer, should probably put everything in one file
  if (SPIFFS.begin()) { // start SPIFFS file system for loading settings
    Serial.println("mounted file system");
    Serial.println("Reading node name");
    File nodename_file = SPIFFS.open("/name.txt", "r");
    File mqtt_server_file = SPIFFS.open("/mqttserver.txt", "r");
    File mqtt_user_file = SPIFFS.open("/mqttuser.txt", "r");
    File mqtt_pass_file = SPIFFS.open("/mqttpass.txt", "r");

    if (nodename_file) {
      size_t size = nodename_file.size();
      nodename_file.readBytes(node_name, size);
      Serial.println(node_name);
    }

    if (mqtt_server_file) {
      size_t size = mqtt_server_file.size();
      mqtt_server_file.readBytes(mqtt_server, size);
      Serial.println(mqtt_server);
    }

    if (mqtt_user_file) {
      size_t size = mqtt_user_file.size();
      mqtt_user_file.readBytes(mqtt_user, size);
      Serial.println(mqtt_user);
    }

    if (mqtt_pass_file) {
      size_t size = mqtt_pass_file.size();
      mqtt_pass_file.readBytes(mqtt_pass, size);
      for (int i = 0; i < size; i++) {
        Serial.print("*");
      }
      Serial.println();
    }

    nodename_file.close();
    mqtt_server_file.close();
    mqtt_user_file.close();
    mqtt_pass_file.close();
  }

  wifi.setSaveConfigCallback(saveConfigCallback);
  wifi.setAPCallback(configModeCallback);

  WiFiManagerParameter node_name_param("nodename", "Node Name", node_name, 40);             // Have the captive portal ask for Node Name
  WiFiManagerParameter mqtt_server_param("mqttserver", "MQTT Server IP", mqtt_server, 40);  // and MQTT server IP
  WiFiManagerParameter mqtt_user_param("mqttuser", "MQTT Username", mqtt_user, 40);         // and the username
  WiFiManagerParameter mqtt_pass_param("mqttpass", "MQTT Password", mqtt_pass, 40);         // as well as the password NOT MASKED!
                                                                                            // TODO: See if we can mask the password input

  wifi.addParameter(&node_name_param);                                                      // Add the parameters to the actual captive portal
  wifi.addParameter(&mqtt_server_param);
  wifi.addParameter(&mqtt_user_param);
  wifi.addParameter(&mqtt_pass_param);

  wifi.autoConnect();                                                                       // start the WiFiManager / configuration

  // if we got this far, we are connected to wifi
  ticker.detach(); // turn off the LED blinking
  digitalWrite(LED_BUILTIN, HIGH);

  // copy in values from config
  strcpy(node_name, node_name_param.getValue());
  strcpy(mqtt_server, mqtt_server_param.getValue());
  strcpy(mqtt_user, mqtt_user_param.getValue());
  strcpy(mqtt_pass, mqtt_pass_param.getValue());

  // save the configuration values to SPI flash
  if (shouldSaveConfig) {
    Serial.println("Attempting to open config file");
    File nodename_file = SPIFFS.open("/name.txt", "w");
    File mqtt_server_file = SPIFFS.open("/mqttserver.txt", "w");
    File mqtt_user_file = SPIFFS.open("/mqttuser.txt", "w");
    File mqtt_pass_file = SPIFFS.open("/mqttpass.txt", "w");

    if (nodename_file) {
      Serial.print("Saving node name: ");
      nodename_file.print(node_name);
      Serial.println(node_name);
    }

    if (mqtt_server_file) {
      Serial.print("Saving MQTT server: ");
      mqtt_server_file.print(mqtt_server);
      Serial.println(mqtt_server);
    }

    if (mqtt_user_file) {
      Serial.print("Saving MQTT User: ");
      mqtt_user_file.print(mqtt_user);
      Serial.println(mqtt_user);
    }

    if (mqtt_pass_file) {
      Serial.println("Saving MQTT Pass: HIDDEN");
      mqtt_pass_file.print(mqtt_pass);
    }

    nodename_file.close();
    mqtt_server_file.close();
    mqtt_user_file.close();
    mqtt_pass_file.close();
  }

  client.setServer(mqtt_server, 1883); // set the MQTT server
  client.connect(node_name, mqtt_user, mqtt_pass); // connect to the MQTT server

  timeClient.begin(); // start the NTP client

  mqtt_topic_prefix = node_name; // set the MQTT topic prefix to the node name
}

void loop() {
  // check if flash button is pressed and held for 2 seconds, if so, reset all settings
  // if button isnt held, reset the cpu
  if (digitalRead(0) == LOW) {
    digitalWrite(LED_BUILTIN, LOW);
    ticker.detach();
    delay(2000);
    if (digitalRead(0) == LOW) {
      wifi.resetSettings();
      for (int i = 0; i < 20; i++) {
        digitalWrite(LED_BUILTIN, LOW);
        delay(75);
        digitalWrite(LED_BUILTIN, HIGH);
        delay(100);
      }
    }
    ESP.reset();
    delay(5000);
  }

  // check if the MQTT server is connected, if not, reconnect
  if (!client.connected()) {
    reconnect();
  }

  client.loop(); // NTP client loop
  // update time if needed
  timeClient.update();

  // update the sensors, then push to MQTT when needed
  updateSensor();
}
