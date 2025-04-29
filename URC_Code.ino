/* 
  Smart Agriculture Monitoring and Control System
    - Created using Arduino GIGA R1 WiFi
    - Reads temperature, humidity, soil moisture, electrical conductivity (EC), pH, barometric pressure, ambient light, and proximity
    - Sends sensor data to a Flask web server via WiFi
    - Automatically waters and fertilizes Zebra plant when soil conditions fall below a set threshold

  Project submitted for:
  Spring 2025 Feigenbaum Undergraduate Research Scholarship and Conference
  Massachusetts College of Liberal Arts (MCLA)  

  Developed by: Thomas Bruso 
    Major: Computer Science
    Concentration: Electrical Engineering
  Project Duration: Spring 2025 Semester

  Hardware:
    - Arduino GIGA R1 WiFi
    - DHT11 Temperature and Humidity Sensor
    - Capacitive Soil Moisture Sensor
    - RS485 Soil EC Sensor
    - TTL to RS485 Converter Module
    - Analog pH Sensor
    - BMP085 Barometric Pressure Sensor
    - AP3216C Ambient Light and Proximity Sensor
    - Relay Module (Water Pump & Fertilizer Pump)
*/
#include <Wire.h>
#include <Adafruit_BMP085.h>
#include <WiFi.h>
#include <WiFiClient.h>
#include "DHT.h"
#include <ModbusMaster.h>

// Wi-Fi Credentials
const char* ssid = "Fidium";
const char* password = "*************";

// Flask Server
const char* server = "3.19.221.244"; 
const int port = 80;
const String endpoint = "/sensor-data";

// I2C Address 
#define AP3216C_I2C_ADDRESS 0x1E

// DHT11 (humidity and temperature) Setup 
#define DHTPIN 2
#define DHTTYPE DHT11
DHT dht(DHTPIN, DHTTYPE);

// pH Sensor Pin 
#define PH_PIN A1

// Soil Moisture Pin  
#define SOIL_MOISTURE_PIN A0

// Relay Pins 
#define FERTILIZER_RELAY_PIN 5
#define WATER_RELAY_PIN 6

// Thresholds
const float EC_THRESHOLD = 0.8;                       // 1-2 mS/cm is optimal. < 00.8 mS/cm → fertilize 
const int MOISTURE_THRESHOLD = 40;                    // 40-60% is optimal. < 40% → water
const unsigned long PUMP_ACTIVATION_DURATION = 2000;  // activate for 2 seconds

// Create barometric pressure sensor and WiFi client objects
Adafruit_BMP085 bmp;
WiFiClient client;

// Create Modbus Master object for soil EC sensor  
ModbusMaster ecSensor;

void setup() {
  Serial.begin(115200);
  Serial1.begin(9600); 
  
  pinMode(4, OUTPUT); 
  digitalWrite(4, LOW);

  ecSensor.begin(1, Serial1);  

  // Relay pin setup
  pinMode(FERTILIZER_RELAY_PIN, OUTPUT);
  pinMode(WATER_RELAY_PIN, OUTPUT);
  digitalWrite(FERTILIZER_RELAY_PIN, LOW);
  digitalWrite(WATER_RELAY_PIN, LOW);

  delay(1000);

  // Conenct to Wi-Fi
  Serial.println("Connecting to WiFi...");
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("\n WiFi Connected!");
  Serial.print("IP: ");
  Serial.println(WiFi.localIP());

  Wire.begin();
  dht.begin();

  if (!bmp.begin()) {
    Serial.println("BMP180 failed to initialize.");
    while (1);
  }

  Wire.beginTransmission(AP3216C_I2C_ADDRESS);
  Wire.write(0x00);
  Wire.write(0x03);
  Wire.endTransmission();

  Serial.println("Sensors initialized.");
}

void loop() {
  float humidity = dht.readHumidity();
  float temperatureC = dht.readTemperature();
  float temperatureF = temperatureC * 1.8 + 32;

  if (isnan(humidity) || isnan(temperatureC)) {
    Serial.println(" Failed to read from DHT11 sensor.");
    delay(3000);
    return;
  }

  float soilMoisture = readSoilMoisture();
  float ecValue = readEC();
  float phValue = readPH();

  float pressure = bmp.readPressure() / 100.0;
  uint16_t ambientLight = readAmbientLight();
  float proximity = readNormalizedProximity();

  // Activate water pump if soil moisture is too low
  if (soilMoisture < MOISTURE_THRESHOLD) {
    Serial.println("Soil moisture low! Activating water pump...");
    digitalWrite(WATER_RELAY_PIN, HIGH);
    delay(PUMP_ACTIVATION_DURATION);
    digitalWrite(WATER_RELAY_PIN, LOW);
    Serial.println("Watering complete.");
  }

  // Activate fertilizer pump if EC is too low
  if (ecValue < EC_THRESHOLD) {
    Serial.println("EC level low! Activating fertilizer pump...");
    digitalWrite(FERTILIZER_RELAY_PIN, HIGH);
    delay(PUMP_ACTIVATION_DURATION);
    digitalWrite(FERTILIZER_RELAY_PIN, LOW);
    Serial.println("Fertilizer delivery complete.");
  }

  // Serial monitor debug 
  Serial.println("-----------");
  Serial.print("Temp (F): "); Serial.println(temperatureF);
  Serial.print("Moisture (%): "); Serial.println(soilMoisture);
  Serial.print("Humidity (%): "); Serial.println(humidity);
  Serial.print("EC (mS/cm): "); Serial.println(ecValue);
  Serial.print("pH: "); Serial.println(phValue);
  Serial.print("Pressure (hPa): "); Serial.println(pressure);
  Serial.print("Light (lux): "); Serial.println(ambientLight);
  Serial.print("Proximity (cm): "); Serial.println(proximity);

  // Build JSON containing data to send to server
  String json = String("{\"temperature\":") + temperatureF +
                ",\"moisture\":" + soilMoisture +
                ",\"humidity\":" + humidity +
                ",\"ec\":" + ecValue +
                ",\"ph\":" + phValue +
                ",\"pressure\":" + pressure +
                ",\"light\":" + ambientLight +
                ",\"proximity\":" + proximity + "}";

  if (client.connect(server, port)) {
    client.println("POST " + endpoint + " HTTP/1.1");
    client.println("Host: " + String(server));
    client.println("Content-Type: application/json");
    client.print("Content-Length: ");
    client.println(json.length());
    client.println();
    client.println(json);

    while (client.connected()) {
      String line = client.readStringUntil('\n');
      if (line == "\r") break;
    }
    client.stop();
    Serial.println(" Sent to Flask");
  } else {
    Serial.println(" Failed to connect to Flask");
  }

  delay(3000);
}

// Read capacitive moisture sensor
int readSoilMoisture() {
  int rawValue = analogRead(SOIL_MOISTURE_PIN);
  int moisturePercent = map(rawValue, 1023, 300, 0, 100);
  moisturePercent = constrain(moisturePercent, 0, 100);
  return moisturePercent;
}

// pH calibration constants 
const float neutralVoltage = 2.52;  // Voltage at 7 pH
const float acidSlope = -0.19;      // Voltage change per pH

// Read pH with calibration
float readPH() {
  int rawValue = analogRead(PH_PIN);
  float voltage = rawValue * (5.0 / 1023.0);
  float ph = 7.0 + (voltage - neutralVoltage) / acidSlope;
  return constrain(ph, 0, 14);
}

// Read Electrical Conductivity and convert to mS/cm
float readEC() {
  uint8_t result = ecSensor.readInputRegisters(0x0000, 2); 
  if (result == ecSensor.ku8MBSuccess) {
    uint16_t rawEC = ecSensor.getResponseBuffer(0);
    return rawEC / 1000.0; // (mS/cm)
  } else {
    Serial.println(" Failed to read EC sensor");
    return 0.0;
  }
}

// Read ambient light (lux)
uint16_t readAmbientLight() {
  Wire.beginTransmission(AP3216C_I2C_ADDRESS);
  Wire.write(0x0C);
  Wire.endTransmission(false);
  Wire.requestFrom(AP3216C_I2C_ADDRESS, 2);
  if (Wire.available() == 2) {
    uint8_t low = Wire.read();
    uint8_t high = Wire.read();
    return (high << 8) | low;
  }
  return 0;
}

// Read promixity
uint16_t readRawProximity() {
  Wire.beginTransmission(AP3216C_I2C_ADDRESS);
  Wire.write(0x0E);
  Wire.endTransmission(false);
  Wire.requestFrom(AP3216C_I2C_ADDRESS, 2);
  if (Wire.available() == 2) {
    uint8_t low = Wire.read();
    uint8_t high = Wire.read();
    return ((high & 0x3F) << 8) | low;
  }
  return 0;
}

// Converts raw sensor value into distance (cm), capping it at 5 cm to avoid inaccuracies.
float readNormalizedProximity() {
  uint16_t raw = readRawProximity();
  float prox = map(raw, 17000, 300, 0, 500) / 100.0;
  prox = constrain(prox, 0.0, 5.0);
  return prox;
}
