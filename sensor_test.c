
//Sensor libraries
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BMP280.h>
#include "DHT.h"
#include <OneWire.h>
#include <DallasTemperature.h>

//AskSensors connection libraries
#include <WiFi.h>
#include <WiFiMulti.h>
#include <HTTPClient.h>

//AskSensors config and network credentials
WiFiMulti WiFiMulti;
HTTPClient ask;

//const char *ssid = "Vallikurvit";
//const char *password = "Kotkanpesa";
const char *ssid = "DNA-WLAN-2G-A178";
const char *password = "64957969891";
const char *apiKeyIn = "hZlLh2GB12ZmbIs2rfE2sgDDbKYTVVbZ";
const char *apiKeyIn2 = "MIXvgXmJyNlRG94TrvJq9lUhNhZhnqq9";
const unsigned int writeInterval = 1800 * 1000; // write interval (in ms): 30 minutes

// ASKSENSORS API host config
const char *host = "api.asksensors.com"; // API host name
const int httpPort = 80;                 // port

// DHT11 sensor
#define DHTPIN 16 // Digital pin connected to the DHT sensor
#define DHTTYPE DHT11
DHT dht(DHTPIN, DHTTYPE); // Initialize DHT sensor

// DS18B20 (Dallas) temp sensor
const int oneWireBus = 4;
OneWire oneWire(oneWireBus);
DallasTemperature sensors(&oneWire);

// TEMT6000 sensor
const int LumSensorPin = 34; //default pin for TEMT6000 sensor

// PIR sensor
int HW740Pin = 26;
bool motionDetected = digitalRead(HW740Pin);
//define interrupt function for motion sensor, ie what should happen on state change
void IRAM_ATTR isr()
{
  //send new state to API as soon as a change is detected
}

//BMP280 sensor
//default pins for the ESP32 are SCL to 22 and SDA to 21
//check I2C address in case of failure, in this case it was 0x76, library settings modified
Adafruit_BMP280 bmp; // I2C

void setup()
{
  Serial.begin(115200);
  bool status;
  if (!bmp.begin())
  {
    //if (!bmp.begin()) {
    Serial.println("Could not find a valid BMP280 sensor, check wiring or "
                   "try a different address!");
    while (1)
      delay(10);
  }

  //Default settings for BMP sensor
  bmp.setSampling(Adafruit_BMP280::MODE_NORMAL,     //Operating Mode.
                  Adafruit_BMP280::SAMPLING_X2,     //Temp. oversampling
                  Adafruit_BMP280::SAMPLING_X16,    // Pressure oversampling
                  Adafruit_BMP280::FILTER_X16,      // Filtering.
                  Adafruit_BMP280::STANDBY_MS_500); // Standby time.

  pinMode(HW740Pin, INPUT);                                      //setting input for motion sensor
  attachInterrupt(digitalPinToInterrupt(HW740Pin), isr, CHANGE); //interrupt for motion sensor

  dht.begin();     //start DHT sensor
  sensors.begin(); //start Dallas sensor

  // Connect to Wi-Fi network with SSID and password
  Serial.print("Connecting to ");
  Serial.println(ssid);
  WiFiMulti.addAP(ssid, password);
  while (WiFiMulti.run() != WL_CONNECTED)
  {
    Serial.print(".");
    delay(500);
  }
  // when connected, print IP address
  Serial.println("WiFi connected");
  Serial.println("IP address: ");
  Serial.println(WiFi.localIP());
}

void loop()
{
  WiFiClient client;
  if (!client.connect(host, httpPort))
  {
    Serial.println("Error connecting");
    return;
  }
  else
  {

    //Sensor readings:
    //luminosity readings
    int luminosity = analogRead(LumSensorPin);
    Serial.print(luminosity);
    Serial.println(" lux");

    //DHT reading (Humidity)
    float humidity = dht.readHumidity();
    Serial.print("Humidity:");
    Serial.print(humidity, 1);
    Serial.println("%");

    //DHT reading (Temperature)
    float temp = dht.readTemperature();
    Serial.print("Temperature: ");
    Serial.print(temp);
    Serial.println(" *C");

    //Dallas reading (Temperature)
    sensors.requestTemperatures(); //this method needs to be called first
    float temperature = sensors.getTempCByIndex(0);
    Serial.print(temperature);
    Serial.println("ºC");

    //BMP280 readings (pressure)
    Serial.print("Pressure = ");
    Serial.print(bmp.readPressure());
    Serial.println(" Pa");
    float pressure = bmp.readPressure();

    //PIR readings
    if (motionDetected == 1)
    {
      Serial.println("movement detected");
    }
    else if (motionDetected == 0)
    {
      Serial.println("Noone in the area");
    }

    // URL for updating module1 and module 2
    String url = "http://api.asksensors.com/write/";
    url += apiKeyIn;
    url += "?module1=";
    url += random(10, 100); //
    url += "&module2=";
    url += random(10, 100); //

    //send url request
    ask.begin(url);
    //Check for the returning code
    int httpCode = ask.GET();

    if (httpCode > 0)
    {

      String payload = ask.getString();
      Serial.println(httpCode);
      Serial.println(payload);
    }
    else
    {
      Serial.println("Error on HTTP request");
    }
    ask.end();
  }
  client.stop();
  delay(writeInterval);
}