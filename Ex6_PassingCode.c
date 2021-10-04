//Requirements for passing grade
//One sensor module with DHT11, TEMT6000, and BMP280 sensors
//Tested with cloud, working

//Sensor libraries
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BMP280.h>
#include "DHT.h"

//AskSensors connection libraries
#include <WiFi.h>
#include <WiFiMulti.h>
#include <HTTPClient.h>

//AskSensors config and network credentials
WiFiMulti WiFiMulti;
HTTPClient ask;

//Insert netwoek credentials
const char *ssid = "";
const char *password = "";
//Insert apiKey
const char *apiKeyIn = "";
const unsigned int writeInterval = 30 * 60 * 1000; // write interval (in ms): 30 min

// ASKSENSORS API host config
const char *host = "api.asksensors.com"; // API host name
const int httpPort = 80;                 // port

// DHT11 sensor
#define DHTPIN 16 // Digital pin connected to the DHT sensor
#define DHTTYPE DHT11

DHT dht(DHTPIN, DHTTYPE); // Initialize DHT sensor

// TEMT6000 sensor
const int LumSensorPin = 34; //default pin for TEMT6000 sensor

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

    dht.begin(); //start DHT sensor

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
        float DHT_humidity = dht.readHumidity();
        Serial.print("Humidity:");
        Serial.print(DHT_humidity, 1);
        Serial.println("%");

        //DHT reading (Temperature)
        float DHT_temp = dht.readTemperature();
        Serial.print("Temperature: ");
        Serial.print(DHT_temp);
        Serial.println(" *C");

        //BMP280 readings (pressure)
        Serial.print("Pressure = ");
        Serial.print(bmp.readPressure());
        Serial.println(" Pa");
        float pressure = bmp.readPressure();

        // URL for updating module1 and module 2
        String url = "http://api.asksensors.com/write/";
        url += apiKeyIn;
        url += "?module1=";
        url += DHT_temp;
        url += "&module2=";
        url += luminosity;
        url += "&module3=";
        url += DHT_humidity;
        url += "&module4=";
        url += pressure;

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