/*
Anna Petelin

Uses two 'Ask Sensors' sensors, with modules for DHT11, TEMT6000, BMP280, Dallas temp and PIR sensors
includes an interrupt for PIR sensor
Extracts JSON data from openweathermaps and send to Ask Sensors.*/

//PIR interrupt is a bit buggy

//Sensor libraries
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BMP280.h>
#include "DHT.h"
#include <OneWire.h>
#include <DallasTemperature.h>

//API connection libraries
#include <WiFi.h>
#include <WiFiMulti.h>
#include <HTTPClient.h>
#include <ArduinoJson.h>

//AskSensors config and network credentials
WiFiMulti WiFiMulti;
HTTPClient ask;

//insert network configuration
const char *ssid = "DNA-WLAN-2G-A178";
const char *password = "64957969891";

//insert API keys for sensor 1, sensor 2
const char *apiKeyIn1 = "hZlLh2GB12ZmbIs2rfE2sgDDbKYTVVbZ";
const char *apiKeyIn2 = "MIXvgXmJyNlRG94TrvJq9lUhNhZhnqq9";

// ASKSENSORS API host config
const char *host = "api.asksensors.com"; // API host name
const int httpPort = 80;                 // port

//Openweather maps
String app_id = "6f830172b6d7108b4a5dfcff5c0ca21a"; //insert openweathermaps app id
String weatherURL = "http://api.openweathermap.org/data/2.5/weather?q=Kajaani&units=metric&appid=";
float kajaani_temp;

// DHT11 sensor
#define DHTPIN 16 // Digital pin connected to the DHT sensor
#define DHTTYPE DHT11
float DHT_humidity = 0;
float DHT_temp = 0;

DHT dht(DHTPIN, DHTTYPE); // Initialize DHT sensor

// TEMT6000 sensor
const int LumSensorPin = 34; //default pin for TEMT6000 sensor
int luminosity = 0;

//BMP280 sensor
//default pins for the ESP32 are SCL to 22 and SDA to 21
//check I2C address in case of failure, in this case it was 0x76, library settings modified
Adafruit_BMP280 bmp; // I2C
float pressure = 0;

// DS18B20 (Dallas) temp sensor
const int oneWireBus = 4;
OneWire oneWire(oneWireBus);
DallasTemperature sensors(&oneWire);
float dallas_temp = 0;

// PIR sensor global variables
int HW740Pin = 26;
volatile bool FLAG = false;
volatile bool PIRValue = 0;
volatile bool *PIRValuePointer = &PIRValue;

//variables for timer
unsigned long timeRunning = 0;
unsigned long stopWatch = 0;
unsigned long timeSinceRequest = 0;
const long notifyInterval = 20 * 1000; //API update interval in milliseconds, 30 mins

//interrupt function
void ISR_function_1()
{
    //if (!(micros() < 60000000))
    //{
    *PIRValuePointer = digitalRead(HW740Pin); //update PIR value
    Serial.println("tripwire - change in PIR sensor detected, Broadcasting from IRS!");
    FLAG = true; //will turn on flag in main loop
    //}
}

// function to send HTTP request when PIR sensor changes state
void updateAPI(bool value)
{
    String url = "http://api.asksensors.com/write/";
    url += apiKeyIn2;
    url += "?module2=";
    url += value;
    url += "&module3=";
    url += value;

    GETRequest(url);
}

//function fro making HTTP GET requests to the API
String GETRequest(String url)
{
    ask.begin(url);
    //Check for the returning code
    int httpCode = ask.GET();
    String payload = "";

    if (httpCode > 0)
    {
        payload = ask.getString();
        Serial.println(httpCode);
        Serial.println(payload);
    }
    else
    {
        Serial.println("Error on HTTP request");
    }
    ask.end();
    delay(1000);
    return payload;
}

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

    dht.begin();     //start DHT sensor
    sensors.begin(); //start Dallas sensor

    pinMode(HW740Pin, INPUT);                                                 //setting input for motion sensor
    attachInterrupt(digitalPinToInterrupt(HW740Pin), ISR_function_1, CHANGE); //interrupt for motion sensor

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
    WiFiClient client; //check if wifi is still connected
    if (!client.connect(host, httpPort))
    {
        Serial.println("Error connecting");
        return;
    }
    else //if ok, execute the rest of the loop
    {

        if (FLAG == true) //first check for notifications from interrupt on each loop iteration
        {
            updateAPI(PIRValue);
            delay(1000);
            FLAG = false; // re-initialise flag
        }

        timeRunning = millis();
        timeSinceRequest = timeRunning - stopWatch; //updating the timer

        if (timeSinceRequest >= notifyInterval) //check the timer for sending sensor readings
        {
            //read off sensor values:
            Serial.println("Printing sensor readings:");
            //luminosity readings
            luminosity = analogRead(LumSensorPin);
            Serial.print(luminosity);
            Serial.println(" lux");

            //DHT reading (Humidity)
            DHT_humidity = dht.readHumidity();
            Serial.print("Humidity:");
            Serial.print(DHT_humidity, 1);
            Serial.println("%");

            //DHT reading (Temperature)
            DHT_temp = dht.readTemperature();
            Serial.print("Temperature: ");
            Serial.print(DHT_temp);
            Serial.println(" *C");

            //BMP280 readings (pressure)
            Serial.print("Pressure = ");
            Serial.print(bmp.readPressure());
            Serial.println(" Pa");
            pressure = bmp.readPressure();

            //Dallas reading (Temperature)
            sensors.requestTemperatures(); //this method needs to be called first to get temp
            dallas_temp = sensors.getTempCByIndex(0);
            Serial.print(dallas_temp);
            Serial.println("ºC");

            //Fetch JSON data from openweathermaps and de-serialize the JSON object that has been returned as a string
            String path = weatherURL + app_id;
            String data = GETRequest(path);
            delay(1000);
            //deserialisation code provided by tool at https://arduinojson.org/

            StaticJsonDocument<1024> doc;
            DeserializationError error = deserializeJson(doc, data);

            if (error)
            {
                Serial.print(F("deserializeJson() failed: "));
                Serial.println(error.f_str());
                return;
            }

            JsonObject main = doc["main"];
            float main_temp = main["temp"];

            kajaani_temp = main_temp;

            Serial.print("Temp in Kajaani: ");
            Serial.println(main_temp);
        }
        else
        {
            Serial.println("Error on HTTP request");
        }

        ask.end();
        delay(2000);

        // URL for updating sensor 1
        String url = "http://api.asksensors.com/write/";
        url += apiKeyIn1;
        url += "?module1=";
        url += DHT_temp;
        url += "&module2=";
        url += luminosity;
        url += "&module3=";
        url += DHT_humidity;
        url += "&module4=";
        url += pressure;

        //send sensor1 request to Ask Sensors
        GETRequest(url);
        delay(2000); // wait a bit to avoid HTTP 429 error

        url = "http://api.asksensors.com/write/"; //re-initialise url
        //URL for updating sensor 2
        url += apiKeyIn2;
        url += "?module1=";
        url += dallas_temp;
        url += "&module4=";
        url += kajaani_temp; //parsed JSON data goes here

        GETRequest(url);
    }
    client.stop();
}
