//Requirements for grade of 4
//Anna Petelin

//Uses two 'Ask Sensors' sensors, with modules for DHT11, TEMT6000, BMP280, Dallas temp and PIR sensors
//includes an interrupt for PIR sensor

//Sensors with interrupt work on testing with Ask Sensors.
//Problems: seems to be some jitteriness from the PIR, maybe a debouncing problem? Doesn't always update correctly,
//or updates even if state is the same.

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

//insert network configuration
const char *ssid = "";
const char *password = "";

//insert API keys for sensor 1 and sensor 2
const char *apiKeyIn1 = "";
const char *apiKeyIn2 = "";

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

// DS18B20 (Dallas) temp sensor
const int oneWireBus = 4;
OneWire oneWire(oneWireBus);
DallasTemperature sensors(&oneWire);

// PIR sensor global variables
int HW740Pin = 26;
volatile bool FLAG = false;
volatile bool PIRValue = 0;
volatile bool *PIRValuePointer = &PIRValue;

//variables for timer
unsigned long timeRunning = 0;
unsigned long stopWatch = 0;
unsigned long timeSinceRequest = 0;
const long notifyInterval = 30 * 60 * 1000; //API update interval in milliseconds

//interrupt function
void ISR_function() //this should have an IRAM_ATTR tag but VS Code objects
{

    *PIRValuePointer = digitalRead(HW740Pin); //update PIR value
    Serial.println("tripwire - change in PIR sensor detected, Broadcasting from IRS!");
    FLAG = true; //will turn on flag in main loop
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

    dht.begin();                                                            //start DHT sensor
    pinMode(HW740Pin, INPUT);                                               //setting input for motion sensor
    attachInterrupt(digitalPinToInterrupt(HW740Pin), ISR_function, CHANGE); //interrupt for motion sensor
    sensors.begin();                                                        //start Dallas sensor

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
            delay(1000);  //delay for a second to deal with PIR's over-excitement before reversing flag
            FLAG = false; // re-initialise flag
        }
        else
        {
            Serial.println("no change in PIR");
        }

        timeRunning = millis();
        timeSinceRequest = timeRunning - stopWatch; //updating the timer

        if (timeSinceRequest >= notifyInterval) //check the timer for sending sensor readings
        {
            //read off sensor values:
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

            //Dallas reading (Temperature)
            sensors.requestTemperatures(); //this method needs to be called first to get temp
            float dallas_temp = sensors.getTempCByIndex(0);
            Serial.print(dallas_temp);
            Serial.println("ºC");

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

            //send first url request
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
            url = "http://api.asksensors.com/write/";

            //URL for updating sensor 2
            url += apiKeyIn2;
            url += "?module1=";
            url += dallas_temp;

            delay(2000); // wait a bit to avoid HTTP 429 error
            //send second url request
            ask.begin(url);
            //Check for the returning code
            httpCode = ask.GET();

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
    }
    client.stop();
}
