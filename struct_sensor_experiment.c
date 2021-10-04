
//works with a static test value for the sensors

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
const unsigned int writeInterval = 30 * 1000; // write interval (in ms):30s

// ASKSENSORS API config
const char *host = "api.asksensors.com"; // API host name
const int httpPort = 80;                 // port

const char *apiKeyIn1 = "hZlLh2GB12ZmbIs2rfE2sgDDbKYTVVbZ";
const char *apiKeyIn2 = "MIXvgXmJyNlRG94TrvJq9lUhNhZhnqq9";

// DHT11 sensor
#define DHTPIN 16 // Digital pin connected to the DHT sensor
#define DHTTYPE DHT11
DHT dht(DHTPIN, DHTTYPE); // Initialize DHT sensor
float dht_temp = dht.readTemperature();
float dht_hum = dht.readHumidity();

// PIR sensor
int HW740Pin = 26;
bool motionDetected = digitalRead(HW740Pin);

// DS18B20 (Dallas) temp sensor
const int oneWireBus = 4;
OneWire oneWire(oneWireBus);
DallasTemperature sensors(&oneWire);

// TEMT6000 sensor
const int LumSensorPin = 34; //default pin for TEMT6000 sensor
float luminosity = analogRead(LumSensorPin);

//BMP280 sensor
//default pins for the ESP32 are SCL to 22 and SDA to 21
//check I2C address in case of failure, in this case it was 0x76, library settings modified
Adafruit_BMP280 bmp; // I2C

struct SENSOR
{
    String type;
    String apiKeyIn;
    float reading; //maybe this should be a pointer?
    int module;
};

struct SENSOR sensor_array[] = {

    {"DHT11_temp", apiKeyIn1, dht_temp, 1},
    {"DHT11_hum", apiKeyIn1, dht_hum, 3},
    {"TEMT6000_lux", apiKeyIn1, luminosity, 2},
    // {"Dallas_temp", apiKeyIn2, sensors.getTempCByIndex(0), 1},
};

void setup()
{
    Serial.begin(115200);
    bool status;

    /*if (!bmp.begin())
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
*/
    //pinMode(HW740Pin, INPUT);                                               //setting input for PIR motion sensor
    //attachInterrupt(digitalPinToInterrupt(HW740Pin), ISR_function, CHANGE); //interrupt for PIR motion sensor

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
        luminosity = analogRead(LumSensorPin);
        sensor_array[2].reading = luminosity;
        Serial.print("Luminosity: ");
        Serial.println(analogRead(LumSensorPin));
        Serial.print("Humidity:");
        Serial.println(dht.readHumidity());
        /* //Dallas sensor might need this function call first?
        sensors.requestTemperatures();
        float temperature = sensors.getTempCByIndex(0);
        Serial.print(temperature);
        Serial.println("ºC");*/

        //test if struct array works
        Serial.print("Sensor array name[0]");
        Serial.println(sensor_array[0].type);
        Serial.print("Sensor array module[0]");
        Serial.println(sensor_array[0].module);

        for (int i = 0; i < 3; i++)
        {
            String url = "http://api.asksensors.com/write/";
            url += sensor_array[i].apiKeyIn;
            url += "?module";
            url += sensor_array[i].module;
            url += "=";
            url += 100; //test value
            Serial.println(url);

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
            url = "";
        }
    }

    client.stop();
    delay(writeInterval);
}
