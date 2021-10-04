
//Sensor libraries
#include <Wire.h>
#include <Adafruit_Sensor.h>
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
const unsigned int writeInterval = 30 * 1000; // write interval (in ms): 30 s

// ASKSENSORS API host config
const char *host = "api.asksensors.com"; // API host name
const int httpPort = 80;                 // port

// DS18B20 (Dallas) temp sensor
const int oneWireBus = 4;
OneWire oneWire(oneWireBus);
DallasTemperature sensors(&oneWire);

// TEMT6000 sensor
const int LumSensorPin = 34; //default pin for TEMT6000 sensor

void setup()
{
    Serial.begin(115200);
    bool status;

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

        //Dallas reading (Temperature)
        sensors.requestTemperatures(); //this method needs to be called first
        float temperature = sensors.getTempCByIndex(0);
        Serial.print(temperature);
        Serial.println("ºC");

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