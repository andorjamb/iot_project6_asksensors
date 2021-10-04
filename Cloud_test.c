//test for connectivity to Ask Sensors Cloud
// use two sensors, DHT11 and PIR, with interrupt

//Notes: DHT11 sensor to cloud is working fine. PIR is sending data to Serial but can't reach cloud, gets http error

//Sensor libraries
#include <Adafruit_Sensor.h>
#include "DHT.h"

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
String url = "http://api.asksensors.com/write/";
const char *apiKeyIn1 = "hZlLh2GB12ZmbIs2rfE2sgDDbKYTVVbZ";
const char *apiKeyIn2 = "MIXvgXmJyNlRG94TrvJq9lUhNhZhnqq9";

// DHT11 sensor
#define DHTPIN 16 // Digital pin connected to the DHT sensor
#define DHTTYPE DHT11
DHT dht(DHTPIN, DHTTYPE); // Initialize DHT sensor

// PIR sensor
int HW740Pin = 26;
bool motionDetected = digitalRead(HW740Pin);
//define interrupt function for motion sensor, ie what should happen on state change
void IRAM_ATTR isr()
{
    String url = "http://api.asksensors.com/write/";
    Serial.println(url);
    Serial.println(apiKeyIn2);
    if (digitalRead(HW740Pin) == 1)
    {
        Serial.println("movement detected, contacting Ask Sensors....");
    }
    else if (digitalRead(HW740Pin) == 0)
    {
        Serial.println("No movement, informing Ask Sensors....");
    }
    //send new state to API as soon as a change is detected

    url += apiKeyIn2;
    url += "?module2=";
    url += digitalRead(HW740Pin);
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

    return;
}

void setup()
{
    Serial.begin(115200);
    bool status;

    pinMode(HW740Pin, INPUT);                                      //setting input for PIR motion sensor
    attachInterrupt(digitalPinToInterrupt(HW740Pin), isr, CHANGE); //interrupt for PIR motion sensor

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

        //Begin sensor readings

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

        // URL for updating module1 and module 2
        String url = "http://api.asksensors.com/write/";
        url += apiKeyIn1;
        url += "?module1=";
        url += temp; //
        url += "&module2=";
        url += temp;
        url += "&module3=";
        url += humidity; //

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