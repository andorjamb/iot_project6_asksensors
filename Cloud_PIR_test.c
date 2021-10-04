//test for connectivity of PIR to Ask Sensors Cloud, without Interrupt
//Notes: it is working!

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
const unsigned int writeInterval = 40 * 1000; // write interval (in ms):40s

// ASKSENSORS API config
const char *host = "api.asksensors.com"; // API host name
const int httpPort = 80;                 // port
String url = "http://api.asksensors.com/write/";
const char *apiKeyIn2 = "MIXvgXmJyNlRG94TrvJq9lUhNhZhnqq9";

// PIR sensor
int HW740Pin = 26;
bool motionDetected = digitalRead(HW740Pin);

void setup()
{
    Serial.begin(115200);
    bool status;

    pinMode(HW740Pin, INPUT); //setting input for PIR motion sensor
    //attachInterrupt(digitalPinToInterrupt(HW740Pin), isr, CHANGE); //interrupt for PIR motion sensor

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
    { //Begin sensor readings
        bool motionDetected = digitalRead(HW740Pin);
        String url = "http://api.asksensors.com/write/";
        Serial.println(url);
        Serial.println(apiKeyIn2);
        if (motionDetected == 1)
        {
            Serial.println("movement detected, contacting Ask Sensors....");
        }
        else if (motionDetected == 0)
        {
            Serial.println("No movement, informing Ask Sensors....");
        }
        //send new state to API

        url += apiKeyIn2;
        url += "?module2=";
        url += motionDetected;
        url += "&module3=";
        url += motionDetected;
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