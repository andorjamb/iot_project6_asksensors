

// Basic interrupt test for PIR sensor
//based on Ruis Santos code

//AskSensors connection libraries
#include <WiFi.h>
#include <WiFiMulti.h>
#include <HTTPClient.h>

#define timeSeconds 10

//AskSensors config and network credentials
WiFiMulti WiFiMulti;
HTTPClient ask;

//const char *ssid = "Vallikurvit";
//const char *password = "Kotkanpesa";
const char *ssid = "DNA-WLAN-2G-A178";
const char *password = "64957969891";
const char *apiKeyIn2 = "MIXvgXmJyNlRG94TrvJq9lUhNhZhnqq9";

// Set GPIOs for  PIR Motion Sensor

const int motionSensor = 26;

// Timer: Auxiliary variables
unsigned long now = millis();
unsigned long lastTrigger = 0;
boolean startTimer = false;

String url = "http://api.asksensors.com/write/";
bool motionDetected = digitalRead(HW740Pin);
// Checks if motion was detected, sets LED HIGH and starts a timer

void IRAM_ATTR detectsMovement()
{
    Serial.println("MOTION DETECTED!!!");

    startTimer = true;
    lastTrigger = millis();

    // send to API - this doesn't work, kernel still panics
    /*
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
    ask.end();*/
    return;
}

void setup()
{
    // Serial port for debugging purposes
    Serial.begin(115200);

    // PIR Motion Sensor mode INPUT_PULLUP
    pinMode(motionSensor, INPUT_PULLUP);
    // Set motionSensor pin as interrupt, assign interrupt function and set RISING mode
    attachInterrupt(digitalPinToInterrupt(motionSensor), detectsMovement, RISING);

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
    // Current time
    now = millis();
    // Turn off the LED after the number of seconds defined in the timeSeconds variable
    if (startTimer && (now - lastTrigger > (timeSeconds * 1000)))
    {
        Serial.println("Motion stopped...");
        //digitalWrite(led, LOW);
        startTimer = false;
    }
}