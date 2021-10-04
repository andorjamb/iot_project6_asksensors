// test with luminosity sensor and PIR sensor using interrupt

//sensor libraries
#include <Wire.h>

//AskSensors connection libraries
#include <WiFi.h>
#include <WiFiMulti.h>
#include <HTTPClient.h>

//AskSensors config and network credentials
WiFiMulti WiFiMulti;
HTTPClient ask;
String url = "http://api.asksensors.com/write/";

//const char *ssid = "Vallikurvit";
//const char *password = "Kotkanpesa";
const char *ssid = "DNA-WLAN-2G-A178";
const char *password = "64957969891";

// ASKSENSORS API config
const char *host = "api.asksensors.com"; // API host name
const int httpPort = 80;                 // port
const char *apiKeyIn1 = "hZlLh2GB12ZmbIs2rfE2sgDDbKYTVVbZ";
const char *apiKeyIn2 = "MIXvgXmJyNlRG94TrvJq9lUhNhZhnqq9";

// PIR sensor global variables
int HW740Pin = 26;
volatile bool FLAG = false;
volatile bool PIRValue = 0;
volatile bool *PIRValuePointer = &PIRValue;
volatile bool oldPIRValue;

//Lum TEMT6000 sensor
const int LumSensorPin = 34; //default pin for TEMT6000 sensor

//variables for timer
unsigned long timeRunning = 0;
unsigned long stopWatch = 0;
unsigned long timeSinceRequest = 0;
const long notifyInterval = 30 * 60 * 1000; //API update interval in miliseconds

//debouncing
volatile unsigned long waitDebounce = 0;
const long debounceTime = 50; //ie 50 milliseconds

//define the interrupt function here
void ISR_function() //this should have an IRAM_ATTR tag but Code objects. When IRAM_ATTR is added, I can't define the String url variable anymore
{
    Serial.println("tripwire - change in PIR sensor detected");
    Serial.println("inside IRS right now");
    Serial.print("former PIR value: ");
    Serial.println(PIRValue);
    oldPIRValue = PIRValue;
    *PIRValuePointer = digitalRead(HW740Pin); //update PIR value
    Serial.print("New PIR value: ");
    Serial.println(PIRValue);
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
        Serial.println(payload); // it is printing '2'?
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
    pinMode(HW740Pin, INPUT);                                               //setting input for motion sensor
    attachInterrupt(digitalPinToInterrupt(HW740Pin), ISR_function, CHANGE); //interrupt for motion sensor

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
    //check wifi is still connected
    WiFiClient client;
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
            FLAG = false; // re-initialise flag
        }
        else
        {
            Serial.println("no change in PIR");
        }

        timeRunning = millis();
        timeSinceRequest = timeRunning - stopWatch; //updating the timer
        if (timeSinceRequest >= 30 * 1000)          //check the timer for sending sensor readings
        {

            //luminosity readings
            int luminosity = analogRead(LumSensorPin);
            Serial.print(luminosity);
            Serial.println(" lux");

            // URL for updating sensor 1
            String url = "http://api.asksensors.com/write/";
            url += apiKeyIn1; //key for lum sensor
            url += "?module2=";
            url += luminosity; //

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
            stopWatch = millis(); //update stopWatch;
        }
        else
        {
            Serial.println("Time not up, Continuing to next iteration");
        }
    }

    client.stop();
    delay(500);
}
