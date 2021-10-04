//AskSensors and API connection libraries
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
const char *url = "http://api.asksensors.com/write/";

// ASKSENSORS API host config
const char *host = "api.asksensors.com"; // API host name
const int httpPort = 80;                 // port

//Openweather maps
const char *appid = "63d516de57524133ca5c7f7557895667"; //insert openweathermaps app id
const char *weatherURL = "https://api.openweathermap.org/data/2.5/weather?q=Kajaani&units=metric&appid=";
const char *kajaani_temp = "";

void setup()
{

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

        //Fetch JSON data from openweathermaps
        weatherURL += appid;

        ask.begin(weatherURL);
        int httpCode = ask.GET();
        if (httpCode > 0)
        {
            String payload = ask.getString(); // returns a string value that is the response to our request
            Serial.println(httpCode);
            Serial.println(payload);
        }
        else
        {
            Serial.println("Error on HTTP request");
        }

        ask.end();
        /*
        char *kajaani_data = httpGETRequest(weatherURL);
        JSONVar myObject = JSON.parse(kajaani_data);
        Serial.println(myObject);
        delay(1000);
        kajaani_temp = parsed["temp"];

        //send weatherdata to Ask Sensors
        //URL for updating sensor 2
        url += apiKeyIn2;
        url += "?module4=";
        url += kajaani_temp; //parsed JSON data goes here

        ask.begin(url);
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
        ask.end();*/
    }
    client.stop();
}