/**
* SEN -  ultrazvukovy sensor/meric stavu vody v nadrzi
*
* @brief program senzoru s ESP32 - DEEP SLEEP, WIFI, MQTT...
* @author xkoupy00@stud.fit.vutbr.cz
* @date 6.11.2018
*/
#include <WiFi.h>
#include <PubSubClient.h>
#include "DHTesp.h"
#include "FS.h"
#include "SPIFFS.h"

//pokud neni fs naformátovaný -- pri prvním pouziti
#define FORMAT_SPIFFS_IF_FAILED true

void calibration();
void setupWifi();
void writeFile(fs::FS &fs, const char * path, const char * message);
String readFile(fs::FS &fs, const char * path);
void listDir(fs::FS &fs, const char * dirname, uint8_t levels);
void reconnectMQTT();
float measureDistance();
float calculateVolume(float height_act, float width_abs);
void subscribeCallback(char* topic, byte* payload, unsigned int length);

/* definice Wifi*/
//zatim napevno
#define WIFI_SSID           "pi"      //ssid
#define WIFI_PASSWORD       "pi"     //passwd

/* deifinice MQTT*/
//zatim napevno
#define MQTT_SERVER         "localhost"      // ip MQTT brokera
#define MQTT_USER           "pi"            // login
#define MQTT_PASSWORD       "pi"    // passwd

/* topics pro pub a sub operace */
#define TOPIC_TEMPERATURE   "uwls/raw/temp"       // Topic teplota xx.x Celsia
#define TOPIC_HUMIDITY      "uwls/raw/humid"      // Topic vlhkost xx %
#define TOPIC_DISTANCE      "uwls/raw/dist"       // Topic vzdálenost od vodni hladiny
#define TOPIC_DEWPOINT      "uwls/calc/dewPoint"  // Topic
#define TOPIC_HEATPOINT     "uwls/calc/heatIndex" // Topic
#define TOPIC_VOLUME        "uwls/calc/volume"    // Topic
#define TOPIC_TANKLEVEL     "uwls/calc/tankLevel"
#define TOPIC_DEBUG         "uwls/debug"          // Topic debugovaci vypisy

#define TOPIC_RELAY1        "uwls/cmd/relay1"
#define TOPIC_RELAY2        "uwls/cmd/relay2"
#define TOPIC_HEIGHT        "uwls/cmd/height"
#define TOPIC_WIDTH         "uwls/cmd/width"
#define TOPIC_CALIBRATION   "uwls/cmd/calibration"

/* definice cest k souborum*/
#define FILE_CHEIGHT"/calibration_height.txt"
#define FILE_CWIDTH"/calibration_width.txt"
/* definice pro deepsleep */
#define DP_uS_TO_S_FACTOR 1000000             // Prevod microsekunda -> sekunda
#define DP_TIME_TO_SLEEP 60                   // Delka spanku 1 minuta
#define DP_TIME_TO_SLEEP_ERROR 300            // pri poruše zasilam data jeednou za 5 minut

/* definice pinu IO modulů */
#define PIN_DHT    32
#define PIN_ECHO   33
#define PIN_TRIG   25
#define PIN_RIN1   26
#define PIN_RIN2   27
#define PIN_BUZZ   26

#define PI_CONST 3.141592

DHTesp dht;
WiFiClient espClient;
PubSubClient client(espClient);

/**
* Callback pro subscribe MQTT clienta
* kvuli deep sleep je nutné používat retained zprávy.
*/
void subscribeCallback(char* topic, byte* payload, unsigned int length)
{
    String topic_in = topic;

    if (topic_in == TOPIC_RELAY1)
    {
        if ((char)payload[0] == '1')
        {
            digitalWrite(PIN_RIN1, HIGH);
            delay(500);
        }
        else
        {
            digitalWrite(PIN_RIN1, LOW);
            delay(500);
        }
    }
    else if (topic_in == TOPIC_RELAY2)
    {
        if ((char)payload[0] == '1')
        {
            digitalWrite(PIN_RIN2, HIGH);
            delay(500);
        }
        else
        {
            digitalWrite(PIN_RIN2, LOW);
            delay(500);
        }
    }
    else if (topic_in == TOPIC_HEIGHT)
    {
        if ((char)payload[0] == '1')
        {
            String payld = (char *) &payload[1];
            writeFile(SPIFFS, FILE_CHEIGHT, (String(payld)).c_str());
        }
    }
    else if (topic_in == TOPIC_WIDTH)
    {
        if ((char)payload[0] == '1')
        {
            String payld = (char *) &payload[1];
            writeFile(SPIFFS, FILE_CWIDTH, (String(payld)).c_str());
        }
    }
    else if (topic_in == TOPIC_CALIBRATION)
    {
        if ((char)payload[0] == '1')
        {
            calibration();
        }
    }
}

/**
* Zakladní konfigurace a vyzitani senzoru -- loop se vola jen kvuli mqtt subscribe -- loop() v setupu
* funkce nereagovala na zpravy.
*/
void setup()
{
    //// konfigurace IO
    //ultrazvukovy senzor
    pinMode(PIN_TRIG, OUTPUT);
    pinMode(PIN_ECHO, INPUT);
    //rele
    pinMode(PIN_RIN1, OUTPUT);
    pinMode(PIN_RIN2, OUTPUT);
    pinMode(PIN_BUZZ, OUTPUT);

    Serial.begin(115200);

    if (!SPIFFS.begin(FORMAT_SPIFFS_IF_FAILED)) {
        Serial.println("setup() : FAIL - SPIFFS Mount Failed");
        return;
    }

    setupWifi();

    //konfigurace a pripojeni mqtt klienta
    client.setServer(MQTT_SERVER, 1883);
    client.setCallback(subscribeCallback);
    if (!client.connected())
    {
        reconnectMQTT();
    }

    client.subscribe(TOPIC_RELAY1, 1);
    client.subscribe(TOPIC_RELAY2, 1);
    client.subscribe(TOPIC_WIDTH, 1);
    client.subscribe(TOPIC_HEIGHT, 1);
    client.subscribe(TOPIC_CALIBRATION, 1);
    //konfigurace a cteni z DHT senzoru
    dht.setup(PIN_DHT, DHTesp::DHT11);

    TempAndHumidity newValues = dht.getTempAndHumidity();

    if (dht.getStatus() != 0)
    {
        Serial.println("setup() : ERROR - DHT11: " + String(dht.getStatusString()));
    }
    //pocitova teplota
    float heatIndex = dht.computeHeatIndex(newValues.temperature, newValues.humidity);
    //mlzny bod
    float dewPoint = dht.computeDewPoint(newValues.temperature, newValues.humidity);
    //vzdalenost od hladiny
    float distance = measureDistance();
    //absolutni vyska - vycteni z flash
    float height_abs = (readFile(SPIFFS, FILE_CHEIGHT)).toFloat();
    //prumer
    float width_abs = (readFile(SPIFFS, FILE_CWIDTH)).toFloat();
    //aktualni vyska vody
    float height_act = height_abs - distance;
    //objem vody dm^3 - litr
    float volume = calculateVolume(height_act, width_abs) / 1000; //cm^3 -> dm^3
    volume = volume < 0 ? 0 : volume;
    //vyska hladiny v procentech
    int tankLevel = (height_act / height_abs) * 100;
    tankLevel = tankLevel < 0 ? 0 : tankLevel;
    //kontrola merenych a vypoctenych odnot
    if ( isnan(newValues.temperature) || isnan(newValues.humidity) || isnan(distance) || isnan(volume))
    {
        Serial.println("setup(): ERROR");
        client.publish(TOPIC_DEBUG, "UWLS ERROR", true);
        // DEEPSLEEP -- pri chybe nekterych hodnot nema cenu aktualizovat po minute -- prodlouzim na 5 minut
        esp_sleep_enable_timer_wakeup(DP_TIME_TO_SLEEP_ERROR * DP_uS_TO_S_FACTOR);
        esp_deep_sleep_start();
        return;
    }

    //zaslani teploty
    client.publish(TOPIC_TEMPERATURE, String(newValues.temperature).c_str(), true);
    // vlhkosti
    client.publish(TOPIC_HUMIDITY, String(newValues.humidity).c_str(), true);
    // pocitove teploty
    client.publish(TOPIC_HEATPOINT, String(heatIndex).c_str(), true);
    // rosny bod
    client.publish(TOPIC_DEWPOINT, String(dewPoint).c_str(), true);
    //namerena vzdalenost
    client.publish(TOPIC_DISTANCE, String(distance).c_str(), true);
    // objem
    client.publish(TOPIC_VOLUME, String(volume).c_str(), true);
    //stav
    client.publish(TOPIC_TANKLEVEL, String(tankLevel).c_str(), true);

    Serial.println("setup() : MQTT PUB DONE");
    client.publish(TOPIC_DEBUG, "MQTT PUB DONE", true);

    delay(200);
}

/**
* Konfigurace klienta
*/
void setupWifi()
{
    delay(20);
    Serial.println();
    Serial.print("setupWifi() : CONNECTING - ");
    Serial.println(WIFI_SSID);

    WiFi.begin(WIFI_SSID, WIFI_PASSWORD);

    while (WiFi.status() != WL_CONNECTED)
    {
        delay(100);
        Serial.print(".");
    }

    Serial.println("");
    Serial.println("setupWifi() : OK -");
    Serial.print(WiFi.localIP());
    Serial.println("");
}

/**
* MQTT znovu pripojeni
*/
void reconnectMQTT()
{
    while (!client.connected())
    {
        Serial.print("reconnectMQTT() : Connecting to MQTT broker ...");
        if (client.connect("UWLS", MQTT_USER, MQTT_PASSWORD))
        {
            Serial.println("reconnectMQTT() : OK");
        }
        else
        {
            Serial.print("reconnectMQTT() : FAIL -  Not connected: ");
            Serial.print(client.state());
            Serial.println("reconnectMQTT() : Wait 5 seconds before retry.");
            delay(5000);
        }
    }
}

/**
* Meření vzdálenosti pomocí HC-SR04
* Měří čas mezi vysláním a návratem ultr. impulzu
*/
float measureDistance()
{
    // Zaslaní ult. impulzu --> PING --> PIN_TRIG
    digitalWrite(PIN_TRIG, LOW);
    delayMicroseconds(2);
    digitalWrite(PIN_TRIG, HIGH);
    delayMicroseconds(10);
    digitalWrite(PIN_TRIG, LOW);
    // Vycte .kdy se vratil PING --> PIN_ECHO, vraci cas v microsekundach
    long duration = pulseIn(PIN_ECHO, HIGH);
    // Calculating the distance
    return duration * 0.034 / 2;
}

/**
* Kalibracní metoda - namerene hodnoty jsou zapsany do flash pameti
* Signalizuje stav pomocí bzucaku nebo diody pripojené k
* Superseal konetroru na pinu 4.
* Pin 1 - zem, 2 - 5v, 3 - GPIO27, 4 -GPIO26
*/
void calibration()
{
    //signalizace zacatku kalibrace
    digitalWrite(PIN_BUZZ, HIGH);
    delay(200);
    digitalWrite(PIN_BUZZ, LOW);
    delay(200);
    digitalWrite(PIN_BUZZ, HIGH);
    delay(200);
    digitalWrite(PIN_BUZZ, LOW);

    delay(10000);
    //measure width
    float width =  measureDistance();
    writeFile(SPIFFS, "/calibration_width.txt", (String(width)).c_str());

    digitalWrite(PIN_BUZZ, HIGH);
    delay(1000);
    digitalWrite(PIN_BUZZ, LOW);
    //measure height
    //delay for rotating sensor
    delay(10000);
    //
    float height = measureDistance();
    //save to calibration.txt xx.x;xx.x
    writeFile(SPIFFS, "/calibration_height.txt", (String(height)).c_str());
    //respond containg measured data as ack

    digitalWrite(PIN_BUZZ, HIGH);
    delay(200);
    digitalWrite(PIN_BUZZ, LOW);
    delay(200);
    digitalWrite(PIN_BUZZ, HIGH);
    delay(200);
    digitalWrite(PIN_BUZZ, LOW);
    delay(100);
}
/**
* Výpočet objemu nádrže
*/
float calculateVolume(float height_act, float width_abs)
{
    return  PI_CONST * ((width_abs * width_abs) / 4) * height_act;
}

//////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////FLASH IO
/**
* Výpis souborů na FLASH
*/
void listDir(fs::FS &fs, const char * dirname, uint8_t levels)
{
    Serial.printf("listDir() : %s\r\n", dirname);

    File root = fs.open(dirname);
    if (!root)
    {
        Serial.println("listDir() : OPEN FAIL");
        return;
    }
    if (!root.isDirectory())
    {
        Serial.println("listDir() : NOT DIR");
        return;
    }

    File file = root.openNextFile();
    while (file) {
        if (file.isDirectory())
        {
            Serial.print("   DIR : ");
            Serial.println(file.name());
            if (levels)
            {
                listDir(fs, file.name(), levels - 1);
            }
        }
        else
        {
            Serial.print("  FILE: ");
            Serial.print(file.name());
            Serial.print("\tSIZE: ");
            Serial.println(file.size());
        }
        file = root.openNextFile();
    }
}
/**
* Vycitani souboru z FLASH , vrací string
*/
String readFile(fs::FS &fs, const char * path)
{
    String res = "";

    File file = fs.open(path);
    if (!file || file.isDirectory())
    {
        Serial.println("readFile() : OPEN FAIL OR DIRECTORY");
        return "";
    }

    while (file.available())
    {
        res = res + (char)file.read();
    }
    return res;
}
/**
* Zápis souborů na FLASH
*/
void writeFile(fs::FS &fs, const char * path, const char * message)
{
    Serial.printf("writeFile(): %s\r\n", path);

    File file = fs.open(path, FILE_WRITE);
    if (!file)
    {
        Serial.println("writeFile() : OPEN FAIL");
        return;
    }
    if (file.print(message))
    {
        Serial.println("writeFile() : OK");
    }
    else
    {
        Serial.println("writeFile() : FAIL");
    }
}

/**
* Volá se zde zpracování mqtt zpráv a pote se prejde do rezimu spánku.
*/
void loop()
{
    for (int i = 0; i < 200; i++)
    {
        client.loop();
        delay(50);
    }

    // zruseni sub. topicu
    client.unsubscribe(TOPIC_RELAY1);
    client.unsubscribe(TOPIC_RELAY2);
    client.unsubscribe(TOPIC_WIDTH);
    client.unsubscribe(TOPIC_HEIGHT);
    client.unsubscribe(TOPIC_CALIBRATION);
    delay(100);
    Serial.println("loop() : DEEP SLEEP");
    delay(100);
    //vstup do DEEP SLEEP
    esp_sleep_enable_timer_wakeup(DP_TIME_TO_SLEEP * DP_uS_TO_S_FACTOR); //go to sleep
    esp_deep_sleep_start();
}
