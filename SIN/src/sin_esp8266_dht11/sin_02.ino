/**
* SIN -  meření teploty a vlhkosti
*
* @brief program senzoru s ESP8266 - Thingboard, WIFI, MQTT...
* @author xkoupy00@stud.fit.vutbr.cz
* @date 2.12.2018
*/
#define FASTLED_ESP8266_RAW_PIN_ORDER
#include <PubSubClient.h>
#include <ESP8266WiFi.h>
#include <FastLED.h>
#include "DHTesp.h"

// wifi definice
#define WIFI_SSID           "WLAN_260"      //ssid
#define WIFI_PASSWORD       "polda2541"     //passwd

//token pro pripojeni Thingsboardu
#define TOKEN "5YmoDeUUC7PIQEGnle2B"

//definice pinu
#define PIN_DHT11 D7
#define LED_PIN   D8

//definice led
#define LED_BRIGHTNESS 20
#define LED_REFRESH    10
#define NUM_LEDS    1
#define LED_TYPE    WS2811
#define COLOR_ORDER GRB

//tb server
const char* THINGSBOARD_SERVER = "10.0.0.1";

//LED
CRGB leds[NUM_LEDS];

CRGBPalette16 currentPalette;
TBlendType    currentBlending;

//definice objektu uzitych knihoven
DHTesp dht;
WiFiClient wifiClient;
PubSubClient client(wifiClient);

int status = WL_IDLE_STATUS;
unsigned long lastSend;

/**
 * Změření a zaslání telemetrie
 */
void getAndSendTemperatureAndHumidityData()
{
  TempAndHumidity newValues = dht.getTempAndHumidity();

  if (dht.getStatus() != 0)
  {
    Serial.println("Error: DHT11: " + String(dht.getStatusString()));
  }

  float heatIndex = dht.computeHeatIndex(newValues.temperature, newValues.humidity);
  float dewPoint = dht.computeDewPoint(newValues.temperature, newValues.humidity);

  //kontrola merenych a vypoctenych odnot
  if ( isnan(newValues.temperature) || isnan(newValues.humidity))
  {
    Serial.println("Error: Check sensors !");
    return;
  }

  String temperature = String(newValues.temperature);
  String humidity = String(newValues.humidity);
  String dewt = String(dewPoint);
  String heatx = String(heatIndex);

  //JSON data string
  String payload = "{";
  payload += "\"temperature\":"; payload += temperature; payload += ",";
  payload += "\"dewpoint\":"; payload += dewt; payload += ",";
  payload += "\"heatindex\":"; payload += heatx; payload += ",";
  payload += "\"humidity\":"; payload += humidity;
  payload += "}";

  // Zaslaní dat
  char attributes[100];
  payload.toCharArray( attributes, 100 );
  client.publish( "v1/devices/me/telemetry", attributes );
  Serial.println( attributes );

}

/**
 * Konfigurace WIFI klienta
 */
void setupWifi()
{
  delay(20);
  Serial.println();
  Serial.print("Connecting to ");
  Serial.println(WIFI_SSID);

  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);

  while (WiFi.status() != WL_CONNECTED)
  {
    delay(100);
    Serial.print(".");
  }

  Serial.println("");
  Serial.println("WiFi is OK ");
  Serial.print(WiFi.localIP());
  Serial.println("");
}

/**
 * Připojení k MQTT- Thingsboard
 */
void reconnect() 
{
  // Loop until we're reconnected
  while (!client.connected()) 
  {
    status = WiFi.status();
    if ( status != WL_CONNECTED) 
    {
      WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
      while (WiFi.status() != WL_CONNECTED) 
      {
        delay(500);
        Serial.print(".");
      }
      Serial.println("Connected to AP");
    }
    if ( client.connect("ESP8266 Device", TOKEN, NULL) )
    {
      Serial.println( "[DONE]" );
    } else {
      // Wait 5 seconds before retrying
      delay( 5000 );
    }
  }
}

/**
 * z příkladu zapisuje barvy na jednotlive diody
 */
void FillLEDsFromPaletteColors( uint8_t colorIndex)
{
  uint8_t brightness = 255;

  for ( int i = 0; i < NUM_LEDS; i++) 
  {
    leds[i] = ColorFromPalette( currentPalette, colorIndex, brightness, currentBlending);
    colorIndex += 3;
  }
}

/**
 * Z příkladu měni paletu barev dle casu
 */
void ChangePalettePeriodically()
{
  uint8_t secondHand = (millis() / 1000) % 60;
  static uint8_t lastSecond = 99;

  if ( lastSecond != secondHand) 
  {
    lastSecond = secondHand;
    if ( secondHand ==  0)  { currentPalette = ForestColors_p;          currentBlending = LINEARBLEND; }
    if ( secondHand == 10)  {currentPalette = ForestColors_p;            currentBlending = LINEARBLEND; ;}
    if ( secondHand == 15)  { currentPalette = ForestColors_p;           currentBlending = LINEARBLEND; }
    if ( secondHand == 20)  { currentPalette = ForestColors_p;           currentBlending = LINEARBLEND; ; }
    if ( secondHand == 25)  {currentPalette = ForestColors_p;              currentBlending = LINEARBLEND; ;}
    if ( secondHand == 30)  {currentPalette = ForestColors_p;              currentBlending = LINEARBLEND;}
    if ( secondHand == 35)  { currentPalette = OceanColors_p;            currentBlending = LINEARBLEND; ;}
    if ( secondHand == 40)  {  currentPalette = OceanColors_p;           currentBlending = LINEARBLEND; }
    if ( secondHand == 45)  { currentPalette = OceanColors_p;            currentBlending = LINEARBLEND;}
    if ( secondHand == 50)  { currentPalette = OceanColors_p;            currentBlending = LINEARBLEND;  }
    if ( secondHand == 55)  { currentPalette = OceanColors_p;  currentBlending = LINEARBLEND; }
  }
}
/**
* Incicalize MQTT clienta , WIFI , DHT11 a LED
*/
void setup()
{
  dht.setup(PIN_DHT11, DHTesp::DHT11);
  delay(1000);
  Serial.begin(115200);
  setupWifi();
  client.setServer(THINGSBOARD_SERVER, 1883);
  //client.setCallback(subscribeCallback);

  delay( 3000 ); // power-up safety delay
  FastLED.addLeds<LED_TYPE, LED_PIN, COLOR_ORDER>(leds, NUM_LEDS).setCorrection( TypicalLEDStrip );
  FastLED.setBrightness(  LED_BRIGHTNESS );

  currentPalette = RainbowColors_p;
  currentBlending = LINEARBLEND;
  lastSend = 0;
}

/**
 * zasilam teplotua vlhkost plus menim barvu diody
 */
void loop()
{
  if ( !client.connected() ) 
  {
    reconnect();
  }

  if ( millis() - lastSend > 1500 ) 
  {
    getAndSendTemperatureAndHumidityData();
    lastSend = millis();
  }

  client.loop();

  ChangePalettePeriodically();

  static uint8_t startIndex = 0;
  startIndex = startIndex + 1; /* motion speed */

  FillLEDsFromPaletteColors( startIndex);

  FastLED.show();
  FastLED.delay(1000 / LED_REFRESH);
}
