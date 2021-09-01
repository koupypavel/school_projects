/**
* SIN -  LED matrice
*
* @brief program senzoru s ESP8266 - WS8211, WIFI, MQTT...
* @author xkoupy00@stud.fit.vutbr.cz
* @date 2.12.2018
*/
#define FASTLED_ESP8266_RAW_PIN_ORDER
#include <ESP8266WiFi.h>
#include <FastLED.h>
#include <ThingsBoard.h>  

//velikost pole
#define COUNT_OF(x) ((sizeof(x)/sizeof(0[x])) / ((size_t)(!(sizeof(x) % sizeof(0[x])))))

// definice WIFI 
#define WIFI_AP_NAME        "WLAN_260"
#define WIFI_PASSWORD       "polda2541"
//MQTT
#define TOKEN               "d7I0OWTh0vz5CUuPr3mX"
#define THINGSBOARD_SERVER  "10.0.0.1"

//definice objektu knihoven
WiFiClient espClient;
ThingsBoard tb(espClient);

int status = WL_IDLE_STATUS;
int gprogram = 1;

//LED 
#define LED_PIN   D8

#define LED_BRIGHTNESS 20
#define LED_REFRESH    60
#define NUM_LEDS       25
#define LED_TYPE    WS2811
#define COLOR_ORDER GRB

CRGB leds[NUM_LEDS];
CRGBPalette16 currentPalette;
TBlendType    currentBlending;

bool subscribed = false;
/** Volání pro RPC setValue
 * See https://arduinojson.org/v5/api/jsonvariant/subscript/ for more details
 */
RPC_Response processProgramChange(const RPC_Data &data)
{
  ChangePalette(data);
  gprogram = data;
  Serial.print("Set program: ");
  Serial.println(gprogram);

  return RPC_Response(NULL, gprogram);
}

/**Volaní pro RPC getValue
 * https://arduinojson.org/v5/api/jsonvariant/subscript/ for more details
 */
RPC_Response processGetProgram(const RPC_Data &data)
{
  Serial.println("Received the get value method");

  return RPC_Response(NULL, gprogram);
}

// RPC handlers
RPC_Callback callbacks[] = {
  { "setValue",         processProgramChange },
  { "getValue",         processGetProgram },
};


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
 * Mením paletu dle zvoleneho programu
 */
void ChangePalette(int index)
{

  if ( gprogram != index) 
  {
    gprogram = index;
    if ( index == 0)  { currentPalette = ForestColors_p;          currentBlending = LINEARBLEND; }
    if ( index == 1)  { currentPalette = OceanColors_p;            currentBlending = LINEARBLEND; ;}
    if ( index == 2)  { currentPalette = LavaColors_p;           currentBlending = LINEARBLEND; }
    if ( index == 3)  { currentPalette = RainbowColors_p;           currentBlending = LINEARBLEND; ; }
  }
}
/**
 * Initi WIFI
 */
void InitWiFi()
{
  Serial.println("Connecting to AP ...");

  WiFi.begin(WIFI_AP_NAME, WIFI_PASSWORD);
  while (WiFi.status() != WL_CONNECTED) 
  {
    delay(500);
    Serial.print(".");
  }
  Serial.println("Connected to AP");
}
/**
 * reconnect
 */
void reconnect() 
{
  status = WiFi.status();
  if ( status != WL_CONNECTED) 
  {
    WiFi.begin(WIFI_AP_NAME, WIFI_PASSWORD);
    while (WiFi.status() != WL_CONNECTED) 
    {
      delay(500);
      Serial.print(".");
    }
    Serial.println("Connected to AP");
  }
}

/**
 * Inicializace Wifi, Serialu a Led 
 */
void setup() 
{
  // Initialize serial for debugging
  Serial.begin(115200);
  WiFi.begin(WIFI_AP_NAME, WIFI_PASSWORD);
  InitWiFi();
  delay( 3000 ); // power-up safety delay
  FastLED.addLeds<LED_TYPE, LED_PIN, COLOR_ORDER>(leds, NUM_LEDS).setCorrection( TypicalLEDStrip );
  FastLED.setBrightness(  LED_BRIGHTNESS );
  ChangePalette(gprogram);

}

/**
 * Hlavní aplikace , registrace callbacku a přepínání barev diody.
 */
void loop() {
  // Reconnect to WiFi, if needed
  if (WiFi.status() != WL_CONNECTED) 
  {
    reconnect();
    return;
  }

  if (!tb.connected()) 
  {
    subscribed = false;

    if (!tb.connect(THINGSBOARD_SERVER, TOKEN)) 
    {
      Serial.println("Failed to connect");
      return;
    }
  }

  if (!subscribed) 
  {
    if (!tb.RPC_Subscribe(callbacks, COUNT_OF(callbacks))) 
    {
      Serial.println("Failed to subscribe for RPC");
      return;
    }
    subscribed = true;
  }

  tb.loop();
  static uint8_t startIndex = 0;
  startIndex = startIndex + 1; /* motion speed */

  FillLEDsFromPaletteColors( startIndex);

  FastLED.show();
  FastLED.delay(1000 / LED_REFRESH);

}
