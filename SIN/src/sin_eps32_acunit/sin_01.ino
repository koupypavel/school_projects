/**
* SIN -  klimatizační jednotka
*
* @brief program senzoru s ESP32 - Rele, WIFI, MQTT...
* @author xkoupy00@stud.fit.vutbr.cz
* @date 2.12.2018
*/
#include <DHTesp.h>       
#include <WiFi.h>        
#include <ThingsBoard.h>  

//velikost pole
#define COUNT_OF(x) ((sizeof(x)/sizeof(0[x])) / ((size_t)(!(sizeof(x) % sizeof(0[x])))))

// definice WIFI
#define WIFI_AP_NAME "WLAN_260"// WiFi password
#define WIFI_PASSWORD "polda2541"
//MQTT
#define TOKEN               "o2UBcReMMDWBP5WS82eh"
#define THINGSBOARD_SERVER  "10.0.0.1"

// Baud rate for debug serial
#define SERIAL_DEBUG_BAUD    115200

// Initialize ThingsBoard client
WiFiClient espClient;
ThingsBoard tb(espClient);

int status = WL_IDLE_STATUS;
uint8_t relay_control[] = { 25, 26, 33, 32 };
bool subscribed = false;

/**Volaní pro RPC setGpioStatus
 * https://arduinojson.org/v5/api/jsonvariant/subscript/ for more details
 */
RPC_Response processSetGpioState(const RPC_Data &data)
{

  int pin = data["pin"];
  bool enabled = data["enabled"];

  if (pin < COUNT_OF(relay_control)) 
  {

    digitalWrite(relay_control[pin], enabled);
  }

  return RPC_Response(data["pin"], (bool)data["enabled"]);
}

// RPC handlers
RPC_Callback callbacks[] = {
  { "setGpioStatus",    processSetGpioState },
};

/**
 * Inicializace Wifi, Serialu a Led 
 */
void setup() 
{
  // Initialize serial for debugging
  Serial.begin(115200);
  WiFi.begin(WIFI_AP_NAME, WIFI_PASSWORD);
  InitWiFi();

  for (size_t i = 0; i < COUNT_OF(relay_control); ++i) 
  {
    pinMode(relay_control[i], OUTPUT);
  }
}

/**
 * Hlavní aplikace , registrace callbacku a přepínání barev diody.
 */
void loop() {
  // Reconnect WIFI
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

  // Subscribe RPC
  if (!subscribed) {
    // callbacks -> callbacks[] array.
    if (!tb.RPC_Subscribe(callbacks, COUNT_OF(callbacks))) 
    {
      Serial.println("Failed to subscribe for RPC");
      return;
    }

    Serial.println("Subscribe done");
    subscribed = true;
  }

  tb.loop();
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
