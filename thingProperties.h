#include <ArduinoIoTCloud.h>
#include <Arduino_ConnectionHandler.h>
#include "arduino_secrets.h"

#define NOTECARD_PRODUCT_UID "com.your.product_uid"

void onRelayChange();

bool relay_closed;

float actual_voltage;
float actual_amps;
float actual_watts;
float actual_Va;

NotecardConnectionHandler ArduinoIoTPreferredConnection(NOTECARD_PRODUCT_UID);

void initProperties(){
  ArduinoCloud.addProperty(relay_closed, Permission::ReadWrite).onUpdate(onRelayChange);

  ArduinoCloud.addProperty(actual_voltage, Permission::Read).publishEvery(60);
  ArduinoCloud.addProperty(actual_amps, Permission::Read).publishEvery(60);
  ArduinoCloud.addProperty(actual_watts, Permission::Read).publishEvery(60);
  ArduinoCloud.addProperty(actual_Va, Permission::Read).publishEvery(60);
  
  if (::strncmp(SECRET_WIFI_SSID, "5at70m", sizeof(SECRET_WIFI_SSID))) {
    ArduinoIoTPreferredConnection.setWiFiCredentials(SECRET_WIFI_SSID, SECRET_WIFI_PASS);
  }
}