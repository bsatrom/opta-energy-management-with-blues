#include <Notecard.h>
#include "stm32h7xx_ll_gpio.h"
#include "thingProperties.h"
#include <ArduinoModbus.h>
#include <ArduinoRS485.h>
#include <Scheduler.h>
#include "OptaBlue.h"

#define F7M24 0x21
#define pause_trigger 15

#define usbSerial Serial

#define DEBUG

// Energy Meter Parameters
float V_actual, V_avg, V_max, V_min;
float A_actual, A_avg, A_max, A_min;
float W_actual, W_avg, W_max, W_min;
float Var_actual;
float Va_actual, Va_avg, Va_max, Va_min;
float Wh_packet, Varh_packet, Wh_Abs_packet;

constexpr auto baudrate { 19200 };

// Calculate preDelay and postDelay in microseconds as per Modbus RTU Specification
// MODBUS over serial line specification and implementation guide V1.02
// Paragraph 2.5.1.1 MODBUS Message RTU Framing
// https://modbus.org/docs/Modbus_over_serial_line_V1_02.pdf
constexpr auto bitduration { 1.f / baudrate };
// constexpr auto preDelayBR { bitduration * 9.6f * 3.5f * 1e6 };
constexpr auto postDelayBR { bitduration * 9.6f * 3.5f * 1e6 };
constexpr auto preDelayBR { bitduration * 10.0f * 3.5f * 1e6 };

int start_time = 0;
int rs485_counter = 0;
int counter = 0;

// Current and previous state of the button.
int buttonState     = 0;
int lastButtonState = 0;

// Variables to implement button debouncing.
unsigned long lastDebounceTime  = 0;
unsigned long debounceDelay     = 50; // In ms

// Arrays of user LEDs
int userLeds[] = {LED_D0, LED_D1, LED_D2, LED_D3};

int numRelays = 4;

long previousMillis = 0;  
long previousUpdateMillis = 0;
long updateInterval = 1000 * 60 * 1;

/*************************************
* Energy Meter related routines
*************************************/


/**
  Displays electrical, power, and energy consumption information retrieved from the energy meter.

  @param V_XXX Voltage information from Energy meter categorized in actual, average, maximum, and minimum value.
  @param A_XXX Current information from Energy meter categorized in actual, average, maximum, and minimum value.
  @param W_XXX Power information from Energy meter categorized in actual, average, maximum, and minimum value.
  @param Var_XXX Reactive power total information from Energy meter categorized in actual.
  @param Va_XXX Apparent power total information from Energy meter categorized in actual, average, maximum, and minimum value.
  @param Wh_packet Power based energy information from Energy meter.
  @param Varh_packet Reactive power total based energy information from Energy meter.
  @param Wh_Abs_packet Absolute Active total based energy information from Energy meter.
*/
struct DataGroup {
  const char* header;
  float* data;
  const char** units;
  uint8_t size;
};

const char* energyUnits[] = { "Wh", "Varh", "Wh" };
const char* actUnits[] = { "V", "A", "W", "Var", "VA" };
const char* avgUnits[] = { "V", "A", "W", "VA" };
const char* maxUnits[] = { "V", "A", "W", "VA" };
const char* minUnits[] = { "V", "A", "W", "VA" };
const char* cloudParamUnits[] = { "u", "W", "Wh" };

void modbus_com_monitor(){
  float energyData[] = { Wh_packet, Varh_packet, Wh_Abs_packet };
  float actData[] = { V_actual, A_actual, W_actual, Var_actual, Va_actual };
  float avgData[] = { V_avg, A_avg, W_avg, Va_avg };
  float maxData[] = { V_max, A_max, W_max, Va_max };
  
  DataGroup groups[] = {
    { "||| Energy  |||||||||||||||||||||", energyData, energyUnits, sizeof(energyData) / sizeof(energyData[0]) },
    { "||| Act. Data  ||||||||||||||||||", actData, actUnits, sizeof(actData) / sizeof(actData[0]) },
    { "||| AVG Data  |||||||||||||||||||", avgData, avgUnits, sizeof(avgData) / sizeof(avgData[0]) },
    { "||| Max. Data  ||||||||||||||||||", maxData, maxUnits, sizeof(maxData) / sizeof(maxData[0]) },
  };

#ifdef DEBUG
  usbSerial.println(); // Empty println for formating reasons
  for(uint8_t i = 0; i < sizeof(groups) / sizeof(groups[0]); i++) {
    usbSerial.println(F(groups[i].header));
    for(uint8_t j = 0; j < groups[i].size; j++) {
      usbSerial.print(groups[i].data[j]);
      usbSerial.print(" ");
      usbSerial.println(groups[i].units[j]);
    }
  }
  usbSerial.println(); // Empty println for formating reasons
#endif
}

/**
  Requests and retrieves actual electrical, and power information from Energy meter over Modbus RTU protocol.
*/
void modbus_com_actual(){
  // Actual Measurements
  // Voltage (V)
  V_actual = getT5(F7M24, 30107);

  if (V_actual != V_actual) {
    V_actual = V_actual;
    V_actual = 0.0f;
  }

  // Current (A)
  A_actual = getT5(F7M24, 30126);
  if (A_actual*100 != A_actual*100) {
    A_actual = A_actual;
    A_actual = 0.0f;
  }

  // Active Power Total - Pt (W)
  W_actual = getT6(F7M24, 30140);
  if (W_actual*100 != W_actual*100) {
    W_actual = W_actual;
    W_actual = 0.0f;
  }
  
  // Reactive Power Total - Qt (var) (IEEE 754)
  Var_actual = getT_Float(F7M24, 32544);
  if (Var_actual*100 != Var_actual*100) {
    Var_actual = Var_actual;
    Var_actual = 0.0f;
  }

  // Apparent Power Total - St (VA)
  Va_actual = getT5(F7M24, 30156);
  if (Va_actual*100 != Va_actual*100) {
    Va_actual = Va_actual;
    Va_actual = 0.0f;
  }
}

/**
  Requests and retrieves average electrical, and power information from Energy meter over Modbus RTU protocol.
*/
void modbus_com_avg(){
  // Average Measurements
  // Voltage (V)
  V_avg = getT5(F7M24, 35507);
  if (V_avg != V_avg) {
    V_avg = V_avg;
    V_avg = 0.0f;
  }

  // Current (A)
  A_avg = getT5(F7M24, 35526);
  if (A_avg != A_avg) {
    A_avg = A_avg;
    A_avg = 0.0f;
  }

  // Active Power Total - Pt (W)
  W_avg = getT6(F7M24, 35540);
  if (W_avg != W_avg) {
    W_avg = W_avg;
    W_avg = 0.0f;
  }

  // Apparent Power Total - St (VA)
  Va_avg = getT5(F7M24, 35556);
  if (Va_avg != Va_avg) {
    Va_avg = Va_avg;
    Va_avg = 0.0f;
  }
}

/**
  Requests and retrieves maximum electrical,and power information from Energy meter over Modbus RTU protocol.
*/
void modbus_com_max(){
  // Maximum Measurements
  // Voltage (V)
  V_max = getT5(F7M24, 35607);
  if (V_max != V_max) {
    V_max = V_max;
    V_max = 0.0f;
  }

  // Current (A)
  A_max = getT5(F7M24, 35626);
  if (A_max != A_max) {
    A_max = A_max;
    A_max = 0.0f;
  }

  // Active Power Total - Pt (W)
  W_max = getT6(F7M24, 35640);
  if (W_max != W_max) {
    W_max = W_max;
    W_max = 0.0f;
  }

  // Apparent Power Total - St (VA)
  Va_max = getT5(F7M24, 35656);
  if (Va_max != Va_max) {
    Va_max = Va_max;
    Va_max = 0.0f;
  }
}

/**
  Requests and retrieves minimum electrical, and power information from Energy meter over Modbus RTU protocol.
*/
void modbus_com_min(){
  // Minimum Measurements
  // Voltage (V)
  V_min = getT5(F7M24, 35707);

  // Current (A)
  A_min = getT5(F7M24, 35726);

  // Active Power Total - Pt (W)
  W_min = getT6(F7M24, 35740);

  // Apparent Power Total - St (VA)
  Va_min = getT5(F7M24, 35756);
}

/**
  Requests and retrieves energy information from Energy meter over Modbus RTU protocol.
*/
void modbus_com_energy(){
  // Energy
  // Energy (Wh) - n1
  Wh_packet = getT_Float(F7M24, 32752);
  if (Wh_packet != Wh_packet) {
    Wh_packet = Wh_packet;
    Wh_packet = 0.0f;
  }

  // Energy (varh) - n4
  Varh_packet = getT_Float(F7M24, 32758);
  if (Varh_packet != Varh_packet) {
    Varh_packet = Varh_packet;
    Varh_packet = 0.0f;
  }

  // Total Absolute Active Energy (Wh)
  Wh_Abs_packet = getT_Float(F7M24, 32760);
  if (Wh_Abs_packet != Wh_Abs_packet) {
    Wh_Abs_packet = Wh_Abs_packet;
    Wh_Abs_packet = 0.0f;
  }
}

/*************************************
* LED PLC Switches
*************************************/

/**
  Defaulting LED states to low when called. 
*/
void plc_led_off() {
  digitalWrite(LEDG, LOW);
  digitalWrite(LEDR, LOW);
  digitalWrite(LEDB, LOW);
  digitalWrite(PIN_SPI_MISO, LOW);
  digitalWrite(PIN_SPI_MOSI, LOW);
  digitalWrite(PIN_SPI_SCK, LOW);
  digitalWrite(PIN_SPI_SS, LOW);
}

/**
  Sets up the LEDs as outputs and calls the plc_led_off() function.
*/
void plc_led_Setup(){
  pinMode(LEDG, OUTPUT);
  pinMode(LEDR, OUTPUT);
  pinMode(LEDB, OUTPUT);
  pinMode(PIN_SPI_MISO, OUTPUT);
  pinMode(PIN_SPI_MOSI, OUTPUT);
  pinMode(PIN_SPI_SCK, OUTPUT);
  pinMode(PIN_SPI_SS, OUTPUT);

  plc_led_off();
}

/*************************************
* Digital Port related tasks
*************************************/

void digitalIO_Setup(){
  pinMode(D0, OUTPUT);
  pinMode(D1, OUTPUT);
  pinMode(D2, OUTPUT);
  pinMode(D3, OUTPUT);
}

/*************************************
* Analog Port related tasks
*************************************/

/**
  Sets up analog ports with 12 bit resolution.
*/
void analogIO_Setup(){
  analogReadResolution(12);

  start_time = millis();
  SCB_DisableDCache();
  usbSerial.println("Start");
}

/*************************************
* RTU related tasks
*************************************/

/**
  Sets up Modbus RTU protocol configuration.
*/
void RTU_Setup(){
  usbSerial.println("Energy Management - Modbus RTU Client");

  RS485.setDelays(preDelayBR, postDelayBR);

  // start the Modbus RTU client
  // 7M.24 Energy meter specifies 19200 of default baudrate and 8N2 frame
  if (!ModbusRTUClient.begin(baudrate, SERIAL_8N2)) {
    usbSerial.println("Failed to start Modbus RTU Client!");
    while (1)
        ;
  }
}

/**
  Obtains T5 data type variable

  @param dev_address Device address.
  @param base_reg Register address.
*/
float getT5(int dev_address, int base_reg) {
  ModbusRTUClient.requestFrom(dev_address, INPUT_REGISTERS, base_reg - 30000, 2);

  while(!ModbusRTUClient.available()) {}

  uint32_t rawreg = ModbusRTUClient.read() << 16 | ModbusRTUClient.read();
  int8_t reg_exp = ((uint8_t*)&rawreg)[3];
  uint32_t reg_base = rawreg & 0x00FFFFFF;
  float reg = reg_base * pow(10, reg_exp);
  
  return reg;
}

/**
  Obtains T6 data type variable

  @param dev_address Device address.
  @param base_reg Register address.
*/
float getT6(int dev_address, int base_reg) {
  ModbusRTUClient.requestFrom(dev_address, INPUT_REGISTERS, base_reg - 30000, 2);
  
  while(!ModbusRTUClient.available()) {}
  
  uint32_t rawreg = ModbusRTUClient.read() << 16 | ModbusRTUClient.read();

  int8_t reg_exp = ((uint8_t*)&rawreg)[3];
  int32_t reg_base = (int32_t)rawreg & 0x007FFFFF;
  if(rawreg & 0x800000) {
    reg_base = -reg_base;
  }
  float reg = reg_base * pow(10, reg_exp);

  return reg;
}

/**
  Obtains T2 data type variable

  @param dev_address Device address.
  @param base_reg Register address.
*/
float getT2(int dev_address, int base_reg) {
  ModbusRTUClient.requestFrom(dev_address, INPUT_REGISTERS, base_reg - 30000, 1);
  while(!ModbusRTUClient.available()) {}

  int16_t rawreg = ModbusRTUClient.read();
  
  return (float)rawreg;
}

/**
  Obtains T_Float data type variable

  @param dev_address Device address.
  @param base_reg Register address.
*/
float getT_Float(int dev_address, int base_reg) {
  ModbusRTUClient.requestFrom(dev_address, INPUT_REGISTERS, base_reg - 30000, 2);
  
  while(!ModbusRTUClient.available()) {}
  
  uint32_t rawreg = ModbusRTUClient.read() << 16 | ModbusRTUClient.read();
  float reg;
  memcpy(&reg, &rawreg, sizeof(float));
  
  return reg;
}

/*****************************
* Arduino Cloud 
*****************************/

/**
  Sets up configuration for Arduino Cloud
*/
void iot_cloud_setup(){
  // Defined in thingProperties.h
  initProperties();

  // Connect to Arduino IoT Cloud
  ArduinoCloud.begin(ArduinoIoTPreferredConnection);
  ArduinoCloud.setNotecardPollingInterval(3000);  // default: 1000ms, min: 250ms
  
  setDebugMessageLevel(DBG_VERBOSE);
  ArduinoCloud.printDebugInfo();
}

void onRelayChange() {
  if (relay_closed) {
    digitalWrite(D3, HIGH); 
    digitalWrite(LED_D3, HIGH);
  } else {
    digitalWrite(D3, LOW); 
    digitalWrite(LED_D3, LOW);
  }
}

/*****************************
* Main
*****************************/
void setup() {

#ifdef DEBUG
  usbSerial.begin(115200);
  const size_t usb_timeout_ms = 3000;
  for (const size_t start_ms = millis(); !usbSerial && (millis() - start_ms) < usb_timeout_ms;)
      ;
#endif

  // Analog/Digital IO Port Configuration
  analogIO_Setup();
  digitalIO_Setup();

  // Modbus RTU Configuration 
  RTU_Setup();
  
  // Status LED configuration;
  plc_led_Setup();

  // IoT Cloud Setup
  iot_cloud_setup();

  pinMode(BTN_USER, INPUT);
  
  for (int i = 0; i < numRelays; i++) {
    digitalWrite(userLeds[i], HIGH); 
    delay(150); 
    digitalWrite(userLeds[i], LOW); 
    delay(150); 
  }
  
  for (int i = numRelays - 1; i >= 0; i--) {
    digitalWrite(userLeds[i], HIGH); 
    delay(150); 
    digitalWrite(userLeds[i], LOW); 
    delay(150); 
  }

  digitalWrite(LED_RESET, HIGH);

  // Make Sure Relay is Open
  digitalWrite(D3, LOW);

  OptaController.begin();
}

void loop() {
  int reading = digitalRead(BTN_USER);
  unsigned long currentMillis = millis();

  static unsigned long nextPollMs = 0;

  OptaController.update();
  ArduinoCloud.update();
  
  // Check if button state has changed.
  if (reading != lastButtonState) {
    lastDebounceTime = millis();
  }

  if ((millis() - lastDebounceTime) > debounceDelay) {
    if (reading != buttonState) {
      buttonState = reading;

      if (buttonState == HIGH) {
        digitalWrite(LED_USER, LOW);
      } else {
        digitalWrite(LED_USER, HIGH);

        toggleRelay();
      }
    }
  }

  if(currentMillis - previousUpdateMillis > updateInterval) {
    previousUpdateMillis = currentMillis;

    modbus_line();
    modbus_com_monitor();
  }

  // Save the current state as the last state, for next time through the loop.
  lastButtonState = reading;
}

void modbus_line() {
  modbus_com_actual();
  modbus_com_avg();
  modbus_com_max();
  modbus_com_min();
  modbus_com_energy();

  updateCloudVariables();
}

void toggleRelay() {
  bool relayChanged = false;

  if (!relay_closed) {
    digitalWrite(D3, HIGH); 
    digitalWrite(LED_D3, HIGH);

    relay_closed = true;
    relayChanged = true;
  } else {
    digitalWrite(D3, LOW); 
    digitalWrite(LED_D3, LOW);
    
    relay_closed = false;
    relayChanged = true;
  }
}

void updateCloudVariables() {
  actual_voltage = V_actual;
  actual_amps = A_actual;
  actual_watts = W_actual;
  actual_Va = Va_actual;
}
