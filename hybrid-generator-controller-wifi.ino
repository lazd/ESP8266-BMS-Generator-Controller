#include "site/html.h"
#include <ESP8266WiFi.h>
#include <NTPClient.h>
#include <WiFiUdp.h>
#include <daly-bms-uart.h>

#define STASSID "davisRouterLegacy"
#define STAPSK "0192837465"
#define UTCOFFSETINSECONDS 8 * -3600

#define BMS_SERIAL Serial // Set the serial port for communication with the Daly BMS

#define GENERATOR_SIGNAL_PIN 5
#define INVERTER_POWER_PIN 14
#define GRID_POWER_PIN 12
#define GENERATOR_POWER_PIN 13

#define MAX_GENERATOR_START_TRIES 2

#define GENERATOR_START_RETRY_TIME 30000 // 0.5 * 60 * 100 // 30 seconds
#define GENERATOR_COOLDOWN_TIME 300000 // 300000 // 5 * 60 * 1000 // 5 minutes
#define SIGNAL_TIME 2000

#define BATTERY_MINIMUM_SOC 15 // 15%
#define BATTERY_MAXIMUM_SOC 100 // 100%

#define LED_ON LOW
#define LED_OFF HIGH

const char* ssid = STASSID;
const char* password = STAPSK;

// Define NTP Client to get time
WiFiUDP ntpUDP;
NTPClient timeClient(ntpUDP, "pool.ntp.org", UTCOFFSETINSECONDS);

// Construct the bms driver and passing in the Serial interface (which pins to use)
Daly_BMS_UART bms(BMS_SERIAL);

// Create an instance of the server
// specify the port to listen on as an argument
WiFiServer server(80);

// State variables
bool bmsCommunicationStatus = false;
bool generatorOn = false;
bool gridPowerOn = false;
bool inverterOn = false;
bool startStopButtonPressed = false;
bool generatorStopRequested = false;
bool generatorStartRequested = false;
int generatorStartTries = 0;
unsigned long lastGeneratorStartTryTime = 0;
unsigned long generatorCooldownStartTime = 0;
unsigned long signalStartTime = 0;

unsigned long gridPowerLostTime = 0;
unsigned long gridPowerRestoredTime = 0;
unsigned long generatorStartTime = 0;
unsigned long generatorStopTime = 0;

char* htmlChars = reinterpret_cast<char*>(site_index_min_html);

void setup() {
  // Pin setup
  pinMode(GENERATOR_SIGNAL_PIN, OUTPUT);
  pinMode(INVERTER_POWER_PIN, INPUT_PULLUP);
  pinMode(GRID_POWER_PIN, INPUT_PULLUP);
  pinMode(GENERATOR_POWER_PIN, INPUT_PULLUP);

  pinMode(LED_BUILTIN, OUTPUT);

  // Serial.print("Connecting to ");
  // Serial.println(ssid);

  WiFi.mode(WIFI_STA);
  WiFi.setSleepMode(WIFI_NONE_SLEEP);
  WiFi.begin(ssid, password);

  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    // Serial.print(".");
  }
  // Serial.println();
  // Serial.println("WiFi connected");

  // Update the time
  timeClient.begin();
  timeClient.update();

  // Start the server
  server.begin();
  // Serial.println("Server started");

  // Print the IP address
  // Serial.println(WiFi.localIP());

  // This call sets up the driver
  bms.Init();
}

bool pressStartStopButton() {
  if (startStopButtonPressed) {
    return false;
  }

  startStopButtonPressed = true;
  digitalWrite(GENERATOR_SIGNAL_PIN, LED_ON); // Press the button  
  signalStartTime = millis();

  return true;
}

bool startGenerator() {
  if (generatorOn) {
    return false;
  }

  // Serial.println("Starting generator");

  // Power generator on
  pressStartStopButton();
  
  return true;
}

bool stopGenerator() {
  if (!generatorOn) {
    return false;
  }

  // Serial.println("Stopping generator");

  // Stop generator
  pressStartStopButton();

  generatorStopRequested = false;

  generatorStartTime = 0;
  generatorStopTime = timeClient.getEpochTime();

  return true;
}

bool requestStopGenerator() {
  if (!generatorOn) {
    return false;
  }

  if (generatorStopRequested) {
    return false;
  }

  // Serial.println("Cooling down generator");

  generatorStopRequested = true;
  generatorCooldownStartTime = millis();

  return true;
}

bool requestStartGenerator() {
  if (!generatorStartRequested) {
    generatorStartRequested = true;
    return true;
  }
  return false;
}

void startStopButtonLoop() {
  if (startStopButtonPressed) {
    if ((millis() - signalStartTime) >= SIGNAL_TIME) {
      digitalWrite(GENERATOR_SIGNAL_PIN, LED_OFF); // Release the button 
      startStopButtonPressed = false; 
    }
  }
}

void generatorCooldownLoop() {
  if (!generatorOn) {
    generatorStopRequested = false;
  }

  if (generatorStopRequested) {
    // We've cooled down enough, stop the generator
    if ((millis() - generatorCooldownStartTime) >= GENERATOR_COOLDOWN_TIME) {
      // Serial.println("Generator cool down complete");
      stopGenerator();
    }
  }
}

void cancelGeneratorStartRequest() {
  generatorStartRequested = false;
  lastGeneratorStartTryTime = 0;
  generatorStartTries = 0;
}

void generatorStartLoop() {
  if (generatorStartRequested && generatorOn) {
    // Generator successfully started
    cancelGeneratorStartRequest();
    // Serial.println("Generator start request successful");
    generatorStartTime = timeClient.getEpochTime();
    generatorStopTime = 0;
    return;
  }

  if (generatorOn) {
    return;
  }

  if (generatorStartRequested) {
    if (generatorStartTries == MAX_GENERATOR_START_TRIES) {
      // Serial.println("Generator failed to start after " + (String)MAX_GENERATOR_START_TRIES + " tries, cannot start generator!");
      return;
    }

    if (generatorStartTries == 0 || (millis() - lastGeneratorStartTryTime) >= GENERATOR_START_RETRY_TIME) {
      generatorStartTries++;
      lastGeneratorStartTryTime = millis();
      startGenerator();
    }
  }
}

void serverLoop() {
  // Check if a client has connected
  WiFiClient client = server.accept();

  if (!client) {
    return;
  }

  client.setTimeout(2000);  // default is 1000

  // Read the first line of the request
  String req = client.readStringUntil('\r');
  yield();
  // Serial.println(req);

  // read/ignore the rest of the request
  while (client.available()) {
    // byte by byte is not very efficient
    client.read();
  }
  yield();
  
  // Outputs
  if (req.indexOf("api/") != -1) {
    // Actions
    if (req.indexOf("/api/generator/on") != -1) {
      requestStartGenerator();
    } else if (req.indexOf("/api/generator/off") != -1) {
      requestStopGenerator();
    } else if (req.indexOf("/api/generator/cancel") != -1) {
      cancelGeneratorStartRequest();
    }

    int chargeDischargeInt = -1;
    if (bmsCommunicationStatus) {
      if (bms.get.chargeDischargeStatus == "1") {
        chargeDischargeInt = 1;
      }
      else if (bms.get.chargeDischargeStatus == "2") {
        chargeDischargeInt = 2;
      }
      else {
        chargeDischargeInt = 0;
      }
    }

    client.print(F("HTTP/1.1 200 OK\r\nContent-Type: application/json\r\n\r\n"));
    client.print(F("{"));

    client.print(F("\"time\":")); client.print(timeClient.getEpochTime()); client.print(F(","));
    client.print(F("\"bmsCycles\":")); client.print(bms.get.bmsCycles); client.print(F(","));
    client.print(F("\"packSOC\":")); client.print(bms.get.packSOC); client.print(F(","));
    client.print(F("\"packVoltage\":")); client.print(bms.get.packVoltage); client.print(F(","));
    client.print(F("\"packCurrent\":")); client.print(bms.get.packCurrent); client.print(F(","));
    client.print(F("\"chargeDischargeStatus\":")); client.print(chargeDischargeInt); client.print(F(","));
    client.print(F("\"cellBalanceActive\":")); client.print(bms.get.cellBalanceActive); client.print(F(","));
    client.print(F("\"resCapacitymAh\":")); client.print(bms.get.resCapacitymAh); client.print(F(","));
    client.print(F("\"maxCellmV\":")); client.print(bms.get.maxCellmV); client.print(F(","));
    client.print(F("\"maxCellVNum\":")); client.print(bms.get.maxCellVNum); client.print(F(","));
    client.print(F("\"minCellmV\":")); client.print(bms.get.minCellmV); client.print(F(","));
    client.print(F("\"minCellVNum\":")); client.print(bms.get.minCellVNum); client.print(F(","));
    client.print(F("\"cellDiff\":")); client.print(bms.get.cellDiff); client.print(F(","));
    client.print(F("\"numberOfCells\":")); client.print(bms.get.numberOfCells); client.print(F(","));

    client.print(F("\"bmsCommunicationStatus\":")); client.print(bmsCommunicationStatus); client.print(F(","));
    client.print(F("\"generatorOn\":")); client.print(generatorOn); client.print(F(","));
    client.print(F("\"gridPowerOn\":")); client.print(gridPowerOn); client.print(F(","));
    client.print(F("\"inverterOn\":")); client.print(inverterOn); client.print(F(","));
    client.print(F("\"generatorStartRequested\":")); client.print(generatorStartRequested); client.print(F(","));
    client.print(F("\"generatorStopRequested\":")); client.print(generatorStopRequested); client.print(F(","));
    client.print(F("\"generatorStartTries\":")); client.print(generatorStartTries); client.print(F(","));

    client.print(F("\"gridPowerLostTime\":")); client.print(gridPowerLostTime); client.print(F(","));
    client.print(F("\"gridPowerRestoredTime\":")); client.print(gridPowerRestoredTime); client.print(F(","));
    client.print(F("\"generatorStartTime\":")); client.print(generatorStartTime); client.print(F(","));
    client.print(F("\"generatorStopTime\":")); client.print(generatorStopTime); client.print(F(","));

    client.print(F("\"cellVoltages\":["));
    for (int i = 0; i < bms.get.numberOfCells; i++) {
      client.print(bms.get.cellVmV[i]);
      if (i != bms.get.numberOfCells - 1) {
        client.print(F(","));
      }
    }
    client.print(F("],"));

    client.print(F("\"alarms\":{\"z\":false"));
    if (bms.alarm.levelOneCellVoltageTooHigh) client.print(F(",\"levelOneCellVoltageTooHigh\": true"));
    if (bms.alarm.levelTwoCellVoltageTooHigh) client.print(F(",\"levelTwoCellVoltageTooHigh\": true"));
    if (bms.alarm.levelOneCellVoltageTooLow) client.print(F(",\"levelOneCellVoltageTooLow\": true"));
    if (bms.alarm.levelTwoCellVoltageTooLow) client.print(F(",\"levelTwoCellVoltageTooLow\": true"));
    if (bms.alarm.levelOnePackVoltageTooHigh) client.print(F(",\"levelOnePackVoltageTooHigh\": true"));
    if (bms.alarm.levelTwoPackVoltageTooHigh) client.print(F(",\"levelTwoPackVoltageTooHigh\": true"));
    if (bms.alarm.levelOnePackVoltageTooLow) client.print(F(",\"levelOnePackVoltageTooLow\": true"));
    if (bms.alarm.levelTwoPackVoltageTooLow) client.print(F(",\"levelTwoPackVoltageTooLow\": true"));
    if (bms.alarm.levelOneChargeTempTooHigh) client.print(F(",\"levelOneChargeTempTooHigh\": true"));
    if (bms.alarm.levelTwoChargeTempTooHigh) client.print(F(",\"levelTwoChargeTempTooHigh\": true"));
    if (bms.alarm.levelOneChargeTempTooLow) client.print(F(",\"levelOneChargeTempTooLow\": true"));
    if (bms.alarm.levelTwoChargeTempTooLow) client.print(F(",\"levelTwoChargeTempTooLow\": true"));
    if (bms.alarm.levelOneDischargeTempTooHigh) client.print(F(",\"levelOneDischargeTempTooHigh\": true"));
    if (bms.alarm.levelTwoDischargeTempTooHigh) client.print(F(",\"levelTwoDischargeTempTooHigh\": true"));
    if (bms.alarm.levelOneDischargeTempTooLow) client.print(F(",\"levelOneDischargeTempTooLow\": true"));
    if (bms.alarm.levelTwoDischargeTempTooLow) client.print(F(",\"levelTwoDischargeTempTooLow\": true"));
    if (bms.alarm.levelOneChargeCurrentTooHigh) client.print(F(",\"levelOneChargeCurrentTooHigh\": true"));
    if (bms.alarm.levelTwoChargeCurrentTooHigh) client.print(F(",\"levelTwoChargeCurrentTooHigh\": true"));
    if (bms.alarm.levelOneDischargeCurrentTooHigh) client.print(F(",\"levelOneDischargeCurrentTooHigh\": true"));
    if (bms.alarm.levelTwoDischargeCurrentTooHigh) client.print(F(",\"levelTwoDischargeCurrentTooHigh\": true"));
    if (bms.alarm.levelOneStateOfChargeTooHigh) client.print(F(",\"levelOneStateOfChargeTooHigh\": true"));
    if (bms.alarm.levelTwoStateOfChargeTooHigh) client.print(F(",\"levelTwoStateOfChargeTooHigh\": true"));
    if (bms.alarm.levelOneStateOfChargeTooLow) client.print(F(",\"levelOneStateOfChargeTooLow\": true"));
    if (bms.alarm.levelTwoStateOfChargeTooLow) client.print(F(",\"levelTwoStateOfChargeTooLow\": true"));
    if (bms.alarm.levelOneCellVoltageDifferenceTooHigh) client.print(F(",\"levelOneCellVoltageDifferenceTooHigh\": true"));
    if (bms.alarm.levelTwoCellVoltageDifferenceTooHigh) client.print(F(",\"levelTwoCellVoltageDifferenceTooHigh\": true"));
    if (bms.alarm.levelOneTempSensorDifferenceTooHigh) client.print(F(",\"levelOneTempSensorDifferenceTooHigh\": true"));
    if (bms.alarm.levelTwoTempSensorDifferenceTooHigh) client.print(F(",\"levelTwoTempSensorDifferenceTooHigh\": true"));
    if (bms.alarm.chargeFETTemperatureTooHigh) client.print(F(",\"chargeFETTemperatureTooHigh\": true"));
    if (bms.alarm.dischargeFETTemperatureTooHigh) client.print(F(",\"dischargeFETTemperatureTooHigh\": true"));
    if (bms.alarm.failureOfChargeFETTemperatureSensor) client.print(F(",\"failureOfChargeFETTemperatureSensor\": true"));
    if (bms.alarm.failureOfDischargeFETTemperatureSensor) client.print(F(",\"failureOfDischargeFETTemperatureSensor\": true"));
    if (bms.alarm.failureOfChargeFETAdhesion) client.print(F(",\"failureOfChargeFETAdhesion\": true"));
    if (bms.alarm.failureOfDischargeFETAdhesion) client.print(F(",\"failureOfDischargeFETAdhesion\": true"));
    if (bms.alarm.failureOfChargeFETTBreaker) client.print(F(",\"failureOfChargeFETTBreaker\": true"));
    if (bms.alarm.failureOfDischargeFETBreaker) client.print(F(",\"failureOfDischargeFETBreaker\": true"));
    if (bms.alarm.failureOfAFEAcquisitionModule) client.print(F(",\"failureOfAFEAcquisitionModule\": true"));
    if (bms.alarm.failureOfVoltageSensorModule) client.print(F(",\"failureOfVoltageSensorModule\": true"));
    if (bms.alarm.failureOfTemperatureSensorModule) client.print(F(",\"failureOfTemperatureSensorModule\": true"));
    if (bms.alarm.failureOfEEPROMStorageModule) client.print(F(",\"failureOfEEPROMStorageModule\": true"));
    if (bms.alarm.failureOfRealtimeClockModule) client.print(F(",\"failureOfRealtimeClockModule\": true"));
    if (bms.alarm.failureOfPrechargeModule) client.print(F(",\"failureOfPrechargeModule\": true"));
    if (bms.alarm.failureOfVehicleCommunicationModule) client.print(F(",\"failureOfVehicleCommunicationModule\": true"));
    if (bms.alarm.failureOfIntranetCommunicationModule) client.print(F(",\"failureOfIntranetCommunicationModule\": true"));
    if (bms.alarm.failureOfCurrentSensorModule) client.print(F(",\"failureOfCurrentSensorModule\": true"));
    if (bms.alarm.failureOfMainVoltageSensorModule) client.print(F(",\"failureOfMainVoltageSensorModule\": true"));
    if (bms.alarm.failureOfShortCircuitProtection) client.print(F(",\"failureOfShortCircuitProtection\": true"));
    if (bms.alarm.failureOfLowVoltageNoCharging) client.print(F(",\"failureOfLowVoltageNoCharging\": true"));
    client.print(F("}"));

    client.print(F("}"));
  }
  else {
    // Send the response to the client
    client.print(F("HTTP/1.1 200 OK\r\nContent-Type: text/html\r\n\r\n"));
    client.print(F("<html>"));
    client.print(htmlChars);
    client.print(F("</html>"));
  }
  yield();

  client.flush();
}

void loop() {
  // Get BMS data
  bmsCommunicationStatus = bms.update();

  // Get grid and generator status
  generatorOn = digitalRead(GENERATOR_POWER_PIN) == LOW;
  gridPowerOn = digitalRead(GRID_POWER_PIN) == LOW;
  inverterOn = digitalRead(INVERTER_POWER_PIN) == LOW;

  if (gridPowerOn) {
    if (gridPowerLostTime != 0) {
      gridPowerRestoredTime = timeClient.getEpochTime();
      gridPowerLostTime = 0;
    }

    if (generatorOn && !generatorStopRequested) {
      // Serial.println("Grid power restored");
      requestStopGenerator();
    }
  }
  else {
    if (gridPowerLostTime == 0) {
      gridPowerLostTime = timeClient.getEpochTime();
      gridPowerRestoredTime = 0;
    }

    if (bmsCommunicationStatus) {
      int packSOC = bms.get.packSOC;

      // Don't do anything if we're not communicating properly with the BMS
      if (packSOC < BATTERY_MINIMUM_SOC) {
        if (!generatorOn && !generatorStartRequested) {
          // Serial.println("Batteries need charging");
          requestStartGenerator();
        }
      }
      else if (packSOC == BATTERY_MAXIMUM_SOC) {
        if (generatorOn && !generatorStopRequested) {
          // Serial.println("Batteries are fully charged");
          requestStopGenerator();
        }
      }
    }
  }

  // Run loops
  generatorStartLoop();
  generatorCooldownLoop();
  startStopButtonLoop();
  serverLoop();
}
