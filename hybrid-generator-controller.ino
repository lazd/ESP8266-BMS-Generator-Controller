#include <Arduino.h>
#include <daly-bms-uart.h>

#define BMS_SERIAL Serial1 // Set the serial port for communication with the Daly BMS

#define TX_LED 30
#define RX_LED 17

#define GENERATOR_SIGNAL_PIN 9
#define GRID_POWER_PIN 16
#define GENERATOR_POWER_PIN 10

#define MAX_GENERATOR_START_TRIES 5

#define GENERATOR_START_RETRY_TIME 30000 // 0.5 * 60 * 100 // 30 seconds
#define GENERATOR_COOLDOWN_TIME 300000 // 300000 // 5 * 60 * 1000 // 5 minutes
#define SAMPLE_SPEED 200
#define SIGNAL_TIME 399

#define BATTERY_MINIMUM_SOC 15 // 15%
#define BATTERY_MAXIMUM_SOC 100 // 100%

// Constructing the bms driver and passing in the Serial interface (which pins to use)
Daly_BMS_UART bms(BMS_SERIAL);

// State variables
bool generatorOn = false;
bool gridPowerOn = false;
bool startStopButtonPressed = false;
bool generatorStopRequested = false;
bool generatorStartRequested = false;
int generatorStartTries = 0;
unsigned long lastGeneratorStartTryTime = 0;
unsigned long generatorCooldownStartTime = 0;
unsigned long signalStartTime = 0;

bool startStopButtonLoop() {
  if (startStopButtonPressed) {
    if ((millis() - signalStartTime) >= SIGNAL_TIME) {
      digitalWrite(GENERATOR_SIGNAL_PIN, LOW); // Release the button 
      startStopButtonPressed = false; 
    }
  }
}

bool pressStartStopButton() {
  if (startStopButtonPressed) {
    return false;
  }

  startStopButtonPressed = true;
  digitalWrite(GENERATOR_SIGNAL_PIN, HIGH); // Press the button  
  signalStartTime = millis();

  return true;
}

bool startGenerator() {
  if (generatorOn) {
    return false;
  }

  Serial.println("Starting generator");

  // Power generator on
  pressStartStopButton();
  
  return true;
}

bool stopGenerator() {
  if (!generatorOn) {
    return false;
  }

  Serial.println("Stopping generator");

  // Stop generator
  pressStartStopButton();

  generatorStopRequested = false;

  return true;
}

bool requestStopGenerator() {
  if (!generatorOn) {
    return false;
  }

  if (generatorStopRequested) {
    return false;
  }

  Serial.println("Cooling down generator");

  generatorStopRequested = true;
  generatorCooldownStartTime = millis();

  return true;
}

void generatorCooldownLoop() {
  if (generatorStopRequested) {
    // We've cooled down enough, stop the generator
    if ((millis() - generatorCooldownStartTime) >= GENERATOR_COOLDOWN_TIME) {
      Serial.println("Generator cool down complete");
      stopGenerator();
    }
  }
}

bool requestStartGenerator() {
  if (!generatorStartRequested) {
    generatorStartRequested = true;
  }
}

void generatorStartLoop() {
  if (generatorStartRequested && generatorOn) {
    // Generator successfully started
    generatorStartRequested = false;
    lastGeneratorStartTryTime = 0;
    generatorStartTries = 0;
    Serial.println("Generator start request successful");
    return;
  }

  if (generatorOn) {
    return;
  }

  if (generatorStartRequested) {
    if (generatorStartTries == MAX_GENERATOR_START_TRIES) {
      Serial.println("Generator failed to start after " + (String)MAX_GENERATOR_START_TRIES + " tries, cannot start generator!");
      return;
    }

    if (generatorStartTries == 0 || (millis() - lastGeneratorStartTryTime) >= GENERATOR_START_RETRY_TIME) {
      generatorStartTries++;
      lastGeneratorStartTryTime = millis();
      startGenerator();
    }
  }
}

void setup()
{
  // This is needed to print stuff to the serial monitor
  Serial.begin(115200);

  pinMode(GENERATOR_SIGNAL_PIN, OUTPUT);
  pinMode(GRID_POWER_PIN, INPUT_PULLUP);
  pinMode(GENERATOR_POWER_PIN, INPUT_PULLUP);

  pinMode(TX_LED, OUTPUT);
  pinMode(RX_LED, OUTPUT);

  // This call sets up the driver
  bms.Init();

  delay(2000);
}

void blinkLED(int times, int waitTime = 250) {
  for (int i = 0; i < times; i++) {
    digitalWrite(RX_LED, LOW);
    delay(waitTime);
    digitalWrite(RX_LED, HIGH);
    delay(waitTime);
  }
}

void loop()
{
  digitalWrite(RX_LED, LOW);

  // Get BMS data
  bool communicationStatus = bms.update();

  // Don't do anything if we have no data
  if (!communicationStatus) {
    Serial.println("Failed to communicate with BMS, waiting...");
    blinkLED(5);
    delay(5000);
    return;
  }

  float packVoltage = bms.get.packVoltage;
  float packSOC = bms.get.packSOC;

  // Get grid and generator status
  generatorOn = digitalRead(GENERATOR_POWER_PIN) == LOW;
  gridPowerOn = digitalRead(GRID_POWER_PIN) == LOW;

  Serial.println("Grid power: " + (String)(gridPowerOn ? "ON" : "OFF"));
  Serial.println("Generator:  " + (String)(generatorOn ? "ON" : "OFF"));
  Serial.println("Cooldown:   " + (String)(generatorStopRequested ? "YES" : "NO"));
  Serial.println("Button:     " + (String)(startStopButtonPressed ? "ON" : "OFF"));
  Serial.println("Batteries:  " + (String)packVoltage + "V " + "("+ (String)packSOC + "%)");

  if (gridPowerOn) {
    if (generatorOn && !generatorStopRequested) {
      Serial.println("Grid power restored");
      requestStopGenerator();
    }
  }
  else {
    if (packSOC < BATTERY_MINIMUM_SOC) {
      if (!generatorOn && !generatorStartRequested) {
        Serial.println("Batteries need charging");
        requestStartGenerator();
      }
    }
    else if (packSOC == BATTERY_MAXIMUM_SOC) {
      if (generatorOn && !generatorStopRequested) {
        Serial.println("Batteries are fully charged");
        requestStopGenerator();
      }
    }
  }

  // Run loops
  generatorStartLoop();
  generatorCooldownLoop();
  startStopButtonLoop();

  Serial.println("");

  delay(SAMPLE_SPEED);
  digitalWrite(RX_LED, HIGH);
}
