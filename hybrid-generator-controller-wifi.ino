#include <daly-bms-uart.h>
#include <ESP8266WiFi.h>

#define STASSID "davisRouter"
#define STAPSK "0192837465"

#define BMS_SERIAL Serial1 // Set the serial port for communication with the Daly BMS

#define GENERATOR_SIGNAL_PIN 18
#define GRID_POWER_PIN 19
#define GENERATOR_POWER_PIN 20

#define MAX_GENERATOR_START_TRIES 5

#define GENERATOR_START_RETRY_TIME 30000 // 0.5 * 60 * 100 // 30 seconds
#define GENERATOR_COOLDOWN_TIME 300000 // 300000 // 5 * 60 * 1000 // 5 minutes
#define SAMPLE_SPEED 200
#define SIGNAL_TIME 399

#define BATTERY_MINIMUM_SOC 15 // 15%
#define BATTERY_MAXIMUM_SOC 100 // 100%

#define LED_ON LOW
#define LED_OFF HIGH

const char* ssid = STASSID;
const char* password = STAPSK;

// Construct the bms driver and passing in the Serial interface (which pins to use)
Daly_BMS_UART bms(BMS_SERIAL);

// Create an instance of the server
// specify the port to listen on as an argument
WiFiServer server(80);

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

// Status variables
float packVoltage = 0;
float packSOC = 0;

void setup() {
  // This is needed to print stuff to the serial monitor
  Serial.begin(115200);

  Serial.print(F("Connecting to "));
  Serial.println(ssid);

  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, password);

  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(F("."));
  }
  Serial.println();
  Serial.println(F("WiFi connected"));

  // Start the server
  server.begin();
  Serial.println(F("Server started"));

  // Print the IP address
  Serial.println(WiFi.localIP());

  // Pin setup
  pinMode(GENERATOR_SIGNAL_PIN, OUTPUT);
  pinMode(GRID_POWER_PIN, INPUT_PULLUP);
  pinMode(GENERATOR_POWER_PIN, INPUT_PULLUP);

  pinMode(LED_BUILTIN, OUTPUT);

  // This call sets up the driver
  bms.Init();
}

void blinkLED(int times, int waitTime = 250) {
  for (int i = 0; i < times; i++) {
    digitalWrite(LED_BUILTIN, LED_OFF);
    delay(waitTime);
    digitalWrite(LED_BUILTIN, LED_ON);
    delay(waitTime);
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
      digitalWrite(GENERATOR_SIGNAL_PIN, LOW); // Release the button 
      startStopButtonPressed = false; 
    }
  }
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

void cancelGeneratorStartRequest() {
  generatorStartRequested = false;
  lastGeneratorStartTryTime = 0;
  generatorStartTries = 0;
}

void generatorStartLoop() {
  if (generatorStartRequested && generatorOn) {
    // Generator successfully started
    cancelGeneratorStartRequest();
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

void printWebStatus(WiFiClient client, String item, String status) {
  client.print("<p><stong>" + item + ":</strong> " + status + "</p>");
}

void serverLoop() {
  // Check if a client has connected
  WiFiClient client = server.accept();

  if (!client) {
    return;
  }

  client.setTimeout(5000);  // default is 1000

  // Read the first line of the request
  String req = client.readStringUntil('\r');
  Serial.println(req);

  // Match the request
  if (req.indexOf("/generator/on") != -1) {
    requestStartGenerator();
  } else if (req.indexOf("/generator/off") != -1) {
    requestStopGenerator();
  } else if (req.indexOf("/generator/cancel") != -1) {
    cancelGeneratorStartRequest();
  } else if (req.indexOf("favicon.svg") != -1) {
    client.print("HTTP/1.1 200 OK\r\nContent-Type: image/svg+xml\r\n\r\n");
    client.print("<svg fill='#FFFF00' width='76' height='76' viewBox='0 0 560.317 560.316' xmlns='http://www.w3.org/2000/svg'><path d='M207.523,560.316c0,0,194.42-421.925,194.444-421.986l10.79-23.997c-41.824,12.02-135.271,34.902-135.57,35.833C286.96,122.816,329.017,0,330.829,0c-39.976,0-79.952,0-119.927,0l-12.167,57.938l-51.176,209.995l135.191-36.806L207.523,560.316z'/></svg>");
    return;
  }

  // read/ignore the rest of the request
  // do not client.flush(): it is for output only, see below
  while (client.available()) {
    // byte by byte is not very efficient
    client.read();
  }

  // Send the response to the client
  // it is OK for multiple small client.print/write,
  // because nagle algorithm will group them into one single packet
  client.print("HTTP/1.1 200 OK\r\nContent-Type: text/html\r\n\r\n");
  client.print("<!DOCTYPE HTML>");
  client.print("<html>");
  client.print("<head><link rel='icon' href='/favicon.svg'></head>");
  client.print("<body>");

  printWebStatus(client, "Batteries", (String)packVoltage + "V " + "("+ (String)packSOC + "%)");
  printWebStatus(client, "Grid power", (String)(gridPowerOn ? "ON" : "OFF"));
  printWebStatus(client, "Generator", (String)(generatorOn ? "ON" : "OFF"));

  printWebStatus(client, "Stop requested", (String)(generatorStopRequested ? "YES" : "NO"));
  printWebStatus(client, "Start requested", (String)(generatorStartRequested ? "YES" : "NO"));
  printWebStatus(client, "Start tries", (String)generatorStartTries);

  const String generatorActionLabel = (generatorOn ? "off" : "on");
  if (!generatorStartRequested) {
    client.print("<button onclick='window.location=\"/generator/" + generatorActionLabel + "\"'>Turn generator " + generatorActionLabel + "</button>");
  }
  else {
    client.print("<button onclick='window.location=\"/generator/cancel\"'>Cancel generator start request</button>");
  }

  client.print("<body></html>");
}

void loop() {
  // Get BMS data
  bool communicationStatus = bms.update();

  // Don't do anything if we have no data
  /*
  if (!communicationStatus) {
    Serial.println("Failed to communicate with BMS, waiting...");
    blinkLED(5);
    delay(5000);
    return;
  }
  */

  packVoltage = bms.get.packVoltage;
  packSOC = bms.get.packSOC;

  // Get grid and generator status
  generatorOn = digitalRead(GENERATOR_POWER_PIN) == HIGH;
  gridPowerOn = digitalRead(GRID_POWER_PIN) == HIGH;

  /*
  Serial.println("Grid power: " + (String)(gridPowerOn ? "ON" : "OFF"));
  Serial.println("Generator:  " + (String)(generatorOn ? "ON" : "OFF"));
  Serial.println("Cooldown:   " + (String)(generatorStopRequested ? "YES" : "NO"));
  Serial.println("Button:     " + (String)(startStopButtonPressed ? "ON" : "OFF"));
  Serial.println("Batteries:  " + (String)packVoltage + "V " + "("+ (String)packSOC + "%)");
  */

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
  serverLoop();

  Serial.println("");
}
