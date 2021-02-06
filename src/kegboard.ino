/**
 * Copyright 2016-2020 The Kegbot Project contributors <https://kegbot.org/>
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */

#define VERSION "0.3.0"
#define KEGBOARD_DEBUG 0

#define CLOUD_PUBLISH_INTERVAL_MILLIS 1000
#define TCP_PUBLISH_INTERVAL_MILLIS 250
#define CONSOLE_PUBLISH_INTERVAL_MILLIS 250
#define CLIENT_WATCHDOG_TIMEOUT_MILLIS 30000

#define COMMAND_WATCHDOG_ON "watchdog on"
#define COMMAND_WATCHDOG_OFF "watchdog off"
#define COMMAND_WATCHDOG_KICK "watchdog kick"

#define NUM_METERS 4
#define METER0_PIN 36
#define METER1_PIN 39
#define METER2_PIN 34
#define METER3_PIN 35

#define ONEWIRE_PIN 32

#define TCP_SERVER_PORT 8321
#define TCP_CLIENT_INCOMING_BUFSIZE 256

#include <WiFi.h>
#include <ESPmDNS.h>
#include <OneWire.h>
#include <ds1820.h>
#include <Preferences.h>

#if KEGBOARD_DEBUG
SerialLogHandler logHandler;
#endif

WiFiServer server(TCP_SERVER_PORT);
WiFiClient client;
Preferences prefs;

char clientBuffer[TCP_CLIENT_INCOMING_BUFSIZE] = { '\0' };
unsigned int clientBufferPos = 0;

volatile unsigned int watchdogEnabled = 0;
unsigned long lastWatchdogKick;

volatile unsigned int meterPending;
volatile unsigned int thermoPending;

volatile unsigned int consolePending;
unsigned long lastConsolePublishMillis;

volatile unsigned int tcpPending;
unsigned long lastTcpPublishMillis;

typedef struct {
  volatile unsigned int ticks;
} meter_t;

meter_t meters[NUM_METERS];

OneWire onewire = OneWire(ONEWIRE_PIN);
DS1820Sensor thermoSensor = DS1820Sensor();

typedef struct {
  float temp;
  char probe[17];
} temp_t;

temp_t temps[NUM_METERS];

#define CREATE_METER_ISR(METER_NUM) \
  void IRAM_ATTR meter##METER_NUM##Interrupt(void) { \
    detachInterrupt(METER##METER_NUM##_PIN); \
    meters[METER_NUM].ticks++; \
    consolePending = tcpPending = meterPending = 1; \
    attachInterrupt(METER##METER_NUM##_PIN, meter##METER_NUM##Interrupt, FALLING); \
  }

#define SETUP_METER(METER_NUM) \
  do { \
    memset(&meters[METER_NUM], 0, sizeof(meter_t)); \
    pinMode(METER##METER_NUM##_PIN, INPUT_PULLUP); \
    attachInterrupt(METER##METER_NUM##_PIN, meter##METER_NUM##Interrupt, FALLING); \
  } while (false)

#define SETUP_TEMP(TEMP_NUM) \
  do { \
    memset(&temps[TEMP_NUM], 0, sizeof(temp_t)); \
  } while (false)

CREATE_METER_ISR(0);
CREATE_METER_ISR(1);
CREATE_METER_ISR(2);
CREATE_METER_ISR(3);

//
// Main program
//

int resetMeter(int meterNum) {
  if (meterNum < 0 || meterNum >= NUM_METERS) {
    return -1;
  }
  memset(&meters[meterNum], 0, sizeof(meter_t));
  return 0;
}

void enableClientWatchdog() {
  lastWatchdogKick = millis();
  watchdogEnabled = 1;
}

void disableClientWatchdog() {
  watchdogEnabled = 0;
}

void kickClientWatchdog() {
  if (!watchdogEnabled) {
    enableClientWatchdog();
  }
  lastWatchdogKick = millis();
}

void checkClientWatchdog() {
  if (!watchdogEnabled) {
    return;
  }
  if (!client.connected()) {
    return;
  }
  if ((millis() - lastWatchdogKick) >= CLIENT_WATCHDOG_TIMEOUT_MILLIS) {
    ESP_LOGI(TAG, "Watchdog expired");
    client.println("error: watchdog expired");
    client.stop();
    watchdogEnabled = 0;
  }
}

void addTemperature(char* probe, float temp) {
  for (int i = 0; i < NUM_METERS; i++) {
   if (strcmp(temps[i].probe, probe) == 0) {
      temps[i].temp = temp;
      return;
    }
  }
  // New probe ID; Store it.
  for (int i = 0; i < NUM_METERS; i++) {
    if (temps[i].probe[0] == '\0') {
      std::copy(probe, probe+17, temps[i].probe);
      temps[i].temp = temp;
      return;
    }
  }
}

int stepOnewireThermoBus() {
  uint8_t addr[8];
  unsigned long now = millis();

  // Are we already working on a sensor? service it, possibly emitting a a
  // thermo packet.
  if (thermoSensor.Initialized() || thermoSensor.Busy()) {
    if (thermoSensor.Update(now)) {
      char buf[64];
      char nameBuf[17];
      // Just finished conversion
      //writeThermoPacket(&gThermoSensor);
      thermoSensor.GetTempC(buf);
      thermoSensor.GetName(nameBuf);
      if (buf[0] != '\0') {
        addTemperature(nameBuf,atof(buf));
        consolePending = tcpPending = thermoPending = 1;
      }
      thermoSensor.Reset();
    } else if (thermoSensor.Busy()) {
      // More cycles needed on this sensor
      return 1;
    } else {
      // finished or not started
    }

    // First time, or finished with last sensor; clean up, and look more more
    // devices.
    int more_search = onewire.search(addr);
    if (!more_search) {
      // Bus exhausted; start over
      onewire.reset_search();
      return 0;
    }
    // New sensor. Initialize and start work.
    thermoSensor.Initialize(&onewire, addr);
    thermoSensor.Update(now);
    return 1;
  }

  // First time, or finished with last sensor; clean up, and look more more
  // devices.
  int more_search = onewire.search(addr);
  if (!more_search) {
    // Bus exhausted; start over
    onewire.reset_search();
    return 0;
  }

  // New sensor. Initialize and start work.
  thermoSensor.Initialize(&onewire, addr);
  thermoSensor.Update(now);
  return 1;
}

void setup() {
  ESP_LOGI(TAG, "Starting setup ...");
  Serial.begin(115200);

  prefs.begin("wifi");
  if (prefs.getBool("sc", true)) {
    WiFi.mode(WIFI_AP_STA);
    WiFi.beginSmartConfig();

    ESP_LOGI(TAG, "Waiting for WiFi SmartConfig...");
    while (!WiFi.smartConfigDone()) {
      delay(500);
    }
    ESP_LOGI(TAG, "SmartConfig received.");
    prefs.putString("ssid", WiFi.SSID());
    prefs.putString("psk", WiFi.psk());
    prefs.putBool("sc", false);
  } else {
    WiFi.begin(prefs.getString("ssid").c_str(), prefs.getString("psk").c_str());
  }
  prefs.end();

  ESP_LOGI(TAG, "Connecting...");
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
  }

  Serial.print("start: kegboard-particle online, ip: ");
  Serial.println(WiFi.localIP());

  server.begin();

  SETUP_METER(0);
  SETUP_METER(1);
  SETUP_METER(2);
  SETUP_METER(3);

  SETUP_TEMP(0);
  SETUP_TEMP(1);
  SETUP_TEMP(2);
  SETUP_TEMP(3);

  MDNS.begin("kegboard");
  MDNS.addService("kegboard", "tcp", TCP_SERVER_PORT);
}

void getStatus(String* statusMessage) {
  for (int i = 0; i < NUM_METERS; i++) {
    meter_t *meter = &meters[i];
    unsigned int ticks = meter->ticks;
    statusMessage->concat("meter");
    statusMessage->concat(i);
    statusMessage->concat(".ticks=");
    statusMessage->concat(ticks);
  }
}

void getThermoStatus(String* statusMessage) {
  for (int i = 0; i < NUM_METERS; i++) {
    if (temps[i].probe[0] != '\0') {
      statusMessage->concat("temp_");
      statusMessage->concat(temps[i].probe);
      statusMessage->concat(".temp=");
      statusMessage->concat(temps[i].temp);
    }
  }
}

void handleCommand(char *command) {
  // ESP_LOGI(TAG, "Handling command: %s ...", command);
  String commandString = String(command);
  if (commandString.equals(COMMAND_WATCHDOG_ON)) {
    ESP_LOGI(TAG, "Enabling watchdog.");
    enableClientWatchdog();
  } else if (commandString.equals(COMMAND_WATCHDOG_OFF)) {
    ESP_LOGI(TAG, "Disabling watchdog.");
    disableClientWatchdog();
  } else if (commandString.equals(COMMAND_WATCHDOG_KICK)) {
    ESP_LOGI(TAG, "Watchdog kicked.");
    kickClientWatchdog();
  } else {
    ESP_LOGI(TAG, "Unknown command");
  }
}

void serviceCurrentClient() {
  if (!client.connected()) {
    return;
  }

  while (client.available()) {
    char c = client.read();
    if ((int)c == -1) {
      break;
    }

    // Always ignore CRs.
    if (c == '\r') {
      continue;
    }

    // Consume and ignore leading whitespace.
    if (clientBufferPos == 0) {
      if (c == '\n' || c == ' ') {
        continue;
      }
    }

    clientBuffer[clientBufferPos++] = c;

    // We've got a complete command.
    if (c == '\n') {
      break;
    }

    // Haven't received a complete command & about to overflow.
    if (clientBufferPos == (TCP_CLIENT_INCOMING_BUFSIZE - 1)) {
      ESP_LOGI(TAG, "Incoming buffer overflow");
      clientBufferPos = 0;
    }
  }

  if (clientBufferPos > 0 && clientBuffer[clientBufferPos - 1] == '\n') {
    clientBuffer[clientBufferPos - 1] = '\0';
    handleCommand(clientBuffer);
    clientBufferPos = 0;
    clientBuffer[0] = '\0';
  }
}

void checkAndServiceTcp() {
  // Check for a new client.
  if (!client.connected()) {
    client = server.available();
    if (client.connected()) {
      ESP_LOGI(TAG, "TCP client connectd");
      watchdogEnabled = 0;
      client.print("info: kegboard-particle device_id=");
      client.print(WiFi.macAddress());
      client.print(" version=");
      client.print(VERSION);
      client.println();
    }
  }

  if (client.connected()) {
    serviceCurrentClient();
  }
}

void publishTcpStatus() {
  if (!tcpPending) {
    return;
  }
  tcpPending = 0;

  if (client.connected()) {
    String statusMessage;
    if (meterPending) {
      getStatus(&statusMessage);
      client.print("kb-status: ");
      client.println(statusMessage);
    }
    if (thermoPending) {
      getThermoStatus(&statusMessage);
      client.print("kb-thermo: ");
      client.println(statusMessage);
    }
  }
  lastTcpPublishMillis = millis();
}

void publishConsoleStatus() {
  if (!consolePending) {
    return;
  }
  consolePending = 0;

  String statusMessage;
  if (meterPending) {
    getStatus(&statusMessage);
    Serial.print("kb-status: ");
    Serial.println(statusMessage);
  }
  if (thermoPending) {
    statusMessage = "";
    getThermoStatus(&statusMessage);
    Serial.print("kb-thermo: ");
    Serial.println(statusMessage);
  }
  lastConsolePublishMillis = millis();
}

void loop() {
  checkClientWatchdog();
  checkAndServiceTcp();
  stepOnewireThermoBus();

  if (client.connected()) {
    if ((millis() - lastTcpPublishMillis) >= TCP_PUBLISH_INTERVAL_MILLIS) {
      publishTcpStatus();
    }
  }

  if ((millis() - lastConsolePublishMillis) >= CONSOLE_PUBLISH_INTERVAL_MILLIS) {
    publishConsoleStatus();
  }
  meterPending = thermoPending = 0;
}
