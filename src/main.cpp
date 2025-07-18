/*
  ROTOM Turbine Controller v1.5 (Analog Pump Control)
  - PUMP_PWM_PIN (GPIO 25)의 제어 방식을 기존 Servo PWM (1000-2000us)에서
    analogWrite (0-255) 방식으로 변경했습니다.
  - 관련 변수, 함수, UI 로직을 모두 새로운 제어 방식에 맞게 수정했습니다.
*/

#include <WiFi.h>
#include <ESPAsyncWebServer.h>
#include <AsyncTCP.h>
#include <SPI.h>
#include <SPIFFS.h>
#include <Adafruit_MAX31855.h>
#include <ESPmDNS.h>
#include <ArduinoJson.h>
#include <Preferences.h>
#include <functional>

// --- Pin assignments ---
#define POWER_PWM_PIN 32
#define THROTTLE_PWM_PIN 33
#define PUMP_RPM_PIN 34
#define TURBINE_RPM_PIN 35
#define PUMP_PWM_PIN 25
#define STARTER_PWM_PIN 26
#define GLOW_PIN 27
#define VALVE_PIN 23
#define MAX31855_CLK 18
#define MAX31855_CS 5
#define MAX31855_DO 19

// --- WiFi Configuration ---
const char *ssid = "afterburner";
const char *password = "12345678";

// --- Web Server & WebSocket ---
AsyncWebServer server(80);
AsyncWebSocket ws("/ws");

// --- NVS (Non-Volatile Storage) for Settings ---
Preferences preferences;

// --- Engine Settings Structure ---
struct EngineSettings
{
  unsigned long glowOnTime;
  unsigned long starterRampTime;
  unsigned long fuelRampTime;
  float fuelRampTemp;
  unsigned long ignitionCheckTime;
  float ignitionCheckTemp;
  float overTemperature;
  float coolingTemperature;
  int targetStarterPWM;
  int coolStarterPWM;
  int targetStartPumpPWM; // Changed from us to 0-255 scale
  int pumpDutyMin;        // 펌프 최소 듀티값
  int pumpDutyMax;        // 추가: 펌프 최대 듀티값
};

EngineSettings settings;

void loadSettings()
{
  preferences.begin("engine-settings", false);
  settings.glowOnTime = preferences.getULong("glowOnTime", 2000);
  settings.starterRampTime = preferences.getULong("starterRampTime", 5000);
  settings.fuelRampTime = preferences.getULong("fuelRampTime", 8000);
  settings.fuelRampTemp = preferences.getFloat("fuelRampTemp", 100.0);
  settings.ignitionCheckTime = preferences.getULong("ignTime", 10000);
  settings.ignitionCheckTemp = preferences.getFloat("ignTemp", 300.0);
  settings.overTemperature = preferences.getFloat("overTemperature", 900.0);
  settings.coolingTemperature = preferences.getFloat("coolingTemp", 100.0);
  settings.targetStarterPWM = preferences.getInt("targetStarter", 1300);
  settings.coolStarterPWM = preferences.getInt("coolStarter", 1300);
  settings.targetStartPumpPWM = preferences.getInt("targetPump", 20);
  settings.pumpDutyMin = preferences.getInt("pumpDutyMin", 20);
  settings.pumpDutyMax = preferences.getInt("pumpDutyMax", 255);
  preferences.end();
  Serial.println("Engine settings loaded.");
}

void saveSettings()
{
  preferences.begin("engine-settings", false);
  preferences.putULong("glowOnTime", settings.glowOnTime);
  preferences.putULong("starterRampTime", settings.starterRampTime);
  preferences.putULong("fuelRampTime", settings.fuelRampTime);
  preferences.putFloat("fuelRampTemp", settings.fuelRampTemp);
  preferences.putULong("ignTime", settings.ignitionCheckTime);
  preferences.putFloat("ignTemp", settings.ignitionCheckTemp);
  preferences.putFloat("overTemperature", settings.overTemperature);
  preferences.putFloat("coolingTemp", settings.coolingTemperature);
  preferences.putInt("targetStarter", settings.targetStarterPWM);
  preferences.putInt("coolStarter", settings.coolStarterPWM);
  preferences.putInt("targetPump", settings.targetStartPumpPWM);
  preferences.putInt("pumpDutyMin", settings.pumpDutyMin);
  preferences.putInt("pumpDutyMax", settings.pumpDutyMax); // 추가: 저장
  preferences.end();
  Serial.println("Engine settings saved permanently.");
}

// --- Engine State Machine ---
enum EngineState
{
  IDLE,
  ARMED,
  GLOW_ON,
  STARTER_ON,
  FUEL_RAMP,
  IGNITION_CHECK,
  RUNNING,
  COOLING,
  ERROR_STOP
};
EngineState engineState = IDLE;
unsigned long stateStartTime = 0;

// --- Control Variables ---
int starterPWM = 1000;
int pumpDuty = 0; // Changed from 1000 to 0 (0-255 scale)
int powerPWM = 1000;
int throttlePWM = 1000;
bool powerSwitch = false;
bool emergencyStop = false;
bool manualValveIntent = false;
bool manualGlowIntent = false;

// --- Flag for Error Reset Logic ---
bool errorAcknowledgedPowerLow = false;
bool errorResetPowerHighSeen = false;

// --- Sensor & Timing Variables ---
float temperature = 0.0;
Adafruit_MAX31855 thermocouple(MAX31855_CLK, MAX31855_CS, MAX31855_DO);
unsigned long lastTempReadTime = 0;
const unsigned long TEMP_READ_INTERVAL = 250;
unsigned long lastWsUpdateTime = 0;
const unsigned long WS_UPDATE_INTERVAL = 250;
unsigned long lastFsmUpdateTime = 0;
const unsigned long FSM_UPDATE_INTERVAL = 50;

// --- PWM Input (Interrupt based) ---
volatile unsigned long powerPulseStart = 0;
volatile int latestPowerPWM = 1000;
portMUX_TYPE powerMux = portMUX_INITIALIZER_UNLOCKED;
volatile unsigned long throttlePulseStart = 0;
volatile int latestThrottlePWM = 1000;
portMUX_TYPE throttleMux = portMUX_INITIALIZER_UNLOCKED;

// --- RPM Measurement Variables ---
portMUX_TYPE pumpRpmMux = portMUX_INITIALIZER_UNLOCKED;
volatile unsigned long pumpLastPulseTimeMicros = 0;
volatile unsigned long pumpPulsePeriodMicros = 0;
float pumpRPM = 0.0;
const unsigned long RPM_TIMEOUT_MS = 1000;
portMUX_TYPE turbineRpmMux = portMUX_INITIALIZER_UNLOCKED;
volatile unsigned long turbineLastPulseTimeMicros = 0;
volatile unsigned long turbinePulsePeriodMicros = 0;
float turbineRPM = 0.0;
unsigned long lastRpmCalcTime = 0;
const unsigned long RPM_CALC_INTERVAL = 100;

// --- Magic Numbers as Constants ---
const int PWM_MIN = 1000;
const int PWM_MAX = 2000;
const int PUMP_MIN = 0;
const int PUMP_MAX = 255;
const int POWER_ARMED = 1800;
const int POWER_DISARM = 1300;
const unsigned long COOL_INTERVAL = 5000;
const unsigned long COOL_PULSE_DURATION = 1000;

// --- PWM Input ISRs ---
void IRAM_ATTR isrPowerPWM()
{
  portENTER_CRITICAL_ISR(&powerMux);
  if (digitalRead(POWER_PWM_PIN) == HIGH)
  {
    powerPulseStart = micros();
  }
  else
  {
    if (powerPulseStart > 0)
    {
      latestPowerPWM = (int)(micros() - powerPulseStart);
      powerPulseStart = 0;
      if (latestPowerPWM < 800 || latestPowerPWM > 2200)
        latestPowerPWM = PWM_MIN;
    }
  }
  portEXIT_CRITICAL_ISR(&powerMux);
}
void IRAM_ATTR isrThrottlePWM()
{
  portENTER_CRITICAL_ISR(&throttleMux);
  if (digitalRead(THROTTLE_PWM_PIN) == HIGH)
  {
    throttlePulseStart = micros();
  }
  else
  {
    if (throttlePulseStart > 0)
    {
      latestThrottlePWM = (int)(micros() - throttlePulseStart);
      throttlePulseStart = 0;
      if (latestThrottlePWM < 800 || latestThrottlePWM > 2200)
        latestThrottlePWM = PWM_MIN;
    }
  }
  portEXIT_CRITICAL_ISR(&throttleMux);
}
void IRAM_ATTR isrPumpRPM()
{
  portENTER_CRITICAL_ISR(&pumpRpmMux);
  unsigned long nowMicros = micros();
  if (pumpLastPulseTimeMicros > 0)
  {
    pumpPulsePeriodMicros = nowMicros - pumpLastPulseTimeMicros;
  }
  pumpLastPulseTimeMicros = nowMicros;
  portEXIT_CRITICAL_ISR(&pumpRpmMux);
}
void IRAM_ATTR isrTurbineRPM()
{
  portENTER_CRITICAL_ISR(&turbineRpmMux);
  unsigned long nowMicros = micros();
  if (turbineLastPulseTimeMicros > 0)
  {
    turbinePulsePeriodMicros = nowMicros - turbineLastPulseTimeMicros;
  }
  turbineLastPulseTimeMicros = nowMicros;
  portEXIT_CRITICAL_ISR(&turbineRpmMux);
}

// --- Helper Functions ---
// This function now handles both analog (pump) and PWM (starter) outputs
void setActuatorOutputs(int pumpValue, int starterUs)
{
  // Pump: analogWrite, 0-255
  pumpValue = constrain(pumpValue, PUMP_MIN, PUMP_MAX);
  analogWrite(PUMP_PWM_PIN, pumpValue);
  // Starter: ledc PWM, 1000-2000us
  starterUs = constrain(starterUs, PWM_MIN, PWM_MAX);
  int starterDuty = map(starterUs, PWM_MIN, PWM_MAX, 3277, 6553);
  ledcWrite(1, starterDuty); // Starter is on channel 1
}

void setState(EngineState newState)
{
  if (engineState != newState)
  {
    Serial.printf("State Change: %d -> %d\n", engineState, newState);
    engineState = newState;
    stateStartTime = millis();

    if (newState != IDLE)
    {
      manualGlowIntent = false;
      manualValveIntent = false;
    }

    if (newState == IDLE || newState == ERROR_STOP)
    {
      powerSwitch = false;
      errorAcknowledgedPowerLow = false;
      errorResetPowerHighSeen = false;
    }
  }
}

void notifyClients()
{
  JsonDocument doc;
  doc["state"] = engineState;
  doc["glow"] = digitalRead(GLOW_PIN) ? "ON" : "OFF";
  doc["valve"] = digitalRead(VALVE_PIN) ? "ON" : "OFF";
  doc["starter"] = starterPWM;
  doc["pump"] = pumpDuty; // This will now be 0-255
  doc["temp"] = isnan(temperature) ? "Error" : String(temperature, 1);
  doc["power"] = powerPWM;
  doc["throttle"] = throttlePWM;
  doc["pumpRPM"] = String(pumpRPM, 0);
  doc["turbineRPM"] = String(turbineRPM, 0);

  JsonObject settingsObj = doc["settings"].to<JsonObject>();
  settingsObj["glowOnTime"] = settings.glowOnTime;
  settingsObj["starterRampTime"] = settings.starterRampTime;
  settingsObj["fuelRampTime"] = settings.fuelRampTime;
  settingsObj["fuelRampTemp"] = settings.fuelRampTemp;
  settingsObj["ignitionCheckTime"] = settings.ignitionCheckTime;
  settingsObj["ignitionCheckTemp"] = settings.ignitionCheckTemp;
  settingsObj["overTemperature"] = settings.overTemperature;
  settingsObj["coolingTemperature"] = settings.coolingTemperature;
  settingsObj["targetStarterPWM"] = settings.targetStarterPWM;
  settingsObj["coolStarterPWM"] = settings.coolStarterPWM;
  settingsObj["targetStartPumpPWM"] = settings.targetStartPumpPWM;
  settingsObj["pumpDutyMin"] = settings.pumpDutyMin;
  settingsObj["pumpDutyMax"] = settings.pumpDutyMax; // 추가: UI로 전송

  String json;
  serializeJson(doc, json);
  ws.textAll(json);
}

// --- WebSocket Handler ---
void handleWebSocket(AsyncWebSocket *server, AsyncWebSocketClient *client, AwsEventType type, void *arg, uint8_t *data, size_t len)
{
  if (type == WS_EVT_CONNECT)
  {
    Serial.printf("WebSocket client #%u connected from %s\n", client->id(), client->remoteIP().toString().c_str());
    notifyClients();
  }
  else if (type == WS_EVT_DISCONNECT)
  {
    Serial.printf("WebSocket client #%u disconnected\n", client->id());
  }
  else if (type == WS_EVT_DATA)
  {
    AwsFrameInfo *info = (AwsFrameInfo *)arg;
    if (info->final && info->index == 0 && info->len == len && info->opcode == WS_TEXT)
    {
      data[len] = 0;
      String msg = (char *)data;

      if (msg == "power:1")
      {
        powerSwitch = true;
      }
      else if (msg == "power:0")
      {
        powerSwitch = false;
        if (engineState == RUNNING)
        {
          setState(COOLING);
        }
      }
      else if (msg == "emergency")
      {
        emergencyStop = true;
      }
      else if (msg.startsWith("glow:"))
      {
        if (engineState == IDLE)
        {
          manualGlowIntent = (msg.substring(5).toInt() == 1);
        }
      }
      else if (msg.startsWith("valve:"))
      {
        if (engineState == IDLE)
        {
          manualValveIntent = (msg.substring(6).toInt() == 1);
        }
      }
      else if (msg.startsWith("starter:"))
      {
        if (engineState == IDLE)
        {
          starterPWM = constrain(msg.substring(8).toInt(), 1000, 2000);
        }
      }
      else if (msg.startsWith("pump:"))
      {
        // Now expects a value from 0-255
        if (engineState == IDLE)
        {
          pumpDuty = constrain(msg.substring(5).toInt(), 0, 255);
        }
      }
      else if (msg.startsWith("save_settings:"))
      {
        String settingsJson = msg.substring(14);
        Serial.println("[save_settings] Received JSON:");
        Serial.println(settingsJson);
        JsonDocument doc;
        DeserializationError error = deserializeJson(doc, settingsJson);
        if (!error)
        {
          settings.glowOnTime = doc["glowOnTime"];
          settings.starterRampTime = doc["starterRampTime"];
          settings.fuelRampTime = doc["fuelRampTime"];
          settings.fuelRampTemp = doc["fuelRampTemp"];
          settings.ignitionCheckTime = doc["ignitionCheckTime"];
          settings.ignitionCheckTemp = doc["ignitionCheckTemp"];
          settings.overTemperature = doc["overTemperature"];
          settings.coolingTemperature = doc["coolingTemperature"];
          settings.targetStarterPWM = doc["targetStarterPWM"];
          settings.coolStarterPWM = doc["coolStarterPWM"];
          settings.targetStartPumpPWM = doc["targetStartPumpPWM"];
          settings.pumpDutyMin = doc["pumpDutyMin"];
          settings.pumpDutyMax = doc["pumpDutyMax"]; // 추가: UI에서 저장
          saveSettings();
        }
        else
        {
          Serial.print(F("deserializeJson() failed: "));
          Serial.println(error.c_str());
        }
      }
    }
  }
  else if (type == WS_EVT_ERROR)
  {
    Serial.printf("WebSocket client #%u error #%u: %s\n", client->id(), *((uint16_t *)arg), (char *)data);
  }
}

// --- WiFi & PWM Setup ---
void setupWiFi()
{
  WiFi.softAP(ssid, password);
  Serial.print("AP Started. SSID: ");
  Serial.print(ssid);
  Serial.print(" IP: ");
  Serial.println(WiFi.softAPIP());
  if (MDNS.begin("afterburner"))
  {
    Serial.println("mDNS responder started: http://afterburner.local");
    MDNS.addService("http", "tcp", 80);
  }
  else
    Serial.println("Error setting up MDNS responder!");
}
void setupPWM()
{
  // Setup for analogWrite on PUMP_PWM_PIN (no need for ledc)
  pinMode(PUMP_PWM_PIN, OUTPUT);

  // Setup for ledc on STARTER_PWM_PIN (Channel 1)
  ledcSetup(1, 50, 16);
  ledcAttachPin(STARTER_PWM_PIN, 1);
}

// --- Finite State Machine (FSM) Logic ---
bool rampingStarter = false;
unsigned long starterRampStartTime = 0;
unsigned long lastCoolPulseTime = 0;
bool coolPulseActive = false;

void handleFSM()
{
  unsigned long now = millis();
  unsigned long stateElapsedTime = now - stateStartTime;

  if (emergencyStop)
  {
    setState(ERROR_STOP);
  }

  switch (engineState)
  {
  case IDLE:
    digitalWrite(GLOW_PIN, manualGlowIntent);
    digitalWrite(VALVE_PIN, manualValveIntent);
    setActuatorOutputs(pumpDuty, starterPWM);
    if ((powerPWM > POWER_ARMED || powerSwitch) && !emergencyStop)
    {
      setState(ARMED);
    }
    break;
  case ARMED:
    digitalWrite(VALVE_PIN, HIGH);
    digitalWrite(GLOW_PIN, HIGH);
    setActuatorOutputs(0, PWM_MIN); // pump off, starter off
    setState(GLOW_ON);
    break;
  case GLOW_ON:
    digitalWrite(VALVE_PIN, HIGH);
    digitalWrite(GLOW_PIN, HIGH);
    if (stateElapsedTime > settings.glowOnTime)
    {
      starterRampStartTime = now;
      rampingStarter = true;
      setState(STARTER_ON);
    }
    if (powerPWM < POWER_DISARM && !powerSwitch)
    {
      setState(IDLE);
    }
    break;
  case STARTER_ON:
    digitalWrite(VALVE_PIN, HIGH);
    digitalWrite(GLOW_PIN, HIGH);
    if (rampingStarter)
    {
      const unsigned long rampDuration = settings.starterRampTime;
      if (now - starterRampStartTime < rampDuration)
      {
        starterPWM = map(now - starterRampStartTime, 0, rampDuration, PWM_MIN, settings.targetStarterPWM);
      }
      else
      {
        starterPWM = settings.targetStarterPWM;
        rampingStarter = false;
        stateStartTime = now;
      }
    }
    else
    {
      if (stateElapsedTime > 1000)
      {
        pumpDuty = settings.targetStartPumpPWM;
        setState(FUEL_RAMP);
      }
    }
    setActuatorOutputs(0, starterPWM);
    if (powerPWM < POWER_DISARM && !powerSwitch)
    {
      setState(COOLING);
    }
    break;
  case FUEL_RAMP:
    digitalWrite(VALVE_PIN, HIGH);
    digitalWrite(GLOW_PIN, HIGH);
    starterPWM = settings.targetStarterPWM;
    pumpDuty = settings.targetStartPumpPWM;
    setActuatorOutputs(pumpDuty, starterPWM);
    if (temperature > settings.fuelRampTemp)
    {
      setState(IGNITION_CHECK);
    }
    if (stateElapsedTime > settings.fuelRampTime)
    {
      setState(ERROR_STOP);
    }
    if (powerPWM < POWER_DISARM && !powerSwitch)
    {
      setState(COOLING);
    }
    break;
  case IGNITION_CHECK:
    digitalWrite(VALVE_PIN, HIGH);
    starterPWM = settings.targetStarterPWM;
    pumpDuty = settings.targetStartPumpPWM;
    if (temperature > settings.ignitionCheckTemp)
    {
      digitalWrite(GLOW_PIN, LOW);
      starterPWM = PWM_MIN;
      setState(RUNNING);
    }
    else
    {
      digitalWrite(GLOW_PIN, HIGH);
    }
    setActuatorOutputs(pumpDuty, starterPWM);
    if (stateElapsedTime > settings.ignitionCheckTime)
    {
      setState(COOLING);
    }
    if (powerPWM < POWER_DISARM && !powerSwitch)
    {
      setState(COOLING);
    }
    break;
  case RUNNING:
    digitalWrite(VALVE_PIN, HIGH);
    digitalWrite(GLOW_PIN, LOW);
    starterPWM = PWM_MIN;
    if (powerPWM <= POWER_DISARM && !powerSwitch)
    {
      setState(COOLING);
    }
    else
    {
      pumpDuty = map(throttlePWM, PWM_MIN, PWM_MAX, settings.pumpDutyMin, settings.pumpDutyMax); // 변경: 설정값 사용
    }
    setActuatorOutputs(pumpDuty, starterPWM);
    if (temperature > settings.overTemperature)
    {
      setState(ERROR_STOP);
    }
    break;
  case COOLING:
  case ERROR_STOP:
    digitalWrite(VALVE_PIN, LOW);
    digitalWrite(GLOW_PIN, LOW);
    pumpDuty = 0;
    if (engineState == COOLING && temperature > settings.coolingTemperature)
    {
      if (!coolPulseActive && (now - lastCoolPulseTime >= COOL_INTERVAL))
      {
        starterPWM = settings.coolStarterPWM;
        lastCoolPulseTime = now;
        coolPulseActive = true;
      }
      else if (coolPulseActive && (now - lastCoolPulseTime >= COOL_PULSE_DURATION))
      {
        starterPWM = PWM_MIN;
        coolPulseActive = false;
        lastCoolPulseTime = now;
      }
      else if (!coolPulseActive)
      {
        starterPWM = PWM_MIN;
      }
    }
    else
    {
      starterPWM = PWM_MIN;
      if (engineState == COOLING && temperature <= settings.coolingTemperature)
      {
        setState(IDLE);
      }
    }
    setActuatorOutputs(pumpDuty, starterPWM);
    if (engineState == ERROR_STOP)
    {
      if (!errorAcknowledgedPowerLow && (powerPWM < POWER_DISARM && !powerSwitch))
      {
        errorAcknowledgedPowerLow = true;
        errorResetPowerHighSeen = false;
      }
      else if (errorAcknowledgedPowerLow && !errorResetPowerHighSeen && (powerPWM > POWER_ARMED || powerSwitch))
      {
        errorResetPowerHighSeen = true;
      }
      else if (errorAcknowledgedPowerLow && errorResetPowerHighSeen && (powerPWM < POWER_DISARM && !powerSwitch))
      {
        setState(IDLE);
        emergencyStop = false;
      }
    }
    break;
  }
}

// --- Arduino Setup Function ---
void setup()
{
  Serial.begin(115200);
  Serial.println("\nROTOM Turbine Controller v1.5 (Analog Pump) Booting...");

  loadSettings();

  pinMode(GLOW_PIN, OUTPUT);
  digitalWrite(GLOW_PIN, LOW);
  pinMode(VALVE_PIN, OUTPUT);
  digitalWrite(VALVE_PIN, LOW);
  pinMode(POWER_PWM_PIN, INPUT_PULLUP);
  pinMode(THROTTLE_PWM_PIN, INPUT_PULLUP);
  pinMode(PUMP_RPM_PIN, INPUT);
  pinMode(TURBINE_RPM_PIN, INPUT);

  if (!SPIFFS.begin(true))
  {
    Serial.println("SPIFFS Mount Failed!");
    while (1)
      delay(1000);
  }

  Serial.print("Initializing MAX31855...");
  if (!thermocouple.begin())
    Serial.println(" ERROR!");
  else
  {
    Serial.println(" OK.");
  }

  setupWiFi();
  setupPWM();
  ws.onEvent(handleWebSocket);
  server.addHandler(&ws);
  server.serveStatic("/", SPIFFS, "/").setDefaultFile("index.html");
  server.onNotFound([](AsyncWebServerRequest *request)
                    { request->send(404, "text/plain", "Not found"); });
  server.begin();

  attachInterrupt(digitalPinToInterrupt(POWER_PWM_PIN), isrPowerPWM, CHANGE);
  attachInterrupt(digitalPinToInterrupt(THROTTLE_PWM_PIN), isrThrottlePWM, CHANGE);
  attachInterrupt(digitalPinToInterrupt(PUMP_RPM_PIN), isrPumpRPM, RISING);
  attachInterrupt(digitalPinToInterrupt(TURBINE_RPM_PIN), isrTurbineRPM, RISING);

  stateStartTime = millis();
  Serial.println("Setup Complete. Entering Main Loop.");
}

// --- Arduino Loop Function ---
void loop()
{
  unsigned long currentTime = millis();
  unsigned long currentMicros = micros();

  portENTER_CRITICAL(&powerMux);
  powerPWM = latestPowerPWM;
  portEXIT_CRITICAL(&powerMux);
  portENTER_CRITICAL(&throttleMux);
  throttlePWM = latestThrottlePWM;
  portEXIT_CRITICAL(&throttleMux);

  if (currentTime - lastTempReadTime >= TEMP_READ_INTERVAL)
  {
    lastTempReadTime = currentTime;
    float newTemp = thermocouple.readCelsius();
    if (!isnan(newTemp))
      temperature = newTemp;
  }

  if (currentTime - lastRpmCalcTime >= RPM_CALC_INTERVAL)
  {
    lastRpmCalcTime = currentTime;
    unsigned long pumpPeriod = 0, pumpLastSeen = 0;
    portENTER_CRITICAL(&pumpRpmMux);
    pumpPeriod = pumpPulsePeriodMicros;
    pumpLastSeen = pumpLastPulseTimeMicros;
    portEXIT_CRITICAL(&pumpRpmMux);
    if ((currentMicros - pumpLastSeen) > (RPM_TIMEOUT_MS * 1000UL))
      pumpRPM = 0.0;
    else if (pumpPeriod > 0)
      pumpRPM = 60000000.0f / pumpPeriod;

    unsigned long turbinePeriod = 0, turbineLastSeen = 0;
    portENTER_CRITICAL(&turbineRpmMux);
    turbinePeriod = turbinePulsePeriodMicros;
    turbineLastSeen = turbineLastPulseTimeMicros;
    portEXIT_CRITICAL(&turbineRpmMux);
    if ((currentMicros - turbineLastSeen) > (RPM_TIMEOUT_MS * 1000UL))
      turbineRPM = 0.0;
    else if (turbinePeriod > 0)
      turbineRPM = 60000000.0f / turbinePeriod;
  }

  if (currentTime - lastFsmUpdateTime >= FSM_UPDATE_INTERVAL)
  {
    lastFsmUpdateTime = currentTime;
    handleFSM();
  }

  if (currentTime - lastWsUpdateTime >= WS_UPDATE_INTERVAL)
  {
    lastWsUpdateTime = currentTime;
    notifyClients();
  }
  ws.cleanupClients();
}