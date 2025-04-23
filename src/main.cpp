/* 조종기 사용 시 작동 시퀀스
1. POWER_ON:        조종기의 Power 채널 스위치를 올려 powerPWM 신호가 1800µs 이상이 되도록 함.
2. GLOW_ON:         glow plug가 켜지고 3초 대기.
3. STARTER_ON:      스타터 모터가 2초에 걸쳐 1000~1500us까지 ramp up. ramping 완료  후 1초 대기.
4. FUEL_RAMP:       연료펌프 작동 시작(1100us). 8초 안에 100도씨 이상으로 온도가 올라가야 함.
5. IGNITION_CHECK:  100도씨 초과 후 5초 안에 300도씨 이상으로 올라가야 함.
6. RUNNING:         300도씨 초과 시 glow plug, 스타터 모터 끔.

7. powerPWM값을 확인하며 1300us 이하일 경우 running 종료.
8. throttlePWM값을 읽어와서 pumpDuty로 내보냄.
9. 온도가 750도씨 초과 시 ERROR_STOP
10. COOLING 시 5초 간격으로 1초씩 스타터 모터 1500us로 구동. 100도씨 이하로 떨어지면 IDLE 상태로 변경.

읽어오는 turbine RPM, Pump RPM등을 활용하여 아이들 RPM 유지 로직, 스로틀 반응 로직, 오버 스피드 방지 로직 등이 여기에 추가될 수 있음.
*/

#include <WiFi.h>
#include <ESPAsyncWebServer.h>
#include <AsyncTCP.h>
#include <SPI.h>
#include <SPIFFS.h>
#include <Adafruit_MAX31855.h>
#include <ESPmDNS.h>
#include <functional> // for std::bind

// --- Pin assignments ---
// Inputs
#define POWER_PWM_PIN 32      // RC Receiver Power/Switch Channel Input
#define THROTTLE_PWM_PIN 33   // RC Receiver Throttle Channel Input
#define PUMP_RPM_PIN 34       // Pump RPM Sensor Input (1 pulse/rev)
#define TURBINE_RPM_PIN 35    // Turbine RPM Sensor Input (1 pulse/rev)

// Outputs
#define PUMP_PWM_PIN 25       // Fuel Pump ESC/Valve Output
#define STARTER_PWM_PIN 26    // Starter Motor ESC Output
#define GLOW_PIN 27           // Glow Plug Output

// SPI for MAX31855 Temperature Sensor
#define MAX31855_CLK 18
#define MAX31855_CS 5
#define MAX31855_DO 19

// --- WiFi Configuration ---
const char* ssid = "afterburner";
const char* password = "12345678";

// --- Web Server & WebSocket ---
AsyncWebServer server(80);
AsyncWebSocket ws("/ws");

// --- Engine State Machine ---
enum EngineState {
  IDLE, ARMED, GLOW_ON, STARTER_ON, FUEL_RAMP, IGNITION_CHECK, RUNNING, COOLING, ERROR_STOP
};
EngineState engineState = IDLE;
unsigned long stateStartTime = 0;

// --- Control Variables ---
int glowState = 0;
int starterPWM = 1000;
int pumpDuty = 1000;
int powerPWM = 1000;
int throttlePWM = 1000;
bool powerSwitch = false;
bool emergencyStop = false;
const int targetStarterPWM = 1100;
const int coolStarterPWM = 1100;

// --- Flag for Error Reset Logic ---
bool errorAcknowledgedPowerLow = false; // Global flag
bool errorResetPowerHighSeen = false; // *** NEW FLAG ***

// --- Sensor & Timing Variables ---
float temperature = 0.0;
Adafruit_MAX31855 thermocouple(MAX31855_CLK, MAX31855_CS, MAX31855_DO);
unsigned long lastTempReadTime = 0;
const unsigned long TEMP_READ_INTERVAL = 250;

unsigned long lastWsUpdateTime = 0;
const unsigned long WS_UPDATE_INTERVAL = 500;

unsigned long lastFsmUpdateTime = 0;
const unsigned long FSM_UPDATE_INTERVAL = 50;

// --- PWM Input (Interrupt based) ---
volatile unsigned long powerPulseStart = 0;
volatile int latestPowerPWM = 1000;
portMUX_TYPE powerMux = portMUX_INITIALIZER_UNLOCKED;

volatile unsigned long throttlePulseStart = 0;
volatile int latestThrottlePWM = 1000;
portMUX_TYPE throttleMux = portMUX_INITIALIZER_UNLOCKED;

// --- NEW: RPM Measurement Variables ---
// Pump RPM
portMUX_TYPE pumpRpmMux = portMUX_INITIALIZER_UNLOCKED;
volatile unsigned long pumpLastPulseTimeMicros = 0;
volatile unsigned long pumpPulsePeriodMicros = 0;
volatile uint32_t pumpPulseCounter = 0; // For alternative calculation if needed
float pumpRPM = 0.0;
const unsigned long RPM_TIMEOUT_MS = 1000; // Timeout to consider RPM as 0

// Turbine RPM
portMUX_TYPE turbineRpmMux = portMUX_INITIALIZER_UNLOCKED;
volatile unsigned long turbineLastPulseTimeMicros = 0;
volatile unsigned long turbinePulsePeriodMicros = 0;
volatile uint32_t turbinePulseCounter = 0; // For alternative calculation if needed
float turbineRPM = 0.0;

unsigned long lastRpmCalcTime = 0;
const unsigned long RPM_CALC_INTERVAL = 100; // Calculate RPM every 100ms


// --- PWM Input ISRs ---
void IRAM_ATTR isrPowerPWM() {
  portENTER_CRITICAL_ISR(&powerMux);
  if (digitalRead(POWER_PWM_PIN) == HIGH) {
    powerPulseStart = micros();
  } else {
    if (powerPulseStart > 0) {
      latestPowerPWM = (int)(micros() - powerPulseStart);
      powerPulseStart = 0;
      if (latestPowerPWM < 800 || latestPowerPWM > 2200) latestPowerPWM = 1000;
    }
  }
  portEXIT_CRITICAL_ISR(&powerMux);
}

void IRAM_ATTR isrThrottlePWM() {
  portENTER_CRITICAL_ISR(&throttleMux);
  if (digitalRead(THROTTLE_PWM_PIN) == HIGH) {
    throttlePulseStart = micros();
  } else {
    if (throttlePulseStart > 0) {
      latestThrottlePWM = (int)(micros() - throttlePulseStart);
      throttlePulseStart = 0;
      if (latestThrottlePWM < 800 || latestThrottlePWM > 2200) latestThrottlePWM = 1000;
    }
  }
  portEXIT_CRITICAL_ISR(&throttleMux);
}

// --- NEW: RPM Input Interrupt Service Routines ---
void IRAM_ATTR isrPumpRPM() {
  portENTER_CRITICAL_ISR(&pumpRpmMux);
  unsigned long nowMicros = micros();
  if (pumpLastPulseTimeMicros > 0) { // Avoid first pulse calculation error
    pumpPulsePeriodMicros = nowMicros - pumpLastPulseTimeMicros;
  }
  pumpLastPulseTimeMicros = nowMicros;
  pumpPulseCounter++; // Increment counter
  portEXIT_CRITICAL_ISR(&pumpRpmMux);
}

void IRAM_ATTR isrTurbineRPM() {
  portENTER_CRITICAL_ISR(&turbineRpmMux);
  unsigned long nowMicros = micros();
  if (turbineLastPulseTimeMicros > 0) {
    turbinePulsePeriodMicros = nowMicros - turbineLastPulseTimeMicros;
  }
  turbineLastPulseTimeMicros = nowMicros;
  turbinePulseCounter++; // Increment counter
  portEXIT_CRITICAL_ISR(&turbineRpmMux);
}


// --- Helper Functions ---
void writePWM_US(int channel, int us); // Prototype needed

void setState(EngineState newState) {
  if (engineState != newState) {
    Serial.printf("State Change: %d -> %d\n", engineState, newState);
    engineState = newState;
    stateStartTime = millis();
    if (newState == IDLE || newState == ERROR_STOP) {
      digitalWrite(GLOW_PIN, LOW); glowState = 0;
      starterPWM = 1000;
      pumpDuty = 1000;
      powerSwitch = false;
      errorAcknowledgedPowerLow = false;
      errorResetPowerHighSeen = false;
    }
    if (newState == COOLING) {
      pumpDuty = 1000;
      digitalWrite(GLOW_PIN, LOW); glowState = 0;
    }
  }
}

// --- MODIFIED: notifyClients() ---
void notifyClients() {
  String json = "{";
  json += "\"state\": " + String(engineState) + ",";
  json += "\"glow\": \"" + String(glowState ? "ON" : "OFF") + "\",";
  json += "\"starter\": " + String(starterPWM) + ",";
  json += "\"pump\": " + String(pumpDuty) + ",";
  json += "\"temp\": " + String(isnan(temperature) ? "\"Error\"" : String(temperature, 1)) + ",";
  json += "\"power\": " + String(powerPWM) + ",";
  json += "\"throttle\": " + String(throttlePWM) + ",";
  // --- NEW: Add RPM data ---
  json += "\"pumpRPM\": " + String(pumpRPM, 0) + ","; // Format as integer
  json += "\"turbineRPM\": " + String(turbineRPM, 0);
  json += "}";
  ws.textAll(json);
}

// --- WebSocket Handler ---
void handleWebSocket(AsyncWebSocket *server, AsyncWebSocketClient *client, AwsEventType type, void *arg, uint8_t *data, size_t len) {
  if (type == WS_EVT_CONNECT) {
    Serial.printf("WebSocket client #%u connected from %s\n", client->id(), client->remoteIP().toString().c_str());
    notifyClients();
  } else if (type == WS_EVT_DISCONNECT) {
    Serial.printf("WebSocket client #%u disconnected\n", client->id());
  } else if (type == WS_EVT_DATA) {
    AwsFrameInfo *info = (AwsFrameInfo*)arg;
    if (info->final && info->index == 0 && info->len == len && info->opcode == WS_TEXT) {
      data[len] = 0;
      String msg = (char*)data;
      Serial.printf("WS Received from #%u: %s\n", client->id(), msg.c_str());

      if (msg == "power:1") { powerSwitch = true; Serial.println("Software Power ON"); }
      else if (msg == "power:0") {
        powerSwitch = false; Serial.println("Software Power OFF");
        if (engineState != RUNNING && engineState != COOLING) setState(IDLE);
        else if (engineState == RUNNING) setState(COOLING);
      }
      else if (msg == "emergency") { emergencyStop = true; setState(ERROR_STOP); Serial.println("EMERGENCY STOP Received"); }
      else if (msg == "glow:1") {
        if (engineState == IDLE || engineState == ARMED) { digitalWrite(GLOW_PIN, HIGH); glowState = 1; Serial.println("Manual Glow ON"); }
        else Serial.println("Manual Glow denied: Not in IDLE or ARMED state.");
      }
      else if (msg == "glow:0") { digitalWrite(GLOW_PIN, LOW); glowState = 0; Serial.println("Manual Glow OFF"); }
      else if (msg.startsWith("starter:")) {
        if (engineState == IDLE || engineState == ARMED) {
          int val = msg.substring(8).toInt(); starterPWM = constrain(val, 1000, 2000); Serial.printf("Manual Starter PWM: %d\n", starterPWM);
        } else Serial.println("Manual Starter denied: Not in IDLE or ARMED state.");
      }
      else if (msg.startsWith("pump:")) {
        if (engineState == IDLE || engineState == ARMED) {
          int val = msg.substring(5).toInt(); pumpDuty = constrain(val, 1000, 2000); Serial.printf("Manual Pump Duty: %d\n", pumpDuty);
        } else Serial.println("Manual Pump denied: Not in IDLE or ARMED state.");
      }
      notifyClients(); // Update UI immediately after command
    }
  } else if (type == WS_EVT_ERROR) {
    Serial.printf("WebSocket client #%u error #%u: %s\n", client->id(), *((uint16_t*)arg), (char*)data);
  }
}

// --- WiFi Setup ---
void setupWiFi() {
  WiFi.softAP(ssid, password);
  Serial.print("AP Started. SSID: "); Serial.print(ssid); Serial.print(" IP: "); Serial.println(WiFi.softAPIP());
  if (MDNS.begin("afterburner")) {
    Serial.println("mDNS responder started: http://afterburner.local");
    MDNS.addService("http", "tcp", 80);
  } else Serial.println("Error setting up MDNS responder!");
}

// --- PWM Setup ---
void setupPWM() {
  ledcSetup(0, 50, 16); ledcAttachPin(PUMP_PWM_PIN, 0);
  ledcSetup(1, 50, 16); ledcAttachPin(STARTER_PWM_PIN, 1);
  writePWM_US(0, 1000); writePWM_US(1, 1000);
}

// --- PWM Write Helper ---
void writePWM_US(int channel, int us) {
  us = constrain(us, 1000, 2000);
  int duty = map(us, 1000, 2000, 3277, 6553);
  ledcWrite(channel, duty);
}

// --- Finite State Machine (FSM) Logic ---
// Note: This FSM doesn't use the RPM values yet. Needs modification.
bool rampingStarter = false;
unsigned long starterRampStartTime = 0;
unsigned long lastCoolPulseTime = 0;
bool coolPulseActive = false;

void handleFSM() {
  unsigned long now = millis();
  unsigned long stateElapsedTime = now - stateStartTime;

  if (emergencyStop) { setState(ERROR_STOP); return; }

  switch (engineState) {
    case IDLE:
      errorAcknowledgedPowerLow = false;
      errorResetPowerHighSeen = false;
      if ((powerPWM > 1800 || powerSwitch) && !emergencyStop) setState(ARMED);
      writePWM_US(0, 1000); writePWM_US(1, 1000);
      break;

    case ARMED:
      Serial.println("ARMED: Turning Glow ON");
      digitalWrite(GLOW_PIN, HIGH); glowState = 1;
      setState(GLOW_ON);
      break;

    case GLOW_ON:
      if (stateElapsedTime > 3000) {
        Serial.println("GLOW_ON: Starting Starter Ramp");
        starterRampStartTime = now; rampingStarter = true; starterPWM = 1000;
        setState(STARTER_ON);
      }
      if (powerPWM < 1300 && !powerSwitch) { Serial.println("GLOW_ON: Power lost, returning to IDLE"); setState(IDLE); }
      break;

    case STARTER_ON:
      if (rampingStarter) {
        const unsigned long rampDuration = 2000; 
        if (now - starterRampStartTime < rampDuration) starterPWM = map(now - starterRampStartTime, 0, rampDuration, 1000, targetStarterPWM);
        else {
          starterPWM = targetStarterPWM; rampingStarter = false; Serial.println("STARTER_ON: Ramp complete"); stateStartTime = now;
        }
      } else {
        if (stateElapsedTime > 1000) { Serial.println("STARTER_ON: Introducing Fuel"); pumpDuty = 1100; setState(FUEL_RAMP); }
      }
      if (powerPWM < 1300 && !powerSwitch) { Serial.println("STARTER_ON: Power lost, going to COOLING"); setState(COOLING); }
      break;

    case FUEL_RAMP:
      // TODO: Use RPM data here (e.g., wait for turbine RPM > threshold before ignition check)
      if (temperature > 100.0) { Serial.println("FUEL_RAMP: Ignition detected?"); setState(IGNITION_CHECK); }
      if (stateElapsedTime > 8000) { Serial.println("FUEL_RAMP: Timeout - No Ignition"); setState(ERROR_STOP); }
      if (powerPWM < 1300 && !powerSwitch) { Serial.println("FUEL_RAMP: Power lost, going to COOLING"); setState(COOLING); }
      break;

    case IGNITION_CHECK:
      // TODO: Use RPM data here (e.g., check for self-sustaining RPM)
      if (temperature > 300.0) {
        Serial.println("IGNITION_CHECK: Confirmed! Turning off Glow & Starter");
        digitalWrite(GLOW_PIN, LOW); glowState = 0; starterPWM = 1000;
        setState(RUNNING);
      }
      if (stateElapsedTime > 5000) { Serial.println("IGNITION_CHECK: Failed - Temp dropped or timeout"); setState(COOLING); }
      if (powerPWM < 1300 && !powerSwitch) { Serial.println("IGNITION_CHECK: Power lost, going to COOLING"); setState(COOLING); }
      break;

    case RUNNING:
      // TODO: Add RPM-based idle control and throttle mapping
      if (powerPWM <= 1300 && !powerSwitch) { Serial.println("RUNNING: Shutdown requested"); setState(COOLING); }
      else {
        pumpDuty = map(throttlePWM, 1000, 2000, 1100, 1900); pumpDuty = constrain(pumpDuty, 1000, 2000);
      }
      if (temperature > 750.0) { Serial.println("RUNNING: Overtemperature detected!"); setState(ERROR_STOP); }
      // TODO: Add RPM overspeed check
      break;

    case COOLING:
      if (temperature <= 100.0 && stateElapsedTime > 10000) {
        Serial.println("COOLING: Cooldown complete"); starterPWM = 1000; coolPulseActive = false; setState(IDLE);
      } else {
        const unsigned long coolInterval = 5000; const unsigned long coolPulseDuration = 1000; 
        if (!coolPulseActive && (now - lastCoolPulseTime >= coolInterval)) {
          Serial.println("COOLING: Starting cool pulse"); starterPWM = coolStarterPWM; lastCoolPulseTime = now; coolPulseActive = true;
        } else if (coolPulseActive && (now - lastCoolPulseTime >= coolPulseDuration)) {
          Serial.println("COOLING: Ending cool pulse"); starterPWM = 1000; coolPulseActive = false; lastCoolPulseTime = now;
        } else if (!coolPulseActive) { starterPWM = 1000; }
      }
      break;

      case ERROR_STOP:
        // *** Reset flag explicitly when entering ERROR_STOP logic (first time) ***
        if (stateElapsedTime < FSM_UPDATE_INTERVAL * 2) { // Only on first couple of runs in this state
            errorAcknowledgedPowerLow = false; // Ensure it's reset when error first occurs
            Serial.println("Entered ERROR_STOP state. Awaiting power cycle.");
        }

        // Ensure outputs are off
        writePWM_US(0, 1000); writePWM_US(1, 1000);

        // --- Automatic Reset Logic (Low -> High -> Low) ---

        // Step 1: Wait for power low (if not already acknowledged)
        if (!errorAcknowledgedPowerLow) {
          if (powerPWM < 1300 && !powerSwitch) {
              Serial.println("ERROR_STOP: Step 1/3 - Power signal low detected.");
              errorAcknowledgedPowerLow = true;
              errorResetPowerHighSeen = false; // Ensure high flag is reset when power goes low
          }
      }
      // Step 2: Wait for power high (if low was seen, but high not yet)
      else if (errorAcknowledgedPowerLow && !errorResetPowerHighSeen) {
          if (powerPWM > 1800 || powerSwitch) {
              Serial.println("ERROR_STOP: Step 2/3 - Power signal high seen after low.");
              errorResetPowerHighSeen = true; // Mark that high signal was seen
          }
          // Check if power went low again *before* going high (user flicked off then on quickly)
          else if (powerPWM < 1300 && !powerSwitch) {
                // Stay in Step 1 acknowledgement, but reset HighSeen flag if it somehow got set
                errorResetPowerHighSeen = false;
          }
      }
      // Step 3: Wait for power low AGAIN (if both low and high were seen)
      else if (errorAcknowledgedPowerLow && errorResetPowerHighSeen) {
          if (powerPWM < 1300 && !powerSwitch) {
              Serial.println("ERROR_STOP: Step 3/3 - Power signal low again detected. Resetting to IDLE.");
              setState(IDLE); // <<< RESET TO IDLE HERE!
              // Flags will be reset by setState(IDLE)
          }
          // Check if power went back high again *after* the high was seen (user flicked on then off quickly)
          else if (powerPWM > 1800 || powerSwitch) {
                // Stay in Step 2 acknowledgement, wait for low again
          }
      }
      break; // Stay in ERROR_STOP until sequence Low -> High -> Low is complete
  }
}

// --- Arduino Setup Function ---
void setup() {
  Serial.begin(115200);
  Serial.println("\nROTOM Turbine Controller v0.2 (RPM Enabled) Booting...");
  errorAcknowledgedPowerLow = false; // Initialize global flag
  errorResetPowerHighSeen = false;

  // Initialize GPIOs
  pinMode(GLOW_PIN, OUTPUT); digitalWrite(GLOW_PIN, LOW);
  pinMode(POWER_PWM_PIN, INPUT_PULLUP); pinMode(THROTTLE_PWM_PIN, INPUT_PULLUP);
  // --- NEW: Initialize RPM Input Pins ---
  pinMode(PUMP_RPM_PIN, INPUT); // Assuming external pullup exists via level shifter
  pinMode(TURBINE_RPM_PIN, INPUT); // Assuming external pullup exists (R16)

  // Initialize SPIFFS
  if (!SPIFFS.begin(true)) { Serial.println("SPIFFS Mount Failed!"); while(1) delay(1000); }
  Serial.println("SPIFFS Mounted.");
  File root = SPIFFS.open("/"); File file = root.openNextFile();
  while(file){ Serial.print("  FILE: "); Serial.println(file.name()); file = root.openNextFile(); } root.close();

  // Initialize Temperature Sensor
  Serial.print("Initializing MAX31855...");
  if (!thermocouple.begin()) Serial.println(" ERROR!");
  else {
    Serial.println(" OK."); temperature = thermocouple.readCelsius();
    if (isnan(temperature)) Serial.println("Failed to read initial temperature.");
    else { Serial.print("Initial Temperature: "); Serial.println(temperature); }
  }

  // Setup WiFi and mDNS
  setupWiFi();
  // Setup PWM outputs
  setupPWM();
  // Setup WebSocket handler
  ws.onEvent(handleWebSocket); server.addHandler(&ws);
  // Serve static files
  server.serveStatic("/", SPIFFS, "/").setDefaultFile("index.html");
  // Handle Not Found
  server.onNotFound([](AsyncWebServerRequest *request){ request->send(404, "text/plain", "Not found"); });
  // Start server
  server.begin(); Serial.println("Web Server Started.");

  // Attach PWM Input Interrupts
  attachInterrupt(digitalPinToInterrupt(POWER_PWM_PIN), isrPowerPWM, CHANGE);
  attachInterrupt(digitalPinToInterrupt(THROTTLE_PWM_PIN), isrThrottlePWM, CHANGE);
  Serial.println("PWM Input Interrupts Attached.");

  // --- NEW: Attach RPM Input Interrupts ---
  // Use RISING or FALLING depending on your sensor signal and schematic
  attachInterrupt(digitalPinToInterrupt(PUMP_RPM_PIN), isrPumpRPM, RISING);
  attachInterrupt(digitalPinToInterrupt(TURBINE_RPM_PIN), isrTurbineRPM, RISING);
  Serial.println("RPM Input Interrupts Attached.");

  // Initialize timers
  stateStartTime = millis(); lastFsmUpdateTime = millis(); lastWsUpdateTime = millis(); lastTempReadTime = millis();
  lastRpmCalcTime = millis(); // Initialize RPM calculation timer

  Serial.println("Setup Complete. Entering Main Loop.");
}

// --- Arduino Loop Function ---
void loop() {
  unsigned long currentTime = millis();
  unsigned long currentMicros = micros();

  // --- Read Inputs ---
  portENTER_CRITICAL(&powerMux); powerPWM = latestPowerPWM; portEXIT_CRITICAL(&powerMux);
  portENTER_CRITICAL(&throttleMux); throttlePWM = latestThrottlePWM; portEXIT_CRITICAL(&throttleMux);

  // Read Temperature
  if (currentTime - lastTempReadTime >= TEMP_READ_INTERVAL) {
    lastTempReadTime = currentTime; float newTemp = thermocouple.readCelsius();
    if (!isnan(newTemp)) temperature = newTemp;
    else Serial.println("Warning: Failed to read temperature!");
    uint8_t fault = thermocouple.readError();
    if (fault) { Serial.print("Thermocouple Fault: 0x"); Serial.println(fault, HEX); /* emergencyStop = true; */ }
  }

  // --- NEW: Calculate RPM ---
  if (currentTime - lastRpmCalcTime >= RPM_CALC_INTERVAL) {
    lastRpmCalcTime = currentTime;

    // Pump RPM
    unsigned long pumpPeriod = 0, pumpLastSeen = 0;
    portENTER_CRITICAL(&pumpRpmMux); pumpPeriod = pumpPulsePeriodMicros; pumpLastSeen = pumpLastPulseTimeMicros; portEXIT_CRITICAL(&pumpRpmMux);
    if ((currentMicros - pumpLastSeen) > (RPM_TIMEOUT_MS * 1000UL)) pumpRPM = 0.0;
    else if (pumpPeriod > 0) pumpRPM = 60000000.0f / pumpPeriod;
    // else pumpRPM = 0.0; // Keep last value if period is 0? Or set to 0?

    // Turbine RPM
    unsigned long turbinePeriod = 0, turbineLastSeen = 0;
    portENTER_CRITICAL(&turbineRpmMux); turbinePeriod = turbinePulsePeriodMicros; turbineLastSeen = turbineLastPulseTimeMicros; portEXIT_CRITICAL(&turbineRpmMux);
    if ((currentMicros - turbineLastSeen) > (RPM_TIMEOUT_MS * 1000UL)) turbineRPM = 0.0;
    else if (turbinePeriod > 0) turbineRPM = 60000000.0f / turbinePeriod;
    // else turbineRPM = 0.0;
  }

  // --- Run Control Logic ---
  if (currentTime - lastFsmUpdateTime >= FSM_UPDATE_INTERVAL) {
    lastFsmUpdateTime = currentTime;
    handleFSM(); // Consider passing RPM: handleFSM(pumpRPM, turbineRPM);
  }

  // --- Update Outputs ---
  writePWM_US(0, pumpDuty); writePWM_US(1, starterPWM);

  // --- Communications ---
  if (currentTime - lastWsUpdateTime >= WS_UPDATE_INTERVAL) {
    lastWsUpdateTime = currentTime;
    notifyClients();
  }
  ws.cleanupClients();
}