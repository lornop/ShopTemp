/*********************************************************************
   SHOP ESP32 CONTROL SYSTEM
   ------------------------------------------------------------
   - Reads DHT22 temperature & humidity sensor
   - Publishes data via MQTT (Adafruit library)
   - Displays status on I2C LCD
   - Controls heater and fan via GPIO outputs
   - Timer function (1-hour increments) with push button
   - Dual-core operation using FreeRTOS
   - Handles WiFi + MQTT reconnections robustly
   - Includes GPIO FAILSAFE watchdog (heater/fan off if timer expired)
*********************************************************************/

#include <DHT.h>
#include <WiFi.h>
#include <WiFiClient.h>
#include "Adafruit_MQTT.h"
#include "Adafruit_MQTT_Client.h"
#include <Arduino.h>
#include <LiquidCrystal_I2C.h>
#include <Wire.h>

/************************* WIFI + MQTT CONFIG *************************/
#include "../secrets.h"   // This now provides all credentials

//#define WLAN_SSID ""
//#define WLAN_PASS ""
//#define MQTT_USER ""
//#define MQTT_KEY  ""
//#define MQTT_HOST ""

#define MQTT_PORT 1883
String newHostname = "SHOPESP32";

/************************* MQTT TOPICS *************************/
#define MQTT_PUB_TEMP "esp/dht/temperature"
#define MQTT_PUB_HUM  "esp/dht/humidity"
#define MQTT_PUB_TIME "esp/set/timeleft"
#define MQTT_SUB_TEMP "esp/set/settemperature"
#define MQTT_SUB_TIME "esp/set/settime"

/************************* DHT CONFIG *************************/
#define DHTPIN 32
#define DHTTYPE DHT22
DHT dht(DHTPIN, DHTTYPE);

/************************* LCD CONFIG *************************/
#define SCREENI2CADDRESS 0x27
// LiquidCrystalI2C_RS_EN(lcd, SCREENI2CADDRESS, false)
LiquidCrystal_I2C lcd(SCREENI2CADDRESS, 16, 2);

/************************* HARDWARE PINS *************************/
const int fan_out     = 33;
const int heater_out  = 25;
const int button_in   = 4;

/************************* WIFI + MQTT OBJECTS *************************/
WiFiClient client;
Adafruit_MQTT_Client mqtt(&client, MQTT_HOST, MQTT_PORT, MQTT_USER, MQTT_KEY);
Adafruit_MQTT_Publish temperature(&mqtt, MQTT_PUB_TEMP);
Adafruit_MQTT_Publish humidity(&mqtt, MQTT_PUB_HUM);
Adafruit_MQTT_Publish timeleft_pub(&mqtt, MQTT_PUB_TIME);
Adafruit_MQTT_Subscribe settemperature(&mqtt, MQTT_SUB_TEMP);
Adafruit_MQTT_Subscribe settime(&mqtt, MQTT_SUB_TIME);

/************************* GLOBAL VARIABLES *************************/
float temp, hum;             // Sensor readings
float on_temp = 18.5;        // Heater ON threshold
float on_hum = 80.0;         // Humidity threshold
float time_left;             // Time left (hours)
uint32_t TIMER_MILLIS = 0;   // Countdown timer
uint32_t time_previous = 0;  // Last LCD update time
char timer_lcd[6];           // Time text buffer

uint32_t previousMillis = 0;
uint32_t LCD_UpdateInterval = 1000;  // 1 second LCD update
uint32_t intervalMillis = 60000;     // 1 minute sensor update

TaskHandle_t Core1;
TaskHandle_t Core2;

SemaphoreHandle_t dhtMutex;   //Creates and Mutual Exclusion lock for the MQTT variables (temp and hum) so theres no race conditions



/**************************** WIFI  VARIABLES ****************************/

// === CONFIG ===
const unsigned long WIFI_FAIL_REBOOT_TIME = 6UL * 60UL * 60UL * 1000UL; // 6 hours
const unsigned long WIFI_RETRY_INTERVAL   = 30000;  // 30 sec between reconnect attempts
const unsigned long WIFI_CONNECT_TIMEOUT  = 10000;  // 10 sec allowed per attempt

// === STATE TRACKING ===
unsigned long lastWiFiSuccess = 0;
unsigned long lastWiFiAttempt = 0;
bool wifiConnecting = false;
unsigned long connectStartTime = 0;

/************************* LCD CUSTOM CHARACTERS *************************/
byte full_box[8] = {0b11111,0b11111,0b11111,0b11111,0b11111,0b11111,0b11111,0b11111};

/************************* FUNCTION PROTOTYPES *************************/
void ensureWiFiConnected();
void ensureMQTTConnected();
void coreOne(void * pvParameters);
void coreTwo(void * pvParameters);
void GET_DHT();
void GET_MQTT();
void PUBLISH_TEMP_HUM();
void FAN_HEAT_CONTROL();
void FAN_HEAT_FAILSAFE();
void PRINT_LCD();
void TIMER_ON_BUTTON();
void timer();
void get_minutes(float, char[]);

/*********************************************************************
   SETUP
*********************************************************************/
void setup() {
  Serial.begin(115200);
  Wire.begin();

  // Initialize LCD
  lcd.begin(16, 2);
  lcd.backlight();    // turn on backlight
  lcd.backlight();
  lcd.createChar(0, full_box);
  lcd.print("POWER ON");
  delay(1000);
  lcd.clear();

  // Initialize DHT sensor
  dht.begin();
  dhtMutex = xSemaphoreCreateMutex();  // Initialize the mutex



  // Initialize I/O pins
  pinMode(heater_out, OUTPUT);
  pinMode(fan_out, OUTPUT);
  pinMode(button_in, INPUT_PULLUP);

  // Power-on failsafe: make sure outputs are OFF
  digitalWrite(heater_out, LOW);
  digitalWrite(fan_out, LOW);
  delay(100);
  if (digitalRead(heater_out) != LOW || digitalRead(fan_out) != LOW) {
    Serial.println("[FAILSAFE] GPIOs not low at boot — forcing low");
    digitalWrite(heater_out, LOW);
    digitalWrite(fan_out, LOW);
  }

  // Button interrupt: add 1 hour when pressed
  attachInterrupt(digitalPinToInterrupt(button_in), TIMER_ON_BUTTON, HIGH);

  // Start WiFi connection
  WiFi.hostname(newHostname.c_str());
  ensureWiFiConnected();
  lastWiFiSuccess = millis();

  // Subscribe to MQTT topics
  mqtt.subscribe(&settemperature);
  mqtt.subscribe(&settime);

  // Launch FreeRTOS tasks on both cores
  xTaskCreatePinnedToCore(coreOne, "Core1", 6000, NULL, 1, &Core1, 0);
  xTaskCreatePinnedToCore(coreTwo, "Core2", 6000, NULL, 2, &Core2, 1);

  lcd.clear();
  lcd.print("Setup complete");
  delay(1000);
  lcd.clear();
}

/*********************************************************************
   LOOP (empty - handled by FreeRTOS)
*********************************************************************/
void loop() {
  // Tasks run independently on each core
}

/*********************************************************************
   CORE 1 TASK (MQTT + WIFI MANAGEMENT)
*********************************************************************/
void coreOne(void * pvParameters) {
  delay(5000);
  uint32_t previousMillisCore1 = millis();   // Timer for 60s publish interval
  const uint32_t intervalCore1 = 60000;      // 60 seconds

  while (1) {
    ensureWiFiConnected();
    ensureMQTTConnected();

    if (mqtt.connected()) {
      // Non-blocking publish every 60s
      if (millis() - previousMillisCore1 >= intervalCore1) {
        GET_MQTT();
        PUBLISH_TEMP_HUM();
        previousMillisCore1 = millis();
      }
    }

    vTaskDelay(100 / portTICK_PERIOD_MS); // Small delay
  }
}

/*********************************************************************
   CORE 2 TASK (SENSOR + LCD DISPLAY + FAILSAFE)
*********************************************************************/
void coreTwo(void * pvParameters) {
  delay(2100);
  uint32_t previousMillisCore2 = millis();       // Timer for 1-minute DHT read
  uint32_t previousLCDMillis = millis();         // Timer for 1-second LCD + failsafe
  const uint32_t intervalCore2 = 60000;          // 1 minute

  while (1) {
    uint32_t now = millis();

    // Every minute: read DHT + control fan/heater
    if (now - previousMillisCore2 >= intervalCore2) {
      GET_DHT();
      FAN_HEAT_CONTROL();
      previousMillisCore2 = now;
    }

    // Every second: update timer, LCD, and failsafe
    if (now - previousLCDMillis >= LCD_UpdateInterval) {
      timer();
      FAN_HEAT_FAILSAFE();
      PRINT_LCD();
      previousLCDMillis = now;
    }

    vTaskDelay(50 / portTICK_PERIOD_MS); // Small delay
  }
}
/*********************************************************************
   WIFI HANDLER (AUTO-RECONNECT + AUTO-REBOOT)
*********************************************************************/
void ensureWiFiConnected() {
  // --- 1. If connected, record success and reset state ---
  if (WiFi.status() == WL_CONNECTED) {
    lastWiFiSuccess = millis();
    wifiConnecting = false;
    return;
  }

  unsigned long now = millis();

  // --- 2. If not currently connecting, start a new attempt every WIFI_RETRY_INTERVAL ---
  if (!wifiConnecting && now - lastWiFiAttempt >= WIFI_RETRY_INTERVAL) {
    Serial.println("WiFi lost — reconnecting...");
    WiFi.disconnect(true);
    WiFi.begin(WLAN_SSID, WLAN_PASS);

    wifiConnecting = true;
    connectStartTime = now;
    lastWiFiAttempt = now;
  }

  // --- 3. If we're currently trying to connect, check timeout ---
  if (wifiConnecting) {
    if (WiFi.status() == WL_CONNECTED) {
      Serial.println("[OK] WiFi reconnected");
      Serial.print("IP: ");
      Serial.println(WiFi.localIP());
      lastWiFiSuccess = now;
      wifiConnecting = false;
    }
    else if (now - connectStartTime > WIFI_CONNECT_TIMEOUT) {
      Serial.println("[WARN] WiFi reconnect attempt timed out");
      wifiConnecting = false; // stop this attempt; will try again later
    }
  }

  // --- 4. If Wi-Fi has been down too long, reboot ---
  if (now - lastWiFiSuccess > WIFI_FAIL_REBOOT_TIME) {
    Serial.println("[REBOOT] WiFi down too long — restarting...");
    ESP.restart();
  }
}

/*********************************************************************
   MQTT HANDLER (RECONNECT LOGIC)
*********************************************************************/
void ensureMQTTConnected() {
  if (mqtt.connected()) return;
  if (WiFi.status() != WL_CONNECTED) return;

  Serial.println("Connecting to MQTT...");
  int8_t ret;
  uint8_t retries = 3;

  while ((ret = mqtt.connect()) != 0 && retries--) {
    Serial.print("[WARN] MQTT connect failed: ");
    Serial.println(mqtt.connectErrorString(ret));
    delay(2000);
  }

  if (mqtt.connected()) {
    Serial.println("[OK] MQTT connected");
  } else {
    Serial.println("[WARN] MQTT reconnect failed");
  }
}

/*********************************************************************
   DHT SENSOR READ
*********************************************************************/
void GET_DHT() {
  float humSum = 0;
  float tempSum = 0;
  int validHumCount = 0;
  int validTempCount = 0;

  if (xSemaphoreTake(dhtMutex, (TickType_t)10) == pdTRUE) {   //Get Exclusive access to dht Variables
    for (int16_t x = 0; x < 5; x++) {
      float h = dht.readHumidity();
      float t = dht.readTemperature();

      if (!isnan(h)) {
        humSum += h;
        validHumCount++;
      }

      if (!isnan(t)) {
        tempSum += t;
        validTempCount++;
      }

      delay(10);
    }

    if (validHumCount > 0)
      hum = humSum / validHumCount;
    else
      hum = hum;  // Just send the last value

    if (validTempCount > 0)
      temp = tempSum / validTempCount;
    else
      temp = temp;
  
    xSemaphoreGive(dhtMutex);     //Release access to DHT Variables
  }
  
}


/*********************************************************************
   MQTT SUBSCRIPTION READ (REMOTE CONTROL)
*********************************************************************/
void GET_MQTT() {
  Adafruit_MQTT_Subscribe *subscription;
  while ((subscription = mqtt.readSubscription(500))) {
    if (subscription == &settemperature)
      on_temp = atof((char *)settemperature.lastread);

    if (subscription == &settime) {
      int mqtt_time = atoi((char *)settime.lastread);
      if (mqtt_time == 1) TIMER_ON_BUTTON();
      if (mqtt_time == 0) TIMER_MILLIS = 1000 * 60;
    }
  }
}

/*********************************************************************
   MQTT PUBLISH (SEND SENSOR DATA)
*********************************************************************/
void PUBLISH_TEMP_HUM() {
  float t;
  float h;

  if (!mqtt.connected()) return;
  if (xSemaphoreTake(dhtMutex, (TickType_t)100) == pdTRUE) {
    t = temp;
    h = hum;
    xSemaphoreGive(dhtMutex);
  }

  temperature.publish(t);
  humidity.publish(h);
  timeleft_pub.publish(time_left);
  Serial.printf("[PUB] Temp: %.1f C, Hum: %.1f%%, Time Left: %.2f h\n", t, h, time_left);

}

/*********************************************************************
   FAN + HEATER CONTROL LOGIC
*********************************************************************/
void FAN_HEAT_CONTROL() {
  if (TIMER_MILLIS > 1) {
    // Turn on heater if below target temp OR humidity too high
    if ((temp < on_temp) || (hum > on_hum))
      digitalWrite(heater_out, HIGH);
    else
      digitalWrite(heater_out, LOW);
  } else {
    // Timer expired, shut off everything
    digitalWrite(fan_out, LOW);
    digitalWrite(heater_out, LOW);
  }
}

/*********************************************************************
   FAILSAFE WATCHDOG
   ------------------------------------------------------------
   Runs every second.
   Ensures outputs are OFF if timer expired, overflows, or DHT fails.
*********************************************************************/
void FAN_HEAT_FAILSAFE() {
  // If timer expired or overflowed
  if (TIMER_MILLIS < 1 || TIMER_MILLIS > 4000000000) {
    if (digitalRead(heater_out) == HIGH || digitalRead(fan_out) == HIGH) {
      Serial.println("[FAILSAFE] Timer expired — forcing outputs OFF");
      digitalWrite(heater_out, LOW);
      digitalWrite(fan_out, LOW);
    }
  }

  // Sensor sanity check: NaN means DHT read failed
  if (isnan(temp) || isnan(hum)) {
    Serial.println("[FAILSAFE] Invalid DHT reading — shutting down outputs");
    digitalWrite(heater_out, LOW);
    digitalWrite(fan_out, LOW);
  }
}

/*********************************************************************
   TIMER LOGIC (COUNTDOWN)
*********************************************************************/
void timer() {
  uint32_t now = millis();
  TIMER_MILLIS -= (now - time_previous);
  time_previous = now;
  if (TIMER_MILLIS < 1 || TIMER_MILLIS > 4000000000) TIMER_MILLIS = 0;
  time_left = TIMER_MILLIS / (1000 * 60 * 60.0);
  get_minutes(time_left, timer_lcd);
  digitalWrite(fan_out, TIMER_MILLIS > 0);
}

/*********************************************************************
   FORMAT TIMER AS HH:MM
*********************************************************************/
void get_minutes(float timeleft, char timer_lcd[]) {
  int hours = (int)timeleft;
  int minutes = (int)((timeleft - hours) * 60);
  sprintf(timer_lcd, "%02d:%02d", hours, minutes);
}

/*********************************************************************
   BUTTON INTERRUPT (ADD 1 HOUR TO TIMER)
*********************************************************************/
ICACHE_RAM_ATTR void TIMER_ON_BUTTON() {
  static unsigned long last_int_time = 0;
  unsigned long int_time = millis();
  const unsigned long one_hour = 60 * 60 * 1000;
  const unsigned long max_time = 5 * one_hour;

  if (int_time - last_int_time > 1000) {
    if (TIMER_MILLIS < max_time)
      TIMER_MILLIS += one_hour;
    last_int_time = int_time;
  }
}

/*********************************************************************
   LCD DISPLAY
*********************************************************************/
void PRINT_LCD() {
  lcd.setCursor(0, 0);
  lcd.print(temp, 1);
  lcd.print((char)223); // degree symbol
  lcd.print("C ");
  lcd.write((byte)0);   // solid block
  lcd.print(" ");
  lcd.print(hum, 1);
  lcd.print("%");

  lcd.setCursor(0, 1);
  lcd.print("Timer: ");
  lcd.print(timer_lcd);
}
