/**
 * ═══════════════════════════════════════════════════════════
 *  Terry Car — 煞車距離測試
 *  ESP32 + TB6612 + Encoder(PPR400) + OLED(0.96") + HC-SR04
 * ═══════════════════════════════════════════════════════════
 *  測試流程：
 *    1. 按下按鈕 → 車子以最大速度啟動
 *    2. 超音波偵測到障礙物 ≤ 煞車觸發距離 → 斷電煞車
 *    3. 偵測車輪靜止（編碼器停止計數）
 *    4. OLED 顯示完整測試結果
 *    5. 再按一次按鈕 → 重置
 *
 *  腳位對照表
 *  ┌──────────┬──────────┬───────┐
 *  │ 元件     │ 信號     │ GPIO  │
 *  ├──────────┼──────────┼───────┤
 *  │ OLED     │ SDA      │  4    │  Wire1
 *  │ OLED     │ SCL      │ 15    │  Wire1
 *  │ TB6612   │ AIN1     │ 25    │
 *  │ TB6612   │ AIN2     │ 13    │
 *  │ TB6612   │ PWMA     │ 19    │
 *  │ TB6612   │ BIN1     │ 16    │
 *  │ TB6612   │ BIN2     │ 18    │
 *  │ TB6612   │ PWMB     │ 26    │
 *  │ TB6612   │ STBY     │ 23    │
 *  │ 編碼器A  │ E1A      │ 34    │  純輸入腳
 *  │ 編碼器B  │ E2A      │ 36    │  純輸入腳(VP)
 *  │ HC-SR04  │ TRIG     │ 27    │
 *  │ HC-SR04  │ ECHO     │ 32    │
 *  │ 按鈕     │ IN       │ 14    │
 *  │ 蜂鳴器   │ OUT      │ 12    │  被動式蜂鳴器
 *  └──────────┴──────────┴───────┘
 *
 *  所需函式庫（Arduino IDE 函式庫管理員安裝）：
 *    - Adafruit SSD1306
 *    - Adafruit GFX Library
 * ═══════════════════════════════════════════════════════════
 */

#include <WiFi.h>
#include <HTTPClient.h>
#include <PubSubClient.h>
#include <ArduinoJson.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

// ═══════════════════════════════════════════════════════════
//  【可調整的全域參數】— 依實際硬體修改這裡
// ═══════════════════════════════════════════════════════════

// ── WiFi 設定 ────────────────────────────────────────────────
const char* WIFI_SSID     = "webduino.io";
const char* WIFI_PASSWORD = "webduino01";
const char* GOOGLE_SCRIPT_URL = "https://script.google.com/macros/s/AKfycbyLbJMM8CIDVdwpZM2v6pxTtUHX-nIvFiKOgrPDUsqM6jBO9kF_YNRsGbzsmMpYhAjN/exec";

// ── MQTT 設定 ────────────────────────────────────────────────
const char* MQTT_BROKER   = "broker.MQTTGO.io";
const int   MQTT_PORT     = 1883;
const char* MQTT_CLIENT_ID = "TerryCar_ESP32";
const char* MQTT_TOPIC_CMD = "TerryCar/cmd";     // 接收命令（網頁發送）
const char* MQTT_TOPIC_STS = "TerryCar/status";  // 發送狀態（回傳給網頁）

// 可由 MQTT 遠端調整的參數
int remoteMotorSpeed      = 100;       // 馬達速度（可遠端修改）
float remoteBrakeTrigger  = 15.0;      // 煞車觸發距離（可遠端修改）

// ── 馬達速度（使用遠端可調參數）───────────────────────────────
const int    PWM_FREQUENCY     = 20000; // PWM 頻率 Hz（20kHz 馬達靜音）
const int    PWM_RESOLUTION   = 8;     // PWM 解析度 8-bit → 0~255

// ── 編碼器與距離換算 ────────────────────────────────────────
const int    ENCODER_PPR      = 400;   // PPR（每圈脈衝 Pulses Per Revolution）
const float  WHEEL_CIRCUMFERENCE = 21.23; // 輪胎周長（公分）

// ── 靜止判定 ────────────────────────────────────────────────
const int    STOP_TIMEOUT_MS  = 400;   // 編碼器停止計數超過此時間 → 視為車輛已靜止
const int    ULTRASONIC_INTERVAL_MS = 80; // 超音波讀取間隔
const int    SPEED_CALC_INTERVAL_MS = 100; // 速度計算間隔（毫秒）

// ── OLED 螢幕 ────────────────────────────────────────────────
const int    OLED_WIDTH       = 128;   // 0.96 吋 SSD1306 寬度（像素）
const int    OLED_HEIGHT      = 64;    // 0.96 吋 SSD1306 高度（像素）

// ═══════════════════════════════════════════════════════════
//  腳位定義（更改接線時修改這裡）
// ═══════════════════════════════════════════════════════════
const int    PIN_OLED_SDA     = 4;
const int    PIN_OLED_SCL     = 15;
const int    PIN_AIN1         = 25;
const int    PIN_AIN2         = 13;
const int    PIN_PWMA         = 19;
const int    PIN_BIN1         = 16;
const int    PIN_BIN2         = 18;
const int    PIN_PWMB         = 26;
const int    PIN_STBY         = 23;
const int    PIN_ENCODER_A    = 34;   // E1A（純輸入腳，支援中斷）
const int    PIN_ENCODER_B    = 36;   // E2A（純輸入腳 GPIO36=VP，支援中斷）
const int    PIN_ULTRASONIC_TRIG = 27;
const int    PIN_ULTRASONIC_ECHO = 32;
const int    PIN_BUTTON       = 14;
const int    PIN_BUZZER       = 12;

// ═══════════════════════════════════════════════════════════
//  由參數自動計算（請勿手動修改）
// ═══════════════════════════════════════════════════════════
const float  DIST_PER_PULSE   = WHEEL_CIRCUMFERENCE / (float)ENCODER_PPR;
// 每脈衝 = 21.23 / 400 = 0.053075 公分

// ═══════════════════════════════════════════════════════════
//  OLED 物件（Wire1 = 獨立 I2C 匯流排）
// ═══════════════════════════════════════════════════════════
Adafruit_SSD1306 oled(OLED_WIDTH, OLED_HEIGHT, &Wire1, -1);

// ═══════════════════════════════════════════════════════════
//  編碼器（ISR 修改，需 volatile）
// ═══════════════════════════════════════════════════════════
volatile long encoderA_Count = 0;
volatile long encoderB_Count = 0;

// ═══════════════════════════════════════════════════════════
//  測試狀態機
// ═══════════════════════════════════════════════════════════
enum TestState_t {
  IDLE,         // 等待按鈕
  RUNNING,     // 全速前進，監控超音波
  BRAKING,     // 斷電，等待車輛靜止
  COMPLETED,   // 顯示結果
  ABORTED      // 使用者手動中止
};
TestState_t currentState = IDLE;

// ═══════════════════════════════════════════════════════════
//  測試數據記錄
// ═══════════════════════════════════════════════════════════
unsigned long startTime_ms    = 0; // 按鈕按下的時間
unsigned long brakeTime_ms    = 0; // 觸發煞車的時間
unsigned long stopTime_ms     = 0; // 車輛靜止的時間

long  brakeTriggerPulse     = 0; // 觸發煞車當下的累計脈衝
long  stopPulse             = 0; // 車輛靜止時的累計脈衝

float brakeInstantSpeed_cms = 0; // 煞車觸發瞬間的速度（cm/s）
float brakeDistance_cm      = 0; // 最終煞車距離（公分）
float brakeTime_s           = 0; // 從煞車到靜止的時間（秒）
float totalDistance_cm      = 0; // 含加速段的總行駛距離（公分）

// 速度計算用（滑動窗口）
long    lastSpeedCalcPulse = 0;
unsigned long lastSpeedCalcTime = 0;
float   instantSpeed_cms   = 0; // 即時速度（cm/s）

// 超音波快取
float   ultrasonicDistance_cm = 999.0;
long    lastStopConfirmPulse = 0;
unsigned long lastPulseChangeTime = 0;

// WiFi 狀態
bool    wifiConnected       = false;
bool    dataUploaded         = false;
String  uploadStatusMessage = "Not sent";

// 按鈕狀態
bool    lastButtonState     = HIGH;  // 按鈕初始狀態（未按下）

// TTC 與緩衝距離
float   TTC_s                = 0;    // 碰撞時間（秒）
float   bufferDistance_cm    = 0;    // 緩衝距離

// MQTT
WiFiClient mqttWifiClient;
PubSubClient mqttClient(mqttWifiClient);

// ═══════════════════════════════════════════════════════════
void mqttCallback(char* topic, byte* payload, unsigned int length) {
  // 收取 MQTT 訊息
  payload[length] = '\0';
  String message = String((char*)payload);
  String cmdTopic = String(topic);

  Serial.printf("[MQTT] 收到主題: %s  訊息: %s\n", topic, message);

  if (cmdTopic == MQTT_TOPIC_CMD) {
    // 解析 JSON 格式的命令
    // 格式: {"speed": 200, "brake": 20}
    JsonDocument doc;
    DeserializationError error = deserializeJson(doc, message);

    if (!error) {
      if (doc["speed"].is<int>()) {
        remoteMotorSpeed = doc["speed"].as<int>();
        Serial.printf("[MQTT] 更新馬達速度: PWM %d\n", remoteMotorSpeed);
      }
      if (doc["brake"].is<float>()) {
        remoteBrakeTrigger = doc["brake"].as<float>();
        Serial.printf("[MQTT] 更新煞車觸發距離: %.1f cm\n", remoteBrakeTrigger);
      }
      // 回覆狀態
      publishStatus();
    }
  }
}

void mqttInit() {
  mqttClient.setServer(MQTT_BROKER, MQTT_PORT);
  mqttClient.setCallback(mqttCallback);

  Serial.printf("[MQTT] 連線到 %s:%d\n", MQTT_BROKER, MQTT_PORT);
}

void mqttConnect() {
  if (!mqttClient.connected()) {
    Serial.print("[MQTT] 連線中...");
    if (mqttClient.connect(MQTT_CLIENT_ID)) {
      Serial.println("成功！");
      mqttClient.subscribe(MQTT_TOPIC_CMD);
      Serial.printf("[MQTT] 已訂閱主題: %s\n", MQTT_TOPIC_CMD);
    } else {
      Serial.printf("失敗！rc=%d\n", mqttClient.state());
    }
  }
  mqttClient.loop();
}

void publishStatus() {
  if (mqttClient.connected()) {
    char jsonBuf[256];
    snprintf(jsonBuf, sizeof(jsonBuf),
      "{\"speed\":%d,\"brake\":%.1f,\"state\":%d,\"obstacle\":%.1f,\"lastSpeed\":%.1f}",
      remoteMotorSpeed, remoteBrakeTrigger, currentState,
      ultrasonicDistance_cm, instantSpeed_cms);

    mqttClient.publish(MQTT_TOPIC_STS, jsonBuf);
    Serial.printf("[MQTT] 發送狀態: %s\n", jsonBuf);
  }
}

// ═══════════════════════════════════════════════════════════
//  前向宣告
// ═══════════════════════════════════════════════════════════
void playReadySound();
void playStartSound();
void playBrakeTriggerSound();
void playCompleteSound();
void playResetSound();
void playAbortSound();

// ═══════════════════════════════════════════════════════════
//  編碼器 ISR（IRAM 加速）
// ═══════════════════════════════════════════════════════════
void IRAM_ATTR isrEncA() { encoderA_Count = encoderA_Count + 1; }
void IRAM_ATTR isrEncB() { encoderB_Count = encoderB_Count + 1; }

// ═══════════════════════════════════════════════════════════
//  馬達控制（ESP32 Arduino Core 3.x LEDC API）
// ═══════════════════════════════════════════════════════════
void motorInit() {
  pinMode(PIN_STBY, OUTPUT);
  pinMode(PIN_AIN1, OUTPUT); pinMode(PIN_AIN2, OUTPUT);
  pinMode(PIN_BIN1, OUTPUT); pinMode(PIN_BIN2, OUTPUT);

  // 確保馬達處於停止狀態
  digitalWrite(PIN_STBY, LOW);      // Standby 模式
  digitalWrite(PIN_AIN1, LOW); digitalWrite(PIN_AIN2, LOW);  // 停止
  digitalWrite(PIN_BIN1, LOW); digitalWrite(PIN_BIN2, LOW);  // 停止

  // 設定 PWM（此時不輸出）
  ledcAttach(PIN_PWMA, PWM_FREQUENCY, PWM_RESOLUTION);
  ledcAttach(PIN_PWMB, PWM_FREQUENCY, PWM_RESOLUTION);
  ledcWrite(PIN_PWMA, 0);  // 速度設為 0
  ledcWrite(PIN_PWMB, 0);  // 速度設為 0
}

void motorForward(int speed) {
  digitalWrite(PIN_STBY, HIGH);
  digitalWrite(PIN_AIN1, HIGH); digitalWrite(PIN_AIN2, LOW);
  digitalWrite(PIN_BIN1, HIGH); digitalWrite(PIN_BIN2, LOW);
  ledcWrite(PIN_PWMA, speed);
  ledcWrite(PIN_PWMB, speed);
}

void motorBrake() {
  // 主動制動：AIN1=AIN2=HIGH（或 LOW）短路馬達線圈產生反向電磁力
  // TB6612 規格：PWMA/PWMB 保持 HIGH + AIN1=AIN2=HIGH → 短路制動
  digitalWrite(PIN_STBY, HIGH);       // STBY 必須維持 HIGH 才能制動
  ledcWrite(PIN_PWMA, 255);           // PWM 滿載確保制動力最強
  ledcWrite(PIN_PWMB, 255);
  digitalWrite(PIN_AIN1, HIGH); digitalWrite(PIN_AIN2, HIGH);  // 馬達A短路
  digitalWrite(PIN_BIN1, HIGH); digitalWrite(PIN_BIN2, HIGH);  // 馬達B短路
}

// ═══════════════════════════════════════════════════════════
//  WiFi 連線與資料上傳
// ═══════════════════════════════════════════════════════════
void wifiInit() {
  Serial.println("\n[WiFi] 正在連線...");
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);

  int connectAttempts = 0;
  while (WiFi.status() != WL_CONNECTED && connectAttempts < 30) {
    delay(500);
    Serial.print(".");
    connectAttempts++;
  }

  if (WiFi.status() == WL_CONNECTED) {
    wifiConnected = true;
    Serial.println("\n[WiFi] 連線成功！");
    Serial.printf("[WiFi] IP: %s\n", WiFi.localIP().toString().c_str());
    playReadySound();
  } else {
    wifiConnected = false;
    Serial.println("\n[WiFi] 連線失敗，請檢查 SSID/密碼");
    uploadStatusMessage = "WiFi Fail";
  }
}

void uploadToGoogleSheets() {
  if (!wifiConnected) {
    Serial.println("[上傳] WiFi 未連線，嘗試重新連線...");
    wifiInit();
    if (!wifiConnected) {
      uploadStatusMessage = "No WiFi";
      return;
    }
  }

  HTTPClient http;
  http.begin(GOOGLE_SCRIPT_URL);
  http.addHeader("Content-Type", "application/json");
  http.addHeader("User-Agent", "ESP32");
  http.setFollowRedirects(HTTPC_FORCE_FOLLOW_REDIRECTS);  // 允許跟隨 redirect

  // 計算 TTC 與緩衝距離
  if (brakeInstantSpeed_cms > 0) {
    TTC_s = ultrasonicDistance_cm / brakeInstantSpeed_cms;
  } else {
    TTC_s = 0;
  }
  bufferDistance_cm = ultrasonicDistance_cm - brakeDistance_cm;

  // 建立 JSON payload（時間由 Google Sheets 自動產生）
  String jsonPayload = "{";
  jsonPayload += "\"pwm\":" + String(remoteMotorSpeed) + ",";
  jsonPayload += "\"initialSpeed\":" + String(brakeInstantSpeed_cms) + ",";
  jsonPayload += "\"obstacleDistance\":" + String(remoteBrakeTrigger) + ",";
  jsonPayload += "\"ttc\":" + String(TTC_s) + ",";
  jsonPayload += "\"brakeDistance\":" + String(brakeDistance_cm) + ",";
  jsonPayload += "\"bufferDistance\":" + String(bufferDistance_cm);
  jsonPayload += "}";

  Serial.printf("[上傳] 發送資料: %s\n", jsonPayload.c_str());

  int httpCode = http.POST(jsonPayload);

  if (httpCode > 0) {
    String response = http.getString();
    Serial.printf("[上傳] HTTP Code: %d\n", httpCode);
    Serial.printf("[上傳] Response: %s\n", response.c_str());

    // 檢查回應是否包含 "success"
    if (response.indexOf("success") != -1) {
      dataUploaded = true;
      uploadStatusMessage = "Uploaded";
      Serial.println("[上傳] 成功！");
    } else {
      uploadStatusMessage = "Failed";
      Serial.println("[上傳] 失敗！");
    }
  } else {
    uploadStatusMessage = "Error";
    Serial.printf("[上傳] 錯誤: %s\n", http.errorToString(httpCode).c_str());
  }

  http.end();
}

// ═══════════════════════════════════════════════════════════
//  HC-SR04 超音波測距
// ═══════════════════════════════════════════════════════════
float readUltrasonicDistance() {
  digitalWrite(PIN_ULTRASONIC_TRIG, LOW);
  delayMicroseconds(2);
  digitalWrite(PIN_ULTRASONIC_TRIG, HIGH);
  delayMicroseconds(10);
  digitalWrite(PIN_ULTRASONIC_TRIG, LOW);
  long duration = pulseIn(PIN_ULTRASONIC_ECHO, HIGH, 25000); // 25ms = 最遠約 430 公分
  return (duration == 0) ? 999.0f : duration * 0.01715f;
}

// ═══════════════════════════════════════════════════════════
//  蜂鳴器音效（被動式蜂鳴器）
// ═══════════════════════════════════════════════════════════
void beep(int frequencyHz, int durationMs) {
  tone(PIN_BUZZER, frequencyHz);
  delay(durationMs);
  noTone(PIN_BUZZER);
}

void playReadySound() {
  beep(1000, 80); delay(60); beep(1500, 120);
}

void playStartSound() {
  beep(1200, 70); delay(50); beep(1800, 70); delay(50); beep(2400, 120);
}

void playBrakeTriggerSound() {
  beep(600, 80); delay(40); beep(400, 150);
}

void playCompleteSound() {
  beep(800, 100); delay(80); beep(1200, 100); delay(80);
  beep(1600, 100); delay(80); beep(2000, 300);
}

void playResetSound()  { beep(700, 200); }
void playAbortSound()  { beep(800, 80); delay(50); beep(500, 200); }

// ═══════════════════════════════════════════════════════════
//  OLED 輔助：繪製進度條
// ═══════════════════════════════════════════════════════════
void drawProgressBar(int x, int y, int w, int h, float percentage) {
  oled.drawRoundRect(x, y, w, h, 2, SSD1306_WHITE);
  int fill = (int)(constrain(percentage, 0.0f, 1.0f) * (w - 4));
  if (fill > 0)
    oled.fillRoundRect(x + 2, y + 2, fill, h - 4, 1, SSD1306_WHITE);
}

// ═══════════════════════════════════════════════════════════
//  OLED 顯示：啟動畫面
// ═══════════════════════════════════════════════════════════
void showStartupScreen() {
  oled.clearDisplay();
  oled.setTextColor(SSD1306_WHITE);

  oled.setTextSize(2);
  oled.setCursor(4, 0);
  oled.print("Terry Car");

  oled.drawLine(0, 17, 128, 17, SSD1306_WHITE);

  oled.setTextSize(1);
  char buf[22];

  oled.setCursor(0, 21);
  snprintf(buf, sizeof(buf), "Speed  : PWM %d", remoteMotorSpeed);
  oled.print(buf);

  oled.setCursor(0, 31);
  snprintf(buf, sizeof(buf), "Brake  : %.0f cm", remoteBrakeTrigger);
  oled.print(buf);

  oled.setCursor(0, 41);
  snprintf(buf, sizeof(buf), "PPR    : %d", ENCODER_PPR);
  oled.print(buf);

  oled.setCursor(0, 51);
  snprintf(buf, sizeof(buf), "cm/pls : %.5f", DIST_PER_PULSE);
  oled.print(buf);

  oled.display();
  delay(3000);
}

// ═══════════════════════════════════════════════════════════
//  OLED 顯示：待機畫面
// ═══════════════════════════════════════════════════════════
void showIdleScreen() {
  oled.clearDisplay();
  oled.setTextColor(SSD1306_WHITE);
  char buf[22];

  // 大字狀態
  oled.setTextSize(2);
  oled.setCursor(10, 0);
  oled.print("  READY");

  oled.setTextSize(1);
  oled.drawLine(0, 17, 128, 17, SSD1306_WHITE);

  oled.setCursor(0, 21);
  snprintf(buf, sizeof(buf), "Speed: PWM %3d", remoteMotorSpeed);
  oled.print(buf);

  oled.setCursor(0, 31);
  snprintf(buf, sizeof(buf), "Brake: %5.0f cm", remoteBrakeTrigger);
  oled.print(buf);

  oled.setCursor(0, 41);
  snprintf(buf, sizeof(buf), "Obs  : %5.1f cm",
           ultrasonicDistance_cm < 400 ? ultrasonicDistance_cm : 999.0f);
  oled.print(buf);

  oled.setCursor(0, 54);
  oled.print("[BTN] Start Test");

  oled.display();
}

// ═══════════════════════════════════════════════════════════
//  OLED 顯示：行駛中
// ═══════════════════════════════════════════════════════════
void showRunningScreen() {
  long avgPulse = (encoderA_Count + encoderB_Count) / 2;
  float distance = avgPulse * DIST_PER_PULSE;

  oled.clearDisplay();
  oled.setTextColor(SSD1306_WHITE);
  char buf[22];

  oled.setTextSize(2);
  oled.setCursor(4, 0);
  oled.print("RUNNING!");

  oled.setTextSize(1);

  oled.setCursor(0, 17);
  snprintf(buf, sizeof(buf), "Speed:%6.1f cm/s", instantSpeed_cms);
  oled.print(buf);

  oled.setCursor(0, 27);
  snprintf(buf, sizeof(buf), "Dist :%6.1f cm", distance);
  oled.print(buf);

  // 超音波距離（煞車觸發警示）
  oled.setCursor(0, 37);
  if (ultrasonicDistance_cm < remoteBrakeTrigger * 1.5f)
    snprintf(buf, sizeof(buf), "Obs  :%6.1f cm !!!", ultrasonicDistance_cm);
  else
    snprintf(buf, sizeof(buf), "Obs  :%6.1f cm", ultrasonicDistance_cm);
  oled.print(buf);

  oled.setCursor(0, 47);
  snprintf(buf, sizeof(buf), "Pulse: A=%4ld B=%4ld", encoderA_Count, encoderB_Count);
  oled.print(buf);

  // 超音波距離進度條（150公分為滿）
  float percentage = constrain(1.0f - ultrasonicDistance_cm / 150.0f, 0.0f, 1.0f);
  drawProgressBar(0, 57, 128, 7, percentage);

  oled.display();
}

// ═══════════════════════════════════════════════════════════
//  OLED 顯示：煞車中
// ═══════════════════════════════════════════════════════════
void showBrakingScreen() {
  long avgPulse       = (encoderA_Count + encoderB_Count) / 2;
  long afterBrakePulse = avgPulse - brakeTriggerPulse;
  float currentBrakeDist = afterBrakePulse * DIST_PER_PULSE;
  float elapsedSeconds   = (millis() - brakeTime_ms) / 1000.0f;

  oled.clearDisplay();
  oled.setTextColor(SSD1306_WHITE);
  char buf[22];

  oled.setTextSize(2);
  oled.setCursor(4, 0);
  oled.print("BRAKING!");

  oled.setTextSize(1);

  oled.setCursor(0, 17);
  snprintf(buf, sizeof(buf), "BrkDst:%5.1f cm", currentBrakeDist);
  oled.print(buf);

  oled.setCursor(0, 27);
  snprintf(buf, sizeof(buf), "BrkTim:%5.2f s", elapsedSeconds);
  oled.print(buf);

  oled.setCursor(0, 37);
  snprintf(buf, sizeof(buf), "BrkSpd:%5.1f cm/s", brakeInstantSpeed_cms);
  oled.print(buf);

  oled.setCursor(0, 47);
  snprintf(buf, sizeof(buf), "CurSpd:%5.1f cm/s", instantSpeed_cms);
  oled.print(buf);

  // 速度衰減進度條
  float speedPercentage = (brakeInstantSpeed_cms > 0)
                          ? constrain(instantSpeed_cms / brakeInstantSpeed_cms, 0.0f, 1.0f)
                          : 0.0f;
  drawProgressBar(0, 57, 128, 7, speedPercentage);

  oled.display();
}

// ═══════════════════════════════════════════════════════════
//  OLED 顯示：測試結果
// ═══════════════════════════════════════════════════════════
void showResultScreen() {
  oled.clearDisplay();
  oled.setTextColor(SSD1306_WHITE);
  char buf[22];

  // 標題列（反白）
  oled.fillRect(0, 0, 128, 11, SSD1306_WHITE);
  oled.setTextColor(SSD1306_BLACK);
  oled.setTextSize(1);
  oled.setCursor(8, 2);
  oled.print("=== BRAKE RESULT ===");
  oled.setTextColor(SSD1306_WHITE);

  oled.setCursor(0, 13);
  snprintf(buf, sizeof(buf), "Spd :%5.1f cm/s", brakeInstantSpeed_cms);
  oled.print(buf);

  oled.setCursor(0, 23);
  snprintf(buf, sizeof(buf), "Brk :%5.1f cm", brakeDistance_cm);
  oled.print(buf);

  oled.setCursor(0, 33);
  snprintf(buf, sizeof(buf), "TTC :%5.3f s", TTC_s);
  oled.print(buf);

  oled.setCursor(0, 43);
  snprintf(buf, sizeof(buf), "Buf :%5.1f cm", bufferDistance_cm);
  oled.print(buf);

  // 上傳狀態
  oled.setCursor(80, 43);
  if (dataUploaded) {
    oled.print("OK");
  } else {
    oled.print("--");
  }

  oled.drawLine(0, 53, 128, 53, SSD1306_WHITE);
  oled.setCursor(0, 55);
  oled.print("[BTN] Reset");

  // 右下角顯示 PWM
  oled.setCursor(72, 55);
  snprintf(buf, sizeof(buf), "PWM=%d", remoteMotorSpeed);
  oled.print(buf);

  oled.display();
}

// ═══════════════════════════════════════════════════════════
//  OLED 顯示：已中止畫面
// ═══════════════════════════════════════════════════════════
void showAbortedScreen() {
  long avgPulse  = (encoderA_Count + encoderB_Count) / 2;
  float traveled = avgPulse * DIST_PER_PULSE;
  float elapsed  = (millis() - startTime_ms) / 1000.0f;

  oled.clearDisplay();
  oled.setTextColor(SSD1306_WHITE);
  char buf[22];

  // 標題列（反白）
  oled.fillRect(0, 0, 128, 11, SSD1306_WHITE);
  oled.setTextColor(SSD1306_BLACK);
  oled.setTextSize(1);
  oled.setCursor(20, 2);
  oled.print("=== ABORTED ===");
  oled.setTextColor(SSD1306_WHITE);

  oled.setCursor(0, 14);
  snprintf(buf, sizeof(buf), "Dist  :%6.1f cm", traveled);
  oled.print(buf);

  oled.setCursor(0, 24);
  snprintf(buf, sizeof(buf), "Speed :%6.1f cm/s", instantSpeed_cms);
  oled.print(buf);

  oled.setCursor(0, 34);
  snprintf(buf, sizeof(buf), "Time  :%6.2f s", elapsed);
  oled.print(buf);

  oled.setCursor(0, 44);
  snprintf(buf, sizeof(buf), "Pulse : A=%4ld B=%4ld", encoderA_Count, encoderB_Count);
  oled.print(buf);

  oled.drawLine(0, 54, 128, 54, SSD1306_WHITE);
  oled.setCursor(0, 56);
  oled.print("[BTN] Reset");

  oled.display();
}

// ═══════════════════════════════════════════════════════════
//  重置所有測試數據
// ═══════════════════════════════════════════════════════════
void resetTest() {
  encoderA_Count       = 0;
  encoderB_Count       = 0;
  brakeTriggerPulse    = 0;
  stopPulse            = 0;
  brakeInstantSpeed_cms = 0;
  brakeDistance_cm     = 0;
  brakeTime_s          = 0;
  totalDistance_cm     = 0;
  instantSpeed_cms     = 0;
  lastSpeedCalcPulse   = 0;
  lastSpeedCalcTime    = 0;
  lastStopConfirmPulse = 0;
  lastPulseChangeTime  = 0;
  currentState         = IDLE;
}

// ═══════════════════════════════════════════════════════════
//  更新即時速度（每 speed_calc_interval_ms 執行一次）
// ═══════════════════════════════════════════════════════════
void updateSpeed() {
  unsigned long now = millis();
  unsigned long timeDiff = now - lastSpeedCalcTime;
  if (timeDiff < (unsigned long)SPEED_CALC_INTERVAL_MS) return;

  long avg = (encoderA_Count + encoderB_Count) / 2;
  long pulseDiff = avg - lastSpeedCalcPulse;
  instantSpeed_cms = (pulseDiff * DIST_PER_PULSE) / (timeDiff / 1000.0f);

  lastSpeedCalcPulse = avg;
  lastSpeedCalcTime = now;
}

// ═══════════════════════════════════════════════════════════
//  setup()
// ═══════════════════════════════════════════════════════════
void setup() {
  Serial.begin(115200);
  Serial.println("\n=== Terry Car 煞車距離測試 ===");
  Serial.printf("啟動速度=PWM%d  煞車觸發=%.0fcm  每脈衝=%.5fcm\n",
    remoteMotorSpeed, remoteBrakeTrigger, DIST_PER_PULSE);

  // WiFi 連線
  wifiInit();

  // MQTT 初始化
  mqttInit();

  // OLED（Wire1：SDA=4, SCL=15）
  Wire1.begin(PIN_OLED_SDA, PIN_OLED_SCL);
  if (!oled.begin(SSD1306_SWITCHCAPVCC, 0x3C)) {
    Serial.println("[錯誤] OLED 初始化失敗！請確認 SDA=4 SCL=15");
    while (true) delay(500);
  }
  showStartupScreen();

  // 馬達
  motorInit();
  Serial.println("[OK] 馬達初始化完成");

  // 編碼器
  pinMode(PIN_ENCODER_A, INPUT);
  pinMode(PIN_ENCODER_B, INPUT);
  attachInterrupt(digitalPinToInterrupt(PIN_ENCODER_A), isrEncA, RISING);
  attachInterrupt(digitalPinToInterrupt(PIN_ENCODER_B), isrEncB, RISING);
  Serial.println("[OK] 編碼器中斷啟動 (A=34, B=36)");

  // 按鈕
  pinMode(PIN_BUTTON, INPUT_PULLUP);
  // 讀取按鈕初始狀態，確保一開始不會誤觸發
  while (digitalRead(PIN_BUTTON) == LOW) {
    delay(10);  // 等待按鈕放開
  }
  lastButtonState = HIGH;  // 初始化按鈕狀態為 HIGH（未按下）

  // 蜂鳴器
  pinMode(PIN_BUZZER, OUTPUT);

  // 超音波
  pinMode(PIN_ULTRASONIC_TRIG, OUTPUT);
  pinMode(PIN_ULTRASONIC_ECHO, INPUT);

  lastSpeedCalcTime = millis();
  lastPulseChangeTime = millis();

  playReadySound();
  Serial.println("[等待] 按下按鈕開始煞車測試...");
}

// ═══════════════════════════════════════════════════════════
//  loop()
// ═══════════════════════════════════════════════════════════
void loop() {
  static unsigned long lastOLEDUpdate = 0;
  static unsigned long lastUltrasonicRead = 0;
  static unsigned long lastStatusPublish = 0;

  unsigned long now = millis();

  // ── MQTT 連線與處理 ─────────────────────────────────────────
  mqttConnect();

  // 定時發布狀態（每 2 秒）
  if (now - lastStatusPublish >= 2000) {
    lastStatusPublish = now;
    publishStatus();
  }

  // ── 按鈕邊緣偵測 ───────────────────────────────────────────
  bool buttonNow = digitalRead(PIN_BUTTON);
  bool buttonPressed = (lastButtonState == HIGH && buttonNow == LOW);
  lastButtonState = buttonNow;

  if (buttonPressed) {
    delay(30);  // 消彈跳
    if (digitalRead(PIN_BUTTON) == LOW) {

      if (currentState == IDLE) {
        // ── 開始測試 ─────────────────────────────────────────
        resetTest();
        startTime_ms       = now;
        lastSpeedCalcTime  = now;
        lastPulseChangeTime = now;
        currentState       = RUNNING;
        motorForward(remoteMotorSpeed);
        playStartSound();
        Serial.println("[開始] 全速前進，監控障礙物距離...");

      } else if (currentState == RUNNING || currentState == BRAKING) {
        // ── 手動中止測試 ─────────────────────────────────────
        motorBrake();
        currentState = ABORTED;
        playAbortSound();
        long avg = (encoderA_Count + encoderB_Count) / 2;
        Serial.printf("[中止] 使用者手動停止  已行駛=%.1fcm  脈衝=%ld\n",
          avg * DIST_PER_PULSE, avg);

      } else if (currentState == COMPLETED || currentState == ABORTED) {
        // ── 重置回待機 ───────────────────────────────────────
        resetTest();
        playResetSound();
        Serial.println("[重置] 系統就緒。");
      }
    }
  }

  // ── 超音波定時量測 ─────────────────────────────────────────
  if (now - lastUltrasonicRead >= (unsigned long)ULTRASONIC_INTERVAL_MS) {
    lastUltrasonicRead = now;
    ultrasonicDistance_cm = readUltrasonicDistance();
  }

  // ── 即時速度更新 ───────────────────────────────────────────
  updateSpeed();

  // ══ RUNNING 狀態 ═══════════════════════════════════════════
  if (currentState == RUNNING) {
    // 偵測到障礙物 ≤ 煞車觸發距離 → 觸發煞車
    if (ultrasonicDistance_cm <= remoteBrakeTrigger) {
      motorBrake();

      brakeTime_ms         = millis();
      brakeTriggerPulse   = (encoderA_Count + encoderB_Count) / 2;
      brakeInstantSpeed_cms = instantSpeed_cms;
      currentState         = BRAKING;
      lastStopConfirmPulse = brakeTriggerPulse;
      lastPulseChangeTime  = millis();

      playBrakeTriggerSound();
      Serial.printf("[煞車] 觸發！距離=%.1fcm  速度=%.1fcm/s  脈衝=%ld\n",
        ultrasonicDistance_cm, brakeInstantSpeed_cms, brakeTriggerPulse);
    }
  }

  // ══ BRAKING 狀態 ═══════════════════════════════════════════
  if (currentState == BRAKING) {
    long avgPulse = (encoderA_Count + encoderB_Count) / 2;

    // 監控脈衝是否仍在變化
    if (avgPulse != lastStopConfirmPulse) {
      lastStopConfirmPulse = avgPulse;
      lastPulseChangeTime = millis();
    }

    // 脈衝超過靜止判定時間未變化 → 車輛已靜止
    if (millis() - lastPulseChangeTime >= (unsigned long)STOP_TIMEOUT_MS) {
      stopPulse        = avgPulse;
      brakeDistance_cm = (stopPulse - brakeTriggerPulse) * DIST_PER_PULSE;
      brakeTime_s      = (millis() - brakeTime_ms - STOP_TIMEOUT_MS) / 1000.0f;
      totalDistance_cm = stopPulse * DIST_PER_PULSE;
      currentState     = COMPLETED;
      dataUploaded     = false;  // 重置上傳狀態

      playCompleteSound();

      // 上傳資料到 Google Sheets
      uploadToGoogleSheets();

      Serial.printf("[完成] 煞車距離=%.2fcm  煞車時間=%.2fs  總行駛=%.1fcm\n",
        brakeDistance_cm, brakeTime_s, totalDistance_cm);
      Serial.printf("       煞車前速=%.1fcm/s  A脈衝=%ld  B脈衝=%ld\n",
        brakeInstantSpeed_cms, encoderA_Count, encoderB_Count);
      Serial.printf("       TTC=%.3fs  緩衝距離=%.1fcm\n", TTC_s, bufferDistance_cm);
    }
  }

  // ── OLED 定時更新（100ms）────────────────────────────────────
  if (now - lastOLEDUpdate >= 100) {
    lastOLEDUpdate = now;
    switch (currentState) {
      case IDLE:     showIdleScreen(); break;
      case RUNNING:  showRunningScreen(); break;
      case BRAKING:  showBrakingScreen(); break;
      case COMPLETED: showResultScreen(); break;
      case ABORTED:  showAbortedScreen(); break;
    }
  }
}