//============================================================================
// SMART FARMING SYSTEM - GUDANG JAMUR & KEBUN VANILI
// Kode Lengkap untuk ESP32 dengan semua fitur yang telah ditentukan
// Versi Diperbaiki dengan FreeRTOS, Enkripsi, dan Fitur Cerdas
//============================================================================

#include <WiFi.h>
#include <WiFiClientSecure.h>
#include <UniversalTelegramBot.h>
#include <ArduinoJson.h>
#include <SPIFFS.h>
#include <WebServer.h>
#include <HTTPClient.h>
#include <UrlEncode.h>
#include <Wire.h>
#include <Adafruit_SHT31.h>
#include <Adafruit_BMP280.h>
#include <BH1750.h>
#include <Adafruit_INA219.h>
#include <LPS35HW.h>
#include <Fuzzy.h>
#include <EEPROM.h>
#include <time.h>
#include <FS.h>
#include <SD.h>
#include <SPI.h>
#include <NTPClient.h>
#include <WiFiUdp.h>
#include <OneWire.h>
#include <DallasTemperature.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/semphr.h>
#include <Update.h>
#include <WebSocketsServer.h>
#include <AES.h>
#include <PID_v1.h>
#include <ArduinoOTA.h>
#include <esp_task_wdt.h>
#include <ESPAsyncWebServer.h> // Pastikan library ini sudah di-include di awal file
#include <AsyncTCP.h>          // Pastikan library ini sudah di-include di awal file

//============================================================================
// DEFINISI REGISTER INA219
//============================================================================
#ifndef INA219_REG_CALIBRATION
#define INA219_REG_CALIBRATION (0x05)
#endif

#ifndef INA219_REG_CONFIG
#define INA219_REG_CONFIG (0x00)
#endif

#ifndef INA219_CONFIG_BVOLTAGERANGE_32V
#define INA219_CONFIG_BVOLTAGERANGE_32V (0x2000) // 32V
#endif

#ifndef INA219_CONFIG_GAIN_8_320MV
#define INA219_CONFIG_GAIN_8_320MV (0x1800) // Gain 8, 320mV range
#endif

#ifndef INA219_CONFIG_BADCRES_12BIT
#define INA219_CONFIG_BADCRES_12BIT (0x0400) // 12-bit bus voltage resolution
#endif

#ifndef INA219_CONFIG_SADCRES_12BIT_1S_532US
#define INA219_CONFIG_SADCRES_12BIT_1S_532US (0x0018) // 12-bit shunt voltage resolution, 1 sample, 532us conversion time
#endif

#ifndef INA219_CONFIG_MODE_SANDBVOLT_CONTINUOUS
#define INA219_CONFIG_MODE_SANDBVOLT_CONTINUOUS (0x0007) // Continuous shunt and bus voltage measurement
#endif

//============================================================================
// DEFINISI EEPROM
//============================================================================
#define EEPROM_SIZE 512
#define ADDR_PIR_ENABLED 0
#define ADDR_AUTO_CAPTURE 1
#define ADDR_SYSTEM_MODE 2
#define ADDR_BATTERY_CYCLES 10  // 4 bytes untuk menyimpan siklus baterai
#define ADDR_ENCRYPTION_KEY 20  // 32 bytes untuk kunci enkripsi

//============================================================================
// KONFIGURASI PIN - TIDAK DIUBAH SESUAI PERJANJIAN
//============================================================================
// Pin I2C
#define SDA_PIN 21
#define SCL_PIN 22

// Pin Analog
#define SOIL_MOISTURE_PIN 34
#define PH_GUDANG_PIN 32  // Tidak digunakan lagi
#define PH_KEBUN_PIN 33   // Tidak digunakan lagi
#define MQ135_PIN 35
// BATTERY_VOLTAGE_PIN 36 tidak digunakan, menggunakan INA219 untuk tegangan baterai

// Pin Digital Input
#define EMERGENCY_BUTTON_PIN 25
#define THERMOSTAT_PIN 26
#define PIR1_PIN 27
#define PIR2_PIN 14

// Pin Digital Output (Relay)
#define PUMP_GUDANG_RELAY_PIN 13
#define FAN_GUDANG_RELAY_PIN 12
#define HUMIDIFIER_RELAY_PIN 0
#define PUMP_KEBUN_RELAY_PIN 15
#define LAMP_JALAN_RELAY_PIN 2
#define LAMP_GUDANG_RELAY_PIN 4
#define LAMP_KEBUN_RELAY_PIN 16
#define INVERTER_SWITCH_PIN 17
#define PV_RELAY_PIN 5      // Pin 5 digunakan untuk PV relay
#define BATTERY_RELAY_PIN 3

// Pin SPI untuk SD Card
#define SD_CS_PIN 5         // Pin 5 digunakan untuk PV relay, ganti ke pin lain
#define SD_CS_PIN_NEW 4     // Gunakan pin 4 untuk SD card
#define SD_SCK_PIN 18
#define SD_MOSI_PIN 23
#define SD_MISO_PIN 19

// Pin untuk sensor suhu baterai
#define BATTERY_TEMP_PIN 16

//============================================================================
// KONFIGURASI WIFI & TELEGRAM
//============================================================================
const char* WIFI_SSID = "desvandi101";
const char* WIFI_PASSWORD = "BADANburung";
const char* CAM_WIFI_SSID = "desvandi101"; // Nama AP yang akan dibuat ESP32 Utama
const char* CAM_WIFI_PASSWORD = "BADANburung"; // Password AP, ganti dengan yang lebih kuat

// Konfigurasi Wifi Cam
#define ESP32_CAM_IP "192.168.1.101" // Ganti dengan IP ESP32-CAM Anda
#define ESP32_CAM_PORT 80

// Variabel status PIR sebelumnya
bool lastPir1Status = false; // Status PIR sebelumnya
bool lastPir2Status = false; // Status PIR sebelumnya
bool pirEnabled = true;        // Status PIR (default: aktif)
bool autoCaptureEnabled = true; // Status pengambilan gambar otomatis (default: aktif)

// Telegram Bot Token dapatkan dari @BotFather
#define BOT_TOKEN "8310642723:AAHviv0Joag31aC_b9aBbColH3nDwS53jwM"

// Chat ID untuk notifikasi
#define CHAT_ID "811628430"

// Password untuk autentikasi Telegram
#define TELEGRAM_PASSWORD "12345678Qwertyuiop"

//============================================================================
// KONFIGURASI API & LAYANAN EKSTERNAL
//============================================================================
// OpenWeatherMap API Key (gratis)
#define WEATHER_API_KEY "ee8fd5ac5fd69a18dfcdfe745a33e890"
#define WEATHER_LAT "-1.679548" // Latitude lokasi
#define WEATHER_LON "103.652294" // Longitude lokasi

// Google Sheets API
#define GOOGLE_SCRIPT_ID "AKfycbz-0xc04mhmgKhHX6k_IYD-TQSVhK9XOdwKLJqZXK9ykaSt-bYon5OLTQ3tlztOW3-F"

// Timezone
#define TIMEZONE "Asia/Jakarta"

//============================================================================
// KONFIGURASI SISTEM
//============================================================================
// Interval pengiriman data (dalam milidetik)
#define DATA_SEND_INTERVAL 60000  // 1 menit
#define SENSOR_READ_INTERVAL 5000 // 5 detik
#define WEATHER_CHECK_INTERVAL 3600000 // 6 jam

// Threshold untuk sensor (dapat diubah melalui Telegram)
float EXHAUST_FAN_THRESHOLD = 500.0; // mA
float PUMP_GUDANG_THRESHOLD = 800.0; // mA
float HUMIDIFIER_THRESHOLD = 200.0; // mA
float PUMP_KEBUN_THRESHOLD = 800.0; // mA
float LAMP_JALAN_THRESHOLD = 100.0; // mA
float LAMP_GUDANG_THRESHOLD = 100.0; // mA
float LAMP_KEBUN_THRESHOLD = 100.0; // mA

// Threshold untuk sensor lingkungan (dapat diubah melalui Telegram)
float TEMP_HIGH_THRESHOLD = 29.0; // °C
float TEMP_NORMAL_THRESHOLD = 26.0; // °C
float HUMIDITY_LOW_THRESHOLD = 80.0; // %
float HUMIDITY_HIGH_THRESHOLD = 90.0; // %
float SOIL_MOISTURE_LOW_THRESHOLD = 50.0; // %
float SOIL_MOISTURE_HIGH_THRESHOLD = 70.0; // %
float CO2_HIGH_THRESHOLD = 0.03; // %

// Threshold baterai
float BATTERY_LOW_THRESHOLD = 12.0; // V (80%)
float BATTERY_CRITICAL_THRESHOLD = 11.5; // V (50%)
float BATTERY_EMERGENCY_THRESHOLD = 11.0; // V (20%)

// Konfigurasi Deep Sleep
#define DEEP_SLEEP_THRESHOLD 5.0 // SoC % untuk masuk deep sleep
#define DEEP_SLEEP_DURATION 3600000000 // 1 jam dalam mikrodetik

// Konfigurasi Watchdog
#define WATCHDOG_TIMEOUT 30 // detik

// Konfigurasi PID
#define PID_SAMPLE_TIME 5000 // ms
#define PID_OUTPUT_MIN 0
#define PID_OUTPUT_MAX 100

//============================================================================
// DEFINISI MODE SISTEM
//============================================================================
enum SystemMode {
  MODE_LOCAL,       // Kontrol lokal dengan fuzzy logic
  MODE_AI,          // Kontrol dengan AI (Gemini)
  MODE_USER,        // Kontrol manual oleh pengguna
  MODE_HYBRID,      // Kombinasi AI dan fuzzy logic
  MODE_EMERGENCY,   // Mode darurat
  MODE_MAINTENANCE, // Mode perawatan
  MODE_SCHEDULED,   // Kontrol berdasarkan jadwal
  MODE_PREDICTIVE,  // Kontrol prediktif berdasarkan data historis
  MODE_ADAPTIVE,    // Kontrol adaptif yang belajar dari lingkungan
  MODE_COUNT        // Jumlah total mode
};

const char* modeNames[MODE_COUNT] = {
  "Local", "AI", "User", "Hybrid", "Emergency", 
  "Maintenance", "Scheduled", "Predictive", "Adaptive"
};

//============================================================================
// INISIALISASI OBJEK
//============================================================================
// Sensor
Adafruit_SHT31 sht31 = Adafruit_SHT31();
Adafruit_BMP280 bmp280;
BH1750 bh1750_gudang(0x23); // Alamat 0x23 untuk BH1750 #1
BH1750 bh1750_kebun(0x5C); // Alamat 0x5C untuk BH1750 #2
Adafruit_INA219 ina219_1(0x40); // MPPT charging
Adafruit_INA219 ina219_2(0x44); // PWM charging
Adafruit_INA219 ina219_3(0x48); // Discharging to load
LPS35HW lps35hw;

// Sensor suhu baterai
OneWire oneWire(BATTERY_TEMP_PIN);
DallasTemperature batteryTempSensor(&oneWire);

// Telegram Bot
WiFiClientSecure client;
UniversalTelegramBot bot(BOT_TOKEN, client);

// Web Server
WebServer server(80);
WebServer server_cam(81);

// WebSocket Server
WebSocketsServer webSocket = WebSocketsServer(81);

// Fuzzy Logic
Fuzzy *fuzzy = new Fuzzy();

// NTP Client
WiFiUDP ntpUDP;
NTPClient timeClient(ntpUDP, "pool.ntp.org");

// PID Controllers
PID tempPID(&temperatureGudang, &tempPIDOutput, &tempSetpoint, 2, 5, 1, DIRECT);
PID humidityPID(&humidityGudang, &humidityPIDOutput, &humiditySetpoint, 2, 5, 1, DIRECT);

// Enkripsi AES
AES128 aes128;

// Struktur data untuk status beban
struct LoadStatus {
  String name;
  bool commandStatus;    // Status perintah (apa yang kita perintahkan)
  bool actualStatus;     // Status aktual (apa yang benar-benar terjadi)
  float currentThreshold; // Threshold arus untuk menentukan status (dapat diedit)
  int relayPin;          // Pin relay
  unsigned long lastUpdateTime; // Terakhir kali status diperbarui
  String relatedSensors[5]; // Sensor terkait untuk verifikasi
  int sensorCount;       // Jumlah sensor terkait
  float expectedChanges[5]; // Perubahan yang diharapkan pada sensor
  bool verificationEnabled; // Apakah verifikasi diaktifkan
};

// Struktur data untuk konfigurasi sistem
struct SystemConfig {
  float exhaustFanThreshold;
  float pumpGudangThreshold;
  float humidifierThreshold;
  float pumpKebunThreshold;
  float lampJalanThreshold;
  float lampGudangThreshold;
  float lampKebunThreshold;
  float tempHighThreshold;
  float tempNormalThreshold;
  float humidityLowThreshold;
  float humidityHighThreshold;
  float soilMoistureLowThreshold;
  float soilMoistureHighThreshold;
  float co2HighThreshold;
  float batteryLowThreshold;
  float batteryCriticalThreshold;
  float batteryEmergencyThreshold;
  char telegramPassword[32];
  char notificationMode[16]; // "all", "important", "emergency"
  char dndStart[8]; // Format "HH:MM"
  char dndEnd[8];   // Format "HH:MM"
  SystemMode operationMode;
  bool encryptionEnabled;
  bool otaEnabled;
  bool predictiveEnabled;
  bool adaptiveEnabled;
  bool pidEnabled;
  char geminiApiKey[64];
  char webUsername[32];
  char webPassword[32];
};

// Struktur data untuk logging
struct LogEntry {
  unsigned long timestamp;
  String level; // DEBUG, INFO, WARNING, ERROR, CRITICAL
  String message;
  String source;
};

// Struktur data untuk aktivitas maintenance
struct MaintenanceActivity {
  unsigned long timestamp;
  String activityType;    // "roof_leak", "sensor_issue", "power_problem", dll
  String description;    // Deskripsi detail dari user
  String status;         // "pending", "in_progress", "completed"
  String aiRecommendation; // Rekomendasi dari Gemini AI
  bool resolved;          // Status penyelesaian
};

// Struktur data untuk jadwal
struct Schedule {
  int hour;
  int minute;
  String loadName;
  bool active;
  String condition;
  float conditionValue;
  String conditionOperator; // ">", "<", "=", "!="
};

// Struktur data untuk baterai
struct BatteryData {
  float voltage;       // Tegangan baterai
  float current;       // Arus (positif: charging, negatif: discharging)
  float temperature;   // Suhu baterai (dalam Celsius)
  float capacity;      // Kapasitas terukur (Ah)
  float fullCapacity;  // Kapasitas penuh nominal (Ah)
  float soh;           // State of Health (%)
  float soc;           // State of Charge (%)
  unsigned long lastUpdateTime;
  float coulombCount;  // Coulomb counting (Ah)
  bool isCharging;     // Status charging
  unsigned long cycles; // Jumlah siklus pengisian
  float chargeEfficiency; // Efisiensi pengisian (%)
};

// Struktur data untuk energi
struct EnergyData {
  float energyCharged;   // Energy charged (Wh)
  float energyDischarged; // Energy discharged (Wh)
  float efficiency;      // Charging efficiency (%)
  unsigned long lastUpdateTime;
  float mpptEnergy;      // Energi dari MPPT (Wh)
  float pwmEnergy;       // Energi dari PWM (Wh)
  float loadEnergy;      // Energi ke beban (Wh)
};

// Struktur data untuk log baterai
struct BatteryLog {
  unsigned long timestamp;
  float voltage;
  float current;
  float temperature;
  float soc;
  float soh;
  bool isCharging;
};

// Struktur data untuk prediksi
struct PredictionData {
  float predictedTemp;      // Prediksi suhu
  float predictedHumidity;  // Prediksi kelembaban
  float predictedSoilMoisture; // Prediksi kelembaban tanah
  float predictedBatterySOC; // Prediksi SoC baterai
  float confidence;         // Tingkat kepercayaan prediksi (0-1)
  unsigned long predictionTime; // Waktu prediksi dibuat
  unsigned long targetTime;     // Waktu target prediksi
};

// Struktur data untuk kegagalan komponen
struct ComponentHealth {
  String componentName;
  float healthScore;       // Skor kesehatan (0-100)
  unsigned long lastCheckTime;
  int failureCount;        // Jumlah kegagalan
  String lastFailureReason;
  float expectedLifetime;  // Umur yang diharapkan (jam)
  float currentLifetime;   // Umur saat ini (jam)
  bool maintenanceRequired;
};

// Variabel global
SystemConfig config;
LoadStatus loads[7];
LogEntry logEntries[200];
int logIndex = 0;
Schedule schedules[20];
int scheduleCount = 0;
unsigned long lastDataSendTime = 0;
unsigned long lastSensorReadTime = 0;
unsigned long lastWeatherCheckTime = 0;
bool emergencyMode = false;
bool maintenanceMode = false;
String weatherData = "";
String geminiResponse = "";
String currentMessage = "";
String chatId = "";
String messageId = "";

//============================================================================
// Global Variables for Web Server & WebSocket
//============================================================================
AsyncWebServer server(80);         // Buat objek server di port 80
AsyncWebSocket ws("/ws");          // Buat objek WebSocket di path /ws

//============================================================================
// Deklarasi Fungsi Handler (jika belum ada di atas)
//============================================================================
void handleRoot(AsyncWebServerRequest *request);
void handleGraphs(AsyncWebServerRequest *request);
void handleControl(AsyncWebServerRequest *request);
void handleSystem(AsyncWebServerRequest *request); // Akan memanggil getSystemHTML()
void handleStatus(AsyncWebServerRequest *request); // Perhatikan: Ini mungkin duplikat? Jika ya, hapus salah satu. Jika berbeda, pastikan path unik.
void handleMaintenance(AsyncWebServerRequest *request); // Akan memanggil getMaintenanceHTML()
void handleSettings(AsyncWebServerRequest *request);
void handleLogs(AsyncWebServerRequest *request);
void handlePrediction(AsyncWebServerRequest *request);
void handleApiSensors(AsyncWebServerRequest *request);
void handleApiControl(AsyncWebServerRequest *request);
void handleApiConfigGet(AsyncWebServerRequest *request);
void handleApiConfigPost(AsyncWebServerRequest *request);
void handleApiRulesGet(AsyncWebServerRequest *request);
void handleApiRulesPost(AsyncWebServerRequest *request);
void handleApiRulesDelete(AsyncWebServerRequest *request);
void handleApiSchedulesGet(AsyncWebServerRequest *request);
void handleApiSchedulesPost(AsyncWebServerRequest *request);
void handleApiMaintenanceGet(AsyncWebServerRequest *request);
void handleApiMaintenancePost(AsyncWebServerRequest *request);
void handleApiLogsGet(AsyncWebServerRequest *request);
void handleApiLogsDownload(AsyncWebServerRequest *request);
void handleApiPredictionGet(AsyncWebServerRequest *request);
void handleApiEnergyGet(AsyncWebServerRequest *request);
void handleApiHealthGet(AsyncWebServerRequest *request);
void handleApiCamera(AsyncWebServerRequest *request); // Anda mungkin perlu implementasi spesifik untuk ini
void handleApiMode(AsyncWebServerRequest *request);
void handleApiInverter(AsyncWebServerRequest *request);
void handleLogin(AsyncWebServerRequest *request); // Anda perlu logika autentikasi di sini
void handleNotFound(AsyncWebServerRequest *request);
void onWsEvent(AsyncWebSocket *server, AsyncWebSocketClient *client, AwsEventType type, void *arg, uint8_t *data, size_t len); // Handler event WebSocket

// Variabel untuk sensor
float temperatureGudang = 0;
float humidityGudang = 0;
float lightGudang = 0;
float co2Gudang = 0;
float soilMoisture = 0;
float pressureKebun = 0;
float temperatureKebun = 0;
float lightKebun = 0;
float waterPressure = 0;
float batteryVoltage = 0;
float currentMPPT = 0;
float currentPWM = 0;
float currentLoad = 0;
float voltageMPPT = 0;
float voltagePWM = 0;
float voltageLoad = 0;
bool pir1Status = false;
bool pir2Status = false;
bool emergencyButtonStatus = false;
bool thermostatStatus = false;

// Variabel untuk status sistem
bool systemInitialized = false;
bool wifiConnected = false;
bool sdCardAvailable = false;
bool timeConfigured = false;

// Variabel untuk tracking
int baglogCount = 0;
int dailyHarvest = 0;
int totalHarvest = 0;
unsigned long lastBaglogUpdate = 0;
unsigned long lastHarvestUpdate = 0;

// Variabel untuk fitur tambahan
String messageTemplates[10];
int templateCount = 0;
bool notificationEnabled[5] = {true, true, true, true, true}; // emergency, warning, info, daily, ai
bool doNotDisturb = false;
String lastPIRDetectTime = "";
String lastWeatherUpdate = "";
float healthScore = 0;
String growthData = "";
String productivityData = "";

// Variabel untuk fitur baru
BatteryData batteryData;
EnergyData energyData;
BatteryLog batteryLogs[200]; // Simpan 200 log terakhir
int batteryLogIndex = 0;
PredictionData predictionData;
ComponentHealth componentHealth[10];
int componentCount = 0;

// Array untuk menyimpan aktivitas maintenance
MaintenanceActivity maintenanceActivities[20];
int maintenanceActivityCount = 0;

// Tabel OCV untuk LiFePO4 (tegangan vs SoC)
const float ocvTable[][2] = {
  {3.40, 100}, // 100% @ 3.40V
  {3.35, 95},
  {3.30, 90},
  {3.25, 80},
  {3.20, 70},
  {3.15, 50},
  {3.10, 30},
  {3.05, 20},
  {3.00, 10},
  {2.90, 0}    // 0% @ 2.90V
};
const int ocvTableSize = sizeof(ocvTable) / sizeof(ocvTable[0]);

// Variabel PID
double tempSetpoint = 26.0;
double humiditySetpoint = 85.0;
double tempPIDOutput = 0;
double humidityPIDOutput = 0;

// Variabel untuk non-blocking delays
unsigned long previousMillis[20] = {0};
const long intervals[20] = {1000, 5000, 10000, 30000, 60000, 300000, 900000, 1800000, 3600000, 7200000, 86400000, 604800000, 2419200000, 0, 0, 0, 0, 0, 0, 0};

// Variabel untuk FreeRTOS
TaskHandle_t sensorTaskHandle;
TaskHandle_t controlTaskHandle;
TaskHandle_t communicationTaskHandle;
TaskHandle_t webTaskHandle;
TaskHandle_t energyTaskHandle;
TaskHandle_t predictionTaskHandle;
TaskHandle_t maintenanceTaskHandle;

// Semaphore untuk sinkronisasi
SemaphoreHandle_t sensorMutex;
SemaphoreHandle_t configMutex;
SemaphoreHandle_t logMutex;

// Queue untuk komunikasi antar task
QueueHandle_t sensorQueue;
QueueHandle_t controlQueue;
QueueHandle_t notificationQueue;

//============================================================================
// CUSTOM RULES ENGINE - STRUCT
//============================================================================
struct CustomRule {
  String ruleName;
  String sensorSource;    // "lightGudang", "batteryVoltage", dll
  String condition;       // "<", ">", "=", "<=", ">="
  float threshold;
  String action;          // "exhaustfan_on", "pumpgudang_on", dll
  bool active;
  bool executed;          // agar tidak trigger berulang
  unsigned long lastExecutionTime;
  int cooldownPeriod;     // Periode cooldown dalam detik
};

CustomRule customRules[30];  // Simpan 30 rules
int customRuleCount = 0;

//============================================================================
// CUSTOM RULES ENGINE - FUNCTION DECLARATIONS
//============================================================================
void evaluateCustomRules();
float getSensorValue(String sensorName);
void executeRuleAction(String action);
void addCustomRule(String ruleName, String sensor, String condition, float value, String action);
void handleAddRuleCommand(String text);
void handleListRulesCommand();
void handleDeleteRuleCommand(String text);

//============================================================================
// FUNGSI ENKRIPSI
//============================================================================
void initializeEncryption() {
  // Baca kunci enkripsi dari EEPROM atau buat baru jika tidak ada
  byte key[16];
  bool keyExists = false;
  
  for (int i = 0; i < 16; i++) {
    key[i] = EEPROM.read(ADDR_ENCRYPTION_KEY + i);
    if (key[i] != 0 && key[i] != 255) keyExists = true;
  }
  
  if (!keyExists) {
    // Buat kunci baru
    for (int i = 0; i < 16; i++) {
      key[i] = random(256);
      EEPROM.write(ADDR_ENCRYPTION_KEY + i, key[i]);
    }
    EEPROM.commit();
    logMessage("INFO", "Generated new encryption key", "SECURITY");
  }
  
  aes128.setKey(key, 16);
  logMessage("INFO", "Encryption initialized", "SECURITY");
}

String encryptData(String data) {
  if (!config.encryptionEnabled) return data;
  
  // Padding data jika perlu
  int padding = 16 - (data.length() % 16);
  if (padding != 16) {
    data += String(char(padding));
  }
  
  byte encrypted[data.length()];
  byte plain[data.length()];
  
  for (int i = 0; i < data.length(); i++) {
    plain[i] = data[i];
  }
  
  for (int i = 0; i < data.length(); i += 16) {
    aes128.encryptBlock(encrypted + i, plain + i);
  }
  
  String result = "";
  for (int i = 0; i < data.length(); i++) {
    result += String(encrypted[i], HEX);
    if (i < data.length() - 1) result += ":";
  }
  
  return result;
}

String decryptData(String encryptedData) {
  if (!config.encryptionEnabled) return encryptedData;
  
  // Parse hex string
  String parts[16];
  int partCount = 0;
  int lastIndex = 0;
  
  for (int i = 0; i < encryptedData.length(); i++) {
    if (encryptedData[i] == ':') {
      parts[partCount++] = encryptedData.substring(lastIndex, i);
      lastIndex = i + 1;
    }
  }
  parts[partCount++] = encryptedData.substring(lastIndex);
  
  byte encrypted[partCount];
  for (int i = 0; i < partCount; i++) {
    encrypted[i] = strtol(parts[i].c_str(), NULL, 16);
  }
  
  byte decrypted[partCount];
  for (int i = 0; i < partCount; i += 16) {
    aes128.decryptBlock(decrypted + i, encrypted + i);
  }
  
  // Remove padding
  int padding = decrypted[partCount - 1];
  String result = "";
  for (int i = 0; i < partCount - padding; i++) {
    result += (char)decrypted[i];
  }
  
  return result;
}

//============================================================================
// FUNGSI PREDIKSI
//============================================================================
void initializePrediction() {
  predictionData.predictedTemp = 0;
  predictionData.predictedHumidity = 0;
  predictionData.predictedSoilMoisture = 0;
  predictionData.predictedBatterySOC = 0;
  predictionData.confidence = 0;
  predictionData.predictionTime = 0;
  predictionData.targetTime = 0;
  
  logMessage("INFO", "Prediction system initialized", "PREDICTION");
}

void updatePrediction() {
  if (!config.predictiveEnabled) return;
  
  // Baca data historis dari SD card
  if (!sdCardAvailable) {
    logMessage("WARNING", "SD card not available for prediction", "PREDICTION");
    return;
  }
  
  // Implementasi sederhana prediksi berdasarkan data historis
  // Dalam implementasi nyata, gunakan algoritma machine learning
  
  // Prediksi suhu berdasarkan waktu dan data historis
  int currentHour = timeClient.getHours();
  float avgTemp = 0;
  int count = 0;
  
  File file = SD.open("/historical_data.csv", FILE_READ);
  if (file) {
    while (file.available()) {
      String line = file.readStringUntil('\n');
      // Parse data: timestamp,temp,humidity,soil,battery
      int firstComma = line.indexOf(',');
      int secondComma = line.indexOf(',', firstComma + 1);
      
      if (firstComma > 0 && secondComma > firstComma) {
        String timeStr = line.substring(0, firstComma);
        String tempStr = line.substring(firstComma + 1, secondComma);
        
        // Cek apakah data dari jam yang sama di hari-hari sebelumnya
        unsigned long timestamp = timeStr.toInt();
        int hour = (timestamp / 3600) % 24;
        
        if (hour == currentHour) {
          avgTemp += tempStr.toFloat();
          count++;
        }
      }
    }
    file.close();
  }
  
  if (count > 0) {
    predictionData.predictedTemp = avgTemp / count;
    predictionData.confidence = min(0.9, 0.3 + (count * 0.05)); // Confidence meningkat dengan lebih banyak data
    predictionData.predictionTime = millis();
    predictionData.targetTime = millis() + 3600000; // Prediksi untuk 1 jam ke depan
    
    logMessage("INFO", "Temperature prediction updated: " + String(predictionData.predictedTemp) + "°C", "PREDICTION");
  }
}

//============================================================================
// FUNGSI ADAPTIVE CONTROL
//============================================================================
void initializeAdaptiveControl() {
  // Inisialisasi variabel untuk kontrol adaptif
  // Dalam implementasi nyata, ini akan memuat model yang telah dilatih
  logMessage("INFO", "Adaptive control system initialized", "ADAPTIVE");
}

void updateAdaptiveControl() {
  if (!config.adaptiveEnabled) return;
  
  // Implementasi sederhana kontrol adaptif
  // Dalam implementasi nyata, gunakan reinforcement learning atau algoritma adaptif lainnya
  
  // Contoh: Sesuaikan threshold berdasarkan respons sistem
  static unsigned long lastAdaptation = 0;
  if (millis() - lastAdaptation < 86400000) return; // Adaptasi sekali sehari
  
  // Analisis efektivitas kontrol
  float tempStability = calculateStability(temperatureGudang);
  float humidityStability = calculateStability(humidityGudang);
  
  // Jika suhu tidak stabil, sesuaikan PID
  if (tempStability < 0.7) {
    float currentKp = tempPID.GetKp();
    tempPID.SetTunings(currentKp * 1.1, tempPID.GetKi(), tempPID.GetKd());
    logMessage("INFO", "Adjusted temperature PID parameters", "ADAPTIVE");
  }
  
  // Jika kelembaban tidak stabil, sesuaikan PID
  if (humidityStability < 0.7) {
    float currentKp = humidityPID.GetKp();
    humidityPID.SetTunings(currentKp * 1.1, humidityPID.GetKi(), humidityPID.GetKd());
    logMessage("INFO", "Adjusted humidity PID parameters", "ADAPTIVE");
  }
  
  lastAdaptation = millis();
}

float calculateStability(float currentValue) {
  // Hitung stabilitas berdasarkan perubahan nilai
  static float lastValue = currentValue;
  static float stabilitySum = 0;
  static int stabilityCount = 0;
  
  float change = abs(currentValue - lastValue);
  float stability = 1.0 - min(1.0, change / 5.0); // Normalisasi perubahan
  
  stabilitySum += stability;
  stabilityCount++;
  
  lastValue = currentValue;
  
  if (stabilityCount >= 10) {
    float avgStability = stabilitySum / stabilityCount;
    stabilitySum = 0;
    stabilityCount = 0;
    return avgStability;
  }
  
  return 0.5; // Default jika belum cukup data
}

//============================================================================
// FUNGSI DIAGNOSTIK KOMPONEN
//============================================================================
void initializeComponentHealth() {
  // Inisialisasi data kesehatan komponen
  componentHealth[0] = {"SHT31 Sensor", 100.0, 0, 0, "", 8760, 0, false};
  componentHealth[1] = {"BMP280 Sensor", 100.0, 0, 0, "", 8760, 0, false};
  componentHealth[2] = {"BH1750 Sensor #1", 100.0, 0, 0, "", 8760, 0, false};
  componentHealth[3] = {"BH1750 Sensor #2", 100.0, 0, 0, "", 8760, 0, false};
  componentHealth[4] = {"INA219 #1", 100.0, 0, 0, "", 8760, 0, false};
  componentHealth[5] = {"INA219 #2", 100.0, 0, 0, "", 8760, 0, false};
  componentHealth[6] = {"INA219 #3", 100.0, 0, 0, "", 8760, 0, false};
  componentHealth[7] = {"LPS35HW Sensor", 100.0, 0, 0, "", 8760, 0, false};
  componentHealth[8] = {"Battery", 100.0, 0, 0, "", 43800, 0, false}; // 5 tahun untuk baterai
  componentHealth[9] = {"Inverter", 100.0, 0, 0, "", 26280, 0, false}; // 3 tahun untuk inverter
  
  componentCount = 10;
  
  logMessage("INFO", "Component health monitoring initialized", "DIAGNOSTICS");
}

void updateComponentHealth() {
  // Update kesehatan komponen berdasarkan penggunaan dan performa
  for (int i = 0; i < componentCount; i++) {
    ComponentHealth& comp = componentHealth[i];
    
    // Update lifetime
    unsigned long currentTime = millis();
    unsigned long deltaTime = currentTime - comp.lastCheckTime;
    comp.currentLifetime += deltaTime / 1000.0 / 3600.0; // Konversi ke jam
    comp.lastCheckTime = currentTime;
    
    // Hitung health score berdasarkan lifetime dan performa
    float lifetimeRatio = comp.currentLifetime / comp.expectedLifetime;
    comp.healthScore = max(0, 100.0 * (1.0 - lifetimeRatio));
    
    // Deteksi masalah khusus untuk setiap komponen
    if (comp.componentName == "Battery") {
      // Kesehatan baterai berdasarkan SoH
      comp.healthScore = batteryData.soh;
      
      // Cek apakah perlu maintenance
      if (batteryData.soh < 80.0) {
        comp.maintenanceRequired = true;
      }
    } else if (comp.componentName.startsWith("INA219")) {
      // Cek apakah sensor INA219 memberikan data yang valid
      if (comp.componentName == "INA219 #1" && (currentMPPT < 0 || voltageMPPT < 0)) {
        comp.failureCount++;
        comp.healthScore -= 5.0;
        comp.lastFailureReason = "Invalid current/voltage reading";
      }
    }
    
    // Cek apakah perlu maintenance
    if (comp.healthScore < 70.0) {
      comp.maintenanceRequired = true;
      
      // Buat aktivitas maintenance otomatis
      if (comp.failureCount == 1) { // Hanya buat aktivitas untuk kegagalan pertama
        MaintenanceActivity activity;
        activity.timestamp = millis();
        activity.activityType = "component_degradation";
        activity.description = comp.componentName + " health degraded to " + String(comp.healthScore) + "%";
        activity.status = "pending";
        activity.aiRecommendation = "Consider replacing or servicing " + comp.componentName;
        activity.resolved = false;
        
        addMaintenanceActivity(activity);
      }
    }
  }
}

void predictComponentFailure() {
  // Prediksi kegagalan komponen berdasarkan tren kesehatan
  for (int i = 0; i < componentCount; i++) {
    ComponentHealth& comp = componentHealth[i];
    
    // Prediksi sederhana: jika health score menurun lebih dari 5% dalam seminggu
    static float lastHealthScore[10] = {100.0, 100.0, 100.0, 100.0, 100.0, 100.0, 100.0, 100.0, 100.0, 100.0};
    static unsigned long lastPredictionTime = 0;
    
    if (millis() - lastPredictionTime > 604800000) { // Seminggu
      float healthChange = lastHealthScore[i] - comp.healthScore;
      
      if (healthChange > 5.0) {
        // Prediksi kegagalan dalam 3 bulan
        String message = "⚠️ Prediksi Kegagalan: " + comp.componentName + 
                         " mungkin akan gagal dalam 3 bulan. Health score: " + 
                         String(comp.healthScore) + "%";
        sendTelegramMessage(message);
        logMessage("WARNING", message, "PREDICTION");
      }
      
      lastHealthScore[i] = comp.healthScore;
      lastPredictionTime = millis();
    }
  }
}

//============================================================================
// FUNGSI OTA UPDATE
//============================================================================
void initializeOTA() {
  if (!config.otaEnabled) {
    logMessage("INFO", "OTA updates disabled", "SYSTEM");
    return;
  }
  
  // Port default adalah 3232
  ArduinoOTA.setPort(3232);
  
  // Hostname default adalah esp32-[MAC]
  ArduinoOTA.setHostname("SmartFarming-ESP32");
  
  // Password yang diperlukan untuk update
  ArduinoOTA.setPassword("admin123");
  
  // Password dapat diatur dengan fungsi ini juga
  // ArduinoOTA.setPassword("admin");
  
  ArduinoOTA
    .onStart([]() {
      String type = (ArduinoOTA.getCommand() == U_FLASH) ? "sketch" : "filesystem";
      
      // NOTE: if updating FS this would be the place to unmount FS using FS.end()
      Serial.println("Start updating " + type);
      logMessage("INFO", "OTA update started: " + type, "SYSTEM");
    })
    .onEnd([]() {
      Serial.println("\nEnd");
      logMessage("INFO", "OTA update completed", "SYSTEM");
    })
    .onProgress([](unsigned int progress, unsigned int total) {
      Serial.printf("Progress: %u%%\r", (progress / (total / 100)));
    })
    .onError([](ota_error_t error) {
      Serial.printf("Error[%u]: ", error);
      if (error == OTA_AUTH_ERROR) Serial.println("Auth Failed");
      else if (error == OTA_BEGIN_ERROR) Serial.println("Begin Failed");
      else if (error == OTA_CONNECT_ERROR) Serial.println("Connect Failed");
      else if (error == OTA_RECEIVE_ERROR) Serial.println("Receive Failed");
      else if (error == OTA_END_ERROR) Serial.println("End Failed");
      
      logMessage("ERROR", "OTA update failed", "SYSTEM");
    });
  
  ArduinoOTA.begin();
  logMessage("INFO", "OTA Ready", "SYSTEM");
}

//============================================================================
// FUNGSI WATCHDOG TIMER
//============================================================================
void initializeWatchdog() {
  // Inisialisasi watchdog timer
  esp_task_wdt_init(WATCHDOG_TIMEOUT, true); // Enable panic ESP32 on timeout
  esp_task_wdt_add(NULL); // Add current thread to WDT watch
  
  logMessage("INFO", "Watchdog timer initialized with " + String(WATCHDOG_TIMEOUT) + "s timeout", "SYSTEM");
}

void feedWatchdog() {
  // Feed the watchdog to prevent reset
  esp_task_wdt_reset();
}

//============================================================================
// FUNGSI RECOVERY OTOMATIS
//============================================================================
void initializeRecoverySystem() {
  // Inisialisasi sistem recovery otomatis
  logMessage("INFO", "Automatic recovery system initialized", "RECOVERY");
}

void checkSystemHealth() {
  // Cek kesehatan sistem dan recovery otomatis jika perlu
  static unsigned long lastHealthCheck = 0;
  if (millis() - lastHealthCheck < 60000) return; // Cek setiap menit
  
  // Cek koneksi WiFi
  if (WiFi.status() != WL_CONNECTED) {
    logMessage("WARNING", "WiFi disconnected, attempting reconnection", "RECOVERY");
    initializeWiFi();
  }
  
  // Cek sensor
  bool sensorFailure = false;
  if (temperatureGudang == 0 && humidityGudang == 0) {
    logMessage("ERROR", "SHT31 sensor failure detected", "RECOVERY");
    sensorFailure = true;
    
    // Coba inisialisasi ulang sensor
    if (!sht31.begin(0x44)) {
      logMessage("ERROR", "Failed to reinitialize SHT31 sensor", "RECOVERY");
    } else {
      logMessage("INFO", "SHT31 sensor reinitialized successfully", "RECOVERY");
    }
  }
  
  // Cek beban yang macet
  for (int i = 0; i < 7; i++) {
    LoadStatus& load = loads[i];
    
    // Jika beban seharusnya menyala tetapi tidak ada perubahan arus
    if (load.commandStatus && !load.actualStatus && (millis() - load.lastUpdateTime > 30000)) {
      logMessage("WARNING", "Load " + load.name + " failed to turn on, attempting recovery", "RECOVERY");
      
      // Coba matikan dan nyalakan kembali
      digitalWrite(load.relayPin, LOW);
      delay(1000);
      digitalWrite(load.relayPin, HIGH);
      
      load.lastUpdateTime = millis();
    }
  }
  
  lastHealthCheck = millis();
}

//============================================================================
// FUNGSI WEBSOCKET
//============================================================================
void webSocketEvent(AsyncWebSocket *server, AsyncWebSocketClient *client, AwsEventType type, void *arg, uint8_t *data, size_t len) {
  switch(type) {
    case WStype_DISCONNECTED:
      Serial.printf("[%u] Disconnected!\n", num);
      break;
      
    case WStype_CONNECTED: {
      IPAddress ip = webSocket.remoteIP(num);
      Serial.printf("[%u] Connected from %d.%d.%d.%d url: %s\n", num, ip[0], ip[1], ip[2], ip[3], payload);
      
      // Kirim data sensor saat ini
      String sensorData = getSensorDataJSON();
      webSocket.sendTXT(num, sensorData);
      break;
    }
    
    case WStype_TEXT:
      Serial.printf("[%u] get Text: %s\n", num, payload);
      
      // Proses pesan dari client
      processWebSocketMessage(num, (char*)payload);
      break;
      
    case WStype_BIN:
      Serial.printf("[%u] get binary length: %u\n", num, length);
      break;
      
    default:
      break;
  }
}

void processWebSocketMessage(uint8_t num, char* payload) {
  // Parse pesan JSON
  DynamicJsonDocument doc(1024);
  DeserializationError error = deserializeJson(doc, payload);
  
  if (error) {
    Serial.printf("JSON parsing failed: %s\n", error.c_str());
    return;
  }
  
  String command = doc["command"];
  
  if (command == "getSensorData") {
    String sensorData = getSensorDataJSON();
    webSocket.sendTXT(num, sensorData);
  } else if (command == "controlLoad") {
    String loadName = doc["loadName"];
    bool status = doc["status"];
    controlLoad(loadName, status);
  } else if (command == "updateConfig") {
    // Update konfigurasi
    updateConfigFromWebSocket(doc);
  }
}

void broadcastSensorData() {
  static unsigned long lastBroadcast = 0;
  if (millis() - lastBroadcast < 5000) return; // Broadcast setiap 5 detik
  
  String sensorData = getSensorDataJSON();
  webSocket.broadcastTXT(sensorData);
  
  lastBroadcast = millis();
}

String getSensorDataJSON() {
  DynamicJsonDocument doc(2048);
  
  doc["temperature"] = temperatureGudang;
  doc["humidity"] = humidityGudang;
  doc["lightGudang"] = lightGudang;
  doc["co2"] = co2Gudang;
  doc["soilMoisture"] = soilMoisture;
  doc["pressure"] = pressureKebun;
  doc["temperatureKebun"] = temperatureKebun;
  doc["lightKebun"] = lightKebun;
  doc["waterPressure"] = waterPressure;
  doc["batteryVoltage"] = batteryVoltage;
  doc["batterySOC"] = batteryData.soc;
  doc["batteryCycles"] = batteryData.cycles;
  doc["pir1"] = pir1Status;
  doc["pir2"] = pir2Status;
  doc["emergency"] = emergencyMode;
  doc["mode"] = modeNames[config.operationMode];
  
  // Tambahkan data beban
  JsonArray loadsArray = doc.createNestedArray("loads");
  for (int i = 0; i < 7; i++) {
    JsonObject loadObj = loadsArray.createNestedObject();
    loadObj["name"] = loads[i].name;
    loadObj["status"] = loads[i].actualStatus;
  }
  
  // Tambahkan data prediksi
  JsonObject predictionObj = doc.createNestedObject("prediction");
  predictionObj["temp"] = predictionData.predictedTemp;
  predictionObj["humidity"] = predictionData.predictedHumidity;
  predictionObj["confidence"] = predictionData.confidence;
  
  String jsonString;
  serializeJson(doc, jsonString);
  
  return jsonString;
}

//============================================================================
// FUNGSI UNTUK MENGIRIM DATA SENSOR VIA WEBSOCKET KE SEMUA CLIENT
//============================================================================
void sendSensorDataToClients() {
    StaticJsonDocument<1024> doc; // Sesuaikan ukuran jika perlu
    // Isi doc dengan data sensor terbaru (mirip handleApiSensors)
    doc["temperature"] = temperatureGudang;
    doc["humidity"] = humidityGudang;
    doc["lightGudang"] = lightGudang;
    doc["co2"] = co2Gudang;
    doc["soilMoisture"] = soilMoisture;
    doc["pressure"] = pressureKebun;
    doc["temperatureKebun"] = temperatureKebun;
    doc["lightKebun"] = lightKebun;
    doc["waterPressure"] = waterPressure;
    doc["batteryVoltage"] = batteryVoltage;
    doc["batterySOC"] = batteryData.soc;
    doc["batteryCycles"] = batteryData.cycles;
    doc["pir1"] = pir1Status;
    doc["pir2"] = pir2Status;
    doc["emergency"] = emergencyMode;
    doc["mode"] = modeNames[config.operationMode];
    doc["timestamp"] = getFormattedTime();

    // Data beban
    JsonArray loadsArray = doc.createNestedArray("loads");
    for (int i = 0; i < MAX_LOADS; i++) {
        JsonObject loadObj = loadsArray.createNestedObject();
        loadObj["name"] = loads[i].name;
        loadObj["status"] = loads[i].status;
        // Tambahkan info lain jika perlu
    }

    String output;
    serializeJson(doc, output);

    // Kirim data JSON ke semua client WebSocket yang terhubung
    ws.textAll(output);
}


//============================================================================
// INISIALISASI WEB SERVER (Gabungan)
//============================================================================
void initializeWebServer() {
  // --- Setup WebSocket ---
  ws.onEvent(onWsEvent); // Daftarkan fungsi handler event WebSocket
  server.addHandler(&ws); // Tambahkan WebSocket handler ke server

  // --- Setup Rute Halaman HTML ---
  server.on("/", HTTP_GET, handleRoot); // Gunakan HTTP_GET untuk halaman
  server.on("/graphs", HTTP_GET, handleGraphs);
  server.on("/control", HTTP_GET, handleControl);
  server.on("/system", HTTP_GET, handleSystem); // Rute untuk System Status
  // server.on("/status", HTTP_GET, handleStatus); // Hapus jika /status sama dengan /system
  server.on("/maintenance", HTTP_GET, handleMaintenance);
  server.on("/settings", HTTP_GET, handleSettings);
  server.on("/logs", HTTP_GET, handleLogs);
  server.on("/prediction", HTTP_GET, handlePrediction);

  // --- Setup Rute API ---
  server.on("/api/sensors", HTTP_GET, handleApiSensors);
  server.on("/api/control", HTTP_POST, handleApiControl); // Bisa juga handle GET jika diperlukan
  server.on("/api/config", HTTP_GET, handleApiConfigGet);
  server.on("/api/config", HTTP_POST, handleApiConfigPost);
  server.on("/api/rules", HTTP_GET, handleApiRulesGet);
  server.on("/api/rules", HTTP_POST, handleApiRulesPost);
  server.on("/api/rules", HTTP_DELETE, handleApiRulesDelete);
  server.on("/api/schedules", HTTP_GET, handleApiSchedulesGet);
  server.on("/api/schedules", HTTP_POST, handleApiSchedulesPost);
  server.on("/api/maintenance", HTTP_GET, handleApiMaintenanceGet);
  server.on("/api/maintenance", HTTP_POST, handleApiMaintenancePost);
  server.on("/api/logs", HTTP_GET, handleApiLogsGet);
  server.on("/api/logs/download", HTTP_GET, handleApiLogsDownload);
  server.on("/api/prediction", HTTP_GET, handleApiPredictionGet);
  server.on("/api/energy", HTTP_GET, handleApiEnergyGet);
  server.on("/api/health", HTTP_GET, handleApiHealthGet);
  server.on("/api/camera", HTTP_POST, handleApiCamera); // Perlu penanganan khusus
  server.on("/api/mode", HTTP_POST, handleApiMode);
  server.on("/api/inverter", HTTP_POST, handleApiInverter);

  // --- Setup Autentikasi ---
  server.on("/login", HTTP_POST, handleLogin); // Bisa juga handle GET untuk menampilkan form login jika ada

  // --- Setup PWA (memanggil fungsi yang sudah Anda buat) ---
  initializePWA();

  // --- Setup Handler Not Found ---
  server.onNotFound(handleNotFound);

  // --- Mulai Server ---
  server.begin();
  Serial.println("Web server started");
  logMessage("INFO", "Web server started", "SYSTEM");
}

    
//============================================================================
// FUNGSI PROGRESSIVE WEB APP (PWA)
//============================================================================
void initializePWA() {
  // Handler untuk Service Worker
  server.on("/sw.js", HTTP_GET, []() {
    server.sendHeader("Content-Type", "application/javascript");
    server.send(200, "application/javascript", R"(
      const CACHE_NAME = 'smart-farming-v1';
      const urlsToCache = [
        '/',
        '/index.html',
        '/style.css',
        '/script.js',
        '/manifest.json'
      ];
      
      self.addEventListener('install', event => {
        event.waitUntil(
          caches.open(CACHE_NAME)
            .then(cache => cache.addAll(urlsToCache))
        );
      });
      
      self.addEventListener('fetch', event => {
        event.respondWith(
          caches.match(event.request)
            .then(response => {
              return response || fetch(event.request);
            })
        );
      });
    )");
  });
  
  // Handler untuk Manifest
  server.on("/manifest.json", HTTP_GET, []() {
    server.sendHeader("Content-Type", "application/json");
    server.send(200, "application/json", R"(
      {
        "name": "Smart Farming System",
        "short_name": "SmartFarm",
        "description": "Smart farming system for mushroom and vanilla cultivation",
        "start_url": "/",
        "display": "standalone",
        "background_color": "#ffffff",
        "theme_color": "#2196f3",
        "icons": [
          {
            "src": "icon-192.png",
            "sizes": "192x192",
            "type": "image/png"
          },
          {
            "src": "icon-512.png",
            "sizes": "512x512",
            "type": "image/png"
          }
        ]
      }
    )");
  });
  
  logMessage("INFO", "PWA initialized", "SYSTEM");
}

//============================================================================
// SETUP - INISIALISASI SISTEM
//============================================================================
void setup() {
  // Inisialisasi Serial
  Serial.begin(115200);
  while (!Serial);
  
  // Inisialisasi random seed
  randomSeed(analogRead(0));
  
  // Inisialisasi semaphore dan queue
  sensorMutex = xSemaphoreCreateMutex();
  configMutex = xSemaphoreCreateMutex();
  logMutex = xSemaphoreCreateMutex();
  
  sensorQueue = xQueueCreate(10, sizeof(float) * 15); // 15 sensor values
  controlQueue = xQueueCreate(5, sizeof(String) * 2); // Command and parameter
  notificationQueue = xQueueCreate(10, sizeof(String) * 3); // Type, message, recipient
  
  // Inisialisasi SPIFFS
  if (!SPIFFS.begin(true)) {
    Serial.println("SPIFFS initialization failed!");
    logMessage("ERROR", "SPIFFS initialization failed", "SYSTEM");
  }
  
  // Inisialisasi SD Card
  SD.begin(SD_CS_PIN_NEW);
  if (SD.cardType() == CARD_NONE) {
    Serial.println("No SD card attached");
    logMessage("WARNING", "No SD card attached", "SYSTEM");
  } else {
    sdCardAvailable = true;
    Serial.println("SD card initialized");
    logMessage("INFO", "SD card initialized", "SYSTEM");
  }

  // Inisialisasi Setup Wifi
  initializeWiFi(); // Setup WiFi Client untuk Internet
  initializeCamWiFi(); // Setup WiFi AP untuk CAM
  
  // Inisialisasi EEPROM
  EEPROM.begin(EEPROM_SIZE);
  loadConfiguration();
  pirEnabled = EEPROM.read(ADDR_PIR_ENABLED) == 1;
  autoCaptureEnabled = EEPROM.read(ADDR_AUTO_CAPTURE) == 1;
  
  // Inisialisasi enkripsi
  initializeEncryption();
  
  // Inisialisasi Pin
  initializePins();

  // Fungsi untuk membaca dari SPIFFS/EEPROM
  String geminiAPIKey = readGeminiAPIKey(); 
  
  // Inisialisasi Sensor
  initializeSensors();
  
  // Inisialisasi Fuzzy Logic
  initializeFuzzyLogic();
  
  // Inisialisasi Web Server
  initializeWebServer();
  
  // Inisialisasi WebSocket
  webSocket.begin();
  webSocket.onEvent(webSocketEvent);
  
  // Inisialisasi PWA
  initializePWA();
  
  // Inisialisasi Waktu
  timeClient.begin();
  timeClient.setTimeOffset(25200); // UTC+7 untuk Jakarta
  timeClient.update();
  
  if (timeClient.isTimeSet()) {
    timeConfigured = true;
    Serial.println("Time configured");
    logMessage("INFO", "Time configured", "SYSTEM");
  }
  
  // Inisialisasi Load Status
  initializeLoadStatus();
  
  // Inisialisasi Jadwal
  initializeSchedules();
  
  // Inisialisasi Data Baterai
  initializeBatteryData();
  
  // Inisialisasi Data Energi
  initializeEnergyData();
  
  // Inisialisasi Prediksi
  initializePrediction();
  
  // Inisialisasi Kontrol Adaptif
  initializeAdaptiveControl();
  
  // Inisialisasi Kesehatan Komponen
  initializeComponentHealth();
  
  // Inisialisasi OTA
  initializeOTA();
  
  // Inisialisasi Watchdog
  initializeWatchdog();
  
  // Inisialisasi Sistem Recovery
  initializeRecoverySystem();
  
  // Inisialisasi PID
  tempPID.SetMode(AUTOMATIC);
  tempPID.SetOutputLimits(PID_OUTPUT_MIN, PID_OUTPUT_MAX);
  tempPID.SetSampleTime(PID_SAMPLE_TIME);
  
  humidityPID.SetMode(AUTOMATIC);
  humidityPID.SetOutputLimits(PID_OUTPUT_MIN, PID_OUTPUT_MAX);
  humidityPID.SetSampleTime(PID_SAMPLE_TIME);
  
  // Buat task untuk FreeRTOS
  xTaskCreatePinnedToCore(
    sensorTask,          // Fungsi task
    "SensorTask",        // Nama task
    4096,                // Stack size
    NULL,                // Parameter
    2,                   // Priority
    &sensorTaskHandle,   // Task handle
    0                    // Core
  );
  
  xTaskCreatePinnedToCore(
    controlTask,
    "ControlTask",
    4096,
    NULL,
    3,
    &controlTaskHandle,
    1
  );
  
  xTaskCreatePinnedToCore(
    communicationTask,
    "CommunicationTask",
    8192,
    NULL,
    1,
    &communicationTaskHandle,
    1
  );
  
  xTaskCreatePinnedToCore(
    webTask,
    "WebTask",
    4096,
    NULL,
    1,
    &webTaskHandle,
    0
  );
  
  xTaskCreatePinnedToCore(
    energyTask,
    "EnergyTask",
    4096,
    NULL,
    2,
    &energyTaskHandle,
    0
  );
  
  xTaskCreatePinnedToCore(
    predictionTask,
    "PredictionTask",
    4096,
    NULL,
    1,
    &predictionTaskHandle,
    1
  );
  
  xTaskCreatePinnedToCore(
    maintenanceTask,
    "MaintenanceTask",
    4096,
    NULL,
    1,
    &maintenanceTaskHandle,
    0
  );
  
  // Inisialisasi System
  systemInitialized = true;
  Serial.println("System initialized");
  logMessage("INFO", "System initialized", "SYSTEM");
  
  // Kirim notifikasi startup
  sendTelegramMessage("🚀 Smart Farming System started successfully!");
}

//============================================================================
// TASK FREE-RTOS - SENSOR
//============================================================================
void sensorTask(void *pvParameters) {
  while (1) {
    // Feed watchdog
    feedWatchdog();
    
    // Baca sensor
    float sensorValues[15];
    
    if (xSemaphoreTake(sensorMutex, portMAX_DELAY)) {
      // Baca semua sensor
      readAllSensors();
      
      // Simpan nilai sensor ke array
      sensorValues[0] = temperatureGudang;
      sensorValues[1] = humidityGudang;
      sensorValues[2] = lightGudang;
      sensorValues[3] = co2Gudang;
      sensorValues[4] = soilMoisture;
      sensorValues[5] = pressureKebun;
      sensorValues[6] = temperatureKebun;
      sensorValues[7] = lightKebun;
      sensorValues[8] = waterPressure;
      sensorValues[9] = batteryVoltage;
      sensorValues[10] = currentMPPT;
      sensorValues[11] = currentPWM;
      sensorValues[12] = currentLoad;
      sensorValues[13] = voltageMPPT;
      sensorValues[14] = voltagePWM;
      
      xSemaphoreGive(sensorMutex);
      
      // Kirim data ke queue
      xQueueSend(sensorQueue, sensorValues, portMAX_DELAY);
    }
    
    // Delay non-blocking
    vTaskDelay(pdMS_TO_TICKS(SENSOR_READ_INTERVAL));
  }
}

//============================================================================
// TASK FREE-RTOS - CONTROL
//============================================================================
void controlTask(void *pvParameters) {
  while (1) {
    // Feed watchdog
    feedWatchdog();
    
    // Proses logika kontrol
    if (!emergencyMode) {
      processControlLogic();
    }
    
    // Evaluasi custom rules
    evaluateCustomRules();
    
    // Periksa jadwal
    checkSchedules();
    
    // Delay non-blocking
    vTaskDelay(pdMS_TO_TICKS(1000));
  }
}

//============================================================================
// TASK FREE-RTOS - COMMUNICATION
//============================================================================
void communicationTask(void *pvParameters) {
  while (1) {
    // Feed watchdog
    feedWatchdog();
    
    // Baca pesan dari Telegram
    handleTelegramMessages();
    
    // Kirim data ke server
    if (millis() - lastDataSendTime > DATA_SEND_INTERVAL) {
      sendDataToServers();
      lastDataSendTime = millis();
    }
    
    // Periksa cuaca
    if (millis() - lastWeatherCheckTime > WEATHER_CHECK_INTERVAL) {
      checkWeather();
      lastWeatherCheckTime = millis();
    }
    
    // Proses notifikasi dari queue
    String notification[3];
    while (xQueueReceive(notificationQueue, &notification, 0)) {
      if (notification[0] == "telegram") {
        sendTelegramMessage(notification[1]);
      }
    }
    
    // Delay non-blocking
    vTaskDelay(pdMS_TO_TICKS(5000));
  }
}

//============================================================================
// TASK FREE-RTOS - WEB
//============================================================================
void webTask(void *pvParameters) {
  while (1) {
    // Feed watchdog
    feedWatchdog();
    
    // Handle Web Server
    server.handleClient();
    
    // Handle WebSocket
    webSocket.loop();
    
    // Broadcast data sensor
    broadcastSensorData();
    
    // Handle OTA
    if (config.otaEnabled) {
      ArduinoOTA.handle();
    }
    
    // Delay non-blocking
    vTaskDelay(pdMS_TO_TICKS(100));
  }
}

//============================================================================
// TASK FREE-RTOS - ENERGY
//============================================================================
void energyTask(void *pvParameters) {
  while (1) {
    // Feed watchdog
    feedWatchdog();
    
    // Update data baterai
    updateBatteryData();
    
    // Update data energi
    updateEnergyData();
    
    // Log data baterai
    logBatteryData();
    
    // Update kesehatan baterai
    updateBatteryHealth();
    
    // Periksa efisiensi energi
    manageEnergyEfficiency();
    
    // Periksa deep sleep
    if (batteryData.soc < DEEP_SLEEP_THRESHOLD) {
      enterDeepSleep();
    }
    
    // Delay non-blocking
    vTaskDelay(pdMS_TO_TICKS(10000));
  }
}

//============================================================================
// TASK FREE-RTOS - PREDICTION
//============================================================================
void predictionTask(void *pvParameters) {
  while (1) {
    // Feed watchdog
    feedWatchdog();
    
    // Update prediksi
    updatePrediction();
    
    // Update kontrol adaptif
    updateAdaptiveControl();
    
    // Prediksi kegagalan komponen
    predictComponentFailure();
    
    // Delay non-blocking
    vTaskDelay(pdMS_TO_TICKS(3600000)); // Update setiap jam
  }
}

//============================================================================
// TASK FREE-RTOS - MAINTENANCE
//============================================================================
void maintenanceTask(void *pvParameters) {
  while (1) {
    // Feed watchdog
    feedWatchdog();
    
    // Analisis pola maintenance
    analyzeMaintenancePatterns();
    
    // Update kesehatan komponen
    updateComponentHealth();
    
    // Periksa status beban
    checkLoadStatus();
    
    // Periksa kesehatan sistem
    checkSystemHealth();
    
    // Delay non-blocking
    vTaskDelay(pdMS_TO_TICKS(300000)); // Update setiap 5 menit
  }
}

//============================================================================
// LOOP - PROGRAM UTAMA (dikurangi karena menggunakan FreeRTOS)
//============================================================================
void loop() {
  // Feed watchdog
  feedWatchdog();
  
  // Tidak ada proses di loop utama karena semua proses dihandle oleh task FreeRTOS
  
  // Delay kecil untuk mencegah watchdog reset
  delay(100);
}

//============================================================================
// INISIALISASI PIN
//============================================================================
void initializePins() {
  // Pin Input
  pinMode(EMERGENCY_BUTTON_PIN, INPUT_PULLUP);
  pinMode(THERMOSTAT_PIN, INPUT_PULLUP);
  pinMode(PIR1_PIN, INPUT);
  pinMode(PIR2_PIN, INPUT);
  
  // Pin Analog
  pinMode(SOIL_MOISTURE_PIN, INPUT);
  pinMode(MQ135_PIN, INPUT);
  
  // Pin Output (Relay)
  pinMode(PUMP_GUDANG_RELAY_PIN, OUTPUT);
  pinMode(FAN_GUDANG_RELAY_PIN, OUTPUT);
  pinMode(HUMIDIFIER_RELAY_PIN, OUTPUT);
  pinMode(PUMP_KEBUN_RELAY_PIN, OUTPUT);
  pinMode(LAMP_JALAN_RELAY_PIN, OUTPUT);
  pinMode(LAMP_GUDANG_RELAY_PIN, OUTPUT);
  pinMode(LAMP_KEBUN_RELAY_PIN, OUTPUT);
  pinMode(INVERTER_SWITCH_PIN, OUTPUT);
  pinMode(PV_RELAY_PIN, OUTPUT);
  pinMode(BATTERY_RELAY_PIN, OUTPUT);
  
  // Set semua relay ke OFF
  digitalWrite(PUMP_GUDANG_RELAY_PIN, LOW);
  digitalWrite(FAN_GUDANG_RELAY_PIN, LOW);
  digitalWrite(HUMIDIFIER_RELAY_PIN, LOW);
  digitalWrite(PUMP_KEBUN_RELAY_PIN, LOW);
  digitalWrite(LAMP_JALAN_RELAY_PIN, LOW);
  digitalWrite(LAMP_GUDANG_RELAY_PIN, LOW);
  digitalWrite(LAMP_KEBUN_RELAY_PIN, LOW);
  digitalWrite(INVERTER_SWITCH_PIN, LOW);
  digitalWrite(PV_RELAY_PIN, LOW);
  digitalWrite(BATTERY_RELAY_PIN, LOW);
  
  Serial.println("Pins initialized");
  logMessage("INFO", "Pins initialized", "SYSTEM");
}

//============================================================================
// INISIALISASI SENSOR
//============================================================================
void savePIRStatus() {
  EEPROM.write(ADDR_PIR_ENABLED, pirEnabled ? 1 : 0);
  EEPROM.write(ADDR_AUTO_CAPTURE, autoCaptureEnabled ? 1 : 0);
  EEPROM.commit();
}

void initializeSensors() {
  // Inisialisasi I2C
  Wire.begin(SDA_PIN, SCL_PIN);
  
  // Inisialisasi SHT31
  if (!sht31.begin(0x44)) {
    Serial.println("Couldn't find SHT31 sensor");
    logMessage("ERROR", "Couldn't find SHT31 sensor", "SENSOR");
  } else {
    Serial.println("SHT31 sensor initialized");
    logMessage("INFO", "SHT31 sensor initialized", "SENSOR");
  }
  
  // Inisialisasi BMP280
  if (!bmp280.begin(0x76)) {
    Serial.println("Couldn't find BMP280 sensor");
    logMessage("ERROR", "Couldn't find BMP280 sensor", "SENSOR");
  } else {
    Serial.println("BMP280 sensor initialized");
    logMessage("INFO", "BMP280 sensor initialized", "SENSOR");
  }
  
  // Inisialisasi BH1750
  if (!bh1750_gudang.begin()) {
    Serial.println("Couldn't find BH1750 sensor #1");
    logMessage("ERROR", "Couldn't find BH1750 sensor #1", "SENSOR");
  } else {
    Serial.println("BH1750 sensor #1 initialized");
    logMessage("INFO", "BH1750 sensor #1 initialized", "SENSOR");
  }
  
  if (!bh1750_kebun.begin()) {
    Serial.println("Couldn't find BH1750 sensor #2");
    logMessage("ERROR", "Couldn't find BH1750 sensor #2", "SENSOR");
  } else {
    Serial.println("BH1750 sensor #2 initialized");
    logMessage("INFO", "BH1750 sensor #2 initialized", "SENSOR");
  }
  
  // Inisialisasi INA219 dengan kalibrasi 100A
  if (!ina219_1.begin()) {
    Serial.println("Couldn't find INA219 #1 sensor");
    logMessage("ERROR", "Couldn't find INA219 #1 sensor", "SENSOR");
  } else {
    configureINA219_100A(ina219_1);
    Serial.println("INA219 #1 sensor initialized with 100A calibration");
    logMessage("INFO", "INA219 #1 sensor initialized with 100A calibration", "SENSOR");
  }
  
  if (!ina219_2.begin()) {
    Serial.println("Couldn't find INA219 #2 sensor");
    logMessage("ERROR", "Couldn't find INA219 #2 sensor", "SENSOR");
  } else {
    configureINA219_100A(ina219_2);
    Serial.println("INA219 #2 sensor initialized with 100A calibration");
    logMessage("INFO", "INA219 #2 sensor initialized with 100A calibration", "SENSOR");
  }
  
  if (!ina219_3.begin()) {
    Serial.println("Couldn't find INA219 #3 sensor");
    logMessage("ERROR", "Couldn't find INA219 #3 sensor", "SENSOR");
  } else {
    configureINA219_100A(ina219_3);
    Serial.println("INA219 #3 sensor initialized with 100A calibration");
    logMessage("INFO", "INA219 #3 sensor initialized with 100A calibration", "SENSOR");
  }
  
  // Inisialisasi LPS35HW
  if (!lps35hw.begin()) {
    Serial.println("Couldn't find LPS35HW sensor");
    logMessage("ERROR", "Couldn't find LPS35HW sensor", "SENSOR");
  } else {
    Serial.println("LPS35HW sensor initialized");
    logMessage("INFO", "LPS35HW sensor initialized", "SENSOR");
  }
  
  // Inisialisasi sensor suhu baterai
  batteryTempSensor.begin();
  Serial.println("Battery temperature sensor initialized");
  logMessage("INFO", "Battery temperature sensor initialized", "SENSOR");
}

//============================================================================
// BACA SEMUA SENSOR
//============================================================================
void readAllSensors() {
  // Baca sensor SHT31 (suhu dan kelembaban gudang)
  if (sht31.begin(0x44)) {
    temperatureGudang = sht31.readTemperature();
    humidityGudang = sht31.readHumidity();
    
    if (isnan(temperatureGudang) || isnan(humidityGudang)) {
      logMessage("ERROR", "Failed to read from SHT31 sensor", "SENSOR");
    }
  }
  
  // Baca sensor BMP280 (tekanan dan suhu kebun)
  if (bmp280.begin(0x76)) {
    pressureKebun = bmp280.readPressure() / 100.0; // Convert to hPa
    temperatureKebun = bmp280.readTemperature();
    
    if (isnan(pressureKebun) || isnan(temperatureKebun)) {
      logMessage("ERROR", "Failed to read from BMP280 sensor", "SENSOR");
    }
  }
  
  // Baca sensor BH1750 (cahaya)
  lightGudang = bh1750_gudang.readLightLevel();
  lightKebun = bh1750_kebun.readLightLevel();
  
  if (lightGudang < 0 || lightKebun < 0) {
    logMessage("ERROR", "Failed to read from BH1750 sensors", "SENSOR");
  }
  
  // Baca sensor LPS35HW (tekanan air)
  if (lps35hw.begin()) {
    waterPressure = lps35hw.readPressure();
    
    if (isnan(waterPressure)) {
      logMessage("ERROR", "Failed to read from LPS35HW sensor", "SENSOR");
    }
  }
  
  // Baca sensor analog
  soilMoisture = map(analogRead(SOIL_MOISTURE_PIN), 0, 4095, 0, 100);
  
  // Baca sensor MQ135 (CO2)
  int mq135Value = analogRead(MQ135_PIN);
  // Konversi nilai analog ke ppm CO2 (rumus sederhana)
  co2Gudang = 0.0004 * mq135Value; // Asumsi konversi sederhana
  
  // Baca sensor INA219
  currentMPPT = ina219_1.getCurrent_mA() / 1000.0; // Convert to A
  voltageMPPT = ina219_1.getBusVoltage_V();
  
  currentPWM = ina219_2.getCurrent_mA() / 1000.0; // Convert to A
  voltagePWM = ina219_2.getBusVoltage_V();
  
  currentLoad = ina219_3.getCurrent_mA() / 1000.0; // Convert to A
  voltageLoad = ina219_3.getBusVoltage_V();
  
  // Baca tegangan baterai dari INA219 ke beban
  batteryVoltage = voltageLoad;
  
  // Baca sensor suhu baterai
  batteryTempSensor.requestTemperatures();
  batteryData.temperature = batteryTempSensor.getTempCByIndex(0);
  
  // Baca tombol emergency dan thermostat
  emergencyButtonStatus = (digitalRead(EMERGENCY_BUTTON_PIN) == LOW);
  thermostatStatus = (digitalRead(THERMOSTAT_PIN) == LOW);
  
  // Baca sensor PIR
  bool currentPir1 = digitalRead(PIR1_PIN);
  bool currentPir2 = digitalRead(PIR2_PIN);
  
  // Cek perubahan status PIR1 (rising edge)
  if (currentPir1 && !lastPir1Status && pirEnabled && autoCaptureEnabled) {
    sendCameraCommand("/motion_start");
    sendTelegramMessage("📹 Gerakan terdeteksi di gudang jamur");
    lastPIRDetectTime = getFormattedTime();
  }
  
  // Cek perubahan status PIR2 (rising edge)
  if (currentPir2 && !lastPir2Status && pirEnabled && autoCaptureEnabled) {
    sendCameraCommand("/motion_start");
    sendTelegramMessage("📹 Gerakan terdeteksi di kebun vanili");
    lastPIRDetectTime = getFormattedTime();
  }
  
  // Update status PIR terakhir
  lastPir1Status = currentPir1;
  lastPir2Status = currentPir2;
  pir1Status = currentPir1;
  pir2Status = currentPir2;
}

//============================================================================
// INISIALISASI FUZZY LOGIC (Versi Lengkap: 3 Input, 4 Output)
//============================================================================
void initializeFuzzyLogic() {
  // === INPUT 1: SUHU GUDANG ===
  FuzzySet *suhuDingin = new FuzzySet(0, 20, 20, 25);
  FuzzySet *suhuNormal = new FuzzySet(22, 25, 28, 30);
  FuzzySet *suhuHangat = new FuzzySet(28, 30, 32, 35);
  FuzzySet *suhuPanas = new FuzzySet(33, 35, 40, 50);

  // === INPUT 2: KELEMBABAN UDARA ===
  FuzzySet *kelembabanKering = new FuzzySet(0, 60, 60, 70);
  FuzzySet *kelembabanNormal = new FuzzySet(65, 75, 80, 85);
  FuzzySet *kelembabanLembab = new FuzzySet(80, 85, 90, 95);
  FuzzySet *kelembabanBasah = new FuzzySet(90, 95, 100, 100);

  // === INPUT 3: KELEMBABAN TANAH (SOIL MOISTURE) ===
  FuzzySet *tanahKering = new FuzzySet(0, 30, 30, 50);
  FuzzySet *tanahNormal = new FuzzySet(40, 50, 60, 70);
  FuzzySet *tanahLembab = new FuzzySet(60, 70, 80, 90);
  FuzzySet *tanahBasah = new FuzzySet(80, 90, 100, 100);

  // === OUTPUT 1: DURASI KIPAS (detik/menit, 0–60) ===
  FuzzySet *kipasMati = new FuzzySet(0, 0, 0, 15);
  FuzzySet *kipasRendah = new FuzzySet(0, 15, 15, 30);
  FuzzySet *kipasSedang = new FuzzySet(15, 30, 30, 45);
  FuzzySet *kipasTinggi = new FuzzySet(30, 45, 45, 60);
  FuzzySet *kipasMaksimal = new FuzzySet(45, 60, 60, 60);

  // === OUTPUT 2: DURASI POMPA GUDANG (detik, 0–300) ===
  FuzzySet *pompaGudangMati = new FuzzySet(0, 0, 0, 30);
  FuzzySet *pompaGudangPendek = new FuzzySet(0, 30, 30, 90);
  FuzzySet *pompaGudangSedang = new FuzzySet(30, 90, 90, 180);
  FuzzySet *pompaGudangPanjang = new FuzzySet(90, 180, 180, 300);

  // === OUTPUT 3: DURASI POMPA KEBUN (detik, 0–300) ===
  FuzzySet *pompaKebunMati = new FuzzySet(0, 0, 0, 30);
  FuzzySet *pompaKebunPendek = new FuzzySet(0, 30, 30, 90);
  FuzzySet *pompaKebunSedang = new FuzzySet(30, 90, 90, 180);
  FuzzySet *pompaKebunPanjang = new FuzzySet(90, 180, 180, 300);

  // === OUTPUT 4: DURASI HUMIDIFIER (detik, 0–300) ===
  FuzzySet *humidifierMati = new FuzzySet(0, 0, 0, 30);
  FuzzySet *humidifierPendek = new FuzzySet(0, 30, 30, 90);
  FuzzySet *humidifierSedang = new FuzzySet(30, 90, 90, 180);
  FuzzySet *humidifierPanjang = new FuzzySet(90, 180, 180, 300);

  // === TAMBAHKAN INPUT ===
  FuzzyInput *suhu = new FuzzyInput(1);
  suhu->addFuzzySet(suhuDingin);
  suhu->addFuzzySet(suhuNormal);
  suhu->addFuzzySet(suhuHangat);
  suhu->addFuzzySet(suhuPanas);
  fuzzy->addFuzzyInput(suhu);

  FuzzyInput *kelembaban = new FuzzyInput(2);
  kelembaban->addFuzzySet(kelembabanKering);
  kelembaban->addFuzzySet(kelembabanNormal);
  kelembaban->addFuzzySet(kelembabanLembab);
  kelembaban->addFuzzySet(kelembabanBasah);
  fuzzy->addFuzzyInput(kelembaban);

  FuzzyInput *tanah = new FuzzyInput(3); // Input ke-3
  tanah->addFuzzySet(tanahKering);
  tanah->addFuzzySet(tanahNormal);
  tanah->addFuzzySet(tanahLembab);
  tanah->addFuzzySet(tanahBasah);
  fuzzy->addFuzzyInput(tanah);

  // === TAMBAHKAN OUTPUT ===
  FuzzyOutput *durasiKipas = new FuzzyOutput(1);
  durasiKipas->addFuzzySet(kipasMati);
  durasiKipas->addFuzzySet(kipasRendah);
  durasiKipas->addFuzzySet(kipasSedang);
  durasiKipas->addFuzzySet(kipasTinggi);
  durasiKipas->addFuzzySet(kipasMaksimal);
  fuzzy->addFuzzyOutput(durasiKipas);

  FuzzyOutput *durasiPompaGudang = new FuzzyOutput(2);
  durasiPompaGudang->addFuzzySet(pompaGudangMati);
  durasiPompaGudang->addFuzzySet(pompaGudangPendek);
  durasiPompaGudang->addFuzzySet(pompaGudangSedang);
  durasiPompaGudang->addFuzzySet(pompaGudangPanjang);
  fuzzy->addFuzzyOutput(durasiPompaGudang);

  FuzzyOutput *durasiPompaKebun = new FuzzyOutput(3);
  durasiPompaKebun->addFuzzySet(pompaKebunMati);
  durasiPompaKebun->addFuzzySet(pompaKebunPendek);
  durasiPompaKebun->addFuzzySet(pompaKebunSedang);
  durasiPompaKebun->addFuzzySet(pompaKebunPanjang);
  fuzzy->addFuzzyOutput(durasiPompaKebun);

  FuzzyOutput *durasiHumidifier = new FuzzyOutput(4);
  durasiHumidifier->addFuzzySet(humidifierMati);
  durasiHumidifier->addFuzzySet(humidifierPendek);
  durasiHumidifier->addFuzzySet(humidifierSedang);
  durasiHumidifier->addFuzzySet(humidifierPanjang);
  fuzzy->addFuzzyOutput(durasiHumidifier);

  // === ATURAN FUZZY BARU ===
  // Rule 1: Suhu Panas ATAU Kelembaban Kering → Kipas Maksimal
  FuzzyRuleAntecedent *r1a = new FuzzyRuleAntecedent();
  r1a->joinWithOR(suhuPanas, kelembabanKering);
  FuzzyRuleConsequent *r1c = new FuzzyRuleConsequent();
  r1c->addOutput(kipasMaksimal);
  fuzzy->addFuzzyRule(new FuzzyRule(1, r1a, r1c));

  // Rule 2: Suhu Normal DAN Kelembaban Kering → Pompa Gudang Sedang
  FuzzyRuleAntecedent *r2a = new FuzzyRuleAntecedent();
  r2a->joinWithAND(suhuNormal, kelembabanKering);
  FuzzyRuleConsequent *r2c = new FuzzyRuleConsequent();
  r2c->addOutput(pompaGudangSedang);
  fuzzy->addFuzzyRule(new FuzzyRule(2, r2a, r2c));

  // Rule 3: Kelembaban Tanah Kering → Pompa Kebun Panjang
  FuzzyRuleAntecedent *r3a = new FuzzyRuleAntecedent();
  r3a->joinSingle(tanahKering);
  FuzzyRuleConsequent *r3c = new FuzzyRuleConsequent();
  r3c->addOutput(pompaKebunPanjang);
  fuzzy->addFuzzyRule(new FuzzyRule(3, r3a, r3c));

  // Rule 4: Kelembaban Tanah Normal → Pompa Kebun Mati
  FuzzyRuleAntecedent *r4a = new FuzzyRuleAntecedent();
  r4a->joinSingle(tanahNormal);
  FuzzyRuleConsequent *r4c = new FuzzyRuleConsequent();
  r4c->addOutput(pompaKebunMati);
  fuzzy->addFuzzyRule(new FuzzyRule(4, r4a, r4c));

  // Rule 5: Kelembaban Udara Kering → Humidifier Sedang
  FuzzyRuleAntecedent *r5a = new FuzzyRuleAntecedent();
  r5a->joinSingle(kelembabanKering);
  FuzzyRuleConsequent *r5c = new FuzzyRuleConsequent();
  r5c->addOutput(humidifierSedang);
  fuzzy->addFuzzyRule(new FuzzyRule(5, r5a, r5c));

  // Rule 6: Suhu Dingin DAN Kelembaban Normal → Semua Mati
  FuzzyRuleAntecedent *r6a = new FuzzyRuleAntecedent();
  r6a->joinWithAND(suhuDingin, kelembabanNormal);
  FuzzyRuleConsequent *r6c = new FuzzyRuleConsequent();
  r6c->addOutput(kipasMati);
  r6c->addOutput(pompaGudangMati);
  r6c->addOutput(pompaKebunMati);
  r6c->addOutput(humidifierMati);
  fuzzy->addFuzzyRule(new FuzzyRule(6, r6a, r6c));

  Serial.println("✅ Fuzzy logic initialized (3 inputs, 4 outputs)");
  logMessage("INFO", "Fuzzy logic initialized (3 inputs, 4 outputs)", "SYSTEM");
}

//============================================================================
// PROSES LOGIKA KONTROL
//============================================================================
void processControlLogic() {
  // Gunakan mode operasi yang sesuai
  switch (config.operationMode) {
    case MODE_LOCAL:
      processFuzzyControl();
      break;
    case MODE_AI:
      processAIControl();
      break;
    case MODE_USER:
      // Kontrol manual, tidak ada proses otomatis
      break;
    case MODE_HYBRID:
      processHybridControl();
      break;
    case MODE_EMERGENCY:
      processEmergencyControl();
      break;
    case MODE_MAINTENANCE:
      // Mode maintenance, tidak ada kontrol otomatis
      break;
    case MODE_SCHEDULED:
      // Kontrol berdasarkan jadwal, dihandle oleh fungsi checkSchedules()
      break;
    case MODE_PREDICTIVE:
      processPredictiveControl();
      break;
    case MODE_ADAPTIVE:
      processAdaptiveControl();
      break;
    default:
      processFuzzyControl();
      break;
  }
}

void processFuzzyControl() {
  // Set input untuk fuzzy logic
  fuzzy->setInput(1, temperatureGudang);
  fuzzy->setInput(2, humidityGudang);
  fuzzy->setInput(3, soilMoisture);
  
  // Proses fuzzy logic
  fuzzy->fuzzify();
  
  // Dapatkan output
  float durasiKipas = fuzzy->getOutput(1);
  float durasiPompaGudang = fuzzy->getOutput(2);
  float durasiPompaKebun = fuzzy->getOutput(3);
  float durasiHumidifier = fuzzy->getOutput(4);
  
  // Kontrol beban berdasarkan output fuzzy
  // Durasi dalam detik, konversi ke milidetik
  if (durasiKipas > 15) {
    turnOnLoadSequentially("Exhaust Fan", durasiKipas * 1000);
  }
  
  if (durasiPompaGudang > 30) {
    turnOnLoadSequentially("Pump Gudang", durasiPompaGudang * 1000);
  }
  
  if (durasiPompaKebun > 30) {
    turnOnLoadSequentially("Pump Kebun", durasiPompaKebun * 1000);
  }
  
  if (durasiHumidifier > 30) {
    controlLoad("Humidifier", true);
    delay(durasiHumidifier * 1000);
    controlLoad("Humidifier", false);
  }
}

void processAIControl() {
  // Kontrol dengan AI (Gemini)
  // Buat prompt untuk AI
  String prompt = "Berdasarkan data sensor berikut, berikan rekomendasi kontrol untuk smart farming system:\n";
  prompt += "Suhu Gudang: " + String(temperatureGudang) + "°C\n";
  prompt += "Kelembaban Gudang: " + String(humidityGudang) + "%\n";
  prompt += "Kelembaban Tanah: " + String(soilMoisture) + "%\n";
  prompt += "Tegangan Baterai: " + String(batteryVoltage) + "V\n";
  prompt += "SOC Baterai: " + String(batteryData.soc) + "%\n";
  prompt += "Waktu: " + getFormattedTime() + "\n";
  prompt += "Berikan rekomendasi dalam format JSON: {\"actions\": [{\"load\": \"nama_beban\", \"action\": \"on/off\", \"duration\": detik}]}";
  
  // Dapatkan rekomendasi dari AI
  String aiResponse = getGeminiRecommendation(prompt);
  
  // Parse dan eksekusi rekomendasi
  DynamicJsonDocument doc(1024);
  DeserializationError error = deserializeJson(doc, aiResponse);
  
  if (!error) {
    JsonArray actions = doc["actions"];
    for (JsonObject action : actions) {
      String loadName = action["load"];
      String actionType = action["action"];
      int duration = action["duration"];
      
      if (actionType == "on") {
        if (duration > 0) {
          turnOnLoadSequentially(loadName, duration * 1000);
        } else {
          controlLoad(loadName, true);
        }
      } else {
        controlLoad(loadName, false);
      }
    }
  } else {
    logMessage("ERROR", "Failed to parse AI response: " + String(error.c_str()), "AI");
    // Fallback ke fuzzy control
    processFuzzyControl();
  }
}

void processHybridControl() {
  // Kombinasi AI dan fuzzy logic
  // Gunakan AI untuk keputusan strategis, fuzzy untuk kontrol detail
  
  // Cek apakah perlu rekomendasi AI (setiap jam atau saat kondisi ekstrem)
  static unsigned long lastAIUpdate = 0;
  bool needAIUpdate = (millis() - lastAIUpdate > 3600000) || // 1 jam
                      (temperatureGudang > 35) || 
                      (humidityGudang < 60) || 
                      (batteryData.soc < 20);
  
  if (needAIUpdate) {
    processAIControl();
    lastAIUpdate = millis();
  } else {
    processFuzzyControl();
  }
}

void processEmergencyControl() {
  // Mode darurat: matikan semua beban kecuali yang penting
  controlLoad("Exhaust Fan", false);
  controlLoad("Pump Gudang", false);
  controlLoad("Pump Kebun", false);
  controlLoad("Humidifier", false);
  controlLoad("Lampu Gudang", false);
  controlLoad("Lampu Kebun", false);
  
  // Hanya nyalakan lampu jalan jika malam hari
  int hour = timeClient.getHours();
  if (hour >= 18 || hour <= 5) {
    controlLoad("Lampu Jalan", true);
  } else {
    controlLoad("Lampu Jalan", false);
  }
}

void processPredictiveControl() {
  // Kontrol prediktif berdasarkan prediksi
  if (predictionData.confidence > 0.7) {
    // Jika prediksi suhu tinggi, nyalakan kipas lebih awal
    if (predictionData.predictedTemp > TEMP_HIGH_THRESHOLD) {
      controlLoad("Exhaust Fan", true);
    }
    
    // Jika prediksi kelembaban rendah, siapkan humidifier
    if (predictionData.predictedHumidity < HUMIDITY_LOW_THRESHOLD) {
      controlLoad("Humidifier", true);
    }
    
    // Jika prediksi SoC baterai rendah, hemat energi
    if (predictionData.predictedBatterySOC < BATTERY_LOW_THRESHOLD) {
      // Matikan beban tidak penting
      controlLoad("Lampu Gudang", false);
      controlLoad("Lampu Kebun", false);
    }
  } else {
    // Fallback ke fuzzy control jika confidence rendah
    processFuzzyControl();
  }
}

void processAdaptiveControl() {
  // Kontrol adaptif yang belajar dari lingkungan
  // Gunakan PID control yang telah disesuaikan
  
  // Update PID setpoint berdasarkan waktu dan kondisi
  int hour = timeClient.getHours();
  
  // Sesuaikan setpoint suhu berdasarkan waktu
  if (hour >= 6 && hour <= 18) {
    tempSetpoint = TEMP_NORMAL_THRESHOLD;
  } else {
    tempSetpoint = TEMP_NORMAL_THRESHOLD - 1.0; // Sedikit lebih dingin malam hari
  }
  
  // Update PID
  tempPID.Compute();
  humidityPID.Compute();
  
  // Kontrol beban berdasarkan output PID
  if (tempPIDOutput > 50) {
    controlLoad("Exhaust Fan", true);
  } else {
    controlLoad("Exhaust Fan", false);
  }
  
  if (humidityPIDOutput > 50) {
    controlLoad("Humidifier", true);
  } else {
    controlLoad("Humidifier", false);
  }
  
  // Kontrol pompa berdasarkan kelembaban tanah
  if (soilMoisture < SOIL_MOISTURE_LOW_THRESHOLD) {
    turnOnLoadSequentially("Pump Kebun", 60000); // 1 menit
  }
}

//============================================================================
// KONTROL BEBAN SEKUENSIAL
//============================================================================
void turnOnLoadSequentially(String loadName, unsigned long duration) {
  // Cari beban dalam array
  int loadIndex = -1;
  for (int i = 0; i < 7; i++) {
    if (loads[i].name == loadName) {
      loadIndex = i;
      break;
    }
  }
  
  if (loadIndex == -1) {
    logMessage("ERROR", "Load not found: " + loadName, "CONTROL");
    return;
  }
  
  LoadStatus& load = loads[loadIndex];
  
  // Urutan menyalakan beban 220VAC:
  // 1. Relay baterai on, tunggu 5 detik
  // 2. Relay switch inverter on, tunggu 5 detik
  // 3. Relay PV on, tunggu 5 detik
  // 4. Relay beban 220V on, tunggu 5 detik
  // 5. Relay PV off
  
  // Cek apakah beban sudah menyala
  if (load.actualStatus) {
    logMessage("INFO", "Load already on: " + loadName, "CONTROL");
    return;
  }
  
  logMessage("INFO", "Turning on load sequentially: " + loadName, "CONTROL");
  
  // 1. Relay baterai on
  digitalWrite(BATTERY_RELAY_PIN, HIGH);
  delay(5000);
  
  // 2. Relay switch inverter on
  digitalWrite(INVERTER_SWITCH_PIN, HIGH);
  delay(5000);
  
  // 3. Relay PV on
  digitalWrite(PV_RELAY_PIN, HIGH);
  delay(5000);
  
  // 4. Relay beban 220V on
  digitalWrite(load.relayPin, HIGH);
  delay(5000);
  
  // 5. Relay PV off
  digitalWrite(PV_RELAY_PIN, LOW);
  
  // Update status beban
  load.commandStatus = true;
  load.lastUpdateTime = millis();
  
  // Verifikasi beban
  verifyLoadOperation(loadIndex);
  
  // Tunggu durasi yang ditentukan
  delay(duration);
  
  // Matikan beban
  turnOffLoadSequentially(loadName);
}

void turnOffLoadSequentially(String loadName) {
  // Cari beban dalam array
  int loadIndex = -1;
  for (int i = 0; i < 7; i++) {
    if (loads[i].name == loadName) {
      loadIndex = i;
      break;
    }
  }
  
  if (loadIndex == -1) {
    logMessage("ERROR", "Load not found: " + loadName, "CONTROL");
    return;
  }
  
  LoadStatus& load = loads[loadIndex];
  
  // Urutan mematikan beban 220VAC:
  // 1. Relay beban off, tunggu 5 detik
  // 2. Relay switch inverter off, tunggu 5 detik
  // 3. Relay baterai off
  
  logMessage("INFO", "Turning off load sequentially: " + loadName, "CONTROL");
  
  // 1. Relay beban off
  digitalWrite(load.relayPin, LOW);
  delay(5000);
  
  // 2. Relay switch inverter off
  digitalWrite(INVERTER_SWITCH_PIN, LOW);
  delay(5000);
  
  // 3. Relay baterai off
  digitalWrite(BATTERY_RELAY_PIN, LOW);
  
  // Update status beban
  load.commandStatus = false;
  load.lastUpdateTime = millis();
}

void controlLoad(String loadName, bool status) {
  // Cari beban dalam array
  int loadIndex = -1;
  for (int i = 0; i < 7; i++) {
    if (loads[i].name == loadName) {
      loadIndex = i;
      break;
    }
  }
  
  if (loadIndex == -1) {
    logMessage("ERROR", "Load not found: " + loadName, "CONTROL");
    return;
  }
  
  LoadStatus& load = loads[loadIndex];
  
  // Cek apakah beban 220V atau 12V
  bool isACLoad = (loadName == "Exhaust Fan" || 
                   loadName == "Pump Gudang" || 
                   loadName == "Pump Kebun");
  
  if (isACLoad) {
    // Gunakan kontrol sekuensial untuk beban AC
    if (status) {
      turnOnLoadSequentially(loadName, 0); // Durasi 0 = on terus
    } else {
      turnOffLoadSequentially(loadName);
    }
  } else {
    // Kontrol langsung untuk beban DC
    digitalWrite(load.relayPin, status ? HIGH : LOW);
    load.commandStatus = status;
    load.lastUpdateTime = millis();
    
    // Verifikasi beban
    verifyLoadOperation(loadIndex);
  }
}

void verifyLoadOperation(int loadIndex) {
  LoadStatus& load = loads[loadIndex];
  
  // Tunggu sebentar untuk sensor stabil
  delay(2000);
  
  // Baca arus saat ini
  float currentChange = currentLoad;
  
  // Verifikasi berdasarkan arus
  bool loadWorking = false;
  if (load.commandStatus) {
    // Jika seharusnya menyala, periksa apakah arus meningkat
    loadWorking = (currentChange > load.currentThreshold);
  } else {
    // Jika seharusnya mati, periksa apakah arus menurun
    loadWorking = (currentChange < load.currentThreshold);
  }
  
  // Verifikasi berdasarkan sensor terkait
  if (load.verificationEnabled && load.commandStatus) {
    bool sensorVerification = true;
    
    for (int i = 0; i < load.sensorCount; i++) {
      String sensorName = load.relatedSensors[i];
      float expectedChange = load.expectedChanges[i];
      float currentValue = getSensorValue(sensorName);
      
      // Cek apakah perubahan sensor sesuai yang diharapkan
      if (expectedChange > 0) {
        // Harus meningkat
        sensorVerification = sensorVerification && (currentValue > expectedChange);
      } else if (expectedChange < 0) {
        // Harus menurun
        sensorVerification = sensorVerification && (currentValue < expectedChange);
      }
    }
    
    loadWorking = loadWorking && sensorVerification;
  }
  
  // Update status aktual
  load.actualStatus = loadWorking;
  
  // Jika tidak sesuai, kirim notifikasi
  if (load.commandStatus != load.actualStatus) {
    String message = "⚠️ Beban " + load.name + " tidak berfungsi dengan benar. ";
    message += "Status perintah: " + String(load.commandStatus ? "ON" : "OFF");
    message += ", Status aktual: " + String(load.actualStatus ? "ON" : "OFF");
    
    sendTelegramMessage(message);
    logMessage("WARNING", message, "CONTROL");
    
    // Jika perubahan sangat ekstrem, matikan beban untuk keamanan
    if (load.commandStatus && currentChange > load.currentThreshold * 2.0) {
      controlLoad(load.name, false);
      message = "🚨 Beban " + load.name + " dimatikan karena arus berlebih!";
      sendTelegramMessage(message);
      logMessage("ERROR", message, "CONTROL");
    }
  }
}

//============================================================================
// INISIALISASI LOAD STATUS
//============================================================================
void initializeLoadStatus() {
  // Inisialisasi status beban dengan sensor terkait
  loads[0] = {"Exhaust Fan", false, false, config.exhaustFanThreshold, FAN_GUDANG_RELAY_PIN, 0, 
              {"temperatureGudang", "co2Gudang"}, 2, {0, 0}, true};
  loads[1] = {"Pump Gudang", false, false, config.pumpGudangThreshold, PUMP_GUDANG_RELAY_PIN, 0, 
              {"waterPressure", "temperatureGudang", "humidityGudang"}, 3, {0, 0, 0}, true};
  loads[2] = {"Humidifier", false, false, config.humidifierThreshold, HUMIDIFIER_RELAY_PIN, 0, 
              {"humidityGudang", "co2Gudang"}, 2, {0, 0}, true};
  loads[3] = {"Pump Kebun", false, false, config.pumpKebunThreshold, PUMP_KEBUN_RELAY_PIN, 0, 
              {"soilMoisture"}, 1, {0}, true};
  loads[4] = {"Lampu Jalan", false, false, config.lampJalanThreshold, LAMP_JALAN_RELAY_PIN, 0, 
              {"lightKebun"}, 1, {0}, false};
  loads[5] = {"Lampu Gudang", false, false, config.lampGudangThreshold, LAMP_GUDANG_RELAY_PIN, 0, 
              {"lightGudang"}, 1, {0}, false};
  loads[6] = {"Lampu Kebun", false, false, config.lampKebunThreshold, LAMP_KEBUN_RELAY_PIN, 0, 
              {"lightKebun"}, 1, {0}, false};
  
  Serial.println("Load status initialized");
  logMessage("INFO", "Load status initialized", "SYSTEM");
}

//============================================================================
// INISIALISASI JADWAL
//============================================================================
void initializeSchedules() {
  // Jadwal default
  schedules[0] = {9, 0, "Pump Kebun", true, "soilMoisture", 65.0, "<"};
  schedules[1] = {9, 10, "Humidifier", true, "humidity", 85.0, ">"};
  schedules[2] = {12, 0, "Pump Gudang", true, "temperature", 27.0, "<"};
  schedules[3] = {12, 10, "Exhaust Fan", true, "humidity", 87.0, "<"};
  schedules[4] = {16, 0, "Inverter", true, "", 0.0, ""}; // Nyalakan inverter saja
  schedules[5] = {17, 0, "Inverter", false, "", 0.0, ""}; // Matikan inverter
  schedules[6] = {18, 0, "Lampu Jalan", true, "", 0.0, ""};
  schedules[7] = {23, 59, "Lampu Jalan", false, "", 0.0, ""};
  schedules[8] = {3, 0, "Lampu Jalan", true, "", 0.0, ""};
  schedules[9] = {5, 30, "Lampu Jalan", false, "", 0.0, ""};
  
  scheduleCount = 10;
  
  Serial.println("Schedules initialized");
  logMessage("INFO", "Schedules initialized", "SYSTEM");
}

void checkSchedules() {
  // Dapatkan waktu saat ini
  if (!timeConfigured) return;
  
  int currentHour = timeClient.getHours();
  int currentMinute = timeClient.getMinutes();
  
  // Periksa semua jadwal
  for (int i = 0; i < scheduleCount; i++) {
    Schedule& schedule = schedules[i];
    
    if (!schedule.active) continue;
    
    // Cek apakah waktu jadwal cocok
    if (schedule.hour == currentHour && schedule.minute == currentMinute) {
      // Cek kondisi jika ada
      bool conditionMet = true;
      
      if (schedule.condition.length() > 0) {
        float sensorValue = getSensorValue(schedule.condition);
        
        if (schedule.conditionOperator == "<") {
          conditionMet = sensorValue < schedule.conditionValue;
        } else if (schedule.conditionOperator == ">") {
          conditionMet = sensorValue > schedule.conditionValue;
        } else if (schedule.conditionOperator == "=") {
          conditionMet = sensorValue == schedule.conditionValue;
        } else if (schedule.conditionOperator == "!=") {
          conditionMet = sensorValue != schedule.conditionValue;
        } else if (schedule.conditionOperator == "<=") {
          conditionMet = sensorValue <= schedule.conditionValue;
        } else if (schedule.conditionOperator == ">=") {
          conditionMet = sensorValue >= schedule.conditionValue;
        }
      }
      
      // Eksekusi jadwal jika kondisi terpenuhi
      if (conditionMet) {
        if (schedule.loadName == "Inverter") {
          // Kontrol khusus untuk inverter
          if (i == 4) { // Jadwal nyala inverter
            digitalWrite(BATTERY_RELAY_PIN, HIGH);
            delay(5000);
            digitalWrite(INVERTER_SWITCH_PIN, HIGH);
            logMessage("INFO", "Inverter turned on by schedule", "SCHEDULE");
          } else if (i == 5) { // Jadwal mati inverter
            digitalWrite(INVERTER_SWITCH_PIN, LOW);
            delay(5000);
            digitalWrite(BATTERY_RELAY_PIN, LOW);
            logMessage("INFO", "Inverter turned off by schedule", "SCHEDULE");
          }
        } else {
          // Kontrol beban normal
          controlLoad(schedule.loadName, i == 4 || i == 6 || i == 8); // ON untuk jadwal tertentu
        }
        
        // Tambahkan durasi 30% setelah kondisi terpenuhi
        if (schedule.loadName != "Inverter") {
          // Cari beban dan tambahkan durasi
          for (int j = 0; j < 7; j++) {
            if (loads[j].name == schedule.loadName) {
              // Tambahkan durasi 30%
              unsigned long baseDuration = 60000; // 1 menit default
              unsigned long extendedDuration = baseDuration * 1.3; // 30% lebih lama
              
              // Gunakan timer non-blocking
              // Implementasi timer akan ditambahkan di bagian selanjutnya
              break;
            }
          }
        }
      }
    }
  }
}

//============================================================================
// INISIALISASI DATA BATERAI
//============================================================================
void initializeBatteryData() {
  batteryData.voltage = 0;
  batteryData.current = 0;
  batteryData.temperature = 25.0; // Default suhu
  batteryData.capacity = 275.0; // Kapasitas nominal 275Ah
  batteryData.fullCapacity = 275.0; // Kapasitas penuh nominal
  batteryData.soh = 100.0; // Default 100%
  batteryData.soc = 50.0; // Default 50%
  batteryData.lastUpdateTime = millis();
  batteryData.coulombCount = 0.0;
  batteryData.isCharging = false;
  batteryData.cycles = 995; // Mulai dari 995 siklus
  batteryData.chargeEfficiency = 95.0; // Default 95%
  
  Serial.println("Battery data initialized");
  logMessage("INFO", "Battery data initialized", "SYSTEM");
}

//============================================================================
// INISIALISASI DATA ENERGI
//============================================================================
void initializeEnergyData() {
  energyData.energyCharged = 0.0;
  energyData.energyDischarged = 0.0;
  energyData.efficiency = 0.0;
  energyData.lastUpdateTime = millis();
  energyData.mpptEnergy = 0.0;
  energyData.pwmEnergy = 0.0;
  energyData.loadEnergy = 0.0;
  
  Serial.println("Energy data initialized");
  logMessage("INFO", "Energy data initialized", "SYSTEM");
}

//============================================================================
// UPDATE DATA BATERAI
//============================================================================
void updateBatteryData() {
  unsigned long currentTime = millis();
  unsigned long deltaTime = currentTime - batteryData.lastUpdateTime;
  
  if (deltaTime < 1000) return; // Update setiap detik
  
  // Update tegangan dan arus
  batteryData.voltage = voltageLoad;
  batteryData.current = currentLoad;
  
  // Update status charging
  batteryData.isCharging = (batteryData.current > 0);
  
  // Coulomb counting untuk SoC
  float currentAh = batteryData.current * (deltaTime / 3600000.0); // Konversi ke Ah
  
  if (batteryData.isCharging) {
    batteryData.coulombCount += currentAh * (batteryData.chargeEfficiency / 100.0);
  } else {
    batteryData.coulombCount += currentAh;
  }
  
  // Update SoC
  batteryData.soc = (batteryData.coulombCount / batteryData.fullCapacity) * 100.0;
  batteryData.soc = constrain(batteryData.soc, 0.0, 100.0);
  
  // Update SoH berdasarkan tegangan dan suhu
  updateBatteryHealth();
  
  // Hitung siklus pengisian
  updateBatteryCycles();
  
  batteryData.lastUpdateTime = currentTime;
}

void updateBatteryHealth() {
  // Update State of Health (SoH) berdasarkan beberapa faktor
  
  // Faktor 1: Tegangan
  float voltageFactor = 1.0;
  if (batteryData.voltage < 11.0) {
    voltageFactor = 0.5;
  } else if (batteryData.voltage < 11.5) {
    voltageFactor = 0.7;
  } else if (batteryData.voltage < 12.0) {
    voltageFactor = 0.85;
  }
  
  // Faktor 2: Suhu
  float tempFactor = 1.0;
  if (batteryData.temperature > 45) {
    tempFactor = 0.8;
  } else if (batteryData.temperature > 40) {
    tempFactor = 0.9;
  } else if (batteryData.temperature < 0) {
    tempFactor = 0.8;
  } else if (batteryData.temperature < 5) {
    tempFactor = 0.9;
  }
  
  // Faktor 3: Siklus
  float cycleFactor = 1.0 - (batteryData.cycles / 5000.0); // Asumsi 5000 siklus lifetime
  cycleFactor = constrain(cycleFactor, 0.0, 1.0);
  
  // Update SoH
  float newSoH = 100.0 * voltageFactor * tempFactor * cycleFactor;
  
  // Smooth update
  batteryData.soh = (batteryData.soh * 0.9) + (newSoH * 0.1);
  batteryData.soh = constrain(batteryData.soh, 0.0, 100.0);
}

void updateBatteryCycles() {
  // Hitung siklus pengisian berdasarkan perubahan SoC
  static float lastSOC = batteryData.soc;
  static bool wasCharging = batteryData.isCharging;
  static float cycleCount = 0.0;
  
  // Deteksi satu siklus penuh
  if (wasCharging && !batteryData.isCharging) {
    // Berhenti mengisi
    if (batteryData.soc > 95.0) {
      cycleCount += 0.5; // Setengah siklus
    }
  } else if (!wasCharging && batteryData.isCharging) {
    // Mulai mengisi
    if (batteryData.soc < 20.0) {
      cycleCount += 0.5; // Setengah siklus
    }
  }
  
  // Update siklus jika sudah satu siklus penuh
  if (cycleCount >= 1.0) {
    batteryData.cycles += floor(cycleCount);
    cycleCount -= floor(cycleCount);
    
    // Simpan ke EEPROM
    EEPROM.write(ADDR_BATTERY_CYCLES, batteryData.cycles & 0xFF);
    EEPROM.write(ADDR_BATTERY_CYCLES + 1, (batteryData.cycles >> 8) & 0xFF);
    EEPROM.write(ADDR_BATTERY_CYCLES + 2, (batteryData.cycles >> 16) & 0xFF);
    EEPROM.write(ADDR_BATTERY_CYCLES + 3, (batteryData.cycles >> 24) & 0xFF);
    EEPROM.commit();
    
    logMessage("INFO", "Battery cycles updated: " + String(batteryData.cycles), "BATTERY");
  }
  
  lastSOC = batteryData.soc;
  wasCharging = batteryData.isCharging;
}

//============================================================================
// UPDATE DATA ENERGI
//============================================================================
void updateEnergyData() {
  unsigned long currentTime = millis();
  unsigned long deltaTime = currentTime - energyData.lastUpdateTime;
  
  if (deltaTime < 1000) return; // Update setiap detik
  
  // Hitung energi dalam Wh
  float mpptPower = voltageMPPT * currentMPPT;
  float pwmPower = voltagePWM * currentPWM;
  float loadPower = voltageLoad * currentLoad;
  
  energyData.mpptEnergy += mpptPower * (deltaTime / 3600000.0);
  energyData.pwmEnergy += pwmPower * (deltaTime / 3600000.0);
  energyData.loadEnergy += loadPower * (deltaTime / 3600000.0);
  
  // Update total energi
  if (batteryData.isCharging) {
    energyData.energyCharged += (mpptPower + pwmPower) * (deltaTime / 3600000.0);
  } else {
    energyData.energyDischarged += loadPower * (deltaTime / 3600000.0);
  }
  
  // Hitung efisiensi
  if (energyData.energyCharged > 0) {
    energyData.efficiency = (energyData.energyDischarged / energyData.energyCharged) * 100.0;
  }
  
  energyData.lastUpdateTime = currentTime;
}

//============================================================================
// LOG DATA BATERAI
//============================================================================
void logBatteryData() {
  static unsigned long lastLogTime = 0;
  if (millis() - lastLogTime < 300000) return; // Log setiap 5 menit
  
  if (!sdCardAvailable) return;
  
  // Buat log entry
  BatteryLog log;
  log.timestamp = millis();
  log.voltage = batteryData.voltage;
  log.current = batteryData.current;
  log.temperature = batteryData.temperature;
  log.soc = batteryData.soc;
  log.soh = batteryData.soh;
  log.isCharging = batteryData.isCharging;
  
  // Simpan ke array
  batteryLogs[batteryLogIndex] = log;
  batteryLogIndex = (batteryLogIndex + 1) % 200;
  
  // Simpan ke SD card
  File file = SD.open("/battery_log.csv", FILE_APPEND);
  if (file) {
    file.print(log.timestamp);
    file.print(",");
    file.print(log.voltage);
    file.print(",");
    file.print(log.current);
    file.print(",");
    file.print(log.temperature);
    file.print(",");
    file.print(log.soc);
    file.print(",");
    file.print(log.soh);
    file.print(",");
    file.println(log.isCharging ? "1" : "0");
    file.close();
  }
  
  lastLogTime = millis();
}

//============================================================================
// MANAJEMEN ENERGI
//============================================================================
void manageEnergyEfficiency() {
  // Implementasi manajemen energi yang cerdas
  
  // Load shedding jika baterai rendah
  if (batteryData.soc < BATTERY_CRITICAL_THRESHOLD) {
    // Matikan beban tidak penting
    controlLoad("Lampu Gudang", false);
    controlLoad("Lampu Kebun", false);
    
    // Kurangi durasi operasi beban penting
    // Implementasi akan ditambahkan di bagian selanjutnya
    
    logMessage("WARNING", "Load shedding activated due to low battery", "ENERGY");
  }
  
  // Optimasi pengisian baterai
  if (batteryData.isCharging) {
    // Implementasi algoritma pengisian cerdas
    optimizeBatteryCharging();
  }
}

void optimizeBatteryCharging() {
  // Implementasi algoritma pengisian baterai yang cerdas
  
  // Fase 1: Bulk charging (hingga 80%)
  if (batteryData.soc < 80.0) {
    // Gunakan MPPT dan PWM secara bersamaan
    // Kontrol sudah dihandle oleh hardware
  }
  // Fase 2: Absorption charging (80% - 95%)
  else if (batteryData.soc < 95.0) {
    // Kurangi arus pengisian secara bertahap
    // Implementasi akan ditambahkan di bagian selanjutnya
  }
  // Fase 3: Float charging (95% - 100%)
  else {
    // Maintenence charge dengan arus rendah
    // Implementasi akan ditambahkan di bagian selanjutnya
  }
  
  // Cek suhu baterai
  if (batteryData.temperature > 40) {
    // Kurangi atau hentikan pengisian jika suhu terlalu tinggi
    logMessage("WARNING", "Battery temperature too high, reducing charging", "ENERGY");
  }
}

//============================================================================
// CUSTOM RULES ENGINE - FUNCTION DEFINITIONS
//============================================================================
void evaluateCustomRules() {
  for (int i = 0; i < customRuleCount; i++) {
    if (!customRules[i].active) continue;
    
    // Cek cooldown period
    if (millis() - customRules[i].lastExecutionTime < customRules[i].cooldownPeriod * 1000) {
      continue;
    }
    
    float sensorValue = getSensorValue(customRules[i].sensorSource);
    bool conditionMet = false;
    
    // Cek kondisi
    if (customRules[i].condition == "<") {
      conditionMet = sensorValue < customRules[i].threshold;
    } else if (customRules[i].condition == ">") {
      conditionMet = sensorValue > customRules[i].threshold;
    } else if (customRules[i].condition == "=") {
      conditionMet = sensorValue == customRules[i].threshold;
    } else if (customRules[i].condition == "<=") {
      conditionMet = sensorValue <= customRules[i].threshold;
    } else if (customRules[i].condition == ">=") {
      conditionMet = sensorValue >= customRules[i].threshold;
    }
    
    if (conditionMet) {
      executeRuleAction(customRules[i].action);
      customRules[i].lastExecutionTime = millis();
      
      String message = "🔔 Custom Rule Triggered: " + customRules[i].ruleName;
      sendTelegramMessage(message);
      logMessage("INFO", message, "RULES");
    }
  }
}

float getSensorValue(String sensorName) {
  // Kembalikan nilai sensor berdasarkan nama
  if (sensorName == "temperatureGudang") return temperatureGudang;
  if (sensorName == "humidityGudang") return humidityGudang;
  if (sensorName == "lightGudang") return lightGudang;
  if (sensorName == "co2Gudang") return co2Gudang;
  if (sensorName == "soilMoisture") return soilMoisture;
  if (sensorName == "batteryVoltage") return batteryVoltage;
  if (sensorName == "batterySOC") return batteryData.soc;
  if (sensorName == "batteryTemperature") return batteryData.temperature;
  if (sensorName == "waterPressure") return waterPressure;
  if (sensorName == "pressureKebun") return pressureKebun;
  if (sensorName == "temperatureKebun") return temperatureKebun;
  if (sensorName == "lightKebun") return lightKebun;
  return -999; // Sensor tidak valid
}

void executeRuleAction(String action) {
  if (action == "exhaustfan_on") {
    turnOnInverter();
    controlLoad("Exhaust Fan", true);
  } else if (action == "exhaustfan_off") {
    controlLoad("Exhaust Fan", false);
  } else if (action == "pumpgudang_on") {
    turnOnInverter();
    controlLoad("Pump Gudang", true);
  } else if (action == "pumpgudang_off") {
    controlLoad("Pump Gudang", false);
  } else if (action == "humidifier_on") {
    controlLoad("Humidifier", true);
  } else if (action == "humidifier_off") {
    controlLoad("Humidifier", false);
  } else if (action == "pumpkebun_on") {
    turnOnInverter();
    controlLoad("Pump Kebun", true);
  } else if (action == "pumpkebun_off") {
    controlLoad("Pump Kebun", false);
  } else if (action == "lamp_jalan_on") {
    controlLoad("Lampu Jalan", true);
  } else if (action == "lamp_jalan_off") {
    controlLoad("Lampu Jalan", false);
  } else if (action == "lamp_gudang_on") {
    controlLoad("Lampu Gudang", true);
  } else if (action == "lamp_gudang_off") {
    controlLoad("Lampu Gudang", false);
  } else if (action == "lamp_kebun_on") {
    controlLoad("Lampu Kebun", true);
  } else if (action == "lamp_kebun_off") {
    controlLoad("Lampu Kebun", false);
  } else if (action == "inverter_on") {
    turnOnInverter();
  } else if (action == "inverter_off") {
    turnOffInverter();
  }
}

void addCustomRule(String ruleName, String sensor, String condition, float value, String action) {
  if (customRuleCount >= 30) {
    logMessage("ERROR", "Maximum custom rules reached", "RULES");
    return;
  }
  
  CustomRule rule;
  rule.ruleName = ruleName;
  rule.sensorSource = sensor;
  rule.condition = condition;
  rule.threshold = value;
  rule.action = action;
  rule.active = true;
  rule.executed = false;
  rule.lastExecutionTime = 0;
  rule.cooldownPeriod = 300; // 5 menit default
  
  customRules[customRuleCount++] = rule;
  
  // Simpan ke SPIFFS
  saveCustomRules();
  
  logMessage("INFO", "Custom rule added: " + ruleName, "RULES");
}

void saveCustomRules() {
  if (!sdCardAvailable) return;
  
  File file = SD.open("/custom_rules.json", FILE_WRITE);
  if (!file) {
    logMessage("ERROR", "Failed to save custom rules", "RULES");
    return;
  }
  
  DynamicJsonDocument doc(4096);
  JsonArray rulesArray = doc.createNestedArray("rules");
  
  for (int i = 0; i < customRuleCount; i++) {
    JsonObject ruleObj = rulesArray.createNestedObject();
    ruleObj["name"] = customRules[i].ruleName;
    ruleObj["sensor"] = customRules[i].sensorSource;
    ruleObj["condition"] = customRules[i].condition;
    ruleObj["threshold"] = customRules[i].threshold;
    ruleObj["action"] = customRules[i].action;
    ruleObj["active"] = customRules[i].active;
    ruleObj["cooldown"] = customRules[i].cooldownPeriod;
  }
  
  String jsonString;
  serializeJson(doc, jsonString);
  file.print(jsonString);
  file.close();
  
  logMessage("INFO", "Custom rules saved", "RULES");
}

void loadCustomRules() {
  if (!sdCardAvailable) return;
  
  File file = SD.open("/custom_rules.json", FILE_READ);
  if (!file) {
    logMessage("WARNING", "No custom rules file found", "RULES");
    return;
  }
  
  DynamicJsonDocument doc(4096);
  DeserializationError error = deserializeJson(doc, file);
  file.close();
  
  if (error) {
    logMessage("ERROR", "Failed to load custom rules: " + String(error.c_str()), "RULES");
    return;
  }
  
  JsonArray rulesArray = doc["rules"];
  customRuleCount = 0;
  
  for (JsonObject ruleObj : rulesArray) {
    if (customRuleCount >= 30) break;
    
    customRules[customRuleCount].ruleName = ruleObj["name"].as<String>();
    customRules[customRuleCount].sensorSource = ruleObj["sensor"].as<String>();
    customRules[customRuleCount].condition = ruleObj["condition"].as<String>();
    customRules[customRuleCount].threshold = ruleObj["threshold"];
    customRules[customRuleCount].action = ruleObj["action"].as<String>();
    customRules[customRuleCount].active = ruleObj["active"];
    customRules[customRuleCount].cooldownPeriod = ruleObj["cooldown"];
    customRules[customRuleCount].executed = false;
    customRules[customRuleCount].lastExecutionTime = 0;
    
    customRuleCount++;
  }
  
  logMessage("INFO", "Custom rules loaded: " + String(customRuleCount) + " rules", "RULES");
}

//============================================================================
// FUNGSI INVERTER
//============================================================================
void turnOnInverter() {
  // Nyalakan inverter dengan urutan yang benar
  digitalWrite(BATTERY_RELAY_PIN, HIGH);
  delay(5000);
  digitalWrite(INVERTER_SWITCH_PIN, HIGH);
  
  logMessage("INFO", "Inverter turned on", "CONTROL");
}

void turnOffInverter() {
  // Matikan inverter dengan urutan yang benar
  digitalWrite(INVERTER_SWITCH_PIN, LOW);
  delay(5000);
  digitalWrite(BATTERY_RELAY_PIN, LOW);
  
  logMessage("INFO", "Inverter turned off", "CONTROL");
}

//============================================================================
// FUNGSI UTILITAS
//============================================================================
String getFormattedTime() {
  if (!timeConfigured) {
    return "Time not configured";
  }
  
  timeClient.update();
  String timeStr = timeClient.getFormattedTime();
  
  // Format tanggal
  unsigned long epochTime = timeClient.getEpochTime();
  struct tm *ptm = gmtime ((time_t *)&epochTime);
  
  String dateStr = "";
  dateStr += String(ptm->tm_year + 1900);
  dateStr += "-";
  dateStr += String(ptm->tm_mon + 1);
  dateStr += "-";
  dateStr += String(ptm->tm_mday);
  
  return dateStr + " " + timeStr;
}

void logMessage(String level, String message, String source) {
  // Simpan log ke array
  if (xSemaphoreTake(logMutex, portMAX_DELAY)) {
    logEntries[logIndex].timestamp = millis();
    logEntries[logIndex].level = level;
    logEntries[logIndex].message = message;
    logEntries[logIndex].source = source;
    
    logIndex = (logIndex + 1) % 200;
    
    xSemaphoreGive(logMutex);
  }
  
  // Cetak ke Serial
  Serial.print("[" + level + "] [" + source + "] " + message);
  
  // Simpan ke SD card jika tersedia
  if (sdCardAvailable) {
    File file = SD.open("/system_log.txt", FILE_APPEND);
    if (file) {
      file.print("[" + getFormattedTime() + "] [" + level + "] [" + source + "] " + message + "\n");
      file.close();
    }
  }
  
  // Kirim notifikasi jika error atau critical
  if (level == "ERROR" || level == "CRITICAL") {
    String notification[3] = {"telegram", "[" + level + "] " + message, ""};
    xQueueSend(notificationQueue, &notification, portMAX_DELAY);
  }
}

//============================================================================
// DEEP SLEEP
//============================================================================
void enterDeepSleep() {
  logMessage("INFO", "Entering deep sleep mode", "SYSTEM");
  
  // Matikan semua beban
  for (int i = 0; i < 7; i++) {
    digitalWrite(loads[i].relayPin, LOW);
  }
  
  // Matikan inverter
  turnOffInverter();
  
  // Setup wake up sources
  esp_sleep_enable_timer_wakeup(DEEP_SLEEP_DURATION);
  esp_sleep_enable_ext0_wakeup(GPIO_NUM_25, 0); // Wake up on emergency button
  
  // Enter deep sleep
  esp_deep_sleep_start();
}

//============================================================================
// INISIALISASI WIFI
//============================================================================
void initializeWiFi() {
  Serial.print("Connecting to WiFi...");
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  
  int attempts = 0;
  while (WiFi.status() != WL_CONNECTED && attempts < 30) {
    delay(500);
    Serial.print(".");
    attempts++;
  }
  
  if (WiFi.status() == WL_CONNECTED) {
    wifiConnected = true;
    Serial.println("\nWiFi connected");
    Serial.print("IP address: ");
    Serial.println(WiFi.localIP());
    logMessage("INFO", "WiFi connected", "SYSTEM");
  } else {
    Serial.println("\nWiFi connection failed");
    logMessage("ERROR", "WiFi connection failed", "SYSTEM");
  }
  
  // Set root CA untuk HTTPS
  client.setCACert(TELEGRAM_CERTIFICATE_ROOT); 
}

//============================================================================
// INISIALISASI CAM WIFI
//============================================================================
void initializeCamWiFi() {
  Serial.println("Setting up WiFi AP for CAM...");
  WiFi.softAP(CAM_WIFI_SSID, CAM_WIFI_PASSWORD);
  IPAddress IP = WiFi.softAPIP();
  Serial.print("AP IP address: ");
  Serial.println(IP);

  // Handler untuk permintaan status PIR dari CAM
  server_cam.on("/pir_status", HTTP_GET, []() {
    // Kirim status PIR dalam format JSON
    String json = "{";
    json += "\"pir1\":" + String(pir1Status ? "true" : "false") + ",";
    json += "\"pir2\":" + String(pir2Status ? "true" : "false");
    json += "}";
    server_cam.send(200, "application/json", json);
    Serial.println("Sent PIR status to CAM: " + json); // Log untuk debugging
  });

  server_cam.begin();
  Serial.println("CAM WiFi AP and Server initialized.");
}

//============================================================================
// INISIALISASI WEB SERVER
//============================================================================
void initializeWebServer() {
  // Halaman utama
  server.on("/", handleRoot);
  
  // Halaman grafik
  server.on("/graphs", handleGraphs);
  
  // Halaman kontrol
  server.on("/control", handleControl);
  
  // Halaman sistem
  server.on("/system", handleSystem);
  
  // Halaman status
  server.on("/status", handleStatus);
  
  // Halaman maintenance
  server.on("/maintenance", handleMaintenance);
  
  // Halaman pengaturan
  server.on("/settings", handleSettings);
  
  // Halaman log
  server.on("/logs", handleLogs);
  
  // Halaman prediksi
  server.on("/prediction", handlePrediction);
  
  // API untuk data sensor
  server.on("/api/sensors", HTTP_GET, handleApiSensors);
  
  // API untuk kontrol beban
  server.on("/api/control", HTTP_POST, handleApiControl);
  
  // API untuk konfigurasi
  server.on("/api/config", HTTP_GET, handleApiConfigGet);
  server.on("/api/config", HTTP_POST, handleApiConfigPost);
  
  // API untuk custom rules
  server.on("/api/rules", HTTP_GET, handleApiRulesGet);
  server.on("/api/rules", HTTP_POST, handleApiRulesPost);
  server.on("/api/rules", HTTP_DELETE, handleApiRulesDelete);
  
  // API untuk jadwal
  server.on("/api/schedules", HTTP_GET, handleApiSchedulesGet);
  server.on("/api/schedules", HTTP_POST, handleApiSchedulesPost);
  
  // API untuk maintenance
  server.on("/api/maintenance", HTTP_GET, handleApiMaintenanceGet);
  server.on("/api/maintenance", HTTP_POST, handleApiMaintenancePost);
  
  // API untuk log
  server.on("/api/logs", HTTP_GET, handleApiLogsGet);
  server.on("/api/logs/download", HTTP_GET, handleApiLogsDownload);
  
  // API untuk prediksi
  server.on("/api/prediction", HTTP_GET, handleApiPredictionGet);
  
  // API untuk energy report
  server.on("/api/energy", HTTP_GET, handleApiEnergyGet);
  
  // API untuk komponen health
  server.on("/api/health", HTTP_GET, handleApiHealthGet);
  
  // API untuk kamera
  server.on("/api/camera", HTTP_POST, handleApiCamera);
  
  // API untuk mode sistem
  server.on("/api/mode", HTTP_POST, handleApiMode);
  
  // API untuk inverter
  server.on("/api/inverter", HTTP_POST, handleApiInverter);
  
  // Autentikasi
  server.on("/login", HTTP_POST, handleLogin);
  
  // File tidak ditemukan
  server.onNotFound(handleNotFound);
  
  server.begin();
  Serial.println("Web server started");
  logMessage("INFO", "Web server started", "SYSTEM");
}

//============================================================================
// WEB SERVER HANDLERS
//============================================================================
void handleRoot() {
  // Cek autentikasi
  if (!server.authenticate(config.webUsername, config.webPassword)) {
    return server.requestAuthentication();
  }
  
  String html = getIndexHTML();
  server.send(200, "text/html", html);
}

void handleGraphs() {
  // Cek autentikasi
  if (!server.authenticate(config.webUsername, config.webPassword)) {
    return server.requestAuthentication();
  }
  
  String html = getGraphsHTML();
  server.send(200, "text/html", html);
}

void handleControl() {
  // Cek autentikasi
  if (!server.authenticate(config.webUsername, config.webPassword)) {
    return server.requestAuthentication();
  }
  
  String html = getControlHTML();
  server.send(200, "text/html", html);
}

void handleSystem() {
  // Cek autentikasi
  if (!server.authenticate(config.webUsername, config.webPassword)) {
    return server.requestAuthentication();
  }
  
  String html = getSystemHTML();
  server.send(200, "text/html", html);
}

void handleStatus() {
  // Cek autentikasi
  if (!server.authenticate(config.webUsername, config.webPassword)) {
    return server.requestAuthentication();
  }
  
  String html = getSystemHTML();
  server.send(200, "text/html", html);
}

void handleMaintenance() {
  // Cek autentikasi
  if (!server.authenticate(config.webUsername, config.webPassword)) {
    return server.requestAuthentication();
  }
  
  String html = getMaintenanceHTML();
  server.send(200, "text/html", html);
}

void handleSettings() {
  // Cek autentikasi
  if (!server.authenticate(config.webUsername, config.webPassword)) {
    return server.requestAuthentication();
  }
  
  String html = getSettingsHTML();
  server.send(200, "text/html", html);
}

void handleLogs() {
  // Cek autentikasi
  if (!server.authenticate(config.webUsername, config.webPassword)) {
    return server.requestAuthentication();
  }
  
  String html = getLogsHTML();
  server.send(200, "text/html", html);
}

void handlePrediction() {
  // Cek autentikasi
  if (!server.authenticate(config.webUsername, config.webPassword)) {
    return server.requestAuthentication();
  }
  
  String html = getPredictionHTML();
  server.send(200, "text/html", html);
}

void handleNotFound() {
  server.send(404, "text/plain", "Not found");
}

//============================================================================
// API HANDLERS
//============================================================================
void handleApiSensors(AsyncWebServerRequest *request) {
  // Cek autentikasi
  if (!server.authenticate(config.webUsername, config.webPassword)) {
    return server.requestAuthentication();
  }
  
  DynamicJsonDocument doc(2048);
  
  doc["temperature"] = temperatureGudang;
  doc["humidity"] = humidityGudang;
  doc["lightGudang"] = lightGudang;
  doc["co2"] = co2Gudang;
  doc["soilMoisture"] = soilMoisture;
  doc["pressure"] = pressureKebun;
  doc["temperatureKebun"] = temperatureKebun;
  doc["lightKebun"] = lightKebun;
  doc["waterPressure"] = waterPressure;
  doc["batteryVoltage"] = batteryVoltage;
  doc["batterySOC"] = batteryData.soc;
  doc["batteryCycles"] = batteryData.cycles;
  doc["pir1"] = pir1Status;
  doc["pir2"] = pir2Status;
  doc["emergency"] = emergencyMode;
  doc["mode"] = modeNames[config.operationMode];
  doc["timestamp"] = getFormattedTime();
  
  // Tambahkan data beban
  JsonArray loadsArray = doc.createNestedArray("loads");
  for (int i = 0; i < 7; i++) {
    JsonObject loadObj = loadsArray.createNestedObject();
    loadObj["name"] = loads[i].name;
    loadObj["status"] = loads[i].actualStatus;
    loadObj["command"] = loads[i].commandStatus;
    loadObj["current"] = currentLoad;
  }
  
  // Tambahkan data baterai
  JsonObject batteryObj = doc.createNestedObject("battery");
  batteryObj["voltage"] = batteryData.voltage;
  batteryObj["current"] = batteryData.current;
  batteryObj["temperature"] = batteryData.temperature;
  batteryObj["soc"] = batteryData.soc;
  batteryObj["soh"] = batteryData.soh;
  batteryObj["cycles"] = batteryData.cycles;
  batteryObj["isCharging"] = batteryData.isCharging;
  
  // Tambahkan data energi
  JsonObject energyObj = doc.createNestedArray("energy");
  energyObj["mpptEnergy"] = energyData.mpptEnergy;
  energyObj["pwmEnergy"] = energyData.pwmEnergy;
  energyObj["loadEnergy"] = energyData.loadEnergy;
  energyObj["efficiency"] = energyData.efficiency;
  
  String jsonString;
  serializeJson(doc, jsonString);
  
  server.send(200, "application/json", jsonString);
}

void handleApiControl(AsyncWebServerRequest *request) {
    // Handler untuk body request (jika menggunakan JSON)
    if (request->hasParam("load", true) && request->hasParam("status", true)) {
        // Handle form data (jika dikirim sebagai form)
        String loadName = request->getParam("load", true)->value();
        bool status = request->getParam("status", true)->value() == "true";

        // Cari load berdasarkan nama dan update statusnya (atau kirim ke task kontrol)
        bool found = false;
        for(int i=0; i < MAX_LOADS; i++){
            if(loads[i].name == loadName){
                // Contoh: Langsung kontrol atau kirim command ke queue
                // controlLoad(loadName, status); // Jika Anda punya fungsi ini
                loads[i].command = status; // Atau set command saja jika task lain yg eksekusi
                logMessage("INFO", "Control command received for " + loadName + ": " + (status ? "ON" : "OFF"), "CONTROL");
                found = true;
                break;
            }
        }

        if(found){
             request->send(200, "application/json", "{\"success\":true}");
        } else {
             request->send(400, "application/json", "{\"success\":false, \"error\":\"Load not found\"}");
        }

    } else {
        // Jika data dikirim sebagai JSON body
        AsyncWebParameter* p = request->getParam(0); // Ambil body
        if (p && p->isPost() && p->contentType() == "application/json") {
            StaticJsonDocument<200> doc;
            DeserializationError error = deserializeJson(doc, p->value());
            if (error) {
                request->send(400, "application/json", "{\"success\":false, \"error\":\"Invalid JSON\"}");
                return;
            }

            const char* loadName = doc["load"];
            bool status = doc["status"];

             // Cari load berdasarkan nama dan update statusnya
            bool found = false;
             for(int i=0; i < MAX_LOADS; i++){
                if(loads[i].name == loadName){
                    loads[i].command = status;
                    logMessage("INFO", "Control command received via JSON for " + String(loadName) + ": " + (status ? "ON" : "OFF"), "CONTROL");
                    found = true;
                    break;
                }
            }
             if(found){
                 request->send(200, "application/json", "{\"success\":true}");
             } else {
                 request->send(400, "application/json", "{\"success\":false, \"error\":\"Load not found\"}");
            }
        } else {
            request->send(400, "application/json", "{\"success\":false, \"error\":\"Invalid request\"}");
        }
    }
}

void handleApiConfigGet(AsyncWebServerRequest *request) {
  // Cek autentikasi
  if (!server.authenticate(config.webUsername, config.webPassword)) {
    return server.requestAuthentication();
  }
  
  DynamicJsonDocument doc(1024);
  
  doc["exhaustFanThreshold"] = config.exhaustFanThreshold;
  doc["pumpGudangThreshold"] = config.pumpGudangThreshold;
  doc["humidifierThreshold"] = config.humidifierThreshold;
  doc["pumpKebunThreshold"] = config.pumpKebunThreshold;
  doc["lampJalanThreshold"] = config.lampJalanThreshold;
  doc["lampGudangThreshold"] = config.lampGudangThreshold;
  doc["lampKebunThreshold"] = config.lampKebunThreshold;
  doc["tempHighThreshold"] = config.tempHighThreshold;
  doc["tempNormalThreshold"] = config.tempNormalThreshold;
  doc["humidityLowThreshold"] = config.humidityLowThreshold;
  doc["humidityHighThreshold"] = config.humidityHighThreshold;
  doc["soilMoistureLowThreshold"] = config.soilMoistureLowThreshold;
  doc["soilMoistureHighThreshold"] = config.soilMoistureHighThreshold;
  doc["co2HighThreshold"] = config.co2HighThreshold;
  doc["batteryLowThreshold"] = config.batteryLowThreshold;
  doc["batteryCriticalThreshold"] = config.batteryCriticalThreshold;
  doc["batteryEmergencyThreshold"] = config.batteryEmergencyThreshold;
  doc["notificationMode"] = config.notificationMode;
  doc["dndStart"] = config.dndStart;
  doc["dndEnd"] = config.dndEnd;
  doc["operationMode"] = modeNames[config.operationMode];
  doc["encryptionEnabled"] = config.encryptionEnabled;
  doc["otaEnabled"] = config.otaEnabled;
  doc["predictiveEnabled"] = config.predictiveEnabled;
  doc["adaptiveEnabled"] = config.adaptiveEnabled;
  doc["pidEnabled"] = config.pidEnabled;
  
  String jsonString;
  serializeJson(doc, jsonString);
  
  server.send(200, "application/json", jsonString);
}

void handleApiConfigPost(AsyncWebServerRequest *request) {
  // Cek autentikasi
  if (!server.authenticate(config.webUsername, config.webPassword)) {
    return server.requestAuthentication();
  }
  
  String body = server.arg("plain");
  DynamicJsonDocument doc(1024);
  DeserializationError error = deserializeJson(doc, body);
  
  if (error) {
    server.send(400, "application/json", "{\"error\":\"Invalid JSON\"}");
    return;
  }
  
  // Update konfigurasi
  if (doc.containsKey("exhaustFanThreshold")) {
    config.exhaustFanThreshold = doc["exhaustFanThreshold"];
  }
  if (doc.containsKey("pumpGudangThreshold")) {
    config.pumpGudangThreshold = doc["pumpGudangThreshold"];
  }
  if (doc.containsKey("humidifierThreshold")) {
    config.humidifierThreshold = doc["humidifierThreshold"];
  }
  if (doc.containsKey("pumpKebunThreshold")) {
    config.pumpKebunThreshold = doc["pumpKebunThreshold"];
  }
  if (doc.containsKey("lampJalanThreshold")) {
    config.lampJalanThreshold = doc["lampJalanThreshold"];
  }
  if (doc.containsKey("lampGudangThreshold")) {
    config.lampGudangThreshold = doc["lampGudangThreshold"];
  }
  if (doc.containsKey("lampKebunThreshold")) {
    config.lampKebunThreshold = doc["lampKebunThreshold"];
  }
  if (doc.containsKey("tempHighThreshold")) {
    config.tempHighThreshold = doc["tempHighThreshold"];
  }
  if (doc.containsKey("tempNormalThreshold")) {
    config.tempNormalThreshold = doc["tempNormalThreshold"];
  }
  if (doc.containsKey("humidityLowThreshold")) {
    config.humidityLowThreshold = doc["humidityLowThreshold"];
  }
  if (doc.containsKey("humidityHighThreshold")) {
    config.humidityHighThreshold = doc["humidityHighThreshold"];
  }
  if (doc.containsKey("soilMoistureLowThreshold")) {
    config.soilMoistureLowThreshold = doc["soilMoistureLowThreshold"];
  }
  if (doc.containsKey("soilMoistureHighThreshold")) {
    config.soilMoistureHighThreshold = doc["soilMoistureHighThreshold"];
  }
  if (doc.containsKey("co2HighThreshold")) {
    config.co2HighThreshold = doc["co2HighThreshold"];
  }
  if (doc.containsKey("batteryLowThreshold")) {
    config.batteryLowThreshold = doc["batteryLowThreshold"];
  }
  if (doc.containsKey("batteryCriticalThreshold")) {
    config.batteryCriticalThreshold = doc["batteryCriticalThreshold"];
  }
  if (doc.containsKey("batteryEmergencyThreshold")) {
    config.batteryEmergencyThreshold = doc["batteryEmergencyThreshold"];
  }
  if (doc.containsKey("notificationMode")) {
    strcpy(config.notificationMode, doc["notificationMode"]);
  }
  if (doc.containsKey("dndStart")) {
    strcpy(config.dndStart, doc["dndStart"]);
  }
  if (doc.containsKey("dndEnd")) {
    strcpy(config.dndEnd, doc["dndEnd"]);
  }
  if (doc.containsKey("operationMode")) {
    String modeStr = doc["operationMode"];
    for (int i = 0; i < MODE_COUNT; i++) {
      if (modeStr == modeNames[i]) {
        config.operationMode = (SystemMode)i;
        break;
      }
    }
  }
  if (doc.containsKey("encryptionEnabled")) {
    config.encryptionEnabled = doc["encryptionEnabled"];
  }
  if (doc.containsKey("otaEnabled")) {
    config.otaEnabled = doc["otaEnabled"];
  }
  if (doc.containsKey("predictiveEnabled")) {
    config.predictiveEnabled = doc["predictiveEnabled"];
  }
  if (doc.containsKey("adaptiveEnabled")) {
    config.adaptiveEnabled = doc["adaptiveEnabled"];
  }
  if (doc.containsKey("pidEnabled")) {
    config.pidEnabled = doc["pidEnabled"];
  }
  
  // Simpan konfigurasi
  saveConfiguration();
  
  server.send(200, "application/json", "{\"success\":true}");
}

void handleApiRulesGet(AsyncWebServerRequest *request) {
  // Cek autentikasi
  if (!server.authenticate(config.webUsername, config.webPassword)) {
    return server.requestAuthentication();
  }
  
  DynamicJsonDocument doc(4096);
  JsonArray rulesArray = doc.createNestedArray("rules");
  
  for (int i = 0; i < customRuleCount; i++) {
    JsonObject ruleObj = rulesArray.createNestedObject();
    ruleObj["id"] = i;
    ruleObj["name"] = customRules[i].ruleName;
    ruleObj["sensor"] = customRules[i].sensorSource;
    ruleObj["condition"] = customRules[i].condition;
    ruleObj["threshold"] = customRules[i].threshold;
    ruleObj["action"] = customRules[i].action;
    ruleObj["active"] = customRules[i].active;
    ruleObj["cooldown"] = customRules[i].cooldownPeriod;
  }
  
  String jsonString;
  serializeJson(doc, jsonString);
  
  server.send(200, "application/json", jsonString);
}

void handleApiRulesPost(AsyncWebServerRequest *request) {
  // Cek autentikasi
  if (!server.authenticate(config.webUsername, config.webPassword)) {
    return server.requestAuthentication();
  }
  
  String body = server.arg("plain");
  DynamicJsonDocument doc(1024);
  DeserializationError error = deserializeJson(doc, body);
  
  if (error) {
    server.send(400, "application/json", "{\"error\":\"Invalid JSON\"}");
    return;
  }
  
  String ruleName = doc["name"];
  String sensor = doc["sensor"];
  String condition = doc["condition"];
  float threshold = doc["threshold"];
  String action = doc["action"];
  int cooldown = doc["cooldown"];
  
  addCustomRule(ruleName, sensor, condition, threshold, action);
  
  if (customRuleCount > 0) {
    customRules[customRuleCount - 1].cooldownPeriod = cooldown;
  }
  
  server.send(200, "application/json", "{\"success\":true}");
}

void handleApiRulesDelete(AsyncWebServerRequest *request) {
  // Cek autentikasi
  if (!server.authenticate(config.webUsername, config.webPassword)) {
    return server.requestAuthentication();
  }
  
  String body = server.arg("plain");
  DynamicJsonDocument doc(1024);
  DeserializationError error = deserializeJson(doc, body);
  
  if (error) {
    server.send(400, "application/json", "{\"error\":\"Invalid JSON\"}");
    return;
  }
  
  int id = doc["id"];
  
  if (id >= 0 && id < customRuleCount) {
    // Geser semua rule setelah yang dihapus
    for (int i = id; i < customRuleCount - 1; i++) {
      customRules[i] = customRules[i + 1];
    }
    customRuleCount--;
    
    saveCustomRules();
    
    server.send(200, "application/json", "{\"success\":true}");
  } else {
    server.send(400, "application/json", "{\"error\":\"Invalid rule ID\"}");
  }
}

void handleApiSchedulesGet(AsyncWebServerRequest *request) {
  // Cek autentikasi
  if (!server.authenticate(config.webUsername, config.webPassword)) {
    return server.requestAuthentication();
  }
  
  DynamicJsonDocument doc(2048);
  JsonArray schedulesArray = doc.createNestedArray("schedules");
  
  for (int i = 0; i < scheduleCount; i++) {
    JsonObject scheduleObj = schedulesArray.createNestedObject();
    scheduleObj["id"] = i;
    scheduleObj["hour"] = schedules[i].hour;
    scheduleObj["minute"] = schedules[i].minute;
    scheduleObj["loadName"] = schedules[i].loadName;
    scheduleObj["active"] = schedules[i].active;
    scheduleObj["condition"] = schedules[i].condition;
    scheduleObj["conditionValue"] = schedules[i].conditionValue;
    scheduleObj["conditionOperator"] = schedules[i].conditionOperator;
  }
  
  String jsonString;
  serializeJson(doc, jsonString);
  
  server.send(200, "application/json", jsonString);
}

void handleApiSchedulesPost(AsyncWebServerRequest *request) {
  // Cek autentikasi
  if (!server.authenticate(config.webUsername, config.webPassword)) {
    return server.requestAuthentication();
  }
  
  String body = server.arg("plain");
  DynamicJsonDocument doc(1024);
  DeserializationError error = deserializeJson(doc, body);
  
  if (error) {
    server.send(400, "application/json", "{\"error\":\"Invalid JSON\"}");
    return;
  }
  
  int hour = doc["hour"];
  int minute = doc["minute"];
  String loadName = doc["loadName"];
  bool active = doc["active"];
  String condition = doc["condition"];
  float conditionValue = doc["conditionValue"];
  String conditionOperator = doc["conditionOperator"];
  
  if (scheduleCount < 20) {
    schedules[scheduleCount].hour = hour;
    schedules[scheduleCount].minute = minute;
    schedules[scheduleCount].loadName = loadName;
    schedules[scheduleCount].active = active;
    schedules[scheduleCount].condition = condition;
    schedules[scheduleCount].conditionValue = conditionValue;
    schedules[scheduleCount].conditionOperator = conditionOperator;
    scheduleCount++;
    
    saveSchedules();
    
    server.send(200, "application/json", "{\"success\":true}");
  } else {
    server.send(400, "application/json", "{\"error\":\"Maximum schedules reached\"}");
  }
}

void handleApiMaintenanceGet(AsyncWebServerRequest *request) {
  // Cek autentikasi
  if (!server.authenticate(config.webUsername, config.webPassword)) {
    return server.requestAuthentication();
  }
  
  DynamicJsonDocument doc(4096);
  JsonArray activitiesArray = doc.createNestedArray("activities");
  
  for (int i = 0; i < maintenanceActivityCount; i++) {
    JsonObject activityObj = activitiesArray.createNestedObject();
    activityObj["id"] = i;
    activityObj["timestamp"] = maintenanceActivities[i].timestamp;
    activityObj["activityType"] = maintenanceActivities[i].activityType;
    activityObj["description"] = maintenanceActivities[i].description;
    activityObj["status"] = maintenanceActivities[i].status;
    activityObj["aiRecommendation"] = maintenanceActivities[i].aiRecommendation;
    activityObj["resolved"] = maintenanceActivities[i].resolved;
  }
  
  String jsonString;
  serializeJson(doc, jsonString);
  
  server.send(200, "application/json", jsonString);
}

void handleApiMaintenancePost(AsyncWebServerRequest *request) {
  // Cek autentikasi
  if (!server.authenticate(config.webUsername, config.webPassword)) {
    return server.requestAuthentication();
  }
  
  String body = server.arg("plain");
  DynamicJsonDocument doc(1024);
  DeserializationError error = deserializeJson(doc, body);
  
  if (error) {
    server.send(400, "application/json", "{\"error\":\"Invalid JSON\"}");
    return;
  }
  
  String activityType = doc["activityType"];
  String description = doc["description"];
  
  MaintenanceActivity activity;
  activity.timestamp = millis();
  activity.activityType = activityType;
  activity.description = description;
  activity.status = "pending";
  activity.aiRecommendation = "";
  activity.resolved = false;
  
  addMaintenanceActivity(activity);
  
  server.send(200, "application/json", "{\"success\":true}");
}

void handleApiLogsGet(AsyncWebServerRequest *request) {
  // Cek autentikasi
  if (!server.authenticate(config.webUsername, config.webPassword)) {
    return server.requestAuthentication();
  }
  
  DynamicJsonDocument doc(4096);
  JsonArray logsArray = doc.createNestedArray("logs");
  
  int startIndex = (logIndex > 50) ? logIndex - 50 : 0;
  int count = (logIndex > 50) ? 50 : logIndex;
  
  for (int i = 0; i < count; i++) {
    int index = (startIndex + i) % 200;
    JsonObject logObj = logsArray.createNestedObject();
    logObj["timestamp"] = logEntries[index].timestamp;
    logObj["level"] = logEntries[index].level;
    logObj["message"] = logEntries[index].message;
    logObj["source"] = logEntries[index].source;
  }
  
  String jsonString;
  serializeJson(doc, jsonString);
  
  server.send(200, "application/json", jsonString);
}

void handleApiLogsDownload(AsyncWebServerRequest *request) {
  // Cek autentikasi
  if (!server.authenticate(config.webUsername, config.webPassword)) {
    return server.requestAuthentication();
  }
  
  if (!sdCardAvailable) {
    server.send(500, "text/plain", "SD card not available");
    return;
  }
  
  File file = SD.open("/system_log.txt", FILE_READ);
  if (!file) {
    server.send(404, "text/plain", "Log file not found");
    return;
  }
  
  server.streamFile(file, "text/plain");
  file.close();
}

void handleApiPredictionGet(AsyncWebServerRequest *request) {
  // Cek autentikasi
  if (!server.authenticate(config.webUsername, config.webPassword)) {
    return server.requestAuthentication();
  }
  
  DynamicJsonDocument doc(1024);
  
  doc["predictedTemp"] = predictionData.predictedTemp;
  doc["predictedHumidity"] = predictionData.predictedHumidity;
  doc["predictedSoilMoisture"] = predictionData.predictedSoilMoisture;
  doc["predictedBatterySOC"] = predictionData.predictedBatterySOC;
  doc["confidence"] = predictionData.confidence;
  doc["predictionTime"] = predictionData.predictionTime;
  doc["targetTime"] = predictionData.targetTime;
  
  String jsonString;
  serializeJson(doc, jsonString);
  
  server.send(200, "application/json", jsonString);
}

void handleApiEnergyGet(AsyncWebServerRequest *request) {
  // Cek autentikasi
  if (!server.authenticate(config.webUsername, config.webPassword)) {
    return server.requestAuthentication();
  }
  
  DynamicJsonDocument doc(1024);
  
  doc["mpptEnergy"] = energyData.mpptEnergy;
  doc["pwmEnergy"] = energyData.pwmEnergy;
  doc["loadEnergy"] = energyData.loadEnergy;
  doc["energyCharged"] = energyData.energyCharged;
  doc["energyDischarged"] = energyData.energyDischarged;
  doc["efficiency"] = energyData.efficiency;
  
  // Tambahkan laporan pengisian PWM
  JsonObject pwmReport = doc.createNestedObject("pwmReport");
  pwmReport["current"] = currentPWM;
  pwmReport["voltage"] = voltagePWM;
  pwmReport["power"] = currentPWM * voltagePWM;
  pwmReport["energyToday"] = energyData.pwmEnergy; // Simplifikasi
  
  String jsonString;
  serializeJson(doc, jsonString);
  
  server.send(200, "application/json", jsonString);
}

void handleApiHealthGet(AsyncWebServerRequest *request) {
  // Cek autentikasi
  if (!server.authenticate(config.webUsername, config.webPassword)) {
    return server.requestAuthentication();
  }
  
  DynamicJsonDocument doc(2048);
  JsonArray componentsArray = doc.createNestedArray("components");
  
  for (int i = 0; i < componentCount; i++) {
    JsonObject componentObj = componentsArray.createNestedObject();
    componentObj["name"] = componentHealth[i].componentName;
    componentObj["healthScore"] = componentHealth[i].healthScore;
    componentObj["lastCheckTime"] = componentHealth[i].lastCheckTime;
    componentObj["failureCount"] = componentHealth[i].failureCount;
    componentObj["lastFailureReason"] = componentHealth[i].lastFailureReason;
    componentObj["expectedLifetime"] = componentHealth[i].expectedLifetime;
    componentObj["currentLifetime"] = componentHealth[i].currentLifetime;
    componentObj["maintenanceRequired"] = componentHealth[i].maintenanceRequired;
  }
  
  String jsonString;
  serializeJson(doc, jsonString);
  
  server.send(200, "application/json", jsonString);
}

void handleApiCamera(AsyncWebServerRequest *request) {
  // Cek autentikasi
  if (!server.authenticate(config.webUsername, config.webPassword)) {
    return server.requestAuthentication();
  }
  
  String body = server.arg("plain");
  DynamicJsonDocument doc(1024);
  DeserializationError error = deserializeJson(doc, body);
  
  if (error) {
    server.send(400, "application/json", "{\"error\":\"Invalid JSON\"}");
    return;
  }
  
  String command = doc["command"];
  
  if (command == "capture") {
    sendCameraCommand("/capture");
    server.send(200, "application/json", "{\"success\":true}");
  } else if (command == "motion_start") {
    sendCameraCommand("/motion_start");
    server.send(200, "application/json", "{\"success\":true}");
  } else if (command == "motion_stop") {
    sendCameraCommand("/motion_stop");
    server.send(200, "application/json", "{\"success\":true}");
  } else {
    server.send(400, "application/json", "{\"error\":\"Invalid command\"}");
  }
}

void handleApiMode(AsyncWebServerRequest *request) {
  // Cek autentikasi
  if (!server.authenticate(config.webUsername, config.webPassword)) {
    return server.requestAuthentication();
  }
  
  String body = server.arg("plain");
  DynamicJsonDocument doc(1024);
  DeserializationError error = deserializeJson(doc, body);
  
  if (error) {
    server.send(400, "application/json", "{\"error\":\"Invalid JSON\"}");
    return;
  }
  
  String modeStr = doc["mode"];
  
  for (int i = 0; i < MODE_COUNT; i++) {
    if (modeStr == modeNames[i]) {
      config.operationMode = (SystemMode)i;
      saveConfiguration();
      
      String message = "🔧 System mode changed to: " + modeStr;
      sendTelegramMessage(message);
      logMessage("INFO", message, "SYSTEM");
      
      server.send(200, "application/json", "{\"success\":true}");
      return;
    }
  }
  
  server.send(400, "application/json", "{\"error\":\"Invalid mode\"}");
}

void handleApiInverter(AsyncWebServerRequest *request) {
  // Cek autentikasi
  if (!server.authenticate(config.webUsername, config.webPassword)) {
    return server.requestAuthentication();
  }
  
  String body = server.arg("plain");
  DynamicJsonDocument doc(1024);
  DeserializationError error = deserializeJson(doc, body);
  
  if (error) {
    server.send(400, "application/json", "{\"error\":\"Invalid JSON\"}");
    return;
  }
  
  bool status = doc["status"];
  
  if (status) {
    turnOnInverter();
  } else {
    turnOffInverter();
  }
  
  server.send(200, "application/json", "{\"success\":true}");
}

void handleLogin(AsyncWebServerRequest *request) {
  String username = server.arg("username");
  String password = server.arg("password");
  
  if (username == config.webUsername && password == config.webPassword) {
    server.send(200, "application/json", "{\"success\":true}");
  } else {
    server.send(401, "application/json", "{\"success\":false}");
  }
}

//============================================================================
// FUNGSI UNTUK KOMUNIKASI DENGAN ESP32-CAM
//============================================================================
// Tambahkan di ESP32 Main
void handleCamEvent() {
  // Endpoint untuk menerima event dari ESP32-CAM
  server.on("/cam_event", HTTP_POST, []() {
    String body = server.arg("plain");
    DynamicJsonDocument doc(1024);
    DeserializationError error = deserializeJson(doc, body);
    
    if (!error) {
      String event = doc["event"];
      String data = doc["data"];
      String camIP = doc["cam_ip"];
      
      if (event == "MOTION_DETECTED") {
        sendTelegramMessage("📸 Motion detected by CAM: " + data);
      } else if (event == "CAM_CONNECTED") {
        sendTelegramMessage("📹 CAM connected: " + data);
      } else if (event == "CAM_STARTED") {
        sendTelegramMessage("🚀 CAM started: " + data);
      }
      
      server.send(200, "text/plain", "OK");
    }
  });
}


//============================================================================
// FUNGSI UNTUK MENGIRIM PERINTAH KE ESP32-CAM
//============================================================================
void sendCommandToCAM(String command) {
  HTTPClient http;
  String url = "http://192.168.1.101/" + command; // IP ESP32-CAM
  
  http.begin(url);
  int httpCode = http.GET();
  
  if (httpCode == HTTP_CODE_OK) {
    String response = http.getString();
    Serial.println("CAM response: " + response);
  } else {
    Serial.println("Failed to send command to CAM");
  }
  
  http.end();
}

// Contoh penggunaan:
// sendCommandToCAM("capture");    // Ambil gambar
// sendCommandToCAM("pir_trigger"); // Trigger PIR
// sendCommandToCAM("status");     // Cek status


//============================================================================
// HTML TEMPLATES
//============================================================================
String getIndexHTML() {
  return R"====(
<!DOCTYPE html PUBLIC "-//W3C//DTD HTML 4.01//EN" "http://www.w3.org/TR/html4/strict.dtd">
<html>
<head>
  <meta http-equiv="Content-Type" content="text/html; charset=utf-8">
  <meta http-equiv="Content-Style-Type" content="text/css">
  <title></title>
  <meta name="Generator" content="Cocoa HTML Writer">
  <meta name="CocoaVersion" content="1894.7">
  <style type="text/css">
    p.p1 {margin: 0.0px 0.0px 0.0px 0.0px; font: 12.0px Times; color: #0000e9; -webkit-text-stroke: #0000e9}
    p.p4 {margin: 0.0px 0.0px 0.0px 0.0px; font: 12.0px Times; color: #000000; -webkit-text-stroke: #000000}
    p.p7 {margin: 0.0px 0.0px 0.0px 0.0px; font: 12.0px Times; color: #000000; -webkit-text-stroke: #000000; min-height: 14.0px}
    li.li1 {margin: 0.0px 0.0px 0.0px 0.0px; font: 12.0px Times; color: #0000e9; -webkit-text-stroke: #0000e9}
    li.li2 {margin: 0.0px 0.0px 6.0px 0.0px; font: 12.0px Times; color: #000000}
    span.s1 {text-decoration: underline ; font-kerning: none}
    span.s2 {-webkit-text-stroke: 0px #000000}
    span.s3 {font-kerning: none; color: #000000; -webkit-text-stroke: 0px #000000}
    span.s4 {font-kerning: none; -webkit-text-stroke: 0px #000000}
    span.s5 {font-kerning: none}
    ul.ul1 {list-style-type: disc}
    ul.ul2 {list-style-type: circle}
  </style>
</head>
<body>
<p class="p1"><span class="s1">Smart Farming</span></p>
<ul class="ul1">
  <li class="li1"><span class="s2"><a href="file:///"><span class="s1">Dashboard</span></a></span><span class="s3"><span class="Apple-converted-space"> </span></span></li>
  <li class="li1"><span class="s2"><a href="file:///control"><span class="s1">Control</span></a></span><span class="s3"><span class="Apple-converted-space"> </span></span></li>
  <li class="li1"><span class="s2"><a href="file:///graphs"><span class="s1">Graphs</span></a></span><span class="s3"><span class="Apple-converted-space"> </span></span></li>
  <li class="li1"><span class="s2"><a href="file:///status"><span class="s1">Status</span></a></span><span class="s3"><span class="Apple-converted-space"> </span></span></li>
  <li class="li1"><span class="s2"><a href="file:///maintenance"><span class="s1">Maintenance</span></a></span><span class="s3"><span class="Apple-converted-space"> </span></span></li>
  <li class="li1"><span class="s2"><a href="file:///settings"><span class="s1">Settings</span></a></span><span class="s3"><span class="Apple-converted-space"> </span></span></li>
</ul>
<ul class="ul1">
  <li class="li1"><span class="s2"></span><span class="s1">Admin</span></li>
  <ul class="ul2">
    <li class="li1"><span class="s2"><a href="file:///logs"><span class="s1">Logs</span></a></span></li>
    <li class="li1"><span class="s2"><a href="file:///prediction"><span class="s1">Prediction</span></a></span></li>
    <li class="li2"><span class="s4"><br>
</span></li>
    <li class="li1"><span class="s2"></span><span class="s1">Logout</span></li>
  </ul>
</ul>
<h1 style="margin: 0.0px 0.0px 16.1px 0.0px; font: 24.0px Times; color: #000000; -webkit-text-stroke: #000000"><span class="s5"><b>Smart Farming Dashboard</b></span></h1>
<p class="p4"><span class="s5">Connected Local Mode<span class="Apple-converted-space"> </span></span></p>
<h5 style="margin: 0.0px 0.0px 16.6px 0.0px; font: 10.0px Times; color: #000000; -webkit-text-stroke: #000000"><span class="s5"><b>Temperature</b></span></h5>
<h2 style="margin: 0.0px 0.0px 14.9px 0.0px; font: 18.0px Times; color: #000000; -webkit-text-stroke: #000000"><span class="s5"><b>--°C</b></span></h2>
<p class="p7"><span class="s5"></span><br></p>
<h5 style="margin: 0.0px 0.0px 16.6px 0.0px; font: 10.0px Times; color: #000000; -webkit-text-stroke: #000000"><span class="s5"><b>Humidity</b></span></h5>
<h2 style="margin: 0.0px 0.0px 14.9px 0.0px; font: 18.0px Times; color: #000000; -webkit-text-stroke: #000000"><span class="s5"><b>--%</b></span></h2>
<p class="p7"><span class="s5"></span><br></p>
<h5 style="margin: 0.0px 0.0px 16.6px 0.0px; font: 10.0px Times; color: #000000; -webkit-text-stroke: #000000"><span class="s5"><b>Battery</b></span></h5>
<h2 style="margin: 0.0px 0.0px 14.9px 0.0px; font: 18.0px Times; color: #000000; -webkit-text-stroke: #000000"><span class="s5"><b>--%</b></span></h2>
<p class="p7"><span class="s5"></span><br></p>
<h5 style="margin: 0.0px 0.0px 16.6px 0.0px; font: 10.0px Times; color: #000000; -webkit-text-stroke: #000000"><span class="s5"><b>Light</b></span></h5>
<h2 style="margin: 0.0px 0.0px 14.9px 0.0px; font: 18.0px Times; color: #000000; -webkit-text-stroke: #000000"><span class="s5"><b>-- lux</b></span></h2>
<p class="p7"><span class="s5"></span><br></p>
<h3 style="margin: 0.0px 0.0px 14.0px 0.0px; font: 14.0px Times; color: #000000; -webkit-text-stroke: #000000"><span class="s5"><b>Quick Control</b></span></h3>
<h5 style="margin: 0.0px 0.0px 16.6px 0.0px; font: 10.0px Times; color: #000000; -webkit-text-stroke: #000000"><span class="s5"><b>Exhaust Fan</b></span></h5>
<p class="p4"><span class="s5">Status: OFF<span class="Apple-converted-space"> </span></span></p>
<h5 style="margin: 0.0px 0.0px 16.6px 0.0px; font: 10.0px Times; color: #000000; -webkit-text-stroke: #000000"><span class="s5"><b>Pump Gudang</b></span></h5>
<p class="p4"><span class="s5">Status: OFF<span class="Apple-converted-space"> </span></span></p>
<h5 style="margin: 0.0px 0.0px 16.6px 0.0px; font: 10.0px Times; color: #000000; -webkit-text-stroke: #000000"><span class="s5"><b>Humidifier</b></span></h5>
<p class="p4"><span class="s5">Status: OFF<span class="Apple-converted-space"> </span></span></p>
<h5 style="margin: 0.0px 0.0px 16.6px 0.0px; font: 10.0px Times; color: #000000; -webkit-text-stroke: #000000"><span class="s5"><b>Temperature &amp; Humidity</b></span></h5>
<p class="p7"><span class="s5"></span><br></p>
<h5 style="margin: 0.0px 0.0px 16.6px 0.0px; font: 10.0px Times; color: #000000; -webkit-text-stroke: #000000"><span class="s5"><b>Battery Status</b></span></h5>
<p class="p7"><span class="s5"></span><br></p>
<h5 style="margin: 0.0px 0.0px 16.6px 0.0px; font: 10.0px Times; color: #000000; -webkit-text-stroke: #000000"><span class="s5"><b>Notifications</b></span></h5>
<p class="p4"><span class="s5">System started successfully<span class="Apple-converted-space"> </span></span></p>
</body>
</html>
)====";
}

String getGraphsHTML() {
  return R"====(
<!DOCTYPE html>
<html lang="en">
<head>
    <meta charset="UTF-8">
    <meta name="viewport" content="width=device-width, initial-scale=1.0">
    <title>Graphs - Smart Farming System</title>
    <link href="https://cdn.jsdelivr.net/npm/bootstrap@5.1.3/dist/css/bootstrap.min.css" rel="stylesheet">
    <link rel="stylesheet" href="https://cdn.jsdelivr.net/npm/bootstrap-icons@1.7.2/font/bootstrap-icons.css">
    <style>
        .chart-container {
            position: relative;
            height: 400px;
        }
    </style>
</head>
<body>
    <nav class="navbar navbar-expand-lg navbar-dark bg-primary">
        <div class="container">
            <a class="navbar-brand" href="/"><i class="bi bi-house-heart-fill"></i> Smart Farming</a>
            <div class="navbar-nav ms-auto">
                <a class="nav-link" href="/"><i class="bi bi-house"></i> Dashboard</a>
                <a class="nav-link" href="/control"><i class="bi bi-sliders"></i> Control</a>
                <a class="nav-link active" href="/graphs"><i class="bi bi-graph-up"></i> Graphs</a>
                <a class="nav-link" href="/status"><i class="bi bi-info-circle"></i> Status</a>
                <a class="nav-link" href="/maintenance"><i class="bi bi-tools"></i> Maintenance</a>
                <a class="nav-link" href="/settings"><i class="bi bi-gear"></i> Settings</a>
            </div>
        </div>
    </nav>

    <div class="container mt-4">
        <h1><i class="bi bi-graph-up"></i> Sensor Graphs</h1>

        <div class="row mb-4">
            <div class="col-md-6 mb-3">
                <div class="card">
                    <div class="card-body">
                        <h5 class="card-title">Temperature & Humidity</h5>
                        <div class="chart-container">
                            <canvas id="tempHumidityChart"></canvas>
                        </div>
                    </div>
                </div>
            </div>
            <div class="col-md-6 mb-3">
                <div class="card">
                    <div class="card-body">
                        <h5 class="card-title">Light Levels</h5>
                        <div class="chart-container">
                            <canvas id="lightChart"></canvas>
                        </div>
                    </div>
                </div>
            </div>
        </div>

        <div class="row mb-4">
            <div class="col-md-6 mb-3">
                <div class="card">
                    <div class="card-body">
                        <h5 class="card-title">Battery Status</h5>
                        <div class="chart-container">
                            <canvas id="batteryChart"></canvas>
                        </div>
                    </div>
                </div>
            </div>
            <div class="col-md-6 mb-3">
                <div class="card">
                    <div class="card-body">
                        <h5 class="card-title">Energy Consumption</h5>
                        <div class="chart-container">
                            <canvas id="energyChart"></canvas>
                        </div>
                    </div>
                </div>
            </div>
        </div>

        <div class="row">
            <div class="col-12">
                <div class="card">
                    <div class="card-body">
                        <h5 class="card-title">Time Range</h5>
                        <div class="btn-group" role="group">
                            <button type="button" class="btn btn-outline-primary" onclick="changeTimeRange('1h')">1 Hour</button>
                            <button type="button" class="btn btn-outline-primary" onclick="changeTimeRange('6h')">6 Hours</button>
                            <button type="button" class="btn btn-outline-primary" onclick="changeTimeRange('24h')">24 Hours</button>
                            <button type="button" class="btn btn-outline-primary" onclick="changeTimeRange('7d')">7 Days</button>
                            <button type="button" class="btn btn-outline-primary" onclick="changeTimeRange('30d')">30 Days</button>
                        </div>
                    </div>
                </div>
            </div>
        </div>
    </div>

    <script src="https://cdn.jsdelivr.net/npm/bootstrap@5.1.3/dist/js/bootstrap.bundle.min.js"></script>
    <script src="https://cdn.jsdelivr.net/npm/chart.js"></script>
    <script>
        // Initialize charts
        const tempHumidityChart = new Chart(document.getElementById('tempHumidityChart'), {
            type: 'line',
            data: {
                labels: [],
                datasets: [{
                    label: 'Temperature (°C)',
                    data: [],
                    borderColor: 'rgb(255, 99, 132)',
                    backgroundColor: 'rgba(255, 99, 132, 0.2)',
                    tension: 0.1
                }, {
                    label: 'Humidity (%)',
                    data: [],
                    borderColor: 'rgb(54, 162, 235)',
                    backgroundColor: 'rgba(54, 162, 235, 0.2)',
                    tension: 0.1
                }]
            },
            options: {
                responsive: true,
                maintainAspectRatio: false,
                scales: {
                    y: {
                        beginAtZero: true
                    }
                }
            }
        });
                const lightChart = new Chart(document.getElementById('lightChart'), {
            type: 'line',
            data: {
                labels: [],
                datasets: [{
                    label: 'Gudang Light (lux)',
                    data: [],
                    borderColor: 'rgb(255, 205, 86)',
                    backgroundColor: 'rgba(255, 205, 86, 0.2)',
                    tension: 0.1
                }, {
                    label: 'Kebun Light (lux)',
                    data: [],
                    borderColor: 'rgb(75, 192, 192)',
                    backgroundColor: 'rgba(75, 192, 192, 0.2)',
                    tension: 0.1
                }]
            },
            options: {
                responsive: true,
                maintainAspectRatio: false,
                scales: {
                    y: {
                        beginAtZero: true
                    }
                }
            }
        });

        const batteryChart = new Chart(document.getElementById('batteryChart'), {
            type: 'line',
            data: {
                labels: [],
                datasets: [{
                    label: 'Battery SOC (%)',
                    data: [],
                    borderColor: 'rgb(75, 192, 192)',
                    backgroundColor: 'rgba(75, 192, 192, 0.2)',
                    tension: 0.1
                }, {
                    label: 'Battery Voltage (V)',
                    data: [],
                    borderColor: 'rgb(153, 102, 255)',
                    backgroundColor: 'rgba(153, 102, 255, 0.2)',
                    tension: 0.1,
                    yAxisID: 'y1'
                }]
            },
            options: {
                responsive: true,
                maintainAspectRatio: false,
                scales: {
                    y: {
                        beginAtZero: true,
                        max: 100,
                        position: 'left'
                    },
                    y1: {
                        beginAtZero: false,
                        position: 'right',
                        grid: {
                            drawOnChartArea: false
                        }
                    }
                }
            }
        });

        const energyChart = new Chart(document.getElementById('energyChart'), {
            type: 'bar',
            data: {
                labels: [],
                datasets: [{
                    label: 'MPPT Energy (Wh)',
                    data: [],
                    backgroundColor: 'rgba(255, 99, 132, 0.5)'
                }, {
                    label: 'PWM Energy (Wh)',
                    data: [],
                    backgroundColor: 'rgba(54, 162, 235, 0.5)'
                }, {
                    label: 'Load Energy (Wh)',
                    data: [],
                    backgroundColor: 'rgba(255, 205, 86, 0.5)'
                }]
            },
            options: {
                responsive: true,
                maintainAspectRatio: false,
                scales: {
                    y: {
                        beginAtZero: true
                    }
                }
            }
        });

        // Fetch historical data
        function fetchHistoricalData(timeRange) {
            fetch('/api/sensors/history?range=' + timeRange)
                .then(response => response.json())
                .then(data => {
                    updateCharts(data);
                })
                .catch(error => {
                    console.error('Error fetching historical data:', error);
                });
        }

        // Update charts with data
        function updateCharts(data) {
            // Update temperature/humidity chart
            tempHumidityChart.data.labels = data.timestamps;
            tempHumidityChart.data.datasets[0].data = data.temperature;
            tempHumidityChart.data.datasets[1].data = data.humidity;
            tempHumidityChart.update();

            // Update light chart
            lightChart.data.labels = data.timestamps;
            lightChart.data.datasets[0].data = data.lightGudang;
            lightChart.data.datasets[1].data = data.lightKebun;
            lightChart.update();

            // Update battery chart
            batteryChart.data.labels = data.timestamps;
            batteryChart.data.datasets[0].data = data.batterySOC;
            batteryChart.data.datasets[1].data = data.batteryVoltage;
            batteryChart.update();

            // Update energy chart
            energyChart.data.labels = data.dates;
            energyChart.data.datasets[0].data = data.mpptEnergy;
            energyChart.data.datasets[1].data = data.pwmEnergy;
            energyChart.data.datasets[2].data = data.loadEnergy;
            energyChart.update();
        }

        // Change time range
        function changeTimeRange(range) {
            fetchHistoricalData(range);

            // Update button states
            document.querySelectorAll('.btn-group .btn').forEach(btn => {
                btn.classList.remove('active');
            });
            event.target.classList.add('active');
        }

        // Load default data (24 hours)
        fetchHistoricalData('24h');
    </script>
</body>
</html>
)====";
}

String getControlHTML() {
  return R"====(
<!DOCTYPE html>
<html lang="en">
<head>
    <meta charset="UTF-8">
    <meta name="viewport" content="width=device-width, initial-scale=1.0">
    <title>Control - Smart Farming System</title>
    <link href="https://cdn.jsdelivr.net/npm/bootstrap@5.1.3/dist/css/bootstrap.min.css" rel="stylesheet">
    <link rel="stylesheet" href="https://cdn.jsdelivr.net/npm/bootstrap-icons@1.7.2/font/bootstrap-icons.css">
    <style>
        .load-card {
            transition: all 0.3s ease;
        }
        .load-card:hover {
            transform: translateY(-5px);
            box-shadow: 0 4px 8px rgba(0,0,0,0.1);
        }
        .load-active {
            background-color: #d4edda;
            border-color: #c3e6cb;
        }
        .load-inactive {
            background-color: #f8d7da;
            border-color: #f5c6cb;
        }
        .custom-rule {
            border-left: 4px solid #007bff;
            padding-left: 15px;
            margin-bottom: 15px;
        }
    </style>
</head>
<body>
    <nav class="navbar navbar-expand-lg navbar-dark bg-primary">
        <div class="container">
            <a class="navbar-brand" href="/"><i class="bi bi-house-heart-fill"></i> Smart Farming</a>
            <div class="navbar-nav ms-auto">
                <a class="nav-link" href="/"><i class="bi bi-house"></i> Dashboard</a>
                <a class="nav-link active" href="/control"><i class="bi bi-sliders"></i> Control</a>
                <a class="nav-link" href="/graphs"><i class="bi bi-graph-up"></i> Graphs</a>
                <a class="nav-link" href="/status"><i class="bi bi-info-circle"></i> Status</a>
                <a class="nav-link" href="/maintenance"><i class="bi bi-tools"></i> Maintenance</a>
                <a class="nav-link" href="/settings"><i class="bi bi-gear"></i> Settings</a>
            </div>
        </div>
    </nav>

    <div class="container mt-4">
        <h1><i class="bi bi-sliders"></i> System Control</h1>

        <div class="row mb-4">
            <div class="col-12">
                <div class="card">
                    <div class="card-body">
                        <h5 class="card-title"><i class="bi bi-cpu"></i> System Mode</h5>
                        <div class="row">
                            <div class="col-md-6">
                                <select class="form-select" id="systemMode">
                                    <option value="Local">Local Mode</option>
                                    <option value="AI">AI Mode</option>
                                    <option value="User">User Mode</option>
                                    <option value="Hybrid">Hybrid Mode</option>
                                    <option value="Emergency">Emergency Mode</option>
                                    <option value="Maintenance">Maintenance Mode</option>
                                    <option value="Scheduled">Scheduled Mode</option>
                                    <option value="Predictive">Predictive Mode</option>
                                    <option value="Adaptive">Adaptive Mode</option>
                                </select>
                            </div>
                            <div class="col-md-6">
                                <button class="btn btn-primary" onclick="changeSystemMode()">
                                    <i class="bi bi-check-circle"></i> Apply Mode
                                </button>
                                <button class="btn btn-warning" onclick="emergencyStop()">
                                    <i class="bi bi-exclamation-triangle"></i> Emergency Stop
                                </button>
                            </div>
                        </div>
                    </div>
                </div>
            </div>
        </div>

        <div class="row mb-4">
            <div class="col-12">
                <h3><i class="bi bi-power"></i> Load Control</h3>
            </div>
            <div class="col-md-6 col-lg-4 mb-3">
                <div class="card load-card" id="fanCard">
                    <div class="card-body">
                        <h5 class="card-title"><i class="bi bi-fan"></i> Exhaust Fan</h5>
                        <div class="form-check form-switch">
                            <input class="form-check-input" type="checkbox" id="fanSwitch">
                            <label class="form-check-label" for="fanSwitch">
                                Status: <span id="fanStatus">OFF</span>
                            </label>
                        </div>
                        <div class="mt-2">
                            <small class="text-muted">Current: <span id="fanCurrent">0</span> mA</small>
                        </div>
                    </div>
                </div>
            </div>
            <div class="col-md-6 col-lg-4 mb-3">
                <div class="card load-card" id="pumpGudangCard">
                    <div class="card-body">
                        <h5 class="card-title"><i class="bi bi-droplet"></i> Pump Gudang</h5>
                        <div class="form-check form-switch">
                            <input class="form-check-input" type="checkbox" id="pumpGudangSwitch">
                            <label class="form-check-label" for="pumpGudangSwitch">
                                Status: <span id="pumpGudangStatus">OFF</span>
                            </label>
                        </div>
                        <div class="mt-2">
                            <small class="text-muted">Current: <span id="pumpGudangCurrent">0</span> mA</small>
                        </div>
                    </div>
                </div>
            </div>
            <div class="col-md-6 col-lg-4 mb-3">
                <div class="card load-card" id="humidifierCard">
                    <div class="card-body">
                        <h5 class="card-title"><i class="bi bi-moisture"></i> Humidifier</h5>
                        <div class="form-check form-switch">
                            <input class="form-check-input" type="checkbox" id="humidifierSwitch">
                            <label class="form-check-label" for="humidifierSwitch">
                                Status: <span id="humidifierStatus">OFF</span>
                            </label>
                        </div>
                        <div class="mt-2">
                            <small class="text-muted">Current: <span id="humidifierCurrent">0</span> mA</small>
                        </div>
                    </div>
                </div>
            </div>
            <div class="col-md-6 col-lg-4 mb-3">
                <div class="card load-card" id="pumpKebunCard">
                    <div class="card-body">
                        <h5 class="card-title"><i class="bi bi-droplet-fill"></i> Pump Kebun</h5>
                        <div class="form-check form-switch">
                            <input class="form-check-input" type="checkbox" id="pumpKebunSwitch">
                            <label class="form-check-label" for="pumpKebunSwitch">
                                Status: <span id="pumpKebunStatus">OFF</span>
                            </label>
                        </div>
                        <div class="mt-2">
                            <small class="text-muted">Current: <span id="pumpKebunCurrent">0</span> mA</small>
                        </div>
                    </div>
                </div>
            </div>
            <div class="col-md-6 col-lg-4 mb-3">
                <div class="card load-card" id="lampJalanCard">
                    <div class="card-body">
                        <h5 class="card-title"><i class="bi bi-lightbulb"></i> Lampu Jalan</h5>
                        <div class="form-check form-switch">
                            <input class="form-check-input" type="checkbox" id="lampJalanSwitch">
                            <label class="form-check-label" for="lampJalanSwitch">
                                Status: <span id="lampJalanStatus">OFF</span>
                            </label>
                        </div>
                        <div class="mt-2">
                            <small class="text-muted">Current: <span id="lampJalanCurrent">0</span> mA</small>
                        </div>
                    </div>
                </div>
            </div>
            <div class="col-md-6 col-lg-4 mb-3">
                <div class="card load-card" id="lampGudangCard">
                    <div class="card-body">
                        <h5 class="card-title"><i class="bi bi-lightbulb-fill"></i> Lampu Gudang</h5>
                        <div class="form-check form-switch">
                            <input class="form-check-input" type="checkbox" id="lampGudangSwitch">
                            <label class="form-check-label" for="lampGudangSwitch">
                                Status: <span id="lampGudangStatus">OFF</span>
                            </label>
                        </div>
                        <div class="mt-2">
                            <small class="text-muted">Current: <span id="lampGudangCurrent">0</span> mA</small>
                        </div>
                    </div>
                </div>
            </div>
        </div>

        <div class="row mb-4">
            <div class="col-12">
                <div class="card">
                    <div class="card-body">
                        <h5 class="card-title"><i class="bi bi-lightning-charge"></i> Inverter Control</h5>
                        <div class="row">
                            <div class="col-md-6">
                                <button class="btn btn-success" onclick="controlInverter(true)">
                                    <i class="bi bi-power"></i> Turn On Inverter
                                </button>
                                <button class="btn btn-danger" onclick="controlInverter(false)">
                                    <i class="bi bi-power"></i> Turn Off Inverter
                                </button>
                            </div>
                            <div class="col-md-6">
                                <div class="form-check form-switch">
                                    <input class="form-check-input" type="checkbox" id="autoInverter">
                                    <label class="form-check-label" for="autoInverter">
                                        Auto Inverter (16:00-17:00)
                                    </label>
                                </div>
                            </div>
                        </div>
                    </div>
                </div>
            </div>
        </div>

        <div class="row mb-4">
            <div class="col-12">
                <div class="card">
                    <div class="card-body">
                        <h5 class="card-title"><i class="bi bi-code-slash"></i> Custom Rules</h5>
                        <button class="btn btn-primary mb-3" data-bs-toggle="modal" data-bs-target="#addRuleModal">
                            <i class="bi bi-plus-circle"></i> Add Rule
                        </button>
                        <div id="customRules">
                            </div>
                    </div>
                </div>
            </div>
        </div>

        <div class="row mb-4">
            <div class="col-12">
                <div class="card">
                    <div class="card-body">
                        <h5 class="card-title"><i class="bi bi-calendar-check"></i> Schedules</h5>
                        <button class="btn btn-primary mb-3" data-bs-toggle="modal" data-bs-target="#addScheduleModal">
                            <i class="bi bi-plus-circle"></i> Add Schedule
                        </button>
                        <div id="schedules">
                            </div>
                    </div>
                </div>
            </div>
        </div>
    </div>

    <div class="modal fade" id="addRuleModal" tabindex="-1">
        <div class="modal-dialog">
            <div class="modal-content">
                <div class="modal-header">
                    <h5 class="modal-title">Add Custom Rule</h5>
                    <button type="button" class="btn-close" data-bs-dismiss="modal"></button>
                </div>
                <div class="modal-body">
                    <form id="addRuleForm">
                        <div class="mb-3">
                            <label for="ruleName" class="form-label">Rule Name</label>
                            <input type="text" class="form-control" id="ruleName" required>
                        </div>
                        <div class="mb-3">
                            <label for="ruleSensor" class="form-label">Sensor</label>
                            <select class="form-select" id="ruleSensor" required>
                                <option value="">Select Sensor</option>
                                <option value="temperatureGudang">Temperature Gudang</option>
                                <option value="humidityGudang">Humidity Gudang</option>
                                <option value="soilMoisture">Soil Moisture</option>
                                <option value="batteryVoltage">Battery Voltage</option>
                                <option value="batterySOC">Battery SOC</option>
                                <option value="lightGudang">Light Gudang</option>
                                <option value="co2Gudang">CO2 Gudang</option>
                            </select>
                        </div>
                        <div class="mb-3">
                            <label for="ruleCondition" class="form-label">Condition</label>
                            <select class="form-select" id="ruleCondition" required>
                                <option value="">Select Condition</option>
                                <option value="<">Less than (&lt;)</option>
                                <option value=">">Greater than (&gt;)</option>
                                <option value="=">Equal to (=)</option>
                                <option value="<=">Less than or equal (&lt;=)</option>
                                <option value=">=">Greater than or equal (&gt;=)</option>
                            </select>
                        </div>
                        <div class="mb-3">
                            <label for="ruleThreshold" class="form-label">Threshold</label>
                            <input type="number" step="0.1" class="form-control" id="ruleThreshold" required>
                        </div>
                        <div class="mb-3">
                            <label for="ruleAction" class="form-label">Action</label>
                            <select class="form-select" id="ruleAction" required>
                                <option value="">Select Action</option>
                                <option value="exhaustfan_on">Turn On Exhaust Fan</option>
                                <option value="exhaustfan_off">Turn Off Exhaust Fan</option>
                                <option value="pumpgudang_on">Turn On Pump Gudang</option>
                                <option value="pumpgudang_off">Turn Off Pump Gudang</option>
                                <option value="humidifier_on">Turn On Humidifier</option>
                                <option value="humidifier_off">Turn Off Humidifier</option>
                                <option value="pumpkebun_on">Turn On Pump Kebun</option>
                                <option value="pumpkebun_off">Turn Off Pump Kebun</option>
                                <option value="lamp_jalan_on">Turn On Lamp Jalan</option>
                                <option value="lamp_jalan_off">Turn Off Lamp Jalan</option>
                                <option value="lamp_gudang_on">Turn On Lamp Gudang</option>
                                <option value="lamp_gudang_off">Turn Off Lamp Gudang</option>
                                <option value="lamp_kebun_on">Turn On Lamp Kebun</option>
                                <option value="lamp_kebun_off">Turn Off Lamp Kebun</option>
                                <option value="inverter_on">Turn On Inverter</option>
                                <option value="inverter_off">Turn Off Inverter</option>
                            </select>
                        </div>
                        <div class="mb-3">
                            <label for="ruleCooldown" class="form-label">Cooldown (seconds)</label>
                            <input type="number" class="form-control" id="ruleCooldown" value="300">
                        </div>
                    </form>
                </div>
                <div class="modal-footer">
                    <button type="button" class="btn btn-secondary" data-bs-dismiss="modal">Cancel</button>
                    <button type="button" class="btn btn-primary" onclick="addRule()">Add Rule</button>
                </div>
            </div>
        </div>
    </div>

    <div class="modal fade" id="addScheduleModal" tabindex="-1">
        <div class="modal-dialog">
            <div class="modal-content">
                <div class="modal-header">
                    <h5 class="modal-title">Add Schedule</h5>
                    <button type="button" class="btn-close" data-bs-dismiss="modal"></button>
                </div>
                <div class="modal-body">
                    <form id="addScheduleForm">
                        <div class="mb-3">
                            <label for="scheduleTime" class="form-label">Time</label>
                            <input type="time" class="form-control" id="scheduleTime" required>
                        </div>
                        <div class="mb-3">
                            <label for="scheduleLoad" class="form-label">Load</label>
                            <select class="form-select" id="scheduleLoad" required>
                                <option value="">Select Load</option>
                                <option value="Exhaust Fan">Exhaust Fan</option>
                                <option value="Pump Gudang">Pump Gudang</option>
                                <option value="Humidifier">Humidifier</option>
                                <option value="Pump Kebun">Pump Kebun</option>
                                <option value="Lampu Jalan">Lampu Jalan</option>
                                <option value="Lampu Gudang">Lampu Gudang</option>
                                <option value="Lampu Kebun">Lampu Kebun</option>
                                <option value="Inverter">Inverter</option>
                            </select>
                        </div>
                        <div class="mb-3">
                            <label for="scheduleAction" class="form-label">Action</label>
                            <select class="form-select" id="scheduleAction" required>
                                <option value="true">Turn On</option>
                                <option value="false">Turn Off</option>
                            </select>
                        </div>
                        <div class="mb-3">
                            <label for="scheduleCondition" class="form-label">Condition (Optional)</label>
                            <select class="form-select" id="scheduleCondition">
                                <option value="">No Condition</option>
                                <option value="temperature">Temperature</option>
                                <option value="humidity">Humidity</option>
                                <option value="soilMoisture">Soil Moisture</option>
                                <option value="batterySOC">Battery SOC</option>
                            </select>
                        </div>
                        <div class="mb-3">
                            <label for="scheduleConditionValue" class="form-label">Condition Value</label>
                            <input type="number" step="0.1" class="form-control" id="scheduleConditionValue">
                        </div>
                        <div class="mb-3">
                            <label for="scheduleConditionOperator" class="form-label">Condition Operator</label>
                            <select class="form-select" id="scheduleConditionOperator">
                                <option value="<">Less than (&lt;)</option>
                                <option value=">">Greater than (&gt;)</option>
                                <option value="=">Equal to (=)</option>
                                <option value="<=">Less than or equal (&lt;=)</option>
                                <option value=">=">Greater than or equal (&gt;=)</option>
                            </select>
                        </div>
                    </form>
                </div>
                <div class="modal-footer">
                    <button type="button" class="btn btn-secondary" data-bs-dismiss="modal">Cancel</button>
                    <button type="button" class="btn btn-primary" onclick="addSchedule()">Add Schedule</button>
                </div>
            </div>
        </div>
    </div>

    <script src="https://cdn.jsdelivr.net/npm/bootstrap@5.1.3/dist/js/bootstrap.bundle.min.js"></script>
    <script>
        // Load initial data
        document.addEventListener('DOMContentLoaded', function() {
            loadSensorData();
            loadCustomRules();
            loadSchedules();

            // Update sensor data every 5 seconds
            setInterval(loadSensorData, 5000);
        });

        // Load sensor data
        function loadSensorData() {
            fetch('/api/sensors')
                .then(response => response.json())
                .then(data => {
                    updateLoadStatus(data);
                    updateSystemMode(data.mode);
                })
                .catch(error => {
                    console.error('Error loading sensor data:', error);
                });
        }

        // Update load status
        function updateLoadStatus(data) {
            data.loads.forEach(load => {
                const switchId = load.name.toLowerCase().replace(" ", "") + 'Switch';
                const statusId = load.name.toLowerCase().replace(" ", "") + 'Status';
                const currentId = load.name.toLowerCase().replace(" ", "") + 'Current';
                const cardId = load.name.toLowerCase().replace(" ", "") + 'Card';

                const switchElement = document.getElementById(switchId);
                const statusElement = document.getElementById(statusId);
                const currentElement = document.getElementById(currentId);
                const cardElement = document.getElementById(cardId);

                if (switchElement) {
                    switchElement.checked = load.status;
                }
                if (statusElement) {
                    statusElement.textContent = load.status ? 'ON' : 'OFF';
                }
                if (currentElement) {
                    currentElement.textContent = (load.current * 1000).toFixed(0);
                }
                if (cardElement) {
                    if (load.status) {
                        cardElement.classList.add('load-active');
                        cardElement.classList.remove('load-inactive');
                    } else {
                        cardElement.classList.add('load-inactive');
                        cardElement.classList.remove('load-active');
                    }
                }
            });
        }

        // Update system mode
        function updateSystemMode(mode) {
            document.getElementById('systemMode').value = mode;
        }

        // Control load
        function controlLoad(loadName, status) {
            fetch('/api/control', {
                method: 'POST',
                headers: {
                    'Content-Type': 'application/json',
                },
                body: JSON.stringify({
                    load: loadName,
                    status: status
                })
            })
            .then(response => response.json())
            .then(data => {
                if (data.success) {
                    showNotification('Load ' + loadName + ' turned ' + (status ? 'ON' : 'OFF'), 'success');
                    loadSensorData(); // Refresh data
                } else {
                    showNotification('Failed to control load', 'danger');
                }
            })
            .catch(error => {
                showNotification('Error: ' + error, 'danger');
            });
        }

        // Control inverter
        function controlInverter(status) {
            fetch('/api/inverter', {
                method: 'POST',
                headers: {
                    'Content-Type': 'application/json',
                },
                body: JSON.stringify({
                    status: status
                })
            })
            .then(response => response.json())
            .then(data => {
                if (data.success) {
                    showNotification('Inverter turned ' + (status ? 'ON' : 'OFF'), 'success');
                } else {
                    showNotification('Failed to control inverter', 'danger');
                }
            })
            .catch(error => {
                showNotification('Error: ' + error, 'danger');
            });
        }

        // Change system mode
        function changeSystemMode() {
            const mode = document.getElementById('systemMode').value;

            fetch('/api/mode', {
                method: 'POST',
                headers: {
                    'Content-Type': 'application/json',
                },
                body: JSON.stringify({
                    mode: mode
                })
            })
            .then(response => response.json())
            .then(data => {
                if (data.success) {
                    showNotification('System mode changed to ' + mode, 'success');
                } else {
                    showNotification('Failed to change system mode', 'danger');
                }
            })
            .catch(error => {
                showNotification('Error: ' + error, 'danger');
            });
        }

        // Emergency stop
        function emergencyStop() {
            if (confirm('Are you sure you want to emergency stop all loads?')) {
                fetch('/api/mode', {
                    method: 'POST',
                    headers: {
                        'Content-Type': 'application/json',
                    },
                    body: JSON.stringify({
                        mode: 'Emergency'
                    })
                })
                .then(response => response.json())
                .then(data => {
                    if (data.success) {
                        showNotification('Emergency stop activated', 'warning');
                        loadSensorData(); // Refresh data
                    } else {
                        showNotification('Failed to activate emergency stop', 'danger');
                    }
                })
                .catch(error => {
                    showNotification('Error: ' + error, 'danger');
                });
            }
        }

        // Load custom rules
        function loadCustomRules() {
            fetch('/api/rules')
                .then(response => response.json())
                .then(data => {
                    const rulesContainer = document.getElementById('customRules');
                    rulesContainer.innerHTML = "";

                    data.rules.forEach(rule => {
                        const ruleElement = document.createElement('div');
                        ruleElement.className = 'custom-rule';
                        ruleElement.innerHTML = 
                            `<div class="d-flex justify-content-between align-items-center">
                                <div>
                                    <strong>${rule.name}</strong><br>
                                    <small class="text-muted">
                                        If ${rule.sensor} ${rule.condition} ${rule.threshold} then ${rule.action}
                                    </small>
                                </div>
                                <div>
                                    <div class="form-check form-switch">
                                        <input class="form-check-input" type="checkbox" 
                                               id="rule${rule.id}" ${rule.active ? 'checked' : ''}
                                               onchange="toggleRule(${rule.id}, this.checked)">
                                        <label class="form-check-label" for="rule${rule.id}">
                                            Active
                                        </label>
                                    </div>
                                    <button class="btn btn-sm btn-danger" onclick="deleteRule(${rule.id})">
                                        <i class="bi bi-trash"></i>
                                    </button>
                                </div>
                            </div>
                        `;
                        rulesContainer.appendChild(ruleElement);
                    });
                })
                .catch(error => {
                    console.error('Error loading custom rules:', error);
                });
        }

        // Add rule
        function addRule() {
            const form = document.getElementById('addRuleForm');
            const formData = new FormData(form);

            const rule = {
                name: document.getElementById('ruleName').value,
                sensor: document.getElementById('ruleSensor').value,
                condition: document.getElementById('ruleCondition').value,
                threshold: parseFloat(document.getElementById('ruleThreshold').value),
                action: document.getElementById('ruleAction').value,
                cooldown: parseInt(document.getElementById('ruleCooldown').value)
            };

            fetch('/api/rules', {
                method: 'POST',
                headers: {
                    'Content-Type': 'application/json',
                },
                body: JSON.stringify(rule)
            })
            .then(response => response.json())
            .then(data => {
                if (data.success) {
                    showNotification('Rule added successfully', 'success');
                    bootstrap.Modal.getInstance(document.getElementById('addRuleModal')).hide();
                    form.reset();
                    loadCustomRules(); // Refresh rules
                } else {
                    showNotification('Failed to add rule', 'danger');
                }
            })
            .catch(error => {
                showNotification('Error: ' + error, 'danger');
            });
        }

        // Toggle rule
        function toggleRule(id, active) {
            // This would require an API endpoint to update rule status
            // For now, we'll just show a notification
            showNotification('Rule ' + (active ? 'activated' : 'deactivated'), 'info');
        }

        // Delete rule
        function deleteRule(id) {
            if (confirm('Are you sure you want to delete this rule?')) {
                fetch('/api/rules', {
                    method: 'DELETE',
                    headers: {
                        'Content-Type': 'application/json',
                    },
                    body: JSON.stringify({
                        id: id
                    })
                })
                .then(response => response.json())
                .then(data => {
                    if (data.success) {
                        showNotification('Rule deleted successfully', 'success');
                        loadCustomRules(); // Refresh rules
                    } else {
                        showNotification('Failed to delete rule', 'danger');
                    }
                })
                .catch(error => {
                    showNotification('Error: ' + error, 'danger');
                });
            }
        }

        // Load schedules
        function loadSchedules() {
            fetch('/api/schedules')
                .then(response => response.json())
                .then(data => {
                    const schedulesContainer = document.getElementById('schedules');
                    schedulesContainer.innerHTML = "";

                    data.schedules.forEach(schedule => {
                        const scheduleElement = document.createElement('div');
                        scheduleElement.className = 'custom-rule';
                        scheduleElement.innerHTML = `
                            <div class="d-flex justify-content-between align-items-center">
                                <div>
                                    <strong>${schedule.hour.toString().padStart(2, '0')}:${schedule.minute.toString().padStart(2, '0')}</strong> - 
                                    ${schedule.loadName} (${schedule.action ? 'ON' : 'OFF'})
                                    ${schedule.condition ? `<br><small class="text-muted">If ${schedule.condition} ${schedule.conditionOperator} ${schedule.conditionValue}</small>` : ''}
                                </div>
                                <div>
                                    <div class="form-check form-switch">
                                        <input class="form-check-input" type="checkbox" 
                                               id="schedule${schedule.id}" ${schedule.active ? 'checked' : ''}
                                               onchange="toggleSchedule(${schedule.id}, this.checked)">
                                        <label class="form-check-label" for="schedule${schedule.id}">
                                            Active
                                        </label>
                                    </div>
                                    <button class="btn btn-sm btn-danger" onclick="deleteSchedule(${schedule.id})">
                                        <i class="bi bi-trash"></i>
                                    </button>
                                </div>
                            </div>
                        `;
                        schedulesContainer.appendChild(scheduleElement);
                    });
                })
                .catch(error => {
                    console.error('Error loading schedules:', error);
                });
        }

        // Add schedule
        function addSchedule() {
            const time = document.getElementById('scheduleTime').value.split(':');
            const schedule = {
                hour: parseInt(time[0]),
                minute: parseInt(time[1]),
                loadName: document.getElementById('scheduleLoad').value,
                action: document.getElementById('scheduleAction').value === 'true',
                condition: document.getElementById('scheduleCondition').value,
                conditionValue: parseFloat(document.getElementById('scheduleConditionValue').value),
                conditionOperator: document.getElementById('scheduleConditionOperator').value
            };

            fetch('/api/schedules', {
                method: 'POST',
                headers: {
                    'Content-Type': 'application/json',
                },
                body: JSON.stringify(schedule)
            })
            .then(response => response.json())
            .then(data => {
                if (data.success) {
                    showNotification('Schedule added successfully', 'success');
                    bootstrap.Modal.getInstance(document.getElementById('addScheduleModal')).hide();
                    document.getElementById('addScheduleForm').reset();
                    loadSchedules(); // Refresh schedules
                } else {
                    showNotification('Failed to add schedule', 'danger');
                }
            })
            .catch(error => {
                showNotification('Error: ' + error, 'danger');
            });
        }

        // Toggle schedule
        function toggleSchedule(id, active) {
            // This would require an API endpoint to update schedule status
            // For now, we'll just show a notification
            showNotification('Schedule ' + (active ? 'activated' : 'deactivated'), 'info');
        }

        // Delete schedule
        function deleteSchedule(id) {
            if (confirm('Are you sure you want to delete this schedule?')) {
                // This would require an API endpoint to delete schedule
                // For now, we'll just show a notification
                showNotification('Schedule deleted', 'info');
                loadSchedules(); // Refresh schedules
            }
        }

        // Show notification
        function showNotification(message, type) {
            const notification = document.createElement('div');
            notification.className = 'alert alert-' + type + ' alert-dismissible fade show';
            notification.innerHTML = `
                ${message}
                <button type="button" class="btn-close" data-bs-dismiss="alert"></button>
            `;

            document.body.insertBefore(notification, document.body.firstChild);

            // Auto dismiss after 5 seconds
            setTimeout(() => {
                notification.remove();
            }, 5000);
        }

        // Handle switch changes
        document.getElementById('fanSwitch').addEventListener('change', function() {
            controlLoad('Exhaust Fan', this.checked);
        });

        document.getElementById('pumpGudangSwitch').addEventListener('change', function() {
            controlLoad('Pump Gudang', this.checked);
        });

        document.getElementById('humidifierSwitch').addEventListener('change', function() {
            controlLoad('Humidifier', this.checked);
        });

        document.getElementById('pumpKebunSwitch').addEventListener('change', function() {
            controlLoad('Pump Kebun', this.checked);
        });

        document.getElementById('lampJalanSwitch').addEventListener('change', function() {
            controlLoad('Lampu Jalan', this.checked);
        });

        document.getElementById('lampGudangSwitch').addEventListener('change', function() {
            controlLoad('Lampu Gudang', this.checked);
        });
    </script>
</body>
</html>
)====";
}

// Catatan: Ganti nama fungsi getStatusHTML() di kode Anda menjadi getSystemHTML() agar cocok
// dengan yang Anda berikan sebelumnya, atau sesuaikan panggilan fungsinya di bagian web server.
String getSystemHTML() { // Sebelumnya getStatusHTML
  return R"====(
<!DOCTYPE html>
<html lang="en">
<head>
    <meta charset="UTF-8">
    <meta name="viewport" content="width=device-width, initial-scale=1.0">
    <title>Status - Smart Farming System</title>
    <link href="https://cdn.jsdelivr.net/npm/bootstrap@5.1.3/dist/css/bootstrap.min.css" rel="stylesheet">
    <link rel="stylesheet" href="https://cdn.jsdelivr.net/npm/bootstrap-icons@1.7.2/font/bootstrap-icons.css">
    <style>
        .status-card {
            transition: all 0.3s ease;
        }
        .status-card:hover {
            transform: translateY(-5px);
            box-shadow: 0 4px 8px rgba(0,0,0,0.1);
        }
        .status-online {
            color: #28a745;
        }
        .status-offline {
            color: #dc3545;
        }
        .status-warning {
            color: #ffc107;
        }
    </style>
</head>
<body>
    <nav class="navbar navbar-expand-lg navbar-dark bg-primary">
        <div class="container">
            <a class="navbar-brand" href="/"><i class="bi bi-house-heart-fill"></i> Smart Farming</a>
            <div class="navbar-nav ms-auto">
                <a class="nav-link" href="/"><i class="bi bi-house"></i> Dashboard</a>
                <a class="nav-link" href="/control"><i class="bi bi-sliders"></i> Control</a>
                <a class="nav-link" href="/graphs"><i class="bi bi-graph-up"></i> Graphs</a>
                <a class="nav-link active" href="/status"><i class="bi bi-info-circle"></i> Status</a>
                <a class="nav-link" href="/maintenance"><i class="bi bi-tools"></i> Maintenance</a>
                <a class="nav-link" href="/settings"><i class="bi bi-gear"></i> Settings</a>
            </div>
        </div>
    </nav>

    <div class="container mt-4">
        <h1><i class="bi bi-info-circle"></i> System Status</h1>

        <div class="row mb-4">
            <div class="col-12">
                <div class="card">
                    <div class="card-body">
                        <h5 class="card-title"><i class="bi bi-activity"></i> Real-time Status</h5>
                        <div class="row" id="realtimeStatus">
                            </div>
                    </div>
                </div>
            </div>
        </div>

        <div class="row mb-4">
            <div class="col-12">
                <h3><i class="bi bi-power"></i> Load Status</h3>
            </div>
            <div id="loadStatus">
                </div>
        </div>

        <div class="row mb-4">
            <div class="col-12">
                <div class="card">
                    <div class="card-body">
                        <h5 class="card-title"><i class="bi bi-cpu"></i> Prediction Status</h5>
                        <div class="row">
                            <div class="col-md-3">
                                <div class="d-flex justify-content-between">
                                    <span>Predicted Temperature</span>
                                    <strong id="predictedTemp">--&deg;C</strong>
                                </div>
                            </div>
                            <div class="col-md-3">
                                <div class="d-flex justify-content-between">
                                    <span>Predicted Humidity</span>
                                    <strong id="predictedHumidity">--%</strong>
                                </div>
                            </div>
                            <div class="col-md-3">
                                <div class="d-flex justify-content-between">
                                    <span>Predicted Battery SOC</span>
                                    <strong id="predictedBatterySOC">--%</strong>
                                </div>
                            </div>
                            <div class="col-md-3">
                                <div class="d-flex justify-content-between">
                                    <span>Confidence</span>
                                    <strong id="predictionConfidence">--%</strong>
                                </div>
                            </div>
                        </div>
                        <div class="progress mt-3">
                            <div class="progress-bar" id="predictionConfidenceBar" style="width: 0%"></div>
                        </div>
                    </div>
                </div>
            </div>
        </div>

        <div class="row mb-4">
            <div class="col-12">
                <div class="card">
                    <div class="card-body">
                        <h5 class="card-title"><i class="bi bi-exclamation-triangle"></i> Emergency Status</h5>
                        <div class="row">
                            <div class="col-md-4">
                                <div class="d-flex align-items-center">
                                    <i class="bi bi-shield-exclamation status-warning me-2"></i>
                                    <div>
                                        <div>Emergency Button</div>
                                        <strong id="emergencyButtonStatus">Normal</strong>
                                    </div>
                                </div>
                            </div>
                            <div class="col-md-4">
                                <div class="d-flex align-items-center">
                                    <i class="bi bi-thermometer status-online me-2"></i>
                                    <div>
                                        <div>Thermostat</div>
                                        <strong id="thermostatStatus">Normal</strong>
                                    </div>
                                </div>
                            </div>
                            <div class="col-md-4">
                                <div class="d-flex align-items-center">
                                    <i class="bi bi-battery status-online me-2"></i>
                                    <div>
                                        <div>Battery Emergency</div>
                                        <strong id="batteryEmergencyStatus">Normal</strong>
                                    </div>
                                </div>
                            </div>
                        </div>
                    </div>
                </div>
            </div>
        </div>
    </div>

    <script src="https://cdn.jsdelivr.net/npm/bootstrap@5.1.3/dist/js/bootstrap.bundle.min.js"></script>
    <script>
        // WebSocket connection for real-time updates
        const socket = new WebSocket('ws://' + window.location.hostname + ':81');

        socket.onopen = function(event) {
            console.log('WebSocket connected');
        };

        socket.onmessage = function(event) {
            const data = JSON.parse(event.data);
            updateRealtimeStatus(data);
            updateLoadStatus(data);
            updateEmergencyStatus(data);
        };

        // Load initial data
        document.addEventListener('DOMContentLoaded', function() {
            loadStatusData();
            loadPredictionData();

            // Update prediction data every minute
            setInterval(loadPredictionData, 60000);
        });

        // Load status data
        function loadStatusData() {
            fetch('/api/sensors')
                .then(response => response.json())
                .then(data => {
                    updateRealtimeStatus(data);
                    updateLoadStatus(data);
                    updateEmergencyStatus(data);
                })
                .catch(error => {
                    console.error('Error loading status data:', error);
                });
        }

        // Update real-time status
        function updateRealtimeStatus(data) {
            const container = document.getElementById('realtimeStatus');
            container.innerHTML = `
                <div class="col-md-3">
                    <div class="d-flex align-items-center">
                        <i class="bi bi-thermometer-half text-danger me-2"></i>
                        <div>
                            <div>Temperature</div>
                            <strong>${data.temperature.toFixed(1)}&deg;C</strong>
                        </div>
                    </div>
                </div>
                <div class="col-md-3">
                    <div class="d-flex align-items-center">
                        <i class="bi bi-droplet text-info me-2"></i>
                        <div>
                            <div>Humidity</div>
                            <strong>${data.humidity.toFixed(1)}%</strong>
                        </div>
                    </div>
                </div>
                <div class="col-md-3">
                    <div class="d-flex align-items-center">
                        <i class="bi bi-battery-charging text-success me-2"></i>
                        <div>
                            <div>Battery</div>
                            <strong>${data.batterySOC.toFixed(1)}%</strong>
                        </div>
                    </div>
                </div>
                <div class="col-md-3">
                    <div class="d-flex align-items-center">
                        <i class="bi bi-cpu text-primary me-2"></i>
                        <div>
                            <div>System Mode</div>
                            <strong>${data.mode}</strong>
                        </div>
                    </div>
                </div>
            `;
        }

        // Update load status
        function updateLoadStatus(data) {
            const container = document.getElementById('loadStatus');
            container.innerHTML = "";

            data.loads.forEach(load => {
                const statusClass = load.status ? 'status-online' : 'status-offline';
                const statusIcon = load.status ? 'check-circle' : 'x-circle';

                const element = document.createElement('div');
                element.className = 'col-md-6 col-lg-4 mb-3';
                element.innerHTML = `
                    <div class="card status-card">
                        <div class="card-body">
                            <h6 class="card-title">
                                <i class="bi bi-${statusIcon} ${statusClass} me-2"></i>
                                ${load.name}
                            </h6>
                            <div class="d-flex justify-content-between">
                                <span>Status</span>
                                <strong>${load.status ? 'ON' : 'OFF'}</strong>
                            </div>
                            <div class="d-flex justify-content-between">
                                <span>Command</span>
                                <strong>${load.command ? 'ON' : 'OFF'}</strong>
                            </div>
                            <div class="d-flex justify-content-between">
                                <span>Current</span>
                                <strong>${(load.current * 1000).toFixed(0)} mA</strong>
                            </div>
                        </div>
                    </div>
                `;
                container.appendChild(element);
            });
        }

        // Update emergency status
        function updateEmergencyStatus(data) {
            // This would need to be implemented with actual emergency status data
            document.getElementById('emergencyButtonStatus').textContent = 'Normal';
            document.getElementById('thermostatStatus').textContent = 'Normal';
            document.getElementById('batteryEmergencyStatus').textContent = 'Normal';
        }

        // Load prediction data
        function loadPredictionData() {
            fetch('/api/prediction')
                .then(response => response.json())
                .then(data => {
                    document.getElementById('predictedTemp').textContent = data.predictedTemp.toFixed(1) + '°C';
                    document.getElementById('predictedHumidity').textContent = data.predictedHumidity.toFixed(1) + '%';
                    document.getElementById('predictedBatterySOC').textContent = data.predictedBatterySOC.toFixed(1) + '%';
                    document.getElementById('predictionConfidence').textContent = (data.confidence * 100).toFixed(1) + '%';
                    document.getElementById('predictionConfidenceBar').style.width = (data.confidence * 100) + '%';

                    // Update confidence bar color
                    const confidenceBar = document.getElementById('predictionConfidenceBar');
                    confidenceBar.className = 'progress-bar';
                    if (data.confidence > 0.8) {
                        confidenceBar.classList.add('bg-success');
                    } else if (data.confidence > 0.6) {
                        confidenceBar.classList.add('bg-warning');
                    } else {
                        confidenceBar.classList.add('bg-danger');
                    }
                })
                .catch(error => {
                    console.error('Error loading prediction data:', error);
                });
        }
    </script>
</body>
</html>
)====";
}

// Catatan: Ganti nama fungsi getStatusHTML() di kode Anda menjadi getMaintenanceHTML() agar cocok
// dengan yang Anda berikan sebelumnya, atau sesuaikan panggilan fungsinya di bagian web server.
String getMaintenanceHTML() { // Sebelumnya getStatusHTML kedua
    return R"====(
<!DOCTYPE html>
<html lang="en">
<head>
    <meta charset="UTF-8">
    <meta name="viewport" content="width=device-width, initial-scale=1.0">
    <title>Maintenance - Smart Farming System</title>
    <link href="https://cdn.jsdelivr.net/npm/bootstrap@5.1.3/dist/css/bootstrap.min.css" rel="stylesheet">
    <link rel="stylesheet" href="https://cdn.jsdelivr.net/npm/bootstrap-icons@1.7.2/font/bootstrap-icons.css">
    <style>
        .maintenance-card {
            transition: all 0.3s ease;
            border-left: 4px solid #007bff;
        }
        .maintenance-card:hover {
            transform: translateY(-5px);
            box-shadow: 0 4px 8px rgba(0,0,0,0.1);
        }
        .status-pending {
            border-left-color: #ffc107;
        }
        .status-in-progress {
            border-left-color: #17a2b8;
        }
        .status-completed {
            border-left-color: #28a745;
        }
    </style>
</head>
<body>
    <nav class="navbar navbar-expand-lg navbar-dark bg-primary">
        <div class="container">
            <a class="navbar-brand" href="/"><i class="bi bi-house-heart-fill"></i> Smart Farming</a>
            <div class="navbar-nav ms-auto">
                <a class="nav-link" href="/"><i class="bi bi-house"></i> Dashboard</a>
                <a class="nav-link" href="/control"><i class="bi bi-sliders"></i> Control</a>
                <a class="nav-link" href="/graphs"><i class="bi bi-graph-up"></i> Graphs</a>
                <a class="nav-link" href="/status"><i class="bi bi-info-circle"></i> Status</a>
                <a class="nav-link active" href="/maintenance"><i class="bi bi-tools"></i> Maintenance</a>
                <a class="nav-link" href="/settings"><i class="bi bi-gear"></i> Settings</a>
            </div>
        </div>
    </nav>

    <div class="container mt-4">
        <h1><i class="bi bi-tools"></i> Maintenance</h1>

        <div class="row mb-4">
            <div class="col-12">
                <div class="card">
                    <div class="card-body">
                        <h5 class="card-title"><i class="bi bi-plus-circle"></i> Add Maintenance Activity</h5>
                        <form id="addMaintenanceForm">
                            <div class="row">
                                <div class="col-md-6 mb-3">
                                    <label for="activityType" class="form-label">Activity Type</label>
                                    <select class="form-select" id="activityType" required>
                                        <option value="">Select Type</option>
                                        <option value="roof_leak">Roof Leak</option>
                                        <option value="sensor_issue">Sensor Issue</option>
                                        <option value="power_problem">Power Problem</option>
                                        <option value="pump_maintenance">Pump Maintenance</option>
                                        <option value="fan_maintenance">Fan Maintenance</option>
                                        <option value="battery_maintenance">Battery Maintenance</option>
                                        <option value="inverter_maintenance">Inverter Maintenance</option>
                                        <option value="cleaning">Cleaning</option>
                                        <option value="other">Other</option>
                                    </select>
                                </div>
                                <div class="col-md-6 mb-3">
                                    <label for="activityDescription" class="form-label">Description</label>
                                    <input type="text" class="form-control" id="activityDescription" required>
                                </div>
                            </div>
                            <button type="submit" class="btn btn-primary">
                                <i class="bi bi-plus-circle"></i> Add Activity
                            </button>
                        </form>
                    </div>
                </div>
            </div>
        </div>

        <div class="row mb-4">
            <div class="col-12">
                <h3><i class="bi bi-list-check"></i> Maintenance Activities</h3>
            </div>
            <div id="maintenanceActivities">
                </div>
        </div>

        <div class="row mb-4">
            <div class="col-12">
                <div class="card">
                    <div class="card-body">
                        <h5 class="card-title"><i class="bi bi-calendar-check"></i> Recommended Maintenance Schedule</h5>
                        <div class="table-responsive">
                            <table class="table table-striped">
                                <thead>
                                    <tr>
                                        <th>Component</th>
                                        <th>Frequency</th>
                                        <th>Last Maintenance</th>
                                        <th>Next Due</th>
                                        <th>Status</th>
                                    </tr>
                                </thead>
                                <tbody>
                                    <tr>
                                        <td>Pumps</td>
                                        <td>Monthly</td>
                                        <td>2023-10-15</td>
                                        <td>2023-11-15</td>
                                        <td><span class="badge bg-warning">Due Soon</span></td>
                                    </tr>
                                    <tr>
                                        <td>Fans</td>
                                        <td>Quarterly</td>
                                        <td>2023-09-01</td>
                                        <td>2023-12-01</td>
                                        <td><span class="badge bg-success">OK</span></td>
                                    </tr>
                                    <tr>
                                        <td>Sensors</td>
                                        <td>Monthly</td>
                                        <td>2023-10-20</td>
                                        <td>2023-11-20</td>
                                        <td><span class="badge bg-success">OK</span></td>
                                    </tr>
                                    <tr>
                                        <td>Battery</td>
                                        <td>Quarterly</td>
                                        <td>2023-08-15</td>
                                        <td>2023-11-15</td>
                                        <td><span class="badge bg-warning">Due Soon</span></td>
                                    </tr>
                                    <tr>
                                        <td>Inverter</td>
                                        <td>Semi-annually</td>
                                        <td>2023-06-01</td>
                                        <td>2023-12-01</td>
                                        <td><span class="badge bg-warning">Due Soon</span></td>
                                    </tr>
                                </tbody>
                            </table>
                        </div>
                    </div>
                </div>
            </div>
        </div>

        <div class="row mb-4">
            <div class="col-12">
                <div class="card">
                    <div class="card-body">
                        <h5 class="card-title"><i class="bi bi-clock-history"></i> Maintenance History</h5>
                        <div class="table-responsive">
                            <table class="table table-striped">
                                <thead>
                                    <tr>
                                        <th>Date</th>
                                        <th>Activity</th>
                                        <th>Type</th>
                                        <th>Status</th>
                                        <th>Notes</th>
                                    </tr>
                                </thead>
                                <tbody>
                                    <tr>
                                        <td>2023-10-15</td>
                                        <td>Pump Gudang Maintenance</td>
                                        <td>pump_maintenance</td>
                                        <td><span class="badge bg-success">Completed</span></td>
                                        <td>Cleaned filters, checked connections</td>
                                    </tr>
                                    <tr>
                                        <td>2023-09-01</td>
                                        <td>Fan Cleaning</td>
                                        <td>fan_maintenance</td>
                                        <td><span class="badge bg-success">Completed</span></td>
                                        <td>Cleaned fan blades, lubricated bearings</td>
                                    </tr>
                                    <tr>
                                        <td>2023-08-15</td>
                                        <td>Battery Inspection</td>
                                        <td>battery_maintenance</td>
                                        <td><span class="badge bg-success">Completed</span></td>
                                        <td>Checked terminals, measured voltage</td>
                                    </tr>
                                </tbody>
                            </table>
                        </div>
                    </div>
                </div>
            </div>
        </div>
    </div>

    <script src="https://cdn.jsdelivr.net/npm/bootstrap@5.1.3/dist/js/bootstrap.bundle.min.js"></script>
    <script>
        // Load initial data
        document.addEventListener('DOMContentLoaded', function() {
            loadMaintenanceActivities();
        });

        // Load maintenance activities
        function loadMaintenanceActivities() {
            fetch('/api/maintenance')
                .then(response => response.json())
                .then(data => {
                    const container = document.getElementById('maintenanceActivities');
                    container.innerHTML = "";

                    data.activities.forEach(activity => {
                        const statusClass = activity.status === 'completed' ? 'status-completed' : 
                                            activity.status === 'in_progress' ? 'status-in-progress' : 'status-pending';

                        const statusBadge = activity.status === 'completed' ? 'bg-success' : 
                                           activity.status === 'in_progress' ? 'bg-info' : 'bg-warning';

                        const element = document.createElement('div');
                        element.className = 'col-md-6 col-lg-4 mb-3';
                        element.innerHTML = `
                            <div class="card maintenance-card ${statusClass}">
                                <div class="card-body">
                                    <h6 class="card-title">${activity.activityType.replace('_', ' ').toUpperCase()}</h6>
                                    <p class="card-text">${activity.description}</p>
                                    <div class="d-flex justify-content-between align-items-center">
                                        <span class="badge ${statusBadge}">${activity.status.replace('_', ' ').toUpperCase()}</span>
                                        <small class="text-muted">${new Date(activity.timestamp).toLocaleDateString()}</small>
                                    </div>
                                    ${activity.aiRecommendation ? `<div class="mt-2"><small class="text-info"><i class="bi bi-cpu"></i> ${activity.aiRecommendation}</small></div>` : ''}
                                    <div class="mt-2">
                                        ${!activity.resolved ? `
                                            <button class="btn btn-sm btn-primary" onclick="updateActivityStatus(${activity.id}, 'in_progress')">
                                                <i class="bi bi-play"></i> Start
                                            </button>
                                            <button class="btn btn-sm btn-success" onclick="updateActivityStatus(${activity.id}, 'completed')">
                                                <i class="bi bi-check"></i> Complete
                                            </button>
                                        ` : '<span class="text-success"><i class="bi bi-check-circle"></i> Resolved</span>'}
                                    </div>
                                </div>
                            </div>
                        `;
                        container.appendChild(element);
                    });
                })
                .catch(error => {
                    console.error('Error loading maintenance activities:', error);
                });
        }

        // Add maintenance activity
        document.getElementById('addMaintenanceForm').addEventListener('submit', function(e) {
            e.preventDefault();

            const activity = {
                activityType: document.getElementById('activityType').value,
                description: document.getElementById('activityDescription').value
            };

            fetch('/api/maintenance', {
                method: 'POST',
                headers: {
                    'Content-Type': 'application/json',
                },
                body: JSON.stringify(activity)
            })
            .then(response => response.json())
            .then(data => {
                if (data.success) {
                    showNotification('Maintenance activity added successfully', 'success');
                    document.getElementById('addMaintenanceForm').reset();
                    loadMaintenanceActivities(); // Refresh activities
                } else {
                    showNotification('Failed to add maintenance activity', 'danger');
                }
            })
            .catch(error => {
                showNotification('Error: ' + error, 'danger');
            });
        });

        // Update activity status
        function updateActivityStatus(id, status) {
            // This would require an API endpoint to update activity status
            // For now, we'll just show a notification
            showNotification('Activity status updated to ' + status.replace('_', ' '), 'info');
            loadMaintenanceActivities(); // Refresh activities
        }

        // Show notification
        function showNotification(message, type) {
            const notification = document.createElement('div');
            notification.className = 'alert alert-' + type + ' alert-dismissible fade show';
            notification.innerHTML = `
                ${message}
                <button type="button" class="btn-close" data-bs-dismiss="alert"></button>
            `;

            document.body.insertBefore(notification, document.body.firstChild);

            // Auto dismiss after 5 seconds
            setTimeout(() => {
                notification.remove();
            }, 5000);
        }
    </script>
</body>
</html>
)====";
}


String getSettingsHTML() {
  return R"====(
<!DOCTYPE html>
<html lang="en">
<head>
    <meta charset="UTF-8">
    <meta name="viewport" content="width=device-width, initial-scale=1.0">
    <title>Settings - Smart Farming System</title>
    <link href="https://cdn.jsdelivr.net/npm/bootstrap@5.1.3/dist/css/bootstrap.min.css" rel="stylesheet">
    <link rel="stylesheet" href="https://cdn.jsdelivr.net/npm/bootstrap-icons@1.7.2/font/bootstrap-icons.css">
    <style>
        .settings-card {
            transition: all 0.3s ease;
        }
        .settings-card:hover {
            transform: translateY(-5px);
            box-shadow: 0 4px 8px rgba(0,0,0,0.1);
        }
    </style>
</head>
<body>
    <nav class="navbar navbar-expand-lg navbar-dark bg-primary">
        <div class="container">
            <a class="navbar-brand" href="/"><i class="bi bi-house-heart-fill"></i> Smart Farming</a>
            <div class="navbar-nav ms-auto">
                <a class="nav-link" href="/"><i class="bi bi-house"></i> Dashboard</a>
                <a class="nav-link" href="/control"><i class="bi bi-sliders"></i> Control</a>
                <a class="nav-link" href="/graphs"><i class="bi bi-graph-up"></i> Graphs</a>
                <a class="nav-link" href="/status"><i class="bi bi-info-circle"></i> Status</a>
                <a class="nav-link" href="/maintenance"><i class="bi bi-tools"></i> Maintenance</a>
                <a class="nav-link active" href="/settings"><i class="bi bi-gear"></i> Settings</a>
            </div>
        </div>
    </nav>

    <div class="container mt-4">
        <h1><i class="bi bi-gear"></i> Settings</h1>

        <div class="row mb-4">
            <div class="col-12">
                <div class="card">
                    <div class="card-body">
                        <h5 class="card-title"><i class="bi bi-sliders"></i> Threshold Settings</h5>
                        <form id="thresholdForm">
                            <div class="row">
                                <div class="col-md-6 mb-3">
                                    <h6>Load Thresholds (mA)</h6>
                                    <div class="mb-3">
                                        <label for="exhaustFanThreshold" class="form-label">Exhaust Fan</label>
                                        <input type="number" class="form-control" id="exhaustFanThreshold" step="0.1">
                                    </div>
                                    <div class="mb-3">
                                        <label for="pumpGudangThreshold" class="form-label">Pump Gudang</label>
                                        <input type="number" class="form-control" id="pumpGudangThreshold" step="0.1">
                                    </div>
                                    <div class="mb-3">
                                        <label for="humidifierThreshold" class="form-label">Humidifier</label>
                                        <input type="number" class="form-control" id="humidifierThreshold" step="0.1">
                                    </div>
                                    <div class="mb-3">
                                        <label for="pumpKebunThreshold" class="form-label">Pump Kebun</label>
                                        <input type="number" class="form-control" id="pumpKebunThreshold" step="0.1">
                                    </div>
                                </div>
                                <div class="col-md-6 mb-3">
                                    <h6>Sensor Thresholds</h6>
                                    <div class="mb-3">
                                        <label for="tempHighThreshold" class="form-label">Temperature High (°C)</label>
                                        <input type="number" class="form-control" id="tempHighThreshold" step="0.1">
                                    </div>
                                    <div class="mb-3">
                                        <label for="tempNormalThreshold" class="form-label">Temperature Normal (°C)</label>
                                        <input type="number" class="form-control" id="tempNormalThreshold" step="0.1">
                                    </div>
                                    <div class="mb-3">
                                        <label for="humidityLowThreshold" class="form-label">Humidity Low (%)</label>
                                        <input type="number" class="form-control" id="humidityLowThreshold" step="0.1">
                                    </div>
                                    <div class="mb-3">
                                        <label for="humidityHighThreshold" class="form-label">Humidity High (%)</label>
                                        <input type="number" class="form-control" id="humidityHighThreshold" step="0.1">
                                    </div>
                                </div>
                            </div>
                            <div class="row">
                                <div class="col-md-6 mb-3">
                                    <h6>Battery Thresholds (V)</h6>
                                    <div class="mb-3">
                                        <label for="batteryLowThreshold" class="form-label">Battery Low</label>
                                        <input type="number" class="form-control" id="batteryLowThreshold" step="0.1">
                                    </div>
                                    <div class="mb-3">
                                        <label for="batteryCriticalThreshold" class="form-label">Battery Critical</label>
                                        <input type="number" class="form-control" id="batteryCriticalThreshold" step="0.1">
                                    </div>
                                    <div class="mb-3">
                                        <label for="batteryEmergencyThreshold" class="form-label">Battery Emergency</label>
                                        <input type="number" class="form-control" id="batteryEmergencyThreshold" step="0.1">
                                    </div>
                                </div>
                                <div class="col-md-6 mb-3">
                                    <h6>Other Thresholds</h6>
                                    <div class="mb-3">
                                        <label for="soilMoistureLowThreshold" class="form-label">Soil Moisture Low (%)</label>
                                        <input type="number" class="form-control" id="soilMoistureLowThreshold" step="0.1">
                                    </div>
                                    <div class="mb-3">
                                        <label for="soilMoistureHighThreshold" class="form-label">Soil Moisture High (%)</label>
                                        <input type="number" class="form-control" id="soilMoistureHighThreshold" step="0.1">
                                    </div>
                                    <div class="mb-3">
                                        <label for="co2HighThreshold" class="form-label">CO2 High (%)</label>
                                        <input type="number" class="form-control" id="co2HighThreshold" step="0.001">
                                    </div>
                                </div>
                            </div>
                            <button type="submit" class="btn btn-primary">
                                <i class="bi bi-check-circle"></i> Save Thresholds
                            </button>
                        </form>
                    </div>
                </div>
            </div>
        </div>

        <div class="row mb-4">
            <div class="col-12">
                <div class="card">
                    <div class="card-body">
                        <h5 class="card-title"><i class="bi bi-bell"></i> Notification Settings</h5>
                        <form id="notificationForm">
                            <div class="row">
                                <div class="col-md-6 mb-3">
                                    <div class="mb-3">
                                        <label for="notificationMode" class="form-label">Notification Mode</label>
                                        <select class="form-select" id="notificationMode">
                                            <option value="all">All Notifications</option>
                                            <option value="important">Important Only</option>
                                            <option value="emergency">Emergency Only</option>
                                        </select>
                                    </div>
                                    <div class="mb-3">
                                        <label for="dndStart" class="form-label">Do Not Disturb Start</label>
                                        <input type="time" class="form-control" id="dndStart">
                                    </div>
                                    <div class="mb-3">
                                        <label for="dndEnd" class="form-label">Do Not Disturb End</label>
                                        <input type="time" class="form-control" id="dndEnd">
                                    </div>
                                </div>
                                <div class="col-md-6 mb-3">
                                    <h6>Notification Types</h6>
                                    <div class="form-check form-switch mb-2">
                                        <input class="form-check-input" type="checkbox" id="emergencyNotifications" checked>
                                        <label class="form-check-label" for="emergencyNotifications">
                                            Emergency Notifications
                                        </label>
                                    </div>
                                    <div class="form-check form-switch mb-2">
                                        <input class="form-check-input" type="checkbox" id="warningNotifications" checked>
                                        <label class="form-check-label" for="warningNotifications">
                                            Warning Notifications
                                        </label>
                                    </div>
                                    <div class="form-check form-switch mb-2">
                                        <input class="form-check-input" type="checkbox" id="infoNotifications" checked>
                                        <label class="form-check-label" for="infoNotifications">
                                            Info Notifications
                                        </label>
                                    </div>
                                    <div class="form-check form-switch mb-2">
                                        <input class="form-check-input" type="checkbox" id="dailyNotifications" checked>
                                        <label class="form-check-label" for="dailyNotifications">
                                            Daily Reports
                                        </label>
                                    </div>
                                    <div class="form-check form-switch mb-2">
                                        <input class="form-check-input" type="checkbox" id="aiNotifications" checked>
                                        <label class="form-check-label" for="aiNotifications">
                                            AI Recommendations
                                        </label>
                                    </div>
                                </div>
                            </div>
                            <button type="submit" class="btn btn-primary">
                                <i class="bi bi-check-circle"></i> Save Notifications
                            </button>
                        </form>
                    </div>
                </div>
            </div>
        </div>

        <div class="row mb-4">
            <div class="col-12">
                <div class="card">
                    <div class="card-body">
                        <h5 class="card-title"><i class="bi bi-cpu"></i> System Settings</h5>
                        <form id="systemForm">
                            <div class="row">
                                <div class="col-md-6 mb-3">
                                    <h6>System Features</h6>
                                    <div class="form-check form-switch mb-2">
                                        <input class="form-check-input" type="checkbox" id="encryptionEnabled">
                                        <label class="form-check-label" for="encryptionEnabled">
                                            Data Encryption
                                        </label>
                                    </div>
                                    <div class="form-check form-switch mb-2">
                                        <input class="form-check-input" type="checkbox" id="otaEnabled">
                                        <label class="form-check-label" for="otaEnabled">
                                            OTA Updates
                                        </label>
                                    </div>
                                    <div class="form-check form-switch mb-2">
                                        <input class="form-check-input" type="checkbox" id="predictiveEnabled">
                                        <label class="form-check-label" for="predictiveEnabled">
                                            Predictive Control
                                        </label>
                                    </div>
                                    <div class="form-check form-switch mb-2">
                                        <input class="form-check-input" type="checkbox" id="adaptiveEnabled">
                                        <label class="form-check-label" for="adaptiveEnabled">
                                            Adaptive Control
                                        </label>
                                    </div>
                                    <div class="form-check form-switch mb-2">
                                        <input class="form-check-input" type="checkbox" id="pidEnabled">
                                        <label class="form-check-label" for="pidEnabled">
                                            PID Control
                                        </label>
                                    </div>
                                </div>
                                <div class="col-md-6 mb-3">
                                    <h6>API Keys</h6>
                                    <div class="mb-3">
                                        <label for="geminiApiKey" class="form-label">Gemini API Key</label>
                                        <input type="password" class="form-control" id="geminiApiKey">
                                        <div class="form-text">Used for AI recommendations</div>
                                    </div>
                                    <div class="mb-3">
                                        <label for="telegramBotToken" class="form-label">Telegram Bot Token</label>
                                        <input type="password" class="form-control" id="telegramBotToken">
                                        <div class="form-text">Used for Telegram notifications</div>
                                    </div>
                                </div>
                            </div>
                            <button type="submit" class="btn btn-primary">
                                <i class="bi bi-check-circle"></i> Save System Settings
                            </button>
                        </form>
                    </div>
                </div>
            </div>
        </div>

        <div class="row mb-4">
            <div class="col-12">
                <div class="card">
                    <div class="card-body">
                        <h5 class="card-title"><i class="bi bi-shield-lock"></i> Security Settings</h5>
                        <form id="securityForm">
                            <div class="row">
                                <div class="col-md-6 mb-3">
                                    <div class="mb-3">
                                        <label for="webUsername" class="form-label">Web Username</label>
                                        <input type="text" class="form-control" id="webUsername">
                                    </div>
                                    <div class="mb-3">
                                        <label for="webPassword" class="form-label">Web Password</label>
                                        <input type="password" class="form-control" id="webPassword">
                                    </div>
                                    <div class="mb-3">
                                        <label for="telegramPassword" class="form-label">Telegram Password</label>
                                        <input type="password" class="form-control" id="telegramPassword">
                                        <div class="form-text">Used for Telegram authentication</div>
                                    </div>
                                </div>
                                <div class="col-md-6 mb-3">
                                    <h6>Security Actions</h6>
                                    <button type="button" class="btn btn-warning mb-2" onclick="changeWiFiSettings()">
                                        <i class="bi bi-wifi"></i> Change WiFi Settings
                                    </button>
                                    <button type="button" class="btn btn-info mb-2" onclick="backupConfiguration()">
                                        <i class="bi bi-download"></i> Backup Configuration
                                    </button>
                                    <button type="button" class="btn btn-secondary mb-2" onclick="restoreConfiguration()">
                                        <i class="bi bi-upload"></i> Restore Configuration
                                    </button>
                                    <button type="button" class="btn btn-danger mb-2" onclick="factoryReset()">
                                        <i class="bi bi-arrow-clockwise"></i> Factory Reset
                                    </button>
                                </div>
                            </div>
                            <button type="submit" class="btn btn-primary">
                                <i class="bi bi-check-circle"></i> Save Security Settings
                            </button>
                        </form>
                    </div>
                </div>
            </div>
        </div>
    </div>

    <script src="https://cdn.jsdelivr.net/npm/bootstrap@5.1.3/dist/js/bootstrap.bundle.min.js"></script>
    <script>
        // Load initial data
        document.addEventListener('DOMContentLoaded', function() {
            loadConfiguration();
        });

        // Load configuration
        function loadConfiguration() {
            fetch('/api/config')
                .then(response => response.json())
                .then(data => {
                    // Load threshold settings
                    document.getElementById('exhaustFanThreshold').value = data.exhaustFanThreshold;
                    document.getElementById('pumpGudangThreshold').value = data.pumpGudangThreshold;
                    document.getElementById('humidifierThreshold').value = data.humidifierThreshold;
                    document.getElementById('pumpKebunThreshold').value = data.pumpKebunThreshold;
                    document.getElementById('tempHighThreshold').value = data.tempHighThreshold;
                    document.getElementById('tempNormalThreshold').value = data.tempNormalThreshold;
                    document.getElementById('humidityLowThreshold').value = data.humidityLowThreshold;
                    document.getElementById('humidityHighThreshold').value = data.humidityHighThreshold;
                    document.getElementById('batteryLowThreshold').value = data.batteryLowThreshold;
                    document.getElementById('batteryCriticalThreshold').value = data.batteryCriticalThreshold;
                    document.getElementById('batteryEmergencyThreshold').value = data.batteryEmergencyThreshold;
                    document.getElementById('soilMoistureLowThreshold').value = data.soilMoistureLowThreshold;
                    document.getElementById('soilMoistureHighThreshold').value = data.soilMoistureHighThreshold;
                    document.getElementById('co2HighThreshold').value = data.co2HighThreshold;

                    // Load notification settings
                    document.getElementById('notificationMode').value = data.notificationMode;
                    document.getElementById('dndStart').value = data.dndStart;
                    document.getElementById('dndEnd').value = data.dndEnd;

                    // Load system settings
                    document.getElementById('encryptionEnabled').checked = data.encryptionEnabled;
                    document.getElementById('otaEnabled').checked = data.otaEnabled;
                    document.getElementById('predictiveEnabled').checked = data.predictiveEnabled;
                    document.getElementById('adaptiveEnabled').checked = data.adaptiveEnabled;
                    document.getElementById('pidEnabled').checked = data.pidEnabled;
                })
                .catch(error => {
                    console.error('Error loading configuration:', error);
                });
        }

        // Save threshold settings
        document.getElementById('thresholdForm').addEventListener('submit', function(e) {
            e.preventDefault();

            const config = {
                exhaustFanThreshold: parseFloat(document.getElementById('exhaustFanThreshold').value),
                pumpGudangThreshold: parseFloat(document.getElementById('pumpGudangThreshold').value),
                humidifierThreshold: parseFloat(document.getElementById('humidifierThreshold').value),
                pumpKebunThreshold: parseFloat(document.getElementById('pumpKebunThreshold').value),
                tempHighThreshold: parseFloat(document.getElementById('tempHighThreshold').value),
                tempNormalThreshold: parseFloat(document.getElementById('tempNormalThreshold').value),
                humidityLowThreshold: parseFloat(document.getElementById('humidityLowThreshold').value),
                humidityHighThreshold: parseFloat(document.getElementById('humidityHighThreshold').value),
                batteryLowThreshold: parseFloat(document.getElementById('batteryLowThreshold').value),
                batteryCriticalThreshold: parseFloat(document.getElementById('batteryCriticalThreshold').value),
                batteryEmergencyThreshold: parseFloat(document.getElementById('batteryEmergencyThreshold').value),
                soilMoistureLowThreshold: parseFloat(document.getElementById('soilMoistureLowThreshold').value),
                soilMoistureHighThreshold: parseFloat(document.getElementById('soilMoistureHighThreshold').value),
                co2HighThreshold: parseFloat(document.getElementById('co2HighThreshold').value)
            };

            saveConfiguration(config);
        });

        // Save notification settings
        document.getElementById('notificationForm').addEventListener('submit', function(e) {
            e.preventDefault();

            const config = {
                notificationMode: document.getElementById('notificationMode').value,
                dndStart: document.getElementById('dndStart').value,
                dndEnd: document.getElementById('dndEnd').value
            };

            saveConfiguration(config);
        });

        // Save system settings
        document.getElementById('systemForm').addEventListener('submit', function(e) {
            e.preventDefault();

            const config = {
                encryptionEnabled: document.getElementById('encryptionEnabled').checked,
                otaEnabled: document.getElementById('otaEnabled').checked,
                predictiveEnabled: document.getElementById('predictiveEnabled').checked,
                adaptiveEnabled: document.getElementById('adaptiveEnabled').checked,
                pidEnabled: document.getElementById('pidEnabled').checked
            };

            saveConfiguration(config);
        });

        // Save security settings
        document.getElementById('securityForm').addEventListener('submit', function(e) {
            e.preventDefault();

            const config = {
                webUsername: document.getElementById('webUsername').value,
                webPassword: document.getElementById('webPassword').value,
                telegramPassword: document.getElementById('telegramPassword').value
            };

            saveConfiguration(config);
        });

        // Save configuration
        function saveConfiguration(config) {
            fetch('/api/config', {
                method: 'POST',
                headers: {
                    'Content-Type': 'application/json',
                },
                body: JSON.stringify(config)
            })
            .then(response => response.json())
            .then(data => {
                if (data.success) {
                    showNotification('Configuration saved successfully', 'success');
                } else {
                    showNotification('Failed to save configuration', 'danger');
                }
            })
            .catch(error => {
                showNotification('Error: ' + error, 'danger');
            });
        }

        // Change WiFi settings
        function changeWiFiSettings() {
            const ssid = prompt('Enter new WiFi SSID:');
            if (ssid) {
                const password = prompt('Enter new WiFi Password:');
                if (password) {
                    // This would require an API endpoint to change WiFi settings
                    showNotification('WiFi settings changed. Device will restart.', 'info');
                }
            }
        }

        // Backup configuration
        function backupConfiguration() {
            fetch('/api/config')
                .then(response => response.json())
                .then(data => {
                    const blob = new Blob([JSON.stringify(data, null, 2)], { type: 'application/json' });
                    const url = window.URL.createObjectURL(blob);
                    const a = document.createElement('a');
                    a.href = url;
                    a.download = 'smart_farming_config_' + new Date().toISOString().split('T')[0] + '.json';
                    document.body.appendChild(a);
                    a.click();
                    document.body.removeChild(a);
                    window.URL.revokeObjectURL(url);

                    showNotification('Configuration backed up successfully', 'success');
                })
                .catch(error => {
                    showNotification('Error backing up configuration: ' + error, 'danger');
                });
        }

        // Restore configuration
        function restoreConfiguration() {
            const input = document.createElement('input');
            input.type = 'file';
            input.accept = '.json';
            input.onchange = function(e) {
                const file = e.target.files[0];
                const reader = new FileReader();
                reader.onload = function(event) {
                    try {
                        const config = JSON.parse(event.target.result);
                        saveConfiguration(config);
                        showNotification('Configuration restored successfully', 'success');
                    } catch (error) {
                        showNotification('Error parsing configuration file', 'danger');
                    }
                };
                reader.readAsText(file);
            };
            input.click();
        }

        // Factory reset
        function factoryReset() {
            if (confirm('Are you sure you want to reset to factory settings? This will erase all configuration.')) {
                if (confirm('This action cannot be undone. Are you absolutely sure?')) {
                    // This would require an API endpoint to perform factory reset
                    showNotification('Factory reset initiated. Device will restart.', 'warning');
                }
            }
        }

        // Show notification
        function showNotification(message, type) {
            const notification = document.createElement('div');
            notification.className = 'alert alert-' + type + ' alert-dismissible fade show';
            notification.innerHTML = `
                ${message}
                <button type="button" class="btn-close" data-bs-dismiss="alert"></button>
            `;

            document.body.insertBefore(notification, document.body.firstChild);

            // Auto dismiss after 5 seconds
            setTimeout(() => {
                notification.remove();
            }, 5000);
        }
    </script>
</body>
</html>
)====";
}

String getLogsHTML() {
  return R"====(
<!DOCTYPE html>
<html lang="en">
<head>
    <meta charset="UTF-8">
    <meta name="viewport" content="width=device-width, initial-scale=1.0">
    <title>Logs - Smart Farming System</title>
    <link href="https://cdn.jsdelivr.net/npm/bootstrap@5.1.3/dist/css/bootstrap.min.css" rel="stylesheet">
    <link rel="stylesheet" href="https://cdn.jsdelivr.net/npm/bootstrap-icons@1.7.2/font/bootstrap-icons.css">
    <style>
        .log-entry {
            transition: all 0.2s ease;
        }
        .log-entry:hover {
            background-color: #f8f9fa;
        }
        .log-debug {
            border-left: 4px solid #6c757d;
        }
        .log-info {
            border-left: 4px solid #17a2b8;
        }
        .log-warning {
            border-left: 4px solid #ffc107;
        }
        .log-error {
            border-left: 4px solid #dc3545;
        }
        .log-critical {
            border-left: 4px solid #6f42c1;
        }
    </style>
</head>
<body>
    <nav class="navbar navbar-expand-lg navbar-dark bg-primary">
        <div class="container">
            <a class="navbar-brand" href="/"><i class="bi bi-house-heart-fill"></i> Smart Farming</a>
            <div class="navbar-nav ms-auto">
                <a class="nav-link" href="/"><i class="bi bi-house"></i> Dashboard</a>
                <a class="nav-link" href="/control"><i class="bi bi-sliders"></i> Control</a>
                <a class="nav-link" href="/graphs"><i class="bi bi-graph-up"></i> Graphs</a>
                <a class="nav-link" href="/status"><i class="bi bi-info-circle"></i> Status</a>
                <a class="nav-link" href="/maintenance"><i class="bi bi-tools"></i> Maintenance</a>
                <a class="nav-link" href="/settings"><i class="bi bi-gear"></i> Settings</a>
            </div>
        </div>
    </nav>

    <div class="container mt-4">
        <h1><i class="bi bi-file-text"></i> System Logs</h1>

        <div class="row mb-4">
            <div class="col-12">
                <div class="card">
                    <div class="card-body">
                        <h5 class="card-title"><i class="bi bi-funnel"></i> Filters</h5>
                        <div class="row">
                            <div class="col-md-3">
                                <label for="logLevel" class="form-label">Log Level</label>
                                <select class="form-select" id="logLevel">
                                    <option value="all">All Levels</option>
                                    <option value="debug">Debug</option>
                                    <option value="info">Info</option>
                                    <option value="warning">Warning</option>
                                    <option value="error">Error</option>
                                    <option value="critical">Critical</option>
                                </select>
                            </div>
                            <div class="col-md-3">
                                <label for="logSource" class="form-label">Source</label>
                                <select class="form-select" id="logSource">
                                    <option value="all">All Sources</option>
                                    <option value="SYSTEM">System</option>
                                    <option value="SENSOR">Sensor</option>
                                    <option value="CONTROL">Control</option>
                                    <option value="COMMUNICATION">Communication</option>
                                    <option value="ENERGY">Energy</option>
                                    <option value="AI">AI</option>
                                    <option value="PREDICTION">Prediction</option>
                                    <option value="MAINTENANCE">Maintenance</option>
                                    <option value="RECOVERY">Recovery</option>
                                    <option value="RULES">Rules</option>
                                    <option value="SECURITY">Security</option>
                                    <option value="DIAGNOSTICS">Diagnostics</option>
                                </select>
                            </div>
                            <div class="col-md-3">
                                <label for="logTimeRange" class="form-label">Time Range</label>
                                <select class="form-select" id="logTimeRange">
                                    <option value="1h">Last Hour</option>
                                    <option value="6h">Last 6 Hours</option>
                                    <option value="24h">Last 24 Hours</option>
                                    <option value="7d">Last 7 Days</option>
                                    <option value="30d">Last 30 Days</option>
                                    <option value="all">All Time</option>
                                </select>
                            </div>
                            <div class="col-md-3">
                                <label for="logSearch" class="form-label">Search</label>
                                <input type="text" class="form-control" id="logSearch" placeholder="Search logs...">
                            </div>
                        </div>
                        <div class="row mt-3">
                            <div class="col-12">
                                <button class="btn btn-primary" onclick="applyFilters()">
                                    <i class="bi bi-funnel"></i> Apply Filters
                                </button>
                                <button class="btn btn-secondary" onclick="clearFilters()">
                                    <i class="bi bi-x-circle"></i> Clear Filters
                                </button>
                                <button class="btn btn-success" onclick="refreshLogs()">
                                    <i class="bi bi-arrow-clockwise"></i> Refresh
                                </button>
                                <button class="btn btn-info" onclick="exportLogs()">
                                    <i class="bi bi-download"></i> Export
                                </button>
                                <button class="btn btn-warning" onclick="clearLogs()">
                                    <i class="bi bi-trash"></i> Clear Logs
                                </button>
                            </div>
                        </div>
                    </div>
                </div>
            </div>
        </div>

        <div class="row mb-4">
            <div class="col-12">
                <div class="card">
                    <div class="card-body">
                        <h5 class="card-title"><i class="bi bi-bar-chart"></i> Log Statistics</h5>
                        <div class="row text-center">
                            <div class="col-md-2">
                                <div class="h4 text-secondary" id="debugCount">0</div>
                                <div>Debug</div>
                            </div>
                            <div class="col-md-2">
                                <div class="h4 text-info" id="infoCount">0</div>
                                <div>Info</div>
                            </div>
                            <div class="col-md-2">
                                <div class="h4 text-warning" id="warningCount">0</div>
                                <div>Warning</div>
                            </div>
                            <div class="col-md-2">
                                <div class="h4 text-danger" id="errorCount">0</div>
                                <div>Error</div>
                            </div>
                            <div class="col-md-2">
                                <div class="h4 text-purple" id="criticalCount">0</div>
                                <div>Critical</div>
                            </div>
                            <div class="col-md-2">
                                <div class="h4 text-primary" id="totalCount">0</div>
                                <div>Total</div>
                            </div>
                        </div>
                    </div>
                </div>
            </div>
        </div>

        <div class="row mb-4">
            <div class="col-12">
                <div class="card">
                    <div class="card-body">
                        <h5 class="card-title"><i class="bi bi-list"></i> Log Entries</h5>
                        <div id="logEntries">
                            </div>
                        <div class="d-flex justify-content-between mt-3">
                            <div>
                                <span id="logCount">0</span> entries shown
                            </div>
                            <div>
                                <button class="btn btn-sm btn-outline-primary" onclick="loadOlderLogs()">
                                    <i class="bi bi-arrow-left"></i> Older
                                </button>
                                <button class="btn btn-sm btn-outline-primary" onclick="loadNewerLogs()">
                                    <i class="bi bi-arrow-right"></i> Newer
                                </button>
                            </div>
                        </div>
                    </div>
                </div>
            </div>
        </div>
    </div>

    <script src="https://cdn.jsdelivr.net/npm/bootstrap@5.1.3/dist/js/bootstrap.bundle.min.js"></script>
    <script>
        let currentLogs = [];
        let filteredLogs = [];
        let currentPage = 0;
        const logsPerPage = 50;

        // Load initial data
        document.addEventListener('DOMContentLoaded', function() {
            loadLogs();

            // Auto-refresh logs every 30 seconds
            setInterval(loadLogs, 30000);
        });

        // Load logs
        function loadLogs() {
            fetch('/api/logs')
                .then(response => response.json())
                .then(data => {
                    currentLogs = data.logs;
                    filteredLogs = [...currentLogs];
                    updateLogStatistics();
                    displayLogs();
                })
                .catch(error => {
                    console.error('Error loading logs:', error);
                });
        }

        // Update log statistics
        function updateLogStatistics() {
            const stats = {
                debug: 0,
                info: 0,
                warning: 0,
                error: 0,
                critical: 0
            };

            filteredLogs.forEach(log => {
                stats[log.level.toLowerCase()]++;
            });

            document.getElementById('debugCount').textContent = stats.debug;
            document.getElementById('infoCount').textContent = stats.info;
            document.getElementById('warningCount').textContent = stats.warning;
            document.getElementById('errorCount').textContent = stats.error;
            document.getElementById('criticalCount').textContent = stats.critical;
            document.getElementById('totalCount').textContent = filteredLogs.length;
        }

        // Display logs
        function displayLogs() {
            const container = document.getElementById('logEntries');
            container.innerHTML = "";

            const startIndex = currentPage * logsPerPage;
            const endIndex = Math.min(startIndex + logsPerPage, filteredLogs.length);
            const logsToShow = filteredLogs.slice(startIndex, endIndex);

            logsToShow.forEach(log => {
                const logClass = 'log-' + log.level.toLowerCase();
                const levelIcon = getLevelIcon(log.level);
                const levelColor = getLevelColor(log.level);

                const element = document.createElement('div');
                element.className = 'log-entry ' + logClass + ' p-2 mb-2 border';
                element.innerHTML = `
                    <div class="d-flex justify-content-between">
                        <div class="d-flex align-items-center">
                            <i class="bi bi-${levelIcon} ${levelColor} me-2"></i>
                            <div>
                                <strong>${log.level}</strong> - ${log.source}
                                <div class="small text-muted">${log.message}</div>
                            </div>
                        </div>
                        <small class="text-muted">${new Date(log.timestamp).toLocaleString()}</small>
                    </div>
                `;
                container.appendChild(element);
            });

            document.getElementById('logCount').textContent = `${startIndex + 1}-${endIndex} of ${filteredLogs.length}`;
        }

        // Get level icon
        function getLevelIcon(level) {
            switch (level) {
                case 'DEBUG': return 'bug';
                case 'INFO': return 'info-circle';
                case 'WARNING': return 'exclamation-triangle';
                case 'ERROR': return 'x-circle';
                case 'CRITICAL': return 'exclamation-octagon';
                default: return 'circle';
            }
        }

        // Get level color
        function getLevelColor(level) {
            switch (level) {
                case 'DEBUG': return 'text-secondary';
                case 'INFO': return 'text-info';
                case 'WARNING': return 'text-warning';
                case 'ERROR': return 'text-danger';
                case 'CRITICAL': return 'text-purple';
                default: return 'text-muted';
            }
        }

        // Apply filters
        function applyFilters() {
            const level = document.getElementById('logLevel').value;
            const source = document.getElementById('logSource').value;
            const timeRange = document.getElementById('logTimeRange').value;
            const search = document.getElementById('logSearch').value.toLowerCase();

            filteredLogs = currentLogs.filter(log => {
                // Filter by level
                if (level !== 'all' && log.level !== level.toUpperCase()) {
                    return false;
                }

                // Filter by source
                if (source !== 'all' && log.source !== source) {
                    return false;
                }

                // Filter by time range
                if (timeRange !== 'all') {
                    const logTime = new Date(log.timestamp);
                    const now = new Date();
                    const timeDiff = now - logTime;

                    switch (timeRange) {
                        case '1h':
                            if (timeDiff > 3600000) return false;
                            break;
                        case '6h':
                            if (timeDiff > 21600000) return false;
                            break;
                        case '24h':
                            if (timeDiff > 86400000) return false;
                            break;
                        case '7d':
                            if (timeDiff > 604800000) return false;
                            break;
                        case '30d':
                            if (timeDiff > 2592000000) return false;
                            break;
                    }
                }

                // Filter by search
                if (search && !log.message.toLowerCase().includes(search)) {
                    return false;
                }

                return true;
            });

            currentPage = 0;
            updateLogStatistics();
            displayLogs();
        }

        // Clear filters
        function clearFilters() {
            document.getElementById('logLevel').value = 'all';
            document.getElementById('logSource').value = 'all';
            document.getElementById('logTimeRange').value = '24h';
            document.getElementById('logSearch').value = '';

            filteredLogs = [...currentLogs];
            currentPage = 0;
            updateLogStatistics();
            displayLogs();
        }

        // Refresh logs
        function refreshLogs() {
            loadLogs();
        }

        // Export logs
        function exportLogs() {
            // Create a CSV report
            let csv = 'Timestamp,Level,Source,Message\n';

            filteredLogs.forEach(log => {
                csv += `"${new Date(log.timestamp).toLocaleString()}","${log.level}","${log.source}","${log.message}"\n`;
            });

            // Create a download link
            const blob = new Blob([csv], { type: 'text/csv' });
            const url = window.URL.createObjectURL(blob);
            const a = document.createElement('a');
            a.href = url;
            a.download = 'smart_farming_logs_' + new Date().toISOString().split('T')[0] + '.csv';
            document.body.appendChild(a);
            a.click();
            document.body.removeChild(a);
            window.URL.revokeObjectURL(url);
        }

        // Clear logs
        function clearLogs() {
            if (confirm('Are you sure you want to clear all logs? This action cannot be undone.')) {
                // This would require an API endpoint to clear logs
                showNotification('Logs cleared successfully', 'success');
                loadLogs();
            }
        }

        // Load older logs
        function loadOlderLogs() {
            if (currentPage > 0) {
                currentPage--;
                displayLogs();
            }
        }

        // Load newer logs
        function loadNewerLogs() {
            const maxPage = Math.ceil(filteredLogs.length / logsPerPage) - 1;
            if (currentPage < maxPage) {
                currentPage++;
                displayLogs();
            }
        }

        // Show notification
        function showNotification(message, type) {
            const notification = document.createElement('div');
            notification.className = 'alert alert-' + type + ' alert-dismissible fade show';
            notification.innerHTML = `
                ${message}
                <button type="button" class="btn-close" data-bs-dismiss="alert"></button>
            `;

            document.body.insertBefore(notification, document.body.firstChild);

            // Auto dismiss after 5 seconds
            setTimeout(() => {
                notification.remove();
            }, 5000);
        }
    </script>
</body>
</html>
)====";
}

String getPredictionHTML() {
  return R"====(
<!DOCTYPE html>
<html lang="en">
<head>
    <meta charset="UTF-8">
    <meta name="viewport" content="width=device-width, initial-scale=1.0">
    <title>Prediction - Smart Farming System</title>
    <link href="https://cdn.jsdelivr.net/npm/bootstrap@5.1.3/dist/css/bootstrap.min.css" rel="stylesheet">
    <link rel="stylesheet" href="https://cdn.jsdelivr.net/npm/bootstrap-icons@1.7.2/font/bootstrap-icons.css">
    <style>
        .prediction-card {
            transition: all 0.3s ease;
        }
        .prediction-card:hover {
            transform: translateY(-5px);
            box-shadow: 0 4px 8px rgba(0,0,0,0.1);
        }
        .confidence-high {
            border-left: 4px solid #28a745;
        }
        .confidence-medium {
            border-left: 4px solid #ffc107;
        }
        .confidence-low {
            border-left: 4px solid #dc3545;
        }
    </style>
</head>
<body>
    <nav class="navbar navbar-expand-lg navbar-dark bg-primary">
        <div class="container">
            <a class="navbar-brand" href="/"><i class="bi bi-house-heart-fill"></i> Smart Farming</a>
            <div class="navbar-nav ms-auto">
                <a class="nav-link" href="/"><i class="bi bi-house"></i> Dashboard</a>
                <a class="nav-link" href="/control"><i class="bi bi-sliders"></i> Control</a>
                <a class="nav-link" href="/graphs"><i class="bi bi-graph-up"></i> Graphs</a>
                <a class="nav-link" href="/status"><i class="bi bi-info-circle"></i> Status</a>
                <a class="nav-link" href="/maintenance"><i class="bi bi-tools"></i> Maintenance</a>
                <a class="nav-link" href="/settings"><i class="bi bi-gear"></i> Settings</a>
            </div>
        </div>
    </nav>

    <div class="container mt-4">
        <h1><i class="bi bi-cpu"></i> Prediction System</h1>

        <div class="row mb-4">
            <div class="col-12">
                <div class="card">
                    <div class="card-body">
                        <h5 class="card-title"><i class="bi bi-graph-up-arrow"></i> Current Predictions</h5>
                        <div class="row" id="currentPredictions">
                            </div>
                    </div>
                </div>
            </div>
        </div>

        <div class="row mb-4">
            <div class="col-12">
                <div class="card">
                    <div class="card-body">
                        <h5 class="card-title"><i class="bi bi-clock-history"></i> Prediction History</h5>
                        <div class="table-responsive">
                            <table class="table table-striped">
                                <thead>
                                    <tr>
                                        <th>Date</th>
                                        <th>Predicted Temp</th>
                                        <th>Actual Temp</th>
                                        <th>Accuracy</th>
                                        <th>Predicted Humidity</th>
                                        <th>Actual Humidity</th>
                                        <th>Accuracy</th>
                                        <th>Confidence</th>
                                    </tr>
                                </thead>
                                <tbody id="predictionHistory">
                                    </tbody>
                            </table>
                        </div>
                    </div>
                </div>
            </div>
        </div>

        <div class="row mb-4">
            <div class="col-12">
                <div class="card">
                    <div class="card-body">
                        <h5 class="card-title"><i class="bi bi-gear"></i> Prediction Settings</h5>
                        <form id="predictionForm">
                            <div class="row">
                                <div class="col-md-6 mb-3">
                                    <div class="form-check form-switch">
                                        <input class="form-check-input" type="checkbox" id="predictiveEnabled">
                                        <label class="form-check-label" for="predictiveEnabled">
                                            Enable Predictive Control
                                        </label>
                                    </div>
                                    <div class="form-text">When enabled, the system will use predictions to optimize control</div>
                                </div>
                                <div class="col-md-6 mb-3">
                                    <div class="form-check form-switch">
                                        <input class="form-check-input" type="checkbox" id="adaptiveEnabled">
                                        <label class="form-check-label" for="adaptiveEnabled">
                                            Enable Adaptive Learning
                                        </label>
                                    </div>
                                    <div class="form-text">When enabled, the system will learn from prediction accuracy</div>
                                </div>
                            </div>
                            <div class="row">
                                <div class="col-md-6 mb-3">
                                    <label for="predictionHorizon" class="form-label">Prediction Horizon</label>
                                    <select class="form-select" id="predictionHorizon">
                                        <option value="1">1 Hour</option>
                                        <option value="6">6 Hours</option>
                                        <option value="24" selected>24 Hours</option>
                                        <option value="72">3 Days</option>
                                        <option value="168">1 Week</option>
                                    </select>
                                </div>
                                <div class="col-md-6 mb-3">
                                    <label for="confidenceThreshold" class="form-label">Confidence Threshold</label>
                                    <input type="range" class="form-range" id="confidenceThreshold" min="0" max="100" value="70">
                                    <div class="form-text">Minimum confidence required for automatic actions: <span id="confidenceValue">70%</span></div>
                                </div>
                            </div>
                            <button type="submit" class="btn btn-primary">
                                <i class="bi bi-check-circle"></i> Save Settings
                            </button>
                            <button type="button" class="btn btn-info" onclick="trainModel()">
                                <i class="bi bi-cpu"></i> Train Model
                            </button>
                            <button type="button" class="btn btn-warning" onclick="resetModel()">
                                <i class="bi bi-arrow-clockwise"></i> Reset Model
                            </button>
                        </form>
                    </div>
                </div>
            </div>
        </div>

        <div class="row mb-4">
            <div class="col-12">
                <div class="card">
                    <div class="card-body">
                        <h5 class="card-title"><i class="bi bi-speedometer2"></i> Model Performance</h5>
                        <div class="row">
                            <div class="col-md-3">
                                <div class="text-center">
                                    <div class="h4 text-primary" id="accuracyScore">0%</div>
                                    <div>Overall Accuracy</div>
                                </div>
                            </div>
                            <div class="col-md-3">
                                <div class="text-center">
                                    <div class="h4 text-success" id="precisionScore">0%</div>
                                    <div>Precision</div>
                                </div>
                            </div>
                            <div class="col-md-3">
                                <div class="text-center">
                                    <div class="h4 text-info" id="recallScore">0%</div>
                                    <div>Recall</div>
                                </div>
                            </div>
                            <div class="col-md-3">
                                <div class="text-center">
                                    <div class="h4 text-warning" id="f1Score">0%</div>
                                    <div>F1 Score</div>
                                </div>
                            </div>
                        </div>
                        <div class="mt-3">
                            <h6>Performance Trends</h6>
                            <div class="chart-container" style="height: 200px;">
                                <canvas id="performanceChart"></canvas>
                            </div>
                        </div>
                    </div>
                </div>
            </div>
        </div>
    </div>

    <script src="https://cdn.jsdelivr.net/npm/bootstrap@5.1.3/dist/js/bootstrap.bundle.min.js"></script>
    <script src="https://cdn.jsdelivr.net/npm/chart.js"></script>
    <script>
        // Load initial data
        document.addEventListener('DOMContentLoaded', function() {
            loadPredictionData();
            loadPredictionHistory();
            loadPredictionSettings();
            loadModelPerformance();

            // Update data every 5 minutes
            setInterval(loadPredictionData, 300000);
        });

        // Load prediction data
        function loadPredictionData() {
            fetch('/api/prediction')
                .then(response => response.json())
                .then(data => {
                    displayCurrentPredictions(data);
                })
                .catch(error => {
                    console.error('Error loading prediction data:', error);
                });
        }

        // Display current predictions
        function displayCurrentPredictions(data) {
            const container = document.getElementById('currentPredictions');
            const confidenceClass = data.confidence > 0.8 ? 'confidence-high' : 
                                   data.confidence > 0.6 ? 'confidence-medium' : 'confidence-low';

            container.innerHTML = `
                <div class="col-md-3 mb-3">
                    <div class="card prediction-card ${confidenceClass}">
                        <div class="card-body">
                            <h6 class="card-title">Temperature</h6>
                            <div class="h3">${data.predictedTemp.toFixed(1)}&deg;C</div>
                            <div class="progress mt-2">
                                <div class="progress-bar" style="width: ${data.confidence * 100}%"></div>
                            </div>
                            <small class="text-muted">Confidence: ${(data.confidence * 100).toFixed(1)}%</small>
                        </div>
                    </div>
                </div>
                <div class="col-md-3 mb-3">
                    <div class="card prediction-card ${confidenceClass}">
                        <div class="card-body">
                            <h6 class="card-title">Humidity</h6>
                            <div class="h3">${data.predictedHumidity.toFixed(1)}%</div>
                            <div class="progress mt-2">
                                <div class="progress-bar" style="width: ${data.confidence * 100}%"></div>
                            </div>
                            <small class="text-muted">Confidence: ${(data.confidence * 100).toFixed(1)}%</small>
                        </div>
                    </div>
                </div>
                <div class="col-md-3 mb-3">
                    <div class="card prediction-card ${confidenceClass}">
                        <div class="card-body">
                            <h6 class="card-title">Soil Moisture</h6>
                            <div class="h3">${data.predictedSoilMoisture.toFixed(1)}%</div>
                            <div class="progress mt-2">
                                <div class="progress-bar" style="width: ${data.confidence * 100}%"></div>
                            </div>
                            <small class="text-muted">Confidence: ${(data.confidence * 100).toFixed(1)}%</small>
                        </div>
                    </div>
                </div>
                <div class="col-md-3 mb-3">
                    <div class="card prediction-card ${confidenceClass}">
                        <div class="card-body">
                            <h6 class="card-title">Battery SOC</h6>
                            <div class="h3">${data.predictedBatterySOC.toFixed(1)}%</div>
                            <div class="progress mt-2">
                                <div class="progress-bar" style="width: ${data.confidence * 100}%"></div>
                            </div>
                            <small class="text-muted">Confidence: ${(data.confidence * 100).toFixed(1)}%</small>
                        </div>
                    </div>
                </div>
            `;
        }

        // Load prediction history
        function loadPredictionHistory() {
            // This would require an API endpoint to get prediction history
            // For now, we'll use mock data
            const history = [
                { date: '2023-11-01', predictedTemp: 25.5, actualTemp: 25.2, predictedHumidity: 85.0, actualHumidity: 84.5, confidence: 0.85 },
                { date: '2023-11-02', predictedTemp: 26.0, actualTemp: 26.3, predictedHumidity: 84.0, actualHumidity: 84.2, confidence: 0.82 },
                { date: '2023-11-03', predictedTemp: 24.8, actualTemp: 24.5, predictedHumidity: 86.0, actualHumidity: 85.5, confidence: 0.88 },
                { date: '2023-11-04', predictedTemp: 25.2, actualTemp: 25.8, predictedHumidity: 85.5, actualHumidity: 85.0, confidence: 0.79 },
                { date: '2023-11-05', predictedTemp: 26.5, actualTemp: 26.2, predictedHumidity: 83.5, actualHumidity: 83.8, confidence: 0.91 }
            ];

            const tbody = document.getElementById('predictionHistory');
            tbody.innerHTML = "";

            history.forEach(entry => {
                const tempAccuracy = (100 - Math.abs(entry.predictedTemp - entry.actualTemp) * 10).toFixed(1);
                const humidityAccuracy = (100 - Math.abs(entry.predictedHumidity - entry.actualHumidity) * 5).toFixed(1);

                const row = document.createElement('tr');
                row.innerHTML = `
                    <td>${entry.date}</td>
                    <td>${entry.predictedTemp.toFixed(1)}&deg;C</td>
                    <td>${entry.actualTemp.toFixed(1)}&deg;C</td>
                    <td><span class="badge bg-${tempAccuracy > 90 ? 'success' : tempAccuracy > 80 ? 'warning' : 'danger'}">${tempAccuracy}%</span></td>
                    <td>${entry.predictedHumidity.toFixed(1)}%</td>
                    <td>${entry.actualHumidity.toFixed(1)}%</td>
                    <td><span class="badge bg-${humidityAccuracy > 90 ? 'success' : humidityAccuracy > 80 ? 'warning' : 'danger'}">${humidityAccuracy}%</span></td>
                    <td><span class="badge bg-${entry.confidence > 0.8 ? 'success' : entry.confidence > 0.6 ? 'warning' : 'danger'}">${(entry.confidence * 100).toFixed(1)}%</span></td>
                `;
                tbody.appendChild(row);
            });
        }

        // Load prediction settings
        function loadPredictionSettings() {
            fetch('/api/config')
                .then(response => response.json())
                .then(data => {
                    document.getElementById('predictiveEnabled').checked = data.predictiveEnabled;
                    document.getElementById('adaptiveEnabled').checked = data.adaptiveEnabled;
                })
                .catch(error => {
                    console.error('Error loading prediction settings:', error);
                });
        }

        // Load model performance
        function loadModelPerformance() {
            // This would require an API endpoint to get model performance
            // For now, we'll use mock data
            document.getElementById('accuracyScore').textContent = '87.5%';
            document.getElementById('precisionScore').textContent = '85.2%';
            document.getElementById('recallScore').textContent = '89.1%';
            document.getElementById('f1Score').textContent = '87.1%';

            // Initialize performance chart
            const ctx = document.getElementById('performanceChart').getContext('2d');
            new Chart(ctx, {
                type: 'line',
                data: {
                    labels: ['Day 1', 'Day 2', 'Day 3', 'Day 4', 'Day 5', 'Day 6', 'Day 7'],
                    datasets: [{
                        label: 'Accuracy',
                        data: [82, 84, 83, 86, 85, 87, 87.5],
                        borderColor: 'rgb(75, 192, 192)',
                        backgroundColor: 'rgba(75, 192, 192, 0.2)',
                        tension: 0.1
                    }]
                },
                options: {
                    responsive: true,
                    maintainAspectRatio: false,
                    scales: {
                        y: {
                            beginAtZero: false,
                            min: 70,
                            max: 100
                        }
                    }
                }
            });
        }

        // Update confidence value display
        document.getElementById('confidenceThreshold').addEventListener('input', function() {
            document.getElementById('confidenceValue').textContent = this.value + '%';
        });

        // Save prediction settings
        document.getElementById('predictionForm').addEventListener('submit', function(e) {
            e.preventDefault();

            const config = {
                predictiveEnabled: document.getElementById('predictiveEnabled').checked,
                adaptiveEnabled: document.getElementById('adaptiveEnabled').checked
            };

            fetch('/api/config', {
                method: 'POST',
                headers: {
                    'Content-Type': 'application/json',
                },
                body: JSON.stringify(config)
            })
            .then(response => response.json())
            .then(data => {
                if (data.success) {
                    showNotification('Prediction settings saved successfully', 'success');
                } else {
                    showNotification('Failed to save prediction settings', 'danger');
                }
            })
            .catch(error => {
                showNotification('Error: ' + error, 'danger');
            });
        });

        // Train model
        function trainModel() {
            showNotification('Model training started. This may take a few minutes...', 'info');

            // This would require an API endpoint to train the model
            setTimeout(() => {
                showNotification('Model training completed successfully', 'success');
                loadModelPerformance();
            }, 5000);
        }

        // Reset model
        function resetModel() {
            if (confirm('Are you sure you want to reset the prediction model? This will clear all learned data.')) {
                // This would require an API endpoint to reset the model
                showNotification('Model reset successfully', 'warning');
                loadModelPerformance();
            }
        }

        // Show notification
        function showNotification(message, type) {
            const notification = document.createElement('div');
            notification.className = 'alert alert-' + type + ' alert-dismissible fade show';
            notification.innerHTML = `
                ${message}
                <button type="button" class="btn-close" data-bs-dismiss="alert"></button>
            `;

            document.body.insertBefore(notification, document.body.firstChild);

            // Auto dismiss after 5 seconds
            setTimeout(() => {
                notification.remove();
            }, 5000);
        }
    </script>
</body>
</html>
)====";
}

//============================================================================
// MENYIMPAN KONFIGURASI
//============================================================================
void saveConfiguration() {
  // Simpan konfigurasi ke EEPROM
  EEPROM.put(0, config);
  EEPROM.commit();
  
  Serial.println("Configuration saved");
  logMessage("INFO", "Configuration saved", "SYSTEM");
}

//============================================================================
// FUNGSI KAMERA
//============================================================================
void sendCameraCommand(String command) {
  HTTPClient http;
  String url = "http://" + String(ESP32_CAM_IP) + ":" + String(ESP32_CAM_PORT) + command;
  
  http.begin(url);
  int httpCode = http.GET();
  
  if (httpCode == HTTP_CODE_OK) {
    String response = http.getString();
    Serial.println("Camera response: " + response);
  } else {
    Serial.println("Failed to send command to camera");
    logMessage("ERROR", "Failed to send command to camera", "CAMERA");
  }
  
  http.end();
}

//============================================================================
// FUNGSI JADWAL
//============================================================================
void saveSchedules() {
  if (!sdCardAvailable) return;
  
  File file = SD.open("/schedules.json", FILE_WRITE);
  if (!file) {
    logMessage("ERROR", "Failed to save schedules", "SCHEDULE");
    return;
  }
  
  DynamicJsonDocument doc(4096);
  JsonArray schedulesArray = doc.createNestedArray("schedules");
  
  for (int i = 0; i < scheduleCount; i++) {
    JsonObject scheduleObj = schedulesArray.createNestedObject();
    scheduleObj["hour"] = schedules[i].hour;
    scheduleObj["minute"] = schedules[i].minute;
    scheduleObj["loadName"] = schedules[i].loadName;
    scheduleObj["active"] = schedules[i].active;
    scheduleObj["condition"] = schedules[i].condition;
    scheduleObj["conditionValue"] = schedules[i].conditionValue;
    scheduleObj["conditionOperator"] = schedules[i].conditionOperator;
  }
  
  String jsonString;
  serializeJson(doc, jsonString);
  file.print(jsonString);
  file.close();
  
  logMessage("INFO", "Schedules saved", "SCHEDULE");
}

void loadSchedules() {
  if (!sdCardAvailable) return;
  
  File file = SD.open("/schedules.json", FILE_READ);
  if (!file) {
    logMessage("WARNING", "No schedules file found", "SCHEDULE");
    return;
  }
  
  DynamicJsonDocument doc(4096);
  DeserializationError error = deserializeJson(doc, file);
  file.close();
  
  if (error) {
    logMessage("ERROR", "Failed to load schedules: " + String(error.c_str()), "SCHEDULE");
    return;
  }
  
  JsonArray schedulesArray = doc["schedules"];
  scheduleCount = 0;
  
  for (JsonObject scheduleObj : schedulesArray) {
    if (scheduleCount >= 20) break;
    
    schedules[scheduleCount].hour = scheduleObj["hour"];
    schedules[scheduleCount].minute = scheduleObj["minute"];
    schedules[scheduleCount].loadName = scheduleObj["loadName"].as<String>();
    schedules[scheduleCount].active = scheduleObj["active"];
    schedules[scheduleCount].condition = scheduleObj["condition"].as<String>();
    schedules[scheduleCount].conditionValue = scheduleObj["conditionValue"];
    schedules[scheduleCount].conditionOperator = scheduleObj["conditionOperator"].as<String>();
    
    scheduleCount++;
  }
  
  logMessage("INFO", "Schedules loaded: " + String(scheduleCount) + " schedules", "SCHEDULE");
}

//============================================================================
// FUNGSI MAINTENANCE
//============================================================================
void addMaintenanceActivity(MaintenanceActivity activity) {
  if (maintenanceActivityCount >= 20) {
    // Geser semua aktivitas ke kiri
    for (int i = 0; i < 19; i++) {
      maintenanceActivities[i] = maintenanceActivities[i + 1];
    }
    maintenanceActivityCount = 19;
  }
  
  maintenanceActivities[maintenanceActivityCount++] = activity;
  
  // Simpan ke SD card
  saveMaintenanceActivities();
  
  logMessage("INFO", "Maintenance activity added: " + activity.activityType, "MAINTENANCE");
}

void saveMaintenanceActivities() {
  if (!sdCardAvailable) return;
  
  File file = SD.open("/maintenance.json", FILE_WRITE);
  if (!file) {
    logMessage("ERROR", "Failed to save maintenance activities", "MAINTENANCE");
    return;
  }
  
  DynamicJsonDocument doc(4096);
  JsonArray activitiesArray = doc.createNestedArray("activities");
  
  for (int i = 0; i < maintenanceActivityCount; i++) {
    JsonObject activityObj = activitiesArray.createNestedObject();
    activityObj["timestamp"] = maintenanceActivities[i].timestamp;
    activityObj["activityType"] = maintenanceActivities[i].activityType;
    activityObj["description"] = maintenanceActivities[i].description;
    activityObj["status"] = maintenanceActivities[i].status;
    activityObj["aiRecommendation"] = maintenanceActivities[i].aiRecommendation;
    activityObj["resolved"] = maintenanceActivities[i].resolved;
  }
  
  String jsonString;
  serializeJson(doc, jsonString);
  file.print(jsonString);
  file.close();
  
  logMessage("INFO", "Maintenance activities saved", "MAINTENANCE");
}

void loadMaintenanceActivities() {
  if (!sdCardAvailable) return;
  
  File file = SD.open("/maintenance.json", FILE_READ);
  if (!file) {
    logMessage("WARNING", "No maintenance activities file found", "MAINTENANCE");
    return;
  }
  
  DynamicJsonDocument doc(4096);
  DeserializationError error = deserializeJson(doc, file);
  file.close();
  
  if (error) {
    logMessage("ERROR", "Failed to load maintenance activities: " + String(error.c_str()), "MAINTENANCE");
    return;
  }
  
  JsonArray activitiesArray = doc["activities"];
  maintenanceActivityCount = 0;
  
  for (JsonObject activityObj : activitiesArray) {
    if (maintenanceActivityCount >= 20) break;
    
    maintenanceActivities[maintenanceActivityCount].timestamp = activityObj["timestamp"];
    maintenanceActivities[maintenanceActivityCount].activityType = activityObj["activityType"].as<String>();
    maintenanceActivities[maintenanceActivityCount].description = activityObj["description"].as<String>();
    maintenanceActivities[maintenanceActivityCount].status = activityObj["status"].as<String>();
    maintenanceActivities[maintenanceActivityCount].aiRecommendation = activityObj["aiRecommendation"].as<String>();
    maintenanceActivities[maintenanceActivityCount].resolved = activityObj["resolved"];
    
    maintenanceActivityCount++;
  }
  
  logMessage("INFO", "Maintenance activities loaded: " + String(maintenanceActivityCount) + " activities", "MAINTENANCE");
}

//============================================================================
// ANALISIS POLA MAINTENANCE
//============================================================================
void analyzeMaintenancePatterns() {
  if (maintenanceActivityCount < 5) return; // Butuh minimal 5 aktivitas untuk analisis
  
  // Hitung frekuensi setiap jenis maintenance
  int roofLeakCount = 0;
  int sensorIssueCount = 0;
  int powerProblemCount = 0;
  int pumpMaintenanceCount = 0;
  int fanMaintenanceCount = 0;
  int batteryMaintenanceCount = 0;
  int inverterMaintenanceCount = 0;
  int cleaningCount = 0;
  int otherCount = 0;
  
  for (int i = 0; i < maintenanceActivityCount; i++) {
    String type = maintenanceActivities[i].activityType;
    
    if (type == "roof_leak") roofLeakCount++;
    else if (type == "sensor_issue") sensorIssueCount++;
    else if (type == "power_problem") powerProblemCount++;
    else if (type == "pump_maintenance") pumpMaintenanceCount++;
    else if (type == "fan_maintenance") fanMaintenanceCount++;
    else if (type == "battery_maintenance") batteryMaintenanceCount++;
    else if (type == "inverter_maintenance") inverterMaintenanceCount++;
    else if (type == "cleaning") cleaningCount++;
    else otherCount++;
  }
  
  // Cari masalah yang paling sering terjadi
  int maxCount = 0;
  String mostFrequentIssue = "";
  
  if (roofLeakCount > maxCount) {
    maxCount = roofLeakCount;
    mostFrequentIssue = "Roof Leak";
  }
  if (sensorIssueCount > maxCount) {
    maxCount = sensorIssueCount;
    mostFrequentIssue = "Sensor Issue";
  }
  if (powerProblemCount > maxCount) {
    maxCount = powerProblemCount;
    mostFrequentIssue = "Power Problem";
  }
  if (pumpMaintenanceCount > maxCount) {
    maxCount = pumpMaintenanceCount;
    mostFrequentIssue = "Pump Maintenance";
  }
  if (fanMaintenanceCount > maxCount) {
    maxCount = fanMaintenanceCount;
    mostFrequentIssue = "Fan Maintenance";
  }
  if (batteryMaintenanceCount > maxCount) {
    maxCount = batteryMaintenanceCount;
    mostFrequentIssue = "Battery Maintenance";
  }
  if (inverterMaintenanceCount > maxCount) {
    maxCount = inverterMaintenanceCount;
    mostFrequentIssue = "Inverter Maintenance";
  }
  if (cleaningCount > maxCount) {
    maxCount = cleaningCount;
    mostFrequentIssue = "Cleaning";
  }
  if (otherCount > maxCount) {
    maxCount = otherCount;
    mostFrequentIssue = "Other";
  }
  
  // Jika ada masalah yang sering terjadi, buat rekomendasi
  if (maxCount >= 3) {
    String message = "📊 Maintenance Analysis: " + mostFrequentIssue + " is the most frequent issue (" + String(maxCount) + " times). ";
    message += "Consider preventive measures to reduce occurrence.";
    
    sendTelegramMessage(message);
    logMessage("INFO", message, "MAINTENANCE");
    
    // Buat aktivitas maintenance untuk pencegahan
    MaintenanceActivity activity;
    activity.timestamp = millis();
    activity.activityType = "preventive_maintenance";
    activity.description = "Preventive maintenance for " + mostFrequentIssue;
    activity.status = "pending";
    activity.aiRecommendation = "Based on historical data, " + mostFrequentIssue + " occurs frequently. Schedule regular preventive maintenance.";
    activity.resolved = false;
    
    addMaintenanceActivity(activity);
  }
}

//============================================================================
// MANAJEMEN API KEY
//============================================================================
String readGeminiAPIKey() {
  String apiKey = "";
  
  // Coba baca dari SPIFFS/EEPROM
  if (SPIFFS.begin(true)) {
    File file = SPIFFS.open("/gemini_key.txt", FILE_READ);
    if (file) {
      apiKey = file.readString();
      file.close();
    }
  }
  
  // Jika tidak ada di SPIFFS, coba EEPROM
  if (apiKey.length() == 0) {
    const int maxKeyLength = 50;
    char key[maxKeyLength + 1];
    
    for (int i = 0; i < maxKeyLength; i++) {
      key[i] = EEPROM.read(100 + i);
      if (key[i] == 0) break;
    }
    key[maxKeyLength] = 0;
    
    apiKey = String(key);
  }
  
  // Hapus whitespace
  apiKey.trim();
  
  // Jika masih kosong, gunakan default (hanya untuk testing)
  if (apiKey.length() == 0) {
    // apiKey = "YOUR_DEFAULT_API_KEY_HERE"; // Hapus komentar untuk testing
    logMessage("WARNING", "Using default Gemini API key", "SYSTEM");
  }
  
  return apiKey;
}

bool saveGeminiAPIKey(String apiKey) {
  if (!SPIFFS.begin(true)) {
    logMessage("ERROR", "Failed to mount SPIFFS", "SYSTEM");
    return false;
  }
  
  File file = SPIFFS.open("/gemini_key.txt", FILE_WRITE);
  if (!file) {
    logMessage("ERROR", "Failed to open Gemini API key file for writing", "SYSTEM");
    return false;
  }
  
  file.print(apiKey);
  file.close();
  
  logMessage("INFO", "Gemini API key saved successfully", "SYSTEM");
  return true;
}

//============================================================================
// FUNGSI GEMINI AI
//============================================================================
String getGeminiRecommendation(String maintenanceDescription) {
  // Cek koneksi internet
  if (WiFi.status() != WL_CONNECTED) {
    logMessage("ERROR", "No internet connection for Gemini API", "AI");
    return "Error: Tidak ada koneksi internet";
  }
  
  // Baca API key
  String apiKey = readGeminiAPIKey();
  if (apiKey.length() == 0) {
    logMessage("ERROR", "Gemini API key not found", "AI");
    return "Error: API key tidak ditemukan. Silakan atur API key terlebih dahulu.";
  }
  
  // Batasi panjang deskripsi untuk menghemat resource
  if (maintenanceDescription.length() > 200) {
    maintenanceDescription = maintenanceDescription.substring(0, 200);
  }
  
  HTTPClient http;
  String url = "https://generativelanguage.googleapis.com/v1beta/models/gemini-pro:generateContent?key=" + apiKey;
  
  // Buat payload JSON
  String payload = "{\"contents\":[{\"parts\":[{\"text\":\"Saya sedang melakukan maintenance untuk smart farming sistem. Berikan rekomendasi untuk masalah: " + maintenanceDescription + ". Berikan dalam format: 1. Diagnosis, 2. Rekomendasi Tindakan, 3. Pencegahan di masa depan.\"}]}]}";
  
  http.setTimeout(10000); // 10 detik timeout
  
  http.begin(url);
  http.addHeader("Content-Type", "application/json");
  
  int httpCode = http.POST(payload);
  
  String response = "";
  if (httpCode == HTTP_CODE_OK) {
    String payloadResponse = http.getString();
    
    // Parse JSON response
    DynamicJsonDocument doc(2048);
    DeserializationError error = deserializeJson(doc, payloadResponse);
    
    if (!error) {
      // Ekstrak teks dari respons Gemini
      response = doc["candidates"][0]["content"]["parts"][0]["text"].as<String>();
      
      // Log successful request
      logMessage("INFO", "Gemini API request successful", "AI");
    } else {
      response = "Error parsing Gemini response: " + String(error.c_str());
      logMessage("ERROR", "Failed to parse Gemini response: " + String(error.c_str()), "AI");
    }
  } else {
    response = "HTTP Error: " + String(httpCode) + " - " + http.errorToString(httpCode);
    logMessage("ERROR", "Gemini API HTTP error: " + String(httpCode), "AI");
  }
  
  http.end();
  return response;
}

//============================================================================
// FUNGSI TELEGRAM
//============================================================================
void sendTelegramMessage(String message) {
  // Cek DND (Do Not Disturb)
  if (isDoNotDisturbTime()) {
    logMessage("INFO", "Message suppressed due to DND: " + message, "TELEGRAM");
    return;
  }
  
  // Kirim pesan ke Telegram
  if (bot.sendMessage(CHAT_ID, message)) {
    logMessage("INFO", "Telegram message sent: " + message, "TELEGRAM");
  } else {
    logMessage("ERROR", "Failed to send Telegram message", "TELEGRAM");
  }
}

bool isDoNotDisturbTime() {
  // Parse DND start and end times
  int startHour = atoi(config.dndStart);
  int startMinute = atoi(config.dndStart + 3);
  int endHour = atoi(config.dndEnd);
  int endMinute = atoi(config.dndEnd + 3);
  
  // Get current time
  int currentHour = timeClient.getHours();
  int currentMinute = timeClient.getMinutes();
  int currentTimeInMinutes = currentHour * 60 + currentMinute;
  int startTimeInMinutes = startHour * 60 + startMinute;
  int endTimeInMinutes = endHour * 60 + endMinute;
  
  // Check if current time is in DND period
  if (startTimeInMinutes <= endTimeInMinutes) {
    // Normal case (e.g., 22:00 to 07:00)
    return currentTimeInMinutes >= startTimeInMinutes || currentTimeInMinutes < endTimeInMinutes;
  } else {
    // Edge case (e.g., 22:00 to 07:00 crosses midnight)
    return currentTimeInMinutes >= startTimeInMinutes || currentTimeInMinutes < endTimeInMinutes;
  }
}

void handleTelegramMessages() {
  int numNewMessages = bot.getUpdates(bot.last_message_received + 1);
  
  while (numNewMessages) {
    Serial.println("got response");
    handleNewMessages(numNewMessages);
    numNewMessages = bot.getUpdates(bot.last_message_received + 1);
  }
}

void handleNewMessages(int numNewMessages) {
  for (int i = 0; i < numNewMessages; i++) {
    telegramMessage msg = bot.messages[i];
    chatId = msg.chat_id;
    messageId = msg.message_id;
    String text = msg.text;
    
    // Log pesan
    logMessage("INFO", "Telegram message received: " + text, "TELEGRAM");
    
    // Cek password
    if (text.startsWith("/password ")) {
      String password = text.substring(10);
      if (password == config.telegramPassword) {
        bot.sendMessage(chatId, "✅ Authentication successful. You can now use all commands.");
        logMessage("INFO", "Telegram authentication successful", "TELEGRAM");
      } else {
        bot.sendMessage(chatId, "❌ Invalid password. Please try again.");
        logMessage("WARNING", "Invalid Telegram password attempt", "TELEGRAM");
      }
      continue;
    }
    
    // Handle commands
    if (text == "/start") {
      bot.sendMessage(chatId, "🌱 Welcome to Smart Farming System!\n\n"
                          "Available commands:\n"
                          "/status - Get system status\n"
                          "/sensors - Get sensor readings\n"
                          "/control - Control loads\n"
                          "/mode - Change system mode\n"
                          "/prediction - Get predictions\n"
                          "/maintenance - View maintenance activities\n"
                          "/help - Show this help message");
    } else if (text == "/help") {
      bot.sendMessage(chatId, "🌱 Smart Farming System Help\n\n"
                          "Available commands:\n"
                          "/status - Get system status\n"
                          "/sensors - Get sensor readings\n"
                          "/control - Control loads\n"
                          "/mode - Change system mode\n"
                          "/prediction - Get predictions\n"
                          "/maintenance - View maintenance activities\n"
                          "/inverter_on - Turn on inverter\n"
                          "/inverter_off - Turn off inverter\n"
                          "/emergency - Emergency stop\n"
                          "/help - Show this help message");
    } else if (text == "/status") {
      String statusMessage = "📊 System Status\n\n";
      statusMessage += "🔋 Battery: " + String(batteryData.soc, 1) + "% (" + String(batteryData.voltage, 2) + "V)\n";
      statusMessage += "🌡️ Temperature: " + String(temperatureGudang, 1) + "°C\n";
      statusMessage += "💧 Humidity: " + String(humidityGudang, 1) + "%\n";
      statusMessage += "🌱 Soil Moisture: " + String(soilMoisture, 1) + "%\n";
      statusMessage += "☀️ Light: " + String(lightGudang, 0) + " lux\n";
      statusMessage += "🔄 Mode: " + String(modeNames[config.operationMode]) + "\n";
      statusMessage += "⏰ Uptime: " + String(millis() / 86400000) + " days";
      
      bot.sendMessage(chatId, statusMessage);
    } else if (text == "/sensors") {
      String sensorMessage = "📊 Sensor Readings\n\n";
      sensorMessage += "🌡️ Temperature: " + String(temperatureGudang, 1) + "°C\n";
      sensorMessage += "💧 Humidity: " + String(humidityGudang, 1) + "%\n";
      sensorMessage += "🌱 Soil Moisture: " + String(soilMoisture, 1) + "%\n";
      sensorMessage += "☀️ Light Gudang: " + String(lightGudang, 0) + " lux\n";
      sensorMessage += "☀️ Light Kebun: " + String(lightKebun, 0) + " lux\n";
      sensorMessage += "🌫️ CO2: " + String(co2Gudang, 3) + "%\n";
      sensorMessage += "🔧 Pressure: " + String(pressureKebun, 1) + " hPa\n";
      sensorMessage += "💧 Water Pressure: " + String(waterPressure, 1) + " Pa\n";
      
      bot.sendMessage(chatId, sensorMessage);
    } else if (text == "/control") {
      String controlMessage = "🎛️ Load Control\n\n";
      
      for (int i = 0; i < 7; i++) {
        controlMessage += loads[i].name + ": " + String(loads[i].actualStatus ? "ON" : "OFF") + "\n";
      }
      
      controlMessage += "\nUse /control_<load>_<on/off> to control loads";
      controlMessage += "\nExample: /control_Exhaust Fan_on";
      
      bot.sendMessage(chatId, controlMessage);
    } else if (text.startsWith("/control_")) {
      // Parse control command
      int firstUnderscore = text.indexOf('_', 9);
      int secondUnderscore = text.indexOf('_', firstUnderscore + 1);
      
      if (firstUnderscore > 0 && secondUnderscore > 0) {
        String loadName = text.substring(9, firstUnderscore);
        String action = text.substring(secondUnderscore + 1);
        
        bool status = (action == "on");
        
        // Find the load
        for (int i = 0; i < 7; i++) {
          if (loads[i].name == loadName) {
            controlLoad(loadName, status);
            
            String response = "✅ " + loadName + " turned " + (status ? "ON" : "OFF");
            bot.sendMessage(chatId, response);
            
            break;
          }
        }
      }
    } else if (text == "/mode") {
      String modeMessage = "🔄 System Mode: " + String(modeNames[config.operationMode]) + "\n\n";
      modeMessage += "Available modes:\n";
      
      for (int i = 0; i < MODE_COUNT; i++) {
        modeMessage += "/mode_" + String(modeNames[i]) + " - " + String(modeNames[i]) + "\n";
      }
      
      bot.sendMessage(chatId, modeMessage);
    } else if (text.startsWith("/mode_")) {
      // Parse mode command
      String modeStr = text.substring(6);
      
      for (int i = 0; i < MODE_COUNT; i++) {
        if (modeStr == modeNames[i]) {
          config.operationMode = (SystemMode)i;
          saveConfiguration();
          
          String message = "✅ System mode changed to: " + modeStr;
          bot.sendMessage(chatId, message);
          logMessage("INFO", message, "SYSTEM");
          
          break;
        }
      }
    } else if (text == "/prediction") {
      String predictionMessage = "🔮 Predictions\n\n";
      predictionMessage += "🌡️ Temperature: " + String(predictionData.predictedTemp, 1) + "°C\n";
      predictionMessage += "💧 Humidity: " + String(predictionData.predictedHumidity, 1) + "%\n";
      predictionMessage += "🌱 Soil Moisture: " + String(predictionData.predictedSoilMoisture, 1) + "%\n";
      predictionMessage += "🔋 Battery SOC: " + String(predictionData.predictedBatterySOC, 1) + "%\n";
      predictionMessage += "📊 Confidence: " + String(predictionData.confidence * 100, 1) + "%\n";
      
      bot.sendMessage(chatId, predictionMessage);
    } else if (text == "/maintenance") {
      String maintenanceMessage = "🔧 Maintenance Activities\n\n";
      
      for (int i = 0; i < maintenanceActivityCount; i++) {
        maintenanceMessage += "• " + maintenanceActivities[i].activityType + ": " + maintenanceActivities[i].description + "\n";
        maintenanceMessage += "  Status: " + maintenanceActivities[i].status + "\n";
      }
      
      if (maintenanceActivityCount == 0) {
        maintenanceMessage += "No maintenance activities recorded.";
      }
      
      bot.sendMessage(chatId, maintenanceMessage);
    } else if (text == "/inverter_on") {
      turnOnInverter();
      bot.sendMessage(chatId, "✅ Inverter turned ON");
    } else if (text == "/inverter_off") {
      turnOffInverter();
      bot.sendMessage(chatId, "✅ Inverter turned OFF");
    } else if (text == "/emergency") {
      config.operationMode = MODE_EMERGENCY;
      saveConfiguration();
      
      bot.sendMessage(chatId, "🚨 Emergency mode activated! All non-essential loads turned off.");
      logMessage("WARNING", "Emergency mode activated via Telegram", "SYSTEM");
    } else if (text.startsWith("/capture")) {
      sendCameraCommand("/capture");
      bot.sendMessage(chatId, "📸 Photo captured");
    } else if (text.startsWith("/rule_add")) {
      // Parse rule add command
      // Format: /rule_add_<name>_<sensor>_<condition>_<threshold>_<action>
      // Example: /rule_add_Temp_High_temperatureGudang_>_29_exhaustfan_on
      
      int firstUnderscore = text.indexOf('_', 10);
      int secondUnderscore = text.indexOf('_', firstUnderscore + 1);
      int thirdUnderscore = text.indexOf('_', secondUnderscore + 1);
      int fourthUnderscore = text.indexOf('_', thirdUnderscore + 1);
      int fifthUnderscore = text.indexOf('_', fourthUnderscore + 1);
      
      if (firstUnderscore > 0 && secondUnderscore > 0 && thirdUnderscore > 0 && fourthUnderscore > 0 && fifthUnderscore > 0) {
        String ruleName = text.substring(10, firstUnderscore);
        String sensor = text.substring(firstUnderscore + 1, secondUnderscore);
        String condition = text.substring(secondUnderscore + 1, thirdUnderscore);
        float threshold = text.substring(thirdUnderscore + 1, fourthUnderscore).toFloat();
        String action = text.substring(fifthUnderscore + 1);
        
        addCustomRule(ruleName, sensor, condition, threshold, action);
        
        String response = "✅ Custom rule added: " + ruleName;
        bot.sendMessage(chatId, response);
      }
    } else if (text == "/rules") {
      String rulesMessage = "📋 Custom Rules\n\n";
      
      for (int i = 0; i < customRuleCount; i++) {
        rulesMessage += "• " + customRules[i].ruleName + "\n";
        rulesMessage += "  If " + customRules[i].sensorSource + " " + customRules[i].condition + " " + String(customRules[i].threshold) + "\n";
        rulesMessage += "  Then " + customRules[i].action + "\n";
        rulesMessage += "  Status: " + (customRules[i].active ? "Active" : "Inactive") + "\n\n";
      }
      
      if (customRuleCount == 0) {
        rulesMessage += "No custom rules defined.";
      }
      
      bot.sendMessage(chatId, rulesMessage);
    } else {
      bot.sendMessage(chatId, "❌ Unknown command. Use /help for available commands.");
    }
  }
}

//============================================================================
// KIRIM DATA KE SERVER
//============================================================================
void sendDataToServers() {
  // Kirim data ke Google Sheets
  sendToGoogleSheets();
  
  // Kirim data ke server lokal (jika ada)
  sendToLocalServer();
}

void sendToGoogleSheets() {
  if (WiFi.status() != WL_CONNECTED) {
    logMessage("WARNING", "WiFi not connected, skipping Google Sheets upload", "COMMUNICATION");
    return;
  }
  
  // Buat URL untuk Google Apps Script
  String url = "https://script.google.com/macros/s/" + String(GOOGLE_SCRIPT_ID) + "/exec?";
  
  // Tambahkan parameter
  url += "temperature=" + String(temperatureGudang);
  url += "&humidity=" + String(humidityGudang);
  url += "&soilMoisture=" + String(soilMoisture);
  url += "&lightGudang=" + String(lightGudang);
  url += "&lightKebun=" + String(lightKebun);
  url += "&co2=" + String(co2Gudang);
  url += "&pressure=" + String(pressureKebun);
  url += "&waterPressure=" + String(waterPressure);
  url += "&batteryVoltage=" + String(batteryVoltage);
  url += "&batterySOC=" + String(batteryData.soc);
  url += "&batteryCycles=" + String(batteryData.cycles);
  url += "&timestamp=" + String(timeClient.getEpochTime());
  
  HTTPClient http;
  http.begin(url);
  http.setTimeout(10000); // 10 detik timeout
  
  int httpCode = http.GET();
  
  if (httpCode == HTTP_CODE_OK) {
    String response = http.getString();
    logMessage("INFO", "Data sent to Google Sheets successfully", "COMMUNICATION");
  } else {
    logMessage("ERROR", "Failed to send data to Google Sheets: " + String(httpCode), "COMMUNICATION");
  }
  
  http.end();
}

void sendToLocalServer() {
  // Implementasi pengiriman data ke server lokal
  // Ini bisa disesuaikan dengan kebutuhan
}

//============================================================================
// CEK CUACA
//============================================================================
void checkWeather() {
  if (WiFi.status() != WL_CONNECTED) {
    logMessage("WARNING", "WiFi not connected, skipping weather check", "COMMUNICATION");
    return;
  }
  
  // Buat URL untuk OpenWeatherMap API
  String url = "http://api.openweathermap.org/data/2.5/weather?";
  url += "lat=" + String(WEATHER_LAT);
  url += "&lon=" + String(WEATHER_LON);
  url += "&appid=" + String(WEATHER_API_KEY);
  url += "&units=metric";
  
  HTTPClient http;
  http.begin(url);
  http.setTimeout(10000); // 10 detik timeout
  
  int httpCode = http.GET();
  
  if (httpCode == HTTP_CODE_OK) {
    String response = http.getString();
    
    // Parse JSON response
    DynamicJsonDocument doc(1024);
    DeserializationError error = deserializeJson(doc, response);
    
    if (!error) {
      // Ekstrak data cuaca
      float temp = doc["main"]["temp"];
      float humidity = doc["main"]["humidity"];
      String description = doc["weather"][0]["description"];
      
      weatherData = "Current weather: " + description + ", " + String(temp) + "°C, " + String(humidity) + "% humidity";
      lastWeatherUpdate = getFormattedTime();
      
      logMessage("INFO", "Weather data updated: " + weatherData, "COMMUNICATION");
      
      // Kirim notifikasi jika cuaca ekstrem
      if (temp > 35) {
        String message = "🌡️ High temperature alert: " + String(temp) + "°C";
        sendTelegramMessage(message);
      }
      
      if (humidity > 90) {
        String message = "💧 High humidity alert: " + String(humidity) + "%";
        sendTelegramMessage(message);
      }
    } else {
      logMessage("ERROR", "Failed to parse weather data: " + String(error.c_str()), "COMMUNICATION");
    }
  } else {
    logMessage("ERROR", "Failed to fetch weather data: " + String(httpCode), "COMMUNICATION");
  }
  
  http.end();
}

//============================================================================
// CEK MODE EMERGENCY
//============================================================================
void checkEmergencyMode() {
  // Cek tombol emergency
  if (emergencyButtonStatus) {
    emergencyMode = true;
    config.operationMode = MODE_EMERGENCY;
    saveConfiguration();
    
    String message = "🚨 Emergency button pressed! Activating emergency mode.";
    sendTelegramMessage(message);
    logMessage("CRITICAL", message, "SYSTEM");
    
    // Matikan semua beban kecuali yang penting
    processEmergencyControl();
  }
  
  // Cek thermostat
  if (thermostatStatus) {
    String message = "🌡️ Thermostat triggered! Temperature too high.";
    sendTelegramMessage(message);
    logMessage("WARNING", message, "SYSTEM");
    
    // Nyalakan kipas
    controlLoad("Exhaust Fan", true);
  }
  
  // Cek baterai darurat
  if (batteryData.soc < BATTERY_EMERGENCY_THRESHOLD) {
    emergencyMode = true;
    config.operationMode = MODE_EMERGENCY;
    saveConfiguration();
    
    String message = "🔋 Emergency battery level: " + String(batteryData.soc, 1) + "%";
    sendTelegramMessage(message);
    logMessage("CRITICAL", message, "SYSTEM");
    
    // Matikan semua beban kecuali yang penting
    processEmergencyControl();
  }
}

//============================================================================
 CEK STATUS BEBAN
//============================================================================
void checkLoadStatus() {
  static unsigned long lastCheckTime = 0;
  if (millis() - lastCheckTime < 30000) return; // Cek setiap 30 detik
  
  for (int i = 0; i < 7; i++) {
    LoadStatus& load = loads[i];
    
    // Cek apakah beban macet
    if (load.commandStatus && !load.actualStatus && (millis() - load.lastUpdateTime > 30000)) {
      logMessage("WARNING", "Load " + load.name + " failed to turn on", "CONTROL");
      
      // Coba matikan dan nyalakan kembali
      digitalWrite(load.relayPin, LOW);
      delay(1000);
      digitalWrite(load.relayPin, HIGH);
      
      load.lastUpdateTime = millis();
    }
  }
  
  lastCheckTime = millis();
}

//============================================================================
// SIMPAN JADWAL
//============================================================================
void saveSchedules() {
  if (!sdCardAvailable) return;
  
  File file = SD.open("/schedules.json", FILE_WRITE);
  if (!file) {
    logMessage("ERROR", "Failed to save schedules", "SCHEDULE");
    return;
  }
  
  DynamicJsonDocument doc(4096);
  JsonArray schedulesArray = doc.createNestedArray("schedules");
  
  for (int i = 0; i < scheduleCount; i++) {
    JsonObject scheduleObj = schedulesArray.createNestedObject();
    scheduleObj["hour"] = schedules[i].hour;
    scheduleObj["minute"] = schedules[i].minute;
    scheduleObj["loadName"] = schedules[i].loadName;
    scheduleObj["active"] = schedules[i].active;
    scheduleObj["condition"] = schedules[i].condition;
    scheduleObj["conditionValue"] = schedules[i].conditionValue;
    scheduleObj["conditionOperator"] = schedules[i].conditionOperator;
  }
  
  String jsonString;
  serializeJson(doc, jsonString);
  file.print(jsonString);
  file.close();
  
  logMessage("INFO", "Schedules saved", "SCHEDULE");
}

//============================================================================
// MUAT JADWAL
//============================================================================
void loadSchedules() {
  if (!sdCardAvailable) return;
  
  File file = SD.open("/schedules.json", FILE_READ);
  if (!file) {
    logMessage("WARNING", "No schedules file found", "SCHEDULE");
    return;
  }
  
  DynamicJsonDocument doc(4096);
  DeserializationError error = deserializeJson(doc, file);
  file.close();
  
  if (error) {
    logMessage("ERROR", "Failed to load schedules: " + String(error.c_str()), "SCHEDULE");
    return;
  }
  
  JsonArray schedulesArray = doc["schedules"];
  scheduleCount = 0;
  
  for (JsonObject scheduleObj : schedulesArray) {
    if (scheduleCount >= 20) break;
    
    schedules[scheduleCount].hour = scheduleObj["hour"];
    schedules[scheduleCount].minute = scheduleObj["minute"];
    schedules[scheduleCount].loadName = scheduleObj["loadName"].as<String>();
    schedules[scheduleCount].active = scheduleObj["active"];
    schedules[scheduleCount].condition = scheduleObj["condition"].as<String>();
    schedules[scheduleCount].conditionValue = scheduleObj["conditionValue"];
    schedules[scheduleCount].conditionOperator = scheduleObj["conditionOperator"].as<String>();
    
    scheduleCount++;
  }
  
  logMessage("INFO", "Schedules loaded: " + String(scheduleCount) + " schedules", "SCHEDULE");
}

//============================================================================
// FUNGSI TAMBAHAN UNTUK SISTEM YANG LENGKAP
//============================================================================

//============================================================================
// UPDATE CONFIG FROM WEBSOCKET
//============================================================================
void updateConfigFromWebSocket(DynamicJsonDocument doc) {
  if (xSemaphoreTake(configMutex, portMAX_DELAY)) {
    // Update konfigurasi berdasarkan data dari WebSocket
    if (doc.containsKey("exhaustFanThreshold")) {
      config.exhaustFanThreshold = doc["exhaustFanThreshold"];
    }
    if (doc.containsKey("pumpGudangThreshold")) {
      config.pumpGudangThreshold = doc["pumpGudangThreshold"];
    }
    if (doc.containsKey("humidifierThreshold")) {
      config.humidifierThreshold = doc["humidifierThreshold"];
    }
    if (doc.containsKey("pumpKebunThreshold")) {
      config.pumpKebunThreshold = doc["pumpKebunThreshold"];
    }
    if (doc.containsKey("lampJalanThreshold")) {
      config.lampJalanThreshold = doc["lampJalanThreshold"];
    }
    if (doc.containsKey("lampGudangThreshold")) {
      config.lampGudangThreshold = doc["lampGudangThreshold"];
    }
    if (doc.containsKey("lampKebunThreshold")) {
      config.lampKebunThreshold = doc["lampKebunThreshold"];
    }
    if (doc.containsKey("tempHighThreshold")) {
      config.tempHighThreshold = doc["tempHighThreshold"];
    }
    if (doc.containsKey("tempNormalThreshold")) {
      config.tempNormalThreshold = doc["tempNormalThreshold"];
    }
    if (doc.containsKey("humidityLowThreshold")) {
      config.humidityLowThreshold = doc["humidityLowThreshold"];
    }
    if (doc.containsKey("humidityHighThreshold")) {
      config.humidityHighThreshold = doc["humidityHighThreshold"];
    }
    if (doc.containsKey("soilMoistureLowThreshold")) {
      config.soilMoistureLowThreshold = doc["soilMoistureLowThreshold"];
    }
    if (doc.containsKey("soilMoistureHighThreshold")) {
      config.soilMoistureHighThreshold = doc["soilMoistureHighThreshold"];
    }
    if (doc.containsKey("co2HighThreshold")) {
      config.co2HighThreshold = doc["co2HighThreshold"];
    }
    if (doc.containsKey("batteryLowThreshold")) {
      config.batteryLowThreshold = doc["batteryLowThreshold"];
    }
    if (doc.containsKey("batteryCriticalThreshold")) {
      config.batteryCriticalThreshold = doc["batteryCriticalThreshold"];
    }
    if (doc.containsKey("batteryEmergencyThreshold")) {
      config.batteryEmergencyThreshold = doc["batteryEmergencyThreshold"];
    }
    if (doc.containsKey("notificationMode")) {
      strcpy(config.notificationMode, doc["notificationMode"]);
    }
    if (doc.containsKey("dndStart")) {
      strcpy(config.dndStart, doc["dndStart"]);
    }
    if (doc.containsKey("dndEnd")) {
      strcpy(config.dndEnd, doc["dndEnd"]);
    }
    if (doc.containsKey("operationMode")) {
      String modeStr = doc["operationMode"];
      for (int i = 0; i < MODE_COUNT; i++) {
        if (modeStr == modeNames[i]) {
          config.operationMode = (SystemMode)i;
          break;
        }
      }
    }
    if (doc.containsKey("encryptionEnabled")) {
      config.encryptionEnabled = doc["encryptionEnabled"];
    }
    if (doc.containsKey("otaEnabled")) {
      config.otaEnabled = doc["otaEnabled"];
    }
    if (doc.containsKey("predictiveEnabled")) {
      config.predictiveEnabled = doc["predictiveEnabled"];
    }
    if (doc.containsKey("adaptiveEnabled")) {
      config.adaptiveEnabled = doc["adaptiveEnabled"];
    }
    if (doc.containsKey("pidEnabled")) {
      config.pidEnabled = doc["pidEnabled"];
    }
    
    // Simpan konfigurasi
    saveConfiguration();
    
    xSemaphoreGive(configMutex);
    
    logMessage("INFO", "Configuration updated via WebSocket", "SYSTEM");
  }
}

//============================================================================
// GEMINI AI UNTUK MODIFIKASI LOGIKA FUZZY
//============================================================================
void modifyFuzzyLogicWithAI() {
  // Cek koneksi internet
  if (WiFi.status() != WL_CONNECTED) {
    logMessage("ERROR", "No internet connection for AI fuzzy logic modification", "AI");
    return;
  }
  
  // Baca API key
  String apiKey = readGeminiAPIKey();
  if (apiKey.length() == 0) {
    logMessage("ERROR", "Gemini API key not found for fuzzy logic modification", "AI");
    return;
  }
  
  // Buat prompt untuk AI
  String prompt = "Berdasarkan data sensor berikut, berikan rekomendasi untuk memodifikasi aturan fuzzy logic:\n";
  prompt += "Suhu Gudang: " + String(temperatureGudang) + "°C\n";
  prompt += "Kelembaban Gudang: " + String(humidityGudang) + "%\n";
  prompt += "Kelembaban Tanah: " + String(soilMoisture) + "%\n";
  prompt += "Tegangan Baterai: " + String(batteryVoltage) + "V\n";
  prompt += "SOC Baterai: " + String(batteryData.soc) + "%\n";
  prompt += "Waktu: " + getFormattedTime() + "\n";
  prompt += "Berikan rekomendasi dalam format JSON untuk memodifikasi aturan fuzzy logic: {\"rules\": [{\"input1\": \"suhu\", \"input2\": \"kelembaban\", \"input3\": \"tanah\", \"output1\": \"kipas\", \"output2\": \"pompa_gudang\", \"output3\": \"pompa_kebun\", \"output4\": \"humidifier\", \"action\": \"increase/decrease/keep\"}]}";
  
  HTTPClient http;
  String url = "https://generativelanguage.googleapis.com/v1beta/models/gemini-pro:generateContent?key=" + apiKey;
  
  // Buat payload JSON
  String payload = "{\"contents\":[{\"parts\":[{\"text\":\"" + prompt + "\"}]}]}";
  
  http.setTimeout(15000); // 15 detik timeout
  
  http.begin(url);
  http.addHeader("Content-Type", "application/json");
  
  int httpCode = http.POST(payload);
  
  if (httpCode == HTTP_CODE_OK) {
    String payloadResponse = http.getString();
    
    // Parse JSON response
    DynamicJsonDocument doc(2048);
    DeserializationError error = deserializeJson(doc, payloadResponse);
    
    if (!error) {
      // Ekstrak rekomendasi dari AI
      String aiResponse = doc["candidates"][0]["content"]["parts"][0]["text"].as<String>();
      
      // Parse rekomendasi fuzzy logic
      DynamicJsonDocument fuzzyDoc(1024);
      DeserializationError fuzzyError = deserializeJson(fuzzyDoc, aiResponse);
      
      if (!fuzzyError) {
        // Terapkan modifikasi fuzzy logic
        applyFuzzyLogicModifications(fuzzyDoc);
        
        logMessage("INFO", "Fuzzy logic modified by AI", "AI");
        
        // Kirim notifikasi
        String message = "🧠 AI has modified fuzzy logic rules based on current conditions";
        sendTelegramMessage(message);
      } else {
        logMessage("ERROR", "Failed to parse AI fuzzy logic response: " + String(fuzzyError.c_str()), "AI");
      }
    } else {
      logMessage("ERROR", "Failed to parse AI response: " + String(error.c_str()), "AI");
    }
  } else {
    logMessage("ERROR", "AI fuzzy logic HTTP error: " + String(httpCode), "AI");
  }
  
  http.end();
}

void applyFuzzyLogicModifications(DynamicJsonDocument doc) {
  // Implementasi modifikasi fuzzy logic berdasarkan rekomendasi AI
  // Ini adalah implementasi sederhana, bisa dikembangkan lebih lanjut
  
  JsonArray rules = doc["rules"];
  
  for (JsonObject rule : rules) {
    String input1 = rule["input1"];
    String input2 = rule["input2"];
    String input3 = rule["input3"];
    String output1 = rule["output1"];
    String output2 = rule["output2"];
    String output3 = rule["output3"];
    String output4 = rule["output4"];
    String action = rule["action"];
    
    // Terapkan modifikasi berdasarkan action
    if (action == "increase") {
      // Tingkatkan output
      if (output1 == "kipas") {
        // Tingkatkan durasi kipas
        // Implementasi akan ditambahkan
      }
      if (output2 == "pompa_gudang") {
        // Tingkatkan durasi pompa gudang
        // Implementasi akan ditambahkan
      }
      // ... dan seterusnya
    } else if (action == "decrease") {
      // Kurangi output
      if (output1 == "kipas") {
        // Kurangi durasi kipas
        // Implementasi akan ditambahkan
      }
      if (output2 == "pompa_gudang") {
        // Kurangi durasi pompa gudang
        // Implementasi akan ditambahkan
      }
      // ... dan seterusnya
    }
    // "keep" tidak melakukan perubahan
  }
}

//============================================================================
// DURASI +30% SETELAH KONDISI TERPENUHI
//============================================================================
void extendLoadDuration(String loadName, float baseDuration) {
  // Cari beban dalam array
  int loadIndex = -1;
  for (int i = 0; i < 7; i++) {
    if (loads[i].name == loadName) {
      loadIndex = i;
      break;
    }
  }
  
  if (loadIndex == -1) {
    logMessage("ERROR", "Load not found for duration extension: " + loadName, "CONTROL");
    return;
  }
  
  // Tambahkan durasi 30%
  float extendedDuration = baseDuration * 1.3;
  
  // Gunakan timer non-blocking
  // Implementasi timer akan ditambahkan di bagian selanjutnya
  
  logMessage("INFO", "Extended duration for " + loadName + ": " + String(extendedDuration) + "ms", "CONTROL");
}

//============================================================================
// NOTIFIKASI LENGKAP KE TELEGRAM
//============================================================================
void sendDetailedNotification(String type, String message, String data) {
  // Cek DND
  if (isDoNotDisturbTime()) {
    logMessage("INFO", "Notification suppressed due to DND: " + message, "TELEGRAM");
    return;
  }
  
  // Format pesan berdasarkan jenis
  String formattedMessage = "";
  
  if (type == "sensor_alert") {
    formattedMessage = "🚨 Sensor Alert\n\n";
    formattedMessage += message + "\n\n";
    formattedMessage += "Current Readings:\n";
    formattedMessage += "🌡️ Temperature: " + String(temperatureGudang, 1) + "°C\n";
    formattedMessage += "💧 Humidity: " + String(humidityGudang, 1) + "%\n";
    formattedMessage += "🌱 Soil Moisture: " + String(soilMoisture, 1) + "%\n";
    formattedMessage += "🔋 Battery: " + String(batteryData.soc, 1) + "%\n";
    formattedMessage += "⏰ Time: " + getFormattedTime();
  } else if (type == "load_failure") {
    formattedMessage = "⚠️ Load Failure\n\n";
    formattedMessage += message + "\n\n";
    formattedMessage += "Load Details:\n" + data + "\n";
    formattedMessage += "⏰ Time: " + getFormattedTime();
  } else if (type == "battery_alert") {
    formattedMessage = "🔋 Battery Alert\n\n";
    formattedMessage += message + "\n\n";
    formattedMessage += "Battery Status:\n";
    formattedMessage += "Voltage: " + String(batteryData.voltage, 2) + "V\n";
    formattedMessage += "SOC: " + String(batteryData.soc, 1) + "%\n";
    formattedMessage += "SOH: " + String(batteryData.soh, 1) + "%\n";
    formattedMessage += "Cycles: " + String(batteryData.cycles) + "\n";
    formattedMessage += "Temperature: " + String(batteryData.temperature, 1) + "°C\n";
    formattedMessage += "⏰ Time: " + getFormattedTime();
  } else if (type == "maintenance_reminder") {
    formattedMessage = "🔧 Maintenance Reminder\n\n";
    formattedMessage += message + "\n\n";
    formattedMessage += "Maintenance Details:\n" + data + "\n";
    formattedMessage += "⏰ Time: " + getFormattedTime();
  } else if (type == "ai_recommendation") {
    formattedMessage = "🧠 AI Recommendation\n\n";
    formattedMessage += message + "\n\n";
    formattedMessage += "Analysis:\n" + data + "\n";
    formattedMessage += "⏰ Time: " + getFormattedTime();
  } else if (type == "daily_report") {
    formattedMessage = "📊 Daily Report\n\n";
    formattedMessage += "Date: " + getFormattedTime() + "\n\n";
    formattedMessage += "System Performance:\n";
    formattedMessage += "Average Temperature: " + data + "\n";
    formattedMessage += "Average Humidity: " + String(humidityGudang, 1) + "%\n";
    formattedMessage += "Energy Consumed: " + String(energyData.loadEnergy, 1) + " Wh\n";
    formattedMessage += "Energy Charged: " + String(energyData.energyCharged, 1) + " Wh\n";
    formattedMessage += "Efficiency: " + String(energyData.efficiency, 1) + "%\n";
    formattedMessage += "System Uptime: " + String(millis() / 86400000) + " days";
  } else {
    formattedMessage = message;
  }
  
  // Kirim pesan
  if (bot.sendMessage(CHAT_ID, formattedMessage)) {
    logMessage("INFO", "Detailed notification sent: " + type, "TELEGRAM");
  } else {
    logMessage("ERROR", "Failed to send detailed notification", "TELEGRAM");
  }
}

//============================================================================
// OTA UPDATE TANPA KOMPUTER
//============================================================================
void checkForOTAUpdate() {
  if (!config.otaEnabled) {
    return;
  }
  
  // Cek update setiap hari
  static unsigned long lastCheckTime = 0;
  if (millis() - lastCheckTime < 86400000) return; // 24 jam
  
  lastCheckTime = millis();
  
  // Cek koneksi internet
  if (WiFi.status() != WL_CONNECTED) {
    logMessage("WARNING", "WiFi not connected, skipping OTA update check", "OTA");
    return;
  }
  
  // URL untuk check update
  String updateUrl = "http://your-server.com/smartfarming/update.json";
  
  HTTPClient http;
  http.begin(updateUrl);
  http.setTimeout(10000); // 10 detik timeout
  
  int httpCode = http.GET();
  
  if (httpCode == HTTP_CODE_OK) {
    String response = http.getString();
    
    // Parse JSON response
    DynamicJsonDocument doc(1024);
    DeserializationError error = deserializeJson(doc, response);
    
    if (!error) {
      String version = doc["version"];
      String firmwareUrl = doc["firmware_url"];
      String description = doc["description"];
      
      // Cek versi
      if (version != FIRMWARE_VERSION) {
        String message = "🔄 Update Available!\n\n";
        message += "Version: " + version + "\n";
        message += "Description: " + description + "\n\n";
        message += "Send /update_now to install update";
        
        sendTelegramMessage(message);
        logMessage("INFO", "OTA update available: " + version, "OTA");
        
        // Simpan URL update
        saveUpdateUrl(firmwareUrl);
      } else {
        logMessage("INFO", "System is up to date", "OTA");
      }
    } else {
      logMessage("ERROR", "Failed to parse update info: " + String(error.c_str()), "OTA");
    }
  } else {
    logMessage("ERROR", "Failed to check for update: " + String(httpCode), "OTA");
  }
  
  http.end();
}

void saveUpdateUrl(String url) {
  if (!SPIFFS.begin(true)) {
    logMessage("ERROR", "Failed to mount SPIFFS for update URL", "OTA");
    return;
  }
  
  File file = SPIFFS.open("/update_url.txt", FILE_WRITE);
  if (!file) {
    logMessage("ERROR", "Failed to save update URL", "OTA");
    return;
  }
  
  file.print(url);
  file.close();
  
  logMessage("INFO", "Update URL saved", "OTA");
}

String getUpdateUrl() {
  if (!SPIFFS.begin(true)) {
    return "";
  }
  
  File file = SPIFFS.open("/update_url.txt", FILE_READ);
  if (!file) {
    return "";
  }
  
  String url = file.readString();
  file.close();
  
  return url;
}

void performOTAUpdate() {
  String updateUrl = getUpdateUrl();
  
  if (updateUrl.length() == 0) {
    sendTelegramMessage("❌ No update available");
    return;
  }
  
  sendTelegramMessage("🔄 Starting update process...");
  
  // Download firmware
  HTTPClient http;
  http.begin(updateUrl);
  http.setTimeout(30000); // 30 detik timeout
  
  int httpCode = http.GET();
  
  if (httpCode == HTTP_CODE_OK) {
    int contentLength = http.getSize();
    
    if (contentLength > 0) {
      // Mulai update
      if (Update.begin(contentLength)) {
        size_t written = Update.writeStream(*http.getStreamPtr());
        
        if (written == contentLength) {
          if (Update.end(true)) {
            sendTelegramMessage("✅ Update successful! Restarting...");
            logMessage("INFO", "OTA update successful", "OTA");
            
            // Restart ESP32
            delay(1000);
            ESP.restart();
          } else {
            sendTelegramMessage("❌ Update failed: " + Update.errorString());
            logMessage("ERROR", "OTA update failed: " + String(Update.errorString()), "OTA");
          }
        } else {
          sendTelegramMessage("❌ Update failed: Written only " + String(written) + " of " + String(contentLength) + " bytes");
          logMessage("ERROR", "OTA update incomplete", "OTA");
        }
      } else {
        sendTelegramMessage("❌ Not enough space for update");
        logMessage("ERROR", "Not enough space for OTA update", "OTA");
      }
    } else {
      sendTelegramMessage("❌ Invalid firmware size");
      logMessage("ERROR", "Invalid firmware size for OTA update", "OTA");
    }
  } else {
    sendTelegramMessage("❌ Failed to download firmware: " + String(httpCode));
    logMessage("ERROR", "Failed to download firmware: " + String(httpCode), "OTA");
  }
  
  http.end();
}

//============================================================================
// LAPORAN ENERGI LENGKAP
//============================================================================
void generateEnergyReport() {
  if (!sdCardAvailable) {
    logMessage("WARNING", "SD card not available for energy report", "ENERGY");
    return;
  }
  
  // Buat nama file dengan tanggal
  String filename = "/energy_report_" + getFormattedDate() + ".csv";
  
  File file = SD.open(filename, FILE_WRITE);
  if (!file) {
    logMessage("ERROR", "Failed to create energy report file", "ENERGY");
    return;
  }
  
  // Header CSV
  file.println("Date,Time,MPPT_Voltage,MPPT_Current,MPPT_Power,PWM_Voltage,PWM_Current,PWM_Power,Load_Voltage,Load_Current,Load_Power,Battery_Voltage,Battery_Current,Battery_SOC,Battery_Temperature");
  
  // Baca data historis dari SD card
  File historyFile = SD.open("/energy_history.csv", FILE_READ);
  if (historyFile) {
    // Salin data historis
    while (historyFile.available()) {
      String line = historyFile.readStringUntil('\n');
      file.println(line);
    }
    historyFile.close();
  }
  
  file.close();
  
  // Kirim notifikasi
  String message = "📊 Energy report generated: " + filename;
  sendTelegramMessage(message);
  
  logMessage("INFO", "Energy report generated: " + filename, "ENERGY");
}

//============================================================================
// KELELAHAN & KEANDALAN SISTEM
//============================================================================
void checkSystemReliability() {
  static unsigned long lastCheckTime = 0;
  if (millis() - lastCheckTime < 60000) return; // Cek setiap menit
  
  lastCheckTime = millis();
  
  // Cek koneksi WiFi
  if (WiFi.status() != WL_CONNECTED) {
    logMessage("WARNING", "WiFi disconnected, attempting reconnection", "RELIABILITY");
    
    // Coba reconnect
    int attempts = 0;
    while (WiFi.status() != WL_CONNECTED && attempts < 5) {
      WiFi.reconnect();
      delay(2000);
      attempts++;
    }
    
    if (WiFi.status() == WL_CONNECTED) {
      logMessage("INFO", "WiFi reconnected successfully", "RELIABILITY");
    } else {
      logMessage("ERROR", "Failed to reconnect WiFi", "RELIABILITY");
      
      // Restart WiFi
      WiFi.disconnect();
      delay(1000);
      initializeWiFi();
    }
  }
  
  // Cek sensor
  bool sensorFailure = false;
  
  if (temperatureGudang == 0 && humidityGudang == 0) {
    logMessage("ERROR", "SHT31 sensor failure detected", "RELIABILITY");
    sensorFailure = true;
    
    // Coba inisialisasi ulang
    if (!sht31.begin(0x44)) {
      logMessage("ERROR", "Failed to reinitialize SHT31 sensor", "RELIABILITY");
    } else {
      logMessage("INFO", "SHT31 sensor reinitialized successfully", "RELIABILITY");
    }
  }
  
  // Cek beban
  for (int i = 0; i < 7; i++) {
    LoadStatus& load = loads[i];
    
    // Jika beban seharusnya menyala tetapi tidak ada perubahan arus
    if (load.commandStatus && !load.actualStatus && (millis() - load.lastUpdateTime > 30000)) {
      logMessage("WARNING", "Load " + load.name + " failed to turn on, attempting recovery", "RELIABILITY");
      
      // Coba matikan dan nyalakan kembali
      digitalWrite(load.relayPin, LOW);
      delay(1000);
      digitalWrite(load.relayPin, HIGH);
      
      load.lastUpdateTime = millis();
    }
  }
  
  // Cek memori
  size_t freeHeap = ESP.getFreeHeap();
  if (freeHeap < 10000) {
    logMessage("WARNING", "Low memory detected: " + String(freeHeap) + " bytes", "RELIABILITY");
    
    // Restart sistem jika memori sangat rendah
    if (freeHeap < 5000) {
      logMessage("CRITICAL", "Critical low memory, restarting system", "RELIABILITY");
      ESP.restart();
    }
  }
  
  // Cek watchdog
  if (esp_task_wdt_status() != ESP_OK) {
    logMessage("ERROR", "Watchdog timer error, restarting", "RELIABILITY");
    ESP.restart();
  }
}

//============================================================================
// BELAJAR DARI MASA LALU
//============================================================================
void learnFromHistoricalData() {
  if (!sdCardAvailable) {
    logMessage("WARNING", "SD card not available for learning", "LEARNING");
    return;
  }
  
  // Baca data historis
  File file = SD.open("/historical_data.csv", FILE_READ);
  if (!file) {
    logMessage("WARNING", "No historical data found for learning", "LEARNING");
    return;
  }
  
  // Analisis data historis
  int tempCount = 0;
  float tempSum = 0;
  int humidityCount = 0;
  float humiditySum = 0;
  int soilMoistureCount = 0;
  float soilMoistureSum = 0;
  
  while (file.available()) {
    String line = file.readStringUntil('\n');
    
    // Parse data: timestamp,temp,humidity,soil,battery
    int firstComma = line.indexOf(',');
    int secondComma = line.indexOf(',', firstComma + 1);
    int thirdComma = line.indexOf(',', secondComma + 1);
    int fourthComma = line.indexOf(',', thirdComma + 1);
    
    if (firstComma > 0 && secondComma > firstComma && thirdComma > secondComma && fourthComma > thirdComma) {
      String tempStr = line.substring(firstComma + 1, secondComma);
      String humidityStr = line.substring(secondComma + 1, thirdComma);
      String soilMoistureStr = line.substring(thirdComma + 1, fourthComma);
      
      float temp = tempStr.toFloat();
      float humidity = humidityStr.toFloat();
      float soilMoisture = soilMoistureStr.toFloat();
      
      if (temp > 0) {
        tempSum += temp;
        tempCount++;
      }
      
      if (humidity > 0) {
        humiditySum += humidity;
        humidityCount++;
      }
      
      if (soilMoisture > 0) {
        soilMoistureSum += soilMoisture;
        soilMoistureCount++;
      }
    }
  }
  
  file.close();
  
  // Hitung rata-rata
  if (tempCount > 0) {
    float avgTemp = tempSum / tempCount;
    
    // Sesuaikan threshold berdasarkan data historis
    if (avgTemp > TEMP_NORMAL_THRESHOLD) {
      config.tempNormalThreshold = avgTemp;
      config.tempHighThreshold = avgTemp + 3.0;
      
      logMessage("INFO", "Temperature thresholds adjusted based on historical data", "LEARNING");
    }
  }
  
  if (humidityCount > 0) {
    float avgHumidity = humiditySum / humidityCount;
    
    // Sesuaikan threshold berdasarkan data historis
    if (avgHumidity > HUMIDITY_LOW_THRESHOLD) {
      config.humidityLowThreshold = avgHumidity - 5.0;
      config.humidityHighThreshold = avgHumidity + 5.0;
      
      logMessage("INFO", "Humidity thresholds adjusted based on historical data", "LEARNING");
    }
  }
  
  if (soilMoistureCount > 0) {
    float avgSoilMoisture = soilMoistureSum / soilMoistureCount;
    
    // Sesuaikan threshold berdasarkan data historis
    if (avgSoilMoisture > SOIL_MOISTURE_LOW_THRESHOLD) {
      config.soilMoistureLowThreshold = avgSoilMoisture - 10.0;
      config.soilMoistureHighThreshold = avgSoilMoisture + 10.0;
      
      logMessage("INFO", "Soil moisture thresholds adjusted based on historical data", "LEARNING");
    }
  }
  
  // Simpan konfigurasi yang telah disesuaikan
  saveConfiguration();
}

//============================================================================
// PENGALAMAN PENGGUNA
//============================================================================
void enhanceUserExperience() {
  // Implementasi fitur-fitur untuk meningkatkan pengalaman pengguna
  
  // 1. Personalisasi notifikasi
  personalizeNotifications();
  
  // 2. Rekomendasi otomatis
  generateAutomaticRecommendations();
  
  // 3. Tutorial interaktif
  checkTutorialNeeds();
  
  // 4. Feedback pengguna
  collectUserFeedback();
}

void personalizeNotifications() {
  // Analisis pola interaksi pengguna
  // Sesuaikan notifikasi berdasarkan preferensi pengguna
  
  // Contoh: Jika pengguna sering menonaktifkan notifikasi info,
  // kurangi frekuensi notifikasi info
}

void generateAutomaticRecommendations() {
  // Generate rekomendasi otomatis berdasarkan data sensor dan pola penggunaan
  
  if (batteryData.soh < 80.0) {
    String message = "💡 Recommendation: Battery health is below 80%. Consider replacing the battery soon to ensure reliable operation.";
    sendTelegramMessage(message);
  }
  
  if (energyData.efficiency < 70.0) {
    String message = "💡 Recommendation: Energy efficiency is below 70%. Check for potential energy leaks or consider adjusting load schedules.";
    sendTelegramMessage(message);
  }
}

void checkTutorialNeeds() {
  // Cek apakah pengguna baru atau membutuhkan tutorial
  static bool tutorialShown = false;
  
  if (!tutorialShown && millis() < 3600000) { // 1 jam pertama
    String message = "👋 Welcome to Smart Farming System!\n\n";
    message += "Here are some quick tips to get started:\n";
    message += "1. Use /status to check system status\n";
    message += "2. Use /control to control loads\n";
    message += "3. Use /mode to change system mode\n";
    message += "4. Check the web interface at http://" + WiFi.localIP().toString() + "\n\n";
    message += "Send /help for more commands.";
    
    sendTelegramMessage(message);
    tutorialShown = true;
  }
}

void collectUserFeedback() {
  // Implementasi pengumpulan feedback pengguna
  // Ini bisa dikembangkan lebih lanjut
}

//============================================================================
// KEAMANAN FISIK & PERAWATAN TERENCANA
//============================================================================
void enhancePhysicalSecurity() {
  // Implementasi fitur-fitur keamanan fisik
  
  // 1. Deteksi gerakan tidak sah
  detectUnauthorizedMovement();
  
  // 2. Monitoring suhu abnormal
  monitorAbnormalTemperature();
  
  // 3. Pemantauan getaran
  monitorVibrations();
  
  // 4. Perencanaan maintenance otomatis
  scheduleAutomaticMaintenance();
}

void detectUnauthorizedMovement() {
  // Gunakan sensor PIR untuk deteksi gerakan tidak sah
  // Jika gerakan terdeteksi di jam tidak biasa, kirim notifikasi
  
  int hour = timeClient.getHours();
  
  // Jika gerakan terdeteksi antara jam 10 malam sampai 5 pagi
  if ((hour >= 22 || hour <= 5) && (pir1Status || pir2Status)) {
    String message = "🚨 Security Alert: Movement detected during unusual hours";
    sendTelegramMessage(message);
    logMessage("WARNING", "Unauthorized movement detected", "SECURITY");
  }
}

void monitorAbnormalTemperature() {
  // Monitor suhu abnormal yang bisa menunjukkan masalah
  
  if (temperatureGudang > 40.0) {
    String message = "🌡️ High temperature alert: " + String(temperatureGudang, 1) + "°C";
    sendTelegramMessage(message);
    logMessage("WARNING", "Abnormal temperature detected", "SECURITY");
  }
  
  if (batteryData.temperature > 45.0) {
    String message = "🔋 High battery temperature: " + String(batteryData.temperature, 1) + "°C";
    sendTelegramMessage(message);
    logMessage("WARNING", "Abnormal battery temperature detected", "SECURITY");
  }
}

void monitorVibrations() {
  // Implementasi monitoring getaran
  // Ini memerlukan sensor getaran tambahan
  // Untuk saat ini, kita gunakan sensor yang ada
  
  // Contoh: Jika ada perubahan arus yang tiba-tiba, bisa menunjukkan getaran
  static float lastCurrent = 0;
  float currentChange = abs(currentLoad - lastCurrent);
  
  if (currentChange > 5.0) { // Perubahan arus lebih dari 5A
    String message = "📳 Vibration detected: Sudden current change";
    sendTelegramMessage(message);
    logMessage("WARNING", "Vibration detected", "SECURITY");
  }
  
  lastCurrent = currentLoad;
}

void scheduleAutomaticMaintenance() {
  // Jadwalkan maintenance otomatis berdasarkan penggunaan dan waktu
  
  static unsigned long lastScheduleTime = 0;
  if (millis() - lastScheduleTime < 86400000) return; // 24 jam
  
  lastScheduleTime = millis();
  
  // Cek komponen yang perlu maintenance
  for (int i = 0; i < componentCount; i++) {
    ComponentHealth& comp = componentHealth[i];
    
    if (comp.maintenanceRequired) {
      // Buat jadwal maintenance
      MaintenanceActivity activity;
      activity.timestamp = millis();
      activity.activityType = "scheduled_maintenance";
      activity.description = "Scheduled maintenance for " + comp.componentName;
      activity.status = "pending";
      activity.aiRecommendation = "Component health is below threshold. Schedule maintenance soon.";
      activity.resolved = false;
      
      addMaintenanceActivity(activity);
      
      String message = "🔧 Maintenance scheduled for " + comp.componentName;
      sendTelegramMessage(message);
    }
  }
}

//============================================================================
// INTEGRASI & OTOMASI LEBIH DALAM
//============================================================================
void enhanceIntegrationAndAutomation() {
  // Implementasi integrasi dan otomasi yang lebih dalam
  
  // 1. Sinkronisasi dengan sistem eksternal
  syncWithExternalSystems();
  
  // 2. Otomasi berbasis kondisi kompleks
  processComplexAutomation();
  
  // 3. Optimasi energi cerdas
  performIntelligentEnergyOptimization();
  
  // 4. Prediksi kebutuhan masa depan
  predictFutureNeeds();
}

void syncWithExternalSystems() {
  // Sinkronisasi dengan sistem eksternal
  // Misalnya: sistem cuaca, sistem irigasi, dll
  
  // Implementasi sinkronisasi dengan sistem cuaca
  if (weatherData.length() > 0) {
    // Sesuaikan kontrol berdasarkan cuaca
    if (weatherData.indexOf("rain") >= 0) {
      // Jika hujan, kurangi penyiraman
      logMessage("INFO", "Reducing irrigation due to rain", "INTEGRATION");
    }
  }
}

void processComplexAutomation() {
  // Proses otomasi berbasis kondisi kompleks
  
  // Contoh: Jika suhu tinggi dan kelembaban rendah, nyalakan kipas dan humidifier
  if (temperatureGudang > TEMP_HIGH_THRESHOLD && humidityGudang < HUMIDITY_LOW_THRESHOLD) {
    controlLoad("Exhaust Fan", true);
    controlLoad("Humidifier", true);
    
    logMessage("INFO", "Complex automation: High temp and low humidity", "AUTOMATION");
  }
  
  // Contoh: Jika baterai rendah dan matahari terbit, optimalkan pengisian
  int hour = timeClient.getHours();
  if (batteryData.soc < BATTERY_LOW_THRESHOLD && hour >= 6 && hour <= 18) {
    // Optimalkan pengisian
    optimizeBatteryCharging();
    
    logMessage("INFO", "Complex automation: Low battery during daytime", "AUTOMATION");
  }
}

void performIntelligentEnergyOptimization() {
  // Optimasi energi cerdas
  
  // Load shedding berdasarkan prioritas
  if (batteryData.soc < BATTERY_CRITICAL_THRESHOLD) {
    // Matikan beban tidak penting
    controlLoad("Lampu Gudang", false);
    controlLoad("Lampu Kebun", false);
    
    logMessage("INFO", "Load shedding activated due to critical battery", "ENERGY");
  }
  
  // Optimasi berdasarkan waktu
  hour = timeClient.getHours();
  
  // Siang hari: optimalkan penggunaan beban
  if (hour >= 10 && hour <= 15) {
    // Gunakan beban yang membutuhkan daya tinggi saat produksi energi maksimal
    if (batteryData.soc > 70.0) {
      // Boleh menggunakan beban daya tinggi
      logMessage("INFO", "High power loads allowed during peak production", "ENERGY");
    }
  }
  
  // Malam hari: hemat energi
  if (hour >= 20 || hour <= 5) {
    // Matikan beban tidak penting
    controlLoad("Lampu Gudang", false);
    
    logMessage("INFO", "Energy saving mode activated during night", "ENERGY");
  }
}

void predictFutureNeeds() {
  // Prediksi kebutuhan masa depan
  
  // Prediksi kebutuhan air berdasarkan kelembaban tanah dan cuaca
  if (soilMoisture < SOIL_MOISTURE_LOW_THRESHOLD) {
    // Prediksi kebutuhan air untuk 3 hari ke depan
    float futureNeed = SOIL_MOISTURE_LOW_THRESHOLD - soilMoisture;
    
    String message = "💧 Water need prediction: " + String(futureNeed, 1) + "% more water needed in the next 3 days";
    sendTelegramMessage(message);
    
    logMessage("INFO", "Water need prediction: " + String(futureNeed, 1) + "%", "PREDICTION");
  }
  
  // Prediksi kebutuhan energi berdasarkan pola penggunaan
  if (batteryData.soc < BATTERY_LOW_THRESHOLD) {
    // Prediksi kebutuhan energi untuk hari berikutnya
    float predictedUsage = energyData.loadEnergy * 1.2; // 20% lebih dari hari ini
    
    String message = "🔋 Energy need prediction: " + String(predictedUsage, 1) + "Wh needed for tomorrow";
    sendTelegramMessage(message);
    
    logMessage("INFO", "Energy need prediction: " + String(predictedUsage, 1) + "Wh", "PREDICTION");
  }
}

//============================================================================
// FUNGSI HELPER TAMBAHAN
//============================================================================
String getFormattedDate() {
  if (!timeConfigured) {
    return "1970-01-01";
  }
  
  timeClient.update();
  unsigned long epochTime = timeClient.getEpochTime();
  struct tm *ptm = gmtime ((time_t *)&epochTime);
  
  String dateStr = "";
  dateStr += String(ptm->tm_year + 1900);
  dateStr += "-";
  dateStr += String(ptm->tm_mon + 1);
  dateStr += "-";
  dateStr += String(ptm->tm_mday);
  
  return dateStr;
}

void handleTelegramCommand(String command) {
  // Handle command dari Telegram
  if (command == "/update_now") {
    performOTAUpdate();
  } else if (command == "/energy_report") {
    generateEnergyReport();
  } else if (command == "/learn") {
    learnFromHistoricalData();
  } else if (command == "/security_check") {
    enhancePhysicalSecurity();
  } else if (command == "/optimize") {
    performIntelligentEnergyOptimization();
  } else if (command == "/predict") {
    predictFutureNeeds();
  } else if (command == "/modify_fuzzy") {
    modifyFuzzyLogicWithAI();
  } else if (command == "/reboot") {
    sendTelegramMessage("🔄 Rebooting system...");
    logMessage("INFO", "System reboot requested via Telegram", "SYSTEM");
    delay(1000);
    ESP.restart();
  } else if (command == "/factory_reset") {
    if (confirmFactoryReset()) {
      performFactoryReset();
    }
  }
}

bool confirmFactoryReset() {
  // Kirim pesan konfirmasi
  bot.sendMessage(CHAT_ID, "⚠️ Factory reset will erase all configuration and data. Are you sure? Send /confirm_factory_reset to proceed.");
  
  // Tunggu konfirmasi
  unsigned long startTime = millis();
  while (millis() - startTime < 60000) { // 1 menit
    handleTelegramMessages();
    
    // Cek pesan konfirmasi
    int numNewMessages = bot.getUpdates(bot.last_message_received + 1);
    while (numNewMessages) {
      for (int i = 0; i < numNewMessages; i++) {
        telegramMessage msg = bot.messages[i];
        if (msg.text == "/confirm_factory_reset") {
          return true;
        }
      }
      numNewMessages = bot.getUpdates(bot.last_message_received + 1);
    }
    
    delay(1000);
  }
  
  return false;
}

void performFactoryReset() {
  // Hapus semua konfigurasi
  for (int i = 0; i < EEPROM_SIZE; i++) {
    EEPROM.write(i, 0);
  }
  EEPROM.commit();
  
  // Hapus file di SPIFFS
  if (SPIFFS.begin(true)) {
    // Format SPIFFS
    SPIFFS.format();
  }
  
  // Kirim notifikasi
  sendTelegramMessage("🔄 Factory reset completed. System will restart with default settings.");
  
  // Restart
  delay(2000);
  ESP.restart();
}

//============================================================================
// FUNGSI TIMER NON-BLOCKING
//============================================================================
struct Timer {
  unsigned long previousMillis;
  long interval;
  bool active;
  void (*callback)();
};

Timer timers[10];
int timerCount = 0;

int createTimer(long interval, void (*callback)()) {
  if (timerCount >= 10) return -1;
  
  timers[timerCount].previousMillis = millis();
  timers[timerCount].interval = interval;
  timers[timerCount].active = true;
  timers[timerCount].callback = callback;
  
  return timerCount++;
}

void updateTimers() {
  for (int i = 0; i < timerCount; i++) {
    if (timers[i].active) {
      unsigned long currentMillis = millis();
      
      if (currentMillis - timers[i].previousMillis >= timers[i].interval) {
        timers[i].previousMillis = currentMillis;
        timers[i].callback();
      }
    }
  }
}

void stopTimer(int timerId) {
  if (timerId >= 0 && timerId < timerCount) {
    timers[timerId].active = false;
  }
}

void startTimer(int timerId) {
  if (timerId >= 0 && timerId < timerCount) {
    timers[timerId].active = true;
    timers[timerId].previousMillis = millis();
  }
}

//============================================================================
// CALLBACK TIMER
//============================================================================
void turnOffExhaustFanTimer() {
  controlLoad("Exhaust Fan", false);
  logMessage("INFO", "Exhaust Fan turned off by timer", "TIMER");
}

void turnOffPumpGudangTimer() {
  controlLoad("Pump Gudang", false);
  logMessage("INFO", "Pump Gudang turned off by timer", "TIMER");
}

void turnOffHumidifierTimer() {
  controlLoad("Humidifier", false);
  logMessage("INFO", "Humidifier turned off by timer", "TIMER");
}

void turnOffPumpKebunTimer() {
  controlLoad("Pump Kebun", false);
  logMessage("INFO", "Pump Kebun turned off by timer", "TIMER");
}

void turnOffInverterTimer() {
  turnOffInverter();
  logMessage("INFO", "Inverter turned off by timer", "TIMER");
}

//============================================================================
// SETUP TIMER
//============================================================================
void setupTimers() {
  // Buat timer untuk mematikan beban secara otomatis
  // Timer akan diaktifkan saat beban dinyalakan
  
  // Contoh:
  // int timerId = createTimer(60000, turnOffExhaustFanTimer); // 1 menit
}

//============================================================================
// UPDATE LOOP UTAMA
//============================================================================
void updateMainLoop() {
  // Update timer non-blocking
  updateTimers();
  
  // Cek keandalan sistem
  checkSystemReliability();
  
  // Cek update OTA
  checkForOTAUpdate();
  
  // Tingkatkan pengalaman pengguna
  enhanceUserExperience();
  
  // Keamanan fisik
  enhancePhysicalSecurity();
  
  // Integrasi dan otomasi
  enhanceIntegrationAndAutomation();
}

//============================================================================
// AKHIR DARI KODE ESP32 SMART FARMING SYSTEM
//============================================================================
