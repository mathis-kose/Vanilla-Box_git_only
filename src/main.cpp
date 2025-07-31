#include <Arduino.h>
// Fehler-Flag für WiFiClient-Probleme
bool wifiClientError = false;
#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SH110X.h>
#include <BH1750.h>            // Licht
#include "ClosedCube_SHT31D.h" // Temperatur und Luftfeuchtigkeit
#include <PID_v1.h>
#include <TimeLib.h>  // Zeitfunktionen
#include <Timezone.h> // Zeitzonen und Sommerzeit

// WiFi
#include <WiFi.h> // für das Zeitmodul
#include <WiFiUdp.h>
#include <NTPClient.h>

// SD und SPI Bibliotheken hinzufügen
#include <SPI.h>
#include <SD.h>

// AHT20-Sensor
#include <Adafruit_AHTX0.h>

// CO2-Sensor
#include <SensirionI2CScd4x.h>

// Nach den bestehenden include-Direktiven
#include <map>
#include <string>

// *** Strukturdefinitionen HIER (EINMALIG) ***
// Struct für eine einzelne Messung mit Zeitstempel
struct TimestampedMeasurement {
    float value;
    unsigned long timestamp;
};

// Struktur für Mutex-Überwachung
struct MutexStats {
    unsigned long totalTimeHeld;    // Gesamtzeit, die der Mutex gehalten wurde (ms)
    unsigned long lastAcquireTime;  // Zeitpunkt der letzten Mutex-Übernahme
    bool isHeld;                    // Ob der Mutex aktuell gehalten wird
};

// Struktur für einen Sensorwert mit eigenem Mutex
struct SensorValueWithMutex {
    float value = 0.0f;
    SemaphoreHandle_t mutex;
    void init() { mutex = xSemaphoreCreateMutex(); }
};

// Neuer SensorManagerV2 mit feingranularen Mutexen
struct SensorManagerV2 {
    SensorValueWithMutex tempInside;
    SensorValueWithMutex humidityInside;
    SensorValueWithMutex tempOutside;
    SensorValueWithMutex humidityOutside;
    SensorValueWithMutex lux;
    SensorValueWithMutex co2;
    SensorValueWithMutex soilMoisture;
    // Initialisierung aller Mutexes
    void init() {
        tempInside.init();
        humidityInside.init();
        tempOutside.init();
        humidityOutside.init();
        lux.init();
        co2.init();
        soilMoisture.init();
    }
    // Getter für atomaren Zugriff
    float getValue(SensorValueWithMutex& s) {
        float v = 0.0f;
        if (xSemaphoreTake(s.mutex, pdMS_TO_TICKS(10)) == pdTRUE) {
            v = s.value;
            xSemaphoreGive(s.mutex);
        }
        return v;
    }
    // Setter für atomaren Zugriff
    void setValue(SensorValueWithMutex& s, float v) {
        if (xSemaphoreTake(s.mutex, pdMS_TO_TICKS(10)) == pdTRUE) {
            s.value = v;
            xSemaphoreGive(s.mutex);
        }
    }
};

// *** Globale Variablen und Definitionen ***

// Bildschirmdefinition
#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64
#define OLED_RESET -1
#define i2c_Address 0x3c

Adafruit_SH1106G display = Adafruit_SH1106G(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

// Sensor-Objekte
BH1750 lightMeter;        // Licht
ClosedCube_SHT31D sht3xd; // Temperatur und Luftfeuchtigkeit
Adafruit_AHTX0 aht;       // Außen Temperatur und Luftfeuchtigkeit
SensirionI2CScd4x scd4x;  // CO2-Sensor

// WiFi und Zeit
WiFiUDP udp;
NTPClient timeClient(udp, "pool.ntp.org", 0, 60000); // UTC

// Zeitzonen-Konfiguration für Berlin
static const TimeChangeRule CEST = {"CEST", Last, Sun, Mar, 2, 120}; // UTC+2
static const TimeChangeRule CET = {"CET ", Last, Sun, Oct, 3, 60};   // UTC+1
Timezone BerlinTime(CEST, CET);

// SD-Karte
const int sd_csPin = 5; // Chip Select Pin für das SD-Kartenmodul
const char *csvFileName = "/Valillia.csv";

// Mutexes
SemaphoreHandle_t i2c_mutex;
SemaphoreHandle_t sd_mutex;
SemaphoreHandle_t buffer_mutex; // Allgemeiner Buffer-Mutex (wird aktuell in updateBuffers verwendet)
SemaphoreHandle_t pid_mutex;
SemaphoreHandle_t oneMinBuffer_mutex;   // Für 1-Minuten-Ringspeicher
SemaphoreHandle_t longTermBuffer_mutex; // Für 24h-Ringspeicher


// ***  Neue Variablen für Button-Handling ***
#define BUTTON_PIN 0 // Boot-Button auf GPIO 0

volatile bool buttonPressed = false;
volatile unsigned long lastButtonPressTime = 0;
volatile unsigned long lastUserInteractionTime = 0;
volatile bool userIsInteracting = false;
const unsigned long userInteractionTimeout = 180000; // 3 Minuten in Millisekunden
const unsigned long debounceDelay = 200;             // Entprellzeit in Millisekunden

// Sensor-Daten (globale Variablen, auf die Tasks zugreifen)
float temperatureInside = 0.0f;
float humidityInside = 0.0f;
float temperatureOutside = 0.0f;
float humidityOutside = 0.0f;
float co2_ppm = 0.0f;
float lux = 0.0f;
float soilMoisturePercent = 0.0f;
uint16_t moistureValue = 0;

// Buffer Konstanten
#define SECONDS_PER_DAY 86400
#define MINUTES_PER_DAY 1440
#define SECONDS_PER_MINUTE 60

const int MAX_24H_POINTS = MINUTES_PER_DAY;     // 1440 Werte für 24 Stunden (1 Wert pro Minute)
const int MAX_1MIN_POINTS = SECONDS_PER_MINUTE; // 60 Werte für 1 Minute (1 Wert pro Sekunde)
const int MAX_DISPLAY_POINTS = 200;             // Maximal auf dem Display angezeigte Punkte

// 1-Minuten-Ringspeicher (Rohdaten für Aggregation)
TimestampedMeasurement temp1MinBuffer[MAX_1MIN_POINTS];
int temp1MinCount = 0;
unsigned long lastTemp1MinRead = 0;

TimestampedMeasurement hum1MinBuffer[MAX_1MIN_POINTS];
int hum1MinCount = 0;
unsigned long lastHum1MinRead = 0;

TimestampedMeasurement light1MinBuffer[MAX_1MIN_POINTS];
int light1MinCount = 0;
unsigned long lastLight1MinRead = 0;

TimestampedMeasurement soil1MinBuffer[MAX_1MIN_POINTS];
int soil1MinCount = 0;
unsigned long lastSoil1MinRead = 0;

TimestampedMeasurement co21MinBuffer[MAX_1MIN_POINTS];
int co21MinCount = 0;
unsigned long lastCO21MinRead = 0;

// 24-Stunden-Ringspeicher (aggregierte Daten für Plots und Langzeit-Logs)
// Auf realistischere Werte für den ESP32-Speicher reduzieren
const int MAX_TEMP_HUM_POINTS = 2880;   // Entspricht einem Wert pro 30 Sekunden für 24h
const int MAX_LIGHT_POINTS = 1440;      // Entspricht einem Wert pro Minute für 24h
const int MAX_CO2_POINTS = 720;         // Entspricht einem Wert pro 2 Minuten für 24h
const int MAX_SOIL_POINTS = 288;        // Entspricht einem Wert pro 5 Minuten für 24h

TimestampedMeasurement temperatureBuffer[MAX_24H_POINTS]; // War MAX_TEMP_HUM_POINTS, angepasst für Klarheit
int temperatureIndex = 0;
unsigned long lastTemperatureUpdate = 0;

TimestampedMeasurement humidityBuffer[MAX_24H_POINTS];  // War MAX_TEMP_HUM_POINTS
int humidityIndex = 0;
unsigned long lastHumidityUpdate = 0;

TimestampedMeasurement lightBuffer[MAX_24H_POINTS];     // War MAX_LIGHT_POINTS
int lightIndex = 0;
unsigned long lastLightUpdate = 0;

TimestampedMeasurement soilMoistureBuffer[MAX_24H_POINTS]; // War MAX_SOIL_POINTS
int soilMoistureIndex = 0;
unsigned long lastSoilMoistureUpdate = 0;

TimestampedMeasurement co2Buffer[MAX_24H_POINTS];       // War MAX_CO2_POINTS
int co2Index = 0;
unsigned long lastCO2Update = 0;


// Lichtsteuerung
const int LED_PWM = 16; // GPIO 16
double Setpoint, inputVal, outputVal;
double Kp = 0.0029, Ki = 0.009, Kd = 0;
PID light_pid(&inputVal, &outputVal, &Setpoint, Kp, Ki, Kd, DIRECT);

// PID-Parameter Grenzen
double Kp_min = 0.05, Kp_max = 0.2;
double Ki_min = 0.0001, Ki_max = 0.001;
double Kd_min = 0.0, Kd_max = 0.01;

// Lichtsteuerungsparameter
int sunriseHour = 8;
int sunriseMinute = 15;
int sunriseDurationMinutes = 60;
int sunsetDurationMinutes = 60;
int totalLightDurationMinutes = 14 * 60;
const int sunrisePhases = 60; // Beibehalten falls woanders verwendet, sonst ggf. redundant
const int sunsetPhases = 60;  // Beibehalten falls woanders verwendet, sonst ggf. redundant
int sunrisePhaseDuration = sunriseDurationMinutes / sunrisePhases; // Beibehalten
int sunsetPhaseDuration = sunsetDurationMinutes / sunsetPhases;   // Beibehalten

// Bodenfeuchtigkeit
const int moistureSensorPin = 32;
const int moistureMin = 1320;
const int moistureMax = 2700;

// Wasser
const int pumpPin = 17;
const int levelSensorPin = 4;
const int waterDetectionSensorPin = 13;
unsigned long wateringDuration = 1000 * 120; // 120 Sekunden
unsigned long lastWateringTime = 0;
unsigned long minIntervalBetweenWatering = 21600000; // 6 Stunden

// Bewässerungszeiten
const int watering1Hour = 8, watering1Minute = 0;
const int watering2Hour = 14, watering2Minute = 0;
const int watering3Hour = 20, watering3Minute = 0;
const int wateringStartHour = 8, wateringEndHour = 20;

// Bewässerung Tracking
bool watering1Done = false, watering2Done = false, watering3Done = false;
int lastWateringDay = -1;

// Bewässerungslogik Parameter
float moistureThreshold = 40.0f;
unsigned long lowMoistureDuration = 3600000; // 1 Stunde
unsigned long postWateringDelay = 7200000;   // 2 Stunden
bool isBelowThreshold = false;
unsigned long timeBelowThresholdStart = 0;

// PID-Regler für Bewässerung
double moistureSetpoint = 1700.0f;
double moistureInput;
double moistureOutput;
double Kp_moisture = 5000.0, Ki_moisture = 0.1, Kd_moisture = 0.0;
PID moisture_pid(&moistureInput, &moistureOutput, &moistureSetpoint, Kp_moisture, Ki_moisture, Kd_moisture, DIRECT);

// Temperatur und Luftfeuchtigkeit Sollwerte
float temperatureSetpointDay; // Werte werden nicht gesetzt, Logik fehlt?
float temperatureSetpointNight; // Werte werden nicht gesetzt, Logik fehlt?
float humiditySetpoint = 75.0f;

// WiFi: Mehrere SSIDs und Passwörter
struct WiFiCredential {
    const char* ssid;
    const char* password;
};

WiFiCredential wifiCredentials[] = {
    {"Finger Weg !!! 2.4GHz", "besterskinever"},
    {"FRITZ!Box 7530 BB", "28584884196075694927"}
    // {"SSID2", "PASS2"},
    // {"SSID3", "PASS3"}
};
const int wifiCredentialCount = sizeof(wifiCredentials) / sizeof(wifiCredentials[0]);

// Weitere globale Variablen
const int totalPages = 19; // 5 neue Seiten für 1-Minuten-Graphen
int currentPage = 0;
unsigned long startTime;
const unsigned long delayBeforeLogging = 10000; // 10 Sekunden

// Zeitintervalle und Zeitstempel
const long pageInterval = 3000;
const long displayRefreshInterval = 300;
const long logInterval = 60000;
unsigned long lastPageChange = 0;
unsigned long lastDisplayRefresh = 0;
unsigned long lastLogTime = 0;

// Bitmaps
const uint8_t scharf_s_bitmap[] PROGMEM = {
    0b00000000, 0b00111100, 0b01100010, 0b00110000,
    0b00001100, 0b01100100, 0b00111100, 0b00000000};

const uint8_t plant_and_drop_32x32_bitmap[] PROGMEM = {
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x80, 0x00, 0x00, 0x00, 0x80, 0x00, 
    0x00, 0x03, 0xe0, 0x00, 0x00, 0x05, 0x90, 0x00, 0x00, 0x04, 0x90, 0x00, 0x00, 0x04, 0x90, 0x00, 
    0x00, 0x18, 0x88, 0x00, 0x00, 0x18, 0x8c, 0x00, 0x00, 0x18, 0x8c, 0x00, 0x07, 0xf8, 0x8f, 0xe0, 
    0x07, 0xf8, 0x8f, 0xf0, 0x18, 0x07, 0xf0, 0x08, 0x3a, 0x86, 0x30, 0xfc, 0x3f, 0x84, 0x11, 0xfe, 
    0x18, 0x78, 0x8e, 0x08, 0x08, 0x39, 0xee, 0x08, 0x07, 0x1b, 0xec, 0x70, 0x00, 0xf8, 0x8f, 0x80, 
    0x00, 0xf8, 0x8f, 0x80, 0x00, 0x84, 0x11, 0x80, 0x03, 0x1b, 0xec, 0x40, 0x03, 0x13, 0xec, 0x40, 
    0x04, 0x63, 0x62, 0x30, 0x04, 0x83, 0x61, 0xb0, 0x06, 0x82, 0x61, 0xb0, 0x07, 0x04, 0x10, 0x70, 
    0x07, 0xf8, 0x0f, 0xf0, 0x07, 0xf0, 0x0f, 0xf0, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00
};

// Median Filter für CO2
const int MEDIAN_WINDOW = 3;
uint16_t co2Values[MEDIAN_WINDOW] = {0};
int co2ValueIndex = 0;
uint16_t co2_raw = 0;

// Lichtsteuerung
float maxLightIntensity = 2000.0f;
float lastEstimatedTimeToSetpoint = 0.0f; // Wird aktuell nicht verwendet?

// Maps zur Verfolgung der Mutex-Nutzung (EINMALIGE DEKLARATION HIER)
std::map<String, MutexStats> i2cMutexStats;
std::map<String, MutexStats> sdMutexStats;
std::map<String, MutexStats> pidMutexStats;
std::map<String, MutexStats> oneMinBufferMutexStats; // War schon oben, hier konsolidiert
std::map<String, MutexStats> longTermBufferMutexStats; // War schon oben, hier konsolidiert
// std::map<String, MutexStats> bufferMutexStats; // `buffer_mutex` wird verwendet, aber die Stats-Map dafür fehlt, wenn sie separat sein soll. Aktuell wird `oneMinBufferMutexStats` in `updateBuffers` verwendet. Das kann verwirrend sein.

// Mutex Statistik Ausgabe
unsigned long lastStatsOutputTime = 0;
const unsigned long STATS_OUTPUT_INTERVAL = 5000; // 5 Sekunden


// Struktur für zentrale Sensordatenverwaltung
struct SensorDataManager {
    // Flags für Datenverfügbarkeit
    volatile bool temperatureInsideUpdated = false;
    volatile bool humidityInsideUpdated = false;
    volatile bool temperatureOutsideUpdated = false;
    volatile bool humidityOutsideUpdated = false;
    volatile bool luxUpdated = false;
    volatile bool co2Updated = false;
    volatile bool soilMoistureUpdated = false;
    
    // Zeitstempel der letzten Aktualisierungen
    volatile unsigned long temperatureInsideLastUpdate = 0;
    volatile unsigned long humidityInsideLastUpdate = 0;
    volatile unsigned long temperatureOutsideLastUpdate = 0;
    volatile unsigned long humidityOutsideLastUpdate = 0;
    volatile unsigned long luxLastUpdate = 0;
    volatile unsigned long co2LastUpdate = 0;
    volatile unsigned long soilMoistureLastUpdate = 0;
    
    // Update-Intervalle in Millisekunden
    const unsigned long TEMP_HUM_INSIDE_INTERVAL = 1000;
    const unsigned long TEMP_HUM_OUTSIDE_INTERVAL = 1000;
    const unsigned long LUX_INTERVAL = 200;
    const unsigned long CO2_INTERVAL = 1000;
    const unsigned long SOIL_MOISTURE_INTERVAL = 5000;
    
    // Timeout-Zähler für problematische Sensoren
    volatile int sht31dErrorCount = 0;
    volatile unsigned long sht31dErrorTime = 0;
    const int MAX_SENSOR_ERRORS = 5;
    const unsigned long ERROR_COOLDOWN = 60000;
    
    SemaphoreHandle_t dataMutex; // Mutex für den Zugriff auf die SensorDataManager-internen Daten (temperatureInside, humidityInside etc.)
    
    void init() {
        dataMutex = xSemaphoreCreateMutex();
        if (dataMutex == NULL) {
            Serial.println("Fehler beim Erstellen des Sensordaten-Mutex");
        } else {
            Serial.println("Sensordaten-Mutex erfolgreich erstellt");
        }
    }
    
    void updateTemperatureInside(float value) {
        if (xSemaphoreTake(dataMutex, pdMS_TO_TICKS(100)) == pdTRUE) {
            temperatureInside = value; // Direkter Zugriff auf globale Variable
            temperatureInsideUpdated = true;
            temperatureInsideLastUpdate = millis();
            xSemaphoreGive(dataMutex);
        }
    }
    
    void updateHumidityInside(float value) {
        if (xSemaphoreTake(dataMutex, pdMS_TO_TICKS(100)) == pdTRUE) {
            humidityInside = value; // Direkter Zugriff auf globale Variable
            humidityInsideUpdated = true;
            humidityInsideLastUpdate = millis();
            xSemaphoreGive(dataMutex);
        }
    }
    
    void updateTemperatureOutside(float value) {
        if (xSemaphoreTake(dataMutex, pdMS_TO_TICKS(100)) == pdTRUE) {
            temperatureOutside = value; // Direkter Zugriff
            temperatureOutsideUpdated = true;
            temperatureOutsideLastUpdate = millis();
            xSemaphoreGive(dataMutex);
        }
    }
    
    void updateHumidityOutside(float value) {
        if (xSemaphoreTake(dataMutex, pdMS_TO_TICKS(100)) == pdTRUE) {
            humidityOutside = value; // Direkter Zugriff
            humidityOutsideUpdated = true;
            humidityOutsideLastUpdate = millis();
            xSemaphoreGive(dataMutex);
        }
    }
    
    void updateLux(float value) {
        if (xSemaphoreTake(dataMutex, pdMS_TO_TICKS(100)) == pdTRUE) {
            lux = value; // Direkter Zugriff
            luxUpdated = true;
            luxLastUpdate = millis();
            xSemaphoreGive(dataMutex);
        }
    }
    
    void updateCO2(float value) {
        if (xSemaphoreTake(dataMutex, pdMS_TO_TICKS(100)) == pdTRUE) {
            co2_ppm = value; // Direkter Zugriff
            co2Updated = true;
            co2LastUpdate = millis();
            xSemaphoreGive(dataMutex);
        }
    }
    
    void updateSoilMoisture(float percent, uint16_t rawValue) {
        if (xSemaphoreTake(dataMutex, pdMS_TO_TICKS(100)) == pdTRUE) {
            soilMoisturePercent = percent; // Direkter Zugriff
            moistureValue = rawValue;    // Direkter Zugriff
            soilMoistureUpdated = true;
            soilMoistureLastUpdate = millis();
            xSemaphoreGive(dataMutex);
        }
    }
    
    bool isUpdateDue(unsigned long lastUpdate, unsigned long interval) {
        return (millis() - lastUpdate) >= interval;
    }

    volatile bool useFallbackForIndoorSensors = false;
    const float INDOOR_OUTDOOR_OFFSET_TEMP = -3.0f;
    const float INDOOR_OUTDOOR_OFFSET_HUM = 10.0f;
};

SensorDataManager sensorManager; // Instanz
SensorManagerV2 sensorManagerV2;
// Ultraschall-Luftbefeuchter
const int humidifierPin = 25;
float humidityHysteresis = 3.0f;
bool humidifierState = false;

// *** Funktionsprototypen ***
// (Viele sind schon oben als globale Variablen oder Teil von Structs)
// String formatAgeString(unsigned long age_ms); // Ist schon oben als globale Funktion
void printBufferStats();
void SensorTask(void *pvParameters);
void DisplayTask(void *pvParameters);
void Licht_Regeln(void *pvParameters);
void Temp_Regeln(void *parameter);
void Humidity_Regeln(void *parameter);
void Logging_Task(void *pvParameters);
void initializeSD();
void writeCSV(String data);
void connectToWiFi();
time_t getNtpTime();
// void updateSetpoints(); // Existiert, aber leer
void WateringTask(void *pvParameters);
bool waterReservoirIsFull();
void startWatering();
String twoDigits(int num);
void displayPage();
void plotBufferData(TimestampedMeasurement buffer[], int totalWriteCount, int maxDisplayPointsUser, String measurementName, String unitSymbol, int bufferSize);
float fmap(float x, float in_min, float in_max, float out_min, float out_max);
void printUint16Hex(uint16_t value);
void printSerialNumber(uint16_t serial0, uint16_t serial1, uint16_t serial2);
float getMedianCO2();
void IRAM_ATTR buttonISR();
void printMutexStats();
bool takeMutexWithStats(SemaphoreHandle_t mutex, const String& funcName, TickType_t timeout, std::map<String, MutexStats>& statsMap);
void giveMutexWithStats(SemaphoreHandle_t mutex, const String& funcName, std::map<String, MutexStats>& statsMap);
void MutexMonitorTask(void *pvParameters);
void updateBuffers(void *pvParameters);
bool isWaterActuallyDetected();
void addRawValueTo1MinBuffer(TimestampedMeasurement buffer[], int &count, float value, unsigned long timestamp, int maxSize, unsigned long &lastReadTime);
void AggregationTask(void *pvParameters);
float calculateMedian(float values[], int count); // Ist schon oben als globale Funktion

// Hilfsfunktion zur formatierten Ausgabe des Alters eines Zeitstempels
String formatAgeString(unsigned long age_ms) { // Bereits oben definiert, hier nur zur Sicherheit der Vollständigkeit
    char buffer[10]; 
    float age_total_seconds = age_ms / 1000.0f;

    if (age_total_seconds < 60.0f) { 
        sprintf(buffer, "-%ds", (int)round(age_total_seconds));
    } else if (age_total_seconds < 3600.0f) { 
        float age_minutes = age_total_seconds / 60.0f;
        sprintf(buffer, "-%.1fm", age_minutes);
    } else { 
        float age_hours = age_total_seconds / 3600.0f;
        sprintf(buffer, "-%.1fh", age_hours);
    }
    return String(buffer);
}

// Neue Funktion für Buffer-Status
void printBufferStatus(const char* bufferName, int currentIndex, int maxSize) {
    int usedSpace = currentIndex >= maxSize ? maxSize : currentIndex;
    int remainingSpace = maxSize - usedSpace;
    
    Serial.print(bufferName); Serial.print(": "); Serial.print(usedSpace); Serial.print("/"); Serial.print(maxSize);
    if (currentIndex >= maxSize) {
        Serial.println(" (Buffer voll, überschreibe alte Daten)");
    } else {
        Serial.print(" (noch "); Serial.print(remainingSpace); Serial.println(" Plätze frei)");
    }
}

// Angepasste addToBuffer Funktion
template <typename T> // T muss .value und .timestamp haben, also TimestampedMeasurement
void addToBuffer(T buffer[], int &index, float value, const char* bufferName, unsigned long &lastUpdateTimestampVar, int maxSize)
{
    unsigned long currentTime = millis();
    int actualIndex = index % maxSize;
    
    buffer[actualIndex].value = value;
    buffer[actualIndex].timestamp = currentTime;
    
    index++;
    lastUpdateTimestampVar = currentTime; // Aktualisiert die zugehörige 'last...Update' Variable
    
    if (index % 10 == 0) { // Status alle 10 Einträge
        int usedSpace = index >= maxSize ? maxSize : index;
        Serial.print(bufferName); Serial.print(": "); Serial.print(usedSpace); Serial.print("/"); Serial.print(maxSize);
        if (index >= maxSize) {
            unsigned long oldestTimestamp = buffer[(index) % maxSize].timestamp;
            Serial.print(" (Buffer voll, ältester Wert: "); Serial.print((currentTime - oldestTimestamp) / 1000); Serial.println("s alt)");
        } else {
            Serial.print(" (noch "); Serial.print(maxSize - usedSpace); Serial.println(" Plätze frei)");
        }
    }
}


#include <WebServer.h>
WebServer server(80);


// =================================================================================================
// Einfache Webserver-Funktion: Zeigt Sensordaten als einfache HTML-Tabelle.
// =================================================================================================
void handleRootSimple() {
    Serial.println("\n--- Simple Web Request gestartet ---");
    server.sendHeader("Content-Type", "text/html; charset=utf-8");
    server.sendHeader("Connection", "close");
    String html = "<!DOCTYPE html><html lang='de'><head><meta charset='UTF-8'><title>Sensordaten</title>";
    // JavaScript Heartbeat/Reload alle 10 Sekunden
    html += "<script>\n"
        "setInterval(function(){location.reload();}, 10000);" // Heartbeat
        "function updateClock(){\n"
        "  var now = new Date();\n"
        "  document.getElementById('jsclock').innerText = now.toLocaleTimeString();\n"
        "}\n"
        "setInterval(updateClock, 1000);\n"
        "window.onload = updateClock;\n"
        "</script>";
    // Anzeige für laufendes JavaScript (Uhrzeit)
    html += "<div style='color:green; font-weight:bold;'>JavaScript läuft! Uhrzeit: <span id='jsclock'></span></div>";
    html += "</head><body>";
    // Fehler-Flag prüfen und Hinweis anzeigen
    extern bool wifiClientError; // Muss global deklariert werden
    if (wifiClientError) {
        html += "<div style='color:red; font-weight:bold; font-size:18px;'>Verbindungsfehler erkannt! Bitte Seite neu laden oder Verbindung prüfen.</div><br>";
    }
    html += "<h2>ESP32 Sensordaten</h2>";
    html += "<table border='1' cellpadding='8' style='border-collapse:collapse;'>";
    html += "<tr><th>Sensor</th><th>Wert</th><th>Einheit</th></tr>";
    html += "<tr><td>Temperatur Innen</td><td>" + String(sensorManagerV2.getValue(sensorManagerV2.tempInside), 1) + "</td><td>°C</td></tr>";
    html += "<tr><td>Luftfeuchte Innen</td><td>" + String(sensorManagerV2.getValue(sensorManagerV2.humidityInside), 1) + "</td><td>%</td></tr>";
    html += "<tr><td>Temperatur Außen</td><td>" + String(sensorManagerV2.getValue(sensorManagerV2.tempOutside), 1) + "</td><td>°C</td></tr>";
    html += "<tr><td>Luftfeuchte Außen</td><td>" + String(sensorManagerV2.getValue(sensorManagerV2.humidityOutside), 1) + "</td><td>%</td></tr>";
    html += "<tr><td>Lichtstärke</td><td>" + String((int)sensorManagerV2.getValue(sensorManagerV2.lux)) + "</td><td>lx</td></tr>";
    html += "<tr><td>CO2 Gehalt</td><td>" + String((int)sensorManagerV2.getValue(sensorManagerV2.co2)) + "</td><td>ppm</td></tr>";
    html += "<tr><td>Bodenfeuchte</td><td>" + String(sensorManagerV2.getValue(sensorManagerV2.soilMoisture), 1) + "</td><td>%</td></tr>";
    html += "</table>";
    time_t lt = BerlinTime.toLocal(now());
    char timeStr[32];
    sprintf(timeStr, "%02d.%02d.%04d %02d:%02d:%02d", day(lt), month(lt), year(lt), hour(lt), minute(lt), second(lt));
    html += "<p>Letzte Aktualisierung: ";
    html += timeStr;
    html += "</p>";
    // Button zum Weiterschalten der Display-Seite
    html += "<br><br><a href='/nextpage'><button style='padding: 10px 20px; font-size: 16px;'>Naechste Display Seite</button></a>";
    html += "</body></html>";
    server.send(200, "text/html", html);
    Serial.println("--- Simple Web Request erfolgreich beendet ---");
}

// Diese Funktion wird aufgerufen, wenn der Button auf der Webseite geklickt wird.
void handleNextPage() {
    // 1. Setze das globale Flag, das vom DisplayTask ausgewertet wird.
    buttonPressed = true;
    // 2. Sende eine Weiterleitung zurück an den Browser, damit er die Hauptseite neu lädt.
    server.sendHeader("Location", "/");
    server.send(302, "text/plain", "Weiterleiten...");
}

void setup()
{
    Serial.begin(115200);
    Serial.println("Serial gestartet mit 115200 Baud");

    startTime = millis();
    Serial.println("Startzeit gespeichert");

    pinMode(BUTTON_PIN, INPUT_PULLUP);
    attachInterrupt(digitalPinToInterrupt(BUTTON_PIN), buttonISR, FALLING);
    Serial.println("Boot-Button initialisiert");

    lastPageChange = millis();
    lastDisplayRefresh = millis();
    lastLogTime = millis();
    lastUserInteractionTime = millis();
    Serial.println("Zeitstempel initialisiert");

    i2c_mutex = xSemaphoreCreateMutex(); Serial.println("i2c_mutex erstellt");
    sd_mutex = xSemaphoreCreateMutex(); Serial.println("sd_mutex erstellt");
    pid_mutex = xSemaphoreCreateMutex(); Serial.println("pid_mutex erstellt");
    oneMinBuffer_mutex = xSemaphoreCreateMutex(); Serial.println("oneMinBuffer_mutex erstellt");
    longTermBuffer_mutex = xSemaphoreCreateMutex(); Serial.println("longTermBuffer_mutex erstellt");
    buffer_mutex = xSemaphoreCreateMutex(); Serial.println("buffer_mutex erstellt"); // Für updateBuffers Task

    pinMode(pumpPin, OUTPUT); digitalWrite(pumpPin, LOW); Serial.println("Pumpe initialisiert");
    pinMode(levelSensorPin, INPUT_PULLUP); Serial.println("Füllstandssensor initialisiert");
    pinMode(waterDetectionSensorPin, INPUT_PULLUP); Serial.println("Wassersensor (Sicherung) initialisiert");
    pinMode(humidifierPin, OUTPUT); digitalWrite(humidifierPin, LOW); Serial.println("Luftbefeuchter initialisiert");


    Wire.begin(); Serial.println("I2C initialisiert");

    if (!display.begin(i2c_Address, OLED_RESET)) {
        Serial.println(F("SH1106G Allozierung fehlgeschlagen")); for(;;);
    }
    Serial.println("Display initialisiert"); display.clearDisplay(); display.display();

    if (!lightMeter.begin(BH1750::CONTINUOUS_LOW_RES_MODE)) {
        Serial.println(F("BH1750 nicht gefunden"));
    } else {
        Serial.println("Lichtmesser initialisiert");
    }

    if (!sht3xd.begin(0x44)) {
        Serial.println(F("SHT3XD nicht gefunden"));
    } else {
        Serial.println("SHT3XD initialisiert");
        if (sht3xd.periodicStart(SHT3XD_REPEATABILITY_HIGH, SHT3XD_FREQUENCY_10HZ) != SHT3XD_NO_ERROR)
            Serial.println("[ERROR] SHT3XD periodischer Modus fehlgeschlagen");
    }

    if (aht.begin()) { Serial.println("AHT20 gefunden"); }
    else { Serial.println("AHT20 nicht gefunden"); }

    scd4x.begin(Wire); Serial.println("SCD4x initialisiert");
    uint16_t error; char errorMessage[256];
    error = scd4x.stopPeriodicMeasurement();
    if (error) { errorToString(error, errorMessage, 256); Serial.print("SCD4x Stopp-Fehler: "); Serial.println(errorMessage); }
    uint16_t serial0, serial1, serial2;
    error = scd4x.getSerialNumber(serial0, serial1, serial2);
    if (error) { errorToString(error, errorMessage, 256); Serial.print("SCD4x Seriennummer-Fehler: "); Serial.println(errorMessage); }
    else { printSerialNumber(serial0, serial1, serial2); }
    error = scd4x.startPeriodicMeasurement();
    if (error) { errorToString(error, errorMessage, 256); Serial.print("SCD4x Start-Fehler: "); Serial.println(errorMessage); }
    else { Serial.println("SCD4x periodische Messung gestartet"); }

    connectToWiFi();
    timeClient.begin(); timeClient.update();
    setSyncProvider(getNtpTime); setSyncInterval(60);
    Serial.println("Zeitsynchronisierung eingerichtet");

    initializeSD();

    pinMode(LED_PWM, OUTPUT);
    light_pid.SetMode(AUTOMATIC); light_pid.SetOutputLimits(0, 255);
    Serial.println("Light PID initialisiert");

    moisture_pid.SetMode(AUTOMATIC); moisture_pid.SetOutputLimits(3600000, 86400000);
    Serial.println("Moisture PID initialisiert");

    sensorManager.init();

    sensorManagerV2.init();

    // Tasks erstellen
    xTaskCreatePinnedToCore(WateringTask, "WateringTask", 10000, NULL, 1, NULL, 1); Serial.println("WateringTask erstellt");
    xTaskCreatePinnedToCore(SensorTask, "SensorTask", 10000, NULL, 2, NULL, 1); Serial.println("SensorTask erstellt (Prio 2)"); // Sensor hat höhere Prio
    xTaskCreatePinnedToCore(DisplayTask, "DisplayTask", 10000, NULL, 1, NULL, 1); Serial.println("DisplayTask erstellt");
    xTaskCreatePinnedToCore(Licht_Regeln, "Licht_Regeln", 10000, NULL, 1, NULL, 0); Serial.println("Licht_Regeln Task erstellt (Core 0)");
    xTaskCreatePinnedToCore(Temp_Regeln, "Temp_Regeln", 10000, NULL, 1, NULL, 1); Serial.println("Temp_Regeln Task erstellt");
    xTaskCreatePinnedToCore(Humidity_Regeln, "Humidity_Regeln", 10000, NULL, 1, NULL, 1); Serial.println("Humidity_Regeln Task erstellt");
    xTaskCreatePinnedToCore(Logging_Task, "Logging_Task", 10000, NULL, 1, NULL, 1); Serial.println("Logging_Task erstellt");
    xTaskCreatePinnedToCore(MutexMonitorTask, "MutexMonitorTask", 10000, NULL, 1, NULL, 0); Serial.println("MutexMonitorTask erstellt (Core 0)");
    xTaskCreatePinnedToCore(AggregationTask, "AggregationTask", 10000, NULL, 1, NULL, 1); Serial.println("AggregationTask erstellt (Prio 1)"); // Aggregation niedrigere Prio als Sensor
    xTaskCreatePinnedToCore(updateBuffers, "UpdateBuffers", 10000, NULL, 1, NULL, 1); Serial.println("UpdateBuffers Task erstellt");

    server.on("/", handleRootSimple);
    server.on("/nextpage", handleNextPage); // NEUE ZEILE
    // server.on("/neumorph", handleRootNeumorphism);
    server.begin();

    Serial.println("Setup abgeschlossen. Warte auf erste Messungen...");

    Serial.print("ESP32 ist erreichbar unter: http://");
    Serial.println(WiFi.localIP());
}

void loop()
{
    server.handleClient();
    vTaskDelay(pdMS_TO_TICKS(10)); // Damit FreeRTOS nicht blockiert
}

// SensorTask liest die Sensoren zyklisch aus
void SensorTask(void *pvParameters)
{
    const TickType_t xDelay = pdMS_TO_TICKS(100); // Kurzes Delay im Task
    const TickType_t mutexTimeoutShort = pdMS_TO_TICKS(50); // Kurzer Timeout für I2C Mutex

    for (;;)
    {
        unsigned long currentTime = millis();
        
        // 1. Temperatur und Luftfeuchtigkeit innen (SHT31D)
        if (sensorManager.sht31dErrorCount < sensorManager.MAX_SENSOR_ERRORS &&
            (sensorManager.isUpdateDue(sensorManager.temperatureInsideLastUpdate, sensorManager.TEMP_HUM_INSIDE_INTERVAL) ||
             sensorManager.isUpdateDue(sensorManager.humidityInsideLastUpdate, sensorManager.TEMP_HUM_INSIDE_INTERVAL)))
        {
            if (takeMutexWithStats(i2c_mutex, "SensorTask_SHT31D", mutexTimeoutShort, i2cMutexStats))
            {
                SHT31D shtResult = sht3xd.periodicFetchData();
                giveMutexWithStats(i2c_mutex, "SensorTask_SHT31D", i2cMutexStats); // Mutex so schnell wie möglich freigeben

                if (shtResult.error == SHT3XD_NO_ERROR) {
                    sensorManager.updateTemperatureInside(shtResult.t);
                    sensorManager.updateHumidityInside(shtResult.rh);
                    sensorManagerV2.setValue(sensorManagerV2.tempInside, shtResult.t);
                    sensorManagerV2.setValue(sensorManagerV2.humidityInside, shtResult.rh);
                    addRawValueTo1MinBuffer(temp1MinBuffer, temp1MinCount, shtResult.t, currentTime, MAX_1MIN_POINTS, lastTemp1MinRead);
                    addRawValueTo1MinBuffer(hum1MinBuffer, hum1MinCount, shtResult.rh, currentTime, MAX_1MIN_POINTS, lastHum1MinRead);
                    sensorManager.sht31dErrorCount = 0;
                } else {
                    Serial.print("SHT31D Fehler: "); Serial.println(shtResult.error);
                    sensorManager.sht31dErrorCount++;
                    if (sensorManager.sht31dErrorCount >= sensorManager.MAX_SENSOR_ERRORS) {
                        sensorManager.sht31dErrorTime = currentTime;
                        Serial.println("SHT31D zu viele Fehler - Cooldown aktiviert");
                    }
                }
            } else { /* Serial.println("SensorTask: SHT31D Mutex Timeout"); */ } // Optional: Loggen
        } else if (sensorManager.sht31dErrorCount >= sensorManager.MAX_SENSOR_ERRORS) {
             if ((currentTime - sensorManager.sht31dErrorTime) >= sensorManager.ERROR_COOLDOWN) {
                sensorManager.sht31dErrorCount = 0; Serial.println("SHT31D Cooldown beendet.");
            } else if (!sensorManager.useFallbackForIndoorSensors) { // Nur aktivieren wenn noch nicht aktiv
                 sensorManager.useFallbackForIndoorSensors = true;
                 Serial.println("FALLBACK: SHT31D Fehler, nutze AHT20 für Innenklima.");
            }
        }
        if (sensorManager.sht31dErrorCount < sensorManager.MAX_SENSOR_ERRORS && sensorManager.useFallbackForIndoorSensors) {
            sensorManager.useFallbackForIndoorSensors = false; // Zurücksetzen wenn SHT31D wieder geht
            Serial.println("FALLBACK: SHT31D wieder OK, nutze SHT31D für Innenklima.");
        }


        // 2. Temperatur und Luftfeuchtigkeit außen (AHT20)
        if (sensorManager.isUpdateDue(sensorManager.temperatureOutsideLastUpdate, sensorManager.TEMP_HUM_OUTSIDE_INTERVAL) ||
            sensorManager.isUpdateDue(sensorManager.humidityOutsideLastUpdate, sensorManager.TEMP_HUM_OUTSIDE_INTERVAL))
        {
            if (takeMutexWithStats(i2c_mutex, "SensorTask_AHT", mutexTimeoutShort, i2cMutexStats))
            {
                sensors_event_t humidityEvent, tempEvent;
                bool success = aht.getEvent(&humidityEvent, &tempEvent);
                giveMutexWithStats(i2c_mutex, "SensorTask_AHT", i2cMutexStats);

                if (success) {
                    sensorManager.updateTemperatureOutside(tempEvent.temperature);
                    sensorManager.updateHumidityOutside(humidityEvent.relative_humidity);
                    sensorManagerV2.setValue(sensorManagerV2.tempOutside, tempEvent.temperature);
                    sensorManagerV2.setValue(sensorManagerV2.humidityOutside, humidityEvent.relative_humidity);
                    if (sensorManager.useFallbackForIndoorSensors) { // Fallback aktiv
                        float estimatedIndoorTemp = tempEvent.temperature + sensorManager.INDOOR_OUTDOOR_OFFSET_TEMP;
                        float estimatedIndoorHum = min(100.0f, humidityEvent.relative_humidity + sensorManager.INDOOR_OUTDOOR_OFFSET_HUM);
                        sensorManager.updateTemperatureInside(estimatedIndoorTemp);
                        sensorManager.updateHumidityInside(estimatedIndoorHum);
                        sensorManagerV2.setValue(sensorManagerV2.tempInside, estimatedIndoorTemp);
                        sensorManagerV2.setValue(sensorManagerV2.humidityInside, estimatedIndoorHum);
                        addRawValueTo1MinBuffer(temp1MinBuffer, temp1MinCount, estimatedIndoorTemp, currentTime, MAX_1MIN_POINTS, lastTemp1MinRead);
                        addRawValueTo1MinBuffer(hum1MinBuffer, hum1MinCount, estimatedIndoorHum, currentTime, MAX_1MIN_POINTS, lastHum1MinRead);
                    }
                } else { Serial.println("AHT20 Lesefehler"); }
            } else { /* Serial.println("SensorTask: AHT20 Mutex Timeout"); */ }
        }

        // 3. Lichtsensor (BH1750)
        if (sensorManager.isUpdateDue(sensorManager.luxLastUpdate, sensorManager.LUX_INTERVAL))
        {
            if (takeMutexWithStats(i2c_mutex, "SensorTask_BH1750", mutexTimeoutShort, i2cMutexStats))
            {
                bool ready = lightMeter.measurementReady(false); // Non-blocking check
                float luxValue = 0;
                if (ready) luxValue = lightMeter.readLightLevel();
                giveMutexWithStats(i2c_mutex, "SensorTask_BH1750", i2cMutexStats);

                if (ready && luxValue >= 0) { // BH1750 gibt -1 bei Fehler
                    sensorManager.updateLux(luxValue);
                    sensorManagerV2.setValue(sensorManagerV2.lux, luxValue);
                    addRawValueTo1MinBuffer(light1MinBuffer, light1MinCount, luxValue, currentTime, MAX_1MIN_POINTS, lastLight1MinRead);
                    if (takeMutexWithStats(pid_mutex, "SensorTask_PID_Lux", pdMS_TO_TICKS(10), pidMutexStats)) { // Sehr kurzer Timeout
                        inputVal = luxValue;
                        giveMutexWithStats(pid_mutex, "SensorTask_PID_Lux", pidMutexStats);
                    }
                } else if (ready && luxValue < 0) { /* Serial.println("BH1750 Lesefehler"); */ }
            } else { /* Serial.println("SensorTask: BH1750 Mutex Timeout"); */ }
        }

        // 4. CO2-Sensor (SCD4x)
        if (sensorManager.isUpdateDue(sensorManager.co2LastUpdate, sensorManager.CO2_INTERVAL))
        {
            if (takeMutexWithStats(i2c_mutex, "SensorTask_SCD4x", mutexTimeoutShort, i2cMutexStats))
            {
                uint16_t errorScd; bool dataReady;
                errorScd = scd4x.getDataReadyFlag(dataReady);
                uint16_t co2ValScd; float tempScd, rhScd;
                if (!errorScd && dataReady) {
                    errorScd = scd4x.readMeasurement(co2ValScd, tempScd, rhScd);
                }
                giveMutexWithStats(i2c_mutex, "SensorTask_SCD4x", i2cMutexStats);

                if (!errorScd && dataReady) {
                    co2_raw = co2ValScd; // Für Medianfilter
                    float medianCo2Val = getMedianCO2();
                    sensorManager.updateCO2(medianCo2Val);
                    sensorManagerV2.setValue(sensorManagerV2.co2, medianCo2Val);
                    addRawValueTo1MinBuffer(co21MinBuffer, co21MinCount, medianCo2Val, currentTime, MAX_1MIN_POINTS, lastCO21MinRead);
                } else if (errorScd) { /* char eMsg[30]; errorToString(errorScd, eMsg, 30); Serial.print("SCD4x Fehler: "); Serial.println(eMsg); */ }
            } else { /* Serial.println("SensorTask: SCD4x Mutex Timeout"); */ }
        }

        // 5. Bodenfeuchtigkeit (analoger Sensor - kein Mutex nötig für analogRead)
        if (sensorManager.isUpdateDue(sensorManager.soilMoistureLastUpdate, sensorManager.SOIL_MOISTURE_INTERVAL))
        {
            uint16_t rawMoisture = analogRead(moistureSensorPin);
            float percentMoisture = fmap((float)rawMoisture, (float)moistureMin, (float)moistureMax, 100.0f, 0.0f);
            percentMoisture = constrain(percentMoisture, 0.0f, 100.0f);
            sensorManager.updateSoilMoisture(percentMoisture, rawMoisture);
            sensorManagerV2.setValue(sensorManagerV2.soilMoisture, percentMoisture);
            addRawValueTo1MinBuffer(soil1MinBuffer, soil1MinCount, percentMoisture, currentTime, MAX_1MIN_POINTS, lastSoil1MinRead);
        }
        
        vTaskDelay(xDelay);
    }
}

void WateringTask(void *pvParameters)
{
    const TickType_t xDelay = pdMS_TO_TICKS(1000); 

    for (;;) {
        time_t localTime = BerlinTime.toLocal(now()); // Korrekte Lokalzeit
        int currentHour = hour(localTime);
        int currentMinute = minute(localTime);
        int currentDay = day(localTime);
        
        if (currentHour == 0 && currentMinute == 0 && currentDay != lastWateringDay) {
            watering1Done = false; watering2Done = false; watering3Done = false;
            Serial.println("Bewässerungszähler zurückgesetzt.");
        }
        
        bool reservoirOK = waterReservoirIsFull();
        bool waterAtPump = isWaterActuallyDetected();

        if (currentHour >= wateringStartHour && currentHour < wateringEndHour) {
            if (!watering1Done && currentHour == watering1Hour && currentMinute == watering1Minute) {
                if (reservoirOK && waterAtPump) {
                    Serial.println("Starte 1. geplante Bewässerung."); startWatering();
                    lastWateringTime = millis(); lastWateringDay = currentDay; watering1Done = true;
                } else { Serial.print("1. Bewässerung übersprungen: Reservoir "); Serial.print(reservoirOK?"OK":"LEER"); Serial.print(", Wasserpumpe "); Serial.println(waterAtPump?"NASS":"TROCKEN");}
            }
            if (!watering2Done && currentHour == watering2Hour && currentMinute == watering2Minute) {
                 if (reservoirOK && waterAtPump) {
                    Serial.println("Starte 2. geplante Bewässerung."); startWatering();
                    lastWateringTime = millis(); lastWateringDay = currentDay; watering2Done = true;
                } else { Serial.print("2. Bewässerung übersprungen: Reservoir "); Serial.print(reservoirOK?"OK":"LEER"); Serial.print(", Wasserpumpe "); Serial.println(waterAtPump?"NASS":"TROCKEN");}
            }
            if (!watering3Done && currentHour == watering3Hour && currentMinute == watering3Minute) {
                 if (reservoirOK && waterAtPump) {
                    Serial.println("Starte 3. geplante Bewässerung."); startWatering();
                    lastWateringTime = millis(); lastWateringDay = currentDay; watering3Done = true;
                } else { Serial.print("3. Bewässerung übersprungen: Reservoir "); Serial.print(reservoirOK?"OK":"LEER"); Serial.print(", Wasserpumpe "); Serial.println(waterAtPump?"NASS":"TROCKEN");}
            }
        }
        
        static unsigned long lastWateringStatusLog = 0;
        if (millis() - lastWateringStatusLog > 3600000) { // Jede Stunde
            Serial.print("Bewässerungsstatus: "); Serial.print(watering1Done ? "1:OK " : "1:- ");
            Serial.print(watering2Done ? "2:OK " : "2:- "); Serial.println(watering3Done ? "3:OK" : "3:-");
            lastWateringStatusLog = millis();
        }
        
        vTaskDelay(xDelay);
    }
}

void DisplayTask(void *pvParameters)
{
    const TickType_t xDelay = pdMS_TO_TICKS(displayRefreshInterval); 
    for (;;)
    {
        unsigned long currentMillis = millis();

        if (buttonPressed) {
            buttonPressed = false;
            currentPage = (currentPage + 1) % totalPages;
            lastUserInteractionTime = currentMillis;
            userIsInteracting = true;
            // Sofortige Anzeige nach Button-Druck
            if (takeMutexWithStats(i2c_mutex, "DisplayTask_Button", pdMS_TO_TICKS(50), i2cMutexStats)){
                displayPage();
                giveMutexWithStats(i2c_mutex, "DisplayTask_Button", i2cMutexStats);
            }
        }

        if (userIsInteracting && (currentMillis - lastUserInteractionTime >= userInteractionTimeout)) {
            userIsInteracting = false;
            lastPageChange = currentMillis; // Reset auto page change timer
        }

        if (!userIsInteracting && (currentMillis - lastPageChange >= pageInterval)) {
            lastPageChange = currentMillis;
            currentPage = (currentPage + 1) % totalPages;
        }
        
        // Reguläre Anzeigeaktualisierung
        if (takeMutexWithStats(i2c_mutex, "DisplayTask_Regular", pdMS_TO_TICKS(50), i2cMutexStats)){
            displayPage(); // displayPage macht sein eigenes display.display()
            giveMutexWithStats(i2c_mutex, "DisplayTask_Regular", i2cMutexStats);
        } else { /* Serial.println("DisplayTask: Mutex Timeout"); */ }

        vTaskDelay(xDelay);
    }
}

void plotBufferData(TimestampedMeasurement buffer[], int totalWriteCount, int maxDisplayPointsUser, String measurementName, String unitSymbol, int bufferSize) {
    if (!takeMutexWithStats(longTermBuffer_mutex, "plotBufferData_Access", pdMS_TO_TICKS(100), longTermBufferMutexStats)) {
        display.clearDisplay(); display.setTextSize(1); display.setTextColor(SH110X_WHITE);
        display.setCursor(0,0); display.print(measurementName); display.setCursor(0,15); display.print("Buffer busy!");
        display.display();
        return;
    }

    int validDataCount = min(totalWriteCount, bufferSize);
    int pointsToRender = min(validDataCount, maxDisplayPointsUser);

    if (pointsToRender == 0) {
        giveMutexWithStats(longTermBuffer_mutex, "plotBufferData_Access", longTermBufferMutexStats);
        display.clearDisplay(); display.setTextSize(1); display.setTextColor(SH110X_WHITE);
        display.setCursor(5, 0); display.print(measurementName); display.setCursor(5, 20); display.print("No data yet.");
        display.display();
        return;
    }
    
    TimestampedMeasurement pointsToPlot[MAX_DISPLAY_POINTS];

    // Daten aus dem Ringpuffer lesen und ggf. komprimieren
    if (validDataCount <= MAX_DISPLAY_POINTS) {
        pointsToRender = validDataCount;
        for (int i = 0; i < pointsToRender; i++) {
            int readPos = (totalWriteCount - validDataCount + i + bufferSize) % bufferSize;
            pointsToPlot[i] = buffer[readPos];
        }
    } else {
        pointsToRender = MAX_DISPLAY_POINTS;
        float bucketSize = (float)validDataCount / MAX_DISPLAY_POINTS;
        for (int i = 0; i < MAX_DISPLAY_POINTS; i++) {
            int startIndex = floor(i * bucketSize);
            int endIndex = floor((i + 1) * bucketSize);
            endIndex = min(endIndex, validDataCount);
            if (startIndex >= endIndex) startIndex = max(0, endIndex -1);

            float sumVal = 0.0f; unsigned long sumTs = 0; int numInBucket = 0;
            for (int j = startIndex; j < endIndex; j++) {
                int readPos = (totalWriteCount - validDataCount + j + bufferSize) % bufferSize;
                sumVal += buffer[readPos].value;
                sumTs += buffer[readPos].timestamp;
                numInBucket++;
            }
            if (numInBucket > 0) {
                pointsToPlot[i].value = sumVal / numInBucket;
                pointsToPlot[i].timestamp = sumTs / numInBucket;
            } else {
                pointsToPlot[i] = (i > 0) ? pointsToPlot[i-1] : TimestampedMeasurement{0.0f, millis()};
            }
        }
    }
    giveMutexWithStats(longTermBuffer_mutex, "plotBufferData_Access", longTermBufferMutexStats);

    float minVal = pointsToPlot[0].value, maxVal = pointsToPlot[0].value;
    for (int i = 1; i < pointsToRender; i++) {
        if (pointsToPlot[i].value < minVal) minVal = pointsToPlot[i].value;
        if (pointsToPlot[i].value > maxVal) maxVal = pointsToPlot[i].value;
    }
    if (maxVal == minVal) {
        maxVal += 1.0f;
        minVal -= 1.0f;
    }

    display.clearDisplay();
    display.setTextColor(SH110X_WHITE);

    // Seitenzahl oben rechts direkt hier zeichnen
    // display.setTextSize(0);
    // extern int currentPage;
    // extern const int totalPages;
    // Seitenzahl rechts oben, bei zweistelligem Wert 5 Pixel weiter nach innen
    // int pageNum = currentPage + 1;
    // int pageNumX = SCREEN_WIDTH - (pageNum >= 10 ? 30 : 25);
    // display.setCursor(pageNumX, 0);
    // display.print(currentPage + 1); display.print("/"); display.print(totalPages);

    display.setTextSize(1);
    // Titel links oben
    display.setCursor(0, 0); display.print(measurementName); display.print(" ("); display.print(unitSymbol); display.println(")");

    // Max/Min Werte rechts
    int rightMargin = SCREEN_WIDTH - 25;
    if (unitSymbol == "C") {
        display.setCursor(rightMargin, 8); display.print(maxVal, 1);
        display.setCursor(rightMargin, 56); display.print(minVal, 1);
    } else {
        display.setCursor(rightMargin, 8); display.print((int)maxVal);
        display.setCursor(rightMargin, 56); display.print((int)minVal);
    }

    // Grafenfläche: oben 10 Pixel frei, rechts 25 Pixel frei
    int xStart = 0;
    int yStart = 10; // 10 Pixel Abstand oben
    int graphWidth = SCREEN_WIDTH - xStart - 30; // 25 Pixel Abstand rechts
    int graphHeight = SCREEN_HEIGHT - yStart; // Höhe ab 10 Pixel bis unten

    if (pointsToRender > 1) {
        for (int i = 0; i < pointsToRender; i++) {
            int x = xStart + round((float)i * graphWidth / (pointsToRender - 1));
            int y = yStart + graphHeight - round(((pointsToPlot[i].value - minVal) * graphHeight) / (maxVal - minVal));
            y = constrain(y, yStart, yStart + graphHeight);
            // Nur Pixel zeichnen, wenn sie NICHT in der reservierten Ecke sind
            if (!(x >= SCREEN_WIDTH - 25 && y <= 10)) {
                display.drawPixel(x, y, SH110X_WHITE);
            }
        }
    } else if (pointsToRender == 1) {
        int x = xStart + graphWidth / 2;
        int y = yStart + graphHeight / 2;
        if (!(x >= SCREEN_WIDTH - 25 && y <= 10)) {
            display.drawPixel(x, y, SH110X_WHITE);
        }
    }

    display.display();
}


void displayPage()
{
    time_t lt = BerlinTime.toLocal(now()); // Korrekte Lokalzeit
    tmElements_t startDate;
    startDate.Year = CalendarYrToTm(2024); startDate.Month = 8; startDate.Day = 24;
    startDate.Hour = 0; startDate.Minute = 0; startDate.Second = 0;
    long daysPassed = (lt - makeTime(startDate)) / SECONDS_PER_DAY;

    bool isPlotPage = (currentPage >= 8 && currentPage <= 12);

    display.clearDisplay();
    display.setTextColor(SH110X_WHITE);

    // Seitenzahl immer anzeigen, auch auf Plot-Seiten
    // display.setTextSize(0);
    // display.setCursor(SCREEN_WIDTH - 25, 0); // Rechtsbündig für Seitenzahl
    // display.print(currentPage + 1); display.print("/"); display.print(totalPages);

    display.setTextSize(1); // Standard Textgröße
    display.setCursor(0,10); // Standard Y-Start für die meisten Seiten

    switch (currentPage) {
    case 0:
        display.drawBitmap(0, 0, plant_and_drop_32x32_bitmap, 32, 32, SH110X_WHITE);
        display.setCursor(0, 35); display.println("Vanilla Planifolia");
        display.setCursor(0, 47); display.print("Tage: "); display.print(daysPassed);
        display.display();
        break;
    case 1:
        display.println("Licht");
        display.setCursor(0, 22); display.print(lux, 1); display.println(" lx");
        display.setCursor(0, 34); display.print("Soll: "); display.print(Setpoint, 0); display.println(" lx");
        display.setCursor(0, 46); display.print("PWM: "); display.print((outputVal / 255.0) * 100.0, 0); display.println(" %");
        display.display();
        break;
    case 2:
        display.println("Temperatur");
        display.setCursor(0, 22); display.print("In: "); display.print(temperatureInside, 1); display.println(" C");
        display.setCursor(0, 34); display.print("Out: "); display.print(temperatureOutside, 1); display.println(" C");
        display.display();
        break;
    case 3:
        display.println("Luftfeuchtigkeit");
        display.setCursor(0, 22); display.print("In: "); display.print(humidityInside, 1); display.println(" %");
        display.setCursor(0, 34); display.print("Out: "); display.print(humidityOutside, 1); display.println(" %");
        display.display();
        break;
    case 4:
        display.println("Bodenfeuchte");
        display.setCursor(0, 22); display.print(soilMoisturePercent, 1); display.println("%");
        if(lastWateringTime > 0) {
             display.setCursor(0, 34); display.print("Letzte Bew: "); display.print((float)(millis() - lastWateringTime) / 3600000.0f, 1); display.println("h");
        } else {
             display.setCursor(0, 34); display.println("Noch keine Bew.");
        }
        display.setCursor(0, 46); display.print("Pumpe: "); display.println(isWaterActuallyDetected() ? "NASS" : "TROCKEN");
        display.display();
        break;
    case 5: // Regelung Bodenfeuchte
        display.println("Regelung Boden");
        display.setCursor(0, 22); display.print("Ist: "); display.print(moistureInput, 0); // Raw Wert
        display.setCursor(0, 34); display.print("Soll: "); display.print(moistureSetpoint, 0); // Raw Wert
        display.display();
        break;
    case 6:
        display.println("WiFi & Zeit");
        display.setCursor(0, 22); display.print("RSSI: "); display.print(WiFi.RSSI()); display.println(" dBm");
        display.setCursor(0, 34); display.println(WiFi.localIP().toString());
        display.setCursor(0, 46);
        char timeBuf[10]; sprintf(timeBuf, "%02d:%02d:%02d", hour(lt), minute(lt), second(lt));
        display.println(timeBuf);
        display.display();
        break;
    case 7:
        display.println("CO2");
        display.setCursor(0, 22); display.print(co2_ppm, 0); display.println(" ppm");
        display.display();
        break;
    case 8:
        plotBufferData(temperatureBuffer, temperatureIndex, MAX_DISPLAY_POINTS, "Temp.", "C", MAX_24H_POINTS);
        break;
    case 9:
        plotBufferData(humidityBuffer, humidityIndex, MAX_DISPLAY_POINTS, "Hum.", "%", MAX_24H_POINTS);
        break;
    case 10:
        plotBufferData(lightBuffer, lightIndex, MAX_DISPLAY_POINTS, "Light", "lx", MAX_24H_POINTS);
        break;
    case 11:
        plotBufferData(soilMoistureBuffer, soilMoistureIndex, MAX_DISPLAY_POINTS, "Soil", "%", MAX_24H_POINTS);
        break;
    case 12:
        plotBufferData(co2Buffer, co2Index, MAX_DISPLAY_POINTS, "CO2", "ppm", MAX_24H_POINTS);
        break;
    case 13: // Rohdaten-Anzahl
        display.setTextSize(1);
        display.setCursor(0, 0); display.println("Rohdaten je Messwert:");
        display.setCursor(0, 14); display.print("Temp in.: "); display.println(temp1MinCount);
        display.setCursor(0, 24); display.print("Temp out.: "); display.println(temp1MinCount);
        display.setCursor(0, 34); display.print("Hum in.: "); display.println(hum1MinCount);
        display.setCursor(0, 44); display.print("Hum out.: "); display.println(hum1MinCount);
        display.setCursor(64, 14); display.print("Licht: "); display.println(light1MinCount);
        display.setCursor(64, 24); display.print("Boden: "); display.println(soil1MinCount);
        display.setCursor(64, 34); display.print("CO2: "); display.println(co21MinCount);
        display.display();
        break;
    case 14:
        plotBufferData(temp1MinBuffer, temp1MinCount, MAX_1MIN_POINTS, "Temp. 1min", "C", MAX_1MIN_POINTS);
        break;
    case 15:
        plotBufferData(hum1MinBuffer, hum1MinCount, MAX_1MIN_POINTS, "Hum. 1min", "%", MAX_1MIN_POINTS);
        break;
    case 16:
        plotBufferData(light1MinBuffer, light1MinCount, MAX_1MIN_POINTS, "Light 1min", "lx", MAX_1MIN_POINTS);
        break;
    case 17:
        plotBufferData(soil1MinBuffer, soil1MinCount, MAX_1MIN_POINTS, "Soil 1min", "%", MAX_1MIN_POINTS);
        break;
    case 18:
        plotBufferData(co21MinBuffer, co21MinCount, MAX_1MIN_POINTS, "CO2 1min", "ppm", MAX_1MIN_POINTS);
        break;
    default:
        display.println("Unbekannte Seite");
        display.display();
        break;
    }
}

float fmap(float x, float in_min, float in_max, float out_min, float out_max) {
    return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

void initializeSD() {
    if (!SD.begin(sd_csPin)) { Serial.println("SD Init fehlgeschlagen!"); return; }
    Serial.println("SD-Karte initialisiert.");
    if (!SD.exists(csvFileName)) {
        File dataFile = SD.open(csvFileName, FILE_WRITE);
        if (dataFile) {
            dataFile.println("Timestamp,TempIn,HumIn,TempOut,HumOut,Lux,SoilPerc,SoilRaw,CO2"); // SoilRaw hinzugefügt
            dataFile.close();
            Serial.println("CSV Header geschrieben.");
        } else { Serial.println("Fehler beim Erstellen der CSV-Datei"); }
    }
}

void writeCSV(String data) {
    if (takeMutexWithStats(sd_mutex, "writeCSV", pdMS_TO_TICKS(200), sdMutexStats)) { // Timeout hinzugefügt
        File dataFile = SD.open(csvFileName, FILE_APPEND);
        if (dataFile) { dataFile.println(data); dataFile.close(); }
        else { Serial.println("Fehler beim Öffnen der CSV zum Schreiben"); }
        giveMutexWithStats(sd_mutex, "writeCSV", sdMutexStats);
    } else { Serial.println("writeCSV: SD Mutex Timeout"); }
}

void connectToWiFi() {
    for (int i = 0; i < wifiCredentialCount; ++i) {
        Serial.print("Verbinde mit WiFi SSID: ");
        Serial.println(wifiCredentials[i].ssid);
        WiFi.disconnect(true); // Trenne und lösche alte Einstellungen
        delay(1000); // Warte kurz, damit alles wirklich getrennt ist
        WiFi.begin(wifiCredentials[i].ssid, wifiCredentials[i].password);
        int attempts = 0;
        while (WiFi.status() != WL_CONNECTED && attempts < 20) { // Max 20 Versuche (10s)
            delay(500); Serial.print("."); attempts++;
        }
        if (WiFi.status() == WL_CONNECTED) {
            Serial.println("\nVerbunden! IP: " + WiFi.localIP().toString());
            return;
        } else {
            Serial.println("\nWiFi Verbindung zu " + String(wifiCredentials[i].ssid) + " fehlgeschlagen.");
        }
    }
    Serial.println("\nKeine WiFi Verbindung möglich!");
}

time_t getNtpTime() { // Wird von TimeLib als SyncProvider aufgerufen
    if (WiFi.status() == WL_CONNECTED) { // Nur wenn WiFi verbunden ist
      timeClient.update(); // Holt Zeit vom NTP Server
      return timeClient.getEpochTime(); // Gibt UTC Zeit zurück
    }
    return 0; // Ungültige Zeit, wenn kein WiFi
}

// void updateSetpoints() { /* Leer */ } // Funktionsprototyp existiert, aber Funktion ist leer

bool waterReservoirIsFull() { return digitalRead(levelSensorPin) == LOW; } // LOW = voll
bool isWaterActuallyDetected() { return digitalRead(waterDetectionSensorPin) == LOW; } // LOW = nass

void startWatering() {
    if (isWaterActuallyDetected()) {
        Serial.println("Pumpe AN");
        digitalWrite(pumpPin, HIGH);
        vTaskDelay(pdMS_TO_TICKS(wateringDuration)); // Task blockiert hier für die Dauer
        digitalWrite(pumpPin, LOW);
        Serial.println("Pumpe AUS");
    } else {
        Serial.println("Pumpe NICHT gestartet: Kein Wasser am Pumpsensor!");
    }
}

String twoDigits(int num) { return (num < 10) ? "0" + String(num) : String(num); }

void printSerialNumber(uint16_t serial0, uint16_t serial1, uint16_t serial2) {
    Serial.print("SCD4x Serial: 0x"); printUint16Hex(serial0); printUint16Hex(serial1); printUint16Hex(serial2); Serial.println();
}
void printUint16Hex(uint16_t value) { char buf[5]; sprintf(buf, "%04X", value); Serial.print(buf); }

float getMedianCO2() {
    co2Values[co2ValueIndex] = co2_raw; // co2_raw wird im SensorTask aktualisiert
    co2ValueIndex = (co2ValueIndex + 1) % MEDIAN_WINDOW;
    uint16_t sortedValues[MEDIAN_WINDOW];
    memcpy(sortedValues, co2Values, sizeof(co2Values));
    std::sort(sortedValues, sortedValues + MEDIAN_WINDOW); // std::sort ist effizienter
    return (float)sortedValues[MEDIAN_WINDOW / 2];
}

void Licht_Regeln(void *pvParameters) {
    const TickType_t xDelay = pdMS_TO_TICKS(10); // Häufigere Prüfung für sanftere Übergänge, war 5ms
    const float SUNSET_LUX_THRESHOLD = 5.0f; // Niedrigerer Threshold für Abschaltung

    for (;;) {
        time_t lt = BerlinTime.toLocal(now());
        int currentHour = hour(lt); int currentMinute = minute(lt);
        int currentTimeInMinutes = currentHour * 60 + currentMinute;
        int startTimeInMinutes = sunriseHour * 60 + sunriseMinute;
        int endTimeInMinutes = startTimeInMinutes + totalLightDurationMinutes;

        double newSetpoint = 0; // Lokale Variable für Setpoint

        if (currentTimeInMinutes >= startTimeInMinutes && currentTimeInMinutes < endTimeInMinutes) {
            int minutesSinceSunrise = currentTimeInMinutes - startTimeInMinutes;
            if (minutesSinceSunrise < sunriseDurationMinutes) { // Sonnenaufgang
                float progress = (float)minutesSinceSunrise / sunriseDurationMinutes;
                newSetpoint = maxLightIntensity * progress * progress; // Quadratischer Anstieg für sanfteren Start
            } else if (minutesSinceSunrise < (totalLightDurationMinutes - sunsetDurationMinutes)) { // Tageslicht
                newSetpoint = maxLightIntensity;
            } else { // Sonnenuntergang
                float minutesUntilEnd = (float)(endTimeInMinutes - currentTimeInMinutes);
                float progress = minutesUntilEnd / sunsetDurationMinutes;
                newSetpoint = maxLightIntensity * progress * progress; // Quadratischer Abfall
                if (newSetpoint < SUNSET_LUX_THRESHOLD) newSetpoint = 0;
            }
        }
        
        if (takeMutexWithStats(pid_mutex, "Licht_Regeln_PID", pdMS_TO_TICKS(20), pidMutexStats)) { // Kurzer Timeout
            Setpoint = newSetpoint; // Globalen Setpoint für Display aktualisieren
            // inputVal wird von SensorTask gesetzt
            light_pid.Compute();
            analogWrite(LED_PWM, outputVal);
            giveMutexWithStats(pid_mutex, "Licht_Regeln_PID", pidMutexStats);
        } else { /* Serial.println("Licht_Regeln: PID Mutex Timeout"); */ }
        
        vTaskDelay(xDelay);
    }
}

void Temp_Regeln(void *parameter) { // Aktuell leer, keine Logik
    for (;;) { vTaskDelay(pdMS_TO_TICKS(5000)); } // Prüft alle 5s
}

void Humidity_Regeln(void *parameter) {
    const TickType_t xDelay = pdMS_TO_TICKS(2000); // Alle 2 Sekunden
    
    for (;;) {
        // Direkter Zugriff auf globale Variable humidityInside ist OK, da sie volatile ist
        // und der SensorTask sie aktualisiert. Für die Regelung ist eine kleine Verzögerung unkritisch.
        if (humidifierState) { // Befeuchter ist AN
            if (humidityInside >= (humiditySetpoint + humidityHysteresis / 2.0f)) { // Ausschalten bei Soll + halbe Hysterese
                humidifierState = false; digitalWrite(humidifierPin, LOW);
                Serial.println("Luftbefeuchter AUS");
            }
        } else { // Befeuchter ist AUS
            if (humidityInside <= (humiditySetpoint - humidityHysteresis / 2.0f)) { // Einschalten bei Soll - halbe Hysterese
                humidifierState = true; digitalWrite(humidifierPin, HIGH);
                Serial.println("Luftbefeuchter EIN");
            }
        }
        
        static unsigned long lastHumDebug = 0;
        if(millis() - lastHumDebug > 30000){
             Serial.print("Luftfeuchtigkeitsregelung: Ist="); Serial.print(humidityInside, 1);
             Serial.print("%, Soll="); Serial.print(humiditySetpoint, 1);
             Serial.print("%, Befeuchter="); Serial.println(humidifierState ? "EIN" : "AUS");
            lastHumDebug = millis();
        }
        vTaskDelay(xDelay);
    }
}

void Logging_Task(void *pvParameters) {
    const TickType_t xDelay = pdMS_TO_TICKS(1000); // Prüft jede Sekunde
    for (;;) {
        unsigned long currentTime = millis();
        if (millis() >= delayBeforeLogging && (currentTime - lastLogTime >= logInterval)) { // Warte 10s nach Start mit Loggen
            time_t lt = BerlinTime.toLocal(now());
            String data = String(lt) + "," +
                          String(temperatureInside, 2) + "," + String(humidityInside, 2) + "," +
                          String(temperatureOutside, 2) + "," + String(humidityOutside, 2) + "," +
                          String(lux, 2) + "," + String(soilMoisturePercent, 2) + "," +
                          String(moistureValue) + "," + String(co2_ppm, 1);
            writeCSV(data);
            lastLogTime = currentTime;
            
            static int logCountForStats = 0; logCountForStats++;
            if (logCountForStats >= 10) { // Alle 10 Logs (ca. 10 Min)
                logCountForStats = 0; printBufferStats(); // Ruft die neue Funktion auf
            }
        }
        vTaskDelay(xDelay);
    }
}

void printBufferStats() { // Neue Funktion für Buffer-Statistiken
    unsigned long currentMillis = millis();
    Serial.println("\n=== BUFFER STATISTIK (" + String(currentMillis/1000) + "s) ===");
    auto printDetails = [&](const char* name, int index, int maxSize, TimestampedMeasurement buf[]) {
        int used = min(index, maxSize);
        unsigned long oldestTs = 0;
        if (used > 0) { oldestTs = (index >= maxSize) ? buf[index % maxSize].timestamp : buf[0].timestamp; }
        Serial.print(name); Serial.print(": "); Serial.print(used); Serial.print("/"); Serial.print(maxSize);
        Serial.print(" ("); Serial.print(used * 100.0 / maxSize, 1); Serial.print("%)");
        if (oldestTs > 0) { Serial.print(", Ältester: "); Serial.print((currentMillis - oldestTs) / 60000UL); Serial.print("min"); }
        Serial.println();
    };
    printDetails("Temp 24h", temperatureIndex, MAX_24H_POINTS, temperatureBuffer);
    printDetails("Hum 24h ", humidityIndex, MAX_24H_POINTS, humidityBuffer);
    printDetails("Light 24h", lightIndex, MAX_24H_POINTS, lightBuffer);
    printDetails("CO2 24h ", co2Index, MAX_24H_POINTS, co2Buffer);
    printDetails("Soil 24h", soilMoistureIndex, MAX_24H_POINTS, soilMoistureBuffer);
    
    size_t totalBufMem = sizeof(temperatureBuffer) + sizeof(humidityBuffer) + sizeof(lightBuffer) + sizeof(soilMoistureBuffer) + sizeof(co2Buffer) +
                         sizeof(temp1MinBuffer) + sizeof(hum1MinBuffer) + sizeof(light1MinBuffer) + sizeof(co21MinBuffer) + sizeof(soil1MinBuffer);
    Serial.print("Gesamter Buffer-Speicher: "); Serial.print(totalBufMem / 1024.0f, 2); Serial.println(" KB");
    Serial.println("=======================");
}


void IRAM_ATTR buttonISR() { // Muss IRAM_ATTR sein für schnelle Reaktion
    unsigned long currentTime = millis();
    if (currentTime - lastButtonPressTime > debounceDelay) {
        lastButtonPressTime = currentTime;
        buttonPressed = true; // volatile Flag setzen
    }
}

void printMutexStats() { // Überprüft und gibt Mutex-Statistiken aus
    unsigned long currentTime = millis();
    if (currentTime - lastStatsOutputTime < STATS_OUTPUT_INTERVAL) return;
    lastStatsOutputTime = currentTime;
    
    Serial.println("========== MUTEX-STATISTIKEN (" + String(currentTime/1000) + "s) ==========");
    auto printMap = [&](const char* title, const std::map<String, MutexStats>& statMap) {
        Serial.println(title);
        for (auto const& [funcName, stats] : statMap) { // C++17 structured binding
            Serial.print("  "); Serial.print(funcName); Serial.print(": ");
            Serial.print(stats.totalTimeHeld / 1000.0f, 3); Serial.print("s (");
            Serial.print((stats.totalTimeHeld * 100.0f) / max(1UL,currentTime), 2); // Verhindere Division durch 0 bei Start
            Serial.println("%)");
        }
    };
    printMap("--- I2C-MUTEX ---", i2cMutexStats);
    printMap("--- SD-MUTEX ---", sdMutexStats);
    printMap("--- PID-MUTEX ---", pidMutexStats);
    printMap("--- 1MIN-BUF-MUTEX ---", oneMinBufferMutexStats);
    printMap("--- LONGTERM-BUF-MUTEX ---", longTermBufferMutexStats);
    printMap("--- GEN-BUF-MUTEX ---", {{ "updateBuffers", oneMinBufferMutexStats["updateBuffers"] }}); // Workaround für buffer_mutex, da es keine eigene Map hat

    Serial.println("===============================================");
}


bool takeMutexWithStats(SemaphoreHandle_t mutex, const String& funcName, TickType_t timeout, std::map<String, MutexStats>& statsMap) {
    if (mutex == NULL) { Serial.println("takeMutex: NULL Mutex!"); return false; }
    unsigned long preTakeTime = millis();
    bool success = (xSemaphoreTake(mutex, timeout) == pdTRUE);
    if (success) {
        statsMap[funcName].lastAcquireTime = millis(); // Zeit nach erfolgreichem Take
        statsMap[funcName].isHeld = true;
    } else {
        // Optional: Loggen, wenn Mutex nicht erhalten wurde
        // unsigned long waitTime = millis() - preTakeTime;
        // if (waitTime > 10) { // Nur loggen wenn Wartezeit signifikant war
        //    Serial.print("WARN: Mutex "); Serial.print(funcName); Serial.print(" nicht erhalten nach "); Serial.print(waitTime); Serial.println("ms");
        // }
    }
    return success;
}

void giveMutexWithStats(SemaphoreHandle_t mutex, const String& funcName, std::map<String, MutexStats>& statsMap) {
    if (mutex == NULL) { Serial.println("giveMutex: NULL Mutex!"); return; }
    if (statsMap[funcName].isHeld) {
        statsMap[funcName].totalTimeHeld += (millis() - statsMap[funcName].lastAcquireTime);
        statsMap[funcName].isHeld = false;
    } else {
        // Optional: Loggen, wenn Mutex freigegeben wird, der nicht als gehalten markiert war
        // Serial.print("WARN: Mutex "); Serial.print(funcName); Serial.println(" freigegeben, war aber nicht als gehalten markiert.");
    }
    xSemaphoreGive(mutex);
}

void MutexMonitorTask(void *pvParameters) {
    for (;;) { printMutexStats(); vTaskDelay(pdMS_TO_TICKS(STATS_OUTPUT_INTERVAL)); } // Verwendet das Intervall
}

// updateBuffers Task: Überträgt Daten von SensorManager Flags in die Langzeit-Buffer
void updateBuffers(void *pvParameters) {
    const TickType_t xDelay = pdMS_TO_TICKS(500); // Alle 0.5s prüfen
    // Wichtig: Die `last...Update` Variablen sind für die *Langzeitbuffer*
    // Die SensorManager Flags (`...Updated`) zeigen an, dass *neue Rohdaten* da sind.
    for (;;) {
        // Nimmt den allgemeinen `buffer_mutex`. Die Statistik wird aktuell in `oneMinBufferMutexStats` unter dem Key "updateBuffers" geschrieben.
        // Das ist nicht ideal, aber funktioniert. Besser wäre eine eigene `bufferMutexStats` Map.
        if (takeMutexWithStats(buffer_mutex, "updateBuffers", pdMS_TO_TICKS(100), oneMinBufferMutexStats)) {
            if (sensorManager.temperatureInsideUpdated) {
                //addToBuffer(temperatureBuffer, temperatureIndex, temperatureInside, "Temp24h", lastTemperatureUpdate, MAX_24H_POINTS); // Alte Logik
                // sensorManager.temperatureInsideUpdated = false; // Reset Flag in SensorManager
                // Neue Logik: SensorTask schreibt direkt in 1Min Buffer, AggregationTask schreibt in 24h Buffer
            }
            if (sensorManager.humidityInsideUpdated) {
                //addToBuffer(humidityBuffer, humidityIndex, humidityInside, "Hum24h", lastHumidityUpdate, MAX_24H_POINTS);
                // sensorManager.humidityInsideUpdated = false;
            }
            // ... und so weiter für andere Sensoren, wenn `updateBuffers` direkt in 24h schreiben soll.
            // Aktuell ist die Logik so, dass SensorTask in 1Min schreibt und AggregationTask in 24h.
            // Dieser Task `updateBuffers` scheint in der aktuellen Konfiguration überflüssig zu werden,
            // WENN der SensorTask die addRawValueTo1MinBuffer direkt aufruft und AggregationTask die 24h-Buffer füllt.
            // Ich lasse ihn vorerst drin, aber kommentiere die addToBuffer Aufrufe aus, da sie nun redundant wären.

            giveMutexWithStats(buffer_mutex, "updateBuffers", oneMinBufferMutexStats);
        } else { /* Serial.println("updateBuffers: Mutex Timeout"); */ }
        vTaskDelay(xDelay);
    }
}


// Hilfsfunktion zum Hinzufügen von Werten zum 1-Minuten-Buffer
void addRawValueTo1MinBuffer(TimestampedMeasurement buffer[], int &count, float value, unsigned long timestamp, int maxSize, unsigned long &lastReadTimeVar) {
    // Nimmt den spezifischen oneMinBuffer_mutex. Statistiken werden in oneMinBufferMutexStats gesammelt.
    if (takeMutexWithStats(oneMinBuffer_mutex, "addRawTo1Min", pdMS_TO_TICKS(20), oneMinBufferMutexStats)) { // Kurzer Timeout
        // Überprüfen, ob die Zeit für einen neuen Eintrag in den 1-Minuten-Buffer abgelaufen ist (jede Sekunde)
        // Der SensorTask ruft dies mit `currentTime` auf, das sollte reichen. Die Prüfung hier ist eine zusätzliche Sicherheit.
        // if (timestamp - lastReadTimeVar >= 1000) { // Wenn mehr als 1 Sekunde seit dem letzten Eintrag vergangen ist.
            int actualIndex = count % maxSize; // `count` ist der absolute Zähler
            buffer[actualIndex].value = value;
            buffer[actualIndex].timestamp = timestamp; // Verwende den übergebenen Zeitstempel
            count++; 
            lastReadTimeVar = timestamp; // Aktualisiere den Zeitstempel der letzten Lesung für diesen spezifischen Buffer
        // }
        giveMutexWithStats(oneMinBuffer_mutex, "addRawTo1Min", oneMinBufferMutexStats);
    } else { /* Serial.println("WARN: addRawValueTo1MinBuffer Mutex Timeout."); */ }
}


// Hilfsfunktion zum Kopieren und Sortieren von Float-Arrays für Median
float calculateMedian(float values[], int count) { // Globale Funktion, kein Mutex hier nötig
    if (count == 0) return NAN; // NAN (Not-A-Number) für ungültigen Fall
    if (count == 1) return values[0];

    float tempArray[count]; // VLA (Variable Length Array) - OK für ESP32 in Grenzen
    memcpy(tempArray, values, count * sizeof(float));
    std::sort(tempArray, tempArray + count);

    if (count % 2 == 0) { // Gerade Anzahl
        return (tempArray[count / 2 - 1] + tempArray[count / 2]) / 2.0f;
    } else { // Ungerade Anzahl
        return tempArray[count / 2];
    }
}

// Neuer Aggregation Task
void AggregationTask(void *pvParameters) {
    const TickType_t xDelay = pdMS_TO_TICKS(1000); // Prüft jede Sekunde
    unsigned long lastAggregationTime = 0;

    for (;;) {
        unsigned long currentTime = millis();

        if (currentTime - lastAggregationTime >= 60000) { // Jede Minute
            lastAggregationTime = currentTime;
            // Serial.println("AggregationTask: Starte minütliche Aggregation.");

            // Temporäre Arrays für Medianberechnung (auf dem Stack, sollte OK sein für MAX_1MIN_POINTS=60)
            float currentMinuteValues[MAX_1MIN_POINTS]; 
            int numValues;
            float medianValue;

            // Lambda für die Aggregationslogik
            auto aggregateSensor = [&](
                const char* sensorName,
                TimestampedMeasurement oneMinBuf[], int& oneMinIdx, unsigned long& lastReadTsVar, // Referenzen für 1-Min-Buffer
                TimestampedMeasurement longTermBuf[], int& longTermIdx, unsigned long& lastUpdateTsVar, // Referenzen für 24h-Buffer
                int max24hPoints
            ) {
                if (takeMutexWithStats(oneMinBuffer_mutex, String("Agg_Read_") + sensorName, pdMS_TO_TICKS(100), oneMinBufferMutexStats)) {
                    numValues = min(oneMinIdx, MAX_1MIN_POINTS); // Wie viele Werte sind tatsächlich in der letzten Minute gesammelt worden
                    if (numValues > 0) {
                        for (int i = 0; i < numValues; i++) {
                            // Lese die letzten `numValues` aus dem Ringpuffer
                            currentMinuteValues[i] = oneMinBuf[(oneMinIdx - numValues + i + MAX_1MIN_POINTS) % MAX_1MIN_POINTS].value;
                        }
                    }
                    oneMinIdx = 0; // Zähler für den 1-Minuten Puffer für diesen Sensor zurücksetzen
                    lastReadTsVar = currentTime; // Setze den letzten Lesezeitpunkt für den 1-Minuten Puffer zurück
                    giveMutexWithStats(oneMinBuffer_mutex, String("Agg_Read_") + sensorName, oneMinBufferMutexStats);

                    if (numValues > 0) {
                        medianValue = calculateMedian(currentMinuteValues, numValues);
                        if (!isnan(medianValue)) { // Nur gültige Mediane speichern
                            if (takeMutexWithStats(longTermBuffer_mutex, String("Agg_Write_") + sensorName, pdMS_TO_TICKS(100), longTermBufferMutexStats)) {
                                int actualIdxLT = longTermIdx % max24hPoints;
                                longTermBuf[actualIdxLT].value = medianValue;
                                longTermBuf[actualIdxLT].timestamp = currentTime;
                                longTermIdx++;
                                lastUpdateTsVar = currentTime; // Aktualisiere den Timestamp für den Langzeitbuffer
                                giveMutexWithStats(longTermBuffer_mutex, String("Agg_Write_") + sensorName, longTermBufferMutexStats);
                                // Serial.print(sensorName); Serial.print(" Median: "); Serial.println(medianValue);
                            } else { /* Serial.print("WARN: Aggregation Konnte LongTermBuffer-Mutex für "); Serial.println(sensorName); */ }
                        } else { /* Serial.print("WARN: Aggregation ungültiger Median für "); Serial.println(sensorName); */ }
                    }
                } else { /* Serial.print("WARN: Aggregation Konnte 1-Minuten-Buffer-Mutex für "); Serial.println(sensorName); */ }
            };
            
            aggregateSensor("Temp", temp1MinBuffer, temp1MinCount, lastTemp1MinRead, temperatureBuffer, temperatureIndex, lastTemperatureUpdate, MAX_24H_POINTS);
            aggregateSensor("Hum", hum1MinBuffer, hum1MinCount, lastHum1MinRead, humidityBuffer, humidityIndex, lastHumidityUpdate, MAX_24H_POINTS);
            aggregateSensor("Light", light1MinBuffer, light1MinCount, lastLight1MinRead, lightBuffer, lightIndex, lastLightUpdate, MAX_24H_POINTS);
            aggregateSensor("CO2", co21MinBuffer, co21MinCount, lastCO21MinRead, co2Buffer, co2Index, lastCO2Update, MAX_24H_POINTS);
            aggregateSensor("Soil", soil1MinBuffer, soil1MinCount, lastSoil1MinRead, soilMoistureBuffer, soilMoistureIndex, lastSoilMoistureUpdate, MAX_24H_POINTS);
        }
        vTaskDelay(xDelay);
    }
}

