#include <Arduino.h>
#include <LovyanGFX.hpp>
#include <math.h>
#include <TinyGPSPlus.h>
#include <SPIFFS.h>
#include <FS.h>
#include <WiFi.h>
#include <HTTPClient.h>
#include <ArduinoJson.h>
#include "time.h"

/* Bmorceli launchers version should be 2.6.10
    the reason behind is that version of launcher stroring wifi credentials on nvs at UTF8 format and the recent
    update were storing kind of hash which is i havent learned yet hot to imply but soon will be updated
*/

// ============================================
// DISPLAY CONFIGURATION FOR ESP32-2432S028
// ============================================
class LGFX : public lgfx::LGFX_Device {
    lgfx::Panel_ST7789 _panel_instance;
    lgfx::Bus_SPI _bus_instance;
    lgfx::Light_PWM _light_instance;

public:
    LGFX(void) {
        { // SPI Bus Configuration
            auto cfg = _bus_instance.config();
            cfg.spi_host = SPI2_HOST;
            cfg.spi_mode = 0;
            cfg.freq_write = 80000000;
            cfg.pin_sclk = 14;
            cfg.pin_mosi = 13;
            cfg.pin_miso = 12;
            cfg.pin_dc   = 2;
            _bus_instance.config(cfg);
            _panel_instance.setBus(&_bus_instance);
        }

        { // Panel Configuration
            auto cfg = _panel_instance.config();
            cfg.pin_cs   = 15;
            cfg.pin_rst  = -1;
            cfg.pin_busy = -1;
            cfg.memory_width  = 240;
            cfg.memory_height = 320;
            cfg.panel_width   = 240;
            cfg.panel_height  = 320;
            cfg.offset_x = 0;
            cfg.offset_y = 0;
            cfg.invert = true;
            _panel_instance.config(cfg);
        }

        { // Backlight Configuration
            auto cfg = _light_instance.config();
            cfg.pin_bl = 21;
            cfg.invert = false;
            cfg.freq   = 88200;
            cfg.pwm_channel = 7;
            _light_instance.config(cfg);
            _panel_instance.setLight(&_light_instance);
        }

        setPanel(&_panel_instance);
    }
};

LGFX tft;

// ============================================
// CONFIGURATION
// ============================================
#define GPS_BAUD 57600
#define NMEA_LINES_DISPLAY 1
#define REFRESH_MS_ALT 1000
#define REFRESH_MS_SPEED 10
#define CPU_FREQ_HIGH 240
#define CPU_FREQ_LOW 80
#define SUPER_HOT_TIMEOUT_MS 300000
#define HOT_TIMEOUT_MS 300000
#define STOPWATCH_DISPLAY_DURATION 180000
#define AGPS_START_DELAY_MS 3000

// ============================================
// FAST NTP CONFIG
// ============================================
#define FAST_NTP_TIMEOUT_MS 500
#define FAST_NTP_INTERVAL 500
#define NTP_BURST_INTERVAL 100

// ============================================
// SPEED SMOOTHING CONFIGURATION
// ============================================
#define SPEED_SMOOTHING_STIFFNESS 0.11f
#define SPEED_SMOOTHING_DAMPING 0.60f
#define SPEED_MAX_ACCELERATION 12.0f
#define SPEED_KALMAN_Q 0.05f
#define SPEED_KALMAN_R 0.3f
#define SPEED_MIN_VALUE 0.0f
#define SPEED_MAX_VALUE 140.0f

// ============================================
// DISPLAY CONFIGURATION
// ============================================
#define SPEED_DISPLAY_DECIMALS 0
#define NEEDLE_UPDATE_RATE 5

// ============================================
// COLORS
// ============================================
#define COLOR_BG 0xFFFF
#define COLOR_GREEN 0x07E0
#define COLOR_RED 0xF800
#define COLOR_CYAN 0x07FF
#define COLOR_YELLOW 0xFFE0
#define COLOR_WHITE 0x0000
#define COLOR_GREY 0x8410
#define COLOR_MAGENTA 0xF81F
#define COLOR_ORANGE 0xFD20
#define COLOR_BLUE 0x001F
#define TFT_BLACK 0x0000
#define TFT_RED 0xF800

// ============================================
// PINS & OBJECTS
// ============================================
#define GPS_TX 22
#define GPS_RX 27
#define LED_PIN 26
#define TEMP_PIN 34
#define GPS_TEMP_PIN 32
#define BAT_PIN_ADC 35
#define GPS_PIN_ADC 36

TinyGPSPlus gps;
HardwareSerial GPSUART(2);

// ============================================
// SENSOR CONSTANTS
// ============================================
#define SERIES_RESISTOR 10000.0
#define NOMINAL_RESISTANCE 10000.0
#define NOMINAL_TEMPERATURE 25.0
#define B_COEFFICIENT 3950.0
#define SAMPLES 10
#define ADC_MAX 4095.0
#define VREF 3.3
#define R_TOP 480000.0
#define R_BOT 480000.0
#define GPS_VOLTAGE_OFFSET 0.5

// ============================================
// BUFFERS
// ============================================
#define MAX_BUFFER_SIZE 256
#define MAX_STATUS_SIZE 64

char nmeaBuffer[MAX_BUFFER_SIZE];
char wifiStatusBuffer[MAX_STATUS_SIZE];
char agpsStatusBuffer[MAX_STATUS_SIZE];
char ntpStatusBuffer[MAX_STATUS_SIZE];
char locStatusBuffer[MAX_STATUS_SIZE];
char nmeaDisplayBuffer[NMEA_LINES_DISPLAY][MAX_BUFFER_SIZE];
char lastDisplayedNMEA[MAX_BUFFER_SIZE];

// ============================================
// KALMAN FILTER CLASS
// ============================================
class KalmanFilter {
public:
  float Q, R, P, K, X;

  KalmanFilter(float q = 0.001f, float r = 0.01f) : Q(q), R(r), P(1.0f), K(0.0f), X(0.0f) {}

  float update(float measurement) {
    P = P + Q;
    K = P / (P + R);
    X = X + K * (measurement - X);
    P = (1.0f - K) * P;
    return X;
  }

  void reset() { 
    P = 1.0f; 
    K = 0.0f; 
    X = 0.0f; 
  }
};

// ============================================
// SPEED SMOOTHING CLASS WITH KALMAN FILTER
// ============================================
class SpeedSmoother {
private:
  float smoothSpeed;
  float needleVel;
  float topSpeed;
  float smoothTopSpeed;
  KalmanFilter kalman;
  unsigned long lastUpdate;

public:
  SpeedSmoother() : smoothSpeed(0.0f), needleVel(0.0f), 
                     topSpeed(0.0f), smoothTopSpeed(0.0f),
                     kalman(SPEED_KALMAN_Q, SPEED_KALMAN_R),
                     lastUpdate(0) {}

  float update(float rawSpeed, unsigned long now) {
    float dt = (now - lastUpdate) / 1000.0f;
    if (dt < 0.001f) dt = 0.001f;
    if (dt > 0.1f) dt = 0.1f;
    lastUpdate = now;

    rawSpeed = constrain(rawSpeed, SPEED_MIN_VALUE, SPEED_MAX_VALUE);

    float kalmanSpeed = kalman.update(rawSpeed);
    kalmanSpeed = constrain(kalmanSpeed, SPEED_MIN_VALUE, SPEED_MAX_VALUE);

    float stiffness = SPEED_SMOOTHING_STIFFNESS;
    float damping = SPEED_SMOOTHING_DAMPING;
    float accel = (kalmanSpeed - smoothSpeed) * stiffness;
    needleVel = needleVel * damping + accel;
    smoothSpeed += needleVel;

    float maxChange = SPEED_MAX_ACCELERATION * dt;
    float change = smoothSpeed - kalmanSpeed;
    if (abs(change) > maxChange) {
      smoothSpeed = kalmanSpeed + (change < 0 ? -maxChange : maxChange);
    }

    smoothSpeed = constrain(smoothSpeed, SPEED_MIN_VALUE, SPEED_MAX_VALUE);

    if (smoothSpeed > topSpeed) {
      topSpeed = smoothSpeed;
    }
    smoothTopSpeed += 0.2f * (topSpeed - smoothTopSpeed);
    if (abs(smoothTopSpeed - topSpeed) < 0.1f) {
      smoothTopSpeed = topSpeed;
    }

    return smoothSpeed;
  }

  float getTopSpeed() const { return smoothTopSpeed; }
  float getRawTopSpeed() const { return topSpeed; }
  void resetTopSpeed() { 
    topSpeed = 0.0f; 
    smoothTopSpeed = 0.0f; 
  }
  void reset() {
    smoothSpeed = 0.0f;
    needleVel = 0.0f;
    kalman.reset();
  }
};

// ============================================
// GLOBAL VARIABLES
// ============================================
SpeedSmoother speedSmoother;
float displaySpeed = 0.0f;
float lastDisplaySpeed = -1.0f;
float needleAngle = -999.0f;
float lastNeedleAngle = -999.0f;
bool gpsHasFix = false;
bool agpsCompleted = false;
bool wifiAvailable = false;
unsigned long lastRefreshSpeed = 0;
unsigned long lastRefreshAlt = 0;
unsigned long lastTopUpdate = 0;

enum AGPSState {
  AGPS_IDLE,
  AGPS_SUPER_HOT,
  AGPS_HOT,
  AGPS_WARM,
  AGPS_FIXED,
  AGPS_ALREADY_FIXED,
  AGPS_OFFLINE_MODE
};
AGPSState agpsState = AGPS_IDLE;
bool usingCachedAGPS = false;

char cacheStatusText[16] = "0";
uint16_t cacheStatusColor = COLOR_YELLOW;
char wifiStatusText[MAX_STATUS_SIZE] = "WiFi: N/A";
char agpsStatusText[MAX_STATUS_SIZE] = "AGPS: N/A";
char ntpServerStatus[MAX_STATUS_SIZE] = "NTP: Idle";
char locStatusText[MAX_STATUS_SIZE] = "LOC: N/A";
uint16_t wifiStatusColor = COLOR_YELLOW;
uint16_t agpsStatusColor = COLOR_YELLOW;
uint16_t ntpStatusColor = COLOR_WHITE;
uint16_t locStatusColor = COLOR_YELLOW;

float avgCpuTempC = 0.0;
float avgGpsTempC = 0.0;
float avgCpuVolt = 0.0;
float avgGpsVolt = 0.0;

unsigned long stopwatchStartTime = 0;
unsigned long stopwatchEndTime = 0;
bool stopwatchRunning = false;
bool stopwatchCompleted = false;
unsigned long stopwatchHideTime = 0;

const char* ntpServer = "time.google.com";
unsigned long lastNTPAttempt = 0;
time_t lastSyncedTime = 0;
bool ntpSuccess = false;
int ntpAttempts = 0;

float approxLat = 0.0;
float approxLon = 0.0;
bool hasApproxLocation = false;
char approxCity[32] = "Unknown";
char approxRegion[32] = "Unknown";

int nmeaGPSPos = 0;
char nmeaBuilder[MAX_BUFFER_SIZE];
int nmeaBuilderLen = 0;
unsigned long lastNmeaDisplayUpdate = 0;
unsigned long lastStatusDisplayUpdate = 0;
bool nmeaDataChanged = false;
int nmeaSentenceCount = 0;
bool nmeaFastMode = false;

#define MAX_SATELLITES 64
struct SatelliteData {
  int prn;
  float elevation;
  float azimuth;
  int snr;
  bool used;
  float priority;
  unsigned long lastUsed;
};
SatelliteData satelliteCache[MAX_SATELLITES];
SatelliteData tempCache[MAX_SATELLITES];
int satelliteCount = 0;
unsigned long lastSatelliteSave = 0;

// ============================================
// GAUGE ARC CACHE FOR OPTIMIZED REDRAW
// ============================================
#define ARC_SEGMENT_COUNT 180
struct ArcSegment {
  uint16_t color;
  bool dirty;
};
ArcSegment arcCache[ARC_SEGMENT_COUNT];
bool arcCacheInitialized = false;
int cx = 120, cy = 220, outerR = 90, innerR = 70;
int startAngle = 180, endAngle = 360;

// ============================================
// DMA BUFFER
// ============================================
#define DMA_BUFFER_SIZE 4096
uint16_t dmaBuffer[DMA_BUFFER_SIZE];
bool dmaInProgress = false;
portMUX_TYPE dmaMux = portMUX_INITIALIZER_UNLOCKED;

// ============================================
// FORWARD DECLARATIONS
// ============================================
void drawTopNotch();
void drawStats();
void drawNMEALog();
void readGPS();
void updateLED();
void manageCpuFrequency();
void updateSensors();
void updateStatusDisplay();
float readTempC(int pin);
float readVoltage(int pin);
float readGPSVoltage();
void setupGPSBaudRate();
void agpsTaskAdaptive(void *parameter);
void performOfflineAGPS();
void fetchApproximateLocation();
void injectLocationToGPS(float lat, float lon, bool urgent = false);
void injectTimeToGPS(time_t utcEpoch, bool urgent = false);
bool syncFastNTP_nonblocking();
void saveLastFixData(float lat, float lon, float alt, float course, time_t timestamp);
bool loadLastFixData(float &lat, float &lon, float &alt, float &course, time_t &timestamp);
void loadSatelliteData();
void saveSatelliteDataAtomic();
void injectSatelliteData();
void sortSatellites();
void mergeSortSatellites(int left, int right);
void mergeSatellites(int left, int mid, int right);
void collectSatelliteData(const char* nmea);
void startStopwatch();
void stopStopwatch();
void formatStopwatch(unsigned long elapsedMs, char* buffer, size_t bufferSize);
void pushGPSNMEA(const char* line);
void bootAnimation();
void initializeArcCache();
void drawArcSegment(int index, bool force);
void redrawNeedleArea(float oldAngle, float newAngle);
void dmaDrawLine(int x1, int y1, int x2, int y2, uint16_t color);
void dmaFillCircle(int x, int y, int r, uint16_t color);
void dmaFillTriangle(int x1, int y1, int x2, int y2, int x3, int y3, uint16_t color);

// ============================================
// DMA DRAW FUNCTIONS
// ============================================
void dmaDrawLine(int x1, int y1, int x2, int y2, uint16_t color) {
  portENTER_CRITICAL(&dmaMux);
  if (dmaInProgress) {
    portEXIT_CRITICAL(&dmaMux);
    tft.drawLine(x1, y1, x2, y2, color);
    return;
  }
  dmaInProgress = true;
  portEXIT_CRITICAL(&dmaMux);
  
  tft.drawLine(x1, y1, x2, y2, color);
  
  portENTER_CRITICAL(&dmaMux);
  dmaInProgress = false;
  portEXIT_CRITICAL(&dmaMux);
}

void dmaFillCircle(int x, int y, int r, uint16_t color) {
  portENTER_CRITICAL(&dmaMux);
  if (dmaInProgress) {
    portEXIT_CRITICAL(&dmaMux);
    tft.fillCircle(x, y, r, color);
    return;
  }
  dmaInProgress = true;
  portEXIT_CRITICAL(&dmaMux);
  
  tft.fillCircle(x, y, r, color);
  
  portENTER_CRITICAL(&dmaMux);
  dmaInProgress = false;
  portEXIT_CRITICAL(&dmaMux);
}

void dmaFillTriangle(int x1, int y1, int x2, int y2, int x3, int y3, uint16_t color) {
  portENTER_CRITICAL(&dmaMux);
  if (dmaInProgress) {
    portEXIT_CRITICAL(&dmaMux);
    tft.fillTriangle(x1, y1, x2, y2, x3, y3, color);
    return;
  }
  dmaInProgress = true;
  portEXIT_CRITICAL(&dmaMux);
  
  tft.fillTriangle(x1, y1, x2, y2, x3, y3, color);
  
  portENTER_CRITICAL(&dmaMux);
  dmaInProgress = false;
  portEXIT_CRITICAL(&dmaMux);
}

// ============================================
// GPS BAUD RATE SETUP
// ============================================
void setupGPSBaudRate() {
  GPSUART.begin(9600, SERIAL_8N1, GPS_RX, GPS_TX);
  delay(500);

  GPSUART.print("$PUBX,41,1,0001,0003,38400,0*18\r\n");
  delay(300);

  byte setBaud38400[] = {
    0xB5, 0x62, 0x06, 0x00, 0x14, 0x00,
    0x01, 0x00, 0x00, 0x00, 0xC0, 0x08,
    0x00, 0x00, 0x00, 0x96, 0x01, 0x00,
    0x01, 0x00, 0x00, 0x00, 0x00, 0x00
  };

  uint8_t ck_a = 0, ck_b = 0;
  for (int i = 2; i < sizeof(setBaud38400); i++) {
    ck_a += setBaud38400[i];
    ck_b += ck_a;
  }
  GPSUART.write(setBaud38400, sizeof(setBaud38400));
  GPSUART.write(ck_a);
  GPSUART.write(ck_b);
  delay(300);

  GPSUART.end();
  GPSUART.begin(38400, SERIAL_8N1, GPS_RX, GPS_TX);
  delay(500);

  GPSUART.print("$PUBX,00*33\r\n");
  delay(100);

  while (GPSUART.available()) {
    GPSUART.read();
  }
}

// ============================================
// TIME INJECTION
// ============================================
void injectTimeToGPS(time_t utcEpoch, bool urgent) {
  if (utcEpoch == 0) return;
  struct tm tmstruct;
  gmtime_r(&utcEpoch, &tmstruct);

  int injections = urgent ? 6 : 4;

  for (int i = 0; i < injections; i++) {
    char nmea1[128], nmea2[128];
    snprintf(nmea1, sizeof(nmea1), "$GNZDA,%02d%02d%02d.00,%02d,%02d,%04d,00,00",
             tmstruct.tm_hour, tmstruct.tm_min, tmstruct.tm_sec,
             tmstruct.tm_mday, tmstruct.tm_mon + 1, tmstruct.tm_year + 1900);
    uint8_t cs1 = 0;
    for (const char *p = nmea1 + 1; *p; ++p) cs1 ^= *p;
    GPSUART.printf("%s*%02X\r\n", nmea1, cs1);

    snprintf(nmea2, sizeof(nmea2), "$GPZDA,%02d%02d%02d.00,%02d,%02d,%04d,00,00",
             tmstruct.tm_hour, tmstruct.tm_min, tmstruct.tm_sec,
             tmstruct.tm_mday, tmstruct.tm_mon + 1, tmstruct.tm_year + 1900);
    uint8_t cs2 = 0;
    for (const char *p = nmea2 + 1; *p; ++p) cs2 ^= *p;
    GPSUART.printf("%s*%02X\r\n", nmea2, cs2);

    delay(urgent ? 3 : 5);
  }
}

// ============================================
// LOCATION INJECTION
// ============================================
void injectLocationToGPS(float lat, float lon, bool urgent) {
  if (lat == 0.0 || lon == 0.0) return;
  char latStr[16], lonStr[16];
  float latDeg = abs(lat);
  float lonDeg = abs(lon);
  snprintf(latStr, sizeof(latStr), "%02d%07.4f", (int)latDeg, (latDeg - (int)latDeg) * 60.0);
  snprintf(lonStr, sizeof(lonStr), "%03d%07.4f", (int)lonDeg, (lonDeg - (int)lonDeg) * 60.0);
  char latDir = (lat >= 0) ? 'N' : 'S';
  char lonDir = (lon >= 0) ? 'E' : 'W';

  char nmea[200];
  snprintf(nmea, sizeof(nmea), "$GPGGA,000000.00,%s,%c,%s,%c,1,12,0.8,0.0,M,0.0,M,,0000",
           latStr, latDir, lonStr, lonDir);
  uint8_t cs = 0;
  for (const char *p = nmea + 1; *p; ++p) cs ^= *p;

  int injections = urgent ? 5 : 3;
  for (int i = 0; i < injections; i++) {
    GPSUART.printf("%s*%02X\r\n", nmea, cs);
    delay(urgent ? 2 : 5);
  }
}

// ============================================
// SATELLITE DATA MANAGEMENT
// ============================================
void mergeSatellites(int left, int mid, int right) {
  int i = left, j = mid + 1, k = left;
  while (i <= mid && j <= right) {
    if (satelliteCache[i].priority >= satelliteCache[j].priority) {
      tempCache[k] = satelliteCache[i];
      i++;
    } else {
      tempCache[k] = satelliteCache[j];
      j++;
    }
    k++;
  }
  while (i <= mid) { tempCache[k] = satelliteCache[i]; i++; k++; }
  while (j <= right) { tempCache[k] = satelliteCache[j]; j++; k++; }
  for (int idx = left; idx <= right; idx++) {
    satelliteCache[idx] = tempCache[idx];
  }
}

void mergeSortSatellites(int left, int right) {
  if (left < right) {
    int mid = left + (right - left) / 2;
    mergeSortSatellites(left, mid);
    mergeSortSatellites(mid + 1, right);
    mergeSatellites(left, mid, right);
  }
}

void sortSatellites() {
  if (satelliteCount <= 1) return;
  unsigned long now = millis();

  for (int i = 0; i < satelliteCount; i++) {
    satelliteCache[i].priority = satelliteCache[i].snr;
    if (satelliteCache[i].used) {
      satelliteCache[i].priority += 10.0f;
      satelliteCache[i].lastUsed = now;
    }
    if (now - satelliteCache[i].lastUsed < 30000) {
      satelliteCache[i].priority += 3.0f;
    }
    if (satelliteCache[i].elevation > 30.0) {
      satelliteCache[i].priority += 5.0f;
    } else if (satelliteCache[i].elevation > 15.0) {
      satelliteCache[i].priority += 2.0f;
    }
  }

  if (satelliteCount <= 16) {
    for (int i = 1; i < satelliteCount; i++) {
      SatelliteData key = satelliteCache[i];
      int j = i - 1;
      while (j >= 0 && satelliteCache[j].priority < key.priority) {
        satelliteCache[j + 1] = satelliteCache[j];
        j--;
      }
      satelliteCache[j + 1] = key;
    }
    return;
  }
  mergeSortSatellites(0, satelliteCount - 1);
}

void collectSatelliteData(const char* nmea) {
  if (!nmea) return;

  if (strncmp(nmea, "$GPGSV", 6) == 0 || strncmp(nmea, "$GLGSV", 6) == 0 || 
      strncmp(nmea, "$GAGSV", 6) == 0 || strncmp(nmea, "$GBGSV", 6) == 0) {

    char tempBuffer[MAX_BUFFER_SIZE];
    strncpy(tempBuffer, nmea, MAX_BUFFER_SIZE - 1);
    tempBuffer[MAX_BUFFER_SIZE - 1] = '\0';

    char* tokens[20];
    int fieldIndex = 0;
    char* token = strtok(tempBuffer, ",");
    while (token && fieldIndex < 20) {
      tokens[fieldIndex++] = token;
      token = strtok(NULL, ",");
    }

    if (fieldIndex >= 4) {
      int numSats = atoi(tokens[3]);
      int satIndex = 4;
      for (int i = 0; i < numSats && i < 4; i++) {
        if (satIndex + 3 < fieldIndex) {
          SatelliteData sat;
          sat.prn = atoi(tokens[satIndex]);
          sat.elevation = atof(tokens[satIndex + 1]);
          sat.azimuth = atof(tokens[satIndex + 2]);
          sat.snr = atoi(tokens[satIndex + 3]);
          sat.used = false;
          sat.priority = sat.snr;
          sat.lastUsed = 0;

          bool found = false;
          for (int j = 0; j < satelliteCount; j++) {
            if (satelliteCache[j].prn == sat.prn) {
              satelliteCache[j] = sat;
              found = true;
              break;
            }
          }
          if (!found && satelliteCount < MAX_SATELLITES) {
            satelliteCache[satelliteCount++] = sat;
          }
          satIndex += 4;
        }
      }
    }
  }

  if (strncmp(nmea, "$GPGSA", 6) == 0 || strncmp(nmea, "$GLGSA", 6) == 0 || 
      strncmp(nmea, "$GAGSA", 6) == 0 || strncmp(nmea, "$GBGSA", 6) == 0) {

    char tempBuffer[MAX_BUFFER_SIZE];
    strncpy(tempBuffer, nmea, MAX_BUFFER_SIZE - 1);
    tempBuffer[MAX_BUFFER_SIZE - 1] = '\0';

    char* tokens[25];
    int fieldIndex = 0;
    char* token = strtok(tempBuffer, ",");
    while (token && fieldIndex < 25) {
      tokens[fieldIndex++] = token;
      token = strtok(NULL, ",");
    }

    if (fieldIndex >= 15) {
      for (int i = 3; i < 15 && i < fieldIndex; i++) {
        int sv = atoi(tokens[i]);
        if (sv > 0) {
          for (int j = 0; j < satelliteCount; j++) {
            if (satelliteCache[j].prn == sv) {
              satelliteCache[j].used = true;
              satelliteCache[j].priority += 10.0f;
              satelliteCache[j].lastUsed = millis();
              break;
            }
          }
        }
      }
    }
  }

  if (gps.location.isValid() && gps.satellites.value() >= 4) {
    unsigned long now = millis();
    if (now - lastSatelliteSave > 30000) {
      lastSatelliteSave = now;
      sortSatellites();
      saveSatelliteDataAtomic();
      snprintf(cacheStatusText, sizeof(cacheStatusText), "%d", satelliteCount);
      cacheStatusColor = COLOR_MAGENTA;
    }
  }
}

void saveSatelliteDataAtomic() {
  if (satelliteCount == 0) return;

  if (SPIFFS.totalBytes() - SPIFFS.usedBytes() < 4096) {
    snprintf(cacheStatusText, sizeof(cacheStatusText), "ERR: No Space");
    cacheStatusColor = COLOR_RED;
    return;
  }

  fs::File temp = SPIFFS.open("/sat_data.tmp", "w");
  if (!temp) {
    snprintf(cacheStatusText, sizeof(cacheStatusText), "ERR: Write Fail");
    cacheStatusColor = COLOR_RED;
    return;
  }

  for (int i = 0; i < satelliteCount; i++) {
    temp.printf("SAT,%d,%.1f,%.1f,%d,%d,%.1f\n",
                satelliteCache[i].prn,
                satelliteCache[i].elevation,
                satelliteCache[i].azimuth,
                satelliteCache[i].snr,
                satelliteCache[i].used ? 1 : 0,
                satelliteCache[i].priority);
  }
  temp.close();

  if (SPIFFS.exists("/sat_data.txt")) {
    SPIFFS.remove("/sat_data.bak");
    SPIFFS.rename("/sat_data.txt", "/sat_data.bak");
  }
  SPIFFS.rename("/sat_data.tmp", "/sat_data.txt");
}

void loadSatelliteData() {
  if (!SPIFFS.exists("/sat_data.txt")) return;
  fs::File file = SPIFFS.open("/sat_data.txt", "r");
  if (!file) return;

  satelliteCount = 0;
  while (file.available() && satelliteCount < MAX_SATELLITES) {
    String line = file.readStringUntil('\n');
    line.trim();
    if (line.startsWith("SAT,")) {
      int prn, snr, used;
      float elevation, azimuth, priority;
      if (sscanf(line.c_str(), "SAT,%d,%f,%f,%d,%d,%f",
                 &prn, &elevation, &azimuth, &snr, &used, &priority) == 6) {
        satelliteCache[satelliteCount].prn = prn;
        satelliteCache[satelliteCount].elevation = elevation;
        satelliteCache[satelliteCount].azimuth = azimuth;
        satelliteCache[satelliteCount].snr = snr;
        satelliteCache[satelliteCount].used = (used == 1);
        satelliteCache[satelliteCount].priority = priority;
        satelliteCache[satelliteCount].lastUsed = 0;
        satelliteCount++;
      }
    }
  }
  file.close();

  if (satelliteCount > 0) {
    sortSatellites();
    snprintf(cacheStatusText, sizeof(cacheStatusText), "%d", satelliteCount);
    cacheStatusColor = COLOR_GREEN;
  }
}

void injectSatelliteData() {
  if (satelliteCount == 0) {
    loadSatelliteData();
    if (satelliteCount == 0) {
      snprintf(cacheStatusText, sizeof(cacheStatusText), "0");
      cacheStatusColor = COLOR_RED;
      return;
    }
  }

  sortSatellites();
  int injectCount = min(satelliteCount, 10);
  for (int i = 0; i < injectCount; i++) {
    char buf[128];
    snprintf(buf, sizeof(buf), "$PUBX,04,%02d,0,0.0,0.0,0.0,0.0,0,0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0,0,0,0,0,0*",
             satelliteCache[i].prn);
    uint8_t cs = 0;
    for (const char *p = buf + 1; *p; ++p) cs ^= *p;
    GPSUART.printf("%s%02X\r\n", buf, cs);
    delay(5);
  }
  snprintf(cacheStatusText, sizeof(cacheStatusText), "%d", injectCount);
  cacheStatusColor = COLOR_CYAN;
}

// ============================================
// SENSOR FUNCTIONS
// ============================================
float readTempC(int pin) {
  int adcValue = analogRead(pin);
  if (adcValue <= 0) adcValue = 1;
  float resistance = SERIES_RESISTOR / ((4095.0 / adcValue) - 1.35);
  float steinhart = log(resistance / NOMINAL_RESISTANCE) / B_COEFFICIENT;
  steinhart += 1.35 / (NOMINAL_TEMPERATURE + 273.15);
  steinhart = 1.35 / steinhart - 273.15;
  return steinhart + 1.35;
}

float readVoltage(int pin) {
  long sum = 0;
  for (int i = 0; i < SAMPLES; i++) {
    sum += analogRead(pin);
    delay(1);
  }
  float raw = sum / (float)SAMPLES;
  float v_adc = (raw / ADC_MAX) * VREF;
  return v_adc * ((R_TOP + R_BOT) / R_BOT) * (4.33 / 3.64) - 0.047;
}

float readGPSVoltage() {
  long sum = 0;
  for (int i = 0; i < SAMPLES; i++) {
    sum += analogRead(GPS_PIN_ADC);
    delay(1);
  }
  float raw = sum / (float)SAMPLES;
  float v_adc = (raw / ADC_MAX) * VREF;
  float voltage = v_adc * ((R_TOP + R_BOT) / R_BOT) * (4.33 / 3.64) - 0.047;
  voltage = voltage - GPS_VOLTAGE_OFFSET;
  if (voltage < 0.0) voltage = 0.0;
  if (voltage > 5.0) voltage = 5.0;
  return voltage;
}

void updateSensors() {
  static float cpuTempFiltered = 0, gpsTempFiltered = 0;
  static float cpuVoltFiltered = 0, gpsVoltFiltered = 0;
  static int sampleCount = 0;
  const float ALPHA = 0.25;

  float cpuTemp = readTempC(TEMP_PIN);
  float gpsTemp = readTempC(GPS_TEMP_PIN);
  float cpuVolt = readVoltage(BAT_PIN_ADC);
  float gpsVolt = readGPSVoltage();

  cpuVolt = cpuVolt - 0.2f;
  if (cpuVolt < 0.0) cpuVolt = 0.0;

  if (sampleCount == 0) {
    cpuTempFiltered = cpuTemp;
    gpsTempFiltered = gpsTemp;
    cpuVoltFiltered = cpuVolt;
    gpsVoltFiltered = gpsVolt;
  } else {
    cpuTempFiltered = cpuTempFiltered * (1 - ALPHA) + cpuTemp * ALPHA;
    gpsTempFiltered = gpsTempFiltered * (1 - ALPHA) + gpsTemp * ALPHA;
    cpuVoltFiltered = cpuVoltFiltered * (1 - ALPHA) + cpuVolt * ALPHA;
    gpsVoltFiltered = gpsVoltFiltered * (1 - ALPHA) + gpsVolt * ALPHA;
  }
  sampleCount++;

  avgCpuTempC = cpuTempFiltered;
  avgGpsTempC = gpsTempFiltered;
  avgCpuVolt = cpuVoltFiltered;
  avgGpsVolt = gpsVoltFiltered;
}

// ============================================
// CPU FREQUENCY MANAGEMENT
// ============================================
void manageCpuFrequency() {
  static unsigned long lastCpuCheck = 0;
  unsigned long now = millis();
  if (now - lastCpuCheck < 2000) return;
  lastCpuCheck = now;

  bool hasFix = (gps.location.isValid() && gps.satellites.value() >= 4);
  static bool cpuFrequencyReduced = false;

  if (hasFix && !cpuFrequencyReduced) {
    setCpuFrequencyMhz(CPU_FREQ_LOW);
    cpuFrequencyReduced = true;
  } else if (!hasFix && cpuFrequencyReduced) {
    setCpuFrequencyMhz(CPU_FREQ_HIGH);
    cpuFrequencyReduced = false;
  }
}

// ============================================
// NTP SYNC
// ============================================
bool syncFastNTP_nonblocking() {
  if (WiFi.status() != WL_CONNECTED) return false;

  enum NTPState { NTP_IDLE, NTP_WAITING, NTP_SYNCED };
  static NTPState ntpState = NTP_IDLE;
  static unsigned long ntpStartTime = 0;
  static KalmanFilter timeKalman(0.001f, 0.01f);
  static bool kalmanInitialized = false;
  static float filteredTimeOffset = 0.0f;

  unsigned long now = millis();

  if (ntpState == NTP_IDLE || ntpState == NTP_WAITING) {
    if (ntpState == NTP_IDLE) {
      configTime(0, 0, ntpServer);
      ntpState = NTP_WAITING;
      ntpStartTime = now;
      snprintf(ntpServerStatus, sizeof(ntpServerStatus), "NTP: Fast sync...");
      ntpStatusColor = COLOR_YELLOW;
      return false;
    }

    struct tm timeinfo;
    if (getLocalTime(&timeinfo, 0)) {
      time_t nowt;
      time(&nowt);
      if (nowt > 1577836800) {
        unsigned long responseTime = now - ntpStartTime;

        if (kalmanInitialized) {
          float offset = (float)(nowt - lastSyncedTime);
          filteredTimeOffset = timeKalman.update(offset);
          time_t adjustedTime = nowt + (time_t)filteredTimeOffset;
          lastSyncedTime = adjustedTime;

          snprintf(ntpServerStatus, sizeof(ntpServerStatus), 
                   "NTP: Kalman %lums (Δ%.3fs)", responseTime, filteredTimeOffset);
          ntpStatusColor = COLOR_MAGENTA;
        } else {
          lastSyncedTime = nowt;
          filteredTimeOffset = 0.0f;
          kalmanInitialized = true;
          snprintf(ntpServerStatus, sizeof(ntpServerStatus), "NTP: Fast %lums", responseTime);
          ntpStatusColor = COLOR_GREEN;
        }

        ntpSuccess = true;
        ntpAttempts = 0;
        ntpState = NTP_SYNCED;

        injectTimeToGPS(lastSyncedTime, true);
        delay(5);
        injectTimeToGPS(lastSyncedTime, true);

        return true;
      }
    }

    unsigned long timeout = FAST_NTP_TIMEOUT_MS * 2;
    if (now - ntpStartTime > timeout) {
      ntpAttempts++;
      ntpState = NTP_IDLE;

      if (ntpAttempts >= 5) {
        snprintf(ntpServerStatus, sizeof(ntpServerStatus), "NTP: Timeout (%d)", ntpAttempts);
        ntpStatusColor = COLOR_RED;
      } else {
        snprintf(ntpServerStatus, sizeof(ntpServerStatus), "NTP: Retry %d", ntpAttempts);
        ntpStatusColor = COLOR_YELLOW;
      }

      if (lastSyncedTime != 0) {
        injectTimeToGPS(lastSyncedTime, true);
        snprintf(ntpServerStatus, sizeof(ntpServerStatus), "NTP: Using cached");
        ntpStatusColor = COLOR_ORANGE;
        return true;
      }

      return false;
    }
  }
  return false;
}

// ============================================
// LOCATION FETCH
// ============================================
void fetchApproximateLocation() {
  if (WiFi.status() != WL_CONNECTED) {
    float lat, lon, alt, course;
    time_t timestamp;
    if (loadLastFixData(lat, lon, alt, course, timestamp) && !hasApproxLocation) {
      approxLat = lat;
      approxLon = lon;
      hasApproxLocation = true;
      snprintf(locStatusText, sizeof(locStatusText), 
               "LOC: Cached (%.0fm)", alt);
      locStatusColor = COLOR_GREEN;
      injectLocationToGPS(approxLat, approxLon, true);
      return;
    }
    snprintf(locStatusText, sizeof(locStatusText), "LOC: No WiFi/Cache");
    locStatusColor = COLOR_YELLOW;
    return;
  }

  HTTPClient http;
  http.begin("http://ip-api.com/json/");
  http.setTimeout(3000);
  unsigned long start = millis();
  int httpCode = http.GET();
  unsigned long responseTime = millis() - start;

  if (httpCode == HTTP_CODE_OK) {
    String payload = http.getString();
    http.end();
    DynamicJsonDocument doc(1024);
    if (!deserializeJson(doc, payload)) {
      String status = doc["status"];
      if (status == "success") {
        strncpy(approxCity, doc["city"] | "Unknown", sizeof(approxCity) - 1);
        strncpy(approxRegion, doc["regionName"] | "Unknown", sizeof(approxRegion) - 1);
        approxLat = doc["lat"] | 0.0f;
        approxLon = doc["lon"] | 0.0f;
        hasApproxLocation = true;

        saveLastFixData(approxLat, approxLon, 0.0, 0.0, lastSyncedTime);

        snprintf(locStatusText, sizeof(locStatusText), 
                 "LOC: %s, %s (%lums)", approxCity, approxRegion, responseTime);
        locStatusColor = COLOR_MAGENTA;
        injectLocationToGPS(approxLat, approxLon, true);
        delay(15);
        injectLocationToGPS(approxLat, approxLon, true);
      }
    }
  } else {
    http.end();
    float lat, lon, alt, course;
    time_t timestamp;
    if (loadLastFixData(lat, lon, alt, course, timestamp) && !hasApproxLocation) {
      approxLat = lat;
      approxLon = lon;
      hasApproxLocation = true;
      snprintf(locStatusText, sizeof(locStatusText), 
               "LOC: Cached (%.0fm)", alt);
      locStatusColor = COLOR_GREEN;
      injectLocationToGPS(approxLat, approxLon, true);
      return;
    }
    snprintf(locStatusText, sizeof(locStatusText), "LOC: HTTP %d", httpCode);
    locStatusColor = COLOR_RED;
  }
}

// ============================================
// LAST FIX DATA
// ============================================
void saveLastFixData(float lat, float lon, float alt, float course, time_t timestamp) {
  fs::File f = SPIFFS.open("/lastfix.txt", "w");
  if (f) {
    f.printf("%.6f,%.6f,%.1f,%.1f,%ld\n", lat, lon, alt, course, timestamp);
    f.close();
  }
}

bool loadLastFixData(float &lat, float &lon, float &alt, float &course, time_t &timestamp) {
  fs::File f = SPIFFS.open("/lastfix.txt", "r");
  if (!f) return false;
  String line = f.readStringUntil('\n');
  f.close();
  line.trim();

  int comma1 = line.indexOf(',');
  int comma2 = line.indexOf(',', comma1 + 1);
  int comma3 = line.indexOf(',', comma2 + 1);
  int comma4 = line.indexOf(',', comma3 + 1);

  if (comma1 == -1 || comma2 == -1 || comma3 == -1 || comma4 == -1) return false;

  lat = line.substring(0, comma1).toFloat();
  lon = line.substring(comma1 + 1, comma2).toFloat();
  alt = line.substring(comma2 + 1, comma3).toFloat();
  course = line.substring(comma3 + 1, comma4).toFloat();
  timestamp = line.substring(comma4 + 1).toInt();

  return true;
}

// ============================================
// NMEA FUNCTIONS
// ============================================
void pushGPSNMEA(const char* line) {
  if (!line || strlen(line) == 0) return;

  strncpy(nmeaDisplayBuffer[nmeaGPSPos], line, MAX_BUFFER_SIZE - 1);
  nmeaDisplayBuffer[nmeaGPSPos][MAX_BUFFER_SIZE - 1] = '\0';
  nmeaGPSPos = (nmeaGPSPos + 1) % NMEA_LINES_DISPLAY;

  nmeaSentenceCount++;
  if (nmeaSentenceCount > 3) {
    nmeaFastMode = true;
  }
  nmeaDataChanged = true;
}

void readGPS() {
  while (GPSUART.available()) {
    char c = (char)GPSUART.read();
    gps.encode(c);

    if (nmeaBuilderLen < MAX_BUFFER_SIZE - 1) {
      nmeaBuilder[nmeaBuilderLen++] = c;
    }

    if (c == '\n') {
      if (nmeaBuilderLen > 0) {
        nmeaBuilder[nmeaBuilderLen] = '\0';
        int len = nmeaBuilderLen;
        while (len > 0 && (nmeaBuilder[len-1] == '\n' || nmeaBuilder[len-1] == '\r')) {
          nmeaBuilder[--len] = '\0';
        }
        if (len > 0) {
          pushGPSNMEA(nmeaBuilder);
          collectSatelliteData(nmeaBuilder);
        }
      }
      nmeaBuilderLen = 0;
      nmeaBuilder[0] = '\0';
    }
  }
}

// ============================================
// STOPWATCH FUNCTIONS
// ============================================
void startStopwatch() {
  stopwatchStartTime = millis();
  stopwatchRunning = true;
  stopwatchCompleted = false;
  stopwatchEndTime = 0;
}

void stopStopwatch() {
  if (stopwatchRunning) {
    stopwatchEndTime = millis();
    stopwatchRunning = false;
    stopwatchCompleted = true;
    stopwatchHideTime = millis() + STOPWATCH_DISPLAY_DURATION;
  }
}

void formatStopwatch(unsigned long elapsedMs, char* buffer, size_t bufferSize) {
  unsigned long totalSeconds = elapsedMs / 1000;
  unsigned long minutes = totalSeconds / 60;
  unsigned long seconds = totalSeconds % 60;
  unsigned long millisRemainder = (elapsedMs % 1000) / 100;
  snprintf(buffer, bufferSize, "%02lu:%02lu.%lu", minutes, seconds, millisRemainder);
}

// ============================================
// PERFORM OFFLINE AGPS
// ============================================
void performOfflineAGPS() {
  usingCachedAGPS = true;
  agpsState = AGPS_OFFLINE_MODE;
  snprintf(agpsStatusText, sizeof(agpsStatusText), "Offline AGPS");
  agpsStatusColor = COLOR_BLUE;

  GPSUART.print("$PUBX,41,1,0001,0003,38400,0*18\r\n");
  delay(80);

  if (satelliteCount > 0) {
    sortSatellites();
    injectSatelliteData();
  } else {
    loadSatelliteData();
    if (satelliteCount > 0) {
      sortSatellites();
      injectSatelliteData();
    }
  }

  if (hasApproxLocation) {
    injectLocationToGPS(approxLat, approxLon, true);
    delay(15);
    injectLocationToGPS(approxLat, approxLon, true);
    delay(15);
    injectLocationToGPS(approxLat, approxLon, true);
  } else {
    float lat, lon, alt, course;
    time_t timestamp;
    if (loadLastFixData(lat, lon, alt, course, timestamp)) {
      approxLat = lat;
      approxLon = lon;
      hasApproxLocation = true;
      injectLocationToGPS(approxLat, approxLon, true);
      delay(15);
      injectLocationToGPS(approxLat, approxLon, true);
    }
  }

  if (lastSyncedTime != 0) {
    injectTimeToGPS(lastSyncedTime, true);
    delay(15);
    injectTimeToGPS(lastSyncedTime, true);
  } else {
    time_t compileTime = time(nullptr);
    if (compileTime > 1577836800) {
      injectTimeToGPS(compileTime, true);
      delay(15);
      injectTimeToGPS(compileTime, true);
    }
  }

  GPSUART.print("$PUBX,41,1,0001,0003,38400,0*18\r\n");
  delay(80);

  startStopwatch();
}

// ============================================
// AGPS TASK
// ============================================
void agpsTaskAdaptive(void *parameter) {
  unsigned long lastWiFiRetry = 0;
  unsigned long wifiRetryInterval = 5000;
  int wifiRetryAttempts = 0;
  wl_status_t lastWiFiState = WL_DISCONNECTED;
  bool gpsConfigured = false;
  bool agpsDelayActive = true;
  unsigned long agpsStartTime = millis();
  unsigned long fixAttemptStartTime = 0;
  bool superHotAttempted = false;
  bool hotAttempted = false;
  bool warmFallbackDone = false;

  WiFi.mode(WIFI_STA);
  WiFi.setAutoReconnect(true);
  WiFi.persistent(true);
  WiFi.begin();
  loadSatelliteData();

  for (;;) {
    wl_status_t current = WiFi.status();

    if (agpsDelayActive) {
      unsigned long elapsed = millis() - agpsStartTime;

      if (gps.location.isValid() && gps.satellites.value() >= 4) {
        gpsHasFix = true;
        agpsCompleted = true;
        agpsState = AGPS_ALREADY_FIXED;
        snprintf(agpsStatusText, sizeof(agpsStatusText), "Already Fixed!");
        agpsStatusColor = COLOR_GREEN;
        agpsDelayActive = false;
        gpsConfigured = true;
        continue;
      }

      if (elapsed >= AGPS_START_DELAY_MS) {
        agpsDelayActive = false;

        if (current == WL_CONNECTED && !gpsConfigured) {
          GPSUART.print("$PUBX,41,1,0001,0003,38400,0*18\r\n");
          delay(80);

          if (satelliteCount > 0) {
            sortSatellites();
            injectSatelliteData();
          }
          if (hasApproxLocation) {
            injectLocationToGPS(approxLat, approxLon, true);
            delay(15);
            injectLocationToGPS(approxLat, approxLon, true);
          }
          if (lastSyncedTime != 0) {
            injectTimeToGPS(lastSyncedTime, true);
          }

          agpsState = AGPS_SUPER_HOT;
          snprintf(agpsStatusText, sizeof(agpsStatusText), "Super Hot");
          agpsStatusColor = COLOR_MAGENTA;
          superHotAttempted = true;
          fixAttemptStartTime = millis();
          gpsConfigured = true;
          startStopwatch();
        } else if (!gpsConfigured && (hasApproxLocation || satelliteCount > 0 || lastSyncedTime != 0)) {
          performOfflineAGPS();
          gpsConfigured = true;
          fixAttemptStartTime = millis();
        } else if (!gpsConfigured) {
          GPSUART.print("$PUBX,41,1,0007,0003,38400,0*2C\r\n");
          delay(80);
          agpsState = AGPS_WARM;
          snprintf(agpsStatusText, sizeof(agpsStatusText), "Warm Start");
          agpsStatusColor = COLOR_ORANGE;
          gpsConfigured = true;
          fixAttemptStartTime = millis();
          startStopwatch();
        }
      } else {
        unsigned long remaining = (AGPS_START_DELAY_MS - elapsed) / 1000;
        if (elapsed % 1000 < 50) {
          snprintf(agpsStatusText, sizeof(agpsStatusText), "Starting in %lus", remaining + 1);
          agpsStatusColor = COLOR_YELLOW;
        }

        if (current != lastWiFiState) {
          lastWiFiState = current;
          if (current == WL_CONNECTED) {
            wifiAvailable = true;
            String ssid = WiFi.SSID();
            snprintf(wifiStatusText, sizeof(wifiStatusText), "WiFi: %s", ssid.c_str());
            wifiStatusColor = COLOR_MAGENTA;
            fetchApproximateLocation();
          } else {
            wifiAvailable = false;
            snprintf(wifiStatusText, sizeof(wifiStatusText), "WiFi: N/A");
            wifiStatusColor = COLOR_YELLOW;
          }
        }

        vTaskDelay(pdMS_TO_TICKS(10));
        continue;
      }
    }

    if (current != WL_CONNECTED) {
      if (millis() - lastWiFiRetry > wifiRetryInterval) {
        lastWiFiRetry = millis();
        WiFi.reconnect();
        wifiRetryAttempts++;
        wifiRetryInterval = min(300000UL, 5000UL * (1UL << min(wifiRetryAttempts, 6)));
      }
    } else {
      wifiRetryAttempts = 0;
      wifiRetryInterval = 5000;
    }

    if (current != lastWiFiState) {
      lastWiFiState = current;
      if (current == WL_CONNECTED) {
        wifiAvailable = true;
        String ssid = WiFi.SSID();
        snprintf(wifiStatusText, sizeof(wifiStatusText), "WiFi: %s", ssid.c_str());
        wifiStatusColor = COLOR_MAGENTA;

        if (agpsState == AGPS_OFFLINE_MODE && !gpsHasFix) {
          GPSUART.print("$PUBX,41,1,0001,0003,38400,0*18\r\n");
          delay(80);
          if (satelliteCount > 0) {
            sortSatellites();
            injectSatelliteData();
          }
          if (hasApproxLocation) {
            injectLocationToGPS(approxLat, approxLon, true);
          }
          if (lastSyncedTime != 0) {
            injectTimeToGPS(lastSyncedTime, true);
          }
          agpsState = AGPS_SUPER_HOT;
          snprintf(agpsStatusText, sizeof(agpsStatusText), "Super Hot");
          agpsStatusColor = COLOR_MAGENTA;
          fixAttemptStartTime = millis();
          startStopwatch();
        }
        fetchApproximateLocation();
      } else {
        wifiAvailable = false;
        snprintf(wifiStatusText, sizeof(wifiStatusText), "WiFi: N/A");
        wifiStatusColor = COLOR_YELLOW;
      }
    }

    unsigned long now = millis();

    if (wifiAvailable && !gpsHasFix && agpsState != AGPS_ALREADY_FIXED) {
      if (now - lastNTPAttempt >= FAST_NTP_INTERVAL) {
        lastNTPAttempt = now;
        syncFastNTP_nonblocking();
      }
    }

    static unsigned long lastInjection = 0;
    if (now - lastInjection >= 2000) {
      lastInjection = now;

      if (lastSyncedTime != 0) {
        injectTimeToGPS(lastSyncedTime, false);
      }
      if (hasApproxLocation) {
        injectLocationToGPS(approxLat, approxLon, false);
      }
      if (satelliteCount > 0 && (now / 15000) % 2 == 0) {
        sortSatellites();
        injectSatelliteData();
      }
    }

    if (gps.location.isValid() && gps.satellites.value() >= 4) {
      if (!gpsHasFix) {
        gpsHasFix = true;
        agpsCompleted = true;
        agpsState = AGPS_FIXED;
        stopStopwatch();
        snprintf(agpsStatusText, sizeof(agpsStatusText), "Fixed");
        agpsStatusColor = COLOR_GREEN;

        if (gps.location.isValid() && gps.altitude.isValid() && gps.course.isValid()) {
          saveLastFixData(
            gps.location.lat(),
            gps.location.lng(),
            gps.altitude.meters(),
            gps.course.deg(),
            lastSyncedTime
          );
        }
      }

      static unsigned long lastFixSave = 0;
      if (now - lastFixSave > 60000) {
        lastFixSave = now;
        if (gps.location.isValid() && gps.altitude.isValid() && gps.course.isValid()) {
          saveLastFixData(
            gps.location.lat(),
            gps.location.lng(),
            gps.altitude.meters(),
            gps.course.deg(),
            lastSyncedTime
          );
        }
      }
    } else {
      gpsHasFix = false;
      if (agpsState == AGPS_SUPER_HOT && (now - fixAttemptStartTime) >= SUPER_HOT_TIMEOUT_MS && !hotAttempted) {
        GPSUART.print("$PUBX,41,1,0001,0003,38400,0*18\r\n");
        delay(80);
        if (satelliteCount > 0) {
          sortSatellites();
          injectSatelliteData();
        }
        if (hasApproxLocation) {
          injectLocationToGPS(approxLat, approxLon, true);
        }
        if (lastSyncedTime != 0) {
          injectTimeToGPS(lastSyncedTime, true);
        }
        agpsState = AGPS_HOT;
        snprintf(agpsStatusText, sizeof(agpsStatusText), "Hot (Timeout)");
        agpsStatusColor = COLOR_CYAN;
        hotAttempted = true;
        fixAttemptStartTime = now;
        startStopwatch();
      } else if (agpsState == AGPS_HOT && (now - fixAttemptStartTime) >= HOT_TIMEOUT_MS && !warmFallbackDone) {
        GPSUART.print("$PUBX,41,1,0007,0003,38400,0*2C\r\n");
        delay(80);
        agpsState = AGPS_WARM;
        snprintf(agpsStatusText, sizeof(agpsStatusText), "Warm (Timeout)");
        agpsStatusColor = COLOR_ORANGE;
        warmFallbackDone = true;
        fixAttemptStartTime = now;
        startStopwatch();
      } else if (agpsState == AGPS_WARM && (now - fixAttemptStartTime) >= HOT_TIMEOUT_MS) {
        performOfflineAGPS();
        snprintf(agpsStatusText, sizeof(agpsStatusText), "Offline Fallback");
        agpsStatusColor = COLOR_BLUE;
        fixAttemptStartTime = now;
      }
    }

    vTaskDelay(pdMS_TO_TICKS(10));
  }
}

// ============================================
// LED CONTROL
// ============================================
void updateLED() {
  digitalWrite(LED_PIN, (gps.location.isValid() && hasApproxLocation) ? HIGH : LOW);
}

// ============================================
// BOOT ANIMATION
// ============================================
float easeInOutCubic(float t) {
  if (t < 0.5f) return 4.0f * t * t * t;
  float f = -2.0f * t + 2.0f;
  return 1.0f - (f * f * f) / 2.0f;
}

void bootAnimation() {
  uint16_t width = tft.width();
  uint16_t barWidth = width - 12;
  uint16_t barHeight = 37;
  uint16_t barX = 6;
  uint16_t barY = 82;

  tft.fillScreen(COLOR_BG);
  tft.setTextSize(1);
  tft.setTextColor(COLOR_YELLOW, COLOR_BG);
  tft.setCursor((width / 2) - (strlen("Shad's Speedometer") * 6 / 2), 40);
  tft.print("Shad's Speedometer");

  for (int s = 0; s <= 140; s++) {
    float progress = (float)s / 140.0f;
    float eased = easeInOutCubic(progress);
    int easedWidth = (int)(eased * barWidth);

    uint8_t r = (uint8_t)(255.0f * progress);
    uint8_t g = (uint8_t)(255.0f * (1.0f - progress));
    uint8_t b = 0;

    tft.fillRect(barX, barY, easedWidth, barHeight, tft.color565(r, g, b));
    tft.fillRect(barX + easedWidth, barY, barWidth - easedWidth, barHeight, COLOR_GREY);

    int textWidth = 70;
    int textX = barX + (barWidth - textWidth) / 2;
    int textY = barY - 16;
    tft.fillRect(textX, textY, textWidth, 16, COLOR_BG);
    tft.setTextColor(COLOR_YELLOW, COLOR_BG);
    tft.setCursor(textX, textY);
    tft.printf("%d km/h", s);
    delay(6);
  }
  tft.fillScreen(COLOR_BG);
}

// ============================================
// OPTIMIZED ARC CACHE FUNCTIONS
// ============================================
void initializeArcCache() {
  if (arcCacheInitialized) return;
  
  for (int i = 0; i < ARC_SEGMENT_COUNT; i++) {
    float angle = startAngle + (i / (float)ARC_SEGMENT_COUNT) * (endAngle - startAngle);
    float progress = (float)i / ARC_SEGMENT_COUNT;
    uint8_t r = (uint8_t)(255.0f * (1.0f - progress));
    uint8_t g = (uint8_t)(255.0f * progress);
    uint8_t b = 255;
    arcCache[i].color = tft.color565(r, g, b);
    arcCache[i].dirty = true;
  }
  arcCacheInitialized = true;
}

void drawArcSegment(int index, bool force) {
  if (!arcCacheInitialized) return;
  if (index < 0 || index >= ARC_SEGMENT_COUNT) return;
  if (!arcCache[index].dirty && !force) return;

  float angle = startAngle + (index / (float)ARC_SEGMENT_COUNT) * (endAngle - startAngle);
  float a1 = radians(angle);
  float a2 = radians(angle + (endAngle - startAngle) / ARC_SEGMENT_COUNT);
  
  int x1_outer = cx + cos(a1) * outerR;
  int y1_outer = cy + sin(a1) * outerR;
  int x2_outer = cx + cos(a2) * outerR;
  int y2_outer = cy + sin(a2) * outerR;
  int x1_inner = cx + cos(a1) * innerR;
  int y1_inner = cy + sin(a1) * innerR;
  int x2_inner = cx + cos(a2) * innerR;
  int y2_inner = cy + sin(a2) * innerR;
  
  dmaFillTriangle(x1_inner, y1_inner, x1_outer, y1_outer, x2_outer, y2_outer, arcCache[index].color);
  dmaFillTriangle(x1_inner, y1_inner, x2_outer, y2_outer, x2_inner, y2_inner, arcCache[index].color);
  
  arcCache[index].dirty = false;
}

void redrawNeedleArea(float oldAngle, float newAngle) {
  if (!arcCacheInitialized) return;
  
  int oldIndex = (int)((oldAngle - startAngle) / (endAngle - startAngle) * ARC_SEGMENT_COUNT);
  int newIndex = (int)((newAngle - startAngle) / (endAngle - startAngle) * ARC_SEGMENT_COUNT);
  
  oldIndex = constrain(oldIndex, 0, ARC_SEGMENT_COUNT - 1);
  newIndex = constrain(newIndex, 0, ARC_SEGMENT_COUNT - 1);
  
  int range = 4;
  int startIdx = min(oldIndex, newIndex);
  int endIdx = max(oldIndex, newIndex);
  
  for (int i = startIdx - range; i <= endIdx + range; i++) {
    if (i >= 0 && i < ARC_SEGMENT_COUNT) {
      arcCache[i].dirty = true;
    }
  }
}

// ============================================
// DRAW FUNCTIONS
// ============================================
void drawTopNotch() {
  static unsigned long lastDraw = 0;
  unsigned long now = millis();
  if (now - lastDraw < 100) return;
  lastDraw = now;

  tft.fillRect(0, 0, tft.width(), 28, COLOR_GREY);
  tft.setTextSize(1);

  tft.setTextColor(cacheStatusColor, COLOR_GREY);
  tft.setCursor(4, 6);
  tft.print("Cache:");
  tft.print(cacheStatusText);

  tft.setTextColor(COLOR_CYAN, COLOR_GREY);
  tft.setCursor(80, 6);
  tft.print("Sat:");
  tft.print(gps.satellites.value());

  tft.setTextColor(gps.location.isValid() ? COLOR_MAGENTA : COLOR_YELLOW, COLOR_GREY);
  tft.setCursor(140, 6);
  tft.print("Fix:");
  tft.print(gps.location.isValid() ? "Yes" : "No");

  tft.setTextColor(COLOR_WHITE, COLOR_GREY);
  tft.setCursor(188, 6);
  tft.printf(" %dMHz", getCpuFrequencyMhz());

  tft.setTextColor(COLOR_WHITE, COLOR_GREY);
  tft.setCursor(4, 19);
  tft.print("CV:");
  tft.setTextColor(COLOR_MAGENTA, COLOR_GREY);
  tft.printf("%.1f", avgCpuTempC);
  tft.setTextColor(COLOR_WHITE, COLOR_GREY);
  tft.print("-");
  tft.setTextColor(COLOR_CYAN, COLOR_GREY);
  tft.printf("%.1f", avgCpuVolt);

  tft.setTextColor(COLOR_WHITE, COLOR_GREY);
  tft.setCursor(100, 19);
  tft.print("GV:");
  tft.setTextColor(COLOR_MAGENTA, COLOR_GREY);
  tft.printf("%.1f", avgGpsTempC);
  tft.setTextColor(COLOR_WHITE, COLOR_GREY);
  tft.print("-");
  tft.setTextColor(COLOR_CYAN, COLOR_GREY);
  tft.printf("%.1f", avgGpsVolt);
}

void drawNMEALog() {
  unsigned long now = millis();

  unsigned long updateInterval;
  if (nmeaFastMode && (now - lastNmeaDisplayUpdate < 1000)) {
    updateInterval = 100;
  } else {
    updateInterval = 500;
    nmeaFastMode = false;
    nmeaSentenceCount = 0;
  }

  if (now - lastNmeaDisplayUpdate < updateInterval || !nmeaDataChanged) {
    return;
  }
  lastNmeaDisplayUpdate = now;
  nmeaDataChanged = false;

  int yStart = tft.height() - (NMEA_LINES_DISPLAY * 16) - 4;

  if (nmeaGPSPos >= 0) {
    int idx = (nmeaGPSPos + 0) % NMEA_LINES_DISPLAY;

    if (strcmp(nmeaDisplayBuffer[idx], lastDisplayedNMEA) != 0) {
      tft.fillRect(0, yStart, tft.width(), NMEA_LINES_DISPLAY * 16 + 4, COLOR_BG);
      tft.setTextSize(1);
      tft.setTextColor(COLOR_GREEN, COLOR_BG);
      for (int i = 0; i < NMEA_LINES_DISPLAY; i++) {
        idx = (nmeaGPSPos + i) % NMEA_LINES_DISPLAY;
        tft.setCursor(4, yStart + i * 16);
        if (strlen(nmeaDisplayBuffer[idx]) > 0) {
          tft.printf("GPS:ACTIVE!");
        }
      }
      strncpy(lastDisplayedNMEA, nmeaDisplayBuffer[(nmeaGPSPos + 0) % NMEA_LINES_DISPLAY], MAX_BUFFER_SIZE - 1);
      lastDisplayedNMEA[MAX_BUFFER_SIZE - 1] = '\0';
    }
  }
}

// ============================================
// OPTIMIZED DRAW STATS WITH 2PX NEEDLE & DMA
// ============================================
void drawStats() {
  unsigned long now = millis();

  static unsigned long lastTopUpdate = 0;
  if (now - lastTopUpdate >= 500) {
    lastTopUpdate = now;
    drawTopNotch();
  }

  if (now - lastRefreshAlt >= REFRESH_MS_ALT) {
    lastRefreshAlt = now;
    tft.fillRect(0, 28, tft.width(), 50, COLOR_BG);
    tft.setTextSize(1);

    tft.setCursor(6, 42);
    tft.setTextColor(COLOR_CYAN, COLOR_BG);
    if (gps.location.isValid()) {
      tft.printf("Lat: %.6f", gps.location.lat());
    } else {
      tft.print("Lat: ---");
    }

    tft.setCursor(6, 58);
    if (gps.location.isValid()) {
      tft.printf("Lon: %.6f", gps.location.lng());
    } else {
      tft.print("Lon: ---");
    }

    tft.setCursor(tft.width() - 50, 42);
    tft.setTextColor(COLOR_ORANGE, COLOR_BG);
    if (gps.course.isValid()) {
      float heading = gps.course.deg();
      const char* dir;
      if (heading >= 337.5 || heading < 22.5) dir = "N";
      else if (heading >= 22.5 && heading < 67.5) dir = "NE";
      else if (heading >= 67.5 && heading < 112.5) dir = "E";
      else if (heading >= 112.5 && heading < 157.5) dir = "SE";
      else if (heading >= 157.5 && heading < 202.5) dir = "S";
      else if (heading >= 202.5 && heading < 247.5) dir = "SW";
      else if (heading >= 247.5 && heading < 292.5) dir = "W";
      else if (heading >= 292.5 && heading < 337.5) dir = "NW";
      else dir = "---";
      tft.printf("%s", dir);
    } else {
      tft.printf("---");
    }
  }

  if (now - lastRefreshSpeed >= REFRESH_MS_SPEED) {
    lastRefreshSpeed = now;

    bool haveGoodFix = (gps.satellites.value() >= 3 && gps.location.isValid());
    float rawSpeed = haveGoodFix ? gps.speed.kmph() : 0.0f;
    rawSpeed = constrain(rawSpeed, SPEED_MIN_VALUE, SPEED_MAX_VALUE);
    displaySpeed = speedSmoother.update(rawSpeed, now);

    int displaySpeedInt = round(displaySpeed);
    int displayTopSpeedInt = round(speedSmoother.getTopSpeed());

    tft.setTextDatum(MC_DATUM);
    tft.setTextSize(3);
    tft.setTextColor(COLOR_RED, COLOR_BG);
    char speedStr[32];
    snprintf(speedStr, sizeof(speedStr), "%d km/h", displaySpeedInt);
    tft.drawString(speedStr, 120, 80);

    tft.setTextSize(2);
    tft.setTextColor(COLOR_ORANGE, COLOR_BG);
    snprintf(speedStr, sizeof(speedStr), "MAX: %d km/h", displayTopSpeedInt);
    tft.drawString(speedStr, 120, 106);

    // Initialize arc cache if not done
    if (!arcCacheInitialized) {
      initializeArcCache();
      for (int i = 0; i < ARC_SEGMENT_COUNT; i++) {
        drawArcSegment(i, true);
      }
    }

    // Calculate needle angle
    float rawNeedleAngle = startAngle + (displaySpeed / 140.0f) * (endAngle - startAngle);
    rawNeedleAngle = constrain(rawNeedleAngle, startAngle, endAngle);

    static float smoothNeedleAngle = startAngle;
    if (needleAngle < -900) {
      smoothNeedleAngle = rawNeedleAngle;
    } else {
      float angleDiff = rawNeedleAngle - smoothNeedleAngle;
      if (angleDiff > 180) angleDiff -= 360;
      if (angleDiff < -180) angleDiff += 360;
      smoothNeedleAngle += angleDiff * 0.3f;
    }
    needleAngle = smoothNeedleAngle;

    // Redraw arc segments along the entire path
    int needleLength = innerR + 10;
    
    if (lastNeedleAngle > -900) {
      redrawNeedleArea(lastNeedleAngle, needleAngle);
    } else {
      int idx = (int)((needleAngle - startAngle) / (endAngle - startAngle) * ARC_SEGMENT_COUNT);
      idx = constrain(idx, 0, ARC_SEGMENT_COUNT - 1);
      for (int i = -5; i <= 5; i++) {
        int idx2 = idx + i;
        if (idx2 >= 0 && idx2 < ARC_SEGMENT_COUNT) {
          arcCache[idx2].dirty = true;
        }
      }
    }
    
    // Redraw all dirty arc segments using DMA
    for (int i = 0; i < ARC_SEGMENT_COUNT; i++) {
      if (arcCache[i].dirty) {
        drawArcSegment(i, false);
      }
    }
    
    // Clear previous needle (2px wide) using DMA
    if (lastNeedleAngle > -900) {
      float rad = radians(lastNeedleAngle);
      int endX = cx + cos(rad) * needleLength;
      int endY = cy + sin(rad) * needleLength;
      
      float perpX = -sin(rad);
      float perpY = cos(rad);
      
      dmaDrawLine(cx, cy, endX, endY, COLOR_BG);
      dmaDrawLine(cx + perpX, cy + perpY, endX + perpX, endY + perpY, COLOR_BG);
    }

    // Draw new needle with 2px thickness using DMA
    float rad = radians(needleAngle);
    int endX = cx + cos(rad) * needleLength;
    int endY = cy + sin(rad) * needleLength;
    
    float perpX = -sin(rad);
    float perpY = cos(rad);
    
    dmaDrawLine(cx, cy, endX, endY, TFT_RED);
    dmaDrawLine(cx + perpX, cy + perpY, endX + perpX, endY + perpY, TFT_RED);
    
    // Redraw center circle on top using DMA
    dmaFillCircle(cx, cy, 6, COLOR_BG);
    dmaFillCircle(cx, cy, 5, TFT_BLACK);
    dmaFillCircle(cx, cy, 3, TFT_RED);
    dmaFillCircle(cx, cy, 1, TFT_WHITE);
    
    lastNeedleAngle = needleAngle;

    // Draw labels
    static int lastLabelSpeed = -1;
    int currentLabelSpeed = (int)(displaySpeed / 10) * 10;
    if (currentLabelSpeed != lastLabelSpeed || lastLabelSpeed == -1) {
      lastLabelSpeed = currentLabelSpeed;
      
      tft.setTextSize(1);
      tft.setTextColor(COLOR_WHITE, COLOR_BG);
      tft.setTextDatum(MC_DATUM);
      int labels[] = {0, 10, 20, 40, 50, 70, 80, 90, 110, 120, 140};
      for (int i = 0; i < 11; i++) {
        float ang = startAngle + (i / 10.0f) * (endAngle - startAngle);
        int tx = cx + cos(radians(ang)) * (outerR + 14);
        int ty = cy + sin(radians(ang)) * (outerR + 10);
        char labelStr[8];
        snprintf(labelStr, sizeof(labelStr), "%d", labels[i]);
        tft.drawString(labelStr, tx, ty);
      }
    }
  }

  drawNMEALog();
}

// ============================================
// STATUS DISPLAY
// ============================================
void updateStatusDisplay() {
  if (millis() - lastStatusDisplayUpdate < 1000) return;
  lastStatusDisplayUpdate = millis();

  #define STATUS_X 6
  #define STATUS_WIDTH 220
  #define STATUS_LINE_HEIGHT 14
  #define WIFI_Y 235
  #define AGPS_Y 250
  #define NTP_Y 265
  #define LOC_Y 280

  tft.setTextSize(1);

  tft.fillRect(STATUS_X, WIFI_Y - 3, STATUS_WIDTH, STATUS_LINE_HEIGHT, COLOR_BG);
  tft.setCursor(STATUS_X, WIFI_Y);
  tft.setTextColor(wifiStatusColor, COLOR_BG);
  tft.print(wifiStatusText);

  tft.fillRect(STATUS_X, AGPS_Y - 3, STATUS_WIDTH, STATUS_LINE_HEIGHT, COLOR_BG);
  tft.setCursor(STATUS_X, AGPS_Y);
  tft.setTextColor(agpsStatusColor, COLOR_BG);
  tft.print("AGPS: ");
  tft.print(agpsStatusText);

  if (stopwatchRunning) {
    unsigned long elapsed = millis() - stopwatchStartTime;
    char stopwatchStr[16];
    formatStopwatch(elapsed, stopwatchStr, sizeof(stopwatchStr));
    tft.setTextColor(COLOR_YELLOW, COLOR_BG);
    tft.print(" | ");
    tft.print(stopwatchStr);
  } else if (stopwatchCompleted) {
    if (millis() < stopwatchHideTime) {
      unsigned long elapsed = stopwatchEndTime - stopwatchStartTime;
      char stopwatchStr[16];
      formatStopwatch(elapsed, stopwatchStr, sizeof(stopwatchStr));
      tft.setTextColor(COLOR_GREEN, COLOR_BG);
      tft.print(" | ");
      tft.print(stopwatchStr);
    }
  }

  tft.fillRect(STATUS_X, NTP_Y - 3, STATUS_WIDTH, STATUS_LINE_HEIGHT, COLOR_BG);
  tft.setCursor(STATUS_X, NTP_Y);
  tft.setTextColor(ntpStatusColor, COLOR_BG);
  tft.print(ntpServerStatus);

  tft.fillRect(STATUS_X, LOC_Y - 3, STATUS_WIDTH, STATUS_LINE_HEIGHT, COLOR_BG);
  tft.setCursor(STATUS_X, LOC_Y);
  tft.setTextColor(locStatusColor, COLOR_BG);
  tft.print(locStatusText);
}

// ============================================
// SETUP
// ============================================
void setup() {
  Serial.begin(115200);
  Serial.println("=== Shad's Speedometer ===");
  Serial.println("Booting...");

  setCpuFrequencyMhz(CPU_FREQ_HIGH);

  Serial.println("Mounting SPIFFS...");
  if (!SPIFFS.begin(true)) {
    Serial.println("SPIFFS mount failed!");
    if (SPIFFS.format()) {
      Serial.println("SPIFFS formatted successfully");
      SPIFFS.begin(true);
    } else {
      Serial.println("SPIFFS format failed!");
    }
  }
  Serial.printf("SPIFFS: Total %d, Used %d\n", SPIFFS.totalBytes(), SPIFFS.usedBytes());

  float lat, lon, alt, course;
  time_t timestamp;
  if (loadLastFixData(lat, lon, alt, course, timestamp)) {
    approxLat = lat;
    approxLon = lon;
    hasApproxLocation = true;
    snprintf(locStatusText, sizeof(locStatusText), 
             "LOC: Cached (%.0fm, %.0f°)", alt, course);
    locStatusColor = COLOR_GREEN;
    injectLocationToGPS(approxLat, approxLon, true);
    Serial.println("Loaded cached location");
  }

  loadSatelliteData();
  if (satelliteCount > 0) {
    Serial.printf("Loaded %d cached satellites\n", satelliteCount);
  }

  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, LOW);

  tft.init();
  tft.setRotation(0);
  tft.fillScreen(COLOR_BG);
  Serial.println("TFT initialized");

  Serial.println("Configuring GPS...");
  setupGPSBaudRate();
  Serial.println("GPS configured at 38400 baud");

  bootAnimation();
  tft.fillScreen(COLOR_BG);
  
  initializeArcCache();
  
  drawTopNotch();
  drawStats();
  delay(500);

  Serial.println("Starting WiFi...");
  WiFi.mode(WIFI_STA);
  WiFi.setAutoReconnect(true);
  WiFi.persistent(true);
  WiFi.begin();

  Serial.printf("AGPS will start after %d ms delay...\n", AGPS_START_DELAY_MS);

  Serial.println("Creating AGPS task...");
  xTaskCreatePinnedToCore(
    agpsTaskAdaptive,
    "AGPS_Task",
    32768,
    NULL,
    1,
    NULL,
    0
  );

  Serial.println("Setup complete!");
  Serial.println("AGPS will use cached data if WiFi is not available");
}

// ============================================
// LOOP
// ============================================
void loop() {
  readGPS();
  drawStats();
  updateStatusDisplay();
  updateLED();
  updateSensors();
  manageCpuFrequency();

  vTaskDelay(pdMS_TO_TICKS(5));

  static unsigned long lastHeapCheck = 0;
  if (millis() - lastHeapCheck > 30000) {
    lastHeapCheck = millis();
    Serial.printf("Heap: Free=%d, MinFree=%d\n", 
                  ESP.getFreeHeap(), 
                  ESP.getMinFreeHeap());
  }
}