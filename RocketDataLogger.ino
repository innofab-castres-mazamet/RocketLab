/*
  AtomS3 + Unit Mini BPS v1.1 (QMP6988) - Logger fusée à eau
  -----------------------------------------------------------
  - IMU officielle M5Unified : M5.Imu.update() / M5.Imu.getImuData()
  - BPS officielle M5Unit-ENV : QMP6988 sur Port.A
  - Stockage binaire rapide dans LittleFS
  - Export CSV à la demande via Wi-Fi AP + interface web
  - Affichage temps réel sur l'écran

  Commandes :
  - Bouton clic quand IDLE -> démarre un nouveau vol
  - Bouton clic quand REC  -> arrête le vol courant
  - Interface web : /start et /stop
  - Arrêt automatique de sécurité après 2 minutes
*/

#include <Arduino.h>
#include <M5Unified.h>
#include "M5UnitENV.h"
#include <Wire.h>
#include <WiFi.h>
#include <WebServer.h>
#include <LittleFS.h>
#include <Preferences.h>
#include <math.h>

#define USING_BPS_V11
// Si tu as l'ancien Mini BPS BMP280, commente la ligne ci-dessus
// et décommente celle-ci :
// #define USING_BPS

#if defined(USING_BPS)
BMP280 bmp;
#elif defined(USING_BPS_V11)
QMP6988 qmp;
#else
#error "Choisir USING_BPS ou USING_BPS_V11"
#endif

static const char* AP_SSID = "Rocket-AtomS3";
static const char* AP_PASS = "rocketcsv";

constexpr uint32_t DISPLAY_PERIOD_MS   = 100;
constexpr uint32_t FLUSH_PERIOD_MS     = 200;
constexpr uint32_t AUTO_STOP_MS        = 120000;  // 2 minutes
constexpr size_t   WRITE_BLOCK_SAMPLES = 64;

constexpr float AXIS_SIGN_X =  1.0f;
constexpr float AXIS_SIGN_Y =  1.0f;
constexpr float AXIS_SIGN_Z =  1.0f;

enum FlightState : uint8_t {
  STATE_IDLE = 0,
  STATE_LOGGING
};

struct SampleRecord {
  uint32_t flightId;
  uint32_t t_us;

  float pressure_pa;
  float altitude_rel_m;
  float temp_c;

  float ax_g;
  float ay_g;
  float az_g;
  float a_norm_g;

  float gx_dps;
  float gy_dps;
  float gz_dps;

  float pitch_deg;
  float roll_deg;
};

WebServer server(80);
Preferences prefs;

FlightState state = STATE_IDLE;

File currentFlightFile;
String currentFlightPath;
String lastError;

uint32_t nextFlightId    = 1;
uint32_t currentFlightId = 0;

uint32_t flightStartUs   = 0;
uint32_t flightStartMs   = 0;
uint32_t lastImuRelUs    = 0;
uint32_t lastFlushMs     = 0;

SampleRecord writeBlock[WRITE_BLOCK_SAMPLES];
size_t writeCount = 0;

// live baro
float livePressurePa   = NAN;
float liveTempC        = NAN;
float launchPressurePa = NAN;
float liveAltRelM      = 0.0f;
float livePeakAltM     = 0.0f;

// live IMU
float liveAxG     = 0.0f;
float liveAyG     = 0.0f;
float liveAzG     = 0.0f;
float liveANormG  = 0.0f;
float liveGxDps   = 0.0f;
float liveGyDps   = 0.0f;
float liveGzDps   = 0.0f;
float livePitch   = 0.0f;
float liveRoll    = 0.0f;

float gyroBiasX = 0.0f;
float gyroBiasY = 0.0f;
float gyroBiasZ = 0.0f;
bool orientationInit = false;

// stats
uint32_t totalWrittenSamples = 0;
uint32_t totalDroppedSamples = 0;
uint32_t samplesThisSecond   = 0;
uint32_t sampleRateHz        = 0;
uint32_t lastRateMs          = 0;

String stateLabel() {
  return (state == STATE_LOGGING) ? "REC" : "IDLE";
}

String makeFlightPath(uint32_t id) {
  char buf[40];
  snprintf(buf, sizeof(buf), "/flights/flight_%04lu.bin", (unsigned long)id);
  return String(buf);
}

float altitudeFromPressureRelative(float pressurePa, float refPressurePa) {
  if (!isfinite(pressurePa) || !isfinite(refPressurePa) || pressurePa <= 0 || refPressurePa <= 0) {
    return NAN;
  }
  return 44330.0f * (1.0f - powf(pressurePa / refPressurePa, 0.1903f));
}

void setError(const String& s) {
  lastError = s;
  Serial.println("[ERR] " + s);
}

bool initIMU() {
  auto imuType = M5.Imu.getType();
  if (imuType == m5::imu_none) {
    setError("Aucune IMU detectee");
    return false;
  }
  M5.Imu.loadOffsetFromNVS();
  return true;
}

bool calibrateGyroAtRest() {
  constexpr int N = 300;
  float sx = 0, sy = 0, sz = 0;
  int ok = 0;
  uint32_t start = millis();

  while (ok < N && (millis() - start) < 8000) {
    if (M5.Imu.update()) {
      auto d = M5.Imu.getImuData();
      sx += d.gyro.x;
      sy += d.gyro.y;
      sz += d.gyro.z;
      ok++;
    } else {
      delay(1);
    }
  }

  if (ok < 50) {
    setError("Calibration gyro impossible");
    return false;
  }

  gyroBiasX = sx / ok;
  gyroBiasY = sy / ok;
  gyroBiasZ = sz / ok;
  return true;
}

bool initBaro() {
  auto pin_num_sda = M5.getPin(m5::pin_name_t::port_a_sda);
  auto pin_num_scl = M5.getPin(m5::pin_name_t::port_a_scl);

  Serial.printf("PortA SDA=%u SCL=%u\n", pin_num_sda, pin_num_scl);
  Wire.begin(pin_num_sda, pin_num_scl, 400000U);

#if defined(USING_BPS)
  if (!bmp.begin(&Wire, BMP280_I2C_ADDR, pin_num_sda, pin_num_scl, 400000U)) {
    setError("BMP280 non detecte");
    return false;
  }

  bmp.setSampling(BMP280::MODE_NORMAL,
                  BMP280::SAMPLING_X1,
                  BMP280::SAMPLING_X4,
                  BMP280::FILTER_OFF,
                  BMP280::STANDBY_MS_1);

#elif defined(USING_BPS_V11)
  if (!qmp.begin(&Wire, QMP6988_SLAVE_ADDRESS_L, pin_num_sda, pin_num_scl, 400000U)) {
    setError("QMP6988 non detecte");
    return false;
  }
#endif

  return true;
}

bool updateBaroLive() {
#if defined(USING_BPS)
  if (!bmp.update()) return false;
  livePressurePa = bmp.pressure;
  liveTempC = bmp.cTemp;
#elif defined(USING_BPS_V11)
  if (!qmp.update()) return false;
  livePressurePa = qmp.pressure;
  liveTempC = qmp.cTemp;
#endif

  if (isfinite(launchPressurePa)) {
    liveAltRelM = altitudeFromPressureRelative(livePressurePa, launchPressurePa);
    if (isfinite(liveAltRelM) && liveAltRelM > livePeakAltM) {
      livePeakAltM = liveAltRelM;
    }
  }

  return true;
}

void flushWriteBlock() {
  if (!currentFlightFile || writeCount == 0) return;

  size_t toWrite = writeCount * sizeof(SampleRecord);
  size_t written = currentFlightFile.write((const uint8_t*)writeBlock, toWrite);

  if (written != toWrite) {
    totalDroppedSamples += writeCount;
    setError("Ecriture partielle LittleFS");
  } else {
    totalWrittenSamples += writeCount;
  }

  currentFlightFile.flush();
  writeCount = 0;
}

void queueSample(const SampleRecord& rec) {
  if (writeCount >= WRITE_BLOCK_SAMPLES) {
    flushWriteBlock();
  }

  if (writeCount < WRITE_BLOCK_SAMPLES) {
    writeBlock[writeCount++] = rec;
  } else {
    totalDroppedSamples++;
  }
}

bool captureLaunchPressure() {
  float sum = 0.0f;
  int valid = 0;
  uint32_t start = millis();

  while (valid < 12 && (millis() - start) < 1500) {
    if (updateBaroLive()) {
      if (isfinite(livePressurePa) && livePressurePa > 10000.0f) {
        sum += livePressurePa;
        valid++;
      }
    }
    delay(10);
  }

  if (valid == 0) {
    setError("Impossible de lire la pression de depart");
    return false;
  }

  launchPressurePa = sum / valid;
  return true;
}

bool startFlight() {
  if (state == STATE_LOGGING) return false;

  currentFlightId = nextFlightId++;
  prefs.putUInt("next_id", nextFlightId);

  currentFlightPath = makeFlightPath(currentFlightId);
  currentFlightFile = LittleFS.open(currentFlightPath, FILE_WRITE);
  if (!currentFlightFile) {
    setError("Impossible d'ouvrir " + currentFlightPath);
    return false;
  }

  Serial.println("Created file: " + currentFlightPath);

  if (!captureLaunchPressure()) {
    currentFlightFile.close();
    return false;
  }

  state = STATE_LOGGING;
  flightStartUs = micros();
  flightStartMs = millis();
  lastImuRelUs = 0;
  lastFlushMs = millis();
  orientationInit = false;

  liveAltRelM = 0.0f;
  livePeakAltM = 0.0f;
  totalWrittenSamples = 0;
  totalDroppedSamples = 0;
  writeCount = 0;
  samplesThisSecond = 0;
  sampleRateHz = 0;
  lastRateMs = millis();

  Serial.printf("[VOL %lu] START %s\n",
                (unsigned long)currentFlightId,
                currentFlightPath.c_str());
  return true;
}

void stopFlight() {
  if (state != STATE_LOGGING) return;

  flushWriteBlock();

  if (currentFlightFile) {
    currentFlightFile.flush();
    currentFlightFile.close();
  }

  state = STATE_IDLE;
  Serial.printf("[VOL %lu] STOP\n", (unsigned long)currentFlightId);
}

void updateSampleRate() {
  uint32_t now = millis();
  if ((now - lastRateMs) >= 1000) {
    sampleRateHz = samplesThisSecond;
    samplesThisSecond = 0;
    lastRateMs = now;
  }
}

void sampleIfReady() {
  updateBaroLive();

  if (state != STATE_LOGGING) return;

  if (!M5.Imu.update()) {
    return;
  }

  auto data = M5.Imu.getImuData();

  float ax = data.accel.x * AXIS_SIGN_X;
  float ay = data.accel.y * AXIS_SIGN_Y;
  float az = data.accel.z * AXIS_SIGN_Z;

  float gx = data.gyro.x - gyroBiasX;
  float gy = data.gyro.y - gyroBiasY;
  float gz = data.gyro.z - gyroBiasZ;

  float aNorm = sqrtf(ax * ax + ay * ay + az * az);

  uint32_t relUs = micros() - flightStartUs;
  float dt = 0.0f;
  if (lastImuRelUs != 0 && relUs > lastImuRelUs) {
    dt = (relUs - lastImuRelUs) * 1e-6f;
  }
  lastImuRelUs = relUs;

  float pitchAcc = atan2f(-ax, sqrtf(ay * ay + az * az)) * 180.0f / PI;
  float rollAcc  = atan2f( ay, az ) * 180.0f / PI;

  if (!orientationInit) {
    livePitch = pitchAcc;
    liveRoll  = rollAcc;
    orientationInit = true;
  } else if (dt > 0.0f && dt < 0.1f) {
    constexpr float alpha = 0.98f;
    livePitch = alpha * (livePitch + gy * dt) + (1.0f - alpha) * pitchAcc;
    liveRoll  = alpha * (liveRoll  + gx * dt) + (1.0f - alpha) * rollAcc;
  }

  liveAxG = ax;
  liveAyG = ay;
  liveAzG = az;
  liveANormG = aNorm;
  liveGxDps = gx;
  liveGyDps = gy;
  liveGzDps = gz;

  SampleRecord rec;
  rec.flightId        = currentFlightId;
  rec.t_us            = relUs;
  rec.pressure_pa     = livePressurePa;
  rec.altitude_rel_m  = liveAltRelM;
  rec.temp_c          = liveTempC;
  rec.ax_g            = ax;
  rec.ay_g            = ay;
  rec.az_g            = az;
  rec.a_norm_g        = aNorm;
  rec.gx_dps          = gx;
  rec.gy_dps          = gy;
  rec.gz_dps          = gz;
  rec.pitch_deg       = livePitch;
  rec.roll_deg        = liveRoll;

  queueSample(rec);
  samplesThisSecond++;
}

String htmlHeader(const String& title) {
  String h;
  h += "<!doctype html><html><head><meta charset='utf-8'>";
  h += "<meta name='viewport' content='width=device-width,initial-scale=1'>";
  h += "<title>" + title + "</title>";
  h += "<style>";
  h += "body{font-family:Arial,sans-serif;margin:20px;line-height:1.4}";
  h += "code{background:#eee;padding:2px 4px}";
  h += "a.button,button{display:inline-block;padding:8px 12px;margin-right:10px;margin-bottom:10px;text-decoration:none;border:1px solid #666;border-radius:6px;background:#f3f3f3;color:#000}";
  h += "a{margin-right:10px}";
  h += ".warn{color:#b00}";
  h += ".ok{color:#080}";
  h += "</style></head><body>";
  return h;
}

void handleRoot() {
  String html = htmlHeader("Rocket AtomS3");
  html += "<h1>Rocket AtomS3</h1>";

  html += "<p>Etat: <b>" + stateLabel() + "</b><br>";
  html += "Vol courant: <b>" + String(currentFlightId) + "</b><br>";
  html += "Prochain vol: <b>" + String(nextFlightId) + "</b><br>";
  html += "IP: <b>" + WiFi.softAPIP().toString() + "</b></p>";

  if (state == STATE_IDLE) {
    html += "<p><a class='button' href='/start'>Demarrer un vol</a></p>";
  } else {
    html += "<p><a class='button' href='/stop'>Arreter le vol</a></p>";
  }

  html += "<p>Altitude: <b>" + String(liveAltRelM, 2) + " m</b><br>";
  html += "Altitude max: <b>" + String(livePeakAltM, 2) + " m</b><br>";
  html += "|a|: <b>" + String(liveANormG, 3) + " g</b><br>";
  html += "Pitch/Roll: <b>" + String(livePitch, 1) + " / " + String(liveRoll, 1) + " deg</b><br>";
  html += "Rate IMU: <b>" + String(sampleRateHz) + " Hz</b></p>";

  if (state == STATE_LOGGING) {
    uint32_t elapsed = millis() - flightStartMs;
    html += "<p>Duree enregistrement: <b>" + String(elapsed / 1000.0f, 1) + " s</b><br>";
    html += "Arret auto de securite a: <b>120 s</b></p>";
  }

  html += "<p>Samples ecrits: <b>" + String(totalWrittenSamples) + "</b><br>";
  html += "Samples perdus: <b>" + String(totalDroppedSamples) + "</b><br>";
  html += "LittleFS: <b>" + String(LittleFS.usedBytes() / 1024) + " / " + String(LittleFS.totalBytes() / 1024) + " KiB</b></p>";

  if (lastError.length()) {
    html += "<p class='warn'><b>Derniere erreur:</b> " + lastError + "</p>";
  }

  if (state == STATE_LOGGING) {
    html += "<p class='warn'>Evite les telechargements pendant l'enregistrement.</p>";
  }

  html += "<h2>Fichiers</h2><ul>";

  File root = LittleFS.open("/flights");
  if (root && root.isDirectory()) {
    File f = root.openNextFile();
    bool any = false;

    while (f) {
      if (!f.isDirectory()) {
        any = true;
        String rawName = f.name();
        String name = rawName.substring(rawName.lastIndexOf('/') + 1);
        String path = "/flights/" + name;

        html += "<li><code>" + name + "</code> (" + String(f.size()) + " octets) ";
        html += "<a href='/csv?file=" + path + "'>CSV</a>";
        html += "<a href='/bin?file=" + path + "'>BIN</a>";
        html += "<a href='/delete?file=" + path + "' onclick=\"return confirm('Supprimer ?');\">Supprimer</a>";
        html += "</li>";
      }
      f = root.openNextFile();
    }

    if (!any) {
      html += "<li>Aucun vol enregistre</li>";
    }
  } else {
    html += "<li>Dossier /flights absent</li>";
  }

  html += "</ul></body></html>";
  server.send(200, "text/html; charset=utf-8", html);
}

void handleStart() {
  if (state == STATE_IDLE) {
    startFlight();
  }
  server.sendHeader("Location", "/", true);
  server.send(303, "text/plain", "OK");
}

void handleStop() {
  if (state == STATE_LOGGING) {
    stopFlight();
  }
  server.sendHeader("Location", "/", true);
  server.send(303, "text/plain", "OK");
}

void handleBin() {
  if (!server.hasArg("file")) {
    server.send(400, "text/plain", "Parametre file manquant");
    return;
  }

  String path = server.arg("file");
  File f = LittleFS.open(path, FILE_READ);
  if (!f || f.isDirectory()) {
    server.send(404, "text/plain", "Fichier introuvable");
    return;
  }

  String name = path.substring(path.lastIndexOf('/') + 1);
  server.sendHeader("Content-Disposition", "attachment; filename=\"" + name + "\"");
  server.streamFile(f, "application/octet-stream");
  f.close();
}

void handleCSV() {
  if (!server.hasArg("file")) {
    server.send(400, "text/plain", "Parametre file manquant");
    return;
  }

  String path = server.arg("file");
  File f = LittleFS.open(path, FILE_READ);
  if (!f || f.isDirectory()) {
    server.send(404, "text/plain", "Fichier introuvable");
    return;
  }

  String name = path.substring(path.lastIndexOf('/') + 1);
  name.replace(".bin", ".csv");

  server.setContentLength(CONTENT_LENGTH_UNKNOWN);
  server.sendHeader("Content-Disposition", "attachment; filename=\"" + name + "\"");
  server.send(200, "text/csv; charset=utf-8", "");

  server.sendContent(
    "flight_id,t_us,pressure_pa,altitude_rel_m,temp_c,"
    "ax_g,ay_g,az_g,a_norm_g,gx_dps,gy_dps,gz_dps,pitch_deg,roll_deg\r\n"
  );

  SampleRecord rec;
  char line[320];

  while (f.available()) {
    if (f.read((uint8_t*)&rec, sizeof(rec)) != sizeof(rec)) break;

    int n = snprintf(
      line, sizeof(line),
      "%lu,%lu,%.2f,%.4f,%.2f,%.5f,%.5f,%.5f,%.5f,%.3f,%.3f,%.3f,%.3f,%.3f\r\n",
      (unsigned long)rec.flightId,
      (unsigned long)rec.t_us,
      rec.pressure_pa,
      rec.altitude_rel_m,
      rec.temp_c,
      rec.ax_g, rec.ay_g, rec.az_g, rec.a_norm_g,
      rec.gx_dps, rec.gy_dps, rec.gz_dps,
      rec.pitch_deg, rec.roll_deg
    );

    server.sendContent(String(line).substring(0, n));
    delay(0);
  }

  f.close();
  server.sendContent("");
}

void handleDelete() {
  if (!server.hasArg("file")) {
    server.send(400, "text/plain", "Parametre file manquant");
    return;
  }

  String path = server.arg("file");
  if (state == STATE_LOGGING && path == currentFlightPath) {
    server.send(409, "text/plain", "Impossible de supprimer le vol courant");
    return;
  }

  if (!LittleFS.exists(path)) {
    server.send(404, "text/plain", "Fichier introuvable");
    return;
  }

  if (LittleFS.remove(path)) {
    server.sendHeader("Location", "/", true);
    server.send(303, "text/plain", "Supprime");
  } else {
    server.send(500, "text/plain", "Suppression impossible");
  }
}

void setupWeb() {
  WiFi.mode(WIFI_AP);
  WiFi.softAP(AP_SSID, AP_PASS);

  Serial.print("AP IP: ");
  Serial.println(WiFi.softAPIP());

  server.on("/", handleRoot);
  server.on("/start", handleStart);
  server.on("/stop", handleStop);
  server.on("/bin", handleBin);
  server.on("/csv", handleCSV);
  server.on("/delete", handleDelete);
  server.onNotFound([]() {
    server.send(404, "text/plain", "404");
  });
  server.begin();
}

void drawScreen() {
  static uint32_t lastMs = 0;
  uint32_t now = millis();
  if ((now - lastMs) < DISPLAY_PERIOD_MS) return;
  lastMs = now;

  M5.Display.fillScreen(TFT_BLACK);
  M5.Display.setCursor(0, 0);
  M5.Display.setTextSize(1);
  M5.Display.setTextColor(TFT_WHITE, TFT_BLACK);

  uint32_t shownFlight = (state == STATE_IDLE) ? nextFlightId : currentFlightId;

  M5.Display.printf("VOL:%lu %s\n", (unsigned long)shownFlight, stateLabel().c_str());
  M5.Display.printf("Alt:%6.2f m\n", liveAltRelM);
  M5.Display.printf("Pic:%6.2f m\n", livePeakAltM);
  M5.Display.printf("|a|:%6.3f g\n", liveANormG);
  M5.Display.printf("P :%6.1f d\n", livePitch);
  M5.Display.printf("R :%6.1f d\n", liveRoll);
  M5.Display.printf("Hz:%4lu\n", (unsigned long)sampleRateHz);

  if (state == STATE_IDLE) {
    M5.Display.println();
    M5.Display.println(AP_SSID);
    M5.Display.print(WiFi.softAPIP().toString());
  } else {
    uint32_t elapsed = (millis() - flightStartMs) / 1000;
    M5.Display.printf("T:%4lus\n", (unsigned long)elapsed);
    M5.Display.printf("N:%lu\n", (unsigned long)totalWrittenSamples);
    M5.Display.printf("D:%lu\n", (unsigned long)totalDroppedSamples);
  }

  if (lastError.length()) {
    M5.Display.setCursor(100, 0);
    M5.Display.setTextColor(TFT_RED, TFT_BLACK);
    M5.Display.print("ERR");
  }
}

void setup() {
  auto cfg = M5.config();
  cfg.serial_baudrate = 115200;
  cfg.internal_imu = true;
  M5.begin(cfg);

  Serial.println();
  Serial.println("Rocket AtomS3 boot");

  M5.Display.setRotation(0);
  M5.Display.fillScreen(TFT_BLACK);
  M5.Display.setCursor(0, 0);
  M5.Display.setTextColor(TFT_WHITE, TFT_BLACK);
  M5.Display.println("Init...");

  if (!LittleFS.begin(true, "/littlefs", 10, "littlefs")) { 
    M5.Display.fillScreen(TFT_RED);
    M5.Display.setCursor(0,0);
    M5.Display.println("LittleFS FAIL");
    while (true) delay(1000);
  }

  LittleFS.mkdir("/flights");

  prefs.begin("rocket", false);
  nextFlightId = prefs.getUInt("next_id", 1);

  if (!initIMU()) {
    M5.Display.fillScreen(TFT_RED);
    M5.Display.println("IMU FAIL");
    M5.Display.println(lastError);
    while (true) delay(1000);
  }

  M5.Display.println("Gyro calib...");
  if (!calibrateGyroAtRest()) {
    M5.Display.fillScreen(TFT_RED);
    M5.Display.println("GYRO FAIL");
    M5.Display.println(lastError);
    while (true) delay(1000);
  }

  M5.Display.println("Baro init...");
  if (!initBaro()) {
    M5.Display.fillScreen(TFT_RED);
    M5.Display.println("BPS FAIL");
    M5.Display.println(lastError);
    while (true) delay(1000);
  }

  for (int i = 0; i < 10; ++i) {
    updateBaroLive();
    delay(10);
  }

  setupWeb();

  M5.Display.fillScreen(TFT_BLACK);
  M5.Display.setCursor(0, 0);
  M5.Display.println("READY");
  M5.Display.println(AP_SSID);
  M5.Display.print(WiFi.softAPIP().toString());

  Serial.printf("Gyro bias: %.3f %.3f %.3f dps\n", gyroBiasX, gyroBiasY, gyroBiasZ);
}

void loop() {
  M5.update();

  // bouton physique
  if (M5.BtnA.wasClicked()) {
    if (state == STATE_IDLE) {
      startFlight();
    } else {
      stopFlight();
    }
  }

  sampleIfReady();
  updateSampleRate();

  // flush périodique
  if (state == STATE_LOGGING && (millis() - lastFlushMs) >= FLUSH_PERIOD_MS) {
    flushWriteBlock();
    lastFlushMs = millis();
  }

  // arrêt auto de sécurité après 2 minutes
  if (state == STATE_LOGGING && (millis() - flightStartMs) >= AUTO_STOP_MS) {
    setError("Arret auto apres 2 minutes");
    stopFlight();
  }

  server.handleClient();
  drawScreen();

  delay(0);
}