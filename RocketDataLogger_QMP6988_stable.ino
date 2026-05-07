/*
  AtomS3 + Unit Mini BPS v1.1 (QMP6988) - Logger fusée à eau
  ----------------------------------------------------------------
  Version modifiée pour stabiliser l'altitude barométrique :
  - QMP6988 configuré avec oversampling pression plus élevé
  - filtre IIR QMP6988 plus fort
  - lecture baromètre à cadence fixe, pas à chaque tour de loop
  - pression de départ calculée sur plusieurs secondes avec tri + moyenne tronquée
  - rejet des lectures de pression absurdes ou des sauts impossibles
  - I2C abaissé à 100 kHz pour diagnostiquer/éviter les lectures instables

  Commandes :
  - Bouton clic quand IDLE -> démarre un nouveau vol
  - Bouton clic quand REC  -> arrête le vol courant
  - Interface web : /start et /stop
  - Arrêt automatique de sécurité après 2 minutes
*/

#include <M5Unified.h>
#include <M5UnitENV.h>
#include <WiFi.h>
#include <WebServer.h>
#include <LittleFS.h>
#include <Preferences.h>
#include <Wire.h>
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

constexpr uint32_t DISPLAY_PERIOD_MS = 100;
constexpr uint32_t FLUSH_PERIOD_MS = 200;
constexpr uint32_t AUTO_STOP_MS = 120000;  // 2 minutes
constexpr size_t WRITE_BLOCK_SAMPLES = 64;

// Baromètre : plus bas que 400 kHz pour tester une liaison I2C plus robuste.
constexpr uint32_t I2C_SPEED_HZ = 100000U;

// Cadence baro : 20 ms = 50 Hz. Suffisant pour une fusée à eau et plus propre
// que de relire le capteur à chaque tour de loop.
constexpr uint32_t BARO_PERIOD_MS = 20;

// Plage de pression plausible pour éviter d'exploiter une lecture I2C corrompue.
constexpr float PRESSURE_MIN_PA = 30000.0f;
constexpr float PRESSURE_MAX_PA = 110000.0f;

// Rejet d'un saut instantané irréaliste entre deux lectures baro.
// 300 Pa ~= 25 m d'altitude. Sur 20 ms, c'est déjà énorme pour ce projet.
constexpr float BARO_MAX_STEP_PA = 300.0f;
constexpr uint8_t BARO_RESYNC_AFTER_REJECTS = 8;

constexpr float AXIS_SIGN_X = 1.0f;
constexpr float AXIS_SIGN_Y = 1.0f;
constexpr float AXIS_SIGN_Z = 1.0f;

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

uint32_t nextFlightId = 1;
uint32_t currentFlightId = 0;
uint32_t flightStartUs = 0;
uint32_t flightStartMs = 0;
uint32_t lastImuRelUs = 0;
uint32_t lastFlushMs = 0;

SampleRecord writeBlock[WRITE_BLOCK_SAMPLES];
size_t writeCount = 0;

// live baro
float livePressurePa = NAN;
float liveTempC = NAN;
float launchPressurePa = NAN;
float liveAltRelM = 0.0f;
float livePeakAltM = 0.0f;
uint32_t lastBaroMs = 0;
uint32_t lastBaroUpdateMs = 0;
uint32_t baroGoodReads = 0;
uint32_t baroBadReads = 0;
uint8_t baroConsecutiveRejects = 0;

// live IMU
float liveAxG = 0.0f;
float liveAyG = 0.0f;
float liveAzG = 0.0f;
float liveANormG = 0.0f;
float liveGxDps = 0.0f;
float liveGyDps = 0.0f;
float liveGzDps = 0.0f;
float livePitch = 0.0f;
float liveRoll = 0.0f;

float gyroBiasX = 0.0f;
float gyroBiasY = 0.0f;
float gyroBiasZ = 0.0f;
bool orientationInit = false;

// stats
uint32_t totalWrittenSamples = 0;
uint32_t totalDroppedSamples = 0;
uint32_t samplesThisSecond = 0;
uint32_t sampleRateHz = 0;
uint32_t lastRateMs = 0;

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

bool pressureIsPlausible(float p) {
  return isfinite(p) && p >= PRESSURE_MIN_PA && p <= PRESSURE_MAX_PA;
}

void sortFloatArray(float* values, int count) {
  for (int i = 0; i < count - 1; i++) {
    for (int j = i + 1; j < count; j++) {
      if (values[j] < values[i]) {
        float tmp = values[i];
        values[i] = values[j];
        values[j] = tmp;
      }
    }
  }
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

  Wire.begin(pin_num_sda, pin_num_scl, I2C_SPEED_HZ);

#if defined(USING_BPS)
  if (!bmp.begin(&Wire, BMP280_I2C_ADDR, pin_num_sda, pin_num_scl, I2C_SPEED_HZ)) {
    setError("BMP280 non detecte");
    return false;
  }

  // Version plus stable que FILTER_OFF.
  // Pour une fusée très rapide, FILTER_X2 ou FILTER_X4 est un bon compromis.
  bmp.setSampling(
    BMP280::MODE_NORMAL,
    BMP280::SAMPLING_X1,
    BMP280::SAMPLING_X8,
    BMP280::FILTER_X4,
    BMP280::STANDBY_MS_1
  );

#elif defined(USING_BPS_V11)
  if (!qmp.begin(&Wire, QMP6988_SLAVE_ADDRESS_L, pin_num_sda, pin_num_scl, I2C_SPEED_HZ)) {
    setError("QMP6988 non detecte");
    return false;
  }

  // Réglages plus stables que les valeurs par défaut de la lib.
  // Si le pic d'altitude paraît trop lissé en vol, descends FILTERCOEFF_16 vers 8.
  qmp.setpPowermode(QMP6988_NORMAL_MODE);
  qmp.setFilter(QMP6988_FILTERCOEFF_16);
  qmp.setOversamplingP(QMP6988_OVERSAMPLING_32X);
  qmp.setOversamplingT(QMP6988_OVERSAMPLING_2X);
#endif

  lastBaroMs = 0;
  lastBaroUpdateMs = 0;
  baroGoodReads = 0;
  baroBadReads = 0;
  baroConsecutiveRejects = 0;

  return true;
}

bool readBaroRaw(float& pressurePa, float& tempC) {
#if defined(USING_BPS)
  if (!bmp.update()) return false;
  pressurePa = bmp.pressure;
  tempC = bmp.cTemp;
#elif defined(USING_BPS_V11)
  if (!qmp.update()) return false;
  pressurePa = qmp.pressure;
  tempC = qmp.cTemp;
#endif

  return pressureIsPlausible(pressurePa);
}

bool acceptBaroReading(float pressurePa) {
  if (!pressureIsPlausible(pressurePa)) {
    baroConsecutiveRejects++;
    return false;
  }

  if (isfinite(livePressurePa)) {
    float step = fabsf(pressurePa - livePressurePa);
    if (step > BARO_MAX_STEP_PA && baroConsecutiveRejects < BARO_RESYNC_AFTER_REJECTS) {
      baroConsecutiveRejects++;
      return false;
    }
  }

  baroConsecutiveRejects = 0;
  return true;
}

bool updateBaroLive(bool force = false) {
  uint32_t now = millis();

  // Ne relit pas le baromètre à chaque tour de loop.
  // Si on a déjà une valeur valide, on la garde jusqu'à la prochaine période.
  if (!force && (now - lastBaroMs) < BARO_PERIOD_MS) {
    return isfinite(livePressurePa);
  }
  lastBaroMs = now;

  float p = NAN;
  float t = NAN;
  if (!readBaroRaw(p, t)) {
    baroBadReads++;
    return false;
  }

  if (!acceptBaroReading(p)) {
    baroBadReads++;
    return false;
  }

  livePressurePa = p;
  liveTempC = t;
  lastBaroUpdateMs = now;
  baroGoodReads++;

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
  // 120 mesures espacées de 20 ms : environ 2,4 s.
  // On trie, puis on moyenne le centre pour éliminer les valeurs aberrantes.
  constexpr int N = 120;
  float values[N];
  int valid = 0;
  uint32_t start = millis();

  // Pendant cette phase, on repart proprement pour éviter qu'une ancienne valeur
  // baro perturbe le rejet de saut.
  livePressurePa = NAN;
  liveTempC = NAN;
  baroConsecutiveRejects = 0;

  while (valid < N && (millis() - start) < 4000) {
    float p = NAN;
    float t = NAN;

    if (readBaroRaw(p, t) && pressureIsPlausible(p)) {
      values[valid++] = p;
      livePressurePa = p;
      liveTempC = t;
    } else {
      baroBadReads++;
    }

    delay(BARO_PERIOD_MS);
  }

  if (valid < 30) {
    setError("Impossible de lire la pression de depart");
    return false;
  }

  sortFloatArray(values, valid);

  int trim = valid / 10;  // retire 10 % en bas et 10 % en haut
  int first = trim;
  int last = valid - trim;
  if (last <= first) {
    first = 0;
    last = valid;
  }

  double sum = 0.0;
  int count = 0;
  for (int i = first; i < last; i++) {
    sum += values[i];
    count++;
  }

  launchPressurePa = (float)(sum / count);
  livePressurePa = launchPressurePa;
  liveAltRelM = 0.0f;
  livePeakAltM = 0.0f;
  lastBaroMs = millis();
  lastBaroUpdateMs = lastBaroMs;
  baroConsecutiveRejects = 0;

  Serial.printf(
    "Pression depart: %.2f Pa, samples=%d, min=%.2f, max=%.2f\n",
    launchPressurePa,
    valid,
    values[0],
    values[valid - 1]
  );

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

  Serial.printf("[VOL %lu] START %s\n", (unsigned long)currentFlightId, currentFlightPath.c_str());
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
  updateBaroLive(false);

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
  float rollAcc = atan2f(ay, az) * 180.0f / PI;

  if (!orientationInit) {
    livePitch = pitchAcc;
    liveRoll = rollAcc;
    orientationInit = true;
  } else if (dt > 0.0f && dt < 0.1f) {
    constexpr float alpha = 0.98f;
    livePitch = alpha * (livePitch + gy * dt) + (1.0f - alpha) * pitchAcc;
    liveRoll = alpha * (liveRoll + gx * dt) + (1.0f - alpha) * rollAcc;
  }

  liveAxG = ax;
  liveAyG = ay;
  liveAzG = az;
  liveANormG = aNorm;
  liveGxDps = gx;
  liveGyDps = gy;
  liveGzDps = gz;

  SampleRecord rec;
  rec.flightId = currentFlightId;
  rec.t_us = relUs;
  rec.pressure_pa = livePressurePa;
  rec.altitude_rel_m = liveAltRelM;
  rec.temp_c = liveTempC;
  rec.ax_g = ax;
  rec.ay_g = ay;
  rec.az_g = az;
  rec.a_norm_g = aNorm;
  rec.gx_dps = gx;
  rec.gy_dps = gy;
  rec.gz_dps = gz;
  rec.pitch_deg = livePitch;
  rec.roll_deg = liveRoll;

  queueSample(rec);
  samplesThisSecond++;
}

String htmlHeader(const String& title) {
  String h;
  h += "<!doctype html><html><head><meta charset='utf-8'>";
  h += "<meta name='viewport' content='width=device-width,initial-scale=1'>";
  h += "<title>" + title + "</title>";
  h += "<style>body{font-family:sans-serif;margin:20px;line-height:1.4}";
  h += "a.button{display:inline-block;padding:10px 14px;background:#0b74de;color:white;text-decoration:none;border-radius:8px;margin:4px 0}";
  h += "a.stop{background:#c62828}";
  h += "code{background:#eee;padding:2px 4px;border-radius:4px}";
  h += ".err{color:#c62828;font-weight:bold}";
  h += "</style></head><body>";
  return h;
}

void handleRoot() {
  String html = htmlHeader("Rocket AtomS3");
  html += "<h1>Rocket AtomS3</h1>";
  html += "<p><b>Etat:</b> " + stateLabel() + "<br>";
  html += "<b>Vol courant:</b> " + String(currentFlightId) + "<br>";
  html += "<b>Prochain vol:</b> " + String(nextFlightId) + "<br>";
  html += "<b>IP:</b> " + WiFi.softAPIP().toString() + "</p>";

  if (state == STATE_IDLE) {
    html += "<p><a class='button' href='/start'>Demarrer un vol</a></p>";
  } else {
    html += "<p><a class='button stop' href='/stop'>Arreter le vol</a></p>";
  }

  html += "<p>";
  html += "<b>Altitude:</b> " + String(liveAltRelM, 2) + " m<br>";
  html += "<b>Altitude max:</b> " + String(livePeakAltM, 2) + " m<br>";
  html += "<b>Pression:</b> " + String(livePressurePa, 2) + " Pa<br>";
  html += "<b>Temperature:</b> " + String(liveTempC, 2) + " &deg;C<br>";
  html += "<b>Lectures baro OK/KO:</b> " + String(baroGoodReads) + " / " + String(baroBadReads) + "<br>";
  html += "<b>|a|:</b> " + String(liveANormG, 3) + " g<br>";
  html += "<b>Pitch/Roll:</b> " + String(livePitch, 1) + " / " + String(liveRoll, 1) + " deg<br>";
  html += "<b>Rate IMU:</b> " + String(sampleRateHz) + " Hz";
  html += "</p>";

  if (state == STATE_LOGGING) {
    uint32_t elapsed = millis() - flightStartMs;
    html += "<p><b>Duree enregistrement:</b> " + String(elapsed / 1000.0f, 1) + " s<br>";
    html += "Arret auto de securite a: 120 s</p>";
  }

  html += "<p>";
  html += "<b>Samples ecrits:</b> " + String(totalWrittenSamples) + "<br>";
  html += "<b>Samples en buffer:</b> " + String(writeCount) + "<br>";
  html += "<b>Samples perdus:</b> " + String(totalDroppedSamples) + "<br>";
  html += "<b>LittleFS:</b> " + String(LittleFS.usedBytes() / 1024) + " / " + String(LittleFS.totalBytes() / 1024) + " KiB";
  html += "</p>";

  if (lastError.length()) {
    html += "<p class='err'>Derniere erreur: " + lastError + "</p>";
  }

  if (state == STATE_LOGGING) {
    html += "<p><b>Note:</b> evite les telechargements pendant l'enregistrement.</p>";
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
        html += "<a href='/csv?file=" + path + "'>CSV</a> ";
        html += "<a href='/bin?file=" + path + "'>BIN</a> ";
        html += "<a href='/delete?file=" + path + "'>Supprimer</a></li>";
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
      line,
      sizeof(line),
      "%lu,%lu,%.2f,%.4f,%.2f,%.5f,%.5f,%.5f,%.5f,%.3f,%.3f,%.3f,%.3f,%.3f\r\n",
      (unsigned long)rec.flightId,
      (unsigned long)rec.t_us,
      rec.pressure_pa,
      rec.altitude_rel_m,
      rec.temp_c,
      rec.ax_g,
      rec.ay_g,
      rec.az_g,
      rec.a_norm_g,
      rec.gx_dps,
      rec.gy_dps,
      rec.gz_dps,
      rec.pitch_deg,
      rec.roll_deg
    );

    if (n > 0) {
      server.sendContent(String(line).substring(0, n));
    }
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
  M5.Display.printf("P:%8.1fPa\n", livePressurePa);
  M5.Display.printf("|a|:%6.3f g\n", liveANormG);
  M5.Display.printf("P/R:%4.0f/%4.0f\n", livePitch, liveRoll);
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
    M5.Display.setCursor(0, 0);
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

  // Petite phase de chauffe/amorçage du capteur.
  M5.Display.println("Baro warmup...");
  for (int i = 0; i < 30; ++i) {
    updateBaroLive(true);
    delay(BARO_PERIOD_MS);
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
