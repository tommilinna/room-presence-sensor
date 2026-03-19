#include <Arduino.h>
#include <WiFi.h>
#include <heltec.h>

#include <ctype.h>
#include <string.h>

// -----------------------------
// User-tunable configuration
// -----------------------------
// Calibrate these values in your actual room. RSSI varies with wall layout,
// mounting height, phone orientation, battery state, and whether the user is
// standing between the device and the sensor.
static const bool CALIBRATION_MODE = true;
static const char SENSOR_NAME[] = "Office Sensor";
static const uint32_t SCAN_INTERVAL_MS = 3000;
static const int16_t PRESENT_RSSI_THRESHOLD = -67;
static const int16_t ABSENT_RSSI_THRESHOLD = -75;
static const uint8_t REQUIRED_PRESENT_COUNT = 2;
static const uint8_t REQUIRED_ABSENT_COUNT = 3;
static const uint8_t MAX_TARGETS = 4;

// If an entry has a BSSID, that BSSID is used as the only match for that slot.
// SSID is only used as a fallback when the BSSID entry is blank.
static const char *TARGET_NAMES[MAX_TARGETS] = {
  "Phone",
  "Bedroom AP",
  "",
  ""
};

static const char *TARGET_BSSIDS[MAX_TARGETS] = {
  "AA:BB:CC:DD:EE:FF",
  "11:22:33:44:55:66",
  "",
  ""
};

static const char *TARGET_SSIDS[MAX_TARGETS] = {
  "",
  "",
  "",
  ""
};

// -----------------------------
// Internal tuning constants
// -----------------------------
static const uint8_t RSSI_WINDOW_SIZE = 5;
static const uint8_t CALIBRATION_TOP_COUNT = 8;
static const int16_t MISSING_RSSI_VALUE = -100;
static const uint32_t WIFI_SCAN_MAX_MS_PER_CHANNEL = 120;

enum PresenceState : uint8_t {
  NOT_PRESENT = 0,
  MAYBE_PRESENT = 1,
  PRESENT = 2
};

struct TargetObservation {
  bool matched;
  bool matchedByBssid;
  int16_t rssi;
  int16_t channel;
  char ssid[33];
  char bssid[18];
};

struct CalibrationNetwork {
  bool valid;
  int16_t rssi;
  int16_t channel;
  char ssid[33];
  char bssid[18];
};

static bool gDisplayReady = false;
static PresenceState gPresenceState = NOT_PRESENT;
static char gStateReason[96] = "boot";

static uint32_t gBootMs = 0;
static uint32_t gNextScanDueMs = 0;
static uint32_t gLastScanStartedMs = 0;
static uint32_t gLastScanCompletedMs = 0;
static uint32_t gLastScanDurationMs = 0;
static uint32_t gScanCounter = 0;
static int16_t gVisibleNetworkCount = 0;

static uint8_t gConsecutivePresent = 0;
static uint8_t gConsecutiveAbsent = 0;

static int16_t gRssiWindow[RSSI_WINDOW_SIZE];
static uint8_t gRssiWindowCount = 0;
static uint8_t gRssiWindowIndex = 0;
static int16_t gSmoothedRssi = MISSING_RSSI_VALUE;

static TargetObservation gMatchedTargets[MAX_TARGETS];
static CalibrationNetwork gCalibrationTop[CALIBRATION_TOP_COUNT];
static uint8_t gMatchedCount = 0;
static int8_t gStrongestTargetIndex = -1;
static int16_t gStrongestRssi = MISSING_RSSI_VALUE;

static bool isConfigured(const char *value) {
  return value != nullptr && value[0] != '\0';
}

static void copyCStringSafe(char *dest, size_t destSize, const char *src) {
  if (destSize == 0) {
    return;
  }

  if (src == nullptr) {
    dest[0] = '\0';
    return;
  }

  strncpy(dest, src, destSize - 1);
  dest[destSize - 1] = '\0';
}

static void copyStringSafe(char *dest, size_t destSize, const String &src) {
  if (destSize == 0) {
    return;
  }

  src.toCharArray(dest, destSize);
  dest[destSize - 1] = '\0';
}

static bool equalsIgnoreCase(const char *a, const char *b) {
  if (a == nullptr || b == nullptr) {
    return false;
  }

  while (*a != '\0' && *b != '\0') {
    if (tolower(static_cast<unsigned char>(*a)) != tolower(static_cast<unsigned char>(*b))) {
      return false;
    }
    ++a;
    ++b;
  }

  return *a == '\0' && *b == '\0';
}

static const char *stateNameForLog(PresenceState state) {
  switch (state) {
    case PRESENT:
      return "PRESENT";
    case MAYBE_PRESENT:
      return "MAYBE_PRESENT";
    case NOT_PRESENT:
    default:
      return "NOT_PRESENT";
  }
}

static const char *stateNameForDisplay(PresenceState state) {
  switch (state) {
    case PRESENT:
      return "PRESENT";
    case MAYBE_PRESENT:
      return "MAYBE";
    case NOT_PRESENT:
    default:
      return "AWAY";
  }
}

static void clearTargetMatches() {
  gMatchedCount = 0;
  gStrongestTargetIndex = -1;
  gStrongestRssi = MISSING_RSSI_VALUE;

  for (uint8_t i = 0; i < MAX_TARGETS; ++i) {
    gMatchedTargets[i].matched = false;
    gMatchedTargets[i].matchedByBssid = false;
    gMatchedTargets[i].rssi = MISSING_RSSI_VALUE;
    gMatchedTargets[i].channel = 0;
    gMatchedTargets[i].ssid[0] = '\0';
    gMatchedTargets[i].bssid[0] = '\0';
  }
}

static void clearCalibrationTop() {
  for (uint8_t i = 0; i < CALIBRATION_TOP_COUNT; ++i) {
    gCalibrationTop[i].valid = false;
    gCalibrationTop[i].rssi = MISSING_RSSI_VALUE;
    gCalibrationTop[i].channel = 0;
    gCalibrationTop[i].ssid[0] = '\0';
    gCalibrationTop[i].bssid[0] = '\0';
  }
}

static void pushRssiSample(int16_t sample) {
  gRssiWindow[gRssiWindowIndex] = sample;
  gRssiWindowIndex = (gRssiWindowIndex + 1) % RSSI_WINDOW_SIZE;

  if (gRssiWindowCount < RSSI_WINDOW_SIZE) {
    ++gRssiWindowCount;
  }

  int16_t sorted[RSSI_WINDOW_SIZE];
  for (uint8_t i = 0; i < gRssiWindowCount; ++i) {
    sorted[i] = gRssiWindow[i];
  }

  for (uint8_t i = 1; i < gRssiWindowCount; ++i) {
    int16_t key = sorted[i];
    int8_t j = static_cast<int8_t>(i) - 1;
    while (j >= 0 && sorted[j] > key) {
      sorted[j + 1] = sorted[j];
      --j;
    }
    sorted[j + 1] = key;
  }

  gSmoothedRssi = sorted[gRssiWindowCount / 2];
}

static bool targetMatchesNetwork(uint8_t targetIndex, const char *ssid, const char *bssid, bool *matchedByBssid) {
  if (matchedByBssid != nullptr) {
    *matchedByBssid = false;
  }

  if (targetIndex >= MAX_TARGETS) {
    return false;
  }

  if (isConfigured(TARGET_BSSIDS[targetIndex])) {
    if (isConfigured(bssid) && equalsIgnoreCase(TARGET_BSSIDS[targetIndex], bssid)) {
      if (matchedByBssid != nullptr) {
        *matchedByBssid = true;
      }
      return true;
    }
    return false;
  }

  if (isConfigured(TARGET_SSIDS[targetIndex]) && isConfigured(ssid)) {
    return strcmp(TARGET_SSIDS[targetIndex], ssid) == 0;
  }

  return false;
}

static void maybeInsertCalibrationNetwork(const char *ssid, const char *bssid, int16_t rssi, int16_t channel) {
  for (uint8_t slot = 0; slot < CALIBRATION_TOP_COUNT; ++slot) {
    if (!gCalibrationTop[slot].valid || rssi > gCalibrationTop[slot].rssi) {
      for (int8_t move = static_cast<int8_t>(CALIBRATION_TOP_COUNT) - 1; move > static_cast<int8_t>(slot); --move) {
        gCalibrationTop[move] = gCalibrationTop[move - 1];
      }

      gCalibrationTop[slot].valid = true;
      gCalibrationTop[slot].rssi = rssi;
      gCalibrationTop[slot].channel = channel;
      copyCStringSafe(gCalibrationTop[slot].ssid, sizeof(gCalibrationTop[slot].ssid), ssid);
      copyCStringSafe(gCalibrationTop[slot].bssid, sizeof(gCalibrationTop[slot].bssid), bssid);
      return;
    }
  }
}

static void buildTargetName(uint8_t targetIndex, char *out, size_t outSize) {
  if (outSize == 0) {
    return;
  }

  if (targetIndex >= MAX_TARGETS) {
    copyCStringSafe(out, outSize, "none");
    return;
  }

  if (isConfigured(TARGET_NAMES[targetIndex])) {
    copyCStringSafe(out, outSize, TARGET_NAMES[targetIndex]);
    return;
  }

  if (gMatchedTargets[targetIndex].bssid[0] != '\0') {
    size_t len = strlen(gMatchedTargets[targetIndex].bssid);
    const char *suffix = gMatchedTargets[targetIndex].bssid;
    if (len > 8) {
      suffix = gMatchedTargets[targetIndex].bssid + (len - 8);
    }
    copyCStringSafe(out, outSize, suffix);
    return;
  }

  if (gMatchedTargets[targetIndex].ssid[0] != '\0') {
    copyCStringSafe(out, outSize, gMatchedTargets[targetIndex].ssid);
    return;
  }

  snprintf(out, outSize, "slot_%u", targetIndex);
}

static void formatScanAge(char *out, size_t outSize) {
  if (outSize == 0) {
    return;
  }

  if (gLastScanCompletedMs == 0) {
    copyCStringSafe(out, outSize, "Age: --");
    return;
  }

  uint32_t ageMs = millis() - gLastScanCompletedMs;
  uint32_t deciseconds = ageMs / 100;
  snprintf(out, outSize, "Age: %lu.%lus", static_cast<unsigned long>(deciseconds / 10), static_cast<unsigned long>(deciseconds % 10));
}

// All display-specific calls live here and in renderDisplay().
// If your installed Heltec package uses slightly different font/text names,
// only these two functions should need small edits.
void initDisplay() {
#if defined(Heltec_Screen)
  gDisplayReady = (Heltec.display != nullptr);
  if (!gDisplayReady) {
    return;
  }

  Heltec.display->clear();
  Heltec.display->setTextAlignment(TEXT_ALIGN_LEFT);
  Heltec.display->setFont(ArialMT_Plain_10);
  Heltec.display->drawString(0, 0, SENSOR_NAME);
  Heltec.display->drawString(0, 12, "State: BOOT");
  Heltec.display->drawString(0, 24, "RSSI: --");
  Heltec.display->drawString(0, 36, "Target: none");
  Heltec.display->drawString(0, 48, "Age: --");
  Heltec.display->display();
#else
  gDisplayReady = false;
#endif
}

void scanWiFiTargets() {
  gLastScanStartedMs = millis();
  ++gScanCounter;
  clearTargetMatches();
  clearCalibrationTop();

  int16_t networkCount = WiFi.scanNetworks(false, true, false, WIFI_SCAN_MAX_MS_PER_CHANNEL);
  gVisibleNetworkCount = (networkCount > 0) ? networkCount : 0;

  if (networkCount > 0) {
    for (int16_t i = 0; i < networkCount; ++i) {
      String ssidString = WiFi.SSID(i);
      String bssidString = WiFi.BSSIDstr(i);
      int16_t rssi = static_cast<int16_t>(WiFi.RSSI(i));
      int16_t channel = static_cast<int16_t>(WiFi.channel(i));

      char ssid[33];
      char bssid[18];
      copyStringSafe(ssid, sizeof(ssid), ssidString);
      copyStringSafe(bssid, sizeof(bssid), bssidString);

      if (CALIBRATION_MODE) {
        maybeInsertCalibrationNetwork(ssid, bssid, rssi, channel);
      }

      for (uint8_t targetIndex = 0; targetIndex < MAX_TARGETS; ++targetIndex) {
        bool matchedByBssid = false;
        if (!targetMatchesNetwork(targetIndex, ssid, bssid, &matchedByBssid)) {
          continue;
        }

        if (!gMatchedTargets[targetIndex].matched || rssi > gMatchedTargets[targetIndex].rssi) {
          gMatchedTargets[targetIndex].matched = true;
          gMatchedTargets[targetIndex].matchedByBssid = matchedByBssid;
          gMatchedTargets[targetIndex].rssi = rssi;
          gMatchedTargets[targetIndex].channel = channel;
          copyCStringSafe(gMatchedTargets[targetIndex].ssid, sizeof(gMatchedTargets[targetIndex].ssid), ssid);
          copyCStringSafe(gMatchedTargets[targetIndex].bssid, sizeof(gMatchedTargets[targetIndex].bssid), bssid);
        }
      }
    }
  }

  if (networkCount >= 0) {
    WiFi.scanDelete();
  }

  for (uint8_t i = 0; i < MAX_TARGETS; ++i) {
    if (!gMatchedTargets[i].matched) {
      continue;
    }

    ++gMatchedCount;
    if (gStrongestTargetIndex < 0 || gMatchedTargets[i].rssi > gStrongestRssi) {
      gStrongestTargetIndex = static_cast<int8_t>(i);
      gStrongestRssi = gMatchedTargets[i].rssi;
    }
  }

  if (gStrongestTargetIndex >= 0) {
    pushRssiSample(gStrongestRssi);
  } else {
    pushRssiSample(MISSING_RSSI_VALUE);
  }

  gLastScanDurationMs = millis() - gLastScanStartedMs;
  gLastScanCompletedMs = millis();
}

void updatePresenceState() {
  PresenceState previousState = gPresenceState;
  char reason[96];
  reason[0] = '\0';

  bool matchedThisScan = (gStrongestTargetIndex >= 0);
  bool strongPresent = matchedThisScan && (gSmoothedRssi >= PRESENT_RSSI_THRESHOLD);
  bool strongAbsent = (!matchedThisScan) || (gSmoothedRssi <= ABSENT_RSSI_THRESHOLD);

  if (strongPresent) {
    if (gConsecutivePresent < 255) {
      ++gConsecutivePresent;
    }
    gConsecutiveAbsent = 0;
  } else if (strongAbsent) {
    if (gConsecutiveAbsent < 255) {
      ++gConsecutiveAbsent;
    }
    gConsecutivePresent = 0;
  } else {
    gConsecutivePresent = 0;
    gConsecutiveAbsent = 0;
  }

  switch (gPresenceState) {
    case NOT_PRESENT:
      if (gConsecutivePresent >= REQUIRED_PRESENT_COUNT) {
        gPresenceState = PRESENT;
        snprintf(reason, sizeof(reason), "promoted_to_present_present_scans=%u", gConsecutivePresent);
      } else if (matchedThisScan) {
        gPresenceState = MAYBE_PRESENT;
        snprintf(reason, sizeof(reason), "target_seen_waiting_confirm_present=%u/%u", gConsecutivePresent, REQUIRED_PRESENT_COUNT);
      } else {
        snprintf(reason, sizeof(reason), "holding_away_no_target_match");
      }
      break;

    case MAYBE_PRESENT:
      if (gConsecutivePresent >= REQUIRED_PRESENT_COUNT) {
        gPresenceState = PRESENT;
        snprintf(reason, sizeof(reason), "promoted_to_present_present_scans=%u", gConsecutivePresent);
      } else if (gConsecutiveAbsent >= REQUIRED_ABSENT_COUNT) {
        gPresenceState = NOT_PRESENT;
        snprintf(reason, sizeof(reason), "returned_to_away_absent_scans=%u", gConsecutiveAbsent);
      } else if (strongPresent) {
        snprintf(reason, sizeof(reason), "holding_maybe_present_evidence=%u/%u", gConsecutivePresent, REQUIRED_PRESENT_COUNT);
      } else if (strongAbsent) {
        snprintf(reason, sizeof(reason), "holding_maybe_absent_evidence=%u/%u", gConsecutiveAbsent, REQUIRED_ABSENT_COUNT);
      } else {
        snprintf(reason, sizeof(reason), "holding_maybe_in_hysteresis_band");
      }
      break;

    case PRESENT:
      if (gConsecutiveAbsent >= REQUIRED_ABSENT_COUNT) {
        gPresenceState = NOT_PRESENT;
        snprintf(reason, sizeof(reason), "dropped_to_away_absent_scans=%u", gConsecutiveAbsent);
      } else if (!strongPresent) {
        gPresenceState = MAYBE_PRESENT;
        if (matchedThisScan) {
          snprintf(reason, sizeof(reason), "present_weakened_into_hysteresis");
        } else {
          snprintf(reason, sizeof(reason), "present_target_missing_first_miss");
        }
      } else {
        snprintf(reason, sizeof(reason), "holding_present");
      }
      break;
  }

  if (previousState != gPresenceState) {
    snprintf(gStateReason, sizeof(gStateReason), "%s_to_%s_%s", stateNameForLog(previousState), stateNameForLog(gPresenceState), reason);
  } else {
    copyCStringSafe(gStateReason, sizeof(gStateReason), reason);
  }
}

void renderDisplay() {
#if defined(Heltec_Screen)
  if (!gDisplayReady || Heltec.display == nullptr) {
    return;
  }

  char line2[24];
  char line3[24];
  char line4[24];
  char line5[24];
  char targetName[16];

  snprintf(line2, sizeof(line2), "State: %s", stateNameForDisplay(gPresenceState));

  if (gStrongestTargetIndex >= 0) {
    snprintf(line3, sizeof(line3), "RSSI: %d dBm", gStrongestRssi);
    buildTargetName(static_cast<uint8_t>(gStrongestTargetIndex), targetName, sizeof(targetName));
    snprintf(line4, sizeof(line4), "Target: %s", targetName);
  } else {
    copyCStringSafe(line3, sizeof(line3), "RSSI: --");
    copyCStringSafe(line4, sizeof(line4), "Target: none");
  }

  formatScanAge(line5, sizeof(line5));

  Heltec.display->clear();
  Heltec.display->setTextAlignment(TEXT_ALIGN_LEFT);
  Heltec.display->setFont(ArialMT_Plain_10);
  Heltec.display->drawString(0, 0, SENSOR_NAME);
  Heltec.display->drawString(0, 12, line2);
  Heltec.display->drawString(0, 24, line3);
  Heltec.display->drawString(0, 36, line4);
  Heltec.display->drawString(0, 48, line5);
  Heltec.display->display();
#endif
}

void logStatus() {
  Serial.printf(
    "SCAN ts_ms=%lu scan=%lu visible=%d matches=%u strongest_rssi=%d smoothed_rssi=%d state=%s reason=%s duration_ms=%lu uptime_s=%lu\n",
    static_cast<unsigned long>(gLastScanCompletedMs),
    static_cast<unsigned long>(gScanCounter),
    gVisibleNetworkCount,
    gMatchedCount,
    gStrongestRssi,
    gSmoothedRssi,
    stateNameForLog(gPresenceState),
    gStateReason,
    static_cast<unsigned long>(gLastScanDurationMs),
    static_cast<unsigned long>((millis() - gBootMs) / 1000UL)
  );

  if (gMatchedCount == 0) {
    Serial.println("MATCH none");
  } else {
    for (uint8_t i = 0; i < MAX_TARGETS; ++i) {
      if (!gMatchedTargets[i].matched) {
        continue;
      }

      char targetName[20];
      buildTargetName(i, targetName, sizeof(targetName));

      Serial.printf(
        "MATCH slot=%u name=%s via=%s ssid=\"%s\" bssid=%s ch=%d rssi=%d\n",
        i,
        targetName,
        gMatchedTargets[i].matchedByBssid ? "BSSID" : "SSID",
        gMatchedTargets[i].ssid,
        gMatchedTargets[i].bssid,
        gMatchedTargets[i].channel,
        gMatchedTargets[i].rssi
      );
    }
  }

  if (CALIBRATION_MODE) {
    Serial.println("CALIBRATION top_networks_begin");
    for (uint8_t i = 0; i < CALIBRATION_TOP_COUNT; ++i) {
      if (!gCalibrationTop[i].valid) {
        continue;
      }

      Serial.printf(
        "CAL slot=%u ssid=\"%s\" bssid=%s rssi=%d ch=%d\n",
        i + 1,
        gCalibrationTop[i].ssid,
        gCalibrationTop[i].bssid,
        gCalibrationTop[i].rssi,
        gCalibrationTop[i].channel
      );
    }
    Serial.println("CALIBRATION top_networks_end");
  }

  Serial.println();
}

void setup() {
  Heltec.begin(true, false, true);
  initDisplay();

  WiFi.mode(WIFI_STA);
  WiFi.disconnect();
  WiFi.setSleep(false);

  gBootMs = millis();
  gNextScanDueMs = 0;

  for (uint8_t i = 0; i < RSSI_WINDOW_SIZE; ++i) {
    gRssiWindow[i] = MISSING_RSSI_VALUE;
  }

  Serial.println();
  Serial.println("Room presence sensor boot");
  Serial.printf(
    "CONFIG sensor=%s scan_interval_ms=%lu present_threshold=%d absent_threshold=%d present_count=%u absent_count=%u window=%u calibration=%s\n",
    SENSOR_NAME,
    static_cast<unsigned long>(SCAN_INTERVAL_MS),
    PRESENT_RSSI_THRESHOLD,
    ABSENT_RSSI_THRESHOLD,
    REQUIRED_PRESENT_COUNT,
    REQUIRED_ABSENT_COUNT,
    RSSI_WINDOW_SIZE,
    CALIBRATION_MODE ? "true" : "false"
  );
  Serial.println("CONFIG note=BSSID_entries_override_SSID_entries_in_same_slot");
}

void loop() {
  uint32_t now = millis();
  if (static_cast<int32_t>(now - gNextScanDueMs) < 0) {
    return;
  }

  scanWiFiTargets();
  updatePresenceState();
  renderDisplay();
  logStatus();

  gNextScanDueMs = millis() + SCAN_INTERVAL_MS;
}
