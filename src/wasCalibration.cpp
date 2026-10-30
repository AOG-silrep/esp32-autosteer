
#include <stdio.h>
#include <string.h>

#include <ESPUI.h>

#include "main.hpp"

// While autosteer holds a straight line steady, the wheels point straight ahead, so the
// WAS counts sampled then are the true center. AgOpenGPS's cross track error only gates
// which samples count, so its sign convention does not matter.

constexpr uint8_t SamplesPerSecond = 10;
constexpr uint16_t WindowSamples = 3 * SamplesPerSecond;
constexpr uint32_t MinPassSamples = 15 * SamplesPerSecond; // steady tracking needed for a pass to count
constexpr uint16_t PassGapSamples = 2 * SamplesPerSecond; // a pass ends after this long outside the gate
constexpr uint8_t MaxPasses = 8;
constexpr int16_t MaxXteCm = 50;
constexpr int16_t MaxWindowXteChangeCm = 4;
constexpr float MaxStraightAngle = 5.0;
constexpr float MaxApplyDegrees = 5.0; // stay below the 5 degree WAS plausibility check in sensor.cpp
constexpr float GoodSpreadDegrees = 0.3;

static const char* const XteNotSteady = "Cross track error not steady yet";

struct WasCalibrationPass {
  float meanCounts;
  float meanDegrees; // pre-Ackermann wheel angle
  uint16_t seconds;
};

struct WasCalibration {
  bool running = false;
  const char* gateReason = nullptr; // why samples are being rejected, nullptr while collecting
  char message[120] = "";

  uint16_t windowSamples = 0;
  int16_t windowXteMin = 0;
  int16_t windowXteMax = 0;
  double windowCountsSum = 0;
  double windowDegreesSum = 0;

  uint16_t gapSamples = 0;
  uint32_t passSamples = 0;
  double passCountsSum = 0;
  double passDegreesSum = 0;

  uint8_t passCount = 0;
  WasCalibrationPass passes[MaxPasses];

  // settings the passes were measured against
  uint16_t center = 0;
  float offset = 0;
  float countsPerDegree = 0;
  bool invert = false;
};
static WasCalibration wasCalibration;

static bool settingsChangedSinceStart() {
  return wasCalibration.center != steerConfig.wheelAnglePositionZero ||
         wasCalibration.offset != steerConfig.wheelAngleOffset ||
         wasCalibration.countsPerDegree != steerConfig.wheelAngleCountsPerDegree ||
         wasCalibration.invert != steerConfig.invertWheelAngleSensor;
}

static bool appliesAsOffset() {
  // tie rod geometry and Fendt are not straight ahead at the center counts, so correct them with the offset
  return steerConfig.wheelAngleSensorType == SteerConfig::WheelAngleSensorType::TieRodDisplacement ||
         steerConfig.wheelAngleInput == SteerConfig::AnalogIn::CanbusFendt;
}

String wasCalibrationNudgeHint() {
  // holding left of the line means AgOpenGPS steadily commands right (positive), so the sensor reads
  // too high when straight; raising the offset, or the center of a non inverted sensor, lowers it
  if( appliesAsOffset() ){
    return "Left of the line: raise Wheel Angle Offset<br>Right of the line: lower Wheel Angle Offset";
  }
  if( steerConfig.invertWheelAngleSensor ){
    return "Left of the line: lower Wheel Angle Sensor Center<br>Right of the line: raise Wheel Angle Sensor Center";
  }
  return "Left of the line: raise Wheel Angle Sensor Center<br>Right of the line: lower Wheel Angle Sensor Center";
}

static long centerFromCounts( float meanCounts ) {
  // keep the configured offset, so the sensor reads zero when driving straight
  float sign = steerConfig.invertWheelAngleSensor ? -1 : 1;
  return lroundf( meanCounts - sign * steerConfig.wheelAngleOffset * steerConfig.wheelAngleCountsPerDegree );
}

enum class WasCalibrationConfidence { Good, NeedMorePasses, PassesDisagree };

struct WasCalibrationResult {
  float meanCounts;
  float meanDegrees;
  float spreadDegrees; // largest spread within one direction
  float slopeDegrees; // half the difference between the directions
  uint8_t group[MaxPasses]; // 0 or 1, the direction each pass was sorted into
  WasCalibrationConfidence confidence;
};

// Side slope or a slightly curved line makes the tractor steer uphill to hold the line, which
// flips sign with the driving direction while the WAS center error does not. No heading reaches
// the module, so passes are split into two directions at the largest gap between their angles,
// and the directions are weighted equally so the slope cancels.
static bool calculateResult( const WasCalibration& calibration, WasCalibrationResult& result ) {
  uint8_t count = calibration.passCount;
  if( count == 0 ){
    return false;
  }
  uint8_t order[MaxPasses];
  for( uint8_t i = 0; i < count; i++ ){
    order[i] = i;
    for( uint8_t j = i; j > 0 && calibration.passes[order[j - 1]].meanDegrees > calibration.passes[order[j]].meanDegrees; j-- ){
      uint8_t swap = order[j];
      order[j] = order[j - 1];
      order[j - 1] = swap;
    }
  }

  // the first pass of the second direction in sorted order, count when all passes agree
  uint8_t split = count;
  if( calibration.passes[order[count - 1]].meanDegrees - calibration.passes[order[0]].meanDegrees >= GoodSpreadDegrees ){
    float largestGap = 0;
    for( uint8_t i = 1; i < count; i++ ){
      float gap = calibration.passes[order[i]].meanDegrees - calibration.passes[order[i - 1]].meanDegrees;
      if( gap > largestGap ){
        largestGap = gap;
        split = i;
      }
    }
  }

  float groupCounts[2] = { 0, 0 };
  float groupDegrees[2] = { 0, 0 };
  uint8_t groupSize[2] = { 0, 0 };
  for( uint8_t i = 0; i < count; i++ ){
    uint8_t group = i < split ? 0 : 1;
    result.group[order[i]] = group;
    groupCounts[group] += calibration.passes[order[i]].meanCounts;
    groupDegrees[group] += calibration.passes[order[i]].meanDegrees;
    groupSize[group]++;
  }
  groupCounts[0] /= groupSize[0];
  groupDegrees[0] /= groupSize[0];
  result.spreadDegrees = calibration.passes[order[split - 1]].meanDegrees - calibration.passes[order[0]].meanDegrees;

  if( split == count ){
    result.meanCounts = groupCounts[0];
    result.meanDegrees = groupDegrees[0];
    result.slopeDegrees = 0;
    result.confidence = count < 2 ? WasCalibrationConfidence::NeedMorePasses : WasCalibrationConfidence::Good;
    return true;
  }

  groupCounts[1] /= groupSize[1];
  groupDegrees[1] /= groupSize[1];
  result.spreadDegrees = max( result.spreadDegrees,
                              calibration.passes[order[count - 1]].meanDegrees - calibration.passes[order[split]].meanDegrees );
  result.meanCounts = ( groupCounts[0] + groupCounts[1] ) / 2;
  result.meanDegrees = ( groupDegrees[0] + groupDegrees[1] ) / 2;
  result.slopeDegrees = ( groupDegrees[1] - groupDegrees[0] ) / 2;
  if( result.spreadDegrees >= GoodSpreadDegrees ){
    result.confidence = WasCalibrationConfidence::PassesDisagree;
  } else if( groupSize[0] < 2 || groupSize[1] < 2 ){
    // a single pass per direction cannot tell slope from a pass that went wrong
    result.confidence = WasCalibrationConfidence::NeedMorePasses;
  } else {
    result.confidence = WasCalibrationConfidence::Good;
  }
  return true;
}

static const char* gateReason() {
  if( millis() - steerSetpoints.lastPacketReceived > 1000 ){
    return "No data from AgOpenGPS";
  }
  if( steerSetpoints.xteValid == false ){
    return "No cross track error from AgOpenGPS (no guidance line, or autosteer off)";
  }
  if( steerSetpoints.enabled == false || machine.autosteerSafetyLock == true ){
    return "Autosteer not engaged";
  }
  if( steerConfig.manualSteerState == true ){
    return "Manual valve control is on";
  }
  if( steerSetpoints.rowSenseControl == true ){
    return "Row sense is steering";
  }
  if( steerSetpoints.speed < steerConfig.minAutosteerSpeed ){
    return "Below minimum autosteer speed";
  }
  if( abs( steerSetpoints.xteCm ) > MaxXteCm ){
    return "More than 50 cm off the line";
  }
  if( fabs( steerSetpoints.actualSteerAngle ) > MaxStraightAngle ){
    return "Wheel angle above 5°, not driving straight";
  }
  return nullptr;
}

static void closePass() {
  if( wasCalibration.passSamples >= MinPassSamples ){
    if( wasCalibration.passCount == MaxPasses ){ // keep the latest passes
      memmove( &wasCalibration.passes[0], &wasCalibration.passes[1], sizeof( wasCalibration.passes[0] ) * ( MaxPasses - 1 ) );
      wasCalibration.passCount--;
    }
    WasCalibrationPass& pass = wasCalibration.passes[wasCalibration.passCount++];
    pass.meanCounts = wasCalibration.passCountsSum / wasCalibration.passSamples;
    pass.meanDegrees = wasCalibration.passDegreesSum / wasCalibration.passSamples;
    pass.seconds = wasCalibration.passSamples / SamplesPerSecond;
  }
  wasCalibration.passSamples = 0;
  wasCalibration.passCountsSum = 0;
  wasCalibration.passDegreesSum = 0;
}

static void sampleWasCalibration() {
  const char* reason = gateReason();
  if( reason != nullptr ){
    wasCalibration.gateReason = reason;
    wasCalibration.windowSamples = 0; // a window must be continuous
    if( wasCalibration.gapSamples < PassGapSamples ){
      wasCalibration.gapSamples++;
    } else {
      closePass();
    }
    return;
  }
  wasCalibration.gapSamples = 0;
  if( wasCalibration.gateReason != XteNotSteady ){ // keep showing it until a window is accepted
    wasCalibration.gateReason = nullptr;
  }

  int16_t xte = steerSetpoints.xteCm;
  if( wasCalibration.windowSamples == 0 ){
    wasCalibration.windowXteMin = xte;
    wasCalibration.windowXteMax = xte;
    wasCalibration.windowCountsSum = 0;
    wasCalibration.windowDegreesSum = 0;
  }
  wasCalibration.windowXteMin = min( wasCalibration.windowXteMin, xte );
  wasCalibration.windowXteMax = max( wasCalibration.windowXteMax, xte );
  wasCalibration.windowCountsSum += steerSetpoints.wheelAngleCounts;
  wasCalibration.windowDegreesSum += steerSetpoints.wheelAnglePreAckermann;

  if( ++wasCalibration.windowSamples < WindowSamples ){
    return;
  }
  wasCalibration.windowSamples = 0;
  if( wasCalibration.windowXteMax - wasCalibration.windowXteMin > MaxWindowXteChangeCm ){
    wasCalibration.gateReason = XteNotSteady;
    return;
  }
  wasCalibration.gateReason = nullptr;
  wasCalibration.passSamples += WindowSamples;
  wasCalibration.passCountsSum += wasCalibration.windowCountsSum;
  wasCalibration.passDegreesSum += wasCalibration.windowDegreesSum;
}

void wasCalibrationWorker10Hz( void* z ) {
  constexpr TickType_t xFrequency = 1000 / SamplesPerSecond;
  TickType_t xLastWakeTime = xTaskGetTickCount();

  for( ;; ) {
    if( xSemaphoreTake( wasCalibrationMutex, pdMS_TO_TICKS( 50 )) == pdTRUE ) {
      if( wasCalibration.running ){
        sampleWasCalibration();
      }
      xSemaphoreGive( wasCalibrationMutex );
    }
    vTaskDelayUntil( &xLastWakeTime, xFrequency );
  }
}

void wasCalibrationStart() {
  if( xSemaphoreTake( wasCalibrationMutex, pdMS_TO_TICKS( 100 )) == pdTRUE ) {
    wasCalibration = WasCalibration();
    wasCalibration.running = true;
    wasCalibration.gateReason = "Starting";
    wasCalibration.center = steerConfig.wheelAnglePositionZero;
    wasCalibration.offset = steerConfig.wheelAngleOffset;
    wasCalibration.countsPerDegree = steerConfig.wheelAngleCountsPerDegree;
    wasCalibration.invert = steerConfig.invertWheelAngleSensor;
    strcpy( wasCalibration.message, "Calibration started" );
    xSemaphoreGive( wasCalibrationMutex );
  }
}

void wasCalibrationReset() {
  if( xSemaphoreTake( wasCalibrationMutex, pdMS_TO_TICKS( 100 )) == pdTRUE ) {
    wasCalibration = WasCalibration();
    strcpy( wasCalibration.message, "Calibration reset" );
    xSemaphoreGive( wasCalibrationMutex );
  }
}

bool wasCalibrationApply() {
  if( xSemaphoreTake( wasCalibrationMutex, pdMS_TO_TICKS( 100 )) != pdTRUE ) {
    return false;
  }
  bool applied = false;
  char message[sizeof( wasCalibration.message )];
  WasCalibrationResult result;
  bool haveResult = calculateResult( wasCalibration, result );
  float meanCounts = result.meanCounts;
  float meanDegrees = result.meanDegrees;
  // the step the measured angle makes when applied, including Ackermann scaling
  float stepDegrees = haveResult ? fabs( meanDegrees ) * max( ( int )steerConfig.ackermann, 100 ) / 100 : 0;

  if( haveResult == false ){
    strcpy( message, "No completed passes to apply" );
  } else if( settingsChangedSinceStart() ){
    strcpy( message, "Wheel angle settings changed during the run, press Reset and drive the passes again" );
  } else if( stepDegrees >= MaxApplyDegrees ){
    snprintf( message, sizeof( message ), "A %.2f° step is too large to apply, check the sensor and set it by hand", stepDegrees );
  } else if( appliesAsOffset() ){
    float previous = steerConfig.wheelAngleOffset;
    steerConfig.wheelAngleOffset = previous + meanDegrees;
    ESPUI.updateControlValue( numberWasOffset, String( steerConfig.wheelAngleOffset ) );
    snprintf( message, sizeof( message ), "Wheel Angle Offset changed from %.2f° to %.2f°", previous, steerConfig.wheelAngleOffset );
    applied = true;
  } else {
    long center = centerFromCounts( meanCounts );
    if( center < 0 || center > 65535 ){
      snprintf( message, sizeof( message ), "Calculated center %ld is outside 0 to 65535", center );
    } else {
      uint16_t previous = steerConfig.wheelAnglePositionZero;
      steerConfig.wheelAnglePositionZero = center;
      ESPUI.updateNumber( numberWasCenter, center );
      snprintf( message, sizeof( message ), "Wheel Angle Sensor Center changed from %u to %ld (%.2f°)", previous, center, meanDegrees );
      applied = true;
    }
  }

  if( applied ){
    saveConfigAfterDelay();
    wasCalibration = WasCalibration(); // passes were measured against the old setting
  }
  strcpy( wasCalibration.message, message );
  xSemaphoreGive( wasCalibrationMutex );
  return applied;
}

String wasCalibrationJson() {
  WasCalibration snapshot;
  if( xSemaphoreTake( wasCalibrationMutex, pdMS_TO_TICKS( 100 )) != pdTRUE ) {
    return String();
  }
  snapshot = wasCalibration; // copy under the lock, format outside it
  xSemaphoreGive( wasCalibrationMutex );

  String json;
  json.reserve( 1000 );
  json = "{\"running\":";
  json += snapshot.running ? "true" : "false";
  json += ",\"collecting\":";
  json += ( snapshot.running && snapshot.gateReason == nullptr ) ? "true" : "false";
  json += ",\"status\":\"";
  if( snapshot.running == false ){
    json += "Not running";
  } else if( snapshot.gateReason == nullptr ){
    json += "Collecting";
  } else {
    json += snapshot.gateReason;
  }
  json += "\",\"message\":\"";
  json += snapshot.message;
  json += "\",\"xteValid\":";
  json += steerSetpoints.xteValid ? "true" : "false";
  json += ",\"xte\":";
  json += steerSetpoints.xteCm;
  json += ",\"angle\":";
  json += String( steerSetpoints.actualSteerAngle, 2 );
  json += ",\"speed\":";
  json += String( steerSetpoints.speed, 1 );
  json += ",\"speedUnits\":\"";
  json += ( steerConfig.speedUnits == SteerConfig::SpeedUnits::MilesPerHour ) ? "MPH" : "KPH";
  json += "\",\"passSeconds\":";
  json += snapshot.passSamples / SamplesPerSecond;
  json += ",\"passTarget\":";
  json += MinPassSamples / SamplesPerSecond;

  WasCalibrationResult result;
  bool haveResult = calculateResult( snapshot, result );
  json += ",\"passes\":[";
  for( uint8_t i = 0; i < snapshot.passCount; i++ ){
    if( i > 0 ){
      json += ",";
    }
    json += "{\"seconds\":";
    json += snapshot.passes[i].seconds;
    json += ",\"counts\":";
    json += String( snapshot.passes[i].meanCounts, 1 );
    json += ",\"degrees\":";
    json += String( snapshot.passes[i].meanDegrees, 2 );
    json += ",\"direction\":";
    json += result.group[i] + 1;
    json += "}";
  }
  json += "],\"applyAsOffset\":";
  json += appliesAsOffset() ? "true" : "false";

  if( haveResult ){
    json += ",\"error\":";
    json += String( result.meanDegrees, 2 );
    json += ",\"spread\":";
    json += String( result.spreadDegrees, 2 );
    json += ",\"slope\":";
    json += String( result.slopeDegrees, 2 );
    json += ",\"confidence\":\"";
    switch( result.confidence ){
      case WasCalibrationConfidence::Good: json += "good"; break;
      case WasCalibrationConfidence::NeedMorePasses: json += "needMorePasses"; break;
      case WasCalibrationConfidence::PassesDisagree: json += "passesDisagree"; break;
    }
    json += "\",\"newValue\":";
    if( appliesAsOffset() ){
      json += String( steerConfig.wheelAngleOffset + result.meanDegrees, 2 );
    } else {
      json += centerFromCounts( result.meanCounts );
    }
  }
  json += "}";
  return json;
}

void initWasCalibration() {
  xTaskCreate( wasCalibrationWorker10Hz, "wasCalibrationWorker", 3096, NULL, 2, NULL );
}
