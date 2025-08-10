// MIT License
//
// Copyright (c) 2020 Christian Riggenbach
//
// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:
//
// The above copyright notice and this permission notice shall be included in all
// copies or substantial portions of the Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
// SOFTWARE.

#include <memory>

#include <FS.h>
#include <LittleFS.h>

#include "main.hpp"
#include "jsonFunctions.hpp"

void loadSavedDiagnostics() {
  {
    auto j = loadJsonFromFile( "/diagnostics.json" );
    parseJsonToDiagnostics( j, diagnostics );
  }
}

void saveDiagnostics() {
  {
    const auto j = parseDiagnosticsToJson( diagnostics );
    saveJsonToFile( j, "/diagnostics.json" );
  }
}

void loadSavedConfig() {
  {
    if( LittleFS.exists( "/autosteer.json" )) {
      auto j = loadJsonFromFile( "/autosteer.json" );
      parseJsonToSteerConfig( j, steerConfig );
    }
    else if( LittleFS.exists( "/config.json" )){ // import from old config
      auto j = loadJsonFromFile( "/config.json" );
      parseJsonToSteerConfig( j, steerConfig );
      saveJsonToFile( j, "/autosteer.json" ); // auto save file for next boot
      LittleFS.remove( "/config.json" ); // remove config.json ->
      //deprecated since it was a universal name with a risk of sharing wrong config to other PCBs
    }
    else{
      json j;
      parseJsonToSteerConfig( j, steerConfig );
    }
  }
}

void saveConfig() {
  {
    const auto j = parseSteerConfigToJson( steerConfig );
    saveJsonToFile( j, "/autosteer.json" );
  }
}

json loadJsonFromFile( const char* fileName ) {
  json j;

  if( LittleFS.exists( fileName )) {
    File file = LittleFS.open( fileName, "r" );

    if( file ) {
      std::vector<uint8_t> data;
      data.resize( file.size() );

      file.read( data.data(), file.size() );

      try {
        j = json::parse( data/*, nullptr, false*/ );
      } catch( json::exception& e ) {
        // output exception information
        Serial.print( "message: " );
        Serial.println( e.what() );
        Serial.print( "exception id: " );
        Serial.println( e.id );
      }
    } else {
      Serial.print( "Could not open file for reading: " );
      Serial.println( fileName );
      Serial.flush();
    }

    file.close();
  }

  return j;
}

void saveJsonToFile( const json& json, const char* fileName ) {
  // pretty print with 2 spaces indentation
  auto data = json.dump( 2 );

  File file = LittleFS.open( fileName, "w" );

  if( file && !file.isDirectory() ) {
    file.write( ( uint8_t* )data.c_str(), data.size() );
  } else {
    Serial.print( "Could not open file for writing: " );
    Serial.println( fileName );
    Serial.flush();
  }

  file.close();
}

json parseDiagnosticsToJson( const Diagnostics& diagnostics ) {
  json j;

  j["steerSupplyVoltageMax"] = diagnostics.steerSupplyVoltageMax;
  j["steerSupplyVoltageMin"] = diagnostics.steerSupplyVoltageMin;
  j["steerEnabledWithNoPower"] = diagnostics.steerEnabledWithNoPower;
  j["fuse1Shorted"] = diagnostics.fuse1Shorted;
  j["fuse2Shorted"] = diagnostics.fuse2Shorted;

  return j;

}

void parseJsonToDiagnostics( json& j, Diagnostics& diagnostics ) {
  if( j.is_object() ) {
    try {
      diagnostics.steerSupplyVoltageMax = j.value( "/steerSupplyVoltageMax"_json_pointer, 0 );
      diagnostics.steerSupplyVoltageMin = j.value( "/steerSupplyVoltageMin"_json_pointer, 0 );
      diagnostics.steerEnabledWithNoPower = j.value( "/steerEnabledWithNoPower"_json_pointer, 0 );
      diagnostics.fuse1Shorted = j.value( "/fuse1Shorted"_json_pointer, 0 );
      diagnostics.fuse2Shorted = j.value( "/fuse2Shorted"_json_pointer, 0 );

    } catch( json::exception& e ) {
      // output exception information
      Serial.print( "message: " );
      Serial.println( e.what() );
      Serial.print( "exception id: " );
      Serial.println( e.id );
      Serial.flush();
    }
  }
}

json parseSteerConfigToJson( const SteerConfig& config ) {
  json j;

  j["wifi"]["ssid"] = config.ssid;
  j["wifi"]["password"] = config.password;
  j["wifi"]["hostname"] = config.hostname;
  j["wifi"]["retainSettings"] = config.retainWifiSettings;

  j["output"]["type"] = int( config.outputType );
  j["output"]["pwmFrequency"] = config.pwmFrequency;
  j["output"]["minPWM"] = config.steeringPidMinPwm;
  j["output"]["maxPWM"] = config.steeringPidMaxPwm;
  j["output"]["invertOutput"] = config.invertOutput;
  j["output"]["dither"] = config.dither;
  j["output"]["minAutosteerSpeed"] = config.minAutosteerSpeed;

  j["PID"]["P"] = config.steeringPidKp;
  j["PID"]["I"] = config.steeringPidKi;
  j["PID"]["IMax"] = config.steeringPidKiMax;
  j["PID"]["D"] = config.steeringPidKd;

  j["workswitch"]["workswitchType"] = int( config.workswitchType );
  j["workswitch"]["workswitchActiveLow"] = config.workswitchActiveLow;
  j["workswitch"]["steerswitchActiveLow"] = config.steerswitchActiveLow;
  j["workswitch"]["steerSwitchIsMomentary"] = config.steerSwitchIsMomentary;
  j["workswitch"]["disengageSwitchType"] = int( config.disengageSwitchType );
  j["workswitch"]["hydraulicSwitchActiveLow"] = config.hydraulicSwitchActiveLow;
  j["workswitch"]["disengageFramePulses"] = config.disengageFramePulses;
  j["workswitch"]["disengageFrameMillis"] = config.disengageFrameMillis;
  j["workswitch"]["disengageHeavyDuty"] = config.disengageHeavyDuty;
  j["workswitch"]["JDVariableDutyChange"] = config.JDVariableDutyChange;
  j["workswitch"]["JDVariableDutyFrameLength"] = config.JDVariableDutyFrameLength;

  j["wheelangle"]["input"] = config.wheelAngleInput;
  j["wheelangle"]["sensorType"] = int( config.wheelAngleSensorType );
  j["wheelangle"]["invert"] = config.invertWheelAngleSensor;
  j["wheelangle"]["countsPerDegree"] = config.wheelAngleCountsPerDegree;
  j["wheelangle"]["positionZero"] = config.wheelAnglePositionZero;
  j["wheelangle"]["offset"] = config.wheelAngleOffset;
  j["wheelangle"]["ackermann"] = config.ackermann;
  j["wheelangle"]["ackermannAboveZero"] = config.ackermannAboveZero;
  j["wheelangle"]["adsGain"] = config.adsGain;

  j["wheelangle"]["tierod"]["FirstArmLenght"] = config.wheelAngleFirstArmLenght;
  j["wheelangle"]["tierod"]["SecondArmLenght"] = config.wheelAngleSecondArmLenght;
  j["wheelangle"]["tierod"]["TieRodStroke"] = config.wheelAngleTieRodStroke;
  j["wheelangle"]["tierod"]["MinimumAngle"] = config.wheelAngleMinimumAngle;
  j["wheelangle"]["tierod"]["TrackArmLenght"] = config.wheelAngleTrackArmLenght;

  j["canBus"]["enabled"] = config.canBusEnabled;
  j["canBus"]["speed"] = int( config.canBusSpeed );
  j["canBus"]["hitchThreshold"] = config.canBusHitchThreshold;
  j["canBus"]["hitchThresholdHysteresis"] = config.canBusHitchThresholdHysteresis;
  j["canBus"]["valveThreshold"] = config.canBusValveThreshold;
  j["canBus"]["valveThresholdHysteresis"] = config.canBusValveThresholdHysteresis;
  j["canBus"]["rpmThreshold"] = config.canBusRpmThreshold;
  j["canBus"]["rpmThresholdHysteresis"] = config.canBusRpmThresholdHysteresis;
  j["canBus"]["fendtEngageVersion"] = config.canBusFendtEngageVersion;
  j["canBus"]["hmsVersion"] = config.canbusHmsVersion;

  j["safety"]["maxAutosteerSpeed"] = config.maxAutosteerSpeed;
  j["safety"]["speedUnits"] = config.speedUnits;
  j["safety"]["steeringShuntVoltsPerAmp"] = config.steeringShuntVoltsPerAmp;
  j["safety"]["maxSteerCurrent"] = config.maxSteerCurrent;

  j["connection"]["baudrate"] = config.baudrate;
  j["connection"]["enableOTA"] = config.enableOTA;

  j["connection"]["aog"]["sendFrom"] = config.aogPortSendFrom;
  j["connection"]["aog"]["listenTo"] = config.aogPortListenTo;
  j["connection"]["aog"]["sendTo"] = config.aogPortSendTo;

  return j;
}

void parseJsonToSteerConfig( json& j, SteerConfig& config ) {
  if( j.is_object() ) {
    try {
      {
        std::string str = j.value( "/wifi/ssid"_json_pointer, steerConfigDefaults.ssid );
        memset( config.ssid, 0, sizeof( config.ssid ) );
        memcpy( config.ssid, str.c_str(), str.size() );
      }
      {
        std::string str = j.value( "/wifi/password"_json_pointer, steerConfigDefaults.password );
        memset( config.password, 0, sizeof( config.password ) );
        memcpy( config.password, str.c_str(), str.size() );
      }
      {
        std::string str = j.value( "/wifi/hostname"_json_pointer, steerConfigDefaults.hostname );
        memset( config.hostname, 0, sizeof( config.hostname ) );
        memcpy( config.hostname, str.c_str(), str.size() );
      }
      config.retainWifiSettings = j.value( "/wifi/retainSettings"_json_pointer, steerConfigDefaults.retainWifiSettings );

      config.outputType = j.value( "/output/type"_json_pointer, steerConfigDefaults.outputType );
      config.pwmFrequency = j.value( "/output/pwmFrequency"_json_pointer, steerConfigDefaults.pwmFrequency );
      config.steeringPidMinPwm = j.value( "/output/minPWM"_json_pointer, steerConfigDefaults.steeringPidMinPwm );
      config.steeringPidMaxPwm = j.value( "/output/maxPWM"_json_pointer, steerConfigDefaults.steeringPidMaxPwm );
      config.invertOutput = j.value( "/output/invertOutput"_json_pointer, steerConfigDefaults.invertOutput );
      config.dither = j.value( "/output/dither"_json_pointer, steerConfigDefaults.dither );
      config.minAutosteerSpeed = j.value( "/output/minAutosteerSpeed"_json_pointer, steerConfigDefaults.minAutosteerSpeed );

      config.steeringPidKp = j.value( "/PID/P"_json_pointer, steerConfigDefaults.steeringPidKp );
      config.steeringPidKi = j.value( "/PID/I"_json_pointer, steerConfigDefaults.steeringPidKi );
      config.steeringPidKiMax = j.value( "/PID/IMax"_json_pointer, steerConfigDefaults.steeringPidKiMax );
      config.steeringPidKd = j.value( "/PID/D"_json_pointer, steerConfigDefaults.steeringPidKd );

      config.workswitchType = j.value( "/workswitch/workswitchType"_json_pointer, steerConfigDefaults.workswitchType );
      config.workswitchActiveLow = j.value( "/workswitch/workswitchActiveLow"_json_pointer, steerConfigDefaults.workswitchActiveLow );
      config.steerswitchActiveLow = j.value( "/workswitch/steerswitchActiveLow"_json_pointer, steerConfigDefaults.steerswitchActiveLow );
      config.steerSwitchIsMomentary = j.value( "/workswitch/steerSwitchIsMomentary"_json_pointer, steerConfigDefaults.steerSwitchIsMomentary );
      config.disengageSwitchType = j.value( "/workswitch/disengageSwitchType"_json_pointer, steerConfigDefaults.disengageSwitchType );
      config.hydraulicSwitchActiveLow = j.value( "/workswitch/hydraulicSwitchActiveLow"_json_pointer, steerConfigDefaults.hydraulicSwitchActiveLow );
      config.disengageFramePulses = j.value( "/workswitch/disengageFramePulses"_json_pointer, steerConfigDefaults.disengageFramePulses );
      config.disengageFrameMillis = j.value( "/workswitch/disengageFrameMillis"_json_pointer, steerConfigDefaults.disengageFrameMillis );
      config.disengageHeavyDuty = j.value( "/workswitch/disengageHeavyDuty"_json_pointer, steerConfigDefaults.disengageHeavyDuty );
      config.JDVariableDutyChange = j.value( "/workswitch/JDVariableDutyChange"_json_pointer, steerConfigDefaults.JDVariableDutyChange );
      config.JDVariableDutyFrameLength = j.value( "/workswitch/JDVariableDutyFrameLength"_json_pointer, steerConfigDefaults.JDVariableDutyFrameLength );

      config.wheelAngleInput = j.value( "/wheelangle/input"_json_pointer, steerConfigDefaults.wheelAngleInput );
      config.wheelAngleSensorType = j.value( "/wheelangle/sensorType"_json_pointer, steerConfigDefaults.wheelAngleSensorType );
      config.invertWheelAngleSensor = j.value( "/wheelangle/invert"_json_pointer, steerConfigDefaults.invertWheelAngleSensor );
      config.wheelAngleCountsPerDegree = j.value( "/wheelangle/countsPerDegree"_json_pointer, steerConfigDefaults.wheelAngleCountsPerDegree );
      config.wheelAnglePositionZero = j.value( "/wheelangle/positionZero"_json_pointer, steerConfigDefaults.wheelAnglePositionZero );
      config.wheelAngleOffset = j.value( "/wheelangle/offset"_json_pointer, steerConfigDefaults.wheelAngleOffset );
      config.ackermann = j.value( "/wheelangle/ackermann"_json_pointer, steerConfigDefaults.ackermann );
      config.ackermannAboveZero = j.value( "/wheelangle/ackermannAboveZero"_json_pointer, steerConfigDefaults.ackermannAboveZero );
      config.adsGain = j.value( "/wheelangle/adsGain"_json_pointer, steerConfigDefaults.adsGain );

      config.wheelAngleFirstArmLenght = j.value( "/wheelangle/tierod/FirstArmLenght"_json_pointer, steerConfigDefaults.wheelAngleFirstArmLenght );
      config.wheelAngleSecondArmLenght = j.value( "/wheelangle/tierod/SecondArmLenght"_json_pointer, steerConfigDefaults.wheelAngleSecondArmLenght );
      config.wheelAngleTieRodStroke = j.value( "/wheelangle/tierod/TieRodStroke"_json_pointer, steerConfigDefaults.wheelAngleTieRodStroke );
      config.wheelAngleMinimumAngle = j.value( "/wheelangle/tierod/MinimumAngle"_json_pointer, steerConfigDefaults.wheelAngleMinimumAngle );
      config.wheelAngleTrackArmLenght = j.value( "/wheelangle/tierod/TrackArmLenght"_json_pointer, steerConfigDefaults.wheelAngleTrackArmLenght );

      config.canBusEnabled = j.value( "/canBus/enabled"_json_pointer, steerConfigDefaults.canBusEnabled );
      config.canBusSpeed = j.value( "/canBus/speed"_json_pointer, steerConfigDefaults.canBusSpeed );
      config.canBusHitchThreshold = j.value( "/canBus/hitchThreshold"_json_pointer, steerConfigDefaults.canBusHitchThreshold );
      config.canBusHitchThresholdHysteresis = j.value( "/canBus/hitchThresholdHysteresis"_json_pointer, steerConfigDefaults.canBusHitchThresholdHysteresis );
      config.canBusValveThreshold = j.value( "/canBus/valveThreshold"_json_pointer, steerConfigDefaults.canBusValveThreshold );
      config.canBusValveThresholdHysteresis = j.value( "/canBus/valveThresholdHysteresis"_json_pointer, steerConfigDefaults.canBusValveThresholdHysteresis );
      config.canBusRpmThreshold = j.value( "/canBus/rpmThreshold"_json_pointer, steerConfigDefaults.canBusRpmThreshold );
      config.canBusRpmThresholdHysteresis = j.value( "/canBus/rpmThresholdHysteresis"_json_pointer, steerConfigDefaults.canBusRpmThresholdHysteresis );
      config.canBusFendtEngageVersion = j.value( "/canBus/fendtEngageVersion"_json_pointer, steerConfigDefaults.canBusFendtEngageVersion );
      config.canbusHmsVersion = j.value( "/canBus/hmsVersion"_json_pointer, steerConfigDefaults.canbusHmsVersion );

      config.maxAutosteerSpeed = j.value( "/safety/maxAutosteerSpeed"_json_pointer, steerConfigDefaults.maxAutosteerSpeed );
      config.speedUnits = j.value( "/safety/speedUnits"_json_pointer, steerConfigDefaults.speedUnits );
      config.steeringShuntVoltsPerAmp = j.value( "/safety/steeringShuntVoltsPerAmp"_json_pointer, steerConfigDefaults.steeringShuntVoltsPerAmp );
      config.maxSteerCurrent = j.value( "/safety/maxSteerCurrent"_json_pointer, steerConfigDefaults.maxSteerCurrent );

      config.baudrate = j.value( "/connection/baudrate"_json_pointer, steerConfigDefaults.baudrate );
      config.enableOTA = j.value( "/connection/enableOTA"_json_pointer, steerConfigDefaults.enableOTA );

      config.aogPortSendFrom = j.value( "/connection/aog/sendFrom"_json_pointer, steerConfigDefaults.aogPortSendFrom );
      config.aogPortListenTo = j.value( "/connection/aog/listenTo"_json_pointer, steerConfigDefaults.aogPortListenTo );
      config.aogPortSendTo = j.value( "/connection/aog/sendTo"_json_pointer, steerConfigDefaults.aogPortSendTo );

    } catch( json::exception& e ) {
      // output exception information
      Serial.print( "message: " );
      Serial.println( e.what() );
      Serial.print( "exception id: " );
      Serial.println( e.id );
      Serial.flush();
    }
  }
}
