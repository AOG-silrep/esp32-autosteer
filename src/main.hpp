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

#pragma once

#include <WiFi.h>
#include <WiFiMulti.h>
#include "SPIFFS.h"

#include <HTTPClient.h>

#include <AsyncUDP.h>

#include <ESPUI.h>
#include <ESP32CAN.h>

#include <Wire.h>

#include "average.hpp"

extern int8_t ditherAmount; // variable gets reset upon user changing dither

extern uint16_t labelLoad;
extern uint16_t labelWheelAngle;
extern uint16_t labelSafetyDisableAutosteer;
extern uint16_t labelSupplyVoltage;
extern uint16_t labelSteerMotorCurrent;
extern uint16_t labelSteerEngagedFaults;
extern uint16_t labelSwitchStates;

extern uint16_t labelStatusOutput;
extern uint16_t labelStatusAdc;
extern uint16_t labelStatusCanESP32;
extern uint16_t labelStatusCanMCP2515;
extern uint16_t manualValveSwitcher;
extern uint16_t manualValvePWMWidget;

extern SemaphoreHandle_t i2cMutex;
extern TaskHandle_t canReceiverHandle;
extern TaskHandle_t canSenderHandle;

struct Diagnostics {
  double steerSupplyVoltageMin;
  double steerSupplyVoltageMax;
  int8_t steerEnabledWithNoPower;
  int8_t fuse1Shorted;
  int8_t fuse2Shorted;
};
extern Diagnostics diagnostics;
///////////////////////////////////////////////////////////////////////////
// Configuration
///////////////////////////////////////////////////////////////////////////

struct Safety {
  bool AOGEnableAutosteerTimeout; // give AOG 1 second to return 'enabled' command, otherwise disable again
  bool autosteerDisabledByMaxEngageSpeed; // do not engage autosteer above an adjustable speed
  time_t timeoutPoint; // do not autosteer if no message from AOG
};
extern Safety safety;

struct Machine {
  bool disengagedBySteeringWheel = false; // disengaged by steering wheel > for diagnostic page
  bool canbusSteeringActive = false; // traditional or Canbus steering
  bool steeringEnabled = false; // ESP32 internal steering state > sent to AOG
  bool workswitchState = false; // current state of workswitch
  uint8_t canbusSteeringState = 0; // machine canbus steering state (receive only)
  uint16_t valveOutput; // output of the pid controller, after valve specific processing
  uint16_t canbusWasCounts; // Wheel Angle Sensor counts from canbus
  uint16_t handwheelPulseCount; // handwheel encoder counts
  uint16_t DeereDutyCycle; // duty cycle of Deere variable duty PWM disengage sensor
  uint16_t DeereDutyAverage; // average duty cycle of Deere variable duty PWM disengage sensor
  uint16_t steerMotorCurrent; // steering motor current
  double steerSupplyVoltage; // voltage from machine, also feeds the steering valve
  float wheelAngle; // wheel angle in degrees
  time_t lastCanbusSteeringMillis; // last time a CANBUS steering message was received
  time_t lastCanbusWasMillis; // last time a WAS CANBUS message was received
  time_t lastMCP2515CanbusMillis; // last time a MCP2515 CANBUS message was received
};
extern Machine machine;

struct SteerConfig {

  enum class AnalogIn : uint16_t {
    None                    = 0,
    ADS1115A0Single         = 100,
    ADS1115A1Single         = 101,
    ADS1115A0A1Differential = 200,
    JDVariableDuty          = 300,
    CanbusValtraMasseyChallenger = 400,
    CanbusCNH               = 401,
    CanbusFendt             = 402,
    CanbusJCB               = 403
  };

  enum class SpeedUnits : int8_t {
    MilesPerHour      = 0,
    KilometersPerHour = 1
  } speedUnits = SpeedUnits::MilesPerHour;

  enum class DisengageSwitchType : uint8_t {
    Encoder = 0,
    Hydraulic = 1,
    JDVariableDuty = 2,
    MotorCurrent = 3
  } disengageSwitchType = DisengageSwitchType::Encoder;

  char ssid[24] = "AOG hub";
  char password[24] = "password";
  char hostname[24] = "ESP32-Steer";
  uint8_t apModePin = 13;

  uint32_t baudrate = 115200;

  bool enableOTA = false;

  //set to 1  if you want to use Steering Motor + Cytron MD30C Driver
  //set to 2  if you want to use Steering Motor + IBT 2  Driver
  //set to 3  if you want to use IBT 2  Driver + PWM 2-Coil Valve
  //set to 4  if you want to use IBT 2  Driver + Danfoss Valve PVE A/H/M
  enum class OutputType : uint8_t {
    None                    = 0,
    SteeringMotorCytron     = 1,
    SteeringMotorIBT2       = 2,
    HydraulicPwm2Coil       = 3,
    HydraulicDanfoss        = 4,
    HydraulicBangBang       = 5,
    Canbus13_19Controller   = 6,
    CanbusF0_240Controller  = 7
  } outputType = OutputType::None;

  double pwmFrequency = 1000;
  bool invertOutput = false;
  float minAutosteerSpeed = 0.1;
  bool manualSteerState = false;
  int manualPWM = 0;
  uint8_t gpioPwm = 27;
  uint8_t gpioRight = 26;
  uint8_t gpioLeft = 25;
  uint8_t gpioEn = 33;

  double steeringPidKp = 20;
  double steeringPidKi = 0.5;
  double steeringPidKiMax = 128;
  double steeringPidKd = 1;
  uint8_t steeringPidMinPwm = 20;
  uint8_t steeringPidMaxPwm = 255;

  uint8_t dither = 0;

  enum class WorkswitchType : uint8_t {
    None                    = 0,
    Gpio                    = 1,
    RearHitchPosition       = 2,
    FrontHitchPosition      = 3,
    RearPtoRpm              = 4,
    FrontPtoRpm             = 5,
    MotorRpm                = 6
  } workswitchType = WorkswitchType::None;
  uint8_t gpioWorkswitch = 2;
  uint8_t gpioWorkLED = 14;
  uint8_t gpioSteerswitch = 15;
  uint8_t gpioSteerLED = 12;
  uint8_t gpioDisengage = 23;
  uint8_t gpioDisengagePullup = 32; // pullup driver for heavy duty disengage switches
  uint8_t gpioWASPulse = 39;
  uint8_t gpioSteerSupplyVoltage = 34;
  uint16_t disengageFramePulses = 3;
  uint16_t disengageFrameMillis = 1000;
  bool disengageHeavyDuty = false;
  uint16_t JDVariableDutyChange = 5;
  double steeringShuntVoltsPerAmp = 1.0;
  uint16_t maxSteerCurrent = 512;
  bool workswitchActiveLow = true;
  bool steerswitchActiveLow = true;
  bool steerSwitchIsMomentary = false;
  bool hydraulicSwitchActiveLow = true;

  enum class WheelAngleSensorType : uint8_t {
    WheelAngle = 0,
    TieRodDisplacement
  } wheelAngleSensorType = WheelAngleSensorType::WheelAngle;
  
  enum class ADSGain : uint16_t {
    GAIN_TWOTHIRDS = 0,
    GAIN_ONE = 512,
    GAIN_TWO = 1024,
    GAIN_FOUR = 1536,
    GAIN_EIGHT = 2048,
    GAIN_SIXTEEN = 2560
  } adsGain = ADSGain::GAIN_TWOTHIRDS;

  SteerConfig::AnalogIn wheelAngleInput = SteerConfig::AnalogIn::None;

  bool invertWheelAngleSensor = false;
  float wheelAngleCountsPerDegree = 118;
  uint16_t wheelAnglePositionZero = 5450;
  uint16_t ackermann = 100;
  bool ackermannAboveZero = true;

  float wheelAngleOffset = 0;

  float wheelAngleFirstArmLenght = 92;
  float wheelAngleSecondArmLenght = 308;
  float wheelAngleTieRodStroke = 210;
  float wheelAngleMinimumAngle = 37;
  float wheelAngleTrackArmLenght = 165;

  uint8_t gpioSDA = 21;
  uint8_t gpioSCL = 22;
  uint32_t i2cBusSpeed = 400000;

  bool canBusEnabled = false;
  uint8_t canBusRx = 18;
  uint8_t canBusTx = 19;
  enum class CanBusSpeed : uint16_t {
    Speed250kbs = 250,
    Speed500kbs = 500
  } canBusSpeed = CanBusSpeed::Speed250kbs;

  uint8_t canBusHitchThreshold = 50;
  uint8_t canBusHitchThresholdHysteresis = 6;

  uint16_t canBusRpmThreshold = 400;
  uint16_t canBusRpmThresholdHysteresis = 100;

  float maxAutosteerSpeed = 10;
  uint8_t gpioAlarm = 4;

  uint16_t aogPortSendFrom = 5577;
  uint16_t aogPortListenTo = 8888;
  uint16_t aogPortSendTo = 9999;

  bool retainWifiSettings = true;
};
extern SteerConfig steerConfig, steerConfigDefaults;

struct Initialisation {
  SteerConfig::OutputType outputType = SteerConfig::OutputType::None;
  SteerConfig::AnalogIn wheelAngleInput = SteerConfig::AnalogIn::None;

  uint16_t portSendFrom = 5577;
  uint16_t portListenTo = 8888;
  uint16_t portSendTo = 9999;

};
extern Initialisation initialisation;

///////////////////////////////////////////////////////////////////////////
// Global Data
///////////////////////////////////////////////////////////////////////////

struct SteerSettings {
  float Ko = 0.0f;  //overall gain
  float Kp = 0.0f;  //proportional gain
  float Ki = 0.0f;//integral gain
  float Kd = 0.0f;  //derivative gain
  uint8_t minPWMValue = 10;
  int maxIntegralValue = 20; //max PWM value for integral PID component
  float wheelAngleCountsPerDegree = 118;
  uint16_t wheelAnglePositionZero = 0;

  time_t lastPacketReceived = 0;
};
extern SteerSettings steerSettings;

struct SteerSetpoints {
  uint8_t relais = 0;
  float speed = 0;
  uint16_t distanceFromLine = 32020;
  double requestedSteerAngle = 0;

  bool enabled = false;
  float receivedRoll = 0;
  double actualSteerAngle = 0;
  double wheelAngleCounts = 0;
  double wheelAngleCurrentDisplacement = 0;
  double wheelAngleRaw = 0;
  double pidOutput = 0;
  float correction = 0;

  time_t lastPacketReceived = 0;
};
extern SteerSetpoints steerSetpoints;

struct SteerMachineControl {
  uint8_t pedalControl = 0;
  float speed = 0;
  uint8_t relais = 0;
  uint8_t youTurn = 0;

  time_t lastPacketReceived = 0;
};
extern SteerMachineControl steerMachineControl;

struct SteerCanData {
  float speed;
  uint16_t motorRpm;
  uint8_t frontHitchPosition;
  uint8_t rearHitchPosition;
  uint16_t frontPtoRpm;
  uint16_t rearPtoRpm;
};
extern SteerCanData steerCanData;

///////////////////////////////////////////////////////////////////////////
// external Libraries
///////////////////////////////////////////////////////////////////////////

extern AsyncUDP udpSendFrom;

///////////////////////////////////////////////////////////////////////////
// Helper Classes
///////////////////////////////////////////////////////////////////////////
extern portMUX_TYPE mux;
class TCritSect {
    TCritSect() {
      portENTER_CRITICAL( &mux );
    }
    ~TCritSect() {
      portEXIT_CRITICAL( &mux );
    }
};

///////////////////////////////////////////////////////////////////////////
// Helper Functions
///////////////////////////////////////////////////////////////////////////
extern void setResetButtonToRed();

extern void initESPUI();
extern void initIdleStats();
extern void initSensors();
extern void initCan();
extern void initAutosteer();
extern void initWiFi();
extern void initDiagnostics();
extern void showCanbusStateOnAOG();
