
#include <stdio.h>

#include <ESPUI.h>
#include <driver/twai.h>  // Native ESP-IDF TWAI driver

#include "main.hpp"
#include "jsonFunctions.hpp"

int8_t ditherAmount = 0;
uint16_t labelLoad;
uint16_t labelWheelAngle;
uint16_t labelRowSense;
uint16_t buttonReset;

uint16_t labelWheelAngleDisplacement;

uint16_t labelStatusOutput;
uint16_t labelStatusCanESP32;
uint16_t labelStatusCanMCP2515;
uint16_t labelBuildDate;
uint16_t manualValveSwitcher;
uint16_t manualValvePWMWidget;
char downloadFilename[50];

void manualValveCallback(Control *sender, int type);
void manualValvePWMCallback(Control *sender, int type);

void setResetButtonToRed() {
  ESPUI.getControl( buttonReset )->color = ControlColor::Alizarin;
  ESPUI.updateControl( buttonReset );
  ESPUI.setPanelStyle( buttonReset, "display: block;" );
}

void saveConfigAfterDelay() {
  xTimerStop( saveTimer, 0 );
  xTimerStart( saveTimer, 0 );
}

void addWasInputSelection( uint16_t parent ) {
  ESPUI.addControl( ControlType::Option, "ADS1115 A0 Single", String( ( uint8_t )SteerConfig::AnalogIn::ADS1115A0Single ), ControlColor::Alizarin, parent );
  ESPUI.addControl( ControlType::Option, "ADS1115 A1 Single", String( ( uint8_t )SteerConfig::AnalogIn::ADS1115A1Single ), ControlColor::Alizarin, parent );
  ESPUI.addControl( ControlType::Option, "ADS1115 A0/A1 Differential", String( ( uint8_t )SteerConfig::AnalogIn::ADS1115A0A1Differential ), ControlColor::Alizarin, parent );
  ESPUI.addControl( ControlType::Option, "Deere Variable Duty Cycle", String( ( uint16_t )SteerConfig::AnalogIn::JDVariableDuty ), ControlColor::Alizarin, parent );
  if( steerConfig.canBusEnabled ) {
    ESPUI.addControl( ControlType::Option, "Canbus: Valtra-Massey-Challenger", String( ( uint16_t )SteerConfig::AnalogIn::CanbusValtraMasseyChallenger ), ControlColor::Alizarin, parent );
    ESPUI.addControl( ControlType::Option, "Canbus: Fendt", String( ( uint16_t )SteerConfig::AnalogIn::CanbusFendt ), ControlColor::Alizarin, parent );
  }
}
  
void initESPUI ( void ) {

  labelLoad = ESPUI.addControl( ControlType::Label, "Status", "", ControlColor::Turquoise );
  labelWheelAngle = ESPUI.addControl( ControlType::Label, "Wheel Angle:", "0°", ControlColor::Emerald );

  buttonReset = ESPUI.addControl( ControlType::Button, "If this turns red, you have to", "Apply & Reboot", ControlColor::Emerald, Control::noParent,
  []( Control * control, int id ) {
    if( id == B_UP ) {
      saveConfig();
      LittleFS.end();
      if( steerConfig.canBusEnabled ){
        twai_stop();
      }
      ESP.restart();
    }
  } );

  uint16_t tabConfigurations;

  // Network Tab
  {
    uint16_t tab = ESPUI.addControl( ControlType::Tab, "Network", "Network" );

    ESPUI.addControl( ControlType::Text, "SSID*", String( steerConfig.ssid ), ControlColor::Wetasphalt, tab,
    []( Control * control, int id ) {
      control->value.toCharArray( steerConfig.ssid, sizeof( steerConfig.ssid ) );
      setResetButtonToRed();
    } );
    ESPUI.addControl( ControlType::Text, "Password*", String( steerConfig.password ), ControlColor::Wetasphalt, tab,
    []( Control * control, int id ) {
      control->value.toCharArray( steerConfig.password, sizeof( steerConfig.password ) );
      setResetButtonToRed();
    } );
    ESPUI.addControl( ControlType::Text, "Hostname*", String( steerConfig.hostname ), ControlColor::Wetasphalt, tab,
    []( Control * control, int id ) {
      control->value.toCharArray( steerConfig.hostname, sizeof( steerConfig.hostname ) );
      setResetButtonToRed();
    } );

  }

  // CAN Bus
  {
    uint16_t tab = ESPUI.addControl( ControlType::Tab, "CAN Bus/J1939", "CAN Bus/J1939" );

    ESPUI.addControl( ControlType::Switcher, "CAN Bus Enabled*", steerConfig.canBusEnabled ? "1" : "0", ControlColor::Wetasphalt, tab,
    []( Control * control, int id ) {
      if( steerConfig.canBusEnabled ){
        if( canReceiverHandle ){
          vTaskDelete( canReceiverHandle );
          canReceiverHandle = NULL;
        }
        if( canSenderHandle ){
          vTaskDelete( canSenderHandle );
          canSenderHandle = NULL;
        }
        twai_stop();
      }
      steerConfig.canBusEnabled = control->value.toInt() == 1;
      setResetButtonToRed();
    } );

    {
      uint16_t sel = ESPUI.addControl( ControlType::Select, "Bus Speed*", String( ( int )steerConfig.canBusSpeed ), ControlColor::Wetasphalt, tab,
      []( Control * control, int id ) {
        steerConfig.canBusSpeed = ( SteerConfig::CanBusSpeed )control->value.toInt();
        setResetButtonToRed();
      } );
      ESPUI.addControl( ControlType::Option, "250kB/s", "250", ControlColor::Alizarin, sel );
      ESPUI.addControl( ControlType::Option, "500kB/s", "500", ControlColor::Alizarin, sel );
    }

    if( steerConfig.canBusEnabled ){
      labelStatusCanESP32 = ESPUI.addControl( ControlType::Label, "ESP32 CAN:", "No CAN BUS configured", ControlColor::Turquoise, tab );
      labelStatusCanMCP2515 = ESPUI.addControl( ControlType::Label, "MCP2515 CAN:", "No CAN BUS configured", ControlColor::Turquoise, tab );

      {
        uint16_t sel = ESPUI.addControl( ControlType::Select, "HMS version*", String( ( int )steerConfig.canbusHmsVersion ), ControlColor::Peterriver, tab,
        []( Control * control, int id ) {
          steerConfig.canbusHmsVersion = ( SteerConfig::HmsVersion )control->value.toInt();
          saveConfigAfterDelay();
        } );
        ESPUI.addControl( ControlType::Option, "None", "0", ControlColor::Alizarin, sel );
        ESPUI.addControl( ControlType::Option, "Deere 0x18FFFA21", "1", ControlColor::Alizarin, sel );
      }
    }
  }

  // Switches/Buttons Tab
  {
    uint16_t tab = ESPUI.addControl( ControlType::Tab, "Work- and Steerswitch", "Work- and Steerswitch" );

    uint16_t sel = ESPUI.addControl( ControlType::Select, "Workswitch Type", String( ( int )steerConfig.workswitchType ), ControlColor::Peterriver, tab,
      []( Control * control, int id ) {
        steerConfig.workswitchType = ( SteerConfig::WorkswitchType )control->value.toInt();
        saveConfigAfterDelay();
      } );
      ESPUI.addControl( ControlType::Option, "None", "0", ControlColor::Alizarin, sel );
      ESPUI.addControl( ControlType::Option, "Gpio", "1", ControlColor::Alizarin, sel );

    if( steerConfig.canBusEnabled ) {
      {
        ESPUI.addControl( ControlType::Option, "Rear Hitch Position (from Can Bus)", "2", ControlColor::Alizarin, sel );
        ESPUI.addControl( ControlType::Option, "Front Hitch Position (from Can Bus)", "3", ControlColor::Alizarin, sel );
        ESPUI.addControl( ControlType::Option, "Rear Pto Rpm (from Can Bus)", "4", ControlColor::Alizarin, sel );
        ESPUI.addControl( ControlType::Option, "Front Pto Rpm (from Can Bus)", "5", ControlColor::Alizarin, sel );
        ESPUI.addControl( ControlType::Option, "Motor Rpm (from Can Bus)", "6", ControlColor::Alizarin, sel );
        ESPUI.addControl( ControlType::Option, "SCV 1 (from Can Bus)", "7", ControlColor::Alizarin, sel );
        ESPUI.addControl( ControlType::Option, "SCV 2 (from Can Bus)", "8", ControlColor::Alizarin, sel );
        ESPUI.addControl( ControlType::Option, "SCV 3 (from Can Bus)", "9", ControlColor::Alizarin, sel );
      }

      {
        uint16_t num = ESPUI.addControl( ControlType::Number, "Hitch Threshold", String( steerConfig.canBusHitchThreshold ), ControlColor::Peterriver, tab,
        []( Control * control, int id ) {
          steerConfig.canBusHitchThreshold = control->value.toInt();
          saveConfigAfterDelay();
        } );
        ESPUI.addControl( ControlType::Min, "Min", "0", ControlColor::Peterriver, num );
        ESPUI.addControl( ControlType::Max, "Max", "100", ControlColor::Peterriver, num );
        ESPUI.addControl( ControlType::Step, "Step", "1", ControlColor::Peterriver, num );
      }
      {
        uint16_t num = ESPUI.addControl( ControlType::Number, "Hitch Threshold Hysteresis", String( steerConfig.canBusHitchThresholdHysteresis ), ControlColor::Peterriver, tab,
        []( Control * control, int id ) {
          steerConfig.canBusHitchThresholdHysteresis = control->value.toInt();
          saveConfigAfterDelay();
        } );
        ESPUI.addControl( ControlType::Min, "Min", "0", ControlColor::Peterriver, num );
        ESPUI.addControl( ControlType::Max, "Max", "100", ControlColor::Peterriver, num );
        ESPUI.addControl( ControlType::Step, "Step", "1", ControlColor::Peterriver, num );
      }

      {
        uint16_t num = ESPUI.addControl( ControlType::Number, "SCV Threshold", String( steerConfig.canBusValveThreshold ), ControlColor::Peterriver, tab,
        []( Control * control, int id ) {
          steerConfig.canBusValveThreshold = control->value.toInt();
          saveConfigAfterDelay();
        } );
        ESPUI.addControl( ControlType::Min, "Min", "0", ControlColor::Peterriver, num );
        ESPUI.addControl( ControlType::Max, "Max", "255", ControlColor::Peterriver, num );
        ESPUI.addControl( ControlType::Step, "Step", "1", ControlColor::Peterriver, num );
      }
      
      {
        uint16_t num = ESPUI.addControl( ControlType::Number, "SCV Hysteresis", String( steerConfig.canBusValveThresholdHysteresis ), ControlColor::Peterriver, tab,
        []( Control * control, int id ) {
          steerConfig.canBusValveThresholdHysteresis = control->value.toInt();
          saveConfigAfterDelay();
        } );
        ESPUI.addControl( ControlType::Min, "Min", "0", ControlColor::Peterriver, num );
        ESPUI.addControl( ControlType::Max, "Max", "255", ControlColor::Peterriver, num );
        ESPUI.addControl( ControlType::Step, "Step", "1", ControlColor::Peterriver, num );
      }

      {
        uint16_t num = ESPUI.addControl( ControlType::Number, "RPM Threshold", String( steerConfig.canBusRpmThreshold ), ControlColor::Peterriver, tab,
        []( Control * control, int id ) {
          steerConfig.canBusRpmThreshold = control->value.toInt();
          saveConfigAfterDelay();
        } );
        ESPUI.addControl( ControlType::Min, "Min", "0", ControlColor::Peterriver, num );
        ESPUI.addControl( ControlType::Max, "Max", "3500", ControlColor::Peterriver, num );
        ESPUI.addControl( ControlType::Step, "Step", "1", ControlColor::Peterriver, num );
      }
      {
        uint16_t num = ESPUI.addControl( ControlType::Number, "RPM Threshold Hysteresis", String( steerConfig.canBusRpmThresholdHysteresis ), ControlColor::Peterriver, tab,
        []( Control * control, int id ) {
          steerConfig.canBusRpmThresholdHysteresis = control->value.toInt();
          saveConfigAfterDelay();
        } );
        ESPUI.addControl( ControlType::Min, "Min", "0", ControlColor::Peterriver, num );
        ESPUI.addControl( ControlType::Max, "Max", "1000", ControlColor::Peterriver, num );
        ESPUI.addControl( ControlType::Step, "Step", "1", ControlColor::Peterriver, num );
      }
    }

    {
      ESPUI.addControl( ControlType::Switcher, "Workswitch Active Low", steerConfig.workswitchActiveLow ? "1" : "0", ControlColor::Peterriver, tab,
      []( Control * control, int id ) {
        steerConfig.workswitchActiveLow = control->value.toInt() == 1;
        saveConfigAfterDelay();
      } );
    }

    {
      ESPUI.addControl( ControlType::Switcher, "Autosteer Switch Active Low", steerConfig.steerswitchActiveLow ? "1" : "0", ControlColor::Peterriver, tab,
      []( Control * control, int id ) {
        steerConfig.steerswitchActiveLow = control->value.toInt() == 1;
        saveConfigAfterDelay();
      } );
    }

    if( steerConfig.outputType < SteerConfig::OutputType::Canbus13_19Controller ){
      ESPUI.addControl( ControlType::Switcher, "Autosteer Switch is Momentary*", steerConfig.steerSwitchIsMomentary ? "1" : "0", ControlColor::Wetasphalt, tab,
      []( Control * control, int id ) {
        steerConfig.steerSwitchIsMomentary = control->value.toInt() == 1;
        setResetButtonToRed();
      } );
    } else if( steerConfig.outputType == SteerConfig::OutputType::CanbusF0_240Controller ){
      uint16_t sel = ESPUI.addControl( ControlType::Select, "Fendt Engage Version*", String( ( int )steerConfig.canBusFendtEngageVersion ), ControlColor::Wetasphalt, tab,
      []( Control * control, int id ) {
        steerConfig.canBusFendtEngageVersion = ( SteerConfig::FendtEngageVersion )control->value.toInt();
        setResetButtonToRed();
      } );
      ESPUI.addControl( ControlType::Option, "0x18EF1CC8", "0", ControlColor::Alizarin, sel );
      ESPUI.addControl( ControlType::Option, "0x18EF2CF0", "1", ControlColor::Alizarin, sel );
    }
  }

  // Wheel Angle Sensor Tab
  {
    uint16_t tab = ESPUI.addControl( ControlType::Tab, "Wheel Angle Sensor", "Wheel Angle Sensor" );

    {
      uint16_t sel = ESPUI.addControl( ControlType::Select, "Wheel Angle Sensor*", String( ( int )steerConfig.wheelAngleInput ), ControlColor::Wetasphalt, tab,
      []( Control * control, int id ) {
        steerConfig.wheelAngleInput = ( SteerConfig::AnalogIn )control->value.toInt();
        if( steerConfig.wheelAngleInput >= SteerConfig::AnalogIn::CanbusValtraMasseyChallenger ){
          steerConfig.wheelAngleSensorType = SteerConfig::WheelAngleSensorType::WheelAngle; // arm linkage not applicable for Canbus
        } else if( steerConfig.wheelAngleInput >= SteerConfig::AnalogIn::CanbusFendt ){
          steerConfig.wheelAngleSensorType = SteerConfig::WheelAngleSensorType::WheelAngle;
        }
        setResetButtonToRed();
      } );
      ESPUI.addControl( ControlType::Option, "None", "0", ControlColor::Alizarin, sel );
      addWasInputSelection( sel );
    }

    if( machine.canbusSteeringActive != true ){
      uint16_t sel = ESPUI.addControl( ControlType::Select, "Wheel Angle Sensor Type*", String( ( int )steerConfig.wheelAngleSensorType ), ControlColor::Wetasphalt, tab,
      []( Control * control, int id ) {
        steerConfig.wheelAngleSensorType = ( SteerConfig::WheelAngleSensorType )control->value.toInt();
        setResetButtonToRed();
      } );
      ESPUI.addControl( ControlType::Option, "Direct Wheel Angle", "0", ControlColor::Alizarin, sel );
      ESPUI.addControl( ControlType::Option, "Two Arms connected to tie rod", "1", ControlColor::Alizarin, sel );

      {
        uint16_t sel = ESPUI.addControl( ControlType::Select, "WAS Voltage (ADS1115 Gain)*", String( ( int )steerConfig.adsGain ), ControlColor::Wetasphalt, tab,
        []( Control * control, int id ) {
          steerConfig.adsGain = ( SteerConfig::ADSGain )control->value.toInt();
          setResetButtonToRed();
        } );
        ESPUI.addControl( ControlType::Option, "6.144V (2/3x gain)", "0", ControlColor::Alizarin, sel );
        ESPUI.addControl( ControlType::Option, "4.096V (1x gain)", "512", ControlColor::Alizarin, sel );
        //ESPUI.addControl( ControlType::Option, "2.048V (2x gain)", "1024", ControlColor::Alizarin, sel );
        //ESPUI.addControl( ControlType::Option, "1.024V (4x gain)", "1536", ControlColor::Alizarin, sel );
        //ESPUI.addControl( ControlType::Option, "0.512V (8x gain)", "2048", ControlColor::Alizarin, sel );
        //ESPUI.addControl( ControlType::Option, "0.256V (16x gain)", "2560", ControlColor::Alizarin, sel );
      }
    }

    if( steerConfig.wheelAngleInput != SteerConfig::AnalogIn::CanbusFendt ) {
      uint16_t num = ESPUI.addControl( ControlType::Number, "Wheel Angle Sensor Center", String( steerConfig.wheelAnglePositionZero ), ControlColor::Peterriver, tab,
      []( Control * control, int id ) {
        steerConfig.wheelAnglePositionZero = control->value.toInt();
        saveConfigAfterDelay();
      } );
      ESPUI.addControl( ControlType::Min, "Min", "0", ControlColor::Peterriver, num );
      ESPUI.addControl( ControlType::Max, "Max", "65535", ControlColor::Peterriver, num );
      ESPUI.addControl( ControlType::Step, "Step", "1", ControlColor::Peterriver, num );
    }

    {
      uint16_t num = ESPUI.addControl( ControlType::Number, "Wheel Angle Counts per Degree", String( steerConfig.wheelAngleCountsPerDegree ), ControlColor::Peterriver, tab,
      []( Control * control, int id ) {
        steerConfig.wheelAngleCountsPerDegree = control->value.toFloat();
        saveConfigAfterDelay();
      } );
      ESPUI.addControl( ControlType::Min, "Min", "0", ControlColor::Peterriver, num );
      ESPUI.addControl( ControlType::Max, "Max", "250", ControlColor::Peterriver, num );
      ESPUI.addControl( ControlType::Step, "Step", "0.1", ControlColor::Peterriver, num );
    }

    ESPUI.addControl( ControlType::Switcher, "Invert Wheel Angle Sensor", steerConfig.invertWheelAngleSensor ? "1" : "0", ControlColor::Peterriver, tab,
    []( Control * control, int id ) {
      steerConfig.invertWheelAngleSensor = control->value.toInt() == 1;
      saveConfigAfterDelay();
    } );

    {
      uint16_t num = ESPUI.addControl( ControlType::Number, "Wheel Angle Offset", String( steerConfig.wheelAngleOffset ), ControlColor::Peterriver, tab,
      []( Control * control, int id ) {
        steerConfig.wheelAngleOffset = control->value.toFloat();
        saveConfigAfterDelay();
      } );
      ESPUI.addControl( ControlType::Min, "Roll Min", "-80", ControlColor::Peterriver, num );
      ESPUI.addControl( ControlType::Max, "Roll Max", "80", ControlColor::Peterriver, num );
      ESPUI.addControl( ControlType::Step, "Roll Step", "0.1", ControlColor::Peterriver, num );
    }

    if( steerConfig.wheelAngleSensorType == SteerConfig::WheelAngleSensorType::TieRodDisplacement ) {
      {
        uint16_t num = ESPUI.addControl( ControlType::Number, "1. Arm connect to sensor (mm)", String( steerConfig.wheelAngleFirstArmLenght ), ControlColor::Peterriver, tab,
        []( Control * control, int id ) {
          steerConfig.wheelAngleFirstArmLenght = control->value.toFloat();
          saveConfigAfterDelay();
        } );
        ESPUI.addControl( ControlType::Min, "Min", "0", ControlColor::Peterriver, num );
        ESPUI.addControl( ControlType::Max, "Max", "500", ControlColor::Peterriver, num );
        ESPUI.addControl( ControlType::Step, "Step", "1", ControlColor::Peterriver, num );
      }

      {
        uint16_t num = ESPUI.addControl( ControlType::Number, "2. Arm connect to tie rod (mm)", String( steerConfig.wheelAngleSecondArmLenght ), ControlColor::Peterriver, tab,
        []( Control * control, int id ) {
          steerConfig.wheelAngleSecondArmLenght = control->value.toFloat();
          saveConfigAfterDelay();
        } );
        ESPUI.addControl( ControlType::Min, "Min", "0", ControlColor::Peterriver, num );
        ESPUI.addControl( ControlType::Max, "Max", "500", ControlColor::Peterriver, num );
        ESPUI.addControl( ControlType::Step, "Step", "1", ControlColor::Peterriver, num );
      }

      {
        uint16_t num = ESPUI.addControl( ControlType::Number, "Tie rod stroke (mm)", String( steerConfig.wheelAngleTieRodStroke ), ControlColor::Peterriver, tab,
        []( Control * control, int id ) {
          steerConfig.wheelAngleTieRodStroke = control->value.toFloat();
          saveConfigAfterDelay();
        } );
        ESPUI.addControl( ControlType::Min, "Min", "0", ControlColor::Peterriver, num );
        ESPUI.addControl( ControlType::Max, "Max", "500", ControlColor::Peterriver, num );
        ESPUI.addControl( ControlType::Step, "Step", "1", ControlColor::Peterriver, num );
      }

      {
        uint16_t num = ESPUI.addControl( ControlType::Number, "Minimum Angle of wheel angle sensor", String( steerConfig.wheelAngleMinimumAngle ), ControlColor::Peterriver, tab,
        []( Control * control, int id ) {
          steerConfig.wheelAngleMinimumAngle = control->value.toFloat();
          saveConfigAfterDelay();
        } );
        ESPUI.addControl( ControlType::Min, "Min", "0", ControlColor::Peterriver, num );
        ESPUI.addControl( ControlType::Max, "Max", "180", ControlColor::Peterriver, num );
        ESPUI.addControl( ControlType::Step, "Step", "1", ControlColor::Peterriver, num );
      }

      {
        uint16_t num = ESPUI.addControl( ControlType::Number, "Lenght of Track Arm (mm)", String( steerConfig.wheelAngleTrackArmLenght ), ControlColor::Peterriver, tab,
        []( Control * control, int id ) {
          steerConfig.wheelAngleTrackArmLenght = control->value.toFloat();
          saveConfigAfterDelay();
        } );
        ESPUI.addControl( ControlType::Min, "Min", "0", ControlColor::Peterriver, num );
        ESPUI.addControl( ControlType::Max, "Max", "500", ControlColor::Peterriver, num );
        ESPUI.addControl( ControlType::Step, "Step", "1", ControlColor::Peterriver, num );
      }
    }

    {
      uint16_t num = ESPUI.addControl( ControlType::Number, "% Ackermann (above 100)", String( steerConfig.ackermann ), ControlColor::Peterriver, tab,
      []( Control * control, int id ) {
        steerConfig.ackermann = control->value.toFloat();
        saveConfigAfterDelay();
      } );
      ESPUI.addControl( ControlType::Min, "Min", "0", ControlColor::Peterriver, num );
      ESPUI.addControl( ControlType::Max, "Max", "200", ControlColor::Peterriver, num );
      ESPUI.addControl( ControlType::Step, "Step", "1", ControlColor::Peterriver, num );
    }

    {
      uint16_t sel = ESPUI.addControl( ControlType::Select, "Ackermann - Wheel with WAS has larger radius", String( ( int )steerConfig.ackermannAboveZero ), ControlColor::Peterriver, tab,
      []( Control * control, int id ) {
        steerConfig.ackermannAboveZero = control->value.toInt() == 1;
        saveConfigAfterDelay();
      } );
      ESPUI.addControl( ControlType::Option, "when Actual degrees are below zero", "0", ControlColor::Alizarin, sel );
      ESPUI.addControl( ControlType::Option, "when Actual degrees are above zero", "1", ControlColor::Alizarin, sel );
    }
  }

  // Row Sense Tab
  {
    uint16_t tab = ESPUI.addControl( ControlType::Tab, "Row Sense", "Row Sense" );

    labelRowSense = ESPUI.addControl( ControlType::Label, "Row sense guidance:", "0°", ControlColor::Turquoise, tab );
    {
      ESPUI.addControl( ControlType::Switcher, "Enable Row Sense*", steerConfig.enableRowSense ? "1" : "0", ControlColor::Wetasphalt, tab,
      []( Control * control, int id ) {
        steerConfig.enableRowSense = control->value.toInt() == 1;
        setResetButtonToRed();
      } );
    }

    if( steerConfig.enableRowSense ){
      {
        uint16_t num = ESPUI.addControl( ControlType::Number, "Row Sense Center", String( steerConfig.rowSensePositionZero ), ControlColor::Peterriver, tab,
        []( Control * control, int id ) {
          steerConfig.rowSensePositionZero = control->value.toInt();
          saveConfigAfterDelay();
        } );
        ESPUI.addControl( ControlType::Min, "Min", "0", ControlColor::Peterriver, num );
        ESPUI.addControl( ControlType::Max, "Max", "65535", ControlColor::Peterriver, num );
        ESPUI.addControl( ControlType::Step, "Step", "1", ControlColor::Peterriver, num );
      }

      {
        uint16_t num = ESPUI.addControl( ControlType::Number, "Row Sense Counts per Degree", String( steerConfig.rowSenseCountsPerDegree ), ControlColor::Peterriver, tab,
        []( Control * control, int id ) {
          steerConfig.rowSenseCountsPerDegree = control->value.toFloat();
          saveConfigAfterDelay();
        } );
        ESPUI.addControl( ControlType::Min, "Min", "0", ControlColor::Peterriver, num );
        ESPUI.addControl( ControlType::Max, "Max", "500", ControlColor::Peterriver, num );
        ESPUI.addControl( ControlType::Step, "Step", "1.0", ControlColor::Peterriver, num );
      }

      {
        ESPUI.addControl( ControlType::Switcher, "Invert Row Sense", steerConfig.invertRowSense ? "1" : "0", ControlColor::Peterriver, tab,
        []( Control * control, int id ) {
          steerConfig.invertRowSense = control->value.toInt() == 1;
          saveConfigAfterDelay();
        } );
      }

      {
        uint16_t num = ESPUI.addControl( ControlType::Number, "Row Sense Min Degrees (Deadband)", String( steerConfig.rowSenseMinDegrees ), ControlColor::Peterriver, tab,
        []( Control * control, int id ) {
          steerConfig.rowSenseMinDegrees = control->value.toInt();
          saveConfigAfterDelay();
        } );
        ESPUI.addControl( ControlType::Min, "Min", "0", ControlColor::Peterriver, num );
        ESPUI.addControl( ControlType::Max, "Max", "10", ControlColor::Peterriver, num );
        ESPUI.addControl( ControlType::Step, "Step", "1.0", ControlColor::Peterriver, num );
      }

      {
        uint16_t num = ESPUI.addControl( ControlType::Number, "Row Sense Kp", String( steerConfig.rowSenseKp ), ControlColor::Peterriver, tab,
        []( Control * control, int id ) {
          steerConfig.rowSenseKp = control->value.toFloat();
          saveConfigAfterDelay();
        } );
        ESPUI.addControl( ControlType::Min, "Min", "0", ControlColor::Peterriver, num );
        ESPUI.addControl( ControlType::Max, "Max", "50", ControlColor::Peterriver, num );
        ESPUI.addControl( ControlType::Step, "Step", "0.01", ControlColor::Peterriver, num );
      }

      {
        uint16_t num = ESPUI.addControl( ControlType::Number, "Row Sense Ki", String( steerConfig.rowSenseKi ), ControlColor::Peterriver, tab,
        []( Control * control, int id ) {
          steerConfig.rowSenseKi = control->value.toFloat();
          saveConfigAfterDelay();
        } );
        ESPUI.addControl( ControlType::Min, "Min", "0", ControlColor::Peterriver, num );
        ESPUI.addControl( ControlType::Max, "Max", "50", ControlColor::Peterriver, num );
        ESPUI.addControl( ControlType::Step, "Step", "0.01", ControlColor::Peterriver, num );
      }

      {
        uint16_t num = ESPUI.addControl( ControlType::Number, "Row Sense Ki Max Degrees", String( steerConfig.rowSenseKiMaxDegrees ), ControlColor::Peterriver, tab,
        []( Control * control, int id ) {
          steerConfig.rowSenseKiMaxDegrees = control->value.toInt();
          saveConfigAfterDelay();
        } );
        ESPUI.addControl( ControlType::Min, "Min", "0", ControlColor::Peterriver, num );
        ESPUI.addControl( ControlType::Max, "Max", "50", ControlColor::Peterriver, num );
        ESPUI.addControl( ControlType::Step, "Step", "1.0", ControlColor::Peterriver, num );
      }
    }
  }

  // Steering Tab
  {
    uint16_t tab = ESPUI.addControl( ControlType::Tab, "Steering", "Steering" );

    labelStatusOutput = ESPUI.addControl( ControlType::Label, "Output:", "No Output configured", ControlColor::Turquoise, tab );
    {
      uint16_t sel = ESPUI.addControl( ControlType::Select, "Output Type*", String( ( int )steerConfig.outputType ), ControlColor::Wetasphalt, tab,
      []( Control * control, int id ) {
        steerConfig.outputType = ( SteerConfig::OutputType )control->value.toInt();
        steerConfig.steeringPidMaxPwm = 191;
        setResetButtonToRed();
      } );
      ESPUI.addControl( ControlType::Option, "None", "0", ControlColor::Alizarin, sel );
      ESPUI.addControl( ControlType::Option, "Motor: Cytron MD30C", "1", ControlColor::Alizarin, sel );
      ESPUI.addControl( ControlType::Option, "Motor: IBT 2", "2", ControlColor::Alizarin, sel );
      ESPUI.addControl( ControlType::Option, "Hydraulic: IBT 2 + PWM 2-Coil Valve", "3", ControlColor::Alizarin, sel );
      ESPUI.addControl( ControlType::Option, "Hydraulic: IBT 2 + Danfoss Valve PVE A/H/M", "4", ControlColor::Alizarin, sel );
      ESPUI.addControl( ControlType::Option, "Hydraulic: IBT 2 + Bang Bang Valve", "5", ControlColor::Alizarin, sel );
      if( steerConfig.canBusEnabled ) {
        ESPUI.addControl( ControlType::Option, "Canbus: 13/19 Controller", "6", ControlColor::Alizarin, sel );
        ESPUI.addControl( ControlType::Option, "Canbus: F0/240 Controller", "7", ControlColor::Alizarin, sel );
      }
    }

    ESPUI.addControl( ControlType::Switcher, "Invert Output", steerConfig.invertOutput ? "1" : "0", ControlColor::Peterriver, tab,
    []( Control * control, int id ) {
      steerConfig.invertOutput = control->value.toInt() == 1;
      saveConfigAfterDelay();
    } );

    {
      uint16_t num = ESPUI.addControl( ControlType::Number, "PWM Frequency*", String( steerConfig.pwmFrequency, 2 ), ControlColor::Wetasphalt, tab,
      []( Control * control, int id ) {
        steerConfig.pwmFrequency = control->value.toDouble();
        setResetButtonToRed();
      } );
      ESPUI.addControl( ControlType::Min, "Min", "1", ControlColor::Peterriver, num );
      ESPUI.addControl( ControlType::Max, "Max", "4000", ControlColor::Peterriver, num );
      ESPUI.addControl( ControlType::Step, "Step", "1", ControlColor::Peterriver, num );
    }

    {
      uint16_t num = ESPUI.addControl( ControlType::Number, "Min autosteer speed", String( steerConfig.minAutosteerSpeed ), ControlColor::Peterriver, tab,
      []( Control * control, int id ) {
        steerConfig.minAutosteerSpeed = control->value.toFloat();
        saveConfigAfterDelay();
      } );
      ESPUI.addControl( ControlType::Min, "Min", "0", ControlColor::Peterriver, num );
      ESPUI.addControl( ControlType::Max, "Max", "2", ControlColor::Peterriver, num );
      ESPUI.addControl( ControlType::Step, "Step", "0.1", ControlColor::Peterriver, num );
    }

    if( steerConfig.outputType == SteerConfig::OutputType::HydraulicPwm2Coil ){
      uint16_t num = ESPUI.addControl( ControlType::Number, "Dither", String( steerConfig.dither ), ControlColor::Peterriver, tab,
      []( Control * control, int id ) {
        steerConfig.dither = control->value.toInt();
        saveConfigAfterDelay();
        ditherAmount = 0;
      } );
      ESPUI.addControl( ControlType::Min, "Min", "0", ControlColor::Peterriver, num );
      ESPUI.addControl( ControlType::Max, "Max", "100", ControlColor::Peterriver, num );
      ESPUI.addControl( ControlType::Step, "Step", "1", ControlColor::Peterriver, num );
    }

    manualValveSwitcher = ESPUI.addControl( Switcher, "Manual valve control", "", Peterriver, tab, manualValveCallback );
    {
      if( steerConfig.outputType == SteerConfig::OutputType::HydraulicDanfoss ){
          steerConfig.manualPWM = 128;
        } else {
          steerConfig.manualPWM = 0;
        }
      manualValvePWMWidget = ESPUI.addControl( Number, "Manual PWM", String( steerConfig.manualPWM ), Peterriver, tab, manualValvePWMCallback );
      ESPUI.addControl( ControlType::Min, "Min", "-255", ControlColor::Peterriver, manualValvePWMWidget );
      ESPUI.addControl( ControlType::Max, "Max", "255", ControlColor::Peterriver, manualValvePWMWidget );
      ESPUI.addControl( ControlType::Step, "Step", "1", ControlColor::Peterriver, manualValvePWMWidget );
    }
  }

  // Steering PID Tab
  {
    uint16_t tab = ESPUI.addControl( ControlType::Tab, "Steering PID", "Steering PID" );

    {
      uint16_t num = ESPUI.addControl( ControlType::Number, "PID Kp", String( steerConfig.steeringPidKp, 2 ), ControlColor::Peterriver, tab,
      []( Control * control, int id ) {
        steerConfig.steeringPidKp = control->value.toDouble();
        saveConfigAfterDelay();
      } );
      ESPUI.addControl( ControlType::Min, "Min", "0", ControlColor::Peterriver, num );
      ESPUI.addControl( ControlType::Max, "Max", "50", ControlColor::Peterriver, num );
      ESPUI.addControl( ControlType::Step, "Step", "0.1", ControlColor::Peterriver, num );
    }
    {
      uint16_t num = ESPUI.addControl( ControlType::Number, "PID Ki", String( steerConfig.steeringPidKi, 2 ), ControlColor::Peterriver, tab,
      []( Control * control, int id ) {
        steerConfig.steeringPidKi = control->value.toDouble();
        saveConfigAfterDelay();
      } );
      ESPUI.addControl( ControlType::Min, "Min", "0", ControlColor::Peterriver, num );
      ESPUI.addControl( ControlType::Max, "Max", "50", ControlColor::Peterriver, num );
      ESPUI.addControl( ControlType::Step, "Step", "0.01", ControlColor::Peterriver, num );
    }
    {
      uint16_t num = ESPUI.addControl( ControlType::Number, "PID Ki Max", String( steerConfig.steeringPidKiMax, 2 ), ControlColor::Peterriver, tab,
      []( Control * control, int id ) {
        steerConfig.steeringPidKiMax = control->value.toDouble();
        saveConfigAfterDelay();
      } );
      ESPUI.addControl( ControlType::Min, "Min", "0", ControlColor::Peterriver, num );
      ESPUI.addControl( ControlType::Max, "Max", "255", ControlColor::Peterriver, num );
      ESPUI.addControl( ControlType::Step, "Step", "1.00", ControlColor::Peterriver, num );
    }
    {
      uint16_t num = ESPUI.addControl( ControlType::Number, "PID Kd", String( steerConfig.steeringPidKd, 2 ), ControlColor::Peterriver, tab,
      []( Control * control, int id ) {
        steerConfig.steeringPidKd = control->value.toDouble();
        saveConfigAfterDelay();
      } );
      ESPUI.addControl( ControlType::Min, "Min", "0", ControlColor::Peterriver, num );
      ESPUI.addControl( ControlType::Max, "Max", "50", ControlColor::Peterriver, num );
      ESPUI.addControl( ControlType::Step, "Step", "0.01", ControlColor::Peterriver, num );
    }
    {
      uint16_t num = ESPUI.addControl( ControlType::Number, "Minimum PWM", String( steerConfig.steeringPidMinPwm ), ControlColor::Peterriver, tab,
      []( Control * control, int id ) {
        steerConfig.steeringPidMinPwm = control->value.toInt();
        saveConfigAfterDelay();
      } );
      ESPUI.addControl( ControlType::Min, "Min", "0", ControlColor::Peterriver, num );
      ESPUI.addControl( ControlType::Max, "Max", "255", ControlColor::Peterriver, num );
      ESPUI.addControl( ControlType::Step, "Step", "1", ControlColor::Peterriver, num );
    }
    {
      uint16_t num = ESPUI.addControl( ControlType::Number, "Maximum PWM", String( steerConfig.steeringPidMaxPwm ), ControlColor::Peterriver, tab,
      []( Control * control, int id ) {
        steerConfig.steeringPidMaxPwm = control->value.toInt();
        saveConfigAfterDelay();
        if ( steerConfig.steeringPidMaxPwm > 255 ){
          steerConfig.steeringPidMaxPwm = 255;
        }
        if( steerConfig.outputType == SteerConfig::OutputType::HydraulicDanfoss && steerConfig.steeringPidMaxPwm > 191 ){
          steerConfig.steeringPidMaxPwm = 191;
          ESPUI.updateText( id, ( String )steerConfig.steeringPidMaxPwm );
        }
        else if ( steerConfig.steeringPidMaxPwm > 255 ){
          steerConfig.steeringPidMaxPwm = 255;
          ESPUI.updateText( id, ( String )steerConfig.steeringPidMaxPwm );
        }
      } );
      ESPUI.addControl( ControlType::Min, "Min", "0", ControlColor::Peterriver, num );
      ESPUI.addControl( ControlType::Max, "Max", "255", ControlColor::Peterriver, num );
      ESPUI.addControl( ControlType::Step, "Step", "1", ControlColor::Peterriver, num );
    }
  }

  // Safety Tab
  {
    uint16_t tab = ESPUI.addControl( ControlType::Tab, "Safety", "Safety" );

    {
      uint16_t num = ESPUI.addControl( ControlType::Number, "Max engage speed", String( steerConfig.maxAutosteerSpeed ), ControlColor::Peterriver, tab,
      []( Control * control, int id ) {
        steerConfig.maxAutosteerSpeed = control->value.toFloat();
        saveConfigAfterDelay();
      } );
      ESPUI.addControl( ControlType::Min, "Min", "0", ControlColor::Peterriver, num );
      ESPUI.addControl( ControlType::Max, "Max", "30", ControlColor::Peterriver, num );
      ESPUI.addControl( ControlType::Step, "Step", "0.5", ControlColor::Peterriver, num );
    }
    {
      uint16_t sel = ESPUI.addControl( ControlType::Select, "Speed units", String( ( int )steerConfig.speedUnits ), ControlColor::Peterriver, tab,
      []( Control * control, int id ) {
        steerConfig.speedUnits =  ( SteerConfig::SpeedUnits )control->value.toInt();
        saveConfigAfterDelay();
      } );
      ESPUI.addControl( ControlType::Option, "MPH", "0", ControlColor::Alizarin, sel );
      ESPUI.addControl( ControlType::Option, "KPH", "1", ControlColor::Alizarin, sel );
    }
    if( machine.canbusSteeringActive == false ) {
      {
      uint16_t sel = ESPUI.addControl( ControlType::Select, "Disengage Switch Type*", String( ( int )steerConfig.disengageSwitchType ), ControlColor::Wetasphalt, tab,
      []( Control * control, int id ) {
        steerConfig.disengageSwitchType = ( SteerConfig::DisengageSwitchType )control->value.toInt();
        if( steerConfig.disengageSwitchType != SteerConfig::DisengageSwitchType::Hydraulic ){
          steerConfig.disengageHeavyDuty = false;
        }
        setResetButtonToRed();
      } );
      ESPUI.addControl( ControlType::Option, "Encoder on steering shaft", "0", ControlColor::Alizarin, sel );
      ESPUI.addControl( ControlType::Option, "Pressure switch in hydraulics", "1", ControlColor::Alizarin, sel );
      ESPUI.addControl( ControlType::Option, "Deere variable duty sensor", "2", ControlColor::Alizarin, sel );
      ESPUI.addControl( ControlType::Option, "Motor current", "3", ControlColor::Alizarin, sel );
      ESPUI.addControl( ControlType::Option, "Logic input on GPIO12", "4", ControlColor::Alizarin, sel );
    }

    switch( steerConfig.disengageSwitchType ){
      case SteerConfig::DisengageSwitchType::Hydraulic: {
        ESPUI.addControl( ControlType::Switcher, "Hydraulic Switch Active Low", steerConfig.hydraulicSwitchActiveLow ? "1" : "0", ControlColor::Peterriver, tab,
        []( Control * control, int id ) {
          steerConfig.hydraulicSwitchActiveLow = control->value.toInt() == 1;
          saveConfigAfterDelay();
        } );

        uint16_t sel = ESPUI.addControl( ControlType::Switcher, "Heavy Duty Disengage Switch", steerConfig.disengageHeavyDuty ? "1" : "0" , ControlColor::Peterriver, tab,
        []( Control * control, int id ) {
          steerConfig.disengageHeavyDuty = control->value.toInt() == 1;
          saveConfigAfterDelay();
          pinMode( steerConfig.gpioDisengagePullup, OUTPUT );
          digitalWrite( steerConfig.gpioDisengagePullup, steerConfig.disengageHeavyDuty );
        } );
      }
      break;
      case SteerConfig::DisengageSwitchType::Encoder: {
        {
          uint16_t num = ESPUI.addControl( ControlType::Number, "Steering Wheel Pulses per Frame", String( steerConfig.disengageFramePulses ), ControlColor::Peterriver, tab,
          []( Control * control, int id ) {
            steerConfig.disengageFramePulses = control->value.toInt();
            saveConfigAfterDelay();
          } );
          ESPUI.addControl( ControlType::Min, "Min", "1", ControlColor::Peterriver, num );
          ESPUI.addControl( ControlType::Max, "Max", "1000", ControlColor::Peterriver, num );
          ESPUI.addControl( ControlType::Step, "Step", "10", ControlColor::Peterriver, num );
        }

        {
          uint16_t num = ESPUI.addControl( ControlType::Number, "Steering Wheel Millis per Frame", String( steerConfig.disengageFrameMillis ), ControlColor::Peterriver, tab,
          []( Control * control, int id ) {
            steerConfig.disengageFrameMillis = control->value.toInt();
            saveConfigAfterDelay();
          } );
          ESPUI.addControl( ControlType::Min, "Min", "1", ControlColor::Peterriver, num );
          ESPUI.addControl( ControlType::Max, "Max", "10000", ControlColor::Peterriver, num );
          ESPUI.addControl( ControlType::Step, "Step", "1000", ControlColor::Peterriver, num );
        }
      }
      break;
      case SteerConfig::DisengageSwitchType::JDVariableDuty: {
        uint16_t num = ESPUI.addControl( ControlType::Number, "Deere Vari-duty Change per Frame", String( steerConfig.JDVariableDutyChange ), ControlColor::Peterriver, tab,
        []( Control * control, int id ) {
          steerConfig.JDVariableDutyChange = control->value.toInt();
          saveConfigAfterDelay();
        } );
        ESPUI.addControl( ControlType::Min, "Min", "1", ControlColor::Peterriver, num );
        ESPUI.addControl( ControlType::Max, "Max", "500", ControlColor::Peterriver, num );
        ESPUI.addControl( ControlType::Step, "Step", "1", ControlColor::Peterriver, num );

        num = ESPUI.addControl( ControlType::Number, "Deere Vari-duty Frame Length (millis)", String( steerConfig.JDVariableDutyFrameLength ), ControlColor::Peterriver, tab,
        []( Control * control, int id ) {
          steerConfig.JDVariableDutyFrameLength = control->value.toInt();
          saveConfigAfterDelay();
        } );
        ESPUI.addControl( ControlType::Min, "Min", "1", ControlColor::Peterriver, num );
        ESPUI.addControl( ControlType::Max, "Max", "1000", ControlColor::Peterriver, num );
        ESPUI.addControl( ControlType::Step, "Step", "1", ControlColor::Peterriver, num );
      }
      break;
      case SteerConfig::DisengageSwitchType::MotorCurrent: {
        uint16_t num = ESPUI.addControl( ControlType::Number, "Max Steer Motor Current", String( steerConfig.maxSteerCurrent ), ControlColor::Peterriver, tab,
        []( Control * control, int id ) {
          steerConfig.maxSteerCurrent = control->value.toInt();
          saveConfigAfterDelay();
        } );
        ESPUI.addControl( ControlType::Min, "Min", "0", ControlColor::Peterriver, num );
        ESPUI.addControl( ControlType::Max, "Max", "4096", ControlColor::Peterriver, num );
        ESPUI.addControl( ControlType::Step, "Step", "10", ControlColor::Peterriver, num );
      }
      break;
      }
    }
  }

  char autosteerDownloadHTML [100];
  sprintf( downloadFilename, "/%s steering.json", steerConfig.hostname );
  sprintf( autosteerDownloadHTML, "<a href='%s'>Configuration</a>", downloadFilename );

  // Default Configurations Tab
  {
    uint16_t tab = ESPUI.addControl( ControlType::Tab, "Configurations", "Configurations" );

    #ifdef CUSTOM_PROG_VERSION
      String buildDate = CUSTOM_PROG_VERSION;
      buildDate += String("\n");
      buildDate += String(__DATE__);
      buildDate += String(" ");
      buildDate += String(__TIME__);
    #else
      String buildDate = String(__DATE__);
      buildDate += String(" ");
      buildDate += String(__TIME__);
    #endif
      labelBuildDate = ESPUI.addControl( ControlType::Label, "Build:", buildDate, ControlColor::Turquoise, tab );
    
    ESPUI.addControl( ControlType::Label, "OTA Update:", "<a href='/update'>Update</a>", ControlColor::Carrot, tab );

    ESPUI.addControl( ControlType::Label, "Download the config:", autosteerDownloadHTML, ControlColor::Carrot, tab );

    ESPUI.addControl( ControlType::Label, "Upload the config:", "<form id='ucForm' enctype='multipart/form-data'><input id='ucFile' name='f' type='file'><input type='submit' value='Submit'></form><span id='ucStatus'></span><script>document.getElementById('ucForm').addEventListener('submit',function(e){e.preventDefault();var f=document.getElementById('ucFile').files[0];if(!f)return;var fd=new FormData();fd.append('f',f);document.getElementById('ucStatus').innerText='Uploading...';fetch('/upload-config',{method:'POST',body:fd}).then(function(){var s=document.getElementById('ucStatus');var n=8;function tick(){s.innerText='Rebooting, redirecting in '+n+'s...';if(n-->0)setTimeout(tick,1000);else window.location.href='/';}tick();}).catch(function(){document.getElementById('ucStatus').innerText='Upload failed.';});});</script>", ControlColor::Carrot, tab );
    
    tabConfigurations = tab;

  }
  
  static String title;

  title = "AOG Control :: ";

  title += steerConfig.hostname;
  ESPUI.begin( title.c_str() );

  ESPUI.setPanelStyle(buttonReset, "display: none;");

  ESPUI.WebServer()->on( downloadFilename, HTTP_GET, []( AsyncWebServerRequest * request ) {
    
    Serial.print( "Preparing " );
    Serial.print( downloadFilename );
    Serial.println( " for download" );

    // Serve the existing LittleFS file directly but set the download filename
    // by adding a Content-Disposition header on the response.
    if ( !LittleFS.exists( "/autosteer.json" ) ){
      Serial.println( "/autosteer.json not available for download" );
      request->send( 404 );
      return;
    }

    String fname = String( downloadFilename );
    if ( fname.length() && fname.charAt( 0 ) == '/' ) {
      fname = fname.substring( 1 );
    }

    AsyncWebServerResponse * response = request->beginResponse( LittleFS, "/autosteer.json", "application/json" );
    response->addHeader( "Content-Disposition", String( "attachment; filename=\"" ) + fname + String( "\"" ) );
    request->send( response );
  } );
  
  // upload a file to /upload-config
  ESPUI.WebServer()->on( "/upload-config", HTTP_POST, []( AsyncWebServerRequest * request ) {
    request->send( 200, "text/plain", "OK" );
  }, [tabConfigurations]( AsyncWebServerRequest * request, String filename, size_t index, uint8_t* data, size_t len, bool final ) {
    if( !index ) {
      request->_tempFile = LittleFS.open( "/autosteer.json", "w" );
    }

    if( request->_tempFile ) {
      if( len ) {
        request->_tempFile.write( data, len );
      }

      if( final ) {
        request->_tempFile.close();
        if( steerConfig.canBusEnabled ){
          twai_stop();
        }
        xTaskCreate( []( void* ) { vTaskDelay( 500 / portTICK_PERIOD_MS ); ESP.restart(); }, 
                      "restart", 1024, nullptr, 1, nullptr );
      }
    }
  } );
}

void manualValveCallback(Control *sender, int type) {
  steerConfig.manualSteerState = sender->value.toInt() == 1;
  if( steerConfig.outputType == SteerConfig::OutputType::HydraulicDanfoss ){
    steerConfig.manualPWM = 128;
  } else {
    steerConfig.manualPWM = 0;
  }
  if( steerConfig.manualSteerState == true ){
    ESPUI.updateNumber( manualValvePWMWidget, steerConfig.manualPWM );
  }
}

void manualValvePWMCallback(Control *sender, int type) {
  steerConfig.manualPWM = sender->value.toInt();
}
