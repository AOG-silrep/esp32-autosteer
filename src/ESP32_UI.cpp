
#include <stdio.h>

#include <ESPUI.h>

#include "main.hpp"
#include "jsonFunctions.hpp"

int8_t ditherAmount = 0;
uint16_t labelLoad;
uint16_t labelWheelAngle;
uint16_t labelSafetyDisableAutosteer;
uint16_t labelSupplyVoltage;
uint16_t labelSteerMotorCurrent;
uint16_t labelSteerEngagedFaults;
uint16_t labelSwitchStates;
uint16_t buttonReset;

uint16_t labelWheelAngleDisplacement;

uint16_t labelStatusOutput;
uint16_t labelStatusAdc;
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

  labelLoad = ESPUI.addControl( ControlType::Label, "Load:", "", ControlColor::Turquoise );
  labelWheelAngle = ESPUI.addControl( ControlType::Label, "Wheel Angle:", "0°", ControlColor::Emerald );

  buttonReset = ESPUI.addControl( ControlType::Button, "Store the Settings", "Apply", ControlColor::Emerald, Control::noParent,
  []( Control * control, int id ) {
    if( id == B_UP ) {
      saveConfig();
    }
  } );

  buttonReset = ESPUI.addControl( ControlType::Button, "If this turns red, you have to", "Apply & Reboot", ControlColor::Emerald, Control::noParent,
  []( Control * control, int id ) {
    if( id == B_UP ) {
      saveConfig();
      SPIFFS.end();
      if( steerConfig.canBusEnabled ){
        ESP32Can.CANStop();
      }
      ESP.restart();
    }
  } );

  uint16_t tabConfigurations;

  // Diagnostics Tab
  {
    uint16_t tab = ESPUI.addControl( ControlType::Tab, "Diagnostics", "Diagnostics" );

    labelSafetyDisableAutosteer = ESPUI.addControl( ControlType::Label, "Safety disable autosteer:", "Not started", ControlColor::Emerald, tab );
    labelSupplyVoltage = ESPUI.addControl( ControlType::Label, "Steer valve supply voltage:", "Not loaded", ControlColor::Emerald, tab );
    labelSteerMotorCurrent = ESPUI.addControl( ControlType::Label, "Steer motor current:", "Not loaded", ControlColor::Emerald, tab );
    labelSteerEngagedFaults = ESPUI.addControl( ControlType::Label, "Steering engaged with no power:", "Not loaded", ControlColor::Emerald, tab );
    labelSwitchStates = ESPUI.addControl( ControlType::Label, "Switch states:", "Not loaded", ControlColor::Emerald, tab );
    ESPUI.addControl( ControlType::Button, "Diagnostics:", "Reset all to zero", ControlColor::Emerald, tab, []( Control * control, int id ) {
      if( id == B_UP ) {
        diagnostics.steerSupplyVoltageMax = machine.steerSupplyVoltage;
        diagnostics.steerSupplyVoltageMin = machine.steerSupplyVoltage;
        diagnostics.steerEnabledWithNoPower = 0;
        diagnostics.fuse1Shorted = 0;
        diagnostics.fuse2Shorted = 0;
        saveDiagnostics();

        Control* labelSteerEngagedFaultsHandle = ESPUI.getControl( labelSteerEngagedFaults );
        String str;
        str.reserve( 30 );
        str = "Number of faults: ";
        str += ( int8_t ) diagnostics.steerEnabledWithNoPower;
        str += "\nFault active since startup: No";
        labelSteerEngagedFaultsHandle->value = str;
        labelSteerEngagedFaultsHandle->color = ControlColor::Emerald;
        ESPUI.updateControl( labelSteerEngagedFaultsHandle );
        
      }
    } );
  }
 
  // Status Tab
  {
    uint16_t tab = ESPUI.addControl( ControlType::Tab, "Status", "Status" );

    labelStatusOutput = ESPUI.addControl( ControlType::Label, "Output:", "No Output configured", ControlColor::Turquoise, tab );
    labelStatusAdc = ESPUI.addControl( ControlType::Label, "WAS:", "No WAS configured", ControlColor::Turquoise, tab );
    labelStatusCanESP32 = ESPUI.addControl( ControlType::Label, "ESP32 CAN:", "No CAN BUS configured", ControlColor::Turquoise, tab );
    labelStatusCanMCP2515 = ESPUI.addControl( ControlType::Label, "MCP2515 CAN:", "No CAN BUS configured", ControlColor::Turquoise, tab );
    String buildDate = String(__DATE__);
    buildDate += String(" ");
    buildDate += String(__TIME__);
    labelBuildDate = ESPUI.addControl( ControlType::Label, "Build date :", buildDate, ControlColor::Turquoise, tab );
  }

  // Network Tab
  {
    uint16_t tab = ESPUI.addControl( ControlType::Tab, "Network", "Network" );

    {
      uint16_t baudrate = ESPUI.addControl( ControlType::Select, "Baudrate Serial", String( steerConfig.baudrate ), ControlColor::Peterriver, tab,
      []( Control * control, int id ) {
        uint32_t baudrate = control->value.toInt();
        steerConfig.baudrate = baudrate;
        Serial.updateBaudRate( baudrate );
      } );
      ESPUI.addControl( ControlType::Option, "4800", "4800", ControlColor::Alizarin, baudrate );
      ESPUI.addControl( ControlType::Option, "9600", "9600", ControlColor::Alizarin, baudrate );
      ESPUI.addControl( ControlType::Option, "19200", "19200", ControlColor::Alizarin, baudrate );
      ESPUI.addControl( ControlType::Option, "38400", "38400", ControlColor::Alizarin, baudrate );
      ESPUI.addControl( ControlType::Option, "57600", "57600", ControlColor::Alizarin, baudrate );
      ESPUI.addControl( ControlType::Option, "115200", "115200", ControlColor::Alizarin, baudrate );
      ESPUI.addControl( ControlType::Option, "230400", "230400", ControlColor::Alizarin, baudrate );
      ESPUI.addControl( ControlType::Option, "460800", "460800", ControlColor::Alizarin, baudrate );
      ESPUI.addControl( ControlType::Option, "921600", "921600", ControlColor::Alizarin, baudrate );
    }

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

    ESPUI.addControl( ControlType::Switcher, "OTA Enabled*", steerConfig.enableOTA ? "1" : "0", ControlColor::Wetasphalt, tab,
    []( Control * control, int id ) {
      steerConfig.enableOTA = control->value.toInt() == 1;
      setResetButtonToRed();
    } );

    ESPUI.addControl( ControlType::Number, "Port to send from*", String( steerConfig.aogPortSendFrom ), ControlColor::Wetasphalt, tab,
    []( Control * control, int id ) {
      steerConfig.aogPortSendFrom = control->value.toInt();
      setResetButtonToRed();
    } );

    ESPUI.addControl( ControlType::Number, "Port to send to*", String( steerConfig.aogPortSendTo ), ControlColor::Wetasphalt, tab,
    []( Control * control, int id ) {
      steerConfig.aogPortSendTo = control->value.toInt();
      setResetButtonToRed();
    } );
    ESPUI.addControl( ControlType::Number, "Port to listen to*", String( steerConfig.aogPortListenTo ), ControlColor::Wetasphalt, tab,
    []( Control * control, int id ) {
      steerConfig.aogPortListenTo = control->value.toInt();
      setResetButtonToRed();
    } );
  }

  // CAN Bus
  {
    uint16_t tab = ESPUI.addControl( ControlType::Tab, "CAN Bus/J1939", "CAN Bus/J1939" );

    ESPUI.addControl( ControlType::Switcher, "CAN Bus Enabled*", steerConfig.canBusEnabled ? "1" : "0", ControlColor::Wetasphalt, tab,
    []( Control * control, int id ) {
      if( steerConfig.canBusEnabled ){
        vTaskDelete( &canReceiverHandle );
        vTaskDelete( &canSenderHandle );
        ESP32Can.CANStop();
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
  }

  // Switches/Buttons Tab
  {
    uint16_t tab = ESPUI.addControl( ControlType::Tab, "Work- and Steerswitch", "Work- and Steerswitch" );

    uint16_t sel = ESPUI.addControl( ControlType::Select, "Workswitch Type", String( ( int )steerConfig.workswitchType ), ControlColor::Peterriver, tab,
      []( Control * control, int id ) {
        steerConfig.workswitchType = ( SteerConfig::WorkswitchType )control->value.toInt();
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
      }
      {
        uint16_t num = ESPUI.addControl( ControlType::Number, "Hitch Threshold", String( steerConfig.canBusHitchThreshold ), ControlColor::Peterriver, tab,
        []( Control * control, int id ) {
          steerConfig.canBusHitchThreshold = control->value.toInt();
        } );
        ESPUI.addControl( ControlType::Min, "Min", "0", ControlColor::Peterriver, num );
        ESPUI.addControl( ControlType::Max, "Max", "100", ControlColor::Peterriver, num );
        ESPUI.addControl( ControlType::Step, "Step", "1", ControlColor::Peterriver, num );
      }
      {
        uint16_t num = ESPUI.addControl( ControlType::Number, "Hitch Threshold Hysteresis", String( steerConfig.canBusHitchThresholdHysteresis ), ControlColor::Peterriver, tab,
        []( Control * control, int id ) {
          steerConfig.canBusHitchThresholdHysteresis = control->value.toInt();
        } );
        ESPUI.addControl( ControlType::Min, "Min", "0", ControlColor::Peterriver, num );
        ESPUI.addControl( ControlType::Max, "Max", "100", ControlColor::Peterriver, num );
        ESPUI.addControl( ControlType::Step, "Step", "1", ControlColor::Peterriver, num );
      }

      {
        uint16_t num = ESPUI.addControl( ControlType::Number, "RPM Threshold", String( steerConfig.canBusRpmThreshold ), ControlColor::Peterriver, tab,
        []( Control * control, int id ) {
          steerConfig.canBusRpmThreshold = control->value.toInt();
        } );
        ESPUI.addControl( ControlType::Min, "Min", "0", ControlColor::Peterriver, num );
        ESPUI.addControl( ControlType::Max, "Max", "3500", ControlColor::Peterriver, num );
        ESPUI.addControl( ControlType::Step, "Step", "1", ControlColor::Peterriver, num );
      }
      {
        uint16_t num = ESPUI.addControl( ControlType::Number, "RPM Threshold Hysteresis", String( steerConfig.canBusRpmThresholdHysteresis ), ControlColor::Peterriver, tab,
        []( Control * control, int id ) {
          steerConfig.canBusRpmThresholdHysteresis = control->value.toInt();
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
      } );
    }

    {
      ESPUI.addControl( ControlType::Switcher, "Autosteer Switch Active Low", steerConfig.steerswitchActiveLow ? "1" : "0", ControlColor::Peterriver, tab,
      []( Control * control, int id ) {
        steerConfig.steerswitchActiveLow = control->value.toInt() == 1;
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
    }

    {
      uint16_t num = ESPUI.addControl( ControlType::Number, "Wheel Angle Sensor Center", String( steerConfig.wheelAnglePositionZero ), ControlColor::Peterriver, tab,
      []( Control * control, int id ) {
        steerConfig.wheelAnglePositionZero = control->value.toInt();
      } );
      ESPUI.addControl( ControlType::Min, "Min", "0", ControlColor::Peterriver, num );
      ESPUI.addControl( ControlType::Max, "Max", "26000", ControlColor::Peterriver, num );
      ESPUI.addControl( ControlType::Step, "Step", "1", ControlColor::Peterriver, num );
    }

    {
      uint16_t num = ESPUI.addControl( ControlType::Number, "Wheel Angle Counts per Degree", String( steerConfig.wheelAngleCountsPerDegree ), ControlColor::Peterriver, tab,
      []( Control * control, int id ) {
        steerConfig.wheelAngleCountsPerDegree = control->value.toFloat();
      } );
      ESPUI.addControl( ControlType::Min, "Min", "0", ControlColor::Peterriver, num );
      ESPUI.addControl( ControlType::Max, "Max", "250", ControlColor::Peterriver, num );
      ESPUI.addControl( ControlType::Step, "Step", "0.1", ControlColor::Peterriver, num );
    }

    ESPUI.addControl( ControlType::Switcher, "Invert Wheel Angle Sensor", steerConfig.invertWheelAngleSensor ? "1" : "0", ControlColor::Peterriver, tab,
    []( Control * control, int id ) {
      steerConfig.invertWheelAngleSensor = control->value.toInt() == 1;
    } );

    {
      uint16_t num = ESPUI.addControl( ControlType::Number, "Wheel Angle Offset", String( steerConfig.wheelAngleOffset ), ControlColor::Peterriver, tab,
      []( Control * control, int id ) {
        steerConfig.wheelAngleOffset = control->value.toFloat();
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
        } );
        ESPUI.addControl( ControlType::Min, "Min", "0", ControlColor::Peterriver, num );
        ESPUI.addControl( ControlType::Max, "Max", "500", ControlColor::Peterriver, num );
        ESPUI.addControl( ControlType::Step, "Step", "1", ControlColor::Peterriver, num );
      }

      {
        uint16_t num = ESPUI.addControl( ControlType::Number, "2. Arm connect to tie rod (mm)", String( steerConfig.wheelAngleSecondArmLenght ), ControlColor::Peterriver, tab,
        []( Control * control, int id ) {
          steerConfig.wheelAngleSecondArmLenght = control->value.toFloat();
        } );
        ESPUI.addControl( ControlType::Min, "Min", "0", ControlColor::Peterriver, num );
        ESPUI.addControl( ControlType::Max, "Max", "500", ControlColor::Peterriver, num );
        ESPUI.addControl( ControlType::Step, "Step", "1", ControlColor::Peterriver, num );
      }

      {
        uint16_t num = ESPUI.addControl( ControlType::Number, "Tie rod stroke (mm)", String( steerConfig.wheelAngleTieRodStroke ), ControlColor::Peterriver, tab,
        []( Control * control, int id ) {
          steerConfig.wheelAngleTieRodStroke = control->value.toFloat();
        } );
        ESPUI.addControl( ControlType::Min, "Min", "0", ControlColor::Peterriver, num );
        ESPUI.addControl( ControlType::Max, "Max", "500", ControlColor::Peterriver, num );
        ESPUI.addControl( ControlType::Step, "Step", "1", ControlColor::Peterriver, num );
      }

      {
        uint16_t num = ESPUI.addControl( ControlType::Number, "Minimum Angle of wheel angle sensor", String( steerConfig.wheelAngleMinimumAngle ), ControlColor::Peterriver, tab,
        []( Control * control, int id ) {
          steerConfig.wheelAngleMinimumAngle = control->value.toFloat();
        } );
        ESPUI.addControl( ControlType::Min, "Min", "0", ControlColor::Peterriver, num );
        ESPUI.addControl( ControlType::Max, "Max", "180", ControlColor::Peterriver, num );
        ESPUI.addControl( ControlType::Step, "Step", "1", ControlColor::Peterriver, num );
      }

      {
        uint16_t num = ESPUI.addControl( ControlType::Number, "Lenght of Track Arm (mm)", String( steerConfig.wheelAngleTrackArmLenght ), ControlColor::Peterriver, tab,
        []( Control * control, int id ) {
          steerConfig.wheelAngleTrackArmLenght = control->value.toFloat();
        } );
        ESPUI.addControl( ControlType::Min, "Min", "0", ControlColor::Peterriver, num );
        ESPUI.addControl( ControlType::Max, "Max", "500", ControlColor::Peterriver, num );
        ESPUI.addControl( ControlType::Step, "Step", "1", ControlColor::Peterriver, num );
      }
    }

    {
      uint16_t num = ESPUI.addControl( ControlType::Number, "Ackermann (%)", String( steerConfig.ackermann ), ControlColor::Peterriver, tab,
      []( Control * control, int id ) {
        steerConfig.ackermann = control->value.toFloat();
      } );
      ESPUI.addControl( ControlType::Min, "Min", "0", ControlColor::Peterriver, num );
      ESPUI.addControl( ControlType::Max, "Max", "200", ControlColor::Peterriver, num );
      ESPUI.addControl( ControlType::Step, "Step", "1", ControlColor::Peterriver, num );
    }

    {
      uint16_t sel = ESPUI.addControl( ControlType::Select, "Ackermann - Wheel with WAS has smaller radius", String( ( int )steerConfig.ackermannAboveZero ), ControlColor::Peterriver, tab,
      []( Control * control, int id ) {
        steerConfig.ackermannAboveZero = control->value.toInt() == 1;
      } );
      ESPUI.addControl( ControlType::Option, "when Actual degrees are below zero", "0", ControlColor::Alizarin, sel );
      ESPUI.addControl( ControlType::Option, "when Actual degrees are above zero", "1", ControlColor::Alizarin, sel );
    }

    {
      uint16_t sel = ESPUI.addControl( ControlType::Select, "WAS Voltage (ADS1115 Gain)*", String( ( int )steerConfig.adsGain ), ControlColor::Wetasphalt, tab,
      []( Control * control, int id ) {
        steerConfig.adsGain = ( SteerConfig::ADSGain )control->value.toInt();
        setResetButtonToRed();
      } );
      ESPUI.addControl( ControlType::Option, "6.144V (2/3x gain)", "0", ControlColor::Alizarin, sel );
      ESPUI.addControl( ControlType::Option, "4.096V (1x gain)", "512", ControlColor::Alizarin, sel );
      ESPUI.addControl( ControlType::Option, "2.048V (2x gain)", "1024", ControlColor::Alizarin, sel );
      ESPUI.addControl( ControlType::Option, "1.024V (4x gain)", "1536", ControlColor::Alizarin, sel );
      ESPUI.addControl( ControlType::Option, "0.512V (8x gain)", "2048", ControlColor::Alizarin, sel );
      ESPUI.addControl( ControlType::Option, "0.256V (16x gain)", "2560", ControlColor::Alizarin, sel );
    }
  }

  // Steering Tab
  {
    uint16_t tab = ESPUI.addControl( ControlType::Tab, "Steering", "Steering" );

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
      } );
      ESPUI.addControl( ControlType::Min, "Min", "0", ControlColor::Peterriver, num );
      ESPUI.addControl( ControlType::Max, "Max", "2", ControlColor::Peterriver, num );
      ESPUI.addControl( ControlType::Step, "Step", "0.1", ControlColor::Peterriver, num );
    }

    if( steerConfig.outputType == SteerConfig::OutputType::HydraulicPwm2Coil ){
      uint16_t num = ESPUI.addControl( ControlType::Number, "Dither", String( steerConfig.dither ), ControlColor::Peterriver, tab,
      []( Control * control, int id ) {
        steerConfig.dither = control->value.toInt();
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
      } );
      ESPUI.addControl( ControlType::Min, "Min", "0", ControlColor::Peterriver, num );
      ESPUI.addControl( ControlType::Max, "Max", "50", ControlColor::Peterriver, num );
      ESPUI.addControl( ControlType::Step, "Step", "0.1", ControlColor::Peterriver, num );
    }
    {
      uint16_t num = ESPUI.addControl( ControlType::Number, "PID Ki", String( steerConfig.steeringPidKi, 2 ), ControlColor::Peterriver, tab,
      []( Control * control, int id ) {
        steerConfig.steeringPidKi = control->value.toDouble();
      } );
      ESPUI.addControl( ControlType::Min, "Min", "0", ControlColor::Peterriver, num );
      ESPUI.addControl( ControlType::Max, "Max", "50", ControlColor::Peterriver, num );
      ESPUI.addControl( ControlType::Step, "Step", "0.01", ControlColor::Peterriver, num );
    }
    {
      uint16_t num = ESPUI.addControl( ControlType::Number, "PID Ki Max", String( steerConfig.steeringPidKiMax, 2 ), ControlColor::Peterriver, tab,
      []( Control * control, int id ) {
        steerConfig.steeringPidKiMax = control->value.toDouble();
      } );
      ESPUI.addControl( ControlType::Min, "Min", "0", ControlColor::Peterriver, num );
      ESPUI.addControl( ControlType::Max, "Max", "255", ControlColor::Peterriver, num );
      ESPUI.addControl( ControlType::Step, "Step", "1.00", ControlColor::Peterriver, num );
    }
    {
      uint16_t num = ESPUI.addControl( ControlType::Number, "PID Kd", String( steerConfig.steeringPidKd, 2 ), ControlColor::Peterriver, tab,
      []( Control * control, int id ) {
        steerConfig.steeringPidKd = control->value.toDouble();
      } );
      ESPUI.addControl( ControlType::Min, "Min", "0", ControlColor::Peterriver, num );
      ESPUI.addControl( ControlType::Max, "Max", "50", ControlColor::Peterriver, num );
      ESPUI.addControl( ControlType::Step, "Step", "0.01", ControlColor::Peterriver, num );
    }
    {
      uint16_t num = ESPUI.addControl( ControlType::Number, "Minimum PWM", String( steerConfig.steeringPidMinPwm ), ControlColor::Peterriver, tab,
      []( Control * control, int id ) {
        steerConfig.steeringPidMinPwm = control->value.toInt();
      } );
      ESPUI.addControl( ControlType::Min, "Min", "0", ControlColor::Peterriver, num );
      ESPUI.addControl( ControlType::Max, "Max", "255", ControlColor::Peterriver, num );
      ESPUI.addControl( ControlType::Step, "Step", "1", ControlColor::Peterriver, num );
    }
    {
      uint16_t num = ESPUI.addControl( ControlType::Number, "Maximum PWM", String( steerConfig.steeringPidMaxPwm ), ControlColor::Peterriver, tab,
      []( Control * control, int id ) {
        steerConfig.steeringPidMaxPwm = control->value.toInt();
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
      } );
      ESPUI.addControl( ControlType::Min, "Min", "0", ControlColor::Peterriver, num );
      ESPUI.addControl( ControlType::Max, "Max", "30", ControlColor::Peterriver, num );
      ESPUI.addControl( ControlType::Step, "Step", "0.5", ControlColor::Peterriver, num );
    }
    {
      uint16_t sel = ESPUI.addControl( ControlType::Select, "Speed units", String( ( int )steerConfig.speedUnits ), ControlColor::Peterriver, tab,
      []( Control * control, int id ) {
        steerConfig.speedUnits =  ( SteerConfig::SpeedUnits )control->value.toInt();
      } );
      ESPUI.addControl( ControlType::Option, "MPH", "0", ControlColor::Alizarin, sel );
      ESPUI.addControl( ControlType::Option, "KPH", "1", ControlColor::Alizarin, sel );
    }
    {
      uint16_t sel = ESPUI.addControl( ControlType::Select, "Disengage Switch Type*", String( ( int )steerConfig.disengageSwitchType ), ControlColor::Wetasphalt, tab,
      []( Control * control, int id ) {
        steerConfig.disengageSwitchType = ( SteerConfig::DisengageSwitchType )control->value.toInt();
        setResetButtonToRed();
      } );
      ESPUI.addControl( ControlType::Option, "Encoder on steering shaft", "0", ControlColor::Alizarin, sel );
      ESPUI.addControl( ControlType::Option, "Pressure switch in hydraulics", "1", ControlColor::Alizarin, sel );
      ESPUI.addControl( ControlType::Option, "Deere variable duty sensor", "2", ControlColor::Alizarin, sel );
      ESPUI.addControl( ControlType::Option, "Motor current", "3", ControlColor::Alizarin, sel );
    }
    {
      uint16_t sel = ESPUI.addControl( ControlType::Switcher, "Heavy Duty Disengage Switch", steerConfig.disengageHeavyDuty ? "1" : "0" , ControlColor::Peterriver, tab,
      []( Control * control, int id ) {
        steerConfig.disengageHeavyDuty = control->value.toInt() == 1;
        pinMode( steerConfig.gpioDisengagePullup, OUTPUT );
        digitalWrite( steerConfig.gpioDisengagePullup, steerConfig.disengageHeavyDuty );
      } );
    }

    switch( steerConfig.disengageSwitchType ){
      case SteerConfig::DisengageSwitchType::Hydraulic: {
        ESPUI.addControl( ControlType::Switcher, "Hydraulic Switch Active Low", steerConfig.hydraulicSwitchActiveLow ? "1" : "0", ControlColor::Peterriver, tab,
        []( Control * control, int id ) {
          steerConfig.hydraulicSwitchActiveLow = control->value.toInt() == 1;
        } );
      }
      break;
      case SteerConfig::DisengageSwitchType::Encoder: {
        {
          uint16_t num = ESPUI.addControl( ControlType::Number, "Steering Wheel Pulses per Frame", String( steerConfig.disengageFramePulses ), ControlColor::Peterriver, tab,
          []( Control * control, int id ) {
            steerConfig.disengageFramePulses = control->value.toInt();
          } );
          ESPUI.addControl( ControlType::Min, "Min", "1", ControlColor::Peterriver, num );
          ESPUI.addControl( ControlType::Max, "Max", "1000", ControlColor::Peterriver, num );
          ESPUI.addControl( ControlType::Step, "Step", "10", ControlColor::Peterriver, num );
        }

        {
          uint16_t num = ESPUI.addControl( ControlType::Number, "Steering Wheel Millis per Frame", String( steerConfig.disengageFrameMillis ), ControlColor::Peterriver, tab,
          []( Control * control, int id ) {
            steerConfig.disengageFrameMillis = control->value.toInt();
          } );
          ESPUI.addControl( ControlType::Min, "Min", "1", ControlColor::Peterriver, num );
          ESPUI.addControl( ControlType::Max, "Max", "10000", ControlColor::Peterriver, num );
          ESPUI.addControl( ControlType::Step, "Step", "1000", ControlColor::Peterriver, num );
        }
      }
      break;
      case SteerConfig::DisengageSwitchType::JDVariableDuty: {
        uint16_t num = ESPUI.addControl( ControlType::Number, "JD PWM sensor duty cycle change", String( steerConfig.JDVariableDutyChange ), ControlColor::Peterriver, tab,
        []( Control * control, int id ) {
          steerConfig.JDVariableDutyChange = control->value.toInt();
        } );
        ESPUI.addControl( ControlType::Min, "Min", "1", ControlColor::Peterriver, num );
        ESPUI.addControl( ControlType::Max, "Max", "100", ControlColor::Peterriver, num );
        ESPUI.addControl( ControlType::Step, "Step", "1", ControlColor::Peterriver, num );
      }
      break;
      case SteerConfig::DisengageSwitchType::MotorCurrent: {
        uint16_t num = ESPUI.addControl( ControlType::Number, "Max Steer Motor Current", String( steerConfig.maxSteerCurrent ), ControlColor::Peterriver, tab,
        []( Control * control, int id ) {
          steerConfig.maxSteerCurrent = control->value.toInt();
        } );
        ESPUI.addControl( ControlType::Min, "Min", "0", ControlColor::Peterriver, num );
        ESPUI.addControl( ControlType::Max, "Max", "4096", ControlColor::Peterriver, num );
        ESPUI.addControl( ControlType::Step, "Step", "10", ControlColor::Peterriver, num );
      }
      break;
     }
  }

  char autosteerDownloadHTML [100];
  sprintf( downloadFilename, "/%s steering.json", steerConfig.hostname );
  sprintf( autosteerDownloadHTML, "<a href='%s'>Configuration</a>", downloadFilename );

  // Default Configurations Tab
  {
    uint16_t tab = ESPUI.addControl( ControlType::Tab, "Configurations", "Configurations" );

    ESPUI.addControl( ControlType::Label, "OTA Update:", "<a href='/update'>Update</a>", ControlColor::Carrot, tab );

    ESPUI.addControl( ControlType::Label, "Download the config:", autosteerDownloadHTML, ControlColor::Carrot, tab );

    ESPUI.addControl( ControlType::Label, "Upload the config:", "<form method='POST' action='/upload-config' enctype='multipart/form-data'><input name='f' type='file'><input type='submit'>ESP32 will restart after submitting</form>", ControlColor::Carrot, tab );
    
    tabConfigurations = tab;

  }
  
  static String title;

  title = "AOG Control :: ";

  title += steerConfig.hostname;
  ESPUI.begin( title.c_str() );

  ESPUI.server->on( downloadFilename, HTTP_GET, []( AsyncWebServerRequest * request ) {
    
    Serial.print( "Preparing " );
    Serial.print( downloadFilename );
    Serial.println( " for download" );

    char ibuffer[64];
    File f1 = SPIFFS.open( "/autosteer.json", "r" );    //open source file to read
    if ( !f1 ){
      Serial.println( "/autosteer.json not available for copying" );
      return;
    }

    File f2 = SPIFFS.open( downloadFilename, "w" );    //open destination file to write
    if ( !f2 ){
      Serial.print( downloadFilename );
      Serial.println( " could not be created" );
      return;
    }
    
    uint8_t blocks = 0;
    Serial.print( "Copied " );
    while ( f1.available() > 0 ){
      byte i = f1.readBytes( ibuffer, 64 ); // i = number of bytes placed in buffer from file f1
      f2.write(( uint8_t* )ibuffer, i );    // write i bytes from buffer to file f2
      blocks += 1;
      Serial.print( blocks );
      Serial.print( " " );
    }
    Serial.println( "blocks" );
    
    f2.close();
    f1.close();
    Serial.println( "File creation successful, downloading..." );
    delay( 5 );
    request->send( SPIFFS, downloadFilename, "application/json", true );
    delay( 5 );
    SPIFFS.remove( downloadFilename );
  } );
  
  // upload a file to /upload-config
  ESPUI.server->on( "/upload-config", HTTP_POST, []( AsyncWebServerRequest * request ) {
    request->send( 200 );
  }, [tabConfigurations]( AsyncWebServerRequest * request, String filename, size_t index, uint8_t* data, size_t len, bool final ) {
    if( !index ) {
      request->_tempFile = SPIFFS.open( "/autosteer.json", "w" );
    }

    if( request->_tempFile ) {
      if( len ) {
        request->_tempFile.write( data, len );
      }

      if( final ) {
        request->_tempFile.close();
        if( steerConfig.canBusEnabled ){
          ESP32Can.CANStop();
        }
        delay(10);
        ESP.restart();
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
