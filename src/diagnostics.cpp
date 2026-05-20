
#include <stdio.h>
#include <string.h>

#include "main.hpp"
#include "jsonFunctions.hpp"

void diagnosticWorker1Hz( void* z ) {
  vTaskDelay( 2000 );
  constexpr TickType_t xFrequency = 1000;
  TickType_t xLastWakeTime = xTaskGetTickCount();
  for( ;; ) {

    {
      String str;
      str.reserve( 30 );
      str = "\nSpeed: ";
      str += ( float )steerSetpoints.speed;
      if( ( SteerConfig::SpeedUnits )steerConfig.speedUnits == SteerConfig::SpeedUnits::MilesPerHour ) {
        str += " MPH";
      } else {
        str += " KPH";
      }
      str += "\nEnable autosteer timed out: ";
      str += ( bool )safety.AOGEnableAutosteerTimeout ? "Yes" : "No" ;
      str += "\nDisabled by max speed: ";
      str += ( bool )safety.autosteerDisabledByMaxEngageSpeed ? "Yes" : "No" ;
      str += "\nDisabled by min speed: ";
      str += ( bool )steerSetpoints.speed < steerConfig.minAutosteerSpeed ? "Yes" : "No" ;
      str += "\nDisabled by safety lock: ";
      str += ( bool )machine.autosteerSafetyLock ? "Yes" : "No" ;
      str += "\nDisabled by steering wheel: ";
      if( machine.disengagedBySteeringWheel ){
        str += "Yes ";
        str += machine.DeereDutyDisengage;
        str += " micros ";
        time_t elapsed = ( millis() - machine.lastDisengageMillis );
        str += elapsed / 1000;
        str += " seconds ago";

      } else str += "No";
      diagnosticsDisplay.safetyDisableAutosteer = str;
    }
    if( machine.canbusSteeringActive == false ){ // voltage monitoring during legacy steering only
      String str;
      str.reserve( 30 );
      str = "\n";
      str += ( uint16_t ) machine.steerSupplyVoltage ;
      str += " counts; ";
      str += ( double ) ( ( double ) ( machine.steerSupplyVoltage * 4.54 ) / 913 ); //divide by 913 for ESP32; 10k/2.2k = 4.54
      str += " volts\n";
      str += ( double ) ( ( double ) ( diagnostics.steerSupplyVoltageMin * 4.54 ) / 913 );
      str += " volts min while steering\n";
      str += ( double ) ( ( double ) ( diagnostics.steerSupplyVoltageMax * 4.54 ) / 913 );
      str += " volts max while steering";
      diagnosticsDisplay.supplyVoltage = str;
    }
    {
      String str;
      str.reserve( 30 );
      str = "\nCurrent: ";
      str += ( uint16_t ) machine.steerMotorCurrent;
      str += "\nOverlimit: ";
      str += ( machine.steerMotorCurrent > steerConfig.maxSteerCurrent ) ? "Yes" : "No";
      diagnosticsDisplay.steerMotorCurrent = str;
    }
    {
      String str;
      str.reserve( 30 );
      str = "\nPlausibility errors: ";
      str += ( uint8_t ) diagnostics.WasPlausibilityErrors;
      str += "\nPower supply shorted to ground: ";
      str += ( uint8_t ) diagnostics.WasPositiveSupplyShortedToGround;
      diagnosticsDisplay.wheelAngleSensor = str;
    }
    {
      String str;
      str.reserve( 30 );
      if( steerConfig.wheelAngleInput == SteerConfig::AnalogIn::CanbusFendt ){
        str = "Fendt Canbus WAS counts: ";
        str += ( uint16_t )machine.canbusWasCounts;
        str += "\nActual: ";
        str += ( float )steerSetpoints.actualSteerAngle;
        str += "°, SetPoint: ";
        str += ( float )steerSetpoints.requestedSteerAngle;
        str += "°\n";
        time_t elapsed = millis() - machine.lastCanbusWasMillis;
        if( elapsed < 1000 ){
          str += ( time_t )elapsed;
          str += " millis ago";
        } else {
          str += ( time_t )elapsed / 1000;
          str += " seconds ago";
        }
      } else if( steerConfig.wheelAngleInput == SteerConfig::AnalogIn::CanbusValtraMasseyChallenger ){
        str = "Valtra-Massey-Challenger Canbus WAS counts: ";
        str += ( uint16_t )machine.canbusWasCounts;
        str += "\nActual: ";
        str += ( float )steerSetpoints.actualSteerAngle;
        str += "°, SetPoint: ";
        str += ( float )steerSetpoints.requestedSteerAngle;
        str += "°\n";
        time_t elapsed = millis() - machine.lastCanbusWasMillis;
        if( elapsed < 1000 ){
          str += ( time_t )elapsed;
          str += " millis ago";
        } else {
          str += ( time_t )elapsed / 1000;
          str += " seconds ago";
        }
      } else {
        if( steerConfig.wheelAngleSensorType == SteerConfig::WheelAngleSensorType::TieRodDisplacement ) {
          str += ( float )steerSetpoints.actualSteerAngle;
          str += "°, Raw ";
          str += ( float )steerSetpoints.wheelAngleRaw;
          str += "°, Displacement ";
          str += ( float )steerSetpoints.wheelAngleCurrentDisplacement;
          str += "mm\n";
        } else {
          str += "A/D count: ";
          str += ( int )steerSetpoints.wheelAngleCounts;
          str += ", Raw: ";
          str += ( float )steerSetpoints.wheelAngleRaw;
          str += "°\nActual: ";
          str += ( float )steerSetpoints.actualSteerAngle;
          str += "°, SetPoint: ";
          str += ( float )steerSetpoints.requestedSteerAngle;
          str += "°\n";
        }
        if( steerConfig.adsGain == SteerConfig::ADSGain::GAIN_TWOTHIRDS ){
          str += ( float )( steerSetpoints.wheelAngleCounts * 0.0001875 );
          str += " volts from WAS";
        }
        else if( steerConfig.adsGain == SteerConfig::ADSGain::GAIN_ONE ){
          str += ( float )( steerSetpoints.wheelAngleCounts * 1.589 * 0.000125 ); // Sensor - 3.3K - ADS - 5.6K - Gnd
          str += " volts from WAS";
        }
      }
      ESPUI.updateLabel( labelWheelAngle, str );
    }
    {
      String str;
      str.reserve( 30 );
      if( steerConfig.outputType == SteerConfig::OutputType::Canbus13_19Controller ){
        str = "\nCanbus steer state: ";
        time_t elapsed = millis() - machine.lastCanbusSteeringMillis;
        if( digitalRead( ( uint8_t )steerConfig.gpioSteerswitch ) == steerConfig.steerswitchActiveLow ){
          str += "override switch pressed";
        } else if( elapsed > 500 ){
          str += " timeout ";
          str += ( time_t ) elapsed;
          str += " millis ago";
        } else {
          switch( machine.canbusSteeringState ){

            case 0x00: {
              str += "handwheel activity";
            }
            break;

            case 0x10: {
              str += "ready";
            }
            break;

            case 0x14: {
              str += "engaged";
            }
            break;

            case 0x20: {
              str += "stagnant/turn handwheel";
            }
            break;

            case 0x50: {
              str += "no message from steering controller";
            }
            break;

            case 0x60: {
              str += "not initialized";
            }
            break;

            default: {
              str += "undefined value ";
              str += machine.canbusSteeringState;
            }
            break;
          }
        }
      } else {
        str = steerConfig.steerSwitchIsMomentary ? "\nMomentary" : "\nMaintained";
        str += " steer switch: ";
        str += ( bool )( digitalRead( steerConfig.gpioSteerswitch ) != steerConfig.steerswitchActiveLow ) ? "On " : "Off " ;
        time_t elapsed = millis() - machine.lastAutosteerMillis;
        if( elapsed < 1000 ){
          str += ( time_t )elapsed;
          str += " millis ago";
        } else {
          str += ( time_t )elapsed / 1000;
          str += " seconds ago";
        }
      }
      if( steerConfig.workswitchType > SteerConfig::WorkswitchType::Gpio ){ // Canbus function
        str += "\nCanbus work function: ";
        str += ( bool ) machine.workswitchState ? "On" : "Off" ;
      } else {
        str += "\nWork switch: ";
        str += ( bool )( digitalRead( steerConfig.gpioWorkswitch ) != steerConfig.workswitchActiveLow ) ? "On" : "Off" ;
      }
      switch( steerConfig.disengageSwitchType ) {
        case SteerConfig::DisengageSwitchType::Encoder: {
          str += "\nEncoder on steering wheel: ";
          str += ( bool )digitalRead( steerConfig.gpioDisengage ) ? "Off" : "On" ; // disengage is inverted
          str += " / ";
          str += machine.handwheelPulseCount;
          str += " counts";
        }
        break;

        case SteerConfig::DisengageSwitchType::Hydraulic: {
          str += "\nHydraulic disengage switch: ";
          str += ( bool )( digitalRead( steerConfig.gpioDisengage ) == steerConfig.hydraulicSwitchActiveLow ) ? "On" : "Off" ;
        }
        break;

        case SteerConfig::DisengageSwitchType::JDVariableDuty: {
          str += "\nDeere variable duty encoder: ";
          str += ( uint16_t )( abs( machine.DeereDutyAverage - machine.DeereDutyCycle ) );
          str += " micros";
        }
        break;
      }
      if( steerConfig.disengageSwitchType != SteerConfig::DisengageSwitchType::JDVariableDuty ){
        str += " ";
        time_t elapsed = millis() - machine.lastDisengageMillis;
        if( elapsed < 1000 ){
          str += ( time_t )elapsed;
          str += " millis ago";
        } else {
          str += ( time_t )elapsed / 1000;
          str += " seconds ago";
        }
      }
      diagnosticsDisplay.switchStates = str;
    }
    {
      String str;
      str.reserve( 30 );
      str = "\nTool lift ";
      if( hydLift == 0 ){
        str = "disabled ";
      } else if( hydLift == 1 ){
        str = "down ";
      } else if( hydLift == 2 ){
        str = "up ";
      }
      time_t elapsed = ( millis() - lastHydLiftMillis ) / 1000;
      str += ( time_t )elapsed;
      str += " seconds ago";
      diagnosticsDisplay.implementStates = str;
    }
    {
      switch( steerConfig.outputType ) {
        case SteerConfig::OutputType::SteeringMotorIBT2: {
          Control* labelStatusOutputHandle = ESPUI.getControl( labelStatusOutput );
          String str;
          str.reserve( 30 );
          str = "IBT2 Motor, SetPoint: ";
          str += ( float )steerSetpoints.requestedSteerAngle;
          str += "°,\ntimeout: ";
          str += ( bool )( steerSetpoints.lastPacketReceived < safety.timeoutPoint ) ? "Yes" : "No" ;
          str += ", enabled: ";
          str += ( bool )steerSetpoints.enabled ? "Yes" : "No" ;
          str += ", output: ";
          str += ( int16_t )machine.valveOutput ;
          labelStatusOutputHandle->color = ControlColor::Emerald;
          ESPUI.updateLabel( labelStatusOutput, str );
        }
        break;

        case SteerConfig::OutputType::SteeringMotorCytron: {
          Control* labelStatusOutputHandle = ESPUI.getControl( labelStatusOutput );
          String str;
          str.reserve( 30 );
          str = "Cytron Motor, SetPoint: ";
          str += ( float )steerSetpoints.requestedSteerAngle;
          str += "°\ntimeout: ";
          str += ( bool )( steerSetpoints.lastPacketReceived < safety.timeoutPoint ) ? "Yes" : "No" ;
          str += ", enabled: ";
          str += ( bool )steerSetpoints.enabled ? "Yes" : "No" ;
          str += ", output: ";
          str += ( int16_t )machine.valveOutput;
          labelStatusOutputHandle->color = ControlColor::Emerald;
          ESPUI.updateLabel( labelStatusOutput, str );
        }
        break;

        case SteerConfig::OutputType::HydraulicPwm2Coil: {
          Control* labelStatusOutputHandle = ESPUI.getControl( labelStatusOutput );
          String str;
          str.reserve( 30 );
          str = "IBT2 Hydraulic PWM 2 Coil, SetPoint: ";
          str += ( float )steerSetpoints.requestedSteerAngle;
          str += "°,\ntimeout: ";
          str += ( bool )( steerSetpoints.lastPacketReceived < safety.timeoutPoint ) ? "Yes" : "No" ;
          str += ", enabled: ";
          str += ( bool )steerSetpoints.enabled ? "Yes" : "No" ;
          str += ",\n output: ";
          str += ( int16_t )machine.valveOutput ;
          str += ", dither: ";
          str += ( float )ditherAmount ;
          labelStatusOutputHandle->color = ControlColor::Emerald;
          ESPUI.updateLabel( labelStatusOutput, str );
        }
        break;

        case SteerConfig::OutputType::HydraulicDanfoss: {
          Control* labelStatusOutputHandle = ESPUI.getControl( labelStatusOutput );
          String str;
          str.reserve( 30 );
          str = "IBT2 Hydraulic Danfoss, SetPoint: ";
          str += ( float )steerSetpoints.requestedSteerAngle;
          str += "°,\ntimeout: ";
          str += ( bool )( steerSetpoints.lastPacketReceived < safety.timeoutPoint ) ? "Yes" : "No" ;
          str += ", enabled: ";
          str += ( bool )steerSetpoints.enabled ? "Yes" : "No" ;
          str += ",\n output: ";
          str += ( int16_t )machine.valveOutput ;
          labelStatusOutputHandle->color = ControlColor::Emerald;
          ESPUI.updateLabel( labelStatusOutput, str );
        }
        break;

        case SteerConfig::OutputType::HydraulicBangBang: {
          Control* labelStatusOutputHandle = ESPUI.getControl( labelStatusOutput );
          String str;
          str.reserve( 30 );
          str = "IBT2 Hydraulic Bang Bang, SetPoint: ";
          str += ( float )steerSetpoints.requestedSteerAngle;
          str += "°,\ntimeout: ";
          str += ( bool )( steerSetpoints.lastPacketReceived < safety.timeoutPoint ) ? "Yes" : "No" ;
          str += ", enabled: ";
          str += ( bool )steerSetpoints.enabled ? "Yes" : "No" ;
          str += ", output: ";
          str += ( int16_t )machine.valveOutput ;
          labelStatusOutputHandle->color = ControlColor::Emerald;
          ESPUI.updateLabel( labelStatusOutput, str );
        }
        break;

        case SteerConfig::OutputType::Canbus13_19Controller: {
          Control* labelStatusOutputHandle = ESPUI.getControl( labelStatusOutput );
          String str;
          str.reserve( 30 );
          str = "Canbus 13/19 Controller, SetPoint: ";
          str += ( float )steerSetpoints.requestedSteerAngle;
          str += "°,\ntimeout: ";
          str += ( bool )( steerSetpoints.lastPacketReceived < safety.timeoutPoint ) ? "Yes" : "No" ;
          str += ", enabled: ";
          str += ( bool )steerSetpoints.enabled ? "Yes" : "No" ;
          str += ", output: ";
          str += ( double )steerSetpoints.pidOutput ;
          labelStatusOutputHandle->color = ControlColor::Emerald;
          ESPUI.updateLabel( labelStatusOutput, str );
        }
        break;

        case SteerConfig::OutputType::CanbusF0_240Controller: {
          Control* labelStatusOutputHandle = ESPUI.getControl( labelStatusOutput );
          String str;
          str.reserve( 30 );
          str = "Canbus F0/240 Controller, SetPoint: ";
          str += ( float )steerSetpoints.requestedSteerAngle;
          str += "°,\ntimeout: ";
          str += ( bool )( steerSetpoints.lastPacketReceived < safety.timeoutPoint ) ? "Yes" : "No" ;
          str += ", enabled: ";
          str += ( bool )steerSetpoints.enabled ? "Yes" : "No" ;
          str += ", PID output: ";
          str += ( double )steerSetpoints.pidOutput ;
          str += "\nCanbus valve setpoint: ";
          str += ( int16_t )machine.valveOutput ;
          labelStatusOutputHandle->color = ControlColor::Emerald;
          ESPUI.updateLabel( labelStatusOutput, str );
        }
        break;

        default:
          break;

        }
    }
    
    Control* labelAgOpenGpsAddressHandle = ESPUI.getControl( labelAgOpenGpsAddress );
    time_t seconds = ( millis() - lastHelloReceivedMillis ) / 1000;
    String str;
    str.reserve( 30 );
    str = "\nIP Address ";
    str += ipDestination.toString();
    str += " ";
    str += ( String )seconds;
    str += " seconds ago\n";
    str += diagnostics.UDPTimeout;
    str += " UDP timeouts\nUDP received ";
    if( millis() - steerSetpoints.lastPacketReceived > 1000 ){
      str += ( String )(( millis() - steerSetpoints.lastPacketReceived ) / 1000 );
      str += " seconds ago";
    } else {
      str += ( String )( steerSetpoints.lastPacketReceived - steerSetpoints.previousPacketReceived );
      str += " millis apart";
    }
    diagnosticsDisplay.agOpenGpsAddress = str;
    
    Control* labelRowSenseHandle = ESPUI.getControl( labelRowSense );
    if( steerConfig.enableRowSense ){
      str = "Voltage: ";
      str += ( double )machine.rowSenseVoltageOne;
      str += " & ";
      str += ( double )machine.rowSenseVoltageTwo;
      str += "\nCounts: ";
      str += ( uint16_t )steerSetpoints.rowSenseCounts;
      str += "\nAngle: ";
      str += ( double )steerSetpoints.rowSenseAngle;
      if( steerSetpoints.rowSenseCounts == 0 ){
        labelRowSenseHandle->color = ControlColor::Alizarin;
      } else labelRowSenseHandle->color = ControlColor::Emerald;
    } else {
      str = "Not enabled";
      labelRowSenseHandle->color = ControlColor::Turquoise;
    }
    
    labelRowSenseHandle->value = str;
    ESPUI.updateControl( labelRowSenseHandle );

    vTaskDelayUntil( &xLastWakeTime, xFrequency );
  }
}

void initDiagnostics() {

  String str;
  str.reserve( 30 );
  str = "\nNumber of faults: ";
  str += ( int8_t ) diagnostics.steerEnabledWithNoPower;
  str += "\nFault active since startup: No";
  diagnosticsDisplay.steerEngagedFaults = str;

  xTaskCreate( diagnosticWorker1Hz, "diagnosticWorker", 3096, NULL, 3, NULL );
}
