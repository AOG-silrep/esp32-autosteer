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

#include <stdio.h>
#include <string.h>

#include <AutoPID.h>

#include "main.hpp"
#include "jsonFunctions.hpp"
#include "driver/gpio.h"

#include <string>       // std::string
#include <sstream>      // std::stringstream

#include "soc/mcpwm_reg.h"
#include "soc/mcpwm_struct.h"
#include "driver/mcpwm.h"

SteerSettings steerSettings;
SteerSetpoints steerSetpoints;
SteerMachineControl steerMachineControl;

AsyncUDP udpSendFrom;
AsyncUDP udpLocalPort;

double pidOutput = 0;
double pidOutputTmp = 0;
AutoPID pid(
        &( steerSetpoints.actualSteerAngle ),
        &( steerSetpoints.requestedSteerAngle ),
        &( pidOutput ),
        -255, 255,
        steerConfig.steeringPidKp, steerConfig.steeringPidKi, steerConfig.steeringPidKd );

constexpr time_t Timeout = 1000;
time_t lastSwitchChangeMillis;
volatile bool disengagePrevState;
volatile bool disengagePrevJdState;
volatile time_t disengageActivityMillis = millis();
volatile uint16_t dutyCycle;
uint16_t dutyAverage;
volatile time_t DRAM_ATTR disengageActivityMicros;
volatile time_t DRAM_ATTR onTime;
volatile time_t DRAM_ATTR offTime;

bool previousAogEnabledState = false;
bool AogToMachineEngagedMismatch = false; // do not update machine engaged state from AOG after switch change until round trip is completed
bool ditherDirection = false;
bool dtcAutosteerPrevious = false;

void ditherWorker10HZ( void* z ) {

  for( ;; ) {
    if( ditherDirection == true ){
      ditherAmount += 1;
    } else {
      ditherAmount -= 1;
    }
    if( ditherAmount >= steerConfig.dither ){
      ditherDirection = false;
    } else if ( ditherAmount <= 0 ){
      ditherDirection = true;
    }
    vTaskDelay( pdMS_TO_TICKS( 100 ) );
  }
  vTaskDelete( NULL );
}

void autosteerWorker100Hz( void* z ) {
  constexpr TickType_t xFrequency = 10;
  TickType_t xLastWakeTime = xTaskGetTickCount();

  pid.setTimeStep( xFrequency );

  for( ;; ) {
    safety.timeoutPoint = millis() - Timeout;

    if( steerSetpoints.enabled == true ){
      if( steerConfig.wheelAngleInput < SteerConfig::AnalogIn::CanbusValtraMasseyChallenger ){
        if( dtcAutosteerPrevious == false && machine.steerSupplyVoltage < 2172 ){  // 10.8 volts
          diagnostics.steerEnabledWithNoPower += 1;

          Control* labelSteerEngagedFaultsHandle = ESPUI.getControl( labelSteerEngagedFaults );
          String str;
          str.reserve( 30 );
          str = "Number of faults: ";
          str += ( int8_t ) diagnostics.steerEnabledWithNoPower;
          str += "\nFault active since startup: Yes";
          labelSteerEngagedFaultsHandle->color = ControlColor::Alizarin;
          ESPUI.updateLabel( labelSteerEngagedFaults, str );
          saveDiagnostics();
        }
      }
    }
    dtcAutosteerPrevious = steerSetpoints.enabled;

    if( steerConfig.manualSteerState == true ){
      if( steerConfig.outputType == SteerConfig::OutputType::HydraulicDanfoss ){
        uint8_t lowRange = 255 - steerConfig.steeringPidMaxPwm; 
        if( steerConfig.manualPWM > steerConfig.steeringPidMaxPwm ){
          steerConfig.manualPWM = steerConfig.steeringPidMaxPwm;
          ESPUI.updateNumber( manualValvePWMWidget, steerConfig.manualPWM );
        } else if( steerConfig.manualPWM < lowRange ){
          steerConfig.manualPWM = lowRange;
          ESPUI.updateNumber( manualValvePWMWidget, steerConfig.manualPWM );
        }
      } else {
        if( steerConfig.manualPWM > 255 ){
          steerConfig.manualPWM = 255;
          ESPUI.updateNumber( manualValvePWMWidget, steerConfig.manualPWM);
        } else if( steerConfig.manualPWM < -255 ){
          steerConfig.manualPWM = -255;
          ESPUI.updateNumber( manualValvePWMWidget, steerConfig.manualPWM );
        }
      }
      switch( initialisation.outputType ) {
        case SteerConfig::OutputType::SteeringMotorIBT2:
        case SteerConfig::OutputType::HydraulicPwm2Coil: {
          ledcWrite( 0, 0 );
          if( steerConfig.manualPWM >= 0 ) {
            ledcWrite( 1, steerConfig.manualPWM );
            ledcWrite( 2, 0 );
          }
          if( steerConfig.manualPWM < 0 ) {
            ledcWrite( 1, 0 );
            ledcWrite( 2, -steerConfig.manualPWM );
          }
          digitalWrite( steerConfig.gpioEn, HIGH );
        }
        break;

        case SteerConfig::OutputType::SteeringMotorCytron: {
          if( steerConfig.manualPWM >= 0 ) {
            ledcWrite( 1, 255 );
          } else {
            ledcWrite( 0, 255 );
            steerConfig.manualPWM = -steerConfig.manualPWM;
          }
          ledcWrite( 0, steerConfig.manualPWM );
          ledcWrite( 2, 255 );
          digitalWrite( steerConfig.gpioEn, HIGH );
        }
        break;

        case SteerConfig::OutputType::HydraulicDanfoss: {
          ledcWrite( 0, steerConfig.manualPWM );
          ledcWrite( 1, 255 );
          ledcWrite( 2, 255 );
          digitalWrite( steerConfig.gpioEn, HIGH );
        }
        break;

        default:
          break;

      }
    }
    // check for timeout, data from AgOpenGPS, safety disable, and mininum autosteer speed
    else if( steerSetpoints.lastPacketReceived < safety.timeoutPoint ||
             steerSetpoints.enabled == false ||
             steerSetpoints.speed < steerConfig.minAutosteerSpeed ||
             machine.autosteerSafetyLock == true ) {
      
      switch( initialisation.outputType ) {
        case SteerConfig::OutputType::CanbusF0_240Controller: {
          pidOutputTmp = 0;
          ledcWrite( 0, 0 );
          ledcWrite( 1, 0 );
          ledcWrite( 2, 0 );
          digitalWrite( steerConfig.gpioEn, LOW );
          machine.valveOutput = machine.canbusWasCounts;
          steerSetpoints.pidOutput = pidOutputTmp;
        }
        break;

        case SteerConfig::OutputType::Canbus13_19Controller: {
          pidOutputTmp = 0;
          ledcWrite( 0, 0 );
          ledcWrite( 1, 0 );
          ledcWrite( 2, 0 );
          digitalWrite( steerConfig.gpioEn, LOW );
          machine.valveOutput = machine.canbusWasCounts;
          steerSetpoints.pidOutput = pidOutputTmp;
        }
        break;

        case SteerConfig::OutputType::HydraulicDanfoss: {
          pidOutputTmp = 128;
          ledcWrite( 0, 128 );
          ledcWrite( 1, 0 );
          ledcWrite( 2, 0 );
          digitalWrite( steerConfig.gpioEn, LOW );
          machine.valveOutput = pidOutputTmp;
          steerSetpoints.pidOutput = pidOutputTmp;
        }
        break;

        default: {
          pidOutputTmp = 0;
          ledcWrite( 0, 0 );
          ledcWrite( 1, 0 );
          ledcWrite( 2, 0 );
          digitalWrite( steerConfig.gpioEn, LOW );
          machine.valveOutput = pidOutputTmp;
          steerSetpoints.pidOutput = pidOutputTmp;
        }
        break;
      }

      digitalWrite( steerConfig.gpioSteerLED, LOW );
    } else {

      diagnostics.steerSupplyVoltageMax = max( machine.steerSupplyVoltage, diagnostics.steerSupplyVoltageMax );
      diagnostics.steerSupplyVoltageMin = min( machine.steerSupplyVoltage, diagnostics.steerSupplyVoltageMin );

      pid.setGains( steerConfig.steeringPidKp, steerConfig.steeringPidKi, steerConfig.steeringPidKd );

      if( pid.getIntegral() > 0 && pid.getIntegral() > steerConfig.steeringPidKiMax ){
        pid.setIntegral(steerConfig.steeringPidKiMax);
      }
      if( -pid.getIntegral() > 0 && -pid.getIntegral() > steerConfig.steeringPidKiMax ){
        pid.setIntegral(-steerConfig.steeringPidKiMax);
      }

      // here comes the magic: executing the PID loop
      // the values are given by pointers, so the AutoPID gets them automaticaly
      pid.run();
 
      pidOutputTmp = steerConfig.invertOutput ? pidOutput : -pidOutput;

      if( pidOutputTmp < 0 ) {
        pidOutputTmp = map( pidOutputTmp, -255, 0, -steerConfig.steeringPidMaxPwm, -steerConfig.steeringPidMinPwm );
        pidOutputTmp = constrain( pidOutputTmp, -steerConfig.steeringPidMaxPwm, -steerConfig.steeringPidMinPwm );
        pidOutputTmp += ditherAmount; // only valid for Hydraulic Pwm 2 Coil, don't decrease PWM output below 255
      }

      if( pidOutputTmp > 0 ) {
        pidOutputTmp = map( pidOutputTmp, 0, 255, steerConfig.steeringPidMinPwm, steerConfig.steeringPidMaxPwm );
        pidOutputTmp = constrain( pidOutputTmp, steerConfig.steeringPidMinPwm, steerConfig.steeringPidMaxPwm );
        pidOutputTmp -= ditherAmount; // only valid for Hydraulic Pwm 2 Coil, don't increase PWM output above 255
      }

      switch( initialisation.outputType ) {
        case SteerConfig::OutputType::SteeringMotorIBT2:
        case SteerConfig::OutputType::HydraulicPwm2Coil: {
          ledcWrite( 0, 0 );
          if( pidOutputTmp >= 0 ) {
            ledcWrite( 1, pidOutputTmp );
            ledcWrite( 2, 0 );
          }
          if( pidOutputTmp < 0 ) {
            ledcWrite( 1, 0 );
            ledcWrite( 2, -pidOutputTmp );
          }
          digitalWrite( steerConfig.gpioEn, HIGH );
          machine.valveOutput = pidOutputTmp;
          steerSetpoints.pidOutput = pidOutputTmp;
        }
        break;

        case SteerConfig::OutputType::SteeringMotorCytron: {
          if( pidOutputTmp >= 0 ) {
            ledcWrite( 1, 255 );
          } else {
            ledcWrite( 0, 255 );
            pidOutputTmp = -pidOutputTmp;
          }

          ledcWrite( 0, pidOutputTmp );
          ledcWrite( 2, 255 );
          machine.valveOutput = pidOutputTmp;
          steerSetpoints.pidOutput = pidOutputTmp;
        }
        break;

        case SteerConfig::OutputType::HydraulicDanfoss: {
          uint8_t lowRange = 255 - steerConfig.steeringPidMaxPwm;
          pidOutputTmp = map( pidOutputTmp, -steerConfig.steeringPidMaxPwm, steerConfig.steeringPidMaxPwm, lowRange, steerConfig.steeringPidMaxPwm );
          ledcWrite( 0, pidOutputTmp );
          ledcWrite( 1, 255 );
          ledcWrite( 2, 255 );
          machine.valveOutput = pidOutputTmp;
          steerSetpoints.pidOutput = pidOutputTmp;
        }
        break;

        case SteerConfig::OutputType::Canbus13_19Controller: {
          ledcWrite( 0, abs( pidOutputTmp ));
          ledcWrite( 1, 0 );
          ledcWrite( 2, 0 );
          machine.valveOutput = machine.canbusWasCounts - pidOutputTmp; //send current position of wheels minus PID output
          steerSetpoints.pidOutput = pidOutputTmp;
        }
        break;

        case SteerConfig::OutputType::CanbusF0_240Controller: {
          ledcWrite( 0, abs( pidOutputTmp ));
          ledcWrite( 1, 0 );
          ledcWrite( 2, 0 );
          machine.valveOutput = machine.canbusWasCounts - ( uint32_t )( pidOutputTmp * 10 ); // Fendt uses 16 bits, otherwise same as VMC family
          steerSetpoints.pidOutput = pidOutputTmp;
        }
        break;

        default:
          break;

      }
      digitalWrite( steerConfig.gpioEn, HIGH );
      digitalWrite( steerConfig.gpioSteerLED, HIGH );
    }

    static uint8_t loopCounter = 0;

    if( ++loopCounter >= 10 ) {
      loopCounter = 0;

      uint8_t data[14] = {0};

      data[0] = 0x80; // AOG specific
      data[1] = 0x81; // AOG specific
      data[2] = 0x7F; // autosteer module to AOG
      data[3] = 0xFD; // autosteer module to AOG
      data[4] = 8;    // length of data

      {
        int16_t steerAngle = steerSetpoints.actualSteerAngle * 100 ;
        data[5] = ( uint16_t )steerAngle;
        data[6] = ( uint16_t )steerAngle >> 8;
      }

      // read inputs
      {
        if( steerConfig.workswitchType != SteerConfig::WorkswitchType::None ) {
          uint16_t value = 0;
          uint16_t threshold = 0;
          uint16_t hysteresis = 0;

          switch( steerConfig.workswitchType ) {
            case SteerConfig::WorkswitchType::Gpio:
              value =  digitalRead( ( uint8_t )steerConfig.gpioWorkswitch ) ? 1 : 0;
              threshold = 1;
              hysteresis = 0;
              break;

            case SteerConfig::WorkswitchType::RearHitchPosition:
              value = steerCanData.rearHitchPosition;
              threshold = steerConfig.canBusHitchThreshold;
              hysteresis = steerConfig.canBusHitchThresholdHysteresis;
              break;

            case SteerConfig::WorkswitchType::FrontHitchPosition:
              value = steerCanData.frontHitchPosition;
              threshold = steerConfig.canBusHitchThreshold;
              hysteresis = steerConfig.canBusHitchThresholdHysteresis;
              break;

            case SteerConfig::WorkswitchType::RearPtoRpm:
              value = steerCanData.rearPtoRpm;
              threshold = steerConfig.canBusRpmThreshold;
              hysteresis = steerConfig.canBusRpmThresholdHysteresis;
              break;

            case SteerConfig::WorkswitchType::FrontPtoRpm:
              value = steerCanData.frontPtoRpm;
              threshold = steerConfig.canBusRpmThreshold;
              hysteresis = steerConfig.canBusRpmThresholdHysteresis;
              break;

            case SteerConfig::WorkswitchType::MotorRpm:
              value = steerCanData.motorRpm;
              threshold = steerConfig.canBusRpmThreshold;
              hysteresis = steerConfig.canBusRpmThresholdHysteresis;
              break;

            default:
              break;
          }

          if( value >= threshold ) {
            machine.workswitchState = true;
          }

          if( value < ( threshold - hysteresis ) ) {
            machine.workswitchState = false;
          }

          if( steerConfig.workswitchActiveLow ) {
            machine.workswitchState = ! machine.workswitchState;
          }

          data[11] |= machine.workswitchState ? 1 : 0;
          digitalWrite( steerConfig.gpioWorkLED, !machine.workswitchState);
        }

          if(( machine.steeringEnabled == false || steerSetpoints.enabled == false ) && steerConfig.manualSteerState == true ){
            steerConfig.manualSteerState = false;
            ESPUI.updateSwitcher( manualValveSwitcher, false );
          }
      }

      // when user presses AOG software button and it is unsafe to autosteer, disable autosteer again
      if( millis() - steerSetpoints.lastEngagedChangeMillis < 300 && machine.autosteerSafetyLock == true && steerSetpoints.enabled == true ) {
        data[11] |= 0; // send `on` for 300 millis then send actual state again afterwards
      } else {
        data[11] |= machine.steeringEnabled ? 0 : 2;
      }
      data[12] = pidOutputTmp; // PWM
      //add the checksum
      int CRCtoAOG = 0;
      for (byte i = 2; i < sizeof(data) - 1; i++)
      {
        CRCtoAOG = (CRCtoAOG + data[i]);
      }
      data[sizeof(data) - 1] = CRCtoAOG;

      udpSendFrom.broadcastTo( data, sizeof( data ), initialisation.portSendTo );

    }
    vTaskDelayUntil( &xLastWakeTime, xFrequency );
  }
}

void autosteerSwitchesWorker1000Hz( void* z ) {
  constexpr TickType_t xFrequency = 1;
  TickType_t xLastWakeTime = xTaskGetTickCount();
  bool previousState;
  bool switchState;

  switchState = digitalRead( ( uint8_t )steerConfig.gpioSteerswitch ); // initialize switch state on startup

  for( ;; ) {
    bool state = digitalRead( ( uint8_t )steerConfig.gpioSteerswitch);
    if( state != previousState ){
      machine.lastAutosteerMillis = millis();
      previousState = state;
    }
    if( millis() - machine.lastAutosteerMillis > 50 and switchState != state ){
      switchState = state;
      lastSwitchChangeMillis = millis();
      if( steerConfig.steerSwitchIsMomentary ){
        if( switchState == steerConfig.steerswitchActiveLow ){
          if( steerSetpoints.speed > steerConfig.maxAutosteerSpeed ) {
            machine.steeringEnabled = false;
            safety.autosteerDisabledByMaxEngageSpeed = true;
          } else {
            machine.steeringEnabled = !machine.steeringEnabled;
            AogToMachineEngagedMismatch = true;
            if( machine.steeringEnabled == true ){
              machine.disengagedBySteeringWheel = false;
              safety.AOGEnableAutosteerTimeout = false;
              safety.autosteerDisabledByMaxEngageSpeed = false;
            }
          }
        }
      } else {
        if( switchState == steerConfig.steerswitchActiveLow ){
          machine.steeringEnabled = false;
          AogToMachineEngagedMismatch = true;
        } else if( steerSetpoints.speed > steerConfig.maxAutosteerSpeed ) {
          machine.steeringEnabled = false;
          safety.autosteerDisabledByMaxEngageSpeed = true;
        } else {
          machine.steeringEnabled = true;
          AogToMachineEngagedMismatch = true;
          machine.disengagedBySteeringWheel = false;
          safety.AOGEnableAutosteerTimeout = false;
          safety.autosteerDisabledByMaxEngageSpeed = false;
        }
      }
    }
    if( millis() - lastSwitchChangeMillis > 1000 && steerSetpoints.enabled == false ){
      if( machine.disengagedBySteeringWheel == false && machine.steeringEnabled == true ){ // only show if user did not disengage
        safety.AOGEnableAutosteerTimeout = true; // AOG did not return enabled command, show in ESP_UI
      }
      machine.steeringEnabled = false; // disable autosteer due to timeout
    }

    bool disengageState = digitalRead( steerConfig.gpioDisengage );
    if( steerConfig.disengageSwitchType == SteerConfig::DisengageSwitchType::Hydraulic ){
      if( disengagePrevState != disengageState && millis() - disengageActivityMillis > 50 ){
        disengagePrevState = disengageState;
        machine.lastDisengageMillis = millis();
      }
      if( disengageState != steerConfig.hydraulicSwitchActiveLow ){
        machine.disengageInput = true;
        machine.steeringEnabled = false;
        machine.disengagedBySteeringWheel = true;
        AogToMachineEngagedMismatch = true;
      } else {
        machine.disengageInput = false;
      }
    }
    else if( steerConfig.disengageSwitchType == SteerConfig::DisengageSwitchType::Encoder ) {
      if( disengagePrevState != disengageState ){
        disengagePrevState = disengageState;
        if( machine.handwheelPulseCount == 0 ){
          disengageActivityMillis = millis();
        }
        machine.handwheelPulseCount += 1;
        machine.lastDisengageMillis = millis();
      }
      if( millis() - disengageActivityMillis < steerConfig.disengageFrameMillis ) {
        if( ( machine.handwheelPulseCount / 2 ) >= steerConfig.disengageFramePulses ) { // divide by two to compensate for LOW and HIGH
          machine.steeringEnabled = false;
          machine.handwheelPulseCount = 0;
          machine.disengagedBySteeringWheel = true;
          machine.disengageInput = true;
          AogToMachineEngagedMismatch = true;
        } else machine.disengageInput = false;
      } else machine.handwheelPulseCount = 0;
    }
    else if( steerConfig.disengageSwitchType == SteerConfig::DisengageSwitchType::JDVariableDuty ){
      uint16_t dutyCycle = abs( onTime - offTime );
      dutyAverage = ( dutyAverage * 0.9 ) + ( dutyCycle * 0.1 );
      if( abs( dutyAverage - dutyCycle ) > steerConfig.JDVariableDutyChange ){
        machine.steeringEnabled = false;
        machine.disengagedBySteeringWheel = true;
        machine.lastDisengageMillis = millis();
        machine.disengageInput = true;
        AogToMachineEngagedMismatch = true;
      } else {
        machine.disengageInput = false;
      }
      machine.DeereDutyCycle = dutyCycle;
      machine.DeereDutyAverage = dutyAverage;
    }
    else if( steerConfig.disengageSwitchType == SteerConfig::DisengageSwitchType::MotorCurrent ){
      machine.steerMotorCurrent = max( analogRead( 34 ), analogRead( 35 ) );
      if( machine.steerMotorCurrent > steerConfig.maxSteerCurrent ){
        if( millis() - disengageActivityMillis > 50 ){
          machine.steeringEnabled = false;
          machine.disengagedBySteeringWheel = true;
          AogToMachineEngagedMismatch = true;
        }
        machine.lastDisengageMillis = millis();
      } else {
        disengageActivityMillis = millis();
      }
    }
    if( steerSetpoints.enabled == true && machine.disengageInput == true ) {
      machine.autosteerSafetyLock = true;
    }
    vTaskDelayUntil( &xLastWakeTime, xFrequency );
  }
}

static void IRAM_ATTR disengageIsr( void* arg ) {
  // interrupt service routine for the steering wheel
  static bool state = LOW;
  if( state == LOW ){
    onTime = esp_timer_get_time() - disengageActivityMicros;
  } else {
    offTime = esp_timer_get_time() - disengageActivityMicros;
  }
  state = !state;
  disengageActivityMicros = esp_timer_get_time();
}

void initAutosteer() {

  udpSendFrom.listen( initialisation.portSendFrom );

  if( udpLocalPort.listen( initialisation.portListenTo ) ) {
    udpLocalPort.onPacket( []( AsyncUDPPacket packet ) {
      uint8_t* data = packet.data();
      if( data[1] + ( data[0] << 8 ) != 0x8081 ){
          return;
      }
      uint16_t pgn = data[3] + ( data[2] << 8 );
      // see pgn.xlsx in https://github.com/farmerbriantee/AgOpenGPS/tree/master/AgOpenGPS_Dev
      switch( pgn ) {
        case 0x7FFE: {
          steerSetpoints.speed = ( float )( (data[5] | data[6] << 8))*0.1 ;
          if( ( SteerConfig::SpeedUnits )steerConfig.speedUnits == SteerConfig::SpeedUnits::MilesPerHour ) {
            steerSetpoints.speed *= 0.62;
          }
          steerSetpoints.enabled = data[7];
          if( steerSetpoints.enabled == true && previousAogEnabledState == false  ) {
            if( steerSetpoints.speed > steerConfig.maxAutosteerSpeed ) {
              machine.autosteerSafetyLock = true;
            }
          }
          if( steerSetpoints.enabled == true ) {
            if( machine.disengageInput == true ) {
              machine.autosteerSafetyLock = true;
            }
          } else machine.autosteerSafetyLock = false;
          if( previousAogEnabledState != steerSetpoints.enabled ){
            previousAogEnabledState = steerSetpoints.enabled;
            steerSetpoints.lastEngagedChangeMillis = millis();
          }
          if ( machine.steeringEnabled == steerSetpoints.enabled ) {
            // clear mismatch flag when machine control and AOG agree again
            AogToMachineEngagedMismatch = false;
          }
          else if( AogToMachineEngagedMismatch == false ) {
            if( machine.autosteerSafetyLock == true ) {
              machine.steeringEnabled = false;
            } else {
              // user pressed AOG autosteer button in software, we need to ACK so AOG listens to disengage
              machine.steeringEnabled = steerSetpoints.enabled; // reflect AOG state, will ACKNOWLEDGE when returned to AOG
            }
            if( steerSetpoints.enabled == true ) {
              machine.disengagedBySteeringWheel = false;
              safety.AOGEnableAutosteerTimeout = false;
              if( machine.autosteerSafetyLock == false ) {
                safety.autosteerDisabledByMaxEngageSpeed = false;
              }
            } else {
              machine.autosteerSafetyLock = false;
            }
          }
          steerSetpoints.requestedSteerAngle = (( double ) ((( int16_t )data[8]) | (( int8_t )data[9] << 8 ))) * 0.01; //horrible code to make negative doubles work

          steerSetpoints.lastPacketReceived = millis();
        }
        break;

        case 0x7FFC: {
          steerSettings.Kp = ( float )data[2] * 1.0; // read Kp from AgOpenGPS
          steerSettings.Ki = ( float )data[3] * 0.001; // read Ki from AgOpenGPS
          steerSettings.Kd = ( float )data[4] * 1.0; // read Kd from AgOpenGPS
          steerSettings.Ko = ( float )data[5] * 0.1; // read Ko from AgOpenGPS
          steerSettings.wheelAnglePositionZero = ( int8_t )data[6]; //read steering zero offset
          steerSettings.minPWMValue = data[7]; //read the minimum amount of PWM for instant on
          steerSettings.maxIntegralValue = data[8] * 0.1; //
          steerSettings.wheelAngleCountsPerDegree = data[9]; //sent as 10 times the setting displayed in AOG

          steerSettings.lastPacketReceived = millis();
        }
        break;

        case 0x7FF6: {
          steerMachineControl.pedalControl = data[2];
          steerMachineControl.speed = ( float )data[3] / 4;
          steerMachineControl.relais = data[4];
          steerMachineControl.youTurn = data[5];

          steerMachineControl.lastPacketReceived = millis();
        }
        break;

        default:
          break;
      }
    } );
  }

  // init output
  {
    pinMode( steerConfig.gpioPwm, OUTPUT );
    ledcSetup( 0, steerConfig.pwmFrequency, 8 );
    ledcAttachPin( steerConfig.gpioPwm, 0 );
    ledcWrite( 0, 0 );

    pinMode( steerConfig.gpioRight, OUTPUT );
    ledcSetup( 1, steerConfig.pwmFrequency, 8 );
    ledcAttachPin( steerConfig.gpioRight, 1 );
    ledcWrite( 1, 0 );

    pinMode( steerConfig.gpioLeft, OUTPUT );
    ledcSetup( 2, steerConfig.pwmFrequency, 8 );
    ledcAttachPin( steerConfig.gpioLeft, 2 );
    ledcWrite( 2, 0 );

    pinMode( steerConfig.gpioEn, OUTPUT );
    digitalWrite( steerConfig.gpioEn, LOW );

    pinMode( steerConfig.gpioSteerLED, OUTPUT );
    digitalWrite( steerConfig.gpioSteerLED, LOW );

    pinMode( steerConfig.gpioAlarm, OUTPUT );
    digitalWrite( steerConfig.gpioAlarm, LOW );

    pinMode( steerConfig.gpioDisengagePullup, OUTPUT );
    digitalWrite( steerConfig.gpioDisengagePullup, steerConfig.disengageHeavyDuty );

    pinMode( steerConfig.gpioSteerSupplyVoltage, INPUT );

    switch( steerConfig.outputType ) {
      case SteerConfig::OutputType::SteeringMotorIBT2: {
        initialisation.outputType = SteerConfig::OutputType::SteeringMotorIBT2;
      }
      break;

      case SteerConfig::OutputType::SteeringMotorCytron: {
        initialisation.outputType = SteerConfig::OutputType::SteeringMotorCytron;
      }
      break;

      case SteerConfig::OutputType::HydraulicPwm2Coil: {
        initialisation.outputType = SteerConfig::OutputType::HydraulicPwm2Coil;
      }
      break;

      case SteerConfig::OutputType::HydraulicDanfoss: {
        initialisation.outputType = SteerConfig::OutputType::HydraulicDanfoss;
      }
      break;

      case SteerConfig::OutputType::HydraulicBangBang: {
        initialisation.outputType = SteerConfig::OutputType::HydraulicBangBang;
      }
      break;

      case SteerConfig::OutputType::Canbus13_19Controller: {
        initialisation.outputType = SteerConfig::OutputType::Canbus13_19Controller;
      }
      break;

      case SteerConfig::OutputType::CanbusF0_240Controller: {
        initialisation.outputType = SteerConfig::OutputType::CanbusF0_240Controller;
      }
      break;

      default:
        break;

    }
  }

  if( machine.canbusSteeringActive == true ){
    String str;
    str.reserve( 30 );
    str = "N/A in CANbus steering";
    
    ESPUI.updateLabel( labelSteerEngagedFaults, str );
    ESPUI.updateLabel( labelSupplyVoltage, str );
  }

  pinMode( steerConfig.gpioWorkswitch, INPUT_PULLUP );
  pinMode( steerConfig.gpioWorkLED, OUTPUT );
  pinMode( steerConfig.gpioSteerswitch, INPUT_PULLUP );

  #define DISENGAGE_GPIO ( gpio_num_t ) steerConfig.gpioDisengage
  gpio_pad_select_gpio( DISENGAGE_GPIO );
  gpio_set_direction( DISENGAGE_GPIO, GPIO_MODE_INPUT );
  gpio_pulldown_dis( DISENGAGE_GPIO );
  gpio_pullup_dis( DISENGAGE_GPIO );
  gpio_set_intr_type( DISENGAGE_GPIO, GPIO_INTR_ANYEDGE );
  gpio_install_isr_service( ESP_INTR_FLAG_IRAM );
  gpio_isr_handler_add(( gpio_num_t ) DISENGAGE_GPIO, disengageIsr, ( void* ) NULL );

  xTaskCreate( autosteerWorker100Hz, "autosteerWorker", 3096, NULL, 3, NULL );
  if( machine.canbusSteeringActive == false ){
    xTaskCreate( autosteerSwitchesWorker1000Hz, "autosteerSwitchesWorker", 3096, NULL, 3, NULL );
  }

  if( steerConfig.outputType == SteerConfig::OutputType::HydraulicPwm2Coil ) {
    xTaskCreate( ditherWorker10HZ, "ditherWorker10HZ", 1024, NULL, 1, NULL );
  }
}
