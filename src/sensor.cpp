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

#include <Adafruit_Sensor.h>

#include <Adafruit_ADS1015.h>

#include <ESPUI.h>

#include "main.hpp"
#include "jsonFunctions.hpp"

Adafruit_ADS1115 ads = Adafruit_ADS1115( 0x48 );

bool WAS_Error = true; // true until WAS angle calculated on startup
bool WAS_SupplyError = false;

volatile time_t DeereWasOnTime;
volatile time_t DeereWasOffTime;

float rowSenseIntegrator; // integrator for Row Sense steering
uint8_t rowSenseActive = 0; // 0:startup, 1:active, 2:disconnected or out of range

// http://www.schwietering.com/jayduino/filtuino/index.php?characteristic=bu&passmode=lp&order=2&usesr=usesr&sr=100&frequencyLow=5&noteLow=&noteHigh=&pw=pw&calctype=float&run=Send
//Low pass butterworth filter order=2 alpha1=0.05
class  FilterBuLp2_3 {
  public:
    FilterBuLp2_3() {
      v[0] = 0.0;
      v[1] = 0.0;
    }
  private:
    float v[3];
  public:
    float step( float x ) { //class II
      v[0] = v[1];
      v[1] = v[2];
      v[2] = ( 2.008336556421122521e-2 * x )
             + ( -0.64135153805756306422 * v[0] )
             + ( 1.56101807580071816339 * v[1] );
      return
              ( v[0] + v[2] )
              + 2 * v[1];
    }
};
FilterBuLp2_3 wheelAngleSensorFilter, rowSenseFilter;

void sensorWorker100HzPoller( void* z ) {
  vTaskDelay( 2000 );
  constexpr TickType_t xFrequency = 10;
  TickType_t xLastWakeTime = xTaskGetTickCount();

  for( ;; ) {

    float wheelAngleTmp = 31250;
    machine.steerSupplyVoltage = map( analogRead( 34 ), 0, 4095, 186, 3909 );
    
    switch( steerConfig.wheelAngleInput ) {
      case SteerConfig::AnalogIn::JDVariableDuty: {
        wheelAngleTmp = DeereWasOnTime - DeereWasOffTime;
      }
      break;

      case SteerConfig::AnalogIn::ADS1115A0Single:
      case SteerConfig::AnalogIn::ADS1115A1Single: {
        if( xSemaphoreTake( i2cMutex, 1000 ) == pdTRUE ) {
          wheelAngleTmp = ads.readADC_SingleEnded(
                                  ( uint8_t )steerConfig.wheelAngleInput - ( uint8_t )SteerConfig::AnalogIn::ADS1115A0Single );
          xSemaphoreGive( i2cMutex );
        }
      }
      break;

      case SteerConfig::AnalogIn::ADS1115A0A1Differential: {
        if( xSemaphoreTake( i2cMutex, 1000 ) == pdTRUE ) {
          wheelAngleTmp = ads.readADC_Differential_0_1();
          xSemaphoreGive( i2cMutex );
        }
      }
      break;
      
      case SteerConfig::AnalogIn::CanbusFendt: {
        wheelAngleTmp = ( int16_t )machine.FendtWasCounts;
      }
      break;

      case SteerConfig::AnalogIn::CanbusValtraMasseyChallenger: {
        wheelAngleTmp = machine.canbusWasCounts;
      }
      break;

      default:
        break;
    }

    {
      steerSetpoints.wheelAngleCounts = wheelAngleTmp;
      if( steerConfig.wheelAngleInput != SteerConfig::AnalogIn::CanbusFendt ) {
        wheelAngleTmp -= steerConfig.wheelAnglePositionZero; // Fendt is zeroed by default, only zero non-Fendt sensors
      }
      wheelAngleTmp /= steerConfig.wheelAngleCountsPerDegree;

      steerSetpoints.wheelAngleRaw = wheelAngleTmp;

      if( steerConfig.wheelAngleSensorType == SteerConfig::WheelAngleSensorType::TieRodDisplacement ) {
        if( steerConfig.wheelAngleFirstArmLenght != 0 && steerConfig.wheelAngleSecondArmLenght != 0 &&
            steerConfig.wheelAngleTrackArmLenght != 0 && steerConfig.wheelAngleTieRodStroke != 0 ) {

          auto getDisplacementFromAngle = []( float angle ) {
            // a: 2. arm, b: 1. arm, c: abstand drehpunkt wineklsensor und anschlagpunt 2. arm an der spurstange
            // gegenwinkel: winkel zwischen 1. arm und spurstange
            double alpha = PI - radians( angle );

            // winkel zwischen spurstange und 2. arm
            double gamma = PI - alpha - ( asin( steerConfig.wheelAngleFirstArmLenght * sin( alpha ) / steerConfig.wheelAngleSecondArmLenght ) );

            // auslenkung
            return steerConfig.wheelAngleSecondArmLenght * sin( gamma ) / sin( alpha );
          };

          steerSetpoints.wheelAngleCurrentDisplacement = getDisplacementFromAngle( wheelAngleTmp );

          double relativeDisplacementToStraightAhead =
                  // real displacement
                  steerSetpoints.wheelAngleCurrentDisplacement -
                  // calculate middle of displacement -
                  ( getDisplacementFromAngle( steerConfig.wheelAngleMinimumAngle ) + ( steerConfig.wheelAngleTieRodStroke / 2 ) );

          wheelAngleTmp = degrees( asin( relativeDisplacementToStraightAhead / steerConfig.wheelAngleTrackArmLenght ) );
        }
      }

      if( steerConfig.invertWheelAngleSensor ) {
        wheelAngleTmp *= ( float ) -1;
      }

      wheelAngleTmp -= steerConfig.wheelAngleOffset;

      if (( wheelAngleTmp > 0 && steerConfig.ackermannAboveZero == true ) || 
          ( wheelAngleTmp < 0 && steerConfig.ackermannAboveZero == false )) {
        wheelAngleTmp = ( wheelAngleTmp * steerConfig.ackermann ) / 100;
      }

      if( digitalRead( steerConfig.gpioWasSupplyDetectionPin ) == LOW ){
        if( WAS_SupplyError == false ){
          WAS_SupplyError = true;
          diagnostics.WasPositiveSupplyShortedToGround += 1;
          saveDiagnostics();
        }
      } else {
        WAS_SupplyError = false;
      }
      if( abs( steerSetpoints.actualSteerAngle - wheelAngleTmp ) > 5.00 ){ // a jump of more than 5.00 degrees is a bad sensor
        if( WAS_Error == false ){
          WAS_Error = true;
          showHardwareStateOnAOG( 0x75 );
          diagnostics.WasPlausibilityErrors += 1;
          saveDiagnostics();
        }
      } else {
        WAS_Error = false;
      }
      wheelAngleTmp = wheelAngleSensorFilter.step( wheelAngleTmp );
      steerSetpoints.actualSteerAngle = wheelAngleTmp;

      if( steerConfig.enableRowSense ){
        machine.rowSenseVoltageOne = ads.readADC_SingleEnded_V( 2 ) * 1.59;
        machine.rowSenseVoltageTwo = ads.readADC_SingleEnded_V( 3 ) * 1.59;
        if( max( machine.rowSenseVoltageOne, machine.rowSenseVoltageTwo ) < 4.7 &&
            min( machine.rowSenseVoltageOne, machine.rowSenseVoltageTwo ) > 0.3 ){
          double rowSenseTmp = ads.readADC_SingleEnded( 2 );
          rowSenseTmp += ads.readADC_SingleEnded( 3 );
          rowSenseTmp /= 2;
          steerSetpoints.rowSenseCounts = rowSenseTmp;
          rowSenseTmp -= steerConfig.rowSensePositionZero;
          rowSenseTmp /= steerConfig.rowSenseCountsPerDegree;
          if( steerConfig.invertRowSense ){
            rowSenseTmp *= ( float ) -1;
          }
          rowSenseTmp  = rowSenseFilter.step( rowSenseTmp );
          rowSenseIntegrator += rowSenseTmp * steerConfig.rowSenseKi;
          rowSenseIntegrator = constrain( rowSenseIntegrator, -steerConfig.rowSenseKiMaxDegrees, steerConfig.rowSenseKiMaxDegrees ); // only limit Ki angle
          float angle = ( rowSenseTmp * steerConfig.rowSenseKp ) + rowSenseIntegrator; // Kp does not get limited
          if( angle < steerConfig.rowSenseMinDegrees && angle > -steerConfig.rowSenseMinDegrees ){
            steerSetpoints.rowSenseAngle = 0.00;
          } else {
            if( angle < 0.00 ){
              steerSetpoints.rowSenseAngle = angle - ( -steerConfig.rowSenseMinDegrees );
            } else {
              steerSetpoints.rowSenseAngle = angle - steerConfig.rowSenseMinDegrees;
            }
          }
          steerSetpoints.rowSenseControl = true; // requested steer angle now comes from row sense instead of software
        } else {
          steerSetpoints.rowSenseCounts = 0;
          steerSetpoints.rowSenseAngle = 0.00;
          steerSetpoints.rowSenseControl = false;
        }
      } else {
        steerSetpoints.rowSenseControl = false;
      }
    }
    

    vTaskDelayUntil( &xLastWakeTime, xFrequency );
  }
}

void IRAM_ATTR DeereVariableDutyWasIsr() {
    // interrupt service routine for the steering wheel
    static time_t sensorActivityMicros = micros();
    static bool previousState;
    bool state = digitalRead( ( uint8_t ) steerConfig.gpioWASPulse );
    if( previousState != state ){
      previousState = state;
      if( state == LOW ){
        DeereWasOnTime = micros() - sensorActivityMicros;
      } else {
        DeereWasOffTime = micros() - sensorActivityMicros;
      }
      sensorActivityMicros = micros();
    }
}

void initSensors() {

  initialisation.wheelAngleInput = steerConfig.wheelAngleInput;

  pinMode( steerConfig.gpioWasSupplyDetectionPin, INPUT );

  switch( steerConfig.wheelAngleInput ) {
    case SteerConfig::AnalogIn::JDVariableDuty: {
      pinMode( steerConfig.gpioWASPulse, INPUT );
      attachInterrupt( steerConfig.gpioWASPulse, DeereVariableDutyWasIsr, CHANGE );
    }
    break;

    case SteerConfig::AnalogIn::CanbusFendt:
    case SteerConfig::AnalogIn::CanbusValtraMasseyChallenger:
      break;

    default: {
      ads.setGain( ( adsGain_t )steerConfig.adsGain );
      ads.begin();
      ads.setSPS( ADS1115_DR_860SPS );
    }
    break;
  }

  xTaskCreate( sensorWorker100HzPoller, "sensorWorker100HzPoller", 4096, NULL, 6, NULL );
}
