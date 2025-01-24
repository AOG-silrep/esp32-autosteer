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

#include "average.hpp"
#include "ringbuffer.hpp"

Adafruit_ADS1115 ads = Adafruit_ADS1115( 0x48 );

volatile time_t DeereWasOnTime;
volatile time_t DeereWasOffTime;

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
} wheelAngleSensorFilter;

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

      case SteerConfig::AnalogIn::CanbusValtraMasseyChallenger: {
        wheelAngleTmp = machine.canbusWasCounts;
      }
      break;

      default:
        break;
    }

    {
      steerSetpoints.wheelAngleCounts = wheelAngleTmp;
      wheelAngleTmp -= steerConfig.wheelAnglePositionZero;
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

      wheelAngleTmp = wheelAngleSensorFilter.step( wheelAngleTmp );
      steerSetpoints.actualSteerAngle = wheelAngleTmp;

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

  Control* handle = ESPUI.getControl( labelStatusAdc );
  String str;
  str.reserve( 30 );

  if( steerConfig.wheelAngleInput == SteerConfig::AnalogIn::CanbusValtraMasseyChallenger ){

    str = "Canbus: Valtra-Massey-Challenger WAS initialized";
    handle->color = ControlColor::Emerald;
    ESPUI.updateLabel( labelStatusAdc, str );
  }
  else if( steerConfig.wheelAngleInput == SteerConfig::AnalogIn::JDVariableDuty ){
    pinMode( steerConfig.gpioWASPulse, INPUT );
    attachInterrupt( steerConfig.gpioWASPulse, DeereVariableDutyWasIsr, CHANGE );

    str = "Deere variable duty WAS initialized";
    handle->color = ControlColor::Emerald;
    ESPUI.updateLabel( labelStatusAdc, str );

  } else {
    
    ads.setGain( (adsGain_t)steerConfig.adsGain );
    // ads.setGain(GAIN_TWOTHIRDS);   // 2/3x gain +/- 6.144V  1 bit = 3mV      0.1875mV (default)  
    // ads.setGain(GAIN_ONE);        // 1x gain   +/- 4.096V  1 bit = 2mV      0.125mV
    // ads.setGain(GAIN_TWO);        // 2x gain   +/- 2.048V  1 bit = 1mV      0.0625mV
    // ads.setGain(GAIN_FOUR);       // 4x gain   +/- 1.024V  1 bit = 0.5mV    0.03125mV
    // ads.setGain(GAIN_EIGHT);      // 8x gain   +/- 0.512V  1 bit = 0.25mV   0.015625mV
    // ads.setGain(GAIN_SIXTEEN);    // 16x gain  +/- 0.256V  1 bit = 0.125mV  0.0078125mV

    ads.begin();
    ads.setSPS( ADS1115_DR_860SPS );

    str = "ADS1115 initialized\n";

    handle->color = ControlColor::Emerald;
    if( steerConfig.wheelAngleInput == SteerConfig::AnalogIn::None ){
      str += "no input selected\n";
      handle->color = ControlColor::Alizarin;
    }
    else if( steerConfig.wheelAngleInput == SteerConfig::AnalogIn::ADS1115A0Single ){
      str += "A0 single input selected\n";
    }
    else if( steerConfig.wheelAngleInput == SteerConfig::AnalogIn::ADS1115A1Single ){
      str += "A1 single input selected\n";
    }
    else if( steerConfig.wheelAngleInput == SteerConfig::AnalogIn::ADS1115A0A1Differential ){
      str += "A0/A1 differential input selected\n";
    }

    if( steerConfig.adsGain == SteerConfig::ADSGain::GAIN_TWOTHIRDS ){
      str += " max voltage: 6.144\n5v WAS directly connected";
    }
    else if( steerConfig.adsGain == SteerConfig::ADSGain::GAIN_ONE ){
      str += " max voltage: 4.096\n5v WAS-3.3k-ADS-5.6k-Gnd divider";
    }
    else if( steerConfig.adsGain == SteerConfig::ADSGain::GAIN_TWO ){
      str += " max voltage: 2.048\nWAS not defined";
    }
    else if( steerConfig.adsGain == SteerConfig::ADSGain::GAIN_FOUR ){
      str += " max voltage: 1.024\nWAS not defined";
    }
    else if( steerConfig.adsGain == SteerConfig::ADSGain::GAIN_EIGHT ){
      str += " max voltage: 0.512\nWAS not defined";
    }
    else if( steerConfig.adsGain == SteerConfig::ADSGain::GAIN_SIXTEEN ){
      str += " max voltage: 0.256\nWAS not defined";
    } else {
      str += "voltage range not defined";
      handle->color = ControlColor::Alizarin;
    }
    ESPUI.updateLabel( labelStatusAdc, str );
  }
  initialisation.wheelAngleInput = steerConfig.wheelAngleInput;

  xTaskCreate( sensorWorker100HzPoller, "sensorWorker100HzPoller", 4096, NULL, 6, NULL );
}
