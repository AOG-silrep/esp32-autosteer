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

#include <ESPUI.h>
#include <CAN_config.h>

#include "main.hpp"
#include "jsonFunctions.hpp"

CAN_device_t CAN_cfg;

constexpr uint8_t rxQueueSize = 10;

constexpr uint16_t j1939PgnEEC1 = 61444;
constexpr uint16_t j1939PgnWBSD = 65096;

constexpr uint16_t j1939PgnPHS = 65093;
constexpr uint16_t j1939PgnFHS = 65094;

constexpr uint16_t j1939PgnRPTO = 65091;
constexpr uint16_t j1939PgnFPTO = 65092;

constexpr uint16_t j1939PgnVMCWas = 44032;
constexpr uint16_t j1939PgnVMCAutosteer = 61184;

bool readyToDisengage;

// see https://gurtam.com/files/ftp/CAN/ (especialy J1939.zip)

void canReceiver10Hz( void* z ) {
  constexpr TickType_t xFrequency = 100;
  TickType_t xLastWakeTime = xTaskGetTickCount();

  CAN_frame_t canFrame;
  time_t lastCanbusMsgMillis;

  for( ;; ) {
    if( xQueueReceive( CAN_cfg.rx_queue, &canFrame, xFrequency ) == pdTRUE ) {
      lastCanbusMsgMillis = millis();
      if( canFrame.FIR.B.FF == CAN_frame_ext ) {

        uint16_t pgn = ( canFrame.MsgID >> 8 ) & 0x03FFFF;
        uint16_t pduFormat = ( pgn >> 8 ) & 0xFF;  // PDU format is bits 6-13
        if( pduFormat < 240 ){   // if PDU format is less than 240, we subtract the PDU specific address
          uint8_t pduSpecific = pgn & 0xFF;
          pgn = pgn - pduSpecific;
        }

        switch( pgn ) {

          // Electronic Engine Controller 1
          case j1939PgnEEC1: {
            steerCanData.motorRpm = ( canFrame.data.u8[4] << 8 | canFrame.data.u8[3] ) / 8;
          }
          break;

             // Wheel-based Speed and Distance
          case j1939PgnWBSD: {
            steerCanData.speed = ( canFrame.data.u8[1] << 8 | canFrame.data.u8[0] ) / 1000 * 3.6;
          }
          break;

          // Primary or Rear Hitch Status
          case j1939PgnPHS: {
            steerCanData.rearHitchPosition = canFrame.data.u8[0];
          }
          break;

          // Secondary or Front Hitch Status
          case j1939PgnFHS: {
            steerCanData.frontHitchPosition = canFrame.data.u8[0];
          }
          break;

          // Primary or Rear Power Take off Output Shaft
          case j1939PgnRPTO: {
            steerCanData.rearPtoRpm = ( canFrame.data.u8[1] << 8 | canFrame.data.u8[0] ) / 8;
          }
          break;

          // Secondary or Front Power Take off Output Shaft
          case j1939PgnFPTO: {
            steerCanData.frontPtoRpm = ( canFrame.data.u8[1] << 8 | canFrame.data.u8[0] ) / 8;
          }
          break;

          case j1939PgnVMCWas: { //0x0CAC1C13
            if( steerConfig.wheelAngleInput == SteerConfig::AnalogIn::CanbusValtraMasseyChallenger ){      
              machine.canbusWasCounts = (( canFrame.data.u8[1] << 8 ) + canFrame.data.u8[0] );  // CAN Buf[1]*256 + CAN Buf[0] = CAN Est Curve
              machine.lastCanbusWasMillis = millis();
            }
            machine.canbusSteeringState = ( canFrame.data.u8[2] );
            machine.lastCanbusSteeringMillis = millis();
            if( digitalRead( ( uint8_t )steerConfig.gpioSteerswitch ) == steerConfig.steerswitchActiveLow ){
              if( machine.canbusSteeringState == 0x10 ){ //only try to engage when machine is ready, to avoid race conditions
                machine.steeringEnabled = true;
              }
            }
            if( machine.canbusSteeringState == 0x14 ){
              readyToDisengage = true; // we need the steer enabled confirmation from the machine before disengaging again
            } else if( readyToDisengage == true ) {
              machine.steeringEnabled = false;
              readyToDisengage = false;
            }
          }
          break;

          case j1939PgnVMCAutosteer:{ //0x18EF1C00
            if(( canFrame.data.u8[0] ) == 15 && ( canFrame.data.u8[1] ) == 96 && ( canFrame.data.u8[2] ) == 1 ){
              if( machine.canbusSteeringState == 0x10 ){ //only try to engage when machine is ready, to avoid race conditions
                machine.steeringEnabled = true;
              }
            } else if(( canFrame.data.u8[0] ) == 15 && ( canFrame.data.u8[1] ) == 96 && ( canFrame.data.u8[2] ) == 0 ){
              machine.steeringEnabled = false;
            }
          }
          break;
        }
      }
    } else { // no Canbus info, let CPU do other stuff
        vTaskDelayUntil( &xLastWakeTime, xFrequency );
    }

    {
      static time_t loopTimeToWaitTo = 0;

      if( loopTimeToWaitTo < millis() ) {

        String str;
        str.reserve( 200 );

        str = "<table style='margin:auto;'><tr><td style='text-align:left; padding: 0px 5px;'>Wheel-based Speed:</td><td style='text-align:left; padding: 0px 5px;'>";
        str += String( steerCanData.speed );
        str += "</td></tr><tr><td style='text-align:left; padding: 0px 5px;'>Motor RPM:</td><td style='text-align:left; padding: 0px 5px;'>";
        str += String( steerCanData.motorRpm );
        str += "</td></tr><tr><td style='text-align:left; padding: 0px 5px;'>Front Hitch Position:</td><td style='text-align:left; padding: 0px 5px;'>";
        str += String( steerCanData.frontHitchPosition );
        str += "</td></tr><tr><td style='text-align:left; padding: 0px 5px;'>Rear Hitch Position:</td><td style='text-align:left; padding: 0px 5px;'>";
        str += String( steerCanData.rearHitchPosition );
        str += "</td></tr><tr><td style='text-align:left; padding: 0px 5px;'>Front PTO RPM:</td><td style='text-align:left; padding: 0px 5px;'>";
        str += String( steerCanData.frontPtoRpm );
        str += "</td></tr><tr><td style='text-align:left; padding: 0px 5px;'>Rear PTO RPM:</td><td style='text-align:left; padding: 0px 5px;'>";
        str += String( steerCanData.rearPtoRpm );
        str += "</td></tr></table>";
        str += "Received ";
        time_t elapse = millis() - lastCanbusMsgMillis;
        if( elapse < 1000 ){
          str += String( elapse );
          str += " millis";
        } else {
          str += String( elapse / 1000 );
          str += " seconds";
        }
        str += " ago";

        ESPUI.updateLabel( labelStatusCan, str );

        loopTimeToWaitTo = millis() + 1000;
      }
    }
  }
}

void canSender10Hz( void* z ) {
  constexpr TickType_t xFrequency = 100;
  TickType_t xLastWakeTime = xTaskGetTickCount();

  for( ;; ) {
    if( millis() - machine.lastCanbusSteeringMillis > 500 ){
      machine.steeringEnabled = false;
    }
    CAN_frame_t canFrame;
    canFrame.MsgID = 0x0CAD131C;
    canFrame.FIR.B.FF = CAN_frame_ext;
    canFrame.FIR.B.DLC = 8;
    canFrame.data.u8[0] = ( uint8_t ) machine.valveOutput;
    canFrame.data.u8[1] = ( uint8_t ) ( machine.valveOutput >> 8 );
    if( steerSetpoints.enabled == true && steerSetpoints.speed > steerConfig.minAutosteerSpeed ){
      canFrame.data.u8[2] = 253;
    } else {
      canFrame.data.u8[2] = 252;
    }
    canFrame.data.u8[3] = 0;
    canFrame.data.u8[4] = 0;
    canFrame.data.u8[5] = 0;
    canFrame.data.u8[6] = 0;
    canFrame.data.u8[7] = 0;
    ESP32Can.CANWriteFrame( &canFrame );

    vTaskDelayUntil( &xLastWakeTime, xFrequency );
  }
}

void initCan() {
  if( steerConfig.canBusEnabled ) {
    CAN_cfg.speed = ( CAN_speed_t )steerConfig.canBusSpeed;
    CAN_cfg.tx_pin_id = ( gpio_num_t )steerConfig.canBusTx;
    CAN_cfg.rx_pin_id = ( gpio_num_t )steerConfig.canBusRx;
    CAN_cfg.rx_queue = xQueueCreate( rxQueueSize, sizeof( CAN_frame_t ) );
    // Init CAN Module
    ESP32Can.CANInit();
    CAN_frame_t msgISO;
    bool claimAddress = false;
    if( steerConfig.wheelAngleInput == SteerConfig::AnalogIn::CanbusValtraMasseyChallenger ){
      msgISO.MsgID = 0x18EEFF1C;
      claimAddress = true;
    }
    if( claimAddress ){
      msgISO.FIR.B.FF = CAN_frame_ext;
      msgISO.FIR.B.DLC = 8;
      msgISO.data.u8[0] = 0x00;
      msgISO.data.u8[1] = 0x00;
      msgISO.data.u8[2] = 0xC0;
      msgISO.data.u8[3] = 0x0C;
      msgISO.data.u8[4] = 0x00;
      msgISO.data.u8[5] = 0x17;
      msgISO.data.u8[6] = 0x02;
      msgISO.data.u8[7] = 0x20;
      ESP32Can.CANWriteFrame( &msgISO );
    }
    xTaskCreate( canReceiver10Hz, "canReceiver", 2048, NULL, 5, NULL );
    if( steerConfig.outputType >= SteerConfig::OutputType::Canbus13_19Controller ){
      xTaskCreate( canSender10Hz, "canSender", 2048, NULL, 5, NULL );
    }
  }
}