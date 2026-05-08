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
#include <SPI.h>
#include <mcp_can.h> // MCP2515
SPIClass *hspi;

#include "main.hpp"
#include "jsonFunctions.hpp"

CAN_device_t CAN_cfg;
TaskHandle_t canReceiverHandle = NULL;
TaskHandle_t canSenderHandle = NULL;

constexpr uint8_t rxQueueSize = 10;

constexpr uint16_t j1939PgnEEC1 = 61444;
constexpr uint16_t j1939PgnWBSD = 65096;

constexpr uint16_t j1939PgnPHS = 65093;
constexpr uint16_t j1939PgnFHS = 65094;

constexpr uint16_t j1939PgnRPTO = 65091;
constexpr uint16_t j1939PgnFPTO = 65092;

constexpr uint16_t j1939PgnAH1 = 65072;
constexpr uint16_t j1939PgnAH2 = 65073;
constexpr uint16_t j1939PgnAH3 = 65074;

constexpr uint16_t j1939PgnVMCWas = 44032;
constexpr uint32_t VMCAutosteer = 0x18EF1C00;
constexpr uint32_t FendtAutosteer = 0x0CEF2CF0;

constexpr uint32_t DeereHydraulicRemotes = 0x18FFFB22;

bool readyToDisengage;

void canFendtSteeringReceiver100Hz( void* z ) {
  constexpr TickType_t xFrequency = 10;
  TickType_t xLastWakeTime = xTaskGetTickCount();

  CAN_frame_t canFrame;
  time_t lastCanbusMsgMillis;

  for( ;; ) {
    if( xQueueReceive( CAN_cfg.rx_queue, &canFrame, xFrequency ) == pdTRUE ) {
      lastCanbusMsgMillis = millis();
      if( canFrame.FIR.B.FF == CAN_frame_ext ) {
        if( canFrame.MsgID == FendtAutosteer ){ //0x0CEF2CF0
          if( canFrame.data.u8[0] == 0x05 && canFrame.data.u8[1] == 0x1A && canFrame.data.u8[2] == 0x00 ){
            machine.steeringEnabled = false;
          } else if( canFrame.data.u8[0] == 0x05 && canFrame.data.u8[1] == 0x0A ){
            machine.canbusWasCounts = ((( int8_t ) canFrame.data.u8[4] << 8 ) + canFrame.data.u8[5] );
            machine.lastCanbusWasMillis = millis();
          }
        }
      }
    } else { // no Canbus info, let CPU do other stuff
      vTaskDelayUntil( &xLastWakeTime, xFrequency );
    }
    static time_t loopTimeToWaitTo = 0;

    if( loopTimeToWaitTo < millis() ) {

      String str;
      str.reserve( 200 );

      str = "Received ";
      time_t elapse = millis() - lastCanbusMsgMillis;
      if( elapse < 1000 ){
        str += String( elapse );
        str += " millis";
      } else {
        str += String( elapse / 1000 );
        str += " seconds";
      }
      str += " ago";

      ESPUI.updateLabel( labelStatusCanESP32, str );

      loopTimeToWaitTo = millis() + 1000;
    }
  }
}


// see https://gurtam.com/files/ftp/CAN/ (especialy J1939.zip)
void canReceiver10Hz( void* z ) {
  constexpr TickType_t xFrequency = 10;
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

          // Auxiliary Hydraulic 1
          case j1939PgnAH1: {
            steerCanData.hydraulicRemote1 = ( canFrame.data.u8[1] << 8 | canFrame.data.u8[0] ) / 8;
          }
          break;

          // Auxiliary Hydraulic 2
          case j1939PgnAH2: {
            steerCanData.hydraulicRemote2 = ( canFrame.data.u8[1] << 8 | canFrame.data.u8[0] ) / 8;
          }
          break;

          // Auxiliary Hydraulic 3
          case j1939PgnAH3: {
            steerCanData.hydraulicRemote3 = ( canFrame.data.u8[1] << 8 | canFrame.data.u8[0] ) / 8;
          }
          break;

          case j1939PgnVMCWas: { //0x0CAC1C13
            if( steerConfig.wheelAngleInput == SteerConfig::AnalogIn::CanbusValtraMasseyChallenger ){      
              machine.canbusWasCounts = (( canFrame.data.u8[1] << 8 ) + canFrame.data.u8[0] );  // CAN Buf[1]*256 + CAN Buf[0] = CAN Est Curve
              machine.lastCanbusWasMillis = millis();
            }
            machine.canbusSteeringState = ( canFrame.data.u8[2] );
            machine.lastCanbusSteeringMillis = millis();
            if( machine.canbusSteeringState == 0x14 ){ // we need the steer enabled confirmation from the machine before disengaging again
              readyToDisengage = true;
              machine.disengageInput = false;
              machine.disengagedBySteeringWheel = false;
            } else if( machine.canbusSteeringState == 0x00 ) { // handwheel activity disengages
              if( steerSetpoints.enabled == true ) {
                machine.autosteerSafetyLock = true;
              }
              machine.steeringEnabled = false;
              machine.disengagedBySteeringWheel = true;
              readyToDisengage = false;
              machine.disengageInput = true;
              machine.lastDisengageMillis = millis();
            } else if ( machine.canbusSteeringState == 0x20 ) { // stagnant
              if( readyToDisengage == true ){
                showHardwareStateOnAOG( 0x61 );
              }
              machine.steeringEnabled = false;
              readyToDisengage = false;
              machine.disengageInput = false;
            } else if( readyToDisengage == true ) { // going from 'engaged' to anything else will disengage
              machine.steeringEnabled = false;
              readyToDisengage = false;
              machine.lastDisengageMillis = millis();
            } else {
              machine.disengageInput = false;
            }
          }
          break;

          default:{
            // Can Frame ID specific, since PGNs match but message IDs don't
            switch( canFrame.MsgID ){

              case VMCAutosteer:{ //0x18EF1C00
                if(( canFrame.data.u8[0] ) == 15 && ( canFrame.data.u8[1] ) == 96 && ( canFrame.data.u8[2] ) == 1 ){
                  if( machine.canbusSteeringState == 0x10 ){ //only try to engage when machine is ready, to avoid race conditions
                    if( steerSetpoints.speed > steerConfig.maxAutosteerSpeed ) {
                      machine.steeringEnabled = false;
                      safety.autosteerDisabledByMaxEngageSpeed = true;
                    } else {
                      machine.steeringEnabled = true;
                      AogToMachineEngagedMismatch = true;
                    }
                  } else {
                    showHardwareStateOnAOG( machine.canbusSteeringState );
                  }
                } else if(( canFrame.data.u8[0] ) == 15 && ( canFrame.data.u8[1] ) == 96 && ( canFrame.data.u8[2] ) == 0 ){
                  machine.steeringEnabled = false;
              }

              case DeereHydraulicRemotes:{ //0x18FFFB22
                if(( canFrame.data.u8[0] ) == 0xF2 && ( canFrame.data.u8[1] ) == 0x17 ){
                  steerCanData.hydraulicRemote1 = ( canFrame.data.u8[2] );
                  steerCanData.hydraulicRemote2 = ( canFrame.data.u8[3] );
                  steerCanData.hydraulicRemote3 = ( canFrame.data.u8[4] );
                }
              }
              
              default:
              break;
              }
            }
          }
          break;
        }
      }
    } else { // no Canbus info, let CPU do other stuff
      vTaskDelayUntil( &xLastWakeTime, xFrequency );
    }
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
      str += "</td></tr><tr><td style='text-align:left; padding: 0px 5px;'>Remote 1 Lever Position:</td><td style='text-align:left; padding: 0px 5px;'>";
      str += String( steerCanData.hydraulicRemote1 );
      str += "</td></tr><tr><td style='text-align:left; padding: 0px 5px;'>Remote 2 Lever Position:</td><td style='text-align:left; padding: 0px 5px;'>";
      str += String( steerCanData.hydraulicRemote2 );
      str += "</td></tr><tr><td style='text-align:left; padding: 0px 5px;'>Remote 3 Lever Position:</td><td style='text-align:left; padding: 0px 5px;'>";
      str += String( steerCanData.hydraulicRemote3 );
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

      ESPUI.updateLabel( labelStatusCanESP32, str );

      loopTimeToWaitTo = millis() + 1000;
    }
  }
}

void can13_19Sender100Hz( void* z ) { // Valtra Massey Challenger
  constexpr TickType_t xFrequency = 10;
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

void canF0_240Sender50Hz( void* z ) { // Fendt
  constexpr TickType_t xFrequency = 20;
  TickType_t xLastWakeTime = xTaskGetTickCount();

  for( ;; ) {

    CAN_frame_t canFrame;
    canFrame.MsgID = 0x0CEFF02C;
    canFrame.FIR.B.FF = CAN_frame_ext;
    canFrame.FIR.B.DLC = 6;
    canFrame.data.u8[0] = 5;
    canFrame.data.u8[1] = 9;
    canFrame.data.u8[3] = 10;
    if( steerSetpoints.enabled == true && steerSetpoints.speed > steerConfig.minAutosteerSpeed ){
      canFrame.data.u8[2] = 3;
      canFrame.data.u8[4] = ( uint8_t ) highByte( machine.valveOutput );
      canFrame.data.u8[5] = ( uint8_t ) lowByte( machine.valveOutput );
    } else {
      canFrame.data.u8[2] = 2;
      canFrame.data.u8[4] = 0;
      canFrame.data.u8[5] = 0;
    }
    ESP32Can.CANWriteFrame( &canFrame );

    vTaskDelayUntil( &xLastWakeTime, xFrequency );
  }
}

void canFendtEngageReceiver10Hz( void* z ) { // Fendt engage bus
  constexpr TickType_t xFrequency = 100;
  TickType_t xLastWakeTime = xTaskGetTickCount();

  uint32_t claimISOBusAddress;
  uint32_t engageMessage;
  if( steerConfig.canBusFendtEngageVersion == SteerConfig::FendtEngageVersion::Hex18EF1CC8 ){
    claimISOBusAddress = 0x98EEFF1C;
    engageMessage = 0x18EF1CC8;
  } else if( steerConfig.canBusFendtEngageVersion == SteerConfig::FendtEngageVersion::Hex18EF2CF0 ){
    claimISOBusAddress = 0x98EEFF2C;
    engageMessage = 0x18EF2CF0;
  }
  bool canEngageMessage = false;

  int8_t SCK = 5, MISO = 16, MOSI = 17, CS = 0;
  hspi = new SPIClass( HSPI );
  hspi->begin( SCK, MISO, MOSI, CS );
  pinMode( CS, OUTPUT );
  MCP_CAN ISOBus( hspi, CS ); // Set SPI instance and CS

  if( ISOBus.begin( MCP_STDEXT, CAN_250KBPS, MCP_16MHZ ) == CAN_OK ){
    Serial.println( "MCP2515 started" );
  } else{
    Serial.println( "MCP2515 failed to start" );
    String str;
    str.reserve( 200 );
    str += "MCP2515 failed to start";
    ESPUI.updateLabel( labelStatusCanMCP2515, str );
    vTaskDelete( NULL );
  }
  ISOBus.init_Mask( 0, 1, 0x00FFFF00 ); // Init first mask
  ISOBus.init_Filt( 0, 1, engageMessage ); // Fendt engage
  ISOBus.init_Filt( 1, 1, engageMessage ); // Fendt engage

  ISOBus.init_Mask( 1, 1, 0x00FFFF00 ); // Init second mask
  ISOBus.init_Filt( 2, 1, engageMessage ); // Fendt engage
  ISOBus.init_Filt( 3, 1, engageMessage ); // Fendt engage
  ISOBus.init_Filt( 4, 1, engageMessage ); // Fendt engage
  ISOBus.init_Filt( 5, 1, engageMessage ); // Fendt engage
  ISOBus.setMode( MCP_NORMAL );
  byte data[8] = {0x00, 0x00, 0xC0, 0x0C, 0x00, 0x17, 0x02, 0x20};
  ISOBus.sendMsgBuf( claimISOBusAddress, 8, data );
  long unsigned int rxId;
  unsigned char len = 0;
  unsigned char rxBuf[8];

  for( ;; ) {
    
    if( ISOBus.checkReceive() == CAN_MSGAVAIL ){
      ISOBus.readMsgBuf( &rxId, &len, rxBuf );  // Read data: len = data length, buf = data byte(s)
      if(( rxId & 0x80000000 ) == 0x80000000 ){ // extended frame
        rxId = rxId & 0x1FFFFFFF;
      }
      if( rxId == engageMessage ){
        if( rxBuf[0] == 0x0F && rxBuf[1] == 0x60 && rxBuf[2] == 0x01 ){
          machine.steeringEnabled = true;
          canEngageMessage = true;
          machine.disengageInput = false;
        } else if( rxBuf[0] == 0x0F && rxBuf[1] == 0x60 && rxBuf[2] == 0x40 ){
          machine.steeringEnabled = false;
          canEngageMessage = false;
          machine.disengageInput = true;
        }
        machine.lastMCP2515CanbusMillis = millis();
      }
    } else { 
        static time_t loopTimeToWaitTo = 0;

        if( loopTimeToWaitTo < millis() ) {

          String str;
          str.reserve( 200 );

          str = String( rxId, HEX );
          str += canEngageMessage ? " Engage On received " : " Engage Off received ";
          time_t elapse = millis() - machine.lastMCP2515CanbusMillis;
          str += String( elapse / 1000 );
          str += " seconds ago";

          ESPUI.updateLabel( labelStatusCanMCP2515, str );

          loopTimeToWaitTo = millis() + 1000;
        }
    }
    vTaskDelayUntil( &xLastWakeTime, xFrequency );
  }
}

void canVmcMcp2515Receiver10Hz( void* z ) { // Valtra-Massey-Challenger second bus
  constexpr TickType_t xFrequency = 100;
  TickType_t xLastWakeTime = xTaskGetTickCount();

  uint32_t canIdVmcScv = 0x00331;

  int8_t SCK = 5, MISO = 16, MOSI = 17, CS = 0;
  hspi = new SPIClass( HSPI );
  hspi->begin( SCK, MISO, MOSI, CS );
  pinMode( CS, OUTPUT );
  MCP_CAN ISOBus( hspi, CS ); // Set SPI instance and CS

  if( ISOBus.begin( MCP_STDEXT, CAN_250KBPS, MCP_16MHZ ) == CAN_OK ){
    Serial.println( "MCP2515 started" );
  } else{
    Serial.println( "MCP2515 failed to start" );
    String str;
    str.reserve( 200 );
    str += "MCP2515 failed to start";
    ESPUI.updateLabel( labelStatusCanMCP2515, str );
    vTaskDelete( NULL );
  }
  ISOBus.init_Mask( 0, 1, 0x00FFFF00 ); // Init first mask
  ISOBus.init_Filt( 0, 1, canIdVmcScv ); // VMC hydraulic SCV
  ISOBus.init_Filt( 1, 1, canIdVmcScv ); // VMC hydraulic SCV

  ISOBus.init_Mask( 1, 1, 0x00FFFF00 ); // Init second mask
  ISOBus.init_Filt( 2, 1, canIdVmcScv ); // VMC hydraulic SCV
  ISOBus.init_Filt( 3, 1, canIdVmcScv ); // VMC hydraulic SCV
  ISOBus.init_Filt( 4, 1, canIdVmcScv ); // VMC hydraulic SCV
  ISOBus.init_Filt( 5, 1, canIdVmcScv ); // VMC hydraulic SCV
  ISOBus.setMode( MCP_NORMAL );
  long unsigned int rxId;
  unsigned char len = 0;
  unsigned char rxBuf[8];

  for( ;; ) {

    if( ISOBus.checkReceive() == CAN_MSGAVAIL ){
      ISOBus.readMsgBuf( &rxId, &len, rxBuf );  // Read data: len = data length, buf = data byte(s)
      if(( rxId & 0x80000000 ) != 0x80000000 ){ // standard frame
        if( rxId == canIdVmcScv ){
          machine.lastMCP2515CanbusMillis = millis();
        }
      }
    } else {
        static time_t loopTimeToWaitTo = 0;

        if( loopTimeToWaitTo < millis() ) {

          String str;
          str.reserve( 200 );

          str = String( rxId, HEX );
          str += " ";
          str += canIdVmcScv;
          str += " ";
          time_t elapse = millis() - machine.lastMCP2515CanbusMillis;
          str += String( elapse / 1000 );
          str += " seconds ago";

          ESPUI.updateLabel( labelStatusCanMCP2515, str );

          loopTimeToWaitTo = millis() + 1000;
        }
    }
    vTaskDelayUntil( &xLastWakeTime, xFrequency );
  }
}

void canbusStateMessage( void* z ){
  for( ;; ){
    if( canReceiverHandle == NULL ){
      uint8_t data[30] = {0x80, 0x81, 0x7F, 0xDD, 24, 3, 0, 0x43, 0x61, 0x6E, 0x62, 0x75, 0x73, 0x20, 0x66, 0x61, 0x69, 0x6C, 0x65, 0x64, 0x20, 0x74, 0x6F, 0x20, 0x73, 0x74, 0x61, 0x72, 0x74}; //Canbus failed to start
      int CRCtoAOG = 0;
      for ( byte i = 2; i < sizeof( data ) - 1; i++ ){
        CRCtoAOG = ( CRCtoAOG + data[ i ] );
      }
      data[ sizeof( data ) - 1 ] = CRCtoAOG;
      udpHardwareMessage.writeTo( data, sizeof( data ), ipDestination, initialisation.portSendTo );
      vTaskDelay( pdMS_TO_TICKS( 5000 )); // keep on showing this message
    } else {
      uint8_t data[33] = {0x80, 0x81, 0x7F, 0xDD, 27, 3, 1, 0x43, 0x61, 0x6E, 0x62, 0x75, 0x73, 0x20, 0x63, 0x6F, 0x6E, 0x74, 0x72, 0x6F, 0x6C, 0x6C, 0x65, 0x72, 0x20, 0x73, 0x74, 0x61, 0x72, 0x74, 0x65, 0x64}; //Canbus controller started
      int CRCtoAOG = 0;
      for ( byte i = 2; i < sizeof( data ) - 1; i++ ){
        CRCtoAOG = ( CRCtoAOG + data[ i ] );
      }
      data[ sizeof( data ) - 1 ] = CRCtoAOG;
      udpHardwareMessage.writeTo( data, sizeof( data ), ipDestination, initialisation.portSendTo );
      vTaskDelete( NULL ); // delete this task, canbus started
    }
  }
}

void canComplementSwitchWorker10Hz( void* z ) {
  constexpr TickType_t xFrequency = 100;
  TickType_t xLastWakeTime = xTaskGetTickCount();
  bool previousState;

  for( ;; ) {
   
    bool state = digitalRead(( uint8_t )steerConfig.gpioSteerswitch );
    if( previousState != state ){
      if( state == steerConfig.steerswitchActiveLow ){
        if(( machine.lastCanbusSteeringMillis + 500 ) < millis() ){ // Canbus steering timeout
          showHardwareStateOnAOG( 0x60 );
        } else if( machine.canbusSteeringState == 0x10 ){ // only try to engage when machine is ready, to avoid race conditions
          if( steerSetpoints.speed > steerConfig.maxAutosteerSpeed ) {
            machine.steeringEnabled = false;
            safety.autosteerDisabledByMaxEngageSpeed = true;
          } else {
            machine.steeringEnabled = true;
            AogToMachineEngagedMismatch = true;
          }
        } else {
          showHardwareStateOnAOG( machine.canbusSteeringState );
        }
      }
      previousState = state;
    }
    vTaskDelayUntil( &xLastWakeTime, xFrequency );
  }
}

void claimAddress( CAN_frame_t msgISO ){
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

void initCan() {
  if( steerConfig.canBusEnabled ) {
    Serial.println("Canbus enabled");
    udpHardwareMessage.listen( initialisation.portSendFrom );
    xTaskCreate( canbusStateMessage, "canbusStateMessage", 2048, NULL, 0, NULL );
    ESPUI.getControl( labelStatusCanESP32 )->color = ControlColor::Alizarin;
    ESPUI.updateLabel( labelStatusCanESP32, "Canbus controller failed to start"); // will be updated when Canbus thread runs
    CAN_cfg.speed = ( CAN_speed_t )steerConfig.canBusSpeed;
    CAN_cfg.tx_pin_id = ( gpio_num_t )steerConfig.canBusTx;
    CAN_cfg.rx_pin_id = ( gpio_num_t )steerConfig.canBusRx;
    CAN_cfg.rx_queue = xQueueCreate( rxQueueSize, sizeof( CAN_frame_t ) );
    // Init CAN Module
    ESP32Can.CANInit();
    CAN_frame_t msgISO;
    if( steerConfig.outputType == SteerConfig::OutputType::Canbus13_19Controller ){
      msgISO.MsgID = 0x18EEFF1C;
      claimAddress( msgISO );
      xTaskCreate( can13_19Sender100Hz, "can13_19Sender", 2048, NULL, 5, &canSenderHandle );
      xTaskCreate( canComplementSwitchWorker10Hz, "canComplementSwitch", 2048, NULL, 5, NULL );
      xTaskCreate( canReceiver10Hz, "canReceiver", 2048, NULL, 5, &canReceiverHandle );
      Serial.println("Danfoss 13/19 valve started");
    } else if( steerConfig.outputType == SteerConfig::OutputType::CanbusF0_240Controller ){
      msgISO.MsgID = 0x18EEFF2C;
      claimAddress( msgISO );
      xTaskCreate( canF0_240Sender50Hz, "canF0_240Sender", 2048, NULL, 5, &canSenderHandle );
      xTaskCreate( canComplementSwitchWorker10Hz, "canComplementSwitch", 2048, NULL, 5, NULL );
      xTaskCreate( canFendtEngageReceiver10Hz, "canFendtEngageReceiver", 4096, NULL, 5, NULL );
      xTaskCreate( canFendtSteeringReceiver100Hz, "canReceiver", 2048, NULL, 5, &canReceiverHandle );
      Serial.println("F0/240 valve started");
    } else {
      xTaskCreate( canReceiver10Hz, "canReceiver", 2048, NULL, 5, &canReceiverHandle );
      Serial.println("Non-steering Canbus receiver started");
    }
    ESPUI.getControl( labelStatusCanESP32 )->color = ControlColor::Emerald;

  }
}