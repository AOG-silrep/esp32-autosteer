// MIT License
//
// Copyright (c) 2026 Reuben Rissler
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

#include "main.hpp"

AsyncUDP udpHardwareMessage;

void showHardwareStateOnAOG( uint8_t state ){
  switch( state ){
    case 0x00: {
      uint8_t data[30] = {0x80, 0x81, 0x7F, 0xDD, 24, 3, 1, 0x52, 0x65, 0x6C, 0x65, 0x61, 0x73, 0x65, 0x20, 0x73, 0x74, 0x65, 0x65, 0x72, 0x69, 0x6E, 0x67, 0x77, 0x68, 0x65, 0x65, 0x6C, 0x00}; //Release steeringwheel
      int CRCtoAOG = 0;
      for ( byte i = 2; i < sizeof( data ) - 1; i++ ){
        CRCtoAOG = ( CRCtoAOG + data[ i ] );
      }
      data[ sizeof( data ) - 1 ] = CRCtoAOG;
      udpHardwareMessage.writeTo( data, sizeof( data ), ipDestination, initialisation.portSendTo );
    }
    break;
    case 0x20: {
      uint8_t data[26] = {0x80, 0x81, 0x7F, 0xDD, 20, 3, 1, 0x54, 0x75, 0x72, 0x6E, 0x20, 0x73, 0x74, 0x65, 0x65, 0x72, 0x69, 0x6E, 0x67, 0x77, 0x68, 0x65, 0x65, 0x6C, 0x00}; //Turn steeringwheel
      int CRCtoAOG = 0;
      for ( byte i = 2; i < sizeof( data ) - 1; i++ ){
        CRCtoAOG = ( CRCtoAOG + data[ i ] );
      }
      data[ sizeof( data ) - 1 ] = CRCtoAOG;
      udpHardwareMessage.writeTo( data, sizeof( data ), ipDestination, initialisation.portSendTo );
    }
    break;
    case 0x50: {
      uint8_t data[34] = {0x80, 0x81, 0x7F, 0xDD, 28, 3, 1, 0x4E, 0x6F, 0x20, 0x43, 0x61, 0x6E, 0x62, 0x75, 0x73, 0x20, 0x6F, 0x75, 0x74, 0x70, 0x75, 0x74, 0x20, 0x66, 0x72, 0x6F, 0x6D, 0x20, 0x41, 0x4F, 0x47, 0x20, 0x00}; //No Canbus output from AOG
      int CRCtoAOG = 0;
      for ( byte i = 2; i < sizeof( data ) - 1; i++ ){
        CRCtoAOG = ( CRCtoAOG + data[ i ] );
      }
      data[ sizeof( data ) - 1 ] = CRCtoAOG;
      udpHardwareMessage.writeTo( data, sizeof( data ), ipDestination, initialisation.portSendTo );
    }
    break;
    case 0x60: {
      uint8_t data[26] = {0x80, 0x81, 0x7F, 0xDD, 20, 3, 1, 0x41, 0x75, 0x74, 0x6F, 0x73, 0x74, 0x65, 0x65, 0x72, 0x20, 0x64, 0x69, 0x73, 0x61, 0x62, 0x6C, 0x65, 0x64, 0x00}; //Autosteer disabled
      int CRCtoAOG = 0;
      for ( byte i = 2; i < sizeof( data ) - 1; i++ ){
        CRCtoAOG = ( CRCtoAOG + data[ i ] );
      }
      data[ sizeof( data ) - 1 ] = CRCtoAOG;
      udpHardwareMessage.writeTo( data, sizeof( data ), ipDestination, initialisation.portSendTo );
    }
    break;
    case 0x61: {
      uint8_t data[27] = {0x80, 0x81, 0x7F, 0xDD, 21, 3, 1, 0x54, 0x72, 0x61, 0x63, 0x74, 0x6F, 0x72, 0x20, 0x6E, 0x6F, 0x74, 0x20, 0x64, 0x72, 0x69, 0x76, 0x69, 0x6E, 0x67, 0x00}; //Tractor not driving
      int CRCtoAOG = 0;
      for ( byte i = 2; i < sizeof( data ) - 1; i++ ){
        CRCtoAOG = ( CRCtoAOG + data[ i ] );
      }
      data[ sizeof( data ) - 1 ] = CRCtoAOG;
      udpHardwareMessage.writeTo( data, sizeof( data ), ipDestination, initialisation.portSendTo );
    }
    break;
    default: {
      uint8_t data[21] = {0x80, 0x81, 0x7F, 0xDD, 16, 3, 1, 0x55, 0x6E, 0x6B, 0x6E, 0x6F, 0x77, 0x6E, 0x20, 0x65, 0x72, 0x72, 0x6F, 0x72, 0x00}; //Unknown error
      int CRCtoAOG = 0;
      for ( byte i = 2; i < sizeof( data ) - 1; i++ ){
        CRCtoAOG = ( CRCtoAOG + data[ i ] );
      }
      data[ sizeof( data ) - 1 ] = CRCtoAOG;
      udpHardwareMessage.writeTo( data, sizeof( data ), ipDestination, initialisation.portSendTo );
    }
    break;
  }
}
