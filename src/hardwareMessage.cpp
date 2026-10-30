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

AsyncUDP udpHardwareMessage; // socket used only for the pop-up text messages sent to AgOpenGPS

// Sends 'text' as PGN 0xDD (221), which AgOpenGPS shows in its hardware message banner:
//   [0..1] 0x80 0x81 header, [2] 0x7F source (steer module), [3] 0xDD PGN,
//   [4] length of bytes 5 and 6, the text and its terminating 0,
//   [5] seconds to show the banner, [6] banner color: 0 salmon (alarm), otherwise bisque,
//   [7..] the text as ASCII, terminated with 0x00,
//   [last] CRC, the sum of bytes 2 up to the terminating 0.
void sendAogMessage( const String& text, bool alarm ){
  uint8_t data[64];
  size_t textLength = min( text.length(), sizeof( data ) - 9 );
  data[0] = 0x80;
  data[1] = 0x81;
  data[2] = 0x7F; // source: steer module
  data[3] = 0xDD;
  data[4] = textLength + 3; // bytes 5 and 6, the text and its terminating 0
  data[5] = 3;
  data[6] = alarm ? 0 : 1;
  memcpy( data + 7, text.c_str(), textLength );
  data[7 + textLength] = 0;
  uint8_t crc = 0;
  for( size_t i = 2; i < 8 + textLength; i++ ){
    crc += data[i];
  }
  data[8 + textLength] = crc;
  udpHardwareMessage.writeTo( data, 9 + textLength, ipDestination, initialisation.portSendTo );
}

// Tells the operator why autosteer did not engage, or what went wrong with the hardware.
//
// 'state' is normally machine.canbusSteeringState as received from the tractor
// (see the decoding in diagnostics.cpp), but callers also pass their own codes
// for conditions that do not come off the canbus. Each code and the text it
// sends is described on its own case below.
void showHardwareStateOnAOG( uint8_t state ){
  switch( state ){
    case 0x00: // handwheel activity, sent from canbus.cpp
      sendAogMessage( "Release steeringwheel" );
      break;
    case 0x20: // stagnant, sent from canbus.cpp
      sendAogMessage( "Turn steeringwheel" );
      break;
    case 0x50: // no message from the steering controller, sent from canbus.cpp
      sendAogMessage( "No Canbus output from AOG" );
      break;
    case 0x60: // not initialized / canbus steering timeout, sent from canbus.cpp
      sendAogMessage( "Autosteer disabled" );
      break;
    case 0x61: // tractor not driving, sent from canbus.cpp
      sendAogMessage( "Tractor not driving" );
      break;
    case 0x70: // too fast to engage, sent from autosteer.cpp
      sendAogMessage( "Slow down to engage" );
      break;
    case 0x75: // WAS jumped more than 5 degrees between samples, sent from sensor.cpp
      sendAogMessage( "WAS implausibility error" );
      break;
    default: // anything else, sent from canbus.cpp
      sendAogMessage( "Unknown error" );
      break;
  }
}
