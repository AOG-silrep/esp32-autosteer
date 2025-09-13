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

#include "jsonFunctions.hpp"
#include "main.hpp"

#include <ESPmDNS.h>
#include <WiFi.h>

#include <DNSServer.h>
#include <ESPUI.h>

#include <AsyncElegantOTA.h>

///////////////////////////////////////////////////////////////////////////
// global data
///////////////////////////////////////////////////////////////////////////

Diagnostics diagnostics;
SteerConfig steerConfig, steerConfigDefaults;
Initialisation initialisation;
SteerCanData steerCanData = {0};
Machine machine;
Safety safety;

portMUX_TYPE mux = portMUX_INITIALIZER_UNLOCKED;
SemaphoreHandle_t i2cMutex;

const byte DNS_PORT = 53;
IPAddress apIP( 192, 168, 1, 1 );

///////////////////////////////////////////////////////////////////////////
// external Libraries
///////////////////////////////////////////////////////////////////////////
DNSServer dnsServer;

///////////////////////////////////////////////////////////////////////////
// Application
///////////////////////////////////////////////////////////////////////////
void setup( void ) {
  Serial.begin( 115200 );

  WiFi.disconnect( true );

  Wire.begin( ( int )steerConfig.gpioSDA, ( int )steerConfig.gpioSCL, steerConfig.i2cBusSpeed );

  if( !LittleFS.begin( true ) ) {
    Serial.println( "LittleFS Mount Failed" );
    return;
  }

  loadSavedConfig();
  loadSavedDiagnostics();

  Serial.updateBaudRate( steerConfig.baudrate );

  Serial.println( "Welcome to esp32-aog.\nThe selected mode is AgOpenGps.\nTo configure, please open the webui." );

  pinMode( steerConfig.apModePin, OUTPUT );
  digitalWrite( steerConfig.apModePin, LOW );

  initWiFi();
  apIP = WiFi.localIP();

  dnsServer.start( DNS_PORT, "*", apIP );

  Serial.println( "\n\nWiFi parameters:" );
  Serial.print( "Mode: " );
  Serial.println( WiFi.getMode() == WIFI_AP ? "Station" : "Client" );
  Serial.print( "IP address: " );
  Serial.println( WiFi.getMode() == WIFI_AP ? WiFi.softAPIP() : WiFi.localIP() );

  i2cMutex = xSemaphoreCreateMutex();

  /*
  * .begin loads and serves all files from PROGMEM directly.
  * If you want to serve the files from LittleFS use ESPUI.beginLittleFS
  * (.prepareFileSystem has to be run in an empty sketch before)
  */

  /*
  * Optionally you can use HTTP BasicAuth. Keep in mind that this is NOT a
  * SECURE way of limiting access.
  * Anyone who is able to sniff traffic will be able to intercept your password
  * since it is transmitted in cleartext. Just add a username and password,
  * for example begin("ESPUI Control", "username", "password")
  */

  if( ( SteerConfig::OutputType ) steerConfig.outputType >= SteerConfig::OutputType::Canbus13_19Controller ||
      ( SteerConfig::AnalogIn ) steerConfig.wheelAngleInput >= SteerConfig::AnalogIn::CanbusValtraMasseyChallenger ){
    machine.canbusSteeringActive = true;
  }
  initESPUI();

  {
  String str;
  str.reserve( 30 );
  str = "Number of faults: ";
  str += ( uint8_t ) diagnostics.steerEnabledWithNoPower;
  str += "\nFault active since startup: No";
  ESPUI.updateLabel( labelSteerEngagedFaults, str );
  }

  if( steerConfig.enableOTA ) {
    AsyncElegantOTA.begin( ESPUI.WebServer() );
  }

  if( steerConfig.aogPortSendFrom != 0 ) {
    initialisation.portSendFrom = steerConfig.aogPortSendFrom;
  }
  if( steerConfig.aogPortListenTo != 0 ) {
    initialisation.portListenTo = steerConfig.aogPortListenTo;
  }
  if( steerConfig.aogPortSendTo != 0 ) {
    initialisation.portSendTo = steerConfig.aogPortSendTo;
  }

  initIdleStats();
  initCan();
  initSensors();
  initAutosteer();
  initDiagnostics();

  if( !MDNS.begin( "steering" )){
    Serial.println( "Error starting mDNS" );
  }
}

void loop( void ) {
  dnsServer.processNextRequest();
  vTaskDelay( 100 );
}
