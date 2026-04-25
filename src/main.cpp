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

#include <ESPUI.h>

#include <AsyncElegantOTA.h>

///////////////////////////////////////////////////////////////////////////
// global data
///////////////////////////////////////////////////////////////////////////

Diagnostics diagnostics;
DiagnosticsDisplay diagnosticsDisplay;
SteerConfig steerConfig, steerConfigDefaults;
Initialisation initialisation;
SteerCanData steerCanData = {0};
Machine machine;
Safety safety;

portMUX_TYPE mux = portMUX_INITIALIZER_UNLOCKED;
SemaphoreHandle_t i2cMutex;
TimerHandle_t saveTimer;
bool saveConfigRequested = false;

IPAddress wifiIP( 192, 168, 1, 1 );


///////////////////////////////////////////////////////////////////////////
// external Libraries
///////////////////////////////////////////////////////////////////////////

///////////////////////////////////////////////////////////////////////////
// Application
///////////////////////////////////////////////////////////////////////////
void saveTimerCallback( TimerHandle_t xTimer ) {
  // Set flag to save config from main task, not from ISR
  saveConfigRequested = true;
}

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

  Serial.println( "Welcome to esp32-aog.\nThe selected mode is AgOpenGps.\nTo configure, please open the webui." );

  pinMode( steerConfig.apModePin, OUTPUT );
  digitalWrite( steerConfig.apModePin, LOW );

  initWiFi();

  Serial.println( "\n\nWiFi parameters:" );
  Serial.print( "Mode: " );
  if( WiFi.getMode() == WIFI_AP_STA ){
    Serial.println( "access point" );
    wifiIP = WiFi.softAPIP();
  } else {
    Serial.println( "client" );
    wifiIP = WiFi.localIP();
  }
  Serial.print( "IP address: " );
  Serial.println( wifiIP );

  i2cMutex = xSemaphoreCreateMutex();
  saveTimer = xTimerCreate( "SaveTimer", pdMS_TO_TICKS( 10000 ), pdFALSE, NULL, saveTimerCallback ); // save after 10 seconds of inactivity

  if( ( SteerConfig::OutputType ) steerConfig.outputType >= SteerConfig::OutputType::Canbus13_19Controller ||
      ( SteerConfig::AnalogIn ) steerConfig.wheelAngleInput >= SteerConfig::AnalogIn::CanbusValtraMasseyChallenger ){
    machine.canbusSteeringActive = true;
  }
  initESPUI();
  initWebServerFunctions();

  AsyncElegantOTA.begin( ESPUI.WebServer() );

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
  if( saveConfigRequested ) {
    saveConfigRequested = false;
    saveConfig();
    Serial.println( "Config saved after 10 seconds" );
  }
  vTaskDelay( 100 );
}
