

#include <ESPUI.h>
#include "main.hpp"
#include "jsonFunctions.hpp"

static String escapeJsonString( const String & input ) {
  String escaped;
  escaped.reserve( input.length() * 2 );
  for( size_t i = 0; i < input.length(); ++i ) {
    char c = input[ i ];
    switch( c ) {
      case '"': escaped += "\\\""; break;
      case '\\': escaped += "\\\\"; break;
      case '\b': escaped += "\\b"; break;
      case '\f': escaped += "\\f"; break;
      case '\n': escaped += "\\n"; break;
      case '\r': escaped += "\\r"; break;
      case '\t': escaped += "\\t"; break;
      default:
        if( ( uint8_t )c < 0x20 ) {
          char buf[ 7 ];
          snprintf( buf, sizeof( buf ), "\\u%04x", ( uint8_t )c );
          escaped += buf;
        } else {
          escaped += c;
        }
        break;
    }
  }
  return escaped;
}

void initWebServerFunctions( void ) {
  ESPUI.WebServer()->on("/diagnostics", HTTP_GET, [](AsyncWebServerRequest *request){
    String html = R"html(
      <!DOCTYPE html>
      <html>
      <head>
      <title>Diagnostics</title>
      <link rel="stylesheet" href="/css/normalize.css">
      <link rel="stylesheet" href="/css/style.css">
      <script>
      function loadData() {
        fetch('/diagnosticData', {method: 'GET'})
          .then(response => response.json())
          .then(data => {
            document.getElementById('agOpenGps').innerText = data.agOpenGpsAddress;
            document.getElementById('safety').innerText = data.safetyDisableAutosteer;
            document.getElementById('voltage').innerText = data.supplyVoltage;
            document.getElementById('current').innerText = data.steerMotorCurrent;
            document.getElementById('faults').innerText = data.steerEngagedFaults;
            document.getElementById('switches').innerText = data.switchStates;
          })
          .catch(error => {
            console.error('Diagnostics load error', error);
            document.getElementById('agOpenGps').innerText = 'Error';
            document.getElementById('safety').innerText = 'Error';
            document.getElementById('voltage').innerText = 'Error';
            document.getElementById('current').innerText = 'Error';
            document.getElementById('faults').innerText = 'Error';
            document.getElementById('switches').innerText = 'Error';
          });
      }
      function reset() {
        fetch('/diagnosticsReset', {method: 'POST'})
          .then(() => loadData());
      }
      setInterval(loadData, 1000);
      window.onload = loadData;
      </script>
      </head>
      <body>
      <div class="container">
      <h1>Diagnostics</h1>
      <p><strong>AgOpenGPS communication:</strong> <span id="agOpenGps"></span></p>
      <p><strong>Safety disable autosteer:</strong> <span id="safety"></span></p>
      <p><strong>Steer valve supply voltage:</strong> <span id="voltage"></span></p>
      <p><strong>Steer motor current:</strong> <span id="current"></span></p>
      <p><strong>Steering engaged with no power:</strong> <span id="faults"></span></p>
      <p><strong>Switch states:</strong> <span id="switches"></span></p>
      <button onclick="reset()">Reset all to zero</button>
      </div>
      </body>
      </html>
      )html";
    request->send(200, "text/html", html);
  });

  ESPUI.WebServer()->on("/diagnosticData", HTTP_GET, [](AsyncWebServerRequest *request){
    String json = "{";
    json += "\"agOpenGpsAddress\":\"" + escapeJsonString( diagnosticsDisplay.agOpenGpsAddress ) + "\",";
    json += "\"safetyDisableAutosteer\":\"" + escapeJsonString( diagnosticsDisplay.safetyDisableAutosteer ) + "\",";
    json += "\"supplyVoltage\":\"" + escapeJsonString( diagnosticsDisplay.supplyVoltage ) + "\",";
    json += "\"steerMotorCurrent\":\"" + escapeJsonString( diagnosticsDisplay.steerMotorCurrent ) + "\",";
    json += "\"steerEngagedFaults\":\"" + escapeJsonString( diagnosticsDisplay.steerEngagedFaults ) + "\",";
    json += "\"switchStates\":\"" + escapeJsonString( diagnosticsDisplay.switchStates ) + "\"";
    json += "}";
    request->send(200, "application/json", json);
  });

  /*ESPUI.WebServer()->on( "/test", HTTP_GET, [](AsyncWebServerRequest *request ) {
    Serial.printf( "Client %s requested /test\n",
                  request->client()->remoteIP().toString().c_str());
    request->send( 200, "text/plain", "OK" );
  });
  */

  ESPUI.WebServer()->on( "/diagnosticsReset", HTTP_POST, [](AsyncWebServerRequest *request ){
    diagnostics.steerSupplyVoltageMax = machine.steerSupplyVoltage;
    diagnostics.steerSupplyVoltageMin = machine.steerSupplyVoltage;
    diagnostics.steerEnabledWithNoPower = 0;
    diagnostics.fuse1Shorted = 0;
    diagnostics.fuse2Shorted = 0;
    diagnostics.UDPTimeout = 0;
    saveDiagnostics();
    // Update the display
    String str;
    str.reserve( 30 );
    str = "\nNumber of faults: ";
    str += ( int8_t ) diagnostics.steerEnabledWithNoPower;
    str += "\nFault active since startup: No";
    diagnosticsDisplay.steerEngagedFaults = str;
    request->send(200, "text/plain", "Reset done");
  });
}
