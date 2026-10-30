

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

static String escapeHtmlString( const String & input ) {
  String escaped;
  escaped.reserve( input.length() * 2 );
  for( size_t i = 0; i < input.length(); ++i ) {
    char c = input[ i ];
    switch( c ) {
      case '&': escaped += "&amp;"; break;
      case '<': escaped += "&lt;"; break;
      case '>': escaped += "&gt;"; break;
      case '"': escaped += "&quot;"; break;
      case '\'': escaped += "&#39;"; break;
      default: escaped += c; break;
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
      <title>Diagnostics :: %HOSTNAME%</title>
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
            document.getElementById('WAS').innerText = data.wheelAngleSensor;
            document.getElementById('switches').innerText = data.switchStates;
            document.getElementById('implement').innerText = data.implementStates;
          })
          .catch(error => {
            console.error('Diagnostics load error', error);
            document.getElementById('agOpenGps').innerText = 'Error';
            document.getElementById('safety').innerText = 'Error';
            document.getElementById('voltage').innerText = 'Error';
            document.getElementById('current').innerText = 'Error';
            document.getElementById('faults').innerText = 'Error';
            document.getElementById('WAS').innerText = 'Error';
            document.getElementById('switches').innerText = 'Error';
            document.getElementById('implement').innerText = 'Error';
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
      <p><strong>Device:</strong> %HOSTNAME%</p>
      <p><strong>AgOpenGPS communication:</strong> <span id="agOpenGps"></span></p>
      <p><strong>Safety disable autosteer:</strong> <span id="safety"></span></p>
      <p><strong>Steer valve supply voltage:</strong> <span id="voltage"></span></p>
      <p><strong>Steer motor current:</strong> <span id="current"></span></p>
      <p><strong>Steering engaged with no power:</strong> <span id="faults"></span></p>
      <p><strong>Wheel angle sensor:</strong> <span id="WAS"></span></p>
      <p><strong>Switch states:</strong> <span id="switches"></span></p>
      <p><strong>Implement:</strong> <span id="implement"></span></p>
      <button onclick="reset()">Reset all to zero</button>
      </div>
      </body>
      </html>
      )html";
    html.replace( "%HOSTNAME%", escapeHtmlString( String( steerConfig.hostname ) ) );
    request->send(200, "text/html", html);
  });

  ESPUI.WebServer()->on("/diagnosticData", HTTP_GET, [](AsyncWebServerRequest *request){
    DiagnosticsDisplay snapshot;
    if( xSemaphoreTake( diagnosticsDisplayMutex, pdMS_TO_TICKS( 100 )) != pdTRUE ) {
      request->send( 503, "text/plain", "Diagnostics busy" );
      return;
    }
    snapshot = diagnosticsDisplay; // copy under the lock, escape and serialize outside it
    xSemaphoreGive( diagnosticsDisplayMutex );

    String json = "{";
    json += "\"agOpenGpsAddress\":\"" + escapeJsonString( snapshot.agOpenGpsAddress ) + "\",";
    json += "\"safetyDisableAutosteer\":\"" + escapeJsonString( snapshot.safetyDisableAutosteer ) + "\",";
    json += "\"supplyVoltage\":\"" + escapeJsonString( snapshot.supplyVoltage ) + "\",";
    json += "\"steerMotorCurrent\":\"" + escapeJsonString( snapshot.steerMotorCurrent ) + "\",";
    json += "\"steerEngagedFaults\":\"" + escapeJsonString( snapshot.steerEngagedFaults ) + "\",";
    json += "\"wheelAngleSensor\":\"" + escapeJsonString( snapshot.wheelAngleSensor ) + "\",";
    json += "\"switchStates\":\"" + escapeJsonString( snapshot.switchStates ) + "\",";
    json += "\"implementStates\":\"" + escapeJsonString( snapshot.implementStates ) + "\"";
    json += "}";
    request->send(200, "application/json", json);
  });

  ESPUI.WebServer()->on( "/diagnosticsReset", HTTP_POST, [](AsyncWebServerRequest *request ){
    diagnostics.steerSupplyVoltageMax = machine.steerSupplyVoltage;
    diagnostics.steerSupplyVoltageMin = machine.steerSupplyVoltage;
    diagnostics.steerEnabledWithNoPower = 0;
    diagnostics.WasPlausibilityErrors = 0;
    diagnostics.WasPositiveSupplyShortedToGround = 0;
    diagnostics.fuse2Shorted = 0;
    diagnostics.UDPTimeout = 0;
    saveDiagnostics();
    // Update the display
    String str;
    str.reserve( 30 );
    str = "\nNumber of faults: ";
    str += diagnostics.steerEnabledWithNoPower;
    str += "\nFault active since startup: No";
    if( xSemaphoreTake( diagnosticsDisplayMutex, pdMS_TO_TICKS( 50 )) == pdTRUE ) {
      diagnosticsDisplay.steerEngagedFaults = str;
      xSemaphoreGive( diagnosticsDisplayMutex );
    }
    request->send(200, "text/plain", "Reset done");
  });

  ESPUI.WebServer()->on("/wasCalibration", HTTP_GET, [](AsyncWebServerRequest *request){
    String html = R"html(
      <!DOCTYPE html>
      <html>
      <head>
      <meta charset="utf-8">
      <meta name="viewport" content="width=device-width, initial-scale=1">
      <title>WAS Calibration :: %HOSTNAME%</title>
      <link rel="stylesheet" href="/css/normalize.css">
      <link rel="stylesheet" href="/css/style.css">
      <style>
      canvas { display: block; box-sizing: border-box; width: 100%; height: 120px; background: #fff; border: 1px solid #ccc; margin-bottom: 8px; }
      td, th { padding: 2px 10px; text-align: right; }
      #error { color: #c0392b; }
      #integralZero { width: 20px; height: 20px; margin: 0 8px 0 0; vertical-align: middle; box-shadow: none; } /* ESPUI makes inputs full width */
      </style>
      <script>
      var samples = [];
      var samplesLength = 240; // 2 minutes at 2 Hz
      var last = null;
      function stateColor(sample) { // grey when not collecting, red until the pass has 15 s, then green
        return !sample.collecting ? '#95a5a6' : sample.passDone ? '#27ae60' : '#e74c3c';
      }
      function drawChart(id, key, minRange, unit) {
        var canvas = document.getElementById(id);
        var ctx = canvas.getContext('2d');
        var w = canvas.width = canvas.clientWidth, h = canvas.height, mid = h / 2;
        var range = minRange;
        samples.forEach(p => { if (p[key] !== null) range = Math.max(range, Math.abs(p[key])); });
        var x = i => i * w / (samplesLength - 1);
        var y = v => mid - v * (mid - 6) / range;
        ctx.clearRect(0, 0, w, h);
        ctx.strokeStyle = '#bbb';
        ctx.lineWidth = 1;
        ctx.beginPath(); ctx.moveTo(0, mid); ctx.lineTo(w, mid); ctx.stroke();
        ctx.fillStyle = '#777';
        ctx.fillText('+' + range.toFixed(1) + unit, 4, 12);
        ctx.fillText('-' + range.toFixed(1) + unit, 4, h - 4);
        ctx.lineWidth = 2;
        for (var i = 1; i < samples.length; i++) {
          var a = samples[i - 1][key], b = samples[i][key];
          if (a === null || b === null) continue;
          ctx.strokeStyle = stateColor(samples[i]);
          ctx.beginPath(); ctx.moveTo(x(i - 1), y(a)); ctx.lineTo(x(i), y(b)); ctx.stroke();
        }
      }
      function updateButtons() {
        document.getElementById('start').disabled = !document.getElementById('integralZero').checked || (last !== null && last.running);
        document.getElementById('apply').disabled = last === null || last.passes.length === 0;
      }
      function loadData() {
        fetch('/wasCalibrationData', {method: 'GET'})
          .then(response => response.json())
          .then(data => {
            last = data;
            var sample = {xte: data.xteValid ? data.xte : null, angle: data.angle, collecting: data.collecting,
              passDone: data.passSeconds >= data.passTarget};
            samples.push(sample);
            if (samples.length > samplesLength) samples.shift();
            var status = document.getElementById('status');
            status.innerText = data.status;
            status.style.color = stateColor(sample);
            document.getElementById('live').innerText = 'cross track error ' + (data.xteValid ? data.xte + ' cm' : 'none') +
              ', wheel angle ' + data.angle.toFixed(2) + '°, speed ' + data.speed.toFixed(1) + ' ' + data.speedUnits;
            document.getElementById('pass').innerText = data.running ?
              data.passSeconds + ' s of steady tracking, ' + data.passTarget + ' s needed. The pass is saved when you leave the line.' : '-';
            var rows = '';
            data.passes.forEach((p, i) => {
              rows += '<tr><td>' + (i + 1) + '</td><td>' + p.direction + '</td><td>' + p.seconds + '</td><td>' + p.counts.toFixed(1) +
                '</td><td>' + p.degrees.toFixed(2) + '°</td></tr>';
            });
            document.getElementById('passes').innerHTML = rows;
            var result = 'Drive a pass to get a result.';
            if (data.passes.length > 0) {
              result = 'Sensor reads ' + data.error.toFixed(2) + '° when driving straight over ' + data.passes.length +
                ' pass(es), spread within a direction ' + data.spread.toFixed(2) + '°';
              if (data.slope !== 0) result += ', side slope or line curvature ±' + data.slope.toFixed(2) + '°';
              result += '. ' + confidenceText(data);
              result += ' Apply sets ' + (data.applyAsOffset ? 'Wheel Angle Offset to ' + data.newValue.toFixed(2) + '°' :
                'Wheel Angle Sensor Center to ' + data.newValue) + '.';
            }
            document.getElementById('result').innerText = result;
            document.getElementById('message').innerText = data.message;
            updateButtons();
            drawChart('xteChart', 'xte', 10, ' cm');
            drawChart('angleChart', 'angle', 2, '°');
          })
          .catch(error => {
            console.error('WAS calibration load error', error);
            document.getElementById('status').innerText = 'Error';
          });
      }
      function send(action) {
        var body = new URLSearchParams();
        body.append('action', action);
        if (document.getElementById('integralZero').checked) body.append('integralZeroConfirmed', '1');
        fetch('/wasCalibrationControl', {method: 'POST', body: body})
          .then(response => response.text().then(text => {
            document.getElementById('error').innerText = response.ok ? '' : text;
            if (action === 'reset' || (action === 'apply' && response.ok)) {
              document.getElementById('integralZero').checked = false; // each run needs a fresh confirmation
            }
            loadData();
          }));
      }
      function confidenceText(data) {
        return {
          good: 'Good.',
          needMorePasses: data.slope !== 0 ? 'Drive another pass in each direction to confirm.' :
            'Drive the line in the other direction to confirm.',
          passesDisagree: 'Passes in the same direction disagree, check the line is straight and the passes steady.'
        }[data.confidence];
      }
      function applyResult() {
        if (last !== null && last.confidence !== 'good' && !confirm(confidenceText(last) + ' Apply anyway?')) return;
        send('apply');
      }
      setInterval(loadData, 500);
      window.onload = loadData;
      </script>
      </head>
      <body>
      <div class="container">
      <h1>WAS Center Calibration</h1>
      <p><strong>Device:</strong> %HOSTNAME% &middot; <a href="/">Settings</a></p>
      <ol>
      <li>In AgOpenGPS, set the steering integral to 0. Note the current value so you can restore it afterwards.</li>
      <li>Use a straight AB line on level ground.</li>
      <li>Engage autosteer at working speed and let it settle on the line.</li>
      <li>Drive at least one pass in each direction, with 15 s or more of steady tracking per pass. On a side slope, drive two passes in each direction.</li>
      <li>Apply when the result shows Good, then restore the AgOpenGPS steering integral.</li>
      </ol>
      <p><label><input type="checkbox" id="integralZero" onchange="updateButtons()"> AgOpenGPS steering integral is set to 0</label></p>
      <p>
      <button id="start" onclick="send('start')" disabled>Start</button>
      <button id="apply" onclick="applyResult()" disabled>Apply</button>
      <button onclick="send('reset')">Reset</button>
      </p>
      <p id="error"></p>
      <p><strong>Status:</strong> <span id="status"></span></p>
      <p><strong>Live:</strong> <span id="live"></span></p>
      <p><strong>Current pass:</strong> <span id="pass"></span></p>
      <p>Cross track error (red while collecting, green once the pass has 15 s, grey when not collecting)</p>
      <canvas id="xteChart" height="120"></canvas>
      <p>Wheel angle</p>
      <canvas id="angleChart" height="120"></canvas>
      <table>
      <thead><tr><th>Pass</th><th>Direction</th><th>Steady s</th><th>WAS counts</th><th>Reads when straight</th></tr></thead>
      <tbody id="passes"></tbody>
      </table>
      <p><strong>Result:</strong> <span id="result"></span></p>
      <p id="message"></p>
      </div>
      </body>
      </html>
      )html";
    html.replace( "%HOSTNAME%", escapeHtmlString( String( steerConfig.hostname ) ) );
    request->send(200, "text/html", html);
  });

  ESPUI.WebServer()->on("/wasCalibrationData", HTTP_GET, [](AsyncWebServerRequest *request){
    String json = wasCalibrationJson();
    if( json.length() == 0 ) {
      request->send( 503, "text/plain", "Calibration busy" );
      return;
    }
    request->send(200, "application/json", json);
  });

  ESPUI.WebServer()->on( "/wasCalibrationControl", HTTP_POST, [](AsyncWebServerRequest *request ){
    if( !request->hasParam( "action", true ) ) {
      request->send( 400, "text/plain", "Missing action" );
      return;
    }
    String action = request->getParam( "action", true )->value();
    if( action == "start" ) {
      // AgOpenGPS does not send its integral gain, so the operator has to confirm it
      if( !request->hasParam( "integralZeroConfirmed", true ) || request->getParam( "integralZeroConfirmed", true )->value() != "1" ) {
        request->send( 400, "text/plain", "Confirm the AgOpenGPS steering integral is set to 0 first" );
        return;
      }
      wasCalibrationStart();
    } else if( action == "reset" ) {
      wasCalibrationReset();
    } else if( action == "apply" ) {
      if( !wasCalibrationApply() ) {
        request->send( 409, "text/plain", "Not applied, see the message below" );
        return;
      }
    } else {
      request->send( 400, "text/plain", "Unknown action" );
      return;
    }
    request->send(200, "text/plain", "OK");
  });
}
