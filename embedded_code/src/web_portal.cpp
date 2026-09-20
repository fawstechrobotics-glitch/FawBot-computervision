#include "config.h"
#include <ESPmDNS.h>

WiFiUDP udp;
const int UDP_PORT = 8888;
char udpBuffer[256];

// Track Python telemetry endpoint for direct UDP feedback
IPAddress pythonClientIP;
uint16_t pythonClientPort = 0;

const char index_html[] PROGMEM = R"rawliteral(
<!DOCTYPE HTML><html><head>
  <title>Stepper Robot Dashboard</title>
  <meta name="viewport" content="width=device-width, initial-scale=1">
  <style>
    body { font-family: Arial, sans-serif; text-align: center; background: #121212; color: #e0e0e0; padding: 5px; margin: 0; }
    .card { background: #1e1e1e; padding: 15px; margin: 10px auto; border-radius: 12px; border: 1px solid #333; max-width: 500px; }
    .status-dot { height: 12px; width: 12px; border-radius: 50%; display: inline-block; background: gray; margin: 0 5px; }
    .green { background: #28a745; } .red { background: #dc3545; }
    input { background: #2c2c2c; border: 1px solid #444; color: white; padding: 8px; width: 60px; border-radius: 4px; text-align: center; }
    label { font-size: 0.85rem; color: #aaa; margin-right: 5px; display: block; margin-bottom: 4px; }
    button { padding: 10px; border-radius: 8px; cursor: pointer; border: none; font-weight: bold; transition: 0.2s; }
    .btn-blue { background: #007bff; color: white; margin: 5px; }
    .btn-toggle { min-width: 110px; margin: 5px; }
    .d-pad { display: grid; grid-template-columns: 75px 75px 75px; gap: 10px; justify-content: center; margin: 15px auto; }
    .btn-manual { background: #333; color: white; font-size: 24px; width: 75px; height: 75px; display: flex; align-items: center; justify-content: center; border-radius: 10px; }
    .btn-manual:active { background: #555; }
    #console { height: 120px; background: #000; color: #00ff00; overflow-y: scroll; text-align: left; padding: 10px; font-size: 11px; margin-top: 10px; border-radius: 5px; font-family: monospace; border: 1px solid #333; }
    .setting-row { display: flex; justify-content: space-around; flex-wrap: wrap; gap: 10px; margin-top: 10px; }
    .input-box { flex: 1; min-width: 80px; }
  </style>
</head><body>
  <h3>Stepper Robot Controller</h3>

  <div class="card">
    L <div id="l-ir" class="status-dot"></div> | R <div id="r-ir" class="status-dot"></div>
    | Dist: <span id="dist-val">0</span> cm
  </div>

  <div class="card">
    <div class="d-pad">
      <div></div>
      <button class="btn-manual" onmousedown="ctrl('F')" onmouseup="ctrl('S')" ontouchstart="ctrl('F')" ontouchend="ctrl('S')">&#9650;</button>
      <div></div>
      <button class="btn-manual" onmousedown="ctrl('L')" onmouseup="ctrl('S')" ontouchstart="ctrl('L')" ontouchend="ctrl('S')">&#9664;</button>
      <button class="btn-manual" style="background:#dc3545" onclick="fetch('/stop')">&#9632;</button>
      <button class="btn-manual" onmousedown="ctrl('R')" onmouseup="ctrl('S')" ontouchstart="ctrl('R')" ontouchend="ctrl('S')">&#9654;</button>
      <div></div>
      <button class="btn-manual" onmousedown="ctrl('B')" onmouseup="ctrl('S')" ontouchstart="ctrl('B')" ontouchend="ctrl('S')">&#9660;</button>
      <div></div>
    </div>
  </div>

  <div class="card">
    <h4>Autonomous Movement</h4>
    <div class="setting-row">
      <div class="input-box"><label>Move (cm)</label><input type="number" id="moveCm" value="10"></div>
      <div class="input-box"><label>Degrees</label><input type="number" id="deg" value="90"></div>
    </div>
    <div class="setting-row">
      <button class="btn-blue" onclick="sendMoveCm(1)">Forward</button>
      <button class="btn-blue" onclick="sendMoveCm(-1)">Backward</button>
      <button class="btn-blue" onclick="sendTurn()">Turn</button>
    </div>
  </div>

  <div class="card">
    <h4>System Configuration</h4>
    <div class="setting-row">
        <div class="input-box"><label>Step Delay(&mu;s)</label><input type="number" id="delayUs" value="1000" onchange="updateConfig('delay', this.value)"></div>
        <div class="input-box"><label>Obs (cm)</label><input type="number" id="obsThresh" value="20" onchange="updateConfig('obs', this.value)"></div>
    </div>
    <div class="setting-row" style="margin-top:15px;">
        <button id="btnIR" class="btn-blue btn-toggle" onclick="toggleSensor('ir')">IR: ENABLED</button>
        <button id="btnUS" class="btn-blue btn-toggle" onclick="toggleSensor('us')">US: ENABLED</button>
    </div>
  </div>

  <div class="card" id="console"></div>

<script>
  function ctrl(c) { fetch(`/manual?cmd=${c}`); }
  function sendMoveCm(d) { fetch(`/move_cm?cm=${document.getElementById('moveCm').value}&dir=${d}`); }
  function sendTurn() { fetch(`/turn?deg=${document.getElementById('deg').value}`); }
  function updateConfig(param, val) { fetch(`/config?param=${param}&val=${val}`); }

  function toggleSensor(type) {
    fetch(`/toggle?type=${type}`).then(r => r.text()).then(state => {
       const btn = type === 'ir' ? document.getElementById('btnIR') : document.getElementById('btnUS');
       const isEnabled = state === '1';
       btn.innerText = (type === 'ir' ? 'IR: ' : 'US: ') + (isEnabled ? 'ENABLED' : 'DISABLED');
       btn.style.background = isEnabled ? '#007bff' : '#dc3545';
    });
  }

  var source = new EventSource('/events');
  source.addEventListener('log', function(e) {
    var c = document.getElementById('console');
    c.innerHTML += e.data + '<br>'; c.scrollTop = c.scrollHeight;
  }, false);
  
  source.addEventListener('sensor_data', function(e) {
    var data = JSON.parse(e.data);
    document.getElementById('l-ir').className = data.l ? "status-dot green" : "status-dot red";
    document.getElementById('r-ir').className = data.r ? "status-dot green" : "status-dot red";
    document.getElementById('dist-val').innerText = data.d;
  }, false);
</script></body></html>)rawliteral";

// Helper function to send UDP feedback to Python
void sendUDPFeedback(String message) {
    if (pythonClientPort != 0) {
        udp.beginPacket(pythonClientIP, pythonClientPort);
        udp.print(message);
        udp.endPacket();
    }
}

void startWebPortal() {
    WiFi.mode(WIFI_STA);
    WiFi.begin("saraths_Lab", "@Sarath123");

    Serial.print("Connecting to WiFi 'saraths_Lab'");
    while (WiFi.status() != WL_CONNECTED) {
        delay(500);
        Serial.print(".");
    }

    Serial.println("");
    Serial.print("Connected! IP Address: ");
    Serial.println(WiFi.localIP());

    if (MDNS.begin(ROBOT_NAME)) {
        Serial.println("mDNS responder started!");
        Serial.print("Access at: http://");
        Serial.print(ROBOT_NAME);
        Serial.println(".local/Fawbot");
    }

    udp.begin(UDP_PORT);
    Serial.print("UDP Listener active on port ");
    Serial.println(UDP_PORT);

    // Web Routes
    server.on("/Fawbot", HTTP_GET, [](AsyncWebServerRequest *r){ 
        r->send(200, "text/html", index_html); 
    });

    server.on("/manual", HTTP_GET, [](AsyncWebServerRequest *r){
        String cmd = r->getParam("cmd")->value();
        if (cmd == "S") {
            isManualMoving = false;
            stopMotors();
        } else {
            isManualMoving = true;
            if (cmd == "F") moveDirection = -1.0;
            else if (cmd == "B") moveDirection = 1.0;
            else if (cmd == "L") moveDirection = -2.0;  
            else if (cmd == "R") moveDirection = 2.0; 
        }
        r->send(200);
    });

    server.on("/move_cm", HTTP_GET, [](AsyncWebServerRequest *r){
        if (r->hasParam("cm")) {
            targetDistanceCm = r->getParam("cm")->value().toFloat();
            moveDirection = r->hasParam("dir") ? r->getParam("dir")->value().toFloat() : 1.0;
            shouldMoveCm = true;
            r->send(200);
        } else {
            r->send(400, "text/plain", "Missing cm parameter");
        }
    });

    server.on("/turn", HTTP_GET, [](AsyncWebServerRequest *r){
        targetDegrees = r->getParam("deg")->value().toFloat();
        shouldTurn = true;
        r->send(200);
    });

    server.on("/config", HTTP_GET, [](AsyncWebServerRequest *r){
        if (r->hasParam("param") && r->hasParam("val")) {
            String param = r->getParam("param")->value();
            float val = r->getParam("val")->value().toFloat();
            if (param == "delay") speedDelayUs = (int)val;
            else if (param == "obs") obstacleThreshold = (int)val;
            sendLog("System: Updated " + param + " to " + String(val));
        }
        r->send(200);
    });

    server.on("/toggle", HTTP_GET, [](AsyncWebServerRequest *r){
        String type = r->getParam("type")->value();
        String state = "";
        if (type == "ir") { irEnabled = !irEnabled; state = String(irEnabled); } 
        else if (type == "us") { ultrasonicEnabled = !ultrasonicEnabled; state = String(ultrasonicEnabled); }
        sendLog("Sensor Bypass: " + type + " is now " + (state == "1" ? "ON" : "OFF"));
        r->send(200, "text/plain", state);
    });

    server.on("/stop", HTTP_GET, [](AsyncWebServerRequest *r){
        emergencyStop = true;
        isManualMoving = false;
        shouldMoveCm = false;
        shouldTurn = false;
        stopMotors();
        r->send(200);
    });

    server.addHandler(&events);
    server.begin();
}

void handleUDP() {
    int packetSize = udp.parsePacket();
    if (!packetSize) return;

    // Save Python sender address for responding
    pythonClientIP = udp.remoteIP();
    pythonClientPort = udp.remotePort();

    int len = udp.read(udpBuffer, sizeof(udpBuffer) - 1);
    if (len <= 0) return;
    
    udpBuffer[len] = 0;
    String msg = String(udpBuffer);
    msg.trim();

    // 1. Precise Turn Control with Real-Time Feedback Confirmation
    if (msg.startsWith("TURN:")) {
        float deg = msg.substring(5).toFloat();
        sendLog("UDP Command: Turning " + String(deg) + " deg");
        
        turnRobot(deg); // Blocking rotation
        
        if (emergencyStop) {
            sendUDPFeedback("ALERT:EMERGENCY_STOP");
        } else if (safetyHalt) {
            sendUDPFeedback("ALERT:SAFETY_HALT");
        } else {
            sendUDPFeedback("COMPLETED:TURN:" + String(deg, 1));
        }
    }
    // 2. Precise Centimeter Distance Move Control with Feedback
    else if (msg.startsWith("MOVE:")) {
        int firstColon = msg.indexOf(':');
        int secondColon = msg.indexOf(':', firstColon + 1);
        float cm = 0.0;
        float dir = 1.0;
        
        if (secondColon != -1) {
            cm = msg.substring(firstColon + 1, secondColon).toFloat();
            dir = msg.substring(secondColon + 1).toFloat();
        } else {
            cm = msg.substring(firstColon + 1).toFloat();
        }

        sendLog("UDP Command: Moving " + String(cm) + " cm");
        
        moveRobotCm(cm, dir); // Blocking movement
        
        if (emergencyStop) {
            sendUDPFeedback("ALERT:EMERGENCY_STOP");
        } else if (safetyHalt) {
            sendUDPFeedback("ALERT:SAFETY_HALT");
        } else {
            sendUDPFeedback("COMPLETED:MOVE:" + String(cm, 1));
        }
    }
    // 3. Sensor Toggles
    else if (msg.startsWith("TOGGLE:")) {
        String sensor = msg.substring(7);
        if (sensor == "ir") {
            irEnabled = !irEnabled;
            sendLog("UDP: IR Sensor toggled to " + String(irEnabled ? "ON" : "OFF"));
        } else if (sensor == "us") {
            ultrasonicEnabled = !ultrasonicEnabled;
            sendLog("UDP: Ultrasonic toggled to " + String(ultrasonicEnabled ? "ON" : "OFF"));
        }
        sendUDPFeedback("ACK:TOGGLE");
    }
    // 4. Configuration Settings
    else if (msg.startsWith("CFG:")) {
        int firstColon = msg.indexOf(':');
        int secondColon = msg.indexOf(':', firstColon + 1);
        if (secondColon != -1) {
            String param = msg.substring(firstColon + 1, secondColon);
            float val = msg.substring(secondColon + 1).toFloat();
            if (param == "delay") speedDelayUs = (int)val;
            else if (param == "obs") obstacleThreshold = (int)val;
            sendLog("UDP Config: Updated " + param + " to " + String(val));
        }
        sendUDPFeedback("ACK:CFG");
    }
    // 5. Emergency Stop
    else if (msg == "STOP") {
        emergencyStop = true;
        isManualMoving = false;
        shouldMoveCm = false;
        shouldTurn = false;
        stopMotors();
        sendUDPFeedback("ALERT:EMERGENCY_STOP");
    }
    // 6. Manual D-Pad Stepping Commands
    else {
        char cmd = msg.charAt(0);
        if (cmd == 'S') {
            isManualMoving = false;
            stopMotors();
        } else {
            isManualMoving = true;
            if (cmd == 'F') moveDirection = -1.0;
            else if (cmd == 'B') moveDirection = 1.0;
            else if (cmd == 'L') moveDirection = -2.0;
            else if (cmd == 'R') moveDirection = 2.0;
        }
    }
}

// Poll only for STOP while a blocking movement is stepping. Other commands
// remain handled by handleUDP() when the main loop is available.
void serviceEmergencyStop() {
    while (udp.parsePacket() > 0) {
        pythonClientIP = udp.remoteIP();
        pythonClientPort = udp.remotePort();

        int len = udp.read(udpBuffer, sizeof(udpBuffer) - 1);
        if (len <= 0) continue;

        udpBuffer[len] = 0;
        String msg = String(udpBuffer);
        msg.trim();
        if (msg == "STOP") {
            emergencyStop = true;
            isManualMoving = false;
            shouldMoveCm = false;
            shouldTurn = false;
            stopMotors();
            sendUDPFeedback("ALERT:EMERGENCY_STOP");
        }
    }
}