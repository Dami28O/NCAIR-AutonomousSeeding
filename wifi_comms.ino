// Handles all wifi communications for robot control
#include <WiFiEspAT.h>


// WiFi credentials for Access Point
char ssid[] = "AGBE";     
char pass[] = "AGBE@Seeding"; 

WiFiServer server(80); // Standard HTTP port

// ---------- FUNCTION DECLARATIONS
void sendWiFiResponse(WiFiClient client, String message, String contentType = "text/html");
String processWiFiCommand(String request);
int extractSpeedParameter(String request, int defaultSpeed);
float extractRadiusParameter(String request, float defaultRadius);
float extractDepthParameter(String request, float defaultDepth);

// Initialize WiFi
void initWiFi() {
  Serial2.begin(115200);   // ESP8266 connection

  Serial.println("=== Arduino Robot Control ===");
  Serial.println("Starting WiFi setup...");

  WiFi.init(&Serial2);

  // Check ESP8266 connection
  if (WiFi.status() == WL_NO_MODULE) {
    Serial.println("ERROR: ESP8266 not found!");
    Serial.println("Check wiring:");
    Serial.println("  ESP8266 TX -> Arduino Pin 17 (RX2)");
    Serial.println("  ESP8266 RX -> Arduino Pin 16 (TX2)");
    Serial.println("  ESP8266 VCC -> 3.3V");
    Serial.println("  ESP8266 GND -> GND");
    while (true) {
      delay(1000);
    }
  }

  Serial.println("ESP8266 found!");
  Serial.print("Creating Access Point: ");
  Serial.println(ssid);

  // Start Access Point
  if (WiFi.beginAP(ssid, pass) != WL_AP_LISTENING) {
    Serial.println("Failed to create Access Point!");
    while (true) {
      delay(1000);
    }
  }

  delay(3000);  // Give it time to stabilize

  Serial.println("SUCCESS: Access Point created!");
  Serial.print("Network Name: ");
  Serial.println(ssid);
  Serial.print("Password: ");
  Serial.println(pass);
  Serial.println("Then go to: http://192.168.4.1");
  Serial.println("");
  Serial.println("=== Ready for commands ===");

  server.begin();

}
// Extract speed parameter from URL
int extractSpeedParameter(String request, int defaultSpeed) {
  int speedIndex = request.indexOf("speed=");
  if (speedIndex != -1) {
    int startPos = speedIndex + 6; // Length of "speed="
    int endPos = request.indexOf('&', startPos);
    if (endPos == -1) endPos = request.indexOf(' ', startPos);
    if (endPos == -1) endPos = request.length();
    
    String speedStr = request.substring(startPos, endPos);
    int speed = speedStr.toInt();
    
    // Constrain speed to reasonable limits
    if (speed > 0 && speed <= 100) {
      return speed;
    }
  }
  return defaultSpeed;
}

// Extract turning radius parameter from URL
float extractRadiusParameter(String request, float defaultRadius) {
  int radiusIndex = request.indexOf("radius=");
  if (radiusIndex != -1) {
    int startPos = radiusIndex + 7; // Length of "radius="
    int endPos = request.indexOf('&', startPos);
    if (endPos == -1) endPos = request.indexOf(' ', startPos);
    if (endPos == -1) endPos = request.length();
    
    String radiusStr = request.substring(startPos, endPos);
    float radius = radiusStr.toFloat();
    
    // Constrain radius to reasonable limits
    if (radius > 0 && radius <= 1000) {
      return radius;
    }
  }
  return defaultRadius;
}

// Extract turning depth parameter from URL
float extractDepthParameter(String request, float defaultDepth) {
  int depthIndex = request.indexOf("depth=");
  if (depthIndex != -1) {
    int startPos = depthIndex + 6; // Length of "radius="
    int endPos = request.indexOf('&', startPos);
    if (endPos == -1) endPos = request.indexOf(' ', startPos);
    if (endPos == -1) endPos = request.length();
    
    String depthStr = request.substring(startPos, endPos);
    float depth = depthStr.toFloat();
    
    // Constrain radius to reasonable limits
    if (depth > 0 && depth <= 5) {
      return depth;
    }
  }
  return defaultDepth;
}

// Handle incoming WiFi commands (non-blocking)
void handleWiFiCommands() {
  WiFiClient client = server.available();
  
  if (client) {
    String request = "";
    unsigned long timeout = millis();

    while (client.connected() && millis() - timeout < 2000) {
      if (client.available()) {
        char c = client.read();
        request += c;

        if (request.endsWith("\r\n\r\n")) {
          break; // full HTTP header received
        }
      }
    }

    // DEBUG
    // Serial.println("=== Request ===");
    // Serial.println(request);
    // Serial.println("===============");

    if (request.length() > 0) {
      String response = processWiFiCommand(request);
      if (response.startsWith("{\"lat\":")) {
        sendWiFiResponse(client, response, "application/json");
      }
      else {
        sendWiFiResponse(client, response);
      }
    }

    // client.stop();
  }
}

// Send HTTP response
void sendWiFiResponse(WiFiClient client, String message, String contentType = "text/html") {
  client.print("HTTP/1.1 200 OK\r\n");
  client.print("Content-Type: " + contentType + "\r\n");
  client.print("Access-Control-Allow-Origin: *\r\n");
  client.print("Connection: close\r\n");
  client.print("Content-Length: ");
  client.print(message.length());
  client.print("\r\n\r\n");
  client.print(message);

  client.flush();
  delay(10);  
  client.stop();
}

String processWiFiCommand(String request) {
  String command = "unknown";
  String response = "unknown action";

  if (request.indexOf("GET /forward") >= 0) {
    int speed = extractSpeedParameter(request, 50); // default 50%
    command = "F" + String(speed);
    processCommand(command);  // Call your existing function
    response = "Forward at speed " + String(speed);
  }
  else if (request.indexOf("GET /backward") >= 0) {
    int speed = extractSpeedParameter(request, 50);
    command = "B" + String(speed);
    processCommand(command);
    response = "Backward at speed " + String(speed);
  }
  else if (request.indexOf("GET /left") >= 0) {
    float radius = extractRadiusParameter(request, 37.5); // default 30cm
    command = "L" + String(radius);
    processCommand(command);
    response = "Left turn, radius " + String(radius) + "cm";
  }
  else if (request.indexOf("GET /right") >= 0) {
    float radius = extractRadiusParameter(request, 37.5);
    command = "R" + String(radius);
    processCommand(command);
    response = "Right turn, radius " + String(radius) + "cm";
  }
  else if (request.indexOf("GET /stop") >= 0) {
    command = "S";
    processCommand(command);
    response = "Motors stopped";
  }
  else if (request.indexOf("GET /furrow_down") >= 0) {
    float depth = extractDepthParameter(request, 3.0);
    command = "D" + String(depth);
    processCommand(command);
    response = "Furrow Down at " + String(depth) + "cm";
  }
  else if (request.indexOf("GET /furrow_up") >= 0) {
    command = "U";
    processCommand(command);
    response = "Furrow Position Reset (UP)";
  }
  else if (request.indexOf("GET /gps") >= 0) {
    String gpsData = String("{\"lat\":") + lat + ",\"lng\":" + lng + "}";
    return gpsData;
  }
  else {
    Serial.println("Unknown command in request");
    return "Unknown command";
  }
  
  Serial.print(">>> EXECUTING: ");
  Serial.println(command);
  
  return response;
}

