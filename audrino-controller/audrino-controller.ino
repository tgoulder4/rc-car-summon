/*
 * This ESP8266 NodeMCU code was developed by newbiely.com
 *
 * This ESP8266 NodeMCU code is made available for public use without any restriction
 *
 * For comprehensive instructions and wiring diagrams, please visit:
 * https://newbiely.com/tutorials/esp8266/esp8266-car
 */

#include <DIYables_IRcontroller.h>  // DIYables_IRcontroller library
#include <ESP8266WebServer.h>
#include <ESP8266WiFi.h>

const char* ssid = "TYE-LAPTOP 6635";
const char* password = "X3r3+172";
const int trigPin = 5;
const int echoPin = 16;

#define IR_RECEIVER_PIN D1  // The ESP8266 pin connected to IR receiver
#define IN1_PIN D2          // The ESP8266 pin connected to the IN1 pin L298N
#define IN2_PIN D5          // The ESP8266 pin connected to the IN2 pin L298N
#define IN3_PIN D6          // The ESP8266 pin connected to the IN3 pin L298N
#define IN4_PIN D7          // The ESP8266 pin connected to the IN4 pin L298N

#define BUTTON RST

DIYables_IRcontroller_17 irController(IR_RECEIVER_PIN, 200);  // debounce time is 200ms
ESP8266WebServer server(80);

void setup() {
  // CAR_moveForward();
  
  Serial.begin(9600);
  irController.begin();

  pinMode(IN1_PIN, OUTPUT);
  pinMode(IN2_PIN, OUTPUT);
  pinMode(IN3_PIN, OUTPUT);
  pinMode(IN4_PIN, OUTPUT);
    // Connect to Wi-Fi
  Serial.println();
  Serial.print("Connecting to ");
  Serial.println(ssid);

  WiFi.begin(ssid, password);

  // Wait for connection
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }

  Serial.println("");
  Serial.println("WiFi connected");
  Serial.print("IP address: ");
  Serial.println(WiFi.localIP());

 // Define HTTP endpoints
server.on("/forward", HTTP_GET, []() {
  Serial.println("Received /forward request");
  // stopMotors();
  moveForward();
  server.send(200, "text/plain", "Moving forward");
});

server.on("/backward", HTTP_GET, []() {
  Serial.println("Received /backward request");
  // stopMotors();
  moveBackward();
  server.send(200, "text/plain", "Moving backward");
});

server.on("/left", HTTP_GET, []() {
  Serial.println("Received /left request");
  // stopMotors();
  turnLeft();
  server.send(200, "text/plain", "Turning left");
});

server.on("/right", HTTP_GET, []() {
  Serial.println("Received /right request");
  // stopMotors();
  turnRight();
  server.send(200, "text/plain", "Turning right");
});

server.on("/stop", HTTP_GET, []() {
  Serial.println("Received /stop request");
  stopMotors();
  server.send(200, "text/plain", "Stopping");
});

server.on("/summon", HTTP_GET, []() {
  Serial.println("Received /summon request");
  // Implement summon functionality here
  server.send(200, "text/plain", "Summoning");
});
  server.begin();
  Serial.println("HTTP server started");
}

void loop() {
    if (WiFi.status() != WL_CONNECTED) {
    Serial.println("Wi-Fi disconnected");
    // Attempt to reconnect or handle accordingly
  }
    server.handleClient();
}
// Motor control functions
void moveForward() {
  digitalWrite(IN1_PIN, LOW);
  digitalWrite(IN2_PIN, HIGH);
  digitalWrite(IN3_PIN, LOW);
  digitalWrite(IN4_PIN, HIGH);
}

void moveBackward() {

    digitalWrite(IN1_PIN, HIGH);
  digitalWrite(IN2_PIN, LOW);
  digitalWrite(IN3_PIN, HIGH);
  digitalWrite(IN4_PIN, LOW);
}

void turnLeft() {
  digitalWrite(IN1_PIN, LOW);
  digitalWrite(IN2_PIN, LOW);
  digitalWrite(IN3_PIN, HIGH);
  digitalWrite(IN4_PIN, LOW);
}

void turnRight() {

    digitalWrite(IN1_PIN, HIGH);
  digitalWrite(IN2_PIN, LOW);
  digitalWrite(IN3_PIN, LOW);
  digitalWrite(IN4_PIN, LOW);
}

void stopMotors() {
  digitalWrite(IN1_PIN, LOW);
  digitalWrite(IN2_PIN, LOW);
  digitalWrite(IN3_PIN, LOW);
  digitalWrite(IN4_PIN, LOW);
}
