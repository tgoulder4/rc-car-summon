#include <ros.h>
#include <std_msgs/String.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Bool.h>

#define IN1_PIN 2
#define IN2_PIN 3
#define IN3_PIN 4
#define IN4_PIN 5
#define TRIG_PIN D5
#define ECHO_PIN D6

ros::NodeHandle nh;

float filterArray[20];
float distance_cm;
unsigned long lastSensorPublishTime = 0;
const int sensorPublishInterval = 200;
bool obstacleDetected = false;
const float OBSTACLE_THRESHOLD = 30.0;

void moveForward();
void moveBackward();
void turnLeft();
void turnRight();
void stopMotors();
float ultrasonicMeasure();
float getFilteredDistance();

std_msgs::Float32 distance_msg;
std_msgs::Bool obstacle_msg;
ros::Publisher distancePub("distance_sensor", &distance_msg);
ros::Publisher obstaclePub("obstacle_detected", &obstacle_msg);

void commandCallback(const std_msgs::String& cmd_msg) {
  String command = cmd_msg.data;
  
  Serial.print("Received command: ");
  Serial.println(command);
  
  if (command == "UP") {
    Serial.println("Processing UP command");
    moveForward();
  } 
  else if (command == "DOWN") {
    Serial.println("Processing DOWN command");
    moveBackward();
  } 
  else if (command == "LEFT") {
    Serial.println("Processing LEFT command");
    turnLeft();
  } 
  else if (command == "RIGHT") {
    Serial.println("Processing RIGHT command");
    turnRight();
  } 
  else if (command == "STOP") {
    Serial.println("Processing STOP command");
    stopMotors();
  }
  else {
    Serial.print("Unknown command: ");
    Serial.println(command);
  }
}

ros::Subscriber<std_msgs::String> sub("robot_commands", &commandCallback);

void setup() {
  Serial.begin(57600);
  Serial.println("Arduino ROS controller starting with ultrasonic sensor...");
  
  pinMode(IN1_PIN, OUTPUT);
  pinMode(IN2_PIN, OUTPUT);
  pinMode(IN3_PIN, OUTPUT);
  pinMode(IN4_PIN, OUTPUT);
  pinMode(TRIG_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);
  
  Serial.println("Initializing ROS node");
  nh.initNode();
  nh.subscribe(sub);
  nh.advertise(distancePub);
  nh.advertise(obstaclePub);
  
  Serial.println("ROS node initialized");
  Serial.println("Subscribed to robot_commands topic");
  Serial.println("Publishing to distance_sensor and obstacle_detected topics");
  
  stopMotors();
  
  Serial.println("Setup complete, entering main loop");
}

void loop() {
  unsigned long currentMillis = millis();
  
  if (currentMillis - lastSensorPublishTime >= sensorPublishInterval) {
    lastSensorPublishTime = currentMillis;
    
    distance_cm = getFilteredDistance();
    
    Serial.print("Distance: ");
    Serial.print(distance_cm);
    Serial.println(" cm");
    
    obstacleDetected = (distance_cm < OBSTACLE_THRESHOLD && distance_cm > 0);
    
    distance_msg.data = distance_cm;
    obstacle_msg.data = obstacleDetected;
    
    distancePub.publish(&distance_msg);
    obstaclePub.publish(&obstacle_msg);
    
    if (obstacleDetected) {
      Serial.println("OBSTACLE DETECTED! Stopping motors");
      stopMotors();
    }
  }
  
  nh.spinOnce();
  delay(10);
}

float getFilteredDistance() {
  Serial.println("Taking multiple measurements for filtering");
  
  for (int sample = 0; sample < 20; sample++) {
    filterArray[sample] = ultrasonicMeasure();
    delay(30);
  }
  
  Serial.println("Sorting measurements");
  for (int i = 0; i < 19; i++) {
    for (int j = i + 1; j < 20; j++) {
      if (filterArray[i] > filterArray[j]) {
        float swap = filterArray[i];
        filterArray[i] = filterArray[j];
        filterArray[j] = swap;
      }
    }
  }
  
  Serial.println("Calculating average of middle samples");
  float sum = 0;
  for (int sample = 5; sample < 15; sample++) {
    sum += filterArray[sample];
  }
  
  return sum / 10;
}

float ultrasonicMeasure() {
  digitalWrite(TRIG_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG_PIN, LOW);
  
  float duration_us = pulseIn(ECHO_PIN, HIGH);
  float distance_cm = 0.017 * duration_us;
  
  return distance_cm;
}

void moveForward() {
  Serial.println("Moving forward");
  digitalWrite(IN1_PIN, HIGH);
  digitalWrite(IN2_PIN, LOW);
  digitalWrite(IN3_PIN, HIGH);
  digitalWrite(IN4_PIN, LOW);
  Serial.println("Waiting 1000ms");
  delay(1000);
  Serial.println("Forward movement complete, stopping motors");
  stopMotors();
}

void moveBackward() {
  Serial.println("Moving backward");
  digitalWrite(IN1_PIN, LOW);
  digitalWrite(IN2_PIN, HIGH);
  digitalWrite(IN3_PIN, LOW);
  digitalWrite(IN4_PIN, HIGH);
  Serial.println("Waiting 1000ms");
  delay(1000);
  Serial.println("Backward movement complete, stopping motors");
  stopMotors();
}

void turnLeft() {
  Serial.println("Turning left");
  digitalWrite(IN1_PIN, LOW);
  digitalWrite(IN2_PIN, HIGH);
  digitalWrite(IN3_PIN, HIGH);
  digitalWrite(IN4_PIN, LOW);
  Serial.println("Waiting 500ms");
  delay(500);
  Serial.println("Left turn complete, stopping motors");
  stopMotors();
}

void turnRight() {
  Serial.println("Turning right");
  digitalWrite(IN1_PIN, HIGH);
  digitalWrite(IN2_PIN, LOW);
  digitalWrite(IN3_PIN, LOW);
  digitalWrite(IN4_PIN, HIGH);
  Serial.println("Waiting 500ms");
  delay(500);
  Serial.println("Right turn complete, stopping motors");
  stopMotors();
}

void stopMotors() {
  Serial.println("Stopping motors");
  digitalWrite(IN1_PIN, LOW);
  digitalWrite(IN2_PIN, LOW);
  digitalWrite(IN3_PIN, LOW);
  digitalWrite(IN4_PIN, LOW);
  Serial.println("Motors stopped");
}

