#include <Servo.h>
#include <AccelStepper.h>
#include <math.h>
#include <TinyGPSPlus.h>

// ---------------------------------------------------
//              PIN DEFINITIONS                       
// ---------------------------------------------------

// ------------ ULTRASONIC SENSOR
#define trigPin 52
#define echoPin 53

// ------------ DC MOTORS
// FR Motor connections
const uint8_t enFR = 9;
const uint8_t in1 = 22;
const uint8_t in2 = 23;

// FL Motor connections
const uint8_t enFL = 10;
const uint8_t in3 = 24;
const uint8_t in4 = 25;

// REAR Motor Connections
const uint8_t enREAR = 11;
const uint8_t REAR_in1 = 26;
const uint8_t REAR_in2 = 27;

// ------------ REAR SERVO
const uint8_t ServoSignal = 12;

// ------------ STEPPER MOTORS
#define HALFSTEP 8
AccelStepper furrowStepper(HALFSTEP, 30, 32, 31, 33);
AccelStepper seedingStepper(HALFSTEP, 34, 36, 35, 37);


// ---------------------------------------------------
//              VARIABLE DEFINITIONS                       
// ---------------------------------------------------
const float WHEELBASE = 39.6;   // Distance between front and rear axles in cm
const float TRACK_WIDTH = 21.6; // Distance between front and rear axles in cm
int distance;
long duration;
int currentSpeed = 0;
bool isMovingForward = false;         
bool isDown = false;                            // Flag for furrow up or down
float furrowDepth = 0.0;                        // Gives us the depth of the furrow
const int MAX_DEPTH = 5;                        // Max depth of the furrow
const int STEPS_PER_REV = 2048;                 // for 28BYJ-48 in half-step mode
const int STEPS_30_DEG = STEPS_PER_REV / 12;    // ~170 steps for 30 degrees
const int DEFAULT_FURROW_ROT = 5;               // Default number of rotations to lower furrow to 3cm below ground 
bool planting = false;                          // Flag for planting mode or not 
unsigned long lastSeedDrop = 0;                 // Timer between seed drops
String lastMotionCommand = "";                  // Store the last valid motion command
bool stoppedByObstacle = false;                 // Flag to track if the stop was due to an obstacle
float lat = 6.5244;                             // last GPS lat and lon
float lng = 7.4951;
unsigned long lastUpdate = 0;
// ---------------------------------------------------
//                   INSTANCES                      
// ---------------------------------------------------

Servo rearServo;                  // SERVO OBJECT
TinyGPSPlus gps;                  // GPS OBJECT


// ---------------------------------------------------
//              FUNCTION PROTOTYPES                      
// ---------------------------------------------------
void raiseFurrow(int numRot = DEFAULT_FURROW_ROT);
void lowerFurrow(int numRot = DEFAULT_FURROW_ROT);
void setupMotors();
void setupDCMotors();
void setupStepperServo();
void setupSensors();
void processCommand(String command);
int radiusToServoAngle(float radius, char dir);
void turn(char dir, int angle, float radius);
void rotateWheel(int angle);
void driveForward(float speed);
void driveBackward(float speed);
void stopMotors();
long readUltrasonic();


// ---------------------------------------------------
//                   SETUP                      
// ---------------------------------------------------
void setup() {
  Serial.begin(9600);
  Serial1.begin(9600);                // GPS Serial

  // initialise the wifi connection
  initWiFi();

  // Setup robot functions
  setupMotors();
  setupStepperServo();

  // ------ SERVO ------ //
  rearServo.attach(ServoSignal);        // Servo signal wire → pin 12
  rearServo.write(90);        // Start centered
}

// -------------- SETUP FUNCTIONS -------------- //


void setupMotors() {
  setupDCMotors();
  setupStepperServo();
}

void setupDCMotors() {
  // Set all the motor control pins to outputs
  pinMode(enFR, OUTPUT);
  pinMode(enFL, OUTPUT);
  pinMode(enREAR, OUTPUT);
  pinMode(in1, OUTPUT);
  pinMode(in2, OUTPUT);
  pinMode(in3, OUTPUT);
  pinMode(in4, OUTPUT);
  pinMode(REAR_in1, OUTPUT);
  pinMode(REAR_in2, OUTPUT);

  // Turn off motors - Initial state
  digitalWrite(in1, LOW);
  digitalWrite(in2, LOW);
  digitalWrite(in3, LOW);
  digitalWrite(in4, LOW);
  digitalWrite(REAR_in1, LOW);
  digitalWrite(REAR_in2, LOW);
}

void setupStepperServo() {
  // Function to set up direction servo and two stepper motors

  // SERVO
  rearServo.attach(ServoSignal);        // Servo signal wire → pin 12
  rearServo.write(90);                  // Start centered

  // STEPPERS
  // Setup speed and acceleration for furrow and seeding stepper motors
  furrowStepper.setMaxSpeed(1000.0);
  furrowStepper.setAcceleration(50.0);

  seedingStepper.setMaxSpeed(1000.0);
  seedingStepper.setAcceleration(50.0);

  // Upon initialisation load the seeds to be ready for dropping
  loadSeeds();
}

void setupSensors() {
  // Set up ultrasonic sensor
  pinMode(trigPin, OUTPUT);
  pinMode(echoPin, INPUT);
}

// ---------- COMMAND PROCESSOR ----------- //
void processCommand(String command) {
  command.trim();                 // clean whitespace
  command.toUpperCase();          // Case insensitive

  if (command.startsWith("F")) {
    lastMotionCommand = command;                                // Save the direction and speed
    int speed = command.substring(1).toFloat();
    driveForward(speed);
  } 
  else if (command.startsWith("B")) {
    lastMotionCommand = command;                                // Save the direction and speed
    int speed = command.substring(1).toFloat();
    driveBackward(speed);
  }
  else if (command.startsWith("S")) {
    lastMotionCommand = "";                                     // Clear motion profile
    stopMotors();
  }
  else if (command.startsWith("L") || command.startsWith("R")) {
    lastMotionCommand = command;                                    // Save information on turning in case of obstacle
    char direction = command[0];                                    // Get direction from command
    float turningRadius = command.substring(1).toFloat();           // [cm]
    int angle = radiusToServoAngle(turningRadius, direction);       // Convert turning radius to angle 
    stopMotors();                                                   // Pause Motion
    
    turn(direction, angle, turningRadius);                           // Apply Ackermann Steering
  }
  else if (command.startsWith("U")) {
    // If the furrow is already down then bring back up
    if (isDown) {
      raiseFurrow();
    }
    else {
      Serial.println("Furrow is already up");
    }
  }
  else if (command.startsWith("D")) {
    if (isDown) {
      Serial.println("Furrow is already down");
    }
    else {
      lowerFurrow();
    }
  }
  else if (command == "GO") {
    // GO for planting seeds
    planting = true;
    Serial.println("Seeding mode activated");
  }
  else {
    Serial.println("Invalid command.");
  }
}

// -------------- LOCOMOTION -------------- //
int radiusToServoAngle(float radius, char dir) {
  // Handle special cases
  if (radius <= 0) {
    Serial.println("Error: Radius must be positive");
    return 90;                                            // Return center position for invalid input
  }
  
  if (radius > 1000) {                                    // Very large radius = straight line
    return 90;
  }

  Serial.print("Direction "); Serial.println(dir);

  // Calculate steering angle with Ackermann steering
  float steeringAngleRad = atan(WHEELBASE / radius);
  float steeringAngleDeg = steeringAngleRad * 180.0 / PI; // Convert to degrees

  // Convert to servo position
  int servoAngle = (dir == 'R') ? (90 - (int)steeringAngleDeg) : (90 + (int)steeringAngleDeg);

  
  // Constrain to servo limits
  servoAngle = constrain(servoAngle, 0, 180);

  // Debug output
  Serial.print("Radius: "); Serial.print(radius);
  Serial.print(" cm -> Steering angle: "); Serial.print(steeringAngleDeg);
  Serial.print("° -> Servo angle: "); Serial.println(servoAngle);
  
  return servoAngle;
}


void turn(char dir, int angle, float radius) {
  int vR = 100;                          // Speed of rear wheel (PWM)
  int vFR = 0;                           // Initialise speed of front right wheel
  int vFL = 0;                           // Initialise speed of front right wheel
  // Turn the back wheel 
  rotateWheel(angle);

  // Compute Ackerman steering velocities
  if (dir == 'R') {
    vFR = round(vR * ((radius - TRACK_WIDTH / 2) / radius));
    vFL = round(vR * ((radius + TRACK_WIDTH / 2) / radius));
  }
  else {
    vFR = round(vR * ((radius + TRACK_WIDTH / 2) / radius));
    vFL = round(vR * ((radius - TRACK_WIDTH / 2) / radius));
  }

  // Constrain to valid PWM values
  vFR = constrain(vFR, 0, 255);
  vFL = constrain(vFL, 0, 255);

  Serial.print("Rear servo angle: "); Serial.print(angle); Serial.print(" | ");
  Serial.print("Right Wheel Speed: "); Serial.print(vFR); Serial.print(" | ");
  Serial.print("Left Wheel Speed: "); Serial.println(vFL);
}

void rotateWheel (int angle) {
  rearServo.write(angle);
  Serial.print("Moved to: ");
  Serial.println(angle);
}

void driveForward(float speed) {
  // convert speed percent to PWM signal
  int PWM = int((speed/100) * 255);

  // Set speed
  analogWrite(enFR, PWM);
  analogWrite(enFL, PWM);
  analogWrite(enREAR, PWM);

  // Drive all three motors
  digitalWrite(in1, HIGH); digitalWrite(in2, LOW);
  digitalWrite(in3, LOW); digitalWrite(in4, HIGH); 
  digitalWrite(REAR_in1, HIGH); digitalWrite(REAR_in2, LOW);
  
  // Update state variables
  isMovingForward = true;
  currentSpeed = PWM;

  Serial.print("Driving forward at: "); Serial.println(PWM);
}

void driveBackward(float speed) {
  // convert speed percent to PWM signal
  int PWM = int((speed/100) * 255);

  // Set speed
  analogWrite(enFR, PWM);
  analogWrite(enFL, PWM);
  analogWrite(enREAR, PWM);

  // Drive all three motors
  digitalWrite(in1, LOW); digitalWrite(in2, HIGH);
  digitalWrite(in3, HIGH); digitalWrite(in4, LOW);
  digitalWrite(REAR_in1, LOW); digitalWrite(REAR_in2, HIGH);

  // Update state variables
  isMovingForward = false;
  currentSpeed = PWM;

  Serial.print("Driving backward at: "); Serial.println(PWM);
}

void stopMotors(){

  // Set PWM to 0
  analogWrite(enFR, 0);
  analogWrite(enFL, 0);
  analogWrite(enREAR, 0);

  // Update State Variables
  isMovingForward = false;
  currentSpeed = 0;
  

  Serial.print("Stopping DC Motor"); Serial.print(" | "); Serial.println("Planting Paused");
}

void resumeMotion() {
  // Check if there's a stored command to resume
  if (lastMotionCommand.length() > 0) {
    if (lastMotionCommand.startsWith("F")) {
      int speed = lastMotionCommand.substring(1).toFloat();
      driveForward(speed);
    } else if (lastMotionCommand.startsWith("B")) {
      int speed = lastMotionCommand.substring(1).toFloat();
      driveBackward(speed);
    } else if (lastMotionCommand.startsWith("L") || lastMotionCommand.startsWith("R")) {
      char direction = lastMotionCommand[0];                                    // Get direction from command
      float turningRadius = lastMotionCommand.substring(1).toFloat();           // [cm]
      int angle = radiusToServoAngle(turningRadius, direction);       // Convert turning radius to angle 
      stopMotors();                                                   // Pause Motion
      
      turn(direction, angle, turningRadius);                           // Apply Ackermann Steering
    }
  }
}

// ----------- FURROW ------------ //
void raiseFurrow(int numRot = DEFAULT_FURROW_ROT) {
  furrowStepper.moveTo(STEPS_PER_REV * numRot);              // Stepper moves anticlockwise
  furrowStepper.run();
  isDown = false;
  Serial.println("Furrow position reset");
}

void lowerFurrow(int numRot = DEFAULT_FURROW_ROT) {
  furrowStepper.moveTo(STEPS_PER_REV * -numRot);              // Stepper moves clockwise
  furrowStepper.run();
  isDown = true; 
  Serial.println("Furrow is now lowered");
}

// ----------- SEEDING ------------ //

void loadSeeds() {
  // Function to spin the stepper motor for the seeds so that it is ready to drop seeds periodically
  seedingStepper.moveTo(STEPS_30_DEG * 5);
  seedingStepper.run();
  lastSeedDrop = millis();
  Serial.println("Seeds ready to drop");
}

void dropSeeds() {
  // Rotate 30 degrees to drop one seed at a time
  seedingStepper.moveTo(seedingStepper.currentPosition() + STEPS_30_DEG);
  Serial.println("Seed dropped");
}

// ----------- SENSORS ------------ //

long readUltrasonic() {
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);
  duration = pulseIn(echoPin, HIGH);
  return duration * 0.034 / 2; // cm
}

// GPS 
void simulateGPSMovement() {
  if (millis() - lastUpdate > 5000) { // Update every 5 seconds
    // Simulate small random movements
    lat += (random(-10, 11) / 100000.0);
    lng += (random(-10, 11) / 100000.0);
    lastUpdate = millis();
    
    Serial.print("GPS Update: ");
    Serial.print(lat, 6);
    Serial.print(", ");
    Serial.println(lng, 6);
  }
}



void loop() {
  // CHECK OBSTACLES AHEAD
  distance = readUltrasonic();

  // if valid gps data, pass that to the web-app else simulate ut
  if (Serial1.available()) {
    if (gps.encode(Serial1.read()) && gps.location.isValid()) { // if we can read the serial data and it is valid
      if (millis() - lastUpdate > 5000) {
        lat = gps.location.lat();
        lng = gps.location.lng();
        lastUpdate = millis();
      }
    }
  }
  else {
    simulateGPSMovement();
  }
  

  if (distance < 20 && isMovingForward) {
    stoppedByObstacle = true;
    stopMotors();
    Serial.println("Obstacle detected! Motors stopped.");
  }
  else if (stoppedByObstacle && distance >= 20) {
    // Resume prior action 
    resumeMotion();
    stoppedByObstacle = false;                    // Reset the flag after resuming
  }

  if (planting && isMovingForward) {
    // if in planting mode and the time since last drop is more than 2s -> drop a seed
    if (millis() - lastSeedDrop >= 2000) {
      dropSeeds();
      lastSeedDrop = millis();
    }
  }

  handleWiFiCommands();
  
  // Check for serial input
  if (Serial.available()) {
    String input = Serial.readStringUntil('\n');
    processCommand(input);
  }

  delay(10);
}