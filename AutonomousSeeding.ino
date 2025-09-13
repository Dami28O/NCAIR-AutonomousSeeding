#include <Servo.h>
#include <math.h>



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


// ---------------------------------------------------
//              VARIABLE DEFINITIONS                       
// ---------------------------------------------------
int distance;
long duration;
bool isMovingForward = false;         
int currentSpeed = 0;
const float WHEELBASE = 39.6; // Distance between front and rear axles in cm
const float TRACK_WIDTH = 21.6; // Distance between front and rear axles in cm


// ---------------------------------------------------
//                   INSTANCES                      
// ---------------------------------------------------

Servo rearServo;  // SERVO OBJECT


void setup() {
  Serial.begin(9600);

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
    int speed = command.substring(1).toFloat();
    driveForward(speed);
  } 
  else if (command.startsWith("B")) {
    int speed = command.substring(1).toFloat();
    driveBackward(speed);
  }
  else if (command.startsWith("S")) {
    stopMotors();
  }
  else if (command.startsWith("L") || command.startsWith("R")) {
    char direction = command[0];                                  // Get direction from command
    float turningRadius = command.substring(1).toFloat();           // [cm]
    int angle = radiusToServoAngle(turningRadius, direction);     // Convert turning radius to angle 
    stopMotors();                                                 // Pause Motion
    
    turn(direction, angle, turningRadius);                           // Apply Ackermann Steering
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
  delay(2000);

  // Update State Variables
  isMovingForward = false;
  currentSpeed = 0;

  Serial.println("Stopping Motor");

  // Add logic for the stepper motors
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



void loop() {
  // CHECK OBSTACLES AHEAD
  if (isMovingForward) {
    distance = readUltrasonic();
    if (distance < 20) {
      stopMotors();
      Serial.println("Obstacle detected! Motors stopped.");
    }
  }

  // Check for serial input
  if (Serial.available()) {
    String input = Serial.readStringUntil('\n');
    processCommand(input);
  }

  delay(50);
}