#include <SPI.h>
#include <Ethernet.h>
#include <Wire.h>               // I²C library

// Ethernet setup
byte mac[] = { 0xDE, 0xAD, 0xBE, 0xEF, 0xFE, 0xED };
IPAddress ip(10, 0, 7, 80);

// Three servers:
//  - commandServer on port 65432 for receiving commands,
//  - statusServer on port 65433 for continuous status updates,
//  - responseServer on port 65434 for sending responses (errors, completion messages, etc.)
EthernetServer commandServer(65432);
EthernetServer statusServer(65433);
EthernetServer responseServer(65434);
EthernetClient commandClient;
EthernetClient statusClient;
EthernetClient responseClient;

// Valid commands stored for reference (you can update/expand this list)
const char* validCommands[] = {
  "STOP",
  "EMERGENCY_STOP",
  "HOME",
  "MOVE_UP",
  "MOVE_DOWN",
  "MOVE_LEFT",
  "MOVE_RIGHT",
  "CLEAR_ICE",
  "GET_STATUS",
  "MOVE_Z1_UP",
  "MOVE_Z1_DOWN",
  "MOVE_Z2_UP",
  "MOVE_Z2_DOWN",
  "MOVE_OFFSET_INCREASE",
  "MOVE_OFFSET_DECREASE"
};
const int numValidCommands = sizeof(validCommands) / sizeof(validCommands[0]);

// Motor and sensor pins
#define STEP_PIN_Z1 2
#define DIR_PIN_Z1 3
#define STEP_PIN_Z2 4
#define DIR_PIN_Z2 5
#define ENABLE_Z1 6
#define ENABLE_Z2 7

#define STEP_PIN_X 8
#define DIR_PIN_X 9
#define ENABLE_X 24

// Limit switch pins
#define LIMIT_UP_Z1 14
#define LIMIT_DOWN_Z1 15
#define LIMIT_UP_Z2 16
#define LIMIT_DOWN_Z2 17
#define LIMIT_LEFT 18
#define LIMIT_RIGHT 19
#define LIMIT_GROUND_Z1 20
#define LIMIT_GROUND_Z2 21
#define EMERGENCY_STOP_PIN 22

// AS5600 Encoder pins
//#define ENCODER_SCL 37
//#define ENCODER_SDA 38
#define AS5600_ADDR  0x36       // AS5600 I2C address

// System State Variables
String activeCommand = "";
bool emergencyStopActive = false;
bool isHomed = false; // Global home position flag
unsigned long motorStartTime = 0;
bool isXMotorRunning = false;
const float ballscrewPitch = 10;   // mm per revolution (adjust to your screw)
volatile float currentAngleDeg = 0.0;
float prevAngleDeg    = 0.0;
long  rotationCount   = 0;          // # of full turns since last home
volatile float encoderPosition = 0.0; // final linear offset (mm)

// Helper function to determine axis status based on limit switches
String getXAxisStatus() {
  bool movementDetected = isEncoderMoving();
  if (digitalRead(LIMIT_LEFT) == LOW) {
    return "Left";
  } else if (digitalRead(LIMIT_RIGHT) == LOW) {
    return "Right";
  } else if (movementDetected) {
    return "Moving";
  } else {
    return "Stationary";
  }
}

String getZAxisStatus(int limitUpPin, int limitDownPin, int groundPin) {
  bool upPressed = (digitalRead(limitUpPin) == LOW);
  bool downPressed = (digitalRead(limitDownPin) == LOW);
  bool groundPressed = (digitalRead(groundPin) == LOW);
  
  // If more than one condition is active, report an error.
  if ((upPressed && (downPressed || groundPressed)) || (downPressed && groundPressed)) {
    return "error";
  } else if (upPressed) {
    return "up";
  } else if (groundPressed) {
    return "ground";
  } else if (downPressed) {
    return "down";
  } else {
    return "moving";
  }
}

// Encoder-based movement detection function
bool isEncoderMoving() {
  static int lastEncoderPos = encoderPosition;
  const int threshold = 1;  // Adjust based on your encoder's sensitivity
  bool moving = (abs(encoderPosition - lastEncoderPos) > threshold);
  lastEncoderPos = encoderPosition;
  return moving;
}

void setup() {
 Serial.begin(115200);
 Wire1.begin();  // initialize I²C on pins 38/37 for AS5600

while (!Serial && millis() < 3000);  // ✅ good
  Serial.println("Initializing Ethernet...");
  
  Ethernet.begin(mac, ip);
  commandServer.begin();
  statusServer.begin();
  responseServer.begin();
  
  Serial.println("Teensy Ethernet Servers Started.");

  // Set pin modes
  pinMode(STEP_PIN_Z1, OUTPUT);
  pinMode(DIR_PIN_Z1, OUTPUT);
  pinMode(STEP_PIN_Z2, OUTPUT);
  pinMode(DIR_PIN_Z2, OUTPUT);
  pinMode(ENABLE_Z1, OUTPUT);
  pinMode(ENABLE_Z2, OUTPUT);
  pinMode(STEP_PIN_X, OUTPUT);
  pinMode(DIR_PIN_X, OUTPUT);
  pinMode(ENABLE_X, OUTPUT);
  pinMode(LIMIT_UP_Z1, INPUT_PULLUP);
  pinMode(LIMIT_DOWN_Z1, INPUT_PULLUP);
  pinMode(LIMIT_UP_Z2, INPUT_PULLUP);
  pinMode(LIMIT_DOWN_Z2, INPUT_PULLUP);
  pinMode(LIMIT_GROUND_Z1, INPUT_PULLUP);
  pinMode(LIMIT_GROUND_Z2, INPUT_PULLUP);
  pinMode(LIMIT_LEFT, INPUT_PULLUP);
  pinMode(LIMIT_RIGHT, INPUT_PULLUP);
  pinMode(EMERGENCY_STOP_PIN, INPUT_PULLUP);  // Button active LOW
}

// Checks if the incoming command is valid.
bool isValidCommand(String command) {
  // Allow PRESET commands by default.
  if (command.startsWith("PRESET(")) {
    return true;
  }
  for (int i = 0; i < numValidCommands; i++) {
    String valid = String(validCommands[i]);
    if (command == valid || command.startsWith(valid + " ")) {
      return true;
    }
  }
  return false;
}

bool canExecuteCommand(String command) {
  if (!isValidCommand(command)) {
    Serial.println("ERROR: Command not recognized.");
    if (responseClient && responseClient.connected()) {
      responseClient.println("ERROR: Command not recognized.");
    }
    return false;
  }
  if (command == "GET_STATUS" || command == "STOP" || command == "EMERGENCY_STOP")
    return true;
  if (activeCommand != "") {
    Serial.println("ERROR: Command " + activeCommand + " is in progress.");
    if (responseClient && responseClient.connected()) {
      responseClient.println("ERROR: Command " + activeCommand + " is in progress.");
    }
    return false;
  }
  activeCommand = command;
  return true;
}

// Checks if the command string is a single digit (from 1 to 9)
bool isNumericCommand(String command) {
  if (command.length() == 1) {
    char c = command.charAt(0);
    return (c >= '1' && c <= '9');
  }
  return false;
}

// Process a numeric command (e.g., from a drop-down list).
void processNumberCommand(String command) {
  int number = command.toInt();
  Serial.print("Number ");
  Serial.print(number);
  Serial.println(" is received.");
  if (responseClient && responseClient.connected()) {
    responseClient.print("Number ");
    responseClient.print(number);
    responseClient.println(" is received.");
  }
}

// Poll the persistent command connection for an emergency stop command.
void pollCommandForEmergencyStop() {
  // First: check hardware emergency stop button on pin 22
  static bool buttonWasPressed = false;
  bool buttonIsPressed = digitalRead(22) == LOW;

  if (buttonIsPressed && !buttonWasPressed) {
    emergencyStopActive = true;
    Serial.println("Emergency stop triggered by hardware button.");
    if (responseClient && responseClient.connected()) {
      responseClient.println("LOG: Hardware Emergency Stop Activated.");
    }
  }

  buttonWasPressed = buttonIsPressed;

  // Then: check if command was received from Ethernet
  if (commandClient && commandClient.available() > 0) {
    String newCmd = commandClient.readStringUntil('\n');
    newCmd.trim();
    if (newCmd == "STOP" || newCmd == "EMERGENCY_STOP") {
      emergencyStopActive = true;
      Serial.println("Emergency stop command received during operation.");
      if (responseClient && responseClient.connected()) {
        responseClient.println("LOG: Emergency Stop Activated.");
      }
    }
  }
}


void processCommand(String command) {
  Serial.print("Received command: ");
  Serial.println(command);
  
  // Process numeric commands immediately.
  if (isNumericCommand(command)) {
    processNumberCommand(command);
    return;
  }
  
  if (command == "GET_STATUS") {
    sendStatusUpdate();
    return;
  }
  
  // Handle MOVE_LEFT with optional parameter (e.g., "MOVE_LEFT 50")
  if (command.startsWith("MOVE_LEFT")) {
    if (!canExecuteCommand(command)) return;
    Serial.println("LOG: Starting MOVE_LEFT...");
    if (responseClient && responseClient.connected()) {
      responseClient.println("LOG: Starting MOVE_LEFT...");
    }
    int steps = 20; // Default value
    int spaceIndex = command.indexOf(' ');
    if (spaceIndex > 0) {
      String stepStr = command.substring(spaceIndex + 1);
      steps = stepStr.toInt();
      if (steps <= 0) steps = 20;
    }
    runXAxis(steps, false);
    Serial.println("LOG: Done MOVE_LEFT");
    if (responseClient && responseClient.connected()) {
      responseClient.println("LOG: Done MOVE_LEFT");
    }
    activeCommand = "";
    return;
  }
  
  // Handle MOVE_RIGHT with optional parameter (e.g., "MOVE_RIGHT 50")
  if (command.startsWith("MOVE_RIGHT")) {
    if (!canExecuteCommand(command)) return;
    Serial.println("LOG: Starting MOVE_RIGHT...");
    if (responseClient && responseClient.connected()) {
      responseClient.println("LOG: Starting MOVE_RIGHT...");
    }
    int steps = 20; // Default value
    int spaceIndex = command.indexOf(' ');
    if (spaceIndex > 0) {
      String stepStr = command.substring(spaceIndex + 1);
      steps = stepStr.toInt();
      if (steps <= 0) steps = 20;
    }
    runXAxis(steps, true);
    Serial.println("LOG: Done MOVE_RIGHT");
    if (responseClient && responseClient.connected()) {
      responseClient.println("LOG: Done MOVE_RIGHT");
    }
    activeCommand = "";
    return;
  }
  
  // Process other commands.
  if (!canExecuteCommand(command)) return;
  
  if (command == "STOP" || command == "EMERGENCY_STOP") {
    emergencyStopActive = true;
    activeCommand = "";
    digitalWrite(ENABLE_Z1, HIGH);
    digitalWrite(ENABLE_Z2, HIGH);
    digitalWrite(ENABLE_X, HIGH);
    Serial.println("LOG: Emergency Stop Activated.");
    if (responseClient && responseClient.connected()) {
      responseClient.println("LOG: Emergency Stop Activated.");
    }
    delay(100);  // Short delay to process the stop
    emergencyStopActive = false; // Reset for future commands

  } else if (command == "HOME") {
    Serial.println("LOG: Starting HOMING...");
    if (responseClient && responseClient.connected()) {
      responseClient.println("LOG: Starting HOMING...");
    }
    startHoming();
    Serial.println("LOG: Done HOMING");
    if (responseClient && responseClient.connected()) {
      responseClient.println("LOG: Done HOMING");
    }
    activeCommand = "";
    
  } else if (command == "MOVE_UP") {  // Generic MOVE_UP: move both Z axes upward.
    Serial.println("LOG: Starting MOVE_UP...");
    if (responseClient && responseClient.connected()) {
      responseClient.println("LOG: Starting MOVE_UP...");
    }
    runZAxis(true, 1);  // Move Z1 upward.
    runZAxis(true, 2);  // Move Z2 upward.
    Serial.println("LOG: Done MOVE_UP");
    if (responseClient && responseClient.connected()) {
      responseClient.println("LOG: Done MOVE_UP");
    }
    activeCommand = "";
    return;
  } else if (command == "MOVE_DOWN") {  // Generic MOVE_DOWN: move both Z axes downward.
    Serial.println("LOG: Starting MOVE_DOWN...");
    if (responseClient && responseClient.connected()) {
      responseClient.println("LOG: Starting MOVE_DOWN...");
    }
    runZAxis(false, 1); // Move Z1 downward.
    runZAxis(false, 2); // Move Z2 downward.
    Serial.println("LOG: Done MOVE_DOWN");
    if (responseClient && responseClient.connected()) {
      responseClient.println("LOG: Done MOVE_DOWN");
    }
    activeCommand = "";
    return;
  } else if (command.startsWith("MOVE_Z1_UP")) {
  int spaceIndex = command.indexOf(' ');
  if (spaceIndex > 0) {
    String stepStr = command.substring(spaceIndex + 1);
    int steps = stepStr.toInt();
    if (steps <= 0) {
      Serial.println("LOG: Starting MOVE_Z1_UP (continuous)...");
      if (responseClient && responseClient.connected()) {
        responseClient.println("LOG: Starting MOVE_Z1_UP (continuous)...");
      }
      runZAxis(true, 1);
    } else {
      Serial.println("LOG: Starting MOVE_Z1_UP for " + String(steps) + " steps...");
      if (responseClient && responseClient.connected()) {
        responseClient.println("LOG: Starting MOVE_Z1_UP for " + String(steps) + " steps...");
      }
      runZAxisSteps(true, 1, steps);
    }
  } else {
    Serial.println("LOG: Starting MOVE_Z1_UP (continuous)...");
    if (responseClient && responseClient.connected()) {
      responseClient.println("LOG: Starting MOVE_Z1_UP (continuous)...");
    }
    runZAxis(true, 1);
  }
  Serial.println("LOG: Done MOVE_Z1_UP");
  if (responseClient && responseClient.connected()) {
    responseClient.println("LOG: Done MOVE_Z1_UP");
  }
  activeCommand = "";
  return;

  } else if (command.startsWith("MOVE_Z1_DOWN")) {
  int spaceIndex = command.indexOf(' ');
  if (spaceIndex > 0) {
    String stepStr = command.substring(spaceIndex + 1);
    int steps = stepStr.toInt();
    if (steps <= 0) {
      Serial.println("LOG: Starting MOVE_Z1_DOWN (continuous)...");
      if (responseClient && responseClient.connected()) {
        responseClient.println("LOG: Starting MOVE_Z1_DOWN (continuous)...");
      }
      runZAxis(false, 1);
    } else {
      Serial.println("LOG: Starting MOVE_Z1_DOWN for " + String(steps) + " steps...");
      if (responseClient && responseClient.connected()) {
        responseClient.println("LOG: Starting MOVE_Z1_DOWN for " + String(steps) + " steps...");
      }
      runZAxisSteps(false, 1, steps);
    }
  } else {
    Serial.println("LOG: Starting MOVE_Z1_DOWN (continuous)...");
    if (responseClient && responseClient.connected()) {
      responseClient.println("LOG: Starting MOVE_Z1_DOWN (continuous)...");
    }
    runZAxis(false, 1);
  }
  Serial.println("LOG: Done MOVE_Z1_DOWN");
  if (responseClient && responseClient.connected()) {
    responseClient.println("LOG: Done MOVE_Z1_DOWN");
  }
  activeCommand = "";
  return;

 } else if (command.startsWith("MOVE_Z2_UP")) {
  int spaceIndex = command.indexOf(' ');
  if (spaceIndex > 0) {
    String stepStr = command.substring(spaceIndex + 1);
    int steps = stepStr.toInt();
    if (steps <= 0) {
      Serial.println("LOG: Starting MOVE_Z2_UP (continuous)...");
      if (responseClient && responseClient.connected()) {
        responseClient.println("LOG: Starting MOVE_Z2_UP (continuous)...");
      }
      runZAxis(true, 2);
    } else {
      Serial.println("LOG: Starting MOVE_Z2_UP for " + String(steps) + " steps...");
      if (responseClient && responseClient.connected()) {
        responseClient.println("LOG: Starting MOVE_Z2_UP for " + String(steps) + " steps...");
      }
      runZAxisSteps(true, 2, steps);
    }
  } else {
    Serial.println("LOG: Starting MOVE_Z2_UP (continuous)...");
    if (responseClient && responseClient.connected()) {
      responseClient.println("LOG: Starting MOVE_Z2_UP (continuous)...");
    }
    runZAxis(true, 2);
  }
  Serial.println("LOG: Done MOVE_Z2_UP");
  if (responseClient && responseClient.connected()) {
    responseClient.println("LOG: Done MOVE_Z2_UP");
  }
  activeCommand = "";
  return;

  } else if (command.startsWith("MOVE_Z2_DOWN")) {
  int spaceIndex = command.indexOf(' ');
  if (spaceIndex > 0) {
    String stepStr = command.substring(spaceIndex + 1);
    int steps = stepStr.toInt();
    if (steps <= 0) {
      Serial.println("LOG: Starting MOVE_Z2_DOWN (continuous)...");
      if (responseClient && responseClient.connected()) {
        responseClient.println("LOG: Starting MOVE_Z2_DOWN (continuous)...");
      }
      runZAxis(false, 2);
    } else {
      Serial.println("LOG: Starting MOVE_Z2_DOWN for " + String(steps) + " steps...");
      if (responseClient && responseClient.connected()) {
        responseClient.println("LOG: Starting MOVE_Z2_DOWN for " + String(steps) + " steps...");
      }
      runZAxisSteps(false, 2, steps);
    }
  } else {
    Serial.println("LOG: Starting MOVE_Z2_DOWN (continuous)...");
    if (responseClient && responseClient.connected()) {
      responseClient.println("LOG: Starting MOVE_Z2_DOWN (continuous)...");
    }
    runZAxis(false, 2);
  }
  Serial.println("LOG: Done MOVE_Z2_DOWN");
  if (responseClient && responseClient.connected()) {
    responseClient.println("LOG: Done MOVE_Z2_DOWN");
  }
  activeCommand = "";
  return;

  } else if (command == "CLEAR_ICE") {
    Serial.println("LOG: Starting CLEAR_ICE...");
    if (responseClient && responseClient.connected()) {
      responseClient.println("LOG: Starting CLEAR_ICE...");
    }
    executeClearIceCommand();
    Serial.println("LOG: Done CLEAR_ICE");
    if (responseClient && responseClient.connected()) {
      responseClient.println("LOG: Done CLEAR_ICE");
    }
    activeCommand = "";
    
  } else if (command.startsWith("PRESET")) { // New PRESET command branch
    int startIdx = command.indexOf('(');
    int endIdx = command.indexOf(')', startIdx);
    if (startIdx == -1 || endIdx == -1 || endIdx <= startIdx + 1) {
      Serial.println("ERROR: Invalid PRESET command format.");
      if (responseClient && responseClient.connected()) {
        responseClient.println("ERROR: Invalid PRESET command format.");
      }
      activeCommand = "";
      return;
    }
    String offsetStr = command.substring(startIdx + 1, endIdx);
    int targetOffset = offsetStr.toInt();
    int delta = targetOffset - encoderPosition;
    if (delta == 0) {
      Serial.println("LOG: Already at target offset.");
      if (responseClient && responseClient.connected()) {
        responseClient.println("LOG: Already at target offset.");
      }
      activeCommand = "";
      return;
    }
    Serial.println("LOG: Starting PRESET command: Lifting Z2...");
    if (responseClient && responseClient.connected()) {
      responseClient.println("LOG: Starting PRESET command: Lifting Z2...");
    }
    runZAxis(true, 2); // Lift Z2 upward.
    if (delta > 0) {
      Serial.print("LOG: Moving LEFT ");
      Serial.print(delta);
      Serial.println(" steps.");
      if (responseClient && responseClient.connected()) {
        responseClient.print("LOG: Moving LEFT ");
        responseClient.print(delta);
        responseClient.println(" steps.");
      }
      runXAxis(delta, false); // Move left.
    } else {
      Serial.print("LOG: Moving RIGHT ");
      Serial.print(-delta);
      Serial.println(" steps.");
      if (responseClient && responseClient.connected()) {
        responseClient.print("LOG: Moving RIGHT ");
        responseClient.print(-delta);
        responseClient.println(" steps.");
      }
      runXAxis(-delta, true); // Move right.
    }
    Serial.println("LOG: Lowering Z2 to complete PRESET command.");
    if (responseClient && responseClient.connected()) {
      responseClient.println("LOG: Lowering Z2 to complete PRESET command.");
    }
    runZAxis(false, 2); // Lower Z2.
    activeCommand = "";
    Serial.print("LOG: PRESET command done.");
    if (responseClient && responseClient.connected()) {
      responseClient.print("LOG: PRESET command done.");
      }
    
  }
}

void readEncoder() {
  // 1) read raw angle from AS5600
  Wire1.beginTransmission(AS5600_ADDR);
  Wire1.write(0x0E);
  if (Wire1.endTransmission() != 0) return;
  Wire1.requestFrom(AS5600_ADDR, (uint8_t)2);
  if (Wire1.available() < 2) return;
  uint16_t raw = (((uint16_t)Wire1.read() << 8) | Wire1.read()) & 0x0FFF;
  currentAngleDeg = raw * 360.0f / 4096.0f;

  // 2) turn‐count detection (handle 359→0 or 0→359 wrap)
  float delta = currentAngleDeg - prevAngleDeg;
  if (delta > 180.0f)     rotationCount--;
  else if (delta < -180.0f) rotationCount++;
  prevAngleDeg = currentAngleDeg;

  // 3) build absolute angle and convert to linear pos
  float fullAngle = rotationCount * 360.0f + currentAngleDeg;
  encoderPosition = fullAngle / 360.0f * ballscrewPitch;
}

void resetEncoder() {
  rotationCount = 0;
  prevAngleDeg  = currentAngleDeg;  // so next delta is from this zero point
  encoderPosition = 0.0;
}

/*void detectErrors() {
  if (isXMotorRunning && millis() - motorStartTime > 3000) {
    int currentEncoderPosition = encoderPosition;
    // If the encoder hasn't changed significantly, flag an error.
    if (currentEncoderPosition == encoderPosition) {
      Serial.println("ERROR: X-axis motor frozen.");
      if (responseClient && responseClient.connected()) {
        responseClient.println("ERROR: X-axis motor frozen.");
      }
      isXMotorRunning = false;
    }
  }
}
*/
void sendStatusUpdate() {
  if (statusClient && statusClient.connected()) {
    String status = "{";
    status += "\"X_axis\":\"" + getXAxisStatus() + "\",";
    status += "\"offset\":\"" + String(encoderPosition) + "\",";
    status += "\"home_position\":\"" + String(isHomed ? "Yes" : "No") + "\",";
    String z1Status = getZAxisStatus(LIMIT_UP_Z1, LIMIT_DOWN_Z1, LIMIT_GROUND_Z1);
    String z2Status = getZAxisStatus(LIMIT_UP_Z2, LIMIT_DOWN_Z2, LIMIT_GROUND_Z2);
    status += "\"Z1_axis\":\"" + z1Status + "\",";
    status += "\"Z2_axis\":\"" + z2Status + "\"}";
    statusClient.println(status);
  }
}


void runZAxis(bool direction, int motor) {
  int stepPin = (motor == 1) ? STEP_PIN_Z1 : STEP_PIN_Z2;
  int dirPin = (motor == 1) ? DIR_PIN_Z1 : DIR_PIN_Z2;
  int enablePin = (motor == 1) ? ENABLE_Z1 : ENABLE_Z2;
  int limitUp = (motor == 1) ? LIMIT_UP_Z1 : LIMIT_UP_Z2;
  int limitDown = (motor == 1) ? LIMIT_DOWN_Z1 : LIMIT_DOWN_Z2;
  int groundLimit = (motor == 1) ? LIMIT_GROUND_Z1 : LIMIT_GROUND_Z2;
  
  digitalWrite(dirPin, direction);
  digitalWrite(enablePin, LOW);
  
  if (direction) { // Upward movement: check only the up limit
    while (digitalRead(limitUp) != LOW) {
      pollCommandForEmergencyStop();
      if (emergencyStopActive) break;
      digitalWrite(stepPin, HIGH);
      delayMicroseconds(800);
      digitalWrite(stepPin, LOW);
      delayMicroseconds(800);
    }
  } else { // Downward movement: check both the down limit and ground limit
    while ((digitalRead(limitDown) != LOW) && (digitalRead(groundLimit) != LOW)) {
      pollCommandForEmergencyStop();
      if (emergencyStopActive) break;
      digitalWrite(stepPin, HIGH);
      delayMicroseconds(800);
      digitalWrite(stepPin, LOW);
      delayMicroseconds(800);
    }
  }
  digitalWrite(enablePin, HIGH);
}

// New helper function: move a given number of steps for a Z-axis motor
void runZAxisSteps(bool direction, int motor, int steps) {
  int stepPin = (motor == 1) ? STEP_PIN_Z1 : STEP_PIN_Z2;
  int dirPin = (motor == 1) ? DIR_PIN_Z1 : DIR_PIN_Z2;
  int enablePin = (motor == 1) ? ENABLE_Z1 : ENABLE_Z2;
  int limitUp = (motor == 1) ? LIMIT_UP_Z1 : LIMIT_UP_Z2;
  int limitDown = (motor == 1) ? LIMIT_DOWN_Z1 : LIMIT_DOWN_Z2;
  int groundLimit = (motor == 1) ? LIMIT_GROUND_Z1 : LIMIT_GROUND_Z2;
  
  digitalWrite(dirPin, direction);
  digitalWrite(enablePin, LOW);
  
  for (int i = 0; i < steps; i++) {
    if (direction) { // Upward movement: check up limit
      if (digitalRead(limitUp) == LOW) {
        Serial.println("Up limit reached. Stopping movement.");
        if (responseClient && responseClient.connected())
          responseClient.println("LOG: Up limit reached. Stopping movement.");
        break;
      }
    } else { // Downward movement: check both down and ground limits
      if (digitalRead(limitDown) == LOW || digitalRead(groundLimit) == LOW) {
        Serial.println("Down or ground limit reached. Stopping movement.");
        if (responseClient && responseClient.connected())
          responseClient.println("LOG: Down or ground limit reached. Stopping movement.");
        break;
      }
    }
    pollCommandForEmergencyStop();
    if (emergencyStopActive) break;
    
    digitalWrite(stepPin, HIGH);
    delayMicroseconds(800);
    digitalWrite(stepPin, LOW);
    delayMicroseconds(800);
  }
  
  digitalWrite(enablePin, HIGH);
}

void runXAxis(int steps, bool direction) {
  digitalWrite(DIR_PIN_X, direction);
  digitalWrite(ENABLE_X, LOW);
  isXMotorRunning = true;
  motorStartTime = millis();
  for (int i = 0; i < steps; i++) {
    pollCommandForEmergencyStop();
    if (emergencyStopActive) break;
    digitalWrite(STEP_PIN_X, HIGH);
    delayMicroseconds(100);
    digitalWrite(STEP_PIN_X, LOW);
    delayMicroseconds(100);
  }
  digitalWrite(ENABLE_X, HIGH);
}

void startHoming() {
  while (digitalRead(LIMIT_LEFT) != LOW) {
    pollCommandForEmergencyStop();
    if (emergencyStopActive) return;
    
  }
  resetEncoder();

  while (digitalRead(LIMIT_DOWN_Z1) != LOW) {
    pollCommandForEmergencyStop();
    if (emergencyStopActive) return;
    runZAxis(false, 1);
  }
  
  while (digitalRead(LIMIT_DOWN_Z2) != LOW) {
    pollCommandForEmergencyStop();
    if (emergencyStopActive) return;
    runZAxis(false, 2);
  }
  runXAxis(1, false);
  
}

void executeClearIceCommand() {
  runXAxis(500, true);  // Move to the end
  delay(2000);
  runXAxis(500, false); // Return home
}

bool detectHomePosition() {
  bool xHome  = (digitalRead(LIMIT_RIGHT) == LOW);
  bool z1Home = (digitalRead(LIMIT_UP_Z1) == LOW);
  bool z2Home = (digitalRead(LIMIT_UP_Z2) == LOW);
  return (xHome && z1Home && z2Home);
}

void loop() {
  EthernetClient newCommandClient = commandServer.available();
  if (newCommandClient) {
    commandClient = newCommandClient;
    Serial.println("Command client connected.");
  }
  
  EthernetClient newStatusClient = statusServer.available();
  if (newStatusClient) {
    statusClient = newStatusClient;
  }
  
  if (!(responseClient && responseClient.connected())) {
    EthernetClient newResponseClient = responseServer.available();
    if (newResponseClient) {
      responseClient = newResponseClient;
      Serial.println("Response client connected.");
    }
  }
  
  if (commandClient && commandClient.connected()) {
    if (commandClient.available()) {
      String command = commandClient.readStringUntil('\n');
      command.trim();
      processCommand(command);
    }
  }
  
  if (statusClient && statusClient.connected()) {
    if (statusClient.available()) {
      String request = statusClient.readStringUntil('\n');
      request.trim();
      if (request == "GET_STATUS") {
        sendStatusUpdate();
      }
    }
  }
  
  isHomed = detectHomePosition();
  readEncoder();
  delay(50);
}
