#include <SPI.h>
#include <Ethernet.h>
#include <Wire.h>               // I²C library
#include <math.h>

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
#define ENABLE_Z1 6

#define STEP_PIN_Z2 4
#define DIR_PIN_Z2 5
#define ENABLE_Z2 7

#define STEP_PIN_X 8
#define DIR_PIN_X 9
#define ENABLE_X 26 // not used

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
//#define AS5600_ADDR  0x36       // AS5600 I2C address
#define NANO_ADDR  0x40    // I²C address of Nano 33 BLE

// System State Variables
String activeCommand = "";
bool emergencyStopActive = false;
bool isHomed = false; // Global home position flag
unsigned long motorStartTime = 0;
bool isXMotorRunning = false;
const float ballscrewPitch = 10;   // mm per revolution (adjust to your screw)
float totalAngle = 0.0;            // absolute angle received from Nano
float angularVelocity = 0.0;            // angular velocity from Nano
const uint8_t SYNC = 0xFF;
volatile float offset = 0.0;       // final linear offset (mm)
float encoderZeroAngle = 0.0;  
const float STEPS_PER_MM = 100.0;

int speedToDelayUs(int speedMmPerS) {
  if (speedMmPerS <= 0) return 200;
  float delayUs = 1000000.0f / (speedMmPerS * STEPS_PER_MM);
  return (int)delayUs;
}

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
  static float lastEncoderPos = offset;
  const float threshold = 1.0f;  // Adjust based on your encoder's sensitivity
  bool moving = (fabsf(offset - lastEncoderPos) > threshold);
  lastEncoderPos = offset;
  return moving;
}

void setup() {
 Serial.begin(115200);
 Wire1.begin();  // initialize I²C on pins 38/37 for AS5600
 Serial1.begin(115200); // communication with Nano over Serial1 (RX on pin 24)
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


void readEncoder() {
  // Receive continuous angle and velocity from the Nano
  if (Serial1.available() && Serial1.peek() == SYNC) {
    Serial1.read();  // consume marker
    // wait for 8 bytes (two floats)
    while (Serial1.available() < int(sizeof(float) * 2)) {
    }
    Serial1.readBytes((char *)&totalAngle, sizeof(totalAngle));
    Serial1.readBytes((char *)&angularVelocity, sizeof(angularVelocity));

    // Convert the received angle to linear offset relative to home
    float relativeAngle = totalAngle - encoderZeroAngle;
    offset = relativeAngle / 360.0f * ballscrewPitch;
  }

  // If the right limit switch is pressed, reset the encoder reference
  if (digitalRead(LIMIT_RIGHT) == LOW) {
    encoderZeroAngle = totalAngle;  // record the absolute home angle
    offset = 0.0f;
  }
}
/*void detectErrors() {
  if (isXMotorRunning && millis() - motorStartTime > 3000) {
    int currentoffset = offset;
    // If the encoder hasn't changed significantly, flag an error.
    if (currentoffset == offset) {
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
    status += "\"offset\":\"" + String(offset) + "\",";
    status += "\"home_position\":\"" + String(isHomed ? "Yes" : "No") + "\",";
    String z1Status = getZAxisStatus(LIMIT_UP_Z1, LIMIT_DOWN_Z1, LIMIT_GROUND_Z1);
    String z2Status = getZAxisStatus(LIMIT_UP_Z2, LIMIT_DOWN_Z2, LIMIT_GROUND_Z2);
    status += "\"Z1_axis\":\"" + z1Status + "\",";
    status += "\"Z2_axis\":\"" + z2Status + "\"}";
    statusClient.println(status);
  }
}


void runZAxis(bool direction, int motor, int steps = -1, int stepDelayUs = 200) {
  int stepPin = (motor == 1) ? STEP_PIN_Z1 : STEP_PIN_Z2;
  int dirPin = (motor == 1) ? DIR_PIN_Z1 : DIR_PIN_Z2;
  int enablePin = (motor == 1) ? ENABLE_Z1 : ENABLE_Z2;
  int limitUp = (motor == 1) ? LIMIT_UP_Z1 : LIMIT_UP_Z2;
  int limitDown = (motor == 1) ? LIMIT_DOWN_Z1 : LIMIT_DOWN_Z2;
  int groundLimit = (motor == 1) ? LIMIT_GROUND_Z1 : LIMIT_GROUND_Z2;
  
  digitalWrite(dirPin, direction);
  digitalWrite(enablePin, LOW);
  
   int i = 0;
  while (true) {
    if (steps >= 0 && i >= steps) break;  // Completed requested steps

    bool limitReached = false;
    if (direction) {
      limitReached = (digitalRead(limitUp) == LOW);
    } else {
      limitReached = (digitalRead(limitDown) == LOW || digitalRead(groundLimit) == LOW);
    }
    if (limitReached) {
      if (responseClient && responseClient.connected()) {
        responseClient.println("LOG: Z-axis limit reached. Stopping movement.");
      }
         break;
    }
    pollCommandForEmergencyStop();
    if (emergencyStopActive) break;
    
    digitalWrite(stepPin, HIGH);
    delayMicroseconds(stepDelayUs);
    digitalWrite(stepPin, LOW);
    delayMicroseconds(stepDelayUs);
    
    if (steps >= 0) i++;  // Only count steps if steps parameter specified
  }
  
  digitalWrite(enablePin, HIGH);
}
// Run both Z motors simultaneously. When 'steps' is negative, move each motor
// until its respective limit is hit. Otherwise, move the specified number of
// steps while monitoring the limits.
void runDualZAxis(bool direction, int steps = -1, int stepDelayUs = 200) {
  digitalWrite(DIR_PIN_Z1, direction);
  digitalWrite(DIR_PIN_Z2, direction);
  digitalWrite(ENABLE_Z1, LOW);
  digitalWrite(ENABLE_Z2, LOW);

  int i = 0;
  while (true) {
    if (steps >= 0 && i >= steps) break;

    bool limit1 = direction ? (digitalRead(LIMIT_UP_Z1) == LOW)
                            : (digitalRead(LIMIT_DOWN_Z1) == LOW || digitalRead(LIMIT_GROUND_Z1) == LOW);
    bool limit2 = direction ? (digitalRead(LIMIT_UP_Z2) == LOW)
                            : (digitalRead(LIMIT_DOWN_Z2) == LOW || digitalRead(LIMIT_GROUND_Z2) == LOW);

    if (steps < 0 && limit1 && limit2) break;
    if (steps >= 0 && limit1 && limit2 && i >= steps) break;

    pollCommandForEmergencyStop();
    if (emergencyStopActive) break;

    if (!limit1) digitalWrite(STEP_PIN_Z1, HIGH);
    if (!limit2) digitalWrite(STEP_PIN_Z2, HIGH);
    delayMicroseconds(stepDelayUs);
    digitalWrite(STEP_PIN_Z1, LOW);
    digitalWrite(STEP_PIN_Z2, LOW);
    delayMicroseconds(stepDelayUs);

    if (steps >= 0) i++;
    if (steps < 0 && limit1 && limit2) break;
  }

  digitalWrite(ENABLE_Z1, HIGH);
  digitalWrite(ENABLE_Z2, HIGH);
}

void runXAxis(int steps, bool direction, int stepDelayUs = 200) {
  digitalWrite(DIR_PIN_X, direction);
  digitalWrite(ENABLE_X, LOW);
  isXMotorRunning = true;
  motorStartTime = millis();
  for (int i = 0; i < steps; i++) {
    if (direction) { // moving right
      if (digitalRead(LIMIT_RIGHT) == LOW) {
        Serial.println("Right limit reached. Stopping X movement.");
        if (responseClient && responseClient.connected())
          responseClient.println("LOG: Right limit reached. Stopping X movement.");
        break;
      }
    } else { // moving left
      if (digitalRead(LIMIT_LEFT) == LOW) {
        Serial.println("Left limit reached. Stopping X movement.");
        if (responseClient && responseClient.connected())
          responseClient.println("LOG: Left limit reached. Stopping X movement.");
        break;
      }
    }
    pollCommandForEmergencyStop();
    if (emergencyStopActive) break;
    delayMicroseconds(stepDelayUs);
    digitalWrite(STEP_PIN_X, LOW);
    delayMicroseconds(stepDelayUs);
  }
  digitalWrite(ENABLE_X, HIGH);
}

void startHoming() {
  // First raise both Z axes simultaneously to their upper limits
  runDualZAxis(true);

  // Then move X axis to the right (home) limit
  while (digitalRead(LIMIT_RIGHT) != LOW) {
    pollCommandForEmergencyStop();
    if (emergencyStopActive) return;
    runXAxis(1, true);
  }
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
  
  // Handle MOVE_LEFT with parameters like "MOVE_LEFT s1000 v50"
  if (command.startsWith("MOVE_LEFT")) {
    if (!canExecuteCommand(command)) return;
    Serial.println("LOG: Starting MOVE_LEFT...");
    if (responseClient && responseClient.connected()) {
      responseClient.println("LOG: Starting MOVE_LEFT...");
    }
    int steps = 20000;
    int speed = 50;
    String params = command.substring(String("MOVE_LEFT").length());
    params.trim();
    int idx = 0;
    while (idx < params.length()) {
      int space = params.indexOf(' ', idx);
      if (space == -1) space = params.length();
      String token = params.substring(idx, space);
      if (token.startsWith("s")) steps = token.substring(1).toInt();
      else if (token.startsWith("v")) speed = token.substring(1).toInt();
      idx = space + 1;
    }
    if (steps <= 0) steps = 200000;
    int delayUs = speedToDelayUs(speed);
    runXAxis(steps, false, delayUs);
    Serial.println("LOG: Done MOVE_LEFT");
    if (responseClient && responseClient.connected()) {
      responseClient.println("LOG: Done MOVE_LEFT");
    }
    activeCommand = "";
    return;
  }
  
  // Handle MOVE_RIGHT with parameters like "MOVE_RIGHT s1000 v50"
  if (command.startsWith("MOVE_RIGHT")) {
    if (!canExecuteCommand(command)) return;
    Serial.println("LOG: Starting MOVE_RIGHT...");
    if (responseClient && responseClient.connected()) {
      responseClient.println("LOG: Starting MOVE_RIGHT...");
    }
    int steps = 20000;
    int speed = 50;
    String params = command.substring(String("MOVE_RIGHT").length());
    params.trim();
    int idx = 0;
    while (idx < params.length()) {
      int space = params.indexOf(' ', idx);
      if (space == -1) space = params.length();
      String token = params.substring(idx, space);
      if (token.startsWith("s")) steps = token.substring(1).toInt();
      else if (token.startsWith("v")) speed = token.substring(1).toInt();
      idx = space + 1;
    }
    if (steps <= 0) steps = 200000;
    int delayUs = speedToDelayUs(speed);
    runXAxis(steps, true, delayUs);
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
    runDualZAxis(true);  // Move both Z axes upward simultaneously.
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
    runDualZAxis(false); // Move both Z axes downward simultaneously.
    Serial.println("LOG: Done MOVE_DOWN");
    if (responseClient && responseClient.connected()) {
      responseClient.println("LOG: Done MOVE_DOWN");
    }
    activeCommand = "";
    return;
  } else if (command.startsWith("MOVE_Z1_UP")) {
    int steps = -1;
  int speed = 50;
  String params = command.substring(String("MOVE_Z1_UP").length());
  params.trim();
  int idx = 0;
  while (idx < params.length()) {
    int space = params.indexOf(' ', idx);
    if (space == -1) space = params.length();
    String token = params.substring(idx, space);
    if (token.startsWith("s")) steps = token.substring(1).toInt();
    else if (token.startsWith("v")) speed = token.substring(1).toInt();
    idx = space + 1;
  }
  if (steps <= 0) {
    Serial.println("LOG: Starting MOVE_Z1_UP (continuous)...");
    if (responseClient && responseClient.connected()) {
      responseClient.println("LOG: Starting MOVE_Z1_UP (continuous)...");
    }
    runZAxis(true, 1, -1, speedToDelayUs(speed));
  } else {
    Serial.println("LOG: Starting MOVE_Z1_UP for " + String(steps) + " steps...");
    if (responseClient && responseClient.connected()) {
      responseClient.println("LOG: Starting MOVE_Z1_UP for " + String(steps) + " steps...");
    }
    runZAxis(true, 1, steps, speedToDelayUs(speed));
  }
  Serial.println("LOG: Done MOVE_Z1_UP");
  if (responseClient && responseClient.connected()) {
    responseClient.println("LOG: Done MOVE_Z1_UP");
  }
  activeCommand = "";
  return;

  } else if (command.startsWith("MOVE_Z1_DOWN")) {
  int firstSpace = command.indexOf(' ');
  int steps = -1;
  int speed = 50;
  String params = command.substring(String("MOVE_Z1_DOWN").length());
  params.trim();
  int idx = 0;
  while (idx < params.length()) {
    int space = params.indexOf(' ', idx);
    if (space == -1) space = params.length();
    String token = params.substring(idx, space);
    if (token.startsWith("s")) steps = token.substring(1).toInt();
    else if (token.startsWith("v")) speed = token.substring(1).toInt();
    idx = space + 1;
  }
  if (steps <= 0) {
    Serial.println("LOG: Starting MOVE_Z1_DOWN (continuous)...");
    if (responseClient && responseClient.connected()) {
      responseClient.println("LOG: Starting MOVE_Z1_DOWN (continuous)...");
    }
    runZAxis(false, 1, -1, speedToDelayUs(speed));
  } else {
    Serial.println("LOG: Starting MOVE_Z1_DOWN for " + String(steps) + " steps...");
    if (responseClient && responseClient.connected()) {
      responseClient.println("LOG: Starting MOVE_Z1_DOWN for " + String(steps) + " steps...");
    }
    runZAxis(false, 1, steps, speedToDelayUs(speed));
  }
  Serial.println("LOG: Done MOVE_Z1_DOWN");
  if (responseClient && responseClient.connected()) {
    responseClient.println("LOG: Done MOVE_Z1_DOWN");
  }
  activeCommand = "";
  return;

 } else if (command.startsWith("MOVE_Z2_UP")) {
   int steps = -1;
  int speed = 50;
  String params = command.substring(String("MOVE_Z2_UP").length());
  params.trim();
  int idx = 0;
  while (idx < params.length()) {
    int space = params.indexOf(' ', idx);
    if (space == -1) space = params.length();
    String token = params.substring(idx, space);
    if (token.startsWith("s")) steps = token.substring(1).toInt();
    else if (token.startsWith("v")) speed = token.substring(1).toInt();
    idx = space + 1;
  }
  if (steps <= 0) {

    Serial.println("LOG: Starting MOVE_Z2_UP (continuous)...");
    if (responseClient && responseClient.connected()) {
      responseClient.println("LOG: Starting MOVE_Z2_UP (continuous)...");
    }
    runZAxis(true, 2, -1, speedToDelayUs(speed));
  } else {
    Serial.println("LOG: Starting MOVE_Z2_UP for " + String(steps) + " steps...");
    if (responseClient && responseClient.connected()) {
      responseClient.println("LOG: Starting MOVE_Z2_UP for " + String(steps) + " steps...");
    }
    runZAxis(true, 2, steps, speedToDelayUs(speed));
  }
  Serial.println("LOG: Done MOVE_Z2_UP");
  if (responseClient && responseClient.connected()) {
    responseClient.println("LOG: Done MOVE_Z2_UP");
  }
  activeCommand = "";
  return;

 } else if (command.startsWith("MOVE_Z2_DOWN")) {
  int steps = -1;
  int speed = 50;
  String params = command.substring(String("MOVE_Z2_DOWN").length());
  params.trim();
  int idx = 0;
  while (idx < params.length()) {
    int space = params.indexOf(' ', idx);
    if (space == -1) space = params.length();
    String token = params.substring(idx, space);
    if (token.startsWith("s")) steps = token.substring(1).toInt();
    else if (token.startsWith("v")) speed = token.substring(1).toInt();
    idx = space + 1;
  }
  if (steps <= 0) {
    Serial.println("LOG: Starting MOVE_Z2_DOWN (continuous)...");
    if (responseClient && responseClient.connected()) {
      responseClient.println("LOG: Starting MOVE_Z2_DOWN (continuous)...");
    }
    runZAxis(false, 2, -1, speedToDelayUs(speed));
  } else {
    Serial.println("LOG: Starting MOVE_Z2_DOWN for " + String(steps) + " steps...");
    if (responseClient && responseClient.connected()) {
      responseClient.println("LOG: Starting MOVE_Z2_DOWN for " + String(steps) + " steps...");
    }
    runZAxis(false, 2, steps, speedToDelayUs(speed));
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
    int targetOffsetSteps = offsetStr.toInt();

    // Convert current encoder position (in mm) to steps for accurate delta
    float currentSteps = offset * STEPS_PER_MM;
    float deltaSteps = targetOffsetSteps - currentSteps;

    if (deltaSteps == 0) {
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
    int stepCount = (int)fabsf(deltaSteps);
    if (deltaSteps > 0) {
      Serial.print("LOG: Moving LEFT ");
      Serial.print(stepCount);
      Serial.println(" steps.");
      if (responseClient && responseClient.connected()) {
        responseClient.print("LOG: Moving LEFT ");
        responseClient.print(stepCount);
        responseClient.println(" steps.");
      }
      runXAxis(stepCount, false); // Move left.
    } else {
      Serial.print("LOG: Moving RIGHT ");
      Serial.print(stepCount);
      Serial.println(" steps.");
      if (responseClient && responseClient.connected()) {
        responseClient.print("LOG: Moving RIGHT ");
        responseClient.print(stepCount);
        responseClient.println(" steps.");
      }
      runXAxis(stepCount, true); // Move right.
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
