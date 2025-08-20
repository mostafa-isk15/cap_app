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
  "MOVE_RIGHT",//swap with move offset increase and decrease
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
#define ETH_CS 14 
// Motor and sensor pins
#define STEP_PIN_Z1 2
#define DIR_PIN_Z1 3
#define ENABLE_Z1 4

#define STEP_PIN_Z2 5
#define DIR_PIN_Z2 6
#define ENABLE_Z2 7

#define STEP_PIN_X 8
#define DIR_PIN_X 9
#define ENABLE_X 10 // not used

// Limit switch pins
//#define EMERGENCY_STOP_PIN 15
#define LIMIT_GROUND_Z1 16
#define LIMIT_GROUND_Z2 17
#define LIMIT_UP_Z1 18
#define LIMIT_DOWN_Z1 19
#define LIMIT_UP_Z2 20
#define LIMIT_DOWN_Z2 21
#define LIMIT_LEFT 22
#define LIMIT_RIGHT 23 


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
const float STEPS_PER_MM = 100.0; // change according to the dip switches on the driver (microstepping) and ball screw pitch
const unsigned long ENCODER_READ_TIMEOUT_MS = 100; // maximum wait for encoder data

void setup() {
 Serial.begin(115200);
 Wire1.begin();  // initialize I²C on pins 38/37 for AS5600
 Serial1.begin(9600); // communication with Nano over Serial1 (RX on pin 24)
while (!Serial && millis() < 3000);
  Serial.println("Initializing Ethernet...");
  Ethernet.init(ETH_CS);   // tell Ethernet library to use that pin
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
}

template <typename T>
void logPrintln(const T &msg) {
  Serial.println(msg);
  if (responseClient && responseClient.connected()) {
    responseClient.println(msg);
  }
} 

void pollCommandForEmergencyStop() {
  if (commandClient && commandClient.available()) {
    String newCmd = commandClient.readStringUntil('\n');
    newCmd.trim();
    if (newCmd == "STOP" || newCmd == "EMERGENCY_STOP") {
      emergencyStopActive = true;
      logPrintln("LOG: Emergency stop command received during operation.");
    }
  }
}


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

bool detectHomePosition() {
  bool xHome  = (digitalRead(LIMIT_RIGHT) == LOW);
  bool z1Home = (digitalRead(LIMIT_UP_Z1) == LOW);
  bool z2Home = (digitalRead(LIMIT_UP_Z2) == LOW);
  return (xHome && z1Home && z2Home);
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


void readEncoder() {
  // Receive continuous angle and velocity from the Nano
  if (Serial1.available() && Serial1.peek() == SYNC) {
    Serial1.read();  // consume marker
    // wait for 8 bytes (two floats) with timeout to avoid blocking
    unsigned long start = millis();
    while (Serial1.available() < int(sizeof(float) * 2)) {
      if (millis() - start > ENCODER_READ_TIMEOUT_MS) {
        return; // abort if data not received in time
      }
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

void detectErrors() {
  if (isXMotorRunning && millis() - motorStartTime > 3000) {
    int currentoffset = offset;
    // If the encoder hasn't changed significantly, flag an error.
    if (currentoffset == offset) {
    logPrintln("ERROR: X-axis motor frozen.");
      isXMotorRunning = false;
    }
  }
}

int speedToDelayUs(int speedMmPerS) {
  if (speedMmPerS <= 0) return 200;
  float delayUs = 1000000.0f / (speedMmPerS * STEPS_PER_MM);
  return (int)delayUs;
}


// Run Z-axis motors. `motor` = 0 for both, 1 for Z1, 2 for Z2. 
// When `steps` is negative, the motor(s) move until their respective limit switches are hit.
void runZAxis(bool direction, int motor, int steps = -1, int stepDelayUs = 200) {
  if (motor == 0) {
    // Run both Z motors simultaneously.
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

      pollCommandForEmergencyStop();
      if (emergencyStopActive) break;

      if (!limit1) digitalWrite(STEP_PIN_Z1, HIGH);
      if (!limit2) digitalWrite(STEP_PIN_Z2, HIGH);
      delayMicroseconds(stepDelayUs);
      digitalWrite(STEP_PIN_Z1, LOW);
      digitalWrite(STEP_PIN_Z2, LOW);
      delayMicroseconds(stepDelayUs);

      if (steps >= 0) i++;
    }

    digitalWrite(ENABLE_Z1, HIGH);
    digitalWrite(ENABLE_Z2, HIGH);
  } else {
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
      if (steps >= 0 && i >= steps) break;

      bool limitReached = direction ? (digitalRead(limitUp) == LOW)
                                    : (digitalRead(limitDown) == LOW || digitalRead(groundLimit) == LOW);
      if (limitReached) {
        logPrintln("LOG: Z-axis limit reached. Stopping movement.");
        break;
      }
      pollCommandForEmergencyStop();
      if (emergencyStopActive) break;

      digitalWrite(stepPin, HIGH);
      delayMicroseconds(stepDelayUs);
      digitalWrite(stepPin, LOW);
      delayMicroseconds(stepDelayUs);

      if (steps >= 0) i++;
    }

    digitalWrite(enablePin, HIGH);
  }

void runXAxis(int steps, bool direction, int stepDelayUs = 200) {
  
    if (/*digitalRead(LIMIT_GROUND_Z1) == LOW || */digitalRead(LIMIT_GROUND_Z2) == LOW) {
    logPrintln("cant move x axis the plates are on the ground");
    return;
  }

  digitalWrite(DIR_PIN_X, direction);
  digitalWrite(ENABLE_X, LOW);
  isXMotorRunning = true;
  motorStartTime = millis();
  for (int i = 0; i < steps; i++) {
    if (direction) { // moving right
      if (digitalRead(LIMIT_RIGHT) == LOW) {
    logPrintln("Right limit reached. Stopping X movement.");
        break;
      }
    } else { // moving left
      if (digitalRead(LIMIT_LEFT) == LOW) {
    logPrintln("Left limit reached. Stopping X movement.");
        break;
      }
    }
    pollCommandForEmergencyStop();
    if (emergencyStopActive) break;
    digitalWrite(STEP_PIN_X, HIGH);
    delayMicroseconds(stepDelayUs);
    digitalWrite(STEP_PIN_X, LOW);
    delayMicroseconds(stepDelayUs);
  }
  digitalWrite(ENABLE_X, HIGH);
}

void startHoming(int stepDelayUs = 200) {
  // First raise both Z axes simultaneously to their upper limits
  runZAxis(true, 0, -1, stepDelayUs);

  // Then move X axis to the right (home) limit
  while (digitalRead(LIMIT_RIGHT) != LOW) {
    pollCommandForEmergencyStop();
    if (emergencyStopActive) return;
    runXAxis(1, true, stepDelayUs);
  }
}
void executeClearIceCommand() {
  /*calibrate*/
  runXAxis(500, true);  // Move to the end
  delay(1000);
  runXAxis(500, false); // Return home
}

bool isValidCommand(String command) {
  // Allow PRESET commands by default.
  if (command.startsWith("PRESET")) {
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
    logPrintln("ERROR: Command not recognized.");
    return false;
  }
  if (command == "GET_STATUS" || command == "STOP" || command == "EMERGENCY_STOP")
    return true;
  if (activeCommand != "") {
    logPrintln("ERROR: Command " + activeCommand + " is in progress.");

    return false;
  }
  activeCommand = command;
  return true;
}

// Checks if the command string is a single digit (from 1 to 9)
/*calibrate*/ 
bool isNumericCommand(String command) {
  if (command.length() == 1) {
    char c = command.charAt(0);
    return (c >= '1' && c <= '9');
  }
  return false;
}

// Process a numeric command (e.g., from a drop-down list).
/*calibrate*/
void processNumberCommand(String command) {
  int number = command.toInt();
    logPrintln("Number " + number + " is recieved");
}

// Parse step and speed parameters from a command suffix like "s1000 v50"
void parseStepSpeed(String params, int &steps, int &speed) {
  size_t idx = 0;
  while (idx < params.length()) {
    int space = params.indexOf(' ', idx);
    if (space == -1) space = params.length();
    String token = params.substring(idx, space);
    if (token.startsWith("s")) {
      steps = token.substring(1).toInt();
    } else if (token.startsWith("v")) {
      speed = token.substring(1).toInt();
    }
    idx = space + 1;
  }
}

// Generic handler for MOVE_LEFT and MOVE_RIGHT commands
void handleMoveX(String command, bool moveRight) {
  if (!canExecuteCommand(command)) return;
  const char *name = moveRight ? "MOVE_RIGHT" : "MOVE_LEFT";
  logPrintln(String("LOG: Starting ") + name + "...");
  int steps = 20000;
  int speed = 50;
  String params = command.substring(String(name).length());
  params.trim();
  parseStepSpeed(params, steps, speed);
  if (steps <= 0) steps = 200000;
  int delayUs = speedToDelayUs(speed);
  runXAxis(steps, moveRight, delayUs);
  logPrintln(String("LOG: Done ") + name);
  activeCommand = "";
}

// Generic handler for MOVE_* commands affecting the Z axis
void handleMoveZ(String command, bool up, int axis, const char *name) {
  if (!canExecuteCommand(command)) return;
  int steps = -1;
  int speed = 50;
  String params = command.substring(String(name).length());
  params.trim();
  parseStepSpeed(params, steps, speed);
  int delayUs = speedToDelayUs(speed);
  if (steps <= 0) {
    logPrintln(String("LOG: Starting ") + name + " (continuous)...");
    runZAxis(up, axis, -1, delayUs);
  } else {
    logPrintln(String("LOG: Starting ") + name + " for " + String(steps) + " steps...");
    runZAxis(up, axis, steps, delayUs);
  }
  logPrintln(String("LOG: Done ") + name);
  activeCommand = "";
}

// Handler for HOME command
void handleHome(String command) {
  if (!canExecuteCommand(command)) return;
  int speed = 50;
  String params = command.substring(String("HOME").length());
  params.trim();
  if (params.startsWith("v")) speed = params.substring(1).toInt();
  logPrintln("LOG: Starting HOMING...");
  startHoming(speedToDelayUs(speed));
  logPrintln("LOG: Done HOMING...");
  activeCommand = "";
}

// Handler for PRESET command
void handlePreset(String command) {
  if (!canExecuteCommand(command)) return;
  int startIdx = command.indexOf('(');
  int endIdx = command.indexOf(')', startIdx);
  if (startIdx == -1 || endIdx == -1 || endIdx <= startIdx + 1) {
    logPrintln("ERROR: Invalid PRESET command format.");
    activeCommand = "";
    return;
  }
  String offsetStr = command.substring(startIdx + 1, endIdx);
  int targetOffsetSteps = offsetStr.toInt();

  int speed = 50;
  String params = command.substring(endIdx + 1);
  params.trim();
  if (params.startsWith("v")) speed = params.substring(1).toInt();

  int delayUs = speedToDelayUs(speed);
  float currentSteps = offset * STEPS_PER_MM;
  float deltaSteps = targetOffsetSteps - currentSteps;

  if (deltaSteps == 0) {
    logPrintln("LOG: Already at target offset.");
    activeCommand = "";
    return;
  }

  logPrintln("LOG: Starting PRESET command: Lifting Z2...");
  runZAxis(true, 2, -1, delayUs);
  int stepCount = (int)fabsf(deltaSteps);
  if (deltaSteps > 0) {
    logPrintln(String("LOG: Moving LEFT ") + stepCount + " steps.");
    runXAxis(stepCount, false, delayUs);
  } else {
    logPrintln(String("LOG: Moving RIGHT ") + stepCount + " steps.");
    runXAxis(stepCount, true, delayUs);
  }
  logPrintln("LOG: Lowering Z2 to complete PRESET command.");
  runZAxis(false, 2, -1, delayUs);
  logPrintln("LOG: PRESET command done.");
  activeCommand = "";
}

void processCommand(String command) {
  Serial.print("Received command: ");
  Serial.println(command);
  
  // Process numeric commands immediately.
/*calibrate*/  if (isNumericCommand(command)) {
    processNumberCommand(command);
    return;
  }
  
/*  if (command == "GET_STATUS") {
    sendStatusUpdate();
    return;
  }*/
  
 // Handle movement commands first
  if (command.startsWith("MOVE_LEFT")) {
    handleMoveX(command, false);

 return;
  }

  if (command.startsWith("MOVE_RIGHT")) {
    handleMoveX(command, true);
    return;
  }

  if (command.startsWith("MOVE_UP")) {
    handleMoveZ(command, true, 0, "MOVE_UP");
    return;
  }

  if (command.startsWith("MOVE_DOWN")) {
    handleMoveZ(command, false, 0, "MOVE_DOWN");
    return;
  }

  if (command.startsWith("MOVE_Z1_UP")) {
    handleMoveZ(command, true, 1, "MOVE_Z1_UP");
    return;
  }

  if (command.startsWith("MOVE_Z1_DOWN")) {
    handleMoveZ(command, false, 1, "MOVE_Z1_DOWN");
    return;
  }

  if (command.startsWith("MOVE_Z2_UP")) {
    handleMoveZ(command, true, 2, "MOVE_Z2_UP");
    return;
  }

  if (command.startsWith("MOVE_Z2_DOWN")) {
    handleMoveZ(command, false, 2, "MOVE_Z2_DOWN");
    return;
  }
  
  // Process other commands.
  if (!canExecuteCommand(command)) return;
  
  if (command.startsWith("PRESET")) {
    handlePreset(command);
    return;
  }

  if (command.startsWith("HOME")) {
    handleHome(command);
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
    logPrintln("LOG: Emergency Stop Activated.");
    delay(100);  // Short delay to process the stop
    emergencyStopActive = false; // Reset for future commands

  } else if (command == "CLEAR_ICE") {
    logPrintln("LOG: Starting CLEAR_ICE...");
    executeClearIceCommand();
    logPrintln("LOG: Done CLEAR_ICE");
    activeCommand = "";
    
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
