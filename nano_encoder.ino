#include <Wire.h>

#define AS5600_ADDR  0x36
const float SCALE = 360.0f / 4096.0f;

uint16_t lastRaw       = 0;
long     rotationCount = 0;
float    lastTotal     = 0;
unsigned long lastTime = 0;

float totalAngle = 0.0f;
float angularvelocity   = 0.0f;

uint16_t readRawAngle() {
  Wire.beginTransmission(AS5600_ADDR);
  Wire.write(0x0E);
  if (Wire.endTransmission() != 0) return lastRaw;
  Wire.requestFrom(AS5600_ADDR, (uint8_t)2);
  if (Wire.available() < 2) return lastRaw;
  uint16_t raw = (Wire.read() << 8) | Wire.read();
  return raw & 0x0FFF;
}

void setup() {
  Serial.begin(115200);    // USB Serial (to laptop)
  Serial1.begin(115200);   // UART Serial1 (to Teensy via TX/RX)
  Wire.begin();

  while (!Serial);         // wait for USB serial

  lastRaw   = readRawAngle();
  lastTotal = lastRaw * SCALE;
  lastTime  = micros();
}

void loop() {
  uint16_t raw   = readRawAngle();
  float    angle = raw * SCALE;

  int16_t diff = int16_t(raw) - int16_t(lastRaw);
  if      (diff >  2048) rotationCount--;
  else if (diff < -2048) rotationCount++;

  totalAngle = rotationCount * 360.0f + angle;

  unsigned long now = micros();
  float dt = (now - lastTime) * 1e-6f;
  angularvelocity = (totalAngle - lastTotal) / dt;

  // Print to USB Serial (laptop)
  Serial.print("Angle: ");
  Serial.print(totalAngle, 2);
  Serial.print(" deg,  angularvelocity: ");
  Serial.print(angularvelocity, 2);
  Serial.println(" deg/s");

  // Send binary to Teensy (8 bytes total)
  Serial1.write((uint8_t*)&totalAngle, sizeof(totalAngle));
  Serial1.write((uint8_t*)&angularvelocity,   sizeof(angularvelocity));

  lastRaw   = raw;
  lastTotal = totalAngle;
  lastTime  = now;

  delay(100);
}
