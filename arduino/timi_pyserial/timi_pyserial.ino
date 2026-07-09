#include "FlySkyIBus.h"
#include <Servo.h>
#include <Wire.h>
#include "MS5837.h"

#define BUFFER_SIZE 128
char buffer[BUFFER_SIZE];
int bufferIndex = 0;

// Channels
int CH1, CH2, CH3, CH4, CH5, CH6, CH7, CH8;

// Pin definitions
int thrustersbfrPin  = 13;
int thrustersbms1Pin = 12;
int thrustersbms2Pin = 11;
int thrustersbafPin  = 10;
int thrusterpsfrPin  = 2;
int thrusterpsms1Pin = 6;
int thrusterpsms2Pin = 8;
int thrusterpsafPin  = 9;
int light1Pin        = 5;
int light2Pin        = 4;
int indicator        = 16;

// Servo objects
Servo thrustersbfr, thrustersbms1, thrustersbms2, thrustersbaf;
Servo thrusterpsfr, thrusterpsms1, thrusterpsms2, thrusterpsaf;
Servo lightfl, lightbl;

// Timing / state
bool first_count = true;
float start_millis, seconds_counter, current_millis;
bool auto_active = false;
unsigned long auto_start_time = 0;
const unsigned long AUTO_TIMEOUT = 120000;

// ── MS5837 sensor ──────────────────────────────────────────────
MS5837 sensor;
bool sensorOK = false;
unsigned long lastSensorRead = 0;
const unsigned long SENSOR_INTERVAL = 1000; // ms between readings
// ──────────────────────────────────────────────────────────────

// ── Helpers ────────────────────────────────────────────────────
int readChannel(byte channelInput, int minLimit, int maxLimit, int defaultValue) {
  uint16_t ch = IBus.readChannel(channelInput);
  if (ch < 5) return defaultValue;
  return map(ch, 1000, 2000, minLimit, maxLimit);
}

void stopAllThrusters() {
  thrusterpsfr.writeMicroseconds(1500);
  thrusterpsaf.writeMicroseconds(1500);
  thrustersbfr.writeMicroseconds(1500);
  thrustersbaf.writeMicroseconds(1500);
  thrusterpsms1.writeMicroseconds(1500);
  thrustersbms1.writeMicroseconds(1500);
  thrusterpsms2.writeMicroseconds(1500);
  thrustersbms2.writeMicroseconds(1500);
  lightfl.writeMicroseconds(1100);
  lightbl.writeMicroseconds(1100);
}
// ──────────────────────────────────────────────────────────────

void RF_MAN(int CH1, int CH2, int CH3, int CH4) {
  if (CH4 < 1530 && CH4 > 1470) {
    thrusterpsfr.writeMicroseconds((CH2 + CH1) / 2);
    thrusterpsaf.writeMicroseconds((CH2 + 3000 - CH1) / 2);
    thrustersbfr.writeMicroseconds((CH2 + 3000 - CH1) / 2);
    thrustersbaf.writeMicroseconds((CH2 + CH1) / 2);
    thrusterpsms1.writeMicroseconds(CH3);
    thrustersbms1.writeMicroseconds(CH3);
    thrusterpsms2.writeMicroseconds(CH3);
    thrustersbms2.writeMicroseconds(CH3);
  } else {
    thrusterpsfr.writeMicroseconds(CH4);
    thrusterpsaf.writeMicroseconds(CH4);
    thrustersbfr.writeMicroseconds(3000 - CH4);
    thrustersbaf.writeMicroseconds(3000 - CH4);
    thrusterpsms1.writeMicroseconds(CH3);
    thrustersbms1.writeMicroseconds(CH3);
    thrusterpsms2.writeMicroseconds(CH3);
    thrustersbms2.writeMicroseconds(CH3);
  }
}

void processMessage(char* message) {
  String input = String(message);
  int values[8];
  int startIndex = 0, commaIndex = 0;
  for (int i = 0; i < 8; i++) {
    commaIndex = input.indexOf(',', startIndex);
    if (commaIndex == -1) commaIndex = input.length();
    values[i] = input.substring(startIndex, commaIndex).toInt();
    startIndex = commaIndex + 1;
  }

  thrusterpsfr.writeMicroseconds(values[0]);
  thrustersbfr.writeMicroseconds(values[1]);
  thrusterpsaf.writeMicroseconds(values[2]);
  thrustersbaf.writeMicroseconds(values[3]);
  thrusterpsms1.writeMicroseconds(values[4]);
  thrustersbms1.writeMicroseconds(values[5]);
  thrusterpsms2.writeMicroseconds(values[6]);
  thrustersbms2.writeMicroseconds(values[7]);
  lightfl.writeMicroseconds(1900);
  lightbl.writeMicroseconds(1900);
}

void Auto() {
  // Use a static buffer to maintain data across loop() calls
  static char buffer[BUFFER_SIZE];
  static int bufferIndex = 0;

  while (Serial.available() > 0) {
    char incomingChar = Serial.read();
    
    if (incomingChar == '\n') {
      buffer[bufferIndex] = '\0'; // Null-terminate
      processMessage(buffer);     // Parse the data
      bufferIndex = 0;            // Reset index
    } else if (incomingChar >= ' ') { // Only store printable characters
      if (bufferIndex < BUFFER_SIZE - 1) {
        buffer[bufferIndex++] = incomingChar;
      } else {
        bufferIndex = 0; // Overflow safety
      }
    }
  }
}

// ── Non-blocking sensor read & transmit ───────────────────────
void updateSensor() {
  if (!sensorOK) return;

  unsigned long now = millis();
  if (now - lastSensorRead < SENSOR_INTERVAL) return;
  lastSensorRead = now;

  sensor.read();

  // Send as a compact CSV line so your topside computer can parse it:
  // "S,<pressure_mbar>,<temp_C>,<depth_m>"
  Serial.print("S,");
  Serial.print(sensor.pressure());
  Serial.print(",");
  Serial.print(sensor.temperature());
  Serial.print(",");
  Serial.println(sensor.depth());
}
// ──────────────────────────────────────────────────────────────

void setup() {
  Serial.begin(115200);
  Serial.setTimeout(10);
  IBus.begin(Serial1);

  // ── Init MS5837 ───────────────────────────────────────────
  Wire.begin();
  if (sensor.init()) {
    sensor.setFluidDensity(1029); // seawater; use 997 for freshwater
    sensorOK = true;
    Serial.println("MS5837 OK");
  } else {
    Serial.println("MS5837 init failed - continuing without sensor");
  }
  // ─────────────────────────────────────────────────────────

  lightfl.attach(light1Pin);
  lightbl.attach(light2Pin);
  pinMode(indicator, OUTPUT);

  thrustersbfr.attach(thrustersbfrPin);
  thrusterpsfr.attach(thrusterpsfrPin);
  thrustersbms1.attach(thrustersbms1Pin);
  thrusterpsms1.attach(thrusterpsms1Pin);
  thrustersbms2.attach(thrustersbms2Pin);
  thrusterpsms2.attach(thrusterpsms2Pin);
  thrustersbaf.attach(thrustersbafPin);
  thrusterpsaf.attach(thrusterpsafPin);

  stopAllThrusters();
  delay(2000);
}

void loop() {
  IBus.loop();
  CH5 = IBus.readChannel(4);
  CH6 = IBus.readChannel(5);
  CH7 = IBus.readChannel(6);
  CH8 = IBus.readChannel(7);

  digitalWrite(indicator, HIGH);

  // ── Read & transmit sensor data (non-blocking) ────────────
  updateSensor();
  // ─────────────────────────────────────────────────────────

  // ── Kill switch ──────────────────────────────────────────
  if (CH6 == 2000) {
    if (first_count) {
      start_millis = millis();
      seconds_counter = 0;
      first_count = false;
    }
    current_millis = millis();
    if (current_millis - start_millis >= 1000) {
      stopAllThrusters();
    }
  }
  // ── Autonomy ─────────────────────────────────────────────
  else if (CH5 == 2000) {
    if (!auto_active) {
    // ✅ First entry into Auto mode — stop all thrusters ONCE
    auto_active = true;
    auto_start_time = millis();
    stopAllThrusters();   // <-- moved here
    delay(500);           // brief pause so ESCs register neutral
    }
    if (millis() - auto_start_time <= AUTO_TIMEOUT) {
      Auto();
      lightfl.writeMicroseconds(1900);
      lightbl.writeMicroseconds(1900);
    } else {
      stopAllThrusters();
    }
  }
  // ── Manual RF ─────────────────────────────────────────────
  else if (CH5 == 1000) {
    first_count = true;
    auto_active  = false;   // reset autonomy flag when back in manual

    CH1 = readChannel(0, 1300, 1700, 1500);
    CH2 = readChannel(1, 1300, 1700, 1500);
    CH3 = readChannel(2, 1300, 1700, 1500);
    CH4 = readChannel(3, 1300, 1700, 1500);

    RF_MAN(CH1, CH2, CH3, CH4);
  }
}