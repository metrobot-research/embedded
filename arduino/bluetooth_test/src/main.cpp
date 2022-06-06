#include "BluetoothSerial.h"
#include "ArduinoJson.h"

// Serialization constants
#define DELIMITER '\n'
#define VELOCITY_LIMIT 0.1
#define THETA_DOT_LIMIT 0.1

float r_v, r_theta_dot;

#if !defined(CONFIG_BT_ENABLED) || !defined(CONFIG_BLUEDROID_ENABLED)
#error Bluetooth is not enabled! Please run `make menuconfig` to and enable it
#endif

BluetoothSerial SerialBT;

String command = "";
unsigned long command_index = 0;

/* OPCODES
 *  0 - Turn servo
 *  1 - Set winding motor
 */

/* TELEMETRY TYPES
 *  0 - Keep alive
 */
void set_velocity(JsonArray arguments) {
  if (arguments.size() != 1) {
    Serial.println("Incorrect number of arguments for setting velocity");
    return;
  }
  float velocity_input = arguments[0];
  if (abs(velocity_input) > 1) {
    Serial.println("Invalid velocity input");
    return;
  }
  r_v = velocity_input * VELOCITY_LIMIT;
  char buffer[40];
  sprintf(buffer, "Setting velocity: %6f.", r_v);
  Serial.println(buffer);
}

void set_theta_dot(JsonArray arguments) {
  if (arguments.size() != 1) {
    Serial.println("Incorrect number of arguments for setting theta_dot");
    return;
  }
  float theta_dot_input = arguments[0];
  if (abs(theta_dot_input) > 1) {
    Serial.println("Invalid theta_dot input");
    return;
  }
  r_theta_dot = theta_dot_input * THETA_DOT_LIMIT;
  char buffer[40];
  sprintf(buffer, "Setting theta_dot: %6f.", r_theta_dot);
  Serial.println(buffer);
}

void processReceivedValue(char b){
  if(b == DELIMITER){
    DynamicJsonDocument doc(1024);
    deserializeJson(doc, command);
    int opcode = doc["command"];
    JsonArray arguments = doc["args"];
    if (arguments != NULL) {
      switch (opcode) {
        case 0:
          set_velocity(arguments);
          break;
        case 1:
          set_theta_dot(arguments);
          break;
      }
    } else {
      Serial.println("Opcode or arguments not passed in");
    }
    command = "";
  }
  else {
    command.concat(b);
  }
 
  return;
}


void setup() {
  Serial.begin(115200);
  SerialBT.begin("ESP32");
  delay(500); 
}

void loop() {
  if (SerialBT.available()) {
    char c = SerialBT.read();
    Serial.print(c);
    processReceivedValue(c);
  }
}