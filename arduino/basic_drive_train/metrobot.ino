const short LEFT_BASE_PWM_PIN = 10;
const short LEFT_BASE_DIR_PIN = 12;
const short RIGHT_BASE_PWM_PIN = 11;
const short RIGHT_BASE_DIR_PIN = 13;

const String CMD_FORWARD = "forward";
const String CMD_BACKWARD = "backward";
const String CMD_RIGHT_PIVOT = "rightP";
const String CMD_LEFT_PIVOT = "leftP";

const String ACK_FORWARD = "doneForward";
const String ACK_BACKWARD = "doneBackward";
const String ACK_RIGHT_PIVOT = "doneRightP";
const String ACK_LEFT_PIVOT = "doneLeftP";

const double LEFT_BASE_SCALE = 1.0;
const double RIGHT_BASE_SCALE = 0.80;

void drive(int leftPower, int rightPower) {
  analogWrite(LEFT_BASE_PWM_PIN, (int) min(max(abs(leftPower) * LEFT_BASE_SCALE, 0), 255));
  analogWrite(RIGHT_BASE_PWM_PIN, (int) min(max(abs(rightPower) * RIGHT_BASE_SCALE, 0), 255));

  if (leftPower < 0) {
    digitalWrite(LEFT_BASE_DIR_PIN, LOW);
  } else {
    digitalWrite(LEFT_BASE_DIR_PIN, HIGH);
  }

  if (rightPower < 0) {
    digitalWrite(RIGHT_BASE_DIR_PIN, LOW);
  } else {
    digitalWrite(RIGHT_BASE_DIR_PIN, HIGH);
  }
}

void coast(int leftFrom, int rightFrom, float stepSize = 0.1, int duration = 20) {
  int stepCount = round(1 / stepSize);
  for (int i = stepCount; i > 0; --i) {
    drive(leftFrom * stepCount * stepSize, rightFrom * stepCount * stepSize);
    delay(duration);
  }
  drive(0, 0);
}

void forward(int duration = 500, bool shouldCoast = false) {
  drive(127, 127);

  delay(duration);

  if (shouldCoast) {
    coast(127, 127);
  } else {
    drive(0, 0);
  }

  Serial.println(ACK_FORWARD);
}

void backward(int duration = 500, bool shouldCoast = false) {
  drive(-127, -127);

  delay(duration);

  if (shouldCoast) {
    coast(-127, -127);
  } else {
    drive(0, 0);
  }

  Serial.println(ACK_BACKWARD);
}

void rightPivot(int duration = 500, bool shouldCoast = false) {
  drive(127, -127);

  delay(duration);

  if (shouldCoast) {
    coast(127, -127);
  } else {
    drive(0, 0);
  }

  drive(0, 0);
  Serial.println(ACK_RIGHT_PIVOT);
}

void leftPivot(int duration = 500, bool shouldCoast = false) {
  drive(-127, 127);

  delay(duration);

  if (shouldCoast) {
    coast(-127, 127);
  } else {
    drive(0, 0);
  }

  drive(0, 0);
  Serial.println(ACK_LEFT_PIVOT);
}

void setup() {
  Serial.begin(9600);

  pinMode(LEFT_BASE_PWM_PIN, OUTPUT);
  pinMode(LEFT_BASE_DIR_PIN, OUTPUT);
  pinMode(RIGHT_BASE_PWM_PIN, OUTPUT);
  pinMode(RIGHT_BASE_DIR_PIN, OUTPUT);
}

void loop() {
  while (true) {
    if (Serial.available() > 0) {
      String request = Serial.readString();
      int cmdIndex = request.indexOf(',');
      if (cmdIndex == -1) {
        Serial.println("Malformed request " + request);
        continue;
      }

      String cmd = request.substring(0, cmdIndex);
      int distIndex = request.indexOf(',', cmdIndex + 1);
      int coastIndex = request.indexOf(',', distIndex + 1);
      if (distIndex == -1 || coastIndex == -1) {
        Serial.println("Malformed command " + request);
        continue;
      }
      String distStr = request.substring(cmdIndex + 1, distIndex);
      int distance = distStr.toInt();
      String coastStr = request.substring(distIndex + 1, coastIndex);
      bool coast = coastStr.equals("true");

      if (cmd.equals(CMD_FORWARD)) {
        forward(distance, coast);
      } else if (cmd.equals(CMD_BACKWARD)) {
        backward(distance, coast);
      } else if (cmd.equals(CMD_RIGHT_PIVOT)) {
        rightPivot(distance, coast);
      } else if (cmd.equals(CMD_LEFT_PIVOT)) {
        leftPivot(distance, coast);
      } else {
        Serial.println("Unknown command " + cmd);
      }
    }
  }
}
