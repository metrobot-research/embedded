// Code heavily based upon Shay Shacket's Self-balancing Inverted Pendulum Robot: https://www.shaysackett.com/inverted-pendulum-robot/

#include <Wire.h>
#include <ICM20948_WE.h>
#include <Adafruit_PWMServoDriver.h>
#include <ESP32Encoder.h>
#include <math.h>
#include <PID_v1.h>

// IMU Constants
#define ICM20948_ADDR 0x69

// Motor Constants
#define MOTOR1_PIN1 0
#define MOTOR1_PIN2 1
#define MOTOR2_PIN1 2
#define MOTOR2_PIN2 3
#define ENCODER1_PIN1 33
#define ENCODER1_PIN2 4
#define ENCODER2_PIN1 5
#define ENCODER2_PIN2 13
#define MAX_PWM 4096

const byte motor1_minimum_motor_speed = 0;
const byte motor2_minimum_motor_speed = 4;

// Timer Constants
#define LOOP_DELAY 4000

ESP32Encoder encoder1;
ESP32Encoder encoder2;
ICM20948_WE myIMU = ICM20948_WE(ICM20948_ADDR);
Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();

long acc_total_vector;
float angle_pitch, angle_roll, acc_angle_pitch, acc_angle_roll;
float angle_pitch_output, angle_roll_output;
unsigned long looptime;
boolean set_gyro_angles;
boolean first_up = true;

long x_final_left = 0;
long x_final_right = 0;
long x_initial_left = 0;
long x_initial_right = 0;

float velocity_left = 0;
float velocity_right = 0;
float average_velocity = 0;
float filtered_velocity_left = 0;
float filtered_velocity_right = 0;
float filtered_velocity_average = 0;

unsigned int time_multiple_right = 1;
unsigned int time_multiple_left = 1;

// Angle Control PID

double angle_setpoint, angle_input, angle_output, angle_error;

double angle_Kp, angle_Ki, angle_Kd;

PID angle_PID(&angle_input, &angle_output, &angle_setpoint, angle_Kp, angle_Ki, angle_Kd, DIRECT);

// Velocity Control PID

double velocity_setpoint,smoothed_velocity_setpoint, velocity_input, velocity_output, velocity_error;

double velocity_Kp, velocity_Ki, velocity_Kd;

PID velocity_PID(&velocity_input, &velocity_output, &smoothed_velocity_setpoint, velocity_Kp, velocity_Ki, velocity_Kd, DIRECT); 

#define DELIMITER '\n'
String command = "";

void motorsOff() {
  // Motor 1:
  pwm.setPin(MOTOR1_PIN1,0,0);
  pwm.setPin(MOTOR1_PIN2,0,0);
  // Motor 2:
  pwm.setPWM(MOTOR2_PIN1,0,0);
  pwm.setPWM(MOTOR1_PIN2,0,0);
}

void motor1Speed(int u) {
  bool forward = u > 0;
  u = abs(u);
  if (u > MAX_PWM)
    u = MAX_PWM;
  if (u < motor1_minimum_motor_speed)
    u = motor1_minimum_motor_speed;
  
  if (forward) {
    pwm.setPin(MOTOR1_PIN1, u, 0);
    pwm.setPin(MOTOR1_PIN2, 0, 0);
  } else {
    pwm.setPin(MOTOR1_PIN1, 0, 0);
    pwm.setPin(MOTOR1_PIN2, u, 0);
  }
}

void motor2Speed(int u) {
  bool forward = u > 0;
  u = abs(u);
  if (u > MAX_PWM)
    u = MAX_PWM;
  if (u < motor2_minimum_motor_speed)
    u = motor2_minimum_motor_speed;
  
  if (forward) {
    pwm.setPin(MOTOR2_PIN1, 0, 0);
    pwm.setPin(MOTOR2_PIN2, u, 0);
  } else {
    pwm.setPin(MOTOR2_PIN1, u, 0);
    pwm.setPin(MOTOR2_PIN2, 0, 0);
  }
}

void setAnglePID(){
   angle_Kp = 28;
   angle_Ki = 13;
   angle_Kd = .35;   //Keep angle_Kd relatively small compared to the other values
   angle_PID.SetTunings(angle_Kp,angle_Ki,angle_Kd);
}

void initAnglePID() {
  angle_input = 0;
  angle_setpoint = 0;
  angle_PID.SetSampleTime(4);
  angle_PID.SetOutputLimits(-MAX_PWM, MAX_PWM);
  setAnglePID();
}

void setVelocityPID(){
  velocity_Kp = .15;    //.2
  velocity_Ki = .08;   //.15
  velocity_Kd = 0;    //0
  velocity_PID.SetTunings(velocity_Kp,velocity_Ki,velocity_Kd);
}

void initVelocityPID() {
  velocity_input = 0;
  velocity_setpoint = 0;
  smoothed_velocity_setpoint = 0;
  velocity_PID.SetMode(AUTOMATIC);
  velocity_PID.SetSampleTime(4);
  velocity_PID.SetOutputLimits(-MAX_PWM, MAX_PWM);
  setVelocityPID();
}

void processInput() {
  if (Serial.available() > 0) {
    char c = Serial.read();
    if (c == DELIMITER) {
      String var = command.substring(0, 2);
      double value = command.substring(2).toDouble();
      if (var.equals("ap")) {
        angle_Kp = value;
      } else if (var.equals("ai")) {
        angle_Ki = value;
      } else if (var.equals("ad")) {
        angle_Kd = value;
      } else if (var.equals("vp")) {
        velocity_Kp = value;
      } else if (var.equals("vi")) {
        velocity_Ki = value;
      } else if (var.equals("vd")) {
        velocity_Kd = value;
      } else {
        Serial.println("Invalid variable");
      }
      angle_PID.SetTunings(angle_Kp,angle_Ki,angle_Kd);
      velocity_PID.SetTunings(velocity_Kp,velocity_Ki,velocity_Kd);
      command = "";
    }
  }
}

void angleCalc() {
  myIMU.readSensor();
  xyzFloat gyr = myIMU.getGyrValues();

  angle_pitch += gyr.y * .0000698;
  angle_roll += gyr.x * .0000698;

  xyzFloat acc = myIMU.getGValues();

  acc_total_vector = sqrt(acc.x * acc.x + acc.y * acc.y + acc.z * acc.z);   //Calculate the total Accelerometer vector (Gravity Vector)
  //57.296 = 1 / (3.142 / 180) The Arduino asin function is in radians
  acc_angle_pitch = asin((float)acc.y/acc_total_vector);       //Calculate the pitch angle
  acc_angle_roll = asin((float)acc.x/acc_total_vector);

  if (set_gyro_angles) {
    angle_pitch = angle_pitch * .9996 + acc_angle_pitch * .0004;
    angle_roll = angle_roll * .9996 + acc_angle_roll * .0004;
  } else {
    angle_pitch = acc_angle_pitch;
    angle_roll = acc_angle_roll;
    set_gyro_angles = true;
  }

  angle_pitch_output = angle_pitch;
  angle_roll_output = angle_roll;
}

void velocityCalc() {
  x_final_left = encoder1.getCount();
  if (x_final_left != x_initial_left) {
    velocity_left = ((x_final_left - x_initial_left)*.000283)/(.004 * time_multiple_left);
    x_initial_left = x_final_left;
    time_multiple_left = 1;                                                                    //if this bit of code has been run reset delta t to be multiplied by 1
  } else {
    time_multiple_left++;
  }

  x_final_right = encoder2.getCount();
  if (x_final_right != x_initial_right) {
    velocity_right = ((x_final_right - x_initial_right)*.000283)/(.004 * time_multiple_right);
    x_initial_right = x_final_right;
    time_multiple_right = 1;                                                                    //if this bit of code has been run reset delta t to be multiplied by 1
  } else {
    time_multiple_right++;
  }

  filtered_velocity_left = filtered_velocity_left * .95 + velocity_left * .05;
  filtered_velocity_right = filtered_velocity_right * .95 + velocity_right * .05;
  filtered_velocity_average = (filtered_velocity_left  + -filtered_velocity_right)/2;
}

void balance() {
  angle_input = angle_pitch_output;
  angle_PID.Compute();
  angle_error = angle_setpoint - angle_input;
  if (abs(angle_error) < 40) {
    angle_PID.SetMode(AUTOMATIC);
    angle_output = int(angle_output);
    motor1Speed(angle_output);
    motor2Speed(angle_output);
  } else {
    angle_PID.SetMode(MANUAL);
    motorsOff();
    angle_output = 0;
  }

  velocity_PID.SetMode(AUTOMATIC);
  velocityCalc();
  velocity_input = filtered_velocity_average;
  velocity_error = velocity_setpoint - velocity_input;
  velocity_PID.Compute();
  angle_setpoint = velocity_output;
}

void setup() {
  Serial.begin(115200);
  // Wait until serial port initialized before progressing...
  while (!Serial){}

  // i2c wire initialization
  Wire.begin();
  Wire.setClock(400000);
  
  if(!myIMU.init()){
    Serial.println("ICM20948 does not respond");
  }
  delay(1000);

  myIMU.setGyrOffsets(-27.5680, -86.3617, -4.2623);
  myIMU.setGyrRange(ICM20948_GYRO_RANGE_250);
  myIMU.setGyrDLPF(ICM20948_DLPF_6);
  myIMU.setAccRange(ICM20948_ACC_RANGE_2G);
  myIMU.setAccDLPF(ICM20948_DLPF_6);
  
  // Initialize encoders
  ESP32Encoder::useInternalWeakPullResistors=UP;
  
  encoder1.attachHalfQuad(ENCODER1_PIN1, ENCODER1_PIN2);
  encoder2.attachHalfQuad(ENCODER2_PIN1, ENCODER2_PIN2);
  
  encoder1.clearCount();
  encoder2.clearCount();
  
  // Initialize PWM controller for motors
  pwm.begin();
  pwm.setOscillatorFrequency(27000000);
  pwm.setPWMFreq(440);

  motorsOff();

  looptime = micros();

  initAnglePID();
  initVelocityPID();
}

void loop() {
  processInput();

  angleCalc(); // Calculate angle of robot

  balance(); // Run the balancing routine

  while (micros() - looptime < LOOP_DELAY);
  looptime = micros();
}