#include <Wire.h>
#include <ICM20948_WE.h>
#include <Adafruit_PWMServoDriver.h>
#include <ESP32Encoder.h>
#include <math.h>
#include <cmath>
#include <ArduinoEigen.h>
#include <PID_v1.h>
#include <BluetoothSerial.h>
#include <ArduinoJson.h>

using namespace Eigen;
using Eigen::Matrix;
using Eigen::MatrixXd;

// Check if bluetooth is enabled
#if !defined(CONFIG_BT_ENABLED) || !defined(CONFIG_BLUEDROID_ENABLED)
#error Bluetooth is not enabled!
#endif

// Pins and addresses
#define ICM20948_ADDR 0x69 // IMU I2C address
#define OCM_1 36
#define OCM_2 39
#define OCM_HIP_RIGHT 34      // Right hip motor current sensor
#define OCM_HIP_LEFT 35       // Left hip motor current sensor
#define WHEEL_MOTOR_LEFT_A 0  // Left wheel motor pin A
#define WHEEL_MOTOR_LEFT_B 1  // Left wheel motor pin B
#define WHEEL_MOTOR_RIGHT_A 2 // Right wheel motor pin A
#define WHEEL_MOTOR_RIGHT_B 3 // Right wheel motor pin B
#define HIP_MOTOR_LEFT_A 6    // Left hip motor pin A
#define HIP_MOTOR_LEFT_B 7    // Left hip motor pin B
#define HIP_MOTOR_RIGHT_A 4   // Right hip motor pin A
#define HIP_MOTOR_RIGHT_B 5   // Right hip motor pin B
#define NECK_MOTOR_A 8        // Neck Angle Motor pin A
#define NECK_MOTOR_B 9        // Neck Angle Motor pin B
#define NECK_SERVO 12         // Servo_1 on PCB
#define GRASPER_SERVO 13      // Servo_2 on PCB
//#define RXD_2 16      //probably unneeded
//#define TXD_2 17      //probably unneeded

// Serialization constants
#define DELIMITER '\n'

// Mathematical/physical constants
#define WHEEL_RADIUS 0.0508 // Wheel radius in meters

// Sensor constants
#define CF_TIME_CONSTANT 0.995 // Time constant a for complementary filter

// General PID constants
#define MAX_VELOCITY 0.1       // Maximum magnitude of velocity PID output
#define MAX_PWM 4096           // Maximum magnitude of motor controller PID outputs
#define PHI_SETPOINT_RATIO 1.0 // Ratio of setpoint of tilt controller to output of velocity controller

// Time constants
#define TIMER_INTERVAL_MS 25 // Interval between timer interrupts

// Serial controller definitions
BluetoothSerial SerialBT;
String command_BT = "";
String command_serial = "";
String command_2 = "";

// Encoder definitions
ESP32Encoder encoder_wheel_right; // Right wheel encoder
ESP32Encoder encoder_wheel_left;  // Left wheel encoder
ESP32Encoder encoder_hip_right;   // Right hip motor encoder
ESP32Encoder encoder_hip_left;    // Left hip motor encoder

volatile int count_r_total = 0; // Right wheel total encoder count
volatile int count_l_total = 0; // Left wheel total encoder count
volatile int count_r = 0;       // Right wheel intermediate encoder count
volatile int count_l = 0;       // Left wheel intermediate encoder count

// PID Constants for wheel velocity (loop one)
double Kp_xdot = 0.030;
double Ki_xdot = 0.3;
double Kd_xdot = 0.002;

double r_xdot = 0; // Commanded wheel speed in m/s
double u_xdot = 0; // Output of wheel velocity PID loop
double xdot_l = 0;
double xdot_r = 0;
double xdot = 0; // Current average velocity of left and right wheels

// PID constants for forward/backward tilt (loop two)
double Kp_phi = 12500;
double Ki_phi = 145000;
double Kd_phi = 2000;

double r_phi = 0;   // Commanded tilt in radians
double u_phi = 0;   // Output of tilt PID loop
double phi = 0;     // Tilt calculated by gyro
double phi_acc = 0; // Tilt calculated by accelerometer

// PID constants for turning velocity
double Kp_theta_dot = 200;
double Ki_theta_dot = 150;
double Kd_theta_dot = 5;

double r_theta_dot = 0; // Commanded turning velocity in rad/s
double u_theta_dot = 0; // Output of turning velocity PID loop
double theta_dot = 0;   // Current turning velocity

PID velocity_PID(&xdot, &u_xdot, &r_xdot, Kp_xdot, Ki_xdot, Kd_xdot, DIRECT);
PID angle_PID(&phi, &u_phi, &r_phi, Kp_phi, Ki_phi, Kd_phi, DIRECT);
PID turning_velocity_PID(&theta_dot, &u_theta_dot, &r_theta_dot, Kp_theta_dot, Ki_theta_dot, Kd_theta_dot, DIRECT);

// Hardware timer definitions
hw_timer_t *timer0;
portMUX_TYPE timerMux0 = portMUX_INITIALIZER_UNLOCKED;
volatile bool interrupt_complete = false; // Flag set upon interrupt, reset during main loop
int timesteps_passed = 0;                 // Interrupts completed since start of execution

// IMU definitions
ICM20948_WE IMU = ICM20948_WE(ICM20948_ADDR);

// Accelerometer definitions
Vector3f ACC_VERTICAL_VEC; // Vertical vector from accelerometer readings while robot is held upright
Vector3f acc_vec;          // Current acceleration vector from accelerometer
Vector3f acc_normal_vec;   // Normal vector to current acceleration and vertical acceleration vectors

// PWM definitions
Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();

// Interrupt handler that runs every TIMER_INTERVAL_MS ms
void IRAM_ATTR timerInterruptHandler()
{
  portENTER_CRITICAL_ISR(&timerMux0);

  // Get current encoder counts
  count_r = -1 * encoder_wheel_right.getCount();
  count_l = encoder_wheel_left.getCount();
  count_r_total += count_r;
  count_l_total += count_l;

  // Clear encoders
  encoder_wheel_right.clearCount();
  encoder_wheel_right.clearCount();

  interrupt_complete = true;
  portEXIT_CRITICAL_ISR(&timerMux0);
}

// Drive motors at desired PWM
void driveMotors(float u_l, float u_r)
{
  // Constrain motor input
  u_l = constrain(u_l, -MAX_PWM, MAX_PWM);
  u_r = constrain(u_r, -MAX_PWM, MAX_PWM);

  // Set motor pins based on drive direction

  if (u_l >= 0)
  {
    pwm.setPin(WHEEL_MOTOR_LEFT_A, u_l, 0);
    pwm.setPin(WHEEL_MOTOR_LEFT_B, 0, 0);
  }
  else
  {
    pwm.setPin(WHEEL_MOTOR_LEFT_A, 0, 0);
    pwm.setPin(WHEEL_MOTOR_LEFT_B, -u_l, 0);
  }

  if (u_r >= 0)
  {
    pwm.setPin(WHEEL_MOTOR_RIGHT_A, 0, 0);
    pwm.setPin(WHEEL_MOTOR_RIGHT_B, u_r, 0);
  }
  else
  {
    pwm.setPin(WHEEL_MOTOR_RIGHT_A, -u_r, 0);
    pwm.setPin(WHEEL_MOTOR_RIGHT_B, 0, 0);
  }
}

// Synthesize and filter gyro and accelerometer readings to get accurate angle measurements
void getAngles()
{
  // Read IMU
  IMU.readSensor();
  xyzFloat acc = IMU.getGValues();   // Acceleration vector in m/s
  xyzFloat gyr = IMU.getGyrValues(); // Angular velocity vector in deg/s

  // Calculate tilt angle from gyro data
  phi = phi - gyr.y * TIMER_INTERVAL_MS / 1000 * (PI / 180); // Integrate gyro angular velocity about the y-axis to get tilt
  theta_dot = gyr.z * TIMER_INTERVAL_MS / 1000 * (PI / 180);

  // Calculate tilt angle from accelerometer data
  acc_vec << acc.x, acc.y, acc.z;
  phi_acc = atan2((ACC_VERTICAL_VEC.cross(acc_vec)).dot(acc_normal_vec), acc_vec.dot(ACC_VERTICAL_VEC));

  // Complementary filter
  phi = CF_TIME_CONSTANT * phi + (1 - CF_TIME_CONSTANT) * phi_acc;
}

// Initialize velocity PID loop
void init_velocity_PID()
{
  velocity_PID.SetSampleTime(TIMER_INTERVAL_MS);
  velocity_PID.SetOutputLimits(-MAX_VELOCITY, MAX_VELOCITY);
  velocity_PID.SetMode(AUTOMATIC);
}

// Initialize angle PID loop
void init_angle_PID()
{
  angle_PID.SetSampleTime(TIMER_INTERVAL_MS);
  angle_PID.SetOutputLimits(-MAX_PWM, MAX_PWM);
  angle_PID.SetMode(AUTOMATIC);
}

// Initialize turning velocity PID loop
void init_turning_velocity_PID()
{
  turning_velocity_PID.SetSampleTime(TIMER_INTERVAL_MS);
  turning_velocity_PID.SetOutputLimits(-MAX_PWM, MAX_PWM);
  turning_velocity_PID.SetMode(AUTOMATIC);
}

// Set velocity setpoint
void set_velocity(JsonArray arguments)
{
  if (arguments.size() != 1)
  {
    Serial.println("Incorrect number of arguments for setting velocity");
    return;
  }
  float velocity_input = arguments[0];
  if (abs(velocity_input) > 1)
  {
    Serial.println("Invalid velocity input");
    return;
  }
  r_xdot = velocity_input * MAX_VELOCITY;
  char buffer[40];
  sprintf(buffer, "Setting velocity: %6f.", r_xdot);
  //Serial.println(buffer);
}

// Set turning velocity setpoint
void set_turning_velocity(JsonArray arguments)
{
  if (arguments.size() != 1)
  {
    Serial.println("Incorrect number of arguments for setting theta_dot");
    return;
  }
  float theta_dot_input = arguments[0];
  if (abs(theta_dot_input) > 1)
  {
    Serial.println("Invalid theta_dot input");
    return;
  }
  r_theta_dot = theta_dot_input * MAX_PWM;
  char buffer[40];
  sprintf(buffer, "Setting theta_dot: %6f.", r_theta_dot);
  //Serial.println(buffer);
}

void set_velocity_pid_constants(JsonArray arguments)
{
  if (arguments.size() != 3)
  {
    Serial.println("Incorrect number of arguments for setting velocity PID constants");
    return;
  }
  Kp_xdot = arguments[0];
  Ki_xdot = arguments[1];
  Kd_xdot = arguments[2];
  velocity_PID.SetTunings(Kp_xdot,Ki_xdot,Kd_xdot);
  char buffer[100];
  sprintf(buffer, "Setting Kp_xdot = %6f, Ki_xdot = %6f, Kd_xdot = %6f.", Kp_xdot, Ki_xdot, Kd_xdot);
  Serial.println(buffer);
}

void set_angle_pid_constants(JsonArray arguments)
{
  if (arguments.size() != 3)
  {
    Serial.println("Incorrect number of arguments for setting angle PID constants");
    return;
  }
  Kp_phi = arguments[0];
  Ki_phi = arguments[1];
  Kd_phi = arguments[2];
  angle_PID.SetTunings(Kp_phi,Ki_phi,Kd_phi);
  char buffer[100];
  sprintf(buffer, "Setting Kp_phi = %6f, Ki_phi = %6f, Kd_phi = %6f.", Kp_phi, Ki_phi, Kd_phi);
  Serial.println(buffer);
}

void set_turning_velocity_pid_constants(JsonArray arguments)
{
  if (arguments.size() != 3)
  {
    Serial.println("Incorrect number of arguments for setting turning velocity PID constants");
    return;
  }
  Kp_theta_dot = arguments[0];
  Ki_theta_dot = arguments[1];
  Kd_theta_dot = arguments[2];
  char buffer[100];
  turning_velocity_PID.SetTunings(Kp_xdot,Ki_xdot,Kd_xdot);

  sprintf(buffer, "Setting Kp_theta_dot = %6f, Ki_theta_dot = %6f, Kd_theta_dot = %6f.", Kp_theta_dot, Ki_theta_dot, Kd_theta_dot);
  Serial.println(buffer);
}

// Process JSON commands sent over Bluetooth
//
// OPCODES
// 0 - set_velocity(double velocity)
// 1 - set_theta_dot(double theta_dot)
// 2 - set_velocity_pid_constants(double Kp, double Ki, double Kd)
// 3 - set_phi_pid_constants(double Kp, double Ki, double Kd)
// 4 - set_theta_dot_pid_constants(double Kp, double Ki, double Kd)

void processReceivedValue(char b, String &command)
{
  if (b == DELIMITER)
  {
    DynamicJsonDocument doc(1024);
    deserializeJson(doc, command);
    int opcode = doc["command"];
    JsonArray arguments = doc["args"];
    if (arguments != NULL)
    {
      switch (opcode)
      {
      case 0:
        set_velocity(arguments);
        break;
      case 1:
        set_turning_velocity(arguments);
        break;
      case 2:
        set_velocity_pid_constants(arguments);
        break;
      case 3:
        set_angle_pid_constants(arguments);
        break;
      case 4:
        set_turning_velocity_pid_constants(arguments);
        break;
      }
    }
    else
    {
      Serial.println("Opcode or arguments not passed in");
    }
    command = "";
  }
  else
  {
    command.concat(b);
  }

  return;
}
void processReceivedJetsonValue(char b, String &command_2)
{
  if (b == DELIMITER)
  {
    // Do what we want with the data. In this case, print it. 
    Serial.println("Message "+command_2 +" received on ESP32");
    // Reset the delimiter for the next message to receive.
    command_2="";
  }
  else
  {
    command_2.concat(b);
  }

  return;
}

void publishSensorValues()
{
// Send sensor values to the Jetson over UART using Serial Library
  
/* Inputs
 Acceleration (from the IMU: inertial measurement unit)
 - acc x,y,z
 Gyroscope
 - gyr x,y,z
 State
 - Filtered  Phi_actual
 encoder values:
 - Wheel speed (LW,RW)
 - Hip Angle (LH,RH)
 - Neck Angle (N)
 Servo values:
 2 values
 - Head Angle
 - Grasper Angle

 13 total

 Outputs: 
 Publish over Serial to ROS node listener
*/
uint8_t buf[2] = {'a', 'b'};
Serial2.write(buf, 2);

}

void setup(){
  Serial.begin(115200);
  SerialBT.begin("ESP32");
  Serial2.begin(115200);

  // Wait until serial port initialized before progressing
  while (!Serial)
    ;
  while (!Serial2)
    ;

  // I2C wire initialization
  Wire.begin();
  Wire.setClock(400000);

  // Try initializing IMU
  if (!IMU.init())
  {
    Serial.println("ICM20948 does not respond");
  }
  delay(1000);

  // Set IMU zero values and other parameters
  IMU.setGyrOffsets(-27.5680, -86.3617, -4.2623);
  IMU.setGyrRange(ICM20948_GYRO_RANGE_250);
  IMU.setGyrDLPF(ICM20948_DLPF_6);
  IMU.setAccRange(ICM20948_ACC_RANGE_2G);
  IMU.setAccDLPF(ICM20948_DLPF_6);

  IMU.readSensor();
  xyzFloat acc = IMU.getGValues();

  // Set initial accelerometer values
  ACC_VERTICAL_VEC << 0.20, -0.03, -0.98;
  acc_vec << acc.x, acc.y, acc.z;

  acc_normal_vec << (ACC_VERTICAL_VEC.cross(acc_vec)).normalized();

  // Reverse normal vector if needed
  if (acc_normal_vec(1) < 0)
  {
    acc_normal_vec = acc_normal_vec * -1;
  }

  phi = atan2((ACC_VERTICAL_VEC.cross(acc_vec)).dot(acc_normal_vec), acc_vec.dot(ACC_VERTICAL_VEC));

  phi_acc = phi; // Both phi values initially come from accelerometer

  // Initialize encoders
  ESP32Encoder::useInternalWeakPullResistors = UP;
  encoder_wheel_right.attachHalfQuad(33, 4);
  encoder_wheel_left.attachHalfQuad(5, 13);
  encoder_wheel_right.clearCount();
  encoder_wheel_left.clearCount();

  pwm.begin();
  pwm.setOscillatorFrequency(27000000);
  pwm.setPWMFreq(440);

  pinMode(OCM_1, INPUT);
  pinMode(OCM_2, INPUT);
  pinMode(OCM_HIP_LEFT, INPUT);
  pinMode(OCM_HIP_RIGHT, INPUT);

  // Motor 1:
  pwm.setPin(0, 0, 0);
  pwm.setPin(1, 0, 0);
  // Motor 2:
  pwm.setPWM(2, 0, 0);
  pwm.setPWM(3, 0, 0);

  timer0 = timerBegin(0, 80, true);                           // timer 0, MWDT clock period = 12.5 ns * TIMGn_Tx_WDT_CLK_PRESCALE -> 12.5 ns * 80 -> 1000 ns = 1 us, countUp
  timerAttachInterrupt(timer0, &timerInterruptHandler, true); // edge (not level) triggered
  timerAlarmWrite(timer0, TIMER_INTERVAL_MS * 1000, true);    // 10000 * 1 us = 10 ms, autoreload true
  timerAlarmEnable(timer0);                                   // enable
  delay(2000);

  init_angle_PID();
  init_velocity_PID();
  init_turning_velocity_PID();
}

void loop()
{
  if (interrupt_complete)
  {
    // ensuring reset isn't skipped:
    portENTER_CRITICAL(&timerMux0);
    interrupt_complete = false;
    portEXIT_CRITICAL(&timerMux0);

    // Calculate velocity from encoder readings
    xdot_l = count_l * 4 * 3.14159265 * WHEEL_RADIUS / (2248.86 * TIMER_INTERVAL_MS / 1000);
    xdot_r = count_r * 4 * 3.14159265 * WHEEL_RADIUS / (2248.86 * TIMER_INTERVAL_MS / 1000);
    xdot = ((count_l + count_r) / 2) * 4 * 3.14159265 * WHEEL_RADIUS / (2248.86 * TIMER_INTERVAL_MS / 1000); //(m/s)

    // Read in gyro data and compute PID outputs
    getAngles();
    velocity_PID.Compute();
    r_phi = PHI_SETPOINT_RATIO * u_xdot;
    angle_PID.Compute();
    turning_velocity_PID.Compute();

    // Print telemetry to serial
    // Serial.print("Motor forward input (PWM):");
    // Serial.print(-1 * u_phi / 4096);
    // Serial.print(", Current forward velocity (m/s):");
    // Serial.print(xdot);
    // Serial.print(", Current tilt angle (rad):");
    // Serial.print(phi);
    // Serial.print(", Tilt angle set point (rad):");
    // Serial.print(r_phi);
    // Serial.print(", Current turning velocity (rad/s):");
    // Serial.print(theta_dot);
    // Serial.print(", Motor turning velocity input:");
    // Serial.print(u_theta_dot);
    // Serial.println();
    
    // Drive motors using output from tilt angle and turning velocity PID loops
    // driveMotors(u_phi + u_theta_dot, u_phi - u_theta_dot);
  }

  // Read in and process commands from Bluetooth or serial controller
  if (SerialBT.available())
  {
    char b = SerialBT.read();
    SerialBT.print(b);
    processReceivedValue(b, command_BT);
  } 
  else if (Serial.available())
  {
    char b = Serial.read();
    Serial.print(b);
    processReceivedValue(b, command_serial);
  }
  else if (Serial2.available())
  {
    char b = Serial2.read();
    //Serial.print(b);
    processReceivedJetsonValue(b, command_2);
  }
}
