#include <Wire.h>
#include <ICM20948_WE.h>
#include <Adafruit_PWMServoDriver.h>
#include <ESP32Encoder.h>
#include <math.h>
#include <cmath>
// #include <ArduinoEigen.h>
#include <BasicLinearAlgebra.h>
#include <PID_v1.h>
#include <BluetoothSerial.h>
#include <ArduinoJson.h>
#include <cstdlib>

// using Eigen::Matrix;
// using Eigen::MatrixXd;
using namespace BLA;

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
#define MAX_VELOCITY 0.5       // Maximum magnitude of velocity PID output
#define MAX_PWM 4096           // Maximum magnitude of motor controller PID outputs
#define MAX_PHI 0.5            // Maximum angle command the robot can take, in radians, from 0.
#define PHI_SETPOINT_RATIO 1.0 // Ratio of setpoint of tilt controller to output of velocity controller

#define HIPS_COMMAND_MIN 0.0 // The maximum hips angle that can be commanded
#define HIPS_COMMAND_MAX 1.0 // The minimum hips angle that can be commanded

// Time constants
#define TIMER_INTERVAL_MS 25. // Interval between timer interrupts

#define GET_VARIABLE_NAME(Variable) = #Variable

// Robot state declarator
volatile bool currently_enabled= false;

// Serial controller definitions
BluetoothSerial SerialBT;
String command_BT = "";
String command_BT_new = "";
String command_serial = "";

const unsigned int MAX_COMMAND_LENGTH = 16;
// Structure for received messages:
typedef struct received_msg{
  unsigned short test;
};

const int received_msg_size = sizeof(received_msg);

typedef union packet{
  received_msg message;
  char receivedFromSerial[received_msg_size];
};

packet latest_command;
static char fullCommand[MAX_COMMAND_LENGTH];
static unsigned int serialIndex = 0;

// Encoder definitions
ESP32Encoder encoder_wheel_right; // Right wheel encoder
ESP32Encoder encoder_wheel_left;  // Left wheel encoder
ESP32Encoder encoder_hip_right;   // Right hip motor encoder
ESP32Encoder encoder_hip_left;    // Left hip motor encoder

// Wheel Encoder Counts:
volatile int count_r_total = 0; // Right wheel total encoder count
volatile int count_l_total = 0; // Left wheel total encoder count
volatile int count_r = 0;       // Right wheel intermediate encoder count
volatile int count_l = 0;       // Left wheel intermediate encoder count

// Hip Encoder Counts:
volatile int count_rh = 0;      // Right Hip Encoder Counts
volatile int count_lh = 0;      // Left hip encoder counts
volatile int count_rh_total = 0;      // Right hip encoder counts
volatile int count_lh_total = 0;      // Left hip encoder counts

// PID Constants for wheel velocity (loop one)
double Kp_xdot = 0.030;
double Ki_xdot = 0.3;
double Kd_xdot = 0.002;

double r_xdot = 0; // Commanded wheel speed in m/s
double u_xdot = 0; // Output of velocity PID, added to balancing loop to command velocity robot motion
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

// PID Constants for Phidot
double phidot = 0;  // Rate of change of phi from gyro only
double r_phidot = 0; // commanded phidot angle. This should pretty much always be zero (balanced).
double Kp_phidot = 1;
double Ki_phidot = 1;
double Kd_phidot = 0.1;

// PID constants for turning velocity
double Kp_theta_dot = 200;
double Ki_theta_dot = 150;
double Kd_theta_dot = 5;

double r_theta_dot = 0; // Commanded turning velocity in rad/s
double u_theta_dot = 0; // Output of turning velocity PID loop
double theta_dot = 0;   // Current turning velocity

// PID constants for hips
double Kp_hips = 10;
double Ki_hips = 1;
double Kd_hips = 1;

double r_hips = 0; // Commanded hip angle in radians
double u_hips = 0; // Output of hip angle PID loop
double hips = 0;   // Current hip angle (nominal angle, for both, to specify height)
double r_hip_angle = 0;   // Current right hip angle
double l_hip_angle = 0;   // Current left hip angle

// PID constants for gamma_body
double Kp_gamma = 10;
double Ki_gamma = 1;
double Kd_gamma = 1;

double r_gamma = 0; // Commanded body roll angle in radians
double u_gamma = 0; // Output of body roll angle PID loop. Added to hip motor drive command. 
double gamma_body = 0;   // Current hip angle (nominal angle, for both, to specify height)

// PID constants for neck
double Kp_neck = 10;
double Ki_neck = 1;
double Kd_neck = 1;

double r_neck = 0; // Commanded neck angle in radians
double u_neck = 0; // Output of neck angle PID loop. Composes neck motor drive command. 
double neck = 0;   // Current neck angle (nominal angle, for both, to specify height)

double NECK_COMMAND_MIN = 0.0; // The maximum neck angle that can be commanded
double NECK_COMMAND_MAX = 10.0; // The minimum neck angle that can be commanded

PID velocity_PID(&xdot, &u_xdot, &r_xdot, Kp_xdot, Ki_xdot, Kd_xdot, DIRECT);
PID phi_PID(&phi, &u_phi, &r_phi, Kp_phi, Ki_phi, Kd_phi, DIRECT);
PID phidot_PID(&phidot,&r_phi,&r_phidot, Kp_phidot, Ki_phidot, Kd_phidot, DIRECT);
PID turning_velocity_PID(&theta_dot, &u_theta_dot, &r_theta_dot, Kp_theta_dot, Ki_theta_dot, Kd_theta_dot, DIRECT);
PID hips_PID(&hips, &u_hips, &r_hips, Kp_hips, Ki_hips, Kd_hips, DIRECT);
PID gamma_PID(&gamma_body, &u_gamma, &r_gamma, Kp_gamma, Ki_gamma, Kd_gamma, DIRECT);
PID neck_PID(&neck, &u_neck, &r_neck, Kp_neck, Ki_neck, Kd_neck, DIRECT);

// Hardware timer definitions
hw_timer_t *timer0;
portMUX_TYPE timerMux0 = portMUX_INITIALIZER_UNLOCKED;
volatile bool interrupt_complete = false; // Flag set upon interrupt, reset during main loop
int timesteps_passed = 0;                 // Interrupts completed since start of execution

// IMU definitions
ICM20948_WE IMU = ICM20948_WE(ICM20948_ADDR);

// Accelerometer definitions
BLA::Matrix<3> ACC_VERTICAL_VEC; // Vertical vector from accelerometer readings while robot is held upright
BLA::Matrix<3> acc_vec;          // Current acceleration vector from accelerometer
BLA::Matrix<3> acc_normal_vec;   // Normal vector to current acceleration and vertical acceleration vectors

// dc_pwm definitions
Adafruit_PWMServoDriver dc_pwm = Adafruit_PWMServoDriver();
Adafruit_PWMServoDriver servo_pwm = Adafruit_PWMServoDriver();
int pulse_length_grasper = 0;
int grasper_pulse_max = 300; // absolute max servo will respond to is [70,500]
int grasper_pulse_min = 70;

int pulse_length_neck = 0;
int neck_pulse_max = 300; // Untested neck servo! #TODO: Calibrate, these are guessed values
int neck_pulse_min = 70;

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
  encoder_wheel_left.clearCount();
  encoder_wheel_right.clearCount();

  count_rh = -1*encoder_hip_right.getCount();
  count_lh = encoder_hip_left.getCount();
  count_rh_total += count_rh;
  count_lh_total += count_lh;
  encoder_hip_right.clearCount();
  encoder_hip_left.clearCount();
  interrupt_complete = true;
  portEXIT_CRITICAL_ISR(&timerMux0);
}

void enable(){
  Serial.println("Enabling...");
  currently_enabled = true;
}
void disable(){
  Serial.println("Disabling...");
  currently_enabled = false;
}

// Drive motors at desired dc_pwm
void driveWheelMotors(float u_l, float u_r)
{
  // Constrain motor input
  u_l = constrain(u_l, -MAX_PWM, MAX_PWM);
  u_r = constrain(u_r, -MAX_PWM, MAX_PWM);

  // Set motor pins based on drive direction
  if(currently_enabled){// switch back to currently_enabled when finished with hip motor testing, or false when wheels are disabled
    if (u_l >= 0)
    {
      dc_pwm.setPin(WHEEL_MOTOR_LEFT_A, u_l, 0);
      dc_pwm.setPin(WHEEL_MOTOR_LEFT_B, 0, 0);
    }
    else
    {
      dc_pwm.setPin(WHEEL_MOTOR_LEFT_A, 0, 0);
      dc_pwm.setPin(WHEEL_MOTOR_LEFT_B, -u_l, 0);
    }

    if (u_r >= 0)
    {
      dc_pwm.setPin(WHEEL_MOTOR_RIGHT_A, 0, 0);
      dc_pwm.setPin(WHEEL_MOTOR_RIGHT_B, u_r, 0);
    }
    else
    {
      dc_pwm.setPin(WHEEL_MOTOR_RIGHT_A, -u_r, 0);
      dc_pwm.setPin(WHEEL_MOTOR_RIGHT_B, 0, 0);
    }
  }
  else{
    // Zero all motors:
    // Motor 1 (Left):
    dc_pwm.setPin(WHEEL_MOTOR_LEFT_A, 0, 0);
    dc_pwm.setPin(WHEEL_MOTOR_LEFT_B, 0, 0);
    // Motor 2 (Right):
    dc_pwm.setPin(WHEEL_MOTOR_RIGHT_A, 0, 0);
    dc_pwm.setPin(WHEEL_MOTOR_RIGHT_B, 0, 0);
  }
}
void driveJointMotors(float u_lh, float u_rh, float u_neck)
{
  // Constrain motor input
  u_lh = constrain(u_lh, -MAX_PWM, MAX_PWM);
  u_rh = constrain(u_rh, -MAX_PWM, MAX_PWM);
  u_neck = constrain(u_neck, -MAX_PWM, MAX_PWM);
  // Set motor pins based on drive direction
  if(currently_enabled){
    if (u_lh >= 0)
    {
      dc_pwm.setPin(HIP_MOTOR_LEFT_A, u_lh, 0);
      dc_pwm.setPin(HIP_MOTOR_LEFT_B, 0, 0);
    }
    else
    {
      dc_pwm.setPin(HIP_MOTOR_LEFT_A, 0, 0);
      dc_pwm.setPin(HIP_MOTOR_LEFT_B, -u_lh, 0);
    }

    if (u_rh >= 0)
    {
      dc_pwm.setPin(HIP_MOTOR_RIGHT_A, 0, 0);
      dc_pwm.setPin(HIP_MOTOR_RIGHT_B, u_rh, 0);
    }
    else
    {
      dc_pwm.setPin(WHEEL_MOTOR_RIGHT_A, -u_rh, 0);
      dc_pwm.setPin(WHEEL_MOTOR_RIGHT_B, 0, 0);
    }

    if (u_neck >= 0)
    {
      dc_pwm.setPin(NECK_MOTOR_A, 0, 0);
      dc_pwm.setPin(NECK_MOTOR_B, u_neck, 0);
    }
    else
    {
      dc_pwm.setPin(NECK_MOTOR_A, -u_neck, 0);
      dc_pwm.setPin(NECK_MOTOR_B, 0, 0);
    }
  }
  else{
    // Zero all motors:
    // Motor 3 (Left Hip):
    dc_pwm.setPin(HIP_MOTOR_LEFT_A, 0, 0);
    dc_pwm.setPin(HIP_MOTOR_LEFT_B, 0, 0);
    // Motor 4 (Right Hip):
    dc_pwm.setPin(HIP_MOTOR_RIGHT_A, 0, 0);
    dc_pwm.setPin(HIP_MOTOR_RIGHT_B, 0, 0);
    // Motor 5 (Lower Neck):
    dc_pwm.setPin(NECK_MOTOR_A, 0, 0);
    dc_pwm.setPin(NECK_MOTOR_B, 0, 0);
  }
}

// Drive Servos at desired angles
void driveServos(float u_grasper, float u_neck)
{
  // Takes in u_grasper, proportion between 0 and 1 of how closed the servo should be. 0 is open and 1 is fully closed. Can overactuate open.
  pulse_length_grasper = grasper_pulse_min+int(u_grasper*(grasper_pulse_max-grasper_pulse_min));
  // Takes in u_neck, representing an angle command for the neck relative to its' zero position. 
  // TODO: Calibrate once robot is more assembled.
  pulse_length_neck = neck_pulse_min+int(u_neck*(neck_pulse_max-neck_pulse_min));
  if(currently_enabled){
    servo_pwm.setPWM(GRASPER_SERVO,0,pulse_length_grasper);
    servo_pwm.setPWM(NECK_SERVO,0,pulse_length_neck);
  }
  else{
    // Grasper (SRV_2)
    servo_pwm.setPWM(GRASPER_SERVO,0,0); // Hopefully this just leaves free rotation an option! Investigate
    // Nodding (SRV_1) 
    servo_pwm.setPWM(NECK_SERVO,0,0);
  }
}

BLA::Matrix<3> crossProduct(BLA::Matrix<3> u, BLA::Matrix<3> v){
  // Return cross product in vector form:
  BLA::Matrix<3> w;
  w = { u(1)*v(2)-u(2)*v(1),
        u(2)*v(0)-u(0)*v(2),
        u(0)*v(1)-u(1)*v(0)};
  return w;
}
float dotProduct(BLA::Matrix<3> u, BLA::Matrix<3> v){
  // Return dot product in float form:
  BLA::Matrix<1> w;
  w = ~u*v;
  return w(0);
}
BLA::Matrix<3> normalize(BLA::Matrix<3> u){
  // Normalize a 3x1 column vector:
  double mag = sqrt(pow(u(0),2)+pow(u(1),2)+pow(u(2),2));
  BLA::Matrix<3> w = {u(0)/mag, u(1)/mag, u(2)/mag};
  return w;
}

// Synthesize and filter gyro and accelerometer readings to get accurate angle measurements
void getAngles()
{
  // Read IMU
  IMU.readSensor();
  xyzFloat acc = IMU.getGValues();   // Acceleration vector in m/s
  xyzFloat gyr = IMU.getGyrValues(); // Angular velocity vector in deg/s

  // Calculate tilt angle from gyro data
  phi = phi - gyr.y * TIMER_INTERVAL_MS / 1000. * (PI / 180.); // Integrate gyro angular velocity about the y-axis to get tilt
  theta_dot = gyr.z * TIMER_INTERVAL_MS / 1000. * (PI / 180.);
  phidot = -gyr.y * (PI / 180.); // Angular velocity as detected by gyro, no filtering
  // Calculate tilt angle from accelerometer data
  acc_vec = {acc.x, acc.y, acc.z};
  phi_acc = atan2(dotProduct((crossProduct(ACC_VERTICAL_VEC,acc_vec)),(acc_normal_vec)), dotProduct(acc_vec,ACC_VERTICAL_VEC));

  // Complementary filter
  phi = CF_TIME_CONSTANT * phi + (1 - CF_TIME_CONSTANT) * phi_acc;
}

// Initialize velocity PID loop
void init_velocity_PID()
{
  velocity_PID.SetSampleTime((int)(TIMER_INTERVAL_MS));
  velocity_PID.SetOutputLimits(-MAX_VELOCITY, MAX_VELOCITY);
  velocity_PID.SetMode(AUTOMATIC);
}

// Initialize angle PID loop
void init_phi_PID()
{
  phi_PID.SetSampleTime(TIMER_INTERVAL_MS);
  phi_PID.SetOutputLimits(-MAX_PWM, MAX_PWM);
  phi_PID.SetMode(AUTOMATIC);
}
void init_phidot_PID()
{
  phidot_PID.SetSampleTime(TIMER_INTERVAL_MS);
  phidot_PID.SetOutputLimits(-MAX_PHI, MAX_PHI);
  phidot_PID.SetMode(AUTOMATIC);
}
// Initialize turning velocity PID loop
void init_turning_velocity_PID()
{
  turning_velocity_PID.SetSampleTime(TIMER_INTERVAL_MS);
  turning_velocity_PID.SetOutputLimits(-MAX_PWM, MAX_PWM);
  turning_velocity_PID.SetMode(AUTOMATIC);
}
void init_hips_PID()
{
  hips_PID.SetSampleTime(TIMER_INTERVAL_MS);
  hips_PID.SetOutputLimits(-MAX_PWM, MAX_PWM);
  hips_PID.SetMode(AUTOMATIC);
}
void init_gamma_PID()
{
  gamma_PID.SetSampleTime(TIMER_INTERVAL_MS);
  gamma_PID.SetOutputLimits(-MAX_PWM, MAX_PWM);
  gamma_PID.SetMode(AUTOMATIC);
}
void init_neck_PID()
{
  neck_PID.SetSampleTime(TIMER_INTERVAL_MS);
  neck_PID.SetOutputLimits(-MAX_PWM, MAX_PWM);
  neck_PID.SetMode(AUTOMATIC);
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
  //sprintf(buffer, "Setting velocity: %6f.", r_xdot);
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
  //sprintf(buffer, "Setting theta_dot: %6f.", r_theta_dot);
  //Serial.println(buffer);
} 
void set_neck_command(JsonArray arguments)
{
  if (arguments.size() != 1)
  {
    Serial.println("Incorrect number of arguments for setting neck angle");
    return;
  }
  float neck_input = arguments[0];
  r_neck += neck_input;
  if ((r_neck) > NECK_COMMAND_MAX){
    r_neck = NECK_COMMAND_MAX;
  }
  else if (r_neck<NECK_COMMAND_MIN)
  {
    r_neck = NECK_COMMAND_MIN;
    return;
  }
  char buffer[40];
  sprintf(buffer, "Setting r_neck: %6f.", r_neck);
  //Serial.println(buffer);
}
void set_hips_command(JsonArray arguments)
{
  if (arguments.size() != 1)
  {
    Serial.println("Incorrect number of arguments for setting hips angle");
    return;
  }
  float hips_input = arguments[0];
  r_hips += hips_input;
  if ((r_hips) > HIPS_COMMAND_MAX){
    r_hips = HIPS_COMMAND_MAX;
  }
  else if (r_hips<HIPS_COMMAND_MIN)
  {
    r_hips = HIPS_COMMAND_MIN;
    return;
  }
  char buffer[40];
  sprintf(buffer, "Setting r_hips: %6f.", r_hips);
  //Serial.println(buffer);
}


// Set PID Constants:
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
  //Serial.println(buffer);
}
void set_phi_pid_constants(JsonArray arguments)
{
  if (arguments.size() != 3)
  {
    Serial.println("Incorrect number of arguments for setting angle PID constants");
    return;
  }
  Kp_phi = arguments[0];
  Ki_phi = arguments[1];
  Kd_phi = arguments[2];
  phi_PID.SetTunings(Kp_phi,Ki_phi,Kd_phi);
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
  turning_velocity_PID.SetTunings(Kp_theta_dot,Ki_theta_dot,Kd_theta_dot);

  sprintf(buffer, "Setting Kp_theta_dot = %6f, Ki_theta_dot = %6f, Kd_theta_dot = %6f.", Kp_theta_dot, Ki_theta_dot, Kd_theta_dot);
  Serial.println(buffer);
}
void set_hips_pid_constants(JsonArray arguments)
{
  if (arguments.size() != 3)
  {
    Serial.println("Incorrect number of arguments for setting turning velocity PID constants");
    return;
  }
  Kp_hips = arguments[0];
  Ki_hips = arguments[1];
  Kd_hips = arguments[2];
  char buffer[100];
  hips_PID.SetTunings(Kp_hips,Ki_hips,Kd_hips);

  sprintf(buffer, "Setting Kp_hips = %6f, Ki_hips = %6f, Kd_hips = %6f.", Kp_hips, Ki_hips, Kd_hips);
  Serial.println(buffer);
}
void set_gamma_pid_constants(JsonArray arguments)
{
  if (arguments.size() != 3)
  {
    Serial.println("Incorrect number of arguments for setting turning velocity PID constants");
    return;
  }
  Kp_gamma = arguments[0];
  Ki_gamma = arguments[1];
  Kd_gamma = arguments[2];
  char buffer[100];
  gamma_PID.SetTunings(Kp_gamma,Ki_gamma,Kd_gamma);

  sprintf(buffer, "Setting Kp_gamma = %6f, Ki_gamma = %6f, Kd_gamma = %6f.", Kp_gamma, Ki_gamma, Kd_gamma);
  Serial.println(buffer);
}
void set_neck_pid_constants(JsonArray arguments)
{
  if (arguments.size() != 3)
  {
    Serial.println("Incorrect number of arguments for setting turning velocity PID constants");
    return;
  }
  Kp_neck = arguments[0];
  Ki_neck = arguments[1];
  Kd_neck = arguments[2];
  char buffer[100];
  neck_PID.SetTunings(Kp_neck,Ki_neck,Kd_neck);

  sprintf(buffer, "Setting Kp_neck = %6f, Ki_neck = %6f, Kd_neck = %6f.", Kp_neck, Ki_neck, Kd_neck);
  Serial.println(buffer);
}
// Improved Serial Comms:
void set_pid_constants(PID &loop, float kp, float ki, float kd){
  // set the PID constants of the PID loop specified
  loop.SetTunings(Kp_xdot,Ki_xdot,Kd_xdot);
  char buffer[100];
  sprintf(buffer, "Setting Kp= %6f, Ki = %6f, Kd = %6f.", Kp_xdot, Ki_xdot, Kd_xdot);
}

void processReceivedValue(char b, String &command)
{
  // Process JSON commands sent over Serial
  //
  // OPCODES
  // 0 - set_velocity(double velocity)
  // 1 - set_theta_dot(double theta_dot)
  // 2 - set_velocity_pid_constants(double Kp, double Ki, double Kd)
  // 3 - set_phi_pid_constants(double Kp, double Ki, double Kd)
  // 4 - set_theta_dot_pid_constants(double Kp, double Ki, double Kd)
  // 5 - enable robot
  // 6 - disable robot
  // 7 - set_hips_pid
  // 8 - set_gamma_pid
  // 9 - set_neck_pid_constants
  // 10 - set_neck_command
  // 11 - set_hips_command 
  if (b == DELIMITER)
  {
    DynamicJsonDocument doc(1024);
    deserializeJson(doc, command);
    char opcode = doc["cmd"];
    JsonArray arguments = doc["args"];
    Serial.println("Opc:"+opcode);
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
        set_phi_pid_constants(arguments);
        break;
      case 4:
        set_turning_velocity_pid_constants(arguments);
        break;
      case 5:
        enable();
        break;
      case 6:
        disable();
        break;
      case 7:
        set_hips_pid_constants(arguments);
        break;
      case 8: 
        set_gamma_pid_constants(arguments);
        break;
      case 9: 
        set_neck_pid_constants(arguments);
        break;
      case 10:
        set_neck_command(arguments);
        break;
      case 11:
        set_hips_command(arguments);
        break;
      }
    }
    else
    {
      //Serial.println("Opcode or arguments not passed in");
    }
    command = "";
  }
  else
  {
    command.concat(b);
  }

  return;
}

void processSerialCommand(char b)
{
  // 
  // //**************///
  
  // Decodes an incoming serial command and calls a number of functions.
  
  //Command format:
  //  Char 0: Message code
  //    If 0: Disable
  //    If 1: Enable - Teleop / standby
  //      Message type 1: command
  //    If 2: Enable - Autonomous
  //    If 3: Set tuning values
  //      Message type 2: PID input
  //    Else: Throw error? 
  
  //    Command message type:
  //      Byte 0: message code
  //      Byte 1-3: 
     
  //    Set Tunings message type:
  //      Byte 0: message code
  //      Byte 1: loop to tune
  //      Byte 2-5: new Kp
  //      Byte 6-9: new Ki
  //      Byte 10-13: new Kd
  //      Byte 14: delimiter

  if (b != DELIMITER && serialIndex<(MAX_COMMAND_LENGTH-1)) // command is not yet finished
  {
    fullCommand[serialIndex] = b;
    serialIndex++;
    
  }
  else // Full command received
  {
    // // Do what we want with the data. 
    // // Decode message:
    // char message_type = fullCommand[0];
    // switch(message_type){
    //   case '0': // message is a disable command
    //     disable();
    //   case '1':
    //     // message is an enable type for standby:
    //     enable();
    //     // process remaining chars into a struct and act on the command
    //     // remaining chars:
    //     // 
    //   case '2':
    //     // command is an autonomous enable command
    //     // TODO
    //   case '3':
    //     // Command will set tuning values

    //     // Remaining data will be: 
    //     // cm2[1] = loop_id
    //     // cm2[2-5]   = kp
    //     // cm2[6-9]   = ki
    //     // cm2[10-13] = kd
    //     char loop_id = fullCommand[1];
    //     float kp_new = bytes2float((fullCommand.substring(2,5)));
    //     float ki_new = bytes2float((fullCommand.substring(6,9));
    //     float kd_new = bytes2float((fullCommand.substring(10,13));
    //     set_pid_constants(loop_id, kp_new, ki_new, kd_new);
    // }
    fullCommand[serialIndex]='\0'; // terminate with escape character to make printable string. Do we need to do this? 

    Serial.print("fullCommand:");
    Serial.println(fullCommand);
    latest_command.receivedFromSerial[0]=fullCommand[0];
    latest_command.receivedFromSerial[1]=fullCommand[1];

    Serial.println("Received:"+latest_command.message.test);

    serialIndex=0;
  }
  // Serial2.print("Message sent from ESP32|");
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

void setup()
{
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
  ACC_VERTICAL_VEC = {0.20, -0.03, -0.98};
  acc_vec = {acc.x, acc.y, acc.z};

  acc_normal_vec = normalize(crossProduct(ACC_VERTICAL_VEC,acc_vec));

  // Reverse normal vector if needed
  if (acc_normal_vec(1) < 0)
  {
    acc_normal_vec = acc_normal_vec * -1;
  }

  phi = atan2(dotProduct(crossProduct(ACC_VERTICAL_VEC,acc_vec),acc_normal_vec), dotProduct(acc_vec,ACC_VERTICAL_VEC));

  phi_acc = phi; // Both phi values initially come from accelerometer

  // Initialize encoders
  ESP32Encoder::useInternalWeakPullResistors = UP;
  encoder_wheel_right.attachHalfQuad(33, 4); // ENC 
  encoder_wheel_left.attachHalfQuad(5, 13); // ENC 2A,B
  encoder_wheel_right.clearCount();
  encoder_wheel_left.clearCount();

  encoder_hip_right.attachHalfQuad(14, 15);
  encoder_hip_left.attachHalfQuad(18, 19);
  encoder_hip_right.clearCount();
  encoder_hip_left.clearCount();

  dc_pwm.begin();
  dc_pwm.setOscillatorFrequency(27000000);
  dc_pwm.setPWMFreq(440);

  servo_pwm.begin();
  servo_pwm.setOscillatorFrequency(27000000);
  servo_pwm.setPWMFreq(50);

  pinMode(OCM_1, INPUT);
  pinMode(OCM_2, INPUT);
  pinMode(OCM_HIP_LEFT, INPUT);
  pinMode(OCM_HIP_RIGHT, INPUT);

  // Motor 1 (Left):
  dc_pwm.setPin(WHEEL_MOTOR_LEFT_A, 0, 0);
  dc_pwm.setPin(WHEEL_MOTOR_LEFT_B, 0, 0);
  // Motor 2 (Right):
  dc_pwm.setPin(WHEEL_MOTOR_RIGHT_A, 0, 0);
  dc_pwm.setPin(WHEEL_MOTOR_RIGHT_B, 0, 0);

  // Motor 3 (Left Hip):
  dc_pwm.setPin(HIP_MOTOR_LEFT_A, 0, 0);
  dc_pwm.setPin(HIP_MOTOR_LEFT_B, 0, 0);
  // Motor 4 (Right Hip):
  dc_pwm.setPin(HIP_MOTOR_RIGHT_A, 0, 0);
  dc_pwm.setPin(HIP_MOTOR_RIGHT_B, 0, 0);

  // Motor 5 (Lower Neck):
  dc_pwm.setPin(NECK_MOTOR_A, 0, 0);
  dc_pwm.setPin(NECK_MOTOR_B, 0, 0);

  // Nodding (SRV_1)
  servo_pwm.setPWM(NECK_SERVO, 0, 0);
  // Grasper (SRV_2)
  servo_pwm.setPWM(GRASPER_SERVO, 0, 0);

  timer0 = timerBegin(0, 80, true);                           // timer 0, MWDT clock period = 12.5 ns * TIMGn_Tx_WDT_CLK_PRESCALE -> 12.5 ns * 80 -> 1000 ns = 1 us, countUp
  timerAttachInterrupt(timer0, &timerInterruptHandler, true); // edge (not level) triggered
  timerAlarmWrite(timer0, TIMER_INTERVAL_MS * 1000, true);    // 10000 * 1 us = 10 ms, autoreload true
  timerAlarmEnable(timer0);                                   // enable
  delay(2000);

  init_phi_PID();
  init_phidot_PID();
  init_velocity_PID();
  init_turning_velocity_PID();
  init_hips_PID();
  init_gamma_PID();
  // init_neck_PID();

  // Serial.println();
}

void loop()
{
  if (interrupt_complete){
    // ensuring reset isn't skipped:
    portENTER_CRITICAL(&timerMux0);
    interrupt_complete = false;
    portEXIT_CRITICAL(&timerMux0);

    // Calculate velocity from encoder readings
    xdot_l = count_l * 4 * PI * WHEEL_RADIUS / (2248.86 * TIMER_INTERVAL_MS / 1000.);
    xdot_r = count_r * 4 * PI * WHEEL_RADIUS / (2248.86 * TIMER_INTERVAL_MS / 1000.);
    xdot = ((count_l + count_r) / 2) * 4 * 3.14159265 * WHEEL_RADIUS / (2248.86 * TIMER_INTERVAL_MS / 1000); //(m/s)
    
    // Calculate current hip angles from encoders
    l_hip_angle = 2*count_lh_total*2*3.14159265*24 /(8400*28); // 8200 counts per revolution
    r_hip_angle = 2*count_rh_total*2*3.14159265*24 /(8400*28); // 8200 counts per revolution
    hips = (r_hip_angle + l_hip_angle)/2;

    // Read in gyro data and compute PID outputs
    getAngles();
    velocity_PID.Compute();
    phidot_PID.Compute();
    phi_PID.Compute();
    turning_velocity_PID.Compute();
    hips_PID.Compute();

    // Print telemetry to serial
    // Serial.print("Motor forward input (dc_pwm):");
    // Serial.print(-1 * u_phi / 4096);
    // Serial.println(">u_phi:"+String(u_phi));
    // Serial.print(", Current forward velocity (m/s):");
    // Serial.println(">x_dot:"+String(xdot));
    // Serial.println(">r_xdt:"+String(r_xdot));
    // Serial.print(xdot);
    // Serial.print(", Current tilt angle (rad):");
    // Serial.println(">phi:"+String(phi));
    // Serial.println(">phidot:"+String(phidot));
    // Serial.print(", Tilt angle set point (rad):");
    // Serial.println(">r_phi:"+String(r_phi));
    
    // Serial.print(r_phi);
    // Serial.print(", Current turning velocity (rad/s):");
    // Serial.print(theta_dot);
    // Serial.print(", Motor turning velocity input:");
    // Serial.println(">u_thd:"+String(u_theta_dot));
    //Serial.println();
    // Serial.println(">hips:"+String(hips));
    // Serial.println(">r_hips:"+String(r_hips));
    // Serial.println(">u_hips:"+String(u_hips));
    // Serial.println(">u_gamma:"+String(u_gamma));
    // Serial.println(">r_hips:"+String(r_hips));
    // Drive motors using output from tilt angle and turning velocity PID loops
    driveWheelMotors(u_phi + u_theta_dot, u_phi - u_theta_dot);
    driveJointMotors(u_hips+u_gamma,u_hips-u_gamma,0); // last term should be u_neck
    driveServos(0,0);

  }

  // Read in and process commands from Bluetooth or serial controller
  if (SerialBT.available())
  {
    char b = SerialBT.read();
    Serial.println("SerialBT received byte:"+b);
    //processReceivedValue(b, command_BT);
    processSerialCommand(b);
  } 
  if (Serial2.available())
  {
    char b = Serial2.read();
    processSerialCommand(b);
  }
}
