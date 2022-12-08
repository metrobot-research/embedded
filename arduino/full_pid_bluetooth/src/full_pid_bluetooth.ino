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

// Motor controller 1 - Rightmost when robot facing forward (stage right)
#define WHEEL_RIGHT_A 0 // Motor 1 pin A
#define WHEEL_RIGHT_B 1 // Motor 1 pin B
#define WHEEL_RIGHT_ENC_A 33// Enc 1A
#define WHEEL_RIGHT_ENC_B 4 // Enc 1B
#define WHEEL_RIGHT_OCM 36

// Motor controller 2 - 2nd from stage rightmost
#define NECK_MOTOR_A 2  // Motor 2 pin A
#define NECK_MOTOR_B 3  // Motor 2 pin B
#define NECK_MOTOR_ENC_A 5  // Enc 2A
#define NECK_MOTOR_ENC_B 13 // Enc 2B
#define NECK_MOTOR_OCM 39

// Motor controller 3 - 3rd from stage rightmost
#define HIP_RIGHT_A 4   // Motor 3 pin A
#define HIP_RIGHT_B 5   // Motor 3 pin B
#define HIP_RIGHT_ENC_A 14  // Enc 3A
#define HIP_RIGHT_ENC_B 15  // Enc 3B
#define HIP_RIGHT_OCM 34      // Right hip motor current sensor

// Motor controller 4 - 4th from stage rightmost
#define HIP_LEFT_A 6    // Left hip motor pin A
#define HIP_LEFT_B 7    // Left hip motor pin B
#define HIP_LEFT_ENC_A 18   // Enc 4A
#define HIP_LEFT_ENC_B 19   // Enc 4B
#define HIP_LEFT_OCM 35       // Left hip motor current sensor

// Motor controller 5 - Stage Leftmost 
#define WHEEL_LEFT_A 8        // Motor pin 5A
#define WHEEL_LEFT_B 9        // Motor pin 5B
#define WHEEL_LEFT_ENC_A 23   // ENC 5A
#define WHEEL_LEFT_ENC_B 26   // ENC 5B


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
#define MAX_PHI 1            // Maximum angle command the robot can take, in radians, from 0.
#define PHI_SETPOINT_RATIO 1.0 // Ratio of setpoint of tilt controller to output of velocity controller

#define HIPS_COMMAND_MIN -.7 // The maximum hips angle that can be commanded
#define HIPS_COMMAND_MAX 0 // The minimum hips angle that can be commanded

// Time constants
#define TIMER_INTERVAL_MS 25. // Interval between timer interrupts

#define GET_VARIABLE_NAME(Variable) = #Variable

#pragma region 

// Robot state declarator
volatile bool currently_enabled= false;

// Serial controller definitions
BluetoothSerial SerialBT;
String command_BT = "";

const unsigned int MAX_COMMAND_LENGTH = 24;
// Structure for received messages:
typedef struct receivedStateCmd{
  char state;                       // Set the robot's state: enabled or disabled
  char furtherState;                // Not used yet, placeholder so that the message does not have unpredicted end behavior
  unsigned short lowerNeckPosition; // Commanded lower neck position  
  
  float xdot_cmd;         // Commanded forward velocity
  float tdot_cmd;                 // Commanded angular velocity
  
  float upperNeckVelocity;          // Commanded upper neck velocity
  unsigned short hipAngle;          // Commanded hip angle
  unsigned short grasperVelocity;      // Commanded grasper angle
  
};

const int receivedStateCmdSize = sizeof(receivedStateCmd);

typedef union receivedPacket{
  receivedStateCmd message;
  char receivedFromSerial[receivedStateCmdSize];
};

/* Structure for interpreting PID constant change command, for live tuning:
Contents:
  - char case
  - char loopid
  - double Kp
  - double Ki
  - double Kd

*/
typedef struct receivedPIDCmd{
  char state;
  char loopId;
  float Kp_cmd;
  float Ki_cmd;
  float Kd_cmd;


};

/* Contents
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
  */
typedef struct sentMsg{
  
  float acc_x;
  float acc_y;
  float acc_z;
  
  float gyr_x;
  float gyr_y;
  float gyr_z;
  
  float phi;                        // Current robot tilt angle
  
  float lw_angvel;                  // Current angular velocity of left wheel
  float rw_angvel;                  // Current angular velocity of right wheel
  
  float lh_ang;                     // Current encoder measurement of left hip angle
  float rh_ang;                     // Current encoder measurement of left hip angle

  float neck_ang;
  
  unsigned short headAngle;         // Current head angle, unsigned short from 0-65535, mappable to actual value

  unsigned short grasperAngle;      // Current commanded grasper state
  char unused1;
  char unused2;
  char delimiter;
  char delimiter2;
};

const int sentMsgSize = sizeof(sentMsg);

typedef union sentPacket{
  sentMsg message;
  char byteArray[sentMsgSize];
};

static receivedPacket latest_command;
static sentPacket latest_state;
static char fullCommand[MAX_COMMAND_LENGTH];
static unsigned int serialIndex = 0;

// Encoder definitions
ESP32Encoder encoder_wheel_right; // Right wheel encoder
ESP32Encoder encoder_wheel_left;  // Left wheel encoder
ESP32Encoder encoder_hip_right;   // Right hip motor encoder
ESP32Encoder encoder_hip_left;    // Left hip motor encoder
ESP32Encoder encoder_neck;    // Left hip motor encoder

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

volatile int count_neck = 0;
volatile int count_neck_total = 0;

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

double delta_phi = 0; // Command that should be added to r_phi
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
double neck_angle = 0;   // Current neck angle (nominal angle, for both, to specify height)

double NECK_COMMAND_MIN = 0.0; // The maximum neck angle that can be commanded
double NECK_COMMAND_MAX = 10.0; // The minimum neck angle that can be commanded

// PID Loops:
// Phidot: tracks falling over rate of change to 0. Output: commanded angle for robot to take.
// PID phidot_PID(&phidot,&delta_phi,&r_phidot, Kp_phidot, Ki_phidot, Kd_phidot, DIRECT);
// Phi: commands wheels with u_phi to track the commanded angle which is an output of the above loop.
PID phi_PID(&phi, &u_phi, &r_phi, Kp_phi, Ki_phi, Kd_phi, DIRECT);
/* 
Velocity: commands wheels to have a set velocity. Should have slower response time than other loops to allow 
for simultaneous balancing and velocity control. Should also probably have negative Kp term & medium-high Ki term 
to allow for balancing.
*/
PID velocity_PID(&xdot, &u_xdot, &r_xdot, Kp_xdot, Ki_xdot, Kd_xdot, DIRECT);
PID turning_velocity_PID(&theta_dot, &u_theta_dot, &r_theta_dot, Kp_theta_dot, Ki_theta_dot, Kd_theta_dot, DIRECT);
PID hips_PID(&hips, &u_hips, &r_hips, Kp_hips, Ki_hips, Kd_hips, REVERSE);
PID gamma_PID(&gamma_body, &u_gamma, &r_gamma, Kp_gamma, Ki_gamma, Kd_gamma, DIRECT);
/*
Neck Angle: 
*/
PID neck_PID(&neck_angle, &u_neck, &r_neck, Kp_neck, Ki_neck, Kd_neck, DIRECT);

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
BLA::Matrix<3> gyr_vec;          // Current gyro vector from accelerometer
BLA::Matrix<3> acc_normal_vec;   // Normal vector to current acceleration and vertical acceleration vectors

// dc_pwm definitions
Adafruit_PWMServoDriver dc_pwm = Adafruit_PWMServoDriver();
Adafruit_PWMServoDriver servo_pwm = Adafruit_PWMServoDriver();
int pulse_length_grasper = 0;
int grasper_pulse_max = 368; // absolute max servo will respond to is [70,500]
int grasper_pulse_min = 96;
float u_grasper = 0.5; // current grasper position command, used for velocity control
float grasper_vel_cmd = 0;

int pulse_length_head = 0;
int head_pulse_max = 386; // Calibrated to install
int head_pulse_min = 100;
float u_head = 0.5; // current head motor position command, used for velocity control
float head_vel_cmd = 0;
#pragma endregion

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

  count_neck = encoder_neck.getCount();
  count_neck_total +=count_neck;
  encoder_neck.clearCount();
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
  driveWheelMotors(0,0);
  driveJointMotors(0,0,0);
  driveServos(0,0);
}

// Drive motors at desired dc_pwm (to be active, 0-4095)
void driveWheelMotors(float u_l, float u_r)
{
  // Constrain motor input
  u_l = constrain(u_l, -MAX_PWM, MAX_PWM);
  u_r = constrain(u_r, -MAX_PWM, MAX_PWM);

  // Set motor pins based on drive direction
  if(currently_enabled){
    if (u_l >= 0)
    {
      dc_pwm.setPin(WHEEL_LEFT_A, u_l, 0);
      dc_pwm.setPin(WHEEL_LEFT_B, 0, 0);
    }
    else
    {
      dc_pwm.setPin(WHEEL_LEFT_A, 0, 0);
      dc_pwm.setPin(WHEEL_LEFT_B, -u_l, 0);
    }

    if (u_r >= 0)
    {
      dc_pwm.setPin(WHEEL_RIGHT_A, 0, 0);
      dc_pwm.setPin(WHEEL_RIGHT_B, u_r, 0);
    }
    else
    {
      dc_pwm.setPin(WHEEL_RIGHT_A, -u_r, 0);
      dc_pwm.setPin(WHEEL_RIGHT_B, 0, 0);
    }
  }
  else{
    // Zero all motors:
    // Motor 1 (Left):
    dc_pwm.setPin(WHEEL_LEFT_A, 0, 0);
    dc_pwm.setPin(WHEEL_LEFT_B, 0, 0);
    // Motor 2 (Right):
    dc_pwm.setPin(WHEEL_RIGHT_A, 0, 0);
    dc_pwm.setPin(WHEEL_RIGHT_B, 0, 0);
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
      dc_pwm.setPin(HIP_LEFT_A, u_lh, 0);
      dc_pwm.setPin(HIP_LEFT_B, 0, 0);
    }
    else
    {
      dc_pwm.setPin(HIP_LEFT_A, 0, 0);
      dc_pwm.setPin(HIP_LEFT_B, -u_lh, 0);
    }

    if (u_rh >= 0)
    {
      dc_pwm.setPin(HIP_RIGHT_A, 0, 0);
      dc_pwm.setPin(HIP_RIGHT_B, u_rh, 0);
    }
    else
    {
      dc_pwm.setPin(WHEEL_RIGHT_A, -u_rh, 0);
      dc_pwm.setPin(WHEEL_RIGHT_B, 0, 0);
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
    dc_pwm.setPin(HIP_LEFT_A, 0, 0);
    dc_pwm.setPin(HIP_LEFT_B, 0, 0);
    // Motor 4 (Right Hip):
    dc_pwm.setPin(HIP_RIGHT_A, 0, 0);
    dc_pwm.setPin(HIP_RIGHT_B, 0, 0);
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
  pulse_length_grasper = constrain(pulse_length_grasper,grasper_pulse_min,grasper_pulse_max);
  // Takes in u_neck, representing an angle command for the neck relative to its' zero position. 
  // TODO: Calibrate once robot is more assembled.
  pulse_length_head = head_pulse_min+int(u_neck*(head_pulse_max-head_pulse_min));
  pulse_length_head = constrain(pulse_length_head,head_pulse_min,head_pulse_max);
  if(currently_enabled){
    servo_pwm.setPWM(GRASPER_SERVO,0,pulse_length_grasper);
    servo_pwm.setPWM(NECK_SERVO,0,pulse_length_head);
  }
  else{
    // Grasper (SRV_2)
    servo_pwm.setPWM(GRASPER_SERVO,0,0); // Hopefully this just leaves free rotation an option! Investigate
    // Nodding (SRV_1)
    servo_pwm.setPWM(NECK_SERVO,0,0);
  }
}

void driveServosVelocity(float grasper_vel_cmd, float neck_vel_cmd, float &u_grasper, float &u_neck)
{
  // TODO: Drive the neck motors with a velocity. 
  // At each timestep, increment u_neck by neck_vel_cmd*timestep_ms
  // and u_grasper by grasper_vel_cmd

  u_neck += neck_vel_cmd*TIMER_INTERVAL_MS/1000.;
  u_neck = constrain(u_neck,0,1);
  u_grasper += grasper_vel_cmd*TIMER_INTERVAL_MS/1000.;
  u_grasper = constrain(u_grasper,0,1);
}

float getHeadAngle(){
  // TODO: Return current head angle. Convert current Servo PWM to degrees. 
  return -1.;
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
  gyr_vec = {gyr.x, gyr.y, gyr.z};
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
// void init_phidot_PID()
// {
//   phidot_PID.SetSampleTime(TIMER_INTERVAL_MS);
//   phidot_PID.SetOutputLimits(-MAX_PHI, MAX_PHI);
//   phidot_PID.SetMode(AUTOMATIC);
// }
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
void set_head_vel_cmd(JsonArray arguments)
{
  if (arguments.size() != 1)
  {
    Serial.println("Incorrect number of arguments for setting head vel command");
    return;
  }
  head_vel_cmd = arguments[0];
  char buffer[40];
  sprintf(buffer, "Setting head_vel_cmd: %4f.", head_vel_cmd);
  //Serial.println(buffer);
}
void set_grasper_vel_cmd(JsonArray arguments)
{
  if (arguments.size() != 1)
  {
    Serial.println("Incorrect number of arguments for setting grasper vel command");
    return;
  }
  grasper_vel_cmd = arguments[0];
  char buffer[40];
  sprintf(buffer, "Setting grasper_vel_cmd: %4f.", grasper_vel_cmd);
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
  r_hips += 0.5*hips_input;
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
  Serial.println(buffer);
}

void jsonArgsToFloat(JsonArray arguments, double &kp, double &ki, double &kd){
  // Will set the kp, ki, and kd to be parsed from arguments
  if (arguments.size() != 3)
  {
    Serial.println("Incorrect number of arguments for setting velocity PID constants");
    return;
  }
  kp = arguments[0];
  ki = arguments[1];
  kd = arguments[2];
}
void jsonArgsToFloat(JsonArray arguments, double &r){
  // Will set the value of r to the value defined in the arguments[0]
  if (arguments.size() != 1)
  {
    Serial.println("Incorrect number of arguments for setting r");
    return;
  }
  r = arguments[0];
}

// Set PID Constants:
void set_pid_constants(PID &loop, float kp, float ki, float kd){
  // set the PID constants of the PID loop specified
  loop.SetTunings(kp,ki,kd);
  char buffer[100];
  sprintf(buffer, "Setting Kp= %6f, Ki = %6f, Kd = %6f.", kp, ki, kd);
}

void processReceivedBTValue(char b, String &command)
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
  // a - set_neck_command
  // b - set_hips_command 
  if (b == DELIMITER)
  {
    DynamicJsonDocument doc(1024);
    deserializeJson(doc, command);
    int opcode = doc["cmd"];
    JsonArray arguments = doc["args"];
    Serial.println("Opc:"+String(opcode));
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
      case 2: // Set velocity PID constants
        jsonArgsToFloat(arguments, Kp_xdot,Ki_xdot,Kd_xdot);
        set_pid_constants(velocity_PID,Kp_xdot,Ki_xdot,Kd_xdot);
        break;
      case 3: // Set phi PID constants
        jsonArgsToFloat(arguments, Kp_phi,Ki_phi,Kd_phi);
        set_pid_constants(phi_PID,Kp_phi,Ki_phi,Kd_phi);
        break;
      case 4: // Set turning velocity PID constants
        jsonArgsToFloat(arguments, Kp_theta_dot,Ki_theta_dot,Kd_theta_dot);
        set_pid_constants(turning_velocity_PID,Kp_theta_dot,Ki_theta_dot,Kd_theta_dot);
        break;
      case 5: // Enable
        enable();
        break;
      case 6: // Disable
        disable();
        break;
      case 7: // Set hips PID constants
        jsonArgsToFloat(arguments,Kp_hips,Ki_hips,Kd_hips);
        set_pid_constants(hips_PID, Kp_hips,Ki_hips,Kd_hips);
        break;
      case 8: // Set phidot PID constants
        jsonArgsToFloat(arguments,Kp_phidot,Ki_phidot,Kd_phidot);
        // set_pid_constants(phidot_PID,Kp_phidot,Ki_phidot,Kd_phidot);
        break;
      case 9: 
        jsonArgsToFloat(arguments,Kp_neck,Ki_neck,Kd_neck);
        set_pid_constants(neck_PID,Kp_neck,Ki_neck,Kd_neck);
        break;
      case 10: // set head command (velocity)
        set_head_vel_cmd(arguments);
        break;
      case 11: // set hips command (velocity)
        set_hips_command(arguments);
        break;
      case 12:
        set_grasper_vel_cmd(arguments);
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
  //      Byte 1-3: fwd velocity limit (tentatively map 0-65535 --> -1, 1, scale by max, min limits on board)
     
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
    fullCommand[serialIndex]='\0'; // terminate with escape character to make printable string. Do we need to do this? Probably not.

    //Serial.print("fullCommand:");
    //Serial.println(fullCommand);
    memcpy(latest_command.receivedFromSerial,fullCommand,receivedStateCmdSize);
    
    Serial.println("Received:"+String(latest_command.message.xdot_cmd, 6));
    Serial.println("NumBytes:"+String(serialIndex));
    // Serial.println("Received:"+String(latest_command.message.test2));

    serialIndex=0;
  }
  // Serial2.print("Message sent from ESP32|");
  return;
}

void publishSensorValues()
{
  // Send sensor values to the Jetson over UART using Serial Library
    
  /* Inputs
    
  float acc_x;
  float acc_y;
  float acc_z;
  
  float gyr_x;
  float gyr_y;
  float gyr_z;
  
  float phi;                        // Current robot tilt angle
  
  float lw_angvel;                  // Current angular velocity of left wheel
  float rw_angvel;                  // Current angular velocity of right wheel
  
  float lh_ang;                     // Current encoder measurement of left hip angle
  float rh_ang;                     // Current encoder measurement of left hip angle

  float neck_ang;
  
  unsigned short headAngle;         // Current head angle, unsigned short from 0-65535, mappable to actual value

  unsigned short grasperAngle;      // Current commanded grasper state

  char delimiter;

  15 total, 

  Outputs: 
  Publish over Serial to ROS node listener
  */
  latest_state.message.acc_x = float(acc_vec(0));
  latest_state.message.acc_y = float(acc_vec(1));
  latest_state.message.acc_z = float(acc_vec(2));
  latest_state.message.gyr_x = float(gyr_vec(0));
  latest_state.message.gyr_y = float(phidot);
  latest_state.message.gyr_z = float(gyr_vec(2));
  latest_state.message.phi= float(phi);
  latest_state.message.lw_angvel = float(xdot_l);
  latest_state.message.rw_angvel = float(xdot_r);
  latest_state.message.lh_ang = float(l_hip_angle);
  latest_state.message.rh_ang = float(r_hip_angle);
  latest_state.message.neck_ang = float(neck_angle);
  latest_state.message.grasperAngle=short(65535.*u_grasper);
  latest_state.message.headAngle=short(65535.*u_head);
  latest_state.message.unused1=' ';
  latest_state.message.unused2=' ';
  latest_state.message.delimiter=DELIMITER;
  latest_state.message.delimiter2=DELIMITER;
  Serial.println(String(short(65535.*u_head)));
  Serial2.write(latest_state.byteArray, sentMsgSize);
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
  IMU.setGyrOffsets(-27.5680, -88.0617, -4.2623); // gyr.y was -86.3617 before 
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
  encoder_wheel_right.attachHalfQuad(WHEEL_RIGHT_ENC_A, WHEEL_RIGHT_ENC_B); // ENC 1A,B
  encoder_wheel_left.attachHalfQuad(WHEEL_LEFT_ENC_A, WHEEL_LEFT_ENC_B); // ENC 2A,B
  encoder_wheel_right.clearCount();
  encoder_wheel_left.clearCount();

  encoder_hip_right.attachHalfQuad(HIP_RIGHT_ENC_A, HIP_RIGHT_ENC_B); // ENC 3A, 3B
  encoder_hip_left.attachHalfQuad(HIP_LEFT_ENC_A, HIP_LEFT_ENC_B);  // ENC 4A, 4B
  encoder_hip_right.clearCount();
  encoder_hip_left.clearCount();

  encoder_neck.attachHalfQuad(23, 26);
  encoder_neck.clearCount();

  dc_pwm.begin();
  dc_pwm.setOscillatorFrequency(27000000);
  dc_pwm.setPWMFreq(440);

  servo_pwm.begin();
  servo_pwm.setOscillatorFrequency(27000000);
  servo_pwm.setPWMFreq(50);

  pinMode(WHEEL_RIGHT_OCM, INPUT);
  pinMode(NECK_MOTOR_OCM, INPUT);
  pinMode(HIP_LEFT_OCM, INPUT);
  pinMode(HIP_RIGHT_OCM, INPUT);

  // Motor 1 (Left):
  dc_pwm.setPin(WHEEL_LEFT_A, 0, 0);
  dc_pwm.setPin(WHEEL_LEFT_B, 0, 0);
  // Motor 2 (Right):
  dc_pwm.setPin(WHEEL_RIGHT_A, 0, 0);
  dc_pwm.setPin(WHEEL_RIGHT_B, 0, 0);

  // Motor 3 (Left Hip):
  dc_pwm.setPin(HIP_LEFT_A, 0, 0);
  dc_pwm.setPin(HIP_LEFT_B, 0, 0);
  // Motor 4 (Right Hip):
  dc_pwm.setPin(HIP_RIGHT_A, 0, 0);
  dc_pwm.setPin(HIP_RIGHT_B, 0, 0);

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
  // init_phidot_PID();
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
    
    unsigned long timekeeping_0 = micros();
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
    hips = r_hip_angle;

    neck_angle = 2*count_neck_total*2*3.14159265*24/(28*10884.47); // 10884.47 counts per revolution
    // Read in gyro data and compute PID outputs
    
    getAngles();
    velocity_PID.Compute();
    // phidot_PID.Compute();
    r_phi = u_xdot;
    phi_PID.Compute();
    turning_velocity_PID.Compute();
    hips_PID.Compute();

    // Print telemetry to serial
    // Serial.print("Motor forward input (dc_pwm):");
    // Serial.print(-1 * u_phi / 4096);
    // Serial.print(", Current forward velocity (m/s):");
    // Serial.println(">x_dot:"+String(xdot));
    // Serial.println(">r_xdt:"+String(r_xdot));
    // Serial.println(">l_hip_angle:"+String(l_hip_angle));
    // Serial.println(">r_hip_angle:"+String(r_hip_angle));
    // Serial.println(">lower_neck_angle:"+String(neck_angle));
    // Serial.println(">head_angle:"+String(getHeadAngle()));

    // Serial.print(", Current tilt angle (rad):");
    Serial.println(">phi:"+String(phi,4));
    Serial.println(">rphi:"+String(r_phi,4));
    Serial.println(">delta_phi:"+String(delta_phi,4));
    Serial.println(">phidot:"+String(phidot,4));
    Serial.println(">u_phi:"+String(u_phi));
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
    //driveWheelMotors(u_phi + u_theta_dot, u_phi - u_theta_dot);
    //driveJointMotors(u_hips+u_gamma,u_hips-u_gamma,0); // last term should be u_neck
    driveServosVelocity(grasper_vel_cmd,head_vel_cmd,u_grasper,u_head);
    //driveServos(u_grasper,u_head);
    unsigned long timekeeping_1 = micros();
    // Serial.println(">loop_time:"+String(timekeeping_1-timekeeping_0,5));
    // Serial.println(">h_u:"+String(u_head));
    // Serial.println(">h_v_cmd:"+String(head_vel_cmd));
    // Serial.println(">g_u:"+String(u_grasper));
    // Serial.println(">g_v_cmd:"+String(grasper_vel_cmd));
    publishSensorValues();
  }

  // Read in and process commands from Bluetooth or serial controller
  if (SerialBT.available())
  {
    char b = SerialBT.read();
    //Serial.println("SerialBT received byte:"+b);
    processReceivedBTValue(b, command_BT);
    //processSerialCommand(b);
  } 
  if (Serial2.available())
  {
    char b = Serial2.read();
    //Serial.println("received:" + String(b));
    processSerialCommand(b);
  }
}
