#include <Wire.h>
#include <ICM20948_WE.h>
#include <Adafruit_PWMServoDriver.h>
#include <ESP32Encoder.h>
#include <math.h>
#include <cmath>
#include <ArduinoEigen.h>
#include <PID_v1.h>
#include "BluetoothSerial.h"
#include "ArduinoJson.h"

// Serialization constants
#define DELIMITER '\n'

#if !defined(CONFIG_BT_ENABLED) || !defined(CONFIG_BLUEDROID_ENABLED)
#error Bluetooth is not enabled! Please run `make menuconfig` to and enable it
#endif

BluetoothSerial SerialBT;

String command = "";
unsigned long command_index = 0;

/* OPCODES
 *  0 - Set velocity
 *  1 - Set theta dot
 */

/* TELEMETRY TYPES
 *  0 - PID data
 */

#define VELOCITY_SCALE 10

#define ICM20948_ADDR 0x69
#define SERIAL_PORT Serial
#define OCM1 36
#define OCM2 39
#define OCM3 34
#define OCM4 35
#define WIRE_PORT Wire 
#define AD0_VAL 1      // The value of the last bit of the I2C address.                

using namespace Eigen;
using Eigen::MatrixXd;
using Eigen::Matrix;

// defining all encoders
ESP32Encoder encoder1;
ESP32Encoder encoder2;
ESP32Encoder encoder3;
ESP32Encoder encoder4;
ESP32Encoder encoder5;
ESP32Encoder encoder6;

// Initializing encoder variables:
volatile int countRTotal = 0; //right wheel
volatile int countLTotal = 0; //left wheel
volatile int countR = 0; //right wheel
volatile int countL = 0; //left wheel

// PID Constants for wheel velocity to create angle (loop one)
double Kp_w = 0.030;
double Ki_w = 0.3;
double Kd_w = 0.002;
#define VELOCITY_LIMIT 0.1

// Controller constants for phi motor speed (loop one)
double Kp_phi = 12500;
double Ki_phi = 145000;
double Kd_phi = 2000; 
#define PHI_LIMIT 4096

double r_phi; // commanded phi, radians
double r_phi_ph; // placeholder in case one variable can't be used in two PID loops at once
double phi_t;     // angle phi as calcd from gyro
double phi_t_acc;     // angle phi as calcd from acc'mtrr_
double e_phi = 0; // angle error

double r_xdot = 0; // commanded wheel speed, m/s
double xdot_t = 0; // wheel velocity at a given time, avg of left and right
double e_xdot = 0; // error in wheel speed

// Theta_dot Controller
double u_theta_dot_l = 0; // used in negative for u_theta_r
double r_theta_dot = 0; //commanded theta_dot, initialized @0 at startup
double theta_dot_t = 0; //theta at a given time, initialized @0

// TODO: Tune theta_dot PID values
double Kp_theta_dot = 200;
double Ki_theta_dot = 150;
double Kd_theta_dot = 5;
#define THETA_DOT_LIMIT 0.1

// Initializing hardware timer
volatile bool deltaT = false;     // check timer interrupt
hw_timer_t * timer0 = NULL;
portMUX_TYPE timerMux0 = portMUX_INITIALIZER_UNLOCKED;
int timestep_ms = 25; // 50ms timestep between timer calls
int timesteps_passed = 0; //int
int time_onafter = 2; // time after setup (s) to wait before running main loop

ICM20948_WE myIMU = ICM20948_WE(ICM20948_ADDR);
Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();
const int MAX_PWM = 4096;

// Physical Constants:
float r_w = 0.0508; // wheel radius, meters  

double v_fwd;           // average forward velocity from encoders
double v_l;
double v_r;
double r_v=0.0;

double u_fwd;          // fwd input
double u_l;            // left motor input
double u_r;            // right motor input

Vector3f acc_vert_e;
Vector3f acc_t_e;
Vector3f acc_n_e;

PID velocity_PID(&v_fwd,&r_phi_ph,&r_v,Kp_w,Ki_w,Kd_w, DIRECT);
PID angle_PID(&phi_t,&u_fwd,&r_phi,Kp_phi,Ki_phi,Kd_phi, DIRECT);
PID theta_dot_PID(&theta_dot_t,&u_theta_dot_l,&r_theta_dot,Kp_theta_dot,Ki_theta_dot,Kd_theta_dot, DIRECT);

void IRAM_ATTR onTime0() {
  portENTER_CRITICAL_ISR(&timerMux0);
  // Code to be called when timer is activated:
  // Get each wheel's encoder count:
  countR = -1*encoder1.getCount();
  countL = encoder2.getCount();
  // 
  countLTotal += countL;
  countRTotal += countR;
  // clear encoders
  encoder1.clearCount();
  encoder2.clearCount();
    
  deltaT = true; // time has passed
  portEXIT_CRITICAL_ISR(&timerMux0);
}

void driveMotors(float u_l, float u_r){
    // Setting Motor speeds:
    if (u_l > MAX_PWM){
      u_l = MAX_PWM;
    }
    else if (u_l < -MAX_PWM){
      u_l = -MAX_PWM;
    }
    if (u_r > MAX_PWM){
      u_r = MAX_PWM;
    }
    else if (u_r < -MAX_PWM){
      u_r = -MAX_PWM;
    }
    
    if (u_l>=0){
      // left wheel
      pwm.setPin(0,u_l,0);
      pwm.setPin(1,0,0);
    }
    else if (u_l<=0){
      // left
      pwm.setPin(0,0,0);
      pwm.setPin(1,-u_l,0);
    }
    if (u_r>=0){
      // right wheel      
      pwm.setPin(2,0,0);
      pwm.setPin(3,u_r,0);
    }
    else if (u_r<=0){
    // right
      pwm.setPin(2,-u_r,0);
      pwm.setPin(3,0,0);
    }
  }

void getAngle(){
  // Synthesize and filter Gyro and Accelerometer readings to get accurate angle measurement

  // Read IMU
  myIMU.readSensor();
  xyzFloat acc = myIMU.getGValues();
  xyzFloat gyr = myIMU.getGyrValues();  // gyr in deg/s

  // Gyro angle calc: 
  phi_t = phi_t - gyr.y*timestep_ms/1000*(3.14159265/180); // converting to rad
  theta_dot_t = gyr.z/1000*(3.14159265/180);
  
  // Accelerometer angle calc: 
  acc_t_e << acc.x, acc.y, acc.z;
  phi_t_acc = atan2((acc_vert_e.cross(acc_t_e)).dot(acc_n_e),acc_t_e.dot(acc_vert_e));

  // Complementary Filter
  phi_t = 0.995*phi_t + 0.005*phi_t_acc;
}

void init_angle_PID(){
  angle_PID.SetSampleTime(timestep_ms);
  angle_PID.SetOutputLimits(-PHI_LIMIT,PHI_LIMIT);
  angle_PID.SetMode(AUTOMATIC);
}
void init_velocity_PID(){
  velocity_PID.SetSampleTime(timestep_ms);
  velocity_PID.SetOutputLimits(-VELOCITY_LIMIT,VELOCITY_LIMIT);
  velocity_PID.SetMode(AUTOMATIC);
}

void init_theta_dot_PID(){
  theta_dot_PID.SetSampleTime(timestep_ms);
  theta_dot_PID.SetOutputLimits(-THETA_DOT_LIMIT,THETA_DOT_LIMIT);
  theta_dot_PID.SetMode(AUTOMATIC);
}

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
  sprintf(buffer, "Setting theta_dot: %d.", r_v);
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
  sprintf(buffer, "Setting theta_dot: %d.", r_theta_dot);
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
  // Wait until serial port initialized before progressing...
  while (!Serial){}

  // i2c wire initialization
  WIRE_PORT.begin();
  WIRE_PORT.setClock(400000);
  
  if(!myIMU.init()){
    Serial.println("ICM20948 does not respond");
  }
  delay(1000);

  myIMU.setGyrOffsets(-27.5680, -86.3617, -4.2623);
  myIMU.setGyrRange(ICM20948_GYRO_RANGE_250);
  myIMU.setGyrDLPF(ICM20948_DLPF_6);
  myIMU.setAccRange(ICM20948_ACC_RANGE_2G);
  myIMU.setAccDLPF(ICM20948_DLPF_6);
  
  myIMU.readSensor();
  xyzFloat acc = myIMU.getGValues();
  
  // New implementation: signed:
  acc_vert_e << 0.20, -0.03, -0.98; // from holding bot vertical and recording values
  acc_t_e << acc.x, acc.y, acc.z;
  
  //acc_n_e << (acc_t_e.cross(acc_vert_e)).normalized();
  //phi_t = atan2((acc_t_e.cross(acc_vert_e)).dot(acc_n_e),acc_vert_e.dot(acc_t_e));
  
  acc_n_e << (acc_vert_e.cross(acc_t_e)).normalized();

  if(acc_n_e(1)<0){
    acc_n_e = acc_n_e*-1; // reverse normal vector if reversed
  }
  
  phi_t = atan2((acc_vert_e.cross(acc_t_e)).dot(acc_n_e),acc_t_e.dot(acc_vert_e));

  phi_t_acc = phi_t; // to start us off, initial phi_t and phi_t_acc both come from acc. 

  Serial.print("phi_0:");
  Serial.println(phi_t);
    
  // Initializing Encoders
  ESP32Encoder::useInternalWeakPullResistors=UP;
  
  // Which wheel?
  encoder1.attachHalfQuad(33, 4);
  // Which wheel?
  encoder2.attachHalfQuad(5, 13);
  
  encoder1.clearCount();
  encoder2.clearCount();
  
  pwm.begin();
  pwm.setOscillatorFrequency(27000000);
  pwm.setPWMFreq(440);
  
  pinMode(OCM1,INPUT);
  pinMode(OCM2,INPUT);
  pinMode(OCM3,INPUT);
  pinMode(OCM4,INPUT);

  // Motor 1:
  pwm.setPin(0,0,0);
  pwm.setPin(1,0,0);
  // Motor 2:
  pwm.setPWM(2,0,0);
  pwm.setPWM(3,0,0);

  timer0 = timerBegin(0, 80, true);  // timer 0, MWDT clock period = 12.5 ns * TIMGn_Tx_WDT_CLK_PRESCALE -> 12.5 ns * 80 -> 1000 ns = 1 us, countUp
  timerAttachInterrupt(timer0, &onTime0, true); // edge (not level) triggered
  timerAlarmWrite(timer0, timestep_ms*1000, true); // 10000 * 1 us = 10 ms, autoreload true
  timerAlarmEnable(timer0); // enable
  delay(2000);
  
  init_angle_PID();
  init_velocity_PID();
  init_theta_dot_PID();
}

void loop() {
  if(deltaT){
    // ensuring reset isn't skipped:
    portENTER_CRITICAL(&timerMux0);
    deltaT = false;
    portEXIT_CRITICAL(&timerMux0);

    // Get new y values (x, xdot, phidot):
    // x: (m, average distance of left and right wheel encoders)
    // xhat(0) = ((countLTotal + countRTotal)/2)*(r_w*4*3.141592/2248.86); 
    
    // x_dot: (m/s)
    // xhat(1) = ((countL+countR)/4)*4*3.14159265*r_w/(2248.86*timestep_ms/1000);
    // phidot:
    // y_act(1) = -1*gyr.y*3.141592/180;     // convert to rad/s, correct direction
    
    // xdot: (m/s)
    v_l = countL*4*3.14159265*r_w/(2248.86*timestep_ms/1000);
    v_r = countR*4*3.14159265*r_w/(2248.86*timestep_ms/1000);
    v_fwd = ((countL+countR)/2)*4*3.14159265*r_w/(2248.86*timestep_ms/1000); //(m/s)
    
    // thetadot (m/s):
        
    getAngle();
    velocity_PID.Compute();
    r_phi=r_phi_ph;
    angle_PID.Compute();
    theta_dot_PID.Compute();
    
    Serial.print("u_fwd(pwm):");
    Serial.print(-1*u_fwd/4096);
    Serial.print(", v_fwd:");
    Serial.print(v_fwd);
    Serial.print(", phi_rad:");
    Serial.print(phi_t);
    Serial.print(", r_phi:");
    Serial.print(r_phi);
    Serial.print(", theta_dot_t:");
    Serial.print(theta_dot_t);
    Serial.print(", u_theta_dot_l:");
    Serial.print(u_theta_dot_l);
    Serial.println();
    
//    Serial.print(", u_l(N):");
//    Serial.print(controller.u(0));
    //Serial.print(", u_r(N):");
    // Serial.println(controller.u(1));

    u_l = u_fwd + u_theta_dot_l;
    u_r = u_fwd - u_theta_dot_l;
        
    driveMotors(u_l,u_r);
    
//    Serial.print("acc.x: ");
//    Serial.print(acc.x);
//    Serial.print(", acc.y: ");
//    Serial.print(acc.y);
//    Serial.print(", acc.z: ");
//    Serial.println(acc.z);

//    Serial.print("y1:");
//    Serial.print(y_act(0));
//    Serial.print(", y2:");
//    Serial.print(y_act(1));
//    Serial.print(", y3:");
//    Serial.print(y_act(2));

  }
  if (SerialBT.available()) {
    char c = SerialBT.read();
    Serial.print(c);
    processReceivedValue(c);
  }

  }
