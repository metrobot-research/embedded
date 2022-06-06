#include <Wire.h>
#include <ICM20948_WE.h>
#include <Adafruit_PWMServoDriver.h>
#include <ESP32Encoder.h>
#include <math.h>
#include <cmath>
#include <ArduinoEigen.h>
#include <PID_v1.h>

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

// Controller constants for phi motor speed (loop one)
double Kp_phi = 12500;
double Ki_phi = 145000;
double Kd_phi = 2000; 

double r_phi; // commanded phi, radians
double r_phi_ph; // placeholder in case one variable can't be used in two PID loops at once
double phi_t;     // angle phi as calcd from gyro
double phi_t_acc;     // angle phi as calcd from acc'mtrr_
double e_phi = 0; // angle error

double r_xdot = 0; // commanded wheel speed, m/s
double xdot_t = 0; // wheel velocity at a given time, avg of left and right
double e_xdot = 0; // error in wheel speed

// Theta Controller
double u_theta_l = 0; // used in negative for u_theta_r
double r_theta = 0; //commanded theta position, initialized @0 at startup
double theta_t = 0; //theta at a given time, initialized @0

double Kp_theta = 200;
double Ki_theta = 150;
double Kd_theta = 5;


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
PID theta_PID(&theta_t,&u_theta_l,&r_theta,Kp_theta,Ki_theta,Kd_theta, DIRECT);

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
  theta_t = theta_t + gyr.z*timestep_ms/1000*(3.14159265/180);
  
  // Accelerometer angle calc: 
  acc_t_e << acc.x, acc.y, acc.z;
  phi_t_acc = atan2((acc_vert_e.cross(acc_t_e)).dot(acc_n_e),acc_t_e.dot(acc_vert_e));

  // Complementary Filter
  phi_t = 0.995*phi_t + 0.005*phi_t_acc;
}

void init_angle_PID(){
  angle_PID.SetSampleTime(timestep_ms);
  angle_PID.SetOutputLimits(-4096,4096);
  angle_PID.SetMode(AUTOMATIC);
}
void init_velocity_PID(){
  velocity_PID.SetSampleTime(timestep_ms);
  velocity_PID.SetOutputLimits(-0.1,0.1);
  velocity_PID.SetMode(AUTOMATIC);
}

void init_theta_PID(){
  theta_PID.SetSampleTime(timestep_ms);
  theta_PID.SetOutputLimits(-4096,4096);
  theta_PID.SetMode(AUTOMATIC);
}

void setup() {
  Serial.begin(115200);
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
  init_theta_PID();
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
    theta_PID.Compute();
    
    Serial.print("u_fwd(pwm):");
    Serial.print(-1*u_fwd/4096);
    Serial.print(", v_fwd:");
    Serial.print(v_fwd);
    Serial.print(", phi_rad:");
    Serial.print(phi_t);
    Serial.print(", r_phi:");
    Serial.print(r_phi);
    Serial.print(", theta_t:");
    Serial.print(theta_t);
    Serial.print(", u_theta_l:");
    Serial.print(u_theta_l);
    Serial.println();
    
//    Serial.print(", u_l(N):");
//    Serial.print(controller.u(0));
    //Serial.print(", u_r(N):");
    // Serial.println(controller.u(1));

    u_l = u_fwd + u_theta_l;
    u_r = u_fwd - u_theta_l;
        
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

  }
