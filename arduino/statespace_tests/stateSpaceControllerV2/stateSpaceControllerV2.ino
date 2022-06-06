#include <Wire.h>
#include <ICM20948_WE.h>
#include <Adafruit_PWMServoDriver.h>
#include <ESP32Encoder.h>
#include <math.h>
#include <cmath>
#include <ArduinoEigenDense.h>
#include <ArduinoEigen.h>

#define ICM20948_ADDR 0x69
#define SERIAL_PORT Serial
#define OCM1 36
#define OCM2 39
#define OCM3 34
#define OCM4 35
#define WIRE_PORT Wire 
#define AD0_VAL 1      // The value of the last bit of the I2C address.                \

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

ICM20948_WE myIMU = ICM20948_WE(ICM20948_ADDR);

// Initializing hardware timer
volatile bool deltaT = false;     // check timer interrupt
hw_timer_t * timer0 = NULL;
portMUX_TYPE timerMux0 = portMUX_INITIALIZER_UNLOCKED;
int timestep_ms = 50; // 50ms timestep between timer calls
int timesteps_passed = 0; //int
int time_onafter = 2; // time after setup (s) to wait before running main loop

Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();
const int MAX_PWM = 4096;

// Gravity direction when bot is balanced:
// static float acc_vert [3] = {0.20, -0.03, -0.98}; // from holding bot vertical and recording values
float phi_init; // initial phi offset when starting
float phi_gyr; // phi as calcd from gyr
float phi_acc; // phi as calcd from accelerometer

// Physical Constants:
float r_w = 0.0508; // wheel radius, meters  

// Initialize Linear Algebra Library:
using namespace Eigen;
using Eigen::MatrixXd;
using Eigen::Matrix4f;
using Eigen::Vector4f;
using Eigen::Matrix;

Vector3f acc_t;     // creating 3x1 vec for vertical acceleration value
Vector3f acc_vert;
Vector3f acc_n;     // normal unit vector

Vector4f xhat;      // creating a 4x1 col vec for state
Vector4f r;         // State command
Vector4f xhat_dot;  // col vector for xhat_dot
// Initialize A, B, C, D:
Matrix4f sys_A;
Vector4f sys_B;             
Matrix<float, 3, 4> sys_C;  
Matrix<float, 3, 1> sys_D;
// LQG Init
Vector4f K_c;               // LQR
Matrix<float, 4,3> K_f;     // Kalman Filter
float u;            // Motor FWD Drive Input

Vector3f y_act; //defining initial sensor outputs:
Vector3f y_hat; //defining predicted sensor outputs:

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
  
    
  // Determining +/- intial lean angle phi_init (radians) (!!! Unsigned, may cause a bug for negatives!!!) 
  
  // Old Implementation: Unsigned
  //float acc_init [3] = {acc_0.x, acc_0.y, acc_0.z};
  //phi_init = acos((acc_init[0]*acc_vert[0] + acc_init[2]*acc_vert[2])/(sqrt(pow(acc_init[0],2.0)+pow(acc_init[2],2.0))*sqrt(pow(acc_vert[0],2.0)+pow(acc_vert[2],2.0))));

  // New implementation: signed:
  acc_vert << 0.20, -0.03, -0.98; // from holding bot vertical and recording values
  acc_t << acc.x, acc.y, acc.z;
  acc_n << (acc_t.cross(acc_vert)).normalized();
  phi_init = atan2((acc_t.cross(acc_vert)).dot(acc_n),acc_vert.dot(acc_t));
  
  Serial.print("phi_init = ");
  Serial.println(phi_init);
  phi_gyr = phi_init; //starting point for phi_gyr

  // Defining Matrices:
  sys_A << 
    0, 1.0, 0, 0, 
    0, 0, -50.9419, 0,
    0, 0, 0, 1,
    0, 0, 56.8205, 0;
  sys_B <<
    0,
    0.0060,
    0,
    -0.0051;
  sys_C <<
    1, 0, 0, 0,
    0, 1, 0, 0,
    0, 0, 0, 1;
  sys_D << 
    0,
    0,
    0; 
  K_c <<
    -1, -37.8731, -22329.5498, -2996.1255;
  K_f <<
    1.0477, 1.0285, -0.046834,
    0.10285, 19.844, -2.3413,
    -0.0053763, -4.3059, 0.61808,
    -0.046834, -23.413, 4.0523;
    
  xhat << //setting initial state
    0, // x
    0, // xdot
    phi_init, // phi
    0; // phidot
  xhat_dot << //setting initial state_dot
    0, 0, 0, 0;
  r <<
    0, 0, 0, 0;
  y_act << 
    0, 0, 0;
  y_hat <<
    0, 0, 0;
  
  

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
}

void loop() {
  if(deltaT){
    // ensuring reset isn't skipped:
    portENTER_CRITICAL(&timerMux0);
    deltaT = false;
    portEXIT_CRITICAL(&timerMux0);

    // Get new y values (x, xdot, phidot):
    // x: (m, average distance of left and right wheel encoders)
    y_act(0) = ((countLTotal + countRTotal)/2)*(r_w*4*3.141592/2248.86); 
    
    // x_dot: (m/s)
    y_act(1) = ((countL+countR)/2)*4*3.14159265*r_w/(2248.86*timestep_ms/1000);

    // phidot: (rad/s)
    myIMU.readSensor();
    xyzFloat gyr = myIMU.getGyrValues();  // gyr in deg/s
    y_act(2) = -1*gyr.y*3.141592/180;     // convert to rad/s, correct direction

    xyzFloat acc = myIMU.getGValues();
    // New implementation: signed:
    
    acc_t << acc.x, acc.y, acc.z;
    acc_n << (acc_t.cross(acc_vert)).normalized();
    phi_init = atan2((acc_t.cross(acc_vert)).dot(acc_n),acc_vert.dot(acc_t));
    // Estimate current state:
    // y_hat = sys_C*xhat;
    // Determine predicted xhat_dot:
    // xhat_dot = (sys_A - sys_B*K_c.transpose() - K_f*sys_C)*xhat + K_f*y_act;
    // xhat = xhat + xhat_dot*timestep_ms/1000;
    
    // u = 1*K_c.transpose()*(r-xhat);

    Serial.print("y1:");
    Serial.print(y_act(0));
    Serial.print(", y2:");
    Serial.print(y_act(1));
    Serial.print(", y3:");
    Serial.print(y_act(2));

    Serial.print("yhat1:");
    Serial.print(y_hat(0));
    Serial.print(", yhat:");
    Serial.print(y_hat(1));
    Serial.print(", yhat3:");
    Serial.println(y_hat(2));

    
    Serial.print("xhat_dot_1:");
    Serial.print(xhat_dot(0));
    Serial.print(", xhat_dot_2:");
    Serial.print(xhat_dot(1));
    Serial.print(", xhat_dot_3:");
    Serial.print(xhat_dot(2));
    Serial.print(", xhat_dot_4:");
    Serial.print(xhat_dot(3));
    Serial.print(", u:");
    Serial.println(u);    

    // Setting Motor speeds:
    if (u > MAX_PWM){
      u = MAX_PWM;
    }
    else if (u < -MAX_PWM){
      u = -MAX_PWM;
    }
    if (u>=0){
      // left wheel
      pwm.setPin(0,u,0);
      pwm.setPin(1,0,0);
      // right wheel      
      pwm.setPin(2,0,0);
      pwm.setPin(3,u,0);
    }
    else if (u<=0){
      // left
      pwm.setPin(0,0,0);
      pwm.setPin(1,-u,0);
      // right
      pwm.setPin(2,-u,0);
      pwm.setPin(3,0,0);
    }
    
    //phi_gyr = phi_gyr - gyr.y*timestep_ms/1000.0;

//    // Getting acc'n values
//    // xyzFloat acc = myIMU.getGValues();
//    // We will compare angle accuracy of the two methods. Starting from phi_init:
//    // Method one: forward integration of IMU. Add dt * phidot to phi (phigyr)
//    phi_acc = 57.2957795*acos((acc.x*acc_vert[0] + acc.z*acc_vert[2])/(sqrt(pow(acc.x,2.0)+pow(acc.z,2.0))*sqrt(pow(acc_vert[0],2.0)+pow(acc_vert[2],2.0))));
    
//    Serial.print("phi_acc: ");
//    Serial.print(phi_acc);
//    Serial.print(", phi_gyr: ");
//    Serial.println(phi_gyr);
    
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
