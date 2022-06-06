#include <Arduino.h>

// IMU definitions
#include "ICM_20948.h" // Click here to get the library: http://librarymanager/All#SparkFun_ICM_20948_IMU
#include <Wire.h>
#include <Arduino.h>
#include <Adafruit_PWMServoDriver.h>
#include <ESP32Encoder.h>

#define SERIAL_PORT Serial
#define OCM1 36
#define OCM2 39
#define OCM3 34
#define OCM4 35

#define PHI_LIM 5

ESP32Encoder encoder1;
ESP32Encoder encoder2;
ESP32Encoder encoder3;
ESP32Encoder encoder4;
ESP32Encoder encoder5;
ESP32Encoder encoder6;

#define WIRE_PORT Wire // Your desired Wire port.      Used when "USE_SPI" is not defined
#define AD0_VAL 1      // The value of the last bit of the I2C address.                \
                       // On the SparkFun 9DoF IMU breakout the default is 1, and when \
                       // the ADR jumper is closed the value becomes 0

// Initializing hardware timer
volatile bool deltaT = false;     // check timer interrupt
hw_timer_t * timer0 = NULL;
portMUX_TYPE timerMux0 = portMUX_INITIALIZER_UNLOCKED;
int timestep_ms = 25; // 25ms timestep between timer calls
int timesteps_passed = 0; //int
int time_onafter = 2; // time after setup (s) to change command speed for quantifying wheel speed

// Controller constants for wheel speed (tune!)
const double Kp_w = 10;
const double Ki_w = 8;
const double Kd_w = 5.0;

double e_left = 0.0;
double e_leftOld = 0.0;

double e_right = 0.0;
double e_rightOld = 0.0;

double e_phidot = 0.0;
double e_phidotOld = 0.0;

// Controller constants for leaning fwd/back
const double Kp_phi = -5;
const double Ki_phi = -3;
const double Kd_phi = -2;

const int MAX_PWM = 4096;
// for integral control:
int sigmaE_phi = 0;
int sigmaE_lw = 0;
int sigmaE_rw = 0;
// for derivative control:
int dE_phidot= 0;
int dE_lw = 0;
int dE_rw = 0;

// IMU omega in forward/back direction:
double phidot = 0.0;
// for rolling average:
double phidot0 = 0.0;
double phidot1 = 0.0;
double phidot2 = 0.0;
double phidot3 = 0.0;
double phidot4 = 0.0;
// Encoder counts:
volatile int count1 = 0; //right wheel
volatile int count1_old = 0; //right wheel
volatile int count2 = 0; //left wheel
volatile int count2_old = 0; //left wheel

double r_wheel = 50.8; //mm

int u_leftwheel; // electrical output to left wheel, pwm proportion out of 4096
double r_leftwheel = 0; // commanded speed of left wheel, rpm
double actual_omegaleft; // actual speed of left wheel, rpm

int u_rightwheel; // command speed of right wheel
double r_rightwheel = 0; // commanded speed of left wheel, rpm
double actual_omegaright; // actual speed of right wheel, rad/s

// IMU initiatlization
ICM_20948_I2C myIMU; 
// initialize pwm code:
Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();

void IRAM_ATTR onTime0() {
  portENTER_CRITICAL_ISR(&timerMux0);
  // Code to be called when timer is activated:
  // Get each wheel's encoder count:
  count1_old = count1;
  count2_old = count2;
  
  count1 = encoder1.getCount( );
  count2 = -1*encoder2.getCount( );
  
  encoder1.clearCount ( );
  encoder2.clearCount ( );
  deltaT = true; // time has passed
  
  portEXIT_CRITICAL_ISR(&timerMux0);
}

void setup() {
  
  SERIAL_PORT.begin(115200);
  // Wait until serial port initialized before progressing...
  while (!SERIAL_PORT)
  {
  };

  // i2c wire initialization
  WIRE_PORT.begin();
  WIRE_PORT.setClock(400000);
  
  bool initialized = false;
  while (!initialized)
  {
    myIMU.begin(WIRE_PORT, AD0_VAL);

    SERIAL_PORT.print(F("Initialization of the sensor returned: "));
    SERIAL_PORT.println(myIMU.statusString());
    if (myIMU.status != ICM_20948_Stat_Ok)
    {
      SERIAL_PORT.println("Trying again...");
      delay(500);
    }
    else
    {
      initialized = true;
    }
  }
  ESP32Encoder::useInternalWeakPullResistors=UP;
  
  encoder1.attachHalfQuad(33, 4);
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
  
  Serial.begin(115200);

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
  r_leftwheel = 0; // RPM
  delay(5000);
}

void loop() {
  // put your main code here, to run repeatedly:
  if(deltaT){
    // ensuring reset isn't skipped:
    portENTER_CRITICAL(&timerMux0);
    deltaT = false;
    timesteps_passed +=1; //adding one timestep to the count 
    portEXIT_CRITICAL(&timerMux0);
    
    if (myIMU.dataReady()){
      myIMU.getAGMT();         // The values are only updated when you call 'getAGMT'
      phidot4 = phidot3;
      phidot3 = phidot2;
      phidot2 = phidot1;
      phidot1 = phidot0;
      phidot0 = myIMU.gyrY()+0.6298311307;
      phidot = (phidot0+phidot1+phidot2+phidot3+phidot4)/5;
      //Serial.print("Phidot: ");
      //Serial.print(phidot);
      //Serial.println();
    }

    /*if ((timesteps_passed*timestep_ms/1000)>=time_onafter){
      r_leftwheel = 100; //rpm
    }*/
    
    actual_omegaleft = 2*60*count1/(2248.86*timestep_ms/1000); //  rpm
    actual_omegaright = 2*60*count2/(2248.86*timestep_ms/1000); // rpm

    e_phidotOld = e_phidot; // getting last error for dE
    e_phidot = (0-phidot); // Error from not falling over
    
    
//    Serial.print("Phidot:");
//    Serial.print(phidot);
//    Serial.print(",sigmaE_phi:");
//    Serial.print(sigmaE_phi);
//    Serial.print(",dE_phidot:");
//    Serial.print(dE_phidot);
    
    
    // Control tipping: 
    if(phidot > PHI_LIM || phidot < PHI_LIM){
      sigmaE_phi = sigmaE_phi + e_phidot; // Command is negative phidot error
      dE_phidot = e_phidot - e_phidotOld; // Now accounting for running average! not so sus

      r_leftwheel = Kp_phi*(phidot) + Ki_phi*sigmaE_phi/10 - Kd_phi*dE_phidot; // units here don't really make sense do they?? 
      r_rightwheel = (Kp_phi*(phidot) + Ki_phi*sigmaE_phi/10 - Kd_phi*dE_phidot); // or here lol
    }
    
    Serial.print("command_omegalw:");
    Serial.print(r_leftwheel); // should be RPM... we'll see
    Serial.print(", actual_omegalw:");
    Serial.print(actual_omegaleft); // should be RPM... we'll see
    Serial.print(",integral term:");
    Serial.print(sigmaE_phi);
    Serial.println();
    
    // Control wheel speed: 
    e_leftOld = e_left;
    e_left = r_leftwheel - actual_omegaleft; // error in left wheel's speed
    sigmaE_lw += e_left;
    dE_lw = e_left - e_leftOld;
    
    e_rightOld = e_right;
    e_right = (r_rightwheel - actual_omegaright); // error in right wheel's speed
    sigmaE_rw += e_right;
    dE_rw = e_right - e_rightOld;
    
    u_leftwheel = Kp_w*(e_left) + Ki_w*sigmaE_lw - Kd_w *(dE_lw); //commanded speed of the left wheel, in rpm
    u_rightwheel = (Kp_w*(e_right) + Ki_w*sigmaE_rw - Kd_w *(dE_rw)); //commanded speed of the right wheel, in rpm
    
    if (u_leftwheel > MAX_PWM){
      u_leftwheel = MAX_PWM;
    }
    else if (u_leftwheel < -MAX_PWM){
      u_leftwheel = -MAX_PWM;
    }
    if (u_rightwheel > MAX_PWM){
      u_rightwheel = MAX_PWM;
    }
    else if (u_rightwheel < -MAX_PWM){
      u_rightwheel = -MAX_PWM;
    }
    
    if (u_leftwheel>=0){
      pwm.setPin(0,u_leftwheel,0);
      pwm.setPin(1,0,0);
    }
    else if (u_leftwheel<=0){
      pwm.setPin(0,0,0);
      pwm.setPin(1,-u_leftwheel,0);
    }
    if (u_rightwheel>=0){
      pwm.setPin(2,0,0);
      pwm.setPin(3,u_rightwheel,0);
    }
    else if (u_rightwheel<=0){
      pwm.setPin(2,-u_rightwheel,0);
      pwm.setPin(3,0,0);
    }

//    Serial.print("phidot:");
//    Serial.print(phidot);
//    Serial.print("OmegaRw:");
//    Serial.print(actual_omegaright);
//    Serial.print("Cmd_Lw:");
//    Serial.print(u_leftwheel);
//    Serial.print("Cmd_Rw:");
//    Serial.print(u_rightwheel);
    Serial.println();
    
    
  }
  else
  {
  }
  // Overcurrent monitoring:
  //int ocm_1 = analogRead(OCM1);
  //int ocm_2 = analogRead(OCM2);
  //int ocm_3 = analogRead(OCM3);
  //int ocm_4 = analogRead(OCM4);
  
}

//void controlSchema1(){
//  // We set the motor speeds to offset tipping:
//    u_leftwheel = Kp*(-1*phidot) + Ki*sigmaE/10;
//    u_rightwheel = -1*(Kp*(-1*phidot) + Ki*sigmaE/10);
//
//    if (u_leftwheel > MAX_PWM){
//      u_leftwheel = MAX_PWM;
//    }
//    else if (u_leftwheel < -MAX_PWM){
//      u_leftwheel = -MAX_PWM;
//    }
//    
//    if (u_rightwheel > MAX_PWM){
//      u_rightwheel = MAX_PWM;
//    }
//    else if (u_rightwheel < -MAX_PWM){
//      u_rightwheel = -MAX_PWM;
//    }
//
//    if (u_leftwheel>=0){
//      pwm.setPWM(0,u_leftwheel,0);
//      pwm.setPWM(1,0,u_leftwheel);
//    }
//    else if (u_leftwheel<=0){
//      pwm.setPWM(0,0,-u_leftwheel);
//      pwm.setPWM(1,-u_leftwheel,0);
//    }
//    if (u_rightwheel>=0){
//      pwm.setPWM(2,u_rightwheel,0);
//      pwm.setPWM(3,0,u_rightwheel);
//    }
//    if (u_rightwheel<=0){
//      pwm.setPWM(2,0,-u_rightwheel);
//      pwm.setPWM(3,-u_rightwheel,0);
//    }
//  
//}