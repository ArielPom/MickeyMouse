// Include required libraries
#include <Arduino.h>
#include <math.h> 

// Define pin numbers for motors and encoders
const int encoderPinA1 = 27;
const int encoderPinB1 = 14;
const int encoderPinA2 = 13;
const int encoderPinB2 = 12;
const int motor1PWMPin = 22;
const int motor1DirPin = 23;
const int motor2PWMPin = 18;
const int motor2DirPin = 19;

// Variables to store encoder counts
volatile int lastEncoderAState1 = 0;
volatile int lastEncoderBState1 = 0;
volatile int lastEncoderAState2 = 0;
volatile int lastEncoderBState2 = 0;
volatile long encoderPos1 = 0;
volatile long encoderPos2 = 0;
volatile int encoderAState1 = 0;
volatile int encoderBState1 = 0;
volatile int encoderAState2 = 0;
volatile int encoderBState2 = 0;
int motor1Speed = 0; 
int motor2Speed = 0; 
volatile int motor1Dir = 1;
volatile int motor2Dir = 1;
int interval_print = 1;
float pT_p = 0.0;
float dt_p = 0.0;
float interval_kalman = 10.0;
long prevT1 = 0;
long currT1 = 0;
long time_last_kalman1 = 0;
float dt1 = 10.0;
long prevT2 = 0;
long currT2 = 0;
long time_last_kalman2 = 0;
float dt2 = 10.0;
float motorSpeed1 = 0.0;
float motorSpeed2 = 0.0;
float motorSpeed1_pre = 0.0;
float motorSpeed2_pre = 0.0;
float motorSpeed1_f = 0.0;
float motorSpeed2_f = 0.0;

// Kalman1 setup
float v_init = (13*464.64/1000)*(13*464.64/1000);  // [encoder/msec]**2
float Fk1[2][2];
float Hk1[1][2];
float Qk1[2][2];
float Rk1 = 1.0;
float Pk1[2][2]; 
float K1[2][1];
float Sk1 = 0.0;
float yk1 = 0.0;
float x_hat1[2][1];
float x_hat_pre1[2][1];
float Pk_pre1[2][2];


// // Kalman2 setup
float Fk2[2][2];
float Hk2[1][2];
float Qk2[2][2];
float Rk2 = 1.0;
float Pk2[2][2]; 
float K2[2][1];
float Sk2 = 0.0;
float yk2 = 0.0;
float x_hat2[2][1];
float x_hat_pre2[2][1];
float Pk_pre2[2][2];

// Localization setup 
float xt_pre = 0.0;
float yt_pre = 0.0;
float dw = 51.858; // in [mm]
float dR = 0.0;
float dL = 0.0;
float d = 0.0;
float pi = 3.14159;
float d_theta = 0.0;
float theta = 0.0;
float theta_pre = 0.0;
float xt = 0.0;
float yt = 0.0;
float wheel_d = 36.3; // in [mm]
float teeth_ratio = 1.4;
volatile float encoderPos1_dot = 0;
volatile float encoderPos2_dot = 0;
long prevTl = 0;
long currTl = 0;
long time_last_l = 0;
float dtl = 100.0;
volatile int encoderPos1_pre = 0;
volatile int encoderPos2_pre = 0;
int init_check = 0;

  // PID1 
  float error1 = 0.0;
  float dedt1 = 0.0;
  float eintegral1 = 0.0;
  float pre_error1 = 0.0;
  float dt_pid1 = 0.0;
  long prevT_pid1 = 0;
  long currT_pid1 = 0;
  float kp1 = 0.08;
  float ki1 = 0.0007;
  float kd1 = 0.000;
  float u1 = 0.0;

  // PID2 
  float error2 = 0.0;
  float dedt2 = 0.0;
  float eintegral2 = 0.0;
  float pre_error2 = 0.0;
  float dt_pid2 = 0.0;
  long prevT_pid2 = 0;
  long currT_pid2 = 0;
  float kp2 = 0.08;
  float ki2 = 0.0007;
  float kd2 = 0.000;
  float u2 = 0.0;

// Setup function
void setup() {
  // Initialize serial communication
  Serial.begin(115200);
  // Kalman1 
  Fk1[0][0] = 1.0;
  Fk1[0][1] = dt1;
  Fk1[1][0] = 0.0;
  Fk1[1][1] = 1.0;
  Qk1[0][0] = (dt1*dt1*dt1*dt1)/4.0;
  Qk1[0][1] = (dt1*dt1*dt1)/2.0;
  Qk1[1][0] = (dt1*dt1*dt1)/2.0;
  Qk1[1][1] = dt1*dt1;
  Hk1[0][0] = 1.0;
  Hk1[0][1] = 0.0;
  Pk1[0][0] = 10.0;
  Pk1[0][1] = 0.0;
  Pk1[1][0] = 0.0;
  Pk1[1][1] = v_init;
  Pk_pre1[0][0] = Pk1[0][0];
  Pk_pre1[0][1] = Pk1[0][1];
  Pk_pre1[1][0] = Pk1[1][0];
  Pk_pre1[1][1] = Pk1[1][1];
  K1[0][0] = 0.0;
  K1[1][0] = 0.0;
  x_hat1[0][0] = 0.0;
  x_hat1[1][0] = 0.0;
  x_hat_pre1[0][0] = 0.0;
  x_hat_pre1[1][0] = 0.0;

  // Kalman2 
  Fk2[0][0] = 1.0;
  Fk2[0][1] = dt2;
  Fk2[1][0] = 0.0;
  Fk2[1][1] = 1.0;
  Qk2[0][0] = (dt2*dt2*dt2*dt2)/4.0;
  Qk2[0][1] = (dt2*dt2*dt2)/2.0;
  Qk2[1][0] = (dt2*dt2*dt2)/2.0;
  Qk2[1][1] = dt2*dt2;
  Hk2[0][0] = 1.0;
  Hk2[0][1] = 0.0;
  Pk2[0][0] = 10.0;
  Pk2[0][1] = 0.0;
  Pk2[1][0] = 0.0;
  Pk2[1][1] = v_init;
  Pk_pre2[0][0] = Pk2[0][0];
  Pk_pre2[0][1] = Pk2[0][1];
  Pk_pre2[1][0] = Pk2[1][0];
  Pk_pre2[1][1] = Pk2[1][1];
  K2[0][0] = 0.0;
  K2[1][0] = 0.0;
  x_hat2[0][0] = 0.0;
  x_hat2[1][0] = 0.0;
  x_hat_pre2[0][0] = 0.0;
  x_hat_pre2[1][0] = 0.0;

  // Set pin modes for motor control
  pinMode(motor1PWMPin, OUTPUT);
  pinMode(motor1DirPin, OUTPUT);
  pinMode(motor2PWMPin, OUTPUT);
  pinMode(motor2DirPin, OUTPUT);

  // Set pin modes for encoder input
  pinMode(encoderPinA1, INPUT_PULLUP);
  pinMode(encoderPinB1, INPUT_PULLUP);
  pinMode(encoderPinA2, INPUT_PULLUP);
  pinMode(encoderPinB2, INPUT_PULLUP);

  // Attach interrupts for encoder input
  attachInterrupt(digitalPinToInterrupt(encoderPinA1), updateEncoder1, CHANGE);
  attachInterrupt(digitalPinToInterrupt(encoderPinA2), updateEncoder2, CHANGE);
  attachInterrupt(digitalPinToInterrupt(encoderPinB1), updateEncoder1, CHANGE);
  attachInterrupt(digitalPinToInterrupt(encoderPinB2), updateEncoder2, CHANGE);
}

// Main loop
void loop() {
  if (Serial.available() > 0) {
    int pwm_1 = Serial.read();
    int pwm_2 = Serial.read();
    int d1 = Serial.read();
    int d2 = Serial.read(); 
    if(d1 == 0) {
      d1 = -1;
    } 
    if(d2 == 0) {
      d2 = -1;
    }  
    currT_pid1 = micros();
    dt_pid1 = (currT_pid1-prevT_pid1)/1e6;    
    if(dt_pid1 >= 0.1){
      u1 = PID1(kp1,ki1,kd1,(pwm_1*d1*760)/255,motorSpeed1_f,dt_pid1);  
      prevT_pid1 = currT_pid1;    
    }  
    motor1Speed += (int) (u1);
    if(motor1Speed > 255){
      motor1Speed = 255;
    }
    if(motor1Speed < -255){
      motor1Speed = -255;
    }    
    if(motor1Speed >= 0){
      motor1Dir = 1;
    }
    if(motor1Speed < 0){
      motor1Dir = -1;
    }     
    currT_pid2 = micros();
    dt_pid2 = (currT_pid2-prevT_pid2)/1e6;    
    if(dt_pid2 >= 0.1){
      u2 = PID2(kp2,ki2,kd2,(pwm_2*d2*760)/255,motorSpeed2_f,dt_pid2); 
      prevT_pid2 = currT_pid2;     
    }  
    motor2Speed += (int) (u2);
    if(motor2Speed > 255){
      motor2Speed = 255;
    }  
    if(motor2Speed < -255){
      motor2Speed = -255;
    }  
    if(motor2Speed >= 0){
      motor2Dir = 1;
    }
    if(motor2Speed < 0){
      motor2Dir = -1;
    }   
    if(pwm_1*760 <= 1)   {
      motor1Speed = 0.0;
    }   
    if(pwm_2*760 <= 1)   {
      motor2Speed = 0.0;
    }                 
    controlMotor(motor1PWMPin, motor1DirPin, abs(motor1Speed), motor1Dir);
    controlMotor(motor2PWMPin, motor2DirPin, abs(motor2Speed), motor2Dir);
  } 
  if(Serial.available() < 0) {
    xt = 0;
    yt = 0;
    theta = 0;
    u1 = 0;
    u2 = 0;
  }
  dt1 = ((float)(currT1-prevT1))/1000.0; // [msec]     
  if((millis() - time_last_kalman1) >= interval_kalman) {
    if (dt1==0) dt1 = 10.0;
    prevT1 = currT1; 
    encoderPos1_dot = KALMAN1(encoderPos1,dt1);
    motorSpeed1 = 60*1000*(encoderPos1_dot)/(464.64); //RPM
    // Filltering
    motorSpeed1_f = 0.854*motorSpeed1_f + 0.0728*motorSpeed1 + 0.0728*motorSpeed1_pre;
    motorSpeed1_pre = motorSpeed1;
    time_last_kalman1 = millis();
  }
  dt2 = ((float)(currT2-prevT2))/1000.0; // [msec]     
  if((millis() - time_last_kalman2) >= interval_kalman) {
    if (dt2==0) dt2 = 10.0;
    prevT2 = currT2; 
    encoderPos2_dot = KALMAN2(encoderPos2,dt2);
    motorSpeed2 = 60*1000*(encoderPos2_dot)/(464.64); //RPM
    // Filltering
    motorSpeed2_f = 0.854*motorSpeed2_f + 0.0728*motorSpeed2 + 0.0728*motorSpeed2_pre;
    motorSpeed2_pre = motorSpeed2;
    time_last_kalman2 = millis();
  }
  currTl = micros();
  dtl = ((float)(currTl-prevTl))/1000.0; // [msec]     
  if((millis() - time_last_l) >= 10) {
    if (dtl==0) dtl = 10.0;
    prevTl = currTl;
    dR = teeth_ratio*(motorSpeed2_f*464.64/60000)*dtl*pi*wheel_d/464.64; // in [mm]
    dL = teeth_ratio*(motorSpeed1_f*464.64/60000)*dtl*pi*wheel_d/464.64; // in [mm] 
    d = (dR + dL)/2;  
    d_theta = ((dR - dL)/(2*dw));
    d_theta = fmod(d_theta,2*pi); // in [rad]
    theta = theta_pre + d_theta;
    theta = fmod(theta,2*pi); // in [rad]
    xt = xt_pre + d*cos(theta_pre + d_theta/2); // in [mm]
    yt = yt_pre + d*sin(theta_pre + d_theta/2); // in [mm]
    theta_pre = theta;
    xt_pre = xt;
    yt_pre = yt;
    time_last_l = millis();
  }
  long cT_p = millis();
  dt_p = (cT_p-pT_p); // [msec]  `
  if (dt_p >= interval_print*40) {  
    pT_p = cT_p;
//Serial.println("Motor1 Speed: " + String(motorSpeed1_f) + " ; " + "Motor2 Speed: " + String(motorSpeed2_f));
    //Serial.println("x : " + String(xt) + " ; " + "y : " + String(yt) + " ; " + "theta : " + String(theta));
    Serial.print(xt, 3);
    Serial.print(";");
    Serial.print(yt, 3);
    Serial.print(";");
    Serial.println(theta, 3);
  }       
}
void updateEncoder1() {
  // Read the current state of encoder pins
  currT1 = micros();
  int encoderAState1 = digitalRead(encoderPinA1);
  int encoderBState1 = digitalRead(encoderPinB1);
  encoderPos1_pre = encoderPos1;
  // Determine the direction of rotation based on the change in encoder states
  if ((lastEncoderAState1 == LOW && lastEncoderBState1 == LOW && encoderAState1 == HIGH && encoderBState1 == LOW) ||
      (lastEncoderAState1 == HIGH && lastEncoderBState1 == LOW && encoderAState1 == HIGH && encoderBState1 == HIGH) ||
      (lastEncoderAState1 == HIGH && lastEncoderBState1 == HIGH && encoderAState1 == LOW && encoderBState1 == HIGH) ||
      (lastEncoderAState1 == LOW && lastEncoderBState1 == HIGH && encoderAState1 == LOW && encoderBState1 == LOW)) {
    encoderPos1--;
  } else {
    encoderPos1++;
  }

  // Update last encoder state
  lastEncoderAState1 = encoderAState1;
  lastEncoderBState1 = encoderBState1;
}

// Update encoder count for motor 2
void updateEncoder2() {
  currT2 = micros();
  // Read the current state of encoder pins
  int encoderAState2 = digitalRead(encoderPinA2);
  int encoderBState2 = digitalRead(encoderPinB2);
  encoderPos2_pre = encoderPos2;
  // Determine the direction of rotation based on the change in encoder states
  if ((lastEncoderAState2 == LOW && lastEncoderBState2 == LOW && encoderAState2 == HIGH && encoderBState2 == LOW) ||
      (lastEncoderAState2 == HIGH && lastEncoderBState2 == LOW && encoderAState2 == HIGH && encoderBState2 == HIGH) ||
      (lastEncoderAState2 == HIGH && lastEncoderBState2 == HIGH && encoderAState2 == LOW && encoderBState2 == HIGH) ||
      (lastEncoderAState2 == LOW && lastEncoderBState2 == HIGH && encoderAState2 == LOW && encoderBState2 == LOW)) {
    encoderPos2--;
  } else {
    encoderPos2++;
  }

  // Update last encoder state
  lastEncoderAState2 = encoderAState2;
  lastEncoderBState2 = encoderBState2;
}

// Function to control motor speed
void controlMotor(int motorPWMPin, int motorDirPin, int PWM, int Dir) {
  // Set motor direction
  if (Dir == 1) {
    digitalWrite(motorDirPin, LOW); // Set direction forward
    analogWrite(motorPWMPin,PWM); // Set PWM duty cycle
  } else {
    digitalWrite(motorDirPin, HIGH); // Set direction backward
    analogWrite(motorPWMPin, PWM); // Set PWM duty cycle
  }
}
float KALMAN1(float encoderPos1,float dt_1){
  Fk1[0][1] = dt_1;
  Qk1[0][0] = (dt_1*dt_1*dt_1*dt_1)/4.0;
  Qk1[0][1] = (dt_1*dt_1*dt_1)/2.0;
  Qk1[1][0] = (dt_1*dt_1*dt_1)/2.0;
  Qk1[1][1] = dt_1*dt_1;
  x_hat_pre1[0][0] = Fk1[0][0]*x_hat1[0][0] + Fk1[0][1]*x_hat1[1][0];
  x_hat_pre1[1][0] = Fk1[1][0]*x_hat1[0][0] + Fk1[1][1]*x_hat1[1][0];
  Pk_pre1[0][0] = Qk1[0][0] + Fk1[0][0]*(Fk1[0][0]*Pk1[0][0]+Fk1[0][1]*Pk1[1][0]) + Fk1[0][1]*(Fk1[0][0]*Pk1[0][1]+Fk1[0][1]*Pk1[1][1]);
  Pk_pre1[0][1] = Qk1[0][1] + Fk1[1][0]*(Fk1[0][0]*Pk1[0][0]+Fk1[0][1]*Pk1[1][0]) + Fk1[1][1]*(Fk1[0][0]*Pk1[0][1]+Fk1[0][1]*Pk1[1][1]);
  Pk_pre1[1][0] = Qk1[1][0] + Fk1[0][0]*(Fk1[1][0]*Pk1[0][0]+Fk1[1][1]*Pk1[1][0]) + Fk1[0][1]*(Fk1[1][0]*Pk1[0][1]+Fk1[1][1]*Pk1[1][1]);
  Pk_pre1[1][1] = Qk1[1][1] + Fk1[1][0]*(Fk1[1][0]*Pk1[0][0]+Fk1[1][1]*Pk1[1][0]) + Fk1[1][1]*(Fk1[1][0]*Pk1[0][1]+Fk1[1][1]*Pk1[1][1]);
  yk1 = encoderPos1-(Hk1[0][0]*x_hat_pre1[0][0] + Hk1[0][1]*x_hat_pre1[1][0]);  
  Sk1 = Rk1 + Hk1[0][0]*(Hk1[0][0]*Pk_pre1[0][0] + Hk1[0][1]*Pk_pre1[1][0]) + Hk1[0][1]*(Hk1[0][0]*Pk_pre1[0][1] + Hk1[0][1]*Pk_pre1[1][1]);
  K1[0][0] = (Hk1[0][0]*Pk_pre1[0][0] + Hk1[0][1]*Pk_pre1[0][1])/Sk1;
  K1[1][0] = (Hk1[0][0]*Pk_pre1[1][0] + Hk1[0][1]*Pk_pre1[1][1])/Sk1;
  x_hat1[0][0] = x_hat_pre1[0][0] + K1[0][0]*yk1;
  x_hat1[1][0] = x_hat_pre1[1][0] + K1[1][0]*yk1;
  Pk1[0][0] = -Pk_pre1[0][0]*(Hk1[0][0]*K1[0][0]-1) -Hk1[0][1]*K1[0][0]*Pk_pre1[1][0];
  Pk1[0][1] = -Pk_pre1[0][1]*(Hk1[0][0]*K1[0][0]-1) -Hk1[0][1]*K1[0][0]*Pk_pre1[1][1];
  Pk1[1][0] = -Pk_pre1[1][0]*(Hk1[0][1]*K1[1][0]-1) -Hk1[0][0]*K1[1][0]*Pk_pre1[0][0];
  Pk1[1][1] = -Pk_pre1[1][1]*(Hk1[0][1]*K1[1][0]-1) -Hk1[0][0]*K1[1][0]*Pk_pre1[0][1];
  return x_hat1[1][0]; // [encoderPos/msec]
}
float KALMAN2(float encoderPos2,float dt_2){
  Fk2[0][1] = dt_2;
  Qk2[0][0] = (dt_2*dt_2*dt_2*dt_2)/4.0;
  Qk2[0][1] = (dt_2*dt_2*dt_2)/2.0;
  Qk2[1][0] = (dt_2*dt_2*dt_2)/2.0;
  Qk2[1][1] = dt_2*dt_2;
  x_hat_pre2[0][0] = Fk2[0][0]*x_hat2[0][0] + Fk2[0][1]*x_hat2[1][0];
  x_hat_pre2[1][0] = Fk2[1][0]*x_hat2[0][0] + Fk2[1][1]*x_hat2[1][0];
  Pk_pre2[0][0] = Qk2[0][0] + Fk2[0][0]*(Fk2[0][0]*Pk2[0][0]+Fk2[0][1]*Pk2[1][0]) + Fk2[0][1]*(Fk2[0][0]*Pk2[0][1]+Fk2[0][1]*Pk2[1][1]);
  Pk_pre2[0][1] = Qk2[0][1] + Fk2[1][0]*(Fk2[0][0]*Pk2[0][0]+Fk2[0][1]*Pk2[1][0]) + Fk2[1][1]*(Fk2[0][0]*Pk2[0][1]+Fk2[0][1]*Pk2[1][1]);
  Pk_pre2[1][0] = Qk2[1][0] + Fk2[0][0]*(Fk2[1][0]*Pk2[0][0]+Fk2[1][1]*Pk2[1][0]) + Fk2[0][1]*(Fk2[1][0]*Pk2[0][1]+Fk2[1][1]*Pk2[1][1]);
  Pk_pre2[1][1] = Qk2[1][1] + Fk2[1][0]*(Fk2[1][0]*Pk2[0][0]+Fk2[1][1]*Pk2[1][0]) + Fk2[1][1]*(Fk2[1][0]*Pk2[0][1]+Fk2[1][1]*Pk2[1][1]);
  yk2 = encoderPos2-(Hk2[0][0]*x_hat_pre2[0][0] + Hk2[0][1]*x_hat_pre2[1][0]);  
  Sk2 = Rk2 + Hk2[0][0]*(Hk2[0][0]*Pk_pre2[0][0] + Hk2[0][1]*Pk_pre2[1][0]) + Hk2[0][1]*(Hk2[0][0]*Pk_pre2[0][1] + Hk2[0][1]*Pk_pre2[1][1]);
  K2[0][0] = (Hk2[0][0]*Pk_pre2[0][0] + Hk2[0][1]*Pk_pre2[0][1])/Sk2;
  K2[1][0] = (Hk2[0][0]*Pk_pre2[1][0] + Hk2[0][1]*Pk_pre2[1][1])/Sk2;
  x_hat2[0][0] = x_hat_pre2[0][0] + K2[0][0]*yk2;
  x_hat2[1][0] = x_hat_pre2[1][0] + K2[1][0]*yk2;
  Pk2[0][0] = -Pk_pre2[0][0]*(Hk2[0][0]*K2[0][0]-1) -Hk2[0][1]*K2[0][0]*Pk_pre2[1][0];
  Pk2[0][1] = -Pk_pre2[0][1]*(Hk2[0][0]*K2[0][0]-1) -Hk2[0][1]*K2[0][0]*Pk_pre2[1][1];
  Pk2[1][0] = -Pk_pre2[1][0]*(Hk2[0][1]*K2[1][0]-1) -Hk2[0][0]*K2[1][0]*Pk_pre2[0][0];
  Pk2[1][1] = -Pk_pre2[1][1]*(Hk2[0][1]*K2[1][0]-1) -Hk2[0][0]*K2[1][0]*Pk_pre2[0][1];
  return x_hat2[1][0]; // [encoderPos/msec]
}

float PID1(float kp1, float ki1, float kd1, float rpm1, float c_rpm1, float dt_pid1){
  error1 = rpm1 - c_rpm1;
  eintegral1 += error1*dt_pid1;
  dedt1 = (error1 - pre_error1)/dt_pid1;
  pre_error1 = error1;
  u1 = kp1*error1 + ki1*eintegral1 + kd1*dedt1;
  return u1;
}

float PID2(float kp2, float ki2, float kd2, float rpm2, float c_rpm2, float dt_pid2){
  error2 = rpm2 - c_rpm2;
  eintegral2 += error2*dt_pid2;
  dedt2 = (error2 - pre_error2)/dt_pid2;
  pre_error2 = error2;
  u2 = kp2*error2 + ki2*eintegral2 + kd2*dedt2;
  return u2;
}

