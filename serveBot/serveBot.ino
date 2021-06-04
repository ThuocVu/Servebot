//Processing incoming serial data 
#include<Wire.h>
#include <math.h>
#include"MPU6050.h"
#include <Messenger.h>
//Contain definition of maximum limits of various data type
#include <limits.h>
//Messenger object
Messenger Messenger_Handler = Messenger();

                                  /*---------------------------------DEFINE---------------------------------*/

// Ultrasonic pins definition
const int Echo = 21, Trig = 20;
long duration, cm;

//-------------------
int16_t AcX, AcY, AcZ, Tmp, GyX, GyY, GyZ;
float axg, ayg, azg, gxrs, gyrs, gzrs;
float roll, pitch, yaw;
float SelfTest[6];
float gyroBias[3] = {0, 0, 0}, accelBias[3] = {0, 0, 0}; // Bias corrections for gyro and accelerometer

// parameters for 6 DoF sensor fusion calculations
float GyroMeasError = PI * (40.0f / 180.0f);     // gyroscope measurement error in rads/s (start at 60 deg/s), then reduce after ~10 s to 3
float beta = sqrt(3.0f / 4.0f) * GyroMeasError;  // compute beta
float GyroMeasDrift = PI * (2.0f / 180.0f);      // gyroscope measurement drift in rad/s/s (start at 0.0 deg/s/s)
float zeta = sqrt(3.0f / 4.0f) * GyroMeasDrift;  // compute zeta, the other free parameter in the Madgwick scheme usually set to a small or zero value
float deltat = 0.0f;                              // integration interval for both filter schemes
uint32_t lastUpdate = 0, firstUpdate = 0;         // used to calculate integration interval
uint32_t Now = 0;                                 // used to calculate integration interval
float q[4] = {1.0f, 0.0f, 0.0f, 0.0f};            // vector to hold quaternion

//parameter of controller PI
float kp = 200, ki = 1500, kd = 0;
float Left_error, Left_previousError = 0, Right_error, Right_previousError = 0;
float Left_cumError, Left_rateError, Right_cumError, Right_rateError;
float Left_volt, Right_volt;

//-------------------
unsigned long currentTime, previousTime = 0;
float elapsedTime;

//--------------------
float output_right_speed, output_left_speed;
#define encoderOutput 480
#define D 0.065
#define pi 3.1416

char data;
 
//Encoder pins definition
// Left encoder
#define Left_Encoder_PinA 2
#define Left_Encoder_PinB 9

volatile long Left_Encoder_Ticks;
volatile bool LeftEncoderBSet;

//Right Encoder
#define Right_Encoder_PinA 3
#define Right_Encoder_PinB 10
volatile long Right_Encoder_Ticks;
volatile bool RightEncoderBSet;

//Motor Pin definition
//Left Motor pins
#define A_1 7
#define B_1 4

//PWM 1 pin number
#define PWM_1 5

//Right Motor
#define A_2 8
#define B_2 12

//PWM 2 pin number
#define PWM_2 6

//Motor left and right speed from PC
float motor_left_speed;
float motor_right_speed;

                                  /*---------------------------------SETUP---------------------------------*/
                                  
void setup() {
  //Init Serial port with 115200 baud rate
  Serial.begin(115200);
  //Setup Encoders
  SetupEncoders();
  //Setup Motors
  SetupMotors();
  //Setup Ultrasonic sensor
//  SetupUltrasonic();
  //Setup imu
  SetupImu();
  //Setup Messenger 
  Messenger_Handler.attach(OnMssageCompleted);
}

/*-------------------------------*/

//void SetupUltrasonic()
//{
//    // Configure the trigger pin to output mode
//    pinMode(Trig, OUTPUT);
//    // Configure the echo pin to input mode
//    pinMode(Echo, INPUT);
//}

/*-------------------------------*/

//Setup mpu6050() function
void SetupImu() {
  //I2C comunication
  Wire.begin();
  Wire.setClock(400000); // 400kHz I2C clock. Comment this line if having compilation difficulties

  // Self Test
  MPU6050SelfTest(SelfTest);

  // Calibrate MPU6050
  calibrateMPU6050(gyroBias, accelBias); // Calibrate gyro and accelerometers, load biases in bias registers

  // Initialize MPU6050
  MPU6050_Init();
}

/*-------------------------------*/

//SetupEncoders() Definition
void SetupEncoders()
{
  // Quadrature encoders
  // Left encoder
  pinMode(Left_Encoder_PinA, INPUT);      // sets pin A as input  
  pinMode(Left_Encoder_PinB, INPUT);      // sets pin B as input
  //Attaching interrupt in Left_Enc_PinA.
  attachInterrupt(digitalPinToInterrupt(Left_Encoder_PinA), do_Left_Encoder, RISING);

  // Right encoder
  pinMode(Right_Encoder_PinA, INPUT);      // sets pin A as input
  pinMode(Right_Encoder_PinB, INPUT);      // sets pin B as input
  //Attaching interrupt in Right_Enc_PinA.
  attachInterrupt(digitalPinToInterrupt(Right_Encoder_PinA), do_Right_Encoder, RISING);

  // Set original value for encoder
  Left_Encoder_Ticks = 0;
  Right_Encoder_Ticks = 0;
}

/*-------------------------------*/

//Setup Motors() function
void SetupMotors()
{
 
 //Left motor
 pinMode(A_1,OUTPUT);
 pinMode(B_1,OUTPUT); 
 

 //Right Motor
 pinMode(A_2,OUTPUT);
 pinMode(B_2,OUTPUT);  
  
 output_right_speed = 0;
 output_left_speed = 0;
}

                                  /*---------------------------------COMUNICATION---------------------------------*/
                                  
//OnMssg Complete function definition
void OnMssageCompleted()
{
  char reset[] = "r";
  char set_speed[] = "s";

  if(Messenger_Handler.checkString(reset))
  {
     Serial.println("Reset Done"); 
  }

  if(Messenger_Handler.checkString(set_speed))
  {
     Set_Speed();
     return; 
  }
}

/*-------------------------------*/

void Read_From_Serial()
{
   while(Serial.available() > 0)
    {
       data = Serial.read();
       Messenger_Handler.process(data);
    } 
}

/*-------------------------------*/

//Set speed
void Set_Speed()
{
  motor_left_speed = Messenger_Handler.readDouble();
  motor_right_speed = Messenger_Handler.readDouble();

}

                                  /*---------------------------------PROGRAM---------------------------------*/


//do_Left_Encoder() Definitions
void do_Left_Encoder()
{
  
   // Test transition; since the interrupt will only fire on 'rising' we don't need to read pin A
  LeftEncoderBSet = digitalRead(Left_Encoder_PinB);   // read the input pin
  if(LeftEncoderBSet > 0)
  {
    Left_Encoder_Ticks ++;
  }
  else
  {
    Left_Encoder_Ticks --;
  }
   
}

/*-------------------------------*/

//do_Right_Encoder() Definitions
void do_Right_Encoder()
{
  RightEncoderBSet = digitalRead(Right_Encoder_PinB);   // read the input pin
  if(RightEncoderBSet > 0)
  {
    Right_Encoder_Ticks --;
  }
  else
  {
    Right_Encoder_Ticks ++;
  }
}

/*-------------------------------*/

//Will update both encoder value through serial port
void Update_Encoders()
 {
   Serial.print("e");
   Serial.print("\t");
   Serial.print(Left_Encoder_Ticks);
   Serial.print("\t");
   Serial.print(Right_Encoder_Ticks);
   Serial.print("\n");
  
 }

/*-------------------------------*/

void Update_Motors()
{
  compute_Wheel(output_right_speed, motor_right_speed, output_left_speed, motor_left_speed);

  Serial.print("s");
  Serial.print("\t");
  Serial.print(output_left_speed);
  Serial.print("\t");
  Serial.print(output_right_speed);  
  Serial.print("\n");
}

/*-------------------------------*/

void Update_Imu()
{
  // Get raw data
  mpu6050_GetData();

  // Update raw data to Quaternion form
  mpu6050_updateQuaternion();

//  Now = micros();
//  deltat = ((Now - lastUpdate)/1000000.0f); // set integration time by time elapsed since last filter update
//  lastUpdate = Now;
    
  // Compute data
  MadgwickQuaternionUpdate(axg, ayg, azg, gxrs, gyrs, gzrs);

  // Print Quaternion to Serial Monitor
  Serial.print("i");Serial.print("\t");
  Serial.print(q[0]); Serial.print("\t");
  Serial.print(q[1]); Serial.print("\t");
  Serial.print(q[2]); Serial.print("\t");
  Serial.print(q[3]);
  Serial.print("\n");
}

/*-------------------------------*/

//void Update_Ultrasonic()
//{
//    digitalWrite(Trig, LOW);
//    delayMicroseconds(2);
//    digitalWrite(Trig, HIGH);
//    delayMicroseconds(10);
//
//    duration = pulseIn(Echo, HIGH);
//    // Convert duration -> cm
//    cm = durationTocm(duration);
//
//    // Sending through serial port
//    Serial.print("u");
//    Serial.print("\t");
//    Serial.print(cm);
//    Serial.print("\n");
//}

/*-------------------------------*/

//Time fucntion
void Update_Time()
{
  Now = micros();
  deltat = ((Now - lastUpdate)/1000000.0f); // set integration time by time elapsed since last filter update
  lastUpdate = Now;
}

/*-------------------------------*/

void moveRightMotor(float rightServoValue)
{
  if (rightServoValue>0)
  {
       
 digitalWrite(A_2,LOW);
 digitalWrite(B_2,HIGH);
 analogWrite(PWM_2,rightServoValue);
    
  }
  else if(rightServoValue<0)
  {
 digitalWrite(A_2,HIGH);
 digitalWrite(B_2,LOW);
 analogWrite(PWM_2,abs(rightServoValue));
 
  }
  
  else if(rightServoValue == 0)
  {
 digitalWrite(A_2,HIGH);
 digitalWrite(B_2,HIGH);
    
    
  }
}

/*-------------------------------*/

void moveLeftMotor(float leftServoValue)
{
 if (leftServoValue > 0)
  {
digitalWrite(A_1,HIGH);
digitalWrite(B_1,LOW);
analogWrite(PWM_1,leftServoValue);
  }
  else if(leftServoValue < 0)
  {
 digitalWrite(A_1,LOW);
 digitalWrite(B_1,HIGH);
 analogWrite(PWM_1,abs(leftServoValue));

  }
  else if(leftServoValue == 0)
  {

   digitalWrite(A_1,HIGH);
   digitalWrite(B_1,HIGH);
  
   }  
}

/*-------------------------------*/

// Convert microseconds to centimeter
//long durationTocm(long microseconds)
//{
//    return (microseconds/29)/2;
//}
                                  
                                  /*---------------------------------MAIN---------------------------------*/
                                  
void loop() {

    //Read from Serial port
    Read_From_Serial();
    //Update time
    Update_Time();
    //Update encoder values
    Update_Encoders();
    //Update motor values with corresponding speed and send speed values through serial port
    Update_Motors();
    //Update ultrasonic sensor - unit cm
//    Update_Ultrasonic();
    //Update quaternion values w, x, y, z
    Update_Imu();
}

                                  /*---------------------------------PID CONTROLLER---------------------------------*/
//PID for left motor                             
float Left_computePID(float input, float setpoint)
{
  Left_error = setpoint - input;
  Left_cumError += Left_error*deltat;
  Left_rateError = (Left_error - Left_previousError)/deltat;
  Left_volt = kp*Left_error + ki*Left_cumError + kd*Left_rateError;

  if (Left_volt > 255)
  {
    Left_volt = 255;
  }
  if(Left_volt < -255)
  {
    Left_volt = -255;
  }
  Left_previousError = Left_error;
  return Left_volt; 
}
//PID for right motor
float Right_computePID(float input, float setpoint)
{
  Right_error = setpoint - input;
  Right_cumError += Right_error*deltat;
  Right_rateError = (Right_error - Right_previousError)/deltat;
  Right_volt = kp*Right_error + ki*Right_cumError + kd*Right_rateError;

  if (Right_volt > 255)
  {
    Right_volt = 255;
  }
  if(Right_volt < -255)
  {
    Right_volt = -255;
  }
  Right_previousError = Right_error;
  return Right_volt; 
}
// Caculate velocity (m/s)
void compute_Wheel(float R_input, float R_setpoint, float L_input, float L_setpoint)
{
  volatile long Right_Encoder_Prev = Right_Encoder_Ticks;
  volatile long Left_Encoder_Prev = Left_Encoder_Ticks;
  
  delay(50);
  
//  currentTime = millis();
//  elapsedTime = (float(currentTime - previousTime))/1000;
//  
  float L_pwmVal = Left_computePID(L_input, L_setpoint);
  moveLeftMotor(L_pwmVal);
  float R_pwmVal = Right_computePID(R_input, R_setpoint);
  moveRightMotor(R_pwmVal);
  
  volatile long Right_Encoder_Curr = Right_Encoder_Ticks;
  volatile long Left_Encoder_Curr = Left_Encoder_Ticks;

  float Right_Encoder_Val = float(Right_Encoder_Curr - Right_Encoder_Prev);
  float R_rpm = (1200*Right_Encoder_Val)/encoderOutput;
  output_right_speed = R_rpm*D*pi/60;
  
  float Left_Encoder_Val = float(Left_Encoder_Curr - Left_Encoder_Prev);
  float L_rpm = (1200*Left_Encoder_Val)/encoderOutput;
  output_left_speed = L_rpm*D*pi/60;

//   previousTime = currentTime;
}

                                  /*---------------------------------IMU CONFIGURE---------------------------------*/

void MPU6050_Init(){
  // MPU6050 Initializing & Reset
  writeByte(MPU6050_DEFAULT_ADDRESS, MPU6050_RA_PWR_MGMT_1, 0x00); // set to zero (wakes up the MPU-6050)

  // MPU6050 Clock Type
  writeByte(MPU6050_DEFAULT_ADDRESS, MPU6050_RA_PWR_MGMT_1, 0x01); // Selection Clock 'PLL with X axis gyroscope reference'

  // MPU6050 Set sample rate = gyroscope output rate/(1 + SMPLRT_DIV) for DMP
  //writeByte(MPU6050_DEFAULT_ADDRESS, MPU6050_RA_SMPLRT_DIV, 0x00); // Default is 1KHz // example 0x04 is 200Hz

  // MPU6050 Gyroscope Configuration Setting
  /* Wire.write(0x00); // FS_SEL=0, Full Scale Range = +/- 250 [degree/sec]
     Wire.write(0x08); // FS_SEL=1, Full Scale Range = +/- 500 [degree/sec]
     Wire.write(0x10); // FS_SEL=2, Full Scale Range = +/- 1000 [degree/sec]
     Wire.write(0x18); // FS_SEL=3, Full Scale Range = +/- 2000 [degree/sec]   */
  writeByte(MPU6050_DEFAULT_ADDRESS, MPU6050_RA_GYRO_CONFIG, 0x00); // FS_SEL=3

  // MPU6050 Accelerometer Configuration Setting
  /* Wire.write(0x00); // AFS_SEL=0, Full Scale Range = +/- 2 [g]
     Wire.write(0x08); // AFS_SEL=1, Full Scale Range = +/- 4 [g]
     Wire.write(0x10); // AFS_SEL=2, Full Scale Range = +/- 8 [g]
     Wire.write(0x18); // AFS_SEL=3, Full Scale Range = +/- 10 [g] */
  writeByte(MPU6050_DEFAULT_ADDRESS, MPU6050_RA_ACCEL_CONFIG, 0x00); // AFS_SEL=2

  // MPU6050 DLPF(Digital Low Pass Filter)
  /*Wire.write(0x00);     // Accel BW 260Hz, Delay 0ms / Gyro BW 256Hz, Delay 0.98ms, Fs 8KHz 
    Wire.write(0x01);     // Accel BW 184Hz, Delay 2ms / Gyro BW 188Hz, Delay 1.9ms, Fs 1KHz 
    Wire.write(0x02);     // Accel BW 94Hz, Delay 3ms / Gyro BW 98Hz, Delay 2.8ms, Fs 1KHz 
    Wire.write(0x03);     // Accel BW 44Hz, Delay 4.9ms / Gyro BW 42Hz, Delay 4.8ms, Fs 1KHz 
    Wire.write(0x04);     // Accel BW 21Hz, Delay 8.5ms / Gyro BW 20Hz, Delay 8.3ms, Fs 1KHz 
    Wire.write(0x05);     // Accel BW 10Hz, Delay 13.8ms / Gyro BW 10Hz, Delay 13.4ms, Fs 1KHz 
    Wire.write(0x06);     // Accel BW 5Hz, Delay 19ms / Gyro BW 5Hz, Delay 18.6ms, Fs 1KHz */
  writeByte(MPU6050_DEFAULT_ADDRESS, MPU6050_RA_CONFIG, 0x00); //Accel BW 260Hz, Delay 0ms / Gyro BW 256Hz, Delay 0.98ms, Fs 8KHz
}


void mpu6050_GetData() {
  uint8_t data_org[14]; // original data of accelerometer and gyro
  readBytes(MPU6050_DEFAULT_ADDRESS, MPU6050_RA_ACCEL_XOUT_H, 14, &data_org[0]);

  AcX = data_org[0] << 8 | data_org[1];  // 0x3B (ACCEL_XOUT_H) & 0x3C (ACCEL_XOUT_L)     
  AcY = data_org[2] << 8 | data_org[3];  // 0x3D (ACCEL_YOUT_H) & 0x3E (ACCEL_YOUT_L)
  AcZ = data_org[4] << 8 | data_org[5];  // 0x3F (ACCEL_ZOUT_H) & 0x40 (ACCEL_ZOUT_L)
  Tmp = data_org[6] << 8 | data_org[7];  // 0x41 (TEMP_OUT_H) & 0x42 (TEMP_OUT_L)
  GyX = data_org[8] << 8 | data_org[9];  // 0x43 (GYRO_XOUT_H) & 0x44 (GYRO_XOUT_L)
  GyY = data_org[10] << 8 | data_org[11];  // 0x45 (GYRO_YOUT_H) & 0x46 (GYRO_YOUT_L)
  GyZ = data_org[12] << 8 | data_org[13];  // 0x47 (GYRO_ZOUT_H) & 0x48 (GYRO_ZOUT_L)
}


void mpu6050_updateQuaternion() {
  axg = (float)(AcX - MPU6050_AXOFFSET) / MPU6050_AXGAIN;
  ayg = (float)(AcY - MPU6050_AYOFFSET) / MPU6050_AYGAIN;
  azg = (float)(AcZ - MPU6050_AZOFFSET) / MPU6050_AZGAIN;
  gxrs = (float)(GyX - MPU6050_GXOFFSET) / MPU6050_GXGAIN * 0.01745329; //degree to radians
  gyrs = (float)(GyY - MPU6050_GYOFFSET) / MPU6050_GYGAIN * 0.01745329; //degree to radians
  gzrs = (float)(GyZ - MPU6050_GZOFFSET) / MPU6050_GZGAIN * 0.01745329; //degree to radians
  // Degree to Radians Pi / 180 = 0.01745329 0.01745329251994329576923690768489
}


void MadgwickQuaternionUpdate(float ax, float ay, float az, float gx, float gy, float gz)
{
    float q1 = q[0], q2 = q[1], q3 = q[2], q4 = q[3];         // short name local variable for readability
    float norm;                                               // vector norm
    float f1, f2, f3;                                         // objetive funcyion elements
    float J_11or24, J_12or23, J_13or22, J_14or21, J_32, J_33; // objective function Jacobian elements
    float qDot1, qDot2, qDot3, qDot4;
    float hatDot1, hatDot2, hatDot3, hatDot4;
    float gerrx, gerry, gerrz, gbiasx, gbiasy, gbiasz;        // gyro bias error

    // Auxiliary variables to avoid repeated arithmetic
    float _halfq1 = 0.5f * q1;
    float _halfq2 = 0.5f * q2;
    float _halfq3 = 0.5f * q3;
    float _halfq4 = 0.5f * q4;
    float _2q1 = 2.0f * q1;
    float _2q2 = 2.0f * q2;
    float _2q3 = 2.0f * q3;
    float _2q4 = 2.0f * q4;
    float _2q1q3 = 2.0f * q1 * q3;
    float _2q3q4 = 2.0f * q3 * q4;

    // Normalise accelerometer measurement
    norm = sqrt(ax * ax + ay * ay + az * az);
    if (norm == 0.0f) return; // handle NaN
    norm = 1.0f/norm;
    ax *= norm;
    ay *= norm;
    az *= norm;
    
    // Compute the objective function and Jacobian
    f1 = _2q2 * q4 - _2q1 * q3 - ax;
    f2 = _2q1 * q2 + _2q3 * q4 - ay;
    f3 = 1.0f - _2q2 * q2 - _2q3 * q3 - az;
    J_11or24 = _2q3;
    J_12or23 = _2q4;
    J_13or22 = _2q1;
    J_14or21 = _2q2;
    J_32 = 2.0f * J_14or21;
    J_33 = 2.0f * J_11or24;
  
    // Compute the gradient (matrix multiplication)
    hatDot1 = J_14or21 * f2 - J_11or24 * f1;
    hatDot2 = J_12or23 * f1 + J_13or22 * f2 - J_32 * f3;
    hatDot3 = J_12or23 * f2 - J_33 *f3 - J_13or22 * f1;
    hatDot4 = J_14or21 * f1 + J_11or24 * f2;
    
    // Normalize the gradient
    norm = sqrt(hatDot1 * hatDot1 + hatDot2 * hatDot2 + hatDot3 * hatDot3 + hatDot4 * hatDot4);
    hatDot1 /= norm;
    hatDot2 /= norm;
    hatDot3 /= norm;
    hatDot4 /= norm;
    
    // Compute estimated gyroscope biases
    gerrx = _2q1 * hatDot2 - _2q2 * hatDot1 - _2q3 * hatDot4 + _2q4 * hatDot3;
    gerry = _2q1 * hatDot3 + _2q2 * hatDot4 - _2q3 * hatDot1 - _2q4 * hatDot2;
    gerrz = _2q1 * hatDot4 - _2q2 * hatDot3 + _2q3 * hatDot2 - _2q4 * hatDot1;
    
    // Compute and remove gyroscope biases
    gbiasx += gerrx * deltat * zeta;
    gbiasy += gerry * deltat * zeta;
    gbiasz += gerrz * deltat * zeta;
    gx -= gbiasx;
    gy -= gbiasy;
    gz -= gbiasz;
    
    // Compute the quaternion derivative
    qDot1 = -_halfq2 * gx - _halfq3 * gy - _halfq4 * gz;
    qDot2 =  _halfq1 * gx + _halfq3 * gz - _halfq4 * gy;
    qDot3 =  _halfq1 * gy - _halfq2 * gz + _halfq4 * gx;
    qDot4 =  _halfq1 * gz + _halfq2 * gy - _halfq3 * gx;

    // Compute then integrate estimated quaternion derivative
    q1 += (qDot1 -(beta * hatDot1)) * deltat;
    q2 += (qDot2 -(beta * hatDot2)) * deltat;
    q3 += (qDot3 -(beta * hatDot3)) * deltat;
    q4 += (qDot4 -(beta * hatDot4)) * deltat;

    // Normalize the quaternion
    norm = sqrt(q1 * q1 + q2 * q2 + q3 * q3 + q4 * q4);    // normalise quaternion
    norm = 1.0f/norm;
    q[0] = q1 * norm;
    q[1] = q2 * norm;
    q[2] = q3 * norm;
    q[3] = q4 * norm;
}


void mpu6050_getRollPitchYaw() {
//  yaw   = atan2(2.0f * (q[1] * q[2] + q[0] * q[3]), q[0] * q[0] + q[1] * q[1] - q[2] * q[2] - q[3] * q[3]) * 57.29577951;   
//  pitch = -asin(2.0f * (q[1] * q[3] - q[0] * q[2])) * 57.29577951;
//  roll  = atan2(2.0f * (q[0] * q[1] + q[2] * q[3]), q[0] * q[0] - q[1] * q[1] - q[2] * q[2] + q[3] * q[3]) * 57.29577951;
  yaw   = -atan2(2.0f * (q[1] * q[2] + q[0] * q[3]), q[0] * q[0] + q[1] * q[1] - q[2] * q[2] - q[3] * q[3]) * 57.29577951;   
  pitch = asin(2.0f * (q[1] * q[3] - q[0] * q[2])) * 57.29577951;
  roll  = atan2(2.0f * (q[0] * q[1] + q[2] * q[3]), q[0] * q[0] - q[1] * q[1] - q[2] * q[2] + q[3] * q[3]) * 57.29577951;
}


// Accelerometer and gyroscope self test; check calibration wrt factory settings
void MPU6050SelfTest(float * destination) // Should return percent deviation from factory trim values, +/- 14 or less deviation is a pass
{
   uint8_t rawData[4];
   uint8_t selfTest[6];
   float factoryTrim[6];
   
   // Configure the accelerometer for self-test
   writeByte(MPU6050_DEFAULT_ADDRESS, MPU6050_RA_ACCEL_CONFIG, 0xF0); // Enable self test on all three axes and set accelerometer range to +/- 8 g
   writeByte(MPU6050_DEFAULT_ADDRESS, MPU6050_RA_GYRO_CONFIG,  0xE0); // Enable self test on all three axes and set gyro range to +/- 250 degrees/s
   delay(250);  // Delay a while to let the device execute the self-test
   rawData[0] = readByte(MPU6050_DEFAULT_ADDRESS, MPU6050_RA_SELF_TEST_X); // X-axis self-test results
   rawData[1] = readByte(MPU6050_DEFAULT_ADDRESS, MPU6050_RA_SELF_TEST_Y); // Y-axis self-test results
   rawData[2] = readByte(MPU6050_DEFAULT_ADDRESS, MPU6050_RA_SELF_TEST_Z); // Z-axis self-test results
   rawData[3] = readByte(MPU6050_DEFAULT_ADDRESS, MPU6050_RA_SELF_TEST_A); // Mixed-axis self-test results
   // Extract the acceleration test results first
   selfTest[0] = (rawData[0] >> 3) | (rawData[3] & 0x30) >> 4 ; // XA_TEST result is a five-bit unsigned integer
   selfTest[1] = (rawData[1] >> 3) | (rawData[3] & 0x0C) >> 2 ; // YA_TEST result is a five-bit unsigned integer
   selfTest[2] = (rawData[2] >> 3) | (rawData[3] & 0x03) ; // ZA_TEST result is a five-bit unsigned integer
   // Extract the gyration test results first
   selfTest[3] = rawData[0]  & 0x1F ; // XG_TEST result is a five-bit unsigned integer
   selfTest[4] = rawData[1]  & 0x1F ; // YG_TEST result is a five-bit unsigned integer
   selfTest[5] = rawData[2]  & 0x1F ; // ZG_TEST result is a five-bit unsigned integer   
   // Process results to allow final comparison with factory set values
   factoryTrim[0] = (4096.0*0.34)*(pow( (0.92/0.34) , (((float)selfTest[0] - 1.0)/30.0))); // FT[Xa] factory trim calculation
   factoryTrim[1] = (4096.0*0.34)*(pow( (0.92/0.34) , (((float)selfTest[1] - 1.0)/30.0))); // FT[Ya] factory trim calculation
   factoryTrim[2] = (4096.0*0.34)*(pow( (0.92/0.34) , (((float)selfTest[2] - 1.0)/30.0))); // FT[Za] factory trim calculation
   factoryTrim[3] =  ( 25.0*131.0)*(pow( 1.046 , ((float)selfTest[3] - 1.0) ));             // FT[Xg] factory trim calculation
   factoryTrim[4] =  (-25.0*131.0)*(pow( 1.046 , ((float)selfTest[4] - 1.0) ));             // FT[Yg] factory trim calculation
   factoryTrim[5] =  ( 25.0*131.0)*(pow( 1.046 , ((float)selfTest[5] - 1.0) ));             // FT[Zg] factory trim calculation
   
 //  Output self-test results and factory trim calculation if desired
 //  Serial.println(selfTest[0]); Serial.println(selfTest[1]); Serial.println(selfTest[2]);
 //  Serial.println(selfTest[3]); Serial.println(selfTest[4]); Serial.println(selfTest[5]);
 //  Serial.println(factoryTrim[0]); Serial.println(factoryTrim[1]); Serial.println(factoryTrim[2]);
 //  Serial.println(factoryTrim[3]); Serial.println(factoryTrim[4]); Serial.println(factoryTrim[5]);

 // Report results as a ratio of (STR - FT)/FT; the change from Factory Trim of the Self-Test Response
 // To get to percent, must multiply by 100 and subtract result from 100
   for (int i = 0; i < 6; i++) {
     destination[i] = 100.0 + 100.0*((float)selfTest[i] - factoryTrim[i])/factoryTrim[i]; // Report percent differences
   }   
}


// Function which accumulates gyro and accelerometer data after device initialization. It calculates the average
// of the at-rest readings and then loads the resulting offsets into accelerometer and gyro bias registers.
void calibrateMPU6050(float * dest1, float * dest2)
{  
  uint8_t data[12]; // data array to hold accelerometer and gyro x, y, z, data
  uint16_t ii, packet_count, fifo_count;
  int32_t gyro_bias[3] = {0, 0, 0}, accel_bias[3] = {0, 0, 0};
  
// reset device, reset all registers, clear gyro and accelerometer bias registers
  writeByte(MPU6050_DEFAULT_ADDRESS, MPU6050_RA_PWR_MGMT_1, 0x80); // Write a one to bit 7 reset bit; toggle reset device
  delay(100);  
   
// get stable time source
// Set clock source to be PLL with x-axis gyroscope reference, bits 2:0 = 001
  writeByte(MPU6050_DEFAULT_ADDRESS, MPU6050_RA_PWR_MGMT_1, 0x01);  
  writeByte(MPU6050_DEFAULT_ADDRESS, MPU6050_RA_PWR_MGMT_2, 0x00); 
  delay(200);
  
// Configure device for bias calculation
  writeByte(MPU6050_DEFAULT_ADDRESS, MPU6050_RA_INT_ENABLE, 0x00);   // Disable all interrupts
  writeByte(MPU6050_DEFAULT_ADDRESS, MPU6050_RA_FIFO_EN, 0x00);      // Disable FIFO
  writeByte(MPU6050_DEFAULT_ADDRESS, MPU6050_RA_PWR_MGMT_1, 0x00);   // Turn on internal clock source
  writeByte(MPU6050_DEFAULT_ADDRESS, MPU6050_RA_I2C_MST_CTRL, 0x00); // Disable I2C master
  writeByte(MPU6050_DEFAULT_ADDRESS, MPU6050_RA_USER_CTRL, 0x00);    // Disable FIFO and I2C master modes
  writeByte(MPU6050_DEFAULT_ADDRESS, MPU6050_RA_USER_CTRL, 0x0C);    // Reset FIFO and DMP
  delay(15);
  
// Configure MPU6050 gyro and accelerometer for bias calculation
  writeByte(MPU6050_DEFAULT_ADDRESS, MPU6050_RA_CONFIG, 0x01);      // Set low-pass filter to 188 Hz
  writeByte(MPU6050_DEFAULT_ADDRESS, MPU6050_RA_SMPLRT_DIV, 0x00);  // Set sample rate to 1 kHz
  writeByte(MPU6050_DEFAULT_ADDRESS, MPU6050_RA_GYRO_CONFIG, 0x00);  // Set gyro full-scale to 250 degrees per second, maximum sensitivity
  writeByte(MPU6050_DEFAULT_ADDRESS, MPU6050_RA_ACCEL_CONFIG, 0x00); // Set accelerometer full-scale to 2 g, maximum sensitivity
 
  uint16_t  gyrosensitivity  = 131;   // = 131 LSB/degrees/sec
  uint16_t  accelsensitivity = 16384;  // = 16384 LSB/g

// Configure FIFO to capture accelerometer and gyro data for bias calculation
  writeByte(MPU6050_DEFAULT_ADDRESS, MPU6050_RA_USER_CTRL, 0x40);   // Enable FIFO  
  writeByte(MPU6050_DEFAULT_ADDRESS, MPU6050_RA_FIFO_EN, 0x78);     // Enable gyro and accelerometer sensors for FIFO  (max size 1024 bytes in MPU-6050)
  delay(80); // accumulate 80 samples in 80 milliseconds = 960 bytes

// At end of sample accumulation, turn off FIFO sensor read
  writeByte(MPU6050_DEFAULT_ADDRESS, MPU6050_RA_FIFO_EN, 0x00);        // Disable gyro and accelerometer sensors for FIFO
  readBytes(MPU6050_DEFAULT_ADDRESS, MPU6050_RA_FIFO_COUNTH, 2, &data[0]); // read FIFO sample count
  fifo_count = ((uint16_t)data[0] << 8) | data[1];
  packet_count = fifo_count/12;// How many sets of full gyro and accelerometer data for averaging

  for (ii = 0; ii < packet_count; ii++) {
    int16_t accel_temp[3] = {0, 0, 0}, gyro_temp[3] = {0, 0, 0};
    readBytes(MPU6050_DEFAULT_ADDRESS, MPU6050_RA_FIFO_R_W, 12, &data[0]); // read data for averaging
    accel_temp[0] = (int16_t) (((int16_t)data[0] << 8) | data[1]  ) ;  // Form signed 16-bit integer for each sample in FIFO
    accel_temp[1] = (int16_t) (((int16_t)data[2] << 8) | data[3]  ) ;
    accel_temp[2] = (int16_t) (((int16_t)data[4] << 8) | data[5]  ) ;    
    gyro_temp[0]  = (int16_t) (((int16_t)data[6] << 8) | data[7]  ) ;
    gyro_temp[1]  = (int16_t) (((int16_t)data[8] << 8) | data[9]  ) ;
    gyro_temp[2]  = (int16_t) (((int16_t)data[10] << 8) | data[11]) ;
    
    accel_bias[0] += (int32_t) accel_temp[0]; // Sum individual signed 16-bit biases to get accumulated signed 32-bit biases
    accel_bias[1] += (int32_t) accel_temp[1];
    accel_bias[2] += (int32_t) accel_temp[2];
    gyro_bias[0]  += (int32_t) gyro_temp[0];
    gyro_bias[1]  += (int32_t) gyro_temp[1];
    gyro_bias[2]  += (int32_t) gyro_temp[2];
            
}
    accel_bias[0] /= (int32_t) packet_count; // Normalize sums to get average count biases
    accel_bias[1] /= (int32_t) packet_count;
    accel_bias[2] /= (int32_t) packet_count;
    gyro_bias[0]  /= (int32_t) packet_count;
    gyro_bias[1]  /= (int32_t) packet_count;
    gyro_bias[2]  /= (int32_t) packet_count;
    
  if(accel_bias[2] > 0L) {accel_bias[2] -= (int32_t) accelsensitivity;}  // Remove gravity from the z-axis accelerometer bias calculation
  else {accel_bias[2] += (int32_t) accelsensitivity;}
 
// Construct the gyro biases for push to the hardware gyro bias registers, which are reset to zero upon device startup
  data[0] = (-gyro_bias[0]/4  >> 8) & 0xFF; // Divide by 4 to get 32.9 LSB per deg/s to conform to expected bias input format
  data[1] = (-gyro_bias[0]/4)       & 0xFF; // Biases are additive, so change sign on calculated average gyro biases
  data[2] = (-gyro_bias[1]/4  >> 8) & 0xFF;
  data[3] = (-gyro_bias[1]/4)       & 0xFF;
  data[4] = (-gyro_bias[2]/4  >> 8) & 0xFF;
  data[5] = (-gyro_bias[2]/4)       & 0xFF;

// Push gyro biases to hardware registers
  writeByte(MPU6050_DEFAULT_ADDRESS, MPU6050_RA_XG_OFFS_USRH, data[0]);// might not be supported in MPU6050
  writeByte(MPU6050_DEFAULT_ADDRESS, MPU6050_RA_XG_OFFS_USRL, data[1]);
  writeByte(MPU6050_DEFAULT_ADDRESS, MPU6050_RA_YG_OFFS_USRH, data[2]);
  writeByte(MPU6050_DEFAULT_ADDRESS, MPU6050_RA_YG_OFFS_USRL, data[3]);
  writeByte(MPU6050_DEFAULT_ADDRESS, MPU6050_RA_ZG_OFFS_USRH, data[4]);
  writeByte(MPU6050_DEFAULT_ADDRESS, MPU6050_RA_ZG_OFFS_USRL, data[5]);

  dest1[0] = (float) gyro_bias[0]/(float) gyrosensitivity; // construct gyro bias in deg/s for later manual subtraction
  dest1[1] = (float) gyro_bias[1]/(float) gyrosensitivity;
  dest1[2] = (float) gyro_bias[2]/(float) gyrosensitivity;

// Construct the accelerometer biases for push to the hardware accelerometer bias registers. These registers contain
// factory trim values which must be added to the calculated accelerometer biases; on boot up these registers will hold
// non-zero values. In addition, bit 0 of the lower byte must be preserved since it is used for temperature
// compensation calculations. Accelerometer bias registers expect bias input as 2048 LSB per g, so that
// the accelerometer biases calculated above must be divided by 8.

  int32_t accel_bias_reg[3] = {0, 0, 0}; // A place to hold the factory accelerometer trim biases
  readBytes(MPU6050_DEFAULT_ADDRESS, MPU6050_RA_XA_OFFS_H, 2, &data[0]); // Read factory accelerometer trim values
  accel_bias_reg[0] = (int16_t) ((int16_t)data[0] << 8) | data[1];
  readBytes(MPU6050_DEFAULT_ADDRESS, MPU6050_RA_YA_OFFS_H, 2, &data[0]);
  accel_bias_reg[1] = (int16_t) ((int16_t)data[0] << 8) | data[1];
  readBytes(MPU6050_DEFAULT_ADDRESS, MPU6050_RA_ZA_OFFS_H, 2, &data[0]);
  accel_bias_reg[2] = (int16_t) ((int16_t)data[0] << 8) | data[1];
  
  uint32_t mask = 1uL; // Define mask for temperature compensation bit 0 of lower byte of accelerometer bias registers
  uint8_t mask_bit[3] = {0, 0, 0}; // Define array to hold mask bit for each accelerometer bias axis
  
  for(ii = 0; ii < 3; ii++) {
    if(accel_bias_reg[ii] & mask) mask_bit[ii] = 0x01; // If temperature compensation bit is set, record that fact in mask_bit
  }

  // Construct total accelerometer bias, including calculated average accelerometer bias from above
  accel_bias_reg[0] -= (accel_bias[0]/8); // Subtract calculated averaged accelerometer bias scaled to 2048 LSB/g (16 g full scale)
  accel_bias_reg[1] -= (accel_bias[1]/8);
  accel_bias_reg[2] -= (accel_bias[2]/8);
 
  data[0] = (accel_bias_reg[0] >> 8) & 0xFF;
  data[1] = (accel_bias_reg[0])      & 0xFF;
  data[1] = data[1] | mask_bit[0]; // preserve temperature compensation bit when writing back to accelerometer bias registers
  data[2] = (accel_bias_reg[1] >> 8) & 0xFF;
  data[3] = (accel_bias_reg[1])      & 0xFF;
  data[3] = data[3] | mask_bit[1]; // preserve temperature compensation bit when writing back to accelerometer bias registers
  data[4] = (accel_bias_reg[2] >> 8) & 0xFF;
  data[5] = (accel_bias_reg[2])      & 0xFF;
  data[5] = data[5] | mask_bit[2]; // preserve temperature compensation bit when writing back to accelerometer bias registers

  // Push accelerometer biases to hardware registers
  writeByte(MPU6050_DEFAULT_ADDRESS, MPU6050_RA_XA_OFFS_H, data[0]); // might not be supported in MPU6050
  writeByte(MPU6050_DEFAULT_ADDRESS, MPU6050_RA_XA_OFFS_L_TC, data[1]);
  writeByte(MPU6050_DEFAULT_ADDRESS, MPU6050_RA_YA_OFFS_H, data[2]);
  writeByte(MPU6050_DEFAULT_ADDRESS, MPU6050_RA_YA_OFFS_L_TC, data[3]);  
  writeByte(MPU6050_DEFAULT_ADDRESS, MPU6050_RA_ZA_OFFS_H, data[4]);
  writeByte(MPU6050_DEFAULT_ADDRESS, MPU6050_RA_ZA_OFFS_L_TC, data[5]);

// Output scaled accelerometer biases for manual subtraction in the main program
   dest2[0] = (float)accel_bias[0]/(float)accelsensitivity; 
   dest2[1] = (float)accel_bias[1]/(float)accelsensitivity;
   dest2[2] = (float)accel_bias[2]/(float)accelsensitivity;
}


void writeByte(uint8_t address, uint8_t subAddress, uint8_t data)
{
  Wire.beginTransmission(address);  // Initialize the Tx buffer
  Wire.write(subAddress);           // Put slave register address in Tx buffer
  Wire.write(data);                 // Put data in Tx buffer
  Wire.endTransmission();           // Send the Tx buffer
}


uint8_t readByte(uint8_t address, uint8_t subAddress)
{
  uint8_t data; // `data` will store the register data   
  Wire.beginTransmission(address);         // Initialize the Tx buffer
  Wire.write(subAddress);                  // Put slave register address in Tx buffer
  Wire.endTransmission(false);             // Send the Tx buffer, but send a restart to keep connection alive
  Wire.requestFrom(address, (uint8_t) 1);  // Read one byte from slave register address 
  data = Wire.read();                      // Fill Rx buffer with result
  return data;                             // Return data read from slave register
}


void readBytes(uint8_t address, uint8_t subAddress, uint8_t count, uint8_t * dest)
{  
  Wire.beginTransmission(address);   // Initialize the Tx buffer
  Wire.write(subAddress);            // Put slave register address in Tx buffer
  Wire.endTransmission(false);       // Send the Tx buffer, but send a restart to keep connection alive
  uint8_t i = 0;
        Wire.requestFrom(address, count);  // Read bytes from slave register address 
  while (Wire.available()) {
        dest[i++] = Wire.read(); }         // Put read results in the Rx buffer
}
