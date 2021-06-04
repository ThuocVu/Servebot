//Processing incoming serial data 
#include <Messenger.h>
//Contain definition of maximum limits of various data type
#include <limits.h>
//Messenger object
Messenger Messenger_Handler = Messenger();

                                  /*---------------------------------DEFINE---------------------------------*/
                                  
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
#define D 0.066
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
float motor_left_speed ;
float motor_right_speed ;

                                  /*---------------------------------SETUP---------------------------------*/
                                  
void setup() {
  //Init Serial port with 115200 baud rate
  Serial.begin(115200);  
  //Setup Encoders
  SetupEncoders();
  //Setup Motors
  SetupMotors();
  //Set up Messenger 
  Messenger_Handler.attach(OnMssageCompleted);
}

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

void Read_From_Serial()
{
   while(Serial.available() > 0)
    {
       data = Serial.read();
       Messenger_Handler.process(data);
    } 
}

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
                                  
                                  /*---------------------------------MAIN---------------------------------*/
                                  
void loop() {

    //Read from Serial port
     Read_From_Serial();
    //Send time information through serial port
    //Send encoders values through serial port
    Update_Encoders();
    //Update motor values with corresponding speed and send speed values through serial port
    Update_Motors();
}


                                  /*---------------------------------PID CONTROLLER---------------------------------*/
//PID for left motor                             
float Left_computePID(float input, float setpoint)
{
  Left_error = setpoint - input;
  Left_cumError += Left_error*elapsedTime;
  Left_rateError = (Left_error - Left_previousError)/elapsedTime;
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
  Right_cumError += Right_error*elapsedTime;
  Right_rateError = (Right_error - Right_previousError)/elapsedTime;
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
  
  currentTime = millis();
  elapsedTime = (float(currentTime - previousTime))/1000;
  
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

   previousTime = currentTime;
}
