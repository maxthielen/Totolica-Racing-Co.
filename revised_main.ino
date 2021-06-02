//===============================================================================================
//-----------------------------------GOBAL VARS--------------------------------------------------
//===============================================================================================
#include <Arduino.h>
#include <Servo.h> 
#include <NewPing.h> //ultrasonic library
//----------------------------------MotorShield--------------------------------------------------
const int BRAKE = 0;
const int FORWARD = 1;
const int REVERSE = 2;

const int PWM_MOTOR_PIN = 5;
const int MOTOR_A1_PIN = 7;
const int MOTOR_B1_PIN = 12; //used to be 8

int motor_Speed = 150; //default motor speed
int motor_State = BRAKE; //default motor state

const int EN_PIN_1 = A0; //TODO:: check what these pins alter on the motor shield!
const int EN_PIN_2 = A1; //TODO:: check what these pins alter on the motor shield!

char readString[4];
//----------------------------------Flying Fish--------------------------------------------------
const int L_SENS_LEFT_PIN = 9; // Left Line Sensor //flipped
const int L_SENS_MID_PIN = 6; // Middle Line Sensor
const int L_SENS_RIGHT_PIN = 4; // Right Line Sensor//flipped

int sensors[3];
//------------------------------------Steering---------------------------------------------------
Servo m_servo;
const int STEER_SERVO_PIN = A2;

//steering directions
const int hard_left = 148;
const int left = 138;
const int slight_left = 128;
const int straight = 118;
const int slight_right = 108;
const int right = 98;
const int hard_right = 88;

int servo_angle = straight; //current steering direction of vehicle

int iterator = 0;

//--------------------------------------SONIC----------------------------------------------------
const byte trigPin1 = 10;    // TRIG pin (front)
const byte echoPin1 = 11;    // ECHO pin (front)
const byte trigPin2 = 2;    // TRIG pin (side)
const byte echoPin2 = 3;    // ECHO pin (side)
const int max_distance = 400;
const int avoid_distance = 65;
int current_distance;

NewPing sonar_top(trigPin1, echoPin1, max_distance);
NewPing sonar_bottom(trigPin2, echoPin2, max_distance);
//---------------------------------------PID-----------------------------------------------------
//double kp=2;
//double ki=5;
//double kd=1;
//
//unsigned long currentTime, previousTime;
//double elapsedTime;
//double error, lastError, cumError, rateError;
//double input, output, setPoint;
//===============================================================================================
//-------------------------------------SET UP----------------------------------------------------
//===============================================================================================
void setup() {
  Serial.begin(500000);
//----------------------------------MotorShield--------------------------------------------------
  pinMode(MOTOR_A1_PIN, OUTPUT);
  pinMode(MOTOR_B1_PIN, OUTPUT);
  pinMode(PWM_MOTOR_PIN, OUTPUT);
  
  pinMode(EN_PIN_1, OUTPUT);      // Uncomment these 4 lines to use the Enable pins
  pinMode(EN_PIN_2, OUTPUT);      // to enable/disable the device.  
                                  // To monitor for fault conditions instead, they 
                                  // would be defined as inputs
  digitalWrite(EN_PIN_1, HIGH);   // Set EN pins high to enable drivers
  digitalWrite(EN_PIN_2, HIGH);
//----------------------------------Flying Fish--------------------------------------------------
  pinMode(L_SENS_LEFT_PIN, INPUT); 
  pinMode(L_SENS_MID_PIN, INPUT); 
  pinMode(L_SENS_RIGHT_PIN, INPUT); 
//------------------------------------Steering---------------------------------------------------
  pinMode(STEER_SERVO_PIN, OUTPUT); 
  m_servo.attach(STEER_SERVO_PIN); 

  m_servo.write(servo_angle); //straight
//--------------------------------------SONIC----------------------------------------------------
  // configure the trigger pin to output mode
  pinMode(trigPin1, OUTPUT);
  pinMode(trigPin2, OUTPUT);
  // configure the echo pin to input mode
  pinMode(echoPin1, INPUT); 
  pinMode(echoPin2, INPUT);
//---------------------------------------PID-----------------------------------------------------
//  setPoint = 118;
}
//================================================================================================
//--------------------------------------LOOP------------------------------------------------------
//================================================================================================
void loop() { 
  follow_line();
  update_sensors();

//----------OUTPUT-------------
//  int test=sonar1.ping_cm();
//  Serial.println(test);
//  check_line();
//  int test1;
//  for(int i=0;i<3;i++){
//    Serial.print(sensors[i]);
//    Serial.print(" ");
//  }
//  Serial.println();
//  
//  delay(100);
//------------------------------  
//  int test=sonar1.ping_cm();
//  Serial.print("front ");
//  Serial.println(test);
//    int test2=sonar2.ping_cm();
//  Serial.print("side ");
//  Serial.println(test2);
//  if(sonar1.ping_cm()<avoid_distance&&sonar1.ping_cm()>0) avoid_obstical();
//  else follow_line();

  //delay(500);
}

//===============================================================================================
//-----------------------------COMPONENT Functions-----------------------------------------------
//===============================================================================================
void Motor_Cmd(int DIR, int PWM)//Function that writes to the motor
{
  if(DIR == FORWARD){ //drive forward
    digitalWrite(MOTOR_A1_PIN, LOW); 
    digitalWrite(MOTOR_B1_PIN, HIGH);
  }
  else if(DIR == REVERSE){ //drive reverse
    digitalWrite(MOTOR_A1_PIN, HIGH);
    digitalWrite(MOTOR_B1_PIN, LOW);      
  }
  else{ //stop
    digitalWrite(MOTOR_A1_PIN, LOW);
    digitalWrite(MOTOR_B1_PIN, LOW);            
  }
  analogWrite(PWM_MOTOR_PIN, PWM); 
}
void update_sensors(){
  current_distance=0;
 // while(current_distance==0){
    //current_distance=sonar_bottom.ping_cm();
    current_distance=sonar_top.ping_cm();
  //}
  Serial.println(current_distance);
  if(current_distance<avoid_distance&&current_distance!=0){
    avoid_obstical();
  }

   check_line(); //ir line sensors
}
//===============================================================================================
//------------------------------OBSTICAL Functions-----------------------------------------------
//===============================================================================================
void avoid_obstical(){
  Serial.println("Avoiding Obstical");
  Motor_Cmd(motor_State, 30);
  
    servo_angle = right;
    m_servo.write(servo_angle);
    
    //while(sonar2.ping_cm()<30);
    delay(500);
    
    servo_angle = left;
    m_servo.write(servo_angle);
    while(sensors[1]==1){//right ir sensor on black
        check_line();
    }
    Serial.println("Avoided Obstical");
}
//===============================================================================================
//--------------------------------LINE Functions-------------------------------------------------
//===============================================================================================
void check_line(){
  digitalRead(L_SENS_LEFT_PIN)==HIGH ? sensors[0]=1 : sensors[0]=0;
  digitalRead(L_SENS_MID_PIN)==HIGH ? sensors[1]=1 : sensors[1]=0;
  digitalRead(L_SENS_RIGHT_PIN)==HIGH ? sensors[2]=1 : sensors[2]=0; 

  int test1;
  for(int i=0;i<3;i++){
    Serial.print(sensors[i]);
    Serial.print(" ");
  }
  Serial.println();
}

void follow_line(){
  if(sensors[0]==1&&sensors[1]==0&&sensors[2]==1){//mid ir sensor on white
    motor_State = FORWARD;
    Motor_Cmd(motor_State, 30);
    
    servo_angle=straight;
    m_servo.write(servo_angle);
  }
  else if(sensors[0]==0&&sensors[1]==1&&sensors[2]==1){//left ir sensor on white
    Motor_Cmd(motor_State, 30);

    servo_angle = left;
    m_servo.write(servo_angle);
  }
  else if(sensors[0]==1&&sensors[1]==1&&sensors[2]==0){//right ir sensor on white
    Motor_Cmd(motor_State, 30);
    
    servo_angle = right; 
    m_servo.write(servo_angle);
  }
  else if(sensors[0]==1&&sensors[1]==1&&sensors[2]==1){//all ir sensors on black
    if(servo_angle==straight){ 
      Motor_Cmd(motor_State, 25);
    }
    else if(servo_angle>straight){
      servo_angle=hard_left;
      m_servo.write(servo_angle);
      while(sensors[1]==1){
        check_line();
      }
    }
    else if(servo_angle<straight){
      servo_angle=hard_right;
      m_servo.write(servo_angle);
      while(sensors[1]==1){
        check_line();
      }
    }
  }
}

//===============================================================================================
//--------------------------------------TRASH----------------------------------------------------
//===============================================================================================
//-------------------------------------9dof------------------------------------------------------
//#include "ICM_20948.h"  //gyroscope library
//#define WIRE_PORT Wire  // Your desired Wire port.(Used when "USE_SPI" is not defined)
//#define AD0_VAL   1    // The value of the last bit of the I2C address.
//ICM_20948_I2C myICM;  // Otherwise create an ICM_20948_I2C object
//-------------------------------------9dof------------------------------------------------------  
//  WIRE_PORT.begin();
//  WIRE_PORT.setClock(400000);
//
//  //myICM.enableDebugging();
//  bool initialized = false;
//  while( !initialized ){
//    myICM.begin( WIRE_PORT, AD0_VAL );
//
//
//    Serial.print( F("Initialization of the sensor returned: ") );
//    Serial.println( myICM.statusString() );
//    if( myICM.status != ICM_20948_Stat_Ok ){
//      Serial.println( "Trying again..." );
//      delay(500);
//    }else{
//      initialized = true;
//    }
//  }
//void check_9DOF()
//{
//  if( myICM.dataReady() ){
//    myICM.getAGMT();                // The values are only updated when you call 'getAGMT'
////    printRawAGMT( myICM.agmt );  // Uncomment this to see the raw values, taken directly from the agmt structure
////    printScaledAGMT( &myICM );    // This function takes into account the scale settings from when the measurement was made to calculate the values with units
//    delay(30);
//  }else{
//    Serial.println("[9dof] Waiting for data");
//    //delay(500);
//  }
//}
