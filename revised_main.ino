//===============================================================================================
//-----------------------------------GOBAL VARS--------------------------------------------------
//===============================================================================================
#include <Servo.h> 
//-------------------------------------9dof------------------------------------------------------
#include "ICM_20948.h"
#define WIRE_PORT Wire  // Your desired Wire port.(Used when "USE_SPI" is not defined)
#define AD0_VAL   1    // The value of the last bit of the I2C address.
ICM_20948_I2C myICM;  // Otherwise create an ICM_20948_I2C object
//----------------------------------MotorShield--------------------------------------------------
const int BRAKE = 0;
const int FORWARD = 1;
const int REVERSE = 2;

const int PWM_MOTOR_PIN = 5;
const int MOTOR_A1_PIN = 7;
const int MOTOR_B1_PIN = 8;

int motor_Speed = 150; //default motor speed
int motor_State = BRAKE; //default motor state

const int EN_PIN_1 = A0; //TODO:: check what these pins alter on the motor shield!
const int EN_PIN_2 = A1; //TODO:: check what these pins alter on the motor shield!

char readString[4];
//-------------------------------------LIDAR-----------------------------------------------------
#include<SoftwareSerial.h>
// soft serial port header file
SoftwareSerial Serial1(2,3); // define the soft serial port as Serial1, pin2 as RX, and pin3 as TX
/*For Arduino board with multiple serial ports such as DUE board, comment out the above two codes, and directly use Serial1 port*/
int dist;// LiDAR actually measured distance value
int strength;// LiDAR signal strength
int check;// check numerical value storage
int uart[9];// store data measured by LiDAR
const int HEADER=0x59;// data package frame header
//----------------------------------Flying Fish--------------------------------------------------
const int L_SENS_LEFT_PIN = 4; // Left Line Sensor
const int L_SENS_MID_PIN = 6; // Middle Line Sensor
const int L_SENS_RIGHT_PIN = 9; // Right Line Sensor

const int S_SENS_LEFT_PIN = 10; // Left Side Sensor
const int S_SENS_RIGHT_PIN = 11; // Right Side Sensor
//------------------------------------Steering---------------------------------------------------
const int STEER_SERVO_PIN = A2;
Servo Servo1;
const int turn_angle = 10;
const int turn_delay = 50;
int servo_angle = 108; //current steering direction of vehicle



//===============================================================================================
//-------------------------------------SET UP----------------------------------------------------
//===============================================================================================
void setup() {
  Serial.begin(74880);
  Serial1.begin(115200); //Lidar & 9dof **(if 9dof is problematic try switching it to 9600)**

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
//----------------------------------MotorShield--------------------------------------------------
  pinMode(MOTOR_A1_PIN, OUTPUT);
  pinMode(MOTOR_B1_PIN, OUTPUT);
  pinMode(PWM_MOTOR_PIN, OUTPUT);

  //?????????????????????????????????????????????????????????????????????????????????
  pinMode(EN_PIN_1, OUTPUT);      // Uncomment these 4 lines to use the Enable pins
  pinMode(EN_PIN_2, OUTPUT);      // to enable/disable the device.  
                                  // To monitor for fault conditions instead, they 
                                  // would be defined as inputs
  digitalWrite(EN_PIN_1, HIGH);   // Set EN pins high to enable drivers
  digitalWrite(EN_PIN_2, HIGH);
  //?????????????????????????????????????????????????????????????????????????????????
  
//----------------------------------Flying Fish--------------------------------------------------
  pinMode(L_SENS_LEFT_PIN, INPUT); 
  pinMode(L_SENS_MID_PIN, INPUT); 
  pinMode(L_SENS_RIGHT_PIN, INPUT); 

  pinMode(S_SENS_LEFT_PIN, INPUT); 
  pinMode(S_SENS_RIGHT_PIN, INPUT); 
//------------------------------------Steering---------------------------------------------------
  pinMode(STEER_SERVO_PIN, OUTPUT); 
  Servo1.attach(STEER_SERVO_PIN); 
}
//================================================================================================
//--------------------------------------LOOP------------------------------------------------------
//================================================================================================
void loop() {
//Servo1.write(50); //most left
//delay(1000);
//
//Servo1.write(150); //most right
//delay(1000);
//
//Servo1.write(servo_angle); //straight
//delay(1000);

//int left = digitalRead(L_SENS_LEFT_PIN);
//Serial.print("left: ");
//Serial.println(left);
//int mid = digitalRead(L_SENS_MID_PIN);
//Serial.print("mid: ");
//Serial.println(mid);
//int right = digitalRead(L_SENS_RIGHT_PIN);
//Serial.print("right: ");
//Serial.println(right);

int dista = check_LIDAR();
Serial.print("dist: ");
Serial.println(dista);
delay(10);

//  motor_State = FORWARD;
//  Motor_Cmd(motor_State, 45);
//  delay(1000);
//  Motor_Cmd(motor_State, 35);
//  delay(6000);
//  Motor_Cmd(motor_State, 25);
//  delay(5000);
//  
//  Motor_Cmd(motor_State, 45);
//  delay(1000);
//  Motor_Cmd(motor_State, 50);
//  delay(1000);
//  Motor_Cmd(motor_State, 60);
//  delay(1000);
//  Motor_Cmd(motor_State, 26);
//
//  Serial.println("Stop/Waiting");
//  motor_State = BRAKE;
//  Motor_Cmd(motor_State, 0);
//  delay(1000);
//  
//  
//  if(L_SENS_LEFT_PIN)==LOW&&digitalRead(L_SENS_MID_PIN)==HIGH&&digitalRead(L_SENS_RIGHT_PIN)==LOW && check_LIDAR()>50){ //normal driving speed
//    motor_State = FORWARD;
//    Motor_Cmd(motor_State, 50);
//  }
//  else if(L_SENS_LEFT_PIN)==HIGH&&digitalRead(L_SENS_MID_PIN)==HIGH&&digitalRead(L_SENS_RIGHT_PIN)==HIGH && check_LIDAR()>50){ //reached end of race
//    motor_State = BRAKE;
//    Motor_Cmd(motor_State, 0);
//    Serial.println("CAR Stop");
//  }
//  else if(L_SENS_LEFT_PIN)==HIGH&&digitalRead(L_SENS_MID_PIN)==LOW&&digitalRead(L_SENS_RIGHT_PIN)==LOW && check_LIDAR()>50){ //too right
//    Motor_Cmd(motor_State, 26); //slow down
//    turn_left(turn_angle);
//    Serial.println("Turn Left");
//  }
//  else if(L_SENS_LEFT_PIN)==LOW&&digitalRead(L_SENS_MID_PIN)==LOW&&digitalRead(L_SENS_RIGHT_PIN)==HIGH && check_LIDAR()>50){ //too left
//    Motor_Cmd(motor_State, 26); //slow down
//    turn_right(turn_angle);
//    Serial.println("Turn Right");
//  }
//  else if(check_LIDAR()<=50){ //avoid obstical
//    Serial.println("Avoiding Obstical...");
//    Motor_Cmd(motor_State, 26); //slow down
//      
//    turn_right(turn_angle);
//    delay(turn_delay);
//    turn_left(turn_angle);
//    
//    check_passed();
//
//    turn_left(turn_angle);
//    while(digitalRead(L_SENS_LEFT_PIN)!=HIGH);
//    turn_right(turn_angle);
//    Serial.println("Avoided Obstical");
//  }
//  else(){ //no statement was called
//    motor_State = BRAKE;
//    Motor_Cmd(motor_State, 0);
//    Serial.println("Fall-through Error");
//  }
}


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

int check_LIDAR()
{
  if (Serial1.available())//check whether the serial port has data input
  {
    if(Serial1.read()==HEADER)// determine data package frame header 0x59
    {
      uart[0]=HEADER;
      if(Serial1.read()==HEADER)//determine data package frame header 0x59
      {
        uart[1]=HEADER;
        for(int i=2;i<9;i++)// store data to array
        {
          uart[i]=Serial1.read();
        }
        check=uart[0]+uart[1]+uart[2]+uart[3]+uart[4]+uart[5]+uart[6]+uart[7];
        if(uart[8]==(check&0xff))// check the received data as per protocols
        {
          dist=uart[2]+uart[3]*256;// calculate distance value
          strength=uart[4]+uart[5]*256;// calculate signal strength value
          
//          Serial.print("[Lidar] dist = ");
//          Serial.print(dist);// output LiDAR tests distance value
//          Serial.print('\t');
//          Serial.print("strength = ");
//          Serial.print(strength);// output signal strength value
//          Serial.print('\n');

          return dist;
        }
      }
    }
  }
}
void check_9DOF()
{
  if( myICM.dataReady() ){
    myICM.getAGMT();                // The values are only updated when you call 'getAGMT'
//    printRawAGMT( myICM.agmt );  // Uncomment this to see the raw values, taken directly from the agmt structure
//    printScaledAGMT( &myICM );    // This function takes into account the scale settings from when the measurement was made to calculate the values with units
    delay(30);
  }else{
    Serial.println("[9dof] Waiting for data");
    delay(500);
  }
}

//===============================================================================================
//-------------------------------STEER Functions-------------------------------------------------
//===============================================================================================

void turn_right(int degree){
  int new_angle = servo_angle+degree;
  
//  for(int pos=servo_angle; pos<=new_angle; pos++){
//    digitalWrite(STEER_SERVO_PIN, pos);
//    delay(15);
//  }

  Servo1.write(new_angle);
  
  servo_angle = new_angle;
}
void turn_left(int degree){
  int new_angle = servo_angle-degree;

//  for(int pos=servo_angle; pos>=new_angle; pos--){
//    digitalWrite(STEER_SERVO_PIN, pos);
//    delay(15);
//  }
  Servo1.write(new_angle);
  servo_angle = new_angle;
}
void turn_straight(){
  servo_angle = 108;
  Servo1.write(servo_angle);
}
//===============================================================================================
//------------------------------OBSTICAL Functions-----------------------------------------------
//===============================================================================================
bool check_passed(){ //only works for right-avoition
  if(digitalRead(S_SENS_LEFT_PIN)==HIGH) return true;
  else return false;
}
bool check_obstical(){
 //if(check_LIDAR()<=20) return true;
 //else return false;
 
/*
  if(obstacle_distance<=20){
    drive.slow();
    if(direction==0) direction=180;
    else direction = 0;

    while(distance<=20) distance = digitalRead(Lidar);

    delay(100);

    return_to_line();
  }
  */
}
//===============================================================================================
//--------------------------------LINE Functions-------------------------------------------------
//===============================================================================================
bool check_line(){
  if(digitalRead(L_SENS_LEFT_PIN)==LOW&&digitalRead(L_SENS_MID_PIN)==HIGH&&digitalRead(L_SENS_RIGHT_PIN)==LOW){
    return true;
  }
  return false;
}

void search_line(){
  bool searching_right = true;
  int instances = 0;
  
  while(!check_line()){
    if(!searching_right){ 
      if(instances==0) turn_left(turn_angle);
      delay(100);
      instances++;
      if(instances>=10){
        searching_right=true;
        instances=0;
      }
    }
    else{
      if(instances==0) turn_right(turn_angle);
      delay(100);
      instances++;
      if(instances>=10){
        searching_right=false;
        instances=0;
      }
    }
  }
  
  follow_line();
}

void follow_line(){
  while(check_line()&&!check_obstical());
  //decelerate();
  
  if(digitalRead(L_SENS_LEFT_PIN)==HIGH){ //correct path left
    turn_left(turn_angle);
    
    if(digitalRead(L_SENS_LEFT_PIN)==LOW){ //return to driving straight
      turn_right(turn_angle);
      follow_line();
    }
    else{
      //code for a turn
    }
    follow_line();
    }
  else if(digitalRead(L_SENS_RIGHT_PIN)==HIGH){ //correct path right
    turn_right(turn_angle);

    if(digitalRead(L_SENS_RIGHT_PIN)==LOW){ //return to driving straight
      turn_left(turn_angle);
      follow_line();
    }
    else{
      //code for a turn
    }
    follow_line();
  }
  else if((L_SENS_LEFT_PIN)==HIGH&&digitalRead(L_SENS_MID_PIN)==HIGH&&digitalRead(L_SENS_RIGHT_PIN)==HIGH){
    motor_State = BRAKE;
    Motor_Cmd(motor_State, 0);
    Serial.println("Motors Stop");
  }
  else{ //avoid obstical
    turn_right(turn_angle);
    delay(turn_delay);
    turn_left(turn_angle);
    
    check_passed();

    turn_left(turn_angle);
    while(digitalRead(L_SENS_LEFT_PIN)!=HIGH);
    turn_right(turn_angle);
    //accelerate();
  }
}

//void return_to_line(){ //only works for right avoition
//  turn_left(10);
//  while(digitalRead(L_SENS_LEFT_PIN)!=HIGH);
//  turn_right(10);
//}



//===============================================================================================
//--------------------------------------TRASH----------------------------------------------------
//===============================================================================================

//---------------------------9dof Serial_print Data----------------------------------------------
//void printPaddedInt16b( int16_t val ){
//  if(val > 0){
//    Serial1.print(" ");
//    if(val < 10000){ Serial1.print("0"); }
//    if(val < 1000 ){ Serial1.print("0"); }
//    if(val < 100  ){ Serial1.print("0"); }
//    if(val < 10   ){ Serial1.print("0"); }
//  }else{
//    Serial1.print("-");
//    if(abs(val) < 10000){ Serial1.print("0"); }
//    if(abs(val) < 1000 ){ Serial1.print("0"); }
//    if(abs(val) < 100  ){ Serial1.print("0"); }
//    if(abs(val) < 10   ){ Serial1.print("0"); }
//  }
//  Serial1.print(abs(val));
//}
//
//void printRawAGMT( ICM_20948_AGMT_t agmt){
//  Serial1.print("[9dof] RAW. Acc [ ");
//  printPaddedInt16b( agmt.acc.axes.x );
//  Serial1.print(", ");
//  printPaddedInt16b( agmt.acc.axes.y );
//  Serial1.print(", ");
//  printPaddedInt16b( agmt.acc.axes.z );
//  Serial1.print(" ], Gyr [ ");
//  printPaddedInt16b( agmt.gyr.axes.x );
//  Serial1.print(", ");
//  printPaddedInt16b( agmt.gyr.axes.y );
//  Serial1.print(", ");
//  printPaddedInt16b( agmt.gyr.axes.z );
//  Serial1.print(" ], Mag [ ");
//  printPaddedInt16b( agmt.mag.axes.x );
//  Serial1.print(", ");
//  printPaddedInt16b( agmt.mag.axes.y );
//  Serial1.print(", ");
//  printPaddedInt16b( agmt.mag.axes.z );
//  Serial1.print(" ], Tmp [ ");
//  printPaddedInt16b( agmt.tmp.val );
//  Serial1.print(" ]");
//  Serial1.println();
//}
//
//
//void printFormattedFloat(float val, uint8_t leading, uint8_t decimals){
//  float aval = abs(val);
//  if(val < 0){
//    Serial1.print("-");
//  }else{
//    Serial1.print(" ");
//  }
//  for( uint8_t indi = 0; indi < leading; indi++ ){
//    uint32_t tenpow = 0;
//    if( indi < (leading-1) ){
//      tenpow = 1;
//    }
//    for(uint8_t c = 0; c < (leading-1-indi); c++){
//      tenpow *= 10;
//    }
//    if( aval < tenpow){
//      Serial1.print("0");
//    }else{
//      break;
//    }
//  }
//  if(val < 0){
//    Serial1.print(-val, decimals);
//  }else{
//    Serial1.print(val, decimals);
//  }
//}

//  void printScaledAGMT( ICM_20948_I2C *sensor ){
//
//  Serial1.print("[9dof] Scaled. Acc (mg) [ ");
//  printFormattedFloat( sensor->accX(), 5, 2 );
//  Serial1.print(", ");
//  printFormattedFloat( sensor->accY(), 5, 2 );
//  Serial1.print(", ");
//  printFormattedFloat( sensor->accZ(), 5, 2 );
//  Serial1.print(" ], Gyr (DPS) [ ");
//  printFormattedFloat( sensor->gyrX(), 5, 2 );
//  Serial1.print(", ");
//  printFormattedFloat( sensor->gyrY(), 5, 2 );
//  Serial1.print(", ");
//  printFormattedFloat( sensor->gyrZ(), 5, 2 );
//  Serial1.print(" ], Mag (uT) [ ");
//  printFormattedFloat( sensor->magX(), 5, 2 );
//  Serial1.print(", ");
//  printFormattedFloat( sensor->magY(), 5, 2 );
//  Serial1.print(", ");
//  printFormattedFloat( sensor->magZ(), 5, 2 );
//  Serial1.print(" ], Tmp (C) [ ");
//  printFormattedFloat( sensor->temp(), 5, 2 );
//  Serial1.print(" ]");
//  Serial1.println();
//}

//----------------------------------MotorShield--------------------------------------------------
//void DoSerial()
//{
//  int index = 0;
//  int pwm_Value = 0;
//  
//  char ch = Serial.read();  // Read the character we know we have
//  Serial.println(ch);       // Echo character typed to show we got it
//
//  // Use Switch/Case statement to handle the different commands
//  switch (ch) {
//  case 'f':   // Motor FORWARD command
//  case 'F':   // This fall-through case statement accepts upper and lower case
//    motor_State = FORWARD;
//    Motor_Cmd(motor_State, motor_Speed);
//    Serial.println("Motors Forward");
//    break;
//
//  case 'r':   // Motor REVERSE command
//  case 'R':
//    motor_State = REVERSE;
//    Motor_Cmd(motor_State, motor_Speed);
//    Serial.println("Motors Reverse");
//    break;
//
//   case 's':   // Motor STOP command
//   case 'S':
//    motor_State = BRAKE;
//    Motor_Cmd(motor_State, 0);
//    Serial.println("Motors Stop");
//    break;
//  
//  case 'p':  // Motor SPEED command
//  case 'P':
//    // This command is a little trickier.  We are looking for a number from 0-255
//    // to follow this command so we can set the PWM speed.  If we see a '?'
//    // we will report our current speed setting, otherwise we start collecting chars
//    // into the readString array.
//    delay(2);  // Give time for more characters to arrive.
//    for (int i; i<4; i++) readString[i] = ' ';  // Clear string array
//    while (Serial.available())  // Read what we get and put into the string array
//    {
//      char c = Serial.read();
//      readString[index] = c;
//      index++;
//      delay(2);
//    }
//    readString[3] = '\0'; // Append null to end of string array to make it a valid string
//    index = 0;            // Reset our index back to the start of the string
//    if (readString[index] == '?')   // ? means report our current speed setting and exit.
//    {
//      Serial.print("Current PWM Setting: ");
//      Serial.println(motor_Speed);
//      break;
//    }
//    pwm_Value = atoi(readString);  // Try to convert string into integer
//    // We assume a 0 value is because of a non-valid input and ignore the command
//    if(pwm_Value!=0) {   
//      if (pwm_Value > 255) pwm_Value = 255;     // Cap WPM setting at 255
//      Serial.println(pwm_Value);        // Echo what we end up with to confirm we got it
//      motor_Speed = pwm_Value;
//      Motor_Cmd(motor_State, motor_Speed);
//    }
//    break;
//  
//  default:
//    break;
//  }
//}
