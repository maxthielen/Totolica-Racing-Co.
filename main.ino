#include "steer.h"
#include "drive.h"

===============================================================================================
-----------------------------------GOBAL VARS--------------------------------------------------
===============================================================================================

int servo_angle; //steering direction of vehicle
int obstacle_distance; //obstacle range infront of vehicle
int motor_velocity; //speed of vehical

----------------------------------MotorShield--------------------------------------------------
/*
Exercise Monster Motor Shield
Uses Serial Monitor window to issue commands for controlling the DC motors 
connected to the shield
S = Stop
F = Forward
R = Reverse
C = Returns the current reading of the motors
Pxxx (P0 - P255) sets the PWM speed value
P? = Returns the current PWM value
*/
const int MOTOR_1 = 0;
const int MOTOR_2 = 1;
const int BRAKE = 0;
const int CW = 1;
const int CCW = 2;
/*
Note:  the pin definitions below are set by the shield pinout. If using
the board as a shield, these pins must remain as specified below.
If wiring the board rather than using as a shield, these can be changed. */
const int MOTOR_A1_PIN = 7; //Motor 1 control inputs
const int MOTOR_A2_PIN = 4;
//const int MOTOR_B1_PIN = 8; // Motor 2 control inputs
//const int MOTOR_B2_PIN = 9;

const int PWM_MOTOR_1 = 5;
//const int PWM_MOTOR_2 = 6;
//const int EN_PIN_1 = A0;
//const int EN_PIN_2 = A1;

int motor_Speed = 150;  //default motor speed
int motor_State = BRAKE;

char readString[4];  // String array to hold PWM value typed in on keyboard
-------------------------------------LIDAR-----------------------------------------------------
#include<SoftwareSerial.h>
// soft serial port header file
SoftwareSerial Serial1(2,3); // define the soft serial port as Serial1, pin2 as RX, and pin3 as TX
/*For Arduino board with multiple serial ports such as DUE board, comment out the above two codes, and directly use Serial1 port*/
int dist;// LiDAR actually measured distance value
int strength;// LiDAR signal strength
int check;// check numerical value storage
int uart[9];// store data measured by LiDAR
const int HEADER=0x59;// data package frame header

-------------------------------------9dof------------------------------------------------------
#include "ICM_20948.h"
#define WIRE_PORT Wire  // Your desired Wire port.(Used when "USE_SPI" is not defined)
#define AD0_VAL   1    // The value of the last bit of the I2C address.
ICM_20948_I2C myICM;  // Otherwise create an ICM_20948_I2C object

----------------------------------Flying Fish---------------------------------------------------
const int Line_Sensor1 = 6; // the number of the IR Proximity Sensor pin
const int Line_Sensor2 = 8; // the number of the IR Proximity Sensor pin
const int Line_Sensor3 = 9; // the number of the IR Proximity Sensor pin

const int Side_Sensor_L = 10; // the number of the IR Proximity Sensor pin
const int Side_Sensor_R = 11; // the number of the IR Proximity Sensor pin

===============================================================================================
-------------------------------------SET UP----------------------------------------------------
===============================================================================================

void setup(){
  Serial.begin(9600);
  Serial1.begin(115200); //Lidar & 9dof

-------------------------------------9dof------------------------------------------------------
  WIRE_PORT.begin();
  WIRE_PORT.setClock(400000);

  //myICM.enableDebugging();
  bool initialized = false;
  while( !initialized ){
    myICM.begin( WIRE_PORT, AD0_VAL );


    Serial1.print( F("Initialization of the sensor returned: ") );
    Serial1.println( myICM.statusString() );
    if( myICM.status != ICM_20948_Stat_Ok ){
      Serial1.println( "Trying again..." );
      delay(500);
    }else{
      initialized = true;
    }
  }

  ----------------------------------MotorShield--------------------------------------------------
  pinMode(MOTOR_A1_PIN, OUTPUT);
//  pinMode(MOTOR_B1_PIN, OUTPUT);
  pinMode(MOTOR_A2_PIN, OUTPUT);
//  pinMode(MOTOR_B2_PIN, OUTPUT);
  pinMode(PWM_MOTOR_1, OUTPUT);
//  pinMode(PWM_MOTOR_2, OUTPUT);
  //pinMode(EN_PIN_1, OUTPUT);      // Uncomment these 4 lines to use the Enable pins
  //pinMode(EN_PIN_2, OUTPUT);      // to enable/disable the device.  
                                    // To monitor for fault conditions instead, they 
                                    // would be defined as inputs
 // digitalWrite(EN_PIN_1, HIGH);  // Set EN pins high to enable drivers
 // digitalWrite(EN_PIN_2, HIGH); 
  
  Serial.println("Enter command:");    // Printout commands
  Serial.println("S = STOP");
  Serial.println("F = FORWARD");
  Serial.println("R = REVERSE");
  Serial.println("Pxxx = PWM SPEED (P000 - P254)");
  Serial.println("P? = RETURNS CURRENT PWM SPEED");
  ----------------------------------Flying Fish---------------------------------------------------
  pinMode(Line_Sensor1, INPUT); // initialize the IR Proximity Sensor pin as an input
  pinMode(Line_Sensor2, INPUT); // initialize the IR Proximity Sensor pin as an input
  pinMode(Line_Sensor3, INPUT); // initialize the IR Proximity Sensor pin as an input
}

===============================================================================================
--------------------------------------LOOP-----------------------------------------------------
===============================================================================================

void loop(){
----------------------------------MotorShield--------------------------------------------------
    if (Serial.available()) DoSerial();
    
-------------------------------------LIDAR-----------------------------------------------------  
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
        Serial.print("dist = ");
        Serial.print(dist);// output LiDAR tests distance value
        Serial.print('\t');
        Serial.print("strength = ");
        Serial.print(strength);// output signal strength value
        Serial.print('\n');
       }
     }
   }
 }
-------------------------------------9dof------------------------------------------------------
  if( myICM.dataReady() ){
    myICM.getAGMT();                // The values are only updated when you call 'getAGMT'
//    printRawAGMT( myICM.agmt );  // Uncomment this to see the raw values, taken directly from the agmt structure
    printScaledAGMT( &myICM );    // This function takes into account the scale settings from when the measurement was made to calculate the values with units
    delay(30);
  }else{
    Serial1.println("Waiting for data");
    delay(500);
  }


}
===============================================================================================
-----------------------------------FUNCTIONS---------------------------------------------------
===============================================================================================

----------------------------------MotorShield--------------------------------------------------
//  Subroutine to handle characters typed via Serial Monitor Window
void DoSerial()
{
  int index = 0;
  int pwm_Value = 0;
  
  char ch = Serial.read();  // Read the character we know we have
  Serial.println(ch);       // Echo character typed to show we got it

  // Use Switch/Case statement to handle the different commands
  switch (ch) {
  case 'f':   // Motor FORWARD command
  case 'F':   // This fall-through case statement accepts upper and lower case
    motor_State = CW;
    Motor_Cmd(MOTOR_1, motor_State, motor_Speed);
    Motor_Cmd(MOTOR_2, motor_State, motor_Speed);
    Serial.println("Motors Forward");
    break;

  case 'r':   // Motor REVERSE command
  case 'R':
    motor_State = CCW;
    Motor_Cmd(MOTOR_1, motor_State, motor_Speed);
    Motor_Cmd(MOTOR_2, motor_State, motor_Speed);
    Serial.println("Motors Reverse");
    break;

   case 's':   // Motor STOP command
   case 'S':
    motor_State = BRAKE;
    Motor_Cmd(MOTOR_1, motor_State, 0);
    Motor_Cmd(MOTOR_2, motor_State, 0);
    Serial.println("Motors Stop");
    break;
  
  case 'p':  // Motor SPEED command
  case 'P':
    // This command is a little trickier.  We are looking for a number from 0-255
    // to follow this command so we can set the PWM speed.  If we see a '?'
    // we will report our current speed setting, otherwise we start collecting chars
    // into the readString array.
    delay(2);  // Give time for more characters to arrive.
    for (int i; i<4; i++) readString[i] = ' ';  // Clear string array
    while (Serial.available())  // Read what we get and put into the string array
    {
      char c = Serial.read();
      readString[index] = c;
      index++;
      delay(2);
    }
    readString[3] = '\0'; // Append null to end of string array to make it a valid string
    index = 0;            // Reset our index back to the start of the string
    if (readString[index] == '?')   // ? means report our current speed setting and exit.
    {
      Serial.print("Current PWM Setting: ");
      Serial.println(motor_Speed);
      break;
    }
    pwm_Value = atoi(readString);  // Try to convert string into integer
    // We assume a 0 value is because of a non-valid input and ignore the command
    if(pwm_Value!=0) {   
      if (pwm_Value > 255) pwm_Value = 255;     // Cap WPM setting at 255
      Serial.println(pwm_Value);        // Echo what we end up with to confirm we got it
      motor_Speed = pwm_Value;
      Motor_Cmd(MOTOR_1, motor_State, motor_Speed);
      Motor_Cmd(MOTOR_2, motor_State, motor_Speed);  
    }
    break;
  
  default:
    break;
  }
}
void Motor_Cmd(int motor, int DIR, int PWM)     //Function that writes to the motors
{
  if(motor == MOTOR_1)
  {
    if(DIR == CW)    {
      digitalWrite(MOTOR_A1_PIN, LOW); 
      digitalWrite(MOTOR_B1_PIN, HIGH);
    }
    else if(DIR == CCW)    {
      digitalWrite(MOTOR_A1_PIN, HIGH);
      digitalWrite(MOTOR_B1_PIN, LOW);      
    }
    else    {
      digitalWrite(MOTOR_A1_PIN, LOW);
      digitalWrite(MOTOR_B1_PIN, LOW);            
    }
    analogWrite(PWM_MOTOR_1, PWM); 
  }
  else if(motor == MOTOR_2)
  {
    if(DIR == CW)    {
      digitalWrite(MOTOR_A2_PIN, LOW);
      digitalWrite(MOTOR_B2_PIN, HIGH);
    }
    else if(DIR == CCW)    {
      digitalWrite(MOTOR_A2_PIN, HIGH);
      digitalWrite(MOTOR_B2_PIN, LOW);      
    }
    else    {
      digitalWrite(MOTOR_A2_PIN, LOW);
      digitalWrite(MOTOR_B2_PIN, LOW);            
    } 
    analogWrite(PWM_MOTOR_2, PWM);
  }
}
---------------------------9dof Serial_print Data-----------------------------------------------
void printPaddedInt16b( int16_t val ){
  if(val > 0){
    Serial1.print(" ");
    if(val < 10000){ Serial1.print("0"); }
    if(val < 1000 ){ Serial1.print("0"); }
    if(val < 100  ){ Serial1.print("0"); }
    if(val < 10   ){ Serial1.print("0"); }
  }else{
    Serial1.print("-");
    if(abs(val) < 10000){ Serial1.print("0"); }
    if(abs(val) < 1000 ){ Serial1.print("0"); }
    if(abs(val) < 100  ){ Serial1.print("0"); }
    if(abs(val) < 10   ){ Serial1.print("0"); }
  }
  Serial1.print(abs(val));
}

void printRawAGMT( ICM_20948_AGMT_t agmt){
  Serial1.print("RAW. Acc [ ");
  printPaddedInt16b( agmt.acc.axes.x );
  Serial1.print(", ");
  printPaddedInt16b( agmt.acc.axes.y );
  Serial1.print(", ");
  printPaddedInt16b( agmt.acc.axes.z );
  Serial1.print(" ], Gyr [ ");
  printPaddedInt16b( agmt.gyr.axes.x );
  Serial1.print(", ");
  printPaddedInt16b( agmt.gyr.axes.y );
  Serial1.print(", ");
  printPaddedInt16b( agmt.gyr.axes.z );
  Serial1.print(" ], Mag [ ");
  printPaddedInt16b( agmt.mag.axes.x );
  Serial1.print(", ");
  printPaddedInt16b( agmt.mag.axes.y );
  Serial1.print(", ");
  printPaddedInt16b( agmt.mag.axes.z );
  Serial1.print(" ], Tmp [ ");
  printPaddedInt16b( agmt.tmp.val );
  Serial1.print(" ]");
  Serial1.println();
}


void printFormattedFloat(float val, uint8_t leading, uint8_t decimals){
  float aval = abs(val);
  if(val < 0){
    Serial1.print("-");
  }else{
    Serial1.print(" ");
  }
  for( uint8_t indi = 0; indi < leading; indi++ ){
    uint32_t tenpow = 0;
    if( indi < (leading-1) ){
      tenpow = 1;
    }
    for(uint8_t c = 0; c < (leading-1-indi); c++){
      tenpow *= 10;
    }
    if( aval < tenpow){
      Serial1.print("0");
    }else{
      break;
    }
  }
  if(val < 0){
    Serial1.print(-val, decimals);
  }else{
    Serial1.print(val, decimals);
  }
}

  void printScaledAGMT( ICM_20948_I2C *sensor ){

  Serial1.print("Scaled. Acc (mg) [ ");
  printFormattedFloat( sensor->accX(), 5, 2 );
  Serial1.print(", ");
  printFormattedFloat( sensor->accY(), 5, 2 );
  Serial1.print(", ");
  printFormattedFloat( sensor->accZ(), 5, 2 );
  Serial1.print(" ], Gyr (DPS) [ ");
  printFormattedFloat( sensor->gyrX(), 5, 2 );
  Serial1.print(", ");
  printFormattedFloat( sensor->gyrY(), 5, 2 );
  Serial1.print(", ");
  printFormattedFloat( sensor->gyrZ(), 5, 2 );
  Serial1.print(" ], Mag (uT) [ ");
  printFormattedFloat( sensor->magX(), 5, 2 );
  Serial1.print(", ");
  printFormattedFloat( sensor->magY(), 5, 2 );
  Serial1.print(", ");
  printFormattedFloat( sensor->magZ(), 5, 2 );
  Serial1.print(" ], Tmp (C) [ ");
  printFormattedFloat( sensor->temp(), 5, 2 );
  Serial1.print(" ]");
  Serial1.println();
}
