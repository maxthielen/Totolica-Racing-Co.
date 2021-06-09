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
//
const int PWM_MOTOR_PIN = 5;
const int MOTOR_A1_PIN = 7; // used to be 7
const int MOTOR_B1_PIN = 8; //used to be 8


//const int PWM_MOTOR_PIN = 5;
//const int MOTOR_A2_PIN = 7; // used to be 7
//const int MOTOR_B2_PIN = 9; //used to be 8

int motor_Speed = 150; //default motor speed
int motor_State = BRAKE; //default motor state

const int EN_PIN_1 = A0; //TODO:: check what these pins alter on the motor shield!
const int EN_PIN_2 = A1; //TODO:: check what these pins alter on the motor shield!

//char readString[4];
//----------------------------------Flying Fish--------------------------------------------------
const int L_SENS_LEFT_PIN = A5; // Left Line Sensor //flipped
const int L_SENS_MID_PIN = A4; // Middle Line Sensor
const int L_SENS_RIGHT_PIN = A3; // Right Line Sensor//flipped

int sensors[3];

////--------------------------------------Gyroscope------------------------------------------
//
//#include "Wire.h" // This library allows you to communicate with I2C devices.
//
//const int MPU_ADDR = 0x68; // I2C address of the MPU-6050. If AD0 pin is set to HIGH, the I2C address will be 0x69.
//
//int16_t accelerometer_x, accelerometer_y, accelerometer_z; // variables for accelerometer raw data
//int16_t gyro_x, gyro_y, gyro_z; // variables for gyro raw data
//int16_t temperature; // variables for temperature data
//
//char tmp_str[7]; // temporary variable used in convert function
//
//char* convert_int16_to_str(int16_t i) { // converts int16 to string. Moreover, resulting strings will have the same length in the debug monitor.
//  sprintf(tmp_str, "%6d", i);
//  return tmp_str;
//}

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

//int iterator = 0;

bool isRamp = false;
//--------------------------------------SONIC----------------------------------------------------
const byte trigPin1 = 2;    // TRIG pin (front)
const byte echoPin1 = 2;    // ECHO pin (front)
const int max_distance = 400;
const int avoid_distance = 25.;
int current_distance = 0;

NewPing sonar_top(trigPin1, echoPin1, max_distance);
//===============================================================================================
//-------------------------------------SET UP----------------------------------------------------
//===============================================================================================
void setup() {
  Serial.begin(9600);
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
  //  //--------------------------------------Gyroscope------------------------------------------
  //  Wire.begin();
  //  Wire.beginTransmission(MPU_ADDR); // Begins a transmission to the I2C slave (GY-521 board)
  //  Wire.write(0x6B); // PWR_MGMT_1 register
  //  Wire.write(0); // set to zero (wakes up the MPU-6050)
  //  Wire.endTransmission(true);
  //------------------------------------Steering---------------------------------------------------
  pinMode(STEER_SERVO_PIN, OUTPUT);
  m_servo.attach(STEER_SERVO_PIN);



  m_servo.write(servo_angle); //straight
  //--------------------------------------SONIC----------------------------------------------------
  // configure the trigger pin to output mode
  pinMode(trigPin1, OUTPUT);
  // configure the echo pin to input mode
  pinMode(echoPin1, INPUT);
}
//================================================================================================
//--------------------------------------LOOP------------------------------------------------------
//================================================================================================
void loop() {
  current_distance = sonar_top.ping_cm();
  Serial.println(current_distance);
  if (current_distance != 0 && current_distance < avoid_distance) {
    Serial.print("We are going to avoid function");
    Serial.println();
    motor_State = BRAKE;
    Motor_Cmd(motor_State, 0);
    //    delay(500);
    //    motor_State = FORWARD;
    //    Motor_Cmd(motor_State, 35);

    //delay(500);
    //    while (current_distance > 30) {
    //      current_distance = sonar_top.ping_cm();
    //    }

    //    servo_angle = right;
    //    m_servo.write(servo_angle);
    //
    //    delay(700);
    //
    //    servo_angle = slight_left;
    //    m_servo.write(servo_angle);
    //    Serial.print("Avoiding has been finished");
    //    Serial.println();
    //    while (sensors[0] == 1) { // middle ir sensor on black
    //      check_line();
    //    }
  }
  else {
    follow_line();
  }
  //  check_gyro();
}

//===============================================================================================
//------------------------------OBSTICAL Functions-----------------------------------------------
//===============================================================================================
void avoid_obstical() {
  //  Motor_Cmd(motor_State, 40);
  //
  //  servo_angle = hard_right;
  //  m_servo.write(servo_angle);
  //
  //  delay(700);
  //
  //  servo_angle = slight_left;
  //  m_servo.write(servo_angle);
  //  Serial.print("Avoiding has been finished");
  //  Serial.println();
  //  while (sensors[0] == 1) { // middle ir sensor on black
  //    check_line();
  //  }
  motor_State = BRAKE;
  Motor_Cmd(motor_State, 0);
}
//===============================================================================================
//--------------------------------LINE Functions-------------------------------------------------
//===============================================================================================
void check_line() {
  digitalRead(L_SENS_LEFT_PIN) == HIGH ? sensors[0] = 1 : sensors[0] = 0;
  digitalRead(L_SENS_MID_PIN) == HIGH ? sensors[1] = 1 : sensors[1] = 0;
  digitalRead(L_SENS_RIGHT_PIN) == HIGH ? sensors[2] = 1 : sensors[2] = 0;

  for (int i = 0; i < 3; i++) {
    Serial.print(sensors[i]);
    Serial.print(" ");
  }
  Serial.println();
}

void follow_line() {
  check_line(); //comment out check_line() in check_sensors();
  if (sensors[0] == 1 && sensors[1] == 0 && sensors[2] == 1) { //mid ir sensor on white
    motor_State = FORWARD;
    Motor_Cmd(motor_State,  40);

    servo_angle = straight;
    m_servo.write(servo_angle);
  }
  else if (sensors[0] == 0 && sensors[1] == 1 && sensors[2] == 1) { //left ir sensor on white
    Motor_Cmd(motor_State, 40);

    servo_angle = left;
    m_servo.write(servo_angle);
  }
  else if (sensors[0] == 1 && sensors[1] == 1 && sensors[2] == 0) { //right ir sensor on white
    Motor_Cmd(motor_State, 40);

    servo_angle = right;
    m_servo.write(servo_angle);
  }
  else if (sensors[0] == 1 && sensors[1] == 1 && sensors[2] == 1) { //all ir sensors on black
    if (servo_angle == straight) {
      Motor_Cmd(motor_State, 40);
    }
    else if (servo_angle > straight) {
      servo_angle = hard_left;
      m_servo.write(servo_angle);
      while (sensors[1] == 1) {
        check_line();
        check_obstical();
      }
    }
    else if (servo_angle < straight) {
      servo_angle = hard_right;
      m_servo.write(servo_angle);
      while (sensors[1] == 1) {
        check_line();
        check_obstical();
      }
      flying_ramp();
    }
  }
  else if (sensors[0] == 0 && sensors[1] == 0 && sensors[2] == 0) { //all ir sensors on white
    motor_State = BRAKE;
    Motor_Cmd(motor_State, 0);
  }
}

//===============================================================================================
//-----------------------------COMPONENT Functions-----------------------------------------------
//===============================================================================================
void flying_ramp() {
//  if (isRamp == true) {
    motor_State = FORWARD;
    Motor_Cmd(motor_State, 150);
//  }
}


void Motor_Cmd(int DIR, int PWM)//Function that writes to the motor
{
  if (DIR == FORWARD) { //drive forward
    digitalWrite(MOTOR_A1_PIN, LOW);
    digitalWrite(MOTOR_B1_PIN, HIGH);

  }
  else if (DIR == REVERSE) { //drive reverse
    digitalWrite(MOTOR_A1_PIN, HIGH);
    digitalWrite(MOTOR_B1_PIN, LOW);
  }
  else { //stop
    digitalWrite(MOTOR_A1_PIN, LOW);
    digitalWrite(MOTOR_B1_PIN, LOW);
  }
  analogWrite(PWM_MOTOR_PIN, PWM);
}

void check_obstical() { //replace update_sensors()
  int current_distance = sonar_top.ping_cm();
  Serial.println(current_distance);
  if (current_distance != 0 && current_distance < avoid_distance) {
    Serial.print("We are going to avoid function");
    Serial.println();
    avoid_obstical();
  }
}

////===============================================================================================
////--------------------------------Gyro Functions-------------------------------------------------
////===============================================================================================
//void check_gyro() {
//  Wire.beginTransmission(MPU_ADDR);
//  Wire.write(0x3B); // starting with register 0x3B (ACCEL_XOUT_H) [MPU-6000 and MPU-6050 Register Map and Descriptions Revision 4.2, p.40]
//  Wire.endTransmission(false); // the parameter indicates that the Arduino will send a restart. As a result, the connection is kept active.
//  Wire.requestFrom(MPU_ADDR, 7 * 2, true); // request a total of 7*2=14 registers
//
//  // "Wire.read()<<8 | Wire.read();" means two registers are read and stored in the same variable
//  accelerometer_x = Wire.read() << 8 | Wire.read(); // reading registers: 0x3B (ACCEL_XOUT_H) and 0x3C (ACCEL_XOUT_L)
//  accelerometer_y = Wire.read() << 8 | Wire.read(); // reading registers: 0x3D (ACCEL_YOUT_H) and 0x3E (ACCEL_YOUT_L)
//  accelerometer_z = Wire.read() << 8 | Wire.read(); // reading registers: 0x3F (ACCEL_ZOUT_H) and 0x40 (ACCEL_ZOUT_L)
//  temperature = Wire.read() << 8 | Wire.read(); // reading registers: 0x41 (TEMP_OUT_H) and 0x42 (TEMP_OUT_L)
//  gyro_x = Wire.read() << 8 | Wire.read(); // reading registers: 0x43 (GYRO_XOUT_H) and 0x44 (GYRO_XOUT_L)
//  gyro_y = Wire.read() << 8 | Wire.read(); // reading registers: 0x45 (GYRO_YOUT_H) and 0x46 (GYRO_YOUT_L)
//  gyro_z = Wire.read() << 8 | Wire.read(); // reading registers: 0x47 (GYRO_ZOUT_H) and 0x48 (GYRO_ZOUT_L)
//
//  // print out data
//  Serial.print("aX = "); Serial.print(convert_int16_to_str(accelerometer_x));
//  Serial.print(" | aY = "); Serial.print(convert_int16_to_str(accelerometer_y));
//  Serial.print(" | aZ = "); Serial.print(convert_int16_to_str(accelerometer_z));
//  // the following equation was taken from the documentation [MPU-6000/MPU-6050 Register Map and Description, p.30]
//  Serial.print(" | tmp = "); Serial.print(temperature / 340.00 + 36.53);
//  Serial.print(" | gX = "); Serial.print(convert_int16_to_str(gyro_x));
//  Serial.print(" | gY = "); Serial.print(convert_int16_to_str(gyro_y));
//  Serial.print(" | gZ = "); Serial.print(convert_int16_to_str(gyro_z));
//  Serial.println();
//
//  // delay
//  delay(1000);
//}
