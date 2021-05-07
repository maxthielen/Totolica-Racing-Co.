#include "steer.h"

void turn_right(const int servo, int& servo_angle, int new_angle){
  if(new_angle>servo_angle){
    for(int pos=servo_angle; pos>=servo_angle; pos--){
      digitalWrite(servo, servo_angle);
      delay(15);
    }
    servo_angle = new_angle;
  }
  else turn_left(servo,servo_angle,new_angle);
}
void turn_left(const int servo, int& servo_angle, int new_angle){
  if(new_angle>servo_angle){
    for(int pos=servo_angle; pos<=servo_angle; pos++){
      digitalWrite(servo, servo_angle);
      delay(15);
    }
    servo_angle = new_angle;
  }
  else turn_right(servo,servo_angle,new_angle);
}

bool check_obstical(const int HEADER, int& uart, int& strength, int& check, int& obstacle_distance){
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
        obstacle_distance=uart[2]+uart[3]*256;// calculate distance value
        strength=uart[4]+uart[5]*256;// calculate signal strength value
        Serial.print("dist = ");
        Serial.print(obstacle_distance);// output LiDAR tests distance value
        Serial.print('\t');
        Serial.print("strength = ");
        Serial.print(strength);// output signal strength value
        Serial.print('\n');
       }
     }
   }
 }
 if(obstacle_distance<=20) return true;
 else return false;
 
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
bool check_line(const int Line_Sensor1, const int Line_Sensor2, const int Line_Sensor3){
  if(digitalRead(Line_Sensor1)==HIGH&&digitalRead(Line_Sensor2)==LOW&&digitalRead(Line_Sensor3)==HIGH){
    return true;
  }
  return false;
}

void search_line(const int Line_Sensor1, const int Line_Sensor2, const int Line_Sensor3, const int servo, int& servo_angle){
  while(!check_line(Line_Sensor1,Line_Sensor2,Line_Sensor3)){
    turn_left(servo,servo_angle,servo_angle+10);
    delay(100);
    turn_right(servo,servo_angle,servo_angle-10);
    delay(100);
  }
  follow_line(Line_Sensor1,Line_Sensor2,Line_Sensor3,servo,servo_angle);
}
void follow_line(const int Line_Sensor1, const int Line_Sensor2, const int Line_Sensor3, const int servo, int& servo_angle){
  while(check_line(Line_Sensor1,Line_Sensor2,Line_Sensor3)){
    delay(100);
  }
  return_to_line(Line_Sensor1,Line_Sensor2,Line_Sensor3,servo,servo_angle);
}
void return_to_line(const int Line_Sensor1, const int Line_Sensor2, const int Line_Sensor3, const int servo, int& servo_angle){
  if(direction==0) direction=180;
  else direction = 0;

  if(check_line(Line_Sensor1,Line_Sensor2,Line_Sensor3)) follow_line(Line_Sensor1,Line_Sensor2,Line_Sensor3,servo,servo_angle);
  else return_to_line(Line_Sensor1,Line_Sensor2,Line_Sensor3,servo,servo_angle);
}
