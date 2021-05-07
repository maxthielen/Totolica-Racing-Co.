#ifndef STEER_H
#define STEER_H

#include <Arduino.h>

void turn_right(const int servo, int& servo_angle, int new_angle);
void turn_left(const int servo, int& servo_angle, int new_angle);

bool check_obstical(const int HEADER, int& uart, int& strength, int& check, int& obstacle_distance);
bool check_line(const int Line_Sensor1, const int Line_Sensor2, const int Line_Sensor3);

void search_line(const int Line_Sensor1, const int Line_Sensor2, const int Line_Sensor3, const int servo, int& servo_angle);
void follow_line(const int Line_Sensor1, const int Line_Sensor2, const int Line_Sensor3, const int servo, int& servo_angle);
void return_to_line(const int Line_Sensor1, const int Line_Sensor2, const int Line_Sensor3, const int servo, int& servo_angle);


#endif
