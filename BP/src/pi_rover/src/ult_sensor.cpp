/**
 *  @file    ult_sensor.cpp
 *  @author  Pavel Koupy (xkoupy00)
 *  @date    10.5.2018
 *  @version 1.0
 *
 *  @brief rozhrani pro zasilani kontrolnich prikazu
 *
 */

#include "ros/ros.h"
#include "std_msgs/Float32.h"
#include <wiringPi.h>
#include <sstream>
#include <iostream>
#include <string>
#include "helper.h"

using namespace std;

float getDistance();


/**
*   @brief  Publikuje data ze senzoru na urceny topic
*
*   @param  argc pocet argumentu
*   @param  argv pole argumentu
*   @return int
*/
int main(int argc, char **argv)
{
  ros::init(argc, argv, "ult_sensor_node");
  ros::NodeHandle n;
  ros::Publisher sensor_pub = n.advertise<std_msgs::Float32>("ult_sensor", G_RQ_SIZE);
  ros::Rate loop_rate(1);

  //inicializace GPIO knihovny
  if (wiringPiSetup () == -1)
  {
    return -1;
  }
  //Trigger vystupni spina mereni
  pinMode(GPIO_TRIGGER, OUTPUT);
  //Echo vstupni -ceka na vstup 
  pinMode(GPIO_ECHO, INPUT);
  std_msgs::Float32 distance;

  //vynuluji trigger
  digitalWrite(GPIO_TRIGGER, LOW);
  delay(50);

  while (ros::ok())
  {

    distance.data = getDistance();
    sensor_pub.publish(distance);

    ros::spinOnce();
    loop_rate.sleep();
  }

  return 0;
}


/**
*   @brief  Funkce pocita vzdalenost od objektu
*   pomoci ultrazvukoveho senzoru
*
*   @return float namerena vzdalenost
*/
float getDistance()
{
  long trig = 0;
  long ech = 0;
  long timeout  = 2000; 

  //zahajeni mereni
  digitalWrite(GPIO_TRIGGER, HIGH);
  delayMicroseconds(10);
  digitalWrite(GPIO_TRIGGER, LOW);

  //cas pri zaslani zvukovych signalu kvuli timeoutu
  long echoStart = millis();
  // Vyckani na vraceni signalu
  while (digitalRead(GPIO_ECHO) == LOW && (millis()-echoStart < timeout)){}
  //zmerim sirku pulzu
  trig = micros();

  while (digitalRead(GPIO_ECHO) == HIGH && (millis()-echoStart < timeout)) {}

  ech = micros();

  return (float) ((ech - trig) * 0.017150); //rozdil * konstanta pro rychlost zvuku ve vzduvhu. viz technicka_zprava
}