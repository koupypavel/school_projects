/**
 *  @file    whiskers_sensor.cpp
 *  @author  Pavel Koupy (xkoupy00)
 *  @date    10.5.2018
 *  @version 1.0
 *
 *  @brief rozhrani pro zasilani kontrolnich prikazu
 *
 */

#include "ros/ros.h"
#include "std_msgs/Bool.h"
#include <wiringPi.h>
#include <sstream>
#include <iostream>
#include <string>
#include "helper.h"

using namespace std;

bool getState();


/**
*   @brief  Publikuje data ze senzoru na urceny topic
*
*   @param  argc pocet argumentu
*   @param  argv pole argumentu
*   @return int
*/
int main(int argc, char **argv)
{
  ros::init(argc, argv, "whiskers_sensor_node");
  ros::NodeHandle n;
  ros::Publisher sensor_pub = n.advertise<std_msgs::Bool>("whiskers_sensor", G_RQ_SIZE);
  ros::Rate loop_rate(1);

  //inicializace GPIO knihovny
  if (wiringPiSetup () == -1)
  {
    return -1;
  }
  //nastaveni pinu na vstupni
  pinMode(GPIO_WHISKERS, INPUT);

  std_msgs::Bool state;
  while (ros::ok())
  {

    state.data = getState();
    sensor_pub.publish(state);

    ros::spinOnce();
    loop_rate.sleep();
  }

  return 0;
}


/**
*   @brief  Vycteni stavu GPIO pinu
*
*   @return bool nastali li kolize vraci true
*/
bool getState()
{
  return (bool)digitalRead(GPIO_WHISKERS);
}