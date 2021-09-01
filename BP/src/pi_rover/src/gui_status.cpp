/**
 *  @file    gui_status.cpp
 *  @author  Pavel Koupy (xkoupy00)
 *  @date    10.5.2018
 *  @version 1.0
 *
 *  @brief rozhrani pro vycitani promennych z ROS frameworku
 *
 */

#include "ros/ros.h"
#include "std_msgs/Float32.h"
#include "std_msgs/Int16.h"
#include "std_msgs/Bool.h"
#include "std_msgs/String.h"
#include "std_msgs/UInt16.h"
#include <sstream>
#include <vector>
#include "helper.h"
#include <iostream>

using namespace std;

void ultSensorCallback(const std_msgs::Float32::ConstPtr& msg);
void whiskersSensorCallback(const std_msgs::Bool::ConstPtr& msg);
void automodeCallback(const std_msgs::Int16::ConstPtr& msg);
void SLAMCallback(const std_msgs::Int16::ConstPtr& msg);

float last_distance = 0;
bool last_whiskers = false;
int last_SLAMState = -1;
int last_automode = 0;

/**
*   @brief  Funkce odposlouchava topic a data posila na standartni vystup
*
*   @param  argc pocet argumentu
*   @param  argv pole argumentu
*   @return int
*/
int main(int argc, char **argv)
{
  ros::init(argc, argv, "gui_status_node");

  ros::NodeHandle n;

  //definice subscribers, ze kterych se ctou informace
  ros::Subscriber ult_sensor = n.subscribe("ult_sensor", G_RQ_SIZE, ultSensorCallback);
  ros::Subscriber whiskers_sensor = n.subscribe("whiskers_sensor", G_RQ_SIZE, whiskersSensorCallback);
  ros::Subscriber automode_status = n.subscribe("automode_status", G_RQ_SIZE, automodeCallback);
  ros::Subscriber slam_info = n.subscribe("orb_slam", G_RQ_SIZE, SLAMCallback);


  stringstream ss;
  //frekvence vykonavani smycky
  ros::Rate loop_rate(2);

  //konkretni prikaz vypada : "distance:xx.xx,whiskers:0/1,autonomode:x,slam:x"
  while (ros::ok())
  {
    ss << "distance:" << last_distance << ",";
    ss << "whiskers:" << (last_whiskers ? 1 : 0) << ",";
    ss << "autonomode:" << last_automode << ",";
    ss << "slam:" << last_SLAMState;
    cout << ss.str() << std::endl;
    ss.str("");
    ros::spinOnce();
    loop_rate.sleep();
  }

  return 0;
}

/**
*   @brief  Funkce volana v pripade pritomnosti dat v topicu ult_sensor
*
*   @param  msg data z topicu
*   @return void
*/
void ultSensorCallback(const std_msgs::Float32::ConstPtr& msg)
{
  last_distance = msg->data;
}

/**
*   @brief  Funkce volana v pripade pritomnosti dat v topicu whiskers_sensor (dotykovy senzor)
*
*   @param  msg data z topicu
*   @return void
*/
void whiskersSensorCallback(const std_msgs::Bool::ConstPtr& msg)
{
  last_whiskers = msg->data;
}

/**
*   @brief  Funkce volana v pripade pritomnosti dat v topicu orb_slam (stav mapovani)
*
*   @param  msg data z topicu
*   @return void
*/
void SLAMCallback(const std_msgs::Int16::ConstPtr& msg)
{
  last_SLAMState = msg->data;
}

/**
*   @brief  Funkce volana v pripade pritomnosti dat v topicu automode_status (stav automatickeho rizeni)
*
*   @param  msg data z topicu
*   @return void
*/
void automodeCallback(const std_msgs::Int16::ConstPtr& msg)
{
  last_automode = msg->data;
}