/**
 *  @file    gui_control.cpp
 *  @author  Pavel Koupy (xkoupy00)
 *  @date    10.5.2018
 *  @version 1.0
 *
 *  @brief rozhrani pro zasilani kontrolnich prikazu
 *
 */
#include "ros/ros.h"
#include "std_msgs/String.h"
#include <sstream>
#include <vector>
#include "helper.h"
#include <string>
using namespace std;

std::string CtrlCallback();

/**
*   @brief  Main. Funkce vycita vstup a pote jej zasila na automode_ctrl topic, ktery ridi motry
*
*   @param  argc pocet argumentu
*   @param  argv pole s argumenty
*   @return int
*/
int main(int argc, char **argv)
{
  ros::init(argc, argv, "gui_control_node");

  ros::NodeHandle n;

  ros::Publisher auto_ctrl = n.advertise<std_msgs::String>("automode_ctrl", G_RQ_SIZE);

  ros::Rate loop_rate(5); 
  std_msgs::String ctrl;

  while (ros::ok())
  {
    ctrl.data = CtrlCallback();
    auto_ctrl.publish(ctrl);
    ros::spinOnce();
    loop_rate.sleep();
  }

  return 0;
}
/**
*   @brief  Ze standartního vstupu cte retezce s prikazy a vklada je do stringu
*
*   @return std::string
*/
std::string CtrlCallback()
{
  std::string input;
  cin >> input;
  return input;

}
