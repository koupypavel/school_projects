/**
 *  @file    auto_mode.cpp
 *  @author  Pavel Koupy (xkoupy00)
 *  @date    10.5.2018
 *  @version 1.0
 *
 *  @brief Rizeni modelu automaticke/manualni
 *
 */

#include "ros/ros.h"
#include "std_msgs/String.h"
#include "std_msgs/Float32.h"
#include "std_msgs/Int16.h"
#include "std_msgs/UInt16.h"
#include "std_msgs/Bool.h"
#include <sstream>
#include <vector>
#include <queue>
#include <boost/circular_buffer.hpp>
#include <map>
#include <string.h>
#include <string>
#include "helper.h"
#include <iostream>
#include <random>

using namespace std;


//prototypy
void ultSensorCallback(const std_msgs::Float32::ConstPtr& msg);
void whiskersSensorCallback(const std_msgs::Bool::ConstPtr& msg);
void debug_printHCS(int state);
void CtrlCallback(const std_msgs::String::ConstPtr & msg);
std::vector<std::string> split(const std::string& s, char delimiter);

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
unsigned int level_WandererWalk(unsigned int direction);

int ww_step_counter = 0;
WandererWalkState ww_state = WWS_INIT;
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

unsigned int level_RandomWalk(unsigned int direction);

int rw_step_counter = 0;
int rw_rnd_counter = 3;
RandomWalkState rw_state = RWS_INIT;
std::vector<float> rw_dists;
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
unsigned int level_collisionAvoid(unsigned int direction);

int hc_step_counter = 0;
HardCollisionState hc_state = HCS_INIT;
std::vector<float> dists;
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

unsigned int manual_state = 0;

float last_distance = 0;
bool last_touch = false;
int last_SLAMState = -1;

NodeLevel level = NodeLevel::manual;
int rw_fw_dir = 0;

bool setbdone = false;
int direction = 0;
boost::circular_buffer<unsigned int> moves(50);
std_msgs::String motor_speeds;


/**
*   @brief  Main
*
*   @param  argc pocet argumentu
*   @param  argv pole argumentu
*   @return int
*/
int main(int argc, char **argv)
{
  ros::init(argc, argv, "autonomode_node");

  ros::NodeHandle n;

  //Publisher a Subscriber inicializace
  ros::Publisher ctrl_motor = n.advertise<std_msgs::UInt16>("motors_ctrl", G_RQ_SIZE);
  ros::Publisher ctrl_motor_speed = n.advertise<std_msgs::String>("motors_speed", G_RQ_SIZE);
  ros::Publisher auto_info = n.advertise<std_msgs::Int16>("automode_status", G_RQ_SIZE);

  ros::Subscriber ult_sensor = n.subscribe("ult_sensor", G_RQ_SIZE, ultSensorCallback);
  ros::Subscriber whiskers_sensor = n.subscribe("whiskers_sensor", G_RQ_SIZE, whiskersSensorCallback);
  //ros::Subscriber slam_info = n.subscribe("orb_slam", G_RQ_SIZE, SLAMStateCallback);
  ros::Subscriber auto_ctrl = n.subscribe("automode_ctrl", G_RQ_SIZE, CtrlCallback);

  //objekty zprav , ktere se budou zasilat
  std_msgs::UInt16 step_move;
  std_msgs::Int16 autonomode_info;

  //frekvence s kterou je smycka volana
  ros::Rate loop_rate(AUTONOMODE_LOOP_RATE); //4hz
  std::string check_tmp = "";

  while (ros::ok())
  {
    ////////////////////////////////////////////////////////
    //volani jednotlivych automatu
    direction = MOVE_FOWARD;
    if (level != manual)
    {
      debug_printHCS(hc_state);
      direction = level_collisionAvoid(direction);
      level = NodeLevel::colisionAvoiding;
      //pokud prvni stupen resi kolizy vyssi jsou potlaceny
      if (hc_state == HCS_COL_CHECK) //
      {
        direction = level_WandererWalk(direction);
        level = NodeLevel::wandererWalk;
        //pokud se vozitko vyhyba na zaklade udaju z ult. sensoru je fukce
        //nahodneho prochazeni potlacena
        if (ww_state == WWS_CHECK_DISTANCE)
        {
          level = NodeLevel::randomWalk;
          direction = level_RandomWalk(direction);
        }
      }

      else
      {
        //vynulovani pocitadla pro randomWalk level
        rw_fw_dir = 0;
      }
    }
    else
    {
      //Manualni rizeni
      direction = manual_state;
    }
    ////////////////////////////////////////////////////////
    //publikace ridiciho kroku
    step_move.data = direction;
    ctrl_motor.publish(step_move);

    //Publikace informace o aktualnim stavu uzlu
    autonomode_info.data = level;
    auto_info.publish(autonomode_info);

    //rychlost motoru
    check_tmp = motor_speeds.data;

    if (check_tmp.size() > 0)
    {
      ctrl_motor_speed.publish(motor_speeds);
      motor_speeds.data = "";
    }

    ros::spinOnce();
    loop_rate.sleep();
  }

  return 0;
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

/**
*   @brief  Deboguvaci vypis pro level_collisionAvoid
*
*   @param  state stav automatu
*   @return void
*/
void debug_printHCS(int state)
{
  std::string state_str = "";
  switch (state)
  {
  case HCS_INIT:
    state_str = "HCS_INIT";
    break;
  case HCS_BACK:
    state_str = "BACK";
    break;
  case HCS_COL_CHECK:
    state_str = "";
    break;
  case HCS_COL_SOLVE:
    state_str = "HCS_COL_SOLVE";
    break;
  case HCS_COL_CHOOSER:
    state_str = "HCS_COL_CHOOSER";
    break;
  case HCS_COL_REVIVE:
    state_str = "HCS_COL_REVIVE";
    break;
  default:
    break;
  }
  std::cout << "HCS state: " << state_str << std::endl;
}

/**
*   @brief  Automat pro reseni kolize zaznamenane
*   dotykovym senzorem
*   (1.level)
*
*   @param  direction smer jizdy
*   @return unsigned int vraci stavajici smer nebo zmeneny smer jizdy
*/
unsigned int level_collisionAvoid(unsigned int direction)
{
  int bg_distance = 0;
  int bg_dir = -1;
  switch (hc_state)
  {
  case HCS_INIT:
    hc_state = HCS_COL_CHECK;
    break;

  case HCS_COL_CHECK:
    if (last_touch == true)
    {
      hc_state = HCS_BACK;
      hc_step_counter = MOVE_CTRL_BACK;
    }
    break;

  case HCS_BACK:
    if (hc_step_counter > 0)
    {
      hc_step_counter--;
      direction = MOVE_BACKWARD;
    }
    else
    {
      hc_step_counter = MOVE_CTRL_CLK;
      hc_state = HCS_COL_SOLVE;
    }
    break;

  case HCS_COL_SOLVE:
    //save mesurements from 3 points
    if (hc_step_counter == MOVE_CTRL_QRT || hc_step_counter == MOVE_CTRL_2QRT)
    {
      dists.push_back(last_distance);
    }
    //rotate 3/4 of circle
    if (hc_step_counter > 0)
    {
      hc_step_counter--;
      direction = ROTATE_RIGHT;
    }
    else
    {
      dists.push_back(last_distance);
      hc_state = HCS_COL_CHOOSER;
    }

    break;

  case HCS_COL_CHOOSER:
    for (int i = 0; i < dists.size(); i++)
    {
      if (dists.at(i) > bg_distance)
      {
        bg_distance = dists.at(i);
        bg_dir = i;
      }
    }
    dists.clear();
    if (bg_dir == 2)
    {
      hc_state = HCS_COL_CHECK;
      direction = MOVE_FOWARD;
    }
    else if (bg_dir == 1)
    {
      hc_step_counter = MOVE_CTRL_QRT;
      hc_state = HCS_COL_REVIVE;
    }
    else if (bg_dir == 0)
    {
      hc_step_counter = MOVE_CTRL_2QRT;
      hc_state = HCS_COL_REVIVE;
    }
    break;

  case HCS_COL_REVIVE:
    if (hc_step_counter > 0)
    {
      hc_step_counter--;
      direction = ROTATE_LEFT;
    }
    else
      hc_state = HCS_COL_CHECK;

    break;

  default:
    break;
  }
  return direction;
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

/**
*   @brief  Deboguvaci vypis pro level_WandererWalk
*
*   @param  state stav automatu
*   @return void
*/
void debug_printWWS(int state)
{
  std::string state_str = "";
  switch (state)
  {
  case WWS_INIT:
    state_str = "WWS_INIT";
    break;
  case WWS_CHECK_DISTANCE:
    state_str = "WWS_CHECK_DISTANCE";
    break;
  case WWS_OBJECT_ALIGNPP:
    state_str = "WWS_OBJECT_ALIGNPP";
    break;
  default:
    break;
  }
  std::cout << "WWS state: " << state_str << std::endl;
}


/**
*   @brief  Automat pro prochazeni mistnosti pomoci
*   ultrazvukoveho senzoru
*   (2.level)
*
*   @param  direction smer jizdy
*   @return unsigned int vraci stavajici smer nebo zmeneny smer jizdy
*/
unsigned int level_WandererWalk(unsigned int direction)
{
  int lst_distance = static_cast<int>(last_distance);
  //std::cout << "distance :" << lst_distance << std::endl;
  //debug_printWWS(ww_state);
  switch (ww_state)
  {
  case WWS_INIT:
    ww_state = WWS_CHECK_DISTANCE;
    break;
  case WWS_CHECK_DISTANCE:
    if (lst_distance < NAV_BREAK_DISTANCE)
    {
      ww_state = WWS_OBJECT_ALIGNPP;
    }
    break;
  case WWS_OBJECT_ALIGNPP:

    if (lst_distance < NAV_GO_DISTANCE)
    {
      direction = ROTATE_LEFT;
    }
    else
    {
      ww_state = WWS_CHECK_DISTANCE;
    }
    break;
  default:
    break;
  }
  return direction;
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

/**
*   @brief  Deboguvaci vypis pro level_RandomWalk
*
*   @param  state stav automatu
*   @return void
*/
void debug_printRWS(int state)
{

  std::string state_str = "";
  switch (state)
  {
  case RWS_INIT:
    state_str = "RWS_INIT";
    break;
  case RWS_CHECK_STEPS:
    state_str = "RWS_CHECK_STEPS";
    break;
  case RWS_ROTATE:
    state_str = "RWS_ROTATE";
    break;
  case RWS_CHOOSER:
    state_str = "RWS_CHOOSER";
    break;
  case RWS_REVIVE:
    state_str = "RWS_REVIVE";
    break;
  default:
    break;
  }
  std::cout << "RWS state: " << state_str << std::endl;
}

/**
*   @brief  Automat pro nahodne prochazeni
*   (3.level)
*
*   @param  direction smer jizdy
*   @return unsigned int vraci stavajici smer nebo zmeneny smer jizdy
*/
unsigned int level_RandomWalk(unsigned int direction)
{
  std::random_device rd; // obtain a random number from hardware
  std::mt19937 eng(rd()); // seed the generator
  std::uniform_int_distribution<> distr(rw_step_counter, MOVE_CTRL_CLK); // define the range
  int lst_distance = last_distance;
  int bg_distance = 0;
  int bg_dir = -1;
  debug_printRWS(rw_state);
  switch (rw_state)
  {
  case RWS_INIT:
    rw_state = RWS_CHECK_STEPS;
    break;
  case RWS_CHECK_STEPS:
    if (rw_fw_dir > RANDOM_STEP_MAX)
    {
      rw_fw_dir = 0;
      rw_state = RWS_ROTATE;
      rw_step_counter = MOVE_CTRL_CLK;
    }
    else
    {
      rw_fw_dir++;
    }
    break;
  case RWS_ROTATE:
    //save mesurements from 3 points
    if (rw_step_counter == distr(eng) && rw_rnd_counter > 0)
    {
      rw_rnd_counter--;
      dists.push_back(lst_distance);
    }
    //rotate 3/4 of circle
    if (rw_step_counter > 0)
    {
      rw_step_counter--;
      direction = ROTATE_LEFT;
    }
    else
    {
      dists.push_back(lst_distance);
      rw_state = RWS_CHOOSER;
    }

    break;
  case RWS_CHOOSER:
    rw_rnd_counter = 3;
    for (int i = 0; i < dists.size(); i++)
    {
      if (dists.at(i) > bg_distance)
      {
        bg_distance = dists.at(i);
        bg_dir = i;
      }
    }
    dists.clear();
    if (bg_dir == 3)
    {
      rw_state = RWS_CHECK_STEPS;
      direction = MOVE_FOWARD;
    }
    else if (bg_dir == 2)
    {
      rw_step_counter = MOVE_CTRL_QRT;
      rw_state = RWS_REVIVE;
    }
    else if (bg_dir == 1)
    {
      rw_step_counter = MOVE_CTRL_2QRT;
      rw_state = RWS_REVIVE;
    }
    else if (bg_dir == 0)
    {
      rw_step_counter = MOVE_CTRL_3QRT;
      rw_state = RWS_REVIVE;
    }
    break;

  case RWS_REVIVE:
    if (rw_step_counter > 0)
    {
      rw_step_counter--;
      direction = ROTATE_RIGHT;
    }
    else
      rw_state = RWS_CHECK_STEPS;

    break;
  default:
    break;
  }
  return direction;
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

/**
*   @brief  Funkce volana v pripade pritomnosti dat v topicu ult_sensor
*
*   @param  msg data z topicu
*   @return void
*/
void ultSensorCallback(const std_msgs::Float32::ConstPtr & msg)
{
  last_distance = msg->data;
}

/**
*   @brief  Funkce volana v pripade pritomnosti dat v topicu whiskers_sensor (dotykovy senzor)
*
*   @param  msg data z topicu
*   @return void
*/
void whiskersSensorCallback(const std_msgs::Bool::ConstPtr & msg)
{
  last_touch = msg->data;
}

/**
*   @brief  Funkce volana v pripade pritomnosti dat v topicu
*   pro ovladani tohoto node automode_ctrl
*
*   @param  msg data z topicu
*   @return void
*/
void CtrlCallback(const std_msgs::String::ConstPtr & msg)
{
  std::string input = msg->data;
  std::vector<std::string> cmds = split(input, ';');

  for (auto cmd : cmds)
  {
    std::vector<std::string> cmd_parts = split(cmd, '=');
    //obsahuje 2 casti
    std::cout << cmd << std::endl;

    if (cmd_parts.size() == 2)
    {
      //vzdalene rizeni
      if (cmd_parts[0].compare("motorCtrl") == 0)
      {
        std::vector<std::string> param = split(cmd_parts[1], ':');
        if (param.size() == 2)
        {

          int step = std::atoi(param[0].c_str());
          manual_state = std::atoi(param[1].c_str());
        }
      }
      //zapnuti/vypnuti automatickeho prozkoumavani
      else if (cmd_parts[0].compare("autoCtrl") == 0)
      {
        int level_cmp = std::atoi(cmd_parts[1].c_str());
        if (level_cmp == NodeLevel::manual)
          level = NodeLevel::colisionAvoiding;
        else
          level = NodeLevel::manual;
      }
      //rychlost motoru
      else if (cmd_parts[0].compare("motorSpeedCtrl") == 0)
      {
        std::vector<std::string> param = split(cmd_parts[1], ':');
        if (param.size() == 4)
        {
          std::stringstream ss;

          ss << param[0] << ";";
          ss << param[1] << ";";
          ss << param[2] << ";";
          ss << param[3] << ";";

          motor_speeds.data = ss.str();
        }
      }
    }
  }
}
/**
*   @brief Rozdeli string dle parametru delimeter
*
*   @param s string pro rozdeleni
*   @return std::vector<std::string> vraci vector stringu na ktere byl str puvodni rozdelen
*/
std::vector<std::string> split(const std::string& s, char delimiter)
{
  std::vector<std::string> tokens;
  std::string token;
  std::istringstream tokenStream(s);
  while (std::getline(tokenStream, token, delimiter))
  {
    tokens.push_back(token);
  }
  return tokens;
}