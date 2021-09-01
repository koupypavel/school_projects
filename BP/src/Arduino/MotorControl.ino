/**
 *  @file    MotorControl.ino
 *  @author  Pavel Koupy (xkoupy00)
 *  @date    10.5.2018
 *  @version 1.0
 *
 *  @brief uzel pro rizeni motoru
 *
 */


#include <ros.h>
#include <std_msgs/UInt16.h>
#include <std_msgs/String.h>
#include <AFMotor.h>

#define DELAY 250
//kontrolni prikazy
typedef enum {CTRL_F      = 0x01, //dopredu 
              CTRL_B      = 0x02, // zpatecka
              CTRL_L      = 0x03, // vlevo
              CTRL_R      = 0x04, //  vpravo
              CTRL_ROT_L  = 0x05, // rotace
              CTRL_ROT_R  = 0x06, 
              STOP        = 0x00
             } states;

//pocatecni stav
states state = STOP;
//aktualni rychlost
unsigned int currentSpeedFL = 200;
unsigned int currentSpeedFR = 255;
unsigned int currentSpeedBL = 200;
unsigned int currentSpeedBR = 255;

AF_DCMotor motorFL(3, MOTOR12_1KHZ);
AF_DCMotor motorFR(4, MOTOR12_1KHZ);
AF_DCMotor motorBL(2, MOTOR12_1KHZ);
AF_DCMotor motorBR(1, MOTOR12_1KHZ);

ros::NodeHandle  nh;
/**
 * @brief Uvolneni motoru
 *
 *   @return void
 */
void release_motors()
{
  motorFL.run(RELEASE); 
  motorFR.run(RELEASE); 
  motorBL.run(RELEASE);   
  motorBR.run(RELEASE);
}

 /**
 * @brief Pohyb dopredu
 *
 *   @return void
 */
void moveFW()
{
  motorFL.run(FORWARD);   
  motorFR.run(FORWARD);
  motorBL.run(FORWARD);   
  motorBR.run(FORWARD);     
  delay(DELAY);
  release_motors();
}

 /**
 * @brief Pohyb vzad
 *
 *   @return void
 */
void moveBW()
{
  motorFL.run(BACKWARD);   
  motorFR.run(BACKWARD);
  motorBL.run(BACKWARD);   
  motorBR.run(BACKWARD);     
  delay(DELAY);
  release_motors();
}

 /**
 * @brief Pohyb doleva
 *
 *   @return void
 */
void moveLeft()
{
  motorFL.run(BACKWARD);   
  motorFR.run(FORWARD);
  motorBL.run(FORWARD);   
  motorBR.run(BACKWARD);     
  delay(DELAY);
  release_motors();
}
 /**
 * @brief Pohyb doprava
 *
 *   @return void
 */
void moveRight()
{
  motorFL.run(FORWARD);   
  motorFR.run(BACKWARD);
  motorBL.run(BACKWARD);   
  motorBR.run(FORWARD);     
  delay(DELAY);
  release_motors(); 
}
 /**
 * @brief Rotace doleva
 *
 *   @return void
 */
void RotateLeft()
{
  motorFL.run(BACKWARD);   
  motorFR.run(FORWARD);
  motorBL.run(BACKWARD);   
  motorBR.run(FORWARD);     
  delay(DELAY);
  release_motors(); 
}
 /**
 * @brief Rotace doprava
 *
 *   @return void
 */
void RotateRight()
{
  motorFL.run(FORWARD);   
  motorFR.run(BACKWARD);
  motorBL.run(FORWARD);   
  motorBR.run(BACKWARD);     
  delay(DELAY);
  release_motors(); 
}
 /**
 * @brief Callback pro ros subscriber
 * pro rizeni
 *
 * @param cmd_msg zprava s kontrolnim prikazem
 * @return void
 */
void handleState (const std_msgs::UInt16& cmd_msg)
{
  switch (cmd_msg.data)
  {
  case CTRL_F:
    moveFW();
    break;
  case CTRL_B:
    moveBW();
    break;
  case CTRL_L:
    moveLeft();
    break;
  case CTRL_R:
    moveRight();
    break;
  case CTRL_ROT_L:
    RotateLeft();
    break;
  case CTRL_ROT_R:
    RotateRight();
    break;
  case STOP:
    //DO NOTHING
    break;
  }
}

 /**
 * @brief Funkce rozdelu zadany string
 * dle separatoru a vraci hodnotu dle indexu
 * https://stackoverflow.com/questions/9072320/split-string-into-string-array
 *
 * @param data zprava s kontrolnim prikazem
 * @param separator delici znak
 * @param index ktery substring vratit
 * @return String vraci substring z rozdeleno stringu dle indexu
 */
String getValue(String data, char separator, int index)
{
  int found = 0;
  int strIndex[] = {0, -1};
  int maxIndex = data.length()-1;

  for(int i=0; i<=maxIndex && found<=index; i++){
    if(data.charAt(i)==separator || i==maxIndex){
        found++;
        strIndex[0] = strIndex[1]+1;
        strIndex[1] = (i == maxIndex) ? i+1 : i;
    }
  }

  return found>index ? data.substring(strIndex[0], strIndex[1]) : "";
}
 /**
 * @brief Callback pro ros subscriber
 * pro rizeni
 *
 * @param cmd_msg zprava s kontrolnim prikazem
 * @return void
 */
void setMotorSpeed (const std_msgs::String& msg)
{
  String cmd(msg.data);
  String motorFLstr = getValue(cmd,';',0);
  String motorFRstr = getValue(cmd,';',1);
  String motorBLstr = getValue(cmd,';',2);
  String motorBRstr = getValue(cmd,';',3);

  currentSpeedFL = motorFLstr.toInt();
  currentSpeedFR = motorFRstr.toInt();
  currentSpeedBL = motorBLstr.toInt();
  currentSpeedBR = motorBRstr.toInt();
  
  release_motors();
  motorFL.setSpeed(currentSpeedFL); 
  motorFR.setSpeed(currentSpeedFR); 
  motorBL.setSpeed(currentSpeedBL); 
  motorBR.setSpeed(currentSpeedBR);

}

//definice subscriberu
ros::Subscriber<std_msgs::UInt16> sub_ctrl("motors_ctrl", handleState);
ros::Subscriber<std_msgs::String> sub_speed("motors_speed", setMotorSpeed);
//pocatecni inicializace
void setup(){
  nh.initNode();
  nh.subscribe(sub_ctrl);
  nh.subscribe(sub_speed);

  //startup init
  state = STOP;
  motorFL.setSpeed(currentSpeedFL); 
  motorFR.setSpeed(currentSpeedFR); 
  motorBL.setSpeed(currentSpeedBL); 
  motorBR.setSpeed(currentSpeedBR);
}
//hlavni smycka
void loop(){
  nh.spinOnce();
}
