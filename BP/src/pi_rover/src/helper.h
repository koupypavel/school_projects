#ifndef _HELPER_H_
#define _HELPER_H_


//kontrolni prikazy
typedef enum
{
  NONE          = 0x00,
  MOVE_FOWARD   = 0x01,
  MOVE_BACKWARD = 0x02,
  MOVE_LEFT     = 0x03,
  MOVE_RIGHT    = 0x04,
  ROTATE_LEFT   = 0x05,
  ROTATE_RIGHT  = 0x06
} CtrlState;

typedef enum {
  manual            = 0x00,
  colisionAvoiding  = 0x01,
  wandererWalk      = 0x02,
  randomWalk        = 0x03,
  SLAMacc           = 0x04
} NodeLevel;

enum RandomWalkState
{
  RWS_INIT            = 0x01,
  RWS_CHECK_STEPS     = 0x02,
  RWS_ROTATE          = 0x03,
  RWS_CHOOSER         = 0x04,
  RWS_REVIVE          = 0x05
};
enum WandererWalkState
{
  WWS_INIT            = 0x01,
  WWS_CHECK_DISTANCE  = 0x02,
  WWS_OBJECT_ALIGNPP  = 0x03
};


enum HardCollisionState
{
  HCS_INIT        = 0x01,
  HCS_COL_CHECK   = 0x02,
  HCS_BACK        = 0x03,
  HCS_COL_SOLVE   = 0x04,
  HCS_COL_CHOOSER = 0x05,
  HCS_COL_REVIVE  = 0x06,
};

#define MOVE_CTRL_CLK   32
#define MOVE_CTRL_QRT   8
#define MOVE_CTRL_2QRT  16
#define MOVE_CTRL_3QRT  24
#define MOVE_CTRL_BACK  7

//general defines
#define AUTONOMODE_LOOP_RATE    3.9
#define G_RQ_SIZE               1000
#define G_VAL_HISTORY           10

//nav defines
#define NAV_BREAK_DISTANCE      10
#define NAV_GO_DISTANCE         15


#define RANDOM_STEP_MAX             200
//USED GPIO
#define GPIO_TRIGGER             4
#define GPIO_ECHO                5
#define GPIO_WHISKERS            6

#endif /* _HELPER_H_ */