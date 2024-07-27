#ifndef STATE_MACHINE_H
#define STAE_MACHINE_H

#include "move.h"

typedef enum {
    STATE_IDLE,
    STATE_MOVE_FORWARD,
    STATE_MOVE_LEFT,
} RobotState;

typedef enum {
    EVENT_NONE,
    EVENT_START_MOVE_FORWARD,
    EVENT_START_MOVE_LEFT,
    EVENT_STOP,
} RobotEvent;

void handle_event(RobotEvent event);
void trigger_event(RobotEvent event);

#endif //STATE_MACHINE_H