#include "state_machine.h"

RobotState current_state = STATE_IDLE;
RobotEvent current_event = EVENT_NONE;

void handle_event(RobotEvent event)
{
    switch (current_state) {
    case STATE_IDLE:
        if (event == EVENT_START_MOVE_FORWARD) {
            current_state = STATE_MOVE_FORWARD;
            move_forward();
        } else if (event == EVENT_START_MOVE_LEFT) {
            current_state = STATE_MOVE_LEFT;
            move_left_turn();
        }
        break;

    case STATE_MOVE_FORWARD:
        if (event == EVENT_STOP) {
            current_state == STATE_IDLE;
            stand_position();
            is_program_running = 0;
        }
        break;

    case STATE_MOVE_LEFT:
        if (event == EVENT_STOP) {
            current_state == STATE_IDLE;
            stand_position();
            is_program_running = 0;
        }
        break;

    default:
        break;
    }

    current_event = EVENT_NONE;
}
void trigger_event(RobotEvent event)
{
    current_event = event;
    handle_event(event);
}