#ifndef MOVE_H
#define MOVE_H

#include "gsl/gsl_spline.h"
#include "ik.h"
#include <pthread.h>
#include <stdbool.h>
#include <unistd.h>
#include "interrupt.h"
#include "trajectory.h"

typedef enum {
    MOVE_FORWARD,
    MOVE_BACKWARD,
    MOVE_LEFT,
    MOVE_RIGHT,
    TURN_LEFT,
    TURN_RIGHT
} MovementCommand;

struct LegThreadData
{
    struct bezier2d *curve;
    SpiderLeg *leg;
    float stride_length;
    float swing_height;
    LegPosition position_leg;
};

#define STRIDE_LENGTH 100.0
#define SWING_HEIGHT 70
#define NUM_POINTS 50
#define DESIRED_TIME 0.00001
#define GROUP_SIZE 2
#define LAG_TIME 0.5
#define NUM_PHASES 2
#define FORWARD_DISPLACEMENT 0.01
#define LEG_HEIGHT_OFFSET 20.0

#define PHASE_OFFSET_1 0.0 // Phase offset for leg 1
#define PHASE_OFFSET_2 0.25 // Phase offset for leg 2
#define PHASE_OFFSET_3 0.5 // Phase offset for leg 3
#define PHASE_OFFSET_4 0.75 // Phase offset for leg 4

// Constants for balance control
#define PITCH_THRESHOLD 5.0 // Threshold for pitch deviation (degrees)
#define ROLL_THRESHOLD 5.0 // Threshold for roll deviation (degrees)
#define LEG_ADJUSTMENT_ANGLE 2.0 // Angle to adjust leg position (degrees)

void generate_turn_left_trajectory(struct bezier3d *curve, SpiderLeg *leg, float stride_length,
                                   float swing_height, LegPosition position_leg);

// making leg posiiton update
void update_leg_position_with_velocity(struct bezier2d *curve, int number_points, SpiderLeg *leg,
                                       LegPosition position_leg);

void update_leg_wave_gait(struct bezier2d curve[NUM_LEGS], int num_points,
                          SpiderLeg *legs[NUM_LEGS], LegPosition leg_positions[NUM_LEGS]);
void update_leg_trot_gait(struct bezier2d curve[NUM_LEGS], int num_points,
                          SpiderLeg *legs[NUM_LEGS], LegPosition leg_positions[NUM_LEGS]);

void update_leg_left(struct bezier3d curve[NUM_LEGS], int num_points, SpiderLeg *legs[NUM_LEGS],
                     LegPosition leg_positons[NUM_LEGS]);
const char *leg_position_to_string(LegPosition position);

void wave_forward(SpiderLeg *legs[NUM_LEGS], LegPosition leg_positions[NUM_LEGS]);

void *move_leg(void *thread_data);

// movement relative function
void stand_position(void);
void move_forward(void);
void move_left_turn(void);
#endif // MOVE_H
