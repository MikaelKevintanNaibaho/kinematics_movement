#ifndef MOVE_H
#define MOVE_H

#include "gsl/gsl_spline.h"
#include "ik.h"
#include <pthread.h>
#include <stdbool.h>
#include <stdio.h>
#include <time.h>

typedef enum
{
    MOVE_FORWARD,
    MOVE_BACKWARD,
    MOVE_LEFT,
    MOVE_RIGHT,
    TURN_LEFT,
    TURN_RIGHT
} MovementCommand;

struct bezier2d
{
    float *xpos;
    float *ypos;
    int npoints;
};

struct LegThreadData
{
    struct bezier2d *curve;
    SpiderLeg *leg;
    float stride_length;
    float swing_height;
    LegPosition position_leg;
};

#define STRIDE_LENGTH 100.0
#define SWING_HEIGTH 50.0
#define NUM_POINTS 30
#define DESIRED_TIME 0.00001
#define GROUP_SIZE 2
#define LAG_TIME 0.5
#define NUM_PHASES 2

#define PHASE_OFFSET_1 0.0 // Phase offset for leg 1
#define PHASE_OFFSET_2 0.25 // Phase offset for leg 2
#define PHASE_OFFSET_3 0.5 // Phase offset for leg 3
#define PHASE_OFFSET_4 0.75 // Phase offset for leg 4

void bezier2d_init(struct bezier2d *curve);
void bezier2d_addPoint(struct bezier2d *curve, float x, float y);
void bezier2d_getPos(struct bezier2d *curve, float t, float *xret, float *yret);
void bezier2d_generate_curve(struct bezier2d *curve, float startx, float startz, float controlx,
                             float controlz, float endx, float endz);
void bezier2d_generate_straight_back(struct bezier2d *stright_back, float startx, float startz,
                                     float endx, float endy);

void generate_walk_trajectory(struct bezier2d *curve, SpiderLeg *leg, float stride_length,
                              float swing_height, LegPosition position_leg);
void generate_stright_back_trajectory(struct bezier2d *stright_back, SpiderLeg *leg,
                                      float stride_length);
void print_trajectory(struct bezier2d *curve, int num_points);
void save_trajectory_points(struct bezier2d *curve, const char *filename, int num_points);
void update_leg_position_with_velocity(struct bezier2d *curve, int number_points, SpiderLeg *leg,
                                       LegPosition position_leg);
void walk_forward(SpiderLeg *legs[NUM_LEGS], float stride_length, float swing_height,
                  int num_points, LegPosition position_leg[NUM_LEGS]);

void crawl_gait(SpiderLeg *legs[NUM_LEGS], LegPosition position_leg[NUM_LEGS]);
void ripple_gait(SpiderLeg *legs[NUM_LEGS], LegPosition position_leg[NUM_LEGS]);
void wave_gait(SpiderLeg *legs[NUM_LEGS], LegPosition leg_positions[NUM_LEGS]);

void update_leg_wave_gait(struct bezier2d curve[NUM_LEGS], int num_points, SpiderLeg *legs[NUM_LEGS], LegPosition leg_positions[NUM_LEGS]);
const char* leg_position_to_string(LegPosition position);

void wave_forward(SpiderLeg *legs[NUM_LEGS], LegPosition leg_positions[NUM_LEGS]);

void *move_leg(void *thread_data);
#endif // MOVE_H