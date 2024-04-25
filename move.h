#ifndef MOVE_H
#define MOVE_H

#include "ik.h"
#include "gsl/gsl_spline.h"
#include <stdio.h>
#include <time.h>

typedef enum {
    MOVE_FORWARD,
    MOVE_BACKWARD,
    MOVE_LEFT,
    MOVE_RIGHT,
    TURN_LEFT,
    TURN_RIGHT
} MovementCommand;

struct bezier2d {
    float *xpos;
    float *ypos;
    int npoints;
};

#define STRIDE_LENGTH 30.0
#define SWING_HEIGTH 50.0
#define NUM_POINTS 50
#define DESIRED_TIME 0.1
#define GROUP_SIZE 2
#define LAG_TIME 0.5

void bezier2d_init(struct bezier2d *curve);
void bezier2d_addPoint(struct bezier2d *curve, float x, float y);
void bezier2d_getPos(struct bezier2d *curve, float t, float *xret, float *yret);
void bezier2d_generate_curve(struct bezier2d *curve, float startx, float startz, float controlx, float controlz, float endx, float endz);
void bezier2d_generate_straight_back(struct bezier2d *stright_back, float startx, float startz, float endx, float endy);

void generate_walk_trajectory(struct bezier2d *curve, SpiderLeg *leg, float stride_length, float swing_height, LegPosition position_leg);
void generate_stright_back_trajectory(struct bezier2d *stright_back, SpiderLeg *leg, float stride_length);
void print_trajectory(struct bezier2d *curve, int num_points) ;
void save_trajectory_points(struct bezier2d *curve, const char *filename, int num_points) ;
void update_leg_position_single(struct bezier2d *curve, int number_points, SpiderLeg *leg, LegPosition position_leg);
void update_leg_position_with_lag(struct bezier2d *curve, int number_points, SpiderLeg *legs[], int num_legs, int group_size, float lag_time);
// void walk_forward(SpiderLeg *legs[NUM_LEGS], float stride_length, float swing_height, int num_points, LegPosition position_leg[NUM_LEGS]);

void crawl_gait(SpiderLeg *legs[NUM_LEGS], LegPosition position_leg[NUM_LEGS]);
#endif //MOVE_H