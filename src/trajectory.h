#ifndef TRAJECTORY_H
#define TRAJECTORY_H

#include "bezier.h"
#include "leg.h"
#include "stdbool.h"

#define SWING_PUSH_BACK 15
#define FRONT 1
#define BACK -1

void generate_swing_phase(struct bezier2d *curve, float startx, float startz, float stride_length,
                          float swing_height, int leg_type);
void generate_stance_phase(struct bezier2d *curve, float startx, float startz, float stride_leght,
                           int leg_type);
void generate_walk_trajectory(struct bezier2d *curve, SpiderLeg *leg, float stride_length,
                              float swing_height, LegPosition leg_positions);
void generate_walk_back_leg_trajectory(struct bezier2d *curve, SpiderLeg *leg, float stride_length,
                                       float swing_height, LegPosition leg_positions);
void calculate_swing_stance_phase(float startx, float startz, float stride_length,
                                  float swing_height, int leg_type, float *controlx,
                                  float *controlz, float *endx, float *endz, bool is_swing_phase);
void print_trajectory(struct bezier2d *curve, int num_points);
void print_trajectory_3d(struct bezier3d *curve, int num_points);
void save_trajectory_points(struct bezier2d *curve, const char *filename, int num_points);

#endif //  TRAJECTORY_H
