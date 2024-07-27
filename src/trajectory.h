#ifndef TRAJECTORY_H
#define TRAJECTORY_H

#include "bezier.h"
#include "leg.h"

#define SWING_PUSH_BACK 15
#define FRONT 1
#define BACK -1

void generate_swing_phase(struct bezier2d *curve, float startx, float startz, float stride_length, float swing_height, int leg_type);
void generate_stance_phase(struct bezier2d *curve, float startx, float startz, float stride_leght, int leg_type);
void generate_walk_trajectory(struct bezier2d *curve, SpiderLeg *leg, float stride_length, float swing_height, LegPosition leg_positions);
void generate_walk_back_leg_trajectory(struct bezier2d *curve, SpiderLeg *leg, float stride_length, float swing_height, LegPosition leg_positions);

#endif //  TRAJECTORY_H