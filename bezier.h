#ifndef BEZIER_H
#define BEZIER_H

#include <stddef.h>
#include <stdlib.h>

struct bezier2d
{
    float *xpos;
    float *ypos;
    int npoints;
};

void bezier2d_init(struct bezier2d *curve);
void bezier2d_addPoint(struct bezier2d *curve, float x, float y);
void bezier2d_getPos(struct bezier2d *curve, float t, float *xret, float *yret);
void bezier2d_generate_curve(struct bezier2d *curve, float startx, float startz, float controlx,
                             float controlz, float endx, float endz);
void bezier2d_generate_straight_back(struct bezier2d *stright_back, float startx, float startz,
                                     float endx, float endy);


#endif /*BEZIER_H*/