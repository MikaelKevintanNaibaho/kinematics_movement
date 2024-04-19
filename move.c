#include "move.h"
#include <stdio.h>

void bezier2d_init(struct bezier2d *curve) {
    curve->xpos = NULL;
    curve->ypos = NULL;
    curve->npoints = 0;
}

void bezier2d_addPoint(struct bezier2d *curve, float x, float y) {
    curve->npoints++;
    curve->xpos = (float *)realloc(curve->xpos, curve->npoints * sizeof(float));
    curve->ypos = (float *)realloc(curve->ypos, curve->npoints * sizeof(float));
    curve->xpos[curve->npoints - 1] = x;
    curve->ypos[curve->npoints - 1] = y;
}

void bezier2d_getPos(struct bezier2d *curve, float t, float *xret, float *yret) {
    int ii, ij;
    float *x, *y;

    if (curve->npoints == 0) {
        *xret = 0;
        *yret = 0;
        return;
    }

    x = (float *)malloc(curve->npoints * sizeof(float));
    y = (float *)malloc(curve->npoints * sizeof(float));

    // load with current points
    for (ii = 0; ii < curve->npoints; ii++) {
        x[ii] = curve->xpos[ii];
        y[ii] = curve->ypos[ii];
    }

    // iterate over levels
    for (ii = 0; ii < curve->npoints - 1; ii++) {
        for (ij = 0; ij < curve->npoints - ii - 1; ij++) {
            x[ij] = (1.0 - t) * x[ij] + t * x[ij + 1];
            y[ij] = (1.0 - t) * y[ij] + t * y[ij + 1];
        }
    }

    *xret = x[0];
    *yret = y[0];

    free(x);
    free(y);
}

void bezier2d_generate_curve(struct bezier2d *curve, float startx, float startz, float controlx, float controlz, float endx, float endz){
    bezier2d_addPoint(curve, startx, startz);
    bezier2d_addPoint(curve, controlx, controlz);
    bezier2d_addPoint(curve, endx, endz);
}


void generate_walk_trajectory(struct bezier2d *curve, SpiderLeg *leg, float stride_legth, float swing_hight) 
{
    //get current position
    float startx = leg->joints[3][0];
    float startz = leg->joints[3][2];

    //control points
    float controlx = startx + stride_legth;
    float controlz = startz + swing_hight;
    float endx = startx;
    float endz = startz;

    //buar bezier curve
    bezier2d_generate_curve(curve, startx, startz, controlx, controlz, endx, endz);

}


void print_trajectory(struct bezier2d *curve, int num_points) {
    printf("Trajectory Points:\n");
    for (int i = 0; i <= num_points; i++) {
        float t = (float)i / num_points;
        float x, y;
        bezier2d_getPos(curve, t, &x, &y);
        printf("Point %d: (%.2f, %.2f)\n", i, x, y);
    }
}


void save_trajectory_points(struct bezier2d *curve, const char *filename, int num_points) {
    FILE *file = fopen(filename, "w");
    if (file == NULL) {
        printf("Error opening file.\n");
        return;
    }

    for (int i = 0; i <= num_points; i++) {
        float t = (float)i / num_points;
        float x, z;
        bezier2d_getPos(curve, t, &x, &z);
        fprintf(file, "%.2f %.2f\n", x, z);
    }

    fclose(file);
}
void update_leg_position_with_velocity(struct bezier2d *curve, int number_points, SpiderLeg *leg, gsl_matrix *intermediate_matrices[])
{
    printf("updating leg position with fixed delay\n");

    // Handle the first point separately
    float x, z;
    bezier2d_getPos(curve, 0, &x, &z); // Get position of the first point
    float y = leg->joints[3][1];
    float target_positions[3] = {x, y, z};
    inverse_kinematics(leg, target_positions, intermediate_matrices);

    // Iterate over remaining points
    for (int i = 1; i < number_points; i++){
        float t = (float)i / number_points;
        bezier2d_getPos(curve, t, &x, &z);
        
        // Print position of current point
        printf("Point %d: (%.2f, %.2f)\n", i, x, z);

        // Calculate target position for inverse kinematics
        float y = leg->joints[3][1];
        float target_positions[3] = {x, y, z};

        // Update leg position using inverse kinematics
        inverse_kinematics(leg, target_positions, intermediate_matrices);

    }
}