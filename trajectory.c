#include "trajectory.h"

//fungsi untuk menciptakan trajectory swing ke depan
void generate_swing_phase(struct bezier2d *curve, float startx, float startz, float stride_length, float swing_height, int leg_type)
{
    float controlx = 0;
    /*ditambahkan dari posisi awal z karena posisi awal adalah negative(-), jadi agar mengangkat kaki maka ketinggian harus dikurangi*/
    float controlz = startz + 2 * swing_height;

    float endx = 0;
    if (leg_type == 1) {
        controlx = startx + stride_length / 2;
        endx = startx + stride_length;
    } else if (leg_type == -1) {
        controlx = startx - stride_length / 2;
        endx = startx - stride_length;
    }
    float endz = startz;

    bezier2d_generate_curve(curve, startx, startz, controlx, controlz, endx, endz);
}

void generate_stance_phase(struct bezier2d *curve, float startx, float startz, float stride_lenght, int leg_type)
{
    float controlx = 0;
    float controlz = startz - SWING_PUSH_BACK;
    float endx = 0;
    if (leg_type == 1) {
        controlx = startx - stride_lenght / 2;
        endx = startx - stride_lenght;
    } else if (leg_type == -1) {
        controlx = startx + stride_lenght / 2;
        endx = startx + stride_lenght;
    }
    float endz = startz;

    bezier2d_generate_curve(curve, startx, startz, controlx, controlz, endx, endz);
}


void generate_walk_trajectory(struct bezier2d *curve, SpiderLeg *leg, float stride_length, float swing_height, LegPosition leg_positions)
{
    //get current position
    float startx_swing = leg->joints[3][0] - stride_length;
    float startz_swing = leg->joints[3][2];

    //buat swing phase
    generate_swing_phase(curve, startx_swing, startz_swing, stride_length, swing_height, FRONT);

    //update start position untuk stance phase
    float startx_stance = startx_swing + stride_length;
    float startz_stance = startz_swing;

    generate_stance_phase(curve, startx_stance, startz_stance, stride_length, FRONT);

}

//trajectory untuk kaki belakang karena beda orientasi
void generate_walk_back_leg_trajectory(struct bezier2d *curve, SpiderLeg *leg, float stride_length, float swing_height, LegPosition leg_positions)
{
     //get current position
    float startx_swing = leg->joints[3][0];
    float startz_swing = leg->joints[3][2];

    //buat swing phase
    

    //update start position untuk stance phase
    float startx_stance = startx_swing - stride_length;
    float startz_stance = startz_swing;

    generate_stance_phase(curve, startx_stance, startz_stance, stride_length, BACK);
    generate_swing_phase(curve, startx_swing, startz_swing, stride_length, swing_height, BACK);

}
