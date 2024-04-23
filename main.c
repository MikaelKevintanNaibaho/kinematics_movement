#include "pwm_servo.h"
#include "ik.h"
#include "move.h"

int main(void) {
    PCA9685_init();

    SpiderLeg leg_kiri_depan;
    SpiderLeg leg_kiri_belakang;
    SpiderLeg leg_kanan_belakang;
    SpiderLeg leg_kanan_depan;

    // Declare an array of pointers to SpiderLeg instances
    SpiderLeg *legs[NUM_LEGS] = {&leg_kiri_depan, &leg_kiri_belakang, &leg_kanan_belakang, &leg_kanan_depan};

    // Pass the array of pointers to SpiderLeg instances to the initialize_all_legs function
    initialize_all_legs(legs);

    float angles_kiri_depan[3] = {45.0 , 130.0, 130.0};
    float angles_kiri_belakang[3] = {45.0 , 130.0, 130.0};
    float angles_kanan_belakang[3] = {45.0 , 130.0, 130.0};
    float angles_kanan_depan[3] = {45.0 , 130.0, 130.0};

    // Pass the address of each leg and its respective angles to the set_angles function
    set_angles(&leg_kiri_depan, angles_kiri_depan);
    set_angles(&leg_kiri_belakang, angles_kiri_belakang);
    set_angles(&leg_kanan_belakang, angles_kanan_belakang);
    set_angles(&leg_kanan_depan, angles_kanan_depan);
    
    return 0;
}