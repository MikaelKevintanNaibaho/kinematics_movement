#include "leg.h"

void initialize_leg(SpiderLeg *leg, const char *name, int servo_ch1, int servo_ch2, int servo_ch3)
{
    strcpy(leg->name, name);
    leg->servo_channles[0] = servo_ch1;
    leg->servo_channles[1] = servo_ch2;
    leg->servo_channles[2] = servo_ch3;
}

void initialize_all_legs(SpiderLeg *legs[NUM_LEGS])
{
    initialize_leg(legs[0], "KIRI_DEPAN", SERVO_CHANNEL_1, SERVO_CHANNEL_2, SERVO_CHANNEL_3);
    initialize_leg(legs[1], "KIRI_BELAKANG", SERVO_CHANNEL_4, SERVO_CHANNEL_5, SERVO_CHANNEL_6);
    initialize_leg(legs[2], "KANAN_BELAKANG", SERVO_CHANNEL_7, SERVO_CHANNEL_8, SERVO_CHANNEL_9);
    initialize_leg(legs[3], "KANAN_DEPAN", SERVO_CHANNEL_10, SERVO_CHANNEL_11, SERVO_CHANNEL_12);
}
