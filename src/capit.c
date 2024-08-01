#include "capit.h"

void set_angle_mg(int angle)
{
    set_pwm_angle(CAPIT_BASE, angle);
}

void set_angle_sg(int angle)
{
    set_pwm_angle(CAPIT_UJUNG, angle);
}

void buka_capit(void)
{
    set_angle_sg(180);
}
void tutup_capit(void)
{
    set_angle_sg(80);
}
void turun_capit(void)
{
    set_angle_mg(180);
}
void naik_capit(void)
{
    set_angle_mg(90);
}

void capit(void)
{
    turun_capit();
    sleep(1);
    buka_capit();
}
void letak(void)
{
    tutup_capit();
    sleep(1);
    naik_capit();
}
