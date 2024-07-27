// common.c
#include "common.h"
#include "mock_i2c_interface.h"

static struct bezier2d curve2d;
static struct bezier3d curve3d;

void setUp(void)
{
    bezier2d_init(&curve2d);
    bezier3d_init(&curve3d);
    set_mock_i2c_interface();
}

void tearDown(void)
{
    free(curve2d.xpos);
    free(curve2d.ypos);
    free(curve3d.xpos);
    free(curve3d.ypos);
    free(curve3d.zpos);
    reset_mock_i2c_counters();
}

struct bezier2d *get_curve2d(void)
{
    return &curve2d;
}
struct bezier3d *get_curve3d(void)
{
    return &curve3d;
}
