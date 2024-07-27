// common.h
#ifndef COMMON_H
#define COMMON_H

#include <unity/unity.h>
#include "bezier.h"

void setUp(void);
void tearDown(void);

struct bezier2d *get_curve2d(void);
struct bezier3d *get_curve3d(void);

#endif // COMMON_H
