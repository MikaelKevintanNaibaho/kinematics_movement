// bezier3d_tests.c
#include <unity/unity.h>
#include "test_bezier3d.h"
#include "common.h"

void test_bezier3d_init(void)
{
    struct bezier3d *curve = get_curve3d();
    TEST_ASSERT_NULL(curve->xpos);
    TEST_ASSERT_NULL(curve->ypos);
    TEST_ASSERT_NULL(curve->zpos);
    TEST_ASSERT_EQUAL(0, curve->npoints);
}

void test_bezier3d_addpoint(void)
{
    struct bezier3d *curve = get_curve3d();
    bezier3d_addpoint(curve, 1.0, 1.0, 1.0);
    TEST_ASSERT_NOT_NULL(curve->xpos);
    TEST_ASSERT_NOT_NULL(curve->ypos);
    TEST_ASSERT_NOT_NULL(curve->zpos);
    TEST_ASSERT_EQUAL(1, curve->npoints);
    TEST_ASSERT_EQUAL_FLOAT(1.0, curve->xpos[0]);
    TEST_ASSERT_EQUAL_FLOAT(1.0, curve->ypos[0]);
    TEST_ASSERT_EQUAL_FLOAT(1.0, curve->zpos[0]);
}

void test_bezier3d_getpos(void)
{
    struct bezier3d *curve = get_curve3d();
    bezier3d_addpoint(curve, 0.0, 0.0, 0.0);
    bezier3d_addpoint(curve, 1.0, 1.0, 1.0);
    bezier3d_addpoint(curve, 2.0, 0.0, 2.0);

    float xret, yret, zret;
    bezier3d_getpos(curve, 0.5, &xret, &yret, &zret);

    // Debugging print
    printf("Expected: x=1.0, y=0.5, z=1.0; Actual: x=%f, y=%f, z=%f\n", xret, yret, zret);

    TEST_ASSERT_EQUAL_FLOAT(1.0, xret); // Adjust if necessary
    TEST_ASSERT_EQUAL_FLOAT(0.5, yret); // Adjust if necessary
    TEST_ASSERT_EQUAL_FLOAT(1.0, zret); // Adjust if necessary
}

void test_bezier3d_generate_curve(void)
{
    struct bezier3d *curve = get_curve3d();
    bezier3d_generate_curve(curve, 0.0, 0.0, 0.0, 1.0, 1.0, 1.0, 2.0, 2.0, 2.0);
    TEST_ASSERT_EQUAL(3, curve->npoints);
    TEST_ASSERT_EQUAL_FLOAT(0.0, curve->xpos[0]);
    TEST_ASSERT_EQUAL_FLOAT(1.0, curve->xpos[1]);
    TEST_ASSERT_EQUAL_FLOAT(2.0, curve->xpos[2]);
    TEST_ASSERT_EQUAL_FLOAT(0.0, curve->ypos[0]);
    TEST_ASSERT_EQUAL_FLOAT(1.0, curve->ypos[1]);
    TEST_ASSERT_EQUAL_FLOAT(2.0, curve->ypos[2]);
    TEST_ASSERT_EQUAL_FLOAT(0.0, curve->zpos[0]);
    TEST_ASSERT_EQUAL_FLOAT(1.0, curve->zpos[1]);
    TEST_ASSERT_EQUAL_FLOAT(2.0, curve->zpos[2]);
}
