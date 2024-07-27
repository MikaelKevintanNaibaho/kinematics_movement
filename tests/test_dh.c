#include <unity/unity.h>
#include "dh.h"

// Helper function to compare matrices
void assert_matrices_equal(gsl_matrix *m1, gsl_matrix *m2, double tolerance)
{
    for (size_t i = 0; i < m1->size1; i++) {
        for (size_t j = 0; j < m1->size2; j++) {
            TEST_ASSERT_TRUE_MESSAGE(fabs(gsl_matrix_get(m1, i, j) - gsl_matrix_get(m2, i, j))
                                         < tolerance,
                                     "Matrices are not equal within the tolerance");
        }
    }
}

void test_init_DH_params(void)
{
    DHParameters params;
    float alpha = M_PI / 4.0;
    float a = 2.0;
    float d = 1.0;
    float theta = M_PI / 6.0;

    init_DH_params(&params, alpha, a, d, theta);

    TEST_ASSERT_FLOAT_WITHIN(1e-5, alpha, params.alpha);
    TEST_ASSERT_FLOAT_WITHIN(1e-5, a, params.a);
    TEST_ASSERT_FLOAT_WITHIN(1e-5, d, params.d);
    TEST_ASSERT_FLOAT_WITHIN(1e-5, theta, params.theta);
}

void test_create_DH_matrix(void)
{
    DHParameters params;
    gsl_matrix *matrix = gsl_matrix_alloc(4, 4);
    gsl_matrix *expected_matrix = gsl_matrix_alloc(4, 4);

    float alpha = M_PI / 4.0;
    float a = 2.0;
    float d = 1.0;
    float theta = M_PI / 6.0;

    init_DH_params(&params, alpha, a, d, theta);
    create_DH_matrix(&params, matrix);

    gsl_matrix_set_identity(expected_matrix);
    gsl_matrix_set(expected_matrix, 0, 0, cos(theta));
    gsl_matrix_set(expected_matrix, 0, 1, -sin(theta) * cos(alpha));
    gsl_matrix_set(expected_matrix, 0, 2, sin(theta) * sin(alpha));
    gsl_matrix_set(expected_matrix, 0, 3, a * cos(theta));

    gsl_matrix_set(expected_matrix, 1, 0, sin(theta));
    gsl_matrix_set(expected_matrix, 1, 1, cos(theta) * cos(alpha));
    gsl_matrix_set(expected_matrix, 1, 2, -cos(theta) * sin(alpha));
    gsl_matrix_set(expected_matrix, 1, 3, a * sin(theta));

    gsl_matrix_set(expected_matrix, 2, 0, 0.0);
    gsl_matrix_set(expected_matrix, 2, 1, sin(alpha));
    gsl_matrix_set(expected_matrix, 2, 2, cos(alpha));
    gsl_matrix_set(expected_matrix, 2, 3, d);

    gsl_matrix_set(expected_matrix, 3, 0, 0.0);
    gsl_matrix_set(expected_matrix, 3, 1, 0.0);
    gsl_matrix_set(expected_matrix, 3, 2, 0.0);
    gsl_matrix_set(expected_matrix, 3, 3, 1.0);

    assert_matrices_equal(matrix, expected_matrix, 1e-5);

    gsl_matrix_free(matrix);
    gsl_matrix_free(expected_matrix);
}

void test_calculate_DH_transformation(void)
{
    int num_links = 2;
    DHParameters params_array[2];
    gsl_matrix *result = gsl_matrix_alloc(4, 4);
    gsl_matrix *expected_result = gsl_matrix_alloc(4, 4);

    // Initialize DH parameters for two links
    init_DH_params(&params_array[0], M_PI / 4.0, 2.0, 1.0, M_PI / 6.0);
    init_DH_params(&params_array[1], M_PI / 6.0, 1.0, 0.5, M_PI / 4.0);

    // Compute the transformation matrix
    calculate_DH_transformation(params_array, num_links, result);

    // Compute the expected result manually
    gsl_matrix_set_identity(expected_result);

    gsl_matrix *linkMatrix1 = gsl_matrix_alloc(4, 4);
    create_DH_matrix(&params_array[0], linkMatrix1);
    gsl_matrix *intermediate = gsl_matrix_alloc(4, 4);
    gsl_blas_dgemm(CblasNoTrans, CblasNoTrans, 1.0, expected_result, linkMatrix1, 0.0,
                   intermediate);
    gsl_matrix_memcpy(expected_result, intermediate);

    gsl_matrix *linkMatrix2 = gsl_matrix_alloc(4, 4);
    create_DH_matrix(&params_array[1], linkMatrix2);
    gsl_blas_dgemm(CblasNoTrans, CblasNoTrans, 1.0, expected_result, linkMatrix2, 0.0,
                   intermediate);
    gsl_matrix_memcpy(expected_result, intermediate);

    gsl_matrix_free(linkMatrix1);
    gsl_matrix_free(linkMatrix2);
    gsl_matrix_free(intermediate);

    assert_matrices_equal(result, expected_result, 1e-5);

    gsl_matrix_free(result);
    gsl_matrix_free(expected_result);
}
