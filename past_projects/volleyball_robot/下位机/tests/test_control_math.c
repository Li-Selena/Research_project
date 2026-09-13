#include <assert.h>
#include <math.h>
#include <stdint.h>
#include <stdio.h>

extern float solution_matrix[3][3];
extern float inverse_solution_matrix[3][3];
void inverse_solution_matrix_init(void);
void solution_matrix_init(void);
void matrix_multiply(float matrix[3][3], float input[3], float output[3]);
uint8_t DeltaInverseKinematics(float x, float y, float z, float joint_deg[3]);

static void expect_near(float actual, float expected, float tolerance)
{
    assert(fabsf(actual - expected) <= tolerance);
}

int main(void)
{
    float body[3] = {123.0f, -456.0f, 78.0f};
    float wheels[3];
    float recovered[3];
    float joint_18[3];
    float joint_20[3];
    float tilted[3];

    inverse_solution_matrix_init();
    solution_matrix_init();
    matrix_multiply(inverse_solution_matrix, body, wheels);
    matrix_multiply(solution_matrix, wheels, recovered);
    for (uint8_t i = 0; i < 3; ++i) expect_near(recovered[i], body[i], 0.001f);

    assert(DeltaInverseKinematics(0.0f, 0.0f, 0.18f, joint_18));
    assert(DeltaInverseKinematics(0.0f, 0.0f, 0.20f, joint_20));
    expect_near(joint_18[0], joint_18[1], 0.001f);
    expect_near(joint_18[1], joint_18[2], 0.001f);
    assert(joint_18[0] > 4.0f && joint_18[0] < 7.0f);
    assert(joint_20[0] > joint_18[0]);
    assert(DeltaInverseKinematics(0.02f, -0.01f, 0.25f, tilted));
    assert(!DeltaInverseKinematics(0.0f, 0.0f, 0.10f, tilted));
    assert(!DeltaInverseKinematics(0.0f, 0.0f, 0.60f, tilted));
    assert(!DeltaInverseKinematics(NAN, 0.0f, 0.20f, tilted));

    puts("control math tests passed");
    return 0;
}
