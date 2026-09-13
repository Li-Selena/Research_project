#ifndef MATRIX_CALC_H
#define MATRIX_CALC_H

extern float solution_matrix[3][3];
extern float inverse_solution_matrix[3][3];
extern float rotate_matrix[3][3];
extern float inverse_transform_matrix[3][3];
extern float calculated_velocity[3];
extern float calc_buffer[3];
extern float theta;
extern float calc_buffer_2nd[3];

void inverse_solution_matrix_init(void);
void solution_matrix_init(void);
void matrix_multiply(float matrix[3][3],float organized_vector[3],float solution_vector[3]);
void rotate_matrix_calc(float theta);
void get_theta(float omega,float dt);
void vel_control(float exp_vel[3]);//更安全的底盘控制

#endif
