#ifndef __ARM_H
#define __ARM_H
#include "sys.h"

#define Matrix_MX 10
#define arm_MX_nodes 6
#define PI 3.1415926

#define N_MX_SIZE 50
#define BADDEST 9999
#define W_MAX 0.9
#define W_MIN 0.5
#define V_MAX 0.35
#define V_MIN -0.35
#define X_MAX PI
#define X_MIN 0

/**
 * @brief 
 * @param m ??
 * @param n ??
 * @param matrix ??
 */
typedef struct __Matrix_t{
    int m;
    int n;
    double matrix[Matrix_MX][Matrix_MX];
}Matrix_t;

/**
 * @brief n???,n+1????,????T1~Tn
 * @param T ????,Ti?i??????????i-1???
 */
typedef struct __robotic_arm_t{
    int n;//number of nodes
    float d[arm_MX_nodes],a[arm_MX_nodes];
    double theta[arm_MX_nodes],alpha[arm_MX_nodes];
    Matrix_t *T[arm_MX_nodes];

}robotic_arm_t;


/**
 * @brief 
 * @param X current state ????(?)
 * @param V current velocity ????
 * @param opt_X optimal solution of individual's history ???????
 * @param opt_A optimal adaptability of individual's history ?????????
 */
typedef struct __bird_t{
    double X[arm_MX_nodes], V[arm_MX_nodes];
    double opt_X[arm_MX_nodes];
    double opt_A;
}bird_t;

/**
 * @brief 
 * @param N size of the bird population ????
 * @param D ????
 * @param K ????
 * @param W ????
 * @param C_ind ??????
 * @param C_pop ??????
 * @param opt_X_pop optimal solution of population's history ???????
 * @param opt_A_pop optimal adapatability of population's history ???????
 */
typedef struct __bird_population_t{
    int N, D, K;
    double W, C_ind, C_pop;
    double opt_X_pop[arm_MX_nodes];
    double opt_A_pop;
    bird_t *bird[N_MX_SIZE];

}bird_population_t;



void printf_matrix(Matrix_t *Mat);
Matrix_t *multiply_matrix(Matrix_t *m1, Matrix_t *m2);
Matrix_t multiply_matrix2(Matrix_t *m1, Matrix_t *m2);
void test_multiply_matrix(void);
void get_Trans(robotic_arm_t *arm);
robotic_arm_t *robotic_arm_init(int n, float *d, float *a, double *theta, double *alpha);

Matrix_t *point_init(double x, double y, double z);
void foward_solve(robotic_arm_t *arm, Matrix_t *point);
void test_foward_solving(void);
void reverse_solve_dichonomy(robotic_arm_t *arm, Matrix_t point);
void test_reverse_solve_dichonomy(void);
bird_t *bird_init(bird_population_t *pop);
void bird_init_hand(bird_population_t *pop);
bird_population_t *bird_population_init(int n, int d, int k, double w, double c_ind, double c_pop);
void DH_update(robotic_arm_t *arm, double *servo);
double rad_angle(double rad);
void reverse_solve_bird(robotic_arm_t *arm, Matrix_t point);
void test_reverse_solve_bird(void);


#endif

