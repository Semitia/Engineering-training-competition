#include <stdio.h>
#include <stdlib.h>
#include <math.h>

#define Matrix_MX 10
#define arm_MX_nodes 6
#define PI 3.1415926

/**
 * @brief 
 * 
 */
typedef struct __Matrix_t{
    int m;
    int n;
    double matrix[Matrix_MX][Matrix_MX];
}Matrix_t;

/**
 * @brief 
 * 
 */
typedef struct __robotic_arm_t{
    int n;//number of nodes
    float d[arm_MX_nodes],a[arm_MX_nodes];
    double theta[arm_MX_nodes],alpha[arm_MX_nodes];
    Matrix_t *T[arm_MX_nodes];

}robotic_arm_t;

/**
 * @brief printf the Matrix
 * @param m 
 */
void printf_matrix(Matrix_t *Mat)
{
    printf("%d x %d\r\n",Mat->m,Mat->n);
    for(int i=0; i<Mat->m; i++)
    {
        for(int j=0; j<Mat->n; j++)
        { printf("%.3f ",Mat->matrix[i][j]); }
        printf("\r\n");
    }
    printf("\r\n");
    return;
}

/**
 * @brief Matrix Mulplication
 * @param m1 
 * @param m2 
 * @return Matrix_t* 
 */
Matrix_t *multiply_matrix(Matrix_t *m1, Matrix_t *m2)
{
    if (m1->n != m2->m) {return NULL;}
    Matrix_t *ans;
    ans = (Matrix_t*)malloc(sizeof(Matrix_t));
    ans->m = m1->m;
    ans->n = m2->n;

    for(int i=0; i < m1->m; i++)
    {
        for(int j=0; j < m2->n; j++)
        {
            float sum=0;
            for(int k=0; k < m1->n; k++)
            { sum += m1->matrix[i][k]*m2->matrix[k][j]; }
            ans->matrix[i][j] = sum;
        }
    }
    return ans;
}

/**
 * @brief Matrix Mulplication
 * @param m1 
 * @param m2 
 * @return Matrix_t
 */
Matrix_t multiply_matrix2(Matrix_t *m1, Matrix_t *m2)
{
    //if (m1->n != m2->m) {return 0;}
    Matrix_t ans;
    ans.m = m1->m;
    ans.n = m2->n;

    for(int i=0; i < m1->m; i++)
    {
        for(int j=0; j < m2->n; j++)
        {
            float sum=0;
            for(int k=0; k < m1->n; k++)
            { sum += m1->matrix[i][k]*m2->matrix[k][j]; }
            ans.matrix[i][j] = sum;
        }
    }
    return ans;
}

/**
 * @brief test the multiply function.
 * 
 */
void test_multiply_matrix(void)
{
    Matrix_t *test1,*test2,*test_ans;
    test1 = (Matrix_t*)malloc(sizeof(Matrix_t));
    test2 = (Matrix_t*)malloc(sizeof(Matrix_t));
    test_ans = (Matrix_t*)malloc(sizeof(Matrix_t));
    test1->m = 2, test1->n = 3;
    test1->matrix[0][0] = 1, test1->matrix[0][1] = 2, test1->matrix[0][2] = 3;
    test1->matrix[1][0] = 2, test1->matrix[1][1] = 6, test1->matrix[1][2] = 1;
    test2->m = 3, test2->n = 4;
    test2->matrix[0][0] = 1, test2->matrix[0][1] = 1, test2->matrix[0][2] = 2, test2->matrix[0][3] = 4;
    test2->matrix[1][0] = 2, test2->matrix[1][1] = 6, test2->matrix[1][2] = 1, test2->matrix[1][3] = 3;
    test2->matrix[2][0] = 3, test2->matrix[2][1] = 1, test2->matrix[2][2] = 2, test2->matrix[2][3] = 2;

    test_ans = multiply_matrix(test1,test2);
    printf_matrix(test_ans);
    return;
}

/**
 * @brief Get the Transparent Matrix relay on the current arm state
 * @param arm 
 */
void get_Trans(robotic_arm_t *arm)
{
    for(int i=1; i<=arm->n; i++)
    {
        arm->T[i]->m=4, arm->T[i]->n=4;
        arm->T[i]->matrix[0][0] =  cos(arm->theta[i]);
        arm->T[i]->matrix[0][1] = -sin(arm->theta[i])*cos(arm->alpha[i]);
        arm->T[i]->matrix[0][2] =  sin(arm->theta[i])*sin(arm->alpha[i]);
        arm->T[i]->matrix[0][3] =  arm->a[i]*cos(arm->theta[i]);

        arm->T[i]->matrix[1][0] =  sin(arm->theta[i]);
        arm->T[i]->matrix[1][1] =  cos(arm->theta[i])*cos(arm->alpha[i]);
        arm->T[i]->matrix[1][2] = -cos(arm->theta[i])*sin(arm->alpha[i]);
        arm->T[i]->matrix[1][3] =  arm->a[i]*sin(arm->theta[i]);

        arm->T[i]->matrix[2][0] =  0;
        arm->T[i]->matrix[2][1] =  sin(arm->alpha[i]);
        arm->T[i]->matrix[2][2] =  cos(arm->alpha[i]);
        arm->T[i]->matrix[2][3] =  arm->d[i];

        arm->T[i]->matrix[3][0] =  0;
        arm->T[i]->matrix[3][1] =  0;
        arm->T[i]->matrix[3][2] =  0;
        arm->T[i]->matrix[3][3] =  1;
    }
    return;
}

/**
 * @brief 
 * @param n the number of the nodes
 * @param d D-H parameters
 * @param a D-H parameters
 * @param theta D-H parameters
 * @param alpha D-H parameters
 * @return robotic_arm_t* 
 */
robotic_arm_t *robotic_arm_init(int n, float *d, float *a, double *theta, double *alpha)
{
    robotic_arm_t *arm;
    arm = (robotic_arm_t*)malloc(sizeof(robotic_arm_t));
    for(int i=1;i<=n;i++)
    arm->T[i] = (Matrix_t*)malloc(sizeof(Matrix_t));

    arm->n = n;
    for(int i=1;i<=n;i++)
    {
        arm->d[i] = d[i];
        arm->a[i] = a[i];
        arm->alpha[i] = alpha[i];
        arm->theta[i] = theta[i];
    }

    get_Trans(arm);

    return arm;
}

/**
 * @brief get a point-matrix
 * @param x 
 * @param y 
 * @param z 
 * @return Matrix_t* 
 */
Matrix_t *point_init(double x, double y, double z)
{
    Matrix_t *point = (Matrix_t*)malloc(sizeof(Matrix_t));
    point->m = 4, point->n = 1;
    point->matrix[0][0] = x;
    point->matrix[1][0] = y;
    point->matrix[2][0] = z;
    point->matrix[3][0] = 1;
    return point;
}

/**
 * @brief Get the coordinates of the point in the end coordinate system in the main coordinate system
 *        Suppose the main coordinate system's number is 0
 *        得到末端坐标系中的一个点在主坐标系中的坐标,假设主坐标系编号为0
 * @param arm 
 * @param point 
 */
void foward_solve(robotic_arm_t *arm, Matrix_t *point)
{
    for(int i=arm->n; i>0; i--)
    *point = multiply_matrix2(arm->T[i],point);
    
    return;
}

/**
 * @brief test the function of foward solving
 */
void test_foward_solving(void)
{
    robotic_arm_t *test_arm;
    Matrix_t *test_point = point_init(2,0,1);
    float test_d[2]={0,0},test_a[2]={0,2};
    double test_theta[2]={0,PI/6},test_alpha[2]={0,0};
    test_arm = robotic_arm_init(1,test_d,test_a,test_theta,test_alpha);

    for(int i=1; i<=test_arm->n; i++)
    printf_matrix(test_arm->T[i]);

    printf_matrix(test_point);
    foward_solve(test_arm, test_point);
    printf_matrix(test_point);

    return;
}



int main()
{

    system("pause");
    return 0;
}