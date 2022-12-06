#include "arm.h"

double servo_angle[arm_MX_nodes];

/**
 * @brief printf the Matrix
 * @param m 
**/
void printf_matrix(Matrix_t *Mat)
{
	u8 i,j;
    printf("%d x %d\r\n",Mat->m,Mat->n);
    for(i=0; i<Mat->m; i++)
    {
        for(j=0; j<Mat->n; j++)
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
	u8 i,j,k;
    Matrix_t *ans;
    ans = (Matrix_t*)malloc(sizeof(Matrix_t));
    ans->m = m1->m;
    ans->n = m2->n;
    if (m1->n != m2->m) {return NULL;}
	
    for(i=0; i < m1->m; i++)
    {
        for(j=0; j < m2->n; j++)
        {
            float sum=0;
            for(k=0; k < m1->n; k++)
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
	u8 i,j,k;
    Matrix_t ans;
    ans.m = m1->m;
    ans.n = m2->n;

    for(i=0; i < m1->m; i++)
    {
        for(j=0; j < m2->n; j++)
        {
            float sum=0;
            for(k=0; k < m1->n; k++)
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
	u8 i;
    for(i=1; i<=arm->n; i++)
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
	u8 i;
    robotic_arm_t *arm;
    arm = (robotic_arm_t*)malloc(sizeof(robotic_arm_t));
    for(i=1;i<=n;i++)
    arm->T[i] = (Matrix_t*)malloc(sizeof(Matrix_t));

    arm->n = n;
    for(i=1;i<=n;i++)
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
 *        ?????????????????????,?????????0
 * @param arm 
 * @param point 
 */
void foward_solve(robotic_arm_t *arm, Matrix_t *point)
{
	u8 i;
    for(i=arm->n; i>0; i--)
    *point = multiply_matrix2(arm->T[i],point);
    
    return;
}

/**
 * @brief test the function of foward solving
 */
void test_foward_solving(void)
{
	u8 i;
    robotic_arm_t *test_arm;
    Matrix_t *test_point = point_init(2,0,1);
    float test_d[2]={0,0},test_a[2]={0,2};
    double test_theta[2]={0,PI/6},test_alpha[2]={0,0};
    test_arm = robotic_arm_init(1,test_d,test_a,test_theta,test_alpha);

    for(i=1; i<=test_arm->n; i++)
    printf_matrix(test_arm->T[i]);

    printf_matrix(test_point);
    foward_solve(test_arm, test_point);
    printf_matrix(test_point);

    return;
}

/**
 * @brief Use the dichotomy method to find the angle of each joint according to the target coordinates 
 *        ?????????????????(??????,?????????)
 *        ????????????????????(???????),???????????
 * @param arm ??????,???
 * @param point ?????????
 */
void reverse_solve_dichonomy(robotic_arm_t *arm, Matrix_t point)
{
    double angle0, angle1, angle2, alpha_l=0, alpha_r=PI, alpha_mid, beta, theta, len_xy, x, y, z, l1, l2, d;
    x = point.matrix[0][0], y = point.matrix[1][0], z = point.matrix[2][0] - arm->d[1];
    d = sqrt(x*x + y*y + z*z);
    l1 = arm->a[2], l2 = arm->a[3];
    len_xy = sqrt(x*x + y*y);
    if(len_xy == 0) {theta = PI/2;}
    else {theta = atan(z/len_xy);}

    while(alpha_r-alpha_l > 0.00175)
    {
        alpha_mid = (alpha_l+alpha_r)/2;
        beta = asin(sin(alpha_mid)*l1/l2);
        if(l1*cos(alpha_mid) + l2*cos(beta) > d)
        {alpha_l = alpha_mid;}
        else 
        {alpha_r = alpha_mid;}
    }
    if(x == 0) {angle0 = PI/2;}
    else {angle0 = atan(y/x);}
    angle1 = theta - alpha_mid;
    angle2 = alpha_mid + beta + PI/2;
    arm->theta[1] =  angle0;
    arm->theta[2] = -angle1;
    arm->theta[3] = PI/2 - angle2;

    return;
}

/**
 * @brief test the function of reverse solve with dichotomy method 
 */
void test_reverse_solve_dichonomy(void)
{
    robotic_arm_t *test_arm;
    Matrix_t *test_point = point_init(8.66,0,16), *test_point2 = point_init(0,0,0);
    float test_d[4]={0,1,0,0},test_a[4]={0,0,10,10};
    double test_theta[4]={0},test_alpha[4]={0,-PI/2,0,0};
    test_arm = robotic_arm_init(3,test_d,test_a,test_theta,test_alpha);


    reverse_solve_dichonomy(test_arm,*test_point);
    get_Trans(test_arm);

    printf_matrix(test_point2);
    foward_solve(test_arm, test_point2);
    printf_matrix(test_point2);

    free(test_arm);
    free(test_point);
    return;
}


/**
 * @brief ???????
 * ??????????????????????
 * @param pop 
 * @return bird_t* 
 */
bird_t *bird_init(bird_population_t *pop)
{
    int i;
    bird_t *bird;
    bird = (bird_t*)malloc(sizeof(bird_t));
    for(i=0; i<pop->D; i++)
    {
        bird->X[i] = PI*(rand()%180)/180;
        bird->V[i] = PI*(rand()%20-10)/180;
    }
    bird->opt_A = BADDEST;

    return bird;
}


/**
 * @brief ??????
 * @param n 
 * @param d 
 * @param k 
 * @param w 
 * @param c_ind 
 * @param c_pop 
 * @return bird_population_t* 
 */
bird_population_t *bird_population_init(int n, int d, int k, double w, double c_ind, double c_pop)
{
	u8 i;
    bird_population_t *bird_population;
    bird_population = (bird_population_t*)malloc(sizeof(bird_population_t));

    bird_population->N = n, bird_population->D = d, bird_population->K = k;
    bird_population->W = w, bird_population->C_ind = c_ind, bird_population->C_pop = c_pop;

    //srand((unsigned)time( NULL ) );     
    for(i=0; i<n; i++)
    {
        bird_population->bird[i] = bird_init(bird_population);
    }
    bird_population->opt_A_pop = BADDEST;
    return bird_population;
}

/**
 * @brief ????????DH??,???????PWM????DH?????
 * @param arm 
 * @param servo ??(??)?? 
 */
void DH_update(robotic_arm_t *arm, double *servo)
{
    arm->theta[1] =  servo[0];
    arm->theta[2] = -servo[1];
    arm->theta[3] =  PI/2-servo[2];
    get_Trans(arm);
    return;
}

/**
 * @brief ?????????
 * @param rad 
 * @return double 
 */
double rad_angle(double rad)
{
    return rad*180/PI;
}

/**
 * @brief ?? Use the PSO algorithm to find the angle of each joint according to the target coordinates 
 * @param arm 
 * @param point 
 */
void reverse_solve_bird(robotic_arm_t *arm, Matrix_t point)
{
    u8 i,j,k,m;
    double w = W_MAX;
    bird_population_t *bird_pop;
    bird_pop = bird_population_init(20,arm->n,20,0.8,1.6,1.8);
    for(i=0; i<bird_pop->K; i++)//K
    {
        for(j=0;j<bird_pop->N;j++)//N
        {
            //?????
            Matrix_t *judge_point = point_init(0,0,0);
            double ada=0;//???
            DH_update(arm,bird_pop->bird[j]->X);
            foward_solve(arm,judge_point);
            ada += (point.matrix[0][0] - judge_point->matrix[0][0])*(point.matrix[0][0] - judge_point->matrix[0][0]);
            ada += (point.matrix[1][0] - judge_point->matrix[1][0])*(point.matrix[1][0] - judge_point->matrix[1][0]);
            ada += (point.matrix[2][0] - judge_point->matrix[2][0])*(point.matrix[2][0] - judge_point->matrix[2][0]);
            
            //????????????????
            if(ada < bird_pop->bird[j]->opt_A)
            {
                bird_pop->bird[j]->opt_A = ada;
                for(k=0; k < arm->n; k++)
                {bird_pop->bird[j]->opt_X[k] = bird_pop->bird[j]->X[k];}
            }
            if(ada < bird_pop->opt_A_pop)
            {
                bird_pop->opt_A_pop = ada;
                for(k=0; k < arm->n; k++)
                {bird_pop->opt_X_pop[k] = bird_pop->bird[j]->X[k];}
            }
            
            //??w?,??????????
            w = W_MAX - (W_MAX - W_MIN)*((double)i/bird_pop->K);
            for(k=0; k < arm->n; k++)
            {
                bird_pop->bird[j]->V[k] = (w+(rand()/16384-1)*0.15)*(bird_pop->bird[j]->V[k]) + (rand()/16384)*1.8*(bird_pop->opt_X_pop[k] - bird_pop->bird[j]->X[k]) + (rand()/16384)*1.6*(bird_pop->bird[j]->opt_X[k] - bird_pop->bird[j]->X[k]);
                bird_pop->bird[j]->V[k] = (bird_pop->bird[j]->V[k] > V_MAX) ? V_MAX : bird_pop->bird[j]->V[k];
                bird_pop->bird[j]->V[k] = (bird_pop->bird[j]->V[k] < V_MIN) ? V_MIN : bird_pop->bird[j]->V[k];
                bird_pop->bird[j]->X[k] += bird_pop->bird[j]->V[k]; 
                bird_pop->bird[j]->X[k] = (bird_pop->bird[j]->X[k] > X_MAX) ? X_MAX : bird_pop->bird[j]->X[k];
                bird_pop->bird[j]->X[k] = (bird_pop->bird[j]->X[k] < X_MIN) ? X_MIN : bird_pop->bird[j]->X[k];
            }
            
            //????
            printf("    bird%d: \r\n        ????X: ",j);
            for(m=0; m<arm->n; m++) {printf("%.3f|%.1f, ",bird_pop->bird[j]->X[m],rad_angle(bird_pop->bird[j]->X[m]));}
            printf("\r\n        ????V: ");
            for(m=0; m<arm->n; m++) {printf("%.3f|%.1f, ",bird_pop->bird[j]->V[m],rad_angle(bird_pop->bird[j]->V[m]));}
            printf("\r\n");
            free(judge_point);
        }
    //????
    printf("%d ???: %.3f ??w: %.3f\r\n",i,bird_pop->opt_A_pop,w);
    printf("   ??X: ");
    for(k=0; k < arm->n; k++) {printf("%.2f|%.1f ",bird_pop->opt_X_pop[k],rad_angle(bird_pop->opt_X_pop[k]));}
    printf("\r\n");

    }
    //?????????????
    DH_update(arm,bird_pop->opt_X_pop);
    return;
}

/**
 * @brief test the function of PSO algorithm
 */
void test_reverse_solve_bird(void)
{
	u8 k;
    robotic_arm_t *test_arm;
    Matrix_t *target_point = point_init(8.66,0,16), *test_point = point_init(0,0,0);
    float test_d[4]={0,1,0,0},test_a[4]={0,0,10,10};
    double test_theta[4]={0},test_alpha[4]={0,-PI/2,0,0};
    test_arm = robotic_arm_init(3,test_d,test_a,test_theta,test_alpha);

    //reverse_solve_bird(test_arm, *target_point);
    reverse_solve_bird(test_arm, *target_point);
    for(k=1; k <= test_arm->n; k++)
    {printf("theta: %.3f ",test_arm->theta[k]);}
    printf("\r\n");
    foward_solve(test_arm,test_point);
    printf_matrix(test_point);

    free(test_arm);
    free(test_point);
    free(target_point);
    return;
}



