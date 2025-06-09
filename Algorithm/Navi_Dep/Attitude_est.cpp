#include "Attitude_est.h"
#include <math.h>

using namespace std;
using namespace Eigen;

/* internal function */
static void AttitudeEstimate_QuaternionUpdate(AttitudeObj_TypeDef *obj, float gyr_x, float gyr_y, float gyr_z);
static void AttitudeEstimate_StateEquation_Update(AttitudeObj_TypeDef *obj, float gyr_x, float gyr_y, float gyr_z);
static void AttitudeEstimate_MeasureEquation_Update(AttitudeObj_TypeDef *obj, float a_x, float a_y, float a_z, float mag_heading);
static void AttitudeEstimate_BiasCovarianceMatrix_Update(AttitudeObj_TypeDef *obj);

bool AttitudeEstimate_Init(AttitudeObj_TypeDef *obj, uint16_t period, float Q_init[7][7], float R_init[4][4], float P_init[7][7])
{
    if ((obj == NULL) || (period == 0))
        return false;

    obj->delta_T = period / 1000.0f;
    obj->q << 1.0f, 0.0f, 0.0f, 0.0f;

    obj->phi.setZero(7,7);
    obj->gyr_b.setZero(3, 1);
    obj->H.setZero(4, 7);
    obj->K.setZero(7, 4);

    for (uint8_t r = 0; r < 7; r ++)
    {
        for (uint8_t c = 0; c < 7; c ++)
        {
            if ((r < 4) && (c < 4))
                obj->R(r, c) = R_init[r][c];

            obj->P(r, c) = P_init[r][c];
            obj->Q(r, c) = Q_init[r][c];
        }
    }

    return true;
}

/*   noticed: input imu data axis direction definition
 *   body coordinate system
 *   x -> right
 *   y -> forward
 *   z -> up
 * 
 *   gyro unit: radians/s
 *   acc unit: m/s^2
 *   mag unit: G
 * 
 *   navigation coordinate system
 *   x -> east
 *   y -> north
 *   z -> up
 */
AttitudeData_TypeDef AttitudeEstimate_Update(AttitudeObj_TypeDef *obj, float gyr_x, float gyr_y, float gyr_z, float acc_x, float acc_y, float acc_z, float mag_x, float mag_y, float mag_z)
{
    float mag_heading = 0.0f;
    AttitudeData_TypeDef tmp;

    memset(&tmp, 0, sizeof(AttitudeData_TypeDef));

    /* calculate mag heading */

    AttitudeEstimate_QuaternionUpdate(obj, gyr_x, gyr_y, gyr_z);
    AttitudeEstimate_StateEquation_Update(obj, gyr_x, gyr_y, gyr_z);
    AttitudeEstimate_MeasureEquation_Update(obj, acc_x, acc_y, acc_z, mag_heading);
    AttitudeEstimate_BiasCovarianceMatrix_Update(obj);

    return tmp;
}

static void AttitudeEstimate_QuaternionUpdate(AttitudeObj_TypeDef *obj, float gyr_x, float gyr_y, float gyr_z)
{
    Matrix<float, 4, 1> q_k_1;
    Matrix<float, 4, 4> Omega;
    Matrix<float, 4, 4> I;

    if (obj == NULL)
        return;

    I.setIdentity(4, 4);
    q_k_1 = obj->q;
    Omega << 0.00f, -gyr_x, -gyr_y, -gyr_z,
             gyr_x,  0.00f,  gyr_z, -gyr_y,
             gyr_y, -gyr_z,  0.00f,  gyr_x,
             gyr_z,  gyr_y, -gyr_x,  0.00f;

    /* Runge Kutta */
    /* q(k) = {I + T/2[Ω(k - 1)]} * q(k -1) */
    obj->q = (I + (obj->delta_T / 2.0f) * Omega) * q_k_1;

    /* update X */
    obj->X << obj->q[0], obj->q[1], obj->q[2], obj->q[3], obj->gyr_b[0], obj->gyr_b[1], obj->gyr_b[2];
}

/* gyro unit: rad/s */
static void AttitudeEstimate_StateEquation_Update(AttitudeObj_TypeDef *obj, float g_x, float g_y, float g_z)
{
    Matrix<float, 3, 1> gyr_tmp;
    Matrix<float, 3, 1> gyr_raw;
    Matrix<float, 7, 7> P_tmp;
    float _TWx_2 = 0.0f;
    float _TWy_2 = 0.0f;
    float _TWz_2 = 0.0f;
    float Tq0_2  = 0.0f;
    float Tq1_2  = 0.0f;
    float Tq2_2  = 0.0f;
    float Tq3_2  = 0.0f;

    gyr_raw << g_x, g_y, g_z;
    gyr_tmp.setZero(3, 1);
    P_tmp.setZero(7, 7);

    if (obj == NULL)
        return;

    /* get measure gyro and remove bias form measurement */
    gyr_tmp = gyr_raw - obj->gyr_b;

    _TWx_2 = -(obj->delta_T * gyr_tmp[Axis_X]) / 2;
    _TWy_2 = -(obj->delta_T * gyr_tmp[Axis_Y]) / 2;
    _TWz_2 = -(obj->delta_T * gyr_tmp[Axis_Z]) / 2;

    Tq0_2 = (obj->delta_T * obj->q[0]) / 2;
    Tq1_2 = (obj->delta_T * obj->q[1]) / 2;
    Tq2_2 = (obj->delta_T * obj->q[2]) / 2;
    Tq3_2 = (obj->delta_T * obj->q[3]) / 2;

    /* calculate PHI matrix */
    obj->phi << 1, _TWx_2, _TWy_2, _TWz_2, Tq1_2, Tq2_2, Tq3_2, \
                -_TWx_2, 1, -_TWz_2, _TWy_2, -Tq0_2, Tq3_2, -Tq2_2, \
                -_TWy_2, -_TWz_2, 1, -_TWx_2, -Tq3_2, -Tq0_2, Tq1_2, \
                -_TWz_2, -_TWy_2, _TWx_2, 1, Tq2_2, -Tq1_2, -Tq0_2, \
                0, 0, 0, 0, 1, 0, 0, \
                0, 0, 0, 0, 0, 1, 0, \
                0, 0, 0, 0, 0, 0, 1;

    /* update P matrix */
    P_tmp = obj->phi * obj->P * obj->phi.transpose() + obj->Q;
    obj->P = P_tmp;
}

/* acc unit: g */
static void AttitudeEstimate_MeasureEquation_Update(AttitudeObj_TypeDef *obj, float a_x, float a_y, float a_z, float mag_heading)
{
    Matrix<float, 4, 1> z;
    Matrix<float, 4, 4> tmp;
    float D1 = 0.0f;
    float D2 = 0.0f;
    float denomiator = 0.0f;

    float M[4][7];

    if (obj == NULL)
        return;
    memset(M, 0.0f, sizeof(M));
    D1 = pow(obj->q[0], 2) - pow(obj->q[1], 2) + pow(obj->q[2], 2) - pow(obj->q[2], 2);
    D2 = obj->q[1] * obj->q[2] - obj->q[0] * obj->q[3];

    M[0][0] = (2 * obj->q[2]);
    M[0][1] = (-2 * obj->q[3]);
    M[0][2] = (2 * obj->q[0]);
    M[0][3] = (-2 * obj->q[1]);

    M[1][0] = (-2 * obj->q[1]);
    M[1][1] = (-2 * obj->q[0]);
    M[1][2] = (-2 * obj->q[3]);
    M[1][3] = (-2 * obj->q[2]);

    M[2][0] = (-2 * obj->q[0]);
    M[2][1] = (2 * obj->q[1]);
    M[2][2] = (2 * obj->q[2]);
    M[2][3] = (2 * obj->q[3]);

    denomiator = pow(D1, 2) + 4 * pow(D2, 2);
    M[3][0] = (2 * obj->q[3] * D1 + 4 * obj->q[0] * D2) / denomiator;
    M[3][1] = (-2 * obj->q[3] * D1 - 4 * obj->q[0] * D2) / denomiator;
    M[3][2] = (-2 * obj->q[3] * D1 + 4 * obj->q[0] * D2) / denomiator;
    M[3][3] = (2 * obj->q[3] * D1 - 4 * obj->q[0] * D2) / denomiator;

    obj->H << M[0][0], M[0][1], M[0][2], M[0][3], M[0][4], M[0][5], M[0][6], \
              M[1][0], M[1][1], M[1][2], M[1][3], M[1][4], M[1][5], M[1][6], \
              M[2][0], M[2][1], M[2][2], M[2][3], M[2][4], M[2][5], M[2][6], \
              M[3][0], M[3][1], M[3][2], M[3][3], M[3][4], M[3][5], M[3][6];

    tmp = obj->H * obj->P * obj->H.transpose() + obj->R;
    obj->K = obj->P * obj->H.transpose() * tmp.inverse();

    /* get estimate value */
    /* x^(k) = x^(k, k - 1) + K(k)(z(k) - h(x^(k, k - 1), k)) */
}

static void AttitudeEstimate_BiasCovarianceMatrix_Update(AttitudeObj_TypeDef *obj)
{
    Matrix<float, 7, 7> tmp;
    Matrix<float, 7, 7> P_tmp;
    Matrix<float, 7, 7> I;

    I.setIdentity();
    tmp.setZero(7, 7);
    P_tmp.setZero(7, 7);

    if (obj == NULL)
        return;

    /* P(k) = (I - K(k) * H(k)) * P(k, k - 1) * (I - K(k) * H(k)).T + K(k) * R(k) * K.T(k) */
    tmp = (I - obj->K * obj->H);
    P_tmp = tmp * obj->P * tmp.transpose() + obj->K * obj->R * obj->K.transpose();
    obj->P = P_tmp;
}
