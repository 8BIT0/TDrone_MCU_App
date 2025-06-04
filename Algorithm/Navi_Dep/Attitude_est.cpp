#include "Attitude_est.h"

using namespace std;
using namespace Eigen;

/* internal function */
static void AttitudeEstimate_QuaternionUpdate(AttitudeObj_TypeDef *obj, float gyr_x, float gyr_y, float gyr_z);
static void AttitudeEstimate_StateEquation_Update(AttitudeObj_TypeDef *obj, float gyr_x, float gyr_y, float gyr_z);

bool AttitudeEstimate_Init(AttitudeObj_TypeDef *obj, uint16_t period)
{
    if ((obj == NULL) || (period == 0))
        return false;

    obj->delta_T = period / 1000.0f;
    obj->q << 1.0f, 0.0f, 0.0f, 0.0f;

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
    AttitudeData_TypeDef tmp;

    memset(&tmp, 0, sizeof(AttitudeData_TypeDef));
    AttitudeEstimate_QuaternionUpdate(obj, gyr_x, gyr_y, gyr_z);

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
}

static void AttitudeEstimate_StateEquation_Update(AttitudeObj_TypeDef *obj, float gyr_x, float gyr_y, float gyr_z)
{
    Matrix<float, 7, 1> X;

    if (obj == NULL)
        return;
}