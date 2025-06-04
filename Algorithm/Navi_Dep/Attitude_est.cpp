#include "Attitude_est.h"

using namespace std;
using namespace Eigen;

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
 *   navigation coordinate system
 *   x -> east
 *   y -> north
 *   z -> up
 */
AttitudeData_TypeDef AttitudeEstimate_Update(AttitudeObj_TypeDef *obj, float gyr_x, float gyr_y, float gyr_z, float acc_x, float acc_y, float acc_z, float mag_x, float mag_y, float mag_z)
{
    AttitudeData_TypeDef tmp;

    memset(&tmp, 0, sizeof(AttitudeData_TypeDef));

    return tmp;
}

static void AttitudeEstimate_QuaternionUpdate(AttitudeObj_TypeDef *obj)
{
    Matrix<float, 4, 1> q_k_1;

    if (obj == NULL)
        return;
}
