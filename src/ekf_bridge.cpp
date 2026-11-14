#include "ekf_bridge.h"
#include <errno.h>
#include <stddef.h>
#include <math.h>
#include "MEKF.h"

// class DummyEkf {
// public:
//     void reset() {
//         for (int i=0;i<3;i++) {
//             position_[i] = 0;
//         }
//     }

//     void step(const double accel[3], double dt_seconds) {
//         for (int i=0;i<3;i++) {
//             position_[i] += 0.5 * accel[i] * dt_seconds * dt_seconds;
//         }
//     }

//     void get_position(double out[3]) const {
//         for (int i=0;i<3;i++) {
//             out[i] = position_[i];
//         }
//     }

// private:
//     double position_[3] = {};

// };

mekf::Vector4d initial_q{1.0, 0.0, 0.0, 0.0};
mekf::Vector3d initial_vel{0.0, 0.0, 0.0};
mekf::Vector3d initial_pos{0.0, 0.0, 0.0};
double gyro_cov = 0.0;
double accel_cov = 0.0;
double gyro_bias_cov = 0.0;
double accel_bias_cov = 0.0;

static mekf::MEKF filter{initial_q, initial_vel, initial_pos, gyro_cov, accel_cov, gyro_bias_cov, accel_bias_cov};
// static bool initialized = false;

// int ekf_init(void) {
//     filter.reset();
//     initialized = true;
//     return 0;
// }

int ekf_step(const double accel[3], 
    double dt_seconds, 
    double position_out[3]) 
{
    if (position_out==NULL) {
        return -EINVAL;
    }
    if (dt_seconds <= 0) {
        return -EINVAL;
    }
    // if (!initialized) {
    //     return -EACCES;
    // }

    mekf::Vector3d accel_vector;
    for (int i=0;i<3;i++) {
        accel_vector[i] = accel[i];
    }
    
    filter.prediction(dt_seconds, nullptr, &accel_vector);
    for (int i=0;i<3;i++) {
        position_out[i] = filter.pos()[i];
    }

    return 0;
}

void ekf_get(double position_out[3]) {
    for (int i=0;i<3;i++) {
        position_out[i] = filter.pos()[i];
    }
}