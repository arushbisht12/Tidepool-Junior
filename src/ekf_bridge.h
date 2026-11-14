#ifndef EKF_BRIDGE_H
#define EKF_BRIDGE_H

#ifdef __cplusplus
extern "C" {
#endif

// int ekf_init(void);

int ekf_step(const double accel[3], 
    double dt_seconds, 
    double position_out[3]);

void ekf_get(double position_out[3]);

#ifdef __cplusplus
}
#endif

#endif