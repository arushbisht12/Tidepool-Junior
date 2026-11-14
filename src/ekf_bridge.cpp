#include "ekf_bridge.h"
#include <errno.h>
#include <stddef.h>
#include <math.h>

class DummyEkf {
public:
    void reset() {
        for (int i=0;i<3;i++) {
            position_[i] = 0;
        }
    }

    void step(const double accel[3], double dt_seconds) {
        for (int i=0;i<3;i++) {
            position_[i] += 0.5 * accel[i] * dt_seconds * dt_seconds;
        }
    }

    void get_position(double out[3]) const {
        for (int i=0;i<3;i++) {
            out[i] = position_[i];
        }
    }

private:
    double position_[3] = {};

};



static DummyEkf filter;
static bool initialized = false;

int ekf_init(void) {
    filter.reset();
    initialized = true;
    return 0;
}

int ekf_step(const double accel[3], 
    double dt_seconds, 
    double position_out[3]) 
{
    if (accel==NULL || position_out==NULL) {
        return -EINVAL;
    }
    if (dt_seconds <= 0) {
        return -EINVAL;
    }
    if (!initialized) {
        return -EACCES;
    }

    
    filter.step(accel, dt_seconds);
    filter.get_position(position_out);

    return 0;
}

void ekf_get(double position_out[3]) {
    filter.get_position(position_out);
}