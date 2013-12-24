#ifndef SONAR_PREFILTER_H
#define SONAR_PREFILTER_H

#include <drivers/drv_hrt.h>


class SonarPrefilter {
  public:
    SonarPrefilter();
    void addMeasurement(uint64_t t, float sonar);
    void getLatestUpdate(uint64_t t, float* sonar, float* sigma);

  private:
    static const float msecToSec(uint64_t t);

    // Parameters.
    float high_cutoff;
    float low_cutoff;
    float sigma_sonar;
    float sigma_sonar_bad;
    float max_speed;

    // State variables.
    float last_good_val;
    float last_sigma;
    uint64_t last_time;
};

const float SonarPrefilter::msecToSec(uint64_t t) {
    return t/1000000.0;
}

void SonarPrefilter::addMeasurement(uint64_t t, float sonar) {
    // Figure out dt.
    float dt = msecToSec(t - last_time);

    float speed = (sonar - last_good_val)/dt;

    bool good = false;

    // If we're outside the range that the sonar can actually sense, throw this
    // measurement out.
    if (sonar < high_cutoff && sonar > low_cutoff) {
        good = true;
    }
    // If the change between the last data point and this one is too great,
    // discard this as a spike.
    if (fabs(speed) > max_speed)) {
        good = false;
    }

    // Update system state.
    last_time = t;
    // Apply a zero-order hold if the value is not good.
    if (!good) {
        // Keep last_good_val the same.
        last_sigma = sigma_sonar_bad;
    } else {
        last_good_val = sonar;
        last_sigma = sigma_sonar;
    }
}

void SonarPrefilter::getLatestUpdate(uint64_t t, float* sonar, float* sigma) {
    // TODO: make the sigma grow over time as we get further away from
    // measurement time.
    *sonar = last_good_val;
    *sigma = last_sigma;
}

#endif