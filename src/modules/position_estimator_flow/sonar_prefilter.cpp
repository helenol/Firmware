#include "math.h"

#include "sonar_prefilter.h"

SonarPrefilter::SonarPrefilter()
    : min_val(0.301), max_val(2.6), threshold(0.5), vel_threshold(5),
      last_measure(0.0f), last_sonar(0.0f), last_time(0) {
}

void SonarPrefilter::init(uint64_t t) {
    // TODO: make this function assign all the other parameters as well.
    last_time = t;
}

void SonarPrefilter::setParams(float min_val_, float max_val_, 
                               float threshold_, float vel_threshold_)
{
    min_val = min_val_;
    max_val = max_val_;
    threshold = threshold_;
    vel_threshold = vel_threshold_;
}

float SonarPrefilter::msecToSec(uint64_t t) {
    return t/1000000.0f;
}

bool SonarPrefilter::isValid(uint64_t t, float sonar) {
    // Figure out dt.
    float dt = msecToSec(t - last_time);

    float vel = (sonar - last_sonar)/dt;

    bool is_valid = false;

    // If we're outside the range that the sonar can actually sense, throw this
    // measurement out.
    if (sonar > min_val && sonar < max_val) {
        is_valid = true;
    }
    // If the change between the last data point and this one is too great,
    // discard this as a spike.
    if (fabs(vel) > vel_threshold) {
        is_valid = false;
    }
    if (fabs(sonar - last_measure) > threshold) {
        is_valid = false;
    }

    // Update system state.
    if (is_valid) {
        last_measure = sonar;
    }
    last_sonar = sonar;
    last_time = t;
    return is_valid;
}