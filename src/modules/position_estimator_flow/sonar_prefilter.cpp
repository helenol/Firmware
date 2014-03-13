#include "math.h"

#include "sonar_prefilter.h"

SonarPrefilter::SonarPrefilter()
    : min_val(0.301), max_val(2.6), mean_threshold(0.5), var_threshold(0.2),
      last_time(0) {
    for (int i = 0; i < window_size; i++) {
        sliding_window[i] = 0;
    }
}

void SonarPrefilter::init(uint64_t t) {
    // TODO: make this function assign all the other parameters as well.
    last_time = t;
}

void SonarPrefilter::setParams(float min_val_, float max_val_, 
                               float mean_threshold_, float var_threshold_)
{
    min_val = min_val_;
    max_val = max_val_;
    mean_threshold = mean_threshold_;
    var_threshold = var_threshold_;
}

float SonarPrefilter::msecToSec(uint64_t t) {
    return t/1000000.0f;
}

bool SonarPrefilter::isValid(uint64_t t, float sonar) {
    // Figure out dt.
    //float dt = msecToSec(t - last_time);

    //float vel = (sonar - last_sonar)/dt;

    bool is_valid = false;

    // If we're outside the range that the sonar can actually sense, throw this
    // measurement out.
    if (sonar > min_val && sonar < max_val) {
        is_valid = true;
    }

    float mean = 0;
    float var = 0;
    // Calculate mean and variance of the last window.
    for (int i=0; i < window_size; i++) {
        mean += 1.0/window_size*window[i];
        var += 1.0/window_size*(window[i]*window[i]);
    }

    var -= mean*mean;

    if (fabs(sonar - mean) > mean_threshold) {
        is_valid = false;
    }
    if (var > var_threshold) {
        is_valid = false;
    }
    
    /*// If the change between the last data point and this one is too great,
    // discard this as a spike.
    if (fabs(vel) > vel_threshold) {
        is_valid = false;
    }
    if (fabs(sonar - sliding_window) > threshold) {
        is_valid = false;
    }

    // Update system state.
    if (is_valid) {
        sliding_window = sonar;
    }
    last_sonar = sonar; */

    // Record the next measurement in the sliding window.
    if (window_index >= window_size) {
        window_index = 0;
    }
    sliding_window[window_index] = sonar;
    window_index++;

    last_time = t;
    return is_valid;
}