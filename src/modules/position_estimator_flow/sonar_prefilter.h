#ifndef SONAR_PREFILTER_H
#define SONAR_PREFILTER_H

#include <drivers/drv_hrt.h>

float msecToSec(uint64_t t);

class SonarPrefilter {
  public:
    SonarPrefilter();
    void init(uint64_t t);
    void setParams(float min_val_, float max_val_, float threshold_, float vel_threshold_);
    bool isValid(uint64_t t, float sonar);

    static const int window_size = 20;

  private:
    // Parameters.
    float min_val;
    float max_val;
    float mean_threshold;
    float vel_threshold;
    float last_sonar;

    // State variables
    char window_index;
    float sliding_window[window_size];
    uint64_t last_time;
};

#endif