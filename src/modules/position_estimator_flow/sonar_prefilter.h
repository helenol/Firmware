#ifndef SONAR_PREFILTER_H
#define SONAR_PREFILTER_H

#include <drivers/drv_hrt.h>

class SonarPrefilter {
  public:
    SonarPrefilter();
    void init(uint64_t t);
    void setParams(float min_val_, float max_val_, float threshold_, float vel_threshold_);
    bool isValid(uint64_t t, float sonar);

  private:
    static float msecToSec(uint64_t t);

    // Parameters.
    float min_val;
    float max_val;
    float threshold;
    float vel_threshold;

    // State variables
    float last_measure;
    float last_sonar;
    uint64_t last_time;
};

#endif