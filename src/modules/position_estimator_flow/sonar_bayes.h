#ifndef SONAR_BAYES_H
#define SONAR_BAYES_H

#include <drivers/drv_hrt.h>
#include <mathlib/mathlib.h>


float msecToSec(uint64_t t);

class SonarBayes {
  public:
    SonarBayes();
    void init(uint64_t t);
    //void setParams(float min_val_, float max_val_);
    bool isValid(uint64_t t, float sonar);

  private:
    // Parameters.
    float min_val;
    float max_val;

    math::Matrix<4, 2> from_state_0;
    math::Matrix<4, 2> from_state_1;

    // State variables
    bool state;
    math::Vector<2> p_state;
    uint64_t last_time;
    float last_sonar;
};

#endif