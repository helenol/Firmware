#include <math.h>

#include "sonar_bayes.h"

SonarBayes::SonarBayes()
    : min_val(0.301f), max_val(3.0f), last_time(0), last_sonar(0) {
}

void SonarBayes::init(uint64_t t) {
    last_time = t;

    // Also create the matrices.
    from_state_0(0, 0) = 0.90f;
    from_state_0(1, 0) = 0.50f;
    from_state_0(2, 0) = 0.40f;
    from_state_0(3, 0) = 0.30f;
    from_state_0(0, 1) = 1 - from_state_0(0, 0);
    from_state_0(1, 1) = 1 - from_state_0(1, 0);
    from_state_0(2, 1) = 1 - from_state_0(2, 0);
    from_state_0(3, 1) = 1 - from_state_0(3, 0);

    from_state_1(0, 0) = 0.05f;
    from_state_1(1, 0) = 0.5f;
    from_state_1(2, 0) = 0.8f;
    from_state_1(3, 0) = 0.95f;
    from_state_1(0, 1) = 1 - from_state_1(0, 0);
    from_state_1(1, 1) = 1 - from_state_1(1, 0);
    from_state_1(2, 1) = 1 - from_state_1(2, 0);
    from_state_1(3, 1) = 1 - from_state_1(3, 0);

    // Fill in current state.
    state = 0;
    p_state(0) = 1.0f;
    p_state(1) = 0.0f;
}

/*void SonarBayes::setParams(float min_val_, float max_val_)
{
    min_val = min_val_;
    max_val = max_val_;
}*/

bool SonarBayes::isValid(uint64_t t, float sonar) {
    // Figure out dt.
    float dt = msecToSec(t - last_time);

    float diff = fabs(sonar - last_sonar);

    // If this is the same value as before, and if it's been less than 100 ms
    // from the last measurement, discard this reading.
    if (dt < 0.1 && diff < 0.01) {
        return false;
    }

    // If we're outside the range that the sonar can actually sense, throw this
    // measurement out.
    if (sonar <= min_val || sonar > max_val) {
        return false;
    }

    char seq = 0;

    if (diff < 0.05f) {
        seq = 0;
    } else if (diff < 0.2f) {
        seq = 1;
    } else if (diff < 0.5f) {
        seq = 2;
    } else {
        seq = 3;
    }

    float prob_trans_0 = from_state_0(seq, 0)*p_state(0) +
            from_state_1(seq, 0)*p_state(1);
    float prob_trans_1 = from_state_0(seq, 1)*p_state(0) +
            from_state_1(seq, 1)*p_state(1);

    if (prob_trans_0 > prob_trans_1) {
        state = 0;
    } else {
        state = 1;
    }

    p_state(0) = prob_trans_0;
    p_state(1) = prob_trans_1;

    last_time = t;
    last_sonar = sonar;

    return state;
}