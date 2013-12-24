#ifndef SONAR_EKF_H
#define SONAR_EKF_H

#include <mathlib/mathlib.h>

class SonarEKF {
  public:
    SonarEKF();
    void init(float sigma_acc, float sigma_acc_bias, float sigma_baro, 
              float sigma_baro_bias, float process_noise_pos,
              const math::Vector& x, const math::Vector& P);

    void ekfStep(uint64_t t, const math::Vector& z_obs, float sonar_sigma, 
                 math::Vector* x_pred);

  private:
    static const float msecToSec(uint64_t t);

    void predict(uint64_t t);
    // Since the sensor sigmas change, we actually construct Q each time.
    void update(const math::Vector& z_obs, float sonar_sigma);

    void stateTransition(const math::Vector& x, float u, float dt, 
                         math::Vector* x_out);
    void predictMeasurements(const math::Vector& x);

    // Parameters
    math::Matrix F;
    math::Matrix H;
    math::Matrix Q;
    math::Matrix R;

    // State variables
    math::Vector x_pri;
    math::Matrix P_pri;
    math::Vector x_post;
    math::Vector P_post;

    // Preallocate this memory.
    math::Vector z_pred;
    math::Vector y;
    math::Matrix S;
    math::Matrix K;
    // Identity matrix the size of the states.
    math::Matrix eye_states;

    uint64_t last_time;
};


SonarEKF::SonarEKF()
    : F(5, 5), H(3, 5), Q(5, 5), R(3, 3), x_pri(5), P_pri(5, 5), 
      x_post(5), P_post(5, 5), z_pred(3), y(3), S(3, 3), K(5, 5), 
      eye_states(5, 5) {
    // Do I need to zero these matrices?
    F.setAll(0);
    H.setAll(0);
    Q.setAll(0);
    R.setAll(0);
    x_pri.setAll(0);
    P_pri.setAll(0);
    x_post.setAll(0);
    P_post.setAll(0);
    z_pred.setAll(0);
    y.setAll(0);
    K.setAll(0);
    eye_states.setAll(0);
}

const float SonarPrefilter::msecToSec(uint64_t t) {
    return t/1000000.0;
}

void SonarEKF::init(float sigma_acc, float sigma_acc_bias, float sigma_baro, 
                    float sigma_baro_bias, float process_noise_pos,
                    const math::Vector& x, const math::Vector& P) {
    // Preallocate all the stuff that's going to have to be resized each time.
    // F depends on dt, so can't pre-make just once. :(
    // But let's set all the diagonal elements.
    // F is the state transition matrix.
    F(1, 1) = 1;
    F(2, 2) = 1;
    F(3, 3) = 1;
    F(4, 4) = 1;
    F(5, 5) = 1;

    // H is the state observation matrix. Row 1 is accel, row 2 is baro
    // row 3 is sonar.
    H(1, 3) = 1;
    H(1, 4) = 1;
    H(2, 1) = 1;
    H(2, 5) = 1;
    H(3, 1) = 1;

    // Q is the process noise matrix.
    Q(1, 1) = process_noise_pos;
    Q(4, 4) = sigma_acc_bias*sigma_acc_bias;
    Q(5, 5) = sigma_baro_bias*sigma_baro_bias;

    // R is measurement noise covariance.
    R(1, 1) = sigma_acc*sigma_acc;
    R(2, 2) = sigma_baro*sigma_baro;
    // Sonar sigma is fed into the loop at every iteration.

    // Fill in eye_states (I(num_states))
    eye_states(1, 1) = 1;
    eye_states(2, 2) = 1;
    eye_states(3, 3) = 1;
    eye_states(4, 4) = 1;
    eye_states(5, 5) = 1;

    // Assign the initial states.
    x_post = x;
    P_post = P;
}

void SonarEKF::ekfStep(uint64_t t, const math::Vector& z_obs, float sonar_sigma, 
                       math::Vector* x_pred) {
    // Update the prior state from posteriors.
    predict(t);

    // Update the filter with our new observations.
    update(z_obs, sonar_sigma);

    // Copy the result to the output.
    *x_pred = x_post;
}

void SonarEKF::predict(uint64_t t) {
    float u = 0;
    float dt = msecToSec(t - last_time);

    // Fill in F matrix.
    F(1, 2) = dt;
    F(1, 3) = dt*dt/2.0;
    F(2, 3) = dt;

    state_transition(x_post, u, dt, &x_pri);
    P_pri = F*P_post*F.transpose() + Q;

    last_time = t;
}

void SonarEKF::update(const math::Vector& z_obs, float sonar_sigma) {
    // Fill in the relevant bits of the R matrix (sonar_sigma).
    R(3, 3) = sonar_sigma*sonar_sigma;

    predictMeasurements(x_pri, &z_pred);
    y = z_obs - z_pred;
    S = H*P_pri*H.transpose() + R;
    K = P_pri*H.transpose()*S.inverse();
    x_post = x_pri + K*y;
    P_post = (eye_states - K*H)*P_pri;
}


void SonarEKF::stateTransition(const math::Vector& x, float u, float dt, 
                                math::Vector* x_out) {
    *x_out = F*x;
}

void SonarEKF::predictMeasurements(const math::Vector& x, math::Vector* z) {
    *z = H*x;

    // TODO: also fix z(3) for ranges outside of 0.3, 4.
}

#endif