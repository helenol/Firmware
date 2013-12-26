#ifndef SONAR_EKF_H
#define SONAR_EKF_H

//#define MATRIX_ASSERT
//#define VECTOR_ASSERT

#include <mathlib/mathlib.h>

class SonarEKF {
  public:
    SonarEKF();
    void init(float sigma_acc, float sigma_acc_bias, float sigma_baro, 
              float sigma_baro_bias, float process_noise_pos,
              const math::Vector& x, const math::Matrix& P, uint64_t t);

    void ekfStep(uint64_t t, const math::Vector& z_obs, float sonar_sigma, 
                 math::Vector* x_pred);

  private:
    static const float msecToSec(uint64_t t);

    void predict(uint64_t t);
    // Since the sensor sigmas change, we actually construct Q each time.
    void update(const math::Vector& z_obs, float sonar_sigma);

    void stateTransition(const math::Vector& x, float u, float dt, 
                         math::Vector* x_out);
    void predictMeasurements(const math::Vector& x, math::Vector* z);

    // Parameters
    math::Matrix F;
    math::Matrix H;
    math::Matrix Q;
    math::Matrix R;

    // State variables
    math::Vector x_pri;
    math::Matrix P_pri;
    math::Vector x_post;
    math::Matrix P_post;

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
      x_post(5), P_post(5, 5), z_pred(3), y(3), S(3, 3), K(5, 3), 
      eye_states(5, 5), last_time(0) {
    // Do I need to zero these matrices?
    F.setAll(0.0f);
    H.setAll(0.0f);
    Q.setAll(0.0f);
    R.setAll(0.0f);
    x_pri.setAll(0.0f);
    P_pri.setAll(0.0f);
    x_post.setAll(0.0f);
    P_post.setAll(0.0f);
    z_pred.setAll(0.0f);
    y.setAll(0.0f);
    K.setAll(0.0f);
    eye_states.setAll(0.0f);
}

const float SonarEKF::msecToSec(uint64_t t) {
    return t/1000000.0f;
}

void SonarEKF::init(float sigma_acc, float sigma_acc_bias, float sigma_baro, 
                    float sigma_baro_bias, float process_noise_pos,
                    const math::Vector& x, const math::Matrix& P, 
                    uint64_t t) {
    // Preallocate all the stuff that's going to have to be resized each time.
    // F depends on dt, so can't pre-make just once. :(
    // But let's set all the diagonal elements.
    // F is the state transition matrix.
    F(0, 0) = 1.0f;
    F(1, 1) = 1.0f;
    F(2, 2) = 1.0f;
    F(3, 3) = 1.0f;
    F(4, 4) = 1.0f;

    // H is the state observation matrix. Row 0 is accel, row 1 is baro
    // row 2 is sonar.
    H(0, 2) = 1.0f;
    H(0, 3) = 1.0f;
    H(1, 0) = 1.0f;
    H(1, 4) = 1.0f;
    H(2, 0) = 1.0f;

    // Q is the process noise matrix.
    Q(0, 0) = process_noise_pos;
    Q(3, 3) = sigma_acc_bias*sigma_acc_bias;
    Q(4, 4) = sigma_baro_bias*sigma_baro_bias;

    // R is measurement noise covariance.
    R(0, 0) = sigma_acc*sigma_acc;
    R(1, 1) = sigma_baro*sigma_baro;
    // Sonar sigma is fed into the loop at every iteration.

    // Fill in eye_states (I(num_states))
    eye_states(0, 0) = 1.0f;
    eye_states(1, 1) = 1.0f;
    eye_states(2, 2) = 1.0f;
    eye_states(3, 3) = 1.0f;
    eye_states(4, 4) = 1.0f;

    // Assign the initial states.
    x_post = x;
    P_post = P;
    last_time = t;
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
    F(0, 1) = dt;
    F(0, 2) = dt*dt/2.0f;
    F(1, 2) = dt;

    stateTransition(x_post, u, dt, &x_pri);
    P_pri = F*P_post*F.transpose() + Q;

    last_time = t;
}

void SonarEKF::update(const math::Vector& z_obs, float sonar_sigma) {
    // Fill in the relevant bits of the R matrix (sonar_sigma).
    R(2, 2) = sonar_sigma*sonar_sigma;

    predictMeasurements(x_pri, &z_pred);
    y = z_obs - z_pred;
    //printf("y: \n");
    //y.print();
    S = H*P_pri*H.transpose() + R;
    K = P_pri*H.transpose()*S.inverse();
    x_post = x_pri + K*y;
    //printf("x_pri: \n");
    //x_pri.print();
    //printf("K*y: \n");
    //(K*y).print();
    //printf("xpost: \n");
    //x_post.print();
    P_post = (eye_states - K*H)*P_pri;
}


void SonarEKF::stateTransition(const math::Vector& x, float u, float dt, 
                                math::Vector* x_out) {
    x_pri = F*x;
    //*x_out = F*x;
}

void SonarEKF::predictMeasurements(const math::Vector& x, math::Vector* z) {
    z_pred = H*x;
    //*z = H*x;

    // TODO: also fix z(3) for ranges outside of 0.3, 4.
}

#endif