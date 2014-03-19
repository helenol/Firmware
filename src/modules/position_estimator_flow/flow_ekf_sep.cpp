#include <systemlib/err.h>
#include "math.h"

#include "flow_ekf_sep.h"

using namespace math;

FlowEKF::FlowEKF()
    : g(9.81f), verbose(false)
{
}

void FlowEKF::setParams(float sigma_acc, float sigma_baro, 
                   float sigma_flow, float sigma_sonar,
                   float sigma_pos_noise, float sigma_vel_noise,
                   float sigma_acc_noise,
                   float sigma_acc_bias, float sigma_baro_bias)
{
    kfxy.setParams(sigma_acc, sigma_baro, sigma_flow, sigma_sonar,
                   sigma_pos_noise, sigma_vel_noise, sigma_acc_noise,
                   sigma_acc_bias, sigma_baro_bias);
    kfz.setParams(sigma_acc, sigma_baro, sigma_flow, sigma_sonar,
                   sigma_pos_noise, sigma_vel_noise, sigma_acc_noise,
                   sigma_acc_bias, sigma_baro_bias);
}

void FlowEKF::ekfStep(uint64_t t,
                 const math::Vector<N_MEASURE>& z,
                 const math::Matrix<3, 3>& rotmat,
                 bool new_sensors, bool new_flow, bool valid_sonar,
                 math::Vector<N_STATES>* x)
{
    float dt = msecToSec(t - t_prev);

    // Rotate the sensor measurements.
    
    // Split in to XYZ for the single KFs.


    // Call the individual KFs.

    for (int i=0; i < N_STATES; i++) {
        if (isnan(x_post(i)) || isinf(x_post(i)))
            x_post(i) = 0;
    }

    // Last step!
    t_prev = t;
    *x = x_post;
}

KalmanSingle::KalmanSingle()
    : verbose(false)
{
}

void KalmanSingle::init(uint64_t t, const math::Vector<states>& x, float p_covar)
{
    t_prev = t;
    x_post = x;
    P_post.identity();
    P_post = P_post*p_covar;
}

void KalmanSingle::setParams(float sigma_acc, float sigma_baro, 
                        float sigma_flow, float sigma_sonar,
                        float sigma_pos_noise, float sigma_vel_noise,
                        float sigma_acc_noise,
                        float sigma_acc_bias, float sigma_baro_bias)
{
    createQ(sigma_pos_noise, sigma_vel_noise, sigma_acc_noise, sigma_acc_bias, sigma_baro_bias);
    createR(sigma_acc, sigma_baro, sigma_flow, sigma_sonar);
}

void KalmanSingle::ekfStep(float dt,
                 const math::Vector<N_MEASURE>& z,
                 bool new_sensors, bool new_flow, bool valid_sonar,
                 math::Vector<N_STATES>* x)
{
    predict(dt);
    update(dt, z, new_sensors, new_flow, valid_sonar);

    for (int i=0; i < N_STATES; i++) {
        if (isnan(x_post(i)) || isinf(x_post(i)))
            x_post(i) = 0;
    }

    *x = x_post;
}


void KalmanSingle::predict(float dt)
{
    // A priori state estimate x_k|k-1, based on previous a posteriori estimate.
    // Get x_pri, F, and Q from the state transition.
    // Q is stationary, so just F is affected.
    stateTransition(dt);
    // a priori estimate covariance.
    P_pri = F*P_post*F.transposed() + Q;
}

void KalmanSingle::update(float dt, const math::Vector<N_MEASURE>& z,
                     bool new_sensors, bool new_flow, bool valid_sonar)
{
    // Measurement residual.
    Vector<N_MEASURE> y;
    Matrix<N_STATES, N_STATES> eye;
    eye.identity();
    // Fill in y, H, R; R is stationary so just H.
    predictError(dt, z, new_sensors, new_flow, valid_sonar, &y);
    S = H*P_pri*H.transposed() + R;
    K = P_pri*H.transposed()*(S.inversed());
    x_post = x_pri + K*y;
    P_post = (eye - K*H)*P_pri;
}

void KalmanXY::createQ(float sigma_pos_noise, float sigma_vel_noise,
                      float sigma_acc_noise, 
                      float sigma_acc_bias, float sigma_baro_bias)
{
    Q.zero();
    Q(0, 0) = sigma_pos_noise*sigma_pos_noise;
    Q(1, 1) = sigma_vel_noise*sigma_vel_noise;
    Q(2, 2) = sigma_acc_noise*sigma_acc_noise;
    Q(3, 3) = sigma_acc_bias*sigma_acc_bias;
}

void KalmanZ::createQ(float sigma_pos_noise, float sigma_vel_noise,
                      float sigma_acc_noise, 
                      float sigma_acc_bias, float sigma_baro_bias)
{
    Q.zero();
    Q(0, 0) = sigma_pos_noise*sigma_pos_noise;
    Q(1, 1) = sigma_vel_noise*sigma_vel_noise;
    Q(2, 2) = sigma_acc_noise*sigma_acc_noise;
    Q(3, 3) = sigma_acc_bias*sigma_acc_bias;
    Q(4, 4) = sigma_baro_bias*sigma_baro_bias;
}

void KalmanXY::createR(float sigma_acc, float sigma_baro, 
                      float sigma_flow, float sigma_sonar)
{
    R.zero();
    R(0, 0) = sigma_acc*sigma_acc;
    R(1, 1) = sigma_flow*sigma_flow;
}

void KalmanZ::createR(float sigma_acc, float sigma_baro, 
                      float sigma_flow, float sigma_sonar)
{
    R.zero();
    R(0, 0) = sigma_acc*sigma_acc;
    R(1, 1) = sigma_baro*sigma_baro;
    R(2, 2) = sigma_sonar*sigma_sonar;
}

void KalmanXY::stateTransition(float dt)
{
    F.identity();
    F(0, 1) = dt;
    F(0, 2) = 1/2*dt^2;
    F(1, 2) = dt;

    x_pri = F*x_post;

    if(verbose) warnx("x_post: %.2f %.2f %.2f", x_post(0), x_post(1), x_post(2));
    if(verbose) warnx("x_pri: %.2f %.2f %.2f", x_pri(0), x_pri(1), x_pri(2));
}

void KalmanZ::stateTransition(float dt)
{
    F.identity();
    F(0, 1) = dt;
    F(0, 2) = 1/2*dt^2;
    F(1, 2) = dt;

    x_pri = F*x_post;

    if(verbose) warnx("x_post: %.2f %.2f %.2f", x_post(0), x_post(1), x_post(2));
    if(verbose) warnx("x_pri: %.2f %.2f %.2f", x_pri(0), x_pri(1), x_pri(2));
}

void KalmanXY::predictError(float dt, const math::Vector<N_MEASURE>& z,
                           bool new_sensors, bool new_flow, bool valid_sonar,
                           math::Vector<N_MEASURE>* y)
{
    // State observation matrix.
    H.zero();
    if (new_sensors)
    {
        // Accelerometer
        H(0, 2) = 1;
        H(0, 3) = 1;
    }
    if (new_flow)
    {
        // Flow 
        H(1, 1) = 1;
    }

    // Compute predicted values from state.
    Vector<N_MEASURE> z_pred = H*x_pri;

    *y = z - z_pred;
}

void KalmanXY::predictError(float dt, const math::Vector<N_MEASURE>& z,
                           bool new_sensors, bool new_flow, bool valid_sonar,
                           math::Vector<N_MEASURE>* y)
{
    // State observation matrix.
    H.zero();
    if (new_sensors)
    {
        // Accelerometer
        H(0, 2) = 1;
        H(0, 3) = 1;
        // Baro - z
        H(1, 0) = -1;
        H(1, 4) = -1;
    }
    if (new_flow)
    {
        // Sonar - z
        H(2, 0) = -((int)valid_sonar);
    }

    // Compute predicted values from state.
    Vector<N_MEASURE> z_pred = H*x_pri;

    *y = z - z_pred;
}
