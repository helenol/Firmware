#include <systemlib/err.h>
#include "math.h"

#include "flow_ekf_sep.h"

using namespace math;

float msecToSec(uint64_t t) { return t/1000000.0f; }

FlowEKFSep::FlowEKFSep()
    : g(9.81f), verbose(false)
{
}

void FlowEKFSep::init(uint64_t t, const math::Vector<N_STATES>& x, float p_covar)
{
    x_x(0) = x(0);
    x_y(0) = x(1);
    x_z(0) = x(2);
    x_x(1) = x(3);
    x_y(1) = x(4);
    x_z(1) = x(5);
    x_x(2) = x(6);
    x_y(2) = x(7);
    x_z(2) = x(8);
    x_x(3) = x(9);
    x_y(3) = x(10);
    x_z(3) = x(11);
    x_z(4) = x(12);

    kfx.init(x_x, p_covar);
    kfy.init(x_y, p_covar);
    kfz.init(x_z, p_covar);
}


void FlowEKFSep::setParams(float sigma_acc, float sigma_baro, 
                   float sigma_flow, float sigma_sonar,
                   float sigma_pos_noise, float sigma_vel_noise,
                   float sigma_acc_noise,
                   float sigma_acc_bias, float sigma_baro_bias)
{
    kfx.setParams(sigma_acc, sigma_baro, sigma_flow, sigma_sonar,
                   sigma_pos_noise, sigma_vel_noise, sigma_acc_noise,
                   sigma_acc_bias, sigma_baro_bias);
    kfy.setParams(sigma_acc, sigma_baro, sigma_flow, sigma_sonar,
                   sigma_pos_noise, 0.03, sigma_acc_noise,
                   sigma_acc_bias, sigma_baro_bias);
    kfz.setParams(sigma_acc, sigma_baro, sigma_flow, sigma_sonar,
                   sigma_pos_noise, 0.03, sigma_acc_noise,
                   sigma_acc_bias, sigma_baro_bias);
}

void FlowEKFSep::ekfStep(uint64_t t,
                 const math::Vector<N_MEASURE>& z,
                 const math::Matrix<3, 3>& rotmat,
                 bool new_sensors, bool new_flow, bool valid_sonar,
                 math::Vector<N_STATES>* x)
{
    float dt = msecToSec(t - t_prev);

    // Rotate the sensor measurements.
    acc(0) = z(0);
    acc(1) = z(1);
    acc(2) = z(2);

    acc = rotmat*acc;

    // Remove the gravity vector.
    acc(2) += g;

    flow(0) = z(4);
    flow(1) = z(5);
    flow(2) = 0;

    flow = rotmat*flow;

    Vector<3> sonar;
    sonar.zero();
    sonar(2) = z(6);
    sonar = rotmat*sonar;

    // Keep baro and sonar unrotated, because they're already in inertial frame.

    // Split in to XYZ for the single KFs.
    z_x(0) = acc(0);
    z_y(0) = acc(1);
    z_z(0) = acc(2);

    z_x(1) = flow(0);
    z_y(1) = flow(1);

    // Baro
    z_z(1) = z(3);
    // Sonar
    z_z(2) = sonar(2);

    // Call the individual KFs.
    kfx.ekfStep(dt, z_x, new_sensors, new_flow, valid_sonar, &x_x);
    kfy.ekfStep(dt, z_y, new_sensors, new_flow, valid_sonar, &x_y);
    kfz.ekfStep(dt, z_z, new_sensors, new_flow, valid_sonar, &x_z);
    
    // Translate back to full state matrix.
    (*x)(0) = x_x(0);
    (*x)(1) = x_y(0);
    (*x)(2) = x_z(0);
    (*x)(3) = x_x(1);
    (*x)(4) = x_y(1);
    (*x)(5) = x_z(1);
    (*x)(6) = x_x(2);
    (*x)(7) = x_y(2);
    (*x)(8) = x_z(2);
    (*x)(9) = x_x(3);
    (*x)(10) = x_y(3);
    (*x)(11) = x_z(3);
    (*x)(12) = x_z(4);

    // Last step!
    t_prev = t;
}

template <int states, int measures>
KalmanSingle<states, measures>::KalmanSingle()
    : verbose(false)
{
}

template <int states, int measures>
void KalmanSingle<states, measures>::init(const math::Vector<states>& x, float p_covar)
{
    x_post = x;
    P_post.identity();
    P_post = P_post*p_covar;
    eye.identity();
}

template <int states, int measures>
void KalmanSingle<states, measures>::setParams(float sigma_acc, float sigma_baro, 
                        float sigma_flow, float sigma_sonar,
                        float sigma_pos_noise, float sigma_vel_noise,
                        float sigma_acc_noise,
                        float sigma_acc_bias, float sigma_baro_bias)
{
    createQ(sigma_pos_noise, sigma_vel_noise, sigma_acc_noise, sigma_acc_bias, sigma_baro_bias);
    createR(sigma_acc, sigma_baro, sigma_flow, sigma_sonar);
}

template <int states, int measures>
void KalmanSingle<states, measures>::ekfStep(float dt,
                 const math::Vector<measures>& z,
                 bool new_sensors, bool new_flow, bool valid_sonar,
                 math::Vector<states>* x)
{
    predict(dt);
    update(dt, z, new_sensors, new_flow, valid_sonar);

    for (int i=0; i < states; i++) {
        if (isnan(x_post(i)) || isinf(x_post(i)))
            x_post(i) = 0;
    }
    *x = x_post;
}

template <int states, int measures>
void KalmanSingle<states, measures>::predict(float dt)
{
    // A priori state estimate x_k|k-1, based on previous a posteriori estimate.
    // Get x_pri, F, and Q from the state transition.
    // Q is stationary, so just F is affected.
    stateTransition(dt);
    // a priori estimate covariance.
    P_pri = F*P_post*F.transposed() + Q;
}

template <int states, int measures>
void KalmanSingle<states, measures>::update(float dt, const math::Vector<measures>& z,
                     bool new_sensors, bool new_flow, bool valid_sonar)
{
    // Measurement residual.
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
    F(0, 2) = 1/2*dt*dt;
    F(1, 2) = dt;

    x_pri = F*x_post;

    if(verbose) warnx("x_post: %.2f %.2f %.2f", x_post(0), x_post(1), x_post(2));
    if(verbose) warnx("x_pri: %.2f %.2f %.2f", x_pri(0), x_pri(1), x_pri(2));
}

void KalmanZ::stateTransition(float dt)
{
    F.identity();
    F(0, 1) = dt;
    F(0, 2) = 1/2*dt*dt;
    F(1, 2) = dt;

    x_pri = F*x_post;

    if(verbose) warnx("x_post: %.2f %.2f %.2f", x_post(0), x_post(1), x_post(2));
    if(verbose) warnx("x_pri: %.2f %.2f %.2f", x_pri(0), x_pri(1), x_pri(2));
}

void KalmanXY::predictError(float dt, const math::Vector<2>& z,
                           bool new_sensors, bool new_flow, bool valid_sonar,
                           math::Vector<2>* y_out)
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
    z_pred = H*x_pri;

    *y_out = z - z_pred;
}

void KalmanZ::predictError(float dt, const math::Vector<3>& z,
                           bool new_sensors, bool new_flow, bool valid_sonar,
                           math::Vector<3>* y_out)
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
    z_pred = H*x_pri;

    *y_out = z - z_pred;
}
