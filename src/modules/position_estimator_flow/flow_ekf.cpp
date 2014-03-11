#include "math.h"

#include "flow_ekf.h"

using namespace math;

FlowEKF::FlowEKF()
    : g(9.81)
{
}

void FlowEKF::init(uint64_t t, const math::Vector<N_STATES>& x, 
              const math::Matrix<N_STATES, N_STATES>& P)
{
    t_prev = t;
    x_post = x;
    P_post = P;
}

void FlowEKF::setParams(float sigma_acc, float sigma_baro, 
                        float sigma_flow, float sigma_sonar,
                        float sigma_pos_noise, float sigma_vel_noise,
                        float sigma_acc_bias, float sigma_baro_bias)
{
    createQ(sigma_pos_noise, sigma_vel_noise, sigma_acc_bias, sigma_baro_bias);
    createR(sigma_acc, sigma_baro, sigma_flow, sigma_sonar);
}

void FlowEKF::ekfStep(uint64_t t,
                 const math::Vector<N_MEASURE>& z,
                 const math::Vector<N_CONTROL>& u,
                 bool new_sensors, bool new_flow, bool valid_sonar,
                 math::Vector<N_STATES>* x)
{
    float dt = msecToSec(t - t_prev);

    // First, figure out what the rotation matrix should be from the controls.
    // I think the rest just need thrust and rotation matrix...
    createRotmat(u);
    predictAccel(u(0));

    predict(dt);

    update(dt, z, new_sensors, new_flow, valid_sonar);

    // Last step!
    t_prev = t;
    *x = x_post;
}

void FlowEKF::createRotmat(const math::Vector<N_CONTROL>& u)
{
    float pitch = u(0);
    float roll = u(1);
    float yaw = u(2);

    rotmat(0, 0) = cos(pitch)*cos(yaw);
    rotmat(0, 1) = cos(pitch)*sin(yaw);
    rotmat(0, 2) = -sin(pitch);
    rotmat(1, 0) = sin(roll)*sin(pitch)*cos(yaw) - cos(roll)*sin(yaw);
    rotmat(1, 1) = sin(roll)*sin(pitch)*sin(yaw) + cos(roll)*cos(yaw);
    rotmat(1, 2) = sin(roll)*cos(pitch);
    rotmat(2, 0) = cos(roll)*sin(pitch)*cos(yaw) + sin(roll)*sin(yaw);
    rotmat(2, 1) = cos(roll)*sin(pitch)*sin(yaw) - sin(roll)*cos(yaw);
    rotmat(2, 2) = cos(roll)*cos(pitch);

    // TODO: check that all of these transposes are correct.
    rotmat = rotmat.transposed();
}

void FlowEKF::predictAccel(float thrust)
{
    // Predict the acceleration vector.
    // This only needs to be done once, and then have a flag for 'on ground'.
    rot_g.zero();
    rot_g(2) = g;
    rot_g = rotmat.transposed()*rot_g;
    // Assume thrust is already in metric units - scaled by mass and so on.
    acc_pred.zero();
    acc_pred(2) = -thrust;
    acc_pred = acc_pred + rot_g;

    if (x_post(4) >= 0 && acc_pred(2) > 0)
    {
        acc_pred.zero();
        on_ground = true;
    } else {
        on_ground = false;
    }
}

void FlowEKF::predict(float dt)
{
    // A priori state estimate x_k|k-1, based on previous a posteriori estimate.
    // Get x_pri, F, and Q from the state transition.
    // Q is stationary, so just F is affected.
    stateTransition(dt);
    // a priori estimate covariance.
    P_pri = F*P_post*F.transposed() + Q;
}

void FlowEKF::update(float dt, const math::Vector<N_MEASURE>& z,
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

void FlowEKF::createQ(float sigma_pos_noise, float sigma_vel_noise,
                      float sigma_acc_bias, float sigma_baro_bias)
{
    // TODO: all indices - 1
    Q.zero();
    Q(0, 0) = sigma_pos_noise*sigma_pos_noise;
    Q(2, 2) = Q(0, 0);
    Q(4, 4) = Q(0, 0);

    Q(1, 1) = sigma_vel_noise*sigma_vel_noise;
    Q(3, 3) = Q(1, 1);
    Q(5, 5) = Q(1, 1);

    Q(6, 6) = sigma_acc_bias*sigma_acc_bias;
    Q(7, 7) = Q(6, 6);
    Q(8, 8) = Q(6, 6);

    Q(9, 9) = sigma_baro_bias*sigma_baro_bias;

}

void FlowEKF::createR(float sigma_acc, float sigma_baro, 
                      float sigma_flow, float sigma_sonar)
{
    // TODO: all indices - 1
    R.zero();
    R(0, 0) = sigma_acc*sigma_acc;
    R(1, 1) = R(0, 0);
    R(2, 2) = R(0, 0);
    R(3, 3) = sigma_baro*sigma_baro;
    R(4, 4) = sigma_flow*sigma_flow;
    R(5, 5) = sigma_flow*sigma_flow;
    R(6, 6) = sigma_sonar*sigma_sonar;
}

void FlowEKF::stateTransition(float dt)
{
    // TODO: all indices - 1
    // TODO: double check that this is still correct.
    F.identity();

    // Rotated velocity
    Matrix<3, 3> dt_rot = rotmat * dt;

    F(0, 1) = dt_rot(0, 0);
    F(0, 3) = dt_rot(0, 1);
    F(0, 5) = dt_rot(0, 2);
    F(2, 1) = dt_rot(1, 0);
    F(2, 3) = dt_rot(1, 1);
    F(2, 5) = dt_rot(1, 2);
    F(4, 1) = dt_rot(2, 0);
    F(4, 3) = dt_rot(2, 1);
    F(4, 5) = dt_rot(2, 2);

    if (on_ground)
    {
        F(4, 4) = 0;
        F(5, 5) = 0;
    }

    x_pri = F*x_post;
    x_pri(0) = x_pri(0) + 1/2*acc_pred(0)*dt*dt;
    x_pri(1) = x_pri(1) + acc_pred(0)*dt;
    x_pri(2) = x_pri(2) + 1/2*acc_pred(1)*dt*dt;
    x_pri(3) = x_pri(3) + acc_pred(1)*dt;
    x_pri(4) = x_pri(4) + 1/2*acc_pred(2)*dt*dt;
    x_pri(5) = x_pri(5) + acc_pred(2)*dt;    

    if (on_ground)
    {
        x_pri(0) = x_post(0);
        x_pri(1) = 0;
        x_pri(2) = x_post(2);
        x_pri(3) = 0;
        x_pri(4) = 0;
        x_pri(5) = 0;
    }
}

void FlowEKF::predictError(float dt, const math::Vector<N_MEASURE>& z,
                           bool new_sensors, bool new_flow, bool valid_sonar,
                           math::Vector<N_MEASURE>* y)
{
    // TODO: all indices - 1    
    // State observation matrix.
    H.zero();
    if (new_sensors)
    {
        // Accelerometer - x
        H(0, 0) = 1/2*dt*dt;
        H(0, 1) = dt;
        H(0, 6) = 1;
        // Accelerometer - y
        H(1, 2) = 1/2*dt*dt;
        H(1, 3) = dt;
        H(1, 7) = 1;
        // Accelerometer - z
        H(2, 4) = 1/2*dt*dt;
        H(2, 5) = dt;
        H(2, 8) = 1;
        // Baro - z
        H(3, 4) = -1;
        H(3, 9) = -1;
    }
    if (new_flow)
    {
        // Flow - x
        H(4, 1) = 1;
        // Flow - y
        H(5, 3) = 1;
        // Sonar - z
        H(6, 4) = -valid_sonar;
    }

    // If we're on the ground, then predict nothing in terms of accel..
    if (on_ground)
    {
        // Throw away sonar too because it's probably garbage?
        H(6, 4) = 0;
    }

    // Compute predicted values from state.
    Vector<N_MEASURE> z_pred = H*x_pri;
    z_pred(0) = acc_pred(0) - rot_g(0);
    z_pred(1) = acc_pred(1) - rot_g(1);
    z_pred(2) = acc_pred(2) - rot_g(2);

    *y = z - z_pred;
}
