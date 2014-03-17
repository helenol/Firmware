#include <systemlib/err.h>
#include "math.h"

#include "flow_ekf.h"

using namespace math;

FlowEKF::FlowEKF()
    : g(9.81), verbose(false)
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
                        float sigma_acc_noise,
                        float sigma_acc_bias, float sigma_baro_bias)
{
    createQ(sigma_pos_noise, sigma_vel_noise, sigma_acc_noise, sigma_acc_bias, sigma_baro_bias);
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
    predictAccel(0.0f);

    if(verbose) warnx("dt: %.2f, rotmat diag: %.2f %.2f %.2f; %.2f %.2f %.2f; %.2f %.2f %.2f", dt,
        rotmat(0, 0), rotmat(0, 1), rotmat(0, 2),
        rotmat(1, 0), rotmat(1, 1), rotmat(1, 2),
        rotmat(2, 0), rotmat(2, 1), rotmat(2, 2));

    predict(dt);

    update(dt, z, new_sensors, new_flow, valid_sonar);

    for (int i=0; i < N_STATES; i++) {
        if (isnan(x_post(i)) || isinf(x_post(i)))
            x_post(i) = 0;
    }

    // Last step!
    t_prev = t;
    *x = x_post;
}

void FlowEKF::createRotmat(const math::Vector<N_CONTROL>& u)
{
    float roll = u(0);
    float pitch = u(1);
    float yaw = u(2);

    rotmat.from_euler(roll, pitch, yaw);

    // Verify that roll is correct... seems wrong. :(
    /*rotmat(0, 0) = cosf(pitch)*cosf(yaw);
    rotmat(0, 1) = cosf(pitch)*sinf(yaw);
    rotmat(0, 2) = -sinf(pitch);
    rotmat(1, 0) = sinf(roll)*sinf(pitch)*cosf(yaw) - cosf(roll)*sinf(yaw);
    rotmat(1, 1) = sinf(roll)*sinf(pitch)*sinf(yaw) + cosf(roll)*cosf(yaw);
    rotmat(1, 2) = sinf(roll)*cosf(pitch);
    rotmat(2, 0) = cosf(roll)*sinf(pitch)*cosf(yaw) + sinf(roll)*sinf(yaw);
    rotmat(2, 1) = cosf(roll)*sinf(pitch)*sinf(yaw) - sinf(roll)*cosf(yaw);
    rotmat(2, 2) = cosf(roll)*cosf(pitch);*/

    // TODO: check that all of these transposes are correct.
    // Looks like it was not correct...
    //rotmat = rotmat.transposed();
}

void FlowEKF::predictAccel(float thrust)
{
    // Predict the acceleration vector.
    // This only needs to be done once, and then have a flag for 'on ground'.
    rot_g.zero();
    rot_g(2) = 9.81f;
    rot_g = rotmat.transposed()*rot_g;

    if (verbose) warnx("rotg: %.2f %.2f %.2f", rot_g(0), rot_g(1), rot_g(2));

    if (x_post(2) >= 0) {
        on_ground = true;
    }


    // Assume thrust is already in metric units - scaled by mass and so on.
    /*acc_pred.zero();
    acc_pred(2) = -thrust;
    acc_pred = acc_pred + rot_g;

    if (x_post(4) >= 0 && acc_pred(2) > 0)
    {
        acc_pred.zero();
        on_ground = true;
    } else {
        on_ground = false;
    } */
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
                      float sigma_acc_noise, 
                      float sigma_acc_bias, float sigma_baro_bias)
{
    Q.zero();
    Q(0, 0) = sigma_pos_noise*sigma_pos_noise;
    Q(1, 1) = Q(0, 0);
    Q(2, 2) = Q(0, 0);

    Q(3, 3) = sigma_vel_noise*sigma_vel_noise;
    Q(4, 4) = Q(3, 3);
    Q(5, 5) = Q(3, 3);

    Q(6, 6) = sigma_acc_noise*sigma_acc_noise;
    Q(7, 7) = Q(6, 6);
    Q(8, 8) = Q(6, 6);

    Q(9, 9) = sigma_acc_bias*sigma_acc_bias;
    Q(10, 10) = Q(6, 6);
    Q(11, 11) = Q(6, 6);

    Q(12, 12) = sigma_baro_bias*sigma_baro_bias;

}

void FlowEKF::createR(float sigma_acc, float sigma_baro, 
                      float sigma_flow, float sigma_sonar)
{
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
    // TODO: double check that this is still correct.
    F.identity();

    // Rotated velocity
    Matrix<3, 3> dt_rot = rotmat * dt;

    // velocity -> pos.
    F(0, 3) = dt_rot(0, 0);
    F(0, 4) = dt_rot(0, 1);
    F(0, 5) = dt_rot(0, 2);
    F(1, 3) = dt_rot(1, 0);
    F(1, 4) = dt_rot(1, 1);
    F(1, 5) = dt_rot(1, 2);
    F(2, 3) = dt_rot(2, 0);
    F(2, 4) = dt_rot(2, 1);
    F(2, 5) = dt_rot(2, 2);

    // acceleration -> pos
    F(0, 6) = dt_rot(0, 0)*dt*0.5;
    F(0, 7) = dt_rot(0, 1)*dt*0.5;
    F(0, 8) = dt_rot(0, 2)*dt*0.5;
    F(1, 6) = dt_rot(1, 0)*dt*0.5;
    F(1, 7) = dt_rot(1, 1)*dt*0.5;
    F(1, 8) = dt_rot(1, 2)*dt*0.5;
    F(2, 6) = dt_rot(2, 0)*dt*0.5;
    F(2, 7) = dt_rot(2, 1)*dt*0.5;
    F(2, 8) = dt_rot(2, 2)*dt*0.5;

    // accel -> velocity
    F(3, 6) = dt;
    F(4, 7) = dt;
    F(5, 8) = dt;

    x_pri = F*x_post;

    if(verbose) warnx("x_post: %.2f %.2f %.2f", x_post(0), x_post(1), x_post(2));
    if(verbose) warnx("x_pri: %.2f %.2f %.2f", x_pri(0), x_pri(1), x_pri(2));

    //if (on_ground)
    //{
        // Hey a ground constraint maybe.
    //    x_pri(2) = 0;
    //    x_pri(5) = 0;
    //    x_pri(8) = 0;
    //}
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
        H(0, 6) = 1;
        H(0, 9) = 1;
        // Accelerometer - y
        H(1, 7) = 1;
        H(1, 10) = 1;
        // Accelerometer - z
        H(2, 8) = 1;
        H(2, 11) = 1;
        // Baro - z
        H(3, 2) = -1;
        H(3, 12) = -1;
    }
    if (new_flow)
    {
        // Flow - x
        H(4, 3) = 1;
        // Flow - y
        H(5, 4) = 1;
        // Sonar - z
        H(6, 2) = -((int)valid_sonar);
    }

    // If we're on the ground, then predict nothing in terms of accel..
    //if (on_ground)
    //{
        // Throw away sonar too because it's probably garbage?
    //    H(6, 4) = 0;
    //}

    // Compute predicted values from state.
    Vector<N_MEASURE> z_pred = H*x_pri;
    z_pred(0) -= rot_g(0);
    z_pred(1) -= rot_g(1);
    z_pred(2) -= rot_g(2);

    *y = z - z_pred;

    if(verbose) warnx("y: a[%.2f %.2f %.2f] b[%.2f] f[%.2f %.2f] s[%.2f]",
          (*y)(0), (*y)(1), (*y)(2), (*y)(3), (*y)(4), (*y)(5),
          (*y)(6)); 
    if(verbose) warnx("z: a[%.2f %.2f %.2f] b[%.2f] f[%.2f %.2f] s[%.2f]",
          z(0), z(1), z(2), z(3), z(4), z(5), z(6)); 
    if(verbose) warnx("p: a[%.2f %.2f %.2f] b[%.2f] f[%.2f %.2f] s[%.2f]",
          z_pred(0), z_pred(1), z_pred(2), z_pred(3), z_pred(4), z_pred(5), z_pred(6)); 
    if(verbose) warnx("valid: a,b: %i, f: %i s: %i", new_sensors, new_flow, valid_sonar);
}
