#ifndef FLOW_EKF_H
#define FLOW_EKF_H

#include <mathlib/mathlib.h>

#define N_STATES 13
#define N_MEASURE 7
#define N_CONTROL 3

class FlowEKF {
  public:
    FlowEKF();

    void init(uint64_t t, const math::Vector<N_STATES>& x, 
              const math::Matrix<N_STATES, N_STATES>& P);

    void setParams(float sigma_acc, float sigma_baro, 
                   float sigma_flow, float sigma_sonar,
                   float sigma_pos_noise, float sigma_vel_noise,
                   float sigma_acc_noise,
                   float sigma_acc_bias, float sigma_baro_bias);

    void ekfStep(uint64_t t,
                 const math::Vector<N_MEASURE>& z,
                 const math::Vector<N_CONTROL>& u,
                 bool new_sensors, bool new_flow, bool valid_sonar,
                 math::Vector<N_STATES>* x_post);

    bool onGround() { return on_ground; }

  protected:
    static float msecToSec(uint64_t t) { return t/1000000.0f; }
    
    void predict(float dt);
    void update(float dt, const math::Vector<N_MEASURE>& z,
                bool new_sensors, bool new_flow, bool valid_sonar);

    void stateTransition(float dt);
    void predictError(float dt, const math::Vector<N_MEASURE>& z,
                      bool new_sensors, bool new_flow, bool valid_sonar,
                      math::Vector<N_MEASURE>* y);

    void createQ(float sigma_pos_noise, float sigma_vel_noise,\
                 float sigma_acc_noise, 
                 float sigma_acc_bias, float sigma_baro_bias);
    void createR(float sigma_acc, float sigma_baro, 
                 float sigma_flow, float sigma_sonar);

    void createRotmat(const math::Vector<N_CONTROL>& u);
    void predictAccel(float thrust);

    // Parameters
    float g; // Gravity, m/s^2

    // States from last iteration.
    uint64_t t_prev;
    math::Vector<N_STATES> x_pri;
    math::Matrix<N_STATES, N_STATES> P_pri;
    math::Vector<N_STATES> x_post;
    math::Matrix<N_STATES, N_STATES> P_post;

    // Derived values from inputs for this iteration.
    bool on_ground;
    math::Matrix<3, 3> rotmat;  // body -> world. transpose is world -> body.
    math::Vector<3> rot_g; // Rotated gravity vector.
    math::Vector<3> acc_pred; // Predictd acceleration from inputs.

    // EKF matrices.
    // F and H are re-generated every iteration, because they depend on dt or
    // the availability of sensor data or ground contact.
    math::Matrix<N_STATES, N_STATES> F;
    math::Matrix<N_MEASURE, N_STATES> H;
    // Q and R are only updated with parameters.
    math::Matrix<N_STATES, N_STATES> Q;
    math::Matrix<N_MEASURE, N_MEASURE> R;
    // K and S are kalman gains.
    math::Matrix<N_STATES, N_MEASURE> K;
    math::Matrix<N_MEASURE, N_MEASURE> S;

    bool verbose;
};

#endif