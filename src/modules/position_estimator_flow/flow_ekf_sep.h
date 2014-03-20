#ifndef FLOW_EKF_SEP_H
#define FLOW_EKF_SEP_H

#include <mathlib/mathlib.h>

#define N_STATES 13
#define N_MEASURE 7

float msecToSec(uint64_t t);

template <int states, int measures>
class KalmanSingle
{
  public:
    KalmanSingle();

    void init(const math::Vector<states>& x, float p_covar);

    void setParams(float sigma_acc, float sigma_baro, 
                   float sigma_flow, float sigma_sonar,
                   float sigma_pos_noise, float sigma_vel_noise,
                   float sigma_acc_noise,
                   float sigma_acc_bias, float sigma_baro_bias);
    void ekfStep(float dt,
                 const math::Vector<measures>& z,
                 bool new_sensors, bool new_flow, bool valid_sonar,
                 math::Vector<states>* x);
  protected:
    void predict(float dt);
    void update(float dt, const math::Vector<measures>& z,
                bool new_sensors, bool new_flow, bool valid_sonar);

    virtual void stateTransition(float dt) {}
    virtual void predictError(float dt, const math::Vector<measures>& z,
                      bool new_sensors, bool new_flow, bool valid_sonar,
                      math::Vector<measures>* y) {}

    virtual void createQ(float sigma_pos_noise, float sigma_vel_noise,
                 float sigma_acc_noise,
                 float sigma_acc_bias, float sigma_baro_bias) {}
    virtual void createR(float sigma_acc, float sigma_baro, 
                 float sigma_flow, float sigma_sonar) {}

    // States from last iteration.
    math::Vector<states> x_pri;
    math::Matrix<states, states> P_pri;
    math::Vector<states> x_post;
    math::Matrix<states, states> P_post;

    // EKF matrices.
    // F and H are re-generated every iteration, because they depend on dt or
    // the availability of sensor data or ground contact.
    math::Matrix<states, states> F;
    math::Matrix<measures, states> H;
    // Q and R are only updated with parameters.
    math::Matrix<states, states> Q;
    math::Matrix<measures, measures> R;
    // K and S are kalman gains.
    math::Matrix<states, measures> K;
    math::Matrix<measures, measures> S;

    // Temporary variables.
    math::Vector<measures> z_pred;
    math::Vector<measures> y;
    math::Matrix<states, states> eye;

    bool verbose;
};

class KalmanXY : public KalmanSingle<4, 2>
{
  protected:
    virtual void stateTransition(float dt);
    virtual void predictError(float dt, const math::Vector<2>& z,
                      bool new_sensors, bool new_flow, bool valid_sonar,
                      math::Vector<2>* y);

    virtual void createQ(float sigma_pos_noise, float sigma_vel_noise,
                 float sigma_acc_noise,
                 float sigma_acc_bias, float sigma_baro_bias);
    virtual void createR(float sigma_acc, float sigma_baro, 
                 float sigma_flow, float sigma_sonar);
};

class KalmanZ : public KalmanSingle<5, 3>
{
  protected:
    virtual void stateTransition(float dt);
    virtual void predictError(float dt, const math::Vector<3>& z,
                      bool new_sensors, bool new_flow, bool valid_sonar,
                      math::Vector<3>* y);

    virtual void createQ(float sigma_pos_noise, float sigma_vel_noise,
                 float sigma_acc_noise,
                 float sigma_acc_bias, float sigma_baro_bias);
    virtual void createR(float sigma_acc, float sigma_baro, 
                 float sigma_flow, float sigma_sonar);
};

class FlowEKFSep {
  public:
    FlowEKFSep();

    void init(uint64_t t, const math::Vector<N_STATES>& x, float p_covar);

    void setParams(float sigma_acc, float sigma_baro, 
                   float sigma_flow, float sigma_sonar,
                   float sigma_pos_noise, float sigma_vel_noise,
                   float sigma_acc_noise,
                   float sigma_acc_bias, float sigma_baro_bias);

    void ekfStep(uint64_t t,
                 const math::Vector<N_MEASURE>& z,
                 const math::Matrix<3, 3>& rotmat,
                 bool new_sensors, bool new_flow, bool valid_sonar,
                 math::Vector<N_STATES>* x);

  protected:
    // Parameters
    float g; // Gravity, m/s^2

    // States from last iteration.
    uint64_t t_prev;

    // Derived values from inputs for this iteration.
    //math::Matrix<3, 3> rotmat;  // body -> world. transpose is world -> body.
    //math::Vector<3> rot_g; // Rotated gravity vector.
    //math::Vector<3> acc_pred; // Predictd acceleration from inputs.

    KalmanXY kfx;
    KalmanXY kfy;
    KalmanZ kfz;

    // Temporary variables.
    math::Vector<3> acc;
    math::Vector<3> flow;
    math::Vector<2> z_x;
    math::Vector<2> z_y;
    math::Vector<3> z_z;
    math::Vector<4> x_x;
    math::Vector<4> x_y;
    math::Vector<5> x_z;

    bool verbose;
};



#endif