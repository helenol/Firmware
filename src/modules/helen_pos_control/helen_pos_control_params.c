#include "helen_pos_control_params.h"

PARAM_DEFINE_FLOAT(HPC_M_KG, 0.478f);
PARAM_DEFINE_FLOAT(HPC_THRUST_SCALE, 7.0f);
PARAM_DEFINE_FLOAT(HPC_KP_Z, 1.0f);
PARAM_DEFINE_FLOAT(HPC_KD_Z, 0.0f);
PARAM_DEFINE_FLOAT(HPC_KI_Z, 0.0f);
PARAM_DEFINE_FLOAT(HPC_KP_XY, 1.0f);
PARAM_DEFINE_FLOAT(HPC_KD_XY, 0.0f);
PARAM_DEFINE_FLOAT(HPC_KI_XY, 0.0f);
PARAM_DEFINE_FLOAT(HPC_F_LIM_Z, 10.0f);
PARAM_DEFINE_FLOAT(HPC_F_LIM_XY, 4.0f);

/*
int parameters_init(struct position_estimator_flow_param_handles *h)
{
    h->sigma_acc = param_find("FLOW_S_ACC");
    h->sigma_acc_bias = param_find("FLOW_S_ACC_BIAS");
    h->sigma_baro = param_find("FLOW_S_BARO");
    h->sigma_baro_bias = param_find("FLOW_S_BARO_BIAS");
    h->sigma_flow = param_find("FLOW_S_FLOW");
    h->sigma_sonar = param_find("FLOW_S_SNR");
    h->sigma_pos_noise = param_find("FLOW_S_POS_NOISE");
    h->sigma_vel_noise = param_find("FLOW_S_VEL_NOISE");
    h->sigma_acc_noise = param_find("FLOW_S_ACC_NOISE");
    h->sonar_min_value = param_find("FLOW_SNR_MIN_VAL");
    h->sonar_max_value = param_find("FLOW_SNR_MAX_VAL");
    h->sonar_mean_threshold = param_find("FLOW_SNR_MEAN_THR");
    h->sonar_vel_threshold = param_find("FLOW_SNR_VEL_THR");
    h->flow_frame_rate = param_find("FLOW_FLOW_FRAME_RATE");

    return OK;
}


int parameters_update(const struct position_estimator_flow_param_handles *h,
                      struct position_estimator_flow_params *p)
{
    param_get(h->sigma_acc, &(p->sigma_acc));
    param_get(h->sigma_acc_bias, &(p->sigma_acc_bias));
    param_get(h->sigma_baro, &(p->sigma_baro));
    param_get(h->sigma_baro_bias, &(p->sigma_baro_bias));
    param_get(h->sigma_flow, &(p->sigma_flow));
    param_get(h->sigma_sonar, &(p->sigma_sonar));
    param_get(h->sigma_pos_noise, &(p->sigma_pos_noise));
    param_get(h->sigma_vel_noise, &(p->sigma_vel_noise));
    param_get(h->sigma_acc_noise, &(p->sigma_acc_noise));
    param_get(h->sonar_min_value, &(p->sonar_min_value));
    param_get(h->sonar_max_value, &(p->sonar_max_value));
    param_get(h->sonar_mean_threshold, &(p->sonar_mean_threshold));
    param_get(h->sonar_vel_threshold, &(p->sonar_vel_threshold));
    param_get(h->flow_frame_rate, &(p->flow_frame_rate));

    return OK;
}
*/
