#ifndef POSITION_ESTIMATOR_FLOW_PARAMS_H
#define POSITION_ESTIMATOR_FLOW_PARAMS_H

#include <systemlib/param/param.h>

struct position_estimator_flow_params {
    float sigma_acc;
    float sigma_acc_bias;
    float sigma_baro;
    float sigma_baro_bias;
    float sigma_flow;
    float sigma_sonar;
    float sigma_pos_noise;
    float sigma_vel_noise;
    float sigma_acc_noise;
    float sonar_min_value;
    float sonar_max_value;
    float sonar_mean_threshold;
    float sonar_vel_threshold;
    float flow_frame_rate;
    float sonar_prefilter;
};

struct position_estimator_flow_param_handles {
    param_t sigma_acc;
    param_t sigma_acc_bias;
    param_t sigma_baro;
    param_t sigma_baro_bias;
    param_t sigma_flow;
    param_t sigma_sonar;
    param_t sigma_pos_noise;
    param_t sigma_vel_noise;
    param_t sigma_acc_noise;
    param_t sonar_min_value;
    param_t sonar_max_value;
    param_t sonar_mean_threshold;
    param_t sonar_vel_threshold;
    param_t flow_frame_rate;
    param_t sonar_prefilter;
};

/**
 * Initialize all parameter handles and values
 *
 */
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
    h->sonar_mean_threshold = param_find("FLOW_SNR_MN_THR");
    h->sonar_vel_threshold = param_find("FLOW_SNR_VEL_THR");
    h->flow_frame_rate = param_find("FLOW_FLOW_FRAME");
    h->sonar_prefilter = param_find("FLOW_SNR_PRFT");

    return OK;
}

/**
 * Update all parameters
 *
 */
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
    param_get(h->sonar_prefilter, &(p->sonar_prefilter));

    return OK;
}

#endif