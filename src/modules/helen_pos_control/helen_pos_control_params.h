#ifndef HELEN_POS_CONTROL_PARAMS_H
#define HELEN_POS_CONTROL_PARAMS_H

#include <systemlib/param/param.h>

struct helen_pos_control_params {
    float m;
    float thrust_scale;
    float k_p_gain_z;
    float k_d_gain_z;
    float k_i_gain_z;
    float k_p_gain_xy;
    float k_d_gain_xy;
    float k_i_gain_xy;
    float f_lim_xy;
    float f_lim_z;
};

struct helen_pos_control_param_handles {
    param_t m;
    param_t thrust_scale;
    param_t k_p_gain_z;
    param_t k_d_gain_z;
    param_t k_i_gain_z;
    param_t k_p_gain_xy;
    param_t k_d_gain_xy;
    param_t k_i_gain_xy;
    param_t f_lim_xy;
    param_t f_lim_z;
};

/**
 * Initialize all parameter handles and values
 *
 */
int parameters_init(struct helen_pos_control_param_handles *h)
{
    h->m = param_find("HPC_M_KG");
    h->thrust_scale = param_find("HPC_THRUST_SCALE");
    h->k_p_gain_z = param_find("HPC_KP_Z");
    h->k_d_gain_z = param_find("HPC_KD_Z");
    h->k_i_gain_z = param_find("HPC_KI_Z");
    h->k_p_gain_xy = param_find("HPC_KP_XY");
    h->k_d_gain_xy = param_find("HPC_KD_XY");
    h->k_i_gain_xy = param_find("HPC_KI_XY");
    h->f_lim_xy = param_find("HPC_F_LIM_XY");
    h->f_lim_z = param_find("HPC_F_LIM_Z");

    return OK;
}

/**
 * Update all parameters
 *
 */
int parameters_update(const struct helen_pos_control_param_handles *h,
                      struct helen_pos_control_params *p)
{
    param_get(h->m, &(p->m));
    param_get(h->thrust_scale, &(p->thrust_scale));
    param_get(h->k_p_gain_z, &(p->k_p_gain_z));
    param_get(h->k_d_gain_z, &(p->k_d_gain_z));
    param_get(h->k_p_gain_xy, &(p->k_p_gain_xy));
    param_get(h->k_d_gain_xy, &(p->k_d_gain_xy));
    param_get(h->k_i_gain_z, &(p->k_i_gain_z));
    param_get(h->k_i_gain_xy, &(p->k_i_gain_xy));
    param_get(h->f_lim_xy, &(p->f_lim_xy));
    param_get(h->f_lim_z, &(p->f_lim_z));

    return OK;
}

#endif