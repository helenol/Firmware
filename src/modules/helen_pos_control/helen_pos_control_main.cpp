#include <drivers/drv_hrt.h>
#include <errno.h>
#include <fcntl.h>
#include <float.h>
#include <limits.h>
#include <math.h>
#include <mathlib/mathlib.h>
#include <mavlink/mavlink_log.h>
#include <nuttx/config.h>
#include <nuttx/sched.h>
#include <poll.h>
#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/prctl.h>
#include <systemlib/err.h>
#include <systemlib/systemlib.h>
#include <termios.h>
#include <unistd.h>
#include <uORB/topics/actuator_armed.h>
#include <uORB/topics/manual_control_setpoint.h>
#include <uORB/topics/parameter_update.h>
#include <uORB/topics/vehicle_attitude.h>
#include <uORB/topics/vehicle_attitude_setpoint.h>
#include <uORB/topics/vehicle_control_mode.h>
#include <uORB/topics/vehicle_local_position.h>
#include <uORB/topics/vehicle_local_position_setpoint.h>
#include <uORB/uORB.h>

#include "helen_pos_control_params.h"

using namespace math;

static bool thread_should_exit = false; /**< Deamon exit flag */
static bool thread_running = false; /**< Deamon status flag */
static int helen_pos_control_task; /**< Handle of deamon task / thread */
static bool verbose_mode = false;
static const uint32_t pub_interval = 10000; // limit publish rate to 100 Hz


extern "C" __EXPORT int helen_pos_control_main(int argc, char *argv[]);

int helen_pos_control_thread_main(int argc, char *argv[]);

static int sign(float num) {
    if (num >= 0.0f)
        return 1;
    else
        return -1;
}

static void usage(const char *reason);

/**
 * Print the correct usage.
 */
static void usage(const char *reason)
{
    if (reason)
        fprintf(stderr, "%s\n", reason);

    fprintf(stderr, "usage: helen_pos_control {start|stop|status} [-v]\n\n");
    exit(1);
}


// Function to start a new position estimator thread... Copy and paste here.
int helen_pos_control_main(int argc, char *argv[])
{
    if (argc < 1)
        usage("missing command");

    if (!strcmp(argv[1], "start")) {
        if (thread_running) {
            warnx("already running");
            /* this is not an error */
            exit(0);
        }
        verbose_mode = false;
        if (argc > 1)
            if (!strcmp(argv[2], "-v"))
                verbose_mode = true;
        thread_should_exit = false;
        helen_pos_control_task = task_spawn_cmd("helen_pos_control",
                           SCHED_DEFAULT, SCHED_PRIORITY_MAX - 100, 5000,
                           helen_pos_control_thread_main,
                           (argv) ? (const char **) &argv[2] : (const char **) NULL);
        exit(0);
    }
    if (!strcmp(argv[1], "stop")) {
        if (thread_running) {
            warnx("stop");
            thread_should_exit = true;
        } else {
            warnx("app not started");
        }
        exit(0);
    }
    if (!strcmp(argv[1], "status")) {
        if (thread_running) {
            warnx("app is running");
        } else {
            warnx("app not started");
        }
        exit(0);
    }
    usage("unrecognized command");
    exit(1);
}


void PositionControllerPID(const Vector<10>& refpoint,
                           const Vector<6>& state,
                           const Matrix<3, 3>& R,
                           const Matrix<3, 3>& Kp,
                           const Matrix<3, 3>& Kd,
                           const Vector<3>& F_lim,
                           float m,
                           Matrix<3, 3>* R_des,
                           float* thrust) {

    //  PD controller
    // error in position earth frame

    Vector<3> e_p, e_v, ff, g_vect;
    e_p.zero();
    e_v.zero();
    ff.zero();
    g_vect.zero();
    e_p(0) = refpoint(0) - state(0);
    e_p(1) = refpoint(1) - state(1);
    e_p(2) = refpoint(2) - state(2);
    // error in velocity earth frame
    e_v(0) = refpoint(3) - state(3);
    e_v(1) = refpoint(4) - state(4);
    e_v(2) = refpoint(5) - state(5);
    // ff
    ff(0) = refpoint(6);
    ff(1) = refpoint(7);
    ff(2) = refpoint(8);
    // gravity vector
    g_vect(2) = -9.81f;

    // desired Force vector in the worldframe
    Vector<3> F_des;
    F_des = -(Kp*e_p + Kd*e_v + ff*m + g_vect*m);

    if (fabsf(F_des(0)) >= F_lim(0)) {
        F_des(0) = sign(F_des(0))*F_lim(0);
    }
    if (fabsf(F_des(1)) >= F_lim(1)) {
        F_des(1) = sign(F_des(1))*F_lim(1);
    }
    if (fabsf(F_des(2)) >= F_lim(2)) {
        F_des(2) = sign(F_des(2))*F_lim(2);
    }

    // body z axis
    Vector<3> z_b, z_B_des, x_C, y_B_des, x_B_des;
    z_b(0) = R(0, 2);
    z_b(1) = R(1, 2);
    z_b(2) = R(2, 2);

    // desired thrust in body frame
    *thrust = F_des*z_b;
    // desired body z axis
    z_B_des = F_des.normalized();
    // desired direction in world coordinates (Yaw angle)
    x_C(0) = cosf(refpoint(9));
    x_C(1) = sinf(refpoint(9));
    x_C(2) = 0.0f;

    // desired body y axis
    y_B_des = (z_B_des%x_C) / ((z_B_des%x_C).length());
    // desired body x axis
    x_B_des = y_B_des%z_B_des;
    // desired Rotation Matrix
    //R_des=[x_B_des,y_B_des,z_B_des];
    (*R_des)(0, 0) = x_B_des(0);
    (*R_des)(1, 0) = x_B_des(1);
    (*R_des)(2, 0) = x_B_des(2);
    (*R_des)(0, 1) = y_B_des(0);
    (*R_des)(1, 1) = y_B_des(1);
    (*R_des)(2, 1) = y_B_des(2);
    (*R_des)(0, 2) = z_B_des(0);
    (*R_des)(1, 2) = z_B_des(1);
    (*R_des)(2, 2) = z_B_des(2);
}




/****************************************************************************
 * main
 ****************************************************************************/
int helen_pos_control_thread_main(int argc, char *argv[])
{
    warnx("started");
    // Start log device for output.
    int mavlink_fd;
    mavlink_fd = open(MAVLINK_LOG_DEVICE, 0);
    mavlink_log_info(mavlink_fd, "[pos control] started");

    /* declare and safely initialize all structs */
    struct actuator_armed_s armed;
    memset(&armed, 0, sizeof(armed));
    struct vehicle_attitude_s att;
    memset(&att, 0, sizeof(att));
    struct vehicle_local_position_s local_pos;
    memset(&local_pos, 0, sizeof(local_pos));
    struct vehicle_attitude_setpoint_s att_sp;
    memset(&att_sp, 0, sizeof(att_sp));
    struct vehicle_control_mode_s control_mode;
    memset(&control_mode, 0, sizeof(control_mode));
    struct vehicle_local_position_setpoint_s local_pos_sp;
    memset(&local_pos_sp, 0, sizeof(local_pos_sp));
    struct manual_control_setpoint_s manual;
    memset(&manual, 0, sizeof(manual));

    /* subscribe */
    int parameter_update_sub = orb_subscribe(ORB_ID(parameter_update));
    int armed_sub = orb_subscribe(ORB_ID(actuator_armed));
    int vehicle_attitude_sub = orb_subscribe(ORB_ID(vehicle_attitude));
    int local_pos_sub = orb_subscribe(ORB_ID(vehicle_local_position));
    int control_mode_sub = orb_subscribe(ORB_ID(vehicle_control_mode));
    int att_sp_sub = orb_subscribe(ORB_ID(vehicle_attitude_setpoint));
    int manual_sub = orb_subscribe(ORB_ID(manual_control_setpoint));

    /* advertise */
    orb_advert_t att_sp_pub = orb_advertise(ORB_ID(vehicle_attitude_setpoint), &att_sp);
    orb_advert_t lpos_sp_pub = orb_advertise(ORB_ID(vehicle_local_position_setpoint), &local_pos_sp);

    // Configure.
    // TODO: Load all the parameter values and configure.
    float g = 9.81f;       // m/s^2
    float m = 0.478f;      // kg
    float thrust_scale = 7.0f; // Thrust -> Force magic conversion units.
    float k_p_gain_z = 1.0f;
    float k_d_gain_z = 0.0f;
    float k_p_gain_xy = 1.0f;
    float k_d_gain_xy = 0.0f;
    float f_lim_xy = 4.0f;
    float f_lim_z = 10.0f;

    // Read in some stuff for parameters.
    struct helen_pos_control_params params;
    struct helen_pos_control_param_handles param_handles;
    /* initialize parameter handles */
    parameters_init(&param_handles);

    /* first parameters read at start up */
    struct parameter_update_s param_update;
    /* read from param topic to clear updated flag */
    orb_copy(ORB_ID(parameter_update), parameter_update_sub, &param_update);
    /* first parameters update */
    parameters_update(&param_handles, &params);

    m = params.m;
    thrust_scale = params.thrust_scale;
    k_p_gain_z = params.k_p_gain_z;
    k_d_gain_z = params.k_d_gain_z;
    k_p_gain_xy = params.k_p_gain_xy;
    k_d_gain_xy = params.k_d_gain_xy;
    f_lim_xy = params.f_lim_xy;
    f_lim_z = params.f_lim_z;

    // Let's start this loop up!
    // Times.
    hrt_abstime pub_last = hrt_absolute_time();
    hrt_abstime t_last = hrt_absolute_time();
    hrt_abstime t = hrt_absolute_time();

    thread_running = true;

    // Here are some matrices.
    Matrix<3, 3> R, R_des, Kp, Kd, R_yaw;
    Vector<3> F_lim;
    Vector<10> refpoint;
    Vector<6> state;
    Vector<3> sp_move_rate;
    float thrust = 0.0f;
    // Keep track of what the previous flags were.
    bool position_enabled_before = false, altitude_enabled_before = false;

    Kp.identity();
    Kp(0, 0) = k_p_gain_xy;
    Kp(1, 1) = k_p_gain_xy;
    Kp(2, 2) = k_p_gain_z;
    Kd.identity();
    Kd(0, 0) = k_d_gain_xy;
    Kd(1, 1) = k_d_gain_xy;
    Kd(2, 2) = k_d_gain_z;
    F_lim(0) = f_lim_xy;
    F_lim(1) = f_lim_xy;
    F_lim(2) = f_lim_z;

    refpoint.zero();

    // Let's set up the main loop now.
    struct pollfd fds[1];
    fds[0].fd = local_pos_sub;
    fds[0].events = POLLIN;

    float dt = 0.0f;

    while (!thread_should_exit) {
        int ret = poll(fds, 1, 200); // wait maximal 20 ms = 50 Hz minimum rate
        t = hrt_absolute_time();
        dt = (t - t_last)*0.000001;


        if (ret < 0) {
            /* poll error */
            warnx("subscriptions poll error.");
            thread_should_exit = true;
            continue;
        } else if (ret > 0) {
            bool updated = false;

            /* parameter update */
            orb_check(parameter_update_sub, &updated);

            if (updated) {
                struct parameter_update_s update;
                orb_copy(ORB_ID(parameter_update), parameter_update_sub, &update);
                parameters_update(&param_handles, &params);

                m = params.m;
                thrust_scale = params.thrust_scale;
                k_p_gain_z = params.k_p_gain_z;
                k_d_gain_z = params.k_d_gain_z;
                k_p_gain_xy = params.k_p_gain_xy;
                k_d_gain_xy = params.k_d_gain_xy;
                f_lim_xy = params.f_lim_xy;
                f_lim_z = params.f_lim_z;
            }

            orb_check(local_pos_sub, &updated);

            if (updated) {
                orb_copy(ORB_ID(vehicle_local_position), local_pos_sub, &local_pos);
            }

            /* vehicle attitude */
            orb_check(vehicle_attitude_sub, &updated);
            if (updated) {
                orb_copy(ORB_ID(vehicle_attitude), vehicle_attitude_sub, &att);
            }

            /* armed */
            orb_check(armed_sub, &updated);
            if (updated) {
                orb_copy(ORB_ID(actuator_armed), armed_sub, &armed);
            }

            orb_check(control_mode_sub, &updated);
            if (updated) {
                orb_copy(ORB_ID(vehicle_control_mode), control_mode_sub, &control_mode);
            }

            orb_check(att_sp_sub, &updated);
            if (updated) {
                orb_copy(ORB_ID(vehicle_attitude_setpoint), att_sp_sub, &att_sp);
            }

            orb_check(manual_sub, &updated);
            if (updated) {
                orb_copy(ORB_ID(manual_control_setpoint), manual_sub, &manual);
            }

            // Crap to figure out refpoints.
            // Set the refpoints if we're just switching into this mode.
            if (control_mode.flag_control_altitude_enabled &&
                !altitude_enabled_before) {
                refpoint(2) = local_pos.z;
                refpoint(9) = att.yaw;
                altitude_enabled_before = true;
            }
            if (!control_mode.flag_control_altitude_enabled &&
                altitude_enabled_before) {
                altitude_enabled_before = false;
            }
            // Set the refpoints if we're just switching into this mode.
            if (control_mode.flag_control_position_enabled &&
                !position_enabled_before) {
                refpoint(0) = local_pos.x;
                refpoint(1) = local_pos.y;
                refpoint(2) = local_pos.z;
                refpoint(9) = att.yaw;
                position_enabled_before = true;
            }
            if (!control_mode.flag_control_position_enabled &&
                position_enabled_before) {
                position_enabled_before = false;
            }



            // Adjust the setpoints if the sticks are not 0.
            float dz = 0.1f;
            float stick_scale = 1.0f;
            if (control_mode.flag_control_position_enabled) {
                // Sticks are in the local frame.
                sp_move_rate.zero();
                if (fabs(manual.x) > 0.1) {
                    sp_move_rate(0) = manual.x * stick_scale * dt;
                }
                if (fabs(manual.y) > 0.1) {
                    sp_move_rate(1) = manual.y * stick_scale * dt;
                }

                // Rotate back into world frame.
                R_yaw.from_euler(0.0f, 0.0f, att.yaw);
                sp_move_rate = R_yaw * sp_move_rate;

                refpoint(0) += sp_move_rate(0);
                refpoint(1) += sp_move_rate(1);
            }

            // Actual control loop.
            if (control_mode.flag_control_altitude_enabled ||
                control_mode.flag_control_position_enabled) {
                // Do all controls.

                // Fill in matrices I guess? State?
                state(0) = local_pos.x;
                state(1) = local_pos.y;
                state(2) = local_pos.z;
                state(3) = local_pos.vx;
                state(4) = local_pos.vy;
                state(5) = local_pos.vz;

                // R! do R. Copy it from attitude.
                memcpy(R.data, att.R, sizeof(R.data));

                // Refpoint is stationary in this implementation.
                // But maybe should make sure yaw is correct...

                if (!control_mode.flag_control_position_enabled) {
                    Kp(0, 0) = 0.0f;
                    Kp(1, 1) = 0.0f;
                    Kd(0, 0) = 0.0f;
                    Kd(1, 1) = 0.0f;
                    Kp(2, 2) = k_p_gain_z;
                    Kd(2, 2) = k_d_gain_z;
                } else {
                    Kp(0, 0) = k_p_gain_xy;
                    Kp(1, 1) = k_p_gain_xy;
                    Kd(0, 0) = k_d_gain_xy;
                    Kd(1, 1) = k_d_gain_xy;
                    Kp(2, 2) = k_p_gain_z;
                    Kd(2, 2) = k_d_gain_z;
                }
                //refpoint(2) = -1.0f;

                PositionControllerPID(refpoint, state, R, Kp, Kd,
                                      F_lim, m, &R_des, &thrust);
                // Scale by the thrust scale, because thrust is not in Newtons,
                // it's in magic units of magic.
                thrust = thrust/thrust_scale;

                if (!control_mode.flag_control_position_enabled) {
                    R_des.from_euler(0.0f, 0.0f, att.yaw);
                    att_sp.roll_body = 0.0f;
                    att_sp.pitch_body = 0.0f;
                    att_sp.yaw_body = att.yaw;
                } else {
                    // Fill this crap in too...
                    Vector<3> euler_angles;
                    euler_angles = R_des.to_euler();
                    att_sp.roll_body = euler_angles(0);
                    att_sp.pitch_body = euler_angles(1);
                    att_sp.yaw_body = euler_angles(2);
                }

                // Fill in the attitude setpoint from this info.
                memcpy(&att_sp.R_body[0][0], R_des.data, sizeof(att_sp.R_body));
                att_sp.R_valid = true;

                att_sp.thrust = thrust;
            } else {
                /*refpoint(9) = att.yaw;
                // Clear out the struct.
                // Do I need to know this? I dunno.
                R.from_euler(0.0f, 0.0f, att.yaw);

                memcpy(&att_sp.R_body[0][0], R.data, sizeof(att_sp.R_body));
                att_sp.R_valid = true;

                att_sp.roll_body = 0.0f;
                att_sp.pitch_body = 0.0f;
                att_sp.yaw_body = att.yaw;
                att_sp.thrust = 0.0f; */
            }

            t_last = t;
        }

        //if (t > pub_last + pub_interval) {
        if (t > pub_last + pub_interval
             && (control_mode.flag_control_altitude_enabled ||
                control_mode.flag_control_position_enabled)) {
            pub_last = t;

            // Fill in extra attitude_sp parameters.
            att_sp.timestamp = t;

            // Fill in extra local position setpoints.
            local_pos_sp.x = refpoint(0);
            local_pos_sp.y = refpoint(1);
            local_pos_sp.z = refpoint(2);
            local_pos_sp.yaw = refpoint(9);

            orb_publish(ORB_ID(vehicle_attitude_setpoint), att_sp_pub, &att_sp);
            orb_publish(ORB_ID(vehicle_local_position_setpoint), lpos_sp_pub, &local_pos_sp);

            //warnx("Published.");
        }

        // Sleep for 10 ms
        usleep(5000); // 5 ms

        //usleep(1000000); // 1 s
        //usleep(40000);
    }

    warnx("stopped");
    mavlink_log_info(mavlink_fd, "[pos control] stopped");
    thread_running = false;
    return 0;
}