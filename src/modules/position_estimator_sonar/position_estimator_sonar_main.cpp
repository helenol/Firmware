/****************************************************************************
 *
 *   Copyright (C) 2013 Helen Oleynikova. All rights reserved.
 *   Author:    Helen Oleynikova
 *   Based on code by: Anton Babushkin  <rk3dov@gmail.com>
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name PX4 nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

/**
 * @file position_estimator_sonar_main.c
 * Model-identification based position estimator for multirotors
 */

#include <unistd.h>
#include <stdlib.h>
#include <stdio.h>
#include <stdbool.h>
#include <fcntl.h>
#include <float.h>
#include <string.h>
#include <nuttx/config.h>
#include <nuttx/sched.h>
#include <sys/prctl.h>
#include <termios.h>
#include <errno.h>
#include <limits.h>
#include <math.h>
#include <uORB/uORB.h>
#include <uORB/topics/parameter_update.h>
#include <uORB/topics/actuator_controls.h>
#include <uORB/topics/actuator_armed.h>
#include <uORB/topics/sensor_combined.h>
#include <uORB/topics/vehicle_attitude.h>
#include <uORB/topics/vehicle_local_position.h>
#include <uORB/topics/vehicle_global_position.h>
#include <uORB/topics/vehicle_gps_position.h>
#include <uORB/topics/optical_flow.h>
#include <mavlink/mavlink_log.h>
#include <poll.h>
#include <systemlib/err.h>
#include <geo/geo.h>
#include <systemlib/systemlib.h>
#include <drivers/drv_hrt.h>

#include "sonar_prefilter.h"
#include "sonar_ekf.h"

#ifdef __cplusplus
extern "C" {
#endif
#include "position_estimator_sonar_params.h"
#ifdef __cplusplus
}
#endif

static bool thread_should_exit = false; /**< Deamon exit flag */
static bool thread_running = false; /**< Deamon status flag */
static int position_estimator_sonar_task; /**< Handle of deamon task / thread */
static bool verbose_mode = false;
static const uint32_t updates_counter_len = 1000000;
static const uint32_t pub_interval = 10000; // limit publish rate to 100 Hz
static const float max_flow = 1.0f; // max flow value that can be used, rad/s

extern "C" __EXPORT int position_estimator_sonar_main(int argc, char *argv[]);

int position_estimator_sonar_thread_main(int argc, char *argv[]);

static void usage(const char *reason);

/**
 * Print the correct usage.
 */
static void usage(const char *reason)
{
    if (reason)
        fprintf(stderr, "%s\n", reason);

    fprintf(stderr, "usage: position_estimator_sonar {start|stop|status} [-v]\n\n");
    exit(1);
}

/**
 * The position_estimator_sonar_thread only briefly exists to start
 * the background job. The stack size assigned in the
 * Makefile does only apply to this management task.
 *
 * The actual stack size should be set in the call
 * to task_create().
 */
int position_estimator_sonar_main(int argc, char *argv[])
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
        position_estimator_sonar_task = task_spawn_cmd("position_estimator_sonar",
                           SCHED_RR, SCHED_PRIORITY_MAX - 5, 4096,
                           position_estimator_sonar_thread_main,
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

/****************************************************************************
 * main
 ****************************************************************************/
int position_estimator_sonar_thread_main(int argc, char *argv[])
{
    warnx("started");
    // Start log device for output.
    int mavlink_fd;
    mavlink_fd = open(MAVLINK_LOG_DEVICE, 0);
    mavlink_log_info(mavlink_fd, "[sonar] started");

    /* declare and safely initialize all structs */
    struct actuator_controls_s actuator;
    memset(&actuator, 0, sizeof(actuator));
    struct actuator_armed_s armed;
    memset(&armed, 0, sizeof(armed));
    struct sensor_combined_s sensor;
    memset(&sensor, 0, sizeof(sensor));
    struct vehicle_attitude_s att;
    memset(&att, 0, sizeof(att));
    struct vehicle_local_position_s local_pos;
    memset(&local_pos, 0, sizeof(local_pos));
    struct optical_flow_s flow;
    memset(&flow, 0, sizeof(flow));
    struct vehicle_global_position_s global_pos;
    memset(&global_pos, 0, sizeof(global_pos));

    /* subscribe */
    int parameter_update_sub = orb_subscribe(ORB_ID(parameter_update));
    int actuator_sub = orb_subscribe(ORB_ID_VEHICLE_ATTITUDE_CONTROLS);
    int armed_sub = orb_subscribe(ORB_ID(actuator_armed));
    int sensor_combined_sub = orb_subscribe(ORB_ID(sensor_combined));
    int vehicle_attitude_sub = orb_subscribe(ORB_ID(vehicle_attitude));
    int optical_flow_sub = orb_subscribe(ORB_ID(optical_flow));

    /* advertise */
    orb_advert_t vehicle_local_position_pub = orb_advertise(ORB_ID(vehicle_local_position), &local_pos);

    struct position_estimator_sonar_params params;
    struct position_estimator_sonar_param_handles pos_sonar_param_handles;
    /* initialize parameter handles */
    parameters_init(&pos_sonar_param_handles);

    /* first parameters read at start up */
    struct parameter_update_s param_update;
    orb_copy(ORB_ID(parameter_update), parameter_update_sub, &param_update); /* read from param topic to clear updated flag */
    /* first parameters update */
    parameters_update(&pos_sonar_param_handles, &params);

    struct pollfd fds_init[1];
    fds_init[0].fd = sensor_combined_sub;
    fds_init[0].events = POLLIN;

    // All the parameters we intend to use.
    // baro offset for reference altitude.
    // This is adjusted by the kalman filter as well, 
    // just good to have an initial value to make the states small.
    float baro_offset = 0.0f;

    // Times.
    hrt_abstime updates_counter_start = hrt_absolute_time();
    hrt_abstime pub_last = hrt_absolute_time();

    hrt_abstime t_prev = 0;

    /* wait for initial baro value */
    bool wait_baro = true;
    unsigned int baro_counter = 0;

    thread_running = true;

    // Do I still care about this? I feel like we're just as well off grabbing
    // the first value and going for it.
    while (wait_baro && !thread_should_exit) {
        int ret = poll(fds_init, 1, 1000);
        int baro_init_cnt = 0;
        int baro_init_num = 200;
        if (ret < 0) {
            /* poll error */
            errx(1, "subscriptions poll error on init.");
        } else if (ret > 0) {
            if (fds_init[0].revents & POLLIN) {
                orb_copy(ORB_ID(sensor_combined), sensor_combined_sub, &sensor);
                baro_offset = sensor.baro_alt_meter;
                wait_baro = false;
                warnx("baro offs: %.2f", baro_offset);
                mavlink_log_info(mavlink_fd, "[sonar] baro offs: %.2f", baro_offset);
            }
        }
    }

    // Steps: check each of the sensors for new stuff, and update their states.
    // If there's any new stuff, update the kalman filter.
    // If it's time for the publish update, update!
    /* main loop */
    struct pollfd fds[1];
    fds[0].fd = vehicle_attitude_sub;
    fds[0].events = POLLIN;

    // All the parameters we're actually going to use for filtering.
    float acc_z = 0;
    float baro_alt = 0;
    float sonar = 0;
    float sonar_sigma = 0;

    // Create a super happy fun Sonar Prefilter and EKF
    SonarPrefilter prefilter;
    prefilter.init(hrt_absolute_time());


    SonarEKF ekf;

    // Initialize with zero state and very high P.
    math::Matrix P_init(5, 5);
    P_init.setAll(0.0f);
    float diag = 1000;
    P_init(0, 0) = diag;
    P_init(1, 1) = diag;
    P_init(2, 2) = diag;
    P_init(3, 3) = diag;
    P_init(4, 4) = diag;
    math::Vector x_init(5);
    x_init.setAll(0.0f);
    ekf.init(0.03f, 0.0006f, 0.5f, 0.005f, 0.00001f, x_init, P_init, 
            hrt_absolute_time());
    math::Vector x_est(3);

    while (!thread_should_exit) {
        int ret = poll(fds, 1, 20); // wait maximal 20 ms = 50 Hz minimum rate
        hrt_abstime t = hrt_absolute_time();

        if (ret < 0) {
            /* poll error */
            warnx("subscriptions poll error.");
            thread_should_exit = true;
            continue;
        } else if (ret > 0) {
            bool updated = false;
            bool new_data = false;

            /* vehicle attitude */
            orb_check(vehicle_attitude_sub, &updated);
            if (updated) {
                orb_copy(ORB_ID(vehicle_attitude), vehicle_attitude_sub, &att);
            }

            /* parameter update */
            orb_check(parameter_update_sub, &updated);
            if (updated) {
                struct parameter_update_s update;
                orb_copy(ORB_ID(parameter_update), parameter_update_sub, &update);
                parameters_update(&pos_sonar_param_handles, &params);
            }

            /* actuator */
            orb_check(actuator_sub, &updated);
            if (updated) {
                orb_copy(ORB_ID_VEHICLE_ATTITUDE_CONTROLS, actuator_sub, &actuator);
            }

            /* armed */
            orb_check(armed_sub, &updated);
            if (updated) {
                orb_copy(ORB_ID(actuator_armed), armed_sub, &armed);
            }

            /* sensor combined */
            orb_check(sensor_combined_sub, &updated);
            if (updated) {
                orb_copy(ORB_ID(sensor_combined), sensor_combined_sub, &sensor);
                acc_z = sensor.accelerometer_m_s2[2] + CONSTANTS_ONE_G;

                baro_alt = sensor.baro_alt_meter - baro_offset;

                new_data = true;

                //warnx("acc_z: %.2f, baro_alt: %.2f", acc_z, baro_alt);
            }
            /* optical flow */
            orb_check(optical_flow_sub, &updated);

            if (updated) {
                orb_copy(ORB_ID(optical_flow), optical_flow_sub, &flow);

                prefilter.addMeasurement(t, flow.ground_distance_m);
                prefilter.getLatestUpdate(t, &sonar, &sonar_sigma);

                new_data = true;
            }

            // Now do the kalman stuff.

            if (new_data) {
                // The state (x) is [z, z_dot, z_ddot, acc_bias, baro_bias]
                // The control input (u) is currently nothing
                // The observed state is (z_obs) = [acc baro sonar]

                // Create z_obs
                math::Vector z_obs(3);
                z_obs.setAll(0.0f);
                z_obs(0) = acc_z;
                z_obs(1) = baro_alt;
                z_obs(2) = sonar;

                //printf("In: %.2f %.2f %.2f\n", acc_z, baro_alt, sonar);

                ekf.ekfStep(t, z_obs, sonar_sigma, &x_est);
            }

            //warnx("Z_obs: %.2f %.2f %.2f", z_obs(0), z_obs(1), z_obs(2));
        }

        if (t > pub_last + pub_interval) {
            pub_last = t;
            /* publish local position */
            local_pos.z_valid = true;
            local_pos.v_z_valid = true;
            local_pos.xy_valid = false;
            local_pos.v_xy_valid = false;
            local_pos.xy_global = false;
            local_pos.z_global = false;
            // TODO - FIX FIX FIX
            local_pos.y = acc_z;
            local_pos.x = baro_alt;
            local_pos.vx = sonar;
            local_pos.vy = sonar_sigma;
            local_pos.z = -x_est(0);
            local_pos.vz = -x_est(1);
            // Don't know if we landed or not?
            local_pos.landed = x_est(0) <= 0.0;
            local_pos.yaw = att.yaw;
            local_pos.timestamp = t;

            orb_publish(ORB_ID(vehicle_local_position), vehicle_local_position_pub, &local_pos);
            //warnx("Published.");

        }

        // Sleep for 1 ms
        usleep(1000);
    }

    warnx("stopped");
    mavlink_log_info(mavlink_fd, "[sonar] stopped");
    thread_running = false;
    return 0;
};
