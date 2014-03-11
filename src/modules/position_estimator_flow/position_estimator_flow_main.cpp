#include <drivers/drv_hrt.h>
#include <errno.h>
#include <fcntl.h>
#include <float.h>
#include <limits.h>
#include <math.h>
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
#include <uORB/topics/actuator_controls.h>
#include <uORB/topics/optical_flow.h>
#include <uORB/topics/parameter_update.h>
#include <uORB/topics/sensor_combined.h>
#include <uORB/topics/vehicle_attitude.h>
#include <uORB/topics/vehicle_local_position.h>
#include <uORB/uORB.h>

#include "flow_ekf.h"
#include "sonar_prefilter.h"
//#include "position_estimator_flow_params.h"

using namespace math;

static bool thread_should_exit = false; /**< Deamon exit flag */
static bool thread_running = false; /**< Deamon status flag */
static int position_estimator_flow_task; /**< Handle of deamon task / thread */
static bool verbose_mode = false;
static const uint32_t pub_interval = 10000; // limit publish rate to 100 Hz


extern "C" __EXPORT int position_estimator_flow_main(int argc, char *argv[]);

int position_estimator_flow_thread_main(int argc, char *argv[]);

static void usage(const char *reason);

/**
 * Print the correct usage.
 */
static void usage(const char *reason)
{
    if (reason)
        fprintf(stderr, "%s\n", reason);

    fprintf(stderr, "usage: position_estimator_flow {start|stop|status} [-v]\n\n");
    exit(1);
}

// Position estimator main.

// Function to start a new position estimator thread... Copy and paste here.

/**
 * The position_estimator_flow_thread only briefly exists to start
 * the background job. The stack size assigned in the
 * Makefile does only apply to this management task.
 *
 * The actual stack size should be set in the call
 * to task_create().
 */
int position_estimator_flow_main(int argc, char *argv[])
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
        position_estimator_flow_task = task_spawn_cmd("position_estimator_flow",
                           SCHED_RR, SCHED_PRIORITY_MAX - 30, 10000,
                           position_estimator_flow_thread_main,
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
int position_estimator_flow_thread_main(int argc, char *argv[])
{
    warnx("started");
    // Start log device for output.
    int mavlink_fd;
    mavlink_fd = open(MAVLINK_LOG_DEVICE, 0);
    mavlink_log_info(mavlink_fd, "[flow ekf] started");

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

    /* subscribe */
    int parameter_update_sub = orb_subscribe(ORB_ID(parameter_update));
    // TODO: fix
    int actuator_sub = orb_subscribe(ORB_ID_VEHICLE_ATTITUDE_CONTROLS);
    int armed_sub = orb_subscribe(ORB_ID(actuator_armed));
    int sensor_combined_sub = orb_subscribe(ORB_ID(sensor_combined));
    int vehicle_attitude_sub = orb_subscribe(ORB_ID(vehicle_attitude));
    int optical_flow_sub = orb_subscribe(ORB_ID(optical_flow));

    /* advertise */
    orb_advert_t vehicle_local_position_pub = orb_advertise(ORB_ID(vehicle_local_position), &local_pos);

    //struct position_estimator_flow_params params;
    //struct position_estimator_flow_param_handles pos_flow_param_handles;
    /* initialize parameter handles */
    //parameters_init(&pos_flow_param_handles);
    
    // Let's start with the fun stuff now.
    // First, create the EKF object.
    // Also create the sonar prefilter object.
    FlowEKF ekf;
    SonarPrefilter prefilter;

    // Initialize.
    Vector<N_STATES> x;
    x.zero();
    Matrix<N_STATES, N_STATES> P;
    P.identity();
    P = P*0.1;

    // State: [x, x_dot, y, y_dot, z, z_dot, bias_ax, bias_ay, bias_az, bias_b]
    // Coordinate system is NED (north, east, down).
    // Observations: [acc_x, acc_y, acc_z, baro, flow_x, flow_y, sonar]
    hrt_abstime t = hrt_absolute_time();
    ekf.init(t, x, P);
    prefilter.init(t);

    // Configure.
    // TODO: Load all the parameter values and configure.
    float g = 9.81;       // m/s^2
    float m = 0.490;      // kg
    float thrust_scale = 7.15; // Thrust -> Force magic conversion units.
    float sigma_acc = 0.03;
    float sigma_acc_bias = 0.0006;
    float sigma_baro = 0.5;
    float sigma_baro_bias = 0.005;
    float sigma_flow = 0.3;
    float sigma_sonar = 0.02;
    float sigma_pos_noise = 0.01;
    float sigma_vel_noise = 0.001;

    float flow_f = 16.0f/(24)*1000.0f;
    float flow_frame_rate = 120.0f;

    ekf.setParams(sigma_acc, sigma_baro, sigma_flow, sigma_sonar, 
                  sigma_pos_noise, sigma_vel_noise, sigma_acc_bias, 
                  sigma_baro_bias);
    // Trust the default params for sonar.

    // Let's start this loop up!

    struct pollfd fds_init[1];
    fds_init[0].fd = sensor_combined_sub;
    fds_init[0].events = POLLIN;

    // Times.
    hrt_abstime updates_counter_start = hrt_absolute_time();
    hrt_abstime pub_last = hrt_absolute_time();

    thread_running = true;

    // Keep offsets.
    float baro_offset;

    // Poll once to get the baro offset... I think we don't need to do more?
    // Wait a max of 1000 ms = 1 second. Sensors SHOULD come up by then?
    {
        int ret = poll(fds_init, 1, 1000);
        if (ret < 0) {
            /* poll error */
            errx(1, "subscriptions poll error on init.");
        } else if (ret > 0) {
            if (fds_init[0].revents & POLLIN) {
                orb_copy(ORB_ID(sensor_combined), sensor_combined_sub, &sensor);
                baro_offset = sensor.baro_alt_meter;
                warnx("baro offs: %.2f", baro_offset);
                mavlink_log_info(mavlink_fd, "[flow ekf] baro offs: %.2f", baro_offset);
            }
        }
    }

    // Let's set up the main loop now.
    struct pollfd fds[1];
    fds[0].fd = vehicle_attitude_sub;
    fds[0].events = POLLIN;

    // This is for checking whether the values we're getting are ACTUALLY new.
    uint32_t accel_counter = 0;
    uint32_t baro_counter = 0;

    // And this is the actual vector we will pass in to the ekf.
    Vector<N_MEASURE> z;
    z.zero();
    Vector<N_CONTROL> u;
    u.zero();

    while (!thread_should_exit) {
        int ret = poll(fds, 1, 20); // wait maximal 20 ms = 50 Hz minimum rate
        t = hrt_absolute_time();

        if (ret < 0) {
            /* poll error */
            warnx("subscriptions poll error.");
            thread_should_exit = true;
            continue;
        } else if (ret > 0) {
            bool updated = false;
            bool new_attitude = false;
            bool new_sensors = false;
            bool new_flow = false;
            bool valid_sonar = false;

            /* vehicle attitude */
            orb_check(vehicle_attitude_sub, &updated);
            if (updated) {
                orb_copy(ORB_ID(vehicle_attitude), vehicle_attitude_sub, &att);
                u(1) = att.pitch;
                u(2) = att.roll;
                u(3) = att.yaw;
                new_attitude = true;
            }

            /* parameter update */
            /*orb_check(parameter_update_sub, &updated);
            if (updated) {
                struct parameter_update_s update;
                orb_copy(ORB_ID(parameter_update), parameter_update_sub, &update);
                parameters_update(&pos_sonar_param_handles, &params);
            }*/

            /* actuator */
            orb_check(actuator_sub, &updated);
            if (updated) {
                orb_copy(ORB_ID_VEHICLE_ATTITUDE_CONTROLS, actuator_sub, &actuator);
                u(0) = actuator.control[3]*thrust_scale/m;
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
                z(0) = sensor.accelerometer_m_s2[0];
                z(1) = sensor.accelerometer_m_s2[1];
                z(2) = sensor.accelerometer_m_s2[2];
                z(3) = sensor.baro_alt_meter - baro_offset;

                new_sensors = true;
            }

            /* optical flow */
            orb_check(optical_flow_sub, &updated);
            if (updated) {
                orb_copy(ORB_ID(optical_flow), optical_flow_sub, &flow);

                // TODO: add gyro compensation.
                z(4) = -flow.flow_raw_y/10.0/flow_f*x(4)*flow_frame_rate;
                z(5) = flow.flow_raw_x/10.0/flow_f*x(4)*flow_frame_rate;
                z(6) = flow.ground_distance_m;
                valid_sonar = prefilter.isValid(t, flow.ground_distance_m);

                new_flow = true;
            }

            // Now do the kalman stuff.

            if (new_attitude || new_sensors || new_flow) {
                ekf.ekfStep(t, z, u, new_sensors, new_flow, valid_sonar, &x);
                warnx("ekf update, u: %.2f %.2f %.2f %.2f", 
                        u(0), u(1), u(2), u(3));
                warnx("ekf update, z: %.2f %.2f %.2f %.2f %.2f %.2f %.2f", 
                        z(0), z(1), z(2), z(3), z(4), z(5), z(6));

            }
        }

        if (t > pub_last + pub_interval) {
            pub_last = t;
            /* publish local position */
            local_pos.z_valid = true;
            local_pos.v_z_valid = true;
            local_pos.xy_valid = true;
            local_pos.v_xy_valid = true;
            local_pos.xy_global = false;
            local_pos.z_global = false;
            local_pos.x = x(0);
            local_pos.y = x(2);
            local_pos.z = x(4);
            local_pos.vx = x(1);
            local_pos.vy = x(3);
            local_pos.vz = x(5);
            // Don't know if we landed or not?
            local_pos.landed = x(4) >= -0.1;
            // I don't know why there is yaw here and how it is used.
            // In the future, maybe use as offset to global frame???
            local_pos.yaw = att.yaw;
            local_pos.timestamp = t;

            orb_publish(ORB_ID(vehicle_local_position),
                        vehicle_local_position_pub, &local_pos);
            warnx("Published.");
        }

        // Sleep for 10 ms
        usleep(40000);
    }

    warnx("stopped");
    mavlink_log_info(mavlink_fd, "[flow ekf] stopped");
    thread_running = false;
    return 0;
}






// Set up subscriptions to the necessary topics.
// Attitude
// Control input (where thrust comes from) (ATTC?)
// Optical flow
// Barometer
// Accelerometers

// Wait for the first attitude message, just to be sure.


// Set the initial position to 0, 0, 0 with pretty high confidence.

// Poll on attitude, assign to u.

// Poll on sensors, assign to proper variables (flip new_sensors to true).


// Poll on flowboard, assign to proper variables (flip new_flow to true).


// Assign values to correct places in the z vector.


// Run an EKF step.


// Check if we have gone long enough without updating to update.


// Outputs should be in proper NED already.