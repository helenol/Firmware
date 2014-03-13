/****************************************************************************
 *
 *   Copyright (C) 2012 PX4 Development Team. All rights reserved.
 *   Author: Lorenz Meier <lm@inf.ethz.ch>
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
 * @file orb_listener.c
 * Monitors ORB topics and sends update messages as appropriate.
 *
 * @author Lorenz Meier <lm@inf.ethz.ch>
 */

// XXX trim includes
#include <nuttx/config.h>
#include <stdio.h>
#include <math.h>
#include <stdbool.h>
#include <fcntl.h>
#include <string.h>
#include <drivers/drv_hrt.h>
#include <time.h>
#include <float.h>
#include <unistd.h>
#include <sys/prctl.h>
#include <stdlib.h>
#include <poll.h>
#include <lib/geo/geo.h>

#include <mavlink/mavlink_log.h>

#include "mavlink_bridge_header.h"
#include "orb_topics.h"
#include "util.h"

extern bool gcs_link;

struct vehicle_global_position_s global_pos;
struct vehicle_local_position_s local_pos;
struct home_position_s home;
struct navigation_capabilities_s nav_cap;
struct vehicle_status_s v_status;
struct position_setpoint_triplet_s pos_sp_triplet;
struct rc_channels_s rc;
struct rc_input_values rc_raw;
struct actuator_armed_s armed;
struct actuator_controls_s actuators_0;
struct vehicle_attitude_s att;
struct airspeed_s airspeed;
struct vehicle_vicon_position_s vicon_position;

struct mavlink_subscriptions mavlink_subs;

static int status_sub;
static int rc_sub;

static unsigned int sensors_raw_counter;
static unsigned int attitude_counter;
static unsigned int gps_counter;

/*
 * Last sensor loop time
 * some outputs are better timestamped
 * with this "global" reference.
 */
static uint64_t last_sensor_timestamp;

static hrt_abstime last_sent_vfr = 0;

static void	*uorb_receive_thread(void *arg);

struct listener {
	void	(*callback)(const struct listener *l);
	int		*subp;
	uintptr_t	arg;
};

uint16_t cm_uint16_from_m_float(float m);

static void	l_vehicle_attitude(const struct listener *l);
static void l_vicon_position(const struct listener *l);

static const struct listener listeners[] = {
	{l_vehicle_attitude,		&mavlink_subs.att_sub,		0},
    {l_vicon_position,         &mavlink_subs.vicon_position_sub,      0},
};

static const unsigned n_listeners = sizeof(listeners) / sizeof(listeners[0]);

uint16_t
cm_uint16_from_m_float(float m)
{
	if (m < 0.0f) {
		return 0;

	} else if (m > 655.35f) {
		return 65535;
	}

	return (uint16_t)(m * 100.0f);
}

void
l_vicon_position(const struct listener *l)
{
    /* copy local position data into local buffer */
    orb_copy(ORB_ID(vehicle_vicon_position), mavlink_subs.vicon_position_sub, &vicon_position);

    if (gcs_link)
        mavlink_msg_vicon_position_estimate_send(MAVLINK_COMM_0,
                            vicon_position.timestamp / 1000,
                            vicon_position.x,
                            vicon_position.y,
                            vicon_position.z,
                            vicon_position.roll,
                            vicon_position.pitch,
                            vicon_position.yaw);
}

void
l_vehicle_attitude(const struct listener *l)
{
	/* copy attitude data into local buffer */
	orb_copy(ORB_ID(vehicle_attitude), mavlink_subs.att_sub, &att);

	if (gcs_link) {
		/* send sensor values */
		mavlink_msg_attitude_send(MAVLINK_COMM_0,
					  last_sensor_timestamp / 1000,
					  att.roll,
					  att.pitch,
					  att.yaw,
					  att.rollspeed,
					  att.pitchspeed,
					  att.yawspeed);
					  	
		/* limit VFR message rate to 10Hz */
		hrt_abstime t = hrt_absolute_time();
		if (t >= last_sent_vfr + 100000) {
			last_sent_vfr = t;
			float groundspeed = sqrtf(global_pos.vel_n * global_pos.vel_n + global_pos.vel_e * global_pos.vel_e);
			uint16_t heading = _wrap_2pi(att.yaw) * M_RAD_TO_DEG_F;
			float throttle = armed.armed ? actuators_0.control[3] * 100.0f : 0.0f;
			mavlink_msg_vfr_hud_send(MAVLINK_COMM_0, airspeed.true_airspeed_m_s, groundspeed, heading, throttle, global_pos.alt, -global_pos.vel_d);
		}
		
		/* send quaternion values if it exists */
		if(att.q_valid) {
			mavlink_msg_attitude_quaternion_send(MAVLINK_COMM_0,
												last_sensor_timestamp / 1000,
												att.q[0],
												att.q[1],
												att.q[2],
												att.q[3],
												att.rollspeed,
												att.pitchspeed,
												att.yawspeed);
		}
	}

	attitude_counter++;
}

static void *
uorb_receive_thread(void *arg)
{
	/* Set thread name */
	prctl(PR_SET_NAME, "mavlink_android_orb_rcv", getpid());

	/*
	 * set up poll to block for new data,
	 * wait for a maximum of 1000 ms (1 second)
	 */
	const int timeout = 1000;

	/*
	 * Initialise listener array.
	 *
	 * Might want to invoke each listener once to set initial state.
	 */
	struct pollfd fds[n_listeners];

	for (unsigned i = 0; i < n_listeners; i++) {
		fds[i].fd = *listeners[i].subp;
		fds[i].events = POLLIN;

		/* Invoke callback to set initial state */
		//listeners[i].callback(&listener[i]);
	}

	while (!thread_should_exit) {

		int poll_ret = poll(fds, n_listeners, timeout);

		/* handle the poll result */
		if (poll_ret == 0) {
			/* silent */

		} else if (poll_ret < 0) {
			//mavlink_missionlib_send_gcs_string("[mavlink] ERROR reading uORB data");

		} else {

			for (unsigned i = 0; i < n_listeners; i++) {
				if (fds[i].revents & POLLIN)
					listeners[i].callback(&listeners[i]);
			}
		}
	}

	return NULL;
}

pthread_t
uorb_receive_start(void)
{
	/* --- ATTITUDE VALUE --- */
	mavlink_subs.att_sub = orb_subscribe(ORB_ID(vehicle_attitude));
	/* rate limit set externally based on interface speed, set a basic default here */
	orb_set_interval(mavlink_subs.att_sub, 100);	/* 100Hz updates */

    /* --- VICON POS VALUE --- */
    mavlink_subs.vicon_position_sub = orb_subscribe(ORB_ID(vehicle_vicon_position));
    orb_set_interval(mavlink_subs.vicon_position_sub, 100); /* 100 Hz active updates */

	/* --- LOCAL POS VALUE --- */
	//mavlink_subs.local_pos_sub = orb_subscribe(ORB_ID(vehicle_local_position));
	//orb_set_interval(mavlink_subs.local_pos_sub, 100);	/* 10Hz active updates */

	/* start the listener loop */
	pthread_attr_t uorb_attr;
	pthread_attr_init(&uorb_attr);

	/* Set stack size, needs less than 2k */
	pthread_attr_setstacksize(&uorb_attr, 2048);

	pthread_t thread;
	pthread_create(&thread, &uorb_attr, uorb_receive_thread, NULL);

	pthread_attr_destroy(&uorb_attr);
	return thread;
}
