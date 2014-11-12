/****************************************************************************
 *
 *   Copyright (c) 2013, 2014 PX4 Development Team. All rights reserved.
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
 * @file heli_pos_control_main.cpp
 * Helicopter position controller.
 *
 * @author Caelan Midwood <c.e.midwood@gmail.com>
 */

#include <nuttx/config.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <fcntl.h>
#include <errno.h>
#include <math.h>
#include <poll.h>
#include <drivers/drv_hrt.h>
#include <arch/board/board.h>
#include <uORB/uORB.h>
#include <uORB/topics/vehicle_attitude_setpoint.h>
#include <uORB/topics/manual_control_setpoint.h>
#include <uORB/topics/actuator_controls.h>
#include <uORB/topics/vehicle_rates_setpoint.h>
#include <uORB/topics/vehicle_attitude.h>
#include <uORB/topics/vehicle_control_mode.h>
#include <uORB/topics/actuator_armed.h>
#include <uORB/topics/parameter_update.h>
#include <uORB/topics/vehicle_local_position.h>
#include <uORB/topics/position_setpoint_triplet.h>
#include <uORB/topics/vehicle_global_velocity_setpoint.h>
#include <uORB/topics/vehicle_local_position_setpoint.h>
#include <systemlib/param/param.h>
#include <systemlib/err.h>
#include <systemlib/systemlib.h>
#include <mathlib/mathlib.h>
#include <lib/geo/geo.h>
#include <mavlink/mavlink_log.h>

/**
 * Helicopter position control app start / stop handling function
 *
 * @ingroup apps
 */
extern "C" __EXPORT int heli_pos_control_main(int argc, char *argv[]);

class HelicopterPositionControl
{
public:
    /**
     * Constructor
     */
    HelicopterPositionControl();
    
    /**
     * Destructor, also kills task.
     */
    ~HelicopterPositionControl();
    
    /**
     * Start task.
     *
     * @return		OK on success.
     */
    int		start();

}

HelicopterPositionControl::~HelicopterPositionControl()
{
    if (_control_task != -1) {
        /* task wakes up every 100ms or so at the longest */
        _task_should_exit = true;
        
        /* wait for a second for the task to quit at our request */
        unsigned i = 0;
        
        do {
            /* wait 20ms */
            usleep(20000);
            
            /* if we have given up, kill it */
            if (++i > 50) {
                task_delete(_control_task);
                break;
            }
        } while (_control_task != -1);
    }
    
    pos_control::g_control = nullptr;
}

void
HelicopterPositionControl::task_main()
{
    
}

int
HelicopterPositionControl::start()
{
    ASSERT(_control_task == -1);
    
    /* start the task */
    _control_task = task_spawn_cmd("heli_pos_control",
                                   SCHED_DEFAULT,
                                   SCHED_PRIORITY_MAX - 5,
                                   2000,
                                   (main_t)&MHelicopterPositionControl::task_main_trampoline,
                                   nullptr);
    
    if (_control_task < 0) {
        warn("task start failed");
        return -errno;
    }
    
    return OK;
}


int heli_pos_control_main(int argc, char *argv[])
{
    if (argc < 1) {
        errx(1, "usage: heli_pos_control {start|stop|status}");
    }
    
    if (!strcmp(argv[1], "start")) {
        
        if (pos_control::g_control != nullptr) {
            errx(1, "already running");
        }
        
        pos_control::g_control = new HelicopterPositionControl;
        
        if (pos_control::g_control == nullptr) {
            errx(1, "alloc failed");
        }
        
        if (OK != pos_control::g_control->start()) {
            delete pos_control::g_control;
            pos_control::g_control = nullptr;
            err(1, "start failed");
        }
        
        exit(0);
    }
    
    if (!strcmp(argv[1], "stop")) {
        if (pos_control::g_control == nullptr) {
            errx(1, "not running");
        }
        
        delete pos_control::g_control;
        pos_control::g_control = nullptr;
        exit(0);
    }
    
    if (!strcmp(argv[1], "status")) {
        if (pos_control::g_control) {
            errx(0, "running");
            
        } else {
            errx(1, "not running");
        }
    }
    
    warnx("unrecognized command");
    return 1;
}

