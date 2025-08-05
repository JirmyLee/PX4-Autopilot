/****************************************************************************
 *
 *   Copyright (c) 2013-2020 Estimation and Control Library (ECL). All rights reserved.
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
 * 3. Neither the name ECL nor the names of its contributors may be
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
 * @file ecl_pitch_controller.cpp
 * Implementation of a simple orthogonal pitch PID controller.
 *
 * Authors and acknowledgements in header.
 */

#include "ecl_pitch_controller.h"
#include <float.h>
#include <lib/geo/geo.h>
#include <mathlib/mathlib.h>

//一个简单的 P（Proportional）控制器来控制飞机的俯仰姿态。
float ECL_PitchController::control_attitude(const float dt, const ECL_ControlData &ctl_data)
{
	/* Do not calculate control signal with bad inputs */
	if (!(PX4_ISFINITE(ctl_data.pitch_setpoint) &&
	      PX4_ISFINITE(ctl_data.roll) &&
	      PX4_ISFINITE(ctl_data.pitch) &&
	      PX4_ISFINITE(ctl_data.euler_yaw_rate_setpoint))) {

		return _body_rate_setpoint;
	}

	/* Calculate the error */	//代码计算俯仰角的误差 pitch_error，即期望俯仰角和当前俯仰角之间的差值。
	float pitch_error = ctl_data.pitch_setpoint - ctl_data.pitch;

	/*  Apply P controller: rate setpoint from current error and time constant */
	//通过 P 控制器来生成角速度目标值 _euler_rate_setpoint，计算公式为： _euler_rate_setpoint = pitch_error / _tc。其中，_tc 是控制器的时间常数，它决定了控制器的响应速度。
	_euler_rate_setpoint =  pitch_error / _tc;

	/* Transform setpoint to body angular rates (jacobian) */
	//内环角速度控制：将期望值（设定值）转换为机体角速度（雅可比矩阵），具体做法：根据飞机的当前横滚角 ctl_data.roll 和俯仰角 ctl_data.pitch，
	//以及偏航角速度目标值 ctl_data.euler_yaw_rate_setpoint，来计算在飞机体坐标系下的俯仰角速度目标值 pitch_body_rate_setpoint_raw。
	const float pitch_body_rate_setpoint_raw = cosf(ctl_data.roll) * _euler_rate_setpoint +
			cosf(ctl_data.pitch) * sinf(ctl_data.roll) * ctl_data.euler_yaw_rate_setpoint;
	//限制角度范围
	_body_rate_setpoint = math::constrain(pitch_body_rate_setpoint_raw, -_max_rate_neg, _max_rate);

	return _body_rate_setpoint;
}
