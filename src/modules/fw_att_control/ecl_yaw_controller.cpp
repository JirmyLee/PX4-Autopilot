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
 * @file ecl_yaw_controller.cpp
 * Implementation of a simple orthogonal coordinated turn yaw PID controller.
 *
 * Authors and acknowledgements in header.
 */

#include "ecl_yaw_controller.h"
#include <float.h>
#include <lib/geo/geo.h>
#include <mathlib/mathlib.h>

//一种简单的正交协调转弯-偏航PID控制器的实现
float ECL_YawController::control_attitude(const float dt, const ECL_ControlData &ctl_data)
{
	//1.判断值是否可用
	/* Do not calculate control signal with bad inputs */
	if (!(PX4_ISFINITE(ctl_data.roll) &&
	      PX4_ISFINITE(ctl_data.pitch) &&
	      PX4_ISFINITE(ctl_data.euler_pitch_rate_setpoint) &&
	      PX4_ISFINITE(ctl_data.airspeed_constrained))) {

		return _body_rate_setpoint;
	}

	float constrained_roll;
	bool inverted = false;	//是否倒飞

	//2.横滚用作前馈项，需要考虑飞行器是否反向飞行，对横滚角进行限幅
	/* roll is used as feedforward term and inverted flight needs to be considered */
	if (fabsf(ctl_data.roll) < math::radians(90.0f)) {
		/* not inverted, but numerically still potentially close to infinity */
		constrained_roll = math::constrain(ctl_data.roll, math::radians(-80.0f), math::radians(80.0f));

	} else {
		//横滚角超过90度为倒飞。此时需要根据不同的半球（左半球或右半球）来计算限制的滚转角度。
		inverted = true;

		// inverted flight, constrain on the two extremes of -pi..+pi to avoid infinity
		//note: the ranges are extended by 10 deg here to avoid numeric resolution effects
		if (ctl_data.roll > 0.0f) {
			/* right hemisphere */
			constrained_roll = math::constrain(ctl_data.roll, math::radians(100.0f), math::radians(180.0f));

		} else {
			/* left hemisphere */
			constrained_roll = math::constrain(ctl_data.roll, math::radians(-180.0f), math::radians(-100.0f));
		}
	}

	constrained_roll = math::constrain(constrained_roll, -fabsf(ctl_data.roll_setpoint), fabsf(ctl_data.roll_setpoint));


	if (!inverted) {
		//3.没倒飞的情况，根据协调转弯方程的约束和无侧向力的原则，通过横滚角和俯仰角计算所需的偏航角速率 _euler_rate_setpoint
		/* Calculate desired yaw rate from coordinated turn constraint / (no side forces) */
		_euler_rate_setpoint = tanf(constrained_roll) * cosf(ctl_data.pitch) * CONSTANTS_ONE_G / ctl_data.airspeed_constrained;

		//4.将偏航角速率设定值从地理坐标系转换到机体角速率（雅可比矩阵）
		/* Transform setpoint to body angular rates (jacobian) */
		const float yaw_body_rate_setpoint_raw = -sinf(ctl_data.roll) * ctl_data.euler_pitch_rate_setpoint +
				cosf(ctl_data.roll) * cosf(ctl_data.pitch) * _euler_rate_setpoint;
		_body_rate_setpoint = math::constrain(yaw_body_rate_setpoint_raw, -_max_rate, _max_rate);
	}

	//如果计算得到的机体角速率不是有限数，发出一个警告，并将机体角速率设为0。
	if (!PX4_ISFINITE(_body_rate_setpoint)) {
		PX4_WARN("yaw rate sepoint not finite");
		_body_rate_setpoint = 0.0f;
	}

	return _body_rate_setpoint;
}
