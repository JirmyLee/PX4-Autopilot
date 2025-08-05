/****************************************************************************
 *
 *   Copyright (c) 2013-2016 Estimation and Control Library (ECL). All rights reserved.
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
 * @file ecl_wheel_controller.cpp
 *
 *
 * Authors and acknowledgements in header.
 */

//实现用于航向跟踪的简单 PID轮式角速率控制器。
#include "ecl_wheel_controller.h"
#include <float.h>
#include <lib/geo/geo.h>
#include <mathlib/mathlib.h>
#include <matrix/math.hpp>

using matrix::wrap_pi;

//内环角速率控制器，实现一个基于比例积分（PI）控制的纵向角速率控制器，用于将输入的控制数据转化为对飞行器的垂直角速率控制信号，从而实现纵向稳定控制。
float ECL_WheelController::control_bodyrate(const float dt, const ECL_ControlData &ctl_data)
{
	/* Do not calculate control signal with bad inputs */
	if (!(PX4_ISFINITE(ctl_data.body_z_rate) &&
	      PX4_ISFINITE(ctl_data.groundspeed) &&
	      PX4_ISFINITE(ctl_data.groundspeed_scaler))) {

		return math::constrain(_last_output, -1.0f, 1.0f);
	}

	/* input conditioning */
	float min_speed = 1.0f;

	/* Calculate body angular rate error */
	//期望角速率-实际角速率=角速度差
	const float rate_error = _body_rate_setpoint - ctl_data.body_z_rate; //body angular rate error

	if (_k_i > 0.0f && ctl_data.groundspeed > min_speed) {

		float id = rate_error * dt * ctl_data.groundspeed_scaler;

		/*
		 * anti-windup: do not allow integrator to increase if actuator is at limit
		 */
		//抗积分饱和，负向饱和就只加正积分，正向饱和就只加负积分
		if (_last_output < -1.0f) {
			/* only allow motion to center: increase value */
			id = math::max(id, 0.0f);

		} else if (_last_output > 1.0f) {
			/* only allow motion to center: decrease value */
			id = math::min(id, 0.0f);
		}

		/* add and constrain */
		//将积分增量乘系数_k_i加到积分项，_k_i为参数FW_RR_I
		_integrator = math::constrain(_integrator + id * _k_i, -_integrator_max, _integrator_max);
	}

	/* Apply PI rate controller and store non-limited output */
	//PI+前馈控制FF得到最终角加速度输出，这个输出就是给混控器的。缩放系数同上，前馈系数_k_ff对应参数FW_RR_FF，比例系数_k_p对应参数FW_RR_P。
	//角速率控制信号计算包括三部分：
	//	前馈项：使用 _body_rate_setpoint、前馈增益 _k_ff 和地速缩放因子计算。
	//	比例项和积分项：使用角速率误差 rate_error、比例增益 _k_p 和积分项 _integrator 进行计算，同时考虑地速缩放因子的平方。
	_last_output = _body_rate_setpoint * _k_ff * ctl_data.groundspeed_scaler +
		       ctl_data.groundspeed_scaler * ctl_data.groundspeed_scaler * (rate_error * _k_p + _integrator);

	return math::constrain(_last_output, -1.0f, 1.0f);
}

//姿态角控制器
float ECL_WheelController::control_attitude(const float dt, const ECL_ControlData &ctl_data)
{
	/* Do not calculate control signal with bad inputs */
	if (!(PX4_ISFINITE(ctl_data.yaw_setpoint) &&
	      PX4_ISFINITE(ctl_data.yaw))) {

		return _body_rate_setpoint;
	}

	//计算期望的偏航角误差 yaw_error，并将其进行正负 PI 角度的规范化。
	/* Calculate the error */
	float yaw_error = wrap_pi(ctl_data.yaw_setpoint - ctl_data.yaw);

	//应用P控制器来生成期望的角速率设定值
	/*  Apply P controller: rate setpoint from current error and time constant */
	_euler_rate_setpoint =  yaw_error / _tc;
	_body_rate_setpoint = _euler_rate_setpoint; // assume 0 pitch and roll angle, thus jacobian is simply identity matrix

	/* limit the rate */
	if (_max_rate > 0.01f) {
		if (_body_rate_setpoint > 0.0f) {
			_body_rate_setpoint = (_body_rate_setpoint > _max_rate) ? _max_rate : _body_rate_setpoint;

		} else {
			_body_rate_setpoint = (_body_rate_setpoint < -_max_rate) ? -_max_rate : _body_rate_setpoint;
		}

	}

	return _body_rate_setpoint;
}
