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
 * @file ecl_roll_controller.cpp
 * Implementation of a simple orthogonal roll PID controller.
 *
 * Authors and acknowledgements in header.
 */

#include "ecl_roll_controller.h"
#include <float.h>
#include <lib/geo/geo.h>
#include <mathlib/mathlib.h>

//外环姿态控制(简单正交横滚PID 控制器的实现)
float ECL_RollController::control_attitude(const float dt, const ECL_ControlData &ctl_data)
{
	//如果值异常，返回上一次的结果
	/* Do not calculate control signal with bad inputs */
	if (!(PX4_ISFINITE(ctl_data.roll_setpoint) &&
	      PX4_ISFINITE(ctl_data.euler_yaw_rate_setpoint) &&
	      PX4_ISFINITE(ctl_data.pitch) &&
	      PX4_ISFINITE(ctl_data.roll))) {

		return _body_rate_setpoint;
	}

	//求roll姿态误差
	/* Calculate the error */
	float roll_error = ctl_data.roll_setpoint - ctl_data.roll;

	//外环P控制，_tc为时间常数，可在地面站参数列表中设置PID调参中的FW_R_TC参数（赋值流程：（1）先找到参数_parameter_handles.p_tc = param_find(“FW_P_TC”);（2）之后赋值param_get(_parameter_handles.p_tc, &(_parameters.p_tc));（3）经过函数set_time_constant赋值给_tc：_tc = time_constant;
	/*  Apply P controller: rate setpoint from current error and time constant */
	_euler_rate_setpoint = roll_error / _tc;

	//内环角速度控制：将将地理系下的期望值（设定值）转换为机体坐标系下的角速度（雅可比矩阵），具体为：根据飞机的当前俯仰角 ctl_data.pitch，以及偏航角速度目标值 ctl_data.euler_yaw_rate_setpoint，
	//来计算在机体坐标系下的横滚速度目标值roll_body_rate_setpoint_raw
	//姿态控制的当前姿态和期望姿态都是基于地理坐标系的，即正北为航向正方向，地理的水平面为横滚的0度。实际控制的时候都是作用于机体进行控制的，
	//所以将地理系下期望的角速度，转换为机体坐标系下的期望角速度，然后调用内环角速度控制
	/* Transform setpoint to body angular rates (jacobian) */
	const float roll_body_rate_setpoint_raw = _euler_rate_setpoint - sinf(ctl_data.pitch) *
			ctl_data.euler_yaw_rate_setpoint;
	_body_rate_setpoint = math::constrain(roll_body_rate_setpoint_raw, -_max_rate, _max_rate);

	return _body_rate_setpoint;
}

