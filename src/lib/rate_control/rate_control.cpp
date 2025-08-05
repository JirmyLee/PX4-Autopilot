/****************************************************************************
 *
 *   Copyright (c) 2019 PX4 Development Team. All rights reserved.
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
 * @file RateControl.cpp
 */

#include "rate_control.hpp"
#include <px4_platform_common/defines.h>

using namespace matrix;

void RateControl::setGains(const Vector3f &P, const Vector3f &I, const Vector3f &D)
{
	_gain_p = P;
	_gain_i = I;
	_gain_d = D;
}

void RateControl::setSaturationStatus(const Vector<bool, 3> &saturation_positive,
				      const Vector<bool, 3> &saturation_negative)
{
	_control_allocator_saturation_positive = saturation_positive;
	_control_allocator_saturation_negative = saturation_negative;
}


//更新角速率控制器，计算并返回所需的控制扭矩。（rate是当前飞行器的角速率；rate_sp是期望的目标角速率；angular_accel是角加速度；dt是时间步长；landed是一个布尔值，表示飞行器是否已经着陆。）
Vector3f RateControl::update(const Vector3f &rate, const Vector3f &rate_sp, const Vector3f &angular_accel,
			     const float dt, const bool landed)
{
	//角速率误差
	// angular rates error
	Vector3f rate_error = rate_sp - rate;

	//前馈PID控制，这里_gain_p、_gain_d、_gain_ff对应PID控制参数
	// PID control with feed forward
	const Vector3f torque = _gain_p.emult(rate_error) + _rate_int - _gain_d.emult(angular_accel) + _gain_ff.emult(rate_sp);

	//没着陆时更新积分
	// update integral only if we are not landed
	if (!landed) {
		updateIntegral(rate_error, dt);
	}

	return torque;
}

//角速率控制器中的积分项更新
void RateControl::updateIntegral(Vector3f &rate_error, const float dt)
{
	//for 循环迭代三次，分别对应飞行器绕三个轴（滚转、俯仰、偏航）的控制。
	for (int i = 0; i < 3; i++) {

		//检查控制输出是否饱和的函数。如果控制输出在正饱和区域内，会限制积分项的增加；如果控制输出在负饱和区域内，会限制积分项的减少。
		// prevent further positive control saturation
		if (_control_allocator_saturation_positive(i)) {
			rate_error(i) = math::min(rate_error(i), 0.f);
		}

		// prevent further negative control saturation
		if (_control_allocator_saturation_negative(i)) {
			rate_error(i) = math::max(rate_error(i), 0.f);
		}

		//i_factor 是积分项的缩放因子。它是基于角速率误差计算的，并且随着角速率误差的增大而减小。这是为了减轻积分项在控制目标发生大变化时的积分累积效应。
		// I term factor: reduce the I gain with increasing rate error.
		// This counteracts a non-linear effect where the integral builds up quickly upon a large setpoint
		// change (noticeable in a bounce-back effect after a flip).
		// The formula leads to a gradual decrease w/o steps, while only affecting the cases where it should:
		// with the parameter set to 400 degrees, up to 100 deg rate error, i_factor is almost 1 (having no effect),
		// and up to 200 deg error leads to <25% reduction of I.
		float i_factor = rate_error(i) / math::radians(400.f);
		i_factor = math::max(0.0f, 1.f - i_factor * i_factor);

		//rate_i 是通过积分算法计算得到的更新后的积分值。
		// Perform the integration using a first order method
		float rate_i = _rate_int(i) + i_factor * _gain_i(i) * rate_error(i) * dt;

		// do not propagate the result if out of range or invalid
		if (PX4_ISFINITE(rate_i)) {
			_rate_int(i) = math::constrain(rate_i, -_lim_int(i), _lim_int(i));
		}
	}
}

//获取角速率控制器的状态信息：updateIntegral()方法计算后的积分项
void RateControl::getRateControlStatus(rate_ctrl_status_s &rate_ctrl_status)
{
	rate_ctrl_status.rollspeed_integ = _rate_int(0);
	rate_ctrl_status.pitchspeed_integ = _rate_int(1);
	rate_ctrl_status.yawspeed_integ = _rate_int(2);
}
