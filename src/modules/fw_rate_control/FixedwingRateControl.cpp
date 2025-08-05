/****************************************************************************
 *
 *   Copyright (c) 2013-2023 PX4 Development Team. All rights reserved.
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

#include "FixedwingRateControl.hpp"

using namespace time_literals;
using namespace matrix;

using math::constrain;
using math::interpolate;
using math::radians;

FixedwingRateControl::FixedwingRateControl(bool vtol) :
	ModuleParams(nullptr),
	ScheduledWorkItem(MODULE_NAME, px4::wq_configurations::nav_and_controllers),
	_actuator_controls_status_pub(vtol ? ORB_ID(actuator_controls_status_1) : ORB_ID(actuator_controls_status_0)),
	_vehicle_torque_setpoint_pub(vtol ? ORB_ID(vehicle_torque_setpoint_virtual_fw) : ORB_ID(vehicle_torque_setpoint)),
	_vehicle_thrust_setpoint_pub(vtol ? ORB_ID(vehicle_thrust_setpoint_virtual_fw) : ORB_ID(vehicle_thrust_setpoint)),
	_loop_perf(perf_alloc(PC_ELAPSED, MODULE_NAME": cycle"))
{
	/* fetch initial parameter values */
	parameters_update();

	_rate_ctrl_status_pub.advertise();
}

FixedwingRateControl::~FixedwingRateControl()
{
	perf_free(_loop_perf);
}

bool
FixedwingRateControl::init()
{
	if (!_vehicle_angular_velocity_sub.registerCallback()) {
		PX4_ERR("callback registration failed");
		return false;
	}

	return true;
}

int
FixedwingRateControl::parameters_update()
{
	const Vector3f rate_p = Vector3f(_param_fw_rr_p.get(), _param_fw_pr_p.get(), _param_fw_yr_p.get());
	const Vector3f rate_i = Vector3f(_param_fw_rr_i.get(), _param_fw_pr_i.get(), _param_fw_yr_i.get());
	const Vector3f rate_d = Vector3f(_param_fw_rr_d.get(), _param_fw_pr_d.get(), _param_fw_yr_d.get());

	_rate_control.setGains(rate_p, rate_i, rate_d);	//PID调参中的FW_RR_P、FW_RR_I参数

	//PID调参中的FW_RR_IMAX参数
	_rate_control.setIntegratorLimit(
		Vector3f(_param_fw_rr_imax.get(), _param_fw_pr_imax.get(), _param_fw_yr_imax.get()));

	_rate_control.setFeedForwardGain(
		// set FF gains to 0 as we add the FF control outside of the rate controller
		Vector3f(0.f, 0.f, 0.f));


	return PX4_OK;
}

//处理手动控制输入,根据手动控制输入生成对应的控制指令，实现飞行器的手动操控(在角速率控制模式下，它生成期望的角速率设定值；在手动/direct控制模式下，它生成推力和扭矩设定值)
void
FixedwingRateControl::vehicle_manual_poll()
{
	//启用了手动控制模式，并且当前飞行模式是固定翼模式或过渡模式
	if (_vcontrol_mode.flag_control_manual_enabled && _in_fw_or_transition_wo_tailsitter_transition) {

		// Always copy the new manual setpoint, even if it wasn't updated, to fill the actuators with valid values
		if (_manual_control_setpoint_sub.copy(&_manual_control_setpoint)) {

			//如果启用了角速率控制模式（RATE mode），并且没有启用姿态控制模式(固定翼特技模式)，代码将根据手动控制输入生成期望的角速率设定值（_rates_sp）
			if (_vcontrol_mode.flag_control_rates_enabled &&
			    !_vcontrol_mode.flag_control_attitude_enabled) {

				// RATE mode we need to generate the rate setpoint from manual user inputs：在特技模式这种角速率控制模式下，摇杆对应的是期望的角速率

				if (_vehicle_status.is_vtol_tailsitter && _vehicle_status.vehicle_type == vehicle_status_s::VEHICLE_TYPE_FIXED_WING) {
					//如果是VTOL垂直起降机型且当前飞行模式是固定翼模式，将摇杆的Yaw输入映射到Roll的角速率设定值，将摇杆的Roll输入映射到Yaw的角速率设定值
					// the rate_sp must always be published in body (hover) frame
					_rates_sp.roll = _manual_control_setpoint.yaw * radians(_param_fw_acro_z_max.get());
					_rates_sp.yaw = -_manual_control_setpoint.roll * radians(_param_fw_acro_x_max.get());

				} else {
					//将摇杆的Roll输入映射到特技模式下Roll的角速率设定值，将摇杆的Yaw输入映射到Yaw的角速率设定值
					_rates_sp.roll = _manual_control_setpoint.roll * radians(_param_fw_acro_x_max.get());	//FW_ACRO_X_MAX为特技模式下机体轴最大速率

					_rates_sp.yaw = _manual_control_setpoint.yaw * radians(_param_fw_acro_z_max.get());
				}

				_rates_sp.timestamp = hrt_absolute_time();
				//Pitch 输入映射到 Pitch 的角速率设定值，油门输入映射到油门推力设定值
				_rates_sp.pitch = -_manual_control_setpoint.pitch * radians(_param_fw_acro_y_max.get());
				_rates_sp.thrust_body[0] = (_manual_control_setpoint.throttle + 1.f) * .5f;

				_rate_sp_pub.publish(_rates_sp);

			} else {
				//手动/direct控制模式，代码将根据手动控制输入生成对应的推力_vehicle_thrust_setpoint和扭矩设定值_vehicle_torque_setpoint
				// Manual/direct control, filled in FW-frame. Note that setpoints will get transformed to body frame prior publishing.

				_vehicle_torque_setpoint.xyz[0] = math::constrain(_manual_control_setpoint.roll * _param_fw_man_r_sc.get() +
								  _param_trim_roll.get(), -1.f, 1.f);	//手动控制输入的Roll映射到Roll扭矩设定值，乘以参数FW_MAN_R_SC
				_vehicle_torque_setpoint.xyz[1] = math::constrain(-_manual_control_setpoint.pitch * _param_fw_man_p_sc.get() +
								  _param_trim_pitch.get(), -1.f, 1.f);	//手动控制输入的Pitch映射到Pitch扭矩设定值，乘以参数FW_MAN_P_SC
				_vehicle_torque_setpoint.xyz[2] = math::constrain(_manual_control_setpoint.yaw * _param_fw_man_y_sc.get() +
								  _param_trim_yaw.get(), -1.f, 1.f);	//手动控制输入的Yaw映射到Yaw扭矩设定值，乘以参数FW_MAN_P_SC

				_vehicle_thrust_setpoint.xyz[0] = math::constrain((_manual_control_setpoint.throttle + 1.f) * .5f, 0.f, 1.f);	//油门输入映射到油门推力设定值，范围限制在0到1之间
			}
		}
	}
}

void
FixedwingRateControl::vehicle_land_detected_poll()
{
	if (_vehicle_land_detected_sub.updated()) {
		vehicle_land_detected_s vehicle_land_detected {};

		if (_vehicle_land_detected_sub.copy(&vehicle_land_detected)) {
			_landed = vehicle_land_detected.landed;
		}
	}
}

//根据空速计算控制器的缩放系数
float FixedwingRateControl::get_airspeed_and_update_scaling()
{
	//判断空速是否可用
	_airspeed_validated_sub.update();
	const bool airspeed_valid = PX4_ISFINITE(_airspeed_validated_sub.get().calibrated_airspeed_m_s)
				    && (hrt_elapsed_time(&_airspeed_validated_sub.get().timestamp) < 1_s);

	//如果空速不可用，就采用固定的巡航配平空速参数FW_AIRSPD_TRIM
	// if no airspeed measurement is available out best guess is to use the trim airspeed
	float airspeed = _param_fw_airspd_trim.get();

	if ((_param_fw_arsp_mode.get() == 0) && airspeed_valid) {
		//如果空速可用，订阅空速数据
		/* prevent numerical drama by requiring 0.5 m/s minimal speed */
		airspeed = math::max(0.5f, _airspeed_validated_sub.get().calibrated_airspeed_m_s);

	} else {
		//如果空速数据不可用，并且载具为垂起并在多旋翼模式，将空速设置为失速空速参数FW_AIRSPD_STALL
		// VTOL: if we have no airspeed available and we are in hover mode then assume the lowest airspeed possible
		// this assumption is good as long as the vehicle is not hovering in a headwind which is much larger
		// than the stall airspeed
		if (_vehicle_status.is_vtol && _vehicle_status.vehicle_type == vehicle_status_s::VEHICLE_TYPE_ROTARY_WING
		    && !_vehicle_status.in_transition_mode) {
			airspeed = _param_fw_airspd_stall.get();
		}
	}

	/*
	 * For scaling our actuators using anything less than the stall
	 * speed doesn't make any sense - its the strongest reasonable deflection we
	 * want to do in flight and it's the baseline a human pilot would choose.
	 *
	 * Forcing the scaling to this value allows reasonable handheld tests.
	 */
	//限幅在失速空速和最大空速之间
	const float airspeed_constrained = constrain(constrain(airspeed, _param_fw_airspd_stall.get(),
					   _param_fw_airspd_max.get()), 0.1f, 1000.0f);

	//根据空速计算缩放值，这个缩放值在上面的姿态控制中有用到。因为空速越大，同样角度的舵面产生的力矩越大。
	_airspeed_scaling = (_param_fw_arsp_scale_en.get()) ? (_param_fw_airspd_trim.get() / airspeed_constrained) : 1.0f;

	return airspeed;
}

//执行速率控制逻辑
void FixedwingRateControl::Run()
{
	//判断是否需要退出速率控制模块，如果需要退出，则取消回调函数注册并进行清理。
	if (should_exit()) {
		_vehicle_angular_velocity_sub.unregisterCallback();
		exit_and_cleanup();
		return;
	}

	perf_begin(_loop_perf);	//开始性能计数

	//当飞行器的角速度数据更新或者距离上次运行速率控制逻辑已经过去超过20毫秒时，执行速率控制逻辑。
	// only run controller if angular velocity changed
	if (_vehicle_angular_velocity_sub.updated() || (hrt_elapsed_time(&_last_run) > 20_ms)) { //TODO rate!

		// only update parameters if they changed
		bool params_updated = _parameter_update_sub.updated();

		//更新参数
		// check for parameter updates
		if (params_updated) {
			// clear update
			parameter_update_s pupdate;
			_parameter_update_sub.copy(&pupdate);

			// update parameters from storage
			updateParams();
			parameters_update();
		}

		//1.从角速度数据中获取角速度和角加速度信息，并根据飞行器是否为VTOL机型，将角速度数据转换到固定翼姿态控制器的参考坐标系。
		float dt = 0.f;

		static constexpr float DT_MIN = 0.002f;
		static constexpr float DT_MAX = 0.04f;

		vehicle_angular_velocity_s vehicle_angular_velocity{};

		//从_vehicle_angular_velocity_sub订阅到数据，则更新dt的值（计算当前时间戳与上次运行时间戳之差并乘以1e-6f得到时间间隔dt，并将其限制在DT_MIN和DT_MAX之间。同时更新_last_run为当前时间戳。）
		if (_vehicle_angular_velocity_sub.copy(&vehicle_angular_velocity)) {
			dt = math::constrain((vehicle_angular_velocity.timestamp_sample - _last_run) * 1e-6f, DT_MIN, DT_MAX);
			_last_run = vehicle_angular_velocity.timestamp_sample;
		}

		if (dt < DT_MIN || dt > DT_MAX) {
			const hrt_abstime time_now_us = hrt_absolute_time();
			dt = math::constrain((time_now_us - _last_run) * 1e-6f, DT_MIN, DT_MAX);
			_last_run = time_now_us;
		}

		//通过_vehicle_angular_velocity_sub订阅到数据并存储在angular_velocity中
		vehicle_angular_velocity_s angular_velocity{};
		_vehicle_angular_velocity_sub.copy(&angular_velocity);

		Vector3f rates(angular_velocity.xyz);			//rates存储角速度
		Vector3f angular_accel{angular_velocity.xyz_derivative};//angular_accel存储角加速度（速度对时间的导数=加速度）

		//如果飞机类型为VTOL，则将rates设置为(-z, y, x)，将angular_accel进行坐标轴旋转，设置为(-z', y', x')，以适应从悬停模式到固定翼模式的转换（控制器是在固定翼模式下工作，接口是在悬停模式下工作）
		// Tailsitter: rotate setpoint from hover to fixed-wing frame (controller is in fixed-wing frame, interface in hover)
		if (_vehicle_status.is_vtol_tailsitter) {
			rates = Vector3f(-angular_velocity.xyz[2], angular_velocity.xyz[1], angular_velocity.xyz[0]);
			angular_accel = Vector3f(-angular_velocity.xyz_derivative[2], angular_velocity.xyz_derivative[1],
						 angular_velocity.xyz_derivative[0]);
		}

		//2.更新飞行器状态数据，包括是否处于过渡模式（状态转换中）以及是否为固定翼飞行器。

		//更新订阅的飞行器状态_vehicle_status_sub和飞行器控制模式_vcontrol_mod，用于存储无人机的状态信息（这一步必须在_vehicle_control_mode_sub订阅更新之前进行，否则在整个转换过程中无法发布角速度设定点。）
		// vehicle status update must be before the vehicle_control_mode poll, otherwise rate sp are not published during whole transition
		_vehicle_status_sub.update(&_vehicle_status);
		const bool is_in_transition_except_tailsitter = _vehicle_status.in_transition_mode
				&& !_vehicle_status.is_vtol_tailsitter;
		const bool is_fixed_wing = _vehicle_status.vehicle_type == vehicle_status_s::VEHICLE_TYPE_FIXED_WING;
		_in_fw_or_transition_wo_tailsitter_transition =  is_fixed_wing || is_in_transition_except_tailsitter;

		//3.更新飞行器控制模式数据。
		_vehicle_control_mode_sub.update(&_vcontrol_mode);


		//4.更新飞机的降落检测
		vehicle_land_detected_poll();

		//处理手动控制输入,根据手动控制输入生成对应的控制指令
		vehicle_manual_poll();
		vehicle_land_detected_poll();

		/* if we are in rotary wing mode, do nothing */
		if (_vehicle_status.vehicle_type == vehicle_status_s::VEHICLE_TYPE_ROTARY_WING && !_vehicle_status.is_vtol) {
			perf_end(_loop_perf);
			return;
		}

		//如果启用了角速率控制
		if (_vcontrol_mode.flag_control_rates_enabled) {

			//获取空速信息并更新缩放系数。
			const float airspeed = get_airspeed_and_update_scaling();

			//需要时重置积分
			/* reset integrals where needed */
			if (_rates_sp.reset_integral) {
				_rate_control.resetIntegral();
			}

			//在地面时重置积分
			// Reset integrators if the aircraft is on ground or not in a state where the fw attitude controller is run
			if (_landed || !_in_fw_or_transition_wo_tailsitter_transition) {

				_rate_control.resetIntegral();
			}

			//根据控制分配反馈更新饱和状态
			// update saturation status from control allocation feedback
			control_allocator_status_s control_allocator_status;

			if (_control_allocator_status_subs[_vehicle_status.is_vtol ? 1 : 0].update(&control_allocator_status)) {
				Vector<bool, 3> saturation_positive;
				Vector<bool, 3> saturation_negative;

				if (!control_allocator_status.torque_setpoint_achieved) {
					for (size_t i = 0; i < 3; i++) {
						if (control_allocator_status.unallocated_torque[i] > FLT_EPSILON) {
							saturation_positive(i) = true;

						} else if (control_allocator_status.unallocated_torque[i] < -FLT_EPSILON) {
							saturation_negative(i) = true;
						}
					}
				}

				// TODO: send the unallocated value directly for better anti-windup
				_rate_control.setSaturationStatus(saturation_positive, saturation_negative);
			}

			//计算控制飞机沿巡航速度飞行所需的舵量配平值：根据参数_fw_airspd_trim设置进行舵面修正，计算roll、pitch、yaw的trim值。
			/* bi-linear interpolation over airspeed for actuator trim scheduling */
			float trim_roll = _param_trim_roll.get();
			float trim_pitch = _param_trim_pitch.get();
			float trim_yaw = _param_trim_yaw.get();

			//根据不同空速状态和控制模式，计算并调整飞机的滚转、俯仰和航向配平值，以保持飞行器的稳定
			if (airspeed < _param_fw_airspd_trim.get()) {
				//当空速小于巡航速度时，计算负的配平值，已使飞机能够提升速度
				trim_roll += interpolate(airspeed, _param_fw_airspd_min.get(), _param_fw_airspd_trim.get(),
							 _param_fw_dtrim_r_vmin.get(),
							 0.0f);
				trim_pitch += interpolate(airspeed, _param_fw_airspd_min.get(), _param_fw_airspd_trim.get(),
							  _param_fw_dtrim_p_vmin.get(),
							  0.0f);
				trim_yaw += interpolate(airspeed, _param_fw_airspd_min.get(), _param_fw_airspd_trim.get(),
							_param_fw_dtrim_y_vmin.get(),
							0.0f);

			} else {
				//当空速大于巡航速度时，计算正的配平值，已使飞机能够降低速度
				trim_roll += interpolate(airspeed, _param_fw_airspd_trim.get(), _param_fw_airspd_max.get(), 0.0f,
							 _param_fw_dtrim_r_vmax.get());
				trim_pitch += interpolate(airspeed, _param_fw_airspd_trim.get(), _param_fw_airspd_max.get(), 0.0f,
							  _param_fw_dtrim_p_vmax.get());
				trim_yaw += interpolate(airspeed, _param_fw_airspd_trim.get(), _param_fw_airspd_max.get(), 0.0f,
							_param_fw_dtrim_y_vmax.get());
			}

			/*
			 * 根据空速划分区间，并使用双线性插值计算roll、pitch、yaw的前馈量，并与角加速度相乘得到绝对扭矩（u）。
			 * 然后将绝对力矩加上配平值trim，并限制在-1到1之间，得到最终力矩设定值_vehicle_torque_setpoint.xyz[0]、[1]、[2]。
			 * 同时将_thrust_body[0]作为油门推力（若有有效输入），并按电池状态进行缩放调整。
			 */
			if (_vcontrol_mode.flag_control_rates_enabled) {
				_rates_sp_sub.update(&_rates_sp);

				Vector3f body_rates_setpoint = Vector3f(_rates_sp.roll, _rates_sp.pitch, _rates_sp.yaw);

				// Tailsitter: rotate setpoint from hover to fixed-wing frame (controller is in fixed-wing frame, interface in hover)
				if (_vehicle_status.is_vtol_tailsitter) {
					body_rates_setpoint = Vector3f(-_rates_sp.yaw, _rates_sp.pitch, _rates_sp.roll);
				}

				//内环角速率控制，根据速率控制器（前馈PID控制、积分项更新）计算的角加速度期望值，计算副翼、升降舵、方向舵的控制扭矩
				/* Run attitude RATE controllers which need the desired attitudes from above, add trim */
				const Vector3f angular_acceleration_setpoint = _rate_control.update(rates, body_rates_setpoint, angular_accel, dt,
						_landed);

				//roll角速率控制
				float roll_feedforward = _param_fw_rr_ff.get() * _airspeed_scaling * body_rates_setpoint(0);	//PID调参中的参数FW_RR_FF参数
				float roll_u = angular_acceleration_setpoint(0) * _airspeed_scaling * _airspeed_scaling + roll_feedforward;
				_vehicle_torque_setpoint.xyz[0] = PX4_ISFINITE(roll_u) ? math::constrain(roll_u + trim_roll, -1.f, 1.f) : trim_roll;	//PX4_ISFINITE作用是判断参数是否有限

				//pith角速率控制
				float pitch_feedforward = _param_fw_pr_ff.get() * _airspeed_scaling * body_rates_setpoint(1);
				float pitch_u = angular_acceleration_setpoint(1) * _airspeed_scaling * _airspeed_scaling + pitch_feedforward;
				_vehicle_torque_setpoint.xyz[1] = PX4_ISFINITE(pitch_u) ? math::constrain(pitch_u + trim_pitch, -1.f, 1.f) : trim_pitch;

				//yaw角速率控制，分为地面的滑行轮控制和方向舵控制
				const float yaw_feedforward = _param_fw_yr_ff.get() * _airspeed_scaling * body_rates_setpoint(2);
				float yaw_u = angular_acceleration_setpoint(2) * _airspeed_scaling * _airspeed_scaling + yaw_feedforward;

				_vehicle_torque_setpoint.xyz[2] = PX4_ISFINITE(yaw_u) ? math::constrain(yaw_u + trim_yaw, -1.f, 1.f) : trim_yaw;

				//判断值是否可用，不可用的话重置积分
				if (!PX4_ISFINITE(roll_u) || !PX4_ISFINITE(pitch_u) || !PX4_ISFINITE(yaw_u)) {
					_rate_control.resetIntegral();
				}

				//将副翼、升降舵、方向舵的控制输出发送到飞行器的控制通道中。
				/* throttle passed through if it is finite */
				_vehicle_thrust_setpoint.xyz[0] = PX4_ISFINITE(_rates_sp.thrust_body[0]) ? _rates_sp.thrust_body[0] : 0.0f;

				//如果使能了电池缩放,根据电池状态对副翼、升降舵、方向舵的控制输出进行缩放。
				/* scale effort by battery status */
				if (_param_fw_bat_scale_en.get() && _vehicle_thrust_setpoint.xyz[0] > 0.1f) {

					if (_battery_status_sub.updated()) {
						battery_status_s battery_status{};

						if (_battery_status_sub.copy(&battery_status) && battery_status.connected && battery_status.scale > 0.f) {
							_battery_scale = battery_status.scale;
						}
					}

					_vehicle_thrust_setpoint.xyz[0] *= _battery_scale;
				}
			}

			// 发布速率控制器的状态信息至rate_ctrl_status主题中用于日志分析和外部订阅者使用。
			// publish rate controller status
			rate_ctrl_status_s rate_ctrl_status{};
			_rate_control.getRateControlStatus(rate_ctrl_status);	//获取rate_control.cpp中updateIntegral()方法计算后的角速率控制器中的积分值
			rate_ctrl_status.timestamp = hrt_absolute_time();

			_rate_ctrl_status_pub.publish(rate_ctrl_status);

		} else {
			//否则重置积分项，进入完全手动模式下不再做任何处理。
			// full manual
			_rate_control.resetIntegral();
		}


		/*
		 * 将副翼和方向舵的控制输出进行前馈调整，以抵消滚转时产生的副作用。
		 * 如果飞行器为 VTOL 垂直起降机型，将副翼和方向舵的控制输出重新转换到飞行器的体轴坐标系。
		 * 如果启用了速率控制模式或者姿态控制模式或者手动控制模式，将操纵面的控制输出发送到飞行器的控制通道中。
		 * 更新执行器控制状态。
		 * 对于手动控制模式，处理襟翼和扰流板控制。
		 */
		// Add feed-forward from roll control output to yaw control output
		// This can be used to counteract the adverse yaw effect when rolling the plane
		//PID调参中的FW_RLL_TO_YAW_FF参数
		_vehicle_torque_setpoint.xyz[2] = math::constrain(_vehicle_torque_setpoint.xyz[2] + _param_fw_rll_to_yaw_ff.get() *
						  _vehicle_torque_setpoint.xyz[0], -1.f, 1.f);

		// Tailsitter: rotate back to body frame from airspeed frame
		if (_vehicle_status.is_vtol_tailsitter) {
			const float helper = _vehicle_torque_setpoint.xyz[0];
			_vehicle_torque_setpoint.xyz[0] = _vehicle_torque_setpoint.xyz[2];
			_vehicle_torque_setpoint.xyz[2] = -helper;
		}

		/* Only publish if any of the proper modes are enabled */
		if (_vcontrol_mode.flag_control_rates_enabled ||
		    _vcontrol_mode.flag_control_attitude_enabled ||
		    _vcontrol_mode.flag_control_manual_enabled) {
			{
				_vehicle_thrust_setpoint.timestamp = hrt_absolute_time();
				_vehicle_thrust_setpoint.timestamp_sample = angular_velocity.timestamp_sample;
				_vehicle_thrust_setpoint_pub.publish(_vehicle_thrust_setpoint);

				_vehicle_torque_setpoint.timestamp = hrt_absolute_time();
				_vehicle_torque_setpoint.timestamp_sample = angular_velocity.timestamp_sample;
				_vehicle_torque_setpoint_pub.publish(_vehicle_torque_setpoint);
			}
		}

		updateActuatorControlsStatus(dt);	//计算控制能量消耗并发布控制器状态

		//手动襟翼/扰流板控制，在 VTOL 悬停中也有效。 如果是自动，则在 FW 位置控制器/VTOL 模块中处理和发布
		// Manual flaps/spoilers control, also active in VTOL Hover. Is handled and published in FW Position controller/VTOL module if Auto.
		if (_vcontrol_mode.flag_control_manual_enabled) {

			// Flaps control
			float flaps_control = 0.f; // default to no flaps

			//直接通过遥控控制
			/* map flaps by default to manual if valid */
			if (PX4_ISFINITE(_manual_control_setpoint.flaps)) {
				flaps_control = math::max(_manual_control_setpoint.flaps, 0.f); // do not consider negative switch settings
			}

			normalized_unsigned_setpoint_s flaps_setpoint;
			flaps_setpoint.timestamp = hrt_absolute_time();
			flaps_setpoint.normalized_setpoint = flaps_control;
			_flaps_setpoint_pub.publish(flaps_setpoint);

			// Spoilers control
			float spoilers_control = 0.f; // default to no spoilers

			switch (_param_fw_spoilers_man.get()) {
			case 0:
				break;

			case 1:
				// do not consider negative switch settings
				spoilers_control = PX4_ISFINITE(_manual_control_setpoint.flaps) ? math::max(_manual_control_setpoint.flaps, 0.f) : 0.f;
				break;

			case 2:
				// do not consider negative switch settings
				spoilers_control = PX4_ISFINITE(_manual_control_setpoint.aux1) ? math::max(_manual_control_setpoint.aux1, 0.f) : 0.f;
				break;
			}

			normalized_unsigned_setpoint_s spoilers_setpoint;
			spoilers_setpoint.timestamp = hrt_absolute_time();
			spoilers_setpoint.normalized_setpoint = spoilers_control;
			_spoilers_setpoint_pub.publish(spoilers_setpoint);
		}
	}

	//设置下一次运行速率控制逻辑的时间计划。
	// backup schedule
	ScheduleDelayed(20_ms);
	//结束性能计数。
	perf_end(_loop_perf);
}

//计算控制能量消耗并发布控制器状态
void FixedwingRateControl::updateActuatorControlsStatus(float dt)
{
	//对每个轴（roll、pitch、yaw）进行迭代
	for (int i = 0; i < 3; i++) {

		// We assume that the attitude is actuated by control surfaces
		// consuming power only when they move
		//计算控制信号，即当前时刻的期望扭矩减去前一时刻的期望扭矩。这个控制信号被假定为控制面的运动，只有在控制面运动时才消耗能量
		const float control_signal = _vehicle_torque_setpoint.xyz[i] - _control_prev[i];
		_control_prev[i] = _vehicle_torque_setpoint.xyz[i];

		//根据控制信号计算控制能量。控制能量被定义为控制信号的平方乘以时间间隔（dt）。这样做是为了量化控制面的运动和能量消耗
		_control_energy[i] += control_signal * control_signal * dt;
	}

	_energy_integration_time += dt;

	if (_energy_integration_time > 500e-3f) {

		actuator_controls_status_s status;
		status.timestamp = _vehicle_torque_setpoint.timestamp;

		//计算平均能量：根据每个轴上累积的控制能量和积分时间，计算平均控制功率（能量除以时间）
		for (int i = 0; i < 3; i++) {
			status.control_power[i] = _control_energy[i] / _energy_integration_time;
			_control_energy[i] = 0.f;
		}

		//发布控制器状态，其中包括每个轴上的平均控制功率
		_actuator_controls_status_pub.publish(status);
		_energy_integration_time = 0.f;	//将能量积分时间重置为零。
	}
}

int FixedwingRateControl::task_spawn(int argc, char *argv[])
{
	bool vtol = false;

	if (argc > 1) {
		if (strcmp(argv[1], "vtol") == 0) {
			vtol = true;
		}
	}

	FixedwingRateControl *instance = new FixedwingRateControl(vtol);

	if (instance) {
		_object.store(instance);
		_task_id = task_id_is_work_queue;

		if (instance->init()) {
			return PX4_OK;
		}

	} else {
		PX4_ERR("alloc failed");
	}

	delete instance;
	_object.store(nullptr);
	_task_id = -1;

	return PX4_ERROR;
}

int FixedwingRateControl::custom_command(int argc, char *argv[])
{
	return print_usage("unknown command");
}

int FixedwingRateControl::print_usage(const char *reason)
{
	if (reason) {
		PX4_WARN("%s\n", reason);
	}

	PRINT_MODULE_DESCRIPTION(
		R"DESCR_STR(
### Description
fw_rate_control is the fixed-wing rate controller.

)DESCR_STR");

	PRINT_MODULE_USAGE_NAME("fw_rate_control", "controller");
	PRINT_MODULE_USAGE_COMMAND("start");
	PRINT_MODULE_USAGE_ARG("vtol", "VTOL mode", true);
	PRINT_MODULE_USAGE_DEFAULT_COMMANDS();

	return 0;
}

extern "C" __EXPORT int fw_rate_control_main(int argc, char *argv[])
{
	return FixedwingRateControl::main(argc, argv);
}
