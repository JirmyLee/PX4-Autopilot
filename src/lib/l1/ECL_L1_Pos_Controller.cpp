/****************************************************************************
 *
 *   Copyright (c) 2013-2021 PX4 Development Team. All rights reserved.
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
 * @file ECL_L1_Pos_Controller.cpp
 * Implementation of L1 position control.
 * Authors and acknowledgements in header.
 *
 */

#include "ECL_L1_Pos_Controller.hpp"

#include <lib/geo/geo.h>

#include <px4_platform_common/defines.h>

#include <float.h>

using matrix::Vector2f;

//(A点，B点，飞机当前位置，速度)使用L1控制律飞曲线轨迹。
void
ECL_L1_Pos_Controller::navigate_waypoints(const Vector2f &vector_A, const Vector2f &vector_B,
		const Vector2f &vector_curr_position, const Vector2f &ground_speed_vector)
{
	/* this follows the logic presented in [1] */
	float eta = 0.0f;	//本函数的目的就是求得eta，飞机飞行速度方向和l1航点之间的夹角。 这个角度会被用于计算横滚角。

	/* get the direction between the last (visited) and next waypoint */
	Vector2f vector_P_to_B = vector_B - vector_curr_position;
	Vector2f vector_P_to_B_unit = vector_P_to_B.normalized();
	//计算期望航点的期望航向值，这个_target_bearing是固件的全局变量,也叫目标方位角（这里要注意和导航方位角相区分），被其它进程使用， 本函数再往下没有再用到该变量
	_target_bearing = atan2f(vector_P_to_B_unit(1), vector_P_to_B_unit(0));

	/* enforce a minimum ground speed of 0.1 m/s to avoid singularities */
	float ground_speed = math::max(ground_speed_vector.length(), 0.1f);	//强制执行0.1 m/s的最小地面速度，以避免奇点，也就是令系统不能正常工作的点。

	/* calculate the L1 length required for the desired period */
	_L1_distance = _L1_ratio * ground_speed;	//依据期望的时间周期去计算L1的长度

	/* calculate vector from A to B */
	Vector2f vector_AB = vector_B - vector_A;	//计算从A到B的矢量

	/*
	 * check if waypoints are on top of each other. If yes,
	 * skip A and directly continue to B
	 */
	if (vector_AB.length() < 1.0e-6f) {
		vector_AB = vector_B - vector_curr_position;
	}

	vector_AB.normalize();

	/* calculate the vector from waypoint A to the aircraft */
	Vector2f vector_A_to_airplane = vector_curr_position - vector_A;	//计算经纬度坐标系下航点A到当前飞机所在位置的向量(假设为AC向量，C代表飞机现在的位置)

	// |AC| *sin(AB和AC的夹角) 为偏离航向的距离
	/* calculate crosstrack error (output only) */
	_crosstrack_error = vector_AB % vector_A_to_airplane;	//计算交叉轨迹误差 (仅输出)

	//如果当前飞机在A点后方+-135度角度范围内，而且飞机到A点的距离大于L1的值，那么使用A点作为L1点,如果飞机现在在A和B航点之间，那么采用正常的L1算法逻辑。
	/*
	 * If the current position is in a +-135 degree angle behind waypoint A
	 * and further away from A than the L1 distance, then A becomes the L1 point.
	 * If the aircraft is already between A and B normal L1 logic is applied.
	 */
	float distance_A_to_airplane = vector_A_to_airplane.length();	//计算A点到飞机的距离
	float alongTrackDist = vector_A_to_airplane * vector_AB;	//|AC|*cos(AB和AC的夹角)， 也就是AC在AB上的投影

	/* 估计飞机位置 WRT to B */
	/* estimate airplane position WRT to B */
	Vector2f vector_B_to_P = vector_curr_position - vector_B;
	Vector2f vector_B_to_P_unit = vector_B_to_P.normalized();

	//计算飞机位置矢量相对于line的角度
	/* calculate angle of airplane position vector relative to line) */

	//这有可能仅是基于点积
	// XXX this could probably also be based solely on the dot product
	float AB_to_BP_bearing = atan2f(vector_B_to_P_unit % vector_AB, vector_B_to_P_unit * vector_AB);

	//|AC|长度大于_L1_distance,并且AC和AB的夹角大于正负135度,将使用A点作为L1参考点
	/* extension from [2], fly directly to A */
	if (distance_A_to_airplane > _L1_distance && alongTrackDist / math::max(distance_A_to_airplane, 1.0f) < -0.7071f) {
		//飞机飞往A点时，飞机的偏航eta(北东地坐标系下)怎么计算？

		/* calculate eta to fly to waypoint A */

		/* unit vector from waypoint A to current position */
		Vector2f vector_A_to_airplane_unit = vector_A_to_airplane.normalized();	// 从A点到飞机当前位置的单位矢量计算

		/* velocity across / orthogonal to line */
		float xtrack_vel = ground_speed_vector % (-vector_A_to_airplane_unit);	//垂直AC向量方向的速度

		/* velocity along line */
		float ltrack_vel = ground_speed_vector * (-vector_A_to_airplane_unit);	//平行于AC向量方向的速度
		eta = atan2f(xtrack_vel, ltrack_vel);	//飞机飞行速度方向和L1的夹角

		/* bearing from current position to L1 point */
		_nav_bearing = atan2f(-vector_A_to_airplane_unit(1), -vector_A_to_airplane_unit(0));	//飞机当前位置到L1点的朝向，就是导航方位角

		/*
		 * If the AB vector and the vector from B to airplane point in the same
		 * direction, we have missed the waypoint. At +- 90 degrees we are just passing it.
		 */

	//如果AB向量和B到飞机的向量是同一个方向，we have missed the waypoint. At +- 90 degrees we are just passing it.
	//如果飞机超过了B点，那就以B点为L1参考点。
	} else if (fabsf(AB_to_BP_bearing) < math::radians(100.0f)) {
		//在从航路点A到航路点B过程中，飞机被切换到了手动模式，然后又回到mission模式， 导致错过了航路点， 也就是飞机本身已经过了B点， 那么这个时候，飞机要飞回到B点。
		//然后再做打算, 那么飞机的偏航角eta(北东地坐标系下)怎么计算呢。
		/*
		 * Extension, fly back to waypoint.
		 *
		 * This corner case is possible if the system was following
		 * the AB line from waypoint A to waypoint B, then is
		 * switched to manual mode (or otherwise misses the waypoint)
		 * and behind the waypoint continues to follow the AB line.
		 */

		/* calculate eta to fly to waypoint B */

		/* velocity across / orthogonal to line */
		float xtrack_vel = ground_speed_vector % (-vector_B_to_P_unit);	//垂直AC向量方向的速度.

		/* velocity along line */
		float ltrack_vel = ground_speed_vector * (-vector_B_to_P_unit);	//平行于AC向量方向的速度.
		eta = atan2f(xtrack_vel, ltrack_vel);	//飞机飞行速度方向和L1航点的夹角

		/* bearing from current position to L1 point */
		_nav_bearing = atan2f(-vector_B_to_P_unit(1), -vector_B_to_P_unit(0));	//飞机当前位置到L1航点的航向，就是导航方位角,但是这个导航方位角在哪里被使用？

	} else {
		//假如飞机位置在A和B之间，来计算飞机飞行速度向量方向和飞机与L1连接线方向的夹角eta， 采用eat=eta1+eta2来计算。
		/* calculate eta to fly along the line between A and B */

		/* velocity across / orthogonal to line */
		float xtrack_vel = ground_speed_vector % vector_AB;	//垂直AC向量方向的速度

		/* velocity along line */
		float ltrack_vel = ground_speed_vector * vector_AB;	//平行于AC向量方向的速度

		//eta2是飞机飞行速度向量方向和与期望路径平行线(经过飞机现在位置)的夹角
		/* calculate eta2 (angle of velocity vector relative to line) */
		float eta2 = atan2f(xtrack_vel, ltrack_vel);		//tann2=d导/V, 这里的V就是平行于AC向量方向的速度。

		//计算 eta1 (经过飞机当前位置的期望路径的平行线和飞机到L1连线的夹角)
		/* calculate eta1 (angle to L1 point) */
		float xtrackErr = vector_A_to_airplane % vector_AB;
		float sine_eta1 = xtrackErr / math::max(_L1_distance, 0.1f);

		//限制输出在？度以内
		//eta1是与期望路径平行线(经过飞机现在位置)和飞机与L1航点连线的夹角
		/* limit output to feasible values */
		sine_eta1 = math::constrain(sine_eta1, -1.0f, 1.0f);
		float eta1 = asinf(sine_eta1);
		eta = eta1 + eta2;	//当无人机位置在A和B位置中间附近时，飞机的偏航（北东地坐标系下）这么计算。

		/* bearing from current position to L1 point */
		_nav_bearing = atan2f(vector_AB(1), vector_AB(0)) + eta1;	// 飞机当前位置到L1点的朝向 就是导航方位角
	}

	/* limit angle to +-90 degrees */
	eta = math::constrain(eta, (-M_PI_F) / 2.0f, +M_PI_F / 2.0f);	//限制角度在90度以内
	_lateral_accel = _K_L1 * ground_speed * ground_speed / _L1_distance * sinf(eta);	//公式， a=2*v*v/l1(n1+n2), 这个值是全局变量，将被用于update_roll_setpoint（）函数中计算横滚值。

}

void ECL_L1_Pos_Controller::set_l1_period(float period)
{
	_L1_period = period;

	/* calculate the ratio introduced in [2] */
	_L1_ratio = 1.0f / M_PI_F * _L1_damping * _L1_period;

	/* calculate normalized frequency for heading tracking */
	_heading_omega = sqrtf(2.0f) * M_PI_F / _L1_period;
}

void ECL_L1_Pos_Controller::set_l1_damping(float damping)
{
	_L1_damping = damping;

	/* calculate the ratio introduced in [2] */
	_L1_ratio = 1.0f / M_PI_F * _L1_damping * _L1_period;

	/* calculate the L1 gain (following [2]) */
	_K_L1 = 4.0f * _L1_damping * _L1_damping;
}
