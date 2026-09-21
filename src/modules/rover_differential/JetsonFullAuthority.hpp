/****************************************************************************
 *
 * DYX Jetson full-authority rover control contract.
 *
 ****************************************************************************/

#pragma once

#include <px4_platform_common/defines.h>

#include <uORB/topics/offboard_control_mode.h>
#include <uORB/topics/trajectory_setpoint.h>
#include <uORB/topics/vehicle_control_mode.h>

namespace RoverControlContract
{

inline bool isJetsonFullAuthority(
	const vehicle_control_mode_s &vehicle_control_mode,
	const offboard_control_mode_s &offboard_control_mode,
	const trajectory_setpoint_s &trajectory_setpoint)
{
	return vehicle_control_mode.flag_control_offboard_enabled
	       && offboard_control_mode.velocity
	       && !offboard_control_mode.position
	       && !offboard_control_mode.attitude
	       && PX4_ISFINITE(trajectory_setpoint.yaw)
	       && PX4_ISFINITE(trajectory_setpoint.yawspeed);
}

} // namespace RoverControlContract
