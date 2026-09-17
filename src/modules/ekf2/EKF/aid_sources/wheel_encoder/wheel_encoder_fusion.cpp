/****************************************************************************
 * DYX 4WD rear-wheel encoder longitudinal velocity fusion for EKF2.
 ****************************************************************************/

#include "ekf.h"

#include "ekf_derivation/generated/compute_body_vel_innov_var_h.h"

void Ekf::controlWheelEncoderFusion(const imuSample &imu_sample)
{
	const uint64_t timeout_us = static_cast<uint64_t>(math::max(_params.wenc_timeout_ms, 100.f) * 1000.f);

	// Wheel aiding is secondary only. Disabling the source, invalid radius, or
	// loss of the primary horizontal solution stops wheel fusion without any
	// navigation-state reset.
	if ((_params.wenc_ctrl == 0) || !PX4_ISFINITE(_params.wenc_rad) || !(_params.wenc_rad > 0.f)
	    || !isHorizontalAidingActive()) {
		stopWheelEncoderFusion();
		return;
	}

	// If no successful wheel update has occurred within the configured timeout,
	// mark wheel aiding inactive. GNSS/IMU aiding continues unchanged.
	if (_wheel_encoder_fusion_active && isTimedOut(_aid_src_wheel_encoder.time_last_fuse, timeout_us)) {
		ECL_WARN("wheel encoder fusion timed out");
		stopWheelEncoderFusion();
	}

	if (_wheel_encoder_buffer == nullptr) {
		return;
	}

	wheelEncoderSample sample;

	if (!_wheel_encoder_buffer->pop_first_older_than(imu_sample.time_us, &sample)) {
		return;
	}

	// Basic sensor sanity only. Statistical consistency is handled by the EKF
	// innovation gate rather than by extra wheel-speed/slip tuning parameters.
	if ((sample.time_us == 0)
	    || !PX4_ISFINITE(sample.vel_body_fwd)
	    || !PX4_ISFINITE(sample.vel_fwd_var)
	    || !(sample.vel_fwd_var > 0.f)
	    || !(imu_sample.delta_ang_dt > FLT_EPSILON)) {
		return;
	}

	// wheel_encoders describes the midpoint of the rear encoder axle. Convert
	// that point velocity to the IMU reference point before fusion.
	// Body FRD: +X forward, +Y right, +Z down.
	const Vector3f pos_offset_body = _params.wenc_pos_body - _params.imu_pos_body;
	const Vector3f angular_velocity = imu_sample.delta_ang / imu_sample.delta_ang_dt - _state.gyro_bias;
	const Vector3f vel_offset_body = angular_velocity % pos_offset_body;

	// True 1D measurement: only longitudinal body-X wheel velocity is fused.
	// No body-Y zero-velocity constraint and no wheel-derived yaw are applied.
	const float measurement = sample.vel_body_fwd - vel_offset_body(0);
	const float observation_var = math::max(sample.vel_fwd_var, sq(0.01f));
	const float gate = math::max(_params.wenc_gate, 1.f);

	VectorState H[3];
	Vector3f innovation_variance;
	const auto state_vector = _state.vector();

	// Reuse PX4's generated body-velocity Jacobian and take X only. The Y/Z
	// Jacobians are calculated by the generated routine but are never fused.
	const Vector3f body_vel_variance{observation_var, observation_var, observation_var};
	sym::ComputeBodyVelInnovVarH(state_vector, P, body_vel_variance,
				     &innovation_variance, &H[0], &H[1], &H[2]);

	const float innovation = Vector3f(_R_to_earth.transpose().row(0)) * _state.vel - measurement;

	updateAidSourceStatus(_aid_src_wheel_encoder,
			      sample.time_us,
			      measurement,
			      observation_var,
			      innovation,
			      innovation_variance(0),
			      gate);

	if (_aid_src_wheel_encoder.innovation_rejected) {
		return;
	}

	// Guard the scalar update against a non-physical/ill-conditioned innovation
	// variance. This does not add a vehicle-speed threshold or tuning parameter.
	if (!PX4_ISFINITE(_aid_src_wheel_encoder.innovation_variance)
	    || !(_aid_src_wheel_encoder.innovation_variance > _aid_src_wheel_encoder.observation_variance)
	    || !(_aid_src_wheel_encoder.innovation_variance > FLT_EPSILON)) {
		_aid_src_wheel_encoder.innovation_rejected = true;
		return;
	}

	VectorState Kfusion = P * H[0] / _aid_src_wheel_encoder.innovation_variance;
	measurementUpdate(Kfusion, H[0], _aid_src_wheel_encoder.observation_variance,
			  _aid_src_wheel_encoder.innovation);

	_aid_src_wheel_encoder.fused = true;
	_aid_src_wheel_encoder.time_last_fuse = _time_delayed_us;

	// Wheel speed is deliberately a secondary rover aid. Do not refresh the
	// shared horizontal-velocity fusion timestamp here: GNSS recovery/reset
	// logic uses that timestamp to decide whether a returning GNSS velocity
	// source needs to reset the velocity state.

	if (!_wheel_encoder_fusion_active) {
		ECL_INFO("starting wheel encoder fusion");
		_wheel_encoder_fusion_active = true;
	}
}

void Ekf::stopWheelEncoderFusion()
{
	if (_wheel_encoder_fusion_active) {
		ECL_INFO("stopping wheel encoder fusion");
		_wheel_encoder_fusion_active = false;
	}

	_aid_src_wheel_encoder.fused = false;
}
