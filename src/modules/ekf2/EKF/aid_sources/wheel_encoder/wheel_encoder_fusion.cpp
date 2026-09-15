/****************************************************************************
 * DYX 4WD rear-wheel encoder velocity fusion for EKF2.
 ****************************************************************************/

#include "ekf.h"

void Ekf::controlWheelEncoderFusion(const imuSample &imu_sample)
{
	if (_wheel_encoder_buffer == nullptr) {
		return;
	}

	wheelEncoderSample sample;

	if (!_wheel_encoder_buffer->pop_first_older_than(imu_sample.time_us, &sample)) {
		return;
	}

	// Secondary aid only. GNSS/other horizontal aiding must establish the
	// navigation solution; rear encoders do not create a global reference.
	if (!isHorizontalAidingActive()) {
		return;
	}

	// wheel_encoders describes the midpoint of the rear encoder axle. Convert
	// that measured point velocity to the IMU reference point before fusion.
	// Body FRD: +X forward, +Y right, +Z down.
	const Vector3f pos_offset_body = _params.wenc_pos_body - _params.imu_pos_body;
	const Vector3f angular_velocity = imu_sample.delta_ang / imu_sample.delta_ang_dt - _state.gyro_bias;
	const Vector3f vel_offset_body = angular_velocity % pos_offset_body;

	// Differential/skid-steer rear-wheel estimate:
	// X = average rear wheel forward speed, Y ~= 0 non-holonomic constraint.
	// On a 4WD skid-steer rover lateral tire scrub is real during pivots, so
	// EKF2_WENC_LAT_N defaults conservatively high (0.5 m/s).
	const Vector3f measurement = Vector3f(sample.vel_body_fwd, 0.f, 0.f) - vel_offset_body;
	const Vector3f measurement_var(
		math::max(sample.vel_fwd_var, sq(0.01f)),
		sq(math::max(_params.wenc_lat_noise, 0.01f)),
		sq(1000.f));

	fuseBodyFrameVelocity(_aid_src_wheel_encoder,
			      sample.time_us,
			      measurement,
			      measurement_var,
			      math::max(_params.wenc_gate, 1.f));
}
