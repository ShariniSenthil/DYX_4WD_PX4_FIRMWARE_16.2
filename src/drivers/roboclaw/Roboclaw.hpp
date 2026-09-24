/****************************************************************************
 *
 *   Copyright (C) 2013-2023 PX4 Development Team. All rights reserved.
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
 * @file Roboclaw.hpp
 *
 * Roboclaw motor control driver
 *
 * Product page: https://www.basicmicro.com/motor-controller
 * Manual: https://downloads.basicmicro.com/docs/roboclaw_user_manual.pdf
 */

#pragma once

#include <lib/mixer_module/mixer_module.hpp>
#include <drivers/drv_hrt.h>
#include <px4_platform_common/module.h>
#include <px4_platform_common/module_params.h>
#include <sys/select.h>

#include <uORB/Subscription.hpp>
#include <uORB/topics/vehicle_status.h>
#include <uORB/topics/actuator_armed.h>
#include <uORB/topics/parameter_update.h>

#include <uORB/Publication.hpp>
#include <uORB/topics/esc_status.h>
#include <uORB/topics/wheel_encoders.h>

class Roboclaw : public ModuleBase<Roboclaw>, public OutputModuleInterface
{
public:
	/**
	 * @param device_name Name of the serial port e.g. "/dev/ttyS2"
	 * @param bad_rate_parameter Name of the parameter that holds the baud rate of this serial port
	 */
	Roboclaw(const char *device_name, const char *bad_rate_parameter);
	virtual ~Roboclaw();

	enum class Motor {
		Right = 0,
		Left = 1
	};

	static int task_spawn(int argc, char *argv[]); ///< @see ModuleBase
	static int custom_command(int argc, char *argv[]); ///< @see ModuleBase
	static int print_usage(const char *reason = nullptr); ///< @see ModuleBase
	int print_status() override; ///< @see ModuleBase

	void Run() override;

	/** @see OutputModuleInterface */
	bool updateOutputs(bool stop_motors, uint16_t outputs[MAX_ACTUATORS],
			   unsigned num_outputs, unsigned num_control_groups_updated) override;

	int setMotorSpeed(Motor motor, float value); ///< normalized side command
	void setMotorDutyCycle(Motor motor, float value);
	int readEncoder();
	void resetEncoders();

private:
	enum class Command : uint8_t {
		ReadStatus = 90,

		DriveForwardMotor1 = 0,
		DriveBackwardsMotor1 = 1,
		DriveForwardMotor2 = 4,
		DriveBackwardsMotor2 = 5,
		DutyCycleMotor1 = 32,
		DutyCycleMotor2 = 33,
		DriveSpeedMotor1 = 35,   // M1 signed speed target (QPPS)
		DriveSpeedMotor2 = 36,   // M2 signed speed target (QPPS)

		ReadSpeedMotor1 = 18,
		ReadSpeedMotor2 = 19,
		ResetEncoders = 20,
		ReadEncoderCounters = 78,
	};

	static constexpr int MAX_ACTUATORS = 2;
	MixingOutput _mixing_output{"RBCLW", MAX_ACTUATORS, *this, MixingOutput::SchedulingPolicy::Auto, false};

	uORB::SubscriptionData<actuator_armed_s> _actuator_armed_sub{ORB_ID(actuator_armed)};
	uORB::Subscription _parameter_update_sub{ORB_ID(parameter_update)};
	uORB::Subscription _vehicle_status_sub{ORB_ID(vehicle_status)};
	uORB::Publication<wheel_encoders_s> _wheel_encoders_pub{ORB_ID(wheel_encoders)};
	uORB::Publication<esc_status_s> _esc_status_pub{ORB_ID(esc_status)};

	char _stored_device_name[256]; // Adjust size as necessary
	char _stored_baud_rate_parameter[256]; // Adjust size as necessary

	int sendUnsigned7Bit(Command command, float data);
	int sendSigned16Bit(Command command, float data);
	int sendSigned32Bit(Command command, int32_t value);
	void publishEscStatus(bool online);
	void markCommunicationFailed(const char *reason);
	void checkVelocityControlConfig();

	// Roboclaw protocol
	int sendTransaction(Command cmd, uint8_t *write_buffer, size_t bytes_to_write);
	int writeCommandWithPayload(Command cmd, uint8_t *wbuff, size_t bytes_to_write);
	int readAcknowledgement();

	int receiveTransaction(Command cmd, uint8_t *read_buffer, size_t bytes_to_read);
	int writeCommand(Command cmd);
	int readResponse(Command command, uint8_t *read_buffer, size_t bytes_to_read);

	static uint16_t _calcCRC(const uint8_t *buf, size_t n, uint16_t init = 0);
	int32_t swapBytesInt32(uint8_t *buffer);

	// UART handling
	int initializeUART();
	bool _uart_initialized{false};
	int _uart_fd{-1};
	fd_set _uart_fd_set;
	struct timeval _uart_fd_timeout;
	hrt_abstime _last_encoder_read{0};
	hrt_abstime _last_encoder_warn{0};
	uint8_t _consecutive_encoder_failures{0};
	uint16_t _esc_status_counter{0};
	bool _vel_ctrl_config_invalid{false};
	bool _vel_ctrl_config_checked{false};

	DEFINE_PARAMETERS(
		(ParamInt<px4::params::RBCLW_ADDRESS>) _param_rbclw_address,
		(ParamInt<px4::params::RBCLW_COUNTS_REV>) _param_rbclw_counts_rev,
		(ParamInt<px4::params::RBCLW_BAUD>) _param_rbclw_baud,
		(ParamInt<px4::params::RBCLW_VEL_CTRL>) _param_rbclw_vel_ctrl,
		(ParamInt<px4::params::RBCLW_QPPS_MAX>) _param_rbclw_qpps_max,
		(ParamInt<px4::params::RBCLW_ENC_HZ>) _param_rbclw_enc_hz
	)
};
