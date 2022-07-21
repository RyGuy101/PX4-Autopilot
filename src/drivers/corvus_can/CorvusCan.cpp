/****************************************************************************
 *
 *   Copyright (c) 2021 PX4 Development Team. All rights reserved.
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
 * @file CorvusCan.cpp
 * @author Ryan Nemiroff <ryan@corvus-robotics.com>
 */

#include <systemlib/mavlink_log.h>

#include "CorvusCan.hpp"

CorvusCan::CorvusCan() :
	ScheduledWorkItem(MODULE_NAME, px4::wq_configurations::uavcan)
{
}

CorvusCan::~CorvusCan()
{
}

void CorvusCan::Run()
{
	if (should_exit()) {
		exit_and_cleanup();
		return;
	}

	if (_fd < 0) {
		struct can_dev_s *can = stm32_caninitialize(1);

		if (can == nullptr) {
			mavlink_log_info(&_mavlink_log_pub, "Failed to get CAN interface");
			return;
		}

		if (can_register("/dev/can0", can) < 0) {
			mavlink_log_info(&_mavlink_log_pub, "can_register failed");
			return;
		}

		if ((_fd = open("/dev/can0", O_RDWR)) < 0) {
			mavlink_log_info(&_mavlink_log_pub, "open error: %s", strerror(errno));
			return;
		}
	}

	CanFrame can_frame;
	int result;

	// Transmit test
	can_frame.id = 0x602;
	uint8_t can_data[8] = { 0x2B, 0x17, 0x10, 0x00, 0xE8, 0x03, 0x00, 0x00 };
	memcpy(can_frame.data, can_data, 8);
	can_frame.data_len = 8;
	result = transmit(&can_frame);
	// mavlink_log_info(&_mavlink_log_pub, "transmit returned %d", result);

	// Receive test
	while ((result = receive(&can_frame)) > 0) {
		mavlink_log_info(&_mavlink_log_pub, "Received CAN message!");
	}

	// mavlink_log_info(&_mavlink_log_pub, "receive returned %d", result);
}

int CorvusCan::transmit(const CanFrame *outgoing_frame)
{
	if (_fd < 0 || outgoing_frame == nullptr) {
		return -1;
	}

	if (outgoing_frame->data_len > 8) {
		PX4_INFO("error: cannot handle CAN frames with more than 8 data bytes");
		return -1;
	}

	struct pollfd fds {};

	fds.fd = _fd;

	fds.events = POLLOUT;

	if (poll(&fds, 1, 0) < 0) {
		return -1;
	}

	if ((fds.revents & POLLOUT) == 0) {
		return -1;
	}

	struct can_msg_s can_msg;

	memset(&can_msg, 0, sizeof(can_msg));

	can_msg.cm_hdr.ch_id = outgoing_frame->id;

	can_msg.cm_hdr.ch_dlc = outgoing_frame->data_len;

	memcpy(can_msg.cm_data, outgoing_frame->data, outgoing_frame->data_len);

	const size_t msg_len = CAN_MSGLEN(can_msg.cm_hdr.ch_dlc);

	const ssize_t nbytes = write(_fd, &can_msg, msg_len);

	if (nbytes < 0 || (size_t)nbytes != msg_len) {
		return -1;
	}

	return 1;
}

int CorvusCan::receive(CanFrame *received_frame)
{
	if (_fd < 0 || received_frame == nullptr) {
		return -1;
	}

	struct pollfd fds {};

	fds.fd = _fd;

	fds.events = POLLIN;

	if (poll(&fds, 1, 0) < 0) {
		return -1;
	}

	if (fds.revents & POLLIN) {

		struct can_msg_s can_msg;
		const ssize_t nbytes = read(_fd, &can_msg, sizeof(can_msg));

		if (nbytes < 0 || (size_t)nbytes < CAN_MSGLEN(0) || (size_t)nbytes > sizeof(can_msg)) {
			return -1;

		} else {
			if (can_msg.cm_hdr.ch_dlc > 8) {
				PX4_INFO("error: cannot handle CAN frames with more than 8 data bytes");
				return -1;
			}

			received_frame->id = can_msg.cm_hdr.ch_id;
			received_frame->data_len = can_msg.cm_hdr.ch_dlc;
			memcpy(received_frame->data, can_msg.cm_data, can_msg.cm_hdr.ch_dlc);
			return nbytes;
		}
	}

	return 0;
}

int CorvusCan::start()
{
	ScheduleOnInterval(1000000 / SAMPLE_RATE);
	return PX4_OK;
}

int CorvusCan::task_spawn(int argc, char *argv[])
{
	CorvusCan *instance = new CorvusCan();

	if (!instance) {
		PX4_ERR("driver allocation failed");
		return PX4_ERROR;
	}

	_object.store(instance);
	_task_id = task_id_is_work_queue;

	instance->start();
	return 0;
}

int CorvusCan::print_usage(const char *reason)
{
	if (reason) {
		printf("%s\n\n", reason);
	}

	PRINT_MODULE_DESCRIPTION(
		R"DESCR_STR(
### Description
CAN test.

)DESCR_STR");

	PRINT_MODULE_USAGE_NAME("corvus_can", "system");
	PRINT_MODULE_USAGE_COMMAND("start");
	PRINT_MODULE_USAGE_DEFAULT_COMMANDS();

	return 0;
}

int CorvusCan::custom_command(int argc, char *argv[])
{
	if (!is_running()) {
		PX4_INFO("not running");
		return PX4_ERROR;
	}

	return print_usage("Unrecognized command.");
}

extern "C" __EXPORT int corvus_can_main(int argc, char *argv[])
{
	return CorvusCan::main(argc, argv);
}
