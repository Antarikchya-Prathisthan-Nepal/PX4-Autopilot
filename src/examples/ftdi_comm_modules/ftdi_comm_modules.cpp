/****************************************************************************
 *
 *   Copyright (c) 2018 PX4 Development Team. All rights reserved.
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

#include "ftdi_comm_modules.h"
#include <poll.h>
#include <string.h>
#include <px4_platform_common/getopt.h>
#include <px4_platform_common/log.h>
#include <px4_platform_common/posix.h>

#include <uORB/topics/parameter_update.h>
#include <uORB/topics/sensor_combined.h>
#include <uORB/topics/custom_uart.h>
#include <uORB/topics/vehicle_acceleration.h>

int FtdiModule::print_status()
{
	PX4_INFO("Running");
	// TODO: print additional runtime information about the state of the module

	return 0;
}

int FtdiModule::custom_command(int argc, char *argv[])
{
	if (!is_running()) {
		print_usage("not running");
		return 1;
	}

	// additional custom commands can be handled like this:
	if (!strcmp(argv[0], "f")) {
		// get_instance()->do_something();
		PX4_INFO("Sth  fone");
		return 0;
	}



	return print_usage("unknown command");
}


int FtdiModule::task_spawn(int argc, char *argv[])
{
	_task_id = px4_task_spawn_cmd("ftdi_comm_modules",
				      SCHED_DEFAULT,
				      SCHED_PRIORITY_DEFAULT,
				      1024,
				      (px4_main_t)&run_trampoline,
				      (char *const *)argv);

	if (_task_id < 0) {
		_task_id = -1;
		return -errno;
	}

	return 0;
}

FtdiModule *FtdiModule::instantiate(int argc, char *argv[]) // **ftdi_comm_modules start**
{
	int example_param = 0;
	bool example_flag = false;
	bool error_flag = false;







//***
	int myoptind = 1;
	int ch;
	const char *myoptarg = nullptr;

	// parse CLI arguments
	while ((ch = px4_getopt(argc, argv, "p:f", &myoptind, &myoptarg)) != EOF) {
		switch (ch) {
		case 'p':
			example_param = (int)strtol(myoptarg, nullptr, 10);
			break;

		case 'f':
			example_flag = true;
			break;

		case '?':
			error_flag = true;
			break;

		default:
			PX4_WARN("unrecognized flag");
			error_flag = true;
			break;
		}
	}

	if (error_flag) {
		return nullptr;
	}

	FtdiModule *instance = new FtdiModule(example_param, example_flag);

	if (instance == nullptr) {
		PX4_ERR("alloc failed");
	}

	return instance;
}

FtdiModule::FtdiModule(int example_param, bool example_flag)
	: ModuleParams(nullptr)
{
}

void FtdiModule::run()
{
	int custom_uart_sub = orb_subscribe(ORB_ID(custom_uart));

	// Example: run the loop synchronized to the sensor_combined topic publication
	int sensor_combined_sub = orb_subscribe(ORB_ID(sensor_combined));
	int sensor_combined_fd = orb_subscribe(ORB_ID(vehicle_acceleration));
	px4_pollfd_struct_t fds[2];
	PX4_INFO("custom: %f\nacc: %f \n sesins: %f",(double)custom_uart_sub, (double)sensor_combined_sub ,(double)sensor_combined_fd);
	fds[0].fd = sensor_combined_sub;
	fds[0].events = POLLIN;
	fds[1].fd = sensor_combined_fd;
	fds[1].events = POLLIN ;



	/*Start code for advvertise ftdi*/
	struct custom_uart_s ftdi;
	memset(&ftdi,0,sizeof(ftdi));
	orb_advert_t att_pub = orb_advertise(ORB_ID(custom_uart),&ftdi);
	PX4_INFO("Data advv is %s",(char*)att_pub);

	/*End code for advvertise ftdi*/

/*code for  auart*/
// //***
	struct pollfd serial_poll;
	memset(&serial_poll,0,sizeof(serial_poll));

	char* buffer;
	// const char* compar="\r" ;
	// // strcpy(&compar,"\r");
	// // strcpy(&buffer,"\0");
	// char buffer_aux[256];
	// int ret;
	// int i=0;

	char test[]="data sent";
	int fd;
	int ret2;
	fd = open("dev/ttsy2",O_RDWR);//O_RDWR
	if(fd==0){
		printf("error uart  connection");

	}
	ret2 = write(fd, &test, sizeof(test));
	PX4_INFO("Data sent is :%s %i",test,ret2);
	close(fd);
		fd = open("dev/ttsy2",O_RDWR);//O_RDWR

	int ret = read(fd, &buffer, sizeof(buffer));
	PX4_INFO("Got this data on uart :%s %i",buffer,ret);
	close(fd);
// 	while (1) {





// // 		ret2 = write(fd, &test, sizeof(test));
// // PX4_INFO("%i",ret2);
// 		ret = read(fd, &buffer, sizeof(buffer));
// 		const char* b1 = buffer;
// 		if (ret > 0) {
// 			strcpy(&buffer_aux[i], b1); // Copy the content pointed by b1 into buffer_aux starting at index i
// 			i += strlen(b1); // Update the index to the next position after the copied content

// 			if (strcmp(b1, compar) == 0) { // Use strcmp correctly for comparison
// 			ret = write(fd, buffer_aux, sizeof(char) * i);

// 			if (ret > 0) {
// 				i = 0;
// 			}
// 			}
// 		}
// 	}

// Code for uart end


	// initialize parameters
	parameters_update(true);
		int counter=0;
	while (!should_exit()) {

		// PX4_INFO("got here ");		// wait for up to 1000ms for data
		int pret = px4_poll(fds, (sizeof(fds) / sizeof(fds[0])), 1000);

		if (pret == 0) {
			// Timeout: let the loop run anyway, don't do `continue` here

		} else if (pret < 0) {
			// this is undesirable but not much we can do
			PX4_ERR("poll error %d, %d", pret, errno);
			px4_usleep(50000);
			continue;

		} else if (fds[0].revents & POLLIN) {
			struct vehicle_acceleration_s accel;
			orb_copy(ORB_ID(vehicle_acceleration), sensor_combined_fd, &accel);
			struct sensor_combined_s sensor_combined;
			orb_copy(ORB_ID(sensor_combined), sensor_combined_sub, &sensor_combined);
			// PX4_INFO(sensor_combined_sub);
			counter++;
			if(counter == 10){
				break;
			}
			PX4_INFO("\nCounter : %f\nAccelerometer:\t%8.4f\t%8.4f\t%8.4f",
					 (double) counter,
					 (double)accel.xyz[0],
					 (double)accel.xyz[1],
					 (double)accel.xyz[2]);
			// TODO: do something with the data...

		}

		parameters_update();
	}

	orb_unsubscribe(sensor_combined_sub);
}

void FtdiModule::parameters_update(bool force)
{
	// check for parameter updates
	if (_parameter_update_sub.updated() || force) {
		// clear update
		parameter_update_s update;
		_parameter_update_sub.copy(&update);

		// update parameters from storage
		updateParams();
	}
}

int FtdiModule::print_usage(const char *reason)
{
	if (reason) {
		PX4_WARN("%s\n", reason);
	}

	PRINT_MODULE_DESCRIPTION(
		R"DESCR_STR(
### Description
Section that describes the provided module functionality.

This is a template for a module running as a task in the background with start/stop/status functionality.

### Implementation
Section describing the high-level implementation of this module.

### Examples
CLI usage example:
$ module start -f -p 42

)DESCR_STR");

	PRINT_MODULE_USAGE_NAME("module", "template");
	PRINT_MODULE_USAGE_COMMAND("start");
	PRINT_MODULE_USAGE_PARAM_FLAG('f', "Optional example flag", true);
	PRINT_MODULE_USAGE_PARAM_INT('p', 0, 0, 1000, "Optional example parameter", true);
	PRINT_MODULE_USAGE_DEFAULT_COMMANDS();

	return 0;
}

int ftdi_comm_modules_main(int argc, char *argv[])
{
	return FtdiModule::main(argc, argv);
}
