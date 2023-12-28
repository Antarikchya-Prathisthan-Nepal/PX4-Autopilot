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

#include "mylogger_main.h"

#include <px4_platform_common/getopt.h>
#include <px4_platform_common/log.h>
#include <px4_platform_common/posix.h>

#include <uORB/topics/parameter_update.h>
#include <uORB/topics/sensor_combined.h>


int Mylogger::print_status()
{
	PX4_INFO("Running");
	// TODO: print additional runtime information about the state of the module

	return 0;
}

int Mylogger::custom_command(int argc, char *argv[])
{

	if (!is_running()) {
		print_usage("not running");
		return 1;
	}

	// additional custom commands can be handled like this:
	// if (!strcmp(argv[0], "do-something")) {
	// 	get_instance()->do_something();
	// 	return 0;
	// }


	return print_usage("unknown command");
}


int Mylogger::task_spawn(int argc, char *argv[])
{
	PX4_INFO("Starting mylogger module");
	px4_sleep(4);
	_task_id = px4_task_spawn_cmd("mylogger",
				      SCHED_DEFAULT,
				      SCHED_PRIORITY_DEFAULT,
				      1024,
				      (px4_main_t)&run_trampoline,
				      (char *const *)argv);

	if (_task_id < 0) {
		_task_id = -1;
		return -errno;
	}

	//calling SmartFS Initialization for logging
	PX4_INFO("Initializing SmartFS on /dev/mtdblock1, on MTD partition, mtd_mainstorage");
	initialize_and_mount_smartfs_for_logging(0, "/dev/mtdblock1", "blk1");

	return 0;
}

Mylogger *Mylogger::instantiate(int argc, char *argv[])
{
	int example_param = 0;
	bool example_flag = false;
	bool error_flag = false;

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

	Mylogger *instance = new Mylogger(example_param, example_flag);

	if (instance == nullptr) {
		PX4_ERR("alloc failed");
	}

	return instance;
}

Mylogger::Mylogger(int example_param, bool example_flag)
	: ModuleParams(nullptr)
{
}

void Mylogger::run()
{
	// Example: run the loop synchronized to the sensor_combined topic publication
	int sensor_combined_sub = orb_subscribe(ORB_ID(sensor_combined));

	px4_pollfd_struct_t fds[1];
	fds[0].fd = sensor_combined_sub;
	fds[0].events = POLLIN;

	// initialize parameters
	parameters_update(true);

	while (!should_exit()) {

		// wait for up to 1000ms for data
		int pret = px4_poll(fds, (sizeof(fds) / sizeof(fds[0])), 1000);

		if (pret == 0) {
			// Timeout: let the loop run anyway, don't do `continue` here

		} else if (pret < 0) {
			// this is undesirable but not much we can do
			PX4_ERR("poll error %d, %d", pret, errno);
			px4_usleep(50000);
			continue;

		} else if (fds[0].revents & POLLIN) {

			struct sensor_combined_s sensor_combined;
			orb_copy(ORB_ID(sensor_combined), sensor_combined_sub, &sensor_combined);
			// TODO: do something with the data...

		}

		parameters_update();
	}

	orb_unsubscribe(sensor_combined_sub);
}

void Mylogger::parameters_update(bool force)
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

int Mylogger::print_usage(const char *reason)
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

int mylogger_main(int argc, char *argv[])
{
	return Mylogger::main(argc, argv);
}


// Function to initialize and mount SmartFS for data logging
void initialize_and_mount_smartfs_for_logging(int mtd_instance,
						unsigned int partno,
						unsigned int smart_part)
{
    int mtd_fd;
    int ret;
    unsigned int num_instance;

    mtd_instance_s **instance = px4_mtd_get_instances(&num_instance);
    syslog(LOG_INFO, "[mylogger] reading available MTD instances\n");
    syslog(LOG_INFO, " [mylogger] No of instances: %i\n", num_instance);

    if(instance) {
	for (unsigned int i=0; i < num_instance; ++i) {
		if(instance[i]->mtd_dev) {
			syslog(LOG_INFO, "[mylogger] Flash Instance"
				" %i:\n", i);
			syslog(LOG_INFO, "[mylogger] No of Available"
				" partitions: %i\n", instance[i]->n_partitions_current);
			for (unsigned int p = 0; p < instance[i]->n_partitions_current; ++p) {
				syslog(LOG_INFO, "        Partition no: %i\n", p);
				syslog(LOG_INFO, "        Name: %s\n", instance[i]->partition_names[p]);
				syslog(LOG_INFO, "        Partition Block count (256bytes): %i\n", instance[i]->partition_block_counts[p]);
			}
			syslog(LOG_INFO, "[mylogger] Flash Device ID: %lx\n",  instance[i]->devid);
		}
	}
    }
    px4_sleep(4);

    // Initialize MTD
    mtd_fd = open((const char *)instance[mtd_instance]->partition_names[partno], O_RDWR);
    if (mtd_fd < 0) {
        syslog(LOG_ERR,"[mylogger] Error opening MTD device\n");
        return;
    }
    else
    {
	syslog(LOG_INFO, "[mylogger] MTD device %i, Partition '%s' opened\n", mtd_instance, instance[mtd_instance]->partition_names[partno]);

    }

    // Initialize SMARTFS
    syslog(LOG_INFO, "[mylogger] Initalizing SMARTFS,on mtd device %i,"
    			"on mtd_partition:, '%s'\n", mtd_instance,
			mtd_partName);
    px4_sleep(2);
    const char smart_partName[];
    sprintf(smart_partName, )
    ret = smart_initialize(mtd_instance, instance[mtd_instance]->part_dev[partno], smart_part);
    if (ret != OK) {
        syslog(LOG_ERR,"Error initializing SMARTFS\n");
        close(mtd_fd);
        return;
    }
    else
    {
	syslog(LOG_INFO, "SMARTFS Initialized\n");
	px4_sleep(2);
    }

    // Mount SMARTFS on the main storage partition
    ret = mount(NULL, "/mnt/smartfs", "smartfs", 0, NULL);
    if (ret != OK) {
        syslog(LOG_ERR,"Error mounting SMARTFS\n");
        // smart_uninitialize(mtd_fd);
        close(mtd_fd);
        return;
    }
    else
    {
	syslog(LOG_INFO, "Mounted SMARTFS\n");
	px4_sleep(2);
    }

    // Perform logging operations with SmartFS
    // ...

    // Unmount SMARTFS
    ret = umount("/mnt/smartfs");
    if (ret != OK) {
        syslog(LOG_ERR,"Error unmounting SMARTFS\n");
	px4_sleep(2);
    }

    // Uninitialize SMARTFS
//     smart_uninitialize(mtd_fd);

    // Close MTD device
    close(mtd_fd);
}
