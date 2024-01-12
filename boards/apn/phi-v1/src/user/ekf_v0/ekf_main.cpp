#include "ekf_main.h"
#include <poll.h>
#include <string.h>
#include <px4_platform_common/getopt.h>
#include <px4_platform_common/log.h>
#include <px4_platform_common/posix.h>

#include <uORB/topics/parameter_update.h>
#include <uORB/topics/sensor_combined.h>
#include <uORB/topics/sensor_accel.h>
#include <uORB/topics/vehicle_magnetometer.h>
#include "estimator_main.c"

int ekf_v0::print_status()
{
	PX4_INFO("Running ekf_v0");
	return 0;
}

int ekf_v0::custom_command(int argc, char *argv[])
{

	PX4_INFO("custom command argc:%i argv:%s", argc, argv[0]);
	char str[255];
	int  j = 0;
	char *c;
	//UART open
	int uart = uart_init("/dev/ttyS2"); //FTDI module commttyS2

	PX4_INFO("uart first :%i ", uart);

	if (false == uart) { return -1; }

	if (false == set_uart_baudrate(uart, 115200)) {
		printf("[YCM]set_uart_baudrate is failed\n");
		//     return -1;
	}

	PX4_INFO("UART %i", uart);
//     else{
	// if(!is_running()){
	// 	print_usage("phicommander is not working");
	// 	PX4_INFO("phicommander is not working");
	// 	return 1;
	// }
	int get_accelerometer_data = orb_subscribe(ORB_ID(sensor_combined));
	int get_temperature_data = orb_subscribe(ORB_ID(sensor_accel));
	int get_magnetometer_data = orb_subscribe(ORB_ID(vehicle_magnetometer));


	px4_pollfd_struct_t fds[] = {
		{ .fd = get_accelerometer_data,   .events = POLLIN },
		{ .fd = get_temperature_data,   .events = POLLIN },
		{ .fd = get_magnetometer_data,   .events = POLLIN },
		/* there could be more file descriptors here, in the form like:
		 * { .fd = other_sub_fd,   .events = POLLIN },
		 */
	};

	int poll_ret = px4_poll(fds, 3, 1000);

	struct sensor_combined_s raw;
	struct sensor_accel_s acceleration;
	struct vehicle_magnetometer_s mag;
	// Eekf ekf_1;
	imu_raw myImu;
	imu_filter filt_imu;
	sat_att_combined brd_att;
	// struct imu_raw *imu = &myImu;

	if (strcmp(argv[0], "accel") == 0) {
		PX4_INFO("Command accel received.");

		if (poll_ret > 0) {
			if (fds[0].revents & POLLIN) {
				orb_copy(ORB_ID(sensor_combined), get_accelerometer_data, &raw);

				//  struct Px4 *imu;

				for (int i = 0; i < 3; i++) {
					myImu.acc_mps2[i] = (float)raw.accelerometer_m_s2[i];
					myImu.gyr_rps[i] = (float)raw.gyro_rad[i];
				}

				// imu->acc_mps2[0] = (float)raw.accelerometer_m_s2[0];
				// imu->acc_mps2[1] = (float)raw.accelerometer_m_s2[1];
				// imu->acc_mps2[2] = (float)raw.accelerometer_m_s2[2];

				// imu->gyr_rps[0] = (float)raw.gyro_rad[0];
				// imu->gyr_rps[1] = (float)raw.gyro_rad[1];
				// imu->gyr_rps[2] = (float)raw.gyro_rad[2];

				// EKF myEkf;
				// EKF *ekf = &myEkf;
				// IMU_ReadAccelerometerComplete(imu);
				// IMU_ReadGyroscopeComplete(imu);
				// Comlementary_Filter(imu);

				filt_imu = IMU_RCFilter(&myImu);
				// EKF_AccelGyro(&myImu, &ekf);
				Attitude_ekfEstimate(&filt_imu, &brd_att);

				for (int i = 0; i < 3; i++) {
					switch (i) {
					case 0:
						sprintf(str, "Ax:%8.4f Gx : %8.4f\n\t", (double)raw.accelerometer_m_s2[0],
							(double)raw.gyro_rad[0]); //Data from accelerometer:\n
						break;

					case 1:
						sprintf(str, "Ay:%8.4f Gy:%8.4f\n\t", (double)raw.accelerometer_m_s2[1], (double)raw.gyro_rad[1]);
						break;

					case 2:
						sprintf(str, "Az:%8.4f Gz:%8.4f\n\t", (double)raw.accelerometer_m_s2[2], (double)raw.gyro_rad[2]);
						break;
					}

					PX4_INFO("%s", str);
					j = 0;
					//  char* c;
					// do {
					// 	c = &str[j];
					// 	j++;
					// 	write(uart, c, 1);
					// } while (*c != '\t');
				}
			}

			// close(uart);
		}

		if (argc > 2) {
			if (strcmp(argv[1], "n") == 0 || strcmp(argv[1], "-n") == 0 || strcmp(argv[1], "- n")) {
				{
					PX4_INFO("Got -n as second arg");
				}
			}
		}

		return 0;
	}

	else if (strcmp(argv[0], "gyro") == 0) {
		if (poll_ret > 0) {
			if (fds[0].revents & POLLIN) {
				orb_copy(ORB_ID(sensor_combined), get_accelerometer_data, &raw);

				for (int i = 0; i < 3; i++) {
					switch (i) {

					case 0:
						sprintf(str, "Data from Gyroscope  :\nGx:%8.4f\n\t", (double)raw.gyro_rad[0]);
						break;

					case 1:
						sprintf(str, "Gy:%8.4f\n\t", (double)raw.gyro_rad[1]);
						break;

					case 2:
						sprintf(str, "Gz:%8.4f\n\t", (double)raw.gyro_rad[2]);
						break;
					}

					j = 0;

					do {
						c = &str[j];
						j++;
						write(uart, c, 1);
					} while (*c != '\t');

					PX4_INFO("%s", str);
				}
			}
		}

		PX4_INFO("Got gyroscope data");
		return 0;

	}

	else if (strcmp(argv[0], "magnetometer") == 0) {
		PX4_INFO("poll ret :%i", poll_ret);

		if (poll_ret > 0) {
			if (fds[2].revents & POLLIN) {
				orb_copy(ORB_ID(vehicle_magnetometer), get_magnetometer_data, &mag);

				PX4_INFO("xmag :%f", (double)mag.magnetometer_ga[0]);

				for (int i = 0; i < 3; i++) {
					switch (i) {

					case 0:
						sprintf(str, "Data from magnetometer  :\nMagx:%8.4f\n\t", (double)mag.magnetometer_ga[0]);
						break;

					case 1:
						sprintf(str, "Magy:%8.4f\n\t", (double)mag.magnetometer_ga[1]);
						break;

					case 2:
						sprintf(str, "Magz:%8.4f\n\t", (double)mag.magnetometer_ga[2]);
						break;
					}

					j = 0;

					do {
						c = &str[j];
						j++;
						write(uart, c, 1);
					} while (*c != '\t');

					PX4_INFO("magnetometer %s", str);
				}
			}

		} else {
			PX4_INFO("Got some srror mag");
		}

		PX4_INFO("Got magnetometer data");
		return 0;

	}

	else if (strcmp(argv[0], "temperature") == 0 || strcmp(argv[0], "temp") == 0) {
		if (poll_ret > 0) {
			if (fds[1].revents & POLLIN) {
				orb_copy(ORB_ID(sensor_accel), get_temperature_data, &acceleration);

				sprintf(str, "System's Temperature : %8.4f C\n\t", (double)acceleration.temperature);
				j = 0;

				do {
					c = &str[j];
					j++;
					write(uart, c, 1);
				} while (*c != '\t');

				PX4_INFO("%s", str);
			}
		}
	}


	else {
		// PX4_INFO(strcmp(argv[0],"temperature"));
		PX4_WARN("Unknown cmd 123");
	}

	close(uart);

	return print_usage("unknown command");
}

int ekf_v0::task_spawn(int argc, char *argv[])
{
	_task_id = px4_task_spawn_cmd("module",
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

ekf_v0 *ekf_v0::instantiate(int argc, char *argv[])  //looks after -dash
{
	const char *myoptarg = nullptr;
	PX4_INFO("Instatiated");
	int ch;
	int myoptind = 1;
	int example_param = 0;
	bool example_flag = false;

	while ((ch = px4_getopt(argc, argv, "p:f", &myoptind, &myoptarg)) != EOF) {
		switch (ch) {
		case 'f': example_param = (int)strtol(myoptarg, nullptr, 10);
			/* code */
			PX4_INFO("gOT F AS AN ARG");
			break;

		case 'p': example_flag = true;
			/* code */
			break;

		case 'n': PX4_INFO("got n as arg");
			break;

		default:
			PX4_WARN("unrecognized flag");
			break;
		}
	}

	// if(error_flag){
	// 	return nullptr;
	// }
	ekf_v0 *instance = new ekf_v0(example_param, example_flag);

	if (instance == nullptr) {
		PX4_ERR("alloc failed");
	}

	return instance;
}

ekf_v0::ekf_v0(int example_param, bool example_flag)
	: ModuleParams(nullptr)
{

}

void ekf_v0::run()
{
	PX4_INFO("Commander module runs ");

}

int ekf_v0::print_usage(const char *reason)
{
	if (reason) {
		PX4_WARN("%s\n", reason);
	}

	PRINT_MODULE_DESCRIPTION(
		R"DESCR_STR(
		### Description
		Section that describes the provided module functionality.
		Phi Commander Module

		This is a template for a phi commander module running as a task in the background with start/stop/status functionality.

		### Implementation
		Section describing the high-level implementation of this module.

		### Examples
		CLI usage example:
		$ ekf_v0 start

		)DESCR_STR");

	PRINT_MODULE_USAGE_NAME("ekf_v0", "template");
	PRINT_MODULE_USAGE_COMMAND_DESCR("accel", "print accelerometer sensor data from satellite");
	PRINT_MODULE_USAGE_COMMAND_DESCR("gyro", "print gyro  data from satellite");
	PRINT_MODULE_USAGE_COMMAND_DESCR("temp", "print temperature  data from satellite");
	PRINT_MODULE_USAGE_DEFAULT_COMMANDS();

	return 0;
}

int ekf_v0_main(int argc, char *argv[])
{
	return ekf_v0::main(argc, argv);
}
