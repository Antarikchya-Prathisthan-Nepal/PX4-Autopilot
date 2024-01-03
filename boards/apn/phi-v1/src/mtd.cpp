/****************************************************************************
 *
 *   Copyright (C) 2022 PX4 Development Team. All rights reserved.
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
#include <nuttx/config.h>
#include <nuttx/fs/fs.h>
#include <nuttx/mtd/mtd.h>
#include <px4_platform_common/px4_mtd.h>
#include <../../platforms/nuttx/NuttX/nuttx/fs/littlefs/lfs_vfs.h>
#include <sys/mount.h>
#include <px4_platform_common/log.h>
#include <px4_platform_common/spi.h>

#include <nuttx/spi/spi.h>
#include <px4_platform_common/px4_manifest.h>



#include <px4_platform_common/px4_config.h>
#include <px4_platform_common/log.h>
#include <px4_platform_common/spi.h>

#include <inttypes.h>
#include <errno.h>
#include <stdbool.h>
#include "systemlib/px4_macros.h"

#include <nuttx/drivers/drivers.h>
#include <nuttx/spi/spi.h>
#include <nuttx/mtd/mtd.h>
#include <nuttx/fs/fs.h>


//                                                              KiB BS    nB
static const px4_mft_device_t spi3 = {             // MT25QL on FMUM 1Gb 2048 X 64K
	.bus_type = px4_mft_device_t::SPI,
	.devid    = SPIDEV_FLASH(0)
};
static const px4_mft_device_t spi4 = {             // MT25QL on FMUM 1Gb 2048 X 64K
	.bus_type = px4_mft_device_t::SPI,
	.devid    = SPIDEV_FLASH(0)
};

static const px4_mtd_entry_t phi_mfm = {
	.device = &spi3,
	.npart = 2,
	.partd = {
		{
			.type = MTD_PARAMETERS,
			.path = "/fs/mfm/mtd_params",
			.nblocks = 128				// this is no of pages to provide
		},
		{
			.type = MTD_MAINSTORAGE,		// storage space for  HKData logging
			.path = "/fs/mfm/mtd_mainstorage",
			// .nblocks = 51200				// 12.5MB in no of pages, each page having 256 bytes
			.nblocks = 524160
		}
		// {
		// 	.type = MTD_SAT_LOG,			// satellite log storage
		// 	.path = "/fs/mfm/mtd_sat_log",
		// 	.nblocks = 51200 			// 12.5MB in no of pages, each page having 256 bytes
		// },
		// {
		// 	.type = MTD_BEACON,			// Storing Beacon data to be transmitted
		// 	.path = "/fs/mfm/mtd_beacon",
		// 	.nblocks = 51200			// 12.5MB in no of pages, each page having 256 bytes
		// },
		// {
		// 	.type = MTD_MISSION,			// Partition for storing MSN data
		// 	.path = "/fs/mfm/mtd_mission",
		// 	.nblocks = 370304			// 90.4 MB in no of pages, each pages having 256 bytes
		// },
		// {
		// 	.type = MTD_FLAGS,			// Partition for storing Critical MSN Flags
		// 	.path = "/fs/mfm/mtd_flags",
		// 	.nblocks = 128				// 12.5MB in no of pages, each page having 256 bytes
		// },
		// {
		// 	.type = MTD_RSV_TABLE,			// Partition for storing Reservation commands
		// 	.path = "/fs/mfm/mtd_rsv_table",
		// 	.nblocks = 128				// 12.5MB in no of pages, each page having 256 bytes
		// }
	},
};

static const px4_mtd_entry_t phi_sfm = {
	.device = &spi4,
	.npart = 2,
	.partd = {
		{
			.type = MTD_PARAMETERS,
			.path = "/fs/sfm/mtd_params",
			.nblocks = 128				// this is no of pages to provide
		},
		// {
		// 	.type = MTD_SAT_LOG,			// satellite log storage
		// 	.path = "/fs/sfm/mtd_sat_log",
		// 	.nblocks = 51200 			// 12.5MB in no of pages, each page having 256 bytes
		// },
		{
			.type = MTD_MAINSTORAGE,			// Partition for storing MSN data
			.path = "/fs/sfm/mtd_mission",
			.nblocks = 524160			// 90.4 MB in no of pages, each pages having 256 bytes
		}
		// {
		// 	.type = MTD_FLAGS,			// Partition for storing Critical MSN Flags
		// 	.path = "/fs/sfm/mtd_flags",
		// 	.nblocks = 128				// 12.5MB in no of pages, each page having 256 bytes
		// }
	},
};

static const px4_mtd_manifest_t board_mtd_config = {
	.nconfigs   = 2,
	.entries = {
		&phi_mfm,
		&phi_sfm,
	}
};

static const px4_mft_entry_s mtd_mft = {
	.type = MTD,
	.pmft = (void *) &board_mtd_config,
};

static const px4_mft_s mft = {
	.nmft = 1,
	.mfts = {
		&mtd_mft
	}
};

const px4_mft_s *board_get_manifest(void)
{
	return &mft;
}

// int  *mount_mtd_littlefs(mtd_instance_s *instances, uint8_t part, int rv, char *blockname)
// {
// 	char mount_point[42];

// 	rv = register_mtddriver(blockname, instances->part_dev[part], 0755, nullptr);

// 	if (rv < 0) {
// 		PX4_ERR("MTD driver %s failed: %d", blockname, rv);
// 		return rv;
// 	}

// 	snprintf(mount_point, sizeof(mount_point), "/mnt%s", instances->partition_names[part]);
// 	rv = nx_mount(blockname, mount_point, "littlefs", 0, "");
// 	printf("nx_mount: blockname: %s partition: %s mount_point: %s\n", blockname, instances->partition_names[part],
// 	       mount_point);
// 	if(rv < 0) {
// 		PX4_ERR("nx_mount %s on mount point: %s failed: %d", instances->partition_names[part], mount_point, rv);
// 		return rv;
// 	}
// 	return rv;
// }
