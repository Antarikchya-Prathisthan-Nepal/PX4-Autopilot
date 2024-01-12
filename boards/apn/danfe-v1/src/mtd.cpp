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
static const px4_mft_device_t spi2 = {             // MT25QL on FMUM 1Gb 2048 X 64K
	.bus_type = px4_mft_device_t::SPI,
	.devid    = SPIDEV_FLASH(0)
};

static const px4_mtd_entry_t dnfe_FRAM = {
	.device = &spi2,
	.npart = 1,
	.partd = {
		{
			.type = MTD_MAINSTORAGE,		// storage space for  HKData logging
			.path = "/fs/mtd_mainstorage",
			// .nblocks = 51200				// 12.5MB in no of pages, each page having 256 bytes
			.nblocks = 128
		}
	},
};

static const px4_mtd_manifest_t board_mtd_config = {
	.nconfigs   = 1,
	.entries = {
		&dnfe_FRAM
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
