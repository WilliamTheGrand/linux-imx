/*
 * inno_common_nxp.c - Innodisk Common Function
 *
 * Copyright (c) 2022-2024, Innodisk CORPORATION.  All rights reserved.
 * All Rights Reserved.
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms and conditions of the GNU General Public License,
 * version 2, as published by the Free Software Foundation.
 *
 * This program is distributed in the hope it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
 * more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#include <linux/version.h>
#include <linux/clk.h>
#include <linux/delay.h>
#include <linux/device.h>
#include <linux/init.h>
#include <linux/module.h>
#include <linux/slab.h>
#include <linux/uaccess.h>
#include <linux/gpio.h>
#include <linux/module.h>
#include <linux/seq_file.h>
#include <linux/of.h>
#include <linux/of_device.h>
#include <linux/of_gpio.h>
#include <linux/pinctrl/consumer.h>
#include <linux/regulator/consumer.h>
#include <linux/regmap.h>
#include <linux/firmware.h>
#include <linux/i2c.h>
#include <linux/v4l2-mediabus.h>
#include <media/v4l2-device.h>
#include <media/v4l2-ctrls.h>
#if defined(_INNO_FOR_NVIDIA_)
#include <media/tegra_v4l2_camera.h>
#include <media/tegracam_core.h>
#include <media/max9295.h>
#include <media/max9296.h>
#endif

#include "inno_debug.h"
#include "inno_common.h"
#include "inno_common_reg.h"
#include "inno_common_data.h"
#include "inno_common_mode_tbls.h"
#if defined(_INNO_FOR_NVIDIA_)
#include "inno_max_gmsl.h"
#include "inno_common_nv.h"
#elif defined(_INNO_FOR_NXP_)
#include "inno_common_nxp.h"
#endif
#include "inno_iic.h"

#if defined(_INNO_FOR_NVIDIA_)
/* Use general GPIO instead of I2C GPIO Expander on EVDM-OOM1 */
#include "../../platform/tegra/camera/camera_gpio.h"
#endif

int inno_comm_gpio_level(struct inno_cam_mod *priv, int bit, int val)
{
	int ret = INNO_RET_OK;

	INNO_DBG("Bit#%d: %s\n", bit, (val == INNO_GPIO_LEVEL_HIGH) ? "HIGH" : "LOW");

	switch (bit) {
		case INNO_II2_IO_EXP_RST:
			gpiod_set_value_cansleep(priv->reset, val);
			break;

		case INNO_II2_IO_EXP_PWR_EN:
		case INNO_II2_IO_EXP_STANDBY:
			gpiod_set_value_cansleep(priv->isp_en, val);
			break;

		default:
			break;
	}
	msleep(INNO_GPIO_LEVEL_DELAY_MS);

	return ret;
}
