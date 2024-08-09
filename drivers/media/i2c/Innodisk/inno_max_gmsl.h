/*
 * inno_max_gmsl.h
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

#ifndef __INNO_MAX_GMSL_H__
#define __INNO_MAX_GMSL_H__

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
#if defined(_INNO_FOR_NVIDIA_)
#include "inno_common_nv.h"
#elif defined(_INNO_FOR_NXP_)
#include "inno_common_nxp.h"
#endif

int inno_max_gmsl_serdes_setup(struct inno_cam_mod *priv);
void inno_max_gmsl_serdes_reset(struct inno_cam_mod *priv);
int inno_max_gmsl_start_streaming(struct tegracam_device *tc_dev);
int inno_max_gmsl_stop_streaming(struct tegracam_device *tc_dev);
int inno_max_gmsl_board_setup(struct inno_cam_mod *priv);
int inno_max_gmsl_probe(struct inno_cam_mod *priv);
int inno_max_gmsl_remove(struct inno_cam_mod *priv);

int inno_max_gmsl_start(struct tegracam_device *tc_dev);
int inno_max_gmsl_stop(struct tegracam_device *tc_dev);
int inno_max_gmsl_init(struct inno_cam_mod *priv);

#endif /* __INNO_MAX_GMSL_H__ */
