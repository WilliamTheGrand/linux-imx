/*
 * inno_common_nv.h
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

#ifndef __INNO_COMMON_NV_H__
#define __INNO_COMMON_NV_H__

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
/* Use general GPIO instead of I2C GPIO Expander on EVDM-OOM1 */
#include "../../platform/tegra/camera/camera_gpio.h"
#endif

void inno_comm_nv_toggle_gpio(u32 gpio, s32 val);
int inno_comm_gpio_level(struct inno_cam_mod *priv, int bit, s32 val);

int inno_comm_nv_set_gain(struct tegracam_device *tc_dev, s64 val);
int inno_comm_nv_set_exposure(struct tegracam_device *tc_dev, s64 val);
int inno_comm_nv_set_frame_rate(struct tegracam_device *tc_dev, s64 val);
int inno_comm_nv_set_group_hold(struct tegracam_device *tc_dev, bool val);
int inno_comm_nv_fill_string_ctrl(struct tegracam_device *tc_dev, struct v4l2_ctrl *ctrl);

int inno_comm_nv_write_table_8(struct inno_cam_mod *priv, const INNO_COMM_REG_8 table[]);
int inno_comm_nv_write_table_16(struct inno_cam_mod *priv, const INNO_COMM_REG_16 table[]);

int inno_comm_nv_power_on(struct camera_common_data *s_data);
int inno_comm_nv_power_off(struct camera_common_data *s_data);
int inno_comm_nv_power_get(struct tegracam_device *tc_dev);
int inno_comm_nv_power_put(struct tegracam_device *tc_dev);
int inno_comm_nv_read_reg(struct camera_common_data *s_data, u16 addr, u8 *val);
int inno_comm_nv_write_reg(struct camera_common_data *s_data, u16 addr, u8 val);
struct camera_common_pdata *inno_comm_nv_parse_dt(struct tegracam_device *tc_dev);

#endif /* __INNO_COMMON_NV_H__ */
