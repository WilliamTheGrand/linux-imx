/*
 * inno_iic.h
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

#ifndef __INNO_IIC_H__
#define __INNO_IIC_H__

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

/* IO Expander device addresses */
#define INNO_II2_IO_EXP_IIC_ADDR		0x41
#define INNO_II2_IO_EXP_IIC_ADDR_UNDEF	0xFF

/* IO Expander register addresses */
#define INNO_II2_IO_EXP_OUTPUT			0x01
#define INNO_II2_IO_EXP_CONFIG			0x03

/* IO Expander output GPIOs */
#define INNO_II2_IO_EXP_PWR_EN			0
#define INNO_II2_IO_EXP_RSV				1
#define INNO_II2_IO_EXP_RST				2
#define INNO_II2_IO_EXP_STANDBY			3

#define INNO_II2_IO_EXP_GPIO(X)			BIT(X)

inline int inno_ii2_hw_rd_reg(struct inno_cam_mod *priv, u16 reg, u16 *val);
inline int inno_iic_hw_wr_reg(struct inno_cam_mod *priv, u16 reg, u16 val);

inline int inno_iic_rd_reg(struct inno_cam_mod *priv, u16 addr, u32 *val, u8 len);
inline int inno_iic_wr_reg(struct inno_cam_mod *priv, u16 addr, u32 val, u8 len);

inline int inno_iic_io_exp_rd_reg(struct inno_cam_mod *priv, u8 addr, u32 *val);
inline int inno_iic_io_exp_wr_reg(struct inno_cam_mod *priv, u8 addr, u8 val);
int inno_iic_io_exp_device_release(struct inno_cam_mod *priv);
int inno_iic_io_exp_device_init(struct inno_cam_mod *priv);
int inno_iic_io_exp_level(struct inno_cam_mod *priv, int bit, int val);
int inno_iic_io_exp_rst(struct inno_cam_mod *priv);

#endif /* __INNO_MAX_GMSL_H__ */
