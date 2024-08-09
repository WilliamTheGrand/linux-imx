/*
 * inno_debug.h
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

#ifndef __INNO_DEBUG_H__
#define __INNO_DEBUG_H__

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

#define INNO_PRT(format, ...) \
	pr_info("[%s][%d]"format, __func__, __LINE__, ##__VA_ARGS__)
#define INNO_DEV_PRT(dev, format, ...) \
	dev_info(dev, "[%s][%d]"format, __func__, __LINE__, ##__VA_ARGS__)
#define INNO_DEV_ERR_PRT(dev, format, ...) \
	dev_err(dev, "[%s][%d]"format, __func__, __LINE__, ##__VA_ARGS__)

#define INNO_ALERT_EN					1
#define INNO_ERR_EN						1
#define INNO_WARN_EN					0
#define INNO_DBG_EN						0
#define INNO_IIC_EN						0
#define INNO_MAX_GMSL_EN				0

#define INNO_PREVIEW_OUT_FMT_EN			0
#define INNO_SYS_START_EN				0

#if INNO_ALERT_EN
#define INNO_ALERT						INNO_PRT
#define INNO_DEV_ALERT					INNO_DEV_PRT
#else
#define INNO_ALERT(fmt, args...) \
	do {} while (0)
#define INNO_DEV_ALERT(dev, fmt, args...) \
	do {} while (0)
#endif

#if INNO_ERR_EN
#define INNO_ERR						INNO_PRT
#define INNO_DEV_ERR					INNO_DEV_ERR_PRT
#else
#define INNO_ERR(fmt, args...) \
	do {} while (0)
#define INNO_DEV_ERR(dev, fmt, args...) \
	do {} while (0)
#endif

#if INNO_WARN_EN
#define INNO_WARN						INNO_PRT
#define INNO_DEV_WARN					INNO_DEV_PRT
#else
#define INNO_WARN(fmt, args...) \
	do {} while (0)
#define INNO_DEV_WARN(dev, fmt, args...) \
	do {} while (0)
#endif

#if INNO_DBG_EN
#define INNO_DBG						INNO_PRT
#define INNO_DEV_DBG					INNO_DEV_PRT
#else
#define INNO_DBG(fmt, args...) \
	do {} while (0)
#define INNO_DEV_DBG(dev, fmt, args...) \
	do {} while (0)
#endif

#if INNO_IIC_EN
#define INNO_IIC						INNO_PRT
#define INNO_DEV_IIC					INNO_DEV_PRT
#else
#define INNO_IIC(fmt, args...) \
	do {} while (0)
#define INNO_DEV_IIC(dev, fmt, args...) \
	do {} while (0)
#endif

#if INNO_MAX_GMSL_EN
#define INNO_GMSL						INNO_PRT
#define INNO_DEV_GMSL					INNO_DEV_PRT
#else
#define INNO_GMSL(fmt, args...) \
	do {} while (0)
#define INNO_DEV_GMSL(dev, fmt, args...) \
	do {} while (0)
#endif

#define INNO_RET_ERR(ret)				INNO_ERR("(%d)\n", ret)
#define INNO_RET_ERR_HANDLE(ret) do { \
	if (ret) { \
		INNO_RET_ERR(ret); \
		return ret; \
	} \
} while(0);

#endif /* __INNO_DEBUG_H__ */
