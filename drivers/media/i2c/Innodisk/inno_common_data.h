/*
 * inno_common_data.h
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

#ifndef __INNO_COMMON_DATA_H__
#define __INNO_COMMON_DATA_H__

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

#define INNO_AUTHOR						"Innodisk Corporation"
#define INNO_LICENSE					"GPL v2"
#define INNO_VERSION					"0.9"
#define INNO_DESCRIPTION(x)				"Media Controller driver for Innodisk "x

#define INNO_RET_OK						0

#define INNO_GPIO_LEVEL_LOW				0
#define INNO_GPIO_LEVEL_HIGH			1
#define INNO_GPIO_LEVEL_DELAY_MS		100
#define INNO_GPIO_IO_DELAY_MS			500
#define INNO_GPIO_RST_DELAY_MS			10

#define INNO_NUM_CONSUMERS				3
#define INNO_FW_DESCRIPTION				256
#define INNO_MBUS_CODE					MEDIA_BUS_FMT_UYVY8_2X8

#define INNO_SENS_PAD_SOURCE			0
#define INNO_SENS_PADS_NUM				1

#define INNO_IIC_VAL_08BIT				1
#define INNO_IIC_VAL_16BIT				2
#define INNO_IIC_VAL_24BIT				3
#define INNO_IIC_VAL_32BIT				4

/* INNO_EVXM_OOM1 - Sensor Parameter Limits */
#define INNO_FW_WINDOW_OFFSET			0x8000
#define INNO_FW_WINDOW_SIZE				0x2000
#define INNO_FW_BUFFER					0x0FF0

#define INNO_EV2M_GOM1_DEVICE_NAME		"EV2M_GOM1"
#define INNO_EV2M_OOM1_DEVICE_NAME		"EV2M_OOM1"
#define INNO_EVDM_OOM1_DEVICE_NAME		"EVDM_OOM1"
#define INNO_EV2M_OOM2_DEVICE_NAME		"EV2M_OOM2"
#define INNO_EV8M_OOM1_DEVICE_NAME		"EV8M_OOM1"

/* Compatible Device ID */
#define INNO_EV2M_GOM1_DEVICE_ID		"inno,ev2m-gom1"
#define INNO_EV2M_OOM1_DEVICE_ID		"inno,ev2m-oom1"
#define INNO_EVDM_OOM1_DEVICE_ID		"inno,evdm-oom1"
#define INNO_EV2M_OOM2_DEVICE_ID		"inno,ev2m-oom2"
#define INNO_EV8M_OOM1_DEVICE_ID		"inno,ev8m-oom1"

/* ISP Module ID */
#define INNO_EV2M_GOM1_MODULE_ID		0x3004
#define INNO_EV2M_OOM1_MODULE_ID		0x0265
#define INNO_EVDM_OOM1_MODULE_ID		INNO_EV2M_OOM1_MODULE_ID
#define INNO_EV2M_OOM2_MODULE_ID		INNO_EV2M_OOM1_MODULE_ID
#define INNO_EV8M_OOM1_MODULE_ID		INNO_EV2M_OOM1_MODULE_ID

/* Sensor ID */
#define INNO_EV2M_GOM1_SENSOR_ID		0x2604
#define INNO_EV2M_OOM1_SENSOR_ID		0x0A56
#define INNO_EVDM_OOM1_SENSOR_ID		0x0153
#define INNO_EV2M_OOM2_SENSOR_ID		0x0856
#define INNO_EV8M_OOM1_SENSOR_ID		0x0F56

/* INNO_EV2M_OOM1 & INNO_EVDDM_OOM1 Sensor register address */
#define INNO_EV2M_GOM1_SEN_CHIP_VER_REG	0x0A50
#define INNO_EV2M_OOM1_SEN_CHIP_VER_REG	0x3000
#define INNO_EVDM_OOM1_SEN_CHIP_VER_REG	0x0000
#define INNO_EV2M_OOM2_SEN_CHIP_VER_REG	INNO_EV2M_OOM1_SEN_CHIP_VER_REG
#define INNO_EV8M_OOM1_SEN_CHIP_VER_REG	INNO_EV2M_OOM1_SEN_CHIP_VER_REG

/* ISP SIP ID */
#define INNO_NULL_SIP_ID				0xFF
#define INNO_EV2M_GOM1_SIP_ID			INNO_NULL_SIP_ID
#define INNO_EV2M_OOM1_SIP_ID			0x20
#define INNO_EVDM_OOM1_SIP_ID			0x6C
#define INNO_EV2M_OOM2_SIP_ID			0x20
#define INNO_EV8M_OOM1_SIP_ID			0x20

/* ISP SIP DMA */
#define INNO_EV2M_OOM1_DMA_SIZE_2_BYTE	2

#define INNO_MIPI_NONE_CONTINUOUS_MODE	0
#define INNO_MIPI_CONTINUOUS_MODE		1

#if defined(_INNO_FOR_NVIDIA_)
#define INNO_EV2M_GOM1_MODE				INNO_MIPI_CONTINUOUS_MODE
#define INNO_EV2M_OOM1_MODE				INNO_MIPI_CONTINUOUS_MODE
#define INNO_EVDM_OOM1_MODE				INNO_MIPI_CONTINUOUS_MODE
#define INNO_EV2M_OOM2_MODE				INNO_MIPI_CONTINUOUS_MODE
#define INNO_EV8M_OOM1_MODE				INNO_MIPI_CONTINUOUS_MODE
#elif defined(_INNO_FOR_NXP_)
#define INNO_EV2M_GOM1_MODE				INNO_MIPI_NONE_CONTINUOUS_MODE
#define INNO_EV2M_OOM1_MODE				INNO_MIPI_NONE_CONTINUOUS_MODE
#define INNO_EVDM_OOM1_MODE				INNO_MIPI_NONE_CONTINUOUS_MODE
#define INNO_EV2M_OOM2_MODE				INNO_MIPI_NONE_CONTINUOUS_MODE
#define INNO_EV8M_OOM1_MODE				INNO_MIPI_NONE_CONTINUOUS_MODE
#endif

#define INNO_MIPI_LANES_1_LANES			1
#define INNO_MIPI_LANES_2_LANES			2
#define INNO_MIPI_LANES_4_LANES			4
#define INNO_MIPI_LANES					INNO_MIPI_LANES_4_LANES	// 1, 2, 4 Lanes

/* EV2M-GOM1 Firmware binary */
#if INNO_EV2M_GOM1_MODE == INNO_MIPI_CONTINUOUS_MODE
#define INNO_EV2M_GOM1_FW_NAME			""
#else
#define INNO_EV2M_GOM1_FW_NAME			""
#endif

/* EV2M-OOM1 Firmware binary */
#if INNO_EV2M_OOM1_MODE == INNO_MIPI_CONTINUOUS_MODE
#define INNO_EV2M_OOM1_FW_NAME			"Innodisk_EV2M_OOM1_CONT.bin"
#else
#define INNO_EV2M_OOM1_FW_NAME			"Innodisk_EV2M_OOM1.bin"
#endif

/* EVDM-OOM1 Firmware binary */
#if INNO_MIPI_LANES == INNO_MIPI_LANES_2_LANES
#define INNO_EVDM_OOM1_FW_NAME			"Innodisk_EVDM_OOM1_CONT_2LANE.bin"
#else
#if INNO_EVDM_OOM1_MODE == INNO_MIPI_CONTINUOUS_MODE
#define INNO_EVDM_OOM1_FW_NAME			"Innodisk_EVDM_OOM1_CONT.bin"
#else
#define INNO_EVDM_OOM1_FW_NAME			"Innodisk_EVDM_OOM1_600.bin" // For NXP i.MX 8M Plus debug
#endif
#endif

/* EV2M-OOM2 Firmware binary */
#if INNO_EV2M_OOM2_MODE == INNO_MIPI_CONTINUOUS_MODE
#define INNO_EV2M_OOM2_FW_NAME			"Innodisk_EV2M_OOM2_CONT.bin"
#else
#define INNO_EV2M_OOM2_FW_NAME			"Innodisk_EV2M_OOM2_600.bin" // For NXP i.MX 8M Plus debug
#endif

/* EV8M-OOM1 Firmware binary */
#if INNO_EV8M_OOM1_MODE == INNO_MIPI_CONTINUOUS_MODE
#define INNO_EV8M_OOM1_FW_NAME			"Innodisk_EV8M_OOM1_CONT.bin"
#else
#define INNO_EV8M_OOM1_FW_NAME			"Innodisk_EV8M_OOM1_600.bin" // For NXP i.MX 8M Plus debug
#endif

#define INNO_CTRL_HDL_NUM				6

#endif /* __INNO_COMMON_DATA_H__ */
