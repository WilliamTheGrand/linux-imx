/*
 * inno_common_reg.h
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

#ifndef __INNO_COMMON_REG_H__
#define __INNO_COMMON_REG_H__

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

/* INNO_EV2M_OOM1 & INNO_EVDDM_OOM1 ISP register address */
#define ATOMIC								0x1184
#define CHIP_VERSION_REG					0x0000
#define SIPS_CRC							0xF052
#define BOOTDATA_STAGE						0x6002
#define HINF_MIPI_FREQ_TGT					0x6034
#define HINF_MIPI_FREQ						0x0068
#define THROUGHPUT_LIMIT					0x6132
#define BOOTDATA_CHECKSUM					0x6134
#define MIN_FW_BLANK_TIME					0x6124

#define ATOMIC_RECORD						BIT(0)
#define ATOMIC_FINISH						BIT(1)
#define ATOMIC_MODE							BIT(2)
#define ATOMIC_UPDATE_FORMAT				BIT(3)
#define ATOMIC_UPDATE						BIT(4)
#define ATOMIC_UPDATE_ALL					(ATOMIC_UPDATE | ATOMIC_UPDATE_FORMAT | ATOMIC_MODE | ATOMIC_FINISH | ATOMIC_RECORD)
#define ATOMIC_UPDATE_ONSEMI				(ATOMIC_UPDATE | ATOMIC_FINISH | ATOMIC_RECORD)
#define ATOMIC_UPDATE_NXP					(ATOMIC_UPDATE_FORMAT | ATOMIC_FINISH | ATOMIC_RECORD)

#define SENSOR_MODE(X)						(X)

#define PREVIEW_MIPI_CTRL					0x2016
#define PREVIEW_WIDTH						0x2000
#define PREVIEW_HEIGHT						0x2002
#define PREVIEW_ROI_X0						0x2004
#define PREVIEW_ROI_Y0						0x2006
#define PREVIEW_OUT_FMT						0x2012
#define PREVIEW_SENSOR_MODE					0x2014
#define PREVIEW_HINF_CTRL					0x2030
#define PREVIEW_HINF_SPOOF_W				0x2032
#define PREVIEW_HINF_SPOOF_H				0x2034
#define PREVIEW_MAX_FPS						0x2020
#define REVIEW_AE_MAX_ET					0x2028

#define PREVIEW_OUT_FMT_YUV_BT601_422		0x3000
#define PREVIEW_OUT_FMT_YUV_BT601_420		0x3100
#define PREVIEW_OUT_FMT_YUV_BT601_400		0x3200
#define PREVIEW_OUT_FMT_YUV_JFIF_422		0x5000
#define PREVIEW_OUT_FMT_YUV_JFIF_420		0x5100
#define PREVIEW_OUT_FMT_YUV_JFIF_400		0x5200

#define VIDEO_MIPI_CTRL						0x4016

#define SYS_START							0x601A
#define SYS_START_PLL_INIT					BIT(0)
#define SYS_START_PATCH_RUN					BIT(1)
#define SYS_START_GO						BIT(4)
#define SYS_START_STALL_MD_STALL_STMEAM 	(0U << 6)
#define SYS_START_STALL_MD_STALL_STOP	 	(1U << 6)
#define SYS_START_STALL_MD_STBY_STOP 		(2U << 6)
#define SYS_START_STALL_MD_STBY_SD	 		(3U << 6)
#define SYS_START_STALL_EN					BIT(8)
#define SYS_START_STALL_STATUS				BIT(9)
#define SYS_START_RESTART_ERR				BIT(11)
#define SYS_START_LOAD_OTP					BIT(12)
#define SYS_START_PLL_LOCK					BIT(15)
#define SYS_START_COMMON					(SYS_START_STALL_MD_STBY_STOP)
#define SYS_START_START_STREAM				(SYS_START_STALL_MD_STALL_STMEAM | SYS_START_STALL_EN | SYS_START_STALL_STATUS)
#define SYS_START_STOP_STREAM				(SYS_START_STALL_MD_STBY_STOP | SYS_START_STALL_EN)

#define DMA_SIZE							0x60A8
#define DMA_SRC								0x60A0
#define DMA_DST								0x60A4
#define DMA_CTRL							0x60AC

#define DMA_SIP_SIPM_0						0
#define DMA_SIP_SIPM_1						1

#define DMA_SIP_SIPM(n)						((n) << 26)
#define DMA_SIP_DATA_16_BIT					BIT(25)
#define DMA_SIP_ADDR_16_BIT					BIT(24)
#define DMA_SIP_ID(n)						((n) << 16)
#define DMA_SIP_REG(n)						((n) << 0)

#define DMA_CTRL_SCH_NORMAL					(0 << 12)
#define DMA_CTRL_SCH_NEXT					(1 << 12)
#define DMA_CTRL_SCH_NOW					(2 << 12)
#define DMA_CTRL_DST_REG					(0 << 8)
#define DMA_CTRL_DST_SRAM					(1 << 8)
#define DMA_CTRL_DST_SPI					(2 << 8)
#define DMA_CTRL_DST_SIP					(3 << 8)
#define DMA_CTRL_SRC_REG					(0 << 4)
#define DMA_CTRL_SRC_SRAM					(1 << 4)
#define DMA_CTRL_SRC_SPI					(2 << 4)
#define DMA_CTRL_SRC_SIP					(3 << 4)
#define DMA_CTRL_MODE_32_BIT				BIT(3)
#define DMA_CTRL_MODE_MASK					(7 << 0)
#define DMA_CTRL_MODE_IDLE					(0 << 0)
#define DMA_CTRL_MODE_SET					(1 << 0)
#define DMA_CTRL_MODE_COPY					(2 << 0)
#define DMA_CTRL_MODE_MAP					(3 << 0)
#define DMA_CTRL_MODE_UNPACK				(4 << 0)
#define DMA_CTRL_MODE_OTP_READ				(5 << 0)
#define DMA_CTRL_MODE_SIP_PROBE				(6 << 0)

#define PREVIEW_HINF_CTRL_CONT_CLK_MASK		(1 << 5)
#define PREVIEW_HINF_CTRL_DIS_CONT_CLK		(0 << 5)
#define PREVIEW_HINF_CTRL_EN_CONT_CLK		(1 << 5)
#define PREVIEW_HINF_CTRL_MIPI_MASK			(7 << 0)
#define PREVIEW_HINF_CTRL_MIPI_1			(1 << 0)
#define PREVIEW_HINF_CTRL_MIPI_2			(2 << 0)
#define PREVIEW_HINF_CTRL_MIPI_3			(3 << 0)
#define PREVIEW_HINF_CTRL_MIPI_4			(4 << 0)

#define POLL_REG_DELAY						10
#define POLL_REG_TIMEOUT					100
#define POLL_REG_COUNT						(POLL_REG_TIMEOUT / POLL_REG_DELAY)
#define EXPECTED_BOOTDATA_STAGE				0xFFFF
#define NO_EXPECTED_BOOTDATA_CHECKSUM		0x0000
#define HINF_MIPI_FREQ_TGT_DEFAULT			0x4000

#define AE_CTRL								0x5002
#define AE_CTRL_MODE_MASK					0x000F
#define AE_MANUAL_GAIN						0x5006
#define AE_MANUAL_EXPOSURE					0x500C
#define AWB_CTRL							0x5100
#define AWB_TEMP_CTRL						0x510A
#define AWB_CTRL_MODE_MASK					0x000F

/* Advanced System Registers */
#define ADV_IRQ_SYS_INTE					0x00230000
#define ADV_IRQ_SYS_INTE_TEST_COUNT			BIT(25)
#define ADV_IRQ_SYS_INTE_HINF_1				BIT(24)
#define ADV_IRQ_SYS_INTE_HINF_0				BIT(23)
#define ADV_IRQ_SYS_INTE_SINF_B_MIPI_L		(7U << 20)
#define ADV_IRQ_SYS_INTE_SINF_B_MIPI		BIT(19)
#define ADV_IRQ_SYS_INTE_SINF_A_MIPI_L		(15U << 14)
#define ADV_IRQ_SYS_INTE_SINF_A_MIPI		BIT(13)
#define ADV_IRQ_SYS_INTE_SINF				BIT(12)
#define ADV_IRQ_SYS_INTE_IPIPE_S			BIT(11)
#define ADV_IRQ_SYS_INTE_IPIPE_B			BIT(10)
#define ADV_IRQ_SYS_INTE_IPIPE_A			BIT(9)
#define ADV_IRQ_SYS_INTE_IP					BIT(8)
#define ADV_IRQ_SYS_INTE_TIMER				BIT(7)
#define ADV_IRQ_SYS_INTE_SIPM				(3U << 6)
#define ADV_IRQ_SYS_INTE_SIPS_ADR_RANGE		BIT(5)
#define ADV_IRQ_SYS_INTE_SIPS_DIRECT_WRITE	BIT(4)
#define ADV_IRQ_SYS_INTE_SIPS_FIFO_WRITE	BIT(3)
#define ADV_IRQ_SYS_INTE_SPI				BIT(2)
#define ADV_IRQ_SYS_INTE_GPIO_CNT			BIT(1)
#define ADV_IRQ_SYS_INTE_GPIO_PIN			BIT(0)

#define GENESYS_CHIP_ID						0x0A50
#define GENESYS_DISPLAY_MODE				0x0A42
#define GENESYS_DEFAULT_DISPLAY_MODE		0x0A46

#endif /* __INNO_COMMON_REG_H__ */
