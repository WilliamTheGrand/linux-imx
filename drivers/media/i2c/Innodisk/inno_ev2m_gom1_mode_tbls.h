/*
 * inno_ev2m_gom1_mode_tbls.h
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

#ifndef __INNO_EV2M_GOM1_MODE_H__
#define __INNO_EV2M_GOM1_MODE_H__

#define INNO_EVXM_YYMZ_MODE_COMMON 			INNO_EV2M_GOM1_MODE_COMMON
#define INNO_EVXM_YYMZ_START_STREAM 		INNO_EV2M_GOM1_START_STREAM
#define INNO_EVXM_YYMZ_STOP_STREAM 			INNO_EV2M_GOM1_STOP_STREAM

#include "inno_common_mode_tbls.h"

INNO_COMM_REG_8 inno_ev2m_gom1_mode_00_1920x1080_30fps[] = {
	{GENESYS_DISPLAY_MODE,	GENESYS_DISPLAY_MODE_00},
	{INNO_COMM_TABLE_END,	0x00}
};

INNO_COMM_REG_8 inno_ev2m_gom1_mode_01_1920x1080_30fps[] = {
	{GENESYS_DISPLAY_MODE,	GENESYS_DISPLAY_MODE_01},
	{INNO_COMM_TABLE_END,	0x00}
};

INNO_COMM_REG_8 inno_ev2m_gom1_mode_02_1920x1080_30fps[] = {
	{GENESYS_DISPLAY_MODE,	GENESYS_DISPLAY_MODE_02},
	{INNO_COMM_TABLE_END,	0x00}
};

INNO_COMM_REG_8 inno_ev2m_gom1_mode_03_1920x1080_30fps[] = {
	{GENESYS_DISPLAY_MODE,	GENESYS_DISPLAY_MODE_03},
	{INNO_COMM_TABLE_END,	0x00}
};

INNO_COMM_REG_8 inno_ev2m_gom1_mode_04_1920x1080_30fps[] = {
	{GENESYS_DISPLAY_MODE,	GENESYS_DISPLAY_MODE_04},
	{INNO_COMM_TABLE_END,	0x00}
};

INNO_COMM_REG_8 inno_ev2m_gom1_mode_05_1920x1080_30fps[] = {
	{GENESYS_DISPLAY_MODE,	GENESYS_DISPLAY_MODE_05},
	{INNO_COMM_TABLE_END,	0x00}
};

INNO_COMM_REG_8 inno_ev2m_gom1_mode_06_1920x1080_30fps[] = {
	{GENESYS_DISPLAY_MODE,	GENESYS_DISPLAY_MODE_06},
	{INNO_COMM_TABLE_END,	0x00}
};

INNO_COMM_REG_8 inno_ev2m_gom1_mode_07_1920x1080_30fps[] = {
	{GENESYS_DISPLAY_MODE,	GENESYS_DISPLAY_MODE_07},
	{INNO_COMM_TABLE_END,	0x00}
};

INNO_COMM_REG_8 inno_ev2m_gom1_mode_08_1920x1080_30fps[] = {
	{GENESYS_DISPLAY_MODE,	GENESYS_DISPLAY_MODE_08},
	{INNO_COMM_TABLE_END,	0x00}
};

enum {
	INNO_EV2M_GOM1_MODE_00_1920x1080_30FPS,
	INNO_EV2M_GOM1_MODE_01_1920x1080_30FPS,
	INNO_EV2M_GOM1_MODE_02_1920x1080_30FPS,
	INNO_EV2M_GOM1_MODE_03_1920x1080_30FPS,
	INNO_EV2M_GOM1_MODE_04_1920x1080_30FPS,
	INNO_EV2M_GOM1_MODE_05_1920x1080_30FPS,
	INNO_EV2M_GOM1_MODE_06_1920x1080_30FPS,
	INNO_EV2M_GOM1_MODE_07_1920x1080_30FPS,
	INNO_EV2M_GOM1_MODE_08_1920x1080_30FPS,

	INNO_EVXM_YYMZ_MODE_COMMON,
	INNO_EVXM_YYMZ_START_STREAM,
	INNO_EVXM_YYMZ_STOP_STREAM,
};

INNO_COMM_REG_8 *inno_ev2m_gom1_mode_table[] = {
	[INNO_EV2M_GOM1_MODE_00_1920x1080_30FPS]	= inno_ev2m_gom1_mode_00_1920x1080_30fps,
	[INNO_EV2M_GOM1_MODE_01_1920x1080_30FPS]	= inno_ev2m_gom1_mode_01_1920x1080_30fps,
	[INNO_EV2M_GOM1_MODE_02_1920x1080_30FPS]	= inno_ev2m_gom1_mode_02_1920x1080_30fps,
	[INNO_EV2M_GOM1_MODE_03_1920x1080_30FPS]	= inno_ev2m_gom1_mode_03_1920x1080_30fps,
	[INNO_EV2M_GOM1_MODE_04_1920x1080_30FPS]	= inno_ev2m_gom1_mode_04_1920x1080_30fps,
	[INNO_EV2M_GOM1_MODE_05_1920x1080_30FPS]	= inno_ev2m_gom1_mode_05_1920x1080_30fps,
	[INNO_EV2M_GOM1_MODE_06_1920x1080_30FPS]	= inno_ev2m_gom1_mode_06_1920x1080_30fps,
	[INNO_EV2M_GOM1_MODE_07_1920x1080_30FPS]	= inno_ev2m_gom1_mode_07_1920x1080_30fps,
	[INNO_EV2M_GOM1_MODE_08_1920x1080_30FPS]	= inno_ev2m_gom1_mode_08_1920x1080_30fps,

	[INNO_EVXM_YYMZ_MODE_COMMON]				= inno_comm_mode_common_8,
	[INNO_EVXM_YYMZ_START_STREAM]				= inno_comm_start_stream_8,
	[INNO_EVXM_YYMZ_STOP_STREAM]				= inno_comm_stop_stream_8,
};

INNO_COMM_REG_8 *inno_ev2m_gom1_sensor_mode_table[] = {
};

#if defined(_INNO_FOR_NVIDIA_)
/*
 * WARNING: frmfmt ordering need to match mode definition in
 * device tree!
 */
static const struct camera_common_frmfmt inno_ev2m_gom1_frmfmt[] = {
	{{INNO_WIDTH_1920, INNO_HEIGHT_1080}, inno_comm_30fps, 1, 0, INNO_EV2M_GOM1_MODE_00_1920x1080_30FPS},
	{{INNO_WIDTH_1920, INNO_HEIGHT_1080}, inno_comm_30fps, 1, 0, INNO_EV2M_GOM1_MODE_01_1920x1080_30FPS},
	{{INNO_WIDTH_1920, INNO_HEIGHT_1080}, inno_comm_30fps, 1, 0, INNO_EV2M_GOM1_MODE_02_1920x1080_30FPS},
	{{INNO_WIDTH_1920, INNO_HEIGHT_1080}, inno_comm_30fps, 1, 0, INNO_EV2M_GOM1_MODE_03_1920x1080_30FPS},
	{{INNO_WIDTH_1920, INNO_HEIGHT_1080}, inno_comm_30fps, 1, 0, INNO_EV2M_GOM1_MODE_04_1920x1080_30FPS},
	{{INNO_WIDTH_1920, INNO_HEIGHT_1080}, inno_comm_30fps, 1, 0, INNO_EV2M_GOM1_MODE_05_1920x1080_30FPS},
	{{INNO_WIDTH_1920, INNO_HEIGHT_1080}, inno_comm_30fps, 1, 0, INNO_EV2M_GOM1_MODE_06_1920x1080_30FPS},
	{{INNO_WIDTH_1920, INNO_HEIGHT_1080}, inno_comm_30fps, 1, 0, INNO_EV2M_GOM1_MODE_07_1920x1080_30FPS},
	{{INNO_WIDTH_1920, INNO_HEIGHT_1080}, inno_comm_30fps, 1, 0, INNO_EV2M_GOM1_MODE_08_1920x1080_30FPS},
	/* Add modes with no device tree support after below */
};
#endif

static const struct inno_res_info inno_ev2m_gom1_prvw_modes[] = {
	{
		.width		= INNO_WIDTH_1920,
		.height		= INNO_HEIGHT_1080,
		.fps		= INNO_FPS_30,
		.format		= PREVIEW_OUT_FMT_YUV_JFIF_422,
		.mode		= 0x00,
	},
	{
		.width		= INNO_WIDTH_1920,
		.height		= INNO_HEIGHT_1080,
		.fps		= INNO_FPS_30,
		.format		= PREVIEW_OUT_FMT_YUV_JFIF_422,
		.mode		= 0x01,
	},
	{
		.width		= INNO_WIDTH_1920,
		.height		= INNO_HEIGHT_1080,
		.fps		= INNO_FPS_30,
		.format		= PREVIEW_OUT_FMT_YUV_JFIF_422,
		.mode		= 0x02,
	},
	{
		.width		= INNO_WIDTH_1920,
		.height		= INNO_HEIGHT_1080,
		.fps		= INNO_FPS_30,
		.format		= PREVIEW_OUT_FMT_YUV_JFIF_422,
		.mode		= 0x03,
	},
	{
		.width		= INNO_WIDTH_1920,
		.height		= INNO_HEIGHT_1080,
		.fps		= INNO_FPS_30,
		.format		= PREVIEW_OUT_FMT_YUV_JFIF_422,
		.mode		= 0x04,
	},
	{
		.width		= INNO_WIDTH_1920,
		.height		= INNO_HEIGHT_1080,
		.fps		= INNO_FPS_30,
		.format		= PREVIEW_OUT_FMT_YUV_JFIF_422,
		.mode		= 0x05,
	},
	{
		.width		= INNO_WIDTH_1920,
		.height		= INNO_HEIGHT_1080,
		.fps		= INNO_FPS_30,
		.format		= PREVIEW_OUT_FMT_YUV_JFIF_422,
		.mode		= 0x06,
	},
	{
		.width		= INNO_WIDTH_1920,
		.height		= INNO_HEIGHT_1080,
		.fps		= INNO_FPS_30,
		.format		= PREVIEW_OUT_FMT_YUV_JFIF_422,
		.mode		= 0x07,
	},
	{
		.width		= INNO_WIDTH_1920,
		.height		= INNO_HEIGHT_1080,
		.fps		= INNO_FPS_30,
		.format		= PREVIEW_OUT_FMT_YUV_JFIF_422,
		.mode		= 0x08,
	},
};

#endif /* __INNO_EV2M_GOM1_MODE_H__ */
