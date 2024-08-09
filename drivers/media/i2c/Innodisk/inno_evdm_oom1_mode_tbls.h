/*
 * inno_evdm_oom1_mode_tbls.h
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

#ifndef __INNO_EVDM_OOM1_MODE_H__
#define __INNO_EVDM_OOM1_MODE_H__

#define INNO_EVXM_YYMZ_MODE_COMMON 			INNO_EVDM_OOM1_MODE_COMMON
#define INNO_EVXM_YYMZ_START_STREAM 		INNO_EVDM_OOM1_START_STREAM
#define INNO_EVXM_YYMZ_STOP_STREAM 			INNO_EVDM_OOM1_STOP_STREAM

#include "inno_common_mode_tbls.h"

enum {
	INNO_EVDM_OOM1_MODE_640x480_90FPS,
	INNO_EVDM_OOM1_MODE_1280x720_120FPS,
	INNO_EVDM_OOM1_MODE_1920x1080_60FPS,
	INNO_EVDM_OOM1_MODE_1920x1200_60FPS,
	INNO_EVDM_OOM1_MODE_3840x2160_30FPS,
	INNO_EVDM_OOM1_MODE_4096x3120_20FPS,
	INNO_EVDM_OOM1_MODE_4192x3120_20FPS,

	INNO_EVXM_YYMZ_MODE_COMMON,
	INNO_EVXM_YYMZ_START_STREAM,
	INNO_EVXM_YYMZ_STOP_STREAM,
};

INNO_COMM_REG_16 *inno_evdm_oom1_mode_table[] = {
	[INNO_EVDM_OOM1_MODE_640x480_90FPS]		= inno_evxm_oymz_mode_640x480_90fps,
	[INNO_EVDM_OOM1_MODE_1280x720_120FPS]	= inno_evxm_oymz_mode_1280x720_120fps,
	[INNO_EVDM_OOM1_MODE_1920x1080_60FPS]	= inno_evxm_oymz_mode_1920x1080_60fps,
	[INNO_EVDM_OOM1_MODE_1920x1200_60FPS]	= inno_evxm_oymz_mode_1920x1200_60fps,
	[INNO_EVDM_OOM1_MODE_3840x2160_30FPS]	= inno_evxm_oymz_mode_3840x2160_30fps,
	[INNO_EVDM_OOM1_MODE_4096x3120_20FPS]	= inno_evxm_oymz_mode_4096x3120_20fps,
	[INNO_EVDM_OOM1_MODE_4192x3120_20FPS]	= inno_evxm_oymz_mode_4192x3120_20fps,

	[INNO_EVXM_YYMZ_MODE_COMMON]			= inno_comm_mode_common_16,
	[INNO_EVXM_YYMZ_START_STREAM]			= inno_comm_start_stream_16,
	[INNO_EVXM_YYMZ_STOP_STREAM]			= inno_comm_stop_stream_16,
};

/*
 * Mode0: 4208*3120
 * Mode1: 4208*1560
 * Mode2: 2104*1560
 * Mode3: 2104*1040
 * Mode4: 2104*780
 * Mode5: 1052*780
 */
INNO_COMM_REG_16 *inno_evdm_oom1_sensor_mode_table[] = {
	[INNO_EVDM_OOM1_MODE_640x480_90FPS]		= inno_evxm_oymz_sensor_mode_5,
	[INNO_EVDM_OOM1_MODE_1280x720_120FPS]	= inno_evxm_oymz_sensor_mode_4,
	[INNO_EVDM_OOM1_MODE_1920x1080_60FPS]	= inno_evxm_oymz_sensor_mode_2,
	[INNO_EVDM_OOM1_MODE_1920x1200_60FPS]	= inno_evxm_oymz_sensor_mode_2,
	[INNO_EVDM_OOM1_MODE_3840x2160_30FPS]	= inno_evxm_oymz_sensor_mode_0,
	[INNO_EVDM_OOM1_MODE_4096x3120_20FPS]	= inno_evxm_oymz_sensor_mode_0,
	[INNO_EVDM_OOM1_MODE_4192x3120_20FPS]	= inno_evxm_oymz_sensor_mode_0,
};

#if defined(_INNO_FOR_NVIDIA_)
/*
 * WARNING: frmfmt ordering need to match mode definition in
 * device tree!
 */
static const struct camera_common_frmfmt inno_evdm_oom1_frmfmt[] = {
	{{ INNO_WIDTH_640,  INNO_HEIGHT_480},  inno_comm_90fps, 1, 0, INNO_EVDM_OOM1_MODE_640x480_90FPS},
	{{INNO_WIDTH_1280,  INNO_HEIGHT_720}, inno_comm_120fps, 1, 0, INNO_EVDM_OOM1_MODE_1280x720_120FPS},
	{{INNO_WIDTH_1920, INNO_HEIGHT_1080},  inno_comm_60fps, 1, 0, INNO_EVDM_OOM1_MODE_1920x1080_60FPS},
	{{INNO_WIDTH_1920, INNO_HEIGHT_1200},  inno_comm_60fps, 1, 0, INNO_EVDM_OOM1_MODE_1920x1200_60FPS},
	{{INNO_WIDTH_3840, INNO_HEIGHT_2160},  inno_comm_30fps, 1, 0, INNO_EVDM_OOM1_MODE_3840x2160_30FPS},
	{{INNO_WIDTH_4096, INNO_HEIGHT_3120},  inno_comm_20fps, 1, 0, INNO_EVDM_OOM1_MODE_4096x3120_20FPS},
	{{INNO_WIDTH_4192, INNO_HEIGHT_3120},  inno_comm_20fps, 1, 0, INNO_EVDM_OOM1_MODE_4192x3120_20FPS},
	/* Add modes with no device tree support after below */
};
#endif

static const struct inno_res_info inno_evdm_oom1_prvw_modes[] = {
	{
		.width		= INNO_WIDTH_640,
		.height		= INNO_HEIGHT_480,
		.fps		= INNO_FPS_90,
		.format		= PREVIEW_OUT_FMT_YUV_JFIF_422,
		.mode		= SENSOR_MODE(5),
	},
	{
		.width		= INNO_WIDTH_1280,
		.height		= INNO_HEIGHT_720,
		.fps		= INNO_FPS_120,
		.format		= PREVIEW_OUT_FMT_YUV_JFIF_422,
		.mode		= SENSOR_MODE(4),
	},
	{
		.width		= INNO_WIDTH_1920,
		.height		= INNO_HEIGHT_1080,
		.fps		= INNO_FPS_60,
		.format		= PREVIEW_OUT_FMT_YUV_JFIF_422,
		.mode		= SENSOR_MODE(2),
	},
	{
		.width		= INNO_WIDTH_1920,
		.height		= INNO_HEIGHT_1200,
		.fps		= INNO_FPS_60,
		.format		= PREVIEW_OUT_FMT_YUV_JFIF_422,
		.mode		= SENSOR_MODE(2),
	},
	{
		.width		= INNO_WIDTH_3840,
		.height		= INNO_HEIGHT_2160,
		.fps		= INNO_FPS_30,
		.format		= PREVIEW_OUT_FMT_YUV_JFIF_422,
		.mode		= SENSOR_MODE(0),
	},
	{
		.width		= INNO_WIDTH_4096,
		.height		= INNO_HEIGHT_3120,
		.fps		= INNO_FPS_20,
		.format		= PREVIEW_OUT_FMT_YUV_JFIF_422,
		.mode		= SENSOR_MODE(0),
	},
};

#endif /* __INNO_EVDM_OOM1_MODE_H__ */
