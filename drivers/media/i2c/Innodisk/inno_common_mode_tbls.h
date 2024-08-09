/*
 * inno_common_mode_tbls.h
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

#ifndef __INNO_COMMON_MODE_TBLS_H__
#define __INNO_COMMON_MODE_TBLS_H__

#define INNO_SWAP16(SWAP_NUM) \
	((((u16)(SWAP_NUM) & 0xff00) >> 8) | \
	 (((u16)(SWAP_NUM) & 0x00ff) << 8))

#define INNO_SWAP32(SWAP_NUM) \
	((((u32)(SWAP_NUM) & 0xff000000) >> 24) | \
	 (((u32)(SWAP_NUM) & 0x00ff0000) >>  8) | \
	 (((u32)(SWAP_NUM) & 0x0000ff00) <<  8) | \
	 (((u32)(SWAP_NUM) & 0x000000ff) << 24))

#define INNO_SWAP32_LSB(SWAP_NUM) \
	((u16)(INNO_SWAP32(SWAP_NUM) & 0x0000ffff))

#define INNO_SWAP32_MSB(SWAP_NUM) \
	((u16)((INNO_SWAP32(SWAP_NUM) & 0xffff0000) >> 16))

#define INNO_COMM_TABLE_WAIT_MS		0
#define INNO_COMM_TABLE_END			1

#define INNO_COMM_MAX_RETRIES		3
#define INNO_COMM_WAIT_MS_MODE		100 // With symbols: POLL_FIELD=ATOMIC,&-0x5,DELAY=10,TIMEOUT=100
#define INNO_COMM_WAIT_MS_STOP		50
#define INNO_COMM_WAIT_MS_START		50
#define INNO_COMM_WAIT_MS_STREAM	255

#define INNO_COMM_GAIN_TABLE_SIZE	255

#define INNO_WIDTH_640				640
#define INNO_WIDTH_1280				1280
#define INNO_WIDTH_1920				1920
#define INNO_WIDTH_3840				3840
#define INNO_WIDTH_4096				4096
#define INNO_WIDTH_4192				4192

#define INNO_HEIGHT_480				480
#define INNO_HEIGHT_720				720
#define INNO_HEIGHT_1080			1080
#define INNO_HEIGHT_1200			1200
#define INNO_HEIGHT_2160			2160
#define INNO_HEIGHT_3120			3120

#define INNO_FPS_10					10
#define INNO_FPS_15					15
#define INNO_FPS_20					20
#define INNO_FPS_25					25
#define INNO_FPS_30					30
#define INNO_FPS_50					50
#define INNO_FPS_60					60
#define INNO_FPS_90					90
#define INNO_FPS_120				120

#define MICRO_SECOND				(1000 * 1000)
#define AE_MAX_ET(FPS)				((u32)((MICRO_SECOND) / (FPS)))

extern INNO_COMM_REG_8				inno_comm_mode_common_8[];
extern INNO_COMM_REG_8				inno_comm_start_stream_8[];
extern INNO_COMM_REG_8				inno_comm_stop_stream_8[];

extern INNO_COMM_REG_16				inno_evxm_oymz_sensor_mode_0[];
extern INNO_COMM_REG_16				inno_evxm_oymz_sensor_mode_1[];
extern INNO_COMM_REG_16				inno_evxm_oymz_sensor_mode_2[];
extern INNO_COMM_REG_16				inno_evxm_oymz_sensor_mode_3[];
extern INNO_COMM_REG_16				inno_evxm_oymz_sensor_mode_4[];
extern INNO_COMM_REG_16				inno_evxm_oymz_sensor_mode_5[];
extern INNO_COMM_REG_16				inno_evxm_oymz_sensor_mode_6[];
extern INNO_COMM_REG_16				inno_evxm_oymz_sensor_mode_7[];

extern INNO_COMM_REG_16				inno_evxm_oymz_mode_640x480_60fps[];
extern INNO_COMM_REG_16				inno_evxm_oymz_mode_640x480_90fps[];
extern INNO_COMM_REG_16				inno_evxm_oymz_mode_640x480_120fps[];
extern INNO_COMM_REG_16				inno_evxm_oymz_mode_1280x720_60fps[];
extern INNO_COMM_REG_16				inno_evxm_oymz_mode_1280x720_90fps[];
extern INNO_COMM_REG_16				inno_evxm_oymz_mode_1280x720_120fps[];
extern INNO_COMM_REG_16				inno_evxm_oymz_mode_1920x1080_50fps[];
extern INNO_COMM_REG_16				inno_evxm_oymz_mode_1920x1080_60fps[];
extern INNO_COMM_REG_16				inno_evxm_oymz_mode_1920x1200_50fps[];
extern INNO_COMM_REG_16				inno_evxm_oymz_mode_1920x1200_60fps[];
extern INNO_COMM_REG_16				inno_evxm_oymz_mode_3840x2160_25fps[];
extern INNO_COMM_REG_16				inno_evxm_oymz_mode_3840x2160_30fps[];
extern INNO_COMM_REG_16				inno_evxm_oymz_mode_4096x3120_10fps[];
extern INNO_COMM_REG_16				inno_evxm_oymz_mode_4096x3120_20fps[];
extern INNO_COMM_REG_16				inno_evxm_oymz_mode_4192x3120_10fps[];
extern INNO_COMM_REG_16				inno_evxm_oymz_mode_4192x3120_20fps[];

extern INNO_COMM_REG_16				inno_comm_mode_common_16[];
extern INNO_COMM_REG_16				inno_comm_start_stream_16[];
extern INNO_COMM_REG_16				inno_comm_stop_stream_16[];

extern const int					inno_comm_10fps[];
extern const int					inno_comm_15fps[];
extern const int					inno_comm_20fps[];
extern const int					inno_comm_25fps[];
extern const int					inno_comm_30fps[];
extern const int					inno_comm_50fps[];
extern const int					inno_comm_60fps[];
extern const int					inno_comm_90fps[];
extern const int					inno_comm_120fps[];

enum display_mode {
	GENESYS_DISPLAY_MODE_00,
	GENESYS_DISPLAY_MODE_01,
	GENESYS_DISPLAY_MODE_02,
	GENESYS_DISPLAY_MODE_03,
	GENESYS_DISPLAY_MODE_04,
	GENESYS_DISPLAY_MODE_05,
	GENESYS_DISPLAY_MODE_06,
	GENESYS_DISPLAY_MODE_07,
	GENESYS_DISPLAY_MODE_08,
	GENESYS_DISPLAY_MODE_NUM,
};

#endif /* __INNO_COMMON_MODE_TBLS_H__ */
