/*
 * inno_ev2m_gom1_core.h
 *
 * Copyright (c) 2015-2016, NVIDIA CORPORATION, All Rights Reserved.
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

#ifndef __INNO_EV2M_GOM1_H__
#define __INNO_EV2M_GOM1_H__

enum power_line_frequency_state {
	FREQ_60Hz_LLE,
	FREQ_50Hz_LLE,
	FREQ_60Hz,
	FREQ_50Hz
};
static const char * const camera_power_line_frequency[] = {
	"60Hz with LLE",
	"50Hz with LLE",
	"60Hz",
	"50Hz"
};

#if 0
enum AE_MAX_SHUTTER_controls {
	SHUTTER_1_2_S,
	SHUTTER_1_4_S,
	SHUTTER_1_7_S,
	SHUTTER_1_15_S,
	SHUTTER_1_30_S,
	SHUTTER_1_60_S,
	SHUTTER_1_120_S,
};

static const char * const AE_MAX_SHUTTER_LIST[] = {
	"  1/2s",
	"  1/4s",
	"1/7.5s",
	" 1/15s",
	" 1/30s",
	" 1/60s",
	"1/120s"
};

enum AE_MAX_GAIN_controls {
	GAIN_6_DB,
	GAIN_12_DB,
	GAIN_18_DB,
	GAIN_24_DB,
	GAIN_30_DB,
	GAIN_36_DB,
	GAIN_42_DB,
};

static const char * const AE_MAX_GAIN_LIST[] = {
	" 6dB",
	"12dB",
	"18dB",
	"24dB",
	"30dB",
	"36dB",
	"42dB"
};

enum AE_WEIGHT_controls {
	WEIGHT_FULL,
	WEIGHT_CENTER,
	WEIGHT_SPOT
};

static const char * const AE_WEIGHT_LIST[] = {
	"Average",
	"Center Weighted",
	"Spot",
};

enum AWB_MODE_controls {
	AWB_AUTO,
	AWB_MANUAL_GAIN,
	AWB_MANUAL_COLOR_TEMP,
	AWB_SUNNY,
	AWB_SHADOW,
	AWB_INDOOR,
	AWB_LAMP,
	AWB_FL1,
	AWB_FL2,
	AWB_One_Push
};

static const char * const AWB_MODE_LIST[] = {
	"AUTO",
	"MANUAL_GAIN",
	"MANUAL_COLOR_TEMP",
	"SUNNY",
	"SHADOW",
	"INDOOR",
	"LAMP",
	"FL1",
	"FL2",
	"One-Push"
};
#endif

enum FISHEYE_MODE_controls {
	Fisheye_View,
	Three_Window_Stitching,
	Panorama_View,
	Perspective_View,
	Two_Windows_Stitching,
	Panorama130_View,
	Panorama360_Single_View,
	Panorama360_Dual_View,
	Meeting360_View,
	Four_Windows	
};

static const char * const FISHEYE_MODE_LIST[] = {
	"Fisheye view",
	"3-window stitching",
	"Panorama view",
	"Perspective view",
	"2-windows stitching",
	"130 Panorama view",
	"360 Panorama + Single view",
	"360 Panorama + Dual view",
	"360 Meeting view",
	"360 4-window"
};

static const s64 hdr_switch_ctrl_qmenu[] = {
	SWITCH_OFF
};

#endif /* __INNO_EV2M_GOM1_H__ */
