/*
 * inno_evdm_oom1_core.c - INNO_EVDM_OOM1 sensor driver
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

#if defined(_INNO_FOR_NVIDIA_)
#define inno_evxm_yymz_frmfmt				inno_evdm_oom1_frmfmt
#endif
#define inno_evxm_yymz_prvw_modes			inno_evdm_oom1_prvw_modes
#define inno_evxm_yymz_mode_table			inno_evdm_oom1_mode_table
#define inno_evxm_yymz_sensor_mode_table	inno_evdm_oom1_sensor_mode_table
#define inno_evxm_yymz_write_table			inno_comm_nv_write_table_16
#define inno_evxm_yymz_prt_tbls				inno_comm_prt_tbls_16

#define INNO_EVXM_YYMZ_DEF_MODE				INNO_EVDM_OOM1_MODE_4096x3120_20FPS
#define INNO_EVXM_YYMZ_DEVICE_ID 			INNO_EVDM_OOM1_DEVICE_ID
#define INNO_EVXM_YYMZ_FW_NAME 				INNO_EVDM_OOM1_FW_NAME
#define INNO_EVXM_YYMZ_MODULE_ID 			INNO_EVDM_OOM1_MODULE_ID
#define INNO_EVXM_YYMZ_SENSOR_ID 			INNO_EVDM_OOM1_SENSOR_ID
#define INNO_EVXM_YYMZ_SEN_CHIP_VER_REG 	INNO_EVDM_OOM1_SEN_CHIP_VER_REG
#define INNO_EVXM_YYMZ_SIP_ID 				INNO_EVDM_OOM1_SIP_ID
#define INNO_EVXM_YYMZ_DEVICE_NAME 			INNO_EVDM_OOM1_DEVICE_NAME

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
#include "inno_max_gmsl.h"
#include "inno_common_nv.h"
#elif defined(_INNO_FOR_NXP_)
#include "inno_common_nxp.h"
#endif
#include "inno_iic.h"
#include "inno_evdm_oom1_core.h"
#include "inno_evdm_oom1_mode_tbls.h"

static const struct of_device_id inno_evxm_yymz_of_match[] = {
	{.compatible = INNO_EVXM_YYMZ_DEVICE_ID},
	{},
};

MODULE_DEVICE_TABLE(of, inno_evxm_yymz_of_match);

static const struct i2c_device_id inno_evxm_yymz_id[] = {
	{INNO_EVXM_YYMZ_DEVICE_NAME, 0},
	{},
};

MODULE_DEVICE_TABLE(i2c, inno_evxm_yymz_id);

#if defined(_INNO_FOR_NVIDIA_)
static const u32 ctrl_cid_list[] = {
	TEGRA_CAMERA_CID_GAIN,
	TEGRA_CAMERA_CID_EXPOSURE,
	TEGRA_CAMERA_CID_EXPOSURE_SHORT,
	TEGRA_CAMERA_CID_FRAME_RATE,
	TEGRA_CAMERA_CID_HDR_EN,
	TEGRA_CAMERA_CID_SENSOR_MODE_ID,
};

static const struct v4l2_subdev_internal_ops inno_evxm_yymz_subdev_internal_ops = {
	.open				= inno_comm_open,
};

static struct tegracam_ctrl_ops inno_evxm_yymz_ctrl_ops = {
	.numctrls			= ARRAY_SIZE(ctrl_cid_list),
	.ctrl_cid_list		= ctrl_cid_list,
	.set_gain			= inno_comm_nv_set_gain,
	.set_exposure		= inno_comm_nv_set_exposure,
	.set_exposure_short	= inno_comm_nv_set_exposure,
	.set_frame_rate		= inno_comm_nv_set_frame_rate,
	.set_group_hold		= inno_comm_nv_set_group_hold,
	.fill_string_ctrl	= inno_comm_nv_fill_string_ctrl,
};

static struct camera_common_pdata *inno_evxm_yymz_parse_dt(struct tegracam_device *tc_dev)
{
	struct device *dev = tc_dev->dev;
	struct device_node *np = dev->of_node;
	const struct of_device_id *match;

	INNO_DEV_ALERT(dev, "\n");

	if (!np)
		return NULL;

	match = of_match_device(inno_evxm_yymz_of_match, dev);
	if (!match) {
		INNO_DEV_ERR(dev, "Error finding matching DT ID\n");
		return NULL;
	}

	return inno_comm_nv_parse_dt(tc_dev);
}

static int inno_evxm_yymz_set_mode(struct tegracam_device *tc_dev)
{
	struct inno_cam_mod *priv = (struct inno_cam_mod *)tegracam_get_privdata(tc_dev);
	struct camera_common_data *s_data = tc_dev->s_data;
	struct device *dev = priv->dev;
	int ret = INNO_RET_OK;

	INNO_DEV_ALERT(dev, "Mode#%d: %dx%d@%d\n",
		s_data->mode,
		inno_evxm_yymz_frmfmt[s_data->mode].size.width,
		inno_evxm_yymz_frmfmt[s_data->mode].size.height,
		inno_evxm_yymz_frmfmt[s_data->mode].framerates[0]);
	inno_evxm_yymz_prt_tbls(inno_evxm_yymz_mode_table[s_data->mode], INNO_COMM_TABLE_END);
	ret = inno_evxm_yymz_write_table(priv, inno_evxm_yymz_mode_table[s_data->mode]);
	if (ret) {
		INNO_DEV_ERR(dev, "Error writing mode table (%d, %d)\n", ret, s_data->mode);
		return ret;
	}

	if (!strcmp(priv->info->dev_name, INNO_EV2M_OOM1_DEVICE_NAME) ||
		!strcmp(priv->info->dev_name, INNO_EVDM_OOM1_DEVICE_NAME) ||
		!strcmp(priv->info->dev_name, INNO_EV2M_OOM2_DEVICE_NAME) ||
		!strcmp(priv->info->dev_name, INNO_EV8M_OOM1_DEVICE_NAME)) {
		INNO_DEV_ALERT(dev, "Sensor Mode: %d\n", inno_evxm_yymz_sensor_mode_table[s_data->mode]->val);
		ret = inno_iic_wr_reg(priv,
							  inno_evxm_yymz_sensor_mode_table[s_data->mode]->addr,
							  inno_evxm_yymz_sensor_mode_table[s_data->mode]->val,
							  INNO_IIC_VAL_16BIT);
		if (ret) {
			INNO_DEV_ERR(dev, "Error writing snesor mode table (%d, %d)\n", ret, inno_evxm_yymz_sensor_mode_table[s_data->mode]->val);
			return ret;
		}
		msleep_range(INNO_COMM_WAIT_MS_MODE);
	}

	return ret;
}

static int inno_evxm_yymz_start_streaming(struct tegracam_device *tc_dev)
{
	struct inno_cam_mod *priv = (struct inno_cam_mod *)tegracam_get_privdata(tc_dev);
	struct device *dev = priv->dev;
	int ret = INNO_RET_OK;


#if INNO_MAX_GMSL
	ret = inno_max_gmsl_start(tc_dev);
	if (ret) {
		INNO_DEV_ERR(dev, "Error starting Maxim GMSL (%d)\n", ret);
		return ret;
	}
#endif

	INNO_DEV_ALERT(dev, "Common Mode\n");
	inno_evxm_yymz_prt_tbls(inno_evxm_yymz_mode_table[INNO_EVXM_YYMZ_MODE_COMMON], INNO_COMM_TABLE_END);
	ret = inno_evxm_yymz_write_table(priv, inno_evxm_yymz_mode_table[INNO_EVXM_YYMZ_MODE_COMMON]);
	if (ret) {
		INNO_DEV_ERR(dev, "Error writing mode table (%d)\n", ret);
		return ret;
	}

	INNO_DEV_ALERT(dev, "Start Stream\n");
	inno_evxm_yymz_prt_tbls(inno_evxm_yymz_mode_table[INNO_EVXM_YYMZ_START_STREAM], INNO_COMM_TABLE_END);
	ret = inno_evxm_yymz_write_table(priv, inno_evxm_yymz_mode_table[INNO_EVXM_YYMZ_START_STREAM]);
	if (ret) {
		INNO_DEV_ERR(dev, "Error writing mode table (%d)\n", ret);
		return ret;
	}

	return ret;
}

static int inno_evxm_yymz_stop_streaming(struct tegracam_device *tc_dev)
{
	struct inno_cam_mod *priv = (struct inno_cam_mod *)tegracam_get_privdata(tc_dev);
	struct device *dev = priv->dev;
	int ret = INNO_RET_OK;

	INNO_DEV_ALERT(dev, "Common Mode\n");
	inno_evxm_yymz_prt_tbls(inno_evxm_yymz_mode_table[INNO_EVXM_YYMZ_MODE_COMMON], INNO_COMM_TABLE_END);
	ret = inno_evxm_yymz_write_table(priv, inno_evxm_yymz_mode_table[INNO_EVXM_YYMZ_MODE_COMMON]);
	if (ret) {
		INNO_DEV_ERR(dev, "Error writing mode table (%d)\n", ret);
		return ret;
	}

	INNO_DEV_ALERT(dev, "Stop Stream\n");
	inno_evxm_yymz_prt_tbls(inno_evxm_yymz_mode_table[INNO_EVXM_YYMZ_STOP_STREAM], INNO_COMM_TABLE_END);
	ret = inno_evxm_yymz_write_table(priv, inno_evxm_yymz_mode_table[INNO_EVXM_YYMZ_STOP_STREAM]);
	if (ret) {
		INNO_DEV_ERR(dev, "Error writing mode table (%d)\n", ret);
		return ret;
	}

#if INNO_MAX_GMSL
	ret = inno_max_gmsl_stop(tc_dev);
	if (ret) {
		INNO_DEV_ERR(dev, "Error stopping Maxim GMSL (%d)\n", ret);
		return ret;
	}
#endif

	return ret;
}

static struct camera_common_sensor_ops inno_evxm_yymz_common_ops = {
	.numfrmfmts			= ARRAY_SIZE(inno_evxm_yymz_frmfmt),
	.frmfmt_table		= inno_evxm_yymz_frmfmt,
	.power_on			= inno_comm_nv_power_on,
	.power_off			= inno_comm_nv_power_off,
	.write_reg			= inno_comm_nv_write_reg,
	.read_reg			= inno_comm_nv_read_reg,
	.power_get			= inno_comm_nv_power_get,
	.power_put			= inno_comm_nv_power_put,
	.parse_dt			= inno_evxm_yymz_parse_dt,
	.set_mode			= inno_evxm_yymz_set_mode,
	.start_streaming	= inno_evxm_yymz_start_streaming,
	.stop_streaming		= inno_evxm_yymz_stop_streaming,
};
#elif defined(_INNO_FOR_NXP_)
static const struct v4l2_ctrl_ops inno_evxm_yymz_ctrl_ops = {
	.g_volatile_ctrl	= inno_comm_g_volatile_ctrl,
	.try_ctrl			= inno_comm_try_ctrl,
	.s_ctrl				= inno_comm_s_ctrl,
};

static struct v4l2_subdev_core_ops inno_evxm_yymz_subdev_core_ops = {
	.s_power			= inno_comm_s_power,
#ifdef CONFIG_VIDEO_ADV_DEBUG
	.g_register			= inno_comm_g_register,
	.s_register			= inno_comm_s_register,
#endif
};

static struct v4l2_subdev_video_ops inno_evxm_yymz_subdev_video_ops = {
	.g_frame_interval	= inno_comm_g_frame_interval,
	.s_frame_interval	= inno_comm_s_frame_interval,
	.s_stream			= inno_comm_s_stream,
};

static const struct v4l2_subdev_pad_ops inno_evxm_yymz_subdev_pad_ops = {
	.enum_mbus_code		= inno_comm_enum_mbus_code,
	.enum_frame_size	= inno_comm_enum_frame_size,
	.enum_frame_interval= inno_comm_enum_frame_interval,
	.get_fmt			= inno_comm_get_fmt,
	.set_fmt			= inno_comm_set_fmt,
};

static struct v4l2_subdev_ops inno_evxm_yymz_subdev_ops = {
	.core				= &inno_evxm_yymz_subdev_core_ops,
	.video				= &inno_evxm_yymz_subdev_video_ops,
	.pad				= &inno_evxm_yymz_subdev_pad_ops,
};

/* Add the link setup callback to the media entity operations struct */
static const struct media_entity_operations inno_evxm_yymz_sd_media_ops = {
	.link_setup			= inno_comm_link_setup,
};
#endif

static int inno_evxm_yymz_probe(struct i2c_client *client,
								const struct i2c_device_id *id)
{
	struct device *dev = &client->dev;
	struct inno_cam_mod *priv;
	struct inno_cam_mod_info *info;
#if defined(_INNO_FOR_NVIDIA_)
	struct tegracam_device *tc_dev;
#elif defined(_INNO_FOR_NXP_)
	struct v4l2_mbus_framefmt *fmt;
#endif
	int ret = INNO_RET_OK;
    int ret_line = 0;

	INNO_DEV_ALERT(dev, "Detecting @ 0x%02X\n", client->addr);

	if (!IS_ENABLED(CONFIG_OF) || !client->dev.of_node) {
		ret = -EINVAL;
		ret_line = __LINE__;
		goto exit;
	}

	/* Allocate device & Initialize sub device */
	priv = devm_kmalloc(dev, sizeof(struct inno_cam_mod), GFP_KERNEL);
	if (!priv) {
		ret = -ENOMEM;
		ret_line = __LINE__;
		goto exit;
	}

	info = devm_kmalloc(dev, sizeof(struct inno_cam_mod_info), GFP_KERNEL);
	if (!info) {
		ret = -ENOMEM;
		ret_line = __LINE__;
		goto exit;
	}

#if defined(_INNO_FOR_NVIDIA_)
	tc_dev = devm_kmalloc(dev, sizeof(struct tegracam_device), GFP_KERNEL);
	if (!tc_dev) {
		ret = -ENOMEM;
		ret_line = __LINE__;
		goto exit;
	}
#endif

	/* Set initial values for the sensor struct. */
	memset(priv, 0, sizeof(struct inno_cam_mod));
	memset(info, 0, sizeof(struct inno_cam_mod_info));

	ret = of_property_read_string(dev->of_node, "sensor_model", &info->dev_name);
	if (ret) {
		INNO_DEV_ERR(dev, "No sensor_model found\n");
		ret = -ENODEV;
		ret_line = __LINE__;
		goto exit;
	} else {
		INNO_ALERT("sensor_model be found (%s)\n", info->dev_name);
	}

	ret = of_property_read_u32(dev->of_node, "io-exp-addr", &info->io_exp_addr);
	if (ret) {
		info->io_exp_addr	= INNO_II2_IO_EXP_IIC_ADDR_UNDEF;
		INNO_DEV_ERR(dev, "No IO expander address found\n");
	} else {
		INNO_ALERT("io-exp-addr be found (0x%X)\n", info->io_exp_addr);
	}

	info->is_gmsl			= false;
	info->fw_name			= INNO_EVXM_YYMZ_FW_NAME;
	info->isp_id			= INNO_EVXM_YYMZ_MODULE_ID;
	info->sensor_id			= INNO_EVXM_YYMZ_SENSOR_ID;
	info->sensor_reg		= INNO_EVXM_YYMZ_SEN_CHIP_VER_REG;
	info->sensor_addr		= INNO_EVXM_YYMZ_SIP_ID;
	info->prvw_modes		= inno_evxm_yymz_prvw_modes;
	info->cur_mode			= &inno_evxm_yymz_prvw_modes[INNO_EVXM_YYMZ_DEF_MODE];
	info->prvw_modes_num	= ARRAY_SIZE(inno_evxm_yymz_prvw_modes);
	info->io_exp_init		= false;

	priv->info				= info;

#if defined(_INNO_FOR_NVIDIA_)
	ret = inno_comm_probe(client,
	                      priv,
	                      tc_dev,
	                      &inno_evxm_yymz_common_ops,
	                      &inno_evxm_yymz_subdev_internal_ops,
	                      &inno_evxm_yymz_ctrl_ops);
#elif defined(_INNO_FOR_NXP_)
	ret = inno_comm_probe(client,
	                      priv,
	                      fmt,
	                      &inno_evxm_yymz_sd_media_ops,
	                      &inno_evxm_yymz_subdev_ops,
	                      &inno_evxm_yymz_ctrl_ops);
#endif

	if (ret) {
		INNO_ERR("Error probing %s (%d)\n", priv->info->dev_name, ret);
		ret = -ENODEV;
		ret_line = __LINE__;
		goto exit;
	}

	return ret;

exit:
#if defined(_INNO_FOR_NVIDIA_)
	if (tc_dev)
		devm_kfree(dev, tc_dev);
#endif

	if (info)
		devm_kfree(dev, info);

	if (priv)
		devm_kfree(dev, priv);

	INNO_DEV_ERR(dev, "Error probing @ 0x%02X[%d, %d]\n", client->addr, ret, ret_line);

	return ret;
}

static struct i2c_driver inno_evxm_yymz_i2c_driver = {
	.driver = {
		.name			= INNO_EVXM_YYMZ_DEVICE_NAME,
		.owner			= THIS_MODULE,
		.of_match_table	= of_match_ptr(inno_evxm_yymz_of_match),
	},
	.probe				= inno_evxm_yymz_probe,
	.remove				= inno_comm_remove,
	.id_table			= inno_evxm_yymz_id,
};

module_i2c_driver(inno_evxm_yymz_i2c_driver);

MODULE_DESCRIPTION(INNO_DESCRIPTION(INNO_EVXM_YYMZ_DEVICE_NAME));
MODULE_AUTHOR(INNO_AUTHOR);
MODULE_LICENSE(INNO_LICENSE);
MODULE_ALIAS(INNO_EVXM_YYMZ_DEVICE_NAME);
MODULE_VERSION(INNO_VERSION);
