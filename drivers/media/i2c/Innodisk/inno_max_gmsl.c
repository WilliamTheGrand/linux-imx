/*
 * inno_max_gmsl.c - Innodisk Maxim GMSL driver
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
#include "inno_max_gmsl.h"
#include "inno_common_nv.h"
#elif defined(_INNO_FOR_NXP_)
#include "inno_common_nxp.h"
#endif

static struct mutex inno_max_gmsl_lock;

int inno_max_gmsl_serdes_setup(struct inno_cam_mod *priv)
{
	struct device *dev;
	int ret = INNO_RET_OK;
	int des_ret = INNO_RET_OK;

	if (!priv || !priv->ser_dev || !priv->dser_dev || !priv->i2c_client)
		return -EINVAL;

	dev = &priv->i2c_client->dev;

	INNO_DEV_ALERT(dev, "\n");

	mutex_lock(&inno_max_gmsl_lock);

	/* For now no separate power on required for serializer device */
	max9296_power_on(priv->dser_dev);

	/* Setup serdes addressing and control pipeline */
	ret = max9296_setup_link(priv->dser_dev, &priv->i2c_client->dev);
	if (ret) {
		INNO_DEV_ERR(priv->dser_dev, "GMSL deserializer link config failed (%d)\n", ret);
		goto error;
	} else {
		INNO_DEV_GMSL(priv->dser_dev, "GMSL deserializer link config successful (%d)\n", ret);
	}

	/* Proceed even if ser setup failed, to setup deser correctly */
	ret = max9295_setup_control(priv->ser_dev);
	if (ret) {
		INNO_DEV_ERR(priv->ser_dev, "GMSL serializer setup failed (%d)\n", ret);
	} else {
		INNO_DEV_GMSL(priv->ser_dev, "GMSL serializer setup successful (%d)\n", ret);
	}

	des_ret = max9296_setup_control(priv->dser_dev, &priv->i2c_client->dev);
	if (des_ret) {
		INNO_DEV_ERR(priv->dser_dev, "GMSL deserializer setup failed (%d)\n", ret);
		/* Overwrite err only if deser setup also failed */
		ret = des_ret;
	} else {
		INNO_DEV_GMSL(priv->dser_dev, "GMSL deserializer setup successful (%d)\n", ret);
	}

error:
	mutex_unlock(&inno_max_gmsl_lock);

	return ret;
}

void inno_max_gmsl_serdes_reset(struct inno_cam_mod *priv)
{
	struct device *dev = priv->dev;

	INNO_DEV_ALERT(dev, "\n");

	mutex_lock(&inno_max_gmsl_lock);

	/* Reset serdes addressing and control pipeline */
	max9295_reset_control(priv->ser_dev);
	max9296_reset_control(priv->dser_dev, &priv->i2c_client->dev);

	max9296_power_off(priv->dser_dev);

	mutex_unlock(&inno_max_gmsl_lock);
}

int inno_max_gmsl_start_streaming(struct tegracam_device *tc_dev)
{
	struct inno_cam_mod *priv = (struct inno_cam_mod *)tegracam_get_privdata(tc_dev);
	struct device *dev = tc_dev->dev;
	int ret = INNO_RET_OK;

	INNO_DEV_ALERT(dev, "\n");

	/* Enable serializer streaming */
	ret = max9295_setup_streaming(priv->ser_dev);
	if (ret) {
		INNO_DEV_ERR(dev, "GMSL serializer setup streaming failed (%d)\n", ret);
		goto exit;
	}

	/* Enable deserializer streaming */
	ret = max9296_setup_streaming(priv->dser_dev, dev);
	if (ret) {
		INNO_DEV_ERR(dev, "GMSL deserializer setup streaming failed (%d)\n", ret);
		goto exit;
	}

	/* Start deserializer streaming */
	ret = max9296_start_streaming(priv->dser_dev, dev);
	if (ret) {
		INNO_DEV_ERR(dev, "GMSL deserializer start streaming failed (%d)\n", ret);
		goto exit;
	}

	msleep(20);

	return ret;

exit:
	INNO_DEV_ERR(dev, "Error setting stream\n");

	return ret;
}

int inno_max_gmsl_stop_streaming(struct tegracam_device *tc_dev)
{
	struct inno_cam_mod *priv = (struct inno_cam_mod *)tegracam_get_privdata(tc_dev);
	struct device *dev = tc_dev->dev;
	int ret = INNO_RET_OK;

	INNO_DEV_ALERT(dev, "\n");

	/* Disable serdes streaming */
	max9296_stop_streaming(priv->dser_dev, dev);

	return ret;
}

int inno_max_gmsl_board_setup(struct inno_cam_mod *priv)
{
	struct device *dev = priv->dev;
	struct device_node *node = dev->of_node;
	struct device_node *ser_node;
	struct device_node *dser_node;
	struct device_node *gmsl;
	struct i2c_client *ser_i2c = NULL;
	struct i2c_client *dser_i2c = NULL;
	const char *str_value;
	const char *str_streams[2];
	int ret = INNO_RET_OK;
	int value = 0xFFFF;
	int  i;

	INNO_DEV_ALERT(dev, "\n");

	ret = of_property_read_u32(node, "reg", &priv->g_ctx.sdev_reg);
	if (ret < 0) {
		INNO_DEV_ERR(dev, "No reg found\n");
		goto error;
	} else {
		INNO_GMSL("reg be found (0x%X)\n", priv->g_ctx.sdev_reg);
	}

	ret = of_property_read_u32(node, "def-addr", &priv->g_ctx.sdev_def);
	if (ret < 0) {
		INNO_DEV_ERR(dev, "No def-addr found\n");
		ret = -ENODEV;
		goto error;
	} else {
		INNO_GMSL("def-addr be found (0x%X)\n", priv->g_ctx.sdev_def);
	}

	ret = of_property_read_u32(node, "io-exp-addr", &priv->g_ctx.io_exp_reg);
	if (ret < 0) {
		INNO_DEV_ERR(dev, "No io-exp-addr found\n");
		goto error;
	} else {
		INNO_GMSL("io-exp-addr be found (0x%X)\n", priv->g_ctx.io_exp_reg);
	}

	ret = of_property_read_u32(node, "def-io-exp-addr", &priv->g_ctx.io_exp_def);
	if (ret < 0) {
		INNO_DEV_ERR(dev, "No def-io-exp-addr found\n");
		ret = -ENODEV;
		goto error;
	} else {
		INNO_GMSL("def-io-exp-addr be found (0x%X)\n", priv->g_ctx.io_exp_def);
	}

	ser_node = of_parse_phandle(node, "inno,gmsl-ser-device", 0);
	if (ser_node == NULL) {
		INNO_DEV_ERR(dev, "Missing handle inno,gmsl-ser-device\n");
		ret = -ENODEV;
		goto error;
	} else {
		INNO_GMSL("gmsl-ser-device be found (0x%p)\n", ser_node);
	}

	ret = of_property_read_u32(ser_node, "reg", &priv->g_ctx.ser_reg);
	if (ret < 0) {
		INNO_DEV_ERR(dev, "No serializer reg found\n");
		goto error;
	} else {
		INNO_GMSL("ser-reg be found (0x%X)\n", priv->g_ctx.ser_reg);
	}

	ser_i2c = of_find_i2c_device_by_node(ser_node);
	of_node_put(ser_node);
	if (ser_i2c == NULL) {
		INNO_DEV_ERR(dev, "Missing serializer node\n");
		ret = -EPROBE_DEFER;
		goto error;
	}
	if (ser_i2c->dev.driver == NULL) {
		INNO_DEV_ERR(dev, "Missing serializer driver\n");
		ret = -EPROBE_DEFER;
		goto error;
	}
	priv->ser_dev = &ser_i2c->dev;

	dser_node = of_parse_phandle(node, "inno,gmsl-dser-device", 0);
	if (dser_node == NULL) {
		INNO_DEV_ERR(dev, "Missing handle inno,gmsl-dser-device\n");
		ret = -ENODEV;
		goto error;
	} else {
		INNO_GMSL("gmsl-dser-device be found (0x%p)\n", dser_node);
	}

	dser_i2c = of_find_i2c_device_by_node(dser_node);
	of_node_put(dser_node);
	if (dser_i2c == NULL) {
		INNO_DEV_ERR(dev, "Missing deserializer node\n");
		ret = -EPROBE_DEFER;
		goto error;
	}
	if (dser_i2c->dev.driver == NULL) {
		INNO_DEV_ERR(dev, "Missing deserializer driver\n");
		ret = -EPROBE_DEFER;
		goto error;
	}
	priv->dser_dev = &dser_i2c->dev;

	/* Populate g_ctx from DT */
	gmsl = of_get_child_by_name(node, "gmsl-link");
	if (gmsl == NULL) {
		INNO_DEV_ERR(dev, "Missing gmsl-link device node\n");
		ret = -EINVAL;
		goto error;
	} else {
		INNO_GMSL("gmsl-link be found (0x%p)\n", gmsl);
	}

	ret = of_property_read_string(gmsl, "dst-csi-port", &str_value);
	if (ret < 0) {
		INNO_DEV_ERR(dev, "No dst-csi-port found\n");
		goto error;
	} else {
		INNO_GMSL("dst-csi-port be found (%s)\n", str_value);
	}
	priv->g_ctx.dst_csi_port =
		(!strcmp(str_value, "a")) ? GMSL_CSI_PORT_A : GMSL_CSI_PORT_B;

	ret = of_property_read_string(gmsl, "src-csi-port", &str_value);
	if (ret < 0) {
		INNO_DEV_ERR(dev, "No src-csi-port found\n");
		goto error;
	} else {
		INNO_GMSL("src-csi-port be found (%s)\n", str_value);
	}
	priv->g_ctx.src_csi_port =
		(!strcmp(str_value, "a")) ? GMSL_CSI_PORT_A : GMSL_CSI_PORT_B;

	ret = of_property_read_string(gmsl, "csi-mode", &str_value);
	if (ret < 0) {
		INNO_DEV_ERR(dev, "No csi-mode found\n");
		goto error;
	} else {
		INNO_GMSL("csi-mode be found (%s)\n", str_value);
	}
	if (!strcmp(str_value, "1x4")) {
		priv->g_ctx.csi_mode = GMSL_CSI_1X4_MODE;
	} else if (!strcmp(str_value, "2x4")) {
		priv->g_ctx.csi_mode = GMSL_CSI_2X4_MODE;
	} else if (!strcmp(str_value, "4x2")) {
		priv->g_ctx.csi_mode = GMSL_CSI_4X2_MODE;
	} else if (!strcmp(str_value, "2x2")) {
		priv->g_ctx.csi_mode = GMSL_CSI_2X2_MODE;
	} else {
		INNO_DEV_ERR(dev, "Invalid csi mode\n");
		ret = -EINVAL;
		goto error;
	}

	ret = of_property_read_string(gmsl, "serdes-csi-link", &str_value);
	if (ret < 0) {
		INNO_DEV_ERR(dev, "No serdes-csi-link found\n");
		goto error;
	} else {
		INNO_GMSL("serdes-csi-link be found (%s)\n", str_value);
	}
	priv->g_ctx.serdes_csi_link =
		(!strcmp(str_value, "a")) ? GMSL_SERDES_CSI_LINK_A : GMSL_SERDES_CSI_LINK_B;

	ret = of_property_read_u32(gmsl, "st-vc", &value);
	if (ret < 0) {
		INNO_DEV_ERR(dev, "No st-vc info\n");
		goto error;
	} else {
		INNO_GMSL("st-vc be found (0x%X)\n", value);
	}
	priv->g_ctx.st_vc = value;

	ret = of_property_read_u32(gmsl, "vc-id", &value);
	if (ret < 0) {
		INNO_DEV_ERR(dev, "No vc-id info\n");
		goto error;
	} else {
		INNO_GMSL("vc-id be found (0x%X)\n", value);
	}
	priv->g_ctx.dst_vc = value;

	ret = of_property_read_u32(gmsl, "num-lanes", &value);
	if (ret < 0) {
		INNO_DEV_ERR(dev, "No num-lanes info\n");
		goto error;
	} else {
		INNO_GMSL("num-lanes be found (0x%X)\n", value);
	}
	priv->g_ctx.num_csi_lanes = value;

	priv->g_ctx.num_streams = of_property_count_strings(gmsl, "streams");
	if (priv->g_ctx.num_streams <= 0) {
		INNO_DEV_ERR(dev, "No streams found\n");
		ret = -EINVAL;
		goto error;
	} else {
		INNO_GMSL("streams be found (0x%X)\n", priv->g_ctx.num_streams);
	}

	for (i = 0; i < priv->g_ctx.num_streams; i++) {
		of_property_read_string_index(gmsl, "streams", i, &str_streams[i]);
		if (!str_streams[i]) {
			INNO_DEV_ERR(dev, "Invalid stream info\n");
			goto error;
		} else {
			INNO_GMSL("streams[%d] be found (%s)\n", i, str_streams[i]);
		}
		if (!strcmp(str_streams[i], "raw12")) {
			priv->g_ctx.streams[i].st_data_type = GMSL_CSI_DT_RAW_12;
#if INNO_MAX_GMSL
		} else if (!strcmp(str_streams[i], "yuv422_8")) {
			priv->g_ctx.streams[i].st_data_type = GMSL_CSI_DT_YUV422_8;
#endif
		} else if (!strcmp(str_streams[i], "embed")) {
			priv->g_ctx.streams[i].st_data_type = GMSL_CSI_DT_EMBED;
		} else if (!strcmp(str_streams[i], "ued-u1")) {
			priv->g_ctx.streams[i].st_data_type = GMSL_CSI_DT_UED_U1;
		} else {
			INNO_DEV_ERR(dev, "Invalid stream data type\n");
			goto error;
		}
	}

	priv->g_ctx.s_dev = dev;

	return ret;

error:
	return ret;
}

int inno_max_gmsl_probe(struct inno_cam_mod *priv)
{
	struct device *dev = priv->dev;
	int ret = INNO_RET_OK;

	INNO_DEV_ALERT(dev, "\n");

	/* Pair sensor to serializer dev */
	ret = max9295_sdev_pair(priv->ser_dev, &priv->g_ctx);
	if (ret) {
		INNO_DEV_ERR(priv->ser_dev, "GMSL serializer pairing failed (%d)\n", ret);
		return ret;
	} else {
		INNO_DEV_GMSL(priv->ser_dev, "GMSL serializer pairing successful (%d)\n", ret);
	}

	/* Register sensor to deserializer dev */
	ret = max9296_sdev_register(priv->dser_dev, &priv->g_ctx);
	if (ret) {
		INNO_DEV_ERR(priv->dser_dev, "GMSL deserializer register failed (%d)\n", ret);
		return ret;
	} else {
		INNO_DEV_GMSL(priv->dser_dev, "GMSL deserializer register successful (%d)\n", ret);
	}

	/*
	 * GMSL serdes setup
	 *
	 * Sensor power on/off should be the right place for serdes
	 * setup/reset. But the problem is, the total required delay
	 * in serdes setup/reset exceeds the frame wait timeout, looks to
	 * be related to multiple channel open and close sequence
	 * issue (#BUG 200477330).
	 * Once this bug is fixed, these may be moved to power on/off.
	 * The delays in serdes is as per guidelines and can't be reduced,
	 * so it is placed in probe/remove, though for that, deserializer
	 * would be powered on always post boot, until 1.2v is supplied
	 * to deserializer from CVB.
	 */
	ret = inno_max_gmsl_serdes_setup(priv);
	if (ret) {
		INNO_DEV_ERR(dev, "GMSL serdes setup failed (%d)\n", ret);
		return ret;
	} else {
		INNO_DEV_GMSL(dev, "GMSL serdes setup successful (%d)\n", ret);
	}

	return ret;
}

int inno_max_gmsl_remove(struct inno_cam_mod *priv)
{
	struct device *dev = priv->dev;
	int ret = INNO_RET_OK;

	INNO_DEV_ALERT(dev, "\n");

	inno_max_gmsl_serdes_reset(priv);

	/* unpair sensor to serializer dev */
	ret = max9295_sdev_unpair(priv->ser_dev, &priv->i2c_client->dev);
	if (ret) {
		INNO_DEV_ERR(priv->ser_dev, "GMSL serializer unpairing failed (%d)\n", ret);
		return ret;
	} else {
		INNO_DEV_GMSL(priv->ser_dev, "GMSL serializer unpairing successful (%d)\n", ret);
	}

	/* Unregister sensor to deserializer dev */
	ret = max9296_sdev_unregister(priv->dser_dev, &priv->i2c_client->dev);
	if (ret) {
		INNO_DEV_ERR(priv->dser_dev, "GMSL deserializer unregister failed (%d)\n", ret);
		return ret;
	} else {
		INNO_DEV_GMSL(priv->dser_dev, "GMSL deserializer unregister successful (%d)\n", ret);
	}

	return ret;
}

int inno_max_gmsl_start(struct tegracam_device *tc_dev)
{
	struct inno_cam_mod *priv = (struct inno_cam_mod *)tegracam_get_privdata(tc_dev);
	struct device *dev = tc_dev->dev;
	int ret = INNO_RET_OK;

	INNO_DEV_ALERT(dev, "\n");

	if (priv->info->is_gmsl) {
		/* Enable serdes streaming */
		ret = max9295_setup_streaming(priv->ser_dev);
		if (ret) {
			INNO_DEV_ERR(priv->ser_dev, "Error Max9295 Setup Streaming (%d)\n", ret);
			return ret;
		}

		ret = max9296_setup_streaming(priv->dser_dev, dev);
		if (ret) {
			INNO_DEV_ERR(priv->dser_dev, "Error Max9296 Setup Streaming (%d)\n", ret);
			return ret;
		}

		ret = max9296_start_streaming(priv->dser_dev, dev);
		if (ret) {
			INNO_DEV_ERR(priv->dser_dev, "Error Max9296 Start Streaming (%d)\n", ret);
			return ret;
		}
	}

	return ret;
}

int inno_max_gmsl_stop(struct tegracam_device *tc_dev)
{
	struct inno_cam_mod *priv = (struct inno_cam_mod *)tegracam_get_privdata(tc_dev);
	struct device *dev = tc_dev->dev;
	int ret = INNO_RET_OK;

	INNO_DEV_ALERT(dev, "\n");

	if (priv->info->is_gmsl) {
		/* Disable serdes streaming */
		ret = max9296_stop_streaming(priv->dser_dev, dev);
		if (ret) {
			INNO_DEV_ERR(priv->dser_dev, "Error Max9296 Stop Streaming (%d)\n", ret);
			return ret;
		}
	}

	return ret;
}

int inno_max_gmsl_init(struct inno_cam_mod *priv)
{
	struct device *dev = priv->dev;
	int ret = INNO_RET_OK;

	INNO_DEV_ALERT(dev, "\n");

	priv->info->is_gmsl = false;

	ret = inno_max_gmsl_board_setup(priv);
	if (ret == -ENODEV) {
		INNO_DEV_ERR(dev, "Unable to get GMSL device (%d)\n", ret);
		/* Continue to register V4L2 device for MIPI */
		ret = INNO_RET_OK;
	} else if (ret) {
		INNO_DEV_ERR(dev, "GMSL setup failed (%d)\n", ret);
		return ret;
	} else {
		INNO_GMSL("GMSL board setup successful (%d)\n", ret);

		ret = inno_max_gmsl_probe(priv);
		if (ret) {
			INNO_DEV_ERR(dev, "GMSL probe failed (%d)\n", ret);
			return ret;
		} else {
			INNO_DEV_GMSL(dev, "GMSL probe successful (%d)\n", ret);
		}

		priv->info->is_gmsl = true;
	}

	return ret;
}