/*
 * inno_common_nv.c - Innodisk Common Function
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
#include "inno_common_mode_tbls.h"
#if defined(_INNO_FOR_NVIDIA_)
#include "inno_max_gmsl.h"
#include "inno_common_nv.h"
#elif defined(_INNO_FOR_NXP_)
#include "inno_common_nxp.h"
#endif
#include "inno_iic.h"

#if defined(_INNO_FOR_NVIDIA_)
/* Use general GPIO instead of I2C GPIO Expander on EVDM-OOM1 */
#include "../../platform/tegra/camera/camera_gpio.h"
#endif

void inno_comm_nv_toggle_gpio(u32 gpio, s32 val)
{
	if (gpio_cansleep(gpio)) {
		gpio_direction_output(gpio, val);
		gpio_set_value_cansleep(gpio, val);
	} else {
		gpio_direction_output(gpio, val);
		gpio_set_value(gpio, val);
	}
}

int inno_comm_gpio_level(struct inno_cam_mod *priv, int bit, s32 val)
{
	struct camera_common_data *s_data = priv->s_data;
	struct camera_common_power_rail *pw = s_data->power;
	int ret = INNO_RET_OK;

	INNO_DBG("Bit#%d: %s\n", bit, (val == INNO_GPIO_LEVEL_HIGH) ? "HIGH" : "LOW");

	switch (bit) {
		case INNO_II2_IO_EXP_RST:
			inno_comm_nv_toggle_gpio(pw->reset_gpio, val);
			break;

		case INNO_II2_IO_EXP_PWR_EN:
		case INNO_II2_IO_EXP_STANDBY:
			inno_comm_nv_toggle_gpio(pw->pwdn_gpio, val);
			break;

		default:
			break;
	}
	msleep(INNO_GPIO_LEVEL_DELAY_MS);

	return ret;
}

int inno_comm_nv_set_gain(struct tegracam_device *tc_dev, s64 val)
{
	struct camera_common_data *s_data = tc_dev->s_data;
	struct inno_cam_mod *priv = (struct inno_cam_mod *)tc_dev->priv;
	const struct sensor_mode_properties *mode = &s_data->sensor_props.sensor_modes[s_data->mode_prop_idx];
	int ret = INNO_RET_OK;

	INNO_DEV_ALERT(priv->dev, "%lld\n", val);

	mutex_lock(&priv->inno_cam_mod_lock);
	(void)priv;
	(void)s_data;
	(void)mode;
	mutex_unlock(&priv->inno_cam_mod_lock);

	return ret;
}

int inno_comm_nv_set_exposure(struct tegracam_device *tc_dev, s64 val)
{
	struct camera_common_data *s_data = tc_dev->s_data;
	struct inno_cam_mod *priv = (struct inno_cam_mod *)tc_dev->priv;
	const struct sensor_mode_properties *mode = &s_data->sensor_props.sensor_modes[s_data->mode_prop_idx];
	int ret = INNO_RET_OK;

	INNO_DEV_ALERT(priv->dev, "%lld\n", val);

	mutex_lock(&priv->inno_cam_mod_lock);
	(void)priv;
	(void)s_data;
	(void)mode;
	mutex_unlock(&priv->inno_cam_mod_lock);

	return ret;
}

int inno_comm_nv_set_frame_rate(struct tegracam_device *tc_dev, s64 val)
{
	struct camera_common_data *s_data = tc_dev->s_data;
	struct inno_cam_mod *priv = (struct inno_cam_mod *)tc_dev->priv;
	const struct sensor_mode_properties *mode = &s_data->sensor_props.sensor_modes[s_data->mode_prop_idx];
	int ret = INNO_RET_OK;

	INNO_DEV_ALERT(priv->dev, "%lld\n", val);

	mutex_lock(&priv->inno_cam_mod_lock);
	(void)priv;
	(void)s_data;
	(void)mode;
	mutex_unlock(&priv->inno_cam_mod_lock);

	return ret;
}

int inno_comm_nv_set_group_hold(struct tegracam_device *tc_dev, bool val)
{
	struct inno_cam_mod *priv = (struct inno_cam_mod *)tc_dev->priv;
	int ret = INNO_RET_OK;

	INNO_DEV_ALERT(priv->dev, "%d\n", val);

	mutex_lock(&priv->inno_cam_mod_lock);
	mutex_unlock(&priv->inno_cam_mod_lock);

	return ret;
}

int inno_comm_nv_fill_string_ctrl(struct tegracam_device *tc_dev, struct v4l2_ctrl *ctrl)
{
	struct inno_cam_mod *priv = (struct inno_cam_mod *)tc_dev->priv;
	int ret = INNO_RET_OK;

	INNO_DEV_ALERT(priv->dev, "\n");

	mutex_lock(&priv->inno_cam_mod_lock);
	mutex_unlock(&priv->inno_cam_mod_lock);

	return ret;
}

int inno_comm_nv_write_table_8(struct inno_cam_mod *priv, const INNO_COMM_REG_8 table[])
{
	struct camera_common_data *s_data = priv->s_data;

	INNO_DBG("\n");

	return regmap_util_write_table_8(s_data->regmap, table, NULL, 0, INNO_COMM_TABLE_WAIT_MS, INNO_COMM_TABLE_END);
}

int inno_comm_nv_write_table_16(struct inno_cam_mod *priv, const INNO_COMM_REG_16 table[])
{
	struct camera_common_data *s_data = priv->s_data;

	INNO_DBG("\n");

	return regmap_util_write_table_16_as_8(s_data->regmap, table, NULL, 0, INNO_COMM_TABLE_WAIT_MS, INNO_COMM_TABLE_END);
}

int inno_comm_nv_power_on(struct camera_common_data *s_data)
{
	struct inno_cam_mod *priv = (struct inno_cam_mod *)s_data->tegracam_ctrl_hdl->tc_dev->priv;
	struct camera_common_power_rail *pw = s_data->power;
	struct camera_common_pdata *pdata = s_data->pdata;
	struct device *dev = s_data->dev;
	int ret = INNO_RET_OK;

	INNO_DEV_ALERT(priv->dev, "\n");

	if (pdata && pdata->power_on) {
		ret = pdata->power_on(pw);
		if (ret)
			INNO_DEV_ERR(dev, " failed (%d)\n", ret);
		else
			pw->state = SWITCH_ON;

		return ret;
	}

	pw->state = SWITCH_ON;
	ret = inno_comm_power(priv, pw->state);

	return ret;
}

int inno_comm_nv_power_off(struct camera_common_data *s_data)
{
	struct inno_cam_mod *priv = (struct inno_cam_mod *)s_data->tegracam_ctrl_hdl->tc_dev->priv;
	struct camera_common_power_rail *pw = s_data->power;
	struct camera_common_pdata *pdata = s_data->pdata;
	struct device *dev = s_data->dev;
	int ret = INNO_RET_OK;

	INNO_DEV_ALERT(priv->dev, "\n");

	if (pdata && pdata->power_off) {
		ret = pdata->power_off(pw);
		if (ret)
			INNO_DEV_ERR(dev, " failed (%d)\n", ret);
		else
			goto power_off_done;

		return ret;
	}

power_off_done:
	pw->state = SWITCH_OFF;
	ret = inno_comm_power(priv, pw->state);

	return ret;
}

int inno_comm_nv_power_get(struct tegracam_device *tc_dev)
{
	struct camera_common_data *s_data = tc_dev->s_data;
	struct camera_common_power_rail *pw = s_data->power;
	struct camera_common_pdata *pdata = s_data->pdata;
	struct device *dev = tc_dev->dev;
	const char *mclk_name;
	struct clk *parent;
	int ret = INNO_RET_OK;

	INNO_DEV_ALERT(dev, "\n");

	if (!pdata) {
		INNO_DEV_ERR(dev, "pdata missing\n");
		return -EFAULT;
	}

	/* Sensor MCLK (aka. INCK) */
	if (pdata->mclk_name) {
		mclk_name = pdata->mclk_name ? pdata->mclk_name : "extperiph1";
		pw->mclk = devm_clk_get(dev, pdata->mclk_name);
		if (IS_ERR(pw->mclk)) {
			INNO_DEV_ERR(dev, "Unable to get clock %s\n", pdata->mclk_name);
			return PTR_ERR(pw->mclk);
		}

		if (pdata->parentclk_name) {
			parent = devm_clk_get(dev, pdata->parentclk_name);
			if (IS_ERR(parent))
				INNO_DEV_ERR(dev, "Unable to get parent clock %s", pdata->parentclk_name);
			else
				clk_set_parent(pw->mclk, parent);
		}
	}

	/* Analog 2.8v */
	if (pdata->regulators.avdd)
		ret |= camera_common_regulator_get(dev, &pw->avdd, pdata->regulators.avdd);
	/* IO 1.8v */
	if (pdata->regulators.iovdd)
		ret |= camera_common_regulator_get(dev, &pw->iovdd, pdata->regulators.iovdd);
	/* Digital 1.2v */
	if (pdata->regulators.dvdd)
		ret |= camera_common_regulator_get(dev, &pw->dvdd, pdata->regulators.dvdd);
	if (ret) {
		INNO_DEV_ERR(dev, "Unable to get regulator(s)\n");
		goto exit;
	}

	/* Reset GPIO */
	pw->reset_gpio = pdata->reset_gpio;
	ret = gpio_request(pw->reset_gpio, "inno_reset_gpio");
	if (ret) {
		INNO_DEV_ERR(dev, "Unable to request reset-gpios (%d)\n", ret);
		goto exit;
	} else  {
		INNO_DBG("Request reset-gpios (%d, 0x%X)\n", ret, pw->reset_gpio);
	}

	/* Power Down GPIO */
	pw->pwdn_gpio = pdata->pwdn_gpio;
	ret = gpio_request(pw->pwdn_gpio, "inno_pwdn_gpio");
	if (ret) {
		INNO_DEV_ERR(dev, "Unable to request pwdn-gpios (%d)\n", ret);
		/* Power Down GPIO is NOT setted @ DTS now */
		ret = INNO_RET_OK;
		goto exit;
	} else  {
		INNO_DBG("Request pwdn-gpios (%d, 0x%X)\n", ret, pw->pwdn_gpio);
	}

exit:
	pw->state = SWITCH_OFF;

	return ret;
}

int inno_comm_nv_power_put(struct tegracam_device *tc_dev)
{
	struct camera_common_data *s_data = tc_dev->s_data;
	struct camera_common_power_rail *pw = s_data->power;
	struct device *dev = tc_dev->dev;
	int ret = INNO_RET_OK;

	INNO_DEV_ALERT(dev, "\n");

	if (unlikely(!pw))
		return -EFAULT;

	if (likely(pw->dvdd))
		devm_regulator_put(pw->dvdd);

	if (likely(pw->avdd))
		devm_regulator_put(pw->avdd);

	if (likely(pw->iovdd))
		devm_regulator_put(pw->iovdd);

	pw->dvdd = NULL;
	pw->avdd = NULL;
	pw->iovdd = NULL;

	if (likely(pw->reset_gpio))
		gpio_free(pw->reset_gpio);

	if (likely(pw->pwdn_gpio))
		gpio_free(pw->pwdn_gpio);

	return ret;
}

int inno_comm_nv_read_reg(struct camera_common_data *s_data, u16 addr, u8 *val)
{
	struct inno_cam_mod *priv = (struct inno_cam_mod *)s_data->tegracam_ctrl_hdl->tc_dev->priv;
	int ret = INNO_RET_OK;
	u32 reg_val = 0U;

	ret = inno_iic_rd_reg(priv, addr, &reg_val, INNO_IIC_VAL_08BIT);
	INNO_RET_ERR_HANDLE(ret);

	*val = reg_val & 0xFF;

	return ret;
}

int inno_comm_nv_write_reg(struct camera_common_data *s_data, u16 addr, u8 val)
{
	struct inno_cam_mod *priv = (struct inno_cam_mod *)s_data->tegracam_ctrl_hdl->tc_dev->priv;
	int ret = INNO_RET_OK;

	inno_iic_wr_reg(priv, addr, (u32)val, INNO_IIC_VAL_08BIT);
	INNO_RET_ERR_HANDLE(ret);

	return ret;
}

struct camera_common_pdata *inno_comm_nv_parse_dt(struct tegracam_device *tc_dev)
{
	struct device *dev = tc_dev->dev;
	struct device_node *np = dev->of_node;
	struct camera_common_pdata *board_priv_pdata;
	int ret = INNO_RET_OK;
	int reset_gpio = 0, pwdn_gpio = 0;

	INNO_DEV_ALERT(dev, "\n");

	board_priv_pdata = devm_kzalloc(dev, sizeof(*board_priv_pdata), GFP_KERNEL);
	if (!board_priv_pdata)
		return NULL;

	reset_gpio = of_get_named_gpio(np, "reset-gpios", 0);
	if (reset_gpio < 0) {
		INNO_DEV_ERR(dev, "reset-gpios not found\n");
		board_priv_pdata->reset_gpio = 0;
	} else {
		board_priv_pdata->reset_gpio = (unsigned int)reset_gpio;
		INNO_DBG("reset-gpios be found (0x%X)\n", board_priv_pdata->reset_gpio);
	}

	pwdn_gpio = of_get_named_gpio(np, "pwdn-gpios", 0);
	if (pwdn_gpio < 0) {
		INNO_DEV_ERR(dev, "pwdn-gpios not found\n");
		board_priv_pdata->pwdn_gpio = 0;
	} else {
		board_priv_pdata->pwdn_gpio = (unsigned int)pwdn_gpio;
		INNO_DBG("pwdn-gpios be found (0x%X)\n", board_priv_pdata->pwdn_gpio);
	}

	ret = of_property_read_string(np, "mclk", &board_priv_pdata->mclk_name);
	if (ret)
		INNO_DEV_ERR(dev, "mclk name not present, " "assume sensor driven externally\n");

	ret = of_property_read_string(np, "avdd-reg", &board_priv_pdata->regulators.avdd);
	ret |= of_property_read_string(np, "iovdd-reg", &board_priv_pdata->regulators.iovdd);
	ret |= of_property_read_string(np, "dvdd-reg", &board_priv_pdata->regulators.dvdd);
	if (ret)
		INNO_DEV_ERR(dev, "avdd, iovdd and/or dvdd reglrs. not present, " "assume sensor powered independently\n");

	board_priv_pdata->has_eeprom = of_property_read_bool(np, "has-eeprom");

	return board_priv_pdata;
}
