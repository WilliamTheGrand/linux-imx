/*
 * inno_common.c - Innodisk Common Function
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
#endif
#include "inno_iic.h"

#if defined(_INNO_FOR_NVIDIA_)
/* Use general GPIO instead of I2C GPIO Expander on EVDM-OOM1 */
#include "../../platform/tegra/camera/camera_gpio.h"
#endif

struct regmap_config sensor_regmap_08_config = {
	.reg_bits = 16,
	.val_bits = 8,
	.reg_stride = 1,
	.reg_format_endian = REGMAP_ENDIAN_BIG,
	.val_format_endian = REGMAP_ENDIAN_BIG,
	.cache_type = REGCACHE_RBTREE,
#if LINUX_VERSION_CODE < KERNEL_VERSION(5, 4, 0)
	.use_single_rw = false,
#else
	.use_single_read = false,
	.use_single_write = false,
#endif
};

struct regmap_config sensor_regmap_16_config = {
	.reg_bits = 16,
	.val_bits = 16,
	.reg_stride = 2,
	.reg_format_endian = REGMAP_ENDIAN_BIG,
	.val_format_endian = REGMAP_ENDIAN_BIG,
	.cache_type = REGCACHE_RBTREE,
#if LINUX_VERSION_CODE < KERNEL_VERSION(5, 4, 0)
	.use_single_rw = false,
#else
	.use_single_read = false,
	.use_single_write = false,
#endif
};

struct regmap_config sensor_regmap_32_config = {
	.reg_bits = 16,
	.val_bits = 32,
	.reg_stride = 4,
	.reg_format_endian = REGMAP_ENDIAN_BIG,
	.val_format_endian = REGMAP_ENDIAN_BIG,
	.cache_type = REGCACHE_RBTREE,
#if LINUX_VERSION_CODE < KERNEL_VERSION(5, 4, 0)
	.use_single_rw = false,
#else
	.use_single_read = false,
	.use_single_write = false,
#endif
};

#if defined(_INNO_FOR_NXP_)
inline void msleep_range(unsigned int delay_base)
{
	usleep_range(delay_base * 1000, delay_base * 1000 + 500);
}
#endif

void inno_comm_prt_tbls_8(INNO_COMM_REG_8 table[], u16 end_addr)
{
	INNO_COMM_REG_8 *next;
	int cnt = 0;

	(void)cnt;

	for (next = table;; next++) {
		if (next->addr == INNO_COMM_TABLE_WAIT_MS) {
			INNO_DBG("%02d: Sleep %dms\n", cnt++, next->val);
		} else if (next->addr == INNO_COMM_TABLE_END) {
			INNO_DBG("%02d: End\n", cnt++);
		} else{
			INNO_DBG("%02d: 0x%04X, 0x%02X\n", cnt++, next->addr, next->val);
		}

		/* Handle special address values */
		if (next->addr == end_addr)
			break;
	}
}

void inno_comm_prt_tbls_16(INNO_COMM_REG_16 table[], u16 end_addr)
{
	INNO_COMM_REG_16 *next;
	int cnt = 0;

	(void)cnt;

	for (next = table;; next++) {
		if (next->addr == INNO_COMM_TABLE_WAIT_MS) {
			INNO_DBG("%02d: Sleep %dms\n", cnt++, next->val);
		} else if (next->addr == INNO_COMM_TABLE_END) {
			INNO_DBG("%02d: End\n", cnt++);
		} else {
			INNO_DBG("%02d: 0x%04X, 0x%04X\n", cnt++, next->addr, INNO_SWAP16(next->val));
		}

		/* Handle special address values */
		if (next->addr == end_addr)
			break;
	}
}

int inno_comm_dump_dbg_reg(struct inno_cam_mod *priv)
{
	u16 addr[] = {
		SYS_START,
		PREVIEW_WIDTH,
		PREVIEW_HEIGHT,
		PREVIEW_MAX_FPS,
		REVIEW_AE_MAX_ET,
		PREVIEW_OUT_FMT,
		PREVIEW_SENSOR_MODE
	};
	int ret = INNO_RET_OK;
	int i;
	u32 reg_val = 0U;

	for (i = 0; i < ARRAY_SIZE(addr); i++) {
		if (addr[i] == REVIEW_AE_MAX_ET) {
			ret = inno_iic_rd_reg(priv, addr[i], &reg_val, INNO_IIC_VAL_32BIT);
			INNO_RET_ERR_HANDLE(ret);
		} else {
			ret = inno_iic_rd_reg(priv, addr[i], &reg_val, INNO_IIC_VAL_16BIT);
			INNO_RET_ERR_HANDLE(ret);
		}
	}

	return ret;
}

int inno_comm_stall(struct inno_cam_mod *priv, bool stall)
{
	int ret = INNO_RET_OK;
#if INNO_SYS_START_EN
	INNO_DBG("%s\n", (stall) ? "True" : "False");

	if (stall) {
		ret = inno_iic_wr_reg(priv, SYS_START, SYS_START_COMMON, INNO_IIC_VAL_16BIT);
		INNO_RET_ERR_HANDLE(ret);
		ret = inno_iic_wr_reg(priv, SYS_START, SYS_START_STOP_STREAM, INNO_IIC_VAL_16BIT);
		INNO_RET_ERR_HANDLE(ret);
	} else {
		ret = inno_iic_wr_reg(priv, SYS_START, SYS_START_COMMON, INNO_IIC_VAL_16BIT);
		INNO_RET_ERR_HANDLE(ret);
		ret = inno_iic_wr_reg(priv, SYS_START, SYS_START_START_STREAM, INNO_IIC_VAL_16BIT);
		INNO_RET_ERR_HANDLE(ret);
	}
	inno_comm_dump_dbg_reg(priv);
#endif
	return ret;
}

int inno_comm_set_format(struct inno_cam_mod *priv, int index)
{
	int ret = INNO_RET_OK;

	INNO_DBG("\n");

	if (!strcmp(priv->info->dev_name, INNO_EV2M_OOM1_DEVICE_NAME) ||
		!strcmp(priv->info->dev_name, INNO_EVDM_OOM1_DEVICE_NAME) ||
		!strcmp(priv->info->dev_name, INNO_EV2M_OOM2_DEVICE_NAME) ||
		!strcmp(priv->info->dev_name, INNO_EV8M_OOM1_DEVICE_NAME)) {
		ret = inno_iic_wr_reg(priv, ATOMIC, ATOMIC_RECORD, INNO_IIC_VAL_16BIT);
		INNO_RET_ERR_HANDLE(ret);
		ret = inno_iic_wr_reg(priv, PREVIEW_WIDTH, priv->info->prvw_modes[index].width, INNO_IIC_VAL_16BIT);
		INNO_RET_ERR_HANDLE(ret);
		ret = inno_iic_wr_reg(priv, PREVIEW_HEIGHT, priv->info->prvw_modes[index].height, INNO_IIC_VAL_16BIT);
		INNO_RET_ERR_HANDLE(ret);
		ret = inno_iic_wr_reg(priv, ATOMIC, ATOMIC_UPDATE_ONSEMI, INNO_IIC_VAL_16BIT);
		INNO_RET_ERR_HANDLE(ret);
		msleep_range(INNO_COMM_WAIT_MS_MODE);
	
		ret = inno_iic_wr_reg(priv, PREVIEW_MAX_FPS, priv->info->prvw_modes[index].fps << 0x08, INNO_IIC_VAL_16BIT);
		INNO_RET_ERR_HANDLE(ret);
		ret = inno_iic_wr_reg(priv, REVIEW_AE_MAX_ET, AE_MAX_ET(priv->info->prvw_modes[index].fps), INNO_IIC_VAL_32BIT);
		INNO_RET_ERR_HANDLE(ret);
		msleep_range(INNO_COMM_WAIT_MS_MODE);

#if INNO_PREVIEW_OUT_FMT_EN
		ret = inno_iic_wr_reg(priv, PREVIEW_OUT_FMT, priv->info->prvw_modes[index].format, INNO_IIC_VAL_16BIT);
		INNO_RET_ERR_HANDLE(ret);
		msleep_range(INNO_COMM_WAIT_MS_MODE);
#endif

		ret = inno_iic_wr_reg(priv, PREVIEW_SENSOR_MODE, priv->info->prvw_modes[index].mode, INNO_IIC_VAL_16BIT);
		INNO_RET_ERR_HANDLE(ret);
		msleep_range(INNO_COMM_WAIT_MS_MODE);

		inno_comm_dump_dbg_reg(priv);
	} else if (!strcmp(priv->info->dev_name, INNO_EV2M_GOM1_DEVICE_NAME)) {
		ret = inno_iic_wr_reg(priv, GENESYS_DISPLAY_MODE, priv->info->prvw_modes[index].mode, INNO_IIC_VAL_08BIT);
		INNO_RET_ERR_HANDLE(ret);
	}

	return ret;
}

int inno_comm_power(struct inno_cam_mod *priv, int val)
{
	int ret = INNO_RET_OK;

	INNO_DEV_ALERT(priv->dev, "%s\n", (val == SWITCH_ON) ? "ON" : "OFF");

	if (!strcmp(priv->info->dev_name, INNO_EV2M_OOM1_DEVICE_NAME) ||
		!strcmp(priv->info->dev_name, INNO_EV2M_OOM2_DEVICE_NAME) ||
		!strcmp(priv->info->dev_name, INNO_EV8M_OOM1_DEVICE_NAME)) {
		if (!priv->info->io_exp_init) {
			ret = inno_iic_io_exp_device_init(priv);
			if (ret) {
				INNO_DEV_ERR(priv->dev, "Error initializing IO Expander (%d)\n", ret);
				return ret;
			} else {
				priv->info->io_exp_init = true;
			}
		}

		ret = inno_iic_io_exp_level(priv, INNO_II2_IO_EXP_STANDBY, (val == SWITCH_ON) ? INNO_GPIO_LEVEL_HIGH : INNO_GPIO_LEVEL_LOW);
	} else if (!strcmp(priv->info->dev_name, INNO_EVDM_OOM1_DEVICE_NAME)) {
		ret = inno_comm_gpio_level(priv, INNO_II2_IO_EXP_STANDBY, (val == SWITCH_ON) ? INNO_GPIO_LEVEL_HIGH : INNO_GPIO_LEVEL_LOW);
	}

	return ret;
}

int inno_comm_gpio_rst(struct inno_cam_mod *priv)
{
	int ret = INNO_RET_OK;

	INNO_DBG("\n");

	mutex_lock(&priv->inno_cam_mod_lock);
	ret = inno_comm_gpio_level(priv, INNO_II2_IO_EXP_RST, INNO_GPIO_LEVEL_LOW);
	msleep(INNO_GPIO_RST_DELAY_MS);
	ret = inno_comm_gpio_level(priv, INNO_II2_IO_EXP_RST, INNO_GPIO_LEVEL_HIGH);
	mutex_unlock(&priv->inno_cam_mod_lock);

	return ret;
}

int inno_comm_rst(struct inno_cam_mod *priv)
{
	int ret = INNO_RET_OK;

	INNO_DBG("\n");

	if (!strcmp(priv->info->dev_name, INNO_EV2M_OOM1_DEVICE_NAME) ||
		!strcmp(priv->info->dev_name, INNO_EV2M_OOM2_DEVICE_NAME) ||
		!strcmp(priv->info->dev_name, INNO_EV8M_OOM1_DEVICE_NAME)) {
		if (!priv->info->io_exp_init) {
			ret = inno_iic_io_exp_device_init(priv);
			if (ret) {
				INNO_DEV_ERR(priv->dev, "Error initializing IO Expander (%d)\n", ret);
				return ret;
			} else {
				priv->info->io_exp_init = true;
			}
		}

		ret = inno_iic_io_exp_rst(priv);
	} else if (!strcmp(priv->info->dev_name, INNO_EVDM_OOM1_DEVICE_NAME)) {
		ret = inno_comm_gpio_rst(priv);
	}

	return ret;
}

int inno_comm_wait_idle(struct inno_cam_mod *priv)
{
	struct device *dev = priv->dev;
	int ret = INNO_RET_OK;
	int try_cnt = 0;
	u32 reg_val = 0U;

	INNO_DBG("\n");

    while (1) {
        ret = inno_iic_rd_reg(priv, DMA_CTRL, &reg_val, INNO_IIC_VAL_16BIT);
        if ((reg_val & DMA_CTRL_MODE_MASK) == DMA_CTRL_MODE_IDLE ||
			(reg_val & DMA_CTRL_MODE_MASK) == DMA_CTRL_MODE_COPY) {
            return INNO_RET_OK;
        } else if (try_cnt == POLL_REG_COUNT) {
        	INNO_DEV_ERR(dev, "DMA timeout\n");
            return -ETIMEDOUT;
    	}
		INNO_RET_ERR_HANDLE(ret);

		try_cnt++;

        msleep(POLL_REG_DELAY);
    }

    return ret;
}

int inno_comm_sipm_read(struct inno_cam_mod *priv, u32 port, u8 id, u32 reg, u32 *val)
{
	int ret = INNO_RET_OK;
	u32 src = 0U;

	INNO_DBG("\n");

	ret = inno_comm_wait_idle(priv);
	INNO_RET_ERR_HANDLE(ret);

	src = INNO_EV2M_OOM1_DMA_SIZE_2_BYTE;
	ret = inno_iic_wr_reg(priv, DMA_SIZE, src, INNO_IIC_VAL_32BIT);
	INNO_RET_ERR_HANDLE(ret);

	src = DMA_SIP_SIPM(port)
	    | DMA_SIP_DATA_16_BIT
	    | DMA_SIP_ADDR_16_BIT
	    | DMA_SIP_ID(id)
	    | reg;
	ret = inno_iic_wr_reg(priv, DMA_SRC, src, INNO_IIC_VAL_32BIT);
	INNO_RET_ERR_HANDLE(ret);

	/*
	 * Use the DMA_DST register as both the destination address, and
	 * the scratch pad to store the read value.
	 */
	src = DMA_DST;
	ret = inno_iic_wr_reg(priv, DMA_DST, src, INNO_IIC_VAL_32BIT);
	INNO_RET_ERR_HANDLE(ret);

	src = DMA_CTRL_SCH_NORMAL
	    | DMA_CTRL_DST_REG
	    | DMA_CTRL_SRC_SIP
	    | DMA_CTRL_MODE_COPY;
	ret = inno_iic_wr_reg(priv, DMA_CTRL, src, INNO_IIC_VAL_16BIT);
	INNO_RET_ERR_HANDLE(ret);

	ret = inno_comm_wait_idle(priv);
	INNO_RET_ERR_HANDLE(ret);

	ret = inno_iic_rd_reg(priv, DMA_DST, val, INNO_IIC_VAL_16BIT);
	INNO_RET_ERR_HANDLE(ret);

	return ret;
}

int inno_comm_get_sensor_id(struct inno_cam_mod *priv)
{
	struct device *dev = priv->dev;
	int ret = INNO_RET_OK;
	u32 sensor_id = 0U;

	mutex_lock(&priv->inno_cam_mod_lock);

	if (priv->info->sensor_addr != INNO_NULL_SIP_ID) {
		ret = inno_comm_sipm_read(priv, DMA_SIP_SIPM_0, priv->info->sensor_addr, priv->info->sensor_reg, &sensor_id);
	}
	INNO_RET_ERR_HANDLE(ret);

	/* Make sure that host connect with Innodisk Camera Module */
	if (priv->info->sensor_id != sensor_id) {
		INNO_DEV_ERR(dev, "Invalid Sensor ID: 0x%04X\n", sensor_id);
		ret = -ENODEV;
	} else {
		INNO_DEV_ALERT(dev, "Sensor ID: 0x%04X\n", sensor_id);
	}

	mutex_unlock(&priv->inno_cam_mod_lock);

	return ret;
}

int inno_comm_get_isp_id(struct inno_cam_mod *priv)
{
	struct device *dev = priv->dev;
	int ret = INNO_RET_OK;
	u32 isp_id = 0U;

	mutex_lock(&priv->inno_cam_mod_lock);
	/* Probe Sensor Module ID */
	if (!strcmp(priv->info->dev_name, INNO_EV2M_OOM1_DEVICE_NAME) ||
		!strcmp(priv->info->dev_name, INNO_EVDM_OOM1_DEVICE_NAME) ||
		!strcmp(priv->info->dev_name, INNO_EV2M_OOM2_DEVICE_NAME) ||
		!strcmp(priv->info->dev_name, INNO_EV8M_OOM1_DEVICE_NAME)) {
		ret = inno_iic_rd_reg(priv, CHIP_VERSION_REG, &isp_id, INNO_IIC_VAL_16BIT);
	} else if (!strcmp(priv->info->dev_name, INNO_EV2M_GOM1_DEVICE_NAME)) {
		ret = inno_iic_rd_reg(priv, GENESYS_CHIP_ID, &isp_id, INNO_IIC_VAL_16BIT);
	}
	INNO_RET_ERR_HANDLE(ret);

	/* Make sure that host connect with Innodisk Camera Module */
	if (priv->info->isp_id != isp_id) {
		INNO_DEV_ERR(dev, "Invalid ISP ID: 0x%04X\n", isp_id);
		ret = -ENODEV;
	} else {
		INNO_DEV_ALERT(dev, "ISP ID: 0x%04X\n", isp_id);
	}
	mutex_unlock(&priv->inno_cam_mod_lock);

	return ret;
}

int inno_comm_get_mipi_info(struct inno_cam_mod *priv)
{
	struct device *dev = priv->dev;
	int ret = INNO_RET_OK;
	u32 reg_clk = 0U, reg_ctrl = 0U;

	mutex_lock(&priv->inno_cam_mod_lock);
	ret = inno_iic_rd_reg(priv, HINF_MIPI_FREQ_TGT, &reg_clk, INNO_IIC_VAL_16BIT);
	INNO_RET_ERR_HANDLE(ret);

	ret = inno_iic_rd_reg(priv, PREVIEW_HINF_CTRL, &reg_ctrl, INNO_IIC_VAL_16BIT);
	INNO_RET_ERR_HANDLE(ret);

	/* Can NOT get default value */
	if (reg_clk == HINF_MIPI_FREQ_TGT_DEFAULT)
		INNO_DEV_ALERT(dev, "MIPI %sContinuous Clock Default Speed %d Data Lane\n",
			((reg_ctrl & PREVIEW_HINF_CTRL_CONT_CLK_MASK) == PREVIEW_HINF_CTRL_EN_CONT_CLK) ? "" : "Un-",
			(reg_ctrl & PREVIEW_HINF_CTRL_MIPI_MASK));
	else
		INNO_DEV_ALERT(dev, "MIPI %sContinuous Clock %dMHz %d Data Lane\n",
			((reg_ctrl & PREVIEW_HINF_CTRL_CONT_CLK_MASK) == PREVIEW_HINF_CTRL_EN_CONT_CLK) ? "" : "Un-",
			reg_clk,
			(reg_ctrl & PREVIEW_HINF_CTRL_MIPI_MASK));
	mutex_unlock(&priv->inno_cam_mod_lock);

	return ret;
}

/* Nvidia function pointer */
int inno_comm_open(struct v4l2_subdev *sd, struct v4l2_subdev_fh *fh)
{
	struct inno_cam_mod *priv = inno_comm_sd_to_cam(sd);
	int ret = INNO_RET_OK;

	INNO_DBG("\n");

	(void)priv;

	return ret;
}
/* Nvidia function pointer end */

/* NXP function pointer */
int inno_comm_g_volatile_ctrl(struct v4l2_ctrl *ctrl)
{
	struct v4l2_subdev *sd = inno_comm_ctrl_to_sd(ctrl);
	struct inno_cam_mod *priv = inno_comm_sd_to_cam(sd);
	int ret = INNO_RET_OK;

	INNO_DBG("\n");

	(void)priv;

#if 0
	switch (ctrl->id) {
	case V4L2_CID_AUTOGAIN:
		val = inno_comm_get_gain(sensor);
		if (val < 0)
			return val;
		sensor->ctrls.gain->val = val;
		break;
	case V4L2_CID_EXPOSURE_AUTO:
		val = inno_comm_get_exposure(sensor);
		if (val < 0)
			return val;
		sensor->ctrls.exposure->val = val;
		break;
	}
#endif

	return ret;
}

int inno_comm_try_ctrl(struct v4l2_ctrl *ctrl)
{
	struct v4l2_subdev *sd = inno_comm_ctrl_to_sd(ctrl);
	struct inno_cam_mod *priv = inno_comm_sd_to_cam(sd);
	int ret = INNO_RET_OK;

	INNO_DBG("\n");

	(void)priv;

	return ret;
}

int inno_comm_set_ctrl_exposure(struct inno_cam_mod *priv, s32 mode)
{
	int ret = INNO_RET_OK;
	u32 val = 0U;

	INNO_DEV_ALERT(priv->dev, "\n");

	ret = inno_iic_rd_reg(priv, AE_CTRL, &val, INNO_IIC_VAL_16BIT);
	if (ret)
		return ret;

	val &= ~AE_CTRL_MODE_MASK;
	/* Auto Exposure */
	if (mode == 0) {
		val |= 0x0C;
	} else {
		val |= 0x00;
	}

	return inno_iic_wr_reg(priv, AE_CTRL, val, INNO_IIC_VAL_16BIT);
}

int inno_comm_set_ctrl_white_balance(struct inno_cam_mod *priv, s32 ctrl_val)
{
	int ret = INNO_RET_OK;
	u32 val = 0U;

	INNO_DEV_ALERT(priv->dev, "\n");

	ret = inno_iic_rd_reg(priv, AWB_CTRL, &val, INNO_IIC_VAL_16BIT);
	if (ret)
		return ret;

	val &= ~AWB_CTRL_MODE_MASK;

	/*
	 * 0x00 = AWB off, white point is defined in AWB_MANUAL_QX and AWB_MANUAL_QY registers
	 * 0x01 = Horizon illumination selected
	 * 0x02 = A illumination selected
	 * 0x03 = CWF illumination selected
	 * 0x04 = D50 illumination selected
	 * 0x05 = D65 illumination selected
	 * 0x06 = D75 illumination selected
	 * 0x07 = white point is defined in AWB_MANUAL_TEMP register
	 * 0x08 = Measure white point (Illumination is automatically selected, white point is updated in AWB_QX and AWB_QY registers and
	 * AWB mode is automatically set to 0x0 after measure)
	 * Note: dbg_stats is used to measure white point
	 * 0x0F = Determine white-point automatically (AWB Enabled)
	 */
	/* Auto AWB */
	if (ctrl_val == 0) {
		val |= 0x0F;
	} else {
		val |= 0x07;
	}
	
	return inno_iic_wr_reg(priv, AWB_CTRL, val, INNO_IIC_VAL_16BIT);
}

int inno_comm_set_ctrl_white_balance_temp(struct inno_cam_mod *priv, s32 ctrl_val)
{
	int ret = INNO_RET_OK;

	INNO_DEV_ALERT(priv->dev, "%d\n", ctrl_val);

	ret = inno_iic_wr_reg(priv, AWB_TEMP_CTRL, (u32) ctrl_val, INNO_IIC_VAL_16BIT);

	return ret;
}

int inno_comm_set_gain(struct inno_cam_mod *priv, s32 ctrl_val)
{
	int ret = INNO_RET_OK;

	INNO_DEV_ALERT(priv->dev, "%d\n", ctrl_val);

	ret = inno_iic_wr_reg(priv, AE_MANUAL_GAIN, (u32) ctrl_val, INNO_IIC_VAL_16BIT);

	return ret;
}

int inno_comm_set_exposure(struct inno_cam_mod *priv, s32 ctrl_val)
{
	int ret = INNO_RET_OK;

	INNO_DEV_ALERT(priv->dev, "%d\n", ctrl_val);

	ret = inno_iic_wr_reg(priv, AE_MANUAL_EXPOSURE, (u32) ctrl_val, INNO_IIC_VAL_32BIT);

	return ret;
}

int inno_comm_set_scene_mode(struct inno_cam_mod *priv, s32 ctrl_val)
{
	int ret = INNO_RET_OK;

	INNO_DEV_ALERT(priv->dev, "%d\n", ctrl_val);

	if (!strcmp(priv->info->dev_name, INNO_EV2M_GOM1_DEVICE_NAME) &&
		ctrl_val < GENESYS_DISPLAY_MODE_NUM) {
		ret = inno_iic_wr_reg(priv, GENESYS_DISPLAY_MODE, (u32) ctrl_val, INNO_IIC_VAL_08BIT);
		INNO_RET_ERR_HANDLE(ret);
	} else {
		INNO_DEV_ERR(priv->dev, "Error inputting mode (%d)\n", ret);
		ret = -ESRCH;
	}

	return ret;
}

int inno_comm_s_ctrl(struct v4l2_ctrl *ctrl)
{
	struct v4l2_subdev *sd = inno_comm_ctrl_to_sd(ctrl);
	struct inno_cam_mod *priv = inno_comm_sd_to_cam(sd);
	int ret = INNO_RET_OK;

	INNO_DEV_ALERT(priv->dev, "(%04X, %d)\n", ctrl->id, ctrl->val);

	switch (ctrl->id) {
		case V4L2_CID_EXPOSURE_AUTO:
			ret = inno_comm_set_ctrl_exposure(priv, ctrl->val);
			break;
		case V4L2_CID_AUTO_WHITE_BALANCE:
			ret = inno_comm_set_ctrl_white_balance(priv, ctrl->val);
			break;
		case V4L2_CID_WHITE_BALANCE_TEMPERATURE:
			ret = inno_comm_set_ctrl_white_balance_temp(priv, ctrl->val);
			break;
		case V4L2_CID_GAIN:
			ret = inno_comm_set_gain(priv, ctrl->val);
			break;
		case V4L2_CID_EXPOSURE:
			ret = inno_comm_set_exposure(priv, ctrl->val);
			break;
		case V4L2_CID_SCENE_MODE:
			ret = inno_comm_set_scene_mode(priv, ctrl->val);
			break;
	}

	return ret;
}

/*
 * inno_comm_s_power - V4L2 sensor interface handler for VIDIOC_S_POWER ioctl
 * @s: pointer to standard V4L2 device structure
 * @on: indicates power mode (on or off)
 *
 * Turns the power on or off, depending on the value of on and returns the
 * appropriate error code.
 */
int inno_comm_s_power(struct v4l2_subdev *sd, int on)
{
	struct inno_cam_mod *priv = inno_comm_sd_to_cam(sd);
	int ret = INNO_RET_OK;

	INNO_DEV_ALERT(priv->dev, "%s\n", (on == SWITCH_ON) ? "ON" : "OFF");

	inno_comm_power(priv, on);

	return ret;
}

int inno_comm_g_register(struct v4l2_subdev *sd, struct v4l2_dbg_register *reg)
{
	struct inno_cam_mod *priv = inno_comm_sd_to_cam(sd);
	int ret = INNO_RET_OK;
	unsigned int aux;

	if (!strcmp(priv->info->dev_name, INNO_EV2M_OOM1_DEVICE_NAME) ||
		!strcmp(priv->info->dev_name, INNO_EVDM_OOM1_DEVICE_NAME) ||
		!strcmp(priv->info->dev_name, INNO_EV2M_OOM2_DEVICE_NAME) ||
		!strcmp(priv->info->dev_name, INNO_EV8M_OOM1_DEVICE_NAME)) {
		inno_iic_rd_reg(priv, reg->reg, &aux, INNO_IIC_VAL_16BIT);
	} else if (!strcmp(priv->info->dev_name, INNO_EV2M_GOM1_DEVICE_NAME)) {
		inno_iic_rd_reg(priv, reg->reg, &aux, INNO_IIC_VAL_08BIT);
	}
	reg->val = aux;

	return ret;
}

int inno_comm_s_register(struct v4l2_subdev *sd, const struct v4l2_dbg_register *reg)
{
	struct inno_cam_mod *priv = inno_comm_sd_to_cam(sd);
	int ret = INNO_RET_OK;

	if (!strcmp(priv->info->dev_name, INNO_EV2M_OOM1_DEVICE_NAME) ||
		!strcmp(priv->info->dev_name, INNO_EVDM_OOM1_DEVICE_NAME) ||
		!strcmp(priv->info->dev_name, INNO_EV2M_OOM2_DEVICE_NAME) ||
		!strcmp(priv->info->dev_name, INNO_EV8M_OOM1_DEVICE_NAME)) {
		ret = inno_iic_wr_reg(priv, reg->reg, reg->val, INNO_IIC_VAL_16BIT);
	} else if (!strcmp(priv->info->dev_name, INNO_EV2M_GOM1_DEVICE_NAME)) {
		ret = inno_iic_wr_reg(priv, reg->reg, reg->val, INNO_IIC_VAL_08BIT);
	}

	return ret;
}

int inno_comm_g_frame_interval(struct v4l2_subdev *sd, struct v4l2_subdev_frame_interval *fi)
{
	struct inno_cam_mod *priv = inno_comm_sd_to_cam(sd);
	int ret = INNO_RET_OK;

	INNO_DBG("\n");

	mutex_lock(&priv->inno_cam_mod_lock);
	fi->interval = priv->frame_interval;
	mutex_unlock(&priv->inno_cam_mod_lock);

	return ret;
}

int inno_comm_s_frame_interval(struct v4l2_subdev *sd, struct v4l2_subdev_frame_interval *fi)
{
	struct inno_cam_mod *priv = inno_comm_sd_to_cam(sd);
	int ret = INNO_RET_OK;

	INNO_DBG("\n");

	(void)priv;

	return ret;
}

int inno_comm_s_stream(struct v4l2_subdev *sd, int enable)
{
	struct inno_cam_mod *priv = inno_comm_sd_to_cam(sd);
	int ret = INNO_RET_OK;

	INNO_DEV_ALERT(priv->dev, "%s\n", (enable) ? "Enable" : "Disable");

	mutex_lock(&priv->inno_cam_mod_lock);
	if (!strcmp(priv->info->dev_name, INNO_EV2M_OOM1_DEVICE_NAME) ||
		!strcmp(priv->info->dev_name, INNO_EVDM_OOM1_DEVICE_NAME) ||
		!strcmp(priv->info->dev_name, INNO_EV2M_OOM2_DEVICE_NAME) ||
		!strcmp(priv->info->dev_name, INNO_EV8M_OOM1_DEVICE_NAME)) {
		if (enable) {
			ret = inno_comm_stall(priv, false);
			msleep_range(INNO_COMM_WAIT_MS_START);;
			msleep_range(INNO_COMM_WAIT_MS_STREAM);
		} else {
			ret = inno_comm_stall(priv, true);
			msleep_range(INNO_COMM_WAIT_MS_STOP);
			msleep_range(INNO_COMM_WAIT_MS_STREAM);
		}
		if (ret)
			INNO_DEV_ERR(priv->dev, "Error stalling\n");
	}
	mutex_unlock(&priv->inno_cam_mod_lock);

	return ret;
}

int inno_comm_enum_mbus_code(struct v4l2_subdev *sd, struct v4l2_subdev_state *sd_state, struct v4l2_subdev_mbus_code_enum *code)
{
	struct inno_cam_mod *priv = inno_comm_sd_to_cam(sd);
	int ret = INNO_RET_OK;

	INNO_DBG("\n");

	(void)priv;

	if (code->pad != 0)
		return -EINVAL;

	if (code->index)
		return -EINVAL;

	code->code = INNO_MBUS_CODE;

	return ret;
}

/*
 * inno_comm_enum_frame_size - V4L2 sensor interface handler for
 *			   VIDIOC_ENUM_FRAMESIZES ioctl
 * @s: pointer to standard V4L2 device structure
 * @fsize: standard V4L2 VIDIOC_ENUM_FRAMESIZES ioctl structure
 *
 * Return 0 if successful, otherwise -EINVAL.
 */
int inno_comm_enum_frame_size(struct v4l2_subdev *sd, struct v4l2_subdev_state *sd_state, struct v4l2_subdev_frame_size_enum *fse)
{
	struct inno_cam_mod *priv = inno_comm_sd_to_cam(sd);
	int ret = INNO_RET_OK;

	INNO_DBG("\n");

	if (fse->pad != 0)
		return -EINVAL;

	if (fse->index >= priv->info->prvw_modes_num)
		return -EINVAL;

	mutex_lock(&priv->inno_cam_mod_lock);
	fse->min_width	= priv->info->prvw_modes[fse->index].width;
	fse->min_height	= priv->info->prvw_modes[fse->index].height;
	fse->max_width	= priv->info->prvw_modes[fse->index].width;
	fse->max_height	= priv->info->prvw_modes[fse->index].height;
	mutex_unlock(&priv->inno_cam_mod_lock);

	INNO_DBG("Frame Size: %dx%d@%d(%d)\n",
		priv->info->prvw_modes[fse->index].width,
		priv->info->prvw_modes[fse->index].height,
		priv->info->prvw_modes[fse->index].fps,
		priv->info->prvw_modes[fse->index].mode);

	return ret;
}

int inno_comm_enum_frame_interval(struct v4l2_subdev *sd, struct v4l2_subdev_state *sd_state, struct v4l2_subdev_frame_interval_enum *fie)
{
	struct inno_cam_mod *priv = inno_comm_sd_to_cam(sd);
	int ret = INNO_RET_OK;

	INNO_DEV_ALERT(priv->dev, "Current Frame: %dx%d@%d(%d)\n",
		priv->info->cur_mode->width,
		priv->info->cur_mode->height,
		priv->info->cur_mode->fps,
		priv->info->cur_mode->mode);

	if (fie->index != 0)
		return -EINVAL;

	fie->code					= INNO_MBUS_CODE;
	fie->width					= priv->info->cur_mode->width;
	fie->height					= priv->info->cur_mode->height;
	fie->interval.denominator	= priv->info->cur_mode->fps;
	fie->interval.numerator		= 1;

	return ret;
}

int inno_comm_get_fmt(struct v4l2_subdev *sd, struct v4l2_subdev_state *sd_state, struct v4l2_subdev_format *format)
{
	struct inno_cam_mod *priv = inno_comm_sd_to_cam(sd);
	struct v4l2_mbus_framefmt *fmt = &priv->fmt;
	int ret = INNO_RET_OK;

	INNO_DEV_ALERT(priv->dev, "Get Format: %dx%d\n", fmt->width, fmt->height);

	if (format->pad != 0)
		return -EINVAL;

	mutex_lock(&priv->inno_cam_mod_lock);
	format->format = *fmt;
	mutex_unlock(&priv->inno_cam_mod_lock);

	return ret;
}

const struct inno_res_info *inno_comm_find_mode(struct inno_cam_mod *priv, u32 width, u32 height)
{
	struct device *dev = &priv->i2c_client->dev;
	const struct inno_res_info *mode;

	mode = v4l2_find_nearest_size(priv->info->prvw_modes,
								  priv->info->prvw_modes_num,
								  width, height,
								  width, height);
	if (!mode) {
		INNO_DEV_ERR(dev, "Error finding format: %dx%d@%d(%d)\n",
			mode->width,
			mode->height,
			mode->fps,
			mode->mode);
		return NULL;
	} else {
		INNO_DBG("Find format: %dx%d@%d(%d)\n",
			mode->width,
			mode->height,
			mode->fps,
			mode->mode);
	}

	return mode;
}

int inno_comm_set_fmt(struct v4l2_subdev *sd, struct v4l2_subdev_state *sd_state, struct v4l2_subdev_format *format)
{
	struct inno_cam_mod *priv = inno_comm_sd_to_cam(sd);
	struct v4l2_mbus_framefmt *fmt = &priv->fmt;
	struct v4l2_mbus_framefmt *mbus_fmt = &format->format;
	const struct inno_res_info *mode;
	int ret = INNO_RET_OK;
	int i = 0;

	INNO_DEV_ALERT(priv->dev, "Set Format: %dx%d\n", mbus_fmt->width, mbus_fmt->height);

	if (format->pad != 0)
		return -EINVAL;

	mutex_lock(&priv->inno_cam_mod_lock);
	mode = inno_comm_find_mode(priv, mbus_fmt->width, mbus_fmt->height);
	if (!mode)
	      return -EINVAL;

	priv->info->mode_change = false;
	if (mode != priv->info->cur_mode) {
		priv->info->cur_mode = mode;
		priv->info->mode_change = true;
	}

	memcpy(fmt, mbus_fmt, sizeof(struct v4l2_mbus_framefmt));
	fmt->width	= mode->width;
	fmt->height	= mode->height;

	if (priv->info->mode_change) {
		for (i = 0; i < priv->info->prvw_modes_num; i++) {
			INNO_ALERT("Mode#%d: %dx%d@%d(%d)\n",
				i,
				priv->info->prvw_modes[i].width,
				priv->info->prvw_modes[i].height,
				priv->info->prvw_modes[i].fps,
				priv->info->prvw_modes[i].mode);
			if ((fmt->width == priv->info->prvw_modes[i].width) &&
				(fmt->height == priv->info->prvw_modes[i].height)) {
				ret = inno_comm_set_format(priv, i);
				if (ret) {
					INNO_DEV_ERR(priv->dev, "Error setting formatn");
					ret = -EIO;
				}

				break;
			}
		}
	}
	mutex_unlock(&priv->inno_cam_mod_lock);

	return ret;
}

int inno_comm_link_setup(struct media_entity *entity, const struct media_pad *local, const struct media_pad *remote, u32 flags)
{
	int ret = INNO_RET_OK;

	INNO_DBG("\n");

	return ret;
}
/* NXP function pointer end */

int inno_comm_get_regulators(struct inno_cam_mod *priv)
{
	int i = 0;

	for (i = 0; i < INNO_NUM_CONSUMERS; i++)
		priv->supplies[i].supply = inno_comm_supply_name[i];

	return devm_regulator_bulk_get(&priv->i2c_client->dev, INNO_NUM_CONSUMERS, priv->supplies);
}

int inno_comm_init_controls(struct inno_cam_mod *priv, const struct v4l2_ctrl_ops *ctrl_ops)
{
	struct inno_evxm_yymz_ctrl *ctrls = &priv->ctrls;
	struct v4l2_ctrl_handler *hdl = &ctrls->handler;
	int ret = INNO_RET_OK;

	INNO_DBG("\n");

	v4l2_ctrl_handler_init(hdl, INNO_CTRL_HDL_NUM);

	/* Use our own mutex for the ctrl lock */
	hdl->lock			= &priv->inno_cam_mod_lock;

	/* Auto or Manual White Balance */
	ctrls->auto_wb		= v4l2_ctrl_new_std(hdl, ctrl_ops,
											V4L2_CID_AUTO_WHITE_BALANCE, 0, 1, 1, 0);
	ctrls->wb_temp		= v4l2_ctrl_new_std(hdl, ctrl_ops,
											V4L2_CID_WHITE_BALANCE_TEMPERATURE, 0, 65535, 1, 5000);

	/* Auto or Manual Exposure */
	ctrls->auto_exp 	= v4l2_ctrl_new_std_menu(hdl, ctrl_ops,
												 V4L2_CID_EXPOSURE_AUTO, V4L2_EXPOSURE_MANUAL, 0,
												 V4L2_EXPOSURE_AUTO);
	ctrls->gain			= v4l2_ctrl_new_std(hdl, ctrl_ops,
											V4L2_CID_GAIN, 0, 65535, 1, 0x100);
	ctrls->exposure		= v4l2_ctrl_new_std(hdl, ctrl_ops,
											V4L2_CID_EXPOSURE, 0, 65535, 1, 10000);

	/* ISO Sensitivity */
	ctrls->scene_mode	= v4l2_ctrl_new_std_menu(hdl, ctrl_ops,
												 V4L2_CID_SCENE_MODE, V4L2_SCENE_MODE_TEXT, ~0x3fff,
												 V4L2_SCENE_MODE_NONE);

	if (hdl->error) {
		ret = hdl->error;
		goto exit;
	}
	
#if defined(_INNO_FOR_NVIDIA_)
	priv->subdev->ctrl_handler = hdl;
#elif defined(_INNO_FOR_NXP_)
	priv->subdev.ctrl_handler = hdl;
#endif

	return ret;

exit:
	INNO_DEV_ERR(priv->dev, "Error initializing controls (%d)\n", ret);

	return ret;
}

int inno_comm_request_firmware(struct inno_cam_mod *priv)
{
	struct device *dev = priv->dev;
	const struct inno_firmware *inno_fw_hdr;
	unsigned int fw_size;
	int ret = INNO_RET_OK;

	INNO_DEV_ALERT(dev, "FW is %s\n", priv->info->fw_name);

	ret = request_firmware(&priv->info->inno_fw_src, priv->info->fw_name, dev);
	if (ret) {
		INNO_DEV_ERR(dev, "Request FW failed %d\n", ret);
	}

	inno_fw_hdr = (const struct inno_firmware *)priv->info->inno_fw_src->data;
	fw_size = priv->info->inno_fw_src->size - sizeof(*inno_fw_hdr);
	INNO_DBG("Source size: %ld, HDR size: %ld, BIN size: %d\n", priv->info->inno_fw_src->size, sizeof(*inno_fw_hdr), fw_size);
	if (inno_fw_hdr->pll_init_size > fw_size) {
		INNO_DEV_ERR(dev, "PLL init size too large\n");
		return -EINVAL;
	}

	return INNO_RET_OK;
}

int inno_comm_reg_polling(struct inno_cam_mod *priv, u16 addr, u16 val, u16 mask, int poll_interval_ms, int retries, bool expected)
{
	struct device *dev = priv->dev;
	int ret = INNO_RET_OK;
	u32 reg_val = 0U;

	INNO_DBG("\n");

	if (retries <= 0) {
		INNO_DEV_ERR(dev, "Retries is not a positive value\n");
		return -EINVAL;
	}

	while (retries > 0) {
		ret = inno_iic_rd_reg(priv, addr, &reg_val, INNO_IIC_VAL_16BIT);
		if (ret)
			return ret;

		if (expected) {
			if ((reg_val & mask) == val)
				break;
		} else {
			if ((reg_val & mask) != val)
				break;
		}

		msleep(poll_interval_ms);
		retries--;
	}

	if (retries == 0) {
		INNO_DEV_ERR(dev, "Polling NOT match read=0x%x, expected=0x%x\n", (reg_val & mask), val);
		return -EINVAL;
	}

	return INNO_RET_OK;
}

int inno_comm_write_fw_window(struct inno_cam_mod *priv, u16 *win_pos, const u8 *buf, u32 len)
{
	struct device *dev = priv->dev;
	int ret = INNO_RET_OK;
	u32 pos = 0U, sub_len = 0U;

	INNO_DBG("\n");

	for (pos = 0; pos < len; pos += sub_len) {
		/* Checking remaining addresses in window */
		if (len - pos < INNO_FW_WINDOW_SIZE - *win_pos)
			sub_len = len - pos;
		else
			sub_len = INNO_FW_WINDOW_SIZE - *win_pos;

		/* Limiting max addresses to use per write to 0x0FF0,
		 * more addresses in one write will fail */
		if (sub_len > INNO_FW_BUFFER)
			sub_len = INNO_FW_BUFFER;

		ret = regmap_raw_write(priv->regmap_16, *win_pos + INNO_FW_WINDOW_OFFSET, buf + pos, sub_len);
		if (ret) {
			INNO_DEV_ERR(dev, "Error writing %d bytes to address 0x%x\n", sub_len, pos);
			return ret;
		}

		*win_pos += sub_len;
		if (*win_pos >= INNO_FW_WINDOW_SIZE)
			*win_pos = 0;
	}

	return INNO_RET_OK;
}

int inno_comm_firmware_handler(struct inno_cam_mod *priv, const struct inno_firmware *inno_fw)
{
	struct device *dev = priv->dev;
	const u8 *inno_fw_data;
	int ret = INNO_RET_OK;
	u16 win_pos = 0;
#if 0 // Can NOT get BOOTDATA_CHECKSUM on EVB
	u16 reg_val = 0U;
#endif

	INNO_DBG("\n");

	inno_fw_data = (u8 *)&inno_fw[1];

	/* Clear CRC register */
	ret = inno_iic_wr_reg(priv, SIPS_CRC, 0xFFFF, INNO_IIC_VAL_16BIT);
	if (ret) {
		INNO_DEV_ERR(dev, "Error clearing CRC\n");
		return ret;
	}

	/* Load FW data for PLL init stage. */
	INNO_DBG("Magic: 0x%08X, Version: 0x%08X, PLL Init: 0x%04X\n", inno_fw->magic, inno_fw->version, inno_fw->pll_init_size);
	ret = inno_comm_write_fw_window(priv, &win_pos, inno_fw_data, inno_fw->pll_init_size);
	if (ret) {
		INNO_DEV_ERR(dev, "Error initializing FW\n");
		return ret;
	}

	/* Write 0x0002 to bootdata_stage register to apply basic_init_hp
	 * settings and enable PLL.
	 */
	ret = inno_iic_wr_reg(priv, BOOTDATA_STAGE, 0x0002, INNO_IIC_VAL_16BIT);
	if (ret) {
		INNO_DEV_ERR(dev, "Error writing BOOTDATA_STAGE\n");
		return ret;
	}

	/* Wait 1ms for PLL to lock. */
	usleep_range(1000, 2000);

	/* Load the rest of bootdata content. */
	ret = inno_comm_write_fw_window(priv,
									&win_pos,
									inno_fw_data + inno_fw->pll_init_size,
									priv->info->inno_fw_src->size - inno_fw->pll_init_size - sizeof(*inno_fw));
	if (ret) {
		INNO_DEV_ERR(dev, "Error loading remaining boot data\n");
		return ret;
	}

#if 0 // Can NOT get BOOTDATA_CHECKSUM on EVB
	msleep(40);
	/* On AP1302 there is a bug in HW implementation that can occasionally cause SIPS_CRC to be calculated incorrectly.
	 * Please also see BOOTDATA_CHECKSUM register.
	 */
	ret = inno_comm_reg_polling(priv,
	                            BOOTDATA_CHECKSUM,
	                            NO_EXPECTED_BOOTDATA_CHECKSUM,
	                            0xFFFF,
	                            POLL_REG_DELAY,
	                            POLL_REG_COUNT,
	                            false);
	ret = inno_iic_rd_reg(priv, BOOTDATA_CHECKSUM, &reg_val);
	if (ret) {
		return ret;
	}

	if (reg_val != fw_hdr->crc) {
		INNO_WARN("CRC mismatch, expected 0x%04X, got 0x%04X\n", fw_hdr->crc, reg_val);
	}
#endif

	/* Write 0xFFFF to bootdata_stage register to indicate Innodisk Camera Module that
	 * the whole bootdata content has been loaded.
	 */
	ret = inno_iic_wr_reg(priv, BOOTDATA_STAGE, EXPECTED_BOOTDATA_STAGE, INNO_IIC_VAL_16BIT);
	if (ret) {
		INNO_DEV_ERR(dev, "Error writing BOOTDATA_STAGE\n");
		return ret;
	}

	msleep(400);

	/* Ensure that FW was loaded correctly */
	ret = inno_comm_reg_polling(priv,
	                            BOOTDATA_STAGE,
	                            EXPECTED_BOOTDATA_STAGE,
	                            0xFFFF,
	                            POLL_REG_DELAY,
	                            POLL_REG_COUNT,
	                            true);
	if (ret) {
		INNO_DEV_ERR(dev, "Error processing boot data\n");
		return -EINVAL;
	}

	msleep(1000);

	/* Get Sensor ID after loading firmware */
	ret = inno_comm_get_sensor_id(priv);
	if (ret) {
		INNO_DEV_ERR(dev, "Error getting Sensor ID\n");
		goto exit;
	}

	ret = inno_comm_get_mipi_info(priv);
	if (ret) {
		INNO_DEV_ERR(dev, "Error getting PREVIEW_HINF_CTRL\n");
		goto exit;
	}

	/* Sensor fine integration time */
	ret = inno_comm_get_fine_integ_time(priv, &priv->fine_integ_time);
	if (ret) {
		INNO_DEV_ERR(dev, "Error querying sensor fine_integ_time\n");
		goto exit;
	}

	/* The starts outputting frames right after boot, stop it. */
	ret = inno_comm_stall(priv, true);
	if (ret) {
		INNO_DEV_ERR(dev, "Error stalling\n");
		goto exit;
	}

#if 0
	/* Minimum time to reserve for firmware execution between frames (in microseconds). */
	inno_iic_wr_reg(priv, MIN_FW_BLANK_TIME, 1, INNO_IIC_VAL_16BIT);
#endif

	INNO_DEV_ALERT(dev, "FW loaded successfully\n");

exit:
	return ret;
}

void inno_comm_nowait_firmware(const struct firmware *fw, void *context)
{
	struct inno_cam_mod *priv = context;
	const struct inno_firmware *inno_fw;
	int ret = INNO_RET_OK;

	INNO_DBG("\n");

	if (fw == NULL) {
		INNO_DEV_ERR(priv->dev, "FW not requested\n");
		return;
	}

	inno_fw = (struct inno_firmware *)fw->data;
	ret = inno_comm_firmware_handler(priv, inno_fw);
	release_firmware(fw);
}

int inno_comm_load_firmware(struct inno_cam_mod *priv)
{
	const struct inno_firmware *inno_fw;
	int ret = INNO_RET_OK;

	INNO_DBG("\n");

	if (!priv->info->inno_fw_src) {
		INNO_DEV_ERR(priv->dev, "FW not requested\n");
		return -EINVAL;
	}

	inno_fw = (const struct inno_firmware *)priv->info->inno_fw_src->data;
	ret = inno_comm_firmware_handler(priv, inno_fw);

	return ret;
}

int inno_comm_get_fine_integ_time(struct inno_cam_mod *priv, u16 *fine_time)
{
	int ret = INNO_RET_OK;

	INNO_DBG("\n");

	return ret;
}

int inno_comm_board_setup(struct inno_cam_mod *priv)
{
#if defined(_INNO_FOR_NVIDIA_)
	struct camera_common_data *s_data = priv->s_data;
	struct camera_common_pdata *pdata = s_data->pdata;
#endif
	struct device *dev = priv->dev;
	int ret = INNO_RET_OK;

	INNO_DBG("\n");
	
#if defined(_INNO_FOR_NVIDIA_)
	if (pdata->mclk_name) {
		ret = camera_common_mclk_enable(s_data);
		if (ret) {
			INNO_DEV_ERR(dev, "Error turning on mclk (%d)\n", ret);
			goto exit;
		}
	}
#endif

	/* Request firmware for Innodisk Camera Module */
	ret = inno_comm_request_firmware(priv);
	if (ret) {
		INNO_DEV_ERR(dev, "Error requesting FW (%d)\n", ret);
		goto err_brd_setup;
	}

#if defined(_INNO_FOR_NVIDIA_)
	ret = inno_comm_nv_power_on(s_data);
#elif defined(_INNO_FOR_NXP_)
	/* Prefer ro call inno_comm_s_power() but need to init subdev */
	ret = inno_comm_s_power(&priv->subdev, SWITCH_ON);
#endif
	if (ret) {
		INNO_DEV_ERR(dev, "Error during power on %s (%d)\n", priv->info->dev_name, ret);
		goto err_brd_setup;
	}

	ret = inno_comm_rst(priv);
	if (ret) {
		INNO_DEV_ERR(dev, "Error during reset %s (%d)\n", priv->info->dev_name, ret);
		goto err_brd_setup;
	}

	/* Get ISP ID */
	ret = inno_comm_get_isp_id(priv);
	if (ret) {
		INNO_DEV_ERR(dev, "Error getting %s ISP ID (%d)\n", priv->info->dev_name, ret);
		goto err_brd_setup;
	}

	ret = inno_comm_rst(priv);
	if (ret) {
		INNO_DEV_ERR(dev, "Error resetting %s (%d)\n", priv->info->dev_name, ret);
		goto err_brd_setup;
	}

#if defined(_INNO_FOR_NVIDIA_)
	ret = inno_comm_load_firmware(priv);
	if (ret) {
		INNO_DEV_ERR(dev, "Error loading %s FW (%d)\n", priv->info->dev_name, ret);
		goto err_brd_setup;
	}
#elif defined(_INNO_FOR_NXP_)
	/* Request firmware for Innodisk Camera Module */
	ret = request_firmware_nowait(THIS_MODULE,
								  FW_ACTION_UEVENT,
								  priv->info->fw_name,
								  priv->dev,
								  GFP_KERNEL,
								  priv,
								  inno_comm_nowait_firmware);
	if (ret) {
		INNO_DEV_ERR(priv->dev, "Error requesting firmware (%d)\n", ret);
		goto err_brd_setup;
	}
#endif

err_brd_setup:
#if defined(_INNO_FOR_NVIDIA_)
	inno_comm_nv_power_off(s_data);

	if (pdata->mclk_name)
		camera_common_mclk_disable(s_data);
#endif

exit:
	return ret;
}

#if defined(_INNO_FOR_NVIDIA_)
int inno_comm_probe(struct i2c_client *client,
					struct inno_cam_mod *priv,
					struct tegracam_device *tc_dev,
					struct camera_common_sensor_ops *sensor_ops,
					const struct v4l2_subdev_internal_ops *v4l2sd_internal_ops,
					struct tegracam_ctrl_ops *tcctrl_ops)
#elif defined(_INNO_FOR_NXP_)
int inno_comm_probe(struct i2c_client *client,
					struct inno_cam_mod *priv,
					struct v4l2_mbus_framefmt *fmt,
					const struct media_entity_operations *media_ops,
					struct v4l2_subdev_ops *subdev_ops,
					const struct v4l2_ctrl_ops *ctrl_ops)
#endif
{
#if defined(_INNO_FOR_NXP_)
	struct v4l2_subdev *sd = &priv->subdev;
#endif
	int ret = INNO_RET_OK;
	int ret_line = 0;

#if defined(_INNO_FOR_NVIDIA_)
	priv->i2c_client = tc_dev->client = client;
#elif defined(_INNO_FOR_NXP_)
	priv->i2c_client = client;
#endif
	priv->dev = &client->dev;

	INNO_DEV_ALERT(priv->dev, "Detecting @ 0x%02X\n", client->addr);

	mutex_init(&priv->inno_cam_mod_lock);

#if defined(_INNO_FOR_NXP_)
	/* Request GPIO */
	priv->reset = devm_gpiod_get_optional(priv->dev, "reset", GPIOD_OUT_HIGH);
	if (IS_ERR(priv->reset)) {
		ret = PTR_ERR(priv->reset);
		INNO_DEV_ERR(priv->dev, "Error getting reset (%d)\n", ret);
		ret = -EIO;
		ret_line = __LINE__;
		goto exit;
	}

	priv->isp_en = devm_gpiod_get_optional(priv->dev, "isp_en", GPIOD_OUT_HIGH);
	if (IS_ERR(priv->isp_en)) {
		ret = PTR_ERR(priv->isp_en);
		INNO_DEV_ERR(priv->dev, "Error getting isp_en (%d)\n", ret);
		ret = -EIO;
		ret_line = __LINE__;
		goto exit;
	}
#endif

#if INNO_MAX_GMSL
	ret = inno_max_gmsl_init(priv);
	if (ret) {
		INNO_DEV_ERR(priv->dev, "Error probing Maxim GMSL (%d)\n", ret);
		ret_line = __LINE__;
		goto error_max_gmsl_init;
	}
#endif

	priv->regmap_08 = devm_regmap_init_i2c(client, &sensor_regmap_08_config);
	if (IS_ERR(priv->regmap_08)) {
		INNO_DEV_ERR(priv->dev, "Error initializing regmap_08 (%ld)\n", PTR_ERR(priv->regmap_08));
		ret = -ENODEV;
		ret_line = __LINE__;
		goto exit;
	}

	priv->regmap_16 = devm_regmap_init_i2c(client, &sensor_regmap_16_config);
	if (IS_ERR(priv->regmap_16)) {
		INNO_DEV_ERR(priv->dev, "Error initializing regmap_16 (%ld)\n", PTR_ERR(priv->regmap_16));
		ret = -ENODEV;
		ret_line = __LINE__;
		goto exit;
	}

	priv->regmap_32 = devm_regmap_init_i2c(client, &sensor_regmap_32_config);
	if (IS_ERR(priv->regmap_32)) {
		INNO_DEV_ERR(priv->dev, "Error initializing regmap_32 (%ld)\n", PTR_ERR(priv->regmap_32));
		ret = -ENODEV;
		ret_line = __LINE__;
		goto exit;
	}

	ret = inno_comm_get_regulators(priv);
	if (ret) {
		INNO_DEV_ERR(priv->dev, "Error getting regulators (%d)\n", ret);
		ret = -EINVAL;
		ret_line = __LINE__;
		goto exit;
	}

	ret = regulator_bulk_enable(INNO_NUM_CONSUMERS, priv->supplies);
	if (ret) {
		INNO_DEV_ERR(priv->dev, "Error enabling regulators (%d)\n", ret);
		ret = -EINVAL;
		ret_line = __LINE__;
		goto exit;
	}

#if defined(_INNO_FOR_NVIDIA_)
	tc_dev->dev					= priv->dev;
	tc_dev->sensor_ops			= sensor_ops;
	tc_dev->v4l2sd_internal_ops	= v4l2sd_internal_ops;
	tc_dev->tcctrl_ops			= tcctrl_ops;
	strncpy(tc_dev->name, priv->info->dev_name, sizeof(tc_dev->name));
	if (!strcmp(priv->info->dev_name, INNO_EV2M_OOM1_DEVICE_NAME) ||
		!strcmp(priv->info->dev_name, INNO_EVDM_OOM1_DEVICE_NAME) ||
		!strcmp(priv->info->dev_name, INNO_EV2M_OOM2_DEVICE_NAME) ||
		!strcmp(priv->info->dev_name, INNO_EV8M_OOM1_DEVICE_NAME)) {
		tc_dev->dev_regmap_config = &sensor_regmap_16_config;
	} else if (!strcmp(priv->info->dev_name, INNO_EV2M_GOM1_DEVICE_NAME)) {
		tc_dev->dev_regmap_config = &sensor_regmap_08_config;
	}

	ret = tegracam_device_register(tc_dev);
	if (ret) {
		INNO_DEV_ERR(priv->dev, "Error registring camera driver (%d)\n", ret);
		ret = -EINVAL;
		ret_line = __LINE__;
		goto exit;
	} else {
		INNO_DEV_DBG(priv->dev, "Registring camera driver (%d)\n", ret);
	}

	priv->tc_dev = tc_dev;
	priv->s_data = tc_dev->s_data;
	priv->subdev = &tc_dev->s_data->subdev;
	tegracam_set_privdata(tc_dev, (void *)priv);

	if (!strcmp(priv->info->dev_name, INNO_EV2M_OOM1_DEVICE_NAME) ||
		!strcmp(priv->info->dev_name, INNO_EVDM_OOM1_DEVICE_NAME) ||
		!strcmp(priv->info->dev_name, INNO_EV2M_OOM2_DEVICE_NAME) ||
		!strcmp(priv->info->dev_name, INNO_EV8M_OOM1_DEVICE_NAME)) {
		ret = inno_comm_board_setup(priv);
	} else if (!strcmp(priv->info->dev_name, INNO_EV2M_GOM1_DEVICE_NAME)) {
		ret = inno_comm_get_isp_id(priv);
	}
	if (ret) {
		INNO_DEV_ERR(priv->dev, "Board setup failed (%d)\n", ret);
		tegracam_device_unregister(priv->tc_dev);
		ret = -EINVAL;
		ret_line = __LINE__;
		goto exit;
	}

	ret = tegracam_v4l2subdev_register(tc_dev, true);
	if (ret) {
		INNO_DEV_ERR(priv->dev, "Error registring camera subdev (%d)\n", ret);
		tegracam_device_unregister(priv->tc_dev);
		ret = -EINVAL;
		ret_line = __LINE__;
		goto exit;
	} else {
		INNO_DEV_DBG(priv->dev, "Registring camera subdev (%d)\n", ret);
	}
#elif defined(_INNO_FOR_NXP_)
	/* Config auto AF mode by default */
	//inno_comm_set_af_mode(priv, AF_MODE_AUTO);

	/* Format initialization */
	fmt										= &priv->fmt;
	fmt->code								= INNO_MBUS_CODE;
	fmt->colorspace							= V4L2_COLORSPACE_SRGB;
	fmt->ycbcr_enc							= V4L2_MAP_YCBCR_ENC_DEFAULT(fmt->colorspace);
	fmt->quantization						= V4L2_QUANTIZATION_FULL_RANGE;
	fmt->xfer_func							= V4L2_MAP_XFER_FUNC_DEFAULT(fmt->colorspace);
	fmt->width								= priv->info->cur_mode->width;
	fmt->height								= priv->info->cur_mode->height;
	fmt->field								= V4L2_FIELD_NONE;

	/* Default 60FPS */
	priv->frame_interval.denominator		= INNO_FPS_60;
	priv->frame_interval.numerator			= 1;

	if (!strcmp(priv->info->dev_name, INNO_EV2M_OOM1_DEVICE_NAME) ||
		!strcmp(priv->info->dev_name, INNO_EVDM_OOM1_DEVICE_NAME) ||
		!strcmp(priv->info->dev_name, INNO_EV2M_OOM2_DEVICE_NAME) ||
		!strcmp(priv->info->dev_name, INNO_EV8M_OOM1_DEVICE_NAME)) {
		ret = inno_comm_board_setup(priv);
	} else if (!strcmp(priv->info->dev_name, INNO_EV2M_GOM1_DEVICE_NAME)) {
		ret = inno_comm_get_isp_id(priv);
	}
	if (ret) {
		INNO_DEV_ERR(priv->dev, "Board setup failed (%d)\n", ret);
		ret = -EINVAL;
		ret_line = __LINE__;
		goto exit;
	}

	/* Initialize subdev */
	v4l2_i2c_subdev_init(sd, client, subdev_ops);

	/* Initialize control handlers */
	ret = inno_comm_init_controls(priv, ctrl_ops);
	if (ret) {
		INNO_DEV_ERR(priv->dev, "Error initializing controls (%d)\n", ret);
		ret = -EINVAL;
		ret_line = __LINE__;
		goto exit;
	}

	/* Initialize subdev */
	sd->flags								|= V4L2_SUBDEV_FL_HAS_DEVNODE | V4L2_SUBDEV_FL_HAS_EVENTS;
	sd->entity.function						= MEDIA_ENT_F_CAM_SENSOR;
	sd->entity.ops							= media_ops;
	priv->pads[INNO_SENS_PAD_SOURCE].flags	= MEDIA_PAD_FL_SOURCE;

	/* Initialize source pad */
	ret = media_entity_pads_init(&sd->entity, INNO_SENS_PADS_NUM, priv->pads);
	if (ret) {
		INNO_DEV_ERR(priv->dev, "Error initializing media entity (%d)\n", ret);
		ret = -EINVAL;
		ret_line = __LINE__;
		goto error_ctrl_handler_free;
	}

	ret = v4l2_async_register_subdev_sensor(sd);
	if (ret) {
		INNO_DEV_ERR(priv->dev, "Error registring camera subdev (%d)\n", ret);
		ret = -EINVAL;
		ret_line = __LINE__;
		goto error_media_entity_clean;
	} else {
		INNO_DEV_DBG(priv->dev, "Registring camera subdev (%d)\n", ret);
	}
#endif

	INNO_DEV_ALERT(priv->dev, "Detected %s @ 0x%02X\n", priv->info->dev_name, client->addr);

	return ret;

#if defined(_INNO_FOR_NXP_)
error_media_entity_clean:
	media_entity_cleanup(&priv->subdev.entity);

error_ctrl_handler_free:
	v4l2_ctrl_handler_free(priv->subdev.ctrl_handler);
#endif

exit:
	regulator_bulk_disable(INNO_NUM_CONSUMERS, priv->supplies);

	if (!strcmp(priv->info->dev_name, INNO_EV2M_OOM1_DEVICE_NAME) ||
		!strcmp(priv->info->dev_name, INNO_EV2M_OOM2_DEVICE_NAME) ||
		!strcmp(priv->info->dev_name, INNO_EV8M_OOM1_DEVICE_NAME)) {
		inno_iic_io_exp_device_release(priv);
	}

error_max_gmsl_init:
#if INNO_MAX_GMSL
	if (priv->info->is_gmsl) {
		inno_max_gmsl_remove(priv);
	}
#endif

	mutex_destroy(&priv->inno_cam_mod_lock);

	INNO_DEV_ERR(priv->dev, "Error probing @ 0x%02X[%d, %d]\n", client->addr, ret, ret_line);

	return ret;
}

#if defined(_INNO_FOR_NVIDIA_)
int inno_comm_remove(struct i2c_client *client)
#elif defined(_INNO_FOR_NXP_)
void inno_comm_remove(struct i2c_client *client)
#endif
{
	struct v4l2_subdev *sd = i2c_get_clientdata(client);
	struct inno_cam_mod *priv = inno_comm_sd_to_cam(sd);
	int ret = INNO_RET_OK;
    int ret_line = 0;

	if (!client) {
		ret = -EINVAL;
		ret_line = __LINE__;
		goto exit;
	}

	if (!priv) {
		ret = -EINVAL;
		ret_line = __LINE__;
		goto exit;
	}

	INNO_DEV_ALERT(priv->dev, "Removing @ 0x%02X\n", client->addr);

#if defined(_INNO_FOR_NVIDIA_)
	tegracam_v4l2subdev_unregister(priv->tc_dev);
	tegracam_device_unregister(priv->tc_dev);
#elif defined(_INNO_FOR_NXP_)
	v4l2_async_unregister_subdev(&priv->subdev);
	v4l2_ctrl_handler_free(priv->subdev.ctrl_handler);
	media_entity_cleanup(&priv->subdev.entity);
#endif

	regulator_bulk_disable(INNO_NUM_CONSUMERS, priv->supplies);

#if INNO_MAX_GMSL
	if (priv->info->is_gmsl) {
		ret = inno_max_gmsl_remove(priv);
	}
#endif

	ret = inno_comm_rst(priv);
	if (!strcmp(priv->info->dev_name, INNO_EV2M_OOM1_DEVICE_NAME) ||
		!strcmp(priv->info->dev_name, INNO_EV2M_OOM2_DEVICE_NAME) ||
		!strcmp(priv->info->dev_name, INNO_EV8M_OOM1_DEVICE_NAME)) {
		ret = inno_iic_io_exp_device_release(priv);
	}

	mutex_destroy(&priv->inno_cam_mod_lock);

	INNO_DEV_ALERT(priv->dev, "Removed %s @ 0x%02X\n", priv->info->dev_name, client->addr);

#if defined(_INNO_FOR_NVIDIA_)
	return ret;
#endif

exit:
	INNO_DEV_ERR(priv->dev, "Error removing %s @ 0x%02X[%d, %d]\n", priv->info->dev_name, client->addr, ret, ret_line);

#if defined(_INNO_FOR_NVIDIA_)
	return ret;
#endif
}
