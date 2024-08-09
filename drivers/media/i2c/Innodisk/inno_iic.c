/*
 * inno_iic.c - Innodisk I2C Function
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
#include "inno_common_nv.h"
#elif defined(_INNO_FOR_NXP_)
#include "inno_common_nxp.h"
#endif
#include "inno_iic.h"

/* Read registers up to 4 at a time */
static inline int inno_iic_host_read_reg(struct inno_cam_mod *priv, u16 reg, u32 len, u32 *val)
{
    int ret = INNO_RET_OK;
    struct i2c_msg msgs[2];
    u8 *data_be_p;
    __be32 data_be = 0;
    __be16 reg_addr_be = cpu_to_be16(reg);

    if (len > 4 || !len)
        return -EINVAL;

    data_be_p = (uint8_t *)&data_be;
    /* Write register address */
    msgs[0].addr	= priv->i2c_client->addr;
    msgs[0].flags	= 0;
    msgs[0].len		= 2U;
    msgs[0].buf		= (u8 *)&reg_addr_be;

    /* Read data from register */
    msgs[1].addr	= priv->i2c_client->addr;
    msgs[1].flags	= I2C_M_RD;
    msgs[1].len		= len;
    msgs[1].buf		= &data_be_p[4 - len];

    if (len != i2c_transfer(priv->i2c_client->adapter, msgs, ARRAY_SIZE(msgs)))
        ret = -EIO;

    *val = be32_to_cpu(data_be);

    return ret;
}

/* Write registers up to 4 at a time */
static inline int inno_iic_host_write_reg(struct inno_cam_mod *priv, u16 reg, u32 len, u32 val)
{
    int ret = INNO_RET_OK;
    u32 buf_i, val_i;
    u8 buf[6];
    u8 *val_p;
    __be32 val_be;

    if (len > 4)
        return -EINVAL;

    buf[0]	= reg >> 8;
    buf[1]	= reg & 0xFF;
    val_be	= cpu_to_be32(val);
    val_p	= (u8 *)&val_be;
    buf_i	= 2U;
    val_i	= 4 - len;

    while (val_i < 4)
        buf[buf_i++] = val_p[val_i++];

    if (i2c_master_send(priv->i2c_client, buf, len + 2) != len + 2)
        ret = -EIO;

    return ret;
}

inline int inno_ii2_hw_rd_reg(struct inno_cam_mod *priv, u16 reg, u16 *val)
{
    int ret = INNO_RET_OK;
    u32 lval = 0U;

    ret = inno_iic_host_read_reg(priv, reg, INNO_IIC_VAL_16BIT, &lval);
    *val = lval;

    INNO_IIC("%s: 0x%04X = 0x%04X(%d)\n", priv->i2c_client->adapter->name, reg, *val, ret);

    return ret;
}

inline int inno_iic_hw_wr_reg(struct inno_cam_mod *priv, u16 reg, u16 val)
{
    int ret = INNO_RET_OK;
    u32 lval = val;

    ret = inno_iic_host_write_reg(priv, reg, INNO_IIC_VAL_16BIT, lval);

    INNO_IIC("%s: 0x%04X = 0x%04X(%d)\n", priv->i2c_client->adapter->name, reg, val, ret);

    return ret;
}

inline int inno_iic_rd_reg(struct inno_cam_mod *priv, u16 addr, u32 *val, u8 len)
{
	int ret = INNO_RET_OK;

	switch (len) {
		case INNO_IIC_VAL_08BIT:
			ret = regmap_read(priv->regmap_08, addr, val);
			*val &= 0xFF;
			INNO_IIC("0x%04X = 0x%02X", addr, *val);
			break;
		case INNO_IIC_VAL_16BIT:
			ret = regmap_read(priv->regmap_16, addr, val);
			*val &= 0xFFFF;
			INNO_IIC("0x%04X = 0x%04X", addr, *val);
			break;
		case INNO_IIC_VAL_32BIT:
			ret = regmap_read(priv->regmap_32, addr, val);
			INNO_IIC("0x%04X = 0x%08X", addr, *val);
			break;
		default:
			ret = -EINVAL;
			INNO_RET_ERR(ret);
			return ret;
	}

	if (ret) {
		INNO_DEV_ERR(priv->dev, "I2C %s failed, 0x%04X = 0x%08X", "read", addr, *val);
		return ret;
	}

	return ret;
}

inline int inno_iic_wr_reg(struct inno_cam_mod *priv, u16 addr, u32 val, u8 len)
{
	int ret = INNO_RET_OK;

	switch (len) {
		case INNO_IIC_VAL_08BIT:
			INNO_IIC("0x%04X = 0x%02X\n", addr, val);
			ret = regmap_write(priv->regmap_08, addr, val);
			val &= 0xFF;
			break;
		case INNO_IIC_VAL_16BIT:
			INNO_IIC("0x%04X = 0x%04X\n", addr, val);
			ret = regmap_write(priv->regmap_16, addr, val);
			val &= 0xFFFF;
			break;
		case INNO_IIC_VAL_32BIT:
			INNO_IIC("0x%04X = 0x%08X\n", addr, val);
			ret = regmap_write(priv->regmap_32, addr, val);
			break;
		default:
			ret = -EINVAL;
			INNO_RET_ERR(ret);
			return ret;
	}

	if (ret) {
		INNO_DEV_ERR(priv->dev, "I2C %s failed, 0x%04X = 0x%08X", "write", addr, val);
		return ret;
	}

	return ret;
}

inline int inno_iic_io_exp_rd_reg(struct inno_cam_mod *priv, u8 addr, u32 *val)
{
	int ret = INNO_RET_OK;

	ret = regmap_read(priv->io_exp.regmap, addr, val);
	*val &= 0xFF;
	if (ret)
		INNO_DEV_ERR(priv->dev, "I2C %s failed, 0x%02X = 0x%02X", "read", addr, *val);
	else
		INNO_IIC("0x%02X = 0x%02X", addr, *val);

	return ret;
}

inline int inno_iic_io_exp_wr_reg(struct inno_cam_mod *priv, u8 addr, u8 val)
{
	int ret = INNO_RET_OK;

	INNO_IIC("0x%02X = 0x%02X\n", addr, val);;

	ret = regmap_write(priv->io_exp.regmap, addr, val);
	if (ret)
		INNO_DEV_ERR(priv->dev, "I2C %s failed, 0x%02X = 0x%02X", "write", addr, val);

	return ret;
}

int inno_iic_io_exp_device_release(struct inno_cam_mod *priv)
{
	int ret = INNO_RET_OK;

	INNO_DEV_DBG(priv->dev, "\n");

	if (priv->io_exp.i2c_client != NULL) {
		i2c_unregister_device(priv->io_exp.i2c_client);
		priv->io_exp.i2c_client = NULL;
	}

	return ret;
}

int inno_iic_io_exp_device_init(struct inno_cam_mod *priv)
{
	char *dev_name = "INNO_IO_EXPANDER";
	static struct regmap_config io_exp_regmap_config = {
		.reg_bits = 8,
		.val_bits = 8,
	};
	int ret = INNO_RET_OK;

	INNO_DEV_DBG(priv->dev, "\n");

	priv->io_exp.adap = i2c_get_adapter(priv->i2c_client->adapter->nr);
	memset(&priv->io_exp.brd, 0, sizeof(priv->io_exp.brd));
	strncpy(priv->io_exp.brd.type, dev_name, sizeof(priv->io_exp.brd.type));
	if (priv->info->io_exp_addr == INNO_II2_IO_EXP_IIC_ADDR_UNDEF)
		priv->info->io_exp_addr = INNO_II2_IO_EXP_IIC_ADDR;
	INNO_DEV_ALERT(priv->dev, "Probing IO Expander @ 0x%04X\n", (u16) priv->info->io_exp_addr);
	priv->io_exp.brd.addr = (u16) priv->info->io_exp_addr; // TODO: create macro for this, ideally get from the device tree
#if LINUX_VERSION_CODE < KERNEL_VERSION(5, 4, 0)
	priv->io_exp.i2c_client = i2c_new_device(priv->io_exp.adap, &priv->io_exp.brd);
#else
	priv->io_exp.i2c_client = i2c_new_client_device(priv->io_exp.adap, &priv->io_exp.brd);
#endif
	priv->io_exp.regmap = devm_regmap_init_i2c(priv->io_exp.i2c_client, &io_exp_regmap_config);
	
	if (IS_ERR(priv->io_exp.regmap)) {
		INNO_DEV_ERR(priv->dev, "Error in regmap\n");
		ret = PTR_ERR(priv->io_exp.regmap);
		inno_iic_io_exp_device_release(priv);
		return ret;
	}

	return ret;
}

int inno_iic_io_exp_level(struct inno_cam_mod *priv, int bit, int val)
{
	int ret = INNO_RET_OK;
	u32 reg_val = 0U;

	INNO_DEV_DBG(priv->dev, "Bit#%d: %s\n", bit, (val == INNO_GPIO_LEVEL_HIGH) ? "HIGH" : "LOW");

	/* 0 is output, 1 is input */
	ret = inno_iic_io_exp_wr_reg(priv, INNO_II2_IO_EXP_CONFIG, INNO_II2_IO_EXP_GPIO(INNO_II2_IO_EXP_RSV));
	if (ret) {
		INNO_DEV_ERR(priv->dev, "Error writing to IO Expander (%d)\n", ret);
		return ret;
	}
	msleep(INNO_GPIO_IO_DELAY_MS);

	ret = inno_iic_io_exp_rd_reg(priv, INNO_II2_IO_EXP_OUTPUT, &reg_val);
	if (ret) {
		INNO_DEV_ERR(priv->dev, "Error reading to IO Expander (%d)\n", ret);
		return ret;
	}

	if (val == INNO_GPIO_LEVEL_HIGH)
		reg_val |= INNO_II2_IO_EXP_GPIO(bit);
	else
		reg_val &= ~(INNO_II2_IO_EXP_GPIO(bit));

	/* Set BIT to LEVEL */
	ret = inno_iic_io_exp_wr_reg(priv, INNO_II2_IO_EXP_OUTPUT, reg_val);
	if (ret) {
		INNO_DEV_ERR(priv->dev, "Error writing to IO Expander (%d)\n", ret);
		return ret;
	}
	msleep(INNO_GPIO_LEVEL_DELAY_MS);

	return ret;
}

int inno_iic_io_exp_rst(struct inno_cam_mod *priv)
{
	int ret = INNO_RET_OK;

	INNO_DEV_DBG(priv->dev, "\n");

	ret = inno_iic_io_exp_level(priv, INNO_II2_IO_EXP_RST, INNO_GPIO_LEVEL_LOW);
	msleep(INNO_GPIO_RST_DELAY_MS);
	ret = inno_iic_io_exp_level(priv, INNO_II2_IO_EXP_RST, INNO_GPIO_LEVEL_HIGH);

	return ret;
}
