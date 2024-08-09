/*
 * inno_common.h
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

#ifndef __INNO_COMMON_H__
#define __INNO_COMMON_H__

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

#include "inno_common_data.h"

#if defined(_INNO_FOR_NVIDIA_)
/* Use general GPIO instead of I2C GPIO Expander on EVDM-OOM1 */
#include "../../platform/tegra/camera/camera_gpio.h"
#endif

/*
 * Header
 * 32-Bit Magic
 * 32-Bit Version
 * 256-Byte Description
 * 16-Bit PLL Size
 * 16-Bit CRC
 */
struct inno_firmware {
	u32							magic;
	u32							version;
	char						desc[INNO_FW_DESCRIPTION];
	u16							pll_init_size;
	u16							crc;
} __packed;

struct inno_res_info {
	u32							width;
	u32							height;
	u32							fps;
	u32							format;
	u32							mode;
};

struct inno_io_expander {
	struct i2c_client			*i2c_client;
	struct i2c_adapter			*adap;
	struct i2c_board_info		brd;
	struct regmap				*regmap;
};

struct inno_cam_mod_info {
	const struct inno_res_info	*prvw_modes;
	const struct inno_res_info	*cur_mode;
	u32							prvw_modes_num;
	bool						mode_change;
	const struct firmware 		*inno_fw_src;
	char						*fw_name;
	const char					*dev_name;
	u32 						isp_id;
	u32							sensor_id;
	u32							sensor_reg;
	u32							sensor_addr;
	u32							io_exp_addr;
	bool						io_exp_init;
	bool						is_gmsl;
};

struct inno_evxm_yymz_ctrl {
	struct v4l2_ctrl_handler	handler;
	struct {
		struct v4l2_ctrl		*auto_exp;
		struct v4l2_ctrl		*exposure;
		struct v4l2_ctrl		*gain;
	};
	struct {
		struct v4l2_ctrl		*auto_wb;
		struct v4l2_ctrl		*wb_temp;
	};
	struct {
		struct v4l2_ctrl		*scene_mode;
	};
};

#if defined(_INNO_FOR_NVIDIA_)
#define inno_comm_sd_to_cam(sd) ({ \
		struct camera_common_data *s_data = container_of(sd, struct camera_common_data, subdev); \
		(struct inno_cam_mod *)s_data->priv; \
})
#define inno_comm_ctrl_to_sd(ctrl) \
		container_of(ctrl->handler, struct inno_cam_mod, ctrls.handler)->subdev

struct inno_cam_mod {
	struct v4l2_subdev			*subdev;
	struct i2c_client			*i2c_client;
	struct device 				*dev;
	struct mutex				inno_cam_mod_lock;

	struct regmap 				*regmap_08;
	struct regmap 				*regmap_16;
	struct regmap 				*regmap_32;

	struct regulator_bulk_data	supplies[INNO_NUM_CONSUMERS];
	struct inno_evxm_yymz_ctrl	ctrls;

	struct v4l2_mbus_framefmt	fmt;
	struct v4l2_fract			frame_interval;

	u16							fine_integ_time;
	u32							frame_length;

	struct inno_cam_mod_info	*info;
	struct inno_io_expander		io_exp;

	struct camera_common_data	*s_data;
	struct tegracam_device		*tc_dev;
	
#if INNO_MAX_GMSL
	struct device				*ser_dev;
	struct device				*dser_dev;
	struct gmsl_link_ctx		g_ctx;
#endif
};
#elif defined(_INNO_FOR_NXP_)
#define inno_comm_sd_to_cam(sd) \
		container_of(sd, struct inno_cam_mod, subdev)
#define inno_comm_ctrl_to_sd(ctrl) \
		&container_of(ctrl->handler, struct inno_cam_mod, ctrls.handler)->subdev

struct inno_cam_mod {
	struct v4l2_subdev			subdev;
	struct i2c_client			*i2c_client;
	struct device 				*dev;
	struct mutex				inno_cam_mod_lock;

	struct regmap 				*regmap_08;
	struct regmap 				*regmap_16;
	struct regmap 				*regmap_32;

	struct regulator_bulk_data	supplies[INNO_NUM_CONSUMERS];
	struct inno_evxm_yymz_ctrl	ctrls;

	struct v4l2_mbus_framefmt	fmt;
	struct v4l2_fract			frame_interval;

	u16							fine_integ_time;
	u32							frame_length;

	struct inno_cam_mod_info	*info;
	struct inno_io_expander		io_exp;

	struct media_pad			pads[INNO_SENS_PADS_NUM];

	/* GPIO Descriptor */
	struct gpio_desc			*reset;
	struct gpio_desc			*isp_en;
};
#endif

#if defined(_INNO_FOR_NVIDIA_)
struct v4l2_subdev_state {
	/* lock for the struct v4l2_subdev_state fields */
	struct mutex _lock;
	struct mutex *lock;
	struct v4l2_subdev_pad_config *pads;
};
#endif

#if defined(_INNO_FOR_NXP_)
struct reg_8 {
	u16							addr;	/* Address of the register */
	u8							val;	/* 8-bit value of the register */
};

struct reg_16 {
	u16							addr;	/* Address of the register */
	u16							val;	/* 16-bit value of the register */
};

enum switch_state {
	SWITCH_OFF,
	SWITCH_ON,
};
#endif

#define INNO_COMM_REG_8			struct reg_8
#define INNO_COMM_REG_16		struct reg_16

/* Regulator Supplies */
static const char * const inno_comm_supply_name[] = {
	"AVDD",
	"DVDD",
	"VDDIO",
};

extern struct regmap_config sensor_regmap_08_config;
extern struct regmap_config sensor_regmap_16_config;
extern struct regmap_config sensor_regmap_32_config;

#if defined(_INNO_FOR_NXP_)
inline void msleep_range(unsigned int delay_base);
#endif

void inno_comm_prt_tbls_8(INNO_COMM_REG_8 table[], u16 end_addr);
void inno_comm_prt_tbls_16(INNO_COMM_REG_16 table[], u16 end_addr);
int inno_comm_dump_dbg_reg(struct inno_cam_mod *priv);

int inno_comm_stall(struct inno_cam_mod *priv, bool stall);
int inno_comm_set_format(struct inno_cam_mod *priv, int index);
int inno_comm_power(struct inno_cam_mod *priv, int val);
int inno_comm_gpio_rst(struct inno_cam_mod *priv);
int inno_comm_rst(struct inno_cam_mod *priv);

int inno_comm_wait_idle(struct inno_cam_mod *priv);
int inno_comm_sipm_read(struct inno_cam_mod *priv, u32 port, u8 id, u32 reg, u32 *val);

int inno_comm_get_sensor_id(struct inno_cam_mod *priv);
int inno_comm_get_isp_id(struct inno_cam_mod *priv);
int inno_comm_get_mipi_info(struct inno_cam_mod *priv);

/* Nvidia function pointer start */
int inno_comm_open(struct v4l2_subdev *sd, struct v4l2_subdev_fh *fh);
/* Nvidia function pointer end */

/* NXP function pointer */
int inno_comm_g_volatile_ctrl(struct v4l2_ctrl *ctrl);
int inno_comm_try_ctrl(struct v4l2_ctrl *ctrl);
int inno_comm_set_ctrl_exposure(struct inno_cam_mod *priv, s32 mode);
int inno_comm_set_ctrl_white_balance(struct inno_cam_mod *priv, s32 ctrl_val);
int inno_comm_set_ctrl_white_balance_temp(struct inno_cam_mod *priv, s32 ctrl_val);
int inno_comm_set_gain(struct inno_cam_mod *priv, s32 ctrl_val);
int inno_comm_set_exposure(struct inno_cam_mod *priv, s32 ctrl_val);
int inno_comm_set_scene_mode(struct inno_cam_mod *priv, s32 ctrl_val);
int inno_comm_s_ctrl(struct v4l2_ctrl *ctrl);
int inno_comm_s_power(struct v4l2_subdev *sd, int on);
int inno_comm_g_register(struct v4l2_subdev *sd, struct v4l2_dbg_register *reg);
int inno_comm_s_register(struct v4l2_subdev *sd, const struct v4l2_dbg_register *reg);
int inno_comm_g_frame_interval(struct v4l2_subdev *sd, struct v4l2_subdev_frame_interval *fi);
int inno_comm_s_frame_interval(struct v4l2_subdev *sd, struct v4l2_subdev_frame_interval *fi);
int inno_comm_s_stream(struct v4l2_subdev *sd, int enable);
int inno_comm_enum_mbus_code(struct v4l2_subdev *sd, struct v4l2_subdev_state *sd_state, struct v4l2_subdev_mbus_code_enum *code);
int inno_comm_enum_frame_size(struct v4l2_subdev *sd, struct v4l2_subdev_state *sd_state, struct v4l2_subdev_frame_size_enum *fse);
int inno_comm_enum_frame_interval(struct v4l2_subdev *sd, struct v4l2_subdev_state *sd_state, struct v4l2_subdev_frame_interval_enum *fie);
int inno_comm_get_fmt(struct v4l2_subdev *sd, struct v4l2_subdev_state *sd_state, struct v4l2_subdev_format *format);
const struct inno_res_info *inno_comm_find_mode(struct inno_cam_mod *priv, u32 width, u32 height);
int inno_comm_set_fmt(struct v4l2_subdev *sd, struct v4l2_subdev_state *sd_state, struct v4l2_subdev_format *format);
int inno_comm_link_setup(struct media_entity *entity, const struct media_pad *local, const struct media_pad *remote, u32 flags);
/* NXP function pointer end */

int inno_comm_get_regulators(struct inno_cam_mod *priv);
int inno_comm_init_controls(struct inno_cam_mod *priv, const struct v4l2_ctrl_ops *ctrl_ops);

int inno_comm_request_firmware(struct inno_cam_mod *priv);
int inno_comm_write_fw_window(struct inno_cam_mod *priv, u16 *win_pos, const u8 *buf, u32 len);
int inno_comm_reg_polling(struct inno_cam_mod *priv, u16 addr, u16 val, u16 mask, int poll_interval_ms, int retries, bool expected);
int inno_comm_firmware_handler(struct inno_cam_mod *priv, const struct inno_firmware *inno_fw);
void inno_comm_nowait_firmware(const struct firmware *fw, void *context);
int inno_comm_load_firmware(struct inno_cam_mod *priv);
int inno_comm_get_fine_integ_time(struct inno_cam_mod *priv, u16 *fine_time);
int inno_comm_board_setup(struct inno_cam_mod *priv);

#if defined(_INNO_FOR_NVIDIA_)
int inno_comm_probe(struct i2c_client *client,
					struct inno_cam_mod *priv,
					struct tegracam_device *tc_dev,
					struct camera_common_sensor_ops *sensor_ops,
					const struct v4l2_subdev_internal_ops *v4l2sd_internal_ops,
					struct tegracam_ctrl_ops *tcctrl_ops);
int inno_comm_remove(struct i2c_client *client);
#elif defined(_INNO_FOR_NXP_)
int inno_comm_probe(struct i2c_client *client,
					struct inno_cam_mod *priv,
					struct v4l2_mbus_framefmt *fmt,
					const struct media_entity_operations *media_ops,
					struct v4l2_subdev_ops *subdev_ops,
					const struct v4l2_ctrl_ops *ctrl_ops);
void inno_comm_remove(struct i2c_client *client);
#endif

#endif /* __INNO_COMMON_H__ */
