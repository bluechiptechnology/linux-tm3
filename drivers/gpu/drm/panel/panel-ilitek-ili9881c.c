// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright (C) 2017-2018, Bootlin
 * Copyright (C) 2021, Henson Li <henson@cutiepi.io>
 * Copyright (C) 2021, Penk Chen <penk@cutiepi.io>
 */

#include <linux/i2c.h>
#include <linux/delay.h>
#include <linux/device.h>
#include <linux/err.h>
#include <linux/errno.h>
#include <linux/fb.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/of_device.h>

#include <linux/gpio/consumer.h>
#include <linux/regulator/consumer.h>

#include <drm/drmP.h>
#include <drm/drm_mipi_dsi.h>
#include <drm/drm_modes.h>
#include <drm/drm_panel.h>
#include <drm/drm_bridge.h>



#include <video/mipi_display.h>


#define DRM_DEVICE(A) A->dev->dev

/* i2c: commands for the MCU driving the panel reset and backlight PWM */
enum REG_ADDR {
	REG_ID = 0x80,
	REG_PORTA,	/* BIT(2) for horizontal flip, BIT(3) for vertical flip */
	REG_PORTB,  // --
	REG_PORTC,
	REG_PORTD,
	REG_POWERON,// --
	REG_PWM,    // --
	REG_DDRA,
	REG_DDRB,
	REG_DDRC,
	REG_DDRD,
	REG_TEST,
	REG_WR_ADDRL,
	REG_WR_ADDRH,
	REG_READH,
	REG_READL,
	REG_WRITEH,
	REG_WRITEL,
	REG_ID2,

	REG_LCD_RST,
	REG_TP_RST,
	REG_TP_STATUS,
	REG_TP_POINT,
	REG_TP_VERSION,
	REG_ADC1,
	REG_ADC2,
	REG_MCU_AUTO_RESET,

	REG_MAX
};

enum ili9881c_op {
	ILI9881C_SWITCH_PAGE,
	ILI9881C_COMMAND0,
	ILI9881C_COMMAND1,
	ILI9881C_BUFFER,
	ILI9881C_MSLEEP,
};

struct ili9881c_instr {
	enum ili9881c_op	op;

	union arg {
		struct cmd {
			u8	cmd;
			u8	data;
		} cmd;
		u8	page;
	} arg;
};


struct ili9881c_setup {
	const u8** buffers;
	const struct ili9881c_instr* init;
	const size_t init_length;
};

struct ili9881c_desc {
	const struct ili9881c_setup *setup;
	const struct drm_display_mode *mode;
	const unsigned flags;
};

struct ili9881c {
	//struct drm_panel	panel;
	struct mipi_dsi_device	*dsi;
	struct i2c_client* i2c;
	struct drm_bridge bridge;
	struct drm_display_mode curr_mode;
	const struct ili9881c_desc	*desc;

	struct regulator	*power;
#ifdef GPIO_RESET
	struct gpio_desc	*reset;
#endif

	enum mipi_dsi_pixel_format pixel_format;
};

#define ILI9881C_SWITCH_PAGE_INSTR(_page)	\
	{					\
		.op = ILI9881C_SWITCH_PAGE,	\
		.arg = {			\
			.page = (_page),	\
		},				\
	}

#define ILI9881C_COMMAND_INSTR0(_cmd)		\
	{						\
		.op = ILI9881C_COMMAND0,		\
		.arg = {				\
			.cmd = {			\
				.cmd = (_cmd),		\
				.data = (0),	\
			},				\
		},					\
	}

#define ILI9881C_COMMAND_INSTR(_cmd, _data)		\
	{						\
		.op = ILI9881C_COMMAND1,		\
		.arg = {				\
			.cmd = {			\
				.cmd = (_cmd),		\
				.data = (_data),	\
			},				\
		},					\
	}

#define ILI9881C_BUFFER_INSTR(_data, _dlen)		\
	{						\
		.op = ILI9881C_BUFFER,		\
		.arg = {				\
			.cmd = {			\
				.cmd = (_dlen),		\
				.data = (_data),	\
			},				\
		},					\
	}

#define ILI9881C_MSLEEP_INSTR(_time)		\
	{						\
		.op = ILI9881C_MSLEEP,		\
		.arg = {				\
			.cmd = {			\
				.cmd = (_time),		\
				.data = (0),	\
			},				\
		},					\
	}

static const u8  reTerminal_buffer0[] = {
	0xA0, 0x00,0x26,0x35,0x16,0x19,0x2C,0x1F,0x1F,0x96,0x1C,0x28,0x80,0x1A,0x18,0x4C,0x21,0x27,0x55,0x65,0x39
};
static const u8 reTerminal_buffer1[] = {
	0xC0, 0x00,0x26,0x35,0x16,0x19,0x2C,0x1F,0x1F,0x96,0x1C,0x28,0x80,0x1A,0x18,0x4C,0x21,0x27,0x55,0x65,0x39
};



static const u8* reTerminal_buffers[] = {
	reTerminal_buffer0,
	reTerminal_buffer1,
};

static const struct ili9881c_instr reTerminal_init[] = {

	ILI9881C_SWITCH_PAGE_INSTR(0x01),
	ILI9881C_COMMAND_INSTR(0x91,0x00),
	ILI9881C_COMMAND_INSTR(0x92,0x00),
	ILI9881C_COMMAND_INSTR(0x93,0x72),
	ILI9881C_COMMAND_INSTR(0x94,0x00),
	ILI9881C_COMMAND_INSTR(0x95,0x00),
	ILI9881C_COMMAND_INSTR(0x96,0x09),
	ILI9881C_COMMAND_INSTR(0x97,0x00),
	ILI9881C_COMMAND_INSTR(0x98,0x00),

	ILI9881C_COMMAND_INSTR(0x09,0x01),
	ILI9881C_COMMAND_INSTR(0x0a,0x00),
	ILI9881C_COMMAND_INSTR(0x0b,0x00),
	ILI9881C_COMMAND_INSTR(0x0c,0x01),
	ILI9881C_COMMAND_INSTR(0x0d,0x00),
	ILI9881C_COMMAND_INSTR(0x0e,0x00),
	ILI9881C_COMMAND_INSTR(0x0f,0x1D),
	ILI9881C_COMMAND_INSTR(0x10,0x1D),
	ILI9881C_COMMAND_INSTR(0x11,0x00),
	ILI9881C_COMMAND_INSTR(0x12,0x00),
	ILI9881C_COMMAND_INSTR(0x13,0x00),
	ILI9881C_COMMAND_INSTR(0x14,0x00),
	ILI9881C_COMMAND_INSTR(0x15,0x00),
	ILI9881C_COMMAND_INSTR(0x16,0x00),
	ILI9881C_COMMAND_INSTR(0x17,0x00),
	ILI9881C_COMMAND_INSTR(0x18,0x00),
	ILI9881C_COMMAND_INSTR(0x19,0x00),
	ILI9881C_COMMAND_INSTR(0x1a,0x00),
	ILI9881C_COMMAND_INSTR(0x1b,0x00),
	ILI9881C_COMMAND_INSTR(0x1c,0x00),
	ILI9881C_COMMAND_INSTR(0x1d,0x00),
	ILI9881C_COMMAND_INSTR(0x1e,0xc0),
	ILI9881C_COMMAND_INSTR(0x1f,0x00),
	ILI9881C_COMMAND_INSTR(0x20,0x06),
	ILI9881C_COMMAND_INSTR(0x21,0x02),
	ILI9881C_COMMAND_INSTR(0x22,0x00),
	ILI9881C_COMMAND_INSTR(0x23,0x00),
	ILI9881C_COMMAND_INSTR(0x24,0x00),
	ILI9881C_COMMAND_INSTR(0x25,0x00),
	ILI9881C_COMMAND_INSTR(0x26,0x00),
	ILI9881C_COMMAND_INSTR(0x27,0x00),
	ILI9881C_COMMAND_INSTR(0x28,0x33),
	ILI9881C_COMMAND_INSTR(0x29,0x03),
	ILI9881C_COMMAND_INSTR(0x2a,0x00),
	ILI9881C_COMMAND_INSTR(0x2b,0x00),
	ILI9881C_COMMAND_INSTR(0x2c,0x00),
	ILI9881C_COMMAND_INSTR(0x2d,0x00),
	ILI9881C_COMMAND_INSTR(0x2e,0x00),
	ILI9881C_COMMAND_INSTR(0x2f,0x00),
	ILI9881C_COMMAND_INSTR(0x30,0x00),
	ILI9881C_COMMAND_INSTR(0x31,0x00),
	ILI9881C_COMMAND_INSTR(0x32,0x00),
	ILI9881C_COMMAND_INSTR(0x33,0x00),
	ILI9881C_COMMAND_INSTR(0x34,0x04),
	ILI9881C_COMMAND_INSTR(0x35,0x00),
	ILI9881C_COMMAND_INSTR(0x36,0x00),
	ILI9881C_COMMAND_INSTR(0x37,0x00),
	ILI9881C_COMMAND_INSTR(0x38,0x3C),
	ILI9881C_COMMAND_INSTR(0x39,0x07),
	ILI9881C_COMMAND_INSTR(0x3a,0x00),
	ILI9881C_COMMAND_INSTR(0x3b,0x00),
	ILI9881C_COMMAND_INSTR(0x3c,0x00),

	ILI9881C_COMMAND_INSTR(0x40,0x03),
	ILI9881C_COMMAND_INSTR(0x41,0x20),
	ILI9881C_COMMAND_INSTR(0x42,0x00),
	ILI9881C_COMMAND_INSTR(0x43,0x00),
	ILI9881C_COMMAND_INSTR(0x44,0x03),
	ILI9881C_COMMAND_INSTR(0x45,0x00),
	ILI9881C_COMMAND_INSTR(0x46,0x01),
	ILI9881C_COMMAND_INSTR(0x47,0x08),
	ILI9881C_COMMAND_INSTR(0x48,0x00),
	ILI9881C_COMMAND_INSTR(0x49,0x00),
	ILI9881C_COMMAND_INSTR(0x4a,0x00),
	ILI9881C_COMMAND_INSTR(0x4b,0x00),

	// ==== GL[3OUT=
	ILI9881C_COMMAND_INSTR(0x4c,0x01),
	ILI9881C_COMMAND_INSTR(0x4d,0x54),
	ILI9881C_COMMAND_INSTR(0x4e,0x57),
	ILI9881C_COMMAND_INSTR(0x4f,0x9b),
	ILI9881C_COMMAND_INSTR(0x50,0xf9),
	ILI9881C_COMMAND_INSTR(0x51,0x27),
	ILI9881C_COMMAND_INSTR(0x52,0x2f),
	ILI9881C_COMMAND_INSTR(0x53,0xf2),
	ILI9881C_COMMAND_INSTR(0x54,0xff),
	ILI9881C_COMMAND_INSTR(0x55,0xff),
	ILI9881C_COMMAND_INSTR(0x56,0xff),

	// ==== GR[3OUT==
	ILI9881C_COMMAND_INSTR(0x57,0x01),
	ILI9881C_COMMAND_INSTR(0x58,0x54),
	ILI9881C_COMMAND_INSTR(0x59,0x46),
	ILI9881C_COMMAND_INSTR(0x5a,0x8a),
	ILI9881C_COMMAND_INSTR(0x5b,0xf8),
	ILI9881C_COMMAND_INSTR(0x5c,0x26),
	ILI9881C_COMMAND_INSTR(0x5d,0x2f),
	ILI9881C_COMMAND_INSTR(0x5e,0xf2),
	ILI9881C_COMMAND_INSTR(0x5f,0xff),
	ILI9881C_COMMAND_INSTR(0x60,0xff),
	ILI9881C_COMMAND_INSTR(0x61,0xff),

	ILI9881C_COMMAND_INSTR(0x62,0x06),

	// == GOUT:4]_BWUTL[5:0]==
	ILI9881C_COMMAND_INSTR(0x63,0x01),
	ILI9881C_COMMAND_INSTR(0x64,0x00),
	ILI9881C_COMMAND_INSTR(0x65,0xa4),
	ILI9881C_COMMAND_INSTR(0x66,0xa5),
	ILI9881C_COMMAND_INSTR(0x67,0x58),
	ILI9881C_COMMAND_INSTR(0x68,0x5a),
	ILI9881C_COMMAND_INSTR(0x69,0x54),
	ILI9881C_COMMAND_INSTR(0x6a,0x56),
	ILI9881C_COMMAND_INSTR(0x6b,0x06),
	ILI9881C_COMMAND_INSTR(0x6c,0xff),
	ILI9881C_COMMAND_INSTR(0x6d,0x08),
	ILI9881C_COMMAND_INSTR(0x6e,0x02),
	ILI9881C_COMMAND_INSTR(0x6f,0xff),
	ILI9881C_COMMAND_INSTR(0x70,0x02),
	ILI9881C_COMMAND_INSTR(0x71,0x02),
	ILI9881C_COMMAND_INSTR(0x72,0xff),
	ILI9881C_COMMAND_INSTR(0x73,0xff),
	ILI9881C_COMMAND_INSTR(0x74,0xff),
	ILI9881C_COMMAND_INSTR(0x75,0xff),
	ILI9881C_COMMAND_INSTR(0x76,0xff),
	ILI9881C_COMMAND_INSTR(0x77,0xff),
	ILI9881C_COMMAND_INSTR(0x78,0xff),

	// == GOUT:4]_BWUTR[5:0]==
	ILI9881C_COMMAND_INSTR(0x79,0x01),
	ILI9881C_COMMAND_INSTR(0x7a,0x00),
	ILI9881C_COMMAND_INSTR(0x7b,0xa4),
	ILI9881C_COMMAND_INSTR(0x7c,0xa5),
	ILI9881C_COMMAND_INSTR(0x7d,0x59),
	ILI9881C_COMMAND_INSTR(0x7e,0x5b),
	ILI9881C_COMMAND_INSTR(0x7f,0x55),
	ILI9881C_COMMAND_INSTR(0x80,0x57),
	ILI9881C_COMMAND_INSTR(0x81,0x07),
	ILI9881C_COMMAND_INSTR(0x82,0xff),
	ILI9881C_COMMAND_INSTR(0x83,0x09),
	ILI9881C_COMMAND_INSTR(0x84,0x02),
	ILI9881C_COMMAND_INSTR(0x85,0xff),
	ILI9881C_COMMAND_INSTR(0x86,0x02),
	ILI9881C_COMMAND_INSTR(0x87,0x02),
	ILI9881C_COMMAND_INSTR(0x88,0xff),
	ILI9881C_COMMAND_INSTR(0x89,0xff),
	ILI9881C_COMMAND_INSTR(0x8a,0xff),
	ILI9881C_COMMAND_INSTR(0x8b,0xff),
	ILI9881C_COMMAND_INSTR(0x8c,0xff),
	ILI9881C_COMMAND_INSTR(0x8d,0xff),
	ILI9881C_COMMAND_INSTR(0x8e,0xff),

	ILI9881C_COMMAND_INSTR(0x8f,0x00),
	ILI9881C_COMMAND_INSTR(0x90,0x00),

	ILI9881C_COMMAND_INSTR(0x9d,0x00),
	ILI9881C_COMMAND_INSTR(0x9e,0x00),

	ILI9881C_COMMAND_INSTR(0xa0,0x35),
	ILI9881C_COMMAND_INSTR(0xa1,0x00),
	ILI9881C_COMMAND_INSTR(0xa2,0x00),
	ILI9881C_COMMAND_INSTR(0xa3,0x00),
	ILI9881C_COMMAND_INSTR(0xa4,0x00),
	ILI9881C_COMMAND_INSTR(0xa5,0x00),
	ILI9881C_COMMAND_INSTR(0xa6,0x08),
	ILI9881C_COMMAND_INSTR(0xa7,0x00),
	ILI9881C_COMMAND_INSTR(0xa8,0x00),
	ILI9881C_COMMAND_INSTR(0xa9,0x00),
	ILI9881C_COMMAND_INSTR(0xaa,0x00),
	ILI9881C_COMMAND_INSTR(0xab,0x00),
	ILI9881C_COMMAND_INSTR(0xac,0x00),
	ILI9881C_COMMAND_INSTR(0xad,0x00),
	ILI9881C_COMMAND_INSTR(0xae,0xff),
	ILI9881C_COMMAND_INSTR(0xaf,0x00),
	ILI9881C_COMMAND_INSTR(0xb0,0x00),

	ILI9881C_SWITCH_PAGE_INSTR(0x02),
	ILI9881C_COMMAND_INSTR(0x08,0x11),
	ILI9881C_COMMAND_INSTR(0x0a,0x0c),
	ILI9881C_COMMAND_INSTR(0x0f,0x06),
//  the panel should work without these - with a brief screen flick (? Negative and positive gamma correction ?)
//	ILI9881C_BUFFER_INSTR(0, sizeof(reTerminal_buffer0)), // buffer index 0
//	ILI9881C_BUFFER_INSTR(1, sizeof(reTerminal_buffer1)), // buffer index 1

	//===== GIP code finish =====//
	ILI9881C_COMMAND_INSTR(0x4C,0xA4), // PS_EN on ,0x default :A4
	ILI9881C_COMMAND_INSTR(0x18,0xF4), // SH on ,0x default E4 

	//=========================//
	ILI9881C_SWITCH_PAGE_INSTR(0x04),
	ILI9881C_COMMAND_INSTR(0x5D,0xAF), // VREG1 5.5V 
	ILI9881C_COMMAND_INSTR(0x5E,0xAF), // VREG2 5.5V
	ILI9881C_COMMAND_INSTR(0x60,0x9B), // VCM1 
	ILI9881C_COMMAND_INSTR(0x62,0x9B), // VCM2 
	ILI9881C_COMMAND_INSTR(0x82,0x38), // VREF_VGH_MOD_CLPSEL 16V 
	ILI9881C_COMMAND_INSTR(0x84,0x38), // VREF_VGH_DC 16V     
	ILI9881C_COMMAND_INSTR(0x86,0x18), // VREF_VGL_CLPSEL -10V       
	ILI9881C_COMMAND_INSTR(0x66,0xC4), // VGH_AC x4 ,0xdefault :04
	ILI9881C_COMMAND_INSTR(0xC1,0xF0), // VGH_DC x4 ,0xdefault :70
	ILI9881C_COMMAND_INSTR(0x70,0x60),
	ILI9881C_COMMAND_INSTR(0x71,0x00),

	//=========================//
	ILI9881C_COMMAND_INSTR(0x5B,0x33), // vcore_sel Voltage
	ILI9881C_COMMAND_INSTR(0x6C,0x10), // vcore bias L
	ILI9881C_COMMAND_INSTR(0x77,0x03), // vcore_sel Voltage
	ILI9881C_COMMAND_INSTR(0x7B,0x02), // vcore bias R

	//=========================//
	ILI9881C_SWITCH_PAGE_INSTR(0x01),
	ILI9881C_COMMAND_INSTR(0xF0,0x00), // 1280 Gate NL
	ILI9881C_COMMAND_INSTR(0xF1,0xC8), // 1280 Gate NL

	ILI9881C_SWITCH_PAGE_INSTR(0x05), // ??? RGB is on page 1  not on 5 ???
	ILI9881C_COMMAND_INSTR(0x22,0x3A), // RGB to BGR 0x3A
//	ILI9881C_COMMAND_INSTR(0x22,0x32), // RGB

	ILI9881C_SWITCH_PAGE_INSTR(0x00),     
	ILI9881C_COMMAND_INSTR(0x35,0x00),   //MIPI_DCS_SET_TEAR_ON = MIPI_DSI_DCS_TEAR_MODE_VBLANK ?


	ILI9881C_COMMAND_INSTR0(0x11),       //MIPI_DCS_EXIT_SLEEP_MODE
	ILI9881C_MSLEEP_INSTR(120),

	ILI9881C_COMMAND_INSTR0(0x29),       //MIPI_DCS_SET_DISPLAY_ON

};



static inline struct ili9881c *bridge_to_ili9881c(struct drm_bridge* bridge)
{
	return container_of(bridge, struct ili9881c, bridge);
}

/*
 * The panel seems to accept some private DCS commands that map
 * directly to registers.
 *
 * It is organised by page, with each page having its own set of
 * registers, and the first page looks like it's holding the standard
 * DCS commands.
 *
 * So before any attempt at sending a command or data, we have to be
 * sure if we're in the right page or not.
 */
static int ili9881c_send_data(struct ili9881c *ctx, const u8* data, size_t len)
{
	int ret;

	if (!ctx->dsi) {
		return -1;
	}

	if (len > 2) {
		printk("ili9881c_send_data size=%d (%02x ... %02x)\n", (u32) len, data[0], data[len -1]);
	}
	ret = mipi_dsi_dcs_write_buffer(ctx->dsi, data, len);
	if (ret < 0)
		return ret;

	return 0;
}

static void ili9881c_i2c_write(struct ili9881c *ctx, u8 reg, u8 val) {
	int ret;

	ret = i2c_smbus_write_byte_data(ctx->i2c, reg, val);
	if (ret)
		dev_err(&ctx->i2c->dev, "I2C write failed: %d\n", ret);
	usleep_range(1000, 1500);
}

static int ili9881c_prepare(struct ili9881c *ctx)
{
	unsigned int i;
	int ret;

	/* Power the panel */
	ret = regulator_enable(ctx->power);
	if (ret)
		return ret;
	msleep(5);

#ifdef GPIO_RESET
	gpiod_set_value(ctx->reset, 1);
	msleep(20);

	gpiod_set_value(ctx->reset, 0);
	msleep(20);
#else
	/* reset controlled by I2C GPIO Expander */
	ili9881c_i2c_write(ctx, REG_LCD_RST, 0);
	msleep(20);
	ili9881c_i2c_write(ctx, REG_LCD_RST, 1);
	msleep(20);

#endif

	for (i = 0; i < ctx->desc->setup->init_length; i++) {
		const struct ili9881c_instr *instr = &ctx->desc->setup->init[i];
		switch (instr->op) {
		case ILI9881C_SWITCH_PAGE: {
			u8 buf[4] = { 0xff, 0x98, 0x81, instr->arg.page};
			ret = ili9881c_send_data(ctx, buf, sizeof(buf));
		} break;
		case ILI9881C_COMMAND0: {
			u8 buf[1] = { instr->arg.cmd.cmd};
			ret = ili9881c_send_data(ctx, buf, sizeof(buf));
		} break;
		case ILI9881C_COMMAND1: {
			u8 buf[2] = { instr->arg.cmd.cmd, instr->arg.cmd.data};
			ret = ili9881c_send_data(ctx, buf, sizeof(buf));
		} break;
		case ILI9881C_BUFFER: {
			const u8* buf =   ctx->desc->setup->buffers[instr->arg.cmd.data];
			size_t len = instr->arg.cmd.cmd;
			ret = ili9881c_send_data(ctx, buf, len);
		} break;
		case ILI9881C_MSLEEP:
			ret = 0;
			msleep(instr->arg.cmd.cmd);
			break;
		}
		if (ret)
			goto error_exit;
	}


#ifndef GPIO_RESET
	ili9881c_i2c_write(ctx, REG_PWM, 255);
#endif

	return 0;
error_exit:
	 dev_err(&ctx->i2c->dev, "prepare failed ret=%d\n", ret);
#ifndef GPIO_RESET
	ili9881c_i2c_write(ctx, REG_POWERON, 0);
	ili9881c_i2c_write(ctx, REG_LCD_RST, 0);
	ili9881c_i2c_write(ctx, REG_PWM, 0);
#endif
	return ret;
}

static int ili9881c_bridge_attach(struct drm_bridge *bridge)
{
	struct ili9881c *ctx = bridge_to_ili9881c(bridge);

	printk("ili9881c_bridge_attach\n");
	return ili9881c_prepare(ctx);
}

static void ili9881c_bridge_mode_set(struct drm_bridge *bridge,
                    struct drm_display_mode *mode,
                    struct drm_display_mode *adj_mode)
{
    struct ili9881c *ctx = bridge_to_ili9881c(bridge);
	printk("ili9881c_bridge_mode_set\n");
    dev_dbg(DRM_DEVICE(bridge), "%s: mode: %d*%d@%d\n",__func__,
            mode->hdisplay,mode->vdisplay,mode->clock);
    drm_mode_copy(&ctx->curr_mode, adj_mode);

}

static void ili9881c_enable(struct drm_bridge *bridge)
{
	//struct ili9881c *ctx = bridge_to_ili9881c(bridge);

	printk("ili9881c_enable\n");

//	msleep(120);

//	mipi_dsi_dcs_set_display_on(ctx->dsi);

}

static void ili9881c_disable(struct drm_bridge *bridge)
{
	//struct ili9881c *ctx = bridge_to_ili9881c(bridge);
	printk("ili9881c_disable\n");

	//return mipi_dsi_dcs_set_display_off(ctx->dsi);
}



static int ili9881c_unprepare(struct ili9881c *ctx)
{

	regulator_disable(ctx->power);
#ifdef GPIO_RESET
	gpiod_set_value(ctx->reset, 1);
#else
	ili9881c_i2c_write(ctx, REG_PWM, 0); // backlight brightness
	ili9881c_i2c_write(ctx, REG_LCD_RST, 0);
#endif

	return 0;
}



static void ili9881c_dsi_register_host(struct ili9881c *ctx, struct mipi_dsi_host* dsi_host) {
	struct mipi_dsi_device_info info = {
		.type = "ili9881c",
		.channel = 0,
		.node = NULL,
	};

	/* register dsi host */
	if (dsi_host) {
		ctx->dsi = mipi_dsi_device_register_full(dsi_host, &info);
		if(IS_ERR(ctx->dsi)) {
			ctx->dsi = NULL;
			dev_err(&ctx->i2c->dev, "dsi device register failed!");
		} else {
			dev_warn(&ctx->i2c->dev, "dsi device registered\n");
		}
	} else {
			dev_warn(&ctx->i2c->dev, "dsi host is NULL\n");
	}
}

static struct drm_bridge_funcs ili9881c_bridge_funcs = {
    .enable = ili9881c_enable,
    .disable = ili9881c_disable,
    .mode_set = ili9881c_bridge_mode_set,
    .attach = ili9881c_bridge_attach,
};



static int ili9881c_dsi_probe(struct i2c_client *i2c, const struct i2c_device_id *id)
{
	struct ili9881c *ctx;
	struct device *dev = &i2c->dev;
	int ret;

	printk("ili9881c: probe\n");

	ctx = devm_kzalloc(dev, sizeof(*ctx), GFP_KERNEL);
	if (!ctx)
		return -ENOMEM;
	ctx->i2c = i2c;
	ctx->desc = of_device_get_match_data(dev);
	ctx->power = devm_regulator_get(dev, "power");
	if (IS_ERR(ctx->power)) {
		dev_err(dev, "Couldn't get our power regulator\n");
		return PTR_ERR(ctx->power);
	}

#ifdef GPIO_RESET
	ctx->reset = devm_gpiod_get(dev, "reset", GPIOD_OUT_LOW);
	if (IS_ERR(ctx->reset)) {
		dev_err(dev, "Couldn't get our reset GPIO\n");
		return PTR_ERR(ctx->reset);
	}
#endif

	i2c_set_clientdata(i2c, ctx);

	ctx->pixel_format = MIPI_DSI_FMT_RGB888;

    ctx->bridge.funcs = &ili9881c_bridge_funcs;
    ctx->bridge.of_node = dev->of_node;

    ret = drm_bridge_add(&ctx->bridge);
    if (ret) {
        dev_err(dev, "failed to add sn65dsi83 bridge\n");
    }

	/* find dsi host */
	{
		struct mipi_dsi_host* dsi_host = NULL;
		struct device_node *host = of_parse_phandle(dev->of_node, "dsi-host", 0);
		if (host) {
			dsi_host = of_find_mipi_dsi_host_by_node(host);
			of_node_put(host);
			dev_warn(dev, "dsi-host found: %p\n", dsi_host);
			ili9881c_dsi_register_host(ctx, dsi_host);
		} else
			dev_warn(dev, "dsi-host not found\n");
	}


	ili9881c_i2c_write(ctx, REG_POWERON, 1);

	ili9881c_i2c_write(ctx, REG_MCU_AUTO_RESET, 0);

	printk("ili9881c: probe end: %d\n", ret);
	return ret;
}

static int ili9881c_dsi_remove(struct i2c_client *i2c)
{
	struct ili9881c *ctx = i2c_get_clientdata(i2c);

	drm_bridge_remove(&ctx->bridge);
	//devm_kfree(ctx); //??
	return 0;
}

static int ili9881c_i2c_command(struct i2c_client *i2c, unsigned int cmd, void *arg)
{
	struct ili9881c *ctx = i2c_get_clientdata(i2c);
	printk("ili9881c i2c command: %d\n", cmd);
	switch (cmd) {
		case 0:
			ili9881c_unprepare(ctx);
			break;
		case 1: 
			return ili9881c_prepare(ctx);
		case 3:
			return ctx->pixel_format;
		case 0xD4:
			ili9881c_dsi_register_host(ctx, (struct mipi_dsi_host*) arg);
			break;
	}

	return 0;
}

static const struct drm_display_mode reTerminal_default_mode = {
	.clock		= 62712,

	.hdisplay	= 720,
	.hsync_start	= 720 + 10,
	.hsync_end	= 720 + 10 + 20,
	.htotal		= 720 + 10 + 20 + 30,

	.vdisplay	= 1280,
	.vsync_start	= 1280 + 10,
	.vsync_end	= 1280 + 10 + 20,
	.vtotal		= 1280 + 10 + 20 + 30,

	.width_mm	= 62,
	.height_mm	= 110,
};

static const struct  ili9881c_setup reTerminal_setup = {
	.buffers = reTerminal_buffers,
	.init = reTerminal_init,
	.init_length = ARRAY_SIZE(reTerminal_init),
};

static const struct ili9881c_desc lhr050h41_desc = {
	.setup = &reTerminal_setup,
	.mode = &reTerminal_default_mode,
	.flags = MIPI_DSI_MODE_VIDEO_SYNC_PULSE,
};


static const struct of_device_id ili9881c_of_match[] = {
	{ .compatible = "bananapi,lhr050h41", .data = &lhr050h41_desc },

	{}
};

static const struct i2c_device_id ili9881c_i2c_ids[] = {
	{ "ili9881c", 0 },
	{ }
};
MODULE_DEVICE_TABLE(of, ili9881c_of_match);

static struct mipi_dsi_driver ili9881c_dsi_driver = {
    .driver.name = "ili9881c",
};

static struct i2c_driver ili9881c_driver = {
	.probe		= ili9881c_dsi_probe,
	.remove		= ili9881c_dsi_remove,
	.command	= ili9881c_i2c_command,
	.id_table	= ili9881c_i2c_ids,
	.driver = {
		.name		= "ili9881c",
		.of_match_table	= ili9881c_of_match,
	},
};

static int __init ili9881c_init(void)
{
    if (IS_ENABLED(CONFIG_DRM_MIPI_DSI))
        mipi_dsi_driver_register(&ili9881c_dsi_driver);

    return i2c_add_driver(&ili9881c_driver);
}
module_init(ili9881c_init);

static void __exit ili9881c_exit(void)
{
    i2c_del_driver(&ili9881c_driver);

    if (IS_ENABLED(CONFIG_DRM_MIPI_DSI))
        mipi_dsi_driver_unregister(&ili9881c_dsi_driver);
}
module_exit(ili9881c_exit);


MODULE_AUTHOR("Maxime Ripard <maxime.ripard@free-electrons.com>");
MODULE_DESCRIPTION("Ilitek ILI9881C Controller Driver");
MODULE_LICENSE("GPL v2");
