// SPDX-License-Identifier: GPL-2.0
/*
 *  Copyright (C) 2020 Texas Instruments Incorporated - https://www.ti.com
 *  Author: Peter Ujfalusi <peter.ujfalusi@ti.com>
 */

#include <linux/clk.h>
#include <linux/device.h>
#include <linux/gpio/consumer.h>
#include <linux/i2c.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/regmap.h>
#include <linux/regulator/consumer.h>
#include <linux/slab.h>
#include <linux/version.h>

#include <drm/drm_atomic_helper.h>
#include <drm/drm_crtc_helper.h>
//#include <drm/drm_drv.h>
#include <drm/drm_mipi_dsi.h>
#include <drm/drm_of.h>
#include <drm/drm_panel.h>
#include <video/mipi_display.h>
#include <video/videomode.h>
#include <drm/drm_bridge.h>
#include <drm/drm_mipi_dsi.h>

/* Global (16-bit addressable) */
#define TC358768_CHIPID			0x0000
#define TC358768_SYSCTL			0x0002
#define TC358768_CONFCTL		0x0004
#define TC358768_VSDLY			0x0006
#define TC358768_DATAFMT		0x0008
#define TC358768_GPIOEN			0x000E
#define TC358768_GPIODIR		0x0010
#define TC358768_GPIOIN			0x0012
#define TC358768_GPIOOUT		0x0014
#define TC358768_PLLCTL0		0x0016
#define TC358768_PLLCTL1		0x0018
#define TC358768_CMDBYTE		0x0022
#define TC358768_PP_MISC		0x0032
#define TC358768_DSITX_DT		0x0050
#define TC358768_FIFOSTATUS		0x00F8

/* Debug (16-bit addressable) */
#define TC358768_VBUFCTRL		0x00E0
#define TC358768_DBG_WIDTH		0x00E2
#define TC358768_DBG_VBLANK		0x00E4
#define TC358768_DBG_DATA		0x00E8


/* TX PHY (32-bit addressable) */
#define TC358768_CLW_DPHYCONTTX		0x0100
#define TC358768_D0W_DPHYCONTTX		0x0104
#define TC358768_D1W_DPHYCONTTX		0x0108
#define TC358768_D2W_DPHYCONTTX		0x010C
#define TC358768_D3W_DPHYCONTTX		0x0110
#define TC358768_CLW_CNTRL		0x0140
#define TC358768_D0W_CNTRL		0x0144
#define TC358768_D1W_CNTRL		0x0148
#define TC358768_D2W_CNTRL		0x014C
#define TC358768_D3W_CNTRL		0x0150

/* TX PPI (32-bit addressable) */
#define TC358768_STARTCNTRL		0x0204
#define TC358768_DSITXSTATUS		0x0208
#define TC358768_LINEINITCNT		0x0210
#define TC358768_LPTXTIMECNT		0x0214
#define TC358768_TCLK_HEADERCNT		0x0218
#define TC358768_TCLK_TRAILCNT		0x021C
#define TC358768_THS_HEADERCNT		0x0220
#define TC358768_TWAKEUP		0x0224
#define TC358768_TCLK_POSTCNT		0x0228
#define TC358768_THS_TRAILCNT		0x022C
#define TC358768_HSTXVREGCNT		0x0230
#define TC358768_HSTXVREGEN		0x0234
#define TC358768_TXOPTIONCNTRL		0x0238
#define TC358768_BTACNTRL1		0x023C

/* TX CTRL (32-bit addressable) */
#define TC358768_DSI_CONTROL		0x040C
#define TC358768_DSI_STATUS		0x0410
#define TC358768_DSI_INT		0x0414
#define TC358768_DSI_INT_ENA		0x0418
#define TC358768_DSICMD_RDFIFO		0x0430
#define TC358768_DSI_ACKERR		0x0434
#define TC358768_DSI_ACKERR_INTENA	0x0438
#define TC358768_DSI_ACKERR_HALT	0x043c
#define TC358768_DSI_RXERR		0x0440
#define TC358768_DSI_RXERR_INTENA	0x0444
#define TC358768_DSI_RXERR_HALT		0x0448
#define TC358768_DSI_ERR		0x044C
#define TC358768_DSI_ERR_INTENA		0x0450
#define TC358768_DSI_ERR_HALT		0x0454
#define TC358768_DSI_CONFW		0x0500
#define TC358768_DSI_LPCMD		0x0500
#define TC358768_DSI_RESET		0x0504
#define TC358768_DSI_INT_CLR		0x050C
#define TC358768_DSI_START		0x0518

/* DSITX CTRL (16-bit addressable) */
#define TC358768_DSICMD_TX		0x0600
#define TC358768_DSICMD_TYPE		0x0602
#define TC358768_DSICMD_WC		0x0604
#define TC358768_DSICMD_WD0		0x0610
#define TC358768_DSICMD_WD1		0x0612
#define TC358768_DSICMD_WD2		0x0614
#define TC358768_DSICMD_WD3		0x0616
#define TC358768_DSI_EVENT		0x0620
#define TC358768_DSI_VSW		0x0622
#define TC358768_DSI_VBPR		0x0624
#define TC358768_DSI_VACT		0x0626
#define TC358768_DSI_HSW		0x0628
#define TC358768_DSI_HBPR		0x062A
#define TC358768_DSI_HACT		0x062C

/* TC358768_DSI_CONTROL (0x040C) register */
#define TC358768_DSI_CONTROL_DIS_MODE	BIT(15)
#define TC358768_DSI_CONTROL_TXMD	BIT(7)
#define TC358768_DSI_CONTROL_HSCKMD	BIT(5)
#define TC358768_DSI_CONTROL_EOTDIS	BIT(0)

/* TC358768_DSI_CONFW (0x0500) register */
#define TC358768_DSI_CONFW_MODE_SET	(5 << 29)
#define TC358768_DSI_CONFW_MODE_CLR	(6 << 29)
#define TC358768_DSI_CONFW_ADDR_DSI_CONTROL	(0x3 << 24)

#define MAX(X,Y) ((X) > (Y)) ? (X) : (Y)

#ifndef MIPI_DSI_MODE_NO_EOT_PACKET
#define MIPI_DSI_MODE_NO_EOT_PACKET MIPI_DSI_MODE_EOT_PACKET
#endif



#define REM(X) if (debug) dev_warn(priv->dev,"\n\t/* %s */\n", X);
#define WR(R,V,C) if (debug) dev_warn(priv->dev, "\treg_write(0x%04x, 0x%04x);%s\n", (R), (V), strlen(C) ?  " /* " C " */" : ""); \
				tc358768_write(priv, (R), (V))
#define DELAYUS(X) if (debug) dev_err(priv->dev,"\tusleep_range(%d, %d);\n", (X), (X) + ((X)/2)) ; \
				usleep_range((X), (X) + ((X)/2) )


static const char * const tc358768_supplies[] = {
	"vddc", "vddmipi", "vddio"
};

struct tc358768_dsi_output {
	struct mipi_dsi_device *dev;
	struct drm_panel *panel;
	struct drm_bridge *bridge;
	struct i2c_client *i2c_bridge;
};

struct dsi_tx_params {
	u32 fbd;	/* PLL feedback divider */
	u32 prd;	/* PLL input divider */
	u32 frs;	/* PLL Freqency range for HSCK (post divider) */
	u32 pllInputClk;
	u32 hsByteClk;
	u32 lineInitControl;
	u32 tlptxTimeCnt;
	u32 tclkPrepareCnt;
	u32 tclkZeroCnt;
	u32 tclkTrailCnt;
	u32 thsPrepareCnt;
	u32 thsZeroCnt;
	u32 twakeUpCnt;
	u32 tclkPostCnt;
	u32 thsTrailCnt;
	u32 rxTasureCnt;
	u32 txTaGoCnt;
	u8  valid;
};

struct tc358768_priv {
	struct device *dev;
	struct regmap *regmap;
	struct gpio_desc *reset_gpio;
	struct regulator_bulk_data supplies[ARRAY_SIZE(tc358768_supplies)];
	int enabled;
	int error;
	u8 reset_gpio_exists;

	struct mipi_dsi_host dsi_host;
	struct drm_bridge bridge;
	struct drm_connector connector;
	struct tc358768_dsi_output output;

	struct dsi_tx_params dtxpar; 

	u32 refclk;    /* 0 if REFCLK is not used, otherwise external crystal value in kHz */
	u32 dsiclk;
	u32 dsi_lanes; /* number of DSI Lanes */
	u32 bpp;       /* bits per pixel */

	enum mipi_dsi_pixel_format dsi_pixel_format;

	u32 dsi_mode_flags;
	u32 hporch_override1; /* DSI HPW + HBP */
	u32 hporch_override2; /* DSI HFP */
	u16 vs_delay;
	u8  rgb_mapping_mode;
};

static int tc358768_calc_clocks(struct tc358768_priv *priv, const struct drm_display_mode *mode);

static int i2c_bridge_command(struct tc358768_priv *priv, int command, void* arg);

static inline struct tc358768_priv *dsi_host_to_tc358768(struct mipi_dsi_host
							 *host)
{
	return container_of(host, struct tc358768_priv, dsi_host);
}

static inline struct tc358768_priv *bridge_to_tc358768(struct drm_bridge
						       *bridge)
{
	return container_of(bridge, struct tc358768_priv, bridge);
}

static inline struct tc358768_priv* connector_to_tc358768(struct drm_connector *c)
{
	return container_of(c, struct tc358768_priv, connector);
}

static int tc358768_clear_error(struct tc358768_priv *priv)
{
	int ret = priv->error;

	priv->error = 0;
	return ret;
}

static void tc358768_write(struct tc358768_priv *priv, u32 reg, u32 val)
{
	/* work around https://gcc.gnu.org/bugzilla/show_bug.cgi?id=81715 */
	int tmpval = val;
	size_t count = 2;

	if (priv->error) {
		return;
	}

	/* 16-bit register? */
	if (reg < 0x100 || reg >= 0x600)
		count = 1;

	priv->error = regmap_bulk_write(priv->regmap, reg, &tmpval, count);
}

static void tc358768_read(struct tc358768_priv *priv, u32 reg, u32 *val)
{
	size_t count = 2;

	if (priv->error)
		return;

	/* 16-bit register? */
	if (reg < 0x100 || reg >= 0x600) {
		*val = 0;
		count = 1;
	}

	priv->error = regmap_bulk_read(priv->regmap, reg, val, count);
}

static void tc358768_update_bits(struct tc358768_priv *priv, u32 reg, u32 mask,
				 u32 val)
{
	u32 tmp, orig;

	tc358768_read(priv, reg, &orig);
	tmp = orig & ~mask;
	tmp |= val & mask;
	if (tmp != orig)
		tc358768_write(priv, reg, tmp);
}


static void tc358768_hw_enable(struct tc358768_priv *priv)
{
	int ret;

	if (priv->enabled)
		return;

	ret = regulator_bulk_enable(ARRAY_SIZE(priv->supplies), priv->supplies);
	if (ret < 0)
		dev_err(priv->dev, "error enabling regulators (%d)\n", ret);

	if (!priv->reset_gpio_exists)
		return;

	if (priv->reset_gpio)
		usleep_range(200, 300);

	/*
	 * The RESX is active low (GPIO_ACTIVE_LOW).
	 * DEASSERT (value = 0) the reset_gpio to enable the chip
	 */
	gpiod_set_value_cansleep(priv->reset_gpio, 0);

	/* wait for encoder clocks to stabilize */
	usleep_range(1000, 2000);


	priv->enabled = true;
}

static void tc358768_hw_disable(struct tc358768_priv *priv)
{
	int ret;

	if (!priv->enabled)
		return;

	if (priv->reset_gpio_exists) {
		/*
		 * The RESX is active low (GPIO_ACTIVE_LOW).
		 * ASSERT (value = 1) the reset_gpio to disable the chip
		 */
		gpiod_set_value_cansleep(priv->reset_gpio, 1);
	}

	ret = regulator_bulk_disable(ARRAY_SIZE(priv->supplies),
				     priv->supplies);
	if (ret < 0)
		dev_err(priv->dev, "error disabling regulators (%d)\n", ret);

	priv->enabled = false;
}




static int tc358768_dsi_host_attach(struct mipi_dsi_host *host,
				    struct mipi_dsi_device *dev)
{
	struct tc358768_priv *priv = dsi_host_to_tc358768(host);
	struct drm_bridge *bridge;
	struct drm_panel *panel;
	int ret;

	if (dev->lanes > 4) {
		dev_err(priv->dev, "unsupported number of data lanes(%u)\n",
			dev->lanes);
		return -EINVAL;
	}

	/*
	 * tc358768 supports both Video and Pulse mode, but the driver only
	 * implements Video (event) mode currently
	 */
	if (!(dev->mode_flags & MIPI_DSI_MODE_VIDEO)) {
		dev_err(priv->dev, "Only MIPI_DSI_MODE_VIDEO is supported\n");
		return -ENOTSUPP;
	}

	/*
	 * tc358768 supports RGB888, RGB666, RGB666_PACKED and RGB565, but only
	 * RGB888 is verified.
	 */
	priv->dsi_pixel_format = dev->format;

	ret = drm_of_find_panel_or_bridge(host->dev->of_node, 1, 0, &panel,
					  &bridge);
	if (ret)
		return ret;


#if LINUX_VERSION_CODE >= KERNEL_VERSION(5,5,0)
	if (panel) {
		bridge = drm_panel_bridge_add_typed(panel,
						    DRM_MODE_CONNECTOR_DSI);
		if (IS_ERR(bridge))
			return PTR_ERR(bridge);
	}
#endif

	priv->output.dev = dev;
	priv->output.bridge = bridge;
	priv->output.panel = panel;

	priv->dsi_lanes = dev->lanes;

	drm_bridge_add(&priv->bridge);

	return 0;
}

static int tc358768_dsi_host_detach(struct mipi_dsi_host *host,
				    struct mipi_dsi_device *dev)
{
	struct tc358768_priv *priv = dsi_host_to_tc358768(host);

	drm_bridge_remove(&priv->bridge);
#if LINUX_VERSION_CODE >= KERNEL_VERSION(4,13,0)
#ifdef CONFIG_DRM_PANEL_BRIDGE
	if (priv->output.panel)
		drm_panel_bridge_remove(priv->output.bridge);
#endif
#endif
	return 0;
}

static ssize_t tc358768_dsi_host_transfer(struct mipi_dsi_host *host,
					  const struct mipi_dsi_msg *msg)
{
	struct tc358768_priv *priv = dsi_host_to_tc358768(host);
	struct mipi_dsi_packet packet;
	int i;
	int ret;

	if (!priv->enabled) {
		dev_err(priv->dev, "Bridge is not enabled\n");
		return -ENODEV;
	}

	if (msg->rx_len) {
		dev_warn(priv->dev, "MIPI rx is not supported\n");
		return -ENOTSUPP;
	}
	if (msg->tx_len > 8) {
		dev_warn(priv->dev, "Maximum 8 byte MIPI tx is supported\n");
		return -ENOTSUPP;
	}

	i = 3; // re-send x times (the transfer is non reliable)
	while (i--) {

		ret = mipi_dsi_create_packet(&packet, msg);
		if (ret) {
			dev_warn(priv->dev, "mipi_dsi_create_packet failed : %d\n", ret);
			return ret;
		}

		if (mipi_dsi_packet_format_is_short(msg->type)) {
	//		dev_warn(priv->dev, "short command\n");
			tc358768_write(priv, TC358768_DSICMD_TYPE,
					   (0x10 << 8) | (packet.header[0] & 0x3f));
			tc358768_write(priv, TC358768_DSICMD_WC, 0);
			tc358768_write(priv, TC358768_DSICMD_WD0,
					   (packet.header[2] << 8) | packet.header[1]);
		} else {
			int i;
	//		dev_warn(priv->dev, "long command: len=%d\n", (int)packet.payload_length);
			tc358768_write(priv, TC358768_DSICMD_TYPE,
					   (0x40 << 8) | (packet.header[0] & 0x3f));
			tc358768_write(priv, TC358768_DSICMD_WC, packet.payload_length);
			for (i = 0; i < packet.payload_length; i += 2) {
				u16 val = packet.payload[i];

				if (i + 1 < packet.payload_length)
					val |= packet.payload[i + 1] << 8;

				tc358768_write(priv, TC358768_DSICMD_WD0 + i, val);
			}
		}

		/* start transfer */
		tc358768_write(priv, TC358768_DSICMD_TX, 1);

		ret = tc358768_clear_error(priv);
		if (ret)
			dev_warn(priv->dev, "Software disable failed: %d\n", ret);
		else
			ret = packet.size;

		msleep(1);
	}

	return ret;
}


static enum drm_connector_status
tc358768_connector_detect(struct drm_connector *connector, bool force)
{
	return connector_status_connected;
}

static const struct mipi_dsi_host_ops tc358768_dsi_host_ops = {
	.attach = tc358768_dsi_host_attach,
	.detach = tc358768_dsi_host_detach,
	.transfer = tc358768_dsi_host_transfer,
};


static const struct drm_connector_funcs tc358768_connector_funcs = {
	.dpms = drm_atomic_helper_connector_dpms,
	.fill_modes = drm_helper_probe_single_connector_modes,
	.detect = tc358768_connector_detect,
	.destroy = drm_connector_cleanup,
	.reset = drm_atomic_helper_connector_reset,
	.atomic_duplicate_state = drm_atomic_helper_connector_duplicate_state,
	.atomic_destroy_state = drm_atomic_helper_connector_destroy_state,
};

static int tc358768_bridge_attach(struct drm_bridge *bridge)
{
#if LINUX_VERSION_CODE >= KERNEL_VERSION(4,11,0)
	struct tc358768_priv *priv = bridge_to_tc358768(bridge);

	if (!drm_core_check_feature(bridge->dev, DRIVER_ATOMIC)) {
		dev_err(priv->dev, "needs atomic updates support\n");
		return -ENOTSUPP;
	}

	return drm_bridge_attach(bridge->encoder, priv->output.bridge, bridge
#if LINUX_VERSION_CODE >= KERNEL_VERSION(5,7,0)
				 ,flags
#endif
	);
#else
	return 0;
#endif
}

#if LINUX_VERSION_CODE >= KERNEL_VERSION(4,17,0)
static enum drm_mode_status
tc358768_bridge_mode_valid(struct drm_bridge *bridge,
#if LINUX_VERSION_CODE >= KERNEL_VERSION(5,9,0)
			   const struct drm_display_info *info,
#endif
			   const struct drm_display_mode *mode)
{
	struct tc358768_priv *priv = bridge_to_tc358768(bridge);

	if (tc358768_calc_clocks(priv, mode))
		return MODE_CLOCK_RANGE;

	return MODE_OK;
}
#endif


static void tc358768_priv_disable(struct tc358768_priv *priv)
{
	int ret;


	/* set FrmStop */
	tc358768_update_bits(priv, TC358768_PP_MISC, BIT(15), BIT(15));

	/* wait at least for one frame */
	msleep(50);

	/* clear PP_en */
	tc358768_update_bits(priv, TC358768_CONFCTL, BIT(6), 0);

	/* set RstPtr */
	tc358768_update_bits(priv, TC358768_PP_MISC, BIT(14), BIT(14));

	ret = tc358768_clear_error(priv);
	if (ret)
		dev_warn(priv->dev, "Software disable failed: %d\n", ret);
}

static void tc358768_bridge_disable(struct drm_bridge *bridge)
{
	struct tc358768_priv *priv = bridge_to_tc358768(bridge);
	tc358768_priv_disable(priv);
}

static void tc358768_priv_post_disable(struct tc358768_priv *priv)
{
	tc358768_hw_disable(priv);
}

static void tc358768_bridge_post_disable(struct drm_bridge *bridge)
{
	struct tc358768_priv *priv = bridge_to_tc358768(bridge);
	tc358768_priv_post_disable(priv);
}

static int tc358768_calc_clocks(struct tc358768_priv *priv, const struct drm_display_mode *mode)
{

	u32 i;
	u32 check, checkMin, checkMax;
	u32 t17;
	u32 pllInputClk;
	u32 hsByteClk;
	u32 lineInitControl;
	u32 tlptxTimeCnt;
	u32 tclkPrepareCnt;
	u32 tclkZeroCnt;
	u32 tclkTrailCnt;
	u32 thsPrepareCnt;
	u32 thsZeroCnt;
	u32 twakeUpCnt;
	u32 tclkPostCnt;
	u32 thsTrailCnt;
	u32 rxTasureCnt;
	u32 txTaGoCnt;

    struct dsi_tx_params* dtxpar = &priv->dtxpar;

	dev_warn(priv->dev, "clocks and timings:\n");

	priv->bpp = 24;
	if (priv->dsi_pixel_format == MIPI_DSI_FMT_RGB565) {
		priv->bpp = 16;
	} else if (priv->dsi_pixel_format == MIPI_DSI_FMT_RGB666_PACKED) { //tightly packed
		priv->bpp = 18;
	}


	priv->dsiclk = (mode->clock * priv->bpp) / priv->dsi_lanes;
	dev_warn(priv->dev,"  dsiclk (synthetic): %d\n", priv->dsiclk);

	if (priv->dsiclk < 62500) {
		dev_err(priv->dev,"Error: DSI clock is too slow\n");
		return -1;
	} else
	if (priv->dsiclk >= 62500 && priv->dsiclk < 125000) {
		dtxpar->frs = 3;
	} else
	if (priv->dsiclk >= 125000 && priv->dsiclk < 250000) {
		dtxpar->frs = 2;
	} else
	if (priv->dsiclk >= 250000 && priv->dsiclk < 500000) {
		dtxpar->frs = 1;
	} else
	if (priv->dsiclk >= 500000 && priv->dsiclk <= 1000000) {
		dtxpar->frs = 0;
	} else {
		dev_warn(priv->dev,"Error: sythetic DSI clock is too high\n");
		return -2;
	}
	dev_warn(priv->dev,"  frs: %d\n", dtxpar->frs);

	//TODO - use different PRD values if the checks fail
	dtxpar->prd = 0;
	dev_warn(priv->dev,"  prd: %d\n", dtxpar->prd);

	// use pixel clock as the PLL source
	if (priv->refclk == 0) {
		pllInputClk =  priv->dsiclk / 4 / (dtxpar->prd + 1);
	} else 
	//use external crystal as the PLL source 
	{
		pllInputClk = priv->refclk / (dtxpar->prd + 1);
	}
	dev_warn(priv->dev,"  PLL input clock: %d\n", pllInputClk);

	//adjust the the dsiClk (raise it to a higher step)
	priv->dsiclk += (pllInputClk / 2);

	//calculate FBD as:
	dtxpar->fbd = ((priv->dsiclk * (1 << dtxpar->frs)) / pllInputClk) - 1;
	dev_warn(priv->dev,"  fbd: %d\n", dtxpar->fbd);

	// calculate the real dsi clock:
	priv->dsiclk = (pllInputClk * (dtxpar->fbd + 1)  * (dtxpar->prd + 1)) / (1 << dtxpar->frs);
	dev_warn(priv->dev,"  dsiclk (real): %d\n", priv->dsiclk);

	// calculate hsByteClk
	hsByteClk = priv->dsiclk / 8;
	if (hsByteClk > 125000) {
		dev_err(priv->dev,"Error: real DSI clock is too high\n");
		return -2;
	}
	dev_warn(priv->dev,"  HS byte clock: %d\n", hsByteClk);

	// REGISTER calculator
	// ==========================================================

	// calculate: lineInitControl ( reg 0x210 [15:0])
	// ----------------------------------------------------------
	lineInitControl = (((180 * hsByteClk) / 2) - 500) / 1000;
	dev_warn(priv->dev, "  Line init control: %d\n", lineInitControl);


	// calculate: tlptxTimeCnt (reg 0x214 [10:0])
	// ----------------------------------------------------------
	//check must be >= 50, but other fields depend on it so it may require adjustments!
	tlptxTimeCnt = 0;
	for (i = 1; i < 2047; i++) {
		//check = (1/32.216)* (1+1) * 1000  ==> 62 (is > 50) OK! use 1
		check = ((i+1) * 1000000) / hsByteClk;
		if (check >= 50) {
			tlptxTimeCnt = i;
			break;
		}
	}
	if (0 == tlptxTimeCnt) {
		dev_err(priv->dev,"Error: could not calculate Tlptx Time Cnt\n");
		return -3;
	}
	t17 = check;
	dev_warn(priv->dev,"  TLPTX TIME CNT: %d (check=%d)\n", tlptxTimeCnt, check);

	// calculate tclkPrepareCnt (reg 0x218 [6:0])
	// ----------------------------------------------------------
	//check must be >= 43 && <= 90
	tclkPrepareCnt = 0xFF;
    for (i = 0; i < 128; i++) {
		check = ((i + 1) * 1000000) / hsByteClk;
		if (check >= 43 && check <= 90) {
			tclkPrepareCnt = i;
			break;
		}
	}
	if (0xFF == tclkPrepareCnt) {
		dev_err(priv->dev,"Error: could not calculate Tclk Prepare Cnt\n");
		return -4;
	}
	dev_warn(priv->dev,"  TCLK PREPARE CNT: %d (check=%d)\n", tclkPrepareCnt, check);

	// calculate tclkZeroCnt (reg 0x218 [15:8])
	// ----------------------------------------------------------
	//check must be >= 300
	tclkZeroCnt = 0xFFFF;
	for (i = 0; i < 256; i++) {
		check = ((((i *1000) + 2500)* 1000) / (hsByteClk) + ((3500000) / priv->dsiclk));
		if (check >= 300) {
			tclkZeroCnt = i;
			break;
		}
	}
	if (0xFFFF == tclkZeroCnt) {
		dev_err(priv->dev,"Error: could not calculate Tclk Zero Cnt\n");
		return -5;
	}
	dev_warn(priv->dev,"  TCLK ZERO CNT: %d (check=%d)\n", tclkZeroCnt, check);

	// calculate tclkTrailCnt (reg 0x21C [7:0])
	// ----------------------------------------------------------
	// only when DSI clock is not continuous (disabled during Low Power state)
	if (priv->dsi_mode_flags & MIPI_DSI_CLOCK_NON_CONTINUOUS) {
		tclkTrailCnt = 0xFFFF;
		for (i = 0; i < 255; i++) {
			check = (((i + 5)* 1000000)/hsByteClk) - ((2500000)/priv->dsiclk);
			if (check >= (60+5) && check <= (121 - 5)) {
				tclkTrailCnt = i;
				break;
			}
		}
		if (0xFFFF == tclkTrailCnt) {
			dev_err(priv->dev,"Error: could not calculate Tclk Trail Cnt. Disable Non-continuous DSI clock flag.\n");
			return -6;
		}
		
	} else {
		tclkTrailCnt = 0;
		check = (((tclkTrailCnt + 5)* 1000000)/hsByteClk) - ((2500000)/priv->dsiclk);
	}
	dev_warn(priv->dev,"  TCLK TRAIL CNT: %d (check=%d)\n", tclkTrailCnt, check);

	// calculate thsPrepareCnt (reg 0x220 [6:0])
	// ----------------------------------------------------------
	//check must be >= 60 && < 103
	checkMin = 40 + ((4 * 1000000)/priv->dsiclk);
	checkMax = 85 + ((6 * 1000000)/priv->dsiclk);
	thsPrepareCnt = 0xFFFF;
	for (i = 0; i < 128; i++) {
		check =((i+1)*1000000)/hsByteClk;
		if ((check >= checkMin + 5) && (check <= checkMax - 5)) {
			thsPrepareCnt = i;
			break;
		}
	}
	if (0xFFFF == thsPrepareCnt) {
		dev_err(priv->dev,"Error: could not calculate Ths Prepare Cnt\n");
		return -7;
	}
	dev_warn(priv->dev,"  THS PREPARE CNT: %d (check=%d min=%d max=%d)\n", thsPrepareCnt, check, checkMin, checkMax);


	// calculate thsZeroCnt (reg 0x220 [14:8])
	// ----------------------------------------------------------
	//check must be >= 145 + (1000*10)/dsiClk
	checkMin = 145 + (10000000 / priv->dsiclk);
	thsZeroCnt = 0xFFFF;
	for (i = 0; i < 128; i++) {
		check = (4 * 1000000/hsByteClk) + ((i+7)*1000000/hsByteClk) + (11*1000000/priv->dsiclk);
		if (check >= checkMin + 5) {
			thsZeroCnt = i;
			break;
		}
	}
	if (0xFFFF == thsZeroCnt) {
		dev_err(priv->dev,"Error: could not calculate Ths Zero Cnt\n");
		return -8;
	}
	dev_warn(priv->dev,"  THS ZERO CNT: %d (check=%d, min=%d)\n", thsZeroCnt, check, checkMin);


	// calculate twakeUpCnt (reg 0x224 [15:0])
	// ----------------------------------------------------------
	checkMin = 1000;
	twakeUpCnt = 0xFFFFF;
	for (i = 100; i < 65500; i+= 100) {
		check =((tlptxTimeCnt+1)*(i+1)*1000)/ hsByteClk;
		if (check >= checkMin) {
			twakeUpCnt = i;
			break;
		}
	}
	if (0xFFFFF == twakeUpCnt) {
		dev_err(priv->dev,"Error: could not calculate TWAKE-UP CNT\n");
		return -9;
	}
	dev_warn(priv->dev,"  TWAKE-UP CNT: %d (check=%d, min=%d)\n", twakeUpCnt, check, checkMin);


	// calculate tclkPostCnt (reg 0x228 [10:0])
	// ----------------------------------------------------------
	// only when DSI clock is not continuous (disabled during Low Power state)
	checkMin = 60 + ((52*1000000) /priv->dsiclk);
	if (priv->dsi_mode_flags & MIPI_DSI_CLOCK_NON_CONTINUOUS) {
		tclkPostCnt = 0xFFFF;
		for (i = 0; i < 2048; i++) {
			check = ((((i * 1000) + 3500)*1000)/hsByteClk) + (2500000/priv->dsiclk);
			if (check >= checkMin + 5) {
				tclkPostCnt = i;
				break;
			}
		}
		if (0xFFFF == tclkPostCnt) {
			dev_err(priv->dev,"Error: could not calculate Tclk Trail Cnt. Disable Non-continuous DSI clock flag.\n");
			return -10;
		}
		
	} else {
		tclkPostCnt = 0;
		check = ((((tclkPostCnt * 1000) + 3500)*1000)/hsByteClk) + (2500000/priv->dsiclk);
	}
	dev_warn(priv->dev,"  TCLK POST CNT: %d (check=%d, min=%d)\n", tclkPostCnt, check, checkMin);


	// calculate thsTrailCnt (reg 0x22C [3:0])
	// ----------------------------------------------------------
	checkMin = MAX(8000000/priv->dsiclk, 60 + (4000000/priv->dsiclk));
	checkMax = 105 + (12000000/priv->dsiclk) - 30;
	thsTrailCnt = 0xFFFFF;
	for (i = 0; i < 15; i++) {
		check = (((i + 5) * 1000000) / hsByteClk) - (11000000 / priv->dsiclk);
		if ((check >= checkMin + 5) && (check <= checkMax - 5) ) {
			thsTrailCnt = i;
			break;
		}
	}
	if (0xFFFFF == thsTrailCnt) {
		dev_err(priv->dev,"Error: could not calculate THS TRAIL CNT\n");
		return -11;
	}
	dev_warn(priv->dev,"  THS TRAIL CNT: %d (check=%d, min=%d max=%d)\n", thsTrailCnt, check, checkMin, checkMax);

	// calculate rxTasureCnt (0x23C [10:0])
	// ----------------------------------------------------------
	checkMin = t17;
	checkMax = t17 * 2;
	rxTasureCnt = 0xFFFFF;
	for (i = 0; i < 128; i++) {
		u32 check1 = ((i + 2) * 1000000) / hsByteClk;
		check = ((i + 3) * 1000000) / hsByteClk;
		if ((check1 >= checkMin) && (check <= checkMax) ) {
			rxTasureCnt = i;
			break;
		}
	}
	if (0xFFFFF == thsTrailCnt) {
		dev_err(priv->dev,"Error: could not calculate RX ASSURE CNT\n");
		return -11;
	}
	dev_warn(priv->dev,"  RX ASSURE CNT: %d (check=%d, min=%d max=%d)\n", rxTasureCnt, check, checkMin, checkMax);

	// calculate txTaGoCnt (0x23C [26:16])
	// ----------------------------------------------------------
	checkMin = t17 * 4;
	txTaGoCnt = 0xFFFFF;
	for (i = 0; i < 128; i++) {
		check = ((i + 1) * 4000000) / hsByteClk;
		if (check == checkMin) {
			txTaGoCnt = i;
			break;
		}
	}
	if (0xFFFFF == txTaGoCnt) {
		dev_err(priv->dev,"Error: could not calculate TX TA GO CNT\n");
		return -11;
	}
	dev_warn(priv->dev,"  TX TA GO CNT: %d (check=%d, min=%d)\n", txTaGoCnt, check, checkMin);


	dtxpar->pllInputClk = pllInputClk;
	dtxpar->hsByteClk = hsByteClk;
	dtxpar->lineInitControl = lineInitControl;
	dtxpar->tlptxTimeCnt = tlptxTimeCnt;
	dtxpar->tclkPrepareCnt = tclkPrepareCnt;
	dtxpar->tclkZeroCnt = tclkZeroCnt;
	dtxpar->tclkTrailCnt = tclkTrailCnt;
	dtxpar->thsPrepareCnt = thsPrepareCnt;
	dtxpar->thsZeroCnt = thsZeroCnt;
	dtxpar->twakeUpCnt = twakeUpCnt;
	dtxpar->tclkPostCnt = tclkPostCnt;
	dtxpar->thsTrailCnt = thsTrailCnt;
	dtxpar->rxTasureCnt = rxTasureCnt;
	dtxpar->txTaGoCnt = txTaGoCnt;
	dtxpar->valid = 1;

	return tc358768_clear_error(priv);
}

static void tc358768_priv_pre_enable(struct tc358768_priv *priv, 
		const struct drm_display_mode *mode)
{
	u32 val;
	int ret, i;
	u32 vspw;
	u32 vbp;
	u32 hspw;
	u32 hbp;
	u8 debug = 0;
	struct dsi_tx_params* dtxpar = &priv->dtxpar;

	tc358768_hw_enable(priv);

	ret = tc358768_calc_clocks(priv, mode);
	if (ret) {
		dev_err(priv->dev, "Clock setup failed: %d\n", ret);
		tc358768_hw_disable(priv);
		return;
	}

	priv->error = 0;

	vspw = mode->vsync_end - mode->vsync_start;
	vbp = mode->vtotal - mode->vsync_end;
	hspw = mode->hsync_end - mode->hsync_start;
	hbp = mode->htotal - mode->hsync_end;

	printk("Register dump: \n");
	REM("Software Reset");
	WR(TC358768_SYSCTL, 0x0001,"SYSctl, S/W Reset");
	DELAYUS(10);
	WR(TC358768_SYSCTL, 0x0000,"SYSctl, S/W Reset release");

	REM("TC358768XBG PLL,Clock Setting");
	WR(TC358768_PLLCTL0, (dtxpar->prd << 12) | dtxpar->fbd  ,"PLL Control Register 0 (PLL_PRD,PLL_FBD)");
	WR(TC358768_PLLCTL1, (dtxpar->frs << 10) | (2 << 8) | 0x3 ,"PLL_FRS,PLL_LBWS, PLL oscillation enable");
	DELAYUS(1000);
	WR(TC358768_PLLCTL1, (dtxpar->frs << 10) | (2 << 8) | 0x13,"PLL_FRS,PLL_LBWS, PLL clock out enable");

	REM("TC358768XBG DPI Input Control");
	WR(TC358768_VSDLY,	priv->vs_delay, "FIFO Control Register"); //VSDly register

	REM("TC358768XBG D-PHY Setting");
	WR(TC358768_CLW_CNTRL, 0x0000, "D-PHY Clock lane enable");
	for (i = 0; i < priv->dsi_lanes; i++) {
		WR(TC358768_D0W_CNTRL + (i * 4), 0x0000, "D-PHY Data lane enable");
	}

	WR(TC358768_CLW_DPHYCONTTX, 0x0002, "D-PHY Clock lane control");
	for (i = 0; i < priv->dsi_lanes; i++) {
		WR(TC358768_D0W_DPHYCONTTX + (i * 4), 0x0002, "D-PHY Data lane enable");
	}

	REM("TC358768XBG DSI-TX PPI Control");
	WR(TC358768_LINEINITCNT, dtxpar->lineInitControl,"LINEINITCNT");
	WR(TC358768_LPTXTIMECNT, dtxpar->tlptxTimeCnt,"LPTXTIMECNT");
	WR(TC358768_TCLK_HEADERCNT,(dtxpar->tclkZeroCnt << 8) | dtxpar->tclkPrepareCnt ,"TCLK_HEADERCNT");

	if (priv->dsi_mode_flags & MIPI_DSI_CLOCK_NON_CONTINUOUS) {
		WR(TC358768_TCLK_TRAILCNT, dtxpar->tclkTrailCnt,"TCLK_TRAILCNT");
	}
	WR(TC358768_THS_HEADERCNT, dtxpar->tclkPrepareCnt | (dtxpar->thsZeroCnt << 8) , "THS_HEADERCNT");
	WR(TC358768_TWAKEUP, dtxpar->twakeUpCnt, "TWAKEUPCNT");

	if (priv->dsi_mode_flags & MIPI_DSI_CLOCK_NON_CONTINUOUS) {
		WR(TC358768_TCLK_POSTCNT, dtxpar->tclkPostCnt, "TCLK_POSTCNT");
	}
	WR(TC358768_THS_TRAILCNT,dtxpar->thsTrailCnt, "THS_TRAILCNT");
	WR(TC358768_HSTXVREGCNT, 0x0005, "HSTXVREGCNT");

	WR(TC358768_HSTXVREGEN,  (((1 << priv->dsi_lanes) -1 ) << 1) | 1,"HSTXVREGEN enable");

	WR(TC358768_TXOPTIONCNTRL, (priv->dsi_mode_flags & MIPI_DSI_CLOCK_NON_CONTINUOUS) ? 0 : 1 , "DSI clock Enable/Disable during LP");

	val = (((u32) dtxpar->txTaGoCnt) << 16) | dtxpar->rxTasureCnt;
	
	WR(TC358768_BTACNTRL1,val,"BTACNTRL1");

	WR(TC358768_STARTCNTRL, 0x0001, "STARTCNTRL");

	REM("TC358768XBG DSI-TX Timing Control");
	val = 1;
	if (priv->dsi_mode_flags & MIPI_DSI_MODE_VIDEO_SYNC_PULSE) {
		val = 0;
	}
	WR(TC358768_DSI_EVENT, val, "Sync Pulse/Sync Event mode setting");

	if (priv->dsi_mode_flags & MIPI_DSI_MODE_VIDEO_SYNC_PULSE) {
		val = vspw;
	} else {
		val = vspw + vbp;
	}
	WR(TC358768_DSI_VSW, val, "V Control Register1");
	WR(TC358768_DSI_VBPR, vbp, "V Control Register2");
	WR(TC358768_DSI_VACT, mode->vdisplay, "V Control Register3");

	{
		if (priv->hporch_override1) {
			val = priv->hporch_override1;
		}
		else  if (priv->dsi_mode_flags & MIPI_DSI_MODE_VIDEO_SYNC_PULSE) {
			val = ((hspw * dtxpar->hsByteClk * priv->dsi_lanes) + (mode->clock / 2)) / mode->clock; //round up
		}
		else {
			val = (((hspw + hbp) * dtxpar->hsByteClk * priv->dsi_lanes) + (mode->clock / 2)) / mode->clock; //round up
		}
		WR(TC358768_DSI_HSW, val, "H Control Register1");


		if (priv->hporch_override2) {
			val = priv->hporch_override2;
		} else {
			val = ((hbp * dtxpar->hsByteClk * priv->dsi_lanes) + (mode->clock / 2)) / mode->clock; //round up
		}
		WR(TC358768_DSI_HBPR, val, "H Control Register2");

		val = mode->hdisplay;
		if (priv->bpp == 18) { //tightly packed RGB666
			if (val & 0x3) {
				val = ((val >> 2) + 1) << 2;
			}
		}
		val = (val * priv->bpp) / 8;
		WR(TC358768_DSI_HACT, val, "H Control Register3");
	}

	WR(0x0518, 0001, "DSI Start");
	WR(0x051A, 0000, "");


	/* Configure DSI_Control register */
	val = TC358768_DSI_CONFW_MODE_CLR | TC358768_DSI_CONFW_ADDR_DSI_CONTROL;
	val |= TC358768_DSI_CONTROL_TXMD | TC358768_DSI_CONTROL_HSCKMD |
	       0x3 << 1 | TC358768_DSI_CONTROL_EOTDIS;
	tc358768_write(priv, TC358768_DSI_CONFW, val);

	val = TC358768_DSI_CONFW_MODE_SET | TC358768_DSI_CONFW_ADDR_DSI_CONTROL;
	val |= (priv->dsi_lanes - 1) << 1;

	if (!(priv->dsi_mode_flags & MIPI_DSI_MODE_LPM))
		val |= TC358768_DSI_CONTROL_TXMD;

	if (!(priv->dsi_mode_flags & MIPI_DSI_CLOCK_NON_CONTINUOUS))
		val |= TC358768_DSI_CONTROL_HSCKMD;

	if (priv->dsi_mode_flags & MIPI_DSI_MODE_NO_EOT_PACKET)
		val |= TC358768_DSI_CONTROL_EOTDIS;

	tc358768_write(priv, TC358768_DSI_CONFW, val);

	val = TC358768_DSI_CONFW_MODE_CLR | TC358768_DSI_CONFW_ADDR_DSI_CONTROL;
	val |= TC358768_DSI_CONTROL_DIS_MODE; /* DSI mode */
	tc358768_write(priv, TC358768_DSI_CONFW, val);

	REM("Host: RGB(DPI) input start");
	val = (priv->bpp == 24) ? (3<<4) : (priv->bpp == 16) ? (5<< 4) : (4<<4);
	val |=  (priv->bpp == 18 && priv->dsi_pixel_format == MIPI_DSI_FMT_RGB666) ?  15 : 7;  //loosely packed RGB666
	WR(TC358768_DATAFMT, val, "DSI-TX Format setting");
	val = (priv->bpp == 16) ? 0xe : (priv->bpp == 24) ? 0x3e : (priv->dsi_pixel_format == MIPI_DSI_FMT_RGB666) ? 0x2e : 0x1e;
	WR(TC358768_DSITX_DT, val, "DSI-TX Pixel stream packet Data Type setting");
	val = (mode->flags & DRM_MODE_FLAG_PHSYNC ) ? 1 : 0;

	WR(TC358768_PP_MISC, val, "HSYNC Polarity");
	val = ((mode->flags & DRM_MODE_FLAG_PHSYNC ) ? (1 << 5) : 0);
	val |=  0  ; //DE polarity: assume Active high, otherwise add (1 << 4)
	val |= (1 << 2); // I2C auto increment
	val |= priv->rgb_mapping_mode;  //Input RGB mapping mode
	val |= (1 << 6);
	WR(TC358768_CONFCTL, val, "Configuration Control Register");

}

static void tc358768_bridge_pre_enable(struct drm_bridge *bridge) {
	struct tc358768_priv *priv = bridge_to_tc358768(bridge);
	const struct drm_display_mode *mode;

	mode = &bridge->encoder->crtc->state->adjusted_mode;
	tc358768_priv_pre_enable(priv, mode);
}

static void tc358768_priv_enable(struct tc358768_priv *priv)
{
	int ret;
	printk("tc358768_bridge_enable\n");
	if (!priv->enabled) {
		dev_err(priv->dev, "Bridge is not enabled\n");
		return;
	}


	/* clear FrmStop and RstPtr */
	tc358768_update_bits(priv, TC358768_PP_MISC, 0x3 << 14, 0);

	/* set PP_en */
	tc358768_update_bits(priv, TC358768_CONFCTL, BIT(6), BIT(6));

	ret = tc358768_clear_error(priv);
	if (ret) {
		dev_err(priv->dev, "Bridge enable failed: %d\n", ret);
		tc358768_priv_disable(priv);
		tc358768_priv_post_disable(priv);
	}
}

static void tc358768_bridge_enable(struct drm_bridge *bridge)
{
	struct tc358768_priv *priv = bridge_to_tc358768(bridge);
	tc358768_priv_enable(priv);
}

static int i2c_bridge_command(struct tc358768_priv *priv, int command, void* arg)
{
	int ret = 0;
	struct i2c_driver* driver;
	if (NULL == priv || priv->output.i2c_bridge == NULL) {
		return 0;
	}
	dev_warn(priv->dev, "OUT bridge driver command called: %d\n", command);
	if (priv->output.i2c_bridge->dev.driver) {
		driver = to_i2c_driver(priv->output.i2c_bridge->dev.driver);
		if (driver == NULL) {
			dev_warn(priv->dev,"OUT bridge driver not found\n");
		} else {
			if (driver->command) {
				ret = driver->command(priv->output.i2c_bridge, command, arg);
				dev_warn(priv->dev,"OUT bridge driver command result: %d\n", ret);
			} else {
				dev_warn(priv->dev,"OUT bridge driver command not found\n");
			}
		}
	} else {
		dev_warn(priv->dev,"OUT bridge has no driver\n");
	}
	return ret;
}


static int tc358768_i2c_command(struct i2c_client *client, unsigned int cmd, void *arg)
{
	struct drm_display_mode* mode;
	struct tc358768_priv *priv = i2c_get_clientdata(client);

	if (priv == NULL) {
		printk("tc358768_i2c_command: probe has not finished.\n");
		return -EPROBE_DEFER;
	}

	printk("tc358768_i2c_command: %d\n", cmd);
	switch (cmd) {
		case 1 : //eanble
			if (NULL == arg) {
				dev_err(priv->dev, "Bridge command ON failed - no display mode\n");
			} else {
				mode = (struct drm_display_mode*) arg;

				tc358768_priv_pre_enable(priv, mode);
				tc358768_priv_enable(priv);
				//notify the output bridge
				i2c_bridge_command(priv, cmd, arg);
			}
			break;
		case 0 : //disable
			//notify the output bridge
			i2c_bridge_command(priv, cmd, arg);
			tc358768_priv_disable(priv);
			tc358768_priv_post_disable(priv);
			break;
	}
	return 0;
}

static const struct drm_bridge_funcs tc358768_bridge_funcs = {
	.attach = tc358768_bridge_attach,
#if LINUX_VERSION_CODE >= KERNEL_VERSION(4,13,0)
	.mode_valid = tc358768_bridge_mode_valid,
#endif
	.pre_enable = tc358768_bridge_pre_enable,
	.enable = tc358768_bridge_enable,
	.disable = tc358768_bridge_disable,
	.post_disable = tc358768_bridge_post_disable,
};

#if LINUX_VERSION_CODE >= KERNEL_VERSION(4,17,0)
static const struct drm_bridge_timings default_tc358768_timings = {
	.input_bus_flags = DRM_BUS_FLAG_PIXDATA_SAMPLE_POSEDGE
		 | DRM_BUS_FLAG_SYNC_SAMPLE_NEGEDGE
		 | DRM_BUS_FLAG_DE_HIGH,
};
#endif

static bool tc358768_is_reserved_reg(unsigned int reg)
{
	switch (reg) {
	case 0x114 ... 0x13f:
	case 0x200:
	case 0x20c:
	case 0x400 ... 0x408:
	case 0x41c ... 0x42f:
		return true;
	default:
		return false;
	}
}

static bool tc358768_writeable_reg(struct device *dev, unsigned int reg)
{
	if (tc358768_is_reserved_reg(reg))
		return false;

	switch (reg) {
	case TC358768_CHIPID:
	case TC358768_FIFOSTATUS:
	case TC358768_DSITXSTATUS ... (TC358768_DSITXSTATUS + 2):
	case TC358768_DSI_CONTROL ... (TC358768_DSI_INT_ENA + 2):
	case TC358768_DSICMD_RDFIFO ... (TC358768_DSI_ERR_HALT + 2):
		return false;
	default:
		return true;
	}
}

static bool tc358768_readable_reg(struct device *dev, unsigned int reg)
{
	if (tc358768_is_reserved_reg(reg))
		return false;

	switch (reg) {
	case TC358768_STARTCNTRL:
	case TC358768_DSI_CONFW ... (TC358768_DSI_CONFW + 2):
	case TC358768_DSI_INT_CLR ... (TC358768_DSI_INT_CLR + 2):
	case TC358768_DSI_START ... (TC358768_DSI_START + 2):
	case TC358768_DBG_DATA:
		return false;
	default:
		return true;
	}
}

static const struct regmap_config tc358768_regmap_config = {
	.name = "tc358768",
	.reg_bits = 16,
	.val_bits = 16,
	.max_register = TC358768_DSI_HACT,
	.cache_type = REGCACHE_NONE,
	.writeable_reg = tc358768_writeable_reg,
	.readable_reg = tc358768_readable_reg,
	.reg_format_endian = REGMAP_ENDIAN_BIG,
	.val_format_endian = REGMAP_ENDIAN_BIG,
};

static const struct i2c_device_id tc358768_i2c_ids[] = {
	{ "tc358768", 0 },
	{ "tc358778", 0 },
	{ }
};
MODULE_DEVICE_TABLE(i2c, tc358768_i2c_ids);

static const struct of_device_id tc358768_of_ids[] = {
	{ .compatible = "toshiba,tc358768", },
	{ .compatible = "toshiba,tc358778", },
	{ }
};
MODULE_DEVICE_TABLE(of, tc358768_of_ids);

static int tc358768_get_regulators(struct tc358768_priv *priv)
{
	int i, ret;

	for (i = 0; i < ARRAY_SIZE(priv->supplies); ++i)
		priv->supplies[i].supply = tc358768_supplies[i];

	ret = devm_regulator_bulk_get(priv->dev, ARRAY_SIZE(priv->supplies),
				      priv->supplies);
	if (ret < 0)
		dev_err(priv->dev, "failed to get regulators: %d\n", ret);

	return ret;
}

static int tc358768_i2c_probe(struct i2c_client *client,
			      const struct i2c_device_id *id)
{
	struct tc358768_priv *priv;
	struct device *dev = &client->dev;
	struct device_node *np = dev->of_node;
	int ret;
	u32 val;
	struct clk * refclk;
	const char *name;

	printk("tc358768_i2c_probe\n");
	if (!np)
		return -ENODEV;
	i2c_set_clientdata(client, NULL);

	priv = devm_kzalloc(dev, sizeof(*priv), GFP_KERNEL);
	if (!priv)
		return -ENOMEM;

	//dev_set_drvdata(dev, priv);
	priv->dev = dev;

	ret = tc358768_get_regulators(priv);
	if (ret)
		return ret;

	refclk = devm_clk_get(dev, "refclk");
	if (IS_ERR(refclk)) {
		priv->refclk = 0;
	} else {
		priv->refclk = clk_get_rate(refclk);
	}

	/*
	 * RESX is low active, to disable tc358768 initially (keep in reset)
	 * the gpio line must be LOW. This is the ASSERTED state of
	 * GPIO_ACTIVE_LOW (GPIOD_OUT_HIGH == ASSERTED).
	 */
	priv->reset_gpio  = devm_gpiod_get_optional(dev, "reset",
						    GPIOD_OUT_HIGH);
	if (IS_ERR(priv->reset_gpio)) {
		priv->reset_gpio_exists = 0;
		dev_warn(dev, "No reset gpio defined\n");
	} else {
		priv->reset_gpio_exists = 1;
	}

	priv->regmap = devm_regmap_init_i2c(client, &tc358768_regmap_config);
	if (IS_ERR(priv->regmap)) {
		dev_err(dev, "Failed to init regmap\n");
		return PTR_ERR(priv->regmap);
	}

	/* output bridge */
	{
		struct device_node *bridge = of_parse_phandle(np, "out-bridge", 0);
		if (bridge) {
			priv->output.i2c_bridge = of_find_i2c_device_by_node(bridge);
			of_node_put(bridge);
			dev_warn(dev, "custom output-bridge found\n");
		} else {
			priv->output.i2c_bridge = NULL;
			dev_warn(dev, "custom output-bridge not defined\n");
		}
	}
	ret = of_property_read_u32(np, "dsi-lanes", &val);
	if (ret) {
		priv->dsi_lanes = 4;
	} else {
		priv->dsi_lanes = val;
	}

	ret = of_property_read_u32(np, "rgb-mapping-mode", &val);
	if (ret) {
		priv->rgb_mapping_mode = 0;
	} else {
		priv->rgb_mapping_mode = val;
	}

	ret = of_property_read_u32(np, "vs-delay", &val);
	if (ret) {
		priv->vs_delay = 10;
	} else {
		priv->vs_delay = val;
	}

	ret = of_property_read_u32(np, "hporch-override1", &val);
	if (ret) {
		priv->hporch_override1 = 0;
	} else {
		priv->hporch_override1 = val;
	}

	ret = of_property_read_u32(np, "hporch-override2", &val);
	if (ret) {
		priv->hporch_override2 = 0;
	} else {
		priv->hporch_override2 = val;
	}


	if (of_property_read_string(np, "dsi-pixel-format", &name))
		priv->dsi_pixel_format = MIPI_DSI_FMT_RGB888;
	else {
		if (!strncmp("RGB565", name, 6)) {
			priv->dsi_pixel_format = MIPI_DSI_FMT_RGB565;
		} else if (!strncmp("RGB666", name, 6)) { /* losely packed to 24 bits */
			priv->dsi_pixel_format = MIPI_DSI_FMT_RGB666;
		} else if (!strncmp("RGB666P", name, 7)) { /* tightly packed to 18 bits */
			priv->dsi_pixel_format = MIPI_DSI_FMT_RGB666_PACKED;
		} else if (!strncmp("RGB888", name, 6)) {
			priv->dsi_pixel_format = MIPI_DSI_FMT_RGB888;
		}
		else {
			dev_warn(dev, "unknown pixel format. Using RGB888\n");
			priv->dsi_pixel_format = MIPI_DSI_FMT_RGB888;
		}
	}
	priv->dsi_mode_flags = MIPI_DSI_MODE_VIDEO; 
	if (of_property_read_bool(np, "dsi-flag-sync-pulse")) {
		priv->dsi_mode_flags |=  MIPI_DSI_MODE_VIDEO_SYNC_PULSE;
	}
	if (of_property_read_bool(np, "dsi-flag-clock-non-continuous")) {
		priv->dsi_mode_flags |=  MIPI_DSI_CLOCK_NON_CONTINUOUS;
	}
	if (of_property_read_bool(np, "dsi-flag-eot-packet")) {
		priv->dsi_mode_flags |=  MIPI_DSI_MODE_EOT_PACKET;
	}
	printk("DISP: lanes =%d\n", priv->dsi_lanes);

	priv->dsi_host.dev = dev;
	priv->dsi_host.ops = &tc358768_dsi_host_ops;

	priv->bridge.funcs = &tc358768_bridge_funcs;


#if LINUX_VERSION_CODE >= KERNEL_VERSION(4,17,0)
	priv->bridge.timings = &default_tc358768_timings;
#endif
	priv->bridge.of_node = np;


	ret = mipi_dsi_host_register(&priv->dsi_host);

	/* notify the bridge the dsi host is registered */
	i2c_bridge_command(priv, 0xD4, &priv->dsi_host);

	i2c_set_clientdata(client, priv);
	printk("tc358768_i2c_probe done %d\n", ret);

	return ret;

}

static int tc358768_i2c_remove(struct i2c_client *client)
{
	struct tc358768_priv *priv = i2c_get_clientdata(client);

	mipi_dsi_host_unregister(&priv->dsi_host);

	return 0;
}

static struct i2c_driver tc358768_driver = {
	.driver = {
		.name = "tc358768",
		.of_match_table = tc358768_of_ids,
	},
	.id_table = tc358768_i2c_ids,
	.probe = tc358768_i2c_probe,
	.remove	= tc358768_i2c_remove,
	.command = tc358768_i2c_command,
};
module_i2c_driver(tc358768_driver);

MODULE_AUTHOR("Peter Ujfalusi <peter.ujfalusi@ti.com>");
MODULE_DESCRIPTION("TC358768AXBG/TC358778XBG DSI bridge");
MODULE_LICENSE("GPL v2");
