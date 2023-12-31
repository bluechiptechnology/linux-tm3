/*
 * acx00.c  --  ACX00 ALSA Soc Audio Codec driver
 *
 * (C) Copyright 2010-2016 Allwinnertech Technology., Ltd.
 *
 * Author: Wolfgang Huang <huangjinhui@allwinner.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 */

#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/firmware.h>
#include <linux/delay.h>
#include <linux/pm.h>
#include <linux/i2c.h>
#include <linux/clk.h>
#include <linux/of_gpio.h>
#include <linux/sunxi-gpio.h>
#include <linux/of_device.h>
#include <linux/of_platform.h>
#include <linux/debugfs.h>
#include <linux/slab.h>
#include <linux/mfd/acx00-mfd.h>
#include <sound/core.h>
#include <sound/pcm.h>
#include <sound/pcm_params.h>
#include <sound/soc.h>
#include <sound/initval.h>
#include <sound/tlv.h>
#include <linux/workqueue.h>

#include "acx00.h"


#define ACX00_DEF_VOL		0x9F9F
#undef ACX00_DAPM_LINEOUT

struct acx00_priv {
	struct acx00 *acx00;	/* parent mfd device struct */
	struct snd_soc_codec *codec;
	struct clk *clk;
	unsigned int sample_rate;
	unsigned int fmt;
	unsigned int enable;
	unsigned int spk_gpio;
	bool spk_gpio_used;
	int spk_gpio_state; /* state before suspend */
	struct mutex mutex;
	struct delayed_work spk_work;
	struct delayed_work resume_work;
};

struct sample_rate {
	unsigned int samplerate;
	unsigned int rate_bit;
};

static const struct sample_rate sample_rate_conv[] = {
	{44100, 7},
	{48000, 8},
	{8000, 0},
	{32000, 6},
	{22050, 4},
	{24000, 5},
	{16000, 3},
	{11025, 1},
	{12000, 2},
	{192000, 10},
	{96000, 9},
};

void __iomem *io_stat_addr;

static const DECLARE_TLV_DB_SCALE(i2s_mixer_adc_tlv, -600, 600, 1);
static const DECLARE_TLV_DB_SCALE(i2s_mixer_dac_tlv, -600, 600, 1);
static const DECLARE_TLV_DB_SCALE(dac_mixer_adc_tlv, -600, 600, 1);
static const DECLARE_TLV_DB_SCALE(dac_mixer_dac_tlv, -600, 600, 1);
static const DECLARE_TLV_DB_SCALE(line_out_tlv, -450, 150, 0);
static const DECLARE_TLV_DB_SCALE(mic_out_tlv, -450, 150, 0);
static const DECLARE_TLV_DB_SCALE(phoneout_tlv, -450, 150, 0);
static const DECLARE_TLV_DB_SCALE(adc_input_tlv, -450, 150, 0);
static const DECLARE_TLV_DB_SCALE(lineout_tlv, -4800, 150, 1);
static const unsigned int mic_boost_tlv[] = {
	TLV_DB_RANGE_HEAD(2),
	0, 0, TLV_DB_SCALE_ITEM(0, 0, 0),
	1, 7, TLV_DB_SCALE_ITEM(2400, 300, 0),
};

static const struct snd_kcontrol_new acx00_codec_controls[] = {
	SOC_DOUBLE_TLV("I2S Mixer ADC Volume", AC_I2S_MIXER_GAIN,
			I2S_MIXERL_GAIN_ADC, I2S_MIXERR_GAIN_ADC,
			0x1, 0, i2s_mixer_adc_tlv),
	SOC_DOUBLE_TLV("I2S Mixer DAC Volume", AC_I2S_MIXER_GAIN,
			I2S_MIXERR_GAIN_DAC, I2S_MIXERR_GAIN_DAC,
			0x1, 0, i2s_mixer_dac_tlv),
	SOC_DOUBLE_TLV("DAC Mixer ADC Volume", AC_DAC_MIXER_GAIN,
			DAC_MIXERL_GAIN_ADC, DAC_MIXERR_GAIN_ADC,
			0x1, 0, dac_mixer_adc_tlv),
	SOC_DOUBLE_TLV("DAC Mxier DAC Volume", AC_DAC_MIXER_GAIN,
			DAC_MIXERL_GAIN_DAC, DAC_MIXERR_GAIN_DAC,
			0x1, 0, dac_mixer_dac_tlv),
	//SOC_SINGLE_TLV("Line Out Mixer Volume", AC_OUT_MIXER_CTL,
	//		OUT_MIXER_LINE_VOL, 0x7, 0, line_out_tlv),
	SOC_DOUBLE_TLV("MIC Out Mixer Volume", AC_OUT_MIXER_CTL,
			OUT_MIXER_MIC1_VOL, OUT_MIXER_MIC2_VOL,
			0x7, 0, mic_out_tlv),
	SOC_SINGLE_TLV("ADC Input Volume", AC_ADC_MIC_CTL,
			ADC_GAIN, 0x07, 0, adc_input_tlv),
	SOC_SINGLE_TLV("LINEOUT Volume", AC_LINEOUT_CTL,
			LINEOUT_VOL, 0x1f, 0, lineout_tlv),
	SOC_SINGLE_TLV("MIC1 Boost Volume", AC_ADC_MIC_CTL,
			MIC1_BOOST, 0x07, 0, mic_boost_tlv),
	SOC_SINGLE_TLV("MIC2 Boost Volume", AC_ADC_MIC_CTL,
			MIC2_BOOST, 0x07, 0, mic_boost_tlv),
};

/* Enable I2S & DAC clk, then enable the DAC digital part */
static int acx00_playback_event(struct snd_soc_dapm_widget *w,
				struct snd_kcontrol *k, int event)
{
	struct snd_soc_codec *codec = snd_soc_dapm_to_codec(w->dapm);

	switch (event) {
	case	SND_SOC_DAPM_POST_PMU:
		snd_soc_update_bits(codec, AC_SYS_CLK_CTL,
				(0x1<<SYS_CLK_DAC), (0x1<<SYS_CLK_DAC));
		snd_soc_update_bits(codec, AC_SYS_MOD_RST,
				(0x1<<MOD_RST_DAC), (0x1<<MOD_RST_DAC));
		snd_soc_update_bits(codec, AC_DAC_CTL,
				(0x1<<DAC_CTL_DAC_EN), (0x1<<DAC_CTL_DAC_EN));
		break;
	case	SND_SOC_DAPM_POST_PMD:
		snd_soc_update_bits(codec, AC_SYS_CLK_CTL,
				(0x1<<SYS_CLK_DAC), (0x0<<SYS_CLK_DAC));
		snd_soc_update_bits(codec, AC_SYS_MOD_RST,
				(0x1<<MOD_RST_DAC), (0x0<<MOD_RST_DAC));
		snd_soc_update_bits(codec, AC_DAC_CTL,
				(0x1<<DAC_CTL_DAC_EN), (0x0<<DAC_CTL_DAC_EN));
		break;
	default:
		break;
	}
	return 0;
}

/* Enable I2S & ADC clk, then enable the ADC digital part */
static int acx00_capture_event(struct snd_soc_dapm_widget *w,
				struct snd_kcontrol *k, int event)
{
	struct snd_soc_codec *codec = snd_soc_dapm_to_codec(w->dapm);

	switch (event) {
	case	SND_SOC_DAPM_POST_PMU:
		snd_soc_update_bits(codec, AC_SYS_CLK_CTL,
				(0x1<<SYS_CLK_ADC), (0x1<<SYS_CLK_ADC));
		snd_soc_update_bits(codec, AC_SYS_MOD_RST,
				(0x1<<MOD_RST_ADC), (0x1<<MOD_RST_ADC));
		snd_soc_update_bits(codec, AC_ADC_CTL,
				(0x1<<ADC_EN), (0x1<<ADC_EN));
		break;
	case	SND_SOC_DAPM_POST_PMD:
		snd_soc_update_bits(codec, AC_SYS_CLK_CTL,
				(0x1<<SYS_CLK_ADC), (0x0<<SYS_CLK_ADC));
		snd_soc_update_bits(codec, AC_SYS_MOD_RST,
				(0x1<<MOD_RST_ADC), (0x0<<MOD_RST_ADC));
		snd_soc_update_bits(codec, AC_ADC_CTL,
				(0x1<<ADC_EN), (0x0<<ADC_EN));
		break;
	default:
		break;
	}
	return 0;
}

/*
 * we used for three scene:
 * 1. No external Spker & DAPM LINEOUT used, we just enable the LINEOUT in the
 * ALSA codec probe(acx00_codec_probe) and resume, and we shutdown the LINEOUT
 * in device shutdown or suspend.
 * 2. No external Spker, but DAPM LINEOUT used, we just using the LINEOUT
 * enable or disable throught the DAPM control.
 * 3. External Spker & DAPM LINEOUT used, we just using the LINEOUT and
 * External Spker control GPIO enable or disable through DAPM control.
 */
static unsigned int spk_delay = 100;
module_param(spk_delay, int, 0644);
MODULE_PARM_DESC(spk_delay, "ACX00-Codec spk mute delay time");

/* delayed task: setting the speaker state on/off */
static void acx00_spk_enable(struct work_struct *work)
{
	struct acx00_priv *priv = container_of(work,
			struct acx00_priv, spk_work.work);
	gpio_set_value(priv->spk_gpio, priv->spk_gpio_state);
}

static int acx00_lineout_event(struct snd_soc_dapm_widget *w,
				struct snd_kcontrol *k, int event)
{
	struct snd_soc_codec *codec = snd_soc_dapm_to_codec(w->dapm);
	struct acx00_priv *priv = snd_soc_codec_get_drvdata(codec);

	switch (event) {
	/* PM event - resume */
	case	SND_SOC_DAPM_POST_PMU:
		if (!priv->enable) {
			snd_soc_update_bits(priv->codec, AC_LINEOUT_CTL,
					(1<<LINEL_SRC_EN), (1<<LINEL_SRC_EN));
			snd_soc_update_bits(priv->codec, AC_LINEOUT_CTL,
					(1<<LINER_SRC_EN), (1<<LINER_SRC_EN));
			msleep(100);
			priv->enable = 1;
		}
#ifdef ACX00_DAPM_LINEOUT
		snd_soc_update_bits(codec, AC_LINEOUT_CTL,
				(1<<LINEOUT_EN), (1<<LINEOUT_EN));
		mdelay(50);
#endif
		if (priv->spk_gpio_used) {
			if (spk_delay == 0) {
				gpio_set_value(priv->spk_gpio, priv->spk_gpio_state);
				/*
				* time delay to wait spk pa work fine,
				* general setting 50ms
				*/
				mdelay(50);
			} else
				schedule_delayed_work(&priv->spk_work,
					msecs_to_jiffies(spk_delay));
		}
		break;
	/* PM event - suspend */
	case	SND_SOC_DAPM_PRE_PMD:
		mdelay(50);
		if (priv->spk_gpio_used) {
			/* store current speaker switch state before suspend */
			priv->spk_gpio_state = gpio_get_value(priv->spk_gpio);
			gpio_set_value(priv->spk_gpio, 0);
			msleep(50);
		}
#ifdef ACX00_DAPM_LINEOUT
		snd_soc_update_bits(codec, AC_LINEOUT_CTL,
				(1<<LINEOUT_EN), (0<<LINEOUT_EN));
#endif
		break;
	default:
		break;
	}
	return 0;
}

/* AC_I2S_MIXER_SRC : 0x2114 */
static const struct snd_kcontrol_new i2sl_mixer_src[] = {
	SOC_DAPM_SINGLE("I2SDACL Switch", AC_I2S_MIXER_SRC,
			I2S_MIXERL_SRC_DAC, 1, 0),
	SOC_DAPM_SINGLE("ADCL Switch", AC_I2S_MIXER_SRC,
			I2S_MIXERL_SRC_ADC, 1, 0),
};

static const struct snd_kcontrol_new i2sr_mixer_src[] = {
	SOC_DAPM_SINGLE("I2SDACR Switch", AC_I2S_MIXER_SRC,
			I2S_MIXERR_SRC_DAC, 1, 0),
	SOC_DAPM_SINGLE("ADCR Switch", AC_I2S_MIXER_SRC,
			I2S_MIXERR_SRC_ADC, 1, 0),
};

/* AC_DAC_MIXER_SRC : 0x2202 */
static const struct snd_kcontrol_new dacl_mixer_src[] = {
	SOC_DAPM_SINGLE("I2SDACL Switch", AC_DAC_MIXER_SRC,
			DAC_MIXERL_SRC_DAC, 1, 0),
	SOC_DAPM_SINGLE("ADCL Switch", AC_DAC_MIXER_SRC,
			DAC_MIXERL_SRC_ADC, 1, 0),
};

static const struct snd_kcontrol_new dacr_mixer_src[] = {
	SOC_DAPM_SINGLE("I2SDACR Switch", AC_DAC_MIXER_SRC,
			DAC_MIXERR_SRC_DAC, 1, 0),
	SOC_DAPM_SINGLE("ADCR Switch", AC_DAC_MIXER_SRC,
			DAC_MIXERR_SRC_ADC, 1, 0),
};

/* AC_OUT_MIXER_SRC : 0x2222 */
static const struct snd_kcontrol_new left_output_mixer[] = {
	SOC_DAPM_SINGLE("MIC1 Switch", AC_OUT_MIXER_SRC,
			OUT_MIXERL_SRC_MIC1, 1, 0),
	//SOC_DAPM_SINGLE("MIC2 Switch", AC_OUT_MIXER_SRC,
	//		OUT_MIXERL_SRC_MIC2, 1, 0),
	//SOC_DAPM_SINGLE("PhonePN Switch", AC_OUT_MIXER_SRC,
	//		OUT_MIXERL_SRC_PHPN, 1, 0),
	//SOC_DAPM_SINGLE("PhoneN Switch", AC_OUT_MIXER_SRC,
	//		OUT_MIXERL_SRC_PHN, 1, 0),
	//SOC_DAPM_SINGLE("LINEINL Switch", AC_OUT_MIXER_SRC,
	//		OUT_MIXERL_SRC_LINEL, 1, 0),
	SOC_DAPM_SINGLE("DACL Switch", AC_OUT_MIXER_SRC,
			OUT_MIXERL_SRC_DACL, 1, 0),
	//SOC_DAPM_SINGLE("DACR Switch", AC_OUT_MIXER_SRC,
	//		OUT_MIXERL_SRC_DACR, 1, 0),
};

static const struct snd_kcontrol_new right_output_mixer[] = {
	//SOC_DAPM_SINGLE("MIC1 Switch", AC_OUT_MIXER_SRC,
	//		OUT_MIXERR_SRC_MIC1, 1, 0),
	SOC_DAPM_SINGLE("MIC2 Switch", AC_OUT_MIXER_SRC,
			OUT_MIXERR_SRC_MIC2, 1, 0),
	//SOC_DAPM_SINGLE("PhonePN Switch", AC_OUT_MIXER_SRC,
	//		OUT_MIXERR_SRC_PHPN, 1, 0),
	//SOC_DAPM_SINGLE("PhoneP Switch", AC_OUT_MIXER_SRC,
	//		OUT_MIXERR_SRC_PHP, 1, 0),
	//SOC_DAPM_SINGLE("LINEINR Switch", AC_OUT_MIXER_SRC,
	//		OUT_MIXERR_SRC_LINER, 1, 0),
	SOC_DAPM_SINGLE("DACR Switch", AC_OUT_MIXER_SRC,
			OUT_MIXERR_SRC_DACR, 1, 0),
	//SOC_DAPM_SINGLE("DACL Switch", AC_OUT_MIXER_SRC,
	//		OUT_MIXERR_SRC_DACL, 1, 0),
};

/* AC_LINEOUT_CTL : 0x2224 */
const char * const left_lineout_text[] = {
	"Left OMixer", "LR OMixer",
};

static const struct soc_enum left_lineout_enum =
	SOC_ENUM_SINGLE(AC_LINEOUT_CTL, LINEL_SRC,
		ARRAY_SIZE(left_lineout_text), left_lineout_text);

static const struct snd_kcontrol_new left_lineout_mux =
	SOC_DAPM_ENUM("Left LINEOUT Mux", left_lineout_enum);

const char * const right_lineout_text[] = {
	"Right OMixer", "LR OMixer",
};

static const struct soc_enum right_lineout_enum =
	SOC_ENUM_SINGLE(AC_LINEOUT_CTL, LINER_SRC,
		ARRAY_SIZE(right_lineout_text), right_lineout_text);

static const struct snd_kcontrol_new right_lineout_mux =
	SOC_DAPM_ENUM("Right LINEOUT Mux", right_lineout_enum);

/* AC_ADC_MIXER_SRC : 0x2322 */
static const struct snd_kcontrol_new left_input_mixer[] = {
	SOC_DAPM_SINGLE("MIC1 Switch", AC_ADC_MIXER_SRC,
			ADC_MIXERL_MIC1, 1, 0),
	//SOC_DAPM_SINGLE("MIC2 Switch", AC_ADC_MIXER_SRC,
	//		ADC_MIXERL_MIC2, 1, 0),
	//SOC_DAPM_SINGLE("PhonePN Switch", AC_ADC_MIXER_SRC,
	//		ADC_MIXERL_PHPN, 1, 0),
	//SOC_DAPM_SINGLE("PhoneN Switch", AC_ADC_MIXER_SRC,
	//		ADC_MIXERL_PHN, 1, 0),
	//SOC_DAPM_SINGLE("LINEINL Switch", AC_ADC_MIXER_SRC,
	//		ADC_MIXERL_LINEL, 1, 0),
	//SOC_DAPM_SINGLE("OMixerL Switch", AC_ADC_MIXER_SRC,
	//		ADC_MIXERL_MIXL, 1, 0),
	//SOC_DAPM_SINGLE("OMixerR Switch", AC_ADC_MIXER_SRC,
	//		ADC_MIXERL_MIXR, 1, 0),
};

static const struct snd_kcontrol_new right_input_mixer[] = {
	//SOC_DAPM_SINGLE("MIC1 Switch", AC_ADC_MIXER_SRC,
	//		ADC_MIXERR_MIC1, 1, 0),
	SOC_DAPM_SINGLE("MIC2 Switch", AC_ADC_MIXER_SRC,
			ADC_MIXERR_MIC2, 1, 0),
	//SOC_DAPM_SINGLE("PhonePN Switch", AC_ADC_MIXER_SRC,
	//		ADC_MIXERR_PHPN, 1, 0),
	//SOC_DAPM_SINGLE("PhoneP Switch", AC_ADC_MIXER_SRC,
	//		ADC_MIXERR_PHP, 1, 0),
	//SOC_DAPM_SINGLE("LINEINR Switch", AC_ADC_MIXER_SRC,
	//		ADC_MIXERR_LINER, 1, 0),
	//SOC_DAPM_SINGLE("OMixerR Switch", AC_ADC_MIXER_SRC,
	//		ADC_MIXERR_MIXR, 1, 0),
	//SOC_DAPM_SINGLE("OMixerL Switch", AC_ADC_MIXER_SRC,
	//		ADC_MIXERR_MIXL, 1, 0),
};

static const struct snd_soc_dapm_widget acx00_codec_dapm_widgets[] = {
	SND_SOC_DAPM_AIF_IN_E("DACL", "Playback", 0, AC_DAC_CTL,
			OUT_MIXER_DACL_EN, 0,
			acx00_playback_event,
			SND_SOC_DAPM_POST_PMU | SND_SOC_DAPM_POST_PMD),
	SND_SOC_DAPM_AIF_IN_E("DACR", "Playback", 0,
			AC_DAC_CTL, OUT_MIXER_DACR_EN, 0,
			acx00_playback_event,
			SND_SOC_DAPM_POST_PMU | SND_SOC_DAPM_POST_PMD),

	SND_SOC_DAPM_AIF_OUT_E("ADCL", "Capture", 0,
			AC_ADC_MIC_CTL, ADCL_EN, 0,
			acx00_capture_event,
			SND_SOC_DAPM_POST_PMU | SND_SOC_DAPM_POST_PMD),
	SND_SOC_DAPM_AIF_OUT_E("ADCR", "Capture", 0,
			AC_ADC_MIC_CTL, ADCR_EN, 0,
			acx00_capture_event,
			SND_SOC_DAPM_POST_PMU | SND_SOC_DAPM_POST_PMD),

	SND_SOC_DAPM_MIXER("Left Output Mixer", AC_OUT_MIXER_CTL,
			OUT_MIXER_LMIX_EN, 0,
			left_output_mixer, ARRAY_SIZE(left_output_mixer)),

	SND_SOC_DAPM_MIXER("Right Output Mixer", AC_OUT_MIXER_CTL,
			OUT_MIXER_RMIX_EN, 0, right_output_mixer,
			ARRAY_SIZE(right_output_mixer)),

	SND_SOC_DAPM_MIXER("Left Input Mixer", SND_SOC_NOPM, 0, 0,
			left_input_mixer, ARRAY_SIZE(left_input_mixer)),
	SND_SOC_DAPM_MIXER("Right Input Mixer", SND_SOC_NOPM, 0, 0,
			right_input_mixer, ARRAY_SIZE(right_input_mixer)),

	SND_SOC_DAPM_MIXER("Left DAC Mixer", AC_OUT_MIXER_CTL,
			OUT_MIXER_DACL_EN, 0, dacl_mixer_src,
			ARRAY_SIZE(dacl_mixer_src)),
	SND_SOC_DAPM_MIXER("Right DAC Mixer", AC_OUT_MIXER_CTL,
			OUT_MIXER_DACR_EN, 0, dacr_mixer_src,
			ARRAY_SIZE(dacr_mixer_src)),

	SND_SOC_DAPM_MIXER("Left I2S Mixer", SND_SOC_NOPM,
			0, 0, i2sl_mixer_src, ARRAY_SIZE(i2sl_mixer_src)),
	SND_SOC_DAPM_MIXER("Right I2S Mixer", SND_SOC_NOPM,
			0, 0, i2sr_mixer_src, ARRAY_SIZE(i2sr_mixer_src)),

	SND_SOC_DAPM_MUX("Left LINEOUT Mux", SND_SOC_NOPM,
			0, 0, &left_lineout_mux),
	SND_SOC_DAPM_MUX("Right LINEOUT Mux", SND_SOC_NOPM,
			0, 0, &right_lineout_mux),

	SND_SOC_DAPM_PGA("MIC1 PGA", AC_ADC_MIC_CTL,
			MIC1_GAIN_EN, 0, NULL, 0),
	SND_SOC_DAPM_PGA("MIC2 PGA", AC_ADC_MIC_CTL,
			MIC2_GAIN_EN, 0, NULL, 0),

	SND_SOC_DAPM_MICBIAS("MIC Bias", AC_MICBIAS_CTL,
			MMBIAS_EN, 0),

	/* PHONEIN & PHONEOUT not enable in pin assign */
	//SND_SOC_DAPM_INPUT("PHONEINP"),
	//SND_SOC_DAPM_INPUT("PHONEINN"),
	//SND_SOC_DAPM_INPUT("PHONEINPN"),

	/* endpoint define */
	//SND_SOC_DAPM_LINE("LINEIN", NULL),
	SND_SOC_DAPM_LINE("LINEOUT", acx00_lineout_event),
	SND_SOC_DAPM_MIC("MIC1", NULL),
	SND_SOC_DAPM_MIC("MIC2", NULL),
};

static const struct snd_soc_dapm_route acx00_codec_dapm_routes[] = {
	{"Left Output Mixer", "MIC1 Switch", "MIC1 PGA"},
	//{"Left Output Mixer", "MIC2 Switch", "MIC2 PGA"},
	//{"Left Output Mixer", "PhonePN Switch", "PHONEINPN"},
	//{"Left Output Mixer", "PhoneN Switch", "PHONEINN"},
	//{"Left Output Mixer", "LINEINL Switch", "LINEIN"},
	//{"Left Output Mixer", "DACR Switch", "Right DAC Mixer"},
	{"Left Output Mixer", "DACL Switch", "Left DAC Mixer"},

	//{"Right Output Mixer", "MIC1 Switch", "MIC1 PGA"},
	{"Right Output Mixer", "MIC2 Switch", "MIC2 PGA"},
	//{"Right Output Mixer", "PhonePN Switch", "PHONEINPN"},
	//{"Right Output Mixer", "PhoneP Switch", "PHONEINP"},
	//{"Right Output Mixer", "LINEINR Switch", "LINEIN"},
	{"Right Output Mixer", "DACR Switch", "Right DAC Mixer"},
	//{"Right Output Mixer", "DACL Switch", "Left DAC Mixer"},

	{"Left LINEOUT Mux", NULL, "Left Output Mixer"},
	//{"Left LINEOUT Mux", "LR OMixer", "Right Output Mixer"},
	{"Right LINEOUT Mux", NULL, "Right Output Mixer"},
	//{"Right LINEOUT Mux", "LR OMixer", "Left Output Mixer"},

	{"Left Input Mixer", "MIC1 Switch", "MIC1 PGA"},
	//{"Left Input Mixer", "MIC2 Switch", "MIC2 PGA"},
	//{"Left Input Mixer", "PhonePN Switch", "PHONEINPN"},
	//{"Left Input Mixer", "PhoneN Switch", "PHONEINN"},
	//{"Left Input Mixer", "LINEINL Switch", "LINEIN"},
	//{"Left Input Mixer", "OMixerL Switch", "Left Output Mixer"},
	//{"Left Input Mixer", "OMixerR Switch", "Right Output Mixer"},

	//{"Right Input Mixer", "MIC1 Switch", "MIC1 PGA"},
	{"Right Input Mixer", "MIC2 Switch", "MIC2 PGA"},
	//{"Right Input Mixer", "PhonePN Switch", "PHONEINPN"},
	//{"Right Input Mixer", "PhoneP Switch", "PHONEINP"},
	//{"Right Input Mixer", "LINEINR Switch", "LINEIN"},
	//{"Right Input Mixer", "OMixerR Switch", "Right Output Mixer"},
	//{"Right Input Mixer", "OMixerL Switch", "Left Output Mixer"},

	{"Left I2S Mixer", "I2SDACL Switch", "DACL"},
	{"Left I2S Mixer", "ADCL Switch", "Left Input Mixer"},

	{"Right I2S Mixer", "I2SDACR Switch", "DACR"},
	{"Right I2S Mixer", "ADCR Switch", "Right Input Mixer"},

	{"Left DAC Mixer", "I2SDACL Switch", "DACL"},
	{"Left DAC Mixer", "ADCL Switch", "Left Input Mixer"},

	{"Right DAC Mixer", "I2SDACR Switch", "DACR"},
	{"Right DAC Mixer", "ADCR Switch", "Right Input Mixer"},

	{"ADCL", NULL, "Left I2S Mixer"},
	{"ADCR", NULL, "Right I2S Mixer"},

	{"LINEOUT", NULL, "Left LINEOUT Mux"},
	{"LINEOUT", NULL, "Right LINEOUT Mux"},

	{"MIC Bias", NULL, "MIC1"},
	{"MIC Bias", NULL, "MIC2"},
	{"MIC1 PGA", NULL, "MIC Bias"},
	{"MIC2 PGA", NULL, "MIC Bias"},
};

static void acx00_codec_txctrl_enable(struct snd_soc_codec *codec,
					int enable)
{
	pr_debug("Enter %s, enable %d\n", __func__, enable);
	if (enable) {
		snd_soc_update_bits(codec, AC_I2S_CTL,
					(1<<I2S_RX_EN), (1<<I2S_RX_EN));
	} else {
		snd_soc_update_bits(codec, AC_I2S_CTL,
					(1<<I2S_RX_EN), (0<<I2S_RX_EN));
	}
	pr_debug("End %s, enable %d\n", __func__, enable);
}

static void acx00_codec_rxctrl_enable(struct snd_soc_codec *codec,
					int enable)
{
	pr_debug("Enter %s, enable %d\n", __func__, enable);
	if (enable) {
		snd_soc_update_bits(codec, AC_I2S_CTL,
					(1<<I2S_TX_EN), (1<<I2S_TX_EN));
	} else {
		snd_soc_update_bits(codec, AC_I2S_CTL,
					(1<<I2S_TX_EN), (0<<I2S_TX_EN));
	}
	pr_debug("End %s, enable %d\n", __func__, enable);
}

static void acx00_codec_init(struct snd_soc_codec *codec)
{
	struct acx00_priv *priv = snd_soc_codec_get_drvdata(codec);

	/* acx00_codec sysctl init */
	acx00_reg_write(priv->acx00, 0x0010, 0x03);
	acx00_reg_write(priv->acx00, 0x0012, 0x01);
	/* The bit3 need to setup to 1 for bias current. */
	snd_soc_update_bits(codec, AC_MICBIAS_CTL,
			(0x1 << ADDA_BIAS_CUR), (0x1 << ADDA_BIAS_CUR));

	/* enable the output & global enable bit */
	snd_soc_update_bits(codec, AC_I2S_CTL,
			(1<<I2S_SDO0_EN), (1<<I2S_SDO0_EN));
	snd_soc_update_bits(codec, AC_I2S_CTL, (1<<I2S_GEN), (1<<I2S_GEN));

	/* Default setting slot width as 32 bit for I2S */
	snd_soc_update_bits(codec, AC_I2S_FMT0,
			(7<<I2S_FMT_SLOT_WIDTH), (7<<I2S_FMT_SLOT_WIDTH));

	/* default setting 0xA0A0 for ADC & DAC Volume */
	snd_soc_write(codec, AC_I2S_DAC_VOL, 0xB0B0);
	snd_soc_write(codec, AC_I2S_ADC_VOL, ACX00_DEF_VOL);

        //snd_soc_write(codec, AC_LINEOUT_CTL, ACX00_DEF_VOL);




	/* Enable HPF for high pass filter */
	snd_soc_update_bits(codec, AC_DAC_CTL,
			(1<<DAC_CTL_HPF_EN), (1<<DAC_CTL_HPF_EN));

	/* LINEOUT ANTI POP & Click noise */
	snd_soc_update_bits(codec, AC_LINEOUT_CTL,
			(0x7<<LINE_ANTI_TIME), (0x3<<LINE_ANTI_TIME));
	snd_soc_update_bits(codec, AC_LINEOUT_CTL,
			(0x3<<LINE_SLOPE_SEL), (0x3<<LINE_SLOPE_SEL));

	/* enable & setting adc convert delay time */
	snd_soc_update_bits(codec, AC_ADC_CTL, (0x3<<ADC_DELAY_TIME),
			(0x3<<ADC_DELAY_TIME));
	snd_soc_update_bits(codec, AC_ADC_CTL, (1<<ADC_DELAY_EN),
			(1<<ADC_DELAY_EN));


	if (priv->spk_gpio_used) {
		snd_soc_update_bits(priv->codec, AC_LINEOUT_CTL,
					(1<<LINEL_SRC_EN), (1<<LINEL_SRC_EN));
		snd_soc_update_bits(priv->codec, AC_LINEOUT_CTL,
					(1<<LINER_SRC_EN), (1<<LINER_SRC_EN));
		priv->enable = 1;
	}
#ifndef ACX00_DAPM_LINEOUT
	snd_soc_update_bits(codec, AC_LINEOUT_CTL, (1<<LINEOUT_EN),
			(1<<LINEOUT_EN));
#endif
}

static int acx00_codec_hw_params(struct snd_pcm_substream *substream,
		struct snd_pcm_hw_params *params, struct snd_soc_dai *dai)
{
	struct snd_soc_codec *codec = dai->codec;
	int i;

	switch (params_format(params)) {
	case SNDRV_PCM_FORMAT_S16_LE:
		snd_soc_update_bits(codec, AC_I2S_FMT0,
			(7<<I2S_FMT_SAMPLE), (3<<I2S_FMT_SAMPLE));
		break;
	case SNDRV_PCM_FORMAT_S24_LE:
		snd_soc_update_bits(codec, AC_I2S_FMT0,
			(7<<I2S_FMT_SAMPLE), (5<<I2S_FMT_SAMPLE));
		break;
	case SNDRV_PCM_FORMAT_S32_LE:
		snd_soc_update_bits(codec, AC_I2S_FMT0,
			(7<<I2S_FMT_SAMPLE), (7<<I2S_FMT_SAMPLE));
		break;
	default:
		dev_err(codec->dev, "unrecognized format support\n");
		break;
	}
	for (i = 0; i < ARRAY_SIZE(sample_rate_conv); i++) {
		if (sample_rate_conv[i].samplerate == params_rate(params)) {
			snd_soc_update_bits(codec, AC_SYS_SR_CTL,
				(SYS_SR_MASK<<SYS_SR_BIT),
				(sample_rate_conv[i].rate_bit<<SYS_SR_BIT));
		}
	}

	return 0;
}

static int acx00_codec_dai_set_sysclk(struct snd_soc_dai *codec_dai,
		int clk_id, unsigned int freq, int dir)
{
	return 0;
}

static int acx00_codec_dai_set_fmt(struct snd_soc_dai *codec_dai,
		unsigned int fmt)
{
	struct acx00_priv *priv = snd_soc_dai_get_drvdata(codec_dai);
	struct snd_soc_codec *codec = priv->codec;

	switch (fmt & SND_SOC_DAIFMT_MASTER_MASK) {
	/* codec clk & FRM master */
	case SND_SOC_DAIFMT_CBM_CFM:
		snd_soc_update_bits(codec, AC_I2S_CLK,
				(0x1<<I2S_BCLK_OUT), (0x1<<I2S_BCLK_OUT));
		snd_soc_update_bits(codec, AC_I2S_CLK,
				(0x1<<I2S_LRCK_OUT), (0x1<<I2S_LRCK_OUT));
		break;
	/* codec clk & FRM slave */
	case SND_SOC_DAIFMT_CBS_CFS:
		snd_soc_update_bits(codec, AC_I2S_CLK,
				(0x1<<I2S_BCLK_OUT), 0x0<<I2S_BCLK_OUT);
		snd_soc_update_bits(codec, AC_I2S_CLK,
				(0x1<<I2S_LRCK_OUT), 0x0<<I2S_LRCK_OUT);
		break;
	}

	switch (fmt & SND_SOC_DAIFMT_FORMAT_MASK) {
	case SND_SOC_DAIFMT_I2S:
		snd_soc_update_bits(codec, AC_I2S_CLK,
				(0x3FF<<I2S_LRCK_PERIOD),
				(0x1F<<I2S_LRCK_PERIOD));
		snd_soc_update_bits(codec, AC_I2S_FMT0,
				(0x3<<I2S_FMT_MODE), (0x1<<I2S_FMT_MODE));
		snd_soc_update_bits(codec, AC_I2S_FMT0,
				(0x1<<I2S_FMT_TX_OFFSET),
				(0x1<<I2S_FMT_TX_OFFSET));
		snd_soc_update_bits(codec, AC_I2S_FMT0,
				(0x1<<I2S_FMT_RX_OFFSET),
				(0x1<<I2S_FMT_RX_OFFSET));
		break;
	case SND_SOC_DAIFMT_RIGHT_J:
		snd_soc_update_bits(codec, AC_I2S_CLK,
				(0x3FF<<I2S_LRCK_PERIOD),
				(0x1F<<I2S_LRCK_PERIOD));
		snd_soc_update_bits(codec, AC_I2S_FMT0,
				(0x3<<I2S_FMT_MODE), (0x2<<I2S_FMT_MODE));
		snd_soc_update_bits(codec, AC_I2S_FMT0,
				(0x1<<I2S_FMT_TX_OFFSET),
				(0x0<<I2S_FMT_TX_OFFSET));
		snd_soc_update_bits(codec, AC_I2S_FMT0,
				(0x1<<I2S_FMT_TX_OFFSET),
				(0x0<<I2S_FMT_RX_OFFSET));
		break;
	case SND_SOC_DAIFMT_LEFT_J:
		snd_soc_update_bits(codec, AC_I2S_CLK,
				(0x3FF<<I2S_LRCK_PERIOD),
				(0x1F<<I2S_LRCK_PERIOD));
		snd_soc_update_bits(codec, AC_I2S_FMT0,
				(0x3<<I2S_FMT_MODE), (0x1<<I2S_FMT_MODE));
		snd_soc_update_bits(codec, AC_I2S_FMT0,
				(0x1<<I2S_FMT_TX_OFFSET),
				(0x0<<I2S_FMT_TX_OFFSET));
		snd_soc_update_bits(codec, AC_I2S_FMT0,
				(0x1<<I2S_FMT_RX_OFFSET),
				(0x0<<I2S_FMT_RX_OFFSET));
		break;
	case SND_SOC_DAIFMT_DSP_A:
		snd_soc_update_bits(codec, AC_I2S_CLK,
				(0x3FF<<I2S_LRCK_PERIOD),
				(0x3F<<I2S_LRCK_PERIOD));
		snd_soc_update_bits(codec, AC_I2S_FMT0,
				(0x3<<I2S_FMT_MODE), (0x0<<I2S_FMT_MODE));
		snd_soc_update_bits(codec, AC_I2S_FMT0,
				(0x1<<I2S_FMT_TX_OFFSET),
				(0x1<<I2S_FMT_TX_OFFSET));
		snd_soc_update_bits(codec, AC_I2S_FMT0,
				(0x1<<I2S_FMT_RX_OFFSET),
				(0x1<<I2S_FMT_RX_OFFSET));
		break;
	case SND_SOC_DAIFMT_DSP_B:
		snd_soc_update_bits(codec, AC_I2S_CLK,
				(0x3FF<<I2S_LRCK_PERIOD),
				(0x3F<<I2S_LRCK_PERIOD));
		snd_soc_update_bits(codec, AC_I2S_FMT0,
				(0x3<<I2S_FMT_MODE), (0x0<<I2S_FMT_MODE));
		snd_soc_update_bits(codec, AC_I2S_FMT0,
				(0x1<<I2S_FMT_TX_OFFSET),
				(0x0<<I2S_FMT_TX_OFFSET));
		snd_soc_update_bits(codec, AC_I2S_FMT0,
				(0x1<<I2S_FMT_RX_OFFSET),
				(0x0<<I2S_FMT_RX_OFFSET));
		break;
	default:
		dev_err(codec->dev, "format setting failed\n");
		break;
	}

	switch (fmt & SND_SOC_DAIFMT_INV_MASK) {
	case SND_SOC_DAIFMT_NB_NF:
		snd_soc_update_bits(codec, AC_I2S_FMT1,
				(0x1<<I2S_FMT_BCLK_POLAR),
				(0x0<<I2S_FMT_BCLK_POLAR));
		snd_soc_update_bits(codec, AC_I2S_FMT1,
				(0x1<<I2S_FMT_LRCK_POLAR),
				(0x0<<I2S_FMT_LRCK_POLAR));
		break;
	case SND_SOC_DAIFMT_NB_IF:
		snd_soc_update_bits(codec, AC_I2S_FMT1,
				(0x1<<I2S_FMT_BCLK_POLAR),
				(0x0<<I2S_FMT_BCLK_POLAR));
		snd_soc_update_bits(codec, AC_I2S_FMT1,
				(0x1<<I2S_FMT_LRCK_POLAR),
				(0x1<<I2S_FMT_LRCK_POLAR));
		break;
	case SND_SOC_DAIFMT_IB_NF:
		snd_soc_update_bits(codec, AC_I2S_FMT1,
				(0x1<<I2S_FMT_BCLK_POLAR),
				(0x1<<I2S_FMT_BCLK_POLAR));
		snd_soc_update_bits(codec, AC_I2S_FMT1,
				(0x1<<I2S_FMT_LRCK_POLAR),
				(0x0<<I2S_FMT_LRCK_POLAR));
		break;
	case SND_SOC_DAIFMT_IB_IF:
		snd_soc_update_bits(codec, AC_I2S_FMT1,
				(0x1<<I2S_FMT_BCLK_POLAR),
				(0x1<<I2S_FMT_BCLK_POLAR));
		snd_soc_update_bits(codec, AC_I2S_FMT1,
				(0x1<<I2S_FMT_LRCK_POLAR),
				(0x1<<I2S_FMT_LRCK_POLAR));
		break;
	default:
		dev_err(codec->dev, "invert clk setting failed\n");
		return -EINVAL;
	}
	return 0;
}

static int acx00_codec_dai_set_clkdiv(struct snd_soc_dai *codec_dai,
		int clk_id, int clk_div)
{
	struct acx00_priv *priv = snd_soc_dai_get_drvdata(codec_dai);
	struct snd_soc_codec *codec = priv->codec;
	unsigned int bclk_div;
	/*
	 * when PCM mode, setting as 64fs, when I2S mode as 32fs,
	 * then two channel, then just as 64fs
	 */
	unsigned int div_ratio = clk_div / 64;

	switch (div_ratio) {
	case 1:
		bclk_div = I2S_BCLK_DIV_1;
		break;
	case 2:
		bclk_div = I2S_BCLK_DIV_2;
		break;
	case 4:
		bclk_div = I2S_BCLK_DIV_3;
		break;
	case 6:
		bclk_div = I2S_BCLK_DIV_4;
		break;
	case 8:
		bclk_div = I2S_BCLK_DIV_5;
		break;
	case 12:
		bclk_div = I2S_BCLK_DIV_6;
		break;
	case 16:
		bclk_div = I2S_BCLK_DIV_7;
		break;
	case 24:
		bclk_div = I2S_BCLK_DIV_8;
		break;
	case 32:
		bclk_div = I2S_BCLK_DIV_9;
		break;
	case 48:
		bclk_div = I2S_BCLK_DIV_10;
		break;
	case 64:
		bclk_div = I2S_BCLK_DIV_11;
		break;
	case 96:
		bclk_div = I2S_BCLK_DIV_12;
		break;
	case 128:
		bclk_div = I2S_BCLK_DIV_13;
		break;
	case 176:
		bclk_div = I2S_BCLK_DIV_14;
		break;
	case 192:
		bclk_div = I2S_BCLK_DIV_15;
		break;
	default:
		dev_err(codec->dev, "setting blck div failed\n");
		break;
	}

	snd_soc_update_bits(codec, AC_I2S_CLK,
			(I2S_BCLK_DIV_MASK<<I2S_BLCK_DIV),
			(bclk_div<<I2S_BLCK_DIV));
	return 0;
}

static int acx00_codec_startup(struct snd_pcm_substream *substream,
		struct snd_soc_dai *codec_dai)
{
	return 0;
}

static bool acx00_loop_en;
module_param(acx00_loop_en, bool, 0644);
MODULE_PARM_DESC(acx00_loop_en, "ACX00-Codec audio loopback debug(Y=enable, N=disable)");

static int acx00_codec_trigger(struct snd_pcm_substream *substream,
		int cmd, struct snd_soc_dai *codec_dai)
{
	return 0;
}

static int acx00_codec_prepare(struct snd_pcm_substream *substream,
		struct snd_soc_dai *codec_dai)
{
	snd_soc_update_bits(codec_dai->codec, AC_SYS_CLK_CTL,
			(0x1<<SYS_CLK_I2S), (0x1<<SYS_CLK_I2S));
	snd_soc_update_bits(codec_dai->codec, AC_SYS_MOD_RST,
			(0x1<<MOD_RST_I2S), (0x1<<MOD_RST_I2S));

	if (substream->stream == SNDRV_PCM_STREAM_PLAYBACK) {
		if (acx00_loop_en)
			snd_soc_update_bits(codec_dai->codec, AC_I2S_FMT0,
				(0x1<<I2S_FMT_LOOP),
				(0x1<<I2S_FMT_LOOP));
		else
			snd_soc_update_bits(codec_dai->codec, AC_I2S_FMT0,
				(0x1<<I2S_FMT_LOOP),
				(0x0<<I2S_FMT_LOOP));
		acx00_codec_txctrl_enable(codec_dai->codec, 1);
	} else
		acx00_codec_rxctrl_enable(codec_dai->codec, 1);
	return 0;
}

static int acx00_codec_digital_mute(struct snd_soc_dai *codec_dai,
				int mute)
{
	struct snd_soc_codec *codec = codec_dai->codec;

	if (mute)
		snd_soc_write(codec, AC_I2S_DAC_VOL, 0);
	else
		snd_soc_write(codec, AC_I2S_DAC_VOL, ACX00_DEF_VOL);
	return 0;
}

static void acx00_codec_shutdown(struct snd_pcm_substream *substream,
				struct snd_soc_dai *dai)
{
	struct snd_soc_codec *codec = dai->codec;

	if (substream->stream == SNDRV_PCM_STREAM_PLAYBACK)
		acx00_codec_txctrl_enable(codec, 0);
	else
		acx00_codec_rxctrl_enable(codec, 0);
}

static const struct snd_soc_dai_ops acx00_codec_dai_ops = {
	.hw_params	= acx00_codec_hw_params,
	.shutdown	= acx00_codec_shutdown,
	.digital_mute	= acx00_codec_digital_mute,
	.set_sysclk	= acx00_codec_dai_set_sysclk,
	.set_fmt	= acx00_codec_dai_set_fmt,
	.set_clkdiv	= acx00_codec_dai_set_clkdiv,
	.startup	= acx00_codec_startup,
	.trigger	= acx00_codec_trigger,
	.prepare	= acx00_codec_prepare,
};

static struct snd_soc_dai_driver acx00_codec_dai[] = {
	{
		.name = "acx00-dai",
		.playback = {
			.stream_name = "Playback",
			.channels_min = 1,
			.channels_max = 2,
			.rates = SNDRV_PCM_RATE_8000_192000
				| SNDRV_PCM_RATE_KNOT,
			.formats = SNDRV_PCM_FMTBIT_S16_LE
				| SNDRV_PCM_FMTBIT_S24_LE
				| SNDRV_PCM_FMTBIT_S32_LE,
		},
		.capture = {
			.stream_name = "Capture",
			.channels_min = 1,
			.channels_max = 2,
			.rates = SNDRV_PCM_RATE_8000_192000
				| SNDRV_PCM_RATE_KNOT,
			.formats = SNDRV_PCM_FMTBIT_S16_LE
				| SNDRV_PCM_FMTBIT_S24_LE
				| SNDRV_PCM_FMTBIT_S32_LE,
		},
		.ops = &acx00_codec_dai_ops,
	},
};

static void acx00_codec_resume_work(struct work_struct *work)
{
	struct acx00_priv *priv = container_of(work,
			struct acx00_priv, resume_work.work);

	acx00_codec_init(priv->codec);
}

static int acx00_codec_probe(struct snd_soc_codec *codec)
{
	struct acx00_priv *priv = snd_soc_codec_get_drvdata(codec);
	struct snd_soc_dapm_context *dapm = snd_soc_codec_get_dapm(codec);
	int ret = 0;

	mutex_init(&priv->mutex);

	priv->codec = codec;

	/* Add virtual switch */
	ret = snd_soc_add_codec_controls(codec, acx00_codec_controls,
					ARRAY_SIZE(acx00_codec_controls));
	if (ret) {
		pr_err("[audio-codec] Failed to register audio mode control, will continue without it.\n");
	}
	snd_soc_dapm_new_controls(dapm, acx00_codec_dapm_widgets, ARRAY_SIZE(acx00_codec_dapm_widgets));
	snd_soc_dapm_add_routes(dapm, acx00_codec_dapm_routes, ARRAY_SIZE(acx00_codec_dapm_routes));

	/* using late_initcall to wait 120ms acx00-core to make chip reset */
	acx00_codec_init(codec);
	INIT_DELAYED_WORK(&priv->spk_work, acx00_spk_enable);
	INIT_DELAYED_WORK(&priv->resume_work, acx00_codec_resume_work);
	return 0;
}

static int acx00_codec_remove(struct snd_soc_codec *codec)
{
	struct acx00_priv *priv = snd_soc_codec_get_drvdata(codec);

	cancel_delayed_work_sync(&priv->spk_work);
	cancel_delayed_work_sync(&priv->resume_work);
	return 0;
}

static unsigned int acx00_codec_read(struct snd_soc_codec *codec,
					unsigned int reg)
{
	unsigned int data;
	struct acx00_priv *priv = snd_soc_codec_get_drvdata(codec);

	/* Device I/O API */
	data = acx00_reg_read(priv->acx00, reg);
	return data;
}

static int acx00_codec_write(struct snd_soc_codec *codec,
			unsigned int reg, unsigned int value)
{
	struct acx00_priv *priv = snd_soc_codec_get_drvdata(codec);

	return acx00_reg_write(priv->acx00, reg, value);
}

static int sunxi_gpio_iodisable(u32 gpio)
{
	char pin_name[8];
	u32 config, ret;

	sunxi_gpio_to_name(gpio, pin_name);
	config = 7 << 16;
	ret = pin_config_set(SUNXI_PINCTRL, pin_name, config);
	return ret;
}

static int acx00_codec_suspend(struct snd_soc_codec *codec)
{
	struct acx00_priv *priv = snd_soc_codec_get_drvdata(codec);

	pr_debug("Enter %s\n", __func__);

	clk_disable_unprepare(priv->clk);

	/* PA_CTRL first setting low state, then make it iodisabled */
	if (priv->spk_gpio_used) {
		sunxi_gpio_iodisable(priv->spk_gpio);
		msleep(30);
	}

	/*
	 * when codec suspend, then the register reset, if auto reset produce
	 * Pop & Click noise, then we should cut down the LINEOUT in this town.
	 */
	if (priv->enable) {
		snd_soc_update_bits(codec, AC_LINEOUT_CTL,
				(1<<LINEOUT_EN), (0<<LINEOUT_EN));
		snd_soc_update_bits(priv->codec, AC_LINEOUT_CTL,
				(1<<LINEL_SRC_EN), (0<<LINEL_SRC_EN));
		snd_soc_update_bits(priv->codec, AC_LINEOUT_CTL,
				(1<<LINER_SRC_EN), (0<<LINER_SRC_EN));
		priv->enable = 0;
	}

	pr_debug("Exit %s\n", __func__);

	return 0;
}

static int acx00_codec_resume(struct snd_soc_codec *codec)
{
	struct acx00_priv *priv = snd_soc_codec_get_drvdata(codec);

	pr_debug("Enter %s\n", __func__);

	if (clk_prepare_enable(priv->clk)) {
		dev_err(codec->dev, "codec resume clk failed\n");
		return -EBUSY;
	}

	schedule_delayed_work(&priv->resume_work, msecs_to_jiffies(300));

	if (priv->spk_gpio_used) {
		gpio_direction_output(priv->spk_gpio, 1);
		gpio_set_value(priv->spk_gpio, 0);
	}

	pr_debug("Exit %s\n", __func__);

	return 0;
}


static int acx00_codec_set_bias_level(struct snd_soc_codec *codec,
		enum snd_soc_bias_level level)
{
	codec->component.dapm.bias_level = level;
	return 0;
}

struct label {
	const char *name;
	int value;
};

#define LABEL(constant) { #constant, constant }
#define LABEL_END { NULL, -1 }

static struct label reg_labels[] = {
	LABEL(AC_SYS_CLK_CTL),
	LABEL(AC_SYS_MOD_RST),
	LABEL(AC_SYS_SR_CTL),
	LABEL(AC_I2S_CTL),
	LABEL(AC_I2S_CLK),
	LABEL(AC_I2S_FMT0),
	LABEL(AC_I2S_FMT1),
	LABEL(AC_I2S_MIXER_SRC),
	LABEL(AC_I2S_MIXER_GAIN),
	LABEL(AC_I2S_DAC_VOL),
	LABEL(AC_I2S_ADC_VOL),
	LABEL(AC_DAC_CTL),
	LABEL(AC_DAC_MIXER_SRC),
	LABEL(AC_DAC_MIXER_GAIN),
	LABEL(AC_OUT_MIXER_CTL),
	LABEL(AC_OUT_MIXER_SRC),
	LABEL(AC_LINEOUT_CTL),
	LABEL(AC_ADC_CTL),
	LABEL(AC_MICBIAS_CTL),
	LABEL(AC_ADC_MIC_CTL),
	LABEL(AC_ADC_MIXER_SRC),
	LABEL(AC_BIAS_CTL),
	LABEL(AC_ANALOG_PROF_CTL),
	LABEL_END,
};

static ssize_t show_audio_reg(struct device *dev,
			struct device_attribute *attr, char *buf)
{
	struct acx00_priv *priv = dev_get_drvdata(dev);
	int count = 0, i = 0;
	unsigned int reg_val;

	count += sprintf(buf, "dump audio reg:\n");

	while (reg_labels[i].name != NULL) {
		reg_val = acx00_reg_read(priv->acx00, reg_labels[i].value);
		count += sprintf(buf + count, "%s 0x%x: 0x%04x\n",
		reg_labels[i].name, (reg_labels[i].value), reg_val);
		i++;
	}

	return count;
}

/*
 * param 1: 0 read;1 write
 * param 2: 1 digital reg; 2 analog reg
 * param 3: reg value;
 * param 4: write value;
 * read:
 * echo 0,1,0x00> audio_reg
 * echo 0,2,0x00> audio_reg
 * write:
 * echo 1,1,0x00,0xa > audio_reg
 * echo 1,2,0x00,0xff > audio_reg
*/
static ssize_t store_audio_reg(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	int ret;
	int rw_flag;
	unsigned int input_reg_val = 0;
	int input_reg_group = 0;
	unsigned int input_reg_offset = 0;
	struct acx00_priv *priv = dev_get_drvdata(dev);

	ret = sscanf(buf, "%d,%d,0x%x,0x%x", &rw_flag, &input_reg_group,
			&input_reg_offset, &input_reg_val);
	dev_info(dev, "ret:%d, reg_group:%d, reg_offset:%d, reg_val:0x%x\n",
			ret, input_reg_group, input_reg_offset, input_reg_val);

	if (input_reg_group != 1) {
		pr_err("not exist reg group\n");
		ret = count;
		goto out;
	}
	if (!(rw_flag == 1 || rw_flag == 0)) {
		pr_err("not rw_flag\n");
		ret = count;
		goto out;
	}

	if (rw_flag) {
		acx00_reg_write(priv->acx00, input_reg_offset, input_reg_val);
	} else {
		input_reg_val = acx00_reg_read(priv->acx00, input_reg_offset);
		dev_info(dev, "\n\n Reg[0x%x] : 0x%04x\n\n",
				input_reg_offset, input_reg_val);
	}
	ret = count;

out:
	return ret;
}

static DEVICE_ATTR(audio_reg, 0644, show_audio_reg, store_audio_reg);

static struct attribute *audio_debug_attrs[] = {
	&dev_attr_audio_reg.attr,
	NULL,
};

static struct attribute_group audio_debug_attr_group = {
	.name   = "audio_reg_debug",
	.attrs  = audio_debug_attrs,
};

static struct snd_soc_codec_driver soc_codec_driver_acx00 = {
	.probe			= acx00_codec_probe,
	.remove			= acx00_codec_remove,
	.suspend		= acx00_codec_suspend,
	.resume			= acx00_codec_resume,
	.read			= acx00_codec_read,
	.write			= acx00_codec_write,
	.ignore_pmdown_time	= 1,
	.set_bias_level		= acx00_codec_set_bias_level,
};

/* through acx00 is part of mfd devices, after the mfd */
static int acx00_codec_dev_probe(struct platform_device *pdev)
{
	struct acx00_priv *priv;
	int ret;
	struct device_node *np = of_find_compatible_node(NULL, NULL,
				"allwinner,ac200_codec");

	priv = devm_kzalloc(&pdev->dev, sizeof(struct acx00_priv), GFP_KERNEL);
	if (!priv) {
		dev_err(&pdev->dev, "acx00 codec priv mem alloc failed\n");
		return -ENOMEM;
	}

	platform_set_drvdata(pdev, priv);
	priv->acx00 = dev_get_drvdata(pdev->dev.parent);

	if (np) {
		ret = of_get_named_gpio(np, "gpio-spk", 0);
		if (ret >= 0) {
			priv->spk_gpio_used = 1;
			priv->spk_gpio = ret;
			if (!gpio_is_valid(priv->spk_gpio)) {
				dev_err(&pdev->dev, "gpio-spk is valid\n");
				ret = -EINVAL;
				goto err_devm_kfree;
			} else {
				ret = devm_gpio_request(&pdev->dev,
				priv->spk_gpio, "SPK");
				if (ret) {
					dev_err(&pdev->dev,
						"failed request gpio-spk\n");
					ret = -EBUSY;
					goto err_devm_kfree;
				} else {
					gpio_direction_output(priv->spk_gpio, 1);
					gpio_set_value(priv->spk_gpio, 0);
					/* Export speaker gpio, so userspace can turn it on/off */
					gpio_export(priv->spk_gpio, false);
				}
			}
		} else {
			priv->spk_gpio_used = 0;
		}
	}

	ret = snd_soc_register_codec(&pdev->dev, &soc_codec_driver_acx00,
			acx00_codec_dai, ARRAY_SIZE(acx00_codec_dai));

	if (ret < 0)
		dev_err(&pdev->dev, "Failed register acx00: %d\n", ret);

	ret  = sysfs_create_group(&pdev->dev.kobj, &audio_debug_attr_group);
	if (ret)
		dev_warn(&pdev->dev, "failed to create attr group\n");

	return 0;

err_devm_kfree:
	devm_kfree(&pdev->dev, priv);
	return ret;
}

/* Mark this space to clear the LINEOUT & gpio */
static void acx00_codec_dev_shutdown(struct platform_device *pdev)
{
	struct acx00_priv *priv = platform_get_drvdata(pdev);

	if (priv->spk_gpio_used)
		gpio_set_value(priv->spk_gpio, 0);
}

static int acx00_codec_dev_remove(struct platform_device *pdev)
{
	struct acx00_priv *priv = platform_get_drvdata(pdev);

#ifndef ACX00_DAPM_LINEOUT
	snd_soc_update_bits(priv->codec, AC_LINEOUT_CTL,
			(1<<LINEOUT_EN), (0<<LINEOUT_EN));
#endif
	snd_soc_unregister_codec(&pdev->dev);
	clk_disable_unprepare(priv->clk);
	devm_kfree(&pdev->dev, priv);
	return 0;
}

static struct platform_driver acx00_codec_driver = {
	.driver = {
		.name = "acx00-codec",
		.owner = THIS_MODULE,
	},
	.probe = acx00_codec_dev_probe,
	.remove = acx00_codec_dev_remove,
	.shutdown = acx00_codec_dev_shutdown,
};

static int __init acx00_codec_driver_init(void)
{
	return platform_driver_register(&acx00_codec_driver);
}

static void __exit acx00_codec_driver_exit(void)
{
	platform_driver_unregister(&acx00_codec_driver);
}
late_initcall(acx00_codec_driver_init);
module_exit(acx00_codec_driver_exit);

MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("SUNXI ASoC ACX00 Codec Driver");
MODULE_AUTHOR("wolfgang huang");
MODULE_ALIAS("platform:acx00-codec");
