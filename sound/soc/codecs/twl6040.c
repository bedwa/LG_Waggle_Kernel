/*
 * ALSA SoC TWL6040 codec driver
 *
 * Author:	 Misael Lopez Cruz <x0052729@ti.com>
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA
 * 02110-1301 USA
 *
 */
#undef DEBUG
#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/init.h>
#include <linux/delay.h>
#include <linux/pm.h>
#include <linux/i2c.h>
#include <linux/gpio.h>
#include <linux/platform_device.h>
#include <linux/slab.h>
#include <linux/i2c/twl.h>
#include <linux/switch.h>
#include <linux/mfd/twl6040-codec.h>

#include <sound/core.h>
#include <sound/pcm.h>
#include <sound/pcm_params.h>
#include <sound/soc.h>
#include <sound/soc-dapm.h>
#include <sound/initval.h>
#include <sound/tlv.h>
#include <linux/wakelock.h>

/* LGE_CHANGE_S ty.lee@lge.com, 20101025 hook detection enable */
#include <sound/jack.h>
/* LGE_CHANGE_E ty.lee@lge.com */
#include "twl6040.h"

#define TWL6040_RATES		SNDRV_PCM_RATE_8000_96000
#define TWL6040_FORMATS	(SNDRV_PCM_FMTBIT_S32_LE)

#define TWL6040_OUTHS_0dB 0x00
#define TWL6040_OUTHS_M30dB 0x0F
#define TWL6040_OUTHF_0dB 0x03
#define TWL6040_OUTHF_M52dB 0x1D

#define TWL6040_RAMP_NONE	0
#define TWL6040_RAMP_UP		1
#define TWL6040_RAMP_DOWN	2

#define TWL6040_HSL_VOL_MASK    0x0F
#define TWL6040_HSL_VOL_SHIFT   0
#define TWL6040_HSR_VOL_MASK    0xF0
#define TWL6040_HSR_VOL_SHIFT   4
#define TWL6040_HF_VOL_MASK	0x1F
#define TWL6040_HF_VOL_SHIFT	0

// LGE LAB4 CH.PARK@LGE.COM 20110106 MUTE_CHECK
#define TWL6040_MUTE_DATA_MAX			20
#define TWL6040_I2C_RETRY_MAX			30
#define HOOK_DEBOUNCE_TIME				500

struct twl6040_mute_data{
	struct snd_soc_dai	*dai;
	int					mute;
};

// LGE LAB4 CH.PARK@LGE.COM 20110106 MUTE_CHECK
static struct twl6040_mute_data s_mute_data[TWL6040_MUTE_DATA_MAX];

struct twl6040_jack_data {
	struct snd_soc_jack *jack;
	int report;
	struct switch_dev sdev;
/* LGE_ADD_S [ty.lee@lge.com] 20101027, remain 2.6.32 */
	int state;
/* LGE_ADD_E [ty.lee@lge.com] 20101027, remain 2.6.32 */
/* LGE_CHANGE_S ty.lee@lge.com, 20110117, for longkey event count*/
	int longkey_cnt;
	struct input_dev *headset_input;
/* LGE_CHANGE_E ty.lee@lge.com */
};

/* codec private data */
struct twl6040_data {
	int audpwron;
	int naudint;
	struct twl6040_jack_data hs_jack;
	int codec_powered;
	int pll;
	int non_lp;
	int earpiece_used;
	unsigned int sysclk;
	struct snd_pcm_hw_constraint_list *sysclk_constraints;
	struct completion ready;
	struct work_struct audint_work;
	struct snd_soc_codec *codec;
/* LGE_ADD_S [ty.lee@lge.com] 20101020, Enable Headset detection */
	int intmask;
	struct delayed_work hsdet_dwork;
	struct delayed_work hook_work;
#if defined(CONFIG_MACH_LGE_COSMO_REV_A)
	unsigned int hsjack_gpio;
	unsigned int hsjack_irq;
#endif
/* LGE_ADD_E [ty.lee@lge.com] 20101020, Enable Headset detection */
	int dl_active;
	int ul_active;

	struct wake_lock wake_lock;
};

/*
 * twl6040 register cache & default register settings
 */
static const u8 twl6040_reg[TWL6040_CACHEREGNUM] = {
	0x00, /* not used		0x00	*/
	0x4B, /* TWL6040_ASICID (ro)	0x01	*/
	0x00, /* TWL6040_ASICREV (ro)	0x02	*/
	0x00, /* TWL6040_INTID		0x03	*/
	0x00, /* TWL6040_INTMR		0x04	*/
	0x00, /* TWL6040_NCPCTRL	0x05	*/
	0x00, /* TWL6040_LDOCTL		0x06	*/
	0x60, /* TWL6040_HPPLLCTL	0x07	*/
	0x00, /* TWL6040_LPPLLCTL	0x08	*/
	0x4A, /* TWL6040_LPPLLDIV	0x09	*/
#if 0
	0x00, /* TWL6040_AMICBCTL	0x0A	*/
#else
	0x44, /* TWL6040_AMICBCTL	0x0A	*/ // AMIC Bias output Pull-Down when Disabled(ty.lee@lge.com)
#endif
	0x00, /* TWL6040_DMICBCTL	0x0B	*/
	0x18, /* TWL6040_MICLCTL	0x0C	- No input selected on Left Mic */
	0x18, /* TWL6040_MICRCTL	0x0D	- No input selected on Right Mic */
	0x00, /* TWL6040_MICGAIN	0x0E	*/
	0x1B, /* TWL6040_LINEGAIN	0x0F	*/
#ifdef CONFIG_MACH_LGE_COSMO_REV_11
	0x00, /* TWL6040_HSLCTL		0x10	*/
#else
	0x04, /* TWL6040_HSLCTL		0x10	*/
#endif
	0x00, /* TWL6040_HSRCTL		0x11	*/
	0xFF, /* TWL6040_HSGAIN		0x12	*/
	0x1E, /* TWL6040_EARCTL		0x13	*/
	0x00, /* TWL6040_HFLCTL		0x14	*/
	0x1D, /* TWL6040_HFLGAIN	0x15	*/
	0x00, /* TWL6040_HFRCTL		0x16	*/
	0x1D, /* TWL6040_HFRGAIN	0x17	*/
	0x00, /* TWL6040_VIBCTLL	0x18	*/
	0x00, /* TWL6040_VIBDATL	0x19	*/
	0x00, /* TWL6040_VIBCTLR	0x1A	*/
	0x00, /* TWL6040_VIBDATR	0x1B	*/
	0x10, /* TWL6040_HKCTL1 	0x1C	*/ //0x00, ty.lee@lge.com     20100905, enable Hook button
	0xC7, /* TWL6040_HKCTL2		0x1D	*/ //0x00, ty.lee@lge.com 20110121, modify hook debounce, disable serial configuration hook
	0x02, /* TWL6040_GPOCTL		0x1E	*/
	0x00, /* TWL6040_ALB		0x1F	*/
	0x00, /* TWL6040_DLB		0x20	*/
	0x00, /* not used		0x21	*/
	0x00, /* not used		0x22	*/
	0x00, /* not used		0x23	*/
	0x00, /* not used		0x24	*/
	0x00, /* not used		0x25	*/
	0x00, /* not used		0x26	*/
	0x00, /* not used		0x27	*/
	0x00, /* TWL6040_TRIM1		0x28	*/
	0x00, /* TWL6040_TRIM2		0x29	*/
	0x00, /* TWL6040_TRIM3		0x2A	*/
	0x00, /* TWL6040_HSOTRIM	0x2B	*/
	0x00, /* TWL6040_HFOTRIM	0x2C	*/
	0x09, /* TWL6040_ACCCTL		0x2D	*/
	0x00, /* TWL6040_STATUS (ro)	0x2E	*/
};

/*
 * twl6040 vio/gnd registers:
 * registers under vio/gnd supply can be accessed
 * before the power-up sequence, after NRESPWRON goes high
 */
static const int twl6040_vio_reg[TWL6040_VIOREGNUM] = {
	TWL6040_REG_ASICID,
	TWL6040_REG_ASICREV,
	TWL6040_REG_INTID,
	TWL6040_REG_INTMR,
	TWL6040_REG_NCPCTL,
	TWL6040_REG_LDOCTL,
	TWL6040_REG_AMICBCTL,
	TWL6040_REG_DMICBCTL,
	TWL6040_REG_HKCTL1,
	TWL6040_REG_HKCTL2,
	TWL6040_REG_GPOCTL,
	TWL6040_REG_TRIM1,
	TWL6040_REG_TRIM2,
	TWL6040_REG_TRIM3,
	TWL6040_REG_HSOTRIM,
	TWL6040_REG_HFOTRIM,
	TWL6040_REG_ACCCTL,
	TWL6040_REG_STATUS,
};

/*
 * twl6040 vdd/vss registers:
 * registers under vdd/vss supplies can only be accessed
 * after the power-up sequence
 */
static const int twl6040_vdd_reg[TWL6040_VDDREGNUM] = {
	TWL6040_REG_HPPLLCTL,
	TWL6040_REG_LPPLLCTL,
	TWL6040_REG_LPPLLDIV,
	TWL6040_REG_MICLCTL,
	TWL6040_REG_MICRCTL,
	TWL6040_REG_MICGAIN,
	TWL6040_REG_LINEGAIN,
	TWL6040_REG_HSLCTL,
	TWL6040_REG_HSRCTL,
	TWL6040_REG_HSGAIN,
	TWL6040_REG_EARCTL,
	TWL6040_REG_HFLCTL,
	TWL6040_REG_HFLGAIN,
	TWL6040_REG_HFRCTL,
	TWL6040_REG_HFRGAIN,
	TWL6040_REG_VIBCTLL,
	TWL6040_REG_VIBDATL,
	TWL6040_REG_VIBCTLR,
	TWL6040_REG_VIBDATR,
	TWL6040_REG_ALB,
	TWL6040_REG_DLB,
};

//[20110125:geayoung.baek]AT%GKPD
extern int get_test_mode(void);
extern void write_gkpd_value(int value);
//[20110125:geayoung.baek]AT%GKPD


/*
 * read twl6040 register cache
 */
static inline unsigned int twl6040_read_reg_cache(struct snd_soc_codec *codec,
						unsigned int reg)
{
	u8 *cache = codec->reg_cache;

	if (reg >= TWL6040_CACHEREGNUM)
		return -EIO;

	return cache[reg];
}

/*
 * write twl6040 register cache
 */
static inline void twl6040_write_reg_cache(struct snd_soc_codec *codec,
						u8 reg, u8 value)
{
	u8 *cache = codec->reg_cache;

	if (reg >= TWL6040_CACHEREGNUM)
		return;
	cache[reg] = value;
}

static int twl6040_i2c_write(u8 reg, u8 value)
{
	int ret = -1;
	int cnt = 0;
	
	// sometimes, twl6040 make error. so, retry more...
	while( ret < 0 && cnt++ < TWL6040_I2C_RETRY_MAX ){
		ret = twl_i2c_write_u8(TWL_MODULE_AUDIO_VOICE, value, reg);
		if( ret < 0 )
			msleep(5);
	}

	return ret;
}

static int twl6040_i2c_read(u8 reg, u8* value)
{
	int ret = -1;
	int cnt = 0;
	
	// sometimes, twl6040 make error. so, retry more...
	while( ret < 0 && cnt++ < TWL6040_I2C_RETRY_MAX ){
		ret = twl_i2c_read_u8(TWL_MODULE_AUDIO_VOICE, value, reg);
		if( ret < 0 )
			msleep(5);
	}
	
	return ret;
}

/*
 * read from twl6040 hardware register
 */
static int twl6040_read_reg_volatile(struct snd_soc_codec *codec,
			unsigned int reg)
{
	u8 value = 0;

	if (reg >= TWL6040_CACHEREGNUM)
		return -EIO;

	twl6040_i2c_read(reg, &value);
	
	twl6040_write_reg_cache(codec, reg, value);

	return value;
}

/*
 * write to the twl6040 register space
 */
static int twl6040_write(struct snd_soc_codec *codec,
			unsigned int reg, unsigned int value)
{
	u8 *cache = codec->reg_cache;
	struct twl6040_data *priv = snd_soc_codec_get_drvdata(codec);

	if (reg >= TWL6040_CACHEREGNUM)
		return -EIO;

	// If no active dl, DAC should not be set!!
	if( reg == TWL6040_REG_HSLCTL || reg == TWL6040_REG_HSRCTL ){
		if( priv->dl_active == 0 && (value & 0x1)){
			twl6040_write_reg_cache(codec, reg, (value & ~0x1));
			return 0;
		}
	}

	if( reg == TWL6040_REG_HFLCTL || reg == TWL6040_REG_HFRCTL ){
		if( priv->dl_active == 0 && (value & 0x1)){
			twl6040_write_reg_cache(codec, reg, (value & ~0x1));
			return 0;
		}
	}


	if( reg == TWL6040_REG_EARCTL ){
		if( priv->dl_active == 0 && (value & 0x1)){
			twl6040_write_reg_cache(codec, reg, (value & ~0x01));
			return 0;
		}
	}
	
	twl6040_write_reg_cache(codec, reg, value);
	return twl6040_i2c_write(reg, value);
}

static void twl6040_init_vio_regs(struct snd_soc_codec *codec)
{
	u8 *cache = codec->reg_cache;
	int reg, i;

	/* allow registers to be accessed by i2c */
	twl6040_write(codec, TWL6040_REG_ACCCTL, cache[TWL6040_REG_ACCCTL]);

	for (i = 0; i < TWL6040_VIOREGNUM; i++) {
		reg = twl6040_vio_reg[i];
		/* skip read-only registers (ASICID, ASICREV, STATUS) */
		switch (reg) {
		case TWL6040_REG_ASICID:
		case TWL6040_REG_ASICREV:
		case TWL6040_REG_STATUS:
			continue;
		default:
			break;
		}
		twl6040_write(codec, reg, cache[reg]);
	}
}

static void twl6040_init_vdd_regs(struct snd_soc_codec *codec)
{
	u8 *cache = codec->reg_cache;
	int reg, i;

	for (i = 0; i < TWL6040_VDDREGNUM; i++) {
		reg = twl6040_vdd_reg[i];
		twl6040_write(codec, reg, cache[reg]);
	}
}

/* twl6040 codec manual power-up sequence */
static void twl6040_power_up(struct snd_soc_codec *codec)
{
	u8 ncpctl, ldoctl, lppllctl, accctl;

	ncpctl = twl6040_read_reg_cache(codec, TWL6040_REG_NCPCTL);
	ldoctl = twl6040_read_reg_cache(codec, TWL6040_REG_LDOCTL);
	lppllctl = twl6040_read_reg_cache(codec, TWL6040_REG_LPPLLCTL);
	accctl = twl6040_read_reg_cache(codec, TWL6040_REG_ACCCTL);

	/* enable reference system */
	ldoctl |= TWL6040_REFENA;
	twl6040_write(codec, TWL6040_REG_LDOCTL, ldoctl);
	msleep(10);
	/* enable internal oscillator */
	ldoctl |= TWL6040_OSCENA;
	twl6040_write(codec, TWL6040_REG_LDOCTL, ldoctl);
	udelay(10);
	/* enable high-side ldo */
	ldoctl |= TWL6040_HSLDOENA;
	twl6040_write(codec, TWL6040_REG_LDOCTL, ldoctl);
	udelay(244);
	/* enable negative charge pump */
	ncpctl |= TWL6040_NCPENA | TWL6040_NCPOPEN;
	twl6040_write(codec, TWL6040_REG_NCPCTL, ncpctl);
	udelay(488);
	/* enable low-side ldo */
	ldoctl |= TWL6040_LSLDOENA;
	twl6040_write(codec, TWL6040_REG_LDOCTL, ldoctl);
	udelay(244);
	/* enable low-power pll */
	lppllctl |= TWL6040_LPLLENA;
	twl6040_write(codec, TWL6040_REG_LPPLLCTL, lppllctl);
	/* reset state machine */
	accctl |= TWL6040_RESETSPLIT;
	twl6040_write(codec, TWL6040_REG_ACCCTL, accctl);
	mdelay(5);
	accctl &= ~TWL6040_RESETSPLIT;
	twl6040_write(codec, TWL6040_REG_ACCCTL, accctl);
	/* disable internal oscillator */
	ldoctl &= ~TWL6040_OSCENA;
	twl6040_write(codec, TWL6040_REG_LDOCTL, ldoctl);
}

/* twl6040 codec manual power-down sequence */
static void twl6040_power_down(struct snd_soc_codec *codec)
{
	u8 ncpctl, ldoctl, lppllctl, accctl;

	ncpctl = twl6040_read_reg_cache(codec, TWL6040_REG_NCPCTL);
	ldoctl = twl6040_read_reg_cache(codec, TWL6040_REG_LDOCTL);
	lppllctl = twl6040_read_reg_cache(codec, TWL6040_REG_LPPLLCTL);
	accctl = twl6040_read_reg_cache(codec, TWL6040_REG_ACCCTL);

	/* enable internal oscillator */
	ldoctl |= TWL6040_OSCENA;
	twl6040_write(codec, TWL6040_REG_LDOCTL, ldoctl);
	udelay(10);
	/* disable low-power pll */
	lppllctl &= ~TWL6040_LPLLENA;
	twl6040_write(codec, TWL6040_REG_LPPLLCTL, lppllctl);
	/* disable low-side ldo */
	ldoctl &= ~TWL6040_LSLDOENA;
	twl6040_write(codec, TWL6040_REG_LDOCTL, ldoctl);
	udelay(244);
	/* disable negative charge pump */
	ncpctl &= ~(TWL6040_NCPENA | TWL6040_NCPOPEN);
	twl6040_write(codec, TWL6040_REG_NCPCTL, ncpctl);
	udelay(488);
	/* disable high-side ldo */
	ldoctl &= ~TWL6040_HSLDOENA;
	twl6040_write(codec, TWL6040_REG_LDOCTL, ldoctl);
	udelay(244);
	/* disable internal oscillator */
	ldoctl &= ~TWL6040_OSCENA;
	twl6040_write(codec, TWL6040_REG_LDOCTL, ldoctl);
	/* disable reference system */
	ldoctl &= ~TWL6040_REFENA;
	twl6040_write(codec, TWL6040_REG_LDOCTL, ldoctl);
	msleep(10);
}

/* set headset dac and driver power mode */
static int headset_power_mode(struct snd_soc_codec *codec, int high_perf)
{
	int hslctl, hsrctl;
	int mask = TWL6040_HSDRVMODEL | TWL6040_HSDACMODEL;
	struct twl6040_data *priv = snd_soc_codec_get_drvdata(codec);

	/* Earphone doesn't support low power mode */
	high_perf |= priv->earpiece_used;

	hslctl = twl6040_read_reg_cache(codec, TWL6040_REG_HSLCTL);
	hsrctl = twl6040_read_reg_cache(codec, TWL6040_REG_HSRCTL);

	if (high_perf) {
		hslctl &= ~mask;
		hsrctl &= ~mask;
	} else {
		hslctl |= mask;
		hsrctl |= mask;
	}

	twl6040_write(codec, TWL6040_REG_HSLCTL, hslctl);
	twl6040_write(codec, TWL6040_REG_HSRCTL, hsrctl);

	return 0;
}

static int twl6040_hs_dac_event(struct snd_soc_dapm_widget *w,
			struct snd_kcontrol *kcontrol, int event)
{
	mdelay(1);    // LGE P3SW4 CH.PARK@LGE.COM 20110605 CTS Audio COLD START SPEC
	return 0;
}

static int twl6040_power_mode_event(struct snd_soc_dapm_widget *w,
			struct snd_kcontrol *kcontrol, int event)
{
	struct snd_soc_codec *codec = w->codec;
	struct twl6040_data *priv = snd_soc_codec_get_drvdata(codec);

	if (SND_SOC_DAPM_EVENT_ON(event)) {
		priv->non_lp++;
		if (!strcmp(w->name,"Earphone Driver")) {
			/* Earphone doesn't support low power mode */
			priv->earpiece_used = 1;
			headset_power_mode(w->codec, 1);
		}
	} else {
		priv->non_lp--;
		if (!strcmp(w->name,"Earphone Driver")) {
			priv->earpiece_used = 0;
		}
	}

	mdelay(1);    // LGE P3SW4 CH.PARK@LGE.COM 20110605 CTS Audio COLD START SPEC

	return 0;
}

/* LGE_CHANGE_S ty.lee@lge.com */
static void twl6040_hs_hook_detect_work(struct work_struct *work)
{
	struct twl6040_data *priv;
	struct snd_soc_codec *codec;
	struct twl6040_jack_data *jack;
/* LGE_ADD_S [ty.lee@lge.com] 20110118, for long key event */
	struct input_dev *ip_dev;
/* LGE_ADD_E [ty.lee@lge.com] 20110118 */

	priv = container_of(work, struct twl6040_data, hook_work.work);
	codec = priv->codec;
	jack = &priv->hs_jack;
/* LGE_ADD_S [ty.lee@lge.com] 20110118, for long key event */
	ip_dev = jack->headset_input;
/* LGE_ADD_E [ty.lee@lge.com] 20110118 */

	if (jack->jack)
	{
		/* LGE_CHANGE_S ty.lee@lge.com, 20101221, for longkey event*/
		if(jack->longkey_cnt == 0){
			input_report_key(ip_dev, KEY_HOOK, 1);
			input_sync(ip_dev);
		}
		if(is_without_mic()){
			if(jack->longkey_cnt < 6000){//10 min.
				schedule_delayed_work(&priv->hook_work, msecs_to_jiffies(100));
				jack->longkey_cnt++;
			}
			else{//long key event
				input_report_key(ip_dev, KEY_HOOK, 0);
				input_sync(ip_dev);
				jack->longkey_cnt = 0;
				//[20110125:geayoung.baek]AT%GKPD
				if(get_test_mode() == 1)
				{
					write_gkpd_value(KEY_HOOK);
				}
				//[20110125:geayoung.baek]AT%GKPD
			}
		}
		else{//short key event
			input_report_key(ip_dev, KEY_HOOK, 0);
			input_sync(ip_dev);
			jack->longkey_cnt = 0;

			//[20110125:geayoung.baek]AT%GKPD
			if(get_test_mode() == 1)
			{
				write_gkpd_value(KEY_HOOK);
			}
			//[20110125:geayoung.baek]AT%GKPD
		}
		/* LGE_CHANGE_E ty.lee@lge.com */
	}
}

static void twl6040_hs_jack_detect_dwork(struct work_struct *dwork)
{
	struct twl6040_data *priv;
	struct snd_soc_codec *codec;
	struct twl6040_jack_data *jack;
	int status = 0;

	priv = container_of(dwork, struct twl6040_data, hsdet_dwork.work);
	codec = priv->codec;
	jack = &priv->hs_jack;

	/*
	 * Early interrupt, CODEC driver cannot report jack status
	 * since jack is not registered yet. MACHINE driver will
	 * register jack and report status through twl6040_hs_jack_detect
	 */
	if (jack->jack) {
#if defined(CONFIG_MACH_LGE_COSMO_REV_A)
		if(gpio_get_value(priv->hsjack_gpio))
#else 
		if(jack->state)
#endif
		{		
			u8 val = 0;
			if( twl6040_i2c_read(TWL6040_REG_AMICBCTL, &val) == 0 ){
				if( val != twl6040_read_reg_cache(codec, TWL6040_REG_AMICBCTL) )
					twl6040_i2c_write(TWL6040_REG_AMICBCTL, twl6040_read_reg_cache(codec, TWL6040_REG_AMICBCTL)); 
			}

			hs_set_bias(codec, 1);
			set_hook_enable(codec, 1);
			mdelay(200);

			if( twl6040_i2c_read(TWL6040_REG_MICLCTL, &val) == 0 ){
				if( val != twl6040_read_reg_cache(codec, TWL6040_REG_MICLCTL) )
					twl6040_i2c_write(TWL6040_REG_MICLCTL, twl6040_read_reg_cache(codec, TWL6040_REG_MICLCTL)); 
			}
			if( twl6040_i2c_read(TWL6040_REG_MICRCTL, &val) == 0 ){
				if( val != twl6040_read_reg_cache(codec, TWL6040_REG_MICRCTL) )
					twl6040_i2c_write(TWL6040_REG_MICRCTL, twl6040_read_reg_cache(codec, TWL6040_REG_MICRCTL)); 
			}

			if(is_without_mic()){
				set_hook_enable(codec, 0);
				hs_set_bias(codec, 0);
				jack->state = WIRED_HEADPHONE;//wired headset without MIC
				status = SND_JACK_HEADPHONE;
			}
			else{
				/* LGE_CHANGE_S ty.lee@lge.com 20110106, enable Hook interrupt
				 * for wired_headset */
				if(priv->intmask & TWL6040_HOOKMSK){
					priv->intmask &= ~TWL6040_HOOKMSK;
					twl6040_write(codec, TWL6040_REG_INTMR, priv->intmask);
				}
				/* LGE_CHANGE_E ty.lee@lge.com */
				status = jack->report;//SND_JACK_HEADSET
			}
		}
		else{
			/* LGE_CHANGE_S ty.lee@lge.com 20110106, disable hook interrupt
			 * when headset removed */
			if(!(priv->intmask & TWL6040_HOOKMSK)) {
				priv->intmask |= TWL6040_HOOKMSK;
				twl6040_write(codec, TWL6040_REG_INTMR, priv->intmask);
			}
			/* LGE_CHANGE_E ty.lee@lge.com */
			hs_set_bias(codec, 0);
			status = 0;
		}
		snd_soc_jack_report(jack->jack, status, jack->report);
	}
	switch_set_state(&jack->sdev, jack->state);
}

int is_without_mic(void)
{
	u8 state = 0;

	twl6040_i2c_read(TWL6040_REG_STATUS, &state);
	state &= TWL6040_HKCOMP;
	return state;
}
/* LGE_CHANGE_E ty.lee@lge.com */

/* audio interrupt handler */
static irqreturn_t twl6040_naudint_handler(int irq, void *data)
{
	static uint64_t tick = 0;
	struct snd_soc_codec *codec = data;
	struct twl6040_data *priv = snd_soc_codec_get_drvdata(codec);
	struct twl6040_jack_data *jack = &priv->hs_jack;
	u8 intid = 0;

	twl6040_i2c_read(TWL6040_REG_INTID, &intid);

	if (intid & TWL6040_THINT)
		dev_alert(codec->dev, "die temp over-limit detection\n");

	if (intid & TWL6040_UNPLUGINT)
	{
		wake_lock_timeout(&priv->wake_lock, 2 * HZ);
		
		tick = jiffies;
		jack->state = WIRED_HEADSET;
		cancel_delayed_work(&priv->hook_work);
		cancel_delayed_work(&priv->hsdet_dwork);
		schedule_delayed_work(&priv->hsdet_dwork, msecs_to_jiffies(200));
	}

	if (intid & TWL6040_PLUGINT)
	{
		u8 val;
		u8 mute = 0;
		
		wake_lock_timeout(&priv->wake_lock, 2 * HZ);
		
//LGE_S jin333.kim@lge.com 20110227 { Tx noise is occured when the headset is unplugged 	
#if 1	// ch.park@lge.com 20110530 MIC OFF
		if( twl6040_i2c_read(TWL6040_REG_MICLCTL, &val) == 0 ){
			if( !(val & 0x18) ){
				twl6040_i2c_write(TWL6040_REG_MICLCTL, 0x18); 	
				mute = 1;
			}
		}
		if( twl6040_i2c_read(TWL6040_REG_MICRCTL, &val) == 0 ){
			if( !(val & 0x18) ){
				twl6040_i2c_write(TWL6040_REG_MICRCTL, 0x18); 			
				mute = 1;
			}
		}
		if( mute ) twl6040_i2c_write(TWL6040_REG_AMICBCTL, 0x00); 
#endif
//LGE_E jin333.kim@lge.com 20110227 } 
		tick = jiffies;
		jack->state = HEADSET_NONE;
		set_hook_enable(codec, 0);
		cancel_delayed_work(&priv->hook_work);
		cancel_delayed_work(&priv->hsdet_dwork);
		schedule_delayed_work(&priv->hsdet_dwork, msecs_to_jiffies(200));
	}

	/* LGE_CHANGE_S ty.lee@lge.com */
	if (intid & TWL6040_HOOKINT){
		if( jack->state > 0 && (tick == 0 || jiffies_to_msecs(jiffies-tick) > 500ul) )
		{
			tick = 0;
			printk(KERN_ERR" [AUD] %s - HOOKINT\n", __func__);
			schedule_delayed_work(&priv->hook_work, 0);
		}
	}
	/* LGE_CHANGE_E ty.lee@lge.com */

	if (intid & TWL6040_HFINT)
		dev_alert(codec->dev, "hf drivers over current detection\n");

	if (intid & TWL6040_VIBINT)
		dev_alert(codec->dev, "vib drivers over current detection\n");

	if (intid & TWL6040_READYINT)
		complete(&priv->ready);
	
	return IRQ_HANDLED;
}

/*
 * MICATT volume control:
 * from -6 to 0 dB in 6 dB steps
 */
static DECLARE_TLV_DB_SCALE(mic_preamp_tlv, -600, 600, 0);

/*
 * MICGAIN volume control:
 * from -6 to 30 dB in 6 dB steps
 */
static DECLARE_TLV_DB_SCALE(mic_amp_tlv, -600, 600, 0);

/*
 * AFMGAIN volume control:
 * from 18 to 24 dB in 6 dB steps
 */
static DECLARE_TLV_DB_SCALE(afm_amp_tlv, 1800, 600, 0);


/*
 * HSGAIN volume control:
 * from -30 to 0 dB in 2 dB steps
 */
static DECLARE_TLV_DB_SCALE(hs_tlv, -3000, 200, 0);

/*
 * HFGAIN volume control:
 * from -52 to 6 dB in 2 dB steps
 */
static DECLARE_TLV_DB_SCALE(hf_tlv, -5200, 200, 0);

/*
 * EPGAIN volume control:
 * from -24 to 6 dB in 2 dB steps
 */
static DECLARE_TLV_DB_SCALE(ep_tlv, -2400, 200, 0);

/* Left analog microphone selection */
static const char *twl6040_amicl_texts[] =
	{"Headset Mic", "Main Mic", "Aux/FM Left", "Off"};

/* Right analog microphone selection */
static const char *twl6040_amicr_texts[] =
	{"Headset Mic", "Sub Mic", "Aux/FM Right", "Off"};

static const char *twl6040_hs_texts[] =
	{"Off", "HS DAC", "Line-In amp"};

static const unsigned int twl6040_hsl_value[] = 
	{0, 9, 17, 1};

// LGE LAB4 CH.PARK@LGE.COM 20110106 TTY
static const char *twl6040_hsr_texts[] =
	{"Off", "HS DAC", "Line-In amp", "No use", "No use", "Left-to-Right"};

static const char *twl6040_hf_texts[] =
	{"Off", "HF DAC", "Line-In amp"};

static const struct soc_enum twl6040_enum[] = {
	SOC_ENUM_SINGLE(TWL6040_REG_MICLCTL, 3, 4, twl6040_amicl_texts),
	SOC_ENUM_SINGLE(TWL6040_REG_MICRCTL, 3, 4, twl6040_amicr_texts),
	SOC_ENUM_SINGLE(TWL6040_REG_HSLCTL, 5, 3, twl6040_hs_texts),
// LGE LAB4 CH.PARK@LGE.COM 20110106 TTY
	SOC_ENUM_SINGLE(TWL6040_REG_HSRCTL, 5, 6, twl6040_hsr_texts),
	SOC_ENUM_SINGLE(TWL6040_REG_HFLCTL, 2, 3, twl6040_hf_texts),
	SOC_ENUM_SINGLE(TWL6040_REG_HFRCTL, 2, 3, twl6040_hf_texts),
};

static const struct snd_kcontrol_new amicl_control =
	SOC_DAPM_ENUM("Route", twl6040_enum[0]);

static const struct snd_kcontrol_new amicr_control =
	SOC_DAPM_ENUM("Route", twl6040_enum[1]);

/* Headset DAC playback switches */
static const struct snd_kcontrol_new hsl_mux_controls =
	SOC_DAPM_ENUM("Route", twl6040_enum[2]);

static const struct snd_kcontrol_new hsr_mux_controls =
	SOC_DAPM_ENUM("Route", twl6040_enum[3]);

/* Handsfree DAC playback switches */
static const struct snd_kcontrol_new hfl_mux_controls =
	SOC_DAPM_ENUM("Route", twl6040_enum[4]);

static const struct snd_kcontrol_new hfr_mux_controls =
	SOC_DAPM_ENUM("Route", twl6040_enum[5]);

static const struct snd_kcontrol_new ep_driver_switch_controls =
	SOC_DAPM_SINGLE("Switch", TWL6040_REG_EARCTL, 0, 1, 0);

static const struct snd_kcontrol_new twl6040_snd_controls[] = {
	/* Capture gains */
	SOC_DOUBLE_TLV("Capture Preamplifier Volume",
		TWL6040_REG_MICGAIN, 6, 7, 1, 1, mic_preamp_tlv),
	SOC_DOUBLE_TLV("Capture Volume",
		TWL6040_REG_MICGAIN, 0, 3, 4, 0, mic_amp_tlv),

	/* AFM gains */
	SOC_DOUBLE_TLV("Aux FM Volume",
		TWL6040_REG_LINEGAIN, 0, 4, 0xF, 0, afm_amp_tlv),

	/* Playback gains */
	SOC_DOUBLE_TLV("Headset Playback Volume",
		TWL6040_REG_HSGAIN, 0, 4, 0xF, 1, hs_tlv),
	SOC_DOUBLE_R_TLV("Handsfree Playback Volume",
		TWL6040_REG_HFLGAIN, TWL6040_REG_HFRGAIN, 0, 0x1D, 1, hf_tlv),
	SOC_SINGLE_TLV("Earphone Playback Volume",
		TWL6040_REG_EARCTL, 1, 0xF, 1, ep_tlv),
};

static const struct snd_soc_dapm_widget twl6040_dapm_widgets[] = {
	/* Inputs */
	SND_SOC_DAPM_INPUT("MAINMIC"),
	SND_SOC_DAPM_INPUT("HSMIC"),
	SND_SOC_DAPM_INPUT("SUBMIC"),
	SND_SOC_DAPM_INPUT("AFML"),
	SND_SOC_DAPM_INPUT("AFMR"),

	/* Outputs */
	SND_SOC_DAPM_OUTPUT("HSOL"),
	SND_SOC_DAPM_OUTPUT("HSOR"),
	SND_SOC_DAPM_OUTPUT("HFL"),
	SND_SOC_DAPM_OUTPUT("HFR"),
	SND_SOC_DAPM_OUTPUT("EP"),

	/* Analog input muxes for the capture amplifiers */
	SND_SOC_DAPM_MUX("Analog Left Capture Route",
			SND_SOC_NOPM, 0, 0, &amicl_control),
	SND_SOC_DAPM_MUX("Analog Right Capture Route",
			SND_SOC_NOPM, 0, 0, &amicr_control),

	/* Analog capture PGAs */
	SND_SOC_DAPM_PGA("MicAmpL",
			TWL6040_REG_MICLCTL, 0, 0, NULL, 0),
	SND_SOC_DAPM_PGA("MicAmpR",
			TWL6040_REG_MICRCTL, 0, 0, NULL, 0),

	/* Auxiliary FM PGAs */
	SND_SOC_DAPM_PGA("AFMAmpL",
			TWL6040_REG_MICLCTL, 1, 0, NULL, 0),
	SND_SOC_DAPM_PGA("AFMAmpR",
			TWL6040_REG_MICRCTL, 1, 0, NULL, 0),

	/* ADCs */
	SND_SOC_DAPM_ADC("ADC Left", "Left Front Capture",
			TWL6040_REG_MICLCTL, 2, 0),
	SND_SOC_DAPM_ADC("ADC Right", "Right Front Capture",
			TWL6040_REG_MICRCTL, 2, 0),

	/* Microphone bias */
// LGE LAB4 CH.PARK@LGE.COM 20110213 On a few device, headset noise is occured.
	SND_SOC_DAPM_MICBIAS("Headset Mic Bias",
			SND_SOC_NOPM, 0, 0),
	SND_SOC_DAPM_MICBIAS("Main Mic Bias",
			TWL6040_REG_AMICBCTL, 4, 0),
	SND_SOC_DAPM_MICBIAS("Digital Mic1 Bias",
			TWL6040_REG_DMICBCTL, 0, 0),
	SND_SOC_DAPM_MICBIAS("Digital Mic2 Bias",
			TWL6040_REG_DMICBCTL, 4, 0),

	/* DACs */
	SND_SOC_DAPM_DAC_E("HSDAC Left", "Headset Playback",
			TWL6040_REG_HSLCTL, 0, 0,
			twl6040_hs_dac_event,
			SND_SOC_DAPM_POST_PMU | SND_SOC_DAPM_POST_PMD),
	SND_SOC_DAPM_DAC_E("HSDAC Right", "Headset Playback",
			TWL6040_REG_HSRCTL, 0, 0,
			twl6040_hs_dac_event,
			SND_SOC_DAPM_POST_PMU | SND_SOC_DAPM_POST_PMD),
	SND_SOC_DAPM_DAC_E("HFDAC Left", "Handsfree Playback",
			TWL6040_REG_HFLCTL, 0, 0,
			twl6040_power_mode_event,
			SND_SOC_DAPM_POST_PMU | SND_SOC_DAPM_POST_PMD),
	SND_SOC_DAPM_DAC_E("HFDAC Right", "Handsfree Playback",
			TWL6040_REG_HFRCTL, 0, 0,
			twl6040_power_mode_event,
			SND_SOC_DAPM_POST_PMU | SND_SOC_DAPM_POST_PMD),

	SND_SOC_DAPM_MUX("HF Left Playback",
			SND_SOC_NOPM, 0, 0, &hfl_mux_controls),
	SND_SOC_DAPM_MUX("HF Right Playback",
			SND_SOC_NOPM, 0, 0, &hfr_mux_controls),
	/* Analog playback Muxes */
	SND_SOC_DAPM_MUX("HS Left Playback",
			SND_SOC_NOPM, 0, 0, &hsl_mux_controls),
	SND_SOC_DAPM_MUX("HS Right Playback",
			SND_SOC_NOPM, 0, 0, &hsr_mux_controls),

	/* Analog playback drivers */
	SND_SOC_DAPM_DRV_E("Handsfree Left Driver",
			TWL6040_REG_HFLCTL, 4, 0, NULL, 0,
			twl6040_power_mode_event,
			SND_SOC_DAPM_POST_PMU | SND_SOC_DAPM_POST_PMD),
	SND_SOC_DAPM_DRV_E("Handsfree Right Driver",
			TWL6040_REG_HFRCTL, 4, 0, NULL, 0,
			twl6040_power_mode_event,
			SND_SOC_DAPM_POST_PMU | SND_SOC_DAPM_POST_PMD),
#ifdef CONFIG_MACH_LGE_COSMO_REV_11
	SND_SOC_DAPM_DRV("Headset Left Driver",
			TWL6040_REG_HSLCTL, 2, 0, NULL, 0),
#else			
	SND_SOC_DAPM_DRV("Headset Left Driver",
			SND_SOC_NOPM, 0, 0, NULL, 0),
#endif
	SND_SOC_DAPM_DRV("Headset Right Driver",
			TWL6040_REG_HSRCTL, 2, 0, NULL, 0),
	SND_SOC_DAPM_SWITCH_E("Earphone Driver",
			SND_SOC_NOPM, 0, 0, &ep_driver_switch_controls,
			twl6040_power_mode_event,
			SND_SOC_DAPM_POST_PMU | SND_SOC_DAPM_POST_PMD),

	/* Analog playback PGAs */
	SND_SOC_DAPM_PGA("HFDAC Left PGA",
			TWL6040_REG_HFLCTL, 1, 0, NULL, 0),
	SND_SOC_DAPM_PGA("HFDAC Right PGA",
			TWL6040_REG_HFRCTL, 1, 0, NULL, 0),

};

static const struct snd_soc_dapm_route intercon[] = {
	/* Capture path */
	{"Analog Left Capture Route", "Headset Mic", "HSMIC"},
	{"Analog Left Capture Route", "Main Mic", "MAINMIC"},
	{"Analog Left Capture Route", "Aux/FM Left", "AFML"},

	{"Analog Right Capture Route", "Headset Mic", "HSMIC"},
	{"Analog Right Capture Route", "Sub Mic", "SUBMIC"},
	{"Analog Right Capture Route", "Aux/FM Right", "AFMR"},

	{"MicAmpL", NULL, "Analog Left Capture Route"},
	{"MicAmpR", NULL, "Analog Right Capture Route"},

	{"ADC Left", NULL, "MicAmpL"},
	{"ADC Right", NULL, "MicAmpR"},

	/* AFM path */
#ifdef CONFIG_MACH_LGE_COSMOPOLITAN
	{"AFMAmpL", NULL, "AFML"},
	{"AFMAmpR", NULL, "AFMR"},
#else
	{"AFMAmpL", "NULL", "AFML"},
	{"AFMAmpR", "NULL", "AFMR"},
#endif

	{"HS Left Playback", "HS DAC", "HSDAC Left"},
	{"HS Left Playback", "Line-In amp", "AFMAmpL"},

	{"HS Right Playback", "HS DAC", "HSDAC Right"},
	{"HS Right Playback", "Line-In amp", "AFMAmpR"},
	{"HS Right Playback", "Left-to-Right", "HSDAC Right"},

#ifdef CONFIG_MACH_LGE_COSMOPOLITAN
	{"Headset Left Driver", NULL, "HS Left Playback"},
	{"Headset Right Driver", NULL, "HS Right Playback"},
#else
	{"Headset Left Driver", "NULL", "HS Left Playback"},
	{"Headset Right Driver", "NULL", "HS Right Playback"},
#endif

	{"HSOL", NULL, "Headset Left Driver"},
	{"HSOR", NULL, "Headset Right Driver"},

	/* Earphone playback path */
	{"Earphone Driver", "Switch", "HSDAC Left"},
	{"EP", NULL, "Earphone Driver"},

	{"HF Left Playback", "HF DAC", "HFDAC Left"},
	{"HF Left Playback", "Line-In amp", "AFMAmpL"},

	{"HF Right Playback", "HF DAC", "HFDAC Right"},
	{"HF Right Playback", "Line-In amp", "AFMAmpR"},

	{"HFDAC Left PGA", NULL, "HF Left Playback"},
	{"HFDAC Right PGA", NULL, "HF Right Playback"},

#ifdef CONFIG_MACH_LGE_COSMOPOLITAN
	{"Handsfree Left Driver", NULL, "HFDAC Left PGA"},
	{"Handsfree Right Driver", NULL, "HFDAC Right PGA"},
#else
	{"Handsfree Left Driver", "Switch", "HFDAC Left PGA"},
	{"Handsfree Right Driver", "Switch", "HFDAC Right PGA"},
#endif
	{"HFL", NULL, "Handsfree Left Driver"},
	{"HFR", NULL, "Handsfree Right Driver"},
};

static int twl6040_add_widgets(struct snd_soc_codec *codec)
{
	snd_soc_dapm_new_controls(codec->dapm, twl6040_dapm_widgets,
				 ARRAY_SIZE(twl6040_dapm_widgets));

	snd_soc_dapm_add_routes(codec->dapm, intercon, ARRAY_SIZE(intercon));

	snd_soc_dapm_new_widgets(codec->dapm);

	return 0;
}

static int twl6040_power_up_completion(struct snd_soc_codec *codec,
					int naudint)
{
	struct twl6040_data *priv = snd_soc_codec_get_drvdata(codec);
	int time_left;
	u8 intid = 0;

	time_left = wait_for_completion_timeout(&priv->ready,
				msecs_to_jiffies(500));	// waitting more....

	if (!time_left) {
		twl6040_i2c_read(TWL6040_REG_INTID, &intid);
		if (!(intid & TWL6040_READYINT)) {
			dev_err(codec->dev, "timeout waiting for READYINT\n");
			return -ETIMEDOUT;
		}
	}

	priv->codec_powered = 1;

	return 0;
}

static int twl6040_set_bias_level(struct snd_soc_codec *codec,
				enum snd_soc_bias_level level)
{
	struct twl6040_data *priv = snd_soc_codec_get_drvdata(codec);
	int audpwron = priv->audpwron;
	int naudint = priv->naudint;
	int ret;

	switch (level) {
	case SND_SOC_BIAS_ON:
		break;
	case SND_SOC_BIAS_PREPARE:
		break;
	case SND_SOC_BIAS_STANDBY:
		if (priv->codec_powered)
			break;

		if (gpio_is_valid(audpwron)) {
			/* use AUDPWRON line */
			gpio_set_value(audpwron, 1);

//LGE_CHANGE_S [kibum.lee@lge.com] 2011-02-01, common : Rev.C off mode -> wakeup i2c error
#if defined(CONFIG_MACH_LGE_COSMO_REV_C)
			mdelay(50);
#endif
//LGE_CHANGE_E [kibum.lee@lge.com] 2011-02-01, common : Rev.C off mode -> wakeup i2c error

			/* wait for power-up completion */
			ret = twl6040_power_up_completion(codec, naudint);
			if (ret)
				return ret;

			/* sync registers updated during power-up sequence */
			twl6040_read_reg_volatile(codec, TWL6040_REG_NCPCTL);
			twl6040_read_reg_volatile(codec, TWL6040_REG_LDOCTL);
			twl6040_read_reg_volatile(codec, TWL6040_REG_LPPLLCTL);
		} else {
			/* use manual power-up sequence */
			twl6040_power_up(codec);
			priv->codec_powered = 1;
		}

		/* initialize vdd/vss registers with reg_cache */
		twl6040_init_vdd_regs(codec);
		break;
	case SND_SOC_BIAS_OFF:
		if (!priv->codec_powered)
			break;

		if (gpio_is_valid(audpwron)) {			
			// Headset Left Driver make pop-noise when twl6040 enter into sleep mode.
			u8 val;
			val = twl6040_read_reg_cache(codec, TWL6040_REG_HSLCTL);
			if( val & 0x04 )	// temp disableed. It will be recovered when power-on.
			{
				twl6040_i2c_write(TWL6040_REG_HSLCTL, (val & (~0x04)));
				msleep(1);	// delay;;
			}

			/* use AUDPWRON line */
			gpio_set_value(audpwron, 0);

			/* power-down sequence latency */
			mdelay(1);	// more more...

			/* sync registers updated during power-down sequence */
			twl6040_read_reg_volatile(codec, TWL6040_REG_NCPCTL);
			twl6040_read_reg_volatile(codec, TWL6040_REG_LDOCTL);
			twl6040_write_reg_cache(codec, TWL6040_REG_LPPLLCTL,
						0x00);
		} else {
			/* use manual power-down sequence */
			twl6040_power_down(codec);
		}

		priv->codec_powered = 0;
		break;
	}

	codec->dapm->bias_level = level;

	return 0;
}

/* set of rates for each pll: low-power and high-performance */

static unsigned int lp_rates[] = {
	88200,
	96000,
};

static struct snd_pcm_hw_constraint_list lp_constraints = {
	.count	= ARRAY_SIZE(lp_rates),
	.list	= lp_rates,
};

static unsigned int hp_rates[] = {
	96000,
};

static struct snd_pcm_hw_constraint_list hp_constraints = {
	.count	= ARRAY_SIZE(hp_rates),
	.list	= hp_rates,
};

static int twl6040_startup(struct snd_pcm_substream *substream,
			struct snd_soc_dai *dai)
{
// LGE LAB4 CH.PARK@LGE.COM 20110106 MUTE_CHECK
	struct snd_soc_codec *codec = dai ? (dai->codec) : 0;
	struct twl6040_data *priv = snd_soc_codec_get_drvdata(codec);
	int idx;
	int cnt = 0;
	int bSet = 0;
	
	for( idx = 0 ; idx < TWL6040_MUTE_DATA_MAX ; idx++ )
	{
		if( s_mute_data[idx].dai == 0 && !bSet )
		{
			bSet = 1;
			s_mute_data[idx].dai = dai;
			s_mute_data[idx].mute = 0;
		}

		if( s_mute_data[idx].dai )
			cnt++;
	}

	if( substream->stream == SNDRV_PCM_STREAM_CAPTURE )
		priv->ul_active++;
	if( substream->stream == SNDRV_PCM_STREAM_PLAYBACK )
		priv->dl_active++;
	
//	if( cnt == 1 ){	// reset TWL6040
//		twl6040_set_bias_level(dai->codec, SND_SOC_BIAS_OFF);
//		mdelay(1);
//		twl6040_set_bias_level(dai->codec, SND_SOC_BIAS_STANDBY);		
//	}
// FIXME - need to create some contraints for backends
#if 0
	struct snd_soc_pcm_runtime *rtd = substream->private_data;
	struct snd_soc_codec *codec = rtd->codec;
	struct twl6040_data *priv = snd_soc_codec_get_drvdata(codec);

	snd_pcm_hw_constraint_list(substream->runtime, 0,
				SNDRV_PCM_HW_PARAM_RATE,
				priv->sysclk_constraints);
#endif
	return 0;
}

// LGE LAB4 CH.PARK@LGE.COM 20110106 MUTE_CHECK
static int twl6040_shutdown(struct snd_pcm_substream *substream,
			struct snd_soc_dai *dai)
{
	struct snd_soc_codec *codec = dai ? (dai->codec) : 0;
	struct twl6040_data *priv = snd_soc_codec_get_drvdata(codec);

	int idx;
	int cnt = 0;
	
	for( idx = 0 ; idx < TWL6040_MUTE_DATA_MAX ; idx++ )
	{
		if( s_mute_data[idx].dai == dai )
		{
			s_mute_data[idx].dai = 0;
			s_mute_data[idx].mute = 0;
		}

		if( s_mute_data[idx].dai )
			cnt++;
	}

	if( substream->stream == SNDRV_PCM_STREAM_CAPTURE )
		priv->ul_active--;
	if( substream->stream == SNDRV_PCM_STREAM_PLAYBACK )
		priv->dl_active--;
		
// LGE LAB4 CH.PARK@LGE.COM 20110111 AUDIO_CRASH
	if( cnt == 0 ) 
	{
		// FIXME : T_T 
		// TWL6040 DAC is going crazy. No way to recover. So, I decide to reset twl6040 here. (normal -> sleep -> normal)
		// reset twl6040
		if( codec->dapm->bias_level != SND_SOC_BIAS_OFF ){
			struct twl6040_data *priv = snd_soc_codec_get_drvdata(codec);
			enum snd_soc_bias_level level = codec->dapm->bias_level;
			printk(KERN_ERR "twl6040_shutdown reset!! start\n");
			twl6040_set_bias_level(codec, SND_SOC_BIAS_OFF);	// normal -> sleep
			msleep(5);	// more more...
			twl6040_set_bias_level(codec, SND_SOC_BIAS_STANDBY);	// sleep -> normal			
			twl6040_set_bias_level(codec, level);
			printk(KERN_ERR "twl6040_shutdown reset!! end\n");
		}
	}

	return 0;
	
}

static int twl6040_hw_params(struct snd_pcm_substream *substream,
			struct snd_pcm_hw_params *params,
			struct snd_soc_dai *dai)
{
	struct snd_soc_pcm_runtime *rtd = substream->private_data;
	struct snd_soc_codec *codec = rtd->codec;
	struct twl6040_data *priv = snd_soc_codec_get_drvdata(codec);
	u8 lppllctl;
	int rate;

	/* nothing to do for high-perf pll, it supports only 48 kHz */
	if (priv->pll == TWL6040_HPPLL_ID)
		return 0;

	lppllctl = twl6040_read_reg_cache(codec, TWL6040_REG_LPPLLCTL);

	rate = params_rate(params);
	switch (rate) {
	case 11250:
	case 22500:
	case 44100:
	case 88200:
		lppllctl |= TWL6040_LPLLFIN;
		priv->sysclk = 17640000;
		break;
	case 8000:
	case 16000:
	case 32000:
	case 48000:
	case 96000:
		lppllctl &= ~TWL6040_LPLLFIN;
		priv->sysclk = 19200000;
		break;
	default:
		dev_err(codec->dev, "unsupported rate %d\n", rate);
		return -EINVAL;
	}

	twl6040_write(codec, TWL6040_REG_LPPLLCTL, lppllctl);

	return 0;
}

static int twl6040_mute(struct snd_soc_dai *codec_dai, int mute)
{
#if 0
	struct snd_soc_codec *codec = codec_dai->codec;
	int hs_gain, hfl_gain, hfr_gain;
// LGE LAB4 CH.PARK@LGE.COM 20110106 MUTE_CHECK
	struct snd_soc_card *card = codec_dai->card;
	int idx;
	int mute_all;

	hfl_gain = twl6040_read_reg_cache(codec, TWL6040_REG_HFLGAIN);
	hfr_gain = twl6040_read_reg_cache(codec, TWL6040_REG_HFRGAIN);
	hs_gain = twl6040_read_reg_cache(codec, TWL6040_REG_HSGAIN);

	if (mute) {
// LGE LAB4 CH.PARK@LGE.COM 20110106 MUTE_CHECK
		mute_all = 1;
		for( idx = 0 ; idx < TWL6040_MUTE_DATA_MAX ; idx++ )
		{
			if( s_mute_data[idx].dai == codec_dai )
			{
				s_mute_data[idx].mute = mute;
			}

			if( s_mute_data[idx].dai && s_mute_data[idx].mute == 0 )
			{
				mute_all = 0;
				break;
			}
		}

		if( mute_all )
		{
			twl6040_i2c_write(TWL6040_REG_HFLGAIN, 0x1D);
			twl6040_i2c_write(TWL6040_REG_HFRGAIN, 0x1D);
			twl6040_i2c_write(TWL6040_REG_HSGAIN, 0xFF);
		}
	} else {
// LGE LAB4 CH.PARK@LGE.COM 20110106 MUTE_CHECK
		mute_all = 0;
		for( idx = 0 ; idx < TWL6040_MUTE_DATA_MAX ; idx++ )
		{
			if( s_mute_data[idx].dai == codec_dai )
			{
				s_mute_data[idx].mute = mute;
			}

			if( s_mute_data[idx].dai && s_mute_data[idx].mute )
			{
				mute_all = 1;
				break;
			}
		}

		if( mute_all == 0 ){
			twl6040_write(codec, TWL6040_REG_HFLGAIN, hfl_gain);
			twl6040_write(codec, TWL6040_REG_HFRGAIN, hfr_gain);
			twl6040_write(codec, TWL6040_REG_HSGAIN, hs_gain);
		}
	}
#endif
	return 0;
}

static int twl6040_prepare(struct snd_pcm_substream *substream,
			struct snd_soc_dai *dai)
{
	struct snd_soc_pcm_runtime *rtd = substream->private_data;
	struct snd_soc_codec *codec = rtd->codec;
	struct twl6040_data *priv = snd_soc_codec_get_drvdata(codec);

	if (!priv->sysclk) {
		dev_err(codec->dev,
			"no mclk configured, call set_sysclk() on init\n");
		return -EINVAL;
	}

	/*
	 * capture is not supported at 17.64 MHz,
	 * it's reserved for headset low-power playback scenario
	 */
	if ((priv->sysclk == 17640000) &&
			substream->stream == SNDRV_PCM_STREAM_CAPTURE) {
		dev_err(codec->dev,
			"capture mode is not supported at %dHz\n",
			priv->sysclk);
		return -EINVAL;
	}

	if ((priv->sysclk == 17640000) && priv->non_lp) {
			dev_err(codec->dev,
				"some enabled paths aren't supported at %dHz\n",
				priv->sysclk);
			return -EPERM;
	}
	return 0;
}

static int twl6040_set_dai_sysclk(struct snd_soc_dai *codec_dai,
		int clk_id, unsigned int freq, int dir)
{
	struct snd_soc_codec *codec = codec_dai->codec;
	struct twl6040_data *priv = snd_soc_codec_get_drvdata(codec);
	u8 hppllctl, lppllctl;

	hppllctl = twl6040_read_reg_cache(codec, TWL6040_REG_HPPLLCTL);
	lppllctl = twl6040_read_reg_cache(codec, TWL6040_REG_LPPLLCTL);

	switch (clk_id) {
	case TWL6040_SYSCLK_SEL_LPPLL:
		switch (freq) {
		case 32768:
			/* headset dac and driver must be in low-power mode */
			headset_power_mode(codec, 0);

			/* clk32k input requires low-power pll */
			lppllctl |= TWL6040_LPLLENA;
			twl6040_write(codec, TWL6040_REG_LPPLLCTL, lppllctl);
			mdelay(5);
			lppllctl &= ~TWL6040_HPLLSEL;
			twl6040_write(codec, TWL6040_REG_LPPLLCTL, lppllctl);
			hppllctl &= ~TWL6040_HPLLENA;
			twl6040_write(codec, TWL6040_REG_HPPLLCTL, hppllctl);
			break;
		default:
			dev_err(codec->dev, "unknown mclk freq %d\n", freq);
			return -EINVAL;
		}

		/* lppll divider */
		switch (priv->sysclk) {
		case 17640000:
			lppllctl |= TWL6040_LPLLFIN;
			break;
		case 19200000:
			lppllctl &= ~TWL6040_LPLLFIN;
			break;
		default:
			/* sysclk not yet configured */
			lppllctl &= ~TWL6040_LPLLFIN;
			priv->sysclk = 19200000;
			break;
		}

		twl6040_write(codec, TWL6040_REG_LPPLLCTL, lppllctl);

		priv->pll = TWL6040_LPPLL_ID;
		priv->sysclk_constraints = &lp_constraints;
		break;
	case TWL6040_SYSCLK_SEL_HPPLL:
		hppllctl &= ~TWL6040_MCLK_MSK;

		switch (freq) {
		case 12000000:
			/* mclk input, pll enabled */
			hppllctl |= TWL6040_MCLK_12000KHZ |
				    TWL6040_HPLLSQRBP |
				    TWL6040_HPLLENA;
			break;
		case 19200000:
			/* mclk input, pll disabled */
			hppllctl |= TWL6040_MCLK_19200KHZ |
				    TWL6040_HPLLSQRENA |
				    TWL6040_HPLLBP;
			break;
		case 26000000:
			/* mclk input, pll enabled */
			hppllctl |= TWL6040_MCLK_26000KHZ |
				    TWL6040_HPLLSQRBP |
				    TWL6040_HPLLENA;
			break;
		case 38400000:
			/* clk slicer, pll disabled */
			hppllctl |= TWL6040_MCLK_38400KHZ |
				    TWL6040_HPLLSQRENA |
				    TWL6040_HPLLBP;
			break;
		default:
			dev_err(codec->dev, "unknown mclk freq %d\n", freq);
			return -EINVAL;
		}

		/* headset dac and driver must be in high-performance mode */
		headset_power_mode(codec, 1);

		twl6040_write(codec, TWL6040_REG_HPPLLCTL, hppllctl);
		udelay(500);
		lppllctl |= TWL6040_HPLLSEL;
		twl6040_write(codec, TWL6040_REG_LPPLLCTL, lppllctl);
		lppllctl &= ~TWL6040_LPLLENA;
		twl6040_write(codec, TWL6040_REG_LPPLLCTL, lppllctl);

		/* high-performance pll can provide only 19.2 MHz */
		priv->pll = TWL6040_HPPLL_ID;
		priv->sysclk = 19200000;
		priv->sysclk_constraints = &hp_constraints;
		break;
	default:
		dev_err(codec->dev, "unknown clk_id %d\n", clk_id);
		return -EINVAL;
	}

	return 0;
}

static struct snd_soc_dai_ops twl6040_dai_ops = {
	.startup	= twl6040_startup,
	.hw_params	= twl6040_hw_params,
	.digital_mute   = twl6040_mute,
	.prepare	= twl6040_prepare,
	.set_sysclk	= twl6040_set_dai_sysclk,
// LGE LAB4 CH.PARK@LGE.COM 20110106 MUTE_CHECK
	.shutdown	= twl6040_shutdown,
};

static struct snd_soc_dai_driver twl6040_dai[] = {
{
	.name = "twl6040-ul",
	.capture = {
		.stream_name = "Capture",
		.channels_min = 1,
		.channels_max = 2,
		.rates = TWL6040_RATES,
		.formats = TWL6040_FORMATS,
	},
	.ops = &twl6040_dai_ops,
},
{
	.name = "twl6040-dl1",
	.playback = {
		.stream_name = "Headset Playback",
		.channels_min = 1,
		.channels_max = 2,
		.rates = TWL6040_RATES,
		.formats = TWL6040_FORMATS,
	},
	.ops = &twl6040_dai_ops,
},
{
	.name = "twl6040-dl2",
	.playback = {
		.stream_name = "Handsfree Playback",
		.channels_min = 1,
		.channels_max = 2,
		.rates = TWL6040_RATES,
		.formats = TWL6040_FORMATS,
	},
	.ops = &twl6040_dai_ops,
},
{
	.name = "twl6040-vib",
	.playback = {
		.stream_name = "Vibra Playback",
		.channels_min = 2,
		.channels_max = 2,
		.rates = SNDRV_PCM_RATE_CONTINUOUS,
		.formats = TWL6040_FORMATS,
	},
	.ops = &twl6040_dai_ops,
},
};

#ifdef CONFIG_PM
static int twl6040_suspend(struct snd_soc_codec *codec, pm_message_t state)
{
	twl6040_set_bias_level(codec, SND_SOC_BIAS_OFF);

	return 0;
}

static int twl6040_resume(struct snd_soc_codec *codec)
{
	/* TODO: read HS jack insertion status */
	twl6040_set_bias_level(codec, SND_SOC_BIAS_STANDBY);
	twl6040_set_bias_level(codec, codec->dapm->suspend_bias_level);

	return 0;
}
#else
#define twl6040_suspend NULL
#define twl6040_resume NULL
#endif

void twl6040_hs_jack_detect(struct snd_soc_codec *codec,
			    struct snd_soc_jack *jack, int report)
{
	struct twl6040_data *priv = snd_soc_codec_get_drvdata(codec);
	int status, state;

	priv->hs_jack.jack = jack;
	priv->hs_jack.report = report;

/* LGE_ADD_S [ty.lee@lge.com] 20101027, merge 2.6.35 */
#if !defined(CONFIG_MACH_LGE_COSMO_REV_A)
	/* Sync status */
	status = twl6040_read_reg_volatile(codec, TWL6040_REG_STATUS);

#if defined(CONFIG_MACH_LGE_COSMOPOLITAN)
	if(status & TWL6040_PLUGCOMP)
		priv->hs_jack.state = HEADSET_NONE;		
	else
		priv->hs_jack.state = WIRED_HEADSET;
#else
	if(status & TWL6040_PLUGCOMP)
		state = report;
	else
		state = 0;
#endif

#endif// !CONFIG_MACH_LGE_COSMO_REV_A
/* LGE_ADD_E [ty.lee@lge.com] 20101027, merge 2.6.35 */


/* LGE_ADD_S [ty.lee@lge.com] 20101020, Enable Headset detection */
#if defined(CONFIG_MACH_LGE_COSMO_REV_A)
	if(gpio_get_value(priv->hsjack_gpio))
#else
	if(priv->hs_jack.state)
#endif
	{
		hs_set_bias(codec, 1);
		set_hook_enable(codec, 1);
		mdelay(200);

		if(is_without_mic()){
			set_hook_enable(codec, 0);
			hs_set_bias(codec, 0);
			state = WIRED_HEADPHONE;//wired headset without MIC
			status = SND_JACK_HEADPHONE;
		}
		else{
			/* LGE_CHANGE_S ty.lee@lge.com 20110106, enable Hook interrupt
			 * for wired_headset */
			if(priv->intmask & TWL6040_HOOKMSK){
				priv->intmask &= ~TWL6040_HOOKMSK;
				twl6040_write(codec, TWL6040_REG_INTMR, priv->intmask);
			}
			/* LGE_CHANGE_E ty.lee@lge.com */
			state = WIRED_HEADSET;//wired headset with MIC
			status = priv->hs_jack.report;//SND_JACK_HEADSET
		}
	}
	else{
		/* LGE_CHANGE_S ty.lee@lge.com 20110106, disable hook interrupt
		 * when headset removed */
		if(!(priv->intmask & TWL6040_HOOKMSK)) {
			priv->intmask |= TWL6040_HOOKMSK;
			twl6040_write(codec, TWL6040_REG_INTMR, priv->intmask);
		}
		/* LGE_CHANGE_E ty.lee@lge.com */
		set_hook_enable(codec, 0);
		hs_set_bias(codec, 0);
		state = HEADSET_NONE;
		status = 0;
	}

	switch_set_state(&priv->hs_jack.sdev, state);
	snd_soc_jack_report(jack, status, report);
/* LGE_ADD_E [ty.lee@lge.com] 20101020, Enable Headset detection */
}
EXPORT_SYMBOL_GPL(twl6040_hs_jack_detect);

/* LGE_ADD_S [ty.lee@lge.com] 20101027, headset Detection */
#if defined(CONFIG_MACH_LGE_COSMO_REV_A)
static irqreturn_t hsjack_irq_handler(int irq, void *dev_id)
{
	struct snd_soc_codec *codec = (struct snd_soc_codec *)dev_id;
	struct twl6040_data *priv = snd_soc_codec_get_drvdata(codec);

	if(gpio_get_value(priv->hsjack_gpio)){
		schedule_work(&priv->hsdet_work);
	}
	else
		schedule_work(&priv->hsdet_work);

	return IRQ_HANDLED;
}
#endif

void hs_set_bias(struct snd_soc_codec *codec, int on)
{
	u8 hsbias;

	hsbias = twl6040_read_reg_cache(codec, TWL6040_REG_AMICBCTL);
	
	if(on)
		hsbias |= TWL6040_HMICENA;
	else
		hsbias &= ~TWL6040_HMICENA;

	twl6040_write(codec, TWL6040_REG_AMICBCTL, hsbias);
}

void set_hook_enable(struct snd_soc_codec *codec, int on)
{
	u8 hkctl1;
	
	hkctl1 = twl6040_read_reg_cache(codec, TWL6040_REG_HKCTL1);
	
	if(on)
		hkctl1 |= TWL6040_HKEN;
	else
		hkctl1 &= ~TWL6040_HKEN;

	twl6040_write(codec, TWL6040_REG_HKCTL1, hkctl1);
}
/* LGE_ADD_E [ty.lee@lge.com] 20101027, headset Detection */

static int twl6040_probe(struct snd_soc_codec *codec)
{
	struct twl4030_codec_audio_data *twl_codec = codec->dev->platform_data;
	struct twl6040_data *priv;
	struct twl6040_jack_data *jack;
	int audpwron, naudint;
/* LGE_CHANGE_S ty.lee@lge.com */
	struct input_dev *ip_dev;
/* LGE_CHANGE_E ty.lee@lge.com */

/* LGE_ADD_S [ty.lee@lge.com] 20101020, Enable Headset detection */
#if defined(CONFIG_MACH_LGE_COSMO_REV_A)
	unsigned hsjack_gpio, hsjack_irq;
	int err;
#endif
/* LGE_ADD_E [ty.lee@lge.com] 20101020, Enable Headset detection */
	int ret = 0;
	u8 icrev = 0, intmr = TWL6040_ALLINT_MSK;
// LGE LAB4 CH.PARK@LGE.COM 20110106 MUTE_CHECK
	int idx;

	for( idx = 0 ; idx < TWL6040_MUTE_DATA_MAX ; idx++ )
	{
		s_mute_data[idx].dai = 0;
		s_mute_data[idx].mute = 0;
	}
	
	priv = kzalloc(sizeof(struct twl6040_data), GFP_KERNEL);
	if (priv == NULL)
		return -ENOMEM;
	snd_soc_codec_set_drvdata(codec, priv);

	priv->codec = codec;
	priv->dl_active = 0;
	priv->ul_active = 0;

	twl6040_i2c_read(TWL6040_REG_ASICREV, &icrev);

	if (twl_codec && (icrev > 0))
		audpwron = twl_codec->audpwron_gpio;
	else
		audpwron = -EINVAL;

/* LGE_ADD_S [ty.lee@lge.com] 20101020, Enable Headset detection */
#if defined(CONFIG_MACH_LGE_COSMO_REV_A)
	if (twl_codec){
		naudint = twl_codec->naudint_irq;
		hsjack_gpio = twl_codec->hsjack_gpio;
		hsjack_irq = twl_codec->hsjack_irq;
	}
	else {
		naudint = 0;
		hsjack_gpio = 0;
		hsjack_irq = 0;
	}
#else
	if (twl_codec)
		naudint = twl_codec->naudint_irq;
	else
		naudint = 0;
#endif
/* LGE_ADD_E [ty.lee@lge.com] 20101020, Enable Headset detection */

	priv->audpwron = audpwron;
	priv->naudint = naudint;
	
/* LGE_ADD_S [ty.lee@lge.com] 20101020, Enable Headset detection */
#if defined(CONFIG_MACH_LGE_COSMO_REV_A)
	priv->hsjack_gpio = hsjack_gpio;
	priv->hsjack_irq = hsjack_irq;
#endif
/* LGE_ADD_E [ty.lee@lge.com] 20101020, Enable Headset detection */
	init_completion(&priv->ready);

	/* Disable safe mode in SYS_NIRQ PAD */
//	omap_writew(0x0118, 0x4A1001A0);

/* LGE_ADD_S [ty.lee@lge.com] 20101027, headset detection, merge 2.6.35 */
	INIT_DELAYED_WORK(&priv->hsdet_dwork, twl6040_hs_jack_detect_dwork);
	INIT_DELAYED_WORK(&priv->hook_work, twl6040_hs_hook_detect_work);
/* LGE_ADD_E [ty.lee@lge.com] 20101027, headset detection, merge 2.6.35 */

#ifndef CONFIG_MACH_LGE_COSMOPOLITAN
	INIT_WORK(&priv->audint_work, twl6040_audint_work);
#endif

	/* LGE_CHANGE_S ty.lee@lge.com, 20110117, for long key event */
	ip_dev	= input_allocate_device();
	if(!ip_dev){
		dev_err(codec->dev, "failed to allocation hook input device");
		goto switch_err;
	}
	__set_bit(EV_KEY, ip_dev->evbit);
	__set_bit(EV_SYN, ip_dev->evbit);
	__set_bit(KEY_HOOK, ip_dev->keybit);
	ip_dev->name = "headset_hook";
	ip_dev->phys = "headset_hook/input0";
	priv->hs_jack.headset_input = ip_dev;
	input_register_device(priv->hs_jack.headset_input);
	/* LGE_CHANGE_E ty.lee@lge.com, 20110117 */

	/* switch-class based headset detection */
	jack = &priv->hs_jack;
	jack->sdev.name = "h2w";
	ret = switch_dev_register(&jack->sdev);
	if (ret) {
		dev_err(codec->dev, "error registering switch device %d\n", ret);
		goto switch_err;
	}
/* LGE_ADD_S [ty.lee@lge.com] 20101020, Enable Headset detection */
#if defined(CONFIG_MACH_LGE_COSMO_REV_A)
	/* GPIO request and direction set */
	if(gpio_is_valid(hsjack_gpio)) {
		err = gpio_request(hsjack_gpio, "ear_sense");
		if (err) {
			printk(KERN_ERR "%s: failed to request GPIO_%d\n",
				   __func__, hsjack_gpio);
			goto err_hs_gpio_request;
		}
		err = gpio_direction_input(hsjack_gpio);
		if (err) {
			printk(KERN_ERR "%s: failed to set direction GPIO_%d\n",
				   __func__, hsjack_gpio);
			goto err_hs_gpio_direction;
		}
	}

	/* IRQ request */
	err = request_irq(hsjack_irq,
					  hsjack_irq_handler,
					  IRQF_TRIGGER_FALLING | IRQF_TRIGGER_RISING,
					  "headset_detect",
					  codec);
	if (err) {
		printk(KERN_ERR "%s: failed to request irq (%d)\n",
			   __func__, hsjack_irq);
		goto err_hs_request_irq;
	}
#endif
/* LGE_ADD_E [ty.lee@lge.com] 20101020, Enable Headset detection */

	if (gpio_is_valid(audpwron)) {
		ret = gpio_request(audpwron, "audpwron");
		if (ret)
			goto gpio1_err;

		ret = gpio_direction_output(audpwron, 0);
		if (ret)
			goto gpio2_err;

		priv->codec_powered = 0;

/* LGE_CHANGE_S ty.lee@lge.com, Hook key enable 20101117 */
		/* enable only codec ready interrupt */
		intmr &= ~(TWL6040_READYMSK | TWL6040_PLUGMSK );
		/* LGE_ADD_S ty.lee@lge.com 20110106, avoid hook detection for wired_headphone */
		priv->intmask = intmr; 
		/* LGE_ADD_E ty.lee@lge.com 20110106 */
/* LGE_CHANGE_E ty.lee@lge.com */

		/* reset interrupt status to allow correct power up sequence */
		twl6040_read_reg_volatile(codec, TWL6040_REG_INTID);
	}
	twl6040_write(codec, TWL6040_REG_INTMR, intmr);

	if (naudint) {
		/* audio interrupt */
		ret = request_threaded_irq(naudint, NULL,
				twl6040_naudint_handler,
				IRQF_TRIGGER_LOW | IRQF_ONESHOT,
				"twl6040_codec", codec);
		if (ret)
			goto gpio2_err;
	}

	/* init vio registers */
	twl6040_init_vio_regs(codec);

	/* power on device */
	ret = twl6040_set_bias_level(codec, SND_SOC_BIAS_STANDBY);
	if (ret)
		goto bias_err;

	snd_soc_add_controls(codec, twl6040_snd_controls,
				ARRAY_SIZE(twl6040_snd_controls));
	twl6040_add_widgets(codec);

	wake_lock_init(&priv->wake_lock, WAKE_LOCK_SUSPEND, "twl6040");

	/* TODO: read HS jack insertion status */

	return 0;

bias_err:
	if (naudint)
		free_irq(naudint, codec);
#if defined(CONFIG_MACH_LGE_COSMO_REV_A)
err_hs_request_irq:
err_hs_gpio_direction:
	if (gpio_is_valid(hsjack_gpio))
		gpio_free(hsjack_gpio);
err_hs_gpio_request:
#endif
gpio2_err:
	if (gpio_is_valid(audpwron))
		gpio_free(audpwron);
gpio1_err:
	switch_dev_unregister(&jack->sdev);
switch_err:
	kfree(priv);
	return ret;
}

static int twl6040_remove(struct snd_soc_codec *codec)
{
	struct twl6040_data *priv = snd_soc_codec_get_drvdata(codec);
	struct twl6040_jack_data *jack = &priv->hs_jack;
	int audpwron = priv->audpwron;
	int naudint = priv->naudint;

/* LGE_ADD_S [ty.lee@lge.com] 20101027, merge 2.6.35 */
#if defined(CONFIG_MACH_LGE_COSMO_REV_A)
	int headset_gpio = priv->hsjack_gpio;
#endif 
/* LGE_ADD_E [ty.lee@lge.com] 20101027, merge 2.6.35 */

	wake_lock_destroy(&priv->wake_lock);

	twl6040_set_bias_level(codec, SND_SOC_BIAS_OFF);


	if (gpio_is_valid(audpwron))
		gpio_free(audpwron);

/* LGE_ADD_S [ty.lee@lge.com] 20101027, merge 2.6.35 */
#if defined(CONFIG_MACH_LGE_COSMO_REV_A)
	if (gpio_is_valid(headset_gpio))
		gpio_free(headset_gpio);

#endif
	cancel_delayed_work_sync(&priv->hsdet_dwork);
	cancel_delayed_work_sync(&priv->hook_work);
/* LGE_ADD_E [ty.lee@lge.com] 20101027, merge 2.6.35 */

	if (naudint)
		free_irq(naudint, codec);
#ifndef CONFIG_MACH_LGE_COSMOPOLITAN
	cancel_work_sync(&priv->audint_work);
#endif
	switch_dev_unregister(&jack->sdev);
	kfree(priv);

	return 0;
}

static struct snd_soc_codec_driver soc_codec_dev_twl6040 = {
	.probe = twl6040_probe,
	.remove = twl6040_remove,
	.suspend = twl6040_suspend,
	.resume = twl6040_resume,
	.read = twl6040_read_reg_cache,
	.write = twl6040_write,
	.set_bias_level = twl6040_set_bias_level,
	.reg_cache_size = ARRAY_SIZE(twl6040_reg),
	.reg_word_size = sizeof(u8),
	.reg_cache_default = twl6040_reg,
};

static int __devinit twl6040_codec_probe(struct platform_device *pdev)
{
	return snd_soc_register_codec(&pdev->dev,
			&soc_codec_dev_twl6040, twl6040_dai, ARRAY_SIZE(twl6040_dai));
}

static int __devexit twl6040_codec_remove(struct platform_device *pdev)
{
	snd_soc_unregister_codec(&pdev->dev);
	return 0;
}

static struct platform_driver twl6040_codec_driver = {
	.driver = {
		.name = "twl6040-codec",
		.owner = THIS_MODULE,
	},
	.probe = twl6040_codec_probe,
	.remove = __devexit_p(twl6040_codec_remove),
};

static int __init twl6040_codec_init(void)
{
	return platform_driver_register(&twl6040_codec_driver);
}
module_init(twl6040_codec_init);

static void __exit twl6040_codec_exit(void)
{
	platform_driver_unregister(&twl6040_codec_driver);
}
module_exit(twl6040_codec_exit);

MODULE_DESCRIPTION("ASoC TWL6040 codec driver");
MODULE_AUTHOR("Misael Lopez Cruz");
MODULE_LICENSE("GPL");
