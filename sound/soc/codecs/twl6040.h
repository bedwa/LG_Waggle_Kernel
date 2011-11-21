/*
 * ALSA SoC TWL6040 codec driver
 *
 * Author:	Misael Lopez Cruz <x0052729@ti.com>
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

#ifndef __TWL6040_H__
#define __TWL6040_H__

struct twl6040_setup_data {
	void (*codec_enable)(int enable);
	void *jack;
};

void twl6040_hs_jack_detect(struct snd_soc_codec *codec,
			    struct snd_soc_jack *jack, int report);

/* LGE_ADD_S [ty.lee@lge.com] 20101027, headset detection, hook detection */
#define HEADSET_NONE	0
#define WIRED_HEADSET	1//with MIC
#define WIRED_HEADPHONE	2//without MIC
int is_without_mic(void);
void hs_set_bias(struct snd_soc_codec *codec, int on);
void set_hook_enable(struct snd_soc_codec *codec, int on);
/* LGE_ADD_E [ty.lee@lge.com] 20101027, headset detection, hook detection */
#endif /* End of __TWL6040_H__ */
