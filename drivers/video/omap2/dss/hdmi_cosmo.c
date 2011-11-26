/*
 * linux/drivers/video/omap2/dss/hdmi.c
 *
 * Copyright (C) 2009 Texas Instruments
 * Author: Yong Zhi
 *
 * HDMI settings from TI's DSS driver
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms of the GNU General Public License version 2 as published by
 * the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
 * more details.
 *
 * You should have received a copy of the GNU General Public License along with
 * this program.  If not, see <http://www.gnu.org/licenses/>.
 * History:
 * Mythripk <mythripk@ti.com>  Apr 2010 Modified for EDID reading and adding OMAP
 *                                      related timing
 *				May 2010 Added support of Hot Plug Detect
 *				July 2010 Redesigned HDMI EDID for Auto-detect of timing
 *				August 2010 Char device user space control for HDMI
 * Munish <munish@ti.com>	Sep 2010 Added support for Mandatory S3D formats
 */

#define DSS_SUBSYS_NAME "HDMI"

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/clk.h>
#include <linux/err.h>
#include <linux/io.h>
#include <linux/interrupt.h>
#include <linux/mutex.h>
#include <linux/completion.h>
#include <linux/delay.h>
#include <linux/string.h>
#include <linux/platform_device.h>
#include <plat/display.h>
#include <plat/cpu.h>
#include <plat/hdmi_lib.h>
#include <plat/gpio.h>
#include <linux/slab.h>
#include <linux/cdev.h>
#include <linux/fs.h>

#include "dss.h"
#include <plat/edid.h>

#include <linux/wakelock.h>
#include <linux/kthread.h>
#include <linux/sched.h>
#include <linux/wait.h>
#undef DSSDBG
#define DSSDBG printk

#include <plat/omap-pm.h>

#define IPU_PM_MM_MPU_LAT_CONSTRAINT         10

/* Latency cstrs */
#ifdef CONFIG_OMAP_PM
static struct pm_qos_request_list *pm_hdmi_qos_handle;
#endif

enum hdmi_internal_status {
	HDMI_STATUS_UNDEFINED 				= -1,
	HDMI_STATUS_OFF						= 0,
	HDMI_STATUS_PLUG_DETECTION_ENABLED	= 1,
	HDMI_STATUS_PLUG_REQUESTED			= 2,
	HDMI_STATUS_PLUG_ESTABLISHED		= 3,
};

struct hdmi_s3d_info {
	bool subsamp;
	int  structure;
	int  subsamp_pos;
};

#define HDMI_VIDEO_PRODUCER_MAX	10

//irq handler callback. runs in irq context
typedef void (*hdmi_irq_handler)(u32 wp_irq, void* data);

struct hdmi_irq_handler_item
{
	hdmi_irq_handler	handler;
	void				*data;
	struct list_head	queue;
};

enum E_HDMI_VIDEO_COMMIT_STATUS {
	HDMI_VIDEO_COMMITED = 0,
	HDMI_VIDEO_WILL_BE_ADDED,
	HDMI_VIDEO_WILL_BE_REMOVED
};

struct hdmi {
	struct kobject kobj;
	void __iomem *base_phy;
	void __iomem *base_pll;

	//Work queue
	struct workqueue_struct	*work_queue;

	//protected by spin lock
	struct {
		bool					enabled;
		bool					requested;
		int						no;
		struct list_head		handlers;
	} irq;

	struct mutex status_lock;
	struct {
		int					user_hpd_request;	// 0: disable
												// 1: enable
		int					hpd_status;
		enum hdmi_internal_status status;		//status
		int  in_reset;							//in reset
		struct wake_lock	hdmi_wake_lock;
	} status;

	struct mutex connection_lock;
	struct {
		int code;
		int mode;
		int deep_color;
		int lr_fr;
		int force_set;
		bool s3d_switch_support;
		bool s3d_enabled;
		struct hdmi_config cfg;
		u8	custom_set;
		struct hdmi_s3d_info s3d_info;
		u8 edid[HDMI_EDID_MAX_LENGTH];
		u8 edid_set;
		struct omap_video_timings edid_timings;
		unsigned int drm_lock;
	} connection;

	struct mutex play_status_lock;
	struct {
		struct {
			void*	id;
			int		layer_count;
			enum E_HDMI_VIDEO_COMMIT_STATUS	commit_status;
		} video_producers[HDMI_VIDEO_PRODUCER_MAX];
		int	video_producer_cnt;
		struct delayed_work	delayed_start_work;
		wait_queue_head_t	video_producer_wq;
		bool		video_play_enabled;
		bool		dirty;
	} play_status;

	struct mutex 		plug_status_lock;
	struct {
		wait_queue_head_t		change_wait;
		struct task_struct*		handler;
		bool					changed;
	} plug_status;

	struct mutex clock_lock;
	struct {
		int		count;
	} clock;
	struct omap_display_platform_data *pdata;
	struct platform_device *pdev;
} hdmi;

static struct omap_dss_device *get_hdmi_device(void);
static void hdmi_get_timings(struct omap_dss_device *dssdev,
			struct omap_video_timings *timings);
static void hdmi_set_timings(struct omap_dss_device *dssdev,
			struct omap_video_timings *timings);
static void hdmi_set_custom_edid_timing_code(struct omap_dss_device *dssdev,
							int code , int mode);
static void hdmi_get_edid(struct omap_dss_device *dssdev);
static int hdmi_check_timings(struct omap_dss_device *dssdev,
			struct omap_video_timings *timings);
static int hdmi_read_edid_with_connection_lock(struct omap_video_timings *);
static int get_edid_timing_data_with_connection_lock(struct HDMI_EDID *edid);
static irqreturn_t hdmi_wp_irq_main_handler(int irq, void *arg);
static int hdmi_enable_hpd(struct omap_dss_device *dssdev);
static int hdmi_disable_hpd(struct omap_dss_device *dssdev);
static int hdmi_open(struct inode *inode, struct file *filp);
static int hdmi_release(struct inode *inode, struct file *filp);
static bool hdmi_get_s3d_enabled(struct omap_dss_device *dssdev);
static int hdmi_enable_s3d(struct omap_dss_device *dssdev, bool enable);
static int hdmi_set_s3d_disp_type(struct omap_dss_device *dssdev,
		struct s3d_disp_info *info);
static int hdmi_reset(struct omap_dss_device *dssdev,
					enum omap_dss_reset_phase phase);
static struct hdmi_cm hdmi_get_code(struct omap_video_timings *timing);
static enum hdmi_internal_status hdmi_get_internal_status(void);
static int hdmi_status_move_to(enum hdmi_internal_status new_status);
static void hdmi_check_status_and_transit(void);
static void hdmi_irq_disable(void);
static void hdmi_irq_enable(void);
static int hdmi_suspend_with_msg(struct device *dev, pm_message_t state);
static int hdmi_suspend(struct device *dev);
static int hdmi_resume(struct device *dev);
static void hdmi_power_off_phy(void);

static void hdmi_video_start_real_with_play_status_lock(int delay_m);
static void hdmi_video_stop_real_with_play_status_lock(void);

static void hdmi_unplug_wait_without_lock(void);
static void hdmi_video_stop_force_internal(void);

static int hdmi_wp_irq_add_handler(hdmi_irq_handler handler, void *data);
static int hdmi_wp_irq_remove_handler(hdmi_irq_handler handelr, void *data);

static void hdmi_wp_connection_irq_handler(u32 wp_irq, void *data);

static int hdmi_connection_handler(void *data);

static int hdmi_get_hpd_status_with_status_lock(void);
static int hdmi_status_move_to_with_status_lock(enum hdmi_internal_status new_status);

static void hdmi_raise_plug_status_change(void);
static void hdmi_raise_fake_plug_status_change(void);

static int hdmi_send_uevent(struct omap_dss_device *dssdev, enum kobject_action action);

static int hdmi_video_check_n_action_with_plays_staus_lock(int start_delay_m);
static int hdmi_stop_plug_with_status_lock(struct omap_dss_device *dssdev);
static int hdmi_start_plug_with_status_lock(struct omap_dss_device *dssdev);

static void hdmi_start_hdmi_video_delay_func(struct work_struct *data);

/* Structures for chardevice move this to panel */
static int hdmi_major;
static struct cdev hdmi_cdev;
static dev_t hdmi_dev_id;

static const struct file_operations hdmi_fops = {
	.owner = THIS_MODULE,
	.open = hdmi_open,
	.release = hdmi_release,
};



static DEFINE_SPINLOCK(irqstatus_lock);

#define HDMI_IN_RESET		0x1000

#define HDMI_PLLCTRL		0x58006200
#define HDMI_PHY		0x58006300

static u8 header[8] = {0x0, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0x0};


enum hdmi_s3d_frame_structure {
	HDMI_S3D_FRAME_PACKING          = 0,
	HDMI_S3D_FIELD_ALTERNATIVE      = 1,
	HDMI_S3D_LINE_ALTERNATIVE       = 2,
	HDMI_S3D_SIDE_BY_SIDE_FULL      = 3,
	HDMI_S3D_L_DEPTH                = 4,
	HDMI_S3D_L_DEPTH_GP_GP_DEPTH    = 5,
	HDMI_S3D_SIDE_BY_SIDE_HALF      = 8
	/* LGE_CHANGE_S [wonki.choi@lge.com] HDMI S3D Switching 2011-2-09*/
	,
	HDMI_S3D_TOP_AND_BOTTOM			= 6,
		//HDMI 1.4a 3D video format extension.
		//Vendor Specific InfoFrame packet PB5 Significant Nibble 3D_Structure
		//		0110 : Top and Bottom
		//		1000 : Side-by-Side (Half)
	/* LGE_CHANGE_E [wonki.choi@lge.com] 2011-2-09 */
};

/* Subsampling types used for Sterioscopic 3D over HDMI. Below HOR
stands for Horizontal, QUI for Quinxcunx Subsampling, O for odd fields,
E for Even fields, L for left view and R for Right view*/
enum hdmi_s3d_subsampling_type {
	HDMI_S3D_HOR_OL_OR = 0,/*horizontal subsampling with odd fields
		from left view and even fields from the right view*/
	HDMI_S3D_HOR_OL_ER = 1,
	HDMI_S3D_HOR_EL_OR = 2,
	HDMI_S3D_HOR_EL_ER = 3,
	HDMI_S3D_QUI_OL_OR = 4,
	HDMI_S3D_QUI_OL_ER = 5,
	HDMI_S3D_QUI_EL_OR = 6,
	HDMI_S3D_QUI_EL_ER = 7
};


/* PLL */
#define PLLCTRL_PLL_CONTROL				0x0ul
#define PLLCTRL_PLL_STATUS				0x4ul
#define PLLCTRL_PLL_GO					0x8ul
#define PLLCTRL_CFG1					0xCul
#define PLLCTRL_CFG2					0x10ul
#define PLLCTRL_CFG3					0x14ul
#define PLLCTRL_CFG4					0x20ul

/* HDMI PHY */
#define HDMI_TXPHY_TX_CTRL				0x0ul
#define HDMI_TXPHY_DIGITAL_CTRL			0x4ul
#define HDMI_TXPHY_POWER_CTRL			0x8ul
#define HDMI_TXPHY_PAD_CFG_CTRL			0xCul

struct hdmi_hvsync_pol {
	int vsync_pol;
	int hsync_pol;
};

/* All supported timing values that OMAP4 supports */
static const struct omap_video_timings all_timings_direct[] = {
	{640, 480, 25200, 96, 16, 48, 2, 10, 33},
	{1280, 720, 74250, 40, 440, 220, 5, 5, 20},
	{1280, 720, 74250, 40, 110, 220, 5, 5, 20},
	{720, 480, 27000, 62, 16, 60, 6, 9, 30},
	{2880, 576, 108000, 256, 48, 272, 5, 5, 39},
	{1440, 240, 27000, 124, 38, 114, 3, 4, 15},
	{1440, 288, 27000, 126, 24, 138, 3, 2, 19},
	{1920, 540, 74250, 44, 528, 148, 5, 2, 15},
	{1920, 540, 74250, 44, 88, 148, 5, 2, 15},
/* LGE_CHANGE_S [novashock.lee@lge.com] 2011-01-03 HDMI. restore original value for playing LCD monitor*/
	{1920, 1080, 148500, 44, 88, 148, 5, 4, 36},
/* LGE_CHANGE_E [novashock.lee@lge.com] 2011-01-03 */
	{720, 576, 27000, 64, 12, 68, 5, 5, 39},
	{1440, 576, 54000, 128, 24, 136, 5, 5, 39},
	{1920, 1080, 148500, 44, 528, 148, 5, 4, 36},
	{2880, 480, 108000, 248, 64, 240, 6, 9, 30},
	{1920, 1080, 74250, 44, 638, 148, 5, 4, 36},
	/* Vesa frome here */
 	{640, 480, 25175, 96, 16, 48, 2, 10, 33},
	{800, 600, 40000, 128, 40, 88, 4 , 1, 23},
	{848, 480, 33750, 112, 16, 112, 8 , 6, 23},
	{1280, 768, 79500, 128, 64, 192, 7 , 3, 20},
	{1280, 800, 83500, 128, 72, 200, 6 , 3, 22},
	{1360, 768, 85500, 112, 64, 256, 6 , 3, 18},
	{1280, 960, 108000, 112, 96, 312, 3 , 1, 36},
	{1280, 1024, 108000, 112, 48, 248, 3 , 1, 38},
	{1024, 768, 65000, 136, 24, 160, 6, 3, 29},
	{1400, 1050, 121750, 144, 88, 232, 4, 3, 32},
	{1440, 900, 106500, 152, 80, 232, 6, 3, 25},
	{1680, 1050, 146250, 176 , 104, 280, 6, 3, 30},
	{1366, 768, 85500, 143, 70, 213, 3, 3, 24},
	{1920, 1080, 148500, 44, 88, 148, 5, 4, 36},
	{1280, 768, 68250, 32, 48, 80, 7, 3, 12},
	{1400, 1050, 101000, 32, 48, 80, 4, 3, 23},
	{1680, 1050, 119000, 32, 48, 80, 6, 3, 21},
	{1280, 800, 79500, 32, 48, 80, 6, 3, 14},
	{1280, 720, 74250, 40, 110, 220, 5, 5, 20},
	{1920, 1200, 154000, 32, 48, 80, 6, 3, 26},
	/* supported 3d timings UNDEROVER full frame */
	{1280, 1470, 148350, 40, 110, 220, 5, 5, 20},
	{1280, 1470, 148500, 40, 110, 220, 5, 5, 20},
	{1280, 1470, 148500, 40, 440, 220, 5, 5, 20}
};

/* Array which maps the timing values with corresponding CEA / VESA code */
static int code_index[ARRAY_SIZE(all_timings_direct)] = {
	1,  19,  4,  2, 37,  6, 21, 20,  5, 16, 17,
	29, 31, 35, 32,
	/* <--15 CEA 22--> vesa*/
	4, 9, 0xE, 0x17, 0x1C, 0x27, 0x20, 0x23, 0x10, 0x2A,
	0X2F, 0x3A, 0X51, 0X52, 0x16, 0x29, 0x39, 0x1B, 0x55, 0X2C,
	4, 4, 19,
};

/* Mapping the Timing values with the corresponding Vsync and Hsync polarity */
static const
struct hdmi_hvsync_pol hvpol_mapping[ARRAY_SIZE(all_timings_direct)] = {
	{0, 0}, {1, 1}, {1, 1}, {0, 0},
	{0, 0}, {0, 0}, {0, 0}, {1, 1},
	{1, 1}, {1, 1}, {0, 0}, {0, 0},
	{1, 1}, {0, 0}, {1, 1}, /* VESA */
	{0, 0}, {1, 1}, {1, 1}, {1, 0},
	{1, 0}, {1, 1}, {1, 1}, {1, 1},
	{0, 0}, {1, 0}, {1, 0}, {1, 0},
	{1, 1}, {1, 1}, {0, 1}, {0, 1},
	{0, 1}, {0, 1}, {1, 1}, {1, 0},
	{1, 1}, {1, 1}, {1, 1}
};

/* Map CEA code to the corresponding timing values (10 entries/line) */
static int code_cea[39] = {
	-1,  0,  3,  3,  2,  8,  5,  5, -1, -1,
	-1, -1, -1, -1, -1, -1,  9, 10, 10,  1,
	7,   6,  6, -1, -1, -1, -1, -1, -1, 11,
	11, 12, 14, -1, -1, 13, 13,  4,  4
};

/* Map CEA code to the corresponding 3D timing values */
static int s3d_code_cea[39] = {
	-1, -1, -1, -1, 36, -1, -1, -1, -1, -1,
	-1, -1, -1, -1, -1, -1, -1, -1, -1, 37,
	-1, -1, -1, -1, -1, -1, -1, -1, -1, -1,
	-1, -1, -1, -1, -1, -1, -1, -1, -1
};

/* Map VESA code to the corresponding timing values */
static int code_vesa[86] = {
	-1, -1, -1, -1, 15, -1, -1, -1, -1, 16,
	-1, -1, -1, -1, 17, -1, 23, -1, -1, -1,
	-1, -1, 29, 18, -1, -1, -1, 32, 19, -1,
	-1, -1, 21, -1, -1, 22, -1, -1, -1, 20,
	-1, 30, 24, -1, 34, -1, -1, 25, -1, -1,
	-1, -1, -1, -1, -1, -1, -1, 31, 26, -1,
	-1, -1, -1, -1, -1, -1, -1, -1, -1, -1,
	-1, -1, -1, -1, -1, -1, -1, -1, -1, -1,
	-1, 27, 28, -1, -1, 33
};



struct hdmi_cm {
	int code;
	int mode;
};

inline enum omap_dss_display_state get_dss_state(struct omap_dss_device *dssdev)
{
	return dssdev->state;
}

static void update_cfg(struct hdmi_config *cfg,
					struct omap_video_timings *timings)
{
	cfg->ppl = timings->x_res;
	cfg->lpp = timings->y_res;
	cfg->hbp = timings->hbp;
	cfg->hfp = timings->hfp;
	cfg->hsw = timings->hsw;
	cfg->vbp = timings->vbp;
	cfg->vfp = timings->vfp;
	cfg->vsw = timings->vsw;
	cfg->pixel_clock = timings->pixel_clock;
}

static void update_cfg_pol(struct hdmi_config *cfg, int  code)
{
	cfg->v_pol = hvpol_mapping[code].vsync_pol;
	cfg->h_pol = hvpol_mapping[code].hsync_pol;
}

static inline void hdmi_write_reg(u32 base, u16 idx, u32 val)
{
	void __iomem *b;

	switch (base) {
	case HDMI_PHY:
	  b = hdmi.base_phy;
	  break;
	case HDMI_PLLCTRL:
	  b = hdmi.base_pll;
	  break;
	default:
	  BUG();
	}
	__raw_writel(val, b + idx);
	/* DBG("write = 0x%x idx =0x%x\r\n", val, idx); */
}

static inline u32 hdmi_read_reg(u32 base, u16 idx)
{
	void __iomem *b;
	u32 l;

	switch (base) {
	case HDMI_PHY:
	 b = hdmi.base_phy;
	 break;
	case HDMI_PLLCTRL:
	 b = hdmi.base_pll;
	 break;
	default:
	 BUG();
	}
	l = __raw_readl(b + idx);

	/* DBG("addr = 0x%p rd = 0x%x idx = 0x%x\r\n", (b+idx), l, idx); */
	return l;
}

#define FLD_GET(val, start, end) (((val) & FLD_MASK(start, end)) >> (end))
#define FLD_MOD(orig, val, start, end) \
	(((orig) & ~FLD_MASK(start, end)) | FLD_VAL(val, start, end))

#define REG_FLD_MOD(b, i, v, s, e) \
	hdmi_write_reg(b, i, FLD_MOD(hdmi_read_reg(b, i), v, s, e))



static ssize_t hdmi_edid_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	mutex_lock(&hdmi.connection_lock);
	memcpy(buf, hdmi.connection.edid, HDMI_EDID_MAX_LENGTH);
	mutex_unlock(&hdmi.connection_lock);
	return HDMI_EDID_MAX_LENGTH;
}

static ssize_t hdmi_edid_store(struct device *dev,
		struct device_attribute *attr,
		const char *buf, size_t size)
{
	return 0;
}

static ssize_t hdmi_yuv_supported(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	bool enabled;
	mutex_lock(&hdmi.connection_lock);
	enabled = hdmi_tv_yuv_supported(hdmi.connection.edid);
	mutex_unlock(&hdmi.connection_lock);
	return snprintf(buf, PAGE_SIZE, "%s\n", enabled ? "true" : "false");
}

static ssize_t hdmi_yuv_set(struct device *dev,
		struct device_attribute *attr,
		const char *buf, size_t size)
{
	unsigned long yuv;
	int r = strict_strtoul(buf, 0, &yuv);
	if (r == 0)
		hdmi_configure_csc(yuv ? RGB_TO_YUV : RGB);
	return r ? : size;
}


static bool hdmi_out_enabled = 0;
static ssize_t hdmi_out_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
//	struct omap_dss_device *dssdev = to_dss_device(dev);
	
	return snprintf(buf, PAGE_SIZE, "%d\n", hdmi_out_enabled);
}

static ssize_t hdmi_out_set(struct device *dev,
		struct device_attribute *attr,
		const char *buf, size_t size)
{
//	struct omap_dss_device *dssdev = to_dss_device(dev);
	
	hdmi_out_enabled = simple_strtoul(buf, NULL, 10);
	
	return size;
}
/* hdmi source sys interface */
static char hdmi_source[10] = "<none>";
static ssize_t hdmi_source_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	return snprintf(buf, PAGE_SIZE, "%s\n",hdmi_source);
}

static ssize_t hdmi_source_set(struct device *dev,
                struct device_attribute *attr,
                const char *buf, size_t size)
{
	
	int len = size;
	
	if (buf[size-1] == '\n')
		--len;

	if(len >= 0 ){
		if (strncpy((char*)hdmi_source, buf, len))
				     hdmi_source[len] = '\0';
	}

	
	return size;
		
}

/* hdmi key sys interface */
static char hdmi_key[256] = "<none>";
static ssize_t hdmi_key_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	return snprintf(buf, PAGE_SIZE, "%s\n",hdmi_key);
}

static ssize_t hdmi_key_set(struct device *dev,
                struct device_attribute *attr,
                const char *buf, size_t size)
{
	int len = size;

	if (buf[size-1] == '\n')
		--len;

	if(len >= 0 ){
		if (strncpy((char*)hdmi_key, buf, len))
				hdmi_key[len] = '\0';
	}
	
	return size;
}

static ssize_t hdmi_deepcolor_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	int deep_color;
	mutex_lock(&hdmi.connection_lock);
	deep_color = hdmi.connection.deep_color;
	mutex_unlock(&hdmi.connection_lock);
	return snprintf(buf, PAGE_SIZE, "%d\n", deep_color);
}

static ssize_t hdmi_deepcolor_store(struct device *dev,
		struct device_attribute *attr,
		const char *buf, size_t size)
{
	unsigned long deep_color;
	int r = strict_strtoul(buf, 0, &deep_color);
	if (r || deep_color > 2)
		return -EINVAL;
	mutex_lock(&hdmi.connection_lock);
	hdmi.connection.deep_color = deep_color;
	mutex_unlock(&hdmi.connection_lock);
	return size;
}

/*
 * This function is used to configure Limited range/full range
 * with RGB format , with YUV format Full range is not supported
 * Please refer to section 6.6 Video quantization ranges in HDMI 1.3a
 * specification for more details.
 * Now conversion to Full range or limited range can either be done at
 * display controller or HDMI IP ,This function allows to select either
 * Please note : To convert to full range it is better to convert the video
 * in the dispc to full range as there will be no loss of data , if a
 * limited range data is sent ot HDMI and converted to Full range in HDMI
 * the data quality would not be good.
 */
static void hdmi_configure_lr_fr_with_connection_lock(void)
{
	int ret = 0;
	int code;
	int mode;
	int lr_fr;
	int force_set;
	code = hdmi.connection.code;
	mode = hdmi.connection.mode;
	lr_fr = hdmi.connection.lr_fr;
	force_set = hdmi.connection.force_set;
	if ( mode == 0 || (mode == 1 && code == 1)) {
		ret = hdmi_configure_lrfr(HDMI_FULL_RANGE, 0);
		if (!ret)
			dispc_setup_color_fr_lr(1);
		return;
	}
	if (lr_fr) {
		ret = hdmi_configure_lrfr(HDMI_FULL_RANGE, force_set);
		if (!ret && !force_set)
			dispc_setup_color_fr_lr(1);
	} else {
		ret = hdmi_configure_lrfr(HDMI_LIMITED_RANGE, force_set);
		if (!ret && !force_set)
			dispc_setup_color_fr_lr(0);
	}
}

static ssize_t hdmi_lr_fr_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	int lr_fr;
	int force_set;
	mutex_lock(&hdmi.connection_lock);
	lr_fr = hdmi.connection.lr_fr;
	force_set = hdmi.connection.force_set;
	mutex_unlock(&hdmi.connection_lock);
	return snprintf(buf, PAGE_SIZE, "%d:%d\n", lr_fr, force_set);
}

static ssize_t hdmi_lr_fr_store(struct device *dev,
		struct device_attribute *attr,
		const char *buf, size_t size)
{
	int lr_fr, force_set = 0;
	if (!*buf || !strchr("yY1nN0", *buf))
		return -EINVAL;
	lr_fr = !!strchr("yY1", *buf++);
	if (*buf && *buf != '\n') {
		if (!strchr(" \t,:", *buf++) ||
		    !*buf || !strchr("yY1nN0", *buf))
			return -EINVAL;
		force_set = !!strchr("yY1", *buf++);
	}
	if (*buf && strcmp(buf, "\n"))
		return -EINVAL;
	mutex_lock(&hdmi.connection_lock);
	hdmi.connection.lr_fr = lr_fr;
	hdmi.connection.force_set = force_set;
	hdmi_configure_lr_fr_with_connection_lock();
	mutex_unlock(&hdmi.connection_lock);
	return size;
}

static ssize_t hdmi_code_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct omap_dss_device *dssdev = to_dss_device(dev);
	struct hdmi_cm cm;

	cm = hdmi_get_code(&dssdev->panel.timings);

	return snprintf(buf, PAGE_SIZE, "%s:%u\n",
			cm.mode ? "CEA" : "VESA", cm.code);
}


static ssize_t hdmi_drm_lock_show(struct device *dev,
							struct device_attribute *attr, char *buf)
{
	unsigned int drm_lock;
	mutex_lock(&hdmi.connection_lock);
	drm_lock = hdmi.connection.drm_lock;
	mutex_unlock(&hdmi.connection_lock);
	return snprintf(buf, PAGE_SIZE, "%d\n",drm_lock);
}

static ssize_t hdmi_drm_lock_set(struct device *dev,
                struct device_attribute *attr,
                const char *buf, size_t size)
{
	unsigned int drm_lock;
	drm_lock = simple_strtoul(buf, NULL, 10);

	mutex_lock(&hdmi.connection_lock);
	if ( hdmi.connection.drm_lock!=drm_lock )
	{
		DSSDBG("HDMI DRM Lock Changed %d->%d\n", hdmi.connection.drm_lock, drm_lock);
		hdmi.connection.drm_lock = drm_lock;
		if ( drm_lock > 0 )
		{
			struct omap_dss_device *dssdev = get_hdmi_device();
			hdmi_send_uevent(dssdev, KOBJ_OFFLINE);
		}
	}
	mutex_unlock(&hdmi.connection_lock);
	return size;
}

static DEVICE_ATTR(edid, S_IRUGO, hdmi_edid_show, hdmi_edid_store);
static DEVICE_ATTR(yuv, S_IRUGO | S_IWUSR, hdmi_yuv_supported, hdmi_yuv_set);
static DEVICE_ATTR(hdmi_out, S_IRUGO | S_IWUSR, hdmi_out_show, hdmi_out_set);
static DEVICE_ATTR(hdmi_source, S_IRUGO | S_IWUSR, hdmi_source_show, hdmi_source_set);
static DEVICE_ATTR(hdmi_key, S_IRUGO | S_IWUSR, hdmi_key_show, hdmi_key_set);
static DEVICE_ATTR(deepcolor, S_IRUGO | S_IWUSR, hdmi_deepcolor_show,
							hdmi_deepcolor_store);
static DEVICE_ATTR(lr_fr, S_IRUGO | S_IWUSR, hdmi_lr_fr_show, hdmi_lr_fr_store);
static DEVICE_ATTR(hdmi_drm_lock, S_IRUGO | S_IWUSR, hdmi_drm_lock_show, hdmi_drm_lock_set);
static DEVICE_ATTR(code, S_IRUGO, hdmi_code_show, NULL);

static int hdmi_send_uevent(struct omap_dss_device *dssdev, enum kobject_action action)
{
	int ret = 0;

	DSSINFO("HDMI uEvent %d\n", action);
	ret = kobject_uevent(&dssdev->dev.kobj, action);
	if (ret)
		DSSWARN("error sending HDMI uEvent(%d)\n", action);
	return ret;
}

static int hdmi_open(struct inode *inode, struct file *filp)
{
	return 0;
}

static int hdmi_release(struct inode *inode, struct file *filp)
{
	return 0;
}

static struct omap_dss_device *get_hdmi_device(void)
{
	int match(struct omap_dss_device *dssdev, void *arg)
	{
		return sysfs_streq(dssdev->name , "hdmi");
	}

	return omap_dss_find_device(NULL, match);
}

/*
 * refclk = (sys_clk/(highfreq+1))/(n+1)
 * so refclk = 38.4/2/(n+1) = 19.2/(n+1)
 * choose n = 15, makes refclk = 1.2
 *
 * m = tclk/cpf*refclk = tclk/2*1.2
 *
 *	for clkin = 38.2/2 = 192
 *	    phy = 2520
 *
 *	m = 2520*16/2* 192 = 105;
 *
 *	for clkin = 38.4
 *	    phy = 2520
 *
 */

struct hdmi_pll_info {
	u16 regn;
	u16 regm;
	u32 regmf;
	u16 regm2;
	u16 regsd;
	u16 dcofreq;
};

static inline void print_omap_video_timings(struct omap_video_timings *timings)
{
	extern unsigned int dss_debug;
	if (dss_debug) {
		printk(KERN_INFO "Timing Info:\n");
		printk(KERN_INFO "  pixel_clk = %d\n", timings->pixel_clock);
		printk(KERN_INFO "  x_res     = %d\n", timings->x_res);
		printk(KERN_INFO "  y_res     = %d\n", timings->y_res);
		printk(KERN_INFO "  hfp       = %d\n", timings->hfp);
		printk(KERN_INFO "  hsw       = %d\n", timings->hsw);
		printk(KERN_INFO "  hbp       = %d\n", timings->hbp);
		printk(KERN_INFO "  vfp       = %d\n", timings->vfp);
		printk(KERN_INFO "  vsw       = %d\n", timings->vsw);
		printk(KERN_INFO "  vbp       = %d\n", timings->vbp);
	}
}

static void compute_pll(int clkin, int phy,
	int n, struct hdmi_pll_info *pi)
{
	int refclk;
	u32 temp, mf;

	refclk = clkin / (n + 1);

	temp = phy * 100/(refclk);

	pi->regn = n;
	pi->regm = temp/100;
	pi->regm2 = 1;

	mf = (phy - pi->regm * refclk) * 262144;
	pi->regmf = mf/(refclk);

	if (phy > 1000 * 100) {
		pi->dcofreq = 1;
	} else {
		pi->dcofreq = 0;
	}

	pi->regsd = ((pi->regm * clkin / 10) / ((n + 1) * 250) + 5) / 10;

	DSSDBG("M = %d Mf = %d\n", pi->regm, pi->regmf);
	DSSDBG("range = %d sd = %d\n", pi->dcofreq, pi->regsd);
}

static int hdmi_pll_init(int refsel, int dcofreq, struct hdmi_pll_info *fmt,
									u16 sd)
{
	u32 r;
	unsigned t = 500000;
	u32 pll = HDMI_PLLCTRL;

	/* PLL start always use manual mode */
	REG_FLD_MOD(pll, PLLCTRL_PLL_CONTROL, 0x0, 0, 0);

	r = hdmi_read_reg(pll, PLLCTRL_CFG1);
	r = FLD_MOD(r, fmt->regm, 20, 9); /* CFG1__PLL_REGM */
	r = FLD_MOD(r, fmt->regn, 8, 1);  /* CFG1__PLL_REGN */

	hdmi_write_reg(pll, PLLCTRL_CFG1, r);

	r = hdmi_read_reg(pll, PLLCTRL_CFG2);

	r = FLD_MOD(r, 0x0, 12, 12); /* PLL_HIGHFREQ divide by 2 */
	r = FLD_MOD(r, 0x1, 13, 13); /* PLL_REFEN */
	r = FLD_MOD(r, 0x0, 14, 14); /* PHY_CLKINEN de-assert during locking */

	if (dcofreq) {
		/* divider programming for 1080p */
		REG_FLD_MOD(pll, PLLCTRL_CFG3, sd, 17, 10);
		r = FLD_MOD(r, 0x4, 3, 1); /* 1000MHz and 2000MHz */
	} else {
		r = FLD_MOD(r, 0x2, 3, 1); /* 500MHz and 1000MHz */
	}

	hdmi_write_reg(pll, PLLCTRL_CFG2, r);

	r = hdmi_read_reg(pll, PLLCTRL_CFG4);
	r = FLD_MOD(r, fmt->regm2, 24, 18);
	r = FLD_MOD(r, fmt->regmf, 17, 0);

	hdmi_write_reg(pll, PLLCTRL_CFG4, r);

	/* go now */
	REG_FLD_MOD(pll, PLLCTRL_PLL_GO, 0x1ul, 0, 0);

	/* wait for bit change */
	while (FLD_GET(hdmi_read_reg(pll, PLLCTRL_PLL_GO), 0, 0))
		;

	/* Wait till the lock bit is set */
	/* read PLL status */
	while (0 == FLD_GET(hdmi_read_reg(pll, PLLCTRL_PLL_STATUS), 1, 1)) {
		udelay(1);
		if (!--t) {
			printk(KERN_WARNING "HDMI: cannot lock PLL\n");
			DSSDBG("CFG1 0x%x\n", hdmi_read_reg(pll, PLLCTRL_CFG1));
			DSSDBG("CFG2 0x%x\n", hdmi_read_reg(pll, PLLCTRL_CFG2));
			DSSDBG("CFG4 0x%x\n", hdmi_read_reg(pll, PLLCTRL_CFG4));
			return -EIO;
		}
	}

	DSSDBG("PLL locked!\n");

	return 0;
}

static int hdmi_pll_reset(void)
{
	int t = 0;

	/* SYSREEST  controled by power FSM*/
	REG_FLD_MOD(HDMI_PLLCTRL, PLLCTRL_PLL_CONTROL, 0x0, 3, 3);

	/* READ 0x0 reset is in progress */
	while (!FLD_GET(hdmi_read_reg(HDMI_PLLCTRL,
			PLLCTRL_PLL_STATUS), 0, 0)) {
		udelay(1);
		if (t++ > 1000) {
			ERR("Failed to sysrest PLL\n");
			return -ENODEV;
		}
	}

	return 0;
}

static int hdmi_pll_program(struct hdmi_pll_info *fmt)
{
	u32 r;
	int refsel;

	HDMI_PllPwr_t PllPwrWaitParam;

	/* wait for wrapper rest */
	HDMI_W1_SetWaitSoftReset();

	/* power off PLL */
	PllPwrWaitParam = HDMI_PLLPWRCMD_ALLOFF;
	r = HDMI_W1_SetWaitPllPwrState(HDMI_WP, PllPwrWaitParam);
	if (r)
		return r;

	/* power on PLL */
	PllPwrWaitParam = HDMI_PLLPWRCMD_BOTHON_ALLCLKS;
	r = HDMI_W1_SetWaitPllPwrState(HDMI_WP, PllPwrWaitParam);
	if (r)
		return r;

	hdmi_pll_reset();

	refsel = 0x3; /* select SYSCLK reference */

	r = hdmi_pll_init(refsel, fmt->dcofreq, fmt, fmt->regsd);

	return r;
}

/* double check the order */
static int hdmi_phy_init(u32 w1,
		u32 phy, int tmds)
{
	int r;

	//not good
	hdmi_notify_pwrchange(HDMI_EVENT_POWERPHYON);

	/* wait till PHY_PWR_STATUS=LDOON */
	/* HDMI_PHYPWRCMD_LDOON = 1 */
	r = HDMI_W1_SetWaitPhyPwrState(w1, 1);
	if (r)
		return r;

	/* wait till PHY_PWR_STATUS=TXON */
	r = HDMI_W1_SetWaitPhyPwrState(w1, 2);
	if (r)
		return r;

	/* read address 0 in order to get the SCPreset done completed */
	/* Dummy access performed to solve resetdone issue */
	hdmi_read_reg(phy, HDMI_TXPHY_TX_CTRL);

	/* write to phy address 0 to configure the clock */
	/* use HFBITCLK write HDMI_TXPHY_TX_CONTROL__FREQOUT field */
	REG_FLD_MOD(phy, HDMI_TXPHY_TX_CTRL, tmds, 31, 30);

	/* write to phy address 1 to start HDMI line (TXVALID and TMDSCLKEN) */
	hdmi_write_reg(phy, HDMI_TXPHY_DIGITAL_CTRL, 0xF0000000);

	/* setup max LDO voltage */
	REG_FLD_MOD(phy, HDMI_TXPHY_POWER_CTRL, 0xB, 3, 0);
	/*  write to phy address 3 to change the polarity control  */
	REG_FLD_MOD(phy, HDMI_TXPHY_PAD_CFG_CTRL, 0x1, 27, 27);

	return 0;
}

static int hdmi_phy_off(u32 name)
{
	int r = 0;

	//not good
	hdmi_notify_pwrchange(HDMI_EVENT_POWERPHYOFF);

	/* wait till PHY_PWR_STATUS=OFF */
	/* HDMI_PHYPWRCMD_OFF = 0 */
	r = HDMI_W1_SetWaitPhyPwrState(name, 0);
	if (r)
		return r;

	return 0;
}

static int get_s3d_timings_index_with_connection_lock(void)
{
	int code;
	code = s3d_code_cea[hdmi.connection.code];

	if (code == -1) {
		hdmi.connection.s3d_enabled = false;
		code = 9;
		hdmi.connection.code = 16;
		hdmi.connection.mode = 1;
	}
	return code;
}
/* driver */
static int get_timings_index_with_connection_lock(void)
{
	int code;

	if (hdmi.connection.mode == 0)
		code = code_vesa[hdmi.connection.code];
	else
		code = code_cea[hdmi.connection.code];

	if (code == -1 || all_timings_direct[code].x_res >= 2048) {
		code = 9;
		hdmi.connection.code = 16;
		hdmi.connection.mode = 1;
	}
	return code;
}

static int hdmi_panel_probe(struct omap_dss_device *dssdev)
{
	int code;
	printk(KERN_DEBUG "ENTER hdmi_panel_probe()\n");

	dssdev->panel.config = OMAP_DSS_LCD_TFT |
			OMAP_DSS_LCD_IVS | OMAP_DSS_LCD_IHS;
	hdmi.connection.deep_color = 0;
	hdmi.connection.lr_fr = HDMI_LIMITED_RANGE;
	mutex_lock(&hdmi.connection_lock);
	code = get_timings_index_with_connection_lock();
	mutex_unlock(&hdmi.connection_lock);

	dssdev->panel.timings = all_timings_direct[code];
	printk(KERN_INFO "hdmi_panel_probe x_res= %d y_res = %d", \
		dssdev->panel.timings.x_res, dssdev->panel.timings.y_res);

	mdelay(50);
	return 0;
}

//////////////////////////////////////////////////////////////////
// Panel will be treated as HDMI Video Play status

static void hdmi_panel_remove(struct omap_dss_device *dssdev)
{
}

/**
 * return HDMI Power Status
 * Very very ugly. But for framework integration, I can't find another way
 */
static bool hdmi_panel_is_enabled(struct omap_dss_device *dssdev)
{
	return ( hdmi_get_internal_status()==HDMI_STATUS_PLUG_ESTABLISHED );
}

/**
 * Resume Video play, if there is an active video producer
 */
static int hdmi_panel_enable(struct omap_dss_device *dssdev)
{
	DSSDBG("HDMI Panel enable. Start video if there is an active video producer\n");
	mutex_lock(&hdmi.play_status_lock);
	hdmi_video_check_n_action_with_plays_staus_lock(0);
	mutex_unlock(&hdmi.play_status_lock);
	return 0;
}

/**
 * Stop Video Play.
 * If there is an active Video producer, Video will stop
 */
static void hdmi_panel_disable(struct omap_dss_device *dssdev)
{
	//consider as HDMI problem happened
	DSSINFO("Ohter module disable HDMI. May be because of error. Only Stop Video\n");
	mutex_lock(&hdmi.play_status_lock);
	hdmi_video_stop_real_with_play_status_lock();
	mutex_unlock(&hdmi.play_status_lock);
}

#ifdef CONFIG_HAS_EARLYSUSPEND

static int hdmi_early_suspend(struct omap_dss_device *dssdev)
{
	enum hdmi_internal_status status;
	int ret;
	DSSDBG("HDMI earlay suspend\n");
	mutex_lock(&hdmi.status_lock);
	status = hdmi.status.status;
	switch ( status )
	{
	case HDMI_STATUS_OFF :
		ret = 0;
		break;
	case HDMI_STATUS_PLUG_ESTABLISHED :
		DSSINFO("HDMI cable is plugged. not suspend\n");
//		mutex_lock(&hdmi.play_status_lock);
//		hdmi_video_stop_real_with_play_status_lock();
//		mutex_unlock(&hdmi.play_status_lock);
		ret = -EPERM;
		break;
	default:
		if ( !hdmi_status_move_to_with_status_lock(HDMI_STATUS_OFF) )
		{
			//for resume work
			DSSDBG("HDMI Suspend\n");

			////////////////////////////////////////////////////////
			dssdev->state = OMAP_DSS_DISPLAY_SUSPENDED;
			dssdev->activate_after_resume = true;
			////////////////////////////////////////////////////////

			ret = 0;
		}
		else
		{
			DSSERR("HDMI PLUG-Detection to Off failed.\n");
			ret = -EPERM;
		}
		break;
	}
	mutex_unlock(&hdmi.status_lock);
	return ret;
}

static int hdmi_late_resume(struct omap_dss_device *dssdev)
{
	DSSDBG("HDMI late resume\n");
	hdmi_check_status_and_transit();
	//when HDMI min enabled, force check HDMI plug status by fake raising plug change
	hdmi_raise_fake_plug_status_change();
	return 0;
}

#endif


static int hdmi_suspend(struct device *dev)
{
	enum hdmi_internal_status status;
	status = hdmi_get_internal_status();
	DSSDBG("hdmi suspend\n");
	if ( status==HDMI_STATUS_OFF )
	{
		//nothing to do
		return 0;
	}
	else if ( status==HDMI_STATUS_PLUG_DETECTION_ENABLED )
	{
		return hdmi_status_move_to(HDMI_STATUS_OFF);
	}
	else
	{
		DSSERR("Unstable HDMI status %d. Anywayt goto off\n", status);
		return hdmi_status_move_to(HDMI_STATUS_OFF);
	}
}

static int hdmi_suspend_with_msg(struct device *dev, pm_message_t state)
{
	return hdmi_suspend(dev);
}


static int hdmi_resume(struct device *dev)
{
	enum hdmi_internal_status status;
	DSSDBG("hdmi resume\n");
	status = hdmi_get_internal_status();
	if ( status!=HDMI_STATUS_OFF )
	{
		DSSERR("When HDMI resume, current status is not off. force to off\n");
		hdmi_status_move_to(HDMI_STATUS_OFF);
	}
	hdmi_check_status_and_transit();
	return 0;
}

static void hdmi_enable_clocks(int enable)
{
	if (enable )
	{
		mutex_lock(&hdmi.clock_lock);
		if ( hdmi.clock.count == 0 )
		{
			dss_clk_enable(DSS_CLK_ICK | DSS_CLK_FCK1 | DSS_CLK_54M |
					DSS_CLK_96M);
		}
		hdmi.clock.count++;
		mutex_unlock(&hdmi.clock_lock);
	}
	else
	{
		mutex_lock(&hdmi.clock_lock);
		if ( hdmi.clock.count==0 )
		{
			mutex_unlock(&hdmi.clock_lock);
			return;
		}
		hdmi.clock.count--;
		if ( hdmi.clock.count==0 )
		{
			dss_clk_disable(DSS_CLK_ICK | DSS_CLK_FCK1 | DSS_CLK_54M |
					DSS_CLK_96M);
		}
		mutex_unlock(&hdmi.clock_lock);
	}
}
// LGE START 2011-02-08 Enhancement of HPD Flow Control --novashock.lee@lge.com //
bool hdmi_state(void)
{
	return (hdmi_get_internal_status()==HDMI_STATUS_PLUG_ESTABLISHED);
}

static struct omap_dss_driver hdmi_driver = {
	.probe		= hdmi_panel_probe,
	.remove		= hdmi_panel_remove,

	.disable	= hdmi_panel_disable,
	.smart_enable	= hdmi_panel_enable,
	.smart_is_enabled	= hdmi_panel_is_enabled,
#ifdef CONFIG_HAS_EARLYSUSPEND
	.suspend	= hdmi_early_suspend,
	.resume		= hdmi_late_resume,
#else
	.suspend	= hdmi_suspend,
	.resume		= hdmi_resume,
#endif
	.get_timings	= hdmi_get_timings,
	.set_timings	= hdmi_set_timings,
	.check_timings	= hdmi_check_timings,
	.get_edid	= hdmi_get_edid,
	.set_custom_edid_timing_code	= hdmi_set_custom_edid_timing_code,
	.hpd_enable	=	hdmi_enable_hpd,
	.hpd_disable	= hdmi_disable_hpd,
	.reset		= hdmi_reset,

	.enable_s3d = hdmi_enable_s3d,
	.get_s3d_enabled = hdmi_get_s3d_enabled,
	.set_s3d_disp_type = hdmi_set_s3d_disp_type,

	.driver			= {
		.name   = "hdmi_panel",
		.owner  = THIS_MODULE,
#ifdef CONFIG_HAS_EARLYSUSPEND
		.suspend = hdmi_suspend_with_msg,
		.resume  = hdmi_resume,
#endif
	},
};
/* driver end */

int hdmi_init(struct platform_device *pdev)
{
	int r = 0;
	struct resource *hdmi_mem;
	printk(KERN_INFO "Enter hdmi_init()\n");

	memset(&hdmi, 0, sizeof(hdmi));


	hdmi.pdata = pdev->dev.platform_data;
	hdmi.pdev = pdev;

	hdmi_mem = platform_get_resource(pdev, IORESOURCE_MEM, 1);
	//LGE_CHANGE [taekeun1.kim@lge.com] 2011-01-12, P920 : For WBT
	if(!hdmi_mem) {
                ERR("can't alloc hdmi_mem\n");
                return -ENOMEM;
	}

	hdmi.base_pll = ioremap((hdmi_mem->start + 0x200), resource_size(hdmi_mem));
	if (!hdmi.base_pll) {
		ERR("can't ioremap pll\n");
		return -ENOMEM;
	}
	hdmi.base_phy = ioremap((hdmi_mem->start + 0x300), 64);

	if (!hdmi.base_phy) {
		ERR("can't ioremap phy\n");
		return -ENOMEM;
	}

	/* Get the major number for this module */
	r = alloc_chrdev_region(&hdmi_dev_id, 0, 1, "hdmi_panel");
	if (r) {
		printk(KERN_WARNING "HDMI: Cound not register character device\n");
		return -ENOMEM;
	}

	hdmi_major = MAJOR(hdmi_dev_id);

	/* initialize character device */
	cdev_init(&hdmi_cdev, &hdmi_fops);

	hdmi_cdev.owner = THIS_MODULE;
	hdmi_cdev.ops = &hdmi_fops;

	/* add char driver */
	r = cdev_add(&hdmi_cdev, hdmi_dev_id, 1);
	if (r) {
		printk(KERN_WARNING "HDMI: Could not add hdmi char driver\n");
		unregister_chrdev_region(hdmi_dev_id, 1);
		return -ENOMEM;
	}

	//work queue creation
	hdmi.work_queue = create_singlethread_workqueue("HDMI WQ");

	//irq init
	hdmi.irq.no = platform_get_irq(pdev, 0);
	hdmi.irq.requested = false;
	hdmi.irq.enabled = false;
	INIT_LIST_HEAD(&hdmi.irq.handlers);

	//status init
	mutex_init(&hdmi.status_lock);
	hdmi.status.status		= HDMI_STATUS_UNDEFINED;
	hdmi.status.in_reset	= false;
	hdmi.status.hpd_status	= HDMI_HPD_LOW;
	wake_lock_init(&hdmi.status.hdmi_wake_lock, WAKE_LOCK_SUSPEND, "hdmi");

	//connection init
	mutex_init(&hdmi.connection_lock);
	hdmi.connection.s3d_enabled	= false;
	hdmi.connection.s3d_switch_support	= false;
	hdmi.connection.s3d_info.structure = HDMI_S3D_FRAME_PACKING;
	hdmi.connection.s3d_info.subsamp = false;
	hdmi.connection.s3d_info.subsamp_pos = 0;
	hdmi.connection.edid_set = 0;
	memset(hdmi.connection.edid, 0, sizeof(hdmi.connection.edid));
	memset(&hdmi.connection.edid_timings, 0, sizeof(hdmi.connection.edid_timings));
	hdmi.connection.drm_lock = 0;

	//play status init
	mutex_init(&hdmi.play_status_lock);
	memset(hdmi.play_status.video_producers, 0, sizeof(hdmi.play_status.video_producers));
	hdmi.play_status.video_producer_cnt = 0;
	hdmi.play_status.dirty = false;
	init_waitqueue_head(&hdmi.play_status.video_producer_wq);
	hdmi.play_status.video_play_enabled = false;
	INIT_DELAYED_WORK(&hdmi.play_status.delayed_start_work, hdmi_start_hdmi_video_delay_func);

	//plug init
	mutex_init(&hdmi.plug_status_lock);
	init_waitqueue_head(&hdmi.plug_status.change_wait);
	hdmi.plug_status.changed = false;
	hdmi.plug_status.handler = kthread_create(hdmi_connection_handler, NULL, "hdmi_plug");
	if ( hdmi.plug_status.handler==NULL )
		return -ENOMEM;
	wake_up_process( hdmi.plug_status.handler );

	mutex_init(&hdmi.clock_lock);
	hdmi.clock.count	= 0;

	//HDMI LIB init
	hdmi_lib_init();

	return omap_dss_register_driver(&hdmi_driver);
}

void hdmi_exit(void)
{
	//irq clean
	hdmi_irq_disable();
	free_irq(OMAP44XX_IRQ_DSS_HDMI, NULL);
	//free handlers
	if ( !list_empty(&hdmi.irq.handlers) )
	{
		struct list_head *pos, *n;
		list_for_each_safe(pos, n, &hdmi.irq.handlers)
		{
			struct hdmi_irq_handler_item *item;
			item = list_entry(pos, struct hdmi_irq_handler_item, queue);
			list_del(pos);
			kfree(item);
		}
		INIT_LIST_HEAD(&hdmi.irq.handlers);
	}

	//plug clean
	if ( hdmi.plug_status.handler!=NULL )
	{
		kthread_stop( hdmi.plug_status.handler);
		hdmi.plug_status.handler = NULL;
	}
	mutex_destroy(&hdmi.plug_status_lock);

	//play status clean
	mutex_destroy(&hdmi.play_status_lock);

	//connection clean
	mutex_destroy(&hdmi.connection_lock);

	//status clean up
	//@todo power down
	wake_lock_destroy(&hdmi.status.hdmi_wake_lock);
	mutex_destroy(&hdmi.status_lock);

	destroy_workqueue(hdmi.work_queue);

	hdmi_lib_exit();
	iounmap(hdmi.base_pll);
	iounmap(hdmi.base_phy);
}


static int hdmi_min_enable(struct omap_dss_device *dssdev)
{
	int r;
	DSSDBG("hdmi_min_enable");

	r = hdmi_phy_init(HDMI_WP, HDMI_PHY, 0);
	if (r)
	{
		DSSERR("Failed to start PHY\n");
		return r;
	}

	return 0;
}

//connection HDMI Device
static int hdmi_connect_device(struct omap_dss_device *dssdev)
{
	int r = 0;
	int code = 0;
	int dirty;
	struct omap_video_timings *p;
	struct hdmi_pll_info pll_data;
	struct deep_color *vsdb_format = NULL;
	int clkin, n, phy = 0, max_tmds = 0, temp = 0, tmds_freq;

	mutex_lock(&hdmi.connection_lock);
	code = get_timings_index_with_connection_lock();
	dssdev->panel.timings = all_timings_direct[code];

	p = &dssdev->panel.timings;


	if (!hdmi.connection.custom_set) {
		code = get_timings_index_with_connection_lock();

  		HDMI_W1_SetWaitSoftReset(); // 2011-03-19 novashock.lee READ EDID Fail patch. //
		DSSDBG("No edid set thus will be calling hdmi_read_edid");
		r = hdmi_read_edid_with_connection_lock(p);
		if (r) {
			r = -EIO;
			goto err;
		}

		vsdb_format = kzalloc(sizeof(*vsdb_format), GFP_KERNEL);
		hdmi_deep_color_support_info(hdmi.connection.edid, vsdb_format);
		DSSINFO("deep_color_bit30=%d bit36=%d max_tmds_freq=%d\n",
			vsdb_format->bit_30, vsdb_format->bit_36,
			vsdb_format->max_tmds_freq);
		max_tmds = vsdb_format->max_tmds_freq * 500;

		dirty = get_timings_index_with_connection_lock() != code;
	} else {
		dirty = true;
	}

	update_cfg(&hdmi.connection.cfg, p);

	if (hdmi.connection.s3d_enabled && (hdmi.connection.s3d_info.structure == HDMI_S3D_FRAME_PACKING))
		code = get_s3d_timings_index_with_connection_lock();
	else
		code = get_timings_index_with_connection_lock();
	update_cfg_pol(&hdmi.connection.cfg, code);

	dssdev->panel.timings = all_timings_direct[code];

	DSSINFO("%s:%d res=%dx%d ", hdmi.connection.mode ? "CEA" : "VESA", hdmi.connection.code,
		dssdev->panel.timings.x_res, dssdev->panel.timings.y_res);

	clkin = 3840; /* 38.4 mHz */
	n = 15; /* this is a constant for our math */

	switch (hdmi.connection.deep_color) {
	case 1:
		temp = (p->pixel_clock * 125) / 100 ;
		if (!hdmi.connection.custom_set) {
			if (vsdb_format->bit_30) {
				if (max_tmds != 0 && max_tmds >= temp)
					phy = temp;
			} else {
				printk(KERN_ERR "TV does not support Deep color");
				goto err;
			}
		} else {
			phy = temp;
		}
		hdmi.connection.cfg.deep_color = 1;
		break;
	case 2:
		if (p->pixel_clock == 148500) {
			printk(KERN_ERR"36 bit deep color not supported");
			goto err;
		}

		temp = (p->pixel_clock * 150) / 100;
		if (!hdmi.connection.custom_set) {
			if (vsdb_format->bit_36) {
				if (max_tmds != 0 && max_tmds >= temp)
					phy = temp;
			} else {
				printk(KERN_ERR "TV does not support Deep color");
				goto err;
			}
		} else {
			phy = temp;
		}
		hdmi.connection.cfg.deep_color = 2;
		break;
	case 0:
	default:
		phy = p->pixel_clock;
		hdmi.connection.cfg.deep_color = 0;
		break;
	}

	compute_pll(clkin, phy, n, &pll_data);

//	HDMI_W1_StopVideoFrame(HDMI_WP);
//	dispc_enable_digit_out(0);

	if (dirty)
		omap_dss_notify(dssdev, OMAP_DSS_SIZE_CHANGE);

	/* config the PLL and PHY first */
	r = hdmi_pll_program(&pll_data);
	if (r) {
		DSSERR("Failed to lock PLL\n");
		r = -EIO;
		goto err;
	}

	/* TMDS freq_out in the PHY should be set based on the TMDS clock */
	if (phy <= 50000)
		tmds_freq = 0x0;
	else if ((phy > 50000) && (phy <= 100000))
		tmds_freq = 0x1;
	else
		tmds_freq = 0x2;

	r = hdmi_phy_init(HDMI_WP, HDMI_PHY, tmds_freq);
	if (r) {
		DSSERR("Failed to start PHY\n");
		r = -EIO;
		goto err;
	}

	if (hdmi.connection.s3d_enabled) {
		hdmi.connection.cfg.vsi_enabled = true;
		hdmi.connection.cfg.s3d_structure = hdmi.connection.s3d_info.structure;
		hdmi.connection.cfg.subsamp_pos = hdmi.connection.s3d_info.subsamp_pos;
	} else {
		hdmi.connection.cfg.vsi_enabled = false;
	}

	hdmi.connection.cfg.hdmi_dvi = hdmi_has_ieee_id((u8*)hdmi.connection.edid) && hdmi.connection.mode;
	hdmi.connection.cfg.video_format = hdmi.connection.code;
	hdmi.connection.cfg.supports_ai = hdmi_ai_supported(hdmi.connection.edid);

	DSSINFO("%s:%d res=%dx%d ", hdmi.connection.cfg.hdmi_dvi ? "CEA" : "VESA",
		hdmi.connection.code, dssdev->panel.timings.x_res,
		dssdev->panel.timings.y_res);
	if ((hdmi.connection.mode)) {
		switch (hdmi.connection.code) {
		case 20:
		case 5:
		case 6:
		case 21:
			hdmi.connection.cfg.interlace = 1;
			break;
		default:
			hdmi.connection.cfg.interlace = 0;
			break;
		}
	}

	hdmi_configure_lr_fr_with_connection_lock();
	hdmi_lib_enable(&hdmi.connection.cfg);


	/* these settings are independent of overlays */
	dss_switch_tv_hdmi(1);

	/* bypass TV gamma table*/
	dispc_enable_gamma_table(0);

	/* allow idle mode */
	dispc_set_idle_mode();

#ifndef CONFIG_OMAP4_ES1
	/*The default reset value for DISPC.DIVISOR1 LCD is 4
	* in ES2.0 and the clock will run at 1/4th the speed
	* resulting in the sync_lost_digit */
	dispc_set_tv_divisor();
#endif

	/* tv size */
	dispc_set_digit_size(dssdev->panel.timings.x_res,
			dssdev->panel.timings.y_res);

	DSSINFO("HDMI Resolution (%d,%d)\n", dssdev->panel.timings.x_res,
			dssdev->panel.timings.y_res);
	//Producer will start video
//	HDMI_W1_StartVideoFrame(HDMI_WP);
//	dispc_enable_digit_out(1);

	kfree(vsdb_format);
	mutex_unlock(&hdmi.connection_lock);

	return 0;
err:
	mutex_unlock(&hdmi.connection_lock);
	kfree(vsdb_format);
	return r;
}


static void hdmi_irq_disable()
{
	unsigned long flags;

	spin_lock_irqsave(&irqstatus_lock, flags);
	if ( hdmi.irq.enabled )
	{
		disable_irq(hdmi.irq.no);
		hdmi.irq.enabled = false;
	}
	spin_unlock_irqrestore(&irqstatus_lock, flags);
}

static void hdmi_irq_enable()
{
	unsigned long flags;

	spin_lock_irqsave(&irqstatus_lock, flags);
	if ( !hdmi.irq.enabled )
	{
		if ( !hdmi.irq.requested )
		{
			if ( request_irq(hdmi.irq.no, hdmi_wp_irq_main_handler, 0, "OMAP HDMI", (void *)0) )
			{
				DSSERR("HDMI IRQ request failed\n");
				spin_unlock_irqrestore(&irqstatus_lock, flags);
				return;
			}
			hdmi.irq.requested = true;
		}
		else
			enable_irq(hdmi.irq.no);
		hdmi.irq.enabled = true;
	}
	spin_unlock_irqrestore(&irqstatus_lock, flags);
}

static void hdmi_status_to_off_with_status_lock(struct omap_dss_device *dssdev)
{
	//not good
	mutex_unlock(&hdmi.status_lock);
	hdmi_notify_pwrchange(HDMI_EVENT_POWEROFF);
	mutex_lock(&hdmi.status_lock);
	//disable clock
	mutex_lock(&hdmi.clock_lock);
	if ( hdmi.clock.count > 0 )
	{
		DSSERR("HDMI clock count is %d. anyway force turn off\n", hdmi.clock.count);
		hdmi.clock.count = 0;
		dss_clk_disable(DSS_CLK_ICK | DSS_CLK_FCK1 | DSS_CLK_54M |
				DSS_CLK_96M);
	}
	mutex_unlock(&hdmi.clock_lock);

	//unlock wake lock
	while( wake_lock_active(&hdmi.status.hdmi_wake_lock) )
		wake_unlock(&hdmi.status.hdmi_wake_lock);

#ifdef CONFIG_OMAP_PM
	{
		int retval =0;
		pr_debug("test\n");
		retval = omap_pm_set_min_bus_tput(&dssdev->dev,	OCP_INITIATOR_AGENT, -1);		
		if (retval) {
			pr_err("%s %d Error setting MPU cstr\n", __func__, __LINE__);
			return; //return PM_UNSUPPORTED;
		}
		retval = omap_pm_set_max_mpu_wakeup_lat(&pm_hdmi_qos_handle, -1);
		if (retval) {
			pr_err("%s %d Error setting MPU cstr\n", __func__, __LINE__);
			return; //return PM_UNSUPPORTED;
		}
	}
#endif

	hdmi_irq_disable();					//irq disable
	if ( dssdev->platform_disable )		//HDMI interface chip off
		dssdev->platform_disable(dssdev);
	//Current DSS implementation doesn't provide holding DSS clock.
	//So set state Active to prevent DSS clock disable
	dssdev->state = OMAP_DSS_DISPLAY_ACTIVE;
	if ( dss_get_mainclk_state() )	//When DSS is enabled
	{
		hdmi_power_off_phy();				// HDMI phy off
		dssdev->state = OMAP_DSS_DISPLAY_DISABLED;	//if not set clock will not be turned off.
		dss_mainclk_state_disable(true);	// if needed DSS main-clock off
	}
	//////////////////////////////////////////////////////
	dssdev->state = OMAP_DSS_DISPLAY_DISABLED;
	//////////////////////////////////////////////////////
}

/**
 * State transition
 */
static int hdmi_status_move_to_with_status_lock(enum hdmi_internal_status new_status)
{
	int ret = 0;
	enum hdmi_internal_status status;
	struct omap_dss_device *dssdev = get_hdmi_device();

	if ( dssdev==NULL )
	{
		DSSERR("No HDMI device is set\n");
		return -EINVAL;
	}
	status = hdmi.status.status;

	if ( status==new_status )
		return 0;	//same status
	DSSDBG("HDMI Status Transition(%d->%d) try \n", status, new_status);

	if ( status < new_status )
	{
		//Connect & turn on

		switch ( status )
		{
		case HDMI_STATUS_UNDEFINED:
			//to OFF
			hdmi_status_to_off_with_status_lock(dssdev);
			if ( new_status==HDMI_STATUS_OFF )
				break;
		case HDMI_STATUS_OFF:
			//to PLUG DETECTION ENABLE
			if ( !dss_get_mainclk_state() )
			{
				DSSERR("DSS Main Clock disable. can't enable hpd now. For resume enable, set as suspend state\n");
				dssdev->state = OMAP_DSS_DISPLAY_SUSPENDED;
				dssdev->activate_after_resume = true;
				hdmi.status.status = HDMI_STATUS_OFF;
				return -EIO;
			}

			////////////////////////////////////////////////////
			dssdev->state = OMAP_DSS_DISPLAY_ACTIVE_NO_DRAW;
			////////////////////////////////////////////////////

			if ( dssdev->platform_enable )
			{
				dssdev->platform_enable(dssdev);	//HDMI interface chip on
				//udelay(250);						//HDMI interface chip needs 250 micro second to work.
													//there are post actions, so doesn't wait
			}
			//to ensure clock enable
			dss_mainclk_state_enable();
			ret = hdmi_min_enable(dssdev);
			if ( ret )
			{
				DSSERR("HDMI Min enable failed\n");
				hdmi.status.status = HDMI_STATUS_UNDEFINED;
				return ret;
			}
			/* PAD0_HDMI_HPD_PAD1_HDMI_CEC */
			//enable HDMI HPD port
			omap_writel(0x01100110, 0x4A100098);
			hdmi_irq_enable();
			///set connection irq hanlder
			HDMI_WP_IRQ_set(HDMI_IRQ_PHY_LINK_DISCON | HDMI_IRQ_PHY_LINK_CONNECT);
			hdmi_wp_irq_add_handler(hdmi_wp_connection_irq_handler, NULL);
			if ( new_status==HDMI_STATUS_PLUG_DETECTION_ENABLED )
				break;
		case HDMI_STATUS_PLUG_DETECTION_ENABLED:
			//to PLUG REQUESTED

			//////////////////////////////////////////////////////
			dssdev->state = OMAP_DSS_DISPLAY_TRANSITION;
			//////////////////////////////////////////////////////

			/* PAD0_HDMI_DDC_SCL_PAD1_HDMI_DDC_SDA */
			//Set DDC : Display Data Channel. I2C interface in HDMI
				//DDC SDA Input Enable
				//DDC SDA Pull Up Select
				//DDC SCL InputEnable
				//DDC SCL Pull Up Select
				//DDC SCL Use
			omap_writel(0x01100110 , 0x4A10009C);
			/* CONTROL_HDMI_TX_PHY */
			//Unknown
			omap_writel(0x10000000, 0x4A100610);
			if ( new_status==HDMI_STATUS_PLUG_REQUESTED )
				break;
		case HDMI_STATUS_PLUG_REQUESTED:
			//to PLUG ESTABLISHED
			ret = omap_dss_start_device(dssdev);
			if ( ret )
			{
				DSSERR("Failed to start HDMI device\n");
				return ret;
			}
			hdmi_enable_clocks(1);
			ret = hdmi_connect_device(dssdev);
			if ( ret )
			{
				DSSERR("HDMI Power on failed\n");
				hdmi.status.status = HDMI_STATUS_UNDEFINED;
				return ret;
			}
// LGE_UPDATE_S 2011-07-01 for HDMI ACR workaround
			//When connection established, audio parameters are fixed
			//SO CTS can be sent from this phase
#ifdef CONFIG_OMAP_HDMI_AUDIO_WA
			DSSINFO("HDMI CTS start\n");
			ret = hdmi_lib_start_acr_wa();
			if ( ret )
			{
				DSSERR("Failed to start ACR workaround[%d]]\n", ret);
				return ret;
			}
#endif
// LGE_UPDATE_E 2011-07-01 for HDMI ACR workaround

#ifdef CONFIG_OMAP_PM
			{
				int retval =0;
				pr_debug("test\n");
				retval = omap_pm_set_min_bus_tput(&dssdev->dev,	OCP_INITIATOR_AGENT, 200 * 1000 * 4);		
				if (retval) {
					pr_err("%s %d Error setting MPU cstr\n", __func__, __LINE__);
					return; //return PM_UNSUPPORTED;
				}
				retval = omap_pm_set_max_mpu_wakeup_lat(&pm_hdmi_qos_handle,  IPU_PM_MM_MPU_LAT_CONSTRAINT);
				if (retval) {
					pr_err("%s %d Error setting MPU cstr\n", __func__, __LINE__);
					return -1; //return PM_UNSUPPORTED;
				}
			}
#endif

			wake_lock(&hdmi.status.hdmi_wake_lock);
			//Current status : There is no handler for HDMI_EVENT_POWER
			//Later hdmi notify should be removed
//			//not good
//			mutex_unlock(&hdmi.status_lock);
//			mdelay(50);		//I don't know T_T
//			hdmi_notify_pwrchange(HDMI_EVENT_POWERON);
//			mdelay(50);		//I don't know T_T
//			mutex_lock(&hdmi.status_lock);

			///////////////////////////////////////////
			dssdev->state = OMAP_DSS_DISPLAY_ACTIVE;
			///////////////////////////////////////////

			HDMI_device_connected(true);

			if ( new_status==HDMI_STATUS_PLUG_ESTABLISHED )
				break;
		case HDMI_STATUS_PLUG_ESTABLISHED:
			//to Un Plug

			HDMI_device_connected(false);
			/////////////////////////////////////////////////////
			dssdev->state = OMAP_DSS_DISPLAY_TRANSITION;
			/////////////////////////////////////////////////////

			break;
		default:
			DSSERR("Status(%d) is not in turn on sequence\n", status);
			return -EINVAL;
			break;
		}
	}
	else
	{
		//Disconnect & turn off

		HDMI_device_connected(false);

		switch ( status )
		{
		case HDMI_STATUS_PLUG_ESTABLISHED:
// LGE_UPDATE_S 2011-07-01 for HDMI ACR workaround
			//CTS Should must be done before power off
#ifdef CONFIG_OMAP_HDMI_AUDIO_WA
			DSSINFO("HDMI CTS stop\n");
			if (hdmi_lib_stop_acr_wa())
				DSSERR("HDMI WA may be in bad state");
#endif
// LGE_UPDATE_E 2011-07-01 for HDMI ACR workaround

			//to PLUG REQUEST
			//not good

			//disable front end chip ASAP to prevent short
			if ( dssdev->platform_disable )		//HDMI interface chip off
				dssdev->platform_disable(dssdev);

			mutex_unlock(&hdmi.status_lock);
			//@todo need check
			hdmi_notify_pwrchange(HDMI_EVENT_POWEROFF);
			mutex_lock(&hdmi.status_lock);

			omap_dss_stop_device(dssdev);
			HDMI_W1_SetWaitPllPwrState(HDMI_WP, HDMI_PLLPWRCMD_ALLOFF);
			hdmi_enable_clocks(0);
			//unlock
			wake_unlock(&hdmi.status.hdmi_wake_lock);

#ifdef CONFIG_OMAP_PM
			{
				int retval =0;
				pr_debug("test\n");
				retval = omap_pm_set_min_bus_tput(&dssdev->dev,	OCP_INITIATOR_AGENT, -1);		
				if (retval) {
					pr_err("%s %d Error setting MPU cstr\n", __func__, __LINE__);
					return; //return PM_UNSUPPORTED;
				}
				retval = omap_pm_set_max_mpu_wakeup_lat(&pm_hdmi_qos_handle, -1);
				if (retval) {
					pr_err("%s %d Error setting MPU cstr\n", __func__, __LINE__);
					return -1; //return PM_UNSUPPORTED;
				}
			}
#endif

			if ( new_status==HDMI_STATUS_PLUG_REQUESTED )
				break;
		case HDMI_STATUS_PLUG_REQUESTED:
			//to PLUG DETECTION

			////////////////////////////////////////
			dssdev->state = OMAP_DSS_DISPLAY_ACTIVE_NO_DRAW;
			////////////////////////////////////////

			if ( new_status==HDMI_STATUS_PLUG_DETECTION_ENABLED )
				break;
		case HDMI_STATUS_PLUG_DETECTION_ENABLED:
			///unset connection irq hanlder
			HDMI_WP_IRQ_unset(HDMI_IRQ_PHY_LINK_DISCON | HDMI_IRQ_PHY_LINK_CONNECT);
			hdmi_wp_irq_remove_handler(hdmi_wp_connection_irq_handler, NULL);
			//to OFF
			hdmi_status_to_off_with_status_lock(dssdev);
			break;
		default:
			DSSERR("Status(%d) is not in turn off sequence\n", status);
			return -EINVAL;
			break;
		}
	}
	DSSDBG("HDMI Status Transition (%d->%d) success\n", status, new_status);
	hdmi.status.status = new_status;
	return ret;
}

static int hdmi_status_move_to(enum hdmi_internal_status new_status)
{
	int r;
	mutex_lock(&hdmi.status_lock);
	r = hdmi_status_move_to_with_status_lock(new_status);
	mutex_unlock(&hdmi.status_lock);
	return r;
}

static void hdmi_check_status_and_transit(void)
{
	enum hdmi_internal_status status;
	status = hdmi_get_internal_status();
	if ( status==HDMI_STATUS_UNDEFINED )
	{
		hdmi_status_move_to(HDMI_STATUS_OFF);
		status = hdmi_get_internal_status();
	}
	switch ( status )
	{
	case HDMI_STATUS_OFF:
		mutex_lock(&hdmi.status_lock);
		if ( hdmi.status.user_hpd_request )
		{
			if ( hdmi_status_move_to_with_status_lock(HDMI_STATUS_PLUG_DETECTION_ENABLED) )
				DSSERR("HDMI PLUG Detection status tranistiona failed\n");
		}
		mutex_unlock(&hdmi.status_lock);
		break;
	case HDMI_STATUS_PLUG_DETECTION_ENABLED:
		mutex_lock(&hdmi.status_lock);
		if ( !hdmi.status.user_hpd_request )
		{
			if ( hdmi_status_move_to_with_status_lock(HDMI_STATUS_OFF) )
				DSSERR("HDMI OFF status tranistiona failed\n");
		}
		mutex_unlock(&hdmi.status_lock);
		break;
	default:
		break;
	}
}

#define HDMI_UNPLUG_OP_COMPLETE_WAIT_TIME	5000	//in milli second
#define HDMI_PLUG_START_WAIT_TIME			1000	//in milli

//wait on-going HDMI operation complete
//must call without holding lock
static void hdmi_unplug_wait_without_lock(void)
{

	DSSDBG("Waiting HDMI video stop\n");
	if ( !wait_event_timeout(hdmi.play_status.video_producer_wq,
								hdmi.play_status.video_play_enabled==false && hdmi.play_status.video_producer_cnt==0,
								HDMI_UNPLUG_OP_COMPLETE_WAIT_TIME * HZ /1000) )
	{
		struct omap_dss_device *dssdev = get_hdmi_device();
		DSSERR("Video not stopped durring(%dms) force to stop\n", HDMI_UNPLUG_OP_COMPLETE_WAIT_TIME);
		//error recovery sequence, so error return will not be handled here

		//clear & stop video play
		hdmi_video_stop_force_internal();

		//unset overlays
		if ( dssdev!=NULL )
		{
			int i, mgr_count = omap_dss_get_num_overlay_managers();
			struct omap_overlay_manager *mgr = NULL;

			//find HDMI overlay manager
			for(i=0;i<mgr_count;i++)
			{
				struct omap_overlay_manager *ith = omap_dss_get_overlay_manager(i);
				if ( ith!=NULL && ith->device==dssdev )
				{
					mgr = ith;
					break;
				}
			}

			//unset all overlays connected to HDMI
			if ( mgr!=NULL )
			{
				//disable overlay
				int ovl_cnt;
				ovl_cnt = omap_dss_get_num_overlays();
				for(i=0;i<ovl_cnt;i++)
				{
					struct omap_overlay *ovl;
					ovl = omap_dss_get_overlay(i);
					if ( ovl!=NULL && ovl->manager==mgr )
					{
						struct omap_overlay_info info;
						ovl->get_overlay_info(ovl, &info);
						if ( info.enabled )
						{
							info.enabled = false;
							ovl->set_overlay_info(ovl, &info);
						}
					}
				}
				mgr->apply(mgr);
			}
		}
	}
	DSSDBG("Waiting HDMI Sound Disable\n");
	if ( !hdmi_audio_disable_wait(HDMI_UNPLUG_OP_COMPLETE_WAIT_TIME) )
	{
		DSSERR("Sound not stopped during(%dms) force to stop\n", HDMI_UNPLUG_OP_COMPLETE_WAIT_TIME);
		hdmi_w1_stop_audio_transfer(HDMI_WP);
		hdmi_w1_wrapper_disable(HDMI_WP);
	}
	DSSDBG("Waiting complteted\n");
}

/**
 * Stop HDMI plug-in
 */
static int hdmi_stop_plug_with_status_lock(struct omap_dss_device *dssdev)
{
	//set HPD LOW for plug detection
	hdmi.status.hpd_status = HDMI_HPD_LOW;
	//set as disconnect
	HDMI_device_connected(false);

	//unlock mutex and wait operation complete
	mutex_unlock(&hdmi.status_lock);
	hdmi_send_uevent(dssdev, KOBJ_REMOVE);
	hdmi_unplug_wait_without_lock();
	mutex_lock(&hdmi.status_lock);

	//turn off & on again
	if ( !hdmi_status_move_to_with_status_lock(HDMI_STATUS_OFF) )
	{
		//give gap 1 second
		return 0;
	}
	else
	{
		DSSERR("HDMI PLUG DETECTION status change failed\n");
		return -EIO;
	}
}

/**
 * Start HDMI Plug-in
 */
static int hdmi_start_plug_with_status_lock(struct omap_dss_device *dssdev)
{
	//consider as HDP HIGH (for disconnection detection)
	hdmi.status.hpd_status = HDMI_HPD_HIGH;
	if ( !hdmi_status_move_to_with_status_lock(HDMI_STATUS_PLUG_ESTABLISHED) )
	{
//		mutex_unlock(&hdmi.status_lock);
		DSSINFO("HDMI Plugged and eanbeld, sending uEvent\n");
//		hdmi_notify_hpd(1);
		hdmi_send_uevent(dssdev, KOBJ_ADD);
//		mutex_lock(&hdmi.status_lock);
		return 0;
	}
	else
	{
		DSSERR("HDMI PLUG REQEUST failed\n");
		return -EIO;
	}
}

static void hdmi_irq_process_with_status_lock(int hpd)
{
	struct omap_dss_device *dssdev = get_hdmi_device();
	enum hdmi_internal_status status;

	if ( dssdev==NULL )
	{
		DSSERR("dssdev in hdmi_work_queue() is null\n");
		return;
	}
	if ( hdmi.status.status==HDMI_STATUS_OFF )
	{
		DSSDBG("dssdev->state is not active\n");
		return;
	}

	//very very not good
	if ( hpd==HDMI_HPD_LOW )
		hdmi_notify_hpd(0);

	status = hdmi.status.status;
	if ( hpd==HDMI_HPD_HIGH )
	{
		if ( hdmi_start_plug_with_status_lock(dssdev)== 0)
			goto IRQ_HANDLED;
	}
	else if ( hpd==HDMI_HPD_LOW )
	{
		DSSINFO("HDMI Un-Plugged\n");
		if ( hdmi.status.status != HDMI_STATUS_PLUG_ESTABLISHED )
		{
			DSSINFO("HDMI not connected. stay at same status\n");
			goto IRQ_HANDLED;
		}

		if ( hdmi_stop_plug_with_status_lock(dssdev)==0 )
			goto IRQ_HANDLED;
	}
	else
	{
		DSSERR("Invalid Event\n");
	}
//IRQ_HANDLING_FAIL:
	DSSERR("HDMI status(%d) and hpd irq(0x%x) handling failed. goto undefined status\n", status, hpd);
	hdmi.status.status = HDMI_STATUS_UNDEFINED;
	return;

IRQ_HANDLED:
	DSSDBG("HDMI status(%d) and hpd irq(0x%x) handling success\n", status, hpd);
	return;
}

static int hdmi_connection_handler(void *data)
{
	while(1)
	{
		int hpd;
		int changed;
		bool hpd_status;
		//wait irq assert
		wait_event(hdmi.plug_status.change_wait, hdmi.plug_status.changed==true);
CHANGE_WAIT:
		if ( kthread_should_stop() )
			break;
		mutex_lock(&hdmi.plug_status_lock);
		hdmi.plug_status.changed = false;
		mutex_unlock(&hdmi.plug_status_lock);

		//if there is no irq assert during 500ms
		DSSDBG("Waiting HDMI connection status change during 500ms\n");
		changed = wait_event_timeout(hdmi.plug_status.change_wait,
							hdmi.plug_status.changed==true,
							5 * HZ / 10	);
		if ( changed )	//irq asserted
		{
			DSSDBG("HDMI connection status changed. wait again\n");
			goto CHANGE_WAIT;
		}

		//there was no connection related events during 500ms
		//consider as stable connection status

		//according HDMI interface chip spec, 150 micro seconds are needed to enable Level Shift.
		//This can't guarantee correction operation, anyway we need delay
		mdelay(50);

		mutex_lock(&hdmi.status_lock);
		hpd_status = hdmi_get_hpd_status_with_status_lock();
		DSSDBG("HDMI connection status is Determined by hdp status(%d)\n", hpd_status);
		if ( hpd_status )
		{
			if ( hdmi_rxdet() )
				hpd = HDMI_HPD_HIGH;
			else
				hpd = HDMI_HPD_LOW;
		}
		else
			hpd = HDMI_HPD_LOW;

		if ( hdmi.status.hpd_status==hpd )
		{
			DSSDBG("Same status(%d) skip\n", hpd);
			mutex_unlock(&hdmi.status_lock);
			continue;
		}
		hdmi.status.hpd_status = hpd;

		DSSDBG("\n***********************************************************\n");
		DSSDBG("* HDMI IRQ Processing enter to handle irq(%s)    *\n", hpd==HDMI_HPD_HIGH ? "plug" : "unplug");
		hdmi_irq_process_with_status_lock(hpd);
		if ( hpd==HDMI_HPD_LOW )
		{
			//sleep 1second
//			msleep(1000);
		}
		DSSDBG("\n* HDMI IRQ Processing exit                                *\n");
		DSSDBG("***********************************************************\n\n");
		mutex_unlock(&hdmi.status_lock);
		//checkt current status and transit to right status
		hdmi_check_status_and_transit();

	}
	return 0;
}

/**
 * Raise HDMI plug status change
 */
static void hdmi_raise_plug_status_change(void)
{
	mutex_lock(&hdmi.plug_status_lock);
	DSSDBG("HDMI Connection status change raised\n");
	hdmi.plug_status.changed = true;
	mutex_unlock(&hdmi.plug_status_lock);

	wake_up(&hdmi.plug_status.change_wait);
}

/**
 * Raise FAKE HDMI plug status change
 */
static void hdmi_raise_fake_plug_status_change(void)
{
	if ( hdmi_get_internal_status() >= HDMI_STATUS_PLUG_DETECTION_ENABLED )
	{
		DSSDBG("Raise Fake HDMI plug change\n");
		hdmi_raise_plug_status_change();
	}
}

//HDMI connection change notifiler
static void hdmi_wp_connection_change_notifier(struct work_struct *ws)
{
	if ( ws==NULL )
		return;
	kfree(ws);
	//inform connection status change
	hdmi_raise_plug_status_change();
}

//Handling HDMI Physical connect & disconnect
static void hdmi_wp_connection_irq_handler(u32 wp_irq, void *data)
{
	if ( (wp_irq & HDMI_IRQ_PHY_LINK_DISCON) || (wp_irq & HDMI_IRQ_PHY_LINK_CONNECT) )
	{
		struct work_struct *work;
		work = kmalloc(sizeof(*work), GFP_ATOMIC);
		if ( work )
		{
			INIT_WORK(work, hdmi_wp_connection_change_notifier);
			queue_work(hdmi.work_queue, work);
		}
	}
}


//add irq handler
static int hdmi_wp_irq_add_handler(hdmi_irq_handler handler, void *data)
{
	unsigned long flags;
	struct hdmi_irq_handler_item *item;
	spin_lock_irqsave(&irqstatus_lock, flags);

	item = (struct hdmi_irq_handler_item*)kmalloc(sizeof(*item), GFP_ATOMIC);
	if ( item==NULL )
	{
		spin_unlock_irqrestore(&irqstatus_lock, flags);
		return -ENOMEM;
	}
	item->handler = handler;
	item->data	= data;
	list_add_tail(&item->queue, &hdmi.irq.handlers);
	//DSSDBG("HDMI Irq Handler(%p,%p) added\n", handler, data);
	spin_unlock_irqrestore(&irqstatus_lock, flags);
	return 0;
}

static int hdmi_wp_irq_remove_handler(hdmi_irq_handler handler, void *data)
{
	int r = -EINVAL;
	unsigned long flags;
	spin_lock_irqsave(&irqstatus_lock, flags);
	if ( !list_empty(&hdmi.irq.handlers) )
	{
		struct list_head *pos, *n;
		list_for_each_safe(pos, n, &hdmi.irq.handlers)
		{
			struct hdmi_irq_handler_item *item;
			item = list_entry(pos, struct hdmi_irq_handler_item, queue);
			if ( item->handler==handler && item->data==data )
			{
				list_del(pos);
				kfree(item);
				r = 0;
				//DSSDBG("HDMI Irq Handler(%p,%p) removed\n", handler, data);
				goto Done;
			}
		}
	}
Done:
	spin_unlock_irqrestore(&irqstatus_lock, flags);

	return r;
}

static irqreturn_t hdmi_wp_irq_main_handler(int irq, void *arg)
{
	unsigned long flags;
	int wp_irq = 0;

	spin_lock_irqsave(&irqstatus_lock, flags);

	//get WP irq
	HDMI_WP_IRQ_get_status(&wp_irq);

	//call irq handlers
	if ( wp_irq && !list_empty(&hdmi.irq.handlers) )
	{
		struct list_head *pos, *n;
		//DSSDBG("HDMI irq 0x%08x handing start\n", wp_irq);
		list_for_each_safe(pos, n, &hdmi.irq.handlers)
		{
			struct hdmi_irq_handler_item *item;
			item = list_entry(pos, struct hdmi_irq_handler_item, queue);
			if ( item->handler!=NULL )
			{
				item->handler(wp_irq, item->data);
				//DSSDBG("HDMI irq handler called\n");
			}
		}
		//DSSDBG("HDMI irq handing end\n");
	}


	spin_unlock_irqrestore(&irqstatus_lock, flags);

	return IRQ_HANDLED;
}

static void hdmi_power_off_phy(void)
{
	mutex_lock(&hdmi.connection_lock);
	hdmi.connection.edid_set = false;
	mutex_unlock(&hdmi.connection_lock);

	hdmi_phy_off(HDMI_WP);

	HDMI_W1_SetWaitPllPwrState(HDMI_WP, HDMI_PLLPWRCMD_ALLOFF);
}

static int hdmi_enable_hpd(struct omap_dss_device *dssdev)
{
	DSSDBG("ENTER hdmi_enable_hpd()\n");
	mutex_lock(&hdmi.status_lock);
	hdmi.status.user_hpd_request = 1;
	mutex_unlock(&hdmi.status_lock);
	hdmi_check_status_and_transit();
	//force HDMI connection check by raising fake HDMI plug change
	hdmi_raise_fake_plug_status_change();
	return 0;
}

static int hdmi_disable_hpd(struct omap_dss_device *dssdev)
{
	DSSDBG("ENTER hdmi_disable_hpd()\n");
	mutex_lock(&hdmi.status_lock);
	hdmi.status.user_hpd_request = 0;
	mutex_unlock(&hdmi.status_lock);
	hdmi_check_status_and_transit();
	return 0;
}


/*
 * For timings change to take effect, phy needs to be off. Interrupts will be
 * ignored during this sequence to avoid queuing work items that generate
 * hotplug events.
 */
static int hdmi_reset(struct omap_dss_device *dssdev,
					enum omap_dss_reset_phase phase)
{
	int r = 0;
	return r;
}


static void hdmi_get_timings(struct omap_dss_device *dssdev,
			struct omap_video_timings *timings)
{
	*timings = dssdev->panel.timings;
}

static void hdmi_set_timings(struct omap_dss_device *dssdev,
			struct omap_video_timings *timings)
{
	//LGE_UPDATE_S 2011-05-09 hdmi reset modifcation for HDMI resolution chage
	int ret = 0;
	//LGE_UPDATE_E 2011-05-09 hdmi reset modifcation for HDMI resolution chage

	DSSDBG("hdmi_set_timings\n");

	dssdev->panel.timings = *timings;

	if ( get_dss_state(dssdev) == OMAP_DSS_DISPLAY_ACTIVE)
	{
		//LGE_UPDATE_S 2011-05-09 hdmi reset modifcation for HDMI resolution chage
		/* turn the phy off and on to get new timings to use */
		//hdmi_reset(dssdev, OMAP_DSS_RESET_BOTH);

		mutex_lock(&hdmi.status_lock);
		ret = hdmi_connect_device(dssdev);
		if ( ret )
		{
			DSSERR("HDMI Power on failed\n");
			hdmi.status.status = HDMI_STATUS_UNDEFINED;
		}
		mutex_unlock(&hdmi.status_lock);
		hdmi_check_status_and_transit();
		//LGE_UPDATE_E 2011-05-09 hdmi reset modifcation for HDMI resolution chage
	}
	
}

static void hdmi_set_custom_edid_timing_code(struct omap_dss_device *dssdev,
							int code, int mode)
{
	if ((!mode && code < ARRAY_SIZE(code_vesa) && code_vesa[code] >= 0) ||
	    (mode == 1 && code < ARRAY_SIZE(code_cea) && code_cea[code] >= 0)) {
		mutex_lock(&hdmi.connection_lock);
		hdmi.connection.code = code;
		hdmi.connection.mode = mode;
		hdmi.connection.custom_set = 1;
		DSSINFO("HDMI Custom Set code(%d), mode(%d)\n", code, mode);
		mutex_unlock(&hdmi.connection_lock);
	}
	else
	{
		mutex_lock(&hdmi.connection_lock);
		hdmi.connection.custom_set = 0;
		DSSINFO("HDMI Custom unset\n");
		mutex_unlock(&hdmi.connection_lock);

	}

	/* for now only set this while on or on HPD */
	if ( get_dss_state(dssdev) != OMAP_DSS_DISPLAY_ACTIVE)
		return;

	mutex_lock(&hdmi.status_lock);
	if ( hdmi.status.status==HDMI_STATUS_PLUG_ESTABLISHED )
	{
		//stop & start
		hdmi_stop_plug_with_status_lock(dssdev);
		hdmi_start_plug_with_status_lock(dssdev);
	}
	mutex_unlock(&hdmi.status_lock);
}

static struct hdmi_cm hdmi_get_code(struct omap_video_timings *timing)
{
	int i = 0, code = -1, temp_vsync = 0, temp_hsync = 0;
	int timing_vsync = 0, timing_hsync = 0;
	struct omap_video_timings temp;
	struct hdmi_cm cm = {-1};
	DSSDBG("hdmi_get_code");

	for (i = 0; i < ARRAY_SIZE(all_timings_direct); i++) {
		temp = all_timings_direct[i];
		if (temp.pixel_clock != timing->pixel_clock ||
		    temp.x_res != timing->x_res ||
		    temp.y_res != timing->y_res)
			continue;

		temp_hsync = temp.hfp + temp.hsw + temp.hbp;
		timing_hsync = timing->hfp + timing->hsw + timing->hbp;
		temp_vsync = temp.vfp + temp.vsw + temp.vbp;
		timing_vsync = timing->vfp + timing->vsw + timing->vbp;

		printk(KERN_INFO "Temp_hsync = %d, temp_vsync = %d, "
			"timing_hsync = %d, timing_vsync = %d",
			temp_hsync, temp_hsync, timing_hsync, timing_vsync);

		if (temp_hsync == timing_hsync && temp_vsync == timing_vsync) {
			code = i;
			cm.code = code_index[i];
			cm.mode = code < OMAP_HDMI_TIMINGS_VESA_START;
			DSSDBG("Hdmi_code = %d mode = %d\n", cm.code, cm.mode);
			print_omap_video_timings(&temp);
			break;
		}
	}
	return cm;
}

static void hdmi_get_edid(struct omap_dss_device *dssdev)
{
	u8 i = 0, mark = 0, *e;
	int offset, addr, length;
	struct HDMI_EDID *edid_st;
	struct image_format *img_format;
	struct audio_format *aud_format;
	struct deep_color *vsdb_format;
	struct latency *lat;
	struct omap_video_timings timings;
	struct hdmi_cm cm;
	int no_timing_info = 0;

	img_format = kzalloc(sizeof(*img_format), GFP_KERNEL);
	if (!img_format) {
		WARN_ON(1);
		return;
	}
	
	aud_format = kzalloc(sizeof(*aud_format), GFP_KERNEL);
	if (!aud_format) {
		WARN_ON(1);
		goto hdmi_get_err1;
	}
	
	vsdb_format = kzalloc(sizeof(*vsdb_format), GFP_KERNEL);
	if (!vsdb_format) {
		WARN_ON(1);
		goto hdmi_get_err2;
	}
	
	lat = kzalloc(sizeof(*lat), GFP_KERNEL);
	if (!lat) {
		WARN_ON(1);
		goto hdmi_get_err3;
	}

	mutex_lock(&hdmi.connection_lock);
	edid_st = (struct HDMI_EDID *)hdmi.connection.edid;
	if (!hdmi.connection.edid_set) {
		DSSERR("Display doesn't seem to be enabled invalid read\n");
		if (HDMI_CORE_DDC_READEDID(HDMI_CORE_SYS, hdmi.connection.edid,
						HDMI_EDID_MAX_LENGTH) != 0)
			DSSERR("HDMI failed to read E-EDID\n");
		hdmi.connection.edid_set = !memcmp(hdmi.connection.edid, header, sizeof(header));
	}

	mdelay(100);

	e = hdmi.connection.edid;
	DSSINFO("\nHeader:\n%02x\t%02x\t%02x\t%02x\t%02x\t%02x\t"
		"%02x\t%02x\n", e[0], e[1], e[2], e[3], e[4], e[5], e[6], e[7]);
	e += 8;
	DSSINFO("Vendor & Product:\n%02x\t%02x\t%02x\t%02x\t%02x\t"
		"%02x\t%02x\t%02x\t%02x\t%02x\n",
		e[0], e[1], e[2], e[3], e[4], e[5], e[6], e[7], e[8], e[9]);
	e += 10;
	DSSINFO("EDID Structure:\n%02x\t%02x\n",
		e[0], e[1]);
	e += 2;
	DSSINFO("Basic Display Parameter:\n%02x\t%02x\t%02x\t%02x\t%02x\n",
		e[0], e[1], e[2], e[3], e[4]);
	e += 5;
	DSSINFO("Color Characteristics:\n%02x\t%02x\t%02x\t%02x\t"
		"%02x\t%02x\t%02x\t%02x\t%02x\t%02x\n",
		e[0], e[1], e[2], e[3], e[4], e[5], e[6], e[7], e[8], e[9]);
	e += 10;
	printk("Established timings:\n%02x\t%02x\t%02x\n",
		e[0], e[1], e[2]);
	e += 3;
	DSSINFO("Standard timings:\n%02x\t%02x\t%02x\t%02x\t%02x\t"
			 "%02x\t%02x\t%02x\n",
		e[0], e[1], e[2], e[3], e[4], e[5], e[6], e[7]);
	e += 8;
	DSSINFO("%02x\t%02x\t%02x\t%02x\t%02x\t%02x\t%02x\t%02x\n",
		e[0], e[1], e[2], e[3], e[4], e[5], e[6], e[7]);
	e += 8;

	for (i = 0; i < EDID_SIZE_BLOCK0_TIMING_DESCRIPTOR; i++) {
		DSSINFO("Extension 0 Block %d\n", i);
		no_timing_info = get_edid_timing_info(&edid_st->DTD[i], &timings);
		mark = dss_debug;
		dss_debug = 1;
		if (!no_timing_info) {
			cm = hdmi_get_code(&timings);
			print_omap_video_timings(&timings);
			DSSINFO("Video code: %d video mode %d",
				cm.code, cm.mode);
		}
		dss_debug = mark;
	}
	if (hdmi.connection.edid[0x7e] != 0x00) {
		offset = hdmi.connection.edid[EDID_DESCRIPTOR_BLOCK1_ADDRESS + 2];
		DSSINFO("offset %x\n", offset);
		if (offset != 0) {
			addr = EDID_DESCRIPTOR_BLOCK1_ADDRESS + offset;
			/*to determine the number of descriptor blocks */
			for (i = 0; i < EDID_SIZE_BLOCK1_TIMING_DESCRIPTOR;
									i++) {
				printk(KERN_INFO "Extension 1 Block %d", i);
				get_eedid_timing_info(addr, hdmi.connection.edid, &timings);
				addr += EDID_TIMING_DESCRIPTOR_SIZE;
				mark = dss_debug;
				dss_debug = 1;
				print_omap_video_timings(&timings);
				cm = hdmi_get_code(&timings);
				DSSINFO("Video code: %d video mode %d",
					cm.code, cm.mode);
				dss_debug = mark;
			}
		}
		hdmi_get_video_svds(hdmi.connection.edid, &offset, &length);
		DSSINFO("No of SVDs: %d", length);
		for (i = 0; i < length; i++) {
			DSSINFO("SVD[%d]: CEA code[%d], native[%s]",
				i, hdmi.connection.edid[offset+i] & HDMI_EDID_EX_VIDEO_MASK,
				hdmi.connection.edid[offset+i] & HDMI_EDID_EX_VIDEO_NATIVE ?
				"YES" : "NO");
		}
		DSSINFO("No. of native DTD: %d",
				(hdmi.connection.edid[EDID_DESCRIPTOR_BLOCK1_ADDRESS + 3]
			& HDMI_VIDEO_NATIVE_DTDS_MASK));
		DSSINFO("Supports basic audio: %s",
				(hdmi.connection.edid[EDID_DESCRIPTOR_BLOCK1_ADDRESS + 3]
			& HDMI_AUDIO_BASIC_MASK) ? "YES" : "NO");
	}
	DSSINFO("Has IEEE HDMI ID: %s",
		hdmi_has_ieee_id(hdmi.connection.edid) ? "YES" : "NO");
	DSSINFO("Supports AI: %s", hdmi.connection.cfg.supports_ai ?
		"YES" : "NO");
	hdmi_get_image_format(hdmi.connection.edid, img_format);
	DSSINFO("%d audio length\n", img_format->length);
	for (i = 0 ; i < img_format->length ; i++)
		DSSINFO("%d %d pref code\n",
			img_format->fmt[i].pref, img_format->fmt[i].code);

	hdmi_get_audio_format(hdmi.connection.edid, aud_format);
	DSSINFO("%d audio length\n", aud_format->length);
	for (i = 0 ; i < aud_format->length ; i++)
		printk(KERN_INFO "%d %d format num_of_channels\n",
			aud_format->fmt[i].format,
			aud_format->fmt[i].num_of_ch);

	hdmi_deep_color_support_info(hdmi.connection.edid, vsdb_format);
	DSSINFO("%d deep color bit 30 %d  deep color 36 bit "
		"%d max tmds freq", vsdb_format->bit_30, vsdb_format->bit_36,
		vsdb_format->max_tmds_freq);

	hdmi_get_av_delay(hdmi.connection.edid, lat);
	DSSINFO("%d vid_latency %d aud_latency "
		"%d interlaced vid latency %d interlaced aud latency",
		lat->vid_latency, lat->aud_latency,
		lat->int_vid_latency, lat->int_aud_latency);

	mutex_unlock(&hdmi.connection_lock);

	DSSINFO("YUV supported %d", hdmi_tv_yuv_supported(hdmi.connection.edid));
	kfree(lat);

hdmi_get_err3:
	kfree(vsdb_format);
hdmi_get_err2:
	kfree(aud_format);
hdmi_get_err1:
	kfree(img_format);
}

static int hdmi_check_timings(struct omap_dss_device *dssdev,
			struct omap_video_timings *timings)
{
	DSSDBG("hdmi_check_timings\n");

	if (memcmp(&dssdev->panel.timings, timings, sizeof(*timings)) == 0)
		return 0;

	return -EINVAL;
}

static int hdmi_set_s3d_disp_type(struct omap_dss_device *dssdev,
						struct s3d_disp_info *info)
{
	int r = -EINVAL;
	struct hdmi_s3d_info tinfo;

	tinfo.structure = 0;
	tinfo.subsamp = false;
	tinfo.subsamp_pos = 0;

	switch (info->type) {
	case S3D_DISP_OVERUNDER:
		if (info->sub_samp == S3D_DISP_SUB_SAMPLE_NONE) {
			tinfo.structure = HDMI_S3D_TOP_AND_BOTTOM;
			r = 0;
		} else {
			goto err;
		}
		break;
	case S3D_DISP_SIDEBYSIDE:
		if (info->sub_samp == S3D_DISP_SUB_SAMPLE_H) {
			tinfo.structure = HDMI_S3D_SIDE_BY_SIDE_HALF;
			tinfo.subsamp = true;
			tinfo.subsamp_pos = HDMI_S3D_HOR_EL_ER;
			r = 0;
		} else {
			goto err;
		}
		break;
	default:
		goto err;
	}
	mutex_lock(&hdmi.connection_lock);
	hdmi.connection.s3d_info = tinfo;
	mutex_unlock(&hdmi.connection_lock);
err:
	return r;
}

static int hdmi_enable_s3d(struct omap_dss_device *dssdev, bool enable)
{
	int r = 0;

	mutex_lock(&hdmi.status_lock);
	if ( enable && ( hdmi.status.status != HDMI_STATUS_PLUG_ESTABLISHED || !HDMI_is_device_connected() ) )
	{
		mutex_unlock(&hdmi.status_lock);
		//DSSERR("HDMI Not plug established\n");
		return 0;
	}
	mutex_lock(&hdmi.connection_lock);
	if ( !hdmi.connection.s3d_switch_support )
	{
		//DSSERR("This HDMI devices doesn't support S3D switching\n");
		r = 0;
		goto Done;
	}
	if ( enable==hdmi.connection.s3d_enabled )
	{
		r = 0;
		goto Done;
	}
	//when VSI change, stop and restart HDMI.
	//To prevent flicking when switching 3D
	mutex_lock(&hdmi.play_status_lock);
	if ( enable )
	{
		int delay;
		//if one more video producer exist, there will be overlay changes.
		//So stop video
		hdmi_video_stop_real_with_play_status_lock();
		//not good. when video producer of s3d should be added, give more delay
		if ( !hdmi.play_status.dirty && hdmi.play_status.video_producer_cnt<2 )
			delay = 1500;
		else
			delay = 0;
		hdmi.play_status.dirty = true;
		//not good. no way to know when configuration will be stabilized
		hdmi_video_start_real_with_play_status_lock(delay);
	}
	mutex_unlock(&hdmi.play_status_lock);
	if ( enable )
	{
		struct hdmi_s3d_config config;
		switch ( hdmi.connection.s3d_info.structure )
		{
		case HDMI_S3D_SIDE_BY_SIDE_HALF:
			DSSINFO("HDMI Side by Side Format\n");
			config.structure = HDMI_S3D_SIDE_BY_SIDE_HALF;	//3D Structure : 1000 Side-by-Side(Half)
			config.s3d_ext_data = HDMI_S3D_HOR_EL_ER;		//3D Ext Data : 0011 horizontal sub-sampling
			break;
		case HDMI_S3D_TOP_AND_BOTTOM:
			DSSINFO("HDMI Top & Bottom Format\n");
			config.structure = HDMI_S3D_TOP_AND_BOTTOM;		//3D Structure : 0110 Top-and-Bottom
			config.s3d_ext_data = 0;
			break;
		default:
			DSSERR("We only support top-bottom and side-by-side\n");
			r = -EINVAL;
			goto Done;
		}
		r = HDMI_S3D_VSI_set(&config);
		if ( r )
		{
			DSSERR("HDMI set S3D VSI failed\n");
			goto Done;
		}
		DSSINFO("HDMI S3D VSI set success\n");
		hdmi.connection.s3d_enabled = enable;
	}
	else
	{
		DSSINFO("Turn off HDMI S3D VSI setting\n");
		r = HDMI_S3D_VSI_unset();
		if ( r )
		{
			DSSERR("Disable HDMI S3D failed\n");
			goto Done;
		}
		hdmi.connection.s3d_enabled = enable;
	}
Done:
	mutex_unlock(&hdmi.connection_lock);
	mutex_unlock(&hdmi.status_lock);
	return r;

}


static bool hdmi_get_s3d_enabled(struct omap_dss_device *dssdev)
{
	bool enabled;
	mutex_lock(&hdmi.connection_lock);
	enabled = hdmi.connection.s3d_enabled;
	mutex_unlock(&hdmi.connection_lock);
	return enabled;
}


int hdmi_init_display(struct omap_dss_device *dssdev)
{
	printk("init_display\n");

	/* register HDMI specific sysfs files */
	/* note: custom_edid_timing should perhaps be moved here too,
	 * instead of generic code?  Or edid sysfs file should be moved
	 * to generic code.. either way they should be in same place..
	 */
	if (device_create_file(&dssdev->dev, &dev_attr_edid))
		DSSERR("failed to create sysfs file\n");
	if (device_create_file(&dssdev->dev, &dev_attr_yuv))
		DSSERR("failed to create sysfs file\n");
	if (device_create_file(&dssdev->dev, &dev_attr_hdmi_out))
		DSSERR("failed to create sysfs file\n");
	if (device_create_file(&dssdev->dev, &dev_attr_hdmi_source))
		DSSERR("failed to create sysfs file\n");
	if (device_create_file(&dssdev->dev, &dev_attr_hdmi_key))
		DSSERR("failed to create sysfs file\n");
	if (device_create_file(&dssdev->dev, &dev_attr_deepcolor))
		DSSERR("failed to create sysfs file\n");
	if (device_create_file(&dssdev->dev, &dev_attr_lr_fr))
		DSSERR("failed to create sysfs file\n");
	if (device_create_file(&dssdev->dev, &dev_attr_code))
		DSSERR("failed to create sysfs file\n");
	if (device_create_file(&dssdev->dev, &dev_attr_hdmi_drm_lock))
		DSSERR("failed to create sysfs file\n");

	return 0;
}

static int hdmi_read_edid_with_connection_lock(struct omap_video_timings *dp)
{
//LGE_CHANGE [taekeun1.kim@lge.com] 2011-01-12, P920 : For WBT
	int r = 0, ret=0, code=0;

	memset(hdmi.connection.edid, 0, HDMI_EDID_MAX_LENGTH);
	if (!hdmi.connection.edid_set)
	{
		HDMI_W1_SetWaitSoftReset();
		ret = HDMI_CORE_DDC_READEDID(HDMI_CORE_SYS, hdmi.connection.edid,
							HDMI_EDID_MAX_LENGTH);
	}
	if (ret != 0) {
		printk(KERN_WARNING "HDMI failed to read E-EDID\n");
//		return ret;
	} else {
		if (!memcmp(hdmi.connection.edid, header, sizeof(header))) {
			/* LGE_CHANGE [wonki.choi@lge.com] HDMI S3D Switching 2011-2-09 */
			hdmi.connection.s3d_switch_support = hdmi_s3d_supported(hdmi.connection.edid);
			if (hdmi.connection.s3d_enabled) {
				/* Update flag to convey if sink supports 3D */
				hdmi.connection.s3d_enabled = hdmi_s3d_supported(hdmi.connection.edid);
			}
			/* search for timings of default resolution */
			if (get_edid_timing_data_with_connection_lock((struct HDMI_EDID *) hdmi.connection.edid))
				hdmi.connection.edid_set = true;
		}
	}

	if (!hdmi.connection.edid_set) {
		DSSDBG("fallback to VGA\n");
		hdmi.connection.code = 16; //4; /*setting default value of 640 480 VGA*/
		hdmi.connection.mode = 1;
		hdmi.connection.edid_set = true;
	}
	if (hdmi.connection.s3d_enabled && hdmi.connection.s3d_info.structure == HDMI_S3D_FRAME_PACKING)
		code = get_s3d_timings_index_with_connection_lock();
	else
		code = get_timings_index_with_connection_lock();

	*dp = all_timings_direct[code];

	DSSDBG(KERN_INFO"hdmi read EDID:\n");
	print_omap_video_timings(dp);

	return r;
}

/*
 *------------------------------------------------------------------------------
 * Function    : get_edid_timing_data
 *------------------------------------------------------------------------------
 * Description : This function gets the resolution information from EDID
 *
 * Parameters  : void
 *
 * Returns     : void
 *------------------------------------------------------------------------------
 */
static int get_edid_timing_data_with_connection_lock(struct HDMI_EDID *edid)
{
	u8 i, j, code, offset = 0, addr = 0;
	struct hdmi_cm cm;
	bool audio_support = false;
	int svd_base, svd_length, svd_code, svd_native;

	/*
	 *  Verify if the sink supports audio
	 */
	/* check if EDID has CEA extension block */
	if ((edid->extension_edid != 0x00))
		/* check if CEA extension block is version 3 */
		if (edid->extention_rev == 3)
			/* check if extension block has the IEEE HDMI ID*/
			if (hdmi_has_ieee_id((u8 *)edid))
				/* check if sink supports basic audio */
				if (edid->num_dtd & HDMI_AUDIO_BASIC_MASK)
					audio_support = true;

	/* Seach block 0, there are 4 DTDs arranged in priority order */
	for (i = 0; i < EDID_SIZE_BLOCK0_TIMING_DESCRIPTOR; i++) {
		get_edid_timing_info(&edid->DTD[i], &hdmi.connection.edid_timings);
		DSSDBG("Block0 [%d] timings:", i);
		print_omap_video_timings(&hdmi.connection.edid_timings);
		cm = hdmi_get_code(&hdmi.connection.edid_timings);
		DSSDBG("Block0[%d] value matches code = %d , mode = %d",
			i, cm.code, cm.mode);
		if (cm.code == -1)
			continue;
		if (hdmi.connection.s3d_enabled && s3d_code_cea[cm.code] == -1)
			continue;
		/* if sink supports audio, use CEA video timing */
		if (audio_support && !cm.mode)
			continue;
		hdmi.connection.code = cm.code;
		hdmi.connection.mode = cm.mode;
		DSSDBG("code = %d , mode = %d", hdmi.connection.code, hdmi.connection.mode);
		return 1;
	}
	/* Search SVDs in block 1 twice: first natives and then all */
	if (edid->extension_edid != 0x00) {
		hdmi_get_video_svds((u8 *)edid, &svd_base, &svd_length);
		for (j = 1; j >= 0; j--) {
			for (i = 0; i < svd_length; i++) {
				svd_native = ((u8 *)edid)[svd_base+i]
					& HDMI_EDID_EX_VIDEO_NATIVE;
				svd_code = ((u8 *)edid)[svd_base+i]
					& HDMI_EDID_EX_VIDEO_MASK;
				if (svd_code >= ARRAY_SIZE(code_cea))
					continue;
				/* Check if this SVD is native*/
				if (!svd_native && j)
					continue;
				/* Check if this 3D CEA timing is supported*/
				if (hdmi.connection.s3d_enabled &&
					s3d_code_cea[svd_code] == -1)
					continue;
				/* Check if this CEA timing is supported*/
				if (code_cea[svd_code] == -1)
					continue;
				hdmi.connection.code = svd_code;
				hdmi.connection.mode = 1;
				return 1;
			}
		}
	}
	/* Search DTDs in block1 */
	if (edid->extension_edid != 0x00) {
		offset = edid->offset_dtd;
		if (offset != 0)
			addr = EDID_DESCRIPTOR_BLOCK1_ADDRESS + offset;
		for (i = 0; i < EDID_SIZE_BLOCK1_TIMING_DESCRIPTOR; i++) {
			get_eedid_timing_info(addr, (u8 *)edid, &hdmi.connection.edid_timings);
			addr += EDID_TIMING_DESCRIPTOR_SIZE;
			cm = hdmi_get_code(&hdmi.connection.edid_timings);
			DSSDBG("Block1[%d] value matches code = %d , mode = %d",
				i, cm.code, cm.mode);
			if (cm.code == -1)
				continue;
			if (hdmi.connection.s3d_enabled && s3d_code_cea[cm.code] == -1)
				continue;
			/* if sink supports audio, use CEA video timing */
			if (audio_support && !cm.mode)
				continue;
			hdmi.connection.code = cm.code;
			hdmi.connection.mode = cm.mode;
			DSSDBG("code = %d , mode = %d", hdmi.connection.code, hdmi.connection.mode);
			return 1;
		}
	}
	/*As last resort, check for best standard timing supported:*/
	if (edid->timing_1 & 0x01) {
		DSSDBG("800x600@60Hz\n");
		hdmi.connection.mode = 0;
		hdmi.connection.code = 9;
		return 1;
	}
	if (edid->timing_2 & 0x08) {
		DSSDBG("1024x768@60Hz\n");
		hdmi.connection.mode = 0;
		hdmi.connection.code = 16;
		return 1;
	}

	hdmi.connection.code = 4; /*setting default value of 640 480 VGA*/
	hdmi.connection.mode = 0;
	code = code_vesa[hdmi.connection.code];
	hdmi.connection.edid_timings = all_timings_direct[code];
	return 1;
}

bool is_hdmi_interlaced(void)
{
	bool interlace;
	mutex_lock(&hdmi.connection_lock);
	interlace = hdmi.connection.cfg.interlace;
	mutex_unlock(&hdmi.connection_lock);
	return interlace;
}
const struct omap_video_timings *hdmi_get_omap_timing(int ix)
{
	if (ix < 0 || ix >= ARRAY_SIZE(all_timings_direct))
		return NULL;
	return all_timings_direct + ix;
}

static enum hdmi_internal_status hdmi_get_internal_status(void)
{
	enum hdmi_internal_status ret;
	mutex_lock(&hdmi.status_lock);
	ret = hdmi.status.status;
	mutex_unlock(&hdmi.status_lock);
	return ret;
}

static void hdmi_last_frame_out_handler(u32 irq, void* data)
{
	if ( irq & HDMI_IRQ_VIDEO_FRAME_DONE )
	{
		struct completion *last_frame_out = (struct completion *)data;
		//unset irq. remove will be done in caller
		HDMI_WP_IRQ_unset(HDMI_IRQ_VIDEO_FRAME_DONE);
		complete(last_frame_out);
	}
}

static void hdmi_disable_hdmi(void)
{
	mutex_lock(&hdmi.status_lock);
	hdmi_stop_plug_with_status_lock(get_hdmi_device());
	mutex_unlock(&hdmi.status_lock);
	hdmi_check_status_and_transit();
}

static void hdmi_disable_hdmi_work_fn(struct work_struct *work)
{
	hdmi_disable_hdmi();
}

static struct work_struct hdmi_disable_work;

static void hdmi_start_hdmi_video_delay_func(struct work_struct *data)
{
	if ( !HDMI_WP_get_video_status() )
	{
		//VIDEO not enabled
		DSSDBG("Checking HDMI Video start condition. HDMI go(%d)\n", dispc_go_busy(OMAP_DSS_CHANNEL_DIGIT));
		if ( dispc_go_busy(OMAP_DSS_CHANNEL_DIGIT) )
		{
			DSSERR("TV Go Bit Set. can't start HDMI Video. goto unplug\n");
			dispc_stop(OMAP_DSS_CHANNEL_DIGIT);
			//directly call hdmi_disable_hdmi() cause dead lock
			//hdmi_disable_hdmi();

			//I don't know workqueue in workqueue has no problem.
			//when glancing workqueu code, there is no dead situation..
			//@todo need verify
			INIT_WORK(&hdmi_disable_work, hdmi_disable_hdmi_work_fn);
			schedule_work(&hdmi_disable_work);
			return;
		}
	}
	//I don't know which sequence is right
	dispc_enable_digit_out(1);
	HDMI_W1_StartVideoFrame(HDMI_WP);
//	dispc_enable_digit_out(1);
	DSSDBG("Start HDMI Video\n");
}

/**
 * Start HDMI Video
 * @param delay : start delay in milli second. 0-default delay 1/10 second
 */
static void hdmi_video_start_real_with_play_status_lock(int delay_m)
{
	hdmi.play_status.video_play_enabled = true;

	if ( hdmi.play_status.dirty)
	{
		//schedule play start when there is change in Video Producer list
		hdmi.play_status.dirty = false;
		//if pending start work, cancel & re-schedule
		cancel_delayed_work_sync(&hdmi.play_status.delayed_start_work);
		//schedule turn on work queue.
		//If there is another video producer, video stat will be stopped.
		//to preventing on-off-on (flicking in TV) I will use delayed workqueue
		if ( delay_m==0 )
			delay_m = 1000 / 10;
		schedule_delayed_work(&hdmi.play_status.delayed_start_work, HZ * delay_m / 1000);
		DSSDBG("HDMI Video will be Started in %d ms.\n", delay_m);
		if ( hdmi.status.status!=HDMI_STATUS_PLUG_ESTABLISHED )
			dump_stack();
	}
}



static void hdmi_video_stop_real_with_play_status_lock(void)
{
	//cancel play work
	cancel_delayed_work_sync(&hdmi.play_status.delayed_start_work);
	if ( HDMI_WP_get_video_status() )
	{
		//Video enable. wait last farme out
		struct completion last_frame_out;
		init_completion(&last_frame_out);

		DSSDBG("HDMI Video Enabled and Stop is requested\n");
//		if ( dispc_go_busy(OMAP_DSS_CHANNEL_DIGIT) )
		{
			int i, count=0;
			DSSDBG("TV GO Bit UnSet wait\n");
			for(i=0;i<120;i++)	//wait 2 second
			{
				if ( !dispc_go_busy(OMAP_DSS_CHANNEL_DIGIT) )
				{
					count++;
					if ( count > 5 )
						break;
				}
				else
					count = 0;
				mdelay(1000/60);	//wait vsync
			}
			if ( dispc_go_busy(OMAP_DSS_CHANNEL_DIGIT) || count <= 5 )
			{
				DSSERR("Wating TV Go bit unset failed. Anyway stop video\n");
			}
			else
				DSSDBG("TV go bit unset\n");
		}

		DSSDBG("Waiting last frame out to stop. HDMI GO(%d)\n", dispc_go_busy(OMAP_DSS_CHANNEL_DIGIT));
		hdmi_wp_irq_add_handler(hdmi_last_frame_out_handler, &last_frame_out);
		HDMI_WP_IRQ_set(HDMI_IRQ_VIDEO_FRAME_DONE);

		HDMI_W1_StopVideoFrame(HDMI_WP);
		wait_for_completion(&last_frame_out);
		hdmi_wp_irq_remove_handler(hdmi_last_frame_out_handler, &last_frame_out);
		DSSDBG("Last Frame out and stopped. HDMI GO(%d)\n", dispc_go_busy(OMAP_DSS_CHANNEL_DIGIT));
		dispc_enable_digit_out(0);

		if ( dispc_go_busy(OMAP_DSS_CHANNEL_DIGIT) )
		{
			DSSERR("Go bit stil set. force unset\n");
			dispc_stop(OMAP_DSS_CHANNEL_DIGIT);
		}
		hdmi.play_status.video_play_enabled = false;
		wake_up(&hdmi.play_status.video_producer_wq);
	}
	else
	{
		DSSDBG("HDMI Video is not on. skip stop hdmi\n");
	}
}

//find producer in list and return index
static int hdmi_video_find_by_id_with_play_status_lock(void *id)
{
	int i;
	if ( id==NULL )
		return -1;
	for(i=0;i<HDMI_VIDEO_PRODUCER_MAX;i++)
	{
		if ( hdmi.play_status.video_producers[i].id==id )
			return i;
	}
	return -1;
}

//add video producer and return added index
static int hdmi_video_add_id_with_play_status_lock(void *id, int layer_count)
{
	int i;
	if ( id==NULL )
		return -1;
	for(i=0;i<HDMI_VIDEO_PRODUCER_MAX;i++)
	{
		if ( hdmi.play_status.video_producers[i].id==NULL )
		{
			hdmi.play_status.video_producers[i].id = id;
			hdmi.play_status.video_producers[i].layer_count = layer_count;
			hdmi.play_status.video_producers[i].commit_status = HDMI_VIDEO_WILL_BE_ADDED;
			hdmi.play_status.video_producer_cnt++;
			DSSDBG("HDMI Video Producre(%p) added at %d. And now %d exist\n", id, i,
					hdmi.play_status.video_producer_cnt);
			return i;
		}
	}
	return -1;
}

static int hdmi_video_remove_id_with_play_status_lock(void *id)
{
	int i;
	if ( id==NULL )
		return -1;
	for(i=0;i<HDMI_VIDEO_PRODUCER_MAX;i++)
	{
		if ( hdmi.play_status.video_producers[i].id==id )
		{
			hdmi.play_status.video_producers[i].id = NULL;
			hdmi.play_status.video_producers[i].layer_count = 0;
			hdmi.play_status.video_producers[i].commit_status = HDMI_VIDEO_COMMITED;
			hdmi.play_status.video_producer_cnt--;
			DSSDBG("HDMI Video Produceer(%p) removed. remains : %d\n",
					id, hdmi.play_status.video_producer_cnt);
			if ( hdmi.play_status.video_producer_cnt==0 )
				wake_up(&hdmi.play_status.video_producer_wq);
			return i;
		}
	}
	return -1;
}



/**
 * set prepated status. if needed stop video player
 * @param id : must be unique
 * @param layer_count : layers used by id
 * @param add_or_change : true(add or change) false(remove)
 * @param reason : why change?
 * @return : 1(need commit) 0(no need commit)
 */
int hdmi_video_prepare_change(void *id, int layer_count, bool add_or_change, char *reason)
{
	bool needs_stop = false;
	bool needs_commit = false;
	int idx;
	if ( id==NULL )
		return 0;
	if ( reason==NULL )
		reason = "";

	if ( hdmi_get_internal_status()!=HDMI_STATUS_PLUG_ESTABLISHED )
	{
		//When PLUG status, video producer can be added or removed
		return 0;
	}

	mutex_lock(&hdmi.play_status_lock);

	//find
	idx = hdmi_video_find_by_id_with_play_status_lock(id);
	if ( idx==-1 )
	{
		//not exist
		if ( add_or_change )
		{
			idx = hdmi_video_add_id_with_play_status_lock(id, layer_count);
			if ( idx!= -1 )
			{
				DSSDBG("Adding Prepare(%s) HDMI Video Producer(%p) and stop HDMI video\n",
						reason, id);
				needs_stop = true;
				needs_commit = true;
			}
			else
			{
				DSSERR("No empty slot in HDMI Video List\n");
			}
		}
		else
		{
//			DSSDBG("Removing(%s) HDMI Video Producer(%p) and not in entry. no need to stop HDMI Video\n",
//					reason, id);
		}
	}
	else
	{
		//existing entry
		if ( add_or_change )
		{
			//change
			if ( hdmi.play_status.video_producers[idx].layer_count != layer_count )
			{
				DSSDBG("Changing(%s) HDMI Video Producer(%p)'s layer count to %d. stop HDMI Video\n",
						reason, id, layer_count);
				hdmi.play_status.video_producers[idx].commit_status = HDMI_VIDEO_WILL_BE_ADDED;
				hdmi.play_status.video_producers[idx].layer_count = layer_count;
				needs_stop = true;
				needs_commit = true;
			}
			else
			{
				if ( hdmi.play_status.video_producers[idx].commit_status != HDMI_VIDEO_COMMITED )
				{
					//not commit yet
					DSSDBG("Change(%s) HDMI Video Producer(%p) is same status but not commited. Stop HDMI Video\n",
							reason, id);
					needs_stop = true;
					needs_commit = true;
				}
				else
				{
					//already committed and same status. nothing to do
				}
			}
		}
		else
		{
			//remove
			DSSDBG("Removing Prepare(%s) HDMI Video Producer(%p)\n",
					reason, id);
			hdmi.play_status.video_producers[idx].commit_status = HDMI_VIDEO_WILL_BE_REMOVED;
			needs_stop = false;	//not stop when remove
			needs_commit = true;
		}
	}

//	if ( needs_stop && HDMI_WP_get_video_status() )
	if ( needs_stop && hdmi.play_status.video_play_enabled )
	{
		DSSDBG("Stop HDMI Video\n");
		hdmi_video_stop_real_with_play_status_lock();
	}
	if ( needs_commit )
		hdmi.play_status.dirty = true;
	mutex_unlock(&hdmi.play_status_lock);
	return (needs_commit ? 1 : 0);
}

/**
 * Check status and turn on/off video. This function will be call start/stop video fucntions.
 */
static int hdmi_video_check_n_action_with_plays_staus_lock(int start_delay_m)
{
	int i, cnt=0;
	bool not_commit_exist = false;

	if ( hdmi.play_status.video_producer_cnt  > 0 )
	{
		//check list
		for(i=0;i<HDMI_VIDEO_PRODUCER_MAX;i++)
			if ( hdmi.play_status.video_producers[i].id!=NULL )
			{
				if ( hdmi.play_status.video_producers[i].commit_status != HDMI_VIDEO_COMMITED )
					not_commit_exist = true;
				else
					cnt++;
			}
	}

	if ( cnt > 0 )	//all committed and there is at least 1 producer then turn on
	{
		if ( !not_commit_exist && HDMI_is_device_connected() )	//no prepare start play & connected
			hdmi_video_start_real_with_play_status_lock(start_delay_m);
	}
	else
	{
		//all stopped
		hdmi_video_stop_real_with_play_status_lock();
	}
	return cnt;
}

/**
 * Commit. Start Video if all video producesr are ready
 */
int hdmi_video_commit_change(void *id)
{
	int idx;
	int start_delay = 0;
	if ( id==NULL )
		return 0;
	if ( hdmi_get_internal_status()!=HDMI_STATUS_PLUG_ESTABLISHED )
	{
		//When PLUG status, video producer can be added or removed
		return 0;
	}

	mutex_lock(&hdmi.play_status_lock);
	idx = hdmi_video_find_by_id_with_play_status_lock(id);
	if ( idx!=-1 )
	{
		switch ( hdmi.play_status.video_producers[idx].commit_status )
		{
		case HDMI_VIDEO_COMMITED:
			break;
		case HDMI_VIDEO_WILL_BE_ADDED:
			hdmi.play_status.video_producers[idx].commit_status = HDMI_VIDEO_COMMITED;
			DSSINFO("HDMI Video Producer(%p) add committed\n", id);
			break;
		case HDMI_VIDEO_WILL_BE_REMOVED:
			hdmi_video_remove_id_with_play_status_lock(id);
			DSSINFO("HDMI Video Producer(%p) remove committed\n", id);
			start_delay = 500;	//to prevent S3D switching flicking
			break;
		default:
			DSSERR("Invalid HDMI Video Commit status :%d\n",
					hdmi.play_status.video_producers[idx].commit_status);
			break;
		}
	}
	hdmi_video_check_n_action_with_plays_staus_lock(start_delay);
	mutex_unlock(&hdmi.play_status_lock);
	return 0;
}

static void hdmi_video_stop_force_internal(void)
{
	int i;
	mutex_lock(&hdmi.play_status_lock);

	//clear list
	for(i=0;i<HDMI_VIDEO_PRODUCER_MAX;i++)
	{
		hdmi.play_status.video_producers[i].id = NULL;
		hdmi.play_status.video_producers[i].layer_count = 0;
		hdmi.play_status.video_producers[i].commit_status = HDMI_VIDEO_COMMITED;
	}

	DSSINFO("Force HDMI Video Stop\n");
	hdmi_video_stop_real_with_play_status_lock();
	hdmi.play_status.video_producer_cnt = 0;
	hdmi.play_status.video_play_enabled = false;
	wake_up(&hdmi.play_status.video_producer_wq);
	mutex_unlock(&hdmi.play_status_lock);
}

int hdmi_set_audio_power(bool _audio_on)
{
	if ( hdmi_get_internal_status()==HDMI_STATUS_PLUG_ESTABLISHED )
		return 0;
	return -EINVAL;
}

//get HPD
static int hdmi_get_hpd_status_with_status_lock(void)
{
	if ( hdmi.status.status==HDMI_STATUS_OFF || hdmi.status.status==HDMI_STATUS_UNDEFINED )
		return 0;
	else
		return HDMI_HPD_get_status();
}
