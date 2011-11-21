/*
 * HUB mipi interface support
 *
 * LGE_CHANGE [Kyungtae.oh@lge.com] 2010-04-01, 
 */

#include <linux/module.h>
#include <linux/delay.h>
#include <linux/platform_device.h>
//#include <linux/i2c/twl6030.h>
#include <linux/spi/spi.h>

#include <plat/mux.h>
#include <linux/err.h>
#include <linux/jiffies.h>
#include <linux/sched.h>
#include <linux/backlight.h>
#include <linux/fb.h>
#include <linux/interrupt.h>
#include <linux/gpio.h>
#include <linux/completion.h>
#include <linux/workqueue.h>

#include <plat/gpio.h>
#include <asm/mach-types.h>
#include <plat/control.h>

#include <plat/display.h>

#include <plat/heaven_lcd.h>

/* DSI Virtual channel. Hardcoded for now. */
#define TCH 0

#define DCS_READ_NUM_ERRORS		0x05
#define DCS_READ_POWER_MODE		0x0a
#define DCS_READ_MADCTL			0x0b
#define DCS_READ_PIXEL_FORMAT	0x0c
#define DCS_RDDSDR				0x0f
#define DCS_SLEEP_IN			0x10
#define DCS_SLEEP_OUT			0x11
#define DCS_DISPLAY_OFF			0x28
#define DCS_DISPLAY_ON			0x29
#define DCS_COLUMN_ADDR			0x2a
#define DCS_PAGE_ADDR			0x2b
#define DCS_MEMORY_WRITE		0x2c
#define DCS_TEAR_OFF			0x34
#define DCS_TEAR_ON				0x35
#define DCS_MEM_ACC_CTRL		0x36
#define DCS_PIXEL_FORMAT		0x3a
#define DCS_BRIGHTNESS			0x51
#define DCS_CTRL_DISPLAY		0x53
#define DCS_WRITE_CABC			0x55
#define DCS_READ_CABC			0x56
#define DCS_GET_ID				0xf8

/* #define HUB_PANEL_USE_ESD_CHECK */
#define HUB_PANEL_ESD_CHECK_PERIOD	msecs_to_jiffies(5000)

#define HUB_PANEL_LCD_RESET_N	30
#define HUB_PANEL_LCD_CS	7

#define LONG_CMD_MIPI	0
#define SHORT_CMD_MIPI	1
#define END_OF_COMMAND	2

static int lcd_boot_status=1;

#if defined(CONFIG_MACH_LGE_HUB_REV_A)
u8 mipi_tear_on[][30] = { 
	{LONG_CMD_MIPI,  0x39, 0x03, 0x44, 0x00, 0x00, }, // set tear scanline
	{SHORT_CMD_MIPI, 0x15, 0x02, 0x35, 0x00, }, // set tear on
	{END_OF_COMMAND, },
};

u8 lcd_command_for_mipi[][30] = { 
	{LONG_CMD_MIPI,  0x29, 0x14, 0xCF, 0x0F, 0x04, 0x00, 0x05, 0xFF, 0xFF, 0xFF, 0x10, 0xFF, 0xFF, 0x01, 0x01, 0x06, 0x06, 0x14, 0x0A, 0x19, 0x00, 0x00,},
	{LONG_CMD_MIPI,  0x29, 0x14, 0xCF, 0x0F, 0x04, 0x00, 0x05, 0xFF, 0xFF, 0xFF, 0x10, 0xFF, 0xFF, 0x01, 0x01, 0x06, 0x06, 0x14, 0x0A, 0x19, 0x00, 0x00,},
	{SHORT_CMD_MIPI, 0x23, 0x02, 0xCF, 0x30,}, //GATESET
	{LONG_CMD_MIPI,  0x39, 0x05, 0x30, 0x00, 0x00, 0x03, 0x1F,}, //set partial area
	{LONG_CMD_MIPI,  0x39, 0x07, 0x33, 0x00, 0x00, 0x03, 0x20, 0x00, 0x00,}, // set scroll area
	{SHORT_CMD_MIPI, 0x15, 0x02, 0x36, 0x0a,}, //set address mode
	{LONG_CMD_MIPI,  0x39, 0x03, 0x37, 0x00, 0x00,}, //set scroll start
	{SHORT_CMD_MIPI, 0x15, 0x02, 0x3a, 0x07,},	//set pixel format
	{SHORT_CMD_MIPI, 0x23, 0x02, 0x71, 0x00,},	//Ex Vsync en
	{SHORT_CMD_MIPI, 0x23, 0x02, 0xb2, 0x00,},	//VCSEL
	{SHORT_CMD_MIPI, 0x23, 0x02, 0xb4, 0xaa,},	//Set vgmpm
	{SHORT_CMD_MIPI, 0x23, 0x02, 0xb5, 0x33,},	//RBIAS1
	{SHORT_CMD_MIPI, 0x23, 0x02, 0xb6, 0x03,},	//RBIAS2
	{LONG_CMD_MIPI,  0x29, 0x11, 0xB7, 0x0c, 0x00, 0x03, 0x03, 0x03, 0x00, 0x00, 0x01, 0x02, 0x00, 0x00, 0x04, 0x00, 0x01, 0x01, 0x02,},	//Set ddvdhp
	{LONG_CMD_MIPI,  0x29, 0x0e, 0xB8, 0x0c, 0x00, 0x03, 0x03, 0x00, 0x01, 0x02, 0x00, 0x00, 0x04, 0x00, 0x01, 0x01, }, //set ddvdhm
	{LONG_CMD_MIPI,  0x29, 0x0B, 0xB9, 0x0C, 0x01, 0x00, 0x00, 0x00, 0x00, 0x02, 0x00, 0x02, 0x01, }, //set vgh
	{LONG_CMD_MIPI,  0x29, 0x0B, 0xBA, 0x07, 0x01, 0x00, 0x00, 0x00, 0x00, 0x01, 0x00, 0x02, 0x01, }, //set vgl
	{LONG_CMD_MIPI,  0x29, 0x08, 0xBB, 0x00, 0x00, 0x00, 0x00, 0x01, 0x02, 0x01, },	//set vcl
	{SHORT_CMD_MIPI, 0x23, 0x02, 0xc1, 0x01, }, // number of lines
	{SHORT_CMD_MIPI, 0x23, 0x02, 0xc2, 0x08, }, // number of FP Lines
	{SHORT_CMD_MIPI, 0x23, 0x02, 0xc3, 0x04, }, // gate set (1)
	{SHORT_CMD_MIPI, 0x23, 0x02, 0xc4, 0x4c, }, // 1h period
	{SHORT_CMD_MIPI, 0x23, 0x02, 0xc5, 0x03, }, // Source Precharge
	{LONG_CMD_MIPI,  0x29, 0x03, 0xC6, 0x04, 0x04, }, //Source Precharge timing
	{SHORT_CMD_MIPI, 0x23, 0x02, 0xc7, 0x00, }, // Source level
	{SHORT_CMD_MIPI, 0x23, 0x02, 0xc8, 0x02, }, // Number of BP Lines
	{SHORT_CMD_MIPI, 0x23, 0x02, 0xc9, 0x10, }, // gateset 2
	{LONG_CMD_MIPI,  0x29, 0x03, 0xCA, 0x04, 0x02, }, // gateset 3
	{SHORT_CMD_MIPI, 0x23, 0x02, 0xcb, 0x03, }, //gate 4
	{SHORT_CMD_MIPI, 0x23, 0x02, 0xcc, 0x12, }, //gate 5
	{SHORT_CMD_MIPI, 0x23, 0x02, 0xcd, 0x12, }, //gate 6
	{SHORT_CMD_MIPI, 0x23, 0x02, 0xce, 0x30, }, //gate 7
	{SHORT_CMD_MIPI, 0x23, 0x02, 0xd0, 0x40, }, //gate 9
	{SHORT_CMD_MIPI, 0x23, 0x02, 0xd1, 0x22, }, //FLHW
	{SHORT_CMD_MIPI, 0x23, 0x02, 0xd2, 0x22, }, //VCKHW
	{SHORT_CMD_MIPI, 0x23, 0x02, 0xd3, 0x04, }, //FLT
	{SHORT_CMD_MIPI, 0x23, 0x02, 0xd4, 0x14, }, //TCTRL
	{SHORT_CMD_MIPI, 0x23, 0x02, 0xd6, 0x01, }, //DOTINV
	{SHORT_CMD_MIPI, 0x23, 0x02, 0xd7, 0x00, }, //On,Off sequence period
	{LONG_CMD_MIPI,  0x29, 0x0A, 0xD8, 0x01, 0x05, 0x06, 0x0A, 0x18, 0x0E, 0x22, 0x23, 0x00, }, //PONSEQA
	{LONG_CMD_MIPI,  0x29, 0x03, 0xD9, 0x24, 0x01, }, //PONSEQB
	{LONG_CMD_MIPI,  0x29, 0x06, 0xDE, 0x09, 0x0F, 0x21, 0x17, 0x04, }, //PONSEQC
	{LONG_CMD_MIPI,  0x29, 0x07, 0xDF, 0x02, 0x06, 0x06, 0x06, 0x06, 0x00, }, //POFSEQA
	{SHORT_CMD_MIPI, 0x23, 0x02, 0xe0, 0x01, }, //POFSEQB
	{LONG_CMD_MIPI,  0x29, 0x06, 0xE1, 0x00, 0x00, 0x00, 0x00, 0x00, }, //POFSEQC
	{SHORT_CMD_MIPI, 0x23, 0x02, 0x51, 0xff, }, // Manual Brightness
	{SHORT_CMD_MIPI, 0x23, 0x02, 0x52, 0x00, }, // minimum brightness
	{SHORT_CMD_MIPI, 0x23, 0x02, 0x53, 0x40, }, // Backlight Control
	{LONG_CMD_MIPI,  0x29, 0x03, 0xE2, 0x00, 0x00, }, // Cabc PWM
	{SHORT_CMD_MIPI, 0x23, 0x02, 0xe3, 0x03, }, //CABC
	{LONG_CMD_MIPI,  0x29, 0x09, 0xE4, 0x66, 0x7B, 0x90, 0xA5, 0xBB, 0xC7, 0xE1, 0xE5, }, // CABC brightness
	{LONG_CMD_MIPI,  0x29, 0x09, 0xE5, 0xC5, 0xC5, 0xC9, 0xC9, 0xD1, 0xE1, 0xF1, 0xFE, }, // CABC brightness
	{SHORT_CMD_MIPI, 0x23, 0x02, 0xe7, 0x2a, }, //CABC
	{SHORT_CMD_MIPI, 0x23, 0x02, 0xe8, 0x00, }, //BRT_REV
	{SHORT_CMD_MIPI, 0x23, 0x02, 0xe9, 0x00, }, //TE
	{SHORT_CMD_MIPI, 0x23, 0x02, 0xea, 0x01, }, //High Speed RAM
	{LONG_CMD_MIPI,  0x29, 0x09, 0xEB, 0x00, 0x34, 0x12, 0x10, 0x99, 0x88, 0x87, 0x0d, }, // Gamma setting R (pos)
	{LONG_CMD_MIPI,  0x29, 0x09, 0xEC, 0x00, 0x34, 0x12, 0x10, 0x99, 0x88, 0x87, 0x0d, }, // Gamma setting R (neg)
	{LONG_CMD_MIPI,  0x29, 0x09, 0xED, 0x00, 0x34, 0x12, 0x10, 0x99, 0x88, 0x87, 0x0d, }, // Gamma setting G (pos)
	{LONG_CMD_MIPI,  0x29, 0x09, 0xEE, 0x00, 0x34, 0x12, 0x10, 0x99, 0x88, 0x87, 0x0d, }, // Gamma setting G (neg)
	{LONG_CMD_MIPI,  0x29, 0x09, 0xEF, 0x00, 0x34, 0x12, 0x10, 0x99, 0x88, 0x87, 0x0d, }, // Gamma setting B (pos)
	{LONG_CMD_MIPI,  0x29, 0x09, 0xF0, 0x00, 0x34, 0x12, 0x10, 0x99, 0x88, 0x87, 0x0d, }, // Gamma setting B (neg)
	{END_OF_COMMAND, }, 
};
#else
u8 lcd_command_for_mipi[][30] = {
    {LONG_CMD_MIPI, 0x29,0x03,0xF1,0x5A,0x5A,}, //level2 command control test key2
    {LONG_CMD_MIPI, 0x29,0x04,0xB7,0x00,0x11,0x11,},    //clock control 3
    {LONG_CMD_MIPI, 0x29,0x03,0xB8,0x2C,0x01,},     //Resolution & interface select control
    {LONG_CMD_MIPI, 0x29,0x03,0xB9,0x00,0x06,},     //Panel case & DSI pad control
    {SHORT_CMD_MIPI, 0x15,0x02,0x35,0x00,},         //TE On
    {LONG_CMD_MIPI, 0x29,0x03,0xF1,0xA5,0xA5,},     //Level2 Command control test key2
    {LONG_CMD_MIPI, 0x29,0x05,0x2A,0x00,0x00,0x01,0xDF,},       //Column Address Set
    {LONG_CMD_MIPI, 0x29,0x05,0x2B,0x00,0x00,0x03,0x1F,},       //Column Address Set
    {SHORT_CMD_MIPI, 0x23,0x02,0x51,0x00,},     //Set manual brightness
    {SHORT_CMD_MIPI, 0x23,0x02,0x53,0x24,},     //Turn on back light control
    {SHORT_CMD_MIPI, 0x23,0x02,0x55,0x00,},     //Write MIE Mode
    {END_OF_COMMAND, },
};
#endif
struct hub_panel_data {
	struct backlight_device *bldev;

	unsigned long	hw_guard_end;	/* next value of jiffies when we can
					 * issue the next sleep in/out command
					 */
	unsigned long	hw_guard_wait;	/* max guard time in jiffies */

	struct omap_dss_device *dssdev;

	bool enabled;
	u8 rotate;
	bool mirror;

	bool te_enabled;
	bool use_ext_te;
	struct completion te_completion;

	bool use_dsi_bl;

	bool intro_printed;

	struct workqueue_struct *esd_wq;
	struct delayed_work esd_work;
};


int dsi_vc_send_bta_sync(enum dsi lcd_ix, int channel);
int dsi_vc_generic_write_short(int channel, u8 cmd, u8 *data, int len);
int dsi_vc_generic_write(int channel, u8 cmd, u8 *data, int len);

static void hub_panel_esd_work(struct work_struct *work);

static void hw_guard_start(struct hub_panel_data *td, int guard_msec)
{
	td->hw_guard_wait = msecs_to_jiffies(guard_msec);
	td->hw_guard_end = jiffies + td->hw_guard_wait;
}

static void hw_guard_wait(struct hub_panel_data *td)
{
	unsigned long wait = td->hw_guard_end - jiffies;

	if ((long)wait > 0 && wait <= td->hw_guard_wait) {
		set_current_state(TASK_UNINTERRUPTIBLE);
		schedule_timeout(wait);
	}
}

static int hub_panel_dcs_read_1(u8 dcs_cmd, u8 *data)
{
	int r;
	u8 buf[1];

	r = dsi_vc_dcs_read(1,TCH, dcs_cmd, buf, 1);

	if (r < 0)
		return r;

	*data = buf[0];

	return 0;
}

static int hub_panel_dcs_write_0(u8 dcs_cmd)
{
	return dsi_vc_dcs_write(1,TCH, &dcs_cmd, 1);
}

static int hub_panel_dcs_write_1(u8 dcs_cmd, u8 param)
{
	u8 buf[2];
	buf[0] = dcs_cmd;
	buf[1] = param;
	return dsi_vc_dcs_write(1,TCH, buf, 2);
}

static int hub_panel_sleep_in(struct hub_panel_data *td)

{
	u8 cmd;
	int r;

	hw_guard_wait(td);

	cmd = DCS_SLEEP_IN;
	r = dsi_vc_dcs_write_nosync(1,TCH, &cmd, 1);
	if (r)
		return r;

	hw_guard_start(td, 120);

	msleep(5);

	return 0;
}

static int hub_panel_sleep_out(struct hub_panel_data *td)
{
	int r;

	hw_guard_wait(td);

	r = hub_panel_dcs_write_0(DCS_SLEEP_OUT);
	if (r)
		return r;

	hw_guard_start(td, 120);

	msleep(5);

	return 0;
}

static int hub_panel_get_id(u8 *buf)
{
	int r;

	r = dsi_vc_dcs_read(1,TCH, DCS_GET_ID, buf, 3);
	if (r)
		return r;

	return 0;
}

static int hub_panel_set_addr_mode(u8 rotate, bool mirror)
{
	int r;
	u8 mode;
	int b5, b6, b7;

	r = hub_panel_dcs_read_1(DCS_READ_MADCTL, &mode);
	if (r)
		return r;

	switch (rotate) {
	default:
	case 0:
		b7 = 0;
		b6 = 0;
		b5 = 0;
		break;
	case 1:
		b7 = 0;
		b6 = 1;
		b5 = 1;
		break;
	case 2:
		b7 = 1;
		b6 = 1;
		b5 = 0;
		break;
	case 3:
		b7 = 1;
		b6 = 0;
		b5 = 1;
		break;
	}

	if (mirror)
		b6 = !b6;

	mode &= ~((1<<7) | (1<<6) | (1<<5));
	mode |= (b7 << 7) | (b6 << 6) | (b5 << 5);

	return hub_panel_dcs_write_1(DCS_MEM_ACC_CTRL, mode);
}

static int hub_panel_set_update_window(u16 x, u16 y, u16 w, u16 h)
{
	int r;
	u16 x1 = x;
	u16 x2 = x + w - 1;
	u16 y1 = y;
	u16 y2 = y + h - 1;

	u8 buf[5];
	buf[0] = DCS_COLUMN_ADDR;
	buf[1] = (x1 >> 8) & 0xff;
	buf[2] = (x1 >> 0) & 0xff;
	buf[3] = (x2 >> 8) & 0xff;
	buf[4] = (x2 >> 0) & 0xff;

	r = dsi_vc_dcs_write_nosync(1,TCH, buf, sizeof(buf));
	if (r)
		return r;

	buf[0] = DCS_PAGE_ADDR;
	buf[1] = (y1 >> 8) & 0xff;
	buf[2] = (y1 >> 0) & 0xff;
	buf[3] = (y2 >> 8) & 0xff;
	buf[4] = (y2 >> 0) & 0xff;

	r = dsi_vc_dcs_write_nosync(1,TCH, buf, sizeof(buf));
	if (r)
		return r;
	dsi_vc_send_bta_sync(1,TCH);
	return r;
}

static int hub_panel_bl_update_status(struct backlight_device *dev)
{
	struct omap_dss_device *dssdev = dev_get_drvdata(&dev->dev);
	struct hub_panel_data *td = dev_get_drvdata(&dssdev->dev);
	int r;
	int level;

	if (dev->props.fb_blank == FB_BLANK_UNBLANK &&
			dev->props.power == FB_BLANK_UNBLANK)
		level = dev->props.brightness;
	else
		level = 0;

	dev_dbg(&dssdev->dev, "update brightness to %d\n", level);

	if (td->use_dsi_bl) {
		if (td->enabled) {
			dsi_bus_lock(1);
			r = hub_panel_dcs_write_1(DCS_BRIGHTNESS, level);
			dsi_bus_unlock(1);
			if (r)
				return r;
		}
	} else {
		if (!dssdev->set_backlight)
			return -EINVAL;

		r = dssdev->set_backlight(dssdev, level);
		if (r)
			return r;
	}

	return 0;
}

static int hub_panel_bl_get_intensity(struct backlight_device *dev)
{
	if (dev->props.fb_blank == FB_BLANK_UNBLANK &&
			dev->props.power == FB_BLANK_UNBLANK)
		return dev->props.brightness;

	return 0;
}

static struct backlight_ops hub_panel_bl_ops = {
	.get_brightness = hub_panel_bl_get_intensity,
	.update_status  = hub_panel_bl_update_status,
};

static void hub_panel_get_timings(struct omap_dss_device *dssdev,
			struct omap_video_timings *timings)
{
	*timings = dssdev->panel.timings;
}

static void hub_panel_get_resolution(struct omap_dss_device *dssdev,
		u16 *xres, u16 *yres)
{
	struct hub_panel_data *td = dev_get_drvdata(&dssdev->dev);

	if (td->rotate == 0 || td->rotate == 2) {
		*xres = dssdev->panel.timings.x_res;
		*yres = dssdev->panel.timings.y_res;
	} else {
		*yres = dssdev->panel.timings.x_res;
		*xres = dssdev->panel.timings.y_res;
	}
}

static irqreturn_t hub_panel_te_isr(int irq, void *data)
{
	struct omap_dss_device *dssdev = data;
	struct hub_panel_data *td = dev_get_drvdata(&dssdev->dev);

//	printk("[JamesLee] hub_panel_te_isr() !!!\n");
	complete_all(&td->te_completion);

	return IRQ_HANDLED;
}

static ssize_t hub_panel_num_errors_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct omap_dss_device *dssdev = to_dss_device(dev);
	struct hub_panel_data *td = dev_get_drvdata(&dssdev->dev);
	u8 errors;
	int r;

	if (td->enabled) {
		dsi_bus_lock(1);
		r = hub_panel_dcs_read_1(DCS_READ_NUM_ERRORS, &errors);
		dsi_bus_unlock(1);
	} else {
		r = -ENODEV;
	}

	if (r)
		return r;

	return snprintf(buf, PAGE_SIZE, "%d\n", errors);
}

static ssize_t hub_panel_hw_revision_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct omap_dss_device *dssdev = to_dss_device(dev);
	struct hub_panel_data *td = dev_get_drvdata(&dssdev->dev);
	u8 dbuf[3];
	int r;

	if (td->enabled) {
		dsi_bus_lock(1);
		r = hub_panel_get_id(dbuf);
		dsi_bus_unlock(1);
	} else {
		r = -ENODEV;
	}

	if (r)
		return r;

	return snprintf(buf, PAGE_SIZE, "%02x.%02x.%02x\n", dbuf[0], dbuf[1], dbuf[2]);
}

static DEVICE_ATTR(num_dsi_errors, S_IRUGO, hub_panel_num_errors_show, NULL);
static DEVICE_ATTR(hw_revision, S_IRUGO, hub_panel_hw_revision_show, NULL);


void hub_panel_init_lcd(void)
{
	if(gpio_request(HUB_PANEL_LCD_RESET_N, "lcd gpio") < 0) {
		return;
	}
	gpio_direction_output(HUB_PANEL_LCD_RESET_N, 1);
    gpio_set_value(HUB_PANEL_LCD_RESET_N,0);

	if(gpio_request(HUB_PANEL_LCD_CS, "lcd cs") < 0) {
		return;
	}
	gpio_direction_output(HUB_PANEL_LCD_CS, 1);
}

void hub_panel_reset_lcd(void)
{
	gpio_set_value(HUB_PANEL_LCD_RESET_N, 0);
	mdelay(1);
	gpio_set_value(HUB_PANEL_LCD_RESET_N, 1);
	mdelay(1);
}

static int hub_panel_enable(struct omap_dss_device *dssdev)
{
	struct hub_panel_data *td = dev_get_drvdata(&dssdev->dev);
	int i, r;

	dev_dbg(&dssdev->dev, "enable\n");

	if (dssdev->platform_enable) {
		r = dssdev->platform_enable(dssdev);
		if (r)
			return r;
	}
	if (lcd_boot_status== 1)
		lcd_boot_status = 0;
	else
		hub_panel_reset_lcd();

//	printk("XXXXXXXXXXXhub_panel_enableXXXXXXXXXXXX");

	/* it seems we have to wait a bit until hub_panel is ready */
	mdelay(5);

	// initialize device
	for (i = 0; lcd_command_for_mipi[i][0] != END_OF_COMMAND; i++) {
		if(lcd_command_for_mipi[i][0] == SHORT_CMD_MIPI) {
			dsi_vc_generic_write_short(TCH, lcd_command_for_mipi[i][1], &lcd_command_for_mipi[i][3], lcd_command_for_mipi[i][2]);
		} else if(lcd_command_for_mipi[i][0] == LONG_CMD_MIPI) {

		//	printk("YYYYYYYYYYYYYYY inside for loopYYYYYYY");
		
			dsi_vc_generic_write(TCH, lcd_command_for_mipi[i][1], &lcd_command_for_mipi[i][3], lcd_command_for_mipi[i][2]);
		}
	}

	r = hub_panel_sleep_out(td);
	if (r)
		goto err;

	hub_panel_dcs_write_1(DCS_CTRL_DISPLAY, (1<<2) | (1<<5)); /* BL | BCTRL */
	hub_panel_dcs_write_1(DCS_PIXEL_FORMAT, 0x7); /* 24bit/pixel */
	hub_panel_set_addr_mode(td->rotate, td->mirror);

	hub_panel_dcs_write_0(DCS_DISPLAY_ON);

	printk("DISPLAY ON & start tearing ON\n");
	mdelay(30); //after 1 frame

#ifdef HUB_PANEL_USE_ESD_CHECK
	queue_delayed_work(td->esd_wq, &td->esd_work, HUB_PANEL_ESD_CHECK_PERIOD);
#endif

	td->enabled = 1;

	return 0;
err:
	if (dssdev->platform_disable)
		dssdev->platform_disable(dssdev);

	return r;
}



static struct attribute *hub_panel_attrs[] = {
	&dev_attr_num_dsi_errors.attr,
	&dev_attr_hw_revision.attr,
	NULL,
};

static struct attribute_group hub_panel_attr_group = {
	.attrs = hub_panel_attrs,
};

#if 1		/* JamesLee */
static void pad_config(unsigned long pad_addr, u32 andmask, u32 ormask)
{
	int val;
	u32 *addr;

	addr = (u32 *) ioremap(pad_addr, 4);
	if (!addr) {
		printk(KERN_ERR"OMAP_pad_config: ioremap failed with addr %lx\n",
			pad_addr);
		return;
	}

	val =  __raw_readl(addr);
	val &= andmask;
	val |= ormask;
	__raw_writel(val, addr);

	iounmap(addr);
}
#endif
static int hub_panel_probe(struct omap_dss_device *dssdev)
{
	struct hub_panel_data *td;
	struct backlight_device *bldev;
	int r;

	dev_dbg(&dssdev->dev, "probe\n");

	td = kzalloc(sizeof(*td), GFP_KERNEL);
	if (!td) {
		r = -ENOMEM;
		goto err0;
	}
	td->dssdev = dssdev;

	td->esd_wq = create_singlethread_workqueue("hub_panel_esd");
	if (td->esd_wq == NULL) {
		dev_err(&dssdev->dev, "can't create ESD workqueue\n");
		r = -ENOMEM;
		goto err2;
	}
	INIT_DELAYED_WORK_DEFERRABLE(&td->esd_work, hub_panel_esd_work);

	dev_set_drvdata(&dssdev->dev, td);

	dssdev->get_timings = hub_panel_get_timings;
	dssdev->get_resolution = hub_panel_get_resolution;

	/* if no platform set_backlight() defined, presume DSI backlight
	 * control */
	if (!dssdev->set_backlight)
		td->use_dsi_bl = true;

	bldev = backlight_device_register("hub_panel", &dssdev->dev, dssdev,
			&hub_panel_bl_ops);
	if (IS_ERR(bldev)) {
		r = PTR_ERR(bldev);
		goto err1;
	}

	td->bldev = bldev;

	bldev->props.fb_blank = FB_BLANK_UNBLANK;
	bldev->props.power = FB_BLANK_UNBLANK;
	if (td->use_dsi_bl) {
		bldev->props.max_brightness = 255;
		bldev->props.brightness = 255;
	} else {
		bldev->props.max_brightness = 127;
		bldev->props.brightness = 127;
	}

	hub_panel_bl_update_status(bldev);

	if (dssdev->phy.dsi.ext_te) {
		int gpio = dssdev->phy.dsi.ext_te_gpio;
		

		r = gpio_request(gpio, "hub_panel irq");
	
		printk("GPIO_hub_panel_irq=%x",gpio);

		if (r) {
			dev_err(&dssdev->dev, "GPIO request failed\n");
			goto err3;
		}

#if 0 // LGE DEV4 CH.PARK@LGE.COM DO NOT ATTEMPT TO CHANGE GPIO CONFIG IN DRV		/* JamesLee */
		pad_config(0x4A1001CC, 0xFEF8FFFF, 0x01030000);
#endif
		gpio_direction_input(gpio);

		//r = request_irq(gpio_to_irq(gpio), hub_panel_te_isr,
		r = request_irq(gpio_to_irq(gpio), hub_panel_te_isr,
				//IRQF_DISABLED | IRQF_TRIGGER_RISING,
				IRQF_TRIGGER_RISING,
				"hub_panel vsync", dssdev);

		if (r) {
			dev_err(&dssdev->dev, "IRQ request failed\n");
			gpio_free(gpio);
			goto err3;
		}

		init_completion(&td->te_completion);

		td->use_ext_te = true;
	}

	r = sysfs_create_group(&dssdev->dev.kobj, &hub_panel_attr_group);
	if (r) {
		dev_err(&dssdev->dev, "failed to create sysfs files\n");
		goto err4;
	}

	return 0;
err4:
	if (td->use_ext_te) {
		int gpio = dssdev->phy.dsi.ext_te_gpio;
		free_irq(gpio_to_irq(gpio), dssdev);
		gpio_free(gpio);
	}
err3:
	backlight_device_unregister(bldev);
err2:
	cancel_delayed_work_sync(&td->esd_work);
	destroy_workqueue(td->esd_wq);
err1:
	kfree(td);
err0:
	return r;
}

static void hub_panel_remove(struct omap_dss_device *dssdev)
{
	struct hub_panel_data *td = dev_get_drvdata(&dssdev->dev);
	struct backlight_device *bldev;

	dev_dbg(&dssdev->dev, "remove\n");

	sysfs_remove_group(&dssdev->dev.kobj, &hub_panel_attr_group);

	if (td->use_ext_te) {
		int gpio = dssdev->phy.dsi.ext_te_gpio;
		free_irq(gpio_to_irq(gpio), dssdev);
		gpio_free(gpio);
	}

	bldev = td->bldev;
	bldev->props.power = FB_BLANK_POWERDOWN;
	hub_panel_bl_update_status(bldev);
	backlight_device_unregister(bldev);

	cancel_delayed_work_sync(&td->esd_work);
	destroy_workqueue(td->esd_wq);

	kfree(td);
}

static void hub_panel_disable(struct omap_dss_device *dssdev)
{
	struct hub_panel_data *td = dev_get_drvdata(&dssdev->dev);

	dev_dbg(&dssdev->dev, "disable\n");

	cancel_delayed_work(&td->esd_work);

	hub_panel_dcs_write_0(DCS_DISPLAY_OFF);
	hub_panel_sleep_in(td);

	/* wait a bit so that the message goes through */
	msleep(10);

	if (dssdev->platform_disable)
		dssdev->platform_disable(dssdev);
	
	td->enabled = 0;
}

static int hub_panel_suspend(struct omap_dss_device *dssdev)
{
	struct hub_panel_data *td = dev_get_drvdata(&dssdev->dev);
	struct backlight_device *bldev = td->bldev;

	bldev->props.power = FB_BLANK_POWERDOWN;
	hub_panel_bl_update_status(bldev);

    hub_panel_dcs_write_0(DCS_DISPLAY_OFF);

    hub_panel_dcs_write_0(DCS_SLEEP_IN);

	return 0;
}

static int hub_panel_resume(struct omap_dss_device *dssdev)
{
	struct hub_panel_data *td = dev_get_drvdata(&dssdev->dev);
	struct backlight_device *bldev = td->bldev;

	bldev->props.power = FB_BLANK_UNBLANK;
	hub_panel_bl_update_status(bldev);
	
    hub_panel_dcs_write_0(DCS_SLEEP_OUT);

	return 0;
}

static void hub_panel_setup_update(struct omap_dss_device *dssdev,
				    u16 x, u16 y, u16 w, u16 h)
{
	hub_panel_set_update_window(x, y, w, h);
}

static int hub_panel_enable_te(struct omap_dss_device *dssdev, bool enable)
{
	struct hub_panel_data *td = dev_get_drvdata(&dssdev->dev);
	int r;

	td->te_enabled = enable;


//	printk("SELWIN>>>>>>>=%x",enable);

	if (enable)
		r = hub_panel_dcs_write_1(DCS_TEAR_ON, 0);
	else
		r = hub_panel_dcs_write_0(DCS_TEAR_OFF);
	return r;
}

static int hub_panel_wait_te(struct omap_dss_device *dssdev)
{
	struct hub_panel_data *td = dev_get_drvdata(&dssdev->dev);
	long wait = msecs_to_jiffies(10);	// LGE DEV4 CH.PARK@LGE.COM 20101023 H/W ERROR. TEMP 10

	if (!td->use_ext_te || !td->te_enabled)
		return 0;

	INIT_COMPLETION(td->te_completion);
	wait = wait_for_completion_timeout(&td->te_completion, wait);
	if (wait == 0) {
		dev_err(&dssdev->dev, "timeout waiting TE\n");
		return -ETIME;
	}

	return 0;
}

static int hub_panel_rotate(struct omap_dss_device *dssdev, u8 rotate)
{
	struct hub_panel_data *td = dev_get_drvdata(&dssdev->dev);
	int r;

	dev_dbg(&dssdev->dev, "rotate %d\n", rotate);

	if (td->enabled) {
		r = hub_panel_set_addr_mode(rotate, td->mirror);

		if (r)
			return r;
	}

	td->rotate = rotate;

	return 0;
}

static u8 hub_panel_get_rotate(struct omap_dss_device *dssdev)
{
	struct hub_panel_data *td = dev_get_drvdata(&dssdev->dev);
	return td->rotate;
}

static int hub_panel_mirror(struct omap_dss_device *dssdev, bool enable)
{
	struct hub_panel_data *td = dev_get_drvdata(&dssdev->dev);
	int r;

	dev_dbg(&dssdev->dev, "mirror %d\n", enable);

	if (td->enabled) {
		r = hub_panel_set_addr_mode(td->rotate, enable);

		if (r)
			return r;
	}

	td->mirror = enable;

	return 0;
}

static bool hub_panel_get_mirror(struct omap_dss_device *dssdev)
{
	struct hub_panel_data *td = dev_get_drvdata(&dssdev->dev);
	return td->mirror;
}

static int hub_panel_run_test(struct omap_dss_device *dssdev, int test_num)
{
	u8 buf[3];
	int r;

	r = dsi_vc_dcs_read(1,TCH, DCS_GET_ID, buf, 3);
	if (r)
		return r;

	return 0;
}

static int hub_panel_memory_read(struct omap_dss_device *dssdev,
		void *buf, size_t size,
		u16 x, u16 y, u16 w, u16 h)
{
	int r;
	int first = 1;
	int plen;
	unsigned buf_used = 0;

	if (size < w * h * 3)
		return -ENOMEM;

	size = min(w * h * 3,
			dssdev->panel.timings.x_res *
			dssdev->panel.timings.y_res * 3);

	/* plen 1 or 2 goes into short packet. until checksum error is fixed,
	 * use short packets. plen 32 works, but bigger packets seem to cause
	 * an error. */
	if (size % 2)
		plen = 1;
	else
		plen = 2;

	hub_panel_setup_update(dssdev, x, y, w, h);

	r = dsi_vc_set_max_rx_packet_size(1,TCH, plen);
	if (r)
		return r;

	while (buf_used < size) {
		u8 dcs_cmd = first ? 0x2e : 0x3e;
		first = 0;

		r = dsi_vc_dcs_read(1,TCH, dcs_cmd,
				buf + buf_used, size - buf_used);

		if (r < 0) {
			dev_err(&dssdev->dev, "read error\n");
			goto err;
		}

		buf_used += r;

		if (r < plen) {
			dev_err(&dssdev->dev, "short read\n");
			break;
		}

		if (signal_pending(current)) {
			dev_err(&dssdev->dev, "signal pending, "
					"aborting memory read\n");
			r = -ERESTARTSYS;
			goto err;
		}
	}

	r = buf_used;

err:
	dsi_vc_set_max_rx_packet_size(1,TCH, 1);

	return r;
}

static void hub_panel_esd_work(struct work_struct *work)
{
	struct hub_panel_data *td = container_of(work, struct hub_panel_data,
			esd_work.work);
	struct omap_dss_device *dssdev = td->dssdev;
	u8 state1, state2;
	int r;

	if (!td->enabled)
		return;

	dsi_bus_lock(1);

	r = hub_panel_dcs_read_1(DCS_RDDSDR, &state1);
	if (r) {
		dev_err(&dssdev->dev, "failed to read Taal status\n");
		goto err;
	}

	/* Run self diagnostics */
	r = hub_panel_sleep_out(td);
	if (r) {
		dev_err(&dssdev->dev, "failed to run Taal self-diagnostics\n");
		goto err;
	}

	r = hub_panel_dcs_read_1(DCS_RDDSDR, &state2);
	if (r) {
		dev_err(&dssdev->dev, "failed to read Taal status\n");
		goto err;
	}

	/* Each sleep out command will trigger a self diagnostic and flip
	 * Bit6 if the test passes.
	 */
	if (!((state1 ^ state2) & (1 << 6))) {
		dev_err(&dssdev->dev, "LCD self diagnostics failed\n");
		goto err;
	}
	/* Self-diagnostics result is also shown on TE GPIO line. We need
	 * to re-enable TE after self diagnostics */
	if (td->use_ext_te && td->te_enabled)
		hub_panel_enable_te(dssdev, true);

	dsi_bus_unlock(1);

	queue_delayed_work(td->esd_wq, &td->esd_work, HUB_PANEL_ESD_CHECK_PERIOD);

	return;
err:
	dev_err(&dssdev->dev, "performing LCD reset\n");

	hub_panel_disable(dssdev);
	hub_panel_enable(dssdev);

	dsi_bus_unlock(1);

	queue_delayed_work(td->esd_wq, &td->esd_work, HUB_PANEL_ESD_CHECK_PERIOD);
}

// LGE DEV4 CH.PARK@LGE.COM 20101023 HUB_LCD_COMPILE_ERROR
int barrier_init_hotkey(bool enable)
{
	// dummy
}
EXPORT_SYMBOL(barrier_init_hotkey);

static struct omap_dss_driver hub_panel_driver = {
	.probe		= hub_panel_probe,
	.remove		= hub_panel_remove,

	.enable		= hub_panel_enable,
	.disable	= hub_panel_disable,
	.suspend	= hub_panel_suspend,
	.resume		= hub_panel_resume,

	.setup_update	= hub_panel_setup_update,
	.enable_te	= hub_panel_enable_te,
	.wait_for_te	= hub_panel_wait_te,
	.set_rotate	= hub_panel_rotate,
	.get_rotate	= hub_panel_get_rotate,
	.set_mirror	= hub_panel_mirror,
	.get_mirror	= hub_panel_get_mirror,
	.run_test	= hub_panel_run_test,
	.memory_read	= hub_panel_memory_read,

	.driver         = {
#if defined(CONFIG_MACH_LGE_HUB_REV_A)
		.name   = "russo_panel",
#else
		.name 	= "cosmo_hub_panel",	// LGE DEV4 CH.PARK@LGE.COM 20101023 TO USE HUB PANEL IN COSMO
#endif
		.owner  = THIS_MODULE,
	},
};

static int __init hub_panel_init(void)
{
	printk("Trying to register driver\n");
	omap_dss_register_driver(&hub_panel_driver);
	printk("Registering driver done\n");

	return 0;
}

static void __exit hub_panel_exit(void)
{
	omap_dss_unregister_driver(&hub_panel_driver);
}

module_init(hub_panel_init);
module_exit(hub_panel_exit);

MODULE_AUTHOR("kyungtae Oh <kyungtae.oh@lge.com>");
MODULE_DESCRIPTION("HUB_PANEL lcd Driver");
MODULE_LICENSE("GPL");

