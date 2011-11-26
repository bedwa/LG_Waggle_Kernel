/*
 * Cosmopolitan mipi interface support
 *
 * LGE_CHANGE [taekeun1.kim@lge.com] 2010-12-06
 */

#include <linux/module.h>
#include <linux/delay.h>
#include <linux/platform_device.h>
//#include <linux/i2c/twl6030.h>
#include <linux/spi/spi.h>
#include <linux/i2c.h>
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
#include <linux/hrtimer.h>

#include <linux/uaccess.h>
#include <linux/fs.h>
#include <linux/syscalls.h>


#include <plat/gpio.h>
#include <asm/mach-types.h>
#include <plat/control.h>

#include <plat/display.h>

#include <plat/heaven_lcd.h>

#include <plat/dmtimer.h>

int cosmo_panel_hdmi_flag=0;
int cosmo_panel_suspend_flag=0;
//#define DSI_VIDEO_MODE /* BRAD */
#define COSMO_LCD

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
// LGE_Change_S [Darren.Kang@lge.com] 2010.12.28 for block LCD pwr leakage [ST]
#define DCS_DEEP_STANDBY_IN		0xC1
// LGE_Change_S [Darren.Kang@lge.com] 2010.12.28 for block LCD pwr leakage [END]
#define DCS_GET_ID				0xf8

#define DSI_DT_DCS_SHORT_WRITE_0	0x05
#define DSI_DT_DCS_SHORT_WRITE_1	0x15
#define DSI_DT_DCS_READ			0x06
#define DSI_DT_SET_MAX_RET_PKG_SIZE	0x37
#define DSI_DT_NULL_PACKET		0x09
#define DSI_DT_DCS_LONG_WRITE		0x39


#define DSI_GEN_SHORTWRITE_NOPARAM 0x3
#define DSI_GEN_SHORTWRITE_1PARAM 0x13
#define DSI_GEN_SHORTWRITE_2PARAM 0x23
#define DSI_GEN_LONGWRITE 0x29

/* #define COSMO_PANEL_USE_ESD_CHECK */
#define COSMO_PANEL_ESD_CHECK_PERIOD	msecs_to_jiffies(5000)

#define COSMO_PANEL_LCD_RESET_N			30
#define COSMO_PANEL_3D_BOOST_ENABLE_GPIO	43
#define COSMO_PANEL_3D_ENABLE_GPIO		144

#if defined(CONFIG_MACH_LGE_COSMO_REV_B)
#define COSMO_PANEL_LCD_EN				27
#else
#define COSMO_PANEL_LCD_EN				190
#endif

//#define COSMO_PANEL_LCD_CS				7		-- by dongjin73.kim (not required for Cosmo LCD)

#define LONG_CMD_MIPI	0
#define SHORT_CMD_MIPI	1
#define END_OF_COMMAND	2

#define RET_IF_ERROR(x) {\
	if((r = (x)))		 \
		return r;		 \
	}

#if defined(CONFIG_MACH_LGE_COSMO_REV_A) || defined(CONFIG_MACH_LGE_COSMO_REV_B)

#define COSMO_PANEL_3D_BANK_SEL_GPIO		139
#define BANK_SEL_HALF_PERIOD_NS (1000000 / 120)

//S3D Pannel banke sel state
static int gpio_banksel_state = 0;

#else

#define S3D_BARRIER_PWM_CLK_SROUCE OMAP_TIMER_SRC_32_KHZ
#define S3D_BARRIER_INVERSION_HZ 60
#define S3D_BARRIER_INVERSION_COUTER_VALUE	 ( 32768 / (S3D_BARRIER_INVERSION_HZ*2) )
#define S3D_BARRIER_INVERSION_TIMER_ID 10

static struct mutex barrier_lock; // [LGE_CHANGES][2011-05-11][wonki.choi@lge.com] add mutex: barrier (un)lock

static struct omap_dm_timer *s3d_pwm_timer = NULL;

#define S3D_MAX_BRIGHTNESS_IN_3D_BARRIER

#ifdef  S3D_MAX_BRIGHTNESS_IN_3D_BARRIER
int lm3528_brightness_3D_Enable = 0;
static int oldBacklightBrightness = -1;

extern int lm3528_getBrightness(void);
extern ssize_t lm3528_setBrightness(int		brightness, size_t count);
int panel_cosmo_brightness_read(void)
{
#if 1
	return lm3528_getBrightness();
#else
	char buf[10];
	int tempbrightness = 0;
	
	int h_file = 0;
	int ret = 0;	

	mm_segment_t oldfs = get_fs();
	memset(buf, 0x00, sizeof(char)*10);	
	set_fs(KERNEL_DS);
	h_file = sys_open("/sys/devices/platform/i2c_omap.2/i2c-2/2-0036/brightness", O_RDWR,0);

	if(h_file >= 0)
	{	
		ret = sys_read( h_file, buf, 10);

		tempbrightness = (int)simple_strtol(buf, NULL, 10);
		printk("panel_cosmo_brightness_read %d.\n", tempbrightness);

		sys_close(h_file);
	}
	else
	{
		printk("panel_cosmo_brightness_read = %d.\n",h_file);			
		return oldBacklightBrightness;
	}

	set_fs(oldfs);

	return tempbrightness;
#endif	
}

void panel_cosmo_brightness_write(int value)

{
#if 1
	if(value == -1) return;

	lm3528_setBrightness(value, 0);
#else
	int h_file = 0;
	int ret = 0;
	char buf[10];
	int size = 0;
	
	if(value == -1) return;

	mm_segment_t oldfs = get_fs();
	memset(buf, 0x00, sizeof(char)*10);	
	set_fs(KERNEL_DS);
	h_file = sys_open("/sys/devices/platform/i2c_omap.2/i2c-2/2-0036/brightness", O_RDWR,0);

	if(h_file >= 0)
	{	
		size = sprintf(buf,"%d",value);

		ret = sys_write( h_file, buf, size);
	

		sys_close(h_file);
	}
	set_fs(oldfs);
#endif
}
#endif

static int panel_cosmo_create_s3d_pwm_timer(void)
{
	s3d_pwm_timer = omap_dm_timer_request_specific(S3D_BARRIER_INVERSION_TIMER_ID);
	if ( s3d_pwm_timer==NULL )
	{
		printk("S3D Barrier: fail creating PWM timer\n");
		return -1;
	}
	omap_dm_timer_disable(s3d_pwm_timer);
	return 0;	
}

static void panel_cosmo_start_s3d_pwm_timer(void)
{

	if ( s3d_pwm_timer==NULL )
	{
		printk("S3D Barrier : no timer. can't start pwm\n");
		return;
	}
	/** L8.2 **/
//	omap_dm_timer_enable(s3d_pwm_timer); 
//	omap_dm_timer_set_source(s3d_pwm_timer, OMAP_TIMER_SRC_32_KHZ);
//	omap_dm_timer_stop(s3d_pwm_timer);
//	omap_dm_timer_set_load(s3d_pwm_timer, 1, -S3D_BARRIER_INVERSION_COUTER_VALUE);
//	omap_dm_timer_set_pwm(s3d_pwm_timer, 0, 1, OMAP_TIMER_TRIGGER_OVERFLOW);
//	omap_dm_timer_write_counter(s3d_pwm_timer, -S3D_BARRIER_INVERSION_COUTER_VALUE);
//	omap_dm_timer_start(s3d_pwm_timer);
	/** L10.1 **/
	omap_dm_timer_enable(s3d_pwm_timer); 
	omap_dm_timer_set_source(s3d_pwm_timer, OMAP_TIMER_SRC_32_KHZ);
	omap_dm_timer_set_pwm(s3d_pwm_timer, 0, 1, OMAP_TIMER_TRIGGER_OVERFLOW);
	omap_dm_timer_set_load_start(s3d_pwm_timer, 1, -S3D_BARRIER_INVERSION_COUTER_VALUE);
}

static void panel_cosmo_stop_s3d_pwm_timer(void) 
{
	if ( s3d_pwm_timer==NULL )
	{
		printk("S3D Barrier : no timer. can't stop pwm\n");
		return;
	}
	omap_dm_timer_stop(s3d_pwm_timer);
	omap_dm_timer_disable(s3d_pwm_timer);
}

#endif

static const struct i2c_device_id barrier_id[] = {
	{ "lgdp4512_barrierA", 0 },
	{ "lgdp4512_barrierB", 1 },
	{ },
};
static struct omap_dss_device *omapdssdev;

//S3D panel 3d enable state
static int gpio_3d_enable_state = 0;
//S3D panel 3d boost enable state
static int gpio_3d_boost_enable_state = 0;


u8 lcd_command_for_mipi[][30] = {
/* [START]2010.08.03 jinseok.choi@lge.com For Cosmo LGD LCD Display, LCD Initial Code : Refer to "LH430WV2-SD01_Initialcode_ver 0.1_100727.pdf"  */
// Darren.Kang 2010.12.24 Change Initial Code for LCD [ST]
	{SHORT_CMD_MIPI,DSI_GEN_SHORTWRITE_1PARAM,0x01,0x20,},											/* Display Inversion */ 																	
	{SHORT_CMD_MIPI,DSI_GEN_SHORTWRITE_2PARAM,0x02,0x36,0x00,}, 									/* Set Address Mode */
	{SHORT_CMD_MIPI,DSI_GEN_SHORTWRITE_2PARAM,0x02,0x3A,0x70,}, 									/* Interface Pixel Fromat */
	//LGE_CHANGE_S [taekeun1.kim@lge.com] 2010-12-14, P920 : code refine.												  
	//	  {LONG_CMD_MIPI, DSI_GEN_LONGWRITE,0x04,0xB1, 0x06, 0x43, 0x1C,},							/* RGB Interface Setting */ 																				
	{LONG_CMD_MIPI, DSI_GEN_LONGWRITE,0x04,0xB1, 0x06, 0x43, 0x0A,},								/* RGB Interface Setting */
	//LGE_CHANGE_E [taekeun1.kim@lge.com] 2010-12-14, P920 : code refine.
	{LONG_CMD_MIPI, DSI_GEN_LONGWRITE,0x03,0xB2, 0x00, 0xC8,},										/* Panel Characteristics Setting */
	{SHORT_CMD_MIPI,DSI_GEN_SHORTWRITE_2PARAM, 0x02, 0xB3, 0x00,},									/* Panel Drive Setting */
	{SHORT_CMD_MIPI,DSI_GEN_SHORTWRITE_2PARAM, 0x02, 0xB4, 0x04,},									/* Display Mode Control */
	//LGE_CHANGE_S [taekeun1.kim@lge.com] 2010-12-14, P920 : code refine.										  
	//	  {LONG_CMD_MIPI, DSI_GEN_LONGWRITE,0x06, 0xB5, 0x42, 0x18, 0x02, 0x04, 0x10 ,},			/* Display Control 1 */
	{LONG_CMD_MIPI, DSI_GEN_LONGWRITE,0x06, 0xB5, 0x40, 0x18, 0x02, 0x00, 0x01 ,},					/* Display Control 1 */
	//LGE_CHANGE_E [taekeun1.kim@lge.com] 2010-12-14, P920 : code refine.												  
	{LONG_CMD_MIPI, DSI_GEN_LONGWRITE,0x07, 0xB6, 0x0B, 0x0F, 0x02, 0x40, 0x10, 0xE8,}, 			/* Display Control 2 */   
	// LGE_ChangeS [Darren.Kang@lge.com] 20110104 change for image stablization <- by sync on [ST]
	//{LONG_CMD_MIPI, DSI_GEN_LONGWRITE,0x06, 0xC3, 0x07, 0x02, 0x02, 0x02, 0x02 ,},				/* Power Control 3 */
	{LONG_CMD_MIPI, DSI_GEN_LONGWRITE,0x06, 0xC3, 0x07, 0x0A, 0x0A, 0x0A, 0x02 ,},					/* Power Control 3 */
	// LGE_ChangeS [Darren.Kang@lge.com] 20110104 change for image stablization <- by sync on [END]
	// LGE_CHANGES Darren.Kang@lge.com 20110126 for optimizing panel gate voltage & VCOM Level  [ST]
	//{LONG_CMD_MIPI, DSI_GEN_LONGWRITE,0x07, 0xC4, 0x12, 0x24, 0x18, 0x18, 0x04, 0x49,},  panle gate voltage						/* Power Control 4 */
	{LONG_CMD_MIPI, DSI_GEN_LONGWRITE,0x07, 0xC4, 0x12, 0x24, 0x18, 0x18, 0x04, 0x49,}, 			/* Power Control 4 */	
	//{SHORT_CMD_MIPI,DSI_GEN_SHORTWRITE_2PARAM, 0x02, 0xC5, 0x5B,}, VCOM level						/* Power Control 5 */
	{SHORT_CMD_MIPI,DSI_GEN_SHORTWRITE_2PARAM, 0x02, 0xC5, 0x6B,},									/* Power Control 5 */	
	// LGE_CHANGES Darren.Kang@lge.com 20110126 for optimizing panel gate voltage & VCOM Leve [END]
	{LONG_CMD_MIPI, DSI_GEN_LONGWRITE,0x04,0xC6, 0x41, 0x63, 0x03,},								/* Power Control 6 */	
	{LONG_CMD_MIPI, DSI_GEN_LONGWRITE,0x0A,0xD0, 0x33, 0x22, 0x77, 0x02, 0x00, 0x00, 0x30, 0x01, 0x01,}, /* Positive Gamma Curve for Red */
	{LONG_CMD_MIPI, DSI_GEN_LONGWRITE,0x0A,0xD1, 0x33, 0x22, 0x77, 0x02, 0x00, 0x00, 0x30, 0x01, 0x01,}, /* Negative Gamma Curve for Red */
	{LONG_CMD_MIPI, DSI_GEN_LONGWRITE,0x0A,0xD2, 0x33, 0x22, 0x77, 0x02, 0x00, 0x00, 0x30, 0x01, 0x01,}, /* Positive Gamma Curve for Green */
	{LONG_CMD_MIPI, DSI_GEN_LONGWRITE,0x0A,0xD3, 0x33, 0x22, 0x77, 0x02, 0x00, 0x00, 0x30, 0x01, 0x01,}, /* Negative Gamma Curve for Green */
	{LONG_CMD_MIPI, DSI_GEN_LONGWRITE,0x0A,0xD4, 0x33, 0x22, 0x77, 0x02, 0x00, 0x00, 0x30, 0x01, 0x01,}, /* Positive Gamma Curve for Blue */
	{LONG_CMD_MIPI, DSI_GEN_LONGWRITE,0x0A,0xD5, 0x33, 0x22, 0x77, 0x02, 0x00, 0x00, 0x30, 0x01, 0x01,}, /* Negative Gamma Curve for Blue */
	{END_OF_COMMAND, },	
// Darren.Kang 2010.12.24 Change Initial Code for LCD [END]
};

static void cosmo_panel_esd_work(struct work_struct *work);

#if defined(CONFIG_MACH_LGE_COSMO_REV_A) || defined(CONFIG_MACH_LGE_COSMO_REV_B)
static void toggle_3d_bank_sel_gpio(void)
{
	gpio_banksel_state = !gpio_banksel_state;
	gpio_set_value(COSMO_PANEL_3D_BANK_SEL_GPIO, gpio_banksel_state);
//	gpio_set_value(COSMO_PANEL_3D_BANK_SEL_GPIO, 1);
//	gpio_set_value(COSMO_PANEL_3D_BANK_SEL_GPIO, 0);
//	printk("banksel inversion %d\n", gpio_banksel_state);
}

static enum hrtimer_restart timer_cb(struct hrtimer *timer)
{	
	toggle_3d_bank_sel_gpio();
	hrtimer_forward_now(timer, ktime_set(0, BANK_SEL_HALF_PERIOD_NS));
	return HRTIMER_RESTART;
}
#endif

#if defined(CONFIG_MACH_LGE_COSMO_REV_A) || defined(CONFIG_MACH_LGE_COSMO_REV_B)
static int barrier_i2c_write(struct i2c_client *client, u8 reg, u8 value1, u8 value2)
{
	u8 data[3];
	int r = 0;
	int retry=5;
	
	data[0] = reg;
	data[1] = value1;
	data[2] = value2;

	while(--retry)
	{
		r = i2c_master_send(client, (char *)&data, 3);	
		if (r != 3) {
			dev_err(&omapdssdev->dev, "I2C error writing reg 0x%02x\n", reg);
			dev_err(&omapdssdev->dev, "retrying %d...\n", retry);
		}
		else {
//			dev_info(&omapdssdev->dev, "I2C write 0x%02x:[0x%02x,0x%02x]\n", reg, value1, value2);
			return 0;
		}	
	}

	return r;
	
}


static struct {
	u8 dac_address;
	u8 data_1;
	u8 data_2;
} barrier_init_seq[] = {
		{ 0x01, 0x03, 0xD7 },
		{ 0x02, 0x02, 0x2c },
		{ 0x03, 0x02, 0x2c },
		{ 0x04, 0x02, 0x2c },
		{ 0x05, 0x03, 0xd7 },
		{ 0x06, 0x03, 0xd7 },
		{ 0x07, 0x00, 0x00 },
		{ 0x08, 0x00, 0x00 },
		{ 0x09, 0x00, 0x00 },
		{ 0x0a, 0x00, 0x00 },
		{ 0x0b, 0x00, 0x00 },
		{ 0x0c, 0x00, 0x00 },
		{ 0x0d, 0x00, 0x00 },
		{ 0x0e, 0x00, 0x00 },
		{ 0x0f, 0x00, 0x00 },
		{ 0x10, 0x00, 0x00 },
		{ 0x11, 0x02, 0x2c },
		{ 0x81, 0x00, 0x80 },
		{ 0x82, 0x02, 0x2c },
		{ 0x83, 0x02, 0x2c },
		{ 0x84, 0x02, 0x2c },
		{ 0x85, 0x00, 0x80 },
		{ 0x86, 0x00, 0x80 },
		{ 0x87, 0x00, 0x00 },
		{ 0x88, 0x00, 0x00 },
		{ 0x89, 0x00, 0x00 },
		{ 0x8a, 0x00, 0x00 },
		{ 0x8b, 0x00, 0x00 },
		{ 0x8c, 0x00, 0x00 },
		{ 0x8d, 0x00, 0x00 },
		{ 0x8e, 0x00, 0x00 },
		{ 0x8f, 0x00, 0x00 },
		{ 0x90, 0x00, 0x00 },
		{ 0x91, 0x02, 0x3c }
};
#endif

static int barrier_init(struct cosmo_panel_data *panel_data, bool enable)
{
	mutex_lock(&barrier_lock); // [LGE_CHANGES][2011-05-11][wonki.choi@lge.com] add mutex: barrier (un)lock
	if ( panel_data->barrier_enabled==enable )
	{
		mutex_unlock(&barrier_lock); // [LGE_CHANGES][2011-05-11][wonki.choi@lge.com] add mutex: barrier (un)lock
		return 0;
	}

	if(enable)
	{

		if ( panel_data->barrier_enabled )	//alredy turn on
		{
			mutex_unlock(&barrier_lock); // [LGE_CHANGES][2011-05-11][wonki.choi@lge.com] add mutex: barrier (un)lock
			return 0;
		}

		printk("barrier enable\n");
#if !defined(CONFIG_MACH_LGE_COSMO_REV_A)		
		//first turn on 3d enable
		gpio_3d_enable_state = 1;
		gpio_set_value(COSMO_PANEL_3D_ENABLE_GPIO, gpio_3d_enable_state);
#endif
		//second turn on boost
		gpio_3d_boost_enable_state = 1;
		gpio_set_value(COSMO_PANEL_3D_BOOST_ENABLE_GPIO, gpio_3d_boost_enable_state);

#if defined(CONFIG_MACH_LGE_COSMO_REV_A) || defined(CONFIG_MACH_LGE_COSMO_REV_B)
		hrtimer_start(&(panel_data->timer), ktime_set(0, BANK_SEL_HALF_PERIOD_NS) ,HRTIMER_MODE_REL);
#else
		//pwm on
		panel_cosmo_start_s3d_pwm_timer();
#endif
		msleep(30);

#if defined(CONFIG_MACH_LGE_COSMO_REV_A) || defined(CONFIG_MACH_LGE_COSMO_REV_B)
		//write I2c
		{
			int i;
			int r;
			for(i=0;i<( sizeof(barrier_init_seq)/sizeof(barrier_init_seq[0]) );i++ )
			{
				if ( (r = barrier_i2c_write(panel_data->barrier_i2c[0],
								barrier_init_seq[i].dac_address,
								barrier_init_seq[i].data_1,
								barrier_init_seq[i].data_2) ) ) 
				{
					printk("S3D Barrier On : I2C Write Fail\n");
				}
			}
			//after 30ms, s3d barrier will work
			msleep(30);
		}
#endif
		panel_data->barrier_enabled = true;

#ifdef  S3D_MAX_BRIGHTNESS_IN_3D_BARRIER
		oldBacklightBrightness = panel_cosmo_brightness_read();
		lm3528_brightness_3D_Enable = 1;
		panel_cosmo_brightness_write(oldBacklightBrightness);
#endif		
	}
	else
	{
		if ( !panel_data->barrier_enabled )	//alredy turn on
		{
			mutex_unlock(&barrier_lock); // [LGE_CHANGES][2011-05-11][wonki.choi@lge.com] add mutex: barrier (un)lock
			return 0;
		}

#if defined(CONFIG_MACH_LGE_COSMO_REV_A) || defined(CONFIG_MACH_LGE_COSMO_REV_B)
		{
			int i;
			int r;
			for(i=0;i<( sizeof(barrier_init_seq)/sizeof(barrier_init_seq[0]) );i++ )
			{
				if ( ( r = barrier_i2c_write(panel_data->barrier_i2c[0], barrier_init_seq[i].dac_address,0x0,0x0) ) ) 
				{
					printk("S3D Barrier Off : I2C Write Fail\n");
					mutex_unlock(&barrier_lock); // [LGE_CHANGES][2011-05-11][wonki.choi@lge.com] add mutex: barrier (un)lock
					return r;
				}
			}
		}
#endif

		//first turn off boost
		gpio_3d_boost_enable_state = 0;
		gpio_set_value(COSMO_PANEL_3D_BOOST_ENABLE_GPIO, gpio_3d_boost_enable_state);
		//turn off 3d
#if !defined(CONFIG_MACH_LGE_COSMO_REV_A)		
		gpio_3d_enable_state = 0;
		gpio_set_value(COSMO_PANEL_3D_ENABLE_GPIO, gpio_3d_enable_state);
#endif

#if defined(CONFIG_MACH_LGE_COSMO_REV_A) || defined(CONFIG_MACH_LGE_COSMO_REV_B)
		hrtimer_cancel(&(panel_data->timer));
#else
		//pwm off
		panel_cosmo_stop_s3d_pwm_timer();
#endif
		panel_data->barrier_enabled = false;

#ifdef  S3D_MAX_BRIGHTNESS_IN_3D_BARRIER
		if(panel_cosmo_brightness_read() <= 40)
		{
			oldBacklightBrightness = -1;
		}
		lm3528_brightness_3D_Enable = 0;
		panel_cosmo_brightness_write(oldBacklightBrightness);
#endif		
	}
	mutex_unlock(&barrier_lock); // [LGE_CHANGES][2011-05-11][wonki.choi@lge.com] add mutex: barrier (un)lock
	return 0;
}

//@todo remove this function
int barrier_init_hotkey(bool enable)
{	
	struct cosmo_panel_data *td = dev_get_drvdata(&omapdssdev->dev);	
	if(omapdssdev->state != OMAP_DSS_DISPLAY_ACTIVE) return;
	return barrier_init(td, enable);
}
EXPORT_SYMBOL(barrier_init_hotkey);


static int cosmo_panel_enable_s3d(struct omap_dss_device *dssdev, bool enable) 
{
	struct cosmo_panel_data *td = dev_get_drvdata(&omapdssdev->dev);
	if(dssdev->state != OMAP_DSS_DISPLAY_ACTIVE) return;
	return barrier_init(td, enable);
}

static bool cosmo_panel_get_s3d_enabled(struct omap_dss_device *dssdev) 
{
	struct cosmo_panel_data *td = dev_get_drvdata(&omapdssdev->dev);
	return td->barrier_enabled;
}

int cosmo_panel_set_s3d_disp_type(struct omap_dss_device *dssdev, struct s3d_disp_info *info)
{
	memcpy((struct s3d_disp_info *)&(dssdev->panel.s3d_info), info, sizeof(*info));
	return 0;
}

int cosmo_panel_get_s3d_disp_type(struct omap_dss_device *dssdev, struct s3d_disp_info *info)
{
	info = &dssdev->panel.s3d_info;
	return 0;
}

static void hw_guard_start(struct cosmo_panel_data *td, int guard_msec)
{
	td->hw_guard_wait = msecs_to_jiffies(guard_msec);
	td->hw_guard_end = jiffies + td->hw_guard_wait;
}

static void hw_guard_wait(struct cosmo_panel_data *td)
{
	unsigned long wait = td->hw_guard_end - jiffies;

	if ((long)wait > 0 && wait <= td->hw_guard_wait) {
		set_current_state(TASK_UNINTERRUPTIBLE);
		schedule_timeout(wait);
	}
}

static int cosmo_panel_dcs_read_1(u8 dcs_cmd, u8 *data)
{
	int r;
	u8 buf[1];

	r = dsi_vc_dcs_read(1,TCH, dcs_cmd, buf, 1);

	if (r < 0)
		return r;

	*data = buf[0];

	return 0;
}

static int cosmo_panel_dcs_write_0(u8 dcs_cmd)
{
	return dsi_vc_dcs_write(1,TCH, &dcs_cmd, 1);
}

static int cosmo_panel_dcs_write_1(u8 dcs_cmd, u8 param)
{
	u8 buf[2];
	buf[0] = dcs_cmd;
	buf[1] = param;
	return dsi_vc_dcs_write(1,TCH, buf, 2);
}

static int cosmo_panel_sleep_in(struct cosmo_panel_data *td)

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

static int cosmo_panel_sleep_out(struct cosmo_panel_data *td)
{
	int r;

	hw_guard_wait(td);	

	r = cosmo_panel_dcs_write_0(DCS_SLEEP_OUT);
	if (r)
		return r;

	hw_guard_start(td, 120);	

	msleep(5);

	return 0;
}

static int cosmo_panel_get_id(u8 *buf)
{
	int r;

	r = dsi_vc_dcs_read(1,TCH, DCS_GET_ID, buf, 3);
	if (r)
		return r;

	return 0;
}

static int cosmo_panel_set_addr_mode(u8 rotate, bool mirror)
{
	int r;
	u8 mode;
	int b5, b6, b7;

	r = cosmo_panel_dcs_read_1(DCS_READ_MADCTL, &mode);
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

	return cosmo_panel_dcs_write_1(DCS_MEM_ACC_CTRL, mode);
}

static int cosmo_panel_bl_update_status(struct backlight_device *dev)
{
	struct omap_dss_device *dssdev = dev_get_drvdata(&dev->dev);
	struct cosmo_panel_data *td = dev_get_drvdata(&dssdev->dev);
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
			r = cosmo_panel_dcs_write_1(DCS_BRIGHTNESS, level);
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

static int cosmo_panel_bl_get_intensity(struct backlight_device *dev)
{
	if (dev->props.fb_blank == FB_BLANK_UNBLANK &&
			dev->props.power == FB_BLANK_UNBLANK)
		return dev->props.brightness;

	return 0;
}

static struct backlight_ops cosmo_panel_bl_ops = {
	.get_brightness = cosmo_panel_bl_get_intensity,
	.update_status  = cosmo_panel_bl_update_status,
};

static void cosmo_panel_get_timings(struct omap_dss_device *dssdev,
			struct omap_video_timings *timings)
{
	*timings = dssdev->panel.timings;
}

static ssize_t cosmo_panel_num_errors_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct omap_dss_device *dssdev = to_dss_device(dev);
	struct cosmo_panel_data *td = dev_get_drvdata(&dssdev->dev);
	u8 errors;
	int r;

	if (td->enabled) {
		dsi_bus_lock(1);
		r = cosmo_panel_dcs_read_1(DCS_READ_NUM_ERRORS, &errors);
		dsi_bus_unlock(1);
	} else {
		r = -ENODEV;
	}

	if (r)
		return r;

	return snprintf(buf, PAGE_SIZE, "%d\n", errors);
}

static ssize_t cosmo_panel_hw_revision_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct omap_dss_device *dssdev = to_dss_device(dev);
	struct cosmo_panel_data *td = dev_get_drvdata(&dssdev->dev);
	u8 dbuf[3];
	int r;

	if (td->enabled) {
		dsi_bus_lock(1);
		r = cosmo_panel_get_id(dbuf);
		dsi_bus_unlock(1);
	} else {
		r = -ENODEV;
	}

	if (r)
		return r;

	return snprintf(buf, PAGE_SIZE, "%02x.%02x.%02x\n", dbuf[0], dbuf[1], dbuf[2]);
}

static ssize_t cosmo_panel_barrier_enable_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct omap_dss_device *dssdev = to_dss_device(dev);
	struct cosmo_panel_data *td = dev_get_drvdata(&dssdev->dev);
	return snprintf(buf, PAGE_SIZE, "%s\n",(td->barrier_enabled) ? "true" : "false");
}

static ssize_t cosmo_panel_barrier_enable_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	struct omap_dss_device *dssdev = to_dss_device(dev);
	struct cosmo_panel_data *td = dev_get_drvdata(&dssdev->dev);
	bool barrier_enable = false;

	barrier_enable = simple_strtoul(buf, NULL, 10);
	
	if(td->barrier_enabled == barrier_enable)
		return count;
// LGE_CHANGE_S [wonki.choi@lge.com] 2011-05-25 HW recovery fot SGX lockup
	return count;
}


static ssize_t cosmo_sgx_manual_recovery_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	return 0;
}

extern void tmm_dmm_free_page_stack(void);
static ssize_t cosmo_sgx_manual_recovery_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	extern unsigned int sgx_manual_recovery;
		
	
		struct omap_dss_device *dssdev = to_dss_device(dev);
		struct cosmo_panel_data *td = dev_get_drvdata(&dssdev->dev);
		int number = 0;
	
		number = simple_strtoul(buf, NULL, 10);
	
		if(number == 3246)
		{
			// to do 
			tmm_dmm_free_page_stack();
		}
		else	
	sgx_manual_recovery = 1;
		
// LGE_CHANGE_E [wonki.choi@lge.com] 2011-05-25 HW recovery fot SGX lockup
	return count;

}


static DEVICE_ATTR(num_dsi_errors, S_IRUGO, cosmo_panel_num_errors_show, NULL);
static DEVICE_ATTR(hw_revision, S_IRUGO, cosmo_panel_hw_revision_show, NULL);
static DEVICE_ATTR(barrier_enable, S_IRUGO|S_IWUSR|S_IWGRP, cosmo_panel_barrier_enable_show, cosmo_panel_barrier_enable_store);
// LGE_CHANGE [wonki.choi@lge.com] 2011-05-25 HW recovery fot SGX lockup
static DEVICE_ATTR(sgx_manual_recovery, S_IRUGO|S_IWUSR|S_IWGRP, cosmo_sgx_manual_recovery_show, cosmo_sgx_manual_recovery_store);

void cosmo_panel_reset_lcd(void)
{
	gpio_set_value(COSMO_PANEL_LCD_RESET_N, 0);
	mdelay(1);
	gpio_set_value(COSMO_PANEL_LCD_RESET_N, 1);
	mdelay(1);
}

extern void dsi_enable_video_mode(struct omap_dss_device *dssdev);
extern int dispc_enable_gamma(enum omap_channel ch, u8 gamma);
static int cosmo_panel_enable(struct omap_dss_device *dssdev)
{
	struct cosmo_panel_data *td = dev_get_drvdata(&dssdev->dev);
	int i, r;

	if(td->enabled) return 0;
	if (dssdev->platform_enable) {
		r = dssdev->platform_enable(dssdev);
		if (r)
			return r;
	}

	cosmo_panel_reset_lcd();

	dsi_bus_lock(DSI2);
	r = omapdss_dsi_display_enable(dssdev);
	if (r) {
		dev_err(&dssdev->dev, "failed to enable DSI\n");
		goto err;
	}

	/* it seems we have to wait a bit until cosmo_panel is ready */
	mdelay(5);	

	//print_all_dss_reg(1); 
	for (i = 0; lcd_command_for_mipi[i][0] != END_OF_COMMAND; i++) {
		dsi_vc_dcs_write(1, TCH, &lcd_command_for_mipi[i][3], lcd_command_for_mipi[i][2]);
	}

	r = cosmo_panel_sleep_out(td);
	if (r)
		goto err;
	cosmo_panel_dcs_write_0(DCS_DISPLAY_ON);	
 /* [End]2010.08.03 jinseok.choi@lge.com For Cosmo LGD LCD Display, LCD Initial Code : Refer to "LH430WV2-SD01_Initialcode_ver 0.1_100727.pdf"  */
#if defined(CONFIG_COSMO_S_CURVE) && defined(CONFIG_COSMO_GAMMA)                         
        dispc_enable_gamma(OMAP_DSS_CHANNEL_LCD, 0);
        dispc_enable_gamma(OMAP_DSS_CHANNEL_LCD2, 0);
#endif                  

	dsi_enable_video_mode(dssdev); 
	mdelay(30); //after 1 frame

	dsi_bus_unlock(DSI2);

#ifdef HUB_PANEL_USE_ESD_CHECK
	queue_delayed_work(td->esd_wq, &td->esd_work, COSMO_PANEL_ESD_CHECK_PERIOD);
#endif
	

	td->enabled = 1;
	dssdev->state = OMAP_DSS_DISPLAY_ACTIVE;
	cosmo_panel_hdmi_flag = 0;
	return 0;
err:
	if (dssdev->platform_disable)
		dssdev->platform_disable(dssdev);

	return r;
}

static struct attribute *cosmo_panel_attrs[] = {
	&dev_attr_num_dsi_errors.attr,
	&dev_attr_hw_revision.attr,
	&dev_attr_barrier_enable.attr,
// LGE_CHANGE [wonki.choi@lge.com] 2011-05-25 HW recovery fot SGX lockup
	&dev_attr_sgx_manual_recovery.attr,
	NULL,
};

static struct attribute_group cosmo_panel_attr_group = {
	.attrs = cosmo_panel_attrs,
};

static int cosmo_i2c_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
	struct cosmo_panel_data *panel_data = dev_get_drvdata(&omapdssdev->dev);
	panel_data->barrier_i2c[id->driver_data] = client;	
	printk("I2C Probe!\n");
	return 0;
}

static int cosmo_i2c_remove(struct i2c_client *client)
{
	return 0;
}


static struct i2c_driver lgs3d_i2c_driver = {
	.driver = {
		.name = "lgdp4512_barrier",
		.owner = THIS_MODULE,
	},
	.probe 		= cosmo_i2c_probe,
	.remove 	= cosmo_i2c_remove,
	.id_table	= barrier_id,
};

static int cosmo_panel_probe(struct omap_dss_device *dssdev)
{
	struct cosmo_panel_data *td;
	struct backlight_device *bldev;
	int r;

	dev_dbg(&dssdev->dev, "probe\n");

	td = kzalloc(sizeof(*td), GFP_KERNEL);
	if (!td) {
		r = -ENOMEM;
		goto err0;
	}
	td->dssdev = dssdev;

 
	td->esd_wq = create_singlethread_workqueue("cosmo_panel_esd");
	if (td->esd_wq == NULL) {
		dev_err(&dssdev->dev, "can't create ESD workqueue\n");
		r = -ENOMEM;
		goto err2;
	}
	INIT_DELAYED_WORK_DEFERRABLE(&td->esd_work, cosmo_panel_esd_work);

	dev_set_drvdata(&dssdev->dev, td);

	omapdssdev = dssdev;

	if(i2c_add_driver(&lgs3d_i2c_driver))
	{	
		dev_err(&dssdev->dev,"error registering I2C or SPI driver!\n");
		kfree(td);
		return -ENODEV;
	}
	
	//[jonathan.kim@lge.com] S add ti patch for s3d interlaced
	dssdev->panel.s3d_info.type = S3D_DISP_ROW_IL;
	/*First scan line is always for right view*/ 
	dssdev->panel.s3d_info.order = S3D_DISP_ORDER_R;
	//[jonathan.kim@lge.com] E add ti patch for s3d interlaced

	/* if no platform set_backlight() defined, presume DSI backlight
	 * control */
	if (!dssdev->set_backlight)
		td->use_dsi_bl = true;


	bldev = backlight_device_register("cosmo_panel", &dssdev->dev, dssdev,
			&cosmo_panel_bl_ops, NULL);
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

	cosmo_panel_bl_update_status(bldev);

	r = sysfs_create_group(&dssdev->dev.kobj, &cosmo_panel_attr_group);
	if (r) {
		dev_err(&dssdev->dev, "failed to create sysfs files\n");
	}

	mutex_init(&barrier_lock); // [LGE_CHANGES][2011-05-11][wonki.choi@lge.com] add mutex: barrier (un)lock
	//initialize gpio
	//3D boost
	if ( gpio_request(COSMO_PANEL_3D_BOOST_ENABLE_GPIO, "3D_boost_enable") )
	{
		dev_err(&dssdev->dev, "3D Panel 3D boost GPIO request failed\n");
		gpio_free(COSMO_PANEL_3D_BOOST_ENABLE_GPIO);
		goto err3;
	}
	gpio_direction_output(COSMO_PANEL_3D_BOOST_ENABLE_GPIO, gpio_3d_boost_enable_state);
		
	//3D enable
	if ( gpio_request(COSMO_PANEL_3D_ENABLE_GPIO, "3D_enable") )
	{
		dev_err(&dssdev->dev, "3D Panel 3D enable GPIO request failed\n");
		gpio_free(COSMO_PANEL_3D_ENABLE_GPIO);
		goto err3;
	}
#if !defined(CONFIG_MACH_LGE_COSMO_REV_A)
	gpio_3d_enable_state = 0;
	gpio_direction_output(COSMO_PANEL_3D_ENABLE_GPIO, gpio_3d_enable_state);
#endif
	td->barrier_enabled = false;
#if defined(CONFIG_MACH_LGE_COSMO_REV_A) || defined(CONFIG_MACH_LGE_COSMO_REV_B)

	//3D_BANK_SEL : inversion
	if ( gpio_request(COSMO_PANEL_3D_BANK_SEL_GPIO, "3D_banksel") )
	{
		dev_err(&dssdev->dev, "3D Panel BANK SEL GPIO request failed\n");
		gpio_free(COSMO_PANEL_3D_BANK_SEL_GPIO);
		goto err3;
	}
	gpio_direction_output(COSMO_PANEL_3D_BANK_SEL_GPIO, gpio_banksel_state);
	
	hrtimer_init(&td->timer, CLOCK_MONOTONIC, HRTIMER_MODE_REL);
	td->timer.function = timer_cb;	
#else
	if ( panel_cosmo_create_s3d_pwm_timer() )
	{
		dev_err(&dssdev->dev, "S3D Inversion PWM Timer creation fail\n");
		goto err3;
	}
#endif
	td->enabled = 1;
	dssdev->state = OMAP_DSS_DISPLAY_ACTIVE;
	return 0;
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

static void cosmo_panel_remove(struct omap_dss_device *dssdev)
{
	struct cosmo_panel_data *td = dev_get_drvdata(&dssdev->dev);
	struct backlight_device *bldev;

	dev_dbg(&dssdev->dev, "remove\n");

	sysfs_remove_group(&dssdev->dev.kobj, &cosmo_panel_attr_group);

	bldev = td->bldev;
	bldev->props.power = FB_BLANK_POWERDOWN;
	cosmo_panel_bl_update_status(bldev);
	backlight_device_unregister(bldev);

	cancel_delayed_work_sync(&td->esd_work);
	destroy_workqueue(td->esd_wq);

	mutex_destroy(&barrier_lock); // [LGE_CHANGES][2011-05-11][wonki.choi@lge.com] add mutex: barrier (un)lock

	kfree(td);
}

static void cosmo_panel_disable(struct omap_dss_device *dssdev)
{
	struct cosmo_panel_data *td = dev_get_drvdata(&dssdev->dev);

	dev_dbg(&dssdev->dev, "disable\n");

	cancel_delayed_work(&td->esd_work);

	cosmo_panel_dcs_write_0(DCS_DISPLAY_OFF);
	cosmo_panel_sleep_in(td);

	/* wait a bit so that the message goes through */
	msleep(10);

	if (dssdev->platform_disable)
		dssdev->platform_disable(dssdev);
	
	td->enabled = 0;
	cosmo_panel_hdmi_flag = 1;
}

static int cosmo_panel_suspend(struct omap_dss_device *dssdev)
{
	struct cosmo_panel_data *td = dev_get_drvdata(&dssdev->dev);
	struct backlight_device *bldev = td->bldev;

	bldev->props.power = FB_BLANK_POWERDOWN;
	cosmo_panel_bl_update_status(bldev);

	if(td->enabled)
	{
		td->enabled =0 ;	
	}
	
	dsi_bus_lock(DSI2);
// LGE_CHANGE Darren.Kang@lge.com 2011.01.25 For send cmd during LCD receiving video [ST]		
	dsi_enable_dcs_cmd(dssdev,DSI2);	
   	cosmo_panel_dcs_write_0(DCS_SLEEP_IN);
	dsi_disable_dcs_cmd(dssdev,DSI2);	

	msleep(100); // skipped 7 frames  

	dsi_enable_dcs_cmd(dssdev,DSI2);	
	cosmo_panel_dcs_write_1(DCS_DEEP_STANDBY_IN, 0x01);
	dsi_disable_dcs_cmd(dssdev,DSI2);	
	
	msleep(10);
// LGE_CHANGE Darren.Kang@lge.com 2011.01.25 For send cmd during LCD receiving video [END]		

	omapdss_dsi_display_disable(dssdev);
//LGE_CHANGE_S [taekeun1.kim@lge.com] 2010-12-14, P920 : code refine.
	barrier_init(td, false);     
	if (dssdev->platform_disable)
		dssdev->platform_disable(dssdev);
//LGE_CHANGE_E [taekeun1.kim@lge.com] 2010-12-14, P920 : code refine.
	dsi_bus_unlock(DSI2);
	dssdev->state = OMAP_DSS_DISPLAY_SUSPENDED;
	cosmo_panel_suspend_flag = 1;
	cosmo_panel_hdmi_flag = 1;
	return 0;
}

static int cosmo_panel_resume(struct omap_dss_device *dssdev)
{
	//struct cosmo_panel_data *td = dev_get_drvdata(&dssdev->dev);
	//struct backlight_device *bldev = td->bldev;

// LGE_Change_S [Darren.Kang@lge.com] 2010.12.28 for block LCD pwr leakage [ST]
/*
	gpio_set_value(COSMO_PANEL_LCD_EN, 1);
	msleep(30);	
	gpio_set_value(COSMO_PANEL_LCD_EN, 0);
	msleep(30);	
	gpio_set_value(COSMO_PANEL_LCD_EN, 1);
	msleep(20);
	*/
// LGE_Change_S [Darren.Kang@lge.com] 2010.12.28 for block LCD pwr leakage [END]
	
	cosmo_panel_enable(dssdev);
	cosmo_panel_suspend_flag = 0;
	cosmo_panel_hdmi_flag =0;

	return 0;
}

static int cosmo_panel_enable_te(struct omap_dss_device *dssdev, bool enable)
{
	struct cosmo_panel_data *td = dev_get_drvdata(&dssdev->dev);
	int r;

	td->te_enabled = enable;

	if (enable)
		r = cosmo_panel_dcs_write_1(DCS_TEAR_ON, 0);
	else
		r = cosmo_panel_dcs_write_0(DCS_TEAR_OFF);
	return r;
}

static int cosmo_panel_rotate(struct omap_dss_device *dssdev, u8 rotate)
{
	struct cosmo_panel_data *td = dev_get_drvdata(&dssdev->dev);
	int r;

	dev_dbg(&dssdev->dev, "rotate %d\n", rotate);

	if (td->enabled) {
		r = cosmo_panel_set_addr_mode(rotate, td->mirror);

		if (r)
			return r;
	}

	td->rotate = rotate;

	return 0;
}

static u8 cosmo_panel_get_rotate(struct omap_dss_device *dssdev)
{
	struct cosmo_panel_data *td = dev_get_drvdata(&dssdev->dev);
	return td->rotate;
}

static int cosmo_panel_mirror(struct omap_dss_device *dssdev, bool enable)
{
	struct cosmo_panel_data *td = dev_get_drvdata(&dssdev->dev);
	int r;

	dev_dbg(&dssdev->dev, "mirror %d\n", enable);

	if (td->enabled) {
		r = cosmo_panel_set_addr_mode(td->rotate, enable);

		if (r)
			return r;
	}

	td->mirror = enable;

	return 0;
}

static bool cosmo_panel_get_mirror(struct omap_dss_device *dssdev)
{
	struct cosmo_panel_data *td = dev_get_drvdata(&dssdev->dev);
	return td->mirror;
}

static int cosmo_panel_run_test(struct omap_dss_device *dssdev, int test_num)
{
	u8 buf[3];
	int r;

	r = dsi_vc_dcs_read(1,TCH, DCS_GET_ID, buf, 3);
	if (r)
		return r;

	return 0;
}


static void cosmo_panel_esd_work(struct work_struct *work)
{
	struct cosmo_panel_data *td = container_of(work, struct cosmo_panel_data,
			esd_work.work);
	struct omap_dss_device *dssdev = td->dssdev;
	u8 state1, state2;
	int r;

	if (!td->enabled)
		return;

	dsi_bus_lock(1);

	r = cosmo_panel_dcs_read_1(DCS_RDDSDR, &state1);
	if (r) {
		dev_err(&dssdev->dev, "failed to read Taal status\n");
		goto err;
	}

	/* Run self diagnostics */
	r = cosmo_panel_sleep_out(td);
	if (r) {
		dev_err(&dssdev->dev, "failed to run Taal self-diagnostics\n");
		goto err;
	}

	r = cosmo_panel_dcs_read_1(DCS_RDDSDR, &state2);
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
	dsi_bus_unlock(1);

	queue_delayed_work(td->esd_wq, &td->esd_work, COSMO_PANEL_ESD_CHECK_PERIOD);

	return;
err:
	dev_err(&dssdev->dev, "performing LCD reset\n");

	cosmo_panel_disable(dssdev);
	cosmo_panel_enable(dssdev);

	dsi_bus_unlock(1);

	queue_delayed_work(td->esd_wq, &td->esd_work, COSMO_PANEL_ESD_CHECK_PERIOD);
}

static struct omap_dss_driver cosmo_panel_driver = {
	.probe		= cosmo_panel_probe,
	.remove		= cosmo_panel_remove,

	.enable		= cosmo_panel_enable,
	.disable	= cosmo_panel_disable,
	.suspend	= cosmo_panel_suspend,
	.resume		= cosmo_panel_resume,

	.enable_te	= cosmo_panel_enable_te,
	.set_rotate	= cosmo_panel_rotate,
	.get_rotate	= cosmo_panel_get_rotate,
	.set_mirror	= cosmo_panel_mirror,
	.get_mirror	= cosmo_panel_get_mirror,
	.run_test	= cosmo_panel_run_test,
	.get_timings = cosmo_panel_get_timings,

	.enable_s3d	= cosmo_panel_enable_s3d,
	.get_s3d_enabled = cosmo_panel_get_s3d_enabled,
	.set_s3d_disp_type = cosmo_panel_set_s3d_disp_type,

	.driver         = {
#if defined(CONFIG_MACH_LGE_HUB_REV_A)
		.name   = "russo_panel",
#else
		.name 	= "cosmo_panel",
#endif
		.owner  = THIS_MODULE,
	},
};

static int __init cosmo_panel_init(void)
{
	printk("Trying to register driver\n");
	omap_dss_register_driver(&cosmo_panel_driver);
	printk("Registering driver done\n");

	return 0;
}

static void __exit cosmo_panel_exit(void)
{
	omap_dss_unregister_driver(&cosmo_panel_driver);
}

module_init(cosmo_panel_init);
module_exit(cosmo_panel_exit);

MODULE_AUTHOR("taekeun Kim <taekeun1.kim@lge.com>");
MODULE_DESCRIPTION("COSMO_PANEL lcd Driver");
MODULE_LICENSE("GPL");

