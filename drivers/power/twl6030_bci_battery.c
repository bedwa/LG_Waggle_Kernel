/*
 * linux/drivers/power/twl6030_bci_battery.c
 *
 * OMAP4:TWL6030 battery driver for Linux
 *
 * Copyright (C) 2008-2009 Texas Instruments, Inc.
 * Author: Texas Instruments, Inc.
 *
 * This package is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * THIS PACKAGE IS PROVIDED ``AS IS'' AND WITHOUT ANY EXPRESS OR
 * IMPLIED WARRANTIES, INCLUDING, WITHOUT LIMITATION, THE IMPLIED
 * WARRANTIES OF MERCHANTIBILITY AND FITNESS FOR A PARTICULAR PURPOSE.
 */

#include <linux/init.h>
#include <linux/module.h>
#include <linux/device.h>
#include <linux/interrupt.h>
#include <linux/delay.h>
#include <linux/platform_device.h>
#include <linux/slab.h>
#include <linux/i2c/twl.h>
#include <linux/power_supply.h>
#include <linux/i2c/twl6030-gpadc.h>
#include <linux/i2c/bq2415x.h>
#include <linux/mutex.h>
#include <linux/reboot.h>
#include <mach/gpio.h>

//roy.park@lge.com 2010.12.8 Start-->[
#include <linux/wakelock.h>
//roy.park@lge.com <---]
#include <plat/lge_nvdata_handler.h>


// LGE_CHANGE [jongho3.lee@lge.com] add driver headers. /start
#include <linux/cosmo/charger_rt9524.h>
#include <linux/cosmo/cosmo_muic.h>
// LGE_CHANGE [jongho3.lee@lge.com] add driver headers. /end
//LGE_CHANGE [jongho3.lee@lge.com] I moved register address list to i2c/twl.h
#include <plat/lge_nvdata_handler.h>	//[hyunhee.jeon@lge.com]

/// LGE_CHANGE [jongho3.lee@lge.com]  add INVALID_CAPACITY for gauge control. gauge reset .
#define RESET_CAP		198000
#define INVALID_CAP		199000
#define NOT_INIT_CAP	200000
#define USE_GAUGE_GRAPH_VALIDATION 1

#define RECHARGING_BAT_SOC_CON	97

#define CHARGING_WAKE_LOCK	1

#define TWL6030_PMIC_CHARGING  1
#define TWL6030_PMIC_FUELGAUGE  1

#define BAT_GAUGE_ZERO 3400
#define BAT_GAUGE_FULL 4200
#define TEMP_BAT_GAUTE_BY_VOLT 1
#define GET_BET_TEMP 1
#define BAT_TEMP_CHANNEL	2

#define CREATE_OWN_WORKQUEUE 1

#define CHR_TIMER_SECS		300

//#define TEMP_BAT_GAUTE_BY_VOLT2 1
//#define TWL6030_PMIC_ENABLE_CHARGING 1

// LGE_CHANGE_S [hyunhee.jeon@lge.com] 2011-02-07
/* access control values for PH_CFG_VBATLOW */
#define PH_CFG_VBATLOW_REG_OFFSET		0x9	/* PH_CFG_VBATLOWV REG ADDR: 0x28 */
#define BB_SEL					(1 << 7)
#define BB_MASK					(1 << 6)
// LGE_CHANGE_E [hyunhee.jeon@lge.com] 2011-02-07

//#if defined(TWL6030_PMIC_CHARGING)
//#if defined(TWL6030_PMIC_FUELGAUGE)

//#if !defined(CONFIG_LG_FW_RT9524_CHARGER)
//#if !defined(CONFIG_LGE_COSMO_MUIC)
//#if !defined(CONFIG_LG_FW_MAX17040_FUEL_GAUGE)
//

#define NO_DEBUG


#define UNLIMITED_TEMP_VAL 0xA4
//#define POWER_OFF_WHEN_NO_BAT 1

//################### global function ######################
static int twl6030_get_gpadc_conversion(int channel_no);
int move_queue_point(int point, int move_count, int queue_size, int limit);
void follow_fg_capacity(void);
//#######################################################

/* Ptr to thermistor table */
static const unsigned int fuelgauge_rate[4] = {4, 16, 64, 256};

#define AVG_VOLT_QUEUE_SIZE	16

struct twl6030_bci_device_info {
	struct device		*dev;
	int			*battery_tmp_tbl;
	unsigned int		tblsize;

	int			voltage_uV;
	int			fg_voltage_mV;
	int			bk_voltage_uV;
	int			current_uA;
	int			current_avg_uA;
	int			temp_C;
	int			charge_status;
	int			charger_source;
	int			vac_priority;
// LGE_CHANGE [jongho3.lee@lge.com] adding battery related variables.. [START_LGE]
	int 		capacity;
	int 		pmic_capacity;
	int 		dummy_cap;
	int 		capacity_validation;
	int			battery_present;
	int			charger_interrupt;
	int			charger_logo_status;
	int			valid_charging_source;
	////[jongho3.lee@lge.com] average volt.
	int 		avg_volt_queue[AVG_VOLT_QUEUE_SIZE];
	int 		avg_volt_queue_count;
	int 		avg_volt_head;
	int 		avg_volt_tail;
	int			avg_voltage_mV;
	int			gauge_control_count;
	u8			temp_control;
	// LGE_CHANGE [jongho3.lee@lge.com] adding battery related variables.. [END_LGE]
	int			bat_health;

	int			fuelgauge_mode;
	int			timer_n2;
	int			timer_n1;
	s32			charge_n1;
	s32			charge_n2;
	s16			cc_offset;
	u8			usb_online;
	u8			ac_online;
	u8			stat1;
	u8			status_int1;
	u8			status_int2;
	u8			smpl_en; /* LGE_CHANGE [hyunhee.jeon@lge.com] 0;disalbe 1:enable*/

	u8			watchdog_duration;
	u16			current_avg_interval;
	u16			monitoring_interval;
	unsigned int		min_vbus;
	unsigned int		max_charger_voltagemV;
	unsigned int		max_charger_currentmA;
	unsigned int		charger_incurrentmA;
	unsigned int		charger_outcurrentmA;
	unsigned int		regulation_voltagemV;
	unsigned int		low_bat_voltagemV;
	unsigned int		termination_currentmA;

	struct power_supply	bat;
	struct power_supply	usb;
	struct power_supply	ac;
	struct power_supply	bk_bat;
	struct delayed_work	twl6030_bci_monitor_work;
	struct delayed_work	twl6030_current_avg_work;
// LGE_CHANGE [jongho3.lee@lge.com] adding charger work. [START_LGE]
	struct delayed_work	charger_work;
	struct delayed_work	gauge_reset_work;
	struct delayed_work	power_off_work;
	struct delayed_work	charger_timer_work;

	struct timer_list off_mode_timer;
	int	   off_mode_timer_working;

#if defined(CHARGING_WAKE_LOCK)
	struct wake_lock charger_wake_lock;
	struct wake_lock off_mode_charger_wake_lock;
#if 0
	struct wake_lock charger_back_light_wake_lock;
#endif
	unsigned int wake_lock_count;
#endif

#if defined(CREATE_OWN_WORKQUEUE)
	struct workqueue_struct *charger_workqueue;
#endif


// LGE_CHANGE [jongho3.lee@lge.com] adding charger work. [END_LGE]
};
static struct blocking_notifier_head notifier_list;
//LGE_CHANGE [jongho3.lee@lge.com] expose device info for external charger.
struct twl6030_bci_device_info* p_di;
DECLARE_WAIT_QUEUE_HEAD(muic_event);
//LGE_CHANGE [jongho3.lee@lge.com] set recharging condition on this.
static recharging_state_t recharging_status;
//LGE_CHANGE [jongho3.lee@lge.com]
static DEFINE_MUTEX(charging_fsm_lock);
//static DEFINE_MUTEX(muic_detect_lock);
// LGE_CHANGE_S [jongho3.lee@lge.com] battery graphs...

#define STRT_ON_PLUG_DET            (1 << 3)
int start_cond;



typedef struct __battery_graph_prop
{
	s32	x;
	s32	y;
} battery_graph_prop;

#define SOC_TIMES 1000
#define VALID_RANGE  10

battery_graph_prop battery_soc_graph[] =
{
	//s32	(voltage, soc);
	{4200, 100 * SOC_TIMES},
	{4100,	98 * SOC_TIMES},
	{3800,  61 * SOC_TIMES},
	{3700,	34 * SOC_TIMES},
	{3620,	13 * SOC_TIMES},
	{3580,	5 * SOC_TIMES},
	{3300,	0 * SOC_TIMES},
};

battery_graph_prop battery_soc_graph_with_no_charger[] =
{
	//s32	(voltage, soc);
	{4170, 100 * SOC_TIMES},
	{4132,	97 * SOC_TIMES},
	{4058,	90 * SOC_TIMES},
	{3986,  80 * SOC_TIMES},
	{3909,	70 * SOC_TIMES},
	{3840,	60 * SOC_TIMES},
	{3783,	50 * SOC_TIMES},
	{3747,	40 * SOC_TIMES},
	{3726,	30 * SOC_TIMES},
	{3697,	20 * SOC_TIMES},
	{3644,	10 * SOC_TIMES},
	{3550,	0 * SOC_TIMES},
};

battery_graph_prop battery_soc_graph_with_ac[] =
{
	//s32	(voltage, soc);
	{4279, 100 * SOC_TIMES},
	{4220,	97 * SOC_TIMES},
	{4132,	90 * SOC_TIMES},
	{4112,  80 * SOC_TIMES},
	{4089,	70 * SOC_TIMES},
	{4038,	60 * SOC_TIMES},
	{3993,	50 * SOC_TIMES},
	{3967,	40 * SOC_TIMES},
	{3947,	30 * SOC_TIMES},
	{3910,	20 * SOC_TIMES},
	{3850,	10 * SOC_TIMES},
	{3680,	0 * SOC_TIMES},
};

battery_graph_prop battery_soc_graph_with_usb[] =
{
	//s32	(voltage, soc);
	{4250, 100 * SOC_TIMES},
	{4200,	97 * SOC_TIMES},
	{4123,	90 * SOC_TIMES},
	{4052,  80 * SOC_TIMES},
	{3968,	70 * SOC_TIMES},
	{3910,	60 * SOC_TIMES},
	{3854,	50 * SOC_TIMES},
	{3808,	40 * SOC_TIMES},
	{3768,	30 * SOC_TIMES},
	{3728,	20 * SOC_TIMES},
	{3688,	10 * SOC_TIMES},
	{3630,	0 * SOC_TIMES},
};

#define TEMP_TIMES 10000

battery_graph_prop battery_temp_graph[] =
{
	//s32	(voltage, soc);
	{1774, -50 * TEMP_TIMES},
	{1748, -40 * TEMP_TIMES},
	{1701, -30 * TEMP_TIMES},
	{1622, -20 * TEMP_TIMES},
	{1504, -10 * TEMP_TIMES},
	{1341, 0 * TEMP_TIMES},
	{1142, 10 * TEMP_TIMES},
	{928, 20 * TEMP_TIMES},
	{723, 30 * TEMP_TIMES},
	{545, 40 * TEMP_TIMES},
	{401, 50 * TEMP_TIMES},
	{292, 60 * TEMP_TIMES},
	{212, 70 * TEMP_TIMES},
	{154, 80 * TEMP_TIMES},
	{113, 90 * TEMP_TIMES},
	{83, 100 * TEMP_TIMES},

};

/////////////////////////////////////////// ksmin for temp average /////////////////////////////

int average_temp(int temp)
{
#define MAX_ABNORMAL_COUNT 2
	static int abnormal_temp_count = 0;
	static int old_temp = 200;
	int av_temp;

	if(temp > 600)
	{
		if( abnormal_temp_count < MAX_ABNORMAL_COUNT )
		{
			abnormal_temp_count++;
			av_temp = old_temp;
		}
		else
		{
			av_temp = temp;
		}
	}
	else
	{
		av_temp = temp;
		old_temp = temp;
		abnormal_temp_count = 0;
	}

	D("temp avg value %d\n",av_temp);

	return 	av_temp;
}
/////////////////////////////////////////// ksmin for temp average /////////////////////////////
int reference_graph(int __x, battery_graph_prop* ref_battery_graph, int arraysize)
{
	int i = 1;
	int __y = 0;
	int slope, const_term;
	int delta_y, delta_x;

	D(" battery graph array size = %d", arraysize );

	while( __x < ref_battery_graph[i].x \
		&& i < (arraysize - 1) )
	{
		i++;
	}

	delta_x = ref_battery_graph[i-1].x - ref_battery_graph[i].x;
	delta_y = (ref_battery_graph[i-1].y - ref_battery_graph[i].y);

	slope = delta_y  / delta_x;

	const_term = (ref_battery_graph[i].y) - (ref_battery_graph[i].x * slope);

	__y = (__x* slope + const_term);

	D(" ####### array_size = %d ##########", arraysize);
	D(" ##### SLOPE = %d, CONST_TERM = %d ##########", slope, const_term);
	D(" ##### CALCULATED __y = %d ##########", __y);

	if(ref_battery_graph[i-1].y > ref_battery_graph[i].y)
	{
		if(__y > ref_battery_graph[i-1].y)
		{
			__y = ref_battery_graph[i-1].y;
			D(" ##### fixing __y = %d ##########", __y);
		}
		else if(__y < ref_battery_graph[i].y)
		{
			__y = ref_battery_graph[i].y;
			D(" ##### fixing __y = %d ##########", __y);
		}
	}
	else
	{
		if(__y < ref_battery_graph[i-1].y)
		{
			__y = ref_battery_graph[i-1].y;
			D(" ##### fixing __y = %d ##########", __y);
		}
		else if(__y > ref_battery_graph[i].y)
		{
			__y = ref_battery_graph[i].y;
			D(" ##### fixing __y = %d ##########", __y);
		}
	}

	return __y;
}
// LGE_CHANGE_E [jongho3.lee@lge.com] battery graphs...
//

void charger_schedule_delayed_work(struct delayed_work *work, unsigned long delay)
{
#if defined(CREATE_OWN_WORKQUEUE)
	queue_delayed_work(p_di->charger_workqueue, work, delay);
#else
	schedule_delayed_work(work, delay);
#endif
}
EXPORT_SYMBOL(charger_schedule_delayed_work);

static void twl6030_work_interval_changed(struct twl6030_bci_device_info *di)
{
	cancel_delayed_work(&di->twl6030_bci_monitor_work);
	charger_schedule_delayed_work(&di->twl6030_bci_monitor_work,
		msecs_to_jiffies(1000 * di->monitoring_interval));
}

struct twl6030_bci_device_info *p_di;

static void twl6030_config_min_vbus_reg(struct twl6030_bci_device_info *di,
						unsigned int value)
{
	u8 rd_reg = 0;
	if (value > 4760 || value < 4200) {
		dev_dbg(di->dev, "invalid min vbus\n");
		return;
	}

	twl_i2c_read_u8(TWL6030_MODULE_CHARGER, &rd_reg, ANTICOLLAPSE_CTRL2);
	rd_reg = rd_reg & 0x1F;
	rd_reg = rd_reg | (((value - 4200)/80) << BUCK_VTH_SHIFT);
	twl_i2c_write_u8(TWL6030_MODULE_CHARGER, rd_reg, ANTICOLLAPSE_CTRL2);

	return;
}

static void twl6030_config_iterm_reg(struct twl6030_bci_device_info *di,
						unsigned int term_currentmA)
{
	if ((term_currentmA > 400) || (term_currentmA < 50)) {
		dev_dbg(di->dev, "invalid termination current\n");
		return;
	}

	term_currentmA = ((term_currentmA - 50)/50) << 5;
	twl_i2c_write_u8(TWL6030_MODULE_CHARGER, term_currentmA,
						CHARGERUSB_CTRL2);
	return;
}

static void twl6030_config_voreg_reg(struct twl6030_bci_device_info *di,
							unsigned int voltagemV)
{
	if ((voltagemV < 3500) || (voltagemV > 4760)) {
		dev_dbg(di->dev, "invalid charger_voltagemV\n");
		return;
	}

	voltagemV = (voltagemV - 3500) / 20;
	twl_i2c_write_u8(TWL6030_MODULE_CHARGER, voltagemV,
						CHARGERUSB_VOREG);
	return;
}

#if defined(TWL6030_PMIC_CHARGING)
// we use exteranl charger.
static void twl6030_config_vichrg_reg(struct twl6030_bci_device_info *di,
							unsigned int currentmA)
{
	if ((currentmA >= 300) && (currentmA <= 450))
		currentmA = (currentmA - 300) / 50;
	else if ((currentmA >= 500) && (currentmA <= 1500))
		currentmA = (currentmA - 500) / 100 + 4;
	else {
		dev_dbg(di->dev, "invalid charger_currentmA\n");
		return;
	}

	twl_i2c_write_u8(TWL6030_MODULE_CHARGER, currentmA,
						CHARGERUSB_VICHRG);
	return;
}
#endif

static void twl6030_config_cinlimit_reg(struct twl6030_bci_device_info *di,
							unsigned int currentmA)
{
	if ((currentmA >= 50) && (currentmA <= 750))
		currentmA = (currentmA - 50) / 50;
	else if (currentmA >= 750)
		currentmA = (800 - 50) / 50;
	else {
		dev_dbg(di->dev, "invalid input current limit\n");
		return;
	}

	twl_i2c_write_u8(TWL6030_MODULE_CHARGER, currentmA,
					CHARGERUSB_CINLIMIT);
	return;
}

static void twl6030_config_limit1_reg(struct twl6030_bci_device_info *di,
							unsigned int voltagemV)
{
	if ((voltagemV < 3500) || (voltagemV > 4760)) {
		dev_dbg(di->dev, "invalid max_charger_voltagemV\n");
		return;
	}

	voltagemV = (voltagemV - 3500) / 20;
	twl_i2c_write_u8(TWL6030_MODULE_CHARGER, voltagemV,
						CHARGERUSB_CTRLLIMIT1);
	return;
}

static void twl6030_config_limit2_reg(struct twl6030_bci_device_info *di,
							unsigned int currentmA)
{
	if ((currentmA >= 300) && (currentmA <= 450))
		currentmA = (currentmA - 300) / 50;
	else if ((currentmA >= 500) && (currentmA <= 1500))
		currentmA = (currentmA - 500) / 100 + 4;
	else {
		dev_dbg(di->dev, "invalid max_charger_currentmA\n");
		return;
	}

	currentmA |= LOCK_LIMIT;
	twl_i2c_write_u8(TWL6030_MODULE_CHARGER, currentmA,
						CHARGERUSB_CTRLLIMIT2);
	return;
}

#if defined(TWL6030_PMIC_ENABLE_CHARGING)
#if 0
// we use exteranl charger.
static void twl6030_stop_usb_charger(struct twl6030_bci_device_info *di)
{
	di->charger_source = 0;
	di->charge_status = POWER_SUPPLY_STATUS_DISCHARGING;
	twl_i2c_write_u8(TWL6030_MODULE_CHARGER, 0, CONTROLLER_CTRL1);
}

static void twl6030_start_usb_charger(struct twl6030_bci_device_info *di)
{
	di->charger_source = POWER_SUPPLY_TYPE_USB;
	di->charge_status = POWER_SUPPLY_STATUS_CHARGING;
	dev_dbg(di->dev, "USB charger detected\n");
	twl6030_config_vichrg_reg(di, di->charger_outcurrentmA);
	twl6030_config_cinlimit_reg(di, di->charger_incurrentmA);
	twl6030_config_voreg_reg(di, di->regulation_voltagemV);
	twl6030_config_iterm_reg(di, di->termination_currentmA);
	twl_i2c_write_u8(TWL6030_MODULE_CHARGER,
			CONTROLLER_CTRL1_EN_CHARGER,
			CONTROLLER_CTRL1);
}

static void twl6030_stop_ac_charger(struct twl6030_bci_device_info *di)
{
	long int events;
	di->charger_source = 0;
	di->charge_status = POWER_SUPPLY_STATUS_DISCHARGING;
	events = BQ2415x_STOP_CHARGING;
	blocking_notifier_call_chain(&notifier_list, events, NULL);
	twl_i2c_write_u8(TWL6030_MODULE_CHARGER, 0, CONTROLLER_CTRL1);
}

static void twl6030_start_ac_charger(struct twl6030_bci_device_info *di)
{
	long int events;

	dev_dbg(di->dev, "AC charger detected\n");
	di->charger_source = POWER_SUPPLY_TYPE_MAINS;
	di->charge_status = POWER_SUPPLY_STATUS_CHARGING;
	events = BQ2415x_START_CHARGING;
	blocking_notifier_call_chain(&notifier_list, events, NULL);
	twl_i2c_write_u8(TWL6030_MODULE_CHARGER,
			CONTROLLER_CTRL1_EN_CHARGER |
			CONTROLLER_CTRL1_SEL_CHARGER,
			CONTROLLER_CTRL1);
}

static void twl6030_stop_charger(struct twl6030_bci_device_info *di)
{
	if (di->charger_source == POWER_SUPPLY_TYPE_MAINS)
		twl6030_stop_ac_charger(di);
	else if (di->charger_source == POWER_SUPPLY_TYPE_USB)
		twl6030_stop_usb_charger(di);
}
#endif
#endif


/* LGE_CHANGE [jongho3.lee@lge.com]
 * Since we are using external charger and fuel gauge,
 * we have to disable PMIC charger and gauge.
 * These functions will control external charger and it's logic.
 */
//LGE_CHANGE [jongho3.lee@lge.com]

int get_bat_soc(void)
{
	// kibum.lee@lge.com
	if (p_di == NULL)
		return 0;
	return p_di->capacity;
}
EXPORT_SYMBOL(get_bat_soc);

int get_bat_volt(void)
{
	// kibum.lee@lge.com
	if (p_di == NULL)
		return 0;
	return p_di->fg_voltage_mV;
}
EXPORT_SYMBOL(get_bat_volt);


struct delayed_work* get_charger_work(void)
{
	return &(p_di->charger_work);
}
EXPORT_SYMBOL(get_charger_work);


int is_tbat_good(int temp)
{
	if( (temp > (TEMP_LOW_DISCHARGING) && temp < (TEMP_HIGH_DISCHARGING)) \
		|| ( temp <= (TEMP_LOW_NO_BAT)) || (p_di->temp_control == UNLIMITED_TEMP_VAL ) )
	{
		return true;
	}
	return false;
}

//LGE_CHANGE [jongho3.lee@lge.com] charging finite state machine

void charger_fsm(charger_fsm_cause fsm_cause)
{

	TYPE_CHARGING_MODE charging_mode = CHARGING_UNKNOWN;
	u8 chargerusb_int_status;
	u8 controller_stat, controller_status_1;
	extern atomic_t muic_charger_detected;
	int ret, old_charge_status, old_valid_charging_source;

	//int timeout;
	old_charge_status = p_di->charge_status;
	old_valid_charging_source = p_di->valid_charging_source;

	D(" ##### CHARGING FSM ##########\n");
	if(fsm_cause == CHARG_FSM_CAUSE_CHARGING_TIMER_EXPIRED)
	{
		D("charging timer expired!!!");
	}
#if 1
	twl_i2c_read_u8(TWL6030_MODULE_CHARGER, &chargerusb_int_status,
		CHARGERUSB_INT_STATUS);

	//[jongho3.lee@lge.com] vbus detection from charger.
	D(" di->charger_source = %d", p_di->charger_source);
	//D(" vbus = %d", present_charge_state  & CHARGERUSB_STAT);
#endif

	mutex_lock(&charging_fsm_lock);
	//[jongho3.lee@lge.com] make it sure that muic examine the charger type.
	//

#if 0 // this is for wait event which is not unsing just now. block it temporary..
	if(p_di->charger_interrupt )
	{
		//[jongho3.lee@lge.com] make it sure that muic examine the charger type.
		////[jongho3.lee@lge.com] in kernel muic doesn't work now..
		D("START : wait muic event...");
		p_di->charger_interrupt = false;
		//[jongho3.lee] wait event stops scheculing... don't use this and find the reason..
		//wait_event_interruptible_timeout(muic_event, get_muic_charger_detected(), HZ*5);
		//wait_event(muic_event, get_muic_charger_detected());
		//wait_event_timeout
		D("wait_event timeout = %d", timeout);
		//wait_event_timeout(muic_event, muic_charger_detected.counter == 1, 2*HZ );
		//msleep_interruptible(1000);
		D("END : wait muic event...");
		//get_muic_charger_detected();
	}
#endif
	charging_mode = get_muic_mode();

	D("muic mode : %d", charging_mode);


	if(charging_mode == CHARGING_NA_TA || charging_mode == CHARGING_LG_TA || charging_mode == CHARGING_TA_1A)
	{
		p_di->charger_source = POWER_SUPPLY_TYPE_MAINS;
#if defined(CHARGING_WAKE_LOCK)
		if(!(p_di->wake_lock_count))
		{
			wake_lock(&(p_di->charger_wake_lock));
			p_di->wake_lock_count = 1;
		}
#endif
	}
	else if( charging_mode == CHARGING_USB )
	{
		p_di->charger_source = POWER_SUPPLY_TYPE_USB;
#if defined(CHARGING_WAKE_LOCK)
		if(!(p_di->wake_lock_count))
		{
			wake_lock(&(p_di->charger_wake_lock));
			p_di->wake_lock_count = 1;
		}
#endif
	}
	else if( charging_mode == MUIC_UNKNOWN || charging_mode == MUIC_NONE)
	{
		p_di->charger_source = POWER_SUPPLY_TYPE_BATTERY;
#if defined(CHARGING_WAKE_LOCK)
		if((p_di->wake_lock_count))
		{
			wake_unlock(&(p_di->charger_wake_lock));
			p_di->wake_lock_count = 0;
		}
#endif
	}
	else
	{
		p_di->charger_source = POWER_SUPPLY_TYPE_FACTROY;
#if defined(CHARGING_WAKE_LOCK)
		if(!(p_di->wake_lock_count))
		{
			wake_lock(&(p_di->charger_wake_lock));
			p_di->wake_lock_count = 1;
		}
#endif
	}

	ret = twl_i2c_read_u8(TWL6030_MODULE_CHARGER, &controller_status_1, CONTROLLER_STAT1);

	if (ret) {
		D("[charger_rt9524]:: fail to read  CONTROLLER_STAT1 in %s !! \n", __func__);
		charging_ic_deactive();
		recharging_status = RECHARGING_WAIT_UNSET;
		p_di->charge_status = POWER_SUPPLY_STATUS_DISCHARGING;
		return;
	}

	p_di->valid_charging_source = controller_status_1 & CONTROLLER_STAT1_VBUS_DET;
	D("[charger_rt9524]:: valid charger_source=%d !! \n", p_di->valid_charging_source);

	/////////////////////////////////////////////////////////
	// [jongho3.lee@lge.com] CHECKGIN BATTERY PRESENCE  start.
	// ///////////////////////////////////////////////////////

	if( !(p_di->battery_present) )
	{
		recharging_status = RECHARGING_WAIT_UNSET;
		D("[charger_rt9524]:: NO BATTERY charger_logo_status=%d, charger_source=%d !! \n", p_di->charger_logo_status, p_di->charger_source);
		if(!(p_di->charger_source == POWER_SUPPLY_TYPE_FACTROY))
		{
			if( (p_di->charger_logo_status == CHARGER_LOGO_STATUS_END) )
			{
				//FIXME: I need normal turn off process.
				charging_ic_deactive();
				p_di->charge_status = POWER_SUPPLY_STATUS_DISCHARGING;
			}
			else if(p_di->charger_logo_status == CHARGER_LOGO_STATUS_STARTED)
			{
				if(p_di->valid_charging_source)
				{
#if defined(POWER_OFF_WHEN_NO_BAT)
					charger_schedule_delayed_work(&p_di->power_off_work, HZ*2);
#else
					charger_schedule_delayed_work(&p_di->power_off_work, 0);
#endif
				}
			}
			else
			{
				//do nothing.
			}
			goto UPDATE_UI;
		}
	}
	/////////////////////////////////////////////////////////
	// [jongho3.lee@lge.com] CHECKGIN BATTERY PRESENCE  end.
	// ///////////////////////////////////////////////////////

	/////////////////////////////////////////////////////////
	// [jongho3.lee@lge.com] DISCHARGING CONDITION CHECK start.
	// ///////////////////////////////////////////////////////

	if(!(p_di->valid_charging_source))
	{
		/* we need check over voltage, battery temperature condition here */
		charging_ic_deactive();
		recharging_status = RECHARGING_WAIT_UNSET;
		p_di->charge_status = POWER_SUPPLY_STATUS_DISCHARGING;
		p_di->charger_source = POWER_SUPPLY_TYPE_BATTERY;

#if defined(CHARGING_WAKE_LOCK)
		if((p_di->wake_lock_count))
		{
			wake_unlock(&(p_di->charger_wake_lock));
			p_di->wake_lock_count = 0;
		}
#endif

		goto UPDATE_UI;
	}

	// recharging status check!!
	if( p_di->capacity > 99 && !(p_di->charger_source == POWER_SUPPLY_TYPE_FACTROY))
	{
		charging_ic_deactive();
		if(p_di->valid_charging_source)
		{
			recharging_status = RECHARGING_WAIT_SET;
		}
		else
		{
			recharging_status = RECHARGING_WAIT_UNSET;
		}
		p_di->charge_status = POWER_SUPPLY_STATUS_FULL;

		if(p_di->charger_logo_status == CHARGER_LOGO_STATUS_STARTED)
		{
			if((p_di->off_mode_timer_working))
			{
				del_timer(&(p_di->off_mode_timer));
				p_di->off_mode_timer_working = 0;
			}
		}

		goto UPDATE_UI;
	}
	else if(p_di->charge_status == POWER_SUPPLY_STATUS_FULL)
	{
		p_di->charge_status = POWER_SUPPLY_STATUS_DISCHARGING;
	}

	if( (p_di->charger_source == POWER_SUPPLY_TYPE_BATTERY) || !is_tbat_good(p_di->temp_C)/* charging deactive condition */ )
	{
		/* we need check over voltage, battery temperature condition here */
		D("[charger_rt9524]:: bat_temp=%d !! \n", p_di->temp_C);
		charging_ic_deactive();
		recharging_status = RECHARGING_WAIT_UNSET;
		p_di->charge_status = POWER_SUPPLY_STATUS_DISCHARGING;

		goto UPDATE_UI;
	}

	if(fsm_cause == CHARG_FSM_CAUSE_CHARGING_TIMER_EXPIRED)
	{
		D("charging stop since...timer expired!!!");
		charging_ic_deactive();
		recharging_status = RECHARGING_WAIT_UNSET;
		p_di->charge_status = POWER_SUPPLY_STATUS_DISCHARGING;

		goto UPDATE_UI;
	}

/////////////////////////////////////////////////////////
// [jongho3.lee@lge.com] DISCHARGING CONDITION CHECK end.
/////////////////////////////////////////////////////////

	// [jongho3.lee@lge.com] FIXME : recharging start soc should be modified with H/W team.
	if( recharging_status == RECHARGING_WAIT_SET && p_di->capacity > RECHARGING_BAT_SOC_CON)
	{
		goto UPDATE_UI;
	}

	recharging_status = RECHARGING_WAIT_UNSET;

	if( p_di->charger_source == POWER_SUPPLY_TYPE_MAINS)
	{
		charging_ic_set_ta_mode();
		p_di->charge_status = POWER_SUPPLY_STATUS_CHARGING;
	}
	else if( p_di->charger_source == POWER_SUPPLY_TYPE_USB)
	{
		charging_ic_active_default();
		p_di->charge_status = POWER_SUPPLY_STATUS_CHARGING;
	}
	else if( p_di->charger_source == POWER_SUPPLY_TYPE_FACTROY)
	{
		charging_ic_set_factory_mode();
		p_di->charge_status = POWER_SUPPLY_STATUS_CHARGING;
	}
	else
	{
		charging_ic_deactive();
		p_di->charge_status = POWER_SUPPLY_STATUS_DISCHARGING;
	}

UPDATE_UI:
	power_supply_changed(&p_di->bat);

#if 1
	if(old_charge_status != p_di->charge_status)
	{
		//charger is changed..  gaterh voltage data again.
		{
			p_di->avg_volt_head = move_queue_point(p_di->avg_volt_head, AVG_VOLT_QUEUE_SIZE, AVG_VOLT_QUEUE_SIZE, p_di->avg_volt_tail);
			p_di->avg_volt_queue_count = 0;
		}

		if(p_di->charge_status == POWER_SUPPLY_STATUS_CHARGING)
		{
			p_di->monitoring_interval = 2;   //sec
			twl6030_work_interval_changed(p_di);
		}
		else
		{
			p_di->monitoring_interval = 10;    //sec
			twl6030_work_interval_changed(p_di);
		}

	}
#endif

#if 0
	if(!(p_di->valid_charging_source) && old_valid_charging_source)
	{
		wake_lock_timeout(&p_di->charger_back_light_wake_lock, HZ*3);
	}
#endif

	mutex_unlock(&charging_fsm_lock);

	return;
}
EXPORT_SYMBOL(charger_fsm);



/*
 * Interrupt service routine
 *
 * Attends to TWL 6030 power module interruptions events, specifically
 * USB_PRES (USB charger presence) CHG_PRES (AC charger presence) events
 *
 */
static irqreturn_t twl6030charger_ctrl_interrupt(int irq, void *_di)
{
	struct twl6030_bci_device_info *di = _di;
	int ret;
	int charger_fault = 0;
#if defined(TWL6030_PMIC_CHARGING)
	long int events;
#endif
	u8 stat_toggle, stat_reset, stat_set = 0;
	u8 charge_state;
	u8 present_charge_state,chargerusb_int_status;
#if defined(TWL6030_PMIC_CHARGING)
	u8 ac_or_vbus, no_ac_and_vbus;
#endif

	/* read charger controller_stat1 */
	ret = twl_i2c_read_u8(TWL6030_MODULE_CHARGER, &present_charge_state,
		CONTROLLER_STAT1);
	if (ret)
		return IRQ_NONE;

	//[jongho3.lee@lge.com] vbus detection from charger.

	twl_i2c_read_u8(TWL6030_MODULE_CHARGER, &chargerusb_int_status,
		CHARGERUSB_INT_STATUS);
#if 1
	if( (di->charger_source && !(present_charge_state & VBUS_DET)) \
		|| (!(di->charger_source) && (present_charge_state & VBUS_DET)) )
#else
	if(chargerusb_int_status & CHARGERUSB_STAT)
#endif
	{
		di->charger_interrupt = true;
	}
	else
	{
		di->charger_interrupt = false;
	}

	charge_state = di->stat1;

	stat_toggle = charge_state ^ present_charge_state;
	stat_set = stat_toggle & present_charge_state;
	stat_reset = stat_toggle & charge_state;

#if defined(TWL6030_PMIC_ENABLE_CHARGING)
#if 0
	no_ac_and_vbus = !((present_charge_state) & (VBUS_DET | VAC_DET));
	ac_or_vbus = charge_state & (VBUS_DET | VAC_DET);
	if (no_ac_and_vbus && ac_or_vbus) {
		di->charger_source = 0;
		dev_dbg(di->dev, "No Charging source\n");
		/* disable charging when no source present */
	}

	charge_state = present_charge_state;
	di->stat1 = present_charge_state;
	if ((charge_state & VAC_DET) &&
		(charge_state & CONTROLLER_STAT1_EXTCHRG_STATZ)) {
		events = BQ2415x_CHARGER_FAULT;
		blocking_notifier_call_chain(&notifier_list, events, NULL);
	}

	if (stat_reset & VBUS_DET) {
		di->usb_online = 0;
		dev_dbg(di->dev, "usb removed\n");
		twl6030_stop_usb_charger(di);
		if (present_charge_state & VAC_DET)
			twl6030_start_ac_charger(di);

	}
	if (stat_set & VBUS_DET) {
		di->usb_online = POWER_SUPPLY_TYPE_USB;
		if ((present_charge_state & VAC_DET) && (di->vac_priority == 2))
			dev_dbg(di->dev,
				"USB charger detected, continue with VAC\n");
		else
			twl6030_start_usb_charger(di);
	}

	if (stat_reset & VAC_DET) {
		di->ac_online = 0;
		dev_dbg(di->dev, "vac removed\n");
		twl6030_stop_ac_charger(di);
		if (present_charge_state & VBUS_DET)
			twl6030_start_usb_charger(di);
	}
	if (stat_set & VAC_DET) {
		di->ac_online = POWER_SUPPLY_TYPE_MAINS;
		if ((present_charge_state & VBUS_DET) &&
						(di->vac_priority == 3))
			dev_dbg(di->dev,
				"AC charger detected, continue with VBUS\n");
		else
			twl6030_start_ac_charger(di);
	}
#endif
#endif



	if (stat_set & CONTROLLER_STAT1_FAULT_WDG) {
		charger_fault = 1;
		dev_dbg(di->dev, "Fault watchdog fired\n");
	}
	if (stat_reset & CONTROLLER_STAT1_FAULT_WDG)
		dev_dbg(di->dev, "Fault watchdog recovered\n");
	if (stat_set & CONTROLLER_STAT1_BAT_REMOVED)
		dev_dbg(di->dev, "Battery removed\n");
	if (stat_reset & CONTROLLER_STAT1_BAT_REMOVED)
		dev_dbg(di->dev, "Battery inserted\n");
	if (stat_set & CONTROLLER_STAT1_BAT_TEMP_OVRANGE)
		dev_dbg(di->dev, "Battery temperature overrange\n");
	if (stat_reset & CONTROLLER_STAT1_BAT_TEMP_OVRANGE)
		dev_dbg(di->dev, "Battery temperature within range\n");

#if defined(TWL6030_PMIC_ENABLE_CHARGING)
	if (charger_fault) {
		twl6030_stop_usb_charger(di);
		di->charge_status = POWER_SUPPLY_STATUS_NOT_CHARGING;
		dev_err(di->dev, "Charger Fault stop charging\n");
	}
#endif


#if defined(CONFIG_LG_FW_RT9524_CHARGER)
	if(!(di->charger_interrupt))
	{
		charger_schedule_delayed_work(&di->charger_work, 0);
	}
	//charger_fsm();
#endif

//	power_supply_changed(&di->bat);

	return IRQ_HANDLED;
}

//FIXME  : we need to fix this.
#if defined(TWL6030_PMIC_ENABLE_CHARGING)
static irqreturn_t twl6030charger_fault_interrupt(int irq, void *_di)
{
	struct twl6030_bci_device_info *di = _di;
	int charger_fault = 0;
	int ret;

	u8 usb_charge_sts, usb_charge_sts1, usb_charge_sts2;

	ret = twl_i2c_read_u8(TWL6030_MODULE_CHARGER, &usb_charge_sts,
						CHARGERUSB_INT_STATUS);
	ret = twl_i2c_read_u8(TWL6030_MODULE_CHARGER, &usb_charge_sts1,
						CHARGERUSB_STATUS_INT1);
	ret = twl_i2c_read_u8(TWL6030_MODULE_CHARGER, &usb_charge_sts2,
						CHARGERUSB_STATUS_INT2);

	di->status_int1 = usb_charge_sts1;
	di->status_int2 = usb_charge_sts2;
	if (usb_charge_sts & CURRENT_TERM_INT)
		dev_dbg(di->dev, "USB CURRENT_TERM_INT\n");
	if (usb_charge_sts & CHARGERUSB_THMREG)
		dev_dbg(di->dev, "USB CHARGERUSB_THMREG\n");
	if (usb_charge_sts & CHARGERUSB_FAULT)
		dev_dbg(di->dev, "USB CHARGERUSB_FAULT\n");

	if (usb_charge_sts1 & CHARGERUSB_STATUS_INT1_TMREG)
		dev_dbg(di->dev, "USB CHARGER Thermal regulation activated\n");
	if (usb_charge_sts1 & CHARGERUSB_STATUS_INT1_NO_BAT)
		dev_dbg(di->dev, "No Battery Present\n");
	if (usb_charge_sts1 & CHARGERUSB_STATUS_INT1_BST_OCP)
		dev_dbg(di->dev, "USB CHARGER Boost Over current protection\n");
	if (usb_charge_sts1 & CHARGERUSB_STATUS_INT1_TH_SHUTD) {
		charger_fault = 1;
		dev_dbg(di->dev, "USB CHARGER Thermal Shutdown\n");
	}
	if (usb_charge_sts1 & CHARGERUSB_STATUS_INT1_BAT_OVP)
		dev_dbg(di->dev, "USB CHARGER Bat Over Voltage Protection\n");
	if (usb_charge_sts1 & CHARGERUSB_STATUS_INT1_POOR_SRC)
		dev_dbg(di->dev, "USB CHARGER Poor input source\n");
	if (usb_charge_sts1 & CHARGERUSB_STATUS_INT1_SLP_MODE)
		dev_dbg(di->dev, "USB CHARGER Sleep mode\n");
	if (usb_charge_sts1 & CHARGERUSB_STATUS_INT1_VBUS_OVP)
		dev_dbg(di->dev, "USB CHARGER VBUS over voltage\n");

	if (usb_charge_sts2 & CHARGE_DONE) {
		di->charge_status = POWER_SUPPLY_STATUS_FULL;
		dev_dbg(di->dev, "USB charge done\n");
	}
	if (usb_charge_sts2 & CURRENT_TERM)
		dev_dbg(di->dev, "USB CURRENT_TERM\n");
	if (usb_charge_sts2 & ICCLOOP)
		dev_dbg(di->dev, "USB ICCLOOP\n");
	if (usb_charge_sts2 & ANTICOLLAPSE)
		dev_dbg(di->dev, "USB ANTICOLLAPSE\n");

#if defined(TWL6030_PMIC_ENABLE_CHARGING)
	if (charger_fault) {
		twl6030_stop_usb_charger(di);
		di->charge_status = POWER_SUPPLY_STATUS_NOT_CHARGING;
		dev_err(di->dev, "Charger Fault stop charging\n");
	}
#endif
	dev_dbg(di->dev, "Charger fault detected STS, INT1, INT2 %x %x %x\n",
	    usb_charge_sts, usb_charge_sts1, usb_charge_sts2);

	power_supply_changed(&di->bat);

	return IRQ_HANDLED;
}
#endif

#if defined(TWL6030_PMIC_CHARGING)
//LGE_CHANGE [jongho3.lee@lge.com] we don't need and don't have the way to monitor bat_current.
static void twl6030battery_current(struct twl6030_bci_device_info *di)
{
	int ret;
	u16 read_value;
	s16 temp;
	int current_now;

	/* FG_REG_10, 11 is 14 bit signed instantaneous current sample value */
	ret = twl_i2c_read(TWL6030_MODULE_GASGAUGE, (u8 *)&read_value,
								FG_REG_10, 2);
	if (ret < 0) {
		dev_dbg(di->dev, "failed to read FG_REG_10: current_now\n");
		return;
	}

	temp = ((s16)(read_value << 2) >> 2);
	current_now = temp - di->cc_offset;

	/* current drawn per sec */
	current_now = current_now * fuelgauge_rate[di->fuelgauge_mode];
	/* current in mAmperes */
	current_now = (current_now * 3000) >> 14;
	/* current in uAmperes */
	current_now = current_now * 1000;
	di->current_uA = current_now;

	return;
}
#endif

/*
 * Return channel value
 * Or < 0 on failure.
 */
static int twl6030_get_gpadc_conversion(int channel_no)
{
	struct twl6030_gpadc_request req;
	int temp = 0;
	int ret;

	req.channels = (1 << channel_no);
	req.method = TWL6030_GPADC_SW2;
	req.active = 0;
	req.func_cb = NULL;
	ret = twl6030_gpadc_conversion(&req);
	if (ret < 0)
		return ret;

	if (req.rbuf[channel_no] > 0)
		temp = req.rbuf[channel_no];

	return temp;
}

/*
 * Setup the twl6030 BCI module to enable backup
 * battery charging.
 */
static int twl6030backupbatt_setup(void)
{
	int ret = 0;
	u8 rd_reg = 0;

        /*
         * add_by lu.guo@lge.com  we should enable VBAT work at low power mode
         * when we take out  the main battery
         */

	//ret = twl_i2c_read_u8(TWL6030_MODULE_ID0, &rd_reg, BBSPOR_CFG);
	rd_reg |= BB_CHG_EN | VRTC_EN_SLP_STS | VRTC_EN_OFF_STS | BB_SEL_2V6;
	ret |= twl_i2c_write_u8(TWL6030_MODULE_ID0, rd_reg, BBSPOR_CFG);

	return ret;
}

/*
 * Setup the twl6030 BCI module to measure battery
 * temperature
 */
static int twl6030battery_temp_setup(void)
{
	int ret;
	u8 rd_reg;

	ret = twl_i2c_read_u8(TWL_MODULE_MADC, &rd_reg, TWL6030_GPADC_CTRL);
#if defined(TWL6030_PMIC_ENABLE_CHARGING)
	rd_reg |= GPADC_CTRL_TEMP1_EN | GPADC_CTRL_TEMP2_EN |
		GPADC_CTRL_TEMP1_EN_MONITOR | GPADC_CTRL_TEMP2_EN_MONITOR |
		GPADC_CTRL_SCALER_DIV4;
#endif
//LGECHANGE [jongho3.lee@lge.com] batttery temp is connected to GPADCIN2.
	rd_reg |= GPADC_CTRL_SCALER_EN | GPADC_CTRL_SCALER_DIV4;
	ret |= twl_i2c_write_u8(TWL_MODULE_MADC, rd_reg, TWL6030_GPADC_CTRL);

	return ret;
}

static int twl6030battery_voltage_setup(void)
{
	int ret;
	u8 rd_reg;

	ret = twl_i2c_read_u8(TWL6030_MODULE_ID0, &rd_reg, REG_MISC1);
	rd_reg = rd_reg | VAC_MEAS | VBAT_MEAS | BB_MEAS;
	ret |= twl_i2c_write_u8(TWL6030_MODULE_ID0, rd_reg, REG_MISC1);

	ret |= twl_i2c_read_u8(TWL_MODULE_USB, &rd_reg, REG_USB_VBUS_CTRL_SET);
	rd_reg = rd_reg | VBUS_MEAS;
	ret |= twl_i2c_write_u8(TWL_MODULE_USB, rd_reg, REG_USB_VBUS_CTRL_SET);

	ret |= twl_i2c_read_u8(TWL_MODULE_USB, &rd_reg, REG_USB_ID_CTRL_SET);
	rd_reg = rd_reg | ID_MEAS;
	ret |= twl_i2c_write_u8(TWL_MODULE_USB, rd_reg, REG_USB_ID_CTRL_SET);

	return ret;
}

#if defined(TWL6030_PMIC_CHARGING)
//LGE_CHANGE [jongho3.lee@lge.com] we don't need and don't have the way to monitor bat_current.
static int twl6030battery_current_setup(void)
{
	int ret;
	u8 rd_reg;

	ret = twl_i2c_read_u8(TWL6030_MODULE_ID1, &rd_reg, PWDNSTATUS2);
	rd_reg = (rd_reg & 0x30) >> 2;
	rd_reg = rd_reg | FGDITHS | FGS;
	ret |= twl_i2c_write_u8(TWL6030_MODULE_ID1, rd_reg, REG_TOGGLE1);
	ret |= twl_i2c_write_u8(TWL6030_MODULE_GASGAUGE, CC_CAL_EN, FG_REG_00);

	return ret;
}
#endif

static enum power_supply_property twl6030_bci_battery_props[] = {
	POWER_SUPPLY_PROP_STATUS,
	POWER_SUPPLY_PROP_HEALTH,
	POWER_SUPPLY_PROP_PRESENT,
	POWER_SUPPLY_PROP_ONLINE,
	POWER_SUPPLY_PROP_TECHNOLOGY,
	POWER_SUPPLY_PROP_VOLTAGE_NOW,
	POWER_SUPPLY_PROP_CURRENT_NOW,
	POWER_SUPPLY_PROP_CURRENT_AVG,
	POWER_SUPPLY_PROP_CAPACITY,
	POWER_SUPPLY_PROP_TEMP,
	POWER_SUPPLY_PROP_PMIC_SOC,
	POWER_SUPPLY_PROP_GAUGE_VOLTAGE,
	POWER_SUPPLY_PROP_GAUGE_CONTROL,
	POWER_SUPPLY_PROP_GAUGE_CONTROL_COUNT,
	POWER_SUPPLY_PROP_CHARGER_MODE,
	POWER_SUPPLY_PROP_CHARGER_TEMP_CONTROL,
// LGE_CHANGE [jongho3.lee@lge.com] adding android related attributes to battery props. //end.
};

static enum power_supply_property twl6030_usb_props[] = {
	POWER_SUPPLY_PROP_ONLINE,
	POWER_SUPPLY_PROP_VOLTAGE_NOW,
};

static enum power_supply_property twl6030_ac_props[] = {
	POWER_SUPPLY_PROP_ONLINE,
	POWER_SUPPLY_PROP_VOLTAGE_NOW,
};

static enum power_supply_property twl6030_bk_bci_battery_props[] = {
	POWER_SUPPLY_PROP_VOLTAGE_NOW,
};

#if defined(TWL6030_PMIC_CHARGING)
//LGE_CHANGE [jongho3.lee@lge.com] we don't need and don't have the way to monitor bat_current.
static void twl6030_current_avg(struct work_struct *work)
{
	s32 samples;
	s16 cc_offset;
	int current_avg_uA;
	struct twl6030_bci_device_info *di = container_of(work,
		struct twl6030_bci_device_info,
		twl6030_current_avg_work.work);

	di->charge_n2 = di->charge_n1;
	di->timer_n2 = di->timer_n1;

	/* FG_REG_01, 02, 03 is 24 bit unsigned sample counter value */
	twl_i2c_read(TWL6030_MODULE_GASGAUGE, (u8 *) &di->timer_n1,
							FG_REG_01, 3);
	/*
	 * FG_REG_04, 5, 6, 7 is 32 bit signed accumulator value
	 * accumulates instantaneous current value
	 */
	twl_i2c_read(TWL6030_MODULE_GASGAUGE, (u8 *) &di->charge_n1,
							FG_REG_04, 4);
	/* FG_REG_08, 09 is 10 bit signed calibration offset value */
	twl_i2c_read(TWL6030_MODULE_GASGAUGE, (u8 *) &cc_offset,
							FG_REG_08, 2);
	cc_offset = ((s16)(cc_offset << 6) >> 6);
	di->cc_offset = cc_offset;

	samples = di->timer_n1 - di->timer_n2;
	/* check for timer overflow */
	if (di->timer_n1 < di->timer_n2)
		samples = samples + (1 << 24);

	cc_offset = cc_offset * samples;
	current_avg_uA = ((di->charge_n1 - di->charge_n2 - cc_offset)
							* 3000) >> 12;
	if (samples)
		current_avg_uA = current_avg_uA / samples;
	di->current_avg_uA = current_avg_uA * 1000;

	charger_schedule_delayed_work(&di->twl6030_current_avg_work,
		msecs_to_jiffies(1000 * di->current_avg_interval));
}
#endif

#if defined(GET_BET_TEMP)
static int get_adc_value(struct twl6030_gpadc_request* req, u16 channels)
{
	int ret;

	req->channels = channels;
	req->method = TWL6030_GPADC_SW2;
	req->active = 0;
	req->func_cb = NULL;
	ret = twl6030_gpadc_conversion(req);

	if (ret < 0) {
		dev_dbg(p_di->dev, "gpadc conversion failed: %d\n", ret);
	}

	return ret;
}
#endif

static void charging_timer_work(struct work_struct *work)
{
	charger_fsm(CHARG_FSM_CAUSE_CHARGING_TIMER_EXPIRED);
}

void off_mode_timer_func(unsigned long try)
{
	del_timer(&(p_di->off_mode_timer));
	p_di->off_mode_timer_working = 0;

	D("#########################################\n timer EXPIRED!!!!\n #####################################");

	if(p_di->charger_logo_status == CHARGER_LOGO_STATUS_STARTED)
	{
		charger_schedule_delayed_work(&(p_di->charger_timer_work), 0);
	}
}

int move_queue_point(int point, int move_count, int queue_size, int limit)
{
	int i;

	if(point < 0 || queue_size < 1)
	{
		return 0;
	}

	for(i = 0 ; i < move_count ; i++)
	{
		point++;

		if(point >= queue_size)
		{
			point = 0;
		}

		if(point == limit)
		{
			return point;
		}
	}

	return point;
}

int get_avg_volt(int volt_now)
{
	int count, sum_of_volt, temp_point;
	static ktime_t old_time = {0};
	static ktime_t interval;
	int monitoring_gap_count;

	count = 0;
	sum_of_volt = 0;
	temp_point = 0;


	//[jongho3.lee@lge.com] if system has slept, we should ignore previous voltage data.
	{
		interval =  ktime_sub(ktime_get(), old_time);

		if( (interval.tv.sec > (p_di->monitoring_interval * AVG_VOLT_QUEUE_SIZE * 20) ) \
			|| !(p_di->battery_present) )
		{
			monitoring_gap_count = (interval.tv.sec / p_di->monitoring_interval);
			p_di->avg_volt_head = move_queue_point(p_di->avg_volt_head, monitoring_gap_count, AVG_VOLT_QUEUE_SIZE, p_di->avg_volt_tail);
			p_di->avg_volt_queue_count = 0;
		}
	}


	p_di->avg_volt_queue[p_di->avg_volt_tail] = volt_now;
	p_di->avg_volt_tail = move_queue_point(p_di->avg_volt_tail, 1, AVG_VOLT_QUEUE_SIZE, p_di->avg_volt_head);

	if(p_di->avg_volt_queue_count < AVG_VOLT_QUEUE_SIZE)
	{
		p_di->avg_volt_queue_count++;
	}

	temp_point = p_di->avg_volt_head;

	D("avg_volt_head = %d, avg_volt_tail= %d", p_di->avg_volt_head, p_di->avg_volt_tail);



	do
	{
		sum_of_volt += p_di->avg_volt_queue[temp_point];
		temp_point = move_queue_point(temp_point, 1, AVG_VOLT_QUEUE_SIZE, p_di->avg_volt_tail);
		count++;
		D("sum_of_volt = %d, p_di->avg_volt_queue[temp_point] = %d", sum_of_volt, p_di->avg_volt_queue[temp_point]);
	}
	while(count < p_di->avg_volt_queue_count);

	if(p_di->avg_volt_tail == p_di->avg_volt_head)
	{
		p_di->avg_volt_head = move_queue_point(p_di->avg_volt_head, 1, AVG_VOLT_QUEUE_SIZE, p_di->avg_volt_tail);
	}

	D("avg_volt = %d, count = %d", sum_of_volt / count, count);
	D("avg_volt_queue_count = %d", p_di->avg_volt_queue_count);

	old_time = ktime_get();

	return sum_of_volt / count;
}

int get_pmic_soc()
{
	static int battery_replace_toggle = 1;  //this should be 1 to set initial soc.
	int temp_val;

	if(!(p_di->battery_present))
	{
		battery_replace_toggle = 1;
	}

	if( (p_di->charge_status == POWER_SUPPLY_STATUS_CHARGING) && (p_di->valid_charging_source) )
	{
		if(p_di->charger_source == POWER_SUPPLY_TYPE_USB)
		{
			temp_val = reference_graph((s64)p_di->avg_voltage_mV, battery_soc_graph_with_usb, ARRAY_SIZE(battery_soc_graph_with_usb)) / SOC_TIMES;
		}
		else   // ta or factory.
		{
			temp_val = reference_graph((s64)p_di->avg_voltage_mV, battery_soc_graph_with_ac, ARRAY_SIZE(battery_soc_graph_with_ac)) / SOC_TIMES;
		}

		//[jongho3.lee@lge.com]  gather voltage data at least 3 times.
		if(p_di->avg_volt_queue_count > 3)
		{

			//[jongho3.lee@lge.com] capacity should not drop below RECHARGING_BAT_SOC_CON while charger is plugged.
			if( (p_di->pmic_capacity >= RECHARGING_BAT_SOC_CON) && (p_di->valid_charging_source) )
			{
				if(temp_val < RECHARGING_BAT_SOC_CON)
				{
					temp_val = RECHARGING_BAT_SOC_CON;
				}
			}

			//[jongho3.lee@lge.com] prevent from decreasing capacity while charging.
			if( (battery_replace_toggle && p_di->battery_present) || (temp_val > p_di->pmic_capacity) )
			{
				p_di->pmic_capacity = temp_val;

				if(battery_replace_toggle)
				{
					battery_replace_toggle = 0;
				}
			}
		}
		else
		{
			p_di->pmic_capacity = temp_val;
		}
	}
	else
	{
		temp_val = reference_graph((s64)p_di->avg_voltage_mV, battery_soc_graph_with_no_charger, ARRAY_SIZE(battery_soc_graph_with_no_charger)) / SOC_TIMES;

		//[jongho3.lee@lge.com]  gather voltage data at least 3 times.
		if(p_di->avg_volt_queue_count > 3)
		{

			//[jongho3.lee@lge.com] capacity should not drop below RECHARGING_BAT_SOC_CON while charger is plugged.
			if( (p_di->pmic_capacity >= RECHARGING_BAT_SOC_CON) && (p_di->valid_charging_source) )
			{
				if(temp_val < RECHARGING_BAT_SOC_CON)
				{
					temp_val = RECHARGING_BAT_SOC_CON;
				}
			}

			//[jongho3.lee@lge.com] prevent from increasing capacity while discharging.
			if( (battery_replace_toggle && p_di->battery_present) || (temp_val < p_di->pmic_capacity) )
			{
				p_di->pmic_capacity = temp_val;

				if(battery_replace_toggle)
				{
					battery_replace_toggle = 0;
				}
			}
		}
		else
		{
			p_di->pmic_capacity = temp_val;
		}
	}

	if(p_di->pmic_capacity > 100)
	{
		p_di->pmic_capacity = 100;
	}
	else if(p_di->pmic_capacity <= 0)
	{
		p_di->pmic_capacity = 1;    ///keep alive for power board...
	}

	temp_val = p_di->pmic_capacity;

	if(p_di->avg_volt_queue_count < 3)
	{
		temp_val = (temp_val * (p_di->avg_volt_queue_count + 3) / 6);
	}

	return temp_val;
}

static void twl6030_bci_battery_work(struct work_struct *work)
{
	struct twl6030_bci_device_info *di = container_of(work,
			struct twl6030_bci_device_info, twl6030_bci_monitor_work.work);
	struct twl6030_gpadc_request req;
	int adc_code;
	int temp_C;
	int ret;
	int fsm_work_needed = 0;
	int ui_update_needed = 0;
	static int old_capacity;
	int charging_ic_status;
	u8 temp_val;

	charger_schedule_delayed_work(&di->twl6030_bci_monitor_work,
			msecs_to_jiffies(1000 * di->monitoring_interval));

	charging_ic_status = get_charging_ic_status();
	if(di->temp_control == 0xFF)
	{
		ret = lge_dynamic_nvdata_read(LGE_NVDATA_DYNAMIC_CHARGING_TEMP_OFFSET, &temp_val , 1);
		D("@@@@@@@@@@@@@@@E_NVDATA_DYNAMIC_CHARGING_TEMP_OFFSET= %x", di->temp_control);
		if(ret == 1)
		{
			di->temp_control = temp_val;
		}
	}

#if 0
	//[jongho3.lee@lge.com] correct charging status missmatch caused by charging timer or any...
	if( ( (charging_ic_status != POWER_SUPPLY_TYPE_BATTERY) \
			&& (di->charge_status != POWER_SUPPLY_STATUS_CHARGING) ) \
		|| ( (charging_ic_status == POWER_SUPPLY_TYPE_BATTERY) \
			&& (di->charge_status == POWER_SUPPLY_STATUS_CHARGING) ) )
	{
		if(charging_ic_status == POWER_SUPPLY_TYPE_BATTERY)
		{
			di->charge_status = POWER_SUPPLY_STATUS_DISCHARGING;
		}
		else
		{
			di->charge_status = POWER_SUPPLY_STATUS_CHARGING;
		}
		ui_update_needed = 1;
	}
#endif
	D(" ####### BATTERY SOC = %d ##########", di->capacity );


#if defined(GET_BET_TEMP)
	req.channels = (1 << BAT_TEMP_CHANNEL) | (1 << 7) | (1 << 8);
	req.method = TWL6030_GPADC_SW2;
	req.active = 0;
	req.func_cb = NULL;

	get_adc_value(&req, req.channels);
#else
	req.channels = (1 << BAT_TEMP_CHANNEL) | (1 << 7) | (1 << 8);
	req.method = TWL6030_GPADC_SW2;
	req.active = 0;
	req.func_cb = NULL;
	ret = twl6030_gpadc_conversion(&req);

	if (ret < 0) {
		dev_dbg(di->dev, "gpadc conversion failed: %d\n", ret);
		return;
	}
#endif



	if (req.rbuf[7] > 0)
		di->voltage_uV = req.rbuf[7];
	if (req.rbuf[8] > 0)
		di->bk_voltage_uV = req.rbuf[8];


	D("pmic _volt = %d", req.rbuf[7]);

//[jongho3.lee@lge.com] get avg_voltage of battery.

/*
 * Let's check if charging ui update is needed.
 */
//LGE_CHANGE [jongho3.lee@lge.com] monitor changes of tbat.
	{
		adc_code = req.rbuf[BAT_TEMP_CHANNEL];
#if 1
		temp_C = average_temp(reference_graph((s64)adc_code, battery_temp_graph, ARRAY_SIZE(battery_temp_graph)) / (TEMP_TIMES / 10));
#else
		temp_C = 10;
#endif
		D(" ####### CALCULATED TEMP = %d ##########", temp_C );

		if( (temp_C/10) != ((di->temp_C)/10)  )
		{
			/* first 2 values are for negative temperature */
			di->temp_C = temp_C;
			ui_update_needed = 1;
		}
		D("Battery adc = %d", adc_code);
	}


/*
 * Let's check if charging fsm work is needed.
 */
//LGE_CHANGE [jongho3.lee@lge.com] monitoring battery presence by tbat.
	if( (di->temp_C < TEMP_LOW_NO_BAT) && (di->temp_control != UNLIMITED_TEMP_VAL))
	{
		di->capacity_validation = NOT_INIT_CAP;
		di->battery_present = 0;
		fsm_work_needed = 1;
	}
	else if(!(di->battery_present))
	{
		di->battery_present = 1;
		fsm_work_needed = 1;
	}

	{
		di->avg_voltage_mV = get_avg_volt(di->voltage_uV);
		get_pmic_soc();

	}

#if defined(TEMP_BAT_GAUTE_BY_VOLT)
	if(di->capacity_validation > 100)
	{
		di->capacity = di->pmic_capacity;
	}
#if 1
	else if( (di->capacity_validation != di->dummy_cap) && di->dummy_cap > 0)
	{
		di->capacity = di->dummy_cap;
		follow_fg_capacity();
	}
	D("di->capacity_validation= %d, di->pmic_cap=%d, di->dummy_cap=%d", di->capacity_validation, di->pmic_capacity, di->dummy_cap);
#endif
#endif

//LGE_CHANGE [jongho3.lee@lge.com] monitor changes of bat soc.
	if(di->capacity != old_capacity)
	{
		old_capacity = di->capacity;
		ui_update_needed = 1;
	}

	//LGE_CHANGE [jongho3.lee@lge.com] monitoring recharging condition.
	if(di->charger_source != POWER_SUPPLY_TYPE_BATTERY  \
			&& di->charge_status == POWER_SUPPLY_STATUS_DISCHARGING \
			&& di->capacity <= RECHARGING_BAT_SOC_CON )
	{
		fsm_work_needed = 1;
	}

//LGE_CHANGE [jongho3.lee@lge.com] monitoring charging tbat condition..
	if( (!is_tbat_good(di->temp_C) && (di->charge_status == POWER_SUPPLY_STATUS_CHARGING)) \
		|| (is_tbat_good(di->temp_C) && (di->charge_status == POWER_SUPPLY_STATUS_DISCHARGING) &&
		recharging_status == RECHARGING_WAIT_UNSET) )
	{
		fsm_work_needed = 1;
	}

//LGE_CHANGE [jongho3.lee@lge.com] monitoring full charging condition by soc..
	if( (di->capacity > 99) && (di->charge_status != POWER_SUPPLY_STATUS_FULL))
	{
		fsm_work_needed = 1;
	}
	else if( (di->capacity < 100) && (di->charge_status == POWER_SUPPLY_STATUS_FULL) )
	{
		fsm_work_needed = 1;
	}

	if( (di->capacity_validation > 100) )
	{
		if(!(di->off_mode_timer_working))
		{
			if(di->pmic_capacity > (RECHARGING_BAT_SOC_CON - 3) && di->pmic_capacity < 100)
			{
				// [jongho3.lee@lge.com] charging timer setting
				u32 wait;

				init_timer(&(di->off_mode_timer));
				(di->off_mode_timer).data = 0;
				wait = (HZ*CHR_TIMER_SECS * ( 100 - di->pmic_capacity) ) ;
				//wait = (HZ*9 ) ;
				(di->off_mode_timer).expires = jiffies + wait;
				(di->off_mode_timer).function = off_mode_timer_func;
				add_timer(&(di->off_mode_timer));
				di->off_mode_timer_working = 1;
				D("#########################################\n timer START!!!!\n #####################################");
				// [jongho3.lee@lge.com] charging timer setting
			}
		}
	}


//LGE_CHANGE [jongho3.lee@lge.com] do charging fsm if battery is present..
	if(fsm_work_needed)
	{
		D("charging_ic interrupt occured!, %d \n", &di->charger_work);
		charger_schedule_delayed_work(&di->charger_work, 0);
		return;
	}

	if(ui_update_needed)
	{
		power_supply_changed(&di->bat);
	}

	return;
}

#if defined(TWL6030_PMIC_CHARGING)
static void twl6030_current_mode_changed(struct twl6030_bci_device_info *di)
{

	/* FG_REG_01, 02, 03 is 24 bit unsigned sample counter value */
	twl_i2c_read(TWL6030_MODULE_GASGAUGE, (u8 *) &di->timer_n1,
							FG_REG_01, 3);
	/*
	 * FG_REG_04, 5, 6, 7 is 32 bit signed accumulator value
	 * accumulates instantaneous current value
	 */
	twl_i2c_read(TWL6030_MODULE_GASGAUGE, (u8 *) &di->charge_n1,
							FG_REG_04, 4);

	cancel_delayed_work(&di->twl6030_current_avg_work);
	charger_schedule_delayed_work(&di->twl6030_current_avg_work,
		msecs_to_jiffies(1000 * di->current_avg_interval));
}
#endif

#define to_twl6030_bci_device_info(x) container_of((x), \
			struct twl6030_bci_device_info, bat);

static void twl6030_bci_battery_external_power_changed(struct power_supply *psy)
{
	struct twl6030_bci_device_info *di = to_twl6030_bci_device_info(psy);

#if 0
	cancel_delayed_work(&di->twl6030_bci_monitor_work);
	charger_schedule_delayed_work(&di->twl6030_bci_monitor_work, 0);
#endif
}

#define to_twl6030_ac_device_info(x) container_of((x), \
		struct twl6030_bci_device_info, ac);

static int twl6030_ac_get_property(struct power_supply *psy,
					enum power_supply_property psp,
					union power_supply_propval *val)
{
	struct twl6030_bci_device_info *di = to_twl6030_ac_device_info(psy);

#if defined(CONFIG_LGE_COSMO_MUIC)


	switch (psp) {
	case POWER_SUPPLY_PROP_ONLINE:
		if( (di->charger_source == POWER_SUPPLY_TYPE_MAINS) && di->valid_charging_source)
		{
			if(di->charger_logo_status == CHARGER_LOGO_STATUS_STARTED && \
				(start_cond & 0x20))
			{
				val->intval = 0;
			}
			else
			{
				val->intval = 1;
			}
		}
		else
		{
			val->intval = POWER_SUPPLY_TYPE_BATTERY;
		}
		break;
#else
	switch (psp) {
	case POWER_SUPPLY_PROP_ONLINE:
		val->intval = di->ac_online;
		break;
#endif
	case POWER_SUPPLY_PROP_VOLTAGE_NOW:
		val->intval = twl6030_get_gpadc_conversion(9);
		break;
	default:
		return -EINVAL;
	}

	return 0;
}

#define to_twl6030_usb_device_info(x) container_of((x), \
		struct twl6030_bci_device_info, usb);

static int twl6030_usb_get_property(struct power_supply *psy,
					enum power_supply_property psp,
					union power_supply_propval *val)
{
	struct twl6030_bci_device_info *di = to_twl6030_usb_device_info(psy);

#if defined(CONFIG_LGE_COSMO_MUIC)
	switch (psp) {
	case POWER_SUPPLY_PROP_ONLINE:
		if( (di->charger_source == POWER_SUPPLY_TYPE_USB) && di->valid_charging_source)
		{
			if(di->charger_logo_status == CHARGER_LOGO_STATUS_STARTED && \
				(start_cond & 0x20))
			{
				val->intval = 0;
			}
			else
			{
				val->intval = 1;
			}
		}
		else
		{
			val->intval = POWER_SUPPLY_TYPE_BATTERY;
		}
		break;
#else
	switch (psp) {
	case POWER_SUPPLY_PROP_ONLINE:
		val->intval = di->usb_online;
		break;
#endif
	case POWER_SUPPLY_PROP_VOLTAGE_NOW:
		val->intval = twl6030_get_gpadc_conversion(10);
		break;
	default:
		return -EINVAL;
	}

	return 0;
}

// LGE_CHANGE [jongho3.lee@lge.com] charger work func. [START_LGE]
static void battery_gauge_reset_work(struct work_struct *work)
{
	mutex_lock(&charging_fsm_lock);
	charging_ic_deactive();
	recharging_status = RECHARGING_WAIT_UNSET;
	p_di->charge_status = POWER_SUPPLY_STATUS_DISCHARGING;

	if(p_di->charger_logo_status == CHARGER_LOGO_STATUS_END)
	{
		msleep(600);   //wait until fuel gauge voltage stable.
		gpio_set_value(OMAP_SEND, 0);
		msleep(1);   //wait reseting fuel gauge.
		gpio_set_value(OMAP_SEND, 1);
		msleep(1);   //wait reseting fuel gauge.
		gpio_set_value(OMAP_SEND, 0);
		msleep(2000);   //wait reseting fuel gauge.
	}

	mutex_unlock(&charging_fsm_lock);

	if( p_di->charger_logo_status == CHARGER_LOGO_STATUS_END)
	{
		charger_schedule_delayed_work(&p_di->charger_work, 0);
	}
}

// LGE_CHANGE [jongho3.lee@lge.com] charger work func. [START_LGE]
static void battery_power_off_work(struct work_struct *work)
{
	// LGE_CHAGNE_S [jongho3.lee@lge.com] resister map
	// /* PHOENIX_DEV_ON*/
//#define TWL6030_CHIP_PM     0x48
#define TWL6030_CHIP_PM     0x14

#define PHOENIX_DEV_ON              0x06
//#define PHOENIX_DEV_ON              0x25

#define SW_RESET                    (1 << 6)
#define MOD_DEVON                   (1 << 5)
#define CON_DEVON                   (1 << 4)
#define APP_DEVON                   (1 << 3)
#define MOD_DEVOFF                  (1 << 2)
#define CON_DEVOFF                  (1 << 1)
#define APP_DEVOFF                  (1 << 0)

#define DEV_OFF						MOD_DEVOFF | CON_DEVOFF | APP_DEVOFF

	mutex_lock(&charging_fsm_lock);

	D("Power off the device!!!!!!!!!!!");
	charging_ic_deactive();

#if defined(POWER_OFF_WHEN_NO_BAT)
	//twl_i2c_write_u8(TWL6030_CHIP_PM, DEV_OFF, PHOENIX_DEV_ON); /* PHOENIX_DEV_ON Reg */
	//kernel_halt();
	//do_exit(0);
	//panic("cannot halt");

	msleep_interruptible(1900);   //wait until fuel gauge voltage stable.

	machine_halt();
#endif
	mutex_unlock(&charging_fsm_lock);

	//kernel_power_off();
}

// LGE_CHANGE [jongho3.lee@lge.com] charger work func. [START_LGE]
static void charging_battery_work(struct work_struct *work)
{
	/*
	struct twl6030_bci_device_info *di = container_of(work,
		struct twl6030_bci_device_info,
		charger_work.work);
	twl6030_bk_bci_battery_read_status(di);
	charger_schedule_delayed_work(&di->twl6030_bk_bci_monitor_work, 5000);
	*/
	charger_fsm(CHARG_FSM_CAUSE_ANY);
}

#define to_twl6030_bk_bci_device_info(x) container_of((x), \
		struct twl6030_bci_device_info, bk_bat);

static int twl6030_bk_bci_battery_get_property(struct power_supply *psy,
					enum power_supply_property psp,
					union power_supply_propval *val)
{
	struct twl6030_bci_device_info *di = to_twl6030_bk_bci_device_info(psy);

	switch (psp) {
	case POWER_SUPPLY_PROP_VOLTAGE_NOW:
		val->intval = di->bk_voltage_uV * 1000;
		break;
	default:
		return -EINVAL;
	}

	return 0;
}

// LGE_CHANGE [jongho3.lee@lge.com] validate_gauge_value. [START_LGE]


// LGE_CHANGE_S [jongho3.lee@lge.com] follow real battery cap.. [START_LGE]
void follow_fg_capacity(void)
{
	static int count = 0;


	if(p_di->capacity_validation <= 100)
	{
		count++;
		if( p_di->charge_status == POWER_SUPPLY_STATUS_CHARGING )
		{
			if(p_di->capacity_validation > p_di->dummy_cap)
			{
				if( !(count %= 6) )
				{
					p_di->dummy_cap++;
					D("p_di->dummy_cap++");
				}
			}
			else
			{
				D("if it's charging, the fg_capacity will follwo dummy_cap..");
			}
		}
		else if( p_di->charge_status != POWER_SUPPLY_STATUS_CHARGING )
		{
			if(p_di->capacity_validation < p_di->dummy_cap)
			{
				if( !(count %= 6 ) )
				{
					p_di->dummy_cap--;
					D("p_di->dummy_cap--");
				}
			}
			else
			{
				D("if it's discharging, the fg_capacity will follwo dummy_cap..");
			}
		}

		if(p_di->dummy_cap >= 100)
		{
			p_di->dummy_cap = 99;
		}
		if(p_di->dummy_cap < 0)
		{
			p_di->dummy_cap = 0;
		}

		if(p_di->capacity_validation == p_di->dummy_cap)
		{
			p_di->dummy_cap = -1;
			D("p_di->capacity_validation == p_di->dummy_cap..");
		}
	}
	D("p_di->dummy_cap = %d..", p_di->dummy_cap);

	return;
}
// LGE_CHANGE_E [jongho3.lee@lge.com] follow real battery cap.. [END_LGE]


int validate_gauge_value(void* _di, int capacity)
{
	//Capacity validation is performed in modem. this is just for in case of no battery..
	static int validation = 1;
	struct twl6030_bci_device_info *di = _di;
	int calculated_soc;


	if( !(p_di->battery_present) )
	{
		di->capacity_validation = NOT_INIT_CAP;
		validation = 0;
		return 0;
	}

	// FIXME: [jongho3.lee@lge.com] gague value should be validated with previous capacity and battery discharging/charging graprh/
	// // [jongho3.lee@lge.com] capacity should not be decreased while charging and vice versa.
	if( validation && ( p_di->charger_logo_status == CHARGER_LOGO_STATUS_END) )
	{
		if(validation > 3)
		{
			if( p_di->charge_status == POWER_SUPPLY_STATUS_CHARGING )
			{
				if(capacity < p_di->capacity)
				{
					return 0;
				}
			}
			else
			{
				if(capacity > p_di->capacity)
				{
					return 0;
				}
			}
		}
		else
		{
			validation++;
		}
	}

	if(capacity > 100 )
	{
		return 0;
	}

	if(validation)
	{
		di->capacity_validation = capacity;
		return 1;
	}
// [jongho3.lee] make sure voltage value is set before capacity calculation..

#if defined(USE_GAUGE_GRAPH_VALIDATION)
	calculated_soc = reference_graph((s64)di->fg_voltage_mV, battery_soc_graph, ARRAY_SIZE(battery_soc_graph)) / SOC_TIMES;
	D(" ####### CALCULATED SOC = %d ##########", calculated_soc);

	if( (capacity < calculated_soc + VALID_RANGE) && (capacity > calculated_soc - VALID_RANGE) )
	{
		D(" ##### SOC & CALCULATED SOC is met  ##########");
		//[jongho3.lee@lge.com] capacity validation should be performed just once.
		validation = 1;
	}
#else
	validation = 1;
#endif

	D(" ##### SOC & CALCULATED SOC is not met!! RESET GAUGE!!  ##########");

	if(validation)
	{
		di->capacity_validation = capacity;
		return 1;
	}

	//[jongho3.lee@lge.com] capacity validation should be performed just once.
	validation = 1;
	di->capacity_validation = INVALID_CAP;
	return 0;

}

// LGE_CHANGE [jongho3.lee@lge.com] report battery status.[start]
int get_battery_health(struct twl6030_bci_device_info *di)
{
		int temp = di->temp_C;
		if(temp >= TEMP_LOW_DISCHARGING && temp < TEMP_HIGH_DISCHARGING)
		{
			return POWER_SUPPLY_HEALTH_GOOD;
		}
		else if(temp < TEMP_LOW_DISCHARGING )
		{
			return POWER_SUPPLY_HEALTH_COLD;
		}
		else if(temp >= TEMP_HIGH_DISCHARGING)
		{
			return POWER_SUPPLY_HEALTH_OVERHEAT;
		}

		//FIXME: [jongho3.lee@lge.com] TODO : voltage check.
		return POWER_SUPPLY_HEALTH_UNKNOWN;
}

// [jongho3.lee@lge.com] add to set_property. //start
int twl6030_bci_battery_set_property(struct power_supply *psy,
					enum power_supply_property psp,
					const union power_supply_propval *val)
{
	struct twl6030_bci_device_info *di;

	di = to_twl6030_bci_device_info(psy);

	switch (psp) {
	case POWER_SUPPLY_PROP_GAUGE_CONTROL:
		D(" GAUGE_CONTROL SOC = %d", val->intval);
		di->gauge_control_count++;
		if( (di->charger_logo_status != CHARGER_LOGO_STATUS_END))
		{
			if(val->intval == 300001)
			{
				di->charger_logo_status = CHARGER_LOGO_STATUS_END;
				wake_unlock(&(p_di->off_mode_charger_wake_lock));

				//<jongho3.lee@lge.com>  show dummy cpapcity
				if( (di->pmic_capacity >= RECHARGING_BAT_SOC_CON) && \
					( (di->charger_source == POWER_SUPPLY_TYPE_MAINS) || \
					(di->charger_source == POWER_SUPPLY_TYPE_USB) ) )
				{
					di->dummy_cap = di->pmic_capacity;
				}

				if(di->battery_present && (di->charger_source != POWER_SUPPLY_TYPE_FACTROY) )
				{
					charger_schedule_delayed_work(&di->gauge_reset_work, 0);
				}
				D("@@@@@@ OFF MODE CHARGING is starteed : ");

				if((di->off_mode_timer_working))
				{
					del_timer(&(p_di->off_mode_timer));
					p_di->off_mode_timer_working = 0;
				}

			}
			else if(val->intval == 300000)
			{
				di->charger_logo_status = CHARGER_LOGO_STATUS_STARTED;
				wake_lock(&(p_di->off_mode_charger_wake_lock));

				if( !(di->battery_present)  && (di->charger_source != POWER_SUPPLY_TYPE_FACTROY) )
				{
					if(p_di->valid_charging_source)
					{
#if defined(POWER_OFF_WHEN_NO_BAT)
						charger_schedule_delayed_work(&di->power_off_work, HZ*2);
#else
						charger_schedule_delayed_work(&di->power_off_work, 0);
#endif
					}
				}
				D("@@@@@@ OFF MODE CHARGING is finished : ");
			}
			break;
		}
		if(val->intval == 600000)
		{
			gpio_set_value(OMAP_SEND, 0);
			break;
		}
		if(val->intval == 600001)
		{
			gpio_set_value(OMAP_SEND, 1);
			break;
		}

		if(val->intval == 600003)
		{
			charger_schedule_delayed_work(&di->gauge_reset_work, 0);
			break;
		}

		if(validate_gauge_value(di,val->intval))
		{
			di->capacity = val->intval;
			//[jongho3.lee@lge.com] first valide soc value. update UI immediately..
			cancel_delayed_work(&di->twl6030_bci_monitor_work);
			charger_schedule_delayed_work(&di->twl6030_bci_monitor_work, HZ);
		}
		break;
	case POWER_SUPPLY_PROP_VOLTAGE_NOW:
		di->fg_voltage_mV = val->intval;
		D(" di->fg_voltage_mV = %d", di->fg_voltage_mV);
		break;
	case POWER_SUPPLY_PROP_CHARGER_MODE:
		if (strncmp((char*)(val->strval), "ac", 1) == 0)
		{
			charging_ic_set_ta_mode();
			p_di->charge_status = POWER_SUPPLY_STATUS_CHARGING;
		}
		else if(strncmp((char*)(val->strval), "usb", 1) == 0)
		{
			charging_ic_active_default();
			p_di->charge_status = POWER_SUPPLY_STATUS_CHARGING;
		}
		else if(strncmp((char*)(val->strval), "factory", 1) == 0)
		{
			charging_ic_set_factory_mode();
			p_di->charge_status = POWER_SUPPLY_STATUS_CHARGING;
		}
		else if(strncmp((char*)(val->strval), "stop", 1) == 0)
		{
			charging_ic_deactive();
			p_di->charge_status = POWER_SUPPLY_STATUS_DISCHARGING;
		}
		break;
	case POWER_SUPPLY_PROP_CHARGER_TEMP_CONTROL:
		if( val->intval == UNLIMITED_TEMP_VAL )
		{
			p_di->temp_control = val->intval;
		}
		else
		{
			p_di->temp_control = 0;
		}
		//FIXME: write to nv data.
		D("p_di->temp_control = %02X ", val->intval);
		lge_dynamic_nvdata_write(LGE_NVDATA_DYNAMIC_CHARGING_TEMP_OFFSET, &(p_di->temp_control), 1);
		break;

	default:
		return -EINVAL;
	}
	return 0;
}
EXPORT_SYMBOL(twl6030_bci_battery_set_property);
// [jongho3.lee@lge.com] add to set_property. //end


void set_modem_alive(int alive)
{
	//do nothing
	return;
}
EXPORT_SYMBOL(set_modem_alive);

// LGE_CHANGE [jongho3.lee@lge.com] validate_gauge_value. [END_LGE]
static int twl6030_bci_battery_get_property(struct power_supply *psy,
					enum power_supply_property psp,
					union power_supply_propval *val)
{
	struct twl6030_bci_device_info *di;
	u8 temp_val = 0;
	int ret = 0;

	di = to_twl6030_bci_device_info(psy);

	switch (psp) {
	case POWER_SUPPLY_PROP_STATUS:
		if(di->capacity >= RECHARGING_BAT_SOC_CON)
		{
			val->intval = POWER_SUPPLY_STATUS_FULL;
		}
		else if(p_di->valid_charging_source
			&& p_di->battery_present
			&& recharging_status == RECHARGING_WAIT_SET
			&& ( di->charger_source == POWER_SUPPLY_TYPE_USB
				|| di->charger_source == POWER_SUPPLY_TYPE_MAINS )
			)
		{
			val->intval = POWER_SUPPLY_STATUS_CHARGING;
		}
		else
		{
			{
				val->intval = di->charge_status;
			}
		}
		break;
	case POWER_SUPPLY_PROP_VOLTAGE_NOW:
#if 0
		val->intval = di->voltage_uV * 1000;
		//val->intval = di->fg_voltage_mV * 1000;
#else
		val->intval = di->avg_voltage_mV * 1000;
#endif
		break;
	case POWER_SUPPLY_PROP_CURRENT_NOW:
#if !defined(CONFIG_LG_FW_MAX17040_FUEL_GAUGE)
		twl6030battery_current(di);
#endif
		val->intval = di->current_uA;
		break;
	case POWER_SUPPLY_PROP_TEMP:
		if(di->temp_control == 0xFF)
		{
			ret = lge_dynamic_nvdata_read(LGE_NVDATA_DYNAMIC_CHARGING_TEMP_OFFSET, &temp_val , 1);
			D("@@@@@@@@@@@@@@@E_NVDATA_DYNAMIC_CHARGING_TEMP_OFFSET= %x", di->temp_control);
			if(ret == 1)
			{
				di->temp_control = temp_val;
			}
		}
		if( (p_di->temp_control == UNLIMITED_TEMP_VAL ) && (di->temp_C > 550) )
		{
			val->intval = 550;
		}
		else
		{
			val->intval = di->temp_C;
		}
		break;
	case POWER_SUPPLY_PROP_ONLINE:
		val->intval = di->charger_source;
		break;
// LGE_CHANGE [jongho3.lee@lge.com] report battery status.[start]
	case POWER_SUPPLY_PROP_PRESENT:
		val->intval = di->battery_present;
		break;
	case POWER_SUPPLY_PROP_TECHNOLOGY:
		val->intval = POWER_SUPPLY_TECHNOLOGY_LION;
		break;
// LGE_CHANGE [jongho3.lee@lge.com] report battery status.[end]
	case POWER_SUPPLY_PROP_CURRENT_AVG:
		val->intval = di->current_avg_uA;
		break;
	case POWER_SUPPLY_PROP_HEALTH:
		di->bat_health = get_battery_health(di);
		val->intval = di->bat_health;
		break;
// LGE_CHANGE [jongho3.lee@lge.com] report battery status.[end]

	case POWER_SUPPLY_PROP_CAPACITY:
		/* FIXME correct the threshold
		 * need to get the correct percentage value per the
		 * battery characteristics. Approx values for now.
		 */
//FIXME: fuel gauge should be in action.
//#if defined(CONFIG_LG_FW_MAX17040_FUEL_GAUGE)
#if 1
		val->intval = ( (( ((di->capacity)*10000) / RECHARGING_BAT_SOC_CON ) + 50 ) / 100 );
#else
		val->intval = di->capacity;
#endif
		if(val->intval > 100)
		{
			val->intval = 100;
		}

		D("@@@@@@@@ CAPACITY : %d , @@@@ UI CAPACITY : %d ", di->capacity, val->intval);
		break;

		// [jongho3.lee@lge.com] add to gauge_property. //start
	case POWER_SUPPLY_PROP_GAUGE_VOLTAGE:
		val->intval = di->fg_voltage_mV * 1000;
		break;
	case POWER_SUPPLY_PROP_GAUGE_CONTROL:
	//FIXME: this is temp code. battery gauge graph should be calculated..
#if defined(USE_GAUGE_GRAPH_VALIDATION)
		val->intval = di->capacity_validation;
		//val->intval = di->capacity;
#else
		if(di->capacity_validation == NOT_INIT_CAP)
		{
			val->intval = INVALID_CAP;
		}
		else
		{
			val->intval = di->capacity_validation;
		}
#endif
		//D(" Capacity_validation value = %d", di->capacity_validation);
		break;

	case POWER_SUPPLY_PROP_PMIC_SOC:
		val->intval = di->pmic_capacity;
		//val->intval = di->dummy_cap;
		//val->intval = di->capacity_validation;
		break;
	case POWER_SUPPLY_PROP_GAUGE_CONTROL_COUNT:
		val->intval = di->gauge_control_count;
		break;
	case POWER_SUPPLY_PROP_CHARGER_MODE:
		val->intval = get_charging_ic_status();
		break;
	case POWER_SUPPLY_PROP_CHARGER_TEMP_CONTROL:
		//FIXME: write to nv data.
		lge_dynamic_nvdata_read(LGE_NVDATA_DYNAMIC_CHARGING_TEMP_OFFSET, &temp_val, 1);
		val->intval = (int)temp_val;
		D("p_di->temp_control = %02X ", (char)val->intval);
		break;

	default:
		return -EINVAL;
	}
	return 0;
}

int twl6030_register_notifier(struct notifier_block *nb,
				unsigned int events)
{
	return blocking_notifier_chain_register(&notifier_list, nb);
}
EXPORT_SYMBOL_GPL(twl6030_register_notifier);

int twl6030_unregister_notifier(struct notifier_block *nb,
				unsigned int events)
{
	return blocking_notifier_chain_unregister(&notifier_list, nb);
}
EXPORT_SYMBOL_GPL(twl6030_unregister_notifier);

#if defined(TWL6030_PMIC_FUELGAUGE)
//LGE_CHANGE [jongho3.lee@lge.com] we have external fuel gauge.
static ssize_t set_fg_mode(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	long val;
	int status = count;
	struct twl6030_bci_device_info *di = dev_get_drvdata(dev);

	if ((strict_strtol(buf, 10, &val) < 0) || (val > 3))
		return -EINVAL;
	di->fuelgauge_mode = val;
	twl_i2c_write_u8(TWL6030_MODULE_GASGAUGE, (val << 6) | CC_CAL_EN,
							FG_REG_00);
	twl6030_current_mode_changed(di);
	return status;
}

static ssize_t show_fg_mode(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	int val;
	struct twl6030_bci_device_info *di = dev_get_drvdata(dev);

	val = di->fuelgauge_mode;
	return sprintf(buf, "%d\n", val);
}
#endif

#if defined(TWL6030_PMIC_CHARGING)
// FIXME: [jongho3.lee@lge.com] charger src is shown in ac/usb attribute.
static ssize_t set_charge_src(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	long val;
	int status = count;
	struct twl6030_bci_device_info *di = dev_get_drvdata(dev);

	if ((strict_strtol(buf, 10, &val) < 0) || (val < 2) || (val > 3))
		return -EINVAL;
	di->vac_priority = val;
	return status;
}

// FIXME: [jongho3.lee@lge.com] show charger source in properway.
static ssize_t show_charge_src(struct device *dev,
			struct device_attribute *attr, char *buf)
{
	int val;
	struct twl6030_bci_device_info *di = dev_get_drvdata(dev);

	val = di->vac_priority;
	return sprintf(buf, "%d\n", val);
}
#endif

static ssize_t show_vbus_voltage(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	int val;

	val = twl6030_get_gpadc_conversion(10);

	return sprintf(buf, "%d\n", val);
}

static ssize_t show_id_level(struct device *dev, struct device_attribute *attr,
							char *buf)
{
	int val;

	val = twl6030_get_gpadc_conversion(14);

	return sprintf(buf, "%d\n", val);
}

static ssize_t set_watchdog(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	long val;
	int status = count;
	struct twl6030_bci_device_info *di = dev_get_drvdata(dev);

	if ((strict_strtol(buf, 10, &val) < 0) || (val < 1) || (val > 127))
		return -EINVAL;
	di->watchdog_duration = val;
	twl_i2c_write_u8(TWL6030_MODULE_CHARGER, val, CONTROLLER_WDG);

	return status;
}

static ssize_t show_watchdog(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	int val;
	struct twl6030_bci_device_info *di = dev_get_drvdata(dev);

	val = di->watchdog_duration;
	return sprintf(buf, "%d\n", val);
}

#if defined(TWL6030_PMIC_CHARGING)
//LGE_CHANGE [jongho3.lee@lge.com] we have external fuel gauge.
static ssize_t show_fg_counter(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	int fg_counter = 0;

	twl_i2c_read(TWL6030_MODULE_GASGAUGE, (u8 *) &fg_counter,
							FG_REG_01, 3);
	return sprintf(buf, "%d\n", fg_counter);
}

static ssize_t show_fg_accumulator(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	long fg_accum = 0;

	twl_i2c_read(TWL6030_MODULE_GASGAUGE, (u8 *) &fg_accum, FG_REG_04, 4);

	return sprintf(buf, "%ld\n", fg_accum);
}

static ssize_t show_fg_offset(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	s16 fg_offset = 0;

	twl_i2c_read(TWL6030_MODULE_GASGAUGE, (u8 *) &fg_offset, FG_REG_08, 2);
	fg_offset = ((s16)(fg_offset << 6) >> 6);

	return sprintf(buf, "%d\n", fg_offset);
}

static ssize_t set_fg_clear(struct device *dev, struct device_attribute *attr,
						const char *buf, size_t count)
{
	long val;
	int status = count;

	if ((strict_strtol(buf, 10, &val) < 0) || (val != 1))
		return -EINVAL;
	twl_i2c_write_u8(TWL6030_MODULE_GASGAUGE, CC_AUTOCLEAR, FG_REG_00);

	return status;
}

static ssize_t set_fg_cal(struct device *dev, struct device_attribute *attr,
						const char *buf, size_t count)
{
	long val;
	int status = count;

	if ((strict_strtol(buf, 10, &val) < 0) || (val != 1))
		return -EINVAL;
	twl_i2c_write_u8(TWL6030_MODULE_GASGAUGE, CC_CAL_EN, FG_REG_00);

	return status;
}
#endif //we dont control PMIC fuel gauge.

static ssize_t set_charging(struct device *dev, struct device_attribute *attr,
					  const char *buf, size_t count)
{
	int status = count;
#if defined(TWL6030_PMIC_CHARGING)
	struct twl6030_bci_device_info *di = dev_get_drvdata(dev);
#endif

//LGE_CHANGE [jongho3.lee@lge.com] we control external charger.
	if (strncmp(buf, "startac", 7) == 0) {
		if(get_charging_ic_status() != POWER_SUPPLY_TYPE_BATTERY )
		{
			charging_ic_deactive();
		}
		charging_ic_set_ta_mode();
#if defined(TWL6030_PMIC_ENABLE_CHARGING)
		if (di->charger_source == POWER_SUPPLY_TYPE_USB)
			twl6030_stop_usb_charger(di);
		twl6030_start_ac_charger(di);
#endif
	} else if (strncmp(buf, "startusb", 8) == 0) {
		if(get_charging_ic_status() != POWER_SUPPLY_TYPE_BATTERY )
		{
			charging_ic_deactive();
		}
		charging_ic_active_default();
	} else if (strncmp(buf, "startfactory", 12) == 0) {
		if(get_charging_ic_status() != POWER_SUPPLY_TYPE_BATTERY )
		{
			charging_ic_deactive();
		}
		charging_ic_set_factory_mode();
#if defined(TWL6030_PMIC_ENABLE_CHARGING)
		if (di->charger_source == POWER_SUPPLY_TYPE_MAINS)
			twl6030_stop_ac_charger(di);
		twl6030_start_usb_charger(di);
#endif
	} else if (strncmp(buf, "stop" , 4) == 0)
	{
		charging_ic_deactive();
#if defined(TWL6030_PMIC_ENABLE_CHARGING)
		twl6030_stop_charger(di);
#endif
	} else
		return -EINVAL;

	return status;
}

static ssize_t set_regulation_voltage(struct device *dev,
	struct device_attribute *attr, const char *buf, size_t count)
{
	long val;
	int status = count;
	struct twl6030_bci_device_info *di = dev_get_drvdata(dev);

	if ((strict_strtol(buf, 10, &val) < 0) || (val < 3500)
				|| (val > di->max_charger_voltagemV))
		return -EINVAL;
	di->regulation_voltagemV = val;
	twl6030_config_voreg_reg(di, val);

	return status;
}

static ssize_t show_regulation_voltage(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	unsigned int val;
	struct twl6030_bci_device_info *di = dev_get_drvdata(dev);

	val = di->regulation_voltagemV;
	return sprintf(buf, "%u\n", val);
}

#if defined(TWL6030_PMIC_CHARGING)
// we use exteranl charger. and no current control
static ssize_t set_termination_current(struct device *dev,
	struct device_attribute *attr, const char *buf, size_t count)
{
	long val;
	int status = count;
	struct twl6030_bci_device_info *di = dev_get_drvdata(dev);

	if ((strict_strtol(buf, 10, &val) < 0) || (val < 50) || (val > 400))
		return -EINVAL;
	di->termination_currentmA = val;
	twl6030_config_iterm_reg(di, val);

	return status;
}

static ssize_t show_termination_current(struct device *dev,
			struct device_attribute *attr, char *buf)
{
	unsigned int val;
	struct twl6030_bci_device_info *di = dev_get_drvdata(dev);

	val = di->termination_currentmA;
	return sprintf(buf, "%u\n", val);
}

static ssize_t set_cin_limit(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	long val;
	int status = count;
	struct twl6030_bci_device_info *di = dev_get_drvdata(dev);

	if ((strict_strtol(buf, 10, &val) < 0) || (val < 50) || (val > 1500))
		return -EINVAL;
	di->charger_incurrentmA = val;
	twl6030_config_cinlimit_reg(di, val);

	return status;
}

static ssize_t show_cin_limit(struct device *dev, struct device_attribute *attr,
								  char *buf)
{
	unsigned int val;
	struct twl6030_bci_device_info *di = dev_get_drvdata(dev);

	val = di->charger_incurrentmA;
	return sprintf(buf, "%u\n", val);
}

static ssize_t set_charge_current(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	long val;
	int status = count;
	struct twl6030_bci_device_info *di = dev_get_drvdata(dev);

	if ((strict_strtol(buf, 10, &val) < 0) || (val < 300)
				|| (val > di->max_charger_currentmA))
		return -EINVAL;
	di->charger_outcurrentmA = val;
	twl6030_config_vichrg_reg(di, val);

	return status;
}

static ssize_t show_charge_current(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	unsigned int val;
	struct twl6030_bci_device_info *di = dev_get_drvdata(dev);

	val = di->charger_outcurrentmA;
	return sprintf(buf, "%u\n", val);
}
#endif

static ssize_t set_min_vbus(struct device *dev, struct device_attribute *attr,
				  const char *buf, size_t count)
{
	long val;
	int status = count;
	struct twl6030_bci_device_info *di = dev_get_drvdata(dev);

	if ((strict_strtol(buf, 10, &val) < 0) || (val < 4200) || (val > 4760))
		return -EINVAL;
	di->min_vbus = val;
	twl6030_config_min_vbus_reg(di, val);

	return status;
}

static ssize_t show_min_vbus(struct device *dev, struct device_attribute *attr,
				  char *buf)
{
	unsigned int val;
	struct twl6030_bci_device_info *di = dev_get_drvdata(dev);

	val = di->min_vbus;
	return sprintf(buf, "%u\n", val);
}

#if defined(TWL6030_PMIC_CHARGING)
// we don't control current.
static ssize_t set_current_avg_interval(struct device *dev,
	  struct device_attribute *attr, const char *buf, size_t count)
{
	long val;
	int status = count;
	struct twl6030_bci_device_info *di = dev_get_drvdata(dev);

	if ((strict_strtol(buf, 10, &val) < 0) || (val < 10) || (val > 3600))
		return -EINVAL;
	di->current_avg_interval = val;
	twl6030_current_mode_changed(di);

	return status;
}

static ssize_t show_current_avg_interval(struct device *dev,
				  struct device_attribute *attr, char *buf)
{
	unsigned int val;
	struct twl6030_bci_device_info *di = dev_get_drvdata(dev);

	val = di->current_avg_interval;
	return sprintf(buf, "%u\n", val);
}
#endif

static ssize_t set_monitoring_interval(struct device *dev,
	  struct device_attribute *attr, const char *buf, size_t count)
{
	long val;
	int status = count;
	struct twl6030_bci_device_info *di = dev_get_drvdata(dev);

	if ((strict_strtol(buf, 10, &val) < 0) || (val < 10) || (val > 3600))
		return -EINVAL;
	di->monitoring_interval = val;
	twl6030_work_interval_changed(di);

	return status;
}

static ssize_t show_monitoring_interval(struct device *dev,
		  struct device_attribute *attr, char *buf)
{
	unsigned int val;
	struct twl6030_bci_device_info *di = dev_get_drvdata(dev);

	val = di->monitoring_interval;
	return sprintf(buf, "%u\n", val);
}

static ssize_t show_bsi(struct device *dev,
		  struct device_attribute *attr, char *buf)
{
	int val;

	val = twl6030_get_gpadc_conversion(0);
	return sprintf(buf, "%d\n", val);
}

static ssize_t show_stat1(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	unsigned val;
	struct twl6030_bci_device_info *di = dev_get_drvdata(dev);

	val = di->stat1;
	return sprintf(buf, "%u\n", val);
}

static ssize_t show_status_int1(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	unsigned val;
	struct twl6030_bci_device_info *di = dev_get_drvdata(dev);

	val = di->status_int1;
	return sprintf(buf, "%u\n", val);
}

static ssize_t show_status_int2(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	unsigned val;
	struct twl6030_bci_device_info *di = dev_get_drvdata(dev);

	val = di->status_int2;
	return sprintf(buf, "%u\n", val);
}

/* LGE_CHANGE_S [hyunhee.jeon@lge.com] 2011-02-07, battery bounce function set */
static ssize_t set_smpl_mode(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	long val;
	int status = count;
	u32 smpl_count = 0;

	/* VBAT low delay = 500 msec
	Minimum: delay x 40ms + 20ms
	Maximum: delay x 40ms + 30ms */
	u16 delay = 0xC;

	struct twl6030_bci_device_info *di = dev_get_drvdata(dev);

	if ((strict_strtol(buf, 10, &val) < 0) || (val > 2))
		return -EINVAL;

	di->smpl_en = val;

	/* Save smpl mode to nv data area */
	lge_dynamic_nvdata_write(LGE_NVDATA_DYNAMIC_SMPL_EN_OFFSET,
				&di->smpl_en, 1);

	if (di->smpl_en) {
		/* Select battery bounce restart mode */
		twl_i2c_write_u8(TWL_MODULE_PM_MASTER,
				BB_SEL | delay, PH_CFG_VBATLOW_REG_OFFSET);
	} else {
		/* Clear VBATLOW register */
		twl_i2c_write_u8(TWL_MODULE_PM_MASTER,
				0, PH_CFG_VBATLOW_REG_OFFSET);

		/* set smpl count to 0 */
		lge_dynamic_nvdata_write(LGE_NVDATA_DYNAMIC_SMPL_COUNT_OFFSET,
			&smpl_count, 4);
	}
	return status;
}

static ssize_t show_smpl_mode(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	int val;
	struct twl6030_bci_device_info *di = dev_get_drvdata(dev);

	/* read smpl saved value */
	lge_dynamic_nvdata_read(LGE_NVDATA_DYNAMIC_SMPL_EN_OFFSET,
				&di->smpl_en, 1);

	val = di->smpl_en;

	return sprintf(buf, "%d\n", di->smpl_en);
}

static ssize_t set_smpl_count(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	u32 val;
	int status = count;

	if (strict_strtol(buf, 10, &val) < 0)
		return -EINVAL;

	lge_dynamic_nvdata_write(LGE_NVDATA_DYNAMIC_SMPL_COUNT_OFFSET,
				&val, 4);

	return status;
}

static ssize_t show_smpl_count(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	u32 val;

	/* read smpl saved value */
	lge_dynamic_nvdata_read(LGE_NVDATA_DYNAMIC_SMPL_COUNT_OFFSET,
				&val, 4);

	return sprintf(buf, "%d\n", val);
}

/* LGE_CHANGE_E [hyunhee.jeon@lge.com] 2011-02-07 */

//roy.park@lge.com 2010.12.8 Start-->[
extern int abnormal_vl;
static ssize_t abnormal_wakelock_show(struct device* dev, struct device_attribute* attr, char* buf)
{
	return sprintf(buf,"%d\n", abnormal_vl);
}
//roy.park@lge.com <---]

//LGE_CHANGE_S [kibum.lee@lge.com] 2011-02-16, common : abnormal_wakelock enable
int abnormal_dis_vl;
static ssize_t abnormal_wakelock_dis_show(struct device* dev, struct device_attribute* attr, char* buf)
{
	return sprintf(buf,"%d\n", abnormal_dis_vl);
}

static ssize_t abnormal_wakelock_dis_set(struct device *dev,	struct device_attribute *attr, const char *buf, size_t count)
{
	long val;
	int status = count;

	if ((strict_strtol(buf, 10, &val) < 0) || (val > 2))
		return -EINVAL;

	abnormal_dis_vl = val;
	//printk("abnormal_wakelock_dis_set count=%d, abnormal_dis_vl=%d\n", count, abnormal_dis_vl);
	return status;
}
//LGE_CHANGE_E [kibum.lee@lge.com] 2011-02-16, common : abnormal_wakelock enable


#if defined(TWL6030_PMIC_FUELGAUGE)
static DEVICE_ATTR(fg_mode, S_IWUSR | S_IRUGO, show_fg_mode, set_fg_mode);
#endif
#if defined(TWL6030_PMIC_CHARGING)
// FIXME: [jongho3.lee@lge.com] charger src is shown in ac/usb attribute.
static DEVICE_ATTR(charge_src, S_IWUSR | S_IRUGO, show_charge_src,
		set_charge_src);
#endif
static DEVICE_ATTR(vbus_voltage, S_IRUGO, show_vbus_voltage, NULL);
static DEVICE_ATTR(id_level, S_IRUGO, show_id_level, NULL);
static DEVICE_ATTR(watchdog, S_IWUSR | S_IRUGO, show_watchdog, set_watchdog);
#if defined(TWL6030_PMIC_CHARGING)
//LGE_CHANGE [jongho3.lee@lge.com] we have external fuel gauge.
static DEVICE_ATTR(fg_counter, S_IRUGO, show_fg_counter, NULL);
static DEVICE_ATTR(fg_accumulator, S_IRUGO, show_fg_accumulator, NULL);
static DEVICE_ATTR(fg_offset, S_IRUGO, show_fg_offset, NULL);
static DEVICE_ATTR(fg_clear, S_IWUSR, NULL, set_fg_clear);
static DEVICE_ATTR(fg_cal, S_IWUSR, NULL, set_fg_cal);
#endif
static DEVICE_ATTR(charging, S_IWUSR | S_IRUGO, NULL, set_charging);
static DEVICE_ATTR(regulation_voltage, S_IWUSR | S_IRUGO,
		show_regulation_voltage, set_regulation_voltage);
#if defined(TWL6030_PMIC_CHARGING)
static DEVICE_ATTR(termination_current, S_IWUSR | S_IRUGO,
		show_termination_current, set_termination_current);
static DEVICE_ATTR(cin_limit, S_IWUSR | S_IRUGO, show_cin_limit,
		set_cin_limit);
static DEVICE_ATTR(charge_current, S_IWUSR | S_IRUGO, show_charge_current,
		set_charge_current);
#endif
static DEVICE_ATTR(min_vbus, S_IWUSR | S_IRUGO, show_min_vbus, set_min_vbus);
static DEVICE_ATTR(monitoring_interval, S_IWUSR | S_IRUGO,
		show_monitoring_interval, set_monitoring_interval);
#if defined(TWL6030_PMIC_FUELGAUGE)
static DEVICE_ATTR(current_avg_interval, S_IWUSR | S_IRUGO,
		show_current_avg_interval, set_current_avg_interval);
#endif
static DEVICE_ATTR(bsi, S_IRUGO, show_bsi, NULL);
static DEVICE_ATTR(stat1, S_IRUGO, show_stat1, NULL);
static DEVICE_ATTR(status_int1, S_IRUGO, show_status_int1, NULL);
static DEVICE_ATTR(status_int2, S_IRUGO, show_status_int2, NULL);
static DEVICE_ATTR(smpl_en, S_IWUSR | S_IRUGO, show_smpl_mode, set_smpl_mode);
static DEVICE_ATTR(smpl_count, S_IWUSR | S_IRUGO, show_smpl_count, set_smpl_count);
//roy.park@lge.com 2010.12.8 Start-->[
static DEVICE_ATTR(abnormal_wakelock, S_IWUSR | S_IRUGO, abnormal_wakelock_show, NULL);
//roy.park@lge.com <---]

//LGE_CHANGE_S [kibum.lee@lge.com] 2011-02-16, common : abnormal_wakelock enable
static DEVICE_ATTR(abnormal_wakelock_dis, S_IWUSR | S_IRUGO, abnormal_wakelock_dis_show, abnormal_wakelock_dis_set);
//LGE_CHANGE_E [kibum.lee@lge.com] 2011-02-16, common : abnormal_wakelock enable


static struct attribute *twl6030_bci_attributes[] = {
#if defined(TWL6030_PMIC_FUELGAUGE)
	&dev_attr_fg_mode.attr,
#endif
#if defined(TWL6030_PMIC_CHARGING)
// FIXME: [jongho3.lee@lge.com] charger src is shown in ac/usb attribute.
	&dev_attr_charge_src.attr,
#endif
	&dev_attr_vbus_voltage.attr,
	&dev_attr_id_level.attr,
	&dev_attr_watchdog.attr,
#if defined(TWL6030_PMIC_FUELGAUGE)
	&dev_attr_fg_counter.attr,
	&dev_attr_fg_accumulator.attr,
	&dev_attr_fg_offset.attr,
	&dev_attr_fg_clear.attr,
	&dev_attr_fg_cal.attr,
#endif
	&dev_attr_charging.attr,
	&dev_attr_regulation_voltage.attr,
#if defined(TWL6030_PMIC_CHARGING)
	&dev_attr_termination_current.attr,
	&dev_attr_cin_limit.attr,
	&dev_attr_charge_current.attr,
#endif
	&dev_attr_min_vbus.attr,
	&dev_attr_monitoring_interval.attr,
#if defined(TWL6030_PMIC_FUELGAUGE)
	&dev_attr_current_avg_interval.attr,
#endif
	&dev_attr_bsi.attr,
	&dev_attr_stat1.attr,
	&dev_attr_status_int1.attr,
	&dev_attr_status_int2.attr,
	&dev_attr_smpl_en.attr,
	&dev_attr_smpl_count.attr,
	&dev_attr_abnormal_wakelock.attr,	// //roy.park@lge.com
	&dev_attr_abnormal_wakelock_dis.attr,	// kibum.lee@lge.com
	NULL,
};

static const struct attribute_group twl6030_bci_attr_group = {
	.attrs = twl6030_bci_attributes,
};

static char *twl6030_bci_supplied_to[] = {
// [jongho3.lee@lge.com] attribute name should be battery for android. //end
	"battery",
};


static int twl6030_bci_battery_property_is_writeable(struct power_supply *psy,
						enum power_supply_property psp)
{
	switch (psp) {
	case POWER_SUPPLY_PROP_VOLTAGE_NOW:
	case POWER_SUPPLY_PROP_GAUGE_CONTROL:
	case POWER_SUPPLY_PROP_CHARGER_MODE:
	case POWER_SUPPLY_PROP_CHARGER_TEMP_CONTROL:
		return 1;

	default:
		break;
	}

	return 0;
}


/* MUIC mode */
typedef enum {
	MUIC_OPEN,              // 0
	MUIC_CHRGER,            // 1
	MUIC_FACTORY_MODE,      // 2
	MUIC_HAEDSET,           // 3
	MUIC_USB,               // 4
#if 0
	MUIC_TV_OUT,            // 5
	MUIC_FULL_USB,          // 6
	MUIC_OTG,               // 7
	MUIC_DEVELOP_MODE,      // 8
	MUIC_TA_1A,             // 9
	MUIC_RESERVE_1,
	MUIC_RESERVE_2,
#endif
	MUIC_END,
} UBOOT_MUIC_MODE_TYPE;



static int __init charger_state(char *str)
{
	int chg_status = simple_strtol(str, NULL, 0);
	int charging_mode;

	charging_mode = chg_status & 0x0F;
	start_cond = (chg_status & 0xF0);

	D("chg_state = %x", chg_status);

	switch(charging_mode)
	{
		case MUIC_OPEN :
			charging_mode = POWER_SUPPLY_TYPE_BATTERY;
			break;
		case MUIC_CHRGER:
			charging_mode = POWER_SUPPLY_TYPE_MAINS;
			break;
		case MUIC_USB:
			charging_mode = POWER_SUPPLY_TYPE_USB;
			break;
		case MUIC_FACTORY_MODE:
		default :
			charging_mode = POWER_SUPPLY_TYPE_FACTROY;
			break;
	}

	set_boot_charging_mode(charging_mode);
	return 1;
}
__setup("chg=", charger_state);

static int __devinit twl6030_bci_battery_probe(struct platform_device *pdev)
{
	struct twl4030_bci_platform_data *pdata = pdev->dev.platform_data;
	struct twl6030_bci_device_info *di;
	int irq;
	int ret;
	struct twl6030_gpadc_request req;
	u8 controller_stat = 0;

	D("twl6030_bci_battery_probe................");

	di = kzalloc(sizeof(*di), GFP_KERNEL);
	if (!di)
		return -ENOMEM;

//LGE_CHANGE_S [jongho3.lee@lge.com] expose di struct for external charger..
#if defined(CHARGING_WAKE_LOCK)
	wake_lock_init(&di->charger_wake_lock, WAKE_LOCK_SUSPEND, "charger_wakelock");
	wake_lock_init(&di->off_mode_charger_wake_lock, WAKE_LOCK_SUSPEND, "off_mode_charger_wakelock");
#if 0
	wake_lock_init(&di->charger_back_light_wake_lock, WAKE_LOCK_IDLE, "charger_light_wakelock");
#endif
#endif

	//LGE_CHANGE_S [jongho3.lee@lge.com] for fuel gauge reset on CP we need one gpio.

#if 1
	ret = gpio_request(OMAP_SEND, "gauge_reset");
	if (ret < 0) {
		printk(KERN_ERR "%s: Failed to request GPIO_%d for charging_ic\n", __func__, OMAP_SEND);
		return -ENOSYS;
	}
	gpio_direction_output(OMAP_SEND, 0);
#endif


	p_di = di;
	di->charger_interrupt = true;
	di->charge_status = POWER_SUPPLY_STATUS_UNKNOWN;
	di->capacity_validation = NOT_INIT_CAP;
	di->charger_source = POWER_SUPPLY_TYPE_BATTERY;
	di->charger_logo_status = CHARGER_LOGO_STATUS_UNKNOWN;
	di->wake_lock_count = 0;
	di->battery_present = true;
	di->temp_C = TEMP_LOW_NO_BAT - 100;
	di->off_mode_timer_working = 0;
	recharging_status = RECHARGING_WAIT_UNSET;
	di->gauge_control_count = 0;
	di->temp_control = 0xFF;
	di->dummy_cap = -1;
//[jongho3.lee] WBT test fix.
	//di->avg_volt_queue[AVG_VOLT_QUEUE_SIZE];
	di->avg_volt_queue_count = 0;
	di->avg_volt_head = 0;
	di->avg_volt_tail = 0;
	di->avg_voltage_mV = 0;
	di->pmic_capacity = 0;
	memset((di->avg_volt_queue), 0, sizeof(int)*AVG_VOLT_QUEUE_SIZE);
//LGE_CHANGE_E [jongho3.lee@lge.com] expose di struct for external charger..

	if (!pdata) {
		dev_dbg(&pdev->dev, "platform_data not available\n");
		ret = -EINVAL;
		goto err_pdata;
	}

	if (pdata->monitoring_interval == 0) {
		di->monitoring_interval = 10;
		di->current_avg_interval = 10;
	} else {
		di->monitoring_interval = pdata->monitoring_interval;
		di->current_avg_interval = pdata->monitoring_interval;
	}

	di->max_charger_currentmA = pdata->max_charger_currentmA;
	di->max_charger_voltagemV = pdata->max_bat_voltagemV;
	di->termination_currentmA = pdata->termination_currentmA;
	di->regulation_voltagemV = pdata->max_bat_voltagemV;
	di->low_bat_voltagemV = pdata->low_bat_voltagemV;
	di->battery_tmp_tbl = pdata->battery_tmp_tbl;
	di->tblsize = pdata->tblsize;

	di->dev = &pdev->dev;
	// [jongho3.lee@lge.com] attribute name should be battery for android.
	di->bat.name = "battery";
	di->bat.supplied_to = twl6030_bci_supplied_to;
	di->bat.num_supplicants = ARRAY_SIZE(twl6030_bci_supplied_to);
	di->bat.type = POWER_SUPPLY_TYPE_BATTERY;
	di->bat.properties = twl6030_bci_battery_props;
	di->bat.num_properties = ARRAY_SIZE(twl6030_bci_battery_props);
	di->bat.get_property = twl6030_bci_battery_get_property;
// [jongho3.lee@lge.com] add to set_property. //start
	di->bat.set_property = twl6030_bci_battery_set_property;
	di->bat.property_is_writeable = twl6030_bci_battery_property_is_writeable;
// [jongho3.lee@lge.com] add to set_property. //end
	di->bat.external_power_changed =
			twl6030_bci_battery_external_power_changed;

// LGE_CHANGE [jongho3.lee@lge.com] replacing charger usb_bat supply type. [START_LGE]
	di->usb.name = "usb";
	di->usb.type = POWER_SUPPLY_TYPE_USB;
	di->usb.properties = twl6030_usb_props;
	di->usb.num_properties = ARRAY_SIZE(twl6030_usb_props);
	di->usb.get_property = twl6030_usb_get_property;

// LGE_CHANGE [jongho3.lee@lge.com] replacing charger usb_bat supply type. [START_LGE]
	di->ac.name = "ac";
	di->ac.type = POWER_SUPPLY_TYPE_MAINS;
	di->ac.properties = twl6030_ac_props;
	di->ac.num_properties = ARRAY_SIZE(twl6030_ac_props);
	di->ac.get_property = twl6030_ac_get_property;

// [jongho3.lee@lge.com] attribute name should be bk_battery for android. //start
	di->bk_bat.name = "bk_battery";
// [jongho3.lee@lge.com] attribute name should be bk_battery for android. //end
	di->bk_bat.type = POWER_SUPPLY_TYPE_BATTERY;
	di->bk_bat.properties = twl6030_bk_bci_battery_props;
	di->bk_bat.num_properties = ARRAY_SIZE(twl6030_bk_bci_battery_props);
	di->bk_bat.get_property = twl6030_bk_bci_battery_get_property;

	di->vac_priority = 2;
	platform_set_drvdata(pdev, di);

	/* settings for temperature sensing */
	ret = twl6030battery_temp_setup();
	if (ret)
		goto temp_setup_fail;

#if defined(TWL6030_PMIC_ENABLE_CHARGING)
	/* request charger fault interruption */
	irq = platform_get_irq(pdev, 1);
	ret = request_threaded_irq(irq, NULL, twl6030charger_fault_interrupt,
		0, "twl_bci_fault", di);
	if (ret) {
		dev_dbg(&pdev->dev, "could not request irq %d, status %d\n",
			irq, ret);
		goto temp_setup_fail;
	}
#endif

	/* request charger ctrl interruption */
	irq = platform_get_irq(pdev, 0);
	ret = request_threaded_irq(irq, NULL, twl6030charger_ctrl_interrupt,
		0, "twl_bci_ctrl", di);

	if (ret) {
		dev_dbg(&pdev->dev, "could not request irq %d, status %d\n",
			irq, ret);
		goto chg_irq_fail;
	}

	ret = power_supply_register(&pdev->dev, &di->bat);
	if (ret) {
		dev_dbg(&pdev->dev, "failed to register main battery\n");
		goto batt_failed;
	}

	BLOCKING_INIT_NOTIFIER_HEAD(&notifier_list);

#if defined(CREATE_OWN_WORKQUEUE)
	di->charger_workqueue = create_workqueue("cosmo_charger_workqueue");
#endif

	INIT_DELAYED_WORK_DEFERRABLE(&di->twl6030_bci_monitor_work,
				twl6030_bci_battery_work);
	charger_schedule_delayed_work(&di->twl6030_bci_monitor_work, HZ*2);

#if defined(CONFIG_LG_FW_RT9524_CHARGER)
	INIT_DELAYED_WORK_DEFERRABLE(&di->charger_work,
				charging_battery_work);
	charger_schedule_delayed_work(&di->charger_work, HZ*3);

	INIT_DELAYED_WORK_DEFERRABLE(&di->gauge_reset_work,
				battery_gauge_reset_work);

	INIT_DELAYED_WORK_DEFERRABLE(&di->power_off_work,
				battery_power_off_work);

	INIT_DELAYED_WORK_DEFERRABLE(&di->charger_timer_work,
				charging_timer_work);
#endif

	ret = power_supply_register(&pdev->dev, &di->usb);
	if (ret) {
		dev_dbg(&pdev->dev, "failed to register usb power supply\n");
		goto usb_failed;
	}

	ret = power_supply_register(&pdev->dev, &di->ac);
	if (ret) {
		dev_dbg(&pdev->dev, "failed to register ac power supply\n");
		goto ac_failed;
	}

	ret = power_supply_register(&pdev->dev, &di->bk_bat);
	if (ret) {
		dev_dbg(&pdev->dev, "failed to register backup battery\n");
		goto bk_batt_failed;
	}
	di->charge_n1 = 0;
	di->timer_n1 = 0;


#if defined(TWL6030_PMIC_CHARGING)
//LGE_CHANGE [jongho3.lee@lge.com] we don't need and don't have the way to monitor bat_current.
	INIT_DELAYED_WORK_DEFERRABLE(&di->twl6030_current_avg_work,
						twl6030_current_avg);
	charger_schedule_delayed_work(&di->twl6030_current_avg_work, 500);
#endif

	ret = twl6030battery_voltage_setup();
	if (ret)
	{
		dev_dbg(&pdev->dev, "voltage measurement setup failed\n");
		D("voltage measurement setup failed\n");
	}
	else
	{
		D("voltage measurement setup done\n");
	}

#if defined(TWL6030_PMIC_CHARGING)
//LGE_CHANGE [jongho3.lee@lge.com] we don't need and don't have the way to monitor bat_current.
	ret = twl6030battery_current_setup();
	if (ret)
		dev_dbg(&pdev->dev, "current measurement setup failed\n");
#endif

	/* initialize for USB charging */
	twl6030_config_limit1_reg(di, pdata->max_charger_voltagemV);
	twl6030_config_limit2_reg(di, di->max_charger_currentmA);
	twl_i2c_write_u8(TWL6030_MODULE_CHARGER, MBAT_TEMP,
						CONTROLLER_INT_MASK);
	twl_i2c_write_u8(TWL6030_MODULE_CHARGER, MASK_MCHARGERUSB_THMREG,
						CHARGERUSB_INT_MASK);

	twl_i2c_read_u8(TWL6030_MODULE_CHARGER, &controller_stat,
		CONTROLLER_STAT1);

	di->charger_incurrentmA = di->max_charger_currentmA;
	di->charger_outcurrentmA = di->max_charger_currentmA;
	di->watchdog_duration = 32;
	di->voltage_uV = twl6030_get_gpadc_conversion(7);
	dev_info(&pdev->dev, "Battery Voltage at Bootup is %d mV\n",
							di->voltage_uV);

//LGE_CHANGE [jongho3.lee@lge.com] we use external charger.
#if defined(TWL6030_PMIC_ENABLE_CHARGING)
	if (controller_stat & VBUS_DET) {
		di->usb_online = POWER_SUPPLY_TYPE_USB;
		twl6030_start_usb_charger(di);
	}

	if (controller_stat & VAC_DET) {
		di->ac_online = POWER_SUPPLY_TYPE_MAINS;
		twl6030_start_ac_charger(di);
	}
#endif

	ret = twl6030backupbatt_setup();
	if (ret)
		dev_dbg(&pdev->dev, "Backup Bat charging setup failed\n");

	twl6030_interrupt_unmask(TWL6030_CHARGER_CTRL_INT_MASK,
						REG_INT_MSK_LINE_C);
	twl6030_interrupt_unmask(TWL6030_CHARGER_CTRL_INT_MASK,
						REG_INT_MSK_STS_C);
	/*
	twl6030_interrupt_unmask(TWL6030_CHARGER_FAULT_INT_MASK,
						REG_INT_MSK_LINE_C);
	twl6030_interrupt_unmask(TWL6030_CHARGER_FAULT_INT_MASK,
						REG_INT_MSK_STS_C);
	*/

	ret = sysfs_create_group(&pdev->dev.kobj, &twl6030_bci_attr_group);
	if (ret)
		dev_dbg(&pdev->dev, "could not create sysfs files\n");

//FIXME : [jongho3.lee@lge.com] need to fix with PMIC datasheet which I don't have now.

#if defined(GET_BET_TEMP)
	{
		int adc_code;

		req.channels = (1 << BAT_TEMP_CHANNEL);
		req.method = TWL6030_GPADC_SW2;
		req.active = 0;
		req.func_cb = NULL;

		get_adc_value(&req, req.channels);

		adc_code = req.rbuf[BAT_TEMP_CHANNEL];
		di->temp_C = average_temp(reference_graph((s64)adc_code, battery_temp_graph, ARRAY_SIZE(battery_temp_graph)) / (TEMP_TIMES / 10));
		if(di->temp_C < TEMP_LOW_NO_BAT)
		{
			di->battery_present = 0;
		}
	}
#endif

	// mask vbat low interrupt
	twl6030_interrupt_mask(TWL6030_VBAT_LOW_MASK, REG_INT_MSK_LINE_A);

	//unmask PREQ transition
	twl_i2c_write_u8(0x0D, 0x00, 0x20);

	//USB_VBUS_CTRL_CLR
	twl_i2c_write_u8(0x0E, 0xFF, 0x05);
	//USB_ID_CRTL_CLR
	twl_i2c_write_u8(0x0E, 0xFF, 0x07);
	twl_i2c_write_u8(0x0D, 0xC3, 0x31);

	//MISC1
	//twl_i2c_write_u8(0x0D, 0x00, 0xE4);  // TODO. after add VBAT_MEAS, david.seo, kibum.lee


	//BBSPOR_CFG - Disable BB charging. It should be taken care by proper driver
//	twl_i2c_write_u8(0x0D, 0x62, 0xE6);

	//CFG_INPUT_PUPD2
	twl_i2c_write_u8(0x0D, 0x65, 0xF1);

	//CFG_INPUT_PUPD4
	twl_i2c_write_u8(0x0D, 0x00, 0xF3);

	//CFG_LDO_PD2
	twl_i2c_write_u8(0x0D, 0x00, 0xF5);

	//CHARGERUSB_CTRL3
	twl_i2c_write_u8(0x0E, 0x21, 0xEA);

	//TOGGLE2  100uA
	twl_i2c_write_u8(0x0E, 0x40, 0x91);
	//TOGGLE3
	twl_i2c_write_u8(0x0E, 0x00, 0x92);

	//Broadcast sleep state to MOD, CON
	twl_i2c_write_u8(0x0D, 0xC3, 0x31);

	//CLK32KG
	twl_i2c_write_u8(0x0D, 0x01, 0xBC);
	twl_i2c_write_u8(0x0D, 0x05, 0xBD);
	twl_i2c_write_u8(0x0D, 0x21, 0xBE);

    //CLK32KAUDIO
	twl_i2c_write_u8(0x0D, 0x01, 0xBF);
	twl_i2c_write_u8(0x0D, 0x05, 0xC0);
	twl_i2c_write_u8(0x0D, 0x21, 0xC1);

	//VUSB
	twl_i2c_write_u8(0x0D, 0x01, 0xA0);
	twl_i2c_write_u8(0x0D, 0x01, 0xA1);
	twl_i2c_write_u8(0x0D, 0x21, 0xA2);

	//VPP
	twl_i2c_write_u8(0x0D, 0x7F, 0xF4); //disable internal PD for VPP LDO
	twl_i2c_write_u8(0x0D, 0x00, 0x9C);
	twl_i2c_write_u8(0x0D, 0x00, 0x9D);
	twl_i2c_write_u8(0x0D, 0x00, 0x9E);

#if defined(CONFIG_MACH_LGE_VMMC_ALWAYSON_FORCED)	//20110504 FW1 KIMBYUNGCHUL SD_CARD_LOCKUP_IN_omap_hsmmc_resume_FUNC	 [START]

twl_i2c_write_u8(0x0D, 0x01, 0x98);
twl_i2c_write_u8(0x0D, 0x15, 0x99);//twl_i2c_write_u8(0x0D, 0x15, 0x99);
twl_i2c_write_u8(0x0D, 0xE1, 0x9A);

	
#endif	//CONFIG_MACH_LGE_VMMC_ALWAYSON_FORCED		//20110504 FW1 KIMBYUNGCHUL SD_CARD_LOCKUP_IN_omap_hsmmc_resume_FUNC	 [END]

	return 0;

bk_batt_failed:
	power_supply_unregister(&di->ac);
ac_failed:
	power_supply_unregister(&di->usb);
usb_failed:
	cancel_delayed_work(&di->twl6030_bci_monitor_work);
	power_supply_unregister(&di->bat);
batt_failed:
	free_irq(irq, di);
chg_irq_fail:
	irq = platform_get_irq(pdev, 1);
	free_irq(irq, di);
temp_setup_fail:
	platform_set_drvdata(pdev, NULL);
err_pdata:
	kfree(di);

	return ret;
}

static int __devexit twl6030_bci_battery_remove(struct platform_device *pdev)
{
	struct twl6030_bci_device_info *di = platform_get_drvdata(pdev);
	int irq;

	twl6030_interrupt_mask(TWL6030_CHARGER_CTRL_INT_MASK,
						REG_INT_MSK_LINE_C);
	twl6030_interrupt_mask(TWL6030_CHARGER_CTRL_INT_MASK,
						REG_INT_MSK_STS_C);
	twl6030_interrupt_mask(TWL6030_CHARGER_FAULT_INT_MASK,
						REG_INT_MSK_LINE_C);
	twl6030_interrupt_mask(TWL6030_CHARGER_FAULT_INT_MASK,
						REG_INT_MSK_STS_C);

	irq = platform_get_irq(pdev, 0);
	free_irq(irq, di);

#if defined(TWL6030_PMIC_ENABLE_CHARGING)
	irq = platform_get_irq(pdev, 1);
	free_irq(irq, di);
#endif

	sysfs_remove_group(&pdev->dev.kobj, &twl6030_bci_attr_group);
	cancel_delayed_work(&di->twl6030_bci_monitor_work);
	cancel_delayed_work(&di->twl6030_current_avg_work);
	flush_scheduled_work();
	power_supply_unregister(&di->bat);
	power_supply_unregister(&di->usb);
	power_supply_unregister(&di->ac);
	power_supply_unregister(&di->bk_bat);
	platform_set_drvdata(pdev, NULL);
	kfree(di);

	return 0;
}

static void twl6030_bci_battery_shutdown(struct platform_device *pdev)
{
	struct twl6030_bci_device_info *di = platform_get_drvdata(pdev);
	int irq;

	twl6030_interrupt_mask(TWL6030_CHARGER_CTRL_INT_MASK,
						REG_INT_MSK_LINE_C);
	twl6030_interrupt_mask(TWL6030_CHARGER_CTRL_INT_MASK,
						REG_INT_MSK_STS_C);
	twl6030_interrupt_mask(TWL6030_CHARGER_FAULT_INT_MASK,
						REG_INT_MSK_LINE_C);
	twl6030_interrupt_mask(TWL6030_CHARGER_FAULT_INT_MASK,
						REG_INT_MSK_STS_C);

	irq = platform_get_irq(pdev, 0);
	free_irq(irq, di);

#if defined(TWL6030_PMIC_ENABLE_CHARGING)
	irq = platform_get_irq(pdev, 1);
	free_irq(irq, di);
#endif

	sysfs_remove_group(&pdev->dev.kobj, &twl6030_bci_attr_group);
	cancel_delayed_work(&di->twl6030_bci_monitor_work);
	cancel_delayed_work(&di->twl6030_current_avg_work);
	flush_scheduled_work();

	//[jongho3.lee@lge.com]
#if 1
	charging_ic_deactive();
	p_di->charge_status = POWER_SUPPLY_STATUS_DISCHARGING;
#endif


	power_supply_unregister(&di->bat);
	power_supply_unregister(&di->usb);
	power_supply_unregister(&di->ac);
	power_supply_unregister(&di->bk_bat);
	platform_set_drvdata(pdev, NULL);
	kfree(di);

	return;
}

#ifdef CONFIG_PM
static int twl6030_bci_battery_suspend(struct platform_device *pdev,
	pm_message_t state)
{
	struct twl6030_bci_device_info *di = platform_get_drvdata(pdev);
	u8 rd_reg;

	/* mask to prevent wakeup due to 32s timeout from External charger */
	twl_i2c_read_u8(TWL6030_MODULE_CHARGER, &rd_reg, CONTROLLER_INT_MASK);
	rd_reg |= MVAC_FAULT;
	twl_i2c_write_u8(TWL6030_MODULE_CHARGER, MBAT_TEMP,
						CONTROLLER_INT_MASK);

	cancel_delayed_work(&di->twl6030_bci_monitor_work);
	cancel_delayed_work(&di->twl6030_current_avg_work);
	return 0;
}

static int twl6030_bci_battery_resume(struct platform_device *pdev)
{
	struct twl6030_bci_device_info *di = platform_get_drvdata(pdev);
	u8 rd_reg;

//LGE_CHANGE_S [david.seo@lge.com] 2011-03-26, common : move from pm44xx.c
	//GPADC_CTRL
	twl_i2c_write_u8(0x0E, 0xFF, 0x2E);
	//TOGGLE1
	twl_i2c_write_u8(0x0E, 0xA2, 0x90);
	//MISC2
	twl_i2c_write_u8(0x0D, 0x10, 0xE5);
//LGE_CHANGE_ [david.seo@lge.com] 2011-03-26, common : move from pm44xx.c

	twl_i2c_read_u8(TWL6030_MODULE_CHARGER, &rd_reg, CONTROLLER_INT_MASK);
	rd_reg &= ~(0xFF & MVAC_FAULT);
	twl_i2c_write_u8(TWL6030_MODULE_CHARGER, MBAT_TEMP,
						CONTROLLER_INT_MASK);

	charger_schedule_delayed_work(&di->twl6030_bci_monitor_work, 0);
	charger_schedule_delayed_work(&di->twl6030_current_avg_work, 50);
	return 0;
}
#else
#define twl6030_bci_battery_suspend	NULL
#define twl6030_bci_battery_resume	NULL
#endif /* CONFIG_PM */

static struct platform_driver twl6030_bci_battery_driver = {
	.probe		= twl6030_bci_battery_probe,
	.remove		= __devexit_p(twl6030_bci_battery_remove),
	.shutdown   = twl6030_bci_battery_shutdown,
	.suspend	= twl6030_bci_battery_suspend,
	.resume		= twl6030_bci_battery_resume,
	.driver		= {
		.name	= "twl6030_bci",
	},
};

static int __init twl6030_battery_init(void)
{
	return platform_driver_register(&twl6030_bci_battery_driver);
}
module_init(twl6030_battery_init);

static void __exit twl6030_battery_exit(void)
{
	platform_driver_unregister(&twl6030_bci_battery_driver);
}
module_exit(twl6030_battery_exit);

MODULE_LICENSE("GPL");
MODULE_ALIAS("platform:twl6030_bci");
MODULE_AUTHOR("Texas Instruments Inc");
