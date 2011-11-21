/*
 * Charging IC driver (rt9524)
 *
 * Copyright (C) 2010 LGE, Inc.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA 02111-1307 USA
 *
 */

#include <linux/platform_device.h>
#include <linux/interrupt.h>
#include <mach/gpio.h>
#include <linux/delay.h>
#include <linux/hrtimer.h>
#include <linux/proc_fs.h>		// might need to get fuel gauge info
#include <linux/power_supply.h>		// might need to get fuel gauge info
#include <linux/i2c/twl.h>
//#include "../staging/android//timed_output.h"
#include <linux/cosmo/charger_rt9524.h>
#include <linux/cosmo/cosmo_muic.h>

#define CHR_IC_DEALY				300	/* 200 us */
#define CHR_IC_SET_DEALY			1700	/* 1500 us */
#define CHR_TIMER_SECS				3600 /* 7200 secs*/

//#define TEMP_NO_BATTERY  1   //[jongho3.lee@lge.com] in kernel muic doesn't work now..

static DEFINE_MUTEX(charging_lock);

enum power_supply_type charging_ic_status;
//static recharging_state_t recharging_status;
static int bat_soc;
struct delayed_work	charger_timer_work;

/* Function Prototype */
static void charging_ic_intialize(void);
static irqreturn_t charging_ic_interrupt_handler(int irq, void *data);

struct timer_list charging_timer;

void charging_timer_func(unsigned long try)
{
	u32 wait;

	// [jongho3.lee@lge.ocm] FIXME : get battery SOC from fuel gague
	bat_soc = get_bat_soc();

	if(charging_timer.data > 3 || bat_soc > 99)
	{
		charger_schedule_delayed_work(&charger_timer_work, 0);
		return;
	}
	else
	{
		charging_timer.data += 1;
		//wait = (HZ*CHR_TIMER_SECS * (100 + 20 - bat_soc)) / 100;
		wait = HZ*CHR_TIMER_SECS;
		charging_timer.expires += wait;

		add_timer(&charging_timer);
	}
}


/*
void set_charging_ic_bat_soc(int soc)
{
	bat_soc = soc;
	return ;
}
EXPORT_SYMBOL(set_charging_ic_bat_soc);
*/

enum power_supply_type get_charging_ic_status()
{
	return charging_ic_status;
}
EXPORT_SYMBOL(get_charging_ic_status);

void charging_ic_active_default()
{
	u32 wait;

	if(charging_ic_status == POWER_SUPPLY_TYPE_USB)
	{
		D("[charger_rt9524]:: it's already  %s mode!! \n", __func__);
		return;
	}
#if defined(TEMP_NO_BATTERY)
	charging_ic_status = POWER_SUPPLY_TYPE_USB;
	D("[charger_rt9524]:: TEMP_NO_BATTERY %s: \n", __func__);
	return;
#endif

	if( charging_ic_status != POWER_SUPPLY_TYPE_BATTERY)
	{
		charging_ic_deactive();
	}

	mutex_lock(&charging_lock);

	gpio_set_value(CHG_EN_SET_N_OMAP, 0);

	udelay(CHR_IC_DEALY);

	charging_ic_status = POWER_SUPPLY_TYPE_USB;

	mutex_unlock(&charging_lock);

	// [jongho3.lee@lge.ocm] FIXME : get battery SOC from fuel gague
	bat_soc = get_bat_soc();
// [jongho3.lee@lge.com] charging timer setting

	init_timer(&charging_timer);
	charging_timer.data = 0;
	//wait = (HZ*CHR_TIMER_SECS * (100 + 20 - bat_soc)) / 100;
	wait = HZ*CHR_TIMER_SECS;
	charging_timer.expires = jiffies + wait;
	charging_timer.function = charging_timer_func;
	add_timer(&charging_timer);
// [jongho3.lee@lge.com] charging timer setting
	

	D("[charger_rt9524]::  %s: \n", __func__);

}
EXPORT_SYMBOL(charging_ic_active_default);

void charging_ic_set_ta_mode()
{
	u32 wait;

	if(charging_ic_status == POWER_SUPPLY_TYPE_MAINS)
	{
		D("[charger_rt9524]:: it's already  %s mode!! \n", __func__);
		return;
	}

#if defined(TEMP_NO_BATTERY)
	charging_ic_status = POWER_SUPPLY_TYPE_MAINS;
	D("[charger_rt9524]:: TEMP_NO_BATTERY %s: \n", __func__);
	return;
#endif

	if( charging_ic_status != POWER_SUPPLY_TYPE_BATTERY)
	{
		charging_ic_deactive();
	}

	mutex_lock(&charging_lock);

	gpio_set_value(CHG_EN_SET_N_OMAP, 0);
	udelay(CHR_IC_SET_DEALY);
	gpio_set_value(CHG_EN_SET_N_OMAP, 1);
	udelay(CHR_IC_DEALY);
	gpio_set_value(CHG_EN_SET_N_OMAP, 0);

	udelay(CHR_IC_SET_DEALY);

	charging_ic_status = POWER_SUPPLY_TYPE_MAINS;

	mutex_unlock(&charging_lock);

	// [jongho3.lee@lge.ocm] FIXME : get battery SOC from fuel gague
	bat_soc = get_bat_soc();
// [jongho3.lee@lge.com] charging timer setting

	init_timer(&charging_timer);
	charging_timer.data = 0;
	//wait = (HZ*CHR_TIMER_SECS * (100 + 20 - bat_soc)) / 100;
	wait = HZ*CHR_TIMER_SECS;
	charging_timer.expires = jiffies + wait;
	charging_timer.function = charging_timer_func;
	add_timer(&charging_timer);
// [jongho3.lee@lge.com] charging timer setting
	
	D("[charger_rt9524]::  %s: \n", __func__);
}
EXPORT_SYMBOL(charging_ic_set_ta_mode);

void charging_ic_set_usb_mode()
{
	charging_ic_active_default();
}
EXPORT_SYMBOL(charging_ic_set_usb_mode);

void charging_ic_set_factory_mode()
{
	u32 wait;

	if( (charging_ic_status == POWER_SUPPLY_TYPE_FACTROY) \
		|| (charging_ic_status == POWER_SUPPLY_TYPE_UNKNOWN))
	{
		charging_ic_status = POWER_SUPPLY_TYPE_FACTROY;
		D("[charger_rt9524]:: it's already  %s mode!! \n", __func__);
		return;
	}

#if defined(TEMP_NO_BATTERY)
	charging_ic_status = POWER_SUPPLY_TYPE_FACTROY;
	D("[charger_rt9524]:: TEMP_NO_BATTERY %s: \n", __func__);
	return;
#endif

	mutex_lock(&charging_lock);

	gpio_set_value(CHG_EN_SET_N_OMAP, 0);
	udelay(CHR_IC_SET_DEALY);
	gpio_set_value(CHG_EN_SET_N_OMAP, 1);
	udelay(CHR_IC_DEALY);
	gpio_set_value(CHG_EN_SET_N_OMAP, 0);
	udelay(CHR_IC_DEALY);
	gpio_set_value(CHG_EN_SET_N_OMAP, 1);
	udelay(CHR_IC_DEALY);
	gpio_set_value(CHG_EN_SET_N_OMAP, 0);
	udelay(CHR_IC_DEALY);
	gpio_set_value(CHG_EN_SET_N_OMAP, 1);
	udelay(CHR_IC_DEALY);
	gpio_set_value(CHG_EN_SET_N_OMAP, 0);

	udelay(CHR_IC_SET_DEALY);

	charging_ic_status = POWER_SUPPLY_TYPE_FACTROY;

	mutex_unlock(&charging_lock);

	// [jongho3.lee@lge.ocm] FIXME : get battery SOC from fuel gauge
	bat_soc = get_bat_soc();
// [jongho3.lee@lge.com] charging timer setting

	init_timer(&charging_timer);
	charging_timer.data = 0;
	//wait = (HZ*CHR_TIMER_SECS * (100 + 20 - bat_soc)) / 100;
	wait = HZ*CHR_TIMER_SECS;
	charging_timer.expires = jiffies + wait;
	charging_timer.function = charging_timer_func;
	add_timer(&charging_timer);
// [jongho3.lee@lge.com] charging timer setting
	
	D("[charger_rt9524]::  %s: \n", __func__);
}
EXPORT_SYMBOL(charging_ic_set_factory_mode);

void charging_ic_deactive()
{

	D("[charger_rt9524]::  %s: \n", __func__);
#if defined(TEMP_NO_BATTERY)
	charging_ic_status = POWER_SUPPLY_TYPE_BATTERY;
	D("[charger_rt9524]::  %s: \n", __func__);
	return;
#endif
	mutex_lock(&charging_lock);

	gpio_set_value(CHG_EN_SET_N_OMAP, 1);

	udelay(CHR_IC_SET_DEALY);

	charging_ic_status = POWER_SUPPLY_TYPE_BATTERY;

	mutex_unlock(&charging_lock);

// [jongho3.lee@lge.com] charging timer setting
	del_timer(&charging_timer);
// [jongho3.lee@lge.com] charging timer setting


}
EXPORT_SYMBOL(charging_ic_deactive);

static void charging_ic_initalize(void)
{
	//charging_ic_status = POWER_SUPPLY_TYPE_UNKNOWN;
}

static irqreturn_t charging_ic_interrupt_handler(int irq, void *data)
{
	struct delayed_work* charger_work;


	charger_work = get_charger_work();

	D("charging_ic interrupt occured!, %d \n", charger_work );

	schedule_delayed_work(charger_work, 0);


	return IRQ_HANDLED;
}

static void charging_timer_work(struct work_struct *work)
{
	charger_fsm(CHARG_FSM_CAUSE_CHARGING_TIMER_EXPIRED);
}

void set_boot_charging_mode(int charging_mode)
{
	charging_ic_status = charging_mode;
}
EXPORT_SYMBOL(set_boot_charging_mode);



static int charging_ic_probe(struct platform_device *dev)
{
	int ret = 0;

	ret = gpio_request(CHG_EN_SET_N_OMAP, "charging_ic_en");
	if (ret < 0) {
		printk(KERN_ERR "%s: Failed to request GPIO_%d for charging_ic\n", __func__, CHG_EN_SET_N_OMAP);
		return -ENOSYS;
	}
	gpio_direction_output(CHG_EN_SET_N_OMAP, 0);

#if CHG_DONE_INT
	//[jongho3.lee@lge.com] This interrupt will not be used.
	ret = gpio_request(CHG_STATUS_N_OMAP, "charging_ic_status");
	if (ret < 0) {
		printk(KERN_ERR "%s: Failed to request GPIO_%d for charging_ic_status\n", __func__, CHG_STATUS_N_OMAP);
		goto err_gpio_request_failed;
	}
	gpio_direction_input(CHG_STATUS_N_OMAP);

	ret = request_irq(gpio_to_irq(CHG_STATUS_N_OMAP), charging_ic_interrupt_handler,  IRQF_TRIGGER_RISING,  "Charging_ic_driver", 	NULL);
	if (ret < 0) {
		printk(KERN_ERR "%s: Failed to request IRQ for charging_ic_status\n", __func__);
		goto err_request_irq_failed;
	}
#endif

	INIT_DELAYED_WORK_DEFERRABLE(&charger_timer_work,
				charging_timer_work);

	//charging_ic_initalize();
	printk("LGE: charging_ic Initialization is done\n");

	return 0;

err_request_irq_failed:
	gpio_free(CHG_STATUS_N_OMAP);
err_gpio_request_failed:
	gpio_free(CHG_EN_SET_N_OMAP);

	printk("+++ LGE: Charging IC probe failed\n");
	return ret;
}

static int charging_ic_remove(struct platform_device *dev)
{
	charging_ic_deactive();

	free_irq(CHG_STATUS_N_OMAP, NULL);

	gpio_free(CHG_EN_SET_N_OMAP);
	gpio_free(CHG_STATUS_N_OMAP);

	return 0;
}

static int charging_ic_suspend(struct platform_device *dev, pm_message_t state)
{
	printk("charging_ic_suspend \n");
	dev->dev.power.power_state = state;
	return 0;
}

static int charging_ic_resume(struct platform_device *dev)
{
	printk("charging_ic_resume \n");
	dev->dev.power.power_state = PMSG_ON;
	return 0;
}

static struct platform_driver charging_ic_driver = {
	.probe = charging_ic_probe,
	.remove = charging_ic_remove,
	.suspend = charging_ic_suspend,
	.resume= charging_ic_resume,
	.driver = {
		.name = "cosmo_charger",
	},
};

static int __init charging_ic_init(void)
{
	printk("LGE: charging_ic Driver Init\n");
	return platform_driver_register(&charging_ic_driver);
}

static void __exit charging_ic_exit(void)
{
	printk("LGE: charging_ic Driver Exit\n");
	platform_driver_unregister(&charging_ic_driver);
}

module_init(charging_ic_init);
module_exit(charging_ic_exit);

MODULE_AUTHOR("LG Electronics");
MODULE_DESCRIPTION("charging_ic Driver");
MODULE_LICENSE("GPL");
