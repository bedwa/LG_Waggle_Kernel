/**
 * twl4030-pwrbutton.c - TWL4030 Power Button Input Driver
 *
 * Copyright (C) 2008-2009 Nokia Corporation
 *
 * Written by Peter De Schrijver <peter.de-schrijver@nokia.com>
 * Several fixes by Felipe Balbi <felipe.balbi@nokia.com>
 *
 * This file is subject to the terms and conditions of the GNU General
 * Public License. See the file "COPYING" in the main directory of this
 * archive for more details.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 */

#include <linux/module.h>
#include <linux/delay.h>
#include <linux/init.h>
#include <linux/kernel.h>
#include <linux/errno.h>
#include <linux/input.h>
#include <linux/interrupt.h>
#include <linux/slab.h>
#include <linux/platform_device.h>
#include <linux/i2c/twl.h>

#define PWR_PWRON_IRQ (1 << 0)

//LGE Change_S 2010.11.17 
//#define STS_HW_CONDITIONS 0xf
#define STS_HW_CONDITIONS 0x2
//LGE Change_E 2010.11.17 

struct twl4030_pwrbutton_data {
	int irq;

	struct work_struct irq_work;
	struct workqueue_struct	 *irq_wq;
	struct input_dev *input_dev;
};

static void powerbutton_work_func(struct work_struct *work)
{
	int err;
	u8 value = 0;
	static int previousKey = 0;
	struct twl4030_pwrbutton_data *data = container_of(work, struct twl4030_pwrbutton_data, irq_work);

	err = twl_i2c_read_u8(TWL4030_MODULE_PM_MASTER, &value,	STS_HW_CONDITIONS);

	printk("%s [%d][%d][%d]\n", __func__, err, !(value & PWR_PWRON_IRQ),previousKey);
	if (!err)  {
		if((!(value & PWR_PWRON_IRQ)) == 0 && previousKey == 0)
		{
			input_report_key(data->input_dev, KEY_POWER, 1);
			input_sync(data->input_dev);
			input_report_key(data->input_dev, KEY_POWER, 0);
			input_sync(data->input_dev);
			printk("%s power key is not press", __func__);
		}
		else
		{	
			input_report_key(data->input_dev, KEY_POWER, !(value & PWR_PWRON_IRQ));
			input_sync(data->input_dev);
		}
		previousKey = !(value & PWR_PWRON_IRQ);
	} else {
		dev_err(data->input_dev->dev.parent, "twl4030: i2c error %d while reading"
			" TWL4030 PM_MASTER STS_HW_CONDITIONS register\n", err);
	}	
}


static irqreturn_t powerbutton_irq(int irq, void *dev_id)
{
	struct twl4030_pwrbutton_data *data = dev_id;

	queue_work(data->irq_wq,&data->irq_work);

	return IRQ_HANDLED; 			
}

static int __devinit twl4030_pwrbutton_probe(struct platform_device *pdev)
{
	struct twl4030_pwrbutton_data *data;
	int err;

	data = kzalloc(sizeof(struct twl4030_pwrbutton_data), GFP_KERNEL);
	if (!data) {
		return -ENOMEM;
	}

	memset(data, 0x00, sizeof(struct twl4030_pwrbutton_data));


	data->irq = platform_get_irq(pdev, 0);
	
	data->input_dev = input_allocate_device();
	if (!data->input_dev) {
		dev_dbg(&pdev->dev, "Can't allocate power button\n");
		return -ENOMEM;
	}

	data->input_dev->evbit[0] = BIT_MASK(EV_KEY);
	data->input_dev->keybit[BIT_WORD(KEY_POWER)] = BIT_MASK(KEY_POWER);
	data->input_dev->name = "twl4030_pwrbutton";
	data->input_dev->phys = "twl4030_pwrbutton/input0";
	data->input_dev->dev.parent = &pdev->dev;


	INIT_WORK(&data->irq_work, powerbutton_work_func);

	data->irq_wq = create_singlethread_workqueue("pwrbutton_wq");

	if (!data->irq_wq)
	{
		goto free_irq;
	}

	err = request_irq(data->irq, powerbutton_irq, IRQF_TRIGGER_FALLING | IRQF_TRIGGER_RISING, "twl4030_pwrbutton", data);
	irq_set_affinity(data->irq, cpumask_of(0));

	err = input_register_device(data->input_dev);
	if (err) {
		dev_dbg(&pdev->dev, "Can't register power button: %d\n", err);
		goto free_irq;
	}

	platform_set_drvdata(pdev, data);

	return 0;

free_irq:
	free_irq(data->irq, NULL);
free_input_dev:
	input_free_device(data->input_dev);
	return err;
}

static int __devexit twl4030_pwrbutton_remove(struct platform_device *pdev)
{
	struct twl4030_pwrbutton_data *data = platform_get_drvdata(pdev);

	free_irq(data->irq, data);

	if (data->irq_wq)
		destroy_workqueue(data->irq_wq);

	
	input_unregister_device(data->input_dev);

	return 0;
}

struct platform_driver twl4030_pwrbutton_driver = {
	.probe		= twl4030_pwrbutton_probe,
	.remove		= __devexit_p(twl4030_pwrbutton_remove),
	.driver		= {
		.name	= "twl4030_pwrbutton",
		.owner	= THIS_MODULE,
	},
};

static int __init twl4030_pwrbutton_init(void)
{
	return platform_driver_register(&twl4030_pwrbutton_driver);
}
module_init(twl4030_pwrbutton_init);

static void __exit twl4030_pwrbutton_exit(void)
{
	platform_driver_unregister(&twl4030_pwrbutton_driver);
}
module_exit(twl4030_pwrbutton_exit);

MODULE_ALIAS("platform:twl4030_pwrbutton");
MODULE_DESCRIPTION("Triton2 Power Button");
MODULE_LICENSE("GPL");
MODULE_AUTHOR("Peter De Schrijver <peter.de-schrijver@nokia.com>");
MODULE_AUTHOR("Felipe Balbi <felipe.balbi@nokia.com>");

