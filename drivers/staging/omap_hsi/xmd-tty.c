/*
 * arch/arm/mach-omap2/xmd-tty.c
 *
 * Copyright (C) 2011 Intel Mobile Communications. All rights reserved.
 *
 * Author: Chaitanya <Chaitanya.Khened@intel.com>
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.  
 *
 */

#include <linux/module.h>
#include <linux/fs.h>
#include <linux/cdev.h>
#include <linux/device.h>
#include <linux/wait.h>
#include <linux/wakelock.h>
#include <linux/errno.h>
#include <linux/mutex.h>
#include <linux/spinlock.h>

#include <linux/tty.h>
#include <linux/tty_driver.h>
#include <linux/tty_flip.h>
#include <linux/slab.h>

#include "xmd-ch.h"

//#define XMD_TTY_ENABLE_DEBUG_MSG
//#define XMD_TTY_ENABLE_ERR_MSG

static DEFINE_MUTEX(xmd_tty_lock);
static struct xmd_ch_info tty_channels[MAX_SMD_TTYS] = {
                                            {0,  "CHANNEL1",  0, XMD_TTY, NULL, 0, SPIN_LOCK_UNLOCKED},
                                            {1,  "CHANNEL2",  0, XMD_TTY, NULL, 0, SPIN_LOCK_UNLOCKED},
                                            {2,  "CHANNEL3",  0, XMD_TTY, NULL, 0, SPIN_LOCK_UNLOCKED},
                                            {3,  "CHANNEL4",  0, XMD_TTY, NULL, 0, SPIN_LOCK_UNLOCKED},
                                            {4,  "CHANNEL5",  0, XMD_TTY, NULL, 0, SPIN_LOCK_UNLOCKED},
                                            {5,  "CHANNEL6",  0, XMD_TTY, NULL, 0, SPIN_LOCK_UNLOCKED},
                                            {6,  "CHANNEL7",  0, XMD_TTY, NULL, 0, SPIN_LOCK_UNLOCKED},
                                            {7,  "CHANNEL8",  0, XMD_TTY, NULL, 0, SPIN_LOCK_UNLOCKED},
                                            {8,  "CHANNEL9",  0, XMD_TTY, NULL, 0, SPIN_LOCK_UNLOCKED},
                                            {9,  "CHANNEL10", 0, XMD_TTY, NULL, 0, SPIN_LOCK_UNLOCKED},
                                            {10, "CHANNEL11", 0, XMD_TTY, NULL, 0, SPIN_LOCK_UNLOCKED},
                                            {11, "CHANNEL12", 0, XMD_TTY, NULL, 0, SPIN_LOCK_UNLOCKED},
                                           };

static int tty_channels_len = ARRAY_SIZE(tty_channels);

static void xmd_ch_tty_send_to_user(int chno)
{
	struct tty_struct *tty = NULL;
	unsigned char *buf = NULL;
	unsigned char *tbuf = NULL;
	int i,len;
  
	buf = (unsigned char *)xmd_ch_read(chno, &len);
#if defined (XMD_TTY_ENABLE_DEBUG_MSG)
	{
		char *str = (char *) kzalloc(len + 1, GFP_ATOMIC);
		memcpy(str, buf, len);
		printk("\nxmdtty: Sending data of size %d to ch %d, buf = %s\n",len,chno, str);
		kfree(str);
	}
#endif  
	for (i=0; i<tty_channels_len; i++) {
		if (tty_channels[i].chno == chno)
			tty = (struct tty_struct *)tty_channels[i].priv;
	}

	if (!tty) {
#if defined (XMD_TTY_ENABLE_ERR_MSG)
		printk("\nxmdtty: invalid chno %d \n", chno);
#endif
		return;
	}

	tty->low_latency = 1;
  
	tty_prepare_flip_string(tty, &tbuf, len); 

	if (!tbuf) {
#if defined (XMD_TTY_ENABLE_ERR_MSG)
		printk("\nxmdtty: memory not allocated by tty core to send to user space\n");
#endif
		return;
	}
	memcpy((void *)tbuf, (void *)buf, len);
    
	tty_flip_buffer_push(tty); 
	tty_wakeup(tty);
}

static int xmd_ch_tty_open(struct tty_struct *tty, struct file *f)
{
	struct xmd_ch_info *tty_ch;
	char init_flag = 0;

	int n = tty->index;
	if (n >= tty_channels_len) {
#if defined (XMD_TTY_ENABLE_DEBUG_MSG)
		printk("\nxmdtty: Error opening channel %d\n",n);
#endif
		return -ENODEV; 
	}
  
#if defined (XMD_TTY_ENABLE_DEBUG_MSG)
	printk("\nxmdtty:Opening channel %d\n",n+1);
#endif
  
	tty_ch = tty_channels + n;  
  
	mutex_lock(&xmd_tty_lock);
	if (tty_ch->open_count > 0)
		init_flag = 1;

	tty_ch->open_count++;
      
	if(init_flag) {
		mutex_unlock(&xmd_tty_lock);
#if defined (XMD_TTY_ENABLE_DEBUG_MSG)
		printk("\nxmdtty: Channel already opened successfully %d\n",tty_ch->chno);
#endif
		return 0;
	}
  
	tty_ch->chno = xmd_ch_open(tty_ch, xmd_ch_tty_send_to_user);
	if (0 > tty_ch->chno) {
#if defined (XMD_TTY_ENABLE_DEBUG_MSG)
		printk("\nError opening channel %d\n",n);
#endif
		mutex_unlock(&xmd_tty_lock);
		tty_ch->open_count = 0;
		tty_ch->chno = 0;
		return -ENOMEM;
	}

#if defined (XMD_TTY_ENABLE_DEBUG_MSG)
	printk("\nxmdtty: Channel opened successfully %d\n",tty_ch->chno);
#endif  
	tty->driver_data = (void *)tty_ch;
	tty_ch->priv = (void*) tty;
	mutex_unlock(&xmd_tty_lock);
  
	return 0;
}

static void xmd_ch_tty_close(struct tty_struct *tty, struct file *f)
{
	struct xmd_ch_info *tty_ch = (struct xmd_ch_info*)tty->driver_data;
	char cleanup_flag = 1;

#if defined (XMD_TTY_ENABLE_DEBUG_MSG)
	printk("\nxmdtty: Channel close function [ch %d]\n",tty_ch->chno);
#endif
	if (!tty_ch) {
#if defined (XMD_TTY_ENABLE_DEBUG_MSG)
		printk("\nxmdtty: Channel close function\n");
#endif
		return;
	}
  
	mutex_lock(&xmd_tty_lock);
	if (tty_ch->open_count > 1)
		cleanup_flag = 0;
  
	tty_ch->open_count--;
	if (cleanup_flag) {
		xmd_ch_close(tty_ch->chno);
		tty->driver_data = NULL;
	}
	mutex_unlock(&xmd_tty_lock);
}

static int xmd_ch_tty_write(struct tty_struct *tty, const unsigned char *buf, int len)
{
	struct xmd_ch_info *tty_ch = tty->driver_data;

#if defined (XMD_TTY_ENABLE_DEBUG_MSG)
	char *str = (char *) kzalloc(len + 1, GFP_ATOMIC);
	memcpy(str, buf, len);
	printk("\nxmdtty: writing data of size %d to ch %d, data: %s\n",len,tty_ch->chno,str);
	kfree(str);
#endif
  
	xmd_ch_write(tty_ch->chno, (void *)buf, len);

	return len;
}

static int xmd_ch_tty_write_room(struct tty_struct *tty)
{
	return 8192; 
}


static int xmd_ch_tty_chars_in_buffer(struct tty_struct *tty)
{
	return 0;
}

static void xmd_ch_tty_unthrottle(struct tty_struct *tty)
{
	return;
}


static struct tty_operations xmd_ch_tty_ops = {
	.open = xmd_ch_tty_open,
	.close = xmd_ch_tty_close,
	.write = xmd_ch_tty_write,
	.write_room = xmd_ch_tty_write_room,
	.chars_in_buffer = xmd_ch_tty_chars_in_buffer,
	.unthrottle = xmd_ch_tty_unthrottle,
};

static struct tty_driver *xmd_ch_tty_driver;

static int __init xmd_ch_tty_init(void)
{
	int ret, i;
    
#if defined (XMD_TTY_ENABLE_DEBUG_MSG)
	printk("\nxmdtty: xmd_ch_tty_init\n");
#endif
	xmd_ch_tty_driver = alloc_tty_driver(MAX_SMD_TTYS);
	if (xmd_ch_tty_driver == 0)
		return -ENOMEM;

	xmd_ch_tty_driver->owner = THIS_MODULE;
	xmd_ch_tty_driver->driver_name = "xmd_ch_tty_driver";
	xmd_ch_tty_driver->name = "xmd-tty"; //"ttyspi";//"xmd-tty";
	xmd_ch_tty_driver->major = 0;
	xmd_ch_tty_driver->minor_start = 0;
	xmd_ch_tty_driver->type = TTY_DRIVER_TYPE_SERIAL;
	xmd_ch_tty_driver->subtype = SERIAL_TYPE_NORMAL;
	xmd_ch_tty_driver->init_termios = tty_std_termios;
	xmd_ch_tty_driver->init_termios.c_iflag = 0;
	xmd_ch_tty_driver->init_termios.c_oflag = 0;
	xmd_ch_tty_driver->init_termios.c_cflag = B38400 | CS8 | CREAD;
	xmd_ch_tty_driver->init_termios.c_lflag = 0;
	xmd_ch_tty_driver->flags = TTY_DRIVER_RESET_TERMIOS |
		TTY_DRIVER_REAL_RAW | TTY_DRIVER_DYNAMIC_DEV;
	tty_set_operations(xmd_ch_tty_driver, &xmd_ch_tty_ops);

	ret = tty_register_driver(xmd_ch_tty_driver);
	if (ret) return ret;

	for (i = 0; i < tty_channels_len; i++)
		tty_register_device(xmd_ch_tty_driver, tty_channels[i].id, 0);
		
	xmd_ch_init();

	return 0;
}

module_init(xmd_ch_tty_init);

