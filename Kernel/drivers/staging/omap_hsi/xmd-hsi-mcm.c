/*
 * arch/arm/mach-omap2/xmd-hsi-mcm.c
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
#include <linux/sched.h>
#include <linux/wait.h>
#include <linux/errno.h>
#include <linux/mutex.h>
#include <linux/semaphore.h>
#include <linux/spinlock.h>
#include <linux/workqueue.h>
#include "xmd-ch.h"
#include "xmd-hsi-mcm.h"
#include "xmd-hsi-ll-if.h"
#include "xmd_hsi_mem.h"
#include <linux/delay.h>
#include <linux/slab.h>

#define MCM_DBG_LOG 0
#define MCM_DBG_ERR_LOG 1

spinlock_t mem_lock;

static struct hsi_chn hsi_all_channels[MAX_HSI_CHANNELS] = {
	{"CONTROL",  HSI_CH_NOT_USED},
	{"CHANNEL1", HSI_CH_FREE},
	{"CHANNEL2", HSI_CH_FREE},
	{"CHANNEL3", HSI_CH_FREE},
	{"CHANNEL4", HSI_CH_FREE},
	{"CHANNEL5", HSI_CH_FREE},
	{"CHANNEL6", HSI_CH_FREE},
	{"CHANNEL7", HSI_CH_FREE},
	{"CHANNEL8", HSI_CH_FREE},
	{"CHANNEL9", HSI_CH_FREE},
	{"CHANNEL10",HSI_CH_FREE},
	{"CHANNEL11",HSI_CH_FREE},
	{"CHANNEL12",HSI_CH_FREE},
	{"CHANNEL13",HSI_CH_FREE},
	{"CHANNEL14",HSI_CH_FREE},
	{"CHANNEL15",HSI_CH_FREE},
};

static struct hsi_channel hsi_channels[MAX_HSI_CHANNELS];

static struct workqueue_struct *hsi_read_wq;
static struct workqueue_struct *hsi_write_wq;

void (*xmd_boot_cb)(void);
void hsi_read_work(struct work_struct *work);
void hsi_write_work(struct work_struct *work);

void init_q(int chno)
{
	int i;
  
	hsi_channels[chno].rx_q.head = 0;
	hsi_channels[chno].rx_q.tail = 0;
	hsi_channels[chno].tx_q.head = 0;
	hsi_channels[chno].tx_q.tail = 0;
  
	for (i=0; i<NUM_X_BUF; i++)
		hsi_channels[chno].rx_q.data[i].being_used = HSI_FALSE;
  
}


struct x_data* read_q(int chno, struct xq* q)  //Head grows on reading from q. "data=q[head];head++;"
{
	struct x_data *data = NULL;
  
	if (!q) {
#if MCM_DBG_ERR_LOG
		printk("mcm: NULL q instance");
#endif
		return NULL;
	}
    
#if MCM_DBG_LOG
	printk("\nmcm: [read_q]  head = %d, tail = %d\n",q->head,q->tail);
#endif
  
	spin_lock_bh(&hsi_channels[chno].lock);
  
	if (q->head == q->tail) {
		spin_unlock_bh(&hsi_channels[chno].lock);
#if MCM_DBG_LOG
		printk("\nmcm: q empty [read] \n");
#endif
		return NULL;
	}
  
	data = q->data + q->head;
  
	q->head = (q->head + 1) % NUM_X_BUF;
  
	spin_unlock_bh(&hsi_channels[chno].lock);
  
	return data;
  
}


int write_q(struct xq* q, char *buf, int size, struct x_data **data)   //Tail grows on writing in q. "q[tail]=data;tail++;"
{
	int temp = 0;
  
	if (!q) {
#if MCM_DBG_ERR_LOG
		printk("mcm: NULL q instance");
#endif
		return 0;
	}
    
	temp = (q->tail + 1) % NUM_X_BUF;
  
	if (temp != q->head) {
  
		q->data[q->tail].buf = buf;
		q->data[q->tail].size = size;
		if (data)
			*data = q->data + q->tail;

		q->tail = temp;
	} else {
#if MCM_DBG_ERR_LOG
		printk("\nmcm: q full [write], head = %d, tail = %d\n",q->head,q->tail);
#endif
		return 0;
	}
  
	return q->tail > q->head ? q->tail - q->head:q->tail - q->head + NUM_X_BUF;
}


void *hsi_mem_alloc(int size)
{
	void *buf = NULL;
	spin_lock_bh(&mem_lock);
#ifdef IFX_HSI_BUFFERS
	buf =  mipi_hsi_mem_alloc(size);
#else
	buf =  kmalloc(size, GFP_DMA | GFP_ATOMIC | GFP_KERNEL);
#endif
	spin_unlock_bh(&mem_lock);
	return buf;
}

void hsi_mem_free(void *buf)
{
	spin_lock_bh(&mem_lock);
#ifdef IFX_HSI_BUFFERS
	mipi_hsi_mem_free(buf);
#else
	kfree(buf);
#endif
	spin_unlock_bh(&mem_lock);
}


static int hsi_ch_net_write(int chno, void *data, int len)
{
	/* Non blocking write */
	void *buf = NULL;
	static struct x_data *d = NULL;
	int n = 0;
	int flag = 1;

#ifdef XMD_TX_MULTI_PACKET
	if (d && hsi_channels[chno].write_queued == HSI_TRUE) {
		if (d->being_used == HSI_FALSE && (d->size + len) < HSI_LARGE_BLOCK_SIZE) {
#if MCM_DBG_LOG
			printk("\nmcm: adding in the queued buffer for ch %d\n",chno);
#endif
			buf = d->buf + d->size;
			d->size += len;
			flag = 0;
		} else
			flag = 1;
	}
#endif
	if (flag) {
#ifdef XMD_TX_MULTI_PACKET
		buf = hsi_mem_alloc(HSI_LARGE_BLOCK_SIZE);
#else
		buf = hsi_mem_alloc(len);
#endif
		flag = 1;
	}
 
	if (!buf || !data)
		return -ENOMEM;

	memcpy(buf, data, len);
  
	if (flag) {
		d = NULL;
		n = write_q(&hsi_channels[chno].tx_q, buf, len, &d);
#if MCM_DBG_LOG
		printk("\nmcm: n = %d\n",n);
#endif
		if (n == 0) {
#if MCM_DBG_ERR_LOG
			printk("\nmcm: Dropping the packet as channel %d is busy writing already queued data\n",chno);
#endif
			hsi_mem_free(buf);
			PREPARE_WORK(&hsi_channels[chno].write_work, hsi_write_work);
			queue_work(hsi_write_wq, &hsi_channels[chno].write_work);
		} else if (n == 1) {
			PREPARE_WORK(&hsi_channels[chno].write_work, hsi_write_work);
			queue_work(hsi_write_wq, &hsi_channels[chno].write_work);
		}
	}
  
	return 0;
}


static int hsi_ch_tty_write(int chno, void *data, int len)
{
	void *buf = NULL;
	int err;
  
	buf = hsi_mem_alloc(len);
  
	if (!buf)
		return -ENOMEM;
    
	//printk("\n data = %c, len = %d\n",((char*)data)[0], len);
    
	memcpy(buf, data, len);

	hsi_channels[chno].write_happening = HSI_TRUE;
  
	err = hsi_ll_write(chno, (unsigned char *)buf, len);
	if (err < 0) {
#if MCM_DBG_ERR_LOG
		printk("\nmcm: hsi_ll_write failed\n");
#endif
		hsi_channels[chno].write_happening = HSI_FALSE;
	}
  
	if (hsi_channels[chno].write_happening == HSI_TRUE) { //spinlock may be used for write_happening
#if MCM_DBG_LOG
		printk("\nmcm:locking mutex for ch: %d\n",chno);
#endif
		wait_event(hsi_channels[chno].write_wait, hsi_channels[chno].write_happening == HSI_FALSE);
	}
  
	return err;
}


static void* hsi_ch_net_read(int chno, int* len)
{
	struct x_data *data = hsi_channels[chno].curr;
	*len = data->size;
	return data->buf;  
}


static void* hsi_ch_tty_read(int chno, int* len)
{
	struct x_data *data = hsi_channels[chno].curr;
	*len = data->size;
	//printk("\nmcm: reading the data %s to user space\n",data->buf);
	return data->buf;
}


int xmd_ch_write(int chno, void *data, int len)
{
	int err;

#if MCM_DBG_LOG
	printk("\nmcm: write entering, ch %d\n",chno);
#endif

	if (!hsi_channels[chno].write) {
#if MCM_DBG_ERR_LOG
		printk("\nmcm:write func NULL for ch: %d\n",chno);
#endif
		return -1;
	}

	err = hsi_channels[chno].write(chno, data, len);
  
#if MCM_DBG_LOG
	printk("\nmcm: write returning, ch %d\n",chno);
#endif
	return err;
}


void* xmd_ch_read(int chno, int* len)
{
	return hsi_channels[chno].read(chno,len);
}


void xmd_ch_close(int chno)
{
	if (hsi_channels[chno].read_happening == HSI_TRUE) { //spinlock may be used for read_happening
#if MCM_DBG_LOG
		printk("\nmcm:locking read mutex for ch: %d\n",chno);
#endif
		wait_event(hsi_channels[chno].read_wait, hsi_channels[chno].read_happening == HSI_FALSE);
	}
	hsi_ll_close(chno);
	spin_lock_bh(&hsi_channels[chno].lock);
	hsi_channels[chno].state = HSI_CH_FREE;
	spin_unlock_bh(&hsi_channels[chno].lock);
}


int xmd_ch_open(struct xmd_ch_info* info, void (*notify_cb)(int chno))
{
	int i;
	int size = ARRAY_SIZE(hsi_channels);
  
	for (i=0; i<size; i++) {
		if (hsi_channels[i].name)
			if (!strcmp(info->name, hsi_channels[i].name)) {
				if (hsi_channels[i].state == HSI_CH_BUSY || hsi_channels[i].state == HSI_CH_NOT_USED) {
#if MCM_DBG_ERR_LOG
					printk("\nmcm:Channel state not suitable %d\n",i);
#endif          
					return -EINVAL;
				}
        
				if (0 != hsi_ll_open(i)) {
#if MCM_DBG_ERR_LOG
					printk("\nmcm:hsi_ll_open failed for channel %d\n",i);
#endif          
					return -EINVAL;
				}
        
				hsi_channels[i].info = info;

				spin_lock_bh(&hsi_channels[i].lock);
				hsi_channels[i].state = HSI_CH_BUSY;
				spin_unlock_bh(&hsi_channels[i].lock);

				hsi_channels[i].notify = notify_cb;
				//printk("\nmcm:user is %d \n",(int)info->user);
				switch(info->user)
				{
				case XMD_TTY:
					hsi_channels[i].read = hsi_ch_tty_read;
					hsi_channels[i].write = hsi_ch_tty_write;
				break;
				case XMD_NET:
					hsi_channels[i].read = hsi_ch_net_read;
					hsi_channels[i].write = hsi_ch_net_write;
				break;
				default:
#if MCM_DBG_ERR_LOG
					printk("\nmcm:Neither TTY nor NET \n");
#endif
					return -EINVAL;
				}
				INIT_WORK(&hsi_channels[i].read_work, hsi_read_work);
				INIT_WORK(&hsi_channels[i].write_work, hsi_write_work);
				return i;
			}
	}
#if MCM_DBG_ERR_LOG
	printk("\n Channel name not proper \n"); 
#endif
	return -EINVAL;
}


void hsi_read_work(struct work_struct *work)
{
	//function registered with read work q
	struct hsi_channel *ch = (struct hsi_channel*) container_of(work, struct hsi_channel,read_work);
	int chno = ch->info->chno;
	struct x_data *data = NULL;
  
	if (hsi_channels[chno].read_queued == HSI_TRUE) {
#if MCM_DBG_ERR_LOG
		printk("\nmcm: read wq already in progress\n");
#endif
		return;
	}
  
	hsi_channels[chno].read_queued = HSI_TRUE;
  
	while ((data = read_q(chno, &hsi_channels[chno].rx_q)) != NULL) {
		char *buf = data->buf;
		hsi_channels[chno].curr = data;
		hsi_channels[chno].notify(chno);
		hsi_mem_free(buf);
	}
	hsi_channels[chno].read_queued = HSI_FALSE;
	spin_lock_bh(&hsi_channels[chno].lock);
	hsi_channels[chno].read_happening = HSI_FALSE; 
	spin_unlock_bh(&hsi_channels[chno].lock);
	wake_up(&hsi_channels[chno].read_wait);
}


void hsi_write_work(struct work_struct *work)
{
	//function registered with write work q
	struct hsi_channel *ch = (struct hsi_channel*) container_of(work, struct hsi_channel,write_work);
	int chno = ch->info->chno;
	struct x_data *data = NULL;
	int err;

	if (hsi_channels[chno].write_queued == HSI_TRUE) {
#if MCM_DBG_ERR_LOG
		printk("\nmcm: write wq already in progress\n");
#endif
		return;
	}
  
	hsi_channels[chno].write_queued = HSI_TRUE;
  
	while ((data = read_q(chno, &hsi_channels[chno].tx_q)) != NULL) {
		hsi_channels[chno].write_happening = HSI_TRUE; //spinlock may be used for read_happening
		data->being_used = HSI_TRUE;
		err = hsi_ll_write(chno, (unsigned char *)data->buf, data->size);
		if (err < 0) {
#if MCM_DBG_ERR_LOG
			printk("\nmcm: hsi_ll_write failed\n");
#endif
			hsi_channels[chno].write_happening = HSI_FALSE;
		}
    
		if (hsi_channels[chno].write_happening == HSI_TRUE) { //spinlock may be used for write_happening
#if MCM_DBG_LOG
			printk("\nmcm:locking mutex for ch: %d\n",chno);
#endif
			wait_event(hsi_channels[chno].write_wait, hsi_channels[chno].write_happening == HSI_FALSE);
		}
		data->being_used = HSI_FALSE;
	}
	hsi_channels[chno].write_queued = HSI_FALSE;
}


void hsi_ch_cb(unsigned int chno, int result, int event, void* arg) 
{
	ll_rx_tx_data *data = (ll_rx_tx_data *) arg;

	if (!(chno <= MAX_HSI_CHANNELS && chno >= 0) || hsi_channels[chno].state == HSI_CH_NOT_USED) {
#if MCM_DBG_ERR_LOG
		printk("\nmcm: Wrong channel number or channel not used\n");
#endif
		return;
	}


	switch(event)
	{
	case HSI_LL_EV_ALLOC_MEM: // if event is allocate read mem, 
	{
#if MCM_DBG_LOG
		printk("\nmcm: Allocating read memory of size %d to channel %d \n", data->size, chno);
#endif
		/* MODEM can't handle NAK so we allocate memory and drop the packet after recieving from MODEM */
#if 0
		spin_lock_bh(&hsi_channels[chno].lock);
		if (hsi_channels[chno].state == HSI_CH_FREE) {
			spin_unlock_bh(&hsi_channels[chno].lock);
#if MCM_DBG_ERR_LOG
			printk("\nmcm: channel not yet opened so not allocating memory\n");
#endif
			data->buffer = NULL;
			break;
		}
		spin_unlock_bh(&hsi_channels[chno].lock);
#endif
		data->buffer = (char *)hsi_mem_alloc(data->size);
	}
	break;
  
	case HSI_LL_EV_FREE_MEM: // if event is free read mem,
	{
#if MCM_DBG_LOG
		printk("\nmcm: Freeing memory for channel %d, ptr = 0x%p \n",chno,data->buffer);
#endif
		spin_lock_bh(&hsi_channels[chno].lock);
		if (hsi_channels[chno].state == HSI_CH_FREE) {
			spin_unlock_bh(&hsi_channels[chno].lock);
#if MCM_DBG_ERR_LOG
			printk("\nmcm: channel not yet opened so cant free mem\n");
#endif
			break;
		}
		spin_unlock_bh(&hsi_channels[chno].lock);
		hsi_mem_free(data->buffer);
	}
	break;
  
	case HSI_LL_EV_RESET_MEM:
	// if event is break, handle it somehow.
	break;
  
	// if event is modem powered on, wake up the event.
	//xmd_boot_cb(); TBD from DLP

	case HSI_LL_EV_WRITE_COMPLETE:
	{
#if MCM_DBG_LOG
		printk("\nmcm:unlocking mutex for ch: %d\n",chno);
#endif
		hsi_channels[chno].write_happening = HSI_FALSE; //spinlock protection for write_happening... TBD
		wake_up(&hsi_channels[chno].write_wait);
		hsi_mem_free(data->buffer);
#if MCM_DBG_LOG
		printk("\nmcm: write complete cb, ch %d\n",chno);
#endif
	}
	break;
  
	case HSI_LL_EV_READ_COMPLETE: // if event is send data, schedule work q to send data to upper layers
	{
		int n = 0;
#if MCM_DBG_LOG
		printk("\nmcm: Read complete... size %d, channel %d, ptr = 0x%p \n", data->size, chno,data->buffer);
#endif
		spin_lock_bh(&hsi_channels[chno].lock);
		if (hsi_channels[chno].state == HSI_CH_FREE) {
			spin_unlock_bh(&hsi_channels[chno].lock);
#if MCM_DBG_ERR_LOG
			printk("\nmcm: channel %d not yet opened so dropping the packet\n",chno);
#endif
			hsi_mem_free(data->buffer);
			break;
		}
    
		n = write_q(&hsi_channels[chno].rx_q, data->buffer, data->size, NULL);

		spin_unlock_bh(&hsi_channels[chno].lock);

		if (n == 0) {
#if MCM_DBG_ERR_LOG
			printk("\nmcm: Dropping the packet as channel %d is busy sending already read data\n",chno);
#endif
			hsi_mem_free(data->buffer);
			PREPARE_WORK(&hsi_channels[chno].read_work, hsi_read_work);
			queue_work(hsi_read_wq, &hsi_channels[chno].read_work);
		} else if (n == 1) {
			if (hsi_channels[chno].read_happening == HSI_FALSE)
			{
				hsi_channels[chno].read_happening = HSI_TRUE; //spinlock protection for read_happening... TBD
			}
			PREPARE_WORK(&hsi_channels[chno].read_work, hsi_read_work);
			queue_work(hsi_read_wq, &hsi_channels[chno].read_work);
      
		}
		// if n > 1, no need to schdule the wq again.
	}
	break;
	default:
		//Wrong event.
	break;
	}
}


void xmd_ch_register_xmd_boot_cb(void (*fn)(void))
{
	xmd_boot_cb = fn;
}


void __init xmd_ch_init(void)
{
	int i;
	int size = ARRAY_SIZE(hsi_all_channels);

#if MCM_DBG_LOG
	printk("\nmcm: xmd_ch_init++\n");
#endif  
	if (0 != mipi_hsi_mem_init())
		panic("\nmcm: memory initialization failed\n");
    
	spin_lock_init(&mem_lock);
  
	for (i=0; i<size; i++) {
		hsi_channels[i].state = hsi_all_channels[i].state;
		hsi_channels[i].name = hsi_all_channels[i].name;
		hsi_channels[i].write_happening = HSI_FALSE;
		hsi_channels[i].write_queued = HSI_FALSE;
		hsi_channels[i].read_queued = HSI_FALSE;
		hsi_channels[i].read_happening = HSI_FALSE;
		spin_lock_init(&hsi_channels[i].lock);
		init_waitqueue_head(&hsi_channels[i].write_wait);
		init_waitqueue_head(&hsi_channels[i].read_wait);
		init_q(i);
	}
  
	hsi_ll_init(1, hsi_ch_cb);
  
	//Create and initialize work q
	hsi_read_wq = create_workqueue("hsi-read-wq");
	hsi_write_wq = create_workqueue("hsi-write-wq");
}


void xmd_ch_exit(void)
{
	flush_workqueue(hsi_read_wq);
	destroy_workqueue(hsi_read_wq);
	flush_workqueue(hsi_write_wq);
	destroy_workqueue(hsi_write_wq);
	hsi_ll_shutdown();
	mipi_hsi_mem_uninit();
}

                                                        
