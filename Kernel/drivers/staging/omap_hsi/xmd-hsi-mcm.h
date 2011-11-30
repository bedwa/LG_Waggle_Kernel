/*
 * arch/arm/mach-omap2/xmd-hsi-mcm.h
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

#ifndef __HSI_MEM_MGR_H__
#define  __HSI_MEM_MGR_H__

#include <linux/errno.h>
#include <linux/mutex.h>
#include <linux/wait.h>
#include <linux/spinlock.h>
#include <linux/workqueue.h>
#include "xmd-ch.h"

#define MAX_HSI_CHANNELS 16
#define NUM_X_BUF 20

#define HSI_TRUE 1
#define HSI_FALSE 0

typedef enum _HSI_CH_STATE_
{
	HSI_CH_FREE,
	HSI_CH_BUSY,
	HSI_CH_NOT_USED,
} HSI_CH_STATE;

struct hsi_chn {
	char name[32];
	HSI_CH_STATE state;
};

typedef enum
{
	XMD_RX_Q,
	XMD_TX_Q,
} XQ_TYPE;

struct x_data {
	unsigned char *buf;
	unsigned int size;
	unsigned char being_used;
};

struct xq {
	struct x_data data[NUM_X_BUF];
	unsigned char head;
	unsigned char tail;
};
  

struct hsi_channel {

	struct xq rx_q;
	struct x_data *curr;
  
	struct xq tx_q;
  
	HSI_CH_STATE state;
  
	struct xmd_ch_info *info;
  
	void (*notify)(int chno); //memory will be freed after completion of this function.

	void* (*read)(int chno, int* len);
	int (*write)(int chno, void *data, int len);

	char *name;
  
	struct work_struct read_work;
	struct work_struct write_work;

	wait_queue_head_t write_wait;
	int write_happening; 
	int write_queued;
  
	wait_queue_head_t read_wait;
	int read_happening;
	int read_queued;
  
	spinlock_t lock;
};

#endif
