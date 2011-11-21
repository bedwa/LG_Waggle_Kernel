/* 
 * arch/arm/mach-omap2/xmd_hsi_mem.c
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

                                                                               
/*****************************************************************************/ 
/* INCLUDES                                                                  */ 
/*****************************************************************************/ 


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
#include <linux/dma-mapping.h>
#include <linux/slab.h>
#include "xmd_hsi_mem.h"

#ifdef IFX_HSI_BUFFERS
#ifndef HSI_DMA_BUFFERS
unsigned char small_block[HSI_NUM_SMALL_BLOCKS][HSI_SMALL_BLOCK_SIZE];
unsigned char medium_block[HSI_NUM_MEDIUM_BLOCKS][HSI_MEDIUM_BLOCK_SIZE];
unsigned char large_block[HSI_NUM_LARGE_BLOCKS][HSI_LARGE_BLOCK_SIZE];
#else
unsigned char *small_block[HSI_NUM_SMALL_BLOCKS];
unsigned char *medium_block[HSI_NUM_MEDIUM_BLOCKS];
unsigned char *large_block[HSI_NUM_LARGE_BLOCKS];
#endif

unsigned char *fallback_block[HSI_NUM_FALLBACK_BLOCKS] = {NULL};

/* internal memory management */
#if HSI_NUM_SMALL_BLOCKS > 32
#error HSI_NUM_SMALL_BLOCKS is too large i.e. larger than 32
#endif
#if HSI_NUM_MEDIUM_BLOCKS > 32
#error HSI_NUM_MEDIUM_BLOCKS is too large i.e. larger than 32
#endif
#if HSI_NUM_LARGE_BLOCKS > 32
#error HSI_NUM_LARGE_BLOCKS is too large i.e. larger than 32
#endif

unsigned long small_block_avail;
unsigned long medium_block_avail;
unsigned long large_block_avail;


#if HSI_MEM_DEBUG
unsigned long mem_dbg_stat_size[MEM_STAT_RANGE + 1]; /* total number of allocated blocks of size MEM_STAT_RANGE, MEM_STAT_RANGE * 2, .... */
unsigned long mem_dbg_small_block_cnt;               /* currently allocated blocks from large pool */
unsigned long mem_dbg_small_block_max;               /* maximum number of allocated blocks from large pool */
unsigned long mem_dbg_medium_block_cnt;              /* currently allocated blocks from medium pool */
unsigned long mem_dbg_medium_block_max;              /* maximum number of allocated blocks from medium pool */
unsigned long mem_dbg_large_block_cnt;               /* currently allocated blocks from large pool */
unsigned long mem_dbg_large_block_max;               /* maximum number of allocated blocks from large pool */
unsigned long mem_dbg_max_block_size;               /* largest block size */
unsigned long mem_dbg_min_block_size;               /* smallest block size */
#endif /* HSI_MEM_DEBUG */

//#define HSI_MEM_CLEAR
#endif


/*****************************************************************************/ 
/* Function:... mipi_hsi_mem_mem_init                                        */
/* Description: Initialization of the memory pools.                          */
/*****************************************************************************/ 

int mipi_hsi_mem_init(void)
{
	unsigned long i;
#ifdef IFX_HSI_BUFFERS
	small_block_avail =  HSI_NUM_SMALL_BLOCKS;
	medium_block_avail = HSI_NUM_MEDIUM_BLOCKS;
	large_block_avail =  HSI_NUM_LARGE_BLOCKS;

#if HSI_MEM_DEBUG
	for (i = 0; i < MEM_STAT_RANGE + 1; i++)
		mem_dbg_stat_size[i] = 0;
	mem_dbg_small_block_cnt = 0;
	mem_dbg_small_block_max = 0;
	mem_dbg_large_block_cnt = 0;
	mem_dbg_large_block_max = 0;
	mem_dbg_medium_block_cnt = 0;
	mem_dbg_medium_block_max = 0;
	mem_dbg_max_block_size = 0;
	mem_dbg_min_block_size = 0xFFFFFFFF;
#endif /* HSI_MEM_DEBUG */

#ifdef HSI_DMA_BUFFERS
	for (i=0; i<HSI_NUM_SMALL_BLOCKS; i++) {
		small_block[i] = (unsigned char *) kmalloc(HSI_SMALL_BLOCK_SIZE, GFP_DMA|GFP_KERNEL);
    
		if (!small_block[i]) {
#ifdef HSI_MEM_DEBUG
			printk("\nHSIMEM: DMA MEM not available");
#endif
			return -ENOMEM;
		}
	}
  
	for (i=0; i<HSI_NUM_MEDIUM_BLOCKS; i++) {
		medium_block[i] = (unsigned char *) kmalloc(HSI_MEDIUM_BLOCK_SIZE, GFP_DMA|GFP_KERNEL);
    
		if (!medium_block[i]) {
#ifdef HSI_MEM_DEBUG
			printk("\nHSIMEM: DMA MEM not available");
#endif      	
			return -ENOMEM;
		}
	}
  
	for (i=0; i<HSI_NUM_LARGE_BLOCKS; i++) {
		large_block[i] = (unsigned char *) kmalloc(HSI_LARGE_BLOCK_SIZE, GFP_DMA|GFP_KERNEL);
    
		if (!large_block[i]) {
#ifdef HSI_MEM_DEBUG
			printk("\nHSIMEM: DMA MEM not available");
#endif      
			return -ENOMEM;
		}
	}
#endif
#endif
	return 0;
} /* mipi_hsi_mem_init */


/*****************************************************************************/ 
/* Function:... mipi_hsi_mem_uninit                                        */
/* Description: Freeing of memory pools                                    */
/*****************************************************************************/ 

int mipi_hsi_mem_uninit(void)
{
	unsigned long i;

#ifdef IFX_HSI_BUFFERS
#ifdef HSI_DMA_BUFFERS
	for (i=0; i<HSI_NUM_SMALL_BLOCKS; i++)
		kfree(small_block[i]);
	for (i=0; i<HSI_NUM_MEDIUM_BLOCKS; i++)
		kfree(medium_block[i]);
	for (i=0; i<HSI_NUM_LARGE_BLOCKS; i++)
		kfree(large_block[i]);
#endif
#endif
	return 0;
} /* mipi_hsi_mem_uninit */


/*****************************************************************************/ 
/* Function:... mipi_hsi_mem_alloc                                           */
/* Description: allocates a block with the requested size. If no memory      */
/* block of the requested size available, NULL will be returned. The         */
/* returned pointer will be aligned to 16 byte borders.                      */
/*****************************************************************************/ 

void *mipi_hsi_mem_alloc(int size)
{
#ifdef IFX_HSI_BUFFERS
	unsigned long i;
	unsigned long mask;
#if HSI_MEM_DEBUG
	unsigned long stat_size;

	for (i = 0, stat_size = MEM_STAT_MIN_SIZE; i < MEM_STAT_RANGE; i++, stat_size *= 2)
		if (size <= stat_size) {
			mem_dbg_stat_size[i]++;
			break;
		}
	if (size > stat_size)
		mem_dbg_stat_size[i]++;
#endif /* HSI_MEM_DEBUG */


	if (size < HSI_SMALL_BLOCK_SIZE) {  /* small blocks */
		for (i = 0, mask = 0x01; i < HSI_NUM_SMALL_BLOCKS; i++, mask <<= 1)
			if (mask & small_block_avail) {
				small_block_avail &= ~mask;
#if HSI_MEM_DEBUG
				mem_dbg_small_block_cnt++;
				if (mem_dbg_small_block_cnt%HSI_NUM_SMALL_BLOCKS > mem_dbg_small_block_max)
					mem_dbg_small_block_max = mem_dbg_small_block_cnt;
#endif /* HSI_MEM_DEBUG */
#ifdef HSI_MEM_CLEAR
				memset((void *)small_block[i],'$', HSI_SMALL_BLOCK_SIZE);
#endif
				return (void *)small_block[i];
			}
	}
	if (size < HSI_MEDIUM_BLOCK_SIZE) {  /* medium blocks */
		for (i = 0, mask = 0x01; i < HSI_NUM_MEDIUM_BLOCKS; i++, mask <<= 1)
			if (mask & medium_block_avail)
			{
				medium_block_avail &= ~mask;
#if HSI_MEM_DEBUG
				mem_dbg_medium_block_cnt++;
				if (mem_dbg_medium_block_cnt%HSI_NUM_MEDIUM_BLOCKS > mem_dbg_medium_block_max)
					mem_dbg_medium_block_max = mem_dbg_medium_block_cnt;
#endif /* HSI_MEM_DEBUG */
#ifdef HSI_MEM_CLEAR
				memset((void *)medium_block[i],'$',HSI_MEDIUM_BLOCK_SIZE);
#endif
				return (void *)medium_block[i];
			}
	}
	if (size < HSI_LARGE_BLOCK_SIZE) {  /* large blocks */
		for (i = 0, mask = 0x01; i < HSI_NUM_LARGE_BLOCKS; i++, mask <<= 1)
			if (mask & large_block_avail) {
				large_block_avail &= ~mask;
#if HSI_MEM_DEBUG
				mem_dbg_large_block_cnt++;
				if (mem_dbg_large_block_cnt%HSI_NUM_LARGE_BLOCKS > mem_dbg_large_block_max)
					mem_dbg_large_block_max = mem_dbg_large_block_cnt;
#endif /* HSI_MEM_DEBUG */
#ifdef HSI_MEM_CLEAR
				memset((void *)large_block[i],'$',HSI_LARGE_BLOCK_SIZE);
#endif
				return (void *)large_block[i];
			}
	}

#if HSI_MEM_DEBUG
	printk("\nHSI_MEM: Going for fallback memory[sz: %d]\n",size);
#endif

  /* fallback to kernel for memory */
	for (i=0; i<HSI_NUM_FALLBACK_BLOCKS; i++) {
		if (fallback_block[i] == NULL) {
			fallback_block[i] = (unsigned char*) kmalloc(size, GFP_DMA|GFP_KERNEL|GFP_ATOMIC);
			return (void *) fallback_block[i];
		}
	}
  
#if HSI_MEM_DEBUG
	printk("\nHSI_MEM: Failed to allocate memory[sz: %d].Returning NULL\n",size);
	printk("\nHSI_MEM: lb, mb, sb cnt = %lu, %lu, %lu\n",mem_dbg_large_block_cnt,mem_dbg_medium_block_cnt,mem_dbg_small_block_cnt);
	printk("\nHSI_MEM: lb, mb, sb max = %lu, %lu, %lu\n",mem_dbg_large_block_max,mem_dbg_medium_block_max,mem_dbg_small_block_max);
#endif

#endif
	return (void *)0;
} /* mipi_hsi_mem_alloc */


/*****************************************************************************/ 
/* Function:... mipi_hsi_mem_free                                            */
/* Description: Frees a memory block, which was allocated before with        */
/* mipi_hsi_mem_alloc.                                                       */
/*****************************************************************************/ 

int mipi_hsi_mem_free(unsigned char* buf)
{
#ifdef IFX_HSI_BUFFERS
	unsigned long i;
	unsigned long mask;
	unsigned char *mem = (unsigned char *) buf;

	for (i = 0, mask = 0x01; i < HSI_NUM_SMALL_BLOCKS; i++, mask <<= 1)
		if (mem == small_block[i]) {
#if HSI_MEM_DEBUG
			if (mask & small_block_avail)
				return 0; //mipi_hsi_error(MIPI_HSI_ERROR_MEM_NOT_ALLOCATED, __LINE__, __FILE__);
#endif  /* HSI_MEM_DEBUG */
			small_block_avail |= mask;
#if HSI_MEM_DEBUG
			mem_dbg_small_block_cnt++;
			if (mem_dbg_small_block_cnt > mem_dbg_small_block_max)
				mem_dbg_small_block_max = mem_dbg_small_block_cnt;
#endif /* HSI_MEM_DEBUG */
			return 0;
		}

	for (i = 0, mask = 0x01; i < HSI_NUM_MEDIUM_BLOCKS; i++, mask <<= 1)
		if (mem == medium_block[i]) {
#if HSI_MEM_DEBUG
			if (mask & medium_block_avail)
				return 0;//mipi_hsi_error(MIPI_HSI_ERROR_MEM_NOT_ALLOCATED, __LINE__, __FILE__);
#endif  /* HSI_MEM_DEBUG */
			medium_block_avail |= mask;
#if HSI_MEM_DEBUG
			mem_dbg_medium_block_cnt++;
			if (mem_dbg_medium_block_cnt > mem_dbg_medium_block_max)
				mem_dbg_medium_block_max = mem_dbg_medium_block_cnt;
#endif /* HSI_MEM_DEBUG */
			return 0;
		}

	for (i = 0, mask = 0x01; i < HSI_NUM_LARGE_BLOCKS; i++, mask <<= 1)
		if (mem == large_block[i]) {
#if HSI_MEM_DEBUG
			if (mask & large_block_avail)
				return 0; //mipi_hsi_error(MIPI_HSI_ERROR_MEM_NOT_ALLOCATED, __LINE__, __FILE__);
#endif  /* HSI_MEM_DEBUG */
			large_block_avail |= mask;
#if HSI_MEM_DEBUG
			mem_dbg_large_block_cnt++;
			if (mem_dbg_large_block_cnt > mem_dbg_large_block_max)
				mem_dbg_large_block_max = mem_dbg_large_block_cnt;
#endif /* HSI_MEM_DEBUG */
			return 0;
		}

  /* free fallback memory */
	for (i=0; i<HSI_NUM_FALLBACK_BLOCKS; i++) {
		if (fallback_block[i] == mem) {
			kfree(mem);
			fallback_block[i] = NULL;
			return 0;
		}
	}
#endif
	return 0;
} /* mipi_hsi_mem_free */

                                                          
