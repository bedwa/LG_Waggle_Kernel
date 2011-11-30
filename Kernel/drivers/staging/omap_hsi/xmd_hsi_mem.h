/* 
 * arch/arm/mach-omap2/xmd_hsi_mem.h.h
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

#if !defined(MIPI_HSI_MEM__H)
#define MIPI_HSI_MEM__H


/*****************************************************************************/
/* INCLUDES                                                                  */
/*****************************************************************************/


/*****************************************************************************/
/* DEFINES                                                                   */
/*****************************************************************************/


#define IFX_HSI_BUFFERS  //If this is defined, preallocated buffers are used else mem is allocated only when required.
#define HSI_DMA_BUFFERS  //If this is defined, preallocated mem is obtained from dma_alloc_coherent else static pools are used.

/* pool for dynamic mem allocation */
#ifndef HSI_DMA_BUFFERS
#define HSI_SMALL_BLOCK_SIZE               512
#define HSI_MEDIUM_BLOCK_SIZE             (2*1024)
#define HSI_LARGE_BLOCK_SIZE              ((32 * 1024))
#else
#define HSI_SMALL_BLOCK_SIZE               512
#define HSI_MEDIUM_BLOCK_SIZE              2048
#define HSI_LARGE_BLOCK_SIZE               16384
#endif

#define HSI_NUM_SMALL_BLOCKS          30
#define HSI_NUM_MEDIUM_BLOCKS         15
#define HSI_NUM_LARGE_BLOCKS          8
#define HSI_NUM_FALLBACK_BLOCKS       32


#define HSI_MEM_DEBUG                   0
/* statistics */
#define MEM_STAT_MIN_SIZE               16
#define MEM_STAT_RANGE                  16


/*****************************************************************************/
/* TYPE DEFINITIONS                                                          */
/*****************************************************************************/


/*****************************************************************************/
/* PROTOTYPES                                                                */
/*****************************************************************************/

int mipi_hsi_mem_init(void);
int mipi_hsi_mem_uninit(void);
void* mipi_hsi_mem_alloc(int size);
int mipi_hsi_mem_free(unsigned char* mem);



#endif /* MIPI_HSI_MEM__H */


