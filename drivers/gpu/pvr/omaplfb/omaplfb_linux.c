/**********************************************************************
 *
 * Copyright(c) 2008 Imagination Technologies Ltd. All rights reserved.
 * 
 * This program is free software; you can redistribute it and/or modify it
 * under the terms and conditions of the GNU General Public License,
 * version 2, as published by the Free Software Foundation.
 * 
 * This program is distributed in the hope it will be useful but, except 
 * as otherwise stated in writing, without any warranty; without even the 
 * implied warranty of merchantability or fitness for a particular purpose. 
 * See the GNU General Public License for more details.
 * 
 * You should have received a copy of the GNU General Public License along with
 * this program; if not, write to the Free Software Foundation, Inc.,
 * 51 Franklin St - Fifth Floor, Boston, MA 02110-1301 USA.
 * 
 * The full GNU General Public License is included in this distribution in
 * the file called "COPYING".
 *
 * Contact Information:
 * Imagination Technologies Ltd. <gpl-support@imgtec.com>
 * Home Park Estate, Kings Langley, Herts, WD4 8LZ, UK 
 *
 ******************************************************************************/

#ifndef AUTOCONF_INCLUDED
#include <linux/config.h>
#endif

#include <linux/version.h>
#include <linux/module.h>
#include <linux/fb.h>
#include <asm/io.h>

#if (LINUX_VERSION_CODE >= KERNEL_VERSION(2,6,32))
#include <plat/vrfb.h>
#include <plat/display.h>
#else
#include <mach/vrfb.h>
#include <mach/display.h>
#endif

#ifdef RELEASE
#include <../drivers/video/omap2/omapfb/omapfb.h>
#undef DEBUG
#else
#undef DEBUG
#include <../drivers/video/omap2/omapfb/omapfb.h>
#endif

#if defined(CONFIG_OUTER_CACHE)  /* Kernel config option */
#include <asm/cacheflush.h>
#define HOST_PAGESIZE			(4096)
#define HOST_PAGEMASK			(~(HOST_PAGESIZE-1))
#define HOST_PAGEALIGN(addr)	(((addr)+HOST_PAGESIZE-1)&HOST_PAGEMASK)
#endif

#if defined(LDM_PLATFORM)
#include <linux/platform_device.h>
#if defined(SGX_EARLYSUSPEND)
#include <linux/earlysuspend.h>
#endif
#endif

//LGE_CHANGE_S [taekeun1.kim@lge.com] 2010-12-27, P920 : For GFX dma copy.
#if defined(CONFIG_GFX_DMA_COPY) || defined(CONFIG_HDMI_DMA_COPY)
#include <plat/dma.h>
#include <linux/time.h>
#endif

#include <linux/delay.h>


//LGE_CHANGE_E [taekeun1.kim@lge.com]

/* LGE_CHANGE_S [wonki.choi@lge.com] Overlay Refactoring 2011-1-24*/
#include <mach/tiler.h>
/* LGE_CHANGE_E [wonki.choi@lge.com] 2011-1-24 */

#include "img_defs.h"
#include "servicesext.h"
#include "kerneldisplay.h"
#include "omaplfb.h"
#include "pvrmodule.h"

/* LGE_CHANGE [wonki.choi@lge.com] 2010-10-21 S3D Extension*/
#include <plat/vram.h>
MODULE_SUPPORTED_DEVICE(DEVNAME);

/* LGE_CHANGE_S [wonki.choi@lge.com] Overlay Refactoring 2011-2-05*/
extern struct omap_overlay_info omap_overlay_info_req[4];
/* LGE_CHANGE_E [wonki.choi@lge.com] 2011-2-05 */

//LGE_CHANGE_S [taekeun1.kim@lge.com] 2010-12-27, P920 : For GFX dma copy.
#ifdef CONFIG_GFX_DMA_COPY
static struct {
	struct completion compl;
	int lch;
} gfx_dma[2];
#endif
//LGE_CHANGE_E [taekeun1.kim@lge.com]

//LGE_CHANGE_S [taekeun1.kim@lge.com] 2011-2-17, P920 : For HDMI dma copy.
#ifdef CONFIG_HDMI_DMA_COPY 
#define HDMI_DMA_MAX	3

enum hdmi_state {
	HDMI_DMA_DONE = 0,
	HDMI_DMA_TRANSFERRING = 1,
	HDMI_DMA_DISPLAY = 2,
};

static struct hdmi_dma_type {
	struct completion compl;
	struct omap_overlay *hdmi; 
	int lch;
	int frame_num;
	int frame_pos;
	int curr_frame;
	int display_queue;
	int queue[HDMI_DMA_MAX];
	enum hdmi_state state;
	struct omap_overlay_info info[HDMI_DMA_MAX];
	struct timeval prev_time;
};

struct work_struct hdmi_dma_work;
static struct hdmi_dma_type hdmi_dma;

int hdmi_frame_num;
#endif
//LGE_CHANGE_E [taekeun1.kim@lge.com]

#if defined(CONFIG_OUTER_CACHE)  /* Kernel config option */
#if defined(__arm__)
static void per_cpu_cache_flush_arm(void *arg)
{
    PVR_UNREFERENCED_PARAMETER(arg);
    flush_cache_all();
}
#endif
#endif

//LGE_CHANGE_S [taekeun1.kim@lge.com] 2010-12-27, P920 : For GFX dma copy.    
#ifdef CONFIG_GFX_DMA_COPY 
static void _omap_gfx_dma_cb_even(int lch, u16 ch_status, void *data)
{
        struct completion *compl = data;
        complete(compl);
}

static void _omap_gfx_dma_cb_odd(int lch, u16 ch_status, void *data)
{
        struct completion *compl = data;
        complete(compl);
}
#endif
//LGE_CHANGE_E [taekeun1.kim@lge.com]

//LGE_CHANGE_S [taekeun1.kim@lge.com] 2010-12-27, P920 : For GFX dma copy.    
#ifdef CONFIG_HDMI_DMA_COPY 
static void _omap_hdmi_dma_cb(int lch, u16 ch_status, void *data)
{
	struct hdmi_dma_type *hdmi_dma = data;
	
	complete(&hdmi_dma->compl);
	hdmi_dma->queue[hdmi_dma->display_queue++] = hdmi_dma->curr_frame;
	//if(hdmi_dma->display_queue < HDMI_DMA_MAX) hdmi_dma->state = HDMI_DMA_DONE;
	hdmi_dma->state = HDMI_DMA_DONE;
	schedule_work(&hdmi_dma_work);
	printk("Q:%d %d\n", hdmi_dma->display_queue, hdmi_dma->curr_frame);
}

static void hdmi_display_worker(struct work_struct *work)
{
	int ch, queue_pos;
	int i;
	struct omap_overlay *hdmi;
	struct omap_overlay_manager *manager;

	queue_pos = --hdmi_dma.display_queue;
	if((queue_pos >= HDMI_DMA_MAX) || (queue_pos < 0)) {
		hdmi_dma.display_queue = 0;
		hdmi_dma.curr_frame = 0;
		hdmi_dma.state = HDMI_DMA_DONE;
		printk("DOLCOM : eerr\n");
		return;
	}
	ch = hdmi_dma.queue[0];
	printk("DQ:%d %x\n", ch, hdmi_dma.info[ch].paddr);
	for(i=0; i<queue_pos; i++) hdmi_dma.queue[i] = hdmi_dma.queue[i+1];
	hdmi = hdmi_dma.hdmi;
	manager = hdmi->manager;

    if (hdmi && manager) {
		manager->wait_for_vsync(manager);
		hdmi->set_overlay_info(hdmi, &hdmi_dma.info[ch]);
		manager->apply(manager);
	}
	//if(hdmi_dma.display_queue >= HDMI_DMA_MAX-1) hdmi_dma.state = HDMI_DMA_DONE;
}

#endif
//LGE_CHANGE_E [taekeun1.kim@lge.com]

/*
 * Kernel malloc
 * in: ui32ByteSize
 */
void *OMAPLFBAllocKernelMem(unsigned long ui32ByteSize)
{
	void *p;

#if defined(CONFIG_OUTER_CACHE)  /* Kernel config option */
	IMG_VOID *pvPageAlignedCPUPAddr;
	IMG_VOID *pvPageAlignedCPUVAddr;
	IMG_UINT32 ui32PageOffset;
	IMG_UINT32 ui32PageCount;
#endif
	p = kmalloc(ui32ByteSize, GFP_KERNEL);

	if(!p)
		return 0;

#if defined(CONFIG_OUTER_CACHE)  /* Kernel config option */
	ui32PageOffset = (IMG_UINT32) p & (HOST_PAGESIZE - 1);
	ui32PageCount = HOST_PAGEALIGN(ui32ByteSize + ui32PageOffset) / HOST_PAGESIZE;

	pvPageAlignedCPUVAddr = (IMG_VOID *)((IMG_UINT8 *)p - ui32PageOffset);
	pvPageAlignedCPUPAddr = (IMG_VOID*) __pa(pvPageAlignedCPUVAddr);

#if defined(__arm__)
      on_each_cpu(per_cpu_cache_flush_arm, NULL, 1);
#endif
	outer_cache.flush_range((unsigned long) pvPageAlignedCPUPAddr, (unsigned long) ((pvPageAlignedCPUPAddr + HOST_PAGESIZE*ui32PageCount) - 1));
#endif
	return p;
}

/*
 * Kernel free
 * in: pvMem
 */
void OMAPLFBFreeKernelMem(void *pvMem)
{
	kfree(pvMem);
}

/*
 * Here we get the function pointer to get jump table from
 * services using an external function.
 * in: szFunctionName
 * out: ppfnFuncTable
 */
OMAP_ERROR OMAPLFBGetLibFuncAddr (char *szFunctionName,
	PFN_DC_GET_PVRJTABLE *ppfnFuncTable)
{
	if(strcmp("PVRGetDisplayClassJTable", szFunctionName) != 0)
	{
		ERROR_PRINTK("Unable to get function pointer for %s"
			" from services", szFunctionName);
		return OMAP_ERROR_INVALID_PARAMS;
	}
	*ppfnFuncTable = PVRGetDisplayClassJTable;

	return OMAP_OK;
}

#if defined(FLIP_TECHNIQUE_FRAMEBUFFER)
/*
 * Presents the flip in the display with the framebuffer API
 * in: psSwapChain, aPhyAddr
 */
static void OMAPLFBFlipNoLock(OMAPLFB_SWAPCHAIN *psSwapChain,
	unsigned long aPhyAddr)
{
	OMAPLFB_DEVINFO *psDevInfo = (OMAPLFB_DEVINFO *)psSwapChain->pvDevInfo;
	struct fb_info *framebuffer = psDevInfo->psLINFBInfo;

	/* Get the framebuffer physical address base */
	unsigned long fb_base_phyaddr =
		psDevInfo->sSystemBuffer.sSysAddr.uiAddr;

	/* Calculate the virtual Y to move in the framebuffer */
	framebuffer->var.yoffset =
		(aPhyAddr - fb_base_phyaddr) / framebuffer->fix.line_length;
	framebuffer->var.activate = FB_ACTIVATE_FORCE;
	fb_set_var(framebuffer, &framebuffer->var);
}

#elif defined(FLIP_TECHNIQUE_OVERLAY)
/* LGE_CHANGE_S [wonki.choi@lge.com] 2010-09-30 S3D Extension*/
//Temp code

#define INTERLEAVE_BUFFER_NUM	2

static struct InterleaveBuffer {
	unsigned char *buffer;		//buffer
	unsigned int size;			//buffer size
	unsigned long	paddr;				//physical address of buffer
	u32 	fbbuffer_paddr;		//physical address of frame buffer mapped
} interleave_buffers[INTERLEAVE_BUFFER_NUM];

static int getInterleaveBufferSize(struct fb_info *fbi)
{
	return (fbi->fix.line_length * fbi->var.yres);
}

static struct InterleaveBuffer* allocInterleaveBufferAt(int at, int size, u32 paddr)
{
	struct InterleaveBuffer *IB = &(interleave_buffers[at]);
	int r;
	if ( IB->buffer!=NULL )
	{
		if ( IB->size==size )
		{
			printk("S3D : size same re-use interlave buffer\n");
			IB->fbbuffer_paddr = paddr;
			return IB;
		}
//		kfree(IB->buffer);
		omap_vram_free(IB->paddr, IB->size);
		IB->buffer = NULL;
	}
//	IB->buffer = kmalloc(size, GFP_DMA | __GFP_COLD);
	r = omap_vram_alloc(OMAP_VRAM_MEMTYPE_SDRAM, size, &IB->paddr);
//	if ( IB->buffer!=NULL )
	if ( !r )
	{
//		IB->paddr = virt_to_bus(IB->buffer);
		IB->buffer = ioremap_wc(IB->paddr, size);
		IB->size = size;
		IB->fbbuffer_paddr = paddr;
		printk("S3D : allocat %dth buffer success. size:%d, vaddr:%p paddr:0x%lx for 0x%x\n",
				at,
				size,
				IB->buffer,
				IB->paddr,
				paddr
				);
		return IB;
	}
	else
	{
		printk("S3D : alloc fail for 0x%x\n", paddr);
		return NULL;
	}
}

static struct InterleaveBuffer* findInterleaveBuffer(struct fb_info *fbi, struct omap_overlay_info *overlay_info)
{
	int i;
	int size;
	static int first_call = 1;
	if ( first_call )
	{
		memset(interleave_buffers, sizeof(interleave_buffers), 0);
		first_call = 0;
	}
	size = getInterleaveBufferSize(fbi);
	for(i=0;i<INTERLEAVE_BUFFER_NUM;i++)
	{
		struct InterleaveBuffer *IB;
		IB = &(interleave_buffers[i]);
//		printk("looking size: %d, current size :%d, looking paddr: 0x%x current paddr:0x%x\n",
//				size, IB->size, overlay_info->paddr, IB->fbbuffer_paddr);
		if ( IB->buffer!=NULL && IB->fbbuffer_paddr==overlay_info->paddr && IB->size==size )
		{
//			printk("S3D : found interleave buffer for 0x%x (vaddr:0x%x, paddr:0x%x)n",
//					overlay_info->paddr,
//					IB->buffer, IB->paddr);
			return IB;
		}
	}
	//need alloc
	printk("S3D : not found interleave for 0x%x", overlay_info->paddr);
	for(i=0;i<INTERLEAVE_BUFFER_NUM;i++)
	{
		struct InterleaveBuffer *IB;
		IB = &(interleave_buffers[i]);
		if ( IB->buffer==NULL )
		{
			struct InterleaveBuffer *ret = allocInterleaveBufferAt(i, size, overlay_info->paddr);
			if ( ret==NULL )
				break;
			else
			{
				printk("S3D : alloc success : interleave buffer for 0x%x\n", overlay_info->paddr);
				return ret;
			}
		}
	}
	printk("S3D : allocation fail : interleave buffer fail\n");
	return NULL;
}

static void makeinterlave(struct fb_info *fbi, struct omapfb_info *ofbi, struct omap_overlay *overlay, struct omap_overlay_info *pInfo)
{
	struct InterleaveBuffer* IB;
	struct omap_overlay_info overlay_info = *pInfo;
	IB = findInterleaveBuffer(fbi, &overlay_info);
	if ( IB!=NULL )
	{
#ifdef CONFIG_GFX_DMA_COPY
//LGE_CHANGE_S [taekeun1.kim@lge.com] 2010-12-27, P920 : For GFX dma copy. 

		if (wait_for_completion_timeout(&gfx_dma[0].compl, msecs_to_jiffies(1000)) == 0) {
			omap_stop_dma(gfx_dma[0].lch);
			DEBUG_PRINTK("GFX DMA: dma timeout while transferring\n");
		}

		omap_set_dma_transfer_params(gfx_dma[0].lch, OMAP_DMA_DATA_TYPE_S32,
			(fbi->fix.line_length>>2), (fbi->var.yres)>>1, 0, 0, 0);
		omap_set_dma_src_params(gfx_dma[0].lch, 0, OMAP_DMA_AMODE_POST_INC,
			overlay_info.paddr, 1, 1);
		omap_set_dma_dest_params(gfx_dma[0].lch, 0, OMAP_DMA_AMODE_DOUBLE_IDX,
			IB->paddr + fbi->fix.line_length, 1, (fbi->fix.line_length) + 1);

		omap_start_dma(gfx_dma[0].lch);
#if 0
		if (wait_for_completion_timeout(&gfx_dma[0].compl, msecs_to_jiffies(1000)) == 0) {
			omap_stop_dma(gfx_dma[0].lch);
			DEBUG_PRINTK("GFX DMA: dma timeout while transferring\n");
		}

		if (wait_for_completion_timeout(&gfx_dma[1].compl, msecs_to_jiffies(1000)) == 0) {
			omap_stop_dma(gfx_dma[1].lch);
			DEBUG_PRINTK("GFX DMA: dma timeout while transferring\n");
		}

		omap_set_dma_transfer_params(gfx_dma[0].lch, OMAP_DMA_DATA_TYPE_S32,
			(fbi->fix.line_length>>2), (fbi->var.yres)>>1, 0, 0, 0);
		omap_set_dma_src_params(gfx_dma[0].lch, 0, OMAP_DMA_AMODE_POST_INC,
			overlay_info.paddr, 1, 1);
		omap_set_dma_dest_params(gfx_dma[0].lch, 0, OMAP_DMA_AMODE_DOUBLE_IDX,
			IB->paddr + fbi->fix.line_length, 1, (fbi->fix.line_length) + 1);

		omap_set_dma_transfer_params(gfx_dma[1].lch, OMAP_DMA_DATA_TYPE_S32,
			(fbi->fix.line_length>>2), (fbi->var.yres)>>1, 0, 0, 0);
		omap_set_dma_src_params(gfx_dma[1].lch, 0, OMAP_DMA_AMODE_POST_INC,
			overlay_info.paddr + (fbi->fix.line_length * (fbi->var.yres>>1)), 1, 1);
		omap_set_dma_dest_params(gfx_dma[1].lch, 0, OMAP_DMA_AMODE_DOUBLE_IDX,
			IB->paddr, 1, (fbi->fix.line_length) + 1);

		omap_start_dma(gfx_dma[0].lch);
		omap_start_dma(gfx_dma[1].lch);
#endif

//LGE_CHANGE_E [taekeun1.kim@lge.com]
#else
//			memcpy(IB->buffer, overlay_info.vaddr, IB->size);
		{
			//copy interleave
			int stride;
			int height;
			unsigned char *src, *dst;
			int i;

			stride = fbi->fix.line_length;
			height = fbi->var.yres;
			src = overlay_info.vaddr;
			dst = IB->buffer;
//				printk("Interleave Copy line %d address:%p\n", height, IB->buffer);
//				printk("overlay->roation_type : %d\n", overlay_info.rotation_type);
			for(i=0;i<height;i++)
			{
				if ( i < (height/2) )
				{
					//left frame copy to odd line
					unsigned char *odd_line;
					odd_line = dst + stride * (i*2 + 1);
					memcpy(odd_line, src, stride);
				}
				else
				{
					//right frame copy to even line
					unsigned char *even_line;
					even_line = dst + stride * ( (i-height/2) * 2 + 1 -1 );
					memcpy(even_line, src, stride);
				}
				src += stride;
			}
		}
#endif
//			flush_cache_all();
		overlay_info.paddr = IB->paddr;
		overlay_info.vaddr = IB->buffer;
//			printk("S3D : set oerlay paddr:0x%x vaddr 0x%x",
//					overlay_info.paddr,
//					overlay_info.vaddr );
//			if ( cacheflush(IB->buffer, IB->size, DCACHE ) )
//				printk("Interleave buffer flush failed\n");
//			overlay->set_overlay_info(overlay, &overlay_info);
//			return;
	}
	overlay_info.s3d_type = omap_dss_overlay_s3d_interlaced;
	overlay->set_overlay_info(overlay, &overlay_info);
}
/* LGE_CHANGE_E [wonki.choi@lge.com] 2010-08-30 */

/* LGE_CHANGE_S [wonki.choi@lge.com] Overlay Refactoring 2011-2-05*/
/**
 * Alloc Tiler Buffer for HDMI
 */
int AllocTilerForHdmi(OMAPLFB_SWAPCHAIN *psSwapChain, OMAPLFB_DEVINFO *psDevInfo)
{
	unsigned long w, h;
	unsigned long line_length;
	int	err;

	if ( psSwapChain->stHdmiTiler.alloc )
		return 0;

	w = psDevInfo->sFBInfo.ulWidth;
	{
		struct fb_fix_screeninfo *fix = &psDevInfo->psLINFBInfo->fix;
		h = fix->smem_len / fix->line_length;
	}

	if ( psSwapChain->stHdmiTiler.overlay==NULL )
	{
		ERROR_PRINTK("Get HDMI Overlay fail\n");
		return -EINVAL;
	}
	if ( psDevInfo->sFBInfo.ePixelFormat!= PVRSRV_PIXEL_FORMAT_ARGB8888 )
	{
		ERROR_PRINTK("Allocated Bufffer is Not ARG32\n");
		return -EINVAL;
	}
	line_length = ALIGN(w*4, PAGE_SIZE);
	w = line_length / 4;

	//alloc tiler
#ifdef CONFIG_HDMI_DMA_COPY
	err = tiler_alloc(TILFMT_32BIT,	w, 800*3,
			(u32 *)&psSwapChain->stHdmiTiler.pAddr );
	if ( err )
	{
		ERROR_PRINTK("Allocation Tiler Memory Failed\n");
		return -ENOMEM;
	}
#else
	err = tiler_alloc(TILFMT_32BIT,	w,h,
			(u32 *)&psSwapChain->stHdmiTiler.pAddr );
	if ( err )
	{
		ERROR_PRINTK("Allocation Tiler Memory Failed\n");
		return -ENOMEM;
	}
#endif
	psSwapChain->stHdmiTiler.vStride = line_length;
	psSwapChain->stHdmiTiler.pStride =
			tiler_stride( tiler_get_natural_addr((void*)psSwapChain->stHdmiTiler.pAddr) );
	psSwapChain->stHdmiTiler.pSize = psSwapChain->stHdmiTiler.pStride * h;
	psSwapChain->stHdmiTiler.vAddr	=
			__arm_multi_strided_ioremap(1,
					(unsigned long*)&psSwapChain->stHdmiTiler.pAddr,
					&psSwapChain->stHdmiTiler.pSize,
					&psSwapChain->stHdmiTiler.pStride,
					&psSwapChain->stHdmiTiler.vStride,
					MT_DEVICE_WC);
	if ( psSwapChain->stHdmiTiler.vAddr == NULL )
	{
		ERROR_PRINTK("Tiler Memory remap Failed\n");
		tiler_free(psSwapChain->stHdmiTiler.pAddr);
		return -ENOMEM;
	}
	psSwapChain->stHdmiTiler.alloc = true;
	DEBUG_PRINTK("Tiler Setup Success pAddr:0x%x, pStride:%d, pSize:%d, vAddr:0x%x, vStride:%d, h:%d\n",
			psSwapChain->stHdmiTiler.pAddr,
			psSwapChain->stHdmiTiler.pStride,
			psSwapChain->stHdmiTiler.pSize,
			psSwapChain->stHdmiTiler.vAddr,
			psSwapChain->stHdmiTiler.vStride,
			h);

//	{
//		//initial setup overlay info
//		struct omap_overlay	*overlay;
//		struct omap_overlay_info info;
//		overlay = psSwapChain->stHdmiTiler.overlay;
//		overlay->get_overlay_info(overlay, &info);
//
//		info.color_mode = OMAP_DSS_COLOR_ARGB32;
//		info.rotation_type = OMAP_DSS_ROT_TILER;
//
//		info.paddr = psSwapChain->stHdmiTiler.pAddr;
//		info.vaddr = NULL;
//
//		info.rotation = 0;
//
//		info.width = psDevInfo->sFBInfo.ulWidth;
//		info.height = psDevInfo->sFBInfo.ulHeight;
//
//		info.pos_x = 0;
//		info.pos_y = 0;
//		info.out_width = info.width;
//		info.out_height = info.height;
//
//		if ( overlay->set_overlay_info(overlay, &info) )
//		{
//			ERROR_PRINTK("Initial overlay1 setup failed\n");
//			return;
//		}
//	}

	return 0;
}

/* LGE_CHANGE_S [wonki.choi@lge.com] HDMI rework 2011-4-21*/
extern int hdmi_video_prepare_change(void *id, int layer_count, bool add_or_change, char *reason);
extern int hdmi_video_commit_change(void *id);
/* LGE_CHANGE_E [wonki.choi@lge.com] 2011-4-21 */

void FreeTilerForHdmi(OMAPLFB_SWAPCHAIN *psSwapChain)
{
	if(psSwapChain->stHdmiTiler.alloc != true)
		return;
	//disable HDMI Layer first
	{
		struct omap_overlay_info info;
		struct omap_overlay *hdmi;
		struct omap_overlay_manager *manager;

		hdmi = psSwapChain->stHdmiTiler.overlay;
		manager = hdmi->manager;
		hdmi->get_overlay_info(hdmi, &info);
		if ( info.enabled )
		{
			info.enabled = false;
			info.paddr = 0;
			info.vaddr = NULL;		//no need
			hdmi_video_prepare_change(psSwapChain, 0, false, "Free HDMI Tiler Memory");
			if ( hdmi->set_overlay_info(hdmi, &info) )
				ERROR_PRINTK("Set HDMI Overlay setting failed");
			if (manager) {
				manager->apply(manager);
			}
			hdmi_video_commit_change(psSwapChain);
			mdelay(100); // WaitForVsyncOfHDMI
		}
	}
	//unmap
	if ( psSwapChain->stHdmiTiler.vAddr!=NULL )
	{
		iounmap(psSwapChain->stHdmiTiler.vAddr);
		psSwapChain->stHdmiTiler.vAddr= NULL;
	}
	//tiler free
	if ( psSwapChain->stHdmiTiler.pAddr!=0 )
	{
		tiler_free(psSwapChain->stHdmiTiler.pAddr);
		psSwapChain->stHdmiTiler.pAddr = 0;
	}
	psSwapChain->stHdmiTiler.alloc = false;
}

/* LGE_CHANGE_E [wonki.choi@lge.com] 2011-2-05 */

/* LGE_CHANGE_S [wonki.choi@lge.com] Code Rearrange 2011-5-24*/
#ifdef CONFIG_HDMI_DMA_COPY
static void OMAPLFBFliepNoLock_HDMI(OMAPLFB_SWAPCHAIN *psSwapChain,
		OMAPLFB_DEVINFO *psDevInfo,
		struct omapfb_info *ofbi,
		struct fb_info *framebuffer,
		unsigned long fb_offset)
{
	struct omap_overlay *ovl_hdmi;
	struct omap_overlay_info info;
	struct omap_overlay *hdmi;
	struct omap_overlay_manager *manager;
	bool overlay_change_requested = false;
	enum omap_dss_overlay_s3d_type  s3d_type_in_video_hdmi = omap_dss_overlay_s3d_none;

	ovl_hdmi = omap_dss_get_overlay(3);
	if(ovl_hdmi->info.enabled)
		s3d_type_in_video_hdmi = ovl_hdmi->info.s3d_type;


	hdmi = psSwapChain->stHdmiTiler.overlay;
	manager = hdmi->manager;
	hdmi->get_overlay_info(hdmi, &info);

	//not good...
	if ( omap_overlay_info_req[hdmi->id].status==2 )
	{

		info.enabled = omap_overlay_info_req[hdmi->id].enabled;
		info.rotation = omap_overlay_info_req[hdmi->id].rotation;
		info.pos_x = omap_overlay_info_req[hdmi->id].pos_x;
		info.pos_y = omap_overlay_info_req[hdmi->id].pos_y;
		info.out_width = omap_overlay_info_req[hdmi->id].out_width;
		info.out_height = omap_overlay_info_req[hdmi->id].out_height;
		info.global_alpha = omap_overlay_info_req[hdmi->id].global_alpha;
		info.zorder = omap_overlay_info_req[hdmi->id].zorder;

		printk("GUI HDMI layer change requested. req_enabled(%d)\n", omap_overlay_info_req[hdmi->id].enabled);
		omap_overlay_info_req[hdmi->id].status = 0;
		overlay_change_requested = true;
	}

	if ( info.enabled )
	{
		if ( !psSwapChain->stHdmiTiler.alloc )
		{
			if ( AllocTilerForHdmi(psSwapChain, psDevInfo) ) {

				ERROR_PRINTK("Alloc tiler memory for HDMI GUI cloning failed\n");
				printk("DOLCOM : return tiler alloc\n");
				return;
			}
		}

		if ( psSwapChain->stHdmiTiler.alloc )	//if Tiler memory is allocated
		{
			unsigned long line_offset;
			unsigned long w, h;
			unsigned long src_stride, dst_stride;
			unsigned long i;
			unsigned char *dst, *src;
			unsigned long pStride;
			u32 j, *src_4, *dst_4, *dst1_4;
			int ch;

			src_stride = psDevInfo->sFBInfo.ulByteStride;
			dst_stride = psSwapChain->stHdmiTiler.vStride;
			line_offset = fb_offset / src_stride;
			h = psDevInfo->sFBInfo.ulHeight;
			w = psDevInfo->sFBInfo.ulWidth;
			pStride = psSwapChain->stHdmiTiler.pStride;

			//Copy
			dst = (unsigned char*)psSwapChain->stHdmiTiler.vAddr +
				(line_offset * dst_stride);

			src = (unsigned char*)framebuffer->screen_base + fb_offset;


			DEBUG_PRINTK("Copy Start h:%d, src:0x%p src_stride:%d, dst:0x%p dst_stride:%d, line offset:%d, pStride:%d\n",
					h, src, src_stride, dst, dst_stride, line_offset, pStride);

			if( psSwapChain->s3d_type==omap_dss_overlay_s3d_side_by_side && s3d_type_in_video_hdmi == omap_dss_overlay_s3d_top_bottom)
			{
				for(j=0; j<h/2; j++)
				{
					src_4  = (u32 *)(src + src_stride*j);
					dst_4   = (u32 *)(dst + dst_stride*2*j);
					dst1_4 = (u32 *)(dst + dst_stride*2*j + w*2);
					for(i=0;i<w/2;i++)
					{
						*dst_4++ = *src_4;
						*dst1_4++ = *src_4++;
						src_4++;
					}
					src_4  = (u32 *)(src + src_stride*j);
					dst_4   = (u32 *)(dst + dst_stride*(2*j+1));
					dst1_4 = (u32 *)(dst + dst_stride*(2*j+1) + w*2);
					for(i=0;i<w/2;i++) {
						*dst_4++ = *src_4;
						*dst1_4++ = *src_4++;
						src_4++;
					}
				}
				info.s3d_type = omap_dss_overlay_s3d_top_bottom;
			}
			else
			{

				if(hdmi_dma.state == HDMI_DMA_DONE)
				{
					ch = hdmi_dma.frame_pos;
					hdmi->get_overlay_info(hdmi, &hdmi_dma.info[ch]);
					hdmi_dma.hdmi = hdmi;

					printk("S:%x\n", psSwapChain->stHdmiTiler.pAddr + (h * pStride * ch));

					omap_set_dma_transfer_params(hdmi_dma.lch, OMAP_DMA_DATA_TYPE_S32,
						src_stride>>2, h, 0, 0, 0);
					omap_set_dma_src_params(hdmi_dma.lch, 0, OMAP_DMA_AMODE_POST_INC,
						framebuffer->fix.smem_start + fb_offset, 1, 1);
					omap_set_dma_dest_params(hdmi_dma.lch, 0, OMAP_DMA_AMODE_DOUBLE_IDX,
						psSwapChain->stHdmiTiler.pAddr + (h * pStride * ch), 1, pStride - src_stride + 1 );
					omap_dma_set_prio_lch(hdmi_dma.lch, DMA_CH_PRIO_HIGH, DMA_CH_PRIO_HIGH);
					omap_set_dma_src_burst_mode(hdmi_dma.lch, OMAP_DMA_DATA_BURST_16);
					omap_set_dma_dest_burst_mode(hdmi_dma.lch, OMAP_DMA_DATA_BURST_16);

					omap_start_dma(hdmi_dma.lch);
					hdmi_dma.state = HDMI_DMA_TRANSFERRING;

					if ( omap_overlay_info_req[hdmi->id].status==2 )
					{
						hdmi_dma.info[ch].enabled = omap_overlay_info_req[hdmi->id].enabled;
						hdmi_dma.info[ch].rotation = omap_overlay_info_req[hdmi->id].rotation;
						hdmi_dma.info[ch].pos_x = omap_overlay_info_req[hdmi->id].pos_x;
						hdmi_dma.info[ch].pos_y = omap_overlay_info_req[hdmi->id].pos_y;
						hdmi_dma.info[ch].out_width = omap_overlay_info_req[hdmi->id].out_width;
						hdmi_dma.info[ch].out_height = omap_overlay_info_req[hdmi->id].out_height;
						hdmi_dma.info[ch].global_alpha = omap_overlay_info_req[hdmi->id].global_alpha;
						hdmi_dma.info[ch].zorder = omap_overlay_info_req[hdmi->id].zorder;
					}
					//fill info

					//@todo not good find another way later
					hdmi_dma.info[ch].color_mode = ofbi->overlays[0]->info.color_mode;
					hdmi_dma.info[ch].paddr = psSwapChain->stHdmiTiler.pAddr + (h * pStride * ch);
					hdmi_dma.info[ch].vaddr = NULL;		//no need
					if ( hdmi_dma.info[ch].rotation==OMAP_DSS_ROT_90 || hdmi_dma.info[ch].rotation==OMAP_DSS_ROT_270 )
					{
						hdmi_dma.info[ch].width = h;
						hdmi_dma.info[ch].height = w;
						hdmi_dma.info[ch].screen_width = h;
					}
					else
					{
						hdmi_dma.info[ch].width =w;
						hdmi_dma.info[ch].height = h;
						hdmi_dma.info[ch].screen_width = w;
					}
					hdmi_dma.info[ch].rotation_type = OMAP_DSS_ROT_TILER;
					hdmi_dma.info[ch].s3d_type = psSwapChain->s3d_type;

					hdmi_dma.curr_frame = hdmi_dma.frame_pos;
					if( ++hdmi_dma.frame_pos >= HDMI_DMA_MAX ) hdmi_dma.frame_pos = 0;

					if( omap_overlay_info_req[hdmi->id].status == 2) {
						omap_overlay_info_req[hdmi->id].status = 0;
					}
				}
				else printk("DOLCOM : DMA busy!!!!!!!!!!!!!!!!!\n");
			}

		} //end of copy & set
	} //overlay enabled
	else
	{
		if ( overlay_change_requested )
		{
			hdmi->set_overlay_info(hdmi, &info);
			manager->apply(manager);
		}
		if ( hdmi_video_prepare_change(psSwapChain, 0, false)!= 0 )	//connect & need commit
			hdmi_video_stop_by_external(psSwapChain, "GUI HDMI layer disabled");
	}

}
#else
//HDMI GUI Cloning
static void OMAPLFBFliepNoLock_HDMI(OMAPLFB_SWAPCHAIN *psSwapChain,
		OMAPLFB_DEVINFO *psDevInfo,
		struct omapfb_info *ofbi,
		struct fb_info *framebuffer,
		unsigned long fb_offset)
{
	struct omap_overlay *ovl_hdmi;
	struct omap_overlay_info info;
	struct omap_overlay *hdmi;
	struct omap_overlay_manager *manager;
	bool overlay_change_requested = false;
	enum omap_dss_overlay_s3d_type  s3d_type_in_video_hdmi = omap_dss_overlay_s3d_none;

	ovl_hdmi = omap_dss_get_overlay(3);
	if(ovl_hdmi->info.enabled)
		s3d_type_in_video_hdmi = ovl_hdmi->info.s3d_type;


	hdmi = psSwapChain->stHdmiTiler.overlay;
	if ( hdmi==NULL )
		return;
	manager = hdmi->manager;
	if ( manager==NULL )
		return;
	hdmi->get_overlay_info(hdmi, &info);

	//not good...
	if ( omap_overlay_info_req[hdmi->id].status==2 )
	{

		info.enabled = omap_overlay_info_req[hdmi->id].enabled;
		info.rotation = omap_overlay_info_req[hdmi->id].rotation;
		info.pos_x = omap_overlay_info_req[hdmi->id].pos_x;
		info.pos_y = omap_overlay_info_req[hdmi->id].pos_y;
		info.out_width = omap_overlay_info_req[hdmi->id].out_width;
		info.out_height = omap_overlay_info_req[hdmi->id].out_height;
		info.global_alpha = omap_overlay_info_req[hdmi->id].global_alpha;
		info.zorder = omap_overlay_info_req[hdmi->id].zorder;

		printk("GUI HDMI layer change requested. req_enabled(%d)\n", omap_overlay_info_req[hdmi->id].enabled);
		omap_overlay_info_req[hdmi->id].status = 0;
		overlay_change_requested = true;
	}

	if ( info.enabled )
	{
		if ( !psSwapChain->stHdmiTiler.alloc )
		{
			if ( AllocTilerForHdmi(psSwapChain, psDevInfo) ) {

				ERROR_PRINTK("Alloc tiler memory for HDMI GUI cloning failed\n");
				printk("DOLCOM : return tiler alloc\n");
				return;
			}
		}

		if ( psSwapChain->stHdmiTiler.alloc )	//if Tiler memory is allocated
		{
			unsigned long line_offset;
			unsigned long w, h;
			unsigned long src_stride, dst_stride;
			unsigned long i;
			unsigned char *dst, *src;
			unsigned long pStride;
			u32 j, *src_4, *dst_4, *dst1_4;
			int need_hdmi_commit;

			src_stride = psDevInfo->sFBInfo.ulByteStride;
			dst_stride = psSwapChain->stHdmiTiler.vStride;
			line_offset = fb_offset / src_stride;
			h = psDevInfo->sFBInfo.ulHeight;
			w = psDevInfo->sFBInfo.ulWidth;
			pStride = psSwapChain->stHdmiTiler.pStride;

			//Copy
			dst = (unsigned char*)psSwapChain->stHdmiTiler.vAddr +
				(line_offset * dst_stride);

			src = (unsigned char*)framebuffer->screen_base + fb_offset;

			DEBUG_PRINTK("Copy Start h:%d, src:0x%p src_stride:%d, dst:0x%p dst_stride:%d, line offset:%d, pStride:%d\n",
					h, src, src_stride, dst, dst_stride, line_offset, pStride);

			if( psSwapChain->s3d_type==omap_dss_overlay_s3d_side_by_side && s3d_type_in_video_hdmi == omap_dss_overlay_s3d_top_bottom) {
				for(j=0; j<h/2; j++) {
					src_4  = (u32 *)(src + src_stride*j);
					dst_4   = (u32 *)(dst + dst_stride*2*j);
					dst1_4 = (u32 *)(dst + dst_stride*2*j + w*2);
					for(i=0;i<w/2;i++) {
						*dst_4++ = *src_4;
						*dst1_4++ = *src_4++;
						src_4++;
					}
					src_4  = (u32 *)(src + src_stride*j);
					dst_4   = (u32 *)(dst + dst_stride*(2*j+1));
					dst1_4 = (u32 *)(dst + dst_stride*(2*j+1) + w*2);
					for(i=0;i<w/2;i++) {
						*dst_4++ = *src_4;
						*dst1_4++ = *src_4++;
						src_4++;
					}
				}
				info.s3d_type = omap_dss_overlay_s3d_top_bottom;
			} else
			{

				for(i=0;i<h;i++)
				{
					memcpy(dst, src, src_stride);
					dst += dst_stride;
					src += src_stride;
				}
				//Side by side
				info.s3d_type = psSwapChain->s3d_type;
			}

			info.color_mode = ofbi->overlays[0]->info.color_mode;
			info.paddr = psSwapChain->stHdmiTiler.pAddr + (line_offset * pStride);
			info.vaddr = NULL;		//no need
			if ( info.rotation==OMAP_DSS_ROT_90 || info.rotation==OMAP_DSS_ROT_270 )
			{
				info.width = h;
				info.height = w;
				info.screen_width = h;
			}
			else
			{
				info.width =w;
				info.height = h;
				info.screen_width = w;
			}

			info.rotation_type = OMAP_DSS_ROT_TILER;
			need_hdmi_commit = hdmi_video_prepare_change(psSwapChain, 1, true, "GUI Cloning");
			if ( hdmi->set_overlay_info(hdmi, &info) ) {
				ERROR_PRINTK("Set HDMI Overlay setting failed");
			}
			manager->apply(manager);
			if ( need_hdmi_commit > 0 )
			{
				hdmi_video_commit_change(psSwapChain);
			}
		} //end of copy & set
	} //overlay enabled
	else
	{
		if ( overlay_change_requested )
		{
			hdmi_video_prepare_change(psSwapChain, 0, false, "GUI Cloning disabled");
			hdmi->set_overlay_info(hdmi, &info);
			manager->apply(manager);
			hdmi_video_commit_change(psSwapChain);
		}
	}
}

#endif

/* LGE_CHANGE_E [wonki.choi@lge.com] 2011-5-24 */


 /*
  * Presents the flip in the display with the DSS2 overlay API
  * in: psSwapChain, aPhyAddr
  */
static void OMAPLFBFlipNoLock(OMAPLFB_SWAPCHAIN *psSwapChain,
	unsigned long aPhyAddr)
{
	OMAPLFB_DEVINFO *psDevInfo = (OMAPLFB_DEVINFO *)psSwapChain->pvDevInfo;
	struct fb_info * framebuffer = psDevInfo->psLINFBInfo;
	struct omapfb_info *ofbi = FB2OFB(framebuffer);
	unsigned long fb_offset;
	int i;


	fb_offset = aPhyAddr - psDevInfo->sSystemBuffer.sSysAddr.uiAddr;

	for(i = 0; i < ofbi->num_overlays ; i++)
	{
		struct omap_dss_device *display = NULL;
		struct omap_dss_driver *driver = NULL;
		struct omap_overlay_manager *manager;
		struct omap_overlay *overlay;
		struct omap_overlay_info overlay_info;

		overlay = ofbi->overlays[i];
		manager = overlay->manager;
		overlay->get_overlay_info( overlay, &overlay_info );

		overlay_info.paddr = framebuffer->fix.smem_start + fb_offset;
		overlay_info.vaddr = framebuffer->screen_base + fb_offset;
		/* LGE_CHANGE_S [wonki.choi@lge.com] 2010-09-30 S3D Extension*/
		if ( i==0 ) {
			overlay_info.s3d_type = psSwapChain->s3d_type;
			if ( overlay_info.s3d_type==omap_dss_overlay_s3d_side_by_side )
				makeinterlave(framebuffer, ofbi, overlay, &overlay_info);
			else
				overlay->set_overlay_info(overlay, &overlay_info);
		} else
			overlay->set_overlay_info(overlay, &overlay_info);
		/* LGE_CHANGE_E [wonki.choi@lge.com] 2010-09-30 S3D Extension*/

		if (manager) {
			display = manager->device;
			/* No display attached to this overlay, don't update */
			if (!display)
				continue;
			driver = display->driver;
			manager->apply(manager);
		}

		if (dss_ovl_manually_updated(overlay)) {
			if (driver->sched_update)
				driver->sched_update(display, 0, 0,
							overlay_info.width,
							overlay_info.height);
			else if (driver->update)
				driver->update(display, 0, 0,
							overlay_info.width,
							overlay_info.height);

		}
	}
	/* LGE_CHANGE_S [wonki.choi@lge.com] Code Rearrange 2011-5-24*/
	OMAPLFBFliepNoLock_HDMI(psSwapChain, psDevInfo, ofbi, framebuffer, fb_offset);
	/* LGE_CHANGE_E [wonki.choi@lge.com] 2011-5-24 */
}

#else
#error No flipping technique selected, please define \
	FLIP_TECHNIQUE_FRAMEBUFFER or FLIP_TECHNIQUE_OVERLAY
#endif

void OMAPLFBFlip(OMAPLFB_SWAPCHAIN *psSwapChain, unsigned long aPhyAddr)
{
	OMAPLFB_DEVINFO *psDevInfo = (OMAPLFB_DEVINFO *)psSwapChain->pvDevInfo;
	struct fb_info *framebuffer = psDevInfo->psLINFBInfo;
	struct omapfb_info *ofbi = FB2OFB(framebuffer);
	struct omapfb2_device *fbdev = ofbi->fbdev;

	omapfb_lock(fbdev);
	OMAPLFBFlipNoLock(psSwapChain, aPhyAddr);
	omapfb_unlock(fbdev);
}

/*
 * Present frame and synchronize with the display to prevent tearing
 * On DSI panels the sync function is used to handle FRAMEDONE IRQ
 * On DPI panels the wait_for_vsync is used to handle VSYNC IRQ
 * in: psDevInfo
 */
void OMAPLFBPresentSync(OMAPLFB_DEVINFO *psDevInfo,
	OMAPLFB_FLIP_ITEM *psFlipItem)
{
	struct fb_info *framebuffer = psDevInfo->psLINFBInfo;
	struct omapfb_info *ofbi = FB2OFB(framebuffer);
	struct omap_dss_device *display;
	struct omapfb2_device *fbdev = ofbi->fbdev;
	struct omap_dss_driver *driver;
	struct omap_overlay_manager *manager;
	int err = 1;

	omapfb_lock(fbdev);

	display = fb2display(framebuffer);
	/* The framebuffer doesn't have a display attached, just bail out */
	if (!display) {
		omapfb_unlock(fbdev);
		return;
	}

	driver = display->driver;
	manager = display->manager;

	if (driver && driver->sync &&
		driver->get_update_mode(display) == OMAP_DSS_UPDATE_MANUAL) {
		/* Wait first for the DSI bus to be released then update */
		err = driver->sync(display);
		OMAPLFBFlipNoLock(psDevInfo->psSwapChain,
			(unsigned long)psFlipItem->sSysAddr->uiAddr);
	} else if (manager && manager->wait_for_vsync) {
		/*
		 * Update the video pipelines registers then wait until the
		 * frame is shown with a VSYNC
		 */
	// LGE_ChangeS Darren.Kang@lge.com	2011.1.13 For prevent broken image [ST]
		err = manager->wait_for_vsync(manager); 		
		if(psFlipItem->bFlipped == OMAP_FALSE)
			OMAPLFBFlipNoLock(psDevInfo->psSwapChain, (unsigned long)psFlipItem->sSysAddr->uiAddr); 				
	// LGE_ChangeS Darren.Kang@lge.com	2011.1.13 For prevent broken image [END]
//LGE_CHANGES [darren.kang] display lock-up WA patch
		//mdelay(1);		
//LGE_CHANGEE [darren.kang] display lock-up WA patch
	}            

	if (err)
		WARNING_PRINTK("Unable to sync with display %u!",
			psDevInfo->uDeviceID);

	omapfb_unlock(fbdev);
}

#if defined(LDM_PLATFORM)

static volatile OMAP_BOOL bDeviceSuspended;

/*
 * Common suspend driver function
 * in: psSwapChain, aPhyAddr
 */
static void OMAPLFBCommonSuspend(void)
{
	if (bDeviceSuspended)
	{
		DEBUG_PRINTK("Driver is already suspended");
		return;
	}

	OMAPLFBDriverSuspend();
	bDeviceSuspended = OMAP_TRUE;
}

#if 0
/*
 * Function called when the driver is requested to release
 * in: pDevice
 */
static void OMAPLFBDeviceRelease_Entry(struct device unref__ *pDevice)
{
	DEBUG_PRINTK("Requested driver release");
	OMAPLFBCommonSuspend();
}

static struct platform_device omaplfb_device = {
	.name = DEVNAME,
	.id = -1,
	.dev = {
		.release = OMAPLFBDeviceRelease_Entry
	}
};
#endif

#if defined(SGX_EARLYSUSPEND) && defined(CONFIG_HAS_EARLYSUSPEND)

static struct early_suspend omaplfb_early_suspend;

/*
 * Android specific, driver is requested to be suspended
 * in: ea_event
 */
static void OMAPLFBDriverSuspend_Entry(struct early_suspend *ea_event)
{
	DEBUG_PRINTK("Requested driver suspend");
	OMAPLFBCommonSuspend();
}

/*
 * Android specific, driver is requested to be suspended
 * in: ea_event
 */
static void OMAPLFBDriverResume_Entry(struct early_suspend *ea_event)
{
	DEBUG_PRINTK("Requested driver resume");
	OMAPLFBDriverResume();
	bDeviceSuspended = OMAP_FALSE;
}

static struct platform_driver omaplfb_driver = {
	.driver = {
		.name = DRVNAME,
	}
};

#else /* defined(SGX_EARLYSUSPEND) && defined(CONFIG_HAS_EARLYSUSPEND) */

/*
 * Function called when the driver is requested to be suspended
 * in: pDevice, state
 */
static int OMAPLFBDriverSuspend_Entry(struct platform_device unref__ *pDevice,
	pm_message_t unref__ state)
{
	DEBUG_PRINTK("Requested driver suspend");
	OMAPLFBCommonSuspend();
	return 0;
}

/*
 * Function called when the driver is requested to resume
 * in: pDevice
 */
static int OMAPLFBDriverResume_Entry(struct platform_device unref__ *pDevice)
{
	DEBUG_PRINTK("Requested driver resume");
	OMAPLFBDriverResume();
	bDeviceSuspended = OMAP_FALSE;
	return 0;
}

/*
 * Function called when the driver is requested to shutdown
 * in: pDevice
 */
static IMG_VOID OMAPLFBDriverShutdown_Entry(
	struct platform_device unref__ *pDevice)
{
	DEBUG_PRINTK("Requested driver shutdown");
	OMAPLFBCommonSuspend();
}

static struct platform_driver omaplfb_driver = {
	.driver = {
		.name = DRVNAME,
	},
	.suspend = OMAPLFBDriverSuspend_Entry,
	.resume	= OMAPLFBDriverResume_Entry,
	.shutdown = OMAPLFBDriverShutdown_Entry,
};

#endif /* defined(SGX_EARLYSUSPEND) && defined(CONFIG_HAS_EARLYSUSPEND) */

#endif /* defined(LDM_PLATFORM) */

/*
 * Driver init function
 */
static int __init OMAPLFB_Init(void)
{
#if defined(CONFIG_GFX_DMA_COPY) || defined(CONFIG_HDMI_DMA_COPY)
//LGE_CHANGE [taekeun1.kim@lge.com] 2010-12-27, P920 : For GFX dma copy. 
	int i, r;
#endif
	if(OMAPLFBInit() != OMAP_OK)
	{
		WARNING_PRINTK("Driver init failed");
		return -ENODEV;
	}

#if defined(LDM_PLATFORM)
	DEBUG_PRINTK("Registering platform driver");
	if (platform_driver_register(&omaplfb_driver))
	{
		WARNING_PRINTK("Unable to register platform driver");
		if(OMAPLFBDeinit() != OMAP_OK)
			WARNING_PRINTK("Driver cleanup failed\n");
		return -ENODEV;
	}
#if 0
	DEBUG_PRINTK("Registering device driver");
	if (platform_device_register(&omaplfb_device))
	{
		WARNING_PRINTK("Unable to register platform device");
		platform_driver_unregister(&omaplfb_driver);
		if(OMAPLFBDeinit() != OMAP_OK)
			WARNING_PRINTK("Driver cleanup failed\n");
		return -ENODEV;
	}
#endif

#ifdef CONFIG_GFX_DMA_COPY
//LGE_CHANGE_S [taekeun1.kim@lge.com] 2010-12-27, P920 : For GFX dma copy. 
	for(i=0; i<2; i++) init_completion(&gfx_dma[i].compl);
	
	r = omap_request_dma(OMAP_DMA_NO_DEVICE, "GFX DMA odd",
		_omap_gfx_dma_cb_odd, 
		&gfx_dma[0].compl, &gfx_dma[0].lch);
	if(r) {
		printk("GFX DMA odd alloc error\n");
		goto dma_request_end;
	}

        r = omap_request_dma(OMAP_DMA_NO_DEVICE, "GFX DMA even",
		_omap_gfx_dma_cb_even, 
		&gfx_dma[1].compl, &gfx_dma[1].lch);
	if(r) {
		printk("GFX DMA even alloc error\n");
		goto dma_request_end;
	}
dma_request_end:
//LGE_CHANGE_E [taekeun1.kim@lge.com]
#endif

#ifdef CONFIG_HDMI_DMA_COPY
//LGE_CHANGE_S [taekeun1.kim@lge.com] 2011-02-17, P920 : For HDMI dma copy. 
	init_completion(&hdmi_dma.compl);
	hdmi_dma.state = HDMI_DMA_DONE;
	hdmi_dma.frame_pos = 0;
	hdmi_dma.display_queue = 0;

    r = omap_request_dma(OMAP_DMA_NO_DEVICE, "HDMI DMA",
		_omap_hdmi_dma_cb, 
		&hdmi_dma, &hdmi_dma.lch);
	if(r) {
		printk("GFX DMA odd alloc error\n");
		return -ENODEV;
	}

	hdmi_frame_num = 0;
	INIT_WORK(&hdmi_dma_work, hdmi_display_worker);
//LGE_CHANGE_E [taekeun1.kim@lge.com]
#endif


#if defined(SGX_EARLYSUSPEND) && defined(CONFIG_HAS_EARLYSUSPEND)
	omaplfb_early_suspend.suspend = OMAPLFBDriverSuspend_Entry;
        omaplfb_early_suspend.resume = OMAPLFBDriverResume_Entry;
        omaplfb_early_suspend.level = EARLY_SUSPEND_LEVEL_STOP_DRAWING;
        register_early_suspend(&omaplfb_early_suspend);
	DEBUG_PRINTK("Registered early suspend support");
#endif

#endif
	return 0;
}

/*
 * Driver exit function
 */
static IMG_VOID __exit OMAPLFB_Cleanup(IMG_VOID)
{    
#if defined(LDM_PLATFORM)
#if 0
	DEBUG_PRINTK(format,...)("Removing platform device");
	platform_device_unregister(&omaplfb_device);
#endif
	DEBUG_PRINTK("Removing platform driver");
	platform_driver_unregister(&omaplfb_driver);
#if defined(SGX_EARLYSUSPEND) && defined(CONFIG_HAS_EARLYSUSPEND)
        unregister_early_suspend(&omaplfb_early_suspend);
#endif
#endif
	if(OMAPLFBDeinit() != OMAP_OK)
		WARNING_PRINTK("Driver cleanup failed");
}

late_initcall(OMAPLFB_Init);
module_exit(OMAPLFB_Cleanup);

