/*
 * omap_vout.c
 *
 * Copyright (C) 2005-2010 Texas Instruments.
 *
 * This file is licensed under the terms of the GNU General Public License
 * version 2. This program is licensed "as is" without any warranty of any
 * kind, whether express or implied.
 *
 * Leveraged code from the OMAP2 camera driver
 * Video-for-Linux (Version 2) camera capture driver for
 * the OMAP24xx camera controller.
 *
 * Author: Andy Lowe (source@mvista.com)
 *
 * Copyright (C) 2004 MontaVista Software, Inc.
 * Copyright (C) 2010 Texas Instruments.
 *
 * History:
 * 20-APR-2006 Khasim		Modified VRFB based Rotation,
 *				The image data is always read from 0 degree
 *				view and written
 *				to the virtual space of desired rotation angle
 * 4-DEC-2006  Jian		Changed to support better memory management
 *
 * 17-Nov-2008 Hardik		Changed driver to use video_ioctl2
 *
 * 23-Feb-2010 Vaibhav H	Modified to use new DSS2 interface
 *
 * 16-Jan-2010 wonki choi	Re-factoring for LGE Cosmo
 *
 */

#include <linux/init.h>
#include <linux/module.h>
#include <linux/vmalloc.h>
#include <linux/sched.h>
#include <linux/types.h>
#include <linux/platform_device.h>
#include <linux/dma-mapping.h>
#include <linux/irq.h>
#include <linux/videodev2.h>
#include <linux/slab.h>

#include <media/videobuf-dma-contig.h>
#include <media/v4l2-device.h>
#include <media/v4l2-ioctl.h>

#include <plat/dma.h>
#include <plat/vram.h>
#include <plat/vrfb.h>
#include <plat/display.h>
#include <plat/cpu.h>
#ifdef CONFIG_PM
#include <plat/omap-pm.h>
#endif
#include "omap_voutlib.h"
#include "omap_voutdef_cosmo.h"

#ifdef CONFIG_TILER_OMAP
#include <mach/tiler.h>
#endif
MODULE_AUTHOR("Texas Instruments");
MODULE_DESCRIPTION("OMAP Video for Linux Video out driver");
MODULE_LICENSE("GPL");


//extern decl
int omap_dss_wb_apply(struct omap_overlay_manager *mgr, struct omap_writeback *wb);
int dispc_enable_plane(enum omap_plane plane, bool enable);

extern bool hdmi_connected;
/* Driver Configuration macros */
#define VOUT_NAME		"omap_vout"

enum omap_vout_channels {
	OMAP_VIDEO1,
	OMAP_VIDEO2,
	OMAP_VIDEO3,
};

enum dma_channel_state {
	DMA_CHAN_NOT_ALLOTED,
	DMA_CHAN_ALLOTED,
};

#define QQVGA_WIDTH		160
#define QQVGA_HEIGHT		120

/* Max Resolution supported by the driver */
#ifdef CONFIG_ARCH_OMAP4
#define VID_MAX_WIDTH		4096	/* Largest width */
#define VID_MAX_HEIGHT		4096	/* Largest height */
#else
#define VID_MAX_WIDTH		1280	/* Largest width */
#define VID_MAX_HEIGHT		720	/* Largest height */
#endif
/* Mimimum requirement is 2x2 for DSS */
#define VID_MIN_WIDTH		2
#define VID_MIN_HEIGHT		2

/* 2048 x 2048 is max res supported by OMAP display controller */
#define MAX_PIXELS_PER_LINE     2048

#define VRFB_TX_TIMEOUT         1000
#define VRFB_NUM_BUFS		4

/* Max buffer size tobe allocated during init */
#define OMAP_VOUT_MAX_BUF_SIZE (VID_MAX_WIDTH*VID_MAX_HEIGHT*4)

#define VDD2_OCP_FREQ_CONST     (cpu_is_omap34xx() ? \
(cpu_is_omap3630() ? 200000 : 166000) : 0)

#define COSMO_LCD_WIDTH	480

static struct videobuf_queue_ops video_vbq_ops;



/* Variables configurable through module params*/
static u32 video1_numbuffers = 3;
static u32 video2_numbuffers = 3;
static u32 video1_bufsize = OMAP_VOUT_MAX_BUF_SIZE;
static u32 video2_bufsize = OMAP_VOUT_MAX_BUF_SIZE;
static u32 video3_numbuffers = 3;
static u32 video3_bufsize = OMAP_VOUT_MAX_BUF_SIZE;
static u32 vid1_static_vrfb_alloc;
static u32 vid2_static_vrfb_alloc;
static int debug;

/* Module parameters */
module_param(video1_numbuffers, uint, S_IRUGO);
MODULE_PARM_DESC(video1_numbuffers,
	"Number of buffers to be allocated at init time for Video1 device.");

module_param(video2_numbuffers, uint, S_IRUGO);
MODULE_PARM_DESC(video2_numbuffers,
	"Number of buffers to be allocated at init time for Video2 device.");

module_param(video1_bufsize, uint, S_IRUGO);
MODULE_PARM_DESC(video1_bufsize,
	"Size of the buffer to be allocated for video1 device");

module_param(video2_bufsize, uint, S_IRUGO);
MODULE_PARM_DESC(video2_bufsize,
	"Size of the buffer to be allocated for video2 device");

module_param(video3_numbuffers, uint, S_IRUGO);
MODULE_PARM_DESC(video3_numbuffers, "Number of buffers to be allocated at \
		init time for Video3 device.");

module_param(video3_bufsize, uint, S_IRUGO);
MODULE_PARM_DESC(video1_bufsize, "Size of the buffer to be allocated for \
		video3 device");
module_param(vid1_static_vrfb_alloc, bool, S_IRUGO);
MODULE_PARM_DESC(vid1_static_vrfb_alloc,
	"Static allocation of the VRFB buffer for video1 device");

module_param(vid2_static_vrfb_alloc, bool, S_IRUGO);
MODULE_PARM_DESC(vid2_static_vrfb_alloc,
	"Static allocation of the VRFB buffer for video2 device");

module_param(debug, bool, S_IRUGO);
MODULE_PARM_DESC(debug, "Debug level (0-1)");


#if 1 /* baeyoung.park 2011-02-17 */
extern bool IsVideoTelephonyActivated(void);
#endif

static int vidioc_streamoff(struct file *file, void *fh, enum v4l2_buf_type i);

static int	omap_vout_vram_buffer_manager_init		(struct omap_vout_vram_buffer_manager *mgr);
static void	omap_vout_vram_buffer_manager_deinit	(struct omap_vout_vram_buffer_manager *mgr);
static int	omap_vout_vram_buffer_manager_get		(struct omap_vout_vram_buffer_manager *mgr, unsigned long size, struct omap_vout_vram_buffer **buffer);
static int	omap_vout_vram_buffer_manager_put		(struct omap_vout_vram_buffer_manager *mgr, struct omap_vout_vram_buffer *buff);

enum omap_color_mode video_mode_to_dss_mode(struct v4l2_pix_format *pix);


/* list of image formats supported by OMAP2 video pipelines */
const static struct v4l2_fmtdesc omap_formats[] = {
	{
		/* Note:  V4L2 defines RGB565 as:
		 *
		 *      Byte 0                    Byte 1
		 *      g2 g1 g0 r4 r3 r2 r1 r0   b4 b3 b2 b1 b0 g5 g4 g3
		 *
		 * We interpret RGB565 as:
		 *
		 *      Byte 0                    Byte 1
		 *      g2 g1 g0 b4 b3 b2 b1 b0   r4 r3 r2 r1 r0 g5 g4 g3
		 */
		.description = "RGB565, le",
		.pixelformat = V4L2_PIX_FMT_RGB565,
	},
	{
		/* Note:  V4L2 defines RGB32 as: RGB-8-8-8-8  we use
		 *  this for RGB24 unpack mode, the last 8 bits are ignored
		 * */
		.description = "RGB32, le",
		.pixelformat = V4L2_PIX_FMT_RGB32,
	},
	{
		/* Note:  V4L2 defines RGB24 as: RGB-8-8-8  we use
		 *        this for RGB24 packed mode
		 *
		 */
		.description = "RGB24, le",
		.pixelformat = V4L2_PIX_FMT_RGB24,
	},
	{
		.description = "YUYV (YUV 4:2:2), packed",
		.pixelformat = V4L2_PIX_FMT_YUYV,
	},
	{
		.description = "UYVY, packed",
		.pixelformat = V4L2_PIX_FMT_UYVY,
	},
	{
	 .description = "NV12 - YUV420 format",
	 .pixelformat = V4L2_PIX_FMT_NV12,
	},
};

#define NUM_OUTPUT_FORMATS (ARRAY_SIZE(omap_formats))

//#define DEBUG
#ifdef DEBUG
	#define DBG_PRINTK	printk
	#define	INFO_PRINTK printk
#else
	#define DBG_PRINTK(format,...)
#endif
#define ERR_PRINTK printk
#define	INFO_PRINTK printk


//-----------------------------------------------------------------------------------------------------------
//	V4L2 Buffer functions

/*
 * Free buffers
 */
static void omap_vout_free_buffer(unsigned long virtaddr, u32 buf_size)
{
	u32 order, size;
	unsigned long addr = virtaddr;

	size = PAGE_ALIGN(buf_size);
	order = get_order(size);

	while (size > 0) {
		ClearPageReserved(virt_to_page(addr));
		addr += PAGE_SIZE;
		size -= PAGE_SIZE;
	}
	free_pages((unsigned long) virtaddr, order);
}

/*
 * Try format
 */
int omap_vout_try_format(struct v4l2_pix_format *pix)
{
	int ifmt, bpp = 0;

	pix->height = clamp(pix->height, (u32)VID_MIN_HEIGHT,
						(u32)VID_MAX_HEIGHT);
	pix->width = clamp(pix->width, (u32)VID_MIN_WIDTH, (u32)VID_MAX_WIDTH);

	for (ifmt = 0; ifmt < NUM_OUTPUT_FORMATS; ifmt++) {
		if (pix->pixelformat == omap_formats[ifmt].pixelformat)
			break;
	}

	if (ifmt == NUM_OUTPUT_FORMATS)
		ifmt = 0;

	pix->pixelformat = omap_formats[ifmt].pixelformat;
	pix->priv = 0;

	switch (pix->pixelformat) {
	case V4L2_PIX_FMT_YUYV:
	case V4L2_PIX_FMT_UYVY:
	default:
		pix->colorspace = V4L2_COLORSPACE_JPEG;
		bpp = YUYV_BPP;
		break;
	case V4L2_PIX_FMT_RGB565:
	case V4L2_PIX_FMT_RGB565X:
		pix->colorspace = V4L2_COLORSPACE_SRGB;
		bpp = RGB565_BPP;
		break;
	case V4L2_PIX_FMT_RGB24:
		pix->colorspace = V4L2_COLORSPACE_SRGB;
		bpp = RGB24_BPP;
		break;
	case V4L2_PIX_FMT_RGB32:
	case V4L2_PIX_FMT_BGR32:
		pix->colorspace = V4L2_COLORSPACE_SRGB;
		bpp = RGB32_BPP;
		break;
	case V4L2_PIX_FMT_NV12:
		pix->colorspace = V4L2_COLORSPACE_JPEG;
		bpp = 1; /* TODO: check this? */
		break;
	}
	/* :NOTE: NV12 has width bytes per line in both Y and UV sections */
	pix->bytesperline = pix->width * bpp;
	pix->bytesperline = (pix->bytesperline + PAGE_SIZE - 1) &
		~(PAGE_SIZE - 1);
	/* :TODO: add 2-pixel round restrictions to YUYV and NV12 formats */
	pix->sizeimage = pix->bytesperline * pix->height;
	if (V4L2_PIX_FMT_NV12 == pix->pixelformat)
		pix->sizeimage += pix->sizeimage >> 1;
	return bpp;
}

/*
 * Free the V4L2 buffers
 */
static void omap_vout_free_buffers(struct omap_vout_device *vout)
{
	int i, numbuffers;

	/* Allocate memory for the buffers */
	if (OMAP_VIDEO3 == vout->vid) {
		numbuffers = video3_numbuffers;
		vout->buf_info.buffer_size = video3_bufsize;
	} else {
		numbuffers = (vout->vid) ?  video2_numbuffers : video1_numbuffers;
		vout->buf_info.buffer_size = (vout->vid) ? video2_bufsize : video1_bufsize;
	}

	for (i = 0; i < numbuffers; i++) {
		omap_vout_free_buffer(vout->buf_info.buf_virt_addr[i],
					vout->buf_info.buffer_size);
		vout->buf_info.buf_phy_addr[i] = 0;
		vout->buf_info.buf_virt_addr[i] = 0;
	}
}

static void omap_vout_tiler_buffer_free(struct omap_vout_device *vout,
					unsigned int count,
					unsigned int startindex)
{
	int i;

	if (startindex < 0)
		startindex = 0;
	if (startindex + count > VIDEO_MAX_FRAME)
		count = VIDEO_MAX_FRAME - startindex;

	for (i = startindex; i < startindex + count; i++) {
//		printk("Tiler Free (%d) phy_addr(0x%08lx) phy_addr(0x%08lx) uv_addr(0x%08lx) uv_addr_alloc(0x%08lx)\n",
//			i,
//			vout->buf_info.buf_phy_addr[i],
//			vout->buf_info.buf_phy_addr_alloced[i],
//			vout->buf_info.buf_phy_uv_addr[i],
//			vout->buf_info.buf_phy_uv_addr_alloced[i]);
		if (vout->buf_info.buf_phy_addr_alloced[i])
			tiler_free(vout->buf_info.buf_phy_addr_alloced[i]);
		if (vout->buf_info.buf_phy_uv_addr_alloced[i])
			tiler_free(vout->buf_info.buf_phy_uv_addr_alloced[i]);
		vout->buf_info.buf_phy_addr[i] = 0;
		vout->buf_info.buf_phy_addr_alloced[i] = 0;
		vout->buf_info.buf_phy_uv_addr[i] = 0;
		vout->buf_info.buf_phy_uv_addr_alloced[i] = 0;
	}
}

/* Allocate the buffers for  TILER space.  Ideally, the buffers will be ONLY
 in tiler space, with different rotated views available by just a convert.
 */
static int omap_vout_tiler_buffer_setup(struct omap_vout_device *vout,
					unsigned int *count, unsigned int startindex,
					struct v4l2_pix_format *pix)
{
	int i, aligned = 1;
	enum tiler_fmt fmt;

	/* normalize buffers to allocate so we stay within bounds */
	int start = (startindex < 0) ? 0 : startindex;
	int n_alloc = (start + *count > VIDEO_MAX_FRAME)
		? VIDEO_MAX_FRAME - start : *count;
	int bpp = omap_vout_try_format(pix);

	v4l2_dbg(1, debug, &vout->vid_dev->v4l2_dev, "tiler buffer alloc:\n"
		"count - %d, start -%d :\n", *count, startindex);

	/* special allocation scheme for NV12 format */
	if (OMAP_DSS_COLOR_NV12 == vout->input_info.dss_mode) {
		tiler_alloc_packed_nv12(&n_alloc, pix->width,
			pix->height,
			(void **) vout->buf_info.buf_phy_addr + start,
			(void **) vout->buf_info.buf_phy_uv_addr + start,
			(void **) vout->buf_info.buf_phy_addr_alloced + start,
			(void **) vout->buf_info.buf_phy_uv_addr_alloced + start,
			aligned);
	} else {
		unsigned int width = pix->width;
		switch ( vout->input_info.dss_mode ) {
		case OMAP_DSS_COLOR_UYVY:
		case OMAP_DSS_COLOR_YUV2:
			width /= 2;
			bpp = 4;
			break;
		default:
			break;
		}
		/* Only bpp of 1, 2, and 4 is supported by tiler */
		fmt = (bpp == 1 ? TILFMT_8BIT :
			bpp == 2 ? TILFMT_16BIT :
			bpp == 4 ? TILFMT_32BIT : TILFMT_INVALID);
		if (fmt == TILFMT_INVALID)
			return -ENOMEM;

		tiler_alloc_packed(&n_alloc, fmt, width,
			pix->height,
			(void **) vout->buf_info.buf_phy_addr + start,
			(void **) vout->buf_info.buf_phy_addr_alloced + start,
			aligned);
	}

	v4l2_dbg(1, debug, &vout->vid_dev->v4l2_dev,
			"allocated %d buffers\n", n_alloc);

	if (n_alloc < *count) {
		if (n_alloc && (startindex == -1 ||
			V4L2_MEMORY_MMAP != vout->buf_info.memory)) {
			/* TODO: check this condition's logic */
			omap_vout_tiler_buffer_free(vout, n_alloc, start);
			*count = 0;
			return -ENOMEM;
		}
	}

	for (i = start; i < start + n_alloc; i++) {
		v4l2_dbg(1, debug, &vout->vid_dev->v4l2_dev,
				"y=%08lx (%d) uv=%08lx (%d)\n",
				vout->buf_info.buf_phy_addr[i],
				vout->buf_info.buf_phy_addr_alloced[i] ? 1 : 0,
				vout->buf_info.buf_phy_uv_addr[i],
				vout->buf_info.buf_phy_uv_addr_alloced[i] ? 1 : 0);
//		printk("Tiler Setup (%d) phy_addr(0x%08lx) phy_addr(0x%08lx) uv_addr(0x%08lx) uv_addr_alloc(0x%08lx)\n",
//			i,
//			vout->buf_info.buf_phy_addr[i],
//			vout->buf_info.buf_phy_addr_alloced[i],
//			vout->buf_info.buf_phy_uv_addr[i],
//			vout->buf_info.buf_phy_uv_addr_alloced[i]);
		
	}

	*count = n_alloc;

	return 0;
}

/* Free tiler buffers */
static void omap_vout_free_tiler_buffers(struct omap_vout_device *vout)
{
	omap_vout_tiler_buffer_free(vout, vout->buf_info.buffer_allocated, 0);
	vout->buf_info.buffer_allocated = 0;
}

//---------------------------------------------------------------------------------------------------
//	Utility functions


/* Convert V4L2 rotation to DSS rotation
 *	V4L2 understand 0, 90, 180, 270.
 *	Convert to 0, 1, 2 and 3 repsectively for DSS
 */
static int v4l2_rot_to_dss_rot(int v4l2_rotation, enum dss_rotation *rotation)
{
	int ret = 0;

	switch (v4l2_rotation) {
	case 90:
		*rotation = dss_rotation_90_degree;
		break;
	case 180:
		*rotation = dss_rotation_180_degree;
		break;
	case 270:
		*rotation = dss_rotation_270_degree;
		break;
	case 0:
		*rotation = dss_rotation_0_degree;
		break;
	default:
		ret = -EINVAL;
	}
	return ret;
}

static int dss_rot_to_v4l2_rot(enum dss_rotation dss_rot, int *v4l2_rotation)
{
	int ret = 0;

	switch (dss_rot) {
	case dss_rotation_90_degree:
		*v4l2_rotation = 90;
		break;
	case dss_rotation_180_degree:
		*v4l2_rotation = 180;
		break;
	case dss_rotation_270_degree:
		*v4l2_rotation = 270;
		break;
	case dss_rotation_0_degree:
		*v4l2_rotation = 0;
		break;
	default:
		ret = -EINVAL;
	}
	return ret;
}


static bool is_supported_rotation_for_3d(u8 rotation)
{
	if(rotation == OMAP_DSS_ROT_90 || rotation == OMAP_DSS_ROT_270)
		return true;
	else
		return false;
}

/*
 * Convert V4L2 pixel format to DSS pixel format
 */
enum omap_color_mode video_mode_to_dss_mode(struct v4l2_pix_format *pix)
{
	enum omap_color_mode mode;

	switch (pix->pixelformat) {
	case V4L2_PIX_FMT_NV12:
		mode = OMAP_DSS_COLOR_NV12;
		break;
	case 0:
		mode = OMAP_DSS_COLOR_CLUT1;
		break;
	case V4L2_PIX_FMT_YUYV:
		mode = OMAP_DSS_COLOR_YUV2;
		break;
	case V4L2_PIX_FMT_UYVY:
		mode = OMAP_DSS_COLOR_UYVY;
		break;
	case V4L2_PIX_FMT_RGB565:
		mode = OMAP_DSS_COLOR_RGB16;
		break;
	case V4L2_PIX_FMT_RGB24:
		mode = OMAP_DSS_COLOR_RGB24P;
		break;
	case V4L2_PIX_FMT_RGB32:
		mode = OMAP_DSS_COLOR_ARGB32;
		break;
	case V4L2_PIX_FMT_BGR32:
		mode = OMAP_DSS_COLOR_RGBX32;
		break;
	default:
		WARN(1, "Not Support pixel format(%d) in DSS", pix->pixelformat);
		mode = -EINVAL;
	}
	return mode;
}

/**
 * convert v4l2 s3d type to dss s3d type
 */
static enum omap_dss_overlay_s3d_type v4l2_s3d_to_dss_s3d_type(enum v4l2_frame_pack_type type)
{
	switch (type )
	{
	case V4L2_FPACK_OVERUNDER:
		return omap_dss_overlay_s3d_top_bottom;
	case V4L2_FPACK_SIDEBYSIDE:
		return omap_dss_overlay_s3d_side_by_side;
	case V4L2_FPACK_COL_IL:
		return omap_dss_overlay_s3d_interlaced;
	default :
		return omap_dss_overlay_s3d_none;
	}
}

/**
 * convert dss s3d type to v4l2 s3d type
 */
//static enum v4l2_frame_pack_type dss_s3d_to_v4l2_s3d_type(enum omap_dss_overlay_s3d_type type)
//{
//	switch (type )
//	{
//	case omap_dss_overlay_s3d_top_bottom:
//		return V4L2_FPACK_OVERUNDER;
//	case omap_dss_overlay_s3d_side_by_side:
//		return V4L2_FPACK_SIDEBYSIDE;
//	case omap_dss_overlay_s3d_interlaced:
//		return V4L2_FPACK_COL_IL;
//	default :
//		return V4L2_FPACK_NONE;
//	}
//
//}

/**
 * Get address at (x,y) in tiler memory
 * @param paddr, uv_addr : physical addres
 * @param paddr_ret, uv_addr_ret : return address at (x,y)
 */
static int omap_vout_get_offset_tiler(enum omap_color_mode color,
		unsigned long paddr, unsigned long uv_addr,
		int x, int y,
		unsigned long *paddr_ret, unsigned long *uv_addr_ret)
{
	int ps = 1;
	unsigned long	addr_offset = 0, uv_offset = 0;

	//get pixel size
	switch (color)
	{
	case OMAP_DSS_COLOR_CLUT8:
	case OMAP_DSS_COLOR_NV12:
		ps = 1;
		break;
	case OMAP_DSS_COLOR_RGB12U:
	case OMAP_DSS_COLOR_RGB16:
	case OMAP_DSS_COLOR_ARGB16:
	case OMAP_DSS_COLOR_YUV2:
	case OMAP_DSS_COLOR_UYVY:
		ps = 2;
		break;
	case OMAP_DSS_COLOR_RGB24P:
		ps = 3;
		break;
	case OMAP_DSS_COLOR_RGB24U:
	case OMAP_DSS_COLOR_ARGB32:
	case OMAP_DSS_COLOR_RGBA32:
	case OMAP_DSS_COLOR_RGBX32:
		ps = 4;
	default:
		WARN(1, "not supported color format %d\n", color);
		return -EINVAL;
	}

	//get offset
	if ( OMAP_DSS_COLOR_NV12==color )
	{
		unsigned long uv_addr_n;
		addr_offset = tiler_stride(paddr) * y + x;
		uv_addr_n	= tiler_get_natural_addr((void*)uv_addr);
		uv_offset	= tiler_stride(uv_addr_n) * (y>>1) + (x & ~1);
	}
	else
		addr_offset = tiler_stride(paddr) * y + x * ps;
//	DBG_PRINTK("Caculate tiler offset paddr:0x%x, uv_addr:0x%x, (%d,%d) paddr_offset:%d, uv_ofset:%d\n",
//			paddr, uv_addr,
//			x, y,
//			addr_offset, uv_offset);
	if ( paddr_ret )
		*paddr_ret = paddr + addr_offset;
	if ( uv_addr_ret )
		*uv_addr_ret = uv_addr + uv_offset;
	return 0;
}


//-----------------------------------------------------------------------------------------------------
//	Display Functions

/**
 * Disable Overlay
 */
static int omap_vout_display_disable_overlay(struct omap_overlay *ovl)
{
	struct omap_overlay_info ovlInfo;
	if ( ovl==NULL )
		return -EINVAL;
	ovl->get_overlay_info(ovl, &ovlInfo);
	if ( !ovlInfo.enabled )
		return 0;
	ovlInfo.enabled = false;
	if ( ovl->set_overlay_info==NULL || ovl->set_overlay_info(ovl, &ovlInfo) )
	{
		ERR_PRINTK("set_overlay_info() failed\n");
		return -EIO;
	}
	if ( ovl->manager!=NULL )
	{
		int r;
		struct omap_overlay_manager *mgr = ovl->manager;
		if ( mgr->apply)
			if ( (r=ovl->manager->apply(ovl->manager)) )
				return r;
	}
	return 0;
}

static int omap_vout_remap_overlay_manager(struct omap_overlay *ovl,struct omap_overlay_manager *mgr)
{
	struct omap_overlay_info ovlInfo;
	int ret;
	if ( ovl==NULL || mgr==NULL )
		return -EINVAL;
	if ( ovl->manager==mgr )
		return 0;
	ovl->get_overlay_info(ovl, &ovlInfo);
	if ( ovlInfo.enabled )
	{
		ovlInfo.enabled = false;
		if ( ovl->set_overlay_info==NULL || ovl->set_overlay_info(ovl, &ovlInfo) )
		{
			ERR_PRINTK("set_overlay_info() failed\n");
			return -EIO;
		}
	}
	
	if ( ovl->manager!=NULL )
	{
		struct omap_overlay_manager *old_mgr = ovl->manager;
		ret = ovl->unset_manager(ovl);
		if(ret)
		{
			ERR_PRINTK("omap_vout_remap_overlay_manager - unsetmgr failed\n");
			return ret;
		}
		ret = old_mgr->apply(old_mgr);
		if ( ret )
		{
			ERR_PRINTK("omap_vout_remap_overlay_manager - unsetmgr apply failed\n");
			return ret;
		}
	}
	
	ret = ovl->set_manager(ovl,mgr);	
	if(ret)
	{
		ERR_PRINTK("omap_vout_remap_overlay_manager - set_manager failed\n");
		return ret;
	}
	ret = mgr->apply(mgr);
	if ( ret )
	{
		ERR_PRINTK("omap_vout_remap_overlay_manager - set_manager apply failed\n");
		return ret;
	}

	return 0;
}

static void omap_vout_clear_overlay_wb(struct omap_overlay *ovl)
{
	struct omap_overlay_info ovlInfo;
	ovl->get_overlay_info(ovl, &ovlInfo);
	if ( ovlInfo.out_wb == true)
	{	
		struct omap_writeback *wb;

		ovlInfo.enabled = false;
		ovlInfo.out_wb = false;
		ovl->set_overlay_info(ovl, &ovlInfo);

		wb = omap_dss_get_wb(0);		
		
		if ( wb!=NULL )
		{
			mutex_lock(&wb->lock);	
			wb->enabled = false;
			wb->info.enabled = false;
			wb->info.source = 0;
			DBG_PRINTK("Disabling Write Back Overlay%d\n",ovl->id);
			if ( omap_dss_wb_apply(ovl->manager, wb) )
				ERR_PRINTK("WriteBack apply(to disable) failed\n");
			mutex_unlock(&wb->lock);
		}
		else
			ERR_PRINTK("Write back enabled in overlay info, but WB get failed\n");
	}
}

/**
 * Set Alpha & transparent color
 */
static int omap_vout_display_set_alpha_and_trans(struct omap_vout_device *vout, struct omap_overlay_manager *mgr)
{
	bool info_changed = false;
	u32 flags;
	u32 trans_key;
	struct omap_overlay_manager_info info;
	if ( vout==NULL || mgr==NULL )
		return -EINVAL;

	mgr->get_manager_info(mgr, &info);

	mutex_lock(&vout->lock);
	flags = vout->buf_info.fbuf.flags;
	trans_key = vout->display_info.lcd.win.chromakey;
	mutex_unlock(&vout->lock);

	//transparent color
	if ( flags & (V4L2_FBUF_FLAG_SRC_CHROMAKEY|V4L2_FBUF_FLAG_CHROMAKEY) )
	{
		//trans key is set
		if ( !info.trans_enabled )
		{
			info.trans_enabled = true;
			info_changed = true;
		}

		//src type
		if ( (flags & V4L2_FBUF_FLAG_SRC_CHROMAKEY) && (info.trans_key_type!=OMAP_DSS_COLOR_KEY_VID_SRC) )
		{
			info.trans_key_type = OMAP_DSS_COLOR_KEY_VID_SRC;
			info_changed =true;
		}

		//dst type
		if ( (flags & V4L2_FBUF_FLAG_CHROMAKEY) && (info.trans_key_type!=OMAP_DSS_COLOR_KEY_GFX_DST) )
		{
			info.trans_key_type = OMAP_DSS_COLOR_KEY_GFX_DST;
			info_changed =true;
		}

		//trans color
		if ( trans_key!=info.trans_key)
		{
			info.trans_key = trans_key;
			info_changed = true;
		}
	}
	else
	{
		if ( info.trans_enabled )
		{
			info.trans_enabled = false;
			info_changed = true;
		}
	}

	if ( (flags&V4L2_FBUF_FLAG_LOCAL_ALPHA) && !info.alpha_enabled )
	{
		info.alpha_enabled = true;
		info_changed = true;
	}
	if ( !(flags&V4L2_FBUF_FLAG_LOCAL_ALPHA) && info.alpha_enabled )
	{
		info.alpha_enabled = false;
		info_changed = true;
	}

	if ( info_changed )
	{
//		DBG_PRINTK
		printk("VOUT Trans & Alpha value changed alpah enalbed(%d), trans_enabled(%d), trans_type(%d), transk_color(%u)\n",
				info.alpha_enabled, info.trans_enabled, info.trans_key_type, info.trans_key);
		mgr->set_manager_info(mgr, &info);
	}

	return 0;
}

/**
 * set left & right overlay of HDMI
 */
static int omap_vout_display_to_overlay_s3d_hdmi(struct omap_overlay_manager *mgr,
		struct omap_overlay *left_ovl,
		struct omap_overlay *right_ovl,
		struct videobuf_buffer* frame,
		u16 pos_x, u16 pos_y,
		u16 out_width, u16 out_height,
		enum omap_dss_overlay_s3d_type s3d_type,
		u16 hdmi_res_x, u16 hdmi_res_y,
		bool bWaitChange)
{
	u16 pos_x_L, pos_y_L;
	u16 pos_x_R, pos_y_R;
	u16 out_width_L, out_height_L;
	u16 out_width_R, out_height_R;
	unsigned long paddr_L, p_uv_addr_L;
	unsigned long paddr_R, p_uv_addr_R;
	u16	in_width, in_height;
	struct omap_vout_frame_info *fInfo;
	struct omap_overlay_info ovlInfo;
	int r = 0;

	if ( mgr==NULL || left_ovl==NULL || right_ovl==NULL || frame==NULL )
		return -EINVAL;
	if ( left_ovl->get_overlay_info==NULL || left_ovl->set_overlay_info==NULL )
		return -EINVAL;
	if ( right_ovl->get_overlay_info==NULL || right_ovl->set_overlay_info==NULL )
		return -EINVAL;
	if ( mgr->apply==NULL )
		return -EINVAL;

	fInfo = (struct omap_vout_frame_info*)frame->priv;
	if ( fInfo==NULL )
	{
		ERR_PRINTK("Video buffer doesn't have frame info\n");
		WARN(1, "No omap_vout_frame_info\n");
		return -EINVAL;
	}

	if ( s3d_type == omap_dss_overlay_s3d_top_bottom )
	{

		//get position
		out_width_L = out_width_R = out_width;
		pos_x_L = pos_x_R = pos_x;
		if ( pos_y >= (hdmi_res_y/2) )
		{
			ERR_PRINTK("%d exceed top half of HDMI(%dx%d)\n", pos_y, hdmi_res_x, hdmi_res_y);
			return -EINVAL;
		}
		pos_y_L = pos_y / 2;
		out_height_L = out_height / 2;
		if ( (pos_y_L + out_height_L) > (hdmi_res_y/2) )
			out_height_L = (hdmi_res_y/2) - pos_y_L;
		pos_y_R = pos_y_L + (hdmi_res_y / 2);
		out_height_R = out_height_L;

		in_width = fInfo->crop.width;
		in_height = (fInfo->crop.height/4)*2;

		//get offset
		paddr_L = fInfo->pAddr;
		p_uv_addr_L = fInfo->pUvAddr;
		r = omap_vout_get_offset_tiler(fInfo->color, fInfo->pAddr, fInfo->pUvAddr, 0, fInfo->crop.height/2,
				&paddr_R, &p_uv_addr_R);
		if ( r )
		{
			ERR_PRINTK("Get offset of right frame failed\n");
			return -EINVAL;
		}
	}
	else if ( s3d_type == omap_dss_overlay_s3d_side_by_side )
	{
		//get position
		out_height_L = out_height_R = out_height;
		pos_y_L = pos_y_R = pos_y;
		if ( pos_x >= (hdmi_res_x/2) )
		{
			ERR_PRINTK("%d exceed left half of HDMI(%dx%d)\n", pos_x, hdmi_res_x, hdmi_res_y);
			return -EINVAL;
		}
		pos_x_L = pos_x / 2;
		out_width_L = out_width / 2;
		if ( (pos_x_L + out_width_L) > (hdmi_res_x/2) )
			out_width_L = (hdmi_res_x/2) - pos_x_L;
		pos_x_R = pos_x_L + (hdmi_res_x / 2);
		out_width_R = out_width_L;

		in_width = (fInfo->crop.width / 4)*2;
		in_height = fInfo->crop.height;

		//get offset
		paddr_L = fInfo->pAddr;
		p_uv_addr_L = fInfo->pUvAddr;
		r = omap_vout_get_offset_tiler(fInfo->color, fInfo->pAddr, fInfo->pUvAddr, fInfo->crop.width/2, 0,
				&paddr_R, &p_uv_addr_R);
		if ( r )
		{
			ERR_PRINTK("Get offset of right frame failed\n");
			return -EINVAL;
		}
	}
	else
	{
		ERR_PRINTK("Not supporting s3d type %d\n", s3d_type);
		return -EINVAL;
	}

	DBG_PRINTK("HDMI Resolution (%dx%d)\n", hdmi_res_x, hdmi_res_y);

	//set Left overlay
	left_ovl->get_overlay_info(left_ovl, &ovlInfo);
	ovlInfo.enabled			= true;
	ovlInfo.paddr			= paddr_L;
	ovlInfo.p_uv_addr		= p_uv_addr_L;
	ovlInfo.vaddr			= NULL;
	ovlInfo.color_mode		= fInfo->color;
	ovlInfo.rotation		= OMAP_DSS_ROT_0;
	ovlInfo.rotation_type	= OMAP_DSS_ROT_TILER;
	ovlInfo.width			= in_width;
	ovlInfo.height			= in_height;
	ovlInfo.screen_width	= fInfo->pix.width;
	ovlInfo.mirror			= 0;
	ovlInfo.pos_x			= pos_x_L;
	ovlInfo.pos_y			= pos_y_L;
	ovlInfo.out_width		= out_width_L;
	ovlInfo.out_height		= out_height_L;
	ovlInfo.zorder			= 1;				//left (LCD Video) 1
	ovlInfo.s3d_type		= s3d_type;
	ovlInfo.out_wb			= false;
	DBG_PRINTK("Left Frame \n");
	DBG_PRINTK("\tinput : %dx%d\n", ovlInfo.width, ovlInfo.height);
	DBG_PRINTK("\tout : (%d,%d), (%dx%d)\n", ovlInfo.pos_x, ovlInfo.pos_y, ovlInfo.out_width, ovlInfo.out_height);
	r = left_ovl->set_overlay_info(left_ovl, &ovlInfo);
	if ( r )
	{
		ERR_PRINTK("Left Frame set_overlay_info() failed\n");
		return r;
	}

	//set Right overlay
	right_ovl->get_overlay_info(right_ovl, &ovlInfo);
	ovlInfo.enabled			= true;
	ovlInfo.paddr			= paddr_R;
	ovlInfo.p_uv_addr		= p_uv_addr_R;
	ovlInfo.vaddr			= NULL;
	ovlInfo.color_mode		= fInfo->color;
	ovlInfo.rotation		= OMAP_DSS_ROT_0;
	ovlInfo.rotation_type	= OMAP_DSS_ROT_TILER;
	ovlInfo.width			= in_width;
	ovlInfo.height			= in_height;
	ovlInfo.screen_width	= fInfo->pix.width;
	ovlInfo.mirror			= 0;
	ovlInfo.pos_x			= pos_x_R;
	ovlInfo.pos_y			= pos_y_R;
	ovlInfo.out_width		= out_width_R;
	ovlInfo.out_height		= out_height_R;
	ovlInfo.zorder			= 2;		//right (HDMI video) 2
	ovlInfo.s3d_type		= s3d_type;
	ovlInfo.out_wb			= false;
	DBG_PRINTK("Right Frame \n");
	DBG_PRINTK("\tinput : %dx%d\n", ovlInfo.width, ovlInfo.height);
	DBG_PRINTK("\tout : (%d,%d), (%dx%d)\n", ovlInfo.pos_x, ovlInfo.pos_y, ovlInfo.out_width, ovlInfo.out_height);
	r = right_ovl->set_overlay_info(right_ovl, &ovlInfo);
	if ( r )
	{
		ERR_PRINTK("Right Frame set_overlay_info() failed\n");
		return r;
	}

	//apply to manager
	r = mgr->apply(mgr);
	if ( r )
	{
		ERR_PRINTK("HDMI Manager apply failed\n");
		return r;
	}

	if ( bWaitChange && mgr->wait_for_vsync )
		mgr->wait_for_vsync(mgr);

	return 0;
}

/**
 * set overlay info with frame
 * @param ovl : overlat to set
 * @param frame : frame to display
 * @param forceEnable : Enable layer
 * @param s3d_type : s3d type
 * @param bWaitChnage : wait Vsync (changed setting apply)
 */
//@todo color mode change
static int omap_vout_display_to_overlay_2d(struct omap_overlay *ovl, struct videobuf_buffer* frame,
		bool forceEnable,
		u8 rotation,
		u16 pos_x, u16 pos_y,
		u16 out_width, u16 out_height,
		bool bWaitChange)
{
	struct omap_vout_frame_info *fInfo;
	struct omap_overlay_info ovlInfo;
	if ( ovl==NULL || frame==NULL )
	{
		WARN(1, "Invalid paramter (%p, %p)\n,", ovl, frame);
		return -EINVAL;
	}
	fInfo = (struct omap_vout_frame_info*)frame->priv;
	if ( fInfo==NULL )
	{
		ERR_PRINTK("Video buffer doesn't have frame info\n");
		WARN(1, "No omap_vout_frame_info\n");
		return -EINVAL;
	}

	if ( ovl->get_overlay_info==NULL )
	{
		ERR_PRINTK("WoW get_overlay_info() not exist\n");
		return -EINVAL;
	}
	ovl->get_overlay_info(ovl, &ovlInfo);

	if ( forceEnable )
		ovlInfo.enabled = true;
	else
	{
		if ( !ovlInfo.enabled )
		{
			DBG_PRINTK("Overlay %d is not enabled\n", ovl->id);
			return 0;
		}
	}

	ovlInfo.paddr	= fInfo->pAddr;
	ovlInfo.p_uv_addr	= fInfo->pUvAddr;
	ovlInfo.vaddr	= NULL;
	ovlInfo.color_mode	= fInfo->color;

	ovlInfo.rotation	= rotation;
	ovlInfo.rotation_type	= OMAP_DSS_ROT_TILER;	//always tiler..
	if ( rotation==OMAP_DSS_ROT_90 || rotation==OMAP_DSS_ROT_270 )
	{
		ovlInfo.width	= fInfo->crop.height;
		ovlInfo.height	= fInfo->crop.width;
		ovlInfo.screen_width = fInfo->pix.height;
	}
	else
	{
		ovlInfo.width	= fInfo->crop.width;
		ovlInfo.height	= fInfo->crop.height;
		ovlInfo.screen_width = fInfo->pix.width;
	}
	ovlInfo.mirror	= 0;
	ovlInfo.pos_x	= pos_x;
	ovlInfo.pos_y	= pos_y;

	ovlInfo.out_width	= out_width;
	ovlInfo.out_height	= out_height;

	ovlInfo.zorder		= 1;	//One Video z-order 1

	ovlInfo.s3d_type	= omap_dss_overlay_s3d_none;
	ovlInfo.out_wb	= false;

	frame->state = VIDEOBUF_ACTIVE;

	if ( ovl->set_overlay_info==NULL )
		return -EINVAL;
	if ( ovl->manager==NULL )
		return -EINVAL;
	if ( ovl->manager->apply==NULL )
		return -EINVAL;
	
	if(out_width < ovlInfo.width/4 ||
		out_height < ovlInfo.height/4)
	{
		WARN(1,"2d downscaling exceed\n");
		return -EINVAL;
	}	

	//set overlay
	if ( ovl->set_overlay_info(ovl, &ovlInfo) )
	{
		ERR_PRINTK("set_overlay_info() failed\n");
		return -EIO;
	}
	if ( ovl->manager!=NULL )
	{
		struct omap_overlay_manager *mgr = ovl->manager;

		if ( mgr->apply)
		{
			if ( ovl->manager->apply(ovl->manager) )
				return -EIO;
		}
		if ( bWaitChange && mgr->wait_for_vsync )
			mgr->wait_for_vsync(mgr);
	}
	return 0;
}

/**
 * Prepare WB buffer for S3D LCD out
 */
static int omap_vout_display_lcd_prepare_wb_buffer(struct omap_vout_device *vout)
{
	int needed_size;
	struct v4l2_window lcd_win;
	struct omap_vout_vram_buffer *buffer;
	int i;

	if ( vout==NULL )
		return -EINVAL;
	mutex_lock(&vout->lock);
	lcd_win = vout->display_info.lcd.win;
	buffer = vout->display_info.lcd.wb_buffer[0];
	mutex_unlock(&vout->lock);

	needed_size = lcd_win.w.width * lcd_win.w.height * 2;	///UYVY YUV422
	if ( buffer!=NULL )
	{
		if ( needed_size <= buffer->alloc_size )
			return 0;		//re-use
	}

//	DBG_PRINTK("In preparing WB Buffer w:%d, h:%d, needed buffer size:%d\n",
//			lcd_win.w.width, lcd_win.w.height, needed_size);
	//get new buffer
	mutex_lock(&vout->lock);
	for(i=0;i<2;i++)
	{
		if ( vout->display_info.lcd.wb_buffer[i]!=NULL )
			omap_vout_vram_buffer_manager_put(&vout->buffer_manager, vout->display_info.lcd.wb_buffer[i]);
		if ( omap_vout_vram_buffer_manager_get(&vout->buffer_manager,
				needed_size,
				&vout->display_info.lcd.wb_buffer[i]) )
		{
			ERR_PRINTK("alloc WB buffer failed\n");
			mutex_unlock(&vout->lock);
			return -ENOMEM;
		}
//		DBG_PRINTK("\t[%d] alloc success. paddr:0x%lx vaddr:0x%p\n", i,
//				vout->display_info.lcd.wb_buffer[i]->paddr,
//				vout->display_info.lcd.wb_buffer[i]->vaddr);
	}


	if(vout->display_info.lcd.wb_pre_buffer != NULL)
		omap_vout_vram_buffer_manager_put(&vout->buffer_manager, vout->display_info.lcd.wb_pre_buffer);
	
	if ( omap_vout_vram_buffer_manager_get(&vout->buffer_manager,
				needed_size,
				&vout->display_info.lcd.wb_pre_buffer) )
	{
		ERR_PRINTK("alloc WB pre buffer failed\n");
		mutex_unlock(&vout->lock);
		return -ENOMEM;
	}

	mutex_unlock(&vout->lock);

	return 0;
}

/**
 * Configure WB's input layer
 */
static int omap_vout_display_lcd_wb_configure_input(struct omap_vout_frame_info *fInfo,
		struct omap_overlay_info *ovlInfo,
		int pos_x, int pos_y, int w, int h,
		enum omap_dss_rotation_angle rotation,
		int dst_x, int dst_y,
		int dst_w, int dst_h)
{
	unsigned long	paddr, uv_addr;
	int 	r;
	if ( fInfo==NULL || ovlInfo==NULL )
		return -EINVAL;

	//caculate offset
	r = omap_vout_get_offset_tiler(fInfo->color, fInfo->pAddr, fInfo->pUvAddr, pos_x, pos_y, &paddr, &uv_addr);
	if ( r )
		return r;
	ovlInfo->enabled	= true;
	ovlInfo->paddr		= paddr;
	ovlInfo->p_uv_addr 	= uv_addr;
	ovlInfo->vaddr		= NULL;
	ovlInfo->rotation	= rotation;
	ovlInfo->rotation_type	= OMAP_DSS_ROT_TILER;
	if ( rotation==OMAP_DSS_ROT_90 || rotation==OMAP_DSS_ROT_270 )
	{
		ovlInfo->width	= h;
		ovlInfo->height	= w;
	}
	else
	{
		ovlInfo->width	= w;
		ovlInfo->height	= h;
	}
	ovlInfo->color_mode	= fInfo->color;
	ovlInfo->pos_x		= dst_x;
	ovlInfo->pos_y		= dst_y;
	ovlInfo->out_width	= dst_w;
	ovlInfo->out_height	= dst_h;
	ovlInfo->out_wb		= true;

//	DBG_PRINTK("In congiguring WB input\n");
//	DBG_PRINTK("\tpaddr:0x%lx, uv_addr:0x%lx, pos_x:%d, pos_y:%d, out paddr:0x%lx, out uv_addr:0x%lx\n",
//			fInfo->pAddr, fInfo->pUvAddr, pos_x, pos_y, paddr, uv_addr);
//	DBG_PRINTK("\torg w:%d, h:%d\n", w, h);
//	DBG_PRINTK("\trotatio :%d\n", rotation);
//	DBG_PRINTK("\tintput_width: %d, input_height:%d\n", ovlInfo->width, ovlInfo->height);
//	DBG_PRINTK("\tcolor : 0x%x\n", ovlInfo->color_mode);
//	DBG_PRINTK("\toutput x:%d, y:%d, w:%d, h:%d\n", ovlInfo->pos_x, ovlInfo->pos_y, ovlInfo->out_width, ovlInfo->out_height);
	return 0;
}

/**
 * Configure Write back
 */
static int omap_vout_display_lcd_wb_configure_out(struct omap_writeback_info *wbInfo,
		unsigned long paddr,
		int overlay_id,
		struct omap_overlay_info *ovlInfo)
{
	if ( wbInfo==NULL || ovlInfo==NULL )
		return -EINVAL;
	wbInfo->enabled		= true;
	wbInfo->capturemode	= OMAP_WB_CAPTURE_ALL;
	wbInfo->dss_mode	= OMAP_DSS_COLOR_UYVY;
	wbInfo->width		= ovlInfo->out_width;
	wbInfo->height		= ovlInfo->out_height;
	wbInfo->out_width	= ovlInfo->out_width;
	wbInfo->out_height	= ovlInfo->out_height;
	wbInfo->source		= overlay_id + 3;
	wbInfo->source_type	= OMAP_WB_SOURCE_OVERLAY;
	wbInfo->paddr		= paddr;
	wbInfo->line_skip	= false;

//	DBG_PRINTK("In congiguring WB \n");
//	DBG_PRINTK("\tpaddr:0x%lx\n", wbInfo->paddr);
//	DBG_PRINTK("\tcolor:0x%x\n", wbInfo->dss_mode);
//	DBG_PRINTK("\tsource overlay id:%d\n", wbInfo->source);
//	DBG_PRINTK("\tw:%lu h:%lu\n", wbInfo->width, wbInfo->height);
//	DBG_PRINTK("\tout w:%lu, h:%lu\n", wbInfo->out_width, wbInfo->out_height);
	return 0;
}

/**
 * configure wb and get result
 */
static int omap_vout_display_lcd_wb(struct videobuf_buffer* frame,
		struct omap_writeback *wb,
		struct omap_vout_vram_buffer *out_buffer,
		struct omap_overlay *wb_input,
		int src_x, int src_y, int src_w, int src_h,
		enum omap_dss_rotation_angle rotation,
		int dst_x, int dst_y, int dst_w, int dst_h)
{
	int r;

	if ( frame==NULL || wb==NULL || out_buffer==NULL || wb_input==NULL )
		return -EINVAL;
	//configure write back
	{
		struct omap_vout_frame_info *fInfo;
		struct omap_writeback_info	wbInfo;
		struct omap_overlay_info	ovlInfo;
		//get needed information

		fInfo = (struct omap_vout_frame_info*)frame->priv;
		if ( fInfo==NULL )
			return -EINVAL;
		if ( wb_input->get_overlay_info==NULL )
			return -EINVAL;
		wb_input->get_overlay_info(wb_input, &ovlInfo);

		//configure input layer
		r = omap_vout_display_lcd_wb_configure_input(fInfo,
				&ovlInfo,
				src_x, src_y, src_w, src_h,
				rotation,
				dst_x, dst_y, dst_w, dst_h);
		if ( r )
			return r;
		//configure write back
		if ( wb->get_wb_info==NULL )
			return -EINVAL;
		wb->get_wb_info(wb, &wbInfo);
		r = omap_vout_display_lcd_wb_configure_out(&wbInfo,
				out_buffer->paddr,
				wb_input->id, &ovlInfo);
		if ( r )
			return r;

		//set info
		if ( wb_input->set_overlay_info==NULL )
			return -EINVAL;
		r = wb_input->set_overlay_info(wb_input, &ovlInfo);
		if ( r )
			return r;
		if ( wb->set_wb_info==NULL )
			return -EINVAL;
		r = wb->set_wb_info(wb, &wbInfo);
		if ( r )
			return r;
	}
	return 0;
}

static struct omap_vout_vram_buffer* omap_vout_display_lcd_get_ping_pong_wb_buffer(struct omap_vout_device *vout)
{
	struct omap_vout_vram_buffer *ret=NULL;
	if ( vout==NULL )
		return NULL;
	mutex_lock(&vout->lock);
	ret = vout->display_info.lcd.wb_buffer[vout->display_info.lcd.wb_buffer_cursor];
	vout->display_info.lcd.wb_buffer_cursor = (vout->display_info.lcd.wb_buffer_cursor+1) % 2;
	mutex_unlock(&vout->lock);
	return ret;
}

/**
 * ISR for write back done
 * @param : mutex
 */
static void omap_vout_display_lcd_wb_done(void *arg, u32 irqsatus)
{
	struct completion *wb_done = (struct completion*)arg;
	omap_dispc_unregister_isr(omap_vout_display_lcd_wb_done, arg, DISPC_IRQ_FRAMEDONE_WB);
	if ( wb_done == NULL )
		return;
	complete(wb_done);
}

static void omap_vout_display_lcd_wb_interleave(struct omap_vout_device *vout,
												unsigned char* dst_vaddr , unsigned char* src_vaddr,
												u16 width, u16 height)
{
	enum v4l2_frame_pack_type	s3d_type;
	int depth_factor;
	int stride;
	unsigned char *odd_src,*even_src, *dst;
	u16 bpp = 2;
	int i;
	int even_margin;
	int odd_margin;
	u16 row_inc;


	depth_factor = width/2 - 1 > vout->display_info.depth_factor ? vout->display_info.depth_factor : width/2 - 1;	
	s3d_type		= vout->display_info.s3d_type;
	

	stride = width*bpp;		
	dst = dst_vaddr;	

	if(stride > COSMO_LCD_WIDTH*bpp)
		return;
	

	//  set src addr by pack order and s3d type
	if(vout->display_info.s3d_pack_order == V4L2_FPACK_ORDER_LF)
	{
		if(s3d_type == V4L2_FPACK_OVERUNDER)
		{
			even_src = 	vout->display_info.lcd.wb_pre_buffer->vaddr;		
			odd_src = vout->display_info.lcd.wb_pre_buffer->vaddr + stride;
		}
		else  //side by side
		{
			odd_src = vout->display_info.lcd.wb_pre_buffer->vaddr;
			even_src = vout->display_info.lcd.wb_pre_buffer->vaddr + (stride*height/2);
		}

		
	}
	else 
	{		
		if(s3d_type == V4L2_FPACK_OVERUNDER)
		{
			even_src = 	vout->display_info.lcd.wb_pre_buffer->vaddr +stride;
			odd_src = vout->display_info.lcd.wb_pre_buffer->vaddr;
		}
		else //side by side
		{
			odd_src = vout->display_info.lcd.wb_pre_buffer->vaddr+ (stride*height/2);
			even_src = vout->display_info.lcd.wb_pre_buffer->vaddr ;		
		}

		
	}

	// set row increasment by s3d type
	if(s3d_type == V4L2_FPACK_OVERUNDER)
		row_inc = 2*stride;		
	else // side by side
		row_inc = stride;

	// src addr shifting by depth
	if(depth_factor <= 0)
	{
		even_src += row_inc*(depth_factor*-1);
		even_margin = height/2+depth_factor;
		odd_margin = -1*depth_factor-1;
		
	}
	else
	{
		odd_src += row_inc*depth_factor;
		even_margin = depth_factor-1;
		odd_margin = height/2-depth_factor;		
	}

	// buffer copy
	if(depth_factor <= 0)
	{
		for(i=0;i<height/2;i++)
		{
			if(i<even_margin )
			{
				memcpy(dst,even_src,stride);
				even_src += row_inc;
			}
			else
			{
				memcpy(dst,vout->display_info.lcd.black_line,stride);
			}
			
			dst += stride;
			
			if(i>odd_margin)
			{
				memcpy(dst,odd_src,stride);
				odd_src += row_inc;
			}
			else
			{
				memcpy(dst,vout->display_info.lcd.black_line,stride);
			}
			
			dst += stride;			
		}
	}
	else if(depth_factor > 0)
	{
		for(i=0;i<height/2;i++)
		{
			if(i>even_margin )
			{
				memcpy(dst,even_src,stride);
				even_src += row_inc;
			}
			else
			{
				memcpy(dst,vout->display_info.lcd.black_line,stride);
			}
			
			dst += stride;
			
			if(i<odd_margin)
			{
				memcpy(dst,odd_src,stride);
				odd_src += row_inc;
			}
			else
			{
				memcpy(dst,vout->display_info.lcd.black_line,stride);
			}
			
			dst += stride;			
		}
	}
}
												

/**
 * Top-Bottom format out to LCD
 */
static int omap_vout_display_lcd_top_bottom_to_interlace(struct omap_vout_device *vout,
		struct videobuf_buffer *frame,
		struct omap_overlay *wb_input,
		struct omap_vout_vram_buffer *wb_buffer,
		u16 src_w, u16 src_h,
		u16 out_width, u16 out_height)
{
	struct omap_writeback *wb;
	int r;
	u8 rotation;

	if ( vout==NULL || wb_input==NULL || wb_buffer==NULL )
		return -EINVAL;

	wb = omap_dss_get_wb(0);
	if ( wb==NULL )
	{
		WARN(1, "get write back failed\n");
		return -EINVAL;
	}

	rotation = OMAP_DSS_ROT_90;
	mutex_lock(&vout->lock);
	if(vout->display_info.lcd.rotation == OMAP_DSS_ROT_270)
		rotation = OMAP_DSS_ROT_270;
	mutex_unlock(&vout->lock);

	if(out_width*2 < src_h/4 ||
		out_height/2 < src_w/5)
	{
		
		WARN(1, "tb downscaling exceed skip frame\n");
		return -EINVAL;
	}

	//configure wb
	//    numric=>Left, Alphabet=>Right
	//	(TB)  => rotate 90 in double width => in org width
	//   0123    EA40                          EA
	//   4567 => FB51                          40
	//   ABCD    GC61                          FB
	//   EFGH    HD73                          51
	//                                         GC
	//                                         61
	//                                         HD
	//                                         73

	r = omap_vout_display_lcd_wb(frame,
			wb,
			vout->display_info.lcd.wb_pre_buffer,
			wb_input,
			0, 0, src_w, src_h,
			rotation,
			0, 0,
			out_width*2, out_height/2);
	if ( r )
	{
		WARN(1, "Configuring WB for Side-by-Side failed\n");
		return r;
	}
	//apply
	if ( wb_input->manager==NULL )
	{
		WARN(1, "Manager not exist\n");
		return -EINVAL;
	}

	//register WB done isr & wait WB done
	{
		struct completion wb_done_completion;
		init_completion(&wb_done_completion);	//completion will be used for waiting write back done
		r = omap_dispc_register_isr(omap_vout_display_lcd_wb_done, &wb_done_completion, DISPC_IRQ_FRAMEDONE_WB);
		if ( r )
		{
			WARN(1, "Register write back done isr failed\n");
			return r;
		}
		r = omap_dss_wb_apply(wb_input->manager, wb);
		if ( r )
		{
			WARN(1, "Write back apply failed\n");
			return r;
		}
		//wait done
//		DBG_PRINTK("Waiting WB Done\n");
		wait_for_completion(&wb_done_completion);
//		DBG_PRINTK("WB Done and go ahead\n");

		omap_vout_display_lcd_wb_interleave(vout,
											wb_buffer->vaddr,vout->display_info.lcd.wb_pre_buffer->vaddr,
											out_width,out_height);
	}
	return 0;
}

/**
 * Side-Side format out to LCD
 */
static int omap_vout_display_lcd_side_side_to_interlace(struct omap_vout_device *vout,
		struct videobuf_buffer *frame,
		struct omap_overlay *wb_input,
		struct omap_vout_vram_buffer *wb_buffer,
		u16 src_w, u16 src_h,
		u16 out_width, u16 out_height)
{
	struct omap_writeback *wb;
	int r;
	u8 rotation;

	if ( vout==NULL || wb_input==NULL || wb_buffer==NULL )
		return -EINVAL;

	wb = omap_dss_get_wb(0);
	if ( wb==NULL )
	{
		WARN(1, "get write back failed\n");
		return -EINVAL;
	}

	rotation = OMAP_DSS_ROT_90;
	mutex_lock(&vout->lock);
	if(vout->display_info.lcd.rotation == OMAP_DSS_ROT_270)
		rotation = OMAP_DSS_ROT_270;
	mutex_unlock(&vout->lock);

	if(out_width < src_h/4 ||
		out_height < src_w/4)
	{
		
		WARN(1, "ss downscaling exceed skip frame\n");
		return -EINVAL;
	}

	r = omap_vout_display_lcd_wb(frame,
			wb,
			vout->display_info.lcd.wb_pre_buffer,
			wb_input,
			0, 0, src_w, src_h,
			rotation,
			0, 0,
			out_width, out_height);
	if ( r )
	{
		WARN(1, "Configuring WB for Side-by-Side failed\n");
		return r;
	}
	//apply
	if ( wb_input->manager==NULL )
	{
		WARN(1, "Manager not exist\n");
		return -EINVAL;
	}

	//register WB done isr & wait WB done
	{
		struct completion wb_done_completion;
		
		init_completion(&wb_done_completion);	//completion will be used for waiting write back done
		r = omap_dispc_register_isr(omap_vout_display_lcd_wb_done, &wb_done_completion, DISPC_IRQ_FRAMEDONE_WB);
		if ( r )
		{
			WARN(1, "Register write back done isr failed\n");
			return r;
		}
		r = omap_dss_wb_apply(wb_input->manager, wb);
		if ( r )
		{
			WARN(1, "Write back apply failed\n");
			return r;
		}
		//wait done
//		DBG_PRINTK("Waiting WB Done\n");
		wait_for_completion(&wb_done_completion);
//		DBG_PRINTK("WB Done and go ahead\n");

		omap_vout_display_lcd_wb_interleave(vout,
									wb_buffer->vaddr,vout->display_info.lcd.wb_pre_buffer->vaddr,
									out_width,out_height);

	}
	return 0;
}


/**
 * RGB16 to LCD
 */
static int omap_vout_display_lcd_argb32_out(struct omap_overlay *lcd,
		struct omap_vout_vram_buffer *buffer,
		u16 pos_x, u16 pos_y,
		u16 out_width, u16 out_height,
		enum omap_dss_overlay_s3d_type s3d_type)
{
	struct omap_overlay_info ovlInfo;
	struct omap_overlay_manager *mgr;
	int r;
	if ( lcd==NULL || buffer==NULL )
		return -EINVAL;

	if ( lcd->get_overlay_info==NULL )
		return -EINVAL;
	if ( lcd->set_overlay_info==NULL )
		return -EINVAL;
	mgr = lcd->manager;
	if ( mgr==NULL )
		return -EINVAL;

	lcd->get_overlay_info(lcd, &ovlInfo);

	//file overlay info
	ovlInfo.enabled	= true;
	ovlInfo.paddr		= buffer->paddr;
	ovlInfo.vaddr		= buffer->vaddr;
	ovlInfo.rotation	= OMAP_DSS_ROT_0;
	ovlInfo.rotation_type	= OMAP_DSS_ROT_DMA;
	ovlInfo.width		= out_width;
	ovlInfo.height		= out_height;
	ovlInfo.color_mode	= OMAP_DSS_COLOR_UYVY;
	ovlInfo.pos_x		= pos_x;
	ovlInfo.pos_y		= pos_y;
	ovlInfo.out_width	= out_width;
	ovlInfo.out_height	= out_height;

	ovlInfo.screen_width	= out_width;
	ovlInfo.out_wb		= false;
	ovlInfo.p_uv_addr	= 0;
	ovlInfo.mirror		= 0;
	ovlInfo.zorder		= 1;	//one video -> 1
	ovlInfo.s3d_type	= s3d_type;

//	DBG_PRINTK("In WB result set\n");
//	DBG_PRINTK("\tpaddr:0x%x, vaddr:0x%p\n", ovlInfo.paddr, ovlInfo.vaddr);
//	DBG_PRINTK("\tcolor mode:0x%x, rotation:%d\n", ovlInfo.color_mode, ovlInfo.rotation);
//	DBG_PRINTK("\tposition:(%d,%d)\n", ovlInfo.pos_x, ovlInfo.pos_y);
//	DBG_PRINTK("\tinput w:%d, h:%d\n", ovlInfo.width, ovlInfo.height);
//	DBG_PRINTK("\tout w:%d, h:%d\n", ovlInfo.out_width, ovlInfo.out_height);
//	DBG_PRINTK("\ts3d type :%d\n", ovlInfo.s3d_type);
	//set & apply
	r = lcd->set_overlay_info(lcd, &ovlInfo);
	if ( r )
		return r;
	r = mgr->apply(mgr);
	return r;
}

/**
 * Display to LCD
 */
static int  omap_vout_display_lcd(struct omap_vout_device *vout,
		struct videobuf_buffer *frame,
		struct omap_vout_frame_info*fInfo)
{
	u8 rotation;
	u16 pos_x, pos_y;
	u16 out_width, out_height;
	int src_w, src_h;
	struct omap_overlay *lcd;
	int r=0;
	enum v4l2_frame_pack_type	s3d_type;
	bool hdmi_enabled;

//	DBG_PRINTK("Displaying one frame to LCD\n");

	if ( vout==NULL || frame==NULL || fInfo==NULL )
		return -EINVAL;

	lcd = vout->vid_info.overlays.named.lcd;
	if ( lcd==NULL )
	{
		WARN(1, "No Lcd overlay\n");
		return -EINVAL;
	}

	//read current status of vout
	//display information can be changed, after frame queue
	//so this is best place to do this
	mutex_lock(&vout->lock);
	rotation		= vout->display_info.lcd.rotation;
	pos_x			= vout->display_info.lcd.win.w.left;
	pos_y			= vout->display_info.lcd.win.w.top;
	out_width		= vout->display_info.lcd.win.w.width;
	out_height		= vout->display_info.lcd.win.w.height;
	s3d_type		= vout->display_info.s3d_type;
	hdmi_enabled	= vout->display_info.hdmi.enable;
	mutex_unlock(&vout->lock);

	src_w			= fInfo->crop.width;
	src_h			= fInfo->crop.height;

	//set info
	if ( s3d_type==V4L2_FPACK_NONE || !is_supported_rotation_for_3d(rotation))
	{
		//NO S3D
		DBG_PRINTK("2D Frame out to LCD\n");
		//set lcd manager
		{
			struct omap_overlay_manager *lcd_mgr = omap_dss_get_overlay_manager(2);	//manger 2 is lcd
			if ( lcd->manager!=lcd_mgr )
			{
				INFO_PRINTK("Layer(%d)'s manager is set to LCD\n", lcd->id);
				omap_vout_remap_overlay_manager(lcd,lcd_mgr);
			}
			//set trans & alpha
			omap_vout_display_set_alpha_and_trans(vout, lcd_mgr);
		}
		r =  omap_vout_display_to_overlay_2d(lcd,
				frame,
				true,	//LCD is always connected
				rotation,
				pos_x, pos_y,
				out_width, out_height,
				true);
		if ( r )
			ERR_PRINTK("Set LCD 2D failed\n");
	}
	else
	{
		//S3D
		if ( hdmi_enabled )
		{
			//HDMI enabled, disable LCD out
			r = omap_vout_display_disable_overlay(lcd);
			if ( r )
				ERR_PRINTK("HDMI enabled and S3D trying disable LCD failed\n");
			DBG_PRINTK("S3D & HDMI enabled. LCD will be disabled\n");
		}
		else
		{
			struct omap_overlay *wb_input;
			struct omap_vout_vram_buffer *wb_buffer;

			if ( !is_supported_rotation_for_3d(rotation) )
			{
				ERR_PRINTK("S3D is supported on 90 degree(%d)\n", rotation);
				return -EINVAL;
			}

			//set lcd manager
			{
				struct omap_overlay_manager *lcd_mgr = omap_dss_get_overlay_manager(2);	//manger 2 is lcd
				if ( lcd->manager!=lcd_mgr )
				{
					INFO_PRINTK("Layer(%d)'s manger is set to LCD\n", lcd->id);
					omap_vout_remap_overlay_manager(lcd, lcd_mgr);
				}
				//set trans & alpha
				omap_vout_display_set_alpha_and_trans(vout, lcd_mgr);
			}

			//HDMI layer will be used as WB input layer
			wb_input = vout->vid_info.overlays.named.hdmi;

			//prepare wb buffer
			r = omap_vout_display_lcd_prepare_wb_buffer(vout);
			if ( r )
			{
				WARN(1, "Preapre WB failed\n");
				return r;
			}

			//ping-pong wb buffer
			wb_buffer = omap_vout_display_lcd_get_ping_pong_wb_buffer(vout);
			if ( wb_buffer==NULL || wb_buffer->paddr==0 )
			{
				WARN(1, "get write back buffer failed\n");
				return -EFAULT;
			}

			//configure wb and out
			if ( s3d_type==V4L2_FPACK_OVERUNDER )
			{
				//top bottom
				r = omap_vout_display_lcd_top_bottom_to_interlace(vout,
						frame,
						wb_input,
						wb_buffer,
						src_w, src_h,
						out_width, out_height);
				if ( r )
				{
					WARN(1, "Top bottom interlace failed\n");
					return r;
				}
			}
			else if ( s3d_type==V4L2_FPACK_SIDEBYSIDE )
			{
				r = omap_vout_display_lcd_side_side_to_interlace(vout,
						frame,
						wb_input,
						wb_buffer,
						src_w, src_h,
						out_width, out_height);
				
				if ( r )
				{
					WARN(1, "Side Side interlace failed\n");
					return r;
				}
			}
			else
			{
				WARN(1, "Unsupported S3D type\n");
				return -EINVAL;
			}

//			DBG_PRINTK("S3D Frame out to LCD\n");
			//set lcd manager
			{
				struct omap_overlay_manager *lcd_mgr = omap_dss_get_overlay_manager(2);	//manger 2 is lcd
				if ( lcd->manager!=lcd_mgr )
				{
					INFO_PRINTK("Layer(%d)'s manger is set to LCD\n", lcd->id);
					omap_vout_remap_overlay_manager(lcd,lcd_mgr);
				}
			}
			//set lcd
			r =  omap_vout_display_lcd_argb32_out(lcd,
					wb_buffer,
					pos_x, pos_y,
					out_width, out_height,
					omap_dss_overlay_s3d_interlaced);

			WARN(r, "Set Interlaced vram to LCD overlay failed\n");
		}
	}
//	DBG_PRINTK("Displaying one frame to LCD end\n");
	return r;
}

extern int hdmi_video_prepare_change(void *id, int layer_count, bool add_or_change, char *reason);
extern int hdmi_video_commit_change(void *id);

static void omap_vout_display_hdmi_disable(struct omap_vout_device *vout,
		struct omap_overlay_manager *hdmi_mgr,
		struct omap_overlay *hdmi, struct omap_overlay *lcd)
{
	struct omap_overlay_info info;
	bool changed = false;
	if ( hdmi->manager==hdmi_mgr )
	{
		hdmi->get_overlay_info(hdmi, &info);
		if ( info.enabled )
		{
			hdmi_video_prepare_change(vout, 0, false, "V4L2 HDMI Disabled");
			omap_vout_display_disable_overlay(hdmi);
			changed = true;
		}
	}
	if ( lcd->manager==hdmi_mgr )
	{
		lcd->get_overlay_info(lcd, &info);
		if ( info.enabled )
		{
			hdmi_video_prepare_change(vout, 0, false, "V4L2 HDMI Disabled");
			omap_vout_display_disable_overlay(lcd);
			changed = true;
		}
	}
	if ( changed )
		hdmi_video_commit_change(vout);
}

static int  omap_vout_display_hdmi(struct omap_vout_device *vout,
		struct videobuf_buffer *frame,
		struct omap_vout_frame_info *fInfo)
{
	u8 rotation;
	u8 lcd_rotation;
	u16 pos_x, pos_y;
	u16 out_width, out_height;
	u16 in_width, in_height;
	struct omap_overlay *hdmi;
	struct omap_overlay *lcd;
	bool enable;
	int r;
	enum v4l2_frame_pack_type s3d_type_v4l2;
	struct omap_overlay_manager *hdmi_mgr;
	int hdmi_need_commit = 0;
	bool out_is_2d;
	static bool previous_enabled = false;

	if ( vout==NULL || frame==NULL || fInfo==NULL )
		return -EINVAL;

	//read current status of vout
	//display information can be changed, after frame queue
	//so this is best place to do this
	mutex_lock(&vout->lock);
	enable		= vout->display_info.hdmi.enable;
	rotation	= vout->display_info.hdmi.rotation;
	pos_x		= vout->display_info.hdmi.win.w.left;
	pos_y		= vout->display_info.hdmi.win.w.top;
	out_width	= vout->display_info.hdmi.win.w.width;
	out_height	= vout->display_info.hdmi.win.w.height;
	s3d_type_v4l2	= vout->display_info.s3d_type;
	lcd_rotation = vout->display_info.lcd.rotation;
	mutex_unlock(&vout->lock);


	hdmi = vout->vid_info.overlays.named.hdmi;
	lcd = vout->vid_info.overlays.named.lcd;
	hdmi_mgr = omap_dss_get_overlay_manager(1);	//manger 1 is hdmi

	if ( hdmi==NULL || lcd==NULL || hdmi_mgr==NULL )
	{
		WARN(1, "no HDMI overlay\n");
		return -EINVAL;
	}
	
	//Check enable
	if ( !enable )
	{
		if ( previous_enabled )
		{
			//only disable when enable -> disable
			omap_vout_display_hdmi_disable(vout, hdmi_mgr, hdmi, lcd);
		}
		previous_enabled = false;
		return 0;
	}
	previous_enabled = true;

	//check scaling limitation
	in_width	= fInfo->crop.width;
	in_height	= fInfo->crop.height;
	if ( (in_width/2)>out_width || (in_height/2)>out_height )
	{
		ERR_PRINTK("HDMI doesn't support down-scaling. input(%dx%d) output(%dx%d)  Disable HDMI\n",
				in_width, in_height, out_width, out_height);
		omap_vout_display_hdmi_disable(vout, hdmi_mgr, hdmi, lcd);
		return 0;
	}

	// Clear wb on overlay for reuse
	omap_vout_clear_overlay_wb(hdmi);

	//set tv out
	if (hdmi_mgr!=hdmi->manager && hdmi->set_manager!=NULL )
	{
		//HDMI will be added
		INFO_PRINTK("Layer(%d)'s manager is set to HDMI\n", hdmi->id);
		hdmi_need_commit |= hdmi_video_prepare_change(vout, 0, true, "Layer hdmi change to HDMI");
		omap_vout_remap_overlay_manager(hdmi,hdmi_mgr);
		//real commit will be done after overlay setting
	}

	out_is_2d = s3d_type_v4l2==V4L2_FPACK_NONE 				//nof s3d
			|| rotation!=OMAP_DSS_ROT_0 					//HDMI only support 0 degree
			|| !is_supported_rotation_for_3d(lcd_rotation);

	if ( out_is_2d )
		hdmi_need_commit |= hdmi_video_prepare_change(vout, 1, true, "V4L2 2D Out");
	else
	{
		//For S3D, LCD & HDMI layers will be used for S3D display in HDMI
		if ( lcd->manager!=hdmi_mgr )
		{
			INFO_PRINTK("Layer(%d)'s manager is set to HDMI\n", lcd->id);
			hdmi_need_commit |= hdmi_video_prepare_change(vout, 0, true, "Layer lcd change to HDMI");
			omap_vout_remap_overlay_manager(lcd,hdmi_mgr);
		}
		hdmi_need_commit |= hdmi_video_prepare_change(vout, 2, true, "V4L2 S3D Out");
	}

	//set alpha & transparent
	omap_vout_display_set_alpha_and_trans(vout, hdmi_mgr);

	//set info
	if ( out_is_2d )
	{
		//2D frame
		DBG_PRINTK("2D frame out to HDMI\n");
		r =  omap_vout_display_to_overlay_2d(hdmi,
				frame,
				true,
				rotation,
				pos_x, pos_y,
				out_width, out_height,
				false);
	}
	else
	{
		//S3D frame
		u16		hdmi_res_x, hdmi_res_y;

		//get hdmi resolution
		{
			struct omap_video_timings timings;
			struct omap_dss_driver *hdmi_driver;
			if ( hdmi_mgr->device==NULL || hdmi_mgr->device->driver==NULL )
			{
				ERR_PRINTK("No Device or driver is conntected to HDMI\n");
				return -EINVAL;
			}
			hdmi_driver = hdmi_mgr->device->driver;
			if ( hdmi_driver->get_timings==NULL )
			{
				ERR_PRINTK("No get_timings() in hdmi driver\n");
				return -EINVAL;
			}
			hdmi_driver->get_timings(hdmi_mgr->device, &timings);
			hdmi_res_x = timings.x_res;
			hdmi_res_y = timings.y_res;
		}

		r =  omap_vout_display_to_overlay_s3d_hdmi(hdmi_mgr,
				lcd,
				hdmi,
				frame,
				pos_x, pos_y,
				out_width, out_height,
				v4l2_s3d_to_dss_s3d_type(s3d_type_v4l2),
				hdmi_res_x, hdmi_res_y,
				false);
	} //end of S3D frame out
	//commit HDMI change
	if ( hdmi_need_commit )
	{
		hdmi_video_commit_change(vout);
	}
	return r;
}

static bool omap_vout_display_can_lcd_out(struct omap_vout_device *vout)
{
	bool ret = false;
	mutex_lock(&vout->lock);
	if ( vout->display_info.s3d_type!=V4L2_FPACK_NONE )
	{
		if ( !is_supported_rotation_for_3d(vout->display_info.lcd.rotation) )
			ret = true;		//not supported rotation, lcd will display s3d without interleaving
		else
		{
			if ( vout->display_info.hdmi.enable)	//HDMI & LCD can't display s3d concurrently
				ret = false;
			else
				ret = true;
		}
	}
	else
		ret = true;
	mutex_unlock(&vout->lock);
	return ret;
}

/**
 * rendering queued frames to display
 * frame out should be serialized in here
 */
static void omap_vout_display(struct work_struct *work)
{
	struct omap_vout_display_work *w;
	struct videobuf_buffer *curFrame;

	if ( work==NULL )
	{
		WARN(1, "No work\n");
		return;
	}

	w = container_of(work, typeof(*w), work);
	if ( w->frame==NULL )
	{
		DBG_PRINTK("No frame exist\n");
		kfree(w);
		return;
	}

	curFrame = w->frame;
	//now displaying frame changed
	if(w->lastframe_update)
	{
		if(w->vout->queing_info.displaying_frame != curFrame)
		{
			kfree(w);
			return;			
		}
	}
	kfree(w);

	//frame out
	{
		struct omap_vout_frame_info	*fInfo;
		struct timeval now;
		struct omap_vout_device *vout;
		enum STREAMING_STATUS	streaming_status;


		fInfo = (struct omap_vout_frame_info*) curFrame->priv;
		if ( fInfo==NULL )
		{
			WARN(1, "no Frmae info\n");
			ERR_PRINTK("No omap_vout_frame info in frame\n");
			return;
		}
		vout = fInfo->vout;
		if ( vout==NULL )
		{
			WARN(1, "no vout\n");
			ERR_PRINTK("No vout info in frame\n");
			return;
		}

		//streaming check
		mutex_lock(&vout->lock);
		streaming_status = vout->streaming_status;
		mutex_unlock(&vout->lock);
		if ( streaming_status==E_STREAMING_STOPPED || streaming_status==E_STREAMING_STOPPING )
		{
			//streaming off
			curFrame->state = VIDEOBUF_DONE;
			wake_up(&curFrame->done);
			DBG_PRINTK("Streaming in stopping or stopped(%d). skip frame\n", streaming_status);
			return;
		}

		//if need more time, wait
		do_gettimeofday(&now);
		if ( timeval_compare(&curFrame->ts, &now) <=0 )
		{
			//need to wait
			struct timeval remain;
			unsigned long remain_jiffies;
			remain.tv_usec = now.tv_usec - curFrame->ts.tv_usec;
			remain.tv_sec = now.tv_sec - curFrame->ts.tv_sec;
			if ( remain.tv_usec < 0 )
			{
				remain.tv_usec += 1000000;
				remain.tv_sec--;
			}
			remain_jiffies = timeval_to_jiffies(&remain);
//			DBG_PRINTK("Frame out time remains wait %lu\n", remain_jiffies);
			schedule_timeout(remain_jiffies);
		}


		//Display
		curFrame->state = VIDEOBUF_ACTIVE;

		mutex_lock(&vout->in_display_operation);

		mutex_lock(&vout->lock);
		streaming_status = vout->streaming_status;
		mutex_unlock(&vout->lock);

		//omap_vout_display_xxx() function wil check current display information,
		//and determine their action in inside function
		if ( streaming_status!=E_STREAMING_NO_DRAW)
		{
			//HDMI
			if ( omap_vout_display_hdmi(vout, curFrame, fInfo) )
				v4l2_err(&vout->vid_dev->v4l2_dev, "HDMI Out failed\n");
			//LCD
			if ( omap_vout_display_can_lcd_out(vout) )
			{
				if ( omap_vout_display_lcd(vout, curFrame, fInfo) )
					v4l2_err(&vout->vid_dev->v4l2_dev, "LCD Out failed\n");
			}
		}
		else
		{
			DBG_PRINTK("NOT DRAW skip frame\n");
		}

		mutex_unlock(&vout->in_display_operation);

		// LGE_CHANGE [darren.kang@lge.com] last frame logic change
		if(vout->queing_info.displaying_frame == curFrame)
			return;
		
		//now displaying frame changed
		mutex_lock(&vout->lock);
		if ( vout->queing_info.displaying_frame!=NULL )
		{
			//dequeue previous frame
			struct videobuf_buffer* pre = vout->queing_info.displaying_frame;
			pre->state = VIDEOBUF_DONE;
			wake_up(&pre->done);
		}
		vout->queing_info.displaying_frame = curFrame;
		mutex_unlock(&vout->lock);
	} //end of processing queued frames
}

/**
 * rendering previous frame 
 * for update display when setting is changed (ex)rotation)
 */
static int omap_vout_display_force_update(struct omap_vout_device *vout)
{
	if(vout->streaming_status == E_STREAMING_ON_GOING)
	{
		struct omap_vout_display_work *work;
		work = kzalloc(sizeof(*work), GFP_ATOMIC);
		if( work==NULL )
		{
			return  -EINVAL;
		}

		work->frame = vout->queing_info.displaying_frame;		
		work->lastframe_update = true;
		work->vout = vout;		

		INIT_WORK(&work->work, omap_vout_display);
		queue_work(vout->workqueue, &work->work);
	}
	return 0;
}

//----------------------------------------------------------------------------------------
//	Videob Buffer member functions

/* Video buffer call backs */

/*
 * Buffer setup function is called by videobuf layer when REQBUF ioctl is
 * called. This is used to setup buffers and return size and count of
 * buffers allocated. After the call to this buffer, videobuf layer will
 * setup buffer queue depending on the size and count of buffers
 */
static int omap_vout_buffer_setup(struct videobuf_queue *q, unsigned int *count,
			  unsigned int *size)
{
	int i;
	struct omap_vout_device *vout = q->priv_data;

	if (!vout)
		return -EINVAL;

	if (V4L2_BUF_TYPE_VIDEO_OUTPUT != q->type)
		return -EINVAL;

	if (V4L2_MEMORY_MMAP != vout->buf_info.memory)
		return 0;

	/* tiler_alloc_buf to be called here
	pre-requisites: rotation, format?
	based on that buffers will be allocated.
	*/
	/* Now allocated the V4L2 buffers */
	/* i is the block-width - either 4K or 8K, depending upon input width*/
	i = (vout->input_info.pix.width * vout->input_info.bpp +
		TILER_PAGE - 1) & ~(TILER_PAGE - 1);

	/* for NV12 format, buffer is height + height / 2*/
	if (OMAP_DSS_COLOR_NV12 == vout->input_info.dss_mode)
		*size = vout->buf_info.buffer_size = (vout->input_info.pix.height * 3/2 * i);
	else
		*size = vout->buf_info.buffer_size = (vout->input_info.pix.height * i);

	v4l2_dbg(1, debug, &vout->vid_dev->v4l2_dev,
			"\nheight=%d, size = %d, vout->buffer_sz=%d\n",
			vout->input_info.pix.height, *size, vout->buf_info.buffer_size);
	if (omap_vout_tiler_buffer_setup(vout, count, 0, &vout->input_info.pix))
			return -ENOMEM;
	return 0;
}

/*
 * This function will be called when VIDIOC_QBUF ioctl is called.
 * It prepare buffers before give out for the display. This function
 * converts user space virtual address into physical address if userptr memory
 * exchange mechanism is used. If rotation is enabled, it copies entire
 * buffer into VRFB memory space before giving it to the DSS.
 */
static int omap_vout_buffer_prepare(struct videobuf_queue *q,
			    struct videobuf_buffer *vb,
			    enum v4l2_field field)
{
	struct omap_vout_device *vout = q->priv_data;

	if (VIDEOBUF_NEEDS_INIT == vb->state) {
		vb->width = vout->input_info.pix.width;
		vb->height = vout->input_info.pix.height;
		vb->size = vb->width * vb->height * vout->input_info.bpp;
		vb->field = field;
		INIT_LIST_HEAD(&vb->queue);
	}
	//attach frame info with V4L2 Video Buffer
	{
		struct omap_vout_frame_info *fInfo;
		unsigned long	paddr, uv_addr;
		int index= vb->i;
		int r;

//		DBG_PRINTK("Preparing buffer %d\n", index);
		//fill current status
		//No needs sycn. qbuf maintain mutex

		fInfo = &vout->queing_info.frames[index];
		fInfo->color	= vout->input_info.dss_mode;

		//get address from cropping
		r = omap_vout_get_offset_tiler(fInfo->color,
				vout->buf_info.buf_phy_addr[index],
				vout->buf_info.buf_phy_uv_addr[index],
				vout->input_info.crop.left,
				vout->input_info.crop.top,
				&paddr, &uv_addr);
		if ( r )
		{
			WARN(1, "get address of corpping failed. anywya go on!!\n");
			paddr = vout->buf_info.buf_phy_addr[index];
			uv_addr = vout->buf_info.buf_phy_uv_addr[index];
		}

		fInfo->pAddr	= paddr;
		fInfo->pUvAddr	= uv_addr;
		fInfo->vAddr	= NULL;		//not used in tiler

		fInfo->pix		= vout->input_info.pix;
		fInfo->crop		= vout->input_info.crop;

		fInfo->vout		= vout;

		vb->priv = fInfo;
	}
	vb->state = VIDEOBUF_PREPARED;

	return 0;
}

/*
 * Buffer queue funtion will be called from the videobuf layer when _QBUF
 * ioctl is called. It is used to enqueue buffer, which is ready to be
 * displayed.
 */
static void omap_vout_buffer_queue(struct videobuf_queue *q,
			  struct videobuf_buffer *vb)
{
	struct omap_vout_device *vout = q->priv_data;

//	DBG_PRINTK("Add %d video buffer to req_queue\n", vb->i);
	list_add_tail(&vb->queue, &vout->queing_info.drawing_req_queue);
	vb->state = VIDEOBUF_QUEUED;
}

/*
 * Buffer release function is called from videobuf layer to release buffer
 * which are already allocated
 */
static void omap_vout_buffer_release(struct videobuf_queue *q,
			    struct videobuf_buffer *vb)
{
	struct omap_vout_device *vout = q->priv_data;

	vb->state = VIDEOBUF_NEEDS_INIT;

	if (V4L2_MEMORY_MMAP != vout->buf_info.memory)
		return;
}

/*
 *  File operations
 */
static void omap_vout_vm_open(struct vm_area_struct *vma)
{
	struct omap_vout_device *vout = vma->vm_private_data;

	v4l2_dbg(1, debug, &vout->vid_dev->v4l2_dev,
		"vm_open [vma=%08lx-%08lx]\n", vma->vm_start, vma->vm_end);
	vout->buf_info.mmap_count++;
}

static void omap_vout_vm_close(struct vm_area_struct *vma)
{
	struct omap_vout_device *vout = vma->vm_private_data;

	v4l2_dbg(1, debug, &vout->vid_dev->v4l2_dev,
		"vm_close [vma=%08lx-%08lx]\n", vma->vm_start, vma->vm_end);
	vout->buf_info.mmap_count--;
}

static struct vm_operations_struct omap_vout_vm_ops = {
	.open	= omap_vout_vm_open,
	.close	= omap_vout_vm_close,
};

static int omap_vout_mmap(struct file *file, struct vm_area_struct *vma)
{
	int i;
	void *pos;
	int j = 0, k = 0, m = 0, p = 0, m_increment = 0;
	struct omap_vout_device *vout = file->private_data;
	struct videobuf_queue *q = &vout->queing_info.vbq;

	v4l2_dbg(1, debug, &vout->vid_dev->v4l2_dev,
			" %s pgoff=0x%lx, start=0x%lx, end=0x%lx\n", __func__,
			vma->vm_pgoff, vma->vm_start, vma->vm_end);

	/* look for the buffer to map */
	for (i = 0; i < VIDEO_MAX_FRAME; i++) {
		if (NULL == q->bufs[i])
			continue;
		if (V4L2_MEMORY_MMAP != q->bufs[i]->memory)
			continue;
		if (q->bufs[i]->boff == (vma->vm_pgoff << PAGE_SHIFT))
			break;
	}

	if (VIDEO_MAX_FRAME == i) {
		v4l2_dbg(1, debug, &vout->vid_dev->v4l2_dev,
				"offset invalid [offset=0x%lx]\n",
				(vma->vm_pgoff << PAGE_SHIFT));
		return -EINVAL;
	}
	q->bufs[i]->baddr = vma->vm_start;

	vma->vm_flags |= VM_RESERVED;
	vma->vm_page_prot = pgprot_writecombine(vma->vm_page_prot);
	vma->vm_ops = &omap_vout_vm_ops;
	vma->vm_private_data = (void *) vout;
	/* Tiler remapping */
	pos = (void *) vout->buf_info.buf_phy_addr[i];
	/* get line width */
	/* for NV12, Y buffer is 1bpp*/
	if (OMAP_DSS_COLOR_NV12 == vout->input_info.dss_mode) {
		p = (vout->input_info.pix.width +
			TILER_PAGE - 1) & ~(TILER_PAGE - 1);
		m_increment = 64 * TILER_WIDTH;
	} else {
		p = (vout->input_info.pix.width * vout->input_info.bpp +
			TILER_PAGE - 1) & ~(TILER_PAGE - 1);

		if (vout->input_info.bpp > 1)
			m_increment = 2*64*TILER_WIDTH;
		else
			m_increment = 64 * TILER_WIDTH;
	}

	for (j = 0; j < vout->input_info.pix.height; j++) {
		/* map each page of the line */
	#if 0
		if (0)
			printk(KERN_NOTICE
				"Y buffer %s::%s():%d: vm_start+%d = 0x%lx,"
				"dma->vmalloc+%d = 0x%lx, w=0x%x\n",
				__FILE__, __func__, __LINE__,
				k, vma->vm_start + k, m,
				(pos + m), p);
	#endif
		vma->vm_pgoff =
			((unsigned long)pos + m) >> PAGE_SHIFT;

		if (remap_pfn_range(vma, vma->vm_start + k,
			((unsigned long)pos + m) >> PAGE_SHIFT,
			p, vma->vm_page_prot))
				return -EAGAIN;
		k += p;
		m += m_increment;
	}
	m = 0;

	/* UV Buffer in case of NV12 format */
	if (OMAP_DSS_COLOR_NV12 == vout->input_info.dss_mode) {
		pos = (void *) vout->buf_info.buf_phy_uv_addr[i];
		/* UV buffer is 2 bpp, but half size, so p remains */
		m_increment = 2*64*TILER_WIDTH;

		/* UV buffer is height / 2*/
		for (j = 0; j < vout->input_info.pix.height / 2; j++) {
			/* map each page of the line */
		#if 0
			if (0)
				printk(KERN_NOTICE
				"UV buffer %s::%s():%d: vm_start+%d = 0x%lx,"
				"dma->vmalloc+%d = 0x%lx, w=0x%x\n",
				__FILE__, __func__, __LINE__,
				k, vma->vm_start + k, m,
				(pos + m), p);
		#endif
			vma->vm_pgoff =
				((unsigned long)pos + m) >> PAGE_SHIFT;

			if (remap_pfn_range(vma, vma->vm_start + k,
				((unsigned long)pos + m) >> PAGE_SHIFT,
				p, vma->vm_page_prot))
				return -EAGAIN;
			k += p;
			m += m_increment;
		}
	}

	vma->vm_flags &= ~VM_IO; /* using shared anonymous pages */
	vout->buf_info.mmap_count++;
	v4l2_dbg(1, debug, &vout->vid_dev->v4l2_dev, "Exiting %s\n", __func__);

	return 0;
}

static unsigned int omap_vout_poll(struct file *file, struct poll_table_struct *poll)
{
	struct omap_vout_device *vout = file->private_data;
	struct videobuf_queue *q = &vout->queing_info.vbq;

	DBG_PRINTK("Vout polling\n");
	/* we handle this here as videobuf_poll_stream would start reading */
	if ( vout->streaming_status==E_STREAMING_STOPPED )
		return POLLERR;

	return videobuf_poll_stream(file, q, poll);
}

static int omap_vout_release(struct file *file)
{
	unsigned int i;
	struct videobuf_queue *q;
	struct omapvideo_info *ovid;
	struct omap_vout_device *vout = file->private_data;

	DBG_PRINTK("omap_vout_release() Enter\n");
	if (!vout)
		return 0;

	mutex_lock(&vout->lock);
	vout->opened -= 1;
	if (vout->opened > 0) {
		/* others still have this device open */
		v4l2_dbg(1, debug, &vout->vid_dev->v4l2_dev,
				"device still opened: %d\n", vout->opened);
		mutex_unlock(&vout->lock);
		return 0;
	}
	mutex_unlock(&vout->lock);

	//little worry about no mutex lock
	if ( vout->streaming_status!=E_STREAMING_STOPPED )
		vidioc_streamoff(file, file->private_data, V4L2_BUF_TYPE_VIDEO_OUTPUT);

	//disable hdmi
	vout->display_info.hdmi.enable = false;

	v4l2_dbg(1, debug, &vout->vid_dev->v4l2_dev, "Entering %s\n", __func__);
	ovid = &vout->vid_info;

	q = &vout->queing_info.vbq;
	/* Disable all the overlay managers connected with this interface */
	for (i = 0; i < ovid->num_overlays; i++) {
		struct omap_overlay *ovl = ovid->overlays.overlays[i];
		if (ovl->manager && ovl->manager->device) {
			struct omap_overlay_info info;
			ovl->get_overlay_info(ovl, &info);
			if ( info.enabled==0 )
				continue;
			info.enabled = 0;
			ovl->set_overlay_info(ovl, &info);
		}
	}

	omap_vout_free_tiler_buffers(vout);
	videobuf_mmap_free(q);

	/* Even if apply changes fails we should continue
	   freeing allocated memeory */
	if (vout->streaming_status!=E_STREAMING_STOPPED ) {
		videobuf_streamoff(q);
		videobuf_queue_cancel(q);
		vout->streaming_status = E_STREAMING_STOPPED;
	}

	if (vout->buf_info.mmap_count != 0)
		vout->buf_info.mmap_count = 0;

	file->private_data = NULL;

	if (vout->buf_info.buffer_allocated)
		videobuf_mmap_free(q);

	v4l2_dbg(1, debug, &vout->vid_dev->v4l2_dev, "Exiting %s\n", __func__);
	return 0;
}

static int omap_vout_open(struct file *file)
{
	struct videobuf_queue *q;
	struct omap_vout_device *vout = NULL;

	vout = video_drvdata(file);


	mutex_lock(&vout->lock);
	vout->opened += 1;
	if (vout->opened > 1) {
		v4l2_dbg(1, debug, &vout->vid_dev->v4l2_dev,
				"device already opened: %d\n", vout->opened);
		file->private_data = vout;
		mutex_unlock(&vout->lock);
		return 0;
	}
	mutex_unlock(&vout->lock);

	v4l2_dbg(1, debug, &vout->vid_dev->v4l2_dev, "Entering %s\n", __func__);

	file->private_data = vout;
	vout->buf_info.type = V4L2_BUF_TYPE_VIDEO_OUTPUT;

	q = &vout->queing_info.vbq;
	video_vbq_ops.buf_setup = omap_vout_buffer_setup;
	video_vbq_ops.buf_prepare = omap_vout_buffer_prepare;
	video_vbq_ops.buf_release = omap_vout_buffer_release;
	video_vbq_ops.buf_queue = omap_vout_buffer_queue;
	spin_lock_init(&vout->queing_info.vbq_lock);

	videobuf_queue_dma_contig_init(q, &video_vbq_ops, q->dev,
			&vout->queing_info.vbq_lock, vout->buf_info.type, V4L2_FIELD_NONE,
			sizeof(struct videobuf_buffer), vout);

	v4l2_dbg(1, debug, &vout->vid_dev->v4l2_dev, "Exiting %s\n", __func__);
	return 0;
}

/*
 * V4L2 ioctls
 */
static int vidioc_querycap(struct file *file, void *fh,
		struct v4l2_capability *cap)
{
	struct omap_vout_device *vout = fh;

	strlcpy(cap->driver, VOUT_NAME, sizeof(cap->driver));
	strlcpy(cap->card, vout->vfd->name, sizeof(cap->card));
	cap->bus_info[0] = '\0';
	cap->capabilities = V4L2_CAP_STREAMING | V4L2_CAP_VIDEO_OUTPUT | V4L2_CAP_VIDEO_OVERLAY;

	return 0;
}

static int vidioc_enum_fmt_vid_out(struct file *file, void *fh,
			struct v4l2_fmtdesc *fmt)
{
	int index = fmt->index;
	enum v4l2_buf_type type = fmt->type;

	fmt->index = index;
	fmt->type = type;
	if (index >= NUM_OUTPUT_FORMATS)
		return -EINVAL;

	fmt->flags = omap_formats[index].flags;
	strlcpy(fmt->description, omap_formats[index].description,
			sizeof(fmt->description));
	fmt->pixelformat = omap_formats[index].pixelformat;

	return 0;
}

static int vidioc_g_fmt_vid_out(struct file *file, void *fh,
			struct v4l2_format *f)
{
	struct omap_vout_device *vout = fh;

	f->fmt.pix = vout->input_info.pix;
	return 0;

}

static int vidioc_try_fmt_vid_out(struct file *file, void *fh,
			struct v4l2_format *f)
{
	struct omap_overlay *ovl;
	struct omapvideo_info *ovid;
	struct omap_video_timings *timing;
	struct omap_vout_device *vout = fh;

	ovid = &vout->vid_info;
	//This ioctl function is used for video capturing device usually.
	//LCD Capturing is enough in cosmo
	ovl = ovid->overlays.named.lcd;

	if (!ovl->manager || !ovl->manager->device)
		return -EINVAL;
	/* get the display device attached to the overlay */
	timing = &ovl->manager->device->panel.timings;

	vout->buf_info.fbuf.fmt.height = timing->y_res;
	vout->buf_info.fbuf.fmt.width = timing->x_res;

	omap_vout_try_format(&f->fmt.pix);
	return 0;
}

static int try_format(struct v4l2_pix_format *pix)
{
	int ifmt, bpp = 0;

	pix->height = clamp(pix->height, (u32) VID_MIN_HEIGHT,
			    (u32) VID_MAX_HEIGHT);

	pix->width =
	    clamp(pix->width, (u32) VID_MIN_WIDTH, (u32) VID_MAX_WIDTH);

	for (ifmt = 0; ifmt < NUM_OUTPUT_FORMATS; ifmt++) {
		if (pix->pixelformat == omap_formats[ifmt].pixelformat)
			break;
	}

	if (ifmt == NUM_OUTPUT_FORMATS)
		ifmt = 0;

	pix->pixelformat = omap_formats[ifmt].pixelformat;
	pix->field = V4L2_FIELD_ANY;
	pix->priv = 0;

	switch (pix->pixelformat) {
	case V4L2_PIX_FMT_YUYV:
	case V4L2_PIX_FMT_UYVY:
	default:
		pix->colorspace = V4L2_COLORSPACE_JPEG;
		bpp = YUYV_BPP;
		break;
	case V4L2_PIX_FMT_RGB565:
	case V4L2_PIX_FMT_RGB565X:
		pix->colorspace = V4L2_COLORSPACE_SRGB;
		bpp = RGB565_BPP;
		break;
	case V4L2_PIX_FMT_RGB24:
		pix->colorspace = V4L2_COLORSPACE_SRGB;
		bpp = RGB24_BPP;
		break;
	case V4L2_PIX_FMT_RGB32:
	case V4L2_PIX_FMT_BGR32:
		pix->colorspace = V4L2_COLORSPACE_SRGB;
		bpp = RGB32_BPP;
		break;
	case V4L2_PIX_FMT_NV12:
		pix->colorspace = V4L2_COLORSPACE_JPEG;
		bpp = 1;
		break;
	}

	pix->bytesperline = pix->width * bpp;
	/*Round to the next page size for TILER buffers,
	* this will be the stride/vert pitch*/
	pix->bytesperline = (pix->bytesperline + PAGE_SIZE - 1) & ~(PAGE_SIZE - 1);

	pix->sizeimage = pix->bytesperline * pix->height;
	if (V4L2_PIX_FMT_NV12 == pix->pixelformat)
		pix->sizeimage += pix->sizeimage >> 1;

	return bpp;
}

static int vidioc_s_fmt_vid_out(struct file *file, void *fh,
			struct v4l2_format *f)
{
	int bpp;
	struct omap_vout_device *vout = fh;

	mutex_lock(&vout->lock);

	if (vout->streaming_status!=E_STREAMING_STOPPED ) {
		mutex_unlock(&vout->lock);
		return -EBUSY;
	}

	bpp = try_format(&f->fmt.pix);
	if ( bpp==0 )
	{
		//not supported type
		mutex_unlock(&vout->lock);
		return -EINVAL;
	}
	//clear related flags
	vout->display_info.s3d_type = V4L2_FPACK_NONE;
	vout->display_info.s3d_pack_order = 0;
	vout->display_info.depth_factor = 0;

	vout->input_info.bpp = bpp;
	vout->input_info.pix = f->fmt.pix;
	vout->input_info.dss_mode = video_mode_to_dss_mode(&vout->input_info.pix);
	if ( vout->input_info.dss_mode == -EINVAL)
	{
		//not supported color format
		printk("Not Supported Pix Format(%d) in vidioc_s_fmt_vid_out()\n", vout->input_info.pix.pixelformat);
		mutex_unlock(&vout->lock);
		return -EINVAL;
	}
	//clear s3d display type
	vout->display_info.s3d_type = omap_dss_overlay_s3d_none;
	mutex_unlock(&vout->lock);
	return 0;
}

static int vidioc_try_fmt_vid_overlay(struct file *file, void *fh,
			struct v4l2_format *f)
{
	int ret = 0;
	struct omap_vout_device *vout = fh;
	struct v4l2_window *win = &f->fmt.win;

	ret = omap_vout_try_window(&vout->buf_info.fbuf, win);

	if (!ret) {
		if (vout->vid == OMAP_VIDEO1)
			win->global_alpha = 255;
		else
			win->global_alpha = f->fmt.win.global_alpha;
	}

	return ret;
}

/**
 * Change LCD Overlay
 */
static int vidioc_s_fmt_vid_overlay(struct file *file, void *fh,
			struct v4l2_format *f)
{
	struct omap_vout_device *vout = fh;
	struct v4l2_window *win = &f->fmt.win;
	struct v4l2_window *lcd_win = &vout->display_info.lcd.win;

	mutex_lock(&vout->lock);
	lcd_win->w	= win->w;	//Only Display Position will be used in cosmo	

	lcd_win->w.top = (lcd_win->w.top/2)*2;
	lcd_win->w.left = (lcd_win->w.left/2)*2;
	lcd_win->w.height = (lcd_win->w.height/2)*2;
	lcd_win->w.width = (lcd_win->w.width/4)*4;
	
	mutex_unlock(&vout->lock);

	// force update frame
	omap_vout_display_force_update(vout);	

	return 0;
}

/**
 * Get LCD Display Information
 */
static int vidioc_g_fmt_vid_overlay(struct file *file, void *fh,
			struct v4l2_format *f)
{
	struct omap_vout_device *vout = fh;
	struct v4l2_window *win = &f->fmt.win;
	struct v4l2_window *lcd_win = &vout->display_info.lcd.win;

	mutex_lock(&vout->lock);
	win->w		= lcd_win->w;	//Only Display Position will be used in cosmo
	mutex_unlock(&vout->lock);

	return 0;
}

/**
 * Change HDMI Overlay
 */
static int vidioc_s_fmt_type_private(struct file *file, void *fh,
			struct v4l2_format *f)
{
	struct omap_vout_device *vout = fh;
	struct v4l2_window *win = &f->fmt.win;
	struct v4l2_window *hdmi_win = &vout->display_info.hdmi.win;

	INFO_PRINTK("vidioc_s_fmt_type_private\n");
	mutex_lock(&vout->lock);
	hdmi_win->w	= win->w;	//Only Display Position will be used in cosmo
	INFO_PRINTK("HDMI rect (%d,%d) (%d,%d)\n", hdmi_win->w.left, hdmi_win->w.top, hdmi_win->w.width, hdmi_win->w.height);
	mutex_unlock(&vout->lock);

	return 0;
}

/**
 * Get HDMI overlay Information
 */
static int vidioc_g_fmt_type_private(struct file *file, void *fh,
			struct v4l2_format *f)
{
	struct omap_vout_device *vout = fh;
	struct v4l2_window *win = &f->fmt.win;
	struct v4l2_window *hdmi_win = &vout->display_info.lcd.win;

	printk("vidioc_g_fmt_type_private\n");
	mutex_lock(&vout->lock);
	win->w		= hdmi_win->w;	//Only Display Position will be used in cosmo
	mutex_unlock(&vout->lock);

	return 0;
}


static int vidioc_enum_fmt_vid_overlay(struct file *file, void *fh,
			struct v4l2_fmtdesc *fmt)
{
	int index = fmt->index;
	enum v4l2_buf_type type = fmt->type;

	fmt->index = index;
	fmt->type = type;
	if (index >= NUM_OUTPUT_FORMATS)
		return -EINVAL;

	fmt->flags = omap_formats[index].flags;
	strlcpy(fmt->description, omap_formats[index].description,
			sizeof(fmt->description));
	fmt->pixelformat = omap_formats[index].pixelformat;
	return 0;
}


static int vidioc_cropcap(struct file *file, void *fh,
		struct v4l2_cropcap *cropcap)
{
	struct omap_vout_device *vout = fh;
	struct v4l2_pix_format *pix = &vout->input_info.pix;

	if (cropcap->type != V4L2_BUF_TYPE_VIDEO_OUTPUT)
		return -EINVAL;

	/* Width and height are always even */
	cropcap->bounds.width = pix->width & ~1;
	cropcap->bounds.height = pix->height & ~1;

	omap_vout_default_crop(&vout->input_info.pix, &vout->buf_info.fbuf, &cropcap->defrect);
	cropcap->pixelaspect.numerator = 1;
	cropcap->pixelaspect.denominator = 1;
	return 0;
}

static int vidioc_g_crop(struct file *file, void *fh, struct v4l2_crop *crop)
{
	struct omap_vout_device *vout = fh;

	if (crop->type != V4L2_BUF_TYPE_VIDEO_OUTPUT)
		return -EINVAL;
	crop->c = vout->input_info.crop;
	return 0;
}

static int vidioc_s_crop(struct file *file, void *fh, struct v4l2_crop *crop)
{
	int ret = -EINVAL;
	int multiplier = 1;
	struct omap_vout_device *vout = fh;
	struct omapvideo_info *ovid;
	struct omap_overlay *ovl;
	struct omap_video_timings *timing;

	/* Currently we only allow changing the crop position while
	   streaming.  */
	if (vout->streaming_status==E_STREAMING_ON_GOING &&
	    (crop->c.height != vout->input_info.crop.height ||
	     crop->c.width != vout->input_info.crop.width))
		return -EBUSY;

	mutex_lock(&vout->lock);
	ovid = &vout->vid_info;
	//I can't understand why cropping needs overlay information..
	ovl = ovid->overlays.named.lcd;

	if (!ovl->manager || !ovl->manager->device) {
		ret = -EINVAL;
		goto s_crop_err;
	}
	/* get the display device attached to the overlay */
	timing = &ovl->manager->device->panel.timings;

	/* y resolution to be doubled in case of interlaced output */
	if (ovl->info.field & OMAP_FLAG_IDEV)
		multiplier = 2;

	// LGE CHANGES Darren.kang 2011.02.07 For prevent exception when window size is zero
	if (vout->display_info.lcd.win.w.width <= 0 || vout->display_info.lcd.win.w.height <= 0)
	{
		ret = 0;
		goto s_crop_err;
	}

	if (crop->type == V4L2_BUF_TYPE_VIDEO_OUTPUT)
		ret = omap_vout_new_crop(&vout->input_info.pix, &vout->input_info.crop, &vout->display_info.lcd.win,
				&vout->buf_info.fbuf, &crop->c);

s_crop_err:
	mutex_unlock(&vout->lock);
	return ret;
}

static int vidioc_queryctrl(struct file *file, void *fh,
		struct v4l2_queryctrl *ctrl)
{
	int ret = 0;
	struct omap_vout_device *vout = fh;

	switch (ctrl->id) {
	case V4L2_CID_ROTATE:
		ret = v4l2_ctrl_query_fill(ctrl, 0, 270, 90, 0);
		break;
	case V4L2_CID_TI_DISPC_OVERLAY:
		/* not settable for now */
		//only lcd
		ret = v4l2_ctrl_query_fill(ctrl,
					   vout->vid_info.overlays.named.lcd->id,
					   vout->vid_info.overlays.named.lcd->id,
					   1,
					   vout->vid_info.overlays.named.lcd->id);
		break;
	default:
		ctrl->name[0] = '\0';
		ret = -EINVAL;
	}
	return ret;
}

static int vidioc_g_ctrl(struct file *file, void *fh, struct v4l2_control *ctrl)
{
	int ret = 0;
	struct omap_vout_device *vout = fh;

	switch (ctrl->id) {
	case V4L2_CID_ROTATE:
		dss_rot_to_v4l2_rot( vout->display_info.lcd.rotation, &ctrl->value);
		break;
	case V4L2_CID_TI_DISPC_OVERLAY:
		ctrl->value = vout->vid_info.overlays.named.lcd->id;
		break;
	case ( V4L2_CID_PRIVATE_BASE + 'H'*0x100 + 'e' ) :	//HDMI enable flag
		mutex_lock(&vout->lock);
		ctrl->value = vout->display_info.hdmi.enable;
		mutex_unlock(&vout->lock);
		break;
	case ( V4L2_CID_PRIVATE_BASE + 'H'*0x100 + 'r' ) : //HDMI Rotation
		dss_rot_to_v4l2_rot( vout->display_info.hdmi.rotation, &ctrl->value);
		break;
	case ( V4L2_CID_PRIVATE_BASE + 'S'*0x100 + 'f' ) : //S3D type
		mutex_lock(&vout->lock);
		ctrl->value = vout->display_info.s3d_type;
		mutex_unlock(&vout->lock);
		break;
	case ( V4L2_CID_PRIVATE_BASE + 'S'*0x100 + 'o' ) : //S3D packorder
		mutex_lock(&vout->lock);
		ctrl->value = vout->display_info.s3d_pack_order;
		mutex_unlock(&vout->lock);
		break;
	case ( V4L2_CID_PRIVATE_BASE + 'S'*0x100 + 'd' ) : //S3D depth
		mutex_lock(&vout->lock);
		ctrl->value = vout->display_info.depth_factor;
		mutex_unlock(&vout->lock);
		break;
		
	default:
		ret = -EINVAL;
	}
	return ret;
}

static int vidioc_s_ctrl(struct file *file, void *fh, struct v4l2_control *a)
{
	int ret = 0;
	struct omap_vout_device *vout = fh;

	switch (a->id) {
	case V4L2_CID_ROTATE:
	{
		int rotation = a->value;

// sujin000.lee@lge.com for VT Front camera WA
//		if (1 == IsVideoTelephonyActivated()) /* baeyoung.park 2011-02-17 */
//		{
//20110217 wootaek.lim Rotation For Domestic Cosmo VT START
//#if defined(CONFIG_MACH_LGE_COSMO_DOMASTIC)
//			DBG_PRINTK("##########VT Domestic Rotation##########");
//			rotation = 270;
//#else
//			DBG_PRINTK("##########VT Global Rotation##########");
//			rotation = 90;	
			//printk("VT activated !!!!!!!!!!\n");
//#endif
//20110217 wootaek.lim Rotation For Domestic Cosmo VT END
//		}        

		mutex_lock(&vout->lock);

		if (v4l2_rot_to_dss_rot(rotation, &vout->display_info.lcd.rotation) )
		{
			mutex_unlock(&vout->lock);
			return  -EINVAL;
		}		
		mutex_unlock(&vout->lock);

		

		break;
	}
	case ( V4L2_CID_PRIVATE_BASE + 'H'*0x100 + 'e' ) :	//HDMI enable flag
		mutex_lock(&vout->lock);
		if ( vout->display_info.hdmi.enable && !a->value && vout->streaming_status==E_STREAMING_ON_GOING )
		{
			//off HDMI, force redraw for remove layers from HDMI
			INFO_PRINTK("HDMI Disabled. force redraw\n");
			omap_vout_display_force_update(vout);
			vout->display_info.hdmi.enable = false;
		}
		else
		{
			vout->display_info.hdmi.enable = a->value;
			INFO_PRINTK("HDMI Enable %d\n", vout->display_info.hdmi.enable);
		}
		mutex_unlock(&vout->lock);
		break;
	case ( V4L2_CID_PRIVATE_BASE + 'H'*0x100 + 'r' ) : //HDMI Rotation
	{
		int rotation = a->value;
		mutex_lock(&vout->lock);
		if (v4l2_rot_to_dss_rot(rotation, &vout->display_info.hdmi.rotation) )
		{
			mutex_unlock(&vout->lock);
			return  -EINVAL;
		}
		mutex_unlock(&vout->lock);
		break;
	}
	case ( V4L2_CID_PRIVATE_BASE + 'S'*0x100 + 'f' ) : //S3D type
		mutex_lock(&vout->lock);
		vout->display_info.s3d_type = a->value;
		mutex_unlock(&vout->lock);
		break;
	
	case ( V4L2_CID_PRIVATE_BASE + 'S'*0x100 + 'o' ) : //S3D packorder
		if(a->value == 0 || a->value == 1)
		{
			mutex_lock(&vout->lock);
			vout->display_info.s3d_pack_order = a->value;
			mutex_unlock(&vout->lock);
		}
		else
			return -EINVAL;
		break;

	case ( V4L2_CID_PRIVATE_BASE + 'S'*0x100 + 'd' ) : //S3D depth
		if(a->value >= - 10 && a->value <= 10)
		{
			mutex_lock(&vout->lock);		
			vout->display_info.depth_factor = a->value;
			mutex_unlock(&vout->lock);
		}
		else
			return -EINVAL;
		break;
		
	default:
		ret = -EINVAL;
	}
	return ret;
}

static int vidioc_reqbufs(struct file *file, void *fh,
			struct v4l2_requestbuffers *req)
{
	int ret = 0;
	unsigned int i;
	struct omap_vout_device *vout = fh;
	struct videobuf_queue *q = &vout->queing_info.vbq;

	if ((req->type != V4L2_BUF_TYPE_VIDEO_OUTPUT) || (req->count < 0))
		return -EINVAL;
	/* if memory is not mmp or userptr
	   return error */
	if ((V4L2_MEMORY_MMAP != req->memory) &&
			(V4L2_MEMORY_USERPTR != req->memory))
		return -EINVAL;

	mutex_lock(&vout->lock);
	/* Cannot be requested when streaming is on */
	if (vout->streaming_status!=E_STREAMING_STOPPED) {
		ret = -EBUSY;
		goto reqbuf_err;
	}

	/* If buffers are already allocated free them */
	if (q->bufs[0] && (V4L2_MEMORY_MMAP == q->bufs[0]->memory)) {
		if (vout->buf_info.mmap_count) {
			ret = -EBUSY;
			goto reqbuf_err;
		}
		omap_vout_tiler_buffer_free(vout, vout->buf_info.buffer_allocated, 0);
		vout->buf_info.buffer_allocated = 0;
		videobuf_mmap_free(q);
	} else if (q->bufs[0] && (V4L2_MEMORY_USERPTR == q->bufs[0]->memory)) {
		if (vout->buf_info.buffer_allocated) {
			videobuf_mmap_free(q);
			for (i = 0; i < vout->buf_info.buffer_allocated; i++) {
				kfree(q->bufs[i]);
				q->bufs[i] = NULL;
			}
			vout->buf_info.buffer_allocated = 0;
		}
	}

	/*store the memory type in data structure */
	vout->buf_info.memory = req->memory;

	INIT_LIST_HEAD(&vout->queing_info.drawing_req_queue);

	/* call videobuf_reqbufs api */
	ret = videobuf_reqbufs(q, req);
	if (ret < 0)
		goto reqbuf_err;

	vout->buf_info.buffer_allocated = req->count;
reqbuf_err:
	mutex_unlock(&vout->lock);
	return ret;
}

static int vidioc_querybuf(struct file *file, void *fh,
			struct v4l2_buffer *b)
{
	struct omap_vout_device *vout = fh;

	return videobuf_querybuf(&vout->queing_info.vbq, b);
}

static int vidioc_qbuf(struct file *file, void *fh,
			struct v4l2_buffer *buffer)
{
	struct omap_vout_device *vout = fh;
	struct videobuf_queue *q = &vout->queing_info.vbq;
	int ret = -EINVAL;

	mutex_lock(&vout->lock);

	if ((V4L2_BUF_TYPE_VIDEO_OUTPUT != buffer->type) ||
			(buffer->index >= vout->buf_info.buffer_allocated) ||
			(q->bufs[buffer->index]->memory != buffer->memory))
		goto err;
	if (V4L2_MEMORY_USERPTR == buffer->memory) {
		if ((buffer->length < vout->input_info.pix.sizeimage) ||
				(0 == buffer->m.userptr))
			goto err;
	}

	DBG_PRINTK("Buffer %d queue\n", buffer->index);
	ret = videobuf_qbuf(q, buffer);

	// LGE_CHANGE [Darren.kang@lge.com] for handling exception
	if(ret != 0)
		goto err;

	//push queued frames to work queue if stream on
	if ( !list_empty(&vout->queing_info.drawing_req_queue) )
	{
		struct list_head *pos, *n;
		list_for_each_safe(pos, n, &vout->queing_info.drawing_req_queue)
		{
			struct omap_vout_display_work *work;
			struct videobuf_buffer *frame;
			work = kzalloc(sizeof(*work), GFP_ATOMIC);
			if( work==NULL )
			{
				ret = -ENOMEM;
				goto err;
			}
			frame = list_entry(pos, struct videobuf_buffer, queue);
			list_del(pos);

			work->frame = frame;
			work->lastframe_update = false;
			work->vout = vout;
			INIT_LIST_HEAD(pos);

			INIT_WORK(&work->work, omap_vout_display);
			queue_work(vout->workqueue, &work->work);

		}
	}
	else
		DBG_PRINTK("Queue is empty. Nothing to display now\n");

err:
	mutex_unlock(&vout->lock);

	return ret;
}


static int vidioc_dqbuf(struct file *file, void *fh, struct v4l2_buffer *b)
{
	int ret;
	struct omap_vout_device *vout = fh;
	struct videobuf_queue *q = &vout->queing_info.vbq;
	ret = videobuf_dqbuf(q, (struct v4l2_buffer *)b, file->f_flags & O_NONBLOCK);
	DBG_PRINTK("vidioc_dqbuf() (%d) (%d)\n", b->index, ret );
	return ret;
}

static void vidioc_disable_layers_locked(struct omapvideo_info *vid_info)
{
	int i;
	struct omap_overlay_info info;
	struct omap_overlay *ovl;
	if ( vid_info==NULL )
		return;
	DBG_PRINTK("disable overlay start\n");
	for(i=0;i<vid_info->num_overlays;i++)
	{
		ovl = vid_info->overlays.overlays[i];
		if ( ovl->get_overlay_info && ovl->set_overlay_info )
		{
			bool layer_for_wb = false;
			ovl->get_overlay_info(ovl, &info);
			if ( info.enabled==false )
				continue;
			info.enabled = false;
			layer_for_wb = info.out_wb;
			if ( ovl->set_overlay_info(ovl, &info) )
				ERR_PRINTK("Overlay %d disable failed\n", i);

			if ( ovl->manager )
			{
				if ( layer_for_wb )
				{
					//if layer is used for write back
					struct omap_writeback *wb;
					wb = omap_dss_get_wb(0);
					if ( wb!=NULL )
					{
						DBG_PRINTK("Disabling Write Back Overlay%d\n",ovl->id);
						if ( omap_dss_wb_apply(ovl->manager, wb) )
							ERR_PRINTK("WriteBack apply(to disable) failed\n");
					}
					else
						ERR_PRINTK("Write back enabled in overlay info, but WB get failed\n");
				}
				else
				{
					if ( ovl->manager->apply )
					{
						DBG_PRINTK("Disabling overlay %d\n", ovl->id);
						if ( ovl->manager->apply(ovl->manager) )
							ERR_PRINTK("Applying overlay disable to manager failed\n");
					}
				}
			}	//manager apply disable
		} //disable one overlay end
	} //end of for
	DBG_PRINTK("disable overlay end\n");
}

#ifdef CONFIG_HAS_EARLYSUSPEND

static void vidioc_drawing_suspend(struct early_suspend *h)
{
	struct omap_vout_device *vout;
	vout = container_of(h, struct omap_vout_device, dawing_stop_suspend_handler);
	DBG_PRINTK("Stop Drawing early suspend in vout\n");
	mutex_lock(&vout->lock);
	if ( vout->streaming_status==E_STREAMING_ON_GOING)
		vout->streaming_status = E_STREAMING_NO_DRAW;
	mutex_unlock(&vout->lock);
}

static void vidioc_drawing_resume(struct early_suspend *h)
{
	struct omap_vout_device *vout;
	vout = container_of(h, struct omap_vout_device, dawing_stop_suspend_handler);
	DBG_PRINTK("Stop Drawing late resume in vout\n");
	mutex_lock(&vout->lock);
	if ( vout->streaming_status==E_STREAMING_NO_DRAW)
		vout->streaming_status=E_STREAMING_ON_GOING;
	mutex_unlock(&vout->lock);
}

static void vidioc_disable_suspend(struct early_suspend *h)
{
	struct omap_vout_device *vout;
	vout = container_of(h, struct omap_vout_device, fb_disable_suspend_handler);
	DBG_PRINTK("Disable FB early suspend in vout\n");
	mutex_lock(&vout->in_display_operation);
	mutex_lock(&vout->lock);
	//disable overlays
	vidioc_disable_layers_locked(&vout->vid_info);
	mutex_unlock(&vout->lock);
	mutex_unlock(&vout->in_display_operation);
}

static void vidioc_disable_resume(struct early_suspend *h)
{
	//no need to restore overlay status
	//Because Display work queue will set overly info
	DBG_PRINTK("Disable FB late resume in vout\n");
}

#endif

static int vidioc_streamon(struct file *file, void *fh, enum v4l2_buf_type i)
{
	int ret = 0;
	struct omap_vout_device *vout = fh;
	struct videobuf_queue *q = &vout->queing_info.vbq;
#ifdef CONFIG_PM
	struct vout_platform_data *pdata =
		(((vout->vid_dev)->v4l2_dev).dev)->platform_data;
#endif

	DBG_PRINTK("Stream On\n");
	mutex_lock(&vout->lock);

	if (vout->streaming_status!=E_STREAMING_STOPPED ) {
		ret = -EBUSY;
		ERR_PRINTK("Stream already on.\n");
		goto streamon_err;
	}
	if ( vout->queing_info.displaying_frame!=NULL )
	{
		ERR_PRINTK("Frame reamains in previous streaming when stream on\n");
		vout->queing_info.displaying_frame->state = VIDEOBUF_ERROR;
		vout->queing_info.displaying_frame = NULL;
	}
	if ( !list_empty(&vout->queing_info.drawing_req_queue) )
	{
		ERR_PRINTK("Drawing request queue is not empty\n");
		INIT_LIST_HEAD(&vout->queing_info.drawing_req_queue);
	}

	//WB buffer put
	{
		int i;
		for(i=0;i<2;i++)
		{
			if ( vout->display_info.lcd.wb_buffer[i]!=NULL )
			{
				omap_vout_vram_buffer_manager_put(&vout->buffer_manager, vout->display_info.lcd.wb_buffer[i]);
				vout->display_info.lcd.wb_buffer[i] = NULL;
			}
		}
		vout->display_info.lcd.wb_buffer_cursor = 0;

		if ( vout->display_info.lcd.wb_pre_buffer!=NULL )
		{
			omap_vout_vram_buffer_manager_put(&vout->buffer_manager, vout->display_info.lcd.wb_pre_buffer);
			vout->display_info.lcd.wb_pre_buffer = NULL;				
		}
	}

	ret = videobuf_streamon(q);
	if (ret)
	{
		ERR_PRINTK("videobuf_streamon() failed\n");
		goto streamon_err;
	}

	vout->streaming_status = E_STREAMING_ON_GOING;


#ifdef CONFIG_PM
	if (pdata->set_min_bus_tput) {
		if (cpu_is_omap3630() || cpu_is_omap44xx()) {
			pdata->set_min_bus_tput(
				((vout->vid_dev)->v4l2_dev).dev ,
					OCP_INITIATOR_AGENT, 200 * 1000 * 4);
		} else {
			pdata->set_min_bus_tput(
				((vout->vid_dev)->v4l2_dev).dev ,
					OCP_INITIATOR_AGENT, 166 * 1000 * 4);
		}
	}
#endif

	ret = 0;

streamon_err:
	if (ret)
	{
		ERR_PRINTK("Stream on failed\n");
		videobuf_streamoff(q);
	}
	else
		DBG_PRINTK("Strea on Success\n");
	mutex_unlock(&vout->lock);
	return ret;
}


static int vidioc_streamoff(struct file *file, void *fh, enum v4l2_buf_type i)
{
	int ret = 0;
	struct omap_vout_device *vout = fh;
#ifdef CONFIG_PM
	struct vout_platform_data *pdata;
#endif
	if ( vout==NULL )
		return -EINVAL;
#ifdef CONFIG_PM
	pdata =(((vout->vid_dev)->v4l2_dev).dev)->platform_data;
#endif

	INFO_PRINTK("v4l2 stream off\n");
	//To stop work queue
	mutex_lock(&vout->lock);
	if ( vout->streaming_status==E_STREAMING_STOPPED )
	{
		mutex_unlock(&vout->lock);
		DBG_PRINTK("Already stream offed \n");
		return 0;
	}
	vout->streaming_status = E_STREAMING_STOPPING;
	mutex_unlock(&vout->lock);

	//flush work queue
	DBG_PRINTK("Waiting workqueue flush\n");
	flush_workqueue(vout->workqueue);
	DBG_PRINTK("workqueue flushed\n");

	DBG_PRINTK("Stream Off before get mutex\n");
	mutex_lock(&vout->lock);
	DBG_PRINTK("Stream Off get mutex and work start\n");


	vout->streaming_status = E_STREAMING_STOPPED;
#ifdef CONFIG_PM
	if (pdata->set_min_bus_tput)
		pdata->set_min_bus_tput(
			((vout->vid_dev)->v4l2_dev).dev,
				OCP_INITIATOR_AGENT, 0);
#endif



	//Displaying Frame dequeue
	if ( vout->queing_info.displaying_frame!=NULL )
	{
		struct videobuf_buffer *frame = vout->queing_info.displaying_frame;
		frame->state = VIDEOBUF_DONE;
		wake_up(&frame->done);
		vout->queing_info.displaying_frame=NULL;
	}

	//Empty drawing request list
	if ( !list_empty(&vout->queing_info.drawing_req_queue) )
	{
		struct list_head *pos, *n;
		DBG_PRINTK("Drawing request list is not empty\n");
		list_for_each_safe(pos, n, &vout->queing_info.drawing_req_queue)
		{
			list_del(pos);
			INIT_LIST_HEAD(pos);
		}
		INIT_LIST_HEAD(&vout->queing_info.drawing_req_queue);
	}

	ret = videobuf_streamoff(&vout->queing_info.vbq);

	DBG_PRINTK("Remove drawing request queue\n");
	//WB buffer put
	{
		int i;
		for(i=0;i<2;i++)
		{
			if ( vout->display_info.lcd.wb_buffer[i]!=NULL )
			{
				omap_vout_vram_buffer_manager_put(&vout->buffer_manager, vout->display_info.lcd.wb_buffer[i]);
				vout->display_info.lcd.wb_buffer[i] = NULL;
			}
		}
		vout->display_info.lcd.wb_buffer_cursor = 0;

		if ( vout->display_info.lcd.wb_pre_buffer!=NULL )
		{
			omap_vout_vram_buffer_manager_put(&vout->buffer_manager, vout->display_info.lcd.wb_pre_buffer);
			vout->display_info.lcd.wb_pre_buffer = NULL;
		}
	}

	if ( vout->display_info.hdmi.enable )
	{
		//disable overlay
		INFO_PRINTK("Stream off and HDMI on. stop HDMI\n");
		hdmi_video_prepare_change(vout, 0, false, "V4L2 stream off");
		vidioc_disable_layers_locked(&vout->vid_info);
		hdmi_video_commit_change(vout);
	}
	else
	{
		//disable overlay
		INFO_PRINTK("Stream off disable overlays\n");
		vidioc_disable_layers_locked(&vout->vid_info);
	}

	mutex_unlock(&vout->lock);

	DBG_PRINTK("Stream Off completed\n");
	return ret;
}

static int vidioc_s_fbuf(struct file *file, void *fh,
				struct v4l2_framebuffer *a)
{
	struct omapvideo_info *ovid;
	struct omap_vout_device *vout = fh;
	int ret = 0;

	mutex_lock(&vout->lock);
	ovid = &vout->vid_info;

	/* OMAP DSS doesn't support Source and Destination color
	   key together */
	if ((a->flags & V4L2_FBUF_FLAG_SRC_CHROMAKEY) &&
			(a->flags & V4L2_FBUF_FLAG_CHROMAKEY))
	{
		ret = -EINVAL;
		goto EXIT;
	}
	/* OMAP DSS Doesn't support the Destination color key
	   and alpha blending together */
	if ((a->flags & V4L2_FBUF_FLAG_CHROMAKEY) &&
			(a->flags & V4L2_FBUF_FLAG_LOCAL_ALPHA))
	{
		ret = -EINVAL;
		goto EXIT;
	}

	//set attributes
	if ((a->flags & V4L2_FBUF_FLAG_SRC_CHROMAKEY)) {
		vout->buf_info.fbuf.flags |= V4L2_FBUF_FLAG_SRC_CHROMAKEY;
	} else
		vout->buf_info.fbuf.flags &= ~V4L2_FBUF_FLAG_SRC_CHROMAKEY;

	if ((a->flags & V4L2_FBUF_FLAG_CHROMAKEY)) {
		vout->buf_info.fbuf.flags |= V4L2_FBUF_FLAG_CHROMAKEY;
	} else
		vout->buf_info.fbuf.flags &=  ~V4L2_FBUF_FLAG_CHROMAKEY;
	if (a->flags & V4L2_FBUF_FLAG_LOCAL_ALPHA) {
		vout->buf_info.fbuf.flags |= V4L2_FBUF_FLAG_LOCAL_ALPHA;
	} else {
		vout->buf_info.fbuf.flags &= ~V4L2_FBUF_FLAG_LOCAL_ALPHA;
	}
EXIT:
	mutex_unlock(&vout->lock);
	return ret;
}

static int vidioc_g_fbuf(struct file *file, void *fh,
		struct v4l2_framebuffer *a)
{
	struct omap_vout_device *vout = fh;

	mutex_lock(&vout->lock);

	a->flags = vout->buf_info.fbuf.flags;
	a->capability = V4L2_FBUF_CAP_LOCAL_ALPHA | V4L2_FBUF_CAP_CHROMAKEY
		| V4L2_FBUF_CAP_SRC_CHROMAKEY;

	mutex_unlock(&vout->lock);

	return 0;
}

static const struct v4l2_ioctl_ops vout_ioctl_ops = {
	.vidioc_querycap      			= vidioc_querycap,

	.vidioc_enum_fmt_vid_out 		= vidioc_enum_fmt_vid_out,
	.vidioc_g_fmt_vid_out			= vidioc_g_fmt_vid_out,
	.vidioc_try_fmt_vid_out			= vidioc_try_fmt_vid_out,
	.vidioc_s_fmt_vid_out			= vidioc_s_fmt_vid_out,

	.vidioc_queryctrl    			= vidioc_queryctrl,
	.vidioc_g_ctrl       			= vidioc_g_ctrl,

	.vidioc_s_fbuf					= vidioc_s_fbuf,
	.vidioc_g_fbuf					= vidioc_g_fbuf,

	.vidioc_s_ctrl       			= vidioc_s_ctrl,

	.vidioc_try_fmt_vid_overlay 	= vidioc_try_fmt_vid_overlay,
	.vidioc_s_fmt_vid_overlay		= vidioc_s_fmt_vid_overlay,
	.vidioc_enum_fmt_vid_overlay	= vidioc_enum_fmt_vid_overlay,
	.vidioc_g_fmt_vid_overlay		= vidioc_g_fmt_vid_overlay,

	.vidioc_s_fmt_type_private		= vidioc_s_fmt_type_private,
	.vidioc_g_fmt_type_private		= vidioc_g_fmt_type_private,

	.vidioc_cropcap					= vidioc_cropcap,
	.vidioc_g_crop					= vidioc_g_crop,
	.vidioc_s_crop					= vidioc_s_crop,

	.vidioc_reqbufs					= vidioc_reqbufs,
	.vidioc_querybuf				= vidioc_querybuf,

	.vidioc_qbuf					= vidioc_qbuf,
	.vidioc_dqbuf					= vidioc_dqbuf,
	.vidioc_streamon				= vidioc_streamon,
	.vidioc_streamoff				= vidioc_streamoff,
};

static const struct v4l2_file_operations omap_vout_fops = {
	.owner 		= THIS_MODULE,
	.unlocked_ioctl	= video_ioctl2,
	.mmap 		= omap_vout_mmap,
	.poll		= omap_vout_poll,
	.open 		= omap_vout_open,
	.release 	= omap_vout_release,
};

/* Init functions used during driver initialization */
/* Initial setup of video_data */
static int __init omap_vout_setup_video_data(struct omap_vout_device *vout)
{
	struct video_device *vfd;
	struct v4l2_pix_format *pix;

	/* set the default pix */
	pix = &vout->input_info.pix;

	/* Set the default picture of QVGA  */
	pix->width = QQVGA_WIDTH;
	pix->height = QQVGA_HEIGHT;

	/* Default pixel format is RGB 5-6-5 */
	pix->pixelformat = V4L2_PIX_FMT_RGB565;
	pix->field = V4L2_FIELD_ANY;
	pix->bytesperline = pix->width * 2;
	pix->sizeimage = pix->bytesperline * pix->height;
	pix->priv = 0;
	pix->colorspace = V4L2_COLORSPACE_JPEG;

	vout->input_info.bpp = RGB565_BPP;
	vout->buf_info.fbuf.fmt.width  =  QQVGA_WIDTH;
	vout->buf_info.fbuf.fmt.height =  QQVGA_HEIGHT;

	/* Set the data structures for the overlay parameters*/
	vout->display_info.lcd.win.global_alpha = 255;
	vout->buf_info.fbuf.flags = 0;
	vout->buf_info.fbuf.capability = V4L2_FBUF_CAP_LOCAL_ALPHA |
		V4L2_FBUF_CAP_SRC_CHROMAKEY | V4L2_FBUF_CAP_CHROMAKEY;
	vout->display_info.lcd.win.chromakey = 0;

	omap_vout_new_format(pix, &vout->buf_info.fbuf, &vout->input_info.crop, &vout->display_info.lcd.win);


	/* initialize the video_device struct */
	vfd = vout->vfd = video_device_alloc();

	if (!vfd) {
		printk(KERN_ERR VOUT_NAME ": could not allocate"
				" video device struct\n");
		return -ENOMEM;
	}
	vfd->release = video_device_release;
	vfd->ioctl_ops = &vout_ioctl_ops;

	strlcpy(vfd->name, VOUT_NAME, sizeof(vfd->name));

	/* need to register for a VID_HARDWARE_* ID in videodev.h */
	vfd->fops = &omap_vout_fops;
	vfd->v4l2_dev = &vout->vid_dev->v4l2_dev;
	mutex_init(&vout->lock);
	mutex_init(&vout->in_display_operation);

	//buffer manager init
	omap_vout_vram_buffer_manager_init(&vout->buffer_manager);

	// prepare black line buffer for depth shifting
	vout->display_info.lcd.black_line = kmalloc(COSMO_LCD_WIDTH*2, GFP_KERNEL);
	if ( vout->display_info.lcd.black_line==NULL )
	{
		printk(KERN_ERR VOUT_NAME ":black line alloc fail\n");
		return -ENOMEM;
	}

	{
		u32 *fill_line = (u32*) vout->display_info.lcd.black_line;
		int i = COSMO_LCD_WIDTH/2;
		do
		{
			*(fill_line++) = 0x00800080; // UYVY black color data
		}
		while(--i > 0);
	}
	vfd->minor = -1;
	return 0;

}


/* Create video out devices */
static int __init omap_vout_create_video_devices(struct platform_device *pdev)
{
	int ret = 0, k=0;
	struct omap_vout_device *vout;
	struct video_device *vfd = NULL;
	struct v4l2_device *v4l2_dev = platform_get_drvdata(pdev);
	struct omap2video_device *vid_dev = container_of(v4l2_dev,
			struct omap2video_device, v4l2_dev);

	vout = kzalloc(sizeof(struct omap_vout_device), GFP_KERNEL);
	if (!vout) {
		dev_err(&pdev->dev, ": could not allocate memory\n");
		return -ENOMEM;
	}

	vout->vid = k;
	vid_dev->vouts[0] = vout;
	vout->vid_dev = vid_dev;

	//setup overlays
	vout->vid_info.overlays.named.lcd  = vid_dev->overlays[2];	//lcd  -> overlay2
	vout->vid_info.overlays.named.hdmi = vid_dev->overlays[3];	//hdmi -> overlay3
	vout->vid_info.num_overlays = 2;

	vout->workqueue = create_singlethread_workqueue("OMAPVOUT");
	if (vout->workqueue == NULL) {
		ret = -ENOMEM;
		goto error;
	}

	/* Setup the default configuration for the video devices
	 */
	if (omap_vout_setup_video_data(vout) != 0) {
		ret = -ENOMEM;
		goto error_q;
	}

	/* Register the Video device with V4L2
	 */
	vfd = vout->vfd;
	if (video_register_device(vfd, VFL_TYPE_GRABBER, k + 1) < 0) {
		dev_err(&pdev->dev, ": Could not register "
				"Video for Linux device\n");
		vfd->minor = -1;
		ret = -ENODEV;
		goto error2;
	}
	video_set_drvdata(vfd, vout);

	if (!ret)
		goto success;

error2:
	omap_vout_free_buffers(vout);
//error1:
	video_device_release(vfd);
error_q:
	destroy_workqueue(vout->workqueue);
error:
	kfree(vout);
	return ret;

success:
#ifdef CONFIG_HAS_EARLYSUSPEND
	//register stop drawing early suspend handler
	vout->dawing_stop_suspend_handler.suspend	= vidioc_drawing_suspend;
	vout->dawing_stop_suspend_handler.resume	= vidioc_drawing_resume;
	vout->dawing_stop_suspend_handler.level		= EARLY_SUSPEND_LEVEL_STOP_DRAWING;
	register_early_suspend(&vout->dawing_stop_suspend_handler);
	DBG_PRINTK("Drawing Stop Early handler registered\n");
	vout->fb_disable_suspend_handler.suspend	= vidioc_disable_suspend;
	vout->fb_disable_suspend_handler.resume		= vidioc_disable_resume;
	vout->fb_disable_suspend_handler.level		= EARLY_SUSPEND_LEVEL_DISABLE_FB - 1;
	register_early_suspend(&vout->fb_disable_suspend_handler);
	DBG_PRINTK("FB Dsiable Early handler registered\n");
#endif
	dev_info(&pdev->dev, ": registered and initialized"
			" video device %d\n", vfd->minor);
	return 0;
}

/* Driver functions */
static void omap_vout_cleanup_device(struct omap_vout_device *vout)
{
	struct video_device *vfd;

	if (!vout)
		return;

	vfd = vout->vfd;
	if (vfd) {
		if (!video_is_registered(vfd)) {
			/*
			 * The device was never registered, so release the
			 * video_device struct directly.
			 */
			video_device_release(vfd);
		} else {
			/*
			 * The unregister function will release the video_device
			 * struct as well as unregistering it.
			 */
			video_unregister_device(vfd);
		}
	}
	
#ifdef CONFIG_HAS_EARLYSUSPEND
	DBG_PRINTK("Early suspend handler unregister start\n");
	unregister_early_suspend(&vout->dawing_stop_suspend_handler);
	unregister_early_suspend(&vout->fb_disable_suspend_handler);
	DBG_PRINTK("Early suspend handler unregister complete\n");
#endif
	if(vout->display_info.lcd.black_line != NULL)
		kfree(vout->display_info.lcd.black_line);

	vout->display_info.lcd.black_line = NULL;

	omap_vout_free_buffers(vout);
	omap_vout_free_tiler_buffers(vout);

	flush_workqueue(vout->workqueue);
	destroy_workqueue(vout->workqueue);

	//de-init buffer manager
	omap_vout_vram_buffer_manager_deinit(&vout->buffer_manager);
	mutex_destroy(&vout->in_display_operation);
	mutex_destroy(&vout->lock);
	kfree(vout);
}



static int omap_vout_remove(struct platform_device *pdev)
{
	int k;
	struct v4l2_device *v4l2_dev = platform_get_drvdata(pdev);
	struct omap2video_device *vid_dev = container_of(v4l2_dev, struct
			omap2video_device, v4l2_dev);


	v4l2_device_unregister(v4l2_dev);
	for (k = 0; k < pdev->num_resources; k++)
		omap_vout_cleanup_device(vid_dev->vouts[k]);

	for (k = 0; k < vid_dev->num_displays; k++) {
		if (vid_dev->displays[k]->state != OMAP_DSS_DISPLAY_DISABLED)
			omapdss_display_disable(vid_dev->displays[k]);

		omap_dss_put_device(vid_dev->displays[k]);
	}
	kfree(vid_dev);
	return 0;
}

static int __init omap_vout_probe(struct platform_device *pdev)
{
	int ret = 0, i;
	struct omap_overlay *ovl;
	struct omap_dss_device *dssdev = NULL;
	struct omap_dss_device *def_display;
	struct omap2video_device *vid_dev = NULL;

	if (pdev->num_resources == 0) {
		dev_err(&pdev->dev, "probed for an unknown device\n");
		return -ENODEV;
	}

	vid_dev = kzalloc(sizeof(struct omap2video_device), GFP_KERNEL);
	if (vid_dev == NULL)
		return -ENOMEM;

	vid_dev->num_displays = 0;
	for_each_dss_dev(dssdev) {
		omap_dss_get_device(dssdev);
		vid_dev->displays[vid_dev->num_displays++] = dssdev;
	}

	if (vid_dev->num_displays == 0) {
		dev_err(&pdev->dev, "no displays\n");
		ret = -EINVAL;
		goto probe_err0;
	}

	vid_dev->num_overlays = omap_dss_get_num_overlays();
	for (i = 0; i < vid_dev->num_overlays; i++)
		vid_dev->overlays[i] = omap_dss_get_overlay(i);

	vid_dev->num_managers = omap_dss_get_num_overlay_managers();
	for (i = 0; i < vid_dev->num_managers; i++)
		vid_dev->managers[i] = omap_dss_get_overlay_manager(i);

	/* Get the Video1 overlay and video2 overlay.
	 * Setup the Display attached to that overlays
	 */
	for (i = 1; i < vid_dev->num_overlays; i++) {
		ovl = omap_dss_get_overlay(i);
		if (ovl->manager && ovl->manager->device) {
			def_display = ovl->manager->device;
		} else {
			dev_warn(&pdev->dev, "cannot find display\n");
			def_display = NULL;
		}
		if (def_display) {
			struct omap_dss_driver *dssdrv = def_display->driver;

			ret = omapdss_display_enable(def_display);
			if (ret) {
				dev_err(&pdev->dev,
					"Failed to enable '%s' display\n",
					def_display->name);
			}
			/* set the update mode */
			if (def_display->caps &
					OMAP_DSS_DISPLAY_CAP_MANUAL_UPDATE) {
#ifdef CONFIG_FB_OMAP2_FORCE_AUTO_UPDATE
				if (dssdrv->enable_te)
					dssdrv->enable_te(def_display, 1);
				if (dssdrv->set_update_mode)
					dssdrv->set_update_mode(def_display,
							OMAP_DSS_UPDATE_AUTO);
#else	/* MANUAL_UPDATE */
				if (dssdrv->enable_te)
					dssdrv->enable_te(def_display, 1);
				if (dssdrv->set_update_mode)
					dssdrv->set_update_mode(def_display,
							OMAP_DSS_UPDATE_MANUAL);
#endif
			} else {
				if (dssdrv->set_update_mode)
					dssdrv->set_update_mode(def_display,
							OMAP_DSS_UPDATE_AUTO);
			}
		}
	}

	if (v4l2_device_register(&pdev->dev, &vid_dev->v4l2_dev) < 0) {
		dev_err(&pdev->dev, "v4l2_device_register failed\n");
		ret = -ENODEV;
		goto probe_err1;
	}

	ret = omap_vout_create_video_devices(pdev);
	if (ret)
		goto probe_err2;

	for (i = 0; i < vid_dev->num_displays; i++) {
		struct omap_dss_device *display = vid_dev->displays[i];
		struct omap_dss_driver *dssdrv = display->driver;

		if (dssdrv->get_update_mode &&
			OMAP_DSS_UPDATE_MANUAL == dssdrv->get_update_mode(display))
			if (display->driver->update)
				display->driver->update(display, 0, 0,
						display->panel.timings.x_res,
						display->panel.timings.y_res);
	}
	return 0;

probe_err2:
	v4l2_device_unregister(&vid_dev->v4l2_dev);
probe_err1:
	for (i = 1; i < vid_dev->num_overlays; i++) {
		def_display = NULL;
		ovl = omap_dss_get_overlay(i);
		if (ovl->manager && ovl->manager->device)
			def_display = ovl->manager->device;

		if (def_display && def_display->driver)
			omapdss_display_disable(def_display);
	}
probe_err0:
	kfree(vid_dev);
	return ret;
}


static struct platform_driver omap_vout_driver = {
	.driver = {
		.name = VOUT_NAME,
	},
	.probe = omap_vout_probe,
	.remove = omap_vout_remove,
};

static int __init omap_vout_init(void)
{
	if (platform_driver_register(&omap_vout_driver) != 0) {
		printk(KERN_ERR VOUT_NAME ":Could not register Video driver\n");
		return -EINVAL;
	}


	return 0;
}

static void omap_vout_cleanup(void)
{
	platform_driver_unregister(&omap_vout_driver);
}

late_initcall(omap_vout_init);
module_exit(omap_vout_cleanup);


//-----------------------------------------------------------------------------------
//	VRAM Buffer Manager


/**
 * Free buffer[index]
 * this function must be called in locking status
 */
static void omap_vout_vram_buffer_manager_free_locked(struct omap_vout_vram_buffer_manager *mgr, int index)
{
	struct omap_vout_vram_buffer *buffer;
	if ( mgr==NULL )
		return;
	if ( index<0 || index>=ARRAY_SIZE(mgr->buffers) )
		return;
	buffer = &mgr->buffers[index];
	if (buffer->paddr!=0 )
	{
		if ( buffer->vaddr!=NULL )
			iounmap(buffer->vaddr);
		omap_vram_free(buffer->paddr, buffer->alloc_size);
		buffer->paddr = 0;
		buffer->alloc_size = 0;
	}
}

/**
 * Initialize buffer manager
 */
static int	omap_vout_vram_buffer_manager_init(struct omap_vout_vram_buffer_manager *mgr)
{
	int i;
	if ( mgr==NULL )
		return -EINVAL;
	mutex_init(&mgr->lock);
	for(i=0;i<ARRAY_SIZE(mgr->buffers);i++)
	{
		mgr->buffers[i].paddr = 0;
		mgr->buffers[i].vaddr = 0;
		mgr->buffers[i].alloc_size = 0;
		mgr->occupied[i] = false;
	}
	return 0;
}

/**
 * De-Init Buffer manager
 */
static void	omap_vout_vram_buffer_manager_deinit(struct omap_vout_vram_buffer_manager *mgr)
{
	int i;
	if ( mgr==NULL )
		return;
	mutex_lock(&mgr->lock);
	for(i=0;i<ARRAY_SIZE(mgr->buffers);i++)
		omap_vout_vram_buffer_manager_free_locked(mgr, i);
	mutex_unlock(&mgr->lock);
	mutex_destroy(&mgr->lock);
	return;
}

/**
 * Get buffer
 */
static int	omap_vout_vram_buffer_manager_get(struct omap_vout_vram_buffer_manager *mgr, unsigned long size, struct omap_vout_vram_buffer **buffer)
{
	int i;
	int ret = 0;
	int selected_index = -1;

	if ( mgr==NULL || buffer==NULL )
		return -EINVAL;

	mutex_lock(&mgr->lock);
	//first find allocated and not occupied
	for(i=0;i<ARRAY_SIZE(mgr->buffers);i++)
	{
		if ( !mgr->occupied[i] && mgr->buffers[i].alloc_size >=size )
		{
			selected_index = i;
			goto Success;
		}
	}

	//second find victim and realloc (For minimize use of VRAM)
	for(i=0;i<ARRAY_SIZE(mgr->buffers);i++)
		if ( !mgr->occupied[i] )
		{
			omap_vout_vram_buffer_manager_free_locked(mgr, i);
			selected_index = i;
			goto Alloc;
		}

	//third find not occupied and alloc
	for(i=0;i<ARRAY_SIZE(mgr->buffers);i++)
	{
		if ( !mgr->occupied[i] && mgr->buffers[i].alloc_size==0 )
		{
			selected_index = i;
			goto Alloc;
		}
	}


Alloc:
	if ( selected_index<0 || selected_index>=ARRAY_SIZE(mgr->buffers) )
	{
		WARN(1, "Selected index is wrong\n");
		ret = -EFAULT;
		goto Error;
	}
	if ( omap_vram_alloc(OMAP_VRAM_MEMTYPE_SDRAM, size, &mgr->buffers[selected_index].paddr) )
	{
		WARN(1, "VRMA alloc failed");
		ret = -ENOMEM;
		goto Error;
	}
	mgr->buffers[selected_index].vaddr = ioremap_wc(mgr->buffers[selected_index].paddr, size);
	mgr->buffers[selected_index].alloc_size = size;

Success:
	if ( selected_index<0 || selected_index>=ARRAY_SIZE(mgr->buffers) )
	{
		WARN(1, "Selected index is wrong\n");
		ret = -EFAULT;
	}
	else
	{
		*buffer = &mgr->buffers[selected_index];
		mgr->occupied[selected_index] = true;
		DBG_PRINTK("Vram manager %d(th) buffer is occupied\n", selected_index);
	}
	mutex_unlock(&mgr->lock);
	return ret;

Error:
	mutex_unlock(&mgr->lock);
	return ret;

}

/**
 * Return Buffer
 */
static int	omap_vout_vram_buffer_manager_put(struct omap_vout_vram_buffer_manager *mgr, struct omap_vout_vram_buffer *buffer)
{
	int i;
	if ( mgr==NULL || buffer==NULL )
		return -EINVAL;

	mutex_lock(&mgr->lock);
	for(i=0;i<ARRAY_SIZE(mgr->buffers);i++)
	{
		if ( buffer==&(mgr->buffers[i]) && mgr->occupied[i] )
		{
			DBG_PRINTK("Vram manager %d(th) buffer is returned\n", i);
			mgr->occupied[i] = false;
			mutex_unlock(&mgr->lock);
			return 0;
		}
	}
	mutex_unlock(&mgr->lock);
	return -EINVAL;
}
