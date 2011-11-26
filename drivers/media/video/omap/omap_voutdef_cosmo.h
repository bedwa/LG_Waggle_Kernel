/*
 * omap_voutdef.h
 *
 * Copyright (C) 2010 Texas Instruments.
 *
 * This file is licensed under the terms of the GNU General Public License
 * version 2. This program is licensed "as is" without any warranty of any
 * kind, whether express or implied.
 */

#ifndef OMAP_VOUTDEF_H
#define OMAP_VOUTDEF_H

#include <plat/display.h>

#ifdef CONFIG_HAS_EARLYSUSPEND
#include <linux/earlysuspend.h>
#endif

#define YUYV_BPP        2
#define RGB565_BPP      2
#define RGB24_BPP       3
#define RGB32_BPP       4
#define TILE_SIZE       32
#define YUYV_VRFB_BPP   2
#define RGB_VRFB_BPP    1
#define MAX_CID		3
#define MAC_VRFB_CTXS	4
#ifdef CONFIG_ARCH_OMAP4
#define MAX_VOUT_DEV	3
#define MAX_OVLS	4
#define MAX_DISPLAYS	4
#else
#define MAX_VOUT_DEV	2
#define MAX_OVLS	3
#define MAX_DISPLAYS	3
#endif
#define MAX_MANAGERS	3

/* TI Private V4L2 ioctls */
#define V4L2_CID_TI_DISPC_OVERLAY	(V4L2_CID_PRIVATE_BASE + 0)

/* Enum for Rotation
 * DSS understands rotation in 0, 1, 2, 3 context
 * while V4L2 driver understands it as 0, 90, 180, 270
 */
enum dss_rotation {
	dss_rotation_0_degree	= 0,
	dss_rotation_90_degree	= 1,
	dss_rotation_180_degree	= 2,
	dss_rotation_270_degree = 3,
};
/*
 * This structure is used to store the DMA transfer parameters
 * for VRFB hidden buffer
 */
struct vid_vrfb_dma {
	int dev_id;
	int dma_ch;
	int req_status;
	int tx_status;
	wait_queue_head_t wait;
};

struct omapvideo_info {
	int id;
	int num_overlays;
	union {
		struct omap_overlay *overlays[MAX_OVLS];
		struct {
			struct omap_overlay *lcd;
			struct omap_overlay *hdmi;
		} named;
	} overlays;
	bool layer_enabled_saved[MAX_OVLS];
	bool layer_wb_saved[MAX_OVLS];
};

struct omap2video_device {
	struct mutex  mtx;

	int state;

	struct v4l2_device v4l2_dev;
	struct omap_vout_device *vouts[MAX_VOUT_DEV];

	int num_displays;
	struct omap_dss_device *displays[MAX_DISPLAYS];
	int num_overlays;
	struct omap_overlay *overlays[MAX_OVLS];
	int num_managers;
	struct omap_overlay_manager *managers[MAX_MANAGERS];
};

/* manual update work */
struct omap_vout_work {
	struct omap_vout_device *vout;
	struct work_struct work;
	bool process;
};

struct omap_vout_device;
struct omap_vout_frame_info {
	//buffer information
	void __iomem			*vAddr;		//virtual address
	unsigned long			pAddr;		//physical address
	unsigned long			pUvAddr;	//physcial UV address for YUV

	//frame information
	enum omap_color_mode	color;		//color mode
	struct v4l2_pix_format	pix;		//frame pix info
	struct v4l2_rect		crop;		//cropping rect

	struct omap_vout_device	*vout;		//back pointer
};

enum v4l2_frame_pack_type {
    V4L2_FPACK_NONE = 0,
    V4L2_FPACK_OVERUNDER,
    V4L2_FPACK_SIDEBYSIDE,
    V4L2_FPACK_ROW_IL,
    V4L2_FPACK_COL_IL,
    V4L2_FPACK_PIX_IL,
    V4L2_FPACK_CHECKB,
    V4L2_FPACK_FRM_SEQ,
};

enum v4l2_frame_pack_order {
    V4L2_FPACK_ORDER_LF = 0,
    V4L2_FPACK_ORDER_RF,
};

/**
 * Work Struct for Frame Display
 */
struct omap_vout_display_work {
	struct videobuf_buffer *frame;
	bool	lastframe_update;
	struct omap_vout_device	*vout;
	struct work_struct	work;
};

#define MAX_VRAM_BUFFER_COUNT	5

struct omap_vout_vram_buffer
{
	unsigned long	paddr;			//Physical address
	void __iomem	*vaddr;			//virtual address
	int				Bpp;			//Bytes Per Pixel
	unsigned long	alloc_size;		//allocate size
};

struct omap_vout_vram_buffer_manager {
	struct mutex	lock;
	struct omap_vout_vram_buffer buffers[MAX_VRAM_BUFFER_COUNT];
	bool	occupied[MAX_VRAM_BUFFER_COUNT];
};

enum STREAMING_STATUS {
	E_STREAMING_STOPPED = 0,
	E_STREAMING_ON_GOING,
	E_STREAMING_STOPPING,
	E_STREAMING_NO_DRAW
};

/* per-device data structure */
struct omap_vout_device {

	struct omapvideo_info		vid_info;

	struct video_device			*vfd;
	struct omap2video_device	*vid_dev;
	int 						vid;
	int 						opened;

	/* Lock to protect the shared data structures in ioctl */
	struct mutex lock;

	enum STREAMING_STATUS streaming_status;

	struct {
		int 					bpp; /* bytes per pixel */
		struct v4l2_pix_format	pix;
		struct v4l2_rect		crop;
		enum omap_color_mode	dss_mode;
	} input_info;

	struct {
		int 					buffer_allocated;
		int 					buffer_size;
		enum v4l2_buf_type		type;
		struct v4l2_framebuffer fbuf;
		enum v4l2_memory 		memory;
		int 					mmap_count;

		//Allocation info
		unsigned long buf_virt_addr[VIDEO_MAX_FRAME];
		unsigned long buf_phy_addr[VIDEO_MAX_FRAME];
		unsigned long buf_phy_uv_addr_alloced[VIDEO_MAX_FRAME];
		unsigned long buf_phy_addr_alloced[VIDEO_MAX_FRAME];
		unsigned long buf_phy_uv_addr[VIDEO_MAX_FRAME];
	} buf_info;

	struct {
		//Video Buffer Queue
		spinlock_t				vbq_lock;		/* spinlock for videobuf queues */
		struct videobuf_queue	vbq;

		//frame drawing request queue
		struct list_head		drawing_req_queue;

		//Additional Frame Info For Display. will be linked with video buffer
		struct omap_vout_frame_info frames[VIDEO_MAX_FRAME];

		struct videobuf_buffer*	displaying_frame;	//frame which is in displaying
	} queing_info;

	struct {
		enum v4l2_frame_pack_type	s3d_type;
		enum v4l2_frame_pack_order	s3d_pack_order;
		struct {
			struct v4l2_window	win;
			enum dss_rotation	rotation;
			bool 				mirror;
			struct omap_vout_vram_buffer *wb_buffer[2];	//for ping-pong write
			struct omap_vout_vram_buffer *wb_pre_buffer;
			int					wb_buffer_cursor;					//current ping-ping buffer
			unsigned char *black_line;
		} lcd;
		struct {
			bool 				enable;
			enum dss_rotation	rotation;
			struct v4l2_window	win;
			bool mirror;
		} hdmi;
		int 	depth_factor;
	} display_info;

	//frame rendering work queue
	struct workqueue_struct *workqueue;

	//vram buffer
	struct omap_vout_vram_buffer_manager	buffer_manager;

#ifdef CONFIG_HAS_EARLYSUSPEND
	struct early_suspend dawing_stop_suspend_handler;	//early suspend drawing stop handler
	struct early_suspend fb_disable_suspend_handler;	//disable suspend handler
#endif

	//Display Loop Mutex
	struct mutex in_display_operation;

};

struct vout_platform_data {
	void (*set_min_bus_tput)(struct device *dev, u8 agent_id,
			unsigned long r);
	void (*set_max_mpu_wakeup_lat)(struct device *dev, long t);
	void (*set_cpu_freq)(unsigned long f);
};
#endif	/* ifndef OMAP_VOUTDEF_H */
