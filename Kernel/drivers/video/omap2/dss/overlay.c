/*
 * linux/drivers/video/omap2/dss/overlay.c
 *
 * Copyright (C) 2009 Nokia Corporation
 * Author: Tomi Valkeinen <tomi.valkeinen@nokia.com>
 *
 * Some code and ideas taken from drivers/video/omap/ driver
 * by Imre Deak.
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms of the GNU General Public License version 2 as published by
 * the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
 * more details.
 *
 * You should have received a copy of the GNU General Public License along with
 * this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#define DSS_SUBSYS_NAME "OVERLAY"

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/err.h>
#include <linux/sysfs.h>
#include <linux/kobject.h>
#include <linux/platform_device.h>
#include <linux/delay.h>
#include <linux/slab.h>

#include <plat/display.h>
#include <plat/cpu.h>

#include "dss.h"

#define MAX_DSS_OVERLAYS (cpu_is_omap44xx() ? 4 : 3)

/* LGE_CHANGE [wonki.choi@lge.com] Overlay Refactoring 2011-2-05 */
#define REQ_OVERLAY
static int num_overlays;
static struct list_head overlay_list;
static struct omap_overlay *dispc_overlays[4];
/* LGE_CHANGE_S [wonki.choi@lge.com] Overlay Refactoring 2011-2-05*/
#ifdef REQ_OVERLAY
#define MAX_MANAGER_NAME_LEN 16

struct omap_overlay_info omap_overlay_info_req[4];
#endif
/* LGE_CHANGE_E [wonki.choi@lge.com] 2011-2-05 */

static ssize_t overlay_name_show(struct omap_overlay *ovl, char *buf)
{
	return snprintf(buf, PAGE_SIZE, "%s\n", ovl->name);
}

static ssize_t overlay_manager_show(struct omap_overlay *ovl, char *buf)
{
	return snprintf(buf, PAGE_SIZE, "%s\n",
			ovl->manager ? ovl->manager->name : "<none>");
}

static ssize_t overlay_manager_store(struct omap_overlay *ovl, const char *buf,
		size_t size)
{
	int i, r;
	struct omap_overlay_manager *mgr = NULL;
	struct omap_overlay_manager *old_mgr;
	struct omap_overlay_info info;
	int len = size;

	if (buf[size-1] == '\n')
		--len;

	if (len > 0) {
		for (i = 0; i < omap_dss_get_num_overlay_managers(); ++i) {
			mgr = omap_dss_get_overlay_manager(i);

			if (strncmp(buf, mgr->name, len) == 0)
				break;

			mgr = NULL;
		}
	}

	if (len > 0 && mgr == NULL)
		return -EINVAL;

	if (mgr)
		DSSDBG("manager %s found\n", mgr->name);

	if (mgr == ovl->manager)
		return size;

	if (mgr && sysfs_streq(mgr->name, "tv")) {
		ovl->get_overlay_info(ovl, &info);
		/* LGE_CHANGE_S [wonki.choi@lge.com] 2010-12-24 HDMI*/
		//OUT_WIDTH is correct
//		if (mgr->device->panel.timings.x_res < info.width ||
//			mgr->device->panel.timings.y_res < info.height) {
		if (mgr->device->panel.timings.x_res < info.out_width ||
			mgr->device->panel.timings.y_res < info.out_height) {
		/* LGE_CHANGE_E [wonki.choi@lge.com] 2010-12-24 HDMI*/
			printk(KERN_ERR"TV does not support downscaling"
			"Please configure overlay to supported format");
			return -EINVAL;
		}
	}

	old_mgr = ovl->manager;

	/* detach old manager */
	if (old_mgr) {
		r = ovl->unset_manager(ovl);
		if (r) {
			DSSERR("detach failed\n");
			return r;
		}

		r = old_mgr->apply(old_mgr);
		if (r)
			return r;
	}

	if (mgr) {
		r = ovl->set_manager(ovl, mgr);
		if (r) {
			DSSERR("Failed to attach overlay\n");
			return r;
		}

		r = mgr->apply(mgr);
		if (r)
			return r;
	}

	return size;
}

static ssize_t overlay_input_size_show(struct omap_overlay *ovl, char *buf)
{
	return snprintf(buf, PAGE_SIZE, "%d,%d\n",
			ovl->info.width, ovl->info.height);
}

static ssize_t overlay_input_size_store(struct omap_overlay *ovl,
		const char *buf, size_t size)
{
	int r;
	char *last;
	struct omap_overlay_info info;

	ovl->get_overlay_info(ovl, &info);

	info.width = simple_strtoul(buf, &last, 10);
	++last;
	if (last - buf >= size)
		return -EINVAL;

	info.height = simple_strtoul(last, &last, 10);

	r = ovl->set_overlay_info(ovl, &info);
	if (r)
		return r;

	if (ovl->manager) {
		r = ovl->manager->apply(ovl->manager);
		if (r)
			return r;
	}

	return size;
}

static ssize_t overlay_screen_width_show(struct omap_overlay *ovl, char *buf)
{
	return snprintf(buf, PAGE_SIZE, "%d\n", ovl->info.screen_width);
}

static ssize_t overlay_position_show(struct omap_overlay *ovl, char *buf)
{
	return snprintf(buf, PAGE_SIZE, "%d,%d\n",
			ovl->info.pos_x, ovl->info.pos_y);
}

static ssize_t overlay_position_store(struct omap_overlay *ovl,
		const char *buf, size_t size)
{
	int r;
	char *last;
	struct omap_overlay_info info;

	ovl->get_overlay_info(ovl, &info);

	info.pos_x = simple_strtoul(buf, &last, 10);
	++last;
	if (last - buf >= size)
		return -EINVAL;

	info.pos_y = simple_strtoul(last, &last, 10);

	r = ovl->set_overlay_info(ovl, &info);
	if (r)
		return r;

	if (ovl->manager) {
		r = ovl->manager->apply(ovl->manager);
		if (r)
			return r;
	}

	return size;
}

static ssize_t overlay_output_size_show(struct omap_overlay *ovl, char *buf)
{
	return snprintf(buf, PAGE_SIZE, "%d,%d\n",
			ovl->info.out_width, ovl->info.out_height);
}

static ssize_t overlay_output_size_store(struct omap_overlay *ovl,
		const char *buf, size_t size)
{
	int r, out_width, out_height;
	char *last;
	struct omap_overlay_info info;

	ovl->get_overlay_info(ovl, &info);

	out_width = simple_strtoul(buf, &last, 10);
	++last;
	if (last - buf >= size)
		return -EINVAL;

	out_height = simple_strtoul(last, &last, 10);

	if (sysfs_streq(ovl->manager->name, "tv")) {
		if (ovl->manager->device->panel.timings.x_res < out_width ||
/* LGE_CHANGE_S [wonki.choi@lge.com] 2010-12-24 HDMI*/
//		ovl->manager->device->panel.timings.y_res < out_height)
//		printk(KERN_ERR"TV does not support downscaling , Wrong output size");
//		return -EINVAL;
		ovl->manager->device->panel.timings.y_res < out_height){
			printk(KERN_ERR"TV does not support downscaling , Wrong output size");
			return -EINVAL;
		}
/* LGE_CHANGE_E [wonki.choi@lge.com] 2010-12-19 */
	}

	info.out_width = out_width;
	info.out_height = out_height;

	r = ovl->set_overlay_info(ovl, &info);
	if (r)
		return r;

	if (ovl->manager) {
		r = ovl->manager->apply(ovl->manager);
		if (r)
			return r;
	}

	return size;
}

static ssize_t overlay_enabled_show(struct omap_overlay *ovl, char *buf)
{
	return snprintf(buf, PAGE_SIZE, "%d\n", ovl->info.enabled);
}

static ssize_t overlay_enabled_store(struct omap_overlay *ovl, const char *buf,
		size_t size)
{
	int r;
	struct omap_overlay_info info;

	ovl->get_overlay_info(ovl, &info);

	info.enabled = simple_strtoul(buf, NULL, 10);

	DSSINFO("overlay '%s' enabled '%d'\n", ovl->name, info.enabled);

	r = ovl->set_overlay_info(ovl, &info);
	if (r)
		return r;

	if (ovl->manager) {
		r = ovl->manager->apply(ovl->manager);
		if (r)
			return r;
	}

	return size;
}

static ssize_t overlay_global_alpha_show(struct omap_overlay *ovl, char *buf)
{
	return snprintf(buf, PAGE_SIZE, "%d\n",
			ovl->info.global_alpha);
}

static ssize_t overlay_global_alpha_store(struct omap_overlay *ovl,
		const char *buf, size_t size)
{
	int r;
	struct omap_overlay_info info;

	ovl->get_overlay_info(ovl, &info);

	/* In OMAP2/3 Video1 plane does not support global alpha
	 * to always make it 255 completely opaque
	 */
	if ((!cpu_is_omap44xx()) && (ovl->id == OMAP_DSS_VIDEO1))
		info.global_alpha = 255;
	else
		info.global_alpha = simple_strtoul(buf, NULL, 10);

	r = ovl->set_overlay_info(ovl, &info);
	if (r)
		return r;

	if (ovl->manager) {
		r = ovl->manager->apply(ovl->manager);
		if (r)
			return r;
	}

	return size;
}


static ssize_t overlay_decim_show(u16 min, u16 max, char *buf)
{
	return snprintf(buf, PAGE_SIZE, "%d..%d\n", min, max);
}

static ssize_t overlay_x_decim_show(struct omap_overlay *ovl, char *buf)
{
	return overlay_decim_show(ovl->info.min_x_decim, ovl->info.max_x_decim,
									buf);
}

static ssize_t overlay_y_decim_show(struct omap_overlay *ovl, char *buf)
{
	return overlay_decim_show(ovl->info.min_y_decim, ovl->info.max_y_decim,
									buf);
}

static ssize_t overlay_decim_store(u16 *min, u16 *max,
						const char *buf, size_t size)
{
	char *last;

	*min = *max = simple_strtoul(buf, &last, 10);
	if (last < buf + size && *last == '.') {
		/* check for .. separator */
		if (last + 2 >= buf + size || last[1] != '.')
			return -EINVAL;

		*max = simple_strtoul(last + 2, &last, 10);

		/* fix order */
		if (*max < *min)
			swap(*min, *max);
	}

	/* decimation must be positive */
	if (*min == 0)
		return -EINVAL;

	return 0;
}

static ssize_t overlay_x_decim_store(struct omap_overlay *ovl,
						const char *buf, size_t size)
{
	int r;
	struct omap_overlay_info info;

	ovl->get_overlay_info(ovl, &info);

	r = overlay_decim_store(&info.min_x_decim, &info.max_x_decim,
				buf, size);

	r = r ? : ovl->set_overlay_info(ovl, &info);

	if (!r && ovl->manager)
		r = ovl->manager->apply(ovl->manager);

	return r ? : size;
}

static ssize_t overlay_y_decim_store(struct omap_overlay *ovl,
						const char *buf, size_t size)
{
	int r;
	struct omap_overlay_info info;

	ovl->get_overlay_info(ovl, &info);

	r = overlay_decim_store(&info.min_y_decim, &info.max_y_decim,
				   buf, size);

	r = r ? : ovl->set_overlay_info(ovl, &info);

	if (!r && ovl->manager)
		r = ovl->manager->apply(ovl->manager);

	return r ? : size;
}

static ssize_t overlay_zorder_show(struct omap_overlay *ovl, char *buf)
{
	return snprintf(buf, PAGE_SIZE, "%d\n",
			ovl->info.zorder);
}

static ssize_t overlay_zorder_store(struct omap_overlay *ovl,
		const char *buf, size_t size)
{
	int r;
	struct omap_overlay_info info;

	ovl->get_overlay_info(ovl, &info);

	info.zorder = simple_strtoul(buf, NULL, 10);

	r = ovl->set_overlay_info(ovl, &info);
	if (r)
		return r;

	if (ovl->manager) {
		r = ovl->manager->apply(ovl->manager);
		if (r)
			return r;
	}

	return size;
}
//LGE START - HDMI GUI Cloning - novashock.lee@lge.com 2011-01-30 //

//show rotation
static ssize_t overlay_rotation_show(struct omap_overlay *ovl, char *buf)
{
	return snprintf(buf, PAGE_SIZE, "%d\n", ovl->info.rotation);
}

//show mirror
static ssize_t overlay_mirror_show(struct omap_overlay *ovl, char *buf)
{
	return snprintf(buf, PAGE_SIZE, "%d\n", ovl->info.mirror);
}

#ifdef REQ_OVERLAY
static ssize_t overlay_req_enabled_show(struct omap_overlay *ovl, char *buf)
{
	return snprintf(buf, PAGE_SIZE, "%d\n", 
		omap_overlay_info_req[ovl->id].enabled);
}

static ssize_t overlay_req_enabled_store(struct omap_overlay *ovl,
		const char *buf, size_t size)
{
	int value = simple_strtoul(buf, NULL, 10);
	
	omap_overlay_info_req[ovl->id].enabled = value;
	return size;
}

static ssize_t overlay_req_rotation_show(struct omap_overlay *ovl, char *buf)
{
	return snprintf(buf, PAGE_SIZE, "%d\n", 
		omap_overlay_info_req[ovl->id].rotation);
}

static ssize_t overlay_req_rotation_store(struct omap_overlay *ovl,
		const char *buf, size_t size)
{
	int value = simple_strtoul(buf, NULL, 10);
	
	if(value < OMAP_DSS_ROT_0 || value > OMAP_DSS_ROT_270)
		return -EINVAL;

	omap_overlay_info_req[ovl->id].rotation = value;
	return size;
}

static ssize_t overlay_req_rotation_type_show(struct omap_overlay *ovl, char *buf)
{
	return snprintf(buf, PAGE_SIZE, "%d\n", 
		omap_overlay_info_req[ovl->id].rotation_type);
}

static ssize_t overlay_req_rotation_type_store(struct omap_overlay *ovl,
		const char *buf, size_t size)
{
	int value = simple_strtoul(buf, NULL, 10);

	if(value < OMAP_DSS_ROT_DMA || value > OMAP_DSS_ROT_TILER)
		return -EINVAL;
	
	omap_overlay_info_req[ovl->id].rotation_type = value;
	return size;
}

static ssize_t overlay_req_position_show(struct omap_overlay *ovl, char *buf)
{
	return snprintf(buf, PAGE_SIZE, "%d,%d\n", 
		omap_overlay_info_req[ovl->id].pos_x, 
		omap_overlay_info_req[ovl->id].pos_y);
}

static ssize_t overlay_req_position_store(struct omap_overlay *ovl,
		const char *buf, size_t size)
{
	char *last;
	int pos_x, pos_y;
	
	pos_x = simple_strtoul(buf, &last, 10);
	++last;
	if (last - buf >= size)
		return -EINVAL;

	pos_y = simple_strtoul(last, &last, 10);

	if(pos_x < 0 || pos_y < 0)
		return -EINVAL;
	
	omap_overlay_info_req[ovl->id].pos_x = pos_x;
	omap_overlay_info_req[ovl->id].pos_y = pos_y;
	
	return size;
}

static ssize_t overlay_req_out_size_show(struct omap_overlay *ovl, char *buf)
{
	return snprintf(buf, PAGE_SIZE, "%d,%d\n", 
		omap_overlay_info_req[ovl->id].out_width,
		omap_overlay_info_req[ovl->id].out_height);
}

static ssize_t overlay_req_out_size_store(struct omap_overlay *ovl,
		const char *buf, size_t size)
{
	char *last;
	int out_width, out_height;

	out_width = simple_strtoul(buf, &last, 10);
	++last;
	if (last - buf >= size)
		return -EINVAL;

	out_height = simple_strtoul(last, &last, 10);

	if(out_width < 0 || out_height < 0)
		return -EINVAL;
	
#if 0
	if (sysfs_streq(ovl->manager->name, "tv")) {
		if (ovl->manager->device->panel.timings.x_res < out_width ||
			ovl->manager->device->panel.timings.y_res < out_height) {
				printk(KERN_ERR "TV does not support downscaling , Wrong output size");
				return -EINVAL;
		}
	}
#endif	

	omap_overlay_info_req[ovl->id].out_width = out_width;
	omap_overlay_info_req[ovl->id].out_height = out_height;

	return size;
}

static ssize_t overlay_req_global_alpha_show(struct omap_overlay *ovl, char *buf)
{
	return snprintf(buf, PAGE_SIZE, "%d\n", 
		omap_overlay_info_req[ovl->id].global_alpha);
}

static ssize_t overlay_req_global_alpha_store(struct omap_overlay *ovl,
		const char *buf, size_t size)
{
	int value = simple_strtoul(buf, NULL, 10);

	if(value < 0 || value > 255)
		return -EINVAL;
	
	omap_overlay_info_req[ovl->id].global_alpha = value;
	return size;
}

static ssize_t overlay_req_zorder_show(struct omap_overlay *ovl, char *buf)
{
	return snprintf(buf, PAGE_SIZE, "%d\n", 
		omap_overlay_info_req[ovl->id].zorder);
}

static ssize_t overlay_req_zorder_store(struct omap_overlay *ovl,
		const char *buf, size_t size)
{
	int value = simple_strtoul(buf, NULL, 10);

	if(value < OMAP_DSS_OVL_ZORDER_0 || value > OMAP_DSS_OVL_ZORDER_3)
		return -EINVAL;

	omap_overlay_info_req[ovl->id].zorder = value;
	return size;
}

static ssize_t overlay_req_manager_show(struct omap_overlay *ovl, char *buf)
{
	return snprintf(buf, PAGE_SIZE, "%s\n",
			omap_overlay_info_req[ovl->id].manager ? 
			omap_overlay_info_req[ovl->id].manager : "<none>");
}

static ssize_t overlay_req_manager_store(struct omap_overlay *ovl,
		const char *buf, size_t size)
{
	int i;
	int len = size;
	struct omap_overlay_manager *mgr = NULL;
	
	if (buf[size - 1] == '\n')
		--len;
	
	if (len <= 0 || len >= MAX_MANAGER_NAME_LEN) 
		return -EINVAL;
	
	for (i = 0; i < omap_dss_get_num_overlay_managers(); ++i) {
		mgr = omap_dss_get_overlay_manager(i);

		if (!strncmp(buf, mgr->name, len))
			break;

		mgr = NULL;
	}

	if (mgr == NULL)
		return -EINVAL;

	memcpy(omap_overlay_info_req[ovl->id].manager, buf, len);
	omap_overlay_info_req[ovl->id].manager[len] = '\0';
	
	return size;
}

static ssize_t overlay_req_request_show(struct omap_overlay *ovl, char *buf)
{
	return snprintf(buf, PAGE_SIZE, "%d\n", omap_overlay_info_req[ovl->id].status);
}

static ssize_t overlay_req_request_store(struct omap_overlay *ovl,
		const char *buf, size_t size)
{
	int ret, i, len, input;
	struct omap_overlay_manager *mgr = NULL;
	struct omap_overlay_manager *old_mgr = NULL;
	struct omap_overlay_info info;

	input = simple_strtoul(buf, NULL, 10);
	
	if(input == 1) {	//prepare
		ovl->get_overlay_info(ovl, &omap_overlay_info_req[ovl->id]);
		omap_overlay_info_req[ovl->id].status = 1;
	}
	else if((input == 2) && (1 == omap_overlay_info_req[ovl->id].status)) {	//commit
		len = strlen(omap_overlay_info_req[ovl->id].manager);
		if(len <= 0)
			goto no_need_set_manager;

		for (i = 0; i < omap_dss_get_num_overlay_managers(); ++i) {
			mgr = omap_dss_get_overlay_manager(i);

			if (!strncmp(omap_overlay_info_req[ovl->id].manager, mgr->name, len))
				break;

			mgr = NULL;
		}

		if (mgr == NULL) 
			goto err_manager;
		
		old_mgr = ovl->manager;
		if (mgr != old_mgr) {
			/* detach old manager */
			if (old_mgr) {
				//disable overlay
				ovl->get_overlay_info(ovl, &info);
				info.enabled = 0;
				ret = ovl->set_overlay_info(ovl, &info);
				if (ret) 
					goto err_manager;
				
				ret = ovl->unset_manager(ovl);
				if (ret) {
					DSSERR("detach failed\n");
					goto err_manager;
				}

				ret = old_mgr->apply(old_mgr);
				if (ret) 
					goto err_manager;
				

				if (ret) 
					goto err_manager;
			}

			if (mgr) {
				ret = ovl->set_manager(ovl, mgr);
				if (ret) {
					DSSERR("Failed to attach overlay\n");
					goto err_manager;
				}

				ret = mgr->apply(mgr);
				if (ret) 
					goto err_manager;
			}
		}	
		omap_overlay_info_req[ovl->id].status = 2;
	}
	return size;		
	
no_need_set_manager:	
	omap_overlay_info_req[ovl->id].status = 2;
	return size;	
	
err_manager:
	omap_overlay_info_req[ovl->id].status = 3;
	return size;
}
#endif
//LGE END - HDMI GUI Cloning - novashock.lee@lge.com 2011-01-30 //
struct overlay_attribute {
	struct attribute attr;
	ssize_t (*show)(struct omap_overlay *, char *);
	ssize_t	(*store)(struct omap_overlay *, const char *, size_t);
};

#define OVERLAY_ATTR(_name, _mode, _show, _store) \
	struct overlay_attribute overlay_attr_##_name = \
	__ATTR(_name, _mode, _show, _store)

static OVERLAY_ATTR(name, S_IRUGO, overlay_name_show, NULL);
static OVERLAY_ATTR(manager, S_IRUGO|S_IWUSR,
		overlay_manager_show, overlay_manager_store);
static OVERLAY_ATTR(input_size, S_IRUGO|S_IWUSR,
		    overlay_input_size_show, overlay_input_size_store);
static OVERLAY_ATTR(screen_width, S_IRUGO, overlay_screen_width_show, NULL);
static OVERLAY_ATTR(position, S_IRUGO|S_IWUSR,
		overlay_position_show, overlay_position_store);
static OVERLAY_ATTR(output_size, S_IRUGO|S_IWUSR,
		overlay_output_size_show, overlay_output_size_store);
static OVERLAY_ATTR(enabled, S_IRUGO|S_IWUSR,
		overlay_enabled_show, overlay_enabled_store);
static OVERLAY_ATTR(global_alpha, S_IRUGO|S_IWUSR,
		overlay_global_alpha_show, overlay_global_alpha_store);
static OVERLAY_ATTR(x_decim, S_IRUGO|S_IWUSR,
		overlay_x_decim_show, overlay_x_decim_store);
static OVERLAY_ATTR(y_decim, S_IRUGO|S_IWUSR,
		overlay_y_decim_show, overlay_y_decim_store);
static OVERLAY_ATTR(zorder, S_IRUGO|S_IWUSR,
		overlay_zorder_show, overlay_zorder_store);
//LGE START - HDMI GUI Cloning - novashock.lee@lge.com 2011-01-30 //
static OVERLAY_ATTR(rotation, S_IRUGO, overlay_rotation_show, NULL);
static OVERLAY_ATTR(mirror,	S_IRUGO, overlay_mirror_show, NULL);
#ifdef REQ_OVERLAY
static OVERLAY_ATTR(req_enabled, S_IRUGO|S_IWUSR,
		overlay_req_enabled_show, overlay_req_enabled_store);
static OVERLAY_ATTR(req_rotation, S_IRUGO|S_IWUSR,
		overlay_req_rotation_show, overlay_req_rotation_store);
static OVERLAY_ATTR(req_rotation_type, S_IRUGO|S_IWUSR,
		overlay_req_rotation_type_show, overlay_req_rotation_type_store);
static OVERLAY_ATTR(req_position, S_IRUGO|S_IWUSR,
		overlay_req_position_show, overlay_req_position_store);
static OVERLAY_ATTR(req_out_size, S_IRUGO|S_IWUSR,
		overlay_req_out_size_show, overlay_req_out_size_store);
static OVERLAY_ATTR(req_global_alpha, S_IRUGO|S_IWUSR,
		overlay_req_global_alpha_show, overlay_req_global_alpha_store);
static OVERLAY_ATTR(req_zorder, S_IRUGO|S_IWUSR,
		overlay_req_zorder_show, overlay_req_zorder_store);
static OVERLAY_ATTR(req_manager, S_IRUGO|S_IWUSR,
		overlay_req_manager_show, overlay_req_manager_store);
static OVERLAY_ATTR(req_request, S_IRUGO|S_IWUSR,
		overlay_req_request_show, overlay_req_request_store);
#endif
//LGE END - HDMI GUI Cloning - novashock.lee@lge.com 2011-01-30 //

static struct attribute *overlay_sysfs_attrs[] = {
	&overlay_attr_name.attr,
	&overlay_attr_manager.attr,
	&overlay_attr_input_size.attr,
	&overlay_attr_screen_width.attr,
	&overlay_attr_position.attr,
	&overlay_attr_output_size.attr,
	&overlay_attr_enabled.attr,
	&overlay_attr_global_alpha.attr,
	&overlay_attr_x_decim.attr,
	&overlay_attr_y_decim.attr,
	&overlay_attr_zorder.attr,
//LGE START - HDMI GUI Cloning - novashock.lee@lge.com 2011-01-30 //
	&overlay_attr_rotation.attr,
	&overlay_attr_mirror.attr,
#ifdef REQ_OVERLAY
	&overlay_attr_req_enabled.attr,
	&overlay_attr_req_rotation.attr,
	&overlay_attr_req_rotation_type.attr,
	&overlay_attr_req_position.attr,
	&overlay_attr_req_out_size.attr,
	&overlay_attr_req_global_alpha.attr,
	&overlay_attr_req_zorder.attr,
	&overlay_attr_req_manager.attr,
	&overlay_attr_req_request.attr,
#endif
//LGE END - HDMI GUI Cloning - novashock.lee@lge.com 2011-01-30 //
	NULL
};

static ssize_t overlay_attr_show(struct kobject *kobj, struct attribute *attr,
		char *buf)
{
	struct omap_overlay *overlay;
	struct overlay_attribute *overlay_attr;

	overlay = container_of(kobj, struct omap_overlay, kobj);
	overlay_attr = container_of(attr, struct overlay_attribute, attr);

	if (!overlay_attr->show)
		return -ENOENT;

	return overlay_attr->show(overlay, buf);
}

static ssize_t overlay_attr_store(struct kobject *kobj, struct attribute *attr,
		const char *buf, size_t size)
{
	struct omap_overlay *overlay;
	struct overlay_attribute *overlay_attr;

	overlay = container_of(kobj, struct omap_overlay, kobj);
	overlay_attr = container_of(attr, struct overlay_attribute, attr);

	if (!overlay_attr->store)
		return -ENOENT;

	return overlay_attr->store(overlay, buf, size);
}

static const struct sysfs_ops overlay_sysfs_ops = {
	.show = overlay_attr_show,
	.store = overlay_attr_store,
};

static struct kobj_type overlay_ktype = {
	.sysfs_ops = &overlay_sysfs_ops,
	.default_attrs = overlay_sysfs_attrs,
};

/* Check if overlay parameters are compatible with display */
int dss_check_overlay(struct omap_overlay *ovl, struct omap_dss_device *dssdev)
{
	struct omap_overlay_info *info;
	u16 outw, outh;
	u16 dw, dh;

	if (!dssdev)
		return 0;

	if (!ovl->info.enabled)
		return 0;

	info = &ovl->info;

	if (info->paddr == 0) {
		DSSDBG("check_overlay failed: paddr 0\n");
		return -EINVAL;
	}

	dssdev->driver->get_resolution(dssdev, &dw, &dh);
	/* y resolution to be doubled in case of interlaced HDMI */
	if ((ovl->info.field == IBUF_IDEV) || (ovl->info.field == PBUF_IDEV))
		dh *= 2;

	DSSDBG("check_overlay %d: (%d,%d %dx%d -> %dx%d) disp (%dx%d)\n",
			ovl->id,
			info->pos_x, info->pos_y,
			info->width, info->height,
			info->out_width, info->out_height,
			dw, dh);

	if ((ovl->caps & OMAP_DSS_OVL_CAP_SCALE) == 0) {
		outw = info->width;
		outh = info->height;
	} else {
		if (info->out_width == 0)
			outw = info->width;
		else
			outw = info->out_width;

		if (info->out_height == 0)
			outh = info->height;
		else
			outh = info->out_height;
	}

	if ((dw < info->pos_x + outw) && !info->out_wb) {
		DSSERR("check_overlay failed 1: %d < %d + %d\n",
				dw, info->pos_x, outw);
		return -EINVAL;
	}

	if ((dh < info->pos_y + outh) && !info->out_wb) {
		DSSERR("check_overlay failed 2: %d < %d + %d\n",
				dh, info->pos_y, outh);
		return -EINVAL;
	}

	if ((ovl->supported_modes & info->color_mode) == 0) {
		DSSERR("overlay doesn't support mode %d\n", info->color_mode);
		return -EINVAL;
	}

	if ((info->zorder < OMAP_DSS_OVL_ZORDER_0) ||
			(info->zorder > OMAP_DSS_OVL_ZORDER_3)) {
		DSSERR("overlay doesn't support zorder %d\n", info->zorder);
		return -EINVAL;
	}

	return 0;
}

static int dss_ovl_set_overlay_info(struct omap_overlay *ovl,
		struct omap_overlay_info *info)
{
	int r;
	struct omap_overlay_info old_info;

	old_info = ovl->info;
	ovl->info = *info;

	if (ovl->manager) {
		r = dss_check_overlay(ovl, ovl->manager->device);
		if (r) {
			ovl->info = old_info;
			return r;
		}
	}

	ovl->info_dirty = true;

	return 0;
}

static void dss_ovl_get_overlay_info(struct omap_overlay *ovl,
		struct omap_overlay_info *info)
{
	*info = ovl->info;
}

static int dss_ovl_wait_for_go(struct omap_overlay *ovl)
{
	return dss_mgr_wait_for_go_ovl(ovl);
}

static int omap_dss_set_manager(struct omap_overlay *ovl,
		struct omap_overlay_manager *mgr)
{
	if (!mgr)
		return -EINVAL;

	if (ovl->manager) {
		dump_stack();
		DSSERR("overlay '%s' already has a manager '%s'\n",
				ovl->name, ovl->manager->name);
		return -EINVAL;
	}

	if (ovl->info.enabled) {
		DSSERR("overlay has to be disabled to change the manager\n");
		return -EINVAL;
	}

	ovl->manager = mgr;

	/* do not set channel out if DSS is off */
	if (!dss_get_mainclk_state())
		return 0;

	dss_clk_enable(DSS_CLK_ICK | DSS_CLK_FCK1);
	/* XXX: on manual update display, in auto update mode, a bug happens
	 * here. When an overlay is first enabled on LCD, then it's disabled,
	 * and the manager is changed to TV, we sometimes get SYNC_LOST_DIGIT
	 * errors. Waiting before changing the channel_out fixes it. I'm
	 * guessing that the overlay is still somehow being used for the LCD,
	 * but I don't understand how or why. */
	msleep(40);
	dispc_set_channel_out(ovl->id, mgr->id);
	dss_clk_disable(DSS_CLK_ICK | DSS_CLK_FCK1);

	return 0;
}

static int omap_dss_unset_manager(struct omap_overlay *ovl)
{
	int r;

	if (!ovl->manager) {
		DSSERR("failed to detach overlay: manager not set\n");
		return -EINVAL;
	}

	if (ovl->info.enabled) {
		DSSERR("overlay has to be disabled to unset the manager\n");
		return -EINVAL;
	}

	r = ovl->wait_for_go(ovl);
	if (r)
		return r;

	ovl->manager = NULL;

	return 0;
}

bool dss_ovl_manually_updated(struct omap_overlay *ovl)
{
	struct omap_dss_device *dev;
	if (!ovl->manager)
		return false;
	dev = ovl->manager->device;
	if (!dev || !dev->driver)
		return false;
	return dssdev_manually_updated(dev);
}

int omap_dss_get_num_overlays(void)
{
	return num_overlays;
}
EXPORT_SYMBOL(omap_dss_get_num_overlays);

struct omap_overlay *omap_dss_get_overlay(int num)
{
	if (num < num_overlays) {
		return dispc_overlays[num];
	}

	return NULL;
}
EXPORT_SYMBOL(omap_dss_get_overlay);

static void omap_dss_add_overlay(struct omap_overlay *overlay)
{
	++num_overlays;
	list_add_tail(&overlay->list, &overlay_list);
}

void dss_overlay_setup_dispc_manager(struct omap_overlay_manager *mgr)
{
	mgr->num_overlays = MAX_DSS_OVERLAYS;
	mgr->overlays = dispc_overlays;
}

#ifdef L4_EXAMPLE
static struct omap_overlay *l4_overlays[1];
void dss_overlay_setup_l4_manager(struct omap_overlay_manager *mgr)
{
	mgr->num_overlays = 1;
	mgr->overlays = l4_overlays;
}
#endif

void dss_init_overlays(struct platform_device *pdev)
{
	int i, r;

	INIT_LIST_HEAD(&overlay_list);

	num_overlays = 0;

	for (i = 0; i < MAX_DSS_OVERLAYS; ++i) {
		struct omap_overlay *ovl;
		ovl = kzalloc(sizeof(*ovl), GFP_KERNEL);

		BUG_ON(ovl == NULL);

		mutex_init(&ovl->lock);
		switch (i) {
		case 0:
			ovl->name = "gfx";
			ovl->id = OMAP_DSS_GFX;
			ovl->supported_modes = (cpu_is_omap44xx() |
				cpu_is_omap34xx()) ?
				OMAP_DSS_COLOR_GFX_OMAP3 :
				OMAP_DSS_COLOR_GFX_OMAP2;
			ovl->caps = OMAP_DSS_OVL_CAP_DISPC;
			ovl->info.global_alpha = 255;
			ovl->info.zorder = OMAP_DSS_OVL_ZORDER_0;
			break;
		case 1:
			ovl->name = "vid1";
			ovl->id = OMAP_DSS_VIDEO1;
			ovl->supported_modes = (cpu_is_omap44xx() |
				cpu_is_omap34xx()) ?
				OMAP_DSS_COLOR_VID1_OMAP3 :
				OMAP_DSS_COLOR_VID_OMAP2;
			ovl->caps = OMAP_DSS_OVL_CAP_SCALE |
				OMAP_DSS_OVL_CAP_DISPC;
			ovl->info.global_alpha = 255;
			ovl->info.zorder = OMAP_DSS_OVL_ZORDER_3;
			break;
		case 2:
			ovl->name = "vid2";
			ovl->id = OMAP_DSS_VIDEO2;
			ovl->supported_modes = (cpu_is_omap44xx() |
				cpu_is_omap34xx()) ?
				OMAP_DSS_COLOR_VID2_OMAP3 :
				OMAP_DSS_COLOR_VID_OMAP2;
			ovl->caps = OMAP_DSS_OVL_CAP_SCALE |
				OMAP_DSS_OVL_CAP_DISPC;
			ovl->info.global_alpha = 255;
			ovl->info.zorder = OMAP_DSS_OVL_ZORDER_2;
			break;
		case 3:
			ovl->name = "vid3";
			ovl->id = OMAP_DSS_VIDEO3;
			ovl->supported_modes = OMAP_DSS_COLOR_VID3_OMAP3;
			ovl->caps = OMAP_DSS_OVL_CAP_SCALE |
				OMAP_DSS_OVL_CAP_DISPC;
			ovl->info.global_alpha = 255;
			ovl->info.zorder = OMAP_DSS_OVL_ZORDER_1;
			break;

		}

		ovl->info.min_x_decim = ovl->info.min_y_decim = 1;
		ovl->info.max_x_decim = ovl->info.max_y_decim =
			cpu_is_omap44xx() ? 16 : 1;

		ovl->set_manager = &omap_dss_set_manager;
		ovl->unset_manager = &omap_dss_unset_manager;
		ovl->set_overlay_info = &dss_ovl_set_overlay_info;
		ovl->get_overlay_info = &dss_ovl_get_overlay_info;
		ovl->wait_for_go = &dss_ovl_wait_for_go;

		omap_dss_add_overlay(ovl);

		r = kobject_init_and_add(&ovl->kobj, &overlay_ktype,
				&pdev->dev.kobj, "overlay%d", i);

		if (r) {
			DSSERR("failed to create sysfs file\n");
			continue;
		}

		dispc_overlays[i] = ovl;
	}

#ifdef L4_EXAMPLE
	{
		struct omap_overlay *ovl;
		ovl = kzalloc(sizeof(*ovl), GFP_KERNEL);

		BUG_ON(ovl == NULL);

		ovl->name = "l4";
		ovl->supported_modes = OMAP_DSS_COLOR_RGB24U;

		ovl->set_manager = &omap_dss_set_manager;
		ovl->unset_manager = &omap_dss_unset_manager;
		ovl->set_overlay_info = &dss_ovl_set_overlay_info;
		ovl->get_overlay_info = &dss_ovl_get_overlay_info;

		omap_dss_add_overlay(ovl);

		r = kobject_init_and_add(&ovl->kobj, &overlay_ktype,
				&pdev->dev.kobj, "overlayl4");

		if (r)
			DSSERR("failed to create sysfs file\n");

		l4_overlays[0] = ovl;
	}
#endif
}

/* connect overlays to the new device, if not already connected. if force
 * selected, connect always. */
void dss_recheck_connections(struct omap_dss_device *dssdev, bool force)
{
	int i;
	struct omap_overlay_manager *lcd_mgr;
	struct omap_overlay_manager *tv_mgr;
	struct omap_overlay_manager *lcd2_mgr = NULL;
	struct omap_overlay_manager *mgr = NULL;

	lcd_mgr = omap_dss_get_overlay_manager(OMAP_DSS_OVL_MGR_LCD);
	tv_mgr = omap_dss_get_overlay_manager(OMAP_DSS_OVL_MGR_TV);
	if (cpu_is_omap44xx())
		lcd2_mgr = omap_dss_get_overlay_manager(OMAP_DSS_OVL_MGR_LCD2);

	if (dssdev->channel == OMAP_DSS_CHANNEL_LCD2) {
		if(lcd2_mgr == NULL) return; //LGE_CHANGE [taekeun1.kim@lge.com] 2011-03-02, P920 : For WBT
		if (!lcd2_mgr->device || force) {
			if (lcd2_mgr->device)
				lcd2_mgr->unset_device(lcd2_mgr);
			lcd2_mgr->set_device(lcd2_mgr, dssdev);
			mgr = lcd2_mgr;
		}
	} else if (dssdev->type != OMAP_DISPLAY_TYPE_VENC
		&& dssdev->type != OMAP_DISPLAY_TYPE_HDMI) {
		if (!lcd_mgr->device || force) {
			if (lcd_mgr->device)
				lcd_mgr->unset_device(lcd_mgr);
			lcd_mgr->set_device(lcd_mgr, dssdev);
			mgr = lcd_mgr;
		}
	}

	if (dssdev->type == OMAP_DISPLAY_TYPE_VENC
		|| dssdev->type == OMAP_DISPLAY_TYPE_HDMI) {
		if (!tv_mgr->device || force) {
			if (tv_mgr->device)
				tv_mgr->unset_device(tv_mgr);
			tv_mgr->set_device(tv_mgr, dssdev);
			mgr = tv_mgr;
		}
	}

//LGE_CHANGE_S [taekeun1.kim@lge.com] 2010-1-03, P920 : prevent to blink .
	if(dssdev->type != OMAP_DISPLAY_TYPE_HDMI) {
//LGE_CHANGE_E [taekeun1.kim@lge.com] 2011-01-03
		if (mgr) {
			for (i = 0; i < MAX_DSS_OVERLAYS; i++) {
				struct omap_overlay *ovl;
				ovl = omap_dss_get_overlay(i);
				if (!ovl->manager || force) {
					if (ovl->manager)
						omap_dss_unset_manager(ovl);
					omap_dss_set_manager(ovl, mgr);
				}
			}
		}
//LGE_CHANGE_S [taekeun1.kim@lge.com] 2010-1-03, P920 : prevent to blink .
	}
//LGE_CHANGE_E [taekeun1.kim@lge.com] 2011-01-03
}

void dss_uninit_overlays(struct platform_device *pdev)
{
	struct omap_overlay *ovl;

	while (!list_empty(&overlay_list)) {
		ovl = list_first_entry(&overlay_list,
				struct omap_overlay, list);
		list_del(&ovl->list);
		kobject_del(&ovl->kobj);
		kobject_put(&ovl->kobj);
		kfree(ovl);
	}

	num_overlays = 0;
}

