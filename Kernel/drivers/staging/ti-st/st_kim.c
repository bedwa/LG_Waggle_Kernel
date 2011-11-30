/*
 *  Shared Transport Line discipline driver Core
 *	Init Manager module responsible for GPIO control
 *	and firmware download
 *  Copyright (C) 2009 Texas Instruments
 *
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License version 2 as
 *  published by the Free Software Foundation.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program; if not, write to the Free Software
 *  Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 *
 */

#include <linux/platform_device.h>
#include <linux/jiffies.h>
#include <linux/firmware.h>
#include <linux/delay.h>
#include <linux/wait.h>
#include <linux/gpio.h>
#include <linux/rtc.h> 
#include <linux/sched.h>

#include "st_kim.h"
/* understand BT events for fw response */
#include <net/bluetooth/bluetooth.h>
#include <net/bluetooth/hci_core.h>
#include <net/bluetooth/hci.h>

//#define VERBOSE

#undef VERBOSE
#undef DEBUG

/* all debug macros go in here */
#define ST_KIM_ERR(fmt, arg...)  printk(KERN_ERR "(stk):"fmt"\n" , ## arg)
#if defined(DEBUG)		/* limited debug messages */
#define ST_KIM_DBG(fmt, arg...)  printk(KERN_INFO "(stk):"fmt"\n" , ## arg)
#define ST_KIM_VER(fmt, arg...)
#elif defined(VERBOSE)		/* very verbose */
#define ST_KIM_DBG(fmt, arg...)  printk(KERN_INFO "(stk):"fmt"\n" , ## arg)
#define ST_KIM_VER(fmt, arg...)  printk(KERN_INFO "(stk):"fmt"\n" , ## arg)
#else /* error msgs only */
#define ST_KIM_DBG(fmt, arg...)
#define ST_KIM_VER(fmt, arg...)
#endif


static int kim_probe(struct platform_device *pdev);
static int kim_remove(struct platform_device *pdev);
static ssize_t show_pid(struct device *dev, struct device_attribute
			*attr, char *buf);
static ssize_t store_pid(struct device *dev, struct device_attribute
			 *devattr, char *buf, size_t count);
static ssize_t show_list(struct device *dev, struct device_attribute
			 *attr, char *buf);
static ssize_t show_version(struct device *dev, struct device_attribute
			 *attr, char *buf);

/* KIM platform device driver structure */
static struct platform_driver kim_platform_driver = {
	.probe = kim_probe,
	.remove = kim_remove,
	/* TODO: ST driver power management during suspend/resume ?
	 */
#if 0
	.suspend = kim_suspend,
	.resume = kim_resume,
#endif
	.driver = {
		   .name = "kim",
		   .owner = THIS_MODULE,
		   },
};

/* structures specific for sysfs entries */
static struct kobj_attribute pid_attr =
__ATTR(pid, 0644, (void *)show_pid, (void *)store_pid);

static struct kobj_attribute list_protocols =
__ATTR(protocols, 0444, (void *)show_list, NULL);

static struct kobj_attribute chip_version =
__ATTR(version, 0444, (void *)show_version, NULL);

static struct attribute *uim_attrs[] = {
	&pid_attr.attr,
	/* add more debug sysfs entries */
	&list_protocols.attr,
	&chip_version.attr,
	NULL,
};

static struct attribute_group uim_attr_grp = {
	.attrs = uim_attrs,
};


/* strings to be used for rfkill entries and by
 * ST Core to be used for sysfs debug entry
 */
#define PROTO_ENTRY(type, name)	name
const unsigned char *protocol_names[] = {
	PROTO_ENTRY(ST_GPIO_BT, "Bluetooth"),
	PROTO_ENTRY(ST_GPIO_FM, "FM"),
	PROTO_ENTRY(ST_GPIO_GPS, "GPS"),
};


static struct kim_data_s *kim_gdata;


/* chip version storage
 */
typedef struct chip_version
{
	unsigned short full;
	unsigned short chip;
	unsigned short min_ver;
	unsigned short maj_ver;
} kim_ver_t; 
static kim_ver_t kim_ver; 	


/**********************************************************************/
/* internal functions */

/*
 * function to return whether the firmware response was proper
 * in case of error don't complete so that waiting for proper
 * response times out
 */
void validate_firmware_response(struct sk_buff *skb)
{
	if (unlikely(skb->data[5] != 0)) {
		ST_KIM_ERR("no proper response during fw download");
		ST_KIM_ERR("data6 %x", skb->data[5]);
		return;		/* keep waiting for the proper response */
	}
	/* becos of all the script being downloaded */
	complete_all(&kim_gdata->kim_rcvd);
	kfree_skb(skb);
}

/* check for data len received inside kim_int_recv
 * most often hit the last case to update state to waiting for data
 */
static inline int kim_check_data_len(int len)
{
	register int room = skb_tailroom(kim_gdata->rx_skb);

	ST_KIM_DBG("len %d room %d", len, room);

	if (!len) {
		validate_firmware_response(kim_gdata->rx_skb);
	} else if (len > room) {
		/* Received packet's payload length is larger.
		 * We can't accommodate it in created skb.
		 */
		ST_KIM_ERR("Data length is too large len %d room %d", len,
			   room);
		kfree_skb(kim_gdata->rx_skb);
	} else {
		/* Packet header has non-zero payload length and
		 * we have enough space in created skb. Lets read
		 * payload data */
		kim_gdata->rx_state = ST_W4_DATA;
		kim_gdata->rx_count = len;
		return len;
	}

	/* Change ST LL state to continue to process next
	 * packet */
	kim_gdata->rx_state = ST_W4_PACKET_TYPE;
	kim_gdata->rx_skb = NULL;
	kim_gdata->rx_count = 0;

	return 0;
}

/* receive function called during firmware download
 * - firmware download responses on different UART drivers
 *   have been observed to come in bursts of different
 *   tty_receive and hence the logic
 */
void kim_int_recv(const unsigned char *data, long count)
{
	register char *ptr;
	struct hci_event_hdr *eh;
	register int len = 0, type = 0;

#if defined(SHOW_ST_LOG) //hsyoon 20110605 -remove log
	printk("%s( %d )\n", __func__, count);
#endif
	/* Decode received bytes here */
	ptr = (char *)data;
	if (unlikely(ptr == NULL)) {
		ST_KIM_ERR(" received null from TTY ");
		return;
	}
	
	while (count) {
		//printk("[%s] Enter\n", __func__);
		if (kim_gdata->rx_count) {
			len = min_t(unsigned int, kim_gdata->rx_count, count);
			memcpy(skb_put(kim_gdata->rx_skb, len), ptr, len);
			kim_gdata->rx_count -= len;
			count -= len;
			ptr += len;

			if (kim_gdata->rx_count)
				continue;

			/* Check ST RX state machine , where are we? */
			switch (kim_gdata->rx_state) {
				/* Waiting for complete packet ? */
			case ST_W4_DATA:
				ST_KIM_DBG("Complete pkt received");
				validate_firmware_response(kim_gdata->rx_skb);
				kim_gdata->rx_state = ST_W4_PACKET_TYPE;
				kim_gdata->rx_skb = NULL;
				continue;
				/* Waiting for Bluetooth event header ? */
			case ST_W4_HEADER:
				eh = (struct hci_event_hdr *)kim_gdata->
				    rx_skb->data;
				ST_KIM_DBG("Event header: evt 0x%2.2x"
					   "plen %d", eh->evt, eh->plen);
				kim_check_data_len(eh->plen);
				continue;
			}	/* end of switch */
		}		/* end of if rx_state */
		switch (*ptr) {
			/* Bluetooth event packet? */
		case HCI_EVENT_PKT:
			ST_KIM_DBG("Event packet");
			kim_gdata->rx_state = ST_W4_HEADER;
			kim_gdata->rx_count = HCI_EVENT_HDR_SIZE;
			type = HCI_EVENT_PKT;
			break;
		default:
			ST_KIM_DBG("unknown packet");
			ptr++;
			count--;
			continue;
		}		/* end of switch *ptr */
		ptr++;
		count--;
		kim_gdata->rx_skb =
		    bt_skb_alloc(HCI_MAX_FRAME_SIZE, GFP_ATOMIC);
		if (!kim_gdata->rx_skb) {
			ST_KIM_ERR("can't allocate mem for new packet");
			kim_gdata->rx_state = ST_W4_PACKET_TYPE;
			kim_gdata->rx_count = 0;
			return;
		} /* not necessary in this case */
		bt_cb(kim_gdata->rx_skb)->pkt_type = type;
	}			/* end of while count */

  mdelay(1); //hsyoon 20110628
  complete_all(&kim_gdata->kim_rcvd);//hsyoon 20110616

	ST_KIM_DBG("done %s", __func__);
	//printk("[%s] End\n", __func__);
	return;
}

#if 1//TI HSYoon 20110613
extern struct st_data_s *st_gdata;//required for st_kim.c
#endif 

static long read_local_version(char *bts_scr_name)
{
	char read_ver_cmd[] = { 0x01, 0x01, 0x10, 0x00 };

	ST_KIM_DBG("%s", __func__);

	INIT_COMPLETION(kim_gdata->kim_rcvd);
  spin_lock(&st_gdata->lock);
	if (4 != st_int_write(read_ver_cmd, 4)) {
		ST_KIM_ERR("kim: couldn't write 4 bytes");
    spin_unlock(&st_gdata->lock);
		return ST_ERR_FAILURE;
	}
  spin_unlock(&st_gdata->lock);

	if (!wait_for_completion_timeout
	    (&kim_gdata->kim_rcvd, msecs_to_jiffies(CMD_RESP_TIME))) {
		ST_KIM_ERR(" waiting for ver info- timed out ");
		return ST_ERR_FAILURE;
	}

	kim_ver.full =
	    MAKEWORD(kim_gdata->resp_buffer[13], kim_gdata->resp_buffer[14]);
	kim_ver.chip = (kim_ver.full & 0x7C00) >> 10;
	kim_ver.min_ver = (kim_ver.full & 0x007F);
	kim_ver.maj_ver = (kim_ver.full & 0x0380) >> 7;

	if (kim_ver.full & 0x8000)
		kim_ver.maj_ver |= 0x0008;

	sprintf(bts_scr_name, "TIInit_%d.%d.%d.bts", kim_ver.chip, kim_ver.maj_ver, kim_ver.min_ver);
	ST_KIM_DBG("%s", bts_scr_name);
	return ST_SUCCESS;
}

/* internal function which parses through the .bts firmware script file
 * intreprets SEND, DELAY actions only as of now
 */
static long download_firmware(void)
{
	long err = ST_SUCCESS;
	long len = 0;
	register unsigned char *ptr = NULL;
	register unsigned char *action_ptr = NULL;
	unsigned char bts_scr_name[30] = { 0 };	/* 30 char long bts scr name? */

	ST_KIM_VER("%s", __func__);
	ST_LOG("[%s] Enter\n", __func__);

/*[Soldel] - Faster Initscript download time - BEGIN*/
	int wr_room_space;
	int cmd_size;
	unsigned long timeout;
/*[Soldel] - Faster Initscript download time - END*/

	//spin_lock(&st_gdata->lock);//TI HSYoon 20110613

	err = read_local_version(bts_scr_name);
	if (err != ST_SUCCESS) {
		ST_KIM_ERR("kim: failed to read local ver");
		
		//spin_unlock(&st_gdata->lock);//TI HSYoon 20110613
		return err;
	}
	err =
	    request_firmware(&kim_gdata->fw_entry, bts_scr_name,
			     &kim_gdata->kim_pdev->dev);
	if (unlikely((err != 0) || (kim_gdata->fw_entry->data == NULL) ||
		     (kim_gdata->fw_entry->size == 0))) {
		ST_KIM_ERR(" request_firmware failed(errno %ld) for %s", err,
			   bts_scr_name);
		
		//spin_unlock(&st_gdata->lock);//TI HSYoon 20110613
		return ST_ERR_FAILURE;
	}
	ptr = (void *)kim_gdata->fw_entry->data;
	len = kim_gdata->fw_entry->size;
	/* bts_header to remove out magic number and
	 * version
	 */
	ptr += sizeof(struct bts_header);
	len -= sizeof(struct bts_header);
	init_completion(&kim_gdata->kim_rcvd); /*[Soldel] - Faster Initscript download time - ADDED*/
	//spin_unlock(&st_gdata->lock);//TI HSYoon 20110613

	while (len > 0 && ptr) {
		
		ST_KIM_VER(" action size %d, type %d ",
			   ((struct bts_action *)ptr)->size,
			   ((struct bts_action *)ptr)->type);
		
		//spin_lock(&st_gdata->lock);//TI HSYoon 20110613
		
		switch (((struct bts_action *)ptr)->type) {
		case ACTION_SEND_COMMAND:	/* action send */
			action_ptr = &(((struct bts_action *)ptr)->data[0]);
			if (unlikely
			    (((struct hci_command *)action_ptr)->opcode ==
			     0xFF36)) {
				/* ignore remote change
				 * baud rate HCI VS command */
				ST_KIM_ERR
				    (" change remote baud\
				    rate command in firmware");
				break;
			}

      INIT_COMPLETION(kim_gdata->kim_rcvd);
			/*INIT_COMPLETION(kim_gdata->kim_rcvd);*/ /*[Soldel] - Faster Initscript download time - COMMENTED*/
   			
/*[Soldel] - Faster Initscript download time - BEGIN*/			
			/*
			 * Make sure we have enough free space in uart
			 * tx buffer to write current firmware command
			 */
			cmd_size = ((struct bts_action *)ptr)->size;
			timeout = jiffies + msecs_to_jiffies(CMD_WR_TIME);
			do {
				wr_room_space = st_get_uart_wr_room();
				
				if (wr_room_space < 0) {
					pr_err("Unable to get free "
					"space info from uart tx buffer");
					release_firmware(kim_gdata->fw_entry);
					return wr_room_space;
				}
			} while ((wr_room_space < cmd_size) &&
				time_before(jiffies, timeout));

			/* Timeout happened ? */
			if (time_after_eq(jiffies, timeout)) {
				pr_err("Timeout while waiting for free "
					"free space in uart tx buffer");
				release_firmware(kim_gdata->fw_entry);
				return -1;
			}
			/*
			 * Free space found in uart buffer, call st_int_write
			 * to send current firmware command to the uart tx
			 * buffer.
			 */
/*[Soldel] - Faster Initscript download time - END*/
      spin_lock(&st_gdata->lock);
			err = st_int_write(((struct bts_action_send *)
					    action_ptr)->data,
					   ((struct bts_action *)ptr)->size);
      spin_unlock(&st_gdata->lock);
			if (unlikely(err < 0)) {
				release_firmware(kim_gdata->fw_entry);
				return ST_ERR_FAILURE;
			}
/*[Soldel] - Faster Initscript Download time - BEGIN*/
			if (err != cmd_size) {
				ST_KIM_ERR("Number of bytes written to uart "
					"tx buffer are not matching with "
					"requested cmd write size");
				release_firmware(kim_gdata->fw_entry);
				return -1;
			}
			break;
		case ACTION_WAIT_EVENT:
			
/*[Soldel] - Faster initscript download time - END*/
			if (!wait_for_completion_timeout
			    (&kim_gdata->kim_rcvd,
			     msecs_to_jiffies(CMD_RESP_TIME))) {
				ST_KIM_ERR
				    (" response timeout during fw download ");
				/* timed out */
				release_firmware(kim_gdata->fw_entry);
				return ST_ERR_FAILURE;
			}

#if 1//hsyoon 20110616
			init_completion(&kim_gdata->kim_rcvd); /*[Soldel] - Faster Initscript Download time - ADDED*/
#endif
			break;
		case ACTION_DELAY:	/* sleep */
			ST_KIM_DBG("sleep command in scr");
			action_ptr = &(((struct bts_action *)ptr)->data[0]);
			mdelay(((struct bts_action_delay *)action_ptr)->msec);
			break;
		}
		len =
		    len - (sizeof(struct bts_action) +
			   ((struct bts_action *)ptr)->size);
		ptr =
		    ptr + sizeof(struct bts_action) +
		    ((struct bts_action *)ptr)->size;
		
		//spin_unlock(&st_gdata->lock);//TI HSYoon 20110613
	}
	/* fw download complete */
	release_firmware(kim_gdata->fw_entry);
	ST_LOG("[%s] End\n", __func__);
	return ST_SUCCESS;
}

/**********************************************************************/
/* functions called from ST core */

/* function to toggle the GPIO
 * needs to know whether the GPIO is active high or active low
 */
void st_kim_chip_toggle(enum gpio_t gpio_id, enum kim_gpio_state state)
{
	ST_KIM_ERR(" %s ", __func__);
	ST_KIM_ERR("******gpio_id : %d, state : %d, kim_gdata->gpios[gpio_id] : %d\n", gpio_id, state, kim_gdata->gpios[gpio_id]);

	if (gpio_id >= ST_GPIO_MAX || kim_gdata->gpios[gpio_id] == -1) {
		ST_KIM_ERR(" gpio not requested for protocol %s",
			   protocol_names[gpio_id]);
		return;
	}

	switch (gpio_id) {
	case ST_GPIO_BT:
		/*Do Nothing */
		break;

	case ST_GPIO_FM:
		/*
		if (state == KIM_GPIO_ACTIVE)
			gpio_set_value(kim_gdata->gpios[ST_GPIO_FM], GPIO_LOW);
		else
			gpio_set_value(kim_gdata->gpios[ST_GPIO_FM], GPIO_HIGH);
			*/
		break;
	case ST_GPIO_GPS:
		if (state == KIM_GPIO_ACTIVE)
			gpio_set_value(kim_gdata->gpios[ST_GPIO_GPS], GPIO_HIGH);
		else
			gpio_set_value(kim_gdata->gpios[ST_GPIO_GPS], GPIO_LOW);
		break;

	default:
		break;

	}
	return;
}




/* called from ST Core, when REG_IN_PROGRESS (registration in progress)
 * can be because of
 * 1. response to read local version
 * 2. during send/recv's of firmware download
 */
void st_kim_recv(const unsigned char *data, long count)
{
	ST_KIM_DBG(" %s ", __func__);
	
	//spin_lock(&st_gdata->lock);//TI HSYoon 20110613
	/* copy to local buffer */
	if (unlikely(data[4] == 0x01 && data[5] == 0x10 && data[0] == 0x04)) {
		/* must be the read_ver_cmd */
		memcpy(kim_gdata->resp_buffer, data, count);
		complete_all(&kim_gdata->kim_rcvd);
		//spin_unlock(&st_gdata->lock);//TI HSYoon 20110613
		return;
	} else {
		kim_int_recv(data, count);
		/* either completes or times out */
	}
	//spin_unlock(&st_gdata->lock);//TI HSYoon 20110613
	return;
}

/* to signal completion of line discipline installation
 * called from ST Core, upon tty_open
 */
void st_kim_complete(void)
{
	complete(&kim_gdata->ldisc_installed);
}

/* for GPIO_166 pad configuration */
#ifdef CONFIG_MACH_LGE_COSMOPOLITAN
static void pad_config(unsigned long pad_addr, u32 andmask, u32 ormask)
{
	int val;
	u32 *addr;

	addr = (u32 *) ioremap(pad_addr, 4);
	if (!addr) {
		printk(KERN_ERR"OMAP_pad_config: ioremap failed with addr %lx\n",
			pad_addr);
		return;
	}

	val =  __raw_readl(addr);
	val &= andmask;
	val |= ormask;
	__raw_writel(val, addr);

	iounmap(addr);
}
#endif

/* called from ST Core upon 1st registration
*/
long st_kim_start(void)
{
	long err = ST_SUCCESS;
	long retry = POR_RETRY_COUNT;//hsyoon 20110616
	//int is_sent_ldisc_install = 0;//hsyoon 20110612//hsyoon 20110616
    int is_ldisc_on = 0; //cmlee 20110627
	ST_KIM_DBG(" %s", __func__);
  ST_LOG("st_kim_start enter....................\n");

	do {
    
		//pad_config(0x4A100170, 0x0, 0x000B000B); //hsyoon 20110612
		/* Configure BT nShutdown to LOW state */
		gpio_set_value(kim_gdata->gpios[ST_GPIO_BT], GPIO_LOW);
		if(POR_RETRY_COUNT == retry) //hsyoon 20110616
			mdelay(100);	/* FIXME: a proper toggle */ //hsyoon 20110612
		else
			mdelay(1000);


		/* Configure BT nShutdown to HIGH state */
		gpio_set_value(kim_gdata->gpios[ST_GPIO_BT], GPIO_HIGH);
#if 1    /* hsyoon_20110610 */
		mdelay(15);
		gpio_set_value(kim_gdata->gpios[ST_GPIO_BT], GPIO_LOW);
		mdelay(1);	/* FIXME: a proper toggle */
		gpio_set_value(kim_gdata->gpios[ST_GPIO_BT], GPIO_HIGH);
		mdelay(100);	/* FIXME: a proper toggle *///hsyoon 20110612
#endif

		/* re-initialize the completion */
		if(!is_ldisc_on) //cmlee 20110627
		{
			INIT_COMPLETION(kim_gdata->ldisc_installed);//hsyoon 20110616
			/* send signal to UIM */
			err = kill_pid(find_get_pid(kim_gdata->uim_pid), SIGUSR2, 0);
			ST_LOG("sending SIGUSR2 to uim....................\n");

			if (err != 0) {
				ST_LOG(" sending SIGUSR2 to uim failed %ld", err);
				err = ST_ERR_FAILURE;
				continue;
			}
			/* wait for ldisc to be installed */
			err = wait_for_completion_timeout(&kim_gdata->ldisc_installed, msecs_to_jiffies(LDISC_TIME));

			if (!err) {	/* timeout */
				ST_KIM_ERR("line disc installation timed out ");
				err = ST_ERR_FAILURE;
				continue;
			}
		}

    // now ldisc on !
     
    {
      /* ldisc installed now */
      is_ldisc_on = 1;  // cmlee
      ST_KIM_DBG(" line discipline installed ");
      err = download_firmware();
      if (err != ST_SUCCESS)
      {
        ST_KIM_ERR("download firmware failed");
        continue;
      }
      else
      { /* on success don't retry */
        struct timespec ts;  
        struct rtc_time tm;  
        getnstimeofday(&ts);
        rtc_time_to_tm(ts.tv_sec, &tm);  

        ST_KIM_ERR("%d-%02d-%02d %02d:%02d:%02d.%09lu download firmware success",tm.tm_year + 1900, tm.tm_mon + 1, tm.tm_mday,  tm.tm_hour, tm.tm_min, tm.tm_sec, ts.tv_nsec);
        break;
      }
	}
	} while (retry--);
	return err;
}


/* called from ST Core, on the last un-registration
*/
long st_kim_stop(void)
{
	long err = ST_SUCCESS;
	int timeout_cnt;//hsyoon 20110616
	
  timeout_cnt = 2000;//hsyoon 20110616

	INIT_COMPLETION(kim_gdata->ldisc_installed);
	/* send signal to UIM */
	err = kill_pid(find_get_pid(kim_gdata->uim_pid), SIGUSR2, 1);
	if (err != 0) {
		ST_KIM_ERR("sending SIGUSR2 to uim failed %ld", err);
		return ST_ERR_FAILURE;
	}

	/* wait for ldisc to be un-installed */
	err = wait_for_completion_timeout(&kim_gdata->ldisc_installed,
			msecs_to_jiffies(LDISC_TIME));
	if (!err) {		/* timeout */
		ST_KIM_ERR(" timed out waiting for ldisc to be un-installed");
		//return ST_ERR_FAILURE; //hsyoon 20110612
	}

		/* By default configure BT nShutdown to LOW state */
	gpio_set_value(kim_gdata->gpios[ST_GPIO_BT], GPIO_LOW);
		mdelay(1);
	gpio_set_value(kim_gdata->gpios[ST_GPIO_BT], GPIO_HIGH);
		mdelay(1);
	gpio_set_value(kim_gdata->gpios[ST_GPIO_BT], GPIO_LOW);
	memset((void*)&kim_ver, 0, sizeof(kim_ver_t));
	return err;
}

/**********************************************************************/
/* functions called from subsystems */

/* called when sysfs entry is written to */
static ssize_t store_pid(struct device *dev, struct device_attribute
			 *devattr, char *buf, size_t count)
{
	ST_KIM_DBG("%s: pid %s ", __func__, buf);
	sscanf(buf, "%ld", &kim_gdata->uim_pid);
	/* to be made use by kim_start to signal SIGUSR2
	 */
	return strlen(buf);
}

/* called when sysfs entry is read from */
static ssize_t show_pid(struct device *dev, struct device_attribute
			*attr, char *buf)
{
	sprintf(buf, "%ld", kim_gdata->uim_pid);
	return strlen(buf);
}

/* called when sysfs entry is read from */
static ssize_t show_list(struct device *dev, struct device_attribute
			 *attr, char *buf)
{
	kim_st_list_channels(buf);
	return strlen(buf);
}

/* called when sysfs entry is read from */
static ssize_t show_version(struct device *dev, struct device_attribute
			 *attr, char *buf)
{
	sprintf(buf, "%04X %d.%d.%d", kim_ver.full, kim_ver.chip, kim_ver.maj_ver, kim_ver.min_ver);
	return strlen(buf);
}

#ifdef LEGACY_RFKILL_SUPPORT
/* function called from rfkill subsystem, when someone from
 * user space would write 0/1 on the sysfs entry
 * /sys/class/rfkill/rfkill0,1,3/state
 */
static int kim_toggle_radio(void *data, enum rfkill_state state)
{
	enum gpio_t gpio_id = (unsigned char)data;
	unsigned char protoid = 0;

	ST_KIM_ERR(" %s ", __func__);
	if (gpio_id == ST_GPIO_BT)
	{
		/* Configure BT nShutdown to HIGH state */
		gpio_set_value(kim_gdata->gpios[ST_GPIO_BT], GPIO_LOW);
		mdelay(1);	/* FIXME: a proper toggle */
		gpio_set_value(kim_gdata->gpios[ST_GPIO_BT], GPIO_HIGH);

		if (state == RFKILL_STATE_SOFT_BLOCKED) {
			gpio_set_value(kim_gdata->gpios[ST_GPIO_BT], GPIO_LOW);
		}
	}
	else if (gpio_id == ST_GPIO_FM || gpio_id == ST_GPIO_GPS)
	{
		switch (state) 
		{
			case RFKILL_STATE_UNBLOCKED:
				st_kim_chip_toggle(gpio_id, KIM_GPIO_ACTIVE);
		break;
			case RFKILL_STATE_SOFT_BLOCKED:
				st_kim_chip_toggle(gpio_id, KIM_GPIO_INACTIVE);
		break;
			default:
				ST_LOG(KERN_ERR "invalid rfkill state %d", state);
		}
	}
	else
	{
		ST_LOG(KERN_ERR "invalid gpio %d", gpio_id);
	}
	mdelay(5);
	return ST_SUCCESS;
}
#endif

/**********************************************************************/
/* functions called from platform device driver subsystem
 * need to have a relevant platform device entry in the platform's
 * board-*.c file
 */

static int kim_probe(struct platform_device *pdev)
{
	long status;
	long gpio_id;
	long *gpios = pdev->dev.platform_data;

	for (gpio_id = 0; gpio_id < ST_GPIO_MAX; gpio_id++) {
		kim_gdata->gpios[gpio_id] = gpios[gpio_id];
		ST_KIM_VER(" %ld gpio to be requested", gpios[gpio_id]);
	}

	for (gpio_id = 0; (gpio_id < ST_GPIO_MAX) && (gpios[gpio_id] != -1); gpio_id++) {
		if  (gpios[gpio_id] == -1) {
		    continue;
		}
		/* Claim the Bluetooth/FM/GPIO
		 * nShutdown gpio from the system
		 */
		status = gpio_request(gpios[gpio_id], "kim");
		ST_KIM_VER("gpio_request(%d): status %d", gpios[gpio_id], status);
		if (unlikely(status)) {
			ST_KIM_ERR(" gpio %ld request failed ", gpios[gpio_id]);
// TI_SOLDEL cmlee 2011/03/14 moved in while loop [START]
			gpio_id -= 1;
			while (gpio_id >= 0) {
				if (gpios[gpio_id] != -1)
					gpio_free(gpios[gpio_id]);
        gpio_id -= 1;
// TI_SOLDEL cmlee 2011/03/14 moved in while loop [END]
			}
			return status;
		}

		/* Configure nShutdown GPIO as output=0 */
		status =
		    gpio_direction_output(gpios[gpio_id], 0);
		ST_KIM_VER("gpio_direction_output(%d): status %d", gpios[gpio_id], status);
		if (unlikely(status)) {
			ST_KIM_ERR(" unable to configure gpio %ld",
				   gpios[gpio_id]);
// TI_SOLDEL cmlee 2011/03/14 moved in while loop [START]
			gpio_id -= 1;
			while (gpio_id >= 0) {
				if (gpios[gpio_id] != -1)
					gpio_free(gpios[gpio_id]);
        gpio_id -= 1;
// TI_SOLDEL cmlee 2011/03/14 moved in while loop [END]
			}
			return status;
		}
	}
	/* pdev to contain BT, FM and GPS enable/N-Shutdown GPIOs
	 * execute request_gpio, set output direction
	 */
	kim_gdata->kim_kobj = kobject_create_and_add("uim", NULL);
	/* create the sysfs entry for UIM to put in pid */
	if (sysfs_create_group(kim_gdata->kim_kobj, &uim_attr_grp)) {
		ST_KIM_ERR(" sysfs entry creation failed");
		kobject_put(kim_gdata->kim_kobj);
		/* free requested GPIOs and fail probe */
		for (gpio_id = 0; gpio_id < ST_GPIO_MAX; gpio_id++) {
			if (gpios[gpio_id] != -1)
				gpio_free(gpios[gpio_id]);
		}
		return -1;	/* fail insmod */
	}

	ST_KIM_DBG(" sysfs entry created ");

	/* get reference of pdev for request_firmware
	 */
	kim_gdata->kim_pdev = pdev;
	init_completion(&kim_gdata->kim_rcvd);
	init_completion(&kim_gdata->ldisc_installed);
#ifdef LEGACY_RFKILL_SUPPORT
	for (gpio_id = 0; gpio_id < ST_GPIO_MAX; gpio_id++) {
	    if (gpios[gpio_id] == -1) {
	        continue;
	    }
		/* TODO: should all types be rfkill_type_bt ? */
		kim_gdata->rfkill[gpio_id] = rfkill_allocate(&pdev->dev,
					   RFKILL_TYPE_BLUETOOTH);
		if (kim_gdata->rfkill[gpio_id] == NULL) {
			ST_KIM_ERR("cannot create rfkill entry for gpio %ld",
				   gpios[gpio_id]);
			continue;
		}
		kim_gdata->rfkill[gpio_id]->name = protocol_names[gpio_id];
		kim_gdata->rfkill[gpio_id]->state = RFKILL_STATE_SOFT_BLOCKED;
		kim_gdata->rfkill[gpio_id]->user_claim_unsupported = 1;
		kim_gdata->rfkill[gpio_id]->user_claim = 0;
		/* sending protocol id instead of gpio */
		kim_gdata->rfkill[gpio_id]->data = (void *)gpio_id;
		kim_gdata->rfkill[gpio_id]->toggle_radio = kim_toggle_radio;
		status = rfkill_register(kim_gdata->rfkill[gpio_id]);
		if (unlikely(status)) {
			ST_KIM_ERR("rfkill registration failed for gpio %ld",
				   gpios[gpio_id]);
			rfkill_free(kim_gdata->rfkill[gpio_id]);
			continue;
		}
		ST_KIM_DBG("rfkill entry created for %ld", gpios[gpio_id]);
	}
#endif


	return ST_SUCCESS;
}

static int kim_remove(struct platform_device *pdev)
{
	/* free the GPIOs requested
	 */
	long *gpios = pdev->dev.platform_data;
	long gpio_id;

	for (gpio_id = 0; gpio_id < ST_GPIO_MAX; gpio_id++) {
	    if (gpios[gpio_id] == -1) {
	        continue;
	    }
		/* Claim the Bluetooth/FM/GPIO
		 * nShutdown gpio from the system
		 */
		gpio_free(gpios[gpio_id]);
#ifdef LEGACY_RFKILL_SUPPORT
		rfkill_unregister(kim_gdata->rfkill[gpio_id]);
		kim_gdata->rfkill[gpio_id] = NULL;
#endif
	}
	ST_KIM_DBG("kim: GPIO Freed");
	/* delete the sysfs entries */
	sysfs_remove_group(kim_gdata->kim_kobj, &uim_attr_grp);
	kobject_put(kim_gdata->kim_kobj);
	kim_gdata->kim_pdev = NULL;
	return ST_SUCCESS;
}

/**********************************************************************/
/* entry point for ST KIM module, called in from ST Core */

long st_kim_init(void)
{
	long ret = ST_SUCCESS;
	kim_gdata = kzalloc(sizeof(struct kim_data_s), GFP_ATOMIC);
	if (!kim_gdata) {
		ST_KIM_ERR("no mem to allocate");
		return -ENOMEM;
	}
	ret = platform_driver_register(&kim_platform_driver);
	if (ret != 0) {
		ST_KIM_ERR("platform drv registration failed");
		return ST_ERR_FAILURE;
	}
	memset((void*)&kim_ver, 0, sizeof(kim_ver_t));
	return ST_SUCCESS;
}

long st_kim_deinit(void)
{
	/* the following returns void */
	platform_driver_unregister(&kim_platform_driver);
	kfree(kim_gdata);
	kim_gdata = NULL;
	return ST_SUCCESS;
}
