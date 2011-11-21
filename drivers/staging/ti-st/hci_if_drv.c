/*
 *   TI BT Char Driver for Texas Instrument's Connectivity Chip.
 *   Copyright (C) 2009 Texas Instruments
 *
 *   This program is free software; you can redistribute it and/or modify
 *   it under the terms of the GNU General Public License version 2 as
 *   published by the Free Software Foundation.
 *
 *   This program is distributed in the hope that it will be useful,
 *   but WITHOUT ANY WARRANTY; without even the implied warranty of
 *   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *   GNU General Public License for more details.
 *
 *   You should have received a copy of the GNU General Public License
 *   along with this program; if not, write to the Free Software
 *   Foundation, Inc.,59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 */

#include <linux/cdev.h>
#include <linux/fs.h>
#include <linux/device.h>
#include <linux/uaccess.h>
#include <linux/tty.h>
#include <linux/sched.h>

#include <linux/delay.h>
#include <linux/firmware.h>
#include <linux/platform_device.h>
#include <linux/poll.h>
#include <linux/skbuff.h>
#include <linux/interrupt.h>
#include <linux/list.h>

#include <net/bluetooth/bluetooth.h>

#include "hci_if_drv.h"

/***********************************************************************
* 							Defenitions								     *
***********************************************************************/
#define DEVICE_NAME     "tihci"

static long hciif_st_event_recv(struct sk_buff *skb);
static long hciif_st_ganeric_recv(struct sk_buff *skb);
static void hciif_st_open_cb(char data);
static int hciif_register_client(struct st_proto_s *hciif_proto);
static struct hciif_client *hciif_add_client_to_array(int channel, unsigned char evt_type[], 
												struct hciif_client *existingClient);
static int hciif_enter_cs(void);
static void hciif_exit_cs(char status);

/***********************************************************************
* 							Variables									     *
***********************************************************************/
static struct hciif_handle *hhciif;

static struct st_proto_s hciif_proto[ST_MAX_CHANNELS];

/***********************************************************************
* 							   APIs									     *
***********************************************************************/
/*********Functions Called from BT host***************************************/

/** hciif_open Function
 *  This function will perform an register on ST driver.
 *  Parameters :
 *  @file  : File pointer for BT char driver
 *  @inod  :
 *  Returns  HCIIF_SUCCESS -  on success
 *           else suitable error code
 */
int hciif_open(struct inode *inod, struct file *file)
{
	HCIIFDRV_DBG(" Inside %s", __func__);
	return HCIIF_SUCCESS;
}

/** hciif_release Function
 *  This function will un-registers from the ST driver.
 *
 *  Parameters :
 *  @file  : File pointer for BT char driver
 *  @inod  :
 *  Returns  HCIIF_SUCCESS -  on success
 *           else suitable error code
 */
int hciif_release(struct inode *inod, struct file *file)
{
	return hciif_dev_down((struct hciif_client *)file->private_data);
}

/** hciif_read Function
 *  This function will wait till the data received from the ST driver
 *  and then send it to BT host application.
 *
 *  Parameters :
 *  @file  : File pointer for BT char driver
 *  @data  : Data which needs to be passed to APP
 *  @size  : Length of the data passesd
 *  offset :
 *  Returns  Size of packet received -  on success
 *           else suitable error code
 */
ssize_t hciif_read(struct file *file, char __user *data, size_t size,
		    loff_t *offset)
{
	int len = 0;
	struct sk_buff *skb = NULL;
	struct hciif_client *client;

	unsigned long timeout = HCIIFDRV_READ_TIMEOUT;

	HCIIFDRV_DBG(" Inside %s", __func__);

	/* Validate input parameters */
	if ((NULL == file) || (((NULL == data) || (0 == size)))) {
		HCIIFDRV_ERR("Invalid input parameters passed to %s",
			   __func__);
		return -EINVAL;
	}

	client = file->private_data;

	HCIIFDRV_DBG(" read: client = 0x%08x", (unsigned int)client);

	/* cannot come here if poll-ed before reading
	 * if not poll-ed wait on the same wait_q
	 */
	timeout = wait_event_interruptible_timeout(client->hciif_client_q,
		!skb_queue_empty(&client->rx_list),
		msecs_to_jiffies(timeout));
	/* Check for timed out condition */
	if (0 == timeout) {
		HCIIFDRV_ERR("BT Device Read timed out");
		return HCIIF_ERR_TIMEOUT;
	}
		
	HCIIFDRV_VER(" Read wait completed, sending packet");

	/* client->rx_list not empty, skb already present */
	spin_lock(&client->rx_lock);
	skb = skb_dequeue(&client->rx_list);
	spin_unlock(&client->rx_lock);
	
	if (!skb) {
		HCIIFDRV_ERR("Dequed SKB is NULL?");
		return HCIIF_ERR_UNKNOWN;
	}

	HCIIFDRV_VER(" Before adding PKT_TYPE: skb->data[0] = %x", skb->data[0]);
	/* Add packet type to skb data */
#if 1    /* TIK_BT OPP fail issue */
	spin_lock(&client->rx_lock);
#endif
	memcpy(skb_push(skb, 1), &(bt_cb(skb)->pkt_type), 1);
	HCIIFDRV_VER(" After adding PKT_TYPE: skb->data[0] = %x", skb->data[0]);
#if 1    /* TIK_BT OPP fail issue */
	spin_unlock(&client->rx_lock);
#endif
	
	if (skb->len > size) {
		HCIIFDRV_DBG("SKB length is Greater than requested size \
			Returning the requested size");

#if 1    /* TIK_BT OPP fail issue */
		spin_lock(&client->rx_lock);
#endif
		copy_to_user(data, skb->data, size);
		skb_pull(skb, size);
#if 1    /* TIK_BT OPP fail issue */
		spin_unlock(&client->rx_lock);
#endif
		
		if (skb->len != 0) {
			spin_lock(&client->rx_lock);
			skb_queue_head(&client->rx_list, skb);
			spin_unlock(&client->rx_lock);
		}
			
		printk(KERN_DEBUG  "gpsdrv: total size read= %d", size);
		return size;
	}

#ifdef VERBOSE
	for (len = 0; (skb) && (len < skb->len); len++)
		HCIIFDRV_DBG(" %x ", skb->data[len]);
#endif

	HCIIFDRV_VER(" Before copying to user...");
	/* Forward the data to the user --> case (skb->len <= size)*/
#if 1    /* TIK_BT OPP fail issue */
	spin_lock(&client->rx_lock);
	//printk("[%s] before copy_to_user\n", __func__);//hsyoon
	if(copy_to_user(data, skb->data, skb->len)) 
	{
		HCIIFDRV_ERR(" Unable to copy to user space");
		/* Queue the skb back to head */
		skb_queue_head(&client->rx_list, skb);
		//printk("[%s] after copy_to_user\n", __func__);//hsyoon
		spin_unlock(&client->rx_lock);
		return HCIIF_ERR_CPY_TO_USR;
	}
	//printk("[%s] after copy_to_user\n", __func__);//hsyoon
	spin_unlock(&client->rx_lock);
#else
	if(copy_to_user(data, skb->data, skb->len)) 
	{
		HCIIFDRV_ERR(" Unable to copy to user space");
		/* Queue the skb back to head */
		spin_lock(&client->rx_lock);
		skb_queue_head(&client->rx_list, skb);
		spin_unlock(&client->rx_lock);
		return HCIIF_ERR_CPY_TO_USR;
	}
#endif

	HCIIFDRV_VER(" After copying to user...");
	len = skb->len;
	kfree_skb(skb);

	HCIIFDRV_DBG(" Exit %s", __func__);
	return len;
}

/** hciif_write Function
 *  This function will forward the incoming packet 
 *  sent from the BT host application, to the ST Core.
 *
 *  Parameters :
 *  @file   : File pointer for BT char driver
 *  @data   : HCI packet data from BT application
 *  @size   : Size of the packet data
 *  @offset :
 *  Returns  Size of packet on success
 *           else suitable error code
 */
ssize_t hciif_write(struct file *file, const char __user *data,
		     size_t size, loff_t *offset)
{
	struct hciif_client 	*client;
	struct sk_buff 		*skb;

	HCIIFDRV_DBG(" Inside %s", __func__);

	/* Validate input parameters */
	if ((NULL == file) || (((NULL == data) || (0 == size)))) {
		HCIIFDRV_ERR("Invalid input parameters passed to %s", __func__);
		return -EINVAL;
	}

	client = file->private_data;

	skb = alloc_skb(size, GFP_ATOMIC);

	/* Validate Created SKB */
	if (NULL == skb) {
		HCIIFDRV_ERR("Error aloacting SKB");
		return -ENOMEM;
	}

	/* Forward the data from the user space to ST core */
	if (copy_from_user(skb_put(skb, size), data, size)) {
		HCIIFDRV_ERR(" Unable to copy from user space");
		kfree_skb(skb);
		return HCIIF_ERR_CPY_FRM_USR;
	}

	return hciif_send_frame(skb, client);
}

/** hciif_ioctl Function
 *  This will peform the functions as directed by the command and command
 *  argument.
 *
 *  Parameters :
 *  @file  : File pointer for BT char driver
 *  @cmd   : IOCTL Command
 *  @arg   : Command argument for IOCTL command
 *  Returns  HCIIF_SUCCESS on success
 *           else suitable error code
 */
static int hciif_ioctl(struct inode *inode, struct file *file,
			unsigned int cmd, unsigned long arg)
{
	struct sk_buff *skb = NULL;
	int retCode = HCIIF_SUCCESS;
	struct hciif_client *client, *tmp;
	
	HCIIFDRV_DBG(" Inside %s", __func__);

	/* Validate input parameters */
	if ((NULL == file) || (0 == cmd)) {
		HCIIFDRV_ERR("Invalid input parameters passed to %s",
			   __func__);
		return -EINVAL;
	}

	client = file->private_data;

	switch (cmd) {
		case TCFLSH:
			HCIIFDRV_VER(" IOCTL TCFLSH invoked with %ld argument", arg);
			spin_lock(&client->rx_lock);
			switch (arg) {
				/* purge Rx/Tx SKB list queues depending on arg value */
				case TCIFLUSH:
					skb_queue_purge(&client->rx_list);
					break;
				case TCOFLUSH:
					/* Not Supporeted, no Tx queue */
					break;
				case TCIOFLUSH:
					skb_queue_purge(&client->rx_list);
					break;
				default:
					HCIIFDRV_ERR("Invalid Command passed for tcflush");
					retCode = -EINVAL;
					break;
			}
			spin_unlock(&client->rx_lock);
		break;
	
		case FIONREAD:
			/* Deque the SKB from the head if rx_list is not empty
			 * And update the argument with skb->len to provide
			 * amount of data available in the available SKB
			 */
			 HCIIFDRV_VER(" IOCTL FIONREAD invoked");
			spin_lock(&client->rx_lock);
			if (!skb_queue_empty(&client->rx_list)) {
				skb = skb_dequeue(&client->rx_list);
				if (skb != NULL) {
					*(unsigned int *)arg = skb->len + 1; /* +1 --> for packet type */
					/* Re-Store the SKB for future RD operations */
					skb_queue_head(&client->rx_list, skb);
				} else
					*(unsigned int *)arg = 0;
			} else {
				*(unsigned int *)arg = 0;
			}
			spin_unlock(&client->rx_lock);
		break;
		
		/* HCI IF IOCTLs */
		case HCIIF_IOCTL_DEVUP:
        {
            struct hciif_filter_t   filter;
			/* Indicates to bring up the device for the channels and events that are sent as an argument */
			HCIIFDRV_VER(" IOCTL HCIIF_IOCTL_DEVUP invoked");
            copy_from_user(&filter, arg, sizeof(struct hciif_filter_t));
            retCode = hciif_dev_up(file, &filter, NULL, &tmp);
        }

		break;

		default:
			HCIIFDRV_DBG("Un-Identified command provided for IOCTL");
			retCode = -EINVAL;
		break;
	}

	HCIIFDRV_DBG(" Exit %s", __func__);
    
	return retCode;
}

/** hciif_poll Function
 *  This function will wait till some data is received to the BT driver from ST
 *
 *  Parameters :
 *  @file  : File pointer for TI BT char driver
 *  @wait  : POLL wait information
 *  Returns  status of POLL on success
 *           else suitable error code
 */
static unsigned int hciif_poll(struct file *file, poll_table *wait)
{
	unsigned long mask = 0;
	struct hciif_client *client;

	client = file->private_data;
	
	/* Wait till data is signalled from hciif_st_recv function */
	poll_wait(file, &client->hciif_client_q, wait);

	if (!skb_queue_empty(&client->rx_list))
		mask |= POLLIN;	/* TODO: check app for mask */

	return mask;
}

/* BT Char driver function pointers
 * These functions are called from USER space by pefroming File Operations
 * on /dev/hciif node exposed by this driver during init
 */
const struct file_operations hciif_chrdev_ops = {
	.owner = THIS_MODULE,
	.open = hciif_open,
	.read = hciif_read,
	.write = hciif_write,
	.ioctl = hciif_ioctl,
	.poll = hciif_poll,
	.release = hciif_release,
};

/*********Other APIs***************************************/

/** hciif_init Function
 *  This function Initializes the TI BT driver parametes and exposes
 *  /dev/hciif node to user space
 *
 *  Parameters : NULL
 *  Returns  HCIIF_SUCCESS on success
 *           else suitable error code
 */
int hciif_init(void)
{
	long err = 0;
	int i = 0;

	HCIIFDRV_DBG(" Inside %s", __func__);

	/* Allocate local resource memory */
	hhciif = kzalloc(sizeof(struct hciif_handle), GFP_KERNEL);
	if (!(hhciif)) {
		HCIIFDRV_ERR("Can't allocate BT data structure");
		return -ENOMEM;
	}

	/* Expose the device DEVICE_NAME to user space
	 * And obtain the major number for the device
	 */
	hhciif->hciif_major =
	    register_chrdev(0, DEVICE_NAME, &hciif_chrdev_ops);
	if (0 > hhciif->hciif_major) {
		err = hhciif->hciif_major;
		HCIIFDRV_ERR("Error when registering to char dev. Error = %ld.",
			   err);
		kfree(hhciif);
		return err;
	}
	HCIIFDRV_VER(" %ld: allocated %d, %d", err, hhciif->hciif_major, 0);

	/* udev */
	hhciif->hciif_class = class_create(THIS_MODULE, DEVICE_NAME);
	if (IS_ERR(hhciif->hciif_class)) {
		HCIIFDRV_ERR(" Something went wrong in class_create");
		unregister_chrdev(hhciif->hciif_major, DEVICE_NAME);
		kfree(hhciif);
		return HCIIF_ERR_CLASS;
	}

	hhciif->hciif_dev =
	    device_create(hhciif->hciif_class, NULL,
			  MKDEV(hhciif->hciif_major, 0), NULL, DEVICE_NAME);
	if (IS_ERR(hhciif->hciif_dev)) {
		err = PTR_ERR(hhciif->hciif_dev);
		HCIIFDRV_ERR(" Error in class_create. Error = %ld", err);
		class_unregister(hhciif->hciif_class);
		class_destroy(hhciif->hciif_class);
		unregister_chrdev(hhciif->hciif_major, DEVICE_NAME);
		kfree(hhciif);
		return err;
	}

	/* Initialize registration complete strucuture */
	init_completion(&hhciif->hciif_reg_completed);

	/* Initialize registration complete strucuture */
	init_completion(&hhciif->hciif_oper_completed);

	/* Initialize Cmds Clients counter */
	hhciif->hciif_cmdClientsCounter = 0;

	/* Initialize Clients Array */
	for(i = 0; i < ST_MAX_CHANNELS; i++)
	{
		hhciif->hciif_RawClients[i] = NULL;
	}
	for(i = 0; i < HCIIF_CMD_CLIENTS_NUM; i++)
	{
		hhciif->hciif_CmdClients[i] = NULL;
	}

	/* Initialize The Write Clients List */
	INIT_LIST_HEAD(&(hhciif->hciif_WriteClientsList.list));

	/* Initialize The Write Clients List lock */
	spin_lock_init(&hhciif->hciif_lock);

	/* Init 'st_proto_s' Static Array */
	for(i = 0; i < ST_MAX_CHANNELS; i++)
	{
		hciif_proto[i].channelid= i;
		hciif_proto[i].recv = hciif_st_ganeric_recv;
		hciif_proto[i].reg_complete_cb = hciif_st_open_cb;
		hciif_proto[i].gpio_id = ST_GPIO_MAX;
		hciif_proto[i].max_frame_size = HCIIF_FRAME_SIZE_DEF;
		hciif_proto[i].header_size = HCIIF_HDR_SIZE_DEF;
		hciif_proto[i].length_offset = HCIIF_LEN_OFFSET_DEF;
		hciif_proto[i].length_size = HCIIF_LEN_SIZE_DEF;
	}
	
	hciif_proto[HCIIF_CHAN_ACL].max_frame_size = HCIIF_FRAME_SIZE_ACL;
	hciif_proto[HCIIF_CHAN_ACL].header_size = HCIIF_HDR_SIZE_ACL;
	hciif_proto[HCIIF_CHAN_ACL]	.length_offset = HCIIF_LEN_OFFSET_ACL;
	hciif_proto[HCIIF_CHAN_ACL].length_size = HCIIF_LEN_SIZE_ACL;
	
	hciif_proto[HCIIF_CHAN_SCO].max_frame_size = HCIIF_FRAME_SIZE_SCO;
	hciif_proto[HCIIF_CHAN_SCO].header_size = HCIIF_HDR_SIZE_SCO;
	hciif_proto[HCIIF_CHAN_SCO].length_offset = HCIIF_LEN_OFFSET_SCO;
	hciif_proto[HCIIF_CHAN_SCO].length_size = HCIIF_LEN_SIZE_SCO;

	hciif_proto[HCIIF_CHAN_EVT].recv = hciif_st_event_recv;
	hciif_proto[HCIIF_CHAN_EVT].max_frame_size = HCIIF_FRAME_SIZE_EVT;
	hciif_proto[HCIIF_CHAN_EVT].header_size = HCIIF_HDR_SIZE_EVT;
	hciif_proto[HCIIF_CHAN_EVT].length_offset = HCIIF_LEN_OFFSET_EVT;
	hciif_proto[HCIIF_CHAN_EVT].length_size = HCIIF_LEN_SIZE_EVT;

	hciif_proto[HCIIF_CHAN_FM].gpio_id = ST_GPIO_FM;
	hciif_proto[HCIIF_CHAN_FM].max_frame_size = HCIIF_FRAME_SIZE_FM;
	hciif_proto[HCIIF_CHAN_FM].header_size = HCIIF_HDR_SIZE_FM;
	hciif_proto[HCIIF_CHAN_FM].length_offset = HCIIF_LEN_OFFSET_FM;
	hciif_proto[HCIIF_CHAN_FM].length_size = HCIIF_LEN_SIZE_FM;


	return HCIIF_SUCCESS;
}

/** hciif_exit Function
 *  This function Destroys the TI BT driver parametes and /dev/hciif node
 *
 *  Parameters : NULL
 *  Returns   NULL
 */
void hciif_exit(void)
{
	int i = 0;

	HCIIFDRV_DBG(" Inside %s", __func__);
	HCIIFDRV_VER(" Bye.. freeing up %d", hhciif->hciif_major);

	device_destroy(hhciif->hciif_class, MKDEV(hhciif->hciif_major, 0));

	class_unregister(hhciif->hciif_class);
	class_destroy(hhciif->hciif_class);

	unregister_chrdev(hhciif->hciif_major, DEVICE_NAME);

	for(i = 0; i < ST_MAX_CHANNELS; i++)
	{
		if(hhciif->hciif_RawClients[i])
			kfree(hhciif->hciif_RawClients[i]);
	}

	for(i = 0; i < HCIIF_CMD_CLIENTS_NUM; i++)
	{
		if(hhciif->hciif_CmdClients[i])
			kfree(hhciif->hciif_CmdClients[i]);
	}

	kfree(hhciif);
}

int hciif_dev_up(struct file *file, struct hciif_filter_t *rcv_filter, 
					struct hciif_hciCore_t *hciCore, struct hciif_client 	**client)
{
	int i, retCode = HCIIF_SUCCESS;
	struct hciif_client 	*newClient = NULL;
	struct hciif_filter_t	filter;

	HCIIFDRV_DBG(" Inside %s", __func__);
	
	retCode = hciif_enter_cs();
	if(retCode != HCIIF_SUCCESS)
		return retCode;
	
    memcpy(&filter, rcv_filter, sizeof(struct hciif_filter_t));

	if(test_bit(HCIIF_CHAN_EVT, &filter.chan_mask))
	{
		if(hhciif->hciif_cmdClientsCounter == HCIIF_CMD_CLIENTS_NUM)
		{
			HCIIFDRV_ERR("Trying to regsiter a client to Events channel --> Max clients are already registered");
			hciif_exit_cs(-EINPROGRESS);
			return -EINVAL;
		}
				
		if(hhciif->hciif_cmdClientsCounter == 0)
		{
			retCode = hciif_register_client(&hciif_proto[HCIIF_CHAN_EVT]);
		}

		if(retCode == HCIIF_SUCCESS)
		{
			newClient = hciif_add_client_to_array(HCIIF_CHAN_EVT, filter.evt_type, NULL);
			if(!newClient)
			{
				HCIIFDRV_ERR("Failed adding client to array");
				hciif_exit_cs(-EINPROGRESS);
				return -EINVAL;
			}
		}
		else
		{
			HCIIFDRV_ERR("hciif_register_client failed");
			hciif_exit_cs(-EINPROGRESS);
			return -EINVAL;
		}
	}
			
	for(i = 0; i < ST_MAX_CHANNELS; i++)
	{
		if(i == HCIIF_CHAN_EVT)
			continue;
			
		if(test_bit(i, &filter.chan_mask))
		{
			if(hhciif->hciif_RawClients[i])
			{
				HCIIFDRV_ERR("There is already a client registered to channel %d", i);
				hciif_exit_cs(-EINPROGRESS);
				return -EINVAL;
			}
			else
			{
				retCode = hciif_register_client(&hciif_proto[i]);
				if(retCode == HCIIF_SUCCESS)
				{
					newClient = hciif_add_client_to_array(i, NULL, newClient);
					if(!newClient)
					{
						HCIIFDRV_ERR("Failed adding client to array");
						hciif_exit_cs(-EINPROGRESS);
						return -EINVAL;
					}
				}
			}
		}
	}

	if(newClient)
	{
		/* Update client's struct with some more params */
		memcpy(&newClient->filter, &filter, sizeof(struct hciif_filter_t));
		
		if(hciCore)
		{
			newClient->file = NULL;
			newClient->hci_core.is_hci_core = hciCore->is_hci_core;
			newClient->hci_core.hci_core_recv = hciCore->hci_core_recv;
			*client = newClient;
		}
		else
		{
			newClient->file = file;
			newClient->hci_core.is_hci_core = false;
			newClient->hci_core.hci_core_recv = NULL;
			/* Set pointer to client struct in file */
			file->private_data = newClient;
		}
	}
	else
	{
		HCIIFDRV_ERR("No channels were set");
		hciif_exit_cs(-EINPROGRESS);
		return -EINVAL;
	}

	hciif_exit_cs(0);

	HCIIFDRV_DBG(" Exit %s", __func__);
	
	return HCIIF_SUCCESS;
}

int hciif_dev_down(struct hciif_client *client)
{
	int retVal = HCIIF_SUCCESS;
	int i = 0;
	unsigned long flags = 0;
	unsigned long    chan_mask;
	struct hciif_client *tmpClient;
	struct list_head *pos, *q;


	retVal = hciif_enter_cs();
	if(retVal != HCIIF_SUCCESS)
		return retVal;
	
	/* Complete 'release' call is portected by the lock */
	spin_lock_irqsave(&hhciif->hciif_lock, flags);

	HCIIFDRV_DBG(" Inside %s", __func__);

	if(!client)
	{
		spin_unlock_irqrestore(&hhciif->hciif_lock, flags);	/*[Soldel] - BT/FM Simultaneous Turn On*/
		HCIIFDRV_VER("client is NULL, nothing to do... Exit");
		hciif_exit_cs(0);
		
		return retVal;
	}
	
	chan_mask = client->filter.chan_mask;
	
	/* First handle EVENT CHANNEL and its special array */
	if(test_bit(HCIIF_CHAN_EVT, &chan_mask))
	{
		/* Find Client Struct */
		for(i = 0; i < HCIIF_CMD_CLIENTS_NUM; i ++)
		{
			if(hhciif->hciif_CmdClients[i] == client) 
				break;
		}

		if(i == HCIIF_CMD_CLIENTS_NUM)
		{
			spin_unlock_irqrestore(&hhciif->hciif_lock, flags);
			HCIIFDRV_ERR("Client is not registered to EVT CHANNEL, Exit!");

			hciif_exit_cs(-EINPROGRESS);
			
			return HCIIF_ERR_UNKNOWN;
		}

		/* Remove client from array */
		hhciif->hciif_CmdClients[i] = NULL;

		/* Update Clients counter */
		hhciif->hciif_cmdClientsCounter--; 

		/* Align rest of the clients to the begining of the array */
		for(i = i+1; (i < HCIIF_CMD_CLIENTS_NUM-1) && (hhciif->hciif_CmdClients[i] != NULL); i++)
		{
			hhciif->hciif_CmdClients[i-1] = hhciif->hciif_CmdClients[i];
			hhciif->hciif_CmdClients[i]  = NULL; /*[Soldel] BT/FM Simultaneous Turn On  */
		}	

		/* Remove this client from the hciif_WriteClientsList */
		list_for_each_safe(pos, q, &hhciif->hciif_WriteClientsList.list)
		{
			tmpClient = list_entry(pos, struct hciif_client, list);
			if(tmpClient == client)
				list_del(pos);
		}
		
		/* If last CMD client --> Un-register EVT_CHAN from ST */
		if(hhciif->hciif_cmdClientsCounter == 0) 
		{
			HCIIFDRV_VER(" last CMD Client --> will perform ST un-registration from EVENT Channel");
		}
		else
		{
			/* st_unregister will be done along with all other channels.
			    If this isn't the last CMD client, clear the EVENT BIT, 
			    so the st_unregister won't be called for this channel. */
			clear_bit(HCIIF_CHAN_EVT, &chan_mask);
		}
	}

	/* Now handle all other RAW CHANNELs */
	for(i = 0; i < ST_MAX_CHANNELS; i++)
	{
		if(i == HCIIF_CHAN_EVT)
			continue;
			
		if(test_bit(i, &chan_mask))
			hhciif->hciif_RawClients[i] = NULL;
	}

	/* De-allocate Client's struct */
	skb_queue_purge(&client->rx_list);
	kfree(client);
	
	spin_unlock_irqrestore(&hhciif->hciif_lock, flags); 

	/* Un-Register all channels */
	for(i = 0; i < ST_MAX_CHANNELS; i++)
	{
		if(test_bit(i, &chan_mask))
		{
			HCIIFDRV_VER("Performing ST un-registration from Channel %d", i);
					
			if (st_unregister(hciif_proto[i].channelid) < 0) 
			{
				HCIIFDRV_ERR(" st_unregister failed");
				retVal = HCIIF_ERR_FAILURE;
			}
			else
			{
				HCIIFDRV_VER(" st_unregister success");
			}
		}
	}
	
	hciif_exit_cs(0);
	
	return retVal;
}

long hciif_send_frame(struct sk_buff *skb, struct hciif_client *client)
{
#ifdef VERBOSE
	long count = 0;
#endif

	HCIIFDRV_DBG(" Inside %s", __func__);
	
	/* Packet is completed! Forward to ST. */
#ifdef VERBOSE
	HCIIFDRV_VER("start data..");
	for (count = 0; count < skb->len; count++)
		HCIIFDRV_VER(" 0x%02x ", skb->data[count]);
	HCIIFDRV_VER("\n..end data");
#endif


	/* If packet is of CMD type --> Save client in write list */
	if(skb->data[0] == HCIIF_CHAN_CMD)
	{
		client->cmd_opcode = *((uint16_t *)(&skb->data[1]));
                HCIIFDRV_VER("Client 0x%08x, command opcode 0x%04x", (unsigned int)client, client->cmd_opcode);

                /* HCI Host Number Of Completed Packets command does not generate event */
                if (client->cmd_opcode != HCIIF_CMD_HOST_NUM_OF_CMPLT_PKTS)
                {
		        spin_lock(&hhciif->hciif_lock);
		        list_add_tail(&(client->list), &(hhciif->hciif_WriteClientsList.list));
		        spin_unlock(&hhciif->hciif_lock);
	        }
	}
	
	return st_write(skb);
}

/***********************************************************************
* 						Internal Functions								     *
***********************************************************************/
/***********Functions called from ST driver**********************************/

/*  hciif_st_event_recv Function
 *  This is Called in from -- ST Core when an event is received
 *  This is a registered callback with ST core when the BT driver
 *  registers with ST.
 *  Parameters:
 *  @skb    : SKB buffer pointer which contains the incoming data.
 *  Returns:
 *          HCIIF_SUCCESS - On Success
 *          else suitable error code
 */
static long hciif_st_event_recv(struct sk_buff *skb)
{
	struct hciif_client *destClient;
	int i, j;
	bool isClientFound = false;

	HCIIFDRV_DBG(" Inside %s", __func__);

	/* SKB is NULL */
	if (NULL == skb) {
		HCIIFDRV_ERR("Input SKB is NULL");
		return HCIIF_ERR_FAILURE;
	}

	if(bt_cb(skb)->pkt_type != HCIIF_CHAN_EVT)
	{
		HCIIFDRV_ERR("Packet received is not of type EVENT");
		return HCIIF_ERR_FAILURE;
	}
	else
	{
		/* If opcode is COMPLETED/STATUS --> Send SKB to client from write list */
		if((skb->data[0] == HCIIF_OP_EVT_COMPLETED) || (skb->data[0] == HCIIF_OP_EVT_STATUS))
		{
			/* Send CMD CMPLT to the client that sent the cmd */
			spin_lock(&hhciif->hciif_lock);
			destClient = list_entry(hhciif->hciif_WriteClientsList.list.next, struct hciif_client, list);

			/* Here we are making sure that the number of completed commands equles to 1 and 
			   that the command complete was recevied for the expected command opcode.
			   If one of the above is not true --> ERROR */
			if( (skb->data[0] == HCIIF_OP_EVT_COMPLETED) && 
				((skb->data[2] != 1) || (destClient->cmd_opcode != *((uint16_t *)(&skb->data[3])))) )
			{
				spin_unlock(&hhciif->hciif_lock);
				HCIIFDRV_ERR("***** ERROR *****: The opcode of the received from client 0x%08x Cmd complete 0x%04x is not matching the expected cmd opcode 0x%04x!!!!!", (unsigned int)destClient, *((uint16_t *)&skb->data[3]), destClient->cmd_opcode);
				return HCIIF_ERR_FAILURE;

			} else if( (skb->data[0] == HCIIF_OP_EVT_STATUS) && 
				((skb->data[3] != 1) || (destClient->cmd_opcode != *((uint16_t *)(&skb->data[4])))) )
			{
				spin_unlock(&hhciif->hciif_lock);
				HCIIFDRV_ERR("***** ERROR *****: The opcode of the received from client 0x%08x Cmd complete 0x%04x is not matching the expected cmd opcode 0x%04x!!!!!", (unsigned int)destClient, *((uint16_t *)&skb->data[4]), destClient->cmd_opcode);
				return HCIIF_ERR_FAILURE;
			}

			list_del(hhciif->hciif_WriteClientsList.list.next);
			INIT_LIST_HEAD(&(destClient->list));
			spin_unlock(&hhciif->hciif_lock);
		}
		else /* If any other opcode --> Send SKB to the client that registered to this event */
		{
			spin_lock(&hhciif->hciif_lock);
			for(i = 0; i < HCIIF_CMD_CLIENTS_NUM && (hhciif->hciif_CmdClients[i] != NULL); i++)
			{
				for(j = 0; j < HCIIF_MAX_REG_EVENTS; j++)
				{
					if(hhciif->hciif_CmdClients[i]->filter.evt_type[j] == 0)
					{
						break;
					}
					else if(hhciif->hciif_CmdClients[i]->filter.evt_type[j] == skb->data[0])
					{
						isClientFound = true;
						break;
					}
				}
				if(isClientFound)
				{
					break;
				}
			}
			if( (i < HCIIF_CMD_CLIENTS_NUM) && (isClientFound) )
			{
				destClient = hhciif->hciif_CmdClients[i];
				spin_unlock(&hhciif->hciif_lock);
			}
			else if( (hhciif->hciif_CmdClients[HCIIF_CMD_CLIENTS_NUM-1] != NULL) && 
				(hhciif->hciif_CmdClients[HCIIF_CMD_CLIENTS_NUM-1]->filter.evt_type[0] == HCIIF_EVT_DEFUALT) )
			{
				destClient = hhciif->hciif_CmdClients[HCIIF_CMD_CLIENTS_NUM-1];
				spin_unlock(&hhciif->hciif_lock);
			}
			else
			{
				spin_unlock(&hhciif->hciif_lock);
				HCIIFDRV_ERR("Can't send EVT --> No client is registered to it and no default client is registered");
				return HCIIF_ERR_FAILURE;
			}
		}
	}

	HCIIFDRV_DBG(" hciif_st_event_recv: dest_client = 0x%08x", (unsigned int)destClient);
	
	/* Forward Rx */
	if(destClient->hci_core.is_hci_core)
		destClient->hci_core.hci_core_recv(skb);
	else
	{
		spin_lock(&destClient->rx_lock);
		skb_queue_tail(&destClient->rx_list, skb);
		spin_unlock(&destClient->rx_lock);
		wake_up_interruptible(&destClient->hciif_client_q);
	}

	return HCIIF_SUCCESS;
}

/*  hciif_st_ganeric_recv Function
 *  This is Called in from -- ST Core when a data is received
 *  This is a registered callback with ST core when the BT driver
 *  registers with ST.
 *  Parameters:
 *  @skb    : SKB buffer pointer which contains the incoming data.
 *  Returns:
 *          HCIIF_SUCCESS - On Success
 *          else suitable error code
 */
static long hciif_st_ganeric_recv(struct sk_buff *skb)
{
	struct hciif_client *destClient;

	HCIIFDRV_DBG(" Inside %s", __func__);

	/* SKB is NULL */
	if (NULL == skb) {
		HCIIFDRV_ERR("Input SKB is NULL");
		return HCIIF_ERR_FAILURE;
	}

	if(bt_cb(skb)->pkt_type >= ST_MAX_CHANNELS)
	{
		HCIIFDRV_ERR("pkt_type is not supported");
		return HCIIF_ERR_FAILURE;
	}

	spin_lock(&hhciif->hciif_lock);
	destClient = hhciif->hciif_RawClients[bt_cb(skb)->pkt_type];
	spin_unlock(&hhciif->hciif_lock);
			
	if(!destClient)
	{
		HCIIFDRV_ERR("Can't send packet --> No client is registered to the requested channel %d", 
						bt_cb(skb)->pkt_type);
		return HCIIF_ERR_FAILURE;
	}
	
	HCIIFDRV_DBG(" hciif_st_ganeric_recv: dest_client = 0x%08x", (unsigned int)destClient);
	
	/* Forward Rx */
	if(destClient->hci_core.is_hci_core)
		destClient->hci_core.hci_core_recv(skb);
	else
	{
		spin_lock(&destClient->rx_lock);
		skb_queue_tail(&destClient->rx_list, skb);
		spin_unlock(&destClient->rx_lock);
		wake_up_interruptible(&destClient->hciif_client_q);
	}

	return HCIIF_SUCCESS;

}

/*  hciif_st_cb Function
 *  This is Called in from -- ST Core when the state is pending
 *  during st_register.
 *  This is a registered callback with ST core when the BT driver
 *  registers with ST.
 *
 *  Parameters:
 *  @data   Status update of BT registration
 *  Returns: NULL
 */
static void hciif_st_open_cb(char data)
{
	HCIIFDRV_DBG(" Inside %s", __func__);
	hhciif->streg_cbdata = data;	/* ST registration callback  status */
	complete_all(&hhciif->hciif_reg_completed);
	return;
}

/*********** Functions used internally **********************************/
static int hciif_register_client(struct st_proto_s *hciif_proto)
{
	int ret = 0;
	unsigned long timeout = HCIIFDRV_REG_TIMEOUT;

	HCIIFDRV_DBG(" Inside %s", __func__);

	/* Initialize  hciif_reg_completed so as to wait for
	* completion on the same
	* if st_register returns with a PENDING status
	*/
	INIT_COMPLETION(hhciif->hciif_reg_completed);
	
	/* Resgister BT with ST */
	ret = st_register(hciif_proto);

	HCIIFDRV_VER(" st_register returned %d", ret);
		
	/* If Registration returned with error, return the appropriate error code */
	if (ret < 0 && ret != ST_ERR_PENDING) {
		HCIIFDRV_ERR(" st_register failed");
			
		if (ret == ST_ERR_ALREADY)
			return HCIIF_ERR_ALREADY;
		
		return HCIIF_ERR_FAILURE;
	}
		
	/* if returned status is pending, wait for the completion */
	if (ret == ST_ERR_PENDING) {
		hhciif->streg_cbdata = -EINPROGRESS;
		HCIIFDRV_VER(" BT Register waiting for completion ");
		timeout =
			wait_for_completion_timeout(&hhciif->hciif_reg_completed,
			msecs_to_jiffies(timeout));
		/* Check for timed out condition */
		if (0 == timeout) {
			HCIIFDRV_ERR("st_register timed out");

			return HCIIF_ERR_TIMEOUT;
		}
			
		if (hhciif->streg_cbdata != 0) {
			HCIIFDRV_ERR
				("BT Device Registration Failed-ST Reg CB called"
				"with invalid value %d", hhciif->streg_cbdata);
			
			return -EAGAIN;
		}
	}
		
	HCIIFDRV_DBG(" bt registration complete ");
		
	return HCIIF_SUCCESS;
}

static struct hciif_client *hciif_add_client_to_array(int channel, unsigned char evt_type[], 
												struct hciif_client *existingClient)
{
	struct hciif_client *client;
	unsigned long flags = 0;
	int i;
	
	spin_lock_irqsave(&hhciif->hciif_lock, flags);

	if(!existingClient)	
	{
		HCIIFDRV_VER(" Creating Client Handler...");
		
		/* Allocate Client Struct */
		client = kzalloc(sizeof(struct hciif_client), GFP_KERNEL);
		if (!(client)) {
			spin_unlock_irqrestore(&hhciif->hciif_lock, flags);
			HCIIFDRV_ERR("Can't allocate client structure");
			return NULL;
		}
		 
		/* Initialize Client Struct */
		INIT_LIST_HEAD(&(client->list));
		skb_queue_head_init(&client->rx_list);
		init_waitqueue_head(&client->hciif_client_q);
		spin_lock_init(&client->rx_lock);

	}
	else
		client = existingClient;

	if(channel == HCIIF_CHAN_EVT)
	{
		if(evt_type[0] == HCIIF_EVT_DEFUALT)
		{
			if(hhciif->hciif_CmdClients[HCIIF_CMD_CLIENTS_NUM -1] != NULL)
			{
				HCIIFDRV_ERR("There is already a client registered to EVT_DEFUALT");
				spin_unlock_irqrestore(&hhciif->hciif_lock, flags);
				kfree(client);
				return NULL;
			}
			else
			{
				hhciif->hciif_CmdClients[HCIIF_CMD_CLIENTS_NUM -1] = client;
			}
		}
		else
		{
			/* Find free spot in Cmd Clients array and save client struct */
			for(i = 0; i < HCIIF_CMD_CLIENTS_NUM; i++)
			{
				if(hhciif->hciif_CmdClients[i] == NULL)
					break;
			}
/*[Soldel] - BT/FM Simultaneous Turn On - BEGIN*/			
/*			hhciif->hciif_CmdClients[i] = client; */
			if( i == HCIIF_CMD_CLIENTS_NUM)
			{
				HCIIFDRV_ERR("Cannot add client as Maximum hciif_CmdClients have reached ");
				spin_unlock_irqrestore(&hhciif->hciif_lock, flags);
				return NULL;
			}
			else
			{
				hhciif->hciif_CmdClients[i] = client;	
			}
/*[Soldel] - BT/FM Simultaneous Turn On - END*/
		}

		hhciif->hciif_cmdClientsCounter++;
	}
	else
	{
		hhciif->hciif_RawClients[channel] = client;
	}

	HCIIFDRV_VER(" Client Handler 0x%08x was created & added successfuly", (unsigned int)client);
				
	spin_unlock_irqrestore(&hhciif->hciif_lock, flags);

	return client;
}

/*  hciif_enter_cs Function
 *  Critical Section service for protecting DEVUP & DEVDWON operations.
 *
 *  Parameters: None
 *
 *  Returns: Status
 */
static int hciif_enter_cs(void)
{
	unsigned long 			timeout = HCIIFDRV_REG_TIMEOUT;	

	HCIIFDRV_DBG(" Inside %s", __func__);

	/* HCIIF_ST_INPROGRESS bit is for synchronizing Open & Close 
	     operations between the different clients */
	while(test_and_set_bit(HCIIF_ST_INPROGRESS, &hhciif->state))
	{
		HCIIFDRV_DBG(" Some other operation is in progress, waiting...");
		timeout = wait_for_completion_timeout(&hhciif->hciif_oper_completed,
										msecs_to_jiffies(timeout));

		/* Check for timed out condition */
		if (0 == timeout) {
			HCIIFDRV_ERR("hciif_oper_completed timed out");
			return HCIIF_ERR_TIMEOUT;
		}
			
		if (hhciif->stoper_cbdata != 0) {
			HCIIFDRV_ERR("Error during the previous operation");
			return -EAGAIN;
		}
	}
	return HCIIF_SUCCESS;
}

/*  hciif_exit_cs Function
 *  Critical Section service for protecting DEVUP & DEVDWON operations.
 *
 *  Parameters: status - Status of operation
 *
 *  Returns: None
 */
static void hciif_exit_cs(char status)
{
	clear_bit(HCIIF_ST_INPROGRESS, &hhciif->state);
	hhciif->stoper_cbdata = status;
	INIT_COMPLETION(hhciif->hciif_oper_completed);
	complete(&hhciif->hciif_oper_completed);	
}

MODULE_LICENSE("GPL");
