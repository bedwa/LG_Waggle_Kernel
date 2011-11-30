/*
 *  Texas Instrument's Bluetooth Driver For Shared Transport.
 *
 *  Bluetooth Driver acts as interface between HCI CORE and
 *  TI Shared Transport Layer.
 *
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

#include <net/bluetooth/bluetooth.h>
#include <net/bluetooth/hci_core.h>

#include "st.h"
#include "bt_drv.h"

/* Define this macro to get debug msg */
#undef DEBUG

#ifdef DEBUG
#define BT_DRV_DBG(fmt, arg...)  printk(KERN_INFO "(btdrv):"fmt"\n" , ## arg)
#define BTDRV_API_START()        printk(KERN_INFO "(btdrv): %s Start\n", \
	__func__)
#define BTDRV_API_EXIT(errno)    printk(KERN_INFO "(btdrv): %s Exit(%d)\n", \
	__func__, errno)
#else
#define BT_DRV_DBG(fmt, arg...)
#define BTDRV_API_START()
#define BTDRV_API_EXIT(errno)
#endif

#define BT_DRV_ERR(fmt, arg...)  printk(KERN_ERR "(btdrv):"fmt"\n" , ## arg)

static int reset;
static struct hci_st *hst;

/* Increments HCI counters based on pocket ID (cmd,acl,sco) */
static inline void hci_st_tx_complete(struct hci_st *hst, int pkt_type)
{
	struct hci_dev *hdev;

	BTDRV_API_START();

	hdev = hst->hdev;

	/* Update HCI stat counters */
	switch (pkt_type) {
	case HCI_COMMAND_PKT:
		hdev->stat.cmd_tx++;
		break;

	case HCI_ACLDATA_PKT:
		hdev->stat.acl_tx++;
		break;

	case HCI_SCODATA_PKT:
		hdev->stat.cmd_tx++;
		break;
	}

	BTDRV_API_EXIT(0);
}

/* ------- Interfaces to Shared Transport ------ */

/* Called by Shared Transport layer when receive data is
 * available */
static long hci_st_receive(struct sk_buff *skb)
{
	int err;
	int len;

	BTDRV_API_START();

	err = 0;
	len = 0;

	if (skb == NULL) {
		BT_DRV_ERR("Invalid SKB received from ST");
		BTDRV_API_EXIT(-EFAULT);
		return -EFAULT;
	}
	if (!hst) {
		kfree_skb(skb);
		BT_DRV_ERR("Invalid hci_st memory,freeing SKB");
		BTDRV_API_EXIT(-EFAULT);
		return -EFAULT;
	}
	if (!test_bit(BT_DRV_RUNNING, &hst->flags)) {
		kfree_skb(skb);
		BT_DRV_ERR("Device is not running,freeing SKB");
		BTDRV_API_EXIT(-EINVAL);
		return -EINVAL;
	}

	len = skb->len;
	skb->dev = (struct net_device *)hst->hdev;

	/* Forward skb to HCI CORE layer */
	err = hci_recv_frame(skb);
	if (err) {
// TI_SOLDEL 2011/03/14 cmlee, prevent double free [START]
//		kfree_skb(skb);
// TI_SOLDEL 2011/03/14 cmlee, prevent double free [END]
		BT_DRV_ERR("Unable to push skb to HCI CORE(%d),freeing SKB",
			   err);
		BTDRV_API_EXIT(err);
		return err;
	}
	hst->hdev->stat.byte_rx += len;

	BTDRV_API_EXIT(0);
	return 0;
}

/* ------- Interfaces to HCI layer ------ */

/* Called from HCI core to initialize the device */
static int hci_st_open(struct hci_dev *hdev)
{
	int err;
	struct hciif_filter_t filter;
	struct hciif_hciCore_t hciCore;

	BTDRV_API_START();

	err = 0;

	BT_DRV_DBG("%s %p", hdev->name, hdev);

	/* Already registered with ST ? */
	if (test_bit(BT_ST_REGISTERED, &hst->flags)) {
		BT_DRV_ERR("Registered with ST already,open called again?");
		BTDRV_API_EXIT(0);
		return 0;
	}

	set_bit(HCIIF_CHAN_ACL, &filter.chan_mask);
	set_bit(HCIIF_CHAN_SCO, &filter.chan_mask);
	set_bit(HCIIF_CHAN_EVT, &filter.chan_mask);
	filter.evt_type[0] = HCIIF_EVT_DEFUALT;
	filter.evt_type[1] = 0;

	hciCore.is_hci_core = true;
	hciCore.hci_core_recv = hci_st_receive;

	err = hciif_dev_up(NULL, &filter, &hciCore, &hst->client);
	if(err)
	{
		BT_DRV_ERR("hciif_devUp failed, err = %d", err);
		BTDRV_API_EXIT(-EAGAIN);
		return -EAGAIN;

	}
	
	/* Registration with ST layer is completed successfully,
	 * now chip is ready to accept commands from HCI CORE.
	 * Mark HCI Device flag as RUNNING
	 */
	set_bit(HCI_RUNNING, &hdev->flags);

	/* Registration with ST successful */
	set_bit(BT_ST_REGISTERED, &hst->flags);

	BTDRV_API_EXIT(err);
	return err;
}

/* Close device */
static int hci_st_close(struct hci_dev *hdev)
{
	int err;

	BTDRV_API_START();

	err = 0;

	if (test_and_clear_bit(BT_ST_REGISTERED, &hst->flags)) {
		ST_LOG("[%s] befor calling hciif_dev_down() ###############\n", __func__);
		err = hciif_dev_down(hst->client);
		if(err)
		{
			BT_DRV_ERR("hciif_close failed, err = %d", err);
			BTDRV_API_EXIT(-EBUSY);
			return -EBUSY;
		}
	}
	
	/* ST layer would have moved chip to inactive state.
	 * So,clear HCI device RUNNING flag.
	 */
	if (!test_and_clear_bit(HCI_RUNNING, &hdev->flags)) {
		BTDRV_API_EXIT(0);
		return 0;
	}

	BTDRV_API_EXIT(err);
	return err;
}

/* Called from HCI CORE , Sends frames to Shared Transport */
static int hci_st_send_frame(struct sk_buff *skb)
{
	struct hci_dev *hdev;
	struct hci_st *hst;
	long len;

	BTDRV_API_START();

	if (skb == NULL) {
		BT_DRV_ERR("Invalid skb received from HCI CORE");
		BTDRV_API_EXIT(-ENOMEM);
		return -ENOMEM;
	}
	hdev = (struct hci_dev *)skb->dev;
	if (!hdev) {
		BT_DRV_ERR("SKB received for invalid HCI Device (hdev=NULL)");
		BTDRV_API_EXIT(-ENODEV);
		return -ENODEV;
	}
	if (!test_bit(HCI_RUNNING, &hdev->flags)) {
		BT_DRV_ERR("Device is not running");
		BTDRV_API_EXIT(-EBUSY);
		return -EBUSY;
	}

	hst = (struct hci_st *)hdev->driver_data;

	/* Prepend skb with frame type */
	memcpy(skb_push(skb, 1), &bt_cb(skb)->pkt_type, 1);

	BT_DRV_DBG(" %s: type %d len %d", hdev->name, bt_cb(skb)->pkt_type,
		   skb->len);

	len = hciif_send_frame(skb, hst->client);
	if (len < 0) {
		/* Something went wrong in st write , free skb memory */
		kfree_skb(skb);
		BT_DRV_ERR(" hciif_sendFrame failed (%ld)", len);
		BTDRV_API_EXIT(-EAGAIN);
		return -EAGAIN;
	}

	/* ST accepted our skb. So, Go ahead and do rest */
	hdev->stat.byte_tx += len;
	hci_st_tx_complete(hst, bt_cb(skb)->pkt_type);

	BTDRV_API_EXIT(0);
	return 0;
}

static void hci_st_destruct(struct hci_dev *hdev)
{
	BTDRV_API_START();

	if (!hdev) {
		BT_DRV_ERR("Destruct called with invalid HCI Device"
			   "(hdev=NULL)");
		BTDRV_API_EXIT(0);
		return;
	}

	BT_DRV_DBG("%s", hdev->name);

	/* free hci_st memory */
	if (hdev->driver_data != NULL)
		kfree(hdev->driver_data);

	BTDRV_API_EXIT(0);
	return;
}

/* Creates new HCI device */
static int hci_st_register_dev(struct hci_st *hst)
{
	struct hci_dev *hdev;

	BTDRV_API_START();

	/* Initialize and register HCI device */
	hdev = hci_alloc_dev();
	if (!hdev) {
		BT_DRV_ERR("Can't allocate HCI device");
		BTDRV_API_EXIT(-ENOMEM);
		return -ENOMEM;
	}
	BT_DRV_DBG(" HCI device allocated. hdev= %p", hdev);

	hst->hdev = hdev;
	hdev->bus = HCI_UART;
	hdev->driver_data = hst;
	hdev->open = hci_st_open;
	hdev->close = hci_st_close;
	hdev->flush = NULL;
	hdev->send = hci_st_send_frame;
	hdev->destruct = hci_st_destruct;
	hdev->owner = THIS_MODULE;

	if (reset)
		set_bit(HCI_QUIRK_NO_RESET, &hdev->quirks);

	if (hci_register_dev(hdev) < 0) {
		BT_DRV_ERR("Can't register HCI device");
		hci_free_dev(hdev);
		BTDRV_API_EXIT(-ENODEV);
		return -ENODEV;
	}

	BT_DRV_DBG(" HCI device registered. hdev= %p", hdev);
	BTDRV_API_EXIT(0);
	return 0;
}

/* ------- Module Init interface ------ */

static int __init bt_drv_init(void)
{
	int err;

	BTDRV_API_START();

	err = 0;

	BT_DRV_DBG(" Bluetooth Driver Version %s", VERSION);

	/* Allocate local resource memory */
	hst = kzalloc(sizeof(struct hci_st), GFP_KERNEL);
	if (!hst) {
		BT_DRV_ERR("Can't allocate control structure");
		BTDRV_API_EXIT(-ENFILE);
		return -ENFILE;
	}

	/* Expose "hciX" device to user space */
	err = hci_st_register_dev(hst);
	if (err) {
		/* Release local resource memory */
		kfree(hst);

		BT_DRV_ERR("Unable to expose hci0 device(%d)", err);
		BTDRV_API_EXIT(err);
		return err;
	}

	err = hciif_init();
	if (err) {
		/* Remove HCI device (hciX) created in module init. */
		hci_unregister_dev(hst->hdev);

		/* Free HCI device memory */
		hci_free_dev(hst->hdev);

		/* Release local resource memory */
		kfree(hst);

		BT_DRV_ERR("hciif_init Failed");
		BTDRV_API_EXIT(err);
		return err;
	}
	
	set_bit(BT_DRV_RUNNING, &hst->flags);

	BTDRV_API_EXIT(err);
	
	return err;
}

/* ------- Module Exit interface ------ */

static void __exit bt_drv_exit(void)
{
	BTDRV_API_START();

	hciif_exit();
	/* Deallocate local resource's memory  */
	if (hst) {
		struct hci_dev *hdev = hst->hdev;

		if (hdev == NULL) {
			BT_DRV_ERR("Invalid hdev memory");
			kfree(hst);
		} else {
			hci_st_close(hdev);
			if (test_and_clear_bit(BT_DRV_RUNNING, &hst->flags)) {
				/* Remove HCI device (hciX) created
				 * in module init.
				 */
				hci_unregister_dev(hdev);

				/* Free HCI device memory */
				hci_free_dev(hdev);
			}
		}
	}
	BTDRV_API_EXIT(0);
}

module_init(bt_drv_init);
module_exit(bt_drv_exit);

/* ------ Module Info ------ */

module_param(reset, bool, 0644);
MODULE_PARM_DESC(reset, "Send HCI reset command on initialization");
MODULE_AUTHOR("Raja Mani <raja_mani@ti.com>");
MODULE_DESCRIPTION("Bluetooth Driver for TI Shared Transport" VERSION);
MODULE_VERSION(VERSION);
MODULE_LICENSE("GPL");
