/*
 *  Shared Transport Line discipline driver Core
 *	This hooks up ST KIM driver and ST LL driver
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

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/tty.h>

/* understand BT, FM and GPS for now */
#include <net/bluetooth/bluetooth.h>
#include <net/bluetooth/hci_core.h>
#include <net/bluetooth/hci.h>
#include <linux/rtc.h> 
#include "fm.h"
/*
 * packet formats for fm and gps
 * #include "gps.h"
 */
#include "st_core.h"
#include "st_kim.h"
#include "st_ll.h"
#include "st.h"

//#define VERBOSE

#undef VERBOSE
#undef DEBUG

/* all debug macros go in here */
#define ST_DRV_ERR(fmt, arg...)  printk(KERN_ERR "(stc):"fmt"\n" , ## arg)
#if defined(DEBUG)		/* limited debug messages */
#define ST_DRV_DBG(fmt, arg...)  printk(KERN_INFO "(stc):"fmt"\n" , ## arg)
#define ST_DRV_VER(fmt, arg...)
#elif defined(VERBOSE)		/* very verbose */
#define ST_DRV_DBG(fmt, arg...)  printk(KERN_INFO "(stc):"fmt"\n" , ## arg)
#define ST_DRV_VER(fmt, arg...)  printk(KERN_INFO "(stc):"fmt"\n" , ## arg)
#else /* error msgs only */
#define ST_DRV_DBG(fmt, arg...)
#define ST_DRV_VER(fmt, arg...)
#endif


// LGE_LGE_WCDMA_FEATURE_MERGE  START
#define TMP_ST_RECV_FIX
// LGE_LGE_WCDMA_FEATURE_MERGE  END
#if 1//TI HSYoon 20110616
static unsigned long irq_save_flags;
#endif

#if 1//TI HSYoon 20110613
struct st_data_s *st_gdata;//required for st_kim.c
EXPORT_SYMBOL_GPL(st_gdata);
#else
/*
 * local data instances
 */
static struct st_data_s *st_gdata;
#endif
/* function pointer pointing to either,
 * st_kim_recv during registration to receive fw download responses
 * st_int_recv after registration to receive proto stack responses
 */
void (*st_recv) (const unsigned char *data, long count);


/*[Soldel] - Faster Initscript download time - BEGIN*/
/*
 * called from KIM during firmware download.
 *
 * This is a wrapper function to tty->ops->write_room.
 * It returns number of free space available in
 * uart tx buffer.
 */
int st_get_uart_wr_room()
{
	struct tty_struct *tty;
	if (unlikely(st_gdata == NULL || st_gdata->tty == NULL)) {
#if 1//hsyoon 20110616
		pr_err("[%s] tty unavailable to perform write %d %d", __func__, st_gdata, st_gdata->tty );    
#else
		pr_err("tty unavailable to perform write");
#endif
		return -1;
	}
	tty = st_gdata->tty;
	return tty->ops->write_room(tty);
}
/*[Soldel] - Faster Initscript download time - END*/


/********************************************************************/
/* internal misc functions */
static bool is_channels_table_empty(void)
{
	return (st_gdata->channels_cntr==0);
}

static long add_channel_to_table(struct st_proto_s *new_proto)
{
	if (new_proto == NULL)
	{
		ST_DRV_ERR("add_channel_to_table: invalid pointer");
		return ST_ERR_FAILURE;
	}
	if (new_proto->channelid >= ST_MAX_CHANNELS) {
		ST_DRV_ERR("add_channel_to_table: channel id (%d) is not valid", new_proto->channelid);
		return ST_ERR_FAILURE;
	}
	/* check for already registered once more,
	 * since the above check is old
	 */
	if (st_gdata->channels[new_proto->channelid] != NULL) {
		ST_DRV_ERR("add_channel_to_table: proto %d already registered ", new_proto->channelid);
		return ST_ERR_ALREADY;
	}

	st_gdata->channels[new_proto->channelid] = new_proto;
	st_gdata->channels_cntr ++;
	return ST_SUCCESS;
}

static long remove_channel_from_table(channel_t channelid)
{
	if (channelid >= ST_MAX_CHANNELS) {
		ST_DRV_ERR("remove_channel_from_table: channel id (%d) is not valid", channelid);
		return ST_ERR_FAILURE;
	}
	if (st_gdata->channels_cntr == 0 || st_gdata->channels[channelid] == NULL) {
		ST_DRV_ERR("remove_channel_from_table: protocol %d not registered", channelid);
		return ST_ERR_NOPROTO;
	}

	st_gdata->channels[channelid] = NULL;
	st_gdata->channels_cntr --;
	return ST_SUCCESS;
}

/* can be called in from
 * -- KIM (during fw download)
 * -- ST Core (during st_write)
 *
 *  This is the internal write function - a wrapper
 *  to tty->ops->write
 */
int st_int_write(const unsigned char *data, int count)
{
#if defined(SHOW_ST_LOG) //hsyoon 20110605 -remove log
  printk("%s( %d )\n", __func__, count);
#endif
	struct tty_struct *tty;
	if (unlikely(st_gdata == NULL || st_gdata->tty == NULL)) {
	ST_DRV_ERR("[%s] tty unavailable to perform write %d %d", __func__, st_gdata, st_gdata->tty );    
		return ST_ERR_FAILURE;
	}
	tty = st_gdata->tty;
	
#if defined(SHOW_ST_LOG)
	printk("%s(%d) : %x %x %x %x %x %x %x\n", __func__, count, data[0],data[1],data[2],data[3],data[4],data[5],data[6]);
#endif

	return tty->ops->write(tty, data, count);
}

/*
 * push the skb received to relevant
 * protocol stacks
 */
static void st_send_frame(channel_t channelid, struct sk_buff *skb)
{
	ST_DRV_DBG(" %s(prot:%d) ", __func__, channelid);

	if (unlikely
	    (st_gdata == NULL || skb == NULL
	     || st_gdata->channels[channelid] == NULL)) {
		ST_DRV_ERR("protocol %d not registered, no data to send?",
			   channelid);
		kfree_skb(skb);
		return;
	}
	/* this cannot fail
	 * this shouldn't take long
	 * - should be just skb_queue_tail for the
	 *   protocol stack driver
	 */
	if (likely(st_gdata->channels[channelid]->recv != NULL)) {
		if (unlikely(st_gdata->channels[channelid]->recv(skb)
			     != ST_SUCCESS)) {
			ST_DRV_ERR(" proto stack %d's ->recv failed", channelid);
			kfree_skb(skb);
			return;
		}
	} else {
		ST_DRV_ERR(" proto stack %d's ->recv null", channelid);
		kfree_skb(skb);
	}
	ST_DRV_DBG(" done %s", __func__);
	return;
}

/*
 * to call registration complete callbacks
 * of all protocol stack drivers
 */
static void st_reg_complete(char err)
{
	unsigned char i = 0;
	ST_DRV_DBG(" %s ", __func__);
	for (i = 0; i < ST_MAX_CHANNELS; i++) {
		if (likely(st_gdata != NULL && st_gdata->channels[i] != NULL &&
			   st_gdata->channels[i]->reg_complete_cb != NULL))
			st_gdata->channels[i]->reg_complete_cb(err);
	}
}

static inline int st_check_data_len(int channelid, int len)
{
	register int room = skb_tailroom(st_gdata->rx_skb);

	ST_DRV_DBG("len %d room %d", len, room);

	if (!len) {
		/* Received packet has only packet header and
		 * has zero length payload. So, ask ST CORE to
		 * forward the packet to protocol driver (BT/FM/GPS)
		 */
		st_send_frame(channelid, st_gdata->rx_skb);

	} else if (len > room) {
		/* Received packet's payload length is larger.
		 * We can't accommodate it in created skb.
		 */
		ST_DRV_ERR("Data length is too large len %d room %d", len,
			   room);
		kfree_skb(st_gdata->rx_skb);
	} else {
		/* Packet header has non-zero payload length and
		 * we have enough space in created skb. Lets read
		 * payload data */
		st_gdata->rx_state = ST_W4_DATA;
		st_gdata->rx_count = len;
		return len;
	}

	/* Change ST state to continue to process next
	 * packet */
	st_gdata->rx_state = ST_W4_PACKET_TYPE;
	st_gdata->rx_skb = NULL;
	st_gdata->rx_count = 0;

	return 0;
}

/* internal function for action when wake-up ack
 * received
 */
static inline void st_wakeup_ack(unsigned char cmd)
{
	register struct sk_buff *waiting_skb;
	unsigned long flags = 0;
	/* de-Q from waitQ and Q in txQ now that the
	 * chip is awake
	 */
	while ((waiting_skb = skb_dequeue(&st_gdata->tx_waitq)))
		skb_queue_tail(&st_gdata->txq, waiting_skb);

	/* state forwarded to ST LL */
	st_ll_sleep_state((unsigned long)cmd);

	/* wake up to send the recently copied skbs from waitQ */
	st_tx_wakeup(st_gdata);
}

/* Decodes received RAW data and forwards to corresponding
 * client drivers (Bluetooth,FM,GPS..etc).
 *
 */
void st_int_recv(const unsigned char *data, long count)
{
	register char *ptr;
	register int len = 0;
	static channel_t channelid = ST_MAX_CHANNELS;

	ST_DRV_DBG("count %ld rx_state %ld"
		   "rx_count %ld", count, st_gdata->rx_state,
		   st_gdata->rx_count);

	ptr = (char *)data;
	/* tty_receive sent null ? */
	if (unlikely(ptr == NULL)) {
		ST_DRV_ERR(" received null from TTY ");
		return;
	} 

	/* Decode received bytes here */
	while (count) {
		if (st_gdata->rx_count) {
			len = min_t(unsigned int, st_gdata->rx_count, count);
			memcpy(skb_put(st_gdata->rx_skb, len), ptr, len);
			st_gdata->rx_count -= len;
			count -= len;
			ptr += len;

			if (st_gdata->rx_count)
				continue;

			/* Check ST RX state machine , where are we? */
			switch (st_gdata->rx_state) {

				case ST_W4_DATA:
				{
					//ST_LOG_X("RX Pkt (data):Complete pkt received\n");//hsyoon 20110612

				/* Ask ST CORE to forward
				 * the packet to protocol driver */
				st_send_frame(st_gdata->channelid, st_gdata->rx_skb);
				st_gdata->rx_state = ST_W4_PACKET_TYPE;
				st_gdata->rx_skb = NULL;
					channelid = ST_MAX_CHANNELS;	/* is this required ? */
				continue;
				}
				/* Waiting for hci header */
				case ST_W4_HEADER:
				{
					unsigned short payload_len;
					unsigned char len_size = st_gdata->channels[st_gdata->channelid]->length_size;
					unsigned char len_offset = st_gdata->channels[st_gdata->channelid]->length_offset;
					unsigned char *plen = &st_gdata->rx_skb->data[len_offset];

					if(len_size == 1)
						payload_len = *(unsigned char *)plen;
					else if (len_size == 2)
						payload_len = __le16_to_cpu(*(unsigned short *)plen);
					else
					{
						ST_DRV_ERR("invalid size of length field  (%d)", len_size);
						st_gdata->rx_state = ST_W4_PACKET_TYPE;
						st_gdata->rx_count = 0;
						return;
					}
					ST_DRV_DBG("RX Pkt (header): payload_len = %d", payload_len);
					st_check_data_len(channelid, payload_len);
				continue;
				}

			}	/* end of switch rx_state */
		}

		/* end of if rx_count */
		/* Check first byte of packet and identify module
		 * owner (BT/FM/GPS) */
		channelid = *ptr;
		switch (channelid) {
		case LL_SLEEP_IND:

			printk("############HCILL : LL_SLEEP_IND\n");//hsyoon 20110612
#if 1//hsyoon 20110612		
			st_ll_sleep_state(channelid);
			break;
#endif
		case LL_SLEEP_ACK:				
			printk("############HCILL : LL_SLEEP_ACK\n");//hsyoon 20110612
#if 1    /* TIK_BT OPP fail issue */			
			st_ll_sleep_state(channelid);
			break;
#endif
		case LL_WAKE_UP_IND:
			printk("############HCILL : LL_WAKE_UP_IND\n");//hsyoon 20110612
			{
			/* this takes appropriate action based on
			 * sleep state received --
			 */
#if 1    /* TIK_BT OPP fail issue */
				st_wakeup_ack(LL_WAKE_UP_IND);
#else
				st_ll_sleep_state(channelid);
#endif		
				break;
			}
		case LL_WAKE_UP_ACK:
			printk("############HCILL : LL_WAKE_UP_ACK\n");//hsyoon 20110612
			{
			/* wake up ack received */
				st_wakeup_ack(channelid);		
			break;
			}
			default: /* non-HCILL channels */
			{	
#if 1    /* Passat issue hsyoon 20110624 
            We make HCILL awake when data packet is received in non-awake state
         */			
				if( st_ll_getstate() != ST_LL_AWAKE)
                                {  
				     printk("########################### Rcvd Channel ID: %d\n", channelid);
				     st_wakeup_ack(LL_WAKE_UP_IND);
			}
#endif	
				if(unlikely(channelid >= ST_MAX_CHANNELS || st_gdata->channels[channelid] == NULL))
				{
					ST_DRV_ERR("Unknown packet type. ch_id =%d, %2.2x", channelid, (__u8) *ptr);
			    break;
				}

				/* bluez skb allocation reserves headroom which can be used by other client for pushing the channel num */
				st_gdata->rx_skb = bt_skb_alloc(st_gdata->channels[channelid]->max_frame_size, GFP_ATOMIC);
			if (!st_gdata->rx_skb) {
					ST_DRV_ERR("RX Pkt (channelid=%d): Can't allocate mem for new packet", channelid);
				st_gdata->rx_state = ST_W4_PACKET_TYPE;
				st_gdata->rx_count = 0;
				return;
			}
				st_gdata->channelid = channelid; /* store current prorocol id for header resoulution state */
				st_gdata->rx_state = ST_W4_HEADER;
				st_gdata->rx_count = st_gdata->channels[channelid]->header_size;
				bt_cb(st_gdata->rx_skb)->pkt_type = channelid;
				ST_DRV_DBG("RX Pkt (channelid=%d): header_length=%ld", channelid, st_gdata->rx_count);
		}			
		};
		ptr++;
		count--;		
	}

	ST_DRV_DBG("done %s", __func__);
	return;
}

/* internal de-Q function
 * -- return previous in-completely written skb
 *  or return the skb in the txQ
 */
struct sk_buff *st_int_dequeue(struct st_data_s *st_data)
{
	struct sk_buff *returning_skb;

	ST_DRV_VER("%s", __func__);
	/* if the previous skb wasn't written completely
	 */
	if (st_gdata->tx_skb != NULL) {
		returning_skb = st_gdata->tx_skb;
		st_gdata->tx_skb = NULL;
		return returning_skb;
	}

	/* de-Q from the txQ always if previous write is complete */
	return skb_dequeue(&st_gdata->txq);
}

/* internal Q-ing function
 * will either Q the skb to txq or the tx_waitq
 * depending on the ST LL state
 *
 * lock the whole func - since ll_getstate and Q-ing should happen
 * in one-shot
 */
void st_int_enqueue(struct sk_buff *skb)
{
	unsigned long flags = 0;

	ST_DRV_VER("%s", __func__);
	/* this function can be invoked in more then one context.
	 * so have a lock */

	switch (st_ll_getstate()) {
	case ST_LL_AWAKE:
		ST_DRV_DBG("ST LL is AWAKE, sending normally");
		skb_queue_tail(&st_gdata->txq, skb);
		break;
	case ST_LL_ASLEEP_TO_AWAKE:
		skb_queue_tail(&st_gdata->tx_waitq, skb);
		break;
	case ST_LL_AWAKE_TO_ASLEEP:	/* host cannot be in this state */
		ST_DRV_ERR("ST LL is illegal state(%ld),"
			   "purging received skb.", st_ll_getstate());
		kfree_skb(skb);
		break;

	case ST_LL_ASLEEP:
		/* call a function of ST LL to put data
		 * in tx_waitQ and wake_ind in txQ
		 */
		skb_queue_tail(&st_gdata->tx_waitq, skb);
		st_ll_wakeup();
		break;
	default:
		ST_DRV_ERR("ST LL is illegal state(%ld),"
			   "purging received skb.", st_ll_getstate());
		kfree_skb(skb);
		break;
	}

	ST_DRV_VER("done %s", __func__);
	return;
}

/*
 * internal wakeup function
 * called from either
 * - TTY layer when write's finished
 * - st_write (in context of the protocol stack)
 */
void st_tx_wakeup(struct st_data_s *st_data)
{
	struct sk_buff *skb;
	unsigned long flags;	/* for irq save flags */
	ST_DRV_VER("%s", __func__);
	/* check for sending & set flag sending here */
	if (test_and_set_bit(ST_TX_SENDING, &st_data->tx_state)) {
		ST_DRV_DBG("ST already sending");
		/* keep sending */
		set_bit(ST_TX_WAKEUP, &st_data->tx_state);
		return;
		/* TX_WAKEUP will be checked in another
		 * context
		 */
	}
	do {			/* come back if st_tx_wakeup is set */
		/* woke-up to write */
		clear_bit(ST_TX_WAKEUP, &st_data->tx_state);
		while ((skb = st_int_dequeue(st_data))) {
			int len;
			/* enable wake-up from TTY */
			set_bit(TTY_DO_WRITE_WAKEUP, &st_data->tty->flags);
			len = st_int_write(skb->data, skb->len);
			skb_pull(skb, len);
			/* if skb->len = len as expected, skb->len=0 */
			if (skb->len) {
				/* would be the next skb to be sent */
				st_data->tx_skb = skb;
				break;
			}
			kfree_skb(skb);
		}
		/* if wake-up is set in another context- restart sending */
	} while (test_bit(ST_TX_WAKEUP, &st_data->tx_state));

	/* clear flag sending */
	clear_bit(ST_TX_SENDING, &st_data->tx_state);
}

/********************************************************************/
/* functions called from ST KIM
*/
void kim_st_list_channels(char *buf)
{
	unsigned long flags = 0;
#ifdef DEBUG
	unsigned char i = ST_MAX_CHANNELS;
#endif
	spin_lock_irqsave(&st_gdata->lock, irq_save_flags);//TI HSYoon 20110616
#ifdef DEBUG			/* more detailed log */
	for (i = 0; i < ST_MAX_CHANNELS; i++) {
		if (i == 0) {
			sprintf(buf, "%d is %s", i,
				st_gdata->channels[i] !=
				NULL ? "Registered" : "Unregistered");
		} else {
			sprintf(buf, "%s\n%d is %s", buf, i,
				st_gdata->channels[i] !=
				NULL ? "Registered" : "Unregistered");
		}
	}
	sprintf(buf, "%s\n", buf);
#else /* limited info */
	sprintf(buf, "BT=%c\nFM=%c\nGPS=%c\n",
		st_gdata->channels[1] != NULL ? 'R' : 'U',
		st_gdata->channels[8] != NULL ? 'R' : 'U',
		st_gdata->channels[9] != NULL ? 'R' : 'U');
#endif
	spin_unlock_irqrestore(&st_gdata->lock, irq_save_flags);//TI HSYoon 20110616
}

/********************************************************************/
/*
 * functions called from protocol stack drivers
 * to be EXPORT-ed
 */
long st_register(struct st_proto_s *new_proto)
{
	long err = ST_SUCCESS;
	unsigned long flags = 0;

  ST_LOG("***************************************\n");
  
	ST_DRV_ERR("%s(%d) ", __func__, new_proto->channelid);
	if (st_gdata == NULL || new_proto == NULL || new_proto->recv == NULL
	    || new_proto->reg_complete_cb == NULL) {
		ST_DRV_ERR("gdata/new_proto/recv or reg_complete_cb not ready");
		return ST_ERR_FAILURE;
	}

	if (new_proto->channelid >= ST_MAX_CHANNELS) {
		ST_DRV_ERR("channelid type %d not supported", new_proto->channelid);
		return ST_ERR_NOPROTO;
	}

	if (st_gdata->channels[new_proto->channelid] != NULL) {
		ST_DRV_ERR("protocol %d already registered", new_proto->channelid);
		return ST_ERR_ALREADY;
	}

	/* can be from process context only */
	spin_lock_irqsave(&st_gdata->lock, irq_save_flags);//TI HSYoon 20110616

	if (test_bit(ST_REG_IN_PROGRESS, &st_gdata->st_state)) {
		ST_DRV_ERR(" ST_REG_IN_PROGRESS:%d ", new_proto->channelid);
		/* fw download in progress */

		if(new_proto->gpio_id != ST_GPIO_MAX)
			st_kim_chip_toggle(new_proto->gpio_id, KIM_GPIO_ACTIVE);

		err = add_channel_to_table(new_proto);
		if(err)
		{
			spin_unlock_irqrestore(&st_gdata->lock, irq_save_flags);//TI HSYoon 20110616
			return err;
		}
		set_bit(ST_REG_PENDING, &st_gdata->st_state);
			spin_unlock_irqrestore(&st_gdata->lock, irq_save_flags);//TI HSYoon 20110616
		return ST_ERR_PENDING;
		} else if (is_channels_table_empty()) {

		ST_DRV_ERR(" protocol list empty :%d ", new_proto->channelid);
		set_bit(ST_REG_IN_PROGRESS, &st_gdata->st_state);
		st_recv = st_kim_recv;

		/* release lock previously held - re-locked below */

		/* enable the ST LL - to set default chip state */
		st_ll_enable();
		/* this may take a while to complete
		 * since it involves BT fw download
		 */
#if 1//hsyoon 20110616
		spin_unlock_irqrestore(&st_gdata->lock, irq_save_flags);//TI HSYoon 20110616
#endif

		err = st_kim_start();
		
		if (err != ST_SUCCESS) {
			clear_bit(ST_REG_IN_PROGRESS, &st_gdata->st_state);
			if ((!is_channels_table_empty()) &&
			    (test_bit(ST_REG_PENDING, &st_gdata->st_state))) {
				ST_DRV_ERR(" KIM failure complete callback ");
				st_reg_complete(ST_ERR_FAILURE);
			}
			
			return ST_ERR_FAILURE;
		}
		ST_DRV_ERR(" KIM complete callback gpio_id : %d, channelid : %d",new_proto->gpio_id,new_proto->channelid );

		/* the protocol might require other gpios to be toggled
		 */
		 
		if(new_proto->gpio_id != ST_GPIO_MAX)
			st_kim_chip_toggle(new_proto->gpio_id, KIM_GPIO_ACTIVE);

#if 1//hsyoon 20110616
		spin_lock_irqsave(&st_gdata->lock, irq_save_flags);//TI HSYoon 20110616
#endif

		clear_bit(ST_REG_IN_PROGRESS, &st_gdata->st_state);
		st_recv = st_int_recv;
    
#if 1//hsyoon 20110616
		spin_unlock_irqrestore(&st_gdata->lock, irq_save_flags);//TI HSYoon 20110616
#endif

		/* this is where all pending registration
		 * are signalled to be complete by calling callback functions
		 */
		if ((!is_channels_table_empty()) &&
		    (test_bit(ST_REG_PENDING, &st_gdata->st_state))) {
			ST_DRV_ERR(" call reg complete callback ");
			st_reg_complete(ST_SUCCESS);
		}
		clear_bit(ST_REG_PENDING, &st_gdata->st_state);

#if 1//hsyoon 20110616
		spin_lock_irqsave(&st_gdata->lock, irq_save_flags);//TI HSYoon 20110616
#endif

		err = add_channel_to_table(new_proto);
			spin_unlock_irqrestore(&st_gdata->lock, irq_save_flags);//TI HSYoon 20110616

		return err;
	}
	/* if fw is already downloaded & new stack registers protocol */
	else {
		if(new_proto->gpio_id != ST_GPIO_MAX)
			st_kim_chip_toggle(new_proto->gpio_id, KIM_GPIO_ACTIVE);
		err = add_channel_to_table(new_proto);

		/* lock already held before entering else */
			spin_unlock_irqrestore(&st_gdata->lock, irq_save_flags);//TI HSYoon 20110616
		return err;
	}
	ST_DRV_ERR("done %s(%d) ", __func__, new_proto->channelid);
}
EXPORT_SYMBOL_GPL(st_register);

/* to unregister a protocol -
 * to be called from protocol stack driver
 */
long st_unregister(channel_t channelid)
{
	long err = ST_SUCCESS;
	unsigned long flags = 0;

	ST_DRV_ERR("%s: %d ", __func__, channelid);

	if (channelid >= ST_MAX_CHANNELS) {
		ST_DRV_ERR(" protocol %d not supported", channelid);
		return ST_ERR_NOPROTO;
	}

			spin_lock_irqsave(&st_gdata->lock, irq_save_flags);//TI HSYoon 20110616

	/* kim ignores BT in the below function
	 * and handles the rest, BT is toggled
	 * only in kim_start and kim_stop
	 */
	if(st_gdata->channels[channelid]->gpio_id != ST_GPIO_MAX)
	{
		st_kim_chip_toggle(st_gdata->channels[channelid]->gpio_id, KIM_GPIO_INACTIVE);
	}
	remove_channel_from_table(channelid);

			spin_unlock_irqrestore(&st_gdata->lock, irq_save_flags);//TI HSYoon 20110616


	if ((is_channels_table_empty()) &&
		(!test_bit(ST_REG_PENDING, &st_gdata->st_state))) 
        {
		ST_DRV_ERR(" all protocols unregistered ");

		/* stop traffic on tty */
		if (st_gdata->tty) {
			tty_ldisc_flush(st_gdata->tty);
			stop_tty(st_gdata->tty);
		}

		/* all protocols now unregistered */
		st_kim_stop();
		/* disable ST LL */
		st_ll_disable();
	}
	else
	{	
		switch(channelid) /* TBD - remove this (should be done by GPS_DRV) */
		{
			case 0x9: 
				{
					/* GPS Power Command - OFF */
                    char gps_power_cmd[] = { 0x01, 0xC0, 0xFF, 0x01, 0x00 };        /*Previous 0x01, 0xFF, 0xC0, 0x01, 0x00*/
					ST_DRV_DBG(" Sending HCI command to turn off GPS...");
					/* Send HCI command to turn off GPS */
					st_int_write(gps_power_cmd, 5);
				}
				break;

			default:
				break;
		}
	}

	return err;
}
/* for channels making use of shared transport */
EXPORT_SYMBOL_GPL(st_unregister);

/*
 * called in protocol stack drivers
 * via the write function pointer
 */
long st_write(struct sk_buff *skb)
{
#ifdef DEBUG
	channel_t channelid;
#endif
	long len;
	struct st_data_s *st_data = st_gdata;

	spin_lock(&st_gdata->lock);//TI HSYoon 20110613

	if (unlikely(skb == NULL || st_gdata == NULL
		|| st_gdata->tty == NULL)) {
		ST_DRV_ERR("data/tty unavailable to perform write");
		spin_unlock(&st_gdata->lock);//TI HSYoon 20110613
		return ST_ERR_FAILURE;
	}
#ifdef DEBUG			/* open-up skb to read the 1st byte */
	channelid = skb->data[0];
	if (unlikely(( channelid != 1) && (channelid >= ST_MAX_CHANNELS || st_gdata->channels[channelid] == NULL))) {
		ST_DRV_ERR(" channelid %d not registered, and writing? ",
			   channelid);
		spin_unlock(&st_gdata->lock);//TI HSYoon 20110613
		return ST_ERR_FAILURE;
	}
#endif
	ST_DRV_DBG("%d to be written", skb->len);
	len = skb->len;

	/* st_ll to decide where to enqueue the skb */
	st_int_enqueue(skb);
	/* wake up */
	st_tx_wakeup(st_data);

	spin_unlock(&st_gdata->lock);//TI HSYoon 20110613

	/* return number of bytes written */
	return len;
}
EXPORT_SYMBOL_GPL(st_write);


/********************************************************************/
/*
 * functions called from TTY layer
 */
static int st_tty_open(struct tty_struct *tty)
{
	int err = ST_SUCCESS;
	ST_DRV_DBG("%s ", __func__);

  ST_LOG("st_tty_open enter....................\n");

	st_gdata->tty = tty;

	/* don't do an wakeup for now */
	clear_bit(TTY_DO_WRITE_WAKEUP, &tty->flags);

	/* mem already allocated
	 */
	tty->receive_room = 65536;
	/* Flush any pending characters in the driver and discipline. */
	tty_ldisc_flush(tty);
	tty_driver_flush_buffer(tty);
	/*
	 * signal to UIM via KIM that -
	 * installation of N_TI_WL ldisc is complete
	 */
	st_kim_complete();
	ST_DRV_DBG("done %s", __func__);
	return err;
}

static void st_tty_close(struct tty_struct *tty)
{
	unsigned char i = ST_MAX_CHANNELS;
	unsigned long flags = 0;

	ST_DRV_DBG("%s ", __func__);
  ST_LOG("st_tty_close enter....................\n");


	/* TODO:
	 * if a channel has been registered & line discipline
	 * un-installed for some reason - what should be done ?
	 */
	spin_lock_irqsave(&st_gdata->lock, irq_save_flags);//TI HSYoon 20110613
	for (i = 0; i < ST_MAX_CHANNELS; i++) {
		if (st_gdata->channels[i] != NULL)
			ST_DRV_ERR("%d not un-registered", i);
		st_gdata->channels[i] = NULL;
	}
	spin_unlock_irqrestore(&st_gdata->lock, irq_save_flags);//TI HSYoon 20110613
	/*
	 * signal to UIM via KIM that -
	 * N_TI_WL ldisc is un-installed
	 */
	st_kim_complete();
	st_gdata->tty = NULL;
	/* Flush any pending characters in the driver and discipline. */
	tty_ldisc_flush(tty);
	tty_driver_flush_buffer(tty);

	spin_lock_irqsave(&st_gdata->lock, irq_save_flags);//TI HSYoon 20110613
	/* empty out txq and tx_waitq */
	skb_queue_purge(&st_gdata->txq);
	skb_queue_purge(&st_gdata->tx_waitq);
	/* reset the TTY Rx states of ST */
	st_gdata->rx_count = 0;
	st_gdata->rx_state = ST_W4_PACKET_TYPE;
	kfree_skb(st_gdata->rx_skb);
	st_gdata->rx_skb = NULL;
	spin_unlock_irqrestore(&st_gdata->lock, irq_save_flags);//TI HSYoon 20110613

	ST_DRV_DBG("%s: done ", __func__);
}

static void st_tty_receive(struct tty_struct *tty, const unsigned char *data,
			   char *tty_flags, int count)
{
#if defined(SHOW_ST_LOG)//VERBOSE
	printk("%s(%d) : %x %x %x %x %x %x %x %x %x\n", __func__, count, data[0],data[1],data[2],data[3],data[4],data[5],data[6],data[7],data[8]);
#endif

#if 1 /* Introducing delay for stability of FM on and OPP 20110630 */
	if( data[0] == 0x8 ) //FM
	     mdelay(5);
	else	//BT
              mdelay(1);
#endif

	/*
	 * if fw download is in progress then route incoming data
	 * to KIM for validation
	 */
// LGE_LGE_WCDMA_FEATURE_MERGE  START
// FIX ME!
#ifdef TMP_ST_RECV_FIX // Temporary changes to fix kernel panic during MO
     if(st_recv == NULL)
     {
        ST_LOG("st_recv == NULL\n");
        return;
     }
#endif //TMP_ST_RECV_FIX
// LGE_LGE_WCDMA_FEATURE_MERGE  END
#if 1    /* TIK_BT OPP fail issue */
        //tty_throttle(tty);
        //spin_lock_irqsave(&st_gdata->lock, irq_save_flags); //TI HSYoon 20110616
        spin_lock(&st_gdata->lock);
#endif
		st_recv(data, count);
#if 1    /* TIK_BT OPP fail issue */
        spin_unlock(&st_gdata->lock);
        //spin_unlock_irqrestore(&st_gdata->lock, irq_save_flags); //TI HSYoon 20110616
        //tty_unthrottle(tty);
#endif

	//msleep(1);
	ST_DRV_VER("done %s", __func__);
}

/* wake-up function called in from the TTY layer
 * inside the internal wakeup function will be called
 */
static void st_tty_wakeup(struct tty_struct *tty)
{
	ST_DRV_DBG("%s ", __func__);
	/* don't do an wakeup for now */
	clear_bit(TTY_DO_WRITE_WAKEUP, &tty->flags);

	/* call our internal wakeup */
	st_tx_wakeup((void *)st_gdata);
}

static void st_tty_flush_buffer(struct tty_struct *tty)
{
	ST_DRV_DBG("%s ", __func__);

	kfree_skb(st_gdata->tx_skb);
	st_gdata->tx_skb = NULL;

	tty->ops->flush_buffer(tty);
	return;
}

/********************************************************************/
static int __init st_core_init(void)
{
	long err;
	static struct tty_ldisc_ops *st_ldisc_ops;

	/* populate and register to TTY line discipline */
	st_ldisc_ops = kzalloc(sizeof(*st_ldisc_ops), GFP_KERNEL);
	if (!st_ldisc_ops) {
		ST_DRV_ERR("no mem to allocate");
		return -ENOMEM;
	}

	st_ldisc_ops->magic = TTY_LDISC_MAGIC;
	st_ldisc_ops->name = "n_st";	/*"n_hci"; */
	st_ldisc_ops->open = st_tty_open;
	st_ldisc_ops->close = st_tty_close;
	st_ldisc_ops->receive_buf = st_tty_receive;
	st_ldisc_ops->write_wakeup = st_tty_wakeup;
	st_ldisc_ops->flush_buffer = st_tty_flush_buffer;
	st_ldisc_ops->owner = THIS_MODULE;

	err = tty_register_ldisc(N_TI_WL, st_ldisc_ops);
	if (err) {
		ST_DRV_ERR("error registering %d line discipline %ld",
			   N_TI_WL, err);
		kfree(st_ldisc_ops);
		return err;
	}
	ST_DRV_DBG("registered n_shared line discipline");

	st_gdata = kzalloc(sizeof(struct st_data_s), GFP_KERNEL);
	if (!st_gdata) {
		ST_DRV_ERR("memory allocation failed");
		err = tty_unregister_ldisc(N_TI_WL);
		if (err)
			ST_DRV_ERR("unable to un-register ldisc %ld", err);
		kfree(st_ldisc_ops);
		err = -ENOMEM;
		return err;
	}

	/* Initialize ST TxQ and Tx waitQ queue head. All BT/FM/GPS module skb's
	 * will be pushed in this queue for actual transmission.
	 */
	skb_queue_head_init(&st_gdata->txq);
	skb_queue_head_init(&st_gdata->tx_waitq);

	/* Locking used in st_int_enqueue() to avoid multiple execution */
	spin_lock_init(&st_gdata->lock);
	//spin_lock_init(&st_gdata->register_lock);//hsyoon 20110616
  

	/* ldisc_ops ref to be only used in __exit of module */
	st_gdata->ldisc_ops = st_ldisc_ops;

	err = st_kim_init();
	if (err) {
		ST_DRV_ERR("error during kim initialization(%ld)", err);
		kfree(st_gdata);
		err = tty_unregister_ldisc(N_TI_WL);
		if (err)
			ST_DRV_ERR("unable to un-register ldisc");
		kfree(st_ldisc_ops);
		return -1;
	}

	err = st_ll_init();
	if (err) {
		ST_DRV_ERR("error during st_ll initialization(%ld)", err);
		err = st_kim_deinit();
		kfree(st_gdata);
		err = tty_unregister_ldisc(N_TI_WL);
		if (err)
			ST_DRV_ERR("unable to un-register ldisc");
		kfree(st_ldisc_ops);
		return -1;
	}
	return 0;
}

static void __exit st_core_exit(void)
{
	long err;
	/* internal module cleanup */
	err = st_ll_deinit();
	if (err)
		ST_DRV_ERR("error during deinit of ST LL %ld", err);
	err = st_kim_deinit();
	if (err)
		ST_DRV_ERR("error during deinit of ST KIM %ld", err);

	if (st_gdata != NULL) {
		/* Free ST Tx Qs and skbs */
		skb_queue_purge(&st_gdata->txq);
		skb_queue_purge(&st_gdata->tx_waitq);
		kfree_skb(st_gdata->rx_skb);
		kfree_skb(st_gdata->tx_skb);
		/* TTY ldisc cleanup */
		err = tty_unregister_ldisc(N_TI_WL);
		if (err)
			ST_DRV_ERR("unable to un-register ldisc %ld", err);
		kfree(st_gdata->ldisc_ops);
		/* free the global data pointer */
		kfree(st_gdata);
	}
}

module_init(st_core_init);
module_exit(st_core_exit);
MODULE_AUTHOR("Pavan Savoy <pavan_savoy@ti.com>");
MODULE_DESCRIPTION("Shared Transport Driver for TI BT/FM/GPS combo chips ");
MODULE_LICENSE("GPL");
