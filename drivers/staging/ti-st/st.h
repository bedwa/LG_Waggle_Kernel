/*
 *  Shared Transport Header file
 *  	To be included by the protocol stack drivers for
 *  	Texas Instruments BT,FM and GPS combo chip drivers
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

#ifndef ST_H
#define ST_H

#include <linux/skbuff.h>
/*
 * st.h
 */
#define N_TI_WL	23	/* Ldisc for TI's WL BT, FM, GPS combo chips */

/* some gpios have active high, others like fm have
 * active low
 */
enum kim_gpio_state {
	KIM_GPIO_INACTIVE,
	KIM_GPIO_ACTIVE,
};

enum {
	ST_ERR_FAILURE = -1,	/* check struct */
	ST_SUCCESS,
	ST_ERR_PENDING = -5,	/* to call reg_complete_cb */
	ST_ERR_ALREADY,		/* already registered */
	ST_ERR_INPROGRESS,
	ST_ERR_NOPROTO,		/* protocol not supported */
};



/*
 * definition of hci channel index
 */
typedef unsigned char channel_t;

#define ST_MAX_CHANNELS  16 /* maximum ST channels (==clients) */	


/*
 * the list of gpios on chip
 */
enum gpio_t {
	ST_GPIO_BT,
	ST_GPIO_FM,
	ST_GPIO_GPS,
	ST_GPIO_MAX
};



/* per protocol structure
 * for BT/FM and GPS
 */
struct st_proto_s {
/*
 * index of HCI channel to be regietered (enable reception of packets on this channel)
 */
	channel_t channelid;	
/*
 * to be called by ST when data arrives
 */
	long (*recv) (struct sk_buff *);
/*
 * for future use, logic now to be in ST
 */
	unsigned char (*match_packet) (const unsigned char *data);
/*
 * subsequent registration return PENDING,
 * signalled complete by this callback function
 */
	void (*reg_complete_cb) (char data);

/*
 * channel specific parameters
 */
	unsigned short max_frame_size;	/* maximum HCI packet length (header + payload) */
	unsigned short header_size;		/* size (in bytes) of packet header */
	unsigned char  length_offset;	/* offset (in bytes) of length field within packet header */
	unsigned char  length_size;		/* size of length field (in bytes) within packet header */  
	enum gpio_t		gpio_id;		/* index (corresponds to "platform_xxx" definition) of Shutdown gpio assigned to the channel 
									 * ST_GPIO_MAX means there is no specific GPIO for this channel */
};

extern long st_register(struct st_proto_s *new_proto);
extern long st_unregister(channel_t channelid);
extern long st_write(struct sk_buff *skb);

#if 0
//#define SHOW_ST_LOG
#define ST_LOG(fmt, arg...)  printk(KERN_ERR "(st):"fmt"\n" , ## arg)
#define ST_LOG_X(fmt, arg...)
#else
#define ST_LOG(fmt, arg...)
#define ST_LOG_X(fmt, arg...)
#endif

#endif /* ST_H */
