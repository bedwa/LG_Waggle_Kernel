/*
 * ifx_n721_spi.c -- Serial peheripheral interface framing layer for IFX modem.
 *
 * Copyright (C) 2009 Texas Instruments
 * Authors:	Umesh Bysani <bysani@ti.com> and
 *		Shreekanth D.H <sdh@ti.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 675 Mass Ave, Cambridge, MA 02139, USA.
 */

#include <linux/init.h>
#include <linux/module.h>
#include <linux/ioctl.h>
#include <linux/fs.h>
#include <linux/device.h>
#include <linux/err.h>
#include <linux/list.h>
#include <linux/errno.h>
#include <linux/mutex.h>
#include <linux/slab.h>
#include <linux/smp_lock.h>
#include <asm/uaccess.h>
#include <linux/irq.h>
#include <mach/gpio.h>
#include <linux/tty.h>
#include <linux/tty_driver.h>
#include <linux/tty_flip.h>
#include <linux/interrupt.h>
#include <linux/completion.h>
#include <linux/spi/spi.h>
#include <linux/workqueue.h>
// LGE_CHANGE_S [youngseok.jeong@lge.com] 2010-10-11 to retry IPC transmission when SRDY is not signaled by CP
#include <linux/delay.h>
// LGE_CHANGE_E [youngseok.jeong@lge.com] 2010-10-11 to retry IPC transmission when SRDY is not signaled by CP

#include <linux/spi/ifx_n721_spi.h>

#include <linux/cosmo/charger_rt9524.h>

// hyoungsuk.jang@lge.com 20110111 
#include <plat/lge_err_handler.h>
// CHEOLGWAK  2011-2-26 CP_CRASH_COUNT
#include <plat/lge_nvdata_handler.h>
// CHEOLGWAK  2011-2-26 CP_CRASH_COUNT
//#define LGE_DUMP_SPI_BUFFER

//#define LGE_VT_DATA_DUMP /* baeyoung.park@lge.com 2011-01-07 Debug code for VT */
// CHEOLGWAK  2011-2-28 
#include <linux/input.h>
// CHEOLGWAK  2011-2-28

//CRASH TIME INFORMATION ADD. 2011-04-23 eunae.kim
#include <linux/rtc.h>


// CHEOLGWAK  2011-2-28 
#ifndef ENABLE_CP_CRASH_RESET
#define EVENT_KEY KEY_F24 //194, Need to be changed
#define EVENT_HARD_RESET_KEY	195	//this key number is not used in input.h
#endif
// CHEOLGWAK  2011-2-28 

#if defined(LGE_VT_DATA_DUMP)
#define VT_MUX12_DEBUG

#if defined(VT_MUX12_DEBUG)
extern bool mux_12_open;
#endif

extern unsigned int mux_written_counter;
#endif


#define MODEM_SEND 121 //gpio_121 MODEM_SEND
/* ################################################################################################################ */

/* Structure used to store private data */
struct ifx_spi_data {
	dev_t			devt;
	spinlock_t		spi_lock;
	struct spi_device	*spi;
	struct list_head	device_entry;
        struct completion       ifx_read_write_completion;
        struct tty_struct       *ifx_tty;

	/* buffer is NULL unless this device is open (users > 0) */
	struct mutex		buf_lock;
	unsigned		users;
        unsigned int		throttle;
        struct work_struct      ifx_work;
#if 1 /* baeyoung.park@lge.com 2011-01-07 Remove compile warning */
	struct workqueue_struct *ifx_wq;
#else
        struct work_queue_struct *ifx_wq;
#endif
// hgahn
int ifx_spi_lock;
		
};

union ifx_spi_frame_header{
	struct{
		unsigned int curr_data_size:12;
		unsigned int more:1;
		unsigned int res1:1;
		unsigned int res2:2;      
		unsigned int next_data_size:12;
		unsigned int ri:1;
		unsigned int dcd:1;
		unsigned int cts_rts:1;
		unsigned int dsr_dtr:1;
	}ifx_spi_header;
	unsigned char framesbytes[IFX_SPI_HEADER_SIZE];
};

struct ifx_spi_data	*gspi_data;
struct tty_driver 	*ifx_spi_tty_driver;

/* ################################################################################################################ */
/* Global Declarations */
unsigned long		minors[IFX_N_SPI_MINORS / BITS_PER_LONG];
unsigned int		ifx_master_initiated_transfer = 0;
unsigned int		ifx_spi_count;
//<jongho3.lee@lge.com> LGE_CHANGE_S  ril_retry_count
unsigned int		ifx_ril_is_modem_alive = 1;
//<jongho3.lee@lge.com> LGE_CHANGE_E  ril_retry_count
unsigned int		ifx_sender_buf_size;
unsigned int		ifx_receiver_buf_size;
unsigned int		ifx_current_frame_size;
unsigned int		ifx_valid_frame_size;
unsigned int		ifx_ret_count;
const unsigned char 	*ifx_spi_buf;
unsigned char		*ifx_tx_buffer;
unsigned char           *ifx_rx_buffer;

/* Function Declarations */
static void ifx_spi_set_header_info(unsigned char *header_buffer, unsigned int curr_buf_size, unsigned int next_buf_size);
static int ifx_spi_get_header_info(unsigned int *valid_buf_size);
static void ifx_spi_set_mrdy_signal(int value);
static void ifx_spi_setup_transmission(void);
static void ifx_spi_send_and_receive_data(struct ifx_spi_data *spi_data);
static int ifx_spi_get_next_frame_size(int count);
static int ifx_spi_allocate_frame_memory(unsigned int memory_size);
static void ifx_spi_buffer_initialization(void);
static unsigned int ifx_spi_sync_read_write(struct ifx_spi_data *spi_data, unsigned int len);
static irqreturn_t ifx_spi_handle_srdy_irq(int irq, void *handle);
static void ifx_spi_handle_work(struct work_struct *work);


// hgahn
unsigned char rx_dummy[]={0xff,0xff,0xff,0xff};


// hyoungsuk.jang@lge.com  20110105 CP Crash INT [START]
static struct work_struct CP_CRASH_INT_wq;
// CHEOLGWAK  2011-5-14 delayed work queue
static struct delayed_work	cp_crash_int_delayed_wq;
// CHEOLGWAK  2011-5-14 delayed work queue

static void CP_CRASH_wq_func(struct work_struct *cp_crash_wq);
static irqreturn_t CP_CRASH_interrupt_handler(s32 irq, void *data);
// hyoungsuk.jang@lge.com  20110105 CP Crash INT [END]

#ifndef ENABLE_CP_CRASH_RESET
//eunae.kim@lge.com 20110301
static struct input_dev *in_dev = NULL;
#endif

/* ################################################################################################################ */

static LIST_HEAD(device_list);
static DEFINE_MUTEX(device_list_lock);

/* ################################################################################################################ */

/* IFX SPI Operations */

/*
 * Function opens a tty device when called from user space
 */
static int 
ifx_spi_open(struct tty_struct *tty, struct file *filp)
{
	int status = 0;

	struct ifx_spi_data *spi_data = gspi_data;  
	spi_data->ifx_tty = tty;
	tty->driver_data = spi_data;
	
// hgahn
	spi_data->ifx_spi_lock =0;
	
	ifx_spi_buffer_initialization();
	spi_data->throttle = 0;
	return status;
}

/*
 * Function closes a opened a tty device when called from user space
 */
static void 
ifx_spi_close(struct tty_struct *tty, struct file *filp)
{  
	//struct ifx_spi_data *spi_data = (struct ifx_spi_data *)tty->driver_data;
	//spi_data->ifx_tty = NULL;
	//tty->driver_data = NULL;
}

/*
 * Function is called from user space to send data to MODEM, it setups the transmission, enable MRDY signal and
 * waits for SRDY signal HIGH from MDOEM. Then starts transmission and reception of data to and from MODEM.
 * Once data read from MODEM is transferred to TTY core flip buffers, then "ifx_read_write_completion" is set
 * and this function returns number of bytes sent to MODEM
 */


#ifdef LGE_DUMP_SPI_BUFFER
#define COL_SIZE 50
static void dump_spi_buffer(const unsigned char *txt, const unsigned char *buf, int count)
{
    char dump_buf_str[COL_SIZE+1];

    if (buf != NULL) 
    {
        int j = 0;
        char *cur_str = dump_buf_str;
        unsigned char ch;
        while((j < COL_SIZE) && (j  < count))
        {
            ch = buf[j];
            if ((ch < 32) || (ch > 126))
            {
                *cur_str = '.';
            } else
            {
                *cur_str = ch;
            }
            cur_str++;
            j++;
        }
        *cur_str = 0;
        printk("%s:count:%d [%s]\n", txt, count, dump_buf_str);                        
    }
    else
    {
        printk("%s: buffer is NULL\n", txt);                 
    }
}
#elif defined(LGE_VT_DATA_DUMP)
#define COL_SIZE 160

static void dump_spi_wr_buffer(const unsigned char *buf, int count)
{

  static unsigned int spi_write_cnt = 0;
  char dump_buf_str[(COL_SIZE+1)*2];

#if 0
  if(mux_12_open == 0)		
   	return;
#endif

  #if 0  
    /* Print all SPI data */
    if ((buf != NULL) && (count >= 160))
    {        
	int j = 0;        
	char *cur_str = dump_buf_str;        
	unsigned char ch;        

	while((j < COL_SIZE) && (j  < count))        
	{            
            ch = buf[j];
            sprintf(cur_str, "%0.2x", ch);			
            cur_str = cur_str+2;
            j++;        
        }
	*cur_str = 0;       
	printk("SPI WR[%d] [%s]\n", spi_write_cnt, dump_buf_str);                            	
	spi_write_cnt++;
    }
  #else
    /* Print only SPI writing count */
    if ((buf != NULL) && (count >= 160))
    {
         printk("IFX OK[%d], MUX[%d]\n", spi_write_cnt, mux_written_counter);         
	 spi_write_cnt++;
    }
  #endif	
}


static void dump_spi_rd_buffer(const unsigned char *buf, int count)
{

    char dump_buf_str[COL_SIZE+1];

#if defined(VT_MUX12_DEBUG)
  if(mux_12_open == 0)		
   	return;
#endif

  if ((buf != NULL)  && (count <= 170))
    {
        int j = 0;
        char *cur_str = dump_buf_str;

        while((j < count))    
        {
            if (0 == buf[j])
            	{
                *cur_str = '.';            	
            	}
	    else
		{
		  if ((j <=4) && ((0x0d == buf[j]) || (0x0a == buf[j])))
		  	{
		                *cur_str = '.'; 
		  	}
		  else
		  	{
		              *cur_str = buf[j];		
		  	}
		}		


		if ((0x0d == buf[j]) && (0x0a == buf[j]))
		{
        		*cur_str = ']';
			break;
		}			
        
            cur_str++;
		
            j++;
        }
        *cur_str = 0;
        printk("SPI Read count:%d [%s]\n", count, dump_buf_str);                        
    }	
}

#else
#define dump_spi_buffer(...)
#endif

static int 
ifx_spi_write(struct tty_struct *tty, const unsigned char *buf, int count)
{	
	struct ifx_spi_data *spi_data = (struct ifx_spi_data *)tty->driver_data;
        ifx_ret_count = 0;

#ifdef LGE_DUMP_SPI_BUFFER
    dump_spi_buffer("ifx_spi_write()", buf, count);
#elif defined(LGE_VT_DATA_DUMP)
    if (count == 167) // 167 means 160(MUX data) + 7 (DLC Frame Header + Tails)
    {
       // dump_spi_wr_buffer(buf, count);
    }
#endif

// hgahn
	if(spi_data->ifx_spi_lock)
		return ifx_ret_count;

	spi_data->ifx_tty = tty;
	spi_data->ifx_tty->low_latency = 1;
	if( !buf ){
		printk("File: ifx_n721_spi.c\tFunction: int ifx_spi_write()\t Buffer NULL\n");
		return ifx_ret_count;
	}
	if(!count){
		printk("File: ifx_n721_spi.c\tFunction: int ifx_spi_write()\t Count is ZERO\n");
		return ifx_ret_count;
	}
	ifx_master_initiated_transfer = 1;
	ifx_spi_buf = buf;
	ifx_spi_count = count;

// LGE_CHANGE_S [youngseok.jeong@lge.com] 2010-10-11 to retry IPC transmission when SRDY is not signaled by CP
	/* original code
	ifx_spi_set_mrdy_signal(1);	
		
	wait_for_completion(&spi_data->ifx_read_write_completion);
	*/
	{
		int i, max_retry_count=8;
		unsigned long timeout=HZ;
		long rc;

		for (i=0 ; i<max_retry_count ; i++) {
			// signal master ready
			ifx_spi_set_mrdy_signal(1);

			// wait for completion with timeout
			rc = wait_for_completion_timeout(
					&spi_data->ifx_read_write_completion,
					timeout);

			if (rc == 0) {		// timeout expired, retry
				printk("***** unable to detect SREADY within %lu, RETRY (counter=%d) *****\n", timeout, i+1);
				// lower master ready
				ifx_spi_set_mrdy_signal(0);

//<jongho3.lee@lge.com> LGE_CHANGE_S  ril_retry_count
				if(i == (max_retry_count-1))
				{
					{
						set_modem_alive(0);
						ifx_ril_is_modem_alive = 0;
					}
				}
//<jongho3.lee@lge.com> LGE_CHANGE_E  ril_retry_count
				// retry after delay
				udelay(100);		// 20 u sec delay
			} else {			// success or failure
				//printk("wait_for_completion_timeout timeout=%ld\n", timeout);
//<jongho3.lee@lge.com> LGE_CHANGE_S  ril_retry_count
				if(!ifx_ril_is_modem_alive)
				{
					set_modem_alive(1);
					ifx_ril_is_modem_alive = 1;
				}
//<jongho3.lee@lge.com> LGE_CHANGE_E  ril_retry_count
				break;
			}
		}
	}
// LGE_CHANGE_E [youngseok.jeong@lge.com] 2010-10-11 to retry IPC transmission when SRDY is not signaled by CP

	init_completion(&spi_data->ifx_read_write_completion);
	return ifx_ret_count; /* Number of bytes sent to the device */
}

/* This function should return number of free bytes left in the write buffer in this case always return 2048 */

static int 
ifx_spi_write_room(struct tty_struct *tty)
{	
	return 2048;
}


/* ################################################################################################################ */
/* These two functions are to be used in future to implement flow control (RTS & CTS)*/
/*static void 
ifx_spi_throttle(struct tty_struct *tty)
{
	unsigned int flags;
	struct ifx_spi_data *spi_data = (struct ifx_spi_data *)tty->driver_data;
	spi_data->ifx_tty = tty;
	spin_lock_irqsave(&spi_data->spi_lock, flags);
	spi_data->throttle = 1;
	spin_unlock_irqrestore(&spi_data->spi_lock, flags);
}

static void 
ifx_spi_unthrottle(struct tty_struct *tty)
{
	unsigned int flags;
	struct ifx_spi_data *spi_data = (struct ifx_spi_data *)tty->driver_data;
	spi_data->ifx_tty = tty;
	spin_lock_irqsave(&spi_data->spi_lock, flags);
	spi_data->throttle = 0;
	if( ifx_rx_buffer != NULL ){
	     tty_insert_flip_string(spi_data->ifx_tty, ifx_rx_buffer, valid_buffer_count);
	}
	spin_unlock_irqrestore(&spi_data->spi_lock, flags);  
}*/
/* ################################################################################################################ */

/* End of IFX SPI Operations */

/* ################################################################################################################ */

/* TTY - SPI driver Operations */

static int 
ifx_spi_probe(struct spi_device *spi)
{
	int ret;
	int status;
	int err;
	struct ifx_spi_data *spi_data;

	printk("[e] ifx_spi_probe\n");

	/* Allocate SPI driver data */
	spi_data = (struct ifx_spi_data*)kmalloc(sizeof(struct ifx_spi_data), GFP_KERNEL);
	if (!spi_data){
		return -ENOMEM;
        }

        status = ifx_spi_allocate_frame_memory(IFX_SPI_MAX_BUF_SIZE + IFX_SPI_HEADER_SIZE);
        if(status != 0){
		printk("File: ifx_n721_spi.c\tFunction: int ifx_spi_probe\tFailed to allocate memory for buffers\n");
		// WBT DEFECT FIX wooyoung1.kim@lge.com
		kfree(spi_data);
		// WBT DEFECT FIX wooyoung1.kim@lge.com
		return -ENOMEM;
        }
	
        dev_set_drvdata(&spi->dev,spi_data);
        spin_lock_init(&spi_data->spi_lock);
        INIT_WORK(&spi_data->ifx_work,ifx_spi_handle_work);
		printk("[e] INIT_WORK\n");

        spi_data->ifx_wq = create_singlethread_workqueue("ifxn721");
        if(!spi_data->ifx_wq){
		printk("Failed to setup workqueue - ifx_wq \n");          
        }
	init_completion(&spi_data->ifx_read_write_completion);

        /* Configure SPI */
        spi_data->spi = spi;
        spi->mode = SPI_MODE_1;
        spi->bits_per_word = 8;
        status = spi_setup(spi);
        if(status < 0){
			printk("Failed to setup SPI \n");
        }             

// hgahn
	spi_data->ifx_spi_lock =1;

	/* Enable SRDY Interrupt request - If the SRDY signal is high then ifx_spi_handle_srdy_irq() is called */
	status = request_irq(spi->irq, ifx_spi_handle_srdy_irq,  IRQF_TRIGGER_RISING, spi->dev.driver->name, spi_data);
	printk("[e] spi->irq:%d  status:%d\n", spi->irq, status);
	if (status != 0){
		printk(KERN_ERR "Failed to request IRQ for SRDY\n");
		printk(KERN_ERR "IFX SPI Probe Failed\n");
		if(ifx_tx_buffer){
			kfree(ifx_tx_buffer);
		}
		if(ifx_rx_buffer){
			kfree(ifx_rx_buffer);            
		}
		if(spi_data){
			kfree(spi_data);
		}          
	}
	else{
		gspi_data = spi_data;
	}
        /////////////////////////////////////////////////////////////////////////////////////////////
// jeehp
        enable_irq_wake(spi->irq);
//LGE_CHANGE [jongho3.lee@lge.com] use MODEM_SNED pin as sleep status of AP.
	ret = gpio_request(MODEM_SEND, "MODEM_SEND");
	if (ret < 0) {
		printk(KERN_ERR "%s: Failed to request GPIO_%d for MODEM_SEND\n", __func__, MODEM_SEND);
		return -ENOSYS;
	}
        gpio_direction_output(MODEM_SEND,1);
        gpio_set_value(MODEM_SEND,1);
////////////////////////////////////////////////////////////////////////////////////////////


// hyoungsuk.jang@lge.com  20110105 CP Crash INT [START]
	#define CP_CRASH_INT_N  26
	
	/* Assign GPIO  */
	ret = gpio_request(CP_CRASH_INT_N, "CP CRASH IRQ GPIO");
	if (ret < 0) {
		printk(KERN_INFO "[CP CRASH IRQ] GPIO#%03d is already occupied by other driver!\n", CP_CRASH_INT_N);
		return -ENOSYS;
	}

	/* Initializes GPIO direction before use or IRQ setting */
	ret = gpio_direction_input(CP_CRASH_INT_N);
	if (ret < 0) {
		printk(KERN_INFO "[CP CRASH IRQ] GPIO#%03d direction initialization failed!\n", CP_CRASH_INT_N);
		return -ENOSYS;
	}

	/* Registers MUIC work queue function */

	INIT_WORK(&CP_CRASH_INT_wq, CP_CRASH_wq_func);
// CHEOLGWAK  2011-5-14 delayed work queue
	INIT_DELAYED_WORK(&cp_crash_int_delayed_wq, CP_CRASH_wq_func);
// CHEOLGWAK  2011-5-14 delayed work queue
	/* 
	 * Set up an IRQ line and enable the involved interrupt handler.
	 * From this point, a MUIC_INT_N can invoke muic_interrupt_handler().
	 * muic_interrupt_handler merely calls schedule_work() with muic_wq_func().
	 * muic_wq_func() actually performs the accessory detection.
	 */
#ifdef CONFIG_MACH_LGE_COSMO_DOMASTIC
	ret = request_irq(gpio_to_irq(CP_CRASH_INT_N), CP_CRASH_interrupt_handler, IRQF_TRIGGER_FALLING, "cp_crash_irq", &spi->dev);
#else
	ret = request_irq(gpio_to_irq(CP_CRASH_INT_N), CP_CRASH_interrupt_handler, IRQF_TRIGGER_RISING, "cp_crash_irq", &spi->dev);
#endif
	if (ret < 0) {
		printk(KERN_INFO "[CP CRASH IRQ] GPIO#%03d IRQ line set up failed!\n", CP_CRASH_INT_N);
		free_irq(gpio_to_irq(CP_CRASH_INT_N), &spi->dev);
		return -ENOSYS;
	}
// hyoungsuk.jang@lge.com  20110105 CP Crash INT [END]

#ifndef ENABLE_CP_CRASH_RESET
	//20110301 eunae.kim LGE_RIL_RECOVERY
	in_dev = input_allocate_device();
	if (!in_dev) {
		printk("Can't allocate power button\n");
		return -ENOMEM;
	}

	in_dev->evbit[0] = BIT_MASK(EV_KEY);
	in_dev->keybit[BIT_WORD(EVENT_KEY)] = BIT_MASK(EVENT_KEY);
	in_dev->name = "ifxn721";
	in_dev->phys = "ifxn721/input0";
	in_dev->dev.parent = &spi->dev;

	err = input_register_device(in_dev);
	if (err) {
		printk("Can't register EVENT_KEY button: %d\n", err);
	}

#endif

	return status;
}

// hyoungsuk.jang@lge.com 20110110 MUIC mode change in case of trap [START]
static void CP_CRASH_wq_func(struct work_struct *cp_crash_wq)
{
	volatile unsigned long *make_panic = 0;
	extern void set_muic_mode(u32 mode);
	int ret;
	
// CRASH TIME INFORMATION ADD. 2011-04-23 eunae.kim
	struct timespec ts;
	struct rtc_time tm;
	getnstimeofday(&ts);
	rtc_time_to_tm(ts.tv_sec, &tm);
	
	printk(KERN_INFO "[CP CRASH IRQ] CP_CRASH_wq_func()");	
	printk(KERN_INFO "(%d-%02d-%02d %02d:%02d:%02d.%09lu UTC)\n", tm.tm_year + 1900, tm.tm_mon + 1, tm.tm_mday,
			tm.tm_hour, tm.tm_min, tm.tm_sec, ts.tv_nsec);	
// CHEOLGWAK  2011-5-14 delayed work queue
#ifdef CONFIG_MACH_LGE_COSMO_DOMASTIC
	if(!gpio_get_value(CP_CRASH_INT_N)){
#else
	if(gpio_get_value(CP_CRASH_INT_N)){
#endif
#if 0   // hyoungsuk.jang@lge.com prevent to make panic
		*make_panic = 0xDEAD;	
#endif
		//LGE_ChangeS jaesung.woo@lge.com 20110131 CIQ [START]
		lge_store_ciq_reset(0, LGE_NVDATA_IQ_RESET_EXCEPTION);
		//LGE_ChangeS jaesung.woo@lge.com 20110131 CIQ [END]

	// CHEOLGWAK  2011-2-26 CP_CRASH_COUNT
		{
			unsigned char data;
			lge_dynamic_nvdata_read(LGE_NVDATA_DYNAMIC_CP_CRASH_COUNT_OFFSET,&data,1);
			data++;
			lge_dynamic_nvdata_write(LGE_NVDATA_DYNAMIC_CP_CRASH_COUNT_OFFSET,&data,1);
		}
	// CHEOLGWAK  2011-2-26 CP_CRASH_COUNT

	// CHEOLGWAK  2011-2-28 

		// hyoungsuk.jang@lge.com 20110111 
		if (lge_is_crash_dump_enabled() != 1)
		{	
#ifndef ENABLE_CP_CRASH_RESET	//20110301 LGE_RIL_RECOVERY
			printk(" CP CRASH! immediate RIL/CP reset");
			input_report_key(in_dev, EVENT_KEY, 1);
			input_report_key(in_dev, EVENT_KEY, 0);
			input_sync(in_dev);
			printk("[CPW] input_report_key(): %d\n", EVENT_KEY);
#endif
			return;
		}

	// hyoungsuk.jang@lge.com 20110130 CP Crash Core Dump Season2 [START]
	//	set_muic_mode(7 /* MUIC_CP_UART */);
		printk(KERN_INFO "[CP CRASH IRQ] launch ifx_coredump process\n");	

	{
		char* argv[] = {"/system/bin/ifx_coredump", "CP_CRASH_IRQ", NULL};
		char *envp[] = { "HOME=/",	"PATH=/sbin:/bin:/system/bin",	NULL };	
		//@@ret = call_usermodehelper(argv[0], argv, envp, UMH_NO_WAIT);
		ret = call_usermodehelper(argv[0], argv, envp, UMH_WAIT_EXEC);
		printk(KERN_INFO "[CP CRASH IRQ] launch ifx_coredump process ret:%d\n",ret);	
	}

		gpio_set_value(82, 1);

#if 0
		// LED toggle
		int toggle = 0;
		for(;;)
		{
			if(toggle == 0)
			{
				gpio_set_value(82, 1);
				toggle = 1;
			}
			else                                 
			{
				gpio_set_value(82, 0);
				toggle = 0;
			}
			msleep(100);
		}
#endif	
	// hyoungsuk.jang@lge.com 20110130 CP Crash Core Dump Season2 [END]
	}
	else
	{
		return;		
	}
// CHEOLGWAK  2011-5-14 delayed work queue
}

static irqreturn_t CP_CRASH_interrupt_handler(s32 irq, void *data)
{
	/* Make the interrupt on CP CRASH INT wake up OMAP which is in suspend mode */
// CHEOLGWAK  2011-5-14 delayed work queue
	//schedule_work(&CP_CRASH_INT_wq);
	schedule_delayed_work( &cp_crash_int_delayed_wq, msecs_to_jiffies(500));
// CHEOLGWAK  2011-5-14 delayed work queue
	return IRQ_HANDLED;
}
// hyoungsuk.jang@lge.com 20110110 MUIC mode change in case of trap [START]


static int 
ifx_spi_remove(struct spi_device *spi)
{	
	struct ifx_spi_data *spi_data;
	spi_data = spi_get_drvdata(spi);

	// WBT DEFECT FIX wooyoung1.kim@lge.com
	if(spi_data == NULL){
		return 0;
	}
	// WBT DEFECT FIX wooyoung1.kim@lge.com
	
	spin_lock_irq(&spi_data->spi_lock);
	spi_data->spi = NULL;
	spi_set_drvdata(spi, NULL);
	spin_unlock_irq(&spi_data->spi_lock);

        if(ifx_tx_buffer){
		kfree(ifx_tx_buffer);
	}
        if(ifx_rx_buffer){
		kfree(ifx_rx_buffer);
	}
        if(spi_data){
		kfree(spi_data);
        }          
        return 0;
}

static int
ifx_spi_suspend(struct spi_device *spi)
{
	//LGE_CHANGE [jongho3.lee@lge.com] set sleep status of AP .
    gpio_set_value(MODEM_SEND,0);
    
    return 0;
}

static int
ifx_spi_resume(struct spi_device *spi)
{
    //@@printk("modem_chk = %d \n",gpio_get_value(177));
	//LGE_CHANGE [jongho3.lee@lge.com] set sleep status of AP .
    gpio_set_value(MODEM_SEND,1);
    
    return 0;
}
/* End of TTY - SPI driver Operations */

/* ################################################################################################################ */

static struct spi_driver ifx_spi_driver = {
	.driver = {
		.name = "ifxn721",
                .bus = &spi_bus_type,
		.owner = THIS_MODULE,
	},
	.probe = ifx_spi_probe,
	.remove = __devexit_p(ifx_spi_remove),
    .suspend = ifx_spi_suspend,
    .resume = ifx_spi_resume,
};

/*
 * Structure to specify tty core about tty driver operations supported in TTY SPI driver.
 */
static const struct tty_operations ifx_spi_ops = {
    .open = ifx_spi_open,
    .close = ifx_spi_close,
    .write = ifx_spi_write,
    .write_room = ifx_spi_write_room,
    //.throttle = ifx_spi_throttle,
    //.unthrottle = ifx_spi_unthrottle,
    //.set_termios = ifx_spi_set_termios,
};

/* ################################################################################################################ */

/*
 * Intialize frame sizes as "IFX_SPI_DEFAULT_BUF_SIZE"(128) bytes for first SPI frame transfer
 */
static void 
ifx_spi_buffer_initialization(void)
{
	ifx_sender_buf_size = IFX_SPI_DEFAULT_BUF_SIZE;
        ifx_receiver_buf_size = IFX_SPI_DEFAULT_BUF_SIZE;
}

/*
 * Allocate memeory for TX_BUFFER and RX_BUFFER
 */
static int 
ifx_spi_allocate_frame_memory(unsigned int memory_size)
{
	int status = 0;
	ifx_rx_buffer = kmalloc(memory_size+IFX_SPI_HEADER_SIZE, GFP_KERNEL);
	if (!ifx_rx_buffer){
		printk("Open Failed ENOMEM\n");
		status = -ENOMEM;
	}
	ifx_tx_buffer = kmalloc(memory_size+IFX_SPI_HEADER_SIZE, GFP_KERNEL);
	if (!ifx_tx_buffer){		
		printk("Open Failed ENOMEM\n");
		status = -ENOMEM;
	}
	if(status == -ENOMEM){
		if(ifx_tx_buffer){
			kfree(ifx_tx_buffer);
		}
		if(ifx_rx_buffer){
			kfree(ifx_rx_buffer);            
		}
	}
	return status;
}

/*
 * Function to set header information according to IFX SPI framing protocol specification
 */
static void 
ifx_spi_set_header_info(unsigned char *header_buffer, unsigned int curr_buf_size, unsigned int next_buf_size)
{
	int i;
	union ifx_spi_frame_header header;
	for(i=0; i<4; i++){
		header.framesbytes[i] = 0;
	}

	header.ifx_spi_header.curr_data_size = curr_buf_size;
	if(next_buf_size){
		header.ifx_spi_header.more=1;
		header.ifx_spi_header.next_data_size = next_buf_size;
	}
	else{
		header.ifx_spi_header.more=0;
		header.ifx_spi_header.next_data_size = 128;
	}

	for(i=3; i>=0; i--){
	header_buffer[i] = header.framesbytes[/*3-*/i];
	}
}

/*
 * Function to get header information according to IFX SPI framing protocol specification
 */
static int 
ifx_spi_get_header_info(unsigned int *valid_buf_size)
{
	int i;
	union ifx_spi_frame_header header;

	for(i=0; i<4; i++){
		header.framesbytes[i] = 0;
	}

	for(i=3; i>=0; i--){
		header.framesbytes[i] = ifx_rx_buffer[/*3-*/i];
	}

	*valid_buf_size = header.ifx_spi_header.curr_data_size;
	if(header.ifx_spi_header.more){
		return header.ifx_spi_header.next_data_size;
	}
	return 0;
}

/*
 * Function to set/reset MRDY signal
 */
static void 
ifx_spi_set_mrdy_signal(int value)
{
	gpio_set_value(IFX_MRDY_GPIO, value);
}

/*
 * Function to calculate next_frame_size required for filling in SPI frame Header
 */
static int 
ifx_spi_get_next_frame_size(int count)
{
	if(count > IFX_SPI_MAX_BUF_SIZE){
		return IFX_SPI_MAX_BUF_SIZE;    
	}
	else{   
		return count;
	}
}

/*
 * Function to setup transmission and reception. It implements a logic to find out the ifx_current_frame_size,
 * valid_frame_size and sender_next_frame_size to set in SPI header frame. Copys the data to be transferred from 
 * user space to TX buffer and set MRDY signal to HIGH to indicate Master is ready to transfer data.
 */
static void 
ifx_spi_setup_transmission(void)
{

	//printk("[e] ifx_spi_setup_transmission\n");

	if( (ifx_sender_buf_size != 0) || (ifx_receiver_buf_size != 0) ){
		if(ifx_sender_buf_size > ifx_receiver_buf_size){
			ifx_current_frame_size = ifx_sender_buf_size;
		}
		else{ 
			ifx_current_frame_size = ifx_receiver_buf_size;    
		}
		if(ifx_spi_count > 0){
			if(ifx_spi_count > ifx_current_frame_size){
				ifx_valid_frame_size = ifx_current_frame_size;
				ifx_spi_count = ifx_spi_count - ifx_current_frame_size;
			}
			else{
				ifx_valid_frame_size = ifx_spi_count;
				ifx_spi_count = 0;
			}
                }
		else{
			ifx_valid_frame_size = 0;
			ifx_sender_buf_size = 0;
		}
		ifx_sender_buf_size = ifx_spi_get_next_frame_size(ifx_spi_count);

		/* memset buffers to 0 */
		memset(ifx_tx_buffer,0,IFX_SPI_MAX_BUF_SIZE+IFX_SPI_HEADER_SIZE);
		memset(ifx_rx_buffer,0,IFX_SPI_MAX_BUF_SIZE+IFX_SPI_HEADER_SIZE);

		/* Set header information */
		ifx_spi_set_header_info(ifx_tx_buffer, ifx_valid_frame_size, ifx_sender_buf_size);
		if( ifx_valid_frame_size > 0 ){      
			memcpy(ifx_tx_buffer+IFX_SPI_HEADER_SIZE, ifx_spi_buf, ifx_valid_frame_size);
			ifx_spi_buf = ifx_spi_buf + ifx_valid_frame_size;
		}
	}
}


/*
 * Function starts Read and write operation and transfers received data to TTY core. It pulls down MRDY signal
 * in case of single frame transfer then sets "ifx_read_write_completion" to indicate transfer complete.
 */
static void 
ifx_spi_send_and_receive_data(struct ifx_spi_data *spi_data)
{
	unsigned int rx_valid_buf_size;
	int status = 0; 

	status = ifx_spi_sync_read_write(spi_data, ifx_current_frame_size+IFX_SPI_HEADER_SIZE); /* 4 bytes for header */                         
	if(status > 0){
#if defined(LGE_DUMP_SPI_BUFFER)
    dump_spi_buffer("ifx_spi_send_and_receive_data()[Trans]", &(ifx_tx_buffer[4]), COL_SIZE);
#elif defined(LGE_VT_DATA_DUMP)
    dump_spi_wr_buffer(&(ifx_tx_buffer[4]), ifx_valid_frame_size + 4);
#endif

		memset(ifx_tx_buffer,0,IFX_SPI_MAX_BUF_SIZE+IFX_SPI_HEADER_SIZE);
		ifx_ret_count = ifx_ret_count + ifx_valid_frame_size;
	}

	// hgahn
	if(memcmp(rx_dummy, ifx_rx_buffer, IFX_SPI_HEADER_SIZE) ==0) {

		ifx_receiver_buf_size = 0;
		return;
	}

	/* Handling Received data */
	ifx_receiver_buf_size = ifx_spi_get_header_info(&rx_valid_buf_size);


// hgahn
	if((spi_data->throttle == 0) && (rx_valid_buf_size != 0) && !(spi_data->ifx_spi_lock)){
#ifdef LGE_DUMP_SPI_BUFFER
    dump_spi_buffer("ifx_spi_send_and_receive_data()[Recev]", &(ifx_rx_buffer[4]), COL_SIZE);
#elif defined(LGE_VT_DATA_DUMP)
    //dump_spi_rd_buffer(&(ifx_rx_buffer[4]), rx_valid_buf_size-2); /* MUX에서의 Ctrl & Flag Byte 제거 */
#endif

		tty_insert_flip_string(spi_data->ifx_tty, ifx_rx_buffer+IFX_SPI_HEADER_SIZE, rx_valid_buf_size);
		tty_flip_buffer_push(spi_data->ifx_tty);
	}  
	/*else
  	{ 
	handle RTS and CTS in SPI flow control
	Reject the packet as of now 
	}*/
}

/*
 * Function copies the TX_BUFFER and RX_BUFFER pointer to a spi_transfer structure and add it to SPI tasks.
 * And calls SPI Driver function "spi_sync" to start data transmission and reception to from MODEM
 */
static unsigned int 
ifx_spi_sync_read_write(struct ifx_spi_data *spi_data, unsigned int len)
{
	int status;
	struct spi_message	m;
	struct spi_transfer	t = {
					.tx_buf		= ifx_tx_buffer,
                        		.rx_buf		= ifx_rx_buffer,
					.len		= len,
				    };

//	printk("[e] ifx_spi_sync_read_write\n");

	
    spi_message_init(&m);
	spi_message_add_tail(&t, &m);
	
	if (spi_data->spi == NULL)
		status = -ESHUTDOWN;
	else
		status = spi_sync(spi_data->spi, &m);
	
	if (status == 0){          
		status = m.status;
		if (status == 0)
			status = m.actual_length;
	}
        else{
		printk("File: ifx_n721_spi.c\tFunction: unsigned int ifx_spi_sync\tTransmission UNsuccessful\n");
        }
	return status;
}

/*
 * Function is a Interrupt service routine, is called when SRDY signal goes HIGH. It set up transmission and
 * reception if it is a Slave initiated data transfer. For both the cases Master intiated/Slave intiated
 * transfer it starts data transfer. 
 */
static irqreturn_t 
ifx_spi_handle_srdy_irq(int irq, void *handle)
{
	struct ifx_spi_data *spi_data = (struct ifx_spi_data *)handle;
	//printk(KERN_ERR "File: ifx_n721_spi.c\tFunction: int ifx_spi_handle_srdy_irq()\t HYO22\n");
	queue_work(spi_data->ifx_wq, &spi_data->ifx_work);    
	return IRQ_HANDLED; 
}

static void 
ifx_spi_handle_work(struct work_struct *work)
{
	struct ifx_spi_data *spi_data = container_of(work, struct ifx_spi_data, ifx_work);
	if (!ifx_master_initiated_transfer){
		ifx_spi_setup_transmission();
		ifx_spi_set_mrdy_signal(1);
		ifx_spi_send_and_receive_data(spi_data);
		/* Once data transmission is completed, the MRDY signal is lowered */
		if((ifx_sender_buf_size == 0)  && (ifx_receiver_buf_size == 0)){
			ifx_spi_set_mrdy_signal(0);
			ifx_spi_buffer_initialization();
		}

		/* We are processing the slave initiated transfer in the mean time Mux has requested master initiated data transfer */
		/* Once Slave initiated transfer is complete then start Master initiated transfer */
		if(ifx_master_initiated_transfer == 1){
		/* It is a condition where Slave has initiated data transfer and both SRDY and MRDY are high and at the end of data transfer		
	 	* MUX has some data to transfer. MUX initiates Master initiated transfer rising MRDY high, which will not be detected at Slave-MODEM.
	 	* So it was required to rise MRDY high again */
	 		udelay(100);//TI JANGHAN
	 		//udelay(10);// reduce delay for performnace
            ifx_spi_set_mrdy_signal(1);
		}
	}
	else{
		ifx_spi_setup_transmission();     
		ifx_spi_send_and_receive_data(spi_data);
		/* Once data transmission is completed, the MRDY signal is lowered */
		if(ifx_sender_buf_size == 0){
			if(ifx_receiver_buf_size == 0){		
				ifx_spi_set_mrdy_signal(0);
				udelay(100);////TI JANGHAN
				//udelay(10);// reduce delay for performance
				ifx_spi_buffer_initialization();
			}
			ifx_master_initiated_transfer = 0;
			complete(&spi_data->ifx_read_write_completion);
		}
	}
}


/* ################################################################################################################ */


/* ################################################################################################################ */

/* Initialization Functions */

/*
 * Initialization function which allocates and set different parameters for TTY SPI driver. Registers the tty driver 
 * with TTY core and register SPI driver with the Kernel. It validates the GPIO pins for MRDY and then request an IRQ
 * on SRDY GPIO pin for SRDY signal going HIGH. In case of failure of SPI driver register cases it unregister tty driver
 * from tty core.
 */
static int 
__init ifx_spi_init(void)
{
int status = 0;

	/* Allocate and Register a TTY device */
	ifx_spi_tty_driver = alloc_tty_driver(IFX_N_SPI_MINORS);
	if (!ifx_spi_tty_driver){
		printk(KERN_ERR "Fail to allocate TTY Driver\n");
		return -ENOMEM;
	}

	/* initialize the tty driver */
	ifx_spi_tty_driver->owner = THIS_MODULE;
	ifx_spi_tty_driver->driver_name = "ifxn721";
	ifx_spi_tty_driver->name = "ttyspi";
	ifx_spi_tty_driver->major = IFX_SPI_MAJOR;
	ifx_spi_tty_driver->minor_start = 0;
	ifx_spi_tty_driver->type = TTY_DRIVER_TYPE_SERIAL;
	ifx_spi_tty_driver->subtype = SERIAL_TYPE_NORMAL;
	ifx_spi_tty_driver->flags = TTY_DRIVER_REAL_RAW;
	ifx_spi_tty_driver->init_termios = tty_std_termios;
	ifx_spi_tty_driver->init_termios.c_cflag = B9600 | CS8 | CREAD | HUPCL | CLOCAL;
	tty_set_operations(ifx_spi_tty_driver, &ifx_spi_ops);

	status = tty_register_driver(ifx_spi_tty_driver);
	if (status){
		printk(KERN_ERR "Failed to register IFX SPI tty driver");
		put_tty_driver(ifx_spi_tty_driver);
		return status;
	}

	/* Register SPI Driver */
	status = spi_register_driver(&ifx_spi_driver);
	printk(KERN_ERR "spi_register_driver return %d\n", status);
	if (status < 0){ 
		printk(KERN_ERR "Failed to register SPI device");
		tty_unregister_driver(ifx_spi_tty_driver);
		put_tty_driver(ifx_spi_tty_driver);
		return status;
	}
	return status;
}

module_init(ifx_spi_init);


/*
 * Exit function to unregister SPI driver and tty SPI driver
 */
static void 
__exit ifx_spi_exit(void)
{  
	spi_unregister_driver(&ifx_spi_driver);
	tty_unregister_driver(ifx_spi_tty_driver);
        put_tty_driver(ifx_spi_tty_driver);
}

module_exit(ifx_spi_exit);

/* End of Initialization Functions */

/* ################################################################################################################ */

MODULE_AUTHOR("Umesh Bysani and Shreekanth D.H, <bysani@ti.com> <sdh@ti.com>");
MODULE_DESCRIPTION("IFX SPI Framing Layer");
MODULE_LICENSE("GPL");
