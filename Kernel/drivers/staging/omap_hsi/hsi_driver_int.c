/*
 * hsi_driver_int.c
 *
 * Implements HSI interrupt functionality.
 *
 * Copyright (C) 2007-2008 Nokia Corporation. All rights reserved.
 * Copyright (C) 2009 Texas Instruments, Inc.
 *
 * Author: Carlos Chinea <carlos.chinea@nokia.com>
 * Author: Sebastien JAN <s-jan@ti.com>
 *
 * This package is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * THIS PACKAGE IS PROVIDED ``AS IS'' AND WITHOUT ANY EXPRESS OR
 * IMPLIED WARRANTIES, INCLUDING, WITHOUT LIMITATION, THE IMPLIED
 * WARRANTIES OF MERCHANTIBILITY AND FITNESS FOR A PARTICULAR PURPOSE.
 */

#include "hsi_driver.h"

void hsi_reset_ch_read(struct hsi_channel *ch)
{
	ch->read_data.addr = NULL;
	ch->read_data.size = 0;
	ch->read_data.lch = -1;
}

void hsi_reset_ch_write(struct hsi_channel *ch)
{
	ch->write_data.addr = NULL;
	ch->write_data.size = 0;
	ch->write_data.lch = -1;
}

/* Check if a Write (data transfer from AP to CP) is
 * ongoing for a given HSI channel
 */
bool hsi_is_channel_busy(struct hsi_channel *ch)
{
	if (ch->write_data.addr == NULL)
		return false;

	/* Note: we do not check if there is a read pending, because incoming */
	/* data will trigger an interrupt (FIFO or DMA), and wake up the */
	/* platform, so no need to keep the clocks ON. */
	return true;
}

/* Check if a HSI port is busy :
 * - data transfer (Write) is ongoing for a given HSI channel
 * - CAWAKE is high
 * - Currently in HSI interrupt tasklet
 * - Currently in HSI CAWAKE tasklet (for SSI)
 */
bool hsi_is_hsi_port_busy(struct hsi_port *pport)
{
	struct hsi_dev *hsi_ctrl = pport->hsi_controller;
	bool cur_cawake = hsi_get_cawake(pport);
	int ch;

	if (pport->in_int_tasklet) {
		dev_dbg(hsi_ctrl->dev, "Interrupt tasklet running\n");
		return true;
	}

	if (pport->in_cawake_tasklet) {
		dev_dbg(hsi_ctrl->dev, "SSI Cawake tasklet running\n");
		return true;
	}

	if (cur_cawake) {
		dev_dbg(hsi_ctrl->dev, "Port %d: WAKE status: acwake_status %d,"
			"cur_cawake %d", pport->port_number,
			pport->acwake_status, cur_cawake);
		return true;
	}

	for (ch = 0; ch < pport->max_ch; ch++)
		if (hsi_is_channel_busy(&pport->hsi_channel[ch])) {
			dev_dbg(hsi_ctrl->dev, "Port %d; channel %d "
				"busy\n", pport->port_number, ch);
			return true;
		}

	return false;
}

/* Check if HSI controller is busy :
 * - One of the HSI port is busy
 * - Currently in HSI DMA tasklet
 */
bool hsi_is_hsi_controller_busy(struct hsi_dev *hsi_ctrl)
{
	int port;

	if (hsi_ctrl->in_dma_tasklet) {
		dev_dbg(hsi_ctrl->dev, "DMA tasklet running\n");
		return true;
	}

	for (port = 0; port < hsi_ctrl->max_p; port++)
		if (hsi_is_hsi_port_busy(&hsi_ctrl->hsi_port[port])) {
			dev_dbg(hsi_ctrl->dev, "Port %d busy\n", port + 1);
			return true;
		}

	dev_dbg(hsi_ctrl->dev, "No activity on HSI controller\n");
	return false;
}

bool hsi_is_hst_port_busy(struct hsi_port *pport)
{
	unsigned int port = pport->port_number;
	void __iomem *base = pport->hsi_controller->base;
	u32 txstateval;

	txstateval = hsi_inl(base, HSI_HST_TXSTATE_REG(port)) &
			HSI_HST_TXSTATE_VAL_MASK;

	if (txstateval != HSI_HST_TXSTATE_IDLE) {
		dev_dbg(pport->hsi_controller->dev, "HST port %d busy, "
			"TXSTATE=%d\n", port, txstateval);
		return true;
	}

	return false;
}

bool hsi_is_hst_controller_busy(struct hsi_dev *hsi_ctrl)
{
	int port;

	for (port = 0; port < hsi_ctrl->max_p; port++)
		if (hsi_is_hst_port_busy(&hsi_ctrl->hsi_port[port]))
			return true;

	return false;
}


/* Enables the CAWAKE, BREAK, or ERROR interrupt for he given port */
int hsi_driver_enable_interrupt(struct hsi_port *pport, u32 flag)
{
	hsi_outl_or(flag, pport->hsi_controller->base,
		    HSI_SYS_MPU_ENABLE_REG(pport->port_number, pport->n_irq));

	return 0;
}

/* Enables the Data Accepted Interrupt of HST for the given channel */
int hsi_driver_enable_write_interrupt(struct hsi_channel *ch, u32 * data)
{
	struct hsi_port *p = ch->hsi_port;
	unsigned int port = p->port_number;
	unsigned int channel = ch->channel_number;

	hsi_outl_or(HSI_HST_DATAACCEPT(channel), p->hsi_controller->base,
		    HSI_SYS_MPU_ENABLE_CH_REG(port, p->n_irq, channel));

	return 0;
}

/* Enables the Data Available Interrupt of HSR for the given channel */
int hsi_driver_enable_read_interrupt(struct hsi_channel *ch, u32 * data)
{
	struct hsi_port *p = ch->hsi_port;
	unsigned int port = p->port_number;
	unsigned int channel = ch->channel_number;

	hsi_outl_or(HSI_HSR_DATAAVAILABLE(channel), p->hsi_controller->base,
		    HSI_SYS_MPU_ENABLE_CH_REG(port, p->n_irq, channel));

	return 0;
}

void hsi_driver_cancel_write_interrupt(struct hsi_channel *ch)
{
	struct hsi_port *p = ch->hsi_port;
	unsigned int port = p->port_number;
	unsigned int channel = ch->channel_number;
	void __iomem *base = p->hsi_controller->base;
	u32 enable;
	long buff_offset;

	enable = hsi_inl(base,
			 HSI_SYS_MPU_ENABLE_CH_REG(port, p->n_irq, channel));

	if (!(enable & HSI_HST_DATAACCEPT(channel))) {
		dev_dbg(&ch->dev->device, "Write cancel on not "
			"enabled channel %d ENABLE REG 0x%08X", channel,
			enable);
		return;
	}

	hsi_outl_and(~HSI_HST_DATAACCEPT(channel), base,
		     HSI_SYS_MPU_ENABLE_CH_REG(port, p->n_irq, channel));

	buff_offset = hsi_hst_bufstate_f_reg(p->hsi_controller, port, channel);
	if (buff_offset >= 0)
		hsi_outl_and(~HSI_BUFSTATE_CHANNEL(channel), base, buff_offset);
	hsi_reset_ch_write(ch);
}

void hsi_driver_cancel_read_interrupt(struct hsi_channel *ch)
{
	struct hsi_port *p = ch->hsi_port;
	unsigned int port = p->port_number;
	unsigned int channel = ch->channel_number;
	void __iomem *base = p->hsi_controller->base;

	hsi_outl_and(~HSI_HSR_DATAAVAILABLE(channel), base,
		     HSI_SYS_MPU_ENABLE_CH_REG(port, p->n_irq, channel));
	hsi_reset_ch_read(ch);
}

void hsi_driver_disable_write_interrupt(struct hsi_channel *ch)
{
	struct hsi_port *p = ch->hsi_port;
	unsigned int port = p->port_number;
	unsigned int channel = ch->channel_number;
	void __iomem *base = p->hsi_controller->base;

	hsi_outl_and(~HSI_HST_DATAACCEPT(channel), base,
		     HSI_SYS_MPU_ENABLE_CH_REG(port, p->n_irq, channel));
}

void hsi_driver_disable_read_interrupt(struct hsi_channel *ch)
{
	struct hsi_port *p = ch->hsi_port;
	unsigned int port = p->port_number;
	unsigned int channel = ch->channel_number;
	void __iomem *base = p->hsi_controller->base;

	hsi_outl_and(~HSI_HSR_DATAAVAILABLE(channel), base,
		     HSI_SYS_MPU_ENABLE_CH_REG(port, p->n_irq, channel));
}

/* HST_ACCEPTED interrupt processing */
static void hsi_do_channel_tx(struct hsi_channel *ch)
{
	struct hsi_dev *hsi_ctrl = ch->hsi_port->hsi_controller;
	void __iomem *base = hsi_ctrl->base;
	unsigned int n_ch;
	unsigned int n_p;
	unsigned int irq;
	long buff_offset;

	n_ch = ch->channel_number;
	n_p = ch->hsi_port->port_number;
	irq = ch->hsi_port->n_irq;

	dev_dbg(hsi_ctrl->dev,
		"Data Accepted interrupt for channel %d.\n", n_ch);

	hsi_driver_disable_write_interrupt(ch);

	if (ch->write_data.addr == NULL) {
		dev_err(hsi_ctrl->dev, "Error, NULL Write address.\n");
		hsi_reset_ch_write(ch);

	} else {
		buff_offset = hsi_hst_buffer_reg(hsi_ctrl, n_p, n_ch);
		if (buff_offset >= 0) {
			hsi_outl(*(ch->write_data.addr), base, buff_offset);
			ch->write_data.addr = NULL;
		}
	}

	spin_unlock(&hsi_ctrl->lock);
	dev_dbg(hsi_ctrl->dev, "Calling ch %d write callback.\n", n_ch);
	(*ch->write_done) (ch->dev, 1);
	spin_lock(&hsi_ctrl->lock);
}

/* HSR_AVAILABLE interrupt processing */
static void hsi_do_channel_rx(struct hsi_channel *ch)
{
	struct hsi_dev *hsi_ctrl = ch->hsi_port->hsi_controller;
	void __iomem *base = ch->hsi_port->hsi_controller->base;
	unsigned int n_ch;
	unsigned int n_p;
	unsigned int irq;
	long buff_offset;
	int rx_poll = 0;
	int data_read = 0;
	int fifo, fifo_words_avail;

	n_ch = ch->channel_number;
	n_p = ch->hsi_port->port_number;
	irq = ch->hsi_port->n_irq;

	dev_dbg(hsi_ctrl->dev,
		"Data Available interrupt for channel %d.\n", n_ch);

	/* Disable interrupts for polling if not needed */
	if (!(ch->flags & HSI_CH_RX_POLL))
		hsi_driver_disable_read_interrupt(ch);

	/*
	 * Check race condition: RX transmission initiated but DMA transmission
	 * already started - acknowledge then ignore interrupt occurence
	 */
	if (ch->read_data.lch != -1)
		goto done;

	if (ch->flags & HSI_CH_RX_POLL)
		rx_poll = 1;

	if (ch->read_data.addr) {
		buff_offset = hsi_hsr_buffer_reg(hsi_ctrl, n_p, n_ch);
		if (buff_offset >= 0) {
			data_read = 1;
			*(ch->read_data.addr) = hsi_inl(base, buff_offset);
		}
	}

	hsi_reset_ch_read(ch);

	/* Check if FIFO is correctly emptied */
	if (hsi_driver_device_is_hsi(to_platform_device(hsi_ctrl->dev))) {
		fifo = hsi_fifo_get_id(hsi_ctrl, n_ch, n_p);
		if (unlikely(fifo < 0)) {
			dev_err(hsi_ctrl->dev, "No valid FIFO id found for "
					       "channel %d.\n", n_ch);
			goto done;
		}
		fifo_words_avail = hsi_get_rx_fifo_occupancy(hsi_ctrl, fifo);
		if (fifo_words_avail)
			dev_dbg(hsi_ctrl->dev,
				"WARNING: RX FIFO %d not empty after CPU copy, "
				"remaining %d/%d frames\n",
				fifo, fifo_words_avail, HSI_HSR_FIFO_SIZE);
	}

done:
	if (rx_poll) {
		spin_unlock(&hsi_ctrl->lock);
		hsi_port_event_handler(ch->hsi_port,
				       HSI_EVENT_HSR_DATAAVAILABLE,
				       (void *)n_ch);
		spin_lock(&hsi_ctrl->lock);
	}

	if (data_read) {
		spin_unlock(&hsi_ctrl->lock);
		dev_dbg(hsi_ctrl->dev, "Calling ch %d read callback.\n", n_ch);
		(*ch->read_done) (ch->dev, 1);
		spin_lock(&hsi_ctrl->lock);
	}
}

/* CAWAKE line management */
void hsi_do_cawake_process(struct hsi_port *pport)
{
	struct hsi_dev *hsi_ctrl = pport->hsi_controller;
	bool cawake_status = hsi_get_cawake(pport);

	/* Deal with init condition */
	if (unlikely(pport->cawake_status < 0))
		pport->cawake_status = !cawake_status;

	/* Check CAWAKE line status */
	if (cawake_status) {
		dev_dbg(hsi_ctrl->dev, "CAWAKE rising edge detected\n");

		/* Check for possible mismatch (race condition) */
		if (unlikely(pport->cawake_status)) {
			dev_warn(hsi_ctrl->dev,
				"CAWAKE race is detected: %s.\n",
				"HI -> LOW -> HI");
			spin_unlock(&hsi_ctrl->lock);
			hsi_port_event_handler(pport, HSI_EVENT_CAWAKE_DOWN,
						NULL);
			spin_lock(&hsi_ctrl->lock);
		}
		pport->cawake_status = 1;

		spin_unlock(&hsi_ctrl->lock);
		hsi_port_event_handler(pport, HSI_EVENT_CAWAKE_UP, NULL);
		spin_lock(&hsi_ctrl->lock);
	} else {
		dev_dbg(hsi_ctrl->dev, "CAWAKE falling edge detected\n");

		if (unlikely(!pport->cawake_status)) {
			dev_warn(hsi_ctrl->dev,
				"CAWAKE race is detected: %s.\n",
				"LOW -> HI -> LOW");
			spin_unlock(&hsi_ctrl->lock);
			hsi_port_event_handler(pport, HSI_EVENT_CAWAKE_UP,
						NULL);
			spin_lock(&hsi_ctrl->lock);
		}
		pport->cawake_status = 0;

		spin_unlock(&hsi_ctrl->lock);
		hsi_port_event_handler(pport, HSI_EVENT_CAWAKE_DOWN, NULL);
		spin_lock(&hsi_ctrl->lock);
	}
}

/**
 * hsi_driver_int_proc - check all channels / ports for interrupts events
 * @hsi_ctrl - HSI controler data
 * @status_offset: interrupt status register offset
 * @enable_offset: interrupt enable regiser offset
 * @start: interrupt index to start on
 * @stop: interrupt index to stop on
 *
 * returns the bitmap of processed events
 *
 * This function calls the related processing functions and triggered events.
 * Events are cleared after corresponding function has been called.
*/
static u32 hsi_driver_int_proc(struct hsi_port *pport,
				unsigned long status_offset,
				unsigned long enable_offset, unsigned int start,
				unsigned int stop)
{
	struct hsi_dev *hsi_ctrl = pport->hsi_controller;
	void __iomem *base = hsi_ctrl->base;
	unsigned int port = pport->port_number;
	unsigned int channel;
	u32 status_reg;
	u32 hsr_err_reg;
	u32 channels_served = 0;

	/* Get events status */
	status_reg = hsi_inl(base, status_offset);
	status_reg &= hsi_inl(base, enable_offset);

	if (pport->cawake_off_event) {
		dev_dbg(hsi_ctrl->dev, "CAWAKE detected from OFF mode.\n");
	} else if (!status_reg) {
		dev_dbg(hsi_ctrl->dev, "Channels [%d,%d] : no event, exit.\n",
			start, stop);
		return 0;
	} else {
		dev_dbg(hsi_ctrl->dev, "Channels [%d,%d] : Events 0x%08x\n",
			start, stop, status_reg);
	}

	if (status_reg & HSI_BREAKDETECTED) {
		dev_info(hsi_ctrl->dev, "Hardware BREAK on port %d\n", port);
		hsi_outl(0, base, HSI_HSR_BREAK_REG(port));
		spin_unlock(&hsi_ctrl->lock);
		hsi_port_event_handler(pport, HSI_EVENT_BREAK_DETECTED, NULL);
		spin_lock(&hsi_ctrl->lock);

		channels_served |= HSI_BREAKDETECTED;
	}

	if (status_reg & HSI_ERROROCCURED) {
		hsr_err_reg = hsi_inl(base, HSI_HSR_ERROR_REG(port));
		if (hsr_err_reg & HSI_HSR_ERROR_SIG)
			dev_err(hsi_ctrl->dev, "HSI ERROR Port %d: 0x%x: %s\n",
				port, hsr_err_reg, "Signal Error");
		if (hsr_err_reg & HSI_HSR_ERROR_FTE)
			dev_err(hsi_ctrl->dev, "HSI ERROR Port %d: 0x%x: %s\n",
				port, hsr_err_reg, "Frame Timeout Error");
		if (hsr_err_reg & HSI_HSR_ERROR_TBE)
			dev_err(hsi_ctrl->dev, "HSI ERROR Port %d: 0x%x: %s\n",
				port, hsr_err_reg, "Tailing Bit Error");
		if (hsr_err_reg & HSI_HSR_ERROR_RME)
			dev_err(hsi_ctrl->dev, "HSI ERROR Port %d: 0x%x: %s\n",
				port, hsr_err_reg, "RX Mapping Error");
		if (hsr_err_reg & HSI_HSR_ERROR_TME)
			dev_err(hsi_ctrl->dev, "HSI ERROR Port %d: 0x%x: %s\n",
				port, hsr_err_reg, "TX Mapping Error");
		/* Clear error event bit */
		hsi_outl(hsr_err_reg, base, HSI_HSR_ERRORACK_REG(port));
		if (hsr_err_reg) {	/* ignore spurious errors */
			spin_unlock(&hsi_ctrl->lock);
			hsi_port_event_handler(pport, HSI_EVENT_ERROR, NULL);
			spin_lock(&hsi_ctrl->lock);
		} else
			dev_dbg(hsi_ctrl->dev, "Spurious HSI error!\n");

		channels_served |= HSI_ERROROCCURED;
	}

	/* CAWAKE falling or rising edge detected */
	if ((status_reg & HSI_CAWAKEDETECTED) || pport->cawake_off_event) {
		hsi_do_cawake_process(pport);

		channels_served |= HSI_CAWAKEDETECTED;
		pport->cawake_off_event = false;
	}

	for (channel = start; channel < stop; channel++) {
		if (status_reg & HSI_HST_DATAACCEPT(channel)) {
			hsi_do_channel_tx(&pport->hsi_channel[channel]);
			channels_served |= HSI_HST_DATAACCEPT(channel);
		}

		if (status_reg & HSI_HSR_DATAAVAILABLE(channel)) {
			hsi_do_channel_rx(&pport->hsi_channel[channel]);
			channels_served |= HSI_HSR_DATAAVAILABLE(channel);
		}

		if (status_reg & HSI_HSR_DATAOVERRUN(channel)) {
			/*HSI_TODO : Data overrun handling*/
			dev_err(hsi_ctrl->dev,
				"Data overrun in real time mode !\n");
		}
	}

	/* Reset status bits */
	hsi_outl(channels_served, base, status_offset);

	return channels_served;
}

static u32 hsi_process_int_event(struct hsi_port *pport)
{
	unsigned int port = pport->port_number;
	unsigned int irq = pport->n_irq;
	u32 status_reg;

	/* Process events for channels 0..7 */
	status_reg = hsi_driver_int_proc(pport,
			    HSI_SYS_MPU_STATUS_REG(port, irq),
			    HSI_SYS_MPU_ENABLE_REG(port, irq),
			    0, min(pport->max_ch, (u8) HSI_SSI_CHANNELS_MAX));

	/* Process events for channels 8..15 */
	if (pport->max_ch > HSI_SSI_CHANNELS_MAX)
		status_reg |= hsi_driver_int_proc(pport,
				    HSI_SYS_MPU_U_STATUS_REG(port, irq),
				    HSI_SYS_MPU_U_ENABLE_REG(port, irq),
				    HSI_SSI_CHANNELS_MAX, pport->max_ch);

	return status_reg;
}

static void do_hsi_tasklet(unsigned long hsi_port)
{
	struct hsi_port *pport = (struct hsi_port *)hsi_port;
	struct hsi_dev *hsi_ctrl = pport->hsi_controller;
	u32 status_reg;

	dev_dbg(hsi_ctrl->dev, "Int Tasklet : clock_enabled=%d\n",
		hsi_ctrl->clock_enabled);

	spin_lock(&hsi_ctrl->lock);
	hsi_clocks_enable(hsi_ctrl->dev, __func__);
	pport->in_int_tasklet = true;

	status_reg = hsi_process_int_event(pport);

	pport->in_int_tasklet = false;
	hsi_clocks_disable(hsi_ctrl->dev, __func__);
	spin_unlock(&hsi_ctrl->lock);

	enable_irq(pport->irq);
}

static irqreturn_t hsi_mpu_handler(int irq, void *p)
{
	struct hsi_port *pport = p;

	tasklet_hi_schedule(&pport->hsi_tasklet);

	/* Disable interrupt until Bottom Half has cleared the IRQ status */
	/* register */
	disable_irq_nosync(pport->irq);

	return IRQ_HANDLED;
}

int __init hsi_mpu_init(struct hsi_port *hsi_p, const char *irq_name)
{
	int err;

	tasklet_init(&hsi_p->hsi_tasklet, do_hsi_tasklet, (unsigned long)hsi_p);

	dev_info(hsi_p->hsi_controller->dev, "Registering IRQ %s (%d)\n",
						irq_name, hsi_p->irq);
	err = request_irq(hsi_p->irq, hsi_mpu_handler, IRQF_NO_SUSPEND,
			  irq_name, hsi_p);
	if (err < 0) {
		dev_err(hsi_p->hsi_controller->dev, "FAILED to MPU request"
			" IRQ (%d) on port %d", hsi_p->irq, hsi_p->port_number);
		return -EBUSY;
	}

	return 0;
}

void hsi_mpu_exit(struct hsi_port *hsi_p)
{
	tasklet_disable(&hsi_p->hsi_tasklet);
	free_irq(hsi_p->irq, hsi_p);
}
