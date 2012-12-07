/*-
 * Copyright (C) 2012 Intel Corporation
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *
 * THIS SOFTWARE IS PROVIDED BY THE AUTHOR AND CONTRIBUTORS ``AS IS'' AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED.  IN NO EVENT SHALL THE AUTHOR OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS
 * OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY
 * OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF
 * SUCH DAMAGE.
 */
#include <sys/modctl.h>
#include <sys/blkdev.h>
#include <sys/types.h>
#include <sys/errno.h>
#include <sys/param.h>
#include <sys/stropts.h>
#include <sys/stream.h>
#include <sys/strsubr.h>
#include <sys/kmem.h>
#include <sys/conf.h>
#include <sys/devops.h>
#include <sys/ksynch.h>
#include <sys/stat.h>
#include <sys/modctl.h>
#include <sys/debug.h>
#include <sys/pci.h>
#include <sys/sysmacros.h>

#include "nvme.h"
#include "nvme_private.h"

extern int nvme_qpair_register_interrupt(nvme_qpair_t *q);
extern int nvme_qpair_unregister_interrupt(nvme_qpair_t *q);

int
nvme_wait_for_completion(nvme_tracker_t *tr)
{
	nvme_qpair_t *qpair = tr->qpair;
	nvme_completion_t *cpl;
	int timeout = 20;

	for (;timeout > 0;) {
		off_t offset;
		cpl = &qpair->cpl[qpair->cq_head];

		offset = (off_t)cpl - (off_t)qpair->cmd;
		(void) ddi_dma_sync(qpair->ctrlr->dma_handle, offset,
			sizeof(struct nvme_completion), DDI_DMA_SYNC_FORKERNEL); 
		if (cpl->p != qpair->phase) {
			DELAY(10);
			continue;
		}
		qpair->sq_head = cpl->sqhd;

		qpair->cq_head ++;
		if (qpair->cq_head == qpair->num_entries) {
			qpair->cq_head = 0;
			qpair->phase = !qpair->phase;
		}
		nvme_mmio_write_4(qpair->ctrlr, doorbell[qpair->id].cq_hdbl,
		    qpair->cq_head);

		qpair->act_tr[tr->cid] = NULL;
		return 0;
	}
	return ETIMEDOUT;
}

void
nvme_qpair_process_completions(nvme_qpair_t *qpair)
{
	nvme_tracker_t	*tr;
	nvme_completion_t	*cpl;

	qpair->num_intr_handler_calls ++;

	/* we work with the completion queue, no mutexes required */
	/* because there is only one code flow per CQ */
	for (;;) {

		off_t offset;  

		cpl = &qpair->cpl[qpair->cq_head];

		offset = (off_t)cpl - (off_t)qpair->cmd;
		(void) ddi_dma_sync(qpair->ctrlr->dma_handle, offset,
			sizeof(struct nvme_completion), DDI_DMA_SYNC_FORKERNEL); 
		if (cpl->p != qpair->phase) {
			break;
		}
		ASSERT(cpl->cid <= qpair->num_entries);

		mutex_enter(&qpair->hw_mutex);

		tr = qpair->act_tr[cpl->cid];
		qpair->act_tr[cpl->cid] = NULL;
		qpair->sq_head = cpl->sqhd;

		mutex_exit(&qpair->hw_mutex);

		ASSERT(tr != NULL);

		if (tr->cb_fn)
			tr->cb_fn(tr->cb_arg, cpl, tr);

		qpair->cq_head ++;

		if (qpair->cq_head == qpair->num_entries) {
			qpair->cq_head = 0;
			qpair->phase = !qpair->phase;
		}
		nvme_mmio_write_4(qpair->ctrlr, doorbell[qpair->id].cq_hdbl,
		    qpair->cq_head);
	}
}

int
nvme_qpair_construct(nvme_qpair_t *qpair, uint32_t id,
    uint16_t vector, uint32_t num_entries, uint32_t num_trackers,
    uint32_t max_xfer_size, nvme_controller_t *ctrlr)
{
	nvme_tracker_t	*tr;
	uint32_t		i;
	size_t len;
	ddi_dma_cookie_t  cookie;
	uint_t  cookie_count;
	int res;

	qpair->id = id;
	qpair->vector = vector;
	qpair->num_entries = num_entries;

	qpair->num_trackers = num_trackers;
	qpair->max_xfer_size = max_xfer_size;
	qpair->ctrlr = ctrlr;
	/*
	 * First time through the completion queue, HW will set phase
	 *  bit on completions to 1.  So set this to 1 here, indicating
	 *  we're looking for a 1 to know which entries have completed.
	 *  we'll toggle the bit each time when the completion queue
	 *  rolls over.
	 */
	qpair->phase = 1;

	if (ctrlr->msix_enabled) {
		/*
		 * MSI-X vector resource IDs start at 1, so we add one to
		 *  the queue's vector to get the corresponding rid to use.
		 */
		qpair->rid = vector + 1;
	}
	else
		qpair->rid = vector;

	qpair->num_cmds = 0;
	qpair->num_intr_handler_calls = 0;
	qpair->sq_head = qpair->sq_tail = qpair->cq_head = 0;

	res = ddi_dma_mem_alloc(qpair->ctrlr->dma_handle, 
			qpair->num_entries * sizeof(nvme_command_t), 
			qpair->ctrlr->devattr, IOMEM_DATA_UNCACHED,
			DDI_DMA_SLEEP, NULL, (char **)&qpair->cmd, &len, 
			&qpair->cmd_dma_acc_handle);

	if (res != DDI_SUCCESS) {
		return (ENOMEM);
	}
	/* is the dma_mem_alloc()ed memory zeroed? */
	memset(qpair->cmd, 0, qpair->num_entries * sizeof(struct nvme_command));

	res = ddi_dma_mem_alloc(qpair->ctrlr->dma_handle, 
			qpair->num_entries * sizeof(nvme_completion_t), 
			qpair->ctrlr->devattr, IOMEM_DATA_UNCACHED,
			DDI_DMA_SLEEP, NULL, (char **)&qpair->cpl, &len, 
			&qpair->cpl_dma_acc_handle);

	if (res != DDI_SUCCESS) {
		(void)ddi_dma_mem_free(&qpair->cmd_dma_acc_handle);
		return (ENOMEM);
	}
	/* is the memory zeroed per default ? */
	memset(qpair->cpl, 0, qpair->num_entries * sizeof(nvme_completion_t));
	res = ddi_dma_addr_bind_handle(qpair->ctrlr->dma_handle, 
			(struct as *)NULL, (caddr_t)qpair->cmd, 
			qpair->num_entries * sizeof(nvme_command_t),
			DDI_DMA_RDWR | DDI_DMA_CONSISTENT, DDI_DMA_DONTWAIT,
			0, &cookie, &cookie_count);
 
	switch (res) {
		case DDI_DMA_MAPPED:
		/* everything is ok */
			break;
		/* TODO: update to count some statistic based on the bind_handle() result */
		default:
			(void)ddi_dma_mem_free(&qpair->cmd_dma_acc_handle);
			(void)ddi_dma_mem_free(&qpair->cpl_dma_acc_handle);
			return (ENOMEM);
	}
	ASSERT(cookie_count == 1);

	qpair->cmd_bus_addr = cookie.dmac_laddress;

	res = ddi_dma_addr_bind_handle(qpair->ctrlr->dma_handle, 
			(struct as *)NULL, (caddr_t)qpair->cpl, 
			qpair->num_entries * sizeof(nvme_completion_t),
			DDI_DMA_RDWR | DDI_DMA_CONSISTENT, DDI_DMA_DONTWAIT,
			0, &cookie, &cookie_count);

	switch (res) {
		case DDI_DMA_MAPPED:
		/* everything is ok */
			break;
		default:
			(void)ddi_dma_mem_free(&qpair->cmd_dma_acc_handle);
			(void)ddi_dma_mem_free(&qpair->cpl_dma_acc_handle);
			return ENOMEM;
	}
	ASSERT(cookie_count == 1);

	qpair->cpl_bus_addr = cookie.dmac_laddress;

	qpair->sq_tdbl_off = nvme_mmio_offsetof(doorbell[id].sq_tdbl);
	qpair->cq_hdbl_off = nvme_mmio_offsetof(doorbell[id].cq_hdbl);

	qpair->trackers = kmem_zalloc(sizeof(nvme_tracker_t) * 
			qpair->num_entries, KM_SLEEP);

	TAILQ_INIT(&qpair->free_trackers);
	for (i = 0; i < qpair->num_entries; i ++) {
		nvme_tracker_t *t = &qpair->trackers[i];

		t->qpair = qpair;
		t->cid = i;
		TAILQ_INSERT_TAIL(&qpair->free_trackers, t, next);
	}
	qpair->act_tr = kmem_zalloc(sizeof(nvme_tracker_t *) *
			 qpair->num_entries, KM_SLEEP); 

	nvme_qpair_register_interrupt(qpair);
	mutex_init(&qpair->hw_mutex, NULL, MUTEX_DRIVER, 
				DDI_INTR_PRI(qpair->soft_intr_pri));
	mutex_init(&qpair->free_trackers_mutex, NULL, MUTEX_DRIVER, 
				DDI_INTR_PRI(qpair->soft_intr_pri));
	return (0);
}

static void
nvme_qpair_destroy(nvme_qpair_t *qpair)
{
	nvme_tracker_t *tr;


	nvme_qpair_unregister_interrupt(qpair);

	if (qpair->act_tr) {
		/* release all active trackers */
		int i;
		for (i = 0; i < qpair->num_entries; i ++) {
			tr = qpair->act_tr[i];
			if (tr) {
				mutex_destroy(&tr->mutex);
				cv_destroy(&tr->cv);
			}
		}
		kmem_free(qpair->act_tr, sizeof(tr) * qpair->num_entries);
		qpair->act_tr = NULL;
	}
	kmem_free(qpair->trackers, sizeof(nvme_tracker_t) * qpair->num_entries);
	qpair->trackers = NULL;

	mutex_destroy(&qpair->hw_mutex);
	mutex_destroy(&qpair->free_trackers_mutex);

	(void) ddi_dma_mem_free(&qpair->cmd_dma_acc_handle);
	(void) ddi_dma_mem_free(&qpair->cpl_dma_acc_handle);
}

void
nvme_admin_qpair_destroy(nvme_qpair_t *qpair)
{
	/*
	 * For NVMe, you don't send delete queue commands for the admin
	 *  queue, so we just need to unload and free the cmd and cpl memory.
	 */
	(void) ddi_dma_mem_free(&qpair->cmd_dma_acc_handle);
	(void) ddi_dma_mem_free(&qpair->cpl_dma_acc_handle);

	(void) ddi_dma_unbind_handle(qpair->ctrlr->dma_handle);

	nvme_qpair_destroy(qpair);
}

static void
nvme_free_cmd_ring(void *arg, const nvme_completion_t *status, nvme_tracker_t *tr)
{
	nvme_qpair_t *qpair = (struct nvme_qpair *)arg;

	(void) ddi_dma_mem_free(&qpair->cmd_dma_acc_handle);
	qpair->cmd = NULL;
}

static void
nvme_free_cpl_ring(void *arg, const nvme_completion_t *status, nvme_tracker_t *tr)
{
	nvme_qpair_t *qpair = (struct nvme_qpair *)arg;

	(void) ddi_dma_mem_free(&qpair->cpl_dma_acc_handle);
	qpair->cpl = NULL;
}

void
nvme_io_qpair_destroy(nvme_qpair_t *qpair)
{
	nvme_controller_t *ctrlr = qpair->ctrlr;

	if (qpair->num_entries > 0) {

		nvme_ctrlr_cmd_delete_io_sq(ctrlr, qpair, nvme_free_cmd_ring,
		    qpair);
		/* Spin until free_cmd_ring sets qpair->cmd to NULL. */
		while (qpair->cmd) {
			DELAY(5);
		}
		nvme_ctrlr_cmd_delete_io_cq(ctrlr, qpair, nvme_free_cpl_ring,
		    qpair);
		/* Spin until free_cpl_ring sets qpair->cmd to NULL. */
		while (qpair->cpl) {
			DELAY(5);
		}
		nvme_qpair_destroy(qpair);
	}
}

void
nvme_qpair_submit_cmd(nvme_tracker_t *tr)
{

	off_t offset;
	nvme_qpair_t *qpair = tr->qpair;
	struct nvme_command *cmd;

	mutex_enter(&qpair->hw_mutex);

	qpair->act_tr[tr->cid] = tr;
	tr->cmd.cid = tr->cid;

	cmd = &qpair->cmd[qpair->sq_tail];

	/* Copy the command from the tracker to the submission queue. */
	memcpy(cmd, &tr->cmd, sizeof(tr->cmd));

	offset = (off_t)cmd - (off_t)qpair->cmd;
	(void) ddi_dma_sync(qpair->ctrlr->dma_handle, offset,
			sizeof(struct nvme_command), DDI_DMA_SYNC_FORDEV); 

	if (++qpair->sq_tail == qpair->num_entries)
		qpair->sq_tail = 0;

	nvme_mmio_write_4(qpair->ctrlr, doorbell[qpair->id].sq_tdbl,
	    qpair->sq_tail);

	membar_producer();

	qpair->num_cmds ++;

	mutex_exit(&qpair->hw_mutex);
}

int
nvme_qpair_submit_request(nvme_tracker_t *tr, int sync)
{
	clock_t deadline;
	int ret = 0;
	
	if (sync == SYNC) {
		/* to avoid races with admin callback */
		mutex_enter(&tr->mutex);

		nvme_map_tracker(tr);
		nvme_qpair_submit_cmd(tr);

		deadline = ddi_get_lbolt() + (clock_t)drv_usectohz(3 * 1000000);
		/* mutex is already held, it is save to cv_timedwait */
		if (cv_timedwait(&tr->cv, &tr->mutex, deadline) < 0) {
			ret = ETIMEDOUT;
		}
		mutex_exit(&tr->mutex);
		nvme_free_tracker(tr);

	} else {
		nvme_map_tracker(tr);
		nvme_qpair_submit_cmd(tr);
	}
	return ret;
}
