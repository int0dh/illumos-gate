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

extern int nvme_qpair_register_interrupt(struct nvme_qpair *q);

static boolean_t
nvme_completion_check_retry(const struct nvme_completion *cpl)
{
	/*
	 * TODO: spec is not clear how commands that are aborted due
	 *  to TLER will be marked.  So for now, it seems
	 *  NAMESPACE_NOT_READY is the only case where we should
	 *  look at the DNR bit.
	 */
	switch (cpl->sf_sct) {
	case NVME_SCT_GENERIC:
		switch (cpl->sf_sc) {
		case NVME_SC_NAMESPACE_NOT_READY:
			if (cpl->sf_dnr)
				return (0);
			else
				return (1);
		case NVME_SC_INVALID_OPCODE:
		case NVME_SC_INVALID_FIELD:
		case NVME_SC_COMMAND_ID_CONFLICT:
		case NVME_SC_DATA_TRANSFER_ERROR:
		case NVME_SC_ABORTED_POWER_LOSS:
		case NVME_SC_INTERNAL_DEVICE_ERROR:
		case NVME_SC_ABORTED_BY_REQUEST:
		case NVME_SC_ABORTED_SQ_DELETION:
		case NVME_SC_ABORTED_FAILED_FUSED:
		case NVME_SC_ABORTED_MISSING_FUSED:
		case NVME_SC_INVALID_NAMESPACE_OR_FORMAT:
		case NVME_SC_COMMAND_SEQUENCE_ERROR:
		case NVME_SC_LBA_OUT_OF_RANGE:
		case NVME_SC_CAPACITY_EXCEEDED:
		default:
			return (0);
		}
	case NVME_SCT_COMMAND_SPECIFIC:
	case NVME_SCT_MEDIA_ERROR:
	case NVME_SCT_VENDOR_SPECIFIC:
	default:
		return (0);
	}
}

void
nvme_qpair_process_completions(struct nvme_qpair *qpair)
{
	struct nvme_tracker	*tr;
	struct nvme_completion	*cpl;
	boolean_t		retry, error;
	int i;

	qpair->num_intr_handler_calls ++;

	for (;;)
	{

		mutex_enter(&qpair->hw_mutex);

		cpl = &qpair->cpl[qpair->cq_head];

		if (cpl->p != qpair->phase)
		{
			mutex_exit(&qpair->hw_mutex);
			break;
		}
		ASSERT(cpl->cid > qpair->num_entries);

		tr = qpair->act_tr[cpl->cid];

		ASSERT(tr == NULL);

		mutex_exit(&qpair->hw_mutex);
		if (tr->cb_fn)
			tr->cb_fn(tr->cb_arg, cpl, tr);
		mutex_enter(&qpair->hw_mutex);

		qpair->sq_head = cpl->sqhd;

		if (++ qpair->cq_head == qpair->num_entries)
		{
			qpair->cq_head = 0;
			qpair->phase = !qpair->phase;
		}
		nvme_mmio_write_4(qpair->ctrlr, doorbell[qpair->id].cq_hdbl,
		    qpair->cq_head);

		mutex_exit(&qpair->hw_mutex);
	}
}

int
nvme_qpair_construct(struct nvme_qpair *qpair, uint32_t id,
    uint16_t vector, uint32_t num_entries, uint32_t num_trackers,
    uint32_t max_xfer_size, struct nvme_controller *ctrlr)
{
	struct nvme_tracker	*tr;
	uint32_t		i;
	size_t len;
	ddi_dma_cookie_t  cookie;
	uint_t  cookie_count;
	struct nvme_request *rq;
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

	if (ctrlr->msix_enabled)
	{

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
			qpair->num_entries * sizeof(struct nvme_command), 
			qpair->ctrlr->devattr, IOMEM_DATA_UNCACHED,
			DDI_DMA_SLEEP, NULL, (char **)&qpair->cmd, &len, 
			&qpair->cmd_dma_acc_handle);

	if (res != DDI_SUCCESS)
		return (ENOMEM);
	/* is the dma_mem_alloc()ed memory zeroed? */
	memset(qpair->cmd, 0, qpair->num_entries * sizeof(struct nvme_command));

	res = ddi_dma_mem_alloc(qpair->ctrlr->dma_handle, 
			qpair->num_entries * sizeof(struct nvme_completion), 
			qpair->ctrlr->devattr, IOMEM_DATA_UNCACHED,
			DDI_DMA_SLEEP, NULL, (char **)&qpair->cpl, &len, 
			&qpair->cpl_dma_acc_handle);

	if (res != DDI_SUCCESS)
	{
		(void)ddi_dma_mem_free(&qpair->cmd_dma_acc_handle);
		return (ENOMEM);
	}
	/* is the memory zeroed per default ? */
	memset(qpair->cpl, 0, qpair->num_entries * sizeof(struct nvme_completion));
	res = ddi_dma_addr_bind_handle(qpair->ctrlr->dma_handle, 
			(struct as *)NULL, (caddr_t)qpair->cmd, 
			qpair->num_entries * sizeof(struct nvme_command),
			DDI_DMA_RDWR | DDI_DMA_CONSISTENT, DDI_DMA_DONTWAIT,
			0, &cookie, &cookie_count);
 
	switch (res)
	{
		case DDI_DMA_MAPPED:
		/* everything is ok */
			break;
		/* TODO: update to count some statistic based on the bind_handle() result */
		default:
			(void)ddi_dma_mem_free(&qpair->cmd_dma_acc_handle);
			(void)ddi_dma_mem_free(&qpair->cpl_dma_acc_handle);
			return (ENOMEM);
	}
	ASSERT(cookie_count != 1);

	qpair->cmd_bus_addr = cookie.dmac_laddress;

	res = ddi_dma_addr_bind_handle(qpair->ctrlr->dma_handle, 
			(struct as *)NULL, (caddr_t)qpair->cpl, 
			qpair->num_entries * sizeof(struct nvme_completion),
			DDI_DMA_RDWR | DDI_DMA_CONSISTENT, DDI_DMA_DONTWAIT,
			0, &cookie, &cookie_count);

	switch (res)
	{
		case DDI_DMA_MAPPED:
		/* everything is ok */
			break;
		default:
			(void)ddi_dma_mem_free(&qpair->cmd_dma_acc_handle);
			(void)ddi_dma_mem_free(&qpair->cpl_dma_acc_handle);
			return ENOMEM;
	}
	ASSERT(cookie_count != 1);

	qpair->cpl_bus_addr = cookie.dmac_laddress;

	qpair->sq_tdbl_off = nvme_mmio_offsetof(doorbell[id].sq_tdbl);
	qpair->cq_hdbl_off = nvme_mmio_offsetof(doorbell[id].cq_hdbl);

	qpair->trackers = kmem_zalloc(sizeof(struct nvme_tracker) * qpair->num_entries, KM_SLEEP);

	TAILQ_INIT(&qpair->free_trackers);
	for (i = 0; i < qpair->num_entries; i ++)
	{
		struct nvme_tracker *t = &qpair->trackers[i];
		t->qpair = qpair;
		t->cid = i;
		TAILQ_INSERT_TAIL(&qpair->free_trackers, t, next);
	}
	qpair->act_tr = kmem_zalloc(sizeof(struct nvme_tracker *) * qpair->num_entries, KM_SLEEP); 
	nvme_qpair_register_interrupt(qpair);

	mutex_init(&qpair->hw_mutex, NULL, MUTEX_DRIVER, 
				DDI_INTR_PRI(qpair->soft_intr_pri));
	mutex_init(&qpair->free_trackers_mutex, NULL, MUTEX_DRIVER, 
				DDI_INTR_PRI(qpair->soft_intr_pri));
	return 0;
}

static void
nvme_qpair_destroy(struct nvme_qpair *qpair)
{
	struct nvme_tracker *tr;


	if (qpair->act_tr)
	{
		/* release all active trackers */
		int i;
		for (i = 0; i < qpair->num_entries; i ++)
		{
			tr = qpair->act_tr[i];
			if (tr)
			{
				mutex_destroy(&tr->mutex);
				cv_destroy(&tr->cv);
			}
		}
		kmem_free(qpair->act_tr, sizeof(tr) * qpair->num_entries);
		qpair->act_tr = NULL;
	}
	kmem_free(qpair->trackers, sizeof(struct nvme_tracker) * qpair->num_entries);	
	qpair->trackers = NULL;
	mutex_destroy(&qpair->hw_mutex);
	mutex_destroy(&qpair->free_trackers_mutex);
}

void
nvme_admin_qpair_destroy(struct nvme_qpair *qpair)
{

	/*
	 * For NVMe, you don't send delete queue commands for the admin
	 *  queue, so we just need to unload and free the cmd and cpl memory.
	 */
	(void)ddi_dma_mem_free(&qpair->cmd_dma_acc_handle);
	(void)ddi_dma_mem_free(&qpair->cpl_dma_acc_handle);

	(void)ddi_dma_unbind_handle(qpair->ctrlr->dma_handle);

	nvme_qpair_destroy(qpair);
}

static void
nvme_free_cmd_ring(void *arg, const struct nvme_completion *status, struct nvme_tracker *tr)
{
	struct nvme_qpair *qpair = (struct nvme_qpair *)arg;

	(void)ddi_dma_mem_free(&qpair->cmd_dma_acc_handle);
	qpair->cmd = NULL;
}

static void
nvme_free_cpl_ring(void *arg, const struct nvme_completion *status, struct nvme_tracker *tr)
{
	struct nvme_qpair *qpair = (struct nvme_qpair *)arg;

	(void)ddi_dma_mem_free(&qpair->cpl_dma_acc_handle);
	qpair->cpl = NULL;
}

void
nvme_io_qpair_destroy(struct nvme_qpair *qpair)
{
	struct nvme_controller *ctrlr = qpair->ctrlr;

	if (qpair->num_entries > 0)
	{

		nvme_ctrlr_cmd_delete_io_sq(ctrlr, qpair, nvme_free_cmd_ring,
		    qpair);
		/* Spin until free_cmd_ring sets qpair->cmd to NULL. */
		while (qpair->cmd)
			DELAY(5);

		nvme_ctrlr_cmd_delete_io_cq(ctrlr, qpair, nvme_free_cpl_ring,
		    qpair);
		/* Spin until free_cpl_ring sets qpair->cmd to NULL. */
		while (qpair->cpl)
			DELAY(5);

		nvme_qpair_destroy(qpair);
	}
}

static void
nvme_timeout(void *arg)
{
	/*
	 * TODO: Add explicit abort operation here, once nvme(4) supports
	 *  abort commands.
	 */
}

void
nvme_qpair_submit_cmd(struct nvme_qpair *qpair, struct nvme_tracker *tr)
{

	mutex_enter(&qpair->hw_mutex);

	qpair->act_tr[tr->cid] = tr;

	tr->cmd.cid = tr->cid;
//	printf("%s: cmd.cid is %d\n", __func__, tr->cid);
//	callout_reset(&tr->timer, NVME_TIMEOUT_IN_SEC * hz, nvme_timeout, tr);

	/* Copy the command from the tracker to the submission queue. */
	memcpy(&qpair->cmd[qpair->sq_tail], &tr->cmd, sizeof(tr->cmd));

	if (++qpair->sq_tail == qpair->num_entries)
		qpair->sq_tail = 0;

	wmb();
	nvme_mmio_write_4(qpair->ctrlr, doorbell[qpair->id].sq_tdbl,
	    qpair->sq_tail);

	qpair->num_cmds++;

	mutex_exit(&qpair->hw_mutex);
}

void
nvme_qpair_submit_request(struct nvme_qpair *qpair, struct nvme_tracker *tr)
{
	int			err;
	ddi_dma_handle_t	dmah;
	ddi_dma_cookie_t        *dmac = NULL;
	int dmac_size = 0;

	if (tr->payload_size > 0)
	{
		if (tr->xfer)
		{
			/* not DMA-mapped xfers are not supported yet */
			ASSERT(tr->xfer->x_ndmac == 0);
			/* xfer is already mapped and ready with cookies */
			dmac = &tr->xfer->x_dmac;		
			dmac_size = tr->xfer->x_ndmac;
		}
		else
			dmah =  tr->qpair->ctrlr->dma_handle;

		nvme_payload_map(tr, dmah, dmac, dmac_size,
				tr->payload, tr->payload_size);
	}
	nvme_qpair_submit_cmd(tr->qpair, tr);
}
