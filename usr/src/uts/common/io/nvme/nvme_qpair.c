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

extern ddi_device_acc_attr_t nvme_dev_attr;

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

static void
nvme_qpair_construct_tracker(struct nvme_qpair *qpair, struct nvme_tracker *tr,
    uint16_t cid)
{

#if 0
	bus_dmamap_create(qpair->dma_tag, 0, &tr->payload_dma_map);
	bus_dmamap_create(qpair->dma_tag, 0, &tr->prp_dma_map);

	bus_dmamap_load(qpair->dma_tag, tr->prp_dma_map, tr->prp,
	    sizeof(tr->prp), nvme_single_map, &tr->prp_bus_addr, 0);

	callout_init_mtx(&tr->timer, &qpair->lock, 0);
#endif
	tr->cid = cid;
	tr->qpair = qpair;
}
void
nvme_qpair_process_completions(struct nvme_qpair *qpair)
{
	struct nvme_tracker	*tr;
	struct nvme_request	*req;
	struct nvme_completion	*cpl;
	boolean_t		retry, error;
	int i;

	qpair->num_intr_handler_calls++;

	for (i = 0; i < qpair->num_entries; i ++) 
	{
//		cpl = &qpair->cpl[i];
		cpl = &qpair->cpl[qpair->cq_head];

		if (cpl->p != qpair->phase)
			break;

		if (cpl->cid > NVME_ADMIN_ENTRIES)
		{
			printf("bad CID!! %d cpl at 0x%p\n", cpl->cid, cpl);
			kmdb_enter();
			continue;
		}
		tr = qpair->act_tr[cpl->cid];
		if (tr == NULL)
		{
			continue;
		}
		req = tr->req;

		printf("soft interrupt: callback at 0x%p\n", req->cb_fn);
		if (req->cb_fn)
			req->cb_fn(req->cb_arg, cpl, req);

		qpair->act_tr[cpl->cid] = NULL;
		qpair->sq_head = cpl->sqhd;


//		printf("free request..\n");
//		nvme_free_request(qpair, req);

		/* release the tracker */
		SLIST_INSERT_HEAD(&qpair->free_tr, tr, slist);

		/* there are pending request w/o tracker. handle one now */
#if 0
		if (!STAILQ_EMPTY(&qpair->queued_req))
		{
			req = STAILQ_FIRST(&qpair->queued_req);
			STAILQ_REMOVE_HEAD(&qpair->queued_req, stailq);
			nvme_qpair_submit_request(qpair, req);
		}
#endif
		if (++qpair->cq_head == qpair->num_entries)
		{
			qpair->cq_head = 0;
			qpair->phase = !qpair->phase;
		}
		nvme_mmio_write_4(qpair->ctrlr, doorbell[qpair->id].cq_hdbl,
		    qpair->cq_head);
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

	res = ddi_dma_mem_alloc(qpair->ctrlr->dma_handle, qpair->num_entries * sizeof(struct nvme_command), &nvme_dev_attr, IOMEM_DATA_UNCACHED, DDI_DMA_SLEEP, NULL, (char **)&qpair->cmd, &len, &qpair->cmd_dma_acc_handle);

	if (res != DDI_SUCCESS)
		return (ENOMEM);
	/* is the dma_mem_alloc()ed memory zeroed? */
	memset(qpair->cmd, 0, qpair->num_entries * sizeof(struct nvme_command));

	res = ddi_dma_mem_alloc(qpair->ctrlr->dma_handle, qpair->num_entries * sizeof(struct nvme_completion), &nvme_dev_attr, IOMEM_DATA_UNCACHED, DDI_DMA_SLEEP, NULL, (char **)&qpair->cpl, &len, &qpair->cpl_dma_acc_handle);

	if (res != DDI_SUCCESS)
	{
		(void)ddi_dma_mem_free(&qpair->cmd_dma_acc_handle);
		return (ENOMEM);
	}
	/* is the memory zeroed per default ? */
	memset(qpair->cpl, 0, qpair->num_entries * sizeof(struct nvme_completion));
	res = ddi_dma_addr_bind_handle(qpair->ctrlr->dma_handle, (struct as *)NULL, (caddr_t)qpair->cmd, qpair->num_entries * sizeof(struct nvme_command),
	DDI_DMA_RDWR | DDI_DMA_CONSISTENT, DDI_DMA_DONTWAIT, 0, &cookie,
	&cookie_count);
 
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
	if (cookie_count != 1)
		panic("invalid cookie count 1");

	qpair->cmd_bus_addr = cookie.dmac_laddress;

	res = ddi_dma_addr_bind_handle(qpair->ctrlr->dma_handle, (struct as *)NULL, (caddr_t)qpair->cpl, qpair->num_entries * sizeof(struct nvme_completion),
	DDI_DMA_RDWR | DDI_DMA_CONSISTENT, DDI_DMA_DONTWAIT, 0, &cookie,
	&cookie_count);

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
	if (cookie_count != 1)
		panic("invalid cookie count 2");

	qpair->cpl_bus_addr = cookie.dmac_laddress;

	qpair->sq_tdbl_off = nvme_mmio_offsetof(doorbell[id].sq_tdbl);
	qpair->cq_hdbl_off = nvme_mmio_offsetof(doorbell[id].cq_hdbl);

	SLIST_INIT(&qpair->free_tr);
	STAILQ_INIT(&qpair->queued_req);

	tr = kmem_zalloc(sizeof(*tr) * qpair->num_trackers, KM_SLEEP);

	qpair->free_tr_mem = tr;

	for (i = 0; i < qpair->num_trackers; i++)
	{
		struct nvme_tracker *t = &tr[i];
		nvme_qpair_construct_tracker(qpair, tr, i);
		SLIST_INSERT_HEAD(&qpair->free_tr, tr, slist);
	}

	qpair->act_tr = kmem_zalloc(sizeof(struct nvme_tracker *) * qpair->num_entries, KM_SLEEP);

	rq = kmem_zalloc(sizeof(struct nvme_request) * qpair->num_entries, KM_NOSLEEP);
	TAILQ_INIT(&qpair->request_queue);
	for (i = 0; i < qpair->num_entries; i ++)
	{
		struct nvme_request *r = &rq[i];
		TAILQ_INSERT_TAIL(&qpair->request_queue, r, rq_next);
	}	
	return 0;
}

static void
nvme_qpair_destroy(struct nvme_qpair *qpair)
{
	struct nvme_tracker *tr;


	if (qpair->act_tr)
	{
		kmem_free(qpair->act_tr, sizeof(tr) * qpair->num_entries);
		qpair->act_tr = NULL;
	}
	kmem_free(qpair->free_tr_mem, sizeof(* tr) * qpair->num_entries);
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
nvme_free_cmd_ring(void *arg, const struct nvme_completion *status, struct nvme_request *req)
{
	struct nvme_qpair *qpair = (struct nvme_qpair *)arg;

	(void)ddi_dma_mem_free(&qpair->cmd_dma_acc_handle);
	qpair->cmd = NULL;
}

static void
nvme_free_cpl_ring(void *arg, const struct nvme_completion *status, struct nvme_request *req)
{
	struct nvme_qpair *qpair = (struct nvme_qpair *)arg;

	(void)ddi_dma_mem_free(&qpair->cpl_dma_acc_handle);
	qpair->cpl = NULL;
}

void
nvme_io_qpair_destroy(struct nvme_qpair *qpair)
{
	struct nvme_controller *ctrlr = qpair->ctrlr;

	if (qpair->num_entries > 0) {

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
	struct nvme_request *req;

	req = tr->req;
	req->cmd.cid = tr->cid;
	qpair->act_tr[tr->cid] = tr;

	printf("%s: cmd.cid is %d\n", __func__, tr->cid);
//	callout_reset(&tr->timer, NVME_TIMEOUT_IN_SEC * hz, nvme_timeout, tr);

	/* Copy the command from the tracker to the submission queue. */
	memcpy(&qpair->cmd[qpair->sq_tail], &req->cmd, sizeof(req->cmd));

	if (++qpair->sq_tail == qpair->num_entries)
		qpair->sq_tail = 0;

	wmb();
	nvme_mmio_write_4(qpair->ctrlr, doorbell[qpair->id].sq_tdbl,
	    qpair->sq_tail);

	qpair->num_cmds++;
}

void
nvme_qpair_submit_request(struct nvme_qpair *qpair, struct nvme_request *req)
{
	struct nvme_tracker	*tr;
	int			err;
	ddi_dma_handle_t	dmah;
	ddi_dma_cookie_t        *dmac = NULL;
	int dmac_size = 0;
//	mtx_lock(&qpair->lock);

	lock_set(&qpair->lock);

	tr = SLIST_FIRST(&qpair->free_tr);

	if (tr == NULL) {
		/*
		 * No tracker is available.  Put the request on the qpair's
		 *  request queue to be processed when a tracker frees up
		 *  via a command completion.
		 */
		STAILQ_INSERT_TAIL(&qpair->queued_req, req, stailq);
		lock_clear(&qpair->lock);
		panic("no available tracker!\n");
	}

	SLIST_REMOVE_HEAD(&qpair->free_tr, slist);
	tr->req = req;
	if (req->payload_size > 0)
	{
		if (req->xfer)
		{
			/* xfer is already mapped and ready with cookies */
			if (req->xfer->x_ndmac)
			{
				dmac = &req->xfer->x_dmac;		
				dmac_size = req->xfer->x_ndmac;
			}
			else
				panic("xfer is not DMA mapped!");
		}
		else
			dmah =  tr->qpair->ctrlr->dma_handle;

		nvme_payload_map(tr, dmah, dmac, dmac_size, req->payload, req->payload_size);
	}
	nvme_qpair_submit_cmd(tr->qpair, tr);
	lock_clear(&qpair->lock);
}
