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
		cpl = &qpair->cpl[qpair->cq_head];

		if (cpl->p != qpair->phase)
			break;
		printf("cpl::sqhd %d\n", cpl->sqhd);
		printf("cpl::sqid %d\n", cpl->sqid);

		tr = qpair->act_tr[cpl->cid];
		req = tr->req;

		if (req->cb_fn)
			req->cb_fn(req->cb_arg, cpl, req);

		qpair->act_tr[cpl->cid] = NULL;
		qpair->sq_head = cpl->sqhd;


		nvme_free_request(req);

		SLIST_INSERT_HEAD(&qpair->free_tr, tr, slist);

		/* there are pending request w/o tracker. handle one now */
		if (!STAILQ_EMPTY(&qpair->queued_req))
		{
			req = STAILQ_FIRST(&qpair->queued_req);
			STAILQ_REMOVE_HEAD(&qpair->queued_req, stailq);
			nvme_qpair_submit_request(qpair, req);
		}

		if (++qpair->cq_head == qpair->num_entries)
		{
			qpair->cq_head = 0;
			qpair->phase = !qpair->phase;
		}

		printf("write into doorbell! id %d\n", qpair->id);
		nvme_mmio_write_4(qpair->ctrlr, doorbell[qpair->id].cq_hdbl,
		    qpair->cq_head);
	}
}

static void
nvme_qpair_msix_handler(void *arg)
{
	struct nvme_qpair *qpair = arg;

	nvme_qpair_process_completions(qpair);
}

void
nvme_qpair_construct(struct nvme_qpair *qpair, uint32_t id,
    uint16_t vector, uint32_t num_entries, uint32_t num_trackers,
    uint32_t max_xfer_size, struct nvme_controller *ctrlr)
{
	struct nvme_tracker	*tr;
	uint32_t		i;
	size_t len;
	ddi_dma_cookie_t  cookie;
	uint_t  cookie_count;

	qpair->id = id;
	qpair->vector = vector;
	qpair->num_entries = num_entries;
#if 0
	/*
	 * Chatham prototype board starts having issues at higher queue
	 *  depths.  So use a conservative estimate here of no more than 64
	 *  outstanding I/O per queue at any one point.
	 */
	if (pci_get_devid(ctrlr->dev) == CHATHAM_PCI_ID)
		num_trackers = min(num_trackers, 64);
#endif
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
#if 0

		qpair->res = bus_alloc_resource_any(ctrlr->dev, SYS_RES_IRQ,
		    &qpair->rid, RF_ACTIVE);

		bus_setup_intr(ctrlr->dev, qpair->res,
		    INTR_TYPE_MISC | INTR_MPSAFE, NULL,
		    nvme_qpair_msix_handler, qpair, &qpair->tag);
#endif
		/* TODO: setup MSI-X here */
	}
	else
		qpair->rid = vector;

	qpair->num_cmds = 0;
	qpair->num_intr_handler_calls = 0;
	qpair->sq_head = qpair->sq_tail = qpair->cq_head = 0;

	/* TODO: how to allocate physically-contig memory in solaris ? */
	(void)ddi_dma_mem_alloc(qpair->ctrlr->dma_handle, qpair->num_entries * sizeof(struct nvme_command), &nvme_dev_attr, IOMEM_DATA_UNCACHED, DDI_DMA_SLEEP, NULL, (char **)&qpair->cmd, &len, &qpair->cmd_dma_acc_handle);

//	qpair->cmd = kmem_zalloc(qpair->num_entries * sizeof(struct nvme_command), KM_SLEEP);

	(void)ddi_dma_mem_alloc(qpair->ctrlr->dma_handle, qpair->num_entries * sizeof(struct nvme_completion), &nvme_dev_attr, IOMEM_DATA_UNCACHED, DDI_DMA_SLEEP, NULL, (char **)&qpair->cpl, &len, &qpair->cpl_dma_acc_handle);

//	qpair->cpl = kmem_zalloc(qpair->num_entries * sizeof(struct nvme_completion), KM_SLEEP);

	
#if 0
	bus_dmamap_create(qpair->dma_tag, 0, &qpair->cmd_dma_map);
	bus_dmamap_create(qpair->dma_tag, 0, &qpair->cpl_dma_map);

	bus_dmamap_load(qpair->dma_tag, qpair->cmd_dma_map,
	    qpair->cmd, qpair->num_entries * sizeof(struct nvme_command),
	    nvme_single_map, &qpair->cmd_bus_addr, 0);
	bus_dmamap_load(qpair->dma_tag, qpair->cpl_dma_map,
	    qpair->cpl, qpair->num_entries * sizeof(struct nvme_completion),
	    nvme_single_map, &qpair->cpl_bus_addr, 0);
#endif
	(void)ddi_dma_addr_bind_handle(qpair->ctrlr->dma_handle, (struct as *)NULL, (caddr_t)qpair->cmd, qpair->num_entries * sizeof(struct nvme_command),
	DDI_DMA_RDWR | DDI_DMA_CONSISTENT, DDI_DMA_DONTWAIT, 0, &cookie,
	&cookie_count);
 
	qpair->cmd_bus_addr = cookie.dmac_laddress;

	(void)ddi_dma_addr_bind_handle(qpair->ctrlr->dma_handle, (struct as *)NULL, (caddr_t)qpair->cpl, qpair->num_entries * sizeof(struct nvme_completion),
	DDI_DMA_RDWR | DDI_DMA_CONSISTENT, DDI_DMA_DONTWAIT, 0, &cookie,
	&cookie_count);

	qpair->cpl_bus_addr = cookie.dmac_laddress;

	qpair->sq_tdbl_off = nvme_mmio_offsetof(doorbell[id].sq_tdbl);
	qpair->cq_hdbl_off = nvme_mmio_offsetof(doorbell[id].cq_hdbl);

	SLIST_INIT(&qpair->free_tr);
	STAILQ_INIT(&qpair->queued_req);

	for (i = 0; i < qpair->num_trackers; i++) {
		tr = kmem_zalloc(sizeof(*tr), KM_NOSLEEP);

		if (tr == NULL) {
			printf("warning: nvme tracker malloc failed\n");
			break;
		}

		nvme_qpair_construct_tracker(qpair, tr, i);
		SLIST_INSERT_HEAD(&qpair->free_tr, tr, slist);
	}

	qpair->act_tr = kmem_zalloc(sizeof(struct nvme_tracker *) * qpair->num_entries, KM_NOSLEEP);
}

static void
nvme_qpair_destroy(struct nvme_qpair *qpair)
{
	struct nvme_tracker *tr;


	if (qpair->act_tr)
		kmem_free(qpair->act_tr, sizeof(* tr));

	while (!SLIST_EMPTY(&qpair->free_tr)) {
		tr = SLIST_FIRST(&qpair->free_tr);
		SLIST_REMOVE_HEAD(&qpair->free_tr, slist);
		kmem_free(tr, sizeof(* tr));
	}
}

void
nvme_admin_qpair_destroy(struct nvme_qpair *qpair)
{

	/*
	 * For NVMe, you don't send delete queue commands for the admin
	 *  queue, so we just need to unload and free the cmd and cpl memory.
	 */
	ddi_dma_mem_free(&qpair->cmd_dma_acc_handle);
#if 0
	bus_dmamap_unload(qpair->dma_tag, qpair->cmd_dma_map);
	bus_dmamap_destroy(qpair->dma_tag, qpair->cmd_dma_map);
#endif
//	kmem_free(qpair->cmd,
//	    qpair->num_entries * sizeof(struct nvme_command));

#if 0
	bus_dmamap_unload(qpair->dma_tag, qpair->cpl_dma_map);
	bus_dmamap_destroy(qpair->dma_tag, qpair->cpl_dma_map);
#endif
//	kmem_free(qpair->cpl,
//	    qpair->num_entries * sizeof(struct nvme_completion));

	ddi_dma_mem_free(&qpair->cpl_dma_acc_handle);
	nvme_qpair_destroy(qpair);
}

static void
nvme_free_cmd_ring(void *arg, const struct nvme_completion *status, struct nvme_request *req)
{
	struct nvme_qpair *qpair;

	qpair = (struct nvme_qpair *)arg;
#if 0
	bus_dmamap_unload(qpair->dma_tag, qpair->cmd_dma_map);
	bus_dmamap_destroy(qpair->dma_tag, qpair->cmd_dma_map);
#endif
	ddi_dma_mem_free(&qpair->cmd_dma_acc_handle);

	qpair->cmd = NULL;
}

static void
nvme_free_cpl_ring(void *arg, const struct nvme_completion *status, struct nvme_request *req)
{
	struct nvme_qpair *qpair;

	qpair = (struct nvme_qpair *)arg;
#if 0
	bus_dmamap_unload(qpair->dma_tag, qpair->cpl_dma_map);
	bus_dmamap_destroy(qpair->dma_tag, qpair->cpl_dma_map);
#endif
	ddi_dma_mem_free(&qpair->cpl_dma_acc_handle);

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

//	mtx_lock(&qpair->lock);

	tr = SLIST_FIRST(&qpair->free_tr);

	if (tr == NULL) {
		/*
		 * No tracker is available.  Put the request on the qpair's
		 *  request queue to be processed when a tracker frees up
		 *  via a command completion.
		 */
		STAILQ_INSERT_TAIL(&qpair->queued_req, req, stailq);
//		goto ret;
		return;
	}

	SLIST_REMOVE_HEAD(&qpair->free_tr, slist);
	tr->req = req;

	if (req->xfer != NULL)
		printf("data transfer is not supported yet!\n");
	else
	{
		if (req->payload_size > 0)
			nvme_payload_map(tr, req->payload, req->payload_size);

		nvme_qpair_submit_cmd(tr->qpair, tr);

	/* dirty check */
	printf("sleep..\n");
	DELAY(1000);
	printf("tr->qpair->ctrlr->cdata.sn[0] 0x%02x [1] 0x%02x\n",
		tr->qpair->ctrlr->cdata.sn[0], tr->qpair->ctrlr->cdata.sn[1]);
		printf("vid 0x%04x\n", tr->qpair->ctrlr->cdata.vid);
	}
	/* TODO: reimplement me! */
#if 0
	if (req->uio == NULL) {
		if (req->payload_size > 0) {
			err = bus_dmamap_load(tr->qpair->dma_tag,
					      tr->payload_dma_map, req->payload,
					      req->payload_size,
					      nvme_payload_map, tr, 0);
			if (err != 0)
				panic("bus_dmamap_load returned non-zero!\n");
		} else
	} else {
		err = bus_dmamap_load_uio(tr->qpair->dma_tag,
					  tr->payload_dma_map, req->uio,
					  nvme_payload_map_uio, tr, 0);
		if (err != 0)
			panic("bus_dmamap_load returned non-zero!\n");
	}

ret:
	mtx_unlock(&qpair->lock);
#endif

}
