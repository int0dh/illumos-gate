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

static void
nvme_ctrlr_cb(void *arg, const struct nvme_completion *status, struct nvme_request *req)
{
	struct nvme_completion	*cpl = arg;

	mutex_enter(&req->mutex);

	/*
	 * Copy status into the argument passed by the caller, so that
	 *  the caller can check the status to determine if the
	 *  the request passed or failed.
	 */
	memcpy(cpl, status, sizeof(*cpl));

	cv_broadcast(&req->cv);

	mutex_exit(&req->mutex);
}

static void
nvme_ctrlr_construct_admin_qpair(struct nvme_controller *ctrlr)
{
	struct nvme_qpair	*qpair;
	uint32_t		num_entries;

	qpair = &ctrlr->adminq;

	num_entries = NVME_ADMIN_ENTRIES;
	/*
	 * If admin_entries was overridden to an invalid value, revert it
	 *  back to our default value.
	 */
	if (num_entries < NVME_MIN_ADMIN_ENTRIES ||
	    num_entries > NVME_MAX_ADMIN_ENTRIES) {
		printf("nvme: invalid hw.nvme.admin_entries=%d specified\n",
		    num_entries);
		num_entries = NVME_ADMIN_ENTRIES;
	}

	/*
	 * The admin queue's max xfer size is treated differently than the
	 *  max I/O xfer size.  16KB is sufficient here - maybe even less?
	 */
	nvme_qpair_construct(qpair, 
			     0, /* qpair ID */
			     0, /* vector */
			     num_entries,
			     NVME_ADMIN_TRACKERS,
			     16*1024, /* max xfer size */
			     ctrlr);
}

static int
nvme_ctrlr_construct_io_qpairs(struct nvme_controller *ctrlr)
{
	struct nvme_qpair	*qpair;
	union cap_lo_register	cap_lo;
	int			i, num_entries, num_trackers;

	num_entries = NVME_IO_ENTRIES;

	/*
	 * NVMe spec sets a hard limit of 64K max entries, but
	 *  devices may specify a smaller limit, so we need to check
	 *  the MQES field in the capabilities register.
	 */
	cap_lo.raw = nvme_mmio_read_4(ctrlr, cap_lo);
	num_entries = min(num_entries, cap_lo.bits.mqes+1);

	num_trackers = NVME_IO_TRACKERS;

	num_trackers = max(num_trackers, NVME_MIN_IO_TRACKERS);
	num_trackers = min(num_trackers, NVME_MAX_IO_TRACKERS);
	/*
	 * No need to have more trackers than entries in the submit queue.
	 *  Note also that for a queue size of N, we can only have (N-1)
	 *  commands outstanding, hence the "-1" here.
	 */
	num_trackers = min(num_trackers, (num_entries-1));

	ctrlr->max_xfer_size = NVME_MAX_XFER_SIZE;
	/*
	 * Check that tunable doesn't specify a size greater than what our
	 *  driver supports, and is an even PAGE_SIZE multiple.
	 */
	if (ctrlr->max_xfer_size > NVME_MAX_XFER_SIZE ||
	    ctrlr->max_xfer_size % PAGESIZE)
		ctrlr->max_xfer_size = NVME_MAX_XFER_SIZE;

	ctrlr->ioq = kmem_zalloc(ctrlr->num_io_queues * sizeof(struct nvme_qpair), KM_SLEEP);

	if (ctrlr->ioq == NULL)
		return (ENOMEM);

	for (i = 0; i < ctrlr->num_io_queues; i++) {
		qpair = &ctrlr->ioq[i];

		/*
		 * Admin queue has ID=0. IO queues start at ID=1 -
		 *  hence the 'i+1' here.
		 *
		 * For I/O queues, use the controller-wide max_xfer_size
		 *  calculated in nvme_attach().
		 */
		nvme_qpair_construct(qpair,
				     i+1, /* qpair ID */
				     ctrlr->msix_enabled ? i+1 : 0, /* vector */
				     num_entries,
				     num_trackers,
				     ctrlr->max_xfer_size,
				     ctrlr);

/* INVESTIGATE ME */
#if 0
		if (ctrlr->per_cpu_io_queues)
			bus_bind_intr(ctrlr->dev, qpair->res, i);
#endif
	}

	return (0);
}

static int
nvme_ctrlr_wait_for_ready(struct nvme_controller *ctrlr)
{
	int ms_waited;
	union cc_register cc;
	union csts_register csts;

	cc.raw = nvme_mmio_read_4(ctrlr, cc);
	csts.raw = nvme_mmio_read_4(ctrlr, csts);

	if (!cc.bits.en) {
		printf("%s called with cc.en = 0\n", __func__);
		return (ENXIO);
	}

	ms_waited = 0;

	while (!csts.bits.rdy) {
		DELAY(1000);
		if (ms_waited++ > ctrlr->ready_timeout_in_ms) {
			printf("controller did not become "
			    "ready within %d ms\n", ctrlr->ready_timeout_in_ms);
			return (ENXIO);
		}
		csts.raw = nvme_mmio_read_4(ctrlr, csts);
	}

	return (0);
}

static void
nvme_ctrlr_disable(struct nvme_controller *ctrlr)
{
	union cc_register cc;
	union csts_register csts;

	cc.raw = nvme_mmio_read_4(ctrlr, cc);
	csts.raw = nvme_mmio_read_4(ctrlr, csts);

	if (cc.bits.en == 1 && csts.bits.rdy == 0)
		nvme_ctrlr_wait_for_ready(ctrlr);

	cc.bits.en = 0;
	nvme_mmio_write_4(ctrlr, cc, cc.raw);
	DELAY(5000);
}

static int
nvme_ctrlr_enable(struct nvme_controller *ctrlr)
{
	union cc_register	cc;
	union csts_register	csts;
	union aqa_register	aqa;

	cc.raw = nvme_mmio_read_4(ctrlr, cc);
	csts.raw = nvme_mmio_read_4(ctrlr, csts);

	if (cc.bits.en == 1) {
		if (csts.bits.rdy == 1)
			return (0);
		else
			return (nvme_ctrlr_wait_for_ready(ctrlr));
	}

	printf("%s: asq val %lld\n", __FUNCTION__, (long long int)ctrlr->adminq.cmd_bus_addr);
	nvme_mmio_write_8(ctrlr, asq, ctrlr->adminq.cmd_bus_addr);
	DELAY(5000);
	printf("%s: acq val %lld\n", __FUNCTION__, (long long int)ctrlr->adminq.cpl_bus_addr);
	nvme_mmio_write_8(ctrlr, acq, ctrlr->adminq.cpl_bus_addr);
	DELAY(5000);

	aqa.raw = 0;
	/* acqs and asqs are 0-based. */
	aqa.bits.acqs = ctrlr->adminq.num_entries-1;
	aqa.bits.asqs = ctrlr->adminq.num_entries-1;
	printf("%s: aqa 0x%08x\n", __FUNCTION__, aqa.raw);
	nvme_mmio_write_4(ctrlr, aqa, aqa.raw);
	DELAY(5000);

	cc.bits.en = 1;
	cc.bits.css = 0;
	cc.bits.ams = 0;
	cc.bits.shn = 0;
	cc.bits.iosqes = 6; /* SQ entry size == 64 == 2^6 */
	cc.bits.iocqes = 4; /* CQ entry size == 16 == 2^4 */

	/* This evaluates to 0, which is according to spec. */
	cc.bits.mps = (PAGESIZE >> 13);

	printf("%s: write into CC, val 0x%08x\n", __FUNCTION__, cc.raw);
	nvme_mmio_write_4(ctrlr, cc, cc.raw);
	DELAY(5000);

	return (nvme_ctrlr_wait_for_ready(ctrlr));
}

int
nvme_ctrlr_reset(struct nvme_controller *ctrlr)
{

	nvme_ctrlr_disable(ctrlr);
	return (nvme_ctrlr_enable(ctrlr));
}

/*
 * Disable this code for now, since Chatham doesn't support
 *  AERs so I have no good way to test them.
 */
#if 0
static void
nvme_async_event_cb(void *arg, const struct nvme_completion *status)
{
	struct nvme_controller *ctrlr = arg;

	printf("Asynchronous event occurred.\n");

	/* TODO: decode async event type based on status */
	/* TODO: check status for any error bits */

	/*
	 * Repost an asynchronous event request so that it can be
	 *  used again by the controller.
	 */
	nvme_ctrlr_cmd_asynchronous_event_request(ctrlr, nvme_async_event_cb,
	    ctrlr);
}
#endif

static int
nvme_ctrlr_identify(struct nvme_controller *ctrlr)
{
	struct nvme_completion	cpl;
	int ret;

	nvme_ctrlr_cmd_identify_controller(ctrlr, &ctrlr->cdata,
	    nvme_ctrlr_cb, &cpl);

	if (cpl.sf_sc || cpl.sf_sct) {
		printf("nvme_identify_controller failed!\n");
		return (ENXIO);
	}
	printf("nvme_identify_controller success!\n");
#if 0
#ifdef CHATHAM2
	if (pci_get_devid(ctrlr->dev) == CHATHAM_PCI_ID)
		nvme_chatham_populate_cdata(ctrlr);
#endif
#endif
	return (0);
}

static int
nvme_ctrlr_set_num_qpairs(struct nvme_controller *ctrlr)
{
	struct nvme_completion	cpl;
	int			cq_allocated, sq_allocated, status;

	nvme_ctrlr_cmd_set_num_queues(ctrlr, ctrlr->num_io_queues,
	    nvme_ctrlr_cb, &cpl);

	if ((status != 0) || cpl.sf_sc || cpl.sf_sct) {
		printf("nvme_set_num_queues failed!\n");
		return (ENXIO);
	}

	/*
	 * Data in cdw0 is 0-based.
	 * Lower 16-bits indicate number of submission queues allocated.
	 * Upper 16-bits indicate number of completion queues allocated.
	 */
	sq_allocated = (cpl.cdw0 & 0xFFFF) + 1;
	cq_allocated = (cpl.cdw0 >> 16) + 1;

	/*
	 * Check that the controller was able to allocate the number of
	 *  queues we requested.  If not, revert to one IO queue.
	 */
	if (sq_allocated < ctrlr->num_io_queues ||
	    cq_allocated < ctrlr->num_io_queues) {
		ctrlr->num_io_queues = 1;
		ctrlr->per_cpu_io_queues = 0;

		/* TODO: destroy extra queues that were created
		 *  previously but now found to be not needed.
		 */
	}

	return (0);
}

static int
nvme_ctrlr_create_qpairs(struct nvme_controller *ctrlr)
{
	struct nvme_qpair	*qpair;
	struct nvme_completion	cpl;
	int			i, status;

	for (i = 0; i < ctrlr->num_io_queues; i++) {
		qpair = &ctrlr->ioq[i];

		nvme_ctrlr_cmd_create_io_cq(ctrlr, qpair, qpair->vector,
		    nvme_ctrlr_cb, &cpl);

		if ((status != 0) || cpl.sf_sc || cpl.sf_sct) {
			printf("nvme_create_io_cq failed!\n");
			return (ENXIO);
		}

		nvme_ctrlr_cmd_create_io_sq(qpair->ctrlr, qpair,
		    nvme_ctrlr_cb, &cpl);

		if ((status != 0) || cpl.sf_sc || cpl.sf_sct) {
			printf("nvme_create_io_sq failed!\n");
			return (ENXIO);
		}
	}

	return (0);
}

static int
nvme_ctrlr_construct_namespaces(struct nvme_controller *ctrlr)
{
	struct nvme_namespace	*ns;
	int			i, status;

	for (i = 0; i < ctrlr->cdata.nn; i++) {
		ns = &ctrlr->ns[i];
		status = nvme_ns_construct(ns, i+1, ctrlr);
		if (status != 0)
			return (status);
	}

	return (0);
}

static void
nvme_ctrlr_configure_aer(struct nvme_controller *ctrlr)
{
	union nvme_critical_warning_state	state;
	uint8_t					num_async_events;

	state.raw = 0xFF;
	state.bits.reserved = 0;
	nvme_ctrlr_cmd_set_asynchronous_event_config(ctrlr, state, NULL, NULL);

	/* aerl is a zero-based value, so we need to add 1 here. */
	num_async_events = min(NVME_MAX_ASYNC_EVENTS, (ctrlr->cdata.aerl+1));

	/*
	 * Disable this code for now, since Chatham doesn't support
	 *  AERs so I have no good way to test them.
	 */
#if 0
	for (int i = 0; i < num_async_events; i++)
		nvme_ctrlr_cmd_asynchronous_event_request(ctrlr,
		    nvme_async_event_cb, ctrlr);
#endif
}

static void
nvme_ctrlr_configure_int_coalescing(struct nvme_controller *ctrlr)
{

	/* todo: do something nice here */
}

void
nvme_ctrlr_start(void *ctrlr_arg)
{
	struct nvme_controller *ctrlr = ctrlr_arg;

	nvme_interrupt_enable(ctrlr);

	if (nvme_ctrlr_identify(ctrlr) != 0)
		return;


	if (nvme_ctrlr_set_num_qpairs(ctrlr) != 0)
		return;

	if (nvme_ctrlr_create_qpairs(ctrlr) != 0)
		return;

	if (nvme_ctrlr_construct_namespaces(ctrlr) != 0)
		return;

	nvme_ctrlr_configure_aer(ctrlr);
	nvme_ctrlr_configure_int_coalescing(ctrlr);

	ctrlr->is_started = B_TRUE;
}

static void
nvme_ctrlr_intx_task(void *arg, int pending)
{
	struct nvme_controller *ctrlr = arg;

	nvme_qpair_process_completions(&ctrlr->adminq);

	if (ctrlr->ioq[0].cpl)
		nvme_qpair_process_completions(&ctrlr->ioq[0]);

	nvme_mmio_write_4(ctrlr, intmc, 1);
}

uint_t 
nvme_ctrlr_intx_handler(char *arg, char *unused)
{
	struct nvme_controller *ctrlr = (struct nvme_controller *)arg;

        printf("nvme interrupt\n");
	nvme_ctrlr_intx_task(ctrlr, 0);

	nvme_mmio_write_4(ctrlr, intms, 1);

	return DDI_INTR_CLAIMED;
}

void
nvme_interrupt_enable(struct nvme_controller *nvme)
{
	printf("ddi_intr_enable interrupt!\n");
	ddi_intr_enable(nvme->intr_handle[0]);
}

void
nvme_interrupt_disable(struct nvme_controller *nvme)
{
	printf("disable NVMe interrupt\n");
	ddi_intr_disable(nvme->intr_handle[0]);
}

static int 
nvme_register_interrupt(struct nvme_controller *nvme, int intr_type)
{
	int count, actual;
	int ret;
	int avail;
	int i;

	ret = ddi_intr_get_nintrs(nvme->devinfo, intr_type, &count);

	if (ret != DDI_SUCCESS)
	{
		printf("ddi_intr_get_nintrs failure. ret %d ._., count %d\n", ret, count);
		return DDI_FAILURE;
	}

	ret = ddi_intr_get_navail(nvme->devinfo, intr_type, &avail);
	if (ret != DDI_SUCCESS)
	{
		printf("ddi_intr_get_navail failure, ret %d\n", ret);
		return DDI_FAILURE;
	}	
	nvme->intr_size = count * sizeof(ddi_intr_handle_t);
	nvme->intr_handle = kmem_alloc(nvme->intr_size, KM_SLEEP);
	
	ret = ddi_intr_alloc(nvme->devinfo, nvme->intr_handle, intr_type, 0, count, &actual, DDI_INTR_ALLOC_NORMAL);

	if (ret != DDI_SUCCESS)
		return ret;

	printf("count = %d actual = %d\n", count, actual);

	for (i = 0; i < count; i ++)
	{
		ret = ddi_intr_add_handler(nvme->intr_handle[i], nvme_ctrlr_intx_handler, (caddr_t)nvme, NULL);
		if (ret != DDI_SUCCESS)
		{
			printf("cannot register interrupt handler!, ret %d\n", ret);
			return DDI_FAILURE;
		}
	}
	printf("%d interrupt handlers registered ok!\n", count); 
	return DDI_SUCCESS;
}
/*FIXME: check status for each DDI function */
static int
nvme_ctrlr_configure_intx(struct nvme_controller *nvme)
{
	int32_t intr_type;

	if (ddi_intr_get_supported_types(nvme->devinfo, &intr_type) != DDI_SUCCESS)
	{
		printf("cannot get supported interrupt type :(\n");
		return -1;
	}

	if (intr_type & (DDI_INTR_TYPE_MSI | DDI_INTR_TYPE_MSIX))
	{
		printf("MSI interrupts are supported!\n");
		nvme->msix_enabled = 1;
		intr_type &= ~DDI_INTR_TYPE_FIXED;
	}
	nvme_register_interrupt(nvme, intr_type);
	
	/* allocate one queue par per available interrupt */
	/* TODO: do not allocate more than number of CPUs */
	/* otherwise, some performance degradation may occur */
	nvme->num_io_queues = nvme->intr_size / sizeof(ddi_intr_handle_t);
	nvme->per_cpu_io_queues = 1;
	nvme->rid = 0;
	return (0);
}

#if 0
static int
nvme_ctrlr_ioctl(struct cdev *cdev, u_long cmd, caddr_t arg, int flag,
    struct thread *td)
{
	struct nvme_controller	*ctrlr;
	struct nvme_completion	cpl;
	struct mtx		*mtx;

	ctrlr = cdev->si_drv1;

	switch (cmd) {
	case NVME_IDENTIFY_CONTROLLER:
#ifdef CHATHAM2
		/*
		 * Don't refresh data on Chatham, since Chatham returns
		 *  garbage on IDENTIFY anyways.
		 */
		if (pci_get_devid(ctrlr->dev) == CHATHAM_PCI_ID) {
			memcpy(arg, &ctrlr->cdata, sizeof(ctrlr->cdata));
			break;
		}
#endif
		/* Refresh data before returning to user. */
		mtx = mtx_pool_find(mtxpool_sleep, &cpl);
		mtx_lock(mtx);
		nvme_ctrlr_cmd_identify_controller(ctrlr, &ctrlr->cdata,
		    nvme_ctrlr_cb, &cpl);
		msleep(&cpl, mtx, PRIBIO, "nvme_ioctl", 0);
		mtx_unlock(mtx);
		if (cpl.sf_sc || cpl.sf_sct)
			return (ENXIO);
		memcpy(arg, &ctrlr->cdata, sizeof(ctrlr->cdata));
		break;
	default:
		return (ENOTTY);
	}

	return (0);
}

static struct cdevsw nvme_ctrlr_cdevsw = {
	.d_version =	D_VERSION,
	.d_flags =	0,
	.d_ioctl =	nvme_ctrlr_ioctl
};
#endif

int
nvme_ctrlr_construct(struct nvme_controller *ctrlr)
{
	union cap_lo_register	cap_lo;
	union cap_hi_register	cap_hi;
	int status = 0;
//	int			num_vectors, per_cpu_io_queues, status = 0;

	ctrlr->is_started = B_FALSE;

	printf("configure interrupts..\n");

	nvme_ctrlr_configure_intx(ctrlr);

	/*
	 * Software emulators may set the doorbell stride to something
	 *  other than zero, but this driver is not set up to handle that.
	 */
	cap_hi.raw = nvme_mmio_read_4(ctrlr, cap_hi);
	if (cap_hi.bits.dstrd != 0)
		return (ENXIO);

	/* Get ready timeout value from controller, in units of 500ms. */
	cap_lo.raw = nvme_mmio_read_4(ctrlr, cap_lo);
	ctrlr->ready_timeout_in_ms = cap_lo.bits.to * 500;

	/* One vector per IO queue, plus one vector for admin queue. */
//	num_vectors = ctrlr->num_io_queues + 1;

	nvme_ctrlr_construct_admin_qpair(ctrlr);

	status = nvme_ctrlr_construct_io_qpairs(ctrlr);

	if (status != 0)
		return (status);

	return (0);
}

void
nvme_ctrlr_submit_admin_request(struct nvme_controller *ctrlr,
    struct nvme_request *req)
{
	clock_t deadline;
	int ret;

	deadline = ddi_get_lbolt() + (clock_t )drv_usectohz(3 * 1000000);

	mutex_enter(&req->mutex);
	nvme_qpair_submit_request(&ctrlr->adminq, req);

	ret = cv_timedwait(&req->cv, &req->mutex, deadline);
	if (ret < 0)
	{
		printf("no response from controller in 3 seconds!\n");
		printf("TODO: %s: update me to return ETIMEDOUT\n", __func__);
	}
}

void
nvme_ctrlr_submit_io_request(struct nvme_controller *ctrlr,
    struct nvme_request *req)
{
	struct nvme_qpair       *qpair;

	qpair = &ctrlr->ioq[0];

	nvme_qpair_submit_request(qpair, req);
}
