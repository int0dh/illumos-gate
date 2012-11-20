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
#define blabla
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

extern void kmdb_enter();

static int
nvme_ctrlr_configure_intx(struct nvme_controller *nvme);

static void
nvme_ctrlr_cb(void *arg, const struct nvme_completion *status, struct nvme_request *req)
{
	struct nvme_completion	*cpl = arg;

	printf("callback %s called! req at 0x%p\n", __func__, req);

	/* TODO: do ddi_dma_sync() here */
	mutex_enter(&req->mutex);

	/*
	 * Copy status into the argument passed by the caller, so that
	 *  the caller can check the status to determine if the
	 *  the request passed or failed.
	 */
	memcpy(cpl, status, sizeof(*cpl));

	cv_broadcast(&req->cv);

	mutex_exit(&req->mutex);
	printf("callback completed!\n");
}

static int 
nvme_ctrlr_construct_admin_qpair(struct nvme_controller *ctrlr)
{
	struct nvme_qpair	*qpair = &ctrlr->adminq;
	/*
	 * The admin queue's max xfer size is treated differently than the
	 *  max I/O xfer size.  16KB is sufficient here - maybe even less?
	 */
	return nvme_qpair_construct(qpair, 
			     0, /* qpair ID */
			     0, /* vector */
			     NVME_ADMIN_ENTRIES,
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

	/* FIXME FIXME FIXME */
	/* we have to be invoked with already initiazlied ctrl->num_io_queues,
	* but we don`t. so initialize this to value more than maximal possible
	* quick and very dirty */
	ctrlr->num_io_queues = 8;

	ctrlr->ioq = kmem_zalloc(ctrlr->num_io_queues * sizeof(struct nvme_qpair), KM_SLEEP);

	for (i = 0; i < ctrlr->num_io_queues; i++)
	{
		qpair = &ctrlr->ioq[i];

		/*
		 * Admin queue has ID=0. IO queues start at ID=1 -
		 *  hence the 'i+1' here.
		 *
		 * For I/O queues, use the controller-wide max_xfer_size
		 *  calculated in nvme_attach().
		 */
		/* TODO: do something when nvme_qpair_construct() fails */
		nvme_qpair_construct(qpair,
				     i+1, /* qpair ID */
				     ctrlr->msix_enabled ? i+1 : 0, /* vector */
				     num_entries,
				     num_trackers,
				     ctrlr->max_xfer_size,
				     ctrlr);

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

	if (!cc.bits.en)
		return (ENXIO);

	ms_waited = 0;

	while (!csts.bits.rdy)
	{
		DELAY(1000);
		if (ms_waited ++ > ctrlr->ready_timeout_in_ms)
			return (ENXIO);
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

	nvme_mmio_write_8(ctrlr, asq, ctrlr->adminq.cmd_bus_addr);
	DELAY(5000);
	nvme_mmio_write_8(ctrlr, acq, ctrlr->adminq.cpl_bus_addr);
	DELAY(5000);

	aqa.raw = 0;
	/* acqs and asqs are 0-based. */
	aqa.bits.acqs = ctrlr->adminq.num_entries-1;
	aqa.bits.asqs = ctrlr->adminq.num_entries-1;
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

	if (cpl.sf_sc || cpl.sf_sct) 
		return (ENXIO);

	printf("nvme->cdata.vid = %04x\n", ctrlr->cdata.vid);
	return (DDI_SUCCESS);
}

static int
nvme_ctrlr_set_num_qpairs(struct nvme_controller *ctrlr)
{
	struct nvme_completion	cpl;
	int			cq_allocated, sq_allocated;

	nvme_ctrlr_cmd_set_num_queues(ctrlr, ctrlr->num_io_queues,
	    nvme_ctrlr_cb, &cpl);

	if (cpl.sf_sc || cpl.sf_sct)
		return (ENXIO);
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
	    cq_allocated < ctrlr->num_io_queues) 
	{
		ctrlr->num_io_queues = 1;
		ctrlr->per_cpu_io_queues = 0;

		/* TODO: destroy extra queues that were created
		 *  previously but now found to be not needed.
		 */
	}
	return 0;
}

static int
nvme_ctrlr_create_qpairs(struct nvme_controller *ctrlr)
{
	struct nvme_qpair	*qpair;
	struct nvme_completion	cpl;
	int			i;

	printf("%s: %d IO queues specified!\n", __func__, ctrlr->num_io_queues);

	for (i = 0; i < ctrlr->num_io_queues; i++)
	{
		qpair = &ctrlr->ioq[i];

		printf("create IO cq!!!!\n");
		nvme_ctrlr_cmd_create_io_cq(ctrlr, qpair, qpair->vector,
		    nvme_ctrlr_cb, &cpl);

		if (cpl.sf_sc || cpl.sf_sct)
			return (ENXIO);

		printf("create IO sq!!!\n");
		nvme_ctrlr_cmd_create_io_sq(qpair->ctrlr, qpair,
		    nvme_ctrlr_cb, &cpl);

		if (cpl.sf_sc || cpl.sf_sct)
			return (ENXIO);
	}
	return 0;
}

static int
nvme_ctrlr_construct_namespaces(struct nvme_controller *ctrlr)
{
	struct nvme_namespace	*ns;
	int			i, status;

	printf("ctrlr->cdata.nn is %d!\n", ctrlr->cdata.nn);
	for (i = 0; i < ctrlr->cdata.nn; i++)
	{
		ns = &ctrlr->ns[i];
		status = nvme_ns_construct(ns, i + 1, ctrlr);
		if (status != 0)
			return (status);
		printf("namespace %d exist!\n", i);
	}
	return 0;
}

static void
nvme_ctrlr_configure_aer(struct nvme_controller *ctrlr)
{
	union nvme_critical_warning_state	state;
	uint8_t					num_async_events;
	/* FIXME FIXME FIXME */
	/* let`s implement this later */
	return;

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

int
nvme_ctrlr_start(void *ctrlr_arg)
{
	struct nvme_controller *ctrlr = ctrlr_arg;

	printf("configure interrupts..\n");

	nvme_ctrlr_configure_intx(ctrlr);
	
	if (nvme_ctrlr_identify(ctrlr) != 0)
		return (ENXIO);

	if (nvme_ctrlr_set_num_qpairs(ctrlr) != 0)
		return (ENXIO);

	printf("create qpairs..\n");
	if (nvme_ctrlr_create_qpairs(ctrlr) != 0)
		return (ENXIO);

	if (nvme_ctrlr_construct_namespaces(ctrlr) != 0)
		return (ENXIO);

	nvme_ctrlr_configure_aer(ctrlr);
	nvme_ctrlr_configure_int_coalescing(ctrlr);

	ctrlr->is_started = B_TRUE;
	return 0;
}

/* TODO: we need one handler per io qpair. how can we get vector number? */
static uint_t
nvme_ctrlr_softintr_handler(char *arg, char *unused)
{
	struct nvme_qpair *qpair = (struct nvme_qpair *)arg;
	struct nvme_controller *nvme = qpair->ctrlr;
// comment-out the code until MSI/MSI-X 
//	nvme_qpair_process_completions(qpair);

	nvme_qpair_process_completions(&nvme->adminq);

	if (nvme->ioq[0].cpl)
		nvme_qpair_process_completions(&nvme->ioq[0]);

	nvme_mmio_write_4(qpair->ctrlr, intmc, 1);
	return DDI_INTR_CLAIMED;
}

uint_t 
nvme_ctrlr_intx_handler(char *arg, char *unused)
{
	struct nvme_qpair *qpair = (struct nvme_qpair *)arg;

	nvme_mmio_write_4(qpair->ctrlr, intms, 0x1);

	if (qpair->soft_intr_handle)
		(void)ddi_intr_trigger_softint(*qpair->soft_intr_handle, NULL);

//	kmdb_enter();

	/* for some reason kernel invokes us again and again when we return */
	/*  DDI_INTR_CLAIMED. Not sure if this not a bug in QEMU */
	/* probably sharing PCI INTx interrupt gives such an effect */
	/* so let`s return 0 here, just as a workaround */
	return 0;
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
nvme_register_interrupts(struct nvme_controller *nvme, int intr_type)
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
	/* allocate one interrupt per I/O qpair + one for admin qpair */
	count = min(count, nvme->num_io_queues + 1);
	printf("updated count is %d\n", count);

	nvme->intr_size = count * sizeof(ddi_intr_handle_t);
	nvme->intr_handle = kmem_alloc(nvme->intr_size, KM_SLEEP);

	nvme->soft_intr_handle = kmem_alloc(nvme->intr_size, KM_SLEEP);
	
	ret = ddi_intr_alloc(nvme->devinfo, nvme->intr_handle, intr_type, 0, count, &actual, DDI_INTR_ALLOC_NORMAL);

	if (ret != DDI_SUCCESS)
	{
		printf("cannot allocate interrupt handle!\n");
		kmem_free(nvme->intr_handle, nvme->intr_size);
		kmem_free(nvme->soft_intr_handle, nvme->intr_size);
		return ret;
	}
/* DISABLE THIS code for now, until we get MSI working */
#if 0
	for (i = 1; i < count; i ++)
	{
		
		ret = ddi_intr_add_handler(nvme->intr_handle[i], nvme_ctrlr_intx_handler, (caddr_t)&nvme->ioq[i - 1], NULL);
		if (ret != DDI_SUCCESS)
		{
			printf("cannot register interrupt handler!, ret %d\n", ret);
			return DDI_FAILURE;
		}
		(void)ddi_intr_add_softint(nvme->devinfo, &nvme->soft_intr_handle[i], DDI_INTR_SOFTPRI_MAX, nvme_ctrlr_softintr_handler, (caddr_t)&nvme->ioq[i - 1]);

		nvme->ioq[i - 1].soft_intr_handle = &nvme->soft_intr_handle[i];
	}
#endif
	/* register handler for admin qpair */
	ddi_intr_add_handler(nvme->intr_handle[0], nvme_ctrlr_intx_handler, (caddr_t)&nvme->adminq, NULL);
	ddi_intr_add_softint(nvme->devinfo, &nvme->soft_intr_handle[0], DDI_INTR_SOFTPRI_MAX, nvme_ctrlr_softintr_handler, (caddr_t)&nvme->adminq);
	nvme->adminq.soft_intr_handle = &nvme->soft_intr_handle[0];
 
	printf("%d interrupt handlers registered ok!\n", count); 
	nvme_interrupt_enable(nvme);

	return DDI_SUCCESS;
}
/* it is not clear yet how to get CPU count, so let`s hardcode the value for now */
#define CPU_COUNT 2
/*FIXME: check status for each DDI function */
static int
nvme_ctrlr_configure_intx(struct nvme_controller *nvme)
{
	int32_t intr_type;

	nvme->msix_enabled = 0;
	nvme->rid = 0;

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
	nvme_register_interrupts(nvme, intr_type);
	
	/* allocate one queue par per available interrupt */
	/* TODO: do not allocate more than number of CPUs */
	/* otherwise, some performance degradation may occur */
	nvme->num_io_queues = min(nvme->intr_size / sizeof(ddi_intr_handle_t), CPU_COUNT);
	printf("NUM I/O queues %d\n", nvme->num_io_queues);
	/* always use one I/O qpair per CPU for best performance */
	nvme->per_cpu_io_queues = 1;
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


	/*
	 * Software emulators may set the doorbell stride to something
	 *  other than zero, but this driver is not set up to handle that.
	 */
	cap_hi.raw = nvme_mmio_read_4(ctrlr, cap_hi);
# if 0
	if (cap_hi.bits.dstrd != 0)
	{
		printf("cannot read"); 
		return (ENXIO);
#endif
	/* Get ready timeout value from controller, in units of 500ms. */
	cap_lo.raw = nvme_mmio_read_4(ctrlr, cap_lo);
	ctrlr->ready_timeout_in_ms = cap_lo.bits.to * 500;

	nvme_ctrlr_construct_admin_qpair(ctrlr);

	status = nvme_ctrlr_construct_io_qpairs(ctrlr);

#if 0
	if (status != 0)
		return (status);
#endif
	return (0);
}

void
nvme_ctrlr_submit_admin_request(struct nvme_controller *ctrlr,
    struct nvme_request *req)
{
	clock_t deadline;
	int ret;

	deadline = ddi_get_lbolt() + (clock_t )drv_usectohz(3 * 1000000);

	printf("%s: mutex at %p, req at %p\n", __FUNCTION__, &req->mutex, req);

	nvme_qpair_submit_request(&ctrlr->adminq, req);

	mutex_enter(&req->mutex);
	ret = cv_timedwait(&req->cv, &req->mutex, deadline);
	mutex_exit(&req->mutex);
	if (ret < 0)
	{
		printf("no response from controller in 3 seconds!\n");
		printf("TODO: %s: update me to return ETIMEDOUT\n", __func__);
	}
	/* FIXME: we also have to release tracker and stuff when timeout expires */
	nvme_free_request(&ctrlr->adminq, req);
}

/* TODO: update me to chose IO qpair */
void
nvme_ctrlr_submit_io_request(struct nvme_controller *ctrlr,
    struct nvme_request *req)
{
	struct nvme_qpair       *qpair = &ctrlr->ioq[0];

	mutex_enter(&req->mutex);
	nvme_qpair_submit_request(qpair, req);
	cv_wait(&req->cv, &req->mutex);

	if (req->xfer)
		bd_xfer_done(req->xfer, 0);

	mutex_exit(&req->mutex);
	nvme_free_request(qpair, req);
}
