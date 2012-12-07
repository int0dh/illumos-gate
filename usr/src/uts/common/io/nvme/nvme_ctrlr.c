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
#include <sys/cpuvar.h>
#include <sys/sysmacros.h>

#include "nvme.h"
#include "nvme_private.h"

extern void kmdb_enter();

void
nvme_admin_cb(void *arg, const nvme_completion_t *status, nvme_tracker_t *tr)
{
	nvme_completion_t	*cpl = arg;
	/*
	 * Copy status into the argument passed by the caller, so that
	 *  the caller can check the status to determine if the
	 *  the request passed or failed.
	 */
	mutex_enter(&tr->mutex);
	memcpy(cpl, status, sizeof(*cpl));
	/* notify about admin command completion */
	cv_broadcast(&tr->cv);
	mutex_exit(&tr->mutex);
}

static int 
nvme_ctrlr_construct_admin_qpair(nvme_controller_t *ctrlr)
{
	nvme_qpair_t	*qpair = &ctrlr->adminq;
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
nvme_ctrlr_construct_io_qpairs(nvme_controller_t *ctrlr)
{
	nvme_qpair_t	*qpair;
	union cap_lo_register	cap_lo;
	int	i, ret, num_entries, num_trackers;

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

	ctrlr->ioq = kmem_zalloc(ctrlr->num_io_queues * sizeof(nvme_qpair_t), KM_SLEEP);

	for (i = 0; i < ctrlr->num_io_queues; i++) {
		qpair = &ctrlr->ioq[i];
		/*
		 * Admin queue has ID=0. IO queues start at ID=1 -
		 *  hence the 'i+1' here.
		 *
		 * For I/O queues, use the controller-wide max_xfer_size
		 *  calculated in nvme_attach().
		 */
		if ((ret = nvme_qpair_construct(qpair,
				     i+1, /* qpair ID */
				     ctrlr->msix_enabled ? i+1 : 0, /* vector */
				     num_entries,
				     num_trackers,
				     ctrlr->max_xfer_size,
				     ctrlr)) != 0) {
			int j;

			for (j = 0; j < i; j ++) {
				nvme_qpair_t *q = &ctrlr->ioq[i];

				nvme_io_qpair_destroy(q);
			}
			(void) nvme_admin_qpair_destroy(&ctrlr->adminq);
			kmem_free(ctrlr->ioq,
				ctrlr->num_io_queues * sizeof(struct nvme_qpair));
			return (ret);
		}

	}
	return (0);
}

static int
nvme_ctrlr_wait_for_ready(nvme_controller_t *ctrlr)
{
	int ms_waited;
	union cc_register cc;
	union csts_register csts;

	cc.raw = nvme_mmio_read_4(ctrlr, cc);
	csts.raw = nvme_mmio_read_4(ctrlr, csts);

	if (!cc.bits.en) {
		return (ENXIO);
	}
	ms_waited = 0;

	while (!csts.bits.rdy) {
		DELAY(1000);
		if (ms_waited ++ > ctrlr->ready_timeout_in_ms) {
			return (ENXIO);
		}
		csts.raw = nvme_mmio_read_4(ctrlr, csts);
	}
	return (0);
}
static void
nvme_ctrlr_disable(nvme_controller_t *ctrlr)
{
	union cc_register cc;
	union csts_register csts;

	cc.raw = nvme_mmio_read_4(ctrlr, cc);
	csts.raw = nvme_mmio_read_4(ctrlr, csts);

	if (cc.bits.en == 1 && csts.bits.rdy == 0) {
		nvme_ctrlr_wait_for_ready(ctrlr);
	}
	cc.bits.en = 0;
	nvme_mmio_write_4(ctrlr, cc, cc.raw);
	DELAY(5000);
}
static int
nvme_ctrlr_enable(nvme_controller_t *ctrlr)
{
	union cc_register	cc;
	union csts_register	csts;
	union aqa_register	aqa;

	cc.raw = nvme_mmio_read_4(ctrlr, cc);
	csts.raw = nvme_mmio_read_4(ctrlr, csts);

	if (cc.bits.en == 1) {
		if (csts.bits.rdy == 1) {
			return (0);
		}
		else {
			return (nvme_ctrlr_wait_for_ready(ctrlr));
		}
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
nvme_ctrlr_reset(nvme_controller_t *ctrlr)
{
	nvme_ctrlr_disable(ctrlr);
	return (nvme_ctrlr_enable(ctrlr));
}

static int
nvme_ctrlr_identify(nvme_controller_t *ctrlr)
{
	nvme_completion_t	cpl;
	int ret;

	ret = nvme_ctrlr_cmd_identify_controller(ctrlr, ctrlr->cdata_phys,
	    nvme_admin_cb, &cpl);

	if (ret) {
		return ret;
	}
	if (cpl.sf_sc || cpl.sf_sct) {
		return (ENXIO);
	}
	return (0);
}

static int
nvme_ctrlr_set_num_qpairs(nvme_controller_t *ctrlr)
{
	nvme_completion_t	cpl;
	int			cq_allocated, sq_allocated;
	int ret;

	ret = nvme_ctrlr_cmd_set_num_queues(ctrlr, ctrlr->num_io_queues,
	    nvme_admin_cb, &cpl);

	if (ret) {
		return ret;
	}
	if (cpl.sf_sc || cpl.sf_sct) {
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
	}
	return (0);
}

static int
nvme_ctrlr_create_qpairs(nvme_controller_t *ctrlr)
{
	nvme_qpair_t	*qpair;
	nvme_completion_t	cpl;
	int i, ret;

	printf("%s: %d IO queues specified!\n", __func__, ctrlr->num_io_queues);

	for (i = 0; i < ctrlr->num_io_queues; i++) {
		qpair = &ctrlr->ioq[i];

		ret = nvme_ctrlr_cmd_create_io_cq(ctrlr, qpair, qpair->vector,
		    nvme_admin_cb, &cpl);

		if (ret) {
			return ret;
		}
		if (cpl.sf_sc || cpl.sf_sct) {
			return (ENXIO);
		}
		ret = nvme_ctrlr_cmd_create_io_sq(qpair->ctrlr, qpair,
		    nvme_admin_cb, &cpl);

		if (ret) {
			return ret;
		}
		if (cpl.sf_sc || cpl.sf_sct) {
			return (ENXIO);
		}
	}
	return 0;
}

static int
nvme_ctrlr_construct_namespaces(nvme_controller_t *ctrlr)
{
	nvme_namespace_t	*ns;
	int			i, status;
	/* one NVMe may have a few namespaces. Identify each */
	for (i = 0; i < ctrlr->cdata.nn; i++) {
		ns = &ctrlr->ns[i];

		status = nvme_ns_construct(ns, i + 1, ctrlr);
		if (status != 0) {
			return (status);
		}
	}
	return 0;
}

int
nvme_ctrlr_start(void *ctrlr_arg)
{
	nvme_controller_t *ctrlr = ctrlr_arg;

	printf("enabling interrupts..");
	nvme_interrupt_enable(ctrlr);

	if (nvme_ctrlr_identify(ctrlr) != 0) {
		nvme_interrupt_disable(ctrlr);
		return (ENXIO);
	}
	if (nvme_ctrlr_set_num_qpairs(ctrlr) != 0) {
		nvme_interrupt_disable(ctrlr);
		return (ENXIO);
	}
	if (nvme_ctrlr_create_qpairs(ctrlr) != 0) {
		nvme_interrupt_disable(ctrlr);
		return (ENXIO);
	}
	if (nvme_ctrlr_construct_namespaces(ctrlr) != 0) {
		nvme_interrupt_disable(ctrlr);
		return (ENXIO);
	}
	ctrlr->is_started = B_TRUE;
	return 0;
}
/* this softintr is used when obsolete INTx PCI interrupts are in use */
uint_t
nvme_ctrlr_softintr_handler(char *arg, char *unused)
{
	nvme_qpair_t *qpair = (nvme_qpair_t *)arg;
	nvme_controller_t *nvme = qpair->ctrlr;

	nvme_qpair_process_completions(&nvme->adminq);

	if (nvme->ioq[0].cpl) {
		nvme_qpair_process_completions(&nvme->ioq[0]);
	}
	nvme_mmio_write_4(qpair->ctrlr, intmc, 1);
	return DDI_INTR_CLAIMED;
}
/* this softintr is for MSI/MSI-X case */
uint_t
nvme_ctrlr_softintr_msi_handler(char *arg, char *unused)
{
	nvme_qpair_t *qpair = (nvme_qpair_t *)arg;

	if (qpair->cpl) {
		nvme_qpair_process_completions(qpair);
	}
	nvme_mmio_write_4(qpair->ctrlr, intmc, 1 << qpair->id);
	return DDI_INTR_CLAIMED;
}

uint_t 
nvme_ctrlr_intr_handler(char *arg, char *unused)
{
	nvme_qpair_t *qpair = (nvme_qpair_t *)arg;

	nvme_mmio_write_4(qpair->ctrlr, intms, 1 << qpair->id);

	if (qpair->soft_intr_handle) {
		(void)ddi_intr_trigger_softint(*qpair->soft_intr_handle, NULL);
	}
	return DDI_INTR_CLAIMED;
}

void
nvme_interrupt_enable(nvme_controller_t *nvme)
{
	if (nvme->intr_handle) {
		ddi_intr_enable(nvme->intr_handle[0]);
	}
}

void
nvme_interrupt_disable(nvme_controller_t *nvme)
{
	if (nvme->intr_handle) {
		ddi_intr_disable(nvme->intr_handle[0]);
	}
}

int
nvme_qpair_unregister_interrupt(nvme_qpair_t *q)
{
	if (q->soft_intr_handle) {
		ddi_intr_remove_softint(*q->soft_intr_handle);
	}
	if (q->hw_intr_handle) {
		ddi_intr_remove_handler(*q->hw_intr_handle);
	}
	return 0;
}
	
int
nvme_qpair_register_interrupt(nvme_qpair_t *q)
{
	int32_t intr_type;
	nvme_controller_t *nvme = q->ctrlr;
	uint_t (*softintr_handler)(char *, char *);
	u_int prio;

	if (nvme->msix_enabled != B_TRUE) {
		softintr_handler = nvme_ctrlr_softintr_handler;
		/* no MSI under QEMU :( */
		if (q != &nvme->adminq) {
			/* required for mutex initialization */
			q->soft_intr_pri = nvme->adminq.soft_intr_pri;
			return 0;
		}
	}
	else
		softintr_handler = nvme_ctrlr_softintr_msi_handler;


	if (ddi_intr_add_softint(nvme->devinfo,
			&nvme->soft_intr_handle[q->id],
			DDI_INTR_SOFTPRI_MAX, 
			softintr_handler, 
			(caddr_t)q) != DDI_SUCCESS) {
		return (EINVAL);
	}
	q->soft_intr_handle = &nvme->soft_intr_handle[q->id];
	q->hw_intr_handle = &nvme->intr_handle[q->id];
	(void) ddi_intr_get_softint_pri(*q->soft_intr_handle, &q->soft_intr_pri);

	if (ddi_intr_add_handler(nvme->intr_handle[q->id], 
				nvme_ctrlr_intr_handler, 
				(caddr_t)q, NULL) != DDI_SUCCESS) {
		(void) ddi_intr_remove_softint(nvme->soft_intr_handle[q->id]);
		return (EINVAL);
	}
	return (0);
}

static int
nvme_allocate_interrupts(nvme_controller_t *nvme)
{
	
	int count, actual;
	int intr_type;

	nvme->msix_enabled = B_FALSE;
	/* fist of all, check if MSI/MSI-x is supported */
	if (ddi_intr_get_supported_types(nvme->devinfo,
		 &intr_type) != DDI_SUCCESS) {
		printf("cannot get supported interrupt types\n");
		return (EINVAL);
	}
	if (intr_type & DDI_INTR_TYPE_MSIX) {

		printf("MSIX interrupts are supported!!\n");
		intr_type &= ~DDI_INTR_TYPE_FIXED;

		if (ddi_intr_get_nintrs(nvme->devinfo,
					intr_type, &count) != DDI_SUCCESS) {
			return (EINVAL);
		}
		nvme->msix_enabled = B_TRUE;
		nvme->num_io_queues = min(ncpus, count);
		/* + one ADMIN queue */
		count = nvme->num_io_queues + 1;

		dev_err(nvme->devinfo, CE_WARN,
			 "%d IO queues to be allocated\n", count);
	} else {
		printf("MSI interrupts are not supported ._.\n");
		nvme->num_io_queues = 1;
		count = 1;
	}
	nvme->intr_handle = kmem_zalloc(count * sizeof(ddi_intr_handle_t), KM_SLEEP);
	nvme->soft_intr_handle = kmem_zalloc(count * sizeof(ddi_softint_handle_t), KM_SLEEP);

	if (ddi_intr_alloc(nvme->devinfo, nvme->intr_handle,
				 intr_type, 0, count, &actual,
				 DDI_INTR_ALLOC_NORMAL) != DDI_SUCCESS)
	{
		kmem_free(nvme->intr_handle, count * sizeof(ddi_intr_handle_t));
		kmem_free(nvme->soft_intr_handle, count * sizeof(ddi_softint_handle_t));
		return (ENOMEM);
	}
	printf("actual interrupts %d\n", actual);
	nvme->intr_size = actual;
	return 0;
}

int nvme_release_interrupts(struct nvme_controller *nvme)
{
	int i;

	for (i = 0; i < nvme->intr_size; i ++) {
		(void) ddi_intr_remove_handler(nvme->intr_handle[i]);
		(void) ddi_intr_free(nvme->intr_handle[i]);
		(void) ddi_intr_remove_softint(nvme->soft_intr_handle[i]);
	}
	kmem_free(nvme->intr_handle, nvme->intr_size * sizeof(ddi_intr_handle_t));
	kmem_free(nvme->soft_intr_handle, nvme->intr_size * sizeof(ddi_softint_handle_t));
	return (0);
}

int
nvme_ctrlr_construct(nvme_controller_t *nvme)
{
	union cap_lo_register	cap_lo;
	union cap_hi_register	cap_hi;
	int status = 0, count;
	
	(void) nvme_allocate_interrupts(nvme);
	/*
	 * Software emulators may set the doorbell stride to something
	 *  other than zero, but this driver is not set up to handle that.
	 */
	cap_hi.raw = nvme_mmio_read_4(nvme, cap_hi);
	/* Get ready timeout value from controller, in units of 500ms. */
	cap_lo.raw = nvme_mmio_read_4(nvme, cap_lo);
	nvme->ready_timeout_in_ms = cap_lo.bits.to * 500;

	nvme_ctrlr_construct_admin_qpair(nvme);

	status = nvme_ctrlr_construct_io_qpairs(nvme);

	if (status != 0) {
		nvme_release_interrupts(nvme);
		return (status);
	}
	return (0);
}
