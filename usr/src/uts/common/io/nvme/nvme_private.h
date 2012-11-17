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
 *
 * $FreeBSD$
 */

#ifndef __NVME_PRIVATE_H__
#define __NVME_PRIVATE_H__

#include "nvme.h"
#include <sys/queue.h>

#define CHATHAM2

#define IDT_PCI_ID		0x80d0111d

#define NVME_MAX_PRP_LIST_ENTRIES	(32)

/*
 * For commands requiring more than 2 PRP entries, one PRP will be
 *  embedded in the command (prp1), and the rest of the PRP entries
 *  will be in a list pointed to by the command (prp2).  This means
 *  that real max number of PRP entries we support is 32+1, which
 *  results in a max xfer size of 32*PAGE_SIZE.
 */
#define NVME_MAX_XFER_SIZE	NVME_MAX_PRP_LIST_ENTRIES * PAGESIZE

#define NVME_ADMIN_TRACKERS	(16)
#define NVME_ADMIN_ENTRIES	(128)
/* min and max are defined in admin queue attributes section of spec */
#define NVME_MIN_ADMIN_ENTRIES	(2)
#define NVME_MAX_ADMIN_ENTRIES	(4096)

/*
 * NVME_IO_ENTRIES defines the size of an I/O qpair's submission and completion
 *  queues, while NVME_IO_TRACKERS defines the maximum number of I/O that we
 *  will allow outstanding on an I/O qpair at any time.  The only advantage in
 *  having IO_ENTRIES > IO_TRACKERS is for debugging purposes - when dumping
 *  the contents of the submission and completion queues, it will show a longer
 *  history of data.
 */
#define NVME_IO_ENTRIES		(256)
#define NVME_IO_TRACKERS	(128)
#define NVME_MIN_IO_TRACKERS	(16)
#define NVME_MAX_IO_TRACKERS	(1024)

/*
 * NVME_MAX_IO_ENTRIES is not defined, since it is specified in CC.MQES
 *  for each controller.
 */

#define NVME_INT_COAL_TIME	(0)	/* disabled */
#define NVME_INT_COAL_THRESHOLD (0)	/* 0-based */

#define NVME_MAX_NAMESPACES	(16)
#define NVME_MAX_CONSUMERS	(2)
#define NVME_MAX_ASYNC_EVENTS	(4)

#define NVME_TIMEOUT_IN_SEC	(30)

#ifndef CACHE_LINE_SIZE
#define CACHE_LINE_SIZE		(64)
#endif

struct nvme_request {

	struct nvme_command		cmd;
	void				*payload;
	uint32_t			payload_size;
	bd_xfer_t			*xfer; /* for now - one request - one xfer */
	
	kcondvar_t			cv;
	kmutex_t			mutex;

	nvme_cb_fn_t			cb_fn;
	void				*cb_arg;
	STAILQ_ENTRY(nvme_request)	stailq;
	TAILQ_ENTRY(nvme_request)	rq_next; /* next in request queue */
};

struct nvme_tracker {

	SLIST_ENTRY(nvme_tracker)	slist;
	struct nvme_request		*req;
	struct nvme_qpair		*qpair;
//	struct callout			timer;
	uint16_t			cid;

	uint64_t			prp[NVME_MAX_PRP_LIST_ENTRIES];
	uint64_t			prp_bus_addr;
//	bus_dmamap_t			prp_dma_map;
};

struct nvme_qpair {

	struct nvme_controller	*ctrlr;
	ddi_acc_handle_t	cmd_dma_acc_handle;
	ddi_acc_handle_t	cpl_dma_acc_handle;
	ddi_softint_handle_t  *soft_intr_handle;

	uint32_t		id;
	uint32_t		phase;

	uint16_t		vector;
	int			rid;
	void 			*tag;

	uint32_t		max_xfer_size;
	uint32_t		num_entries;
	uint32_t		num_trackers;
	uint32_t		sq_tdbl_off;
	uint32_t		cq_hdbl_off;

	uint32_t		sq_head;
	uint32_t		sq_tail;
	uint32_t		cq_head;

	int64_t			num_cmds;
	int64_t			num_intr_handler_calls;

	struct nvme_command	*cmd;
	struct nvme_completion	*cpl;

	ddi_dma_handle_t	dma_tag;

//	bus_dma_tag_t		dma_tag;

//	bus_dmamap_t		cmd_dma_map;
	uint64_t		cmd_bus_addr;

//	bus_dmamap_t		cpl_dma_map;
	uint64_t		cpl_bus_addr;

	SLIST_HEAD(, nvme_tracker)	free_tr;
	STAILQ_HEAD(, nvme_request)	queued_req;

	TAILQ_HEAD(, nvme_request)	request_queue;
	struct nvme_tracker	**act_tr;
	struct nvme_tracker     *free_tr_mem;
	lock_t		lock;
	
} __aligned(CACHE_LINE_SIZE);

struct nvme_namespace {

	struct nvme_controller		*ctrlr;
	struct nvme_namespace_data	data;
	bd_handle_t			bd_handle;
	uint16_t			id;
	uint16_t			flags;
};

/*
 * One of these per allocated PCI device.
 */
struct nvme_controller {

	dev_info_t		*devinfo;
	ddi_dma_handle_t        dma_handle;
	dev_info_t		*dev;
	int			nvme_nbloks;

	ddi_intr_handle_t	*intr_handle;
	ddi_softint_handle_t    *soft_intr_handle;
	int			intr_size;

	uint32_t		ready_timeout_in_ms;
	ddi_acc_handle_t        nvme_regs_handle;
	uint8_t			*nvme_regs_base;

	uint32_t		msix_enabled;
	uint32_t		force_intx;

	uint32_t		num_io_queues;
	boolean_t		per_cpu_io_queues;

	/* Fields for tracking progress during controller initialization. */
//	struct intr_config_hook	config_hook;
	uint32_t		ns_identified;
	uint32_t		queues_created;

	/* For shared legacy interrupt. */
	int			rid;
//	struct resource		*res;
	void			*tag;
//	struct task		task;
//	struct taskqueue	*taskqueue;

	ddi_dma_handle_t	hw_desc_tag;
//	bus_dmamap_t		hw_desc_map;

	/** maximum i/o size in bytes */
	uint32_t		max_xfer_size;

	/** interrupt coalescing time period (in microseconds) */
	uint32_t		int_coal_time;

	/** interrupt coalescing threshold */
	uint32_t		int_coal_threshold;

	struct nvme_qpair	adminq;
	struct nvme_qpair	*ioq;

	struct nvme_registers		*regs;

	struct nvme_controller_data	cdata;
	struct nvme_namespace		ns[NVME_MAX_NAMESPACES];

//	struct cdev			*cdev;

	boolean_t			is_started;
};

#define nvme_mmio_offsetof(reg)						       \
	offsetof(struct nvme_registers, reg)

#define nvme_mmio_read_4(sc, reg)					       \
	ddi_get32((sc)->nvme_regs_handle, (uint32_t *)((sc)->nvme_regs_base + nvme_mmio_offsetof(reg)))

#define nvme_mmio_write_4(sc, reg, val)					       \
	ddi_put32((sc)->nvme_regs_handle, (uint32_t *)((sc)->nvme_regs_base + nvme_mmio_offsetof(reg)), val)

#define nvme_mmio_write_8(sc, reg, val) \
	do { \
		ddi_put32((sc)->nvme_regs_handle, (uint32_t *)((sc)->nvme_regs_base + nvme_mmio_offsetof(reg)), (val) & 0xFFFFFFFF); \
		ddi_put32((sc)->nvme_regs_handle, (uint32_t *)((sc)->nvme_regs_base + nvme_mmio_offsetof(reg) + 4), ((val) & 0xFFFFFFFF00000000UL) >> 32); \
	} while (0)


#if __FreeBSD_version < 800054
#define wmb()	__asm volatile("sfence" ::: "memory")
#define mb()	__asm volatile("mfence" ::: "memory")
#endif

void    nvme_interrupt_enable(struct nvme_controller *nvme);
void	nvme_interrupt_disable(struct nvme_controller *nvme);

void	nvme_ns_test(struct nvme_namespace *ns, u_long cmd, caddr_t arg);

int    nvme_ctrlr_construct(struct nvme_controller *ctrlr);
void	nvme_ctrlr_cmd_set_feature(struct nvme_controller *ctrlr,
				   uint8_t feature, uint32_t cdw11,
				   void *payload, uint32_t payload_size,
				   nvme_cb_fn_t cb_fn, void *cb_arg);
void	nvme_ctrlr_cmd_get_feature(struct nvme_controller *ctrlr,
				   uint8_t feature, uint32_t cdw11,
				   void *payload, uint32_t payload_size,
				   nvme_cb_fn_t cb_fn, void *cb_arg);
void	nvme_ctrlr_cmd_identify_controller(struct nvme_controller *ctrlr,
					   void *payload,
					   nvme_cb_fn_t cb_fn, void *cb_arg);
void	nvme_ctrlr_cmd_identify_namespace(struct nvme_controller *ctrlr,
					  uint16_t nsid, void *payload,
					  nvme_cb_fn_t cb_fn, void *cb_arg);
void	nvme_ctrlr_cmd_set_interrupt_coalescing(struct nvme_controller *ctrlr,
						uint32_t microseconds,
						uint32_t threshold,
						nvme_cb_fn_t cb_fn,
						void *cb_arg);
void	nvme_ctrlr_cmd_get_health_information_page(struct nvme_controller *ctrlr,
						   uint32_t nsid,
						   struct nvme_health_information_page *payload,
						   nvme_cb_fn_t cb_fn,
						   void *cb_arg);
void	nvme_ctrlr_cmd_create_io_cq(struct nvme_controller *ctrlr,
				    struct nvme_qpair *io_que, uint16_t vector,
				    nvme_cb_fn_t cb_fn, void *cb_arg);
void	nvme_ctrlr_cmd_create_io_sq(struct nvme_controller *ctrlr,
				    struct nvme_qpair *io_que,
				    nvme_cb_fn_t cb_fn, void *cb_arg);
void	nvme_ctrlr_cmd_delete_io_cq(struct nvme_controller *ctrlr,
				    struct nvme_qpair *io_que,
				    nvme_cb_fn_t cb_fn, void *cb_arg);
void	nvme_ctrlr_cmd_delete_io_sq(struct nvme_controller *ctrlr,
				    struct nvme_qpair *io_que,
				    nvme_cb_fn_t cb_fn, void *cb_arg);
void	nvme_ctrlr_cmd_set_num_queues(struct nvme_controller *ctrlr,
				      uint32_t num_queues, nvme_cb_fn_t cb_fn,
				      void *cb_arg);
void	nvme_ctrlr_cmd_set_asynchronous_event_config(struct nvme_controller *ctrlr,
					   union nvme_critical_warning_state state,
					   nvme_cb_fn_t cb_fn, void *cb_arg);
void	nvme_ctrlr_cmd_asynchronous_event_request(struct nvme_controller *ctrlr,
						  nvme_cb_fn_t cb_fn,
						  void *cb_arg);

void	nvme_payload_map(struct nvme_tracker *tr, ddi_dma_handle_t dmah, ddi_dma_cookie_t *dmac, void *addr, size_t len);
//void	nvme_payload_map_uio(void *arg, bus_dma_segment_t *seg, int nseg,
//			     bus_size_t mapsize, int error);

//int	nvme_ctrlr_construct(struct nvme_controller *ctrlr, device_t dev);
int	nvme_ctrlr_reset(struct nvme_controller *ctrlr);
/* ctrlr defined as void * to allow use with config_intrhook. */
int	nvme_ctrlr_start(void *ctrlr_arg);
void	nvme_ctrlr_submit_admin_request(struct nvme_controller *ctrlr,
					struct nvme_request *req);
void	nvme_ctrlr_submit_io_request(struct nvme_controller *ctrlr,
				     struct nvme_request *req);

int	nvme_qpair_construct(struct nvme_qpair *qpair, uint32_t id,
			     uint16_t vector, uint32_t num_entries,
			     uint32_t num_trackers, uint32_t max_xfer_size,
			     struct nvme_controller *ctrlr);
void	nvme_qpair_submit_cmd(struct nvme_qpair *qpair,
			      struct nvme_tracker *tr);
void	nvme_qpair_process_completions(struct nvme_qpair *qpair);
void	nvme_qpair_submit_request(struct nvme_qpair *qpair,
				  struct nvme_request *req);

void	nvme_admin_qpair_destroy(struct nvme_qpair *qpair);

void	nvme_io_qpair_destroy(struct nvme_qpair *qpair);

int	nvme_ns_construct(struct nvme_namespace *ns, uint16_t id,
			  struct nvme_controller *ctrlr);

//int	nvme_ns_physio(struct cdev *dev, struct uio *uio, int ioflag);

void	nvme_sysctl_initialize_ctrlr(struct nvme_controller *ctrlr);

void	nvme_dump_command(struct nvme_command *cmd);
void	nvme_dump_completion(struct nvme_completion *cpl);

#if 0
static __inline void
nvme_single_map(void *arg, bus_dma_segment_t *seg, int nseg, int error)
{
	uint64_t *bus_addr = (uint64_t *)arg;

	*bus_addr = seg[0].ds_addr;
}
#endif

static __inline struct nvme_request *
nvme_allocate_request(struct nvme_qpair *q, void *payload, uint32_t payload_size, nvme_cb_fn_t cb_fn, void *cb_arg)
{
	struct nvme_request *req = NULL;

#if 0
	req = kmem_zalloc(sizeof(* req), KM_NOSLEEP);
	if (req == NULL)
		return (NULL);
#endif
	lock_set(&q->lock);

	if (TAILQ_EMPTY(&q->request_queue))
	{
		lock_clear(&q->lock);
		return NULL;
	}
	req = TAILQ_FIRST(&q->request_queue);
	TAILQ_REMOVE(&q->request_queue, req, rq_next);
	lock_clear(&q->lock);

	memset(req, 0, sizeof(*req));

	req->payload = payload;
	req->payload_size = payload_size;
	req->cb_fn = cb_fn;
	req->cb_arg = cb_arg;

	mutex_init(&req->mutex, NULL, MUTEX_DRIVER, NULL);
	cv_init(&req->cv, NULL, CV_DRIVER, NULL);

	printf("mutex at %p initialized!\n", &req->mutex);
	return (req);
}

static __inline void
nvme_free_request(struct nvme_qpair *q, struct nvme_request *req)
{
	mutex_destroy(&req->mutex);
	cv_destroy(&req->cv);

	lock_set(&q->lock);
	TAILQ_INSERT_HEAD(&q->request_queue, req, rq_next);
	lock_clear(&q->lock);
}
#endif /* __NVME_PRIVATE_H__ */
