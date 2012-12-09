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
#include <sys/sdt.h>
#include "nvme.h"
#include "nvme_private.h"

int
nvme_ctrlr_cmd_identify_controller(nvme_controller_t *ctrlr, uint64_t payload,
	nvme_cb_fn_t cb_fn, void *cb_arg)
{
	nvme_command_t *cmd;
	nvme_tracker_t *tr;

	tr = nvme_allocate_tracker(&ctrlr->adminq, payload,
	    sizeof(struct nvme_controller_data), cb_fn, cb_arg);

	/* no resources in the admin qpair */
	if (tr == NULL) {
		DTRACE_PROBE(no_resources_in_admin_qpair);
		return (EAGAIN);
	}
	cmd = &tr->cmd;
	cmd->opc = NVME_OPC_IDENTIFY;
	cmd->cdw10 = 1;

	return nvme_qpair_submit_request(tr, SYNC);
}

int
nvme_ctrlr_cmd_identify_namespace(nvme_controller_t *ctrlr, uint16_t nsid,
	uint64_t payload, nvme_cb_fn_t cb_fn, void *cb_arg)
{
	nvme_tracker_t *tr;
	nvme_command_t *cmd;

	tr = nvme_allocate_tracker(&ctrlr->adminq, payload,
	    sizeof(struct nvme_namespace_data), cb_fn, cb_arg);

	if (tr == NULL) {
		DTRACE_PROBE(no_resources_in_admin_qpair);
		return EAGAIN;
	}
	cmd = &tr->cmd;
	cmd->opc = NVME_OPC_IDENTIFY;
	cmd->nsid = nsid;

	return nvme_qpair_submit_request(tr, SYNC);
}

int
nvme_ctrlr_cmd_create_io_cq(nvme_controller_t *ctrlr,
    nvme_qpair_t *io_que, uint16_t vector, nvme_cb_fn_t cb_fn,
    void *cb_arg)
{
	nvme_tracker_t *tr;
	nvme_command_t *cmd;

	tr = nvme_allocate_tracker(&ctrlr->adminq, NULL, 0, cb_fn, cb_arg);

	if (tr == NULL) {
		DTRACE_PROBE(no_resources_in_admin_qpair);
		return EAGAIN;
	}
	cmd = &tr->cmd;
	cmd->opc = NVME_OPC_CREATE_IO_CQ;
	cmd->cdw10 = ((io_que->num_entries-1) << 16) | io_que->id;
	/* 0x3 = interrupts enabled | physically contiguous */
	cmd->cdw11 = (vector << 16) | 0x3;
	cmd->prp1 = (uint64_t)io_que->cpl_bus_addr;

	return nvme_qpair_submit_request(tr, SYNC);
}

int
nvme_ctrlr_cmd_create_io_sq(nvme_controller_t *ctrlr, nvme_qpair_t *io_que,
	 nvme_cb_fn_t cb_fn, void *cb_arg)
{
	nvme_tracker_t *tr;
	nvme_command_t *cmd;

	tr = nvme_allocate_tracker(&ctrlr->adminq, NULL, 0, cb_fn, cb_arg);

	if (tr == NULL) {
		DTRACE_PROBE(no_resources_in_admin_qpair);
		return EAGAIN;
	}
	cmd = &tr->cmd;
	cmd->opc = NVME_OPC_CREATE_IO_SQ;
	cmd->cdw10 = ((io_que->num_entries-1) << 16) | io_que->id;
	/* 0x1 = physically contiguous */
	cmd->cdw11 = (io_que->id << 16) | 0x1;
	cmd->prp1 = (uint64_t)io_que->cmd_bus_addr;

	return nvme_qpair_submit_request(tr, SYNC);
}

int
nvme_ctrlr_cmd_delete_io_cq(nvme_controller_t *ctrlr, nvme_qpair_t *io_que,
		 nvme_cb_fn_t cb_fn, void *cb_arg)
{
	nvme_tracker_t *tr;
	nvme_command_t *cmd;

	tr = nvme_allocate_tracker(&ctrlr->adminq, NULL, 0, cb_fn, cb_arg);

	if (tr == NULL) {
		DTRACE_PROBE(no_resources_in_admin_qpair);
		return EAGAIN;
	}
	cmd = &tr->cmd;
	cmd->opc = NVME_OPC_DELETE_IO_CQ;
	cmd->cdw10 = io_que->id;

	return nvme_qpair_submit_request(tr, SYNC);
}

int
nvme_ctrlr_cmd_delete_io_sq(nvme_controller_t *ctrlr,
    nvme_qpair_t *io_que, nvme_cb_fn_t cb_fn, void *cb_arg)
{
	nvme_tracker_t *tr;
	nvme_command_t *cmd;

	tr = nvme_allocate_tracker(&ctrlr->adminq, NULL, 0, cb_fn, cb_arg);

	if (tr == NULL) {
		DTRACE_PROBE(no_resources_in_admin_qpair);
		return EAGAIN;
	}
	cmd = &tr->cmd;
	cmd->opc = NVME_OPC_DELETE_IO_SQ;
	cmd->cdw10 = io_que->id;

	return nvme_qpair_submit_request(tr, SYNC);
}

int
nvme_ctrlr_cmd_set_feature(nvme_controller_t *ctrlr, uint8_t feature,
    uint32_t cdw11, void *payload, uint32_t payload_size,
    nvme_cb_fn_t cb_fn, void *cb_arg)
{
	nvme_tracker_t *tr;
	nvme_command_t *cmd;

	tr = nvme_allocate_tracker(&ctrlr->adminq, NULL, 0, cb_fn, cb_arg);

	if (tr == NULL) {
		DTRACE_PROBE(no_resources_in_admin_qpair);
		return EAGAIN;
	}
	cmd = &tr->cmd;
	cmd->opc = NVME_OPC_SET_FEATURES;
	cmd->cdw10 = feature;
	cmd->cdw11 = cdw11;

	return nvme_qpair_submit_request(tr, SYNC);
}

int
nvme_ctrlr_cmd_get_feature(nvme_controller_t *ctrlr, uint8_t feature,
    uint32_t cdw11, void *payload, uint32_t payload_size,
    nvme_cb_fn_t cb_fn, void *cb_arg)
{
	nvme_tracker_t *tr;
	nvme_command_t *cmd;

	tr = nvme_allocate_tracker(&ctrlr->adminq, NULL, 0, cb_fn, cb_arg);

	if (tr == NULL) {
		DTRACE_PROBE(no_resources_in_admin_qpair);
		return EAGAIN;
	}
	cmd = &tr->cmd;
	cmd->opc = NVME_OPC_GET_FEATURES;
	cmd->cdw10 = feature;
	cmd->cdw11 = cdw11;

	return nvme_qpair_submit_request(tr, SYNC);
}

int
nvme_ctrlr_cmd_set_num_queues(nvme_controller_t *ctrlr,
    uint32_t num_queues, nvme_cb_fn_t cb_fn, void *cb_arg)
{
	uint32_t cdw11;

	cdw11 = ((num_queues - 1) << 16) || (num_queues - 1);
	return nvme_ctrlr_cmd_set_feature(ctrlr, NVME_FEAT_NUMBER_OF_QUEUES,
						cdw11, NULL, 0, cb_fn, cb_arg);
}

int
nvme_ctrlr_cmd_set_interrupt_coalescing(nvme_controller_t *ctrlr,
    uint32_t microseconds, uint32_t threshold, nvme_cb_fn_t cb_fn, void *cb_arg)
{
	uint32_t cdw11;

	if ((microseconds/100) >= 0x100) {
		printf("invalid coal time %d, disabling\n", microseconds);
		microseconds = 0;
		threshold = 0;
	}

	if (threshold >= 0x100) {
		printf("invalid threshold %d, disabling\n", threshold);
		threshold = 0;
		microseconds = 0;
	}

	cdw11 = ((microseconds/100) << 8) | threshold;
	return nvme_ctrlr_cmd_set_feature(ctrlr, NVME_FEAT_INTERRUPT_COALESCING,
						 cdw11, NULL, 0, cb_fn, cb_arg);
}

int
nvme_ctrlr_cmd_asynchronous_event_request(nvme_controller_t *ctrlr,
    nvme_cb_fn_t cb_fn, void *cb_arg)
{
	nvme_tracker_t *tr;
	nvme_command_t *cmd;

	tr = nvme_allocate_tracker(&ctrlr->adminq, NULL, 0, cb_fn, cb_arg);

	if (tr == NULL) {
		DTRACE_PROBE(no_resources_in_admin_qpair);
		return EAGAIN;
	}
	cmd = &tr->cmd;
	cmd->opc = NVME_OPC_ASYNC_EVENT_REQUEST;

	return nvme_qpair_submit_request(tr, SYNC);
}
