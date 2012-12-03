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
#include <sys/cpuvar.h>

#include "nvme_private.h"

int
nvme_ns_start_io(struct nvme_namespace *ns, bd_xfer_t *xfer,
		 nvme_cb_fn_t cb_fn, void *cb_arg,
		 enum nvme_nvm_opcode cmd_code)
{
	struct nvme_tracker *tr;
	struct nvme_command *cmd;
	struct nvme_qpair *q;
	int ret;
	int blk_size = nvme_ns_get_sector_size(ns);

	/* when MSI/MSI-X is enabled, we use one IO qpair per CPU */
	if (ns->ctrlr->msix_enabled == B_TRUE)
	{
		ASSERT(ns->ctrlr->num_io_queues > CPU->cpu_id);
		q = &ns->ctrlr->ioq[CPU->cpu_id];
	}
	else
		q = &ns->ctrlr->ioq[0];

	tr = nvme_allocate_tracker(q, NULL, xfer->x_nblks * blk_size, cb_fn, cb_arg);

	/* no resources in the IO qpair */
	if (tr == NULL)
		return (EAGAIN);

	tr->xfer = xfer;

	cmd = &tr->cmd;
	cmd->opc = cmd_code;
	cmd->nsid = ns->id;

	if (cmd_code != NVME_OPC_FLUSH)
	{
		*(uint64_t *)&cmd->cdw10 = xfer->x_blkno;
		cmd->cdw12 = (xfer->x_dmac.dmac_size / blk_size) - 1;
	}
	ret = nvme_ctrlr_submit_io_request(ns->ctrlr, tr);

	if (xfer->x_flags & BD_XFER_POLL)
		ret = nvme_wait_for_completion(q, tr);

	return ret;
}
