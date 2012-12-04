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


#include "nvme_private.h"

uint32_t
nvme_ns_get_max_io_xfer_size(struct nvme_namespace *ns)
{
	return ns->ctrlr->max_xfer_size;
}

uint32_t
nvme_ns_get_sector_size(struct nvme_namespace *ns)
{
	return (1 << ns->data.lbaf[0].lbads);
}

uint64_t
nvme_ns_get_num_sectors(struct nvme_namespace *ns)
{
	return (ns->data.nsze);
}

uint64_t
nvme_ns_get_size(struct nvme_namespace *ns)
{
	return (nvme_ns_get_num_sectors(ns) * nvme_ns_get_sector_size(ns));
}

uint32_t
nvme_ns_get_flags(struct nvme_namespace *ns)
{
	return (ns->flags);
}

const char *
nvme_ns_get_serial_number(struct nvme_namespace *ns)
{
	return ((const char *)ns->ctrlr->cdata.sn);
}

const char *
nvme_ns_get_model_number(struct nvme_namespace *ns)
{
	return ((const char *)ns->ctrlr->cdata.mn);
}

int
nvme_ns_construct(struct nvme_namespace *ns, uint16_t id,
    struct nvme_controller *ctrlr)
{
	struct nvme_completion	cpl;
	int			status;
	int ns_data_offset;

	ns->ctrlr = ctrlr;
	ns->id = id;

	ns_data_offset = ctrlr->ns_data_phys + sizeof(struct nvme_namespace) * (id - 1) + offsetof(struct nvme_namespace, data);

	nvme_ctrlr_cmd_identify_namespace(ctrlr, id, ns_data_offset,
		    nvme_admin_cb, &cpl);

	if (cpl.sf_sc || cpl.sf_sct)
	{
		printf("nvme_identify_namespace failed!\n");
		return (ENXIO);
	}

	if (ctrlr->cdata.oncs.dsm && ns->data.nsfeat.thin_prov)
		ns->flags |= NVME_NS_DEALLOCATE_SUPPORTED;

	if (ctrlr->cdata.vwc.present)
		ns->flags |= NVME_NS_FLUSH_SUPPORTED;

	return (0);
}
