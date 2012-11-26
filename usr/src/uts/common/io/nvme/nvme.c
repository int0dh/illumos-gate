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

#include "nvme.h"
#include "nvme_private.h"

static int nvme_blk_read(void *arg, bd_xfer_t *xfer);
static int nvme_blk_write(void *arg, bd_xfer_t *xfer); 
static void nvme_blk_driveinfo(void *, bd_drive_t *);
static int  nvme_blk_mediainfo(void *, bd_media_t *);
static int  nvme_blk_devid_init(void *, dev_info_t *, ddi_devid_t *);
static int  nvme_blk_flush(void *, bd_xfer_t *xfer);

static char nvme_ident[] = "NVMe block driver";

static bd_ops_t nvme_blk_ops =
{
	.o_version = BD_OPS_VERSION_0,
	.o_drive_info = nvme_blk_driveinfo,
	.o_media_info = nvme_blk_mediainfo,
	.o_devid_init = nvme_blk_devid_init,
	.o_sync_cache = NULL,
	.o_read = nvme_blk_read,
	.o_write = nvme_blk_write,
};

int nvme_attach(dev_info_t *dev, ddi_attach_cmd_t cmd);
int nvme_detach(dev_info_t *dev, ddi_detach_cmd_t cmd);
static int nvme_quiesce(dev_info_t *);

static struct dev_ops nvme_dev_ops =
{
	.devo_rev = DEVO_REV,
	.devo_refcnt = 0,
	.devo_getinfo = ddi_no_info,
	.devo_identify = nulldev,
	.devo_probe = nulldev,
	.devo_attach = nvme_attach,
	.devo_detach = nvme_detach,
	.devo_reset = nodev,
	.devo_cb_ops = NULL,
	.devo_bus_ops = NULL,
	.devo_power = NULL,
	.devo_quiesce = nvme_quiesce,
};

extern struct mod_ops mod_driverops;

static struct modldrv modldrv =
{
	.drv_modops = &mod_driverops,
	.drv_linkinfo = nvme_ident,
	.drv_dev_ops = &nvme_dev_ops
}; 

static struct modlinkage modlinkage =
{
	.ml_rev = MODREV_1,
	.ml_linkage[0] = (void *)&modldrv,
	.ml_linkage[1] = NULL,
};

static ddi_device_acc_attr_t nvme_dev_attr =
{
	.devacc_attr_version = DDI_DEVICE_ATTR_V0,
	.devacc_attr_endian_flags = DDI_NEVERSWAP_ACC,
	.devacc_attr_dataorder = DDI_STORECACHING_OK_ACC,
	.devacc_attr_access = DDI_DEFAULT_ACC
};

/* DMA attributes */
static ddi_dma_attr_t nvme_req_dma_attr =
{
        DMA_ATTR_V0,                    /* dma_attr version     */
        0,                              /* dma_attr_addr_lo     */
        0xFFFFFFFFFFFFFFFFull,          /* dma_attr_addr_hi     */
        0x00000000FFFFFFFFull,          /* dma_attr_count_max   */
        4096,		                     /* dma_attr_align       */
        1,                              /* dma_attr_burstsizes  */
        1,                              /* dma_attr_minxfer     */
        (128 * 1024),			/* dma_attr_maxxfer     */
        0xFFFFFFFFFFFFFFFFull,          /* dma_attr_seg         */
        1,                              /* dma_attr_sgllen      */
        1,                              /* dma_attr_granular    */
  	0,
};

static ddi_dma_attr_t nvme_bd_dma_attr =
{
        DMA_ATTR_V0,                    /* dma_attr version     */
        0,                              /* dma_attr_addr_lo     */
        0xFFFFFFFFFFFFFFFFull,          /* dma_attr_addr_hi     */
        0x00000000FFFFFFFFull,          /* dma_attr_count_max   */
        4096,		                     /* dma_attr_align       */
        1,                              /* dma_attr_burstsizes  */
        1,                              /* dma_attr_minxfer     */
        4096,				/* dma_attr_maxxfer     */
        0xFFFFFFFFFFFFFFFFull,          /* dma_attr_seg         */
        2,                              /* dma_attr_sgllen      */
        1,                              /* dma_attr_granular    */
  	0,
};

void
nvme_dump_command(struct nvme_command *cmd)
{
#if 0
	printf("opc:%lx f:%lx r1:%lx cid:%lx nsid:%lx r2:%lx r3:%lx "
	    "mptr:%qx prp1:%qx prp2:%qx cdw:%x %x %x %x %x %x\n",
	    cmd->opc, cmd->fuse, cmd->rsvd1, cmd->cid, cmd->nsid,
	    cmd->rsvd2, cmd->rsvd3,
	    (long long unsigned int)cmd->mptr,
	    (long long unsigned int)cmd->prp1,
	    (long long unsigned int)cmd->prp2,
	    cmd->cdw10, cmd->cdw11, cmd->cdw12, cmd->cdw13, cmd->cdw14,
	    cmd->cdw15);
#endif
}

void
nvme_dump_completion(struct nvme_completion *cpl)
{
#if 0
	printf("cdw0:%08x sqhd:%04x sqid:%04x "
	    "cid:%04x p:%x sc:%02x sct:%x m:%x dnr:%x\n",
	    cpl->cdw0, cpl->sqhd, cpl->sqid,
	    cpl->cid, cpl->p, cpl->sf_sc, cpl->sf_sct, cpl->sf_m,
	    cpl->sf_dnr);
#endif
}

/* IO operation completed */
static void
nvme_io_completed(void *arg, const struct nvme_completion *status, struct nvme_tracker *tr)
{
	bd_xfer_t *xfer = tr->xfer;
	struct nvme_namespace *ns = arg;

	/* release the tracker. If the operation completed with error,
	*  complete blkdev request with EIO */
	nvme_free_tracker(tr);

	ASSERT(xfer != NULL);

	if (status->sf_sc || status->sf_sct)
		bd_xfer_done(xfer, EIO);
	else
		bd_xfer_done(xfer, 0);
}
	
static int
nvme_blk_read(void *arg, bd_xfer_t *xfer)
{
	struct nvme_namespace *ns = (struct nvme_namespace *)arg;
	int ret;

	if (xfer->x_flags & BD_XFER_POLL)
		return EIO;

	if ((xfer->x_blkno + xfer->x_nblks) > nvme_ns_get_size(ns))	 
		return EINVAL;

	/* issue READ cmd, put it into the NVMe command queue */
	/* after command completiom the nvme_io_completed callback */
	/* will be called */
	ret = nvme_ns_cmd_read(ns, xfer, nvme_io_completed, ns);
	if (ret != 0)
		bd_xfer_done(xfer, ret);
	/* TODO: wait for timeout, then cancel request and return with error */
	return DDI_SUCCESS; 
}

static int
nvme_blk_write(void *arg, bd_xfer_t *xfer)
{
	struct nvme_namespace *ns = (struct nvme_namespace *)arg;
	int ret;

	if (xfer->x_flags & BD_XFER_POLL)
		return EIO;

	if ((xfer->x_blkno + xfer->x_nblks) > ns->data.nsze)	 
		return EINVAL;
	
	/* issue WRITE command and put it into NVMe command queue */
	ret = nvme_ns_cmd_write(ns, xfer, nvme_io_completed, ns);
	if (ret != 0)
		bd_xfer_done(xfer, ret);
	/* TODO: wait for timeout, then cancel request and exit with error */
	return DDI_SUCCESS; 
}

static int 
nvme_blk_flush(void *arg, bd_xfer_t *xfer)
{
	struct nvme_namespace *ns = (struct nvme_namespace *)arg;
	int ret;

	if (xfer->x_flags & BD_XFER_POLL)
		return EIO;

	ret = nvme_ns_cmd_flush(ns, xfer, nvme_io_completed, ns);
	if (ret != 0)
		bd_xfer_done(xfer, ret);

	return DDI_SUCCESS;
}

int
nvme_attach(dev_info_t *devinfo, ddi_attach_cmd_t cmd)
{
	int instance, i;
	int ret;
	struct nvme_controller *nvme = NULL;	
	size_t nvme_len = sizeof(*nvme);
	ddi_acc_handle_t dmaac;
	ddi_dma_handle_t dmah;

	instance = ddi_get_instance(devinfo);

	printf("CPU cpu is %d\n", CPU->cpu_id);
	printf("total CPUs %d\n", ncpus);

	switch (cmd)
	{
		case DDI_ATTACH:
			break;
		case DDI_RESUME:
		case DDI_PM_RESUME:
			dev_err(devinfo, CE_WARN, "resume is not supported");
			return DDI_FAILURE;
		default:
			dev_err(devinfo, CE_WARN, "unknown cmd");
			return DDI_FAILURE; 
				
	}
	/* allocate softc structure in the DMAble memory */
	if (ddi_dma_alloc_handle(devinfo,
				&nvme_req_dma_attr,
				DDI_DMA_SLEEP, NULL, &dmah) != DDI_SUCCESS)
	{
		dev_err(devinfo, CE_WARN, "cannot allocate DMA handle");
		return DDI_FAILURE;
	}
	if (ddi_dma_mem_alloc(dmah,
			sizeof(* nvme), &nvme_dev_attr,
			IOMEM_DATA_UNCACHED, DDI_DMA_SLEEP, 
			NULL, (char **)&nvme, &nvme_len, &dmaac) != DDI_SUCCESS)
	{
		dev_err(devinfo, CE_WARN, "cannot allocate DMA mem");
		goto dma_handle_free;
	}
	if (ddi_regs_map_setup(devinfo, 1, (caddr_t *)&nvme->nvme_regs_base,
				0, 0, &nvme_dev_attr, 
				&nvme->nvme_regs_handle) != DDI_SUCCESS)
	{
		dev_err(devinfo, CE_WARN, "cannot map registers");
		goto dma_mem_free;
	}

	ddi_set_driver_private(devinfo, nvme);	

	nvme->dma_handle = dmah;
	nvme->dma_acc	= dmaac;

	nvme->devinfo = devinfo;
	nvme->devattr = &nvme_dev_attr;

	ret = nvme_ctrlr_construct(nvme);
	if (ret != 0)
	{
		dev_err(devinfo, CE_WARN, "cannot construct controller!");
		goto regs_map_free;
	}
	ret = nvme_ctrlr_reset(nvme);
	if (ret != 0)
	{
		dev_err(devinfo, CE_WARN, "cannot reset controller!");
		goto regs_map_free;
	} 
	ret = nvme_ctrlr_reset(nvme);
	if (ret != 0)
	{
		dev_err(devinfo, CE_WARN, "cannot reset controller (again)");
		goto regs_map_free;
	}
	if (nvme_ctrlr_start(nvme) != 0)
	{
		dev_err(devinfo, CE_WARN, "cannot start controller");
		goto regs_map_free;
	}
	printf("%d LUNs. allocate blkdev handle\n", nvme->cdata.nn);

	for (i = 0; i < nvme->cdata.nn; i ++)
	{
		struct nvme_namespace *ns = &nvme->ns[i];

		ns->bd_handle = bd_alloc_handle(ns, &nvme_blk_ops, &nvme_bd_dma_attr, KM_SLEEP);

		ret = bd_attach_handle(devinfo, ns->bd_handle);

		if (ret != DDI_SUCCESS)
		{
			int j;
			/* Ok. detach/release already allocated blkdev handles */
			for (j = 0; j < i; j ++)
			{
				struct nvme_namespace *n = &nvme->ns[j];
				(void)bd_detach_handle(n->bd_handle);
				(void)bd_free_handle(n->bd_handle);
			}
			goto regs_map_free;
		}
	}
	return DDI_SUCCESS;
regs_map_free:
	(void)ddi_regs_map_free(&nvme->nvme_regs_handle);
dma_mem_free:
	(void)ddi_dma_mem_free(&dmaac);
dma_handle_free:
	(void)ddi_dma_free_handle(&dmah);
	return DDI_FAILURE;
}

int
nvme_detach(dev_info_t *devinfo, ddi_detach_cmd_t cmd)
{
	struct nvme_controller *nvme = NULL;
	int i = 0;
	ddi_dma_handle_t dmah;
	ddi_acc_handle_t dmaac;

	nvme = (struct nvme_controller *)ddi_get_driver_private(devinfo);

	dmah = nvme->dma_handle;
	dmaac = nvme->dma_acc;
 
	/* detach/free blkdev handles, we are going to off-line */
	for (i = 0; i < nvme->cdata.nn; i ++)
	{
		struct nvme_namespace *ns = &nvme->ns[i];

		(void)bd_detach_handle(ns->bd_handle);
		(void)bd_free_handle(ns->bd_handle);
	}
	/* destroy IO qpairs first */
	for (i = 0; i < nvme->num_io_queues; i ++)
	{
		struct nvme_qpair *q = &nvme->ioq[i];
		nvme_io_qpair_destroy(q);
	}
	/* then destroy admin qpair */	
	nvme_admin_qpair_destroy(&nvme->adminq);
	/* disable interrupts */
	nvme_interrupt_disable(nvme);
	/* unmap PCI registers */	
	(void)ddi_regs_map_free(&nvme->nvme_regs_handle);
	/* release softc */
	(void)ddi_dma_mem_free(&dmaac);
	/* release DMA handle */
	(void)ddi_dma_free_handle(&dmah);

	return DDI_SUCCESS;
}

int
nvme_quiesce(dev_info_t *dev)
{
	return -1;
}


static void
nvme_blk_driveinfo(void *arg, bd_drive_t *drive)
{
	struct nvme_namespace *ns = (struct nvme_namespace *)arg;

	ASSERT(ns->ctrlr->cdata.nn != 0);

	drive->d_maxxfer = nvme_ns_get_max_io_xfer_size(ns); 
	/* let`s assume that a few namespaces share one IO qpair */
	/* that is true in the worst case */
	drive->d_qsize = NVME_IO_ENTRIES / ns->ctrlr->cdata.nn;
	drive->d_removable = B_FALSE;
	drive->d_hotpluggable = B_FALSE;
	drive->d_target = ddi_get_instance(ns->ctrlr->devinfo); 
	drive->d_lun = ((char *)ns - (char *)&ns->ctrlr->ns[0]) / sizeof(struct nvme_namespace);
}

static int
nvme_blk_mediainfo(void *arg, bd_media_t *media)
{
	struct nvme_namespace *ns = (struct nvme_namespace *)arg;	

	media->m_nblks = nvme_ns_get_num_sectors(ns); 
	media->m_blksize = nvme_ns_get_sector_size(ns);
	media->m_readonly = B_FALSE;
	return 0;
}

int
nvme_blk_devid_init(void *arg, dev_info_t *devinfo, ddi_devid_t *devid)
{
	struct nvme_namespace *ns = (struct nvme_namespace *)arg;
	char ns_id[0x100];

	memcpy(ns_id, &ns->data, sizeof(ns_id));

	if (ddi_devid_init(devinfo, DEVID_ATA_SERIAL, sizeof(ns_id), ns_id, devid) != DDI_SUCCESS)
	{
		printf("cannot build device id!\n");
		return DDI_FAILURE;
	}
	printf("device id created!\n");
	return DDI_SUCCESS;
}

int _init(void)
{
	int rv;

	bd_mod_init(&nvme_dev_ops);

	if ((rv = mod_install(&modlinkage)) != 0) {
		printf("mod_install has returned %d\n", rv);
		bd_mod_fini(&nvme_dev_ops);
	}
	return rv;
}

int _fini(void)
{
	if (mod_remove(&modlinkage) == 0)
	{
		bd_mod_fini(&nvme_dev_ops);
	}
	return DDI_SUCCESS;
}

int _info(struct modinfo *modinfop)
{
	return (mod_info(&modlinkage, modinfop));
}

void
nvme_payload_map(struct nvme_tracker *tr, ddi_dma_handle_t dmah, ddi_dma_cookie_t *dmac, int dmac_size, void *kaddr, size_t payload_size)
{
	/* Let`s implement Scatter/Gather. Some days.. */
	ddi_dma_cookie_t cookie[NVME_MAX_PRP_LIST_ENTRIES];
	uint_t cookie_count, cur_nseg;
	int res;
	/*
	 * Note that we specified PAGESIZE for alignment and max
	 *  segment size when creating the bus dma tags.  So here
	 *  we can safely just transfer each segment to its
	 *  associated PRP entry.
	 */
	ASSERT(tr != NULL);
	ASSERT(tr->qpair != NULL);

	/* we do not use S/G, so only prp1 and prp2 needs to be set up */
	if (dmac)
	{
		ASSERT(dmac_size <= 2);

		tr->cmd.prp1 = dmac->dmac_laddress;
		if (dmac_size > 1)
			tr->cmd.prp2 = dmac[1].dmac_laddress;
	}
	else
	{
		res = ddi_dma_addr_bind_handle(dmah, (struct as *)NULL,
						 (caddr_t)kaddr, payload_size,
						DDI_DMA_RDWR | DDI_DMA_CONSISTENT,
						DDI_DMA_DONTWAIT, 0, cookie,
						&cookie_count);
		switch (res)
		{
			case DDI_DMA_MAPPED:
				break;
			default:
				/* shall we be more delicate here ? */
				panic("nvme_payload_map: cannot map DMA");
		}
		ASSERT(cookie_count <= 2);

		tr->cmd.prp1 = cookie[0].dmac_laddress;

		if (cookie_count > 1)
			tr->cmd.prp2 = cookie[1].dmac_laddress;
	}
}
