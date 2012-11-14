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

static int nvme_blk_read(void *arg, bd_xfer_t *xfer);
static int nvme_blk_write(void *arg, bd_xfer_t *xfer); 
static void nvme_blk_driveinfo(void *, bd_drive_t *);
static int  nvme_blk_mediainfo(void *, bd_media_t *);
static int  nvme_blk_devid_init(void *, dev_info_t *, ddi_devid_t *);
static int  nvme_blk_flush(void *, bd_xfer_t *xfer);

static char nvme_ident[] = "NVMe block driver";

static bd_ops_t nvme_blk_ops = {
	.o_version = BD_OPS_VERSION_0,
	.o_drive_info = nvme_blk_driveinfo,
	.o_media_info = nvme_blk_mediainfo,
	.o_devid_init = nvme_blk_devid_init,
	.o_sync_cache = nvme_blk_flush,
	.o_read = nvme_blk_read,
	.o_write = nvme_blk_write,
};

static int nvme_attach(dev_info_t *dev, ddi_attach_cmd_t cmd);
static int nvme_detach(dev_info_t *dev, ddi_detach_cmd_t cmd);
static int nvme_quiesce(dev_info_t *);

static struct dev_ops nvme_dev_ops = {
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

static struct modldrv modldrv = {
	.drv_modops = &mod_driverops,
	.drv_linkinfo = nvme_ident,
	.drv_dev_ops = &nvme_dev_ops
}; 

static struct modlinkage modlinkage = {
	.ml_rev = MODREV_1,
	.ml_linkage[0] = (void *)&modldrv,
	.ml_linkage[1] = NULL,
};

ddi_device_acc_attr_t nvme_dev_attr = {
	DDI_DEVICE_ATTR_V0,
	DDI_NEVERSWAP_ACC,
	DDI_STORECACHING_OK_ACC,
	DDI_DEFAULT_ACC
};

/* DMA attributes */
static ddi_dma_attr_t nvme_req_dma_attr = {
        DMA_ATTR_V0,                    /* dma_attr version     */
        0,                              /* dma_attr_addr_lo     */
        0xFFFFFFFFFFFFFFFFull,          /* dma_attr_addr_hi     */
        0x00000000FFFFFFFFull,          /* dma_attr_count_max   */
        4096,		                     /* dma_attr_align       */
        1,                              /* dma_attr_burstsizes  */
        1,                              /* dma_attr_minxfer     */
        0xFFFFFFFFull,                  /* dma_attr_maxxfer     */
        0xFFFFFFFFFFFFFFFFull,          /* dma_attr_seg         */
        1,                              /* dma_attr_sgllen      */
        1,                              /* dma_attr_granular    */
  	0,
};

/* DMA attributes for the data blocks */
static ddi_dma_attr_t nvme_bd_dma_attr = {
        DMA_ATTR_V0,                    /* dma_attr version     */
        0,                              /* dma_attr_addr_lo     */
        0xFFFFFFFFFFFFFFFFull,          /* dma_attr_addr_hi     */
        0x00000000FFFFFFFFull,          /* dma_attr_count_max   */
        1,                              /* dma_attr_align       */
        1,                              /* dma_attr_burstsizes  */
        1,                              /* dma_attr_minxfer     */
        0,                              /* dma_attr_maxxfer, set in attach */
        0xFFFFFFFFFFFFFFFFull,          /* dma_attr_seg         */
        0,                              /* dma_attr_sgllen, set in attach */
        1,                              /* dma_attr_granular    */
        0,                              /* dma_attr_flags       */
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

static int
nvme_blk_read(void *arg, bd_xfer_t *xfer)
{
	struct nvme_namespace *ns = (struct nvme_namespace *)arg;
	struct nvme_request *req;
	struct nvme_command *cmd;

	int npages = 0;
	caddr_t data_start;
	int data_len, i;

	printf("block read called!\n");
	return -1;

	if (xfer->x_flags & BD_XFER_POLL)
	{
		printf("%s: polling mode is not supported\n", __FUNCTION__);
		return EINVAL;
	}
	if ((xfer->x_blkno + xfer->x_nblks) > nvme_ns_get_size(ns))	 
		return EINVAL;

	data_len = xfer->x_nblks * nvme_ns_get_sector_size(ns);

	npages = (data_len) / PAGESIZE;
	/* shall we setup some DMA ? */
#if 0
	if (xfer->x_ndmac)
		data_start = (caddr_t)xfer->x_dmac.dmac_laddress;
	else
#endif
		data_start = (caddr_t)xfer->x_kaddr;
		
	/* it seems NVMe cannot handle more than PAGE_SIZE per I/O request ._. */
	for (i = 0; i < npages; i ++)
	{
		int transfer_size = min(PAGESIZE, data_len);
		uint64_t lba;

		req = nvme_allocate_request(&ns->ctrlr->ioq[0], data_start, transfer_size, NULL, NULL);

		cmd = &req->cmd;
		cmd->opc = NVME_OPC_READ;
		cmd->nsid = ns->id; 
		lba = xfer->x_blkno / nvme_ns_get_sector_size(ns);

		*(uint64_t *)&cmd->cdw10 = lba;
		cmd->cdw12 = transfer_size;

		nvme_ctrlr_submit_io_request(ns->ctrlr, req);

		data_len -= transfer_size; 
	}	
	printf("%s: called!\n", __FUNCTION__);
	return DDI_SUCCESS; 
}

static int
nvme_blk_write(void *arg, bd_xfer_t *xfer)
{
	struct nvme_namespace *ns = (struct nvme_namespace *)arg;
	struct nvme_request *req;
	struct nvme_command *cmd;

	int npages = 0;
	caddr_t data_start;
	int data_len, i;

	printf("blk write called!\n");
	return -1;

	if (xfer->x_flags & BD_XFER_POLL)
	{
		printf("%s: polling mode is not supported\n", __FUNCTION__);
		return EINVAL;
	}
	if ((xfer->x_blkno + xfer->x_nblks) > ns->data.nsze)	 
		return EINVAL;

	data_len = xfer->x_nblks * nvme_ns_get_sector_size(ns);

	npages = (data_len) / PAGESIZE;
	/* shall we setup some DMA ? */
#if 0
	if (xfer->x_ndmac)
		data_start = (caddr_t)xfer->x_dmac.dmac_laddress;
	else
#endif
		data_start = (caddr_t)xfer->x_kaddr;
		
	/* it seems NVMe cannot handle more than PAGE_SIZE per I/O request ._. */
	for (i = 0; i < npages; i ++)
	{
		int transfer_size = min(PAGESIZE, data_len);
		uint64_t lba;

		req = nvme_allocate_request(&ns->ctrlr->ioq[0], data_start, transfer_size, NULL, NULL);

		cmd = &req->cmd;
		cmd->opc = NVME_OPC_WRITE;
		cmd->nsid = ns->id;
		lba = xfer->x_blkno / nvme_ns_get_sector_size(ns);

		*(uint64_t *)&cmd->cdw10 = lba;
		cmd->cdw12 = transfer_size;

		nvme_ctrlr_submit_io_request(ns->ctrlr, req);

		data_len -= transfer_size; 
	}	
	return DDI_SUCCESS; 
}

static int
nvme_attach(dev_info_t *devinfo, ddi_attach_cmd_t cmd)
{
	int ret = DDI_SUCCESS;
	int instance, i;
	struct nvme_controller *nvme = NULL;	
	size_t nvme_len = sizeof(*nvme);

	ddi_acc_handle_t dmaac;
	ddi_dma_handle_t dmah;

	instance = ddi_get_instance(devinfo);
	printf("nvme_attach is called!, rev 0.01\n");

	switch (cmd)
	{
		case DDI_ATTACH:
			break;
		case DDI_RESUME:
		case DDI_PM_RESUME:
			dev_err(devinfo, CE_WARN, "resume is not supported");
			ret = DDI_FAILURE;
			goto exit;
		default:
			dev_err(devinfo, CE_WARN, "unknown cmd");
	
	}
	/* allocate softc structure in the DMAble memory */
	(void)ddi_dma_alloc_handle(devinfo, &nvme_req_dma_attr, DDI_DMA_SLEEP, NULL, &dmah);
	ddi_dma_mem_alloc(dmah, sizeof(* nvme), &nvme_dev_attr, IOMEM_DATA_UNCACHED, DDI_DMA_SLEEP, NULL, (char **)&nvme, &nvme_len, &dmaac);

	printf("nvme softc at %p\n", nvme);		
	ddi_set_driver_private(devinfo, nvme);	

	nvme->dma_handle = dmah;

	if (ddi_regs_map_setup(devinfo, 1, (caddr_t *)&nvme->nvme_regs_base,
		0, 0, &nvme_dev_attr, &nvme->nvme_regs_handle) != DDI_SUCCESS)
	{
		printf("cannot map BAR register\n");
		return DDI_FAILURE;
	}
	printf("BAR register mapped ok!\n");
	printf("nvme version register is 0x%08x\n", nvme_mmio_read_4(nvme, vs));

	nvme->devinfo = devinfo;

	ret = nvme_ctrlr_construct(nvme);
	if (ret != 0)
	{
		printf("cannot initialize controller!\n");
		return ENXIO;
	}
	ret = nvme_ctrlr_reset(nvme);
	/* TODO: release resources after each failure */
	if (ret != 0)
	{
		printf("cannot reset controller!\n");
		return ENXIO;
	} 
	ret = nvme_ctrlr_reset(nvme);
	if (ret != 0)
	{
		printf("cannot reset controller!\n");
		return ENXIO;
	}
	if (nvme_ctrlr_start(nvme) != 0)
	{
		printf("cannot start controller!\n");
		return ENXIO;
	}
	printf("%d LUNs. allocate blkdev handle\n", nvme->cdata.nn);

	for (i = 0; i < nvme->cdata.nn; i ++)
	{
		struct nvme_namespace *ns = &nvme->ns[i];

		printf("NS at 0x%p\n", ns);

		ns->bd_handle = bd_alloc_handle(ns, &nvme_blk_ops, &nvme_req_dma_attr, KM_SLEEP);

		ret = bd_attach_handle(devinfo, ns->bd_handle);
		if (ret != DDI_SUCCESS)
		{
			/* FIXME: shall we free some resources ? */
			printf("failed to attach blkdev :((\n");
		}
	}
exit:
	return ret;
}

static int
nvme_detach(dev_info_t *devinfo, ddi_detach_cmd_t cmd)
{
	struct nvme_controller *nvme = NULL;
	int i = 0;
	nvme = (struct nvme_controller *)ddi_get_driver_private(devinfo);

	for (i = 0; i < nvme->cdata.nn; i ++)
	{
		struct nvme_namespace *ns = &nvme->ns[i];

		if (bd_detach_handle(ns->bd_handle) != DDI_SUCCESS)
		{
			printf("cannot detach blkdev handle :(\n");
			return DDI_FAILURE;
		}
	}
	return DDI_SUCCESS;
}

static int
nvme_quiesce(dev_info_t *dev)
{
	return -1;
}

static int 
nvme_blk_flush(void *arg, bd_xfer_t *xfer)
{
	return 0;
}

static void
nvme_blk_driveinfo(void *arg, bd_drive_t *drive)
{
	struct nvme_namespace *ns = (struct nvme_namespace *)arg;

	printf("%s: called!\n", __FUNCTION__);

	/* FIXME!! */
	drive->d_maxxfer = nvme_ns_get_max_io_xfer_size(ns);
	drive->d_qsize = 2;
	drive->d_removable = B_FALSE;
	drive->d_hotpluggable = B_FALSE;
	drive->d_target = 0;
	drive->d_lun = ((char *)ns - (char *)&ns->ctrlr->ns[0]) / sizeof(struct nvme_namespace);

	/* align maxxfer down to page size */
	drive->d_maxxfer = (drive->d_maxxfer / PAGESIZE) * PAGESIZE;

	printf("drive info: maxxfer size %d\n", drive->d_maxxfer);
}

static int
nvme_blk_mediainfo(void *arg, bd_media_t *media)
{
	struct nvme_namespace *ns = (struct nvme_namespace *)arg;	

	media->m_nblks = nvme_ns_get_num_sectors(ns); 
	media->m_blksize = nvme_ns_get_sector_size(ns);
	media->m_readonly = B_FALSE;

	printf("media info: blksize %d nblks %ld\n", media->m_blksize, (long)media->m_nblks);
	return 0;
}

static int
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
nvme_payload_map(struct nvme_tracker *tr, ddi_dma_handle_t dmah, void *kaddr, size_t payload_size)
{
	ddi_dma_cookie_t cookie[NVME_MAX_PRP_LIST_ENTRIES];
	uint_t cookie_count, cur_nseg;
	/*
	 * Note that we specified PAGE_SIZE for alignment and max
	 *  segment size when creating the bus dma tags.  So here
	 *  we can safely just transfer each segment to its
	 *  associated PRP entry.
	 */
	if (tr->qpair == NULL)
	{
		printf("%s: wrong tracker (qpair == NULL)\n", __FUNCTION__);
		return;
	}
	/* FIXME!! check status!! */
	(void)ddi_dma_addr_bind_handle(dmah, (struct as *)NULL, 
		(caddr_t)kaddr, payload_size, DDI_DMA_RDWR | DDI_DMA_CONSISTENT, DDI_DMA_DONTWAIT, 0, cookie, &cookie_count);

	tr->req->cmd.prp1 = cookie[0].dmac_laddress;

	if (cookie_count == 2)
	{
		tr->req->cmd.prp1 = cookie[1].dmac_laddress;
	} 
	else if (cookie_count > 2)
	{
		cur_nseg = 1;
		tr->req->cmd.prp2 = (uint64_t)tr->prp_bus_addr;	
		while (cur_nseg < cookie_count)
		{
			tr->prp[cur_nseg - 1] = cookie[cur_nseg].dmac_laddress;
			cur_nseg ++;
		}
	} 
			
}
#if 0
#include "nvme_private.h"

struct nvme_consumer {
	nvme_consumer_cb_fn_t		cb_fn;
	void				*cb_arg;
};

struct nvme_consumer nvme_consumer[NVME_MAX_CONSUMERS];

uma_zone_t nvme_request_zone;

MALLOC_DEFINE(M_NVME, "nvme", "nvme(4) memory allocations");

static int    nvme_probe(device_t);
static int    nvme_attach(device_t);
static int    nvme_detach(device_t);

static devclass_t nvme_devclass;

static device_method_t nvme_pci_methods[] = {
	/* Device interface */
	DEVMETHOD(device_probe,     nvme_probe),
	DEVMETHOD(device_attach,    nvme_attach),
	DEVMETHOD(device_detach,    nvme_detach),
	{ 0, 0 }
};

static driver_t nvme_pci_driver = {
	"nvme",
	nvme_pci_methods,
	sizeof(struct nvme_controller),
};

DRIVER_MODULE(nvme, pci, nvme_pci_driver, nvme_devclass, 0, 0);
MODULE_VERSION(nvme, 1);

static struct _pcsid
{
	u_int32_t   type;
	const char  *desc;
} pci_ids[] = {
	{ 0x01118086,		"NVMe Controller"  },
	{ CHATHAM_PCI_ID,	"Chatham Prototype NVMe Controller"  },
	{ IDT_PCI_ID,		"IDT NVMe Controller"  },
	{ 0x00000000,		NULL  }
};

static int
nvme_probe (device_t device)
{
	struct _pcsid	*ep;
	u_int32_t	type;

	type = pci_get_devid(device);
	ep = pci_ids;

	while (ep->type && ep->type != type)
		++ep;

	if (ep->desc) {
		device_set_desc(device, ep->desc);
		return (BUS_PROBE_DEFAULT);
	}

#if defined(PCIS_STORAGE_NVM)
	if (pci_get_class(device)    == PCIC_STORAGE &&
	    pci_get_subclass(device) == PCIS_STORAGE_NVM &&
	    pci_get_progif(device)   == PCIP_STORAGE_NVM_ENTERPRISE_NVMHCI_1_0) {
		device_set_desc(device, "Generic NVMe Device");
		return (BUS_PROBE_GENERIC);
	}
#endif

	return (ENXIO);
}

static void
nvme_init(void)
{
	nvme_request_zone = uma_zcreate("nvme_request",
	    sizeof(struct nvme_request), NULL, NULL, NULL, NULL, 0, 0);
}

SYSINIT(nvme_register, SI_SUB_DRIVERS, SI_ORDER_SECOND, nvme_init, NULL);

static void
nvme_uninit(void)
{
	uma_zdestroy(nvme_request_zone);
}

SYSUNINIT(nvme_unregister, SI_SUB_DRIVERS, SI_ORDER_SECOND, nvme_uninit, NULL);

static void
nvme_load(void)
{
}

static void
nvme_unload(void)
{
}

static void
nvme_shutdown(void)
{
	device_t		*devlist;
	struct nvme_controller	*ctrlr;
	union cc_register	cc;
	union csts_register	csts;
	int			dev, devcount;

	if (devclass_get_devices(nvme_devclass, &devlist, &devcount))
		return;

	for (dev = 0; dev < devcount; dev++) {
		/*
		 * Only notify controller of shutdown when a real shutdown is
		 *  in process, not when a module unload occurs.  It seems at
		 *  least some controllers (Chatham at least) don't let you
		 *  re-enable the controller after shutdown notification has
		 *  been received.
		 */
		ctrlr = DEVICE2SOFTC(devlist[dev]);
		cc.raw = nvme_mmio_read_4(ctrlr, cc);
		cc.bits.shn = NVME_SHN_NORMAL;
		nvme_mmio_write_4(ctrlr, cc, cc.raw);
		csts.raw = nvme_mmio_read_4(ctrlr, csts);
		while (csts.bits.shst != NVME_SHST_COMPLETE) {
			DELAY(5);
			csts.raw = nvme_mmio_read_4(ctrlr, csts);
		}
	}

	free(devlist, M_TEMP);
}

static int
nvme_modevent(module_t mod, int type, void *arg)
{

	switch (type) {
	case MOD_LOAD:
		nvme_load();
		break;
	case MOD_UNLOAD:
		nvme_unload();
		break;
	case MOD_SHUTDOWN:
		nvme_shutdown();
		break;
	default:
		break;
	}

	return (0);
}

moduledata_t nvme_mod = {
	"nvme",
	(modeventhand_t)nvme_modevent,
	0
};

DECLARE_MODULE(nvme, nvme_mod, SI_SUB_DRIVERS, SI_ORDER_FIRST);



static int
nvme_attach(device_t dev)
{
	struct nvme_controller	*ctrlr = DEVICE2SOFTC(dev);
	int			status;

	status = nvme_ctrlr_construct(ctrlr, dev);

	if (status != 0)
		return (status);

	/*
	 * Reset controller twice to ensure we do a transition from cc.en==1
	 *  to cc.en==0.  This is because we don't really know what status
	 *  the controller was left in when boot handed off to OS.
	 */
	status = nvme_ctrlr_reset(ctrlr);
	if (status != 0)
		return (status);

	status = nvme_ctrlr_reset(ctrlr);
	if (status != 0)
		return (status);

	ctrlr->config_hook.ich_func = nvme_ctrlr_start;
	ctrlr->config_hook.ich_arg = ctrlr;

	config_intrhook_establish(&ctrlr->config_hook);

	return (0);
}

static int
nvme_detach (device_t dev)
{
	struct nvme_controller	*ctrlr = DEVICE2SOFTC(dev);
	struct nvme_namespace	*ns;
	int			i;

	if (ctrlr->taskqueue) {
		taskqueue_drain(ctrlr->taskqueue, &ctrlr->task);
		taskqueue_free(ctrlr->taskqueue);
	}

	for (i = 0; i < NVME_MAX_NAMESPACES; i++) {
		ns = &ctrlr->ns[i];
		if (ns->cdev)
			destroy_dev(ns->cdev);
	}

	if (ctrlr->cdev)
		destroy_dev(ctrlr->cdev);

	for (i = 0; i < ctrlr->num_io_queues; i++) {
		nvme_io_qpair_destroy(&ctrlr->ioq[i]);
	}

	free(ctrlr->ioq, M_NVME);

	nvme_admin_qpair_destroy(&ctrlr->adminq);

	if (ctrlr->resource != NULL) {
		bus_release_resource(dev, SYS_RES_MEMORY,
		    ctrlr->resource_id, ctrlr->resource);
	}

#ifdef CHATHAM2
	if (ctrlr->chatham_resource != NULL) {
		bus_release_resource(dev, SYS_RES_MEMORY,
		    ctrlr->chatham_resource_id, ctrlr->chatham_resource);
	}
#endif

	if (ctrlr->tag)
		bus_teardown_intr(ctrlr->dev, ctrlr->res, ctrlr->tag);

	if (ctrlr->res)
		bus_release_resource(ctrlr->dev, SYS_RES_IRQ,
		    rman_get_rid(ctrlr->res), ctrlr->res);

	if (ctrlr->msix_enabled)
		pci_release_msi(dev);

	return (0);
}

static void
nvme_notify_consumer(struct nvme_consumer *consumer)
{
	device_t		*devlist;
	struct nvme_controller	*ctrlr;
	int			dev, ns, devcount;

	if (devclass_get_devices(nvme_devclass, &devlist, &devcount))
		return;

	for (dev = 0; dev < devcount; dev++) {
		ctrlr = DEVICE2SOFTC(devlist[dev]);
		for (ns = 0; ns < ctrlr->cdata.nn; ns++)
			(*consumer->cb_fn)(consumer->cb_arg, &ctrlr->ns[ns]);
	}

	free(devlist, M_TEMP);
}

struct nvme_consumer *
nvme_register_consumer(nvme_consumer_cb_fn_t cb_fn, void *cb_arg)
{
	int i;

	/*
	 * TODO: add locking around consumer registration.  Not an issue
	 *  right now since we only have one nvme consumer - nvd(4).
	 */
	for (i = 0; i < NVME_MAX_CONSUMERS; i++)
		if (nvme_consumer[i].cb_fn == NULL) {
			nvme_consumer[i].cb_fn = cb_fn;
			nvme_consumer[i].cb_arg = cb_arg;

			nvme_notify_consumer(&nvme_consumer[i]);
			return (&nvme_consumer[i]);
		}

	printf("nvme(4): consumer not registered - no slots available\n");
	return (NULL);
}

void
nvme_unregister_consumer(struct nvme_consumer *consumer)
{

	consumer->cb_fn = NULL;
	consumer->cb_arg = NULL;
}
#endif
