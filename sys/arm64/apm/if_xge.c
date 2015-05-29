/*-
 * Copyright (c) 2015 AppliedMicro Inc
 * All rights reserved.
 *
 * Developed by Semihalf.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
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

#include <sys/cdefs.h>
__FBSDID("$FreeBSD$");

#include <sys/param.h>
#include <sys/systm.h>
#include <sys/rman.h>
#include <sys/endian.h>
#include <sys/bus.h>
#include <sys/sockio.h>
#include <sys/mbuf.h>
#include <sys/malloc.h>
#include <sys/module.h>
#include <sys/kernel.h>
#include <sys/socket.h>
#include <sys/lock.h>
#include <sys/mutex.h>
#include <sys/sysctl.h>
#include <sys/taskqueue.h>

#include <net/bpf.h>
#include <net/ethernet.h>
#include <net/if.h>
#include <net/if_var.h>
#include <net/if_arp.h>
#include <net/if_dl.h>
#include <net/if_media.h>
#include <net/if_types.h>
#include <net/if_vlan_var.h>

#include <netinet/in.h>
#include <netinet/in_systm.h>
#include <netinet/if_ether.h>
#include <netinet/ip.h>
#include <netinet/tcp.h>

#include <machine/bus.h>
#include <machine/resource.h>

#include <dev/mii/mii.h>
#include <dev/mii/miivar.h>

#include <dev/ofw/ofw_bus.h>
#include <dev/ofw/ofw_bus_subr.h>

#include <vm/pmap.h>

#include "if_xge_var.h"

#define	RES_ENET_CSR	0	/* (M)ENET registers */
#define	RES_RING_CSR	1	/* QM(Lite) registers */
#define	RES_RING_CMD	2	/* QM(Lite) fabric area */
#define	RES_QMDEQ_IRQ	0	/* QM(Lite) dequeue interrupt */

#define	XGE_GLOBAL_LOCK_INIT(sc)				\
    mtx_init(&(sc)->globl_mtx, device_get_nameunit((sc)->dev),	\
	MTX_NETWORK_LOCK, MTX_DEF)

#define	XGE_GLOBAL_LOCK_DESTROY(sc)				\
    mtx_destroy(&(sc)->globl_mtx)

#define	XGE_GLOBAL_LOCK(sc)		mtx_lock(&(sc)->globl_mtx)
#define	XGE_GLOBAL_UNLOCK(sc)	mtx_unlock(&(sc)->globl_mtx)

#define	XGE_GLOBAL_LOCK_ASSERT(sc)				\
    mtx_assert(&(sc)->globl_mtx, MA_OWNED)

#define	XGE_RING_DESC_ALIGNMENT		256	/* Has to be 256 bytes */
#define	XGE_RING_DESC_NSEGMENTS		1	/* Single segment */

#define	XGE_BUFF_ALIGNMENT		MCLBYTES
#define	XGE_BUFF_NSEGMENTS		1

#define	XGE_WD_TIMEOUT			5

MALLOC_DEFINE(M_XGE, "xge", "X-Gene ENET dynamic memory");

static void xge_init_locked(struct xge_softc *);
static void xge_stop_locked(struct xge_softc *);
static void xge_txstart_locked(struct xge_softc *);

static int xge_ioctl(struct ifnet *, u_long, caddr_t);
static void xge_init(void *);
static void xge_txstart(struct ifnet *);

static void xge_watchdog(struct xge_softc *);
static void xge_tick(void *);

static int xge_media_change_locked(struct xge_softc *);

static int xge_media_change(struct ifnet *);
static void xge_media_status(struct ifnet *, struct ifmediareq *);

static void xge_setup_ops(struct xge_softc *);
static int xge_init_hw(struct xge_softc *);

static int xge_new_bufpool(struct xgene_enet_desc_ring *, uint32_t);

/* Get initial MAC address */
static void
xge_acquire_macaddr(struct xge_softc *sc, uint8_t *hwaddr)
{
	uint32_t rnd;
	uint8_t zeromac[ETHER_ADDR_LEN] = { [0 ... (ETHER_ADDR_LEN - 1)] = 0 };

	/*
	 * XXX: Try to use default address taken from FDT,
	 *      in case of missing default value don't read
	 *      from hardware and set 'bsd' + random 24 low-order bits.
	 */
	if (memcmp(zeromac, sc->hwaddr, ETHER_ADDR_LEN) == 0) {
		rnd = arc4random() & 0x00ffffff;
		hwaddr[0] = 'b';
		hwaddr[1] = 's';
		hwaddr[2] = 'd';
		hwaddr[3] = rnd >> 16;
		hwaddr[4] = rnd >> 8;
		hwaddr[5] = rnd >> 0;
		/* Save current MAC address to context */
		memcpy(sc->hwaddr, hwaddr, ETHER_ADDR_LEN);
	} else
		memcpy(hwaddr, sc->hwaddr, ETHER_ADDR_LEN);
}

static __inline void
xge_mac_reset(struct xge_softc *sc)
{
	struct xgene_enet_pdata *pdata = &sc->pdata;

	pdata->mac_ops->reset(pdata);
}

static __inline uint16_t
xge_get_ring_id(enum xgene_ring_owner owner, uint8_t bufnum)
{

	return ((owner << 6) | (bufnum & GENMASK(5, 0)));
}

static size_t
xge_get_ring_size(enum xgene_enet_ring_cfgsize cfgsize)
{
	size_t size;

	switch (cfgsize) {
	case RING_CFGSIZE_512B:
		size = 0x200;
		break;
	case RING_CFGSIZE_2KB:
		size = 0x800;
		break;
	case RING_CFGSIZE_16KB:
		size = 0x4000;
		break;
	case RING_CFGSIZE_64KB:
		size = 0x10000;
		break;
	case RING_CFGSIZE_512KB:
		size = 0x80000;
		break;
	default:
		size = 0;
		break;
	}

	return (size);
}

static __inline uint16_t
xge_dst_ring_num(struct xgene_enet_desc_ring *ring)
{
	struct xge_softc *sc;
	struct xgene_enet_pdata *pdata;

	sc = device_get_softc(ring->ndev);
	pdata = &sc->pdata;

	return (((uint16_t)pdata->rm << 10) | ring->num);
}

static void
xge_init_bufpool(struct xgene_enet_desc_ring *buf_pool)
{
	struct xgene_enet_raw_desc16 *raw_desc;
	size_t i;

	xge_new_bufpool(buf_pool, buf_pool->slots);

	for (i = 0; i < buf_pool->slots; i++) {
		raw_desc = &buf_pool->raw_desc16[i];

		/* Hardware expects descriptor in little endian format */
		/*
		 * XXX: Seems to be not necessary here since we don't
		 *      support BE anyway.
		 */
		raw_desc->m0 =
		    htole64(i | SET_VAL(FPQNUM, buf_pool->dst_ring_num) |
		    SET_VAL(STASH, 3));
	}

	buf_pool->tail = 0;
}

static void
xge_delete_ring(struct xgene_enet_desc_ring *ring)
{

	xgene_enet_clear_ring(ring);
	bus_dmamem_free(ring->dmat, ring->desc_addr, ring->dmap);
	bus_dma_tag_destroy(ring->dmat);
	free(ring, M_XGE);
}

static void
xge_map_dma_addr(void *arg, bus_dma_segment_t *segs, int nseg, int error)
{
	u_int32_t *paddr;

	KASSERT(nseg == 1, ("wrong number of segments, should be 1"));
	paddr = arg;
	*paddr = segs->ds_addr;
}

static struct xgene_enet_desc_ring *
xge_create_ring(struct xge_softc *sc, uint32_t ring_num, enum
    xgene_enet_ring_cfgsize cfgsize, uint32_t ring_id)
{
	struct xgene_enet_pdata *pdata;
	struct xgene_enet_desc_ring *ring;
	size_t size;
	int err;

	pdata = &sc->pdata;

	size = xge_get_ring_size(cfgsize);
	if (size == 0) {
		device_printf(sc->dev,
		    "Unsupported circular buffer size for message queue\n");
		return (NULL);
	}
	ring = malloc(sizeof(*ring), M_XGE, (M_NOWAIT | M_ZERO));
	if (ring == NULL) {
		device_printf(sc->dev,
		    "Cannot allocate memory for ring structure\n");
		return (NULL);
	}

	ring->ndev = sc->dev;
	ring->num = ring_num;
	ring->cfgsize = cfgsize;
	ring->id = ring_id;

	err = bus_dma_tag_create(
	    bus_get_dma_tag(sc->dev),		/* parent */
	    XGE_RING_DESC_ALIGNMENT,		/* alignment */
	    0,					/* boundary */
	    BUS_SPACE_MAXADDR,			/* lowaddr */
	    BUS_SPACE_MAXADDR,			/* highaddr */
	    NULL, NULL,				/* filtfunc, filtfuncarg */
	    size,				/* maxsize */
	    XGE_RING_DESC_NSEGMENTS,		/* nsegments */
	    (size / XGE_RING_DESC_NSEGMENTS),	/* maxsegsize */
	    0,					/* flags */
	    NULL, NULL,				/* lockfunc, lockfuncarg */
	    &ring->dmat);			/* dmat */
	if (err) {
		device_printf(sc->dev,
		    "Failed to allocate busdma tag for descriptors ring\n");
		goto dmatag_fail;
	}

	err = bus_dmamem_alloc(ring->dmat, &ring->desc_addr,
	    (BUS_DMA_NOWAIT | BUS_DMA_ZERO), &ring->dmap);
	if (err) {
		device_printf(sc->dev,
		    "Failed to allocate DMA safe memory for "
		    "descriptors ring\n");
		goto dmamem_fail;
	}

	err = bus_dmamap_load(ring->dmat, ring->dmap, ring->desc_addr, size,
	    xge_map_dma_addr, &ring->dma, BUS_DMA_NOWAIT);
	if (err) {
		device_printf(sc->dev,
		    "Cannot get physical address of descriptors ring\n");
		goto dmamap_fail;
	}

	ring->size = size;
	/* To be used as offsets from ring_cmd_addr resource */
	ring->cmd_base = (ring->num << 6);
	ring->cmd = ring->cmd_base + INC_DEC_CMD_ADDR;

	ring = xgene_enet_setup_ring(ring);

	if (bootverbose) {
		device_printf(sc->dev,
		    "ring info: num: %d, size: %d, id: %d, slots: %d\n",
		    ring->num, ring->size, ring->id, ring->slots);
	}

	return (ring);

dmamap_fail:
	bus_dmamem_free(ring->dmat, ring->desc_addr, ring->dmap);
dmamem_fail:
	bus_dma_tag_destroy(ring->dmat);
	ring->desc_addr = NULL;
dmatag_fail:
	free(ring, M_XGE);

	return (NULL);
}

static int
xge_create_desc_rings(struct xge_softc *sc)
{
	struct xgene_enet_pdata *pdata = &sc->pdata;
	struct xgene_enet_desc_ring *rx_ring, *tx_ring, *cp_ring;
	struct xgene_enet_desc_ring *buf_pool;
	uint16_t ring_id, ring_num;
	uint8_t cpu_bufnum, eth_bufnum, bp_bufnum;
	int err = 0;

	cpu_bufnum = 0;
	eth_bufnum = START_ETH_BUFNUM;
	bp_bufnum = START_BP_BUFNUM;
	buf_pool = NULL;

	ring_num = START_RING_NUM;

	/* Allocate Rx descriptor ring (work queue) */
	ring_id = xge_get_ring_id(RING_OWNER_CPU, cpu_bufnum);
	cpu_bufnum++;

	rx_ring = xge_create_ring(sc, ring_num, RING_CFGSIZE_16KB,
	    ring_id);
	ring_num++;

	if (rx_ring == NULL) {
		device_printf(sc->dev, "Cannot allocate Rx descriptor ring\n");
		err = ENOMEM;
		goto error;
	}
	pdata->rx_ring = rx_ring;

	/* Allocate Rx buffers pool (free queue) */
	ring_id = xge_get_ring_id(RING_OWNER_ETH0, bp_bufnum);
	bp_bufnum++;
	buf_pool = xge_create_ring(sc, ring_num, RING_CFGSIZE_2KB,
	    ring_id);
	ring_num++;

	if (buf_pool == NULL) {
		device_printf(sc->dev, "Cannot allocate buffer pool\n");
		err = ENOMEM;
		goto error;
	}

	rx_ring->nbufpool = NUM_BUFPOOL;
	rx_ring->buf_pool = buf_pool;

	buf_pool->rx_buf = malloc((buf_pool->slots * sizeof(struct xge_buff)),
	    M_XGE, (M_WAITOK | M_ZERO));

	buf_pool->dst_ring_num = xge_dst_ring_num(buf_pool);

	/* Allocate Tx descriptor ring (work queue) */
	ring_id = xge_get_ring_id(RING_OWNER_ETH0, eth_bufnum);
	eth_bufnum++;
	tx_ring = xge_create_ring(sc, ring_num, RING_CFGSIZE_16KB,
	    ring_id);
	ring_num++;

	if (tx_ring == NULL) {
		device_printf(sc->dev, "Cannot allocate Tx descriptor ring\n");
		err = ENOMEM;
		goto error;
	}
	/*
	 * Allocate DMA maps for Tx buffers.
	 * TODO: Should be moved elsewhere.
	 */
	size_t i;

	tx_ring->tx_buf = malloc((tx_ring->slots * sizeof(struct xge_buff)),
	    M_XGE, (M_WAITOK | M_ZERO));
	err = bus_dma_tag_create(
	    bus_get_dma_tag(sc->dev),		/* parent */
	    XGE_BUFF_ALIGNMENT,			/* alignment */
	    0,					/* boundary */
	    BUS_SPACE_MAXADDR,			/* lowaddr */
	    BUS_SPACE_MAXADDR,			/* highaddr */
	    NULL, NULL,				/* filtfunc, filtfuncarg */
	    MCLBYTES,				/* maxsize */
	    XGE_BUFF_NSEGMENTS,			/* nsegments */
	    MCLBYTES,				/* maxsegsize */
	    0,					/* flags */
	    NULL, NULL,				/* lockfunc, lockfuncarg */
	    &tx_ring->tx_buf_dmat);		/* dmat */

	if (err != 0) {
		device_printf(sc->dev,
		    "Failed to allocate busdma tag for Tx buffers\n");
		goto error;
	}

	for (i = 0; i < tx_ring->slots; i++) {
		err = bus_dmamap_create(tx_ring->tx_buf_dmat, 0,
		    &tx_ring->tx_buf[i].dmap);
		if (err != 0) {
			device_printf(sc->dev,
			    "Failed to create busdma map for Tx buffers\n");
			goto error;
		}
	}
	/* End of maps creation */

	tx_ring->tail = 0;
	pdata->tx_ring = tx_ring;

	cp_ring = pdata->rx_ring;
	cp_ring->cp_buf = malloc((tx_ring->slots * sizeof(struct xge_buff)),
	    M_XGE, (M_WAITOK | M_ZERO));

	pdata->tx_ring->cp_ring = cp_ring;
	pdata->tx_ring->dst_ring_num = xge_dst_ring_num(cp_ring);

	pdata->tx_qcnt_hi = pdata->tx_ring->slots / 2;
	pdata->cp_qcnt_hi = pdata->rx_ring->slots / 2;
	pdata->cp_qcnt_low = pdata->cp_qcnt_hi / 2;

	return (0);

error:
	/* TODO: Clean up */
	return (err);
}

static void
xge_rx_intr(void *arg)
{
	struct xge_softc *sc __unused = arg;

	panic("%s", __func__);
}

int
xge_attach(device_t dev)
{
	struct xge_softc *sc = device_get_softc(dev);
	struct xgene_enet_pdata *pdata = &sc->pdata;
	struct ifnet *ifp;
	uint8_t hwaddr[ETHER_ADDR_LEN];
	int err;
	int rid;

	/*
	 * XXX: Limit driver to RGMII (MENET) port only for now!
	 */
	if (sc->phy_conn_type != PHY_CONN_RGMII)
		return (ENXIO);

	sc->dev = dev;

	/* Initialize global lock */
	XGE_GLOBAL_LOCK_INIT(sc);
	/* Initialize timeout */
	callout_init_mtx(&sc->wd_callout, &sc->globl_mtx, 0);
	/* Initialize watchdog - inactive */
	sc->wd_timeout = 0;

	/* Get initial MAC address */
	xge_acquire_macaddr(sc, hwaddr);

	/*
	 * Allocate resources
	 */
	/* Ethernet control and status register address space */
	rid = RES_ENET_CSR;
	sc->enet_csr = bus_alloc_resource_any(dev, SYS_RES_MEMORY, &rid, RF_ACTIVE);
	/* Descriptor ring control and status register address space */
	rid = RES_RING_CSR;
	sc->ring_csr = bus_alloc_resource_any(dev, SYS_RES_MEMORY, &rid, RF_ACTIVE);
	/* Descriptor ring command register address space */
	rid = RES_RING_CMD;
	sc->ring_cmd = bus_alloc_resource_any(dev, SYS_RES_MEMORY, &rid, RF_ACTIVE);

	if (!sc->enet_csr || !sc->ring_csr || !sc->ring_cmd) {
		device_printf(dev, "Could not allocate resource for %s\n",
		    !sc->enet_csr ? "ENET registers" :
		    !sc->ring_csr ? "QM registers" : "QM commands");

		xge_detach(dev);
		return (ENOMEM);
	}

	rid = RES_QMDEQ_IRQ;
	sc->rx_irq = bus_alloc_resource_any(dev, SYS_RES_IRQ, &rid,
	    (RF_ACTIVE | RF_SHAREABLE));
	if (sc->rx_irq == NULL) {
		device_printf(dev,
		    "Cannot allocate QM dequeue interrupt (Rx)\n");

		xge_detach(dev);
		return (ENXIO);
	}

	/* Setup interrupts */
	err = bus_setup_intr(dev, sc->rx_irq, INTR_TYPE_NET, NULL, xge_rx_intr,
	    sc, &sc->rx_irq_ihl);
	if (err != 0) {
		device_printf(dev,
		    "Cannot set-up QM dequeue interrupt (Rx)\n");

		xge_detach(dev);
		return (ENXIO);
	}

	/*
	 * XXX: For Linux HW layer this was a reference to
	 *      Linux net device. On FreeBSD however we pass
	 *      device_t to extract any necessary context from it.
	 */
	pdata->ndev = dev;

	pdata->hwaddr = sc->hwaddr;
	pdata->rx_buff_cnt = NUM_PKT_BUF;
	pdata->base_addr = sc->enet_csr;
	pdata->ring_csr_addr = sc->ring_csr;
	pdata->ring_cmd_addr = sc->ring_cmd;
	/* Frequently used offsets from base_addr */
	pdata->mcx_mac_addr = BLOCK_ETH_CSR_OFFSET;
	pdata->eth_ring_if_addr = BLOCK_ETH_RING_IF_OFFSET;
	pdata->eth_diag_csr_addr = BLOCK_ETH_DIAG_CSR_OFFSET;
	if (sc->phy_conn_type == PHY_CONN_RGMII ||
	    sc->phy_conn_type == PHY_CONN_SGMII) {
		pdata->mcx_mac_addr = BLOCK_ETH_MAC_OFFSET;
		pdata->mcx_mac_csr_addr = BLOCK_ETH_MAC_CSR_OFFSET;
	} else {
		pdata->mcx_mac_addr = BLOCK_AXG_MAC_OFFSET;
		pdata->mcx_mac_csr_addr = BLOCK_AXG_MAC_CSR_OFFSET;
	}

	/* Setup X-Gene ENET ops */
	xge_setup_ops(sc);

	/* Configure ENET hardware */
	err = xge_init_hw(sc);
	if (err != 0) {
		xge_detach(dev);
		return (err);
	}

	/* Allocate and set up the ethernet interface. */
	sc->ifp = ifp = if_alloc(IFT_ETHER);

	ifp->if_softc = sc;
	if_initname(ifp, device_get_name(dev), device_get_unit(dev));
	ifp->if_flags = IFF_BROADCAST | IFF_SIMPLEX | IFF_MULTICAST;
	ifp->if_capabilities = IFCAP_VLAN_MTU;
	/*
	 * TODO: HW checksum
	 *       Currently disabled.
	 */
	ifp->if_capenable = ifp->if_capabilities;

#if 0
	/* XXX: This is not supported so cannot be used at the moment */
#ifdef DEVICE_POLLING
	/* Advertise that polling is supported */
	ifp->if_capabilities |= IFCAP_POLLING;
#endif
#endif
	ifp->if_mtu = ETHERMTU;

	ifp->if_init = xge_init;
	ifp->if_start = xge_txstart;
	ifp->if_ioctl = xge_ioctl;

	/* TODO: Set runtime values of those lists */
	ifp->if_snd.ifq_drv_maxlen = 511;
	IFQ_SET_MAXLEN(&ifp->if_snd, 511);
	IFQ_SET_READY(&ifp->if_snd);

	/* Attach PHYs */
	if (sc->phy_conn_type == PHY_CONN_RGMII) {
		err = mii_attach(dev, &sc->miibus, ifp, xge_media_change,
		    xge_media_status, BMSR_DEFCAPMASK, sc->phyaddr,
		    MII_OFFSET_ANY, 0);
		if (err)
			return (err);
	}

	ether_ifattach(ifp, hwaddr);

	return (0);
}

int
xge_detach(device_t dev)
{
	struct xge_softc *sc;

	sc = device_get_softc(dev);

	if (sc->enet_csr != NULL)
		bus_release_resource(dev, SYS_RES_MEMORY, 0, sc->enet_csr);
	if (sc->ring_csr != NULL)
		bus_release_resource(dev, SYS_RES_MEMORY, 1, sc->ring_csr);
	if (sc->ring_cmd != NULL)
		bus_release_resource(dev, SYS_RES_MEMORY, 2, sc->ring_cmd);

	XGE_GLOBAL_LOCK_DESTROY(sc);

	return (0);
}

static int
xge_ioctl(struct ifnet *ifp, u_long cmd, caddr_t data)
{
	struct xge_softc *sc;
	struct mii_data *mii_sc;
	struct ifreq *ifr;
	uint32_t flags;
	int mask, err;

	sc = ifp->if_softc;
	ifr = (struct ifreq *)data;

	err = 0;
	switch (cmd) {
	case SIOCSIFFLAGS:
		XGE_GLOBAL_LOCK(sc);
		if (ifp->if_flags & IFF_UP) {
			if (ifp->if_drv_flags & IFF_DRV_RUNNING) {
				flags = ifp->if_flags ^ sc->if_flags;
				/* TODO: Set-up mode */
				printf("\n%s: SIOCSIFLAGS not implemented\n",
				    __func__);
			} else
				xge_init_locked(sc);

		} else {
			if (ifp->if_drv_flags & IFF_DRV_RUNNING)
				xge_stop_locked(sc);
		}
		sc->if_flags = ifp->if_flags;
		XGE_GLOBAL_UNLOCK(sc);
		break;

	case SIOCADDMULTI:
	case SIOCDELMULTI:
		if (ifp->if_drv_flags & IFF_DRV_RUNNING) {
			XGE_GLOBAL_LOCK(sc);
			/* TODO: Implement setup_multicast() */
			XGE_GLOBAL_UNLOCK(sc);
		}
		break;

	case SIOCSIFMEDIA:
	case SIOCGIFMEDIA:
		mii_sc = device_get_softc(sc->miibus);
		err = ifmedia_ioctl(ifp, ifr, &mii_sc->mii_media, cmd);
		break;

	case SIOCSIFCAP:
		mask = ifp->if_capenable ^ ifr->ifr_reqcap;
		if (mask & IFCAP_VLAN_MTU) {
			/* No work to do except acknowledge the change took. */
			ifp->if_capenable ^= IFCAP_VLAN_MTU;
		}
		break;

	default:
		err = ether_ioctl(ifp, cmd, data);
		break;
	}

	return (err);
}

static void
xge_init(void *if_softc)
{
	struct xge_softc *sc = if_softc;

	XGE_GLOBAL_LOCK(sc);
	xge_init_locked(sc);
	XGE_GLOBAL_UNLOCK(sc);
}

static void
xge_txstart(struct ifnet *ifp)
{
	struct xge_softc *sc = ifp->if_softc;

	XGE_GLOBAL_LOCK(sc);
	xge_txstart_locked(sc);
	XGE_GLOBAL_UNLOCK(sc);
}

static void
xge_init_locked(struct xge_softc *sc)
{
	struct xgene_enet_pdata *pdata;
	struct xgene_mac_ops *mac_ops;
	struct ifnet *ifp;

	XGE_GLOBAL_LOCK_ASSERT(sc);

	pdata = &sc->pdata;
	mac_ops = pdata->mac_ops;
	ifp = sc->ifp;

	if (ifp->if_drv_flags & IFF_DRV_RUNNING)
		return;

	/* Enable Tx and Rx */
	mac_ops->tx_enable(pdata);
	mac_ops->rx_enable(pdata);

	/* Activate network interface */
	ifp->if_drv_flags |= IFF_DRV_RUNNING;
	ifp->if_drv_flags &= ~IFF_DRV_OACTIVE;

	/* Schedule watchdog timeout */
	callout_reset(&sc->wd_callout, hz, xge_tick, sc);
}

static uint32_t
xge_ring_len(struct xgene_enet_desc_ring *ring)
{
	struct xge_softc *sc;
	uint32_t ring_state, num_msgs;

	sc = device_get_softc(ring->ndev);

	/* TODO: Find better way to generate offset to the cmd_base[1] */
	ring_state =
	    RING_CMD_READ32(&sc->pdata, ring->cmd_base + sizeof(ring_state));
	num_msgs = ring_state & CREATE_MASK(NUMMSGSINQ_POS, NUMMSGSINQ_LEN);

	return (num_msgs >> NUMMSGSINQ_POS);
}

static uint64_t
xge_work_msg(struct mbuf *m0)
{
	struct ether_header *eth;
	struct ip *ip;
	struct tcphdr *tcp;
	uint8_t l3hlen, l4hlen;
	uint8_t csum_enable, proto, ethhdr;
	uint64_t hopinfo;

	l4hlen = proto = csum_enable = 0;

	eth = (struct ether_header *)m0->m_data;
	ip = (struct ip *)(m0->m_data + sizeof(struct ether_header));
	tcp = (struct tcphdr *)(ip + (ip->ip_hl << 2));

	if (ip->ip_p == IPPROTO_TCP) {
		l4hlen = tcp->th_off * 4;
		proto = TSO_IPPROTO_TCP;
		csum_enable = 1;
	} else if (ip->ip_p == IPPROTO_UDP) {
		l4hlen = UDP_HDR_SIZE;
		csum_enable = 1;
	}

	/* XXX: All of this is hardcoded and no offload features */
	l3hlen = ip->ip_hl; /* IP header length in words (4 bytes) */
	ethhdr = (eth->ether_type == htons(ETHERTYPE_VLAN)) ? 18 : ETHER_HDR_LEN;
	/* Disable HW checksum */
	csum_enable = 0;

	hopinfo = SET_VAL(TCPHDR, l4hlen) |
		  SET_VAL(IPHDR, l3hlen) |
		  SET_VAL(ETHHDR, ethhdr) |
		  SET_VAL(EC, csum_enable) |
		  SET_VAL(IS, proto) |
		  SET_BIT(IC) |
		  SET_BIT(TYPE_ETH_WORK_MESSAGE);

	return (hopinfo);
}

static void
xge_setup_tx_desc(struct xgene_enet_desc_ring *tx_ring, struct mbuf *m0)
{
	struct xgene_enet_raw_desc *raw_desc;
	bus_dma_tag_t dmat;
	bus_dmamap_t dmap;
	bus_dma_segment_t seg[1];
	uint64_t hopinfo;
	int nsegs;
	uint16_t tail;
	int err;

	tail = tx_ring->tail;

	dmat = tx_ring->tx_buf_dmat;
	/* Fetch unused dmap */
	dmap = tx_ring->tx_buf[tail].dmap;
	/* Create mapping in DMA memory */
	err = bus_dmamap_load_mbuf_sg(dmat, dmap, m0, seg, &nsegs,
	    BUS_DMA_NOWAIT);

	if ((nsegs != 1) || (err != 0)) {
		panic("%s: cannot dmamap_load mbuf, nsegs: %d, error: %d",
		    __func__, nsegs, err);
	}

	raw_desc = &tx_ring->raw_desc[tail];
	memset(raw_desc, 0, sizeof(struct xgene_enet_raw_desc));

	/* Hardware expects descriptor in little endian format */
	raw_desc->m0 = htole64(tail);
	raw_desc->m1 = htole64(SET_VAL(DATAADDR, seg[0].ds_addr) |
	    SET_VAL(BUFDATALEN, seg[0].ds_len) | SET_BIT(COHERENT));
	hopinfo = xge_work_msg(m0);
	raw_desc->m3 = htole64(SET_VAL(HENQNUM, tx_ring->dst_ring_num) |
	    hopinfo);
	tx_ring->cp_ring->cp_buf[tail].mbuf = m0;
}

static int
xge_encap(struct xge_softc *sc, struct mbuf *m0)
{
	struct xgene_enet_raw_desc *raw_desc;
	struct xgene_enet_desc_ring *tx_ring;
	struct xgene_enet_desc_ring *cp_ring;
	uint32_t tx_level, cq_level;

	tx_ring = sc->pdata.tx_ring;
	cp_ring = tx_ring->cp_ring;

	tx_level = xge_ring_len(tx_ring);
	cq_level = xge_ring_len(cp_ring);

	if (tx_level > sc->pdata.tx_qcnt_hi ||
	    cq_level > sc->pdata.cp_qcnt_hi)
		return (ENOBUFS);

	xge_setup_tx_desc(tx_ring, m0);

	RING_CMD_WRITE32(&sc->pdata, tx_ring->cmd, 1);
	raw_desc = &tx_ring->raw_desc[tx_ring->tail];

	tx_ring->tail = (tx_ring->tail + 1) & (tx_ring->slots - 1);

	return (0);
}

static void
xge_txstart_locked(struct xge_softc *sc)
{
	struct mbuf *m0, *mtmp;
	struct ifnet *ifp;
	int csum_flags;
	unsigned int queued;

	XGE_GLOBAL_LOCK_ASSERT(sc);

	ifp = sc->ifp;
	queued = 0;

	if ((ifp->if_drv_flags &
	    (IFF_DRV_RUNNING | IFF_DRV_OACTIVE)) != IFF_DRV_RUNNING)
		return;

	while (!IFQ_DRV_IS_EMPTY(&ifp->if_snd)) {
		/*
		 * Dequeue packets to the driver managed queue and
		 * return first mbuf here.
		 */
		IFQ_DRV_DEQUEUE(&ifp->if_snd, m0);
		if (m0 == NULL)
			break;

		csum_flags = m0->m_pkthdr.csum_flags;
		if (csum_flags != 0) {
			/* TODO: Use offloading features here */
		}

		mtmp = m_defrag(m0, M_NOWAIT);
		if (mtmp != NULL)
			m0 = mtmp;

		if (xge_encap(sc, m0) != 0) {
			IFQ_DRV_PREPEND(&ifp->if_snd, m0);
			ifp->if_drv_flags |= IFF_DRV_OACTIVE;
			break;
		}

		queued++;
		BPF_MTAP(ifp, m0);
	}

	if (queued) {
		/* TODO: Enable transmission */
		/* Set watchdog timeout (in hz ticks) */
		sc->wd_timeout = XGE_WD_TIMEOUT;
	}
}

static void
xge_stop_locked(struct xge_softc *sc)
{
	struct xgene_enet_pdata *pdata;
	struct xgene_mac_ops *mac_ops;
	struct ifnet *ifp;

	XGE_GLOBAL_LOCK_ASSERT(sc);

	ifp = sc->ifp;
	pdata = &sc->pdata;
	mac_ops = pdata->mac_ops;

	/* Stop tick engine */
	callout_stop(&sc->wd_callout);
	/* Clean watchdog */
	sc->wd_timeout = 0;

	/* Disable interface */
	ifp->if_drv_flags &= ~(IFF_DRV_RUNNING | IFF_DRV_OACTIVE);

	/* TODO: Disable interrupts */

	/* TODO: Process ring */

	/* Disable Rx and Tx */
	mac_ops->tx_disable(pdata);
	mac_ops->rx_disable(pdata);
}

static void
xge_watchdog(struct xge_softc *sc)
{
	struct ifnet *ifp;

	XGE_GLOBAL_LOCK_ASSERT(sc);

	if ((sc->wd_timeout == 0) || (--sc->wd_timeout > 0))
		return;

	ifp = sc->ifp;
	if_inc_counter(ifp, IFCOUNTER_OERRORS, 1);
	if_printf(ifp, "watchdog timeout\n");

	/* Restart controller */
	xge_mac_reset(sc);

	if (!IFQ_DRV_IS_EMPTY(&ifp->if_snd))
		xge_txstart_locked(sc);
}

static void
xge_tick(void *arg)
{
	struct xge_softc *sc;
	struct mii_data *mii_sc;
	struct ifnet *ifp;
	boolean_t link_was_down;

	sc = arg;
	mii_sc = device_get_softc(sc->miibus);
	ifp = sc->ifp;

	XGE_GLOBAL_LOCK_ASSERT(sc);

	xge_watchdog(sc);

	link_was_down = ((mii_sc->mii_media_status & IFM_ACTIVE) == 0) ;
	mii_tick(mii_sc);
	if (link_was_down && ((mii_sc->mii_media_status & IFM_ACTIVE) != 0) &&
	    !IFQ_DRV_IS_EMPTY(&ifp->if_snd)) {
		/* Start transmission here since the link went up */
		xge_txstart_locked(sc);
	}

	/* Schedule another check one second from now. */
	callout_reset(&sc->wd_callout, hz, xge_tick, sc);
}

int
xge_miibus_readreg(device_t dev, int phy, int reg)
{
	struct xge_softc *sc = device_get_softc(dev);
	struct xgene_enet_pdata *pdata = &sc->pdata;

	return (xgene_mii_phy_read(pdata, phy, reg));
}

int
xge_miibus_writereg(device_t dev, int phy, int reg, int val)
{
	struct xge_softc *sc = device_get_softc(dev);
	struct xgene_enet_pdata *pdata = &sc->pdata;

	return (xgene_mii_phy_write(pdata, phy, reg, val));
}

static int
xge_media_change_locked(struct xge_softc *sc)
{
	struct mii_data *mii_sc;

	XGE_GLOBAL_LOCK_ASSERT(sc);

	mii_sc = device_get_softc(sc->miibus);

	return (mii_mediachg(mii_sc));
}

static int
xge_media_change(struct ifnet *ifp)
{
	struct xge_softc *sc;
	int err;

	sc = ifp->if_softc;

	XGE_GLOBAL_LOCK(sc);
	err = xge_media_change_locked(sc);
	XGE_GLOBAL_UNLOCK(sc);

	return (err);
}

static void
xge_media_status(struct ifnet *ifp, struct ifmediareq *ifmr)
{
	struct xge_softc *sc;
	struct mii_data *mii_sc;

	sc = ifp->if_softc;
	mii_sc = device_get_softc(sc->miibus);

	XGE_GLOBAL_LOCK(sc);
	mii_pollstat(mii_sc);
	ifmr->ifm_active = mii_sc->mii_media_active;
	ifmr->ifm_status = mii_sc->mii_media_status;
	XGE_GLOBAL_UNLOCK(sc);
}

static void
xge_setup_ops(struct xge_softc *sc)
{
	struct xgene_enet_pdata *pdata = &sc->pdata;

	switch (sc->phy_conn_type) {
	case PHY_CONN_RGMII:
		pdata->mac_ops = &xgene_gmac_ops;
		pdata->port_ops = &xgene_gport_ops;
		pdata->rm = RM3;
		break;
	default:
#if 0
		pdata->mac_ops = &xgene_xgmac_ops;
		pdata->port_ops = &xgene_xgport_ops;
		pdata->rm = RM0;
#else
		panic("unsupported connection type");
#endif
		break;
	}
}

static void
xge_delete_desc_rings(struct xge_softc *sc)
{
	struct xgene_enet_pdata *pdata;
	struct xgene_enet_desc_ring *buf_pool;

	pdata = &sc->pdata;

	if (pdata->tx_ring != NULL) {
		xge_delete_ring(pdata->tx_ring);
		pdata->tx_ring = NULL;
	}

	if (pdata->rx_ring != NULL) {
		buf_pool = pdata->rx_ring->buf_pool;
#if 0
		/* XXX: Not yet */
		xge_delete_bufpool(buf_pool);
#endif
		xge_delete_ring(buf_pool);
		xge_delete_ring(pdata->rx_ring);
		pdata->rx_ring = NULL;
	}
}

static int
xge_new_bufpool(struct xgene_enet_desc_ring *buf_pool, uint32_t nbuf)
{
	size_t i;
	int err;

	err = bus_dma_tag_create(
	    bus_get_dma_tag(buf_pool->ndev),	/* parent */
	    XGE_BUFF_ALIGNMENT,			/* alignment */
	    0,					/* boundary */
	    BUS_SPACE_MAXADDR,			/* lowaddr */
	    BUS_SPACE_MAXADDR,			/* highaddr */
	    NULL, NULL,				/* filtfunc, filtfuncarg */
	    MCLBYTES,				/* maxsize */
	    XGE_BUFF_NSEGMENTS,			/* nsegments */
	    MCLBYTES,				/* maxsegsize */
	    0,					/* flags */
	    NULL, NULL,				/* lockfunc, lockfuncarg */
	    &buf_pool->rx_buf_dmat);		/* dmat */
	if (err) {
		device_printf(buf_pool->ndev,
		    "Failed to allocate busdma tag for buffers pool\n");
		goto dmatag_fail;
	}

	for (i = 0; i < nbuf; i++) {
		err = bus_dmamap_create(buf_pool->rx_buf_dmat, 0,
		    &buf_pool->rx_buf[i].dmap);
		if (err) {
			device_printf(buf_pool->ndev,
			    "Failed to create busdma map for buffers pool\n");
			goto dmamap_fail;
		}
	}

	return (0);

dmamap_fail:
	if (i > 0) {
		for (; i > 0; i--) {
			bus_dmamap_destroy(buf_pool->rx_buf_dmat,
			    buf_pool->rx_buf[i].dmap);
		}
	}
	bus_dma_tag_destroy(buf_pool->rx_buf_dmat);
dmatag_fail:
	return (err);
}

static int
xge_new_rxbuf(bus_dma_tag_t tag, bus_dmamap_t map, struct mbuf **mbufp,
    bus_addr_t *paddr)
{
	struct mbuf *new_mbuf;
	bus_dma_segment_t seg[1];
	int err;
	int nsegs;

	KASSERT(mbufp != NULL, ("NULL mbuf pointer!"));

	new_mbuf = m_getjcl(M_NOWAIT, MT_DATA, M_PKTHDR, MCLBYTES);
	if (new_mbuf == NULL)
		return (ENOBUFS);
	new_mbuf->m_len = new_mbuf->m_pkthdr.len = new_mbuf->m_ext.ext_size;

	if (*mbufp) {
		bus_dmamap_sync(tag, map, BUS_DMASYNC_POSTREAD);
		bus_dmamap_unload(tag, map);
	}

	err = bus_dmamap_load_mbuf_sg(tag, map, new_mbuf, seg, &nsegs,
	    BUS_DMA_NOWAIT);
	if ((nsegs != 1) || (err != 0)) {
		panic("%s: cannot dmamap_load mbuf, nsegs: %d, error: %d",
		    __func__, nsegs, err);
	}

	bus_dmamap_sync(tag, map, BUS_DMASYNC_PREREAD);

	(*mbufp) = new_mbuf;
	(*paddr) = seg->ds_addr;
	return (0);
}

static int
xge_refill_bufpool(struct xgene_enet_desc_ring *buf_pool, uint32_t nbuf)
{
	struct xge_softc *sc;
	struct xgene_enet_raw_desc16 *raw_desc;
	bus_dma_tag_t dmat;
	bus_dmamap_t dmap;
	bus_addr_t *paddr;
	struct mbuf **mbuf;
	uint16_t tail, slots;
	uint16_t bufdatalen;
	size_t i;
	int err;

	sc = device_get_softc(buf_pool->ndev);

	XGE_GLOBAL_LOCK(sc);

	tail = buf_pool->tail;
	slots = buf_pool->slots - 1;

	/*
	 * XXX: Revise that since XGENE_ENET_MAX_MTU == 1536
	 *      and MCLBYTES is 2KB in length
	 */
	bufdatalen = BUF_LEN_CODE_2K | (MCLBYTES & GENMASK(11, 0));

	dmat = buf_pool->rx_buf_dmat;

	for (i = 0; i < nbuf; i++) {
		raw_desc = &buf_pool->raw_desc16[tail];

		dmap = buf_pool->rx_buf[tail].dmap;
		mbuf = &buf_pool->rx_buf[tail].mbuf;
		paddr = &buf_pool->rx_buf[tail].paddr;

		err = xge_new_rxbuf(dmat, dmap, mbuf, paddr);
		if (err != 0) {
			/* XXX: We should handle this somehow different */
			panic("%s: cannot allocate new buffer", __func__);
			return (ENOBUFS);
		}

		raw_desc->m1 = htole64(SET_VAL(DATAADDR, *paddr) |
		    SET_VAL(BUFDATALEN, bufdatalen) | SET_BIT(COHERENT));

		tail = (tail + 1) & slots;
	}

	RING_CMD_WRITE32(&sc->pdata, buf_pool->cmd, nbuf);
	buf_pool->tail = tail;
	XGE_GLOBAL_UNLOCK(sc);

	return (0);
}

static int
xge_init_hw(struct xge_softc *sc)
{
	struct xgene_enet_pdata *pdata;
	struct xgene_enet_desc_ring *buf_pool;
	uint16_t dst_ring_num;
	int err;

	pdata = &sc->pdata;
	/* Port reset */
	err = pdata->port_ops->reset(pdata);
	if (err != 0)
		return (err);

	/* Allocate DMA ring buffers */
	err = xge_create_desc_rings(sc);
	if (err != 0)
		return (err);

	/* Set-up buffer pool */
	buf_pool = pdata->rx_ring->buf_pool;
	xge_init_bufpool(buf_pool);
	err = xge_refill_bufpool(buf_pool, pdata->rx_buff_cnt);
	if (err) {
		xge_delete_desc_rings(sc);
		return (err);
	}

	dst_ring_num = xge_dst_ring_num(pdata->rx_ring);
	pdata->port_ops->cle_bypass(pdata, dst_ring_num, buf_pool->id);
	pdata->mac_ops->init(pdata);

	return (err);
}
