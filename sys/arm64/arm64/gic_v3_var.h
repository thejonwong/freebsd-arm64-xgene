/*-
 * Copyright (c) 2014 The FreeBSD Foundation
 * All rights reserved.
 *
 * This software was developed by Semihalf under
 * the sponsorship of the FreeBSD Foundation.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 * 1. Redistributions of source code must retain the above copyright
 * notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 * notice, this list of conditions and the following disclaimer in the
 * documentation and/or other materials provided with the distribution.
 *
 * THIS SOFTWARE IS PROVIDED BY THE AUTHOR AND CONTRIBUTORS ``AS IS'' AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE AUTHOR OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS
 * OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY
 * OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF
 * SUCH DAMAGE.
 */

#ifndef _GIC_V3_VAR_H_
#define _GIC_V3_VAR_H_

#define	GIC_V3_DEVSTR	"ARM Generic Interrupt Controller v3.0"

#define	LPI_CONFTAB_SIZE	PAGE_SIZE_64K
/* 1 bit per LPI + 1 KB more for the obligatory PPI, SGI, SPI stuff */
#define	LPI_PENDTAB_SIZE	((LPI_CONFTAB_SIZE / 8) + 0x400)

struct redist_lpis {
	vm_offset_t		conf_base;
};

struct gic_redists {
	/*
	 * Re-Distributor region description.
	 * We will have few of those depending
	 * on the #redistributor-regions property in FDT.
	 */
	struct resource **	regions;
	/* Number of Re-Distributor regions */
	u_int			nregions;
	/* Per-CPU Re-Distributor handler */
	struct resource	*	pcpu[MAXCPU];
	/* LPIs data */
	struct redist_lpis	lpis;
};

struct gic_v3_softc {
	device_t		dev;
	struct resource	**	gic_res;
	struct mtx		gic_mtx;
	/* Distributor */
	struct resource *	gic_dist;
	/* Re-Distributors */
	struct gic_redists	gic_redists;

	u_int			gic_nirqs;
};

extern devclass_t gic_v3_devclass;

MALLOC_DECLARE(M_GIC_V3);

/* Device and PIC methods */
int gic_v3_attach(device_t dev);
int gic_v3_detach(device_t dev);

void gic_v3_dispatch(device_t, struct trapframe *);
void gic_v3_eoi(device_t, u_int);
void gic_v3_mask_irq(device_t, u_int);
void gic_v3_unmask_irq(device_t, u_int);

/*
 * ITS
 */
#define	GIC_V3_ITS_DEVSTR	"ARM GIC Interrupt Translation Service"
#define	GIC_V3_ITS_COMPSTR	"arm,gic-v3-its"


/* ITS commands description. Each command is 32 bytes long */
struct its_cmd {
	uint64_t its_cmd_desc[4];	/* ITS command descriptor */
};

/* ITS private table description */
struct its_ptab {
	vm_offset_t	ptab_vaddr;	/* Virtual Address of table */
	size_t		ptab_pgsz;	/* Page size */
	size_t		ptab_npages;	/* Number of pages */
};

/* ITS collection description. */
struct its_col {
	uint64_t	col_target;	/* Target Re-Distributor */
	uint64_t	col_id;		/* Collection ID */
};

#define	ITS_CMDQ_SIZE		PAGE_SIZE_64K
#define	ITS_CMDQ_NENTRIES	(ITS_CMDQ_SIZE / sizeof(struct its_cmd))

#define	ITS_FLAGS_CMDQ_FLUSH	(1UL << 0)

struct gic_v3_its_softc {
	device_t		dev;
	struct resource	*	its_res;

	struct its_cmd *	its_cmdq;	/* ITS command queue */
	struct its_ptab		its_ptabs[GITS_BASER_NUM];/* ITS private tables */
	struct its_col *	its_cols;	/* Per-CPU collections */

	uint64_t		its_flags;
};

extern devclass_t gic_v3_its_devclass;

int gic_v3_its_attach(device_t);
int gic_v3_its_detach(device_t);

#endif /* _GIC_V3_VAR_H_ */
