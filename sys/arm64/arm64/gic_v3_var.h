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

/*
 * Re-Distributor region description.
 * We will have few of those depending
 * on the #redistributor-regions property in FDT.
 */
struct redist_region {
	bus_space_tag_t		r_bst;
	bus_space_handle_t	r_bsh;
};
/*
 * Per-CPU Re-Distributor description.
 * Includes redundant busdma tag (which is the
 * same as corresponding redist_region tag).
 * This will help to simplify access to the tag
 * when using redist_pcpu structure.
 */
struct redist_pcpu {
	bus_space_tag_t		r_pcpu_bst;
	bus_space_handle_t	r_pcpu_bsh;
	vm_paddr_t		r_pcpu_pa;
};

struct gic_v3_softc {
	device_t		dev;
	struct resource	**	gic_res;
	struct mtx		gic_mtx;
	/* Distributor */
	bus_space_tag_t		gic_d_bst;
	bus_space_handle_t	gic_d_bsh;
	/* Re-Distributor regions */
	struct redist_region *	gic_r_regions;
	/* Number of Re-Distributor regions */
	u_int			gic_r_nregions;

	/* Per-CPU Re-Distributor handler */
	struct redist_pcpu	gic_r_pcpu[MAXCPU];

	u_int			gic_nirqs;
};

extern devclass_t arm_gic_v3_devclass;

MALLOC_DECLARE(M_GIC_V3);

/* Device and PIC methods starting with arm_gic_v3_ */
int arm_gic_v3_attach(device_t dev);
int arm_gic_v3_detach(device_t dev);

void arm_gic_v3_dispatch(device_t, struct trapframe *);
void arm_gic_v3_eoi(device_t, u_int);
void arm_gic_v3_mask_irq(device_t, u_int);
void arm_gic_v3_unmask_irq(device_t, u_int);

/*
 * ITS
 */
#define	GIC_V3_ITS_COMPSTR	"arm,gic-v3-its"

#endif /* _GIC_V3_VAR_H_ */
