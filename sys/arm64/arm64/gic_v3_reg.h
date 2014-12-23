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

#ifndef _GIC_V3_REG_H_
#define _GIC_V3_REG_H_

/* Priority MAX/MIN values */
#define	GIC_PRIORITY_MAX	0x00UL
#define	GIC_PRIORITY_MIN	0xFFUL

/*
 * Registers (v2/v3)
 */
#define	GICD_CTLR		0x0000
#define	GICD_CTLR_G1		(1 << 0)
#define	GICD_CTLR_G1A		(1 << 1)
#define	GICD_CTLR_ARE_NS	(1 << 4)
#define	GICD_CTLR_RWP		(1 << 31)

#define	GICD_TYPER		(0x0004)
#define	GICD_ISENABLER(n)	(0x0100 + ((n) * 4))
#define	GICD_ICENABLER(n)	(0x0180 + ((n) * 4))
#define	GICD_IPRIORITYR(n)	(0x0400 + ((n) * 4))

#define	GICD_ICFGR(n)		(0x0C00 + ((n) * 4))
/* First bit is a polarity bit (0 - low, 1 - high) */
#define		GICD_ICFGR_POL_LOW	(0 << 0)
#define		GICD_ICFGR_POL_HIGH	(1 << 0)
#define		GICD_ICFGR_POL_MASK	0x1
/* Second bit is a trigger bit (0 - level, 1 - edge) */
#define		GICD_ICFGR_TRIG_LVL	(0 << 1)
#define		GICD_ICFGR_TRIG_EDGE	(1 << 1)
#define		GICD_ICFGR_TRIG_MASK	0x2

/*
 * Registers (v3)
 */
#define	GICD_IROUTER(n)		(0x6000 + ((n) * 8))
#define	GICD_PIDR2		(0xFFE8)

#define	GICR_PIDR2_ARCH_MASK	(0xF0)
#define	GICR_PIDR2_ARCH_GICv3	(0x30)
#define	GICR_PIDR2_ARCH_GICv4	(0x40)

/* Redistributor registers */
#define	GICR_PIDR2		GICD_PIDR2

#define	GICR_TYPER		(0x0008)
#define	GICR_TYPER_VLPIS	(1 << 1)
#define	GICR_TYPER_LAST		(1 << 4)

#define	GICR_WAKER		(0x0014)
#define	GICR_WAKER_PS		(1 << 1) /* Processor sleep */
#define	GICR_WAKER_CA		(1 << 2) /* Children asleep */

/* Re-distributor registers for SGIs and PPIs */
#define	GICR_ISENABLER0		(0x0100)
#define	GICR_ICENABLER0		(0x0180)

/*
 * CPU interface
 */

/* EL2 */
#define	ICC_SRE_EL2_EN		(1U << 3)
/* EL1 */
#define	ICC_SRE_EL1_SRE		(1U << 0)
#define	ICC_CTLR_EL1_EOI	(1U << 1)
#define	ICC_IGRPEN0_EL1_EN	(1U << 0)

#define	ICC_IAR1_EL1_SPUR	(0x03ff)

/*
 * Registers list (ICC_xyz_EL1):
 *
 * PMR     - Priority Mask Register
 *		* interrupts of priority higher than specified
 *		  in this mask will be signalled to the CPU.
 *		  (0xff - lowest possible prio., 0x00 - highest prio.)
 *
 * CTLR    - Control Register
 *		* controls behavior of the CPU interface and displays
 *		  implemented features.
 *
 * IGRPEN1 - Interrupt Group 1 Enable Register
 *
 * IAR1    - Interrupt Acknowledge Register Group 1
 *		* contains number of the highest priority pending
 *		  interrupt from the Group 1.
 *
 * EOIR1   - End of Interrupt Register Group 1
 *		* Writes inform CPU interface about completed Group 1
 *		  interrupts processing.
 */

#define	gic_icc_write(reg, val)					\
do {								\
	__asm __volatile(					\
	    "msr ICC_"__STRING(reg)"_EL1, %0	\n"		\
	    "isb" : : "r" ((val)));				\
} while (0)

#define	gic_icc_read(reg)					\
({								\
	uint64_t val;						\
	__asm __volatile(					\
	    "mrs %0, ICC_"__STRING(reg)"_EL1" : "=r" (val));	\
	(val);							\
})

#define	gic_icc_set(reg, mask)					\
do {								\
	uint64_t val;						\
	val = gic_icc_read(reg);				\
	val |= (mask);						\
	gic_icc_write(reg, val);				\
} while (0)

#define	gic_icc_clear(reg, mask)				\
do {								\
	uint64_t val;						\
	val = gic_icc_read(reg);				\
	val &= ~(mask);						\
	gic_icc_write(reg, val);				\
} while (0)

#endif /* _GIC_V3_REG_H_ */
