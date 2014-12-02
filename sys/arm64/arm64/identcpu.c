/*-
 * Copyright (c) 2014 Andrew Turner
 * Copyright (c) 2014 The FreeBSD Foundation
 * All rights reserved.
 *
 * Portions of this software were developed by Semihalf
 * under sponsorship of the FreeBSD Foundation.
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
 */

#include <sys/cdefs.h>
__FBSDID("$FreeBSD$");

#include <sys/param.h>
#include <sys/pcpu.h>
#include <sys/systm.h>

#include <machine/cpu.h>
#include <machine/cpufunc.h>

char machine[] = "arm64";

/*
 * Per-CPU affinity as provided in MPIDR_EL1
 * Indexed by CPU number in logical order selected by the system.
 * Relevant fields can be extracetd using CPU_AFFn macros,
 * Aff3.Aff2.Aff1.Aff0 construct a unique CPU address in the system.
 *
 * Fields used by us:
 * Aff1 - Cluster number
 * Aff0 - CPU number in Aff1 cluster
 */
uint64_t __cpu_affinity[MAXCPU];

#define	CPU_IMPL_ARM		0x41
#define	CPU_IMPL_BROADCOM	0x42
#define	CPU_IMPL_CAVIUM		0x43
#define	CPU_IMPL_DEC		0x44
#define	CPU_IMPL_INFINEON	0x49
#define	CPU_IMPL_FREESCALE	0x4D
#define	CPU_IMPL_NVIDIA		0x4E
#define	CPU_IMPL_APM		0x50
#define	CPU_IMPL_QUALCOMM	0x51
#define	CPU_IMPL_MARVELL	0x56
#define	CPU_IMPL_INTEL		0x69

#define	CPU_PART_CORTEX_A57	0xD00
#define	CPU_PART_CORTEX_A53	0xD03

#define	CPU_IMPL(midr)	(((midr) >> 24) & 0xff)
#define	CPU_PART(midr)	(((midr) >> 4) & 0xfff)
#define	CPU_VAR(midr)	(((midr) >> 20) & 0xf)
#define	CPU_REV(midr)	(((midr) >> 0) & 0xf)

struct cpu_desc {
	u_int		cpu_impl;
	u_int		cpu_part_num;
	u_int		cpu_variant;
	u_int		cpu_revision;
	const char	*cpu_impl_name;
	const char	*cpu_part_name;
};

struct cpu_desc cpu_desc;

struct cpu_implementers {
	u_int		impl_id;
	const char	*impl_name;
};

const struct cpu_implementers cpu_implementers[] = {
	{ CPU_IMPL_ARM,		"ARM" },
	{ CPU_IMPL_BROADCOM,	"Broadcom" },
	{ CPU_IMPL_CAVIUM,	"Cavium" },
	{ CPU_IMPL_DEC,		"DEC" },
	{ CPU_IMPL_INFINEON,	"IFX" },
	{ CPU_IMPL_FREESCALE,	"Freescale" },
	{ CPU_IMPL_NVIDIA,	"NVIDIA" },
	{ CPU_IMPL_APM,		"APM" },
	{ CPU_IMPL_QUALCOMM,	"Qualcomm" },
	{ CPU_IMPL_MARVELL,	"Marvell" },
	{ CPU_IMPL_INTEL,	"Intel" },
	{ 0,			"Unknown Implementer" }
};

struct cpu_parts {
	u_int		part_id;
	const char	*part_name;
};

const struct cpu_parts cpu_parts[] = {
	{ CPU_PART_CORTEX_A57, "Cortex-A57" },
	{ CPU_PART_CORTEX_A53, "Cortex-A53" },
	{ 0, "Unknown Processor" }
};

void identify_cpu(void);

void
identify_cpu(void)
{
	u_int midr;
	u_int impl_id;
	u_int part_id;
	uint64_t mpidr;
	size_t i;

	midr = get_midr();

	impl_id = CPU_IMPL(midr);
	for (i = 0; i < nitems(cpu_implementers); i++) {
		if (impl_id == cpu_implementers[i].impl_id ||
		    cpu_implementers[i].impl_id == 0) {
			cpu_desc.cpu_impl = impl_id;
			cpu_desc.cpu_impl_name = cpu_implementers[i].impl_name;
			break;
		}
	}

	part_id = CPU_PART(midr);
	for (i = 0; i < nitems(cpu_parts); i++) {
		if (part_id == cpu_parts[i].part_id ||
		    cpu_parts[i].part_id == 0) {
			cpu_desc.cpu_part_num = part_id;
			cpu_desc.cpu_part_name = cpu_parts[i].part_name;
			break;
		}
	}

	printf("CPU: %s %s r%dp%d\n", cpu_desc.cpu_impl_name,
	    cpu_desc.cpu_part_name, CPU_VAR(midr), CPU_REV(midr));

	/*
	 * Save affinity for the boot CPU.
	 * (CPU0 in the internal system enumeration.
	 */
	mpidr = get_mpidr();
	CPU_AFFINITY(0) = mpidr & CPU_AFF_MASK;

	if (bootverbose)
		printf("CPU%u affinity: %u.%u.%u.%u\n", 0, CPU_AFF0(mpidr),
		    CPU_AFF1(mpidr), CPU_AFF2(mpidr), CPU_AFF3(mpidr));
}
