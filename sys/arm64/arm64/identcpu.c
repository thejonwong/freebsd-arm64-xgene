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
#include <sys/sysctl.h>
#include <sys/systm.h>

#include <machine/cpu.h>
#include <machine/cpufunc.h>

char machine[] = "arm64";

SYSCTL_STRING(_hw, HW_MACHINE, machine, CTLFLAG_RD, machine, 0,
    "Machine class");

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

#define	CPU_PART_THUNDER	0x0A1
#define	CPU_PART_FOUNDATION	0xD00
#define	CPU_PART_CORTEX_A53	0xD03
#define	CPU_PART_CORTEX_A57	0xD07

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

struct cpu_desc cpu_desc[MAXCPU];

struct cpu_parts {
	u_int		part_id;
	const char	*part_name;
};
#define	CPU_PART_NONE	{ 0, "Unknown Processor" }

struct cpu_implementers {
	u_int			impl_id;
	const char		*impl_name;
	/*
	 * Part number is implementation defined
	 * so each vendor will have its own set of values and names.
	 */
	const struct cpu_parts	*cpu_parts;
};
#define	CPU_IMPLEMENTER_NONE	{ 0, "Unknown Implementer", cpu_parts_none }

/*
 * Per-implementer table of (PartNum, CPU Name) pairs.
 */
/* ARM Ltd. */
static const struct cpu_parts cpu_parts_arm[] = {
	{ 0xD00, "Foundation-Model" },
	{ 0xD03, "Cortex-A53" },
	{ 0xD07, "Cortex-A57" },
	CPU_PART_NONE,
};
/* Cavium */
static const struct cpu_parts cpu_parts_cavium[] = {
	{ 0x0A1, "Thunder" },
	CPU_PART_NONE,
};

/* AppliedMicro */
static const struct cpu_parts cpu_parts_apm[] = {
	{ 0x000, "Potenza" },
	CPU_PART_NONE,
};

/* Unknown */
static const struct cpu_parts cpu_parts_none[] = {
	CPU_PART_NONE,
};

/*
 * Implementers table.
 */
const struct cpu_implementers cpu_implementers[] = {
	{ CPU_IMPL_ARM,		"ARM",		cpu_parts_arm },
	{ CPU_IMPL_BROADCOM,	"Broadcom",	cpu_parts_none },
	{ CPU_IMPL_CAVIUM,	"Cavium",	cpu_parts_cavium },
	{ CPU_IMPL_DEC,		"DEC",		cpu_parts_none },
	{ CPU_IMPL_INFINEON,	"IFX",		cpu_parts_none },
	{ CPU_IMPL_FREESCALE,	"Freescale",	cpu_parts_none },
	{ CPU_IMPL_NVIDIA,	"NVIDIA",	cpu_parts_none },
	{ CPU_IMPL_APM,		"APM",		cpu_parts_apm },
	{ CPU_IMPL_QUALCOMM,	"Qualcomm",	cpu_parts_none },
	{ CPU_IMPL_MARVELL,	"Marvell",	cpu_parts_none },
	{ CPU_IMPL_INTEL,	"Intel",	cpu_parts_none },
	CPU_IMPLEMENTER_NONE,
};

void identify_cpu(void);

void
identify_cpu(void)
{
	u_int midr;
	u_int impl_id;
	u_int part_id;
	u_int cpu;
	uint64_t mpidr;
	size_t i;
	const struct cpu_parts *cpu_partsp = NULL;

	cpu = PCPU_GET(cpuid);
	midr = get_midr();

	impl_id = CPU_IMPL(midr);
	for (i = 0; i < nitems(cpu_implementers); i++) {
		if (impl_id == cpu_implementers[i].impl_id ||
		    cpu_implementers[i].impl_id == 0) {
			cpu_desc[cpu].cpu_impl = impl_id;
			cpu_desc[cpu].cpu_impl_name = cpu_implementers[i].impl_name;
			cpu_partsp = cpu_implementers[i].cpu_parts;
			break;
		}
	}

	part_id = CPU_PART(midr);
	for (i = 0; &cpu_partsp[i] != NULL; i++) {
		if (part_id == cpu_partsp[i].part_id ||
		    cpu_partsp[i].part_id == 0) {
			cpu_desc[cpu].cpu_part_num = part_id;
			cpu_desc[cpu].cpu_part_name = cpu_partsp[i].part_name;
			break;
		}
	}

	printf("CPU: %s %s r%dp%d\n", cpu_desc[cpu].cpu_impl_name,
	    cpu_desc[cpu].cpu_part_name, CPU_VAR(midr), CPU_REV(midr));

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
