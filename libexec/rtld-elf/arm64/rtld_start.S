/*-
 * Copyright (c) 2014 The FreeBSD Foundation
 * All rights reserved.
 *
 * This software was developed by Andrew Turner under
 * sponsorship from the FreeBSD Foundation.
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

#include <machine/asm.h>
__FBSDID("$FreeBSD$");

ENTRY(.rtld_start)
	mov	x19, x2		/* Put ps_strings in a callee-saved register */
	mov	x20, sp		/* And the stack pointer */

	sub	x8, x20, #16	/* Make room for obj_main & exit proc */
	mov	sp, x8		/* Update the stack pointer */

	adr	x3, .Ldelta
	ldr	x2, [x3]
	sub	x1, x3, x2	/* Load the relocation delta */
	adr	x0, _DYNAMIC	/* Load &_DYNAMIC */
	bl	reloc_non_plt_self

	mov	x0, x20		/* Pass the stack we were given to _rtld */
	mov	x1, sp		/* exit_proc */
	add	x2, x1, #8	/* obj_main */
	/* Good */
	bl	_rtld		/* Call the loader */
	mov	x8, x0		/* Backup the entry point */

	ldr	x2, [sp]	/* Load cleanup */
	ldr	x1, [sp, #8]	/* Load obj_main */
	mov	x0, x19		/* Restore ps_strings */
	mov	sp, x20		/* Restore the stack pointer */
	br	x8		/* Jump to the entry point */
.Ldelta:
	.word	.
	.word	0
END(.rtld_start)

/*
 * uint64_t _rtld_tlsdesc(struct tlsdesc *);
 *
 * struct tlsdesc {
 *  uint64_t ptr;
 *  uint64_t data;
 * };
 *
 * Returns the data.
 */
ENTRY(_rtld_tlsdesc)
	ldr	x0, [x0, #8]
	RET
END(_rtld_tlsdesc)
