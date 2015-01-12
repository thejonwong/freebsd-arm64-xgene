/* ===-- fixunstfsi.c - Implement __fixunstfsi -----------------------------===
 *
 *                     The LLVM Compiler Infrastructure
 *
 * This file is dual licensed under the MIT and the University of Illinois Open
 * Source Licenses. See LICENSE.TXT for details.
 *
 * ===----------------------------------------------------------------------===
 *
 * This file implements __fixunstfsi for the compiler_rt library.
 *
 * ===----------------------------------------------------------------------===
 */

#define QUAD_PRECISION
#include "fp_lib.h"

#if defined(CRT_HAS_128BIT) && defined(CRT_LDBL_128BIT)
/* Returns: convert a to a unsigned int, rounding toward zero.
 *          Negative values all become zero.
 */

/* Assumption: long double is a IEEE 128 bit floating point type
 *             su_int is a 32 bit integral type
 *             value in long double is representable in su_int or is negative
 *                 (no range checking performed)
 */

/* seee eeee eeee eeee mmmm mmmm mmmm mmmm | mmmm mmmm mmmm mmmm mmmm mmmm mmmm mmmm
   mmmm mmmm mmmm mmmm mmmm mmmm mmmm mmmm | mmmm mmmm mmmm mmmm mmmm mmmm mmmm mmmm */

COMPILER_RT_ABI su_int
__fixunstfsi(long double a)
{
    long_double_bits fb;
    fb.f = a;
    int e = ((fb.u.high.s.high & 0x7FFF0000) >> 16) - 16383;
    if (e < 0 || (fb.u.high.s.high & 0x80000000))
        return 0;
    return (
                0x80000000u                      |
                ((fb.u.high.s.high & 0x0000FFFF) << 15) |
                (fb.u.high.s.low >> 17)
           ) >> (31 - e);
}

#endif
