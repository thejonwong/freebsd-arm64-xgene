//===-- lib/fixdfsi.c - Double-precision -> integer conversion ----*- C -*-===//
//
//                     The LLVM Compiler Infrastructure
//
// This file is dual licensed under the MIT and the University of Illinois Open
// Source Licenses. See LICENSE.TXT for details.
//
//===----------------------------------------------------------------------===//
//
// This file implements float to integer conversion for the
// compiler-rt library.
//
//===----------------------------------------------------------------------===//

#include "fp_lib.h"

static inline int __fixXsi(fp_t a) {
    // Break a into sign, exponent, significand
    const rep_t aRep = toRep(a);
    const rep_t aAbs = aRep & absMask;
    const int sign = aRep & signBit ? -1 : 1;
    const int exponent = (aAbs >> significandBits) - exponentBias;
    const rep_t significand = (aAbs & significandMask) | implicitBit;

    // If exponent is negative, the result is zero.
    if (exponent < 0)
        return 0;

    // If 0 <= exponent < significandBits, right shift to get the result.
    if (exponent < significandBits) {
        // If the value is too large for int, saturate.
        if (exponent >= sizeof(int) * CHAR_BIT)
            return sign == 1 ? INT_MAX : INT_MIN;
        return sign * (significand >> (significandBits - exponent));
    }

    if (exponent >= sizeof(int) * CHAR_BIT)
        return sign == 1 ? INT_MAX : INT_MIN;
    return sign * (significand << (exponent - significandBits));
}
