# $FreeBSD$

.include <src.opts.mk>

SUBDIR= csu libgcc libdialog libregex libreadline

# XXX ARM64TODO: Fix these
.if ${MACHINE} != "arm64"
SUBDIR+=libgcov libgomp
.endif

.if ${MK_SSP} != "no"
SUBDIR+= libssp
.endif

.if ${MK_TESTS} != "no"
SUBDIR+= tests
.endif

# libsupc++ uses libstdc++ headers, although 'make includes' should
# have taken care of that already.
.if ${MK_GNUCXX} != "no"
SUBDIR+= libstdc++ libsupc++
.endif

.include <bsd.subdir.mk>
