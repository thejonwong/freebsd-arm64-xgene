# $FreeBSD$

SRCS+=	__vdso_gettc.c

#MDASM= brk.S ptrace.S sbrk.S
MDASM=	cerror.S \
	pipe.S \
	shmat.S \
	sigreturn.S \
	syscall.S \
	vfork.S

# Don't generate default code for these syscalls:
NOASM=	break.o \
	exit.o \
	getlogin.o \
	openbsd_poll.o \
	sstk.o \
	vfork.o \
	yield.o

PSEUDO= _exit.o \
	_getlogin.o
.if ${MK_SYSCALL_COMPAT} != "no"
PSEUDO+= _pread.o \
	_pwrite.o \
	_lseek.o \
	_mmap.o \
	_ftruncate.o \
	_truncate.o
.endif
