# SPDX-License-Identifier: GPL-2.0
include ../scripts/Makefile.include

CC = $(CROSS_COMPILE)gcc
LD = $(CROSS_COMPILE)ld
ifneq (,$(wildcard $(CROSS_COMPILE)pkg-config))
PKG_CONFIG = $(CROSS_COMPILE)pkg-config
endif

ifeq ($(srctree),)
srctree := $(patsubst %/,%,$(dir $(shell pwd)))
srctree := $(patsubst %/,%,$(dir $(srctree)))
#$(info Determined 'srctree' to be $(srctree))
endif

INSTALL = install
prefix ?= /usr/local
bindir = $(prefix)/bin


CFLAGS += $(HOSTCFLAGS)
CFLAGS += -D__EXPORTED_HEADERS__
CFLAGS += -Wall -Wextra
ifdef RPMB_STATIC
LDFLAGS += -pthread -static
CFLAGS +=  -pthread -static
PKG_STATIC = --static
endif
CFLAGS += -I$(srctree)/include/uapi -I$(srctree)/include
ifneq ($(SYSROOT),)
CFLAGS += --sysroot=$(SYSROOT)
LDLIBS += --sysroot=$(SYSROOT)
endif
ifneq ($(PKG_CONFIG),)
LDLIBS += $(shell $(PKG_CONFIG) --libs $(PKG_STATIC) libcrypto)
else
LDLIBS += -lcrypto
endif

prog := rpmb

all : $(prog)

$(prog): rpmb.o

clean :
	$(RM) $(prog) *.o

install: $(prog)
	$(INSTALL) -m755 -d $(DESTDIR)$(bindir)
	$(INSTALL) $(prog) $(DESTDIR)$(bindir)
