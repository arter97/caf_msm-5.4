virtio-video-objs := \
	virtio_video_driver.o \
	virtio_video_vq.o \
	virtio_video_device.o \
	virtio_video_dec.o \
	virtio_video_enc.o \
	virtio_video_cam.o \
	virtio_video_caps.o \
	virtio_video_helpers.o

obj-$(CONFIG_VIRTIO_VIDEO) += virtio-video.o

ccflags-y += -DDRIVER_VERSION=\"2.3.0\"
#
# Makefile for mod coqos
#
KDIR ?= $(KERNEL_SRC)
ifeq ($(KDIR),)
$(error "KDIR must be specified.")
endif

# The variable "M" is used to point to the location of this module, and it is
# passed to kbuild to build this module.
#
# Allow to specify variable "M" from outside. It is needed to set it to the
# relative path to this module. It must be relative to the kernel source
# directory.
#
# If kernel is built with "O" option then setting "M" to the relative path
# empowers the build system to put output/object files (.o, .ko.) into a
# directory different from the module source directory.
M ?= $$PWD

# Some build systems may rsync module sources out of git repository to have
# sources directory untouched during build. Then, they should specify Git
# repository directory separately for 'git describe' to work properly.
MODULE_GIT_REPOSITORY_DIR ?= $(M)

default:
	$(MAKE) -C $(KDIR) M=$(M) MODULE_GIT_REPOSITORY_DIR=$(MODULE_GIT_REPOSITORY_DIR)

modules_install:
	$(MAKE) -C $(KDIR) M=$(M) $@

clean:
	$(MAKE) -C $(KDIR) M=$(M) $@
