GCC_VERSION      := $(shell $(CONFIG_SHELL) $(PWD)/scripts/gcc-version.sh $(CROSS_COMPILE)gcc)
ccflags-y += -Idrivers/media/video/msm
ccflags-y += -Idrivers/media/video/msm/server
ccflags-y += -Idrivers/media/video/msm/sensors
ccflags-y += -Idrivers/media/video/msm/io
ccflags-y += -Idrivers/media/video/msm/eeprom
obj-$(CONFIG_ARCH_MSM8960) += msm_vfe32.o
obj-$(CONFIG_MSM_CAMERA_V4L2) += msm_vfe_stats_buf.o
