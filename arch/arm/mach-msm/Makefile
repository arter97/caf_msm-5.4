obj-y += io.o
obj-$(CONFIG_USE_OF) += board-dt.o

obj-$(CONFIG_ARCH_MSM_KRAIT) += msm-krait-l2-accessors.o
obj-$(CONFIG_DEBUG_FS) += perf_debug.o

obj-$(CONFIG_MSM_TIMER) += timer.o
obj-$(CONFIG_MSM_PROC_COMM) += clock.o

obj-$(CONFIG_MSM_VIC) += irq-vic.o

obj-$(CONFIG_ARCH_MSM7X00A) += irq.o
obj-$(CONFIG_ARCH_QSD8X50) += sirc.o

obj-$(CONFIG_MSM_PROC_COMM) += proc_comm.o clock-pcom.o vreg.o

obj-$(CONFIG_QPNP_BMS) += bms-batterydata.o bms-batterydata-desay.o
obj-$(CONFIG_QPNP_BMS) += bms-batterydata-oem.o bms-batterydata-qrd-4v35-2000mah.o bms-batterydata-qrd-4v2-1300mah.o

obj-$(CONFIG_ARCH_MSM7X00A) += dma.o io.o
obj-$(CONFIG_ARCH_MSM7X30) += dma.o io.o
obj-$(CONFIG_ARCH_QSD8X50) += dma.o io.o

obj-$(CONFIG_SMP) += headsmp.o platsmp.o
obj-$(CONFIG_HOTPLUG_CPU) += hotplug.o

obj-$(CONFIG_KRAIT_REGULATOR) += krait-regulator.o  krait-regulator-pmic.o

obj-$(CONFIG_MACH_TROUT) += board-trout.o board-trout-gpio.o board-trout-mmc.o devices-msm7x00.o
obj-$(CONFIG_MACH_TROUT) += board-trout.o board-trout-gpio.o board-trout-mmc.o board-trout-panel.o devices-msm7x00.o
obj-$(CONFIG_MACH_HALIBUT) += board-halibut.o devices-msm7x00.o
obj-$(CONFIG_ARCH_MSM7X30) += board-msm7x30.o devices-msm7x30.o
obj-$(CONFIG_ARCH_QSD8X50) += board-qsd8x50.o devices-qsd8x50.o

obj-$(CONFIG_ARCH_APQ8084) += gpiomux-v2.o gpiomux.o

obj-$(CONFIG_ARCH_APQ8084) += board-8084.o board-8084-gpiomux.o
obj-$(CONFIG_ARCH_APQ8084) += clock-8084.o clock-mdss-8974.o

obj-$(CONFIG_ARCH_QSD8X50) += gpiomux-8x50.o

obj-$(CONFIG_MSM_CACHE_ERP) += cache_erp.o
obj-$(CONFIG_MEMORY_HOLE_CARVEOUT) +=  msm_mem_hole.o

obj-$(CONFIG_MSM_HSIC_SYSMON) += hsic_sysmon.o
obj-$(CONFIG_MSM_HSIC_SYSMON_TEST) += hsic_sysmon_test.o

obj-$(CONFIG_MSM_TEST_QMI_CLIENT) += kernel_test_service_v01.o test_qmi_client.o

obj-$(CONFIG_PERFMAP) += perfmap.o
