# When adding new entries keep the list in alphabetical order
CFLAGS_scm.o :=$(call as-instr,.arch_extension sec,-DREQUIRES_SEC=1)

obj-$(CONFIG_ARM64) += cpu_ops.o
obj-$(CONFIG_QCOM_GSBI)	+=	qcom_gsbi.o
obj-$(CONFIG_MSM_IPC_ROUTER_HSIC_XPRT) += ipc_router_hsic_xprt.o
obj-$(CONFIG_MSM_IPC_ROUTER_SMD_XPRT) += ipc_router_smd_xprt.o
obj-$(CONFIG_MSM_OCMEM)	+=	ocmem.o ocmem_allocator.o ocmem_notifier.o
obj-$(CONFIG_MSM_OCMEM)	+=	ocmem_sched.o ocmem_api.o ocmem_rdm.o ocmem_core.o
obj-$(CONFIG_MSM_QMI_INTERFACE) += qmi_interface.o
obj-$(CONFIG_MSM_RPM_SMD) +=	rpm-smd.o
ifdef CONFIG_DEBUG_FS
obj-$(CONFIG_MSM_RPM_SMD) +=	rpm-smd-debug.o
endif
obj-$(CONFIG_MSM_SCM)	+=	scm.o scm-boot.o
obj-$(CONFIG_MSM_SMD)	+=	smd.o smd_debug.o smd_private.o smd_init_dt.o smsm_debug.o
obj-$(CONFIG_MSM_SMEM)	+=	smem.o smem_debug.o
obj-$(CONFIG_MSM_SMEM_LOGGING) += smem_log.o
obj-$(CONFIG_MSM_SMP2P) += smp2p.o smp2p_debug.o
obj-$(CONFIG_MSM_SMP2P_TEST) += smp2p_loopback.o smp2p_test.o smp2p_spinlock_test.o
obj-$(CONFIG_SOC_BUS)	+=	socinfo.o
obj-y			+=	qdsp6v2/
