dtb-$(CONFIG_ARCH_VEXPRESS) += rtsm_ve-aemv8a.dtb foundation-v8.dtb \
				fvp-base-gicv2-psci.dtb
dtb-$(CONFIG_ARCH_XGENE) += apm-mustang.dtb
dtb-$(CONFIG_ARCH_VEXPRESS) += juno.dtb

targets += dtbs
targets += $(dtb-y)

#used for cleaning - not building
subdir- := qcom

dtbs: $(addprefix $(obj)/, $(dtb-y))

clean-files := *.dtb
