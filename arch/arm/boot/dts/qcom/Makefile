ifeq ($(CONFIG_OF),y)

dtb-$(CONFIG_ARCH_APQ8084) += apq8084-v1.0-old-pwr-grid-liquid.dtb \
	apq8084-v1.1-old-pwr-grid-liquid.dtb \
	apq8084-v1.1-liquid.dtb \
	apq8084-v1.0-old-pwr-grid-smb1357-cdp.dtb \
	apq8084-v1.1-old-pwr-grid-smb1357-cdp.dtb \
	apq8084-v1.1-smb1357-cdp.dtb \
	apq8084-v1.0-old-pwr-grid-smb1357-mtp.dtb \
	apq8084-v1.1-old-pwr-grid-smb1357-mtp.dtb \
	apq8084-v1.1-smb1357-mtp.dtb \
	apq8084-v1.0-old-pwr-grid-smb1359-mdm9x35-alt-mtp.dtb \
	apq8084-v1.1-old-pwr-grid-smb1359-mdm9x35-alt-mtp.dtb \
	apq8084-v1.1-smb1359-mdm9x35-alt-mtp.dtb \
	apq8084-old-pwr-grid-smb349-cdp.dtb \
	apq8084-old-pwr-grid-smb349-mtp.dtb \
	apq8084-v1.0-old-pwr-grid-smb1357-mdm9x35-alt-rcm.dtb \
	apq8084-v1.1-old-pwr-grid-smb1357-mdm9x35-alt-rcm.dtb \
	apq8084-v1.1-smb1357-mdm9x35-alt-rcm.dtb \
	apq8084-v1.0-old-pwr-grid-smb1357-mdm9x35-mtp.dtb \
	apq8084-v1.1-old-pwr-grid-smb1357-mdm9x35-mtp.dtb \
	apq8084-v1.1-smb1357-mdm9x35-mtp.dtb \
	apq8084-v1.0-old-pwr-grid-smb1357-mdm9x35-cdp.dtb \
	apq8084-v1.1-old-pwr-grid-smb1357-mdm9x35-cdp.dtb \
	apq8084-v1.1-smb1357-mdm9x35-cdp.dtb \
	apq8084-v1.0-old-pwr-grid-mdm9x35-alt-cdp.dtb \
	apq8084-v1.1-old-pwr-grid-mdm9x35-alt-cdp.dtb \
	apq8084-v1.1-mdm9x35-alt-cdp.dtb \
	apq8084-v1.1-sbc.dtb
dtb-$(CONFIG_ARCH_MSM8916) += msm8916-sim.dtb \
	msm8916-rumi.dtb \
	msm8916-cdp.dtb \
	msm8916-cdp-smb1360.dtb \
	msm8916-mtp.dtb \
	msm8916-512mb-mtp.dtb \
	msm8916-mtp-smb1360.dtb \
	msm8916-512mb-mtp-smb1360.dtb \
	msm8916-512mb-qrd-skui.dtb \
	msm8916-qrd-skuh.dtb \
	msm8916-qrd-skuhf.dtb \
	msm8916-qrd-skui.dtb \
	msm8916-512mb-qrd-skuh.dtb \
	msm8939-sim.dtb \
	msm8939-rumi.dtb \
	msm8939-qrd-skuk.dtb \
	msm8939-cdp.dtb \
	msm8939-cdp-smb1360.dtb \
	msm8939-mtp.dtb \
	msm8939-mtp-smb1360.dtb

targets += dtbs
targets += $(addprefix ../, $(dtb-y))
endif

$(obj)/../%.dtb: $(src)/%.dts FORCE
	$(call if_changed_dep,dtc)

dtbs: $(addprefix $(obj)/../,$(dtb-y))

clean-files := *.dtb
