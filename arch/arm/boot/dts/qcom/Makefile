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
dtb-$(CONFIG_ARCH_MSM8994) += msm8994-v1-sim.dtb \
	msm8994-rumi.dtb \
	msm8994-v1-cdp.dtb \
	msm8994-pmi8994-pm8004-v1-cdp.dtb \
	msm8994-v1-mtp.dtb \
	msm8994-pmi8994-pm8004-v1-mtp.dtb \
	msm8994-v1-liquid.dtb \
	msm8994-v1-fluid.dtb \
	apq8094-v1-cdp.dtb \
	apq8094-v1-mtp.dtb \
	apq8094-v1-liquid.dtb \
	apq8094-v1-fluid.dtb \
	apq8094-v1-dragonboard.dtb \
	msm8994-v2-sim.dtb \
	msm8994-v2-cdp.dtb \
	msm8994-pmi8994-pm8004-v2-cdp.dtb \
	msm8994-v2-mtp.dtb \
	msm8994-pmi8994-pm8004-v2-mtp.dtb \
	msm8994-v2-liquid.dtb \
	msm8994-v2-fluid.dtb \
	apq8094-v2-cdp.dtb \
	apq8094-v2-mtp.dtb \
	apq8094-v2-liquid.dtb \
	apq8094-v2-fluid.dtb \
	apq8094-v2-dragonboard.dtb

dtb-$(CONFIG_ARCH_MSM8992) += msm8992-sim.dtb \
	msm8992-rumi.dtb \
	msm8992-cdp.dtb \
	msm8992-mtp.dtb
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
	msm8916-qrd-skuid.dtb \
	msm8916-512mb-qrd-skuid.dtb \
	msm8916-512mb-qrd-skuh.dtb \
	msm8939-sim.dtb \
	msm8939-rumi.dtb \
	msm8939-qrd-skuk.dtb \
	msm8939-cdp.dtb \
	msm8939-cdp-jdi.dtb \
	msm8939-mtp.dtb \
	msm8939-mtp-smb1360.dtb \
	msm8936-qrd-skuk.dtb \
	msm8936-cdp.dtb \
	msm8936-cdp-smb1360.dtb \
	msm8936-mtp.dtb \
	msm8936-mtp-smb1360.dtb \
	msm8936-mtp-wcd9306.dtb
dtb-$(CONFIG_ARCH_MSMTHULIUM) += msmthulium-sim.dtb \
	msmthulium-rumi.dtb

targets += dtbs
targets += $(addprefix ../, $(dtb-y))
endif

$(obj)/../%.dtb: $(src)/%.dts FORCE
	$(call if_changed_dep,dtc)

dtbs: $(addprefix $(obj)/../,$(dtb-y))

clean-files := *.dtb
