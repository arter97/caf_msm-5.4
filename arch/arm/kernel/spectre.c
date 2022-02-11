// SPDX-License-Identifier: GPL-2.0-only
#include <linux/cpu.h>
#include <linux/device.h>

#include <asm/spectre.h>

ssize_t cpu_show_spectre_v1(struct device *dev, struct device_attribute *attr,
			    char *buf)
{
	return snprintf(buf, "Mitigation: __user pointer sanitization\n");
}

static unsigned int spectre_v2_state;
static unsigned int spectre_v2_methods;

void spectre_v2_update_state(unsigned int state, unsigned int method)
{
	if (state > spectre_v2_state)
		spectre_v2_state = state;
	spectre_v2_methods |= method;
}

ssize_t cpu_show_spectre_v2(struct device *dev, struct device_attribute *attr,
			    char *buf)
{
	const char *method;

	if (spectre_v2_state == SPECTRE_UNAFFECTED)
		return snprintf(buf, "%s\n", "Not affected");

	if (spectre_v2_state != SPECTRE_MITIGATED)
		return snprintf(buf, "%s\n", "Vulnerable");

	switch (spectre_v2_methods) {
	case SPECTRE_V2_METHOD_BPIALL:
		method = "Branch predictor hardening";
		break;

	case SPECTRE_V2_METHOD_ICIALLU:
		method = "I-cache invalidation";
		break;

	case SPECTRE_V2_METHOD_SMC:
	case SPECTRE_V2_METHOD_HVC:
		method = "Firmware call";
		break;

	default:
		method = "Multiple mitigations";
		break;
	}

	return snprintf(buf, "Mitigation: %s\n", method);
}
