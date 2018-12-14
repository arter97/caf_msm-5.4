/* Macros for events to userspace */
#define	QPNP_SMBCHG_BATT_HOT				0x000001
#define	QPNP_SMBCHG_BATT_WARM				0x000002
#define	QPNP_SMBCHG_BATT_COLD				0x000004
#define	QPNP_SMBCHG_BATT_COOL				0x000008
#define	QPNP_SMBCHG_BATT_MISSING			0x000010
#define	QPNP_SMBCHG_BATT_LOW				0x000020
#define	QPNP_SMBCHG_CHG_ERR					0x000040
#define	QPNP_SMBCHG_CHG_FAST				0x000080
#define	QPNP_SMBCHG_CHG_FAST_REMOVED		0x000100
#define	QPNP_SMBCHG_TEMP_SHUTDOWN			0x000200
#define	QPNP_SMBCHG_WDOG_TIMEOUT			0x000400
#define	QPNP_SMBCHG_CHG_TERM				0x000800
#define	QPNP_SMBCHG_CHG_TAPER				0x001000
#define	QPNP_SMBCHG_CHG_RECHRG				0x002000
#define	QPNP_SMBCHG_CHG_USB_SRC_INSERTED	0x004000
#define	QPNP_SMBCHG_CHG_USB_SRC_REMOVED		0x008000
#define	QPNP_SMBCHG_USB_OTG_OVER_CURRENT	0x010000
#define	QPNP_SMBCHG_CHG_USB_OV_THR_CRSD		0x020000
#define	QPNP_SMBCHG_CHG_USB_UV_THR_CRSD		0x040000
#define	QPNP_SMBCHG_USB_HOST				0x080000

/* 
	data to be read from user space 
*/

struct battery_mon_data
{
	char batt_chg_sts;
	char batt_chg_type;
	char batt_capacity;
	uint32_t batt_temp;
	uint32_t batt_current;
	uint32_t batt_voltage;
}__attribute__((packed));
