// SPDX-License-Identifier: GPL-2.0-only
/* r8169_phy_config.c: RTL8168H-only PHY configuration. */

#include <linux/phy.h>

#include "r8169.h"

static void r8168g_phy_param(struct phy_device *phydev, u16 parm,
			     u16 mask, u16 val)
{
	int oldpage = phy_select_page(phydev, 0x0a43);

	__phy_write(phydev, 0x13, parm);
	__phy_modify(phydev, 0x14, mask, val);

	phy_restore_page(phydev, oldpage, 0);
}

static void rtl8168g_config_eee_phy(struct phy_device *phydev)
{
	phy_modify_paged(phydev, 0x0a43, 0x11, 0, BIT(4));
}

static void rtl8168h_config_eee_phy(struct phy_device *phydev)
{
	rtl8168g_config_eee_phy(phydev);

	phy_modify_paged(phydev, 0x0a4a, 0x11, 0x0000, 0x0200);
	phy_modify_paged(phydev, 0x0a42, 0x14, 0x0000, 0x0080);
}

static void rtl8168g_disable_aldps(struct phy_device *phydev)
{
	phy_modify_paged(phydev, 0x0a43, 0x10, BIT(2), 0);
}

static void rtl8168g_enable_gphy_10m(struct phy_device *phydev)
{
	phy_modify_paged(phydev, 0x0a44, 0x11, 0, BIT(11));
}

static void rtl8168h_2_hw_phy_config(struct rtl8169_private *tp,
				     struct phy_device *phydev)
{
	u16 ioffset, rlen;
	u32 data;

	r8169_apply_firmware(tp);

	/* CHIN EST parameter update */
	r8168g_phy_param(phydev, 0x808a, 0x003f, 0x000a);

	/* Enable R-tune & PGA-retune function. */
	r8168g_phy_param(phydev, 0x0811, 0x0000, 0x0800);
	phy_modify_paged(phydev, 0x0a42, 0x16, 0x0000, 0x0002);

	rtl8168g_enable_gphy_10m(phydev);

	ioffset = rtl8168h_2_get_adc_bias_ioffset(tp);
	if (ioffset != 0xffff)
		phy_write_paged(phydev, 0x0bcf, 0x16, ioffset);

	/* Modify rlen (TX LPF corner frequency) level. */
	data = phy_read_paged(phydev, 0x0bcd, 0x16);
	data &= 0x000f;
	rlen = 0;
	if (data > 3)
		rlen = data - 3;
	data = rlen | (rlen << 4) | (rlen << 8) | (rlen << 12);
	phy_write_paged(phydev, 0x0bcd, 0x17, data);

	/* Disable PHY power save modes that increase latency variance. */
	phy_modify_paged(phydev, 0x0a44, 0x11, BIT(7), 0);
	phy_modify_paged(phydev, 0x0a43, 0x10, BIT(0), 0);

	rtl8168g_disable_aldps(phydev);
	rtl8168h_config_eee_phy(phydev);
}

void r8169_hw_phy_config(struct rtl8169_private *tp, struct phy_device *phydev)
{
	rtl8168h_2_hw_phy_config(tp, phydev);
}
