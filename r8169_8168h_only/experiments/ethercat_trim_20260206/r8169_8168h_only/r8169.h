/* SPDX-License-Identifier: GPL-2.0-only */
/* r8169.h: RealTek 8169/8168/8101 ethernet driver.
 *
 * Copyright (c) 2002 ShuChen <shuchen@realtek.com.tw>
 * Copyright (c) 2003 - 2007 Francois Romieu <romieu@fr.zoreil.com>
 * Copyright (c) a lot of people too. Please respect their work.
 *
 * See MAINTAINERS file for support contact information.
 */

#include <linux/types.h>
#include <linux/phy.h>

struct rtl8169_private;

void r8169_apply_firmware(struct rtl8169_private *tp);
u16 rtl8168h_2_get_adc_bias_ioffset(struct rtl8169_private *tp);
u8 rtl8168d_efuse_read(struct rtl8169_private *tp, int reg_addr);
void r8169_hw_phy_config(struct rtl8169_private *tp, struct phy_device *phydev);
