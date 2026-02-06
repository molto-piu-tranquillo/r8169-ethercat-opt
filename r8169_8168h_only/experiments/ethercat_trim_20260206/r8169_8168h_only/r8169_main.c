// SPDX-License-Identifier: GPL-2.0-only
/*
 * r8169.c: RealTek 8169/8168/8101 ethernet driver.
 *
 * Copyright (c) 2002 ShuChen <shuchen@realtek.com.tw>
 * Copyright (c) 2003 - 2007 Francois Romieu <romieu@fr.zoreil.com>
 * Copyright (c) a lot of people too. Please respect their work.
 *
 * See MAINTAINERS file for support contact information.
 */

#include <linux/module.h>
#include <linux/pci.h>
#include <linux/netdevice.h>
#include <linux/etherdevice.h>
#include <linux/delay.h>
#include <linux/ethtool.h>
#include <linux/phy.h>
#include <linux/if_vlan.h>
#include <linux/in.h>
#include <linux/io.h>
#include <linux/ip.h>
#include <linux/tcp.h>
#include <linux/interrupt.h>
#include <linux/dma-mapping.h>
#include <linux/bitfield.h>
#include <linux/prefetch.h>
#include <linux/ipv6.h>
#include <asm/unaligned.h>
#include <net/ip6_checksum.h>
#include <net/netdev_queues.h>

#include "r8169.h"
#include "r8169_firmware.h"

#define FIRMWARE_8168H_2	"rtl_nic/rtl8168h-2.fw"

#define TX_DMA_BURST	7	/* Maximum PCI burst, '7' is unlimited */
#define InterFrameGap	0x03	/* 3 means InterFrameGap = the shortest one */

#define R8169_RX_BUF_SIZE	(SZ_16K - 1)
#define NUM_TX_DESC	256	/* Number of Tx descriptor registers */
#define NUM_RX_DESC	256	/* Number of Rx descriptor registers */
#define R8169_TX_RING_BYTES	(NUM_TX_DESC * sizeof(struct TxDesc))
#define R8169_RX_RING_BYTES	(NUM_RX_DESC * sizeof(struct RxDesc))
#define R8169_TX_STOP_THRS	(MAX_SKB_FRAGS + 1)
#define R8169_TX_START_THRS	(2 * R8169_TX_STOP_THRS)

#define OCP_STD_PHY_BASE	0xa400

/* write/read MMIO register */
#define RTL_W8(tp, reg, val8)	writeb((val8), tp->mmio_addr + (reg))
#define RTL_W16(tp, reg, val16)	writew((val16), tp->mmio_addr + (reg))
#define RTL_W32(tp, reg, val32)	writel((val32), tp->mmio_addr + (reg))
#define RTL_R8(tp, reg)		readb(tp->mmio_addr + (reg))
#define RTL_R16(tp, reg)		readw(tp->mmio_addr + (reg))
#define RTL_R32(tp, reg)		readl(tp->mmio_addr + (reg))

static const struct {
	const char *name;
	const char *fw_name;
} rtl_chip_info = {
	.name = "RTL8168h/8111h",
	.fw_name = FIRMWARE_8168H_2,
};

static const struct pci_device_id rtl8169_pci_tbl[] = {
	{ PCI_VDEVICE(REALTEK,	0x8168) },
	{}
};

MODULE_DEVICE_TABLE(pci, rtl8169_pci_tbl);

enum rtl_registers {
	MAC0		= 0,	/* Ethernet hardware address. */
	MAC4		= 4,
	MAR0		= 8,	/* Multicast filter. */
	CounterAddrLow		= 0x10,
	CounterAddrHigh		= 0x14,
	TxDescStartAddrLow	= 0x20,
	TxDescStartAddrHigh	= 0x24,
	TxHDescStartAddrLow	= 0x28,
	TxHDescStartAddrHigh	= 0x2c,
	FLASH		= 0x30,
	ERSR		= 0x36,
	ChipCmd		= 0x37,
	TxPoll		= 0x38,
	IntrMask	= 0x3c,
	IntrStatus	= 0x3e,

	TxConfig	= 0x40,
#define	TXCFG_AUTO_FIFO			(1 << 7)	/* 8111e-vl */
#define	TXCFG_EMPTY			(1 << 11)	/* 8111e-vl */

	RxConfig	= 0x44,
#define	RX128_INT_EN			(1 << 15)	/* 8111c and later */
#define	RX_MULTI_EN			(1 << 14)	/* 8111c only */
#define	RXCFG_FIFO_SHIFT		13
					/* No threshold before first PCI xfer */
#define	RX_FIFO_THRESH			(7 << RXCFG_FIFO_SHIFT)
#define	RX_EARLY_OFF			(1 << 11)
#define	RX_PAUSE_SLOT_ON		(1 << 11)	/* 8125b and later */
#define	RXCFG_DMA_SHIFT			8
					/* Unlimited maximum PCI burst. */
#define	RX_DMA_BURST			(7 << RXCFG_DMA_SHIFT)

	Cfg9346		= 0x50,
	Config0		= 0x51,
	Config1		= 0x52,
	Config2		= 0x53,
#define PME_SIGNAL			(1 << 5)	/* 8168c and later */

	Config3		= 0x54,
	Config4		= 0x55,
	Config5		= 0x56,
	PHYAR		= 0x60,
	PHYstatus	= 0x6c,
	RxMaxSize	= 0xda,
	CPlusCmd	= 0xe0,
	IntrMitigate	= 0xe2,

#define RTL_COALESCE_TX_USECS	GENMASK(15, 12)
#define RTL_COALESCE_TX_FRAMES	GENMASK(11, 8)
#define RTL_COALESCE_RX_USECS	GENMASK(7, 4)
#define RTL_COALESCE_RX_FRAMES	GENMASK(3, 0)

#define RTL_COALESCE_T_MAX	0x0fU
#define RTL_COALESCE_FRAME_MAX	(RTL_COALESCE_T_MAX * 4)

	RxDescAddrLow	= 0xe4,
	RxDescAddrHigh	= 0xe8,
	EarlyTxThres	= 0xec,	/* 8169. Unit of 32 bytes. */

#define NoEarlyTx	0x3f	/* Max value : no early transmit. */

	MaxTxPacketSize	= 0xec,	/* 8101/8168. Unit of 128 bytes. */

#define TxPacketMax	(8064 >> 7)
#define EarlySize	0x27

	FuncEvent	= 0xf0,
	FuncEventMask	= 0xf4,
	FuncPresetState	= 0xf8,
	IBCR0           = 0xf8,
	IBCR2           = 0xf9,
	IBIMR0          = 0xfa,
	IBISR0          = 0xfb,
	FuncForceEvent	= 0xfc,
};

enum rtl8168_8101_registers {
	CSIDR			= 0x64,
	CSIAR			= 0x68,
#define	CSIAR_FLAG			0x80000000
#define	CSIAR_WRITE_CMD			0x80000000
#define	CSIAR_BYTE_ENABLE		0x0000f000
#define	CSIAR_ADDR_MASK			0x00000fff
	PMCH			= 0x6f,
#define D3COLD_NO_PLL_DOWN		BIT(7)
#define D3HOT_NO_PLL_DOWN		BIT(6)
#define D3_NO_PLL_DOWN			(BIT(7) | BIT(6))
	EPHYAR			= 0x80,
#define	EPHYAR_FLAG			0x80000000
#define	EPHYAR_WRITE_CMD		0x80000000
#define	EPHYAR_REG_MASK			0x1f
#define	EPHYAR_REG_SHIFT		16
#define	EPHYAR_DATA_MASK		0xffff
	DLLPR			= 0xd0,
#define	PFM_EN				(1 << 6)
#define	TX_10M_PS_EN			(1 << 7)
	DBG_REG			= 0xd1,
#define	FIX_NAK_1			(1 << 4)
#define	FIX_NAK_2			(1 << 3)
	TWSI			= 0xd2,
	MCU			= 0xd3,
#define	NOW_IS_OOB			(1 << 7)
#define	TX_EMPTY			(1 << 5)
#define	RX_EMPTY			(1 << 4)
#define	RXTX_EMPTY			(TX_EMPTY | RX_EMPTY)
#define	EN_NDP				(1 << 3)
#define	EN_OOB_RESET			(1 << 2)
#define	LINK_LIST_RDY			(1 << 1)
	EFUSEAR			= 0xdc,
#define	EFUSEAR_FLAG			0x80000000
#define	EFUSEAR_WRITE_CMD		0x80000000
#define	EFUSEAR_READ_CMD		0x00000000
#define	EFUSEAR_REG_MASK		0x03ff
#define	EFUSEAR_REG_SHIFT		8
#define	EFUSEAR_DATA_MASK		0xff
	MISC_1			= 0xf2,
#define	PFM_D3COLD_EN			(1 << 6)
};

enum rtl8168_registers {
	LED_CTRL		= 0x18,
	LED_FREQ		= 0x1a,
	EEE_LED			= 0x1b,
	ERIDR			= 0x70,
	ERIAR			= 0x74,
#define ERIAR_FLAG			0x80000000
#define ERIAR_WRITE_CMD			0x80000000
#define ERIAR_READ_CMD			0x00000000
#define ERIAR_ADDR_BYTE_ALIGN		4
#define ERIAR_TYPE_SHIFT		16
#define ERIAR_EXGMAC			(0x00 << ERIAR_TYPE_SHIFT)
#define ERIAR_MSIX			(0x01 << ERIAR_TYPE_SHIFT)
#define ERIAR_ASF			(0x02 << ERIAR_TYPE_SHIFT)
#define ERIAR_OOB			(0x02 << ERIAR_TYPE_SHIFT)
#define ERIAR_MASK_SHIFT		12
#define ERIAR_MASK_0001			(0x1 << ERIAR_MASK_SHIFT)
#define ERIAR_MASK_0011			(0x3 << ERIAR_MASK_SHIFT)
#define ERIAR_MASK_0100			(0x4 << ERIAR_MASK_SHIFT)
#define ERIAR_MASK_0101			(0x5 << ERIAR_MASK_SHIFT)
#define ERIAR_MASK_1111			(0xf << ERIAR_MASK_SHIFT)
	EPHY_RXER_NUM		= 0x7c,
	OCPDR			= 0xb0,	/* OCP GPHY access */
#define OCPDR_WRITE_CMD			0x80000000
#define OCPDR_READ_CMD			0x00000000
#define OCPDR_REG_MASK			0x7f
#define OCPDR_GPHY_REG_SHIFT		16
#define OCPDR_DATA_MASK			0xffff
	OCPAR			= 0xb4,
#define OCPAR_FLAG			0x80000000
#define OCPAR_GPHY_WRITE_CMD		0x8000f060
#define OCPAR_GPHY_READ_CMD		0x0000f060
	GPHY_OCP		= 0xb8,
	RDSAR1			= 0xd0,	/* 8168c only. Undocumented on 8168dp */
	MISC			= 0xf0,	/* 8168e only. */
#define TXPLA_RST			(1 << 29)
#define DISABLE_LAN_EN			(1 << 23) /* Enable GPIO pin */
#define PWM_EN				(1 << 22)
#define RXDV_GATED_EN			(1 << 19)
#define EARLY_TALLY_EN			(1 << 16)
};

enum rtl8125_registers {
	IntrMask_8125		= 0x38,
	IntrStatus_8125		= 0x3c,
	TxPoll_8125		= 0x90,
	MAC0_BKP		= 0x19e0,
	EEE_TXIDLE_TIMER_8125	= 0x6048,
};

#define RX_VLAN_INNER_8125	BIT(22)
#define RX_VLAN_OUTER_8125	BIT(23)
#define RX_VLAN_8125		(RX_VLAN_INNER_8125 | RX_VLAN_OUTER_8125)

#define RX_FETCH_DFLT_8125	(8 << 27)

enum rtl_register_content {
	/* InterruptStatusBits */
	SYSErr		= 0x8000,
	PCSTimeout	= 0x4000,
	SWInt		= 0x0100,
	TxDescUnavail	= 0x0080,
	RxFIFOOver	= 0x0040,
	LinkChg		= 0x0020,
	RxOverflow	= 0x0010,
	TxErr		= 0x0008,
	TxOK		= 0x0004,
	RxErr		= 0x0002,
	RxOK		= 0x0001,

	/* RxStatusDesc */
	RxRWT	= (1 << 22),
	RxRES	= (1 << 21),
	RxRUNT	= (1 << 20),
	RxCRC	= (1 << 19),

	/* ChipCmdBits */
	StopReq		= 0x80,
	CmdReset	= 0x10,
	CmdRxEnb	= 0x08,
	CmdTxEnb	= 0x04,
	RxBufEmpty	= 0x01,

	/* TXPoll register p.5 */
	HPQ		= 0x80,		/* Poll cmd on the high prio queue */
	NPQ		= 0x40,		/* Poll cmd on the low prio queue */
	FSWInt		= 0x01,		/* Forced software interrupt */

	/* Cfg9346Bits */
	Cfg9346_Lock	= 0x00,
	Cfg9346_Unlock	= 0xc0,

	/* rx_mode_bits */
	AcceptErr	= 0x20,
	AcceptRunt	= 0x10,
#define RX_CONFIG_ACCEPT_ERR_MASK	0x30
	AcceptBroadcast	= 0x08,
	AcceptMulticast	= 0x04,
	AcceptMyPhys	= 0x02,
	AcceptAllPhys	= 0x01,
#define RX_CONFIG_ACCEPT_OK_MASK	0x0f
#define RX_CONFIG_ACCEPT_MASK		0x3f

	/* TxConfigBits */
	TxInterFrameGapShift = 24,
	TxDMAShift = 8,	/* DMA burst value (0-7) is shift this many bits */

	/* Config1 register p.24 */
	LEDS1		= (1 << 7),
	LEDS0		= (1 << 6),
	Speed_down	= (1 << 4),
	MEMMAP		= (1 << 3),
	IOMAP		= (1 << 2),
	VPD		= (1 << 1),
	PMEnable	= (1 << 0),	/* Power Management Enable */

	/* Config2 register p. 25 */
	ClkReqEn	= (1 << 7),	/* Clock Request Enable */
	MSIEnable	= (1 << 5),	/* 8169 only. Reserved in the 8168. */
	PCI_Clock_66MHz = 0x01,
	PCI_Clock_33MHz = 0x00,

	/* Config3 register p.25 */
	MagicPacket	= (1 << 5),	/* Wake up when receives a Magic Packet */
	LinkUp		= (1 << 4),	/* Wake up when the cable connection is re-established */
	Jumbo_En0	= (1 << 2),	/* 8168 only. Reserved in the 8168b */
	Rdy_to_L23	= (1 << 1),	/* L23 Enable */
	Beacon_en	= (1 << 0),	/* 8168 only. Reserved in the 8168b */

	/* Config4 register */
	Jumbo_En1	= (1 << 1),	/* 8168 only. Reserved in the 8168b */

	/* Config5 register p.27 */
	BWF		= (1 << 6),	/* Accept Broadcast wakeup frame */
	MWF		= (1 << 5),	/* Accept Multicast wakeup frame */
	UWF		= (1 << 4),	/* Accept Unicast wakeup frame */
	Spi_en		= (1 << 3),
	LanWake		= (1 << 1),	/* LanWake enable/disable */
	PMEStatus	= (1 << 0),	/* PME status can be reset by PCI RST# */
	ASPM_en		= (1 << 0),	/* ASPM enable */

	/* CPlusCmd p.31 */
	EnableBist	= (1 << 15),	// 8168 8101
	Mac_dbgo_oe	= (1 << 14),	// 8168 8101
	EnAnaPLL	= (1 << 14),	// 8169
	Normal_mode	= (1 << 13),	// unused
	Force_half_dup	= (1 << 12),	// 8168 8101
	Force_rxflow_en	= (1 << 11),	// 8168 8101
	Force_txflow_en	= (1 << 10),	// 8168 8101
	Cxpl_dbg_sel	= (1 << 9),	// 8168 8101
	ASF		= (1 << 8),	// 8168 8101
	PktCntrDisable	= (1 << 7),	// 8168 8101
	Mac_dbgo_sel	= 0x001c,	// 8168
	RxVlan		= (1 << 6),
	RxChkSum	= (1 << 5),
	PCIDAC		= (1 << 4),
	PCIMulRW	= (1 << 3),
#define INTT_MASK	GENMASK(1, 0)
#define CPCMD_MASK	(Normal_mode | RxVlan | RxChkSum | INTT_MASK)

	/* rtl8169_PHYstatus */
	TBI_Enable	= 0x80,
	TxFlowCtrl	= 0x40,
	RxFlowCtrl	= 0x20,
	_1000bpsF	= 0x10,
	_100bps		= 0x08,
	_10bps		= 0x04,
	LinkStatus	= 0x02,
	FullDup		= 0x01,

	/* ResetCounterCommand */
	CounterReset	= 0x1,

	/* DumpCounterCommand */
	CounterDump	= 0x8,

	/* magic enable v2 */
	MagicPacket_v2	= (1 << 16),	/* Wake up when receives a Magic Packet */
};

enum rtl_desc_bit {
	/* First doubleword. */
	DescOwn		= (1 << 31), /* Descriptor is owned by NIC */
	RingEnd		= (1 << 30), /* End of descriptor ring */
	FirstFrag	= (1 << 29), /* First segment of a packet */
	LastFrag	= (1 << 28), /* Final segment of a packet */
};

/* Generic case. */
enum rtl_tx_desc_bit {
	/* First doubleword. */
	TD_LSO		= (1 << 27),		/* Large Send Offload */
#define TD_MSS_MAX			0x07ffu	/* MSS value */

	/* Second doubleword. */
	TxVlanTag	= (1 << 17),		/* Add VLAN tag */
};

/* 8169, 8168b and 810x except 8102e. */
enum rtl_tx_desc_bit_0 {
	/* First doubleword. */
#define TD0_MSS_SHIFT			16	/* MSS position (11 bits) */
	TD0_TCP_CS	= (1 << 16),		/* Calculate TCP/IP checksum */
	TD0_UDP_CS	= (1 << 17),		/* Calculate UDP/IP checksum */
	TD0_IP_CS	= (1 << 18),		/* Calculate IP checksum */
};

/* 8102e, 8168c and beyond. */
enum rtl_tx_desc_bit_1 {
	/* First doubleword. */
	TD1_GTSENV4	= (1 << 26),		/* Giant Send for IPv4 */
	TD1_GTSENV6	= (1 << 25),		/* Giant Send for IPv6 */
#define GTTCPHO_SHIFT			18
#define GTTCPHO_MAX			0x7f

	/* Second doubleword. */
#define TCPHO_SHIFT			18
#define TCPHO_MAX			0x3ff
#define TD1_MSS_SHIFT			18	/* MSS position (11 bits) */
	TD1_IPv6_CS	= (1 << 28),		/* Calculate IPv6 checksum */
	TD1_IPv4_CS	= (1 << 29),		/* Calculate IPv4 checksum */
	TD1_TCP_CS	= (1 << 30),		/* Calculate TCP/IP checksum */
	TD1_UDP_CS	= (1 << 31),		/* Calculate UDP/IP checksum */
};

enum rtl_rx_desc_bit {
	/* Rx private */
	PID1		= (1 << 18), /* Protocol ID bit 1/2 */
	PID0		= (1 << 17), /* Protocol ID bit 0/2 */

#define RxProtoUDP	(PID1)
#define RxProtoTCP	(PID0)
#define RxProtoIP	(PID1 | PID0)
#define RxProtoMask	RxProtoIP

	IPFail		= (1 << 16), /* IP checksum failed */
	UDPFail		= (1 << 15), /* UDP/IP checksum failed */
	TCPFail		= (1 << 14), /* TCP/IP checksum failed */

#define RxCSFailMask	(IPFail | UDPFail | TCPFail)

	RxVlanTag	= (1 << 16), /* VLAN tag available */
};

#define RTL_GSO_MAX_SIZE_V1	32000
#define RTL_GSO_MAX_SEGS_V1	24
#define RTL_GSO_MAX_SIZE_V2	64000
#define RTL_GSO_MAX_SEGS_V2	64

struct TxDesc {
	__le32 opts1;
	__le32 opts2;
	__le64 addr;
};

struct RxDesc {
	__le32 opts1;
	__le32 opts2;
	__le64 addr;
};

struct ring_info {
	struct sk_buff	*skb;
	u32		len;
};

struct rtl8169_counters {
	__le64	tx_packets;
	__le64	rx_packets;
	__le64	tx_errors;
	__le32	rx_errors;
	__le16	rx_missed;
	__le16	align_errors;
	__le32	tx_one_collision;
	__le32	tx_multi_collision;
	__le64	rx_unicast;
	__le64	rx_broadcast;
	__le32	rx_multicast;
	__le16	tx_aborted;
	__le16	tx_underun;
};

struct rtl8169_tc_offsets {
	bool	inited;
	__le64	tx_errors;
	__le32	tx_multi_collision;
	__le16	tx_aborted;
	__le16	rx_missed;
};

enum rtl_flag {
	RTL_FLAG_TASK_ENABLED = 0,
	RTL_FLAG_TASK_RESET_PENDING,
	RTL_FLAG_TASK_RESET_NO_QUEUE_WAKE,
	RTL_FLAG_TASK_TX_TIMEOUT,
	RTL_FLAG_MAX
};

struct rtl8169_private {
	void __iomem *mmio_addr;	/* memory map physical address */
	struct pci_dev *pci_dev;
	struct net_device *dev;
	struct phy_device *phydev;
	struct napi_struct napi;
	u32 cur_rx; /* Index into the Rx descriptor buffer of next Rx pkt. */
	u32 cur_tx; /* Index into the Tx descriptor buffer of next Rx pkt. */
	u32 dirty_tx;
	struct TxDesc *TxDescArray;	/* 256-aligned Tx descriptor ring */
	struct RxDesc *RxDescArray;	/* 256-aligned Rx descriptor ring */
	dma_addr_t TxPhyAddr;
	dma_addr_t RxPhyAddr;
	struct page *Rx_databuff[NUM_RX_DESC];	/* Rx data buffers */
	struct ring_info tx_skb[NUM_TX_DESC];	/* Tx data buffers */
	u16 cp_cmd;
	u32 irq_mask;
	int irq;

	struct {
		DECLARE_BITMAP(flags, RTL_FLAG_MAX);
		struct work_struct work;
	} wk;

	raw_spinlock_t config25_lock;
	raw_spinlock_t mac_ocp_lock;

	raw_spinlock_t cfg9346_usage_lock;
	int cfg9346_usage_count;

	unsigned supports_gmii:1;
	unsigned aspm_manageable:1;
	dma_addr_t counters_phys_addr;
	struct rtl8169_counters *counters;
	struct rtl8169_tc_offsets tc_offset;

	const char *fw_name;
	struct rtl_fw *rtl_fw;

	u32 ocp_base;
};

MODULE_AUTHOR("Realtek and the Linux r8169 crew <netdev@vger.kernel.org>");
MODULE_DESCRIPTION("RealTek RTL8168H-only Ethernet driver");
MODULE_SOFTDEP("pre: realtek");
MODULE_LICENSE("GPL");
MODULE_FIRMWARE(FIRMWARE_8168H_2);

static inline struct device *tp_to_dev(struct rtl8169_private *tp)
{
	return &tp->pci_dev->dev;
}

static void rtl_lock_config_regs(struct rtl8169_private *tp)
{
	unsigned long flags;

	raw_spin_lock_irqsave(&tp->cfg9346_usage_lock, flags);
	if (!--tp->cfg9346_usage_count)
		RTL_W8(tp, Cfg9346, Cfg9346_Lock);
	raw_spin_unlock_irqrestore(&tp->cfg9346_usage_lock, flags);
}

static void rtl_unlock_config_regs(struct rtl8169_private *tp)
{
	unsigned long flags;

	raw_spin_lock_irqsave(&tp->cfg9346_usage_lock, flags);
	if (!tp->cfg9346_usage_count++)
		RTL_W8(tp, Cfg9346, Cfg9346_Unlock);
	raw_spin_unlock_irqrestore(&tp->cfg9346_usage_lock, flags);
}

static void rtl_pci_commit(struct rtl8169_private *tp)
{
	/* Read an arbitrary register to commit a preceding PCI write */
	RTL_R8(tp, ChipCmd);
}

static void rtl_mod_config2(struct rtl8169_private *tp, u8 clear, u8 set)
{
	unsigned long flags;
	u8 val;

	raw_spin_lock_irqsave(&tp->config25_lock, flags);
	val = RTL_R8(tp, Config2);
	RTL_W8(tp, Config2, (val & ~clear) | set);
	raw_spin_unlock_irqrestore(&tp->config25_lock, flags);
}

static void rtl_mod_config5(struct rtl8169_private *tp, u8 clear, u8 set)
{
	unsigned long flags;
	u8 val;

	raw_spin_lock_irqsave(&tp->config25_lock, flags);
	val = RTL_R8(tp, Config5);
	RTL_W8(tp, Config5, (val & ~clear) | set);
	raw_spin_unlock_irqrestore(&tp->config25_lock, flags);
}

static void rtl_read_mac_from_reg(struct rtl8169_private *tp, u8 *mac, int reg)
{
	int i;

	for (i = 0; i < ETH_ALEN; i++)
		mac[i] = RTL_R8(tp, reg + i);
}

struct rtl_cond {
	bool (*check)(struct rtl8169_private *);
	const char *msg;
};

static bool rtl_loop_wait(struct rtl8169_private *tp, const struct rtl_cond *c,
			  unsigned long usecs, int n, bool high)
{
	int i;

	for (i = 0; i < n; i++) {
		if (c->check(tp) == high)
			return true;
		fsleep(usecs);
	}

	if (net_ratelimit())
		netdev_err(tp->dev, "%s == %d (loop: %d, delay: %lu).\n",
			   c->msg, !high, n, usecs);
	return false;
}

static bool rtl_loop_wait_high(struct rtl8169_private *tp,
			       const struct rtl_cond *c,
			       unsigned long d, int n)
{
	return rtl_loop_wait(tp, c, d, n, true);
}

static bool rtl_loop_wait_low(struct rtl8169_private *tp,
			      const struct rtl_cond *c,
			      unsigned long d, int n)
{
	return rtl_loop_wait(tp, c, d, n, false);
}

#define DECLARE_RTL_COND(name)				\
static bool name ## _check(struct rtl8169_private *);	\
							\
static const struct rtl_cond name = {			\
	.check	= name ## _check,			\
	.msg	= #name					\
};							\
							\
static bool name ## _check(struct rtl8169_private *tp)

DECLARE_RTL_COND(rtl_eriar_cond)
{
	return RTL_R32(tp, ERIAR) & ERIAR_FLAG;
}

static void _rtl_eri_write(struct rtl8169_private *tp, int addr, u32 mask,
			   u32 val, int type)
{
	u32 cmd = ERIAR_WRITE_CMD | type | mask | addr;

	if (WARN(addr & 3 || !mask, "addr: 0x%x, mask: 0x%08x\n", addr, mask))
		return;

	RTL_W32(tp, ERIDR, val);
	RTL_W32(tp, ERIAR, cmd);

	rtl_loop_wait_low(tp, &rtl_eriar_cond, 100, 100);
}

static void rtl_eri_write(struct rtl8169_private *tp, int addr, u32 mask,
			  u32 val)
{
	_rtl_eri_write(tp, addr, mask, val, ERIAR_EXGMAC);
}

static u32 _rtl_eri_read(struct rtl8169_private *tp, int addr, int type)
{
	u32 cmd = ERIAR_READ_CMD | type | ERIAR_MASK_1111 | addr;

	RTL_W32(tp, ERIAR, cmd);

	return rtl_loop_wait_high(tp, &rtl_eriar_cond, 100, 100) ?
		RTL_R32(tp, ERIDR) : ~0;
}

static u32 rtl_eri_read(struct rtl8169_private *tp, int addr)
{
	return _rtl_eri_read(tp, addr, ERIAR_EXGMAC);
}

static void rtl_w0w1_eri(struct rtl8169_private *tp, int addr, u32 p, u32 m)
{
	u32 val = rtl_eri_read(tp, addr);

	rtl_eri_write(tp, addr, ERIAR_MASK_1111, (val & ~m) | p);
}

static void rtl_eri_set_bits(struct rtl8169_private *tp, int addr, u32 p)
{
	rtl_w0w1_eri(tp, addr, p, 0);
}

static void rtl_eri_clear_bits(struct rtl8169_private *tp, int addr, u32 m)
{
	rtl_w0w1_eri(tp, addr, 0, m);
}

static bool rtl_ocp_reg_failure(u32 reg)
{
	return WARN_ONCE(reg & 0xffff0001, "Invalid ocp reg %x!\n", reg);
}

DECLARE_RTL_COND(rtl_ocp_gphy_cond)
{
	return RTL_R32(tp, GPHY_OCP) & OCPAR_FLAG;
}

static void r8168_phy_ocp_write(struct rtl8169_private *tp, u32 reg, u32 data)
{
	if (rtl_ocp_reg_failure(reg))
		return;

	RTL_W32(tp, GPHY_OCP, OCPAR_FLAG | (reg << 15) | data);

	rtl_loop_wait_low(tp, &rtl_ocp_gphy_cond, 25, 10);
}

static int r8168_phy_ocp_read(struct rtl8169_private *tp, u32 reg)
{
	if (rtl_ocp_reg_failure(reg))
		return 0;

	RTL_W32(tp, GPHY_OCP, reg << 15);

	return rtl_loop_wait_high(tp, &rtl_ocp_gphy_cond, 25, 10) ?
		(RTL_R32(tp, GPHY_OCP) & 0xffff) : -ETIMEDOUT;
}

static void __r8168_mac_ocp_write(struct rtl8169_private *tp, u32 reg, u32 data)
{
	if (rtl_ocp_reg_failure(reg))
		return;

	RTL_W32(tp, OCPDR, OCPAR_FLAG | (reg << 15) | data);
}

static void r8168_mac_ocp_write(struct rtl8169_private *tp, u32 reg, u32 data)
{
	unsigned long flags;

	raw_spin_lock_irqsave(&tp->mac_ocp_lock, flags);
	__r8168_mac_ocp_write(tp, reg, data);
	raw_spin_unlock_irqrestore(&tp->mac_ocp_lock, flags);
}

static u16 __r8168_mac_ocp_read(struct rtl8169_private *tp, u32 reg)
{
	if (rtl_ocp_reg_failure(reg))
		return 0;

	RTL_W32(tp, OCPDR, reg << 15);

	return RTL_R32(tp, OCPDR);
}

static u16 r8168_mac_ocp_read(struct rtl8169_private *tp, u32 reg)
{
	unsigned long flags;
	u16 val;

	raw_spin_lock_irqsave(&tp->mac_ocp_lock, flags);
	val = __r8168_mac_ocp_read(tp, reg);
	raw_spin_unlock_irqrestore(&tp->mac_ocp_lock, flags);

	return val;
}

static void r8168_mac_ocp_modify(struct rtl8169_private *tp, u32 reg, u16 mask,
				 u16 set)
{
	unsigned long flags;
	u16 data;

	raw_spin_lock_irqsave(&tp->mac_ocp_lock, flags);
	data = __r8168_mac_ocp_read(tp, reg);
	__r8168_mac_ocp_write(tp, reg, (data & ~mask) | set);
	raw_spin_unlock_irqrestore(&tp->mac_ocp_lock, flags);
}

static void r8168g_mdio_write(struct rtl8169_private *tp, int reg, int value)
{
	if (reg == 0x1f) {
		tp->ocp_base = value ? value << 4 : OCP_STD_PHY_BASE;
		return;
	}

	if (tp->ocp_base != OCP_STD_PHY_BASE)
		reg -= 0x10;

	r8168_phy_ocp_write(tp, tp->ocp_base + reg * 2, value);
}

static int r8168g_mdio_read(struct rtl8169_private *tp, int reg)
{
	if (reg == 0x1f)
		return tp->ocp_base == OCP_STD_PHY_BASE ? 0 : tp->ocp_base >> 4;

	if (tp->ocp_base != OCP_STD_PHY_BASE)
		reg -= 0x10;

	return r8168_phy_ocp_read(tp, tp->ocp_base + reg * 2);
}

static void mac_mcu_write(struct rtl8169_private *tp, int reg, int value)
{
	if (reg == 0x1f) {
		tp->ocp_base = value << 4;
		return;
	}

	r8168_mac_ocp_write(tp, tp->ocp_base + reg, value);
}

static int mac_mcu_read(struct rtl8169_private *tp, int reg)
{
	return r8168_mac_ocp_read(tp, tp->ocp_base + reg);
}

DECLARE_RTL_COND(rtl_ocpar_cond)
{
	return RTL_R32(tp, OCPAR) & OCPAR_FLAG;
}

static void rtl_writephy(struct rtl8169_private *tp, int location, int val)
{
	r8168g_mdio_write(tp, location, val);
}

static int rtl_readphy(struct rtl8169_private *tp, int location)
{
	return r8168g_mdio_read(tp, location);
}

DECLARE_RTL_COND(rtl_ephyar_cond)
{
	return RTL_R32(tp, EPHYAR) & EPHYAR_FLAG;
}

static void rtl_ephy_write(struct rtl8169_private *tp, int reg_addr, int value)
{
	RTL_W32(tp, EPHYAR, EPHYAR_WRITE_CMD | (value & EPHYAR_DATA_MASK) |
		(reg_addr & EPHYAR_REG_MASK) << EPHYAR_REG_SHIFT);

	rtl_loop_wait_low(tp, &rtl_ephyar_cond, 10, 100);

	udelay(10);
}

static u16 rtl_ephy_read(struct rtl8169_private *tp, int reg_addr)
{
	RTL_W32(tp, EPHYAR, (reg_addr & EPHYAR_REG_MASK) << EPHYAR_REG_SHIFT);

	return rtl_loop_wait_high(tp, &rtl_ephyar_cond, 10, 100) ?
		RTL_R32(tp, EPHYAR) & EPHYAR_DATA_MASK : ~0;
}

static void rtl_reset_packet_filter(struct rtl8169_private *tp)
{
	rtl_eri_clear_bits(tp, 0xdc, BIT(0));
	rtl_eri_set_bits(tp, 0xdc, BIT(0));
}

DECLARE_RTL_COND(rtl_efusear_cond)
{
	return RTL_R32(tp, EFUSEAR) & EFUSEAR_FLAG;
}

u8 rtl8168d_efuse_read(struct rtl8169_private *tp, int reg_addr)
{
	RTL_W32(tp, EFUSEAR, (reg_addr & EFUSEAR_REG_MASK) << EFUSEAR_REG_SHIFT);

	return rtl_loop_wait_high(tp, &rtl_efusear_cond, 100, 300) ?
		RTL_R32(tp, EFUSEAR) & EFUSEAR_DATA_MASK : ~0;
}

static u32 rtl_get_events(struct rtl8169_private *tp)
{
	return RTL_R16(tp, IntrStatus);
}

static void rtl_ack_events(struct rtl8169_private *tp, u32 bits)
{
	RTL_W16(tp, IntrStatus, bits);
}

static void rtl_irq_disable(struct rtl8169_private *tp)
{
	RTL_W16(tp, IntrMask, 0);
}

static void rtl_irq_enable(struct rtl8169_private *tp)
{
	RTL_W16(tp, IntrMask, tp->irq_mask);
}

static void rtl8169_irq_mask_and_ack(struct rtl8169_private *tp)
{
	rtl_irq_disable(tp);
	rtl_ack_events(tp, 0xffffffff);
	rtl_pci_commit(tp);
}

static void rtl_link_chg_patch(struct rtl8169_private *tp)
{
	(void)tp;
}

static void rtl8169_get_drvinfo(struct net_device *dev,
				struct ethtool_drvinfo *info)
{
	struct rtl8169_private *tp = netdev_priv(dev);
	struct rtl_fw *rtl_fw = tp->rtl_fw;

	strscpy(info->driver, KBUILD_MODNAME, sizeof(info->driver));
	strscpy(info->bus_info, pci_name(tp->pci_dev), sizeof(info->bus_info));
	BUILD_BUG_ON(sizeof(info->fw_version) < sizeof(rtl_fw->version));
	if (rtl_fw)
		strscpy(info->fw_version, rtl_fw->version,
			sizeof(info->fw_version));
}

static void rtl_set_rx_config_defaults(struct rtl8169_private *tp)
{
	u32 rx_config = RTL_R32(tp, RxConfig);

	rx_config &= ~RX_CONFIG_ACCEPT_ERR_MASK;

	RTL_W32(tp, RxConfig, rx_config);
}

static const char rtl8169_gstrings[][ETH_GSTRING_LEN] = {
	"tx_packets",
	"rx_packets",
	"tx_errors",
	"rx_errors",
	"rx_missed",
	"align_errors",
	"tx_single_collisions",
	"tx_multi_collisions",
	"unicast",
	"broadcast",
	"multicast",
	"tx_aborted",
	"tx_underrun",
};

static int rtl8169_get_sset_count(struct net_device *dev, int sset)
{
	switch (sset) {
	case ETH_SS_STATS:
		return ARRAY_SIZE(rtl8169_gstrings);
	default:
		return -EOPNOTSUPP;
	}
}

DECLARE_RTL_COND(rtl_counters_cond)
{
	return RTL_R32(tp, CounterAddrLow) & (CounterReset | CounterDump);
}

static void rtl8169_do_counters(struct rtl8169_private *tp, u32 counter_cmd)
{
	u32 cmd = lower_32_bits(tp->counters_phys_addr);

	RTL_W32(tp, CounterAddrHigh, upper_32_bits(tp->counters_phys_addr));
	rtl_pci_commit(tp);
	RTL_W32(tp, CounterAddrLow, cmd);
	RTL_W32(tp, CounterAddrLow, cmd | counter_cmd);

	rtl_loop_wait_low(tp, &rtl_counters_cond, 10, 1000);
}

static void rtl8169_update_counters(struct rtl8169_private *tp)
{
	u8 val = RTL_R8(tp, ChipCmd);

	/*
	 * Some chips are unable to dump tally counters when the receiver
	 * is disabled. If 0xff chip may be in a PCI power-save state.
	 */
	if (val & CmdRxEnb && val != 0xff)
		rtl8169_do_counters(tp, CounterDump);
}

static void rtl8169_init_counter_offsets(struct rtl8169_private *tp)
{
	if (tp->tc_offset.inited)
		return;

	rtl8169_do_counters(tp, CounterReset);

	tp->tc_offset.inited = true;
}

static void rtl8169_get_ethtool_stats(struct net_device *dev,
				      struct ethtool_stats *stats, u64 *data)
{
	struct rtl8169_private *tp = netdev_priv(dev);
	struct rtl8169_counters *counters;

	counters = tp->counters;
	rtl8169_update_counters(tp);

	data[0] = le64_to_cpu(counters->tx_packets);
	data[1] = le64_to_cpu(counters->rx_packets);
	data[2] = le64_to_cpu(counters->tx_errors);
	data[3] = le32_to_cpu(counters->rx_errors);
	data[4] = le16_to_cpu(counters->rx_missed);
	data[5] = le16_to_cpu(counters->align_errors);
	data[6] = le32_to_cpu(counters->tx_one_collision);
	data[7] = le32_to_cpu(counters->tx_multi_collision);
	data[8] = le64_to_cpu(counters->rx_unicast);
	data[9] = le64_to_cpu(counters->rx_broadcast);
	data[10] = le32_to_cpu(counters->rx_multicast);
	data[11] = le16_to_cpu(counters->tx_aborted);
	data[12] = le16_to_cpu(counters->tx_underun);
}

static void rtl8169_get_strings(struct net_device *dev, u32 stringset, u8 *data)
{
	switch(stringset) {
	case ETH_SS_STATS:
		memcpy(data, rtl8169_gstrings, sizeof(rtl8169_gstrings));
		break;
	}
}

static const struct ethtool_ops rtl8169_ethtool_ops = {
	.get_drvinfo		= rtl8169_get_drvinfo,
	.get_link		= ethtool_op_get_link,
	.get_strings		= rtl8169_get_strings,
	.get_sset_count		= rtl8169_get_sset_count,
	.get_ethtool_stats	= rtl8169_get_ethtool_stats,
	.get_ts_info		= ethtool_op_get_ts_info,
	.nway_reset		= phy_ethtool_nway_reset,
	.get_link_ksettings	= phy_ethtool_get_link_ksettings,
	.set_link_ksettings	= phy_ethtool_set_link_ksettings,
};

static void rtl_enable_eee(struct rtl8169_private *tp)
{
	struct phy_device *phydev = tp->phydev;
	int adv;

	adv = phy_read_mmd(phydev, MDIO_MMD_PCS, MDIO_PCS_EEE_ABLE);

	if (adv >= 0)
		phy_write_mmd(phydev, MDIO_MMD_AN, MDIO_AN_EEE_ADV, adv);
}

static bool rtl8169_is_supported_xid(u16 xid, bool gmii)
{
	return (xid & 0x7cf) == 0x541 && gmii;
}

static void rtl_release_firmware(struct rtl8169_private *tp)
{
	if (tp->rtl_fw) {
		rtl_fw_release_firmware(tp->rtl_fw);
		kfree(tp->rtl_fw);
		tp->rtl_fw = NULL;
	}
}

void r8169_apply_firmware(struct rtl8169_private *tp)
{
	int val;

	/* TODO: release firmware if rtl_fw_write_firmware signals failure. */
	if (tp->rtl_fw) {
		rtl_fw_write_firmware(tp, tp->rtl_fw);
		/* At least one firmware doesn't reset tp->ocp_base. */
		tp->ocp_base = OCP_STD_PHY_BASE;

		/* PHY soft reset may still be in progress */
		phy_read_poll_timeout(tp->phydev, MII_BMCR, val,
				      !(val & BMCR_RESET),
				      50000, 600000, true);
	}
}

static void rtl8168_config_eee_mac(struct rtl8169_private *tp)
{
	/* Adjust EEE LED frequency */
	RTL_W8(tp, EEE_LED, RTL_R8(tp, EEE_LED) & ~0x07);

	rtl_eri_set_bits(tp, 0x1b0, 0x0003);
}

u16 rtl8168h_2_get_adc_bias_ioffset(struct rtl8169_private *tp)
{
	u16 data1, data2, ioffset;

	r8168_mac_ocp_write(tp, 0xdd02, 0x807d);
	data1 = r8168_mac_ocp_read(tp, 0xdd02);
	data2 = r8168_mac_ocp_read(tp, 0xdd00);

	ioffset = (data2 >> 1) & 0x7ff8;
	ioffset |= data2 & 0x0007;
	if (data1 & BIT(7))
		ioffset |= BIT(15);

	return ioffset;
}

static void rtl_schedule_task(struct rtl8169_private *tp, enum rtl_flag flag)
{
	if (!test_bit(RTL_FLAG_TASK_ENABLED, tp->wk.flags))
		return;

	set_bit(flag, tp->wk.flags);
	schedule_work(&tp->wk.work);
}

static void rtl8169_init_phy(struct rtl8169_private *tp)
{
	r8169_hw_phy_config(tp, tp->phydev);

	/* We may have called phy_speed_down before */
	phy_speed_up(tp->phydev);

	rtl_enable_eee(tp);

	genphy_soft_reset(tp->phydev);
}

static void rtl_rar_set(struct rtl8169_private *tp, const u8 *addr)
{
	rtl_unlock_config_regs(tp);

	RTL_W32(tp, MAC4, get_unaligned_le16(addr + 4));
	rtl_pci_commit(tp);

	RTL_W32(tp, MAC0, get_unaligned_le32(addr));
	rtl_pci_commit(tp);

	rtl_lock_config_regs(tp);
}

static int rtl_set_mac_address(struct net_device *dev, void *p)
{
	struct rtl8169_private *tp = netdev_priv(dev);
	int ret;

	ret = eth_mac_addr(dev, p);
	if (ret)
		return ret;

	rtl_rar_set(tp, dev->dev_addr);

	return 0;
}

static void rtl_init_rxcfg(struct rtl8169_private *tp)
{
	RTL_W32(tp, RxConfig,
		RX128_INT_EN | RX_MULTI_EN | RX_DMA_BURST | RX_EARLY_OFF);
}

static void rtl8169_init_ring_indexes(struct rtl8169_private *tp)
{
	tp->dirty_tx = tp->cur_tx = tp->cur_rx = 0;
}

DECLARE_RTL_COND(rtl_chipcmd_cond)
{
	return RTL_R8(tp, ChipCmd) & CmdReset;
}

static void rtl_hw_reset(struct rtl8169_private *tp)
{
	RTL_W8(tp, ChipCmd, CmdReset);

	rtl_loop_wait_low(tp, &rtl_chipcmd_cond, 100, 100);
}

static void rtl_request_firmware(struct rtl8169_private *tp)
{
	struct rtl_fw *rtl_fw;

	/* firmware loaded already or no firmware available */
	if (tp->rtl_fw || !tp->fw_name)
		return;

	rtl_fw = kzalloc(sizeof(*rtl_fw), GFP_KERNEL);
	if (!rtl_fw)
		return;

	rtl_fw->phy_write = rtl_writephy;
	rtl_fw->phy_read = rtl_readphy;
	rtl_fw->mac_mcu_write = mac_mcu_write;
	rtl_fw->mac_mcu_read = mac_mcu_read;
	rtl_fw->fw_name = tp->fw_name;
	rtl_fw->dev = tp_to_dev(tp);

	if (rtl_fw_request_firmware(rtl_fw))
		kfree(rtl_fw);
	else
		tp->rtl_fw = rtl_fw;
}

static void rtl_rx_close(struct rtl8169_private *tp)
{
	RTL_W32(tp, RxConfig, RTL_R32(tp, RxConfig) & ~RX_CONFIG_ACCEPT_MASK);
}

DECLARE_RTL_COND(rtl_txcfg_empty_cond)
{
	return RTL_R32(tp, TxConfig) & TXCFG_EMPTY;
}

DECLARE_RTL_COND(rtl_rxtx_empty_cond)
{
	return (RTL_R8(tp, MCU) & RXTX_EMPTY) == RXTX_EMPTY;
}

static void rtl_wait_txrx_fifo_empty(struct rtl8169_private *tp)
{
	/* rtl8168h/8111h path */
	rtl_loop_wait_high(tp, &rtl_txcfg_empty_cond, 100, 42);
	rtl_loop_wait_high(tp, &rtl_rxtx_empty_cond, 100, 42);
}

static void rtl_disable_rxdvgate(struct rtl8169_private *tp)
{
	RTL_W32(tp, MISC, RTL_R32(tp, MISC) & ~RXDV_GATED_EN);
}

static void rtl_enable_rxdvgate(struct rtl8169_private *tp)
{
	RTL_W32(tp, MISC, RTL_R32(tp, MISC) | RXDV_GATED_EN);
	fsleep(2000);
	rtl_wait_txrx_fifo_empty(tp);
}

static void rtl_set_tx_config_registers(struct rtl8169_private *tp)
{
	u32 val = TX_DMA_BURST << TxDMAShift |
		  InterFrameGap << TxInterFrameGapShift |
		  TXCFG_AUTO_FIFO;

	RTL_W32(tp, TxConfig, val);
}

static void rtl_set_rx_max_size(struct rtl8169_private *tp)
{
	/* Low hurts. Let's disable the filtering. */
	RTL_W16(tp, RxMaxSize, R8169_RX_BUF_SIZE + 1);
}

static void rtl_set_rx_tx_desc_registers(struct rtl8169_private *tp)
{
	/*
	 * Magic spell: some iop3xx ARM board needs the TxDescAddrHigh
	 * register to be written before TxDescAddrLow to work.
	 * Switching from MMIO to I/O access fixes the issue as well.
	 */
	RTL_W32(tp, TxDescStartAddrHigh, ((u64) tp->TxPhyAddr) >> 32);
	RTL_W32(tp, TxDescStartAddrLow, ((u64) tp->TxPhyAddr) & DMA_BIT_MASK(32));
	RTL_W32(tp, RxDescAddrHigh, ((u64) tp->RxPhyAddr) >> 32);
	RTL_W32(tp, RxDescAddrLow, ((u64) tp->RxPhyAddr) & DMA_BIT_MASK(32));
}

static void rtl_set_rx_mode(struct rtl8169_private *tp)
{
	u32 rx_mode = AcceptBroadcast | AcceptMyPhys | AcceptMulticast;
	u32 tmp;

	RTL_W32(tp, MAR0 + 4, 0xffffffff);
	RTL_W32(tp, MAR0 + 0, 0xffffffff);

	tmp = RTL_R32(tp, RxConfig);
	RTL_W32(tp, RxConfig, (tmp & ~RX_CONFIG_ACCEPT_OK_MASK) | rx_mode);
}

DECLARE_RTL_COND(rtl_csiar_cond)
{
	return RTL_R32(tp, CSIAR) & CSIAR_FLAG;
}

static void rtl_csi_write(struct rtl8169_private *tp, int addr, int value)
{
	u32 func = PCI_FUNC(tp->pci_dev->devfn);

	RTL_W32(tp, CSIDR, value);
	RTL_W32(tp, CSIAR, CSIAR_WRITE_CMD | (addr & CSIAR_ADDR_MASK) |
		CSIAR_BYTE_ENABLE | func << 16);

	rtl_loop_wait_low(tp, &rtl_csiar_cond, 10, 100);
}

static u32 rtl_csi_read(struct rtl8169_private *tp, int addr)
{
	u32 func = PCI_FUNC(tp->pci_dev->devfn);

	RTL_W32(tp, CSIAR, (addr & CSIAR_ADDR_MASK) | func << 16 |
		CSIAR_BYTE_ENABLE);

	return rtl_loop_wait_high(tp, &rtl_csiar_cond, 10, 100) ?
		RTL_R32(tp, CSIDR) : ~0;
}

static void rtl_set_aspm_entry_latency(struct rtl8169_private *tp, u8 val)
{
	struct pci_dev *pdev = tp->pci_dev;
	u32 csi;

	/* According to Realtek the value at config space address 0x070f
	 * controls the L0s/L1 entrance latency. We try standard ECAM access
	 * first and if it fails fall back to CSI.
	 * bit 0..2: L0: 0 = 1us, 1 = 2us .. 6 = 7us, 7 = 7us (no typo)
	 * bit 3..5: L1: 0 = 1us, 1 = 2us .. 6 = 64us, 7 = 64us
	 */
	if (pdev->cfg_size > 0x070f &&
	    pci_write_config_byte(pdev, 0x070f, val) == PCIBIOS_SUCCESSFUL)
		return;

	netdev_notice_once(tp->dev,
		"No native access to PCI extended config space, falling back to CSI\n");
	csi = rtl_csi_read(tp, 0x070c) & 0x00ffffff;
	rtl_csi_write(tp, 0x070c, csi | val << 24);
}

static void rtl_set_def_aspm_entry_latency(struct rtl8169_private *tp)
{
	/* L0 7us, L1 16us */
	rtl_set_aspm_entry_latency(tp, 0x27);
}

struct ephy_info {
	unsigned int offset;
	u16 mask;
	u16 bits;
};

static void __rtl_ephy_init(struct rtl8169_private *tp,
			    const struct ephy_info *e, int len)
{
	u16 w;

	while (len-- > 0) {
		w = (rtl_ephy_read(tp, e->offset) & ~e->mask) | e->bits;
		rtl_ephy_write(tp, e->offset, w);
		e++;
	}
}

#define rtl_ephy_init(tp, a) __rtl_ephy_init(tp, a, ARRAY_SIZE(a))

 static void rtl_pcie_state_l2l3_disable(struct rtl8169_private *tp)
{
	/* work around an issue when PCI reset occurs during L2/L3 state */
	RTL_W8(tp, Config3, RTL_R8(tp, Config3) & ~Rdy_to_L23);
}

static void rtl_enable_exit_l1(struct rtl8169_private *tp)
{
	r8168_mac_ocp_modify(tp, 0xc0ac, 0, 0x1f80);
}

static void rtl_disable_exit_l1(struct rtl8169_private *tp)
{
	r8168_mac_ocp_modify(tp, 0xc0ac, 0x1f80, 0);
}

static void rtl_hw_aspm_clkreq_enable(struct rtl8169_private *tp, bool enable)
{
	if (enable && tp->aspm_manageable) {
		rtl_mod_config5(tp, 0, ASPM_en);
		rtl_mod_config2(tp, 0, ClkReqEn);
		r8168_mac_ocp_modify(tp, 0xe094, 0xff00, 0);
		r8168_mac_ocp_modify(tp, 0xe092, 0x00ff, BIT(2));
	} else {
		r8168_mac_ocp_modify(tp, 0xe092, 0x00ff, 0);
		rtl_mod_config2(tp, ClkReqEn, 0);
		rtl_mod_config5(tp, ASPM_en, 0);
	}
}

static void rtl_set_fifo_size(struct rtl8169_private *tp, u16 rx_stat,
			      u16 tx_stat, u16 rx_dyn, u16 tx_dyn)
{
	/* Usage of dynamic vs. static FIFO is controlled by bit
	 * TXCFG_AUTO_FIFO. Exact meaning of FIFO values isn't known.
	 */
	rtl_eri_write(tp, 0xc8, ERIAR_MASK_1111, (rx_stat << 16) | rx_dyn);
	rtl_eri_write(tp, 0xe8, ERIAR_MASK_1111, (tx_stat << 16) | tx_dyn);
}

static void rtl8168g_set_pause_thresholds(struct rtl8169_private *tp,
					  u8 low, u8 high)
{
	/* FIFO thresholds for pause flow control */
	rtl_eri_write(tp, 0xcc, ERIAR_MASK_0001, low);
	rtl_eri_write(tp, 0xd0, ERIAR_MASK_0001, high);
}

static void rtl_hw_start_8168h_1(struct rtl8169_private *tp)
{
	static const struct ephy_info e_info_8168h_1[] = {
		{ 0x1e, 0x0800,	0x0001 },
		{ 0x1d, 0x0000,	0x0800 },
		{ 0x05, 0xffff,	0x2089 },
		{ 0x06, 0xffff,	0x5881 },
		{ 0x04, 0xffff,	0x854a },
		{ 0x01, 0xffff,	0x068b }
	};
	int rg_saw_cnt;

	rtl_ephy_init(tp, e_info_8168h_1);

	rtl_set_fifo_size(tp, 0x08, 0x10, 0x02, 0x06);
	rtl8168g_set_pause_thresholds(tp, 0x38, 0x48);

	rtl_set_def_aspm_entry_latency(tp);

	rtl_reset_packet_filter(tp);

	rtl_eri_set_bits(tp, 0xdc, 0x001c);
	rtl_eri_write(tp, 0x5f0, ERIAR_MASK_0011, 0x4f87);

	rtl_disable_rxdvgate(tp);

	rtl_eri_write(tp, 0xc0, ERIAR_MASK_0011, 0x0000);
	rtl_eri_write(tp, 0xb8, ERIAR_MASK_0011, 0x0000);

	rtl8168_config_eee_mac(tp);

	RTL_W8(tp, DLLPR, RTL_R8(tp, DLLPR) & ~PFM_EN);
	RTL_W8(tp, MISC_1, RTL_R8(tp, MISC_1) & ~PFM_D3COLD_EN);
	RTL_W8(tp, DLLPR, RTL_R8(tp, DLLPR) & ~TX_10M_PS_EN);

	rtl_eri_clear_bits(tp, 0x1b0, BIT(12));
	rtl_pcie_state_l2l3_disable(tp);

	rg_saw_cnt = phy_read_paged(tp->phydev, 0x0c42, 0x13) & 0x3fff;
	if (rg_saw_cnt > 0) {
		u16 sw_cnt_1ms_ini;

		sw_cnt_1ms_ini = 16000000 / rg_saw_cnt;
		sw_cnt_1ms_ini &= 0x0fff;
		r8168_mac_ocp_modify(tp, 0xd412, 0x0fff, sw_cnt_1ms_ini);
	}

	r8168_mac_ocp_modify(tp, 0xe056, 0x00f0, 0x0070);
	r8168_mac_ocp_modify(tp, 0xe052, 0x6000, 0x8008);
	r8168_mac_ocp_modify(tp, 0xe0d6, 0x01ff, 0x017f);
	r8168_mac_ocp_modify(tp, 0xd420, 0x0fff, 0x047f);

	r8168_mac_ocp_write(tp, 0xe63e, 0x0001);
	r8168_mac_ocp_write(tp, 0xe63e, 0x0000);
	r8168_mac_ocp_write(tp, 0xc094, 0x0000);
	r8168_mac_ocp_write(tp, 0xc09e, 0x0000);
}

static void rtl_hw_config(struct rtl8169_private *tp)
{
	rtl_hw_start_8168h_1(tp);
}

static void rtl_hw_start_8168(struct rtl8169_private *tp)
{
	RTL_W8(tp, MaxTxPacketSize, EarlySize);

	rtl_hw_config(tp);

	/* disable interrupt coalescing */
	RTL_W16(tp, IntrMitigate, 0x0000);
}

static void rtl_hw_start(struct  rtl8169_private *tp)
{
	rtl_unlock_config_regs(tp);
	/* disable aspm and clock request before ephy access */
	rtl_hw_aspm_clkreq_enable(tp, false);
	RTL_W16(tp, CPlusCmd, tp->cp_cmd);

	rtl_hw_start_8168(tp);

	rtl_enable_exit_l1(tp);
	rtl_hw_aspm_clkreq_enable(tp, true);
	if (pci_is_pcie(tp->pci_dev))
		pcie_set_readrq(tp->pci_dev, 4096);
	rtl_set_rx_max_size(tp);
	rtl_set_rx_tx_desc_registers(tp);
	rtl_lock_config_regs(tp);

	/* Initially a 10 us delay. Turned it into a PCI commit. - FR */
	rtl_pci_commit(tp);

	RTL_W8(tp, ChipCmd, CmdTxEnb | CmdRxEnb);
	rtl_init_rxcfg(tp);
	rtl_set_tx_config_registers(tp);
	rtl_set_rx_config_defaults(tp);
	rtl_set_rx_mode(tp);
	rtl_irq_enable(tp);
}

static void rtl8169_mark_to_asic(struct RxDesc *desc)
{
	u32 eor = le32_to_cpu(desc->opts1) & RingEnd;

	desc->opts2 = 0;
	/* Force memory writes to complete before releasing descriptor */
	dma_wmb();
	WRITE_ONCE(desc->opts1, cpu_to_le32(DescOwn | eor | R8169_RX_BUF_SIZE));
}

static struct page *rtl8169_alloc_rx_data(struct rtl8169_private *tp,
					  struct RxDesc *desc)
{
	struct device *d = tp_to_dev(tp);
	int node = dev_to_node(d);
	dma_addr_t mapping;
	struct page *data;

	data = alloc_pages_node(node, GFP_KERNEL, get_order(R8169_RX_BUF_SIZE));
	if (!data)
		return NULL;

	mapping = dma_map_page(d, data, 0, R8169_RX_BUF_SIZE, DMA_FROM_DEVICE);
	if (unlikely(dma_mapping_error(d, mapping))) {
		netdev_err(tp->dev, "Failed to map RX DMA!\n");
		__free_pages(data, get_order(R8169_RX_BUF_SIZE));
		return NULL;
	}

	desc->addr = cpu_to_le64(mapping);
	rtl8169_mark_to_asic(desc);

	return data;
}

static void rtl8169_rx_clear(struct rtl8169_private *tp)
{
	int i;

	for (i = 0; i < NUM_RX_DESC && tp->Rx_databuff[i]; i++) {
		dma_unmap_page(tp_to_dev(tp),
			       le64_to_cpu(tp->RxDescArray[i].addr),
			       R8169_RX_BUF_SIZE, DMA_FROM_DEVICE);
		__free_pages(tp->Rx_databuff[i], get_order(R8169_RX_BUF_SIZE));
		tp->Rx_databuff[i] = NULL;
		tp->RxDescArray[i].addr = 0;
		tp->RxDescArray[i].opts1 = 0;
	}
}

static int rtl8169_rx_fill(struct rtl8169_private *tp)
{
	int i;

	for (i = 0; i < NUM_RX_DESC; i++) {
		struct page *data;

		data = rtl8169_alloc_rx_data(tp, tp->RxDescArray + i);
		if (!data) {
			rtl8169_rx_clear(tp);
			return -ENOMEM;
		}
		tp->Rx_databuff[i] = data;
	}

	/* mark as last descriptor in the ring */
	tp->RxDescArray[NUM_RX_DESC - 1].opts1 |= cpu_to_le32(RingEnd);

	return 0;
}

static int rtl8169_init_ring(struct rtl8169_private *tp)
{
	rtl8169_init_ring_indexes(tp);

	memset(tp->tx_skb, 0, sizeof(tp->tx_skb));
	memset(tp->Rx_databuff, 0, sizeof(tp->Rx_databuff));

	return rtl8169_rx_fill(tp);
}

static void rtl8169_unmap_tx_skb(struct rtl8169_private *tp, unsigned int entry)
{
	struct ring_info *tx_skb = tp->tx_skb + entry;
	struct TxDesc *desc = tp->TxDescArray + entry;

	dma_unmap_single(tp_to_dev(tp), le64_to_cpu(desc->addr), tx_skb->len,
			 DMA_TO_DEVICE);
	memset(desc, 0, sizeof(*desc));
	memset(tx_skb, 0, sizeof(*tx_skb));
}

static void rtl8169_tx_clear_range(struct rtl8169_private *tp, u32 start,
				   unsigned int n)
{
	unsigned int i;

	for (i = 0; i < n; i++) {
		unsigned int entry = (start + i) % NUM_TX_DESC;
		struct ring_info *tx_skb = tp->tx_skb + entry;
		unsigned int len = tx_skb->len;

		if (len) {
			struct sk_buff *skb = tx_skb->skb;

			rtl8169_unmap_tx_skb(tp, entry);
			if (skb)
				dev_consume_skb_any(skb);
		}
	}
}

static void rtl8169_tx_clear(struct rtl8169_private *tp)
{
	rtl8169_tx_clear_range(tp, tp->dirty_tx, NUM_TX_DESC);
	netdev_reset_queue(tp->dev);
}

static void rtl8169_cleanup(struct rtl8169_private *tp)
{
	napi_disable(&tp->napi);

	/* Give a racing hard_start_xmit a few cycles to complete. */
	synchronize_net();

	/* Disable interrupts */
	rtl8169_irq_mask_and_ack(tp);

	rtl_rx_close(tp);

	/* rtl8168h/8111h cleanup path */
	rtl_enable_rxdvgate(tp);
	fsleep(2000);

	rtl_hw_reset(tp);

	rtl8169_tx_clear(tp);
	rtl8169_init_ring_indexes(tp);
}

static void rtl_reset_work(struct rtl8169_private *tp)
{
	int i;

	netif_stop_queue(tp->dev);

	rtl8169_cleanup(tp);

	for (i = 0; i < NUM_RX_DESC; i++)
		rtl8169_mark_to_asic(tp->RxDescArray + i);

	napi_enable(&tp->napi);
	rtl_hw_start(tp);
}

static void rtl8169_tx_timeout(struct net_device *dev, unsigned int txqueue)
{
	struct rtl8169_private *tp = netdev_priv(dev);

	rtl_schedule_task(tp, RTL_FLAG_TASK_TX_TIMEOUT);
}

static int rtl8169_tx_map(struct rtl8169_private *tp, const u32 *opts, u32 len,
			  void *addr, unsigned int entry, bool desc_own)
{
	struct TxDesc *txd = tp->TxDescArray + entry;
	struct device *d = tp_to_dev(tp);
	dma_addr_t mapping;
	u32 opts1;
	int ret;

	mapping = dma_map_single(d, addr, len, DMA_TO_DEVICE);
	ret = dma_mapping_error(d, mapping);
	if (unlikely(ret)) {
		if (net_ratelimit())
			netdev_err(tp->dev, "Failed to map TX data!\n");
		return ret;
	}

	txd->addr = cpu_to_le64(mapping);
	txd->opts2 = cpu_to_le32(opts[1]);

	opts1 = opts[0] | len;
	if (entry == NUM_TX_DESC - 1)
		opts1 |= RingEnd;
	if (desc_own)
		opts1 |= DescOwn;
	txd->opts1 = cpu_to_le32(opts1);

	tp->tx_skb[entry].len = len;

	return 0;
}

static bool rtl8169_hw_csum(struct sk_buff *skb, u32 *opts)
{
	u8 ip_protocol;

	if (skb->ip_summed != CHECKSUM_PARTIAL)
		return true;

	switch (vlan_get_protocol(skb)) {
	case htons(ETH_P_IP):
		opts[1] |= TD1_IPv4_CS;
		ip_protocol = ip_hdr(skb)->protocol;
		break;
	case htons(ETH_P_IPV6):
		opts[1] |= TD1_IPv6_CS;
		ip_protocol = ipv6_hdr(skb)->nexthdr;
		break;
	default:
		return !skb_checksum_help(skb);
	}

	if (ip_protocol == IPPROTO_TCP)
		opts[1] |= TD1_TCP_CS;
	else if (ip_protocol == IPPROTO_UDP)
		opts[1] |= TD1_UDP_CS;
	else
		return !skb_checksum_help(skb);

	opts[1] |= skb_transport_offset(skb) << TCPHO_SHIFT;

	return true;
}

static unsigned int rtl_tx_slots_avail(struct rtl8169_private *tp)
{
	return READ_ONCE(tp->dirty_tx) + NUM_TX_DESC - READ_ONCE(tp->cur_tx);
}

static void rtl8169_doorbell(struct rtl8169_private *tp)
{
	RTL_W8(tp, TxPoll, NPQ);
}

static netdev_tx_t rtl8169_start_xmit(struct sk_buff *skb,
				      struct net_device *dev)
{
	struct rtl8169_private *tp = netdev_priv(dev);
	unsigned int entry = tp->cur_tx % NUM_TX_DESC;
	struct TxDesc *txd_first;
	bool stop_queue, door_bell;
	u32 opts[2];

	if (unlikely(!rtl_tx_slots_avail(tp))) {
		if (net_ratelimit())
			netdev_err(dev, "BUG! Tx Ring full when queue awake!\n");
		goto err_stop_0;
	}

	if (unlikely(skb_is_nonlinear(skb) && skb_linearize(skb)))
		goto err_dma_0;

	opts[1] = 0;
	opts[0] = 0;

	if (!rtl8169_hw_csum(skb, opts))
		goto err_dma_0;

	if (unlikely(rtl8169_tx_map(tp, opts, skb_headlen(skb), skb->data,
				    entry, false)))
		goto err_dma_0;

	txd_first = tp->TxDescArray + entry;
	txd_first->opts1 |= cpu_to_le32(LastFrag);
	tp->tx_skb[entry].skb = skb;

	skb_tx_timestamp(skb);

	/* Force memory writes to complete before releasing descriptor */
	dma_wmb();

	door_bell = __netdev_sent_queue(dev, skb->len, netdev_xmit_more());

	txd_first->opts1 |= cpu_to_le32(DescOwn | FirstFrag);

	/* rtl_tx needs to see descriptor changes before updated tp->cur_tx */
	smp_wmb();

	WRITE_ONCE(tp->cur_tx, tp->cur_tx + 1);

	stop_queue = !netif_subqueue_maybe_stop(dev, 0, rtl_tx_slots_avail(tp),
						R8169_TX_STOP_THRS,
						R8169_TX_START_THRS);
	if (door_bell || stop_queue)
		rtl8169_doorbell(tp);

	return NETDEV_TX_OK;
err_dma_0:
	dev_kfree_skb_any(skb);
	dev->stats.tx_dropped++;
	return NETDEV_TX_OK;

err_stop_0:
	netif_stop_queue(dev);
	dev->stats.tx_dropped++;
	return NETDEV_TX_BUSY;
}

static netdev_features_t rtl8169_features_check(struct sk_buff *skb,
						struct net_device *dev,
						netdev_features_t features)
{
	(void)dev;

	if (skb->ip_summed == CHECKSUM_PARTIAL) {
		/* work around hw bug on some chip versions */
		if (skb->len < ETH_ZLEN)
			features &= ~NETIF_F_CSUM_MASK;

		if (skb_transport_offset(skb) > TCPHO_MAX)
			features &= ~NETIF_F_CSUM_MASK;
	}

	return features;
}

static void rtl8169_pcierr_interrupt(struct net_device *dev)
{
	struct rtl8169_private *tp = netdev_priv(dev);
	struct pci_dev *pdev = tp->pci_dev;
	int pci_status_errs;
	u16 pci_cmd;

	pci_read_config_word(pdev, PCI_COMMAND, &pci_cmd);

	pci_status_errs = pci_status_get_and_clear_errors(pdev);

	if (net_ratelimit())
		netdev_err(dev, "PCI error (cmd = 0x%04x, status_errs = 0x%04x)\n",
			   pci_cmd, pci_status_errs);

	rtl_schedule_task(tp, RTL_FLAG_TASK_RESET_PENDING);
}

static void rtl_tx(struct net_device *dev, struct rtl8169_private *tp,
		   int budget)
{
	unsigned int dirty_tx, bytes_compl = 0, pkts_compl = 0;
	struct sk_buff *skb;

	dirty_tx = tp->dirty_tx;

	while (READ_ONCE(tp->cur_tx) != dirty_tx) {
		unsigned int entry = dirty_tx % NUM_TX_DESC;
		u32 status;

		status = le32_to_cpu(READ_ONCE(tp->TxDescArray[entry].opts1));
		if (status & DescOwn)
			break;

		skb = tp->tx_skb[entry].skb;
		rtl8169_unmap_tx_skb(tp, entry);

		if (skb) {
			pkts_compl++;
			bytes_compl += skb->len;
			napi_consume_skb(skb, budget);
		}
		dirty_tx++;
	}

	if (tp->dirty_tx != dirty_tx) {
		dev_sw_netstats_tx_add(dev, pkts_compl, bytes_compl);
		WRITE_ONCE(tp->dirty_tx, dirty_tx);

		netif_subqueue_completed_wake(dev, 0, pkts_compl, bytes_compl,
					      rtl_tx_slots_avail(tp),
					      R8169_TX_START_THRS);
		/*
		 * 8168 hack: TxPoll requests are lost when the Tx packets are
		 * too close. Let's kick an extra TxPoll request when a burst
		 * of start_xmit activity is detected (if it is not detected,
		 * it is slow enough). -- FR
		 * If skb is NULL then we come here again once a tx irq is
		 * triggered after the last fragment is marked transmitted.
		 */
		if (READ_ONCE(tp->cur_tx) != dirty_tx && skb)
			rtl8169_doorbell(tp);
	}
}

static inline int rtl8169_fragmented_frame(u32 status)
{
	return (status & (FirstFrag | LastFrag)) != (FirstFrag | LastFrag);
}

static inline void rtl8169_rx_csum(struct sk_buff *skb, u32 opts1)
{
	u32 status = opts1 & (RxProtoMask | RxCSFailMask);

	if (status == RxProtoTCP || status == RxProtoUDP)
		skb->ip_summed = CHECKSUM_UNNECESSARY;
	else
		skb_checksum_none_assert(skb);
}

static int rtl_rx(struct net_device *dev, struct rtl8169_private *tp, int budget)
{
	struct device *d = tp_to_dev(tp);
	int count;

	for (count = 0; count < budget; count++, tp->cur_rx++) {
		unsigned int pkt_size, entry = tp->cur_rx % NUM_RX_DESC;
		struct RxDesc *desc = tp->RxDescArray + entry;
		struct sk_buff *skb;
		const void *rx_buf;
		dma_addr_t addr;
		u32 status;

		status = le32_to_cpu(READ_ONCE(desc->opts1));
		if (status & DescOwn)
			break;

		/* This barrier is needed to keep us from reading
		 * any other fields out of the Rx descriptor until
		 * we know the status of DescOwn
		 */
		dma_rmb();

		if (unlikely(status & RxRES)) {
			if (net_ratelimit())
				netdev_warn(dev, "Rx ERROR. status = %08x\n",
					    status);
			dev->stats.rx_errors++;
			if (status & (RxRWT | RxRUNT))
				dev->stats.rx_length_errors++;
			if (status & RxCRC)
				dev->stats.rx_crc_errors++;
			goto release_descriptor;
		}

		pkt_size = status & GENMASK(13, 0);
		pkt_size -= ETH_FCS_LEN;

		/* The driver does not support incoming fragmented frames.
		 * They are seen as a symptom of over-mtu sized frames.
		 */
		if (unlikely(rtl8169_fragmented_frame(status))) {
			dev->stats.rx_dropped++;
			dev->stats.rx_length_errors++;
			goto release_descriptor;
		}

		skb = napi_alloc_skb(&tp->napi, pkt_size);
		if (unlikely(!skb)) {
			dev->stats.rx_dropped++;
			goto release_descriptor;
		}

		addr = le64_to_cpu(desc->addr);
		rx_buf = page_address(tp->Rx_databuff[entry]);

		dma_sync_single_for_cpu(d, addr, pkt_size, DMA_FROM_DEVICE);
		prefetch(rx_buf);
		skb_copy_to_linear_data(skb, rx_buf, pkt_size);
		skb->tail += pkt_size;
		skb->len = pkt_size;
		dma_sync_single_for_device(d, addr, pkt_size, DMA_FROM_DEVICE);

		rtl8169_rx_csum(skb, status);
		skb->protocol = eth_type_trans(skb, dev);

		if (skb->pkt_type == PACKET_MULTICAST)
			dev->stats.multicast++;

		napi_gro_receive(&tp->napi, skb);

		dev_sw_netstats_rx_add(dev, pkt_size);
release_descriptor:
		rtl8169_mark_to_asic(desc);
	}

	return count;
}

static irqreturn_t rtl8169_interrupt(int irq, void *dev_instance)
{
	struct rtl8169_private *tp = dev_instance;
	u32 status = rtl_get_events(tp);

	if ((status & 0xffff) == 0xffff || !(status & tp->irq_mask))
		return IRQ_NONE;

	if (unlikely(status & SYSErr)) {
		rtl8169_pcierr_interrupt(tp->dev);
		goto out;
	}

	if (status & LinkChg)
		phy_mac_interrupt(tp->phydev);

	if (napi_schedule_prep(&tp->napi)) {
		rtl_irq_disable(tp);
		__napi_schedule(&tp->napi);
	}
out:
	rtl_ack_events(tp, status);

	return IRQ_HANDLED;
}

static void rtl_task(struct work_struct *work)
{
	struct rtl8169_private *tp =
		container_of(work, struct rtl8169_private, wk.work);
	int ret;

	rtnl_lock();

	if (!test_bit(RTL_FLAG_TASK_ENABLED, tp->wk.flags))
		goto out_unlock;

	if (test_and_clear_bit(RTL_FLAG_TASK_TX_TIMEOUT, tp->wk.flags)) {
		/* if chip isn't accessible, reset bus to revive it */
		if (RTL_R32(tp, TxConfig) == ~0) {
			ret = pci_reset_bus(tp->pci_dev);
			if (ret < 0) {
				netdev_err(tp->dev, "Can't reset secondary PCI bus, detach NIC\n");
				netif_device_detach(tp->dev);
				goto out_unlock;
			}
		}

		/* ASPM compatibility issues are a typical reason for tx timeouts */
		ret = pci_disable_link_state(tp->pci_dev, PCIE_LINK_STATE_L1 |
							  PCIE_LINK_STATE_L0S);
		if (!ret)
			netdev_warn_once(tp->dev, "ASPM disabled on Tx timeout\n");
		goto reset;
	}

	if (test_and_clear_bit(RTL_FLAG_TASK_RESET_PENDING, tp->wk.flags)) {
reset:
		rtl_reset_work(tp);
		netif_wake_queue(tp->dev);
	} else if (test_and_clear_bit(RTL_FLAG_TASK_RESET_NO_QUEUE_WAKE, tp->wk.flags)) {
		rtl_reset_work(tp);
	}
out_unlock:
	rtnl_unlock();
}

static int rtl8169_poll(struct napi_struct *napi, int budget)
{
	struct rtl8169_private *tp = container_of(napi, struct rtl8169_private, napi);
	struct net_device *dev = tp->dev;
	int work_done;

	rtl_tx(dev, tp, budget);

	work_done = rtl_rx(dev, tp, budget);

	if (work_done < budget && napi_complete_done(napi, work_done))
		rtl_irq_enable(tp);

	return work_done;
}

static void r8169_phylink_handler(struct net_device *ndev)
{
	struct rtl8169_private *tp = netdev_priv(ndev);

	if (netif_carrier_ok(ndev)) {
		rtl_link_chg_patch(tp);
		netif_wake_queue(tp->dev);
	}

	phy_print_status(tp->phydev);
}

static int r8169_phy_connect(struct rtl8169_private *tp)
{
	struct phy_device *phydev = tp->phydev;
	int ret;

	ret = phy_connect_direct(tp->dev, phydev, r8169_phylink_handler,
				 PHY_INTERFACE_MODE_GMII);
	if (ret)
		return ret;

	phy_attached_info(phydev);

	return 0;
}

static void rtl8169_down(struct rtl8169_private *tp)
{
	/* Clear all task flags */
	bitmap_zero(tp->wk.flags, RTL_FLAG_MAX);

	phy_stop(tp->phydev);

	rtl8169_update_counters(tp);

	pci_clear_master(tp->pci_dev);
	rtl_pci_commit(tp);

	rtl8169_cleanup(tp);
	rtl_disable_exit_l1(tp);
}

static void rtl8169_up(struct rtl8169_private *tp)
{
	pci_set_master(tp->pci_dev);
	phy_init_hw(tp->phydev);
	phy_resume(tp->phydev);
	rtl8169_init_phy(tp);
	napi_enable(&tp->napi);
	set_bit(RTL_FLAG_TASK_ENABLED, tp->wk.flags);
	rtl_reset_work(tp);

	phy_start(tp->phydev);
}

static int rtl8169_close(struct net_device *dev)
{
	struct rtl8169_private *tp = netdev_priv(dev);
	struct pci_dev *pdev = tp->pci_dev;

	netif_stop_queue(dev);
	rtl8169_down(tp);
	rtl8169_rx_clear(tp);

	cancel_work(&tp->wk.work);

	free_irq(tp->irq, tp);

	phy_disconnect(tp->phydev);

	dma_free_coherent(&pdev->dev, R8169_RX_RING_BYTES, tp->RxDescArray,
			  tp->RxPhyAddr);
	dma_free_coherent(&pdev->dev, R8169_TX_RING_BYTES, tp->TxDescArray,
			  tp->TxPhyAddr);
	tp->TxDescArray = NULL;
	tp->RxDescArray = NULL;

	return 0;
}

static int rtl_open(struct net_device *dev)
{
	struct rtl8169_private *tp = netdev_priv(dev);
	struct pci_dev *pdev = tp->pci_dev;
	unsigned long irqflags;
	int retval = -ENOMEM;

	/*
	 * Rx and Tx descriptors needs 256 bytes alignment.
	 * dma_alloc_coherent provides more.
	 */
	tp->TxDescArray = dma_alloc_coherent(&pdev->dev, R8169_TX_RING_BYTES,
					     &tp->TxPhyAddr, GFP_KERNEL);
	if (!tp->TxDescArray)
		return retval;

	tp->RxDescArray = dma_alloc_coherent(&pdev->dev, R8169_RX_RING_BYTES,
					     &tp->RxPhyAddr, GFP_KERNEL);
	if (!tp->RxDescArray)
		goto err_free_tx_0;

	retval = rtl8169_init_ring(tp);
	if (retval < 0)
		goto err_free_rx_1;

	rtl_request_firmware(tp);

	irqflags = pci_dev_msi_enabled(pdev) ? IRQF_NO_THREAD : IRQF_SHARED;
	retval = request_irq(tp->irq, rtl8169_interrupt, irqflags, dev->name, tp);
	if (retval < 0)
		goto err_release_fw_2;

	retval = r8169_phy_connect(tp);
	if (retval)
		goto err_free_irq;

	rtl8169_up(tp);
	rtl8169_init_counter_offsets(tp);
	netif_start_queue(dev);
	return retval;

err_free_irq:
	free_irq(tp->irq, tp);
err_release_fw_2:
	rtl_release_firmware(tp);
	rtl8169_rx_clear(tp);
err_free_rx_1:
	dma_free_coherent(&pdev->dev, R8169_RX_RING_BYTES, tp->RxDescArray,
			  tp->RxPhyAddr);
	tp->RxDescArray = NULL;
err_free_tx_0:
	dma_free_coherent(&pdev->dev, R8169_TX_RING_BYTES, tp->TxDescArray,
			  tp->TxPhyAddr);
	tp->TxDescArray = NULL;
	return retval;
}

static void
rtl8169_get_stats64(struct net_device *dev, struct rtnl_link_stats64 *stats)
{
	struct rtl8169_private *tp = netdev_priv(dev);
	struct rtl8169_counters *counters = tp->counters;

	netdev_stats_to_stats64(stats, &dev->stats);
	dev_fetch_sw_netstats(stats, dev->tstats);

	/*
	 * Fetch additional counter values missing in stats collected by driver
	 * from tally counters.
	 */
	rtl8169_update_counters(tp);

	/*
	 * Subtract values fetched during initalization.
	 * See rtl8169_init_counter_offsets for a description why we do that.
	 */
	stats->tx_errors = le64_to_cpu(counters->tx_errors) -
		le64_to_cpu(tp->tc_offset.tx_errors);
	stats->collisions = le32_to_cpu(counters->tx_multi_collision) -
		le32_to_cpu(tp->tc_offset.tx_multi_collision);
	stats->tx_aborted_errors = le16_to_cpu(counters->tx_aborted) -
		le16_to_cpu(tp->tc_offset.tx_aborted);
	stats->rx_missed_errors = le16_to_cpu(counters->rx_missed) -
		le16_to_cpu(tp->tc_offset.rx_missed);
}

static void rtl_shutdown(struct pci_dev *pdev)
{
	struct rtl8169_private *tp = pci_get_drvdata(pdev);

	rtnl_lock();
	netif_device_detach(tp->dev);
	if (netif_running(tp->dev))
		rtl8169_down(tp);
	rtnl_unlock();

	/* Restore original MAC address */
	rtl_rar_set(tp, tp->dev->perm_addr);

	if (system_state == SYSTEM_POWER_OFF) {
		pci_wake_from_d3(pdev, false);
		pci_set_power_state(pdev, PCI_D3hot);
	}
}

static void rtl_remove_one(struct pci_dev *pdev)
{
	struct rtl8169_private *tp = pci_get_drvdata(pdev);

	cancel_work_sync(&tp->wk.work);

	unregister_netdev(tp->dev);

	rtl_release_firmware(tp);

	/* restore original MAC address */
	rtl_rar_set(tp, tp->dev->perm_addr);
}

static const struct net_device_ops rtl_netdev_ops = {
	.ndo_open		= rtl_open,
	.ndo_stop		= rtl8169_close,
	.ndo_get_stats64	= rtl8169_get_stats64,
	.ndo_start_xmit		= rtl8169_start_xmit,
	.ndo_features_check	= rtl8169_features_check,
	.ndo_tx_timeout		= rtl8169_tx_timeout,
	.ndo_validate_addr	= eth_validate_addr,
	.ndo_set_mac_address	= rtl_set_mac_address,
};

static void rtl_set_irq_mask(struct rtl8169_private *tp)
{
	tp->irq_mask = RxOK | RxErr | TxOK | TxErr | LinkChg;
	tp->irq_mask |= RxOverflow;
}

static int rtl_alloc_irq(struct rtl8169_private *tp)
{
	return pci_alloc_irq_vectors(tp->pci_dev, 1, 1, PCI_IRQ_ALL_TYPES);
}

static void rtl_read_mac_address(struct rtl8169_private *tp,
				 u8 mac_addr[ETH_ALEN])
{
	u32 value;

	value = rtl_eri_read(tp, 0xe0);
	put_unaligned_le32(value, mac_addr);
	value = rtl_eri_read(tp, 0xe4);
	put_unaligned_le16(value, mac_addr + 4);
}

DECLARE_RTL_COND(rtl_link_list_ready_cond)
{
	return RTL_R8(tp, MCU) & LINK_LIST_RDY;
}

static void r8168g_wait_ll_share_fifo_ready(struct rtl8169_private *tp)
{
	rtl_loop_wait_high(tp, &rtl_link_list_ready_cond, 100, 42);
}

static int r8169_mdio_read_reg(struct mii_bus *mii_bus, int phyaddr, int phyreg)
{
	struct rtl8169_private *tp = mii_bus->priv;

	if (phyaddr > 0)
		return -ENODEV;

	return rtl_readphy(tp, phyreg);
}

static int r8169_mdio_write_reg(struct mii_bus *mii_bus, int phyaddr,
				int phyreg, u16 val)
{
	struct rtl8169_private *tp = mii_bus->priv;

	if (phyaddr > 0)
		return -ENODEV;

	rtl_writephy(tp, phyreg, val);

	return 0;
}

static int r8169_mdio_register(struct rtl8169_private *tp)
{
	struct pci_dev *pdev = tp->pci_dev;
	struct mii_bus *new_bus;
	int ret;

	new_bus = devm_mdiobus_alloc(&pdev->dev);
	if (!new_bus)
		return -ENOMEM;

	new_bus->name = "r8169";
	new_bus->priv = tp;
	new_bus->parent = &pdev->dev;
	new_bus->irq[0] = PHY_MAC_INTERRUPT;
	snprintf(new_bus->id, MII_BUS_ID_SIZE, "r8169-%x-%x",
		 pci_domain_nr(pdev->bus), pci_dev_id(pdev));

	new_bus->read = r8169_mdio_read_reg;
	new_bus->write = r8169_mdio_write_reg;

	ret = devm_mdiobus_register(&pdev->dev, new_bus);
	if (ret)
		return ret;

	tp->phydev = mdiobus_get_phy(new_bus, 0);
	if (!tp->phydev) {
		return -ENODEV;
	} else if (!tp->phydev->drv) {
		/* Most chip versions fail with the genphy driver.
		 * Therefore ensure that the dedicated PHY driver is loaded.
		 */
		dev_err(&pdev->dev, "no dedicated PHY driver found for PHY ID 0x%08x, maybe realtek.ko needs to be added to initramfs?\n",
			tp->phydev->phy_id);
		return -EUNATCH;
	}

	phy_support_asym_pause(tp->phydev);

	/* PHY will be woken up in rtl_open() */
	phy_suspend(tp->phydev);

	return 0;
}

static void rtl_hw_init_8168g(struct rtl8169_private *tp)
{
	rtl_enable_rxdvgate(tp);

	RTL_W8(tp, ChipCmd, RTL_R8(tp, ChipCmd) & ~(CmdTxEnb | CmdRxEnb));
	msleep(1);
	RTL_W8(tp, MCU, RTL_R8(tp, MCU) & ~NOW_IS_OOB);

	r8168_mac_ocp_modify(tp, 0xe8de, BIT(14), 0);
	r8168g_wait_ll_share_fifo_ready(tp);

	r8168_mac_ocp_modify(tp, 0xe8de, 0, BIT(15));
	r8168g_wait_ll_share_fifo_ready(tp);
}

static void rtl_hw_initialize(struct rtl8169_private *tp)
{
	rtl_hw_init_8168g(tp);
}

static void rtl_init_mac_address(struct rtl8169_private *tp)
{
	u8 mac_addr[ETH_ALEN] __aligned(2) = {};
	struct net_device *dev = tp->dev;
	int rc;

	rc = eth_platform_get_mac_address(tp_to_dev(tp), mac_addr);
	if (!rc)
		goto done;

	rtl_read_mac_address(tp, mac_addr);
	if (is_valid_ether_addr(mac_addr))
		goto done;

	rtl_read_mac_from_reg(tp, mac_addr, MAC0);
	if (is_valid_ether_addr(mac_addr))
		goto done;

	eth_random_addr(mac_addr);
	dev->addr_assign_type = NET_ADDR_RANDOM;
	dev_warn(tp_to_dev(tp), "can't read MAC address, setting random one\n");
done:
	eth_hw_addr_set(dev, mac_addr);
	rtl_rar_set(tp, mac_addr);
}

/* register is set if system vendor successfully tested ASPM 1.2 */
static bool rtl_aspm_is_safe(struct rtl8169_private *tp)
{
	(void)tp;
	return false;
}

static int rtl_init_one(struct pci_dev *pdev, const struct pci_device_id *ent)
{
	struct rtl8169_private *tp;
	int region, rc;
	struct net_device *dev;
	u32 txconfig;
	u16 xid;

	(void)ent;

	dev = devm_alloc_etherdev(&pdev->dev, sizeof (*tp));
	if (!dev)
		return -ENOMEM;

	SET_NETDEV_DEV(dev, &pdev->dev);
	dev->netdev_ops = &rtl_netdev_ops;
	tp = netdev_priv(dev);
	tp->dev = dev;
	tp->pci_dev = pdev;
	tp->supports_gmii = 1;
	tp->ocp_base = OCP_STD_PHY_BASE;

	raw_spin_lock_init(&tp->cfg9346_usage_lock);
	raw_spin_lock_init(&tp->config25_lock);
	raw_spin_lock_init(&tp->mac_ocp_lock);

	dev->tstats = devm_netdev_alloc_pcpu_stats(&pdev->dev,
						   struct pcpu_sw_netstats);
	if (!dev->tstats)
		return -ENOMEM;

	/* enable device (incl. PCI PM wakeup and hotplug setup) */
	rc = pcim_enable_device(pdev);
	if (rc < 0)
		return dev_err_probe(&pdev->dev, rc, "enable failure\n");

	if (pcim_set_mwi(pdev) < 0)
		dev_info(&pdev->dev, "Mem-Wr-Inval unavailable\n");

	/* use first MMIO region */
	region = ffs(pci_select_bars(pdev, IORESOURCE_MEM)) - 1;
	if (region < 0)
		return dev_err_probe(&pdev->dev, -ENODEV, "no MMIO resource found\n");

	rc = pcim_iomap_regions(pdev, BIT(region), KBUILD_MODNAME);
	if (rc < 0)
		return dev_err_probe(&pdev->dev, rc, "cannot remap MMIO, aborting\n");

	tp->mmio_addr = pcim_iomap_table(pdev)[region];

	txconfig = RTL_R32(tp, TxConfig);
	if (txconfig == ~0U)
		return dev_err_probe(&pdev->dev, -EIO, "PCI read failed\n");

	xid = (txconfig >> 20) & 0xfcf;

	/* Identify chip attached to board */
	if (!rtl8169_is_supported_xid(xid, tp->supports_gmii))
		return dev_err_probe(&pdev->dev, -ENODEV,
				     "unsupported XID %03x for rtl8168h-only driver\n",
				     xid);

	/* Disable ASPM L1 as that cause random device stop working
	 * problems as well as full system hangs for some PCIe devices users.
	 */
	if (rtl_aspm_is_safe(tp))
		rc = 0;
	else
		rc = pci_disable_link_state(pdev, PCIE_LINK_STATE_L1);
	tp->aspm_manageable = !rc;

	tp->cp_cmd = RTL_R16(tp, CPlusCmd) & CPCMD_MASK;
	tp->cp_cmd |= RxChkSum;
	tp->cp_cmd &= ~RxVlan;

	if (sizeof(dma_addr_t) > 4 &&
	    !dma_set_mask_and_coherent(&pdev->dev, DMA_BIT_MASK(64)))
		dev->features |= NETIF_F_HIGHDMA;

	rtl_init_rxcfg(tp);

	rtl8169_irq_mask_and_ack(tp);

	rtl_hw_initialize(tp);

	rtl_hw_reset(tp);

	rc = rtl_alloc_irq(tp);
	if (rc < 0)
		return dev_err_probe(&pdev->dev, rc, "Can't allocate interrupt\n");

	tp->irq = pci_irq_vector(pdev, 0);

	INIT_WORK(&tp->wk.work, rtl_task);

	rtl_init_mac_address(tp);

	dev->ethtool_ops = &rtl8169_ethtool_ops;

	netif_napi_add(dev, &tp->napi, rtl8169_poll);

	dev->hw_features = 0;
	dev->vlan_features = 0;
	dev->priv_flags |= IFF_LIVE_ADDR_CHANGE;

	dev->features |= NETIF_F_IP_CSUM | NETIF_F_IPV6_CSUM | NETIF_F_RXCSUM;

	netdev_sw_irq_coalesce_default_on(dev);

	rtl_set_irq_mask(tp);

	tp->fw_name = rtl_chip_info.fw_name;

	tp->counters = dmam_alloc_coherent (&pdev->dev, sizeof(*tp->counters),
					    &tp->counters_phys_addr,
					    GFP_KERNEL);
	if (!tp->counters)
		return -ENOMEM;

	pci_set_drvdata(pdev, tp);

	rc = r8169_mdio_register(tp);
	if (rc)
		return rc;

	rc = register_netdev(dev);
	if (rc)
		return rc;

	netdev_info(dev, "%s, %pM, XID %03x, IRQ %d\n",
		    rtl_chip_info.name, dev->dev_addr, xid, tp->irq);

	return 0;
}

static struct pci_driver rtl8169_pci_driver = {
	.name		= KBUILD_MODNAME,
	.id_table	= rtl8169_pci_tbl,
	.probe		= rtl_init_one,
	.remove		= rtl_remove_one,
	.shutdown	= rtl_shutdown,
};

module_pci_driver(rtl8169_pci_driver);
