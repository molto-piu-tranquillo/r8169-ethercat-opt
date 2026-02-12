/* ethercat_test.c — Standalone EtherCAT send/recv test via UIO
 *
 * Sends an EtherCAT broadcast read (BRD) frame and waits for
 * the response by busy-polling HW descriptors directly.
 * Zero syscalls in the send/recv hot path.
 */

#define _GNU_SOURCE
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <signal.h>
#include <time.h>
#include <sched.h>
#include <unistd.h>
#include <arpa/inet.h>

#include "r8169_uio.h"

#define ETH_P_ECAT  0x88A4
#define ECAT_CMD_BRD 0x07   /* Broadcast Read */

/* Minimal EtherCAT frame: Ethernet header + EtherCAT header + datagram */
struct __attribute__((packed)) ecat_frame {
	/* Ethernet header (14 bytes) */
	uint8_t  dst[6];
	uint8_t  src[6];
	uint16_t ethertype;
	/* EtherCAT header (2 bytes) */
	uint16_t ecat_hdr;    /* bits 0-10: length, bit 12: type=1 */
	/* EtherCAT datagram header (10 bytes) */
	uint8_t  cmd;
	uint8_t  idx;
	uint16_t ado;         /* slave address offset */
	uint16_t adp;         /* slave address position (0=auto-inc) */
	uint16_t dlength;     /* bits 0-10: data length, bit 15: more */
	uint16_t irq;
	/* Data (2 bytes for DL status register 0x0110) */
	uint8_t  data[2];
	/* Working counter (2 bytes) */
	uint16_t wkc;
};

static volatile int running = 1;

static void sighandler(int sig)
{
	(void)sig;
	running = 0;
}

static uint64_t now_ns(void)
{
	struct timespec ts;
	clock_gettime(CLOCK_MONOTONIC, &ts);
	return (uint64_t)ts.tv_sec * 1000000000ULL + ts.tv_nsec;
}

/* Build a BRD frame to read DL Status (register 0x0110, 2 bytes) */
static void build_ecat_brd(struct ecat_frame *f, const uint8_t *src_mac)
{
	memset(f, 0, sizeof(*f));

	/* Ethernet: broadcast dest */
	memset(f->dst, 0xFF, 6);
	memcpy(f->src, src_mac, 6);
	f->ethertype = htons(ETH_P_ECAT);

	/* EtherCAT header: length=sizeof(datagram), type=1 */
	uint16_t dg_len = 10 + 2 + 2;  /* header + data + wkc */
	f->ecat_hdr = htole16(dg_len | (1 << 12));

	/* Datagram: BRD, read DL Status register */
	f->cmd = ECAT_CMD_BRD;
	f->idx = 0x01;
	f->adp = htole16(0);       /* auto-increment */
	f->ado = htole16(0x0110);  /* DL Status register */
	f->dlength = htole16(2);   /* 2 bytes */
	f->irq = 0;
	f->wkc = 0;
}

#define HIST_BINS 200   /* 0-199 μs */
#define NUM_WARMUP 100

int main(int argc, char **argv)
{
	struct r8169_uio dev;
	const char *pci_bdf = RTL8168_PCI_BDF;

	if (argc > 1)
		pci_bdf = argv[1];

	signal(SIGINT, sighandler);
	signal(SIGTERM, sighandler);

	printf("r8169_uio EtherCAT test\n");
	printf("PCI: %s\n\n", pci_bdf);

	if (r8169_uio_init(&dev, pci_bdf) < 0) {
		fprintf(stderr, "Failed to initialize UIO device\n");
		return 1;
	}

	/* Build EtherCAT BRD frame */
	struct ecat_frame tx_frame;
	build_ecat_brd(&tx_frame, dev.mac);

	uint8_t rx_buf[2048];
	uint32_t hist[HIST_BINS] = {0};
	uint32_t total = 0, timeouts = 0;
	uint64_t min_ns = UINT64_MAX, max_ns = 0, sum_ns = 0;

	/* Diagnostic: send one frame and check if NIC processes the TX descriptor */
	{
		printf("TX diag: sending first frame (%zu bytes)...\n", sizeof(tx_frame));
		int txr = r8169_tx(&dev, &tx_frame, sizeof(tx_frame));
		printf("  r8169_tx returned %d\n", txr);

		/* Wait up to 10ms for TX to complete */
		for (int i = 0; i < 100; i++) {
			usleep(100);
			r8169_tx_complete(&dev);
			if (dev.dirty_tx == dev.cur_tx)
				break;
		}
		printf("  TX complete: cur_tx=%u dirty_tx=%u (%s)\n",
		       dev.cur_tx, dev.dirty_tx,
		       (dev.dirty_tx == dev.cur_tx) ? "OK" : "STUCK - NIC not processing TX");

		/* Check IntrStatus for TX/RX events */
		uint16_t isr = rtl_r16(dev.mmio, REG_IntrStatus);
		printf("  IntrStatus=0x%04x (TxOK=%d RxOK=%d TxErr=%d RxErr=%d)\n",
		       isr, !!(isr & TxOK), !!(isr & RxOK),
		       !!(isr & TxErr), !!(isr & RxErr));
		/* ACK all */
		rtl_w16(dev.mmio, REG_IntrStatus, 0xFFFF);

		/* Try to receive the response */
		usleep(5000);
		int rx_len = r8169_rx(&dev, rx_buf, sizeof(rx_buf));
		printf("  RX after 5ms: %d bytes\n", rx_len);
		if (rx_len > 0) {
			printf("  RX ethertype: 0x%04x\n",
			       ntohs(*(uint16_t *)(rx_buf + 12)));
		}
		printf("\n");
	}

	printf("Sending EtherCAT BRD frames... (Ctrl+C to stop)\n\n");

	/* Main loop: send frame, busy-poll for response, measure latency */
	while (running) {
		/* TX */
		uint64_t t_start = now_ns();
		r8169_tx(&dev, &tx_frame, sizeof(tx_frame));

		/* RX busy-poll (zero syscall) */
		int rx_len = 0;
		uint64_t deadline = t_start + 1000000;  /* 1ms timeout */

		while (now_ns() < deadline) {
			rx_len = r8169_rx(&dev, rx_buf, sizeof(rx_buf));
			if (rx_len > 0) {
				/* Check ethertype */
				uint16_t etype = ntohs(*(uint16_t *)(rx_buf + 12));
				if (etype == ETH_P_ECAT)
					break;
				rx_len = 0;  /* not our frame, keep polling */
			}
		}

		uint64_t t_end = now_ns();
		uint64_t elapsed_ns = t_end - t_start;

		/* Complete TX descriptors */
		r8169_tx_complete(&dev);

		total++;

		if (total <= NUM_WARMUP)
			goto next;

		if (rx_len > 0) {
			/* Record latency */
			if (elapsed_ns < min_ns) min_ns = elapsed_ns;
			if (elapsed_ns > max_ns) max_ns = elapsed_ns;
			sum_ns += elapsed_ns;

			uint32_t us = (uint32_t)(elapsed_ns / 1000);
			if (us < HIST_BINS)
				hist[us]++;
			else
				hist[HIST_BINS - 1]++;

			/* Check working counter */
			struct ecat_frame *resp = (struct ecat_frame *)rx_buf;
			uint16_t wkc = le16toh(resp->wkc);
			if (total == NUM_WARMUP + 1)
				printf("First response: wkc=%u, latency=%lu ns\n",
				       wkc, (unsigned long)elapsed_ns);
		} else {
			timeouts++;
		}

next:
		/* 1ms cycle */
		uint64_t next_cycle = t_start + 1000000;
		while (now_ns() < next_cycle)
			;  /* busy-wait for cycle alignment */

		/* Print histogram every second */
		if (total > NUM_WARMUP && (total - NUM_WARMUP) % 1000 == 0) {
			uint32_t measured = total - NUM_WARMUP;
			printf("\n--- After %u cycles (timeouts=%u) ---\n",
			       measured, timeouts);
			printf("min=%lu  avg=%lu  max=%lu ns\n",
			       (unsigned long)min_ns,
			       measured > timeouts ?
				 (unsigned long)(sum_ns / (measured - timeouts)) : 0,
			       (unsigned long)max_ns);

			/* Print histogram */
			printf("Histogram (us):\n");
			for (int i = 0; i < HIST_BINS; i++) {
				if (hist[i] > 0)
					printf("  %3d: %u\n", i, hist[i]);
			}
		}
	}

	printf("\nShutting down...\n");
	r8169_uio_close(&dev);
	return 0;
}
