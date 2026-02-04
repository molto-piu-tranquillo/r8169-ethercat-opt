// SPDX-License-Identifier: GPL-2.0
/*
 * AF_XDP EtherCAT Test Utility
 * 
 * Tests AF_XDP zero-copy performance with r8169_xdp driver.
 * Measures latency for EtherCAT-like packet processing.
 * 
 * Build: make
 * Usage: ./af_xdp_test -i <interface> [-q queue_id] [-t duration_sec]
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <errno.h>
#include <unistd.h>
#include <signal.h>
#include <time.h>
#include <getopt.h>
#include <poll.h>
#include <sys/socket.h>
#include <sys/mman.h>
#include <net/if.h>
#include <linux/if_link.h>
#include <linux/if_ether.h>

#include <bpf/bpf.h>
#include <bpf/libbpf.h>
#include <xdp/xsk.h>
#include <xdp/libxdp.h>

#define NUM_FRAMES 4096
#define FRAME_SIZE XSK_UMEM__DEFAULT_FRAME_SIZE
#define RX_BATCH_SIZE 64
#define FRAME_HEADROOM XSK_UMEM__DEFAULT_FRAME_HEADROOM

#define ETH_P_ETHERCAT 0x88A4

struct xsk_socket_info {
	struct xsk_ring_cons rx;
	struct xsk_ring_prod tx;
	struct xsk_ring_prod fq;
	struct xsk_ring_cons cq;
	struct xsk_socket *xsk;
	struct xsk_umem *umem;
	void *umem_area;
	__u64 rx_packets;
	__u64 rx_bytes;
	__u64 rx_timestamp_sum;
	__u64 rx_timestamp_min;
	__u64 rx_timestamp_max;
	__u64 rx_count;
};

static volatile int running = 1;
static struct xdp_program *xdp_prog = NULL;

static void signal_handler(int sig)
{
	running = 0;
}

static __u64 gettime_ns(void)
{
	struct timespec ts;
	clock_gettime(CLOCK_MONOTONIC, &ts);
	return ts.tv_sec * 1000000000ULL + ts.tv_nsec;
}

static struct xsk_socket_info *xsk_configure_socket(const char *ifname,
						    int queue_id)
{
	struct xsk_socket_info *xsk_info;
	struct xsk_socket_config xsk_cfg;
	struct xsk_umem_config umem_cfg;
	void *umem_area;
	int ret;
	__u32 idx;

	xsk_info = calloc(1, sizeof(*xsk_info));
	if (!xsk_info)
		return NULL;

	/* Allocate UMEM */
	umem_area = mmap(NULL, NUM_FRAMES * FRAME_SIZE, PROT_READ | PROT_WRITE,
			 MAP_PRIVATE | MAP_ANONYMOUS, -1, 0);
	if (umem_area == MAP_FAILED) {
		fprintf(stderr, "Failed to allocate UMEM: %s\n",
			strerror(errno));
		free(xsk_info);
		return NULL;
	}
	xsk_info->umem_area = umem_area;

	/* Configure UMEM */
	memset(&umem_cfg, 0, sizeof(umem_cfg));
	umem_cfg.fill_size = NUM_FRAMES;
	umem_cfg.comp_size = NUM_FRAMES;
	umem_cfg.frame_size = FRAME_SIZE;
	umem_cfg.frame_headroom = FRAME_HEADROOM;
	umem_cfg.flags = 0;

	ret = xsk_umem__create(&xsk_info->umem, umem_area,
			       NUM_FRAMES * FRAME_SIZE, &xsk_info->fq,
			       &xsk_info->cq, &umem_cfg);
	if (ret) {
		fprintf(stderr, "Failed to create UMEM: %s\n", strerror(-ret));
		munmap(umem_area, NUM_FRAMES * FRAME_SIZE);
		free(xsk_info);
		return NULL;
	}

	/* Configure socket */
	memset(&xsk_cfg, 0, sizeof(xsk_cfg));
	xsk_cfg.rx_size = NUM_FRAMES;
	xsk_cfg.tx_size = NUM_FRAMES;
	xsk_cfg.bind_flags = XDP_COPY; /* Start with copy mode for testing */
	xsk_cfg.xdp_flags = 0;

	ret = xsk_socket__create(&xsk_info->xsk, ifname, queue_id,
				 xsk_info->umem, &xsk_info->rx, &xsk_info->tx,
				 &xsk_cfg);
	if (ret) {
		fprintf(stderr, "Failed to create XSK socket: %s\n",
			strerror(-ret));
		xsk_umem__delete(xsk_info->umem);
		munmap(umem_area, NUM_FRAMES * FRAME_SIZE);
		free(xsk_info);
		return NULL;
	}

	/* Populate fill queue */
	ret = xsk_ring_prod__reserve(&xsk_info->fq, NUM_FRAMES / 2, &idx);
	if (ret != NUM_FRAMES / 2) {
		fprintf(stderr, "Failed to populate FQ\n");
		xsk_socket__delete(xsk_info->xsk);
		xsk_umem__delete(xsk_info->umem);
		munmap(umem_area, NUM_FRAMES * FRAME_SIZE);
		free(xsk_info);
		return NULL;
	}

	for (__u32 i = 0; i < NUM_FRAMES / 2; i++)
		*xsk_ring_prod__fill_addr(&xsk_info->fq, idx + i) =
			i * FRAME_SIZE;
	xsk_ring_prod__submit(&xsk_info->fq, NUM_FRAMES / 2);

	xsk_info->rx_timestamp_min = ~0ULL;

	return xsk_info;
}

static void xsk_cleanup(struct xsk_socket_info *xsk_info)
{
	if (xsk_info->xsk)
		xsk_socket__delete(xsk_info->xsk);
	if (xsk_info->umem)
		xsk_umem__delete(xsk_info->umem);
	if (xsk_info->umem_area)
		munmap(xsk_info->umem_area, NUM_FRAMES * FRAME_SIZE);
	free(xsk_info);
}

static void process_packets(struct xsk_socket_info *xsk_info)
{
	__u32 idx_rx = 0;
	__u32 rcvd;
	__u64 start_time = gettime_ns();

	rcvd = xsk_ring_cons__peek(&xsk_info->rx, RX_BATCH_SIZE, &idx_rx);
	if (!rcvd)
		return;

	for (__u32 i = 0; i < rcvd; i++) {
		const struct xdp_desc *desc =
			xsk_ring_cons__rx_desc(&xsk_info->rx, idx_rx + i);
		void *pkt = xsk_umem__get_data(xsk_info->umem_area, desc->addr);
		__u32 len = desc->len;
		struct ethhdr *eth = pkt;

		/* Process EtherCAT packet */
		if (len >= sizeof(struct ethhdr) &&
		    eth->h_proto == htons(ETH_P_ETHERCAT)) {
			__u64 latency = gettime_ns() - start_time;

			xsk_info->rx_packets++;
			xsk_info->rx_bytes += len;
			xsk_info->rx_timestamp_sum += latency;
			xsk_info->rx_count++;

			if (latency < xsk_info->rx_timestamp_min)
				xsk_info->rx_timestamp_min = latency;
			if (latency > xsk_info->rx_timestamp_max)
				xsk_info->rx_timestamp_max = latency;
		}
	}

	xsk_ring_cons__release(&xsk_info->rx, rcvd);

	/* Refill FQ */
	__u32 idx_fq;
	int ret = xsk_ring_prod__reserve(&xsk_info->fq, rcvd, &idx_fq);
	if (ret == (int)rcvd) {
		for (__u32 i = 0; i < rcvd; i++) {
			const struct xdp_desc *desc = xsk_ring_cons__rx_desc(
				&xsk_info->rx, idx_rx + i);
			*xsk_ring_prod__fill_addr(&xsk_info->fq, idx_fq + i) =
				desc->addr;
		}
		xsk_ring_prod__submit(&xsk_info->fq, rcvd);
	}
}

static void print_stats(struct xsk_socket_info *xsk_info, double elapsed_sec)
{
	printf("\n=== AF_XDP Test Results ===\n");
	printf("Duration: %.2f seconds\n", elapsed_sec);
	printf("RX Packets: %llu\n", (unsigned long long)xsk_info->rx_packets);
	printf("RX Bytes: %llu\n", (unsigned long long)xsk_info->rx_bytes);
	printf("RX Rate: %.2f pps\n", xsk_info->rx_packets / elapsed_sec);
	printf("Throughput: %.2f Mbps\n",
	       (xsk_info->rx_bytes * 8.0) / (elapsed_sec * 1000000));

	if (xsk_info->rx_count > 0) {
		printf("\n=== Latency Statistics ===\n");
		printf("Min: %llu ns (%.2f us)\n",
		       (unsigned long long)xsk_info->rx_timestamp_min,
		       xsk_info->rx_timestamp_min / 1000.0);
		printf("Max: %llu ns (%.2f us)\n",
		       (unsigned long long)xsk_info->rx_timestamp_max,
		       xsk_info->rx_timestamp_max / 1000.0);
		printf("Avg: %.2f ns (%.2f us)\n",
		       (double)xsk_info->rx_timestamp_sum / xsk_info->rx_count,
		       (double)xsk_info->rx_timestamp_sum / xsk_info->rx_count /
			       1000.0);
	}
}

static void usage(const char *prog)
{
	fprintf(stderr,
		"Usage: %s -i <interface> [-q queue_id] [-t duration_sec]\n",
		prog);
	fprintf(stderr, "  -i  Network interface name\n");
	fprintf(stderr, "  -q  Queue ID (default: 0)\n");
	fprintf(stderr, "  -t  Test duration in seconds (default: 10)\n");
}

int main(int argc, char **argv)
{
	const char *ifname = NULL;
	int queue_id = 0;
	int duration = 10;
	int opt;
	struct xsk_socket_info *xsk_info;
	struct pollfd fds[1];
	__u64 start_time, end_time;

	while ((opt = getopt(argc, argv, "i:q:t:h")) != -1) {
		switch (opt) {
		case 'i':
			ifname = optarg;
			break;
		case 'q':
			queue_id = atoi(optarg);
			break;
		case 't':
			duration = atoi(optarg);
			break;
		case 'h':
		default:
			usage(argv[0]);
			return 1;
		}
	}

	if (!ifname) {
		fprintf(stderr, "Error: Interface name required\n");
		usage(argv[0]);
		return 1;
	}

	signal(SIGINT, signal_handler);
	signal(SIGTERM, signal_handler);

	printf("AF_XDP EtherCAT Test\n");
	printf("Interface: %s\n", ifname);
	printf("Queue ID: %d\n", queue_id);
	printf("Duration: %d seconds\n", duration);

	/* Configure XSK socket */
	xsk_info = xsk_configure_socket(ifname, queue_id);
	if (!xsk_info) {
		fprintf(stderr, "Failed to configure XSK socket\n");
		return 1;
	}

	printf("XSK socket created successfully\n");
	printf("Waiting for EtherCAT packets...\n");

	fds[0].fd = xsk_socket__fd(xsk_info->xsk);
	fds[0].events = POLLIN;

	start_time = gettime_ns();
	end_time = start_time + duration * 1000000000ULL;

	while (running && gettime_ns() < end_time) {
		int ret = poll(fds, 1, 100);
		if (ret < 0) {
			if (errno == EINTR)
				continue;
			fprintf(stderr, "Poll error: %s\n", strerror(errno));
			break;
		}

		if (fds[0].revents & POLLIN)
			process_packets(xsk_info);
	}

	double elapsed = (gettime_ns() - start_time) / 1000000000.0;
	print_stats(xsk_info, elapsed);

	xsk_cleanup(xsk_info);
	printf("Test completed.\n");

	return 0;
}
