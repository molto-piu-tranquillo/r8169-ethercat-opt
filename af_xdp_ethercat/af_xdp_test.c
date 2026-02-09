// SPDX-License-Identifier: GPL-2.0
/*
 * AF_XDP EtherCAT Test Utility (libbpf-only version)
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
#include <sys/resource.h>
#include <stdbool.h>
#include <net/if.h>
#include <linux/if_link.h>
#include <linux/if_ether.h>
#include <linux/if_xdp.h>

#include <bpf/bpf.h>
#include <bpf/libbpf.h>
#include <bpf/xsk.h>

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

static void signal_handler(int sig)
{
	(void)sig;
	running = 0;
}

static __u64 gettime_ns(void)
{
	struct timespec ts;
	clock_gettime(CLOCK_MONOTONIC, &ts);
	return ts.tv_sec * 1000000000ULL + ts.tv_nsec;
}

static int bump_memlock_rlimit(void)
{
	struct rlimit rlim = {
		.rlim_cur = RLIM_INFINITY,
		.rlim_max = RLIM_INFINITY,
	};

	return setrlimit(RLIMIT_MEMLOCK, &rlim);
}

static bool has_no_new_privs(void)
{
	FILE *fp;
	char line[128];
	bool enabled = false;

	fp = fopen("/proc/self/status", "r");
	if (!fp)
		return false;

	while (fgets(line, sizeof(line), fp)) {
		if (!strncmp(line, "NoNewPrivs:", 11)) {
			enabled = (atoi(line + 11) == 1);
			break;
		}
	}

	fclose(fp);
	return enabled;
}

static void clear_xdp_prog(unsigned int ifindex)
{
	int ret;

	ret = bpf_set_link_xdp_fd(ifindex, -1, 0);
	if (ret && ret != -ENOENT && ret != -EOPNOTSUPP && ret != -EINVAL) {
		fprintf(stderr, "Warning: failed to clear existing XDP prog: %s\n",
			strerror(-ret));
	}
}

static struct xsk_socket_info *xsk_configure_socket(const char *ifname,
						    int queue_id)
{
	struct xsk_socket_info *xsk_info;
	struct xsk_socket_config xsk_cfg;
	struct xsk_umem_config umem_cfg;
	unsigned int ifindex;
	int xsks_map_fd;
	void *umem_area;
	int ret;
	__u32 idx;

	xsk_info = calloc(1, sizeof(*xsk_info));
	if (!xsk_info)
		return NULL;

	/* Allocate UMEM */
	umem_area = mmap(NULL, NUM_FRAMES * FRAME_SIZE, PROT_READ | PROT_WRITE,
			 MAP_PRIVATE | MAP_ANONYMOUS | MAP_HUGETLB, -1, 0);
	if (umem_area == MAP_FAILED) {
		/* Fall back to regular pages if huge pages not available */
		umem_area = mmap(NULL, NUM_FRAMES * FRAME_SIZE,
				 PROT_READ | PROT_WRITE,
				 MAP_PRIVATE | MAP_ANONYMOUS, -1, 0);
		if (umem_area == MAP_FAILED) {
			fprintf(stderr, "Failed to allocate UMEM: %s\n",
				strerror(errno));
			free(xsk_info);
			return NULL;
		}
	}
	xsk_info->umem_area = umem_area;

	/* Configure UMEM */
	memset(&umem_cfg, 0, sizeof(umem_cfg));
	umem_cfg.fill_size = XSK_RING_PROD__DEFAULT_NUM_DESCS;
	umem_cfg.comp_size = XSK_RING_CONS__DEFAULT_NUM_DESCS;
	umem_cfg.frame_size = FRAME_SIZE;
	umem_cfg.frame_headroom = FRAME_HEADROOM;
	umem_cfg.flags = 0;

	ret = xsk_umem__create(&xsk_info->umem, umem_area,
			       NUM_FRAMES * FRAME_SIZE, &xsk_info->fq,
			       &xsk_info->cq, &umem_cfg);
	if (ret) {
		fprintf(stderr, "Failed to create UMEM: %s\n", strerror(-ret));
		if (ret == -EPERM) {
			fprintf(stderr,
				"Hint: run as root (or with CAP_NET_ADMIN/CAP_BPF) and set 'ulimit -l unlimited'.\n");
			if (has_no_new_privs()) {
				fprintf(stderr,
					"Hint: NoNewPrivs=1 in this environment; privilege escalation is blocked.\n");
			}
		} else if (ret == -ENOMEM) {
			fprintf(stderr,
				"Hint: increase memlock limit and ensure enough memory for UMEM.\n");
		}
		munmap(umem_area, NUM_FRAMES * FRAME_SIZE);
		free(xsk_info);
		return NULL;
	}

	/* Configure socket */
	memset(&xsk_cfg, 0, sizeof(xsk_cfg));
	xsk_cfg.rx_size = XSK_RING_CONS__DEFAULT_NUM_DESCS;
	xsk_cfg.tx_size = XSK_RING_PROD__DEFAULT_NUM_DESCS;
	xsk_cfg.libbpf_flags = XSK_LIBBPF_FLAGS__INHIBIT_PROG_LOAD;
	xsk_cfg.bind_flags = XDP_COPY; /* Start with copy mode for testing */
	xsk_cfg.xdp_flags = XDP_FLAGS_SKB_MODE;

	ifindex = if_nametoindex(ifname);
	if (!ifindex) {
		fprintf(stderr, "if_nametoindex(%s) failed: %s\n", ifname,
			strerror(errno));
		xsk_umem__delete(xsk_info->umem);
		munmap(umem_area, NUM_FRAMES * FRAME_SIZE);
		free(xsk_info);
		return NULL;
	}

	clear_xdp_prog(ifindex);

	ret = xsk_socket__create(&xsk_info->xsk, ifname, queue_id,
				 xsk_info->umem, &xsk_info->rx, &xsk_info->tx,
				 &xsk_cfg);
	if (ret) {
		if (ret == -EBUSY) {
			fprintf(stderr,
				"XSK queue busy on first try, clearing stale XDP and retrying once\n");
			clear_xdp_prog(ifindex);
			ret = xsk_socket__create(&xsk_info->xsk, ifname, queue_id,
						 xsk_info->umem, &xsk_info->rx,
						 &xsk_info->tx, &xsk_cfg);
		}
		if (ret) {
			fprintf(stderr, "Failed to create XSK socket: %s\n",
				strerror(-ret));
			if (ret == -EBUSY)
				fprintf(stderr,
					"Hint: queue %d may already be bound by another AF_XDP process.\n",
					queue_id);
			xsk_umem__delete(xsk_info->umem);
			munmap(umem_area, NUM_FRAMES * FRAME_SIZE);
			free(xsk_info);
			return NULL;
		}
	}

	ret = xsk_setup_xdp_prog(ifindex, &xsks_map_fd);
	if (ret) {
		if (ret == -EBUSY) {
			clear_xdp_prog(ifindex);
			ret = xsk_setup_xdp_prog(ifindex, &xsks_map_fd);
		}
		if (ret) {
			fprintf(stderr, "xsk_setup_xdp_prog failed: %s\n",
				strerror(-ret));
			xsk_socket__delete(xsk_info->xsk);
			xsk_umem__delete(xsk_info->umem);
			munmap(umem_area, NUM_FRAMES * FRAME_SIZE);
			free(xsk_info);
			return NULL;
		}
	}

	ret = xsk_socket__update_xskmap(xsk_info->xsk, xsks_map_fd);
	close(xsks_map_fd);
	if (ret) {
		fprintf(stderr, "xsk_socket__update_xskmap failed: %s\n",
			strerror(-ret));
		xsk_socket__delete(xsk_info->xsk);
		xsk_umem__delete(xsk_info->umem);
		munmap(umem_area, NUM_FRAMES * FRAME_SIZE);
		free(xsk_info);
		return NULL;
	}

	/* Populate fill queue */
	ret = xsk_ring_prod__reserve(&xsk_info->fq,
				     XSK_RING_PROD__DEFAULT_NUM_DESCS, &idx);
	if (ret != XSK_RING_PROD__DEFAULT_NUM_DESCS) {
		fprintf(stderr, "Failed to populate FQ (got %d, expected %d)\n",
			ret, XSK_RING_PROD__DEFAULT_NUM_DESCS);
		xsk_socket__delete(xsk_info->xsk);
		xsk_umem__delete(xsk_info->umem);
		munmap(umem_area, NUM_FRAMES * FRAME_SIZE);
		free(xsk_info);
		return NULL;
	}

	for (__u32 i = 0; i < XSK_RING_PROD__DEFAULT_NUM_DESCS; i++)
		*xsk_ring_prod__fill_addr(&xsk_info->fq, idx + i) =
			i * FRAME_SIZE;
	xsk_ring_prod__submit(&xsk_info->fq, XSK_RING_PROD__DEFAULT_NUM_DESCS);

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
	__u32 idx_rx = 0, idx_fq = 0;
	__u32 rcvd;
	__u64 addr_cache[RX_BATCH_SIZE];
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
		addr_cache[i] = desc->addr;

		/* Process any packet (or filter EtherCAT) */
		if (len >= sizeof(struct ethhdr)) {
			__u64 latency = gettime_ns() - start_time;

			xsk_info->rx_packets++;
			xsk_info->rx_bytes += len;
			xsk_info->rx_timestamp_sum += latency;
			xsk_info->rx_count++;

			if (latency < xsk_info->rx_timestamp_min)
				xsk_info->rx_timestamp_min = latency;
			if (latency > xsk_info->rx_timestamp_max)
				xsk_info->rx_timestamp_max = latency;

			/* Check if EtherCAT */
			if (ntohs(eth->h_proto) == ETH_P_ETHERCAT) {
				/* EtherCAT packet received */
			}
		}
	}

	xsk_ring_cons__release(&xsk_info->rx, rcvd);

	/* Refill FQ */
	if (xsk_ring_prod__reserve(&xsk_info->fq, rcvd, &idx_fq) == rcvd) {
		for (__u32 i = 0; i < rcvd; i++) {
			*xsk_ring_prod__fill_addr(&xsk_info->fq, idx_fq + i) =
				addr_cache[i];
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
	printf("RX Rate: %.2f pps\n",
	       elapsed_sec > 0 ? xsk_info->rx_packets / elapsed_sec : 0);
	printf("Throughput: %.2f Mbps\n",
	       elapsed_sec > 0 ?
		       (xsk_info->rx_bytes * 8.0) / (elapsed_sec * 1000000) :
		       0);

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
	} else {
		printf("\nNo packets received.\n");
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
	setvbuf(stdout, NULL, _IONBF, 0);

	if (bump_memlock_rlimit())
		fprintf(stderr, "Warning: failed to raise RLIMIT_MEMLOCK: %s\n",
			strerror(errno));

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
	printf("Waiting for packets...\n");

	fds[0].fd = xsk_socket__fd(xsk_info->xsk);
	fds[0].events = POLLIN;

	start_time = gettime_ns();
	end_time = start_time + (__u64)duration * 1000000000ULL;

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
