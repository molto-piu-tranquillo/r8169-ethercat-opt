// SPDX-License-Identifier: GPL-2.0
/*
 * XDP EtherCAT Filter Program
 * 
 * Filters EtherCAT frames (EtherType 0x88A4) and redirects to AF_XDP socket.
 * Non-EtherCAT traffic passes to normal network stack.
 */

#include <linux/bpf.h>
#include <linux/if_ether.h>
#include <bpf/bpf_helpers.h>
#include <bpf/bpf_endian.h>

#define ETH_P_ETHERCAT 0x88A4

/* XSK map for AF_XDP socket */
struct {
	__uint(type, BPF_MAP_TYPE_XSKMAP);
	__uint(max_entries, 1);
	__uint(key_size, sizeof(int));
	__uint(value_size, sizeof(int));
} xsk_map SEC(".maps");

/* Statistics map */
struct {
	__uint(type, BPF_MAP_TYPE_PERCPU_ARRAY);
	__uint(max_entries, 4);
	__type(key, __u32);
	__type(value, __u64);
} stats_map SEC(".maps");

enum {
	STATS_PKT_TOTAL = 0,
	STATS_PKT_ETHERCAT = 1,
	STATS_PKT_REDIRECTED = 2,
	STATS_PKT_DROPPED = 3,
};

static __always_inline void update_stats(__u32 key)
{
	__u64 *count = bpf_map_lookup_elem(&stats_map, &key);
	if (count)
		__sync_fetch_and_add(count, 1);
}

SEC("xdp")
int xdp_ethercat_filter(struct xdp_md *ctx)
{
	void *data_end = (void *)(long)ctx->data_end;
	void *data = (void *)(long)ctx->data;
	struct ethhdr *eth = data;
	int ret;

	update_stats(STATS_PKT_TOTAL);

	/* Bounds check */
	if ((void *)(eth + 1) > data_end)
		return XDP_PASS;

	/* Check if EtherCAT frame (EtherType 0x88A4) */
	if (eth->h_proto == bpf_htons(ETH_P_ETHERCAT)) {
		update_stats(STATS_PKT_ETHERCAT);

		/* Redirect to AF_XDP socket on queue 0 */
		ret = bpf_redirect_map(&xsk_map, 0, XDP_PASS);
		if (ret == XDP_REDIRECT) {
			update_stats(STATS_PKT_REDIRECTED);
		}
		return ret;
	}

	/* Non-EtherCAT: pass to network stack */
	return XDP_PASS;
}

char LICENSE[] SEC("license") = "GPL";
