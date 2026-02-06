# AF_XDP EtherCAT Test Suite

Low-latency EtherCAT communication testing with AF_XDP zero-copy.

## Prerequisites

```bash
sudo apt install libbpf-dev libelf-dev clang
```

## Build

```bash
make
```

## Usage

### 1. Load the r8169_xdp driver

```bash
cd ../r8169_xdp
make install
```

### 2. Run test

```bash
# Basic test (10 seconds)
sudo ./af_xdp_test -i eth0 -t 10

# With RT priority (recommended)
sudo chrt -f 99 taskset -c 3 ./af_xdp_test -i eth0 -t 60
```

### 3. Options

| Option | Description        | Default    |
| ------ | ------------------ | ---------- |
| `-i`   | Network interface  | (required) |
| `-q`   | Queue ID           | 0          |
| `-t`   | Duration (seconds) | 10         |

## Output Example

```
=== AF_XDP Test Results ===
Duration: 10.00 seconds
RX Packets: 10000
RX Rate: 1000.00 pps

=== Latency Statistics ===
Min: 2500 ns (2.50 us)
Max: 15000 ns (15.00 us)
Avg: 5000.00 ns (5.00 us)
```

## Files

| File                 | Description                 |
| -------------------- | --------------------------- |
| `af_xdp_test.c`      | AF_XDP latency test utility |
| `xdp_ethercat.bpf.c` | XDP BPF filter (optional)   |
| `Makefile`           | Build system                |

## Cleanup

```bash
cd ../r8169_xdp
make uninstall
```
