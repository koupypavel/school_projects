# PDS – Network Stack Throughput Testing

**Course project** · PDS (Data Communications, Computer Networks and Protocols) · Brno University of Technology, Faculty of Information Technology · 2020
**Author:** Pavel Koupý

> English adaptation of the original Czech documentation ([`dokumentace.pdf`](dokumentace.pdf)), translated and condensed. The Czech setup notes in [`linux/`](linux) are translated in [Appendix A](#appendix-a--setup-notes).

## Overview

Comparison of packet forwarding on a **Raspberry Pi 3** using (a) Linux kernel IP forwarding and (b) **XDP** (eXpress Data Path), an eBPF hook that processes packets before the kernel allocates `sk_buff`. Packet rate was measured for three scenarios: kernel forwarding, XDP redirect, and a direct link without routing (baseline). Result: XDP forwarded ≈ 2× the packet rate of the kernel stack, close to the direct-link rate.

**Keywords:** XDP, eBPF, packet forwarding, Linux network stack, Raspberry Pi, throughput, packets per second

## Contents

1. [Scope](#1-scope)
2. [XDP technology](#2-xdp-technology)
3. [Methodology](#3-methodology)
   - [3.1 Topology](#31-topology)
   - [3.2 Scenarios](#32-scenarios)
   - [3.3 XDP program](#33-xdp-program)
4. [Hardware and environment](#4-hardware-and-environment)
5. [Measurements](#5-measurements)
6. [Conclusion](#6-conclusion)
7. [Repository layout](#repository-layout)
8. [Appendix A – Setup notes](#appendix-a--setup-notes)
9. [References](#references)

---

## 1. Scope

- **Subject:** packet routing via the Linux network stack vs. XDP on a Raspberry Pi 3 single-board computer.
- **Context:** XDP targets dedicated routers with NICs at tens of Gbps and above. The target here is a quad-core ARM board whose Ethernet controller is attached over USB.
- **Interfaces:** built-in Ethernet port and an external USB Ethernet adapter.

## 2. XDP technology

XDP [2, 3] processes packets at a low level, typically inside the network device driver, before the `sk_buff` structure is allocated.

- Built on extended BPF (eBPF): verified XDP bytecode is loaded dynamically and attached to the XDP network hook.
- An in-kernel Just-In-Time compiler translates BPF bytecode to native opcodes.
- The program's return value selects the packet action:

| Action | Effect | Typical use |
|---|---|---|
| `XDP_PASS` | Hands the packet to the regular network stack | Normal processing |
| `XDP_REDIRECT` | Redirects the packet to another interface | Forwarding / routing (used in this project) |
| `XDP_TX` | Transmits the packet back out of the ingress interface | Load balancing |
| `XDP_DROP` | Drops the packet | Firewalls, DDoS mitigation |

## 3. Methodology

- **Metric:** packets per second (pps). Packet rate reflects the packet-processing capacity of the kernel; bit rate (hundreds of Mbps to Gbps) mainly reflects NIC speed.
- **Tooling:** sender/receiver code from *How to receive a million packets* [1] (not included in this repository).
  - **Sender:** transmits a configurable number of UDP packets with a configurable payload to a given address and port.
  - **Receiver:** receives the packets and prints the number processed. This output is the data source for [Section 5](#5-measurements).

### 3.1 Topology

Three Linux nodes: **RPi1** (router), **RPi2** (receiver), **PC1** (traffic generator).

<p align="center">
  <img src="docs/images/topology.png" alt="Topology: PC1 – RPi1 (router) – RPi2" width="750"><br>
  <em>Figure 1: Topology used</em>
</p>

| Node | Interface | IP address | MAC address |
|---|---|---|---|
| PC1 | `enp0s25` | 192.168.0.4/24 | `68:f7:28:55:ce:ce` |
| RPi1 | `eth0` (built-in) | 192.168.0.1/24 | `b8:27:eb:a2:a3:28` |
| RPi1 | `eth1` (USB adapter) | 192.168.1.1/24 | `00:e0:81:31:03:53` |
| RPi2 | `eth0` | 192.168.1.2/24 | `b8:27:eb:d8:29:bf` |

### 3.2 Scenarios

| # | Scenario | Router (RPi1) | Generator | Receiver |
|---|---|---|---|---|
| 1 | Kernel forwarding | `ip_forward` enabled | PC1, static route to 192.168.1.0/24 | RPi2, static route to 192.168.0.0/24 |
| 2 | XDP `bpf_redirect()` | `ip_forward` disabled; XDP program redirects all packets statically to the egress interface and rewrites destination MAC to a constant ([`src/xdp/xdp_prog_kern.c`](src/xdp/xdp_prog_kern.c)) | PC1 | RPi2 |
| 3 | No routing (direct link) | — (RPi1 is the receiver) | PC1 | RPi1 |

- Scenario 3 is the reference for Linux network stack receive performance on the Raspberry Pi without routing.
- **Transport:** UDP. No replies/acknowledgements are routed back and no connection state is created, which permits the static-redirect example from the XDP tutorial [3].
- **Measured quantity:** packets delivered from the sender process to the receiver process.
- **Independent variable:** routing technology on RPi1.
- **Packet parameters:** 32-byte payload, 74 bytes on Ethernet, batches of 1024 packets.

### 3.3 XDP program

[`src/xdp/xdp_prog_kern.c`](src/xdp/xdp_prog_kern.c) is derived from the *packet03 – redirecting packets* lesson of the xdp-tutorial [3]. Program sections:

| Section | Function | Behaviour |
|---|---|---|
| `xdp_eth` | `xdp_redirect_funcc` | Sets destination MAC to `b8:27:eb:d8:29:bf` (RPi2), calls `bpf_redirect(3, 0)`. Attached to RPi1 `eth0`; forwards PC1 → RPi2. |
| `xdp_usb` | `xdp_redirect_func` | Sets destination MAC to `68:f7:28:55:ce:ce` (PC1), calls `bpf_redirect(2, 0)`. Reverse direction, for the USB interface. |
| `xdp_pass` | `xdp_pass_func` | Returns `XDP_PASS` only. |

- Redirecting sections parse only the Ethernet header (up to 4 VLAN tags).
- Every action is counted in the per-CPU array map `xdp_stats_map`.
- Interface indexes 2 and 3 are hard-coded and correspond to RPi1 `eth0` and `eth1`.

## 4. Hardware and environment

| Device | CPU | RAM | Network |
|---|---|---|---|
| Raspberry Pi 3 Model B V1.2 (RPi1, RPi2) | 4-core ARMv8 Cortex-A53, 1.2 GHz | 1 GB | 10/100 Mbps Ethernet connected over USB |
| PC1 | 4-core Intel Core i5-5200U, 2.2 GHz | 8 GB | 10/100 Mbps Ethernet connected over PCI |

- **OS:** Raspbian Buster (official Raspberry Pi image).
- **Kernel:** v4.19.114-v7, rebuilt with BPF bytecode loading and JIT support (native build on the Raspberry Pi or cross-compiled on a PC).
- **Required kernel options:**
  - XDP sockets (networking options; not strictly required for this measurement),
  - BPF framework and the `bpf()` system call,
  - BPF Just-In-Time compiler.
- **Build artefacts:** `zImage` (kernel image), modules, Device Tree Blobs. Procedure: [Appendix A](#appendix-a--setup-notes).

## 5. Measurements

**Setup:** topology per [Section 3.1](#31-topology); 74-byte UDP packets, batches of 1024.
**Procedure:** scenarios 1–3 per [Section 3.2](#32-scenarios); receiver packet count recorded over time.

<p align="center">
  <img src="docs/images/throughput-graph.png" alt="Packet processing rate over time for kernel forwarding, XDP and no routing" width="720"><br>
  <em>Figure 2: Packet processing rate (Mpps) over time. Blue: kernel forwarding, red: XDP, yellow: "Bez směrování" = no routing (direct link)</em>
</p>

**Result** (approximate steady-state values read from Figure 2):

| Scenario | Packet rate |
|---|---|
| 1 – Kernel forwarding | ≈ 0.050 Mpps |
| 2 – XDP redirect | ≈ 0.109 Mpps |
| 3 – No routing (direct link) | ≈ 0.123 Mpps |

**Theoretical limit:**

- Link throughput measured with `iperf`: **95 Mbps**.
- Theoretical maximum for 74-byte UDP packets: **≈ 0.158 Mpps**.
- The direct link reaches ≈ 0.125 Mpps (text of the original documentation; the graph reading above gives ≈ 0.123 Mpps). XDP is close to this value.

## 6. Conclusion

- XDP redirect forwarded packets through the Raspberry Pi 3 at ≈ 2× the rate of kernel IP forwarding (≈ 0.109 vs. ≈ 0.050 Mpps).
- XDP rate was close to the direct-link rate, i.e. the maximum achievable without further system tuning. Such tuning is possible and would probably raise the packet rate further.
- **Known limitation:** the XDP program uses hard-coded interface indexes and destination MACs and is not suitable for real routing. Proper routing requires BPF maps with forwarding information, populated either from a first slow-path pass through the kernel stack or from user-configured static routes.

## Repository layout

```
PDS/
├── dokumentace.pdf             # original documentation (Czech)
├── docs/images/                # figures used in this README
├── linux/                      # setup notes (Czech), translated in Appendix A
│   ├── kernel_forwarding.txt   # IP forwarding, iptables and static routes
│   ├── preklad_konfigurace_kernel.txt  # building the Raspberry Pi kernel
│   └── xdp.txt                 # loading/unloading the XDP program
└── src/
    ├── xdp/
    │   ├── xdp_prog_kern.c     # XDP redirect programs (sections xdp_eth, xdp_usb, xdp_pass)
    │   └── Makefile            # builds xdp_prog_kern.o via common.mk
    ├── common/                 # build rules and helpers from xdp-tutorial (common.mk, parsing helpers, ...)
    ├── headers/                # BPF / kernel UAPI headers from xdp-tutorial
    └── libbpf/                 # vendored copy of libbpf (third-party)
```

Key files: [`src/xdp/xdp_prog_kern.c`](src/xdp/xdp_prog_kern.c), [`src/xdp/Makefile`](src/xdp/Makefile), [`src/common/common.mk`](src/common/common.mk).

- `src/common`, `src/headers` and the build system originate from the xdp-tutorial [3].
- `src/libbpf` is a vendored copy of the third-party [libbpf](https://github.com/libbpf/libbpf) library.
- `make` in `src/xdp` (requires `clang`/`llc` and a built libbpf) produces only the BPF object `xdp_prog_kern.o`.
- The `xdp_loader` tool referenced in Appendix A is not built here; it is part of the xdp-tutorial repository.

## Appendix A – Setup notes

Translated from the Czech notes in [`linux/`](linux). Commands are verbatim from the source.

### A.1 Building the kernel with BPF/XDP support

Source: [`linux/preklad_konfigurace_kernel.txt`](linux/preklad_konfigurace_kernel.txt). Native build on a Raspberry Pi 3 (32-bit `kernel7`):

```bash
# build dependencies
sudo apt install git bc bison flex libssl-dev make
sudo apt install libncurses5-dev
# Raspberry Pi kernel sources
git clone --depth=1 https://github.com/raspberrypi/linux
cd linux
KERNEL=kernel7
make bcm2709_defconfig
# enable XDP sockets, BPF, the bpf() syscall and the BPF JIT (see Section 4)
make menuconfig

# build kernel image, modules and device tree blobs, then install them
make -j4 zImage modules dtbs
sudo make modules_install
sudo cp arch/arm/boot/dts/*.dtb /boot/
sudo cp arch/arm/boot/dts/overlays/*.dtb* /boot/overlays/
sudo cp arch/arm/boot/dts/overlays/README /boot/overlays/
sudo cp arch/arm/boot/zImage /boot/$KERNEL.img
```

### A.2 Kernel forwarding (scenario 1)

Source: [`linux/kernel_forwarding.txt`](linux/kernel_forwarding.txt).

```bash
# enable IPv4 forwarding on the router
echo 1 > /proc/sys/net/ipv4/ip_forward

# allow forwarding between the two interfaces
iptables -A FORWARD -i enp0s25 -o enx00e081310353 -j ACCEPT
iptables -A FORWARD -i enx00e081310353 -o enp0s25 -m state --state ESTABLISHED,RELATED -j ACCEPT

# static routes on the end hosts (sender and receiver)
sudo ip route add 192.168.1.0/24 via 192.168.0.1
sudo ip route add 192.168.0.0/24 via 192.168.1.1
```

`enx00e081310353` is the predictable interface name of the USB Ethernet adapter with MAC `00:e0:81:31:03:53` (RPi1 `eth1` in Figure 1).

### A.3 Loading the XDP program (scenario 2)

Source: [`linux/xdp.txt`](linux/xdp.txt).

Load with the xdp-tutorial loader (supports BPF map creation):

```bash
sudo ./xdp_loader -A -F -d eth0 --filename xdp_prog_kern.o --progsec xdp_eth
```

Load with iproute2 (no map creation, no BPF filesystem support):

```bash
ip link set dev lo xdpgeneric obj xdp_prog_kern.o sec xdp_eth
```

Unload:

```bash
sudo ip link set dev eth0 xdpgeneric off
```

## References

1. Cloudflare Blog. *How to receive a million packets per second.* <https://blog.cloudflare.com/how-to-receive-a-million-packets/>
2. IO Visor Project. *XDP – eXpress Data Path.* <https://www.iovisor.org/technology/xdp>
3. xdp-project. *XDP tutorial.* <https://github.com/xdp-project/xdp-tutorial>
4. Suchakra Sharma. *An entertaining eBPF XDP adventure.* <https://suchakra.wordpress.com/2017/05/23/an-entertaining-ebpf-xdp-adventure/>
5. FIT BUT, PDS course. *Zpracování paketů* (Packet processing), lecture slides. <https://wis.fit.vutbr.cz/FIT/st/cfs.php.cs?file=%2Fcourse%2FPDS-IT%2Flectures%2Fpds-zpracovani-paketu.pdf&cid=13440>
