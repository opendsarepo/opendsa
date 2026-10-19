# OpenDSA

**OpenDSA** is an FPGA-based framework for accelerating fine-grained I/O through software-defined, hardware-managed submission and completion queues (SQ/CQ). It moves request metadata processing, DMA command generation, and request progression into FPGA hardware, reducing software control overhead.

This README documents the **OpenDSA PCIe Gen3 x8 hardware prototype** and **OpenDSA's own experimental results** measured on that platform.

## Repository Structure

```text
opendsa/
├── hardware_code/      # OpenDSA FPGA hardware sources
├── software_code/      # Host software and on-card runtime sources
├── pcb_schematic/      # Prototype board schematic files
└── README.md
```

## 1. Hardware Prototype

The OpenDSA prototype is implemented on a Xilinx Zynq UltraScale+ ZU19EG FPGA PCIe add-in card installed in a Supermicro server. The FPGA-based Host Bus Adapter (HBA) presents a block device to the host Linux system.

| Component | Configuration |
| --- | --- |
| FPGA | Xilinx Zynq UltraScale+ ZU19EG |
| On-card processor | Arm Cortex-A53 |
| Host server | Supermicro Super Server |
| Host CPU | Intel Xeon Silver 4314 |
| Host memory | 64 GB DDR4-3200 |
| Host interface | PCIe Gen3 x8 |
| FPGA data-path clock | 250 MHz |
| FPGA control-path clock | 150 MHz |
| DMA channels | 1 read + 1 write |
| Submission queue depth | 32 |
| PCIe MPS / MRRS | 512 B / 512 B |
| Read Completion Boundary | 64 B |
| Host completion notification | MSI-X interrupts |
| Device backing store | 1 MB BRAM or external DDR (depending on experiment) |

### OpenDSA Architecture

OpenDSA consists of four layers:

1. **API:** Software-visible configuration and SQ/CQ interfaces.
2. **IRAL (I/O Request Acceleration Layer):** Hardware-based metadata fetching, parsing, dependent transfer generation, and request completion.
3. **DIAL (DMA Interface Abstraction Layer):** Translation of transfer requests into DMA-specific commands.
4. **DMA & Protocol Layer:** PCIe/AXI data movement and DMA execution.

In the full **IRAL-enabled OpenDSA** design, the on-card Cortex-A53 submits request identifiers, while the FPGA hardware carries out metadata processing, data transfer orchestration, and completion generation.

## 2. Software Environment

| Component | Version |
| --- | --- |
| Host operating system | Ubuntu 20.04.6 LTS |
| Linux kernel | 5.4.0 |
| FPGA design tool | Vivado 2023.1 |
| Firmware development tool | Vitis 2023.1 |
| I/O benchmark | fio 3.16 |
| Application benchmark | RocksDB (fillseq and readrandom) |

The measurements below use the **Linux block-device I/O path**, including the host driver, PCIe link, and OpenDSA FPGA request-processing logic; they are not raw DMA-only microbenchmarks.

## 3. fio Evaluation

### Benchmark Configuration

| Parameter | Configuration |
| --- | --- |
| OpenDSA implementation | Full IRAL-enabled OpenDSA |
| PCIe interface | Gen3 x8 hardware prototype |
| I/O operations | Read and write |
| I/O sizes | 4, 8, 16, 32, 64, 128, 256, 512 KB; 1 MB |
| Queue depth (`iodepth`) | 128 |
| Parallel jobs (`numjobs`) | 16 |
| Reported metrics | Throughput and IOPS |


## 4. RocksDB Evaluation

RocksDB is evaluated on the PCIe Gen3 x8 OpenDSA prototype using **128 MB of DDR-backed device storage**.

### Workload Configuration

| Parameter | Configuration |
| --- | --- |
| Workloads | `fillseq`, `readrandom` |
| Key size | 16 B |
| Value size | 128 B |
| Block size | 4 KB |
| Compression | Disabled |
| `fillseq` | 320,000 writes, 16 threads, WAL disabled |
| `readrandom` | Direct I/O, 30-second duration, 1-32 threads |

### Measured Application Results

| Workload | OpenDSA result |
| --- | --- |
| `fillseq` (16 threads) | 948.0 Kops/s; 16.69 us average latency |
| `readrandom` (32 threads) | 537.8 Kops/s; 59.5 us average latency |

## 5. Environment Verification

After programming the FPGA, deploying the on-card runtime, and loading the host-side driver from the repository, confirm the operating environment and the Linux-visible block device before running benchmarks.

```bash
uname -r
lspci -vv
lsblk
fio --version
```

The precise FPGA build commands, driver-loading commands, device name, and benchmark job files depend on the released sources and target board setup. They are not specified in the paper and are therefore not fabricated here.

> **Caution:** Block-device write benchmarks can overwrite existing data. Verify the test-device path before running `fio` or RocksDB.
