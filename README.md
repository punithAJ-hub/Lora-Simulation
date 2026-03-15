# LoRa Network Simulation

A low-power wide-area network (LPWAN) simulation implementing the LoRa radio protocol in C. Models multi-gateway topologies, packet collision rates, and signal propagation for IoT deployment planning.

---

## Overview

This project simulates a LoRaWAN-style network to help plan and evaluate IoT deployments before physical hardware is provisioned. It models the key physical and MAC-layer behaviors of LoRa — spreading factors, signal-to-noise ratios, collision windows, and gateway coverage areas.

---

## Features

- **Multi-Gateway Topology** — simulate networks with multiple gateways and hundreds of end devices
- **Packet Collision Modeling** — calculate collision probability based on spreading factor and airtime
- **Signal Propagation** — path loss model (log-distance) with configurable terrain parameters
- **RSSI / SNR Estimation** — per-link signal quality metrics
- **Coverage Mapping** — output gateway coverage zones as CSV for visualization
- **Configurable Parameters** — spreading factor, bandwidth, transmission power, environment type

---

## Tech Stack

| Layer | Technology |
|-------|-----------|
| Language | C (C99 standard) |
| Protocol | LoRa / LoRaWAN MAC layer (simplified) |
| Math | libm (standard math library) |
| Build | GCC + Makefile |
| Output | CSV / stdout logging |

---

## Getting Started

### Prerequisites

- GCC (or any C99-compatible compiler)
- GNU Make

### Build & Run

```bash
git clone https://github.com/punithAJ-hub/Lora-Simulation.git
cd Lora-Simulation
make
./lora_sim
```

### Configuration

Edit `config.h` to customize simulation parameters:

```c
#define NUM_DEVICES       200
#define NUM_GATEWAYS      3
#define SPREADING_FACTOR  7       // SF7 to SF12
#define BANDWIDTH_KHZ     125
#define TX_POWER_DBM      14
#define ENVIRONMENT       URBAN   // URBAN, SUBURBAN, RURAL
```

---

## Output

```
[SIM] Devices: 200 | Gateways: 3 | SF: 7 | BW: 125kHz
[GW0] Coverage radius: 2.4 km | Connected devices: 87
[GW1] Coverage radius: 2.1 km | Connected devices: 74
[GW2] Coverage radius: 2.6 km | Connected devices: 63
[STATS] Packet Delivery Ratio: 94.2%
[STATS] Avg collision rate: 3.8%
[STATS] Uncovered devices: 0
```

---

## References

- [LoRa Alliance LoRaWAN Specification](https://lora-alliance.org/resource_hub/lorawan-specification-v1-0-3/)
- Semtech SX1276 Datasheet

---

## Author

**Punith Jayaramu**  
[LinkedIn](https://www.linkedin.com/in/punith-agraharavalagehalli-jayaramu-b3289339a/) · [Portfolio](https://punith-portfolio-ten.vercel.app)
