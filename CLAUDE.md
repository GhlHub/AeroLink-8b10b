# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Project Overview

AeroLink-8b10b is an FPGA IP core implementing a novel drone communication protocol between flight controllers and ESCs (Electronic Speed Controllers), intended to replace PWM, DShot, and CAN Bus. It uses 8B10B encoding over LVDS at a 2.5 MHz symbol rate, half-duplex on a single wire pair (master/slave).

The full protocol specification lives in `doc/instructions.txt`.

## Protocol Key Parameters

- Symbol rate: 2.5 MHz, receiver oversamples at 5x (12.5 MHz)
- Frame period: 200 µs (master transmits every 200 µs)
- Max frame: 135 symbols (K28.5 + K28.5 + /S/ + priority + 128 data + 2 CRC + /T/)
- Min frame: 11 symbols (K28.5 + K28.5 + /S/ + priority + 4 data + 2 CRC + /T/)
- Idle: 4 K28.5 symbols (master) or silence (slave)
- Valid packet sizes: multiples of 32-bits only
- 16-bit CRC-CCITT (poly 0x1021, init 0xFFFF) over priority byte + data bytes
- Time budget per half-cycle: 54 µs max frame + 46 µs turnaround

## Architecture

- **AXI-Lite slave** interface for register access (125 MHz default clock)
- **Configurable number of AeroLink ports**, each set as master or slave at synthesis time
- **Half-duplex tristate I/O**: each port exposes `aerolink_o`, `aerolink_i`, `aerolink_t` for connection to Xilinx `IOBUFDS` (LVDS) or `IOBUF` (LVCMOS)
- **Local echo suppression**: RX is gated to idle while local TX is driving, preventing self-reception on the shared bus
- **Slave TX triggering**: slave TX engine waits for `rx_frame_done` (from data frames or idle comma sequences) before checking FIFOs, ensuring it only transmits during the turnaround window
- **Per-port resources:**
  - 2 TX FIFOs (high priority + regular), 512 bytes each, 32-bit AXI access
  - 2 RX FIFOs (high priority + regular), 512 bytes each, 32-bit AXI access
  - TX control word FIFO: priority byte, auto-CRC flag, repeat count (0-7), data word count
  - RX control word FIFO: priority byte, data length/4, error flags (CRC/disparity/symbol), received CRC
  - Control bits: TX enable, RX enable, TX reset (self-clearing), RX reset (self-clearing), drop errored frames
  - Maskable interrupts: TX queue empty, TX complete, RX FIFO not empty (per priority level)
  - 16-bit saturating statistics counters: TX frames, TX idle frames, RX frames, RX CRC errors, RX disparity errors, RX symbol errors

## RTL Module Hierarchy

```
aerolink_axil_top          Top-level: AXI-Lite + N ports
├── aerolink_axil_regs     AXI-Lite register decode/mux
└── aerolink_port [N]      Per-port wrapper
    ├── aerolink_fifo [8]  TX/RX data + control FIFOs
    ├── aerolink_tx_engine TX frame assembly + gapless serialization
    │   ├── aerolink_enc_8b10b  Combinational 8B10B encoder
    │   └── aerolink_crc16     CRC-16-CCITT calculator
    └── aerolink_rx_engine RX oversampling + frame parsing
        ├── aerolink_dec_8b10b  Combinational 8B10B decoder
        └── aerolink_crc16     CRC-16-CCITT calculator
```

## Register Map

Global registers at `0x000`-`0x0FF`. Per-port registers at `0x100 * (port + 1)`.

| Offset | Name | Access | Description |
|--------|------|--------|-------------|
| 0x000 | VERSION | RO | 0x00010000 |
| 0x004 | GLOBAL_IRQ | RO | bit[0] = IRQ pin status |
| 0x008 | NUM_PORTS | RO | Port count |
| +0x00 | CTRL | RW | [0]=tx_en [1]=rx_en [2]=tx_rst [3]=rx_rst [4]=drop_err |
| +0x04 | FIFO_STATUS | RO | 16 bits: empty/full per FIFO |
| +0x08 | IRQ_STATUS | RO | 6-bit raw IRQ sources |
| +0x0C | IRQ_MASK | RW | 6-bit mask (1=enabled) |
| +0x10 | TX_HIPRI_DATA | WO | Push to TX high-priority data FIFO |
| +0x14 | TX_HIPRI_CTRL | WO | Push to TX high-priority control FIFO |
| +0x18 | TX_REGPRI_DATA | WO | Push to TX regular-priority data FIFO |
| +0x1C | TX_REGPRI_CTRL | WO | Push to TX regular-priority control FIFO |
| +0x20 | RX_HIPRI_DATA | RO | Pop from RX high-priority data FIFO |
| +0x24 | RX_HIPRI_CTRL | RO | Pop from RX high-priority control FIFO |
| +0x28 | RX_REGPRI_DATA | RO | Pop from RX regular-priority data FIFO |
| +0x2C | RX_REGPRI_CTRL | RO | Pop from RX regular-priority control FIFO |
| +0x30-0x44 | STAT_* | RO | 16-bit statistics counters |

## TX/RX Control Word Formats

**TX Control Word** (written to TX_*_CTRL):
`[7:0]` priority byte, `[8]` auto-CRC, `[11:9]` repeat count, `[18:12]` data length in 32-bit words

**RX Control Word** (read from RX_*_CTRL):
`[7:0]` priority byte, `[12:8]` data length/4, `[13]` CRC error, `[14]` disparity error, `[15]` symbol error, `[31:16]` received CRC

## Build and Simulation

```bash
# Compile and run master testbench (Icarus Verilog)
iverilog -g2005 -o tb/master_tb.vvp rtl/*.v tb/iobufds_sim.v tb/aerolink_master_tb.v
vvp tb/master_tb.vvp

# Compile and run slave testbench
iverilog -g2005 -o tb/slave_tb.vvp rtl/*.v tb/iobufds_sim.v tb/aerolink_slave_tb.v
vvp tb/slave_tb.vvp

# Package IP core for Vivado (requires Vivado in PATH)
vivado -mode batch -source package_ip_core.tcl
```

## Design Decisions and Pitfalls

- **Gapless TX serialization**: the TX shift register and FSM share a single `always` block so the shift register can inline-reload the next symbol when the current one finishes, avoiding inter-symbol gaps that cause RX drift
- **FWFT FIFO read timing**: after asserting `rd_en`, the FIFO pointer advances on the next posedge — the TX engine uses a `S_LOAD_WAIT` state between reads to let the pointer settle before sampling the next word
- **Combinational encoder inputs**: the 8B10B encoder input mux is combinational (driven by `sym_idx`), not registered, so `enc_dout` is valid in the same cycle and no pipeline delay is needed
- **CRC settling**: the CRC module output lags one cycle behind the last `valid` pulse — the TX engine uses a `crc_settling` flag to wait before latching `crc_out`
- **RX majority vote**: uses `(CLKS_PER_BIT + 1) / 2` threshold (3 of 5 for default 125 MHz clock)
- **Idle frame period reset**: the master resets its 200 µs period counter after both data frames (via `S_DONE`) and idle comma sequences (in `S_IDLE_SHIFT`), ensuring a turnaround window for slave responses
