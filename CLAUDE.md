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
- 16-bit CRC on payload bytes
- Time budget per half-cycle: 54 µs max frame + 46 µs turnaround

## Architecture

The IP core follows the same structure as sibling projects in `/raid/work/ghl_ip/` (especially `dshot`):

- **AXI-Lite slave** interface for register access
- **Configurable number of AeroLink ports**, each set as master or slave at synthesis time
- **Per-port resources:**
  - 2 TX FIFOs (high priority + regular), 512 bytes each, 32-bit AXI access
  - 2 RX FIFOs (high priority + regular), 512 bytes each, 32-bit AXI access
  - TX control word FIFO (priority, CRC auto-gen, repeat count 0-8, optional CRC value)
  - RX control word FIFO (priority byte, message length, error status, received CRC)
  - Control bits: TX enable, RX enable, TX reset, RX reset, drop errored frames
  - Maskable interrupts: TX queue empty (per FIFO), TX complete (per FIFO), RX FIFO not empty (per FIFO)
  - 16-bit statistics counters: TX frames, TX idle frames, RX frames, RX CRC errors, RX disparity errors, RX symbol errors
- **I/O selectable** between single-ended LVCMOS and LVDS differential
- **Vivado GUI** for port count and I/O style selection

## Directory Structure

| Directory | Purpose |
|-----------|---------|
| `doc/` | Protocol specification |
| `rtl/` | Synthesizable Verilog RTL source |
| `tb/` | Simulation testbenches |
| `ip_repo/` | Vivado-packaged IP core (component.xml, xgui, src) |

## Build and Simulation

This project targets Xilinx Vivado. Follow sibling project conventions (see `/raid/work/ghl_ip/dshot/` as reference):

- **RTL language:** Verilog (`.v` files)
- **IP packaging:** Tcl script (`package_ip_core.tcl`) generates `ip_repo/<core>/component.xml`
- **Simulation:** Vivado simulator or Icarus Verilog with testbenches in `tb/`
- **IP repo discovery:** Place packaged IP under `ip_repo/` for Vivado block design integration

## Sibling IP References

The `dshot` project in the same parent directory is the closest architectural reference — it has a similar AXI-Lite controlled, FIFO-based TX/RX design with oversampling, CRC, and interrupt support.
