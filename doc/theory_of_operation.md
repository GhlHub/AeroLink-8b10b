# AeroLink-8b10b RTL Theory of Operation

## 1. Scope

This document describes the behavior implemented in the RTL under [`rtl/`](../rtl). It is intentionally focused on the current design as written, not only on the original protocol intent in [`doc/instructions.txt`](./instructions.txt).

The delivered RTL implements:

- An AXI-Lite controlled top level, `aerolink_axil_top`
- A parameterizable number of independent AeroLink ports
- Per-port transmit and receive queues split into high-priority and regular-priority channels
- A transmit engine that frames bytes, generates or forwards CRC, 8b/10b encodes symbols, and serializes them
- A receive engine that oversamples the incoming serial stream, aligns on K28.5 commas, 8b/10b decodes symbols, checks CRC, and deposits received data into FIFOs
- Per-port statistics and interrupt aggregation

The RTL is synchronous to a single clock domain. The AXI-Lite bus, register interface, FIFO logic, TX engine, and RX engine all run from the same `s_axi_aclk` domain.

## 2. Top-Level Architecture

`aerolink_axil_top` is the integration point. It converts AXI-Lite reset polarity, instantiates one `aerolink_axil_regs` block, and generates `NUM_PORTS` copies of `aerolink_port`.

Each `aerolink_port` contains:

- 4 TX-side FIFOs
  - high-priority data
  - high-priority control
  - regular-priority data
  - regular-priority control
- 4 RX-side FIFOs
  - high-priority data
  - high-priority control
  - regular-priority data
  - regular-priority control
- 1 TX engine
- 1 RX engine
- 16-bit saturating statistics counters

The AXI-Lite block exposes a flat register view. It does not directly interpret packet contents beyond moving control words and FIFO payload words between software and each port instance.

## 3. Port-Level Data Model

Each port separates traffic by priority. Software writes packets into either the high-priority or regular-priority transmit queue pair:

- A TX data FIFO carries 32-bit payload words
- A TX control FIFO carries one 32-bit descriptor per frame

Likewise, the receive side exposes:

- An RX data FIFO containing received 32-bit payload words
- An RX control FIFO containing one 32-bit status/control word per received frame

The design assumes packet lengths are multiples of 32 bits. The TX engine reads whole 32-bit words and expands them into bytes in big-endian byte order within each word:

- bits `[31:24]`
- bits `[23:16]`
- bits `[15:8]`
- bits `[7:0]`

The RX engine reconstructs FIFO words in that same order.

## 4. Frame Format Implemented in RTL

The TX engine emits the following symbol sequence for a non-idle frame:

1. `K28.5`
2. `K28.5`
3. `/S/` implemented as `K27.7`
4. 1 priority byte
5. `N` payload bytes
6. 2 CRC bytes, MSB first
7. `/T/` implemented as `K29.7`

So the encoded frame length is `N + 7` symbols, where `N` is the payload byte count excluding CRC.

Master idle behavior is implemented as four consecutive `K28.5` symbols when no TX control FIFO contains a pending frame. A slave with no pending frame remains idle and transmits nothing.

## 5. AXI-Lite Register Organization

The register file is split into global space and per-port windows.

Global space:

- `0x0000`: version, fixed to `0x0001_0000`
- `0x0004`: combined IRQ state in bit 0
- `0x0008`: `NUM_PORTS`

Per-port space:

- Port `n` base address = `0x0100 + n * 0x0100`

Per-port offsets:

- `0x00`: control
- `0x04`: FIFO status
- `0x08`: IRQ status
- `0x0C`: IRQ mask
- `0x10`: TX high-priority data write
- `0x14`: TX high-priority control write
- `0x18`: TX regular-priority data write
- `0x1C`: TX regular-priority control write
- `0x20`: RX high-priority data read
- `0x24`: RX high-priority control read
- `0x28`: RX regular-priority data read
- `0x2C`: RX regular-priority control read
- `0x30`: TX frame count
- `0x34`: TX idle frame count
- `0x38`: RX frame count
- `0x3C`: RX CRC error count
- `0x40`: RX disparity error count
- `0x44`: RX symbol error count

The AXI-Lite implementation is simple:

- write address and write data are captured independently
- the write executes once both are present
- read data is returned one cycle after address acceptance
- reading an RX FIFO register both returns the current FWFT word and issues a one-cycle FIFO pop pulse

Control bits in `REG_CTRL`:

- bit 0: `tx_enable`
- bit 1: `rx_enable`
- bit 2: `tx_reset` self-clearing pulse
- bit 3: `rx_reset` self-clearing pulse
- bit 4: `drop_errored`

## 6. FIFO Operation

`aerolink_fifo` is a synchronous first-word-fall-through FIFO. The head element is always visible on `rd_data`, and a read pulse advances the pointer if the FIFO is non-empty.

Data FIFOs use parameter `FIFO_DEPTH`, default `128` 32-bit words. Control FIFOs are fixed at depth `32`.

Because the register file reads FWFT data and simultaneously asserts a read pulse, software sees the current head word on the read transaction and the FIFO advances after that cycle.

## 7. TX Path Theory of Operation

### 7.1 Queue selection

The TX engine checks the high-priority control FIFO first. If it is not empty, that queue is serviced before the regular-priority queue. If neither control FIFO contains a descriptor:

- a master port emits a 4-comma idle frame
- a slave port returns to idle without transmitting

### 7.2 TX control word format

The TX engine consumes one 32-bit control word per frame:

- `[7:0]`: priority byte
- `[8]`: auto-generate CRC
- `[11:9]`: repeat count
- `[18:12]`: payload word count

Other bits are currently unused by the RTL.

The `repeat count` is the number of additional retransmissions after the first send. A value of `0` sends the frame once. A value of `7` sends the same frame eight times total.

### 7.3 Payload staging

After latching the control word, the TX engine reads `payload word count` entries from the selected TX data FIFO and copies them into an internal byte buffer. Each word becomes four payload bytes.

The engine relies on the control FIFO descriptor to determine how many data words to read. It does not check `hipri_data_empty` or `regpri_data_empty` before issuing those reads. Correct software sequencing is therefore:

1. write all payload words to the selected TX data FIFO
2. write the matching control word to the selected TX control FIFO

If software pushes a control word before the corresponding payload words are available, frame contents are not protected by a ready/valid handshake inside the engine.

### 7.4 CRC handling

CRC is computed by `aerolink_crc16` using CRC-16-CCITT with polynomial `0x1021`, initial value `0xFFFF`, processed MSB-first one byte per cycle.

The TX engine always includes the priority byte in the CRC calculation.

Two CRC modes exist:

- Auto CRC enabled:
  - CRC is calculated over `priority byte + all payload bytes`
  - the generated 16-bit CRC is appended to the transmitted frame
- Auto CRC disabled:
  - the last two bytes already present in the staged data buffer are treated as the CRC value to transmit
  - the local CRC calculator still runs over `priority byte + payload without those final two CRC bytes`

This means manual-CRC mode expects software to include the CRC bytes inside the payload word stream and to include those two bytes in the word count.

### 7.5 Frame cadence and serialization

The TX engine derives:

- `CLKS_PER_BIT = CLK_FREQ_HZ / (SYMBOL_RATE * 10)`
- `FRAME_PERIOD = CLK_FREQ_HZ / 5000`

At the default parameters:

- `CLK_FREQ_HZ = 125 MHz`
- `SYMBOL_RATE = 2.5 MHz`
- `CLKS_PER_BIT = 5`
- `FRAME_PERIOD = 25,000` clocks = `200 us`

Master cadence is enforced by a period counter that only decrements while the engine is in `S_IDLE`. When the counter expires, the engine checks for queued traffic and either sends a frame or sends a 4-comma idle pattern. After completion it reloads the 200 us timer.

Serialization is done with a 10-bit shift register fed by the 8b/10b encoder. The line transmits `data_out[9]` first, matching the encoder and decoder comments.

## 8. RX Path Theory of Operation

### 8.1 Oversampling and symbol lock

The RX engine uses the same `CLKS_PER_BIT` expression as the transmitter. At the default build point this produces 5x oversampling of each serial bit.

For each bit period, the engine:

- samples `rx_serial` `CLKS_PER_BIT` times
- performs majority vote across those samples
- shifts the voted bit into a 10-bit sliding window

Comma detection looks for the 7-bit K28.5 comma signature in bits `[9:3]` of the current 10-bit window. Once detected, the engine treats that point as a symbol boundary. While aligned, it continues to count 10 bits per symbol and can opportunistically re-align on later commas.

### 8.2 Frame parser

The RX parser state machine is:

1. `F_HUNT`: search for first `K28.5`
2. `F_SYNC`: require second `K28.5`
3. `F_WAIT_SOP`: wait for `/S/` (`K27.7`), tolerating additional commas
4. `F_RX_DATA`: collect decoded bytes until `/T/` (`K29.7`)
5. `F_CHECK_CRC`: reject undersized frames and initialize CRC checking
6. `F_OUTPUT`: finish CRC comparison and choose drop-or-commit
7. `F_OUT_DATA`: write payload words to the selected RX data FIFO
8. `F_OUT_CTRL`: write one RX control word to the matching RX control FIFO

The receive buffer stores up to 132 bytes total:

- 1 priority byte
- up to 128 payload bytes
- 2 CRC bytes
- no storage for `/S/` or `/T/` because they are framing symbols

### 8.3 Error tracking

The decoder returns:

- decoded byte
- K/data indication
- updated running disparity
- disparity error
- symbol/code error

During `F_RX_DATA`, the RX engine accumulates disparity and symbol errors across the entire frame. When `/T/` arrives it:

- checks minimum size: at least 7 buffered bytes
- extracts received CRC from the last two bytes
- computes CRC across `priority byte + payload bytes`
- compares the computed CRC with the received CRC

### 8.4 RX control word format

For accepted frames, the RX engine emits one 32-bit control word:

- `[7:0]`: received priority byte
- `[12:8]`: payload length in 32-bit words
- `[13]`: CRC error
- `[14]`: disparity error
- `[15]`: symbol error
- `[31:16]`: received CRC

Priority routing is based on priority bit 7:

- `priority[7] = 1`: high-priority RX FIFOs
- `priority[7] = 0`: regular-priority RX FIFOs

### 8.5 Drop policy

If `drop_errored` is set and any of these conditions are true:

- CRC mismatch
- disparity error seen
- symbol error seen

then the frame is discarded and neither RX FIFO pair is written. Statistics are still updated.

If `drop_errored` is clear, the frame is still committed to the RX FIFOs and the error bits are reported in the RX control word.

## 9. 8b/10b Codec Behavior

The encoder and decoder are implemented locally in combinational logic and follow standard IBM 8b/10b partitioning:

- 5b/6b subcode
- 3b/4b subcode
- running disparity tracking
- K-character support for `K28.x` and `K23.7`, `K27.7`, `K29.7`, `K30.7`

The TX engine uses:

- `K28.5` for comma / alignment
- `K27.7` for start-of-packet
- `K29.7` for end-of-packet

The RX engine uses the same symbol set for framing recognition.

## 10. Interrupts and Statistics

Per-port interrupt sources are:

- TX high-priority queue empty
- TX regular-priority queue empty
- TX high-priority complete
- TX regular-priority complete
- RX high-priority not empty
- RX regular-priority not empty

IRQ masks are held per port in a 6-bit register. The top-level `irq` output is the OR of all enabled interrupt sources across all ports.

Important implementation detail:

- `TX queue empty` is driven from the corresponding TX control FIFO empty flag, not the data FIFO empty flag
- `RX not empty` is driven from the RX data FIFO empty flag

Each port also maintains six 16-bit saturating counters:

- transmitted frames
- transmitted idle frames
- received frames
- received CRC-error frames
- received disparity-error frames
- received symbol-error frames

## 11. Master and Slave Behavior

At synthesis time, each port is designated as master or slave through `PORT_IS_MASTER`.

Current TX-side behavior:

- Master:
  - waits for the 200 us frame timer
  - transmits either a queued frame or a 4-comma idle pattern
- Slave:
  - transmits whenever a control FIFO is non-empty
  - otherwise remains silent

Current RX-side behavior:

- identical for masters and slaves
- continuously attempts to recover comma-aligned frames when `rx_enable` is set

The present RTL does not implement explicit half-duplex bus ownership, turnaround timing enforcement, or reply-slot scheduling between master and slave. The serial TX and RX pins are modeled as independent wires at the top level.

## 12. Notable Implementation Limits and Assumptions

These points are important for integration because they are properties of the delivered RTL, not just possible future work.

- IO style selection is not implemented in RTL. The top level exposes simple single-bit `aerolink_tx` and `aerolink_rx` ports with no LVDS/LVCMOS wrapper logic.
- The design is not a single shared half-duplex pin-pair controller. It behaves as separate TX and RX signals per port.
- Turnaround windows and master reply timing are not enforced in the current state machines.
- The TX engine uses a 128-byte internal payload buffer, so the payload limit implemented in the engine matches the intended maximum payload size.
- The RX engine accepts frames down to 7 buffered bytes total, which corresponds to `priority + 4 data bytes + 2 CRC bytes`.
- Packet length reporting in the RX control word is in 32-bit words, using bits `[12:8]`.
- TX control descriptors also express length in 32-bit words, using bits `[18:12]`.
- Software must keep TX data FIFO contents coherent with TX control FIFO descriptors because the engine trusts the descriptor length.

## 13. Practical Software Sequence

For software driving one transmit frame:

1. Confirm the chosen TX data and TX control FIFOs are not full.
2. Write all payload words into the selected TX data FIFO.
3. Build the TX control word with priority, CRC mode, repeat count, and payload word count.
4. Write the TX control word.
5. Optionally wait for the corresponding transmit-complete interrupt or poll status/statistics.

For software receiving one frame:

1. Poll or interrupt on RX-not-empty.
2. Read all payload words from the selected RX data FIFO.
3. Read the matching RX control word from the selected RX control FIFO.
4. Use the control word to determine priority, length, CRC value, and error status.

The RTL writes payload data first and the RX control word last, so software should treat the control FIFO entry as the end-of-frame marker for a completed receive transaction.
