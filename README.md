# BMW i3 CSC BMS — LilyGo T-CAN485

A battery management system firmware for reusing BMW i3 Cell Supervision Circuit (CSC) modules in DIY EV and energy storage builds. Runs on the [LilyGo T-CAN485](https://github.com/Xinyuan-LilyGO/T-CAN485) (ESP32 + SN65HVD231 CAN transceiver) and communicates with up to 8 BMW i3 CSC modules over CAN bus.

Based on the [SimpBMS](https://github.com/Tom-evnut/SimpBMS) architecture by Tom-evnut, ported from Teensy 3.2 to ESP32 with a fully reverse-engineered BMW i3 CSC protocol.

A note about the BMWI3BUS Variant.  A number of BMWI3 battery packs were bought at the receivership auction of Vicinity Motor Corp.  It appears that a transit bus they were working on had 4 of these packs in it and 3 of these 120aH packs were purchased.  It was discovered that the SME and CSC's were different from a standard BMW i3 SME/CSCs.   So this repository has an additional CSCVariant for these oddball packs.  The hardware in the packs appears identical but the firmware on the SME and CSC's appears to be different.

The CSCVariant=BMSI3BUS can messages were decoded using Claude and code was made by Claude with steering/review/auditing by me (hpeyerl@).
---

## Features

- Reads cell voltages (12 cells per module, 1 mV resolution) and temperatures from up to 8 BMW i3 CSC modules
- Outputs standard Victron/SimpBMS CAN messages (0x351, 0x355, 0x356, 0x35A, 0x35E, 0x35F) for integration with Victron GX devices, inverters, and chargers
- WiFi AP mode with JSON status endpoint for monitoring
- Serial console for configuration and diagnostics
- NVS-backed settings with XOR checksum validation
- Supports three CSC variants (selectable at runtime):
  - `BMWI3` — Original BMW i3 CSC (different CAN ID scheme, default)
  - `MINIE` — BMW Mini-E CSC modules
  - `BMWI3BUS` — BMW i3 pack CSC modules (confirmed working)

---

## Hardware

| Component | Details |
|-----------|---------|
| MCU board | LilyGo T-CAN485 v1.1 (ESP32, 4 MB flash) |
| CAN transceiver | SN65HVD231 (on-board) |
| USB-UART | CH9102 (on-board) |
| CAN speed | 500 kbps |

### Key Pin Assignments

| Pin | GPIO | Function |
|-----|------|----------|
| CAN TX | 27 | SN65HVD231 TXD |
| CAN RX | 26 | SN65HVD231 RXD |
| CAN SE | 23 | SN65HVD231 RS — must be driven LOW for high-speed mode |
| BOOST EN | 16 | ME2107A 5V boost — must be HIGH before CAN init |

> **Note:** Some T-CAN485 board revisions use GPIO5/GPIO4 for CAN TX/RX. Verify against your board schematic and update `pin_config.h` if needed.

---

## BMW i3 CSC Wiring

The CSC modules daisy-chain from one to the next. Per the SimpBMS manual (p.16):

**Individual slave connector (12-pin):**

| Pin | Function |
|-----|----------|
| 1 | 5V supply |
| 4 | CAN L |
| 5 | CAN H |
| 6 | GND |
| 7 | 5V supply (to next module) |
| 10 | CAN L (to next module) |
| 11 | CAN H (to next module) |
| 12 | GND (to next module) |

**Master connector** (connects to first and last CSC in chain):

| Pin | Function |
|-----|----------|
| 5, 11 | 5V supply |
| 6, 12 | GND |
| 7 | CAN H (to BMS) |
| 8 | CAN L (to BMS) |
| 1, 2 | 120Ω termination resistor |

Place a 120Ω resistor between CAN H and CAN L at the BMS end. The master connector pins 1+2 provide the far-end termination.

> **Important:** CSC modules without a battery module connected will hold the CAN bus dominant and prevent communication. Use passive bridge dongles (shorting power and CAN pins through) for unpopulated slots.

---

## Protocol — BMWI3BUS Variant

Reverse engineered from SME (battery management ECU) CAN captures.

### TX — BMS → CSC (0x080–0x087)

Sent every 24 ms as a burst of 8 frames (one per CSC slot):

```
Byte:  D1    D2    D3    D4    D5    D6    D7       D8
       0xC7  0x10  0x00  0x50  0x20  0x00  counter  CRC8
```

D4 steps through an init sequence on startup: `0x00 → 0x00 → 0x00 → 0x10 → 0x50`

Counter increments by 0x10 per cycle (0x00, 0x10, 0x20 … 0xF0, wrap).

CRC8: polynomial 0x1D, init 0xFF, input = `[0x00, id_lo, D1–D7]`, XOR result with `finalxor[slot]`.

### RX — CSC → BMS

| Frame ID | Content |
|----------|---------|
| `0x10N` | Heartbeat / status (N = module address 0–7) |
| `0x12N` | Cells 1–3: 3 × LE 16-bit, 1 mV/bit |
| `0x13N` | Cells 4–6 |
| `0x14N` | Cells 7–9 |
| `0x15N` | Cells 10–12 |
| `0x16N` | Raw NTC ADC values (3 thermistors, LE 16-bit each) |
| `0x17N` | Status2: D5 = temperature + 40 = °C |
| `0x1CN` | Balance / fault status flags |
| `0x1DN` | Additional status flags |

Cell voltage frames only appear after the init sequence completes and `0x011N` transitions to `0x11`.

---

## Building

Requires [PlatformIO](https://platformio.org/).

```bash
pio run                        # build
pio run --target upload        # flash
pio device monitor             # serial console at 115200 baud
```

Tested with `espressif32 @ 6.5.0`. Flash settings are fixed to `dio` / `40 MHz` — do not change these for the T-CAN485.

---

## Serial Console

Connect at 115200 baud. Press `s` to enter the settings menu.

Key commands:

| Command | Function |
|---------|----------|
| `s` | Enter settings menu |
| `CSCVARIANT 0/1/2` | Set CSC variant (0=BMWI3, 1=MiniE, 2=BMWI3BUS) |
| `OVERV <mV>` | Over-voltage setpoint per cell |
| `UNDERV <mV>` | Under-voltage setpoint per cell |
| `OVERT <C>` | Over-temperature setpoint |
| `NUMCELLS <n>` | Cells per module |
| `NUMSERIES <n>` | Modules in series |

---

## WiFi

Default AP: `BMWI3BMS` / password: `bmwpack1`

JSON status endpoint: `http://192.168.4.1/status`

---

## CSC Module Notes

- Label: `CSC BEV 13 D1` (BMW part 8839458-01, made by Preh)
- 12 cells per module, 3 thermistors (pins 1/2, 16/17, 14/15 on 26-pin module connector)
- Module address (lower nibble of CAN frame ID) is firmware-coded in the CSC, not harness-assigned
- Maximum 8 CSC modules per CAN bus (addresses 0–7)
- All modules must have battery cells connected — unpopulated modules hold the bus dominant

---

## Credits

- [Collin Kidder / collin80](https://github.com/collin80/TeslaBMS) — original SimpBMS architecture (MIT)
- [Tom-evnut](https://github.com/Tom-evnut/BMWPhevBMS) — BMW i3 CSC CAN protocol research (MIT)
- [ronaegis](https://github.com/ronaegis/tesla-bms-esp32s3) — ESP32 TWAI port pattern (MIT)
- [SimpBMS / Engovis](https://github.com/Tom-evnut/SimpBMS) — wiring reference and CSC connector pinouts

---

## License

MIT
