# u-blox GNSS Parser

A portable, **zero-heap** C11 library for parsing both **NMEA 0183** sentences
and **UBX binary** protocol frames from u-blox GNSS receivers (NEO-M8, ZED-F9P,
MAX-M10S, etc.).

Designed for **embedded / bare-metal** targets — no dynamic allocation, no OS
dependencies, safe to call from UART ISRs.

---

## Features

| Layer | Coverage |
|-------|----------|
| **NMEA 0183** | GGA, RMC, GSA, GSV, VTG, GLL |
| **UBX Binary** | NAV-PVT, NAV-STATUS, ACK-ACK, ACK-NAK |
| **Streaming** | Byte-feed state machine for UART / DMA integration |
| **Checksums** | NMEA XOR + UBX Fletcher-8, verified on every frame |
| **Utilities** | Haversine distance, coord conversion, fix-type strings |

---

## Project Structure

```
ublox_gnss_parser/
├── include/
│   └── gnss_parser.h       # Public API & all data structures
├── src/
│   ├── gnss_parser.c       # Streaming state machine + utilities
│   ├── nmea_parser.c       # NMEA sentence parsers
│   └── ubx_parser.c        # UBX binary frame parser
├── test/
│   └── test_gnss_parser.c  # 66-assertion test suite (no framework needed)
└── Makefile
```

---

## Quick Start

### Build & run tests (Linux / macOS)

```bash
make          # builds + runs test suite
```

Expected output:
```
Results: 66/66 passed  (all good!)
```

### Integrate into your embedded project

1. Copy `include/gnss_parser.h`, `src/*.c` into your build system.
2. Add `-lm` (for `sin`/`cos`/`atan2` in haversine — omit if unused).
3. For no-FPU targets, replace `haversine` with a fixed-point approximation.

---

## API Overview

### 1. Streaming parser (recommended for UART/DMA use)

```c
#include "gnss_parser.h"

static void on_nmea(const char *sentence, void *ctx) {
    nmea_gga_t gga;
    if (nmea_parse_gga(sentence, &gga) == GNSS_OK && gga.valid) {
        printf("Lat: %.6f  Lon: %.6f  Alt: %.1f m  Sats: %d\n",
               gga.latitude.decimal, gga.longitude.decimal,
               gga.altitude_m, gga.num_satellites);
    }
}

static void on_ubx(const ubx_message_t *msg, void *ctx) {
    if (msg->type == UBX_MSG_NAV_PVT) {
        const ubx_nav_pvt_t *pvt = &msg->payload.nav_pvt;
        printf("Fix: %s  Sats: %d  Lat: %.7f  Lon: %.7f\n",
               gnss_fix_type_str(pvt->fixType),
               pvt->numSV,
               pvt->lat / 1e7,
               pvt->lon / 1e7);
    }
}

// In your UART receive callback / DMA complete ISR:
gnss_parser_ctx_t parser;
gnss_parser_init(&parser, on_nmea, on_ubx, NULL);

void UART_RxCpltCallback(uint8_t *buf, uint16_t len) {
    gnss_parser_feed(&parser, buf, len);
}
```

### 2. Direct sentence parsing

```c
nmea_gga_t gga;
gnss_err_t err = nmea_parse_gga("$GPGGA,...*XX", &gga);
if (err == GNSS_OK && gga.valid) { /* use gga */ }

nmea_rmc_t rmc;
nmea_parse_rmc("$GPRMC,...*XX", &rmc);

ubx_message_t msg;
ubx_parse_message(raw_bytes, len, &msg);
```

### 3. Utilities

```c
// Distance between two coordinates (metres)
double dist = gnss_haversine_m(lat1, lon1, lat2, lon2);

// Human-readable fix type
const char *s = gnss_fix_type_str(pvt->fixType);  // "3D Fix"
```

---

## UBX Protocol Notes

### Frame layout

```
┌──────┬──────┬───────┬────┬───────────┬─────────────┬──────┬──────┐
│ 0xB5 │ 0x62 │ Class │ ID │  Len (LE) │   Payload   │ CK_A │ CK_B │
└──────┴──────┴───────┴────┴───────────┴─────────────┴──────┴──────┘
```

- Checksum = Fletcher-8 over `[Class, ID, Len_L, Len_H, Payload...]`
- All multi-byte fields are **little-endian**

### Supported messages

| Class | ID   | Name          | Key fields |
|-------|------|---------------|------------|
| 0x01  | 0x07 | NAV-PVT       | lat/lon (1e-7°), height (mm), fixType, numSV, speed, time |
| 0x01  | 0x03 | NAV-STATUS    | gpsFix, TTFF, uptime |
| 0x05  | 0x01 | ACK-ACK       | acknowledged class+id |
| 0x05  | 0x00 | ACK-NAK       | rejected class+id |

---

## Error Codes

| Code | Meaning |
|------|---------|
| `GNSS_OK` | Success |
| `GNSS_ERR_INVALID` | Null pointer or malformed input |
| `GNSS_ERR_CHECKSUM` | Checksum mismatch — frame corrupted |
| `GNSS_ERR_INCOMPLETE` | Not enough bytes received yet |
| `GNSS_ERR_UNKNOWN_MSG` | Unrecognised class/ID or sentence type |
| `GNSS_ERR_OVERFLOW` | Buffer capacity exceeded |

---

## Porting to Bare Metal

The library has **no OS or heap dependencies**. To port:

1. The only `stdlib` headers used are `<stdint.h>`, `<stdbool.h>`, `<string.h>`,
   `<stdlib.h>` (for `strtod`/`strtol`), and `<math.h>` (haversine only).
2. On targets without `strtod` (e.g. ultra-minimal libc), replace with a
   fixed-point coordinate parser.
3. The `gnss_parser_ctx_t` struct (~700 bytes) can live in BSS/stack — no `malloc`.
4. `gnss_parser_feed` is re-entrant per context instance; protect with a mutex
   if called from both ISR and task context.

---

## Testing

```
── NMEA Checksum ──   ✓ 1/1
── NMEA GGA       ──  ✓ 14/14
── NMEA RMC       ──  ✓ 8/8
── NMEA GSA       ──  ✓ 6/6
── NMEA VTG       ──  ✓ 5/5
── UBX NAV-PVT    ──  ✓ 11/11
── UBX ACK        ──  ✓ 8/8
── UBX bad CK     ──  ✓ 2/2
── Streaming SM   ──  ✓ 7/7
── Haversine      ──  ✓ 2/2
── Fix strings    ──  ✓ 4/4
                     ══════
                     66/66
```

---

## Author

Built as a demonstration of embedded GNSS protocol parsing for u-blox hardware.
