

A lightweight, portable **GNSS parser** written in **C11** for u-blox receivers. Parses both **NMEA 0183** sentences and **UBX binary** protocol frames with zero heap allocation — designed to run on bare-metal embedded targets like STM32, nRF52, and ESP32.

**What it does:**
- Parses GGA, RMC, GSA, GSV, VTG sentences from any GNSS receiver
- Decodes UBX binary frames: NAV-PVT, NAV-STATUS, ACK-ACK, ACK-NAK
- Streaming byte-feed state machine — safe to call directly from a UART ISR or DMA callback
- Validates every frame with NMEA XOR and UBX Fletcher-8 checksums
- No malloc, no OS dependencies, no external libraries
- 66 unit tests, all passing

**Tested with:**
- u-blox NEO-M8N
- u-blox ZED-F9P (RTK)
- u-blox MAX-M10S

**Build & run:**
```bash
make
# Results: 66/66 passed (all good!)
```

---

