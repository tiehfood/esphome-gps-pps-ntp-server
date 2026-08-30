
# ESPHome GPS/PPS NTP Server
## Overview

Stratum-1 NTP server running on ESPHome, good enough for home lab. A u-blox LEA-M8T GPS module provides coarse UTC time via NMEA sentences, while its PPS output disciplines the system clock to microsecond accuracy.

**Hardware**: Waveshare ESP32-S3-ETH + WD22UGRC board (LEA-M8T-0-10)  
**Accuracy**: ~10 µs worst-case between PPS corrections, self-correcting every second

## Images

| ![Image01](docs/images/image01.jpg) | ![Image02](docs/images/image02.jpg) |
|:-----------------------------------:|:-----------------------------------:|
| ![Image03](docs/images/image03.jpg) | ![Image04](docs/images/image04.jpg) |
Printables on [MakerWorld](https://makerworld.com/de/models/2774039-gps-box-clock-esphome-gps-pps-ntp-server) :)


## Components

### `gps_pps_time` - PPS-Disciplined Time Source

ESPHome time platform that combines NMEA time with PPS.

**How it works:**

1. NMEA provides the UTC epoch (which second it is)
2. PPS rising edge marks the exact boundary of each UTC second
3. On each PPS pulse, the system clock is measured and corrected

**Startup sequence:**

1. GPS module acquires fix, begins outputting NMEA + PPS
2. First valid NMEA sentence sets coarse system time (`settimeofday`)
3. First PPS after valid NMEA hard-syncs the clock to the correct second boundary
4. Subsequent PPS pulses apply continuous drift correction via `adjtime()`

### `ntp_server` - NTP Server

Serves NTPv4 over UDP port 123 using BSD sockets (non-blocking). Reads system time via `gettimeofday()`, which reflects the PPS-disciplined clock.

**NTP response fields:**

| Field | Synchronized | Unsynchronized |
|-------|-------------|----------------|
| LI (Leap Indicator) | 0 (no warning) | 3 (clock not synchronized) |
| Stratum | 1 (primary reference) | 16 (unsynchronized) |
| Reference ID | `GPS` | `GPS` |
| Precision | 2^-20 (~1 µs) | 2^-20 |

The server transitions to unsynchronized when PPS is lost for >10 seconds (GPS unplugged, cable fault, etc.).

## Clock Correction

### ESP-IDF (primary platform)

Uses `adjtime()` to gradually slew the system clock without jumps. This is ideal for NTP serving because clients never see time discontinuities.

- Each PPS pulse measures the drift (system clock vs expected PPS time)
- `adjtime(-(drift + mean))` overcorrects by the running mean drift rate, centering the sawtooth error around zero. Without overcorrection, the clock drifts from 0 to +D µs between PPS pulses. With overcorrection, it swings from -D/2 to +D/2, halving the peak error NTP clients see.
- A spike filter (threshold: ~500 µs) prevents measurement noise from corrupting the clock

### Other platforms (fallback)

Uses `settimeofday()` on every PPS with a drift pre-compensation EMA. The compensation value predicts the crystal drift so the clock is set slightly behind, and the crystal drift brings it to zero by the next PPS.

## Drift Measurement

At each PPS pulse, the system reconstructs what the system clock read at the exact PPS edge:

```
system_time_at_pps = gettimeofday() - isr_to_loop_delay - isr_latency
drift = system_time_at_pps - expected_time
```

Two delays are compensated:

1. **ISR-to-loop delay**: Time between the PPS ISR capturing `micros()` and `loop()` processing it. Measured directly via `micros() - last_pps_micros_`.

2. **ISR latency**: If the UART ISR delays the PPS ISR, the `micros()` timestamp in the ISR is late. Detected by measuring the interval between consecutive PPS ISRs — a deviation from 1,000,000 µs reveals the delay. Only compensated for plausible values (500 µs to 10 ms).

### Drift sensor

The drift sensor shows the **raw clock error** at each PPS edge — the actual offset between the system clock and GPS time. This is the worst-case error an NTP client could see at that moment.

With the adjtime overcorrection, the sensor reads ~D/2 in steady state (e.g., ~5 µs for a 10 ppm crystal) instead of the full drift rate. Spikes (e.g., from ISR contention) appear as the true measured error, making anomalies immediately visible.

## ISR Latency Detection

The PPS GPIO interrupt can be delayed by higher-priority interrupts (typically the UART ISR processing GPS NMEA data). When this happens, `micros()` in the PPS ISR captures a late timestamp.

Detection method: the interval between consecutive PPS ISR timestamps should be exactly 1,000,000 µs. A positive deviation of 500-10,000 µs indicates ISR latency on the current pulse. This deviation is subtracted from the drift measurement to recover the true PPS edge time.

## Race Condition Protection

A guard prevents NMEA from overwriting the epoch counter between PPS ISR and loop processing:

```cpp
if (!pps_synced_ && !pps_flag_) {
    last_gps_epoch_ = val.timestamp;
}
```

Without this, if the main loop is delayed >50 ms (e.g., during boot, Ethernet init), NMEA for second T could arrive after the PPS ISR for second T but before `loop()` processes it. This would set `last_gps_epoch_ = T`, causing `corrected_epoch = T + 1` — one second ahead, permanently.

## Failure Recovery

### PPS gap (e.g., GPS briefly disconnected)

If drift exceeds 500 ms, the epoch counter has diverged from reality. Instead of hard-syncing with the stale epoch (which would set the clock to the wrong time), the component resets sync state:

- `pps_synced_ = false` — allows NMEA to re-establish the correct epoch
- `gps_time_valid_ = false` — ignores PPS until fresh NMEA arrives
- Recovery takes 1-2 seconds once GPS signal returns

### GPS fully unplugged

Without PPS pulses, the crystal free-runs at ~10 ppm (~36 ms/hour drift). After the 10-second PPS timeout, the NTP server sets LI=3 and stratum=16, signaling clients that the time source is unsynchronized.

## Sensors

| Sensor | Type | Description |
|--------|------|-------------|
| `satellites` | Numeric | Total satellites in view (sum of GPS + GLONASS + Galileo GSV counts) |
| `gps_satellites` | Numeric | GPS satellites in view (from $GPGSV field 3) |
| `glonass_satellites` | Numeric | GLONASS satellites in view (from $GLGSV field 3) |
| `galileo_satellites` | Numeric | Galileo satellites in view (from $GAGSV field 3) |
| `clock_offset` | Numeric (µs) | Filtered clock error as seen by NTP clients. Only updated on clean measurements; on spikes, estimates drift from crystal rate. Steady state ~5 µs with overcorrection. |
| `pps_drift` | Numeric (µs) | Raw unfiltered drift measurement at each PPS edge. Shows ISR contention and measurement anomalies for diagnostics. Spikes (e.g. -900 µs) indicate measurement artifacts, not actual clock error. |
| `gps_time` | Text | Current PPS-disciplined UTC time (YYYY-MM-DD HH:MM:SS) |

Per-constellation satellite counts use `TinyGPSCustom` to parse GSV sentence field 3 (total satellites in view for that constellation). Requires NMEA 4.10 and GNSS-specific GSV talker IDs for Galileo (`$GAGSV`).

## u-blox Configuration

Compensates for signal propagation delay in the antenna cable (~5 ns per meter of coax).

```yaml
UBX-CFG-TP5
```
Active GNSS constellations. Enable GPS, SBAS and Galileo.

```yaml
UBX-CFG-GNSS
```

Protocol Version 4.10 for Galileo satellite IDs in NMEA output. Without this, Galileo satellites are reported under `$GPGSV` instead of `$GAGSV`, and per-constellation counting doesn't work. Also enables extended SV numbering and GNSS-specific GSV talker IDs.

```yaml
UBX-CFG-NMEA
```
GPS module UART baud rate.

```yaml
UBX-CFG-PRT
```

## Temperature Effects

The system clock is derived from the ESP32's 40 MHz crystal oscillator (via the SYSTIMER peripheral, independent of CPU frequency). Crystal frequency follows a parabolic temperature curve:

- Turnover point near ~25 °C
- Deviation: ~0.035 ppm/°C²
- At 50 °C: ~22 ppm drift rate (vs ~10 ppm at 25 °C)

The PPS discipline loop automatically tracks temperature-induced drift changes. Higher temperatures increase the measured drift value, but `adjtime()` compensates every second. No manual temperature compensation is needed.

## Accuracy Budget

| Source | Magnitude | Handling |
|--------|-----------|----------|
| Crystal drift (10 ppm) | ~10 µs/s | Corrected by `adjtime()` every PPS |
| ISR latency (UART contention) | 0-10 ms (rare) | Detected and compensated; spikes filtered |
| ISR-to-loop delay | ~100-500 µs | Measured and subtracted from drift |
| `micros()`/`gettimeofday()` sampling gap | ~1-3 µs | Constant bias, absorbed by adjtime steady state |
| Antenna cable delay | ~5 ns/m | Compensated via UBX-CFG-TP5 |
| GPS PPS accuracy (LEA-M8T) | ~30 ns RMS | Reference accuracy, not compensatable |

**Resulting NTP server accuracy: ~10 µs** (bounded by crystal drift between PPS corrections). Does not accumulate over time.
