# ESPHome GPS/PPS Stratum-1 NTP Server

**Hardware**: Waveshare ESP32-S3-ETH + WD22UGRC (u-blox LEA-M8T) · **Framework**: ESPHome on ESP-IDF

| ![Image01](docs/images/image01.jpg) | ![Image02](docs/images/image02.jpg) |
|:-----------------------------------:|:-----------------------------------:|
| ![Image03](docs/images/image03.jpg) | ![Image04](docs/images/image04.jpg) |

> –\> Printables on [MakerWorld](https://makerworld.com/de/models/2774039-gps-box-clock-esphome-gps-pps-ntp-server) :)

A GPS module supplies UTC over NMEA and a 1 PPS pulse on a GPIO. The PPS edge disciplines
the ESP32 system clock; an NTP server on the device serves that clock to the LAN.

Clock accuracy is **~5 µs** against GPS, stable over 100 days. Client-visible accuracy is
**~60–100 µs**, limited by network path asymmetry rather than by the device.

## The gap between clock accuracy and served accuracy

These are not the same number, and the difference was originally 3.3 ms.

An NTP exchange uses four timestamps: the client's send (T1) and receive (T4), the server's
receive (T2) and transmit (T3). The server's timestamps need to be *accurate*, not early —
however long the server takes between T2 and T3 cancels out in the client's calculation.

An error in *when* T2 or T3 is stamped does not cancel. It enters the client's computed
offset at half its size, and the round-trip delay measurement is unaffected, so no client
can detect or filter it. Serving latency is therefore a correctness problem, not a
performance one.

## Changes and measured effect

| Change | Effect |
|---|---|
| Serve from a dedicated FreeRTOS task instead of ESPHome's 16 ms main loop | −3,300 µs |
| Stamp T2 in the Ethernet driver's receive path, before the network stack | 659 → 97 µs device residual |
| Stamp T2 at the driver's `Sn_RX_RSR` read, before the SPI payload transfer | −107 … −222 µs |
| Stamp T2 at the first SPI transaction of the receive burst | −53 ± 19 µs |
| Learn the T3 pre-correction from the measured transmit trigger | −154 ± 20 µs |
| Clamp how far one outlier can move the T3 estimate | removes a 156 µs step lasting 8 samples |
| Prime ARP for recent clients | removes a rare stall between T2 and T3 |
| Root dispersion from a measured budget (5 ms → 250 µs) | honest error bound |
| Remove `gettimeofday()` from the PPS interrupt handler | ends 26 reboots per 149 days |

T3 is worth a note. It is a prediction — current time plus an estimate of the send cost —
and the estimate originally learned from how long `sendto()` took to return. The packet is
already in flight before that call returns, so the estimate ran long and every reply carried
a T3 that was too late by roughly 180 µs. Because the custom SPI driver sees every
transaction, the actual instant the chip is told to transmit is available, and the prediction
error can be measured directly: it moved from a systematic −180 µs to zero ±23 µs.

## Measurement notes

These cost more time than the code did.

**Unpaired before/after comparisons are not evidence on a routed path.** Two runs five
minutes apart differed by 143 µs from network conditions alone. Every change above is
switchable at runtime so it can be measured in interleaved blocks.

**Delay is downstream of the change.** Regressing offset on delay looks like the right way
to control for network conditions, but delay is computed from T2 and T3 — controlling for it
absorbs part of the effect. One change measured as −12 ± 19 µs (indistinguishable from
nothing) was −53 ± 19 µs once the known timestamp shift was added back before fitting.

**Tuning found the existing value was already correct.** Sweeping the T3 filter's smoothing
rate gave a settled error RMS of 9.85 µs at the default, against 11.12, 11.91 and 15.27 at
other rates. The weakness was not the rate but a single 1251 µs outlier that shifted the
estimate by 156 µs.

## Bounds on the remaining error

Three checks, none of which depend on the network behaving:

- **Hardware reference.** The W5500 asserts an interrupt when a frame lands, and MCPWM
  capture timestamps that edge in hardware — on the same pin the Ethernet driver already
  uses, since the pin matrix allows both. The gap to our own stamp is 21 µs. Using the
  hardware edge as T2 measures +1.4 ± 14 µs, i.e. no gain, so it stays off as a diagnostic.
- **Packet-size sweep.** From 48 to 1400 byte requests, offset grows 90 ns/byte. Two
  store-and-forward hops of wire time predict 80 ns/byte; SPI payload time leaking into T2
  would give 240 or more.
- **T3 prediction error** sits on zero.

What remains is roughly 60–100 µs of path asymmetry — the difference between outbound and
return transit time. NTP cannot separate that from a genuine clock offset, and measuring
below it requires a client on the same network segment.

## Accuracy budget

| Source | Magnitude | Handling |
|---|---|---|
| Crystal drift | ~5 µs/s measured | `adjtime()` every PPS |
| PPS interrupt latency | 0–10 ms, rare | detected, compensated, spikes filtered |
| Clock read granularity | 1–3 µs | constant, absorbed by the loop |
| Antenna cable delay | ~5 ns/m | compensated via UBX-CFG-TP5 |
| GPS PPS itself | ~30 ns RMS | the reference |

Advertised root dispersion is 250 µs, summed from measured terms: clock 50, T2 stamping 21,
T3 prediction 23, timestamp granularity 31.

Temperature is not modelled. The loop measures whatever the drift currently is and corrects
it each second. The residual is a steady 5 µs/s over seven days, but the board has only run
between 43 and 46 °C — too narrow to fit a temperature coefficient, so any figure for 0 °C
or 70 °C would come from a datasheet, not from this device.

## Diagnostics

`t3_error` should sit at zero, `int_lead` near 20 µs, `arp_primes` silent. Each reports its
own regression.

## Building

```bash
source virtenv/bin/activate
esphome compile ntp_server.yaml
```

## Components

| Component | Purpose |
|---|---|
| `gps_pps_time` | PPS-disciplined time source. The ISR only reads `micros()`; wall-clock time is reconstructed in the main loop and corrected with `adjtime()` each second. Detects and corrects whole-second errors against NMEA. |
| `ntp_server` | RFC 5905 server on a dedicated core-1 task. Stamps T2 in the Ethernet driver's receive path, pre-corrects T3 from the measured transmit trigger, keeps client ARP entries warm, and drops requests while unsynchronised rather than serve a wrong time. |
| `ethernet` | Fork of ESPHome's Ethernet component. Supplies the custom W5500 SPI driver that the T2/T3 timestamping depends on, and captures the W5500 interrupt edge in hardware via MCPWM. |
