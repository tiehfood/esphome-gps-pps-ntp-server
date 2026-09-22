# ESPHome GPS/PPS Stratum-1 NTP Server

**Hardware**: Waveshare ESP32-S3-ETH + WD22UGRC (u-blox LEA-M8T) · **Framework**: ESPHome on ESP-IDF

| ![Image01](docs/images/image01.jpg) | ![Image02](docs/images/image02.jpg) |
|:-----------------------------------:|:-----------------------------------:|
| ![Image03](docs/images/image03.jpg) | ![Image04](docs/images/image04.jpg) |

> –\> Printables on [MakerWorld](https://makerworld.com/de/models/2774039-gps-box-clock-esphome-gps-pps-ntp-server) :)

A GPS module supplies UTC over NMEA and a 1 PPS pulse on a GPIO. The PPS edge disciplines
the ESP32 system clock; an NTP server on the device serves that clock to the LAN.

Clock accuracy is **~5 µs** against GPS, stable over 100 days. Served time — what a client
actually receives — measures **+5…+7 µs from GPS** against a hardware-timestamping reference
on the same network segment. Part of that figure is the reference itself; see Measuring
against an independent reference.

Earlier versions of this file quoted 60–100 µs and said the limit was the lack of a good
enough client. That was true until one was built.

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
| Rewrite T3 into the chip's transmit buffer just before it sends | per-sample σ 7.14 → 2.83 µs |
| Resolve ARP before stamping T3, refuse if it cannot | removes a ~1.5 ms early T3 on 24 % of windows |
| Refuse requests whose arrival interrupt is missing or stale | replies off by >1 ms: 6.9 → 0.5 per 1000 |
| Shorten the chip's interrupt assert wait (with fresh-burst admission) | replies served late >150 µs: 0.64 → 0.19 % |
| Stamp T2 from the chip's own interrupt edge | **−5.60 µs** measured against GPS |
| Reference T2 to the frame's first bit, not its last | **−2.60 µs** measured against GPS |
| Correct both stamps by the chip's measured transmit/receive latency | **+19.94 µs**, predicted +19.8 |

The last three were measured against GPS rather than inferred. Each was predicted before the
code existed; the third's constants came from one-way delay measurements on a separate board
and predicted the server-side result to 0.14 µs.

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

**The client's own stack is part of the measurement.** Taking T4 in userspace instead of
from the kernel's `SO_TIMESTAMPING` added 41 µs of receive-path delay, and half of that
landed directly in the reported offset. Correcting both client timestamps tightened the
spread from 27 µs to 14.5 µs.

**Tuning found the existing value was already correct.** Sweeping the T3 filter's smoothing
rate gave a settled error RMS of 9.85 µs at the default, against 11.12, 11.91 and 15.27 at
other rates. The weakness was not the rate but a single 1251 µs outlier that shifted the
estimate by 156 µs.

## Bounds on the remaining error

Three checks, none of which depend on the network behaving:

- **Hardware reference.** The W5500 asserts an interrupt when a frame lands, and MCPWM
  capture timestamps that edge in hardware — on the same pin the Ethernet driver already
  uses, since the pin matrix allows both. The gap to our own stamp is 13–21 µs. Using the
  hardware edge as T2 first measured +1.4 ± 14 µs — a confidence interval 27 µs wide around
  a 10 µs effect, so it was left off. Re-measured against a proper reference it is
  **−5.60 µs**, and it now ships. The instrument was the problem, not the idea.
- **Packet-size sweep.** From 48 to 1400 byte requests, offset grows 90 ns/byte. Two
  store-and-forward hops of wire time predict 80 ns/byte; SPI payload time leaking into T2
  would give 240 or more.
- **T3 prediction error** sits on zero.

What remains is unattributed, and honestly so. Moving the server onto the client's own
network segment cut round-trip delay from 674 µs to 129 µs and left the measured offset
unchanged — so the residual is not mainly path asymmetry, as this section previously claimed.

## Measuring against an independent reference

The limit was the reference: measuring a server to within tens of microseconds needs a
client whose own clock is better than that, and neither available client qualified — one is
disciplined by this very server (circular), the other tracks an internet stratum-1 with ±4 ms
root distance. Both were reporting their own error as ours.

So a reference was built: an ESP32-P4 on the same switch, timestamping packets in its
Ethernet MAC at the start-of-frame delimiter, **fed the same GPS pulse as the server**. Two
endpoints sharing one physical pulse is what makes absolute measurement possible at all.

What it found, in order:

- Served time was **+14 µs** from GPS, not the ~60–100 µs the routed clients suggested.
- Correcting T2 to the chip's interrupt edge, then to the frame's first bit, moved it to
  **−14 µs** — *further* from GPS. The early stamp had been masking an opposite error.
- The chip's own latencies, measured from one-way delays: it transmits **57 µs** after we
  stamp T3 and signals reception **17 µs** after a frame ends. Correcting both landed the
  server at **+5.3 µs**, within 0.14 µs of prediction.
- The remainder is **not the server's**. Its clock arithmetic, checked against its own GPS
  edge with no network involved, is accurate to **+1.00 µs** over 206 samples with zero
  drift. What is left is asymmetry between the two boards' physical layers, which no
  firmware change can remove.

**One measurement error is worth recording**, because every absolute figure above was wrong
by ~20 µs until it was found. The reference reports its clock error once per second while
drifting 34 µs/s, so each sample was up to a second stale; the analysis used it as if it were
current. It surfaced because the answer changed by +4.6 µs when only the sampling interval
changed — a measurement that depends on how you sample it is measuring the instrument. The
same bug had been masquerading as an unexplained 2–3 µs wander for four days.

## Accuracy budget

| Source | Magnitude | Handling |
|---|---|---|
| Crystal drift | ~5 µs/s measured | `adjtime()` every PPS |
| PPS interrupt latency | 0–10 ms, rare | detected, compensated, spikes filtered |
| Clock read granularity | 1–3 µs | constant, absorbed by the loop |
| Antenna cable delay | ~5 ns/m | compensated via UBX-CFG-TP5 |
| GPS PPS itself | ~30 ns RMS | the reference |

Advertised root dispersion is 100 µs on the PPS-anchor path (250 µs on the fallback), plus
10 µs for every second since the last PPS sync. It bounds the error rather than claiming it:
the measured distance to GPS is a few µs.

Temperature is not modelled. The loop measures whatever the drift currently is and corrects
it each second. The residual is a steady 5 µs/s over seven days, but the board has only run
between 43 and 46 °C — too narrow to fit a temperature coefficient, so any figure for 0 °C
or 70 °C would come from a datasheet, not from this device.

## Diagnostics

`t3_error` should sit at zero, `int_lead` near 13–20 µs, `arp_primes` silent,
`anchor_pred_error` within ±2 µs. Each reports its own regression.

`anchor_pred_error` is the useful one if you suspect the clock: it evaluates the previous
PPS anchor's prediction at the next edge and compares it with that edge's exact second, so it
measures the server against GPS with no network and no second device involved.

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
