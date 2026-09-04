# A Stratum-1 NTP Server That Was Right and Still Wrong

**Hardware**: Waveshare ESP32-S3-ETH + WD22UGRC (u-blox LEA-M8T) · **Framework**: ESPHome on ESP-IDF

| ![Image01](docs/images/image01.jpg) | ![Image02](docs/images/image02.jpg) |
|:-----------------------------------:|:-----------------------------------:|
| ![Image03](docs/images/image03.jpg) | ![Image04](docs/images/image04.jpg) |

> –\> Printables on [MakerWorld](https://makerworld.com/de/models/2774039-gps-box-clock-esphome-gps-pps-ntp-server) :)

The GPS module hands the ESP32 two things: the current time as text, and a wire that
twitches once a second, exactly on the second. Discipline the system clock to that wire and
you have a very good clock. Ours held **5 µs** against GPS, and kept holding it for a
hundred days.

Then I measured what clients actually received. Every answer was **3.3 milliseconds** wrong.

## Why that is a correctness problem

An NTP exchange has four timestamps. The client notes when it sent (T1) and when the reply
came back (T4). The server fills in when the request arrived (T2) and when the reply left
(T3).

The server's timestamps only have to be **honest, not fast**. A server that takes a full
second to reply is fine, as long as T2 and T3 truthfully say when things happened — the
client subtracts the server's own processing time and it cancels.

What does not cancel is stamping at the wrong moment. If T2 claims a packet arrived later
than it did, that error lands in the client's answer at half its size, and the round-trip
measurement looks perfectly normal, so nothing downstream can detect or filter it.

3.3 ms was not slowness. It was a silent, systematic lie.

## Out of the queue

The original code stamped T2 when ESPHome's main loop reached the component, and that loop
sleeps out a 16 ms interval. Moving serving into its own FreeRTOS task, pinned to the second
core, removed it:

```
                        slope    device error
16 ms loop queueing     0.503    —
own task                0.007    +659 µs
```

That slope became the main instrument for everything after. Plot the client's offset against
round-trip delay: if the error is packets waiting in a queue, offset rises with delay at
exactly **half** the slope — a one-sided delay landing in a two-sided formula. 0.5 is a
queue. Near zero means what remains is a fixed internal latency.

## Stamp it where it arrives

659 µs is the distance between a frame landing in the Ethernet chip and our task waking to
look at it. That has to be stamped earlier, not scheduled away.

ESP-IDF lets you hook the Ethernet driver's receive path, before the frame reaches the
network stack. That took the device error from **659 µs to 97 µs**. But the driver doesn't
know a frame arrived either — it finds out by reading a register over SPI, then spends time
clocking the payload across that same bus. Since we supply the SPI driver, we can watch
every transaction and stamp the first one of a receive burst.

| stamp moved to | how much earlier | client offset |
|---|---|---|
| the "how much is waiting?" register read | 287 µs | −107 … −222 µs |
| the first SPI transaction of the burst | 115 µs | −53 ± 19 µs |

## The half nobody had checked

T2 got all the attention, because T2 was the half we had instrumented. T3 was a
*prediction*: current time plus an estimate of how long sending takes.

The estimate learned from how long `sendto()` took to return — which is wrong, because the
packet is already on its way before that call comes back. It was systematically too long, so
**every reply carried a T3 that was too late.**

We could see it, because we watch the SPI bus: there is an exact instant where the driver
tells the chip to transmit.

```
before:  drifts from +63 µs to −430 µs, settling near −180 µs   ← a bias, not noise
after:   sits on zero, ±23 µs
```

That measurement needs no network at all — it is the device checking its own homework.
Fixing it was worth about **−154 µs** to clients, the largest single win of the project,
hiding in the half nobody had looked at.

## Not fooling yourself

Twice I nearly shipped a wrong conclusion, and both times the mistake was in the
measurement, not the code.

**Run-to-run comparison is worthless here.** Two measurements five minutes apart differed by
143 µs from network conditions alone. Anything smaller is invisible unless you flip the
change back and forth and compare paired blocks. Every change above has a runtime switch for
exactly that.

**The obvious statistical fix is a trap.** The natural move is to regress offset on delay to
control for conditions. But moving T2 also changes the measured delay — delay is *computed
from* T2. Controlling for something downstream of your change quietly removes part of the
effect. One change first measured as a clean null (−12 ± 19 µs) was really worth −53 ± 19 µs
once the known shift was added back before fitting.

The same discipline applies to tuning. Sweeping the T3 filter's smoothing rate against its
own error showed the existing setting was already the best of four — the weakness was not the
rate but a single 1251 µs outlier that shifted the estimate by 156 µs and took eight samples
to decay. Clamping how far one sample may move the estimate fixed the real problem.

## The limit

The W5500 pulls an interrupt line low the instant a frame lands, and the ESP32-S3 can
timestamp a pin change in hardware. Its pin matrix even lets a second peripheral watch a pin
that already has an interrupt on it, so this can be measured without disturbing anything.

The gap between the chip's own "frame is here" signal and our stamp is **21 µs**. Wired up
behind a switch and measured properly, using it as T2 is worth **+1.4 ± 14 µs** — a wash.
So it stays off, but the measurement is the point: it is the first hardware-referenced bound
on how honest T2 is. Two other checks agree, neither depending on a well-behaved network:

- Sweeping request size from 48 to 1400 bytes, offset grows **90 ns per byte**. Pure wire
  time across two store-and-forward hops predicts 80. Any SPI payload read still leaking
  into T2 would make it 240 or worse.
- The T3 prediction error sits on zero.

The timestamps are as honest as this hardware can make them.

## Where it ended up

| | before | after |
|---|---|---|
| Client-visible offset | +3,286 µs | **+63 … +105 µs** |
| Round-trip delay | 8,502 µs | ~580 µs |
| Clock vs GPS | 5 µs | 5 µs |

The clock never changed. It was always the good part.

What remains is roughly 60–100 µs, most of it probably not ours: path asymmetry, the small
difference between how long a packet takes to travel out versus back. NTP cannot separate
that from a real clock offset, and neither can we while the measuring machine sits a router
hop away. Going below 100 µs honestly needs a client on the same segment — equipment, not
code.

The server also stopped rebooting along the way. Twenty-six unexplained restarts over 149
days turned out to be one function call in an interrupt handler that takes a lock.

---

## Practical notes

**Components** — `gps_pps_time` disciplines the clock from the PPS edge (the interrupt only
reads `micros()`; wall-clock time is reconstructed in the main loop) and self-corrects
whole-second errors against NMEA. `ntp_server` serves RFC 5905 from a dedicated core-1 task,
stamps T2 in the Ethernet driver's receive path, pre-corrects T3 from the measured transmit
trigger, keeps client ARP entries warm, and refuses to answer at all when unsynchronised
rather than serve a wrong time. `ethernet` is a fork of ESPHome's that supplies the custom
SPI driver the timing work depends on.

**Accuracy budget**

| Source | Magnitude | Handling |
|---|---|---|
| Crystal drift | ~5 µs/s measured | `adjtime()` every PPS |
| PPS interrupt latency | 0–10 ms, rare | detected, compensated, spikes filtered |
| Clock read granularity | 1–3 µs | constant, absorbed by the loop |
| Antenna cable delay | ~5 ns/m | compensated via UBX-CFG-TP5 |
| GPS PPS itself | ~30 ns RMS | the reference |

Clock accuracy is **~5 µs**, measured over 100 days, and does not accumulate. The advertised
root dispersion is 250 µs, built from those measured terms rather than a guess: clock 50,
T2 stamping 21, T3 prediction 23, timestamp granularity 31.

**Diagnostics** — `t3_error` should sit on zero, `int_lead` near 20 µs, `arp_primes` silent.
Each is a regression that announces itself.

**Building**

```bash
source virtenv/bin/activate
esphome compile ntp_server.yaml
```

Temperature is handled implicitly rather than modelled: the loop measures whatever the drift
currently is and corrects it every second. Over seven days the residual is a steady 5 µs/s
while the board has only ever lived between 43 and 46 °C — far too narrow a window to fit a
temperature coefficient to, so any figure quoted for 0 °C or 70 °C would be borrowed from a
datasheet rather than measured here.
