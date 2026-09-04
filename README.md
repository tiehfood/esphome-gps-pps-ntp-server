# A Stratum-1 NTP Server That Was Right and Still Wrong

**Hardware**: Waveshare ESP32-S3-ETH + WD22UGRC (u-blox LEA-M8T) · **Framework**: ESPHome on ESP-IDF

| ![Image01](docs/images/image01.jpg) | ![Image02](docs/images/image02.jpg) |
|:-----------------------------------:|:-----------------------------------:|
| ![Image03](docs/images/image03.jpg) | ![Image04](docs/images/image04.jpg) |

The GPS module hands the ESP32 two things: the current time as text, and a wire that
twitches once a second, exactly on the second. Discipline the system clock to that wire and
you have a very good clock. Ours held **5 µs** against GPS, and kept holding it for a
hundred days.

Then I measured what clients actually received. Every answer was **3.3 milliseconds** wrong.

The clock was fine. The clock had never been the problem. Everything between the packet
arriving and the packet leaving was the problem, and none of it showed up in any sensor we
had.

## Where the time actually goes

An NTP exchange has four timestamps. The client notes when it sent (T1) and when the reply
came back (T4). The server fills in when the request arrived (T2) and when the reply left
(T3). From those four numbers the client works out how far its clock is off.

The subtle part: **the server's timestamps only have to be honest, not fast.** A server that
takes a full second to reply is fine, as long as T2 and T3 truthfully say when things
happened. The client subtracts the server's own processing time and it cancels out.

What does *not* cancel out is stamping at the wrong moment. If T2 says a packet arrived
later than it really did, that error goes straight into the client's answer at half its
size — and the round-trip measurement looks completely normal, so no client can detect it
or filter it out. It is a silent, systematic lie.

That is what a 3.3 ms error means. Not "slow". Wrong.

## Getting out of the queue

The original code stamped T2 when ESPHome's main loop got round to the component. That loop
sleeps out a 16 ms interval, so a packet could sit for anything from zero to 16 ms before
anyone looked at it.

The fix is unglamorous — move serving into its own FreeRTOS task, pinned to the second core
so nothing else can delay it — and the payoff is not:

```
                        slope    device error
16 ms loop queueing     0.503    —
own task                0.007    +659 µs
```

That slope is worth explaining, because it became the main measuring instrument for
everything after. Plot the client's computed offset against the round-trip delay. If our
error comes from packets waiting in a queue, offset rises with delay at exactly **half** the
slope — that is the arithmetic of a one-sided delay landing in a two-sided formula. A slope
of 0.5 is a queue. A slope near zero means the offset has stopped tracking the network,
which means what is left is a fixed internal latency.

Three milliseconds gone. And now a flat 659 µs that no amount of scheduling would touch.

## Stamp it where it arrives

659 µs is the distance between a frame landing in the Ethernet chip and our task waking up
to look at it. You cannot schedule that away. You have to stamp earlier.

The Ethernet chip here is a W5500 on a SPI bus, and ESP-IDF lets you hook the driver's
receive path — the moment it hands a frame up to the network stack, before any of it
reaches us. Stamping T2 there took the device error from **659 µs to 97 µs**.

Then it got interesting. The driver doesn't magically know a frame arrived either; it finds
out by reading a register over SPI, and then spends time clocking the payload across that
same bus before it hands anything up. Since we already supply the SPI driver, we can watch
every transaction go past and stamp the moment the *first* one of a receive burst starts.

Two steps, both measured before being believed:

| stamp moved to | how much earlier | client offset |
|---|---|---|
| the "how much is waiting?" register read | 287 µs | −107 … −222 µs |
| the first SPI transaction of the burst | 115 µs | −53 ± 19 µs |

## The half nobody had checked

T2 got all the attention for a day, because T2 was the half we had instrumented. T3 —
when the reply leaves — was a *prediction*: take the current time, add an estimate of how
long sending takes, write that down.

The estimate learned from how long the `sendto()` call took to return. That felt reasonable
and it is wrong, because the packet is already on its way before that call comes back. The
estimate was systematically too long, so **every single reply carried a T3 that was too
late.**

We could measure it, because we watch the SPI bus: there is an exact instant where the
driver tells the chip "transmit now". Comparing prediction against that instant:

```
before:  drifts from +63 µs to −430 µs, settling around −180 µs   ← a bias, not noise
after:   sits on zero, ±23 µs
```

That is the measurement I like best in this whole project, because **it needs no network at
all.** It is the device checking its own homework. Fixing it was worth about **−154 µs** to
clients — a bigger single win than anything found in T2 that day, hiding in the half nobody
had looked at.

## How you know you did not fool yourself

Twice I nearly shipped a wrong conclusion, and both times the mistake was in the
measurement rather than the code.

**Run-to-run comparison is worthless here.** Two measurements five minutes apart differed by
143 µs purely from network conditions. Anything smaller than that is invisible unless you
flip the change back and forth and compare paired blocks. Every improvement above has a
runtime switch for exactly that reason.

**And the obvious statistical fix is a trap.** The natural move is to regress offset on
delay to control for network conditions. But moving T2 earlier *also changes the measured
delay* — delay is computed from T2. It is downstream of the change, so controlling for it
quietly removes part of the effect you are trying to measure. One change first measured as
"no effect at all" (−12 ± 19 µs) turned out to be worth −53 ± 19 µs once the known shift was
added back before fitting. The code had been right the whole time.

## Knowing when to stop

The tempting next step was a hardware trick: the W5500 has an interrupt line that goes low
the instant a frame lands, and the ESP32-S3 can timestamp a pin change in hardware, with no
software in the path. Better still, the chip's pin matrix lets a second peripheral watch a
pin that already has an interrupt on it, so we could measure without disturbing anything.

It worked. And it said the gap between the chip's own "frame is here" signal and our stamp
is **21 µs**.

Twenty-one microseconds is not worth chasing. So we did not — but the measurement is the
point, because it is the first *hardware-referenced* bound on how honest T2 is. Two other
checks agree:

- Sweeping the request size from 48 to 1400 bytes, offset grows by **90 ns per byte**. Pure
  wire time across two store-and-forward hops predicts 80. If any of the SPI payload read
  were still leaking into T2, it would be 240 or worse. It isn't there.
- The T3 prediction error sits on zero.

Three independent checks, none of which depend on the network being well-behaved, all
saying the same thing: **the timestamps are as honest as this hardware can make them.**

## Where it ended up

| | before | after |
|---|---|---|
| Client-visible offset | +3,286 µs | **+63 … +105 µs** |
| Round-trip delay | 8,502 µs | ~580 µs |
| Clock vs GPS | 5 µs | 5 µs |

The clock never changed. It was always the good part.

What remains is roughly 60–100 µs, and most of it is probably not ours. It is path
asymmetry — the small difference between how long a packet takes to travel out versus back.
NTP fundamentally cannot separate that from a real clock offset, and neither can we, because
our measuring machine sits a router hop away. Measuring below 100 µs honestly needs a client
on the same network segment. That is the next piece of equipment, not the next piece of code.

Along the way the server also stopped rebooting — 26 unexplained restarts over 149 days
turned out to be one function call in an interrupt handler that takes a lock, which is a
different story — and it now advertises an error bound of 1 ms instead of the 5 ms it
inherited, because we finally knew enough to say something honest.

## Two things worth more than the next microsecond

**A supercapacitor on the GPS backup pin.** There is no battery, so every power cut is a
cold start: almanac gone, ephemeris gone, and with a weak signal that took **80 minutes**
before the server would answer at all. A supercap turns that into seconds.

**A better antenna position.** The module sees about 26 dB-Hz where 40–50 is normal. That is
what makes cold starts take an hour instead of a minute.

Neither is code. Both matter more, on any real day, than everything above.

---

## Practical notes

**Components**

- `gps_pps_time` — PPS-disciplined time source. Captures the PPS edge in an interrupt (only
  `micros()`, nothing that takes a lock), reconstructs wall-clock time in the main loop, and
  corrects with `adjtime()` every second. Detects and self-corrects whole-second errors by
  comparing the clock against NMEA absolute time.
- `ntp_server` — RFC 5905 server on a dedicated core-1 task. Stamps T2 in the Ethernet
  driver's receive path, pre-corrects T3 from the measured transmit trigger, keeps client
  ARP entries warm, and refuses to answer at all when unsynchronised rather than serve a
  wrong time.
- `ethernet` — a fork of ESPHome's, supplying a custom SPI driver so timing can be observed,
  and sharing one SPI handle rather than adding a second device.

**Accuracy budget**

| Source | Magnitude | Handling |
|---|---|---|
| Crystal drift (10 ppm) | ~10 µs/s | `adjtime()` every PPS |
| PPS interrupt latency | 0–10 ms, rare | detected, compensated, spikes filtered |
| Clock read granularity | 1–3 µs | constant, absorbed by the loop |
| Antenna cable delay | ~5 ns/m | compensated via UBX-CFG-TP5 |
| GPS PPS itself | ~30 ns RMS | the reference; not compensatable |

Clock accuracy is **~5 µs**, bounded by crystal drift between corrections, measured over
100 days. It does not accumulate.

**Diagnostics worth watching** — `t3_error` should sit on zero, `int_lead` around 20 µs,
`arp_primes` should stay silent. Each is a regression that announces itself.

**Building**

```bash
source virtenv/bin/activate
esphome compile ntp_server.yaml
```

Temperature is handled implicitly: the crystal follows a parabola with its turnover near
25 °C, drifting roughly 22 ppm at 50 °C against 10 ppm at 25 °C. The discipline loop simply
measures whatever the drift currently is and corrects it, so no explicit temperature
compensation is needed.
