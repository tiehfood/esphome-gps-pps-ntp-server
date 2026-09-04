# NTP Serving Quality Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Make what LAN clients receive match the quality of the clock. The clock holds ±5 µs against GPS; the server adds +3.29 ms of offset (measured) and advertises a perfect error budget it does not have. Fix the protocol lies first (cheap, low risk), then the latency (staged, measured).

**Architecture:** Two independent problem classes, addressed in order of risk-adjusted value. **Phase 1 — protocol conformance:** four RFC 5905 defects, all in `build_ntp_response_()`, none touching concurrency. **Phase 2 — serving latency:** the NTP receive timestamp is stamped when ESPHome's main loop dispatches the component, not when the packet arrived. Move serving off the shared loop into a dedicated FreeRTOS task. **Phase 3 — W5500 hardware timestamping:** gated behind measurement; only justified if Phase 2 leaves ≫100 µs.

**Tech Stack:** ESPHome 2025.12.7 / ESP-IDF 5.5.1, C++ ESPHome external component, lwIP BSD sockets, W5500 SPI Ethernet, `chrony` on a LAN host for measurement.

**Spec:** This document. Source findings: `.claude/tracker.md` (Precision evaluation, 2026-08-30 / 2026-09-03) plus two research reports summarised inline with citations.

## Global Constraints

- **DO NOT FLASH ANYTHING until the ISR panic fix has ≥21 days of clean uptime.** Device booted `2026-08-30 21:04`; earliest flash `2026-09-20`. Flashing resets the uptime clock and destroys the crash-validation run. This is the hardest constraint here.
- Activate the virtualenv first: `source virtenv/bin/activate` (Python 3.12.14, `esphome==2025.12.7` pinned).
- Verification for every code task: `esphome compile ntp_server.yaml` reports `Successfully compiled program` with zero warnings in our files.
- No Wi-Fi components. W5500 Ethernet only.
- Never call `gettimeofday()` or any libc/system call from an ISR — see `.claude/rules/firmware.md`.
- Never "calibrate out" the +5 µs steady-state reading. It is `D/2`, the designed sawtooth peak. See `.claude/tracker.md`.
- `loop_interval_` (16 ms) is **not** exposed in ESPHome YAML. There is no configuration lever for loop latency.
- **`SO_TIMESTAMP` does not exist in lwIP** — not behind a Kconfig, not in Espressif's fork, not upstream. Verified in `esp-lwip/src/api/sockets.c`: the complete `SO_*` list has no timestamping option, and `recvmsg()` only ever emits `IP_PKTINFO`. Do not go looking for it.

## RESULTS — measured 2026-09-04 (wired Pi, `/root/ntp/ntp_probe.py`)

Phases 1 and 2 plus step 9 are **flashed and verified end to end**.

| | pre-flash | after | note |
|---|---|---|---|
| offset | +3,286 µs (sd 2,405) | **~+670 µs** (min +607) | 5x better, scatter 26x tighter |
| delay | 8,502 µs (sd 4,749) | **~1,700 µs** (min 1,213) | = network RTT; ping min 1,099 |
| root dispersion | 0.000000 s | 0.004990 s | no longer claims a perfect clock |
| reference time | = transmit | = last PPS sync | `time_id` fix confirmed on the wire |
| precision | −20 hardcoded | −15 measured | `gettimeofday()` costs ~16–30 µs here |

**`q/2` confirmed by measurement**, settling two wrong revisions of this plan:
regression of offset on delay gives **slope 0.503, r = 0.993**; delay spread 15,362 µs
(= the 16 ms `loop_interval_`) against offset spread 7,666 µs vs 7,681 predicted.

**Residual ~670 µs** is our NTP task at priority 7 waking behind lwIP (18) and the eth
driver task. That is what the input-path hook attacks — now implemented and compiled
(`58194d8`), **not flashed**: it sits in the path of every received frame, so a mistake
takes the network down and recovery is USB-only.

## A one-second epoch bug was found, and it dwarfs all of the above

While measuring, the device was caught serving **+1.002 s** — confirmed internally
(it labelled the edge at 06:03:20.998 UTC as `06:03:22`) while reporting `drift: 5 us`.
Intermittent, months old, and **structurally invisible**: `clock_offset` measures system
time against the same epoch counter that is wrong, and the NMEA/PPS cross-check needs
`|diff| > 2 s`. It only surfaced because an external reference was used for the first time.

Now instrumented and corrected — see `.claude/tracker.md`. The lesson for this plan:
**every microsecond here was chased with an instrument that could not see a whole second.**

---

## Step summary and expected improvement

Baseline today (wired Pi, 30 samples, 2026-09-03): offset **+3,286 µs** (sd 2,405), delay **8,502 µs** (sd 4,749). Clock vs GPS: **±5 µs**.
Confidence key: **[M-us]** measured on this device · **[M-ref]** measured by a comparable ESP32+W5500+GPS project · **[D]** derived from RFC algebra or datasheet · **[E]** estimated.

| # | Step | What it changes | Expected effect | Cumulative state after | Conf. | Risk |
|---|------|-----------------|-----------------|------------------------|-------|------|
| — | *baseline* | — | — | **+3,286 µs offset** (sd 2,405), delay 8,502 µs | [M-us] | — |
| **Phase 1 — protocol conformance. No accuracy change; fixes four things we currently misreport.** ||||||
| 1 | Echo client NTP version | v3 clients get a v3 reply instead of v4 | 0 µs | +3,286 µs | [D] | none |
| 2 | Reference timestamp = last PPS sync | Clients can see sync age; keeps reftime ≤ rec ≤ xmt | 0 µs | +3,286 µs | [D] | none |
| 3 | Root dispersion ≥1 ms, growing with age | **Stops us being voted a falseticker.** Zero dispersion shrinks our advertised error interval to nothing, so honest peers get rejected in favour of us | 0 µs, but changes client *selection* outcomes | +3,286 µs | [M-ref] | none |
| 4 | Measure clock-read cost, set precision | −20 confirmed or corrected to −18 per §11.1 | negligible | +3,286 µs | [D] | none |
| **Phase 2 — remove the queueing delay. This is the big one.** ||||||
| 5 | Valid baseline (wired, same subnet) | Nothing — makes everything below measurable | 0 µs | — | — | none |
| 6 | Dedicated FreeRTOS task, core 1, prio 7 | T2 stamped on `recvfrom` return instead of when ESPHome's loop dispatches. Removes the full 16 ms `loop_interval_` queueing term **and** the "another component blocked for 50 ms" tail | **−3.3 ms** (removes q/2) | **~tens of µs offset; delay collapses 8,502 → ~2,400 µs** — *the sign may flip: with q gone, T3's SPI residual makes the server read slightly behind* | [M-us] + [M-ref] | **medium** — concurrency |
| **Phase 3 — hardware-adjacent timestamping. Takes it from sub-ms to tens of µs.** ||||||
| 7 | Stamp T2 on the cheap RX-size register read, before clocking the payload over SPI | SPI read duration stops inflating T2 | contributes to the σ drop in step 8 | ~300 µs offset | [M-ref] | low |
| 8 | W5500 `INTn` GPIO ISR stamps T2 + seqlock for the 64-bit value | Arrival captured at the hardware boundary, not after the stack | **jitter σ 26 µs → 2.6 µs** | ~300 µs offset, **~3 µs jitter** | [M-ref] | **high** — ISR + tearing |
| 9 | Pre-correct T3 by EWMA of measured SPI send duration | T3 is currently written ~636 µs before the packet physically leaves; one-sided, so it costs **half** that in offset and inflates delay by the full amount | **−300 µs offset** | **~tens of µs offset, ~3 µs jitter** | [D] from [M-ref] 636 µs | medium |
| 10 | Prime ARP before replying | Cold cache causes a 100–200 ms busy-wait between T2 and T3 | removes rare catastrophic outliers | unchanged typical, no 100 ms spikes | [M-ref] | low |
| 11 | Re-measure, revise `ROOT_DISP_BASE_S` down | Keeps the advertised budget honest once the error shrinks | 0 µs | **~60 µs, path-dominated** | [M-ref] | none |

**Endpoint.** The reference project reached ~60 µs offset with 10–25 µs jitter, verified against an independent PTP grandmaster, and attributed the residual to routing asymmetry rather than the server. On a switched same-segment LAN you may do better. That is roughly **50× better than today**, and the remaining error would finally be dominated by the network rather than by us.

**Where the effort is.** Steps 1–5 are a day. Step 6 is the highest value-per-risk in the whole plan and should be soaked alone. Steps 7–11 are the long tail: together they buy ~300 µs and a 10× jitter reduction, at the cost of ISR work, a seqlock, and W5500 driver interaction.

---

## Measured: the error is `q/2`, and it IS visible to clients

**This section has been wrong twice. This version is measured, not derived.**

Only **T2** is late. The packet waits `q` for ESPHome's loop, we stamp T2, then stamp
T3 and send immediately — so **T3 is correct**; it genuinely is when we transmitted.
That is one-sided asymmetry:

```
θ′ = ½[(T2 + q − T1) + (T3 − T4)] = θ + q/2
δ′ = (T4 − T1) − (T3 − (T2 + q))  = δ + q      <-- delay INFLATES
```

An earlier revision of this plan claimed both stamps shift by `q`, giving error `q`
with the delay unchanged and therefore undetectable. That premise was false.

**Evidence (2026-09-03, wired Pi at 192.168.35.2, 30 samples, `/root/ntp/ntp_probe.py`):**

| | |
|---|---|
| regression of offset on delay | **slope 0.503, r = 0.993** |
| delay spread | 15,362 µs ≈ ESPHome's 16 ms `loop_interval_` |
| offset spread | 7,666 µs — predicted from q/2: 7,681 µs (**0.2% apart**) |
| delay min | 2,369 µs ≈ network RTT (ping min 1.1 ms) → q ≈ 0 |
| delay max | 17,731 µs → q ≈ 15.4 ms |

Slope 0.5 is the one-sided signature; slope 1.0 would have been common-mode. It is 0.503.

**Consequence for severity:** because the delay inflates by the full `q`, clients CAN
see this. chrony and ntpd weight by delay, so they already down-weight our worst
samples. This is a real error but not the silent, unfilterable one previously claimed.

**Superseded measurement:** an earlier +4.656 ms figure came from a Wi-Fi Mac over a
routed path (ping 5/17.7/83.6 ms). It appeared to confirm the `q` model to 6%; that was
coincidence — Wi-Fi asymmetry roughly doubled a 2.2 ms error. Do not cite it.

---

# Phase 1 — Protocol conformance

Four defects, all in `components/ntp_server/ntp_server.cpp`. No concurrency, no new failure modes. Do these first: they are cheap, and one of them (root dispersion) actively harms clients today.

### Task 1: Echo the client's NTP version instead of hardcoding v4

RFC 5905 §14 Figure 31 specifies `x.version <-- r.version`. We hardcode `response[0] = 0x24` (LI=0, VN=4, Mode=4), so an NTPv3 client gets a v4 reply.

**Files:**
- Modify: `components/ntp_server/ntp_server.cpp` — `build_ntp_response_()`

**Interfaces:**
- Consumes: `const uint8_t *request` (already a parameter).
- Produces: nothing new.

- [ ] **Step 1: Replace the hardcoded first byte**

Replace:

```cpp
  bool synced = this->is_time_synchronized_();
  if (synced) {
    // LI=0 (no warning), VN=4, Mode=4 (server)
    response[0] = 0x24;
    // Stratum 1 (primary reference)
    response[1] = 1;
  } else {
    // LI=3 (clock unsynchronized), VN=4, Mode=4 (server)
    response[0] = 0xE4;
    // Stratum 16 (unsynchronized)
    response[1] = 16;
  }
```

with:

```cpp
  // RFC 5905 s14 Figure 31: x.version <-- r.version. Echo the client's version
  // rather than forcing v4 — NTPv3 clients exist and must get a v3 reply.
  const uint8_t client_version = (request[0] >> 3) & 0x07;
  bool synced = this->is_time_synchronized_();
  if (synced) {
    // LI=0 (no warning), VN=<echoed>, Mode=4 (server)
    response[0] = static_cast<uint8_t>((0 << 6) | (client_version << 3) | 4);
    // Stratum 1 (primary reference)
    response[1] = 1;
  } else {
    // LI=3 (clock unsynchronized), VN=<echoed>, Mode=4 (server)
    response[0] = static_cast<uint8_t>((3 << 6) | (client_version << 3) | 4);
    // Stratum 16 (unsynchronized)
    response[1] = 16;
  }
```

- [ ] **Step 2: Verify it compiles**

```bash
source virtenv/bin/activate && esphome compile ntp_server.yaml
```

Expected: `Successfully compiled program`, no warnings in `ntp_server.*`.

- [ ] **Step 3: Commit**

```bash
git add components/ntp_server/ntp_server.cpp
git commit -m "fix(ntp): echo client NTP version instead of hardcoding v4"
```

---

### Task 2: Reference timestamp must be the last clock update, not "now"

RFC 5905 §7.3, verbatim: *"Reference Timestamp: Time when the system clock was last set or corrected."* We write the current time. Clients and monitoring use this to judge staleness — a perpetually-fresh reftime tells them we are perfectly synced even as sync ages. Both chrony (`our_ref_time`) and NTPsec (`sys_reftime = peer->dst` inside `clock_update()`) set it only when the clock is actually disciplined.

This also establishes the value Task 3 needs to grow dispersion from, and preserves the invariant **reftime ≤ rec ≤ xmt**, which chrony explicitly checks.

**Files:**
- Modify: `components/gps_pps_time/gps_pps_time.h` — add a public getter
- Modify: `components/ntp_server/ntp_server.cpp` — `build_ntp_response_()`

**Interfaces:**
- Produces: `time_t GPSPPSTime::get_last_sync_epoch() const` — UTC epoch seconds of the most recent PPS-disciplined correction, or `0` when never synced. Consumed by Task 3.

- [ ] **Step 1: Add the getter to the time component**

In `components/gps_pps_time/gps_pps_time.h`, in the public section next to `is_synchronized()`:

```cpp
  /// UTC epoch second of the most recent PPS-disciplined correction, or 0 if
  /// never synced. Used by the NTP server for the Reference Timestamp field
  /// (RFC 5905 s7.3: "time when the system clock was last set or corrected").
  time_t get_last_sync_epoch() const {
    return this->pps_synced_ ? static_cast<time_t>(this->last_gps_epoch_) : 0;
  }
```

`last_gps_epoch_` is `volatile time_t`; reading it in a getter is fine. Do not make this getter non-const or add locking — Task 6 revisits thread-safety if and only if the task is built.

- [ ] **Step 2: Use it for the reference timestamp**

In `build_ntp_response_()`, replace the reference-timestamp block:

```cpp
  // Reference timestamp (last sync time) = current time
  response[16] = (receive_ts.seconds >> 24) & 0xFF;
  response[17] = (receive_ts.seconds >> 16) & 0xFF;
  response[18] = (receive_ts.seconds >> 8) & 0xFF;
  response[19] = receive_ts.seconds & 0xFF;
  response[20] = (receive_ts.fraction >> 24) & 0xFF;
  response[21] = (receive_ts.fraction >> 16) & 0xFF;
  response[22] = (receive_ts.fraction >> 8) & 0xFF;
  response[23] = receive_ts.fraction & 0xFF;
```

with:

```cpp
  // Reference timestamp: RFC 5905 s7.3 — "time when the system clock was last
  // set or corrected", NOT the current time. This is the epoch of the last PPS
  // correction, which also keeps the invariant reftime <= rec <= xmt that
  // chrony checks. Fraction is 0: PPS corrections land on a second boundary.
  uint32_t ref_seconds = 0;
  if (this->time_source_ != nullptr) {
    time_t last_sync = this->time_source_->get_last_sync_epoch();
    if (last_sync > 0)
      ref_seconds = static_cast<uint32_t>(last_sync) + NTP_UNIX_OFFSET;
  }
  response[16] = (ref_seconds >> 24) & 0xFF;
  response[17] = (ref_seconds >> 16) & 0xFF;
  response[18] = (ref_seconds >> 8) & 0xFF;
  response[19] = ref_seconds & 0xFF;
  // response[20..23] stay zero from the memset (fraction = 0)
```

- [ ] **Step 3: Verify it compiles**

```bash
source virtenv/bin/activate && esphome compile ntp_server.yaml
```

Expected: `Successfully compiled program`. If `get_last_sync_epoch` is reported as not a member, check that `ntp_server.cpp` still includes `esphome/components/gps_pps_time/gps_pps_time.h` (it does today, line 3).

- [ ] **Step 4: Commit**

```bash
git add components/gps_pps_time/gps_pps_time.h components/ntp_server/ntp_server.cpp
git commit -m "fix(ntp): reference timestamp is last PPS sync, not current time"
```

---

### Task 3: Advertise a root dispersion that actually bounds our error

Bytes 8–11 are left zero by the `memset`. Zero claims a perfect clock. This is not a cosmetic problem: RFC 5905 §11.2.1 builds the client's correctness interval as `θ ± λ` where `λ = rootdisp + rootdelay/2`. **A server whose true error is milliseconds but whose advertised λ is ~0 gets voted a falseticker by Marzullo intersection when compared against honest peers** — and is trusted absolutely when it is a client's only source. Zero makes us look *better*, which is exactly the harm.

Neither reference implementation will send zero. NTPsec clamps to `MINDISTANCE .001` (1 ms) in `clock_update()`. chrony's `get_root_dispersion()` returns a base plus a term growing with time since the last update. Public stratum-1 GPS servers advertise ~0.9–2.4 ms.

Root delay (bytes 4–7) stays zero — correct for stratum 1, there is no upstream path.

NTP short format is 16.16 fixed point: 1 LSB = 1/65536 s ≈ 15.259 µs.

**Files:**
- Modify: `components/ntp_server/ntp_server.cpp` — constants and `build_ntp_response_()`

**Interfaces:**
- Consumes: `GPSPPSTime::get_last_sync_epoch()` from Task 2, and `NTPTimestamp receive_ts`.

- [ ] **Step 1: Add the budget constants**

Near `static const int NTP_PACKET_SIZE = 48;`:

```cpp
/// Base root dispersion in seconds — our own error budget, independent of age.
/// MUST be revised down as serving latency improves. Current value reflects the
/// measured +3.29ms server-added offset (2026-09-03) plus margin; after Phase 2
/// re-measure and reduce. Floor of 0.001 matches NTPsec's MINDISTANCE.
static const float ROOT_DISP_BASE_S = 0.005f;
/// Dispersion growth per second since the last PPS correction, from the ~10ppm
/// crystal (README: "free-runs at ~10 ppm").
static const float ROOT_DISP_RATE_S_PER_S = 10.0e-6f;
/// NTP short format: 16.16 fixed point seconds. 1 LSB = 1/65536 s = 15.259us.
static const float NTP_SHORT_SCALE = 65536.0f;
```

- [ ] **Step 2: Compute and write it**

In `build_ntp_response_()`, immediately after the reference-timestamp block from Task 2:

```cpp
  // Root dispersion (RFC 5905 s7.3, NTP short format 16.16 fixed point).
  // Base budget plus growth since the last correction, mirroring chrony's
  // get_root_dispersion(). Zero here would claim a perfect clock and get us
  // voted a falseticker against honest peers (s11.2.1 correctness interval).
  float disp_s = ROOT_DISP_BASE_S;
  if (ref_seconds != 0) {
    uint32_t age_s = receive_ts.seconds - ref_seconds;  // both NTP epoch
    disp_s += static_cast<float>(age_s) * ROOT_DISP_RATE_S_PER_S;
  }
  uint32_t root_disp = static_cast<uint32_t>(disp_s * NTP_SHORT_SCALE);
  response[8] = (root_disp >> 24) & 0xFF;
  response[9] = (root_disp >> 16) & 0xFF;
  response[10] = (root_disp >> 8) & 0xFF;
  response[11] = root_disp & 0xFF;
  // response[4..7] (root delay) stay zero: stratum 1 has no upstream path.
```

`ref_seconds` is the local declared in Task 2 Step 2 — this block must come after it in the same function.

- [ ] **Step 3: Verify it compiles**

```bash
source virtenv/bin/activate && esphome compile ntp_server.yaml
```

- [ ] **Step 4: Verify on the wire after flashing**

From a LAN host:

```bash
ntpdate -q 192.168.34.131
```

Expected: non-zero `dispersion`, roughly 0.005 s rising slowly. `chronyc sourcestats` against it should no longer show an exact zero.

- [ ] **Step 5: Commit**

```bash
git add components/ntp_server/ntp_server.cpp
git commit -m "fix(ntp): advertise real root dispersion instead of claiming zero"
```

---

### Task 4: Measure the clock-read cost and set precision from it

**This corrects a correction.** The `precision = -20` field was flagged as dishonest earlier in this project, then retracted on the grounds that precision means resolution rather than accuracy. The retraction was right — RFC 5905 §7.3: *"the precision of the system clock, in log2 seconds… can be determined when the service first starts up as the minimum time of several iterations to read the system clock."* (RFC Errata 2476 also corrects the RFC's own worked example: −20, not −18, is ≈1 µs.)

But §11.1 requires **the larger of resolution and clock-read time**: *"The precision is defined as the larger of the resolution and time to read the clock, in log2 units."* Our resolution is 1 µs. If `gettimeofday()` **costs** more than 1 µs on this hardware — and it takes a mutex, so it may well — then −20 understates and the spec-correct value is larger. chrony measures this at startup (`measure_clock_read_delay()`); do not guess.

**Files:**
- Modify: `components/ntp_server/ntp_server.cpp` — `setup()` and `build_ntp_response_()`
- Modify: `components/ntp_server/ntp_server.h` — one member

**Interfaces:**
- Produces: `int8_t NTPServer::precision_` — measured at setup, written to byte 3.

- [ ] **Step 1: Add the member**

In `components/ntp_server/ntp_server.h`, protected section:

```cpp
  /// log2(seconds) of the larger of clock resolution and clock-read cost,
  /// measured once at setup per RFC 5905 s11.1. Defaults to -20 (1us).
  int8_t precision_{-20};
```

- [ ] **Step 2: Measure it at startup**

In the `#ifdef USE_ESP_IDF` `NTPServer::setup()`, before the final log line:

```cpp
  // RFC 5905 s11.1: precision is the LARGER of clock resolution and the time to
  // read the clock. Measure the read cost the way chrony does — the smallest
  // non-zero delta over repeated back-to-back reads.
  {
    uint32_t best_us = UINT32_MAX;
    for (int i = 0; i < 100; i++) {
      struct timeval a, b;
      gettimeofday(&a, nullptr);
      gettimeofday(&b, nullptr);
      int32_t d = static_cast<int32_t>((b.tv_sec - a.tv_sec) * 1000000 + (b.tv_usec - a.tv_usec));
      if (d > 0 && static_cast<uint32_t>(d) < best_us)
        best_us = static_cast<uint32_t>(d);
    }
    if (best_us == UINT32_MAX)
      best_us = 1;  // reads faster than the 1us quantum: resolution dominates
    int8_t p = -20;
    while (p < 0 && (1.0f / (1u << -p)) < (best_us / 1000000.0f))
      p++;
    this->precision_ = p;
    ESP_LOGI(TAG, "Clock read cost %ums, advertising precision %d", best_us, this->precision_);
  }
```

- [ ] **Step 3: Use the measured value**

Replace:

```cpp
  // Precision: ~1 microsecond = 2^-20 seconds
  response[3] = static_cast<uint8_t>(-20 & 0xFF);
```

with:

```cpp
  // Precision: measured at setup (RFC 5905 s11.1 — larger of resolution and
  // clock-read cost). NOT our accuracy; that lives in root dispersion.
  response[3] = static_cast<uint8_t>(this->precision_);
```

- [ ] **Step 4: Verify it compiles**

```bash
source virtenv/bin/activate && esphome compile ntp_server.yaml
```

- [ ] **Step 5: Verify the logged value after flashing**

Check the ESPHome log line `Clock read cost Nus, advertising precision P`. If `N` is 1 and `P` is −20, the original value was right all along. If `N` is 4, expect `P` = −18.

- [ ] **Step 6: Commit**

```bash
git add components/ntp_server/ntp_server.h components/ntp_server/ntp_server.cpp
git commit -m "feat(ntp): measure clock read cost and advertise precision per RFC 5905 s11.1"
```

---

# Phase 2 — Serving latency

### Task 5: Establish a valid baseline

The measurement that started this went from `192.168.33.235` to `192.168.34.131` — **different subnets, routed**, ping `6.5 / 19.9 / 75.1 ms`. A sub-millisecond effect is not observable over that path.

**Files:** none (measurement only)

**Interfaces:**
- Produces: `docs/measurements/baseline-YYYY-MM-DD.txt`.

- [ ] **Step 1: Get a wired host on the ESP32's subnet**

Must be on `192.168.34.0/24`, wired, not Wi-Fi:

```bash
ping -c 20 -q 192.168.34.131
```

Expected: `avg` under 1 ms, `stddev` under 0.5 ms. If not, you are still routed or on Wi-Fi. **Stop and fix the path** — a Wi-Fi ESP32 NTP server has been measured at 31.6 ms jitter and classified a falseticker by chrony. You cannot out-engineer a bad path.

- [ ] **Step 2: Capture sntp and chrony baselines**

```bash
mkdir -p docs/measurements
for i in $(seq 1 30); do sntp 192.168.34.131 | tail -1; sleep 2; done \
  | tee docs/measurements/baseline-$(date +%F).txt
chronyd -Q -t 30 'server 192.168.34.131 iburst minpoll 2 maxpoll 2' \
  | tee -a docs/measurements/baseline-$(date +%F).txt
```

- [ ] **Step 3: Record the prediction**

Append to the file the pre-flash numbers to beat. Measured 2026-09-03 from a wired Pi: offset **+3,286 µs** (sd 2,405), delay **8,502 µs** (sd 4,749), delay spread 15,362 µs ≈ the 16 ms `loop_interval_`. Client error is **q/2** (regression slope 0.503, r=0.993 — see the measured section above), so removing the queueing should take offset to tens of µs and collapse delay to the network RTT (~2,400 µs).

---

### Task 6: Move serving into a dedicated FreeRTOS task

`HighFrequencyLoopRequester` was considered and **rejected**: it shortens the poll interval but does not remove the poll, and every other component's blocking time still lands inside our T2→T3 gap. Our own `loop_time` max of 20–54 ms shows that tail exists; a comparable project measured `gps->loop()` blocking **up to 100 ms** immediately before the NTP poll. None of the seven ESPHome components using `HighFrequencyLoopRequester` are network components.

Spawning a task is precedented in ESPHome core: `esp32_camera`, `i2s_audio`, `openthread`, `zigbee`, `mqtt_backend_esp32`, `usb_host` all call `xTaskCreate*`.

**Mandatory design constraints:**

- **Pin to core 1.** `CONFIG_ESP_MAIN_TASK_AFFINITY_CPU0=y` — ESPHome's loop is on core 0. A core-1 task runs concurrently and can never preempt `apply_pps_correction_()` between its `gettimeofday()` and `micros()` reads. Preemption there would inflate `elapsed_since_edge_us` and inject a negative drift spike, undoing the 4× precision gain from the ISR fix.
- **Priority 7, below lwIP's 18** (`CONFIG_LWIP_TCPIP_TASK_PRIO=18`). Espressif's guidance is that tasks doing TCP/IP work run below the TCP/IP task. Note `ESP32TimeServer` uses 20 (above lwIP), contrary to that guidance; the reference W5500 project uses 7. Follow 7.
- **Stack 4096 bytes.** RAM is at 5.9%.
- **No ESPHome API calls from the task** — no `ESP_LOGD`, no `publish_state`. ESPHome's logger pushes to the API and is not task-safe.
- **`gettimeofday()` from a task is safe.** In task context `lock_acquire_generic` calls `xSemaphoreTake(portMAX_DELAY)` and blocks; the `abort()` path is ISR-only. Hold time is microseconds.
- **Blocking `recvfrom` replaces the non-blocking socket.** Remove the `O_NONBLOCK` `fcntl` from `setup()`, or use `select()` with a timeout so the task can be stopped.

- [ ] **Step 1: Write a design note before any code**

Given the concurrency risk and that this project already had a panic in the adjacent subsystem, do not improvise. Cover: task lifetime and shutdown; socket ownership between `setup()` and the task; behaviour if the task dies; what happens to `is_time_synchronized_()` reads across threads (`GPSPPSTime::pps_synced_` and `last_pps_millis_` at `gps_pps_time.h:75,93` are plain members — benign on Xtensa but a formal race; mark them `volatile`). Then write a task-level plan for it and execute that.

- [ ] **Step 2: Soak it alone**

Do not flash this together with Phase 1 or Phase 3. If panics resume you must be able to attribute them.

---

### Task 7: Re-measure and gate Phase 3

- [ ] **Step 1: Repeat Task 5 verbatim** into `docs/measurements/after-task-$(date +%F).txt`.

- [ ] **Step 2: Decide**

| Measured server-added error | Action |
|---|---|
| < 100 µs | **Stop.** Revise `ROOT_DISP_BASE_S` down to match, and you are done. |
| 100 µs – 1 ms | Judgement call. Phase 3 buys roughly 10×, at real cost. |
| > 1 ms | The model is wrong. Re-investigate before writing code. |

---

# Phase 3 — W5500 hardware-adjacent timestamping (CONDITIONAL)

**Do not start without a Task 7 measurement justifying it.** Reference project measurements, ESP32 + W5500 + GPS:

| Change | Effect |
|---|---|
| RX stamp in the UDP read path | σ = 26 µs |
| RX stamp moved into the W5500 `INTn` ISR | **σ = 2.6 µs** |
| T3 pre-corrected by EWMA of SPI send duration (~636 µs measured at 20 MHz) | ~1.4 ms per request |
| ARP primed before replying | removes a 100–200 ms cold-cache busy-wait |
| End result | 1.88 ms RTT → sub-ms; chrony `^-` → `^*`; independent PTP cross-check ~60 µs offset, 10–25 µs jitter |

- [ ] **Step 1: Wire W5500 `INTn` to a GPIO and stamp T2 in the ISR**, waking the task with `vTaskNotifyGiveFromISR()`. This is the only sub-100 µs receive path available — `SO_TIMESTAMP` does not exist in lwIP.
- [ ] **Step 2: Guard the 64-bit capture with a seqlock.** A 64-bit store is two words on Xtensa; without it the value tears when the low word wraps (every 71.6 min) and T2 lands thousands of seconds away. `.claude/rules/firmware.md` already warns about this.
- [ ] **Step 3: Stamp T2 on the cheap RX-size-register read, before clocking the payload over SPI**, so the SPI read duration does not inflate T2.
- [ ] **Step 4: Pre-correct T3 by an EWMA of measured send duration.** Measure yours; the reference measured ~636 µs at 20 MHz SPI. Note 20 MHz is the proven ceiling on GPIO-matrix pins — reads corrupt silently above it.
- [ ] **Step 5: Prime ARP for known clients.**

---

## Self-Review

**1. Spec coverage.** Every finding maps to a task: hardcoded version → Task 1; reference timestamp → Task 2; zero root dispersion → Task 3; precision semantics → Task 4; invalid measurement path → Task 5; loop queueing → Task 6; decision gate → Task 7; W5500 timestamping, T3 SPI correction, ARP → Phase 3. `HighFrequencyLoopRequester` is explicitly rejected in Task 6 with reasons rather than silently dropped, as is the `SO_TIMESTAMP` dead end (Global Constraints).

**2. Placeholder scan.** No TBDs. Phase 1 tasks carry complete code. Task 6 Step 1 deliberately stops at "write a design note" — a gate, not a placeholder, justified by the concurrency risk. Phase 3 steps are descriptive rather than code because they are gated behind a measurement that may make them unnecessary.

**3. Type consistency.** `get_last_sync_epoch()` returns `time_t`, defined in Task 2 Step 1 and consumed in Task 2 Step 2 and Task 3 Step 2. `ref_seconds` is a `uint32_t` local declared in Task 2 Step 2 and reused in Task 3 Step 2 — **Task 3 must be applied after Task 2, in the same function.** `precision_` is `int8_t`, declared Task 4 Step 1, set Step 2, read Step 3. `NTP_UNIX_OFFSET` and `NTPTimestamp` are pre-existing and unchanged.

**4. Known gaps.**
- The q/2 model is now **measured**, not inferred (slope 0.503, r=0.993, n=30). Task 7's table still treats "> 1 ms remaining" as evidence the model is wrong rather than a cue to keep coding.
- This plan asserted the wrong algebra twice before measurement settled it. Prefer instrumenting over deriving.
- `ROOT_DISP_BASE_S = 0.005f` is calibrated to today's measured error and **must be revised after Phase 2**. If it is left at 5 ms after latency is fixed, we understate our quality instead of overstating it — the opposite error, but still a lie.
- Dropping requests while unsynced (existing behaviour) is a deviation from RFC 5905, which specifies LI=3 / stratum 16. It is defensible against poisoning clients at boot, but clients see unreachability rather than an explicit "not synced". Not changed by this plan; recorded so the choice stays deliberate.
