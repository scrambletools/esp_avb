# AVB Talker, Listener, and Bridge Component

This component provides an AVB (Audio Video Bridging) stack with talker,
listener, and L2 bridge support. It runs as a wired endpoint, a Wi-Fi
endpoint, or as an Ethernet ↔ Wi-Fi bridge depending on per-port
configuration.

For a demo on how to use it, see the endpoint app on Github
(scrambletools/ESP-AVB-Endpoint). For the bridge counterpart, see
scrambletools/ESP-AVB-Bridge.

The source lives at <https://github.com/scrambletools/esp_avb>.
The component is published in the ESP Component Registry
(<https://components.espressif.com>) under the **scrambletools**
namespace, so depend on it as:

```yaml
dependencies:
  scrambletools/esp_avb: "*"
```

Old github.com/scrambletoolsllc/... links redirect to the
scrambletools repos.

## Terminology

Where IEEE 1722.1 / 802.1AS would use "grandmaster clock" or "GM",
this component uses **BTC** (best timetransmitter clock). Where the
spec uses "BMCA" (best master clock algorithm), this uses **BTCA**.
The wire format and algorithm are unchanged — only the identifiers
and prose differ. The renaming applies to both code identifiers
(e.g. the `gptp_btc_id` field in `aecp_get_avb_info_rsp_s`) and
human-readable comments / docs.

## Roles

The role is selected automatically from the per-port configuration owned
by `esp_ptp` (see *Configuration* below). No separate AVB role switch is
needed.

- **endpoint_wired** — talker/listener on a wired EMAC port. Default.
  Full ATDECC entity, codec/I2S enabled.
- **endpoint_wireless** — talker/listener on a Wi-Fi STA port. Same
  AVB stack on the wireless data plane via `esp_wifi_internal_tx` /
  `esp_wifi_internal_reg_rxcb`. Codec is optional; disable it via
  `avb_config_s::codec_disabled = true` for boards without audio
  hardware.
- **bridge (experimental)** — transparent L2 AVB bridge between an Ethernet port and a
  Wi-Fi SoftAP. Forwards AVTP control (ADP/AECP/ACMP/MAAP), MSRP, MVRP,
  and VLAN-tagged stream frames in both directions. No codec, no own
  ATDECC entity. Includes FQTSS Credit-Based Shaper and MSRP admission
  control on the egress port (Class B-only on Wi-Fi per the v1 plan;
  Class A reservations propagating to Wi-Fi are rejected with
  `insufficient_bandwidth_for_traffic_class`).

## Current features

Streaming:

- 1 Talker, 1 Listener (run simultaneously on endpoints)
- Up to 8 channels per stream
- AAF PCM (24-bit samples in 32-bit containers, Milan base formats) and
  AM824; 44.1, 48, 88.2, 96 and 192 kHz
- CRF media clock talker and listener

Bridging:

- Bidirectional L2 forwarder between Ethernet and Wi-Fi (SoftAP) ports
- Bridge classifier per IEEE 802.1Q (PTP/MSRP/MVRP terminate locally;
  AVTP control + VLAN-tagged streams forward)
- FQTSS shaping decisions and MSRP admission control on Wi-Fi egress
- Beacon Vendor IE publish for gPTP `FollowUpInformation` transport
  (provided by `esp_ptp`)

Mediums:

- Ethernet (EMAC) — endpoint and bridge
- Wi-Fi (native, e.g. ESP32-C6) — endpoint
- Wi-Fi (`esp_wifi_remote` over ESP-Hosted SDIO/SPI) — bridge

## Configuration

Per-port topology (number of ports, medium, type, Wi-Fi role) is
configured in `esp_ptp`'s Kconfig under *PTP Daemon Configuration*. The
matching `ESP_AVB_*` symbols (`NUM_PORTS`, `PORT{0,1}_MEDIUM_*`,
`TIME_SOURCE_*`, `ROLE_*`) are derived automatically — they have no
prompt and cannot drift from the `esp_ptp` selection.

Topology examples:

| Build | esp_ptp settings | Resulting AVB role |
| --- | --- | --- |
| Wired endpoint | `NUM_PORTS=1`, `PORT0_MEDIUM=ethernet`, `PORT0_TYPE=primary` | `endpoint_wired` |
| Wireless endpoint | `NUM_PORTS=1`, `PORT0_MEDIUM=wifi`, `PORT0_TYPE=primary`, `PORT0_WIFI_ROLE=sta` | `endpoint_wireless` |
| Bridge | `NUM_PORTS=2`, `PORT0_MEDIUM=ethernet` + `TYPE=bridged`, `PORT1_MEDIUM=wifi_cp` + `TYPE=bridged` + `WIFI_ROLE=ap` | `bridge` |

AVB-specific knobs (stream VLAN, Class A/B PCP, MILAN compliance, codec
selection) remain in this component's own Kconfig.

### Codec clock plans

Clocking is a codec property. Each codec in `avbcodec.c` carries a
clock plan: for every sample rate it is offered at, the MCLK
frequencies it can run that rate from, in preference order, plus any
codec-private register row that must be programmed for the pair. The
rates a codec advertises come from its plan (a rate with no row is not
offered), the `allowed_sample_rates` policy in `avbconfig.h` narrows
them, and a rate change keeps the MCLK already running whenever the new
rate lists it, so the APLL and its converged trim survive the change.

The plan is checked once per boot against the codec's MCLK ceiling and
MCLK/BCLK floor, the I2S integer-divider rule and the APLL range; a
candidate that fails is logged and skipped. When NVS restores a stream
binding, the check is deferred 30 s so its console output cannot delay
superfast connect.

| Codec | Rate | MCLK candidates (fs multiple) |
| --- | --- | --- |
| ES8389 | 44.1 kHz | 11.2896 MHz (256) |
| ES8389 | 48 kHz | 24.576 MHz (512), 18.432 MHz (384) |
| ES8389 | 88.2 kHz | 11.2896 MHz (128) |
| ES8389 | 96 kHz | 24.576 MHz (256) |
| ES8389 | 192 kHz | 24.576 MHz (128) |
| ES8311, ES8388 | 48 kHz | 18.432 MHz (384), 12.288 MHz (256) |
| ES8311, ES8388 | 96 kHz | 24.576 MHz (256) |

## Tested with

Targets:

- ESP32-P4 — wired endpoint, bridge host
- ESP32-C6 — wireless endpoint (native Wi-Fi), bridge Wi-Fi coprocessor
  (over ESP-Hosted SDIO)

Boards:

- Waveshare ESP32-P4-ETH — wired endpoint, with the hat-trik v4 audio
  hat (ES8389) or the board's onboard ES8311
- Waveshare ESP32-P4-WIFI6-POE-ETH — bridge (P4 host + onboard C6 Wi-Fi)
- Waveshare ESP32-C6-LCD-1.69 — wireless endpoint (onboard ES8311)

Network peers:

- MOTU AVB Switch
- PreSonus AVB switch
- MOTU 8D AVB endpoint
- Apple Mac mini M1 and M4
- Sonnet Thunderbolt AVB Adapter
- Hive AVB Controller
- Linux AVB endpoints and controllers (avb-endpointd)

## Milan 1.3 test results

No Avnu certification has been run, and no conformance claim is made.
These are bench results against the Avnu Milan Specification v1.3,
with the first esp_avb release that carries each fix. Dates are 2026.
gPTP timing results (IEEE 802.1AS Annex B) are in the esp_ptp README.

### How it was tested

- **Devices under test:** Waveshare ESP32-P4-ETH wired endpoints with
  the hat-trik v4 (ES8389) or the onboard ES8311, and Waveshare
  ESP32-C6-LCD-1.69 Wi-Fi endpoints behind a Waveshare
  ESP32-P4-WIFI6-POE-ETH bridge. The bridge has no ATDECC entity.
- **Network:** a MOTU AVB Switch (gPTP BTC and SRP bridge) and,
  separately, a PreSonus AVB switch.
- **Controllers:**
  - Hive;
  - macOS (Mac mini M4, macOS 26.5);
  - avb-endpointd on Linux;
  - a scripted AECP/ACMP controller (`tools/atdecc_controller.py`),
    including fake controllers for the lock and notification limits.
- **Peers:** the MOTU 8D, the Mac as talker and listener, and Linux
  AVB endpoints.
- **Wire checks:** captures on tapped links for ACMP, AECP and MSRP.
- **Presentation time:** checked on an oscilloscope. A talker test
  pulse played through a listener's analog output is compared with the
  gPTP second pulse of another board.

### ATDECC (AECP/AEM)

| Item | Result | Release |
| --- | --- | --- |
| ACQUIRE_ENTITY answers NOT_SUPPORTED (§5.4.2.1); LOCK_ENTITY owner-only, 60 s expiry | verified | 2.36.0 |
| Unsolicited notifications (§5.4.2.21): 16 registrations, TIME_LIMITED expiry at 300 s, NO_RESOURCES for a 17th, CONTROLLER_AVAILABLE every 30–60 s | verified | 2.36.0–2.37.1 |
| SET_STREAM_FORMAT: STREAM_IS_RUNNING while bound or streaming (§5.4.2.7), BAD_ARGUMENTS for an unsupported format, the response carries the current format | verified | 2.41.0 |
| GET/SET_SAMPLING_RATE, START/STOP_STREAMING | verified | 2.43.0–2.44.0 |
| SET_CONFIGURATION, GET_DYNAMIC_INFO | implemented; SET_CONFIGURATION's success path not exercised | 2.44.0 |
| STREAM_ID_VALID on stream inputs; GET_COUNTERS | verified | 2.35.3, 2.20.0 |
| Localized string references | verified with Hive | — |
| MVU GET_MILAN_INFO | implemented; see gaps | — |

### Connection management (ACMP)

| Item | Result | Release |
| --- | --- | --- |
| Fast connect: PROBE_TX within 0–1 s of discovering the talker (§5.5.3.6.9) | verified: +399 ms at boot, +840 ms after the talker came back (11-13) | 2.45.0 |
| PROBE_TX / CONNECT_TX response: connection_count 0, REGISTERING_FAILED clear (Table 5.48) | verified on the wire | 2.45.0 |
| DISCONNECT_TX reply-only and zeroed (Table 5.50), never stops a stream that other listeners hold | verified on the wire | 2.45.0 |
| A probe for the other SR class gets the stream's current class, not TALKER_EXCLUSIVE (§5.5.4.1) | verified | 2.34.1 |
| GET_TX_STATE connection_count 0 (Table 5.52); GET_TX_CONNECTION NOT_SUPPORTED (§5.5.4.4) | implemented | — |
| LISTENER_EXCLUSIVE; connect and disconnect with the MOTU 8D | verified | — |

### Stream formats, rates and CRF

| Item | Result |
| --- | --- |
| AAF base formats (§6.2): bit depth 32, 6/12/24 samples per frame at 48/96/192 kHz | verified, the Mac as listener at all three rates |
| CRF (Table 7.1): 48 kHz base, timestamp interval 96, 1 timestamp per PDU | verified; matches the Mac's CRF on the wire |
| Format, rate, presentation offset and binding kept across a reboot (§5.3.5.1, §5.3.7.1, §5.3.7.6, §5.3.8.2) | verified |
| Format sweep against Linux AVB endpoints: AAF 32-bit, 24-in-32 and AM824 at 44.1–192 kHz | 23 of 24 wired pairs passed (10-27); 44.1/88.2 kHz 10 of 10 |
| 44.1 kHz CRF (§7.3.2) | not tested |

### SRP (MSRP/MVRP)

- LeaveAll is handled per attribute type (5 Mac connect cycles with no
  sequence gaps), and pure-LeaveAll vectors are parsed. With these, the
  MOTU AVB Switch reserves and prunes correctly.
- A malformed EndMark that upset the PreSonus switch is fixed.
- A talker whose reservation grows moves to a new stream ID, so no
  bridge applies the old, smaller reservation. It plays again within
  1–3 s (2.41.0), and the ID is kept across a reboot (unreleased,
  esp_avb main).
- A listener's declaration change is sent as New (2.37.1).
- A talker's SR class is kept across a reboot (2.39.0).

### Media clock and presentation time

- Strict presentation time is the default. On the oscilloscope, a
  hat-trik v4 (ES8389) listener's analog output lands −7 / +9 / −1 µs
  from the gPTP second at 48 / 96 / 192 kHz (10-29). The onboard
  ES8311 boards have not been measured.
- Switching the media clock source to a CRF stream ran clean for
  12 minutes.

### Interoperability notes

- **Hive:** enumeration and localized names verified. Hive flags a
  non-zero GET_TX_STATE count as a Milan error; esp_avb reports 0 in
  Milan mode. Over Wi-Fi, READ_DESCRIPTOR can exceed Hive's 250 ms
  timeout.
- **macOS (Mac mini M4, macOS 26.5):** connections made by an external
  controller work. As a controller the Mac uses LOCK (never ACQUIRE),
  registers TIME_LIMITED notifications and answers
  CONTROLLER_AVAILABLE. Two issues are on the Mac's side: clocking a
  Mac input from CRF stops coreaudiod, and the Mac as a 192 kHz
  listener plays at a third of the speed.
- **MOTU 8D:** it decodes every input as AM824, its INPUT_STREAM
  clocking is unstable, and it refuses SET_SAMPLING_RATE.
- **PreSonus AVB switch:** full AVB with four endpoints.
- **MOTU AVB Switch:** gPTP BTC and SRP bridge for the bench. On its
  own it loses about 40 frames an hour (07-22) and passes about 92 % of
  a stream sent at MaxIntervalFrames 2 (10-11).

### Known gaps

- macOS sends MVU GET_MILAN_INFO with zero target and controller IDs,
  and esp_avb's AECP target filter drops it. The Mac then treats the
  endpoints as plain IEEE 1722.1 entities.
- After AskingFailed the talker waits 10 s before acting; Milan has no
  such hold-down.
- An ACMP connection row stays behind when a listener disappears without
  DISCONNECT_TX.
- Listener recovery does not yet use ADP available_index.
- Wi-Fi endpoints run Class B only. A station listener sets CLASS_B,
  which Table 5.25 lists as unused; Milan peers are not affected.

## ATDECC support

Note that ATDECC support is limited to functionality that is required
for an AVB talker and listener. No effort has been made to implement
functionality that is needed to operate as a controller. The bridge role
is transparent at L2 and exposes no ATDECC entity of its own.

- Discovery
    Entity Available
- Enumeration and Control
  - Descriptors
    - Entity
    - Configuration
    - Audio Unit
    - Stream Input
    - Stream Output
    - AVB Interface
    - Clock Source
    - Memory Object
    - Locale
    - Strings
    - Stream Port Input
    - Stream Port Output
    - Audio Cluster
    - Audio Map
    - Control
    - Clock Domain
  - Commands
    - Acquire Entity
    - Lock Entity
    - Entity Available
    - Controller Available
    - Read Descriptor
    - Get Configuration
    - Set Stream Format
    - Get Stream Info
    - Register Unsolicited Notification
    - Deregister Unsolicited Notification
    - Get Counters
    - Get Memory Object Length
    - Start Operation / Abort Operation (memory objects: erase the crash
      dump; upload, store and store-and-reboot the firmware image)
  - Address Access (read the memory objects; write the firmware image)
- Connection Management
  - Connect TX
  - Disconnect TX
  - Get TX State
  - Connect RX
  - Disconnect RX
  - Get RX State
  - Get TX Connection
- Milan Vendor Unique (MVU, Milan v1.3 §5)
  - Get Milan Info
  - Get System Unique ID
  - Get Media Clock Reference Info
  - Set Media Clock Reference Info
  - Bind Stream
  - Unbind Stream
  - Get Stream Input Info Ex
- AVB Community Vendor Unique (CVU)
  - Tunnels SRP attributes over AECP Vendor Unique on networks without
    native MSRP. Used when the PTP daemon falls back from gPTP to
    standard PTP (`ESP_AVB_AVB_LITE_COMPLIANT`).
  - Talker Advertise
  - Talker Failed
  - Listener (Asking Failed / Ready / Ready Failed / Ignore)

### Memory objects

Besides the entity logo, an endpoint serves:

- the console log (`LOG_OBJECT`, `ESP_AVB_LOG_OBJECT`): a 128-byte ASCII
  header (`AVBLOG1 total= drop= boot= ring= ptp= ms= rst= resets=`), then
  a RAM ring of the newest log lines, read with Address Access;
- the last crash's core dump (`CRASH_DUMP`, `ESP_AVB_CRASH_OBJECT`, with
  ESP-IDF core dumps to flash): the raw image, cleared with Start
  Operation ERASE;
- the firmware image (`FIRMWARE_IMAGE`, `ESP_AVB_FIRMWARE_OBJECT`, with
  two OTA slots): updated the IEEE 1722.1-2021 Annex D way. New firmware
  keeps itself once gPTP is valid and otherwise rolls back.

### Notes for controllers

- A talker refuses Set Stream Format while its stream is still sent
  (STREAM_IS_RUNNING). After a Disconnect, its listeners' MSRP
  declarations take a few seconds to go: wait for "Stream out N stopped"
  in its log (2 to 5 s) before changing its format.
- A format change that grows a talker's reservation moves its stream to
  a new stream ID (`ESP_AVB_TALKER_NEW_STREAM_ID_ON_GROW`), so no bridge
  applies the old, smaller reservation to it. The ID is kept across a
  reboot.
- A firmware upload while a stream runs gets STREAM_IS_RUNNING unless
  the controller holds the entity (Lock or Acquire); the endpoint then
  stops its streams for the upload.
