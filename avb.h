/*
 * Copyright 2024-2026 Scramble Tools
 * License: MIT
 *
 * ESP_AVB Component
 *
 * This component provides an implementation of an AVB talker and listener.
 *
 * This file provides the common definitions and types for the AVB protocol.
 */

#ifndef _ESP_AVB_AVB_H_
#define _ESP_AVB_AVB_H_

#include "avbconfig.h"
#include "avbutils.h"
#include "esp_avb.h"
#include "esp_eth_clock.h"
#include <arpa/inet.h>
#include <driver/i2c_master.h>
#include <driver/i2s_std.h>
#include <esp_check.h>
#include <esp_err.h>
#include <esp_eth_driver.h>
#include <esp_heap_caps.h>
#include <esp_log.h>
#include <esp_ptp.h>
#include <esp_vfs_l2tap.h>
#include <fcntl.h>
#include <freertos/FreeRTOS.h>
#include <freertos/semphr.h>
#include <freertos/task.h>
#include <lwip/prot/ethernet.h> // Ethernet headers
#include <net/if.h>
#include <netinet/in.h>
#include <sched.h>
#include <semaphore.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/ioctl.h>
#include <sys/poll.h>
#include <sys/time.h>
#include <sys/timex.h>
#include <time.h>
#include <unistd.h>

/* Number of protocols to use for L2TAP (AVTP, MSRP, MVRP) */
#define AVB_NUM_PROTOCOLS 4

/* Protocol indexes */
#define AVTP 0
#define MSRP 1
#define MVRP 2
#define VLAN 3

/* Maximum number of endpoints to remember */
#define AVB_MAX_NUM_TALKERS 10
#define AVB_MAX_NUM_LISTENERS 10
#define AVB_MAX_NUM_CONTROLLERS 10

/* Maximum number of listeners connected to a talker stream */
#define AVB_MAX_NUM_CONNECTED_LISTENERS 10

/* Unicast fan-out limit is Kconfig-gated on AVB Lite compliance;
 * builds without it still need the tx_da[] mailbox to compile. */
#ifndef CONFIG_ESP_AVB_LITE_UNICAST_FANOUT
#define CONFIG_ESP_AVB_LITE_UNICAST_FANOUT 1
#endif

/* Maximum number of inflight commands */
#define AVB_MAX_NUM_INFLIGHT_COMMANDS 20

/* Maximum number of streams.
 * When audio listener support is enabled: input stream 0 is AAF/61883 audio
 * and input stream 1 is CRF media clock. In talker-only mode: the only input
 * stream is CRF media clock at index 0. Talker endpoints expose output stream
 * 0 as audio and output stream 1 as CRF media clock. The CRF streams are
 * present regardless of CONFIG_ESP_AVB_MILAN; the Kconfig option only toggles
 * whether we advertise Milan compliance in discovery. */
#define AVB_MAX_NUM_INPUT_STREAMS 2
#define AVB_MAX_NUM_OUTPUT_STREAMS 2
#define AVB_DEFAULT_CRF_INPUT_INDEX 1
#define AVB_DEFAULT_CRF_OUTPUT_INDEX 1

/* Periodic message intervals */
#define MSRP_DOMAIN_INTERVAL_MSEC 500
#define MVRP_VLAN_ID_INTERVAL_MSEC 500
#define MSRP_TALKER_IDLE_INTERVAL_MSEC 1000 // when idle
#define MSRP_TALKER_CONN_INTERVAL_MSEC                                         \
  500 // when connected (must be < MRP Leave timer ~1s)
#define MSRP_LISTENER_CONN_INTERVAL_MSEC                                       \
  500 // when connected (must be < MRP Leave timer ~1s)
#define MSRP_LEAVEALL_INTERVAL_MSEC 10000
#define ADP_ENTITY_AVAIL_INTERVAL_MSEC 5000
#define PTP_STATUS_UPDATE_INTERVAL_MSEC 3000
/* Milan §5.5.3.6.x TMR_RETRY is 4s on probe failure. Use the same
 * cadence for fast-connect retries while a saved binding waits for
 * its talker to be discovered / respond. */
#define AVB_FAST_CONNECT_RETRY_MSEC 4000
#define UNSOL_NOTIF_INTERVAL_MSEC 2000
/* Poll interval for the dedicated NVS-save task. The task wakes every
 * N ms, and if state->persist_dirty is set, snapshots and writes to
 * flash. Naturally coalesces bursts of marks in one flash write. */
#define AVB_PERSIST_POLL_MSEC 1000
/* Maximum age a dirty persist snapshot is allowed to wait while audio
 * streams are active before we force-flush it anyway.
 *
 * The snapshot carries only non-critical config (volume, mic gain,
 * descriptor names, clock source) — stream bindings meet the Milan
 * §5.5.3.6.17 durability guarantee through the 32-byte journal
 * appends, which fire immediately at connect/disconnect. So there is
 * no protocol reason to force a ~1.2 KB blob write mid-stream: the
 * flash commit disables cache system-wide for tens of ms, starving
 * the listener drain and the EMAC RX path (a SET_CONTROL volume
 * change used to go silent 1-2 s later exactly this way, and on
 * pre-un-wedge-guard firmware the resulting RX overflow could latch
 * ingress permanently). Defer during streaming; the save lands when
 * streams stop, or after this ceiling for devices that stream
 * indefinitely — one bounded glitch per config change at worst. */
#define AVB_PERSIST_FORCED_FLUSH_MSEC (10 * 60 * 1000)

// Commonly used mac addresses
#define BCAST_MAC_ADDR                                                         \
  (uint8_t[6]){0x91, 0xe0, 0xf0, 0x01, 0x00, 0x00} // adp,acmp
#define MAAP_MCAST_MAC_ADDR                                                    \
  (uint8_t[6]){0x91, 0xe0, 0xf0, 0x00, 0xff, 0x00} // maap
#define SPANTREE_MAC_ADDR                                                      \
  (uint8_t[6]){0x01, 0x80, 0xc2, 0x00, 0x00, 0x21} // mvrp
#define LLDP_MCAST_MAC_ADDR                                                    \
  (uint8_t[6]){0x01, 0x80, 0xc2, 0x00, 0x00, 0x0e} // msrp

/* Empty ID */
#define EMPTY_ID (uint8_t[8]){0, 0, 0, 0, 0, 0, 0, 0}

/* CVU Protocol ID — AVB Lite CVU SRP, per profiles/avb_lite.md §6.
 * MA-S OUI 0x8C1F6436C (bits 47..12), sub-protocol 0x002 (bits 11..0). */
#define CVU_PROTOCOL_ID {0x8C, 0x1F, 0x64, 0x36, 0xC0, 0x02}

/* MVU Protocol ID */
#define MVU_PROTOCOL_ID {0x00, 0x1B, 0xC5, 0x0A, 0xC1, 0x00}

/* IEEE 1722 CRF AudioSample stream format used for the media-clock input.
 * This is also the Milan-compatible 48 kHz CRF profile. 64-bit value;
 * encoded as 8 big-endian bytes in the stream_format field:
 *   subtype=0x04 (CRF), crf_type=1 (AudioSample), timestamp_interval=96,
 *   timestamps_per_pdu=1, pull=0 (1.0x), base_frequency=48000 Hz. */
#define AVB_CRF_AUDIO_SAMPLE_48K_FORMAT_BYTES                                  \
  {0x04, 0x10, 0x60, 0x01, 0x00, 0x00, 0xBB, 0x80}

/* Milan CRF (Milan v1.3 §7.3.1): base_frequency is ALWAYS 48000 Hz,
 * timestamp_interval 96, ONE timestamp per PDU, 500 PDU/s — regardless
 * of the audio sampling rate (96/192 kHz devices lock to the 48 kHz
 * reference at 2x/4x). ATDECC format string 0x041060010000BB80.
 * macOS lists this in its CRF input supported formats at every device
 * rate. Do NOT use the 1722 Table-28 "300 ts/s, 6/PDU" cadence toward
 * macOS: its media-clock servo runs +17 % fast on that flavor no
 * matter how the timestamps are generated (measured 2026-07-08). */
#define AVB_CRF_BASE_FREQ_HZ 48000
#define AVB_CRF_TS_PER_SEC 500
#define AVB_CRF_TS_PER_PDU 1

/* Build the 8-byte Milan CRF AudioSample stream format. The audio rate
 * argument is ignored — Milan CRF is rate-independent (see above). */
static inline void avb_crf_format_for_rate(uint32_t rate, uint8_t out[8]) {
  (void)rate;
  uint16_t interval = (uint16_t)(AVB_CRF_BASE_FREQ_HZ / AVB_CRF_TS_PER_SEC);
  out[0] = 0x04;
  out[1] = 0x10 | ((interval >> 8) & 0x0F);
  out[2] = interval & 0xFF;
  out[3] = AVB_CRF_TS_PER_PDU;
  out[4] = (AVB_CRF_BASE_FREQ_HZ >> 24) & 0x1F;
  out[5] = (AVB_CRF_BASE_FREQ_HZ >> 16) & 0xFF;
  out[6] = (AVB_CRF_BASE_FREQ_HZ >> 8) & 0xFF;
  out[7] = AVB_CRF_BASE_FREQ_HZ & 0xFF;
}

/* Poll interval for checking for incoming frames on L2TAP FDs */
#define AVB_POLL_INTERVAL_MS 1

/* Maximum message length */
#define AVB_MAX_MSG_LEN 600

/* Preamble before the control data */
#define AVTP_CDL_PREAMBLE_LEN 12

/* Preamble before the descriptor */
#define AECP_DESC_PREAMBLE_LEN 28

/* Size of a unique ID */
#define UNIQUE_ID_LEN 8

/* Configuration index settings */
#define DEFAULT_CONFIG_INDEX 0

/* AVTP data limits. Must cover the largest Class A packet we accept:
 * 8ch x 32-bit at 192 kHz = 24 samples x 8 x 4 = 768 B (AAF), 776 B
 * with the AM824 CIP header. 512 silently dropped every packet of a
 * 192 kHz 8ch stream in the listener RX path. */
#define AVTP_STREAM_DATA_PER_MSG 1024 // max stream data bytes per AVTP packet

/* AEM descriptor limits */
#define AEM_MAX_DESC_LEN                                                       \
  504 // descriptor max length (not including descriptor type and index)
#define AEM_MAX_NUM_CONFIGS 1       // only one config supported for now
#define AEM_MAX_NUM_DESC 20         // max number of descriptors in a config
#define AEM_MAX_DESC_COUNT 1        // max count of each descriptor in a config
#define AEM_NUM_CONTROLS 3          // IDENTIFY, Speaker Volume, Mic Gain
#define AEM_MAX_NUM_SAMPLE_RATES 10 // max number of sample rates
#define AEM_MAX_NUM_FORMATS 12      // max number of formats
#define AEM_MAX_NUM_MAPPINGS                                                   \
  10 // max number of mappings per map descriptor (spec max is 62)
#define AEM_MAX_NUM_CLOCK_SOURCES                                              \
  10 // max number of clock sources per clock domain descriptor (spec max is
     // 216)
#define AEM_MAX_LEN_CONTROL_VAL_DETAILS                                        \
  100 // max length of control value details (spec max is 404)

/* Descriptor name indices for state->descriptor_names[] */
typedef enum {
  AVB_NAME_ENTITY = 0, // entity_name (name_index 0)
  AVB_NAME_GROUP,      // group_name (name_index 1)
  AVB_NAME_AVB_INTERFACE,
  AVB_NAME_CONTROL_0, // IDENTIFY
  AVB_NAME_CONTROL_1, // Speaker Volume
  AVB_NAME_CONTROL_2, // Mic Gain
  AVB_NAME_STREAM_INPUT_0,
  AVB_NAME_STREAM_INPUT_1, // CRF media clock input when audio input is present
  AVB_NAME_STREAM_OUTPUT_0,
  AVB_NAME_STREAM_OUTPUT_1, // CRF media clock output
  AVB_NAME_COUNT            // total number of settable names
} avb_name_index_t;

/* Persistent data saved to NVS flash.
 * Holds user-settable names, control values, stream formats, and
 * listener connection state that survive reboots. Stored as a single
 * blob under NVS namespace "avb" / key "persist".
 *
 * WARNING — THIS STRUCT IS APPEND-ONLY.
 *
 * On-disk layout is a raw memcpy of this struct. Loader tolerance
 * (avb_persist_load) only works if every byte offset of every existing
 * field stays identical across firmware versions.
 *
 * MUST: only ADD fields at the END; bump AVB_PERSIST_VERSION.
 * MUST NOT: reorder/resize/remove/retype existing fields; change any
 * AVB_PERSIST_MAX_* below (those freeze array dimensions independently
 * of build-time AVB_MAX_* so config changes cannot shift offsets).
 *
 * Breaking these silently corrupts saved data on every upgrade — no
 * wire integrity check will catch it. */

/* Fixed upper bounds for the NVS blob layout. Do NOT change these
 * without also writing a migration path — see the warning above.
 * Build-time AVB_MAX_* constants may shrink/grow independently as
 * long as they stay ≤ these persist maxes (_Static_assert below). */
#define AVB_PERSIST_MAX_NAMES 16
#define AVB_PERSIST_MAX_INPUT_STREAMS 8
#define AVB_PERSIST_MAX_OUTPUT_STREAMS 8

/* Per-input-stream (listener) persistence. Only fields not derivable
 * on reconnect: vlan_id, class, and presentation-time offset all come
 * back from the talker's ACMP CONNECT_TX_RESPONSE and the AVTP stream
 * itself, so they are not stored. Inner fields are APPEND-ONLY — to
 * add new per-stream state later, append a parallel array at the end
 * of avb_persistent_data_s rather than growing this struct. */
typedef struct {
  uint8_t talker_id[8];     /* talker entity ID; all-zero = not configured */
  uint8_t talker_uid[2];    /* talker unique_id / stream output index */
  uint8_t controller_id[8]; /* controller that originated the CONNECT_RX */
  uint8_t stream_format[8]; /* AVTP stream format */
  uint8_t connected;        /* 1 = was connected at last save */
  uint8_t streaming_wait;   /* 1 = STREAMING_WAIT mode, per Milan §5.5.3.6.17 */
  uint8_t reserved[2];      /* pad to 4-byte boundary */
} avb_persist_input_stream_s; /* 32 bytes */

/* Per-output-stream (talker) persistence. APPEND-ONLY (see above). */
typedef struct {
  uint8_t stream_format[8]; /* AVTP stream format */
  uint32_t
      presentation_time_offset_ns; /* user-configured presentation offset. */
} avb_persist_output_stream_s;     /* 12 bytes */

#define AVB_PERSIST_VERSION 4
typedef struct {
  uint8_t version;       /* struct version — bump on every append */
  uint8_t reserved_v[3]; /* pad to 4-byte boundary */

  /* ----- Fixed-size arrays — capped by AVB_PERSIST_MAX_* above ----- */

  /* Descriptor names — indexed by avb_name_index_t */
  char descriptor_names[AVB_PERSIST_MAX_NAMES][64];

  /* Per-stream state (format + connection for inputs, format + PTO for outputs)
   */
  avb_persist_input_stream_s input_streams[AVB_PERSIST_MAX_INPUT_STREAMS];
  avb_persist_output_stream_s output_streams[AVB_PERSIST_MAX_OUTPUT_STREAMS];

  /* ----- Controls (growable tail — new controls append HERE) ----- */
  /* New scalar controls go at the very end so that appending a new
   * one doesn't shift any existing field's offset. */
  float speaker_vol_db;               /* speaker volume in dB */
  float mic_gain_db;                  /* mic gain in dB */
  uint16_t active_clock_source_index; /* CLOCK_DOMAIN 0 active source */
  uint16_t reserved_cs;               /* pad to 4-byte boundary */
  uint32_t audio_unit_sample_rate_hz; /* placeholder — wiring TBD */
  /* v4: last converged media-clock PLL correction (Q16 ppm). Preloaded
   * into the APLL at boot (before streams start) so the servo begins
   * within a few ppm of equilibrium instead of slewing tens of ppm
   * from zero — the crystal offset is a per-board constant modulo
   * temperature and grandmaster changes; the acquisition one-shot
   * trims any residual. 0 = never converged / no preload. */
  int32_t pll_trim_ppm_q16;
} avb_persistent_data_s;

/* Enforce that build-time sizes fit in the frozen persist layout.
 * If these fail: either shrink the AVB_MAX_* constant, or bump the
 * AVB_PERSIST_MAX_* constant AND write a migration path for every
 * device already in the field (see warning above). */
_Static_assert(AVB_NAME_COUNT <= AVB_PERSIST_MAX_NAMES,
               "AVB_NAME_COUNT exceeds AVB_PERSIST_MAX_NAMES — "
               "persist layout cannot hold all descriptor names");
_Static_assert(
    AVB_MAX_NUM_INPUT_STREAMS <= AVB_PERSIST_MAX_INPUT_STREAMS,
    "AVB_MAX_NUM_INPUT_STREAMS exceeds AVB_PERSIST_MAX_INPUT_STREAMS");
_Static_assert(
    AVB_MAX_NUM_OUTPUT_STREAMS <= AVB_PERSIST_MAX_OUTPUT_STREAMS,
    "AVB_MAX_NUM_OUTPUT_STREAMS exceeds AVB_PERSIST_MAX_OUTPUT_STREAMS");

/* Codec hardware capability. Codec-specific truth lives in avbcodec.c;
 * avbconfig.h policy filters are intersected with these capabilities during
 * AVB state initialization to produce the effective advertised capabilities. */
typedef struct {
  avb_sample_rates_s sample_rates;
  avb_bit_rates_s bit_rates;
  uint8_t max_input_channels;
  uint8_t max_output_channels;
  codec_control_range_s control_ranges;
} avb_codec_caps_s;

/* Default format */
#define AVB_DEFAULT_FORMAT_AM824(cip_sfc_sample_rate, channels)                \
  {.subtype = 0,                                                               \
   .vendor_defined = 0,                                                        \
   .format = 0x10,                                                             \
   .sf = 1,                                                                    \
   .fdf_sfc = cip_sfc_sample_rate,                                             \
   .fdf_evt = 0,                                                               \
   .dbs = (channels),                                                          \
   .sc = 0,                                                                    \
   .ut = 0,                                                                    \
   .nb = 1,                                                                    \
   .b = 0,                                                                     \
   .label_iec_60958_cnt = 0,                                                   \
   .label_mbla_cnt = (channels),                                               \
   .label_smptecnt = 0,                                                        \
   .label_midi_cnt = 0}

/* Default format for AAF */
#define AVB_DEFAULT_FORMAT_AAF(bit_rate, aaf_pcm_sample_rate, channels, upto)  \
  {.subtype = 2,                                                               \
   .vendor_defined = 0,                                                        \
   .sample_rate = aaf_pcm_sample_rate,                                         \
   .ut = (upto),                                                               \
   .format = 0x02,                                                             \
   .bit_depth = bit_rate,                                                      \
   .chan_per_frame_h = ((channels) >> 2),                                      \
   .chan_per_frame = ((channels) & 0x03),                                      \
   .samples_per_frame_h = 0,                                                   \
   .samples_per_frame = 6}

/* AVB Enums*/

/* Ethertypes
 * must convert to big-endian before writing to Ethernet header
 */
typedef enum {
  ethertype_msrp = 0x22ea,
  ethertype_avtp = 0x22f0,
  ethertype_vlan = 0x8100,
  ethertype_mvrp = 0x88f5,
  ethertype_gptp = 0x88f7
} ethertype_t;

/* AVB types of entities */
typedef enum {
  avb_entity_type_talker,
  avb_entity_type_listener,
  avb_entity_type_controller
} avb_entity_type_t;

/* ATDECC enums (ADP/AECP/AEM/ACMP) live in atdecc.h. */

/* Network types */

typedef uint8_t eth_addr_t[ETH_ADDR_LEN];
typedef uint8_t unique_id_t[UNIQUE_ID_LEN];
typedef struct {
  unique_id_t id; // entity id
  uint8_t uid[2]; // unique id
} identity_pair_t;

/* MRP / MSRP / MVRP types and protocol message declarations */
#include "mrp.h"

/* AVTP / MAAP types and protocol message declarations */
#include "avtp.h"

/* ATDECC (IEEE 1722.1) types and protocol message declarations.
 * Also provides avtp_msgbuf_u — the AVTP envelope union spanning
 * every subtype, including the ATDECC ones (ADP/AECP/ACMP). */
#include "atdecc.h"

/* General */

/* Generic AVB message buffer */
typedef union {
  avtp_msgbuf_u avtp;
  msrp_msgbuf_s msrp;
  mvrp_vlan_id_message_s mvrp;
  uint8_t raw[AVB_MAX_MSG_LEN];
} avb_msgbuf_u;

/* Talker */
typedef struct {
  unique_id_t entity_id;       // entity ID
  unique_id_t model_id;        // model ID
  eth_addr_t mac_addr;         // mac address
  talker_adv_info_s info;      // from talker advertise
  aem_stream_summary_s stream; // from get stream info
  uint8_t talker_uid[2];       // stream output descr index
  uint8_t failure_code;        // msrp failure code
  bool streaming;              // streaming status
  uint8_t last_msrp_event;     // last talker event
  bool ready;                  // general status
} avb_talker_s;

/* Listener */
typedef struct {
  unique_id_t stream_id;       // stream ID
  uint8_t vlan_id[2];          // vlan ID
  unique_id_t entity_id;       // entity ID
  unique_id_t model_id;        // model ID
  eth_addr_t mac_addr;         // mac address
  uint8_t listener_uid[2];     // stream input descr index
  uint8_t last_msrp_event;     // last msrp event
  uint8_t last_listener_event; // last listener event
  bool ready;                  // status
} avb_listener_s;

/* Controller */
typedef struct {
  unique_id_t entity_id; // entity ID
  unique_id_t model_id;  // model ID
  eth_addr_t mac_addr;   // mac address
  bool ready;            // status
} avb_controller_s;

/* Listener stream flags */
typedef struct {
  uint8_t class_b : 1;      // connection is class B, instead of class A
  uint8_t fast_connect : 1; // connection is using fast connect mode
  uint8_t
      saved_state : 1; // connection has saved ACMP state for fast connect mode
  uint8_t streaming_wait : 1;          // for milan, this must be false
  uint8_t supports_encrypted : 1;      // stream supports encrypted PDU
  uint8_t encrypted_pdu : 1;           // stream is using encrypted PDU
  uint8_t srp_registration_failed : 1; // listener has registered an SRP talker
                                       // failed attr or talker as registered an
                                       // SRP listener asking failed attr
  uint8_t cl_entries_valid : 1;        // connected_listeners field is valid
  uint8_t no_srp : 1;                  // SRP not used for the stream
  uint8_t udp : 1;                     // stream using UDP transport
  uint8_t reserved : 6;                // reserved for future use
} avb_listener_stream_flags_s;         // 2 bytes

/* Listener stream */
typedef struct {
  unique_id_t talker_id;       // talker entity ID
  uint8_t talker_uid[2];       // talker UID (same as stream output descr index)
  unique_id_t controller_id;   // controller entity ID
  unique_id_t stream_id;       // stream ID
  eth_addr_t stream_dest_addr; // stream destination address
  avb_listener_stream_flags_s stream_flags;  // stream flags
  aem_stream_info_flags_s stream_info_flags; // stream info flags
  uint8_t vlan_id[2];                        // vlan ID
  avtp_stream_format_s stream_format;        // stream format
  uint8_t msrp_accumulated_latency[4];       // msrp accumulated latency
  uint8_t msrp_failure_code[2];              // msrp failure code (kept in
                                             // sync by the MRP talker callback
                                             // for ATDECC stream_info responses;
                                             // decl_event derivation queries
                                             // the SM directly instead)
  bool connected;                            // status as connected
  bool pending_connection;                   // status as pending connection
  /* Timestamp of last fast-connect (CONNECT_TX) attempt. Runtime-only,
   * not persisted. Used by avb_periodic_send to throttle retries when
   * a saved binding is waiting for its talker to come back. */
  struct timespec last_fast_connect_attempt;
  /* Superfast connect (Kconfig): stream-in was started provisionally
   * at boot from the NVS binding with a derived stream_id, before
   * ACMP/MSRP completed. Fast-connect keeps probing while set (its
   * gate ignores `connected`); cleared by the first authoritative
   * ACMP connect response or a disconnect. */
  bool provisional;
  /* Staleness clock for the connected-demotion pass: stamped while
   * the stream shows life (frames arriving or talker advertise
   * registered); when it runs 10 s without a restamp (Milan
   * TMR_NO_TK), `connected` is demoted back to provisional so
   * fast-connect resumes probing — heals the case where the talker
   * changed stream identity during a link outage. ADP
   * ENTITY_DEPARTING demotes immediately. Runtime-only. */
  int64_t demote_ref_us;
} avb_listener_stream_s;

/* Talker stream */
typedef struct {
  unique_id_t stream_id;                     // stream ID
  eth_addr_t stream_dest_addr;               // stream destination address
  aem_stream_info_flags_s stream_info_flags; // stream info flags
  uint8_t vlan_id[2];                        // vlan ID
  avtp_stream_format_s stream_format;        // stream format
  uint8_t msrp_accumulated_latency[4];       // msrp accumulated latency
  uint8_t msrp_failure_code[2];              // msrp failure code
  uint8_t connection_count[2];               // number of connected listeners
  volatile bool stop_streaming;              // cross-core stop signal
  bool streaming;                       // true when stream_out_task running
  uint32_t presentation_time_offset_ns; // AVTP presentation offset /
                                        // max_transit_time
  /* AVB Lite unicast transport (avb_lite.md §6, "Stream transport
   * addressing"): active per-listener destination MACs, published by
   * the control plane (AVB main loop) and read lock-free by the TX
   * paths on the other core (stream-out task, CRF timer). Seqlock:
   * tx_da_seq is odd while a write is in progress; readers retry on
   * a seq change. tx_da_count == 0 means transmit to the stream's
   * multicast (MAAP) address — both the non-Lite default and the
   * Lite fan-out-exceeded escalation. */
  volatile uint32_t tx_da_seq;
  volatile uint8_t tx_da_count;
  eth_addr_t tx_da[CONFIG_ESP_AVB_LITE_UNICAST_FANOUT];
  struct {
    eth_addr_t mac_addr;      // from MSRP source
    identity_pair_t identity; // from ACMP connect_tx
    bool msrp_ready;          // MSRP listener declared ready
    /* True when msrp_ready was set by a live wire Ready this boot (vs
     * restored from the NVS journal). An AskingFailed declaration —
     * "no downstream listener can receive" per the §35.2.4.4.3 merge
     * rules — clears only live readiness: journal-restored readiness
     * is provisional (superfast connect streams on it while listeners
     * transiently declare AskingFailed during their own boot) and is
     * torn down only by the leave/age-out paths. */
    bool msrp_ready_live;
    bool acmp_connected;      // ACMP connect_tx received
    /* True when this listener currently has an MSRP Listener Asking
     * Failed declaration registered against this talker stream. Used
     * by ACMP GET_TX_STATE_RESPONSE to set REGISTERING_FAILED per
     * Milan v1.3 Table 5.23 — any non-zero entry flips the flag. */
    bool asking_failed;
    /* Per-listener liveness stamp, refreshed by this listener's own
     * CVU/MSRP declarations and ACMP activity. Needed because the
     * MRP listener registrar is keyed by stream_id — with several
     * listeners on one stream, the survivors' refreshes keep the
     * shared registrar alive forever, so a vanished listener can
     * only be detected per-row. AVB Lite ages stale rows on this
     * stamp (a dead listener's unicast copy otherwise floods as
     * unknown-unicast indefinitely); plain AVB ignores it. */
    int64_t last_seen_us;
  } connected_listeners[AVB_MAX_NUM_CONNECTED_LISTENERS];
} avb_talker_stream_s;

/* Stream Input — handled by avb_stream_rx_handler callback registered
 * via avb_net_set_stream_rx_handler. Runs inline in EMAC RX task. */

/* Control frame queue item — used by EMAC RX dispatcher to forward
 * non-stream frames (AVTP control, MSRP, MVRP) to the AVB main loop */
typedef struct {
  uint8_t protocol_idx;           // AVTP=0, MSRP=1, MVRP=2
  uint8_t ingress_port;           // 0 = Ethernet (port[0]), 1 = Wi-Fi (port[1]
                                  // on bridge builds). Always 0 in endpoint
                                  // mode where NUM_PORTS = 1.
  uint8_t src_addr[ETH_ADDR_LEN]; // source MAC address
  uint16_t length;                // payload length (ETH header stripped)
  uint8_t data[AVB_MAX_MSG_LEN];  // payload data
} ctrl_rx_pkt_t;

/* Stream RX handler callback — invoked inline from the EMAC RX task
 * context (see avb_unified_rx_cb in avbnet.c).
 *   avtp_data/len — raw AVTP data (ETH+VLAN headers already stripped). */
typedef void (*avb_stream_rx_handler_t)(uint8_t *avtp_data, uint16_t len,
                                        void *ctx);

/* Stream Output params */
struct stream_out_params_s {
  void *state;                          // pointer to AVB state (avb_state_s *)
  i2s_chan_handle_t i2s_rx_handle;      // handle to i2s rx channel
  esp_eth_handle_t eth_handle;          // EMAC handle for esp_eth_transmit
  uint16_t buffer_size;                 // buffer size in bytes
  uint16_t samples_per_packet;          // samples per AVTP packet
  uint16_t interval;                    // interval in microseconds
  uint16_t stream_index;                // output stream index (talker_uid)
  unique_id_t stream_id;                // stream ID
  eth_addr_t dest_addr;                 // stream destination MAC address
  uint8_t vlan_id[2];                   // stream VLAN ID
  uint8_t bit_depth;                    // bit depth
  uint8_t channels;                     // channels per frame
  uint32_t sample_rate;                 // sample rate
  uint32_t presentation_time_offset_ns; // AVTP presentation offset /
                                        // max_transit_time
  bool use_sine_wave;     // generate sine wave instead of reading mic
  float sine_freq;        // sine wave frequency in Hz
  uint8_t format_subtype; // 0 = IEC 61883 (AM824), 2 = AAF
  uint8_t cip_sfc;        // CIP SFC code (for AM824 format)
  uint8_t dbs;            // data block size (for AM824 format)
};

/* Carrier structure for querying AVB status */
typedef struct {
  sem_t *done;
  avb_status_s *dest;
} avb_statusreq_s;

/* Per-port AVB state. Holds per-port configuration plus asCapable /
 * neighborGptpCapable. Some historical per-port runtime state
 * (eth_handle, internal_mac_addr, l2if[], last_transmitted_* timers)
 * still lives in avb_state_s; migrating those scalars in and
 * threading port_index through the network + timer paths is a
 * pending cleanup.
 *
 * With CONFIG_ESP_AVB_NUM_PORTS=1 (the default) only port[0] exists. */
typedef struct {
  /* Configuration. */
  bool enabled;
  avb_port_medium_e medium;
  avb_port_host_if_e host_if;       /* how this port attaches to the SoC */
  avb_port_type_e type;             /* primary / failover / bridged */
  avb_port_wifi_mode_e wifi_mode;   /* ap/sta for wifi, none otherwise */
  uint32_t link_speed_mbps;         /* nominal PHY-rate cap */
  avb_port_time_source_e time_source;
  char eth_interface[16];

  /* IEEE 802.1AS-2020 §10.2.5 / §12.4 per-port flags. Populated by
   * port-init in avb_task; read by per-port logic. */
  bool as_capable;
  bool neighbor_gptp_capable;

  /* Per-port runtime state. */
  eth_addr_t internal_mac_addr;
  int l2if[AVB_NUM_PROTOCOLS]; // L2TAP fds for AVTP, MSRP, MVRP

  /* Last time we sent a periodic message on this port. */
  struct timespec last_transmitted_adp_entity_avail;
  struct timespec last_transmitted_mvrp_vlan_id;
  struct timespec last_transmitted_msrp_domain;
  struct timespec last_transmitted_msrp_talker_adv;
  struct timespec last_transmitted_msrp_listener;
  struct timespec last_transmitted_msrp_leaveall;

  /* MRP per-port timers (IEEE 802.1Q-2018 §10.7.11). All in
   * monotonic microseconds (esp_timer_get_time). 0 = idle/disarmed.
   * Fed by mrp_port_tick() in mrp.c. */
  int64_t mrp_join_timer_us;     /* tx! fires when now >= this */
  int64_t mrp_leaveall_timer_us; /* LeaveAll fires when now >= this */
  int64_t mrp_periodic_timer_us; /* periodic! fires when now >= this */
  bool mrp_leaveall_tx_pending;  /* next outgoing MRPDU sets LeaveAll bit */
} avb_port_s;

/* Main AVB state storage */
typedef struct avb_state_s {

  /* AVB configuration */
  avb_config_s config;
  avb_sample_rates_s supported_sample_rates; // codec caps ∩ config policy
  avb_bit_rates_s supported_bits_per_sample; // codec caps ∩ config policy

  /* Per-port array. */
  avb_port_s port[CONFIG_ESP_AVB_NUM_PORTS];

  /* Request for AVB task to stop or report status */
  bool stop;
  avb_statusreq_s status_req;
  struct ptpd_status_s ptp_status;

  QueueHandle_t ctrl_rx_queue; // control frame queue from EMAC dispatcher
  bool stream_in_active;       // stream-in handler is registered
  bool avb_lite;               // operating in AVB Lite mode (standard PTP)
  bool codec_enabled;          // codec enabled
  const void *codec_if;        // codec interface (audio_codec_if_t *)

  /* AECP control values */
  codec_control_range_s codec_ranges;        // codec-specific control ranges
  uint8_t ctrl_identify;                     // IDENTIFY control (0=off, 255=on)
  float ctrl_speaker_vol;                    // speaker volume in dB
  float ctrl_mic_gain;                       // mic gain in dB
  char descriptor_names[AVB_NAME_COUNT][64]; // user-settable descriptor names
  avb_persistent_data_s persist;             // persistent data (saved to NVS)

  // PTP clock snapshot for stream out media clock PLL (updated by main task
  // on core 0, read by stream out task on core 1)
  volatile uint32_t ptp_avtp_ts; // AVTP-format timestamp (lower 32b of PTP ns)
  volatile int64_t ptp_snapshot_us; // esp_timer_get_time() at moment of read

  /* Media-clock drift tracking. Measures how far the talker's media
   * clock (carried in stream AVTP timestamps or CRF payload
   * timestamps) has drifted from our local gPTP clock. Updated
   * inline by RX handlers (EMAC task); read by periodic log + the
   * future MCLK PLL. Values are plain (not atomic) — occasional
   * torn reads in log output are acceptable. */
  struct {
    /* CRF Media Clock input (STREAM_INPUT[avb_get_crf_input_index(state)]).
     * drift_ns = crf_timestamp - local_gptp_ns at moment of reception. */
    int64_t crf_last_drift_ns;
    int32_t crf_drift_min_ns;
    int32_t crf_drift_max_ns;
    int64_t crf_drift_sum_ns; /* for running mean */
    uint32_t crf_samples;
    /* Audio stream input (AAF or IEC 61883-6 AM824, STREAM_INPUT[0]).
     * drift_ns = avtp_timestamp (32-bit) - local_gptp_ns & 0xFFFFFFFF.
     * Expected to be ~+max_transit_time (2 ms Class A). */
    int32_t stream_last_drift_ns;
    int32_t stream_drift_min_ns;
    int32_t stream_drift_max_ns;
    int64_t stream_drift_sum_ns;
    uint32_t stream_samples;

    /* MCLK PLL counter: cumulative bytes accepted by I2S DMA. Pumped by
     * the stream-in drain callback (avtp.c) using the bytes-sent count
     * reported per DMA descriptor. avb_pll.c reads this paired with a
     * CRF-timestamp anchor to measure I2S rate error. */
    _Atomic uint64_t i2s_bytes_written;

    /* ADC-side twin of i2s_bytes_written: bytes the I2S RX DMA has
     * captured, counted in the on_recv callback (avbcodec.c). This is
     * the media-clock rate sensor for a TALKER: a pure talker never
     * writes I2S TX, so the DAC counter above is frozen and the PLL
     * would measure zero error forever while the APLL free-runs
     * (observed +31.5 ppm against the PreSonus grandmaster). ADC and
     * DAC share the APLL/MCLK, so either counter senses the same
     * clock; avb_pll's INTERNAL reference picks whichever is live. */
    _Atomic uint64_t i2s_bytes_captured;

    /* Last CONVERGED-STABLE PLL correction (Q16 ppm), 0 = none yet.
     * This is the authoritative live source for the persisted trim:
     * avb_persist_gather rebuilds the whole persist struct from live
     * state on every save, so the PLL cannot write persist fields
     * directly (observed: a direct write was wiped by the gather in
     * the same request_save call and 0 hit flash). Written by the
     * PLL's stability hook; seeded from the blob at load. */
    int32_t pll_converged_trim_q16;

    /* CRF-driven PLL anchor — atomically-published pair of
     *   (crf_ts_ns, i2s_bytes_written at the moment the CRF PDU arrived)
     * Writer: avb_crf_rx_handler (EMAC RX task). Reader: avb_pll_tick.
     * Uses a seqlock pattern: `seq` odd = write in progress, even =
     * complete; readers retry if they observe a torn pair. Milan §7.2
     * designates the CRF stream as the media clock reference, so the
     * PLL uses these timestamps rather than local gPTP wall-clock
     * reads whenever a CRF stream is connected. */
    struct {
      _Atomic uint32_t seq;
      uint64_t ts_ns;
      uint64_t bytes;
    } crf_anchor;

    /* PLL state owned by avb_pll.c. Fields public for log visibility. */
    int32_t pll_last_ppm_error_q16;       /* last 5 s window, Q16 */
    int32_t pll_cumulative_ppm_error_q16; /* since baseline, Q16 */
    int32_t pll_applied_ppm_q16;          /* last correction written to HW */
    int32_t pll_target_ppm_q16;           /* slew destination for applied */

    /* Milan GET_COUNTERS media_reset counter — bumped by the PLL when
     * gPTP discontinuity detection resets the baseline. Read by
     * avb_get_stream_in_counters. */
    uint32_t media_reset_count;

    /* Currently-selected CLOCK_SOURCE for the CLOCK_DOMAIN. Written by
     * AECP SET_CLOCK_SOURCE, read by the PLL (picks CRF vs gPTP), and
     * echoed in GET_CLOCK_SOURCE and the CLOCK_DOMAIN descriptor.
     *   0 = INTERNAL (gPTP) — PLL measures against local gPTP wall clock
     *   1 = CRF stream input — PLL requires a fresh CRF anchor to tick
     * Uint16 to match on-wire width; atomic isn't needed because this is
     * only written from the AECP RX task. */
    uint16_t active_clock_source_index;

    /* Effective listener-side sample rate and byterate, derived from
     * state->config at I2S init time. `listener_byterate` is the
     * nominal bytes/s the DAC should consume at zero ppm error — the
     * PLL uses it as its reference in compute_ppm_q16. `sample_rate`
     * is the audio sample rate in Hz. Both assume stereo 24-bit out
     * (2 ch × 3 B/frame) which the drain always produces; only the
     * rate varies across configurations. Set once at init; changing
     * the DAC rate at runtime would require I2S reconfiguration so
     * these stay constant for the lifetime of the AVB stack. */
    uint32_t listener_sample_rate;
    uint32_t listener_byterate;
  } media_clock;

  /* Our own entity */
  aem_entity_desc_s own_entity;

  /* AVB interface */
  // only one interface is supported currently
  aem_avb_interface_desc_s avb_interface;
  aem_msrp_mapping_s msrp_mappings[2];
  uint16_t msrp_mappings_count;

  /* Inflight commands */
  atdecc_inflight_command_s inflight_commands[AVB_MAX_NUM_INFLIGHT_COMMANDS];
  size_t num_inflight_commands;

  /* AVB streams */
  // index of input_streams is the listener_uid
  // index of output_streams is the talker_uid
  avb_listener_stream_s input_streams[AVB_MAX_NUM_INPUT_STREAMS];
  avb_talker_stream_s output_streams[AVB_MAX_NUM_OUTPUT_STREAMS];
  maap_stream_state_s maap[AVB_MAX_NUM_OUTPUT_STREAMS];
  size_t num_input_streams;
  size_t num_output_streams;

  /* Supported stream formats */
  avtp_stream_format_s supported_formats_in[AEM_MAX_NUM_FORMATS];
  size_t num_supported_formats_in;
  avtp_stream_format_s supported_formats_out[AEM_MAX_NUM_FORMATS];
  size_t num_supported_formats_out;
  avtp_stream_format_s default_format;

  /* Endpoints that we are aware of */
  avb_talker_s talkers[AVB_MAX_NUM_TALKERS];
  avb_listener_s listeners[AVB_MAX_NUM_LISTENERS];
  avb_controller_s controllers[AVB_MAX_NUM_CONTROLLERS];
  size_t num_talkers;
  size_t num_listeners;
  size_t num_controllers;

  /* Access restriction */
  bool acquired;
  bool locked;
  unique_id_t acquired_by;
  unique_id_t locked_by;
  struct timeval last_acquired;
  struct timeval last_locked;

  /* Unsolicited notifications */
  bool unsol_notif_enabled;
  struct timespec last_unsol_notif;

  /* Deferred NVS persist.
   *  persist_dirty  — set by any writer that wants the current state
   *                   flushed to flash; cleared by the persist task
   *                   before it performs the gather. Single-byte store,
   *                   no lock needed to set (idempotent across writers).
   *  persist_mutex  — guards state->persist (the gather snapshot buffer)
   *                   against concurrent access between AVB main
   *                   (writer, inside avb_persist_request_save) and the
   *                   persist task (reader, copying out for flash).
   *                   Held for a ~µs memcpy; never held during flash I/O. */
  volatile bool persist_dirty;
  /* Tick count (xTaskGetTickCount) of the moment persist_dirty most
   * recently transitioned 0→1. Used by the persist task to force a
   * flush past the streaming gate once the snapshot has waited longer
   * than AVB_PERSIST_FORCED_FLUSH_MSEC. Cleared together with
   * persist_dirty when the task completes a write. */
  volatile TickType_t persist_dirty_since_tick;
  SemaphoreHandle_t persist_mutex;

  /* Latest received packet and its timestamp (CLOCK_REALTIME)
   * 3 elements, 1 for each protocol (AVTP, MSRP, MVRP)
   */
  avb_msgbuf_u rxbuf[AVB_NUM_PROTOCOLS];     // received frame buffer
  size_t rxbuf_size[AVB_NUM_PROTOCOLS];      // size of the received frame
  struct timespec rxtime[AVB_NUM_PROTOCOLS]; // timestamp of the received frame
  eth_addr_t rxsrc[AVB_NUM_PROTOCOLS]; // source address of the received frame
  uint8_t rxport[AVB_NUM_PROTOCOLS];   // ingress port of the received frame
                                       // (0 = Eth, 1 = Wi-Fi on bridge)

  /* I2S handles */
  i2s_chan_handle_t i2s_tx_handle;
  i2s_chan_handle_t i2s_rx_handle;

  /* Last time we sent a periodic message (per-port timers migrated
   * into avb_port_s; these two stay in state because they are not
   * tied to a specific port). */
  struct timespec last_ptp_status_update;
  struct timespec last_transmitted_unsol_notif;

  /* Sequence IDs for outbound ATDECC messages */
  uint32_t adp_seq_id;
  uint16_t aecp_seq_id;
  int16_t acmp_seq_id;

  /* Logo */
  const uint8_t *logo_start;
  uint32_t logo_length;

} avb_state_s;

/* AVB Functions */

/* Network functions */
int avb_net_init(avb_state_s *state);
void avb_create_eth_frame(uint8_t *eth_frame, eth_addr_t *dest_addr,
                          avb_state_s *state, ethertype_t ethertype, void *msg,
                          uint16_t msg_len, uint8_t *vlan_id);
int avb_net_send_to(avb_state_s *state, ethertype_t ethertype, void *msg,
                    uint16_t msg_len, struct timespec *ts,
                    eth_addr_t *dest_addr);
int avb_net_send_to_vlan(avb_state_s *state, ethertype_t ethertype, void *msg,
                         uint16_t msg_len, struct timespec *ts,
                         eth_addr_t *dest_addr, uint8_t *vlan_id);
int avb_net_send(avb_state_s *state, ethertype_t ethertype, void *msg,
                 uint16_t msg_len, struct timespec *t);
/* Per-port variant of avb_net_send. Identical to avb_net_send for
 * port 0. For port > 0 the frame is built with the egress port's own
 * source MAC and dispatched via that port's medium — esp_wifi_internal_tx
 * (WIFI_IF_AP) for the bridge SoftAP, L2TAP for a second Ethernet port.
 * Used by bridge MAP to route SM TX flushes to the right port. */
int avb_net_send_on(avb_state_s *state, int port_index,
                    ethertype_t ethertype, void *msg, uint16_t msg_len,
                    struct timespec *ts);
int avb_net_recv_ctrl(avb_state_s *state, int *protocol_idx, int *ingress_port,
                      void *msg,
                      uint16_t msg_len, eth_addr_t *src_addr, int timeout_ms);

/* Fast-path raw frame TX for AVTP stream egress.
 *
 * The control-plane senders (avb_net_send / _send_to / _send_to_vlan)
 * write to L2TAP file descriptors so the netif stack handles framing.
 * The talker stream tasks bypass that and shoot the already-built
 * Ethernet frame directly into the DMA ring — every microsecond
 * counts on the audio path.
 *
 * Implementation is the medium-abstraction seam: wraps
 * esp_eth_transmit on EMAC-backed ports and esp_wifi_internal_tx on
 * wifi-backed ports. Callers don't branch on medium. */
esp_err_t avb_net_transmit_raw(esp_eth_handle_t eth_handle, const void *frame,
                               size_t frame_len);

void avb_net_set_stream_rx_handler(avb_stream_rx_handler_t handler, void *ctx);

/* Number of incoming stream frames dropped because the AVB-IN queue
 * was full. Diagnostic counter; persistent across the session. */
uint32_t avb_net_stream_rx_drops(void);
/* Total PTP (0x88f7) frames seen in the EMAC RX callback — before the
 * L2TAP filter/forward layer. Compare against ptpd's rx_sync counter to
 * split where drops occur. */
uint32_t avb_net_ptp_rx_seen(void);

/* Per-ethertype RX counters into avb_unified_rx_cb. Used to confirm that
 * frames actually arrive on a given port's Wi-Fi/EMAC ingress, separate
 * from whether any specific handler later acts on them. Counters are
 * monotonic. */
void avb_net_rx_breakdown(uint32_t *total, uint32_t *avtp, uint32_t *msrp,
                          uint32_t *mvrp, uint32_t *vlan, uint32_t *other);

/* L2 RX instrument: count of entries into the Wi-Fi RX callback, taken
 * before the per-frame malloc. Compare against avb_net_rx_breakdown's
 * total to localize a Wi-Fi RX stall (driver-not-delivering vs OOM-drop
 * vs dispatch). Monotonic; zero on builds with no Wi-Fi RX path. */
uint32_t avb_net_wifi_rx_cb_count(void);

/* Bridge forwarding stats — per-direction OK / fail / OOM counters
 * bumped inside avb_bridge_forward. Used to diagnose missing
 * wired->Wi-Fi or Wi-Fi->wired multicast traffic. Zero on non-bridge
 * builds (the forwarder isn't compiled in). */
void avb_bridge_forward_stats(uint32_t *eth_ok, uint32_t *eth_fail,
                              uint32_t *wifi_ok, uint32_t *wifi_fail,
                              uint32_t *wifi_oom);

/* Wi-Fi-side OK forwards split by destination MAC type (multicast bit).
 * Used by the bridge heartbeat to surface whether the AP path can deliver
 * unicast frames to associated STAs vs only multicast — IDF's wifi driver
 * uses different TX paths for the two cases. */
void avb_bridge_forward_stats_wifi_split(uint32_t *wifi_ok_ucast,
                                         uint32_t *wifi_ok_mcast);

/* AVB send functions */

/* MVRP and MSRP send functions live in mrp.h. */

/* dest = NULL broadcasts (talker declarations); listener declarations
 * pass the talker's MAC per avb_lite.md §6 item 2. */
int avb_send_cvu_srp_attr(avb_state_s *state, void *attr, int attr_list_len,
                          const char *label, const eth_addr_t *dest);

/* AVTP / MAAP send functions live in avtp.h. */

/* AVB processing functions */

/* MVRP and MSRP processing functions live in mrp.h. */

/* AVTP / MAAP processing functions live in avtp.h. */


/* Identify tone */
void avb_identify_tone(avb_state_s *state, uint32_t duration_ms);


/* Listener decl_event derivation. Returns the msrp_listener_event_t
 * code (AskingFailed / Ready / ReadyFailed) that an input_stream
 * should declare based on its ACMP connection state and the upstream
 * talker's MSRP state. See avb.c for the rule. */
msrp_listener_event_t
avb_input_stream_decl_event(const avb_listener_stream_s *s);

/* Stream functions */
int avb_start_stream_in(avb_state_s *state, uint16_t index);
void avb_stop_stream_in(avb_state_s *state, uint16_t index);
uint16_t avb_get_crf_input_index(avb_state_s *state);
uint16_t avb_get_crf_output_index(avb_state_s *state);
bool avb_crf_stream_valid(void);
void avb_stream_in_print_diag(void);
/* Deferred drift sampler — called from AVB main's periodic loop.
 * Replaces the per-packet clock_gettime(CLOCK_PTP_SYSTEM) calls that
 * used to live in avb_stream_rx_handler / avb_crf_rx_handler and were
 * contending with PTPD on CPU0. Rate-limits itself to ~100 Hz. */
void avb_stream_in_sample_drift(avb_state_s *state);

/* avbpll.c — Milan media-clock PLL: measures I2S MCLK rate vs gPTP and
 * retunes the underlying hardware (APLL on ESP32-P4) to close the loop.
 * The hardware backend is kept behind a thin internal abstraction so
 * porting to a different SoC / to an external clock chip (e.g. Cirrus
 * CS2000) only touches avbpll.c. */
int avb_pll_init(uint32_t nominal_mclk_hz);
void avb_pll_deinit(void);
/* Preload the persisted media-clock trim (persist v4) — call after
 * avb_pll_init and before streams start. */
void avb_pll_preload_trim(avb_state_s *state, int32_t trim_ppm_q16);
/* Called ~once per second from the AVB main loop. Updates measurements,
 * logs stats (including the drift sums maintained by the RX handlers),
 * and applies any due MCLK correction. */
void avb_pll_tick(avb_state_s *state);
/* EMAC RX un-wedge guard — re-issues the receive poll demand when the
 * RX DMA is stuck in Suspended state (lost poll-demand race after a
 * descriptor-exhaustion burst). Call every main-loop pass; no-op on
 * targets without an on-chip EMAC. */
void avb_emac_rx_unwedge_tick(void);
void avb_get_stream_in_counters(aem_stream_in_counters_val_s *valid,
                                aem_stream_in_counters_s *counters);
uint32_t aaf_code_to_sample_rate(uint8_t code);
int avb_start_stream_out(avb_state_s *state, uint16_t index);
int avb_stop_stream_out(avb_state_s *state, uint16_t index);
void avb_remove_talker_listener_by_index(avb_talker_stream_s *stream, int idx);

#ifdef CONFIG_ESP_AVB_SUPERFAST_CONNECT
/* Provisional stream-in start from restored NVS bindings — called
 * once at boot after avb_persist_load (see avtp.c). */
void avb_superfast_connect_start(avb_state_s *state);
#endif

/* Most recent stream-frame arrival for an input stream (µs,
 * esp_timer domain); 0 when RX inactive. Defined in avtp.c. */
int64_t avb_stream_in_last_rx_us(avb_state_s *state, uint16_t index);

/* AVB Lite unicast transport — control-plane writer, called from the
 * AVB main loop. Publishes each output stream's active unicast
 * destination MACs (from connected_listeners[]) into the tx_da[]
 * mailbox, or count 0 for multicast (non-Lite, no listeners, or
 * fan-out exceeded → MAAP escalation). */
void avb_lite_update_stream_tx_addrs(avb_state_s *state);

/* Lock-free TX-side snapshot of a stream's published unicast DAs.
 * Safe from the stream-out task / CRF timer while the control plane
 * updates concurrently. Returns the number of DAs copied into out[]
 * (0 → caller transmits to its multicast template DA). */
static inline int avb_stream_tx_addrs_snapshot(const avb_talker_stream_s *stream,
                                               eth_addr_t *out) {
  uint32_t s1, s2;
  int n;
  do {
    s1 = stream->tx_da_seq;
    __sync_synchronize();
    n = stream->tx_da_count;
    if (n > CONFIG_ESP_AVB_LITE_UNICAST_FANOUT)
      n = CONFIG_ESP_AVB_LITE_UNICAST_FANOUT;
    memcpy(out, stream->tx_da, (size_t)n * sizeof(eth_addr_t));
    __sync_synchronize();
    s2 = stream->tx_da_seq;
  } while (s1 != s2 || (s1 & 1));
  return n;
}

/* Codec functions */
const avb_codec_caps_s *avb_codec_get_caps(avb_codec_type_t codec_type);
esp_err_t avb_config_i2s(avb_state_s *state);
esp_err_t avb_config_codec(avb_state_s *state);
/* Reconfigure I2S + codec to a new sample rate. Only valid while no
 * stream is running in either direction (codecs need a stop/start
 * around a rate change); the AECP SET_STREAM_FORMAT handler gates
 * this with STREAM_IS_RUNNING. */
esp_err_t avb_audio_set_rate(avb_state_s *state, uint32_t rate);
int16_t avb_codec_quantize_tenth_db(const codec_control_range_s *ranges,
                                    bool gain, int16_t value_tenth_db);
void avb_codec_set_vol(avb_state_s *state, float db);
void avb_codec_set_mic_gain(avb_state_s *state, float db);

/* NVS persistent storage */
esp_err_t avb_persist_load(avb_state_s *state);
esp_err_t avb_persist_save(avb_state_s *state);

/* Centralized periodic diagnostic — prints per-task %CPU, per-core
 * utilization, stack high-water marks, STREAM-IN, STREAM-OUT, and
 * MCLK/PLL state every AVB_STATS_WINDOW_MS (see avbstats.c). All
 * periodic logging runs from this low-priority sampler; measurement
 * code (handlers, drain, PLL tick) only updates shared counters. */
void avb_cpu_stats_start(void);

/* Give AVB-STATS a pointer to the AVB state so it can sample
 * media-clock/PLL data for the MCLK line. Must be called before
 * the first stats tick fires (safe to call immediately after
 * avb_create_state). */
void avb_cpu_stats_set_state(avb_state_s *state);

/* Print the MCLK/PLL status line using the latest computed values in
 * state->media_clock. Called by AVB-STATS. Resets the per-window
 * drift accumulators after printing. */
void avb_pll_print_stats(avb_state_s *state);

/* Cumulative productive-work microseconds spent inside avb_stream_out_task
 * (excludes busy-wait). Delta over a window gives the "real" CPU AVB-OUT
 * would need if the busy-wait were replaced with a yielding wait. */
uint64_t avb_stream_out_work_us_total(void);

/* Print the per-window STREAM-OUT diagnostic line — called by AVB-STATS.
 * Silent when no TX session is active. Mirrors avb_stream_in_print_diag. */
void avb_stream_out_print_diag(void);
/* Hot-path API: capture a snapshot of current state under the persist
 * mutex and mark it dirty. Returns immediately — the actual flash
 * write happens later from the persist task. Safe to call from any
 * task that writes persist-tracked state. */
void avb_persist_request_save(avb_state_s *state);
/* Stream-journal persistence: append one 32-byte stream record instead of
 * rewriting the whole persistent blob. Intended for connect/disconnect hot
 * paths where a full NVS blob write is too disruptive. */
esp_err_t avb_persist_append_input_stream(avb_state_s *state, uint16_t index);
esp_err_t avb_persist_append_output_stream(avb_state_s *state, uint16_t index);
/* FreeRTOS task entry point for the deferred NVS writer. */
void avb_persist_task(void *arg);
void i2s_task(void *arg);

/* Helper functions */
void stream_id_from_mac(eth_addr_t *mac_addr, uint8_t *stream_id, size_t uid);

#endif /* _ESP_AVB_AVB_H_ */
