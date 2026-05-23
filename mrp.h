/*
 * Copyright 2024-2026 Scramble Tools
 * License: MIT
 *
 * MRP / MSRP / MVRP types and protocol message declarations.
 *
 * Wire-format structs follow IEEE 802.1Q-2018 §10 (MRP), §35 (MSRP),
 * and §11 (MVRP). Multi-byte fields use uint8_t[N] byte arrays (always
 * big-endian on wire); bit-fields stay within single octets for
 * compiler-portable layout. The struct IS the wire format — RX/TX use
 * direct memcpy, no separate encode/decode pass.
 *
 * This header is included from avb.h after the prerequisite types
 * (eth_addr_t, unique_id_t) are defined. avb_state_s is forward-
 * declared below; its full definition follows in avb.h.
 *
 * CVU (AVB Lite SRP-over-AECP) wire wrappers live in the ATDECC
 * section of avb.h and embed the native MSRP wire types defined here.
 * The MSRP state machine in mrp.c drives both native MSRP and CVU
 * transports; the selection happens at TX time, per port.
 */

#ifndef ESP_AVB_MRP_H_
#define ESP_AVB_MRP_H_

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

/* Forward decl — full definition in avb.h. */
typedef struct avb_state_s avb_state_s;

/* ===== MRP event limits ===== */

#define MRP_MAX_NUM_EVENTS 20 // max number of events

/* ===== MVRP / MSRP attribute enums ===== */

/* MVRP attribute types in enumerated order */
typedef enum {
  mvrp_attr_type_none,
  mvrp_attr_type_vlan_identifier
} mvrp_attr_type_t;

/* MSRP attribute types in enumerated order */
typedef enum {
  msrp_attr_type_none,
  msrp_attr_type_talker_advertise,
  msrp_attr_type_talker_failed,
  msrp_attr_type_listener,
  msrp_attr_type_domain
} msrp_attr_type_t;

/* MRP attribute events in enumerated order */
typedef enum {
  mrp_attr_event_new,
  mrp_attr_event_join_in,
  mrp_attr_event_in,
  mrp_attr_event_join_mt,
  mrp_attr_event_mt,
  mrp_attr_event_lv,
  mrp_attr_event_none // no event
} mrp_attr_event_t;

/* MSRP listener events in enumerated order */
typedef enum {
  msrp_listener_event_ignore,
  msrp_listener_event_asking_failed,
  msrp_listener_event_ready,
  msrp_listener_event_ready_failed
} msrp_listener_event_t;

/* MSRP reservation failure codes in enumerated order */
typedef enum {
  no_failure,
  insufficient_bandwidth,
  insufficient_bridge_resources,
  insufficient_bandwidth_for_traffic_class,
  stream_id_in_use_by_another_talker,
  stream_destination_address_already_in_use,
  stream_preempted_by_higher_rank,
  reported_latency_has_changed,
  egress_port_is_not_avb_capable,
  use_a_different_destination_address,
  out_of_msrp_resources,
  out_of_mmrp_resources,
  cannot_store_destination_address,
  requested_priority_is_not_an_sr_class_priority,
  max_frame_size_is_too_large_for_media,
  max_fan_in_ports_limit_has_been_reached,
  changes_in_first_value_for_a_registered_stream_id,
  vlan_is_blocked_on_this_egress_port__registration_forbidden,
  vlan_tagging_is_disabled_on_this_egress_port__untagged_set,
  sr_class_priority_mismatch,
  enhanced_feature_cannot_be_propagated_to_original_port,
  max_latency_exceeded,
  nearest_bridge_cannot_provide_network_identification_for_stream_transformation,
  stream_transformation_not_supported,
  stream_identification_type_not_supported_for_stream_transformation,
  enhanced_feature_cannot_be_supported_without_a_cnc
} msrp_reservation_failure_code_t;

/* ===== MRP wire types =====
 *
 * Per IEEE 802.1Q-2018 §10. All multi-byte fields are big-endian.
 */

/* MRP three packed event (IEEE 802.1Q-2018 Clause 10.8.2.10)
 * (event1 * 36) + (event2 * 6) + event3 = 3pe value
 */
typedef uint8_t mrp_3pe_event_t;

/* MRP four packed event (IEEE 802.1Q-2018 Clause 10.8.2.11)
 * (event1 * 64) + (event2 * 16) + (event3 * 4) + event4 = 4pe value
 * The formula is not needed, as the struct below is sufficient.
 */
typedef struct {
  uint8_t event4 : 2; // 4th event
  uint8_t event3 : 2; // 3rd event
  uint8_t event2 : 2; // 2nd event
  uint8_t event1 : 2; // 1st event
} mrp_4pe_event_s;    // 1 byte

/* Union of 3PE and 4PE events */
typedef union {
  mrp_3pe_event_t event;
  mrp_4pe_event_s declaration;
} mrp_event_u;

/* ===== MVRP wire types =====
 *
 * Per IEEE 802.1Q-2018 §11. All multi-byte fields are big-endian.
 */

/* MVRP attribute header */
typedef struct {
  uint8_t attr_type;            // attribute type
  uint8_t attr_len;             // attribute length
  uint8_t vechead_padding : 5;  // padding (ignored part of num_vals)
  uint8_t vechead_leaveall : 3; // 0 or 1, if 0 then num_vals is non-zero
  uint8_t vechead_num_vals; // # of events (div by 3, round up for # of 3pes)
} mrp_attr_header_s;        // 6 bytes

/* MVRP VLAN identifier message */
typedef struct {
  uint8_t protocol_ver;           // protocol version
  mrp_attr_header_s header;       // attribute header
  uint8_t vlan_id[2];             // vlan ID
  mrp_3pe_event_t attr_event[20]; // allow up to 20 events, ignore the rest
} mvrp_vlan_id_message_s;         // 23 bytes limit

/* ===== MSRP wire types =====
 *
 * Per IEEE 802.1Q-2018 §35. All multi-byte fields are big-endian.
 */

/* MSRP attribute header */
typedef struct {
  uint8_t attr_type;            // attribute type
  uint8_t attr_len;             // attribute length
  uint8_t attr_list_len[2];     // attribute list length
  uint8_t vechead_padding : 5;  // padding (ignored part of num_vals)
  uint8_t vechead_leaveall : 3; // 0 or 1, if 0 then num_vals is non-zero
  uint8_t vechead_num_vals; // # of events (div by 3, round up for # of 3pes)
} msrp_attr_header_s;       // 6 bytes

/* MSRP domain message */
typedef struct {
  msrp_attr_header_s header;                      // attribute header
  uint8_t sr_class_id;                            // sr class ID
  uint8_t sr_class_priority;                      // sr class priority
  uint8_t sr_class_vid[2];                        // sr class VID
  mrp_3pe_event_t attr_event[MRP_MAX_NUM_EVENTS]; // attribute events
} msrp_domain_message_s;                          // 25 bytes limit

/* Talker advertise information */
typedef struct {
  unique_id_t stream_id;               // stream ID
  eth_addr_t stream_dest_addr;         // stream destination address
  uint8_t vlan_id[2];                  // vlan ID
  uint8_t tspec_max_frame_size[2];     // tspec max frame size
  uint8_t tspec_max_frame_interval[2]; // tspec max frame interval
  uint8_t reserved : 4;
  uint8_t rank : 1;               // 1 = non-emergency
  uint8_t priority : 3;           // 3 = class A, 2 = class B
  uint8_t accumulated_latency[4]; // as stated by the talker
} talker_adv_info_s;

/* MSRP talker advertise message */
typedef struct {
  msrp_attr_header_s header; // attribute header
  talker_adv_info_s info;    // talker advertise information
  mrp_3pe_event_t event_data[MRP_MAX_NUM_EVENTS]; // attribute events
} msrp_talker_adv_message_s;                      // 44 bytes limit

/* MSRP talker failed message */
typedef struct {
  msrp_attr_header_s header;    // attribute header
  talker_adv_info_s info;       // talker advertise information
  uint8_t failure_bridge_id[8]; // failure bridge ID or padded mac
  uint8_t failure_code;         // failure code
  mrp_3pe_event_t event_data[MRP_MAX_NUM_EVENTS]; // attribute events
} msrp_talker_failed_message_s;                   // 53 bytes limit

/* MSRP talker message union */
typedef union {
  msrp_attr_header_s header;
  msrp_talker_adv_message_s talker;
  msrp_talker_failed_message_s talker_failed;
} msrp_talker_message_u;

/* MSRP listener message */
typedef struct {
  msrp_attr_header_s header;                       // attribute header
  unique_id_t stream_id;                           // stream ID
  mrp_event_u event_decl_data[MRP_MAX_NUM_EVENTS]; // events and declarations
} msrp_listener_message_s;                         // 29 bytes limit

/* MSRP attribute union */
typedef union {
  msrp_attr_header_s header;
  msrp_domain_message_s domain;
  msrp_talker_adv_message_s talker_adv;
  msrp_talker_failed_message_s talker_failed;
  msrp_listener_message_s listener;
  uint8_t raw[53];
} msrp_attribute_u; // 53 bytes limit

/* MSRP message buffer */
typedef struct {
  uint8_t protocol_ver;      // protocol version
  uint8_t messages_raw[200]; // variable length depending on attributes
} msrp_msgbuf_s;             // 266 bytes limit

/* ===== MVRP — SM-driven API ===== */

/* RX entry point. avbnet.c hands each MVRP MRPDU here; the SMs
 * handle attribute book-keeping. */
void mrp_rx_mvrp(avb_state_s *state, int port,
                 mvrp_vlan_id_message_s *msg, size_t length,
                 eth_addr_t *src_addr);

/* Idempotent SM-driven origination: first call seeds the Applicant
 * via New!, subsequent calls via Join!. JoinTimer + PeriodicTimer
 * drive the MRPDU cadence. */
void mrp_declare_vlan(avb_state_s *state, int port,
                      const uint8_t *vlan_id);
void mrp_withdraw_vlan(avb_state_s *state, int port,
                       const uint8_t *vlan_id);

/* ===== MRP state machines (IEEE 802.1Q-2018 §10) =====
 *
 * Per-attribute Applicant + Registrar pair. Mode-agnostic — same
 * code drives endpoint and bridge. MSRP and MVRP applications (in
 * mrp.c §6/§7) hold their own typed attribute tables and call the
 * generic SM step functions on each entry's mrp_sm_state_t.
 */

/* Applicant SM states (IEEE 802.1Q-2018 §10.7.7 Table 10-3).
 * V_ = Very anxious, A_ = Anxious, Q_ = Quiet, L_ = LeaveAll.
 * Suffix: O = Observer, P = Passive, N = New, A = Active, L = Leaving. */
typedef enum {
  mrp_applicant_vo, // Very anxious Observer
  mrp_applicant_vp, // Very anxious Passive
  mrp_applicant_vn, // Very anxious New
  mrp_applicant_an, // Anxious New
  mrp_applicant_aa, // Anxious Active
  mrp_applicant_qa, // Quiet Active
  mrp_applicant_la, // Leaving Active
  mrp_applicant_ao, // Anxious Observer
  mrp_applicant_qo, // Quiet Observer
  mrp_applicant_ap, // Anxious Passive
  mrp_applicant_qp, // Quiet Passive
  mrp_applicant_lo  // Leaving Observer
} mrp_applicant_state_e;

/* Registrar SM states (IEEE 802.1Q-2018 §10.7.8 Table 10-4). */
typedef enum {
  mrp_registrar_in, // IN: peer has declared, registration active
  mrp_registrar_lv, // LV: peer's declaration is being torn down (LeaveTimer)
  mrp_registrar_mt  // MT: empty / no registration
} mrp_registrar_state_e;

/* Peer-side events ("received" events) per §10.7.5 Table 10-2.
 * These are decoded from the 3pe vector in an inbound MRPDU and fed
 * into both the Applicant and the Registrar SMs for the matching
 * attribute. */
typedef enum {
  mrp_event_r_new,     // rNew: peer issued New
  mrp_event_r_join_in, // rJoinIn: peer issued JoinIn
  mrp_event_r_in,      // rIn: peer issued In
  mrp_event_r_join_mt, // rJoinMt: peer issued JoinMt
  mrp_event_r_mt,      // rMt: peer issued Mt
  mrp_event_r_lv,      // rLv: peer issued Lv
  mrp_event_r_la,      // rLA: peer issued LeaveAll
  /* Local-side events ("originator" events) per §10.7.5 Table 10-2.
   * These come from the application layer (avb.c endpoint origination
   * or bridge MAP) via mrp_declare_*. */
  mrp_event_new,       // New!: local origin declares New
  mrp_event_join,      // Join!: local origin declares Join
  mrp_event_lv,        // Lv!:  local origin declares Leave
  /* Timer-side events. */
  mrp_event_tx,        // tx!: JoinTimer expired; encode pending event
  mrp_event_tx_la,     // txLA!: tx! coincides with LeaveAll TX
  mrp_event_periodic,  // periodic!: PeriodicTransmission timer fired
  mrp_event_leave_timer // Registrar LeaveTimer expired
} mrp_event_e;

/* Applicant TX action — the "what to send" decision the Applicant
 * SM leaves for the encoder. The Applicant doesn't know the Registrar
 * state at decision time; the encoder resolves sJ → JoinIn (3pe=1)
 * or JoinMt (3pe=3) based on the Registrar's current state when the
 * PDU is built. */
typedef enum {
  mrp_tx_none,  // no transmission pending
  mrp_tx_new,   // sN  → 3pe New (0)
  mrp_tx_join,  // sJ  → 3pe JoinIn (1) if Registrar IN, JoinMt (3) if MT
  mrp_tx_leave  // sL  → 3pe Lv (5)
} mrp_tx_action_e;

/* Per-attribute SM state. Embedded in each application's typed
 * attribute-table entry (e.g. msrp_talker_entry_t in mrp.c §6). */
typedef struct {
  mrp_applicant_state_e applicant; // §10.7.7 state
  mrp_registrar_state_e registrar; // §10.7.8 state
  int64_t leave_timer_us;          // Registrar LeaveTimer expiry (0 = idle)
  mrp_tx_action_e pending_tx;      // queued TX action for next tx!
} mrp_sm_state_t;

/* ===== MRP public API (generic SM driver) =====
 *
 * Called by the §6/§7 application code on its own typed table entries.
 * The SM driver is unaware of MSRP vs MVRP — it operates on the state
 * fields alone.
 */

/* Step the Applicant SM by one event. May mutate sm->applicant and
 * set sm->pending_tx if the transition demands TX. */
void mrp_applicant_step(mrp_sm_state_t *sm, mrp_event_e ev);

/* Step the Registrar SM by one event. May mutate sm->registrar and
 * sm->leave_timer_us. */
void mrp_registrar_step(mrp_sm_state_t *sm, mrp_event_e ev);

/* Per-port timer machinery (IEEE 802.1Q-2018 §10.7.11). */

/* Initialize the per-port timers for `port`. Called at port bring-up. */
void mrp_port_init(avb_state_s *state, int port);

/* Advance per-port timers. Called from the AVB main loop on every
 * tick. Returns true if anything fired this tick. */
bool mrp_port_tick(avb_state_s *state, int port);

/* Arm the JoinTimer on `port` if not already armed. Called by the
 * application layer (§6/§7) after an Applicant transition leaves a
 * non-none pending_tx. */
void mrp_port_arm_join_timer(avb_state_s *state, int port);

/* SM-driven MSRP RX entry point (§6a). Walks an inbound MSRP
 * buffer, decodes 3pe events, dispatches into per-attribute SMs,
 * and runs application bookkeeping for each. */
void mrp_rx_msrp(avb_state_s *state, int port, msrp_msgbuf_s *msg,
                 size_t length, eth_addr_t *src_addr);

/* SM-driven MSRP origination (§6b). Called by avb.c on local
 * stream-state changes; each fills a typed wire struct in the
 * matching attribute table, drives the Applicant via the
 * appropriate local-origin event, and arms the JoinTimer. */
void mrp_declare_talker_advertise(avb_state_s *state, int port,
                                  const unique_id_t *stream_id,
                                  const eth_addr_t *stream_dest_addr,
                                  const uint8_t *vlan_id,
                                  uint16_t max_frame_size,
                                  bool class_b);
/* src_bridge_id: 8 bytes of the bridge that originally detected the
 * failure (preserved on multi-bridge propagation per §35.2.4 cascade
 * semantics). Pass NULL when *this* bridge / endpoint is the source
 * of the failure; the function will then derive an EUI-64 from
 * port[0]'s MAC. */
void mrp_declare_talker_failed(avb_state_s *state, int port,
                               const unique_id_t *stream_id,
                               const eth_addr_t *stream_dest_addr,
                               const uint8_t *vlan_id,
                               uint16_t max_frame_size, uint8_t failure_code,
                               bool class_b,
                               const uint8_t *src_bridge_id);
void mrp_declare_listener(avb_state_s *state, int port,
                          const unique_id_t *stream_id,
                          msrp_listener_event_t decl);
void mrp_declare_domain(avb_state_s *state, int port, uint8_t sr_class_id,
                        uint8_t sr_class_priority, const uint8_t *sr_class_vid);
void mrp_withdraw_talker(avb_state_s *state, int port,
                         const unique_id_t *stream_id);
void mrp_withdraw_listener(avb_state_s *state, int port,
                           const unique_id_t *stream_id);

/* Other MSRP utilities. mrp_declare_* / mrp_withdraw_* emit;
 * mrp_rx_msrp consumes. Endpoint bookkeeping (talker DB,
 * accumulated_latency, listener readiness) runs inside the SM
 * transition callbacks in mrp.c §6a.
 *
 * Note: avb_send_cvu_srp_attr (CVU SRP-over-AECP) lives in avb.h's
 * ATDECC section — same MSRP payload, different envelope. */

uint16_t avb_compute_tspec_max_frame_size(avb_state_s *state, uint16_t index);

/* Query the current Registrar state of an upstream MSRP TALKER
 * attribute. Returns true iff the matching SM is currently IN (i.e.
 * the peer/bridge has the talker declared on this port). Used by the
 * listener-side decl_event helper to drive Ready vs AskingFailed
 * independently of whether the listener saw a fresh transition.
 *
 * port: ingress port to query (single-port endpoints pass 0).
 * stream_id: the 8-byte stream ID to look up. */
bool mrp_talker_advertise_active(int port, const unique_id_t *stream_id);

/* Like above but for the TALKER_FAILED attribute. Returns true iff
 * the FAILED Registrar is IN; if it is, *failure_code_out (when not
 * NULL) is populated with the §35.2.2.8.4 reason code. */
bool mrp_talker_failed_active(int port, const unique_id_t *stream_id,
                              uint8_t *failure_code_out);

#endif /* ESP_AVB_MRP_H_ */
