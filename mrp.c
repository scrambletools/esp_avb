/*
 * Copyright 2024-2026 Scramble Tools
 * License: MIT
 *
 * SPDX-License-Identifier: MIT
 *
 * ESP_AVB MRP — Multiple Registration Protocol family
 *
 * File sections (the file collapses what would otherwise be mrp.c +
 * msrp.c + mvrp.c into one translation unit; the sectioning is the
 * boundary):
 *
 *   §1–§5  Generic MRP state machines (Applicant, Registrar,
 *          LeaveAll, PeriodicTransmission) and timers, the
 *          3-packed-event codec, and the MRPDU envelope encoder.
 *          Mode-agnostic; the same code drives endpoint and bridge.
 *          Per IEEE 802.1Q-2018 §10.
 *
 *   §6     MSRP application — endpoint origination, listener
 *          bookkeeping, admission, bridge MAP. Per §35.
 *
 *   §7     MVRP application (VLAN registration). Per §11.
 *
 *   §8     MAAP (Multicast Address Acquisition) — co-located here
 *          for historical reasons; uses AVTP transport, not MRP.
 *
 * Renamed from msrp.c in the MRP-refactor. The MSRP application code
 * stays in this file; the generic MRP SMs are being added alongside
 * the existing direct-call MSRP TX/RX path, and the latter is being
 * cut over to the SMs incrementally.
 *
 * Bridge-only admission control (the 75 % link-rate cap per SR class)
 * is excluded from endpoint builds via CONFIG_ESP_AVB_ROLE_BRIDGE.
 */

#include "avb.h"
#include "avbbridge.h"
#include "esp_log.h"
#include "esp_random.h"
#include "esp_timer.h"

/* ===== §1  Generic MRP state machines =====
 *
 * IEEE 802.1Q-2018 §10.7. Per-attribute Applicant + Registrar.
 *
 * The existing direct-call MSRP path (avb_send_msrp_*,
 * avb_process_msrp_*) below in §6 remains the active path until
 * cutover. These SMs are not yet wired into RX/TX.
 *
 * Convention: the [sJ] optional send from observer states (relevant
 * only in point-to-point Ethernet links) is treated as a no-op here.
 * AVB SoftAP, Wi-Fi STA, and shared-media Ethernet are all "shared"
 * media from the MRP point of view; only true p2p links benefit from
 * the optional send.
 */

/* LeaveTimer per §10.7.11. Spec mandates 600 ms < LV < 1.4 s; we use
 * 1 s. Per-attribute, runs on the Registrar side. */
#define MRP_LEAVE_TIMER_US (1000 * 1000)

/* §10.7.7 Table 10-3 — Applicant SM transitions.
 *
 * State naming key: V_=Very anxious, A_=Anxious, Q_=Quiet, L_=Leaving;
 * _O=Observer, _P=Passive, _N=New, _A=Active, _L=Leaving.
 *
 * Action codes: sN=send New, sJ=send JoinIn or JoinMt (resolved at
 * encode time from the Registrar state), sL=send Lv. */
void mrp_applicant_step(mrp_sm_state_t *sm, mrp_event_e ev) {
  mrp_applicant_state_e s = sm->applicant;
  mrp_applicant_state_e next = s;
  mrp_tx_action_e tx = sm->pending_tx;

  switch (ev) {
  /* ----- Local-origin events (from application via mrp_declare_*) ----- */

  case mrp_event_new: /* New! — local declares fresh registration */
    /* From any state → VN (very anxious to send New). */
    next = mrp_applicant_vn;
    break;

  case mrp_event_join: /* Join! — local declares (already-known) attr */
    switch (s) {
    case mrp_applicant_vo: next = mrp_applicant_vp; break;
    case mrp_applicant_la: next = mrp_applicant_vp; break; /* cancel leave */
    case mrp_applicant_lo: next = mrp_applicant_vp; break;
    case mrp_applicant_ao: next = mrp_applicant_ap; break;
    case mrp_applicant_qo: next = mrp_applicant_qp; break;
    default: break; /* VP, VN, AN, AA, QA, AP, QP unchanged */
    }
    break;

  case mrp_event_lv: /* Lv! — local withdraws registration */
    switch (s) {
    case mrp_applicant_vo: next = mrp_applicant_lo; break;
    case mrp_applicant_vp: next = mrp_applicant_vo; break;
    case mrp_applicant_vn: next = mrp_applicant_lo; break;
    case mrp_applicant_an: next = mrp_applicant_lo; break;
    case mrp_applicant_aa: next = mrp_applicant_la; break;
    case mrp_applicant_qa: next = mrp_applicant_la; break;
    case mrp_applicant_ap: next = mrp_applicant_ao; break;
    case mrp_applicant_qp: next = mrp_applicant_qo; break;
    default: break; /* LA, LO, AO, QO unchanged */
    }
    break;

  /* ----- Peer-receive events (decoded from inbound 3pe vectors) ----- */

  case mrp_event_r_new:
    /* rNew is consumed by the Registrar; Applicant is unaffected
     * per §10.7.7 — except in some implementations rNew makes us
     * anxious too, but the standard table says no Applicant change. */
    break;

  case mrp_event_r_join_in: /* peer is registered with us */
    switch (s) {
    case mrp_applicant_vo: next = mrp_applicant_ao; break;
    case mrp_applicant_vp: next = mrp_applicant_ap; break;
    case mrp_applicant_aa: next = mrp_applicant_qa; break;
    case mrp_applicant_ao: next = mrp_applicant_qo; break;
    case mrp_applicant_ap: next = mrp_applicant_qp; break;
    default: break; /* VN/AN/QA/LA/QO/QP/LO unchanged */
    }
    break;

  case mrp_event_r_in: /* peer In (we already see them, no JoinMt) */
    /* Only relevant on point-to-point: AA → QA. Otherwise no-op. */
    if (s == mrp_applicant_aa) {
      next = mrp_applicant_qa;
    }
    break;

  case mrp_event_r_join_mt: /* peer Joined but their Registrar saw empty */
    /* Applicant becomes anxious so we re-declare and they re-register. */
    if (s == mrp_applicant_qa) {
      next = mrp_applicant_aa;
    }
    break;

  case mrp_event_r_mt: /* peer Mt — registration was empty */
    if (s == mrp_applicant_qa) {
      next = mrp_applicant_aa; /* re-declare to populate peer */
    }
    break;

  case mrp_event_r_lv:
  case mrp_event_r_la: /* peer is leaving / asked everyone to re-declare */
    switch (s) {
    case mrp_applicant_vo: next = mrp_applicant_lo; break;
    case mrp_applicant_an: next = mrp_applicant_vn; break;
    case mrp_applicant_aa: next = mrp_applicant_vp; break;
    case mrp_applicant_qa: next = mrp_applicant_vp; break;
    case mrp_applicant_ao: next = mrp_applicant_lo; break;
    case mrp_applicant_qo: next = mrp_applicant_lo; break;
    case mrp_applicant_ap: next = mrp_applicant_vp; break;
    case mrp_applicant_qp: next = mrp_applicant_vp; break;
    case mrp_applicant_lo: next = mrp_applicant_vo; break;
    default: break; /* VP, VN, LA unchanged */
    }
    break;

  /* ----- Timer-driven events ----- */

  case mrp_event_tx: /* JoinTimer fired — encode + transmit pending event */
  case mrp_event_tx_la: /* tx! coincident with LeaveAll TX (same effect for
                         * the Applicant; LeaveAll is handled at envelope
                         * level) */
    switch (s) {
    case mrp_applicant_vp:
      next = mrp_applicant_aa;
      tx = mrp_tx_join; /* [sJ] in tx!; sJ in txLA! — emit either way */
      break;
    case mrp_applicant_vn:
      next = mrp_applicant_an;
      tx = mrp_tx_new;
      break;
    case mrp_applicant_an:
      next = mrp_applicant_qa;
      tx = mrp_tx_new;
      break;
    case mrp_applicant_aa:
      next = mrp_applicant_qa;
      tx = mrp_tx_join;
      break;
    case mrp_applicant_la:
      next = mrp_applicant_vo;
      tx = mrp_tx_leave; /* §10.7.7 LA → VO sL: active leave, definitely TX */
      break;
    case mrp_applicant_lo:
      /* §10.7.7 LO → VO [s]: optional send on shared media. AVB
       * operates on shared Ethernet / Wi-Fi (never point-to-point),
       * so we suppress to avoid noisy observer-state Lv emissions
       * after every LeaveAll cycle. State transition still occurs. */
      next = mrp_applicant_vo;
      break;
    case mrp_applicant_qa:
      /* tx! is a no-op (quiet); txLA! does sJ to re-confirm. */
      if (ev == mrp_event_tx_la) {
        next = mrp_applicant_aa;
        tx = mrp_tx_join;
      }
      break;
    /* Observer states: [sJ] is optional, treat as no-op on shared media. */
    default: break;
    }
    break;

  case mrp_event_periodic: /* PeriodicTransmission timer fired */
    /* Per §10.7.9 — Quiet states re-arm to Anxious to drive re-TX. */
    if (s == mrp_applicant_qa) next = mrp_applicant_aa;
    else if (s == mrp_applicant_qo) next = mrp_applicant_ao;
    else if (s == mrp_applicant_qp) next = mrp_applicant_ap;
    break;

  case mrp_event_leave_timer:
    /* Registrar-only event; no Applicant effect. */
    break;
  }

  sm->applicant = next;
  sm->pending_tx = tx;
}

/* §10.7.8 Table 10-4 — Registrar SM transitions.
 *
 * IN: peer's declaration is currently registered.
 * LV: peer's declaration is being torn down (LeaveTimer running);
 *     reverts to IN if peer re-declares, or to MT on timer expiry.
 * MT: no peer registration. */
void mrp_registrar_step(mrp_sm_state_t *sm, mrp_event_e ev) {
  mrp_registrar_state_e r = sm->registrar;
  mrp_registrar_state_e next = r;
  int64_t leave_timer = sm->leave_timer_us;

  switch (ev) {
  case mrp_event_r_new:
  case mrp_event_r_join_in:
  case mrp_event_r_in:
    /* Peer is here. From any state → IN, cancel LeaveTimer. */
    next = mrp_registrar_in;
    leave_timer = 0;
    break;

  case mrp_event_r_join_mt:
  case mrp_event_r_mt:
    /* Peer is not registered (but their Applicant is alive). Go MT
     * immediately if not currently leaving; if leaving, let the
     * LeaveTimer run out. */
    if (r != mrp_registrar_lv) {
      next = mrp_registrar_mt;
      leave_timer = 0;
    }
    break;

  case mrp_event_r_lv:
  case mrp_event_r_la:
    /* Peer (or LeaveAll) says go away. From IN, enter LV and start
     * the LeaveTimer. From MT, stay MT. From LV, restart the timer. */
    if (r == mrp_registrar_in || r == mrp_registrar_lv) {
      next = mrp_registrar_lv;
      leave_timer = esp_timer_get_time() + MRP_LEAVE_TIMER_US;
    }
    break;

  case mrp_event_leave_timer:
    /* LeaveTimer expired while in LV → go MT. */
    if (r == mrp_registrar_lv) {
      next = mrp_registrar_mt;
      leave_timer = 0;
    }
    break;

  /* Local-origin and tx! events don't affect the Registrar. */
  case mrp_event_new:
  case mrp_event_join:
  case mrp_event_lv:
  case mrp_event_tx:
  case mrp_event_tx_la:
  case mrp_event_periodic:
    break;
  }

  sm->registrar = next;
  sm->leave_timer_us = leave_timer;
}

/* Map a decoded 3pe component (0..5) to the matching peer event.
 * The 3pe encoding (§10.8.2.10) packs three attribute events into one
 * octet: V = e1*36 + e2*6 + e3, where each e is 0..5. */
static mrp_event_e mrp_3pe_to_event(int pe_val) {
  switch (pe_val) {
  case 0: return mrp_event_r_new;
  case 1: return mrp_event_r_join_in;
  case 2: return mrp_event_r_in;
  case 3: return mrp_event_r_join_mt;
  case 4: return mrp_event_r_mt;
  case 5: return mrp_event_r_lv;
  default: return mrp_event_r_mt; /* malformed; treat as no-op-ish */
  }
}

/* Convenience: step both Applicant and Registrar with the same event. */
static void mrp_sm_step(mrp_sm_state_t *sm, mrp_event_e ev) {
  mrp_applicant_step(sm, ev);
  mrp_registrar_step(sm, ev);
}

/* ===== §1b  Per-port MRP timer machinery =====
 *
 * Per-port (not per-attribute) timers per IEEE 802.1Q-2018 §10.7.11.
 * The actual event dispatch into the per-attribute SMs happens
 * during cutover (§6 will iterate its attribute tables when these
 * fire). For now mrp_port_tick() advances the timers and sets the
 * tx-pending / LeaveAll-pending flags; no attribute events fire yet.
 */

#define MRP_JOIN_TIMER_US      (200 * 1000)         /* §10.7.11 JoinTime */
#define MRP_LEAVEALL_TIMER_US  (10 * 1000 * 1000)   /* §10.7.11 LeaveAllTime */
#define MRP_PERIODIC_TIMER_US  (1 * 1000 * 1000)    /* §10.7.9  PeriodicTime */

/* Forward decl — definition in §6c (after the attribute tables). */
void mrp_tx_flush_port(avb_state_s *state, int port);

/* Spec-allowed jitter spread for LeaveAllTime: 10 s ≤ T ≤ 15 s.
 * Returns an absolute monotonic-µs expiry. */
static int64_t mrp_leaveall_next_expiry_us(void) {
  uint32_t jitter_us = (uint32_t)(esp_random() % (5 * 1000 * 1000));
  return esp_timer_get_time() + MRP_LEAVEALL_TIMER_US + jitter_us;
}

/* JoinTimer: random within 0..JoinTime per §10.7.4.3 to avoid collisions
 * on shared media. Returns absolute monotonic-µs expiry. */
static int64_t mrp_join_next_expiry_us(void) {
  uint32_t jitter_us = (uint32_t)(esp_random() % MRP_JOIN_TIMER_US);
  return esp_timer_get_time() + jitter_us;
}

/* Initialize per-port MRP timer state. Called at port bring-up. */
void mrp_port_init(avb_state_s *state, int port) {
  if (port < 0 || port >= CONFIG_ESP_AVB_NUM_PORTS) return;
  avb_port_s *p = &state->port[port];
  p->mrp_join_timer_us = 0; /* disarmed; armed when first sm->pending_tx set */
  p->mrp_leaveall_timer_us = mrp_leaveall_next_expiry_us();
  p->mrp_periodic_timer_us = esp_timer_get_time() + MRP_PERIODIC_TIMER_US;
  p->mrp_leaveall_tx_pending = false;
}

/* Called from avb_periodic / avb main loop on every tick. Fires
 * expired port-scoped timers. Returns true if anything fired. */
bool mrp_port_tick(avb_state_s *state, int port) {
  if (port < 0 || port >= CONFIG_ESP_AVB_NUM_PORTS) return false;
  avb_port_s *p = &state->port[port];
  int64_t now = esp_timer_get_time();
  bool fired = false;

  /* LeaveAllTimer: when fires, mark the next PDU to carry LeaveAll
   * and dispatch rLA into every Applicant/Registrar on this port.
   * The dispatch loop is added in cutover; for now only the timer
   * advances and the flag latches. */
  if (p->mrp_leaveall_timer_us != 0 && now >= p->mrp_leaveall_timer_us) {
    p->mrp_leaveall_tx_pending = true;
    p->mrp_leaveall_timer_us = mrp_leaveall_next_expiry_us();
    fired = true;
    /* TODO (cutover): iterate this port's attribute table and call
     * mrp_applicant_step(..., mrp_event_r_la) and
     * mrp_registrar_step(..., mrp_event_r_la) on each entry. */
  }

  /* PeriodicTimer: every 1 s, drive periodic! into active Applicants
   * to bump them out of Quiet states. */
  if (p->mrp_periodic_timer_us != 0 && now >= p->mrp_periodic_timer_us) {
    p->mrp_periodic_timer_us = now + MRP_PERIODIC_TIMER_US;
    fired = true;
    /* TODO (cutover): for each attribute on this port,
     * mrp_applicant_step(..., mrp_event_periodic). */
  }

  /* JoinTimer: armed when a state-change leaves an Applicant with a
   * non-none pending_tx. When fires, build + transmit a single MRPDU
   * per attribute on this port collecting all pending events. */
  if (p->mrp_join_timer_us != 0 && now >= p->mrp_join_timer_us) {
    p->mrp_join_timer_us = 0; /* disarm; rearm when next state change */
    fired = true;
    mrp_tx_flush_port(state, port);
  }

  return fired;
}

/* Arm the JoinTimer on `port` if not already armed. Called whenever
 * a per-attribute SM transitions to a state with pending TX. */
void mrp_port_arm_join_timer(avb_state_s *state, int port) {
  if (port < 0 || port >= CONFIG_ESP_AVB_NUM_PORTS) return;
  avb_port_s *p = &state->port[port];
  if (p->mrp_join_timer_us == 0) {
    p->mrp_join_timer_us = mrp_join_next_expiry_us();
  }
}

/* ===== §6  MSRP application =====
 *
 * Existing direct-call path (avb_send_msrp_* / avb_process_msrp_*)
 * is below. Above that: SM-driven attribute tables, populated by
 * mrp_declare_* and mrp_rx() once the cutover happens. The two paths
 * coexist; cutover replaces the direct-call origination sites in
 * avb.c and the RX dispatch in avbnet.c.
 */

/* Per-port attribute tables. Linear-scan find/insert; tables are
 * small (≤10 entries) so the O(N) cost is fine. */
#define MSRP_TALKER_TABLE_SIZE                                                 \
  (AVB_MAX_NUM_TALKERS + AVB_MAX_NUM_OUTPUT_STREAMS)
#define MSRP_LISTENER_TABLE_SIZE                                               \
  (AVB_MAX_NUM_LISTENERS + AVB_MAX_NUM_INPUT_STREAMS)
#define MSRP_DOMAIN_TABLE_SIZE 2 /* Class A and Class B */

typedef struct {
  bool valid;
  /* attr_type discriminates TALKER_ADVERTISE vs TALKER_FAILED.
   * wire.header.attr_type holds the same value; replicated here for
   * clarity. */
  msrp_attr_type_t attr_type;
  msrp_talker_message_u wire;
  mrp_sm_state_t sm;
} msrp_talker_entry_t;

typedef struct {
  bool valid;
  msrp_listener_message_s wire;
  msrp_listener_event_t decl_event; /* asking_failed / ready / ready_failed */
  mrp_sm_state_t sm;
} msrp_listener_entry_t;

typedef struct {
  bool valid;
  msrp_domain_message_s wire;
  mrp_sm_state_t sm;
} msrp_domain_entry_t;

static msrp_talker_entry_t
    s_msrp_talkers[CONFIG_ESP_AVB_NUM_PORTS][MSRP_TALKER_TABLE_SIZE];
static msrp_listener_entry_t
    s_msrp_listeners[CONFIG_ESP_AVB_NUM_PORTS][MSRP_LISTENER_TABLE_SIZE];
static msrp_domain_entry_t
    s_msrp_domains[CONFIG_ESP_AVB_NUM_PORTS][MSRP_DOMAIN_TABLE_SIZE];

static bool stream_id_eq(const unique_id_t *a, const unique_id_t *b) {
  return memcmp(a, b, sizeof(unique_id_t)) == 0;
}

/* Talker entries are keyed by (stream_id, attr_type) — TALKER_ADVERTISE
 * and TALKER_FAILED for the same stream are distinct MRP attributes
 * with independent Applicant/Registrar pairs per §10.2. */
static msrp_talker_entry_t *msrp_talker_find(int port,
                                             const unique_id_t *stream_id,
                                             msrp_attr_type_t attr_type) {
  if (port < 0 || port >= CONFIG_ESP_AVB_NUM_PORTS) return NULL;
  for (int i = 0; i < MSRP_TALKER_TABLE_SIZE; i++) {
    msrp_talker_entry_t *e = &s_msrp_talkers[port][i];
    if (e->valid && e->attr_type == attr_type &&
        stream_id_eq((const unique_id_t *)&e->wire.talker.info.stream_id,
                     stream_id)) {
      return e;
    }
  }
  return NULL;
}

static msrp_talker_entry_t *
msrp_talker_find_or_insert(int port, const unique_id_t *stream_id,
                           msrp_attr_type_t attr_type) {
  msrp_talker_entry_t *e = msrp_talker_find(port, stream_id, attr_type);
  if (e != NULL) return e;
  if (port < 0 || port >= CONFIG_ESP_AVB_NUM_PORTS) return NULL;
  for (int i = 0; i < MSRP_TALKER_TABLE_SIZE; i++) {
    msrp_talker_entry_t *slot = &s_msrp_talkers[port][i];
    if (!slot->valid) {
      memset(slot, 0, sizeof(*slot));
      slot->valid = true;
      slot->attr_type = attr_type;
      memcpy(&slot->wire.talker.info.stream_id, stream_id,
             sizeof(unique_id_t));
      slot->sm.applicant = mrp_applicant_vo;
      slot->sm.registrar = mrp_registrar_mt;
      slot->sm.pending_tx = mrp_tx_none;
      return slot;
    }
  }
  return NULL; /* table full */
}

static msrp_listener_entry_t *msrp_listener_find(int port,
                                                 const unique_id_t *stream_id) {
  if (port < 0 || port >= CONFIG_ESP_AVB_NUM_PORTS) return NULL;
  for (int i = 0; i < MSRP_LISTENER_TABLE_SIZE; i++) {
    msrp_listener_entry_t *e = &s_msrp_listeners[port][i];
    if (e->valid &&
        stream_id_eq((const unique_id_t *)&e->wire.stream_id, stream_id)) {
      return e;
    }
  }
  return NULL;
}

static msrp_listener_entry_t *
msrp_listener_find_or_insert(int port, const unique_id_t *stream_id) {
  msrp_listener_entry_t *e = msrp_listener_find(port, stream_id);
  if (e != NULL) return e;
  if (port < 0 || port >= CONFIG_ESP_AVB_NUM_PORTS) return NULL;
  for (int i = 0; i < MSRP_LISTENER_TABLE_SIZE; i++) {
    msrp_listener_entry_t *slot = &s_msrp_listeners[port][i];
    if (!slot->valid) {
      memset(slot, 0, sizeof(*slot));
      slot->valid = true;
      memcpy(&slot->wire.stream_id, stream_id, sizeof(unique_id_t));
      slot->sm.applicant = mrp_applicant_vo;
      slot->sm.registrar = mrp_registrar_mt;
      slot->sm.pending_tx = mrp_tx_none;
      return slot;
    }
  }
  return NULL;
}

static msrp_domain_entry_t *msrp_domain_find_or_insert(int port,
                                                       uint8_t sr_class_id) {
  if (port < 0 || port >= CONFIG_ESP_AVB_NUM_PORTS) return NULL;
  for (int i = 0; i < MSRP_DOMAIN_TABLE_SIZE; i++) {
    msrp_domain_entry_t *e = &s_msrp_domains[port][i];
    if (e->valid && e->wire.sr_class_id == sr_class_id) return e;
  }
  for (int i = 0; i < MSRP_DOMAIN_TABLE_SIZE; i++) {
    msrp_domain_entry_t *slot = &s_msrp_domains[port][i];
    if (!slot->valid) {
      memset(slot, 0, sizeof(*slot));
      slot->valid = true;
      slot->wire.sr_class_id = sr_class_id;
      slot->sm.applicant = mrp_applicant_vo;
      slot->sm.registrar = mrp_registrar_mt;
      slot->sm.pending_tx = mrp_tx_none;
      return slot;
    }
  }
  return NULL;
}

/* ----- §6a  SM-driven RX entry point -----
 *
 * Walks an MSRP message buffer, dispatches peer events into the
 * matching per-attribute SMs. Not yet wired into the RX path; the
 * existing avb_process_msrp_* path below remains active until
 * cutover. */

/* Dispatch rLA to every SM on this port. Called when a received
 * MRPDU carries the LeaveAll bit. */
static void msrp_dispatch_leaveall(int port) {
  for (int i = 0; i < MSRP_TALKER_TABLE_SIZE; i++) {
    if (s_msrp_talkers[port][i].valid) {
      mrp_sm_step(&s_msrp_talkers[port][i].sm, mrp_event_r_la);
    }
  }
  for (int i = 0; i < MSRP_LISTENER_TABLE_SIZE; i++) {
    if (s_msrp_listeners[port][i].valid) {
      mrp_sm_step(&s_msrp_listeners[port][i].sm, mrp_event_r_la);
    }
  }
  for (int i = 0; i < MSRP_DOMAIN_TABLE_SIZE; i++) {
    if (s_msrp_domains[port][i].valid) {
      mrp_sm_step(&s_msrp_domains[port][i].sm, mrp_event_r_la);
    }
  }
}

/* Process one TALKER_ADVERTISE or TALKER_FAILED attribute, including
 * its 3pe event vector. */
static void msrp_rx_talker_attr(int port, msrp_attr_type_t attr_type,
                                const msrp_talker_message_u *wire) {
  msrp_talker_entry_t *e = msrp_talker_find_or_insert(
      port, (const unique_id_t *)&wire->talker.info.stream_id, attr_type);
  if (e == NULL) return; /* table full */
  memcpy(&e->wire, wire, sizeof(*wire));
  /* Decode the 3pe vector. event_data[] is at the same offset in
   * both talker_adv and talker_failed (after the per-type info
   * block); since both share the union, indexing via .talker works
   * regardless of which discriminant is in use. */
  int e1, e2, e3;
  three_pe_to_int(wire->talker.event_data[0], &e1, &e2, &e3);
  /* For MSRP, num_vals is almost always 1, so only the first event
   * in the first 3pe slot is meaningful. The remaining e2/e3 carry
   * 6 (no-op) per the spec when num_vals < 3. */
  mrp_sm_step(&e->sm, mrp_3pe_to_event(e1));
}

/* Process one LISTENER attribute. */
static void msrp_rx_listener_attr(int port, const msrp_listener_message_s *wire) {
  msrp_listener_entry_t *e = msrp_listener_find_or_insert(
      port, (const unique_id_t *)&wire->stream_id);
  if (e == NULL) return;
  memcpy(&e->wire, wire, sizeof(*wire));
  /* event_decl_data[0] carries both the 3pe attribute event and the
   * 4pe listener-declaration nibble. Decode the attribute event. */
  int e1, e2, e3;
  three_pe_to_int(wire->event_decl_data[0].event, &e1, &e2, &e3);
  mrp_sm_step(&e->sm, mrp_3pe_to_event(e1));
  /* The listener-declaration (asking_failed / ready / ready_failed)
   * lives in event_decl_data[1].declaration per the spec; cutover
   * will copy it into e->decl_event. */
}

/* Process one DOMAIN attribute. */
static void msrp_rx_domain_attr(int port, const msrp_domain_message_s *wire) {
  msrp_domain_entry_t *e = msrp_domain_find_or_insert(port, wire->sr_class_id);
  if (e == NULL) return;
  memcpy(&e->wire, wire, sizeof(*wire));
  int e1, e2, e3;
  three_pe_to_int(wire->attr_event[0], &e1, &e2, &e3);
  mrp_sm_step(&e->sm, mrp_3pe_to_event(e1));
}

/* Walk an inbound MSRP buffer and dispatch SM events + run the
 * legacy application bookkeeping (avb_process_msrp_*) for each
 * attribute. This is the single MSRP RX entry point — the legacy
 * dispatch loop in avb_process_rx_message is gone. */
void mrp_rx_msrp(avb_state_s *state, int port, msrp_msgbuf_s *msg,
                 size_t length, eth_addr_t *src_addr) {
  (void)length;
  if (port < 0 || port >= CONFIG_ESP_AVB_NUM_PORTS) return;

  int offset = 0;
  msrp_attr_header_s header;
  bool leaveall_dispatched = false;

  while ((size_t)offset + sizeof(msrp_attr_header_s) <=
         sizeof(msg->messages_raw)) {
    memcpy(&header, &msg->messages_raw[offset], sizeof(msrp_attr_header_s));
    if (header.attr_type == msrp_attr_type_none) break;

    size_t attr_size = octets_to_uint(header.attr_list_len, 2) + 4;

    /* LeaveAll bit fires once per PDU, applies to all SMs on this port. */
    if (header.vechead_leaveall && !leaveall_dispatched) {
      msrp_dispatch_leaveall(port);
      leaveall_dispatched = true;
    }

    switch (header.attr_type) {
    case msrp_attr_type_domain: {
      msrp_domain_message_s wire;
      memcpy(&wire, &msg->messages_raw[offset], sizeof(wire));
      msrp_rx_domain_attr(port, &wire);
      avb_process_msrp_domain(state, msg, offset, attr_size);
      break;
    }
    case msrp_attr_type_talker_advertise:
    case msrp_attr_type_talker_failed: {
      msrp_talker_message_u wire;
      memcpy(&wire, &msg->messages_raw[offset], sizeof(wire));
      msrp_rx_talker_attr(port, header.attr_type, &wire);
      avb_process_msrp_talker(
          state, msg, offset, attr_size,
          header.attr_type == msrp_attr_type_talker_failed, src_addr);
      break;
    }
    case msrp_attr_type_listener: {
      msrp_listener_message_s wire;
      memcpy(&wire, &msg->messages_raw[offset], sizeof(wire));
      msrp_rx_listener_attr(port, &wire);
      avb_process_msrp_listener(state, msg, offset, attr_size, src_addr);
      break;
    }
    default:
      break; /* unknown attr — skip */
    }
    offset += attr_size;
  }
}

/* ----- §6b  SM-driven origination entry points -----
 *
 * Called by avb.c when local stream state changes (talker starts,
 * listener subscribes, etc.). Drives the matching Applicant via
 * the appropriate local-origin event and arms the JoinTimer. */

/* Build the TALKER ADVERTISE wire struct from individual fields the
 * way the legacy avb_send_msrp_talker does. Used by both mrp_declare_
 * entry points; the SM-driven TX flush re-uses these fields verbatim
 * when assembling the outbound MRPDU. */
static void mrp_build_talker_info(avb_state_s *state, talker_adv_info_s *info,
                                  const unique_id_t *stream_id,
                                  const eth_addr_t *stream_dest_addr,
                                  const uint8_t *vlan_id,
                                  uint16_t max_frame_size) {
  memcpy(info->stream_id, stream_id, UNIQUE_ID_LEN);
  memcpy(info->stream_dest_addr, stream_dest_addr, ETH_ADDR_LEN);
  memcpy(info->vlan_id, vlan_id, 2);
  int tspec_mfs = max_frame_size;
  int_to_octets(&tspec_mfs, info->tspec_max_frame_size, 2);
  int tspec_mfi = 1;
  int_to_octets(&tspec_mfi, info->tspec_max_frame_interval, 2);
  uint16_t mapping_index = 0;
  for (uint16_t i = 0; i < state->msrp_mappings_count; i++) {
    if (memcmp(vlan_id, state->msrp_mappings[i].vlan_id, 2) == 0) {
      mapping_index = i;
      break;
    }
  }
  info->priority = state->msrp_mappings[mapping_index].priority;
  info->rank = 1;
  int accumulated_latency = 15000;
  int_to_octets(&accumulated_latency, info->accumulated_latency, 4);
}

/* If the Applicant is already actively declaring (non-observer state),
 * a repeated call is treated as a refresh (Join!), not a fresh New!.
 * This matches the legacy direct-call path's periodic re-tx behavior
 * without resetting the SM's anxious/quiet cycle. */
static mrp_event_e mrp_declare_event(const mrp_sm_state_t *sm) {
  switch (sm->applicant) {
  case mrp_applicant_vo:
  case mrp_applicant_ao:
  case mrp_applicant_qo:
  case mrp_applicant_lo:
    return mrp_event_new;
  default:
    return mrp_event_join;
  }
}

void mrp_declare_talker_advertise(avb_state_s *state, int port,
                                  const unique_id_t *stream_id,
                                  const eth_addr_t *stream_dest_addr,
                                  const uint8_t *vlan_id,
                                  uint16_t max_frame_size) {
  msrp_talker_entry_t *e = msrp_talker_find_or_insert(
      port, stream_id, msrp_attr_type_talker_advertise);
  if (e == NULL) return;
  memset(&e->wire, 0, sizeof(e->wire));
  mrp_build_talker_info(state, &e->wire.talker.info, stream_id,
                        stream_dest_addr, vlan_id, max_frame_size);
  mrp_applicant_step(&e->sm, mrp_declare_event(&e->sm));
  mrp_port_arm_join_timer(state, port);
}

void mrp_declare_talker_failed(avb_state_s *state, int port,
                               const unique_id_t *stream_id,
                               const eth_addr_t *stream_dest_addr,
                               const uint8_t *vlan_id,
                               uint16_t max_frame_size,
                               uint8_t failure_code) {
  msrp_talker_entry_t *e = msrp_talker_find_or_insert(
      port, stream_id, msrp_attr_type_talker_failed);
  if (e == NULL) return;
  memset(&e->wire, 0, sizeof(e->wire));
  mrp_build_talker_info(state, &e->wire.talker_failed.info, stream_id,
                        stream_dest_addr, vlan_id, max_frame_size);
  e->wire.talker_failed.failure_code = failure_code;
  /* failure_bridge_id is left zero — the bridge will populate it when
   * it generates a real TALKER_FAILED. */
  mrp_applicant_step(&e->sm, mrp_declare_event(&e->sm));
  mrp_port_arm_join_timer(state, port);
}

void mrp_declare_listener(avb_state_s *state, int port,
                          const unique_id_t *stream_id,
                          msrp_listener_event_t decl) {
  msrp_listener_entry_t *e = msrp_listener_find_or_insert(port, stream_id);
  if (e == NULL) return;
  e->decl_event = decl;
  mrp_applicant_step(&e->sm, mrp_declare_event(&e->sm));
  mrp_port_arm_join_timer(state, port);
}

void mrp_declare_domain(avb_state_s *state, int port, uint8_t sr_class_id,
                        uint8_t sr_class_priority, const uint8_t *sr_class_vid) {
  msrp_domain_entry_t *e = msrp_domain_find_or_insert(port, sr_class_id);
  if (e == NULL) return;
  e->wire.sr_class_id = sr_class_id;
  e->wire.sr_class_priority = sr_class_priority;
  memcpy(e->wire.sr_class_vid, sr_class_vid, 2);
  mrp_applicant_step(&e->sm, mrp_declare_event(&e->sm));
  mrp_port_arm_join_timer(state, port);
}

/* Withdraw applies to both TALKER_ADVERTISE and TALKER_FAILED entries
 * for the given stream_id — the caller doesn't need to track which
 * form is currently declared. */
void mrp_withdraw_talker(avb_state_s *state, int port,
                         const unique_id_t *stream_id) {
  bool armed = false;
  msrp_talker_entry_t *e =
      msrp_talker_find(port, stream_id, msrp_attr_type_talker_advertise);
  if (e != NULL) {
    mrp_applicant_step(&e->sm, mrp_event_lv);
    armed = true;
  }
  e = msrp_talker_find(port, stream_id, msrp_attr_type_talker_failed);
  if (e != NULL) {
    mrp_applicant_step(&e->sm, mrp_event_lv);
    armed = true;
  }
  if (armed) mrp_port_arm_join_timer(state, port);
}

void mrp_withdraw_listener(avb_state_s *state, int port,
                           const unique_id_t *stream_id) {
  msrp_listener_entry_t *e = msrp_listener_find(port, stream_id);
  if (e == NULL) return;
  mrp_applicant_step(&e->sm, mrp_event_lv);
  mrp_port_arm_join_timer(state, port);
}

/* ----- §6c  SM-driven TX flush -----
 *
 * Called from mrp_port_tick() when the JoinTimer expires. Walks each
 * attribute table, fires tx! into the Applicant SMs, and emits any
 * resulting pending TX actions as single-attribute MRPDUs.
 *
 * Cutover staging: while s_mrp_tx_shadow is true, the flush helpers
 * log what they would TX instead of calling avb_net_send. Flipped to
 * false after shadow-mode validation passed on ESP2 (2026-05-12);
 * SMs now actually transmit. With the legacy avb_send_msrp_* periodic
 * calls still in place this means doubled MSRP traffic — intentional
 * to let us compare wire frame counts. The next cutover step removes
 * the legacy calls. */
static bool s_mrp_tx_shadow = false;

/* Forward decl — definition is in the existing direct-call section. */
static int avb_send_msrp_attr(avb_state_s *state, void *attr,
                              int attr_list_len, const char *label);

static const char *mrp_tx_action_name(mrp_tx_action_e a) {
  switch (a) {
  case mrp_tx_new: return "sN";
  case mrp_tx_join: return "sJ";
  case mrp_tx_leave: return "sL";
  default: return "?";
  }
}

/* Resolve an Applicant TX action to a concrete 3pe event value (0..5)
 * given the current Registrar state. sJ → JoinIn (1) if Registrar IN,
 * JoinMt (3) otherwise; sN → New (0); sL → Lv (5). */
static int mrp_resolve_3pe(mrp_tx_action_e action,
                           mrp_registrar_state_e reg) {
  switch (action) {
  case mrp_tx_new: return 0; /* New */
  case mrp_tx_join: return (reg == mrp_registrar_in) ? 1 : 3; /* JoinIn/JoinMt */
  case mrp_tx_leave: return 5; /* Lv */
  default: return -1;
  }
}

static void mrp_tx_flush_talker(avb_state_s *state, int port,
                                msrp_talker_entry_t *e, bool leaveall) {
  msrp_talker_message_u msg;
  msg = e->wire;
  int attr_list_len;
  if (e->attr_type == msrp_attr_type_talker_advertise) {
    msg.header.attr_type = msrp_attr_type_talker_advertise;
    msg.header.attr_len = 25;
    attr_list_len = 29;
  } else {
    msg.header.attr_type = msrp_attr_type_talker_failed;
    msg.header.attr_len = 34;
    attr_list_len = 38;
  }
  int_to_octets(&attr_list_len, msg.header.attr_list_len, 2);
  msg.header.vechead_leaveall = leaveall ? 1 : 0;
  msg.header.vechead_num_vals = 1;
  int pe = mrp_resolve_3pe(e->sm.pending_tx, e->sm.registrar);
  if (pe < 0) return;
  if (e->attr_type == msrp_attr_type_talker_advertise) {
    msg.talker.event_data[0] = int_to_3pe(pe, 0, 0);
  } else {
    msg.talker_failed.event_data[0] = int_to_3pe(pe, 0, 0);
  }
  if (s_mrp_tx_shadow) {
    avbinfo("mrp shadow: would TX talker_%s %s (port %d, pe=%d)",
            (e->attr_type == msrp_attr_type_talker_advertise) ? "adv" : "failed",
            mrp_tx_action_name(e->sm.pending_tx), port, pe);
    return;
  }
  avb_send_msrp_attr(state, &msg, attr_list_len,
                     (e->attr_type == msrp_attr_type_talker_advertise)
                         ? "talker advertise"
                         : "talker failed");
}

static void mrp_tx_flush_listener(avb_state_s *state, int port,
                                  msrp_listener_entry_t *e, bool leaveall) {
  msrp_listener_message_s msg;
  msg = e->wire;
  msg.header.attr_type = msrp_attr_type_listener;
  msg.header.attr_len = 8; /* sizeof(stream_id) */
  int attr_list_len = 14;
  int_to_octets(&attr_list_len, msg.header.attr_list_len, 2);
  msg.header.vechead_leaveall = leaveall ? 1 : 0;
  msg.header.vechead_num_vals = 1;
  int pe = mrp_resolve_3pe(e->sm.pending_tx, e->sm.registrar);
  if (pe < 0) return;
  /* event_decl_data[0]: 3pe event in the .event byte; the 4pe
   * listener declaration is packed into .declaration of slot 1. */
  msg.event_decl_data[0].event = int_to_3pe(pe, 0, 0);
  msg.event_decl_data[1].declaration.event1 = e->decl_event;
  msg.event_decl_data[1].declaration.event2 = 0;
  msg.event_decl_data[1].declaration.event3 = 0;
  msg.event_decl_data[1].declaration.event4 = 0;
  if (s_mrp_tx_shadow) {
    avbinfo("mrp shadow: would TX listener %s (port %d, pe=%d, decl=%d)",
            mrp_tx_action_name(e->sm.pending_tx), port, pe, e->decl_event);
    return;
  }
  avb_send_msrp_attr(state, &msg, attr_list_len, "listener");
}

static void mrp_tx_flush_domain(avb_state_s *state, int port,
                                msrp_domain_entry_t *e, bool leaveall) {
  msrp_domain_message_s msg;
  msg = e->wire;
  msg.header.attr_type = msrp_attr_type_domain;
  msg.header.attr_len = 4; /* class_id + priority + vid */
  int attr_list_len = 9;
  int_to_octets(&attr_list_len, msg.header.attr_list_len, 2);
  msg.header.vechead_leaveall = leaveall ? 1 : 0;
  msg.header.vechead_num_vals = 1;
  int pe = mrp_resolve_3pe(e->sm.pending_tx, e->sm.registrar);
  if (pe < 0) return;
  msg.attr_event[0] = int_to_3pe(pe, 0, 0);
  if (s_mrp_tx_shadow) {
    avbinfo("mrp shadow: would TX domain %s (port %d, pe=%d, class=%u)",
            mrp_tx_action_name(e->sm.pending_tx), port, pe, e->wire.sr_class_id);
    return;
  }
  avb_send_msrp_attr(state, &msg, attr_list_len, "domain");
}

/* Walks all per-attribute SMs on this port, fires tx! to drive them
 * through Applicant transitions, and emits PDUs for those that
 * resolve to a non-none TX action. */
void mrp_tx_flush_port(avb_state_s *state, int port) {
  if (port < 0 || port >= CONFIG_ESP_AVB_NUM_PORTS) return;
  avb_port_s *p = &state->port[port];
  mrp_event_e tx_ev = p->mrp_leaveall_tx_pending ? mrp_event_tx_la
                                                 : mrp_event_tx;
  bool leaveall = p->mrp_leaveall_tx_pending;

  for (int i = 0; i < MSRP_TALKER_TABLE_SIZE; i++) {
    msrp_talker_entry_t *e = &s_msrp_talkers[port][i];
    if (!e->valid) continue;
    mrp_applicant_step(&e->sm, tx_ev);
    if (e->sm.pending_tx != mrp_tx_none) {
      mrp_tx_flush_talker(state, port, e, leaveall);
      e->sm.pending_tx = mrp_tx_none;
    }
  }
  for (int i = 0; i < MSRP_LISTENER_TABLE_SIZE; i++) {
    msrp_listener_entry_t *e = &s_msrp_listeners[port][i];
    if (!e->valid) continue;
    mrp_applicant_step(&e->sm, tx_ev);
    if (e->sm.pending_tx != mrp_tx_none) {
      mrp_tx_flush_listener(state, port, e, leaveall);
      e->sm.pending_tx = mrp_tx_none;
    }
  }
  for (int i = 0; i < MSRP_DOMAIN_TABLE_SIZE; i++) {
    msrp_domain_entry_t *e = &s_msrp_domains[port][i];
    if (!e->valid) continue;
    mrp_applicant_step(&e->sm, tx_ev);
    if (e->sm.pending_tx != mrp_tx_none) {
      mrp_tx_flush_domain(state, port, e, leaveall);
      e->sm.pending_tx = mrp_tx_none;
    }
  }

  p->mrp_leaveall_tx_pending = false;
}

/* SRP class-mapping helpers. SR Class A = mapping index 0 (sr_class_id 6),
 * SR Class B = mapping index 1 (sr_class_id 5). The class_id values
 * match IEEE 802.1Q-2018 Table 35-7. */

static uint16_t avb_msrp_mapping_index_for_class(bool class_b) {
  return class_b ? 1 : 0;
}

static uint8_t avb_msrp_class_id_for_index(uint16_t mapping_index) {
  return mapping_index == 1 ? 5 : 6;
}

static uint16_t avb_msrp_mapping_index_for_class_id(uint8_t class_id) {
  return class_id == 5 ? 1 : 0;
}

/* CIP SFC → sample rate, used by both the AVTP talker (avtp.c) and the
 * MSRP TALKER advertise (this file) for bandwidth calculation.
 * Declared in avb.h so avtp.c can also call it. */
uint32_t cip_sfc_to_sample_rate(uint8_t sfc) {
  switch (sfc) {
  case cip_sfc_sample_rate_32k:
    return 32000;
  case cip_sfc_sample_rate_44_1k:
    return 44100;
  case cip_sfc_sample_rate_48k:
    return 48000;
  case cip_sfc_sample_rate_88_2k:
    return 88200;
  case cip_sfc_sample_rate_96k:
    return 96000;
  case cip_sfc_sample_rate_176_4k:
    return 176400;
  case cip_sfc_sample_rate_192k:
    return 192000;
  default:
    return 48000;
  }
}

int avb_send_mvrp_vlan_id(avb_state_s *state, mrp_attr_event_t attr_event,
                          bool leave_all) {
  mvrp_vlan_id_message_s msg;
  struct timespec ts;
  int ret;
  uint16_t mapping_index = avb_msrp_mapping_index_for_class(false);
  memset(&msg, 0, sizeof(msg));

  // Populate the message
  msg.protocol_ver = 0;
  msg.header.attr_type = mvrp_attr_type_vlan_identifier;
  msg.header.attr_len = 2;
  msg.header.vechead_leaveall = leave_all;
  msg.header.vechead_padding = 0;
  msg.header.vechead_num_vals = 1;
  memcpy(msg.vlan_id, state->msrp_mappings[mapping_index].vlan_id, 2);
  msg.attr_event[0] = int_to_3pe(attr_event, 0, 0);
  uint16_t msg_len =
      8 + 2 + 2; // all of the above + end mark list and end mark msg

  // send the message
  ret = avb_net_send(state, ethertype_mvrp, &msg, msg_len, &ts);
  if (ret < 0) {
    avberr("send MVRP VLAN ID failed: %d", errno);
    return ret;
  }
  return OK;
}



/* Compute the MSRP TSpec MaxFrameSize for an output stream based on its
 * current stream_format. Per IEEE 802.1Qat, MaxFrameSize is the L2 frame
 * length excluding FCS: ETH header + VLAN tag + AVTP header + payload. */
uint16_t avb_compute_tspec_max_frame_size(avb_state_s *state, uint16_t index) {
  avtp_stream_format_s *fmt = &state->output_streams[index].stream_format;
  bool class_b = state->output_streams[index].stream_info_flags.class_b;
  int interval_us = class_b ? 250 : 125;
  int channels, bytes_per_sample, sample_rate, avtp_hdr;
  uint8_t subtype = fmt->aaf_pcm.subtype;
  if (subtype == avtp_subtype_crf) {
    /* CRF: ETH + VLAN + AVTP CRF header + one 64-bit timestamp. */
    return (uint16_t)(14 /*ETH*/ + 4 /*VLAN*/ + 28 /*CRF AVTPDU*/);
  }
  if (subtype == avtp_subtype_61883) {
    channels = fmt->am824.dbs;
    bytes_per_sample = 4; /* AM824 always carries 32-bit quadlets */
    sample_rate = cip_sfc_to_sample_rate(fmt->am824.fdf_sfc);
    avtp_hdr = 24 + 8; /* AVTP common + CIP header */
  } else {
    channels =
        (fmt->aaf_pcm.chan_per_frame_h << 2) | fmt->aaf_pcm.chan_per_frame;
    int bit_depth = fmt->aaf_pcm.bit_depth;
    bytes_per_sample = (bit_depth == 24) ? 4 : (bit_depth / 8);
    sample_rate = aaf_code_to_sample_rate(fmt->aaf_pcm.sample_rate);
    avtp_hdr = 24;
  }
  if (channels <= 0 || sample_rate <= 0)
    return 0;
  int samples_per_packet = sample_rate / (1000000 / interval_us);
  int payload = samples_per_packet * channels * bytes_per_sample;
  return (uint16_t)(14 /*ETH*/ + 4 /*VLAN*/ + avtp_hdr + payload);
}

static int avb_send_msrp_attr(avb_state_s *state, void *attr,
                              int attr_list_len, const char *label) {
  size_t attr_size = 4 + attr_list_len; /* attr hdr w/o vechead + attr list */
  struct timespec ts;
  int ret;

  if (state->avb_lite) {
    return avb_send_cvu_srp_attr(state, attr, attr_list_len, label);
  }

  msrp_msgbuf_s msrp_msg;
  memset(&msrp_msg, 0, sizeof(msrp_msg));
  msrp_msg.protocol_ver = 0;
  memcpy(msrp_msg.messages_raw, attr, attr_size);
  uint16_t msg_len = 5 + attr_list_len + 2; // header + attr_list_len + end mark

  ret = avb_net_send(state, ethertype_msrp, &msrp_msg, msg_len, &ts);
  if (ret < 0) {
    avberr("send MSRP %s failed: %d", label, errno);
  }
  return ret;
}



/* MAAP timing constants (IEEE 1722-2016 Annex B) */
#define MAAP_PROBE_RETRANSMITS 3
#define MAAP_PROBE_INTERVAL_BASE_MS 500
#define MAAP_PROBE_INTERVAL_VAR_MS 100
#define MAAP_ANNOUNCE_INTERVAL_BASE_MS 30000
#define MAAP_ANNOUNCE_INTERVAL_VAR_MS 2000
/* MAAP dynamic allocation pool: 91:E0:F0:00:00:00 to 91:E0:F0:00:FD:FF */
#define MAAP_POOL_SIZE 0xFE00

/* Generate a deterministic address within the MAAP pool from the device
 * MAC. Deterministic across boots is important: switches cache per-
 * stream_id MSRP state, and if our picked Stream DA changed each boot,
 * the switch's stale registration would block forwarding for the new
 * address. MAAP PROBE still resolves cross-device conflicts if two
 * devices happen to hash to the same slot. */
static void maap_generate_addr(avb_state_s *state, eth_addr_t *addr,
                               int stream_index) {
  uint32_t seed = state->port[0].internal_mac_addr[2] ^ state->port[0].internal_mac_addr[3];
  seed = (seed << 8) | state->port[0].internal_mac_addr[4];
  seed = (seed << 8) | state->port[0].internal_mac_addr[5];
  seed += stream_index;
  uint16_t offset = seed % MAAP_POOL_SIZE;
  uint8_t *a = (uint8_t *)addr;
  a[0] = 0x91;
  a[1] = 0xe0;
  a[2] = 0xf0;
  a[3] = 0x00;
  a[4] = (offset >> 8) & 0xFF;
  a[5] = offset & 0xFF;
}

/* Get a random timer jitter value */
static int64_t maap_jitter_us(int base_ms, int var_ms) {
  uint32_t r = (uint32_t)esp_timer_get_time();
  int jitter = (r % (var_ms + 1));
  return (int64_t)(base_ms + jitter) * 1000;
}

/* Check if two single-address ranges conflict */
static bool maap_addr_conflicts(eth_addr_t *a, eth_addr_t *b) {
  return memcmp(a, b, ETH_ADDR_LEN) == 0;
}

/* Send a MAAP message */
int avb_send_maap_msg(avb_state_s *state, maap_msg_type_t msg_type,
                      eth_addr_t *req_addr, uint16_t req_count,
                      eth_addr_t *confl_addr, uint16_t confl_count) {
  maap_message_s msg;
  struct timespec ts;
  memset(&msg, 0, sizeof(msg));

  msg.subtype = avtp_subtype_maap;
  msg.msg_type = msg_type;
  msg.sv = 0;
  msg.version = 0;
  msg.maap_version = 1;
  msg.control_data_len = 16;
  memcpy(msg.req_start_addr, req_addr, ETH_ADDR_LEN);
  int_to_octets(&req_count, msg.req_count, 2);
  if (confl_addr)
    memcpy(msg.confl_start_addr, confl_addr, ETH_ADDR_LEN);
  if (confl_count > 0)
    int_to_octets(&confl_count, msg.confl_count, 2);

  int ret = avb_net_send(state, ethertype_avtp, &msg, sizeof(msg), &ts);
  if (ret < 0) {
    avberr("send MAAP %s failed: %d",
           msg_type == maap_msg_type_probe     ? "probe"
           : msg_type == maap_msg_type_defend   ? "defend"
           : msg_type == maap_msg_type_announce ? "announce"
                                                : "unknown",
           errno);
  }
  return ret;
}

/* Initialize MAAP state for all output streams */
void avb_maap_init(avb_state_s *state) {
  for (int i = 0; i < state->num_output_streams; i++) {
    maap_stream_state_s *m = &state->maap[i];
    m->state = maap_state_initial;
    m->acquired = false;
    maap_generate_addr(state, &m->acquired_addr, i);
    m->probe_count = MAAP_PROBE_RETRANSMITS;
    m->timer_expiry_us =
        esp_timer_get_time() + maap_jitter_us(MAAP_PROBE_INTERVAL_BASE_MS,
                                              MAAP_PROBE_INTERVAL_VAR_MS);
    m->state = maap_state_probe;
    avb_send_maap_msg(state, maap_msg_type_probe, &m->acquired_addr, 1, NULL,
                      0);
    avbinfo("MAAP: probing %02x:%02x:%02x:%02x:%02x:%02x for stream %d",
            m->acquired_addr[0], m->acquired_addr[1], m->acquired_addr[2],
            m->acquired_addr[3], m->acquired_addr[4], m->acquired_addr[5], i);
  }
}

/* MAAP periodic tick — called from AVB main loop.
 * Handles probe retransmits and announce refresh. */
void avb_maap_tick(avb_state_s *state) {
  int64_t now = esp_timer_get_time();

  for (int i = 0; i < state->num_output_streams; i++) {
    maap_stream_state_s *m = &state->maap[i];
    if (now < m->timer_expiry_us)
      continue;

    if (m->state == maap_state_probe) {
      m->probe_count--;
      if (m->probe_count == 0) {
        /* Probing complete — address acquired */
        m->state = maap_state_defend;
        m->acquired = true;
        memcpy(state->output_streams[i].stream_dest_addr, m->acquired_addr,
               ETH_ADDR_LEN);
        avb_send_maap_msg(state, maap_msg_type_announce, &m->acquired_addr, 1,
                          NULL, 0);
        m->timer_expiry_us =
            now + maap_jitter_us(MAAP_ANNOUNCE_INTERVAL_BASE_MS,
                                 MAAP_ANNOUNCE_INTERVAL_VAR_MS);
        avbinfo("MAAP: acquired %02x:%02x:%02x:%02x:%02x:%02x for stream %d",
                m->acquired_addr[0], m->acquired_addr[1], m->acquired_addr[2],
                m->acquired_addr[3], m->acquired_addr[4], m->acquired_addr[5],
                i);
      } else {
        /* Send another probe */
        avb_send_maap_msg(state, maap_msg_type_probe, &m->acquired_addr, 1,
                          NULL, 0);
        m->timer_expiry_us =
            now + maap_jitter_us(MAAP_PROBE_INTERVAL_BASE_MS,
                                 MAAP_PROBE_INTERVAL_VAR_MS);
      }
    } else if (m->state == maap_state_defend) {
      /* Periodic announce refresh */
      avb_send_maap_msg(state, maap_msg_type_announce, &m->acquired_addr, 1,
                        NULL, 0);
      m->timer_expiry_us =
          now + maap_jitter_us(MAAP_ANNOUNCE_INTERVAL_BASE_MS,
                               MAAP_ANNOUNCE_INTERVAL_VAR_MS);
    }
  }
}

/* Process received MSRP domain message */
int avb_process_msrp_domain(avb_state_s *state, msrp_msgbuf_s *msg, int offset,
                            size_t length) {
  msrp_domain_message_s domain;
  memset(&domain, 0, sizeof(domain));
  memcpy(&domain, &msg->messages_raw[offset], sizeof(msrp_domain_message_s));

  uint16_t vid = octets_to_uint(domain.sr_class_vid, 2);
  uint16_t mapping_index =
      avb_msrp_mapping_index_for_class_id(domain.sr_class_id);
  uint16_t our_vid = octets_to_uint(state->msrp_mappings[mapping_index].vlan_id, 2);

  /* Log and validate — the network's SR class must match ours */
  if ((domain.sr_class_id == 6 || domain.sr_class_id == 5) && vid != our_vid) {
    avberr("MSRP domain: class %d network VLAN %d != our VLAN %d",
           domain.sr_class_id, vid, our_vid);
  }
  return OK;
}

/* Process received MSRP talker advertise message */
int avb_process_msrp_talker(avb_state_s *state, msrp_msgbuf_s *msg_data,
                            int offset, size_t length, bool is_failed,
                            eth_addr_t *src_addr) {
  msrp_talker_message_u msg;
  memset(&msg, 0, sizeof(msrp_talker_message_u));
  memcpy(&msg, &msg_data->messages_raw[offset], sizeof(msrp_talker_message_u));

  // get the talker addr from the stream id
  eth_addr_t talker_addr;
  memcpy(&talker_addr, msg.talker.info.stream_id, ETH_ADDR_LEN);

#ifdef CONFIG_ESP_AVB_ROLE_BRIDGE
  /* Bridge admission control. A TALKER_ADVERTISE arriving on one port
   * would be re-declared by the bridge MAP on the other port; before
   * propagating, check that the egress port has the budget for it. If
   * not, the bridge re-declares as TALKER_FAILED with failure_code =
   * insufficient_bandwidth_for_traffic_class (IEEE 802.1Q-2018
   * §35.2.2.8.4 and Table 35-7).
   *
   * is_failed=true means the upstream side already gave up; nothing
   * to admit.
   *
   * TODO: derive ingress port from the L2 forwarder so egress is
   * !ingress instead of always port 1. For now assume worst-case
   * egress — the Wi-Fi port (port 1 when NUM_PORTS=2). */
  if (!is_failed && CONFIG_ESP_AVB_NUM_PORTS > 1) {
    avb_sr_class_e cls =
        (msg.talker.info.priority == 3) ? AVB_SR_CLASS_A : AVB_SR_CLASS_B;
    uint16_t mfs =
        (uint16_t)octets_to_uint(msg.talker.info.tspec_max_frame_size, 2);
    uint16_t mfi = (uint16_t)octets_to_uint(
        msg.talker.info.tspec_max_frame_interval, 2);
    /* Observation interval per SR class:
     *   Class A = 125 µs → 8000 intervals/s
     *   Class B = 250 µs → 4000 intervals/s
     * Bandwidth = max_frame_size × frames_per_interval × intervals/s × 8. */
    uint32_t intervals_per_sec = (cls == AVB_SR_CLASS_A) ? 8000u : 4000u;
    uint32_t request_bps =
        (uint32_t)mfs * (uint32_t)mfi * intervals_per_sec * 8u;

    int egress_port = 1; /* TODO: replace with !ingress_port */
    int adm = avb_srp_admission_try_admit(egress_port, cls, request_bps);
    if (adm < 0) {
      avbwarn("MSRP TALKER over budget on port %d class %s: %u bps; "
              "re-declaring as TALKER_FAILED",
              egress_port, (cls == AVB_SR_CLASS_A) ? "A" : "B",
              (unsigned)request_bps);

      /* Re-declare as TALKER_FAILED on the egress side. Build the
       * full failed message directly so we can set failure_code =
       * insufficient_bandwidth_for_traffic_class (the existing
       * avb_send_msrp_talker hard-codes failure_code = 0). */
      msrp_talker_message_u failed;
      memset(&failed, 0, sizeof(failed));
      failed.header.attr_type = msrp_attr_type_talker_failed;
      failed.header.attr_len = 34;
      int attr_list_len = 38;
      int_to_octets(&attr_list_len, failed.header.attr_list_len, 2);
      failed.header.vechead_num_vals = 1;
      memcpy(&failed.talker_failed.info, &msg.talker.info,
             sizeof(talker_adv_info_s));
      memcpy(failed.talker_failed.failure_bridge_id,
             state->port[0].internal_mac_addr, ETH_ADDR_LEN);
      failed.talker_failed.failure_code =
          insufficient_bandwidth_for_traffic_class;
      failed.talker_failed.event_data[0] = int_to_3pe(mrp_attr_event_new, 0, 0);
      avb_send_msrp_attr(state, &failed, attr_list_len, "Talker Failed");
      return OK;
    }
  }
#endif /* CONFIG_ESP_AVB_ROLE_BRIDGE */

  // If the talker is known then update talker info
  int index =
      avb_find_entity_by_addr(state, &talker_addr, avb_entity_type_talker);
  if (index >= 0) {
    memcpy(&state->talkers[index].info, &msg.talker.info,
           sizeof(talker_adv_info_s));
  }

  /* Propagate the SRP-accumulated latency onto any listener input_stream
   * with a matching stream_id. This is the authoritative end-to-end
   * max_transit_time (talker origination + bridge hops), per
   * IEEE 802.1Qat / Milan §5.4. avb_start_stream_in reads it at connect
   * time to size the presentation-time startup fill.
   * Guard against a zero-stream-id malformed MSRP packet matching an
   * unconnected slot's zeroed stream_id. */
  static const uint8_t zero_stream_id[UNIQUE_ID_LEN] = {0};
  if (memcmp(msg.talker.info.stream_id, zero_stream_id, UNIQUE_ID_LEN) != 0) {
    for (int i = 0; i < AVB_MAX_NUM_INPUT_STREAMS; i++) {
      if (memcmp(state->input_streams[i].stream_id, msg.talker.info.stream_id,
                 UNIQUE_ID_LEN) == 0) {
        memcpy(state->input_streams[i].msrp_accumulated_latency,
               msg.talker.info.accumulated_latency, 4);
        break;
      }
    }
  }
  return OK;
}

/* Find a connected listener by MAC address in a stream's listener list.
 * Returns index or -1 if not found. */
static int find_connected_listener(avb_talker_stream_s *stream,
                                   eth_addr_t *mac_addr) {
  uint16_t count = octets_to_uint(stream->connection_count, 2);
  for (int i = 0; i < count && i < AVB_MAX_NUM_CONNECTED_LISTENERS; i++) {
    if (memcmp(stream->connected_listeners[i].mac_addr, mac_addr,
               ETH_ADDR_LEN) == 0) {
      return i;
    }
  }
  return -1;
}

static int find_connected_listener_by_entity(avb_talker_stream_s *stream,
                                             const unique_id_t entity_id) {
  static const unique_id_t zero_id = {0};
  if (memcmp(entity_id, zero_id, UNIQUE_ID_LEN) == 0)
    return -1;
  uint16_t count = octets_to_uint(stream->connection_count, 2);
  for (int i = 0; i < count && i < AVB_MAX_NUM_CONNECTED_LISTENERS; i++) {
    if (memcmp(stream->connected_listeners[i].identity.id, entity_id,
               UNIQUE_ID_LEN) == 0) {
      return i;
    }
  }
  return -1;
}

/* Add a listener to a stream's connected_listeners list.
 * Returns index of added entry, or -1 if full. */
static int add_connected_listener(avb_talker_stream_s *stream,
                                  eth_addr_t *mac_addr) {
  uint16_t count = octets_to_uint(stream->connection_count, 2);
  if (count >= AVB_MAX_NUM_CONNECTED_LISTENERS)
    return -1;
  memcpy(stream->connected_listeners[count].mac_addr, mac_addr, ETH_ADDR_LEN);
  count++;
  int_to_octets(&count, stream->connection_count, 2);
  return count - 1;
}

/* Resolve/merge an MSRP listener declaration (known by source MAC) with any
 * existing ACMP CONNECT_TX state (known by listener entity_id/uid).  Streaming
 * is allowed only when both halves are present: MSRP Ready + ACMP connected. */
static int find_or_add_msrp_listener(avb_state_s *state,
                                     avb_talker_stream_s *stream,
                                     eth_addr_t *mac_addr) {
  int idx = find_connected_listener(stream, mac_addr);
  if (idx >= 0)
    return idx;

  int listener_idx = avb_find_entity_by_addr(state, mac_addr,
                                             avb_entity_type_listener);
  if (listener_idx != NOT_FOUND) {
    idx = find_connected_listener_by_entity(
        stream, state->listeners[listener_idx].entity_id);
    if (idx >= 0) {
      memcpy(stream->connected_listeners[idx].mac_addr, mac_addr,
             ETH_ADDR_LEN);
      return idx;
    }
  }

  idx = add_connected_listener(stream, mac_addr);
  if (idx >= 0 && listener_idx != NOT_FOUND) {
    memcpy(stream->connected_listeners[idx].identity.id,
           state->listeners[listener_idx].entity_id, UNIQUE_ID_LEN);
    memset(stream->connected_listeners[idx].identity.uid, 0, 2);
  }
  return idx;
}

static bool any_listener_ready(avb_talker_stream_s *stream) {
  uint16_t count = octets_to_uint(stream->connection_count, 2);
  for (int i = 0; i < count && i < AVB_MAX_NUM_CONNECTED_LISTENERS; i++) {
    if (stream->connected_listeners[i].msrp_ready) {
      return true;
    }
  }
  return false;
}

static bool any_listener_acmp_connected(avb_talker_stream_s *stream) {
  uint16_t count = octets_to_uint(stream->connection_count, 2);
  for (int i = 0; i < count && i < AVB_MAX_NUM_CONNECTED_LISTENERS; i++) {
    if (stream->connected_listeners[i].acmp_connected) {
      return true;
    }
  }
  return false;
}


/* Process received MSRP listener message.
 * As a talker, this tells us a listener has declared ready/asking_failed
 * for one of our streams. Drives streaming start/stop decisions. */
int avb_process_msrp_listener(avb_state_s *state, msrp_msgbuf_s *msg,
                              int offset, size_t length,
                              eth_addr_t *src_addr) {
  msrp_listener_message_s listener_msg;
  memset(&listener_msg, 0, sizeof(listener_msg));
  memcpy(&listener_msg, &msg->messages_raw[offset],
         sizeof(msrp_listener_message_s));

  /* Decode the MRP attribute event and listener declaration */
  int attr_event = 0, unused1 = 0, unused2 = 0;
  three_pe_to_int(listener_msg.event_decl_data[0].event, &attr_event, &unused1,
                  &unused2);
  uint8_t listener_decl = listener_msg.event_decl_data[1].declaration.event1;

  /* Check if this listener declaration is for one of our output streams */
  for (int i = 0; i < state->num_output_streams; i++) {
    avb_talker_stream_s *stream = &state->output_streams[i];
    if (memcmp(listener_msg.stream_id, stream->stream_id, UNIQUE_ID_LEN) != 0)
      continue;

    /* Listener leaving — clear MSRP readiness, but keep ACMP identity/state.
     * ACMP DISCONNECT_TX is authoritative for removing the listener entry. */
    if (attr_event == mrp_attr_event_lv) {
      int idx = find_connected_listener(stream, src_addr);
      if (idx >= 0) {
        avbinfo("MSRP: listener leaving stream %d", i);
        stream->connected_listeners[idx].msrp_ready = false;
        stream->connected_listeners[idx].asking_failed = false;
        bool should_stop = !any_listener_ready(stream) ||
                           (!state->config.milan_compliant &&
                            !any_listener_acmp_connected(stream));
        if (should_stop && stream->streaming) {
          avb_stop_stream_out(state, i);
        }
      }
      return OK;
    }

    /* Listener ready — add/merge MSRP state. Only start once ACMP has also
     * connected this listener. This prevents stale periodic MSRP Ready
     * re-declarations from restarting a stream after ACMP disconnect/stop. */
    if (listener_decl == msrp_listener_event_ready) {
      int idx = find_or_add_msrp_listener(state, stream, src_addr);
      if (idx < 0) {
        avberr("MSRP: connected_listeners full for stream %d", i);
        return OK;
      }
      bool was_ready = stream->connected_listeners[idx].msrp_ready;
      stream->connected_listeners[idx].msrp_ready = true;
      if (!was_ready) {
        avbinfo("MSRP: listener ready for stream %d (count=%d, acmp=%d)", i,
                octets_to_uint(stream->connection_count, 2),
                stream->connected_listeners[idx].acmp_connected);
      }
      /* Clear any stale asking_failed state — the listener moved out of
       * that declaration and into Ready. */
      stream->connected_listeners[idx].asking_failed = false;
      /* Plain AVB starts only when both ACMP and MSRP agree that the listener is
       * connected/ready. Milan Talkers do not maintain ACMP listener state;
       * MSRP Listener Ready alone drives streaming. */
      bool should_start = state->config.milan_compliant
                              ? stream->connected_listeners[idx].msrp_ready
                              : stream->connected_listeners[idx].acmp_connected;
      if (should_start && !stream->streaming) {
        avb_start_stream_out(state, i);
      }
      return OK;
    }

    /* Listener asking_failed — track per-listener so ACMP
     * GET_TX_STATE_RESPONSE can set REGISTERING_FAILED per Milan
     * v1.3 Table 5.23. ready_failed is a transient MRP event with
     * the same meaning at the talker side; treat identically. */
    if (listener_decl == msrp_listener_event_asking_failed ||
        listener_decl == msrp_listener_event_ready_failed) {
      int idx = find_or_add_msrp_listener(state, stream, src_addr);
      if (idx < 0) {
        avberr("MSRP: connected_listeners full for stream %d", i);
        return OK;
      }
      if (!stream->connected_listeners[idx].msrp_ready) {
        /* New listener arriving directly in a failed state. Track it, but
         * don't start streaming. */
        stream->connected_listeners[idx].msrp_ready = false;
      }
      stream->connected_listeners[idx].asking_failed = true;
      avbinfo("MSRP: listener %s for stream %d",
              listener_decl == msrp_listener_event_asking_failed
                  ? "asking_failed"
                  : "ready_failed",
              i);
      return OK;
    }
    return OK;
  }

  return OK;
}

/* Process received AVTP IEC 61883 message */
int avb_process_iec_61883(avb_state_s *state, iec_61883_6_message_s *msg) {
  return OK;
}

/* Process received AVTP AAF PCM message */
int avb_process_aaf(avb_state_s *state, aaf_pcm_message_s *msg) {
  return OK;
}

/* Process received AVTP MAAP message */
int avb_process_maap(avb_state_s *state, maap_message_s *msg) {
  for (int i = 0; i < state->num_output_streams; i++) {
    maap_stream_state_s *m = &state->maap[i];

    /* Only handle messages that conflict with our address */
    if (!maap_addr_conflicts(&m->acquired_addr, (eth_addr_t *)msg->req_start_addr))
      continue;

    switch (msg->msg_type) {
    case maap_msg_type_probe:
      /* Someone is probing our address — defend it if we own it */
      if (m->state == maap_state_defend) {
        avbinfo("MAAP: defending stream %d address against probe", i);
        avb_send_maap_msg(state, maap_msg_type_defend,
                          (eth_addr_t *)msg->req_start_addr, 1,
                          &m->acquired_addr, 1);
      } else if (m->state == maap_state_probe) {
        /* We're also probing — lower MAC yields. Compare our MAC vs sender.
         * For simplicity, restart with a new address. */
        avbinfo("MAAP: probe conflict on stream %d, selecting new address", i);
        maap_generate_addr(state, &m->acquired_addr, i);
        m->probe_count = MAAP_PROBE_RETRANSMITS;
        m->timer_expiry_us =
            esp_timer_get_time() +
            maap_jitter_us(MAAP_PROBE_INTERVAL_BASE_MS,
                           MAAP_PROBE_INTERVAL_VAR_MS);
        avb_send_maap_msg(state, maap_msg_type_probe, &m->acquired_addr, 1,
                          NULL, 0);
      }
      break;

    case maap_msg_type_defend:
      /* Our probe was defended — pick a new address */
      if (m->state == maap_state_probe) {
        avbinfo("MAAP: probe defended on stream %d, selecting new address", i);
        maap_generate_addr(state, &m->acquired_addr, i);
        m->probe_count = MAAP_PROBE_RETRANSMITS;
        m->timer_expiry_us =
            esp_timer_get_time() +
            maap_jitter_us(MAAP_PROBE_INTERVAL_BASE_MS,
                           MAAP_PROBE_INTERVAL_VAR_MS);
        avb_send_maap_msg(state, maap_msg_type_probe, &m->acquired_addr, 1,
                          NULL, 0);
      }
      break;

    case maap_msg_type_announce:
      /* Conflicting announce — yield and restart with new address */
      if (m->state == maap_state_defend) {
        avbinfo("MAAP: conflicting announce on stream %d, restarting", i);
        m->acquired = false;
        maap_generate_addr(state, &m->acquired_addr, i);
        m->state = maap_state_probe;
        m->probe_count = MAAP_PROBE_RETRANSMITS;
        m->timer_expiry_us =
            esp_timer_get_time() +
            maap_jitter_us(MAAP_PROBE_INTERVAL_BASE_MS,
                           MAAP_PROBE_INTERVAL_VAR_MS);
        avb_send_maap_msg(state, maap_msg_type_probe, &m->acquired_addr, 1,
                          NULL, 0);
      }
      break;

    default:
      break;
    }
  }
  return OK;
}

/* ======================================================================
 * Bridge-only: MSRP admission control with 75 % link-rate cap per SR class
 *
 * Per IEEE 802.1Q-2018 §35.2.2.8.4, SR Class A is allotted at most
 * 75 % of the port bandwidth, and Class B shares that 75 % budget.
 * The bridge tracks per-port-per-class running bandwidth and rejects
 * a TALKER_ADVERTISE propagation when admitted + new would exceed the
 * cap. Hooked into avb_process_msrp_talker so an over-budget request
 * propagates as TALKER_FAILED with failure_code =
 * insufficient_bandwidth_for_traffic_class (avb.h:407).
 *
 * On Wi-Fi the bridge admits Class B only; Class A reservations
 * propagating from the wired side onto the wireless egress are
 * rejected at admission rather than dropped at forwarding time.
 *
 * Excluded from endpoint builds via CONFIG_ESP_AVB_ROLE_BRIDGE.
 * ====================================================================== */

#ifdef CONFIG_ESP_AVB_ROLE_BRIDGE

static avb_admission_class_s
    s_admission[CONFIG_ESP_AVB_NUM_PORTS][AVB_SR_CLASS_COUNT];

int avb_srp_admission_init(avb_state_s *state) {
  for (int p = 0; p < CONFIG_ESP_AVB_NUM_PORTS; p++) {
    for (int c = 0; c < AVB_SR_CLASS_COUNT; c++) {
      uint32_t link_rate = (state->port[p].medium == avb_port_medium_ethernet)
                               ? 1000000000u
                               : 0u;
      s_admission[p][c].link_rate_bps = link_rate;
      s_admission[p][c].admitted_bps = 0;
      s_admission[p][c].cap_bps = link_rate * 3u / 4u;
    }
  }
  ESP_LOGI("avb_srp", "MSRP admission init; 75%% cap per SR class per port");
  return 0;
}

void avb_srp_admission_stop(avb_state_s *state) { (void)state; }

int avb_srp_admission_try_admit(int port_index, avb_sr_class_e cls,
                                uint32_t request_bps) {
  if (port_index < 0 || port_index >= CONFIG_ESP_AVB_NUM_PORTS) return -1;
  if (cls < 0 || cls >= AVB_SR_CLASS_COUNT) return -1;
  avb_admission_class_s *e = &s_admission[port_index][cls];
  if (e->cap_bps == 0) {
    return -2; /* -ENOSPC: link rate not yet known (Wi-Fi port not up) */
  }
  if (e->admitted_bps + request_bps > e->cap_bps) {
    return -2; /* -ENOSPC: would exceed 75 % cap */
  }
  e->admitted_bps += request_bps;
  /* Push the new aggregate idle_slope into the FQTSS shaper so the
   * credit refill rate matches the admitted bandwidth. */
  avb_fqtss_set_idle_slope(port_index, cls, (int64_t)e->admitted_bps);
  return 0;
}

void avb_srp_admission_release(int port_index, avb_sr_class_e cls,
                               uint32_t request_bps) {
  if (port_index < 0 || port_index >= CONFIG_ESP_AVB_NUM_PORTS) return;
  if (cls < 0 || cls >= AVB_SR_CLASS_COUNT) return;
  avb_admission_class_s *e = &s_admission[port_index][cls];
  if (e->admitted_bps > request_bps) {
    e->admitted_bps -= request_bps;
  } else {
    e->admitted_bps = 0;
  }
  avb_fqtss_set_idle_slope(port_index, cls, (int64_t)e->admitted_bps);
}

#endif /* CONFIG_ESP_AVB_ROLE_BRIDGE */
