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
  case mrp_event_r_join_mt:
  case mrp_event_r_in:
    /* All Join-bearing receive events register the peer per §10.7.8
     * Table 10-4. rJoinIn vs rJoinMt differ only in the *sender's*
     * Registrar state — both encode an active Join action that the
     * receiver must register. rIn is informational (sender Applicant
     * in observer state, but Registrar IN) — register too, since the
     * peer has the attribute. From any state → IN, cancel LeaveTimer. */
    next = mrp_registrar_in;
    leave_timer = 0;
    break;

  case mrp_event_r_mt:
    /* rMt is purely informational: sender's Applicant is in observer
     * state and their Registrar is empty. Not a Join, not a Leave;
     * leave our Registrar where it is. */
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
/* Wi-Fi efficiency cut #2: a LeaveAll burst transmits one MRPDU per
 * attribute on the port and triggers re-declarations on every peer.
 * On Wi-Fi this is expensive (airtime ∝ table size, no concurrency
 * with audio frames). Triple the spec floor to 30 s and widen jitter
 * to 30 s, so the burst lands at 30–60 s instead of 10–15 s. Still
 * well within the spec's permissive upper bound (§10.7.11 allows
 * implementer-chosen LeaveAllTime ≥ 600 ms). */
#define MRP_LEAVEALL_TIMER_WIFI_US     (30 * 1000 * 1000)
#define MRP_LEAVEALL_JITTER_WIFI_US    (30 * 1000 * 1000)

/* Forward decl — definition in §6c (after the attribute tables). */
void mrp_tx_flush_port(avb_state_s *state, int port);

/* Spec-allowed jitter spread for LeaveAllTime: 10 s ≤ T ≤ 15 s on
 * wired ports; 30 s ≤ T ≤ 60 s on Wi-Fi (cut #2 above). Returns an
 * absolute monotonic-µs expiry. */
static int64_t mrp_leaveall_next_expiry_us(const avb_port_s *p) {
  uint32_t base = MRP_LEAVEALL_TIMER_US;
  uint32_t jitter_span = 5 * 1000 * 1000;
  if (p->medium == avb_port_medium_wifi) {
    base = MRP_LEAVEALL_TIMER_WIFI_US;
    jitter_span = MRP_LEAVEALL_JITTER_WIFI_US;
  }
  uint32_t jitter_us = (uint32_t)(esp_random() % jitter_span);
  return esp_timer_get_time() + base + jitter_us;
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
  p->mrp_leaveall_timer_us = mrp_leaveall_next_expiry_us(p);
  p->mrp_periodic_timer_us = esp_timer_get_time() + MRP_PERIODIC_TIMER_US;
  p->mrp_leaveall_tx_pending = false;
}

/* LeaveTimer dispatch helper — implementation after the §6/§7
 * attribute tables it walks. Returns true if any Registrar fired. */
static bool mrp_port_dispatch_leave_timers(int port, int64_t now);

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
   * advances and the flag latches.
   *
   * Wi-Fi efficiency cut #3: on a wifi AP port with zero associated
   * STAs, the LeaveAll burst goes nowhere. Roll the timer forward
   * (so we don't fire instantly when an STA joins) but skip the
   * dispatch. */
  if (p->mrp_leaveall_timer_us != 0 && now >= p->mrp_leaveall_timer_us) {
    bool suppress = false;
#ifdef CONFIG_ESP_AVB_ROLE_BRIDGE
    if (p->medium == avb_port_medium_wifi &&
        p->wifi_mode == avb_port_wifi_mode_ap &&
        avb_bridge_wifi_ap_sta_count() == 0) {
      suppress = true;
    }
#endif
    if (!suppress) {
      p->mrp_leaveall_tx_pending = true;
    }
    p->mrp_leaveall_timer_us = mrp_leaveall_next_expiry_us(p);
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

  /* LeaveTimer dispatch — implementation below in §7 (after the
   * MSRP / MVRP tables are declared). Without this, a peer that
   * goes silent without rLv (yanked cable, crashed talker) holds
   * its IN-state Registrar forever and keeps admission allocated. */
  if (mrp_port_dispatch_leave_timers(port, now)) {
    fired = true;
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
 * Origination via mrp_declare_* drives per-attribute Applicants in
 * the typed tables defined below; RX flows through mrp_rx_msrp and
 * dispatches to the on_*_registrar_change callbacks in §6a. The
 * legacy direct-call avb_send_msrp_* / avb_process_msrp_* surface is
 * gone — these SMs are the only path.
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
  /* MAP-side state, used on the bridge's ingress port only: the
   * wire->priority value we last propagated to other bridged ports.
   * 0 = never propagated; 2 = Class B; 3 = Class A. Lets the MAP code
   * detect a class change mid-life (Registrar stays IN but wire
   * content shifts) and trigger a withdraw + re-declare on egress
   * instead of leaving the egress Applicant stuck on the old class. */
  uint8_t last_propagated_priority;
} msrp_talker_entry_t;

typedef struct {
  bool valid;
  msrp_listener_message_s wire;
  msrp_listener_event_t decl_event; /* asking_failed / ready / ready_failed */
  /* Peer's most recent declaration on this port, populated by
   * msrp_rx_listener_attr. Distinct from decl_event (which records
   * what WE declare on egress) so a bridge can read each port's
   * peer-side decl independently when computing the §35.2.4.4.3
   * merged value for the talker-facing port. */
  msrp_listener_event_t peer_decl;
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

/* Public query: is the TALKER_ADVERTISE Registrar currently IN for
 * this (port, stream_id)? Used by the listener-side decl_event
 * helper so its decision is based on actual SM state, not on a
 * transition-only cached flag (which misses already-IN bind events). */
bool mrp_talker_advertise_active(int port, const unique_id_t *stream_id) {
  msrp_talker_entry_t *e =
      msrp_talker_find(port, stream_id, msrp_attr_type_talker_advertise);
  return e != NULL && e->sm.registrar == mrp_registrar_in;
}

bool mrp_talker_failed_active(int port, const unique_id_t *stream_id,
                              uint8_t *failure_code_out) {
  msrp_talker_entry_t *e =
      msrp_talker_find(port, stream_id, msrp_attr_type_talker_failed);
  if (e == NULL || e->sm.registrar != mrp_registrar_in) {
    return false;
  }
  if (failure_code_out) {
    *failure_code_out = e->wire.talker_failed.failure_code;
  }
  return true;
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
 * matching per-attribute SMs and fires the on_*_registrar_change
 * application callbacks below for both endpoint bookkeeping and
 * bridge MAP propagation. */

/* Registrar transition direction for application/MAP dispatch.
 *
 * §10.7.8 distinguishes IN (registered), LV (peer leave pending,
 * LeaveTimer running) and MT (empty). For MAP propagation the only
 * question is whether the peer's registration is alive *right now*,
 * so IN is "registered" and {LV, MT} are both "not registered." A
 * peer Lv (rLv) that takes the Registrar IN→LV must propagate as a
 * withdraw immediately — waiting for the LeaveTimer to drain to MT
 * would stall downstream tear-down by up to 1 s. */
typedef enum {
  mrp_reg_transition_none = 0,
  mrp_reg_transition_register,    /* not-IN → IN */
  mrp_reg_transition_deregister,  /* IN → not-IN */
} mrp_reg_transition_e;

static inline mrp_reg_transition_e
mrp_reg_transition(mrp_registrar_state_e before,
                   mrp_registrar_state_e after) {
  bool was_in = (before == mrp_registrar_in);
  bool is_in  = (after  == mrp_registrar_in);
  if (!was_in && is_in) return mrp_reg_transition_register;
  if (was_in && !is_in) return mrp_reg_transition_deregister;
  return mrp_reg_transition_none;
}

/* Application transition callbacks. Fire on every MSRP RX; the `tr`
 * argument tells the body whether it's a Registrar register/deregister
 * edge (for MAP propagation + admission accounting, which must run
 * once per edge) or just an in-registration update (for endpoint-side
 * bookkeeping — talker DB refresh, accumulated_latency propagation,
 * listener msrp_ready / asking_failed tracking, which can run per RX).
 *
 * Replaces the legacy per-RX avb_process_msrp_* reactions. The legacy
 * functions are gone; this is the only consumer of inbound MSRP
 * attributes outside the SM itself. */

#ifdef CONFIG_ESP_AVB_ROLE_BRIDGE
/* Forward decls — definitions live in §6b (below) but the callbacks
 * are in §6a (above §6b). */
extern int avb_srp_admission_try_admit(int port_index, avb_sr_class_e cls,
                                       uint32_t request_bps);
extern void avb_srp_admission_release(int port_index, avb_sr_class_e cls,
                                      uint32_t request_bps);
#endif
/* Same forward-decl rationale — used by the domain callback. */
static uint16_t avb_msrp_mapping_index_for_class_id(uint8_t class_id);

/* ----- Listener-side connection-tracking helpers used by the
 * listener callback. Operate on the talker-side output_streams[]
 * table; a peer listener's MSRP declarations drive msrp_ready /
 * asking_failed flags on the per-stream connected_listeners[] list. */

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
 * existing ACMP CONNECT_TX state (known by listener entity_id/uid). Streaming
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

/* §35.2.2.8.6 helper — bridge's own forwarding-latency contribution
 * for one egress port, in nanoseconds. Aggregated onto the inbound
 * accumulated_latency at MAP propagation time. Estimate is worst-
 * case full-MFS serialization at link rate plus a small bridge
 * processing constant; refine when we have real per-egress queue
 * measurements. Wi-Fi gets a much larger constant — variance is
 * dominated by CSMA backoff and beacon-driven scheduling rather
 * than nominal link rate. */
static uint32_t avb_port_forwarding_latency_ns(const avb_port_s *p) {
  const uint32_t bridge_proc_ns = 30000u; /* ~30 µs SoC proc + queue */
  if (p->medium == avb_port_medium_wifi_ftm) {
    return 500000u + bridge_proc_ns; /* ~500 µs Wi-Fi worst case */
  }
  uint32_t link_mbps = p->link_speed_mbps ? p->link_speed_mbps : 100u;
  /* Serialization time = bits / link_rate. 1500 B = 12000 bits.
   * 12000 ns/Mbps × 1000 = 12,000,000 / link_mbps ns per MFS. */
  uint32_t ser_ns = 12000000u / link_mbps;
  return ser_ns + bridge_proc_ns;
}

/* Patch the just-declared egress entry's accumulated_latency to the
 * aggregated value (inbound + bridge contribution). Done AFTER
 * mrp_declare_*_advertise / _failed because mrp_build_talker_info
 * always rebuilds the field from a port-local default; the bridge
 * propagation needs to override with the cumulative path latency
 * per §35.2.2.8.6. */
static void mrp_patch_egress_accumulated_latency(
    int port, const unique_id_t *stream_id, msrp_attr_type_t attr_type,
    uint32_t total_ns) {
  msrp_talker_entry_t *eg = msrp_talker_find(port, stream_id, attr_type);
  if (eg == NULL) return;
  uint8_t *field = (attr_type == msrp_attr_type_talker_failed)
                       ? eg->wire.talker_failed.info.accumulated_latency
                       : eg->wire.talker.info.accumulated_latency;
  int_to_octets(&total_ns, field, 4);
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

static void mrp_on_talker_registrar_change(
    avb_state_s *state, int port, msrp_attr_type_t attr_type,
    const msrp_talker_message_u *wire, mrp_reg_transition_e tr,
    eth_addr_t *src_addr) {
  (void)src_addr;

  const talker_adv_info_s *info = &wire->talker.info;

  /* ----- Endpoint-side bookkeeping — runs every RX -----
   * Updates the local talker discovery DB and the per-input_stream
   * MSRP state (accumulated_latency, talker_advertised flag, failure
   * code) used by avb_input_stream_decl_event to drive our outgoing
   * listener declarations per Milan §5.5. */
  eth_addr_t talker_addr;
  memcpy(&talker_addr, info->stream_id, ETH_ADDR_LEN);
  int idx =
      avb_find_entity_by_addr(state, &talker_addr, avb_entity_type_talker);
  if (idx >= 0) {
    memcpy(&state->talkers[idx].info, info, sizeof(talker_adv_info_s));
  }
  /* Mirror the §35.2.2.8.4 failure_code into our input_stream slot
   * for ATDECC stream_info responses. decl_event derivation in
   * avb_input_stream_decl_event queries the MRP SM directly, so it
   * doesn't depend on this mirror — but the AECP GET_STREAM_INFO
   * path expects a stored value. */
  bool is_failed_attr = (attr_type == msrp_attr_type_talker_failed);
  static const uint8_t zero_stream_id[UNIQUE_ID_LEN] = {0};
  if (memcmp(info->stream_id, zero_stream_id, UNIQUE_ID_LEN) != 0) {
    for (int i = 0; i < AVB_MAX_NUM_INPUT_STREAMS; i++) {
      if (memcmp(state->input_streams[i].stream_id, info->stream_id,
                 UNIQUE_ID_LEN) == 0) {
        /* msrp_accumulated_latency and msrp_failure_code reflect the
         * wire's CURRENT content and must be refreshed on every RX,
         * not only on Registrar transition edges — otherwise an ACMP
         * CONNECT that binds input_streams[i].stream_id AFTER the
         * peer's first TALKER_FAILED arrival would miss the register
         * edge and never record the failure code (steady-state
         * subsequent RX is tr=none and would not fire the edge). */
        memcpy(state->input_streams[i].msrp_accumulated_latency,
               info->accumulated_latency, 4);
        if (is_failed_attr) {
          state->input_streams[i].msrp_failure_code[0] =
              wire->talker_failed.failure_code;
          state->input_streams[i].msrp_failure_code[1] = 0;
        } else {
          /* Fresh TALKER_ADVERTISE supersedes any stale failure. */
          state->input_streams[i].msrp_failure_code[0] = 0;
          state->input_streams[i].msrp_failure_code[1] = 0;
        }
        /* Talker left a previously-failed registration: wipe the
         * code so AECP doesn't keep reporting a stale failure after
         * the talker is gone. */
        if (tr == mrp_reg_transition_deregister && is_failed_attr) {
          state->input_streams[i].msrp_failure_code[0] = 0;
          state->input_streams[i].msrp_failure_code[1] = 0;
        }
        break;
      }
    }
  }

#ifdef CONFIG_ESP_AVB_ROLE_BRIDGE
  /* ----- Bridge MAP propagation -----
   *
   * Fires on Registrar transition edges (the common case: peer just
   * registered or deregistered the attribute), AND on mid-life
   * class-change events — when the Registrar stays IN but the
   * wire's priority field shifts (e.g., a controller re-bound the
   * talker stream from Class A to Class B via ACMP CONNECT_TX). The
   * SM doesn't transition on a content change, so without the
   * class-update path the egress Applicant would stay stuck on the
   * old class indefinitely. */
  if (state->port[port].type != avb_port_type_bridged) return;

  /* §35.2.2.8.6 accumulated_latency must aggregate the bridge's own
   * forwarding contribution onto the inbound value as we propagate
   * port→port. Inbound value lives in the wire info we just RX'd;
   * the per-port forwarding-latency estimate comes from the link
   * speed and medium (worst-case serialization for one full MFS
   * frame plus a queueing/processing constant). */
  uint32_t inbound_lat_ns =
      (uint32_t)octets_to_uint(info->accumulated_latency, 4);

  const unique_id_t *stream_id = (const unique_id_t *)&info->stream_id;
  const eth_addr_t *dest = &info->stream_dest_addr;
  const uint8_t *vlan = info->vlan_id;
  uint16_t mfs = (uint16_t)octets_to_uint(info->tspec_max_frame_size, 2);
  uint16_t mfi = (uint16_t)octets_to_uint(info->tspec_max_frame_interval, 2);
  uint8_t new_priority = info->priority;
  avb_sr_class_e new_cls =
      (new_priority == 3) ? AVB_SR_CLASS_A : AVB_SR_CLASS_B;
  uint32_t new_intervals = (new_cls == AVB_SR_CLASS_A) ? 8000u : 4000u;
  uint32_t new_bps = (uint32_t)mfs * (uint32_t)mfi * new_intervals * 8u;
  bool is_failed = (attr_type == msrp_attr_type_talker_failed);

  /* Look up the ingress-side entry so we can track and update the
   * last-propagated priority. The entry was either just inserted or
   * already existed when msrp_rx_talker_attr stepped its SM. */
  msrp_talker_entry_t *ingress =
      msrp_talker_find(port, stream_id, attr_type);
  uint8_t old_priority = ingress ? ingress->last_propagated_priority : 0;
  avb_sr_class_e old_cls =
      (old_priority == 3) ? AVB_SR_CLASS_A : AVB_SR_CLASS_B;
  uint32_t old_intervals = (old_cls == AVB_SR_CLASS_A) ? 8000u : 4000u;
  uint32_t old_bps = (uint32_t)mfs * (uint32_t)mfi * old_intervals * 8u;

  /* Decide the kind of MAP work needed. */
  bool do_release = false;     /* release admission for old_cls/old_bps */
  bool do_withdraw = false;    /* mrp_withdraw_talker on egress */
  bool do_admit_declare = false; /* admit + declare with new class */

  if (tr == mrp_reg_transition_register) {
    do_admit_declare = true;
  } else if (tr == mrp_reg_transition_deregister) {
    do_withdraw = true;
    do_release = (old_priority != 0 && !is_failed);
  } else /* tr == none */ {
    if (old_priority != 0 && old_priority != new_priority) {
      /* Class (or priority) change mid-life — full cycle. */
      do_release = !is_failed;
      do_withdraw = true;
      do_admit_declare = true;
    } else {
      return; /* steady-state RX, nothing for MAP to do */
    }
  }

  bool propagate_class_b = (new_cls == AVB_SR_CLASS_B);

  for (int y = 0; y < CONFIG_ESP_AVB_NUM_PORTS; y++) {
    if (y == port) continue;
    if (state->port[y].type != avb_port_type_bridged) continue;

    if (do_release) {
      avb_srp_admission_release(y, old_cls, old_bps);
    }
    if (do_withdraw) {
      mrp_withdraw_talker(state, y, stream_id);
    }
    if (!do_admit_declare) {
      continue;
    }

    uint32_t egress_lat_ns =
        inbound_lat_ns + avb_port_forwarding_latency_ns(&state->port[y]);

    if (is_failed) {
      /* Pass FAILED through verbatim — admission doesn't apply. */
      mrp_declare_talker_failed(state, y, stream_id, dest, vlan, mfs,
                                wire->talker_failed.failure_code,
                                propagate_class_b);
      mrp_patch_egress_accumulated_latency(
          y, stream_id, msrp_attr_type_talker_failed, egress_lat_ns);
      continue;
    }

    /* Class A on Wi-Fi egress: 125 µs presentation budget cannot be
     * met by any 802.11 medium, so the stream is not admissible. Per
     * 802.1Q-2018 §35.2.4.3, declare TALKER_FAILED with
     * insufficient_bandwidth_for_traffic_class so the downstream
     * listener sees ReadyFailed and reports the problem upward. The
     * earlier "silent skip" looked equivalent but actually masked the
     * failure — ACMP/MSRP both reported SUCCESS while no audio could
     * ever flow, because the talker never learned the path was
     * broken. */
    if (state->port[y].medium == avb_port_medium_wifi &&
        new_cls == AVB_SR_CLASS_A) {
      mrp_declare_talker_failed(state, y, stream_id, dest, vlan, mfs,
                                insufficient_bandwidth_for_traffic_class,
                                propagate_class_b);
      mrp_patch_egress_accumulated_latency(
          y, stream_id, msrp_attr_type_talker_failed, egress_lat_ns);
      continue;
    }

    int adm = avb_srp_admission_try_admit(y, new_cls, new_bps);
    if (adm == 0) {
      mrp_declare_talker_advertise(state, y, stream_id, dest, vlan, mfs,
                                   propagate_class_b);
      mrp_patch_egress_accumulated_latency(
          y, stream_id, msrp_attr_type_talker_advertise, egress_lat_ns);
    } else {
      mrp_declare_talker_failed(state, y, stream_id, dest, vlan, mfs,
                                insufficient_bandwidth_for_traffic_class,
                                propagate_class_b);
      mrp_patch_egress_accumulated_latency(
          y, stream_id, msrp_attr_type_talker_failed, egress_lat_ns);
    }
  }

  /* Update the per-stream priority tracker so the next RX can detect
   * subsequent class changes. */
  if (ingress) {
    if (do_admit_declare)
      ingress->last_propagated_priority = new_priority;
    else if (do_withdraw)
      ingress->last_propagated_priority = 0;
  }
#else
  (void)port; (void)attr_type; (void)tr;
#endif
}

#ifdef CONFIG_ESP_AVB_ROLE_BRIDGE
/* IEEE 802.1Q-2018 §35.2.4.4.3 — merge listener declarations from
 * every bridged port except `exclude_port` for `stream_id`. Returns
 * true and writes *out if at least one downstream listener has its
 * Registrar IN for this stream; false otherwise (caller should
 * withdraw on the egress port).
 *
 *   any Ready Failed                       -> Ready Failed
 *   any Ready + any Asking Failed          -> Ready Failed
 *   any Ready, no failed                   -> Ready
 *   only Asking Failed                     -> Asking Failed
 *
 * Only Registrar=IN entries contribute; LV/MT means the peer's
 * declaration is no longer live and must not pin the merged value. */
static bool msrp_merge_listener_decls(int exclude_port,
                                      const unique_id_t *stream_id,
                                      msrp_listener_event_t *out) {
  bool any_ready = false;
  bool any_asking = false;
  bool any_ready_failed = false;
  for (int p = 0; p < CONFIG_ESP_AVB_NUM_PORTS; p++) {
    if (p == exclude_port) continue;
    msrp_listener_entry_t *e = msrp_listener_find(p, stream_id);
    if (e == NULL) continue;
    if (e->sm.registrar != mrp_registrar_in) continue;
    switch (e->peer_decl) {
    case msrp_listener_event_ready:        any_ready = true; break;
    case msrp_listener_event_asking_failed: any_asking = true; break;
    case msrp_listener_event_ready_failed:  any_ready_failed = true; break;
    default: break;
    }
  }
  if (!any_ready && !any_asking && !any_ready_failed) return false;
  if (any_ready_failed || (any_ready && any_asking)) {
    *out = msrp_listener_event_ready_failed;
  } else if (any_ready) {
    *out = msrp_listener_event_ready;
  } else {
    *out = msrp_listener_event_asking_failed;
  }
  return true;
}
#endif

static void mrp_on_listener_registrar_change(
    avb_state_s *state, int port, const msrp_listener_message_s *wire,
    mrp_reg_transition_e tr, bool peer_decl_changed,
    eth_addr_t *src_addr) {
  /* §35.2.2.4.4 listener declaration nibble lives in event_decl_data[1]
   * event1 (2 bits). Values match msrp_listener_event_t (1=AskingFailed,
   * 2=Ready, 3=ReadyFailed). */
  msrp_listener_event_t decl =
      (msrp_listener_event_t)wire->event_decl_data[1].declaration.event1;
  int attr_event = 0, unused1 = 0, unused2 = 0;
  three_pe_to_int(wire->event_decl_data[0].event, &attr_event, &unused1,
                  &unused2);

  /* ----- Endpoint-side (talker-side) bookkeeping — runs every RX -----
   * If this listener declaration is for one of our output streams,
   * update the per-listener msrp_ready / asking_failed flags and
   * start/stop streaming as appropriate. ACMP DISCONNECT_TX is
   * authoritative for tearing down the listener entry itself;
   * MSRP only flips the readiness flags. */
  for (int i = 0; i < state->num_output_streams; i++) {
    avb_talker_stream_s *stream = &state->output_streams[i];
    if (memcmp(wire->stream_id, stream->stream_id, UNIQUE_ID_LEN) != 0)
      continue;

    /* Listener leaving — wire-level rLv. */
    if (attr_event == mrp_attr_event_lv) {
      int lidx = find_connected_listener(stream, src_addr);
      if (lidx >= 0) {
        avbinfo("MSRP: listener leaving stream %d", i);
        stream->connected_listeners[lidx].msrp_ready = false;
        stream->connected_listeners[lidx].asking_failed = false;
        bool should_stop = !any_listener_ready(stream) ||
                           (!state->config.milan_compliant &&
                            !any_listener_acmp_connected(stream));
        if (should_stop && stream->streaming) {
          avb_stop_stream_out(state, i);
        }
      }
      break;
    }

    /* Listener ready — Plain AVB starts only when both ACMP and MSRP
     * agree that the listener is connected/ready. Milan Talkers do
     * not maintain ACMP listener state; MSRP Listener Ready alone
     * drives streaming. */
    if (decl == msrp_listener_event_ready) {
      int lidx = find_or_add_msrp_listener(state, stream, src_addr);
      if (lidx < 0) {
        avberr("MSRP: connected_listeners full for stream %d", i);
        break;
      }
      bool was_ready = stream->connected_listeners[lidx].msrp_ready;
      stream->connected_listeners[lidx].msrp_ready = true;
      if (!was_ready) {
        avbinfo("MSRP: listener ready for stream %d (count=%d, acmp=%d)", i,
                octets_to_uint(stream->connection_count, 2),
                stream->connected_listeners[lidx].acmp_connected);
      }
      stream->connected_listeners[lidx].asking_failed = false;
      bool should_start =
          state->config.milan_compliant
              ? stream->connected_listeners[lidx].msrp_ready
              : stream->connected_listeners[lidx].acmp_connected;
      if (should_start && !stream->streaming) {
        avb_start_stream_out(state, i);
      }
      break;
    }

    /* AskingFailed / ReadyFailed — track per-listener so ACMP
     * GET_TX_STATE_RESPONSE can set REGISTERING_FAILED per Milan
     * v1.3 Table 5.23. ready_failed is a transient MRP event with
     * the same talker-side meaning; treat identically. */
    if (decl == msrp_listener_event_asking_failed ||
        decl == msrp_listener_event_ready_failed) {
      int lidx = find_or_add_msrp_listener(state, stream, src_addr);
      if (lidx < 0) {
        avberr("MSRP: connected_listeners full for stream %d", i);
        break;
      }
      if (!stream->connected_listeners[lidx].msrp_ready) {
        stream->connected_listeners[lidx].msrp_ready = false;
      }
      stream->connected_listeners[lidx].asking_failed = true;
      avbinfo("MSRP: listener %s for stream %d",
              decl == msrp_listener_event_asking_failed ? "asking_failed"
                                                         : "ready_failed",
              i);
      break;
    }
    break;
  }

#ifdef CONFIG_ESP_AVB_ROLE_BRIDGE
  /* ----- Bridge MAP propagation with §35.2.4.4.3 merging -----
   * Fire when the peer's Registrar transitioned (join/leave edge)
   * OR when the peer's declaration nibble changed mid-registration
   * (e.g. Ready -> AskingFailed). For each other bridged port,
   * recompute the merged value across all remaining peers and
   * re-declare only when the merged decl actually changes. */
  if (tr == mrp_reg_transition_none && !peer_decl_changed) return;
  if (state->port[port].type != avb_port_type_bridged) return;

  const unique_id_t *stream_id = (const unique_id_t *)&wire->stream_id;
  for (int y = 0; y < CONFIG_ESP_AVB_NUM_PORTS; y++) {
    if (y == port) continue;
    if (state->port[y].type != avb_port_type_bridged) continue;

    msrp_listener_event_t merged;
    bool has_merged = msrp_merge_listener_decls(y, stream_id, &merged);
    msrp_listener_entry_t *egress = msrp_listener_find(y, stream_id);
    bool currently_declaring =
        (egress != NULL && egress->sm.applicant != mrp_applicant_vo);

    if (!has_merged) {
      if (currently_declaring) {
        mrp_withdraw_listener(state, y, stream_id);
      }
    } else {
      if (!currently_declaring || egress->decl_event != merged) {
        mrp_declare_listener(state, y, stream_id, merged);
      }
    }
  }
#else
  (void)port; (void)tr; (void)peer_decl_changed;
#endif
}

static void mrp_on_domain_registrar_change(
    avb_state_s *state, int port, const msrp_domain_message_s *wire,
    mrp_reg_transition_e tr) {
  /* ----- Endpoint-side validation — runs every RX -----
   * The network's SR-class VLAN must match our own. Mismatches mean
   * a configuration drift that will break MSRP path establishment;
   * log loudly so the operator notices. */
  uint16_t vid = octets_to_uint(wire->sr_class_vid, 2);
  uint16_t mapping_index =
      avb_msrp_mapping_index_for_class_id(wire->sr_class_id);
  uint16_t our_vid =
      octets_to_uint(state->msrp_mappings[mapping_index].vlan_id, 2);
  if ((wire->sr_class_id == 6 || wire->sr_class_id == 5) && vid != our_vid) {
    avberr("MSRP domain: class %d network VLAN %d != our VLAN %d",
           wire->sr_class_id, vid, our_vid);
  }

#ifdef CONFIG_ESP_AVB_ROLE_BRIDGE
  /* ----- Bridge MAP propagation — transition-edge only ----- */
  if (tr == mrp_reg_transition_none) return;
  if (state->port[port].type != avb_port_type_bridged) return;

  for (int y = 0; y < CONFIG_ESP_AVB_NUM_PORTS; y++) {
    if (y == port) continue;
    if (state->port[y].type != avb_port_type_bridged) continue;

    if (tr == mrp_reg_transition_deregister) {
      /* No mrp_withdraw_domain API — MSRP domain entries are
       * essentially static SR-class config (§35.2.2.10). Leave the
       * propagated copy in place; LeaveAll will eventually cycle it
       * if it's truly gone. */
      continue;
    }
    mrp_declare_domain(state, y, wire->sr_class_id, wire->sr_class_priority,
                       wire->sr_class_vid);
  }
#else
  (void)port; (void)tr;
#endif
}

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
 * its 3pe event vector. Returns the Registrar transition (register
 * /deregister edge) so the caller can fire the MAP / application
 * callback only on transition edges, not on every RX. */
static mrp_reg_transition_e msrp_rx_talker_attr(
    int port, msrp_attr_type_t attr_type,
    const msrp_talker_message_u *wire) {
  msrp_talker_entry_t *e = msrp_talker_find_or_insert(
      port, (const unique_id_t *)&wire->talker.info.stream_id, attr_type);
  if (e == NULL) return mrp_reg_transition_none; /* table full */
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
  mrp_registrar_state_e before = e->sm.registrar;
  mrp_sm_step(&e->sm, mrp_3pe_to_event(e1));
  return mrp_reg_transition(before, e->sm.registrar);
}

/* Process one LISTENER attribute. */
static mrp_reg_transition_e msrp_rx_listener_attr(
    int port, const msrp_listener_message_s *wire,
    bool *peer_decl_changed_out) {
  msrp_listener_entry_t *e = msrp_listener_find_or_insert(
      port, (const unique_id_t *)&wire->stream_id);
  if (e == NULL) {
    if (peer_decl_changed_out) *peer_decl_changed_out = false;
    return mrp_reg_transition_none;
  }
  memcpy(&e->wire, wire, sizeof(*wire));
  /* event_decl_data[0] carries both the 3pe attribute event and the
   * 4pe listener-declaration nibble. Decode the attribute event. */
  int e1, e2, e3;
  three_pe_to_int(wire->event_decl_data[0].event, &e1, &e2, &e3);
  mrp_registrar_state_e before = e->sm.registrar;
  mrp_sm_step(&e->sm, mrp_3pe_to_event(e1));
  /* Track the peer's declaration nibble (asking_failed / ready /
   * ready_failed) separately from our outbound decl_event. Bridge
   * MAP needs the peer-side value to compute §35.2.4.4.3 merges
   * across ports without confusing it with our own egress decl. */
  msrp_listener_event_t new_peer_decl =
      (msrp_listener_event_t)wire->event_decl_data[1].declaration.event1;
  if (peer_decl_changed_out) {
    *peer_decl_changed_out = (new_peer_decl != e->peer_decl);
  }
  e->peer_decl = new_peer_decl;
  return mrp_reg_transition(before, e->sm.registrar);
}

/* Process one DOMAIN attribute. */
static mrp_reg_transition_e msrp_rx_domain_attr(
    int port, const msrp_domain_message_s *wire) {
  msrp_domain_entry_t *e = msrp_domain_find_or_insert(port, wire->sr_class_id);
  if (e == NULL) return mrp_reg_transition_none;
  memcpy(&e->wire, wire, sizeof(*wire));
  int e1, e2, e3;
  three_pe_to_int(wire->attr_event[0], &e1, &e2, &e3);
  mrp_registrar_state_e before = e->sm.registrar;
  mrp_sm_step(&e->sm, mrp_3pe_to_event(e1));
  return mrp_reg_transition(before, e->sm.registrar);
}

/* Walk an inbound MSRP buffer and dispatch SM events + fire the
 * on_*_registrar_change application callbacks for each attribute.
 * Single MSRP RX entry point — avb_process_rx_message just calls
 * here. */
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
      mrp_reg_transition_e tr = msrp_rx_domain_attr(port, &wire);
      /* Fire unconditionally: the callback runs endpoint validation
       * every RX and gates MAP propagation on `tr` internally. */
      mrp_on_domain_registrar_change(state, port, &wire, tr);
      break;
    }
    case msrp_attr_type_talker_advertise:
    case msrp_attr_type_talker_failed: {
      msrp_talker_message_u wire;
      memcpy(&wire, &msg->messages_raw[offset], sizeof(wire));
      mrp_reg_transition_e tr =
          msrp_rx_talker_attr(port, header.attr_type, &wire);
      mrp_on_talker_registrar_change(state, port, header.attr_type,
                                     &wire, tr, src_addr);
      break;
    }
    case msrp_attr_type_listener: {
      msrp_listener_message_s wire;
      memcpy(&wire, &msg->messages_raw[offset], sizeof(wire));
      bool peer_decl_changed = false;
      mrp_reg_transition_e tr =
          msrp_rx_listener_attr(port, &wire, &peer_decl_changed);
      mrp_on_listener_registrar_change(state, port, &wire, tr,
                                       peer_decl_changed, src_addr);
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
                                  uint16_t max_frame_size,
                                  bool class_b) {
  memcpy(info->stream_id, stream_id, UNIQUE_ID_LEN);
  memcpy(info->stream_dest_addr, stream_dest_addr, ETH_ADDR_LEN);
  memcpy(info->vlan_id, vlan_id, 2);
  int tspec_mfs = max_frame_size;
  int_to_octets(&tspec_mfs, info->tspec_max_frame_size, 2);
  int tspec_mfi = 1;
  int_to_octets(&tspec_mfi, info->tspec_max_frame_interval, 2);
  /* SR-class mapping: index 0 = Class A (priority 3), index 1 = Class B
   * (priority 2) per the §35.2.2.8.2.3 default mappings. Caller passes
   * class_b explicitly — endpoint origination derives it from the
   * output_stream's stream_info_flags.class_b (set by the ACMP CONNECT
   * handler from the CLASS_B flag bit), and bridge MAP derives it from
   * the inbound TALKER_ADVERTISE priority. The legacy "match by VLAN"
   * heuristic doesn't work when both classes share a VLAN. */
  uint16_t mapping_index = class_b ? 1 : 0;
  if (mapping_index >= state->msrp_mappings_count) mapping_index = 0;
  info->priority = state->msrp_mappings[mapping_index].priority;
  info->rank = 1;
  int accumulated_latency = 15000;
  int_to_octets(&accumulated_latency, info->accumulated_latency, 4);
}

/* If the Applicant is already actively declaring (non-observer state),
 * a repeated call is treated as a refresh (Join!), not a fresh New!.
 * Lets the periodic-send loop call mrp_declare_* idempotently without
 * resetting the SM's anxious/quiet cycle. */
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
                                  uint16_t max_frame_size,
                                  bool class_b) {
  msrp_talker_entry_t *e = msrp_talker_find_or_insert(
      port, stream_id, msrp_attr_type_talker_advertise);
  if (e == NULL) return;
  memset(&e->wire, 0, sizeof(e->wire));
  mrp_build_talker_info(state, &e->wire.talker.info, stream_id,
                        stream_dest_addr, vlan_id, max_frame_size, class_b);
  mrp_applicant_step(&e->sm, mrp_declare_event(&e->sm));
  mrp_port_arm_join_timer(state, port);
}

void mrp_declare_talker_failed(avb_state_s *state, int port,
                               const unique_id_t *stream_id,
                               const eth_addr_t *stream_dest_addr,
                               const uint8_t *vlan_id,
                               uint16_t max_frame_size,
                               uint8_t failure_code,
                               bool class_b) {
  msrp_talker_entry_t *e = msrp_talker_find_or_insert(
      port, stream_id, msrp_attr_type_talker_failed);
  if (e == NULL) return;
  memset(&e->wire, 0, sizeof(e->wire));
  mrp_build_talker_info(state, &e->wire.talker_failed.info, stream_id,
                        stream_dest_addr, vlan_id, max_frame_size, class_b);
  e->wire.talker_failed.failure_code = failure_code;
  /* §35.2.2.8.5 failure_bridge_id: 8-byte EUI-64 of the bridge that
   * detected the failure, so listeners can identify *which* bridge
   * along the path refused admission. EUI-64 from MAC per 802.1AS
   * §8.5.2.2: insert 0xff:0xfe between bytes 3 and 4 of the 48-bit
   * MAC. Use port[0]'s interface MAC (EMAC) — same value the gPTP
   * daemon uses for clockIdentity, so a listener can cross-reference
   * TALKER_FAILED's source against the GM hierarchy. */
  const uint8_t *m = (const uint8_t *)state->port[0].internal_mac_addr;
  e->wire.talker_failed.failure_bridge_id[0] = m[0];
  e->wire.talker_failed.failure_bridge_id[1] = m[1];
  e->wire.talker_failed.failure_bridge_id[2] = m[2];
  e->wire.talker_failed.failure_bridge_id[3] = 0xff;
  e->wire.talker_failed.failure_bridge_id[4] = 0xfe;
  e->wire.talker_failed.failure_bridge_id[5] = m[3];
  e->wire.talker_failed.failure_bridge_id[6] = m[4];
  e->wire.talker_failed.failure_bridge_id[7] = m[5];
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
 * resulting pending TX actions as single-attribute MRPDUs. */

/* Forward decl — definition is below in §6, after the periodic
 * declare helpers. */
static int mrp_send_attr(avb_state_s *state, int port, void *attr,
                              int attr_list_len, const char *label);

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
  mrp_send_attr(state, port, &msg, attr_list_len,
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
  mrp_send_attr(state, port, &msg, attr_list_len, "listener");
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
  mrp_send_attr(state, port, &msg, attr_list_len, "domain");
}

/* MVRP per-port flush, defined in §7. Iterates s_mvrp_vlans[port]
 * and emits any MVRP attribute SMs with pending TX. */
static void mrp_tx_flush_mvrp(avb_state_s *state, int port, bool leaveall,
                              mrp_event_e tx_ev);

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

  /* MVRP attributes share the same per-port LeaveAll flag — the
   * fan-out happens inside mrp_tx_flush_mvrp using the same tx_ev. */
  mrp_tx_flush_mvrp(state, port, leaveall, tx_ev);

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

/* ===== §7  MVRP application =====
 *
 * VLAN registration per IEEE 802.1Q-2018 §11. MVRP has a single
 * attribute type (vlan_identifier), no per-class semantics, and no
 * admission control — the application layer is dramatically simpler
 * than MSRP. The same generic MRP SMs (§1) drive Applicant/Registrar
 * state per VLAN ID per port; the helpers below mirror the §6
 * pattern for MSRP.
 *
 * Wire layout per §11.2.3.2: one MVRP attribute per PDU in the
 * common case (num_vals=1). MaxNumValues per the spec allows packing
 * multiple VIDs but our peers don't, so we don't either — keeps the
 * encode/decode trivial.
 */

#define MVRP_VLAN_TABLE_SIZE 4 /* small; typical use is 1–2 VLANs */

typedef struct {
  bool valid;
  uint8_t vlan_id[2]; /* big-endian, network order */
  mrp_sm_state_t sm;
} mvrp_vlan_entry_t;

static mvrp_vlan_entry_t
    s_mvrp_vlans[CONFIG_ESP_AVB_NUM_PORTS][MVRP_VLAN_TABLE_SIZE];

/* LeaveTimer dispatch — declared in §1b above mrp_port_tick. The
 * Registrar SM (§1) handles mrp_event_leave_timer (LV → MT)
 * correctly; this loop is what actually delivers the event when
 * the per-Registrar leave_timer_us expires. mrp_sm_step dispatches
 * into both Applicant and Registrar; the Applicant ignores the
 * leave-timer event so the cost of the unconditional dispatch on
 * each entry is small. */
static bool mrp_port_dispatch_leave_timers(int port, int64_t now) {
  if (port < 0 || port >= CONFIG_ESP_AVB_NUM_PORTS) return false;
  bool fired = false;
  for (int i = 0; i < MSRP_TALKER_TABLE_SIZE; ++i) {
    msrp_talker_entry_t *e = &s_msrp_talkers[port][i];
    if (!e->valid) continue;
    if (e->sm.leave_timer_us != 0 && now >= e->sm.leave_timer_us) {
      mrp_sm_step(&e->sm, mrp_event_leave_timer);
      fired = true;
    }
  }
  for (int i = 0; i < MSRP_LISTENER_TABLE_SIZE; ++i) {
    msrp_listener_entry_t *e = &s_msrp_listeners[port][i];
    if (!e->valid) continue;
    if (e->sm.leave_timer_us != 0 && now >= e->sm.leave_timer_us) {
      mrp_sm_step(&e->sm, mrp_event_leave_timer);
      fired = true;
    }
  }
  for (int i = 0; i < MSRP_DOMAIN_TABLE_SIZE; ++i) {
    msrp_domain_entry_t *e = &s_msrp_domains[port][i];
    if (!e->valid) continue;
    if (e->sm.leave_timer_us != 0 && now >= e->sm.leave_timer_us) {
      mrp_sm_step(&e->sm, mrp_event_leave_timer);
      fired = true;
    }
  }
  for (int i = 0; i < MVRP_VLAN_TABLE_SIZE; ++i) {
    mvrp_vlan_entry_t *e = &s_mvrp_vlans[port][i];
    if (!e->valid) continue;
    if (e->sm.leave_timer_us != 0 && now >= e->sm.leave_timer_us) {
      mrp_sm_step(&e->sm, mrp_event_leave_timer);
      fired = true;
    }
  }
  return fired;
}

static bool vlan_id_eq(const uint8_t *a, const uint8_t *b) {
  return a[0] == b[0] && a[1] == b[1];
}

static mvrp_vlan_entry_t *mvrp_vlan_find(int port, const uint8_t *vlan_id) {
  if (port < 0 || port >= CONFIG_ESP_AVB_NUM_PORTS) return NULL;
  for (int i = 0; i < MVRP_VLAN_TABLE_SIZE; i++) {
    mvrp_vlan_entry_t *e = &s_mvrp_vlans[port][i];
    if (e->valid && vlan_id_eq(e->vlan_id, vlan_id)) return e;
  }
  return NULL;
}

static mvrp_vlan_entry_t *mvrp_vlan_find_or_insert(int port,
                                                   const uint8_t *vlan_id) {
  mvrp_vlan_entry_t *e = mvrp_vlan_find(port, vlan_id);
  if (e != NULL) return e;
  if (port < 0 || port >= CONFIG_ESP_AVB_NUM_PORTS) return NULL;
  for (int i = 0; i < MVRP_VLAN_TABLE_SIZE; i++) {
    mvrp_vlan_entry_t *slot = &s_mvrp_vlans[port][i];
    if (!slot->valid) {
      memset(slot, 0, sizeof(*slot));
      slot->valid = true;
      memcpy(slot->vlan_id, vlan_id, 2);
      slot->sm.applicant = mrp_applicant_vo;
      slot->sm.registrar = mrp_registrar_mt;
      slot->sm.pending_tx = mrp_tx_none;
      return slot;
    }
  }
  return NULL; /* table full */
}

/* Dispatch rLA to every MVRP SM on this port. Called when a received
 * MVRP MRPDU carries the LeaveAll bit. */
static void mvrp_dispatch_leaveall(int port) {
  for (int i = 0; i < MVRP_VLAN_TABLE_SIZE; i++) {
    if (s_mvrp_vlans[port][i].valid) {
      mrp_sm_step(&s_mvrp_vlans[port][i].sm, mrp_event_r_la);
    }
  }
}

/* MAP transition callback for MVRP — parallel to mrp_on_talker_/
 * listener_/domain_registrar_change in §6a. Fires only on Registrar
 * register/deregister edges. Bridge mode re-declares VLAN membership
 * on every other bridged port so a peer's VLAN registration on one
 * port flows out the other ports as well. No admission gate — MVRP
 * has no per-class semantics. */
static void mrp_on_vlan_registrar_change(avb_state_s *state, int port,
                                         const uint8_t *vlan_id,
                                         mrp_reg_transition_e tr) {
#ifdef CONFIG_ESP_AVB_ROLE_BRIDGE
  if (state->port[port].type != avb_port_type_bridged) return;
  for (int y = 0; y < CONFIG_ESP_AVB_NUM_PORTS; y++) {
    if (y == port) continue;
    if (state->port[y].type != avb_port_type_bridged) continue;
    if (tr == mrp_reg_transition_register) {
      mrp_declare_vlan(state, y, vlan_id);
    } else {
      mrp_withdraw_vlan(state, y, vlan_id);
    }
  }
#else
  (void)state; (void)port; (void)vlan_id; (void)tr;
#endif
}

/* Single MVRP RX entry point. The wire struct as-defined carries one
 * vlan_id slot, so we treat each PDU as a single attribute (num_vals
 * effectively forced to 1). avb_process_rx_message calls this for
 * every MVRP frame; no legacy reaction layer to coexist with.
 *
 * The Registrar transition is computed across mrp_sm_step so the MAP
 * callback only fires on register/deregister edges, not per RX. */
void mrp_rx_mvrp(avb_state_s *state, int port, mvrp_vlan_id_message_s *msg,
                 size_t length, eth_addr_t *src_addr) {
  (void)length; (void)src_addr;
  if (port < 0 || port >= CONFIG_ESP_AVB_NUM_PORTS) return;
  if (msg->header.attr_type != mvrp_attr_type_vlan_identifier) return;

  if (msg->header.vechead_leaveall) {
    mvrp_dispatch_leaveall(port);
  }

  mvrp_vlan_entry_t *e = mvrp_vlan_find_or_insert(port, msg->vlan_id);
  if (e == NULL) return;

  int e1, e2, e3;
  three_pe_to_int(msg->attr_event[0], &e1, &e2, &e3);
  mrp_registrar_state_e before = e->sm.registrar;
  mrp_sm_step(&e->sm, mrp_3pe_to_event(e1));
  mrp_reg_transition_e tr = mrp_reg_transition(before, e->sm.registrar);
  if (tr != mrp_reg_transition_none) {
    mrp_on_vlan_registrar_change(state, port, msg->vlan_id, tr);
  }
}

/* SM-driven origination. Idempotent — repeat calls keep the
 * Applicant alive (Join!), first call seeds it (New!). */
void mrp_declare_vlan(avb_state_s *state, int port,
                      const uint8_t *vlan_id) {
  if (port < 0 || port >= CONFIG_ESP_AVB_NUM_PORTS) return;
  mvrp_vlan_entry_t *e = mvrp_vlan_find_or_insert(port, vlan_id);
  if (e == NULL) return;
  mrp_applicant_step(&e->sm, mrp_declare_event(&e->sm));
  mrp_port_arm_join_timer(state, port);
}

void mrp_withdraw_vlan(avb_state_s *state, int port,
                       const uint8_t *vlan_id) {
  mvrp_vlan_entry_t *e = mvrp_vlan_find(port, vlan_id);
  if (e == NULL) return;
  mrp_applicant_step(&e->sm, mrp_event_lv);
  mrp_port_arm_join_timer(state, port);
}

/* Per-attribute TX builder: emit one MVRP MRPDU for one VLAN entry.
 * MVRP PDUs only carry one attribute type so each entry maps 1:1 to
 * its own frame on the wire. */
static void mvrp_tx_emit(avb_state_s *state, int port, mvrp_vlan_entry_t *e,
                         bool leaveall) {
  mvrp_vlan_id_message_s msg;
  memset(&msg, 0, sizeof(msg));
  msg.protocol_ver = 0;
  msg.header.attr_type = mvrp_attr_type_vlan_identifier;
  msg.header.attr_len = 2;
  msg.header.vechead_leaveall = leaveall ? 1 : 0;
  msg.header.vechead_num_vals = 1;
  memcpy(msg.vlan_id, e->vlan_id, 2);
  int pe = mrp_resolve_3pe(e->sm.pending_tx, e->sm.registrar);
  if (pe < 0) return;
  msg.attr_event[0] = int_to_3pe(pe, 0, 0);
  /* MVRP MRPDU size: protocol_ver(1) + header(4) + vlan_id(2) +
   * attr_event(1) + end-mark-list(2) + end-mark-msg(2) = 12. */
  struct timespec ts;
  (void)avb_net_send_on(state, port, ethertype_mvrp, &msg, 12, &ts);
}

/* Per-port MVRP flush, called from mrp_tx_flush_port (§6c). */
static void mrp_tx_flush_mvrp(avb_state_s *state, int port, bool leaveall,
                              mrp_event_e tx_ev) {
  for (int i = 0; i < MVRP_VLAN_TABLE_SIZE; i++) {
    mvrp_vlan_entry_t *e = &s_mvrp_vlans[port][i];
    if (!e->valid) continue;
    mrp_applicant_step(&e->sm, tx_ev);
    if (e->sm.pending_tx != mrp_tx_none) {
      mvrp_tx_emit(state, port, e, leaveall);
      e->sm.pending_tx = mrp_tx_none;
    }
  }
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

static int mrp_send_attr(avb_state_s *state, int port, void *attr,
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

  /* Route to the named port's egress so bridge MAP re-declarations land on
   * the correct port (port 0 = Ethernet, port 1 = Wi-Fi AP). Endpoints have
   * only port 0 so the behavior matches avb_net_send. */
  ret = avb_net_send_on(state, port, ethertype_msrp, &msrp_msg, msg_len, &ts);
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
 * cap. Hooked into mrp_on_talker_registrar_change (§6a) so an
 * over-budget request propagates as TALKER_FAILED with failure_code
 * = insufficient_bandwidth_for_traffic_class.
 *
 * On Wi-Fi the bridge skips Class A entirely (Wi-Fi efficiency
 * cut #1) — no listener can honor the 125 µs SLA. Class B is
 * admitted normally.
 *
 * Excluded from endpoint builds via CONFIG_ESP_AVB_ROLE_BRIDGE.
 * ====================================================================== */

#ifdef CONFIG_ESP_AVB_ROLE_BRIDGE

static avb_admission_class_s
    s_admission[CONFIG_ESP_AVB_NUM_PORTS][AVB_SR_CLASS_COUNT];

int avb_srp_admission_init(avb_state_s *state) {
  for (int p = 0; p < CONFIG_ESP_AVB_NUM_PORTS; p++) {
    /* Per-port link rate from the new typology. link_speed_mbps is the
     * nominal PHY-rate cap (1000 for gigabit eth, ~150 for Wi-Fi 6 STA).
     * Falls back to 0 if unset (admission then rejects all on this port). */
    uint32_t link_rate = (uint32_t)state->port[p].link_speed_mbps * 1000000u;
    for (int c = 0; c < AVB_SR_CLASS_COUNT; c++) {
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
