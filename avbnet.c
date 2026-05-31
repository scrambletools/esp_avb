/*
 * Copyright 2024-2026 Scramble Tools
 * License: MIT
 *
 * ESP_AVB Component
 *
 * This component provides an implementation of an AVB talker and listener.
 *
 * This file provides the network interface for the ESP_AVB component.
 *
 * RX Architecture: A single EMAC RX callback (avb_unified_rx_cb) handles ALL
 * incoming Ethernet frames. VLAN-tagged AVTP stream data is dispatched to a
 * registered handler callback (runs inline in EMAC task for lowest latency).
 * Control frames (AVTP, MSRP, MVRP) are copied to a queue and consumed by the
 * AVB main loop via avb_net_recv_ctrl(). All other frames (PTP, ARP, IP) pass
 * through to the IP stack via esp_netif_receive().
 *
 * TX Architecture: Unchanged — uses L2TAP write() on per-ethertype fds.
 */

#include "avb.h"
#include "esp_ptp.h"
#include "esp_timer.h"
#include <esp_netif.h>
#include <esp_vfs_l2tap.h>
#include <esp_wifi.h>
#ifdef CONFIG_ESP_AVB_ROLE_BRIDGE
#include "avbbridge.h"
#endif

#define TAG "AVB-NET"

/* Control frame queue depth — ample for MSRP/MVRP/ATDECC rates.
 * Deliberately small: under a controller's enumeration burst, the
 * Wi-Fi medium can't drain responses faster than ~50 fps, so a deeper
 * queue just delays the tail. With xQueueSend(..., 0) the surplus
 * gets dropped here; the controller's retry is faster than waiting
 * for a backlog to drain. */
#define CTRL_RX_QUEUE_DEPTH 16

/* File-static state for unified EMAC RX dispatcher */
static QueueHandle_t s_ctrl_rx_queue = NULL;
static avb_stream_rx_handler_t s_stream_handler = NULL;
static void *s_stream_ctx = NULL;
static esp_netif_t *s_eth_netif = NULL;
#if CONFIG_ESP_AVB_NUM_PORTS > 1
static esp_netif_t *s_wifi_netif = NULL;
#endif

/* Bridge forwarding counters. Bumped inside avb_bridge_forward (bridge
 * builds only); on endpoint builds they stay zero and the accessor
 * still links. Diagnoses wired<->Wi-Fi multicast forwarding asymmetry. */
static volatile uint32_t s_fwd_eth_ok, s_fwd_eth_fail;
static volatile uint32_t s_fwd_wifi_ok, s_fwd_wifi_fail, s_fwd_wifi_oom;
/* Sub-counts of s_fwd_wifi_ok split by destination address type. Used by the
 * bridge heartbeat to surface the very specific question "is the AP actually
 * able to send unicast frames to associated STAs?" — multicast/broadcast and
 * unicast take different paths inside the IDF wifi driver, and the two paths
 * can be in very different states for the first minute or so after a cold
 * boot. Hot path adds 1 branch + 1 store. */
static volatile uint32_t s_fwd_wifi_ok_ucast, s_fwd_wifi_ok_mcast;

void avb_bridge_forward_stats(uint32_t *eth_ok, uint32_t *eth_fail,
                              uint32_t *wifi_ok, uint32_t *wifi_fail,
                              uint32_t *wifi_oom) {
  if (eth_ok)    *eth_ok    = s_fwd_eth_ok;
  if (eth_fail)  *eth_fail  = s_fwd_eth_fail;
  if (wifi_ok)   *wifi_ok   = s_fwd_wifi_ok;
  if (wifi_fail) *wifi_fail = s_fwd_wifi_fail;
  if (wifi_oom)  *wifi_oom  = s_fwd_wifi_oom;
}

void avb_bridge_forward_stats_wifi_split(uint32_t *wifi_ok_ucast,
                                         uint32_t *wifi_ok_mcast) {
  if (wifi_ok_ucast) *wifi_ok_ucast = s_fwd_wifi_ok_ucast;
  if (wifi_ok_mcast) *wifi_ok_mcast = s_fwd_wifi_ok_mcast;
}

/* Ingress-port lookup by medium. Populated once at avb_net_init from
 * the per-port topology (state->port[].medium) so the RX callback
 * doesn't need to know whether it's running on a bridge (port[0]=eth,
 * port[1]=wifi) or an endpoint (single port, either medium). -1 means
 * "no port configured for this medium" — RX from that medium is
 * dropped. */
static int s_eth_port_idx = -1;
static int s_wifi_port_idx = -1;

static void avb_net_cache_port_indices(avb_state_s *state) {
  s_eth_port_idx = -1;
  s_wifi_port_idx = -1;
  for (int p = 0; p < CONFIG_ESP_AVB_NUM_PORTS; p++) {
    if (!state->port[p].enabled) continue;
    if (state->port[p].medium == avb_port_medium_eth_hwts &&
        s_eth_port_idx < 0) {
      s_eth_port_idx = p;
    } else if (state->port[p].medium == avb_port_medium_wifi_ftm &&
               s_wifi_port_idx < 0) {
      s_wifi_port_idx = p;
    }
  }
}

#ifdef CONFIG_ESP_AVB_ROLE_BRIDGE
/* esp_wifi_internal_tx forward decl. Same rationale as the equivalent
 * decl above avb_net_transmit_raw further down — declared locally so
 * esp_avb doesn't gain a hard esp_wifi PRIV_REQUIRES; the symbol is
 * provided by esp_wifi (native) or esp_wifi_remote (P4 host with
 * coprocessor). The bridge needs this for the Wi-Fi-egress branch in
 * avb_bridge_forward. */
extern esp_err_t esp_wifi_internal_tx(int wifi_if, void *buffer, size_t len);

/* Zero-copy TX path: tx_by_ref hands ownership of the buffer to Wi-Fi
 * and IDF releases it later via the registered netstack-buf callbacks.
 * Saves the IDF-internal memcpy that esp_wifi_internal_tx() does. */
typedef void (*wifi_netstack_buf_ref_cb_t)(void *netstack_buf);
typedef void (*wifi_netstack_buf_free_cb_t)(void *netstack_buf);
extern esp_err_t esp_wifi_internal_tx_by_ref(int wifi_if, void *buffer,
                                             size_t len, void *netstack_buf);
extern esp_err_t esp_wifi_internal_reg_netstack_buf_cb(
    wifi_netstack_buf_ref_cb_t ref, wifi_netstack_buf_free_cb_t free);
/* lwIP's registered handlers, exported by esp_netif. We chain through
 * to them for non-tagged netstack_buf (i.e. genuine lwIP pbufs) so the
 * non-bridge data path keeps working. */
extern void esp_netif_netstack_buf_ref(void *netstack_buf);
extern void esp_netif_netstack_buf_free(void *netstack_buf);

/* Low bit of netstack_buf is our discriminator. Pointers from malloc()
 * are at least 8-byte aligned on this platform; lwIP pbufs are 4-byte
 * aligned at minimum; the low bit is therefore free for tagging. */
#define BRIDGE_BUF_TAG  ((uintptr_t)1)

static void avb_bridge_netstack_buf_ref(void *nb) {
  if (((uintptr_t)nb & BRIDGE_BUF_TAG) != 0) return;
  esp_netif_netstack_buf_ref(nb);
}

static void avb_bridge_netstack_buf_free(void *nb) {
  if (((uintptr_t)nb & BRIDGE_BUF_TAG) != 0) {
    free((void *)((uintptr_t)nb & ~BRIDGE_BUF_TAG));
    return;
  }
  esp_netif_netstack_buf_free(nb);
}

/* Public init: install our chained netstack-buf callbacks. Must be
 * called after Wi-Fi/netif init so lwIP's prior registration is in
 * place before we wrap it. */
void avb_bridge_install_zero_copy_tx(void) {
  esp_wifi_internal_reg_netstack_buf_cb(avb_bridge_netstack_buf_ref,
                                        avb_bridge_netstack_buf_free);
}

/* avb_state_s pointer for the bridge RX path. Set during avb_net_init,
 * read by the EMAC + Wi-Fi RX callbacks to dispatch egress sends. */
static avb_state_s *s_bridge_state = NULL;

/* Map an EtherType to the L2 protocol index used to select a per-port
 * L2TAP fd. Returns -1 for unknown EtherTypes. */
static int avb_bridge_protocol_idx(uint16_t ethertype) {
  switch (ethertype) {
  case 0x22f0:
    return AVTP;
  case 0x22ea:
    return MSRP;
  case 0x88f5:
    return MVRP;
  case 0x8100:
    return VLAN;
  default:
    return -1;
  }
}

/* Forward a raw Ethernet frame to the egress port. Dispatches by the
 * egress port's medium:
 *   - ethernet: esp_eth_transmit on the bridge's EMAC handle. The
 *     frame buffer is borrowed (synchronous DMA enqueue copies into
 *     the descriptor ring, so the caller may free immediately).
 *   - wifi:     esp_wifi_internal_tx on WIFI_IF_AP. We malloc-copy
 *     because the wifi driver may reference the buffer until tx_done;
 *     freeing here is safe because IDF's wifi TX path internally
 *     copies the frame onto its own descriptor (see the matching
 *     comment in avb_net_transmit_raw).
 * Other media drop silently. The (void)ethertype param is kept for
 * future per-ethertype shaping/policing decisions. */
static void avb_bridge_forward(int egress_port, uint16_t ethertype,
                               const uint8_t *frame, uint32_t len) {
  (void)ethertype;
  if (egress_port < 0 || egress_port >= CONFIG_ESP_AVB_NUM_PORTS) {
    return;
  }
  avb_port_s *p = &s_bridge_state->port[egress_port];

  if (p->medium == avb_port_medium_eth_hwts) {
    if (s_bridge_state->config.eth_handle) {
      esp_err_t r = esp_eth_transmit(s_bridge_state->config.eth_handle,
                                     (void *)frame, len);
      if (r == ESP_OK) s_fwd_eth_ok++; else s_fwd_eth_fail++;
    }
    return;
  }

  if (p->medium == avb_port_medium_wifi_ftm) {
    /* esp_wifi_internal_tx copies the buffer into its own DMA-aligned
     * TX queue and returns; the caller can free immediately. Skip the
     * redundant local malloc+memcpy+free that used to sit here — a
     * measurable CPU win on the EMAC RX task at 8000 fps. */
    esp_err_t r = esp_wifi_internal_tx(1 /* WIFI_IF_AP */, (void *)frame, len);
    if (r == ESP_OK) {
      s_fwd_wifi_ok++;
      if (frame[0] & 0x01) s_fwd_wifi_ok_mcast++;
      else                 s_fwd_wifi_ok_ucast++;
    } else {
      s_fwd_wifi_fail++;
    }
    return;
  }
}
#endif /* CONFIG_ESP_AVB_ROLE_BRIDGE */

/* Drop counter no longer used (inline handler has no queue to overflow),
 * but keep the function to avoid breaking callers that read it. */
static volatile uint32_t s_stream_rx_drops = 0;

/* Diagnostic: count PTP frames (0x88f7) that reach avb_unified_rx_cb so we
 * can compare against ptpd's own rx_sync counter. */
static volatile uint32_t s_ptp_rx_seen = 0;
uint32_t avb_net_ptp_rx_seen(void) { return s_ptp_rx_seen; }

/* RX-into-avb_unified_rx_cb breakdown — every frame that enters the
 * unified dispatcher gets tallied so we can localize where a frame is
 * dropped (esp_wifi RX vs. classifier vs. handler). */
static volatile uint32_t s_rx_total = 0;
static volatile uint32_t s_rx_avtp = 0;
static volatile uint32_t s_rx_msrp = 0;
static volatile uint32_t s_rx_mvrp = 0;
static volatile uint32_t s_rx_vlan = 0;
static volatile uint32_t s_rx_other = 0;
/* L2 instrument: every entry into the Wi-Fi RX callback, counted before
 * the malloc that could drop the frame. Compared against s_rx_total
 * (which the dispatcher bumps after a successful malloc): a gap = OOM
 * drops; both frozen while the driver's own RX stats climb = the wifi
 * driver stopped delivering to our callback. Diagnostic for the C6
 * RX-stall. */
static volatile uint32_t s_rx_wifi_cb_entry = 0;
uint32_t avb_net_wifi_rx_cb_count(void) { return s_rx_wifi_cb_entry; }
void avb_net_rx_breakdown(uint32_t *total, uint32_t *avtp, uint32_t *msrp,
                          uint32_t *mvrp, uint32_t *vlan, uint32_t *other) {
  if (total) *total = s_rx_total;
  if (avtp)  *avtp  = s_rx_avtp;
  if (msrp)  *msrp  = s_rx_msrp;
  if (mvrp)  *mvrp  = s_rx_mvrp;
  if (vlan)  *vlan  = s_rx_vlan;
  if (other) *other = s_rx_other;
}

/* Unified EMAC RX callback — dispatches ALL incoming Ethernet frames.
 * Runs in the EMAC RX FreeRTOS task context (not ISR).
 *
 * Routing:
 *   0x8100 (VLAN) → stream handler callback (if registered)
 *   0x22f0 (AVTP) → ctrl_rx_queue (protocol_idx = AVTP)
 *   0x22ea (MSRP) → ctrl_rx_queue (protocol_idx = MSRP)
 *   0x88f5 (MVRP) → ctrl_rx_queue (protocol_idx = MVRP)
 *   0x88f7 (PTP)  → L2TAP filter on EMAC ingress; ptp_handler on Wi-Fi
 *   default        → L2TAP filter then esp_netif_receive (ARP, IP, etc.)
 */
static esp_err_t avb_unified_rx_cb(esp_eth_handle_t eth_handle, uint8_t *buf,
                                   uint32_t len, void *priv, void *info) {
  if (len < ETH_HEADER_LEN) {
    free(buf);
    return ESP_OK;
  }

  /* Read ethertype at offset 12-13 (big-endian) */
  uint16_t ethertype = (buf[12] << 8) | buf[13];

#ifdef CONFIG_ESP_AVB_ROLE_BRIDGE
  /* Hot-path fast-cut for VLAN-tagged AVTP from EMAC → Wi-Fi.
   *
   * This is the highest-rate flow on the bridge (8000 fps for Class A)
   * and runs entirely inside the EMAC RX task. Bypassing the per-frame
   * stats updates, the classifier function call, the disposition struct
   * and the verdict switch trims ~20 instructions per frame; at 8000 pps
   * that recovers measurable CPU on the RX core. The non-fast paths
   * (PTP, MSRP, MVRP, control AVTP, Wi-Fi RX) still get full instrumentation
   * below.
   *
   * Recognised pattern: ingress from EMAC (eth_handle != NULL) AND
   * ethertype 0x8100 (VLAN-tagged frame, which on this network is always
   * AVTP stream data). Egress is the other port (Wi-Fi, port 1).
   *
   * Class A → Wi-Fi is dropped at the data plane (defense-in-depth for
   * the MAP §35.2.4.3 rule) unless allow_class_a_over_wifi is set.
   * Reading PCP is one shift+mask of buf[14]; cheaper than a function
   * call. */
  if (s_bridge_state != NULL && eth_handle != NULL && ethertype == 0x8100 &&
      len >= 16) {
    if (!s_bridge_state->config.allow_class_a_over_wifi) {
      uint8_t pcp = (buf[14] >> 5) & 0x07;
      if (pcp == CONFIG_ESP_AVB_VLAN_PRIO_CLASS_A) {
        free(buf);
        return ESP_OK;
      }
    }
    /* Zero-copy hand-off: pass buf to Wi-Fi by reference with the low
     * bit of netstack_buf set as our discriminator. Our chained free
     * callback (avb_bridge_netstack_buf_free) recognises the tag and
     * frees the buffer once Wi-Fi finishes the actual TX. Saves the
     * IDF-internal memcpy that esp_wifi_internal_tx() would do.
     *
     * On error we deliberately do NOT free here — IDF documents
     * esp_wifi_internal_tx_by_ref as "firstly increases the ref
     * counter, then forwards", meaning the registered free callback
     * has already been promised the buffer. Errors are rare; the
     * counter tracks them. */
    void *nb = (void *)((uintptr_t)buf | BRIDGE_BUF_TAG);
    esp_err_t r = esp_wifi_internal_tx_by_ref(1 /* WIFI_IF_AP */,
                                              (void *)buf, len, nb);
    if (r == ESP_OK) s_fwd_wifi_ok++; else s_fwd_wifi_fail++;
    return ESP_OK;
  }
#endif

  /* Count ALL PTP frames (0x88f7) at entry — before any switch branch
   * and before any early-return. Compared against ptpd's own rx counters
   * to identify where in the path PTP frames are being lost. */
  if (ethertype == 0x88f7) {
    s_ptp_rx_seen++;
  }

  /* Per-ethertype RX breakdown into the dispatcher. */
  s_rx_total++;
  switch (ethertype) {
    case 0x22f0: s_rx_avtp++; break;
    case 0x22ea: s_rx_msrp++; break;
    case 0x88f5: s_rx_mvrp++; break;
    case 0x8100: s_rx_vlan++; break;
    default:     s_rx_other++; break;
  }

#ifdef CONFIG_ESP_AVB_ROLE_BRIDGE
  /* Bridge dispatch for everything the fast-cut above didn't catch
   * — Wi-Fi ingress, plus EMAC ingress of non-VLAN ethertypes
   * (ATDECC control, MSRP, MVRP, anything else needing classification). */
  if (s_bridge_state != NULL) {
    uint8_t pcp = 0;
    if (ethertype == 0x8100 && len >= 16) {
      pcp = (buf[14] >> 5) & 0x07; /* TPID at 12-13, TCI at 14-15; PCP = TCI[15:13] */
    }
    int ingress_port = (eth_handle == NULL) ? 1 : 0;
    avb_bridge_disposition_t d =
        avb_bridge_classify(ingress_port, ethertype, pcp);
    switch (d.verdict) {
    case AVB_BRIDGE_DROP:
      free(buf);
      return ESP_OK;
    case AVB_BRIDGE_BRIDGE:
      avb_bridge_forward(d.egress_port, ethertype, buf, len);
      free(buf);
      return ESP_OK;
    case AVB_BRIDGE_TERMINATE:
    default:
      break; /* fall through to existing per-protocol dispatch */
    }
  }
#endif /* CONFIG_ESP_AVB_ROLE_BRIDGE */

  switch (ethertype) {
  case 0x8100: { /* VLAN — stream data */
    /* Inline handler call — matches the original stable architecture.
     * The queue-based split we tried (emac_rx → queue → AVB-IN task)
     * added ~10 µs per frame of task-wake + context-switch overhead
     * that at 8000 pps cost ~80 ms/sec of extra CPU and drove the NIC
     * DMA ring into overflow. Keeping the handler here means emac_rx
     * does one self-contained pass per frame: alloc → memcpy →
     * handler → free. */
    if (s_stream_handler && len > 18) {
      /* Strip ETH header (14) + VLAN tag (4) = 18 bytes → raw AVTP */
      s_stream_handler(buf + 18, len - 18, s_stream_ctx);
    }
    free(buf);
    return ESP_OK;
  }
  case 0x22f0: /* AVTP (control: ADP, AECP, ACMP, MAAP) */
  case 0x22ea: /* MSRP */
  case 0x88f5: /* MVRP */
  {
    if (s_ctrl_rx_queue) {
      ctrl_rx_pkt_t pkt;
      /* Map ethertype to protocol index */
      switch (ethertype) {
      case 0x22f0:
        pkt.protocol_idx = AVTP;
        break;
      case 0x22ea:
        pkt.protocol_idx = MSRP;
        break;
      case 0x88f5:
        pkt.protocol_idx = MVRP;
        break;
      default:
        pkt.protocol_idx = AVTP;
        break;
      }
      /* Ingress port is derived from the medium that delivered the
       * frame: EMAC -> the port configured as ethernet, Wi-Fi
       * (eth_handle NULL) -> the port configured as wifi. Indices are
       * looked up once at init in avb_net_init so neither callback
       * needs to know the bridge vs. endpoint topology. -1 means
       * "no port configured for this medium" — drop the frame. */
      int ingress = (eth_handle == NULL) ? s_wifi_port_idx : s_eth_port_idx;
      if (ingress < 0) {
        free(buf);
        return ESP_OK;
      }
      pkt.ingress_port = (uint8_t)ingress;
      /* Copy source MAC from offset 6 */
      memcpy(pkt.src_addr, buf + ETH_ADDR_LEN, ETH_ADDR_LEN);
      /* Copy payload (strip ETH header) */
      uint32_t payload_len = len - ETH_HEADER_LEN;
      if (payload_len > AVB_MAX_MSG_LEN)
        payload_len = AVB_MAX_MSG_LEN;
      pkt.length = payload_len;
      memcpy(pkt.data, buf + ETH_HEADER_LEN, payload_len);
      /* Non-blocking send — drop if queue full rather than stalling EMAC */
      xQueueSend(s_ctrl_rx_queue, &pkt, 0);
    }
    /* Control frames are fully handled via ctrl_rx_queue — skip L2TAP
     * filter (nobody reads from L2TAP fds, main loop uses the queue).
     * Must free buf since we're not passing it to esp_netif_receive. */
    free(buf);
    return ESP_OK;
  }
  case 0x88f7: { /* IEEE 1588 / IEEE 802.1AS.
                    Wi-Fi ingress: inject straight into esp_ptp (Wi-Fi
                    netif has no L2TAP backend). EMAC ingress: fan out
                    through L2TAP so the gPTP daemon's L2TAP socket
                    wakes up on poll() — symmetric handoff, just a
                    different downstream API. */
    if (eth_handle == NULL) {
      if (s_wifi_port_idx >= 0 && len >= ETH_HEADER_LEN) {
        (void)ptp_inject_received_frame(s_wifi_port_idx,
                                        buf + ETH_HEADER_LEN,
                                        (uint16_t)(len - ETH_HEADER_LEN));
      }
      free(buf);
      return ESP_OK;
    }
    size_t frame_len = len;
    esp_vfs_l2tap_eth_filter_frame(eth_handle, buf, &frame_len, info);
    if (frame_len > 0) {
      /* No L2TAP fd matched (gPTP daemon not yet up, or socket closed) —
       * free the unconsumed buffer ourselves. */
      free(buf);
    }
    return ESP_OK;
  }
  default: {
    /* ARP, IP, etc.
     *
     * On EMAC ingress (eth_handle != NULL) we pass through the L2TAP
     * filter and then to esp_netif_receive — the host needs the IP
     * stack for management traffic. esp_vfs_l2tap_eth_filter_frame
     * frees buf when it matches a filter (eb_handle=NULL path frees
     * internally), so we must NOT free buf ourselves when
     * frame_len==0.
     *
     * On Wi-Fi ingress (eth_handle == NULL) the AVB-only endpoint
     * has no IP sockets — every broadcast the AP forwards (UDP
     * discovery, ARP, STP, mDNS, etc.) would otherwise queue in
     * lwIP's pbuf chain indefinitely and slowly drain the default
     * heap. Drop them. */
    if (eth_handle == NULL || s_eth_netif == NULL) {
      free(buf);
      return ESP_OK;
    }
    size_t frame_len = len;
    esp_vfs_l2tap_eth_filter_frame(eth_handle, buf, &frame_len, info);
    if (frame_len > 0) {
      return esp_netif_receive(s_eth_netif, buf, frame_len, NULL);
    }
    /* L2TAP consumed and freed buf — nothing more to do */
    return ESP_OK;
  }
  }
}

/* Wi-Fi RX path. esp_wifi delivers each L2 frame via this callback
 * in the Wi-Fi driver's task context. The eb descriptor owns the
 * buffer, so we copy out, free eb, and feed the fresh heap buffer
 * into avb_unified_rx_cb (which assumes ownership and free()s on
 * its way out). One memcpy per frame; acceptable for control-plane
 * rates. The talker fast-path TX bypasses this entirely (it sends
 * via avb_net_transmit_raw → esp_wifi_internal_tx). */
extern esp_err_t esp_wifi_internal_free_rx_buffer(void *eb);
extern esp_err_t esp_wifi_internal_reg_rxcb(
    int wifi_if, esp_err_t (*fn)(void *buffer, uint16_t len, void *eb));

static esp_err_t avb_wifi_rx_cb(void *buffer, uint16_t len, void *eb) {
  s_rx_wifi_cb_entry++; /* L2 counter — bare increment, no logging in hot path */
  uint8_t *buf = malloc(len);
  if (!buf) {
    esp_wifi_internal_free_rx_buffer(eb);
    return ESP_FAIL;
  }
  memcpy(buf, buffer, len);
  esp_wifi_internal_free_rx_buffer(eb);
  return avb_unified_rx_cb(NULL /* no eth_handle */, buf, (uint32_t)len,
                           NULL /* priv */, NULL /* info */);
}

/* Re-register the Wi-Fi RX callback on WIFI_IF_STA. The single rxcb slot
 * can be displaced at runtime — e.g. the default STA netif's reconnect
 * machinery (esp_netif_attach_wifi_station) re-points it at the lwIP
 * path, after which the MAC keeps receiving (L1 climbs) but our
 * dispatcher never runs (L2 frozen). The endpoint's RX-stall watchdog
 * calls this as a light first-line recovery; it restores delivery
 * without the full esp_wifi_stop/start cycle when displacement is the
 * cause. Returns whatever esp_wifi_internal_reg_rxcb reports. */
esp_err_t avb_net_wifi_reregister_rxcb(void) {
  return esp_wifi_internal_reg_rxcb(0 /* WIFI_IF_STA */, avb_wifi_rx_cb);
}

#ifdef CONFIG_ESP_AVB_ROLE_BRIDGE
/* Override esp_wifi_remote's WEAK channel_rx symbol. esp_wifi_remote_net2.c
 * (the variant the P4 host links because CONFIG_ESP_WIFI_ENABLED=y) routes
 * frames coming up from the C6 coprocessor over SDIO into a private static
 * s_rx_fn[] array that's set by its own wifi_start handler — and crucially,
 * esp_wifi_internal_reg_rxcb resolves to the real esp_wifi's variant which
 * writes to a *different* (unused) array. So a normal _reg_rxcb call is a
 * silent no-op on this host.
 *
 * We provide a strong override of esp_wifi_remote_channel_rx that snoops
 * the frame into avb_wifi_rx_cb (which malloc-copies, frees the source
 * buffer, and re-enters avb_unified_rx_cb with eth_handle=NULL — the
 * bridge classifier interprets that as ingress_port=1 → Eth egress).
 *
 * Bridge topology has no STA, so every frame here is from WIFI_IF_AP. */
extern esp_err_t esp_wifi_remote_channel_rx(void *h, void *buffer,
                                            void *buff_to_free, size_t len);
esp_err_t esp_wifi_remote_channel_rx(void *h, void *buffer,
                                     void *buff_to_free, size_t len) {
  (void)h;
  return avb_wifi_rx_cb(buffer, (uint16_t)len, buff_to_free);
}
#endif /* CONFIG_ESP_AVB_ROLE_BRIDGE */

/* Open the four per-protocol L2TAP fds for one port and bind them to
 * the port's interface key. Works for any netif (Eth or Wi-Fi); the
 * caller does the medium-specific MAC / promiscuous setup separately. */
static int avb_net_init_port_l2tap(avb_state_s *state, int port_index) {
  const char *iface = state->port[port_index].eth_interface;
  if (!iface || iface[0] == '\0') {
    avberr("port[%d]: empty interface name", port_index);
    return ERROR;
  }
  for (int i = 0; i < AVB_NUM_PROTOCOLS; i++) {
    int fd = open("/dev/net/tap", 0);
    state->port[port_index].l2if[i] = fd;
    if (fd < 0) {
      avberr("port[%d]: failed to create l2if[%d]: %d", port_index, i, errno);
      return ERROR;
    }
    if (ioctl(fd, L2TAP_S_INTF_DEVICE, iface) < 0) {
      avberr("port[%d]: bind fd %d to %s failed: %d", port_index, fd, iface,
             errno);
      return ERROR;
    }
    uint16_t ethertype;
    switch (i) {
    case AVTP: ethertype = ethertype_avtp; break;
    case MSRP: ethertype = ethertype_msrp; break;
    case MVRP: ethertype = ethertype_mvrp; break;
    case VLAN: ethertype = ethertype_vlan; break;
    default: avberr("Invalid protocol index"); return ERROR;
    }
    if (ioctl(fd, L2TAP_S_RCV_FILTER, &ethertype) < 0) {
      avberr("port[%d]: set ethertype filter on fd %d: %d", port_index, fd,
             errno);
      return ERROR;
    }
    avbinfo("port[%d]: L2TAP fd %d for ethertype %x", port_index, fd,
            ethertype);
  }
  return OK;
}

/* Initialize the network interfaces for all configured ports.
 *
 * Port-0 medium can be ethernet (wired endpoint or bridge) or wifi
 * (e.g. c6 wireless endpoint). On the wifi path we skip L2TAP fds
 * (L2TAP is Ethernet-only) and pull the MAC via esp_netif — the
 * frame TX/RX path uses esp_wifi_internal_* APIs instead, wired
 * later in this function.
 *
 * When NUM_PORTS > 1, port 1 is the second medium (typically wifi
 * for the bridge's SoftAP).
 */
int avb_net_init(avb_state_s *state) {
  /* Cache RX port indices by medium so the unified RX callback can
   * tag ingress without knowing the bridge/endpoint topology. */
  avb_net_cache_port_indices(state);

  /* ---------- Port 0 ---------- */
  if (state->port[0].medium == avb_port_medium_eth_hwts) {
    /* L2TAP fds are kept even on the bridge build: the per-protocol
     * fds are used by the TX path (avb_net_send_to writes the frame
     * via write(l2if[ethertype], ...) which routes to esp_eth_transmit).
     * The L2TAP RX filter is benign here because the netif glue's
     * eth_input_to_netif handler — the only caller of
     * esp_vfs_l2tap_eth_filter_frame — is replaced below by
     * avb_unified_rx_cb via esp_eth_update_input_path_info, so the
     * RX filter never fires. */
    if (avb_net_init_port_l2tap(state, 0) != OK) {
      return ERROR;
    }
    esp_eth_handle_t eth_handle;
    if (ioctl(state->port[0].l2if[0], L2TAP_G_DEVICE_DRV_HNDL, &eth_handle) <
        0) {
      avberr("Failed to get eth_handle: %d", errno);
      return ERROR;
    }
    esp_eth_ioctl(eth_handle, ETH_CMD_G_MAC_ADDR,
                  &state->port[0].internal_mac_addr);
  } else if (state->port[0].medium == avb_port_medium_wifi_ftm) {
    /* Wi-Fi endpoint or AP. No L2TAP — esp_vfs_l2tap is Ethernet-only
     * in IDF. The data-plane RX/TX is wired further down via
     * esp_wifi_internal_*; here we only need the MAC and (if present)
     * the netif handle. The netif is optional on a STA endpoint: when
     * the app skips esp_netif_create_default_wifi_sta() — to stop the
     * default STA netif from re-registering its own RX callback on
     * reassociation and displacing ours — the if_key lookup returns
     * NULL and we read the MAC straight from the Wi-Fi driver. */
    s_eth_netif =
        esp_netif_get_handle_from_ifkey(state->port[0].eth_interface);
    if (s_eth_netif) {
      if (esp_netif_get_mac(s_eth_netif, state->port[0].internal_mac_addr) !=
          ESP_OK) {
        avbwarn("port[0]: esp_netif_get_mac on '%s' failed",
                state->port[0].eth_interface);
      }
    } else if (esp_wifi_get_mac(WIFI_IF_STA,
                                state->port[0].internal_mac_addr) != ESP_OK) {
      avbwarn("port[0]: esp_wifi_get_mac(STA) failed");
    }
    avbinfo("port[0]: WIFI MAC %02x:%02x:%02x:%02x:%02x:%02x",
            state->port[0].internal_mac_addr[0],
            state->port[0].internal_mac_addr[1],
            state->port[0].internal_mac_addr[2],
            state->port[0].internal_mac_addr[3],
            state->port[0].internal_mac_addr[4],
            state->port[0].internal_mac_addr[5]);
  }

  /* ---------- Port 1+ : Wi-Fi (or other media) ---------- */
#if CONFIG_ESP_AVB_NUM_PORTS > 1
  if (state->port[1].medium == avb_port_medium_wifi_ftm) {
    /* Wi-Fi: skip L2TAP (esp_vfs_l2tap is Ethernet-only in IDF) and
     * mirror the port[0]-wifi path — resolve the netif by if_key, pull
     * the MAC via esp_netif. RX/TX wiring happens further down via
     * esp_wifi_internal_reg_rxcb / esp_wifi_internal_tx. */
    s_wifi_netif = esp_netif_get_handle_from_ifkey(
        state->port[1].eth_interface);
    if (!s_wifi_netif) {
      avbwarn("port[1]: no netif for if_key '%s'",
              state->port[1].eth_interface);
    }
    if (s_wifi_netif &&
        esp_netif_get_mac(s_wifi_netif,
                          state->port[1].internal_mac_addr) != ESP_OK) {
      avbwarn("port[1]: esp_netif_get_mac on '%s' failed",
              state->port[1].eth_interface);
    }
    avbinfo("port[1]: WIFI_AP MAC %02x:%02x:%02x:%02x:%02x:%02x",
            state->port[1].internal_mac_addr[0],
            state->port[1].internal_mac_addr[1],
            state->port[1].internal_mac_addr[2],
            state->port[1].internal_mac_addr[3],
            state->port[1].internal_mac_addr[4],
            state->port[1].internal_mac_addr[5]);
  } else if (state->port[1].medium == avb_port_medium_eth_hwts) {
    /* Second Ethernet — same setup pattern as port 0 (not exercised
     * in current bridge designs). */
    if (avb_net_init_port_l2tap(state, 1) != OK) {
      return ERROR;
    }
  }
#endif

  /* Create the control frame RX queue (shared across ports). */
  s_ctrl_rx_queue = xQueueCreate(CTRL_RX_QUEUE_DEPTH, sizeof(ctrl_rx_pkt_t));
  if (!s_ctrl_rx_queue) {
    avberr("Failed to create ctrl_rx_queue");
    return ERROR;
  }
  state->ctrl_rx_queue = s_ctrl_rx_queue;

  /* Register the unified RX callback for port 0. On Ethernet this is
   * the EMAC's input-path hook; on Wi-Fi (c6 endpoint) it's the
   * esp_wifi_internal_reg_rxcb shim that adapts the Wi-Fi RX buffer
   * ownership model and re-enters avb_unified_rx_cb. */
  if (state->port[0].medium == avb_port_medium_eth_hwts) {
    s_eth_netif = esp_netif_get_handle_from_ifkey(state->config.eth_interface);
    esp_eth_update_input_path_info(state->config.eth_handle, avb_unified_rx_cb,
                                   s_eth_netif);
    avbinfo("Unified EMAC RX dispatcher registered");
  } else {
    esp_err_t r = esp_wifi_internal_reg_rxcb(0 /* WIFI_IF_STA */,
                                             avb_wifi_rx_cb);
    if (r != ESP_OK) {
      avberr("esp_wifi_internal_reg_rxcb failed: %s", esp_err_to_name(r));
      return ERROR;
    }
    avbinfo("Unified Wi-Fi RX dispatcher registered on WIFI_IF_STA");
  }

#ifdef CONFIG_ESP_AVB_ROLE_BRIDGE
  s_bridge_state = state;

  /* Bridge port[1] = Wi-Fi AP. Wi-Fi-side ingress is wired up at link
   * time via the WEAK esp_wifi_remote_channel_rx override below
   * (esp_wifi_internal_reg_rxcb doesn't work here — esp_wifi_remote
   * uses esp_wifi_remote_net2.c which has its own private s_rx_fn[]
   * that internal_reg_rxcb doesn't touch, so the registration would
   * be a no-op). All frames coming up from the C6 coprocessor arrive
   * via esp_wifi_remote_channel_rx → avb_wifi_rx_cb → avb_unified_rx_cb
   * with eth_handle=NULL (= ingress_port 1). */
#if CONFIG_ESP_AVB_NUM_PORTS > 1
  if (state->port[1].medium == avb_port_medium_wifi_ftm) {
    avbinfo("Bridge L2 forwarder armed: Eth(port0) <-> Wi-Fi-AP(port1) "
            "(via esp_wifi_remote_channel_rx override)");
  } else {
    avbwarn("Bridge build but port[1] medium is not wifi; "
            "Wi-Fi-side ingress will not bridge");
  }
#endif
#endif /* CONFIG_ESP_AVB_ROLE_BRIDGE */

  return OK;
}

/* Create an Ethernet frame */
void avb_create_eth_frame(uint8_t *eth_frame, eth_addr_t *dest_addr,
                          avb_state_s *state, ethertype_t ethertype, void *msg,
                          uint16_t msg_len, uint8_t *vlan_id) {
  struct eth_hdr eth_hdr = {.type = htons(ethertype)};
  uint16_t vid = vlan_id ? octets_to_uint(vlan_id, 2) : 0;
  uint16_t prio = state->msrp_mappings[0].priority;
  if (vlan_id) {
    for (uint16_t i = 0; i < state->msrp_mappings_count; i++) {
      if (memcmp(vlan_id, state->msrp_mappings[i].vlan_id, 2) == 0) {
        prio = state->msrp_mappings[i].priority;
        break;
      }
    }
  }
  struct eth_vlan_hdr eth_vlan_hdr = {.prio_vid =
                                          htons((prio << 13) | (vid & 0x0FFF)),
                                      .tpid = htons(ethertype_avtp)};
  memcpy(&eth_hdr.dest.addr, dest_addr, ETH_ADDR_LEN);
  memcpy(&eth_hdr.src.addr, state->port[0].internal_mac_addr, ETH_ADDR_LEN);
  memcpy(eth_frame, &eth_hdr, sizeof(eth_hdr));
  if (ethertype == ethertype_vlan) {
    memcpy(eth_frame + sizeof(eth_hdr), &eth_vlan_hdr, sizeof(eth_vlan_hdr));
    memcpy(eth_frame + sizeof(eth_hdr) + sizeof(eth_vlan_hdr), msg, msg_len);
  } else {
    memcpy(eth_frame + sizeof(eth_hdr), msg, msg_len);
  }
}

/* Send an Ethernet frame */
int avb_net_send_to(avb_state_s *state, ethertype_t ethertype, void *msg,
                    uint16_t msg_len, struct timespec *ts,
                    eth_addr_t *dest_addr) {
  /* Match the guard in avb_net_send_on: skip TX silently when port 0
   * has no carrier so periodic ADP / MAAP / AECP cycles do not spam
   * the failure log. Single source of truth is esp_ptp. */
  if (!ptpd_port_link_up(0)) {
    return 0;
  }

  uint8_t eth_frame[msg_len + ETH_HEADER_LEN];

  // Create the Ethernet frame
  avb_create_eth_frame(eth_frame, dest_addr, state, ethertype, msg, msg_len,
                       NULL);

  /* Wi-Fi port: L2TAP is Ethernet-only, so we bypass the per-ethertype
   * fds and push the frame straight at the Wi-Fi driver. */
  if (state->port[0].medium == avb_port_medium_wifi_ftm) {
    return (avb_net_transmit_raw(NULL, eth_frame, sizeof(eth_frame)) == ESP_OK)
               ? (int)sizeof(eth_frame)
               : ERROR;
  }

  // Get the L2IF for the given ethertype
  int l2if;
  switch (ethertype) {
  case ethertype_avtp:
    l2if = state->port[0].l2if[AVTP];
    break;
  case ethertype_msrp:
    l2if = state->port[0].l2if[MSRP];
    break;
  case ethertype_mvrp:
    l2if = state->port[0].l2if[MVRP];
    break;
  case ethertype_vlan:
    l2if = state->port[0].l2if[VLAN];
    break;
  default:
    avberr("Invalid ethertype: %d", ethertype);
    return ERROR;
  }

  int ret = write(l2if, eth_frame, sizeof(eth_frame));
  return ret;
}

/* Send an Ethernet frame with VLAN ID */
int avb_net_send_to_vlan(avb_state_s *state, ethertype_t ethertype, void *msg,
                         uint16_t msg_len, struct timespec *ts,
                         eth_addr_t *dest_addr, uint8_t *vlan_id) {
  if (!ptpd_port_link_up(0)) {
    return 0;
  }

  uint8_t eth_frame[msg_len + ETH_HEADER_LEN + sizeof(struct eth_vlan_hdr)];

  // Create the Ethernet frame
  avb_create_eth_frame(eth_frame, dest_addr, state, ethertype, msg, msg_len,
                       vlan_id);

  /* Wi-Fi port: L2TAP is Ethernet-only — go straight to wifi raw TX. */
  if (state->port[0].medium == avb_port_medium_wifi_ftm) {
    return (avb_net_transmit_raw(NULL, eth_frame, sizeof(eth_frame)) == ESP_OK)
               ? (int)sizeof(eth_frame)
               : ERROR;
  }

  int l2if;
  switch (ethertype) {
  case ethertype_avtp:
    l2if = state->port[0].l2if[AVTP];
    break;
  case ethertype_msrp:
    l2if = state->port[0].l2if[MSRP];
    break;
  case ethertype_mvrp:
    l2if = state->port[0].l2if[MVRP];
    break;
  case ethertype_vlan:
    l2if = state->port[0].l2if[VLAN];
    break;
  default:
    avberr("Invalid ethertype: %d", ethertype);
    return ERROR;
  }

  int ret = write(l2if, eth_frame, sizeof(eth_frame));
  return ret;
}

int avb_net_send(avb_state_s *state, ethertype_t ethertype, void *msg,
                 uint16_t msg_len, struct timespec *ts) {
  eth_addr_t dest_addr;

  // Set destination address based on ethertype
  switch (ethertype) {
  case ethertype_avtp:
    uint8_t subtype;
    memcpy(&subtype, msg, 1);
    if (subtype == avtp_subtype_maap) {
      memcpy(&dest_addr, &MAAP_MCAST_MAC_ADDR, ETH_ADDR_LEN);
    } else {
      memcpy(&dest_addr, &BCAST_MAC_ADDR, ETH_ADDR_LEN);
    }
    break;
  case ethertype_msrp:
    memcpy(&dest_addr, &LLDP_MCAST_MAC_ADDR, ETH_ADDR_LEN);
    break;
  case ethertype_mvrp:
    memcpy(&dest_addr, &SPANTREE_MAC_ADDR, ETH_ADDR_LEN);
    break;
  default:
    avberr("Invalid ethertype: %d", ethertype);
    return ERROR;
  }
  return avb_net_send_to(state, ethertype, msg, msg_len, ts, &dest_addr);
}

/* Forward decl repeated here so endpoint builds (which don't define
 * CONFIG_ESP_AVB_ROLE_BRIDGE) can still see the symbol. esp_avb avoids
 * a hard esp_wifi PRIV_REQUIRES; native Wi-Fi or esp_wifi_remote
 * provides the link symbol when it's actually called. */
extern esp_err_t esp_wifi_internal_tx(int wifi_if, void *buffer, size_t len);

/* Per-port control-plane TX. Routes the frame through the named port's
 * egress, building it with that port's own source MAC. Dispatch is
 * uniform across ports: medium picks the egress (eth → L2TAP write,
 * wifi → esp_wifi_internal_tx) and wifi_mode (sta=0 / ap=1) selects
 * the IDF Wi-Fi interface — no port-index assumptions and no hardcoded
 * WIFI_IF. */
int avb_net_send_on(avb_state_s *state, int port_index,
                    ethertype_t ethertype, void *msg, uint16_t msg_len,
                    struct timespec *ts) {
  if (port_index < 0 || port_index >= CONFIG_ESP_AVB_NUM_PORTS) {
    avberr("avb_net_send_on: invalid port %d", port_index);
    return ERROR;
  }
  (void)ts;

  avb_port_s *p = &state->port[port_index];

  /* Skip TX when the port is not carrying frames. The periodic
   * cycles (MSRP / MVRP / ADP) tick continuously while STA is
   * disassociated or the ETH cable is unplugged; without this gate
   * each one would call into esp_wifi_internal_tx / L2TAP write
   * with no carrier and spam the failure log at multi-Hz. Single
   * source of truth lives in esp_ptp — it already owns the per-port
   * topology and event handlers. Returning 0 (zero bytes sent) lets
   * the callers' `if (ret < 0)` failure check pass silently. */
  if (!ptpd_port_link_up(port_index)) {
    return 0;
  }

  /* Derive dest_addr, mirroring avb_net_send. */
  eth_addr_t dest_addr;
  switch (ethertype) {
  case ethertype_avtp: {
    uint8_t subtype;
    memcpy(&subtype, msg, 1);
    if (subtype == avtp_subtype_maap) {
      memcpy(&dest_addr, &MAAP_MCAST_MAC_ADDR, ETH_ADDR_LEN);
    } else {
      memcpy(&dest_addr, &BCAST_MAC_ADDR, ETH_ADDR_LEN);
    }
    break;
  }
  case ethertype_msrp:
    memcpy(&dest_addr, &LLDP_MCAST_MAC_ADDR, ETH_ADDR_LEN);
    break;
  case ethertype_mvrp:
    memcpy(&dest_addr, &SPANTREE_MAC_ADDR, ETH_ADDR_LEN);
    break;
  default:
    avberr("avb_net_send_on: invalid ethertype %d", ethertype);
    return ERROR;
  }

  /* avb_create_eth_frame hardcodes port[0]'s MAC as SA; overwrite
   * bytes 6..11 with this port's own MAC. */
  uint8_t eth_frame[msg_len + ETH_HEADER_LEN];
  avb_create_eth_frame(eth_frame, &dest_addr, state, ethertype, msg, msg_len,
                       NULL);
  memcpy(eth_frame + ETH_ADDR_LEN, p->internal_mac_addr, ETH_ADDR_LEN);

  if (p->medium == avb_port_medium_wifi_ftm) {
    /* Use this port's wifi_mode (numerically WIFI_IF_STA=0 or
     * WIFI_IF_AP=1) as the IDF wifi_if argument. Heap-copy the
     * buffer — some IDF wifi builds hold a reference until tx_done. */
    if (p->wifi_mode == avb_port_wifi_mode_none) {
      avberr("avb_net_send_on: port %d medium=wifi but wifi_mode=none",
             port_index);
      return ERROR;
    }
    void *buf = malloc(sizeof(eth_frame));
    if (!buf) return ERROR;
    memcpy(buf, eth_frame, sizeof(eth_frame));
    esp_err_t r =
        esp_wifi_internal_tx((int)p->wifi_mode, buf, sizeof(eth_frame));
    free(buf);
    return (r == ESP_OK) ? (int)sizeof(eth_frame) : ERROR;
  }

  if (p->medium == avb_port_medium_eth_hwts) {
    int l2if;
    switch (ethertype) {
    case ethertype_avtp: l2if = p->l2if[AVTP]; break;
    case ethertype_msrp: l2if = p->l2if[MSRP]; break;
    case ethertype_mvrp: l2if = p->l2if[MVRP]; break;
    default:
      avberr("avb_net_send_on: invalid ethertype %d", ethertype);
      return ERROR;
    }
    return write(l2if, eth_frame, sizeof(eth_frame));
  }

  return ERROR;
}

/* Receive next control frame from the unified EMAC RX dispatcher.
 * Blocks up to timeout_ms. Returns payload length, or 0 on timeout.
 * protocol_idx is set to AVTP/MSRP/MVRP. */
int avb_net_recv_ctrl(avb_state_s *state, int *protocol_idx, int *ingress_port,
                      void *msg, uint16_t msg_len, eth_addr_t *src_addr,
                      int timeout_ms) {
  ctrl_rx_pkt_t pkt;
  if (xQueueReceive(s_ctrl_rx_queue, &pkt, pdMS_TO_TICKS(timeout_ms)) !=
      pdTRUE) {
    return 0; /* timeout — no frame available */
  }
  *protocol_idx = pkt.protocol_idx;
  *ingress_port = pkt.ingress_port;
  memcpy(src_addr, pkt.src_addr, ETH_ADDR_LEN);
  uint16_t copy_len = pkt.length < msg_len ? pkt.length : msg_len;
  memcpy(msg, pkt.data, copy_len);
  return copy_len;
}

/* esp_wifi_internal_tx — raw L2 frame send on a Wi-Fi interface.
 * Declared in esp_private/wifi.h but we forward-declare here so
 * esp_avb doesn't gain a hard esp_wifi PRIV_REQUIRES. The link
 * resolves on Wi-Fi-capable targets (esp_wifi is in the build via
 * the application's own deps); on Ethernet-only targets like
 * esp32p4 the symbol is provided by esp_wifi_remote's stub layer
 * and would error at runtime if called — but we only call it from
 * the wifi branch below, which an ethernet build never enters. */
extern esp_err_t esp_wifi_internal_tx(int wifi_if, void *buffer, size_t len);

/* Fast-path raw frame TX. See avb.h docstring for rationale.
 *
 * Branch on the caller's eth_handle: a non-NULL handle means the
 * port has an EMAC-backed transport (esp_eth — wired endpoint or
 * bridge), and we go through esp_eth_transmit for direct DMA. A NULL
 * handle means the wifi data-plane (c6 wireless endpoint, Phase
 * 6b.2), and we shoot the frame at the Wi-Fi STA interface via
 * esp_wifi_internal_tx. */
esp_err_t avb_net_transmit_raw(esp_eth_handle_t eth_handle,
                               const void *frame, size_t frame_len) {
  if (eth_handle != NULL) {
    return esp_eth_transmit(eth_handle, (void *)frame, frame_len);
  }
  /* Wi-Fi path: esp_wifi_internal_tx synchronously enqueues the buffer
   * to the Wi-Fi MAC's TX descriptor ring. Some IDF builds reference
   * the caller's buffer until tx_done — passing a stack buffer (as
   * avb_net_send_to does for control frames) is a use-after-stack-pop
   * footgun. Copy into the heap so the buffer outlives the caller's
   * stack frame; the wifi driver will free or we let it leak per its
   * ownership model. To stay safe, malloc + esp_wifi_internal_tx +
   * free on error path; if tx succeeds, the wifi driver currently
   * copies internally so freeing here is OK. */
  /* Wi-Fi path: esp_wifi_internal_tx synchronously enqueues to the
   * wifi MAC's TX descriptors. Some IDF builds reference the caller's
   * buffer until tx_done — copy into heap so the buffer outlives the
   * caller's stack frame. */
  void *buf = malloc(frame_len);
  if (!buf) {
    return ESP_ERR_NO_MEM;
  }
  memcpy(buf, frame, frame_len);
  esp_err_t r = esp_wifi_internal_tx(0 /* WIFI_IF_STA */, buf, frame_len);
  free(buf);
  return r;
}

/* Register stream RX handler. Invoked inline from the EMAC RX callback
 * context (avb_unified_rx_cb VLAN branch), so the handler must be fast
 * (~25 µs at 8000 pps). Pass NULL to unregister. */
void avb_net_set_stream_rx_handler(avb_stream_rx_handler_t handler, void *ctx) {
  s_stream_ctx = ctx;
  /* Write handler last with memory barrier semantics —
   * the callback checks s_stream_handler != NULL as gate */
  s_stream_handler = handler;
}

uint32_t avb_net_stream_rx_drops(void) { return s_stream_rx_drops; }
