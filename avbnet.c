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
#include "mrp.h"
#include "esp_ptp.h"
#include "esp_cpu.h"
#include "esp_rom_sys.h"
#include "esp_timer.h"
#include "driver/gptimer.h"
#include <stdatomic.h>
#include <esp_netif.h>
#include <esp_vfs_l2tap.h>
#include <esp_idf_version.h>
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
/* Port 0 MAC, cached at init for the dispatcher's DA guard. All-zero
 * until avb_net_init runs (guard passes everything until then). */
static uint8_t s_own_mac[ETH_ADDR_LEN] = {0};
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
/* Free entries in the hosted data queue to the coprocessor, absent when
 * Wi-Fi is not hosted, and the entries a stream leaves to control frames. */
extern int esp_hosted_data_tx_queue_space(void) __attribute__((weak));
#define AVB_WIFI_CONTROL_RESERVE 4
#ifdef CONFIG_ESP_AVB_WIFI_UNICAST_STREAMS
/* Stream frames whose destination was re-addressed for the Wi-Fi hop,
 * and those left alone because no Listener declaration resolved. */
static volatile uint32_t s_fwd_wifi_readdressed, s_fwd_wifi_no_mapping;
/* Frames dropped at the Wi-Fi in-flight cap (backpressure). */
static volatile uint32_t s_fwd_wifi_bp_drop;
/* Uplink stream frames whose BSSID-addressed destination was restored
 * to the talker's advertised MAAP address on wired egress. */
static volatile uint32_t s_fwd_eth_restored;
#endif

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

#ifdef CONFIG_ESP_AVB_WIFI_UNICAST_STREAMS
void avb_bridge_forward_stats_readdress(uint32_t *readdressed,
                                        uint32_t *no_mapping) {
  if (readdressed) *readdressed = s_fwd_wifi_readdressed;
  if (no_mapping)  *no_mapping  = s_fwd_wifi_no_mapping;
}

uint32_t avb_bridge_forward_stats_bp_drop(void) { return s_fwd_wifi_bp_drop; }

uint32_t avb_bridge_forward_stats_restored(void) { return s_fwd_eth_restored; }
#endif

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

/* Station stream pacing. Wi-Fi hands the bridge a station's stream in
 * bursts of up to an A-MPDU, which forwarded as received exceed the
 * stream's TSpec on the wire, and a downstream shaper holding them to
 * the reserved rate keeps the backlog as a standing queue (measured 8
 * to 30 ms through one switch). 802.1BA 6.4 leaves that smoothing to
 * the wireless hop: each frame of a reserved station stream goes out
 * no sooner than its class measurement interval over its
 * MaxIntervalFrames after the previous one. A frame already that far
 * apart passes straight through; a burst waits in the stream's queue.
 * Queues live in PSRAM, the bridge's internal RAM has no room. */
#define STATION_PACER_STREAMS 8
#define STATION_PACER_DEPTH 64
#define STATION_PACER_SLOT_BYTES 1522
#define STATION_PACER_MAX_FRAMES 8 /* MaxIntervalFrames honored up to this */
#define STATION_PACER_IDLE_US 2000000

typedef struct {
  bool valid;
  uint8_t stream_id[UNIQUE_ID_LEN];
  int egress_port;
  int64_t interval_ns;
  uint8_t max_frames;
  /* Send times of the last max_frames frames; sent_ns[sent_idx] is the
   * oldest, and the next frame may go one interval after it. */
  int64_t sent_ns[STATION_PACER_MAX_FRAMES];
  uint8_t sent_idx;
  int64_t last_seen_us;
  uint16_t head;
  uint16_t count;
  uint16_t in_flight; /* taken off the queue, not yet transmitted */
  uint16_t len[STATION_PACER_DEPTH];
  uint8_t (*slot)[STATION_PACER_SLOT_BYTES];
} station_pacer_t;

static station_pacer_t *s_pacers;
static portMUX_TYPE s_pacer_lock = portMUX_INITIALIZER_UNLOCKED;
static bool s_pacer_draining;
/* A GPTimer alarm wakes the pacer task: esp_timer callbacks share the
 * bridge's busiest core and ran too late to drain a burst between
 * Wi-Fi deliveries. */
static gptimer_handle_t s_pacer_timer;
static TaskHandle_t s_pacer_task;
/* gptimer_set_alarm_action() refuses a config outside internal RAM when
 * CONFIG_GPTIMER_CTRL_FUNC_IN_IRAM is set, and the forward path drains
 * on an esp-hosted thread whose stack is in PSRAM with
 * CONFIG_ESP_HOSTED_DFLT_TASK_FROM_SPIRAM, so the config is static.
 * Written under s_pacer_lock. */
static DRAM_ATTR gptimer_alarm_config_t s_pacer_alarm;
/* Alarms fired, pacer task wakeups, queued frames released by the pacer
 * task and by the forward path, and alarm arms that failed. */
static volatile uint32_t s_pacer_alarms, s_pacer_wakeups;
static volatile uint32_t s_pacer_task_sent, s_pacer_forward_sent;
static volatile uint32_t s_pacer_arm_fail;
static volatile uint32_t s_fwd_eth_paced, s_fwd_eth_pace_overflow;
static volatile uint32_t s_pacer_depth_max;
static volatile uint32_t s_pacer_no_tspec, s_pacer_no_entry;

static inline int64_t station_pacer_next_ns(const station_pacer_t *pacer) {
  return pacer->sent_ns[pacer->sent_idx] + pacer->interval_ns;
}

static inline void station_pacer_note_send(station_pacer_t *pacer,
                                           int64_t now_ns) {
  pacer->sent_ns[pacer->sent_idx] = now_ns;
  pacer->sent_idx = (uint8_t)((pacer->sent_idx + 1) % pacer->max_frames);
}

/* Send every queued frame whose turn has come, then arm the timer for
 * the next one. One context drains at a time, so a stream's frames
 * leave in order; transmits happen outside the lock. Returns the frames
 * sent. */
static uint32_t station_pacer_drain(void) {
  portENTER_CRITICAL(&s_pacer_lock);
  if (s_pacer_draining) {
    portEXIT_CRITICAL(&s_pacer_lock);
    return 0;
  }
  s_pacer_draining = true;
  portEXIT_CRITICAL(&s_pacer_lock);
  uint32_t sent = 0;
  int64_t earliest_ns;
  for (;;) {
    int64_t now_ns = esp_timer_get_time() * 1000;
    station_pacer_t *due = NULL;
    const uint8_t *frame = NULL;
    uint16_t len = 0;
    earliest_ns = INT64_MAX;
    portENTER_CRITICAL(&s_pacer_lock);
    for (int i = 0; i < STATION_PACER_STREAMS && !due; i++) {
      station_pacer_t *pacer = &s_pacers[i];
      if (!pacer->valid || pacer->count == 0)
        continue;
      int64_t next_ns = station_pacer_next_ns(pacer);
      if (now_ns >= next_ns) {
        due = pacer;
        frame = pacer->slot[pacer->head];
        len = pacer->len[pacer->head];
        pacer->head = (uint16_t)((pacer->head + 1) % STATION_PACER_DEPTH);
        pacer->count--;
        pacer->in_flight++;
        station_pacer_note_send(pacer, now_ns);
      } else if (next_ns < earliest_ns) {
        earliest_ns = next_ns;
      }
    }
    if (!due)
      s_pacer_draining = false;
    portEXIT_CRITICAL(&s_pacer_lock);
    if (!due)
      break;
    avb_bridge_forward(due->egress_port, 0x8100, frame, len);
    sent++;
    portENTER_CRITICAL(&s_pacer_lock);
    due->in_flight--;
    portEXIT_CRITICAL(&s_pacer_lock);
  }
  if (earliest_ns != INT64_MAX) {
    int64_t delay_us = (earliest_ns - esp_timer_get_time() * 1000 + 999) / 1000;
    uint64_t now_ticks = 0;
    gptimer_get_raw_count(s_pacer_timer, &now_ticks);
    portENTER_CRITICAL(&s_pacer_lock);
    s_pacer_alarm.alarm_count =
        now_ticks + (uint64_t)(delay_us < 5 ? 5 : delay_us);
    esp_err_t armed = gptimer_set_alarm_action(s_pacer_timer, &s_pacer_alarm);
    portEXIT_CRITICAL(&s_pacer_lock);
    if (armed != ESP_OK)
      s_pacer_arm_fail++;
  }
  return sent;
}

static bool IRAM_ATTR
station_pacer_alarm(gptimer_handle_t timer,
                    const gptimer_alarm_event_data_t *event, void *arg) {
  (void)timer;
  (void)event;
  (void)arg;
  BaseType_t woken = pdFALSE;
  s_pacer_alarms++;
  vTaskNotifyGiveFromISR(s_pacer_task, &woken);
  return woken == pdTRUE;
}

static void station_pacer_task(void *arg) {
  (void)arg;
  for (;;) {
    ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
    s_pacer_wakeups++;
    s_pacer_task_sent += station_pacer_drain();
  }
}

static bool station_pacer_init(void) {
  s_pacers = heap_caps_calloc(STATION_PACER_STREAMS, sizeof(*s_pacers),
                              MALLOC_CAP_SPIRAM);
  if (!s_pacers)
    return false;
  gptimer_config_t timer_config = {.clk_src = GPTIMER_CLK_SRC_DEFAULT,
                                   .direction = GPTIMER_COUNT_UP,
                                   .resolution_hz = 1000000};
  gptimer_event_callbacks_t callbacks = {.on_alarm = station_pacer_alarm};
  if (xTaskCreate(station_pacer_task, "station_pacer", 3072, NULL, 23,
                  &s_pacer_task) != pdPASS ||
      gptimer_new_timer(&timer_config, &s_pacer_timer) != ESP_OK ||
      gptimer_register_event_callbacks(s_pacer_timer, &callbacks, NULL) !=
          ESP_OK ||
      gptimer_enable(s_pacer_timer) != ESP_OK ||
      gptimer_start(s_pacer_timer) != ESP_OK) {
    avberr("station pacer: timer setup failed, forwarding unpaced");
    return false;
  }
  return true;
}

/* Forward a station stream frame through its pacer. Returns false when
 * the frame is not a reserved stream (or pacing is unavailable), so the
 * caller forwards it as usual. */
static bool station_pacer_forward(int egress_port, const uint8_t *frame,
                                  uint32_t len) {
  /* VLAN-tagged AVTP stream data with a valid stream_id. */
  if (len < 30 || len > STATION_PACER_SLOT_BYTES || frame[16] != 0x22 ||
      frame[17] != 0xf0 || (frame[18] & 0x80) || !(frame[19] & 0x80) ||
      s_wifi_port_idx < 0)
    return false;
  uint16_t max_frame_size = 0, max_interval_frames = 0;
  if (!mrp_talker_advertise_tspec(s_wifi_port_idx,
                                  (const unique_id_t *)(frame + 22),
                                  &max_frame_size, &max_interval_frames)) {
    s_pacer_no_tspec++;
    return false;
  }
  if (max_interval_frames == 0)
    max_interval_frames = 1;
  if (max_interval_frames > STATION_PACER_MAX_FRAMES)
    max_interval_frames = STATION_PACER_MAX_FRAMES;
  /* PCP 3 is SR class A (125 us), otherwise class B (250 us). */
  int64_t interval_ns = (frame[14] >> 5) == 3 ? 125000 : 250000;

  static bool init_tried, init_ok;
  if (!init_tried) {
    init_tried = true;
    init_ok = station_pacer_init();
  }
  if (!init_ok)
    return false;

  int64_t now_us = esp_timer_get_time();
  station_pacer_t *pacer = NULL, *spare = NULL;
  portENTER_CRITICAL(&s_pacer_lock);
  for (int i = 0; i < STATION_PACER_STREAMS; i++) {
    station_pacer_t *candidate = &s_pacers[i];
    if (candidate->valid &&
        memcmp(candidate->stream_id, frame + 22, UNIQUE_ID_LEN) == 0) {
      pacer = candidate;
      break;
    }
    if (!spare && (!candidate->valid ||
                   (candidate->count == 0 && candidate->in_flight == 0 &&
                    now_us - candidate->last_seen_us > STATION_PACER_IDLE_US)))
      spare = candidate;
  }
  if (!pacer && spare && spare->slot) {
    pacer = spare;
    pacer->valid = true;
    memcpy(pacer->stream_id, frame + 22, UNIQUE_ID_LEN);
    pacer->head = pacer->count = 0;
    pacer->max_frames = 0; /* forces the send history reset below */
  }
  portEXIT_CRITICAL(&s_pacer_lock);
  if (!pacer) {
    /* Allocate a free entry's queue outside the lock, pace from the
     * next frame on. */
    if (spare && !spare->slot)
      spare->slot = heap_caps_malloc(
          (size_t)STATION_PACER_DEPTH * STATION_PACER_SLOT_BYTES,
          MALLOC_CAP_SPIRAM);
    s_pacer_no_entry++;
    return false;
  }

  int64_t now_ns = now_us * 1000;
  portENTER_CRITICAL(&s_pacer_lock);
  pacer->egress_port = egress_port;
  pacer->interval_ns = interval_ns;
  if (pacer->max_frames != max_interval_frames) {
    pacer->max_frames = (uint8_t)max_interval_frames;
    memset(pacer->sent_ns, 0, sizeof(pacer->sent_ns));
    pacer->sent_idx = 0;
  }
  pacer->last_seen_us = now_us;
  portEXIT_CRITICAL(&s_pacer_lock);
  for (;;) {
    bool send_now = false, send_head = false;
    const uint8_t *head_frame = NULL;
    uint16_t head_len = 0;
    portENTER_CRITICAL(&s_pacer_lock);
    if (pacer->count == 0 && pacer->in_flight == 0 && !s_pacer_draining &&
        now_ns >= station_pacer_next_ns(pacer)) {
      send_now = true;
      pacer->in_flight++;
      station_pacer_note_send(pacer, now_ns);
    } else if (pacer->count + pacer->in_flight < STATION_PACER_DEPTH) {
      int tail = (pacer->head + pacer->count) % STATION_PACER_DEPTH;
      memcpy(pacer->slot[tail], frame, len);
      pacer->len[tail] = (uint16_t)len;
      pacer->count++;
      s_fwd_eth_paced++;
      if (pacer->count > s_pacer_depth_max)
        s_pacer_depth_max = pacer->count;
    } else if (pacer->count > 0) {
      /* Full: let the oldest go early, then queue this one behind the
       * rest once its slot is free. Sending this one now would put it
       * ahead of the whole queue. */
      send_head = true;
      head_frame = pacer->slot[pacer->head];
      head_len = pacer->len[pacer->head];
      pacer->head = (uint16_t)((pacer->head + 1) % STATION_PACER_DEPTH);
      pacer->count--;
      pacer->in_flight++;
      station_pacer_note_send(pacer, now_ns);
      s_fwd_eth_pace_overflow++;
    } else {
      send_now = true; /* every slot is being transmitted */
      pacer->in_flight++;
    }
    portEXIT_CRITICAL(&s_pacer_lock);
    if (send_head) {
      avb_bridge_forward(egress_port, 0x8100, head_frame, head_len);
      portENTER_CRITICAL(&s_pacer_lock);
      pacer->in_flight--;
      portEXIT_CRITICAL(&s_pacer_lock);
      continue;
    }
    if (send_now) {
      avb_bridge_forward(egress_port, 0x8100, frame, len);
      portENTER_CRITICAL(&s_pacer_lock);
      pacer->in_flight--;
      portEXIT_CRITICAL(&s_pacer_lock);
    }
    break;
  }
  s_pacer_forward_sent += station_pacer_drain();
  return true;
}

void avb_bridge_pacer_bypass_stats(uint32_t *no_tspec, uint32_t *no_entry) {
  if (no_tspec) *no_tspec = s_pacer_no_tspec;
  if (no_entry) *no_entry = s_pacer_no_entry;
}

void avb_bridge_pacer_wake_stats(uint32_t *alarms, uint32_t *wakeups,
                                 uint32_t *task_sent, uint32_t *forward_sent,
                                 uint32_t *arm_fail) {
  if (alarms) *alarms = s_pacer_alarms;
  if (wakeups) *wakeups = s_pacer_wakeups;
  if (task_sent) *task_sent = s_pacer_task_sent;
  if (forward_sent) *forward_sent = s_pacer_forward_sent;
  if (arm_fail) *arm_fail = s_pacer_arm_fail;
}

void avb_bridge_forward_stats_paced(uint32_t *paced, uint32_t *overflow,
                                    uint32_t *depth_max) {
  if (paced) *paced = s_fwd_eth_paced;
  if (overflow) *overflow = s_fwd_eth_pace_overflow;
  if (depth_max) {
    *depth_max = s_pacer_depth_max;
    s_pacer_depth_max = 0;
  }
}

/* AVB Wireless §3.5: a group addressed frame toward a station is neither
 * acknowledged nor retried, so ACMP toward the Wi-Fi port also goes,
 * individually addressed, to the station that runs the entity it is for.
 * Which station runs an entity comes from the source address of its ADP
 * advertisements on the Wi-Fi port. */
#define AVB_WIFI_ENTITY_TABLE_SIZE 16

typedef struct {
  unique_id_t entity_id;
  eth_addr_t station;
  int64_t expires_us; /* 0 for a free slot */
} avb_wifi_entity_t;

static avb_wifi_entity_t s_wifi_entities[AVB_WIFI_ENTITY_TABLE_SIZE];
static portMUX_TYPE s_wifi_entities_lock = portMUX_INITIALIZER_UNLOCKED;
static volatile uint32_t s_fwd_wifi_acmp_copies;

uint32_t avb_bridge_forward_stats_acmp_copies(void) {
  return s_fwd_wifi_acmp_copies;
}

static void avb_wifi_entity_learn(const uint8_t *frame, uint32_t len) {
  if (len < ETH_HEADER_LEN + 12)
    return;
  const atdecc_header_s *header =
      (const atdecc_header_s *)(frame + ETH_HEADER_LEN);
  const uint8_t *entity_id = frame + ETH_HEADER_LEN + 4;
  bool departing = header->msg_type == adp_msg_type_entity_departing;
  if (header->msg_type != adp_msg_type_entity_available && !departing)
    return;
  static const unique_id_t zero_id = {0};
  if (memcmp(entity_id, zero_id, UNIQUE_ID_LEN) == 0)
    return;
  int64_t now_us = esp_timer_get_time();
  /* valid_time counts 2 s units. */
  int64_t expires_us = now_us + (int64_t)header->status_valtime * 2000000;
  portENTER_CRITICAL(&s_wifi_entities_lock);
  avb_wifi_entity_t *slot = NULL;
  avb_wifi_entity_t *oldest = &s_wifi_entities[0];
  for (int i = 0; i < AVB_WIFI_ENTITY_TABLE_SIZE; i++) {
    avb_wifi_entity_t *entity = &s_wifi_entities[i];
    if (entity->expires_us != 0 &&
        memcmp(entity->entity_id, entity_id, UNIQUE_ID_LEN) == 0) {
      slot = entity;
      break;
    }
    if (entity->expires_us < oldest->expires_us)
      oldest = entity;
  }
  if (departing) {
    if (slot != NULL)
      slot->expires_us = 0;
  } else {
    if (slot == NULL)
      slot = oldest;
    memcpy(slot->entity_id, entity_id, UNIQUE_ID_LEN);
    memcpy(slot->station, frame + ETH_ADDR_LEN, ETH_ADDR_LEN);
    slot->expires_us = expires_us;
  }
  portEXIT_CRITICAL(&s_wifi_entities_lock);
}

static bool avb_wifi_entity_station(const uint8_t *entity_id,
                                    uint8_t *station) {
  int64_t now_us = esp_timer_get_time();
  bool found = false;
  portENTER_CRITICAL(&s_wifi_entities_lock);
  for (int i = 0; i < AVB_WIFI_ENTITY_TABLE_SIZE; i++) {
    const avb_wifi_entity_t *entity = &s_wifi_entities[i];
    if (entity->expires_us > now_us &&
        memcmp(entity->entity_id, entity_id, UNIQUE_ID_LEN) == 0) {
      memcpy(station, entity->station, ETH_ADDR_LEN);
      found = true;
      break;
    }
  }
  portEXIT_CRITICAL(&s_wifi_entities_lock);
  return found;
}

/* After the group addressed frame went out: learn from ADP that came in
 * on the Wi-Fi port, and copy ACMP that went out on it to its station.
 * The frame is the caller's to free, and its destination is rewritten. */
static void avb_bridge_wifi_control(int ingress_port, int egress_port,
                                    uint8_t *frame, uint32_t len) {
  if (len <= ETH_HEADER_LEN)
    return;
  uint8_t subtype = frame[ETH_HEADER_LEN];
  if (subtype == avtp_subtype_adp && ingress_port == s_wifi_port_idx) {
    avb_wifi_entity_learn(frame, len);
    return;
  }
  if (subtype != avtp_subtype_acmp || egress_port != s_wifi_port_idx ||
      len < ETH_HEADER_LEN + sizeof(acmp_message_s) || !(frame[0] & 0x01))
    return;
  const uint8_t *entity_id = acmp_target_entity_id(
      (const acmp_message_s *)(frame + ETH_HEADER_LEN));
  uint8_t station[ETH_ADDR_LEN];
  if (entity_id == NULL || !avb_wifi_entity_station(entity_id, station))
    return;
  memcpy(frame, station, ETH_ADDR_LEN);
  if (esp_wifi_internal_tx(1 /* WIFI_IF_AP */, frame, len) == ESP_OK)
    s_fwd_wifi_acmp_copies++;
  else
    s_fwd_wifi_fail++;
}
#endif /* CONFIG_ESP_AVB_ROLE_BRIDGE */

/* Drop counter no longer used (inline handler has no queue to overflow),
 * but keep the function to avoid breaking callers that read it. */
static volatile uint32_t s_stream_rx_drops = 0;

/* Diagnostic: count PTP frames (0x88f7) that reach avb_unified_rx_cb so we
 * can compare against ptpd's own rx_sync counter. */
static volatile uint32_t s_ptp_rx_seen = 0;
uint32_t avb_net_ptp_rx_seen(void) { return s_ptp_rx_seen; }

/* PTP-frame L2TAP handoff outcome counters (wedge diagnosis): after
 * esp_vfs_l2tap_eth_filter_frame, frame_len==0 means an fd matched and
 * consumed the frame; frame_len>0 means no fd matched (dropped here). */
extern void ptpd_note_wired_peer_follow_up(const uint8_t *message, size_t length)
    __attribute__((weak));
static volatile uint32_t s_ptp_l2tap_consumed = 0;
static volatile uint32_t s_ptp_l2tap_unmatched = 0;
uint32_t avb_net_ptp_l2tap_consumed(void) { return s_ptp_l2tap_consumed; }
uint32_t avb_net_ptp_l2tap_unmatched(void) { return s_ptp_l2tap_unmatched; }

/* RX-into-avb_unified_rx_cb breakdown — every frame that enters the
 * unified dispatcher gets tallied so we can localize where a frame is
 * dropped (esp_wifi RX vs. classifier vs. handler). */
static volatile uint32_t s_rx_total = 0;
static volatile uint32_t s_rx_avtp = 0;
static volatile uint32_t s_rx_msrp = 0;
static volatile uint32_t s_rx_mvrp = 0;
static volatile uint32_t s_rx_vlan = 0;
static volatile uint32_t s_rx_other = 0;
/* EMAC frames of an ethertype nobody here takes, dropped on arrival. */
static volatile uint32_t s_rx_dropped_other = 0;

uint32_t avb_net_rx_dropped_other(void) { return s_rx_dropped_other; }
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
/* Hot-path cycle profiler: total dispatcher cycles and stream-handler
 * cycles, accumulated per frame; read+reset by the stats printer to
 * localize emac_rx CPU burn (our callback vs the esp_eth driver). */
static volatile uint32_t s_prof_cb_frames = 0;
static volatile uint32_t s_prof_cb_cycles = 0;
static volatile uint32_t s_prof_stream_frames = 0;
static volatile uint32_t s_prof_stream_cycles = 0;
void avb_net_rx_prof(uint32_t *cb_frames, uint32_t *cb_cycles,
                     uint32_t *stream_frames, uint32_t *stream_cycles) {
  if (cb_frames) *cb_frames = s_prof_cb_frames;
  if (cb_cycles) *cb_cycles = s_prof_cb_cycles;
  if (stream_frames) *stream_frames = s_prof_stream_frames;
  if (stream_cycles) *stream_cycles = s_prof_stream_cycles;
  s_prof_cb_frames = s_prof_cb_cycles = 0;
  s_prof_stream_frames = s_prof_stream_cycles = 0;
}

static esp_err_t avb_unified_rx_cb_inner(esp_eth_handle_t eth_handle,
                                         uint8_t *buf, uint32_t len,
                                         void *priv, void *info);

/* Wi-Fi stream bundle: several AVTPDUs of one stream in one 802.11
 * frame between the bridge and a station. Every 802.11 frame pays a
 * fixed contention, preamble and acknowledgment cost, so a Class B
 * stream's 4000 small frames a second take most of the channel. A
 * bundle is an AVTP experimental stream PDU after the frame's own
 * Ethernet and VLAN header:
 *   18  subtype 0x7F   19 sv   20 seq low   21 0   22..29 stream_id
 *   30..33 "AVBW"   34..35 bundle seq   36 count   37 flags
 *   38.. count x { length (2), AVTPDU }
 * The receiver rebuilds each original frame (same header) and handles
 * it as if it had arrived alone. */
#define WIFI_BUNDLE_SUBTYPE 0x7F
#define WIFI_BUNDLE_AVTP 18      /* AVTP header offset in the frame */
#define WIFI_BUNDLE_ENTRIES 38   /* first entry offset */
#define WIFI_BUNDLE_MAX_BYTES 1500
#define WIFI_BUNDLE_FLAG_RETX 0x01
#define WIFI_BUNDLE_FLAG_COMPACT 0x02

/* Wi-Fi access category. The C6 picks a frame's 802.11 TID from IPv4
 * precedence and sends everything else as best effort, so a bundle sent
 * inside an IPv4 header with precedence 5 (protocol 253, never handed to
 * an IP stack) goes out as video: shorter AIFS and contention window, and
 * still aggregated. Receivers accept both forms. Bundles leave room for
 * the header so the wrapped frame still fits one esp-hosted buffer. */
#define WIFI_VI_HEADER_LEN 22 /* ethertype 0x0800 + IPv4 header */
#define WIFI_VI_IP_PROTOCOL 253
#define WIFI_VI_IP_ID 0x4156
#define WIFI_BUNDLE_FILL_BYTES (WIFI_BUNDLE_MAX_BYTES - WIFI_VI_HEADER_LEN)
#if CONFIG_ESP_AVB_WIFI_VIDEO_AC
static volatile bool s_wifi_vi_wrap = true;
#else
static volatile bool s_wifi_vi_wrap;
#endif
static SemaphoreHandle_t s_wifi_vi_lock;
static uint8_t *s_wifi_vi_buf;
static volatile uint32_t s_wifi_vi_tx, s_wifi_vi_rx;

static void wifi_vi_init(void) {
  static portMUX_TYPE init_lock = portMUX_INITIALIZER_UNLOCKED;
  if (s_wifi_vi_lock)
    return;
  SemaphoreHandle_t lock = xSemaphoreCreateMutex();
  uint8_t *buf = heap_caps_malloc(WIFI_BUNDLE_MAX_BYTES + WIFI_VI_HEADER_LEN,
                                  MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT);
  if (!lock || !buf) {
    if (lock)
      vSemaphoreDelete(lock);
    free(buf);
    return;
  }
  portENTER_CRITICAL(&init_lock);
  bool installed = !s_wifi_vi_lock;
  if (installed) {
    s_wifi_vi_buf = buf;
    s_wifi_vi_lock = lock;
  }
  portEXIT_CRITICAL(&init_lock);
  if (!installed) {
    vSemaphoreDelete(lock);
    free(buf);
  }
}

extern esp_err_t esp_wifi_internal_tx(int wifi_if, void *buffer, size_t len);

/* Sends a bundle, wrapped for the video access category when enabled. */
static esp_err_t wifi_tx_bundle_frame(int wifi_if, const uint8_t *frame,
                                      uint32_t len) {
  if (!s_wifi_vi_wrap)
    return esp_wifi_internal_tx(wifi_if, (void *)frame, len);
  wifi_vi_init();
  if (!s_wifi_vi_lock || len < 14 || len > WIFI_BUNDLE_MAX_BYTES)
    return esp_wifi_internal_tx(wifi_if, (void *)frame, len);
  xSemaphoreTake(s_wifi_vi_lock, portMAX_DELAY);
  uint8_t *out = s_wifi_vi_buf;
  uint8_t *ip = out + 14;
  uint16_t ip_len = (uint16_t)(len - 12 + WIFI_VI_HEADER_LEN - 2);
  memcpy(out, frame, 12);
  out[12] = 0x08;
  out[13] = 0x00;
  memset(ip, 0, 20);
  ip[0] = 0x45;
  ip[1] = 0xa0; /* precedence 5 */
  ip[2] = (uint8_t)(ip_len >> 8);
  ip[3] = (uint8_t)ip_len;
  ip[4] = (uint8_t)(WIFI_VI_IP_ID >> 8);
  ip[5] = (uint8_t)WIFI_VI_IP_ID;
  ip[6] = 0x40;
  ip[8] = 1;
  ip[9] = WIFI_VI_IP_PROTOCOL;
  uint32_t sum = 0;
  for (int word = 0; word < 20; word += 2)
    sum += (uint32_t)(ip[word] << 8 | ip[word + 1]);
  sum = (sum & 0xffff) + (sum >> 16);
  sum = (sum & 0xffff) + (sum >> 16);
  ip[10] = (uint8_t)(~sum >> 8);
  ip[11] = (uint8_t)~sum;
  memcpy(out + 12 + WIFI_VI_HEADER_LEN, frame + 12, len - 12);
  esp_err_t result =
      esp_wifi_internal_tx(wifi_if, out, len + WIFI_VI_HEADER_LEN);
  xSemaphoreGive(s_wifi_vi_lock);
  if (result == ESP_OK)
    s_wifi_vi_tx++;
  return result;
}

/* Restores a wrapped frame in place and returns its length. */
static uint32_t wifi_vi_unwrap(uint8_t *buf, uint32_t len) {
  if (len < 12 + WIFI_VI_HEADER_LEN + 4 || buf[12] != 0x08 || buf[13] != 0x00 ||
      buf[14] != 0x45 || buf[23] != WIFI_VI_IP_PROTOCOL ||
      buf[18] != (uint8_t)(WIFI_VI_IP_ID >> 8) ||
      buf[19] != (uint8_t)WIFI_VI_IP_ID)
    return len;
  memmove(buf + 12, buf + 12 + WIFI_VI_HEADER_LEN, len - 12 - WIFI_VI_HEADER_LEN);
  s_wifi_vi_rx++;
  return len - WIFI_VI_HEADER_LEN;
}

#if CONFIG_ESP_AVB_WIFI_VIDEO_AC_AB
/* Bench A/B: switch the wrapper every 6 log intervals of 10 s. */
static void wifi_vi_ab_task(void *arg) {
  (void)arg;
  wifi_vi_init();
  for (unsigned interval = 1;; interval++) {
    vTaskDelay(pdMS_TO_TICKS(10000));
    ESP_LOGI(TAG, "vi_ab: ab=%d tx=%" PRIu32 " rx=%" PRIu32, s_wifi_vi_wrap,
             s_wifi_vi_tx, s_wifi_vi_rx);
    if (!(interval % 6))
      s_wifi_vi_wrap = !s_wifi_vi_wrap;
  }
}
#endif
static const uint8_t s_wifi_bundle_tag[4] = {'A', 'V', 'B', 'W'};
static volatile uint32_t s_wifi_bundles_rx, s_wifi_bundle_frames_rx,
    s_wifi_split_fail;

static bool wifi_bundle_is(const uint8_t *buf, uint32_t len) {
  return len >= WIFI_BUNDLE_ENTRIES && buf[12] == 0x81 && buf[13] == 0x00 &&
         buf[16] == 0x22 && buf[17] == 0xf0 &&
         buf[WIFI_BUNDLE_AVTP] == WIFI_BUNDLE_SUBTYPE &&
         memcmp(buf + 30, s_wifi_bundle_tag, 4) == 0;
}

/* Compact bundle body: the first frame's 24-octet stream header as a
 * template, then per frame
 *   flags (1)   seq (1)   avtp_timestamp (4)   payload length (2)
 *   [own 24-octet header, flag 2]   payload
 * with 32-bit AAF samples carried as their upper 24 bits when every low
 * octet is zero (flag 1). */
#define WIFI_COMPACT_HEADER 24
#define WIFI_COMPACT_FIXED 8
#define WIFI_COMPACT_PACKED 0x01
#define WIFI_COMPACT_OWN_HEADER 0x02

/* Flags for one AVTPDU against the bundle's template header. */
static __attribute__((unused)) uint8_t
wifi_compact_flags(const uint8_t *tmpl, const uint8_t *avtp, uint32_t avtp_len) {
  uint8_t flags = 0;
  if (avtp_len < WIFI_COMPACT_HEADER || tmpl[0] != avtp[0] ||
      tmpl[1] != avtp[1] || memcmp(tmpl + 3, avtp + 3, 9) != 0 ||
      memcmp(tmpl + 16, avtp + 16, WIFI_COMPACT_HEADER - 16) != 0)
    flags |= WIFI_COMPACT_OWN_HEADER;
  uint32_t payload_len = avtp_len - WIFI_COMPACT_HEADER;
  /* AAF, INT_32BIT samples */
  if (avtp_len >= WIFI_COMPACT_HEADER && avtp[0] == 0x02 &&
      avtp[16] == 0x02 && payload_len % 4 == 0) {
    const uint8_t *sample = avtp + WIFI_COMPACT_HEADER;
    uint8_t low = 0;
    for (uint32_t i = 3; i < payload_len; i += 4)
      low |= sample[i];
    if (low == 0)
      flags |= WIFI_COMPACT_PACKED;
  }
  return flags;
}

static uint32_t wifi_compact_entry_len(uint8_t flags, uint32_t avtp_len) {
  uint32_t payload_len = avtp_len - WIFI_COMPACT_HEADER;
  return WIFI_COMPACT_FIXED +
         ((flags & WIFI_COMPACT_OWN_HEADER) ? WIFI_COMPACT_HEADER : 0) +
         ((flags & WIFI_COMPACT_PACKED) ? payload_len / 4 * 3 : payload_len);
}

/* Write one entry; returns its length. */
static __attribute__((unused)) uint32_t
wifi_compact_put(uint8_t *out, uint8_t flags, const uint8_t *avtp,
                 uint32_t avtp_len) {
  uint32_t payload_len = avtp_len - WIFI_COMPACT_HEADER;
  uint8_t *pos = out;
  *pos++ = flags;
  *pos++ = avtp[2];
  memcpy(pos, avtp + 12, 4);
  pos += 4;
  *pos++ = (uint8_t)(payload_len >> 8);
  *pos++ = (uint8_t)payload_len;
  if (flags & WIFI_COMPACT_OWN_HEADER) {
    memcpy(pos, avtp, WIFI_COMPACT_HEADER);
    pos += WIFI_COMPACT_HEADER;
  }
  const uint8_t *payload = avtp + WIFI_COMPACT_HEADER;
  if (flags & WIFI_COMPACT_PACKED) {
    for (uint32_t i = 0; i < payload_len; i += 4) {
      *pos++ = payload[i];
      *pos++ = payload[i + 1];
      *pos++ = payload[i + 2];
    }
  } else {
    memcpy(pos, payload, payload_len);
    pos += payload_len;
  }
  return (uint32_t)(pos - out);
}

/* Split a compact bundle; same contract as wifi_bundle_split. */
static void wifi_bundle_split_compact(uint8_t *buf, uint32_t len,
                                      void *priv, void *info) {
  int count = buf[36];
  const uint8_t *tmpl = buf + WIFI_BUNDLE_ENTRIES;
  uint32_t off = WIFI_BUNDLE_ENTRIES + WIFI_COMPACT_HEADER;
  for (int i = 0; i < count && off + WIFI_COMPACT_FIXED <= len; i++) {
    const uint8_t *entry = buf + off;
    uint8_t flags = entry[0];
    uint32_t payload_len = ((uint32_t)entry[6] << 8) | entry[7];
    uint32_t entry_len =
        wifi_compact_entry_len(flags, WIFI_COMPACT_HEADER + payload_len);
    if (off + entry_len > len ||
        ((flags & WIFI_COMPACT_PACKED) && payload_len % 4 != 0))
      break;
    uint32_t frame_len = WIFI_BUNDLE_AVTP + WIFI_COMPACT_HEADER + payload_len;
    uint8_t *frame = malloc(frame_len);
    if (frame == NULL) {
      s_wifi_split_fail += count - i;
      break;
    }
    const uint8_t *src = entry + WIFI_COMPACT_FIXED;
    uint8_t *header = frame + WIFI_BUNDLE_AVTP;
    memcpy(frame, buf, WIFI_BUNDLE_AVTP);
    if (flags & WIFI_COMPACT_OWN_HEADER) {
      memcpy(header, src, WIFI_COMPACT_HEADER);
      src += WIFI_COMPACT_HEADER;
    } else {
      memcpy(header, tmpl, WIFI_COMPACT_HEADER);
    }
    header[2] = entry[1];
    memcpy(header + 12, entry + 2, 4);
    uint8_t *payload = header + WIFI_COMPACT_HEADER;
    if (flags & WIFI_COMPACT_PACKED) {
      for (uint32_t k = 0; k < payload_len; k += 4) {
        payload[k] = *src++;
        payload[k + 1] = *src++;
        payload[k + 2] = *src++;
        payload[k + 3] = 0;
      }
    } else {
      memcpy(payload, src, payload_len);
    }
    off += entry_len;
    s_wifi_bundle_frames_rx++;
    avb_unified_rx_cb_inner(NULL, frame, frame_len, priv, info);
  }
  free(buf);
}

/* Split a received bundle into its frames and hand each to the normal
 * receive path. Consumes buf. */
static void wifi_bundle_split(uint8_t *buf, uint32_t len, void *priv,
                              void *info) {
  uint32_t off = WIFI_BUNDLE_ENTRIES;
  int count = buf[36];
  s_wifi_bundles_rx++;
  if ((buf[37] & WIFI_BUNDLE_FLAG_COMPACT) &&
      len >= WIFI_BUNDLE_ENTRIES + WIFI_COMPACT_HEADER) {
    wifi_bundle_split_compact(buf, len, priv, info);
    return;
  }
  for (int i = 0; i < count && off + 2 <= len; i++) {
    uint32_t avtp_len = ((uint32_t)buf[off] << 8) | buf[off + 1];
    off += 2;
    if (avtp_len < 12 || off + avtp_len > len)
      break;
    uint8_t *frame = malloc(WIFI_BUNDLE_AVTP + avtp_len);
    if (frame == NULL) {
      s_wifi_split_fail += count - i;
      break;
    }
    memcpy(frame, buf, WIFI_BUNDLE_AVTP);
    memcpy(frame + WIFI_BUNDLE_AVTP, buf + off, avtp_len);
    off += avtp_len;
    s_wifi_bundle_frames_rx++;
    avb_unified_rx_cb_inner(NULL, frame, WIFI_BUNDLE_AVTP + avtp_len, priv,
                            info);
  }
  free(buf);
}

#ifdef CONFIG_ESP_AVB_WIFI_BUNDLE_COMPACT
#define WIFI_BUNDLE_MIN_AVTP WIFI_COMPACT_HEADER
#define WIFI_BUNDLE_FIRST_COST(avtp_len)                                       \
  (WIFI_BUNDLE_ENTRIES + WIFI_COMPACT_HEADER + WIFI_COMPACT_FIXED + (avtp_len))
#define WIFI_BUNDLE_ENTRY_COST(buf, count, avtp, avtp_len)                     \
  ((count) ? wifi_compact_entry_len(                                           \
                 wifi_compact_flags((buf) + WIFI_BUNDLE_ENTRIES, avtp,         \
                                    avtp_len),                                 \
                 avtp_len)                                                     \
           : 0)
#else
#define WIFI_BUNDLE_MIN_AVTP 12
#define WIFI_BUNDLE_FIRST_COST(avtp_len) (WIFI_BUNDLE_ENTRIES + 2 + (avtp_len))
#define WIFI_BUNDLE_ENTRY_COST(buf, count, avtp, avtp_len) (2 + (avtp_len))
#endif

/* Start a bundle in out for the stream of frame; returns its length. */
static __attribute__((unused)) uint32_t wifi_bundle_begin(uint8_t *out,
                                                          const uint8_t *frame) {
  memcpy(out, frame, WIFI_BUNDLE_AVTP); /* Ethernet, VLAN, ethertype */
  memset(out + WIFI_BUNDLE_AVTP, 0, WIFI_BUNDLE_ENTRIES - WIFI_BUNDLE_AVTP);
  out[WIFI_BUNDLE_AVTP] = WIFI_BUNDLE_SUBTYPE;
  out[WIFI_BUNDLE_AVTP + 1] = 0x80; /* sv */
  memcpy(out + 22, frame + 22, UNIQUE_ID_LEN);
  memcpy(out + 30, s_wifi_bundle_tag, 4);
#ifdef CONFIG_ESP_AVB_WIFI_BUNDLE_COMPACT
  out[37] = WIFI_BUNDLE_FLAG_COMPACT;
  memcpy(out + WIFI_BUNDLE_ENTRIES, frame + WIFI_BUNDLE_AVTP,
         WIFI_COMPACT_HEADER);
  return WIFI_BUNDLE_ENTRIES + WIFI_COMPACT_HEADER;
#else
  return WIFI_BUNDLE_ENTRIES;
#endif
}

/* Append one AVTPDU at offset len of the bundle; returns the octets added. */
static __attribute__((unused)) uint32_t wifi_bundle_put(uint8_t *out,
                                                        uint32_t len,
                                                        const uint8_t *avtp,
                                                        uint32_t avtp_len) {
#ifdef CONFIG_ESP_AVB_WIFI_BUNDLE_COMPACT
  uint8_t flags = wifi_compact_flags(out + WIFI_BUNDLE_ENTRIES, avtp, avtp_len);
  return wifi_compact_put(out + len, flags, avtp, avtp_len);
#else
  out[len] = (uint8_t)(avtp_len >> 8);
  out[len + 1] = (uint8_t)avtp_len;
  memcpy(out + len + 2, avtp, avtp_len);
  return 2 + avtp_len;
#endif
}

void avb_wifi_bundle_rx_stats(uint32_t *bundles, uint32_t *frames,
                              uint32_t *split_fail) {
  if (bundles) *bundles = s_wifi_bundles_rx;
  if (frames) *frames = s_wifi_bundle_frames_rx;
  if (split_fail) *split_fail = s_wifi_split_fail;
}

/* Retransmission on the Wi-Fi hop. A receiver that sees a gap in a
 * stream's bundle sequence holds the bundles after it, asks the sender
 * for the missing ones with a NACK, and delivers in order once they
 * arrive. When the hold time runs out it moves on and the listener
 * conceals the gap as before. The NACK is an untagged AVTP experimental
 * stream PDU:
 *   14 subtype 0x7F   15 sv   18..25 stream_id   26..29 "AVBN"
 *   30..31 first missing bundle seq   32 count
 *   33..34 time the receiver still waits, 100 us units, 0 unknown */
#define WIFI_NACK_AVTP 14
#define WIFI_NACK_FRAME_LEN 60
#ifndef CONFIG_ESP_AVB_WIFI_RETX_HOLD_MAX_US
#define CONFIG_ESP_AVB_WIFI_RETX_HOLD_MAX_US 40000
#endif
#ifndef CONFIG_ESP_AVB_WIFI_RETX_DOWNSTREAM_LATENCY_US
#define CONFIG_ESP_AVB_WIFI_RETX_DOWNSTREAM_LATENCY_US 0
#endif
#define WIFI_BUNDLE_HOLD 32
#define WIFI_BUNDLE_HOLD_US CONFIG_ESP_AVB_WIFI_RETX_HOLD_MAX_US
/* Held this long past their deadline, bundles belong to a stopped run. */
#define WIFI_BUNDLE_STALE_US 200000
/* A NACK and its answer each cross a loaded FIFO; repeating sooner only
 * duplicates the answer still on its way. */
#define WIFI_NACK_REPEAT_US 15000
/* Below this a copy cannot cross the hop and still be played. */
#define WIFI_RETX_MIN_BUDGET_US 3000
/* A NACK's waiting time less its own trip to the sender. */
#define WIFI_NACK_TRIP_US 2000
#define WIFI_NACK_BUDGET 33
#define WIFI_RX_STREAMS 4
static const uint8_t s_wifi_nack_tag[4] = {'A', 'V', 'B', 'N'};

typedef struct {
  uint16_t seq;
  uint32_t len;
  uint8_t *buf;
} wifi_held_bundle_t;

typedef struct {
  bool valid;
  bool seeded;
  uint8_t stream_id[UNIQUE_ID_LEN];
  uint8_t peer[ETH_ADDR_LEN]; /* sender of the bundles */
  int64_t last_rx_us;
  uint16_t expected;
  uint16_t nack_next; /* first bundle seq not NACKed yet */
  int64_t deadline_us; /* 0 while nothing is held */
  int64_t last_nack_us;
  int held_count;
  wifi_held_bundle_t held[WIFI_BUNDLE_HOLD];
} wifi_bundle_rx_t;

static wifi_bundle_rx_t s_bundle_rx[WIFI_RX_STREAMS];
static volatile uint32_t s_wifi_nacks_tx, s_wifi_recovered, s_wifi_given_up,
    s_wifi_dup_rx, s_wifi_nacks_skipped;

#if !defined(CONFIG_ESP_AVB_ROLE_BRIDGE) && defined(CONFIG_ESP_AVB_WIFI_BUNDLE)
static void wifi_tx_nack_handle(const uint8_t *nack);
#endif

static bool wifi_nack_is(const uint8_t *buf, uint32_t len) {
  return len >= WIFI_NACK_AVTP + 19 && buf[12] == 0x22 && buf[13] == 0xf0 &&
         buf[WIFI_NACK_AVTP] == WIFI_BUNDLE_SUBTYPE &&
         memcmp(buf + 26, s_wifi_nack_tag, 4) == 0;
}

/* At most this many NACKs a second: under sustained loss they cannot
 * help, and would only take more of the channel (loss bursts ran 30 to
 * 50 gaps a second and recovered about 1 %). Called from the Wi-Fi
 * receive task only. */
#define WIFI_NACK_MAX_PER_S 20

static bool wifi_nack_rate_ok(void) {
  static int64_t window_us;
  static int sent_in_window;
  int64_t now_us = esp_timer_get_time();
  if (now_us - window_us >= 1000000) {
    window_us = now_us;
    sent_in_window = 0;
  }
  if (sent_in_window >= WIFI_NACK_MAX_PER_S)
    return false;
  sent_in_window++;
  return true;
}

#if !defined(CONFIG_ESP_AVB_ROLE_BRIDGE)
/* A station sends NACKs to the bridge's Wi-Fi address with the
 * locally administered bit set: frames for the AP's own address end in
 * the coprocessor and never reach the bridge. */
static void wifi_nack_send(const wifi_bundle_rx_t *rx, uint16_t first,
                           int count, int64_t budget_us) {
  if (!wifi_nack_rate_ok())
    return;
  wifi_ap_record_t ap;
  if (esp_wifi_sta_get_ap_info(&ap) != ESP_OK)
    return;
  uint8_t frame[WIFI_NACK_FRAME_LEN] = {0};
  memcpy(frame, ap.bssid, ETH_ADDR_LEN);
  frame[0] |= 0x02;
  memcpy(frame + ETH_ADDR_LEN, s_own_mac, ETH_ADDR_LEN);
  frame[12] = 0x22;
  frame[13] = 0xf0;
  frame[WIFI_NACK_AVTP] = WIFI_BUNDLE_SUBTYPE;
  frame[WIFI_NACK_AVTP + 1] = 0x80;
  memcpy(frame + 18, rx->stream_id, UNIQUE_ID_LEN);
  memcpy(frame + 26, s_wifi_nack_tag, 4);
  frame[30] = (uint8_t)(first >> 8);
  frame[31] = (uint8_t)first;
  frame[32] = (uint8_t)(count > 255 ? 255 : count);
  int64_t units = budget_us / 100;
  if (units > 0xFFFF)
    units = 0xFFFF;
  frame[WIFI_NACK_BUDGET] = (uint8_t)(units >> 8);
  frame[WIFI_NACK_BUDGET + 1] = (uint8_t)units;
  if (avb_net_transmit_raw(NULL, frame, sizeof(frame)) == ESP_OK)
    s_wifi_nacks_tx++;
}
#else
/* The bridge NACKs a station talker's uplink bundles at the station's
 * own address. */
static void wifi_nack_send(const wifi_bundle_rx_t *rx, uint16_t first,
                           int count, int64_t budget_us) {
  if (!wifi_nack_rate_ok())
    return;
  uint8_t frame[WIFI_NACK_FRAME_LEN] = {0};
  memcpy(frame, rx->peer, ETH_ADDR_LEN);
  memcpy(frame + ETH_ADDR_LEN, s_own_mac, ETH_ADDR_LEN);
  frame[12] = 0x22;
  frame[13] = 0xf0;
  frame[WIFI_NACK_AVTP] = WIFI_BUNDLE_SUBTYPE;
  frame[WIFI_NACK_AVTP + 1] = 0x80;
  memcpy(frame + 18, rx->stream_id, UNIQUE_ID_LEN);
  memcpy(frame + 26, s_wifi_nack_tag, 4);
  frame[30] = (uint8_t)(first >> 8);
  frame[31] = (uint8_t)first;
  frame[32] = (uint8_t)(count > 255 ? 255 : count);
  int64_t units = budget_us / 100;
  if (units > 0xFFFF)
    units = 0xFFFF;
  frame[WIFI_NACK_BUDGET] = (uint8_t)(units >> 8);
  frame[WIFI_NACK_BUDGET + 1] = (uint8_t)units;
  if (esp_wifi_internal_tx(1 /* WIFI_IF_AP */, frame, sizeof(frame)) == ESP_OK)
    s_wifi_nacks_tx++;
}
#endif

/* A gap is held for a retransmission only while the frames behind it
 * can still be played on time. A local listener plays the audio queued
 * before the gap first, so the gap may wait until that runs out less a
 * margin; its playout runs tens of ms behind presentation time on Wi-Fi,
 * so presentation time alone left only the 1 ms floor. Without a local
 * listener (the bridge) the bound is presentation time plus the latency
 * the downstream listener is assumed to add. Never longer than
 * WIFI_BUNDLE_HOLD_US. */
#define WIFI_HOLD_PLAYOUT_MARGIN_US 3000
#define WIFI_HOLD_MIN_US 1000

/* Presentation time of a bundle's first frame; false when it has none. */
static bool wifi_bundle_first_pts(const uint8_t *buf, uint32_t len,
                                  uint32_t *pts) {
  const uint8_t *stamp;
  if (buf[37] & WIFI_BUNDLE_FLAG_COMPACT) {
    uint32_t off = WIFI_BUNDLE_ENTRIES + WIFI_COMPACT_HEADER;
    if (len < off + WIFI_COMPACT_FIXED || !(buf[WIFI_BUNDLE_ENTRIES + 1] & 0x01))
      return false;
    stamp = buf + off + 2;
  } else {
    if (len < WIFI_BUNDLE_ENTRIES + 2 + 16 ||
        !(buf[WIFI_BUNDLE_ENTRIES + 2 + 1] & 0x01))
      return false;
    stamp = buf + WIFI_BUNDLE_ENTRIES + 2 + 12;
  }
  *pts = ((uint32_t)stamp[0] << 24) | ((uint32_t)stamp[1] << 16) |
         ((uint32_t)stamp[2] << 8) | (uint32_t)stamp[3];
  return true;
}

/* How long the earliest held frame may still wait, from presentation
 * time; false when that is unknown. */
static bool wifi_bundle_pts_budget(const wifi_bundle_rx_t *rx,
                                   int64_t *budget_us) {
  int lowest = -1;
  for (int i = 0; i < rx->held_count; i++)
    if (lowest < 0 || (int16_t)(rx->held[i].seq - rx->held[lowest].seq) < 0)
      lowest = i;
  uint32_t pts;
  struct timespec clock_now;
  if (lowest < 0 ||
      !wifi_bundle_first_pts(rx->held[lowest].buf, rx->held[lowest].len,
                             &pts) ||
      ptpd_now(&clock_now) != 0)
    return false;
  uint32_t now32 = (uint32_t)((uint64_t)clock_now.tv_sec * 1000000000ULL +
                              (uint64_t)clock_now.tv_nsec);
  int32_t lead_ns = (int32_t)(pts - now32);
  if (lead_ns < -400000000 || lead_ns > 400000000)
    return false; /* not on this clock's timeline */
  *budget_us = lead_ns / 1000 + CONFIG_ESP_AVB_WIFI_RETX_DOWNSTREAM_LATENCY_US -
               WIFI_HOLD_PLAYOUT_MARGIN_US;
  return true;
}

static int64_t wifi_bundle_hold_until(const wifi_bundle_rx_t *rx,
                                      int64_t now_us) {
  int64_t budget_us = WIFI_BUNDLE_HOLD_US;
#if !defined(CONFIG_ESP_AVB_ROLE_BRIDGE)
  int32_t buffered_us;
  if (avb_stream_in_buffered_us(rx->stream_id, &buffered_us))
    budget_us = (int64_t)buffered_us - WIFI_HOLD_PLAYOUT_MARGIN_US;
  else
#endif
    wifi_bundle_pts_budget(rx, &budget_us);
  if (budget_us < WIFI_HOLD_MIN_US)
    budget_us = WIFI_HOLD_MIN_US;
  if (budget_us > WIFI_BUNDLE_HOLD_US)
    budget_us = WIFI_BUNDLE_HOLD_US;
  return now_us + budget_us;
}

/* Bundles given up per gap, by gap length: 1, 2, 3, 4, 5-8, 9-16, more
 * (a sender restart counts as more). */
static volatile uint32_t s_wifi_gap_hist[7];

static void wifi_gap_hist_add(uint32_t gap) {
  int bucket = gap <= 4 ? (int)gap - 1 : gap <= 8 ? 4 : gap <= 16 ? 5 : 6;
  if (bucket >= 0)
    s_wifi_gap_hist[bucket]++;
}

void avb_wifi_gap_histogram(uint32_t hist[7]) {
  for (int i = 0; i < 7; i++)
    hist[i] = s_wifi_gap_hist[i];
}

/* Deliver held bundles that are next in sequence; when the hold time
 * has run out, give up on the gap and move on. */
static void wifi_bundle_drain(wifi_bundle_rx_t *rx, int64_t now_us,
                              void *priv, void *info) {
  for (;;) {
    int next = -1, lowest = -1;
    for (int i = 0; i < rx->held_count; i++) {
      if (rx->held[i].seq == rx->expected)
        next = i;
      if (lowest < 0 ||
          (int16_t)(rx->held[i].seq - rx->held[lowest].seq) < 0)
        lowest = i;
    }
    if (next < 0) {
      if (rx->held_count == 0) {
        rx->deadline_us = 0;
        return;
      }
      if (now_us < rx->deadline_us)
        return;
      s_wifi_given_up++;
      wifi_gap_hist_add((uint16_t)(rx->held[lowest].seq - rx->expected));
      rx->expected = rx->held[lowest].seq;
      rx->deadline_us = wifi_bundle_hold_until(rx, now_us);
      continue;
    }
    wifi_held_bundle_t h = rx->held[next];
    rx->held[next] = rx->held[--rx->held_count];
    if (h.buf[37] & WIFI_BUNDLE_FLAG_RETX)
      s_wifi_recovered++;
    wifi_bundle_split(h.buf, h.len, priv, info);
    rx->expected++;
    rx->deadline_us = rx->held_count ? wifi_bundle_hold_until(rx, now_us) : 0;
  }
}

/* Receive a bundle: deliver in sequence, hold past a gap and NACK it. */
static void wifi_bundle_receive(uint8_t *buf, uint32_t len, void *priv,
                                void *info) {
  uint16_t seq = ((uint16_t)buf[34] << 8) | buf[35];
  int64_t now_us = esp_timer_get_time();
  wifi_bundle_rx_t *rx = NULL, *spare = NULL;
  for (int i = 0; i < WIFI_RX_STREAMS; i++) {
    if (s_bundle_rx[i].valid &&
        memcmp(s_bundle_rx[i].stream_id, buf + 22, UNIQUE_ID_LEN) == 0) {
      rx = &s_bundle_rx[i];
      break;
    }
    /* A talker moves to a new stream ID when its reservation grows, so
     * a stream gone quiet with nothing held gives its entry up. */
    if (!spare && (!s_bundle_rx[i].valid ||
                   (s_bundle_rx[i].held_count == 0 &&
                    now_us - s_bundle_rx[i].last_rx_us > 1000000)))
      spare = &s_bundle_rx[i];
  }
  if (!rx && spare) {
    rx = spare;
    memset(rx, 0, sizeof(*rx));
    rx->valid = true;
    memcpy(rx->stream_id, buf + 22, UNIQUE_ID_LEN);
  }
  if (!rx) {
    wifi_bundle_split(buf, len, priv, info);
    return;
  }
  memcpy(rx->peer, buf + ETH_ADDR_LEN, ETH_ADDR_LEN);
  rx->last_rx_us = now_us;
  /* Bundles held past a gap when the stream stopped wait for its next
   * bundle where no hold timer runs (the bridge): its sender keeps the
   * bundle sequence across runs, so they went out ahead of the next
   * run's first frame, minutes late (seen: 5 frames of the previous run
   * before seq 0 at the listener). Too late to play: drop them. */
  bool stale = rx->held_count && rx->deadline_us &&
               now_us - rx->deadline_us > WIFI_BUNDLE_STALE_US;
  int16_t ahead = rx->seeded ? (int16_t)(seq - rx->expected) : 0;
  if (!rx->seeded || stale || ahead > 64 || ahead < -64) {
    /* First bundle, or the sender restarted: start over. */
    if (rx->seeded)
      wifi_gap_hist_add(UINT16_MAX);
    for (int i = 0; i < rx->held_count; i++)
      free(rx->held[i].buf);
    rx->held_count = 0;
    rx->deadline_us = 0;
    rx->expected = seq;
    rx->nack_next = seq;
    rx->seeded = true;
    ahead = 0;
  }
  if (ahead < 0) {
    s_wifi_dup_rx++; /* a duplicate, or a retransmission after giving up */
    free(buf);
    return;
  }
  if (ahead == 0) {
    if (buf[37] & WIFI_BUNDLE_FLAG_RETX)
      s_wifi_recovered++;
    wifi_bundle_split(buf, len, priv, info);
    rx->expected++;
    if ((int16_t)(rx->expected - rx->nack_next) > 0)
      rx->nack_next = rx->expected;
    wifi_bundle_drain(rx, now_us, priv, info);
    return;
  }
  for (int i = 0; i < rx->held_count; i++) {
    if (rx->held[i].seq == seq) {
      s_wifi_dup_rx++;
      free(buf);
      wifi_bundle_drain(rx, now_us, priv, info);
      return;
    }
  }
  if (rx->held_count >= WIFI_BUNDLE_HOLD) {
    /* Hold is full: give up on the gap now, then place this bundle. */
    rx->deadline_us = now_us;
    wifi_bundle_drain(rx, now_us, priv, info);
    wifi_bundle_receive(buf, len, priv, info);
    return;
  }
  rx->held[rx->held_count].seq = seq;
  rx->held[rx->held_count].len = len;
  rx->held[rx->held_count].buf = buf;
  rx->held_count++;
  if (rx->deadline_us == 0)
    rx->deadline_us = wifi_bundle_hold_until(rx, now_us);
  /* NACK each newly missing bundle once; repeat only the oldest gap,
   * and only after a NACK or its answer could have been lost. No NACK
   * when the gap cannot wait long enough for an answer. */
  if ((int16_t)(rx->nack_next - rx->expected) < 0)
    rx->nack_next = rx->expected;
  int64_t budget_us = rx->deadline_us - now_us;
  if ((int16_t)(seq - rx->nack_next) > 0) {
    if (budget_us >= WIFI_RETX_MIN_BUDGET_US) {
      rx->last_nack_us = now_us;
      wifi_nack_send(rx, rx->nack_next, (int16_t)(seq - rx->nack_next),
                     budget_us);
    } else {
      s_wifi_nacks_skipped++;
    }
  } else if (now_us - rx->last_nack_us >= WIFI_NACK_REPEAT_US &&
             budget_us >= WIFI_RETX_MIN_BUDGET_US) {
    rx->last_nack_us = now_us;
    wifi_nack_send(rx, rx->expected, 1, budget_us);
  }
  if ((int16_t)(seq + 1 - rx->nack_next) > 0)
    rx->nack_next = (uint16_t)(seq + 1);
  wifi_bundle_drain(rx, now_us, priv, info);
}

/* Earliest hold deadline, 0 when nothing is held: the caller arms a
 * one-shot timer for it. */
static __attribute__((unused)) int64_t wifi_bundle_next_deadline(void) {
  int64_t next = 0;
  for (int i = 0; i < WIFI_RX_STREAMS; i++)
    if (s_bundle_rx[i].valid && s_bundle_rx[i].held_count &&
        (next == 0 || s_bundle_rx[i].deadline_us < next))
      next = s_bundle_rx[i].deadline_us;
  return next;
}

/* Give up on gaps whose time ran out while nothing arrived, so the held
 * audio still plays. Runs from that timer, under the same lock as
 * wifi_bundle_receive. */
static __attribute__((unused)) void wifi_bundle_poll(int64_t now_us,
                                                    void *priv, void *info) {
  for (int i = 0; i < WIFI_RX_STREAMS; i++)
    if (s_bundle_rx[i].valid && s_bundle_rx[i].held_count &&
        now_us >= s_bundle_rx[i].deadline_us)
      wifi_bundle_drain(&s_bundle_rx[i], now_us, priv, info);
}

/* NACKs left unsent because the gap could not wait for an answer. */
void avb_wifi_retx_detail(uint32_t *skipped) {
  if (skipped) *skipped = s_wifi_nacks_skipped;
}

void avb_wifi_retx_stats(uint32_t *nacks, uint32_t *recovered,
                         uint32_t *given_up, uint32_t *duplicates) {
  if (nacks) *nacks = s_wifi_nacks_tx;
  if (recovered) *recovered = s_wifi_recovered;
  if (given_up) *given_up = s_wifi_given_up;
  if (duplicates) *duplicates = s_wifi_dup_rx;
}

#if defined(CONFIG_ESP_AVB_ROLE_BRIDGE) && defined(CONFIG_ESP_AVB_WIFI_BUNDLE)
/* Downlink bundler, run only from the Ethernet receive task, so it
 * needs no lock. One slot per station and stream. */
#define WIFI_BUNDLE_SLOTS 8
#define WIFI_BUNDLE_MAX_AGE_US 2000

/* Sent bundles kept for retransmission; only those younger than the
 * age limit are resent, as older ones would miss their presentation
 * time. 48 cover 48 ms at 8 channels, longer than a station waits. */
#define WIFI_BUNDLE_HISTORY 48
/* Age limit for a NACK that carries no waiting time (older stations). */
#define WIFI_BUNDLE_RETX_MAX_AGE_US 18000

typedef struct {
  uint16_t seq;
  uint16_t len;
  int64_t sent_us;
  int64_t retx_us; /* last retransmission */
} wifi_bundle_hist_t;
#define WIFI_BUNDLE_RETX_SPACING_US 12000

typedef struct {
  bool valid;
  uint8_t dst[ETH_ADDR_LEN];
  uint8_t stream_id[UNIQUE_ID_LEN];
  uint16_t seq;
  uint8_t count;
  uint32_t len;
  int64_t first_us;
  uint8_t *buf;
  uint8_t *hist_buf;
  wifi_bundle_hist_t hist[WIFI_BUNDLE_HISTORY];
  uint8_t hist_next;
  bool avtp_seq_valid;
  uint8_t avtp_seq; /* last stream frame seq from Ethernet */
} wifi_bundle_slot_t;

static wifi_bundle_slot_t s_bundle_slots[WIFI_BUNDLE_SLOTS];
static volatile uint32_t s_wifi_bundles_tx, s_wifi_bundle_frames_tx;
static volatile uint32_t s_wifi_nacks_rx, s_wifi_retx, s_wifi_retx_stale;
/* Every requested bundle ends in exactly one of these. */
static volatile uint32_t s_retx_requested, s_retx_no_slot, s_retx_evicted,
    s_retx_late, s_retx_spaced, s_retx_no_room, s_retx_tx_fail;
static volatile uint32_t s_wifi_ingress_lost, s_wifi_ingress_late,
    s_wifi_bundle_bypass;
static portMUX_TYPE s_bundle_hist_lock = portMUX_INITIALIZER_UNLOCKED;
static uint8_t *s_retx_buf;

static void wifi_bundle_send(wifi_bundle_slot_t *slot) {
  if (slot->count == 0)
    return;
  uint8_t *out = slot->buf;
  out[WIFI_BUNDLE_AVTP + 2] = (uint8_t)slot->seq;
  out[34] = (uint8_t)(slot->seq >> 8);
  out[35] = (uint8_t)slot->seq;
  out[36] = slot->count;
  esp_err_t result = ESP_ERR_NO_MEM;
  if (!esp_hosted_data_tx_queue_space ||
      esp_hosted_data_tx_queue_space() > AVB_WIFI_CONTROL_RESERVE)
    result = wifi_tx_bundle_frame(1 /* WIFI_IF_AP */, out, slot->len);
  if (slot->hist_buf) {
    /* Kept even when the hosted queue refused it: after a burst, a NACK
     * may still bring it back in time. */
    portENTER_CRITICAL(&s_bundle_hist_lock);
    wifi_bundle_hist_t *entry = &slot->hist[slot->hist_next];
    memcpy(slot->hist_buf + (size_t)slot->hist_next * WIFI_BUNDLE_MAX_BYTES,
           out, slot->len);
    entry->seq = slot->seq;
    entry->len = (uint16_t)slot->len;
    entry->sent_us = esp_timer_get_time();
    entry->retx_us = 0;
    slot->hist_next = (uint8_t)((slot->hist_next + 1) % WIFI_BUNDLE_HISTORY);
    portEXIT_CRITICAL(&s_bundle_hist_lock);
  }
  if (result == ESP_OK) {
    s_fwd_wifi_ok += slot->count;
    s_wifi_bundles_tx++;
    s_wifi_bundle_frames_tx += slot->count;
  } else if (result == ESP_ERR_NO_MEM) {
    s_fwd_wifi_bp_drop += slot->count;
  } else {
    s_fwd_wifi_fail += slot->count;
  }
  slot->seq++;
  slot->count = 0;
  slot->len = 0;
}

/* Add a re-addressed stream frame to its station's bundle. Returns false
 * when the frame is not bundled, so the caller sends it as is. */
static bool wifi_bundle_add(const uint8_t *frame, uint32_t len) {
  if (len < WIFI_BUNDLE_AVTP + WIFI_BUNDLE_MIN_AVTP || frame[16] != 0x22 ||
      frame[17] != 0xf0 || (frame[18] & 0x80) || !(frame[19] & 0x80) ||
      WIFI_BUNDLE_FIRST_COST(len - WIFI_BUNDLE_AVTP) > WIFI_BUNDLE_FILL_BYTES)
    return false;
  int64_t now_us = esp_timer_get_time();
  wifi_bundle_slot_t *slot = NULL, *spare = NULL;
  for (int i = 0; i < WIFI_BUNDLE_SLOTS; i++) {
    wifi_bundle_slot_t *candidate = &s_bundle_slots[i];
    if (candidate->valid && candidate->count > 0 &&
        now_us - candidate->first_us > WIFI_BUNDLE_MAX_AGE_US)
      wifi_bundle_send(candidate); /* stale partial bundle */
    if (candidate->valid && memcmp(candidate->dst, frame, ETH_ADDR_LEN) == 0 &&
        memcmp(candidate->stream_id, frame + 22, UNIQUE_ID_LEN) == 0)
      slot = candidate;
    else if (!spare &&
             (!candidate->valid || (candidate->count == 0 &&
                                    now_us - candidate->first_us > 1000000)))
      spare = candidate; /* unused, or idle for a second */
  }
  if (!slot) {
    if (!spare) {
      s_wifi_bundle_bypass++;
      return false;
    }
    if (!spare->buf)
      spare->buf = heap_caps_malloc(WIFI_BUNDLE_MAX_BYTES,
                                    MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT);
    if (!spare->buf)
      spare->buf = heap_caps_malloc(WIFI_BUNDLE_MAX_BYTES, MALLOC_CAP_SPIRAM);
    if (!spare->buf) {
      s_wifi_bundle_bypass++;
      return false;
    }
    /* PSRAM first: 48 bundles per slot are too many for internal RAM,
     * and in loaded tests the copy into PSRAM lost no received frames. */
    if (!spare->hist_buf)
      spare->hist_buf =
          heap_caps_calloc(WIFI_BUNDLE_HISTORY, WIFI_BUNDLE_MAX_BYTES,
                           MALLOC_CAP_SPIRAM);
    if (!spare->hist_buf)
      spare->hist_buf =
          heap_caps_calloc(WIFI_BUNDLE_HISTORY, WIFI_BUNDLE_MAX_BYTES,
                           MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT);
    memset(spare->hist, 0, sizeof(spare->hist));
    spare->hist_next = 0;
    spare->avtp_seq_valid = false;
    slot = spare;
    slot->valid = true;
    memcpy(slot->dst, frame, ETH_ADDR_LEN);
    memcpy(slot->stream_id, frame + 22, UNIQUE_ID_LEN);
    slot->count = 0;
    slot->len = 0;
  }
  /* Stream frames already missing on arrival from Ethernet */
  uint8_t avtp_seq = frame[WIFI_BUNDLE_AVTP + 2];
  if (slot->avtp_seq_valid) {
    uint8_t skipped = (uint8_t)(avtp_seq - slot->avtp_seq - 1);
    if (skipped >= 200)
      s_wifi_ingress_late++;
    else
      s_wifi_ingress_lost += skipped;
  }
  slot->avtp_seq = avtp_seq;
  slot->avtp_seq_valid = true;
  uint32_t avtp_len = len - WIFI_BUNDLE_AVTP;
  const uint8_t *avtp = frame + WIFI_BUNDLE_AVTP;
  uint32_t entry_len = WIFI_BUNDLE_ENTRY_COST(slot->buf, slot->count, avtp,
                                              avtp_len);
  if (slot->count > 0 && slot->len + entry_len > WIFI_BUNDLE_FILL_BYTES)
    wifi_bundle_send(slot);
  if (slot->count == 0) {
    slot->len = wifi_bundle_begin(slot->buf, frame);
    slot->first_us = now_us;
  }
  slot->len += wifi_bundle_put(slot->buf, slot->len, avtp, avtp_len);
  slot->count++;
  if (slot->count >= CONFIG_ESP_AVB_WIFI_BUNDLE_FRAMES)
    wifi_bundle_send(slot);
  return true;
}

/* A station asks for bundles it missed: resend those still in the
 * history that can arrive while the station waits for them. Runs on the
 * Wi-Fi receive task, concurrently with the sender. */
static void wifi_nack_handle(const uint8_t *nack) {
  s_wifi_nacks_rx++;
  const uint8_t *station = nack + ETH_ADDR_LEN;
  uint16_t first = ((uint16_t)nack[30] << 8) | nack[31];
  int count = nack[32];
  int64_t budget_us =
      (int64_t)(((uint32_t)nack[WIFI_NACK_BUDGET] << 8) |
                nack[WIFI_NACK_BUDGET + 1]) * 100;
  s_retx_requested += count;
  if (!s_retx_buf)
    s_retx_buf = heap_caps_malloc(WIFI_BUNDLE_MAX_BYTES,
                                  MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT);
  if (!s_retx_buf) {
    s_retx_no_room += count;
    return;
  }
  for (int i = 0; i < WIFI_BUNDLE_SLOTS; i++) {
    wifi_bundle_slot_t *slot = &s_bundle_slots[i];
    if (!slot->valid || !slot->hist_buf ||
        memcmp(slot->dst, station, ETH_ADDR_LEN) != 0 ||
        memcmp(slot->stream_id, nack + 18, UNIQUE_ID_LEN) != 0)
      continue;
    for (int n = 0; n < count; n++) {
      uint16_t want = (uint16_t)(first + n);
      uint16_t len = 0;
      int64_t now_us = esp_timer_get_time();
      bool found = false, late = false, spaced = false;
      portENTER_CRITICAL(&s_bundle_hist_lock);
      for (int k = 0; k < WIFI_BUNDLE_HISTORY; k++) {
        wifi_bundle_hist_t *entry = &slot->hist[k];
        if (entry->len == 0 || entry->seq != want)
          continue;
        found = true;
        int64_t age_us = now_us - entry->sent_us;
        if (budget_us == 0 ? age_us > WIFI_BUNDLE_RETX_MAX_AGE_US
                           : budget_us - WIFI_NACK_TRIP_US <
                                 WIFI_RETX_MIN_BUDGET_US) {
          late = true;
          break;
        }
        if (entry->retx_us != 0 &&
            now_us - entry->retx_us < WIFI_BUNDLE_RETX_SPACING_US) {
          spaced = true; /* just resent: let that copy arrive */
          break;
        }
        entry->retx_us = now_us;
        len = entry->len;
        memcpy(s_retx_buf, slot->hist_buf + (size_t)k * WIFI_BUNDLE_MAX_BYTES,
               len);
        break;
      }
      portEXIT_CRITICAL(&s_bundle_hist_lock);
      if (!found) {
        s_retx_evicted++;
        continue;
      }
      if (late || spaced) {
        if (late)
          s_retx_late++;
        else
          s_retx_spaced++;
        s_wifi_retx_stale++;
        continue;
      }
      if (esp_hosted_data_tx_queue_space &&
          esp_hosted_data_tx_queue_space() <= AVB_WIFI_CONTROL_RESERVE) {
        s_retx_no_room += count - n; /* the reserve is for control frames */
        return;
      }
      s_retx_buf[37] |= WIFI_BUNDLE_FLAG_RETX;
      if (wifi_tx_bundle_frame(1 /* WIFI_IF_AP */, s_retx_buf, len) == ESP_OK)
        s_wifi_retx++;
      else
        s_retx_tx_fail++;
    }
    return;
  }
  s_retx_no_slot += count;
}

/* Outcome of every bundle a NACK asked for: requested = sent (the retx
 * counter) + late + spaced + evicted + no_slot + no_room + tx_fail. */
void avb_bridge_retx_detail(uint32_t *requested, uint32_t *late,
                            uint32_t *spaced, uint32_t *evicted,
                            uint32_t *no_slot, uint32_t *no_room,
                            uint32_t *tx_fail) {
  if (requested) *requested = s_retx_requested;
  if (late) *late = s_retx_late;
  if (spaced) *spaced = s_retx_spaced;
  if (evicted) *evicted = s_retx_evicted;
  if (no_slot) *no_slot = s_retx_no_slot;
  if (no_room) *no_room = s_retx_no_room;
  if (tx_fail) *tx_fail = s_retx_tx_fail;
}

void avb_bridge_retx_stats(uint32_t *nacks, uint32_t *retx, uint32_t *stale) {
  if (nacks) *nacks = s_wifi_nacks_rx;
  if (retx) *retx = s_wifi_retx;
  if (stale) *stale = s_wifi_retx_stale;
}

void avb_bridge_forward_stats_bundles(uint32_t *bundles, uint32_t *frames) {
  if (bundles) *bundles = s_wifi_bundles_tx;
  if (frames) *frames = s_wifi_bundle_frames_tx;
}

void avb_bridge_bundle_ingress_stats(uint32_t *lost, uint32_t *late,
                                     uint32_t *bypass) {
  if (lost) *lost = s_wifi_ingress_lost;
  if (late) *late = s_wifi_ingress_late;
  if (bypass) *bypass = s_wifi_bundle_bypass;
}
#endif

/* esp_timer time (low 32 bits, made odd so it is never 0) at which the
 * EMAC frame now in the callback entered it, 0 while none is inside. */
static volatile uint32_t s_emac_rx_cb_entered_us = 0;

uint32_t avb_emac_rx_cb_busy_ms(void) {
  uint32_t entered_us = s_emac_rx_cb_entered_us;
  if (entered_us == 0)
    return 0;
  return ((uint32_t)esp_timer_get_time() - entered_us) / 1000;
}

static esp_err_t avb_unified_rx_cb(esp_eth_handle_t eth_handle, uint8_t *buf,
                                   uint32_t len, void *priv, void *info) {
  uint32_t c0 = esp_cpu_get_cycle_count();
  if (eth_handle)
    s_emac_rx_cb_entered_us = (uint32_t)esp_timer_get_time() | 1;
  esp_err_t r = avb_unified_rx_cb_inner(eth_handle, buf, len, priv, info);
  if (eth_handle)
    s_emac_rx_cb_entered_us = 0;
  s_prof_cb_cycles += esp_cpu_get_cycle_count() - c0;
  s_prof_cb_frames++;
  return r;
}

static esp_err_t avb_unified_rx_cb_inner(esp_eth_handle_t eth_handle,
                                         uint8_t *buf, uint32_t len,
                                         void *priv, void *info) {
  if (len < ETH_HEADER_LEN) {
    free(buf);
    return ESP_OK;
  }

  /* Read ethertype at offset 12-13 (big-endian) */
  uint16_t ethertype = (buf[12] << 8) | buf[13];

  /* AVB Lite PTP may arrive priority-tagged, VLAN 0 at PCP 7, and must
   * be accepted tagged or untagged (avb_lite.md §5). Drop the tag so
   * the frame takes the PTP path below with its hardware timestamp,
   * which the info argument carries separately. */
  if (ethertype == 0x8100 && len >= ETH_HEADER_LEN + 4 && buf[16] == 0x88 &&
      buf[17] == 0xf7) {
    memmove(buf + 12, buf + 16, len - 16);
    len -= 4;
    ethertype = 0x88f7;
  }

  /* A bundle from the Wi-Fi hop: handle each AVTPDU in it on its own. */
  if (eth_handle == NULL && ethertype == 0x8100 && wifi_bundle_is(buf, len)) {
    wifi_bundle_receive(buf, len, priv, info);
    return ESP_OK;
  }
  if (eth_handle == NULL && ethertype == 0x22f0 && wifi_nack_is(buf, len)) {
#if defined(CONFIG_ESP_AVB_ROLE_BRIDGE) && defined(CONFIG_ESP_AVB_WIFI_BUNDLE)
    wifi_nack_handle(buf);
#elif defined(CONFIG_ESP_AVB_WIFI_BUNDLE)
    wifi_tx_nack_handle(buf);
#endif
    free(buf);
    return ESP_OK;
  }

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
#ifdef CONFIG_ESP_AVB_WIFI_UNICAST_STREAMS
    /* Re-address the stream for the Wi-Fi hop. 802.11 buffers group
     * addressed frames to the DTIM cycle and neither acknowledges nor
     * retries them (IEEE 802.11 10.3.6), which measures here as ~397
     * pps against the ~8342 pps the same frames reach when individually
     * addressed, far below the 4000 pps a Class B stream needs.
     *
     * This is DMS-equivalent behaviour, NOT IEEE 802.11 DMS: real DMS
     * carries the group addressed MSDU inside an individually addressed
     * A-MSDU and is negotiated with Action frames, neither of which
     * ESP-IDF exposes. The observable difference is that the Listener
     * sees an individual destination address; that is harmless because
     * IEEE 1722 4.4.4.8 makes stream_id, carried in every AVTPDU,
     * the stream's identity, and SRP/Milan keep seeing the MAAP address
     * because nothing upstream of this point is modified.
     *
     * Cost is one indexed load plus a 6-octet compare, and on a hit a
     * 6-octet store into a cache line this path has already touched.
     * buf is ours until the Wi-Fi free callback runs, so the write is
     * in place, no copy, matching the zero-copy intent below. */
    const uint8_t *ucast_da = msrp_wifi_ucast_lookup_da(buf);
    if (ucast_da != NULL) {
      memcpy(buf, ucast_da, ETH_ADDR_LEN);
      s_fwd_wifi_readdressed++;
    } else if ((buf[0] & 0x01) != 0) {
      /* Group addressed with no Listener declaration to resolve against:
       * forward as-is and let the counter show it. */
      s_fwd_wifi_no_mapping++;
    }
#endif
#ifdef CONFIG_ESP_AVB_WIFI_BUNDLE
    if (ucast_da != NULL && wifi_bundle_add(buf, len)) {
      free(buf);
      return ESP_OK;
    }
#endif
    /* Stream frames leave the last few hosted queue entries to control
     * frames (PTP, MSRP, ATDECC), which would otherwise be dropped behind
     * an over-admitted stream. */
    if (esp_hosted_data_tx_queue_space &&
        esp_hosted_data_tx_queue_space() <= AVB_WIFI_CONTROL_RESERVE) {
      s_fwd_wifi_bp_drop++;
      free(buf);
      return ESP_OK;
    }
    /* NOT tx_by_ref: on esp_wifi_remote that entry point is a WEAK stub
     * that forwards to the copying esp_wifi_internal_tx and silently
     * discards the netstack_buf, the ref/free callbacks registered by
     * avb_bridge_install_zero_copy_tx are likewise dropped by a stub
     * (esp_wifi_remote_net.c). By-ref hand-off therefore leaked every
     * forwarded frame (~1.6 MB/s at stream rate) until the internal/DMA
     * pool starved. The hosted transport copies into its own SDIO
     * buffers no matter what, so zero-copy is not achievable on this
     * medium; copy-and-free is both correct and no slower.
     *
     * ESP_ERR_NO_MEM is the driver's own backpressure (hosted TX queue
     * full): the frame is dropped and counted, which is the right
     * behaviour for an over-admitted stream: bounded queue, bounded
     * latency, loss visible in the bpdrop counter. */
    esp_err_t r = esp_wifi_internal_tx(1 /* WIFI_IF_AP */, (void *)buf, len);
    if (r == ESP_OK) {
      s_fwd_wifi_ok++;
    } else if (r == ESP_ERR_NO_MEM) {
      s_fwd_wifi_bp_drop++;
    } else {
      s_fwd_wifi_fail++;
    }
    free(buf);
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
#ifdef CONFIG_ESP_AVB_WIFI_UNICAST_STREAMS
    /* AVB Wireless profile 3.3 restore: a wireless talker transmits
     * stream frames individually addressed to the BSSID, restore the
     * destination it advertised in its Talker Advertise before wired
     * egress so wired listeners and SRP filtering see the MAAP
     * address. Applies only to Wi-Fi-ingress VLAN-tagged AVTP frames
     * addressed to this bridge's own wireless MAC, anything else
     * (already multicast, or unicast straight to a wired listener per
     * AVB Lite) passes untouched. */
    if (d.verdict == AVB_BRIDGE_BRIDGE && ingress_port == 1 &&
        ethertype == 0x8100 && len >= 30 && buf[16] == 0x22 &&
        buf[17] == 0xf0 && (buf[0] & 0x01) == 0) {
      /* Any individually addressed carrier DA qualifies, the talker
       * uses the BSSID with the U/L bit set (frames addressed to the
       * AP's own MAC never cross the esp_hosted transport), and the
       * stream_id lookup is what authorizes the rewrite. A frame whose
       * stream_id is not a wireless talker's advertised stream passes
       * untouched, which also keeps AVB Lite unicast-to-listener
       * frames intact. */
      const uint8_t *adv_da = msrp_wifi_ucast_lookup_stream(buf + 22);
      if (adv_da != NULL) {
        memcpy(buf, adv_da, ETH_ADDR_LEN);
        s_fwd_eth_restored++;
      }
    }
#endif
    switch (d.verdict) {
    case AVB_BRIDGE_DROP:
      free(buf);
      return ESP_OK;
    case AVB_BRIDGE_BRIDGE:
      if (ingress_port == 1 && ethertype == 0x8100 &&
          d.egress_port < CONFIG_ESP_AVB_NUM_PORTS &&
          s_bridge_state->port[d.egress_port].medium ==
              avb_port_medium_eth_hwts &&
          station_pacer_forward(d.egress_port, buf, len)) {
        free(buf);
        return ESP_OK;
      }
      avb_bridge_forward(d.egress_port, ethertype, buf, len);
      if (ethertype == 0x22f0)
        avb_bridge_wifi_control(ingress_port, d.egress_port, buf, len);
      free(buf);
      return ESP_OK;
    case AVB_BRIDGE_TERMINATE:
    default:
      break; /* fall through to existing per-protocol dispatch */
    }
  }
#endif /* CONFIG_ESP_AVB_ROLE_BRIDGE */

  switch (ethertype) {
  case 0x8100: { /* VLAN — stream data, or VLAN-tagged AVTP control */
    /* Destination-address guard (avb_lite.md §6 item 4: accept frames
     * addressed to our MAC or to a multicast group). The EMAC runs
     * promiscuous, and unicast transport makes this load-bearing:
     * when a peer listener's link drops, the switch floods that
     * peer's unicast stream copy as unknown-unicast, and matching by
     * stream_id alone would ingest the same stream twice — duplicate
     * sequence numbers and 2x ring input for the whole outage
     * (observed: ~55k inflated seq_gap + pegged jitter ring per
     * 6-second peer reboot). Multicast bit passes so MAAP-escalated
     * streams and tagged control multicasts are unaffected. */
    static const uint8_t zero_mac[ETH_ADDR_LEN] = {0};
    if (!(buf[0] & 1) && memcmp(buf, s_own_mac, ETH_ADDR_LEN) != 0 &&
        memcmp(s_own_mac, zero_mac, ETH_ADDR_LEN) != 0) {
      free(buf);
      return ESP_OK;
    }
    /* AVB Lite CVU SRP frames are VLAN-tagged in the media VLAN so
     * VLAN-aware switches learn the sender's MAC in that VLAN's
     * filtering database — otherwise unicast stream frames are
     * flooded as unknown-unicast, defeating unicast transport
     * (avb_lite.md §6). Peek the AVTP subtype behind the tag: AECP
     * goes to the control queue; everything else stays on the
     * stream hot path (one byte-compare per frame). */
    if (len > 19 && buf[16] == 0x22 && buf[17] == 0xf0 &&
        buf[18] == avtp_subtype_aecp) {
      if (s_ctrl_rx_queue) {
        ctrl_rx_pkt_t pkt;
        int ingress = (eth_handle == NULL) ? s_wifi_port_idx : s_eth_port_idx;
        if (ingress < 0) {
          free(buf);
          return ESP_OK;
        }
        pkt.protocol_idx = AVTP;
        pkt.ingress_port = (uint8_t)ingress;
        memcpy(pkt.src_addr, buf + ETH_ADDR_LEN, ETH_ADDR_LEN);
        /* Strip ETH header (14) + VLAN tag (4) = 18 bytes */
        uint32_t payload_len = len - 18;
        if (payload_len > AVB_MAX_MSG_LEN)
          payload_len = AVB_MAX_MSG_LEN;
        pkt.length = payload_len;
        memcpy(pkt.data, buf + 18, payload_len);
        xQueueSend(s_ctrl_rx_queue, &pkt, 0);
      }
      free(buf);
      return ESP_OK;
    }
    /* Inline handler call — matches the original stable architecture.
     * The queue-based split we tried (emac_rx → queue → AVB-IN task)
     * added ~10 µs per frame of task-wake + context-switch overhead
     * that at 8000 pps cost ~80 ms/sec of extra CPU and drove the NIC
     * DMA ring into overflow. Keeping the handler here means emac_rx
     * does one self-contained pass per frame: alloc → memcpy →
     * handler → free. */
    if (s_stream_handler && len > 18) {
      /* Strip ETH header (14) + VLAN tag (4) = 18 bytes → raw AVTP */
      uint32_t c0 = esp_cpu_get_cycle_count();
      s_stream_handler(buf + 18, len - 18, s_stream_ctx);
      s_prof_stream_cycles += esp_cpu_get_cycle_count() - c0;
      s_prof_stream_frames++;
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
        (void)ptp_inject_received_frame_from(s_wifi_port_idx,
                                        buf + ETH_HEADER_LEN,
                                        (uint16_t)(len - ETH_HEADER_LEN),
                                        buf + ETH_ADDR_LEN);
      }
      free(buf);
      return ESP_OK;
    }
    if (len >= ETH_HEADER_LEN + 54 &&
        (buf[ETH_HEADER_LEN] & 0x0f) == 0x0a && ptpd_note_wired_peer_follow_up)
      ptpd_note_wired_peer_follow_up(buf + ETH_HEADER_LEN, len - ETH_HEADER_LEN);
    size_t frame_len = len;
#if ESP_IDF_VERSION >= ESP_IDF_VERSION_VAL(6, 2, 0)
    /* v6.2 changed the filter contract: info is now a typed struct.
     * The rx-cb info memory layout still matches eth_mac_time_t, so
     * wrap it the same way the upstream netif glue does. */
    l2tap_eth_filter_info_t l2tap_info = {
        .l2_buffer = NULL,
        .hw_ts = (l2tap_timestamp_t *)info,
    };
    esp_vfs_l2tap_eth_filter_frame(eth_handle, buf, &frame_len, &l2tap_info);
#else
    esp_vfs_l2tap_eth_filter_frame(eth_handle, buf, &frame_len, info);
#endif
    if (frame_len > 0) {
      /* No L2TAP fd matched (gPTP daemon not yet up, or socket closed) —
       * free the unconsumed buffer ourselves. */
      s_ptp_l2tap_unmatched++;
      free(buf);
    } else {
      s_ptp_l2tap_consumed++;
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
    /* The IP stack takes only IPv4, ARP and IPv6, and no L2TAP socket
     * listens for anything else, so drop other ethertypes here instead
     * of paying an L2TAP walk, an lwIP buffer and a queue hop for each.
     * This task runs above the AVB task: a flood of 30,000 unrelated
     * frames/s kept it busy enough that the AVB task stopped
     * re-declaring MSRP and answering ACMP, and the switch dropped the
     * talker's reservation. */
    if (ethertype != 0x0800 && ethertype != 0x0806 && ethertype != 0x86dd) {
      s_rx_dropped_other++;
      free(buf);
      return ESP_OK;
    }
    size_t frame_len = len;
#if ESP_IDF_VERSION >= ESP_IDF_VERSION_VAL(6, 2, 0)
    l2tap_eth_filter_info_t l2tap_info = {
        .l2_buffer = NULL,
        .hw_ts = (l2tap_timestamp_t *)info,
    };
    esp_vfs_l2tap_eth_filter_frame(eth_handle, buf, &frame_len, &l2tap_info);
#else
    esp_vfs_l2tap_eth_filter_frame(eth_handle, buf, &frame_len, info);
#endif
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

#if !defined(CONFIG_ESP_AVB_ROLE_BRIDGE)
/* A held gap is given up when its time runs out even if nothing else
 * arrives, so the audio held behind it still plays. Wi-Fi receive and
 * this timer both feed the listener; the lock keeps them one at a time. */
static SemaphoreHandle_t s_wifi_rx_lock;
static esp_timer_handle_t s_hold_timer;
static int64_t s_hold_timer_due;

/* Called with s_wifi_rx_lock held. */
static void wifi_hold_timer_arm(void) {
  int64_t due = wifi_bundle_next_deadline();
  if (!s_hold_timer || due == s_hold_timer_due)
    return;
  esp_timer_stop(s_hold_timer);
  s_hold_timer_due = due;
  if (due == 0)
    return;
  int64_t wait_us = due - esp_timer_get_time();
  esp_timer_start_once(s_hold_timer, wait_us > 100 ? (uint64_t)wait_us : 100);
}

static void wifi_hold_timer_cb(void *arg) {
  (void)arg;
  xSemaphoreTake(s_wifi_rx_lock, portMAX_DELAY);
  s_hold_timer_due = 0;
  wifi_bundle_poll(esp_timer_get_time(), NULL, NULL);
  wifi_hold_timer_arm();
  xSemaphoreGive(s_wifi_rx_lock);
}

static void wifi_rx_lock_init(void) {
  if (s_wifi_rx_lock)
    return;
  s_wifi_rx_lock = xSemaphoreCreateMutex();
  const esp_timer_create_args_t args = {.callback = wifi_hold_timer_cb,
                                        .name = "wifi_hold"};
  if (esp_timer_create(&args, &s_hold_timer) != ESP_OK)
    s_hold_timer = NULL;
}

#if CONFIG_ESP_AVB_STATION_RX_WORKER
/* Received frames leave the Wi-Fi task for a worker just below the
 * station's talker: unpacking a downlink bundle costs a few hundred
 * microseconds, and a burst of them in the Wi-Fi task held the talker
 * off for milliseconds. The listener's buffer absorbs the wait. Every
 * frame goes, so the Wi-Fi task never waits on s_wifi_rx_lock: the
 * worker holds it while it may transmit (a NACK), and a transmit can
 * wait on the Wi-Fi task (that deadlocked receive and transmit when
 * control frames stayed in the Wi-Fi task). */
typedef struct {
  uint8_t *buf;
  uint16_t len;
} wifi_rx_item_t;
#define WIFI_RX_WORKER_DEPTH 64
/* Each queued frame is a heap copy. Below a talker that takes a third
 * of the CPU the queue reached 34 frames and left the heap 1 KB free, so
 * the uplink failed sends; above it always, the talker ran up to 23 ms
 * late. The worker rises above the talker only while it is behind. */
#define WIFI_RX_WORKER_BOOST_DEPTH 8
#define WIFI_RX_WORKER_CALM_DEPTH 2
#ifdef CONFIG_ESP_AVB_STATION_TALKER_PRIORITY
#define WIFI_RX_WORKER_BOOST_PRIORITY (CONFIG_ESP_AVB_STATION_TALKER_PRIORITY + 1)
#else
#define WIFI_RX_WORKER_BOOST_PRIORITY (CONFIG_ESP_AVB_STATION_RX_WORKER_PRIORITY + 2)
#endif
static QueueHandle_t s_wifi_rx_queue;

static void wifi_rx_worker(void *arg) {
  (void)arg;
  wifi_rx_item_t item;
  bool boosted = false;
  for (;;) {
    if (xQueueReceive(s_wifi_rx_queue, &item, portMAX_DELAY) != pdTRUE)
      continue;
    UBaseType_t waiting = uxQueueMessagesWaiting(s_wifi_rx_queue);
    if (!boosted && waiting >= WIFI_RX_WORKER_BOOST_DEPTH) {
      boosted = true;
      vTaskPrioritySet(NULL, WIFI_RX_WORKER_BOOST_PRIORITY);
    } else if (boosted && waiting <= WIFI_RX_WORKER_CALM_DEPTH) {
      boosted = false;
      vTaskPrioritySet(NULL, CONFIG_ESP_AVB_STATION_RX_WORKER_PRIORITY);
    }
    xSemaphoreTake(s_wifi_rx_lock, portMAX_DELAY);
    avb_unified_rx_cb(NULL /* no eth_handle */, item.buf, (uint32_t)item.len,
                      NULL, NULL);
    wifi_hold_timer_arm();
    xSemaphoreGive(s_wifi_rx_lock);
  }
}

/* Runs in the Wi-Fi task, its only caller. */
static bool wifi_rx_worker_ready(void) {
  static bool s_failed;
  if (s_wifi_rx_queue || s_failed)
    return s_wifi_rx_queue != NULL;
  QueueHandle_t queue =
      xQueueCreate(WIFI_RX_WORKER_DEPTH, sizeof(wifi_rx_item_t));
  if (!queue ||
      xTaskCreate(wifi_rx_worker, "avb_wifi_rx", 4096, NULL,
                  CONFIG_ESP_AVB_STATION_RX_WORKER_PRIORITY, NULL) != pdPASS) {
    avberr("Wi-Fi RX worker not started, frames stay in the Wi-Fi task");
    if (queue)
      vQueueDelete(queue);
    s_failed = true;
    return false;
  }
  s_wifi_rx_queue = queue;
  return true;
}
#endif
#endif

static esp_err_t avb_wifi_rx_cb(void *buffer, uint16_t len, void *eb) {
  s_rx_wifi_cb_entry++; /* L2 counter — bare increment, no logging in hot path */
  uint8_t *buf = malloc(len);
  if (!buf) {
    esp_wifi_internal_free_rx_buffer(eb);
    return ESP_FAIL;
  }
  memcpy(buf, buffer, len);
  esp_wifi_internal_free_rx_buffer(eb);
  len = (uint16_t)wifi_vi_unwrap(buf, len);
#if !defined(CONFIG_ESP_AVB_ROLE_BRIDGE)
  wifi_rx_lock_init();
#if CONFIG_ESP_AVB_STATION_RX_WORKER
#if defined(CONFIG_ESP_AVB_WIFI_BUNDLE)
  /* A NACK for the uplink is answered here: behind the worker's queue of
   * downlink bundles it waited 20 to 60 ms, and the bundles it asked
   * for had left the history. Not when bundles are wrapped: the wrap
   * lock's holder can be waiting on this task. */
  if (!s_wifi_vi_wrap && wifi_nack_is(buf, len)) {
    wifi_tx_nack_handle(buf);
    free(buf);
    return ESP_OK;
  }
#endif
  /* To the worker; a full queue drops the frame rather than make the
   * Wi-Fi task wait. */
  if (s_wifi_rx_lock && wifi_rx_worker_ready()) {
    wifi_rx_item_t item = {.buf = buf, .len = len};
    if (xQueueSend(s_wifi_rx_queue, &item, 0) == pdTRUE)
      return ESP_OK;
    free(buf);
    s_stream_rx_drops++;
    return ESP_OK;
  }
#endif
  if (s_wifi_rx_lock) {
    xSemaphoreTake(s_wifi_rx_lock, portMAX_DELAY);
    esp_err_t result = avb_unified_rx_cb(NULL /* no eth_handle */, buf,
                                         (uint32_t)len, NULL, NULL);
    wifi_hold_timer_arm();
    xSemaphoreGive(s_wifi_rx_lock);
    return result;
  }
#endif
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
/* Clause 22 access to a Clause 45 MMD register (IEEE 802.3 22.2.4.3.11,
 * registers 13 and 14). */
static esp_err_t avb_phy_mmd_access(esp_eth_handle_t eth_handle, uint8_t devad,
                                    uint16_t mmd_reg, uint32_t *value,
                                    bool write) {
  uint32_t control = devad;
  uint32_t address = mmd_reg;
  esp_eth_phy_reg_rw_data_t control_rw = {.reg_addr = 13,
                                          .reg_value_p = &control};
  esp_eth_phy_reg_rw_data_t data_rw = {.reg_addr = 14, .reg_value_p = &address};
  esp_err_t err = esp_eth_ioctl(eth_handle, ETH_CMD_WRITE_PHY_REG, &control_rw);
  if (err == ESP_OK)
    err = esp_eth_ioctl(eth_handle, ETH_CMD_WRITE_PHY_REG, &data_rw);
  control = 0x4000u | devad; /* data, no post increment */
  if (err == ESP_OK)
    err = esp_eth_ioctl(eth_handle, ETH_CMD_WRITE_PHY_REG, &control_rw);
  data_rw.reg_value_p = value;
  if (err == ESP_OK)
    err = esp_eth_ioctl(eth_handle,
                        write ? ETH_CMD_WRITE_PHY_REG : ETH_CMD_READ_PHY_REG,
                        &data_rw);
  return err;
}

/* Endpoint link policy (avb_lite.md §6 item 9): advertise neither
 * Energy-Efficient Ethernet nor PAUSE, and do not act on PAUSE. Called
 * between esp_eth_driver_install and esp_eth_start the first
 * negotiation already uses it. With restart set, a changed
 * advertisement is renegotiated at once. */
bool avb_eth_apply_link_policy(esp_eth_handle_t eth_handle, bool restart) {
  if (eth_handle == NULL)
    return false;
  bool changed = false;
  uint32_t anar = 0;
  esp_eth_phy_reg_rw_data_t anar_rw = {.reg_addr = 4, .reg_value_p = &anar};
  bool pause_advertised =
      esp_eth_ioctl(eth_handle, ETH_CMD_READ_PHY_REG, &anar_rw) == ESP_OK &&
      (anar & 0x0C00u) != 0;
  /* Turns MAC flow control off and clears the PAUSE bits in ANAR. */
  uint32_t flow_ctrl = 0;
  esp_eth_ioctl(eth_handle, ETH_CMD_S_FLOW_CTRL, &flow_ctrl);
  changed |= pause_advertised;
  /* EEE advertisement, MMD 7 register 60: bit 1 100BASE-TX, bit 2
   * 1000BASE-T. All ones means the PHY has no such register. */
  uint32_t eee_adv = 0;
  bool eee_read = avb_phy_mmd_access(eth_handle, 7, 60, &eee_adv, false) ==
                  ESP_OK;
  bool eee_advertised =
      eee_read && eee_adv != 0xFFFFu && (eee_adv & 0x0006u) != 0;
  if (eee_advertised) {
    uint32_t cleared = eee_adv & ~0x0006u;
    avb_phy_mmd_access(eth_handle, 7, 60, &cleared, true);
    changed = true;
  }
  ESP_LOGI(TAG, "Link policy: PAUSE %s, EEE advertisement %s0x%04x%s",
           pause_advertised ? "was advertised, cleared" : "not advertised",
           eee_read ? "" : "unreadable ", (unsigned)eee_adv,
           eee_advertised ? ", cleared" : "");
  if (changed && restart) {
    esp_eth_phy_t *phy = NULL;
    if (esp_eth_get_phy_instance(eth_handle, &phy) == ESP_OK && phy) {
      bool autonego_enabled = true;
      phy->autonego_ctrl(phy, ESP_ETH_PHY_AUTONEGO_RESTART, &autonego_enabled);
      ESP_LOGW(TAG, "Link policy applied after start, renegotiating");
    }
  }
  return changed;
}

int avb_net_init(avb_state_s *state) {
#if CONFIG_ESP_AVB_WIFI_VIDEO_AC_AB
  static bool vi_ab_started;
  if (!vi_ab_started) {
    vi_ab_started = true;
    /* Above the streaming tasks: on a single-core station a low priority
     * task does not run while audio flows. */
    xTaskCreate(wifi_vi_ab_task, "vi_ab", 3072, NULL, 20, NULL);
  }
#endif
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
#if !defined(CONFIG_ESP_AVB_ROLE_BRIDGE)
    /* Applications should apply the endpoint link policy before
     * esp_eth_start; this catches the ones that do not. */
    avb_eth_apply_link_policy(eth_handle, true);
#endif
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

  /* Cache port 0's MAC for the dispatcher's destination-address
   * guard (set before the RX callback below can fire). */
  memcpy(s_own_mac, state->port[0].internal_mac_addr, ETH_ADDR_LEN);

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
  if (state->avb_lite)
    prio = AVB_LITE_PCP;
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

/* Forward decl repeated here so endpoint builds (which don't define
 * CONFIG_ESP_AVB_ROLE_BRIDGE) can still see the symbol. esp_avb avoids
 * a hard esp_wifi PRIV_REQUIRES; native Wi-Fi or esp_wifi_remote
 * provides the link symbol when it's actually called. */
extern esp_err_t esp_wifi_internal_tx(int wifi_if, void *buffer, size_t len);

/* Control frames from a station wait out a full TX queue. A station's
 * stream sends more frames than the air drains, so its queue is full
 * most of the time and a control frame sent once would mostly fail.
 * The stream task runs below every control sender on the single core,
 * so while a sender polls here the next buffer the Wi-Fi task frees
 * goes to the control frame, not the stream. At the stream's air rate
 * a buffer frees about every millisecond. */
#define WIFI_CONTROL_TX_WAIT_US 10000
#define WIFI_CONTROL_TX_POLL_US 100

static esp_err_t avb_net_wifi_tx_control(int wifi_if, void *frame,
                                         size_t frame_len) {
  esp_err_t result = esp_wifi_internal_tx(wifi_if, frame, frame_len);
  if (result != ESP_ERR_NO_MEM || wifi_if != 0 /* WIFI_IF_STA */) {
    return result;
  }
  int64_t deadline_us = esp_timer_get_time() + WIFI_CONTROL_TX_WAIT_US;
  while (result == ESP_ERR_NO_MEM && esp_timer_get_time() < deadline_us) {
    esp_rom_delay_us(WIFI_CONTROL_TX_POLL_US);
    result = esp_wifi_internal_tx(wifi_if, frame, frame_len);
  }
  return result;
}

/* Frames sent from core 0 share the EMAC transmit mutex with the
 * talker on core 1. Two things kept the talker waiting on it:
 * - A sender the EMAC RX task preempted while holding it. Priority
 *   inheritance raises a holder that is not running but does not
 *   switch it in, so the talker waited until the RX task idled, 70-120
 *   us. A sender now runs above the RX task for the send.
 * - A frame sent just before a talker slot. A timed PTP frame holds the
 *   mutex until its timestamp is back, and talker frames behind one
 *   went out up to 100 us late, 4 a second, which a bridge's shaper
 *   then turned into a backlog. A sender now waits for the talker's
 *   next frame (unless one just went), so its frame queues right
 *   behind it and the mutex is free long before the next slot. Bounded
 *   for a stalled or stopping talker. */
#define AVB_NET_TX_GATE_RECENT_US 20
#define AVB_NET_TX_GATE_MAX_US 300
static _Atomic uint32_t s_talker_tx_done_us;
static _Atomic int s_talkers_wired;

void avb_net_talker_tx_wired(bool wired) {
  atomic_fetch_add(&s_talkers_wired, wired ? 1 : -1);
}

void avb_net_talker_tx_done(uint32_t done_us) {
  atomic_store_explicit(&s_talker_tx_done_us, done_us, memory_order_release);
}

#if portNUM_PROCESSORS > 1 && !defined(CONFIG_ESP_AVB_ROLE_BRIDGE)
#define AVB_NET_TX_PRIORITY (configMAX_PRIORITIES - 2)
UBaseType_t avb_net_tx_begin(void) {
  UBaseType_t prio = uxTaskPriorityGet(NULL);
  if (prio < AVB_NET_TX_PRIORITY)
    vTaskPrioritySet(NULL, AVB_NET_TX_PRIORITY);
  if (atomic_load(&s_talkers_wired) <= 0)
    return prio;
  uint32_t last =
      atomic_load_explicit(&s_talker_tx_done_us, memory_order_acquire);
  uint32_t now_us = (uint32_t)esp_timer_get_time();
  if (now_us - last < AVB_NET_TX_GATE_RECENT_US)
    return prio;
  while (atomic_load_explicit(&s_talker_tx_done_us, memory_order_acquire) ==
             last &&
         (uint32_t)esp_timer_get_time() - now_us < AVB_NET_TX_GATE_MAX_US) {
  }
  return prio;
}

void avb_net_tx_end(UBaseType_t prio) {
  if (prio < AVB_NET_TX_PRIORITY)
    vTaskPrioritySet(NULL, prio);
}
#else
UBaseType_t avb_net_tx_begin(void) { return 0; }
void avb_net_tx_end(UBaseType_t prio) { (void)prio; }
#endif

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
    return (avb_net_wifi_tx_control(0 /* WIFI_IF_STA */, eth_frame,
                                    sizeof(eth_frame)) == ESP_OK)
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

  UBaseType_t prio = avb_net_tx_begin();
  int ret = write(l2if, eth_frame, sizeof(eth_frame));
  avb_net_tx_end(prio);
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
    return (avb_net_wifi_tx_control(0 /* WIFI_IF_STA */, eth_frame,
                                    sizeof(eth_frame)) == ESP_OK)
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

  UBaseType_t prio = avb_net_tx_begin();
  int ret = write(l2if, eth_frame, sizeof(eth_frame));
  avb_net_tx_end(prio);
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
      memcpy(&dest_addr, &ADP_ACMP_MCAST_MAC_ADDR, ETH_ADDR_LEN);
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
      memcpy(&dest_addr, &ADP_ACMP_MCAST_MAC_ADDR, ETH_ADDR_LEN);
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
        avb_net_wifi_tx_control((int)p->wifi_mode, buf, sizeof(eth_frame));
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
    UBaseType_t prio = avb_net_tx_begin();
    int ret = write(l2if, eth_frame, sizeof(eth_frame));
    avb_net_tx_end(prio);
    return ret;
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
#if !defined(CONFIG_ESP_AVB_ROLE_BRIDGE) && defined(CONFIG_ESP_AVB_WIFI_BUNDLE)
/* Uplink bundler on a station: a talker's stream frames go to the
 * bridge several to a Wi-Fi frame. Each stream is sent from one task,
 * so a slot has one writer; the slot table itself is claimed under a
 * lock. */
#define WIFI_TX_BUNDLE_SLOTS 2
#define WIFI_TX_BUNDLE_MAX_AGE_US 2000
/* A partial bundle this old was left when its stream stopped. */
#define WIFI_TX_BUNDLE_STALE_US 200000
/* Enough sent bundles, about 2 ms each, to cover the resend age below.
 * Each entry is a full-size buffer, and a single-core station is short
 * of internal RAM: 16 entries left the Wi-Fi driver without TX buffers
 * once received frames queued for the receive worker. */
#define WIFI_TX_HISTORY 10
#define WIFI_TX_RETX_MAX_AGE_US 18000 /* NACKs without a waiting time */
#define WIFI_TX_RETX_SPACING_US 12000
typedef struct {
  uint16_t seq;
  uint16_t len;
  int64_t sent_us;
  int64_t retx_us;
} wifi_tx_hist_t;
/* A slot idle this long can take another stream ID: a talker moves to a
 * new one when its reservation grows. */
#define WIFI_TX_BUNDLE_IDLE_US 1000000
typedef struct {
  bool valid;
  uint8_t stream_id[UNIQUE_ID_LEN];
  uint16_t seq;
  uint8_t count;
  uint32_t len;
  int64_t first_us;
  int64_t last_us; /* last frame added */
  uint8_t buf[WIFI_BUNDLE_MAX_BYTES];
  /* Sent bundles kept for the bridge's NACKs. */
  uint8_t *hist_buf;
  wifi_tx_hist_t hist[WIFI_TX_HISTORY];
  uint8_t hist_next;
} wifi_tx_bundle_t;
static wifi_tx_bundle_t *s_tx_bundles;
static portMUX_TYPE s_tx_bundle_lock = portMUX_INITIALIZER_UNLOCKED;
static volatile uint32_t s_wifi_tx_bundles, s_wifi_tx_bundle_frames;
static volatile uint32_t s_wifi_tx_nacks_rx, s_wifi_tx_retx;
static volatile uint32_t s_wifi_tx_retx_late, s_wifi_tx_retx_evicted;
static uint8_t *s_tx_retx_buf;

static esp_err_t wifi_tx_bundle_send(wifi_tx_bundle_t *bundle) {
  if (bundle->count == 0)
    return ESP_OK;
  bundle->buf[WIFI_BUNDLE_AVTP + 2] = (uint8_t)bundle->seq;
  bundle->buf[34] = (uint8_t)(bundle->seq >> 8);
  bundle->buf[35] = (uint8_t)bundle->seq;
  bundle->buf[36] = bundle->count;
  esp_err_t result =
      wifi_tx_bundle_frame(0 /* WIFI_IF_STA */, bundle->buf, bundle->len);
  if (result == ESP_OK) {
    s_wifi_tx_bundles++;
    s_wifi_tx_bundle_frames += bundle->count;
  }
  if (!bundle->hist_buf)
    bundle->hist_buf =
        heap_caps_malloc((size_t)WIFI_TX_HISTORY * WIFI_BUNDLE_MAX_BYTES,
                         MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT);
  if (bundle->hist_buf) {
    portENTER_CRITICAL(&s_tx_bundle_lock);
    wifi_tx_hist_t *entry = &bundle->hist[bundle->hist_next];
    memcpy(bundle->hist_buf +
               (size_t)bundle->hist_next * WIFI_BUNDLE_MAX_BYTES,
           bundle->buf, bundle->len);
    entry->seq = bundle->seq;
    entry->len = (uint16_t)bundle->len;
    entry->sent_us = esp_timer_get_time();
    entry->retx_us = 0;
    bundle->hist_next = (uint8_t)((bundle->hist_next + 1) % WIFI_TX_HISTORY);
    portEXIT_CRITICAL(&s_tx_bundle_lock);
  }
  bundle->seq++;
  bundle->count = 0;
  bundle->len = 0;
  return result;
}

/* Returns true when the frame went into a bundle. */
static bool wifi_tx_bundle_add(const uint8_t *frame, size_t len,
                               esp_err_t *result) {
  if (len < WIFI_BUNDLE_AVTP + WIFI_BUNDLE_MIN_AVTP || frame[12] != 0x81 ||
      frame[13] != 0x00 || frame[16] != 0x22 || frame[17] != 0xf0 ||
      (frame[18] & 0x80) || !(frame[19] & 0x80) ||
      WIFI_BUNDLE_FIRST_COST(len - WIFI_BUNDLE_AVTP) > WIFI_BUNDLE_FILL_BYTES)
    return false;
  if (!s_tx_bundles) {
    wifi_tx_bundle_t *slots =
        heap_caps_calloc(WIFI_TX_BUNDLE_SLOTS, sizeof(*slots),
                         MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT);
    if (!slots)
      return false;
    portENTER_CRITICAL(&s_tx_bundle_lock);
    if (!s_tx_bundles) {
      s_tx_bundles = slots;
      slots = NULL;
    }
    portEXIT_CRITICAL(&s_tx_bundle_lock);
    free(slots);
  }
  wifi_tx_bundle_t *bundle = NULL;
  int64_t now_us = esp_timer_get_time();
  portENTER_CRITICAL(&s_tx_bundle_lock);
  for (int i = 0; i < WIFI_TX_BUNDLE_SLOTS && !bundle; i++)
    if (s_tx_bundles[i].valid &&
        memcmp(s_tx_bundles[i].stream_id, frame + 22, UNIQUE_ID_LEN) == 0)
      bundle = &s_tx_bundles[i];
  for (int i = 0; i < WIFI_TX_BUNDLE_SLOTS && !bundle; i++)
    if (!s_tx_bundles[i].valid ||
        now_us - s_tx_bundles[i].last_us > WIFI_TX_BUNDLE_IDLE_US) {
      /* Its sent bundles belong to the old stream ID: NACKs for the new
       * one must not find them. */
      bundle = &s_tx_bundles[i];
      bundle->valid = true;
      memcpy(bundle->stream_id, frame + 22, UNIQUE_ID_LEN);
      bundle->count = 0;
      memset(bundle->hist, 0, sizeof(bundle->hist));
      bundle->hist_next = 0;
    }
  portEXIT_CRITICAL(&s_tx_bundle_lock);
  if (!bundle)
    return false;
  bundle->last_us = now_us;
  uint32_t avtp_len = len - WIFI_BUNDLE_AVTP;
  const uint8_t *avtp = frame + WIFI_BUNDLE_AVTP;
  *result = ESP_OK;
  /* Sending it would open the next run with the last frames of the
   * previous one (seen: a frame of an earlier run before seq 0). It was
   * never numbered, so dropping it leaves no gap at the bridge. */
  if (bundle->count > 0 && now_us - bundle->first_us > WIFI_TX_BUNDLE_STALE_US)
    bundle->count = 0;
  if (bundle->count > 0 &&
      (bundle->len + WIFI_BUNDLE_ENTRY_COST(bundle->buf, bundle->count, avtp,
                                            avtp_len) >
           WIFI_BUNDLE_FILL_BYTES ||
       now_us - bundle->first_us > WIFI_TX_BUNDLE_MAX_AGE_US))
    *result = wifi_tx_bundle_send(bundle);
  if (bundle->count == 0) {
    bundle->len = wifi_bundle_begin(bundle->buf, frame);
    bundle->first_us = now_us;
  }
  bundle->len += wifi_bundle_put(bundle->buf, bundle->len, avtp, avtp_len);
  bundle->count++;
  if (bundle->count >= CONFIG_ESP_AVB_WIFI_BUNDLE_FRAMES)
    *result = wifi_tx_bundle_send(bundle);
  return true;
}

static int wifi_tx_bundle_frames(const uint8_t *stream_id) {
  int frames = 0;
  portENTER_CRITICAL(&s_tx_bundle_lock);
  for (int i = 0; s_tx_bundles && i < WIFI_TX_BUNDLE_SLOTS; i++)
    if (s_tx_bundles[i].valid &&
        memcmp(s_tx_bundles[i].stream_id, stream_id, UNIQUE_ID_LEN) == 0) {
      frames = s_tx_bundles[i].count;
      break;
    }
  portEXIT_CRITICAL(&s_tx_bundle_lock);
  return frames;
}

/* The bridge asks for uplink bundles it missed. Runs on the Wi-Fi
 * task, concurrently with the talker that sends. */
static void wifi_tx_nack_handle(const uint8_t *nack) {
  s_wifi_tx_nacks_rx++;
  if (!s_tx_bundles)
    return;
  uint16_t first = ((uint16_t)nack[30] << 8) | nack[31];
  int count = nack[32];
  int64_t budget_us =
      (int64_t)(((uint32_t)nack[WIFI_NACK_BUDGET] << 8) |
                nack[WIFI_NACK_BUDGET + 1]) * 100;
  if (!s_tx_retx_buf)
    s_tx_retx_buf = heap_caps_malloc(WIFI_BUNDLE_MAX_BYTES,
                                     MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT);
  if (!s_tx_retx_buf)
    return;
  for (int i = 0; i < WIFI_TX_BUNDLE_SLOTS; i++) {
    wifi_tx_bundle_t *bundle = &s_tx_bundles[i];
    if (!bundle->valid || !bundle->hist_buf ||
        memcmp(bundle->stream_id, nack + 18, UNIQUE_ID_LEN) != 0)
      continue;
    for (int n = 0; n < count; n++) {
      uint16_t want = (uint16_t)(first + n);
      uint16_t len = 0;
      int64_t now_us = esp_timer_get_time();
      bool found = false;
      portENTER_CRITICAL(&s_tx_bundle_lock);
      for (int k = 0; k < WIFI_TX_HISTORY; k++) {
        wifi_tx_hist_t *entry = &bundle->hist[k];
        if (entry->len == 0 || entry->seq != want)
          continue;
        found = true;
        if (budget_us == 0
                ? now_us - entry->sent_us > WIFI_TX_RETX_MAX_AGE_US
                : budget_us - WIFI_NACK_TRIP_US < WIFI_RETX_MIN_BUDGET_US) {
          s_wifi_tx_retx_late++;
          break;
        }
        if (entry->retx_us != 0 &&
            now_us - entry->retx_us < WIFI_TX_RETX_SPACING_US)
          break;
        entry->retx_us = now_us;
        len = entry->len;
        memcpy(s_tx_retx_buf,
               bundle->hist_buf + (size_t)k * WIFI_BUNDLE_MAX_BYTES, len);
        break;
      }
      portEXIT_CRITICAL(&s_tx_bundle_lock);
      if (!found)
        s_wifi_tx_retx_evicted++;
      if (len == 0)
        continue;
      s_tx_retx_buf[37] |= WIFI_BUNDLE_FLAG_RETX;
      if (wifi_tx_bundle_frame(0 /* WIFI_IF_STA */, s_tx_retx_buf, len) ==
          ESP_OK)
        s_wifi_tx_retx++;
    }
    return;
  }
}

void avb_wifi_tx_retx_stats(uint32_t *nacks, uint32_t *retx) {
  if (nacks) *nacks = s_wifi_tx_nacks_rx;
  if (retx) *retx = s_wifi_tx_retx;
}

void avb_wifi_tx_retx_detail(uint32_t *late, uint32_t *evicted) {
  if (late) *late = s_wifi_tx_retx_late;
  if (evicted) *evicted = s_wifi_tx_retx_evicted;
}

void avb_wifi_tx_bundle_stats(uint32_t *bundles, uint32_t *frames) {
  if (bundles) *bundles = s_wifi_tx_bundles;
  if (frames) *frames = s_wifi_tx_bundle_frames;
}
#endif

AVB_TALKER_IRAM_ATTR esp_err_t
avb_net_transmit_raw(esp_eth_handle_t eth_handle, const void *frame,
                     size_t frame_len) {
  if (eth_handle != NULL) {
    return esp_eth_transmit(eth_handle, (void *)frame, frame_len);
  }
#if !defined(CONFIG_ESP_AVB_ROLE_BRIDGE) && defined(CONFIG_ESP_AVB_WIFI_BUNDLE)
  esp_err_t bundled = ESP_OK;
  if (wifi_tx_bundle_add(frame, frame_len, &bundled))
    return bundled;
#endif
  /* Wi-Fi path: esp_wifi_internal_tx copies the frame into one of the
   * driver's dynamic TX buffers before returning, so the caller's buffer
   * is free again at once; no copy of its own is needed. */
  return esp_wifi_internal_tx(0 /* WIFI_IF_STA */, (void *)frame, frame_len);
}

int avb_net_tx_bundle_frames(const uint8_t *stream_id) {
#if !defined(CONFIG_ESP_AVB_ROLE_BRIDGE) && defined(CONFIG_ESP_AVB_WIFI_BUNDLE)
  return wifi_tx_bundle_frames(stream_id);
#else
  (void)stream_id;
  return 0;
#endif
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
