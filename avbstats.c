/*
 * Copyright 2024-2026 Scramble Tools
 * License: MIT
 *
 * ESP_AVB Component — CPU/task diagnostic
 *
 * Temporary investigative tool. Prints per-task %CPU, per-core
 * utilization, and stack high-water marks every AVB_STATS_WINDOW_MS.
 * Computes deltas between two snapshots of uxTaskGetSystemState so
 * each window is independent.
 *
 * Enable by calling avb_cpu_stats_start() during init. Remove or
 * #ifdef-gate once the performance investigation is complete.
 */

#include "avb.h"
#include "esp_heap_caps.h"
#include "esp_timer.h"
#include "soc/soc_caps.h"
#if SOC_EMAC_SUPPORTED
#include "esp_rom_sys.h" /* esp_rom_delay_us for the RX FIFO recovery */
#include "soc/emac_dma_struct.h"
#include "soc/emac_mac_struct.h"
#endif
/* L1 instrument: esp_wifi driver/MAC RX counters, available only on a
 * native HE Wi-Fi target (the C6 endpoint). The P4 bridge reaches Wi-Fi
 * via esp_wifi_remote and exposes no local HE stats, so gate it out.
 *
 * Both headers live in esp_wifi's public "include" dir (the private one
 * at include/esp_private/), reachable now that esp_wifi is in the
 * component's PRIV_REQUIRES. */
#if defined(CONFIG_IDF_TARGET_ESP32C6)
#define AVB_WIFI_RX_STATS 1
#include "esp_wifi_he.h"
#include "esp_private/esp_wifi_he_private.h"
#endif
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

#define AVB_STATS_MAX_TASKS 32
#define AVB_STATS_WINDOW_MS 10000

/* Pointer to avb_state so AVB-STATS can drive the MCLK print that was
 * previously embedded in the main loop. Stream-in/out diag state is
 * owned by their respective contexts in avtp.c. */
static avb_state_s *s_state = NULL;

#if SOC_EMAC_SUPPORTED
static uint32_t s_emac_kick_count = 0;
static uint32_t s_emac_toggle_count = 0;
static uint32_t s_emac_recover_count = 0;
#endif

/* EMAC RX un-wedge guard — call frequently (main AVB loop).
 *
 * Two distinct ingress-death modes recovered here, both triggered by
 * RX descriptor-exhaustion bursts and both otherwise permanent:
 *
 * Case 1 — RX DMA Suspended with free descriptors: the driver's
 * recovery (free descriptors, then write the receive poll demand) can
 * lose a race against the DMA's transition into Suspended; the demand
 * is ignored and a suspended DMA raises no further interrupts.
 * Re-issuing the poll demand is always safe: with free descriptors the
 * DMA resumes; with none it re-raises RBU, which wakes the RX task.
 *
 * Case 2 — MTL RX FIFO lockup: after a FIFO overflow during the burst
 * the MTL stops forwarding to the DMA entirely. Observed live state:
 * receive process "Running: waiting for packet" (RS=3), all
 * descriptors free, overflow flag latched, interrupt counters frozen
 * while the wire carries full-rate traffic. The overflow interrupt
 * summary only edge-triggers, so nothing ever fires again. Recovery
 * per DWC practice: stop the RX DMA, toggle the MAC receiver enable
 * (resets the MTL RX FIFO state), restart, poll demand. Detection is
 * debounced on a stagnant RX buffer pointer so transient overflow
 * during normal operation never triggers it; a false trigger would
 * cost ~100 µs of reception, vs. permanent deafness untreated. */
void avb_emac_rx_unwedge_tick(void) {
#if SOC_EMAC_SUPPORTED
  uint32_t status = EMAC_DMA.dmastatus.val;
  uint32_t recv_state = (status >> 17) & 0x7; /* RS, bits 19:17 */

  /* Case 1: Suspended (descriptor unavailable) = 100b */
  if (recv_state == 4) {
    EMAC_DMA.dmarxpolldemand = 1;
    s_emac_kick_count++;
    return;
  }

  /* Case 2: Running-waiting (011b) + latched overflow + stagnant RX
   * buffer pointer across consecutive passes. The P4 MTL RX FIFO is
   * only 256 B, forcing threshold mode (store-and-forward cannot fit a
   * frame), and threshold-mode FIFOs can latch dead after an overflow.
   * Escalate: first the cheap MAC-receiver toggle; if the pointer is
   * still stagnant afterwards, a full EMAC core software reset +
   * reconfigure (emac_esp_rx_full_recover — the only recovery that
   * rebuilds the FIFO state), rate-limited so a persistent hardware
   * fault cannot become a reset storm. */
  static uint32_t last_rx_buf = 0;
  static uint32_t stagnant_passes = 0;
  static int64_t last_restart_us = 0;
  uint32_t cur_rx_buf = EMAC_DMA.dmarxcurraddr_buf;
  bool overflow_latched = (status & 0x10) != 0;
  if (recv_state == 3 && overflow_latched && cur_rx_buf == last_rx_buf) {
    stagnant_passes++;
    if (stagnant_passes == 3) {
      /* Cheap attempt: receiver toggle + poll demand. */
      EMAC_DMA.dmaoperation_mode.start_stop_rx = 0;
      EMAC_MAC.gmacconfig.rx = 0;
      esp_rom_delay_us(100);
      EMAC_MAC.gmacconfig.rx = 1;
      EMAC_DMA.dmaoperation_mode.start_stop_rx = 1;
      EMAC_DMA.dmarxpolldemand = 1;
      s_emac_toggle_count++;
    } else if (stagnant_passes >= 6) {
      int64_t now_us = esp_timer_get_time();
      if (now_us - last_restart_us > 10 * 1000 * 1000) {
        last_restart_us = now_us;
        s_emac_recover_count++;
        /* Full EMAC core reset + reconfigure (local IDF patch). A plain
         * esp_eth stop/start cannot clear the latched MTL FIFO; only
         * the DWC software reset does, and it needs the link-up PHY
         * clocks that are guaranteed present in this state. */
        extern esp_err_t emac_esp_rx_full_recover(void);
        emac_esp_rx_full_recover();
      }
      stagnant_passes = 0;
    }
  } else {
    stagnant_passes = 0;
  }
  last_rx_buf = cur_rx_buf;
#endif
}

typedef struct {
  UBaseType_t tn; /* xTaskNumber — stable per task, used to match snapshots */
  uint32_t rt;    /* ulRunTimeCounter at snapshot time */
} snap_entry_t;

static snap_entry_t s_prev[AVB_STATS_MAX_TASKS];
static UBaseType_t s_prev_n = 0;
static int64_t s_prev_at_us = 0;

/* Previous cumulative productive-work microseconds for AVB-OUT. Delta
 * against the window gives the "real" CPU AVB-OUT needs (excludes
 * busy-wait). */
static uint64_t s_prev_avbout_work_us = 0;

/* Previous cumulative PTP frame count seen in avb_unified_rx_cb. Delta is
 * the count of PTP frames that emac_rx delivered to our callback this
 * window — compare against ptpd's rx_sync to see if drops are in our
 * callback or in the L2TAP filter/queue path below it. */
static uint32_t s_prev_ptp_rx_seen = 0;

/* Starts with "IDLE" — we match by prefix because ESP-IDF's idle task
 * name is "IDLE0" / "IDLE1" on dual-core builds. */
static bool is_idle_task(const char *name) {
  return name && name[0] == 'I' && name[1] == 'D' && name[2] == 'L' &&
         name[3] == 'E';
}

static void avb_cpu_stats_tick(void) {
  TaskStatus_t cur[AVB_STATS_MAX_TASKS];
  uint32_t cur_total;
  UBaseType_t cur_n =
      uxTaskGetSystemState(cur, AVB_STATS_MAX_TASKS, &cur_total);
  int64_t now = esp_timer_get_time();
  int64_t window_us = now - s_prev_at_us;

  /* On the first call we just save the snapshot and wait for the next
   * window — there's no prior baseline to diff against. */
  if (s_prev_at_us == 0 || window_us < 1000) {
    goto save_baseline;
  }

  uint32_t idle0_delta = 0, idle1_delta = 0;
  avbinfo("CPU-STATS: window=%lldms (%u tasks)", window_us / 1000,
          (unsigned)cur_n);
  avbinfo("  %-16s core prio  %%cpu  stackHW  state", "task");

  for (UBaseType_t i = 0; i < cur_n; i++) {
    /* Find matching prior entry by stable xTaskNumber. */
    uint32_t prev_rt = 0;
    bool found = false;
    for (UBaseType_t j = 0; j < s_prev_n; j++) {
      if (s_prev[j].tn == cur[i].xTaskNumber) {
        prev_rt = s_prev[j].rt;
        found = true;
        break;
      }
    }
    uint32_t drt = found ? (cur[i].ulRunTimeCounter - prev_rt) : 0;

    /* Runtime counter is sourced from esp_timer (µs) in ESP-IDF by
     * default, matching window_us units — so percentage is direct. */
    uint32_t pct = (uint32_t)((uint64_t)drt * 100u / (uint64_t)window_us);

    const char *core_s;
    switch (cur[i].xCoreID) {
    case 0:
      core_s = "0";
      break;
    case 1:
      core_s = "1";
      break;
    default:
      core_s = "*";
      break; /* tskNO_AFFINITY / unpinned */
    }

    const char *state_s;
    switch (cur[i].eCurrentState) {
    case eRunning:
      state_s = "run";
      break;
    case eReady:
      state_s = "rdy";
      break;
    case eBlocked:
      state_s = "blk";
      break;
    case eSuspended:
      state_s = "sus";
      break;
    case eDeleted:
      state_s = "del";
      break;
    default:
      state_s = "?";
      break;
    }

    avbinfo("  %-16s %4s %4u  %3u%%  %7u  %s", cur[i].pcTaskName, core_s,
            (unsigned)cur[i].uxCurrentPriority, pct,
            (unsigned)cur[i].usStackHighWaterMark, state_s);

    if (is_idle_task(cur[i].pcTaskName)) {
      if (cur[i].xCoreID == 0)
        idle0_delta = drt;
      else if (cur[i].xCoreID == 1)
        idle1_delta = drt;
    }
  }

  /* Per-core busy % derived directly from IDLE runtime on that core.
   * Each core had `window_us` of wall time available; IDLE time is
   * time not spent running anything else. */
  uint32_t cpu0_busy =
      100 - (uint32_t)((uint64_t)idle0_delta * 100u / (uint64_t)window_us);
  uint32_t cpu1_busy =
      100 - (uint32_t)((uint64_t)idle1_delta * 100u / (uint64_t)window_us);
  avbinfo("  ====> CPU0 busy=%u%% idle=%u%%  |  CPU1 busy=%u%% idle=%u%%",
          cpu0_busy, 100 - cpu0_busy, cpu1_busy, 100 - cpu1_busy);

  /* Break AVB-OUT's CPU time down into "real work" vs. busy-wait spin.
   * work_pct = productive µs accumulated this window / window µs × 100.
   * The busy-wait portion = (AVB-OUT total %CPU) − work_pct. */
  uint64_t cur_work = avb_stream_out_work_us_total();
  if (cur_work >= s_prev_avbout_work_us) {
    uint64_t dwork = cur_work - s_prev_avbout_work_us;
    uint32_t work_pct =
        (uint32_t)((dwork * 100ull) / (uint64_t)window_us);
    avbinfo("  ====> AVB-OUT real work=%u%% (rest of its slot is busy-wait)",
            work_pct);
  }
  s_prev_avbout_work_us = cur_work;

  /* PTP frames seen in our RX callback this window. ptpd's rx_sync from
   * the ptpd-late line should be close to this for syncs; a gap here
   * means L2TAP tail-dropped on full queue. */
  uint32_t cur_ptp = avb_net_ptp_rx_seen();
  uint32_t dptp = cur_ptp - s_prev_ptp_rx_seen;
  avbinfo("  ====> PTP frames seen in emac_rx_cb this window: %u", dptp);
  s_prev_ptp_rx_seen = cur_ptp;

  /* Per-ethertype RX breakdown into avb_unified_rx_cb (monotonic totals).
   * Diagnostic for SoftAP→STA delivery: if total stays flat on a Wi-Fi
   * endpoint while the bridge's fwd-wifi counter is climbing, the
   * SoftAP isn't actually putting our frames OTA (or the STA filter
   * is rejecting them). */
  uint32_t rx_total, rx_avtp, rx_msrp, rx_mvrp, rx_vlan, rx_other;
  avb_net_rx_breakdown(&rx_total, &rx_avtp, &rx_msrp, &rx_mvrp, &rx_vlan,
                       &rx_other);
  avbinfo("  ====> RX breakdown total=%u avtp=%u msrp=%u mvrp=%u vlan=%u other=%u",
          rx_total, rx_avtp, rx_msrp, rx_mvrp, rx_vlan, rx_other);

#ifdef AVB_WIFI_RX_STATS
  /* L1/L2 RX localization for the C6 Wi-Fi RX-stall. Three layers, all
   * printed here in the slow stats task — NO logging in the RX hot path:
   *   L1 = esp_wifi driver/MAC counters (esp_wifi_get_rx_statistics):
   *        frames the radio+driver actually received, below our callback.
   *   L2 = avb_net_wifi_rx_cb_count(): entries into avb_wifi_rx_cb,
   *        counted before the per-frame malloc.
   *   L3 = RX breakdown total (above): frames that reached the dispatcher.
   * In the frozen state, L1-climbs-while-L2/L3-frozen => the driver
   * stopped delivering to our callback (host-side wedge); L1-also-frozen
   * => the stall is at/below the MAC/driver RX path. esp_wifi_get_rx_
   * statistics counts ALL PHY RX (beacons etc.), so discriminate AECP
   * delivery with a dense burst vs the beacon-only baseline, not absolute
   * values. Enable the driver's stat collection once (HT/legacy SU only;
   * skip the heavy MU-MIMO/OFDMA buffers). */
  static bool s_rx_stats_enabled = false;
  if (!s_rx_stats_enabled &&
      esp_wifi_enable_rx_statistics(true, false) == ESP_OK) {
    s_rx_stats_enabled = true;
  }
  esp_test_rx_statistics_t rxs;
  memset(&rxs, 0, sizeof(rxs));
  if (s_rx_stats_enabled && esp_wifi_get_rx_statistics(0, &rxs) == ESP_OK) {
    avbinfo("  ====> WIFI L1 rx legacy=%u ht=%u ht_retry=%u rx_isr=%u "
            "rx_nblks=%u | L2 wifi_cb=%u",
            rxs.legacy, rxs.ht, rxs.ht_retry, rxs.rx_isr, rxs.rx_nblks,
            avb_net_wifi_rx_cb_count());
  } else {
    avbinfo("  ====> WIFI L1 rx stats pending/unavailable | L2 wifi_cb=%u",
            avb_net_wifi_rx_cb_count());
  }
#endif

  /* Heap watcher — surfaces exhaustion vs. fragmentation. The wifi
   * RX path mallocs per-frame at ~1500 B; when the largest contiguous
   * block falls below that, RX stops cold. */
  size_t heap_free = heap_caps_get_free_size(MALLOC_CAP_DEFAULT);
  size_t heap_largest = heap_caps_get_largest_free_block(MALLOC_CAP_DEFAULT);
  size_t heap_min_ever = heap_caps_get_minimum_free_size(MALLOC_CAP_DEFAULT);
  avbinfo("  ====> HEAP free=%u largest=%u min_ever=%u",
          (unsigned)heap_free, (unsigned)heap_largest, (unsigned)heap_min_ever);

  /* EMAC DMA hardware missed-frame counter. missed_fc counts frames the
   * DMA discarded because the Host Receive Buffer was unavailable (the
   * RX descriptor ring was full). overflow_fc counts frames lost inside
   * the MTL FIFO (before DMA). Both fields clear on read. If missed_fc
   * is non-zero we're losing packets at the NIC because emac_rx isn't
   * draining fast enough — the most likely explanation for PTP frames
   * visible on the wire but never reaching avb_unified_rx_cb.
   *
   * Only meaningful on targets with on-chip EMAC (e.g. esp32p4). On
   * Wi-Fi-only targets (e.g. esp32c6 endpoint) the AVB ingress comes
   * from esp_wifi instead and these registers don't exist. */
#if SOC_EMAC_SUPPORTED
  uint32_t dma_miss = EMAC_DMA.dmamissedfr.val;
  avbinfo("  ====> EMAC DMA missed_fc=%u overflow_fc=%u bmfc_ovf=%u bfoc_ovf=%u",
          (unsigned)(dma_miss & 0xFFFF),
          (unsigned)((dma_miss >> 17) & 0x7FF),
          (unsigned)((dma_miss >> 16) & 1),
          (unsigned)((dma_miss >> 28) & 1));
  /* RX un-wedge guard activity (see avb_emac_rx_unwedge_tick): all
   * three counters are 0 in normal operation. Nonzero kicks/toggles
   * mean the guard caught a suspended DMA; nonzero recovers mean a
   * full MTL FIFO reset was needed. */
  avbinfo("  ====> EMAC unwedge kicks=%u toggles=%u recovers=%u rs=%u",
          (unsigned)s_emac_kick_count,
          (unsigned)s_emac_toggle_count,
          (unsigned)s_emac_recover_count,
          (unsigned)((EMAC_DMA.dmastatus.val >> 17) & 0x7));
#endif

  /* Stream-in (listener) + stream-out (talker) + MCLK / PLL state —
   * centralized periodic diagnostics. Each print function is silent
   * when its session isn't active. */
  avb_stream_in_print_diag();
  avb_stream_out_print_diag();
  if (s_state)
    avb_pll_print_stats(s_state);

save_baseline:
  s_prev_n = cur_n < AVB_STATS_MAX_TASKS ? cur_n : AVB_STATS_MAX_TASKS;
  for (UBaseType_t i = 0; i < s_prev_n; i++) {
    s_prev[i].tn = cur[i].xTaskNumber;
    s_prev[i].rt = cur[i].ulRunTimeCounter;
  }
  s_prev_at_us = now;
}

static void avb_cpu_stats_task(void *arg) {
  (void)arg;
  /* Initial settle before the first snapshot. */
  vTaskDelay(pdMS_TO_TICKS(5000));
  avb_cpu_stats_tick(); /* seed baseline */
  for (;;) {
    vTaskDelay(pdMS_TO_TICKS(AVB_STATS_WINDOW_MS));
    avb_cpu_stats_tick();
  }
}

void avb_cpu_stats_set_state(avb_state_s *state) {
  s_state = state;
}

void avb_cpu_stats_start(void) {
  /* Pin to core 0. Unpinned placed us on core 1 behind AVB-OUT's prio-24
   * busy-wait, which meant we only woke up every ~22 s instead of the
   * requested 10 s — making the "window" math in the per-task %CPU
   * numbers misleading. Core 0 has >20 % idle headroom even under
   * streaming, so a low-priority sampler there is reliable. Priority 5
   * sits above AVB-NVS (3) and PTPD (2) so neither can delay our wake,
   * and well below real-time work so we never disturb it. */
  xTaskCreatePinnedToCore(avb_cpu_stats_task, "AVB-STATS", 4096, NULL, 5,
                          NULL, 0);
}
