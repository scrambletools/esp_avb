/*
 * Copyright 2024-2026 Scramble Tools
 * License: MIT
 *
 * ESP_AVB Component
 *
 * This component provides an implementation of an AVB talker and listener.
 *
 * This file provides the codec interface for the ESP_AVB component.
 */

#include "avb.h"
#include "es8311_codec.h"
#include "es8388_codec.h"
#include "es8389_codec.h"
#include "esp_codec_dev.h"
#include "esp_codec_dev_defaults.h"
#include "soc/soc_caps.h" /* SOC_CLK_APLL_SUPPORTED */
#include <stdatomic.h>    /* media-clock byte counter in i2s_tx_on_sent_cb */
#include <string.h>

#define I2C_NUM (0)
/* Clocking is a codec property, so it lives in each codec's clock plan
 * below (avb_codec_clock_plan_s): the MCLKs a codec can run each rate
 * from, in preference order. The I2S master derives BCLK = 64 fs from
 * MCLK with an integer divider (2 x 32-bit slots, which also lifts the
 * driver's multiple-of-3 rule for 24-bit slots), so every candidate is
 * a multiple of 64 fs, and the APLL runs at MCLK x 2. Candidate order
 * encodes the preferred family: a rate change keeps the MCLK already
 * running whenever the new rate lists it, which is what lets the APLL
 * and its converged trim survive the change (avb_audio_set_rate). The
 * plan is checked once per boot (avb_codec_validate_clock_plan) and a
 * candidate that fails is skipped from then on.
 *
 * ES8389 (datasheet rev 6.0): MCLK <= 49.2 MHz, LRCK <= 192 kHz. Its
 * plan is the set of (rate, MCLK) pairs the esp_codec_dev coefficient
 * table has a row for; 176.4 kHz has none, so it is not offered, and
 * 192 kHz only runs at 128 fs (256 fs = 49.152 MHz silences the ADC).
 * The ESP32-P4 full-duplex RX slave warns below MCLK/BCLK 4; ratio 2
 * (88.2 and 192 kHz) is proven clean on the wire, and half of the
 * RATE-CHANGE starts carry the slot-slip episodes at every multiple
 * tried, so that defect is not a property of the MCLK (bench notes).
 * 48 kHz prefers 24.576 MHz (512 fs): constant MCLK across 48/96/192
 * kHz, the XMOS lib_tsn arrangement, A/B-tested equal to 384 fs. */

#define TAG "AVB-CODEC"

/* ES8389 clock-manager rows for the (MCLK, LRCK) pairs this driver
 * generates, copied from esp_codec_dev's es8389.c coefficient table
 * (Reg0x04..0x0A, 0x0F, 0x11, 0x21, 0x22, 0x26, 0x30, 0x41, 0x42, 0x43,
 * 0xF0, 0xF1, 0x16, 0x18, 0x19). With use_mclk the driver's set_fs
 * skips that table and leaves the codec's auto clock mode in charge,
 * which is fine through 96 kHz but at 192 kHz the ADC keeps running at
 * single speed and repeats every sample (wire capture: consecutive
 * samples in near-identical pairs, a 96 kHz signal on a 192 kHz frame
 * clock). Programming the row the driver itself would use puts the
 * modulator and oversampling ratios where the rate needs them. */
typedef struct {
  uint32_t rate_hz;
  uint32_t mclk_hz;
  uint8_t reg[21];
} es8389_clock_row_s;

static const es8389_clock_row_s s_es8389_row_44k1_at_11m = {
    44100, 11289600,
    {0x01, 0x41, 0x04, 0xD0, 0x10, 0xD1, 0x80,
     0x40, 0x00, 0x1F, 0x7F, 0xBF, 0xC0, 0x7F,
     0x7F, 0x00, 0x12, 0x00, 0x35, 0x91, 0x28}};
static const es8389_clock_row_s s_es8389_row_48k_at_18m = {
    48000, 18432000,
    {0x02, 0x41, 0x04, 0xD0, 0x10, 0xD1, 0x80,
     0x40, 0x00, 0x1F, 0x7F, 0xBF, 0xC0, 0x7F,
     0x7F, 0x00, 0x12, 0x00, 0x35, 0x91, 0x28}};
static const es8389_clock_row_s s_es8389_row_48k_at_24m = {
    48000, 24576000,
    {0x03, 0x41, 0x04, 0xD0, 0x10, 0xD1, 0x80,
     0xC0, 0x00, 0x1F, 0x7F, 0xBF, 0xC0, 0x7F,
     0x7F, 0x00, 0x12, 0x00, 0x35, 0x91, 0x28}};
static const es8389_clock_row_s s_es8389_row_88k2_at_11m = {
    88200, 11289600,
    {0x00, 0x50, 0x00, 0xC0, 0x10, 0xC1, 0x80,
     0x40, 0x00, 0x9F, 0x7F, 0xBF, 0xC0, 0x7F,
     0x7F, 0x80, 0x12, 0xC0, 0x32, 0x89, 0x25}};
static const es8389_clock_row_s s_es8389_row_96k_at_24m = {
    96000, 24576000,
    {0x00, 0x40, 0x00, 0xC0, 0x10, 0xC1, 0x80,
     0xC0, 0x00, 0x9F, 0x7F, 0xBF, 0xC0, 0x7F,
     0x7F, 0x80, 0x12, 0xC0, 0x35, 0x91, 0x28}};
static const es8389_clock_row_s s_es8389_row_192k_at_24m = {
    192000, 24576000,
    {0x00, 0x50, 0x00, 0xC0, 0x18, 0xC1, 0x81,
     0xC0, 0x00, 0x8F, 0x7F, 0xEF, 0xC0, 0x3F,
     0x7F, 0x80, 0x12, 0xC0, 0x3F, 0xF9, 0x3F}};

/* Program the codec's clock manager for (rate, MCLK); defined with the
 * ES8389 factory below. */
static esp_err_t es8389_apply_clock(avb_state_s *state, uint32_t rate_hz,
                                    uint32_t mclk_hz, const void *codec_data);

/* ES8389 clock plan: every (rate, MCLK) pair above. 48 kHz prefers the
 * 24.576 MHz family shared with 96/192 kHz. */
static const avb_codec_clock_plan_s s_es8389_clock_plan[] = {
    {44100, {11289600, 0}, {&s_es8389_row_44k1_at_11m, NULL}},
    {48000,
     {24576000, 18432000},
     {&s_es8389_row_48k_at_24m, &s_es8389_row_48k_at_18m}},
    {88200, {11289600, 0}, {&s_es8389_row_88k2_at_11m, NULL}},
    {96000, {24576000, 0}, {&s_es8389_row_96k_at_24m, NULL}},
    {192000, {24576000, 0}, {&s_es8389_row_192k_at_24m, NULL}},
};

/* ES8311 / ES8388: the esp_codec_dev drivers derive the codec's own
 * dividers from (MCLK, fs) in set_fs, so these plans carry no rows.
 * Candidates are the driver coefficient rows that keep MCLK/BCLK at 4
 * or better; 384 fs at 48 kHz is the multiple both ran on before the
 * plan existed. Not re-verified on hardware since. */
static const avb_codec_clock_plan_s s_es8311_clock_plan[] = {
    {48000, {18432000, 12288000}, {NULL, NULL}},
    {96000, {24576000, 0}, {NULL, NULL}},
};
static const avb_codec_clock_plan_s s_es8388_clock_plan[] = {
    {48000, {18432000, 12288000}, {NULL, NULL}},
    {96000, {24576000, 0}, {NULL, NULL}},
};
#define AVB_PLAN_ROWS(plan) (sizeof(plan) / sizeof((plan)[0]))

static const avb_codec_caps_s s_es8311_caps = {
    .bit_rates = {.bit_rates = {24}, .num_rates = 1},
    .max_input_channels = 1,
    .max_output_channels = 1,
    .control_ranges = {.vol_min_tenth_db = -955,
                       .vol_max_tenth_db = 320,
                       .vol_step_tenth_db = 5,
                       .vol_default_tenth_db = 100,
                       .gain_min_tenth_db = 0,
                       .gain_max_tenth_db = 420,
                       .gain_step_tenth_db = 60,
                       .gain_default_tenth_db = 60},
    .clock_plan = s_es8311_clock_plan,
    .num_clock_plans = AVB_PLAN_ROWS(s_es8311_clock_plan),
    .max_mclk_hz = 24576000, /* highest MCLK in the driver's table */
    .min_mclk_bclk_ratio = 4,
    .apply_clock = NULL,
};

/* ES8388: 24-bit, max 96 kHz (datasheet); 2-ch ADC/DAC. DAC digital
 * volume -96..0 dB (0.5 dB step), mic PGA 0..24 dB (3 dB step). */
static const avb_codec_caps_s s_es8388_caps = {
    .bit_rates = {.bit_rates = {24}, .num_rates = 1},
    .max_input_channels = 2,
    .max_output_channels = 2,
    .control_ranges = {.vol_min_tenth_db = -960,
                       .vol_max_tenth_db = 0,
                       .vol_step_tenth_db = 5,
                       .vol_default_tenth_db = -100,
                       .gain_min_tenth_db = 0,
                       .gain_max_tenth_db = 240,
                       .gain_step_tenth_db = 30,
                       .gain_default_tenth_db = 90},
    .clock_plan = s_es8388_clock_plan,
    .num_clock_plans = AVB_PLAN_ROWS(s_es8388_clock_plan),
    .max_mclk_hz = 24576000, /* 256 fs at its 96 kHz ceiling */
    .min_mclk_bclk_ratio = 4,
    .apply_clock = NULL,
};

/* ES8389: 24-bit, up to 192 kHz (rates per its clock plan); 2-ch ADC/DAC. DAC digital volume
 * -95.5..+32 dB (0.5 dB step) and mic PGA 0..36.5 dB (~3 dB step) per
 * the esp_codec_dev es8389 driver vol_range and PGA gain table. */
static const avb_codec_caps_s s_es8389_caps = {
    .bit_rates = {.bit_rates = {24}, .num_rates = 1},
    .max_input_channels = 2,
    .max_output_channels = 2,
    .control_ranges = {.vol_min_tenth_db = -955,
                       .vol_max_tenth_db = 320,
                       .vol_step_tenth_db = 5,
                       .vol_default_tenth_db = -100,
                       .gain_min_tenth_db = 0,
                       .gain_max_tenth_db = 365,
                       .gain_step_tenth_db = 30,
                       .gain_default_tenth_db = 90},
    .clock_plan = s_es8389_clock_plan,
    .num_clock_plans = AVB_PLAN_ROWS(s_es8389_clock_plan),
    .max_mclk_hz = 49200000, /* datasheet rev 6.0 */
    .min_mclk_bclk_ratio = 2, /* ratio 2 proven at 88.2 and 192 kHz */
    .apply_clock = es8389_apply_clock,
};

int16_t avb_codec_quantize_tenth_db(const codec_control_range_s *ranges,
                                     bool gain, int16_t value_tenth_db) {
  int16_t min = gain ? ranges->gain_min_tenth_db : ranges->vol_min_tenth_db;
  int16_t max = gain ? ranges->gain_max_tenth_db : ranges->vol_max_tenth_db;
  int16_t step = gain ? ranges->gain_step_tenth_db : ranges->vol_step_tenth_db;

  if (value_tenth_db <= min) {
    return min;
  }
  if (value_tenth_db >= max) {
    return max;
  }
  if (step <= 0) {
    return value_tenth_db;
  }

  int32_t offset = value_tenth_db - min;
  int32_t steps = (offset + (step / 2)) / step;
  int32_t quantized = min + (steps * step);
  if (quantized < min) {
    quantized = min;
  } else if (quantized > max) {
    quantized = max;
  }
  return (int16_t)quantized;
}

const avb_codec_caps_s *avb_codec_get_caps(avb_codec_type_t codec_type) {
  switch (codec_type) {
  case avb_codec_type_es8311:
    return &s_es8311_caps;
  case avb_codec_type_es8388:
    return &s_es8388_caps;
  case avb_codec_type_es8389:
    return &s_es8389_caps;
  default:
    return NULL;
  }
}

/* The plan row for sample_rate, with its index for the reject mask. */
static const avb_codec_clock_plan_s *
codec_plan_for_rate(const avb_codec_caps_s *caps, uint32_t sample_rate,
                    uint8_t *row_index) {
  for (uint8_t i = 0; i < caps->num_clock_plans; i++) {
    if (caps->clock_plan[i].sample_rate_hz == sample_rate) {
      if (row_index)
        *row_index = i;
      return &caps->clock_plan[i];
    }
  }
  return NULL;
}

static bool codec_caps_support_sample_rate(const avb_codec_caps_s *caps,
                                           uint32_t sample_rate) {
  return codec_plan_for_rate(caps, sample_rate, NULL) != NULL;
}

void avb_codec_plan_sample_rates(const avb_codec_caps_s *caps,
                                 avb_sample_rates_s *out) {
  memset(out, 0, sizeof(*out));
  size_t max_rates = sizeof(out->sample_rates) / sizeof(out->sample_rates[0]);
  for (uint8_t i = 0; i < caps->num_clock_plans && out->num_rates < max_rates;
       i++) {
    out->sample_rates[out->num_rates++] = caps->clock_plan[i].sample_rate_hz;
  }
}

uint32_t avb_codec_select_mclk(const avb_state_s *state, uint32_t rate,
                               uint32_t preferred_mclk_hz,
                               const void **codec_data) {
  const avb_codec_caps_s *caps = avb_codec_get_caps(state->config.codec_type);
  uint8_t row_index = 0;
  const avb_codec_clock_plan_s *plan =
      caps ? codec_plan_for_rate(caps, rate, &row_index) : NULL;
  if (codec_data)
    *codec_data = NULL;
  if (!plan)
    return 0;
  int chosen = -1;
  for (int candidate = 0; candidate < AVB_CODEC_MCLK_CANDIDATES; candidate++) {
    uint32_t mclk_hz = plan->mclk_hz[candidate];
    if (mclk_hz == 0)
      break;
    uint32_t bit_index =
        (uint32_t)row_index * AVB_CODEC_MCLK_CANDIDATES + (uint32_t)candidate;
    if (bit_index < 32 && (state->clock_plan_invalid_mask & (1u << bit_index)))
      continue;
    if (mclk_hz == preferred_mclk_hz) {
      chosen = candidate;
      break;
    }
    if (chosen < 0)
      chosen = candidate;
  }
  if (chosen < 0)
    return 0;
  if (codec_data)
    *codec_data = plan->codec_data[chosen];
  return plan->mclk_hz[chosen];
}

esp_err_t avb_codec_validate_clock_plan(avb_state_s *state) {
  const avb_codec_caps_s *caps = avb_codec_get_caps(state->config.codec_type);
  if (!caps || !caps->clock_plan)
    return ESP_ERR_INVALID_STATE;
  const uint32_t bclk_per_fs = 2u * 32u; /* stereo 32-bit slots */
  int rejected = 0;
  state->clock_plan_invalid_mask = 0;
  for (uint8_t row = 0; row < caps->num_clock_plans; row++) {
    const avb_codec_clock_plan_s *plan = &caps->clock_plan[row];
    uint32_t bclk_hz = plan->sample_rate_hz * bclk_per_fs;
    for (int candidate = 0; candidate < AVB_CODEC_MCLK_CANDIDATES; candidate++) {
      uint32_t mclk_hz = plan->mclk_hz[candidate];
      if (mclk_hz == 0)
        break;
      uint32_t apll_hz = 0;
      const char *reason = NULL;
      if (mclk_hz % bclk_hz != 0)
        reason = "MCLK/BCLK is not an integer";
      else if (mclk_hz / bclk_hz < caps->min_mclk_bclk_ratio)
        reason = "MCLK/BCLK below the codec floor";
      else if (mclk_hz > caps->max_mclk_hz)
        reason = "MCLK above the codec ceiling";
      else if (caps->apply_clock && !plan->codec_data[candidate])
        reason = "no codec clock row";
      else if (!avb_pll_mclk_derivable(mclk_hz, &apll_hz))
        reason = "APLL cannot produce this MCLK";
      bool running = mclk_hz == state->codec_mclk_hz &&
                     plan->sample_rate_hz == state->config.default_sample_rate;
      uint32_t bit_index =
          (uint32_t)row * AVB_CODEC_MCLK_CANDIDATES + (uint32_t)candidate;
      if (reason) {
        rejected++;
        if (bit_index < 32)
          state->clock_plan_invalid_mask |= 1u << bit_index;
        ESP_LOGE(TAG, "clock plan: %lu Hz from MCLK %lu Hz (%lu fs) rejected: %s%s",
                 (unsigned long)plan->sample_rate_hz, (unsigned long)mclk_hz,
                 (unsigned long)(mclk_hz / plan->sample_rate_hz), reason,
                 running ? " (currently running)" : "");
      } else {
        ESP_LOGI(TAG,
                 "clock plan: %lu Hz from MCLK %lu Hz (%lu fs, MCLK/BCLK %lu, "
                 "APLL %lu Hz)%s",
                 (unsigned long)plan->sample_rate_hz, (unsigned long)mclk_hz,
                 (unsigned long)(mclk_hz / plan->sample_rate_hz),
                 (unsigned long)(mclk_hz / bclk_hz), (unsigned long)apll_hz,
                 running ? " active" : "");
      }
    }
  }
  if (rejected) {
    ESP_LOGW(TAG, "clock plan: %d candidate(s) rejected, see above", rejected);
    return ESP_FAIL;
  }
  ESP_LOGI(TAG, "clock plan: all %u rates check out", caps->num_clock_plans);
  return ESP_OK;
}

/* TX DMA completion callback — accumulates DAC-consumed bytes for the
 * media-clock PLL. ISR context: one relaxed atomic add, nothing else. */
static IRAM_ATTR bool i2s_tx_on_sent_cb(i2s_chan_handle_t handle,
                                        i2s_event_data_t *event, void *arg) {
  avb_state_s *state = (avb_state_s *)arg;
  atomic_fetch_add_explicit(&state->media_clock.i2s_bytes_written,
                            (uint64_t)event->size, memory_order_relaxed);
  return false;
}

/* RX DMA completion callback — accumulates ADC-captured bytes: the
 * talker-side media-clock sensor (see i2s_bytes_captured in avb.h).
 * Same ISR discipline as the TX twin. */
static IRAM_ATTR bool i2s_rx_on_recv_cb(i2s_chan_handle_t handle,
                                        i2s_event_data_t *event, void *arg) {
  avb_state_s *state = (avb_state_s *)arg;
  atomic_fetch_add_explicit(&state->media_clock.i2s_bytes_captured,
                            (uint64_t)event->size, memory_order_relaxed);
  return false;
}

/* Configure the I2S driver
 * Typically the I2S driver must be reconfigured when the stream params change
 *
 * @param state: AVB state
 */
esp_err_t avb_config_i2s(avb_state_s *state) {
  /* First plan candidate for the boot rate; later rate changes prefer
   * whatever MCLK is already running (avb_codec_select_mclk). */
  state->codec_mclk_hz = avb_codec_select_mclk(
      state, state->config.default_sample_rate, 0, NULL);
  if (state->codec_mclk_hz == 0) {
    ESP_LOGE(TAG, "No clock plan for %lu Hz on this codec",
             (unsigned long)state->config.default_sample_rate);
    return ESP_ERR_NOT_SUPPORTED;
  }

  // Create an I2S channel and set the handles in the state
  i2s_chan_config_t chan_cfg =
      I2S_CHANNEL_DEFAULT_CONFIG(state->config.i2s_port, I2S_ROLE_MASTER);
  /* Auto-clear stale DMA content so the DAC emits silence on underrun
   * rather than a loop of old samples. */
  chan_cfg.auto_clear = true;
  /* DMA pool must exceed one 1 ms drain period of audio at the highest
   * rate, or i2s_channel_write can never fit a full drain batch and the
   * excess is dropped at the jitter-ring write (measured ~16% sample
   * loss at 192 kHz with the old 16-descriptor / 1 KB pool). Keep the
   * small 6-frame descriptors — empirically, raising dma_frame_num to
   * 24 (192 B buffers) silenced the DAC entirely at 192 kHz (cause not
   * yet understood; 64 B buffers are proven) — and scale the descriptor
   * COUNT instead: 64 × 64 B = 4 KB ≈ 2.7 ms at 192 kHz stereo
   * 24-in-32. The listener drain in avtp.c runs as an esp_timer 1 ms
   * task that calls i2s_channel_write; the I2S driver paces playout at
   * the DAC rate. */
  chan_cfg.dma_frame_num = 6;
  chan_cfg.dma_desc_num = 64;
  ESP_ERROR_CHECK(
      i2s_new_channel(&chan_cfg, &state->i2s_tx_handle, &state->i2s_rx_handle));
  i2s_std_config_t std_cfg = {
      .clk_cfg = I2S_STD_CLK_DEFAULT_CONFIG(state->config.default_sample_rate),
      .slot_cfg = I2S_STD_PHILIPS_SLOT_DEFAULT_CONFIG(
          state->config.default_bits_per_sample, I2S_SLOT_MODE_STEREO),
      .gpio_cfg =
          {
              .mclk = state->config.codec_pins.mclk,
              .bclk = state->config.codec_pins.bclk,
              .ws = state->config.codec_pins.ws,
              .dout = state->config.codec_pins.dout,
              .din = state->config.codec_pins.din,
              .invert_flags =
                  {
                      .mclk_inv = false,
                      .bclk_inv = false,
                      .ws_inv = false,
                  },
          },
  };
  std_cfg.clk_cfg.mclk_multiple =
      state->codec_mclk_hz / state->config.default_sample_rate;
  /* Use APLL as the clock source so the Milan media-clock PLL
   * (avb_mclk / avb_mclk_apll) can retune MCLK with sub-ppm precision
   * without having to disable/reconfigure the I2S channel.
   *
   * On SOCs without an APLL (e.g. esp32c6) fall back to XTAL — the
   * Milan PLL's hardware-tune path is a no-op there (see avbpll.c's
   * SOC_CLK_APLL_SUPPORTED gate). A software-only clock-recovery
   * alternative is needed on those targets. */
#if SOC_CLK_APLL_SUPPORTED
  std_cfg.clk_cfg.clk_src = I2S_CLK_SRC_APLL;
#else
  std_cfg.clk_cfg.clk_src = I2S_CLK_SRC_XTAL;
#endif
  /* Big-endian in-memory sample layout: byte[0]=MSB, byte[2]=LSB. This
   * matches AVTP wire order so the stream-in handler can memcpy AAF
   * payloads straight to the jitter ring with no per-sample shuffle. */
  std_cfg.slot_cfg.big_endian = true;
  /* 24-bit PCM carried MSB-justified in 32-bit slots: [MSB MID LSB 00]
   * in memory (big_endian). 32-bit slots lift the IDF requirement that
   * mclk_multiple be a multiple of 3, which is what allows the 128/256
   * ratios the ES8389 needs at 96/192 kHz. */
  std_cfg.slot_cfg.data_bit_width = I2S_DATA_BIT_WIDTH_32BIT;
  std_cfg.slot_cfg.slot_bit_width = I2S_SLOT_BIT_WIDTH_32BIT;

  /* Initialize the I2S TX and RX channels. The channel initialised
   * second becomes the full-duplex internal slave (RX here). The
   * driver warns below MCLK/BCLK 4 for an RX slave (ratio 2 at
   * 176.4/192 kHz with the 128 fs the ES8389 needs); wire captures
   * with RX as master showed the identical corruption, so that ratio
   * is not what breaks 192 kHz capture (see the DMA-queue note in
   * avtp.c and the clock-row programming below). */
  ESP_ERROR_CHECK(i2s_channel_init_std_mode(state->i2s_tx_handle, &std_cfg));
  ESP_ERROR_CHECK(i2s_channel_init_std_mode(state->i2s_rx_handle, &std_cfg));

  /* Media-clock sensor: count bytes the DAC actually consumed via the
   * TX DMA on_sent callback. This ticks at the true local playout rate
   * regardless of jitter-buffer fill — with auto_clear the DMA keeps
   * consuming (zeros) through underruns — so the avb_pll loop can
   * measure the local media clock at any listener latency, including
   * strict presentation-time playout where the standing fill is only
   * the talker's transit margin. (Counting bytes written INTO the DMA,
   * as before, tracked the talker's arrival rate whenever the DMA was
   * not backpressured, blinding the loop at thin fill.) Must be
   * registered before i2s_channel_enable. */
  i2s_event_callbacks_t tx_cbs = {.on_sent = i2s_tx_on_sent_cb};
  ESP_ERROR_CHECK(i2s_channel_register_event_callback(state->i2s_tx_handle,
                                                      &tx_cbs, state));
  i2s_event_callbacks_t rx_cbs = {.on_recv = i2s_rx_on_recv_cb};
  ESP_ERROR_CHECK(i2s_channel_register_event_callback(state->i2s_rx_handle,
                                                      &rx_cbs, state));

  // Enable the I2S TX and RX channels
  ESP_ERROR_CHECK(i2s_channel_enable(state->i2s_tx_handle));
  ESP_ERROR_CHECK(i2s_channel_enable(state->i2s_rx_handle));

  /* Publish the effective listener-side rates so the PLL and the
   * stream-input drain no longer hardcode 48 kHz / 288000 B/s. The
   * drain always outputs stereo 24-bit (2 ch × 3 B/frame); only the
   * sample rate changes with config. */
  state->media_clock.listener_sample_rate = state->config.default_sample_rate;
  state->media_clock.listener_byterate =
      state->config.default_sample_rate * 2u * 4u; /* 24-in-32 slots */

  /* Initialise the media-clock PLL now that I2S (and hence APLL) is up */
  if (avb_pll_init(state->codec_mclk_hz) != 0) {
    avbwarn("PLL init failed (sample clock will free-run)");
  }

  avbinfo("I2S channels initialized");
  return ESP_OK;
}

/* Switch the audio hardware to a new sample rate. Codecs cannot
 * retune live — they need a stop/start (or reset) around a rate
 * change — so this must only run while NO stream is active in either
 * direction (the AECP layer enforces that with STREAM_IS_RUNNING).
 * Sequence: codec off → I2S channels disabled → I2S clock tree
 * reconfigured (rate + per-rate MCLK multiple) → codec re-programmed
 * at the new fs → everything back on → media-clock/PLL re-seeded. */
esp_err_t avb_audio_set_rate(avb_state_s *state, uint32_t rate) {
  if (rate == state->config.default_sample_rate)
    return ESP_OK;
  const avb_codec_caps_s *caps = avb_codec_get_caps(state->config.codec_type);
  const void *codec_clock_data = NULL;
  uint32_t mclk_hz =
      caps ? avb_codec_select_mclk(state, rate, state->codec_mclk_hz,
                                   &codec_clock_data)
           : 0;
  if (mclk_hz == 0) {
    ESP_LOGE(TAG, "Rate change to %lu Hz: no usable clock plan on this codec",
             (unsigned long)rate);
    return ESP_ERR_NOT_SUPPORTED;
  }
  if (!state->codec_enabled || !state->codec_if || !state->i2s_tx_handle ||
      !state->i2s_rx_handle) {
    return ESP_ERR_INVALID_STATE;
  }

  const audio_codec_if_t *cif = (const audio_codec_if_t *)state->codec_if;
  if (cif->enable)
    cif->enable(cif, false);
  ESP_RETURN_ON_ERROR(i2s_channel_disable(state->i2s_tx_handle), TAG,
                      "i2s tx disable");
  ESP_RETURN_ON_ERROR(i2s_channel_disable(state->i2s_rx_handle), TAG,
                      "i2s rx disable");

  uint32_t mclk_multiple = mclk_hz / rate;
  bool same_mclk = mclk_hz == state->codec_mclk_hz;
  i2s_std_clk_config_t clk_cfg = I2S_STD_CLK_DEFAULT_CONFIG(rate);
  clk_cfg.mclk_multiple = mclk_multiple;
#if SOC_CLK_APLL_SUPPORTED
  /* Both channels hold the APLL, and the driver's refcount gate refuses
   * to retune it while more than one owner remains — reconfiguring one
   * channel at a time can never move the APLL to the new rate. Park
   * both channels on XTAL at a low, always-derivable MCLK first so the
   * APLL is fully released, then re-acquire it at the new rate.
   * When the plan keeps the running MCLK (24.576 MHz across 48/96/192
   * kHz, 11.2896 MHz across 44.1/88.2 kHz on the ES8389) the driver is
   * asked for the APLL frequency it already records, so it leaves the
   * coefficients (and the servo's trim) alone; only the BCLK divider
   * and the codec row move, and the park is skipped. */
  if (!same_mclk) {
    i2s_std_clk_config_t park_cfg = I2S_STD_CLK_DEFAULT_CONFIG(48000);
    park_cfg.mclk_multiple = I2S_MCLK_MULTIPLE_256;
    park_cfg.clk_src = I2S_CLK_SRC_XTAL;
    ESP_RETURN_ON_ERROR(
        i2s_channel_reconfig_std_clock(state->i2s_tx_handle, &park_cfg), TAG,
        "i2s tx park");
    ESP_RETURN_ON_ERROR(
        i2s_channel_reconfig_std_clock(state->i2s_rx_handle, &park_cfg), TAG,
        "i2s rx park");
  }
  clk_cfg.clk_src = I2S_CLK_SRC_APLL;
#else
  clk_cfg.clk_src = I2S_CLK_SRC_XTAL;
#endif
  ESP_RETURN_ON_ERROR(
      i2s_channel_reconfig_std_clock(state->i2s_tx_handle, &clk_cfg), TAG,
      "i2s tx reclock");
  ESP_RETURN_ON_ERROR(
      i2s_channel_reconfig_std_clock(state->i2s_rx_handle, &clk_cfg), TAG,
      "i2s rx reclock");

  esp_codec_dev_sample_info_t fs = {
      .bits_per_sample = 32,
      .channel = 2,
      .sample_rate = rate,
      .mclk_multiple = mclk_multiple,
  };
  if (cif->set_fs && cif->set_fs(cif, &fs) != 0) {
    ESP_LOGE(TAG, "Rate change: codec set_fs failed");
    return ESP_FAIL;
  }
  if (caps->apply_clock &&
      caps->apply_clock(state, rate, mclk_hz, codec_clock_data) != ESP_OK) {
    ESP_LOGE(TAG, "Rate change: codec clock programming failed");
    return ESP_FAIL;
  }
  if (cif->enable && cif->enable(cif, true) != 0) {
    ESP_LOGE(TAG, "Rate change: codec re-enable failed");
    return ESP_FAIL;
  }

  ESP_RETURN_ON_ERROR(i2s_channel_enable(state->i2s_tx_handle), TAG,
                      "i2s tx enable");
  ESP_RETURN_ON_ERROR(i2s_channel_enable(state->i2s_rx_handle), TAG,
                      "i2s rx enable");

  /* Commit the new rate only now that every hardware step has succeeded.
   * A failure above leaves default_sample_rate untouched, so a retry
   * re-runs the full path instead of short-circuiting on the rate
   * compare at the top and reporting success with stale hardware. */
  state->config.default_sample_rate = rate;
  state->codec_mclk_hz = mclk_hz;

  /* Mirror avb_config_i2s's media-clock bookkeeping and re-seed the
   * PLL at the new nominal MCLK. */
  state->media_clock.listener_sample_rate = rate;
  state->media_clock.listener_byterate = rate * 2u * 4u; /* 24-in-32 slots */
  if (same_mclk) {
    /* APLL untouched, so the backend and the trim stand; only the
     * measurement windows must restart, the byte rate just changed. */
    avb_pll_reseed();
    avbinfo("Audio hardware reconfigured to %lu Hz (MCLK unchanged, APLL kept)",
            (unsigned long)rate);
    return ESP_OK;
  }
  if (avb_pll_init(mclk_hz) != 0) {
    avbwarn("Rate change: PLL re-init failed (sample clock will free-run)");
  } else {
    /* The re-init put the APLL back at nominal; the servo still holds
     * the converged trim, so put it back on the hardware too. */
    avb_pll_restore_trim(state);
  }

  avbinfo("Audio hardware reconfigured to %lu Hz", (unsigned long)rate);
  return ESP_OK;
}

/* Per-codec factory result: the chip-specific create step yields these. */
typedef struct {
  const audio_codec_if_t *codec_if;
  const audio_codec_ctrl_if_t *ctrl_if; /* for writes the driver does not do */
} codec_factory_result_s;


static int es8389_reg_write(const audio_codec_ctrl_if_t *ctrl, uint8_t reg,
                            uint8_t value) {
  int data = value;
  return ctrl->write_reg(ctrl, reg, 1, &data, 1);
}

static int es8389_reg_update(const audio_codec_ctrl_if_t *ctrl, uint8_t reg,
                             uint8_t mask, uint8_t value) {
  int data = 0;
  int err = ctrl->read_reg(ctrl, reg, 1, &data, 1);
  if (err != 0)
    return err;
  data = (data & ~mask) | (value & mask);
  return ctrl->write_reg(ctrl, reg, 1, &data, 1);
}

/* avb_codec_caps_s::apply_clock for the ES8389: write the plan's row
 * for (rate_hz, mclk_hz), mirroring es8389_config_sample's register
 * sequence. Call with the codec disabled (before enable), after set_fs. */
static esp_err_t es8389_apply_clock(avb_state_s *state, uint32_t rate_hz,
                                    uint32_t mclk_hz, const void *codec_data) {
  const audio_codec_ctrl_if_t *ctrl =
      (const audio_codec_ctrl_if_t *)state->codec_ctrl_if;
  const es8389_clock_row_s *row = (const es8389_clock_row_s *)codec_data;
  if (!ctrl)
    return ESP_ERR_INVALID_STATE;
  if (!row || row->rate_hz != rate_hz || row->mclk_hz != mclk_hz) {
    ESP_LOGE(TAG, "ES8389: no clock row for %lu Hz at %lu Hz MCLK",
             (unsigned long)rate_hz, (unsigned long)mclk_hz);
    return ESP_ERR_NOT_FOUND;
  }
  const uint8_t *r = row->reg;
  int err = 0;
  for (uint8_t reg = 0x04; reg <= 0x0A; reg++)
    err |= es8389_reg_write(ctrl, reg, r[reg - 0x04]);
  err |= es8389_reg_update(ctrl, 0x0F, 0xC0, r[7]);
  err |= es8389_reg_write(ctrl, 0x11, r[8]);
  err |= es8389_reg_write(ctrl, 0x21, r[9]);
  err |= es8389_reg_write(ctrl, 0x22, r[10]);
  err |= es8389_reg_write(ctrl, 0x26, r[11]);
  err |= es8389_reg_update(ctrl, 0x30, 0xC0, r[12]);
  err |= es8389_reg_write(ctrl, 0x41, r[13]);
  err |= es8389_reg_write(ctrl, 0x42, r[14]);
  err |= es8389_reg_update(ctrl, 0x43, 0x81, r[15]);
  err |= es8389_reg_update(ctrl, 0xF0, 0x73, r[16]);
  err |= es8389_reg_write(ctrl, 0xF1, r[17]);
  err |= es8389_reg_write(ctrl, 0x16, r[18]);
  err |= es8389_reg_write(ctrl, 0x18, r[19]);
  err |= es8389_reg_write(ctrl, 0x19, r[20]);
  if (err != 0) {
    ESP_LOGE(TAG, "ES8389: clock row write failed (%d)", err);
    return ESP_FAIL;
  }
  ESP_LOGI(TAG, "ES8389: clock row applied for %lu Hz at %lu Hz MCLK",
           (unsigned long)rate_hz, (unsigned long)mclk_hz);
  return ESP_OK;
}

static esp_err_t codec_factory_es8311(avb_state_s *state,
                                      i2c_master_bus_handle_t bus,
                                      const audio_codec_gpio_if_t *gpio_if,
                                      codec_factory_result_s *out) {
  const avb_codec_caps_s *caps = avb_codec_get_caps(avb_codec_type_es8311);
  if (!codec_caps_support_sample_rate(caps, state->config.default_sample_rate)) {
    ESP_LOGE(TAG, "ES8311: unsupported sample rate %lu",
             state->config.default_sample_rate);
    return ESP_FAIL;
  }
  audio_codec_i2c_cfg_t i2c_cfg = {.addr = ES8311_CODEC_DEFAULT_ADDR,
                                   .bus_handle = bus};
  const audio_codec_ctrl_if_t *ctrl = audio_codec_new_i2c_ctrl(&i2c_cfg);
  if (!ctrl) {
    ESP_LOGE(TAG, "ES8311: failed to create I2C control interface");
    return ESP_FAIL;
  }
  es8311_codec_cfg_t cfg = {
      .codec_mode = ESP_CODEC_DEV_WORK_MODE_BOTH,
      .ctrl_if = ctrl,
      .gpio_if = gpio_if,
      .pa_pin = state->config.codec_pins.pa,
      .pa_reverted = state->config.codec_pins.pa_reverted,
      .use_mclk = true,
      .mclk_div = state->codec_mclk_hz / state->config.default_sample_rate,
  };
  out->codec_if = es8311_codec_new(&cfg);
  if (!out->codec_if) {
    ESP_LOGE(TAG, "ES8311: failed to create codec interface");
    return ESP_FAIL;
  }
  return ESP_OK;
}

static esp_err_t codec_factory_es8388(avb_state_s *state,
                                      i2c_master_bus_handle_t bus,
                                      const audio_codec_gpio_if_t *gpio_if,
                                      codec_factory_result_s *out) {
  const avb_codec_caps_s *caps = avb_codec_get_caps(avb_codec_type_es8388);
  if (!codec_caps_support_sample_rate(caps, state->config.default_sample_rate)) {
    ESP_LOGE(TAG, "ES8388: unsupported sample rate %lu",
             state->config.default_sample_rate);
    return ESP_FAIL;
  }
  audio_codec_i2c_cfg_t i2c_cfg = {.addr = ES8388_CODEC_DEFAULT_ADDR,
                                   .bus_handle = bus};
  const audio_codec_ctrl_if_t *ctrl = audio_codec_new_i2c_ctrl(&i2c_cfg);
  if (!ctrl) {
    ESP_LOGE(TAG, "ES8388: failed to create I2C control interface");
    return ESP_FAIL;
  }
  es8388_codec_cfg_t cfg = {
      .codec_mode = ESP_CODEC_DEV_WORK_MODE_BOTH,
      .ctrl_if = ctrl,
      .gpio_if = gpio_if,
      .pa_pin = state->config.codec_pins.pa,
      .pa_reverted = state->config.codec_pins.pa_reverted,
      .master_mode = false,
  };
  out->codec_if = es8388_codec_new(&cfg);
  if (!out->codec_if) {
    ESP_LOGE(TAG, "ES8388: failed to create codec interface");
    return ESP_FAIL;
  }
  return ESP_OK;
}

static esp_err_t codec_factory_es8389(avb_state_s *state,
                                      i2c_master_bus_handle_t bus,
                                      const audio_codec_gpio_if_t *gpio_if,
                                      codec_factory_result_s *out) {
  const avb_codec_caps_s *caps = avb_codec_get_caps(avb_codec_type_es8389);
  if (!codec_caps_support_sample_rate(caps, state->config.default_sample_rate)) {
    ESP_LOGE(TAG, "ES8389: unsupported sample rate %lu",
             state->config.default_sample_rate);
    return ESP_FAIL;
  }
  audio_codec_i2c_cfg_t i2c_cfg = {.addr = ES8389_CODEC_DEFAULT_ADDR,
                                   .bus_handle = bus};
  const audio_codec_ctrl_if_t *ctrl = audio_codec_new_i2c_ctrl(&i2c_cfg);
  if (!ctrl) {
    ESP_LOGE(TAG, "ES8389: failed to create I2C control interface");
    return ESP_FAIL;
  }
  /* The hat feeds an external MCLK from the P4 (GPIO16), so use_mclk=true.
   * That makes the driver run off the provided MCLK in slave mode and skip
   * its internal MCLK=fs*bits*4 coefficient path (es8389.c set_fs), which
   * assumes a different ratio than the plan's MCLK. mclk_div carries the
   * actual MCLK/LRCK ratio; the clock row itself is applied through
   * apply_clock. */
  es8389_codec_cfg_t cfg = {
      .ctrl_if = ctrl,
      .gpio_if = gpio_if,
      .codec_mode = ESP_CODEC_DEV_WORK_MODE_BOTH,
      .pa_pin = state->config.codec_pins.pa,
      .pa_reverted = state->config.codec_pins.pa_reverted,
      .master_mode = false,
      .use_mclk = true,
      .mclk_div = state->codec_mclk_hz / state->config.default_sample_rate,
  };
  out->codec_if = es8389_codec_new(&cfg);
  if (!out->codec_if) {
    ESP_LOGE(TAG, "ES8389: failed to create codec interface");
    return ESP_FAIL;
  }
  out->ctrl_if = ctrl;
  return ESP_OK;
}

/* Configure the codec selected by state->config.codec_type.
 *
 * Generic shell handles I2C bus + GPIO + sample-format/enable + AECP control
 * range loading. Per-codec factory builds the chip-specific cfg struct and
 * calls the matching *_codec_new(). Both codecs are then driven through the
 * codec-agnostic audio_codec_if_t vtable.
 *
 * Bypasses esp_codec_dev_open() because that reconfigures I2S (disable/
 * re-enable) and can misalign BCLK phase — corrupting the lower bits of
 * 24-bit captures. Talker/listener use i2s_channel_read/write directly,
 * so no audio_codec_data_if is needed here.
 */
esp_err_t avb_config_codec(avb_state_s *state) {
  const avb_codec_caps_s *caps = avb_codec_get_caps(state->config.codec_type);
  if (!caps) {
    ESP_LOGE(TAG, "Unsupported codec type: %d", state->config.codec_type);
    return ESP_FAIL;
  }
  if (state->config.input_channels_usable > caps->max_input_channels ||
      state->config.output_channels_usable > caps->max_output_channels) {
    ESP_LOGE(TAG, "Unsupported channel count: %d in, %d out (caps %u/%u)",
             state->config.input_channels_usable,
             state->config.output_channels_usable, caps->max_input_channels,
             caps->max_output_channels);
    return ESP_FAIL;
  }
  if (state->supported_bits_per_sample.num_rates == 0) {
    ESP_LOGE(TAG, "No effective codec bit-depth capability");
    return ESP_FAIL;
  }

  i2c_master_bus_handle_t bus;
  i2c_master_bus_config_t bus_cfg = {
      .i2c_port = I2C_NUM,
      .sda_io_num = state->config.codec_pins.i2c_sda,
      .scl_io_num = state->config.codec_pins.i2c_scl,
      .clk_source = I2C_CLK_SRC_DEFAULT,
      .glitch_ignore_cnt = 7,
      .flags.enable_internal_pullup = true,
  };
  ESP_RETURN_ON_ERROR(i2c_new_master_bus(&bus_cfg, &bus), TAG,
                      "create I2C master bus failed");

  const audio_codec_gpio_if_t *gpio_if = audio_codec_new_gpio();

  codec_factory_result_s result = {0};
  esp_err_t err;
  switch (state->config.codec_type) {
  case avb_codec_type_es8311:
    err = codec_factory_es8311(state, bus, gpio_if, &result);
    break;
  case avb_codec_type_es8388:
    err = codec_factory_es8388(state, bus, gpio_if, &result);
    break;
  case avb_codec_type_es8389:
    err = codec_factory_es8389(state, bus, gpio_if, &result);
    break;
  default:
    ESP_LOGE(TAG, "Unsupported codec type: %d", state->config.codec_type);
    return ESP_FAIL;
  }
  if (err != ESP_OK) {
    return err;
  }

  esp_codec_dev_sample_info_t fs = {
      /* Match the 32-bit I2S slots (PCM is 24-in-32, MSB-justified). */
      .bits_per_sample = 32,
      .channel = 2,
      .sample_rate = state->config.default_sample_rate,
      .mclk_multiple = state->codec_mclk_hz / state->config.default_sample_rate,
  };
  if (result.codec_if->set_fs &&
      result.codec_if->set_fs(result.codec_if, &fs) != 0) {
    ESP_LOGE(TAG, "Failed to set codec sample format");
    return ESP_FAIL;
  }
  state->codec_ctrl_if = result.ctrl_if;
  if (caps->apply_clock) {
    const void *codec_clock_data = NULL;
    uint32_t mclk_hz =
        avb_codec_select_mclk(state, state->config.default_sample_rate,
                              state->codec_mclk_hz, &codec_clock_data);
    if (mclk_hz != state->codec_mclk_hz ||
        caps->apply_clock(state, state->config.default_sample_rate, mclk_hz,
                          codec_clock_data) != ESP_OK) {
      ESP_LOGE(TAG, "Failed to program codec clocks");
      return ESP_FAIL;
    }
  }
  if (result.codec_if->enable &&
      result.codec_if->enable(result.codec_if, true) != 0) {
    ESP_LOGE(TAG, "Failed to enable codec");
    return ESP_FAIL;
  }
  state->codec_enabled = true;
  state->codec_if = result.codec_if;

  state->codec_ranges = caps->control_ranges;
  state->codec_ranges.vol_default_tenth_db = avb_codec_quantize_tenth_db(
      &state->codec_ranges, false, state->config.default_speaker_vol_tenth_db);
  state->codec_ranges.gain_default_tenth_db = avb_codec_quantize_tenth_db(
      &state->codec_ranges, true, state->config.default_mic_gain_tenth_db);
  state->ctrl_speaker_vol = state->codec_ranges.vol_default_tenth_db / 10.0f;
  state->ctrl_mic_gain = state->codec_ranges.gain_default_tenth_db / 10.0f;

  if (result.codec_if->set_vol) {
    result.codec_if->set_vol(result.codec_if, state->ctrl_speaker_vol);
  }
  if (result.codec_if->set_mic_gain) {
    result.codec_if->set_mic_gain(result.codec_if, state->ctrl_mic_gain);
  }

  ESP_LOGI(TAG, "Codec configured and enabled (ADC+DAC active)");
  return ESP_OK;
}

/* Set speaker volume via codec interface */
void avb_codec_set_vol(avb_state_s *state, float db) {
  const audio_codec_if_t *codec = (const audio_codec_if_t *)state->codec_if;
  if (codec && codec->set_vol) {
    codec->set_vol(codec, db);
  }
}

/* Set mic gain via codec interface */
void avb_codec_set_mic_gain(avb_state_s *state, float db) {
  const audio_codec_if_t *codec = (const audio_codec_if_t *)state->codec_if;
  if (codec && codec->set_mic_gain) {
    codec->set_mic_gain(codec, db);
  }
}
