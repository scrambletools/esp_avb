/*
 * Copyright 2024-2026 Scramble Tools
 * License: MIT
 *
 * ESP_AVB Component
 *
 * This component provides an implementation of an AVB talker and listener.
 *
 * This file provides the required features of the AVTP protocol including
 * support for MSRP.
 */

#include "avb.h"
#include "avbsrc.h"
#include "driver/gptimer.h"
#include "driver/i2s_std.h"
#include "esp_codec_dev.h"
#include "esp_cpu.h"       /* drain cycle profile */
#include "esp_heap_caps.h" /* largest free block when the jitter ring fails */
#include "esp_log.h"
#include "esp_task_wdt.h"
#include "esp_timer.h"
#include "freertos/portmacro.h"
#include <stdatomic.h>

/* Cumulative "productive" microseconds spent inside avb_stream_out_task
 * per iteration — excludes the busy-wait spin on esp_timer_get_time().
 * Used by the CPU stats diagnostic to show how much CPU AVB-OUT would
 * consume if its busy-wait were replaced with a yielding wait. */
static _Atomic uint64_t s_stream_out_work_us = 0;

uint64_t avb_stream_out_work_us_total(void) {
  return atomic_load_explicit(&s_stream_out_work_us, memory_order_relaxed);
}

/* MSRP class-mapping helpers moved to msrp.c (used by both files
 * as static-but-shared via the msrp.c definition; avtp.c reaches
 * them by inlining the trivial expression where needed). */

static uint8_t avb_msrp_priority_for_stream(avb_state_s *state,
                                            uint16_t stream_index) {
  /* SR class A → mapping index 0, SR class B → mapping index 1. */
  uint16_t mapping_index =
      state->output_streams[stream_index].stream_info_flags.class_b ? 1 : 0;
  if (state->avb_lite)
    return AVB_LITE_PCP;
  return state->msrp_mappings[mapping_index].priority;
}

/* Monotonic stream-out counters that outlive the TX context, which is
 * freed on every stop (Milan counters must not reset with the stream). */
static volatile uint32_t s_tx_frames_total;
static volatile uint32_t s_tx_starts;
static volatile uint32_t s_tx_stops;

/* Stream-out diagnostic context — mirrors stream_rx_ctx_t pattern.
 * Writer is avb_stream_out_task (core 1 TX task), reader is AVB-STATS
 * (core 0). Plain volatile uint32_t reads/writes are torn-free on
 * RISC-V at 4-byte alignment; int64 overrun_max is only sampled when
 * the task is paused in its busy-wait so torn reads aren't a concern
 * for that one. The context is allocated at task start and freed on
 * stop — same lifecycle as stream_rx_ctx_t. */
typedef struct {
  volatile bool active;              /* true while stream_out_task running */
  uint32_t start_serial;             /* increments per task start (stats re-baseline) */
  volatile uint32_t pkt_count;       /* TX packets attempted */
  volatile uint32_t send_fail_count; /* avb_net_transmit_raw failures */
  volatile uint32_t overrun_count;   /* send loop late beyond interval */
  volatile int64_t overrun_max_us;   /* worst overrun this session */
  volatile uint32_t i2s_zero_reads;  /* mic ring read returned 0 bytes */
  volatile uint32_t i2s_nonzero_reads; /* healthy mic reads */
  volatile uint32_t ring_underruns;    /* mic ring empty: zero-filled packet */
  volatile int32_t fill_ppm_q8;        /* cadence trim following the ADC, ppm x 256 */
  volatile int32_t fill_backlog_bytes; /* captured - sent, vs the pre-fill setpoint */
  volatile uint32_t slip_events;       /* slot-slip guard: detections */
  volatile uint32_t slip_muted_pkts;   /* slot-slip guard: packets muted */
  /* TX PLL offset range, cumulative; AVB-STATS resets via print_diag
   * after reading so the reported range is always per-window. */
  volatile int32_t pll_offset_min_ns;
  volatile int32_t pll_offset_max_ns;
  volatile uint32_t pll_skip_count; /* PLL measurements rejected as outliers */
  /* gPTP-paced cadence drift tracking (see avb_stream_out_task) */
  volatile int32_t drift_ppm_q8;    /* (gptp_rate − esp_rate) / esp_rate × 1e6, Q8 */
  volatile int32_t remaining_ns_min; /* narrowest gPTP-vs-target gap this window */
  volatile int32_t remaining_ns_max; /* widest gPTP-vs-target gap this window */
  volatile uint32_t gptp_resync_count; /* sanity-branch re-seeds (gPTP jumps) */
  volatile uint32_t ts_reanchors;      /* timestamp re-anchored to gPTP (tu) */
  volatile uint32_t i2s_dropped_bytes; /* captured audio the RX DMA overwrote */
  volatile int32_t ts_err_ns;          /* stamp error vs gPTP schedule */
  volatile uint32_t wakes;             /* batched pacing: sleeps ended */
  volatile uint32_t stalls;            /* packets sent TALKER_STALL_US late */
  volatile uint32_t src_cycles;        /* CPU cycles per resampled frame */
  volatile uint32_t src_walks;         /* converter walks started */
} stream_tx_ctx_t;

static stream_tx_ctx_t *s_stream_tx_ctx = NULL;

/* Note: Ethernet MAC DMA transmit is accessed concurrently by the AVTP stream
 * task (core 1) and the PTP daemon (core 0). The ESP-IDF driver lacks internal
 * locking. A lock here was attempted but the PTP timestamped TX path holds the
 * DMA for too long (busy-waits for TX completion), causing the AVTP real-time
 * task to miss its 125μs deadlines. The DMA ring appears to handle concurrent
 * access in practice (separate TX descriptors for each caller). */


/* Compute GCD for sine LUT sizing */
static uint32_t gcd(uint32_t a, uint32_t b) {
  while (b) {
    uint32_t t = b;
    b = a % b;
    a = t;
  }
  return a;
}

/* Per-channel test tone spacing: channel n plays freq + n x this. */
#define TONE_CHANNEL_STEP_HZ 250

/* Build a precomputed sine wave lookup table (one exact cycle)
 *
 * Each entry is a big-endian PCM sample, the same on every channel, or
 * with CONFIG_ESP_AVB_TEST_TONE_PER_CHANNEL at freq + 250 Hz x channel
 * so a capture shows channel order.
 * Returns the number of samples in one cycle, or 0 on allocation failure.
 * Caller must free *lut_out when done.
 */
static uint32_t build_sine_lut(uint8_t **lut_out, int channels, int bit_depth,
                               uint32_t sample_rate, float freq) {
  // One exact cycle length = sample_rate / gcd(sample_rate, freq)
  uint32_t freq_int = (uint32_t)freq;
#if CONFIG_ESP_AVB_TEST_TONE_PER_CHANNEL
  /* Every channel's frequency is a multiple of this, so one table
   * period holds a whole number of cycles of each. */
  uint32_t cycle_samples =
      sample_rate / gcd(sample_rate, gcd(freq_int, TONE_CHANNEL_STEP_HZ));
#else
  uint32_t cycle_samples = sample_rate / gcd(sample_rate, freq_int);
#endif

  int stride = (bit_depth == 24) ? 4 : (bit_depth / 8);
  int frame_size = stride * channels;
  uint8_t *lut = calloc(cycle_samples, frame_size);
  if (!lut) {
    *lut_out = NULL;
    return 0;
  }

  /* -20 dBFS nominal (x0.7 below lands ~-23 dBFS): loud enough to
   * register on listener meters, far from clipping. */
  float amplitude = (bit_depth == 24) ? 838860.0f : 3277.0f;
  float phase_inc = 2.0f * M_PI * freq / (float)sample_rate;
  float phase = 0.0f;

  for (uint32_t i = 0; i < cycle_samples; i++) {
    int32_t sample = (int32_t)(sinf(phase) * amplitude * 0.7f); // 70% amplitude

    for (int ch = 0; ch < channels; ch++) {
#if CONFIG_ESP_AVB_TEST_TONE_PER_CHANNEL
      /* Phase from the index, (f x i) mod rate, so it stays exact. */
      uint32_t ch_freq = freq_int + TONE_CHANNEL_STEP_HZ * (uint32_t)ch;
      float ch_phase = 2.0f * (float)M_PI *
                       (float)(((uint64_t)ch_freq * i) % sample_rate) /
                       (float)sample_rate;
      sample = (int32_t)(sinf(ch_phase) * amplitude * 0.7f);
#endif
      int offset = (i * channels + ch) * stride;
      if (bit_depth == 24) {
        // IEEE 1722 AAF §7.3.5: 24-bit sample left-justified in 32-bit
        // big-endian container. AM824 path reads bytes 0-2 as 24-bit sample.
        lut[offset + 0] = (sample >> 16) & 0xFF; // MSB
        lut[offset + 1] = (sample >> 8) & 0xFF;
        lut[offset + 2] = (sample >> 0) & 0xFF; // LSB
        lut[offset + 3] = 0;                    // padding
      } else {
        lut[offset + 0] = (sample >> 8) & 0xFF;
        lut[offset + 1] = (sample >> 0) & 0xFF;
      }
    }
    phase += phase_inc;
    if (phase >= 2.0f * M_PI) {
      phase -= 2.0f * M_PI;
    }
  }

  *lut_out = lut;
  return cycle_samples;
}

/* Copy samples from the precomputed sine LUT into a packet buffer
 *
 * @param buf: output buffer for PCM samples
 * @param num_samples: number of samples per channel to copy
 * @param channels: number of channels
 * @param bit_depth: bits per sample (16 or 24)
 * @param lut: precomputed sine wave LUT (one cycle)
 * @param lut_samples: number of samples in the LUT
 * @param lut_pos: pointer to current LUT position (updated on return, wraps)
 */
static void copy_sine_from_lut(uint8_t *buf, int num_samples, int channels,
                               int bit_depth, const uint8_t *lut,
                               uint32_t lut_samples, uint32_t *lut_pos) {
  int stride = (bit_depth == 24) ? 4 : (bit_depth / 8);
  int frame_size = stride * channels;

  for (int i = 0; i < num_samples; i++) {
    memcpy(buf + i * frame_size, lut + (*lut_pos) * frame_size, frame_size);
    *lut_pos = (*lut_pos + 1) % lut_samples;
  }
}

#if CONFIG_ESP_AVB_TALKER_TEST_PULSE
#define TEST_PULSE_WIDTH_NS 1000000

/* Fill a packet with the test pulse: -6 dBFS for 1 ms from each whole
 * gPTP second of presentation time, silence otherwise, in the sine
 * table's 32-bit BE layout. first_pt_ns is the full presentation time
 * of the packet's first sample. */
static void fill_test_pulse(uint8_t *buf, int num_samples, int channels,
                            int64_t first_pt_ns, uint32_t ns_per_sample) {
  for (int i = 0; i < num_samples; i++) {
    int64_t into_second =
        (first_pt_ns + (int64_t)i * ns_per_sample) % 1000000000LL;
    uint8_t level = into_second < TEST_PULSE_WIDTH_NS ? 0x40 : 0x00;
    for (int ch = 0; ch < channels; ch++) {
      uint8_t *sample = buf + (i * channels + ch) * 4;
      sample[0] = level;
      sample[1] = 0;
      sample[2] = 0;
      sample[3] = 0;
    }
  }
}
#endif

/* Read PCM samples from an embedded file into a buffer
 *
 * Reads one packet's worth of samples from flash-mapped PCM data,
 * wrapping around to the beginning when the end is reached.
 *
 * @param buf: output buffer for PCM samples
 * @param num_samples: number of samples per channel to read
 * @param channels: number of channels
 * @param bit_depth: bits per sample (16 or 24)
 * @param src: pointer to embedded PCM file data (flash-mapped)
 * @param src_len: total length of embedded PCM data in bytes
 * @param offset: pointer to current read offset (updated on return, wraps)
 */
static void read_pcm_file(uint8_t *buf, int num_samples, int channels,
                          int bit_depth, const uint8_t *src, uint32_t src_len,
                          uint32_t *offset) {
  int bytes_per_sample = (bit_depth == 24) ? 4 : (bit_depth / 8);
  int frame_bytes = num_samples * channels * bytes_per_sample;
  int pos = 0;

  while (pos < frame_bytes) {
    int remaining = frame_bytes - pos;
    int available = src_len - *offset;
    int to_copy = (remaining < available) ? remaining : available;
    memcpy(buf + pos, src + *offset, to_copy);
    pos += to_copy;
    *offset += to_copy;
    if (*offset >= src_len) {
      *offset = 0;
    }
  }
}

/* Build and send an AAF PCM AVTP packet
 *
 * @param state: AVB state
 * @param stream_id: stream ID for the packet
 * @param pcm_data: raw PCM audio data
 * @param data_len: length of pcm_data in bytes
 * @param seq_num: sequence number (incremented by caller)
 * @param sample_rate: AAF sample rate enum value
 * @param channels: channels per frame
 * @param bit_depth: bit depth
 * @param dest_addr: destination MAC address
 * @param vlan_id: VLAN ID
 * @param avtp_ts: AVTP presentation timestamp (lower 32 bits of PTP ns count)
 */
static int avb_send_aaf_pcm_packet(avb_state_s *state, unique_id_t *stream_id,
                                   uint8_t *pcm_data, uint16_t data_len,
                                   uint8_t seq_num, uint8_t sample_rate_code,
                                   uint8_t channels, uint8_t bit_depth,
                                   eth_addr_t *dest_addr, uint8_t *vlan_id,
                                   uint32_t avtp_ts,
                                   uint32_t presentation_time_offset_ns) {
  aaf_pcm_message_s msg;
  memset(&msg, 0, sizeof(msg));

  // Populate the AAF header
  msg.subtype = avtp_subtype_aaf;
  msg.sv = 1; // stream ID valid
  msg.version = 0;
  msg.timestamp_valid = 0;
  // IEEE 1722 §6.4: MCR=1 only on true media clock restart (stream startup),
  // not on every seq_num rollover. One-shot: set on first packet, then clear.
  static bool mcr_pending = true;
  msg.media_clock_restart = mcr_pending ? 1 : 0;
  if (mcr_pending)
    mcr_pending = false;
  msg.seq_num = seq_num;
  msg.timestamp_uncertain = 0;
  memcpy(msg.stream_id, stream_id, UNIQUE_ID_LEN);

  // Set AVTP presentation timestamp (lower 32 bits of PTP nanosecond count).
  // Add the stream's presentation-time offset so the listener can buffer until
  // the presentation time to compensate for network jitter.
  uint32_t presentation_ts = avtp_ts + presentation_time_offset_ns;
  msg.avtp_ts[0] = (presentation_ts >> 24) & 0xFF;
  msg.avtp_ts[1] = (presentation_ts >> 16) & 0xFF;
  msg.avtp_ts[2] = (presentation_ts >> 8) & 0xFF;
  msg.avtp_ts[3] = presentation_ts & 0xFF;
  msg.timestamp_valid = 1;

  msg.format = (bit_depth <= 16) ? aaf_format_int_16bit : aaf_format_int_32bit;
  msg.sample_rate = sample_rate_code;
  msg.chan_per_frame = channels;
  msg.bit_depth = bit_depth;
  msg.evt = 0; // normal
  msg.sparse_ts = 0;

  // Set stream data length
  uint16_t sdl = (data_len > AVTP_STREAM_DATA_PER_MSG)
                     ? AVTP_STREAM_DATA_PER_MSG
                     : data_len;
  msg.stream_data_len[0] = (sdl >> 8) & 0xFF;
  msg.stream_data_len[1] = sdl & 0xFF;

  // Copy PCM data into the message
  memcpy(msg.stream_data, pcm_data, sdl);

  // Calculate total message length (header + stream data)
  // AAF header is 24 bytes before stream_data
  uint16_t msg_len = sizeof(aaf_pcm_message_s) - AVTP_STREAM_DATA_PER_MSG + sdl;

  // Send to the stream destination address using the non-timestamped path.
  // The timestamped path (L2TAP_IREC_TIME_STAMP) busy-waits for DMA completion
  // while holding the EMAC transmit mutex, which blocks this real-time task
  // when PTP also transmits with timestamps.
  uint8_t eth_frame[msg_len + ETH_HEADER_LEN + sizeof(struct eth_vlan_hdr)];
  avb_create_eth_frame(eth_frame, dest_addr, state, ethertype_vlan, &msg,
                       msg_len, vlan_id);
  int ret = write(state->port[0].l2if[VLAN], eth_frame, sizeof(eth_frame));
  if (ret < 0) {
    avberr("send AAF PCM failed: %d", errno);
  }
  return ret;
}

/* Build and send an IEC 61883-6 AM824 AVTP packet
 *
 * AM824 wraps 24-bit PCM samples in IEC 61883-6 data blocks with a CIP header.
 * Each data block is 1 quadlet (4 bytes) per channel: label byte + 24-bit
 * sample. The CIP header is 2 quadlets (8 bytes) prepended to the data blocks.
 */
static int avb_send_iec_61883_packet(avb_state_s *state, unique_id_t *stream_id,
                                     uint8_t *pcm_data, uint16_t data_len,
                                     uint8_t seq_num, uint8_t dbs, uint8_t sfc,
                                     uint8_t channels, eth_addr_t *dest_addr,
                                     uint8_t *vlan_id, uint32_t avtp_ts,
                                     uint16_t dbc,
                                     uint32_t presentation_time_offset_ns) {
  iec_61883_6_message_s msg;
  memset(&msg, 0, sizeof(msg));

  // Populate the IEC 61883 AVTP header
  msg.subtype = avtp_subtype_61883;
  msg.sv = 1;
  msg.version = 0;
  msg.timestamp_valid = 0;
  static bool mcr_pending_61883 = true;
  msg.media_clock_restart = mcr_pending_61883 ? 1 : 0;
  if (mcr_pending_61883)
    mcr_pending_61883 = false;
  msg.seq_num = seq_num;
  msg.timestamp_uncertain = 0;
  memcpy(msg.stream_id, stream_id, UNIQUE_ID_LEN);

  // AVTP presentation timestamp with the stream's presentation-time offset.
  uint32_t presentation_ts = avtp_ts + presentation_time_offset_ns;
  msg.avtp_ts[0] = (presentation_ts >> 24) & 0xFF;
  msg.avtp_ts[1] = (presentation_ts >> 16) & 0xFF;
  msg.avtp_ts[2] = (presentation_ts >> 8) & 0xFF;
  msg.avtp_ts[3] = presentation_ts & 0xFF;
  msg.timestamp_valid = 1;

  // IEEE 1394 fields
  msg.tag = 1;      // CIP header present
  msg.channel = 31; // 31 = native AVTP (not IEEE 1394)
  msg.tcode = 0x0A; // must be 1010 on transmit
  msg.sy = 0;

  // Build CIP header (2 quadlets = 8 bytes) per IEC 61883-1 §2.3
  //
  // Quadlet 0: | EOH(1)=0 | Fmt_hi(1)=0 | SID(6)=0x3F | DBS(8) | FN(2)=0 |
  // QPC(3)=0 | SPH(1)=0 | rsv(1)=0 | DBC(8) | Quadlet 1: | EOH(1)=1 |
  // Fmt_hi(1)=0 | FMT(6)=0x10 | FDF(8)          | SYT(16)=0xFFFF          |
  //
  // SID=0x3F: no source ID info (standard for AVTP)
  // FMT=0x10: AM824 format (IEC 61883-6)
  // FDF: evt(3)=000 (AM824) | sfc(3) | reserved(2)=0
  msg.stream_data[0] = 0x3F; // EOH=0, Fmt_hi=0, SID=111111 (0x3F = no info)
  msg.stream_data[1] = dbs;  // data block size (quadlets per data block)
  msg.stream_data[2] = 0x00; // FN=00, QPC=000, SPH=0, rsv=0, DBC[7]=0
  msg.stream_data[3] = (uint8_t)(dbc & 0xFF); // DBC lower 8 bits
  msg.stream_data[4] = 0x90; // EOH=1, Fmt_hi=0, FMT=010000 (0x10 = AM824)
  msg.stream_data[5] =
      (sfc & 0x07);          // FDF: evt=00000 in bits[7:3], SFC in bits[2:0]
  msg.stream_data[6] = 0xFF; // SYT = 0xFFFF (no SYT info — use AVTP timestamp)
  msg.stream_data[7] = 0xFF;

  // Convert PCM data to AM824 format: each sample becomes a labeled quadlet
  // Label 0x40 = multi-bit linear audio (MBLA)
  // PCM data comes in big-endian 24-bit-in-32-bit format (same as AAF)
  uint16_t cip_header_len = 8;
  uint16_t pcm_offset = 0;
  uint16_t am824_offset = cip_header_len;
  int stride = 4; // 32-bit container for 24-bit audio
  int num_samples = data_len / (channels * stride);

  for (int s = 0; s < num_samples; s++) {
    for (int ch = 0; ch < channels; ch++) {
      if (am824_offset + 4 <= AVTP_STREAM_DATA_PER_MSG &&
          pcm_offset + 3 < data_len) {
        // AM824 quadlet: label(8) + sample(24), big-endian
        // PCM buffer is left-justified: [0]=MSB, [1]=MID, [2]=LSB, [3]=pad
        msg.stream_data[am824_offset + 0] = 0x40; // MBLA label
        msg.stream_data[am824_offset + 1] = pcm_data[pcm_offset + 0]; // MSB
        msg.stream_data[am824_offset + 2] = pcm_data[pcm_offset + 1];
        msg.stream_data[am824_offset + 3] = pcm_data[pcm_offset + 2]; // LSB
      }
      pcm_offset += stride;
      am824_offset += 4; // one quadlet per channel per sample
    }
  }

  // Stream data length = CIP header + AM824 data blocks
  uint16_t sdl = am824_offset;
  msg.stream_data_len[0] = (sdl >> 8) & 0xFF;
  msg.stream_data_len[1] = sdl & 0xFF;

  // Total message length
  uint16_t msg_len =
      sizeof(iec_61883_6_message_s) - AVTP_STREAM_DATA_PER_MSG + sdl;

  uint8_t eth_frame[msg_len + ETH_HEADER_LEN + sizeof(struct eth_vlan_hdr)];
  avb_create_eth_frame(eth_frame, dest_addr, state, ethertype_vlan, &msg,
                       msg_len, vlan_id);
  int ret = write(state->port[0].l2if[VLAN], eth_frame, sizeof(eth_frame));
  if (ret < 0) {
    avberr("send IEC 61883 failed: %d", errno);
  }
  return ret;
}

/* cip_sfc_to_sample_rate moved to msrp.c (the SRP TALKER advertise
 * needs it to compute reservation bandwidth; the AVTP talker still
 * calls it via the avb.h forward declaration). */

/* Map AAF sample rate enum to Hz */

/* Map sample rate in Hz to AAF sample rate enum */
static uint8_t sample_rate_to_aaf_code(uint32_t sample_rate) {
  switch (sample_rate) {
  case 8000:
    return aaf_pcm_sample_rate_8k;
  case 16000:
    return aaf_pcm_sample_rate_16k;
  case 24000:
    return aaf_pcm_sample_rate_24k;
  case 32000:
    return aaf_pcm_sample_rate_32k;
  case 44100:
    return aaf_pcm_sample_rate_44_1k;
  case 48000:
    return aaf_pcm_sample_rate_48k;
  case 88200:
    return aaf_pcm_sample_rate_88_2k;
  case 96000:
    return aaf_pcm_sample_rate_96k;
  case 176400:
    return aaf_pcm_sample_rate_176_4k;
  case 192000:
    return aaf_pcm_sample_rate_192k;
  default:
    return aaf_pcm_sample_rate_48k;
  }
}

/* AVB Stream output task - generates sine wave and sends as AVTP AAF stream
 *
 * This task generates a sine wave using the esp_codec_dev component (ES8311)
 * for I2S clocking, and packages the audio as AVTP AAF PCM packets sent
 * over Ethernet.
 *
 * For Class A streams: 125us intervals, 6 samples/packet at 48kHz
 * For Class B streams: 250us intervals, 12 samples/packet at 48kHz
 */
/* Convert I2S stereo 24-bit (3 bytes/sample) to multi-channel AVTP.
 * I2S Philips format on ESP32: MSB-first on wire, stored in DMA as
 * little-endian 24-bit: byte[0]=LSB, byte[1]=MID, byte[2]=MSB.
 * ES8311 is mono — L channel has mic data, R channel duplicated or silence.
 * Writes L channel data to ch0+ch1, pads remaining channels with silence.
 *
 * AM824: [0x40, MSB, MID, LSB] per channel (4 bytes)
 * AAF:   [MSB, MID, LSB, 0x00] per channel (4 bytes)
 */
/* Mic-path DSP for low-output microphones (e.g. a dynamic mic on the
 * ES8389, whose PGA tops out at +36.5 dB): a one-pole DC-block strips
 * the codec MICBIAS offset (measured -12 %FS with a dynamic capsule on
 * a biased input), then digital gain with 24-bit saturation.
 * DC-block: y = x - x_prev + a*y_prev, a = 32735/32768 -> -3 dB at
 * ~30 Hz @192 kHz. Gain factor is Q16, derived from
 * CONFIG_ESP_AVB_TALKER_MIC_DIGITAL_GAIN_DB at stream start. */
typedef struct {
  int32_t x_prev;
  int32_t y_prev;
  int32_t gain_q16; /* 65536 = unity */
  /* Output mute for the slot-slip guard: applied after the DC block so
   * the filter keeps tracking the live input while muted and unmutes
   * without a DC step. mute_step ramps mute_q16 back to unity. */
  int32_t mute_q16; /* 65536 = open */
  int32_t mute_step;
} mic_dsp_state_s;

static inline int32_t mic_dsp_sample(mic_dsp_state_s *st, int32_t x) {
  int64_t y = (int64_t)x - st->x_prev + (((int64_t)st->y_prev * 32735) >> 15);
  st->x_prev = x;
  if (y > INT32_MAX)
    y = INT32_MAX;
  if (y < INT32_MIN)
    y = INT32_MIN;
  st->y_prev = (int32_t)y;
  int64_t g = (y * st->gain_q16) >> 16;
  if (g > 0x7FFFFF)
    g = 0x7FFFFF;
  if (g < -0x800000)
    g = -0x800000;
  if (st->mute_q16 < 65536) {
    g = (g * st->mute_q16) >> 16;
    if (st->mute_step) {
      st->mute_q16 += st->mute_step;
      if (st->mute_q16 >= 65536) {
        st->mute_q16 = 65536;
        st->mute_step = 0;
      }
    }
  }
  return (int32_t)g;
}

/* Sign-extend a 24-bit BE sample and run it through the mic DSP,
 * returning 24-bit BE bytes in b[0..2]. */
static inline void mic_dsp_be24(mic_dsp_state_s *st, const uint8_t *src,
                                uint8_t *b) {
  int32_t x = ((int32_t)((src[0] << 24) | (src[1] << 16) | (src[2] << 8))) >> 8;
  int32_t v = mic_dsp_sample(st, x);
  b[0] = (v >> 16) & 0xFF;
  b[1] = (v >> 8) & 0xFF;
  b[2] = v & 0xFF;
}

static void i2s24_to_am824_mono(const uint8_t *in, uint8_t *out,
                                int num_samples, int stream_channels,
                                mic_dsp_state_s *dsp) {
  for (int s = 0; s < num_samples; s++) {
    /* Carry only ADC-L — byte[0]=MSB, byte[1]=MID, byte[2]=LSB
     * (big_endian=true in slot_cfg matches AVTP wire order). The XLR
     * mic lands on ADC-L (MIC1); ADC-R has no connected input on this
     * board and captures only floating-input hiss (measured: constant
     * −42 dBFS on the listener's R output), so send digital silence on
     * every channel past ch0 rather than the noise. */
    const uint8_t *l = in;
    in += 8; /* 24-in-32 I2S slots, stereo capture frame */
    for (int ch = 0; ch < stream_channels; ch++) {
      const uint8_t *src = (ch == 0) ? l : NULL;
      out[0] = 0x40;
      if (src) {
        mic_dsp_be24(dsp, src, &out[1]);
      } else {
        out[1] = 0;
        out[2] = 0;
        out[3] = 0;
      }
      out += 4;
    }
  }
}

static void i2s24_to_aaf_mono(const uint8_t *in, uint8_t *out, int num_samples,
                              int stream_channels, mic_dsp_state_s *dsp) {
  for (int s = 0; s < num_samples; s++) {
    /* Carry only ADC-L (see i2s24_to_am824_mono — ADC-R is an
     * unconnected input on this board, floating-input hiss only). */
    const uint8_t *l = in;
    in += 8; /* 24-in-32 I2S slots, stereo capture frame */
    for (int ch = 0; ch < stream_channels; ch++) {
      const uint8_t *src = (ch == 0) ? l : NULL;
      if (src) {
        mic_dsp_be24(dsp, src, out);
      } else {
        out[0] = 0;
        out[1] = 0;
        out[2] = 0;
      }
      out[3] = 0;
      out += 4;
    }
  }
}

/* Sine LUT 32-bit BE [MSB,MID,LSB,pad] (4 bytes) → AM824 [0x40,MSB,MID,LSB] */
static inline void be32_to_am824(const uint8_t *in, uint8_t *out, int n) {
  for (int i = 0; i < n; i++) {
    out[0] = 0x40;
    out[1] = in[0];
    out[2] = in[1];
    out[3] = in[2];
    in += 4;
    out += 4;
  }
}

/* Sine LUT 32-bit BE [MSB,MID,LSB,pad] (4 bytes) → AAF (same layout, copy) */
static inline void be32_to_aaf(const uint8_t *in, uint8_t *out, int n) {
  memcpy(out, in, n * 4);
}

#if CONFIG_ESP_AVB_LISTENER_MIRROR
/* Listener mirror (test builds): the listener drain pushes every byte it
 * hands the I2S TX DMA, and the talker pops the same count per packet in
 * place of its capture, so its stream carries the playout bit for bit.
 * TX and RX share one I2S clock and frame layout (stereo 24-in-32), so
 * the fill holds steady once both run. One producer, one consumer. */
#if CONFIG_IDF_TARGET_ESP32P4
#define MIRROR_CAPACITY 32768 /* power of two: ~21 ms at 192 kHz */
#else
#define MIRROR_CAPACITY 16384 /* stations stop at 96 kHz: ~21 ms */
#endif
#define MIRROR_FRAME 8        /* one stereo 24-in-32 I2S frame */
#define MIRROR_TARGET_MS 4    /* standing fill: rides out drain bursts */
static uint8_t s_mirror_buf[MIRROR_CAPACITY];
static _Atomic uint32_t s_mirror_head, s_mirror_tail;
static uint32_t s_mirror_over_bytes;    /* producer found no room */
static uint32_t s_mirror_under_bytes;   /* consumer zero-filled while primed */
static uint32_t s_mirror_skipped_bytes; /* consumer trimmed a backlog */
static uint32_t s_mirror_reprimes;      /* underruns that emptied it */
static bool s_mirror_primed;
static int64_t s_mirror_last_pop_us;
static volatile uint32_t s_mirror_last_push_ms;

static void mirror_push(const uint8_t *src, uint32_t len) {
  uint32_t head = atomic_load_explicit(&s_mirror_head, memory_order_relaxed);
  uint32_t tail = atomic_load_explicit(&s_mirror_tail, memory_order_acquire);
  if (MIRROR_CAPACITY - (head - tail) < len) {
    s_mirror_over_bytes += len;
    return;
  }
  uint32_t pos = head & (MIRROR_CAPACITY - 1);
  uint32_t first = MIRROR_CAPACITY - pos;
  if (first > len)
    first = len;
  memcpy(s_mirror_buf + pos, src, first);
  memcpy(s_mirror_buf, src + first, len - first);
  atomic_store_explicit(&s_mirror_head, head + len, memory_order_release);
  s_mirror_last_push_ms = (uint32_t)(esp_timer_get_time() / 1000);
}

/* Fill dst with the next len bytes of playout, zeros where the listener
 * wrote none (idle, or the DAC running on auto_clear silence). The drain
 * writes whole DMA buffers each tick while the talker takes a packet's
 * worth, so the mirror plays out of a standing fill of target bytes: it
 * sends silence until that much is queued, and again after an underrun
 * empties it. A talker that (re)starts finds what the listener left
 * since it last ran; that backlog, or one near overflow, is trimmed back
 * to the target, or dropped whole when the listener has stopped (its
 * last 4 ms opened the next stream: plaa saw an earlier cell's tone).
 * Steady state never drops played samples. */
static void mirror_pop(uint8_t *dst, uint32_t len, uint32_t target) {
  int64_t now_us = esp_timer_get_time();
  uint32_t tail = atomic_load_explicit(&s_mirror_tail, memory_order_relaxed);
  uint32_t head = atomic_load_explicit(&s_mirror_head, memory_order_acquire);
  uint32_t avail = head - tail;
  if ((now_us - s_mirror_last_pop_us > 100000 ||
       avail > MIRROR_CAPACITY * 3 / 4) &&
      avail > target) {
    bool listening =
        (uint32_t)(now_us / 1000) - s_mirror_last_push_ms <= 100;
    uint32_t skip = avail - (listening ? target : 0);
    skip -= skip % MIRROR_FRAME;
    s_mirror_skipped_bytes += skip;
    tail += skip;
    avail -= skip;
    if (!listening)
      s_mirror_primed = false;
  }
  s_mirror_last_pop_us = now_us;
  if (!s_mirror_primed) {
    if (avail < target + len) {
      memset(dst, 0, len);
      atomic_store_explicit(&s_mirror_tail, tail, memory_order_release);
      return;
    }
    s_mirror_primed = true;
  }
  uint32_t take = avail < len ? avail : len;
  take -= take % MIRROR_FRAME;
  uint32_t pos = tail & (MIRROR_CAPACITY - 1);
  uint32_t first = MIRROR_CAPACITY - pos;
  if (first > take)
    first = take;
  memcpy(dst, s_mirror_buf + pos, first);
  memcpy(dst + first, s_mirror_buf, take - first);
  if (take < len) {
    memset(dst + take, 0, len - take);
    s_mirror_under_bytes += len - take;
    s_mirror_reprimes++;
    s_mirror_primed = false;
  }
  atomic_store_explicit(&s_mirror_tail, tail + take, memory_order_release);
}

/* Mirror frames to the wire: the playout's left and right slots, bit for
 * bit, on channels 0 and 1, silence on the rest. No mic DSP. */
static void i2s24_to_aaf_stereo(const uint8_t *in, uint8_t *out,
                                int num_samples, int stream_channels) {
  for (int s = 0; s < num_samples; s++) {
    for (int ch = 0; ch < stream_channels; ch++) {
      if (ch < 2)
        memcpy(out, in + ch * 4, 3);
      else
        memset(out, 0, 3);
      out[3] = 0;
      out += 4;
    }
    in += MIRROR_FRAME;
  }
}

static void i2s24_to_am824_stereo(const uint8_t *in, uint8_t *out,
                                  int num_samples, int stream_channels) {
  for (int s = 0; s < num_samples; s++) {
    for (int ch = 0; ch < stream_channels; ch++) {
      out[0] = 0x40;
      if (ch < 2)
        memcpy(out + 1, in + ch * 4, 3);
      else
        memset(out + 1, 0, 3);
      out += 4;
    }
    in += MIRROR_FRAME;
  }
}
#endif

/* A packet sent this long after its slot counts as a talker stall
 * (STREAM-OUT stalls=), whatever the pacing. */
#define TALKER_STALL_US 4000

#if CONFIG_ESP_AVB_STATION_TALKER_BATCHED
/* Batched pacing on a single-core station (see the top of the talker
 * loop): a GPTimer alarm wakes the talker when its next batch is due.
 * One talker at a time owns it; the timer is made once and kept.
 * gptimer_set_alarm_action() wants its config in internal RAM. */
static gptimer_handle_t s_talker_timer;
static TaskHandle_t volatile s_talker_task;
static DRAM_ATTR gptimer_alarm_config_t s_talker_alarm;
static portMUX_TYPE s_talker_timer_lock = portMUX_INITIALIZER_UNLOCKED;

static bool IRAM_ATTR
talker_batch_alarm(gptimer_handle_t timer,
                   const gptimer_alarm_event_data_t *event, void *arg) {
  (void)timer;
  (void)event;
  (void)arg;
  TaskHandle_t task = s_talker_task;
  BaseType_t woken = pdFALSE;
  if (task)
    vTaskNotifyGiveFromISR(task, &woken);
  return woken == pdTRUE;
}

/* Claim the batch timer for the calling task, making it on first use.
 * False when another talker holds it or the timer cannot be made. */
static bool talker_batch_timer_claim(void) {
  bool claimed = false;
  portENTER_CRITICAL(&s_talker_timer_lock);
  if (s_talker_task == NULL) {
    s_talker_task = xTaskGetCurrentTaskHandle();
    claimed = true;
  }
  portEXIT_CRITICAL(&s_talker_timer_lock);
  if (!claimed || s_talker_timer)
    return claimed;
  gptimer_config_t timer_config = {.clk_src = GPTIMER_CLK_SRC_DEFAULT,
                                   .direction = GPTIMER_COUNT_UP,
                                   .resolution_hz = 1000000};
  gptimer_event_callbacks_t callbacks = {.on_alarm = talker_batch_alarm};
  gptimer_handle_t timer = NULL;
  if (gptimer_new_timer(&timer_config, &timer) != ESP_OK ||
      gptimer_register_event_callbacks(timer, &callbacks, NULL) != ESP_OK ||
      gptimer_enable(timer) != ESP_OK || gptimer_start(timer) != ESP_OK) {
    avberr("Stream out: no batch timer, spinning");
    if (timer) {
      gptimer_disable(timer);
      gptimer_del_timer(timer);
    }
    s_talker_task = NULL;
    return false;
  }
  s_talker_timer = timer;
  return true;
}

/* Wake the claiming talker in delay_us. */
static void talker_batch_timer_arm(int64_t delay_us) {
  uint64_t now_ticks = 0;
  gptimer_get_raw_count(s_talker_timer, &now_ticks);
  s_talker_alarm.alarm_count =
      now_ticks + (uint64_t)(delay_us < 5 ? 5 : delay_us);
  gptimer_set_alarm_action(s_talker_timer, &s_talker_alarm);
}
#endif

/* Frame layout constants for ETH+VLAN+AVTP */
#define TX_ETH_HDR_LEN 14  /* dst(6) + src(6) + ethertype(2) */
#define TX_VLAN_TAG_LEN 4  /* TCI(2) + inner ethertype(2) */
#define TX_AVTP_HDR_LEN 24 /* AVTP common header */
#define TX_CIP_HDR_LEN 8   /* IEC 61883 CIP header */
#define TX_HDR_LEN_AAF (TX_ETH_HDR_LEN + TX_VLAN_TAG_LEN + TX_AVTP_HDR_LEN)
#define TX_HDR_LEN_61883 (TX_HDR_LEN_AAF + TX_CIP_HDR_LEN)

/* Samples per packet the talker's converter takes: 48 at 192 kHz in
 * Class B. */
#define TALKER_SRC_MAX_SPP 64

static void avb_stream_out_task(void *task_param) {
  /* Reset productive-work accumulator on every stream_out_task
   * start so each streaming session stands alone in the CPU stats. */
  atomic_store_explicit(&s_stream_out_work_us, 0, memory_order_relaxed);

  /* Allocate and publish the diag context — mirrors stream-in pattern.
   * Counters zeroed via calloc; active=true once populated. Freed in
   * the cleanup block below so the AVB-STATS reader never sees a
   * dangling pointer. */
  stream_tx_ctx_t *tx_ctx = calloc(1, sizeof(stream_tx_ctx_t));
  if (tx_ctx) {
    {
      static uint32_t s_tx_start_serial = 0;
      tx_ctx->start_serial = ++s_tx_start_serial;
    }
    tx_ctx->active = true;
    s_stream_tx_ctx = tx_ctx;
    s_tx_starts++;
  }
  avbinfo("Starting stream out task");

  uint8_t *sine_lut = NULL;
  uint32_t lut_samples = 0;
  uint32_t lut_pos = 0;
  uint8_t *pcm_buf = NULL;
  uint8_t *i2s_buf = NULL;
  uint8_t *i2s_ring = NULL;
  uint8_t *tx_frame = NULL;
  esp_task_wdt_user_handle_t wdt_handle = NULL;
  struct stream_out_params_s *params = (struct stream_out_params_s *)task_param;
  if (params == NULL)
    goto err;

  avb_state_s *state = (avb_state_s *)params->state;
  uint8_t seq_num = 0;
  uint16_t dbc = 0;
  uint8_t sample_rate_code = sample_rate_to_aaf_code(params->sample_rate);
  bool is_am824 = (params->format_subtype == avtp_subtype_61883);

  /* I2S reads stereo (2ch) regardless of stream channel count.
   * Extra channels (ch2-7) are zero-padded in the AVTP conversion.
   * Codec configures I2S with 24-bit data / 24-bit slot = 3 bytes/sample. */
  int i2s_channels = 2;         /* ES8311 mic is stereo */
  int i2s_bytes_per_sample = 4; /* 24-in-32 slot: [MSB MID LSB 00] */
  /* Samples per packet: rate / (1e6/interval). Integer for 48/96/192 kHz;
   * fractional for the 44.1 kHz family (44100/8000 = 5.5125), where a
   * Bresenham accumulator emits spp_base or spp_base+1 each packet so
   * the running average is exactly the true rate (a plain floor shipped
   * a 40 kHz stream at 44.1 kHz). Everything sized per-packet from here. */
  int spp_den = 1000000 / params->interval;      /* 8000 (A) or 4000 (B) */
  int spp_base = (int)params->sample_rate / spp_den;
  int spp_rem = (int)params->sample_rate % spp_den;
  int spp_acc = 0;
  int spp_max = spp_base + (spp_rem ? 1 : 0);
  int i2s_frame_bytes_pkt = i2s_channels * i2s_bytes_per_sample;
  int i2s_read_nominal = spp_base * i2s_frame_bytes_pkt; /* fill-loop scale */
  int i2s_read_size = spp_max * i2s_frame_bytes_pkt;     /* buffer size */
#if CONFIG_ESP_AVB_LISTENER_MIRROR
  uint32_t mirror_target =
      params->sample_rate / 1000 * MIRROR_TARGET_MS * MIRROR_FRAME;
#endif

  i2s_buf = calloc(1, i2s_read_size);
  if (!i2s_buf) {
    avberr("Stream out: no memory for I2S buffer");
    goto err;
  }

  /* Sine table: the whole source in the test tone build, and what the
   * Test Tone control plays in place of the codec input. */
  lut_samples = build_sine_lut(&sine_lut, params->channels, params->bit_depth,
                               params->sample_rate, params->sine_freq);
  if (!sine_lut) {
    avberr("Stream out: Failed to build sine LUT");
    goto err;
  }
  int bps = (params->bit_depth == 24) ? 4 : (params->bit_depth / 8);
  pcm_buf = calloc(1, spp_max * params->channels * bps);
  if (!pcm_buf)
    goto err;

  /* Pre-build the TX frame — constant fields filled once.
   * Audio data offset depends on format (AAF vs AM824). */
  int audio_data_max = spp_max * params->channels * 4;
  int audio_data_len = spp_base * params->channels * 4; /* nominal, for hdr init/log */
  int stream_data_len =
      is_am824 ? (TX_CIP_HDR_LEN + audio_data_len) : audio_data_len;
  int audio_offset = is_am824 ? TX_HDR_LEN_61883 : TX_HDR_LEN_AAF;
  int frame_len = audio_offset + audio_data_max;
  tx_frame = calloc(1, frame_len);
  if (!tx_frame) {
    avberr("Stream out: no memory for TX frame");
    goto err;
  }

  /* ETH header: dst MAC + src MAC + 0x8100 (VLAN ethertype) */
  memcpy(tx_frame, &params->dest_addr, ETH_ADDR_LEN);
  memcpy(tx_frame + ETH_ADDR_LEN, state->port[0].internal_mac_addr, ETH_ADDR_LEN);
  tx_frame[12] = 0x81;
  tx_frame[13] = 0x00; /* VLAN ethertype */

  /* VLAN tag: PCP + VID, inner ethertype 0x22F0 */
  uint16_t vid = (params->vlan_id[0] << 8) | params->vlan_id[1];
  uint16_t pcp = avb_msrp_priority_for_stream(state, params->stream_index);
  uint16_t tci = (pcp << 13) | (vid & 0x0FFF);
  tx_frame[14] = (tci >> 8) & 0xFF;
  tx_frame[15] = tci & 0xFF;
  tx_frame[16] = 0x22;
  tx_frame[17] = 0xF0; /* inner AVTP ethertype */

  /* AVTP header (starts at offset 18) */
  uint8_t *avtp = tx_frame + TX_ETH_HDR_LEN + TX_VLAN_TAG_LEN;
  avtp[0] = is_am824 ? avtp_subtype_61883 : avtp_subtype_aaf;
  avtp[1] = 0x81; /* sv=1, version=0, mr=1 (first pkt), tv=1 */
  /* avtp[2] = seq_num — set per-packet */
  avtp[3] = 0x00; /* tu=0 */
  memcpy(avtp + 4, &params->stream_id,
         UNIQUE_ID_LEN); /* stream_id at [4..11] */
  /* avtp[12..15] = avtp_ts — set per-packet */

  if (is_am824) {
    /* IEC 61883 specific fields */
    avtp[16] = 0x00;
    avtp[17] = 0x00;
    avtp[18] = 0x00;
    avtp[19] = 0x00; /* gateway */
    avtp[20] = (stream_data_len >> 8) & 0xFF;
    avtp[21] = stream_data_len & 0xFF;
    avtp[22] =
        (1 << 6) | 31;      /* tag=1 (CIP present), channel=31 (AVTP native) */
    avtp[23] = (0x0A << 4); /* tcode=1010, sy=0 */

    /* CIP header (at avtp + 24) */
    uint8_t *cip = avtp + TX_AVTP_HDR_LEN;
    cip[0] = 0x3F; /* SID=0x3F */
    cip[1] = params->dbs;
    cip[2] = 0x00; /* FN=0, QPC=0, SPH=0 */
    /* cip[3] = DBC — set per-packet */
    cip[4] = 0x90; /* EOH=1, FMT=0x10 (AM824) */
    cip[5] = params->cip_sfc & 0x07;
    cip[6] = 0xFF;
    cip[7] = 0xFF; /* SYT=0xFFFF */
  } else {
    /* AAF specific fields */
    avtp[16] = (params->bit_depth <= 16) ? 0x02 : 0x04; /* format */
    avtp[17] =
        (sample_rate_code << 4); /* sample_rate in upper nibble, padding=0 */
    /* Actually need to check AAF header layout more carefully */
    avtp[16] =
        (params->bit_depth <= 16) ? aaf_format_int_16bit : aaf_format_int_32bit;
    avtp[17] =
        (sample_rate_code & 0x0F); /* nsr in lower 4 bits, padding in upper */
    /* Rewriting properly using struct knowledge */
    avtp[16] =
        (params->bit_depth <= 16) ? aaf_format_int_16bit : aaf_format_int_32bit;
    avtp[17] =
        ((sample_rate_code & 0x0F) << 4); /* sample rate in upper nibble */
    avtp[18] = params->channels;
    avtp[19] = params->bit_depth;
    avtp[20] = (audio_data_len >> 8) & 0xFF;
    avtp[21] = audio_data_len & 0xFF;
    avtp[22] = 0x00; /* evt=0, sparse_ts=0 */
    avtp[23] = 0x00; /* reserved */
  }

  avbinfo("Stream out: rate=%luHz ch=%d samples/pkt=%d interval=%dus "
          "frame=%d bytes audio@%d %s",
          params->sample_rate, params->channels, params->samples_per_packet,
          params->interval, frame_len, audio_offset,
          params->use_sine_wave ? "sine" : "mic");

  uint32_t avtp_media_ts = 0;
  /* Base media-clock advance per packet, exact in Q16 ns/sample so a
   * fractional samples-per-packet still sums to the true rate. */
  uint64_t ns_per_sample_q16 =
      ((uint64_t)1000000000ULL << 16) / params->sample_rate;
  uint64_t ts_base_frac_q16 = 0;
  /* 61883-6 SYT_INTERVAL by rate family (IEC 61883-6 Table 4):
   * ≤48 kHz → 8, 88.2/96 kHz → 16, 176.4/192 kHz → 32. Used with DBC
   * to pick which data block the avtp_timestamp refers to. */
  uint32_t syt_interval = params->sample_rate <= 48000   ? 8
                          : params->sample_rate <= 96000 ? 16
                                                         : 32;
  uint32_t ns_per_sample = (uint32_t)((1000000000ULL + params->sample_rate / 2) /
                                      params->sample_rate);

  /* Per-window min/max — seeded as INT32_MAX/MIN so the first sample
   * always captures as both ends of the range. Reset by the 1 Hz
   * publish branch to the same sentinels. */
  int32_t pll_offset_max = INT32_MIN, pll_offset_min = INT32_MAX;
  uint32_t pll_measure_count = 0, pll_skip_count = 0;

  /* gPTP-cadence drift tracking — each publish cycle computes the
   * gPTP-vs-esp_timer rate ratio over the elapsed window. Tells us
   * how much the gPTP-paced scheduler is actually correcting for. */
  int64_t drift_gptp_ref_ns = 0;
  int64_t drift_esp_ref_us = 0;
  bool drift_ref_valid = false;
  int32_t remaining_ns_min = INT32_MAX, remaining_ns_max = INT32_MIN;
  uint32_t gptp_resync_count = 0;
  /* Sub-µs carry for the ns→µs truncation in the gPTP-paced scheduler. */
  int32_t sched_ns_carry = 0;
  /* Periodic gPTP-vs-esp_timer resync measurement to produce a filtered
   * per-packet correction. Reading gPTP every packet injected ptpd's
   * 125 ms servo ripple directly into TX cadence. Sampling every ~1 s
   * still passed servo phase noise (measured ±12 ppm rate wander with
   * ~70 s period at the wire). The slip measurement carries a roughly
   * fixed few-µs noise regardless of baseline length, so a long
   * baseline attenuates injected wander proportionally: at 60 s the
   * same noise is ±0.05 ppm. Between samples the cadence is pure
   * esp_timer-paced with the filtered ppm offset applied.
   *
   * Acquire/track split: the first sample uses a short warmup baseline
   * and is applied directly, so the ~tens-of-ppm crystal offset is
   * trimmed within seconds; afterwards samples arrive every
   * CONFIG_ESP_AVB_TALKER_CADENCE_LOCK_INTERVAL_S and enter the EMA
   * with the per-update step clamped to ±1 ppm. Interval 0 disables
   * the lock entirely (free-run with gPTP-seeded timestamps). */
  #define SCHED_FILTER_SHIFT 3       /* EMA tau ≈ 8 lock intervals */
  #define SCHED_WARMUP_US 4000000LL  /* acquire baseline: ±1 ppm noise */
  /* ±1 ppm per update, in q16 ns-per-packet: interval_us µs × 1e3 ns/µs
   * × 1 ppm = interval_us × 0.001 ns. */
  const int64_t sched_step_max_q16 =
      ((int64_t)params->interval << 16) / 1000;
  /* Packets per lock interval; 0 disables the rate lock. */
  const int64_t sched_resync_pkts =
      (int64_t)CONFIG_ESP_AVB_TALKER_CADENCE_LOCK_INTERVAL_S * 1000000LL /
      params->interval;
  const int64_t sched_warmup_pkts = SCHED_WARMUP_US / params->interval;
  int64_t sched_pkts_since_sample = 0;
  bool sched_locked = false; /* false = next sample is an acquire */
  int64_t sched_gptp_ref_ns = 0;
  int64_t sched_esp_ref_us = 0;
  bool sched_ref_valid = false;
  /* per-packet correction in nanoseconds, signed. Positive = wait a bit
   * longer each packet (ESP crystal fast). Q16 fixed-point so we can
   * carry sub-nanosecond resolution across packets. */
  int64_t sched_ns_adj_q16 = 0;

  esp_task_wdt_add_user("AVB-OUT", &wdt_handle);
  esp_log_level_set("ptpd", ESP_LOG_NONE);
  esp_log_level_set("esp.emac", ESP_LOG_NONE);
  /* The Wi-Fi driver logs each FTM ranging session at warning level; on
   * a single-core station those console writes stall the stream. */
  esp_log_level_set("wifi", ESP_LOG_ERROR);

  uint32_t loop_count = 0;
  uint32_t overrun_count = 0;
  int64_t overrun_max = 0;
  uint32_t send_fail_count = 0;
  esp_err_t last_send_error = ESP_OK;
  uint32_t i2s_zero_reads = 0;    /* reads that returned 0 bytes */
  uint32_t i2s_nonzero_audio = 0; /* reads with non-zero audio data */
  uint32_t ring_underruns = 0;    /* packets sent zero-filled, ring empty */
  uint32_t ts_reanchors = 0;
  int32_t ts_err_last = 0;
  uint64_t i2s_dropped_base = atomic_load_explicit(
      &state->media_clock.i2s_bytes_dropped, memory_order_relaxed);
  /* ADC-following cadence, see the fill controller in the 1 s branch. */
  int64_t fill_adj_q16 = 0;      /* extra ns per packet, Q16 */
  int32_t fill_frac_q16 = 0;     /* sub-ns carry of fill_adj_q16 */
  int64_t fill_integ_ppm_q8 = 0; /* integral term, ppm x 256 */
  int64_t fill_err_ema_q8 = 0;   /* smoothed backlog error, bytes x 256 */
  int32_t fill_ppm_q8 = 0;
  uint64_t adc_bytes_base = 0;

  /* gPTP discontinuity detection — mr and tu bit management.
   * mr toggles on media clock restart, tu=1 on gPTP BTC change.
   * Both held for MCR_HOLD_PACKETS after the event. */
#define MCR_HOLD_PACKETS 8
  uint8_t last_btc_id[8];
  memcpy(last_btc_id, state->ptp_status.clock_source_info.btc_id, 8);
  bool mr_state = true;         /* starts toggled (first packet = restart) */
  int mcr_hold_remaining = MCR_HOLD_PACKETS;
  bool tu_active = false;
  int tu_hold_remaining = 0;
  /* Presentation-timestamp rate lock. ts_rate_corr_q16 is the extra
   * ns/packet (Q16) added on top of the nominal increment so the
   * timestamp clock advances at the MEASURED gPTP rate instead of the
   * nominal esp_timer rate (~±30 ppm apart). Updated once per ~1 s
   * window with a ±0.25 ns/pkt (≈2 ppm) step clamp: convergence is
   * slow and one-directional, so listener clock-recovery loops (macOS
   * SYT recovery chased the earlier sign-flipping slew into audible
   * oscillation) see an essentially constant timestamp rate.
   * ts_rate_frac_q16 carries the sub-ns remainder between packets. */
  int32_t ts_rate_corr_q16 = 0;
  int32_t ts_rate_frac_q16 = 0;
  /* The stream's samples against its own presentation times, which is
   * the rate every listener recovers: running totals of the timestamp
   * advance and of the samples sent, and the window they are compared
   * over (0 = none, restarted by a re-anchor). */
  int64_t rate_ts_ns = 0, rate_window_ts0 = 0;
  uint64_t rate_samples = 0, rate_window_samples0 = 0;
  int64_t rate_window_us = 0;

/* I2S RX local ring — absorbs DMA buffer timing mismatch.
 * Read larger chunks when available, consume i2s_read_size per packet. */
  /* Ring size must be a multiple of i2s_read_size (bytes per AVTP packet)
   * to avoid partial-frame reads at ring wrap boundaries.  Target ~5ms. */
  int i2s_frame_bytes = i2s_channels * i2s_bytes_per_sample;
  int i2s_ring_size = (int)(params->sample_rate * i2s_frame_bytes * 5 / 1000);
  /* Multiple of the frame size so ring wraps land on frame boundaries
   * (per-packet consume varies with the fractional spp, so it need not
   * divide the packet size); at least a few max packets deep. */
  i2s_ring_size -= i2s_ring_size % i2s_frame_bytes;
  if (i2s_ring_size < i2s_read_size * 4)
    i2s_ring_size = i2s_read_size * 4; /* minimum 4 packets */
  /* Backlog setpoint: half the 5 ms ring, ~2.5 ms of audio. */
  int ring_setpoint = i2s_ring_size / 2;
  /* Single-core stations: Wi-Fi preempts the stream task for up to
   * ~25 ms, longer than the RX DMA queue holds (~8 ms), and the
   * driver's read path then drops or repeats audio. There the RX
   * interrupt copies every buffer into a 32 ms ring (avbcodec.c), so a
   * stall delays audio instead of losing it. */
#if portNUM_PROCESSORS == 1
  bool capture_ring = !params->use_sine_wave;
#else
  const bool capture_ring = false;
#endif
  if (capture_ring) {
    i2s_ring_size = (int)(params->sample_rate * i2s_frame_bytes * 32 / 1000);
    i2s_ring_size -= i2s_ring_size % i2s_frame_bytes;
  }
  /* Without an APLL the codec clock cannot be trimmed to gPTP. Its audio
   * is converted to the stream rate instead: the stamps advance by
   * exactly the samples sent, the cadence follows the stamps, and the
   * backlog loop below sets the conversion ratio (a cadence that
   * followed the codec ran the station's stream 11 to 49 ppm off its
   * stamps, and every listener that holds its clock trimmed it). */
  static avb_src_s s_talker_src;
  /* Codec frames for one packet: one or two more than it carries. */
  static uint8_t src_in_buf[(TALKER_SRC_MAX_SPP + 2) * 8];
#if CONFIG_ESP_AVB_TALKER_RESAMPLE
  bool resample = false;
  if (!params->use_sine_wave && spp_max <= TALKER_SRC_MAX_SPP &&
      i2s_frame_bytes_pkt == 8) {
    resample = avb_src_init(&s_talker_src);
    if (resample)
      avbinfo("Stream out: codec input resampled to the stream rate");
    else
      avbwarn("Stream out: no memory to resample, pacing by the codec");
  }
#else
  const bool resample = false;
#endif
#if CONFIG_ESP_AVB_LISTENER_MIRROR
  const int src_slots = 2; /* the mirror carries both slots */
#else
  const int src_slots = 1; /* the converters carry only the left slot */
#endif
  int32_t src_rate_q32 = 0;
  int64_t cadence_integ_ppm_q8 = 0; /* cadence PI on the stamp error */
  uint32_t src_cycles = 0, src_frames = 0;
  uint64_t capture_dropped = 0;
  /* Monotonic byte counters — MUST be 64-bit. As 32-bit ints they
   * cross 2^31 after ~31 min of 192 kHz stereo 24-bit capture
   * (1.152 MB/s), C's % on the now-negative value yields a NEGATIVE
   * write_pos, and i2s_channel_read then DMA-writes up to a full
   * ring of mic audio BELOW the heap block — stomping neighboring
   * allocations/task stacks (observed: core 1 Instruction access
   * fault with the stack sprayed with near-silence audio, every
   * ~1871 s of the overnight 192 kHz soak). */
  uint64_t i2s_ring_head = 0, i2s_ring_tail = 0;
  /* Mic DSP: DC-block always on; digital gain from Kconfig (unity when
   * 0). powf at task start only. */
  mic_dsp_state_s mic_dsp = {0};
  mic_dsp.mute_q16 = 65536;
  /* Slot-slip guard (see the packet loop): jump-density state, hold
   * counter, counters, and a log budget for the first detections. */
  int32_t slip_prev_x = 0;
  /* Running mean |dx| in Q4. Kept in whole LSB, a >> 10 step cannot
   * rise on steps under 1024 above the mean and reads hundreds of LSB
   * low, which disables the relative gate. */
  int32_t slip_absdiff_ema_q4 = 0;
  uint32_t slip_sample = 100;       /* running sample index */
  uint32_t slip_edge_sample = 0;    /* sample index of the last step */
  int32_t slip_edge_dx = 0, slip_edge_mag = 0, slip_inner_max = 0;
  uint8_t slip_window[512] = {0};
  /* Pulse window spans 64 ms at any packet interval (512 at Class A,
   * 256 at Class B). */
  unsigned slip_window_len = (unsigned)(64000 / params->interval);
  if (slip_window_len < 1)
    slip_window_len = 1;
  if (slip_window_len > sizeof(slip_window))
    slip_window_len = sizeof(slip_window);
  unsigned slip_window_pos = 0, slip_window_sum = 0;
  uint32_t slip_armed_pkts = 0;
  uint32_t slip_mute_hold = 0;
  uint32_t slip_events = 0, slip_muted_pkts = 0;
  int slip_log_budget = 6;
  mic_dsp.gain_q16 =
      (int32_t)(powf(10.0f, (float)CONFIG_ESP_AVB_TALKER_MIC_DIGITAL_GAIN_DB /
                                 20.0f) *
                65536.0f);
  if (!params->use_sine_wave) {
    /* One talker runs at a time, so its I2S ring is kept between
     * starts and only grows. Allocated at every start, the 12 KB ring
     * failed on a C6 station whose Wi-Fi buffers had fragmented the
     * heap. */
    static uint8_t *s_i2s_ring_buf;
    static int s_i2s_ring_capacity;
    if (s_i2s_ring_capacity < i2s_ring_size) {
      free(s_i2s_ring_buf);
      s_i2s_ring_buf = calloc(1, i2s_ring_size);
      s_i2s_ring_capacity = s_i2s_ring_buf ? i2s_ring_size : 0;
    }
    i2s_ring = s_i2s_ring_buf;
    if (!i2s_ring) {
      avberr("Stream out: no memory for I2S ring (%d B, largest free block "
             "%u B)",
             i2s_ring_size,
             (unsigned)heap_caps_get_largest_free_block(MALLOC_CAP_DEFAULT));
      goto err;
    }
    memset(i2s_ring, 0, i2s_ring_size);
    if (capture_ring) {
      state->media_clock.capture_ring_size = (uint32_t)i2s_ring_size;
      state->media_clock.capture_write_pos = 0;
      atomic_store_explicit(&state->media_clock.capture_head, 0,
                            memory_order_relaxed);
      atomic_thread_fence(memory_order_release);
      state->media_clock.capture_ring = i2s_ring;
      int64_t prefill_deadline_us = esp_timer_get_time() + 200000;
      while (atomic_load_explicit(&state->media_clock.capture_head,
                                  memory_order_acquire) <
                 (uint64_t)ring_setpoint &&
             esp_timer_get_time() < prefill_deadline_us)
        vTaskDelay(1);
      i2s_ring_head = atomic_load_explicit(&state->media_clock.capture_head,
                                           memory_order_acquire);
      if (i2s_ring_head > (uint64_t)ring_setpoint) {
        i2s_ring_tail = i2s_ring_head - ring_setpoint;
        i2s_ring_tail -= i2s_ring_tail % i2s_frame_bytes;
      }
      avbinfo("Stream out: capture ring %d bytes, level %d bytes",
              i2s_ring_size, (int)(i2s_ring_head - i2s_ring_tail));
    } else {
      /* Pre-fill the I2S ring with blocking reads to establish buffer level.
       * I2S DMA returns in chunks larger than requested (driver adjusts
       * dma_frame_num), so we fill as much as we can. */
      /* Discard what the RX DMA queue already holds: the channel runs
       * from boot, so a stream starts on a full queue of stale audio.
       * Left there, it was consumed by the first packets but missing from
       * the backlog baseline below, and the fill trim then held the queue
       * at overflow. */
      for (int flush = 0; flush < 64; flush++) {
        size_t stale = 0;
        i2s_channel_read(params->i2s_rx_handle, i2s_ring, i2s_ring_size, &stale,
                         0);
        if (stale == 0)
          break;
      }
      int frame_size = i2s_channels * i2s_bytes_per_sample;
      int prefill_target = ring_setpoint;
      while (i2s_ring_head < (uint64_t)prefill_target) {
        int write_pos = (int)(i2s_ring_head % (uint64_t)i2s_ring_size);
        int space = i2s_ring_size - (int)i2s_ring_head;
        int chunk = i2s_ring_size - write_pos;
        if (chunk > space)
          chunk = space;
        /* Align to frame boundary to prevent partial-frame reads */
        chunk -= chunk % frame_size;
        if (chunk == 0)
          break;
        size_t got = 0;
        i2s_channel_read(params->i2s_rx_handle, i2s_ring + write_pos, chunk,
                         &got, 100);
        got -= got % frame_size; /* discard any trailing partial frame */
        if (got == 0)
          break; /* timeout — give up pre-fill */
        i2s_ring_head += got;
      }
      avbinfo("Stream out: I2S ring pre-filled %d bytes", (int)i2s_ring_head);
      /* Backlog setpoint: the blocking pre-fill just drained the RX DMA,
       * so from here (captured - consumed) tracks how far the ring + DMA
       * backlog has moved from the pre-fill level. */
      adc_bytes_base =
          atomic_load_explicit(&state->media_clock.i2s_bytes_captured,
                               memory_order_relaxed) -
          atomic_load_explicit(&state->media_clock.i2s_bytes_dropped,
                               memory_order_relaxed);
    }
  }

  /* Sample PTP and send-time as late as possible — all logging and pre-fill
   * is done.  This ensures the first packet's presentation timestamp is
   * accurate rather than stale by the pre-fill duration.
   *
   * Stagger initial send phase by a device-unique offset derived from the MAC.
   * Without this, PTP-synchronized ESPs transmit in lockstep (same 125μs
   * phase), causing systematic DMA contention between TX and RX on the
   * receiving side — both fire at the same instant every interval. */
  uint8_t mac_hash = state->port[0].internal_mac_addr[4] ^ state->port[0].internal_mac_addr[5];
  int64_t phase_offset = (mac_hash % params->interval);
  int64_t next_send_time = esp_timer_get_time() + phase_offset;

  /* gPTP-paced cadence: each packet's busy-wait target is re-projected
   * from the current gPTP time. Locks TX rate to gPTP (8000 packets per
   * gPTP-second) rather than to ESP's crystal, preventing cumulative
   * sample-count drift that causes listeners to drop/dup samples. */
  bool gptp_cadence_ready = false;
  bool gptp_seeded = false;
  for (int init_try = 0; init_try < 5; init_try++) {
    struct timespec ptp_a, ptp_b;
    if (ptpd_now(&ptp_a) == 0 &&
        ptpd_now(&ptp_b) == 0) {
      uint64_t ts_a = (uint64_t)ptp_a.tv_sec * 1000000000ULL +
                      (uint64_t)ptp_a.tv_nsec;
      uint64_t ts_b = (uint64_t)ptp_b.tv_sec * 1000000000ULL +
                      (uint64_t)ptp_b.tv_nsec;
      int64_t diff = (int64_t)(ts_b - ts_a);
      if (diff >= 0 && diff < 500000) {
        avtp_media_ts = (uint32_t)ts_a;
        gptp_seeded = true;
        /* Arm the filtered gPTP rate lock below only when a lock
         * interval is configured. At interval 0 the timestamp is still
         * gPTP-seeded but cadence free-runs on esp_timer (no gPTP
         * feedback into packet spacing). */
        gptp_cadence_ready = sched_resync_pkts > 0;
        break;
      }
    }
  }

  /* Batched pacing on a single-core station (see the top of the
   * loop). One talker can hold the batch timer; another, or the test
   * tone, keeps the spin. */
  int64_t late_allowance_us = 0;
  uint32_t wake_count = 0;
  uint32_t stall_count = 0;
#if CONFIG_ESP_AVB_STATION_TALKER_BATCHED
  /* A batch is one full Wi-Fi bundle of the stream's frames. */
#ifdef CONFIG_ESP_AVB_WIFI_BUNDLE
  const int batch_packets = CONFIG_ESP_AVB_WIFI_BUNDLE_FRAMES;
#else
  const int batch_packets = 8;
#endif
  bool batched = !params->use_sine_wave && talker_batch_timer_claim();
  if (batched) {
    late_allowance_us = (int64_t)(batch_packets - 1) * params->interval;
    vTaskPrioritySet(NULL, CONFIG_ESP_AVB_STATION_TALKER_PRIORITY);
    avbinfo("Stream out: batched, %d packets per wake, priority %d",
            batch_packets, CONFIG_ESP_AVB_STATION_TALKER_PRIORITY);
  }
#endif

  while (!state->output_streams[params->stream_index].stop_streaming) {
#if CONFIG_ESP_AVB_STATION_TALKER_BATCHED
    /* Batched pacing. With nothing due, sleep until the slot of the
     * packet that fills the open Wi-Fi bundle (a batch's worth when none
     * is open), then send what is due, each packet stamped for its own
     * slot: sending early packets late moves only their arrival, and
     * bundles fill as they did when every packet went out in its slot.
     * The talker yields to everything below it between batches instead
     * of spinning under it. The one-tick timeout lets a stop request
     * through. */
    if (batched) {
      int64_t until_due_us = next_send_time - esp_timer_get_time();
      if (until_due_us > 0) {
        int open_frames = avb_net_tx_bundle_frames(avtp + 4);
        int to_fill = batch_packets - open_frames % batch_packets;
        /* 20 us past that slot, which whole-us cadence rounding can
         * move by a microsecond. */
        talker_batch_timer_arm(until_due_us +
                               (int64_t)(to_fill - 1) * params->interval + 20);
        ulTaskNotifyTake(pdTRUE, 1);
        wake_count++;
        continue;
      }
    }
#endif
    /* Busy-wait until next send time. Prio 24 on core 1 — nothing else
     * needs that capacity (emac_rx is unpinned and lands on core 0 when
     * core 1 spins, AVB-IN is on core 0). Sub-µs precision required for
     * audio cadence; attempts to switch to a sem+timer approach failed
     * because the esp_timer ISR is masked by portENTER_CRITICAL and its
     * wake latency rose to 400+ µs under AVB-IN load — audible
     * distortion. */
    while (esp_timer_get_time() < next_send_time) {
#if portNUM_PROCESSORS == 1
      /* Single-core target: a pure spin at high priority starves the
       * Wi-Fi task, the console, and the idle task, the board wedges
       * the moment the stream starts. Yield each spin iteration so
       * equal-priority work runs; the task is also created at a low
       * priority on single-core (see task creation), so higher
       * priority Wi-Fi and protocol work preempts the spin. Cadence
       * precision degrades versus the dedicated-core design, which is
       * acceptable for a Class B wireless talker, listeners conceal. */
      taskYIELD();
#endif
    }

    /* Productive-work clock: counts only the time from end of busy-wait
     * through end of this iteration. Compared against wall-clock window
     * by avb_stream_out_work_us_delta() to show how much of AVB-OUT's
     * 100 % runtime is real work vs. busy-wait spin. */
    int64_t work_start_us = esp_timer_get_time();

    int64_t now = esp_timer_get_time();
    int64_t packet_sched_us = next_send_time;
    int64_t overrun = now - next_send_time;
    /* Late beyond the pacing's own allowance: half an interval, plus a
     * batch when batched. */
    if (overrun > late_allowance_us + params->interval / 2) {
      overrun_count++;
      if (overrun > overrun_max)
        overrun_max = overrun;
    }
    if (overrun >= TALKER_STALL_US)
      stall_count++;
    /* Filtered gPTP cadence: most packets advance by (interval_ns +
     * sched_ns_adj) with Q16-fixed-point carry to preserve sub-ns
     * precision. Every SCHED_RESYNC_PKTS packets we read gPTP, measure
     * how far off our free-running esp_timer schedule has slipped, and
     * update sched_ns_adj via an EMA. Result: esp_timer-precision
     * cadence with a heavily-filtered rate lock to gPTP. */
    sched_pkts_since_sample++;
    if (gptp_cadence_ready &&
        sched_pkts_since_sample >=
            (sched_locked ? sched_resync_pkts : sched_warmup_pkts)) {
      struct timespec ts_now;
      if (ptpd_now(&ts_now) == 0) {
        int64_t now_gptp_ns = (int64_t)ts_now.tv_sec * 1000000000LL +
                              (int64_t)ts_now.tv_nsec;
        if (!sched_ref_valid) {
          sched_gptp_ref_ns = now_gptp_ns;
          sched_esp_ref_us = now;
          sched_ref_valid = true;
          drift_gptp_ref_ns = now_gptp_ns;
          drift_esp_ref_us = now;
          drift_ref_valid = true;
        } else {
          int64_t gptp_delta = now_gptp_ns - sched_gptp_ref_ns;
          int64_t esp_delta_us = now - sched_esp_ref_us;
          int64_t esp_delta_ns = esp_delta_us * 1000LL;
          /* Guard against bogus baselines: accept only a window between
           * half and double the expected span (first tick / stalls). */
          int64_t expect_us = sched_pkts_since_sample * params->interval;
          if (esp_delta_us > expect_us / 2 && esp_delta_us < expect_us * 2) {
            int64_t diff_ns = gptp_delta - esp_delta_ns;
            /* Per-packet correction = accumulated diff over the actual
             * packet count in this window. Q16 so sub-ns survives. */
            /* gPTP counted diff_ns MORE than esp_timer over the window,
             * so each esp_timer ns is worth (1 + diff/esp) gPTP ns and
             * the esp_timer wait per packet must SHRINK by diff/pkts to
             * span one nominal gPTP interval. Adding it (the earlier
             * sign) doubled the crystal error instead of cancelling it,
             * and because the lock is open-loop (an absolute
             * measurement applied as trim) the wrong sign was perfectly
             * stable, showing only as a 2x packet-rate error at every
             * listener (observed: ~60 ppm ring drain, re-anchor splices
             * every few minutes on all listener boards). */
            int64_t sample_q16 = -((diff_ns << 16) / sched_pkts_since_sample);
            /* Large jump (gPTP step): re-baseline and drop back to
             * acquire so the trim recovers in one warmup window instead
             * of creeping at the tracking clamp rate. */
            int64_t thresh_ns_per_pkt_q16 = (int64_t)1000 << 16; /* 1 µs */
            if (sample_q16 > thresh_ns_per_pkt_q16 ||
                sample_q16 < -thresh_ns_per_pkt_q16) {
              gptp_resync_count++;
              sched_ns_adj_q16 = 0;
              sched_locked = false;
            } else if (!sched_locked) {
              /* Acquire: apply the warmup measurement directly — it is
               * dominated by the static crystal offset. The sample is
               * an absolute rate measurement (raw gPTP vs raw
               * esp_timer), independent of the currently applied trim.
               * Bound it to ±100 ppm: any real crystal sits well
               * inside; beyond that it is a gPTP disturbance mid-warmup,
               * so re-baseline and retry the acquire next window. */
              int64_t acquire_max_q16 =
                  ((int64_t)params->interval << 16) / 10; /* 100 ppm */
              if (sample_q16 > acquire_max_q16 ||
                  sample_q16 < -acquire_max_q16) {
                gptp_resync_count++;
              } else {
                sched_ns_adj_q16 = sample_q16;
                sched_locked = true;
              }
            } else {
              /* Track: EMA new = (old*(2^K-1) + sample) >> K, with the
               * per-update step clamped to ±1 ppm so servo noise cannot
               * modulate the wire rate faster than that. The sample is
               * absolute (see acquire above), so it is the EMA target
               * itself, not a residual on top of the current trim. */
              int64_t next_q16 =
                  ((sched_ns_adj_q16 *
                    ((1 << SCHED_FILTER_SHIFT) - 1)) +
                   sample_q16) >>
                  SCHED_FILTER_SHIFT;
              int64_t step_q16 = next_q16 - sched_ns_adj_q16;
              if (step_q16 > sched_step_max_q16)
                step_q16 = sched_step_max_q16;
              else if (step_q16 < -sched_step_max_q16)
                step_q16 = -sched_step_max_q16;
              sched_ns_adj_q16 += step_q16;
            }
            /* Track remaining_ns-like stat for diagnostics: the
             * accumulated diff is the slip over the sample window. */
            if (diff_ns >= (int64_t)INT32_MIN && diff_ns <= (int64_t)INT32_MAX) {
              int32_t diff32 = (int32_t)diff_ns;
              if (diff32 < remaining_ns_min) remaining_ns_min = diff32;
              if (diff32 > remaining_ns_max) remaining_ns_max = diff32;
            }
          }
          sched_gptp_ref_ns = now_gptp_ns;
          sched_esp_ref_us = now;
        }
        sched_pkts_since_sample = 0;
      }
    }
    /* Per-packet advance: nominal interval plus filtered adjustment,
     * with sub-µs carry. adj_q16 is ns per packet × 65536. */
    int64_t base_ns = (int64_t)params->interval * 1000LL;
    int64_t adj_ns = gptp_cadence_ready ? (sched_ns_adj_q16 >> 16) : 0;
    /* ADC-following term with its own sub-ns carry. */
    fill_frac_q16 += (int32_t)fill_adj_q16;
    int32_t fill_ns = fill_frac_q16 >> 16;
    fill_frac_q16 -= fill_ns << 16;
    int64_t total_ns = base_ns + adj_ns + fill_ns + sched_ns_carry;
    int32_t us_to_wait = (int32_t)(total_ns / 1000);
    sched_ns_carry = (int32_t)(total_ns - (int64_t)us_to_wait * 1000);
    /* A talker with gPTP-seeded stamps catches up after a stall like
     * the rate-locked one: skipping slots without advancing the stamps
     * put them behind gPTP and dropped the audio meant for those
     * slots. */
    if (!gptp_seeded &&
        overrun > params->interval * 10 + late_allowance_us) {
      next_send_time = now + params->interval;
    } else {
      next_send_time += us_to_wait;
    }

    /* Fractional samples-per-packet (Bresenham): emit spp_base or
     * spp_base+1 so the running average equals the true rate. */
    int this_spp = spp_base;
    spp_acc += spp_rem;
    if (spp_acc >= spp_den) {
      spp_acc -= spp_den;
      this_spp++;
    }
    int this_audio_len = this_spp * params->channels * 4;
    int this_sdl = is_am824 ? (TX_CIP_HDR_LEN + this_audio_len) : this_audio_len;
    int this_frame_len = audio_offset + this_audio_len;
    int this_i2s_read = this_spp * i2s_frame_bytes_pkt;
    /* Codec frames this packet takes: one per sample, or what the
     * conversion to the stream rate consumes. */
    int this_in_read =
        resample ? avb_src_inputs(&s_talker_src, this_spp, src_rate_q32) *
                       i2s_frame_bytes_pkt
                 : this_i2s_read;
    uint8_t *in_buf = resample ? src_in_buf : i2s_buf;

    /* Advance AVTP media clock by the nominal increment plus the
     * rate-lock correction (Q16 sub-ns carry). The nominal-only
     * advance accumulated residual pacing error (~35 ppm) without
     * bound — measured 86 ms of presentation-timestamp staleness on
     * the wire after 40 min, which strict listeners (macOS) answer
     * with silence while lenient ones (our pt-gate fallback, 8D)
     * keep playing. */
    ts_base_frac_q16 += (uint64_t)this_spp * ns_per_sample_q16;
    uint32_t ts_base_whole = (uint32_t)(ts_base_frac_q16 >> 16);
    ts_base_frac_q16 &= 0xFFFFULL;
    ts_rate_frac_q16 += ts_rate_corr_q16;
    int32_t ts_whole_ns = ts_rate_frac_q16 >> 16;
    ts_rate_frac_q16 -= ts_whole_ns << 16;
    avtp_media_ts += ts_base_whole + (uint32_t)ts_whole_ns;
    rate_ts_ns += (int64_t)ts_base_whole + ts_whole_ns;
    rate_samples += (uint64_t)this_spp;

    /* Check for gPTP BTC change every ~1 second (8000 packets at 125us) */
    if ((loop_count & 0x1FFF) == 0 && loop_count > 0) {
      if (memcmp(last_btc_id, state->ptp_status.clock_source_info.btc_id, 8) != 0) {
        memcpy(last_btc_id, state->ptp_status.clock_source_info.btc_id, 8);
        mr_state = !mr_state;
        mcr_hold_remaining = MCR_HOLD_PACKETS;
        tu_active = true;
        tu_hold_remaining = MCR_HOLD_PACKETS;
      }
      /* Compute gPTP-vs-esp_timer drift (ppm × 256) over the window,
       * using the baseline captured in the cadence block above. Only
       * valid if we've actually taken a gPTP reference; skipped on the
       * very first publish. */
      int32_t drift_ppm_q8 = 0;
      {
        /* One gPTP read per ~1 s window, independent of the cadence
         * filter — the previous placement inside the gptp_cadence_ready
         * path meant that with CADENCE_LOCK_INTERVAL_S=0 the drift stat
         * AND the timestamp anchor below never armed (drift printed
         * 0.00 and timestamps walked off unbounded). */
        struct timespec ts_now;
        if (ptpd_now(&ts_now) == 0) {
          int64_t now_gptp_ns = (int64_t)ts_now.tv_sec * 1000000000LL +
                                (int64_t)ts_now.tv_nsec;
          int64_t now_esp_us = esp_timer_get_time();
          bool drift_fresh = false;
          if (drift_ref_valid) {
            int64_t gptp_elapsed_ns = now_gptp_ns - drift_gptp_ref_ns;
            int64_t esp_elapsed_ns = (now_esp_us - drift_esp_ref_us) * 1000LL;
            if (esp_elapsed_ns > 100000000LL /* > 100 ms of baseline */) {
              int64_t diff_ns = gptp_elapsed_ns - esp_elapsed_ns;
              /* ppm × 256 = diff/esp_elapsed × 1e6 × 256 */
              drift_ppm_q8 =
                  (int32_t)((diff_ns * 1000000LL * 256LL) / esp_elapsed_ns);
              drift_fresh = true;
            }
          }
          /* (Re-)baseline so next window's measurement is independent. */
          drift_gptp_ref_ns = now_gptp_ns;
          drift_esp_ref_us = now_esp_us;
          drift_ref_valid = true;

          /* Presentation-timestamp rate lock. The timestamp clock must
           * advance at the true gPTP rate; packets are paced by
           * esp_timer, so the correct per-packet increment is
           * nominal × (1 + drift), with drift_ppm_q8 the rate error
           * measured directly above. Slew ts_rate_corr_q16 toward that
           * measured target with a ±0.25 ns/pkt (≈2 ppm) step per
           * window — earlier error-nulling slews flipped sign every
           * second (±40 ppm square wave) and macOS's SYT clock
           * recovery chased them into audible oscillation; a
           * rate-target lock converges one-directionally in ~15 s and
           * then holds essentially constant. A clamped ±0.5 ns/pkt
           * (≈4 ppm) offset-pull drains residual phase error; > 5 ms
           * error (BTC change, servo step, long stall) hard re-anchors
           * with tu so listeners resync. */
          /* Measure against the gPTP time this packet was scheduled
           * for. After a stall the loop runs behind its schedule while
           * it catches up; anchoring to the time it got here stamped
           * the catch-up burst ahead of its samples, then pulled back
           * at the next check (a sawtooth of re-anchors). */
          int64_t sched_gptp_ns =
              now_gptp_ns - (now_esp_us - packet_sched_us) * 1000LL;
          int32_t ts_err = (int32_t)(avtp_media_ts - (uint32_t)sched_gptp_ns);
          ts_err_last = ts_err;
          if (ts_err > 5000000 || ts_err < -5000000) {
            avtp_media_ts = (uint32_t)sched_gptp_ns;
            ts_reanchors++;
            rate_window_us = 0;
            ts_rate_corr_q16 = 0;
            ts_rate_frac_q16 = 0;
            tu_active = true;
            tu_hold_remaining = MCR_HOLD_PACKETS;
          } else if (resample) {
            /* The stamps advance by exactly the samples sent, so the
             * cadence follows them instead: a PI on the stamp error,
             * 0.05 ppm per us and 0.00128 ppm per us second, settles in
             * about 20 s with damping 0.7 and absorbs what the
             * cadence lock above leaves (30 ppm while a station warms
             * up). A stamp ahead of its gPTP slot slows the cadence. */
            int64_t window_ms = 8192LL * params->interval / 1000;
            cadence_integ_ppm_q8 += (int64_t)ts_err * window_ms * 328 /
                                    1000000000LL;
            if (cadence_integ_ppm_q8 > 150 * 256)
              cadence_integ_ppm_q8 = 150 * 256;
            else if (cadence_integ_ppm_q8 < -150 * 256)
              cadence_integ_ppm_q8 = -150 * 256;
            int64_t cadence_ppm_q8 =
                (int64_t)ts_err * 128 / 10000 + cadence_integ_ppm_q8;
            if (cadence_ppm_q8 > 200 * 256)
              cadence_ppm_q8 = 200 * 256;
            else if (cadence_ppm_q8 < -200 * 256)
              cadence_ppm_q8 = -200 * 256;
            fill_adj_q16 = (((int64_t)params->interval * 1000LL *
                             cadence_ppm_q8) << 16) /
                           (256LL * 1000000LL);
          } else {
            /* Target extra ns/pkt (Q16) from the measured rate error:
             * interval_ns × drift_ppm_q8 / (256 × 1e6). On windows
             * with no qualifying drift measurement, hold the current
             * rate rather than dragging it toward zero. */
            int64_t target_q16 = ts_rate_corr_q16;
            if (drift_fresh) {
              /* Packets are spaced (interval + sched adj) esp_timer ns
               * and each esp_timer ns is (1 + drift) gPTP ns, so the
               * stamp must advance by that same real span:
               *   extra = adj + (interval + adj) * drift
               * With the cadence lock off (adj 0) this is the old
               * interval * drift; with it on, the terms cancel to ~0.
               * Stamping interval * drift on top of an already
               * gPTP-corrected cadence double-counted the crystal
               * offset (observed: -22 ppm timestamp slide at every
               * listener). */
              int64_t cadence_adj_q16 = sched_ns_adj_q16 + fill_adj_q16;
              int64_t spacing_q16 =
                  (((int64_t)params->interval * 1000LL) << 16) + cadence_adj_q16;
              target_q16 = cadence_adj_q16 +
                           (spacing_q16 * drift_ppm_q8) / (256LL * 1000000LL);
            }
            /* Offset pull: drain ts_err at up to 8 ppm at any packet
             * interval. A fixed 0.5 ns/pkt was 4 ppm at 125 us but only
             * 2 ppm at 250 us, below the residual rate error of a Class B
             * talker, whose stamps then crept ~1.5 us/s toward the 5 ms
             * re-anchor (a glitch at every listener). */
            int64_t off_max_q16 =
                ((int64_t)params->interval * 8LL << 16) / 1000LL;
            int64_t off_q16 = -(((int64_t)ts_err) << 16) / (8192 * 4);
            if (off_q16 > off_max_q16)
              off_q16 = off_max_q16;
            else if (off_q16 < -off_max_q16)
              off_q16 = -off_max_q16;
            int64_t want = target_q16 + off_q16;
            /* Clamp the total correction to the widest cadence trim
             * (rate lock plus ADC fill, 300 ppm) at any packet
             * interval: a fixed 16 ns/pkt was 128 ppm at 125 us but
             * only 64 ppm at 250 us, so the stamps fell behind a
             * faster cadence. */
            int64_t want_max_q16 =
                ((int64_t)params->interval * 300LL << 16) / 1000LL;
            if (want > want_max_q16)
              want = want_max_q16;
            else if (want < -want_max_q16)
              want = -want_max_q16;
            /* Step toward the target by at most 2 ppm per second. The
             * window is 8192 packets, so that is 0.25 ns/pkt at 125 us
             * but 1 ns/pkt at 250 us; a fixed 0.25 ns/pkt there was
             * 0.5 ppm/s and fell behind the fill trim (observed: stamps
             * 3 ms ahead after 2 min on a talker whose ADC ran 85 ppm
             * fast). */
            int64_t step_max_q16 = ((int64_t)params->interval *
                                        params->interval * 16384LL
                                    << 16) /
                                   1000000000LL;
            int64_t step = want - ts_rate_corr_q16;
            if (step > step_max_q16)
              step = step_max_q16;
            else if (step < -step_max_q16)
              step = -step_max_q16;
            ts_rate_corr_q16 += (int32_t)step;
          }
        }
      }

      /* ADC-following cadence. The mic ring is 5 ms deep and the packet
       * cadence is gPTP-paced, so any residual between the ADC clock
       * and gPTP (a held PLL trim drifts with temperature) drains or
       * floods the ring, and every deficit becomes a zero-filled
       * packet (observed: 17 ppm -> one every 7 s). Steer the cadence
       * to what the ADC delivers instead: the backlog (captured since
       * pre-fill minus consumed) is an exact fill measurement and its
       * drift IS the rate error. PI on that integrator plant, P 4 ppm
       * per packet of error, I 1/16 ppm per packet per second, error
       * smoothed over 8 s: damping 0.7, ~4 min to absorb a 20 ppm
       * step, a 30 ppm residual (the PLL hold breaker bound) parks
       * the backlog 0.9 ms below the 2.5 ms setpoint until the
       * integrator absorbs it, and the 64 B DMA quantisation leaves
       * under 1 ppm of cadence wander at 192 kHz. The timestamp
       * rate lock above follows the same adjustment, so presentation
       * stamps stay true gPTP time and the stream's implied media
       * clock is the ADC rate. */
      if (!params->use_sine_wave) {
        /* Buffers the RX DMA queue overwrote during a stall are lost
         * audio, not backlog: counting them pinned the trim at its
         * clamp and drained the ring into zero-filled packets. */
        uint64_t captured =
            atomic_load_explicit(&state->media_clock.i2s_bytes_captured,
                                 memory_order_relaxed) -
            atomic_load_explicit(&state->media_clock.i2s_bytes_dropped,
                                 memory_order_relaxed) -
            adc_bytes_base;
        int64_t backlog_err;
        if (capture_ring) {
          /* Backlog as of this packet's slot: a packet sent late (in a
           * batch, or after a stall) finds the capture of its lateness
           * already in the ring, which is not surplus. */
          i2s_ring_head = atomic_load_explicit(
              &state->media_clock.capture_head, memory_order_acquire);
          int64_t late_us = esp_timer_get_time() - packet_sched_us;
          if (late_us < 0)
            late_us = 0;
          int64_t late_bytes = late_us * (int64_t)params->sample_rate *
                               i2s_frame_bytes / 1000000;
          backlog_err = (int64_t)(i2s_ring_head - i2s_ring_tail) -
                        late_bytes - ring_setpoint;
        } else {
          backlog_err = (int64_t)captured - (int64_t)i2s_ring_tail;
        }
        fill_err_ema_q8 += ((backlog_err << 8) - fill_err_ema_q8) >> 3;
        int64_t err_pkts_q8 = fill_err_ema_q8 / i2s_read_nominal;
        fill_integ_ppm_q8 -= err_pkts_q8 / 16;
        if (fill_integ_ppm_q8 > 150 * 256)
          fill_integ_ppm_q8 = 150 * 256;
        if (fill_integ_ppm_q8 < -150 * 256)
          fill_integ_ppm_q8 = -150 * 256;
        int64_t ppm_q8 = -err_pkts_q8 * 4 + fill_integ_ppm_q8;
        if (ppm_q8 > 200 * 256)
          ppm_q8 = 200 * 256;
        if (ppm_q8 < -200 * 256)
          ppm_q8 = -200 * 256;
        fill_ppm_q8 = (int32_t)ppm_q8;
        /* The held-trim refinement reads the rate against the stream's
         * own timestamps (below), not this trim: on a P4 the two
         * differed by a steady 6-7 ppm, so a trim refined to null this
         * one left the talker 6-9 ppm off at every listener. */
        int64_t rate_now_us = esp_timer_get_time();
        if (rate_window_us == 0) {
          rate_window_us = rate_now_us;
          rate_window_ts0 = rate_ts_ns;
          rate_window_samples0 = rate_samples;
        } else if (rate_now_us - rate_window_us >= 10000000LL) {
          int64_t span_ns = rate_ts_ns - rate_window_ts0;
          int64_t samples_ns =
              (int64_t)((rate_samples - rate_window_samples0) *
                        1000000000ULL / params->sample_rate);
          if (span_ns > 0) {
            /* Positive: more samples than the timestamps span, the ADC
             * runs fast against gPTP. ppm x 256. Converted, the stream
             * matches its stamps exactly, and the conversion ratio is
             * the ADC's error. */
            state->media_clock.talker_adc_error_ppm_q8 =
                resample ? (int32_t)-ppm_q8
                         : (int32_t)(((samples_ns - span_ns) * 256000000LL) /
                                     span_ns);
            /* Never 0, which means no reading. */
            state->media_clock.talker_adc_error_stamp_ms =
                (uint32_t)(rate_now_us / 1000) | 1;
          }
          rate_window_us = rate_now_us;
          rate_window_ts0 = rate_ts_ns;
          rate_window_samples0 = rate_samples;
        }
        /* ns per packet, Q16: interval_ns x ppm / 1e6. */
        if (resample)
          src_rate_q32 = avb_src_rate_from_ppm_q8((int32_t)ppm_q8);
        else
          fill_adj_q16 = (((int64_t)params->interval * 1000LL * ppm_q8) << 16) /
                         (256LL * 1000000LL);
      }

      /* Publish diagnostic counters for AVB-STATS (~1 Hz). Piggybacks
       * on the existing sparse branch so the hot path pays nothing. */
      stream_tx_ctx_t *tx_ctx = s_stream_tx_ctx;
      if (tx_ctx) {
        tx_ctx->pkt_count = loop_count;
        tx_ctx->send_fail_count = send_fail_count;
        tx_ctx->overrun_count = overrun_count;
        tx_ctx->overrun_max_us = overrun_max;
        tx_ctx->i2s_zero_reads = i2s_zero_reads;
        tx_ctx->i2s_nonzero_reads = i2s_nonzero_audio;
        tx_ctx->ring_underruns = ring_underruns;
        tx_ctx->fill_ppm_q8 = fill_ppm_q8;
        tx_ctx->fill_backlog_bytes = (int32_t)(fill_err_ema_q8 >> 8);
        tx_ctx->slip_events = slip_events;
        tx_ctx->slip_muted_pkts = slip_muted_pkts;
        tx_ctx->pll_offset_min_ns = pll_offset_min;
        tx_ctx->pll_offset_max_ns = pll_offset_max;
        tx_ctx->pll_skip_count = pll_skip_count;
        tx_ctx->drift_ppm_q8 = drift_ppm_q8;
        tx_ctx->remaining_ns_min = remaining_ns_min;
        tx_ctx->remaining_ns_max = remaining_ns_max;
        tx_ctx->gptp_resync_count = gptp_resync_count;
        tx_ctx->ts_reanchors = ts_reanchors;
        if (src_frames) {
          tx_ctx->src_cycles = src_cycles / src_frames;
          src_cycles = src_frames = 0;
        }
        tx_ctx->src_walks = s_talker_src.walks;
        tx_ctx->ts_err_ns = ts_err_last;
        tx_ctx->wakes = wake_count;
        tx_ctx->stalls = stall_count;
        tx_ctx->i2s_dropped_bytes =
            capture_ring
                ? (uint32_t)capture_dropped
                : (uint32_t)(atomic_load_explicit(
                                 &state->media_clock.i2s_bytes_dropped,
                                 memory_order_relaxed) -
                             i2s_dropped_base);
      }
      /* Reset PLL offset range and cadence-drift range so the published
       * values stay per-window instead of latching worst-ever. */
      pll_offset_min = INT32_MAX;
      pll_offset_max = INT32_MIN;
      remaining_ns_min = INT32_MAX;
      remaining_ns_max = INT32_MIN;
    }

    /* Update per-packet fields in tx_frame.
     * sv=1 always, mr per state, tu per gPTP status. tv per the
     * 61883-6 SYT rule below (AAF: always valid, first sample). */
    uint8_t hdr1 = 0x80; /* sv=1, version=0, mr=0 */
    if (mcr_hold_remaining > 0) {
      if (mr_state)
        hdr1 |= 0x08; /* mr=1 */
      mcr_hold_remaining--;
    }
    avtp[3] = tu_active ? 0x01 : 0x00;
    if (tu_hold_remaining > 0) {
      tu_hold_remaining--;
      if (tu_hold_remaining == 0)
        tu_active = false;
    }
    avtp[2] = seq_num++;
    /* Per-packet stream_data_length: AAF counts audio bytes, 61883
     * adds the CIP header. Varies by 1 sample on fractional rates. */
    avtp[20] = (this_sdl >> 8) & 0xFF;
    avtp[21] = this_sdl & 0xFF;
    uint32_t presentation_ts =
        avtp_media_ts + params->presentation_time_offset_ns;
    bool ts_valid = true;

    if (is_am824) {
      /* Update DBC in CIP header, and apply the IEEE 1722 61883-6
       * timestamp rule: with SYT=0xFFFF the avtp_timestamp refers to
       * the data block at the next SYT_INTERVAL boundary — the first
       * block i in this PDU with (DBC + i) mod SYT_INTERVAL == 0 —
       * NOT to the PDU's first sample; PDUs containing no boundary
       * carry tv=0. Stamping every PDU's first sample with tv=1 made
       * spec-strict listeners (macOS) place samples with a cyclic
       * error of up to SYT_INTERVAL-1 samples (24 spp vs interval 32
       * at 192 kHz → 0/8/16/invalid pattern, ±125 µs) — audible as
       * constant grainy glitching that lenient listeners never see. */
      uint8_t *cip = avtp + TX_AVTP_HDR_LEN;
      cip[3] = (uint8_t)(dbc & 0xFF);
      uint32_t phase = (uint32_t)dbc % syt_interval;
      uint32_t k = (syt_interval - phase) % syt_interval;
      if (k >= (uint32_t)this_spp) {
        ts_valid = false; /* no SYT_INTERVAL boundary in this PDU */
      } else {
        presentation_ts += k * ns_per_sample;
      }
#ifdef CONFIG_ESP_AVB_AM824_CIP_SYT
      /* MOTU convention, captured from an 8D talker on the wire: the
       * SYT field carries the constant SYT_INTERVAL value (0x0008 at
       * 48 k) in every PDU rather than 0xFFFF or a 1394 cycle
       * timestamp; presentation timing rides only in avtp_timestamp.
       * MOTU listeners appear to gate stream clockability on
       * SYT != 0xFFFF. Match it exactly, including a zeroed
       * avtp_timestamp on tv=0 PDUs. */
      cip[6] = (uint8_t)(syt_interval >> 8);
      cip[7] = (uint8_t)(syt_interval & 0xFF);
      if (!ts_valid) {
        presentation_ts = 0;
      }
#endif
      dbc += this_spp;
    }

    if (ts_valid)
      hdr1 |= 0x01; /* tv=1 */
    avtp[1] = hdr1;
    avtp[12] = (presentation_ts >> 24) & 0xFF;
    avtp[13] = (presentation_ts >> 16) & 0xFF;
    avtp[14] = (presentation_ts >> 8) & 0xFF;
    avtp[15] = presentation_ts & 0xFF;

    /* Get audio data and convert directly into tx_frame */
    uint8_t *audio_dst = tx_frame + audio_offset;
    int total_samples = this_spp * params->channels;

    if (params->use_sine_wave) {
      copy_sine_from_lut(pcm_buf, this_spp, params->channels,
                         params->bit_depth, sine_lut, lut_samples, &lut_pos);
      if (is_am824)
        be32_to_am824(pcm_buf, audio_dst, total_samples);
      else
        be32_to_aaf(pcm_buf, audio_dst, total_samples);
    } else {
      /* Read mic audio via local ring: drain the I2S DMA queue into the
       * ring on EVERY packet, consume i2s_read_size bytes per packet.
       * Refilling only when the ring ran low left the standing backlog
       * (up to the 2.5 ms pre-fill) parked in the RX DMA queue, and at
       * 192 kHz that queue (64 x 64 B) holds only 2.67 ms: the driver's
       * read loop then abandons the buffer it is on whenever the queue
       * is one slot from full ("curr_ptr is nearly to be invalid"),
       * observed as a waveform step on every 8-frame DMA boundary in
       * wire captures at 192 kHz (clean at 96 kHz, where the same queue
       * holds 5.3 ms). Keeping the backlog in this 5 ms ring instead
       * leaves the DMA queue near empty at any rate. */
      int ring_avail;
      if (capture_ring) {
        i2s_ring_head = atomic_load_explicit(&state->media_clock.capture_head,
                                             memory_order_acquire);
        /* The interrupt is about to overwrite unread audio: skip ahead
         * to the setpoint and count what was lost. */
        if (i2s_ring_head - i2s_ring_tail >
            (uint64_t)(i2s_ring_size - 2 * i2s_read_size)) {
          uint64_t resume = i2s_ring_head - ring_setpoint;
          resume -= resume % i2s_frame_bytes;
          capture_dropped += resume - i2s_ring_tail;
          i2s_ring_tail = resume;
        }
        ring_avail = (int)(i2s_ring_head - i2s_ring_tail);
        goto skip_refill;
      }
      ring_avail = (int)(i2s_ring_head - i2s_ring_tail);
      if (ring_avail < i2s_ring_size) {
        /* Refill: read as much as fits from I2S into the ring */
        int ring_space = i2s_ring_size - ring_avail;
        int write_pos = (int)(i2s_ring_head % (uint64_t)i2s_ring_size);
        int chunk = i2s_ring_size - write_pos; /* to end of buffer */
        if (chunk > ring_space)
          chunk = ring_space;
        size_t bytes_read = 0;
        /* Use short timeout — I2S DMA buffers arrive periodically.
         * If no data now, the ring has buffered audio from previous fills. */
        /* Align chunk to frame boundary (6 bytes = 1 stereo 24-bit frame)
         * to prevent partial-frame reads that desync the ring buffer */
        chunk -= chunk % (i2s_channels * i2s_bytes_per_sample);
        if (chunk == 0)
          goto skip_refill;
        i2s_channel_read(params->i2s_rx_handle, i2s_ring + write_pos, chunk,
                         &bytes_read, 0);
        /* Discard any trailing partial frame the driver might return */
        bytes_read -= bytes_read % (i2s_channels * i2s_bytes_per_sample);
        if (bytes_read > 0) {
          i2s_ring_head += bytes_read;
          i2s_nonzero_audio++;
        } else {
          i2s_zero_reads++;
        }
        ring_avail = (int)(i2s_ring_head - i2s_ring_tail);
      }
      skip_refill:
      /* Consume this_in_read bytes from ring */
      if (ring_avail >= this_in_read) {
        int read_pos = (int)(i2s_ring_tail % (uint64_t)i2s_ring_size);
        int first = i2s_ring_size - read_pos;
        if (first >= this_in_read) {
          memcpy(in_buf, i2s_ring + read_pos, this_in_read);
        } else {
          memcpy(in_buf, i2s_ring + read_pos, first);
          memcpy(in_buf + first, i2s_ring, this_in_read - first);
        }
        i2s_ring_tail += this_in_read;
      } else {
        memset(in_buf, 0, this_in_read); /* underrun — silence */
        ring_underruns++;
      }
      if (resample) {
#if CONFIG_ESP_AVB_LISTENER_MIRROR
        /* The playout runs on the codec clock too: it replaces the
         * capture before the conversion. */
        mirror_pop(src_in_buf, (uint32_t)this_in_read, mirror_target);
#endif
        uint32_t src_start = esp_cpu_get_cycle_count();
        avb_src_run(&s_talker_src, src_in_buf,
                    this_in_read / i2s_frame_bytes_pkt, i2s_buf, this_spp,
                    src_rate_q32, src_slots, i2s_frame_bytes_pkt);
        src_cycles += esp_cpu_get_cycle_count() - src_start;
        src_frames += (uint32_t)this_spp;
      }
#if CONFIG_ESP_AVB_TALKER_SLIP_MUTE
      /* Slot-slip guard. After a rate change (and less often after a
       * plain reconnect) the RX path goes through seconds-long episodes
       * in which 3-4 frame runs of the left slot are replaced by data
       * from elsewhere, heard as a dense crackle. The wire signature of
       * one slip is a PULSE: a large step (beyond 8x the running mean
       * |dx| and above a silence floor), 2-5 smooth samples of foreign
       * audio (their |dx| stays near the running mean), then a step of
       * similar size back the other way. Speech and percussive room
       * sounds fail the middle condition: their |dx| stays large across
       * the whole event (checked offline against a -25 dBFS room event
       * and every clean capture: zero detections; every corrupted
       * capture: detected within seconds). Three pulses inside the last
       * 64 ms is an episode: mute the output, hold it until 500 ms pass
       * without another episode, then fade back in over 1 ms. Fewer
       * pulses never extend the hold: a steady background of one or two
       * per window would otherwise keep the talker muted indefinitely.
       * Runs on the raw left input, before the DC block and the mute, so
       * the mute never hides the evidence from the detector. */
      {
        int pulses = 0;
        for (int f = 0; f < this_spp; f++) {
          const uint8_t *fr = i2s_buf + f * 8;
          int32_t x = ((int32_t)((fr[0] << 24) | (fr[1] << 16) | (fr[2] << 8))) >> 8;
          int32_t dx = x - slip_prev_x;
          slip_prev_x = x;
          int32_t adx = dx < 0 ? -dx : dx;
          int32_t mean_absdiff = slip_absdiff_ema_q4 >> 4;
          uint32_t since = slip_sample - slip_edge_sample;
          if (slip_armed_pkts > 400 && adx > 4096 && adx > 8 * mean_absdiff) {
            bool returns = since >= 2 && since <= 5 &&
                           ((dx < 0) != (slip_edge_dx < 0)) &&
                           adx < 2 * slip_edge_mag && adx > slip_edge_mag / 2;
            int32_t smooth_limit = mean_absdiff > 512 ? mean_absdiff : 512;
            if (returns && slip_inner_max <= 4 * smooth_limit) {
              pulses++;
              slip_edge_sample = slip_sample - 100; /* consumed */
            } else {
              slip_edge_sample = slip_sample;
              slip_edge_dx = dx;
              slip_edge_mag = adx;
              slip_inner_max = 0;
            }
          } else if (since <= 5 && adx > slip_inner_max) {
            slip_inner_max = adx;
          }
          slip_absdiff_ema_q4 += ((adx << 4) - slip_absdiff_ema_q4) >> 10;
          slip_sample++;
        }
        if (slip_armed_pkts <= 400)
          slip_armed_pkts++;
        slip_window_sum -= slip_window[slip_window_pos];
        slip_window[slip_window_pos] = (uint8_t)(pulses > 255 ? 255 : pulses);
        slip_window_sum += slip_window[slip_window_pos];
        if (++slip_window_pos >= slip_window_len)
          slip_window_pos = 0;
        if (slip_window_sum >= 3) {
          if (slip_mute_hold == 0) {
            slip_events++;
            mic_dsp.mute_q16 = 0;
            mic_dsp.mute_step = 0;
            if (slip_log_budget > 0) {
              slip_log_budget--;
              avbwarn("slot slip: %u pulses in 64 ms (mean |dx| %ld), muting",
                      (unsigned)slip_window_sum,
                      (long)(slip_absdiff_ema_q4 >> 4));
            }
          }
          slip_mute_hold = (uint32_t)(500000 / params->interval); /* 500 ms */
        }
        if (slip_mute_hold > 0) {
          slip_muted_pkts++;
          if (--slip_mute_hold == 0) {
            /* 500 ms without an episode: 1 ms fade-in. */
            mic_dsp.mute_step = (int32_t)(65536 / (params->sample_rate / 1000));
            if (mic_dsp.mute_step < 1)
              mic_dsp.mute_step = 1;
          }
        }
      }
#endif
#if CONFIG_ESP_AVB_LISTENER_MIRROR
      /* The capture above still paces the talker; the playout replaces
       * its samples. */
      if (!resample)
        mirror_pop(i2s_buf, (uint32_t)this_i2s_read, mirror_target);
      if (is_am824)
        i2s24_to_am824_stereo(i2s_buf, audio_dst, this_spp,
                              params->channels);
      else
        i2s24_to_aaf_stereo(i2s_buf, audio_dst, this_spp, params->channels);
#else
      if (is_am824)
        i2s24_to_am824_mono(i2s_buf, audio_dst, this_spp,
                            params->channels, &mic_dsp);
      else
        i2s24_to_aaf_mono(i2s_buf, audio_dst, this_spp,
                          params->channels, &mic_dsp);
#endif
      /* Test Tone control: the tone replaces the converted input, so the
       * capture ring, its pacing and the input filters run as usual and
       * the input resumes without a step when the tone stops. */
      if (state->ctrl_test_tone) {
        copy_sine_from_lut(pcm_buf, this_spp, params->channels,
                           params->bit_depth, sine_lut, lut_samples, &lut_pos);
        if (is_am824)
          be32_to_am824(pcm_buf, audio_dst, total_samples);
        else
          be32_to_aaf(pcm_buf, audio_dst, total_samples);
      }
    } /* end else (mic path) */
#if CONFIG_ESP_AVB_TALKER_TEST_PULSE
    /* The pulse replaces the source. The last gPTP read, at most about
     * 2 s old, supplies the upper bits of the 32-bit stamp. */
    if (drift_ref_valid && params->bit_depth >= 24) {
      uint32_t first_pt = avtp_media_ts + params->presentation_time_offset_ns;
      int64_t first_pt_ns = drift_gptp_ref_ns +
                            (int32_t)(first_pt - (uint32_t)drift_gptp_ref_ns);
      fill_test_pulse(pcm_buf, this_spp, params->channels, first_pt_ns,
                      ns_per_sample);
    } else {
      memset(pcm_buf, 0, (size_t)total_samples * 4);
    }
    if (is_am824)
      be32_to_am824(pcm_buf, audio_dst, total_samples);
    else
      be32_to_aaf(pcm_buf, audio_dst, total_samples);
#endif

    /* Transmit via avbnet's medium-abstracted raw send. AVB Lite
     * unicast transport: the control plane publishes per-listener
     * destination MACs; patch the DA and send one copy per listener.
     * No published DAs → one copy to the multicast template DA. */
    eth_addr_t das[CONFIG_ESP_AVB_LITE_UNICAST_FANOUT];
    int nda = avb_stream_tx_addrs_snapshot(
        &state->output_streams[params->stream_index], das);
    if (nda == 0) {
      memcpy(&das[0], &params->dest_addr, ETH_ADDR_LEN);
      nda = 1;
    }
    for (int copy = 0; copy < nda; copy++) {
      memcpy(tx_frame, &das[copy], ETH_ADDR_LEN);
      esp_err_t sent =
          avb_net_transmit_raw(params->eth_handle, tx_frame, this_frame_len);
      if (sent != ESP_OK && params->eth_handle != NULL) {
        /* One immediate retry on Ethernet. Per-listener duplication raises
         * TX-ring pressure enough that the once-per-second control-plane
         * burst (CVU declarations, beacons) collides with a stream slot —
         * measured exactly ~1 failed copy/s at fan-out 3, gone with a
         * retry. The failed frame has already drained by re-entry
         * (~3.5 µs per 426 B at 1 Gbps). On Wi-Fi a full TX queue drains
         * at air rate, so a retry only adds pressure. */
        sent = avb_net_transmit_raw(params->eth_handle, tx_frame,
                                    this_frame_len);
      }
      if (sent != ESP_OK) {
        send_fail_count++;
        last_send_error = sent;
      }
    }

    loop_count++;

    /* Feed WDT every ~125ms */
    if (wdt_handle && loop_count % 1000 == 0) {
      esp_task_wdt_reset_user(wdt_handle);
    }


    /* Accumulate productive time for this iteration. Last statement in
     * the loop so it captures everything from work_start_us onward. */
    atomic_fetch_add_explicit(&s_stream_out_work_us,
                              (uint64_t)(esp_timer_get_time() - work_start_us),
                              memory_order_relaxed);
  }

  /* Publish NULL first so AVB-STATS stops reading, then free. */
  if (s_stream_tx_ctx) {
    stream_tx_ctx_t *tx_to_free = s_stream_tx_ctx;
    tx_to_free->active = false;
    s_stream_tx_ctx = NULL;
    s_tx_frames_total += tx_to_free->pkt_count;
    s_tx_stops++;
    free(tx_to_free);
  }
  esp_log_level_set("*", ESP_LOG_INFO);
  avbinfo("Stream out stopped: %lu pkts, %lu fails (last %s), %lu overruns "
          "(max %lldus), i2s: %lu zero_reads, %lu nonzero_audio, %lu ring "
          "underruns, %lu bytes dropped, %lu re-anchors",
          loop_count, send_fail_count, esp_err_to_name(last_send_error),
          overrun_count, overrun_max, i2s_zero_reads, i2s_nonzero_audio,
          ring_underruns,
          (unsigned long)(capture_ring
                              ? capture_dropped
                              : atomic_load_explicit(
                                    &state->media_clock.i2s_bytes_dropped,
                                    memory_order_relaxed) -
                                    i2s_dropped_base),
          (unsigned long)ts_reanchors);
  avbinfo("PLL: %lu measures, %lu skipped, offset [%ldns, %ldns]",
          pll_measure_count, pll_skip_count, (long)pll_offset_min,
          (long)pll_offset_max);

err:
  esp_log_level_set("*", ESP_LOG_INFO);
#if CONFIG_ESP_AVB_STATION_TALKER_BATCHED
  /* Release the batch timer; a pending alarm then finds no task. */
  if (s_talker_task == xTaskGetCurrentTaskHandle())
    s_talker_task = NULL;
#endif
  if (wdt_handle)
    esp_task_wdt_delete_user(wdt_handle);
  free(sine_lut);
  free(pcm_buf);
  free(i2s_buf);
  if (i2s_ring && state->media_clock.capture_ring == i2s_ring) {
    /* Detach, then give a copy already in the interrupt time to end. */
    state->media_clock.capture_ring = NULL;
    vTaskDelay(1);
  }
  /* i2s_ring is kept for the next start. */
  free(tx_frame);
  free(params);
  vTaskDelete(NULL);
}

/* Stream Input (Listener) — jitter-buffered AVTP → I2S.
 *   EMAC RX handler → ring_write (non-blocking)
 *   drain task      → ring_read → i2s_channel_write
 * SPSC lock-free ring decouples bursty EMAC RX from steady I2S DMA.
 * Milan-compliant: buffers ≥ 2.126 ms. */

/* Time-scheduled packet queue for presentation-time rendering
 * (IEEE 1722 AVTP audio + Milan v1.3 §7.2): each received packet is stored
 * with its avtp_timestamp; the drain emits the packet whose
 * presentation time has come rather than in arrival order.
 *
 * Producer: audio stream RX handler (EMAC task).
 * Consumer: drain callback (drain task).
 * Single-producer / single-consumer ring of fixed-size packet slots.
 *
 * Class-agnostic: queue depth is sized for Class B's 50 ms
 * max_transit_time plus a safety margin, so the same queue works for
 * both Class A (2 ms presentation offset) and Class B (50 ms) without
 * recompilation. Packet duration and byte count are bounded per-slot
 * rather than hard-coded so multi-sample-rate or larger-packet formats
 * work too. */
/* Upper bound on samples per audio packet after listener downmix to
 * stereo. Class B at 96 kHz = 96 samples/packet (1 ms × 96 kHz); Class
 * A at 192 kHz = 24. We size the queue slots for the worst of the
 * rates we currently test with. Bump this if 192 kHz Class B streams
 * (192 sa/packet) ever need to be accepted — at the cost of queue
 * memory (queue_capacity × max_packet_bytes of SRAM). */
#define STREAM_MAX_PACKET_SAMPLES 96
#define STREAM_MAX_PACKET_BYTES   (STREAM_MAX_PACKET_SAMPLES * 2 /*ch*/ * 3 /*bytes*/)

/* Jitter ring buffer — SPSC byte FIFO. Single producer = EMAC RX handler
 * (inline in emac_rx task). Single consumer = drain esp_timer callback.
 * Capacity MUST be a power of 2 for fast index masking.
 *
 * Sized at 16 KB (~56 ms of 48 kHz stereo 24-bit audio) so that the
 * drain can absorb up to ~50 ms of flash-cache-disable from rare NVS
 * writes without underrunning. The startup prefill (smaller) is what
 * determines actual listener-internal latency. */
typedef struct {
  uint8_t *buf;
  uint32_t capacity;       /* power of 2 */
  _Atomic uint32_t head;   /* write position (producer only) */
  _Atomic uint32_t tail;   /* read position (consumer only) */
} jitter_ring_t;

#define JITTER_RING_SIZE 65536
/* FALLBACK-mode startup prefill (~28 ms at 48 kHz stereo 24-bit) —
 * used only when the presentation-time gate cannot arm (tu=1 stream,
 * no valid gPTP, stale anchor). Arrival-relative latency, sized for
 * stall absorption since fallback streams get no resync anchor. In
 * the normal presentation-gated path the operating fill is set by the
 * talker's presentation offset plus ESP_AVB_LISTENER_EXTRA_LATENCY_US,
 * and the media-clock sensor is fill-independent (DAC-consumed bytes
 * from the I2S on_sent callback — see avbcodec.c). */
#define JITTER_PREFILL   8192   /* ~28 ms at 48 kHz stereo 24-bit */
/* One marker per received packet; enough for a full ring of the
 * smallest packets in use (64 KB of 12-sample stereo packets is 683). */
#define STREAM_PT_MARKERS 1024

static inline uint32_t ring_readable(const jitter_ring_t *r) {
  return atomic_load_explicit(&r->head, memory_order_acquire) -
         atomic_load_explicit(&r->tail, memory_order_relaxed);
}

static inline uint32_t ring_writable(const jitter_ring_t *r) {
  return r->capacity - ring_readable(r);
}

static inline uint32_t ring_write(jitter_ring_t *r, const uint8_t *data,
                                  uint32_t len) {
  uint32_t avail = ring_writable(r);
  if (len > avail)
    len = avail;
  if (len == 0)
    return 0;
  uint32_t h = atomic_load_explicit(&r->head, memory_order_relaxed);
  uint32_t mask = r->capacity - 1;
  uint32_t first = r->capacity - (h & mask);
  if (first > len)
    first = len;
  memcpy(r->buf + (h & mask), data, first);
  if (len > first)
    memcpy(r->buf, data + first, len - first);
  atomic_store_explicit(&r->head, h + len, memory_order_release);
  return len;
}

static inline uint32_t ring_read(jitter_ring_t *r, uint8_t *dst, uint32_t len) {
  uint32_t avail = ring_readable(r);
  if (len > avail)
    len = avail;
  if (len == 0)
    return 0;
  uint32_t t = atomic_load_explicit(&r->tail, memory_order_relaxed);
  uint32_t mask = r->capacity - 1;
  uint32_t first = r->capacity - (t & mask);
  if (first > len)
    first = len;
  memcpy(dst, r->buf + (t & mask), first);
  if (len > first)
    memcpy(dst + first, r->buf, len - first);
  atomic_store_explicit(&r->tail, t + len, memory_order_release);
  return len;
}

/* Stream RX handler context — file-static, accessed by EMAC RX task
 * (handler, producer) and the drain task (consumer). Allocated
 * by avb_start_stream_in. All hot-path counters are plain volatile —
 * the emac_rx handler is the only writer, periodic stats task is the
 * only reader, and word-aligned 32-bit accesses are torn-free on RISC-V. */
typedef struct {
  avb_state_s *state;                        /* for media_clock stats update */
  i2s_chan_handle_t i2s_tx_handle;
  jitter_ring_t ring;                        /* SPSC byte ring, handler→drain */
  esp_timer_handle_t drain_timer;            /* wired: 1 ms drain wake */
  TaskHandle_t drain_task;                   /* see stream_in_drain_task */
  volatile bool drain_stop;
  volatile bool drain_task_exited;
  uint8_t expected_stream_id[UNIQUE_ID_LEN]; /* filter: only accept this stream */
  /* Wire-format guard: until a frame's subtype and rate code match the
   * configured input format, drop instead of ingesting. Protects a
   * provisional (superfast) start against the wire rate having changed
   * while we were down, and an explicit connect against a talker whose
   * stream doesn't match our configured format (e.g. a 44.1 kHz 61883
   * talker into a 96 kHz AAF pipeline would otherwise play
   * pitch-shifted). Two byte-compares per frame, only until first
   * match. */
  bool rate_check_pending;
  uint8_t expected_subtype;   /* AVTP subtype of the configured format */
  uint8_t expected_rate_code; /* AAF nsr or 61883 SFC, by subtype */
  volatile uint32_t rate_mismatch; /* frames dropped by the check */
  /* diagnostics — volatile, handler-writes-only, stats-reads-only */
  volatile uint32_t pkt_count;        /* total stream packets received */
  volatile uint32_t ring_write_fail;  /* ring full — packets (or partial) dropped */
  volatile uint32_t ring_write_ok;    /* packets fully written to ring */
  volatile uint32_t drain_count;      /* drain-timer ticks that emitted bytes */
  volatile uint32_t drain_underrun;   /* drain ticks that found ring empty */
  volatile uint32_t stream_id_mismatch; /* packets dropped: wrong stream_id */
  volatile uint32_t seq_num_mismatch; /* sequence number gaps */
  volatile uint32_t seq_late;         /* duplicate or late frames dropped */
  volatile uint32_t loss_fill_events; /* loss-concealment silence inserts */
  volatile uint32_t loss_fill_frames; /* wire-lost frames replaced by silence */
  volatile uint32_t splice_aligned;   /* inserts sized from presentation time */
  /* Waveform-repetition concealment (stream_conceal_fill): the period
   * repeated through the last insert, and the crossfade still owed to
   * the real audio after it. */
  uint32_t conceal_src;               /* ring position of that period */
  uint32_t conceal_produced;          /* repetition frames produced so far */
  uint16_t conceal_period;            /* frames; 0 when none */
  uint16_t conceal_crossfade;         /* real frames still to crossfade */
  bool conceal_step_pending;          /* measure the step into real audio */
  uint64_t conceal_step_sum[2];       /* |step| into and out of inserts */
  uint32_t conceal_step_count[2];
  uint32_t conceal_repeat_head;       /* ring position of the last repetition */
  /* Standing ring depth, sampled before writing each packet that
   * arrives in sequence while playout is locked (Q4 fixed point), and
   * how many samples it holds. A concealment insert may restore the
   * depth a loss took away but never push the ring past it. Sampling
   * pauses for STANDING_DEPTH_HOLD_PACKETS after every gap, so repeated
   * gaps cannot ratchet the reference up with their own inserts. */
  uint32_t standing_depth_q4;
  uint32_t standing_depth_samples;
  uint32_t standing_depth_hold;
  uint32_t ring_bytes_per_sec;        /* ring byte rate at nominal DAC rate */
  uint32_t output_delay_ns; /* DAC position to analog output at its rate */
  volatile uint32_t ts_uncertain_count; /* AVTPDU with tu=1 received */
  uint8_t last_seq_num;               /* last received sequence number */
  bool seq_num_valid;                 /* false until first packet received */
  /* Milan GET_COUNTERS state (Table 5.13). Set by the periodic main
   * loop from ring_write_ok / drain_underrun transitions, not hot path. */
  uint32_t media_locked_count;
  uint32_t media_unlocked_count;
  bool ever_locked;
  bool drain_locked; /* playout gate passed; owned by the drain callback */
  /* Presentation-time playout gate. Producer (RX handler) anchors the
   * first packet written into an EMPTY ring — its first byte sits at
   * ring tail, so its avtp_timestamp is the presentation time of the
   * next byte the drain will emit. Consumer (drain) holds playout
   * until gPTP reaches that timestamp plus the configured extra
   * latency. gate_armed is the release/acquire hand-off for the two
   * plain fields. */
  _Atomic bool gate_armed;
  uint32_t gate_avtp_ts;  /* anchor packet's avtp_timestamp (ns, 32-bit) */
  int64_t gate_arm_us;    /* esp_timer at anchor capture (staleness guard) */
  /* gate diagnostics — written once per lock by drain, read by stats */
  volatile uint8_t diag_gate_mode;   /* 0=never locked, 1=presentation, 2=prefill */
  volatile int32_t diag_gate_err_ns; /* start lateness vs target (pt mode) */
  volatile uint32_t diag_gate_fill;  /* ring fill at playout start */
  /* One-time re-gate: set after playout that locked on the prefill
   * fallback (gPTP not yet synced — early/superfast start) has been
   * re-armed onto presentation time. Prevents a re-gate loop for
   * streams that can never pt-gate (tu=1). */
  bool pt_regate_done;
  /* Set at re-gate, cleared at the next lock: suppresses the prefill
   * fallback so the (already full) ring can't instantly re-lock
   * arrival-relative before a fresh packet re-arms the pt anchor.
   * The ring-pressure escape still overrides it. */
  bool pt_await_anchor;
  int64_t pt_await_start_us; /* give-up bound for tu=1 streams */
  /* tu=1 packets that found the flushed ring empty, so could not arm the
   * anchor: the give-up's evidence. The handler counts, the drain keeps
   * the count at the start of each wait. */
  volatile uint32_t pt_unarmed_arrivals;
  uint32_t pt_await_unarmed_base;
  int64_t gate_late_since_us; /* first late anchor passed over, 0 = none */
  /* Playout latency the path has needed (the presentation phase, kept
   * by the drain while locked on presentation time): a relock and a
   * long-pause splice aim for it rather than for presentation time. */
  volatile int32_t learned_latency_ns;
  /* The lock's own lateness, see stream_in_lock_residual. */
  int32_t lock_aim_ns; /* learned latency the lock aimed for */
  int64_t lock_us;     /* esp_timer at the lock */
  int64_t lock_sum_ns;
  uint32_t lock_samples;
  int64_t lock_steps_ns; /* phase_steps_ns when the samples began */
  bool lock_residual_done;
  volatile int32_t diag_lock_residual_ns; /* last lock's, 0 until seen */
  volatile uint32_t pt_regates; /* diagnostic: re-gates performed */
  int64_t pt_regate_interval_us; /* next re-gate spacing, 0 = shortest */
  /* Timeline-validity cache: the check reads the PTP clock, which
   * contends with PTPD internals — at the drain's 1 kHz that matters
   * (the drift sampler was moved off the RX path for the same
   * reason). Re-evaluate at most every 100 ms. */
  int64_t timeline_check_us;
  bool timeline_valid_cached;
  volatile int64_t last_packet_us;    /* esp_timer at most recent packet RX */
  /* Presentation markers: ring position of each packet's first byte and
   * its avtp_timestamp, so the drain can tell the presentation time of
   * the audio at the DAC. Written by the RX handler, read by the drain. */
  struct {
    uint32_t pos;
    uint32_t pt;
  } pt_markers[STREAM_PT_MARKERS];
  _Atomic uint32_t pt_marker_count;
  uint32_t pt_marker_read; /* drain: marker at or before the DAC */
  /* Playout phase, owned by the drain: the ring position at the DAC is
   * the position at lock plus the stream bytes played since (DAC bytes
   * less starved silence) plus the ring bytes trimmed and slipped. */
  bool phase_valid;
  uint32_t phase_lock_pos;
  uint64_t phase_lock_played;
  uint32_t phase_trimmed;
  int64_t phase_check_us;
  uint32_t phase_clamps; /* dac position clamped to the ring tail */
  /* Phase steps taken out of the media-clock reference (see
   * stream_phase in avb.h): the previous sample and the counters whose
   * change explains a step. */
  bool phase_have_prev;
  int32_t phase_prev_ns;
  int64_t phase_steps_ns;
  uint64_t phase_prev_starved;
  uint32_t phase_prev_fills;
  uint32_t phase_prev_trimmed;
  uint32_t phase_prev_clamps;
  volatile uint32_t trims;      /* latency trims */
  volatile uint32_t trim_bytes; /* ring bytes the trims dropped */
  int64_t trim_window_start_us;      /* 0 = no window running */
  int32_t trim_min_ahead_us;         /* least audio buffered this window */
  /* Software rate correction (see stream_in_rate_slip): frames owed,
   * Q16 ppm-frames, and the DAC count it was last advanced to. */
  int64_t slip_owed;
  uint64_t slip_played;
  uint32_t slip_passes;
  int32_t phase_slipped; /* ring bytes: + dropped, - repeated */
  volatile uint32_t slip_drops;
  volatile uint32_t slip_repeats;
  /* A frame slip in progress (see stream_in_slip_frame): direction
   * (+1 drop, -1 repeat, 0 none), fade length and frames faded so far,
   * the ring position of the next frame to fade, and for a repeat the
   * original samples of the frame before it. */
  int8_t slip_dir;
  uint32_t slip_frames;
  uint32_t slip_done;
  uint32_t slip_pos;
  int32_t slip_prev[2];
  /* Held-clock drift slips (see stream_in_held_slip): the filtered
   * continuous phase, the phase it is held to, and timing. */
  bool held_valid;
  /* With the drain converting, the held clock follows its input by the
   * converter's rate (see stream_in_held_rate): the rate, its integral,
   * and the APLL correction when the hold began, ppm x 65536. */
  int32_t held_rate_q32;
  int64_t held_integ_ppm_q16;
  int32_t held_base_ppm_q16;
  bool held_base_valid;
  int64_t held_filt_ns;
  int64_t held_ref_ns;
  int64_t held_settle_us;
  int64_t held_last_slip_us;
  /* Phase statistics per stats window: playout time less presentation
   * time (plus the extra latency), and the audio buffered ahead of the
   * DAC. The printer sets phase_reset to start a new window. */
  volatile bool phase_reset;
  volatile int32_t phase_min_us;
  volatile int32_t phase_max_us;
  volatile int64_t phase_sum_us;
  volatile uint32_t phase_samples;
  volatile int32_t ahead_min_us;
  volatile int32_t ahead_max_us;
  /* first-packet snapshot (written by handler, printed by main loop) */
  uint8_t diag_subtype;
  uint16_t diag_sdl;
  uint8_t diag_channels;
  uint8_t diag_samples;
  uint16_t diag_i2s_bytes;
  uint8_t diag_first_audio[8];
  uint8_t diag_captured; /* 0=waiting, 1=pending print, 2=printed */
  uint32_t start_serial; /* increments per stream-in start (stats re-baseline) */
  /* Milan late/early ts counters (updated by the drift sampler in
   * main loop from drift_latest_*). Not in the hot path. */
  uint32_t late_ts_count;
  uint32_t early_ts_count;
  /* Latest-packet avtp_timestamp for the deferred drift sampler. Sparse
   * write (every 8 packets) to keep the hot path lean. Read from main
   * at 10 ms rate — 1-8 packets of staleness doesn't matter. */
  volatile uint32_t drift_latest_avtp_ts;
  volatile int64_t drift_latest_rx_ts_us;
  volatile bool drift_latest_valid;
  /* Conversion to the DAC's rate (see stream_in_write_converted): the
   * converter, ring frames copied out for it, its output and the part
   * of that output the DMA has not taken yet. src_out is NULL when the
   * drain writes the ring as it is. */
  avb_src_s src;
  uint8_t *src_in;
  uint8_t *src_out;
  bool src_primed;
  uint32_t src_out_pos;
  uint32_t src_pending;
  uint32_t src_cycles, src_frames; /* CPU cycles, frames converted */
  uint32_t src_trim_bytes; /* latency trim still to take by walks */
} stream_rx_ctx_t;

static stream_rx_ctx_t *s_stream_rx_ctx = NULL;

/* CRF stream RX context. Allocated by avb_start_stream_in when index is the
 * endpoint's CRF media-clock STREAM_INPUT. Unlike the audio stream
 * context this has no I2S/jitter-buffer machinery, just packet
 * reception state for media-clock recovery. */
typedef struct {
  avb_state_s *state; /* for media_clock stats update */
  uint8_t expected_stream_id[UNIQUE_ID_LEN]; /* filter: CRF talker's stream_id */
  uint32_t pkt_count;
  uint32_t stream_id_mismatch;
  uint32_t seq_num_mismatch;
  uint8_t last_seq_num;
  bool seq_num_valid;
  /* Last CRF timestamp observed (IEEE 1722-2016 §10, 64-bit gPTP ns).
   * Reserved for media-clock PLL. */
  uint64_t last_timestamp_ns;
  uint32_t timestamp_count;
  /* Latest-packet snapshot for the deferred drift sampler. Same
   * rationale as the audio stream variant. */
  volatile uint64_t drift_latest_crf_ts_ns;
  volatile int64_t drift_latest_rx_ts_us;
  volatile bool drift_latest_valid;
} crf_rx_ctx_t;

static crf_rx_ctx_t *s_crf_rx_ctx = NULL;

/* True once the CRF media-clock input is active and at least one valid CRF
 * timestamp has been received. Used by AECP SET_CLOCK_SOURCE so controllers
 * cannot select an input-stream clock source that is not yet usable. */
bool avb_crf_stream_valid(void) {
  return s_crf_rx_ctx != NULL && s_crf_rx_ctx->timestamp_count > 0 &&
         s_crf_rx_ctx->drift_latest_valid;
}

/* Listener drain — runs in its task (see stream_in_drain_task) at
 * least every 1 ms. Pulls whatever bytes are available from the jitter
 * ring and pushes them to I2S TX. The I2S driver paces playout at the
 * DAC rate, so the ring fill level (= playout latency) self-stabilises
 * once the PLL has locked MCLK to the talker's rate.
 *
 * This replaces the former on_sent callback drain which ran in I2S ISR
 * context and had to carry partial-packet state across DMA-descriptor
 * boundaries. Simpler drain = fewer per-frame cycles in the hot path.
 *
 * No log prints, no blocking — just ring_read + i2s_channel_write + the
 * PLL byte-counter update. */
/* True when the local PTP clock demonstrably shares the talker's
 * timeline: the last CRF timestamp received from the talker lies
 * within a small window of local PTP now (CRF carries full 64-bit
 * timestamps — no wrap ambiguity). Role-independent: works for
 * clients, for the BTC itself (whose clock_source_valid is false by
 * definition — it has no REMOTE source), and across a BTC reboot
 * while the talker re-converges onto the new timebase. Falls back to
 * clock_source_valid when no CRF is flowing. */
static bool stream_in_timeline_valid(stream_rx_ctx_t *ctx) {
  avb_state_s *state = ctx->state;
  if (!state)
    return false;
  int64_t now_us = esp_timer_get_time();
  if (ctx->timeline_check_us != 0 &&
      now_us - ctx->timeline_check_us < 100000) {
    return ctx->timeline_valid_cached;
  }
  ctx->timeline_check_us = now_us;
  ctx->timeline_valid_cached = false;
  uint32_t s1 = atomic_load_explicit(&state->media_clock.crf_anchor.seq,
                                     memory_order_acquire);
  uint64_t crf_ts = state->media_clock.crf_anchor.ts_ns;
  uint32_t s2 = atomic_load_explicit(&state->media_clock.crf_anchor.seq,
                                     memory_order_acquire);
  if (s1 == s2 && !(s1 & 1) && crf_ts != 0) {
    struct timespec now;
    if (ptpd_now(&now) != 0)
      return false;
    int64_t d = ((int64_t)now.tv_sec * 1000000000LL + now.tv_nsec) -
                (int64_t)crf_ts;
    if (d < 0)
      d = -d;
    /* Tight window: this bounds the residual clock error at re-gate
     * time, and the pt gate then WAITS out that error with the ring
     * filling — beyond ~60 ms of fill the ring-pressure escape wins
     * and playout falls back to prefill. 5 ms keeps the wait far
     * inside that budget. */
    ctx->timeline_valid_cached = d < 5000000LL;
  } else {
    ctx->timeline_valid_cached = state->ptp_status.clock_source_valid;
  }
  return ctx->timeline_valid_cached;
}

static void stream_in_drain_cb(void *arg);

/* Playout delay assumed for a codec with none measured. */
#define STREAM_IN_OUTPUT_DELAY_DEFAULT_NS (2 * 1000 * 1000)

/* Playout delay from the DAC position to the analog output at the
 * listener's rate, from the codec's measured frames. */
static uint32_t stream_in_output_delay_ns(const avb_state_s *state) {
  const avb_codec_caps_s *caps = avb_codec_get_caps(state->config.codec_type);
  uint32_t rate = state->media_clock.listener_sample_rate;
  if (!caps || rate == 0)
    return STREAM_IN_OUTPUT_DELAY_DEFAULT_NS;
  uint32_t frames = rate > 48000 ? caps->output_delay_frames_fast
                                 : caps->output_delay_frames;
  if (frames == 0)
    return STREAM_IN_OUTPUT_DELAY_DEFAULT_NS;
  return (uint32_t)(((uint64_t)frames * 1000000000ULL + rate / 2) / rate);
}

/* Frames the drain converts per pass: past a 1 ms pass at 192 kHz on a
 * wired board. */
#if portNUM_PROCESSORS == 1
#define STREAM_SRC_MAX_FRAMES 256
#else
#define STREAM_SRC_MAX_FRAMES 512
#endif
#define GATE_ANCHOR_STALE_US (2 * 1000 * 1000) /* 32-bit ns wrap guard */
/* A presentation lock fires within a drain tick of its target; an
 * anchor later than this is passed over (see the gate). */
#define GATE_LATE_NS (2 * 1000 * 1000)
#define GATE_LATE_PATIENCE_US (500 * 1000)
/* Re-gate spacing from a prefill lock, see the re-gate below. */
#define PT_REGATE_INTERVAL_MIN_US (5 * 1000 * 1000)
#define PT_REGATE_INTERVAL_MAX_US (60 * 1000 * 1000)
/* How often the drain samples the playout phase. */
#define STREAM_PHASE_INTERVAL_US 20000

/* Presentation time of the audio at ring position pos, from the
 * markers; false when no marker at or before pos is still held. */
static bool stream_in_pt_at(stream_rx_ctx_t *ctx, uint32_t pos, uint32_t *pt) {
  uint32_t count =
      atomic_load_explicit(&ctx->pt_marker_count, memory_order_acquire);
  if (count == 0 || ctx->ring_bytes_per_sec == 0)
    return false;
  /* Keep clear of the slots the handler may be overwriting. */
  uint32_t oldest =
      count > STREAM_PT_MARKERS - 8 ? count - (STREAM_PT_MARKERS - 8) : 0;
  uint32_t read = ctx->pt_marker_read;
  if ((int32_t)(read - oldest) < 0)
    read = oldest;
  if ((int32_t)(ctx->pt_markers[read % STREAM_PT_MARKERS].pos - pos) > 0)
    return false;
  while (read + 1 != count &&
         (int32_t)(ctx->pt_markers[(read + 1) % STREAM_PT_MARKERS].pos -
                   pos) <= 0)
    read++;
  ctx->pt_marker_read = read;
  uint32_t offset = pos - ctx->pt_markers[read % STREAM_PT_MARKERS].pos;
  *pt = ctx->pt_markers[read % STREAM_PT_MARKERS].pt +
        (uint32_t)(((uint64_t)offset * 1000000000ULL) /
                   ctx->ring_bytes_per_sec);
  return true;
}

/* Stream bytes the DAC has played: DMA completions less the silence it
 * sent from buffers nobody refilled. */
static uint64_t stream_in_played(const stream_rx_ctx_t *ctx) {
  avb_state_s *state = ctx->state;
  return atomic_load_explicit(&state->media_clock.i2s_bytes_written,
                              memory_order_relaxed) -
         atomic_load_explicit(&state->media_clock.i2s_tx_starved_bytes,
                              memory_order_relaxed);
}

/* Ring position of the audio at the DAC, see phase_lock_pos. */
static uint32_t stream_in_dac_pos(stream_rx_ctx_t *ctx) {
  uint32_t tail = atomic_load_explicit(&ctx->ring.tail, memory_order_relaxed);
  uint32_t played = (uint32_t)(stream_in_played(ctx) - ctx->phase_lock_played);
  uint32_t dac_pos = ctx->phase_lock_pos + played + ctx->phase_trimmed +
                     (uint32_t)ctx->phase_slipped;
  if ((int32_t)(dac_pos - tail) > 0) {
    /* Silence the starvation count missed: the DAC cannot be past the
     * ring tail. */
    ctx->phase_lock_played += (uint32_t)(dac_pos - tail);
    ctx->phase_clamps++;
    dac_pos = tail;
  }
  return dac_pos;
}

/* The DAC position now: completions move stream_in_dac_pos in whole DMA
 * buffers, a 0 to 0.5 ms sawtooth in every phase sample whose short
 * averages were biased by where the drain's wakes fell (a lock's second
 * of phase read 0.15 ms off its later mean). Add what has played of the
 * next buffer since the last completion, at most one buffer. */
static uint32_t stream_in_dac_pos_now(stream_rx_ctx_t *ctx) {
  avb_state_s *state = ctx->state;
  for (int retry = 0; retry < 4; retry++) {
    uint32_t sent_us = atomic_load_explicit(
        &state->media_clock.i2s_tx_sent_us, memory_order_acquire);
    uint32_t dac_pos = stream_in_dac_pos(ctx);
    uint32_t now_us = (uint32_t)esp_timer_get_time();
    if (atomic_load_explicit(&state->media_clock.i2s_tx_sent_us,
                             memory_order_acquire) != sent_us)
      continue; /* a completion in between */
    if (sent_us == 0)
      return dac_pos;
    uint64_t partial =
        (uint64_t)(now_us - sent_us) * ctx->ring_bytes_per_sec / 1000000ULL;
    uint32_t buffer_bytes = state->media_clock.i2s_tx_dma_buf_bytes;
    if (partial > buffer_bytes)
      partial = buffer_bytes;
    return dac_pos + (uint32_t)partial;
  }
  return stream_in_dac_pos(ctx);
}

/* Playout latency beyond presentation time: the configured extra
 * latency, plus the Wi-Fi cushion on a station. */
static inline uint32_t stream_in_extra_latency_ns(const stream_rx_ctx_t *ctx) {
  uint32_t extra_us = CONFIG_ESP_AVB_LISTENER_EXTRA_LATENCY_US;
  if (ctx->state && ctx->state->port[0].medium == avb_port_medium_wifi_ftm)
    extra_us += CONFIG_ESP_AVB_LISTENER_EXTRA_LATENCY_WIFI_US;
  return extra_us * 1000U;
}

/* Playout phase: when the audio at the DAC plays against when it should
 * (its presentation time plus the configured extra latency), in ns, and
 * the audio buffered ahead of the DAC in µs. False when unknown. */
static bool stream_in_phase(stream_rx_ctx_t *ctx, int32_t *phase_ns,
                            int32_t *ahead_us) {
  if (!ctx->phase_valid || !ctx->state)
    return false;
  uint32_t dac_pos = stream_in_dac_pos_now(ctx);
  uint32_t pt;
  struct timespec now_ts;
  if (!stream_in_pt_at(ctx, dac_pos, &pt) || ptpd_now(&now_ts) != 0)
    return false;
  uint32_t now32 = (uint32_t)((uint64_t)now_ts.tv_sec * 1000000000ULL +
                              (uint64_t)now_ts.tv_nsec);
  uint32_t due = pt + stream_in_extra_latency_ns(ctx) -
                 ctx->output_delay_ns;
  *phase_ns = (int32_t)(now32 - due);
  uint32_t head = atomic_load_explicit(&ctx->ring.head, memory_order_acquire);
  *ahead_us = (int32_t)(((uint64_t)(head - dac_pos) * 1000000ULL) /
                        ctx->ring_bytes_per_sec);
  return true;
}

/* Baseline the phase at a playout lock: the byte at the ring tail is the
 * next one the DAC plays. */
static void stream_in_phase_lock(stream_rx_ctx_t *ctx) {
  if (!ctx->state)
    return;
  ctx->phase_lock_pos = atomic_load_explicit(&ctx->ring.tail,
                                             memory_order_relaxed);
  ctx->phase_lock_played = stream_in_played(ctx);
  ctx->phase_trimmed = 0;
  ctx->phase_check_us = 0;
  ctx->phase_have_prev = false;
  ctx->trim_window_start_us = 0;
  ctx->phase_slipped = 0;
  ctx->slip_owed = 0;
  ctx->slip_played = ctx->phase_lock_played;
  ctx->slip_dir = 0;
  ctx->held_valid = false;
  ctx->lock_us = esp_timer_get_time();
  ctx->lock_samples = 0;
  ctx->lock_residual_done = false;
  ctx->diag_lock_residual_ns = 0;
  /* Converted audio still queued belongs to the playout before the
   * lock. */
  if (ctx->src_out) {
    avb_src_init(&ctx->src);
    ctx->src_primed = false;
    ctx->src_pending = 0;
    ctx->src_trim_bytes = 0;
  }
  ctx->phase_valid = true;
  atomic_fetch_add_explicit(&ctx->state->media_clock.stream_phase_epoch, 1u,
                            memory_order_relaxed);
}

/* A phase step this large between two samples 20 ms apart is never
 * clock drift (150 ppm moves 3 µs); smaller ones count as drift unless
 * an event explains them. */
#define STREAM_PHASE_STEP_NS 500000

/* Publish the playout phase to the media-clock loop with its steps
 * taken out, see stream_phase in avb.h. */
static void stream_in_phase_publish(stream_rx_ctx_t *ctx, int32_t phase_ns) {
  avb_state_s *state = ctx->state;
  uint64_t starved = atomic_load_explicit(
      &state->media_clock.i2s_tx_starved_bytes, memory_order_relaxed);
  if (ctx->phase_have_prev) {
    int32_t step_ns = phase_ns - ctx->phase_prev_ns;
    bool explained = starved != ctx->phase_prev_starved ||
                     ctx->loss_fill_events != ctx->phase_prev_fills ||
                     ctx->phase_trimmed != ctx->phase_prev_trimmed ||
                     ctx->phase_clamps != ctx->phase_prev_clamps;
    if (explained || step_ns > STREAM_PHASE_STEP_NS ||
        step_ns < -STREAM_PHASE_STEP_NS)
      ctx->phase_steps_ns += step_ns;
  }
  ctx->phase_have_prev = true;
  ctx->phase_prev_ns = phase_ns;
  ctx->phase_prev_starved = starved;
  ctx->phase_prev_fills = ctx->loss_fill_events;
  ctx->phase_prev_trimmed = ctx->phase_trimmed;
  ctx->phase_prev_clamps = ctx->phase_clamps;

  struct timespec now_ts;
  if (ptpd_now(&now_ts) != 0)
    return;
  uint64_t gptp_ns =
      (uint64_t)now_ts.tv_sec * 1000000000ULL + (uint64_t)now_ts.tv_nsec;
  int64_t continuous_ns = (int64_t)phase_ns - ctx->phase_steps_ns;
  uint32_t seq = atomic_load_explicit(&state->media_clock.stream_phase.seq,
                                      memory_order_relaxed);
  atomic_store_explicit(&state->media_clock.stream_phase.seq, seq + 1,
                        memory_order_release);
  state->media_clock.stream_phase.media_ns =
      (uint64_t)((int64_t)gptp_ns - continuous_ns);
  state->media_clock.stream_phase.gptp_ns = gptp_ns;
  state->media_clock.stream_phase.stamp_us = esp_timer_get_time();
  atomic_store_explicit(&state->media_clock.stream_phase.seq, seq + 2,
                        memory_order_release);
}

static inline int32_t ring_sample_get(const uint8_t *buf, uint32_t mask,
                                      uint32_t pos);
static inline void ring_sample_put(uint8_t *buf, uint32_t mask, uint32_t pos,
                                   int32_t sample);

/* Ring frames are two 32-bit big-endian slots holding 24 bits. */
#define STREAM_FRAME_BYTES 8

/* A trim joins the audio at the tail to the audio surplus_bytes later,
 * a step in the waveform that clicks. Crossfade the join instead: the
 * frames after the new tail fade from what would have played into what
 * now plays. Backwards, because with a trim shorter than the fade the
 * frames written are still to be read as the outgoing audio. A one
 * frame slip uses it too. */
#define STREAM_TRIM_CROSSFADE_US 2000
static void stream_trim_crossfade(stream_rx_ctx_t *ctx, uint32_t tail,
                                  uint32_t surplus_bytes, uint32_t after_bytes) {
  uint8_t *buf = ctx->ring.buf;
  uint32_t mask = ctx->ring.capacity - 1;
  uint32_t frames = (uint32_t)(((uint64_t)ctx->ring_bytes_per_sec *
                                STREAM_TRIM_CROSSFADE_US / 1000000ULL) /
                               STREAM_FRAME_BYTES);
  if (frames > after_bytes / STREAM_FRAME_BYTES)
    frames = after_bytes / STREAM_FRAME_BYTES;
  for (uint32_t frame = frames; frame-- > 0;) {
    uint32_t outgoing = tail + frame * STREAM_FRAME_BYTES;
    uint32_t incoming = outgoing + surplus_bytes;
    int64_t in_share = frame + 1;
    int64_t out_share = frames + 1 - in_share;
    for (int ch = 0; ch < 2; ch++) {
      int64_t out_sample = ring_sample_get(buf, mask, outgoing + ch * 4);
      int64_t in_sample = ring_sample_get(buf, mask, incoming + ch * 4);
      ring_sample_put(buf, mask, incoming + ch * 4,
                      (int32_t)((out_sample * out_share + in_sample * in_share) /
                                (frames + 1)));
    }
  }
}

/* Adaptive playout latency. Audio that arrives after its slot leaves
 * the DAC playing silence and playout that much later; the listener
 * keeps that latency, so the next equally late arrival plays. Once a
 * whole trim window has passed with more audio buffered at every
 * moment than STREAM_TRIM_MARGIN_US, it drops that surplus in one
 * crossfaded skip, so latency follows the path instead of keeping its
 * worst moment. Never below presentation time. Whole DMA buffers only. */
#define STREAM_TRIM_MARGIN_US 3000
static void stream_in_trim_latency(stream_rx_ctx_t *ctx, int32_t phase_ns,
                                   int32_t ahead_us, uint32_t unit) {
#if CONFIG_ESP_AVB_LISTENER_LATENCY_TRIM_S > 0 || \
    CONFIG_ESP_AVB_LISTENER_LATENCY_TRIM_WIFI_S > 0
  if (ctx->diag_gate_mode != 1 || ctx->slip_dir != 0 || ctx->src_trim_bytes)
    return;
  int64_t window_s = ctx->state->port[0].medium == avb_port_medium_wifi_ftm
                         ? CONFIG_ESP_AVB_LISTENER_LATENCY_TRIM_WIFI_S
                         : CONFIG_ESP_AVB_LISTENER_LATENCY_TRIM_S;
  if (window_s == 0)
    return;
  int64_t now_us = esp_timer_get_time();
  if (ctx->trim_window_start_us == 0 || ahead_us < ctx->trim_min_ahead_us)
    ctx->trim_min_ahead_us = ahead_us;
  if (ctx->trim_window_start_us == 0) {
    ctx->trim_window_start_us = now_us;
    return;
  }
  if (now_us - ctx->trim_window_start_us < window_s * 1000000LL)
    return;
  int32_t surplus_us = ctx->trim_min_ahead_us - STREAM_TRIM_MARGIN_US;
  if (surplus_us > phase_ns / 1000)
    surplus_us = phase_ns / 1000;
  ctx->trim_window_start_us = now_us;
  ctx->trim_min_ahead_us = ahead_us;
  if (surplus_us < 1000)
    return;
  uint32_t surplus_bytes =
      (uint32_t)(((uint64_t)surplus_us * ctx->ring_bytes_per_sec) / 1000000ULL);
  uint32_t avail = ring_readable(&ctx->ring);
  if (surplus_bytes > avail)
    surplus_bytes = avail;
  surplus_bytes -= surplus_bytes % unit;
  if (surplus_bytes == 0)
    return;
  ctx->trims++;
  ctx->trim_bytes += surplus_bytes;
  if (ctx->src_out) {
    /* Walks take the surplus, about 4 s for 1 ms, with no splice: a
     * crossfade over 1 ms of a tone clicks unless it cuts whole cycles. */
    avb_src_owe(&ctx->src, (int32_t)(surplus_bytes / STREAM_FRAME_BYTES));
    ctx->src_trim_bytes += surplus_bytes;
    return;
  }
  uint32_t tail = atomic_load_explicit(&ctx->ring.tail, memory_order_relaxed);
  stream_trim_crossfade(ctx, tail, surplus_bytes, avail - surplus_bytes);
  atomic_store_explicit(&ctx->ring.tail, tail + surplus_bytes,
                        memory_order_release);
  ctx->phase_trimmed += surplus_bytes;
#else
  (void)ctx;
  (void)phase_ns;
  (void)ahead_us;
  (void)unit;
#endif
}

/* One frame slip at the ring tail, faded in as the frames reach the
 * drain: a drop runs playout one frame ahead, a repeat one frame behind.
 * A bare slip is a step in the waveform. Over STREAM_SLIP_FADE_US each
 * frame is blended with its neighbour by a share growing from 0 to 1, a
 * fractional delay walked from 0 to 1 frame, and the frame skipped or
 * repeated where the walk ends (stream_in_slip_fade, stream_in_slip_cap
 * and stream_in_slip_land in the drain). The fade covers whatever is
 * buffered, so it is as smooth at 2 ms of playout as at 40. Returns
 * false while a slip is still in progress. */
#define STREAM_SLIP_FADE_US 10000
static bool stream_in_slip_frame(stream_rx_ctx_t *ctx, bool drop,
                                 uint32_t unit) {
  (void)unit;
  if (ctx->slip_dir != 0 || ctx->ring_bytes_per_sec == 0)
    return false;
  uint32_t frames = (uint32_t)(((uint64_t)ctx->ring_bytes_per_sec *
                                STREAM_SLIP_FADE_US / 1000000ULL) /
                               STREAM_FRAME_BYTES);
  if (frames < 1)
    frames = 1;
  ctx->slip_frames = frames;
  ctx->slip_done = 0;
  ctx->slip_pos = atomic_load_explicit(&ctx->ring.tail, memory_order_relaxed);
  ctx->slip_prev[0] = ctx->slip_prev[1] = 0;
  ctx->slip_dir = drop ? 1 : -1;
  return true;
}

/* Fade the frames of the slip in progress that have arrived, in place.
 * A drop blends frame k with frame k+1 (which must have arrived), a
 * repeat with the original frame k-1. */
static void stream_in_slip_fade(stream_rx_ctx_t *ctx) {
  if (ctx->slip_dir == 0)
    return;
  uint32_t tail = atomic_load_explicit(&ctx->ring.tail, memory_order_relaxed);
  if ((int32_t)(ctx->slip_pos - tail) < 0) {
    /* Something else moved the tail past it (a relock or a trim). */
    ctx->slip_dir = 0;
    return;
  }
  uint8_t *buf = ctx->ring.buf;
  uint32_t mask = ctx->ring.capacity - 1;
  uint32_t head = atomic_load_explicit(&ctx->ring.head, memory_order_acquire);
  int64_t frames = ctx->slip_frames;
  while (ctx->slip_done < ctx->slip_frames) {
    uint32_t pos = ctx->slip_pos;
    uint32_t need = ctx->slip_dir > 0 ? 2 * STREAM_FRAME_BYTES : STREAM_FRAME_BYTES;
    if ((int32_t)(head - pos) < (int32_t)need)
      break;
    int64_t done = ctx->slip_done;
    for (int ch = 0; ch < 2; ch++) {
      int64_t current = ring_sample_get(buf, mask, pos + ch * 4);
      int32_t blended;
      if (ctx->slip_dir > 0) {
        int64_t next = ring_sample_get(buf, mask, pos + STREAM_FRAME_BYTES + ch * 4);
        blended = (int32_t)((current * (frames - done) + next * (done + 1)) /
                            (frames + 1));
      } else {
        blended = (int32_t)((current * (frames - done) +
                             (int64_t)ctx->slip_prev[ch] * done) /
                            frames);
        ctx->slip_prev[ch] = (int32_t)current;
      }
      ring_sample_put(buf, mask, pos + ch * 4, blended);
    }
    ctx->slip_pos = pos + STREAM_FRAME_BYTES;
    ctx->slip_done++;
  }
}

/* The drain may write only faded frames while a slip runs, and must stop
 * where it lands: returns the bytes it may write from the tail. */
static uint32_t stream_in_slip_cap(const stream_rx_ctx_t *ctx, uint32_t tail,
                                   uint32_t to_read, uint32_t unit) {
  if (ctx->slip_dir == 0)
    return to_read;
  uint32_t faded = ctx->slip_pos - tail;
  if (ctx->slip_done == ctx->slip_frames)
    return to_read < faded ? to_read : faded; /* the landing point */
  if (to_read > faded)
    to_read = faded - faded % unit;
  return to_read;
}

/* At the landing point, skip the frame a drop leaves out, or play the
 * last faded-over original again for a repeat. */
static void stream_in_slip_land(stream_rx_ctx_t *ctx) {
  if (ctx->slip_dir == 0 || ctx->slip_done != ctx->slip_frames)
    return;
  uint32_t tail = atomic_load_explicit(&ctx->ring.tail, memory_order_relaxed);
  if (tail != ctx->slip_pos)
    return;
  if (ctx->slip_dir > 0) {
    if (ring_readable(&ctx->ring) < STREAM_FRAME_BYTES)
      return;
    atomic_store_explicit(&ctx->ring.tail, tail + STREAM_FRAME_BYTES,
                          memory_order_release);
    ctx->phase_slipped += STREAM_FRAME_BYTES;
    ctx->slip_drops++;
  } else {
    /* The slot behind the tail has played; the producer is not near it
     * while the ring has room to spare. */
    if (ring_writable(&ctx->ring) < 1024)
      return;
    uint32_t mask = ctx->ring.capacity - 1;
    uint32_t new_tail = tail - STREAM_FRAME_BYTES;
    for (int ch = 0; ch < 2; ch++)
      ring_sample_put(ctx->ring.buf, mask, new_tail + ch * 4, ctx->slip_prev[ch]);
    atomic_store_explicit(&ctx->ring.tail, new_tail, memory_order_release);
    ctx->phase_slipped -= STREAM_FRAME_BYTES;
    ctx->slip_repeats++;
  }
  ctx->slip_dir = 0;
}

/* Software rate correction where the audio clock cannot be tuned
 * (avb_pll_software_rate_ppm_q16): the media-clock loop's correction is
 * paid out as single-frame slips at the ring tail, dropping a frame to
 * run faster and repeating one to run slower, at the rate the loop
 * asks. Slips count in the playout phase, which is how the loop sees
 * them work. */
static void stream_in_rate_slip(stream_rx_ctx_t *ctx, uint32_t unit) {
  int32_t ppm_q16 = avb_pll_software_rate_ppm_q16();
  /* A few slips a second need no finer timing than every eighth drain
   * pass (the single-core drain runs once per DMA buffer). A held
   * clock's slips come from stream_in_held_slip instead. */
  if (ppm_q16 == 0 || ctx->state->media_clock.stream_ref_held ||
      ctx->src_out || (++ctx->slip_passes & 7) != 0)
    return;
  uint64_t played = stream_in_played(ctx);
  uint32_t frames = (uint32_t)(played - ctx->slip_played) / STREAM_FRAME_BYTES;
  ctx->slip_played += (uint64_t)frames * STREAM_FRAME_BYTES;
  if (ctx->diag_gate_mode != 1)
    return;
  const int64_t one_frame = 1000000LL << 16;
  ctx->slip_owed += (int64_t)frames * ppm_q16;
  /* Owed slips the ring could not pay stay bounded. */
  if (ctx->slip_owed > 4 * one_frame)
    ctx->slip_owed = 4 * one_frame;
  if (ctx->slip_owed < -4 * one_frame)
    ctx->slip_owed = -4 * one_frame;
  if (ctx->slip_owed >= one_frame) {
    if (stream_in_slip_frame(ctx, true, unit))
      ctx->slip_owed -= one_frame;
  } else if (ctx->slip_owed <= -one_frame) {
    if (stream_in_slip_frame(ctx, false, unit))
      ctx->slip_owed += one_frame;
  }
}

/* Held clock, converting drain: a PI from the held phase error to the
 * converter's rate, 0.03 ppm per us and 0.00046 ppm per us second, a
 * 33 s loop with damping 0.7. The clocks' difference is then paid in
 * walks at its own rate: slips on a 1.5 frame band held against the
 * drain's +/-0.25 ms phase jitter went both ways about once a second.
 * Late playout raises the rate, which takes more input per frame. The
 * APLL's moves since the hold began (its slew from following the input
 * to the held trim, 0.8 ms of phase in 20 s, and later refinements)
 * go into the rate at once. */
static void stream_in_held_rate(stream_rx_ctx_t *ctx, int64_t error_ns) {
  int32_t applied_ppm_q16 = ctx->state->media_clock.pll_applied_ppm_q16;
  /* Per phase tick: Ki x STREAM_PHASE_INTERVAL_US, ppm x 65536 per ns. */
  ctx->held_integ_ppm_q16 += error_ns * 603 / 1000000;
  if (ctx->held_integ_ppm_q16 > 150LL * 65536)
    ctx->held_integ_ppm_q16 = 150LL * 65536;
  else if (ctx->held_integ_ppm_q16 < -150LL * 65536)
    ctx->held_integ_ppm_q16 = -150LL * 65536;
  int64_t ppm_q16 = error_ns * 1966 / 1000 + ctx->held_integ_ppm_q16 +
                    (ctx->held_base_ppm_q16 - applied_ppm_q16);
  if (ppm_q16 > 200LL * 65536)
    ppm_q16 = 200LL * 65536;
  else if (ppm_q16 < -200LL * 65536)
    ppm_q16 = -200LL * 65536;
  ctx->held_rate_q32 = (int32_t)(ppm_q16 * 65536 / 1000000);
}

/* A listener whose entity streams too holds its own clock (avb_pll,
 * stream_ref_hold), so the DAC drifts against the input stream by the
 * two clocks' difference, a few ppm, and the playout ring drained into
 * an underrun or filled into a trim. Hold the playout phase with frame
 * slips instead: the continuous phase (steps taken out), filtered over
 * a few seconds, is kept within a frame and a half of where it settled;
 * a step beyond STREAM_HELD_REANCHOR_NS is an outage or trim, not
 * drift, and moves the reference. */
#define STREAM_HELD_FILTER_SHIFT 7 /* 128 phase ticks, ~2.6 s */
#define STREAM_HELD_SETTLE_US 3000000
#define STREAM_HELD_REANCHOR_NS 2000000
#define STREAM_HELD_MIN_GAP_US 40000
static void stream_in_held_slip(stream_rx_ctx_t *ctx, int64_t continuous_ns,
                                uint32_t unit) {
  avb_state_s *state = ctx->state;
  int64_t now_us = esp_timer_get_time();
  if (!state->media_clock.stream_ref_held || ctx->ring_bytes_per_sec == 0) {
    ctx->held_valid = false;
    ctx->held_rate_q32 = 0;
    ctx->held_integ_ppm_q16 = 0;
    ctx->held_base_valid = false;
    return;
  }
  if (!ctx->held_base_valid) {
    ctx->held_base_ppm_q16 = state->media_clock.pll_applied_ppm_q16;
    ctx->held_base_valid = true;
  }
  /* The error against the settled phase: zero while it settles and
   * across a step. */
  int64_t error_ns = 0;
  bool settled = false;
  if (!ctx->held_valid) {
    ctx->held_valid = true;
    ctx->held_filt_ns = continuous_ns;
    ctx->held_ref_ns = continuous_ns;
    ctx->held_settle_us = now_us + STREAM_HELD_SETTLE_US;
    ctx->held_last_slip_us = 0;
  } else {
    ctx->held_filt_ns += (continuous_ns - ctx->held_filt_ns) >>
                         STREAM_HELD_FILTER_SHIFT;
    if (now_us < ctx->held_settle_us) {
      ctx->held_ref_ns = ctx->held_filt_ns;
    } else {
      error_ns = ctx->held_filt_ns - ctx->held_ref_ns;
      settled = error_ns <= STREAM_HELD_REANCHOR_NS &&
                error_ns >= -STREAM_HELD_REANCHOR_NS;
      if (!settled) {
        ctx->held_ref_ns = ctx->held_filt_ns;
        error_ns = 0;
      }
    }
  }
  if (ctx->src_out) {
    /* Every tick: the APLL may move before the phase has settled. */
    stream_in_held_rate(ctx, error_ns);
    return;
  }
  if (!settled)
    return;
  int64_t frame_ns = (int64_t)STREAM_FRAME_BYTES * 1000000000LL /
                     ctx->ring_bytes_per_sec;
  int64_t threshold_ns = frame_ns * 3 / 2;
  if (threshold_ns < 20000)
    threshold_ns = 20000;
  if (now_us - ctx->held_last_slip_us < STREAM_HELD_MIN_GAP_US)
    return;
  /* Late playout drops a frame, early playout repeats one; the filter
   * takes the slip's known effect at once so it is not paid twice. */
  if (error_ns > threshold_ns && stream_in_slip_frame(ctx, true, unit)) {
    ctx->held_filt_ns -= frame_ns;
    ctx->held_last_slip_us = now_us;
  } else if (error_ns < -threshold_ns &&
             stream_in_slip_frame(ctx, false, unit)) {
    ctx->held_filt_ns += frame_ns;
    ctx->held_last_slip_us = now_us;
  }
}

/* A presentation lock plays late by the wait for the DMA buffer already
 * playing and a little more (0.3 to 0.7 ms measured at 48 kHz). The
 * phase loops held that lateness, the trim takes only 1 ms or more, and
 * the next lock aimed for it again as learned latency, so each relock
 * added its own (Synthia: 0.3, 1.0, 1.3, 2.0 ms over four). Once a
 * second of phase after the lock shows it, walk it out through the
 * converter, as a trim. */
#define STREAM_LOCK_SETTLE_US 200000
#define STREAM_LOCK_SAMPLES 50 /* a second of phase ticks */
#define STREAM_LOCK_RESIDUAL_MAX_NS GATE_LATE_NS
static void stream_in_lock_residual(stream_rx_ctx_t *ctx, int32_t phase_ns) {
  if (!ctx->src_out || ctx->lock_residual_done ||
      esp_timer_get_time() - ctx->lock_us < STREAM_LOCK_SETTLE_US)
    return;
  /* A starvation or slip in the samples moves the phase: start again. */
  if (ctx->lock_samples == 0 || ctx->lock_steps_ns != ctx->phase_steps_ns) {
    ctx->lock_samples = 0;
    ctx->lock_sum_ns = 0;
    ctx->lock_steps_ns = ctx->phase_steps_ns;
  }
  ctx->lock_sum_ns += phase_ns;
  if (++ctx->lock_samples < STREAM_LOCK_SAMPLES)
    return;
  ctx->lock_residual_done = true;
  int64_t late_ns = ctx->lock_sum_ns / ctx->lock_samples - ctx->lock_aim_ns;
  ctx->diag_lock_residual_ns = (int32_t)late_ns;
  if (late_ns > STREAM_LOCK_RESIDUAL_MAX_NS || ctx->src_trim_bytes)
    return;
  int32_t frames = (int32_t)(late_ns * ctx->ring_bytes_per_sec /
                             STREAM_FRAME_BYTES / 1000000000LL);
  if (frames <= 0)
    return;
  avb_src_owe(&ctx->src, frames);
  ctx->src_trim_bytes += (uint32_t)frames * STREAM_FRAME_BYTES;
}

/* Every STREAM_PHASE_INTERVAL_US while playout is locked: sample the
 * phase for the statistics and the media-clock loop, then trim surplus
 * latency. Runs before the drain's write so dropped audio never reaches
 * the DMA. */
static void stream_in_phase_tick(stream_rx_ctx_t *ctx, uint32_t unit) {
  int64_t now_us = esp_timer_get_time();
  if (now_us - ctx->phase_check_us < STREAM_PHASE_INTERVAL_US)
    return;
  ctx->phase_check_us = now_us;
  int32_t phase_ns, ahead_us;
  if (!stream_in_phase(ctx, &phase_ns, &ahead_us))
    return;
  if (ctx->diag_gate_mode == 1) {
    stream_in_phase_publish(ctx, phase_ns);
    if (ahead_us >= 1000)
      stream_in_held_slip(ctx, (int64_t)phase_ns - ctx->phase_steps_ns, unit);
    else
      ctx->held_valid = false;
    if (ahead_us >= 1000)
      stream_in_lock_residual(ctx, phase_ns);
    /* Only while audio is buffered: through an outage the DAC plays
     * silence and the phase climbs by the outage itself. */
    if (ahead_us >= 1000)
      ctx->learned_latency_ns = phase_ns < 0           ? 0
                                : phase_ns > 50000000 ? 50000000
                                                      : phase_ns;
  }
  stream_in_trim_latency(ctx, phase_ns, ahead_us, unit);
  int32_t phase_us = phase_ns / 1000;
  if (ctx->phase_reset || ctx->phase_samples == 0) {
    ctx->phase_reset = false;
    ctx->phase_min_us = ctx->phase_max_us = phase_us;
    ctx->phase_sum_us = 0;
    ctx->phase_samples = 0;
    ctx->ahead_min_us = ctx->ahead_max_us = ahead_us;
  }
  if (phase_us < ctx->phase_min_us)
    ctx->phase_min_us = phase_us;
  if (phase_us > ctx->phase_max_us)
    ctx->phase_max_us = phase_us;
  if (ahead_us < ctx->ahead_min_us)
    ctx->ahead_min_us = ahead_us;
  if (ahead_us > ctx->ahead_max_us)
    ctx->ahead_max_us = ahead_us;
  ctx->phase_sum_us += phase_us;
  ctx->phase_samples++;
}

/* Hold playout for a fresh presentation anchor, see the anchor wait in
 * the drain. */
static void stream_in_await_anchor(stream_rx_ctx_t *ctx, int64_t now_us) {
  ctx->pt_await_anchor = true;
  ctx->pt_await_start_us = now_us;
  ctx->pt_await_unarmed_base = ctx->pt_unarmed_arrivals;
}

/* Arrivals into the empty ring that the give-up needs before it decides
 * the stream cannot arm: a tu=1 stream supplies one per drain pass, a
 * talker's 8-packet tu hold around a BTC change at most 8. */
#define PT_AWAIT_UNARMED_MIN 16

/* Converts the next block of the ring for the DAC into src_out: up to
 * STREAM_SRC_MAX_FRAMES, whole DMA buffers. False when too little has
 * arrived. */
static bool stream_in_convert_block(stream_rx_ctx_t *ctx, uint32_t avail,
                                    uint32_t unit) {
  uint32_t mask = ctx->ring.capacity - 1;
  /* A frame spare: the block may take one input more than it plays. */
  uint32_t out_bytes =
      avail > STREAM_FRAME_BYTES ? avail - STREAM_FRAME_BYTES : 0;
  if (out_bytes > STREAM_SRC_MAX_FRAMES * STREAM_FRAME_BYTES)
    out_bytes = STREAM_SRC_MAX_FRAMES * STREAM_FRAME_BYTES;
  out_bytes -= out_bytes % unit;
  if (out_bytes == 0)
    return false;
  /* ppm x 65536 to the ratio's 2^-32 frames: x 2^16 / 1e6. A faster
   * DAC consumes more of the stream per frame. */
  int32_t rate_q32 = (int32_t)(
      ((int64_t)avb_pll_software_rate_ppm_q16() * 65536LL) / 1000000LL) +
      ctx->held_rate_q32;
  int out_frames = (int)(out_bytes / STREAM_FRAME_BYTES);
  int in_frames = avb_src_inputs(&ctx->src, out_frames, rate_q32);
  while (in_frames * STREAM_FRAME_BYTES > (int)avail && out_frames > 0) {
    out_frames -= (int)(unit / STREAM_FRAME_BYTES);
    in_frames = avb_src_inputs(&ctx->src, out_frames, rate_q32);
  }
  if (out_frames <= 0)
    return false;
  uint32_t in_bytes = (uint32_t)in_frames * STREAM_FRAME_BYTES;
  uint32_t tail = atomic_load_explicit(&ctx->ring.tail, memory_order_relaxed);
  uint32_t first = ctx->ring.capacity - (tail & mask);
  if (first > in_bytes)
    first = in_bytes;
  memcpy(ctx->src_in, ctx->ring.buf + (tail & mask), first);
  memcpy(ctx->src_in + first, ctx->ring.buf, in_bytes - first);
  uint32_t start = esp_cpu_get_cycle_count();
  avb_src_run(&ctx->src, ctx->src_in, in_frames, ctx->src_out, out_frames,
              rate_q32, 2, STREAM_FRAME_BYTES);
  ctx->src_cycles += esp_cpu_get_cycle_count() - start;
  ctx->src_frames += (uint32_t)out_frames;
  atomic_store_explicit(&ctx->ring.tail, tail + in_bytes,
                        memory_order_release);
  /* Frames a trim's walks take count as trimmed, which the phase
   * sensors leave out like the crossfaded trim they replace. */
  int32_t taken = (in_frames - out_frames) * STREAM_FRAME_BYTES;
  if (taken > 0 && ctx->src_trim_bytes) {
    uint32_t trimmed = (uint32_t)taken < ctx->src_trim_bytes
                           ? (uint32_t)taken
                           : ctx->src_trim_bytes;
    ctx->src_trim_bytes -= trimmed;
    ctx->phase_trimmed += trimmed;
    taken -= (int32_t)trimmed;
  }
  ctx->phase_slipped += taken;
  ctx->src_out_pos = 0;
  ctx->src_pending = (uint32_t)out_frames * STREAM_FRAME_BYTES;
  return true;
}

/* With CONFIG_ESP_AVB_LISTENER_RESAMPLE the drain plays the ring through
 * the converter: it pays the media-clock loop's correction where the DAC
 * cannot be tuned, and a held clock's slips, as walks instead of slipped
 * frames. Ring frames are consumed as they are converted, and the
 * difference from the frames produced counts in the playout phase as
 * slips do; output the DMA did not take waits for the next pass. Returns
 * the bytes written to the DAC. */
static uint32_t stream_in_write_converted(stream_rx_ctx_t *ctx,
                                          uint32_t avail, uint32_t unit) {
  if (!ctx->src_primed) {
    /* Centre the converter's window on the next frame to play: it reads
     * half a window ahead of what plays, which plays one for one, so the
     * playout phase does not move. */
    const uint32_t half = AVB_SRC_TAPS / 2 * STREAM_FRAME_BYTES;
    uint32_t mask = ctx->ring.capacity - 1;
    if (avail < half + unit)
      return 0;
    uint32_t tail = atomic_load_explicit(&ctx->ring.tail, memory_order_relaxed);
    for (uint32_t byte = 0; byte < 2 * half; byte++)
      ctx->src_in[byte] = ctx->ring.buf[(tail - half + byte) & mask];
    avb_src_prime(&ctx->src, ctx->src_in, 2, STREAM_FRAME_BYTES);
    atomic_store_explicit(&ctx->ring.tail, tail + half, memory_order_release);
    ctx->src_primed = true;
  }
  /* Fill the DMA as the ring-to-DMA path does: convert and write until
   * the DMA takes no more or too little has arrived. A pass that only
   * finished the last block's remainder left what had arrived since for
   * the next pass, up to a pass less audio queued. */
  uint32_t written_total = 0;
  for (;;) {
    if (ctx->src_pending == 0 &&
        !stream_in_convert_block(ctx, ring_readable(&ctx->ring), unit))
      break;
    size_t written = 0;
    i2s_channel_write(ctx->i2s_tx_handle, ctx->src_out + ctx->src_out_pos,
                      ctx->src_pending, &written, 1);
#if CONFIG_ESP_AVB_LISTENER_MIRROR
    mirror_push(ctx->src_out + ctx->src_out_pos, (uint32_t)written);
#endif
    ctx->src_out_pos += (uint32_t)written;
    ctx->src_pending -= (uint32_t)written;
    written_total += (uint32_t)written;
    if (ctx->src_pending)
      break; /* the DMA is full */
  }
  return written_total;
}

/* The drain's task. Single-core stations: the 1 ms esp_timer drain ran
 * below the Wi-Fi task, which also runs this station's own receive
 * path, so during a burst the drain waited past the ~2 ms of audio
 * queued in the TX DMA and the DAC played silence without consuming the
 * ring (measured ~0.1 % of playback, and the ring then overflowed). This
 * task sits above Wi-Fi and is woken by each TX DMA completion; its work
 * is a few microseconds a millisecond.
 * Wired boards: the timer task shares core 0 and its priority with
 * Ethernet receive, and its passes came up to 1.33 ms apart while a
 * whole DMA buffer of arrived audio waited for the next one. Playing
 * about 1 ms after arrival, the DAC starved a buffer every minute or
 * two. This task sits above receive, which wakes it as each half
 * millisecond of audio arrives; a 1 ms timer wakes it in between. */
static void stream_in_drain_task(void *arg) {
  stream_rx_ctx_t *ctx = (stream_rx_ctx_t *)arg;
  while (!ctx->drain_stop) {
    ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(20));
    if (ctx->drain_stop)
      break;
    stream_in_drain_cb(ctx);
  }
  ctx->drain_task_exited = true;
  vTaskDelete(NULL);
}

static void stream_in_drain_task_stop(stream_rx_ctx_t *ctx) {
  ctx->drain_stop = true;
  xTaskNotifyGive(ctx->drain_task);
  while (!ctx->drain_task_exited)
    vTaskDelay(1);
}

#if portNUM_PROCESSORS > 1
static void stream_in_drain_wake_cb(void *arg) {
  xTaskNotifyGive(((stream_rx_ctx_t *)arg)->drain_task);
}

/* From the receive path: wake the drain each time another half
 * millisecond, and at least a whole DMA buffer, is ready to write. A
 * wake per 24-frame buffer at 192 kHz (8000 a second) starved gPTP on
 * core 0 within two seconds. */
static inline void stream_in_wake_drain(stream_rx_ctx_t *ctx,
                                        uint32_t before, uint32_t total) {
  TaskHandle_t drain = ctx->drain_task;
  if (!drain || ctx->drain_stop)
    return;
  uint32_t unit = ctx->state->media_clock.i2s_tx_dma_buf_bytes
                      ? ctx->state->media_clock.i2s_tx_dma_buf_bytes
                      : 8;
  uint32_t ready = ctx->ring_bytes_per_sec / 2000;
  ready -= ready % unit;
  if (ready < unit)
    ready = unit;
  if (ctx->src_out)
    before -= before < STREAM_FRAME_BYTES ? before : STREAM_FRAME_BYTES;
  if ((before + total) / ready != before / ready)
    xTaskNotifyGive(drain);
}
#endif

/* Drain cost per stats window (diagnostic, see avb_stream_in_drain_prof):
 * calls, CPU cycles in the whole callback and in its I2S writes. */
static uint32_t s_drain_prof_calls, s_drain_prof_cycles, s_drain_prof_write_cycles;

void avb_stream_in_drain_prof(uint32_t *calls, uint32_t *cycles,
                              uint32_t *write_cycles) {
  *calls = s_drain_prof_calls;
  *cycles = s_drain_prof_cycles;
  *write_cycles = s_drain_prof_write_cycles;
  s_drain_prof_calls = s_drain_prof_cycles = s_drain_prof_write_cycles = 0;
}

static void stream_in_drain_body(void *arg);
/* Longest wait between drain passes this stats window: the DMA queue
 * holds only what has arrived, so a late pass starves the DAC. */
static int64_t s_drain_last_us;
static volatile uint32_t s_drain_gap_max_us;

static void stream_in_drain_cb(void *arg) {
  uint32_t start = esp_cpu_get_cycle_count();
  int64_t now_us = esp_timer_get_time();
  if (s_drain_last_us && now_us - s_drain_last_us > s_drain_gap_max_us)
    s_drain_gap_max_us = (uint32_t)(now_us - s_drain_last_us);
  s_drain_last_us = now_us;
  stream_in_drain_body(arg);
  s_drain_prof_cycles += esp_cpu_get_cycle_count() - start;
  s_drain_prof_calls++;
}

/* Empty ring is NORMAL at thin (presentation-time) fill: the standing
 * buffer is only the talker's transit margin (~2 ms), so a 1 ms drain
 * tick routinely drains to zero between packet arrivals while the I2S
 * DMA (2 ms) rides through. Only treat it as a lost stream — unlock and
 * re-arm the presentation anchor so the next packet re-gates at its own
 * presentation time — when packets have stopped for longer than the
 * loss-concealment splice can repair. The splice must fit outage +
 * transit lead (~12 ms) into the 64 KB ring (≈85 ms at 96 kHz), so
 * unlock at 65 ms. Shorter outages stay locked: the DAC free-runs
 * auto_clear silence and the resume packet splices back in at its exact
 * presentation time, with no flush or gate wait. */
static void stream_in_drain_empty(stream_rx_ctx_t *ctx) {
  int64_t since_pkt_us = esp_timer_get_time() - ctx->last_packet_us;
  if (since_pkt_us <= 65000)
    return;
  ctx->drain_underrun++;
  ctx->drain_locked = false;
  ctx->gate_late_since_us = 0;
  /* Drop the remainder so the next packet finds the ring empty and can
   * anchor the gate. */
  atomic_store_explicit(
      &ctx->ring.tail,
      atomic_load_explicit(&ctx->ring.head, memory_order_acquire),
      memory_order_release);
  atomic_store_explicit(&ctx->gate_armed, false, memory_order_release);
  if (ctx->ever_locked) {
    ctx->media_unlocked_count++;
    ctx->ever_locked = false;
  }
}

static void stream_in_drain_body(void *arg) {
  stream_rx_ctx_t *ctx = (stream_rx_ctx_t *)arg;
  if (!ctx)
    return;

  /* Playout gate. Primary mode: presentation-time-driven — hold the
   * drain until gPTP reaches the anchor packet's avtp_timestamp plus
   * the configured extra latency (minus the output-pipeline delay).
   * The byte at ring tail then hits the DAC at its presentation time
   * + EXTRA_LATENCY, deterministically: every listener with the same
   * setting plays in sample-accurate sync. Fallback mode (no anchor:
   * tu=1 stream, no valid gPTP, stale anchor, or ring pressure):
   * byte-prefill gate, which gives arrival-relative latency only.
   * Gate state lives in ctx so a stream restart re-gates freshly. */
  if (!ctx->drain_locked) {
    /* Post-re-gate anchor wait: the anchor only arms on a packet
     * arriving into an EMPTY ring (its timestamp must describe the
     * ring tail). Keep the ring flushed until that happens so the
     * next packet arms with correct alignment; give up after 100 ms
     * in which tu=1 packets kept reaching the flushed ring without
     * arming (tu=1 streams never arm) and fall back to prefill. Packets
     * that land before the first flush cannot arm, and a pause with no
     * packets at all proves nothing (a Wi-Fi talker stalls longer than
     * that), so neither counts: keep waiting. */
    if (ctx->pt_await_anchor &&
        !atomic_load_explicit(&ctx->gate_armed, memory_order_acquire)) {
      if (esp_timer_get_time() - ctx->pt_await_start_us > 100000 &&
          ctx->pt_unarmed_arrivals - ctx->pt_await_unarmed_base >=
              PT_AWAIT_UNARMED_MIN) {
        /* Packets arrived for 100 ms into the flushed ring and none
         * armed an anchor: tu=1 stream (or equivalent), pt-gating is
         * impossible, stop re-gate attempts for good. */
        ctx->pt_await_anchor = false;
        ctx->pt_regate_done = true;
      } else {
        uint32_t h =
            atomic_load_explicit(&ctx->ring.head, memory_order_acquire);
        atomic_store_explicit(&ctx->ring.tail, h, memory_order_release);
        return;
      }
    }
    uint32_t fill = ring_readable(&ctx->ring);
    if (fill == 0)
      return;
    bool start = false;
    bool pt_mode = false;
    int32_t pt_err_ns = 0;
    if (atomic_load_explicit(&ctx->gate_armed, memory_order_acquire)) {
      struct timespec now_ts;
      int64_t anchor_age_us = esp_timer_get_time() - ctx->gate_arm_us;
      if (anchor_age_us > GATE_ANCHOR_STALE_US) {
        /* Anchor older than half the 32-bit ns wrap: comparison is
         * ambiguous. Fall back to prefill. */
        if (fill >= JITTER_PREFILL)
          start = true;
      } else if (stream_in_timeline_valid(ctx) && ptpd_now(&now_ts) == 0) {
        /* Timeline-validity gate: ptpd_now() succeeding does NOT
         * mean the clock shares the talker's timeline — before sync
         * it runs free, and comparing presentation times against it
         * "starts" seconds late with an arbitrary offset (observed
         * err=1.78 s on an early/superfast boot). Not yet valid →
         * prefill fallback below; the re-gate after the gate block
         * moves playout onto presentation time once it becomes so. */
        uint32_t now32 = (uint32_t)((uint64_t)now_ts.tv_sec * 1000000000ULL +
                                    (uint64_t)now_ts.tv_nsec);
        /* The learned latency carries over a relock, so an outage does
         * not throw away what the path has shown it needs. */
        uint32_t target = ctx->gate_avtp_ts + stream_in_extra_latency_ns(ctx) -
                          ctx->output_delay_ns +
                          (uint32_t)ctx->learned_latency_ns;
        int32_t diff = (int32_t)(now32 - target);
        int64_t now_us = esp_timer_get_time();
        if (diff > GATE_LATE_NS && diff < 50000000 &&
            (ctx->gate_late_since_us == 0 ||
             now_us - ctx->gate_late_since_us < GATE_LATE_PATIENCE_US)) {
          /* Late anchor, typically the head of a catch-up burst after
           * an outage: starting on it would play everything that late
           * from now on. Re-arm on a fresher packet; lateness falls
           * packet by packet through the burst. A path that is simply
           * later than the talker's offset never offers an on-time
           * anchor, so after GATE_LATE_PATIENCE_US the next late one
           * is taken. */
          if (ctx->gate_late_since_us == 0)
            ctx->gate_late_since_us = now_us;
          atomic_store_explicit(&ctx->gate_armed, false,
                                memory_order_release);
          stream_in_await_anchor(ctx, now_us);
          return;
        }
        if (diff >= 0 && diff < 50000000) {
          /* Target reached. A genuine pt lock fires with diff near
           * zero — the drain polls every 1 ms — so a large positive
           * diff means the anchor was stamped in a different
           * timeline than the clock now agrees with (observed
           * +1.74 s when a stream frame anchored just before the
           * talker re-synced to a rebooted BTC, and the CRF
           * validity check passed just after). */
          start = true;
          pt_mode = true;
          pt_err_ns = diff;
          ctx->lock_aim_ns = ctx->learned_latency_ns;
        } else if (diff >= 0) {
          /* Stale/mis-timeline anchor: discard it and hold for a
           * fresh packet to re-arm on an empty ring (via the
           * anchor-await flush loop above). */
          atomic_store_explicit(&ctx->gate_armed, false,
                                memory_order_release);
          stream_in_await_anchor(ctx, esp_timer_get_time());
          return;
        }
      } else if (fill >= JITTER_PREFILL && !ctx->pt_await_anchor) {
        start = true; /* no gPTP yet — arrival-relative fallback */
      }
    } else if (fill >= JITTER_PREFILL && !ctx->pt_await_anchor) {
      start = true; /* no trustworthy anchor — arrival-relative fallback */
    }
    /* Ring-pressure escape: never let the gate hold so long that the
     * producer starts dropping. */
    if (!start && fill > (3 * ctx->ring.capacity) / 4)
      start = true;
    if (!start)
      return;
    ctx->pt_await_anchor = false;
    ctx->gate_late_since_us = 0;
    ctx->drain_locked = true;
    if (!ctx->ever_locked) {
      ctx->ever_locked = true;
    }
    ctx->media_locked_count++;
    /* A presentation lock fires up to a drain tick late (measured
     * 0.76 ms): skip that much audio before any of it plays. Played
     * late, it stayed late until the first latency trim spliced it out a
     * minute in, a click on any tone it cut mid-cycle. */
    if (pt_mode && pt_err_ns > 0 && ctx->ring_bytes_per_sec) {
      uint32_t late_bytes = (uint32_t)((uint64_t)pt_err_ns *
                                       ctx->ring_bytes_per_sec / 1000000000ULL);
      if (late_bytes > fill)
        late_bytes = fill;
      late_bytes -= late_bytes % STREAM_FRAME_BYTES;
      uint32_t tail = atomic_load_explicit(&ctx->ring.tail, memory_order_relaxed);
      atomic_store_explicit(&ctx->ring.tail, tail + late_bytes,
                            memory_order_release);
    }
    stream_in_phase_lock(ctx);
    /* The gate wait moved consumption relative to arrival: tell the
     * stream-referenced PLL to re-seed rather than read it as rate. */
    if (ctx->state)
      atomic_fetch_add_explicit(&ctx->state->media_clock.stream_ref_epoch, 1u,
                                memory_order_relaxed);
    /* No logging here (drain task) — stash for the stats printer. */
    ctx->diag_gate_mode = pt_mode ? 1 : 2;
    if (pt_mode) {
      ctx->pt_regate_interval_us = 0;
      ctx->pt_regate_done = false; /* this stream can arm after all */
    }
    ctx->diag_gate_err_ns = pt_err_ns;
    ctx->diag_gate_fill = fill;
  }

  /* Re-gate onto presentation time. Playout that locked on the
   * prefill fallback (gPTP not yet sharing the talker's timeline —
   * early/superfast start, or a BTC reboot) plays arrival-relative,
   * out of the sample-accurate timeline the other listeners share.
   * Once the timeline is valid, unlock and re-arm the anchor so the
   * next packet re-gates at its true presentation time. Each attempt
   * costs a ~fill-target pause; retries start 5 s apart in case an
   * attempt loses to the ring-pressure escape (residual clock error at
   * re-gate time makes the pt target sit in the future while the ring
   * fills), and back off to 60 s while they keep losing (a station
   * talker's clock can sit 100 ms ahead for minutes after it boots).
   * pt_regate_done is set permanently by the anchor-await give-up
   * above — streams that cannot arm an anchor (tu=1) must not pay a
   * periodic pause forever. */
  int64_t regate_interval_us = ctx->pt_regate_interval_us
                                   ? ctx->pt_regate_interval_us
                                   : PT_REGATE_INTERVAL_MIN_US;
  if (ctx->drain_locked && ctx->diag_gate_mode == 2 && !ctx->pt_regate_done &&
      stream_in_timeline_valid(ctx) &&
      (ctx->pt_await_start_us == 0 ||
       esp_timer_get_time() - ctx->pt_await_start_us > regate_interval_us)) {
    ctx->pt_regate_interval_us =
        regate_interval_us * 2 > PT_REGATE_INTERVAL_MAX_US
            ? PT_REGATE_INTERVAL_MAX_US
            : regate_interval_us * 2;
    stream_in_await_anchor(ctx, esp_timer_get_time());
    ctx->pt_regates++;
    ctx->drain_locked = false;
    ctx->gate_late_since_us = 0;
    atomic_store_explicit(&ctx->gate_armed, false, memory_order_release);
    if (ctx->ever_locked) {
      ctx->media_unlocked_count++;
      ctx->ever_locked = false;
    }
    return;
  }

  /* Whole DMA buffers only: the driver counts a starved DMA's silence
   * only while no partly written buffer is pending, and the playout
   * phase relies on that count. A remainder under one buffer waits for
   * the next packet. */
  uint32_t unit = ctx->state && ctx->state->media_clock.i2s_tx_dma_buf_bytes
                      ? ctx->state->media_clock.i2s_tx_dma_buf_bytes
                      : 8;
  stream_in_phase_tick(ctx, unit);
  stream_in_rate_slip(ctx, unit);
  stream_in_slip_land(ctx);
  stream_in_slip_fade(ctx);
  uint32_t avail = ring_readable(&ctx->ring);
  if (avail < unit) {
    stream_in_drain_empty(ctx);
    return;
  }

  /* Cap per-tick drain so we don't push unbounded audio into I2S DMA
   * in one call. Must exceed one tick of media at the highest rate —
   * 192 kHz stereo 24-bit needs 1152 B/ms; 1024 capped consumption at
   * 170.7 k samples/s and the ring shed ~11 % of every 192 kHz stream.
   * 4096 = ~3.5 ms at 192 kHz (matches the old headroom at 48 kHz). */
  #define DRAIN_MAX_BYTES 4096
  uint32_t to_read = avail;
  if (to_read > DRAIN_MAX_BYTES)
    to_read = DRAIN_MAX_BYTES;
  to_read -= to_read % unit;
  if (to_read == 0)
    return;

  size_t bytes_written = 0;
  if (ctx->src_out) {
    uint32_t write_start = esp_cpu_get_cycle_count();
    bytes_written = stream_in_write_converted(ctx, avail, unit);
    s_drain_prof_write_cycles += esp_cpu_get_cycle_count() - write_start;
    /* The converter takes a frame more than it plays and half its
     * window to prime, so it can leave a DMA buffer's worth it never
     * plays: an empty ring too (observed: a 4 min outage that never
     * unlocked, and playout 10 ms early once packets came back). */
    if (bytes_written == 0 && ctx->src_pending == 0) {
      stream_in_drain_empty(ctx);
      return;
    }
    goto written;
  }
  /* Zero-copy drain: i2s_channel_write straight from the ring buffer in
   * up to two contiguous segments (split at ring wrap). Saves the
   * intermediate ring_read copy. */
  uint32_t t = atomic_load_explicit(&ctx->ring.tail, memory_order_relaxed);
  to_read = stream_in_slip_cap(ctx, t, to_read, unit);
  if (to_read == 0)
    return;
  uint32_t mask = ctx->ring.capacity - 1;
  uint32_t first_chunk = ctx->ring.capacity - (t & mask);
  if (first_chunk > to_read)
    first_chunk = to_read;
  uint32_t second_chunk = to_read - first_chunk;

  size_t w1 = 0;
  uint32_t write_start = esp_cpu_get_cycle_count();
  i2s_channel_write(ctx->i2s_tx_handle, ctx->ring.buf + (t & mask),
                    first_chunk, &w1, 1);
  bytes_written += w1;
#if CONFIG_ESP_AVB_LISTENER_MIRROR
  mirror_push(ctx->ring.buf + (t & mask), (uint32_t)w1);
#endif
  /* Only attempt the wrap-around segment if the first fully drained —
   * otherwise I2S DMA is backpressured; next tick picks it up. */
  if (w1 == first_chunk && second_chunk > 0) {
    size_t w2 = 0;
    i2s_channel_write(ctx->i2s_tx_handle, ctx->ring.buf, second_chunk,
                      &w2, 1);
    bytes_written += w2;
#if CONFIG_ESP_AVB_LISTENER_MIRROR
    mirror_push(ctx->ring.buf, (uint32_t)w2);
#endif
  }
  s_drain_prof_write_cycles += esp_cpu_get_cycle_count() - write_start;
  atomic_store_explicit(&ctx->ring.tail, t + (uint32_t)bytes_written,
                        memory_order_release);
  stream_in_slip_land(ctx);
written:
  ctx->drain_count++;
  /* Stream-reference pair for the media-clock PLL, see avb.h. */
  if (ctx->state && bytes_written > 0) {
    avb_state_s *state = ctx->state;
    uint64_t written_now = atomic_load_explicit(
        &state->media_clock.i2s_bytes_written, memory_order_relaxed);
    uint64_t credited_now = atomic_load_explicit(
        &state->media_clock.stream_bytes_received, memory_order_relaxed);
    uint32_t seq = atomic_load_explicit(&state->media_clock.stream_anchor.seq,
                                        memory_order_relaxed);
    atomic_store_explicit(&state->media_clock.stream_anchor.seq, seq + 1,
                          memory_order_release);
    state->media_clock.stream_anchor.written = written_now;
    state->media_clock.stream_anchor.credited = credited_now;
    state->media_clock.stream_anchor.stamp_us = esp_timer_get_time();
    atomic_store_explicit(&state->media_clock.stream_anchor.seq, seq + 2,
                          memory_order_release);
  }

  /* The media-clock PLL counter is fed by the I2S TX on_sent DMA
   * callback (avbcodec.c), not here: bytes-into-DMA only equals the
   * local playout rate while the DMA is backpressured, which fails at
   * thin (presentation-time) fill levels. */
}

/* CRF stream RX handler — counts CRF AVTPDUs and records the most
 * recent media-clock timestamp. Called inline from EMAC RX task via
 * the dispatcher; must return quickly. Timestamps are collected for
 * a future media-clock PLL. */
static void avb_crf_rx_handler(uint8_t *avtp_data, uint16_t len,
                               crf_rx_ctx_t *ctx) {
  int64_t rx_ts_us = esp_timer_get_time();
  if (!ctx || len < 28)
    return;

  ctx->pkt_count++;

  /* Track sequence number gaps (avtp_data[2] = sequence_num) */
  uint8_t seq_num = avtp_data[2];
  if (ctx->seq_num_valid) {
    uint8_t expected = ctx->last_seq_num + 1;
    if (seq_num != expected)
      ctx->seq_num_mismatch++;
  }
  ctx->last_seq_num = seq_num;
  ctx->seq_num_valid = true;

  /* CRF AVTPDU layout (IEEE 1722-2016 §10.4, Figure 13):
   *   bytes 0-3:   subtype, sv|version|mr, seq_num, type_specific
   *   bytes 4-11:  stream_id
   *   bytes 12-15: pull(3) | base_frequency(29)
   *   bytes 16-17: crf_data_length
   *   bytes 18-19: timestamp_interval
   *   bytes 20+:   CRF timestamps, 8 bytes each (big-endian uint64 ns)
   * For Milan's 1-timestamp-per-PDU format the 8 bytes at offset 20 hold
   * the single sample-time timestamp. */
  if (len < 28)
    return;
  uint64_t timestamp_ns = 0;
  for (int i = 0; i < 8; i++)
    timestamp_ns = (timestamp_ns << 8) | avtp_data[20 + i];
  ctx->last_timestamp_ns = timestamp_ns;
  ctx->timestamp_count++;

  /* Publish a (crf_ts, bytes_written) anchor for the PLL (seqlock).
   * Done before the drift log below so the snapshot captures
   * bytes_written as close to CRF arrival as possible. */
  if (ctx->state) {
    avb_state_s *state = ctx->state;
    uint64_t bytes_snapshot = atomic_load_explicit(
        &state->media_clock.i2s_bytes_written, memory_order_acquire);
    uint32_t seq = atomic_load_explicit(
        &state->media_clock.crf_anchor.seq, memory_order_relaxed);
    atomic_store_explicit(&state->media_clock.crf_anchor.seq, seq + 1,
                          memory_order_release); /* odd: write in progress */
    state->media_clock.crf_anchor.ts_ns = timestamp_ns;
    state->media_clock.crf_anchor.bytes = bytes_snapshot;
    atomic_store_explicit(&state->media_clock.crf_anchor.seq, seq + 2,
                          memory_order_release); /* even: complete */
  }

  /* Stash the latest CRF timestamp + arrival esp_timer value for the
   * deferred drift sampler. Same rationale as the audio stream path:
   * clock_gettime(CLOCK_PTP_SYSTEM) goes into the PTP clock driver
   * and was contending with PTPD internals. */
  ctx->drift_latest_crf_ts_ns = timestamp_ns;
  ctx->drift_latest_rx_ts_us = rx_ts_us;
  ctx->drift_latest_valid = true;
}

/* Stream RX handler — called inline from EMAC RX task for each
 * VLAN-tagged AVTP packet. Parses AVTP, converts audio to stereo 24-bit
 * LE (I2S slot format), writes to the jitter ring. Must return quickly:
 * at 8000 pps any µs spent here subtracts directly from emac_rx's DMA
 * drain budget, and going over budget means the NIC overflows its RX
 * descriptor ring and drops frames at hardware level.
 *
 * No `ESP_LOG*` calls here — all diagnostics via `volatile` counters the
 * main loop samples. No per-packet `esp_timer_get_time()`, no
 * `clock_gettime()`, no per-packet allocation. */
/* Loss concealment by waveform repetition: instead of silence, an insert
 * repeats the last pitch period of the audio before it, fading to
 * silence over 20 ms, and the real audio after it is crossfaded in over
 * 1 ms. Ring frames are two 32-bit big-endian slots holding 24 bits. */
#define CONCEAL_FRAME_BYTES 8
#define CONCEAL_PERIOD_MIN 48   /* 1 ms at 48 kHz */
#define CONCEAL_PERIOD_MAX 480  /* 10 ms */
#define CONCEAL_MATCH_FRAMES 128 /* compared every 4th frame */
#define CONCEAL_LAG_STEP 4      /* 83 us at 48 kHz */
#define CONCEAL_FADE_FRAMES 960 /* 20 ms */
#define CONCEAL_CROSSFADE_FRAMES 48
/* Reciprocals of the fade and crossfade lengths (Q22, Q20), so a gain
 * is a multiply and a shift: these loops run inline in emac_rx, where a
 * 64-bit division per sample cost a whole packet interval per insert. */
#define CONCEAL_FADE_RECIP_Q22 ((1u << 22) / CONCEAL_FADE_FRAMES)
#define CONCEAL_CROSSFADE_RECIP_Q20 ((1u << 20) / CONCEAL_CROSSFADE_FRAMES)
/* In-sequence packets sampled before the standing depth caps inserts,
 * and in-sequence packets after a gap before sampling resumes (about
 * 30-60 ms). */
#define STANDING_DEPTH_MIN_SAMPLES 64
#define STANDING_DEPTH_HOLD_PACKETS 256
#if CONFIG_ESP_AVB_LISTENER_CONCEAL_REPEAT
static volatile bool s_conceal_repeat = true;
#else
static volatile bool s_conceal_repeat;
#endif

static inline int32_t ring_sample_get(const uint8_t *buf, uint32_t mask,
                                      uint32_t pos) {
  return (int32_t)(((uint32_t)buf[pos & mask] << 24) |
                   ((uint32_t)buf[(pos + 1) & mask] << 16) |
                   ((uint32_t)buf[(pos + 2) & mask] << 8)) >> 8;
}

static inline void ring_sample_put(uint8_t *buf, uint32_t mask, uint32_t pos,
                                   int32_t sample) {
  if (sample > 0x7fffff)
    sample = 0x7fffff;
  else if (sample < -0x800000)
    sample = -0x800000;
  buf[pos & mask] = (uint8_t)(sample >> 16);
  buf[(pos + 1) & mask] = (uint8_t)(sample >> 8);
  buf[(pos + 2) & mask] = (uint8_t)sample;
  buf[(pos + 3) & mask] = 0;
}

/* Left-channel lag, in frames, that best matches the audio before head. */
static uint16_t stream_conceal_period(const uint8_t *buf, uint32_t mask,
                                      uint32_t head) {
  uint16_t best_lag = CONCEAL_PERIOD_MAX;
  float best_score = 0.0f;
  for (int lag = CONCEAL_PERIOD_MIN; lag <= CONCEAL_PERIOD_MAX;
       lag += CONCEAL_LAG_STEP) {
    int64_t correlation = 0, energy = 0;
    for (int back = 1; back <= CONCEAL_MATCH_FRAMES; back += 4) {
      int32_t recent =
          ring_sample_get(buf, mask, head - back * CONCEAL_FRAME_BYTES) >> 8;
      int32_t earlier = ring_sample_get(
          buf, mask, head - (back + lag) * CONCEAL_FRAME_BYTES) >> 8;
      correlation += (int64_t)recent * earlier;
      energy += (int64_t)earlier * earlier;
    }
    if (correlation <= 0 || energy == 0)
      continue;
    float score = (float)correlation * (float)correlation / (float)energy;
    if (score > best_score) {
      best_score = score;
      best_lag = (uint16_t)lag;
    }
  }
  return best_lag;
}

/* One repetition frame, channel ch, number produced since the insert began. */
static inline int32_t stream_conceal_sample(const stream_rx_ctx_t *ctx,
                                            const uint8_t *buf, uint32_t mask,
                                            uint32_t produced, int ch) {
  if (produced >= CONCEAL_FADE_FRAMES)
    return 0;
  int32_t sample = ring_sample_get(
      buf, mask,
      ctx->conceal_src + (produced % ctx->conceal_period) * CONCEAL_FRAME_BYTES +
          ch * 4);
  uint32_t gain_q16 =
      ((CONCEAL_FADE_FRAMES - produced) * CONCEAL_FADE_RECIP_Q22) >> 6;
  return (int32_t)(((int64_t)sample * gain_q16) >> 16);
}

/* Writes fill_bytes of concealment at head (not yet published). */
static void stream_conceal_fill(stream_rx_ctx_t *ctx, uint32_t head,
                                uint32_t fill_bytes) {
  uint8_t *buf = ctx->ring.buf;
  uint32_t mask = ctx->ring.capacity - 1;
  uint32_t frames = fill_bytes / CONCEAL_FRAME_BYTES;
  int32_t before = ring_sample_get(buf, mask, head - CONCEAL_FRAME_BYTES);
  uint32_t repeated = 0;
  /* The pitch search is too costly to run for every packet. When gaps
   * come back to back, as from a stream a switch cuts down to its old
   * reservation, an insert that starts within one fade of the last
   * repetition is silence (observed: a gap on nearly every packet held
   * emac_rx on core 0 for 11 s, and the lost Pdelay responses sprang
   * the AVB Lite fallback). */
  if (s_conceal_repeat && head - ctx->conceal_repeat_head >=
                              CONCEAL_FADE_FRAMES * CONCEAL_FRAME_BYTES) {
    ctx->conceal_repeat_head = head;
    ctx->conceal_period = stream_conceal_period(buf, mask, head);
    ctx->conceal_src = head - ctx->conceal_period * CONCEAL_FRAME_BYTES;
    /* Past the fade the repetition is silence, written below. */
    repeated = frames < CONCEAL_FADE_FRAMES ? frames : CONCEAL_FADE_FRAMES;
    for (uint32_t frame = 0; frame < repeated; frame++)
      for (int ch = 0; ch < 2; ch++)
        ring_sample_put(buf, mask, head + frame * CONCEAL_FRAME_BYTES + ch * 4,
                        stream_conceal_sample(ctx, buf, mask, frame, ch));
    ctx->conceal_produced = frames;
    ctx->conceal_crossfade = frames < CONCEAL_FADE_FRAMES ? CONCEAL_CROSSFADE_FRAMES : 0;
  } else {
    ctx->conceal_period = 0;
    ctx->conceal_crossfade = 0;
  }
  uint32_t zero_start = (head + repeated * CONCEAL_FRAME_BYTES) & mask;
  uint32_t zero_bytes = fill_bytes - repeated * CONCEAL_FRAME_BYTES;
  uint32_t zero_first = ctx->ring.capacity - zero_start;
  if (zero_first > zero_bytes)
    zero_first = zero_bytes;
  memset(buf + zero_start, 0, zero_first);
  if (zero_bytes > zero_first)
    memset(buf, 0, zero_bytes - zero_first);
  int32_t first = ring_sample_get(buf, mask, head);
  ctx->conceal_step_sum[0] += (uint32_t)abs(first - before);
  ctx->conceal_step_count[0]++;
  ctx->conceal_step_pending = true;
}

/* Crossfades the frames just written at head into the repetition, and
 * measures the step from the insert into the real audio. */
static void stream_conceal_after_write(stream_rx_ctx_t *ctx, uint32_t head,
                                       uint32_t bytes) {
  uint8_t *buf = ctx->ring.buf;
  uint32_t mask = ctx->ring.capacity - 1;
  uint32_t frames = bytes / CONCEAL_FRAME_BYTES;
  for (uint32_t frame = 0;
       frame < frames && ctx->conceal_crossfade > 0 && ctx->conceal_period;
       frame++) {
    uint32_t real_share = CONCEAL_CROSSFADE_FRAMES - ctx->conceal_crossfade + 1;
    uint32_t real_q16 = (real_share * CONCEAL_CROSSFADE_RECIP_Q20) >> 4;
    for (int ch = 0; ch < 2; ch++) {
      uint32_t pos = head + frame * CONCEAL_FRAME_BYTES + ch * 4;
      int32_t real = ring_sample_get(buf, mask, pos);
      int32_t repeated =
          stream_conceal_sample(ctx, buf, mask, ctx->conceal_produced, ch);
      ring_sample_put(buf, mask, pos,
                      (int32_t)(((int64_t)real * real_q16 +
                                 (int64_t)repeated * (65536u - real_q16)) >>
                                16));
    }
    ctx->conceal_produced++;
    ctx->conceal_crossfade--;
  }
  if (ctx->conceal_step_pending && frames) {
    ctx->conceal_step_pending = false;
    int32_t last = ring_sample_get(buf, mask, head - CONCEAL_FRAME_BYTES);
    int32_t first = ring_sample_get(buf, mask, head);
    ctx->conceal_step_sum[1] += (uint32_t)abs(first - last);
    ctx->conceal_step_count[1]++;
  }
}

static void avb_stream_rx_handler(uint8_t *avtp_data, uint16_t len,
                                  void *arg) {
  stream_rx_ctx_t *ctx = (stream_rx_ctx_t *)arg;
  if (!ctx || len < 24)
    return;

  /* Stream-id filter: 8 bytes at [4..11] common to AAF and IEC 61883. */
  if (memcmp(avtp_data + 4, ctx->expected_stream_id, UNIQUE_ID_LEN) != 0) {
    ctx->stream_id_mismatch++;
    return;
  }

  /* seq_num gap tracking (avtp_data[2]) and tu-bit (avtp_data[3] bit 0).
   * Both are just volatile counter increments — cheap. lost_frames feeds
   * the loss-concealment insert below. */
  uint8_t seq_num = avtp_data[2];
  uint32_t lost_frames = 0;
  if (ctx->seq_num_valid && seq_num != (uint8_t)(ctx->last_seq_num + 1)) {
    ctx->seq_num_mismatch++;
    lost_frames = (uint8_t)(seq_num - (uint8_t)(ctx->last_seq_num + 1));
    if (lost_frames >= 250) {
      /* Duplicate, or late after its time was concealed: writing it
       * would add its length to the buffered timeline. */
      ctx->seq_late++;
      return;
    }
  }
  ctx->last_seq_num = seq_num;
  ctx->seq_num_valid = true;
  if (avtp_data[3] & 0x01)
    ctx->ts_uncertain_count++;

  uint8_t subtype = avtp_data[0] & 0x7F;

  /* Wire-format guard (see ctx field comment). AAF carries nsr in the
   * top nibble of byte 17; 61883 carries the CIP SFC in the low 3 bits
   * of byte 29 (CIP header byte 5). */
  if (ctx->rate_check_pending) {
    uint8_t rate_code = (subtype == avtp_subtype_aaf)
                            ? (avtp_data[17] >> 4) & 0x0F
                            : (len > 29 ? (avtp_data[29] & 0x07) : 0xFF);
    if (subtype != ctx->expected_subtype ||
        rate_code != ctx->expected_rate_code) {
      ctx->rate_mismatch++;
      return;
    }
    ctx->rate_check_pending = false;
  }

  uint8_t *pcm_data = NULL;
  int pcm_len = 0;
  int channels = 0;
  int samples = 0;

  if (subtype == avtp_subtype_aaf) {
    aaf_pcm_message_s *aaf_msg = (aaf_pcm_message_s *)avtp_data;
    uint16_t stream_data_len =
        (aaf_msg->stream_data_len[0] << 8) | aaf_msg->stream_data_len[1];
    if (stream_data_len == 0 || stream_data_len > AVTP_STREAM_DATA_PER_MSG)
      return;
    channels = aaf_msg->chan_per_frame;
    if (channels == 0)
      channels = 8;
    samples = stream_data_len / (channels * 4);
    pcm_data = aaf_msg->stream_data;
    pcm_len = stream_data_len;
  } else if (subtype == avtp_subtype_61883) {
    iec_61883_6_message_s *iec_msg = (iec_61883_6_message_s *)avtp_data;
    uint16_t stream_data_len =
        (iec_msg->stream_data_len[0] << 8) | iec_msg->stream_data_len[1];
    if (stream_data_len <= 8)
      return;
    uint8_t dbs = iec_msg->stream_data[1];
    channels = dbs;
    if (channels == 0)
      channels = 8;
    int data_bytes = stream_data_len - 8;
    samples = data_bytes / (channels * 4);
    pcm_data = iec_msg->stream_data + 8;
    pcm_len = data_bytes;
  } else {
    return;
  }

  if (samples <= 0 || !pcm_data)
    return;

  /* Capture first-packet diagnostics into ctx fields; avb_stream_in_print_diag
   * prints them from the main loop. Hot path never touches UART. */
  if (ctx->diag_captured == 0) {
    ctx->diag_subtype = subtype;
    ctx->diag_sdl =
        (subtype == avtp_subtype_61883)
            ? ((iec_61883_6_message_s *)avtp_data)->stream_data_len[0] << 8 |
                  ((iec_61883_6_message_s *)avtp_data)->stream_data_len[1]
            : 0;
    ctx->diag_channels = channels;
    ctx->diag_samples = samples;
    int copy = pcm_len < 8 ? pcm_len : 8;
    memcpy(ctx->diag_first_audio, pcm_data, copy);
    ctx->diag_captured = 1;
  }

  /* AVTP wire → I2S stereo 24-bit samples, written directly into the
   * jitter ring with a single atomic head-publish at the end. I2S slot
   * config is big-endian (avbcodec.c: slot_cfg.big_endian = true) so the
   * DMA/DAC path expects [MSB, MID, LSB] in memory — same byte order as
   * AAF wire format, which lets us memcpy the 3 sample bytes straight
   * across. AM824 prefixes a label byte; skip it. Extra channels are
   * ignored. */
  uint32_t total = (uint32_t)samples * 8; /* stereo 24-in-32 slots */

  /* Stream reference for the media-clock PLL: credit the talker's
   * media time by sequence number, so packets the network lost still
   * count as delivered and a loss clump reads as loss (concealed or
   * re-gated), never as a DAC rate error (observed: byte-based credit
   * read 8000 ppm at every clump). A gap the 8-bit sequence cannot
   * size is a discontinuity: bump the epoch so the PLL re-seeds. */
  if (ctx->state) {
    uint32_t credited_packets = 1;
    if (lost_frames >= 250) {
      credited_packets = 0; /* duplicate or reordered, already credited */
    } else if (lost_frames > 200) {
      atomic_fetch_add_explicit(&ctx->state->media_clock.stream_ref_epoch,
                                1u, memory_order_relaxed);
    } else {
      credited_packets += lost_frames;
    }
    if (credited_packets)
      atomic_fetch_add_explicit(
          &ctx->state->media_clock.stream_bytes_received,
          (uint64_t)total * credited_packets, memory_order_relaxed);
  }

  /* Loss concealment: the switch drops multicast frames on egress in
   * clumps (measured: ~8-25 frames every 1-3 min, absent at the talker
   * tap), plus occasional 50-70 ms outages. The DAC keeps consuming,
   * so every lost frame permanently shortens the buffered timeline —
   * playout creeps early and ring depth erodes until the starvation
   * re-gate churns. Insert the lost duration as silence so depth and
   * presentation alignment survive. Only while playout is locked
   * (during gating the anchor logic owns ring alignment), never
   * across a media-clock restart (mr=1).
   *
   * Sizing, by outage class:
   * - No/short pause (pure drop clump): wire-true packet count. The
   *   8-bit seq residue is disambiguated by the arrival gap (packet
   *   cadence = total/byterate); no DAC free-run occurred, so
   *   restoring exactly the lost duration is exactly right.
   * - Pause ≥ 10 ms (ring + I2S DMA exhausted, DAC free-ran
   *   auto_clear zeros): the free-run time must NOT be re-inserted
   *   or playout lands late. Size from presentation time instead:
   *   queue exactly enough silence that this packet's first byte
   *   plays at avtp_ts + EXTRA_LATENCY − OUTPUT_DELAY — the same
   *   equation the pt gate solves, done as a splice with no unlock,
   *   no ring flush, no gate wait. Mid-outage the DMA holds no
   *   scheduled audio, so ring fill alone is the queued depth.
   * A distrusted estimate (reorder signature, absurd lead) falls
   * back to the bounded residue fill; outages beyond what the ring
   * can hold still take the starvation re-gate. */
  /* Hot-path budget: read the clock only when a loss was seen or the
   * ring is empty (any pause that matters drains the ring first) —
   * routine packets skip both branches. */
  int64_t pause_us = 0;
  if (ctx->drain_locked && ctx->last_packet_us != 0 &&
      (lost_frames > 0 || ring_readable(&ctx->ring) == 0))
    pause_us = esp_timer_get_time() - ctx->last_packet_us;
  if (ctx->drain_locked && !(avtp_data[1] & 0x08) && total > 0 &&
      (lost_frames > 0 || pause_us > 10000)) {
    uint32_t fill_bytes = 0;
    bool aligned = false;
    if (pause_us > 10000) {
      /* The DAC free-ran on auto_clear zeros through this pause while
       * no sequence numbers arrived to credit, and the 8-bit residue
       * cannot size the gap: the stream-reference PLL must re-seed
       * (observed: a 33 ms outage read as 6500 ppm and railed the
       * trim before the crossing guard caught it). */
      if (ctx->state)
        atomic_fetch_add_explicit(&ctx->state->media_clock.stream_ref_epoch,
                                  1u, memory_order_relaxed);
      /* 10 ms: above this the ring + I2S DMA (~9 ms standing) are
       * drained, so ring fill alone is the queued depth and the
       * presentation-time sizing is exact. Below it the residue path
       * is exact instead (no DAC free-run yet). A 15 ms threshold
       * left a band where the residue path re-inserted time the DAC
       * had already free-run — measured +6 KB standing-depth ratchet.
       * Residual error in the crossover band is ≤~2 ms and the
       * absolute (want−have) sizing bleeds it off at the next long
       * pause. */
      /* Long pause: prefer presentation-time sizing — it is exact for
       * every mix of lost, delayed, and free-run time, including the
       * seq-blind exact-256 loss (residue 0). */
      if (ctx->diag_gate_mode == 1 && (avtp_data[1] & 0x01)) {
        struct timespec ts_now;
        if (ptpd_now(&ts_now) == 0) {
          uint32_t now32 = (uint32_t)((uint64_t)ts_now.tv_sec *
                                          1000000000ULL +
                                      (uint64_t)ts_now.tv_nsec);
          uint32_t pkt_ts = ((uint32_t)avtp_data[12] << 24) |
                            ((uint32_t)avtp_data[13] << 16) |
                            ((uint32_t)avtp_data[14] << 8) |
                            (uint32_t)avtp_data[15];
          int32_t lead = (int32_t)(
              pkt_ts + stream_in_extra_latency_ns(ctx) -
              ctx->output_delay_ns + (uint32_t)ctx->learned_latency_ns -
              now32);
          if (lead > -400000000 && lead < 400000000) {
            /* A sane clock read owns the decision either way. lead ≤ 0
             * means the resume packet is already at/past its
             * presentation time — the switch queued the stream and
             * dumped part of it (frames lost with little wall-clock
             * pause). Insert NOTHING: the DAC consumed real audio
             * through that interval, and the cadence fallback would
             * re-insert time that never left the buffer (measured
             * +19 KB standing-depth ratchet). */
            if (lead > 0) {
              int64_t want = ((int64_t)lead * ctx->ring_bytes_per_sec) /
                             1000000000LL;
              int64_t b = want - (int64_t)ring_readable(&ctx->ring);
              fill_bytes = b > 0 ? ((uint32_t)b & ~7u) : 0;
            } else {
              fill_bytes = 0;
            }
            aligned = true;
          }
        }
      }
      /* No fallback when the aligned sizing is unavailable (prefill
       * lock / tu=1 / clock unavailable): a long pause means the DAC
       * consumed buffer and then free-ran an unknowable amount, and
       * cadence sizing re-inserts that free-run time — measured
       * +20 KB standing-depth ratchet from a 37 ms queue-dump that
       * landed while a post-outage lock was still in prefill mode.
       * Aligned-or-nothing is provably ratchet-free; the eventual pt
       * re-gate realigns anything a skipped insert left behind. */
    } else if (lost_frames > 0 && lost_frames < 200) {
      /* Short/no pause: real-time drop clump — residue is the count,
       * and the wall-clock pause bounds it (the DAC consumed only
       * pause-worth of buffer; a big residue after a tiny pause is a
       * switch queue-dump whose time must not be re-inserted). +16
       * covers the ≤1 ms staleness of the sparse last_packet_us. */
      uint32_t n_fill = lost_frames;
      if (pause_us > 0 && ctx->ring_bytes_per_sec > 0) {
        uint32_t pause_pkts = (uint32_t)(
            (pause_us * (int64_t)ctx->ring_bytes_per_sec) /
            ((int64_t)total * 1000000LL)) + 16;
        if (n_fill > pause_pkts)
          n_fill = pause_pkts;
      }
      fill_bytes = n_fill * total;
    }
    /* A loss can take at most the ring's standing depth plus what the
     * I2S DMA played while no packet arrived, so an insert may refill
     * the ring to that, with 1 ms for the drain's own swing. A gap that
     * cost no playout time, such as two talkers sending one stream ID
     * or a restarted sequence count, then cannot grow the ring
     * (observed: 160 stray packets over 2 s filled the 64 KB ring and
     * held it full for a minute, +170 ms). */
    if (fill_bytes > 0 &&
        ctx->standing_depth_samples >= STANDING_DEPTH_MIN_SAMPLES) {
      uint64_t pause_bytes =
          pause_us > 0
              ? (uint64_t)pause_us * ctx->ring_bytes_per_sec / 1000000
              : 0;
      uint64_t ceiling = (ctx->standing_depth_q4 >> 4) +
                         ctx->ring_bytes_per_sec / 1000 + pause_bytes;
      uint32_t readable = ring_readable(&ctx->ring);
      uint64_t room = ceiling > readable ? (ceiling - readable) & ~7ull : 0;
      if (fill_bytes > room)
        fill_bytes = (uint32_t)room;
    }
    if (fill_bytes > 0 && ring_writable(&ctx->ring) >= fill_bytes + total) {
      uint32_t fh = atomic_load_explicit(&ctx->ring.head, memory_order_relaxed);
      stream_conceal_fill(ctx, fh, fill_bytes);
      atomic_store_explicit(&ctx->ring.head, fh + fill_bytes,
                            memory_order_release);
      ctx->loss_fill_events++;
      ctx->loss_fill_frames += fill_bytes / total;
      if (aligned)
        ctx->splice_aligned++;
    }
  }

  uint32_t pre_fill = ring_readable(&ctx->ring);
  if (!ctx->drain_locked) {
    ctx->standing_depth_samples = 0;
  } else if (lost_frames > 0) {
    ctx->standing_depth_hold = STANDING_DEPTH_HOLD_PACKETS;
  } else if (ctx->standing_depth_hold > 0) {
    ctx->standing_depth_hold--;
  } else {
    int32_t sample_q4 = (int32_t)(pre_fill << 4);
    if (ctx->standing_depth_samples == 0)
      ctx->standing_depth_q4 = (uint32_t)sample_q4;
    else
      ctx->standing_depth_q4 = (uint32_t)(
          (int32_t)ctx->standing_depth_q4 +
          ((sample_q4 - (int32_t)ctx->standing_depth_q4) >> 6));
    if (ctx->standing_depth_samples < STANDING_DEPTH_MIN_SAMPLES)
      ctx->standing_depth_samples++;
  }
  if (total > 0 && ring_writable(&ctx->ring) >= total) {
    uint32_t h = atomic_load_explicit(&ctx->ring.head, memory_order_relaxed);
    uint32_t mask = ctx->ring.capacity - 1;
    uint8_t *rbuf = ctx->ring.buf;
    uint32_t offset = 0;
    for (int s = 0; s < samples; s++) {
      for (int ch = 0; ch < 2; ch++) {
        int src_ch = (ch < channels) ? ch : 0;
        int src_offset = (s * channels + src_ch) * 4;
        uint32_t p0 = (h + offset + 0) & mask;
        uint32_t p1 = (h + offset + 1) & mask;
        uint32_t p2 = (h + offset + 2) & mask;
        uint32_t p3 = (h + offset + 3) & mask;
        if (subtype == avtp_subtype_aaf) {
          /* AAF: [MSB, MID, LSB, pad] → I2S memory [MSB, MID, LSB, 0] */
          if (src_offset + 2 < pcm_len) {
            rbuf[p0] = pcm_data[src_offset + 0];
            rbuf[p1] = pcm_data[src_offset + 1];
            rbuf[p2] = pcm_data[src_offset + 2];
          } else {
            rbuf[p0] = rbuf[p1] = rbuf[p2] = 0;
          }
        } else { /* AM824: [label, MSB, MID, LSB] → [MSB, MID, LSB, 0] */
          if (src_offset + 3 < pcm_len) {
            rbuf[p0] = pcm_data[src_offset + 1];
            rbuf[p1] = pcm_data[src_offset + 2];
            rbuf[p2] = pcm_data[src_offset + 3];
          } else {
            rbuf[p0] = rbuf[p1] = rbuf[p2] = 0;
          }
        }
        rbuf[p3] = 0; /* LSB pad of the 32-bit slot */
        offset += 4;
      }
    }
    if (ctx->conceal_crossfade || ctx->conceal_step_pending)
      stream_conceal_after_write(ctx, h, total);
    /* Publish — drain can't see any of the bytes we just wrote until
     * this store lands (release ordering pairs with ring_readable's
     * acquire load of head). */
    atomic_store_explicit(&ctx->ring.head, h + total, memory_order_release);
    ctx->ring_write_ok++;
    if (avtp_data[1] & 0x01) { /* tv */
      uint32_t marker = atomic_load_explicit(&ctx->pt_marker_count,
                                             memory_order_relaxed);
      ctx->pt_markers[marker % STREAM_PT_MARKERS].pos = h;
      ctx->pt_markers[marker % STREAM_PT_MARKERS].pt =
          ((uint32_t)avtp_data[12] << 24) | ((uint32_t)avtp_data[13] << 16) |
          ((uint32_t)avtp_data[14] << 8) | (uint32_t)avtp_data[15];
      atomic_store_explicit(&ctx->pt_marker_count, marker + 1,
                            memory_order_release);
    }
    if (!ctx->diag_i2s_bytes)
      ctx->diag_i2s_bytes = (int)total;
    /* Presentation anchor: first packet into an empty ring with a
     * trustworthy timestamp (tu clear). Its first byte is at ring
     * tail, so its avtp_timestamp is the presentation time of the
     * drain's next byte. tu=1 packets never arm — the drain then
     * falls back to the byte-prefill gate. */
    if (pre_fill == 0 &&
        !atomic_load_explicit(&ctx->gate_armed, memory_order_relaxed)) {
      if (avtp_data[3] & 0x01) {
        ctx->pt_unarmed_arrivals++;
      } else {
        ctx->gate_avtp_ts = ((uint32_t)avtp_data[12] << 24) |
                            ((uint32_t)avtp_data[13] << 16) |
                            ((uint32_t)avtp_data[14] << 8) |
                            (uint32_t)avtp_data[15];
        ctx->gate_arm_us = esp_timer_get_time();
        atomic_store_explicit(&ctx->gate_armed, true, memory_order_release);
      }
    }
#if portNUM_PROCESSORS > 1
    stream_in_wake_drain(ctx, pre_fill, total);
#endif
  } else if (total > 0) {
    ctx->ring_write_fail++;
  }

  /* Sparse drift-sampler stash: every 8 packets (~1 ms at 8000 pps),
   * copy this packet's AVTP presentation timestamp and arrival time.
   * The 10 ms-rate drift sampler in the main loop reads from here — a
   * few ms of staleness in the rarest case doesn't affect the moving
   * average. */
  ctx->pkt_count++;
  if ((ctx->pkt_count & 0x7) == 0) {
    ctx->drift_latest_avtp_ts =
        ((uint32_t)avtp_data[12] << 24) | ((uint32_t)avtp_data[13] << 16) |
        ((uint32_t)avtp_data[14] << 8) | (uint32_t)avtp_data[15];
    ctx->drift_latest_rx_ts_us = esp_timer_get_time();
    ctx->drift_latest_valid = true;
    ctx->last_packet_us = ctx->drift_latest_rx_ts_us;
  }
}

/* Audio the listener of stream_id holds ahead of its playout, in us;
 * false unless that listener is playing. Called from the Wi-Fi receive
 * path: head and tail are atomics, the rest is fixed while playing. */
bool avb_stream_in_buffered_us(const uint8_t *stream_id, int32_t *buffered_us) {
  stream_rx_ctx_t *ctx = s_stream_rx_ctx;
  if (!ctx || !ctx->phase_valid || ctx->ring_bytes_per_sec == 0 ||
      memcmp(ctx->expected_stream_id, stream_id, UNIQUE_ID_LEN) != 0)
    return false;
  *buffered_us = (int32_t)(((uint64_t)ring_readable(&ctx->ring) * 1000000ULL) /
                           ctx->ring_bytes_per_sec);
  return true;
}

/* One-shot "first packet received" line: the handler captured the
 * details silently into ctx fields and AVB-STATS prints them within a
 * few hundred ms, so the line's time stamp marks when audio arrived and
 * UART I/O never touches the hot path. */
void avb_stream_in_print_first_packet(void) {
  stream_rx_ctx_t *ctx = s_stream_rx_ctx;
  if (!ctx)
    return;
  if (ctx->diag_captured == 1) {
    ctx->diag_captured = 2;
    avbinfo("STREAM: first packet sub=%d sdl=%d ch=%d samp=%d i2s=%d "
            "audio=[%02x %02x %02x %02x %02x %02x %02x %02x]",
            ctx->diag_subtype, ctx->diag_sdl, ctx->diag_channels,
            ctx->diag_samples, ctx->diag_i2s_bytes, ctx->diag_first_audio[0],
            ctx->diag_first_audio[1], ctx->diag_first_audio[2],
            ctx->diag_first_audio[3], ctx->diag_first_audio[4],
            ctx->diag_first_audio[5], ctx->diag_first_audio[6],
            ctx->diag_first_audio[7]);
  }
}

/* Print stream-in diagnostics — called from AVB-STATS at the window
 * cadence (safe for UART). */
void avb_stream_in_print_diag(void) {
  stream_rx_ctx_t *ctx = s_stream_rx_ctx;
  if (!ctx)
    return;
  avb_stream_in_print_first_packet();

  /* Per-window drain-health + counters snapshot. Deltas per-window.
   * Read the volatile counters directly — no atomics because SPSC with
   * aligned 32-bit fields is torn-free on RISC-V. Cadence is set by the
   * caller (AVB-STATS). */
  static uint32_t last_pkt = 0, last_ok = 0, last_fail = 0;
  static uint32_t last_drain = 0, last_under = 0;
  /* A new stream's counters start at zero: re-baseline, as stream-out
   * does, or the first window prints wrapped deltas. */
  static uint32_t prev_serial = 0;
  if (ctx->start_serial != prev_serial) {
    prev_serial = ctx->start_serial;
    last_pkt = last_ok = last_fail = last_drain = last_under = 0;
  }

  uint32_t packets = ctx->pkt_count;
  uint32_t ok = ctx->ring_write_ok;
  uint32_t fail = ctx->ring_write_fail;
  uint32_t drain = ctx->drain_count;
  uint32_t under = ctx->drain_underrun;

  uint32_t rx_drops = avb_net_stream_rx_drops();
  uint32_t bundles = 0, bundle_frames = 0, split_fail = 0;
  avb_wifi_bundle_rx_stats(&bundles, &bundle_frames, &split_fail);
  uint32_t nacks = 0, recovered = 0, given_up = 0, duplicates = 0;
  uint32_t nacks_skipped = 0;
  avb_wifi_retx_stats(&nacks, &recovered, &given_up, &duplicates);
  avb_wifi_retx_detail(&nacks_skipped);

  {
    uint32_t into = ctx->conceal_step_count[0], out = ctx->conceal_step_count[1];
    avbinfo("CONCEAL: ab=%d inserts=%lu step_in=%lu step_out=%lu",
            s_conceal_repeat, (unsigned long)into,
            (unsigned long)(into ? ctx->conceal_step_sum[0] / into : 0),
            (unsigned long)(out ? ctx->conceal_step_sum[1] / out : 0));
    ctx->conceal_step_sum[0] = ctx->conceal_step_sum[1] = 0;
    ctx->conceal_step_count[0] = ctx->conceal_step_count[1] = 0;
#if CONFIG_ESP_AVB_LISTENER_CONCEAL_AB
    static unsigned conceal_intervals;
    if (!(++conceal_intervals % 6))
      s_conceal_repeat = !s_conceal_repeat;
#endif
  }
  uint32_t gap_hist[7];
  avb_wifi_gap_histogram(gap_hist);
  if (bundles)
    avbinfo("WIFI-GAPS: 1=%lu 2=%lu 3=%lu 4=%lu 5-8=%lu 9-16=%lu more=%lu",
            (unsigned long)gap_hist[0], (unsigned long)gap_hist[1],
            (unsigned long)gap_hist[2], (unsigned long)gap_hist[3],
            (unsigned long)gap_hist[4], (unsigned long)gap_hist[5],
            (unsigned long)gap_hist[6]);

  /* Playout phase window (see stream_in_phase_tick). */
  uint32_t phase_n = ctx->phase_reset ? 0 : ctx->phase_samples;
  int32_t phase_min = phase_n ? ctx->phase_min_us : 0;
  int32_t phase_max = phase_n ? ctx->phase_max_us : 0;
  int32_t phase_avg = phase_n ? (int32_t)(ctx->phase_sum_us / phase_n) : 0;
  int32_t ahead_min = phase_n ? ctx->ahead_min_us : 0;
  int32_t ahead_max = phase_n ? ctx->ahead_max_us : 0;
  ctx->phase_reset = true;
  uint32_t src_cycles = ctx->src_frames ? ctx->src_cycles / ctx->src_frames : 0;
  uint32_t drain_gap_us = s_drain_gap_max_us;
  s_drain_gap_max_us = 0;
  ctx->src_cycles = ctx->src_frames = 0;

  avbinfo("STREAM: pkts=%lu ok=%lu fail=%lu drain=%lu under=%lu "
          "qfill=%lu id_skip=%lu rate_mm=%lu seq_gap=%lu conceal=%lu/%lu "
          "spl=%lu rx_drops=%lu locked=%lu/%lu gate=%s(err=%ldns fill=%lu) "
          "bundles=%lu/%lu nack=%lu nack_skip=%lu rec=%lu gaveup=%lu dup=%lu "
          "starved=%luB "
          "late=%lu split_fail=%lu phase=%ld/%ld/%ldus ahead=%ld/%ldus "
          "phase_n=%lu trims=%lu/%luB slips=%lu/%lu src=%lucyc/%lu "
          "dgap=%luus lockres=%ldus",
          (unsigned long)(packets - last_pkt),
          (unsigned long)(ok - last_ok),
          (unsigned long)(fail - last_fail),
          (unsigned long)(drain - last_drain),
          (unsigned long)(under - last_under),
          (unsigned long)ring_readable(&ctx->ring),
          (unsigned long)ctx->stream_id_mismatch,
          (unsigned long)ctx->rate_mismatch,
          (unsigned long)ctx->seq_num_mismatch,
          (unsigned long)ctx->loss_fill_events,
          (unsigned long)ctx->loss_fill_frames,
          (unsigned long)ctx->splice_aligned,
          (unsigned long)rx_drops,
          (unsigned long)ctx->media_locked_count,
          (unsigned long)ctx->media_unlocked_count,
          ctx->diag_gate_mode == 1   ? "pt"
          : ctx->diag_gate_mode == 2 ? "prefill"
                                     : "none",
          (long)ctx->diag_gate_err_ns, (unsigned long)ctx->diag_gate_fill,
          (unsigned long)bundles, (unsigned long)bundle_frames,
          (unsigned long)nacks, (unsigned long)nacks_skipped,
          (unsigned long)recovered, (unsigned long)given_up,
          (unsigned long)duplicates,
          ctx->state ? (unsigned long)atomic_load_explicit(
                           &ctx->state->media_clock.i2s_tx_starved_bytes,
                           memory_order_relaxed)
                     : 0UL,
          (unsigned long)ctx->seq_late, (unsigned long)split_fail,
          (long)phase_min, (long)phase_avg, (long)phase_max, (long)ahead_min,
          (long)ahead_max, (unsigned long)phase_n,
          (unsigned long)ctx->trims, (unsigned long)ctx->trim_bytes,
          (unsigned long)ctx->slip_drops, (unsigned long)ctx->slip_repeats,
          (unsigned long)src_cycles, (unsigned long)ctx->src.walks,
          (unsigned long)drain_gap_us,
          (long)(ctx->diag_lock_residual_ns / 1000));
  /* Ground-truth snapshot of what the drain is feeding the DAC: 12
   * bytes at the ring tail (2 stereo 24-bit frames). Distinguishes
   * "ring holds real audio, fault is downstream of i2s_channel_write"
   * from "ring holds silence/garbage, fault is in extraction". */
  {
    uint32_t t = atomic_load_explicit(&ctx->ring.tail, memory_order_relaxed);
    uint32_t mask = ctx->ring.capacity - 1;
    uint8_t snap[12];
    for (int i = 0; i < 12; i++)
      snap[i] = ctx->ring.buf[(t + i) & mask];
    avbdebug("STREAM-RING tail: %02x%02x%02x %02x%02x%02x | %02x%02x%02x %02x%02x%02x",
            snap[0], snap[1], snap[2], snap[3], snap[4], snap[5], snap[6],
            snap[7], snap[8], snap[9], snap[10], snap[11]);
  }
  last_pkt = packets;
  last_ok = ok;
  last_fail = fail;
  last_drain = drain;
  last_under = under;
}

/* Print stream-out diagnostics — called from AVB-STATS at the window
 * cadence. Mirrors avb_stream_in_print_diag: reads the volatile ctx
 * (written ~1 Hz by the TX task) and prints a delta-per-window summary.
 * Silent when no TX session is active. */
void avb_stream_out_print_diag(void) {
  static uint32_t prev_serial = 0;
  stream_tx_ctx_t *ctx = s_stream_tx_ctx;
  if (!ctx || !ctx->active)
    return;
  /* A new start means counters that begin at zero, so the per-window
   * deltas must re-baseline. Keyed on a start serial: a stop shorter
   * than one stats period is never seen here, and a freed context is
   * usually re-allocated at the same address. */
  bool prev_active = ctx->start_serial == prev_serial;
  prev_serial = ctx->start_serial;

  static uint32_t last_pkts = 0, last_fail = 0, last_over = 0;
  static uint32_t last_z = 0, last_nz = 0, last_skip = 0, last_under = 0;
  static uint32_t last_slip = 0, last_slip_pkts = 0;
  static uint32_t last_resync = 0;
  static uint32_t last_anchors = 0, last_dropped = 0;
  static uint32_t last_wakes = 0, last_stalls = 0;
  if (!prev_active) {
    last_pkts = last_fail = last_over = 0;
    last_z = last_nz = last_skip = last_resync = last_under = 0;
    last_slip = last_slip_pkts = 0;
    last_anchors = last_dropped = 0;
    last_wakes = last_stalls = 0;
  }

  uint32_t pkts = ctx->pkt_count;
  uint32_t fail = ctx->send_fail_count;
  uint32_t over = ctx->overrun_count;
  uint32_t z = ctx->i2s_zero_reads;
  uint32_t nz = ctx->i2s_nonzero_reads;
  uint32_t under = ctx->ring_underruns;
  uint32_t slips = ctx->slip_events;
  uint32_t slip_pkts = ctx->slip_muted_pkts;
  uint32_t skip = ctx->pll_skip_count;
  uint32_t resync = ctx->gptp_resync_count;
  uint32_t anchors = ctx->ts_reanchors;
  uint32_t dropped = ctx->i2s_dropped_bytes;
  uint32_t wakes = ctx->wakes;
  uint32_t stalls = ctx->stalls;
  uint32_t wbundles = 0, wbundle_frames = 0, wnacks = 0, wretx = 0;
  uint32_t wretx_late = 0, wretx_evicted = 0;
#if !defined(CONFIG_ESP_AVB_ROLE_BRIDGE) && defined(CONFIG_ESP_AVB_WIFI_BUNDLE)
  avb_wifi_tx_bundle_stats(&wbundles, &wbundle_frames);
  avb_wifi_tx_retx_stats(&wnacks, &wretx);
  avb_wifi_tx_retx_detail(&wretx_late, &wretx_evicted);
#endif
  (void)wbundle_frames;
  int64_t over_max = ctx->overrun_max_us;
  int32_t pll_min = ctx->pll_offset_min_ns;
  int32_t pll_max = ctx->pll_offset_max_ns;
  int32_t drift_ppm_q8 = ctx->drift_ppm_q8;
  int32_t rem_min = ctx->remaining_ns_min;
  int32_t rem_max = ctx->remaining_ns_max;

  bool pll_has_sample = pll_min <= pll_max;
  bool rem_has_sample = rem_min <= rem_max;

  /* ppm with 2-decimal precision: drift_ppm_q8 / 256 = ppm, ×100 → centippm */
  int32_t drift_centippm = (int32_t)((int64_t)drift_ppm_q8 * 100 / 256);
  int abs_cppm = drift_centippm < 0 ? -drift_centippm : drift_centippm;
  int32_t fill_centippm = (int32_t)((int64_t)ctx->fill_ppm_q8 * 100 / 256);
  int abs_fill_cppm = fill_centippm < 0 ? -fill_centippm : fill_centippm;
  long backlog_bytes = (long)ctx->fill_backlog_bytes;

  avbinfo("STREAM-OUT: pkts=%lu fail=%lu over=%lu(max=%lldus) "
          "stalls=%lu wakes=%lu "
          "i2s_zero=%lu i2s_nz=%lu under=%lu pll=[%s] pll_skip=%lu "
          "drift=%s%ld.%02d ppm fill=%s%ld.%02d ppm backlog=%ldB rem=[%s] "
          "resync=%lu slips=%lu/%lu anchors=%lu dropped=%luB ts_err=%ldus "
          "wbundles=%lu wnack=%lu wretx=%lu/%lu/%lu src=%lucyc/%lu",
          (unsigned long)(pkts - last_pkts),
          (unsigned long)(fail - last_fail),
          (unsigned long)(over - last_over), (long long)over_max,
          (unsigned long)(stalls - last_stalls),
          (unsigned long)(wakes - last_wakes),
          (unsigned long)(z - last_z),
          (unsigned long)(nz - last_nz),
          (unsigned long)(under - last_under),
          pll_has_sample ? "" : "no-sample",
          (unsigned long)(skip - last_skip),
          drift_centippm < 0 ? "-" : "",
          (long)(abs_cppm / 100), (int)(abs_cppm % 100),
          fill_centippm < 0 ? "-" : "",
          (long)(abs_fill_cppm / 100), (int)(abs_fill_cppm % 100),
          backlog_bytes,
          rem_has_sample ? "" : "no-sample",
          (unsigned long)(resync - last_resync),
          (unsigned long)(slips - last_slip),
          (unsigned long)(slip_pkts - last_slip_pkts),
          (unsigned long)(anchors - last_anchors),
          (unsigned long)(dropped - last_dropped),
          (long)(ctx->ts_err_ns / 1000), (unsigned long)wbundles,
          (unsigned long)wnacks, (unsigned long)wretx,
          (unsigned long)wretx_late, (unsigned long)wretx_evicted,
          (unsigned long)ctx->src_cycles, (unsigned long)ctx->src_walks);

  /* Secondary detail lines only emitted when samples were actually
   * collected, to keep noise-free output when the stream is idle. */
  if (pll_has_sample) {
    avbinfo("  STREAM-OUT-pll: min=%ldns max=%ldns",
            (long)pll_min, (long)pll_max);
  }
  if (rem_has_sample) {
    avbdebug("  STREAM-OUT-rem: min=%ldns max=%ldns",
            (long)rem_min, (long)rem_max);
  }
#if CONFIG_ESP_AVB_LISTENER_MIRROR
  /* Running totals since boot. */
  avbinfo("  STREAM-OUT-mirror: under=%luB reprimes=%lu over=%luB skipped=%luB "
          "fill=%luB",
          (unsigned long)s_mirror_under_bytes,
          (unsigned long)s_mirror_reprimes,
          (unsigned long)s_mirror_over_bytes,
          (unsigned long)s_mirror_skipped_bytes,
          (unsigned long)(atomic_load(&s_mirror_head) -
                          atomic_load(&s_mirror_tail)));
#endif

  last_pkts = pkts;
  last_fail = fail;
  last_over = over;
  last_wakes = wakes;
  last_stalls = stalls;
  last_z = z;
  last_nz = nz;
  last_under = under;
  last_slip = slips;
  last_slip_pkts = slip_pkts;
  last_skip = skip;
  last_resync = resync;
  last_anchors = anchors;
  last_dropped = dropped;
}

/* Deferred drift sampler. Called from AVB main's periodic loop at
 * ~100 Hz. Reads the "latest packet" snapshot each RX handler stashed,
 * does the single expensive clock_gettime(CLOCK_PTP_SYSTEM) call, and
 * updates the stream_* / crf_* drift stats that avb_pll_tick consumes.
 *
 * Moved out of the RX hot path so the audio stream handler no longer
 * contends with PTPD on the PTP-clock driver. ~100 samples/s over the
 * PLL's 5-s window = ~500 samples per PLL computation — plenty for a
 * stable mean.
 *
 * Races with the handlers writing the latest_* fields are benign: at
 * worst we compute drift against a torn timestamp once. The next tick
 * will replace it.
 *
 * The esp_timer delta from rx_ts_us to now_us is used to slide the
 * PTP "now" timestamp back to packet arrival time. esp_timer and
 * CLOCK_PTP_SYSTEM run at effectively the same rate over the ~10 ms
 * window we care about, so the conversion error is negligible. */
void avb_stream_in_sample_drift(avb_state_s *state) {
  if (!state)
    return;
  /* Rate-limit. Called frequently from main periodic; we only want one
   * sample per ~10 ms. */
  static int64_t next_sample_us = 0;
  int64_t now_us = esp_timer_get_time();
  if (now_us < next_sample_us)
    return;
  next_sample_us = now_us + 10 * 1000; /* 10 ms */

  /* Snapshot whichever streams are active. Single PTP read shared
   * between the audio stream and CRF. */
  stream_rx_ctx_t *stream_ctx = s_stream_rx_ctx;
  crf_rx_ctx_t *crf_ctx = s_crf_rx_ctx;
  bool have_stream = (stream_ctx && stream_ctx->drift_latest_valid);
  bool have_crf = (crf_ctx && crf_ctx->drift_latest_valid);
  if (!have_stream && !have_crf)
    return;

  struct timespec now_ts;
  if (ptpd_now(&now_ts) != 0)
    return;
  uint64_t now_ptp_ns =
      (uint64_t)now_ts.tv_sec * 1000000000ULL + (uint64_t)now_ts.tv_nsec;
  int64_t now_esp_us = esp_timer_get_time();

  if (have_stream) {
    uint32_t avtp_ts = stream_ctx->drift_latest_avtp_ts;
    int64_t rx_ts_us = stream_ctx->drift_latest_rx_ts_us;
    int64_t elapsed_us = now_esp_us - rx_ts_us;
    if (elapsed_us < 0)
      elapsed_us = 0;
    uint64_t arrival_ptp_ns =
        now_ptp_ns - (uint64_t)((int64_t)elapsed_us * 1000);
    int32_t drift = (int32_t)(avtp_ts - (uint32_t)arrival_ptp_ns);
    state->media_clock.stream_last_drift_ns = drift;
    state->media_clock.stream_drift_sum_ns += drift;
    state->media_clock.stream_samples++;
    if (state->media_clock.stream_samples == 1 ||
        drift < state->media_clock.stream_drift_min_ns) {
      state->media_clock.stream_drift_min_ns = drift;
    }
    if (state->media_clock.stream_samples == 1 ||
        drift > state->media_clock.stream_drift_max_ns) {
      state->media_clock.stream_drift_max_ns = drift;
    }
    /* Milan late_ts / early_ts — approximate at sample rate, which is
     * enough for trend. */
    if (drift < 0)
      stream_ctx->late_ts_count++;
    else if (drift > 50 * 1000 * 1000)
      stream_ctx->early_ts_count++;
  }

  if (have_crf) {
    uint64_t crf_ts_ns = crf_ctx->drift_latest_crf_ts_ns;
    int64_t rx_ts_us = crf_ctx->drift_latest_rx_ts_us;
    int64_t elapsed_us = now_esp_us - rx_ts_us;
    if (elapsed_us < 0)
      elapsed_us = 0;
    uint64_t arrival_ptp_ns =
        now_ptp_ns - (uint64_t)((int64_t)elapsed_us * 1000);
    int64_t drift = (int64_t)(crf_ts_ns - arrival_ptp_ns);
    state->media_clock.crf_last_drift_ns = drift;
    state->media_clock.crf_drift_sum_ns += drift;
    state->media_clock.crf_samples++;
    if (state->media_clock.crf_samples == 1 ||
        drift < state->media_clock.crf_drift_min_ns) {
      state->media_clock.crf_drift_min_ns = (int32_t)drift;
    }
    if (state->media_clock.crf_samples == 1 ||
        drift > state->media_clock.crf_drift_max_ns) {
      state->media_clock.crf_drift_max_ns = (int32_t)drift;
    }
  }
}

/* Fill stream input counters for GET_COUNTERS response (Milan Table 5.13) */
void avb_get_stream_out_counters(aem_stream_out_counters_val_s *valid,
                                 aem_stream_out_counters_s *counters) {
  memset(valid, 0, sizeof(*valid));
  memset(counters, 0, sizeof(*counters));
  /* Milan Table 5.14 mandatory counters; media_reset and ts_uncertain
   * never toggle on this talker, so they stay at zero. */
  valid->stream_start = true;
  valid->stream_stop = true;
  valid->media_reset = true;
  valid->ts_uncertain = true;
  valid->frames_tx = true;
  stream_tx_ctx_t *ctx = s_stream_tx_ctx;
  uint32_t frames = s_tx_frames_total + (ctx ? ctx->pkt_count : 0);
  uint32_t starts = s_tx_starts;
  uint32_t stops = s_tx_stops;
  int_to_octets(&frames, counters->frames_tx, 4);
  int_to_octets(&starts, counters->stream_start, 4);
  int_to_octets(&stops, counters->stream_stop, 4);
}

void avb_get_stream_in_counters(aem_stream_in_counters_val_s *valid,
                                aem_stream_in_counters_s *counters) {
  memset(valid, 0, sizeof(*valid));
  memset(counters, 0, sizeof(*counters));

  stream_rx_ctx_t *ctx = s_stream_rx_ctx;

  /* Set valid flags for all Milan mandatory counters (Table 5.13) */
  valid->media_locked = true;
  valid->media_unlocked = true;
  valid->stream_interrupted = true;
  valid->seq_num_mismatch = true;
  valid->media_reset = true;
  valid->ts_uncertain = true;
  valid->unsupported_format = true;
  valid->late_ts = true;
  valid->early_ts = true;
  valid->frames_rx = true;

  if (!ctx)
    return;

  /* Populate counters from stream RX context + media_clock state.
   * All are 32-bit monotonic rollovers per Milan §5.4.8. The
   * valid->unsupported_format flag MUST stay true (set above) — some
   * Milan controllers reject enumeration if any of the mandatory
   * Table 5.13 flags are false, even when the counter itself is zero. */
  uint32_t counter_val;
  counter_val = ctx->pkt_count;
  int_to_octets(&counter_val, counters->frames_rx, 4);
  counter_val = ctx->seq_num_mismatch;
  int_to_octets(&counter_val, counters->seq_num_mismatch, 4);
  counter_val = ctx->ring_write_fail;
  int_to_octets(&counter_val, counters->stream_interrupted, 4);
  counter_val = ctx->media_locked_count;
  int_to_octets(&counter_val, counters->media_locked, 4);
  counter_val = ctx->media_unlocked_count;
  int_to_octets(&counter_val, counters->media_unlocked, 4);
  counter_val = ctx->ts_uncertain_count;
  int_to_octets(&counter_val, counters->ts_uncertain, 4);
  counter_val = ctx->late_ts_count;
  int_to_octets(&counter_val, counters->late_ts, 4);
  counter_val = ctx->early_ts_count;
  int_to_octets(&counter_val, counters->early_ts, 4);
  if (ctx->state) {
    counter_val = ctx->state->media_clock.media_reset_count;
    int_to_octets(&counter_val, counters->media_reset, 4);
  }
}

/* Stream RX dispatcher — registered with the net layer; routes incoming
 * VLAN AVTP frames by stream_id to either the audio stream handler or the CRF
 * handler. Both contexts are checked because either may be active. */
static void avb_stream_rx_dispatcher(uint8_t *avtp_data, uint16_t len,
                                     void *unused_ctx) {
  (void)unused_ctx;
  if (!avtp_data || len < 12)
    return;
  /* stream_id lives at bytes [4..11] in all AVTP stream subtypes */
  if (s_crf_rx_ctx != NULL &&
      memcmp(avtp_data + 4, s_crf_rx_ctx->expected_stream_id,
             UNIQUE_ID_LEN) == 0) {
    avb_crf_rx_handler(avtp_data, len, s_crf_rx_ctx);
    return;
  }
  if (s_stream_rx_ctx != NULL) {
    avb_stream_rx_handler(avtp_data, len, s_stream_rx_ctx);
  }
}

/* Start CRF media clock input (Milan v1.3 §7.2.2).
 * Lightweight compared to the audio stream input: no I2S, no jitter
 * buffer — just a context for packet reception and stats. Recorded
 * timestamps feed a future media-clock PLL. */
static int avb_start_stream_in_crf(avb_state_s *state, uint16_t index) {
  if (s_crf_rx_ctx != NULL) {
    avberr("CRF stream-in already running");
    return ERROR;
  }
  crf_rx_ctx_t *ctx = calloc(1, sizeof(crf_rx_ctx_t));
  if (!ctx) {
    avberr("CRF stream in: no memory for context");
    return ERROR;
  }
  ctx->state = state;
  memcpy(ctx->expected_stream_id, state->input_streams[index].stream_id,
         UNIQUE_ID_LEN);
  s_crf_rx_ctx = ctx;
  state->input_streams[index].connected = true;
  /* Ensure the dispatcher is registered even if audio stream input isn't running yet */
  avb_net_set_stream_rx_handler(avb_stream_rx_dispatcher, NULL);
  avbinfo("CRF stream in started (stream index %d)", index);
  return OK;
}

/* One stream-in runs at a time, so its jitter ring is allocated once and
 * kept, as early as the boot reserves it. Allocated at a start, it failed
 * on a C6 nine minutes after boot, and at a station's first listener
 * start after talker runs: no 64 KB block was left. */
static uint8_t *s_jitter_ring_buf;
#if CONFIG_ESP_AVB_LISTENER_RESAMPLE
static uint8_t s_src_in_buf[(STREAM_SRC_MAX_FRAMES + 2) * STREAM_FRAME_BYTES];
static uint8_t s_src_out_buf[STREAM_SRC_MAX_FRAMES * STREAM_FRAME_BYTES];
#endif

void avb_stream_in_reserve(void) {
  if (!s_jitter_ring_buf)
    s_jitter_ring_buf = calloc(1, JITTER_RING_SIZE);
#if CONFIG_ESP_AVB_LISTENER_RESAMPLE
  avb_src_s probe;
  avb_src_init(&probe); /* builds the shared table */
#endif
}

/* Start AVB stream input — opens codec, creates jitter buffer,
 * starts drain timer, registers stream RX handler. */
int avb_start_stream_in(avb_state_s *state, uint16_t index) {

  if (index == avb_get_crf_input_index(state)) {
    return avb_start_stream_in_crf(state, index);
  }

  if (state->stream_in_active) {
    avberr("Another instance of stream-in is already running");
    return ERROR;
  }

  /* Allocate ctx + the ring/drain/convert buffers. Ring size is power of 2
   * (see JITTER_RING_SIZE) — 16 KB ≈ 56 ms of 48 kHz stereo 24-bit audio.
   * Oversized vs the Class A transit budget (2 ms) on purpose so NVS
   * flash-cache-disable windows (≤50 ms) don't starve the drain. */
  stream_rx_ctx_t *ctx = calloc(1, sizeof(stream_rx_ctx_t));
  if (!ctx) {
    avberr("Stream in: no memory for context");
    return ERROR;
  }
  ctx->state = state;
  ctx->i2s_tx_handle = state->i2s_tx_handle;
  static uint32_t s_rx_start_serial = 0;
  ctx->start_serial = ++s_rx_start_serial;
  /* Ring carries stereo 24-in-32 slots: 8 B per sample frame. Used by
   * loss concealment to convert arrival gaps and presentation leads
   * into ring bytes. */
  ctx->ring_bytes_per_sec = state->media_clock.listener_sample_rate * 8;
  ctx->output_delay_ns = stream_in_output_delay_ns(state);
  memcpy(ctx->expected_stream_id, state->input_streams[index].stream_id,
         UNIQUE_ID_LEN);
  {
    /* Gate ingest on the wire subtype and rate matching the configured
     * input format — on every start, not just provisional (superfast)
     * ones: an explicitly connected talker can equally stream a
     * different rate or subtype than this listener is configured for. */
    avtp_stream_format_s *fmt = &state->input_streams[index].stream_format;
    ctx->expected_subtype = fmt->aaf_pcm.subtype;
    ctx->expected_rate_code = (fmt->aaf_pcm.subtype == avtp_subtype_61883)
                                  ? (fmt->am824.fdf_sfc & 0x07)
                                  : fmt->aaf_pcm.sample_rate;
    ctx->rate_check_pending = true;
  }

  avb_stream_in_reserve();
  if (!s_jitter_ring_buf) {
    avberr("Stream in: no memory for jitter ring (%d B, largest free block "
           "%u B)",
           JITTER_RING_SIZE,
           (unsigned)heap_caps_get_largest_free_block(MALLOC_CAP_DEFAULT));
    free(ctx);
    return ERROR;
  }
  memset(s_jitter_ring_buf, 0, JITTER_RING_SIZE);
  ctx->ring.buf = s_jitter_ring_buf;
  ctx->ring.capacity = JITTER_RING_SIZE;
  atomic_store(&ctx->ring.head, 0);
  atomic_store(&ctx->ring.tail, 0);
#if CONFIG_ESP_AVB_LISTENER_RESAMPLE
  if (avb_src_init(&ctx->src)) {
    ctx->src_in = s_src_in_buf;
    ctx->src_out = s_src_out_buf;
  } else {
    avbwarn("Stream in: no memory to resample, slipping frames instead");
  }
#endif

  s_drain_last_us = 0;
#if portNUM_PROCESSORS == 1
  if (xTaskCreate(stream_in_drain_task, "avb_drain", 3072, ctx,
                  configMAX_PRIORITIES - 1, &ctx->drain_task) != pdPASS) {
    avberr("Stream in: failed to start drain task");
    free(ctx);
    return ERROR;
  }
  state->media_clock.drain_task = ctx->drain_task;
#else
  /* Core 0, away from the talker's busy-wait on core 1, above Ethernet
   * receive (22). */
  const esp_timer_create_args_t drain_args = {
      .callback = stream_in_drain_wake_cb,
      .arg = ctx,
      .dispatch_method = ESP_TIMER_TASK,
      .name = "stream-in-drain",
      .skip_unhandled_events = true,
  };
  if (esp_timer_create(&drain_args, &ctx->drain_timer) != ESP_OK) {
    avberr("Stream in: failed to create drain timer");
    free(ctx);
    return ERROR;
  }
  if (xTaskCreatePinnedToCore(stream_in_drain_task, "avb_drain", 4096, ctx,
                              23, &ctx->drain_task, 0) != pdPASS) {
    avberr("Stream in: failed to start drain task");
    esp_timer_delete(ctx->drain_timer);
    free(ctx);
    return ERROR;
  }
  if (esp_timer_start_periodic(ctx->drain_timer, 1000) != ESP_OK) {
    avberr("Stream in: failed to start drain timer");
    stream_in_drain_task_stop(ctx);
    esp_timer_delete(ctx->drain_timer);
    free(ctx);
    return ERROR;
  }
#endif

  s_stream_rx_ctx = ctx;
  state->stream_in_active = true;
  state->input_streams[index].connected = true;

  /* Register the shared dispatcher — routes to audio stream input or CRF by stream_id */
  avb_net_set_stream_rx_handler(avb_stream_rx_dispatcher, NULL);

  avbinfo("Stream in started (ring=%d B, prefill=%d B, 1 ms drain)",
          JITTER_RING_SIZE, JITTER_PREFILL);
  return OK;
}

/* Most recent stream-frame arrival for an input stream (µs in the
 * esp_timer domain); 0 when RX is not active or has never seen a
 * frame. Used by the connected-demotion pass in avb.c. */
int64_t avb_stream_in_last_rx_us(avb_state_s *state, uint16_t index) {
  if (index == avb_get_crf_input_index(state)) {
    return (s_crf_rx_ctx && s_crf_rx_ctx->drift_latest_valid)
               ? s_crf_rx_ctx->drift_latest_rx_ts_us
               : 0;
  }
  return s_stream_rx_ctx ? s_stream_rx_ctx->last_packet_us : 0;
}

#ifdef CONFIG_ESP_AVB_SUPERFAST_CONNECT
/* Superfast connect: start stream-in provisionally right after NVS
 * restores a binding — before gPTP lock, MSRP/CVU declarations, or
 * ACMP fast-connect complete. The expected stream_id is derived from
 * the persisted talker identity by this stack's MAC+uid convention
 * (a talker using a different convention just means zero matches
 * until the ACMP response corrects it). Playout opens on the prefill
 * gate and moves onto presentation time once PTP locks — the same
 * gate handoff every normal start already exercises. If the talker
 * is not currently transmitting to us (e.g. gone longer than the
 * AVB Lite listener aging window), this idles harmlessly until the
 * handshakes land; fast-connect keeps probing while `provisional`
 * is set. */
void avb_superfast_connect_start(avb_state_s *state) {
  static const unique_id_t zero_id = {0};
  for (uint16_t i = 0; i < state->num_input_streams; i++) {
    avb_listener_stream_s *s = &state->input_streams[i];
    if (memcmp(s->talker_id, zero_id, UNIQUE_ID_LEN) == 0)
      continue;
    stream_id_from_mac((eth_addr_t *)s->talker_id, s->stream_id,
                       octets_to_uint(s->talker_uid, 2));
    /* Expect our own MAC as DA (unicast transport) until the connect
     * response provides the authoritative one. Keeps GET_RX_STATE
     * honest: connected with non-zero stream_id, dest, and vlan. */
    memcpy(s->stream_dest_addr, state->port[0].internal_mac_addr,
           ETH_ADDR_LEN);
    if (octets_to_uint(s->vlan_id, 2) == 0) {
      memcpy(s->vlan_id, state->msrp_mappings[0].vlan_id, 2);
    }
    s->provisional = true;
    if (avb_start_stream_in(state, i) == OK) {
      avbinfo("Superfast: provisional stream-in %u started "
              "(stream %02x%02x%02x%02x%02x%02x%02x%02x)",
              i, s->stream_id[0], s->stream_id[1], s->stream_id[2],
              s->stream_id[3], s->stream_id[4], s->stream_id[5],
              s->stream_id[6], s->stream_id[7]);
    } else {
      s->provisional = false;
      s->connected = false;
    }
  }
}
#endif /* CONFIG_ESP_AVB_SUPERFAST_CONNECT */

/* Stop CRF media clock input. Symmetric with avb_start_stream_in_crf. */
static void avb_stop_stream_in_crf(avb_state_s *state, uint16_t index) {
  if (!s_crf_rx_ctx)
    return;
  avbinfo("CRF stream in stopped: %lu pkts, ts=%lu, last_ts_ns=%llu, "
          "id_skip=%lu, seq_miss=%lu",
          s_crf_rx_ctx->pkt_count, s_crf_rx_ctx->timestamp_count,
          s_crf_rx_ctx->last_timestamp_ns, s_crf_rx_ctx->stream_id_mismatch,
          s_crf_rx_ctx->seq_num_mismatch);
  free(s_crf_rx_ctx);
  s_crf_rx_ctx = NULL;
  state->input_streams[index].connected = false;
  /* Unregister the dispatcher only if no other stream is active */
  if (!state->stream_in_active) {
    avb_net_set_stream_rx_handler(NULL, NULL);
  }
}

/* Stop AVB stream input — unregisters handler, stops timer, frees resources */
void avb_stop_stream_in(avb_state_s *state, uint16_t index) {

  if (index == avb_get_crf_input_index(state)) {
    avb_stop_stream_in_crf(state, index);
    return;
  }

  if (!state->stream_in_active)
    return;

  state->stream_in_active = false;

  if (s_stream_rx_ctx) {
    stream_rx_ctx_t *ctx_to_free = s_stream_rx_ctx;
    s_stream_rx_ctx = NULL; /* publishes NULL so handler short-circuits */

    /* Stop the drain before freeing anything it reads. */
    if (ctx_to_free->drain_timer) {
      esp_timer_stop(ctx_to_free->drain_timer);
      esp_timer_delete(ctx_to_free->drain_timer);
    }
    if (ctx_to_free->drain_task) {
      state->media_clock.drain_task = NULL;
      stream_in_drain_task_stop(ctx_to_free);
    }

    avbinfo("Stream in stopped: pkts=%lu ok=%lu fail=%lu drain=%lu under=%lu "
            "id_skip=%lu seq_gap=%lu locked=%lu/%lu",
            (unsigned long)ctx_to_free->pkt_count,
            (unsigned long)ctx_to_free->ring_write_ok,
            (unsigned long)ctx_to_free->ring_write_fail,
            (unsigned long)ctx_to_free->drain_count,
            (unsigned long)ctx_to_free->drain_underrun,
            (unsigned long)ctx_to_free->stream_id_mismatch,
            (unsigned long)ctx_to_free->seq_num_mismatch,
            (unsigned long)ctx_to_free->media_locked_count,
            (unsigned long)ctx_to_free->media_unlocked_count);

    free(ctx_to_free);
  }

  /* Unregister dispatcher only if no other stream is active */
  if (!s_crf_rx_ctx) {
    avb_net_set_stream_rx_handler(NULL, NULL);
  }

  // Codec stays open — shared with talker stream-out. Closed at AVB shutdown.
}

/* Stream Output (Talker) */

/* CRF media-clock output (talker). Sends one AudioSample CRF timestamp per PDU
 * using the Milan-compatible 48 kHz profile advertised in AEM.
 *
 * Paced by a 2 ms esp_timer (task dispatch), NOT a busy-wait task: at the
 * 100 Hz FreeRTOS tick a task cannot sleep for 2 ms, so the previous task spun
 * on esp_timer_get_time() at near-max priority (configMAX_PRIORITIES-2, core 1).
 * That second RT spinner alongside the audio talker could starve the gPTP loop
 * long enough to trip the AVB Lite fallback and drop the endpoint out of the
 * clock domain. The timer callback runs on the esp_timer task and only stamps +
 * transmits one frame -- the same approach ptpd uses to pace Pdelay_Req. */

#define AVB_CRF_PDU_LEN (20 + AVB_CRF_TS_PER_PDU * 8)

/* CRF timestamps are future-dated by Max Transit Time per IEEE
 * 1722-2016 §10.7 Eq. 14 (Class A budget; at 192 kHz this is exactly
 * 384 sample periods, so it stays on the media-clock grid). */
#define AVB_CRF_MAX_TRANSIT_NS 2000000
/* Re-anchor the timestamp grid if it drifts this far from gPTP now +
 * lead (esp_timer xtal vs gPTP drift, or a PTP time step). */
#define AVB_CRF_GRID_RESYNC_NS 10000000

struct avb_crf_out_ctx_s {
  esp_timer_handle_t timer;
  esp_eth_handle_t eth_handle;
  avb_state_s *state;    /* for the unicast DA mailbox */
  eth_addr_t mcast_da;   /* multicast template DA (MAAP) */
  uint16_t stream_index;
  uint8_t seq_num;
  size_t frame_len;
  uint32_t rate;         /* media clock base frequency (Hz) */
  uint32_t step_ns;      /* whole ns between consecutive CRF timestamps */
  uint32_t step_rem;     /* fractional remainder (units of 1/rate ns) */
  uint64_t next_ts_ns;   /* next grid timestamp to emit */
  uint32_t frac_acc;     /* fractional accumulator (units of 1/rate ns) */
  bool grid_valid;
  uint8_t frame[TX_ETH_HDR_LEN + TX_VLAN_TAG_LEN + AVB_CRF_PDU_LEN];
};

/* Single CRF output stream (AVB_DEFAULT_CRF_OUTPUT_INDEX). */
static struct avb_crf_out_ctx_s *s_crf_out;

static void avb_crf_send_cb(void *arg) {
  struct avb_crf_out_ctx_s *ctx = (struct avb_crf_out_ctx_s *)arg;
  uint8_t *avtp = ctx->frame + TX_ETH_HDR_LEN + TX_VLAN_TAG_LEN;

  struct timespec ts;
  if (ptpd_now(&ts) != 0)
    return;
  uint64_t now_ns =
      (uint64_t)ts.tv_sec * 1000000000ULL + (uint64_t)ts.tv_nsec;

  /* IEEE 1722-2016 §10.7 Eq. 14: CRF timestamps are a CONTINUOUS grid
   * of media-clock event times, future-dated by Max Transit Time —
   * NOT re-anchored to "now" every PDU. Listeners phase-lock to the
   * grid (media stream avtp_timestamps must align to it within ±25 %
   * of a sample period, §10.8); a per-PDU wall-clock anchor makes the
   * macOS media-clock servo run persistently fast (+8..17 % measured).
   * The grid advances by exact fractional accumulation so it never
   * drifts from base_frequency; it re-anchors only if it diverges
   * from gPTP time (xtal drift vs the 20 ms esp_timer, or PTP step). */
  uint64_t target = now_ns + AVB_CRF_MAX_TRANSIT_NS;
  if (!ctx->grid_valid ||
      (ctx->next_ts_ns > target + AVB_CRF_GRID_RESYNC_NS) ||
      (ctx->next_ts_ns + AVB_CRF_GRID_RESYNC_NS < target)) {
    /* Anchor on the media-clock sample grid (multiples of 1/rate s
     * from the gPTP epoch), rounded up from now + lead. Split into
     * seconds + ns to stay within 64-bit intermediates. */
    uint64_t sec = target / 1000000000ULL;
    uint64_t ns = target % 1000000000ULL;
    uint64_t n_in_sec =
        (ns * ctx->rate + 999999999ULL) / 1000000000ULL; /* ceil */
    uint64_t n = sec * ctx->rate + n_in_sec; /* samples since epoch */
    uint64_t rem = n % ctx->rate;
    ctx->next_ts_ns =
        (n / ctx->rate) * 1000000000ULL + (rem * 1000000000ULL) / ctx->rate;
    ctx->frac_acc = (uint32_t)((rem * 1000000000ULL) % ctx->rate);
    ctx->grid_valid = true;
  }

  avtp[2] = ctx->seq_num++;
  for (int k = 0; k < AVB_CRF_TS_PER_PDU; k++) {
    uint64_t crf_ts_ns = ctx->next_ts_ns;
    uint8_t *slot = avtp + 20 + k * 8;
    for (int i = 7; i >= 0; i--) {
      slot[i] = (uint8_t)(crf_ts_ns & 0xFF);
      crf_ts_ns >>= 8;
    }
    /* Advance exactly timestamp_interval/rate seconds. */
    ctx->next_ts_ns += ctx->step_ns;
    ctx->frac_acc += ctx->step_rem;
    if (ctx->frac_acc >= ctx->rate) {
      ctx->next_ts_ns++;
      ctx->frac_acc -= ctx->rate;
    }
  }
  /* AVB Lite unicast transport — same per-listener DA duplication as
   * the audio stream task (see stream-out TX site). */
  eth_addr_t das[CONFIG_ESP_AVB_LITE_UNICAST_FANOUT];
  int nda = avb_stream_tx_addrs_snapshot(
      &ctx->state->output_streams[ctx->stream_index], das);
  if (nda == 0) {
    memcpy(&das[0], &ctx->mcast_da, ETH_ADDR_LEN);
    nda = 1;
  }
  for (int copy = 0; copy < nda; copy++) {
    memcpy(ctx->frame, &das[copy], ETH_ADDR_LEN);
    if (avb_net_transmit_raw(ctx->eth_handle, ctx->frame, ctx->frame_len) !=
        ESP_OK) {
      /* One immediate retry — see the stream-out TX loop. */
      avb_net_transmit_raw(ctx->eth_handle, ctx->frame, ctx->frame_len);
    }
  }
}

/* Build the CRF frame header and start the 2 ms send timer. Consumes params
 * (frees it on both success and failure). */
static int avb_crf_stream_out_start(avb_state_s *state,
                                    struct stream_out_params_s *params) {
  struct avb_crf_out_ctx_s *ctx = calloc(1, sizeof(*ctx));
  if (!ctx) {
    free(params);
    return ERROR;
  }
  ctx->eth_handle = params->eth_handle;
  ctx->state = state;
  memcpy(&ctx->mcast_da, &params->dest_addr, ETH_ADDR_LEN);
  ctx->stream_index = params->stream_index;
  ctx->frame_len = TX_ETH_HDR_LEN + TX_VLAN_TAG_LEN + AVB_CRF_PDU_LEN;

  uint8_t *frame = ctx->frame;
  uint8_t *avtp = frame + TX_ETH_HDR_LEN + TX_VLAN_TAG_LEN;
  memcpy(frame, &params->dest_addr, ETH_ADDR_LEN);
  memcpy(frame + ETH_ADDR_LEN, state->port[0].internal_mac_addr, ETH_ADDR_LEN);
  frame[12] = 0x81;
  frame[13] = 0x00;
  uint16_t vid = (params->vlan_id[0] << 8) | params->vlan_id[1];
  uint16_t pcp = avb_msrp_priority_for_stream(state, params->stream_index);
  uint16_t tci = (pcp << 13) | (vid & 0x0FFF);
  frame[14] = (tci >> 8) & 0xFF;
  frame[15] = tci & 0xFF;
  frame[16] = 0x22;
  frame[17] = 0xF0;

  avtp[0] = avtp_subtype_crf;
  avtp[1] = 0x80; /* sv=1, version=0 */
  avtp[3] = 0x01; /* type = CRF_AUDIO_SAMPLE */
  memcpy(avtp + 4, &params->stream_id, UNIQUE_ID_LEN);
  /* pull=0 (1.0x). Milan CRF is always the 48 kHz reference — one
   * timestamp per PDU, interval 96, 500 PDU/s — independent of the
   * audio sampling rate (see avb_crf_format_for_rate). */
  uint32_t crf_rate = AVB_CRF_BASE_FREQ_HZ;
  uint16_t crf_interval = (uint16_t)(crf_rate / AVB_CRF_TS_PER_SEC);
  avtp[12] = (crf_rate >> 24) & 0x1F;
  avtp[13] = (crf_rate >> 16) & 0xFF;
  avtp[14] = (crf_rate >> 8) & 0xFF;
  avtp[15] = crf_rate & 0xFF;
  avtp[16] = 0x00;
  avtp[17] = AVB_CRF_TS_PER_PDU * 8; /* crf_data_length */
  avtp[18] = (crf_interval >> 8) & 0xFF;
  avtp[19] = crf_interval & 0xFF;
  /* Exact per-timestamp spacing: interval samples at base_frequency,
   * kept exact via whole-ns step + fractional remainder accumulator. */
  ctx->rate = crf_rate;
  uint64_t step_total = (uint64_t)crf_interval * 1000000000ULL;
  ctx->step_ns = (uint32_t)(step_total / crf_rate);
  ctx->step_rem = (uint32_t)(step_total % crf_rate);
  ctx->grid_valid = false;

  free(params);

  const esp_timer_create_args_t targs = {
      .callback = avb_crf_send_cb,
      .arg = ctx,
      .dispatch_method = ESP_TIMER_TASK,
      .name = "avb_crf_out",
  };
  if (esp_timer_create(&targs, &ctx->timer) != ESP_OK) {
    free(ctx);
    return ERROR;
  }
  s_crf_out = ctx;
  if (esp_timer_start_periodic(ctx->timer,
                               1000000 * AVB_CRF_TS_PER_PDU /
                                   AVB_CRF_TS_PER_SEC /* 20 ms = 50 PDU/s */)
      != ESP_OK) {
    esp_timer_delete(ctx->timer);
    free(ctx);
    s_crf_out = NULL;
    return ERROR;
  }
  return OK;
}

/* Start the AVB stream output task (talker)
 *
 * Generates a sine wave, outputs it through the ES8311 codec via I2S,
 * and simultaneously sends it as an AVTP AAF PCM stream over Ethernet.
 *
 * @param state: AVB state
 * @param index: output stream index
 * @return OK on success, ERROR on failure
 */
int avb_start_stream_out(avb_state_s *state, uint16_t index) {

  if (state->output_streams[index].streaming) {
    avberr("Stream out %d is already active", index);
    return ERROR;
  }

  /* Refuse to start if MAAP hasn't finished acquiring a dest address yet.
   * The task would lock `params->dest_addr = 0` on the first copy and
   * transmit all frames to 00:00:00:00:00:00 forever. The MSRP listener
   * handler will retry on the next Listener Ready re-declaration (every
   * MSRP_LISTENER_CONN_INTERVAL_MSEC); MAAP probe finishes in ~1.5s. */
  static const uint8_t zero_mac[ETH_ADDR_LEN] = {0};
  if (memcmp(state->output_streams[index].stream_dest_addr, zero_mac,
             ETH_ADDR_LEN) == 0) {
    avbinfo("Stream out %d deferred: MAAP address not yet acquired", index);
    return ERROR;
  }

  struct stream_out_params_s *params =
      calloc(1, sizeof(struct stream_out_params_s));
  if (!params) {
    avberr("Stream out: No memory for params");
    return ERROR;
  }

  params->state = state;
  params->stream_index = index;
  params->i2s_rx_handle = state->i2s_rx_handle;
  params->eth_handle = state->config.eth_handle;
  memcpy(&params->stream_id, state->output_streams[index].stream_id,
         UNIQUE_ID_LEN);
  memcpy(&params->dest_addr, &state->output_streams[index].stream_dest_addr,
         ETH_ADDR_LEN);
  memcpy(params->vlan_id, state->output_streams[index].vlan_id, 2);
  params->presentation_time_offset_ns =
      avb_output_presentation_offset_ns(state, index);

  // Get format parameters from the stream format in state
  avtp_stream_format_s *fmt = &state->output_streams[index].stream_format;
  params->format_subtype = fmt->aaf_pcm.subtype; // 0=61883, 2=AAF, 4=CRF

  if (params->format_subtype == avtp_subtype_crf) {
    if (avb_crf_stream_out_start(state, params) != OK) {
      return ERROR; /* helper freed params */
    }
    state->output_streams[index].stop_streaming = false;
    state->output_streams[index].streaming = true;
    avbinfo("CRF stream out %d started", index);
    return OK;
  }

  if (params->format_subtype == avtp_subtype_61883) {
    // IEC 61883-6 AM824 format
    params->cip_sfc = fmt->am824.fdf_sfc;
    params->dbs = fmt->am824.dbs;
    params->channels = fmt->am824.dbs; // dbs = channels for AM824 PCM
    params->bit_depth = 24;            // AM824 always carries 24-bit samples
    params->sample_rate = cip_sfc_to_sample_rate(fmt->am824.fdf_sfc);
  } else {
    // AAF PCM format
    params->bit_depth = fmt->aaf_pcm.bit_depth;
    params->channels =
        (fmt->aaf_pcm.chan_per_frame_h << 2) | fmt->aaf_pcm.chan_per_frame;
    params->sample_rate = aaf_code_to_sample_rate(fmt->aaf_pcm.sample_rate);
  }

  // Audio source: codec input by default, generated sine for bench tests
#ifdef CONFIG_ESP_AVB_TALKER_TEST_TONE
  params->use_sine_wave = true;
  params->sine_freq = (float)CONFIG_ESP_AVB_TALKER_TEST_TONE_HZ;
#else
  params->use_sine_wave = false;
  params->sine_freq = 1000.0f;
#endif

  // Packet interval: AM824 sends one frame each interval of its class,
  // Class A 125 us (8000 packets/sec), Class B 250 us (4000 packets/sec).
  // AAF follows its format: a frame each 125 us at Milan's cadence, each
  // 250 us at Class B's own, so its TSpec and its packets agree.
  bool class_b = state->output_streams[index].stream_info_flags.class_b;
  params->interval = class_b ? 250 : 125; // microseconds
  if (params->format_subtype == avtp_subtype_aaf) {
    uint32_t format_spf = (fmt->aaf_pcm.samples_per_frame_h << 4) |
                          fmt->aaf_pcm.samples_per_frame;
    uint32_t milan_spf = (params->sample_rate + 7999) / 8000;
    if (format_spf)
      params->interval = format_spf > milan_spf ? 250 : 125;
  }
  params->samples_per_packet =
      params->sample_rate / (1000000 / params->interval);

  // Calculate buffer size (PCM data in 32-bit containers for both formats)
  int bytes_per_sample =
      (params->bit_depth == 24) ? 4 : (params->bit_depth / 8);
  params->buffer_size =
      params->samples_per_packet * params->channels * bytes_per_sample;

  // Clear stop flag and start the stream output task
  state->output_streams[index].stop_streaming = false;
  state->output_streams[index].streaming = true;
  /* Pin to the last core: core 1 on the dual-core P4 (keeps AVB-OUT off
   * core 0 with the network stack), core 0 on single-core targets, a
   * hardcoded 1 asserts inside xTaskCreatePinnedToCore on the C6 the
   * moment a wireless talker's listener goes Ready. */
#if portNUM_PROCESSORS == 1
  /* Single core: the spin-paced TX loop must sit BELOW the Wi-Fi and
   * protocol tasks or it starves them (see the yield in the pacing
   * loop). */
  xTaskCreatePinnedToCore(avb_stream_out_task, "AVB-OUT", 8192, (void *)params,
                          tskIDLE_PRIORITY + 2, NULL, 0);
#else
  xTaskCreatePinnedToCore(avb_stream_out_task, "AVB-OUT", 8192, (void *)params,
                          configMAX_PRIORITIES - 1, NULL,
                          portNUM_PROCESSORS - 1);
#endif

  avbinfo("Stream out %d started: %s -> AVTP %s", index,
          params->use_sine_wave ? "sine wave" : "mic input",
          params->format_subtype == avtp_subtype_61883 ? "IEC 61883" : "AAF");
  return OK;
}

/* True while an audio (not CRF) output stream is streaming. */
bool avb_talker_audio_streaming(const avb_state_s *state) {
  for (size_t i = 0; i < state->num_output_streams; i++) {
    const avb_talker_stream_s *stream = &state->output_streams[i];
    if (stream->streaming &&
        stream->stream_format.aaf_pcm.subtype != avtp_subtype_crf)
      return true;
  }
  return false;
}

/* Stop the AVB stream output task */
int avb_stop_stream_out(avb_state_s *state, uint16_t index) {
  state->output_streams[index].stop_streaming = true;
  state->output_streams[index].streaming = false;
  if (s_crf_out && s_crf_out->stream_index == index) {
    /* Tear down the CRF media-clock send timer (see avb_crf_send_cb). */
    esp_timer_stop(s_crf_out->timer);
    esp_timer_delete(s_crf_out->timer);
    free(s_crf_out);
    s_crf_out = NULL;
  }
  avbinfo("Stream out %d stopped", index);
  /* The test tone lasts only while the talker streams: off with the last
   * audio stream, so the next listener never starts on a tone. */
  if (state->ctrl_test_tone && !avb_talker_audio_streaming(state)) {
    state->ctrl_test_tone = 0;
#ifdef CONFIG_ESP_AVB_ATDECC
    avb_send_aecp_unsol_control(state, AEM_CONTROL_TEST_TONE, 0);
#endif
    avbinfo("Test Tone off: talker stopped");
  }
  return OK;
}
