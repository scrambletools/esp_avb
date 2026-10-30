/*
 * Copyright 2024-2026 Scramble Tools
 * License: MIT
 *
 * ESP_AVB Component
 *
 * Sample-rate converter: polyphase Kaiser-windowed sinc with linear
 * interpolation between phases, fixed point. See avbsrc.h.
 */

#include "avbsrc.h"

#include <math.h>
#include <stdlib.h>
#include <string.h>

#ifdef ESP_PLATFORM
#include "esp_attr.h"
#else
#define IRAM_ATTR
#endif

/* Cutoff (fraction of the input rate) and Kaiser beta. */
#define SRC_CUTOFF 0.44f
#define SRC_BETA 9.0f

/* (AVB_SRC_PHASES + 1) rows of AVB_SRC_TAPS Q30 coefficients. Row p
 * weights the history oldest first for an output p / AVB_SRC_PHASES of a
 * frame past the window's centre; the last row is the first shifted by
 * one input, so the phase interpolation is continuous across inputs.
 * Static: a station's first listener start needs a 64 KB block of heap,
 * and what its talker starts keep fragments it. */
static int32_t s_src_table[(AVB_SRC_PHASES + 1) * AVB_SRC_TAPS];
static bool s_src_table_ready;

static float bessel_i0(float value) {
  float sum = 1.0f, term = 1.0f;
  float half = value / 2.0f;
  for (int k = 1; k < 30; k++) {
    term *= half / (float)k;
    sum += term * term;
  }
  return sum;
}

static bool src_table_build(void) {
  const int taps = AVB_SRC_TAPS, phases = AVB_SRC_PHASES;
  const int len = taps * phases + 1;
  float *proto = malloc(len * sizeof(float));
  if (!proto)
    return false;
  float beta_norm = bessel_i0(SRC_BETA);
  for (int n = 0; n < len; n++) {
    float t = ((float)n - (float)(len - 1) / 2.0f) / (float)phases;
    float arg = 2.0f * SRC_CUTOFF * t;
    float sinc = arg == 0.0f ? 1.0f : sinf((float)M_PI * arg) / ((float)M_PI * arg);
    float edge = 2.0f * (float)n / (float)(len - 1) - 1.0f;
    float window = bessel_i0(SRC_BETA * sqrtf(fmaxf(0.0f, 1.0f - edge * edge))) /
                   beta_norm;
    proto[n] = 2.0f * SRC_CUTOFF * sinc * window;
  }
  /* Zero ends keep the first and last rows one input apart exactly. */
  proto[0] = proto[len - 1] = 0.0f;
  /* Unity gain at every phase: a phase whose taps summed above or below
   * 1 modulated the level as the phase swept. */
  for (int phase = 0; phase < phases; phase++) {
    float sum = 0.0f;
    for (int tap = 0; tap < taps; tap++)
      sum += proto[tap * phases + phase];
    for (int tap = 0; tap < taps; tap++)
      proto[tap * phases + phase] /= sum;
  }
  for (int row = 0; row <= phases; row++)
    for (int oldest = 0; oldest < taps; oldest++)
      s_src_table[row * taps + oldest] = (int32_t)lroundf(
          proto[(taps - 1 - oldest) * phases + row] * (float)(1 << 30));
  free(proto);
  s_src_table_ready = true;
  return true;
}

bool avb_src_init(avb_src_s *src) {
  memset(src, 0, sizeof(*src));
  return s_src_table_ready || src_table_build();
}

#define SRC_ONE (1LL << 32)

static inline void src_push(avb_src_s *src, const uint8_t *frame, int slots) {
  for (int slot = 0; slot < slots; slot++) {
    int32_t sample = 0;
    if (frame) {
      const uint8_t *bytes = frame + slot * 4;
      sample = (int32_t)(((uint32_t)bytes[0] << 24) |
                         ((uint32_t)bytes[1] << 16) | ((uint32_t)bytes[2] << 8));
    }
    src->hist[slot][src->hist_pos] = sample;
    src->hist[slot][src->hist_pos + AVB_SRC_TAPS] = sample;
  }
  if (++src->hist_pos == AVB_SRC_TAPS)
    src->hist_pos = 0;
}

void avb_src_prime(avb_src_s *src, const uint8_t *in, int slots,
                   int frame_bytes) {
  if (slots > AVB_SRC_SLOTS)
    slots = AVB_SRC_SLOTS;
  /* Each output takes one more input and plays the frame AVB_SRC_TAPS / 2
   * behind it. */
  for (int frame = 0; frame < AVB_SRC_TAPS; frame++)
    src_push(src, in + frame * frame_bytes, slots);
  src->pos.frac = 0;
  src->pos.walk_left = 0;
}

/* The next output's position change: zero between walks, the walk's step
 * while one runs. A walk starts once a whole frame is owed. */
static inline int32_t src_next_step(avb_src_pos_s *pos, int32_t rate_q32) {
  pos->owed += rate_q32;
  /* A ratio beyond what walks can pay leaves at most a few frames owed. */
  if (pos->owed > 4 * SRC_ONE)
    pos->owed = 4 * SRC_ONE;
  else if (pos->owed < -4 * SRC_ONE)
    pos->owed = -4 * SRC_ONE;
  if (pos->walk_left == 0) {
    int32_t sign;
    if (pos->owed >= SRC_ONE || pos->owed <= -SRC_ONE) {
      sign = pos->owed > 0 ? 1 : -1;
      pos->owed -= sign * SRC_ONE;
    } else if (pos->extra) {
      sign = pos->extra > 0 ? 1 : -1;
      pos->extra -= sign;
    } else {
      return 0;
    }
    pos->walk_step = sign * (int32_t)(SRC_ONE / AVB_SRC_WALK_FRAMES);
    pos->walk_left = AVB_SRC_WALK_FRAMES;
  }
  pos->walk_left--;
  return pos->walk_step;
}

/* Inputs one output takes at this step, and the position after it. */
static inline int src_advance(avb_src_pos_s *pos, int32_t step) {
  int64_t next = (int64_t)pos->frac + SRC_ONE + step;
  pos->frac = (uint32_t)next;
  return (int)(next >> 32);
}

void avb_src_owe(avb_src_s *src, int32_t frames) { src->pos.extra += frames; }

IRAM_ATTR int avb_src_inputs(const avb_src_s *src, int out_frames,
                             int32_t rate_q32) {
  avb_src_pos_s pos = src->pos;
  int inputs = 0;
  for (int frame = 0; frame < out_frames; frame++)
    inputs += src_advance(&pos, src_next_step(&pos, rate_q32));
  return inputs;
}

static inline int32_t mul_high(int32_t lhs, int32_t rhs) {
  return (int32_t)(((int64_t)lhs * rhs) >> 32);
}

/* In IRAM: a station runs it for every sample in both directions, out
 * of a flash cache it shares with the Wi-Fi driver. */
IRAM_ATTR void avb_src_run(avb_src_s *src, const uint8_t *in, int in_frames,
                           uint8_t *out, int out_frames, int32_t rate_q32,
                           int slots, int frame_bytes) {
  const int taps = AVB_SRC_TAPS;
  int used = 0;
  if (slots > AVB_SRC_SLOTS)
    slots = AVB_SRC_SLOTS;
  for (int frame = 0; frame < out_frames; frame++) {
    bool walk_was_idle = src->pos.walk_left == 0;
    int32_t step = src_next_step(&src->pos, rate_q32);
    if (walk_was_idle && step != 0)
      src->walks++;
    for (int push = src_advance(&src->pos, step); push > 0; push--) {
      src_push(src, used < in_frames ? in + used * frame_bytes : NULL, slots);
      used++;
    }
    int32_t acc[AVB_SRC_SLOTS] = {0};
    if (src->pos.frac == 0) {
      /* On an input: the window's centre, as it came (Q31 to Q29). */
      for (int slot = 0; slot < slots; slot++)
        acc[slot] = src->hist[slot][src->hist_pos + taps / 2 - 1] >> 2;
    } else {
      uint32_t phase = src->pos.frac >> (32 - AVB_SRC_PHASES_LOG2);
      int32_t between =
          (int32_t)((src->pos.frac << AVB_SRC_PHASES_LOG2) >> 1);
      const int32_t *row = s_src_table + phase * taps;
      const int32_t *left = &src->hist[0][src->hist_pos];
      const int32_t *right = &src->hist[1][src->hist_pos];
      /* Each tap's coefficient interpolated between the two phases,
       * then applied to every slot. */
      if (slots == 1) {
        for (int tap = 0; tap < taps; tap++) {
          int32_t coeff =
              row[tap] + mul_high(row[tap + taps] - row[tap], between) * 2;
          acc[0] += mul_high(coeff, left[tap]);
        }
      } else {
        for (int tap = 0; tap < taps; tap++) {
          int32_t coeff =
              row[tap] + mul_high(row[tap + taps] - row[tap], between) * 2;
          acc[0] += mul_high(coeff, left[tap]);
          acc[1] += mul_high(coeff, right[tap]);
        }
      }
    }
    uint8_t *frame_out = out + frame * frame_bytes;
    for (int slot = 0; slot < frame_bytes / 4; slot++) {
      /* Q29 to 24 bits, rounded and clipped. */
      int32_t sample = slot < slots ? (acc[slot] + 32) >> 6 : 0;
      if (sample > 8388607)
        sample = 8388607;
      else if (sample < -8388608)
        sample = -8388608;
      uint8_t *bytes = frame_out + slot * 4;
      bytes[0] = (uint8_t)(sample >> 16);
      bytes[1] = (uint8_t)(sample >> 8);
      bytes[2] = (uint8_t)sample;
      bytes[3] = 0;
    }
  }
}
