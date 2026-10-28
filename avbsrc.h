/*
 * Copyright 2024-2026 Scramble Tools
 * License: MIT
 *
 * ESP_AVB Component
 *
 * Sample-rate converter for a codec whose clock cannot be trimmed: moves
 * audio between the codec's frame rate and the stream's at a ratio close
 * to 1, set per call. Frames are I2S frames of 24-in-32 big-endian slots
 * ([MSB MID LSB 00] per slot).
 *
 * The audio passes through unchanged while the ratio's running total
 * stays under a frame. Each whole frame it owes is paid by a walk: the
 * output position moves one frame against the input over
 * AVB_SRC_WALK_FRAMES, through a polyphase fractional-delay filter.
 */

#ifndef ESP_AVB_AVBSRC_H_
#define ESP_AVB_AVBSRC_H_

#include <stdbool.h>
#include <stdint.h>

/* Taps per output sample, and the interpolated phases between inputs.
 * 16 taps keep the error below -90 dB to 10 kHz and the response flat
 * within 0.1 dB to 15 kHz at any phase (48 kHz). */
#define AVB_SRC_TAPS 16
#define AVB_SRC_PHASES_LOG2 6
#define AVB_SRC_PHASES (1 << AVB_SRC_PHASES_LOG2)
#define AVB_SRC_SLOTS 2
/* A walk's length: 85 ms at 48 kHz, a 244 ppm change of rate while it
 * runs (0.4 cent), and the fastest ratio it can follow. */
#define AVB_SRC_WALK_FRAMES 4096

/* The converter's position and what it owes. */
typedef struct {
  /* Output position past the newest input, in input frames (Q32); zero
   * between walks. */
  uint32_t frac;
  /* Frames owed to the ratio (Q32): positive takes an extra input. */
  int64_t owed;
  int32_t walk_step; /* per-frame position change of the walk in progress */
  int32_t walk_left; /* its frames still to run */
  int32_t extra;     /* whole frames to take (+) or give back (-) by walks */
} avb_src_pos_s;

typedef struct {
  /* Input history per slot, written twice so the newest AVB_SRC_TAPS
   * are always contiguous from hist_pos. */
  int32_t hist[AVB_SRC_SLOTS][2 * AVB_SRC_TAPS];
  int hist_pos;
  avb_src_pos_s pos;
  uint32_t walks; /* walks started */
} avb_src_s;

/* Builds the shared coefficient table on first use; false if out of
 * memory. */
bool avb_src_init(avb_src_s *src);

/* Loads AVB_SRC_TAPS input frames into the history, the next output
 * then being the one in the middle (index AVB_SRC_TAPS / 2): the frames
 * before it may be ones already played, the rest count as consumed. */
void avb_src_prime(avb_src_s *src, const uint8_t *in, int slots,
                   int frame_bytes);

/* Takes frames more (positive) or fewer input frames than the ratio
 * asks, one walk each, between the walks the ratio owes. */
void avb_src_owe(avb_src_s *src, int32_t frames);

/* Input frames the next out_frames outputs consume at a ratio of
 * 1 + rate_q32 / 2^32 inputs per output. */
int avb_src_inputs(const avb_src_s *src, int out_frames, int32_t rate_q32);

/* Converts in_frames (exactly avb_src_inputs for the same arguments)
 * into out_frames. Only the first slots slots are converted; later slots
 * of each output frame are zero. */
void avb_src_run(avb_src_s *src, const uint8_t *in, int in_frames,
                 uint8_t *out, int out_frames, int32_t rate_q32, int slots,
                 int frame_bytes);

/* Rate for a trim in ppm x 256: positive ppm consumes the input slower. */
static inline int32_t avb_src_rate_from_ppm_q8(int32_t ppm_q8) {
  /* 2^32 / (256 x 1e6) = 16.777216 */
  return (int32_t)(-((int64_t)ppm_q8 * 16777216LL) / 1000000LL);
}

#endif /* ESP_AVB_AVBSRC_H_ */
