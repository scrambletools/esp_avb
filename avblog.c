/*
 * Copyright 2024-2026 Scramble Tools
 * License: MIT
 *
 * ESP_AVB Component — console log as an ATDECC LOG_OBJECT
 * (IEEE 1722.1-2021 Table 7-19).
 *
 * Every ESP_LOGx line also goes into a RAM ring, and the ring is served as
 * MEMORY_OBJECT 1 of type LOG_OBJECT, which a controller reads with
 * ADDRESS_ACCESS. Nothing is sent unless a controller asks.
 *
 * The object's address space, from AVB_LOG_OBJECT_BASE:
 *   0..127       one ASCII line, the header:
 *                "AVBLOG1 total=%016x drop=%08x boot=%08x ring=%08x
 *                 ptp=%016x ms=%08x" padded with spaces, '\n' at 127
 *   128..128+R   the ring; byte n of the log is at 128 + (n % R)
 * total is the bytes written since boot, drop the lines the hook threw
 * away whole, boot a per-boot ID, ring R, and ptp/ms the gPTP time (0 while
 * unsynced) and the log's ms clock, sampled when a read is answered. The
 * ring bytes of one command are copied under one lock, together with the
 * total the header reports, so the header and the data agree.
 *
 * The hook formats each line once: into the ring, then the same bytes go
 * to the app's line writer (esp_avb_log_set_line_writer) or stdout, so
 * the console output is unchanged. It never blocks: a line that finds no
 * free scratch buffer, or comes from an ISR, still reaches the console
 * through the previous vprintf but not the ring (counted in drop). A line
 * too long for the scratch buffer goes into the ring cut, and to the
 * console whole.
 *
 * The code that runs with the lock held (interrupts off on this core) is
 * in IRAM: from flash, its instruction cache misses kept the lock for tens
 * of microseconds.
 */
#include "sdkconfig.h"

#include "avb.h"

#if CONFIG_ESP_AVB_LOG_OBJECT

#include <stdarg.h>
#include <stdatomic.h>
#include <stdio.h>
#include <string.h>

#include "esp_attr.h"
#include "esp_cpu.h"
#include "esp_log.h"
#include "esp_random.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"

#define LOG_RING_SIZE (CONFIG_ESP_AVB_LOG_RING_KB * 1024)
#define LOG_LINE_MAX 512 /* longer lines are cut, keeping their newline */
#define LOG_SCRATCH_COUNT 4
#define LOG_SUMMARY_US (60 * 1000 * 1000LL)

static char s_ring[LOG_RING_SIZE];
static uint64_t s_total;   /* bytes written into the ring since boot */
static uint32_t s_write;   /* s_total % LOG_RING_SIZE */
static atomic_uint s_drop; /* lines thrown away whole */
static uint32_t s_boot_id;
static bool s_started;
static vprintf_like_t s_prev;
static esp_avb_log_line_writer_t s_line_writer;
static portMUX_TYPE s_lock = portMUX_INITIALIZER_UNLOCKED;

static char s_scratch[LOG_SCRATCH_COUNT][LOG_LINE_MAX];
static atomic_flag s_scratch_busy[LOG_SCRATCH_COUNT] = {
    ATOMIC_FLAG_INIT, ATOMIC_FLAG_INIT, ATOMIC_FLAG_INIT, ATOMIC_FLAG_INIT};

/* Cost since the last summary, in CPU cycles (under s_lock). format is
 * vsnprintf, which the console paid before the ring existed; copy and read
 * are the time the lock is held for a line and for a controller's read;
 * command is a whole ADDRESS_ACCESS command, response sent included. */
typedef struct {
  uint64_t sum;
  uint32_t max;
} cost_s;
static uint32_t s_lines;
static uint64_t s_bytes_at_summary;
static cost_s s_format, s_copy, s_read, s_command;
static uint32_t s_reads, s_commands;
static int64_t s_last_summary_us;

static IRAM_ATTR void cost_add(cost_s *cost, uint32_t cycles) {
  cost->sum += cycles;
  if (cycles > cost->max)
    cost->max = cycles;
}

/* Copies a formatted line into the ring. Called with s_lock held. */
static IRAM_ATTR void ring_put(const char *line, uint32_t len) {
  if (len > LOG_RING_SIZE)
    len = LOG_RING_SIZE;
  uint32_t first = LOG_RING_SIZE - s_write;
  if (first > len)
    first = len;
  memcpy(s_ring + s_write, line, first);
  memcpy(s_ring, line + first, len - first);
  s_write = (s_write + len) % LOG_RING_SIZE;
  s_total += len;
  s_lines++;
}

static int avb_log_vprintf(const char *format, va_list args) {
  if (xPortInIsrContext()) {
    atomic_fetch_add(&s_drop, 1);
    return s_prev(format, args);
  }
  int slot = -1;
  for (int i = 0; i < LOG_SCRATCH_COUNT; i++) {
    if (!atomic_flag_test_and_set(&s_scratch_busy[i])) {
      slot = i;
      break;
    }
  }
  if (slot < 0) {
    atomic_fetch_add(&s_drop, 1);
    return s_prev(format, args);
  }

  uint32_t start = esp_cpu_get_cycle_count();
  char *line = s_scratch[slot];
  va_list copy;
  va_copy(copy, args);
  int len = vsnprintf(line, LOG_LINE_MAX, format, copy);
  va_end(copy);
  bool whole = len >= 0 && len < LOG_LINE_MAX;
  uint32_t stored = len < 0 ? 0 : (whole ? (uint32_t)len : LOG_LINE_MAX - 1);
  if (!whole && stored > 0)
    line[stored - 1] = '\n';

  if (stored > 0) {
    uint32_t formatted = esp_cpu_get_cycle_count();
    portENTER_CRITICAL(&s_lock);
    ring_put(line, stored);
    cost_add(&s_copy, esp_cpu_get_cycle_count() - formatted);
    cost_add(&s_format, formatted - start);
    portEXIT_CRITICAL(&s_lock);
  }

  int ret;
  if (whole && s_line_writer) {
    s_line_writer(line, len); /* the console gets the bytes as formatted */
    ret = len;
  } else if (whole && s_prev == vprintf) {
    ret = (int)fwrite(line, 1, (size_t)len, stdout);
  } else {
    ret = s_prev(format, args);
  }
  atomic_flag_clear(&s_scratch_busy[slot]);
  return ret;
}

void esp_avb_log_capture_start(void) {
  if (s_started)
    return;
  s_started = true;
  s_boot_id = esp_random();
  s_last_summary_us = esp_timer_get_time();
  s_prev = esp_log_set_vprintf(avb_log_vprintf);
}

void esp_avb_log_set_line_writer(esp_avb_log_line_writer_t writer) {
  s_line_writer = writer;
}

bool avb_log_object_enabled(void) { return s_started; }

uint64_t avb_log_object_length(void) {
  portENTER_CRITICAL(&s_lock);
  uint64_t total = s_total;
  portEXIT_CRITICAL(&s_lock);
  return AVB_LOG_HEADER_LEN +
         (total < LOG_RING_SIZE ? total : (uint64_t)LOG_RING_SIZE);
}

uint64_t avb_log_object_max_length(void) {
  return AVB_LOG_HEADER_LEN + LOG_RING_SIZE;
}

/* Writes value as `digits` lowercase hex digits, zero-padded. */
static char *put_hex(char *out, uint64_t value, int digits) {
  for (int i = digits - 1; i >= 0; i--) {
    out[i] = "0123456789abcdef"[value & 0xF];
    value >>= 4;
  }
  return out + digits;
}

static char *put_str(char *out, const char *text) {
  size_t len = strlen(text);
  memcpy(out, text, len);
  return out + len;
}

static void build_header(char *header, uint64_t total, uint64_t ptp_ns,
                         uint32_t ms) {
  char *out = header;
  out = put_str(out, "AVBLOG1 total=");
  out = put_hex(out, total, 16);
  out = put_str(out, " drop=");
  out = put_hex(out, atomic_load(&s_drop), 8);
  out = put_str(out, " boot=");
  out = put_hex(out, s_boot_id, 8);
  out = put_str(out, " ring=");
  out = put_hex(out, LOG_RING_SIZE, 8);
  out = put_str(out, " ptp=");
  out = put_hex(out, ptp_ns, 16);
  out = put_str(out, " ms=");
  out = put_hex(out, ms, 8);
  memset(out, ' ', header + AVB_LOG_HEADER_LEN - 1 - out);
  header[AVB_LOG_HEADER_LEN - 1] = '\n';
}

/* The ring part of each read (offsets past the header) and the total the
 * header will report, under one lock. */
static IRAM_ATTR uint64_t ring_read(avb_log_read_s *reads, int count) {
  uint32_t start = esp_cpu_get_cycle_count();
  portENTER_CRITICAL(&s_lock);
  uint64_t total = s_total;
  for (int i = 0; i < count; i++) {
    uint64_t offset = reads[i].offset;
    uint32_t length = reads[i].length;
    uint8_t *out = reads[i].data;
    if (offset < AVB_LOG_HEADER_LEN) {
      uint32_t skip = AVB_LOG_HEADER_LEN - (uint32_t)offset;
      if (skip >= length)
        continue;
      out += skip;
      length -= skip;
      offset = AVB_LOG_HEADER_LEN;
    }
    memcpy(out, s_ring + (offset - AVB_LOG_HEADER_LEN), length);
  }
  s_reads++;
  cost_add(&s_read, esp_cpu_get_cycle_count() - start);
  portEXIT_CRITICAL(&s_lock);
  return total;
}

int avb_log_object_read(avb_log_read_s *reads, int count, uint64_t ptp_ns) {
  uint64_t end = AVB_LOG_HEADER_LEN + (uint64_t)LOG_RING_SIZE;
  for (int i = 0; i < count; i++) {
    if (reads[i].offset > end || reads[i].length > end - reads[i].offset)
      return AVB_LOG_READ_TOO_HIGH;
  }
  uint32_t ms = esp_log_timestamp();
  uint64_t total = ring_read(reads, count);

  char header[AVB_LOG_HEADER_LEN];
  build_header(header, total, ptp_ns, ms);
  for (int i = 0; i < count; i++) {
    if (reads[i].offset >= AVB_LOG_HEADER_LEN)
      continue;
    uint32_t take = AVB_LOG_HEADER_LEN - (uint32_t)reads[i].offset;
    if (take > reads[i].length)
      take = reads[i].length;
    memcpy(reads[i].data, header + reads[i].offset, take);
  }
  return AVB_LOG_READ_OK;
}

void avb_log_object_note_command(uint32_t cycles) {
  portENTER_CRITICAL(&s_lock);
  s_commands++;
  cost_add(&s_command, cycles);
  portEXIT_CRITICAL(&s_lock);
}

/* "avg/max" in microseconds with one decimal, from cycles. */
static void put_cost(char *out, size_t size, const cost_s *cost,
                     uint32_t count) {
  uint32_t mhz = CONFIG_ESP_DEFAULT_CPU_FREQ_MHZ;
  uint32_t avg = count ? (uint32_t)(cost->sum * 10 / count / mhz) : 0;
  uint32_t max = (uint32_t)((uint64_t)cost->max * 10 / mhz);
  snprintf(out, size, "%lu.%lu/%lu.%lu", (unsigned long)(avg / 10),
           (unsigned long)(avg % 10), (unsigned long)(max / 10),
           (unsigned long)(max % 10));
}

void avb_log_object_tick(void) {
  if (!s_started)
    return;
  int64_t now_us = esp_timer_get_time();
  int64_t elapsed_us = now_us - s_last_summary_us;
  if (elapsed_us < LOG_SUMMARY_US)
    return;
  s_last_summary_us = now_us;

  portENTER_CRITICAL(&s_lock);
  uint32_t lines = s_lines;
  uint64_t bytes = s_total - s_bytes_at_summary;
  cost_s format = s_format, copy = s_copy, read = s_read;
  cost_s command = s_command;
  uint32_t reads = s_reads, commands = s_commands;
  s_lines = 0;
  s_bytes_at_summary = s_total;
  s_format = s_copy = s_read = s_command = (cost_s){0};
  s_reads = s_commands = 0;
  portEXIT_CRITICAL(&s_lock);

  char format_us[24], copy_us[24], read_us[24], command_us[24];
  put_cost(format_us, sizeof(format_us), &format, lines);
  put_cost(copy_us, sizeof(copy_us), &copy, lines);
  put_cost(read_us, sizeof(read_us), &read, reads);
  put_cost(command_us, sizeof(command_us), &command, commands);
  avbinfo("LOG-OBJECT: %lu s: lines=%lu bytes=%llu dropped=%u format=%sus "
          "copy=%sus reads=%lu read=%sus aa=%lu aa_cmd=%sus",
          (unsigned long)(elapsed_us / 1000000), (unsigned long)lines,
          (unsigned long long)bytes, atomic_load(&s_drop), format_us, copy_us,
          (unsigned long)reads, read_us, (unsigned long)commands, command_us);
}

#else /* !CONFIG_ESP_AVB_LOG_OBJECT */

void esp_avb_log_capture_start(void) {}

void esp_avb_log_set_line_writer(esp_avb_log_line_writer_t writer) {
  (void)writer;
}

#endif
