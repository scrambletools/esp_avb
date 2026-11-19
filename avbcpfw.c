/*
 * Copyright 2024-2026 Scramble Tools
 * License: MIT
 *
 * ESP_AVB Component — a coprocessor's firmware over ATDECC, the IEEE
 * 1722.1-2021 Annex D way, as a second FIRMWARE_IMAGE memory object from
 * AVB_COPROCESSOR_FW_OBJECT_BASE.
 *
 * The app registers how to store and start an image
 * (esp_avb_register_coprocessor_fw). A controller sends START_OPERATION
 * UPLOAD with the image length, the plain .bin in ADDRESS_ACCESS WRITEs in
 * order, then STORE or STORE_AND_REBOOT:
 *   - the upload goes to a PSRAM buffer, so no write waits on the
 *     coprocessor and this chip's flash is never touched;
 *   - a repeat of the last chunk (a retry after a lost response) is
 *     answered SUCCESS, other writes out of order get ADDRESS_INVALID,
 *     writes past the length ADDRESS_TOO_HIGH;
 *   - STORE hands the image to the app's store() in a task, then
 *     activate() switches the coprocessor to it; progress and the result
 *     go out as OPERATION_STATUS (1 to 999, then 1000 or 0);
 *   - after both passed the entity departs and the board restarts, for
 *     STORE too.
 * EFU_MODE is advertised from UPLOAD until the restart, an abort or a
 * failure. Streams keep running through the upload: nothing in it writes
 * flash here.
 */
#include "sdkconfig.h"

#include "avb.h"

#if CONFIG_ESP_AVB_COPROCESSOR_FW_OBJECT

#include "esp_heap_caps.h"
#include "esp_system.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#define CPFW_MIN_LENGTH 32 /* an ESP image header and more */
#define CPFW_PROGRESS_PERIOD_US (1000 * 1000LL)
#define CPFW_RESTART_DELAY_US (300 * 1000LL)

typedef enum {
  cpfw_idle,
  cpfw_uploading,
  cpfw_storing,
  cpfw_restarting,
} cpfw_phase_t;

static struct {
  esp_avb_coprocessor_fw_t target;
  bool registered;
  volatile cpfw_phase_t phase;
  uint8_t *image; /* PSRAM, image_length bytes */
  uint32_t image_length;
  uint32_t written;
  uint32_t last_offset, last_length; /* the last chunk, for repeats */
  uint16_t next_op_id;
  uint16_t upload_op, store_op;
  volatile int store_result;  /* -1 running, 0 failed, 1 passed */
  volatile uint16_t progress; /* permille from store() */
  uint16_t progress_sent;
  int64_t progress_sent_us;
  int64_t restart_at_us;
} s_cpfw;

esp_err_t esp_avb_register_coprocessor_fw(const esp_avb_coprocessor_fw_t *fw) {
  if (fw == NULL || fw->name == NULL || fw->store == NULL ||
      fw->activate == NULL || fw->max_length < CPFW_MIN_LENGTH)
    return ESP_ERR_INVALID_ARG;
  if (s_cpfw.phase != cpfw_idle)
    return ESP_ERR_INVALID_STATE;
  s_cpfw.target = *fw;
  s_cpfw.registered = true;
  return ESP_OK;
}

bool avb_cpfw_object_enabled(void) { return s_cpfw.registered; }

const char *avb_cpfw_object_name(void) { return s_cpfw.target.name; }

uint64_t avb_cpfw_object_length(void) {
  return s_cpfw.phase == cpfw_restarting ? s_cpfw.image_length : 0;
}

uint64_t avb_cpfw_object_max_length(void) { return s_cpfw.target.max_length; }

static uint16_t new_op_id(void) {
  if (++s_cpfw.next_op_id == 0)
    s_cpfw.next_op_id = 1;
  return s_cpfw.next_op_id;
}

static void set_efu_mode(avb_state_s *state, bool on) {
  avb_entity_cap_s caps;
  memcpy(&caps, &state->own_entity.summary.entity_capabilities, sizeof(caps));
  if (caps.efu_mode == on)
    return;
  caps.efu_mode = on;
  memcpy(&state->own_entity.summary.entity_capabilities, &caps, sizeof(caps));
  avb_send_adp_entity_available(state);
}

static void release_image(void) {
  heap_caps_free(s_cpfw.image);
  s_cpfw.image = NULL;
}

static void stop_upload(avb_state_s *state, const char *why) {
  s_cpfw.phase = cpfw_idle;
  release_image();
  set_efu_mode(state, false);
  avbwarn("Coprocessor firmware: upload ended: %s", why);
}

aecp_status_t avb_cpfw_start_upload(avb_state_s *state, uint64_t length,
                                    uint16_t *op_id) {
  if (s_cpfw.phase == cpfw_storing || s_cpfw.phase == cpfw_restarting)
    return aecp_status_in_progress;
  if (length < CPFW_MIN_LENGTH || length > s_cpfw.target.max_length)
    return aecp_status_bad_arguments;
  if (s_cpfw.phase != cpfw_idle)
    stop_upload(state, "replaced by a new upload");
  s_cpfw.image = heap_caps_malloc((size_t)length,
                                  MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
  if (s_cpfw.image == NULL) {
    avberr("Coprocessor firmware: no PSRAM for %lu bytes",
           (unsigned long)length);
    return aecp_status_no_resources;
  }
  s_cpfw.image_length = (uint32_t)length;
  s_cpfw.written = 0;
  s_cpfw.last_offset = s_cpfw.last_length = 0;
  s_cpfw.phase = cpfw_uploading;
  s_cpfw.upload_op = new_op_id();
  *op_id = s_cpfw.upload_op;
  set_efu_mode(state, true);
  avbwarn("Coprocessor firmware: upload of %lu bytes started (op %u)",
          (unsigned long)length, s_cpfw.upload_op);
  return aecp_status_success;
}

int avb_cpfw_write(uint64_t offset, const uint8_t *data, uint32_t length) {
  if (s_cpfw.phase != cpfw_uploading)
    return AVB_FW_WRITE_INVALID;
  if (offset == s_cpfw.last_offset && length == s_cpfw.last_length &&
      s_cpfw.written == offset + length && length > 0)
    return AVB_FW_WRITE_OK; /* the controller's retry of the last chunk */
  if (offset != s_cpfw.written)
    return AVB_FW_WRITE_INVALID;
  if (offset + length > s_cpfw.image_length)
    return AVB_FW_WRITE_TOO_HIGH;
  memcpy(s_cpfw.image + offset, data, length);
  s_cpfw.last_offset = (uint32_t)offset;
  s_cpfw.last_length = length;
  s_cpfw.written = (uint32_t)offset + length;
  return AVB_FW_WRITE_OK;
}

static void report_progress(uint16_t permille) {
  if (permille < 1)
    permille = 1;
  if (permille > 999)
    permille = 999;
  s_cpfw.progress = permille;
}

static void store_task(void *arg) {
  esp_err_t err = s_cpfw.target.store(s_cpfw.image, s_cpfw.image_length,
                                      report_progress, s_cpfw.target.context);
  if (err != ESP_OK) {
    avberr("Coprocessor firmware: store failed: %s", esp_err_to_name(err));
  } else {
    err = s_cpfw.target.activate(s_cpfw.target.context);
    if (err != ESP_OK)
      avberr("Coprocessor firmware: activate failed: %s",
             esp_err_to_name(err));
  }
  s_cpfw.store_result = err == ESP_OK ? 1 : 0;
  vTaskDelete(NULL);
}

aecp_status_t avb_cpfw_start_store(avb_state_s *state, uint16_t *op_id) {
  if (s_cpfw.phase != cpfw_uploading || s_cpfw.written != s_cpfw.image_length)
    return aecp_status_bad_arguments;
  s_cpfw.phase = cpfw_storing;
  s_cpfw.store_op = new_op_id();
  *op_id = s_cpfw.store_op;
  s_cpfw.store_result = -1;
  s_cpfw.progress = 1;
  s_cpfw.progress_sent = 0;
  s_cpfw.progress_sent_us = 0;
  /* Low priority on core 0, beside the AVB task: store() waits on the
   * coprocessor for most of its run. */
  if (xTaskCreatePinnedToCore(store_task, "avb_cpfw_store", 4096, NULL, 2,
                              NULL, 0) != pdPASS) {
    s_cpfw.phase = cpfw_uploading;
    return aecp_status_no_resources;
  }
  avbwarn("Coprocessor firmware: storing %lu bytes (op %u)",
          (unsigned long)s_cpfw.image_length, s_cpfw.store_op);
  return aecp_status_success;
}

aecp_status_t avb_cpfw_abort(avb_state_s *state, uint16_t op_id) {
  if (s_cpfw.phase != cpfw_uploading || op_id != s_cpfw.upload_op)
    return aecp_status_bad_arguments;
  stop_upload(state, "aborted");
  return aecp_status_success;
}

void avb_cpfw_tick(avb_state_s *state) {
  if (!s_cpfw.registered)
    return;
  int64_t now = esp_timer_get_time();
  uint16_t index = (uint16_t)avb_cpfw_object_index();

  if (s_cpfw.phase == cpfw_storing && s_cpfw.store_result < 0) {
    uint16_t progress = s_cpfw.progress;
    if (progress != s_cpfw.progress_sent &&
        now - s_cpfw.progress_sent_us >= CPFW_PROGRESS_PERIOD_US) {
      avb_send_operation_status(state, aem_desc_type_memory_object, index,
                                s_cpfw.store_op, progress);
      s_cpfw.progress_sent = progress;
      s_cpfw.progress_sent_us = now;
    }
  }

  if (s_cpfw.phase == cpfw_storing && s_cpfw.store_result >= 0) {
    bool passed = s_cpfw.store_result == 1;
    uint16_t percent = passed ? 1000 : 0;
    avb_send_operation_status(state, aem_desc_type_memory_object, index,
                              s_cpfw.store_op, percent);
    avb_send_operation_status(state, aem_desc_type_memory_object, index,
                              s_cpfw.upload_op, percent);
    release_image();
    if (passed) {
      s_cpfw.phase = cpfw_restarting;
      s_cpfw.restart_at_us = now + CPFW_RESTART_DELAY_US;
      avb_send_adp_entity_departing(state);
    } else {
      s_cpfw.phase = cpfw_idle;
      set_efu_mode(state, false);
    }
  }

  if (s_cpfw.phase == cpfw_restarting && now >= s_cpfw.restart_at_us) {
    avbwarn("Coprocessor firmware: restarting with the new image");
    esp_restart();
  }
}

#else /* !CONFIG_ESP_AVB_COPROCESSOR_FW_OBJECT */

esp_err_t esp_avb_register_coprocessor_fw(const esp_avb_coprocessor_fw_t *fw) {
  return ESP_ERR_NOT_SUPPORTED;
}

#endif /* CONFIG_ESP_AVB_COPROCESSOR_FW_OBJECT */
