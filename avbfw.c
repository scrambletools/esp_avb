/*
 * Copyright 2024-2026 Scramble Tools
 * License: MIT
 *
 * ESP_AVB Component — firmware update over ATDECC, the IEEE 1722.1-2021
 * Annex D way (as Hive does it), onto the ESP-IDF OTA slots.
 *
 * The inactive OTA slot is a FIRMWARE_IMAGE memory object from
 * AVB_FIRMWARE_OBJECT_BASE. A controller sends START_OPERATION UPLOAD with
 * the image length, writes the plain ESP .bin with ADDRESS_ACCESS WRITEs in
 * order, then START_OPERATION STORE or STORE_AND_REBOOT:
 *   - UPLOAD is answered at once; a low-priority task erases the slot ahead
 *     of the writes in 64 KB blocks, so a write only programs flash;
 *   - a repeat of the last chunk (a retry after a lost response) is answered
 *     SUCCESS without writing it twice; other writes out of order get
 *     ADDRESS_INVALID, writes past the length ADDRESS_TOO_HIGH;
 *   - STORE checks the image in a task (chip, project name, then
 *     esp_ota_set_boot_partition: SHA-256 and, in signed builds, the
 *     signature against the running image's keys) and reports through
 *     OPERATION_STATUS: progress from 1, 1000 done, 0 failed. After
 *     STORE_AND_REBOOT the board sends ADP ENTITY_DEPARTING and restarts.
 * EFU_MODE is advertised from UPLOAD until the reboot, an abort or a
 * failure.
 *
 * While a talker sends or a listener receives, UPLOAD gets STREAM_IS_RUNNING
 * unless the controller holds the entity (LOCK_ENTITY or ACQUIRE_ENTITY):
 * then the board stops its streams itself, in RAM only, and starts them
 * again if the lock goes with no reboot pending.
 * CONFIG_ESP_AVB_FW_UPLOAD_WHILE_STREAMING keeps them running instead.
 *
 * New firmware keeps itself once gPTP is valid; if not within 60 s of boot
 * it restarts and the bootloader goes back to the old one.
 */
#include "sdkconfig.h"

#include "avb.h"

#if CONFIG_ESP_AVB_FIRMWARE_OBJECT

#include "esp_app_desc.h"
#include "esp_app_format.h"
#include "esp_image_format.h"
#include "esp_ota_ops.h"
#include "esp_partition.h"
#include "esp_system.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "freertos/task.h"

#define FW_ERASE_BLOCK (64 * 1024)
#define FW_SECTOR 4096
#define FW_WRITE_WAIT_MS 2000   /* a write waiting for the eraser */
#define FW_VALID_DEADLINE_US (60 * 1000 * 1000LL)
#define FW_VALID_MIN_US (5 * 1000 * 1000LL)
#if CONFIG_SECURE_SIGNED_ON_UPDATE
#define FW_SIGNATURE_NOTE " or signature"
#else
#define FW_SIGNATURE_NOTE ""
#endif

typedef enum {
  fw_idle,
  fw_uploading,
  fw_storing,
  fw_stored,  /* checked and set to boot, waiting for a reboot */
  fw_rebooting,
} fw_phase_t;

static struct {
  const esp_partition_t *slot;
  volatile fw_phase_t phase;
  uint32_t image_length;
  uint32_t written;
  uint32_t last_offset, last_length; /* the last chunk, for repeats */
  volatile uint32_t erased;          /* bytes of the slot erased so far */
  SemaphoreHandle_t erased_sem;      /* given after each erased block */
  TaskHandle_t eraser;
  uint16_t next_op_id;
  uint16_t upload_op, store_op;
  bool reboot_after_store;
  volatile int store_result; /* -1 running, 0 failed, 1 passed */
  bool paused;               /* streams stopped for the upload */
  bool paused_input[AVB_MAX_NUM_INPUT_STREAMS];
  bool pending_verify;       /* this boot runs an image not yet kept */
  int64_t reboot_at_us;
} s_fw;

static uint16_t new_op_id(void) {
  if (++s_fw.next_op_id == 0)
    s_fw.next_op_id = 1;
  return s_fw.next_op_id;
}

void avb_fw_init(void) {
  s_fw.slot = esp_ota_get_next_update_partition(NULL);
  if (s_fw.slot == NULL) {
    avbinfo("Firmware: no second OTA slot, no updates over ATDECC");
    return;
  }
  s_fw.erased_sem = xSemaphoreCreateBinary();
  const esp_partition_t *running = esp_ota_get_running_partition();
  esp_ota_img_states_t img_state;
  if (running &&
      esp_ota_get_state_partition(running, &img_state) == ESP_OK &&
      img_state == ESP_OTA_IMG_PENDING_VERIFY) {
    s_fw.pending_verify = true;
    avbwarn("Firmware: new image pending verify (kept once gPTP is valid, "
            "rolled back if not within 60 s)");
  }
}

bool avb_fw_object_enabled(void) { return s_fw.slot != NULL; }

uint64_t avb_fw_object_max_length(void) {
  return s_fw.slot ? s_fw.slot->size : 0;
}

uint64_t avb_fw_object_length(void) {
  return (s_fw.phase == fw_stored || s_fw.phase == fw_rebooting)
             ? s_fw.image_length
             : 0;
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

static bool streams_running(avb_state_s *state) {
  for (int i = 0; i < state->num_output_streams; i++)
    if (state->output_streams[i].streaming)
      return true;
  return state->stream_in_active;
}

static void pause_streams(avb_state_s *state) {
  s_fw.paused = true; /* from here avb_start_stream_* refuse */
  for (int i = 0; i < state->num_output_streams; i++)
    if (state->output_streams[i].streaming)
      avb_stop_stream_out(state, i);
  uint16_t crf = avb_get_crf_input_index(state);
  for (int i = 0; i < state->num_input_streams; i++) {
    s_fw.paused_input[i] = false;
    if (i != crf && state->stream_in_active &&
        state->input_streams[i].connected) {
      avb_stop_stream_in(state, i);
      s_fw.paused_input[i] = true;
    }
  }
  avbwarn("Firmware: streams stopped for the upload");
}

static void resume_streams(avb_state_s *state) {
  if (!s_fw.paused)
    return;
  s_fw.paused = false;
  for (int i = 0; i < state->num_input_streams; i++) {
    if (s_fw.paused_input[i] && state->input_streams[i].connected)
      avb_start_stream_in(state, i);
    s_fw.paused_input[i] = false;
  }
  /* Talkers start again at their listeners' next declaration. */
  avbwarn("Firmware: streams started again");
}

bool avb_fw_streams_paused(void) { return s_fw.paused; }

static void eraser_task(void *arg) {
  /* One sector past the image too: no stale signature sector from the
   * slot's last image may follow the new one. */
  uint32_t end = (s_fw.image_length + FW_SECTOR - 1) / FW_SECTOR * FW_SECTOR +
                 FW_SECTOR;
  if (end > s_fw.slot->size)
    end = s_fw.slot->size;
  while (s_fw.phase == fw_uploading && s_fw.erased < end) {
    uint32_t start = s_fw.erased;
    /* 64 KB blocks where aligned (one block erase), sectors elsewhere. */
    uint32_t size = (start % FW_ERASE_BLOCK == 0 && end - start >= FW_ERASE_BLOCK)
                        ? FW_ERASE_BLOCK
                        : FW_SECTOR;
    if (esp_partition_erase_range(s_fw.slot, start, size) != ESP_OK) {
      avberr("Firmware: erase at %lu failed", (unsigned long)start);
      break;
    }
    s_fw.erased = start + size;
    xSemaphoreGive(s_fw.erased_sem);
  }
  s_fw.eraser = NULL;
  vTaskDelete(NULL);
}

static void stop_upload(avb_state_s *state, const char *why) {
  s_fw.phase = fw_idle;
  for (int i = 0; s_fw.eraser && i < 300; i++) /* it stops after a block */
    vTaskDelay(pdMS_TO_TICKS(10));
  set_efu_mode(state, false);
  avbwarn("Firmware: upload ended: %s", why);
}

aecp_status_t avb_fw_start_upload(avb_state_s *state, uint64_t length,
                                  uint16_t *op_id) {
  if (s_fw.phase == fw_storing || s_fw.phase == fw_rebooting)
    return aecp_status_in_progress;
  if (length < sizeof(esp_image_header_t) || length > s_fw.slot->size)
    return aecp_status_bad_arguments;
#if !CONFIG_ESP_AVB_FW_UPLOAD_WHILE_STREAMING
  if (!s_fw.paused && streams_running(state)) {
    if (!state->locked && !state->acquired)
      return aecp_status_stream_is_running;
    pause_streams(state);
  }
#endif
  if (s_fw.phase != fw_idle)
    stop_upload(state, "replaced by a new upload");
  s_fw.image_length = (uint32_t)length;
  s_fw.written = 0;
  s_fw.last_offset = s_fw.last_length = 0;
  s_fw.erased = 0;
  xSemaphoreTake(s_fw.erased_sem, 0);
  s_fw.phase = fw_uploading;
  s_fw.upload_op = new_op_id();
  *op_id = s_fw.upload_op;
  if (xTaskCreatePinnedToCore(eraser_task, "avb_fw_erase", 3072, NULL, 2,
                              &s_fw.eraser, 0) != pdPASS) {
    s_fw.phase = fw_idle;
    return aecp_status_no_resources;
  }
  set_efu_mode(state, true);
  avbwarn("Firmware: upload of %lu bytes to %s started (op %u)",
          (unsigned long)length, s_fw.slot->label, s_fw.upload_op);
  return aecp_status_success;
}

int avb_fw_write(uint64_t offset, const uint8_t *data, uint32_t length) {
  if (s_fw.phase != fw_uploading)
    return AVB_FW_WRITE_INVALID;
  if (offset == s_fw.last_offset && length == s_fw.last_length &&
      s_fw.written == offset + length && length > 0)
    return AVB_FW_WRITE_OK; /* the controller's retry of the last chunk */
  if (offset != s_fw.written)
    return AVB_FW_WRITE_INVALID;
  if (offset + length > s_fw.image_length)
    return AVB_FW_WRITE_TOO_HIGH;
  int64_t give_up = esp_timer_get_time() + FW_WRITE_WAIT_MS * 1000LL;
  while (s_fw.erased < offset + length) {
    if (esp_timer_get_time() > give_up)
      return AVB_FW_WRITE_INVALID;
    xSemaphoreTake(s_fw.erased_sem, pdMS_TO_TICKS(50));
  }
  if (esp_partition_write(s_fw.slot, (size_t)offset, data, length) != ESP_OK)
    return AVB_FW_WRITE_INVALID;
  s_fw.last_offset = (uint32_t)offset;
  s_fw.last_length = length;
  s_fw.written = (uint32_t)offset + length;
  return AVB_FW_WRITE_OK;
}

/* The image must pass its hash (and, in signed builds, signature) check,
 * and as its header describes it (with its signature sector, in signed
 * builds) lie within the bytes uploaded: nothing an earlier image left in
 * the slot may count. NULL when it does, else what is wrong. */
static const char *image_problem(void) {
  esp_image_metadata_t meta;
  const esp_partition_pos_t pos = {.offset = s_fw.slot->address,
                                   .size = s_fw.slot->size};
  if (esp_image_verify(ESP_IMAGE_VERIFY_SILENT, &pos, &meta) != ESP_OK)
    return "it fails its hash" FW_SIGNATURE_NOTE " check";
  /* A verified signed image's length takes in its signature sector. */
  if (meta.image_len > s_fw.image_length)
    return "it runs past the bytes uploaded";
  return NULL;
}

/* The checks and the boot switch, off the AVB task: esp_image_verify reads
 * and hashes the whole image. */
static void store_task(void *arg) {
  int result = 0;
  const char *problem;
  esp_image_header_t header;
  esp_app_desc_t desc;
  const esp_app_desc_t *own = esp_app_get_description();
  if (esp_partition_read(s_fw.slot, 0, &header, sizeof(header)) != ESP_OK ||
      header.magic != ESP_IMAGE_HEADER_MAGIC) {
    avberr("Firmware: not an ESP image");
  } else if (header.chip_id != CONFIG_IDF_FIRMWARE_CHIP_ID) {
    avberr("Firmware: image is for chip %u, this is %u", header.chip_id,
           CONFIG_IDF_FIRMWARE_CHIP_ID);
  } else if (esp_ota_get_partition_description(s_fw.slot, &desc) != ESP_OK) {
    avberr("Firmware: no app description in the image");
  } else if (strncmp(desc.project_name, own->project_name,
                     sizeof(desc.project_name)) != 0) {
    avberr("Firmware: image is project '%.32s', this runs '%.32s'",
           desc.project_name, own->project_name);
  } else if ((problem = image_problem()) != NULL) {
    avberr("Firmware: image refused: %s", problem);
  } else if (esp_ota_set_boot_partition(s_fw.slot) != ESP_OK) {
    avberr("Firmware: image check failed (hash or signature)");
  } else {
    avbwarn("Firmware: '%.32s' %.32s stored in %s, boots next", desc.project_name,
            desc.version, s_fw.slot->label);
    result = 1;
  }
  s_fw.store_result = result;
  vTaskDelete(NULL);
}

aecp_status_t avb_fw_start_store(avb_state_s *state, bool reboot,
                                 uint16_t *op_id) {
  if (s_fw.phase != fw_uploading || s_fw.written != s_fw.image_length)
    return aecp_status_bad_arguments;
  for (int i = 0; s_fw.eraser && i < 300; i++)
    vTaskDelay(pdMS_TO_TICKS(10));
  s_fw.phase = fw_storing;
  s_fw.reboot_after_store = reboot;
  s_fw.store_op = new_op_id();
  *op_id = s_fw.store_op;
  s_fw.store_result = -1;
  if (xTaskCreatePinnedToCore(store_task, "avb_fw_store", 4096, NULL, 2, NULL,
                              0) != pdPASS) {
    s_fw.phase = fw_uploading;
    return aecp_status_no_resources;
  }
  return aecp_status_success;
}

aecp_status_t avb_fw_abort(avb_state_s *state, uint16_t op_id) {
  if (s_fw.phase != fw_uploading || op_id != s_fw.upload_op)
    return aecp_status_bad_arguments;
  stop_upload(state, "aborted");
  return aecp_status_success;
}

void avb_fw_tick(avb_state_s *state) {
  if (s_fw.slot == NULL)
    return;
  int64_t now = esp_timer_get_time();

  if (s_fw.pending_verify) {
    if (now > FW_VALID_MIN_US && state->ptp_status.clock_source_valid) {
      esp_ota_mark_app_valid_cancel_rollback();
      s_fw.pending_verify = false;
      avbwarn("Firmware: marked valid (gPTP valid %lld s after boot)",
              now / 1000000);
    } else if (now > FW_VALID_DEADLINE_US) {
      avberr("Firmware: not valid 60 s after boot, rolling back");
      esp_ota_mark_app_invalid_rollback_and_reboot();
    }
  }

  if (s_fw.phase == fw_storing && s_fw.store_result >= 0) {
    bool passed = s_fw.store_result == 1;
    uint16_t percent = passed ? 1000 : 0;
    avb_send_operation_status(state, aem_desc_type_memory_object,
                              avb_fw_object_index(), s_fw.store_op, percent);
    avb_send_operation_status(state, aem_desc_type_memory_object,
                              avb_fw_object_index(), s_fw.upload_op, percent);
    if (!passed) {
      s_fw.phase = fw_idle;
      set_efu_mode(state, false);
    } else if (s_fw.reboot_after_store) {
      s_fw.phase = fw_rebooting;
      s_fw.reboot_at_us = now + 300 * 1000;
      avb_send_adp_entity_departing(state);
    } else {
      s_fw.phase = fw_stored;
      set_efu_mode(state, false);
    }
  }

  if (s_fw.phase == fw_rebooting && now >= s_fw.reboot_at_us) {
    avbwarn("Firmware: restarting into the new image");
    esp_restart();
  }

  /* The lock went with no reboot pending: start the streams again. */
  if (s_fw.paused && !state->locked && !state->acquired &&
      s_fw.phase != fw_uploading && s_fw.phase != fw_storing &&
      s_fw.phase != fw_rebooting)
    resume_streams(state);
  if (s_fw.paused && !state->locked && !state->acquired &&
      s_fw.phase == fw_uploading) {
    stop_upload(state, "the controller's lock went");
    resume_streams(state);
  }
}

#endif /* CONFIG_ESP_AVB_FIRMWARE_OBJECT */
