/*
 * Copyright 2024-2026 Scramble Tools
 * License: MIT
 *
 * ESP_AVB Component — the last crash's core dump as an ATDECC CRASH_DUMP
 * memory object (IEEE 1722.1-2021 Table 7-19).
 *
 * With ESP-IDF core dumps going to flash, a panic stores an ELF core dump
 * in the coredump partition. It is served from AVB_CRASH_OBJECT_BASE as
 * the raw image ESP-IDF stores (decode it with `esp-coredump info_corefile
 * --core-format raw`). Its length is the stored image's size, 0 when there
 * is none: checked once at boot (a crash always reboots) and set to 0 by an
 * erase. Reading never clears it. START_OPERATION ERASE does, by erasing
 * the partition's first sector, which invalidates the image; the next panic
 * erases what it needs.
 */
#include "sdkconfig.h"

#include "avb.h"

#if CONFIG_ESP_AVB_CRASH_OBJECT

#include "esp_core_dump.h"
#include "esp_partition.h"
#include "esp_system.h"

static const esp_partition_t *s_part;
static uint32_t s_offset; /* the image's start within the partition */
static uint32_t s_length; /* 0: no dump */

void avb_crash_object_init(void) {
  s_part = esp_partition_find_first(ESP_PARTITION_TYPE_DATA,
                                    ESP_PARTITION_SUBTYPE_DATA_COREDUMP, NULL);
  if (s_part == NULL) {
    avbwarn("No coredump partition: no crash dump for controllers");
    return;
  }
  size_t address = 0, size = 0;
  if (esp_core_dump_image_check() == ESP_OK &&
      esp_core_dump_image_get(&address, &size) == ESP_OK &&
      address >= s_part->address &&
      address + size <= s_part->address + s_part->size) {
    s_offset = address - s_part->address;
    s_length = size;
    avbwarn("A crash dump is stored (%lu bytes), for controllers as "
            "memory object %d",
            (unsigned long)size, avb_crash_object_index());
  }
}

bool avb_crash_object_enabled(void) { return s_part != NULL; }

uint64_t avb_crash_object_length(void) { return s_length; }

uint64_t avb_crash_object_max_length(void) {
  return s_part ? s_part->size : 0;
}

int avb_crash_object_read(uint64_t offset, uint32_t length, uint8_t *data) {
  if (offset > s_length || length > s_length - offset)
    return AVB_LOG_READ_TOO_HIGH;
  if (esp_partition_read(s_part, s_offset + offset, data, length) != ESP_OK)
    return AVB_LOG_READ_TOO_HIGH;
  return AVB_LOG_READ_OK;
}

esp_err_t avb_crash_object_erase(void) {
  if (s_part == NULL)
    return ESP_ERR_NOT_FOUND;
  esp_err_t err = esp_partition_erase_range(s_part, 0, s_part->erase_size);
  if (err == ESP_OK) {
    s_length = 0;
    avbinfo("Crash dump erased");
  }
  return err;
}

#if CONFIG_ESP_AVB_CRASH_TEST
void avb_crash_object_test_crash(void) {
  avbwarn("Crash test asked for by a controller: aborting now");
  esp_system_abort("ATDECC crash test");
}
#endif

#endif /* CONFIG_ESP_AVB_CRASH_OBJECT */
