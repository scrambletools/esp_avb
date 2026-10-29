/*
 * Device addresses from the Scramble Tools MA-S 8C:1F:64:36:C, as the
 * scrambletools/registry (eui.toml) assigns them: a run of 16 per
 * device from 8C:1F:64:36:C1:10, keyed here by the factory MAC in
 * eFuse. Keep the two in step. A board missing here keeps its factory
 * MAC.
 */

#include <stdbool.h>
#include <stdint.h>
#include <string.h>

#include "esp_log.h"
#include "esp_mac.h"

#include "avb.h"

static const char *TAG = "avb_registry";

typedef struct {
  uint8_t factory[6]; /* eFuse base MAC */
  uint8_t base[6];    /* first address of the device's run */
} avb_registry_device_s;

/* The base is the Ethernet MAC on a P4 (one universal address) and the
 * station MAC on a C6. The bridge's coprocessor takes base + 1 for its
 * AP, the soft-AP offset IDF gives a C6 (registry entry per device). */
static const avb_registry_device_s s_devices[] = {
    /* Synthia, P4 endpoint */
    {{0xe8, 0xf6, 0x0a, 0xe0, 0x92, 0x20}, {0x8c, 0x1f, 0x64, 0x36, 0xc1, 0x10}},
    /* Condenser mic, P4 endpoint */
    {{0xe8, 0xf6, 0x0a, 0xe8, 0x16, 0xf5}, {0x8c, 0x1f, 0x64, 0x36, 0xc1, 0x20}},
    /* Power amp, P4 endpoint */
    {{0xe8, 0xf6, 0x0a, 0xe0, 0x97, 0x21}, {0x8c, 0x1f, 0x64, 0x36, 0xc1, 0x30}},
    /* Guitar, P4 endpoint */
    {{0xe8, 0xf6, 0x0a, 0xe0, 0x97, 0x20}, {0x8c, 0x1f, 0x64, 0x36, 0xc1, 0x40}},
    /* Wireless ESP3, C6 station endpoint */
    {{0xfc, 0x01, 0x2c, 0xfd, 0xfe, 0x80}, {0x8c, 0x1f, 0x64, 0x36, 0xc1, 0x50}},
    /* ESP0 bridge, P4 (its C6 coprocessor's AP is base + 1) */
    {{0x80, 0xf1, 0xb2, 0xd2, 0xca, 0xa9}, {0x8c, 0x1f, 0x64, 0x36, 0xc1, 0x60}},
};

/* The 6-byte eFuse MAC: esp_efuse_mac_get_default writes an 8-byte
 * EUI-64 on a chip with 802.15.4 (a C6). */
static esp_err_t avb_registry_factory_mac(uint8_t factory[6]) {
  return esp_read_mac(factory, ESP_MAC_EFUSE_FACTORY);
}

bool avb_registry_base_mac(uint8_t base[6]) {
  uint8_t factory[6];
  if (avb_registry_factory_mac(factory) != ESP_OK)
    return false;
  for (size_t i = 0; i < sizeof(s_devices) / sizeof(s_devices[0]); i++) {
    if (memcmp(s_devices[i].factory, factory, sizeof(factory)) == 0) {
      memcpy(base, s_devices[i].base, sizeof(s_devices[i].base));
      return true;
    }
  }
  return false;
}

bool avb_registry_apply_base_mac(void) {
  uint8_t base[6];
  if (!avb_registry_base_mac(base)) {
    uint8_t factory[6] = {0};
    esp_err_t err = avb_registry_factory_mac(factory);
    ESP_LOGW(TAG,
             "factory MAC %02x:%02x:%02x:%02x:%02x:%02x (%s) not in the "
             "registry, kept",
             factory[0], factory[1], factory[2], factory[3], factory[4],
             factory[5], esp_err_to_name(err));
    return false;
  }
  esp_err_t err = esp_base_mac_addr_set(base);
  if (err != ESP_OK) {
    ESP_LOGE(TAG, "esp_base_mac_addr_set failed (%s), factory MAC kept",
             esp_err_to_name(err));
    return false;
  }
  ESP_LOGI(TAG, "base MAC %02x:%02x:%02x:%02x:%02x:%02x from the registry",
           base[0], base[1], base[2], base[3], base[4], base[5]);
  return true;
}

/* An entity ID made from a registered device's factory MAC (MAC plus
 * 00:00, as these boards make it) becomes the one from its run, so a
 * binding saved before the move still finds its talker. */
bool avb_registry_translate_entity_id(uint8_t entity_id[8]) {
  if (entity_id[6] != 0 || entity_id[7] != 0)
    return false;
  for (size_t i = 0; i < sizeof(s_devices) / sizeof(s_devices[0]); i++) {
    if (memcmp(s_devices[i].factory, entity_id, 6) == 0) {
      memcpy(entity_id, s_devices[i].base, 6);
      return true;
    }
  }
  return false;
}
