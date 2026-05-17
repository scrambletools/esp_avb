/*
 * Copyright 2024 Scramble Tools
 * License: MIT
 *
 * ESP_AVB Component
 *
 * This component provides an implementation of an AVB talker and listener.
 *
 * This file provides the public API for the ESP_AVB component.
 */

#ifndef _ESP_AVB_H_
#define _ESP_AVB_H_

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <esp_eth.h>
#include <stdbool.h>
#include <time.h>

/****************************************************************************
 * Compiler Definitions
 ****************************************************************************/

/* Default Ethernet interface */
#define DEF_ETH_IF "ETH_0"

/****************************************************************************
 * Public Types
 ****************************************************************************/

/* High-level role of this AVB instance. Selected at compile time via
 * CONFIG_ESP_AVB_ROLE_*. The default is endpoint_wired, which preserves
 * historical behavior. Other roles gate codec / talker-listener init
 * and (for bridge) enable forwarding + FQTSS. */
typedef enum {
  avb_role_endpoint_wired = 0,
  avb_role_endpoint_wireless = 1,
  avb_role_bridge = 2,
} avb_role_e;

/* Per-port medium + timing-mechanism. Mirrors ptp_port_medium_e in
 * esp_ptp:
 *   eth_hwts — Ethernet medium with IEEE 1588 HW timestamping and
 *              on-wire 802.1AS Pdelay.
 *   wifi_ftm — Wi-Fi medium with IEEE 802.11mc FTM peer-delay and
 *              Sync transported in the SoftAP's 802.11 Beacon Vendor
 *              IE. */
typedef enum {
  avb_port_medium_eth_hwts = 0,
  avb_port_medium_wifi_ftm = 1,
} avb_port_medium_e;

/* Port-host-interface. How the port attaches to the host SoC.
 * Determines timestamp accuracy, latency floor, admission budget.
 * Mirrors ptp_port_host_if_e in esp_ptp. */
typedef enum {
  avb_port_host_if_emac  = 0,   /* on-chip MAC, RMII/RGMII; hardware TS */
  avb_port_host_if_ahb   = 1,   /* on-chip peripheral (native Wi-Fi) */
  avb_port_host_if_sdio  = 2,   /* external coprocessor over SDIO */
  avb_port_host_if_spi   = 3,   /* external Eth/Wi-Fi over SPI */
  avb_port_host_if_usb   = 4,   /* USB-attached */
  avb_port_host_if_other = 5,
} avb_port_host_if_e;

/* Port type — role in the AVB topology. Any port with type=bridged
 * implies the entity is in bridge mode (mirror of ESP_AVB_ROLE_BRIDGE). */
typedef enum {
  avb_port_type_primary  = 0,
  avb_port_type_failover = 1,
  avb_port_type_bridged  = 2,
} avb_port_type_e;

/* Wi-Fi mode for medium=wifi ports. Numeric values match IDF's
 * WIFI_IF_STA=0 / WIFI_IF_AP=1 so the field doubles as the wifi_if
 * argument to esp_wifi_internal_tx. NONE is the sentinel for non-wifi. */
typedef enum {
  avb_port_wifi_mode_sta  = 0,
  avb_port_wifi_mode_ap   = 1,
  avb_port_wifi_mode_none = 0xFF,
} avb_port_wifi_mode_e;

/* How the port's slave-side time is synchronized. GPTP_WIRED is the
 * historical 802.1AS path (Sync/Follow_Up + Pdelay on-wire). BEACON_IE_WIFI
 * is the wireless variant where Sync rides the AP's 802.11 beacon Vendor IE
 * and peer-delay rides FTM. FTM_ONLY is a peer-delay-only port (Sync
 * transport handled elsewhere). */
typedef enum {
  avb_port_time_source_gptp_wired = 0,
  avb_port_time_source_beacon_ie_wifi = 1,
  avb_port_time_source_ftm_only = 2,
} avb_port_time_source_e;

/* Codec types */
typedef enum {
  avb_codec_type_es8311, // Everest Semiconductor ES8311
  avb_codec_type_es8388  // Everest Semiconductor ES8388
} avb_codec_type_t;

/* Codec control ranges (in tenths of dB for AECP control values) */
typedef struct {
  int16_t vol_min_tenth_db;      // speaker volume minimum
  int16_t vol_max_tenth_db;      // speaker volume maximum
  int16_t vol_step_tenth_db;     // speaker volume step
  int16_t vol_default_tenth_db;  // speaker volume default
  int16_t gain_min_tenth_db;     // mic gain minimum
  int16_t gain_max_tenth_db;     // mic gain maximum
  int16_t gain_step_tenth_db;    // mic gain step
  int16_t gain_default_tenth_db; // mic gain default
} codec_control_range_s;

/* Sample rates struct */
typedef struct {
  uint32_t sample_rates[8];
  uint8_t num_rates;
} avb_sample_rates_s;

/* Bit rates struct */
typedef struct {
  uint8_t bit_rates[8]; // bits per sample
  uint8_t num_rates;
} avb_bit_rates_s;

/* Pin assignments for the codec audio buses */
typedef struct {
  uint8_t mclk;       // I2S MCLK
  uint8_t bclk;       // I2S BCLK
  uint8_t ws;         // I2S word select / LRCK
  uint8_t dout;       // I2S data out (host -> codec DAC)
  uint8_t din;        // I2S data in (codec ADC -> host)
  uint8_t i2c_scl;    // I2C SCL
  uint8_t i2c_sda;    // I2C SDA
  int16_t pa;         // PA enable (output amplifier), -1 if unused
  bool pa_reverted;   // true if PA enable pin is active-low (inverter on board)
} avb_codec_pins_s;

/* AVB configuration structure
 * Currently sample rate and bit rate must be same for
 * for all inputs and outputs.
 */
typedef struct {
  bool talker;                               // enable talker
  bool listener;                             // enable listener
  bool codec_disabled;                       // skip I2S+codec init (no audio hardware, e.g. c6 Wi-Fi endpoint)
  bool atdecc_control;                       // allow remote control via ATDECC
  bool milan_compliant;                      // enable Milan-specific behavior
  bool avb_lite_compliant;                  // allow AVB Lite mode with standard PTP
  uint64_t association_id;                   // AVDECC association ID
  uint64_t model_id;                         // AVDECC entity model ID
  uint16_t port_id;                          // AVB interface port number
  const char *entity_name;                   // entity name
  const char *vendor_name;                   // vendor name
  const char *model_name;                    // model name
  const char *group_name;                    // group name
  const char *firmware_version;              // firmware version
  const char *serial_number;                 // serial number
  char *eth_interface;                       // ethernet interface name
  char *wifi_interface;                      // wifi netif if_key (bridge port[1]); NULL for endpoints
  uint8_t i2s_port;                          // i2s port number
  avb_codec_pins_s codec_pins;               // i2s/i2c/PA pin assignments
  esp_eth_handle_t *eth_handle;              // ethernet handle
  const avb_codec_type_t codec_type;         // codec type
  uint32_t default_sample_rate;              // default sample rate
  uint32_t default_presentation_time_offset_ns; // default Stream Output presentation offset
  uint8_t default_bits_per_sample;           // default bits per sample
  uint8_t input_channels_usable;             // usable input channels
  uint8_t output_channels_usable;            // usable output channels
  uint8_t channels_per_stream;               // advertised AVTP audio channels per stream
  uint8_t num_allowed_sample_rates;          // number of allowed sample rates
  uint32_t allowed_sample_rates[8];          // app policy filter for sample rates
  uint8_t num_allowed_bits_per_sample;       // number of allowed bits/sample
  uint8_t allowed_bits_per_sample[8];        // app policy filter for bits/sample
  int16_t default_mic_gain_tenth_db;         // default mic gain in tenths of dB
  int16_t default_speaker_vol_tenth_db;      // default speaker volume in tenths of dB
  uint16_t class_a_vlan_id;                  // SR Class A VID; 0 = use CONFIG_ESP_AVB_STREAM_VLAN_ID_CLASS_A
  uint16_t class_b_vlan_id;                  // SR Class B VID; 0 = use CONFIG_ESP_AVB_STREAM_VLAN_ID_CLASS_B
  bool allow_class_a_over_wifi;              // bridge MAP: allow Class A egress on Wi-Fi (off by default; the 125 us Milan §5.6 budget can't be met on 802.11, so this is opt-in for bench experiments)
} avb_config_s;

/* AVB status information structure */
typedef struct {
  bool clock_source_valid; // clock source valid
  bool avb_lite;           // operating in AVB Lite mode (standard PTP)
  bool streaming_in;       // one or more input streams are active
  bool streaming_out;      // one or more output streams are active
  struct {
    uint8_t id[8]; // Entity ID
  } entity;
} avb_status_s;

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

#ifdef __cplusplus
#define EXTERN extern "C"
extern "C" {
#else
#define EXTERN extern
#endif

/* @brief Start the AVB task and bind it to a specified interface
 *
 * @param config Configuration structure
 *
 * @return OK on success, ERROR on failure
 *
 * @note Make sure i2c is initialized before starting AVB task.
 *       AVB task will handle i2s and codec initialization.
 *       Only one instance of AVB task can run at a time.
 *       Attempting to start multiple instances will fail with an error.
 */
int avb_start(avb_config_s *config);

/* @brief Query status from a running AVB task
 *
 * @param status Pointer to status storage structure
 *
 * @return OK on success, negative errno on failure
 *
 * @note Multiple threads with priority less than CONFIG_ESP_AVB_TASKPRIO
 *       can request status simultaneously. If higher priority threads
 *       request status simultaneously, some of the requests may timeout.
 */
int avb_status(avb_status_s *status);

/* @brief Stop the AVB task
 *
 * @return OK on success, negative errno on failure
 **/
int avb_stop();

#undef EXTERN
#ifdef __cplusplus
}
#endif

#include "avbconfig.h"

#endif /* _ESP_AVB_H_ */
