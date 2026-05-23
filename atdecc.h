/*
 * Copyright 2024-2026 Scramble Tools
 * License: MIT
 *
 * ATDECC (IEEE 1722.1-2021) wire types and protocol message declarations.
 *
 * Covers: ADP, AECP, ACMP, AEM descriptors, AECP commands/responses,
 * CVU (AVB Lite SRP-over-AECP), MVU (Milan vendor unique).
 *
 * Wire-format structs follow IEEE 1722.1-2021. Multi-byte fields use
 * uint8_t[N] byte arrays (always big-endian on wire); bit-fields stay
 * within single octets for compiler-portable layout. The struct IS the
 * wire format — RX/TX use direct memcpy, no separate encode/decode
 * pass.
 *
 * ATDECC rides inside AVTP (subtypes 0xfa/0xfb/0xfc for ADP/AECP/ACMP),
 * so this header #includes avtp.h. The avtp_msgbuf_u envelope union
 * spanning all AVTP subtypes lives here because it embeds ATDECC
 * payload types and so must see them defined first.
 *
 * CVU wrappers (aecp_cvu_srp_talker_s, aecp_cvu_srp_listener_s) embed
 * native MSRP wire types from mrp.h — the MSRP state machine in mrp.c
 * drives both native MSRP and CVU transports.
 *
 * This header is included from avb.h after the prerequisite types
 * (eth_addr_t, unique_id_t, AEM_MAX_DESC_LEN, AEM_MAX_NUM_*,
 * AVB_MAX_MSG_LEN) are defined. avb_state_s is forward-declared
 * below; its full definition follows in avb.h.
 */

#ifndef ESP_AVB_ATDECC_H_
#define ESP_AVB_ATDECC_H_

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "avtp.h"  /* AVTP types: subtype enum, stream formats, MAAP, ... */
#include "mrp.h"   /* MSRP types embedded in CVU wrappers */

/* Forward decl — full definition in avb.h. */
typedef struct avb_state_s avb_state_s;

/* ===== ATDECC enums ===== */

/* ADP message types in enumerated order (1722.1 Clause 6.2) */
typedef enum {
  adp_msg_type_entity_available,
  adp_msg_type_entity_departing,
  adp_msg_type_entity_discover
} adp_msg_type_t;

/* AECP message types (subset) and their values (1722.1 Clause 9) */
typedef enum {
  aecp_msg_type_aem_command = 0,
  aecp_msg_type_aem_response = 1,
  aecp_msg_type_addr_access_command = 2,
  aecp_msg_type_addr_access_response = 3,
  aecp_msg_type_vendor_unique_command = 6,
  aecp_msg_type_vendor_unique_response = 7
} aecp_msg_type_t;

/* AECP command codes (subset) and their values (1722.1 Clause 7.4)
 * must change to big-endian before writing to message body
 */
typedef enum {
  aecp_cmd_code_acquire_entity = 0x0000,
  aecp_cmd_code_lock_entity = 0x0001,
  aecp_cmd_code_entity_available = 0x0002,
  aecp_cmd_code_controller_available = 0x0003, // unsupported
  aecp_cmd_code_read_descriptor = 0x0004,
  aecp_cmd_code_get_configuration = 0x0007,
  aecp_cmd_code_set_stream_format = 0x0008,
  aecp_cmd_code_get_stream_format = 0x0009,
  aecp_cmd_code_get_stream_info = 0x000f,
  aecp_cmd_code_set_name = 0x0010,
  aecp_cmd_code_get_name = 0x0011,
  aecp_cmd_code_set_clock_source = 0x0016,
  aecp_cmd_code_get_clock_source = 0x0017,
  aecp_cmd_code_set_control = 0x0018,
  aecp_cmd_code_get_control = 0x0019,
  aecp_cmd_code_start_streaming = 0x0022, // unsupported
  aecp_cmd_code_stop_streaming = 0x0023,  // unsupported
  aecp_cmd_code_register_unsol_notif = 0x0024,
  aecp_cmd_code_deregister_unsol_notif = 0x0025,
  aecp_cmd_code_get_avb_info = 0x0027,
  aecp_cmd_code_get_as_path = 0x0028, // unsupported
  aecp_cmd_code_get_counters = 0x0029,
  aecp_cmd_code_set_max_transit_time = 0x004c,
  aecp_cmd_code_get_max_transit_time = 0x004d,
  aecp_cmd_code_expansion = 0xffff // for milan
} aecp_cmd_code_t;

/* AECP statuses in enumerated order */
typedef enum {
  aecp_status_success, // The ATDECC Entity successfully performed the command
                       // and has valid results.
  aecp_status_not_implemented, // The ATDECC Entity does not support the command
                               // type.
  aecp_status_no_such_descriptor, // A descriptor with the descriptor_type and
                                  // descriptor_index specified does not exist.
  aecp_status_entity_locked,   // The ATDECC Entity has been locked by another
                               // ATDECC Controller.
  aecp_status_entity_acquired, // The ATDECC Entity has been acquired by another
                               // ATDECC Controller.
  aecp_status_not_authenticated, // The ATDECC Controller is not authenticated
                                 // with the ATDECC Entity.
  aecp_status_authentication_disabled, // The ATDECC Controller is trying to use
                                       // an authentication command when
                                       // authentication is not enabled on the
                                       // ATDECC Entity.
  aecp_status_bad_arguments, // One or more of the values in the fields of the
                             // frame were deemed to be bad by the ATDECC Entity
                             // (unsupported, incorrect combination, etc.).
  aecp_status_no_resources,  // The ATDECC Entity cannot complete the command
                            // because it does not have the resources to support
                            // it.
  aecp_status_in_progress, // The ATDECC Entity is processing the command and
                           // will send a second response at a later time with
                           // the result of the command.
  aecp_status_entity_misbehaving, // The ATDECC Entity generated an internal
                                  // error while trying to process the command.
  aecp_status_not_supported, // The command is implemented but the target of the
                             // command is not supported. For example, trying to
                             // set the value of a read-only control.
  aecp_status_stream_is_running // The stream is currently streaming and the
                                // command is one which cannot be executed on a
                                // streaming stream.
} aecp_status_t;

/* AEM descriptor types and their values
 * must change to big-endian before writing to message body
 */
typedef enum {
  aem_desc_type_entity = 0x0000,        // The ATDECC Entity
  aem_desc_type_configuration = 0x0001, // a configuration of the ATDECC Entity
  aem_desc_type_audio_unit = 0x0002,    // an Audio Unit
  aem_desc_type_video_unit = 0x0003,    // a Video Unit
  aem_desc_type_sensor_unit = 0x0004,  // a Sensor Unit with one or more sensors
                                       // sampled with the same clock
  aem_desc_type_stream_input = 0x0005, // an input stream to the ATDECC Entity
  aem_desc_type_stream_output =
      0x0006, // an output stream from the ATDECC Entity
  aem_desc_type_avb_interface = 0x0009, // an AVB interface
  aem_desc_type_clock_source = 0x000a,  // an clock source
  aem_desc_type_memory_object =
      0x000b,                     // an memory object for files or firmware
  aem_desc_type_locale = 0x000c,  // a locale
  aem_desc_type_strings = 0x000d, // a localized strings
  aem_desc_type_stream_port_input = 0x000e,  // an input stream port on a Unit
  aem_desc_type_stream_port_output = 0x000f, // an output stream port on a Unit
  aem_desc_type_audio_cluster =
      0x0014, // a cluster of channels within an audio Stream
  aem_desc_type_video_cluster = 0x0015, // an element of the video Stream
  aem_desc_type_sensor_cluster =
      0x0016,                       // the sensor elements of a sensor stream
  aem_desc_type_audio_map = 0x0017, // mapping between the channels of an audio
                                    // Stream and the channels of the audio Port
  aem_desc_type_video_map =
      0x0018, // mapping between the components of a video Stream and the video
              // clusters of the video Port
  aem_desc_type_sensor_map =
      0x0019, // mapping between a Sensor signal and the Sensor stream
  aem_desc_type_control = 0x001a,         // a generic Control
  aem_desc_type_signal_selector = 0x001b, // a Signal Selector Control
  aem_desc_type_clock_domain = 0x0024,    // a Clock Domain
  aem_desc_type_invalid = 0xffff          // there is no valid descriptor
} aem_desc_type_t;

/* AEM Clock Source Types and their values */
typedef enum {
  aem_clock_source_type_internal = 0x0000,
  aem_clock_source_type_external = 0x0001,
  aem_clock_source_type_input_stream = 0x0002,
  aem_clock_source_type_expansion = 0xffff
} aem_clock_source_type_t;

/* AEM Memory Object Types and their values */
typedef enum {
  aem_memory_obj_type_firmware_image = 0x0000,
  aem_memory_obj_type_vendor_specific = 0x0001,
  aem_memory_obj_type_crash_dump = 0x0002,
  aem_memory_obj_type_log_object = 0x0003,
  aem_memory_obj_type_autostart_settings = 0x0004,
  aem_memory_obj_type_snapshot_settings = 0x0005,
  aem_memory_obj_type_svg_manufacturer = 0x0006,
  aem_memory_obj_type_svg_entity = 0x0007,
  aem_memory_obj_type_svg_generic = 0x0008,
  aem_memory_obj_type_png_manufacturer = 0x0009,
  aem_memory_obj_type_png_entity = 0x000a,
  aem_memory_obj_type_png_generic = 0x000b,
  aem_memory_obj_type_dae_manufacturer = 0x000c,
  aem_memory_obj_type_dae_entity = 0x000d,
  aem_memory_obj_type_dae_generic = 0x000e
} aem_memory_obj_type_t;

/* AEM Memory Object Operation Types and their values */
typedef enum {
  aem_memory_obj_op_type_store = 0x0000,
  aem_memory_obj_op_type_store_and_reboot = 0x0001,
  aem_memory_obj_op_type_read = 0x0002,
  aem_memory_obj_op_type_erase = 0x0003,
  aem_memory_obj_op_type_upload = 0x0004
} aem_memory_obj_op_type_t;

/* AEM Audio Cluster formats and their values */
typedef enum {
  aem_audio_cluster_format_iec_60958 = 0x00,
  aem_audio_cluster_format_mbla = 0x40,
  aem_audio_cluster_format_midi = 0x80,
  aem_audio_cluster_format_smpte = 0x88
} aem_audio_cluster_format_t;

/* ACMP message types (subset) in enumerated order (1722.1 Clause 8.2) */
typedef enum {
  acmp_msg_type_connect_tx_command,     // Connect Talker source stream command
  acmp_msg_type_connect_tx_response,    // Connect Talker source stream response
  acmp_msg_type_disconnect_tx_command,  // Disconnect Talker source stream
                                        // command
  acmp_msg_type_disconnect_tx_response, // Disconnect Talker source stream
                                        // response
  acmp_msg_type_get_tx_state_command,   // Get Talker source stream connection
                                        // state command
  acmp_msg_type_get_tx_state_response,  // Get Talker source stream connection
                                        // state response
  acmp_msg_type_connect_rx_command,     // Connect Listener sink stream command
  acmp_msg_type_connect_rx_response,    // Connect Listener sink stream response
  acmp_msg_type_disconnect_rx_command,  // Disconnect Listener sink stream
                                        // command
  acmp_msg_type_disconnect_rx_response, // Disconnect Listener sink stream
                                        // response
  acmp_msg_type_get_rx_state_command,   // Get Listener sink stream connection
                                        // state command
  acmp_msg_type_get_rx_state_response,  // Get Listener sink stream connection
                                        // state response
  acmp_msg_type_get_tx_connection_command,  // Get a specific Talker connection
                                            // info command
  acmp_msg_type_get_tx_connection_response, // Get a specific Talker connection
                                            // info response
  acmp_msg_type_count
} acmp_msg_type_t;

/* ACMP statuses in enumerated order */
typedef enum {
  acmp_status_success,              // Command executed successfully
  acmp_status_listener_unknown_id,  // Listener does not have the specified
                                    // unique identifier
  acmp_status_talker_unknown_id,    // Talker does not have the specified unique
                                    // identifier
  acmp_status_talker_dest_mac_fail, // Talker could not allocate a destination
                                    // MAC for the stream
  acmp_status_talker_no_stream_index, // Talker does not have an available
                                      // stream index for the stream
  acmp_status_talker_no_bandwidth,    // Talker could not allocate bandwidth for
                                      // the stream
  acmp_status_talker_exclusive, // Talker already has an established stream and
                                // only supports one Listener
  acmp_status_listener_talker_timeout, // Listener had timeout for all retries
                                       // when trying to send command to Talker
  acmp_status_listener_exclusive,      // The ATDECC Listener already has an
                                       // established connection to stream
  acmp_status_state_unavailable, // Could not get the state from the ATDECC
                                 // Entity
  acmp_status_not_connected, // Trying to disconnect when not connected or not
                             // connected to the ATDECC Talker specified
  acmp_status_no_such_connection, // Trying to obtain connection information for
                                  // an ATDECC Talker connection which does not
                                  // exist
  acmp_status_could_not_send_message, // The ATDECC Listener failed to send the
                                      // message to the ATDECC Talker
  acmp_status_talker_misbehaving,   // Talker was unable to complete the command
                                    // because an internal error occurred
  acmp_status_listener_misbehaving, // Listener was unable to complete the
                                    // command because an internal error
                                    // occurred
  acmp_status_reserved,             // Reserved for future use
  acmp_status_controller_not_authorized, // The ATDECC Controller with the
                                         // specified Entity ID is not
                                         // authorized to change stream
                                         // connections
  acmp_status_incompatable_request, // The ATDECC Listener is trying to connect
                                    // to an ATDECC Talker that is already
                                    // streaming with a different traffic class,
                                    // etc. or does not support the requested
                                    // traffic class
  acmp_status_listener_invalid_connection, // ATDECC Listener is being asked to
                                           // connect to something that it
                                           // cannot listen to, e.g. it is being
                                           // asked to listen to its own ATDECC
                                           // Talker stream
  acmp_status_listener_can_only_listen_once, // The ATDECC Listener is being
                                             // asked to connect to a stream
                                             // that is already connected to
                                             // another one of its streams sinks
                                             // and it is only capable of
                                             // listening on one of them
  acmp_status_not_supported = 31             // The command is not supported
} acmp_status_t;

/* ACMP timeouts in milliseconds */
typedef enum {
  acmp_timeout_connect_tx = 3500,
  acmp_timeout_disconnect_tx = 200,
  acmp_timeout_get_tx_state = 200,
  acmp_timeout_connect_rx = 4500,
  acmp_timeout_disconnect_rx = 500,
  acmp_timeout_get_rx_state = 200,
  acmp_timeout_get_tx_connection = 200
} acmp_timeout_t;

/* ===== ATDECC capability and entity types ===== */

/* ATDECC types*/

/* The data structures in this section are defined in IEEE 1722.1-2021
 * All multi-byte fields are big-endian.
 */

/* AVB Entity Capabilities */
typedef struct {
  uint8_t multiple_ptp_instances : 1; // The Entity has multiple PTP Instances
                                      // using an interface.
  uint8_t
      aem_config_index_valid : 1; // The current_configuration_index field
                                  // contains a valid index of an AEM
                                  // CONFIGURATION descriptor for the current
                                  // Configuration. This flag shall only be set
                                  // if the AEM_SUPPORTED flag is set.
  uint8_t reserved : 6;           // Reserved for future use
  uint8_t general_controller_ignore : 1; // General purpose ATDECC Controllers
                                         // ignore the presence of this ATDECC
                                         // Entity when this flag is set.
  uint8_t
      entity_not_ready : 1; // The ATDECC Entity is not ready to be enumerated
                            // or connected by an ATDECC Controller.
  uint8_t acmp_acquire_with_aem : 1; // ACMP respects any acquisition made with
                                     // the ACQUIRE_ENTITY command.
  uint8_t acmp_auth_with_aem : 1;    // ACMP requires that the ATDECC Controller
                                     // authenticate using the AEM AUTHENTICATE
                                     // command.
  uint8_t supports_udpv4_atdecc : 1; // The Entity supports ATDECC via AVTP over
                                     // UDP using IPv4.
  uint8_t supports_udpv4_streaming : 1; // The Entity supports streaming via
                                        // AVTP over UDP using IPv4.
  uint8_t supports_udpv6_atdecc : 1; // The Entity supports ATDECC via AVTP over
                                     // UDP using IPv6.
  uint8_t supports_udpv6_streaming : 1; // The Entity supports streaming via
                                        // AVTP over UDP using IPv6.
  uint8_t class_a : 1; // Supports sending and/or receiving Class A streams.
  uint8_t class_b : 1; // Supports sending and/or receiving Class B streams.
  uint8_t
      gptp_supported : 1; // The ATDECC Entity implements IEEE Std 802.1AS-2020.
  uint8_t aem_auth_supported : 1; // Supports using AEM Authentication via the
                                  // AUTHENTICATE command as defined in 7.4.66.
                                  // This flag shall only be set if the
                                  // AEM_SUPPORTED flag is set.
  uint8_t aem_auth_required : 1;  // Requires the use of AEM Authentication via
                                  // the AUTHENTICATE command as defined
                                  // in 7.4.66. This flag shall only be set if
                                  // the AEM_SUPPORTED flag is set.
  uint8_t
      aem_persistent_acquire_supported : 1; // Supports the use of the
                                            // PERSISTENT flag in the ACQUIRE
                                            // command as defined in 7.4.1. This
                                            // flag shall only be set if the
                                            // AEM_SUPPORTED flag is set.
  uint8_t
      aem_identify_control_index_valid : 1; // The identify_control_index field
                                            // contains a valid index of an AEM
                                            // CONTROL descriptor for the
                                            // primary IDENTIFY control in the
                                            // current Configuration. This flag
                                            // shall only be set if the
                                            // AEM_SUPPORTED flag is set.
  uint8_t
      aem_interface_index_valid : 1; // The interface_index field contains a
                                     // valid index of an AEM AVB_INTERFACE
                                     // descriptor for interface in the current
                                     // Configuration which is transmitting the
                                     // ADPDU. This flag shall only be set if
                                     // the AEM_SUPPORTED flag is set.
  uint8_t
      efu_mode : 1; // Entity Firmware Upgrade mode is enabled on the ATDECC
                    // Entity. When this flag is set, the ATDECC Entity is in
                    // the mode to perform an ATDECC Entity firmware upgrade.
  uint8_t address_access_supported : 1; // Supports receiving the ADDRESS_ACCESS
                                        // commands as definedin 1722.1-2021
                                        // section 9.4.
  uint8_t
      gateway_entity : 1;    // ATDECC Entity serves as a gateway to a device on
                             // another typeof media (typically a IEEE Std 1394
                             // device) by proxying control services for it
  uint8_t aem_supported : 1; // Supports receiving the ATDECC Entity Model (AEM)
                             // AECP commands as defined in 9.3.
  uint8_t
      legacy_avc : 1; // Supports using IEEE Std 1394 AV/C protocol (For
                      // example, for a IEEE Std 1394 device through a gateway).
  uint8_t assoc_id_supported : 1; // The ATDECC Entity supports the use of the
                                  // association_id field for associating the
                                  // ATDECC Entity with other ATDECC entities.
  uint8_t assoc_id_valid : 1;     // The association_id field contains a valid
                                  // value. This bit shall only be set in
                                  // conjunction with ASSOCIATION_ID_SUPPORTED.
  uint8_t
      vendor_unique_supported : 1; // Supports receiving the AEM VENDOR_UNIQUE
                                   // commands as defined in 9.5.3.
} avb_entity_cap_s;                // 4 bytes

/* AVB Talker Capabilities */
typedef struct {
  uint8_t reserved1 : 1;
  uint8_t other_source : 1;
  uint8_t control_source : 1;
  uint8_t media_clock_source : 1;
  uint8_t smpte_source : 1;
  uint8_t midi_source : 1;
  uint8_t audio_source : 1;
  uint8_t video_source : 1;
  uint8_t implemented : 1;
  uint8_t reserved2 : 7;
} avb_talker_cap_s; // 2 bytes

/* AVB Listener Capabilities */
typedef struct {
  uint8_t reserved1 : 1;
  uint8_t other_sink : 1;
  uint8_t control_sink : 1;
  uint8_t media_clock_sink : 1;
  uint8_t smpte_sink : 1;
  uint8_t midi_sink : 1;
  uint8_t audio_sink : 1;
  uint8_t video_sink : 1;
  uint8_t implemented : 1;
  uint8_t reserved2 : 7;
} avb_listener_cap_s; // 2 bytes

/* AVB Controller Capabilities */
typedef struct {
  uint8_t reserved1[3];
  uint8_t implemented : 1;
  uint8_t layer3_proxy : 1;
  uint8_t reserved2 : 6;
} avb_controller_cap_s; // 4 bytes

/* AVB Entity Summary
 * used in entity available message and entity descriptor
 */
typedef struct {
  unique_id_t entity_id;
  unique_id_t model_id;
  avb_entity_cap_s entity_capabilities;
  uint8_t talker_stream_sources[2];
  avb_talker_cap_s talker_capabilities;
  uint8_t listener_stream_sinks[2];
  avb_listener_cap_s listener_capabilities;
  avb_controller_cap_s controller_capabilities;
  uint8_t available_index[4];
} avb_entity_summary_s; // 40 bytes

/* AVB Entity Detail
 * used in entity descriptor
 */
typedef struct {
  unique_id_t association_id;
  uint8_t entity_name[64];
  uint8_t vendor_name_ref[2];
  uint8_t model_name_ref[2];
  uint8_t firmware_version[64];
  uint8_t group_name[64];
  uint8_t serial_number[64];
  uint8_t configurations_count[2];
  uint8_t current_configuration[2];
} avb_entity_detail_s; // 272 bytes

/* ATDECC header */
typedef struct {
  uint8_t subtype;                // AVTP message subtype
  uint8_t msg_type : 4;           // ATDECC message type
  uint8_t version : 3;            // AVTP version
  uint8_t sv : 1;                 // always 0 for ADP messages
  uint8_t control_data_len_h : 3; // 3 high order bits of control data length
  uint8_t status_valtime : 5;     // status or valid time in 2sec increments
  uint8_t control_data_len;       // control data length (8 low order bits)
} atdecc_header_s;                // 4 bytes

/* ADP message */
typedef struct {
  atdecc_header_s header;
  avb_entity_summary_s entity;       // avb entity summary
  unique_id_t gptp_btc_id;            // gPTP BTC ID (1722.1 field)
  uint8_t gptp_domain_num;           // gptp domain number
  uint8_t reserved1;                 // reserved
  uint8_t current_config_index[2];   // current configuration index
  uint8_t identify_control_index[2]; // identify control index
  uint8_t interface_index[2];        // interface index
  unique_id_t association_id;        // association ID
  uint8_t reserved2[4];              // reserved
} adp_message_s;

/* AECP common data */
typedef struct {
  atdecc_header_s header;
  unique_id_t target_entity_id;     // target entity ID
  unique_id_t controller_entity_id; // controller entity ID
  uint8_t seq_id[2];                // sequence ID
} aecp_common_s;                    // 22 bytes

/* AECP AEM common data */
typedef struct {
  uint8_t cr : 1;             // request for controller to perform an action
  uint8_t unsolicited : 1;    // unsolicited notification
  uint8_t command_type_h : 6; // high order bits of command type
  uint8_t command_type;       // command type
} aecp_common_aem_s;          // 2 bytes

/* AECP AEM basic command */
// used for some commands
typedef struct {
  aecp_common_s common;
  aecp_common_aem_s aem;
} aecp_aem_basic_s;

/* AECP AEM short command */
typedef struct {
  aecp_common_s common;
  aecp_common_aem_s aem;
  uint8_t descriptor_type[2];  // descriptor type
  uint8_t descriptor_index[2]; // descriptor index
} aecp_aem_short_s;            // 28 bytes

/* AECP acquire entity command and response */
typedef struct {
  aecp_common_s common;
  aecp_common_aem_s aem;
  uint8_t reserved1 : 7;  // reserved
  uint8_t release : 1;    // release the acquired entity
  uint8_t reserved2[2];   // reserved
  uint8_t persistent : 1; // Acquire the ATDECC Entity and disable the
                          // CONTROLLER_AVAILABLE test for future ACQUIRE_ENTITY
                          // commands until released. The ATDECC Entity returns
                          // an ENTITY_ACQUIRED response immediately to any
                          // other Controller.
  uint8_t reserved3 : 7;  // reserved
  unique_id_t owner_id;   // 0 for command, owner id for response
  uint8_t descriptor_type[2];  // descriptor type being acquired
  uint8_t descriptor_index[2]; // descriptor index being acquired
} aecp_acquire_entity_s;       // 40 bytes

/* AECP lock entity command and response */
typedef struct {
  aecp_common_s common;
  aecp_common_aem_s aem;
  uint8_t reserved1[3];       // reserved
  uint8_t unlock : 1;         // Unlock the entity
  uint8_t reserved2 : 7;      // reserved
  unique_id_t locked_id;      // 0 for command, id of locked entity for response
  uint8_t descriptor_type[2]; // descriptor type being locked
  uint8_t descriptor_index[2]; // descriptor index being locked
} aecp_lock_entity_s;          // 40 bytes

/* AECP entity available command
 * uses common AEM basic command format
 */
typedef aecp_aem_basic_s aecp_entity_available_s; // 24 bytes

/* AECP entity available response */
typedef struct {
  aecp_common_s common;
  aecp_common_aem_s aem;
  uint8_t reserved1[3];               // reserved
  uint8_t entity_acquired : 1;        // Entity is acquired
  uint8_t entity_locked : 1;          // Entity is locked
  uint8_t subentity_acquired : 1;     // subentity is acquired
  uint8_t subentity_locked : 1;       // subentity is locked
  uint8_t reserved2 : 4;              // reserved
  unique_id_t acquired_controller_id; // acquired controller id
  unique_id_t locked_controller_id;   // locked controller id
} aecp_entity_available_rsp_s;        // 44 bytes

/* AECP controller available command and response
 * both use common format with no command specific data
 */
typedef aecp_aem_basic_s aecp_controller_available_s; // 24 bytes

/* AECP read descriptor command */
typedef struct {
  aecp_common_s common;
  aecp_common_aem_s aem;
  uint8_t configuration_index[2]; // configuration index
  uint8_t reserved[2];            // reserved
  uint8_t descriptor_type[2];     // descriptor type
  uint8_t descriptor_index[2];    // descriptor index
} aecp_read_descriptor_s;         // 32 bytes

/* AECP read descriptor response */
typedef struct {
  aecp_common_s common;
  aecp_common_aem_s aem;
  uint8_t configuration_index[2]; // configuration index
  uint8_t reserved[2];            // reserved
  uint8_t descriptor_type[2];
  uint8_t descriptor_index[2];
  uint8_t descriptor_data[AEM_MAX_DESC_LEN]; // descriptor data; variable length
} aecp_read_descriptor_rsp_s;                // 536 bytes

/* AECP get configuration command
 * uses common format with no command specific data
 */
typedef aecp_aem_basic_s aecp_get_configuration_s; // 24 bytes

/* AECP get configuration response */
typedef struct {
  aecp_common_s common;
  aecp_common_aem_s aem;
  uint8_t reserved[2];            // reserved
  uint8_t configuration_index[2]; // configuration index
} aecp_get_configuration_rsp_s;   // 28 bytes

/* AECP stream flags
 * used in stream descriptor
 */
typedef struct {
  uint8_t clock_sync_source : 1; // Indicates that the Stream is a preferred
                                 // clock synchronization source.
  uint8_t
      class_a : 1; // Indicates that the Stream supports streaming at Class A.
  uint8_t
      class_b : 1; // Indicates that the Stream supports streaming at Class B.
  uint8_t supports_encrypted : 1;       // Indicates that the Stream supports
                                        // streaming with encrypted PDUs.
  uint8_t primary_backup_supported : 1; // Indicates that the
                                        // backup_talker_entity_id_0 and the
                                        // backup_talker_entity_id_0 fields are
                                        // supported.
  uint8_t
      primary_backup_valid : 1; // Indicates that the backup_talker_entity_id_0
                                // and the backup_talker_entity_id_0 fields are
                                // valid.
  uint8_t secondary_backup_supported : 1; // Indicates that the
                                          // backup_talker_entity_id_1 and the
                                          // backup_talker_entity_id_1 fields
                                          // are supported.
  uint8_t
      secondary_backup_valid : 1; // Indicates that the
                                  // backup_talker_entity_id_1 and the
                                  // backup_talker_entity_id_1 fields are valid.
  uint8_t tertiary_backup_supported : 1; // Indicates that the
                                         // backup_talker_entity_id_2 and the
                                         // backup_talker_entity_id_2 fields are
                                         // supported.
  uint8_t
      tertiary_backup_valid : 1; // Indicates that the backup_talker_entity_id_2
                                 // and the backup_talker_entity_id_2 fields are
                                 // valid.
  uint8_t
      supports_avtp_udp_v4 : 1; // Indicates that the Stream supports streaming
                                // using AVTP over UDP/IPv4 (1722-2016 Annex J).
  uint8_t
      supports_avtp_udp_v6 : 1; // Indicates that the Stream supports streaming
                                // using AVTP over UDP/IPv6 (1722-2016 Annex J).
  uint8_t no_support_avtp_native : 1; // Indicates that the Stream does not
                                      // support streaming with native (L2,
                                      // Ethertype 0x22f0) AVTPDUs.
  uint8_t timing_field_valid : 1; // Indicates that the timing field contains a
                                  // valid TIMING descriptor index
  uint8_t no_media_clock : 1;  // Indicates that the stream does not use a media
                               // clock and so the clock_domain_index field does
                               // not contain a valid index.
  uint8_t supports_no_srp : 1; // Indicates that the Stream supports streaming
                               // without an SRP preservation.
} aem_stream_flags_s;          // 2 bytes

/* AECP stream info flags
 * used in get stream info response
 */
typedef struct {
  /* 1722.1 Table 7-145 lists flag field values in AVDECC bit order:
   * STREAM_FORMAT_VALID is 0x80000000 and CLASS_B is 0x00000001. ESP-IDF/GCC
   * allocates uint8_t bitfields LSB-to-MSB within each byte, so fields below
   * are declared in that byte-local order for direct wire serialization. */
  uint8_t not_registering_srp : 1;  // For a STREAM_INPUT, indicates that the
                                    // Listener is not registering an SRP
                                    // TalkerAdvertise or TalkerFailed attribute
                                    // for the stream. For a STREAM_OUTPUT,
                                    // indicates that the Talker is declaring an
                                    // SRP TalkerAdvertise or TalkerFailed
                                    // attribute and not registering a matching
                                    // Listener attribute for the stream.
  uint8_t stream_vlan_id_valid : 1; // Indicates that the stream_vlan_id field
                                    // is valid.
  uint8_t connected : 1; // The Stream has been connected with ACMP. This may
                         // only be set in a response.
  uint8_t msrp_failure_valid : 1;    // The values in the msrp_failure_code and
                                     // msrp_failure_bridge_id fields are valid.
  uint8_t stream_dest_mac_valid : 1; // The value in the stream_dest_mac field
                                     // is valid.
  uint8_t msrp_acc_lat_valid : 1; // The value in the msrp_accumulated_latency
                                  // field is valid.
  uint8_t stream_id_valid : 1;    // The value in the stream_id field is valid.
  uint8_t
      stream_format_valid : 1; // The value in the stream_format field is valid
                               // and is to be used to change the StreamFormat
                               // if it is a SET_STREAM_INFO command.
  uint8_t reserved1 : 3;       // Reserved bits
  uint8_t ip_flags_valid : 1;  // The value in the ip_flags field is valid.
  uint8_t ip_src_port_valid : 1; // The value in the source_port field is valid.
  uint8_t ip_dst_port_valid : 1; // The value in the destination_port field is
                                 // valid.
  uint8_t ip_src_addr_valid : 1; // The value in the source_ip_address field is
                                 // valid.
  uint8_t ip_dst_addr_valid : 1; // The value in the destination_ip_address
                                 // field is valid.
  uint8_t no_srp : 1; // Indicates that SRP is not being used for the stream.
                      // The Talker will not register a TalkerAdvertise or wait
                      // for a Listener registration before streaming.
  uint8_t reserved2 : 7; // Reserved bits
  uint8_t class_b : 1; // Indicates that the Stream is Class B instead of Class
                       // A (default 0 is classA)
  uint8_t fast_connect : 1; // Reserved for backward compatibility. This flag
                            // used to indicate that the Stream was connected in
                            // Fast Connect Mode or is presently trying to
                            // connect in Fast Connect Mode.
  uint8_t saved_state : 1;  // Reserved for backward compatibility. This flag
                            // used to indicate that the connection has saved
                            // ACMP state associated with FAST_CONNECT.
  uint8_t
      streaming_wait : 1; // The Stream is presently in STREAMING_WAIT, either
                          // it was connected with STREAMING_WAIT flag set or it
                          // was stopped with STOP_STREAMING command.
  uint8_t supports_encrypted : 1; // Indicates that the Stream supports
                                  // streaming with encrypted PDUs.
  uint8_t encrypted_pdu : 1; // Indicates that the Stream is using encrypted
                             // PDUs.
  uint8_t talker_failed : 1; // Indicates that the Listener has registered an
                             // SRP TalkerFailed attribute for the Stream.
  uint8_t reserved3 : 1;     // Reserved bit
} aem_stream_info_flags_s;   // 4 bytes

/* AVTP stream format types (am824, aaf_pcm, crf, and the
 * avtp_stream_format_s union) live in avtp.h. */

/* Stream descriptor (input or output) */
typedef struct {
  uint8_t object_name[64];          // UTF8 string containing a Stream name.
  uint8_t localized_description[2]; // The localized string reference pointing
                                    // to the localized Stream name. See 7.3.7.
  uint8_t clock_domain_index[2];    // The descriptor_index of the Clock Domain
                                    // providing the media clock for the Stream.
                                    // See 7.2.9.
  aem_stream_flags_s stream_flags;  // Flags describing capabilities or features
                                    // of the Stream. See Table 79.
  avtp_stream_format_s current_format; // The Stream format of the current
                                       // format, as defined in 7.3.3.
  uint8_t formats_offset[2]; // The offset from the start of the descriptor for
                             // the first octet of the formats. This field is
                             // 138 for this version of AEM.
  uint8_t number_of_formats[2]; // The number of formats supported by this audio
                                // Stream. The value of this field is referred
                                // to as N. The maximum value for this field is
                                // 46 for this version of AEM.
  unique_id_t
      backup_talker_entity_id_0; // The primary backup ATDECCTalker's EntityID.
  uint8_t backup_talker_unique_id_0[2];  // The primary backup ATDECCTalker's
                                         // UniqueID.
  unique_id_t backup_talker_entity_id_1; // The secondary backup ATDECCTalker's
                                         // EntityID.
  uint8_t backup_talker_unique_id_1[2];  // The secondary backup ATDECCTalker's
                                         // UniqueID.
  unique_id_t
      backup_talker_entity_id_2; // The tertiary backup ATDECCTalker's EntityID.
  uint8_t backup_talker_unique_id_2[2];  // The tertiary backup ATDECCTalker's
                                         // UniqueID.
  unique_id_t backedup_talker_entity_id; // The EntityID of the ATDECCTalker
                                         // that this Stream is backing up.
  uint8_t backedup_talker_unique_id[2]; // The UniqueID of the ATDECCTalker that
                                        // this Stream is backing up.
  uint8_t avb_interface_index[2]; // The descriptor_index of the AVB_INTERFACE
                                  // from which this Stream is sourced or to
                                  // which it is sinked.
  uint8_t buffer_length
      [4]; // The length in nanoseconds of the MAC's ingress or egress buffer as
           // defined in IEEE Std1722-2016 Figure5.4. For a STREAM_INPUT this is
           // the MAC's ingress buffer size and for a STREAM_OUTPUT this is the
           // MAC's egress buffer size. This is the length of the buffer between
           // the IEEE Std1722-2016 reference plane and the MAC.
  uint8_t
      redundant_offset[2]; // The offset from the start of the descriptor for
                           // the first octet of the redundant_streams array.
                           // This field is 138+8*N for this version of AEM.
  uint8_t number_of_redundant_streams
      [2]; // The number of redundant streams supported by this audio Stream.
           // The value of this field is referred to as R. The maximum value for
           // this field is 8 for this version of AEM.
  uint8_t timing[2]; // The TIMING descriptor index which represents the source
                     // of gPTP time for the stream.
  avtp_stream_format_s
      formats[AEM_MAX_NUM_FORMATS]; // Array of Stream formats of the supported
                                    // formats, as defined in 7.3.3.
  // uint8_t redundant_streams[2*R];                     // NOT YET SUPPORTED.
  // Array of redundant STREAM_INPUT or STREAM_OUTPUT descriptor indices. The
  // current version of AEM doesn’t specify an ordering for the elements of this
  // array.
} aem_stream_desc_s;

/* AEM stream summary
 * used in get stream info response and talker/listener list
 */
typedef struct {
  aem_stream_info_flags_s flags;       // stream descriptor flags
  avtp_stream_format_s stream_format;  // stream format
  unique_id_t stream_id;               // stream ID
  uint8_t msrp_accumulated_latency[4]; // MSRP accumulated latency
  eth_addr_t dest_addr;                // stream destination MAC address
  uint8_t msrp_failure_code;           // MSRP failure code
} aem_stream_summary_s;                // 17 bytes

/* AECP set/get stream format command and response */
typedef struct {
  aecp_common_s common;
  aecp_common_aem_s aem;
  uint8_t descriptor_type[2];         // descriptor type
  uint8_t descriptor_index[2];        // descriptor index
  avtp_stream_format_s stream_format; // stream format
} aecp_stream_format_s;               // 36 bytes

/* AECP get stream info command */
typedef aecp_aem_short_s aecp_get_stream_info_s; // 28 bytes

/* AECP set stream info command
 * also used for get/set stream info response
 */
typedef struct {
  aecp_common_s common;
  aecp_common_aem_s aem;
  uint8_t descriptor_type[2];  // descriptor type
  uint8_t descriptor_index[2]; // descriptor index
  aem_stream_summary_s stream; // stream summary
  uint8_t reserved;
  uint8_t msrp_failure_bridge_id[8]; // MSRP failure bridge ID
  uint8_t vlan_id[2];                // stream VLAN ID
  uint8_t ip_flags[2];               // stream destination port
  uint8_t src_port[2];               // stream source port
  uint8_t dest_port[2];              // stream destination port
  uint8_t src_ip_addr[16];           // stream source IP address
  uint8_t dest_ip_addr[16];          // stream destination IP address
} aecp_set_stream_info_s;            // 108 bytes

/* AECP get stream info response */
// same as set stream info command
typedef aecp_set_stream_info_s aecp_get_stream_info_rsp_s;

/* AECP set stream info response */
// same as set stream info command
typedef aecp_set_stream_info_s aecp_set_stream_info_rsp_s;

/* AECP get AVB info command */
typedef aecp_aem_short_s aecp_get_avb_info_s; // 28 bytes

/* AECP get AVB info flags */
typedef struct {
  uint8_t as_capable : 1;   // the interface is IEEE 802.1AS-2020 capable
  uint8_t gptp_enabled : 1; // gPTP is enabled on this interface
  uint8_t srp_enabled : 1;  // SRP is enabled on this interface
  uint8_t avtp_down : 1;    // the interface is not capable of transmitting or
                            // receiving AVTPDUs
  uint8_t avtp_down_valid : 1; // the value of the AVTP_DOWN flag bit is valid
  uint8_t reserved : 3;        // reserved
} aecp_avb_info_flags_s;       // 1 byte

/* AEM MSRP mapping */
typedef struct {
  uint8_t traffic_class; // the traffic class
  uint8_t priority;      // the priority for the traffic class
  uint8_t vlan_id[2];    // the VLAN ID for the traffic class
} aem_msrp_mapping_s;    // 4 bytes

/* AECP get AVB info response */
typedef struct {
  aecp_common_s common;
  aecp_common_aem_s aem;
  uint8_t descriptor_type[2];  // descriptor type (AVB_INTERFACE)
  uint8_t descriptor_index[2]; // descriptor index
  unique_id_t
      gptp_btc_id; // gPTP BTC identity for this interface (1722.1 field)
  uint8_t
      propagation_delay[4]; // propagation delay in nanoseconds from ptp pdelay
  uint8_t gptp_domain_number;  // gPTP domain number of the BTC on this interface
  aecp_avb_info_flags_s flags; // AVB info flags
  uint8_t msrp_mappings_count[2];      // number of MSRP mappings
  aem_msrp_mapping_s msrp_mappings[2]; // MSRP mappings (class A and B)
} aecp_get_avb_info_rsp_s;             // 52 bytes

/* AECP get AS path command */
typedef struct {
  aecp_common_s common;
  aecp_common_aem_s aem;
  uint8_t descriptor_index[2]; // descriptor index
  uint8_t reserved[2];         // reserved
} aecp_get_as_path_s;          // 28 bytes

/* AECP get AS path response */
#define AECP_MAX_AS_PATH_COUNT 7
typedef struct {
  aecp_common_s common;
  aecp_common_aem_s aem;
  uint8_t descriptor_index[2]; // descriptor index
  uint8_t count[2];            // number of clock identities in path sequence
  unique_id_t path_sequence[AECP_MAX_AS_PATH_COUNT]; // clock identity path
} aecp_get_as_path_rsp_s;                            // 44 bytes

/* AEM Entity counters valid flags */
typedef struct {
  uint8_t entity_specific8 : 1;
  uint8_t entity_specific7 : 1;
  uint8_t entity_specific6 : 1;
  uint8_t entity_specific5 : 1;
  uint8_t entity_specific4 : 1;
  uint8_t entity_specific3 : 1;
  uint8_t entity_specific2 : 1;
  uint8_t entity_specific1 : 1;
  uint8_t reserved[3];
} aem_entity_counters_val_s; // 4 bytes

/* AEM Stream input counters valid flags */
typedef struct {
  uint8_t entity_specific8 : 1;
  uint8_t entity_specific7 : 1;
  uint8_t entity_specific6 : 1;
  uint8_t entity_specific5 : 1;
  uint8_t entity_specific4 : 1;
  uint8_t entity_specific3 : 1;
  uint8_t entity_specific2 : 1;
  uint8_t entity_specific1 : 1;
  uint8_t reserved1;
  uint8_t unsupported_format : 1;
  uint8_t late_ts : 1;
  uint8_t early_ts : 1;
  uint8_t frames_rx : 1;
  uint8_t reserved2 : 4;
  uint8_t media_locked : 1;
  uint8_t media_unlocked : 1;
  uint8_t stream_interrupted : 1;
  uint8_t seq_num_mismatch : 1;
  uint8_t media_reset : 1;
  uint8_t ts_uncertain : 1;
  uint8_t ts_valid : 1;
  uint8_t ts_not_valid : 1;
} aem_stream_in_counters_val_s; // 4 bytes

/* AEM Stream output counters valid flags */
typedef struct {
  uint8_t entity_specific8 : 1;
  uint8_t entity_specific7 : 1;
  uint8_t entity_specific6 : 1;
  uint8_t entity_specific5 : 1;
  uint8_t entity_specific4 : 1;
  uint8_t entity_specific3 : 1;
  uint8_t entity_specific2 : 1;
  uint8_t entity_specific1 : 1;
  uint8_t reserved[2];
  uint8_t stream_start : 1;
  uint8_t stream_stop : 1;
  uint8_t stream_interrupted : 1;
  uint8_t media_reset : 1;
  uint8_t ts_uncertain : 1;
  uint8_t ts_valid : 1;
  uint8_t ts_not_valid : 1;
  uint8_t frames_tx : 1;
} aem_stream_out_counters_val_s; // 4 bytes

/* AEM AVB interface counters valid flags */
typedef struct {
  uint8_t entity_specific8 : 1;
  uint8_t entity_specific7 : 1;
  uint8_t entity_specific6 : 1;
  uint8_t entity_specific5 : 1;
  uint8_t entity_specific4 : 1;
  uint8_t entity_specific3 : 1;
  uint8_t entity_specific2 : 1;
  uint8_t entity_specific1 : 1;
  uint8_t reserved1[2];
  uint8_t link_up : 1;
  uint8_t link_down : 1;
  uint8_t frames_tx : 1;
  uint8_t frames_rx : 1;
  uint8_t rx_crc_error : 1;
  uint8_t gptp_btc_changed : 1;
  uint8_t reserved2 : 2;
} aem_avb_interface_counters_val_s; // 4 bytes

/* AEM clock domain counters valid flags */
typedef struct {
  uint8_t entity_specific8 : 1;
  uint8_t entity_specific7 : 1;
  uint8_t entity_specific6 : 1;
  uint8_t entity_specific5 : 1;
  uint8_t entity_specific4 : 1;
  uint8_t entity_specific3 : 1;
  uint8_t entity_specific2 : 1;
  uint8_t entity_specific1 : 1;
  uint8_t reserved1[2];
  uint8_t locked : 1;
  uint8_t unlocked : 1;
  uint8_t reserved2 : 6;
} aem_clock_domain_counters_val_s; // 4 bytes

/* AEM entity counters block */
typedef struct {
  uint8_t reserved[96];
  uint8_t entity_specific8[4];
  uint8_t entity_specific7[4];
  uint8_t entity_specific6[4];
  uint8_t entity_specific5[4];
  uint8_t entity_specific4[4];
  uint8_t entity_specific3[4];
  uint8_t entity_specific2[4];
  uint8_t entity_specific1[4];
} aem_entity_counters_s; // 128 bytes

/* AEM Stream input counters block */
typedef struct {
  uint8_t media_locked[4];   // Increments on a Stream media clock locking.
  uint8_t media_unlocked[4]; // Increments on a Stream media clock unlocking.
  uint8_t
      stream_interrupted[4]; // Increments when Stream playback is interrupted.
  uint8_t
      seq_num_mismatch[4]; // Increments when a Stream data AVTPDU is received
                           // with a non-sequential sequence_num field.
  uint8_t media_reset[4];  // Increments on a toggle of the mr bit in the Stream
                           // data AVTPDU.
  uint8_t ts_uncertain[4]; // Increments on a toggle of the tu bit in the Stream
                           // data AVTPDU.
  uint8_t ts_valid[4]; // Increments on receipt of a Stream data AVTPDU with the
                       // tv bit set.
  uint8_t ts_not_valid[4]; // Increments on receipt of a Stream data AVTPDU with
                           // the tv bit cleared.
  uint8_t
      unsupported_format[4]; // Increments on receipt of a Stream data AVTPDU
                             // that contains an unsupported media type.
  uint8_t late_ts[4];   // Increments on receipt of a Stream data AVTPDU with an
                        // avtp_timestamp field that is in the past.
  uint8_t early_ts[4];  // Increments on receipt of a Stream data AVTPDU with an
                        // avtp_timestamp field that is too far in the future to
                        // process.
  uint8_t frames_rx[4]; // Increments on each Stream data AVTPDU received.
  uint8_t reserved[48]; // Reserved for future use
  uint8_t entity_specific8[4];
  uint8_t entity_specific7[4];
  uint8_t entity_specific6[4];
  uint8_t entity_specific5[4];
  uint8_t entity_specific4[4];
  uint8_t entity_specific3[4];
  uint8_t entity_specific2[4];
  uint8_t entity_specific1[4];
} aem_stream_in_counters_s; // 128 bytes

/* AEM Stream output counters block */
typedef struct {
  uint8_t stream_start[4]; // Increments when a stream is started.
  uint8_t stream_stop[4];  // Increments when a stream is stopped.
  uint8_t
      stream_interrupted[4]; // Increments when Stream playback is interrupted.
  uint8_t media_reset[4];  // Increments on a toggle of the mr bit in the Stream
                           // data AVTPDU.
  uint8_t ts_uncertain[4]; // Increments on a toggle of the tu bit in the Stream
                           // data AVTPDU.
  uint8_t ts_valid[4]; // Increments on receipt of a Stream data AVTPDU with the
                       // tv bit set.
  uint8_t ts_not_valid[4]; // Increments on receipt of a Stream data AVTPDU with
                           // the tv bit cleared.
  uint8_t frames_tx[4];    // Increments on each Stream data AVTPDU transmitted.
  uint8_t reserved[64];    // Reserved for future use
  uint8_t entity_specific8[4];
  uint8_t entity_specific7[4];
  uint8_t entity_specific6[4];
  uint8_t entity_specific5[4];
  uint8_t entity_specific4[4];
  uint8_t entity_specific3[4];
  uint8_t entity_specific2[4];
  uint8_t entity_specific1[4];
} aem_stream_out_counters_s; // 128 bytes

/* AEM AVB interface counters block */
typedef struct {
  uint8_t link_up[4];         // Total number of network link up events.
  uint8_t link_down[4];       // Total number of network link down events.
  uint8_t frames_tx[4];       // Total number of network frames transmitted.
  uint8_t frames_rx[4];       // Total number of network frames received.
  uint8_t rx_crc_error[4];    // Total number of network frames received with
                              // incorrect CRC.
  uint8_t gptp_btc_changed[4]; // gPTP BTC change count.
  uint8_t reserved[72];       // Reserved for future use
  uint8_t entity_specific8[4];
  uint8_t entity_specific7[4];
  uint8_t entity_specific6[4];
  uint8_t entity_specific5[4];
  uint8_t entity_specific4[4];
  uint8_t entity_specific3[4];
  uint8_t entity_specific2[4];
  uint8_t entity_specific1[4];
} aem_avb_interface_counters_s; // 128 bytes

/* AEM clock domain counters block */
typedef struct {
  uint8_t locked[4];    // Increments on a clock locking event.
  uint8_t unlocked[4];  // Increments on a clock unlocking event.
  uint8_t reserved[88]; // Reserved for future use
  uint8_t entity_specific8[4];
  uint8_t entity_specific7[4];
  uint8_t entity_specific6[4];
  uint8_t entity_specific5[4];
  uint8_t entity_specific4[4];
  uint8_t entity_specific3[4];
  uint8_t entity_specific2[4];
  uint8_t entity_specific1[4];
} aem_clock_domain_counters_s; // 128 bytes

/* AEM counters valid flags union */
typedef union {
  aem_entity_counters_val_s entity_counters_val;
  aem_stream_in_counters_val_s stream_in_counters_val;
  aem_stream_out_counters_val_s stream_out_counters_val;
  aem_avb_interface_counters_val_s avb_interface_counters_val;
  aem_clock_domain_counters_val_s clock_domain_counters_val;
} aem_counters_val_u; // 4 bytes

/* AEM counters block union */
typedef union {
  aem_entity_counters_s entity_counters;
  aem_stream_in_counters_s stream_in_counters;
  aem_stream_out_counters_s stream_out_counters;
  aem_avb_interface_counters_s avb_interface_counters;
  aem_clock_domain_counters_s clock_domain_counters;
} aem_counters_block_u; // 128 bytes

/* AECP get counters command uses basic command format */
typedef aecp_aem_short_s aecp_get_counters_s; // 28 bytes

/* SET/GET_MAX_TRANSIT_TIME command and response (IEEE 1722.1-2021 §7.4.77/78)
 * max_transit_time is a 64-bit uint in nanoseconds. GET command omits the
 * max_transit_time field (payload = 4 bytes); SET command and both responses
 * include it (payload = 12 bytes). */
typedef struct {
  aecp_common_s common;
  aecp_common_aem_s aem;
  uint8_t descriptor_type[2];  // always STREAM_OUTPUT
  uint8_t descriptor_index[2]; // stream output index
  uint8_t max_transit_time[8]; // max transit time in nanoseconds (uint64)
} aecp_max_transit_time_s;     // 36 bytes

/* AECP get counters response */
typedef struct {
  aecp_common_s common;
  aecp_common_aem_s aem;
  uint8_t descriptor_type[2];          // descriptor type
  uint8_t descriptor_index[2];         // descriptor index
  aem_counters_val_u counters_valid;   // counters valid
  aem_counters_block_u counters_block; // counters block
} aecp_get_counters_rsp_s;             // 160 bytes

/* AECP register unsol flags */
typedef struct {
  uint8_t reserved1[3];
  uint8_t time_limited : 1;
  uint8_t reserved2 : 7;
} aecp_register_unsol_flags_s; // 4 bytes

/* AECP register unsolicited notification command and response */

typedef struct {
  aecp_common_s common;
  aecp_common_aem_s aem;
  aecp_register_unsol_flags_s flags;
} aecp_register_unsol_notif_s; // 28 bytes

/* AECP deregister unsolicited notification command and response
 * uses common format with no command specific data
 */
typedef aecp_aem_basic_s aecp_deregister_unsol_notif_s; // 24 bytes

/* AEM configuration descriptor counts */
typedef struct {
  uint8_t descriptor_type[2];
  uint8_t count[2];
} aem_config_desc_count_s; // 4 bytes

/* AEM descriptors */

/* AEM Entity descriptor */
typedef struct {
  avb_entity_summary_s summary;
  avb_entity_detail_s detail;
} aem_entity_desc_s; // 312 bytes

/* AEM Configuration descriptor */
typedef struct {
  uint8_t
      object_name[64]; // 64-octet UTF8 string containing a Configuration name.
  uint8_t
      localized_description[2]; // The localized string reference pointing to
                                // the localized Configuration name. See 7.3.7.
  uint8_t
      descriptor_counts_count[2]; // The number of descriptor counts in the
                                  // descriptor_countsfield. This is referred to
                                  // as N.The maximum value for this field is
                                  // 108 for this version of AEM.
  uint8_t descriptor_counts_offset
      [2]; // The offset to the descriptor_counts field from the start of the
           // descriptor. This field is set to 74 for this version of AEM.
  aem_config_desc_count_s
      descriptor_counts[AEM_MAX_NUM_DESC]; // Counts of the top-level
                                           // descriptors. See 7.2.2.1.
} aem_config_desc_s;                       // 114 bytes

/* AEM sample rate type */
typedef uint8_t aem_sample_rate_t[4];

/* AEM Audio Unit descriptor */
typedef struct {
  uint8_t object_name[64];          // 64-octet UTF8 string containing a name.
  uint8_t localized_description[2]; // Pointer to the localized name. See 7.3.7.
  uint8_t clock_domain_index[2];
  uint8_t num_stream_input_ports[2];
  uint8_t base_stream_input_port[2];
  uint8_t num_stream_output_ports[2];
  uint8_t base_stream_output_port[2];
  uint8_t unused[56];                 // these fields not yet supported
  uint8_t current_sampling_rate[4];
  uint8_t sampling_rate_offset[2];    // 144 for this version of AEM
  uint8_t sampling_rates_count[2];
  aem_sample_rate_t sampling_rates[AEM_MAX_NUM_SAMPLE_RATES];
} aem_audio_unit_desc_s; // 180 bytes

/* AEM Stream input and output descriptors are defined further above */

/* AEM AVB Interface flags (used for descriptor) */
typedef struct {
  uint8_t reserved1;               // Reserved for future use
  uint8_t gptp_btc_supported : 1;   // Supports 802.1AS gPTP BTC functionality
  uint8_t gptp_supported : 1;      // Supports 802.1AS gPTP functionality
  uint8_t srp_supported : 1;       // Supports 802.1Q clause for SRP
  uint8_t fqtss_not_supported : 1; // Does not support 802.1Q clause for FQTSS
  uint8_t sched_traffic_supported : 1; // Supports 802.1Q clauses for scheduled
                                       // traffic
  uint8_t can_listen_to_self : 1; //  Listener stream sink on this interface can
                                  //  listen to a talker stream source on the
                                  //  sam interface.
  uint8_t
      can_listen_to_other_self : 1; // Listener stream sink on this interface
                                    // can listen to a talker stream source of
                                    // another interface within same Entity
  uint8_t reserved2 : 1;            // Reserved for future use
} aem_avb_interface_flags_s;        // 2 bytes

/* AEM AVB Interface descriptor */
typedef struct {
  uint8_t object_name[64];          // 64-octet UTF8 string containing a name.
  uint8_t localized_description[2]; // Pointer to the localized name. See 7.3.7.
  eth_addr_t mac_address;
  aem_avb_interface_flags_s flags;
  unique_id_t clock_identity;
  uint8_t priority1;
  uint8_t clock_class;
  uint8_t offset_scaled_log_variance[2];
  uint8_t clock_accuracy;
  uint8_t priority2;
  uint8_t domain_number;
  uint8_t log_sync_interval;
  uint8_t log_announce_interval;
  uint8_t log_pdelay_interval;
  uint8_t port_number[2];
  uint8_t number_of_controls[2];
  uint8_t base_control[2];
} aem_avb_interface_desc_s;              // 98 bytes

/* AEM Clock Source flags (used for descriptor) */
typedef struct {
  uint8_t reserved1;     // Reserved for future use
  uint8_t stream_id : 1; // The input stream clock source is identified by the
                         // stream ID.
  uint8_t local_id : 1;  // The input stream clock source is identified by its
                         // local ID.
  uint8_t reserved2 : 6; // Reserved for future use
} aem_clock_source_flags_s; // 2 bytes

/* AEM Clock Source descriptor */
typedef struct {
  uint8_t object_name[64];          // 64-octet UTF8 string containing a name.
  uint8_t localized_description[2]; // Pointer to the localized name. See 7.3.7.
  aem_clock_source_flags_s clock_source_flags;
  uint8_t clock_source_type[2]; // must be one of aem_clock_source_type_t
  unique_id_t clock_source_id;
  uint8_t clock_source_location_type[2];  // must be one of aem_desc_type_t
  uint8_t clock_source_location_index[2];
} aem_clock_source_desc_s;                // 82 bytes

/* AEM Memory Object descriptor */
typedef struct {
  uint8_t object_name[64];          // 64-octet UTF8 string containing a name.
  uint8_t localized_description[2]; // Pointer to the localized name. See 7.3.7.
  uint8_t memory_object_type[2];    // must be one of aem_memory_obj_type_t
  uint8_t target_descriptor_type[2];  // must be one of aem_desc_type_t
  uint8_t target_descriptor_index[2];
  uint8_t start_address[8];
  uint8_t maximum_length[8];
  uint8_t length[8];
  uint8_t maximum_segment_length[8];
} aem_memory_object_desc_s;           // 104 bytes

/* AEM Locale descriptor */
typedef struct {
  uint8_t locale_identifier[64]; // 64-octet UTF8 string containing the locale
                                 // identifier.
  uint8_t number_of_strings[2]; // Number of strings descriptors in this locale.
                                // This is the same value for all locales in an
                                // ATDECC Entity.
  uint8_t base_strings[2]; // Descriptor index of the first Strings descriptor
                           // for this locale.
} aem_locale_desc_s;       // 68 bytes

/* AEM Strings descriptor */
typedef struct {
  uint8_t string_0[64]; // 64-octet UTF8 string at index 0.
  uint8_t string_1[64]; // 64-octet UTF8 string at index 1.
  uint8_t string_2[64]; // 64-octet UTF8 string at index 2.
  uint8_t string_3[64]; // 64-octet UTF8 string at index 3.
  uint8_t string_4[64]; // 64-octet UTF8 string at index 4.
  uint8_t string_5[64]; // 64-octet UTF8 string at index 5.
  uint8_t string_6[64]; // 64-octet UTF8 string at index 6.
} aem_strings_desc_s;   // 448 bytes

/* AEM Stream Port flags (used for descriptor) */
typedef struct {
  uint8_t reserved1;                  // Reserved for future use
  uint8_t clock_sync_source : 1;
  uint8_t async_sample_rate_conv : 1;
  uint8_t sync_sample_rate_conv : 1;
  uint8_t reserved2 : 5;              // Reserved for future use
} aem_stream_port_flags_s;            // 2 bytes

/* AEM Stream Port descriptor (input and output) */
typedef struct {
  uint8_t clock_domain_index[2];
  aem_stream_port_flags_s port_flags;
  uint8_t number_of_controls[2];
  uint8_t base_control[2];
  uint8_t number_of_clusters[2];
  uint8_t base_cluster[2];
  uint8_t number_of_maps[2];
  uint8_t base_map[2];
} aem_stream_port_desc_s;             // 16 bytes

/* AEM Audio Cluster descriptor */
typedef struct {
  uint8_t object_name[64];          // 64-octet UTF8 string containing a name.
  uint8_t localized_description[2]; // Pointer to the localized name. See 7.3.7.
  uint8_t signal_type[2];           // must be one of aem_desc_type_t
  uint8_t signal_index[2];
  uint8_t signal_output[2];
  uint8_t path_latency[4];
  uint8_t block_latency[4];
  uint8_t channel_count[2];
  uint8_t format[2];                // must be one of aem_audio_cluster_format_t
  uint8_t aes_data_type_reference[2];
  uint8_t aes_data_type[2];
} aem_audio_cluster_desc_s;           // 86 bytes

/* AEM Audio Mapping */
typedef struct {
  uint8_t mapping_stream_index[2];
  uint8_t mapping_stream_channel[2];
  uint8_t mapping_cluster_offset[2];
  uint8_t mapping_cluster_channel[2];
} aem_audio_mapping_s;                // 8 bytes

/* AEM Audio Map descriptor */
typedef struct {
  uint8_t mappings_offset[2];    // set to 8 for this version of AEM
  uint8_t number_of_mappings[2]; // number of channel mappings in the descriptor
                                 // (max is 62)
  aem_audio_mapping_s mappings[AEM_MAX_NUM_MAPPINGS];
} aem_audio_map_desc_s; // 84 bytes

/* AEM Identify Control value details */
typedef struct {
  uint8_t values[5];
  uint8_t units[2];
  uint8_t string_ref[2];
} aem_identify_control_value_s;

/* AEM Control descriptor */
typedef struct {
  uint8_t object_name[64];          // 64-octet UTF8 string containing a name.
  uint8_t localized_description[2]; // Pointer to the localized name. See 7.3.7.
  uint8_t block_latency[4];
  uint8_t control_latency[4];
  uint8_t control_domain[2];
  uint8_t control_value_type[2];
  uint8_t control_type[8];
  uint8_t reset_time[4];
  uint8_t values_offset[2];
  uint8_t number_of_values[2];
  uint8_t signal_type[2];
  uint8_t signal_index[2];
  uint8_t signal_output[2];
  uint8_t value_details[AEM_MAX_LEN_CONTROL_VAL_DETAILS];
} aem_control_desc_s; // 68 bytes

/* AEM Clock Source type used in Clock Domain descriptor */
typedef uint8_t aem_clock_source_t[2];

/* AEM Clock Domain descriptor */
typedef struct {
  uint8_t object_name[64];          // 64-octet UTF8 string containing a name.
  uint8_t localized_description[2]; // Pointer to the localized name. See 7.3.7.
  uint8_t clock_source_index[2];
  uint8_t clock_sources_offset[2];
  uint8_t clock_sources_count[2];   // number of clock sources in the descriptor
                                    // (max is 216)
  aem_clock_source_t
      clock_sources[AEM_MAX_NUM_CLOCK_SOURCES]; // 2 bytes per clock source
} aem_clock_domain_desc_s;                      // 92 bytes

/* AECP address access command and response */
typedef struct {
  aecp_common_s common;
  uint8_t tlv_count[2];
  uint8_t tlv_data[AVB_MAX_MSG_LEN];
} aecp_addr_access_s; // 624 bytes

/* AECP AVB community vendor unique common header */
typedef struct {
  aecp_common_s common;
  uint8_t protocol_id[6]; // CVU protocol ID (00-11-22-33-00-00)
  uint8_t command_type;   // command type
} aecp_cvu_common_s;

/* AVB Lite CVU SRP talker message */
typedef struct {
  aecp_cvu_common_s cvu;             // CVU common header
  msrp_talker_message_u msrp_talker; // MSRP talker message body
} aecp_cvu_srp_talker_s;

/* AVB Lite CVU SRP listener message */
typedef struct {
  aecp_cvu_common_s cvu;                 // CVU common header
  msrp_listener_message_s msrp_listener; // MSRP listener message body
} aecp_cvu_srp_listener_s;

/* AECP Milan vendor unique common header (Milan v1.3 Figure 5.4) */
typedef struct {
  aecp_common_s common;
  uint8_t protocol_id[6];     // MVU protocol ID (00-1B-C5-0A-C1-00)
  uint8_t u : 1;              // 0=command/response, 1=notification
  uint8_t command_type_h : 7; // command type high bits
  uint8_t command_type;       // command type low byte
} aecp_mvu_common_s;          // 30 bytes

/* GET_MILAN_INFO command (Figure 5.5) — no command-specific data */
typedef struct {
  aecp_mvu_common_s mvu;
  uint8_t reserved[2];
} aecp_mvu_get_milan_info_s; // 32 bytes

/* GET_MILAN_INFO response (Figure 5.6) */
typedef struct {
  aecp_mvu_common_s mvu;
  uint8_t reserved[2];
  uint8_t protocol_version[4];      // Milan protocol version (= 1)
  uint8_t features_flags[4];        // Table 5.17
  uint8_t certification_version[4]; // 4x uint8 dotted version, 0 if uncertified
  uint8_t specification_version[4]; // Milan spec version (1.3.0.0)
} aecp_mvu_get_milan_info_rsp_s;    // 48 bytes

/* BIND_STREAM command and response (Figure 5.10) */
typedef struct {
  aecp_mvu_common_s mvu;
  uint8_t flags[2];               // bit 15: STREAMING_WAIT
  uint8_t descriptor_type[2];     // always STREAM_INPUT
  uint8_t descriptor_index[2];    // stream input index
  unique_id_t talker_entity_id;   // talker to bind to
  uint8_t talker_stream_index[2]; // talker stream output index
  uint8_t reserved[2];
} aecp_mvu_bind_stream_s; // 48 bytes

/* UNBIND_STREAM command and response (Figure 5.11) */
typedef struct {
  aecp_mvu_common_s mvu;
  uint8_t reserved[2];
  uint8_t descriptor_type[2];  // always STREAM_INPUT
  uint8_t descriptor_index[2]; // stream input index
} aecp_mvu_unbind_stream_s;    // 36 bytes

/* SET/GET_MEDIA_CLOCK_REFERENCE_INFO command and response (Figure 5.8) */
typedef struct {
  aecp_mvu_common_s mvu;
  uint8_t clock_domain_index[2]; // CLOCK_DOMAIN descriptor index
  uint8_t flags;                 // Table 5.18
  uint8_t reserved1;
  uint8_t default_mcr_prio; // default media clock reference priority
  uint8_t user_mcr_prio;    // user media clock reference priority
  uint8_t reserved2[4];
  uint8_t media_clock_domain_name[64]; // UTF-8 name
} aecp_mvu_media_clock_ref_info_s;     // 104 bytes

/* GET_STREAM_INPUT_INFO_EX command (Figure 5.11 — same as UNBIND) */
typedef aecp_mvu_unbind_stream_s aecp_mvu_get_stream_input_info_ex_s;

/* GET_STREAM_INPUT_INFO_EX response (Figure 5.12)
 * Milan v1.3 §5.4.4.8: probing_status (3 bits) and acmp_status (5 bits) are
 * packed into a single byte, followed by 1 reserved byte. */
typedef struct {
  aecp_mvu_common_s mvu;
  uint8_t reserved1[2];
  uint8_t descriptor_type[2];   // always STREAM_INPUT
  uint8_t descriptor_index[2];  // stream input index
  unique_id_t talker_entity_id; // bound talker entity ID
  uint8_t talker_unique_id[2];  // bound talker stream index
  uint8_t probing_acmp_status;  // probing_status<<5 | acmp_status
  uint8_t reserved2;
} aecp_mvu_get_stream_input_info_ex_rsp_s; // 48 bytes

/* GET_SYSTEM_UNIQUE_ID command — same as GET_MILAN_INFO (no data) */
typedef aecp_mvu_get_milan_info_s aecp_mvu_get_system_unique_id_s;

/* SET/GET_SYSTEM_UNIQUE_ID response (Figure 5.7) */
typedef struct {
  aecp_mvu_common_s mvu;
  uint8_t reserved[2];
  uint8_t number[8];                   // 64-bit network-wide unique identifier
  uint8_t name[64];                    // UTF-8 name
} aecp_mvu_get_system_unique_id_rsp_s; // 104 bytes

/* AECP message union */
typedef union {
  atdecc_header_s header;
  aecp_aem_basic_s basic;
  aecp_common_s common;
  aecp_acquire_entity_s acquire_entity;
  aecp_lock_entity_s lock_entity;
  aecp_entity_available_s entity_available;
  aecp_entity_available_rsp_s entity_available_rsp;
  aecp_controller_available_s controller_available;
  aecp_get_configuration_s get_configuration;
  aecp_get_configuration_rsp_s get_configuration_rsp;
  aecp_read_descriptor_s read_descriptor;
  aecp_read_descriptor_rsp_s read_descriptor_rsp;
  aecp_stream_format_s stream_format;
  aecp_get_stream_info_s get_stream_info;
  aecp_get_stream_info_rsp_s get_stream_info_rsp;
  aecp_set_stream_info_s set_stream_info;         // not supported
  aecp_set_stream_info_rsp_s set_stream_info_rsp; // not supported
  aecp_get_avb_info_s get_avb_info;
  aecp_get_avb_info_rsp_s get_avb_info_rsp;
  aecp_get_as_path_s get_as_path;
  aecp_get_as_path_rsp_s get_as_path_rsp;
  aecp_max_transit_time_s max_transit_time;
  aecp_get_counters_s get_counters;
  aecp_get_counters_rsp_s get_counters_rsp;
  aecp_register_unsol_notif_s register_unsol_notif;
  aecp_deregister_unsol_notif_s deregister_unsol_notif;
  aecp_addr_access_s addr_access;
  aecp_cvu_common_s cvu;
  aecp_cvu_srp_talker_s cvu_srp_talker;
  aecp_cvu_srp_listener_s cvu_srp_listener;
  aecp_mvu_common_s mvu;
  aecp_mvu_get_milan_info_s mvu_get_milan_info;
  aecp_mvu_get_milan_info_rsp_s mvu_get_milan_info_rsp;
  aecp_mvu_bind_stream_s mvu_bind_stream;
  aecp_mvu_unbind_stream_s mvu_unbind_stream;
  aecp_mvu_media_clock_ref_info_s mvu_media_clock_ref_info;
  aecp_mvu_get_stream_input_info_ex_s mvu_get_stream_input_info_ex;
  aecp_mvu_get_stream_input_info_ex_rsp_s mvu_get_stream_input_info_ex_rsp;
  aecp_mvu_get_system_unique_id_s mvu_get_system_unique_id;
  aecp_mvu_get_system_unique_id_rsp_s mvu_get_system_unique_id_rsp;
  uint8_t raw[AVB_MAX_MSG_LEN];
} aecp_message_u;

/* MVU command types (Milan v1.3 Table 5.15) */
typedef enum {
  mvu_cmd_get_milan_info = 0x0000,
  mvu_cmd_set_system_unique_id = 0x0001,
  mvu_cmd_get_system_unique_id = 0x0002,
  mvu_cmd_set_media_clock_ref_info = 0x0003,
  mvu_cmd_get_media_clock_ref_info = 0x0004,
  mvu_cmd_bind_stream = 0x0005,
  mvu_cmd_unbind_stream = 0x0006,
  mvu_cmd_get_stream_input_info_ex = 0x0007
} mvu_cmd_type_t;

/* ACMP Message */
typedef struct {
  atdecc_header_s header;
  unique_id_t stream_id;             // stream ID
  unique_id_t controller_entity_id;  // controller entity ID
  unique_id_t talker_entity_id;      // talker entity ID
  unique_id_t listener_entity_id;    // listener entity ID
  uint8_t talker_uid[2];             // talker UID = stream output descr index
  uint8_t listener_uid[2];           // listener UID = stream input descr index
  eth_addr_t stream_dest_addr;       // stream destination address
  uint8_t connection_count[2];       // connection count
  uint8_t seq_id[2];                 // sequence ID
  uint8_t flags[2];                  // flags
  uint8_t stream_vlan_id[2];         // stream VLAN ID
  uint8_t conn_listeners_entries[2]; // connection listeners entries
} acmp_message_s;                    // 56 bytes

/* ACMP Message Extended */
typedef struct {
  acmp_message_s base;
  uint8_t ip_flags[2];      // IP flags
  uint8_t reserved[2];      // reserved
  uint8_t src_port[2];      // source port
  uint8_t dest_port[2];     // destination port
  uint8_t src_ip_addr[16];  // source IP address
  uint8_t dest_ip_addr[16]; // destination IP address
} acmp_message_extended_s;  // 96 bytes

/* ===== AVTP envelope union (spans all subtypes; lives here because it embeds ATDECC payloads) ===== */

/* AVTP message buffer */
typedef union {
  uint8_t subtype;
  aaf_pcm_message_s aaf;
  iec_61883_6_message_s iec;
  maap_message_s maap;
  adp_message_s adp;
  aecp_message_u aecp;
  acmp_message_s acmp;
  uint8_t raw[AVB_MAX_MSG_LEN];
} avtp_msgbuf_u;

/* ===== ATDECC inflight command tracking ===== */

/* ATDECC command message union */
// used for inflight commands
typedef union {
  atdecc_header_s header;
  aecp_aem_short_s aecp;
  acmp_message_s acmp;
  uint8_t raw[56];
} atdecc_command_u;

/* Inflight command (state for processing an asynchronous response to a command)
 */
// used for AECP and ACMP command tracking
typedef struct {
  struct timeval timeout;   // command timeout as a timeval
  bool retried;             // indicates if the command has been retried
  uint16_t aecp_seq_id;     // original AECP command sequence ID
  uint16_t acmp_seq_id;     // original ACMP command sequence ID
  atdecc_command_u command; // the command or partial command
  bool inbound;             // indicates if the command is inbound
} atdecc_inflight_command_s;

/* ===== ATDECC send / process functions ===== */

/* ATDECC send functions */
int avb_send_adp_entity_available(avb_state_s *state);
int avb_send_aecp_cmd_controller_available(avb_state_s *state,
                                           unique_id_t *target_id);
int avb_send_aecp_cmd_entity_available(avb_state_s *state,
                                       unique_id_t *target_id);
int avb_send_aecp_cmd_get_stream_info(avb_state_s *state,
                                      unique_id_t *target_id);
int avb_send_aecp_rsp_get_stream_info(avb_state_s *state,
                                      aecp_get_stream_info_s *msg,
                                      eth_addr_t *dest_addr);
int avb_send_aecp_unsol_get_stream_info(avb_state_s *state, uint16_t index,
                                        bool is_output);
int avb_send_aecp_cmd_get_counters(avb_state_s *state, unique_id_t *target_id);
int avb_send_aecp_rsp_get_counters(avb_state_s *state, aecp_get_counters_s *msg,
                                   eth_addr_t *dest_addr);
int avb_send_aecp_unsol_get_counters(avb_state_s *state,
                                     aem_desc_type_t descriptor_type,
                                     uint16_t index);
int avb_send_aecp_rsp_read_descr_entity(avb_state_s *state,
                                        aecp_read_descriptor_rsp_s *msg,
                                        eth_addr_t *dest_addr);
int avb_send_aecp_rsp_read_descr_configuration(avb_state_s *state,
                                               aecp_read_descriptor_rsp_s *msg,
                                               eth_addr_t *dest_addr);
int avb_send_aecp_rsp_read_descr_audio_unit(avb_state_s *state,
                                            aecp_read_descriptor_rsp_s *msg,
                                            eth_addr_t *dest_addr);
int avb_send_aecp_rsp_read_descr_stream(avb_state_s *state,
                                        aecp_read_descriptor_rsp_s *msg,
                                        eth_addr_t *dest_addr, bool is_output);
void avb_update_avb_interface_from_ptp(avb_state_s *state);
int avb_send_aecp_rsp_read_descr_avb_interface(avb_state_s *state,
                                               aecp_read_descriptor_rsp_s *msg,
                                               eth_addr_t *dest_addr);
int avb_send_aecp_rsp_read_descr_clock_source(avb_state_s *state,
                                              aecp_read_descriptor_rsp_s *msg,
                                              eth_addr_t *dest_addr);
int avb_send_aecp_rsp_read_descr_memory_obj(avb_state_s *state,
                                            aecp_read_descriptor_rsp_s *msg,
                                            eth_addr_t *dest_addr);
int avb_send_aecp_rsp_read_descr_locale(avb_state_s *state,
                                        aecp_read_descriptor_rsp_s *msg,
                                        eth_addr_t *dest_addr);
int avb_send_aecp_rsp_read_descr_strings(avb_state_s *state,
                                         aecp_read_descriptor_rsp_s *msg,
                                         eth_addr_t *dest_addr);
int avb_send_aecp_rsp_read_descr_stream_port(avb_state_s *state,
                                             aecp_read_descriptor_rsp_s *msg,
                                             eth_addr_t *dest_addr,
                                             bool is_output);
int avb_send_aecp_rsp_read_descr_audio_cluster(avb_state_s *state,
                                               aecp_read_descriptor_rsp_s *msg,
                                               eth_addr_t *dest_addr);
int avb_send_aecp_rsp_read_descr_audio_map(avb_state_s *state,
                                           aecp_read_descriptor_rsp_s *msg,
                                           eth_addr_t *dest_addr);
int avb_send_aecp_rsp_read_descr_control(avb_state_s *state,
                                         aecp_read_descriptor_rsp_s *msg,
                                         eth_addr_t *dest_addr);
int avb_send_aecp_rsp_read_descr_clock_domain(avb_state_s *state,
                                              aecp_read_descriptor_rsp_s *msg,
                                              eth_addr_t *dest_addr);
int avb_send_acmp_connect_rx_command(
    avb_state_s *state, unique_id_t *talker_id,
    unique_id_t *listener_id); // acting as controller
int avb_send_acmp_connect_tx_command(
    avb_state_s *state, unique_id_t *controller_id,
    unique_id_t *talker_id); // acting as listener
int avb_send_acmp_command(avb_state_s *state, acmp_msg_type_t msg_type,
                          acmp_message_s *command, bool retried,
                          bool track_inflight);
int avb_send_acmp_response(avb_state_s *state, acmp_msg_type_t msg_type,
                           acmp_message_s *response, acmp_status_t status);

/* ATDECC processing functions */
/* ADP processing functions */
int avb_process_adp(avb_state_s *state, adp_message_s *msg,
                    eth_addr_t *src_addr); // handle all adp messages

/* AECP processing functions */
int avb_process_aecp(avb_state_s *state, aecp_message_u *msg,
                     eth_addr_t *src_addr); // route to specific func
int avb_process_acmp(avb_state_s *state,
                     acmp_message_s *msg); // route to specific func
int avb_process_aecp_cmd_entity_available(avb_state_s *state,
                                          aecp_message_u *msg,
                                          eth_addr_t *src_addr);
int avb_process_aecp_cmd_register_unsol_notif(avb_state_s *state,
                                              aecp_message_u *msg,
                                              eth_addr_t *src_addr);
int avb_process_aecp_cmd_deregister_unsol_notif(avb_state_s *state,
                                                aecp_message_u *msg,
                                                eth_addr_t *src_addr);
int avb_process_aecp_cmd_lock_entity(avb_state_s *state, aecp_message_u *msg,
                                     eth_addr_t *src_addr);
int avb_process_aecp_cmd_acquire_entity(avb_state_s *state, aecp_message_u *msg,
                                        eth_addr_t *src_addr);
int avb_process_aecp_cmd_get_configuration(avb_state_s *state,
                                           aecp_message_u *msg,
                                           eth_addr_t *src_add);
int avb_process_aecp_cmd_read_descriptor(avb_state_s *state,
                                         aecp_message_u *msg,
                                         eth_addr_t *src_addr);
int avb_process_aecp_cmd_set_stream_format(avb_state_s *state,
                                           aecp_message_u *msg,
                                           eth_addr_t *src_addr);
int avb_process_aecp_cmd_get_stream_format(avb_state_s *state,
                                           aecp_message_u *msg,
                                           eth_addr_t *src_addr);
int avb_process_aecp_cmd_get_clock_source(avb_state_s *state,
                                          aecp_message_u *msg,
                                          eth_addr_t *src_addr);
int avb_process_aecp_cmd_set_clock_source(avb_state_s *state,
                                          aecp_message_u *msg,
                                          eth_addr_t *src_addr);
int avb_process_aecp_cmd_get_max_transit_time(avb_state_s *state,
                                              aecp_message_u *msg,
                                              eth_addr_t *src_addr);
int avb_process_aecp_cmd_get_stream_info(avb_state_s *state,
                                         aecp_message_u *msg,
                                         eth_addr_t *src_addr);
int avb_process_aecp_cmd_get_avb_info(avb_state_s *state, aecp_message_u *msg,
                                      eth_addr_t *src_addr);
int avb_process_aecp_cmd_get_as_path(avb_state_s *state, aecp_message_u *msg,
                                     eth_addr_t *src_addr);
int avb_process_aecp_cmd_get_counters(avb_state_s *state, aecp_message_u *msg,
                                      eth_addr_t *src_addr);
int avb_process_aecp_cmd_mvu_get_milan_info(avb_state_s *state,
                                            aecp_message_u *msg,
                                            eth_addr_t *src_addr);
int avb_process_aecp_cmd_mvu_get_system_unique_id(avb_state_s *state,
                                                  aecp_message_u *msg,
                                                  eth_addr_t *src_addr);
int avb_process_aecp_cmd_mvu_bind_stream(avb_state_s *state,
                                         aecp_message_u *msg,
                                         eth_addr_t *src_addr, bool unbind);
int avb_process_aecp_cmd_mvu_get_media_clock_ref_info(avb_state_s *state,
                                                      aecp_message_u *msg,
                                                      eth_addr_t *src_addr);
int avb_process_aecp_cmd_mvu_get_stream_input_info_ex(avb_state_s *state,
                                                      aecp_message_u *msg,
                                                      eth_addr_t *src_addr);
int avb_process_aecp_rsp_register_unsol_notif(avb_state_s *state,
                                              aecp_message_u *msg);
int avb_process_aecp_rsp_deregister_unsol_notif(avb_state_s *state,
                                                aecp_message_u *msg);
int avb_process_aecp_rsp_entity_available(avb_state_s *state,
                                          aecp_message_u *msg);
int avb_process_aecp_rsp_controller_available(avb_state_s *state,
                                              aecp_message_u *msg);
int avb_process_aecp_rsp_get_stream_info(avb_state_s *state,
                                         aecp_message_u *msg);
int avb_process_aecp_rsp_get_counters(avb_state_s *state, aecp_message_u *msg);
int avb_process_aecp_cmd_set_control(avb_state_s *state, aecp_message_u *msg,
                                     eth_addr_t *src_addr);
int avb_process_aecp_cmd_get_control(avb_state_s *state, aecp_message_u *msg,
                                     eth_addr_t *src_addr);
int avb_process_aecp_addr_access(avb_state_s *state, aecp_message_u *msg,
                                 eth_addr_t *src_addr);
int avb_process_aecp_cmd_set_name(avb_state_s *state, aecp_message_u *msg,
                                  eth_addr_t *src_addr);
int avb_process_aecp_cmd_get_name(avb_state_s *state, aecp_message_u *msg,
                                  eth_addr_t *src_addr);

/* Identify tone */
void avb_identify_tone(avb_state_s *state, uint32_t duration_ms);

/* ACMP processing functions */
int avb_process_acmp_connect_rx_command(avb_state_s *state, acmp_message_s *msg,
                                        bool disconnect, bool mvu_bind);
int avb_process_acmp_connect_tx_command(avb_state_s *state, acmp_message_s *msg,
                                        bool disconnect);
int avb_process_acmp_connect_tx_response(avb_state_s *state,
                                         acmp_message_s *msg, bool disconnect);
int avb_process_acmp_get_rx_state_command(avb_state_s *state,
                                          acmp_message_s *msg, bool rx);
int avb_process_acmp_get_tx_state_command(avb_state_s *state,
                                          acmp_message_s *msg, bool tx);
int avb_process_acmp_get_tx_connection_command(avb_state_s *state,
                                               acmp_message_s *msg, bool tx);


/* ===== ATDECC helpers ===== */

int avb_find_entity_by_id(avb_state_s *state, unique_id_t *entity_id,
                          avb_entity_type_t entity_typ);
int avb_find_entity_by_addr(avb_state_s *state, eth_addr_t *entity_addr,
                            avb_entity_type_t entity_type);
const char *get_adp_message_type_name(adp_msg_type_t message_type);
const char *get_aecp_command_code_name(aecp_cmd_code_t command_code);
const char *get_acmp_message_type_name(acmp_msg_type_t message_type);
bool avb_acquired_or_locked_by_other(avb_state_s *state,
                                     unique_id_t *entity_id);
bool avb_listener_is_connected(avb_state_s *state, acmp_message_s *msg,
                               bool same_talker);
bool avb_valid_talker_listener_uid(avb_state_s *state, uint16_t uid,
                                   avb_entity_type_t entity_type);
int avb_add_inflight_command(avb_state_s *state, atdecc_command_u *command,
                             bool inbound);
int avb_find_inflight_command(avb_state_s *state, uint16_t seq_id,
                              bool inbound);
int avb_find_inflight_command_by_data(avb_state_s *state,
                                      atdecc_command_u *data, bool inbound);
void avb_remove_inflight_command(avb_state_s *state, uint16_t seq_id,
                                 bool inbound);
acmp_status_t avb_connect_listener(avb_state_s *state,
                                   acmp_message_s *response);
void avb_periodic_fast_connect(avb_state_s *state);
acmp_status_t avb_disconnect_listener(avb_state_s *state,
                                      acmp_message_s *response);
int avb_get_acmp_timeout_ms(acmp_msg_type_t msg_type);
void avb_process_inflight_timeouts(avb_state_s *state);

#endif /* ESP_AVB_ATDECC_H_ */
