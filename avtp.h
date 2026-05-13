/*
 * Copyright 2024-2026 Scramble Tools
 * License: MIT
 *
 * AVTP wire types and protocol message declarations.
 *
 * Wire-format structs follow IEEE 1722-2016. Multi-byte fields use
 * uint8_t[N] byte arrays (always big-endian on wire); bit-fields stay
 * within single octets for compiler-portable layout. The struct IS the
 * wire format — RX/TX use direct memcpy, no separate encode/decode
 * pass.
 *
 * MAAP (1722.1 multicast address acquisition) is included here because
 * it rides in the AVTP envelope (subtype 0xfe) and shares the subtype
 * space.
 *
 * The avtp_msgbuf_u union (which spans every AVTP-subtype payload
 * including the ATDECC ones — ADP/AECP/ACMP) lives in avb.h alongside
 * the ATDECC types until the atdecc.h extraction; avtp.h itself owns
 * only the 1722 / MAAP payloads.
 *
 * This header is included from avb.h after the prerequisite types
 * (eth_addr_t, unique_id_t, AVTP_STREAM_DATA_PER_MSG) are defined.
 * avb_state_s is forward-declared below; its full definition follows
 * in avb.h.
 */

#ifndef ESP_AVB_AVTP_H_
#define ESP_AVB_AVTP_H_

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

/* Forward decl — full definition in avb.h. */
typedef struct avb_state_s avb_state_s;

/* ===== AVTP / MAAP enums ===== */

/* AVTP subtypes and their values.
 * Includes ATDECC control subtypes (ADP/AECP/ACMP) and MAAP — they
 * all share the AVTP subtype space per IEEE 1722-2016 / 1722.1-2021. */
typedef enum {
  avtp_subtype_61883 = 0x00,
  avtp_subtype_aaf = 0x02,
  avtp_subtype_crf = 0x04,
  avtp_subtype_adp = 0xfa,
  avtp_subtype_aecp = 0xfb,
  avtp_subtype_acmp = 0xfc,
  avtp_subtype_maap = 0xfe
} avtp_subtype_t;

/* MAAP message types and their values */
typedef enum {
  maap_msg_type_probe = 0x01,
  maap_msg_type_defend = 0x02,
  maap_msg_type_announce = 0x03
} maap_msg_type_t;

/* 61883 formats and their values */
typedef enum {
  iec61883_format_bt_601 = 0x01,
  iec61883_format_audio = 0x10,
  iec61883_format_mpeg2_ts = 0x20,
  iec61883_format_bo_1294 = 0x21
} iec61883_format_t;

/* AAF format values in enumerated order */
typedef enum {
  aaf_format_user,        // user defined
  aaf_format_float_32bit, // 32-bit floating point PCM
  aaf_format_int_32bit,   // 32-bit integer PCM; default for talker
  aaf_format_int_24bit,   // 24-bit integer PCM
  aaf_format_int_16bit,   // 16-bit integer PCM
  aaf_format_aes_32bit    // 32-bit AES3
} aaf_format_t;

/* AAF PCM sample rates in enumerated order  */
typedef enum {
  aaf_pcm_sample_rate_user,
  aaf_pcm_sample_rate_8k,
  aaf_pcm_sample_rate_16k,
  aaf_pcm_sample_rate_32k,
  aaf_pcm_sample_rate_44_1k,
  aaf_pcm_sample_rate_48k,
  aaf_pcm_sample_rate_88_2k,
  aaf_pcm_sample_rate_96k,
  aaf_pcm_sample_rate_176_4k,
  aaf_pcm_sample_rate_192k,
  aaf_pcm_sample_rate_24k
} aaf_pcm_sample_rate_t;

/* 61883-6 CIP SFC sample rates in enumerated order */
typedef enum {
  cip_sfc_sample_rate_32k,
  cip_sfc_sample_rate_44_1k,
  cip_sfc_sample_rate_48k,
  cip_sfc_sample_rate_88_2k,
  cip_sfc_sample_rate_96k,
  cip_sfc_sample_rate_176_4k,
  cip_sfc_sample_rate_192k
} cip_sfc_sample_rate_t;

/* CIP SFC code → sample rate in Hz. Defined in msrp.c (the MSRP
 * TALKER advertise needs it for bandwidth calculation); the AVTP
 * talker also calls it. */
uint32_t cip_sfc_to_sample_rate(uint8_t sfc);

/* ===== AVTP wire types =====
 *
 * Per IEEE 1722-2016. All multi-byte fields are big-endian.
 */

/* IEC 61883-6 Message */
typedef struct {
  uint8_t subtype;                 // AVTP message subtype
  uint8_t timestamp_valid : 1;     // source timestamp valid
  uint8_t gateway_valid : 1;       // gateway valid
  uint8_t reserved : 1;            // reserved
  uint8_t media_clock_restart : 1; // media clock restart
  uint8_t version : 3;             // AVTP version
  uint8_t sv : 1;                  // stream id valid
  uint8_t seq_num;                 // sequence number
  uint8_t timestamp_uncertain : 1; // avtp timestamp uncertain
  uint8_t reserved2 : 7;           // reserved
  unique_id_t stream_id;           // stream ID
  uint8_t avtp_ts[4];              // AVTP timestamp
  uint8_t gateway_info[4];         // gateway info
  uint8_t stream_data_len[2];      // stream data length
  uint8_t channel : 6; // IEEE 1394 channel (0-30, 32-63, 31=native AVTP)
  uint8_t tag : 2;     // IEEE 1394 tag (00=no CIP, 01=CIP)
  uint8_t sy : 4;      // sy (for IIDC or DTCP)
  uint8_t tcode : 4;   // IEEE 1394 tcode (must be 1010 on transmit)
  uint8_t stream_data[AVTP_STREAM_DATA_PER_MSG]; // variable length (CIP header
                                                 // may be present)
} iec_61883_6_message_s;

/* AAF PCM Message */
typedef struct {
  uint8_t subtype;                 // AVTP message subtype
  uint8_t timestamp_valid : 1;     // source timestamp valid
  uint8_t reserved1 : 2;           // reserved
  uint8_t media_clock_restart : 1; // media clock restart
  uint8_t version : 3;             // AVTP version
  uint8_t sv : 1;                  // stream id valid
  uint8_t seq_num;                 // sequence number
  uint8_t timestamp_uncertain : 1; // avtp timestamp uncertain
  uint8_t reserved2 : 7;           // reserved
  unique_id_t stream_id;           // stream ID
  uint8_t avtp_ts[4];              // AVTP timestamp
  uint8_t format;                  // format
  uint8_t padding : 2;             // ignored part of channels per frame
  uint8_t reserved3 : 2;           // reserved
  uint8_t sample_rate : 4;         // nominal sample rate
  uint8_t chan_per_frame; // channels per frame; using 1 byte for convenience
  uint8_t bit_depth;      // cannot be larger than what is specified in format
  uint8_t stream_data_len[2];                    // stream data length
  uint8_t evt : 4;                               // upper-level event
  uint8_t sparse_ts : 1;                         // sparse timestamp
  uint8_t reserved4 : 3;                         // reserved
  uint8_t reserved5;                             // reserved
  uint8_t stream_data[AVTP_STREAM_DATA_PER_MSG]; // variable length
} aaf_pcm_message_s;

/* MAAP message */
typedef struct {
  uint8_t subtype;             // AVTP message subtype
  uint8_t msg_type : 4;        // MAAP message type
  uint8_t version : 3;         // AVTP version
  uint8_t sv : 1;              // always 0 for MAAP messages
  uint8_t padding : 3;         // ignored part of control data length
  uint8_t maap_version : 5;    // maap version
  uint8_t control_data_len;    // control data length (limited to 1 byte for
                               // convenience)
  unique_id_t stream_id;       // stream ID
  eth_addr_t req_start_addr;   // requested start address
  uint8_t req_count[2];        // requested count
  eth_addr_t confl_start_addr; // conflict start address
  uint8_t confl_count[2];      // conflict count
} maap_message_s;              // 28 bytes

/* MAAP state machine states */
typedef enum {
  maap_state_initial,
  maap_state_probe,
  maap_state_defend
} maap_state_t;

/* MAAP state per output stream */
typedef struct {
  maap_state_t state;
  eth_addr_t acquired_addr; // acquired multicast address
  uint8_t probe_count;      // remaining probes to send
  int64_t timer_expiry_us;  // next probe or announce time (esp_timer_get_time)
  bool acquired;            // true when address successfully acquired
} maap_stream_state_s;

/* ===== AVTP stream formats ===== */

/* IEC 61883-6 AM824 stream format
 * used in stream summary and stream descriptor
 */
typedef struct {
  uint8_t subtype : 7; // 0 for 61883
  uint8_t
      vendor_defined : 1; // 0 for AVTP standard, 1 for vendor or ATDECC defined
  uint8_t reserved1 : 1;  //
  uint8_t
      format : 6; // 61883 format ID; 0x10 for IEC 61883-6 (1722 Clause I.2.2.3)
  uint8_t sf : 1; // 1=61883, 0=IIDC
  uint8_t fdf_sfc : 3; // See Clause 9.1
  uint8_t
      fdf_evt : 5; // 00000=AM824, 00100=32-bit float, 00110= 32-bit fixed-point
  uint8_t
      dbs; // num of data blocks, same as sum of all label_ fields (Clause 9.2)
  uint8_t reserved2 : 4; //
  uint8_t sc : 1; // packetization clock and media clock are synchronous; should
                  // be 0 (async)
  uint8_t ut : 1; // can source or sink a stream with less than num of data
                  // blocks indicated in dbs
  uint8_t nb : 1; // supports non-blocking mode (Annex A)
  uint8_t b : 1;  // supports blocking mode (Annex A)
  uint8_t label_iec_60958_cnt; // count of IEC 60958 quadlets (see Clause 8.2.2)
  uint8_t label_mbla_cnt;      // count of multi-bit linear audio quadlets (see
                               // Clause 8.2.3)
  uint8_t label_smptecnt : 4;  // count of SMPTE quadlets (see Clause 8.2.6)
  uint8_t label_midi_cnt : 4;  // count of MIDI quadlets (see Clause 8.2.5)
} avtp_stream_format_am824_s;  // 8 bytes

/* AAF PCM stream format
 * used in stream summary and stream descriptor
 */
typedef struct {
  uint8_t subtype : 7; // 0x02 for AAF
  uint8_t
      vendor_defined : 1; // 0 for AVTP standard, 1 for vendor or ATDECC defined
  uint8_t sample_rate : 4; // nominal base freq; same as sample_rate in stream
                           // message
  uint8_t ut : 1; // capable of handling less than channels_per_frame amount
  uint8_t reserved1 : 3;
  uint8_t format; // the AAF format value; 0x02 for 32-bit integer PCM
  uint8_t
      bit_depth; // num bits in each sample; same as bit_depth in stream message
  uint8_t chan_per_frame_h;        // hight 8 bits of channels_per_frame
  uint8_t samples_per_frame_h : 6; // high 6 bits of samples_per_frame
  uint8_t chan_per_frame : 2; // num channels in each frame; same as in stream
                              // message
  uint8_t reserved2 : 4;
  uint8_t samples_per_frame : 4; // num samples per channel per frame
  uint8_t reserved3;
} avtp_stream_format_aaf_pcm_s; // 8 bytes

/* CRF stream format (IEEE 1722-2016 §10).
 * Bit layout matches the 8-byte wire encoding above. */
typedef struct {
  uint8_t subtype : 7; // 0x04 for CRF
  uint8_t vendor_defined : 1;
  uint8_t timestamp_interval_h : 4; // high 4 bits of timestamp_interval
  uint8_t crf_type : 4;             // 1 = AudioSample (Milan)
  uint8_t timestamp_interval_l;     // low 8 bits of timestamp_interval
  uint8_t timestamps_per_pdu;       // Milan uses 1
  uint8_t base_frequency_h : 5;     // high 5 bits of base_frequency
  uint8_t pull : 3;                 // 0 = 1.0x
  uint8_t base_frequency_mh;        // middle-high byte
  uint8_t base_frequency_ml;        // middle-low byte
  uint8_t base_frequency_l;         // low byte
} avtp_stream_format_crf_s;         // 8 bytes

/* AVTP Stream Format */
typedef union {
  uint8_t subtype : 7;
  uint8_t vendor_defined : 1;
  avtp_stream_format_am824_s am824;
  avtp_stream_format_aaf_pcm_s aaf_pcm;
  avtp_stream_format_crf_s crf;
} avtp_stream_format_s; // 64 bytes

/* ===== AVTP / MAAP send + process ===== */

int avb_send_aaf_pcm(avb_state_s *state);

void avb_maap_init(avb_state_s *state);
void avb_maap_tick(avb_state_s *state);
int avb_send_maap_msg(avb_state_s *state, maap_msg_type_t msg_type,
                      eth_addr_t *req_addr, uint16_t req_count,
                      eth_addr_t *confl_addr, uint16_t confl_count);

int avb_process_iec_61883(avb_state_s *state, iec_61883_6_message_s *msg);
int avb_process_aaf(avb_state_s *state, aaf_pcm_message_s *msg);
int avb_process_maap(avb_state_s *state, maap_message_s *msg);

#endif /* ESP_AVB_AVTP_H_ */
