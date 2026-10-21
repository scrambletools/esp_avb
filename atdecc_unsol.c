/*
 * Copyright 2026 Scramble Tools
 * License: MIT
 *
 * ESP_AVB Component
 *
 * This file keeps the controllers registered for unsolicited notifications
 * and sends the notifications to them (IEEE 1722.1-2021 §7.5.2, Milan v1.3
 * §5.3.4.2 and §5.4.5). Everything runs on the AVB task.
 */

#include "avb.h"
#include "esp_random.h"
#include "esp_timer.h"

/* Milan §5.4.5.4: a registered controller that sent no command for a
 * random 30 to 60 s is asked CONTROLLER_AVAILABLE, with one retry. */
#define UNSOL_CHECK_MIN_US (30 * 1000000LL)
#define UNSOL_CHECK_SPAN_US (30 * 1000000U)
#define UNSOL_CHECK_SENDS 2
/* Reply wait per send; Wi-Fi gets longer for its airtime. */
#define UNSOL_REPLY_WIRED_US 250000
#define UNSOL_REPLY_WIFI_US 1000000
/* IEEE 1722.1-2021 §7.4.37.2: a TIME_LIMITED registration lapses 300 s
 * after the controller last registered. */
#define UNSOL_TIME_LIMIT_US (300 * 1000000LL)
/* State a controller would read is rebuilt every 100 ms and sent when it
 * changed. GET_COUNTERS goes at most once a second per descriptor (Milan
 * Table 5.20), every 10 s over Wi-Fi for its airtime. */
#define UNSOL_ASYNC_PERIOD_US 100000
#define UNSOL_COUNTERS_WIRED_US 1000000
#define UNSOL_COUNTERS_WIFI_US 10000000
/* Propagation delay jitters by a few ns; a smaller step is no change */
#define UNSOL_PDELAY_STEP_NS 100

static uint8_t s_frame[AVB_MAX_MSG_LEN];

/* The response last sent for one asynchronous notification */
typedef struct {
  bool valid;
  uint16_t msg_len;
  union {
    aecp_get_stream_info_rsp_s stream_info;
    aecp_get_avb_info_rsp_s avb_info;
    aecp_get_as_path_rsp_s as_path;
    aecp_get_counters_rsp_s counters;
    aecp_mvu_get_stream_input_info_ex_rsp_s stream_input_info_ex;
  } rsp;
} avb_unsol_sent_s;

static avb_unsol_sent_s s_stream_info_in[AVB_MAX_NUM_INPUT_STREAMS];
static avb_unsol_sent_s s_stream_info_out[AVB_MAX_NUM_OUTPUT_STREAMS];
static avb_unsol_sent_s s_stream_input_info_ex[AVB_MAX_NUM_INPUT_STREAMS];
static avb_unsol_sent_s s_avb_info;
static avb_unsol_sent_s s_as_path;
static avb_unsol_sent_s s_counters_in[AVB_MAX_NUM_INPUT_STREAMS];
static avb_unsol_sent_s s_counters_out[AVB_MAX_NUM_OUTPUT_STREAMS];
static int64_t s_async_due_us;
static int64_t s_counters_due_us;

static int64_t avb_unsol_check_delay_us(void) {
  return UNSOL_CHECK_MIN_US + (int64_t)(esp_random() % UNSOL_CHECK_SPAN_US);
}

static int avb_unsol_find(avb_state_s *state, const uint8_t *controller_id,
                          const uint8_t *controller_addr) {
  for (size_t i = 0; i < state->num_unsol_controllers; i++) {
    avb_unsol_controller_s *entry = &state->unsol_controllers[i];
    if (memcmp(entry->entity_id, controller_id, UNIQUE_ID_LEN) == 0 &&
        memcmp(entry->mac_addr, controller_addr, ETH_ADDR_LEN) == 0)
      return (int)i;
  }
  return NOT_FOUND;
}

static void avb_unsol_remove(avb_state_s *state, size_t index) {
  memmove(&state->unsol_controllers[index],
          &state->unsol_controllers[index + 1],
          (state->num_unsol_controllers - 1 - index) *
              sizeof(avb_unsol_controller_s));
  state->num_unsol_controllers--;
}

static void avb_unsol_log(const char *event, const uint8_t *controller_id) {
  char id[UNIQUE_ID_LEN * 3 + 1];
  octets_to_hex_string(controller_id, UNIQUE_ID_LEN, id, ':');
  avbinfo("Unsolicited notifications: controller %s %s", id, event);
}

/* Send one notification to one controller: its entity ID and its next
 * sequence ID, counted whether or not the frame leaves (Milan §5.4.5.1). */
static void avb_unsol_send_to(avb_state_s *state, avb_unsol_controller_s *entry,
                              const void *rsp, uint16_t msg_len,
                              avb_unsol_kind_t kind) {
  if (msg_len > sizeof(s_frame))
    return;
  memcpy(s_frame, rsp, msg_len);
  aecp_common_s *common = (aecp_common_s *)s_frame;
  memcpy(common->controller_entity_id, entry->entity_id, UNIQUE_ID_LEN);
  uint16_t *counter = kind == avb_unsol_kind_aem   ? &entry->aem_seq_id
                      : kind == avb_unsol_kind_mvu ? &entry->mvu_seq_id
                                                   : &entry->vu_seq_id;
  uint16_t seq_id = (*counter)++;
  int_to_octets(&seq_id, common->seq_id, 2);
  if (kind == avb_unsol_kind_aem)
    ((aecp_aem_basic_s *)s_frame)->aem.unsolicited = 1;
  else if (kind == avb_unsol_kind_mvu)
    ((aecp_mvu_common_s *)s_frame)->u = 1;
  struct timespec ts;
  if (avb_net_send_to(state, ethertype_avtp, s_frame, msg_len, &ts,
                      &entry->mac_addr) < 0)
    avbwarn("send unsolicited notification failed: %d", errno);
}

void avb_unsol_notify(avb_state_s *state, const void *rsp, uint16_t msg_len,
                      avb_unsol_kind_t kind, const uint8_t *requester_id,
                      const eth_addr_t *requester_addr) {
  for (size_t i = 0; i < state->num_unsol_controllers; i++) {
    avb_unsol_controller_s *entry = &state->unsol_controllers[i];
    if (requester_id &&
        memcmp(entry->entity_id, requester_id, UNIQUE_ID_LEN) == 0 &&
        (!requester_addr ||
         memcmp(entry->mac_addr, requester_addr, ETH_ADDR_LEN) == 0))
      continue;
    avb_unsol_send_to(state, entry, rsp, msg_len, kind);
  }
}

/* The unsolicited DEREGISTER a removed controller gets, to it only
 * (Milan Table 5.20); a base AEM PDU (IEEE 1722.1-2021 §7.4.38). */
static void avb_unsol_send_deregister(avb_state_s *state,
                                      avb_unsol_controller_s *entry) {
  aecp_aem_basic_s rsp;
  memset(&rsp, 0, sizeof(rsp));
  rsp.common.header.subtype = avtp_subtype_aecp;
  rsp.common.header.msg_type = aecp_msg_type_aem_response;
  rsp.common.header.status_valtime = aecp_status_success;
  uint16_t cdl = sizeof(rsp) - AVTP_CDL_PREAMBLE_LEN;
  rsp.common.header.control_data_len_h = (cdl >> 8) & 0x07;
  rsp.common.header.control_data_len = cdl & 0xFF;
  memcpy(rsp.common.target_entity_id, state->own_entity.summary.entity_id,
         UNIQUE_ID_LEN);
  rsp.aem.command_type = aecp_cmd_code_deregister_unsol_notif;
  avb_unsol_send_to(state, entry, &rsp, sizeof(rsp), avb_unsol_kind_aem);
}

/* CONTROLLER_AVAILABLE to a registered controller; a retry reuses the
 * sequence ID of the first send. */
static void avb_unsol_send_check(avb_state_s *state,
                                 avb_unsol_controller_s *entry,
                                 int64_t now_us) {
  aecp_aem_basic_s cmd;
  memset(&cmd, 0, sizeof(cmd));
  cmd.common.header.subtype = avtp_subtype_aecp;
  cmd.common.header.msg_type = aecp_msg_type_aem_command;
  uint16_t cdl = sizeof(cmd) - AVTP_CDL_PREAMBLE_LEN;
  cmd.common.header.control_data_len_h = (cdl >> 8) & 0x07;
  cmd.common.header.control_data_len = cdl & 0xFF;
  memcpy(cmd.common.target_entity_id, entry->entity_id, UNIQUE_ID_LEN);
  memcpy(cmd.common.controller_entity_id, state->own_entity.summary.entity_id,
         UNIQUE_ID_LEN);
  if (entry->checks_sent == 0)
    entry->check_seq_id = state->aecp_seq_id++;
  int_to_octets(&entry->check_seq_id, cmd.common.seq_id, 2);
  cmd.aem.command_type = aecp_cmd_code_controller_available;
  struct timespec ts;
  if (avb_net_send_to(state, ethertype_avtp, &cmd, sizeof(cmd), &ts,
                      &entry->mac_addr) < 0)
    avbwarn("send CONTROLLER_AVAILABLE failed: %d", errno);
  entry->checks_sent++;
  bool wifi = state->port[0].medium == avb_port_medium_wifi_ftm;
  entry->reply_due_us =
      now_us + (wifi ? UNSOL_REPLY_WIFI_US : UNSOL_REPLY_WIRED_US);
}

static void avb_unsol_rearm(avb_unsol_controller_s *entry, int64_t now_us) {
  entry->check_due_us = now_us + avb_unsol_check_delay_us();
  entry->reply_due_us = 0;
  entry->checks_sent = 0;
}

/* Registrations are kept per (entity ID, MAC address); registering again
 * refreshes the entry and keeps its sequence IDs (Milan §5.4.2.21). */
aecp_status_t avb_unsol_register(avb_state_s *state, unique_id_t *controller_id,
                                 eth_addr_t *controller_addr,
                                 bool time_limited) {
  int64_t now_us = esp_timer_get_time();
  int index = avb_unsol_find(state, *controller_id, *controller_addr);
  if (index == NOT_FOUND) {
    if (state->num_unsol_controllers >= AVB_MAX_UNSOL_CONTROLLERS) {
      /* Check every registered controller now, so one that departed
       * frees its entry before this controller retries. */
      for (size_t i = 0; i < state->num_unsol_controllers; i++) {
        if (state->unsol_controllers[i].reply_due_us == 0)
          state->unsol_controllers[i].check_due_us = now_us;
      }
      avb_unsol_log("refused, the list is full", *controller_id);
      return aecp_status_no_resources;
    }
    index = (int)state->num_unsol_controllers++;
    avb_unsol_controller_s *entry = &state->unsol_controllers[index];
    memset(entry, 0, sizeof(*entry));
    memcpy(entry->entity_id, controller_id, UNIQUE_ID_LEN);
    memcpy(entry->mac_addr, controller_addr, ETH_ADDR_LEN);
    avb_unsol_log("registered", *controller_id);
  }
  avb_unsol_controller_s *entry = &state->unsol_controllers[index];
  entry->time_limited = time_limited;
  entry->expires_us = now_us + UNSOL_TIME_LIMIT_US;
  avb_unsol_rearm(entry, now_us);
  return aecp_status_success;
}

void avb_unsol_deregister(avb_state_s *state, unique_id_t *controller_id,
                          eth_addr_t *controller_addr) {
  int index = avb_unsol_find(state, *controller_id, *controller_addr);
  if (index == NOT_FOUND)
    return;
  avb_unsol_remove(state, (size_t)index);
  avb_unsol_log("deregistered", *controller_id);
}

/* Any valid command from a registered controller restarts its check
 * timer (Milan §5.4.5.4). */
void avb_unsol_heard(avb_state_s *state, unique_id_t *controller_id,
                     eth_addr_t *controller_addr) {
  int index = avb_unsol_find(state, *controller_id, *controller_addr);
  if (index != NOT_FOUND)
    avb_unsol_rearm(&state->unsol_controllers[index], esp_timer_get_time());
}

/* A CONTROLLER_AVAILABLE response, whatever its status, keeps the
 * registration. */
void avb_unsol_controller_replied(avb_state_s *state, aecp_message_u *msg) {
  uint16_t seq_id = (uint16_t)octets_to_uint(msg->common.seq_id, 2);
  for (size_t i = 0; i < state->num_unsol_controllers; i++) {
    avb_unsol_controller_s *entry = &state->unsol_controllers[i];
    if (entry->reply_due_us != 0 && entry->check_seq_id == seq_id &&
        memcmp(entry->entity_id, msg->common.target_entity_id,
               UNIQUE_ID_LEN) == 0) {
      avb_unsol_rearm(entry, esp_timer_get_time());
      return;
    }
  }
}

/* Lapsed time-limited registrations, and under Milan the departed
 * controllers, are removed and told so. */
static void avb_unsol_registrations_tick(avb_state_s *state, int64_t now_us) {
  for (size_t i = 0; i < state->num_unsol_controllers;) {
    avb_unsol_controller_s *entry = &state->unsol_controllers[i];
    bool lapsed = entry->time_limited && now_us >= entry->expires_us;
    bool departed = false;
    if (!lapsed && state->config.milan_compliant) {
      if (entry->reply_due_us != 0) {
        if (now_us >= entry->reply_due_us) {
          if (entry->checks_sent < UNSOL_CHECK_SENDS)
            avb_unsol_send_check(state, entry, now_us);
          else
            departed = true;
        }
      } else if (now_us >= entry->check_due_us) {
        avb_unsol_send_check(state, entry, now_us);
      }
    }
    if (lapsed || departed) {
      avb_unsol_log(lapsed ? "did not re-register, deregistered"
                           : "did not answer CONTROLLER_AVAILABLE, "
                             "deregistered",
                    entry->entity_id);
      avb_unsol_send_deregister(state, entry);
      avb_unsol_remove(state, i);
      continue;
    }
    i++;
  }
}

/* An expired lock is released and every registered controller is told,
 * the one that held it only if it registered (Milan v1.3 §5.4.2.2). */
static void avb_unsol_lock_tick(avb_state_s *state, int64_t now_us) {
  if (!state->locked || now_us < state->lock_expires_us)
    return;
  state->locked = false;
  memset(state->locked_by, 0, UNIQUE_ID_LEN);
  avbinfo("Entity lock expired");
  aecp_lock_entity_s rsp;
  memset(&rsp, 0, sizeof(rsp));
  rsp.common.header.subtype = avtp_subtype_aecp;
  rsp.common.header.msg_type = aecp_msg_type_aem_response;
  rsp.common.header.status_valtime = aecp_status_success;
  uint16_t cdl = sizeof(rsp) - AVTP_CDL_PREAMBLE_LEN;
  rsp.common.header.control_data_len_h = (cdl >> 8) & 0x07;
  rsp.common.header.control_data_len = cdl & 0xFF;
  memcpy(rsp.common.target_entity_id, state->own_entity.summary.entity_id,
         UNIQUE_ID_LEN);
  rsp.aem.command_type = aecp_cmd_code_lock_entity;
  rsp.unlock = 1;
  avb_unsol_notify(state, &rsp, sizeof(rsp), avb_unsol_kind_aem, NULL, NULL);
}

/* Send a rebuilt response to every registered controller when it differs
 * from the copy last sent. The first build only becomes the copy, so a
 * controller registering later is not sent the state it reads anyway. */
static void avb_unsol_send_changed(avb_state_s *state, avb_unsol_sent_s *sent,
                                   const void *rsp, uint16_t msg_len,
                                   avb_unsol_kind_t kind) {
  if (msg_len == 0 || msg_len > sizeof(sent->rsp))
    return;
  if (sent->valid && sent->msg_len == msg_len &&
      memcmp(&sent->rsp, rsp, msg_len) == 0)
    return;
  bool first = !sent->valid;
  sent->valid = true;
  sent->msg_len = msg_len;
  memcpy(&sent->rsp, rsp, msg_len);
  if (!first)
    avb_unsol_notify(state, rsp, msg_len, kind, NULL, NULL);
}

/* A GET command as a controller would send it, for the shared builders */
static void avb_unsol_get_command(avb_state_s *state, aecp_aem_short_s *cmd,
                                  uint8_t command_type,
                                  uint16_t descriptor_type,
                                  uint16_t descriptor_index) {
  memset(cmd, 0, sizeof(*cmd));
  cmd->common.header.subtype = avtp_subtype_aecp;
  cmd->common.header.msg_type = aecp_msg_type_aem_command;
  memcpy(cmd->common.target_entity_id, state->own_entity.summary.entity_id,
         UNIQUE_ID_LEN);
  cmd->aem.command_type = command_type;
  int_to_octets(&descriptor_type, cmd->descriptor_type, 2);
  int_to_octets(&descriptor_index, cmd->descriptor_index, 2);
}

static void avb_unsol_stream_info_tick(avb_state_s *state) {
  aecp_get_stream_info_s cmd;
  aecp_get_stream_info_rsp_s rsp;
  for (uint16_t i = 0; i < state->num_input_streams; i++) {
    avb_unsol_get_command(state, &cmd, aecp_cmd_code_get_stream_info,
                          aem_desc_type_stream_input, i);
    uint16_t msg_len = avb_fill_rsp_get_stream_info(state, &cmd, &rsp);
    avb_unsol_send_changed(state, &s_stream_info_in[i], &rsp, msg_len,
                           avb_unsol_kind_aem);
  }
  for (uint16_t i = 0; i < state->num_output_streams; i++) {
    avb_unsol_get_command(state, &cmd, aecp_cmd_code_get_stream_info,
                          aem_desc_type_stream_output, i);
    uint16_t msg_len = avb_fill_rsp_get_stream_info(state, &cmd, &rsp);
    avb_unsol_send_changed(state, &s_stream_info_out[i], &rsp, msg_len,
                           avb_unsol_kind_aem);
  }
}

static void avb_unsol_avb_info_tick(avb_state_s *state) {
  aecp_get_avb_info_s cmd;
  aecp_get_avb_info_rsp_s rsp;
  avb_unsol_get_command(state, &cmd, aecp_cmd_code_get_avb_info,
                        aem_desc_type_avb_interface, 0);
  uint16_t msg_len = avb_fill_rsp_get_avb_info(state, &cmd, &rsp);
  if (s_avb_info.valid) {
    uint8_t *sent_delay = s_avb_info.rsp.avb_info.propagation_delay;
    uint32_t sent_ns = (uint32_t)octets_to_uint(sent_delay, 4);
    uint32_t now_ns = (uint32_t)octets_to_uint(rsp.propagation_delay, 4);
    uint32_t step_ns = now_ns > sent_ns ? now_ns - sent_ns : sent_ns - now_ns;
    if (step_ns < UNSOL_PDELAY_STEP_NS)
      memcpy(rsp.propagation_delay, sent_delay, 4);
  }
  avb_unsol_send_changed(state, &s_avb_info, &rsp, msg_len,
                         avb_unsol_kind_aem);

  aecp_get_as_path_s path_cmd;
  aecp_get_as_path_rsp_s path_rsp;
  memset(&path_cmd, 0, sizeof(path_cmd));
  memcpy(&path_cmd, &cmd, sizeof(aecp_common_s));
  path_cmd.aem.command_type = aecp_cmd_code_get_as_path;
  msg_len = avb_fill_rsp_get_as_path(state, &path_cmd, &path_rsp);
  avb_unsol_send_changed(state, &s_as_path, &path_rsp, msg_len,
                         avb_unsol_kind_aem);
}

/* Milan Table 5.21: the binding as GET_STREAM_INPUT_INFO_EX reports it */
static void avb_unsol_stream_input_info_ex_tick(avb_state_s *state) {
  aecp_mvu_get_stream_input_info_ex_s cmd;
  aecp_mvu_get_stream_input_info_ex_rsp_s rsp;
  uint8_t protocol_id[] = MVU_PROTOCOL_ID;
  for (uint16_t i = 0; i < state->num_input_streams; i++) {
    memset(&cmd, 0, sizeof(cmd));
    cmd.mvu.common.header.subtype = avtp_subtype_aecp;
    cmd.mvu.common.header.msg_type = aecp_msg_type_vendor_unique_command;
    memcpy(cmd.mvu.common.target_entity_id,
           state->own_entity.summary.entity_id, UNIQUE_ID_LEN);
    memcpy(cmd.mvu.protocol_id, protocol_id, sizeof(cmd.mvu.protocol_id));
    cmd.mvu.command_type = mvu_cmd_get_stream_input_info_ex;
    uint16_t descriptor_type = aem_desc_type_stream_input;
    int_to_octets(&descriptor_type, cmd.descriptor_type, 2);
    int_to_octets(&i, cmd.descriptor_index, 2);
    avb_fill_rsp_mvu_stream_input_info_ex(state, &cmd, &rsp);
    avb_unsol_send_changed(state, &s_stream_input_info_ex[i], &rsp,
                           sizeof(rsp), avb_unsol_kind_mvu);
  }
}

static void avb_unsol_counters_tick(avb_state_s *state) {
  aecp_get_counters_s cmd;
  aecp_get_counters_rsp_s rsp;
  for (uint16_t i = 0; i < state->num_input_streams; i++) {
    avb_unsol_get_command(state, &cmd, aecp_cmd_code_get_counters,
                          aem_desc_type_stream_input, i);
    uint16_t msg_len = avb_fill_rsp_get_counters(state, &cmd, &rsp);
    avb_unsol_send_changed(state, &s_counters_in[i], &rsp, msg_len,
                           avb_unsol_kind_aem);
  }
  for (uint16_t i = 0; i < state->num_output_streams; i++) {
    avb_unsol_get_command(state, &cmd, aecp_cmd_code_get_counters,
                          aem_desc_type_stream_output, i);
    uint16_t msg_len = avb_fill_rsp_get_counters(state, &cmd, &rsp);
    avb_unsol_send_changed(state, &s_counters_out[i], &rsp, msg_len,
                           avb_unsol_kind_aem);
  }
}

void avb_unsol_tick(avb_state_s *state) {
  int64_t now_us = esp_timer_get_time();
  avb_unsol_registrations_tick(state, now_us);
  avb_unsol_lock_tick(state, now_us);
  if (now_us >= s_async_due_us) {
    s_async_due_us = now_us + UNSOL_ASYNC_PERIOD_US;
    avb_unsol_stream_info_tick(state);
    avb_unsol_avb_info_tick(state);
    if (state->config.milan_compliant)
      avb_unsol_stream_input_info_ex_tick(state);
  }
  if (now_us >= s_counters_due_us) {
    bool wifi = state->port[0].medium == avb_port_medium_wifi_ftm;
    s_counters_due_us =
        now_us + (wifi ? UNSOL_COUNTERS_WIFI_US : UNSOL_COUNTERS_WIRED_US);
    avb_unsol_counters_tick(state);
  }
}
