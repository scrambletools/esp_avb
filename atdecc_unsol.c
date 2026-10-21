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

static uint8_t s_frame[AVB_MAX_MSG_LEN];

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

void avb_unsol_tick(avb_state_s *state) {
  int64_t now_us = esp_timer_get_time();
  avb_unsol_registrations_tick(state, now_us);
  avb_unsol_lock_tick(state, now_us);
}
