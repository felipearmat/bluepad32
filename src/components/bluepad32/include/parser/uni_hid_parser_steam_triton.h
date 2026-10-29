// SPDX-License-Identifier: Apache-2.0
// Steam Controller 2026 (Triton) BLE — Bluepad32 / OGX-Mini

#ifndef UNI_HID_PARSER_STEAM_TRITON_H
#define UNI_HID_PARSER_STEAM_TRITON_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdbool.h>
#include <stdint.h>

struct uni_hid_device_s;

// IDs also used by the BLE advertisement parsing (uni_bt_le.c).
#define UNI_TRITON_VALVE_VID 0x28de   // Valve Corporation
#define UNI_TRITON_PID_USB 0x1302     // Steam Controller 2026 ("Triton") over USB
#define UNI_TRITON_PID_BLE 0x1303     // Steam Controller 2026 ("Triton") over BLE
#define UNI_TRITON_PID_PUCK 0x1304    // Steam Controller 2026 ("Triton") Wireless Puck
#define UNI_TRITON_PID_NEREID 0x1305  // Steam Controller 2026 ("Triton") Nereid

void uni_hid_parser_steam_triton_setup(struct uni_hid_device_s* d);
void uni_hid_parser_steam_triton_teardown(struct uni_hid_device_s* d);
void uni_hid_parser_steam_triton_init_report(struct uni_hid_device_s* d);
void uni_hid_parser_steam_triton_parse_input_report(struct uni_hid_device_s* d,
                                                    const uint8_t* report,
                                                    uint16_t len);
void uni_hid_parser_steam_triton_play_dual_rumble(struct uni_hid_device_s* d,
                                                  uint16_t start_delay_ms,
                                                  uint16_t duration_ms,
                                                  uint8_t weak_magnitude,
                                                  uint8_t strong_magnitude);

/** True for Valve Triton BLE / USB / puck PIDs (0x1302–0x1305). */
bool uni_hid_parser_steam_triton_is_device(const struct uni_hid_device_s* d);

#ifdef __cplusplus
}
#endif

#endif  // UNI_HID_PARSER_STEAM_TRITON_H
