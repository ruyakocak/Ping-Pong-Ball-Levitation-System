/*
 * Control-board configuration (Section 2.3: every program reads its ports from a
 * configuration file). Override any value with a build flag (-D ...) or in
 * include/wifi_secrets.h, which is not in git:
 *
 *   #define M3_WIFI_SSID "rig-hotspot"
 *   #define M3_WIFI_PASS "..."
 *   #define M3_GATEWAY_IP "192.168.137.1"
 */
#pragma once

#if __has_include("wifi_secrets.h")
#include "wifi_secrets.h"
#endif

#ifndef M3_WIFI_SSID
#define M3_WIFI_SSID "ultrasonik"
#endif
#ifndef M3_WIFI_PASS
#define M3_WIFI_PASS ""
#endif

#ifndef M3_UDP_PORT
#define M3_UDP_PORT 5005 /* control board: command, target_points in */
#endif

/* status goes to the gateway (M5) and the trajectory program (M4). Broadcast by
 * default, so the board needs no address until the rig network is fixed. */
#ifndef M3_GATEWAY_IP
#define M3_GATEWAY_IP "255.255.255.255"
#endif
#ifndef M3_GATEWAY_PORT
#define M3_GATEWAY_PORT 5010
#endif
#ifndef M3_M4_IP
#define M3_M4_IP "255.255.255.255"
#endif
#ifndef M3_M4_PORT
#define M3_M4_PORT 5006
#endif

#ifndef M3_STATUS_PERIOD_MS
#define M3_STATUS_PERIOD_MS 33 /* 30 Hz */
#endif
