/*
 * Copyright (c) 2006-2018, RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author       Notes
 * 2019-02-20     zyh          first version
 */

#ifndef __WLAN_PORT_H__
#define __WLAN_PORT_H__

#define WIFI_EVENT_LINK_UP      (0x00)
#define WIFI_EVENT_DISASSOC_IND (0x01)

extern void scan_results_handler(rw007_ap_info * record);
extern void join_events_handler(rt_uint32_t event_type);
extern int rt_hw_wlan_init(void);
#endif
