/*
 * COPYRIGHT (C) 2018, Real-Thread Information Technology Ltd
 * 
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author       Notes
 * 2014-07-31     aozima       the first version
 * 2014-09-18     aozima       update command & response.
 */

#ifndef SPI_WIFI_H_INCLUDED
#define SPI_WIFI_H_INCLUDED

#include <stdint.h>
#include "lwipopts.h"
#include <rtdevice.h>

// little-endian
struct spi_cmd_request
{
    uint32_t flag;
    uint32_t M2S_len; // master to slave data len.
    uint32_t magic1;
    uint32_t magic2;
};

#define CMD_MAGIC1 (0x67452301)
#define CMD_MAGIC2 (0xEFCDAB89)

#define CMD_FLAG_MRDY (0x01)

// little-endian
struct spi_response
{
    uint32_t flag;
    uint32_t S2M_len; // slave to master data len.
    uint32_t magic1;
    uint32_t magic2;
};

#define RESP_FLAG_SRDY (0x01)
#define RESP_MAGIC1 (0x98BADCFE)
#define RESP_MAGIC2 (0x10325476)

/* spi slave configure. */
#define SPI_MAX_DATA_LEN 1520
#define SPI_TX_POOL_SIZE 4
#define SPI_RX_POOL_SIZE 4

typedef enum
{
    data_type_sta_eth_data = 0,
    data_type_ap_eth_data,
    data_type_promisc_data,
    data_type_cmd,
    data_type_resp,
    data_type_cb,
}
app_data_type_typedef;

struct spi_data_packet
{
    uint32_t data_len;
    uint32_t data_type;
    char buffer[SPI_MAX_DATA_LEN];
};

/********************************* RW007 **************************************/

/* option */
#define RW007_CMD_TIMEOUT (RT_TICK_PER_SECOND * 3)
#define SSID_NAME_LENGTH_MAX (32)
#define PASSWORD_LENGTH_MAX (64)

typedef enum
{
    MODE_STATION = 0,
    MODE_SOFTAP = 1,
} wifi_mode_t;

typedef struct rw00x_ap_info_value
{
    struct rt_wlan_info info;
    char passwd[PASSWORD_LENGTH_MAX];
} * rw00x_ap_info_value_t;

/* littel endian */
typedef struct rw00x_cmd
{
    uint32_t cmd;
    uint32_t len;

    /** command body */
    union
    {
        uint32_t int_value;
        uint8_t mac_value[8];/* padding 2bytes */
        struct rw00x_ap_info_value ap_info_value;
        char string_value[UINT16_MAX];
    } value;
} * rw00x_cmd_t;

struct rw00x_resp
{
    uint32_t cmd;
    uint32_t len;

    int32_t result; /* result of CMD. */

    /** resp Body */
    union
    {
        uint32_t int_value;
        uint8_t mac_value[8];/* padding 2bytes */
        struct rw00x_ap_info_value ap_info_value;
        char string_value[UINT16_MAX];
    } value;
};

/* tools */
#define node_entry(node, type, member) ((type *)((char *)(node) - (unsigned long)(&((type *)0)->member)))
#define member_offset(type, member) ((unsigned long)(&((type *)0)->member))

#define MAX_ADDR_LEN (6) 
#define MAX_SPI_PACKET_SIZE (member_offset(struct spi_data_packet, buffer) + SPI_MAX_DATA_LEN)
#define MAX_SPI_BUFFER_SIZE (sizeof(struct spi_response) + MAX_SPI_PACKET_SIZE)

typedef enum 
{
    RW00x_CMD_INIT = 0x00,
    RW00x_CMD_SET_MODE,
    RW00x_CMD_MAC_GET,
    RW00x_CMD_MAC_SET,
    RW00x_CMD_GET_SN,
    RW00x_CMD_GET_VSR,
    RW00x_CMD_SCAN,
    RW00x_CMD_JOIN,
    RW00x_CMD_SOFTAP,
    RW00x_CMD_DISCONNECT,
    RW00x_CMD_AP_STOP,
    RW00x_CMD_AP_DEAUTH,
    RW00x_CMD_SCAN_STOP,
    RW00x_CMD_GET_RSSI,
    RW00x_CMD_SET_PWR_SAVE,
    RW00x_CMD_GET_PWR_SAVE,
    RW00x_CMD_CFG_PROMISC,
    RW00x_CMD_CFG_FILTER,
    RW00x_CMD_SET_CHANNEL,
    RW00x_CMD_GET_CHANNEL,
    RW00x_CMD_SET_COUNTRY,
    RW00x_CMD_GET_COUNTRY,
    RW00x_CMD_AP_MAC_GET,
    RW00x_CMD_AP_MAC_SET,
    RW00x_CMD_MAX_NUM
}RW00x_CMD;

struct rw007_spi
{
    struct rt_spi_device *spi_device;

    struct rt_mempool spi_tx_mp;

    struct rt_mailbox spi_tx_mb;
    
    struct rt_mempool spi_rx_mp;

    struct rt_mailbox spi_rx_mb;

    int spi_tx_mb_pool[SPI_TX_POOL_SIZE + 1];
    
    int spi_rx_mb_pool[SPI_RX_POOL_SIZE + 1];

    rt_event_t rw007_cmd_event;

    ALIGN(4)
    rt_uint8_t spi_tx_mempool[(sizeof(struct spi_data_packet) + 4) * SPI_TX_POOL_SIZE];
    
    ALIGN(4)
    rt_uint8_t spi_rx_mempool[(sizeof(struct spi_data_packet) + 4) * SPI_RX_POOL_SIZE];
    
    
    struct rw00x_resp * resp[RW00x_CMD_MAX_NUM];
};

struct rw007_wifi
{
    /* inherit from ethernet device */
    struct rt_wlan_device *wlan;

    struct rw007_spi * hspi;
};


#define RW00x_CMD_RESP_EVENT(n)     (0x01UL << n)

/* porting */
extern void spi_wifi_hw_init(void);
extern void spi_wifi_int_cmd(rt_bool_t cmd);
extern rt_bool_t spi_wifi_is_busy(void);

extern rt_err_t rw007_sn_get(char sn[24]);
extern rt_err_t rw007_version_get(char version[16]);

extern rt_err_t rt_hw_wifi_init(const char *spi_device_name);

#endif // SPI_WIFI_H_INCLUDED
