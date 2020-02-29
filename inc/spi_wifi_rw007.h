/*
 * COPYRIGHT (C) 2018, Real-Thread Information Technology Ltd
 * 
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author       Notes
 * 2014-07-31     aozima       the first version
 * 2014-09-18     aozima       update command & response.
 * 2020-02-28     shaoguoji    add spi transfer retry 
 */

#ifndef SPI_WIFI_H_INCLUDED
#define SPI_WIFI_H_INCLUDED

#include <stdint.h>
#include <rtdevice.h>

/* little-endian */
struct spi_cmd_request
{
    uint32_t flag;
    uint32_t M2S_len; /* master to slave data len. */
    uint32_t magic1;
    uint32_t magic2;
};

#ifndef RW007_SPI_MAX_HZ
#define RW007_SPI_MAX_HZ    30000000
#endif

#define SPI_MAX_RETRY_COUNT    5

#define CMD_MAGIC1 (0x67452301)
#define CMD_MAGIC2 (0xEFCDAB89)

#define CMD_FLAG_MRDY (0x01)

/* little-endian */
struct spi_response
{
    uint32_t flag;
    uint32_t S2M_len; /* slave to master data len. */
    uint32_t magic1;
    uint32_t magic2;
};

#define RESP_MAGIC1 (0x98BADCFE)
#define RESP_MAGIC2 (0x10325476)

#define RESP_FLAG_SRDY (0x01)

/* spi buffer configure. */
#define SPI_MAX_DATA_LEN 1520
#define SPI_TX_POOL_SIZE 4
#define SPI_RX_POOL_SIZE 4

typedef enum
{
    DATA_TYPE_STA_ETH_DATA = 0,
    DATA_TYPE_AP_ETH_DATA,
    DATA_TYPE_PROMISC_ETH_DATA,
    DATA_TYPE_CMD,
    DATA_TYPE_RESP,
    DATA_TYPE_CB,
} app_data_type_t;

struct spi_data_packet
{
    uint32_t data_len;  /* length for buffer */
    uint32_t data_type; /* app_data_type_t */
    char buffer[SPI_MAX_DATA_LEN];
};

typedef struct rw007_ap_info_value
{
    struct rt_wlan_info info;
    char passwd[RT_WLAN_PASSWORD_MAX_LENGTH];
} * rw007_ap_info_value_t;

/* littel endian */
typedef struct rw007_cmd
{
    uint32_t cmd;
    uint32_t len;

    /* command parameter */
    union
    {
        uint32_t int_value;
        uint8_t mac_value[8]; /* padding 2bytes */
        struct rw007_ap_info_value ap_info_value;
        char string_value[UINT16_MAX];
    } value;
} * rw007_cmd_t;

struct rw007_resp
{
    uint32_t cmd;
    uint32_t len;

    int32_t result; /* result of CMD. */

    /* response value */
    union
    {
        uint32_t int_value;
        uint8_t mac_value[8]; /* padding 2bytes */
        struct rw007_ap_info_value ap_info_value;
        char string_value[UINT16_MAX];
    } value;
};

/* tools */
#define node_entry(node, type, member) ((type *)((char *)(node) - (unsigned long)(&((type *)0)->member)))
#define member_offset(type, member) ((unsigned long)(&((type *)0)->member))

#define MAX_SPI_PACKET_SIZE (member_offset(struct spi_data_packet, buffer) + SPI_MAX_DATA_LEN)

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
    /* Device handle for spi device */
    struct rt_spi_device *spi_device;

    /* Tx mempool and mailbox */
    struct rt_mempool spi_tx_mp;
    ALIGN(4)
    rt_uint8_t spi_tx_mempool[(sizeof(struct spi_data_packet) + 4) * SPI_TX_POOL_SIZE];
    struct rt_mailbox spi_tx_mb;
    int spi_tx_mb_pool[SPI_TX_POOL_SIZE + 1];
    
    /* Rx mempool and mailbox */
    struct rt_mempool spi_rx_mp;
    ALIGN(4)
    rt_uint8_t spi_rx_mempool[(sizeof(struct spi_data_packet) + 4) * SPI_RX_POOL_SIZE];
    struct rt_mailbox spi_rx_mb;
    int spi_rx_mb_pool[SPI_RX_POOL_SIZE + 1];

    /* response event */
    rt_event_t rw007_cmd_event;
    /* response data */
    struct rw007_resp * resp[RW00x_CMD_MAX_NUM];
};

#define RW00x_CMD_RESP_EVENT(n)     (0x01UL << n)

struct rw007_wifi
{
    /* inherit from ethernet device */
    struct rt_wlan_device *wlan;
    /* spi transfer layer handle */
    struct rw007_spi * hspi;
};

/* porting */
extern void spi_wifi_hw_init(void);
extern void spi_wifi_int_cmd(rt_bool_t cmd);
extern rt_bool_t spi_wifi_is_busy(void);
/* end porting */

/* api exclude in wlan framework */
extern rt_err_t rw007_sn_get(char sn[24]);
extern rt_err_t rw007_version_get(char version[16]);
/* end api exclude in wlan framework */

extern rt_err_t rt_hw_wifi_init(const char *spi_device_name);

#endif /* SPI_WIFI_H_INCLUDED */
