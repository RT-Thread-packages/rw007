/*
 * COPYRIGHT (C) 2018, Real-Thread Information Technology Ltd
 * 
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author       Notes
 * 2014-07-31     aozima       the first version
 * 2014-09-18     aozima       update command & response.
 * 2017-07-28     armink       fix auto reconnect feature
 * 2018-12-24     zyh          porting rw007 from rw009
 * 2019-02-25     zyh          porting rw007 to wlan 
 */
#include <rtthread.h>
#include <string.h>

#ifndef RW007_LOG_LEVEL
#define RW007_LOG_LEVEL DBG_LOG
#endif

#define DBG_ENABLE
#define DBG_SECTION_NAME "[RW007]"
#define DBG_LEVEL RW007_LOG_LEVEL
#define DBG_COLOR
#include <rtdbg.h>

#include "spi_wifi_rw007.h"

static struct rw007_spi rw007_spi;
static struct rw007_wifi wifi_sta, wifi_ap;
static struct rt_event spi_wifi_data_event;

static rt_err_t spi_wifi_transfer(struct rw007_spi *dev)
{
    struct spi_cmd_request cmd;
    struct spi_response resp;

    rt_err_t result;
    const struct spi_data_packet *data_packet = RT_NULL;
    struct rt_spi_device *rt_spi_device = dev->spi_device;
    uint8_t * rx_buffer = rt_mp_alloc(&dev->spi_rx_mp, RT_WAITING_NO);
    /* Disable INT Pin interrupt */
    spi_wifi_int_cmd(0);

    while (spi_wifi_is_busy())
    {
        /* wait for idel */
    }

    /* Clear cmd */
    cmd.flag = 0;
    cmd.M2S_len = 0;

    /* Set magic word */
    cmd.magic1 = CMD_MAGIC1;
    cmd.magic2 = CMD_MAGIC2;

    /* Set master ready flag bit */
    if(rx_buffer)
    {
        cmd.flag |= CMD_FLAG_MRDY;
    }
    
    /* Try get data to send to rw007 */
    result = rt_mb_recv(&dev->spi_tx_mb,
                        (rt_ubase_t *)&data_packet,
                        0);
    /* Set length for master to slave when data ready*/
    if ((result == RT_EOK) && (data_packet != RT_NULL) && (data_packet->data_len > 0))
    {
        cmd.M2S_len = data_packet->data_len + member_offset(struct spi_data_packet, buffer);
    }

    /* Stage 1: Send command to rw007 */
    rt_spi_send(rt_spi_device, &cmd, sizeof(cmd));
    while (spi_wifi_is_busy())
    {
        /* wait for idel */
    }

    /* Stage 2: Receive response from rw007 and transmit data */
    {
        struct rt_spi_message message;
        uint32_t max_data_len = 0;

        /* Setup message */
        message.send_buf = RT_NULL;
        message.recv_buf = &resp;
        message.length = sizeof(resp);
        message.cs_take = 1;
        message.cs_release = 0;

        /* Start a SPI transmit */
        rt_spi_take_bus(rt_spi_device);

        /* Receive response from rw007 */
        rt_spi_device->bus->ops->xfer(rt_spi_device, &message);

        /* Check response's magic word */
        if ((resp.magic1 != RESP_MAGIC1) || (resp.magic2 != RESP_MAGIC2))
        {
            resp.S2M_len = 0;
            goto _bad_resp_magic;
        }

        /* Check rw007's data ready flag */
        if (resp.flag & RESP_FLAG_SRDY)
        {
            max_data_len = cmd.M2S_len;
        }

        if (resp.S2M_len)
        {
            if (resp.S2M_len > MAX_SPI_PACKET_SIZE)
            {
                /* Drop error data */
                resp.S2M_len = 0;
            }

            if (resp.S2M_len > max_data_len)
                max_data_len = resp.S2M_len;
        }
_bad_resp_magic:
        /* Setup message */
        if(!resp.S2M_len && rx_buffer)
        {
            rt_mp_free(rx_buffer);
            rx_buffer = RT_NULL;
        }
        message.send_buf = data_packet;
        message.recv_buf = rx_buffer;
        message.length = RT_ALIGN(max_data_len, 4);/* align clk to word */
        message.cs_take = 0;
        message.cs_release = 1;

        /* Transmit data */
        rt_spi_device->bus->ops->xfer(rt_spi_device, &message);

        /* End a SPI transmit */
        rt_spi_release_bus(rt_spi_device);

        /* Free send data space */
        if (data_packet)
        {
            rt_mp_free((void *)data_packet);
            data_packet = RT_NULL;
        }

        /* Parse recevied data */
        if(rx_buffer)
        {
            rt_mb_send(&dev->spi_rx_mb, (rt_ubase_t)rx_buffer);
        }
    }
    /* Enable INT Pin interrupt */
    spi_wifi_int_cmd(1);

    if ((cmd.M2S_len == 0) && (resp.S2M_len == 0))
    {
        return -RT_ERROR;
    }

    return RT_EOK;
}

static void wifi_data_process_thread_entry(void *parameter)
{
    const struct spi_data_packet *data_packet = RT_NULL;
    struct rw007_spi *dev = (struct rw007_spi *)parameter;
    while(1)
    {
        /* get the mempool memory for recv data package */
        if(rt_mb_recv(&dev->spi_rx_mb, (rt_ubase_t *)&data_packet, RT_WAITING_FOREVER) == RT_EOK)
        {
            if (data_packet->data_type == DATA_TYPE_STA_ETH_DATA)
            {
                /* Ethernet package from station device */
                rt_wlan_dev_report_data(wifi_sta.wlan, (void *)data_packet->buffer, data_packet->data_len);
            }
            else if (data_packet->data_type == DATA_TYPE_AP_ETH_DATA)
            {
                /* Ethernet package from ap device */
                rt_wlan_dev_report_data(wifi_ap.wlan, (void *)data_packet->buffer, data_packet->data_len);
            }
            else if (data_packet->data_type == DATA_TYPE_PROMISC_ETH_DATA)
            {
                /* air wifi package from promisc */
                rt_wlan_dev_promisc_handler(wifi_sta.wlan, (void *)data_packet->buffer, data_packet->data_len);
            }
            /* event callback */
            else if(data_packet->data_type == DATA_TYPE_CB)
            {
                struct rw007_resp * resp = (struct rw007_resp *)data_packet->buffer;
                if(resp->cmd == RT_WLAN_DEV_EVT_SCAN_REPORT)
                {
                    /* parse scan report event data */
                    struct rt_wlan_buff buff;
                    struct rt_wlan_info * wlan_info;
                    wlan_info = (struct rt_wlan_info *)&resp->value;
                    buff.data = wlan_info;
                    buff.len = sizeof(struct rt_wlan_info);

                    /* indicate scan report event */
                    rt_wlan_dev_indicate_event_handle(wifi_sta.wlan, RT_WLAN_DEV_EVT_SCAN_REPORT, &buff);
                }
                else
                {
                    if(resp->cmd == RT_WLAN_DEV_EVT_AP_START || resp->cmd == RT_WLAN_DEV_EVT_AP_STOP || 
                       resp->cmd == RT_WLAN_DEV_EVT_AP_ASSOCIATED || resp->cmd == RT_WLAN_DEV_EVT_AP_DISASSOCIATED)
                    {
                        /* indicate ap device event */
                        rt_wlan_dev_indicate_event_handle(wifi_ap.wlan, (rt_wlan_dev_event_t)resp->cmd, RT_NULL);
                    }
                    else
                    {
                        /* indicate sta device event */
                        rt_wlan_dev_indicate_event_handle(wifi_sta.wlan, (rt_wlan_dev_event_t)resp->cmd, RT_NULL);
                    }
                }
            }
            else if (data_packet->data_type == DATA_TYPE_RESP)
            {
                /* parse cmd's response */
                struct rw007_resp * resp = (struct rw007_resp *)data_packet->buffer;
                if(resp->cmd < RW00x_CMD_MAX_NUM)
                {
                    if(dev->resp[resp->cmd])
                    {
                        rt_free(dev->resp[resp->cmd]);
                    }

                    /* stash response result */
                    dev->resp[resp->cmd] = rt_malloc(MAX_SPI_PACKET_SIZE);
                    if(dev->resp[resp->cmd])
                    {
                        rt_memcpy(dev->resp[resp->cmd], resp, MAX_SPI_PACKET_SIZE);

                        /* notify response arrived */
                        rt_event_send(dev->rw007_cmd_event, RW00x_CMD_RESP_EVENT(resp->cmd));
                    }
                }
            }
            /* free recv mempool memory */
            rt_mp_free((void *)data_packet);
        }
    }
}

static void spi_wifi_data_thread_entry(void *parameter)
{
    rt_uint32_t e;
    rt_err_t result;

    while (1)
    {
        /* receive first event */
        if (rt_event_recv(&spi_wifi_data_event,
                          1,
                          RT_EVENT_FLAG_AND | RT_EVENT_FLAG_CLEAR,
                          RT_WAITING_FOREVER,
                          &e) != RT_EOK)
        {
            continue;
        }

        /* transfer */
        result = spi_wifi_transfer(&rw007_spi);

        /* continue transfer once */
        if (result == RT_EOK)
        {
            rt_event_send(&spi_wifi_data_event, 1);
        }
    }
}

rt_inline struct rw007_wifi *wifi_get_dev_by_wlan(struct rt_wlan_device *wlan)
{
    if (wlan == wifi_sta.wlan)
    {
        return &wifi_sta;
    }
    if (wlan == wifi_ap.wlan)
    {
        return &wifi_ap;
    }
    return RT_NULL;
}

rt_inline void spi_send_cmd(struct rw007_spi * hspi, RW00x_CMD COMMAND, void * buffer, rt_uint32_t len)
{
    struct spi_data_packet * data_packet;
    struct rw007_cmd * cmd;

    data_packet = rt_mp_alloc(&hspi->spi_tx_mp, RT_WAITING_FOREVER);
    data_packet->data_type = DATA_TYPE_CMD;

    cmd = (struct rw007_cmd *)data_packet->buffer;
    cmd->cmd = COMMAND;
    cmd->len = len;
    if(cmd->len)
    {
        rt_memcpy(&cmd->value, buffer, cmd->len);
    }

    data_packet->data_len = member_offset(struct rw007_cmd, value) + cmd->len;

    rt_mb_send(&hspi->spi_tx_mb, (rt_uint32_t)data_packet);
    rt_event_send(&spi_wifi_data_event, 1);
}

rt_inline rt_err_t spi_set_data(struct rt_wlan_device *wlan, RW00x_CMD COMMAND, void * buffer, rt_uint32_t len)
{
    struct rw007_spi * hspi = wifi_get_dev_by_wlan(wlan)->hspi;
    rt_uint32_t result_event;
    rt_err_t result = RT_EOK;
    spi_send_cmd(hspi, COMMAND, buffer, len);
    if(rt_event_recv(hspi->rw007_cmd_event, 
                    RW00x_CMD_RESP_EVENT(COMMAND), 
                    RT_EVENT_FLAG_AND | RT_EVENT_FLAG_CLEAR, 
                    rt_tick_from_millisecond(10000),
                    &result_event) != RT_EOK)
    {
        return -RT_ETIMEOUT;
    }

    if(hspi->resp[COMMAND])
    {
        result = hspi->resp[COMMAND]->result;
        rt_free(hspi->resp[COMMAND]);
        hspi->resp[COMMAND] = RT_NULL;
        return result;
    }
    return RT_EOK;
}

rt_inline rt_err_t spi_get_data(struct rt_wlan_device *wlan, RW00x_CMD COMMAND, void * buffer, rt_uint32_t *len)
{
    struct rw007_spi * hspi = wifi_get_dev_by_wlan(wlan)->hspi;
    rt_uint32_t result_event;
    rt_err_t result = RT_EOK;
    spi_send_cmd(hspi, COMMAND, RT_NULL, 0);
    if(rt_event_recv(hspi->rw007_cmd_event, 
                    RW00x_CMD_RESP_EVENT(COMMAND), 
                    RT_EVENT_FLAG_AND | RT_EVENT_FLAG_CLEAR, 
                    rt_tick_from_millisecond(10000),
                    &result_event) != RT_EOK)
    {
        return -RT_ETIMEOUT;
    }

    if(hspi->resp[COMMAND])
    {
        *len = hspi->resp[COMMAND]->len;
        rt_memcpy(buffer, &hspi->resp[COMMAND]->value, hspi->resp[COMMAND]->len);
        result = hspi->resp[COMMAND]->result;
        rt_free(hspi->resp[COMMAND]);
        hspi->resp[COMMAND] = RT_NULL;
        return result;
    }
    return RT_EOK;
}

rt_err_t rw007_sn_get(char sn[24])
{
    rt_uint32_t size_of_data;
    return spi_get_data(wifi_sta.wlan, RW00x_CMD_GET_SN, sn, &size_of_data);
}

rt_err_t rw007_version_get(char version[16])
{
    rt_uint32_t size_of_data;
    return spi_get_data(wifi_sta.wlan, RW00x_CMD_GET_VSR, version, &size_of_data);
}

static rt_err_t wlan_init(struct rt_wlan_device *wlan)
{
    static int inited = 0;
    if(!inited)
    {
        inited = 1;
        return spi_set_data(wlan, RW00x_CMD_INIT, RT_NULL, 0);
    }
    return RT_EOK;
}

static rt_err_t wlan_mode(struct rt_wlan_device *wlan, rt_wlan_mode_t mode)
{
    return spi_set_data(wlan, RW00x_CMD_SET_MODE, &mode, sizeof(mode));
}

static rt_err_t wlan_scan(struct rt_wlan_device *wlan, struct rt_scan_info *scan_info)
{
    return spi_set_data(wlan, RW00x_CMD_SCAN, RT_NULL, 0);
}

static rt_err_t wlan_join(struct rt_wlan_device *wlan, struct rt_sta_info *sta_info)
{
    struct rw007_ap_info_value value;
    value.info.security = sta_info->security;
    value.info.band = RT_802_11_BAND_2_4GHZ;
    value.info.datarate = 0;
    value.info.channel = sta_info->channel;
    value.info.hidden = 0;
    value.info.rssi = 0;
    value.info.ssid = sta_info->ssid;
    rt_memcpy(value.info.bssid, sta_info->bssid, 6);
    strncpy(value.passwd, (const char *)&sta_info->key.val[0], sta_info->key.len);
    value.passwd[sta_info->key.len] = '\0';
    return spi_set_data(wlan, RW00x_CMD_JOIN, &value, sizeof(value));
}

static rt_err_t wlan_softap(struct rt_wlan_device *wlan, struct rt_ap_info *ap_info)
{
    struct rw007_ap_info_value value;
    value.info.security = ap_info->security;
    value.info.band = RT_802_11_BAND_2_4GHZ;
    value.info.datarate = 0;
    value.info.channel = ap_info->channel;
    value.info.hidden = ap_info->hidden;
    value.info.rssi = 0;
    value.info.ssid = ap_info->ssid;
    strncpy(value.passwd, (const char *)&ap_info->key.val[0], ap_info->key.len);
    value.passwd[ap_info->key.len] = '\0';
    return spi_set_data(wlan, RW00x_CMD_SOFTAP, &value, sizeof(value));
}

static rt_err_t wlan_disconnect(struct rt_wlan_device *wlan)
{
    return spi_set_data(wlan, RW00x_CMD_DISCONNECT, RT_NULL, 0);
}

static rt_err_t wlan_ap_stop(struct rt_wlan_device *wlan)
{
    return spi_set_data(wlan, RW00x_CMD_AP_STOP, RT_NULL, 0);
}

static rt_err_t wlan_ap_deauth(struct rt_wlan_device *wlan, rt_uint8_t mac[])
{
    return spi_set_data(wlan, RW00x_CMD_AP_DEAUTH, mac, 6);
}

static rt_err_t wlan_scan_stop(struct rt_wlan_device *wlan)
{
    return spi_set_data(wlan, RW00x_CMD_SCAN_STOP, RT_NULL, 0);
}

static int wlan_get_rssi(struct rt_wlan_device *wlan)
{
    int rssi = -1;
    rt_uint32_t size_of_data;
    spi_get_data(wlan, RW00x_CMD_GET_RSSI, &rssi, &size_of_data);
    return rssi;
}

static rt_err_t wlan_set_powersave(struct rt_wlan_device *wlan, int level)
{
    return spi_set_data(wlan, RW00x_CMD_SET_PWR_SAVE, &level, sizeof(level));
}

static int wlan_get_powersave(struct rt_wlan_device *wlan)
{
    int level = -1;
    rt_uint32_t size_of_data;
    spi_get_data(wlan, RW00x_CMD_GET_PWR_SAVE, &level, &size_of_data);
    return level;
}

static rt_err_t wlan_cfg_promisc(struct rt_wlan_device *wlan, rt_bool_t start)
{
    return spi_set_data(wlan, RW00x_CMD_CFG_PROMISC, &start, sizeof(start));
}

static rt_err_t wlan_cfg_filter(struct rt_wlan_device *wlan, struct rt_wlan_filter *filter)
{
    return -RT_ENOSYS;
}

static rt_err_t wlan_set_channel(struct rt_wlan_device *wlan, int channel)
{
    return spi_set_data(wlan, RW00x_CMD_SET_CHANNEL, &channel, sizeof(channel));
}

static int wlan_get_channel(struct rt_wlan_device *wlan)
{
    int channel = -1;
    rt_uint32_t size_of_data;
    spi_get_data(wlan, RW00x_CMD_GET_CHANNEL, &channel, &size_of_data);
    return channel;
}

static rt_err_t wlan_set_country(struct rt_wlan_device *wlan, rt_country_code_t country_code)
{
    return spi_set_data(wlan, RW00x_CMD_SET_COUNTRY, &country_code, sizeof(country_code));
}

static rt_country_code_t wlan_get_country(struct rt_wlan_device *wlan)
{
    rt_country_code_t code;
    rt_uint32_t size_of_data;
    spi_get_data(wlan, RW00x_CMD_GET_COUNTRY, &code, &size_of_data);
    return code;
}

static rt_err_t wlan_set_mac(struct rt_wlan_device *wlan, rt_uint8_t mac[])
{
    if(wlan == wifi_sta.wlan)
    {
        return spi_set_data(wlan, RW00x_CMD_MAC_SET, mac, 6);
    }
    return spi_set_data(wlan, RW00x_CMD_AP_MAC_SET, mac, 6);
}

static rt_err_t wlan_get_mac(struct rt_wlan_device *wlan, rt_uint8_t mac[])
{
    rt_uint32_t size_of_data;
    if(wlan == wifi_sta.wlan)
    {
        return spi_get_data(wlan, RW00x_CMD_MAC_GET, mac, &size_of_data);
    }
    return spi_get_data(wlan, RW00x_CMD_AP_MAC_GET, mac, &size_of_data);
}

static int wlan_send(struct rt_wlan_device *wlan, void *buff, int len)
{
    struct rw007_spi * hspi = wifi_get_dev_by_wlan(wlan)->hspi;
    struct spi_data_packet * data_packet;

    if(wlan == RT_NULL)
    {
        return -1;
    }

    data_packet = rt_mp_alloc(&hspi->spi_tx_mp, RT_WAITING_FOREVER);

    if (wlan == wifi_sta.wlan)
    {
        data_packet->data_type = DATA_TYPE_STA_ETH_DATA;
    }
    else
    {
        data_packet->data_type = DATA_TYPE_AP_ETH_DATA;
    }
    data_packet->data_len = len;

    rt_memcpy(data_packet->buffer, buff, len);

    rt_mb_send(&hspi->spi_tx_mb, (rt_uint32_t)data_packet);
    rt_event_send(&spi_wifi_data_event, 1);
    return len;
}

const static struct rt_wlan_dev_ops ops =
{
    .wlan_init          = wlan_init,
    .wlan_mode          = wlan_mode,
    .wlan_scan          = wlan_scan,
    .wlan_join          = wlan_join,
    .wlan_softap        = wlan_softap,
    .wlan_disconnect    = wlan_disconnect,
    .wlan_ap_stop       = wlan_ap_stop,
    .wlan_ap_deauth     = wlan_ap_deauth,
    .wlan_scan_stop     = wlan_scan_stop,
    .wlan_get_rssi      = wlan_get_rssi,
    .wlan_set_powersave = wlan_set_powersave,
    .wlan_get_powersave = wlan_get_powersave,
    .wlan_cfg_promisc   = wlan_cfg_promisc,
    .wlan_cfg_filter    = wlan_cfg_filter,
    .wlan_set_channel   = wlan_set_channel,
    .wlan_get_channel   = wlan_get_channel,
    .wlan_set_country   = wlan_set_country,
    .wlan_get_country   = wlan_get_country,
    .wlan_set_mac       = wlan_set_mac,
    .wlan_get_mac       = wlan_get_mac,
    .wlan_recv          = RT_NULL,
    .wlan_send          = wlan_send,
};

rt_err_t rt_hw_wifi_init(const char *spi_device_name)
{
    static struct rt_wlan_device wlan_sta, wlan_ap;
    rt_err_t ret;
    wifi_sta.wlan = &wlan_sta;
    wifi_sta.hspi = &rw007_spi;
    wifi_ap.wlan = &wlan_ap;
    wifi_ap.hspi = &rw007_spi;
    /* align and struct size check. */
    RT_ASSERT((SPI_MAX_DATA_LEN & 0x03) == 0);

    memset(&rw007_spi, 0, sizeof(struct rw007_spi));

    rw007_spi.spi_device = (struct rt_spi_device *)rt_device_find(spi_device_name);

    if (rw007_spi.spi_device == RT_NULL)
    {
        LOG_E("spi device %s not found!\r", spi_device_name);
        return -RT_ENOSYS;
    }

    spi_wifi_hw_init();

    /* config spi */
    {
        struct rt_spi_configuration cfg;
        cfg.data_width = 8;
        cfg.mode = RT_SPI_MODE_0 | RT_SPI_MSB; /* SPI Compatible: Mode 0. */
        cfg.max_hz = 30 * 1000000;             /* 15M 007 max 30M */
        rt_spi_configure(rw007_spi.spi_device, &cfg);
    }

    /* init spi send mempool */
    rt_mp_init(&rw007_spi.spi_tx_mp,
               "spi_tx",
               &rw007_spi.spi_tx_mempool[0],
               sizeof(rw007_spi.spi_tx_mempool),
               sizeof(struct spi_data_packet));

    /* init spi send mailbox */
    rt_mb_init(&rw007_spi.spi_tx_mb,
               "spi_tx",
               &rw007_spi.spi_tx_mb_pool[0],
               SPI_TX_POOL_SIZE,
               RT_IPC_FLAG_PRIO);
    
    /* init spi recv mempool */
    rt_mp_init(&rw007_spi.spi_rx_mp,
               "spi_rx",
               &rw007_spi.spi_rx_mempool[0],
               sizeof(rw007_spi.spi_rx_mempool),
               sizeof(struct spi_data_packet));

    /* init spi recv mailbox */
    rt_mb_init(&rw007_spi.spi_rx_mb,
               "spi_rx",
               &rw007_spi.spi_rx_mb_pool[0],
               SPI_TX_POOL_SIZE,
               RT_IPC_FLAG_PRIO);

    /* init spi data notify event */
    rt_event_init(&spi_wifi_data_event, "wifi", RT_IPC_FLAG_FIFO);

    rw007_spi.rw007_cmd_event = rt_event_create("wifi_cmd", RT_IPC_FLAG_FIFO);

    /* register wlan device for ap */
    ret = rt_wlan_dev_register(&wlan_ap, RT_WLAN_DEVICE_AP_NAME, &ops, 0, &wifi_ap);
    if (ret != RT_EOK)
    {
        return ret;
    }

    /* register wlan device for sta */
    ret = rt_wlan_dev_register(&wlan_sta, RT_WLAN_DEVICE_STA_NAME, &ops, 0, &wifi_sta);
    if (ret != RT_EOK)
    {
        return ret;
    }

    {
        rt_thread_t tid;

        /* Create package parse thread */
        tid = rt_thread_create("wifi_handle",
                               wifi_data_process_thread_entry,
                               &rw007_spi,
                               2048,
                               8,
                               20);
        if(!tid)
        {
            return -RT_ERROR;
        }
        rt_thread_startup(tid);

        /* Create wifi transfer thread */
        tid = rt_thread_create("wifi_xfer",
                               spi_wifi_data_thread_entry,
                               RT_NULL,
                               2048,
                               9,
                               20);
        if(!tid)
        {
            return -RT_ERROR;
        }
        rt_thread_startup(tid);
    }
    /* Start first transfer */
    rt_event_send(&spi_wifi_data_event, 1);
    return RT_EOK;
}

void spi_wifi_isr(int vector)
{
    /* enter interrupt */
    rt_interrupt_enter();

    /* device has a package to ready transfer */
    rt_event_send(&spi_wifi_data_event, 1);

    /* leave interrupt */
    rt_interrupt_leave();
}
