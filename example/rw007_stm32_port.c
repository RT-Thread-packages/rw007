#include <rtthread.h>

#ifdef RW007_USING_STM32_DRIVERS
#include <rtdevice.h>
#include <drv_spi.h>
#include <board.h>
#include <spi_wifi_rw007.h>

extern void spi_wifi_isr(int vector);

static void rw007_gpio_init(void)
{
    /* Configure IO */
    rt_pin_mode(RW007_RST_PIN, PIN_MODE_OUTPUT);
    rt_pin_mode(RW007_INT_BUSY_PIN, PIN_MODE_INPUT_PULLDOWN);

    /* Reset rw007 and config mode */
    rt_pin_write(RW007_RST_PIN, PIN_LOW);
    rt_thread_delay(rt_tick_from_millisecond(100));
    rt_pin_write(RW007_RST_PIN, PIN_HIGH);

    /* Wait rw007 ready(exit busy stat) */
    while(!rt_pin_read(RW007_INT_BUSY_PIN))
    {
        rt_thread_delay(5);
    }

    rt_thread_delay(rt_tick_from_millisecond(200));
    rt_pin_mode(RW007_INT_BUSY_PIN, PIN_MODE_INPUT_PULLUP);
}

int wifi_spi_device_init(void)
{
    char sn_version[32];
    
    GPIO_TypeDef *cs_gpiox;
    uint16_t cs_pin;

    cs_gpiox = (GPIO_TypeDef *)((rt_base_t)GPIOA + (rt_base_t)(RW007_CS_PIN / 16) * 0x0400UL);
    cs_pin = (uint16_t)(1 << RW007_CS_PIN % 16);
    
    rw007_gpio_init();
    rt_hw_spi_device_attach(RW007_SPI_BUS_NAME, "wspi", cs_gpiox, cs_pin);
    rt_hw_wifi_init("wspi");

    rt_wlan_set_mode(RT_WLAN_DEVICE_STA_NAME, RT_WLAN_STATION);
    rt_wlan_set_mode(RT_WLAN_DEVICE_AP_NAME, RT_WLAN_AP);

    rw007_sn_get(sn_version);
    rt_kprintf("\nrw007  sn: [%s]\n", sn_version);
    rw007_version_get(sn_version);
    rt_kprintf("rw007 ver: [%s]\n\n", sn_version);

    return 0;
}
INIT_APP_EXPORT(wifi_spi_device_init);

static void int_wifi_irq(void * p)
{
    ((void)p);
    spi_wifi_isr(0);
}

void spi_wifi_hw_init(void)
{
    rt_pin_attach_irq(RW007_INT_BUSY_PIN, PIN_IRQ_MODE_FALLING, int_wifi_irq, 0);
    rt_pin_irq_enable(RW007_INT_BUSY_PIN, RT_TRUE);
}

rt_err_t rw007_open_ble(void)
{
    return rw007_ble_mode_set(RW00x_BLE_ENABLE, NONAUTO_CONNECT_MODE);
}
MSH_CMD_EXPORT(rw007_open_ble, open rw007 ble function)

rt_err_t rw007_ble_networking(void)
{
    struct sdio_ble_connect_info ble_connect_info;
    rt_uint32_t ret;

    ret = rw007_ble_wifi_info_get(&ble_connect_info);
    if(ret != RT_EOK)
    {
        return RT_ERROR;
    }
    return rt_wlan_connect(ble_connect_info.ssid.val, ble_connect_info.key.val);
}
MSH_CMD_EXPORT(rw007_ble_networking, rw007 ble networking)

#endif /* RW007_USING_STM32_DRIVERS */
