#include <rtthread.h>

#ifdef RW007_USING_STM32_DRIVERS
#include <rtdevice.h>
#include <drv_spi.h>
#include <board.h>
#include <spi_wifi_rw007.h>

#define RW007_AT_MODE   3
#define RW007_SPI_MODE  1

#define RW007_WORKING_STATION_MODE

extern void spi_wifi_isr(int vector);

static void set_rw007_mode(int mode)
{
    /* Configure IO */
    rt_pin_mode(RW007_RST_PIN, PIN_MODE_OUTPUT);
    rt_pin_mode(RW007_INT_BUSY_PIN, PIN_MODE_INPUT_PULLDOWN);
    rt_pin_mode(RW007_BOOT0_PIN, PIN_MODE_OUTPUT);
    rt_pin_mode(RW007_BOOT1_PIN, PIN_MODE_OUTPUT);
    
    /* Reset rw007 and config mode */
    rt_pin_write(RW007_RST_PIN, PIN_LOW);
    rt_pin_write(RW007_BOOT0_PIN, mode & 0x01 ? PIN_HIGH : PIN_LOW);
    rt_pin_write(RW007_BOOT1_PIN, mode & 0x02 ? PIN_HIGH : PIN_LOW);
    rt_thread_delay(rt_tick_from_millisecond(100));
    rt_pin_write(RW007_RST_PIN, PIN_HIGH);

    /* Wait rw007 ready(exit busy stat) */
    while(!rt_pin_read(RW007_INT_BUSY_PIN))
    {
    }
    rt_thread_delay(rt_tick_from_millisecond(100));
}

int wifi_spi_device_init(void)
{
    char sn_version[32];
    set_rw007_mode(RW007_SPI_MODE);
    stm32_spi_bus_attach_device(RW007_CS_PIN, RW007_SPI_BUS_NAME, "wspi");
    rt_hw_wifi_init("wspi");
#ifdef RW007_WORKING_STATION_MODE
    rt_wlan_set_mode(RT_WLAN_DEVICE_STA_NAME, RT_WLAN_STATION);
#else
    rt_wlan_set_mode(RT_WLAN_DEVICE_AP_NAME, RT_WLAN_AP);
#endif
    
    rw007_sn_get(sn_version);
    rt_kprintf("\nrw007  sn: [%s]\n", sn_version);
    rw007_version_get(sn_version);
    rt_kprintf("rw007 ver: [%s]\n\n", sn_version);
    return 0;
}
INIT_APP_EXPORT(wifi_spi_device_init);

static int rw007_update(void)
{
    rt_device_t device = rt_device_find(RW007_SPI_BUS_NAME);
    struct stm32_spi *hspi = (struct stm32_spi *)device->user_data;
    __HAL_SPI_DISABLE((SPI_HandleTypeDef *)hspi);
    HAL_SPI_MspDeInit((SPI_HandleTypeDef *)hspi);
    set_rw007_mode(RW007_AT_MODE);
    return 0;
}
MSH_CMD_EXPORT(rw007_update, rw007_update);

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

rt_bool_t spi_wifi_is_busy(void)
{
    return !rt_pin_read(RW007_INT_BUSY_PIN);
}

void spi_wifi_int_cmd(rt_bool_t cmd)
{
    rt_pin_irq_enable(RW007_INT_BUSY_PIN, cmd);
}

#endif /* RW007_USING_STM32_DRIVERS */
