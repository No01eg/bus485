//'compatible = "custom,bus485"' node in the Devicetree
#define DT_DRV_COMPAT custom_bus485

#include <errno.h>
#include <zephyr/logging/log.h>

#include "bus485.h"

#define CUSTOM_BUS485_INIT_PRIORITY CONFIG_CUSTOM_BUS485_INIT_PRIORITY

//Enable logging
LOG_MODULE_REGISTER(bus485);



static int bus485_init(const struct device * dev);

static int32_t b485_lock(const struct device * dev);

static int32_t b485_release(const struct device * dev);

static int32_t b485_send(const struct device * dev,
                           const uint8_t * buffer,
                           uint32_t count);

static int32_t b485_recv(const struct device * dev,
                           const uint8_t * buffer,
                           uint32_t buffer_size,
                           uint32_t timeout_ms);

static int32_t b485_flush(const struct device * dev);

static int32_t b485_set_baudrate(const struct device * dev,
                                   uint32_t baudrate);

//----private------------------------------------------------

static void interrupt_handler(const struct device *dev, void *user_data)
{
    ARG_UNUSED(user_data);

    while(uart_irq_update(dev) && uart_irq_is_pending(dev)){

    }
}

static int bus485_init(const struct device * dev)
{
    int ret;

    const struct bus485_config *cfg = (const struct bus485_config*)dev->config;

    const struct gpio_dt_spec *dir = &cfg->dir;

    
    const struct device *uart_dev = cfg->uart_dev;
    LOG_DBG("Initializing bus485 (instance ID: %u)\r\n", cfg->id);

    //if (!device_is_ready(uart_dev)){
    if(!uart_dev){
        LOG_ERR("UART is not found\r\n");
        return -ENODEV;
    }

    if(!gpio_is_ready_dt(dir)){
        LOG_ERR("GPIO is not ready\r\n");
        return -ENODEV;
    }


    ret = gpio_pin_configure_dt(dir, GPIO_OUTPUT_ACTIVE);
    if(ret < 0){
        LOG_ERR("Could not configure GPIO as output\r\n");
        return -ENODEV;
    }

    LOG_DBG("ACCEPT INTERRUPT FOR UART\r\n");
    uart_irq_rx_disable(uart_dev);
	uart_irq_tx_disable(uart_dev);
    ret = uart_irq_callback_user_data_set(uart_dev, interrupt_handler, NULL);

    if(ret < 0){
        LOG_ERR("Could not accept interrupt for uart (%d)\r\n", ret);
        return ret;
    }
    return 0;
}

//----public api---------------------------------------------

static int32_t b485_lock(const struct device * dev)
{
    int ret;

    return 0;
}

static int32_t b485_release(const struct device * dev)
{
    int ret;

    return 0;
}

static int32_t b485_send(const struct device * dev,
                           const uint8_t * buffer,
                           uint32_t count)
{
    int ret;

    const struct bus485_config *cfg = (const struct bus485_config*)dev->config;

    const struct gpio_dt_spec *dir = &cfg->dir;
    const struct device *uart_dev = cfg->uart_dev;

    ret = gpio_pin_set_dt(dir, 1);
    if(ret < 0){
        LOG_ERR("Error (%d): failed to set dir pin\r\n", ret);
        return ret;
    }

    uint32_t total_send = 0;
    
    while(total_send < count){
        int send = uart_fifo_fill(uart_dev, &buffer[total_send], count - total_send);
        if(send > 0){
            total_send += send;
        }
    }

    while(!uart_irq_tx_complete(uart_dev))
        k_sleep(K_USEC(1));
    
    ret = gpio_pin_set_dt(dir, 0);
    if(ret < 0){
        LOG_ERR("Error (%d): failed to reset dir pin\r\n", ret);
        return ret;
    }
    return total_send;
}

static int32_t b485_recv(const struct device * dev,
                           const uint8_t * buffer,
                           uint32_t buffer_size,
                           uint32_t timeout_ms)
{
    int ret;

    return 0;
}

static int32_t b485_flush(const struct device * dev)
{
    int ret;

    return 0;
}

static int32_t b485_set_baudrate(const struct device * dev,
                                   uint32_t baudrate)
{
    int ret;

    struct uart_config config;

    const struct bus485_config *cfg = (const struct bus485_config*)dev->config;

    const struct device * uart = cfg->uart_dev;

    ret = uart_config_get(uart, &config);
    if(ret < 0){
        LOG_ERR("Error (%d): failed to read uart param\r\n");
        return ret;
    }

    config.baudrate = baudrate;

    ret = uart_configure(uart, &config);
    if(ret < 0){
        LOG_ERR("Error (%d): failed to accept new uart param\r\n");
        return ret;
    }

    return 0;
}

//--------Devicetree handling --------------------
static const struct bus485_driver_api bus485_driver_api_funcs = {
    .bus485_lock = b485_lock,
    .bus485_release = b485_release,
    .bus485_send = b485_send,
    .bus485_recv = b485_recv,
    .bus485_flush = b485_flush, 
    .bus485_set_baudrate = b485_set_baudrate,
};

//macro to define driver instance
#define BUS485_DEFINE(inst)\
    static const struct bus485_config bus485_config_##inst = {      \
        .dir = GPIO_DT_SPEC_INST_GET(inst, pin_gpios),  \
        .uart_dev = DEVICE_DT_GET(DT_INST_BUS(inst)),\
        .id = inst                                                              \
    };          \
    DEVICE_DT_INST_DEFINE(inst,                                 \
                            bus485_init,                        \
                            NULL,                               \
                            NULL,                               \
                            &bus485_config_##inst,              \
                            POST_KERNEL,                  \
                            CUSTOM_BUS485_INIT_PRIORITY,          \
                            &bus485_driver_api_funcs);          \


DT_INST_FOREACH_STATUS_OKAY(BUS485_DEFINE)