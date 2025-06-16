//'compatible = "custom,bus485"' node in the Devicetree
#define DT_DRV_COMPAT custom_bus485

#include <errno.h>
#include <zephyr/logging/log.h>

#include "bus485.h"

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

static int bus485_init(const struct device * dev)
{
    int ret;

    const struct bus485_config *cfg = (const struct bus485_config*)dev->config;

    const struct gpio_dt_spec *dir = &cfg->dir;

    
    const struct device *uart = &cfg->uart_dev;
    LOG_DBG("Initializing bus485 (instance ID: %u)\r\n", cfg->id);

    if(!uart){
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

    //!TODO CONFIGURE UART

    
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

    ret = gpio_pin_set_dt(dir, 1);
    if(ret < 0){
        LOG_ERR("Error (%d): failed to set dir pin\r\n", ret);
        return ret;
    }

    //send to uart;

    ret = gpio_pin_set_dt(dir, 0);//may be it's call in callback
    if(ret < 0){
        LOG_ERR("Error (%d): failed to reset dir pin\r\n", ret);
        return ret;
    }
    return 0;
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

    return 0;
}

//--------Devicetree handling --------------------
static const bus485_driver_api bus485_driver_api_funcs {
    .bus485_lock = b485_lock,
    .bus485_release = b485_release,
    .bus485_send = b485_send,
    .bus485_recv = b485_recv,
    .bus485_flush = b485_flush,
    .bus485_set_baudrate = b485_set_baudrate
};

//macro to define driver instance
#define BUS485_DEFINE(inst)\
    static const struct bus485_config bus485_config_##inst = {      \
        .dir = GPIO_DT_SPEC_GET(                                    \
            DT_PHANDLE(DT_INST(inst, custom_bus485), pin), gpios),  \
        .uart_dev = DT_LABEL(DT_PHANDLE(DT_INST(inst, custom_bus485), uart)),\
        .id = inst                                                              \
    };          \
    DEVICE_DT_INST_DEFINE(inst,                                 \
                            bus485_init,                        \
                            NULL,                               \
                            NULL,                               \
                            &bus485_config_##inst,              \
                            POST_KERNEL,                        \
                            CONFIG_GPIO_INIT_PRIORITY,          \
                            &bus485_driver_api_funcs);          \


DT_INST_FOREACH_STATUS_OKAY(BUS485_DEFINE)