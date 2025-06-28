//'compatible = "custom,bus485"' node in the Devicetree
#define DT_DRV_COMPAT custom_bus485

#include <errno.h>
#include <zephyr/logging/log.h>
#include <zephyr/kernel.h>
#include <zephyr/devicetree.h>

#include "bus485.h"

#define CUSTOM_BUS485_INIT_PRIORITY CONFIG_CUSTOM_BUS485_INIT_PRIORITY
#define QUEUE_SIZE CONFIG_CUSTOM_BUS485_QUEUE_SIZE
#define SYM_COUNT_IDLE CONFIG_CUSTOM_BUS485_RECV_SYM_IDLE_COUNT

#if DT_NODE_HAS_PROP(DT_NODELABEL(bus485), pin-gpios)
#define DATA_ENABLE_ACTIVE
#endif

#define BITS_IN_SYM 10

#define READ_ONE_BYTE_AFTER_WRITE
bool isWrongByteAfterSend = false;
bool isFirstReceive = false;
uint32_t us_per_sym = 1;
//Enable logging
LOG_MODULE_REGISTER(bus485);

uint8_t uart_rx_msgq_buffer[QUEUE_SIZE * sizeof(uint8_t)];
struct k_msgq uart_rx_msgq;

struct k_sem bus_sem;


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
    //ARG_UNUSED(user_data);
    const struct device *uart_td = (struct device*)user_data;
    uint8_t buf[1];

    int ret;

    while(uart_irq_update(uart_td) > 0 && uart_irq_is_pending(uart_td)){
        ret = uart_irq_rx_ready(uart_td);
        if(ret < 0)
            return;
    
        ret = uart_fifo_read(uart_td, buf, 1);
        if(ret <= 0)
            return;
        else{
            if(isWrongByteAfterSend)
            {
                isWrongByteAfterSend = false;
                return;
            }
            if(!isFirstReceive)
                isFirstReceive = true;
            k_msgq_put(&uart_rx_msgq, buf, K_NO_WAIT);
        }
    }
}

static int bus485_init(const struct device * dev)
{
    int ret;

    const struct bus485_config *cfg = (const struct bus485_config*)dev->config;

    const struct gpio_dt_spec *data_enable = &cfg->data_enable;

    
    const struct device *uart_dev = cfg->uart_dev;
    LOG_DBG("Initializing bus485 (instance ID: %u)\r\n", cfg->id);

    if (!device_is_ready(uart_dev)){
        LOG_ERR("UART is not found\r\n");
        return -ENODEV;
    }
#ifdef DATA_ENABLE_ACTIVE
    if(!gpio_is_ready_dt(data_enable)){
        LOG_ERR("GPIO is not ready\r\n");
        return -ENODEV;
    }


    ret = gpio_pin_configure_dt(data_enable, GPIO_OUTPUT_ACTIVE);
    if(ret < 0){
        LOG_ERR("Could not configure GPIO as output\r\n");
        return -ENODEV;
    }
#endif

    LOG_DBG("ACCEPT INTERRUPT FOR UART\r\n");
    uart_irq_rx_disable(uart_dev);
	uart_irq_tx_disable(uart_dev);
    ret = uart_irq_callback_user_data_set(uart_dev, interrupt_handler, (void*)uart_dev);
    if(ret < 0){
        LOG_ERR("Could not accept interrupt for uart (%d)\r\n", ret);
        return ret;
    }
    k_msgq_init(&uart_rx_msgq, uart_rx_msgq_buffer, sizeof(uint8_t), QUEUE_SIZE);

    k_sem_init(&bus_sem, 1, 1);

    return 0;
}

//----public api---------------------------------------------

static int32_t b485_lock(const struct device * dev)
{
    int ret;

    ret = k_sem_take(&bus_sem, K_FOREVER);
    if(ret < 0){
        return ret;
    }

    return 0;
}

static int32_t b485_release(const struct device * dev)
{
    int ret;

    k_sem_give(&bus_sem);

    return 0;
}

static int32_t b485_send(const struct device * dev,
                           const uint8_t * buffer,
                           uint32_t count)
{
    int ret;

    const struct bus485_config *cfg = (const struct bus485_config*)dev->config;

    const struct device *uart_dev = cfg->uart_dev;
#ifdef DATA_ENABLE_ACTIVE
    const struct gpio_dt_spec *data_enable = &cfg->data_enable;

    ret = gpio_pin_set_dt(data_enable, 1);
    if(ret < 0){
        LOG_ERR("Error (%d): failed to set data_enable pin\r\n", ret);
        return ret;
    }
#endif

    uint32_t total_send = 0;
    
    while(total_send < count){
        int send = uart_fifo_fill(uart_dev, &buffer[total_send], count - total_send);
        if(send > 0){
            total_send += send;
        }
    }

    while(!uart_irq_tx_complete(uart_dev))
        k_sleep(K_USEC(1));
#ifdef DATA_ENABLE_ACTIVE    
    //k_sleep(K_USEC(100));
    ret = gpio_pin_set_dt(data_enable, 0);
    if(ret < 0){
        LOG_ERR("Error (%d): failed to reset data_enable pin\r\n", ret);
        return ret;
    }
    k_sleep(K_MSEC(8));
    #if defined(READ_ONE_BYTE_AFTER_WRITE)
    isWrongByteAfterSend= true;
    #endif
#endif
    return total_send;
}

static int32_t b485_recv(const struct device * dev,
                           const uint8_t * buffer,
                           uint32_t buffer_size,
                           uint32_t timeout_ms)
{
    int ret;
    int count = 0;

    const struct bus485_config *cfg = (const struct bus485_config*)dev->config;
    const struct device *uart_dev = cfg->uart_dev;

    uint64_t start_time = k_uptime_get();
    
    uart_irq_rx_enable(uart_dev);
    uint8_t *tmpBuf = buffer;
    while(1){
        if(count == 0 && ((k_uptime_get() - start_time) > timeout_ms)) //timeout
            return -EAGAIN;
        if(!isFirstReceive){
            k_sleep(K_MSEC(10));
            continue;
        }
        //read from queue
        uint8_t data = 0;
        k_timeout_t time_wait = Z_TIMEOUT_US(us_per_sym * SYM_COUNT_IDLE);
        ret = k_msgq_get(&uart_rx_msgq, (uint8_t*)&data, time_wait);
        if( ret == -EAGAIN && count > 0)
            break;
        else{
            tmpBuf[count] = data;
            count++;
            if(count == buffer_size)
                break;
        }
    }
    LOG_DBG("pack rcv with %d bytes\r\n", count);
    if(count > 0){
       LOG_HEXDUMP_INF(buffer, count, "Rcv buff");   
    }
    isFirstReceive = false;
    uart_irq_rx_disable(uart_dev);
    return count;
}

static int32_t b485_flush(const struct device * dev)
{
    int ret;

    ret = k_msgq_cleanup(&uart_rx_msgq);
    if(ret < 0){
        LOG_ERR("Failed to cleanup queue (%d)\r\n", ret);
        return ret;
    }

    return 0;
}

static int32_t b485_set_baudrate(const struct device * dev,
                                   uint32_t baudrate)
{
    int ret;

    struct uart_config config;

    const struct bus485_config *cfg = (const struct bus485_config*)dev->config;

    const struct device * uart = cfg->uart_dev;

    us_per_sym = BITS_IN_SYM * 1000000 / baudrate; 

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
        .data_enable = GPIO_DT_SPEC_INST_GET(inst, pin_gpios),  \
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