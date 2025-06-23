//'compatible = "custom,bus485"' node in the Devicetree
#define DT_DRV_COMPAT custom_bus485

#include <errno.h>
#include <zephyr/logging/log.h>
#include <zephyr/kernel.h>

#include "bus485.h"

#define CUSTOM_BUS485_INIT_PRIORITY CONFIG_CUSTOM_BUS485_INIT_PRIORITY

bool isWrongByteAfterSend = false;
bool isFirstReceive = false;
//Enable logging
LOG_MODULE_REGISTER(bus485);
#define QUEUE_SIZE 256

uint8_t uart_rx_msgq_buffer[QUEUE_SIZE * sizeof(uint8_t)];
struct k_msgq uart_rx_msgq;
//K_MSGQ_DEFINE(uart_rx_queue, sizeof(uint8_t), QUEUE_SIZE, 4);

static struct device *uart_tmp;

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

    
#if 1
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
            //LOG_DBG("pack rcv with %d bytes\r\n", ret);
            //if(ret > 0){
             //   LOG_HEXDUMP_INF(buf, ret, "Rcv buff");   
            //}
        }
    }
#else
    if(!uart_irq_update(uart_t))
        return;

    if(!uart_irq_is_pending(uart_t))
        return;
    uint8_t count_wait = 0;
    int rx = uart_irq_rx_ready(uart_t);
    if(rx < 0)
        return;
    if(rx == 0)
        return;

        uint32_t total_rcv = 0;
        uint32_t counter = 0;
        //while(total_rcv < 256){
            uint8_t tmp[4];
            int recv_len = uart_fifo_read(uart_t, tmp, 4);
            if(recv_len > 0){
                LOG_DBG("pack rcv with %d bytes\r\n", total_rcv);
                if(total_rcv > 0){
                    LOG_HEXDUMP_INF(buf, total_rcv, "Rcv buff");   
                }
            }

        //}
#endif
    /*while(uart_irq_update(dev) && uart_irq_is_pending(dev)){
        if(uart_irq_rx_ready(dev)){
            int recv_len = uart_fifo_read(dev, buf, 256);
            LOG_DBG("pack rcv with %d bytes\r\n", recv_len);
            if(recv_len > 0){
                LOG_HEXDUMP_INF(buf, recv_len, "Rcv buff");
            }
        }
    }*/
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

    uart_tmp = uart_dev;

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
    ret = uart_irq_callback_user_data_set(uart_dev, interrupt_handler, (void*)uart_dev);
    if(ret < 0){
        LOG_ERR("Could not accept interrupt for uart (%d)\r\n", ret);
        return ret;
    }
    k_msgq_init(&uart_rx_msgq, uart_rx_msgq_buffer, sizeof(uint8_t), QUEUE_SIZE);
    //uart_irq_rx_enable(uart_dev);

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
    
    k_sleep(K_USEC(100));
    ret = gpio_pin_set_dt(dir, 0);
    if(ret < 0){
        LOG_ERR("Error (%d): failed to reset dir pin\r\n", ret);
        return ret;
    }
    k_sleep(K_USEC(100));
    isWrongByteAfterSend= true;
    //uint8_t b[1];
    //uart_fifo_read(uart_dev, b, 1);
    //uart_irq_rx_enable(uart_dev);
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
        /*else if(count > 0){
            if(k_uptime_get() - start_time > 15){ //is idle 
                break;
            } 
        }*/
        if(!isFirstReceive){
            k_sleep(K_MSEC(10));
            continue;
        }
        //read from queue
        uint8_t data = 0;
        k_timeout_t time_wait = Z_TIMEOUT_MS(15);
        ret = k_msgq_get(&uart_rx_msgq, (uint8_t*)&data, time_wait);
        if( ret == -EAGAIN && count > 0)
            break;
        else{
            tmpBuf[count] = data;
            count++;
            if(count == buffer_size)
                break;
        }
        //start_time = k_uptime_get();
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