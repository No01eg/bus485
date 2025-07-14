//'compatible = "custom,bus485"' node in the Devicetree
#define DT_DRV_COMPAT custom_bus485

#include <errno.h>
#include <zephyr/logging/log.h>
#include <zephyr/kernel.h>
#include <zephyr/devicetree.h>

#include "bus485.h"

#define CUSTOM_BUS485_INIT_PRIORITY CONFIG_CUSTOM_BUS485_INIT_PRIORITY
#define QUEUE_SIZE CONFIG_CUSTOM_BUS485_RX_QUEUE_SIZE
#define SYM_COUNT_IDLE CONFIG_CUSTOM_BUS485_RECV_SYM_IDLE_COUNT
#define TX_QUEUE_SIZE CONFIG_CUSTOM_BUS485_TX_QUEUE_SIZE


#define BITS_IN_SYM 10 // 1 start bit, 8 data bits, no parity, 1 stop bit (8n1 format )



//Enable logging
LOG_MODULE_REGISTER(bus485);

struct bus_data{
    uint32_t us_per_sym;
    bool is_use_data_enable;
    uint8_t uart_rx_msgq_buffer[QUEUE_SIZE * sizeof(uint8_t)];
    struct k_msgq uart_rx_msgq;
    uint8_t uart_tx_msgq_buffer[TX_QUEUE_SIZE * sizeof(uint8_t)];
    struct k_msgq uart_tx_msgq;
    struct k_sem bus_sem;
    struct k_sem tx_sem;
};

//configuration
struct bus485_config {
    const struct gpio_dt_spec data_enable;
    const struct device * uart_dev;
    struct bus_data *data;
    uint32_t id;
    
};

//----private------------------------------------------------

static void interrupt_handler(const struct device *dev, void *user_data)
{
    struct bus_data *bus_dat = (struct bus_data*)user_data;
    const struct device *uart_td = dev;
    uint8_t buf[1];

    int ret;

    while(uart_irq_update(uart_td) > 0 && uart_irq_is_pending(uart_td)){
        ret = uart_irq_rx_ready(uart_td);
        if(ret == 1){
            ret = uart_fifo_read(uart_td, buf, 1);
            if(ret <= 0)
                return;
            else{
                k_msgq_put(&bus_dat->uart_rx_msgq, buf, K_NO_WAIT);
            }
        }
        else if(uart_irq_tx_ready(uart_td)){
            uint8_t data;
            if(k_msgq_get(&bus_dat->uart_tx_msgq, &data, K_NO_WAIT) == 0){
                uart_fifo_fill(uart_td, &data, 1);
            }
            else{
                uart_irq_tx_disable(uart_td);
                k_sem_give(&bus_dat->tx_sem);
            }
        }
        else
            return;
    }
}

static int bus485_init(const struct device * dev)
{
    int ret;

    const struct bus485_config *cfg = (const struct bus485_config*)dev->config;

    const struct gpio_dt_spec *data_enable = &cfg->data_enable;

    
    const struct device *uart_dev = cfg->uart_dev;

    struct bus_data *bus_dat = (struct bus_data*)dev->data;
    #if CONFIG_CUSTOM_BUS485_DE_ACTIVE
    bus_dat->is_use_data_enable = true;
    #else
    bus_dat->is_use_data_enable = false;
    #endif
    LOG_DBG("Initializing bus485 (instance ID: %u)\r\n", cfg->id);

    if (!device_is_ready(uart_dev)){
        LOG_ERR("UART is not found\r\n");
        return -ENODEV;
    }
    if(bus_dat->is_use_data_enable){
        if(!gpio_is_ready_dt(data_enable)){
            LOG_ERR("GPIO is not ready\r\n");
            return -ENODEV;
        }

        ret = gpio_pin_configure_dt(data_enable, GPIO_OUTPUT_ACTIVE);
        if(ret < 0){
            LOG_ERR("Could not configure GPIO as output\r\n");
            return -ENODEV;
        }
    }

    uart_irq_rx_disable(uart_dev);
	uart_irq_tx_disable(uart_dev);
    ret = uart_irq_callback_user_data_set(uart_dev, interrupt_handler, (void*)bus_dat);
    if(ret < 0){
        LOG_ERR("Could not accept interrupt for uart (%d)\r\n", ret);
        return ret;
    }
    k_msgq_init(&bus_dat->uart_rx_msgq, bus_dat->uart_rx_msgq_buffer, sizeof(uint8_t), QUEUE_SIZE);

    k_msgq_init(&bus_dat->uart_tx_msgq, bus_dat->uart_tx_msgq_buffer, sizeof(uint8_t), TX_QUEUE_SIZE);

    k_sem_init(&bus_dat->bus_sem, 1, 1);
    k_sem_init(&bus_dat->tx_sem, 0, 1);
    uart_irq_rx_enable(uart_dev);

    return 0;
}

//----public api---------------------------------------------

int32_t bus485_lock(const struct device * dev)
{
    int ret;
    struct bus_data *bus_dat = (struct bus_data*)dev->data;
    ret = k_sem_take(&bus_dat->bus_sem, K_FOREVER);
    if(ret < 0){
        return ret;
    }

    return 0;
}

int32_t bus485_release(const struct device * dev)
{
    struct bus_data *bus_dat = (struct bus_data*)dev->data;
    k_sem_give(&bus_dat->bus_sem);

    return 0;
}

int32_t bus485_send(const struct device * dev,
                           const uint8_t * buffer,
                           uint32_t count)                           
{
    int ret;

    const struct bus485_config *cfg = (const struct bus485_config*)dev->config;
    struct bus_data *bus_dat = (struct bus_data*)dev->data;
    const struct device *uart_dev = cfg->uart_dev;
    const struct gpio_dt_spec *data_enable = &cfg->data_enable;
    if(bus_dat->is_use_data_enable){
        ret = gpio_pin_set_dt(data_enable, 1);
        if(ret < 0){
            LOG_ERR("Error (%d): failed to set data_enable pin\r\n", ret);
            return ret;
        }
    }

    int32_t total_send = 0;
    
    while(total_send < count){
        if(k_msgq_put(&bus_dat->uart_tx_msgq, &buffer[total_send], K_NO_WAIT) == 0)
            total_send++;
    }
    uart_irq_tx_enable(uart_dev);

    k_sem_take(&bus_dat->tx_sem, K_FOREVER);

    if(bus_dat->is_use_data_enable){  
        ret = gpio_pin_set_dt(data_enable, 0);
        if(ret < 0){
            LOG_ERR("Error (%d): failed to reset data_enable pin\r\n", ret);
            return ret;
        }
    }
    return total_send;
}

int32_t bus485_recv(const struct device * dev,
                    uint8_t * buffer,
                    uint32_t buffer_size,
                    uint32_t timeout_ms)                           
{
    int ret;
    int count = 0;

    const struct bus485_config *cfg = (const struct bus485_config*)dev->config;
    const struct device *uart_dev = cfg->uart_dev;
    struct bus_data *dat = (struct bus_data*)dev->data;

    uint8_t data = 0; 
    k_timeout_t time_wait = Z_TIMEOUT_MS(timeout_ms);
    ret = k_msgq_get(&dat->uart_rx_msgq, (uint8_t*)&data, time_wait);
    if(ret < 0){//break on timeout
        return -EAGAIN;
    }
    else{
        buffer[count] = data;
        count++;
    }

    time_wait = Z_TIMEOUT_US(dat->us_per_sym * SYM_COUNT_IDLE);

    while(1){
        ret = k_msgq_get(&dat->uart_rx_msgq, (uint8_t*)&data, time_wait);
        if( ret == -EAGAIN)//check idle 
            break;
        else{
            buffer[count] = data;
            count++;
            if(count == buffer_size)
                break;
        }
    }

    LOG_DBG("pack rcv with %d bytes\r\n", count);
    if(count > 0){
       LOG_HEXDUMP_INF(buffer, count, "Rcv buff");   
    }
    return count;
}

int32_t bus485_flush(const struct device * dev)
{
    int ret;
    struct bus_data *bus_dat = (struct bus_data*)dev->data;

    ret = k_msgq_cleanup(&bus_dat->uart_rx_msgq);
    if(ret < 0){
        LOG_ERR("Failed to cleanup RX queue (%d)\r\n", ret);
        return ret;
    }

    ret = k_msgq_cleanup(&bus_dat->uart_tx_msgq);
    if(ret < 0){
        LOG_ERR("Failed to cleanup TX queue (%d)\r\n", ret);
        return ret;
    }

    return 0;
}

int32_t bus485_set_baudrate(const struct device * dev,
                                   uint32_t baudrate)
{
    int ret;

    struct uart_config config;

    const struct bus485_config *cfg = (const struct bus485_config*)dev->config;
    struct bus_data *dat = (struct bus_data*)dev->data;

    const struct device * uart = cfg->uart_dev;

    dat->us_per_sym = BITS_IN_SYM * 1000000 / baudrate; 

    ret = uart_config_get(uart, &config);
    if(ret < 0){
        LOG_ERR("Error (%d): failed to read uart param\r\n",ret);
        return ret;
    }

    config.baudrate = baudrate;

    ret = uart_configure(uart, &config);
    if(ret < 0){
        LOG_ERR("Error (%d): failed to accept new uart param\r\n", ret);
        return ret;
    }

    return 0;
}


//macro to define driver instance
#define BUS485_DEFINE(inst)\
    static struct bus_data bus_data_peripheral_data_##inst = { \
        .us_per_sym = 1,                                        \
    };\
    static const struct bus485_config bus485_config_##inst = {      \
        .data_enable = GPIO_DT_SPEC_INST_GET(inst, pin_gpios),  \
        .uart_dev = DEVICE_DT_GET(DT_INST_BUS(inst)),\
        .data = &bus_data_peripheral_data_##inst, \
        .id = inst                                                              \
    };          \
    DEVICE_DT_INST_DEFINE(inst,                                 \
                            bus485_init,                        \
                            NULL,                               \
                            &bus_data_peripheral_data_##inst,   \
                            &bus485_config_##inst,              \
                            POST_KERNEL,                  \
                            CUSTOM_BUS485_INIT_PRIORITY,          \
                            NULL);          \


DT_INST_FOREACH_STATUS_OKAY(BUS485_DEFINE)