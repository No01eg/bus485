#ifndef ZEPHYR_DRIVERS_BUS485_H_
#define ZEPHYR_DRIVERS_BUS485_H_

#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/uart.h>

#define QUEUE_SIZE CONFIG_CUSTOM_BUS485_QUEUE_SIZE

//predefined api
struct bus485_driver_api {
    int32_t (*bus485_lock)(const struct device * dev);

    int32_t (*bus485_release)(const struct device * dev);

    int32_t (*bus485_send)(const struct device * dev,
                           const uint8_t * buffer,
                           uint32_t count);

    int32_t (*bus485_recv)(const struct device * dev,
                           const uint8_t * buffer,
                           uint32_t buffer_size,
                           uint32_t timeout_ms);

    int32_t (*bus485_flush)(const struct device * dev);

    int32_t (*bus485_set_baudrate)(const struct device * dev,
                                   uint32_t baudrate);
};



#endif /* ZEPHYR_DRIVERS_BUS485_H_ */