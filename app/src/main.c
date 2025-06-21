#include <stdio.h>
#include <zephyr/kernel.h>

#include "bus485.h"

static const int32_t sleep_time_ms = 50;

#define SLEEP_TIME_MS   500

/* The devicetree node identifier for the "led0" alias. */
#define LED0_NODE DT_ALIAS(led0)

/*
 * A build error on this line means your board is unsupported.
 * See the sample documentation for information on how to fix this.
 */
static const struct gpio_dt_spec led = GPIO_DT_SPEC_GET(LED0_NODE, gpios);

static const struct device * bus = DEVICE_DT_GET(DT_NODELABEL(b485));


int main(void){
    int ret;
    if(!device_is_ready(bus)){
        printk("Error: bus are not ready\r\n");
        return -ENODEV;
    }
    
    bool led_state = true;
    
	if (!gpio_is_ready_dt(&led)) {
        return 0;
	}
    
	ret = gpio_pin_configure_dt(&led, GPIO_OUTPUT_ACTIVE);
	if (ret < 0) {
        return 0;
	}
    
    printk("PROJECT start\r\n");

	const struct bus485_driver_api * b485_api = (struct bus485_driver_api*)bus->api;

	uint8_t buf[3] = {0x33, 0x05A, 0x02};
	b485_api->bus485_send(bus, buf, 3);
	while (1) {
		ret = gpio_pin_toggle_dt(&led);
		if (ret < 0) {
			return 0;
		}

		led_state = !led_state;
		printf("LED state: %s\n", led_state ? "ON" : "OFF");
		k_msleep(SLEEP_TIME_MS);

		b485_api->bus485_send(bus, buf, 3);
	}

    return 0;
}