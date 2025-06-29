#include <stdio.h>
#include <zephyr/kernel.h>


#include <zephyr/logging/log.h>
#include "bus485.h"


//Stack size set
#define TH1_THREAD_STACK_SIZE 512
#define TH2_THREAD_STACK_SIZE 512


K_THREAD_STACK_DEFINE(th1_stack, TH1_THREAD_STACK_SIZE);
static struct k_thread th1_thread;

K_THREAD_STACK_DEFINE(th2_stack, TH2_THREAD_STACK_SIZE);
static struct k_thread th2_thread;

static const struct device * bus = DEVICE_DT_GET(DT_NODELABEL(b485));

void th1_thread_start(void *arg_1, void *arg_2, void *arg_3){
	int ret;
	uint8_t buf_req[8] = {0x01, 0x03, 0x00, 0x00, 0x00, 0x0a, 0xc5, 0xcd};
	uint8_t buff_resp[40];
	const struct bus485_driver_api * b485_api = (struct bus485_driver_api*)bus->api;

	while(1){
		b485_api->bus485_lock(bus);
		b485_api->bus485_set_baudrate(bus, 9600);
		b485_api->bus485_flush(bus);
		b485_api->bus485_send(bus, buf_req, 8);
		printk("wait receive\r\n\r\n\r\n");
		ret = b485_api->bus485_recv(bus, buff_resp, 40, 5000);
		if(ret < 0){
			printk("receive timeout\r\n");
		}
		else{
			printk("\r\npack rcv with %d bytes: ", ret);
			if(ret > 0){
				uint8_t i;
				for(i = 0; i < ret; ++i)
					printk("%#2x ", buff_resp[i]);
				printk("\r\n");
			}
		}

		b485_api->bus485_release(bus);

		k_msleep(1000);
	}
}

void th2_thread_start(void *arg_1, void *arg_2, void *arg_3){
	int ret;
	uint8_t buf_req[8] = {0x02, 0x01, 0x00, 0x00, 0x00, 0x0a, 0xc5, 0xcd};
	uint8_t buff_resp[40];
	const struct bus485_driver_api * b485_api = (struct bus485_driver_api*)bus->api;

	while(1){
		b485_api->bus485_lock(bus);
		b485_api->bus485_set_baudrate(bus, 9600);
		b485_api->bus485_flush(bus);
		b485_api->bus485_send(bus, buf_req, 8);
		printk("wait receive\r\n\r\n\r\n");
		ret = b485_api->bus485_recv(bus, buff_resp, 40, 5000);
		if(ret < 0){
			printk("receive timeout\r\n");
		}
		else{
			printk("\r\npack rcv with %d bytes: ", ret);
			if(ret > 0){
				uint8_t i;
				for(i = 0; i < ret; ++i)
					printk("%#2x ", buff_resp[i]);
				printk("\r\n");
			}
		}

		b485_api->bus485_release(bus);

		k_msleep(3000);
	}
}


/* The devicetree node identifier for the "led0" alias. */

/*
 * A build error on this line means your board is unsupported.
 * See the sample documentation for information on how to fix this.
 */



int main(void){
    int ret;
	k_tid_t th1_tid;
	k_tid_t th2_tid;
    if(!device_is_ready(bus)){
        printk("Error: bus are not ready\r\n");
        return -ENODEV;
    }
    
    
    printk("PROJECT start\r\n");

	th1_tid = k_thread_create(&th1_thread,
							  th1_stack,
							  K_THREAD_STACK_SIZEOF(th1_stack),
							  th1_thread_start,
							  NULL,
							  NULL,
							  NULL,
							  7, 
							  0,
							  K_NO_WAIT);

	th2_tid = k_thread_create(&th2_thread,
							  th2_stack,
							  K_THREAD_STACK_SIZEOF(th2_stack),
							  th2_thread_start,
							  NULL,
							  NULL,
							  NULL,
							  7, 
							  0,
							  K_NO_WAIT);
	
	while(1){
		k_msleep(2000);
	}

    return 0;
}