#include <stdio.h>
#include <zephyr/kernel.h>


#include <zephyr/logging/log.h>
#include "bus485.h"

LOG_MODULE_REGISTER(app);
//Stack size set
#define TH1_THREAD_STACK_SIZE 512
#define TH2_THREAD_STACK_SIZE 512
#define TH3_THREAD_STACK_SIZE 512

K_THREAD_STACK_DEFINE(th1_stack, TH1_THREAD_STACK_SIZE);
static struct k_thread th1_thread;

K_THREAD_STACK_DEFINE(th2_stack, TH2_THREAD_STACK_SIZE);
static struct k_thread th2_thread;

K_THREAD_STACK_DEFINE(th3_stack, TH3_THREAD_STACK_SIZE);
static struct k_thread th3_thread;

static const struct device * bus = DEVICE_DT_GET(DT_NODELABEL(b485));

static const struct device * bus_1 = DEVICE_DT_GET(DT_NODELABEL(b485_1));


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
		LOG_DBG("TH1 WAIT ANSWER\r\n");
		ret = b485_api->bus485_recv(bus, buff_resp, 40, 5000);
		if(ret < 0){
			LOG_DBG("TH1 receive timeout\r\n");
		}
		else{

			LOG_DBG("TH1 pack rcv with %d bytes\r\n", ret);
			if(ret > 0){
				LOG_HEXDUMP_INF(buff_resp, ret, "Rcv buff");   
			}
		}

		b485_api->bus485_release(bus);

		k_msleep(1000);
	}
}

void th2_thread_start(void *arg_1, void *arg_2, void *arg_3){
	int ret;
	uint8_t buf_req[8] = {0x02, 0x01, 0x00, 0x00, 0x00, 0x0a, 0xbc, 0x3e};
	uint8_t buff_resp[40];
	const struct bus485_driver_api * b485_api = (struct bus485_driver_api*)bus->api;

	while(1){
		b485_api->bus485_lock(bus);
		b485_api->bus485_set_baudrate(bus, 9600);
		b485_api->bus485_flush(bus);
		b485_api->bus485_send(bus, buf_req, 8);
		LOG_DBG("TH2 WAIT ANSWER\r\n");
		ret = b485_api->bus485_recv(bus, buff_resp, 40, 5000);
		if(ret < 0){
			LOG_DBG("TH2 receive timeout\r\n");
		}
		else{
			LOG_DBG("TH2 pack rcv with %d bytes\r\n", ret);
			if(ret > 0){
				LOG_HEXDUMP_INF(buff_resp, ret, "Rcv buff");   
			}
		}

		b485_api->bus485_release(bus);

		k_msleep(3000);
	}
}

void th3_thread_start(void *arg_1, void *arg_2, void *arg_3){
	int ret;
	uint8_t buf_req[8] = {0x01, 0x03, 0x00, 0x00, 0x00, 0x0a, 0xc5, 0xcd};
	uint8_t buff_resp[40];
	const struct bus485_driver_api * b485_api = (struct bus485_driver_api*)bus_1->api;

	while(1){
		b485_api->bus485_lock(bus_1);
		b485_api->bus485_set_baudrate(bus_1, 9600);
		b485_api->bus485_flush(bus_1);
		b485_api->bus485_send(bus_1, buf_req, 8);
		LOG_DBG("TH3 WAIT ANSWER\r\n");
		ret = b485_api->bus485_recv(bus_1, buff_resp, 40, 5000);
		if(ret < 0){
			LOG_DBG("TH3 receive timeout\r\n");
		}
		else{

			LOG_DBG("TH3 pack rcv with %d bytes\r\n", ret);
			if(ret > 0){
				LOG_HEXDUMP_INF(buff_resp, ret, "Rcv buff");   
			}
		}

		b485_api->bus485_release(bus_1);

		k_msleep(1000);
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
	k_tid_t th3_tid;
    if(!device_is_ready(bus)){
        printk("Error: bus are not ready\r\n");
        return -ENODEV;
    }
    
	if(!device_is_ready(bus_1)){
        printk("Error: bus_1 are not ready\r\n");
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
			
	th3_tid = k_thread_create(&th3_thread,
							  th3_stack,
							  K_THREAD_STACK_SIZEOF(th3_stack),
							  th3_thread_start,
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