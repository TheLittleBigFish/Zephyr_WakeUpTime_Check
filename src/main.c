/*
 * Copyright (c) 2012-2014 Wind River Systems, Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <stdio.h>
#include <stdlib.h>
#include <math.h>


#include <zephyr/kernel.h>
// For prints and Logs
#include <zephyr/sys/printk.h>
#include <zephyr/logging/log.h>
// For GPIO
//#include <zephyr/drivers/gpio.h>

// Getting the device pointers
#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/sensor.h>

// For SPI control
#include <zephyr/drivers/spi.h>


LOG_MODULE_REGISTER(Log_Main, LOG_LEVEL_DBG);

/*
 * See https://www.doxygen.nl/manual/commands.html 
 * for coments
 */

/** 
 * @fn char checkDivice(const struct device *dev)
 * @brief Recives a device type pointer and verefies if its value is null
 * or if the device in question has been inicialized.
 * 
 * @param dev a device type pointer.
 * 
 * @retval 0 and dev becomes NULL if the device is fails.
 * @retval 1 if the device passed.
 */
char checkDivice(const struct device *dev) {
	LOG_INF("%s value %d\n", dev->name, dev);
	//printk("dev_lis value %d\n", dev_lis);

	if (dev == NULL) {
		LOG_WRN("%s not found", dev->name);
		dev = NULL;
    	return 0;
	}

	LOG_INF("%s ready values %d, %d\n", dev->name, dev->state->initialized, (dev->state->init_res == 0U));
	
	if (!device_is_ready(dev)) {
		LOG_WRN("%s not ready", dev->name);
		dev = NULL;
    	return 0;
	}

	return 1;
}

int main(void) {
	LOG_INF("Hello World! %s\n", CONFIG_BOARD);
	
	const struct device *dev_bme = DEVICE_DT_GET(DT_NODELABEL(spi_bme280));

	if(!checkDivice(dev_bme)){
		dev_bme = NULL;
		return 0;
	}

	struct spi_dt_spec spec_bme = SPI_DT_SPEC_GET(DT_NODELABEL(spi_bme280), 
			(SPI_TRANSFER_MSB | SPI_MODE_CPOL | SPI_MODE_CPHA | 
			SPI_WORD_SET(8) | SPI_LINES_SINGLE), 0);
	

	while(1){
		uint8_t addr;
		struct spi_buf tx_buf = {.buf = &addr, .len = 1};
		struct spi_buf_set tx = {.buffers = &tx_buf, .count = 1};

		uint8_t buf[2];
		struct spi_buf rx_buf[2] = {{.buf=&buf[0],.len=1},{.buf=&buf[1],.len=1}};
		//struct spi_buf rx_buf = {.buf=&buf[0],.len=1};
		struct spi_buf_set rx = {.buffers = &rx_buf, .count = ARRAY_SIZE(rx_buf)};

		//const struct device *spi = device_get_binding("SPI_0");;

		addr = 0xD0 | 0x80; //| 0x80;

		int ret = spi_transceive_dt(&spec_bme, &tx, &rx);
		if (ret) {
			printk("\n\nspi_transceive_dt ret code: %d", ret);
		}

		/*
		 * Shows buf[0] = 0xff and buf[0] = 0x60 becouse of full-duplex
		 * meaning that the first buf is read at the same time the tx is being written
		 */ 
		printk("\n\nbuff rx(0): %x", buf[0]);
		printk("\nbuff rx(1): %x", buf[1]);

		k_sleep(K_MSEC(5000));
	}

	return 0;
}
