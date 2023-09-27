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
// For GPIO and interrupt
#include <zephyr/drivers/gpio.h>
#include <zephyr/sys/util.h>

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

char configGPIO(const struct gpio_dt_spec *gpio_spec, uint32_t gpio_type) {
	int err;

	if (!gpio_is_ready_dt(gpio_spec)) {
		printf("The load switch pin GPIO port is not ready.\n");
		return 0;
	}

	err = gpio_pin_configure_dt(gpio_spec, gpio_type);
	if (err != 0) {
		printk("Error %d: failed to configure %s pin %d\n", err, gpio_spec->port->name, gpio_spec->pin);
		return 0;
	}

	return 1;
}


/** 
 * @fn void button_pressed(const struct device *dev, struct gpio_callback *cb, uint32_t pins)
 * @brief Callback function for the interrupt resolted from pressing 
 * button 1 (sw0) in the nRF52 PDK board.
 * 
 * @param dev a device type pointer.
 * @param cb callback indicator ???
 * @param pins pin in which the interrupt was originated 
 */
void button_pressed(const struct device *dev, struct gpio_callback *cb, uint32_t pins)
{
	printk("Button pressed at %" PRIu32 "\n", k_cycle_get_32());
}

int main(void) {
	LOG_INF("Hello World! %s\n", CONFIG_BOARD);
	// ---------------------------------- SPI --------------------------------------------------
	// Variable with the bme280 SPI device specs
	struct spi_dt_spec spec_bme = SPI_DT_SPEC_GET(DT_NODELABEL(spi_bme280), 
			(SPI_TRANSFER_MSB | SPI_MODE_CPOL | SPI_MODE_CPHA | 
			SPI_WORD_SET(8) | SPI_LINES_SINGLE), 0);
	
	if(!checkDivice(spec_bme.bus)){
		return 0;
	}

	// Buffers used in SPI
	uint8_t addr;
	struct spi_buf tx_buf = {.buf = &addr, .len = 1};
	struct spi_buf_set tx = {.buffers = &tx_buf, .count = 1};

	uint8_t buf[2];
	struct spi_buf rx_buf[2] = {{.buf=&buf[0],.len=1},{.buf=&buf[1],.len=1}};
	struct spi_buf_set rx = {.buffers = &rx_buf, .count = ARRAY_SIZE(rx_buf)};

	addr = 0xD0 | 0x80; //| 0x80;

	// ---------------------------------- Interrupt ----------------------------------------------
	// Variable with the nrf52 button 1 device specs
	static const struct gpio_dt_spec button = GPIO_DT_SPEC_GET_OR(DT_ALIAS(sw0), gpios, {0});
	static struct gpio_callback button_cb_data;
	
	// Configures button pin as input
	if(!configGPIO(&button, GPIO_INPUT)){
		return 0;
	}

	int ret;

	// Configures button pin as interrupt
	ret = gpio_pin_interrupt_configure_dt(&button, GPIO_INT_EDGE_TO_ACTIVE);
	if (ret != 0) {
		printk("Error %d: failed to configure interrupt on %s pin %d\n",
			ret, button.port->name, button.pin);
		return 0;
	}

	// Inicializes the callback for the button interrupt
	gpio_init_callback(&button_cb_data, button_pressed, BIT(button.pin));
	gpio_add_callback(button.port, &button_cb_data);

	// ---------------------------------- Pin Control ----------------------------------------------
	static const struct gpio_dt_spec mosfet = GPIO_DT_SPEC_GET_OR(DT_NODELABEL(mosfet), gpios, {0});

	// Configures button pin as output starting inactive (in this case 0) 
	if(!configGPIO(&mosfet, GPIO_OUTPUT_INACTIVE)){
		return 0;
	}


	// Loop
	while(1){
		// Sendes the tx buffer and recives in the rx buffer
		int ret = spi_transceive_dt(&spec_bme, &tx, &rx);
		if (ret) {
			printk("\n\nspi_transceive_dt ret code: %d", ret);
		}

		/*
		 * Shows buf[0] = 0xff and buf[0] = 0x60 becouse of full-duplex
		 * meaning that the first buf is read at the same time the tx is being written
		 */ 
		//printk("\n\nbuff rx(0): %x", buf[0]);
		//printk("\nbuff rx(1): %x", buf[1]);

		if (buf[1] == 0x60)
			LOG_DBG("ON");

		ret = gpio_pin_toggle_dt(&mosfet);

		/*
		err = gpio_pin_set_dt(&load_switch, 1);*/
		if (ret != 0) {
			printf("Setting GPIO pin level failed: %d\n", ret);
		}

		k_sleep(K_MSEC(500));
	}

	return 0;
}
