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

// Variable to check the button interrupt 
char checkButtonPress = 0;

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
	LOG_INF("%s value %p\n", dev->name, dev);
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

/** 
 * @fn char configGPIO(const struct gpio_dt_spec *gpio_spec, uint32_t gpio_type)
 * @brief Recives a gpio spec type pointer and verefies if its gpio is ready to use.
 * It also recives a gpio type to configure the gpio's behavior.
 * 
 * @param dgpio_spec a gpio_dt_spec type pointer.
 * @param gpio_type a gpio_flags_t type that defines the gpio behavior
 * 
 * @retval 0 if the device is fails.
 * @retval 1 if the device passed.
 */
char configGPIO(const struct gpio_dt_spec *gpio_spec, gpio_flags_t gpio_type) {
	int err;

	if (!gpio_is_ready_dt(gpio_spec)) {
		printk("The load switch pin GPIO port is not ready.\n");
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
void button_pressed(const struct device *dev, struct gpio_callback *cb, uint32_t pins) {
	printk("Button pressed at %u\n", k_cycle_get_32());
	checkButtonPress = 1;
}


// Control the device read
#define SPI_BME280 1
#define SPI_LIS2DH 2

#define SPI_DEV_NODELABEL SPI_LIS2DH

int main(void) {
	LOG_INF("Hello World! %s\n", CONFIG_BOARD);
	// ---------------------------------- SPI --------------------------------------------------
	// Buffers used in SPI
	uint8_t addr;
	struct spi_buf tx_buf = {.buf = &addr, .len = 1};
	struct spi_buf_set tx = {.buffers = &tx_buf, .count = 1};

	uint8_t buf[2];
	struct spi_buf rx_buf[2] = {{.buf=&buf[0],.len=1},{.buf=&buf[1],.len=1}};
	struct spi_buf_set rx = {.buffers = rx_buf, .count = ARRAY_SIZE(rx_buf)};

	/* SPI configurations
	 * bme280: 
	 * 		addr: 0xD0 | 0x80;
	 * 		responce: 0x60;
	 * 		nodelabel: spi_bme280;
	 * lis2dh: 
	 * 		addr: 0x0F & 0xBF;
	 * 		responce: 0x33;
     * 		nodelabel: spi_lis2;
	 */ 
	#if SPI_DEV_NODELABEL == SPI_BME280
		addr = 0xD0 | 0x80;
		uint8_t buf1_resp = 0x60;
		#define DEV_NODELABEL spi_bme280
	#elif SPI_DEV_NODELABEL == SPI_LIS2DH
		addr = (/*0x0F*/0x27 & 0xBF) | 0x80;
		uint8_t buf1_resp = 0x0F;
		#define DEV_NODELABEL spi_lis2
	#endif

	// Variable with the SPI device specs
	struct spi_dt_spec spec_spi_dev = SPI_DT_SPEC_GET(DT_NODELABEL(DEV_NODELABEL), 
			(SPI_TRANSFER_MSB | SPI_MODE_CPOL | SPI_MODE_CPHA | 
			SPI_WORD_SET(8) | SPI_LINES_SINGLE), 0);
	
	if(!checkDivice(spec_spi_dev.bus)){
		return 0;
	}

	int err;
	/*
	err = lis2dh_spi_init(spec_spi_dev.bus);
	if (err) {
		printk("\n\nspi_transceive_dt err code: %d\n", err);
	}*/

	// ---------------------------------- Interrupt ----------------------------------------------
	// Variable with the nrf52 button 1 device specs
	static const struct gpio_dt_spec button = GPIO_DT_SPEC_GET_OR(DT_ALIAS(sw0), gpios, {0});
	static struct gpio_callback button_cb_data;
	
	// Configures button pin as input
	if(!configGPIO(&button, GPIO_INPUT)){
		return 0;
	}

	// Configures button pin as interrupt
	err = gpio_pin_interrupt_configure_dt(&button, GPIO_INT_EDGE_TO_ACTIVE);
	if (err != 0) {
		printk("Error %d: failed to configure interrupt on %s pin %d\n",
			err, button.port->name, button.pin);
		return 0;
	}

	// Inicializes the callback for the button interrupt
	gpio_init_callback(&button_cb_data, button_pressed, BIT(button.pin));
	gpio_add_callback(button.port, &button_cb_data);

	// ---------------------------------- Pin Control ----------------------------------------------
	// Mosfet specifications
	static const struct gpio_dt_spec mosfet = GPIO_DT_SPEC_GET_OR(DT_NODELABEL(mosfet), gpios, {0});
	const int mosfetOnValue = 1;

	// Configures button pin as output starting inactive (in this case 0) 
	if(!configGPIO(&mosfet, GPIO_OUTPUT_INACTIVE)){
		return 0;
	}

	err = gpio_pin_set_dt(&mosfet, mosfetOnValue);
	if (err != 0) {
		printk("Setting GPIO pin level %d failed: %d\n", mosfetOnValue, err);
		return 0;
	}

	printk("Waiting in loop");
	
	uint32_t start = 0;
	char letSPIPrint = 0;

	// Loop
	while(1){
		// Sends the tx buffer and recives in the rx buffer
		err = spi_transceive_dt(&spec_spi_dev, &tx, &rx);
		if (err) {
			printk("\n\nspi_transceive_dt err code: %d\n", err);
		}

		/*
		 * Shows buf[0] = 0xff and buf[0] = 0x60 becouse of full-duplex
		 * meaning that the first buf is read at the same time the tx is being written
		 */ 
		/*printk("\n\nbuff rx(0): %x", buf[0]);
		printk("\nbuff rx(1): %x", buf[1]);
		k_sleep(K_SECONDS(2));
		continue;*/

		#if SPI_DEV_NODELABEL == SPI_BME280
			if (buf[1] == buf1_resp && letSPIPrint)
		#elif SPI_DEV_NODELABEL == SPI_LIS2DH
			if (((buf[1] & 0x0F) == buf1_resp) && letSPIPrint)
		#endif
		{
			int time_us = k_cyc_to_us_ceil32(k_cycle_get_32() - start);
			printk("Time to ON in us %d\n", time_us);
			letSPIPrint = 0;
		} /*else if(letSPIPrint){
			printk("buff val: %x\n", buf[1]);
		}*/

		if (checkButtonPress){
			// Turns the mosfet off
			err = gpio_pin_set_dt(&mosfet, !mosfetOnValue);
			if (err != 0) {
				printk("Setting GPIO pin level %d failed: %d\n", !mosfetOnValue, err);
				return 0;
			}

			letSPIPrint = 1;

			printk("Mosfet turn off waiting 2 sec\n");
			k_sleep(K_SECONDS(2));

			// Turns the mosfet on
			err = gpio_pin_set_dt(&mosfet, mosfetOnValue);
			if (err != 0) {
				printk("Setting GPIO pin level %d failed: %d\n", mosfetOnValue, err);
				return 0;
			}

			// Gets kernel cycle number
			start = k_cycle_get_32();
			//printk("Mosfet turn on\n");

			checkButtonPress = 0;
		}	
	}

	return 0;
}
