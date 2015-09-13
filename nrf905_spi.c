/*
 * nrf905_spi.c
 *
 * Copyright (C) 2015 Tero Salminen
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include "nrf905.h"

void nrf905_data_ready(struct work_struct *work)
{
	struct nrf905_dev_data *dev =
		container_of(work, struct nrf905_dev_data, work);

	if (mutex_lock_interruptible(&dev->dev_mutex))
		return;

	/*
	 * Ignore data ready pin changes when performing carrier
	 * detect before tx.
	 */
	if (dev->chip.cd_active)
		goto out;

	if (dev->chip.mode == NRF905_RX) {
		set_nrf905_op_mode(dev, NRF905_SPI_READY);
		nrf905_read_chip_rx_buf(dev);
		set_nrf905_op_mode(dev, NRF905_POWER_DOWN);

		memcpy(dev->dev_recv_buf, dev->chip.rx_payload,
			NRF905_PAYLOAD_LEN);
//		dev_info(dev->char_dev, "data ready, message received\n");

		wake_up_interruptible(&dev->recv_waitq);
	}

	if (dev->chip.mode == NRF905_TX) {
		dev_info(dev->char_dev, "TX complete\n");

		/* back to RX as there are active clients listening */
		if (dev->active_listener)
			set_nrf905_op_mode(dev, NRF905_RX);
		else
			set_nrf905_op_mode(dev, NRF905_POWER_DOWN);
	}
out:
	mutex_unlock(&dev->dev_mutex);
}

irqreturn_t nrf905_dr_irq(int irq, void *devptr)
{
	struct nrf905_dev_data *dev = devptr;
	unsigned long flags;

	if (!dev)
		return IRQ_HANDLED;

	spin_lock_irqsave(&dev->chip.irq_lock, flags);

	queue_work(dev->work_q, &dev->work);

	spin_unlock_irqrestore(&dev->chip.irq_lock, flags);

	return IRQ_HANDLED;
}

int nrf905_wait_free_channel(struct nrf905_dev_data *dev)
{
	int retval = 0;
	int i;
	int gpio_high;
	struct nrf905_gpios *gpios = &dev->chip.gpios;

	dev_info(dev->char_dev, "carrier detect\n");

	dev->chip.cd_active = 1;

	set_nrf905_op_mode(dev, NRF905_RX);

	for (i = 0; i < 10; i++) {
		gpio_high = gpiod_get_value(gpios->carrier_detect);

		if (!gpio_high)
			break;
		udelay(100);
	}

	set_nrf905_op_mode(dev, NRF905_SPI_READY);

	dev->chip.cd_active = 0;

	/* channel busy */
	if (gpio_high) {
		dev_err(dev->char_dev, "channel busy\n");
		retval = -EAGAIN;
	}

	return retval;
}

void nrf905_set_gpio(struct nrf905_dev_data *dev,
			     enum nrf905_op_mode mode)
{
	struct nrf905_gpios *gpios = &dev->chip.gpios;

	switch (mode) {
	case NRF905_POWER_DOWN:
//		dev_info(dev->char_dev, "chip NRF905_POWER_DOWN\n");
		gpiod_set_value(gpios->chip_pwr, 0);
		gpiod_set_value(gpios->trxce, 0);
		gpiod_set_value(gpios->txen, 0);
		break;
	case NRF905_SPI_READY:
//		dev_info(dev->char_dev, "chip NRF905_SPI_READY\n");
		gpiod_set_value(gpios->chip_pwr, 1);
		gpiod_set_value(gpios->trxce, 0);
		gpiod_set_value(gpios->txen, 0);
		break;
	case NRF905_RX:
//		dev_info(dev->char_dev, "chip NRF905_RX\n");
		gpiod_set_value(gpios->chip_pwr, 1);
		gpiod_set_value(gpios->trxce, 1);
		gpiod_set_value(gpios->txen, 0);
		break;
	case NRF905_TX:
//		dev_info(dev->char_dev, "chip NRF905_TX\n");
		gpiod_set_value(gpios->chip_pwr, 1);
		gpiod_set_value(gpios->trxce, 1);
		gpiod_set_value(gpios->txen, 1);
		break;
	/* nothing for default */
	}
}

static void nrf905_gpio_release_of(struct nrf905_dev_data *dev)
{
	struct device *spi_dev = &dev->spi_dev->dev;
	struct nrf905_gpios *gpios = &dev->chip.gpios;

	if (gpios->data_ready)
		devm_gpiod_put(spi_dev, gpios->data_ready);
	if (gpios->chipselect)
		devm_gpiod_put(spi_dev, gpios->chipselect);
	if (gpios->trxce)
		devm_gpiod_put(spi_dev, gpios->trxce);
	if (gpios->chip_pwr)
		devm_gpiod_put(spi_dev, gpios->chip_pwr);
	if (gpios->txen)
		devm_gpiod_put(spi_dev, gpios->txen);
	if (gpios->carrier_detect)
		devm_gpiod_put(spi_dev, gpios->carrier_detect);
}

static void nrf905_gpio_release_pdata(struct nrf905_dev_data *dev,
		     struct nrf905_platform_data *p_data)
{
	struct gpio ctrl_gpios[] = {
		{ p_data->gpio_csn, GPIOF_OUT_INIT_HIGH, "NRF905 CS" },
		{ p_data->gpio_dr, GPIOF_IN, "NRF905 DR" },
		{ p_data->gpio_cd, GPIOF_IN, "NRF905 CD" },
		{ p_data->gpio_pwr, GPIOF_OUT_INIT_LOW, "NRF905 PWR" },
		{ p_data->gpio_trxce, GPIOF_OUT_INIT_LOW, "NRF905 TRXCE" },
		{ p_data->gpio_txen, GPIOF_OUT_INIT_LOW, "NRF905 TXEN" },
	};

	gpio_free_array(ctrl_gpios, ARRAY_SIZE(ctrl_gpios));
}

void nrf905_gpio_release(struct nrf905_dev_data *dev)
{
	struct nrf905_platform_data *p_data =
		dev_get_platdata(&dev->spi_dev->dev);

	if (p_data)
		nrf905_gpio_release_pdata(dev, p_data);
	else
		nrf905_gpio_release_of(dev);
}

static int nrf905_gpio_config_of(struct nrf905_dev_data *dev)
{
	int retval = 0;
	struct device *spi_dev = &dev->spi_dev->dev;
	struct nrf905_gpios *gpios = &dev->chip.gpios;

	/*  chip_pwr, trxce & txen will init nRF905 to power down mode */
	gpios->chip_pwr = devm_gpiod_get(spi_dev, "chip_pwr", GPIOD_OUT_LOW);
	if (IS_ERR(gpios->chip_pwr)) {
		retval = PTR_ERR(gpios->chip_pwr);
		gpios->chip_pwr = NULL;
		dev_err(spi_dev, "can't get gpio chip_pwr: %d\n", retval);
		goto err;
	}
	gpios->trxce = devm_gpiod_get(spi_dev, "trxce", GPIOD_OUT_LOW);
	if (IS_ERR(gpios->trxce)) {
		retval = PTR_ERR(gpios->trxce);
		gpios->trxce = NULL;
		dev_err(spi_dev, "can't get gpio trxce: %d\n", retval);
		goto err;
	}
	gpios->txen = devm_gpiod_get(spi_dev, "txen", GPIOD_OUT_LOW);
	if (IS_ERR(gpios->txen)) {
		retval = PTR_ERR(gpios->txen);
		gpios->txen = NULL;
		dev_err(spi_dev, "can't get gpio txen: %d\n", retval);
		goto err;
	}

	gpios->data_ready = devm_gpiod_get(spi_dev, "data_ready", GPIOD_IN);
	if (IS_ERR(gpios->data_ready)) {
		retval = PTR_ERR(gpios->data_ready);
		gpios->data_ready = NULL;
		dev_err(spi_dev, "can't get gpio data_ready: %d\n", retval);
		goto err;
	}

	/* SPI not working with NRF905 if chipselect-pin run in SPI mode*/
	gpios->chipselect = devm_gpiod_get(spi_dev, "chipselect",
		GPIOD_OUT_HIGH);
	if (IS_ERR(gpios->chipselect)) {
		retval = PTR_ERR(gpios->chipselect);
		gpios->chipselect = NULL;
		dev_err(spi_dev, "can't get gpio chipselect: %d\n", retval);
		goto err;
	}

	gpios->carrier_detect = devm_gpiod_get(spi_dev, "carrier_detect",
		GPIOD_IN);
	if (IS_ERR(gpios->carrier_detect)) {
		retval = PTR_ERR(gpios->carrier_detect);
		gpios->carrier_detect = NULL;
		dev_err(spi_dev, "can't get gpio carrier_detect: %d\n", retval);
		goto err;
	}

	return retval;
err:
	nrf905_gpio_release_of(dev);
	return retval;

}

#define GPIO_DESC_ERROR(dev, ctrl, str)	{			\
	if (IS_ERR(ctrl)) {					\
		dev_info(dev, "can not get gpio %s: %ld\n",	\
			str, PTR_ERR(ctrl));			\
		retval = PTR_ERR(ctrl);				\
		goto err;					\
	} }
static int nrf905_gpio_config_pdata(struct nrf905_dev_data *dev,
			      struct nrf905_platform_data *p_data)
{
	int retval = 0;
	struct nrf905_gpios *gpios = &dev->chip.gpios;
	struct device *spi_dev = &dev->spi_dev->dev;

	/* this will init nRF905 to power down mode */
	struct gpio ctrl_gpios[] = {
		{ p_data->gpio_csn, GPIOF_OUT_INIT_HIGH, "NRF905 CS" },
		{ p_data->gpio_dr, GPIOF_IN, "NRF905 DR" },
		{ p_data->gpio_cd, GPIOF_IN, "NRF905 CD" },
		{ p_data->gpio_pwr, GPIOF_OUT_INIT_LOW, "NRF905 PWR" },
		{ p_data->gpio_trxce, GPIOF_OUT_INIT_LOW, "NRF905 TRXCE" },
		{ p_data->gpio_txen, GPIOF_OUT_INIT_LOW, "NRF905 TXEN" },
	};

	retval = gpio_request_array(ctrl_gpios, ARRAY_SIZE(ctrl_gpios));
	if (retval) {
		dev_err(spi_dev, "failed to request GPIOs: %d\n", retval);
		return retval;
	}

	gpios->chip_pwr = gpio_to_desc(p_data->gpio_pwr);
	GPIO_DESC_ERROR(spi_dev, gpios->chip_pwr, "pwr");

	gpios->trxce = gpio_to_desc(p_data->gpio_trxce);
	GPIO_DESC_ERROR(spi_dev, gpios->trxce, "trxce");

	gpios->txen = gpio_to_desc(p_data->gpio_txen);
	GPIO_DESC_ERROR(spi_dev, gpios->txen, "txen");

	gpios->data_ready = gpio_to_desc(p_data->gpio_dr);
	GPIO_DESC_ERROR(spi_dev, gpios->data_ready, "dr");

	gpios->carrier_detect = gpio_to_desc(p_data->gpio_cd);
	GPIO_DESC_ERROR(spi_dev, gpios->carrier_detect, "cd");

	/* SPI communication not working if CS-pin run in SPI mode*/
	gpios->chipselect = gpio_to_desc(p_data->gpio_csn);
	GPIO_DESC_ERROR(spi_dev, gpios->chipselect, "cs");

	return retval;

err:
	gpio_free_array(ctrl_gpios, ARRAY_SIZE(ctrl_gpios));

	return retval;

}

int nrf905_gpio_config(struct nrf905_dev_data *dev)
{
	int retval = 0;
	struct nrf905_platform_data *p_data =
		dev_get_platdata(&dev->spi_dev->dev);
	struct nrf905_gpios *gpios = &dev->chip.gpios;
	struct device *spi_dev = &dev->spi_dev->dev;
	int irq;

	dev_info(spi_dev, "GPIO config\n");

	if (p_data) {
		dev_info(spi_dev, "Init with platformdata\n");
		retval = nrf905_gpio_config_pdata(dev, p_data);
	} else {
		dev_info(spi_dev, "No platform data.\n");

		if (dev->spi_dev->dev.of_node == NULL) {
			dev_err(spi_dev, "Devicetree not found\n");
			retval = -ENODEV;
			goto err;
		}

		dev_info(spi_dev, "Devicetree entry found.\n");

		retval = nrf905_gpio_config_of(dev);
	}

	if (retval < 0)
		goto err;

	irq = gpiod_to_irq(gpios->data_ready);

	if (irq < 0) {
		dev_err(spi_dev, "can not get GPIO irq: %d\n", irq);
		retval = irq;
		goto err;
	}
	dev_info(spi_dev, "dataready irq: %d\n", irq);

	dev->chip.dataready_irq = irq;
err:
	return retval;
}

static void nrf905_set_config_data(struct nrf905_dev_data *dev)
{
	int pos = 0;
	struct nrf905_chip_conf *settings = &dev->chip.conf;
	uint8_t *reg = dev->chip.config_reg;

	/* TODO: some of this could be moved to platform data */
	settings->high_freq = 0; /* 433Mhz freq*/
	settings->channel = 0x6c; /* 433.3 Mhz*/
	settings->pa_pwr = 0x3; /* +10 dBm */
	settings->reduce_rx = 0;
	settings->auto_retran = 0;
	settings->rx_addrw = 0x4;
	settings->tx_addrw = 0x4;
	settings->rx_payloadw = 0x20; /* 32-bit */
	settings->tx_payloadw = 0x20; /* 32-bit */
	settings->up_clock_freq = 0x2;
	settings->up_clock = 0;
	settings->xof = 0x3;
	settings->crc = 1;
	settings->crc_mode = 1; /* 16-bit */

	memset(reg, 0, NRF905_CONFIG_REG_LEN);

	/* Byte 0, first 8-bits from channel */
	reg[pos++] = (uint8_t) settings->channel;

	/* Byte 1 */

	/* channel 9. bit*/
	reg[pos] |= (uint8_t)((uint16_t)(settings->channel & 0x100) >> 8);

	reg[pos] |= (settings->high_freq << 1);
	reg[pos] |= (settings->pa_pwr << 2);
	reg[pos] |= (settings->reduce_rx << 4);
	reg[pos++] |= (settings->auto_retran << 5);

	/* Byte 2 */
	reg[pos] |= (settings->rx_addrw);
	reg[pos++] |= (settings->tx_addrw << 4);

	/* Byte 3..8 */
	reg[pos++] |= (settings->rx_payloadw);
	reg[pos++] |= (settings->tx_payloadw);
	reg[pos++] |= (settings->rx_addr[0]);
	reg[pos++] |= (settings->rx_addr[1]);
	reg[pos++] |= (settings->rx_addr[2]);
	reg[pos++] |= (settings->rx_addr[3]);

	/* Byte 9 */
	reg[pos] |= (settings->up_clock_freq);
	reg[pos] |= (settings->up_clock << 2);
	reg[pos] |= (settings->xof << 3);
	reg[pos] |= (settings->crc << 6);

	reg[pos] |= (settings->crc_mode << 7);

	dev_info(dev->char_dev, "chip configuration:\n");
	print_hex_dump_bytes("", DUMP_PREFIX_NONE, reg, NRF905_CONFIG_REG_LEN);
}

static int nrf905_alloc_spi_buffers(struct nrf905_dev_data *dev,
				    struct spi_transfer *spi_t,
				    int len)
{
	int retval  = 0;

	spi_t->rx_buf = devm_kzalloc(dev->char_dev, len, GFP_KERNEL);
	if (!spi_t->rx_buf) {
		retval = -ENOMEM;
		return retval;
	}

	spi_t->tx_buf = devm_kzalloc(dev->char_dev, len, GFP_KERNEL);
	if (!spi_t->tx_buf) {
		devm_kfree(dev->char_dev, spi_t->rx_buf);
		retval = -ENOMEM;
	}

	return retval;
}


static void nrf905_free_spi_buffers(struct nrf905_dev_data *dev,
				    struct spi_transfer *spi_t)
{
	if (spi_t == NULL)
		return;

	if (spi_t->rx_buf != NULL)
		devm_kfree(dev->char_dev, (uint8_t *)spi_t->rx_buf);

	if (spi_t->tx_buf != NULL)
		devm_kfree(dev->char_dev, (uint8_t *)spi_t->tx_buf);
}

static int nrf905_spi_send_cmd(struct nrf905_dev_data *dev,
			       uint8_t cmd,
			       int len,
			       uint8_t *rx_data,
			       uint8_t *tx_data)
{
	int retval = 0;
	struct spi_transfer spi_t;
	struct spi_message msg;
	struct nrf905_gpios *gpios = &dev->chip.gpios;

	if (len > NRF905_PAYLOAD_LEN)
		len = NRF905_PAYLOAD_LEN;

	memset(&spi_t, 0x0, sizeof(spi_t));

	retval = nrf905_alloc_spi_buffers(dev, &spi_t,
		1 + len);
	if (retval < 0)
		goto out;

	/* command byte + data */
	spi_t.len = 1 + len;

	((uint8_t *)spi_t.tx_buf)[0] = cmd;

	if (tx_data != NULL)
		memcpy(&((uint8_t *)spi_t.tx_buf)[1], tx_data, len);

	spi_message_init(&msg);
	spi_message_add_tail(&spi_t, &msg);

	gpiod_set_value(gpios->chipselect, 0);

	retval = spi_sync(dev->spi_dev, &msg);
	if (retval < 0) {
		dev_err(dev->char_dev, "spi send error: %d\n", retval);
		goto err;
	}

	gpiod_set_value(gpios->chipselect, 1);

//	dev_info(dev->char_dev, "spi transfer buffers:\n");
//	print_hex_dump_bytes("rx:", DUMP_PREFIX_NONE, spi_t.rx_buf, spi_t.len);
//	print_hex_dump_bytes("tx:", DUMP_PREFIX_NONE, spi_t.tx_buf, spi_t.len);

	/* first byte is status register, not part of received data */
	if (rx_data != NULL)
		memcpy(rx_data, &((uint8_t *)spi_t.rx_buf)[1],
			NRF905_PAYLOAD_LEN);

err:
	nrf905_free_spi_buffers(dev, &spi_t);
out:
	return retval;
}

static int nrf905_read_and_verify_chip_reg(struct nrf905_dev_data *dev)
{
	int retval = 0;
	uint8_t *reg = dev->chip.config_reg;

	dev_info(dev->char_dev, "verify chip configuration register and SPI communication\n");

	memset(dev->chip.rx_payload, 0, NRF905_PAYLOAD_LEN);

	retval = nrf905_spi_send_cmd(dev,
				      SPI_CMD_R_CONFIG,
				      NRF905_CONFIG_REG_LEN,
				      dev->chip.rx_payload,
				      NULL);

	/*
	 * Configuration register content read from chip and
	 * cached data should match. Otherwise there is some
	 * problem in spi communication.
	 */
	retval = memcmp(reg, dev->chip.rx_payload, NRF905_CONFIG_REG_LEN);
	if (retval) {
		dev_err(dev->char_dev, "spi communication fails: %d\n", retval);
		retval = -EIO;
	}

	return retval;
}

static int nrf905_configure_chip(struct nrf905_dev_data *dev)
{
	int retval = 0;
	uint8_t *reg = dev->chip.config_reg;

	memset(dev->chip.rx_payload, 0, NRF905_PAYLOAD_LEN);

	retval = nrf905_spi_send_cmd(dev,
				      SPI_CMD_W_CONFIG,
				      NRF905_CONFIG_REG_LEN,
				      NULL,
				      reg);

	return retval;
}

int nrf905_init_chip(struct nrf905_dev_data *dev)
{
	int retval  = 0;

	dev_info(dev->char_dev, "init chip\n");
	/* Chip mode is power down after GPIO init, no need to change */

	/* build up config reg content from settings */
	nrf905_set_config_data(dev);

	/* send reg content to chip*/
	retval = nrf905_configure_chip(dev);
	if (retval < 0)
		goto out;

	retval = nrf905_read_and_verify_chip_reg(dev);
	if (retval < 0)
		goto out;

out:
	return retval;
}

int nrf905_read_chip_rx_buf(struct nrf905_dev_data *dev)
{
	int retval = 0;

//	dev_info(dev->char_dev, "read rx payload register\n");

	retval = nrf905_spi_send_cmd(dev,
				      SPI_CMD_R_RX_PAYLOAD,
				      NRF905_PAYLOAD_LEN,
				      dev->chip.rx_payload,
				      NULL);

	return retval;
}


int nrf905_set_tx_addr(struct nrf905_dev_data *dev)
{
	int retval = 0;

	retval = nrf905_spi_send_cmd(dev,
				      SPI_CMD_W_TX_ADDR,
				      NRF905_ADDR_LEN,
				      NULL,
				      dev->chip.conf.tx_addr);

	return retval;
}

int nrf905_write_tx_payload(struct nrf905_dev_data *dev)
{
	int retval = 0;

	dev_info(dev->char_dev, "write tx payload register\n");

	retval = nrf905_spi_send_cmd(dev,
				      SPI_CMD_W_TX_PAYLOAD,
				      NRF905_PAYLOAD_LEN,
				      NULL,
				      dev->chip.tx_payload);

	return retval;
}
