/*
 * nrf905_driver.c
 *
 * Copyright (C) 2015 Tero Salminen
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include "nrf905.h"

static int nrf905_major = NRF905_MAJOR;
module_param(nrf905_major, int, S_IRUGO);

static int nrf905_minor = NRF905_MINOR;
module_param(nrf905_minor, int, S_IRUGO);

static struct nrf905_dev_data *nrf905_dev;

static ssize_t nrf905_status_show(struct device *sys_dev,
				  struct device_attribute *attr,
				  char *buf)
{
	ssize_t retval = 0;
	struct nrf905_dev_data *dev = dev_get_drvdata(sys_dev);

	retval += scnprintf(buf + retval, PAGE_SIZE - retval,
		"dev major: %d\n", nrf905_major);

	retval += scnprintf(buf + retval, PAGE_SIZE - retval,
		"dev_recv_buf:\n   ");
	hex_dump_to_buffer(dev->dev_recv_buf,
			NRF905_PAYLOAD_LEN,
			32,
			1,
			buf + retval,
			PAGE_SIZE - retval,
			0);
	retval = strlen(buf);

	retval += scnprintf(buf + retval, PAGE_SIZE - retval,
		"\ndev_send_buf:\n   ");
	hex_dump_to_buffer(dev->dev_send_buf,
			NRF905_PAYLOAD_LEN,
			32,
			1,
			buf + retval,
			PAGE_SIZE - retval,
			0);
	retval = strlen(buf);

	retval += scnprintf(buf + retval, PAGE_SIZE - retval,
		"\nactive_sender: 0x%p\n", dev->active_sender);
	retval += scnprintf(buf + retval, PAGE_SIZE - retval,
		"active_listener: 0x%p\n", dev->active_listener);
	retval += scnprintf(buf + retval, PAGE_SIZE - retval,
		"fp_open: %d\n", atomic_read(&dev->fp_open));
	retval += scnprintf(buf + retval, PAGE_SIZE - retval,
		"spi_dev:\n");
	retval += scnprintf(buf + retval, PAGE_SIZE - retval,
		"    max_speed_hz: %d\n", dev->spi_dev->max_speed_hz);
	retval += scnprintf(buf + retval, PAGE_SIZE - retval,
		"    chip_select: %d\n", dev->spi_dev->chip_select);
	retval += scnprintf(buf + retval, PAGE_SIZE - retval,
		"    bits_per_word: %d\n", dev->spi_dev->bits_per_word);
	retval += scnprintf(buf + retval, PAGE_SIZE - retval,
		"    mode: %d\n", dev->spi_dev->mode);
	retval += scnprintf(buf + retval, PAGE_SIZE - retval,
		"nRF905 chip mode: %d\n", dev->chip.mode);
	retval += scnprintf(buf + retval, PAGE_SIZE - retval,
		"nRF905 chip cd_active: %d\n", dev->chip.cd_active);
	retval += scnprintf(buf + retval, PAGE_SIZE - retval,
		"nRF905 chip configuration registry:\n   ");
	hex_dump_to_buffer(dev->chip.config_reg,
			NRF905_CONFIG_REG_LEN,
			32,
			1,
			buf + retval,
			PAGE_SIZE - retval,
			0);
	retval = strlen(buf);
	retval += scnprintf(buf + retval, PAGE_SIZE - retval,
		"\nnRF905 chip rx payload:\n   ");
	hex_dump_to_buffer(dev->chip.rx_payload,
			NRF905_PAYLOAD_LEN,
			32,
			1,
			buf + retval,
			PAGE_SIZE - retval,
			0);
	retval = strlen(buf);
	retval += scnprintf(buf + retval, PAGE_SIZE - retval,
		"\nnRF905 chip tx payload:\n   ");
	hex_dump_to_buffer(dev->chip.tx_payload,
			NRF905_PAYLOAD_LEN,
			32,
			1,
			buf + retval,
			PAGE_SIZE - retval,
			0);
	retval = strlen(buf);
	retval += scnprintf(buf + retval, PAGE_SIZE - retval,
		"\nnRF905 chip rx addr:\n   ");
	hex_dump_to_buffer(dev->chip.conf.rx_addr,
			NRF905_ADDR_LEN,
			32,
			1,
			buf + retval,
			PAGE_SIZE - retval,
			0);
	retval = strlen(buf);
	retval += scnprintf(buf + retval, PAGE_SIZE - retval,
		"\nnRF905 chip tx addr:\n   ");
	hex_dump_to_buffer(dev->chip.conf.tx_addr,
			NRF905_ADDR_LEN,
			32,
			1,
			buf + retval,
			PAGE_SIZE - retval,
			0);
	retval = strlen(buf);

	retval += scnprintf(buf + retval, PAGE_SIZE - retval,
		"\nSeconds since last received message: %lu\n",
		((long)jiffies - (long)dev->latest_msg) / HZ);

	retval += scnprintf(buf + retval, PAGE_SIZE - retval,
		"\retval: %d\n", retval);

	return retval;
}

static ssize_t nrf905_addr_to_str(char *source, char *buf)
{
	ssize_t retval = 0;
	int i;

	for (i = 0; i < NRF905_ADDR_LEN; i++) {
		retval += scnprintf(buf + retval, PAGE_SIZE, "%#x,",
			source[i]);
	}
	retval += scnprintf(buf + retval, PAGE_SIZE, "\n");

	return retval;
}

static ssize_t nrf905_tx_addr_show(struct device *sys_dev,
				   struct device_attribute *attr,
				   char *buf)
{
	ssize_t retval = 0;
	struct nrf905_dev_data *dev = dev_get_drvdata(sys_dev);

	retval = nrf905_addr_to_str(dev->chip.conf.tx_addr, buf);

	return retval;
}

static ssize_t nrf905_rx_addr_show(struct device *sys_dev,
				   struct device_attribute *attr,
				   char *buf)
{
	ssize_t retval = 0;
	struct nrf905_dev_data *dev = dev_get_drvdata(sys_dev);

	retval = nrf905_addr_to_str(dev->chip.conf.rx_addr, buf);

	return retval;
}

static ssize_t nrf905_str_to_addr(struct device *sys_dev,
				  char *chip_addr,
				  const char *buf,
				  size_t count)
{
	int i;

	/* get_options use first element for lenght info */
	int addr_bytes[NRF905_ADDR_LEN + 1];

	get_options(buf, NRF905_ADDR_LEN + 1, addr_bytes);

	if (addr_bytes[0] != NRF905_ADDR_LEN) {
		dev_err(sys_dev, "Invalid address\n");
		return -EINVAL;
	}

	/* from int to byte */
	for (i = 0; i < NRF905_ADDR_LEN; i++)
		chip_addr[i] = addr_bytes[i+1];

	return count;
}

static ssize_t nrf905_tx_addr_store(struct device *sys_dev,
				    struct device_attribute *attr,
				    const char *buf,
				    size_t count)
{
	ssize_t retval = 0;
	struct nrf905_dev_data *dev = dev_get_drvdata(sys_dev);

	dev_info(sys_dev, "TX addr: %s\n", buf);

	if (mutex_lock_interruptible(&dev->dev_mutex))
		return -EINTR;

	retval = nrf905_str_to_addr(sys_dev, dev->chip.conf.tx_addr, buf,
		count);

	mutex_unlock(&dev->dev_mutex);

	return retval;
}

static ssize_t nrf905_rx_addr_store(struct device *sys_dev,
				    struct device_attribute *attr,
				    const char *buf,
				    size_t count)
{
	ssize_t retval = 0;
	struct nrf905_dev_data *dev = dev_get_drvdata(sys_dev);

	dev_info(sys_dev, "RX addr: %s\n", buf);

	retval = nrf905_str_to_addr(sys_dev, dev->chip.conf.rx_addr, buf,
		count);
	if (retval < 0)
		return retval;

	if (mutex_lock_interruptible(&dev->dev_mutex))
		return -EINTR;

	/*
	 * Operation mode change during rx or tx will likely
	 * cause problems.
	 */
	if (dev->chip.mode != NRF905_POWER_DOWN) {
		dev_err(sys_dev, "Chip busy: %d\n", dev->chip.mode);
		retval = -EBUSY;
		goto out;
	}

	set_nrf905_op_mode(dev, NRF905_SPI_READY);

	/*
	 * Don't chek return value. After driver probe,
	 * trust the communication is ok
	 */
	nrf905_init_chip(dev);

	set_nrf905_op_mode(dev, NRF905_POWER_DOWN);

out:
	mutex_unlock(&dev->dev_mutex);

	return retval;
}

static DEVICE_ATTR(rx_addr, 0664, nrf905_rx_addr_show, nrf905_rx_addr_store);
static DEVICE_ATTR(tx_addr, 0664, nrf905_tx_addr_show, nrf905_tx_addr_store);
static DEVICE_ATTR(status, 0444, nrf905_status_show, NULL);

static struct attribute *chip_config_attrs[] = {
	&dev_attr_rx_addr.attr,
	&dev_attr_tx_addr.attr,
	&dev_attr_status.attr,
	NULL
};

ATTRIBUTE_GROUPS(chip_config);

static ssize_t nrf905_read(struct file *filp,
			   char __user *buf,
			   size_t count,
			   loff_t *f_pos)
{
	ssize_t retval = 0;
//	char devno[20];
	int wait_new_data = 1;

	struct nrf905_dev_data *dev = filp->private_data;

	if (mutex_lock_interruptible(&dev->dev_mutex))
		return -EINTR;

//	dev_info(dev->char_dev, "read: proc: %s, PID: %d, devno: %s, count: %lu, pos: %lld\n",
//		current->comm, current->pid, format_dev_t(devno, dev->cdev.dev),
//		(unsigned long) count, *f_pos);

	/*
	 * For non-blocking and middle of buffer reads,
	 * return whatever is available in recv buffer.
	 */
	if ((filp->f_flags & O_NONBLOCK) || *f_pos > 0)
		wait_new_data = 0;

	/* Don't start listening while TX in progress */
	if (dev->active_sender != NULL && wait_new_data) {
		dev_info(dev->char_dev, "TX active\n");
		retval = -EBUSY;
		goto out;
	}

	/* First client to start listening */
	if (dev->chip.mode != NRF905_RX && wait_new_data) {
		retval = set_nrf905_op_mode(dev, NRF905_RX);
		if (retval < 0)
			goto out;
	}

	dev->active_listener = filp;

	if (dev->chip.mode == NRF905_RX && wait_new_data) {

		mutex_unlock(&dev->dev_mutex);

		/*
		 * nRF905 will raise data ready line,
		 * when new data is available .
		 */
		if (wait_event_interruptible(dev->recv_waitq,
			(dev->chip.mode != NRF905_RX)))
			return -EINTR;

		dev->latest_msg = jiffies;

		if (mutex_lock_interruptible(&dev->dev_mutex))
			return -EINTR;

	}

	/* End of buffer, reset file positions so RX sequence starts again */
	if (*f_pos >= NRF905_PAYLOAD_LEN) {
		*f_pos = 0;
		goto out;
	}

	if (*f_pos + count > NRF905_PAYLOAD_LEN)
		count = NRF905_PAYLOAD_LEN - *f_pos;

//	dev_info(dev->char_dev, "nrf905_read, copy_to_user: count: %lu, pos: %lld\n",
//		(unsigned long) count, *f_pos);

	if (copy_to_user(buf, dev->dev_recv_buf + *f_pos, count)) {
		retval = -EFAULT;
		goto out;
	}

	*f_pos += count;
	retval = count;

out:
	dev->active_listener = NULL;
	mutex_unlock(&dev->dev_mutex);
	return retval;
}

static ssize_t nrf905_write(struct file *filp,
			    const char __user *buf,
			    size_t count, loff_t *f_pos)
{
	ssize_t retval = -EFBIG;
	char devno[20];

	struct nrf905_dev_data *dev = filp->private_data;

	if (mutex_lock_interruptible(&dev->dev_mutex))
		return -EINTR;

	if (dev->chip.mode != NRF905_POWER_DOWN &&
	    dev->chip.mode != NRF905_RX) {
		dev_err(dev->char_dev, "Chip busy\n");
		retval = -EBUSY;
		goto out;
	}

	if (dev->active_sender != NULL && dev->active_sender != filp) {
		dev_err(dev->char_dev, "Only one TX operation at the time\n");
		retval = -EBUSY;
		goto out;
	}

	dev->active_sender = filp;

	dev_info(dev->char_dev, "write: proc: %s, PID: %d, devno: %s, count: %lu, pos: %lld\n",
		current->comm, current->pid, format_dev_t(devno, dev->cdev.dev),
		(unsigned long) count, *f_pos);

	if (*f_pos >= NRF905_PAYLOAD_LEN ||
		count > NRF905_PAYLOAD_LEN ||
		*f_pos + count > NRF905_PAYLOAD_LEN) {
		dev_err(dev->char_dev, "Out of buffer\n");
		goto err;
	}

	/* Clear buffer only when write started at the beginning. */
	if (*f_pos == 0)
		memset(dev->dev_send_buf, 0, NRF905_PAYLOAD_LEN);

	if (copy_from_user(dev->dev_send_buf + *f_pos, buf, count)) {
		retval = -EFAULT;
		goto err;
	}

	*f_pos += count;
	retval = count;

	if (*f_pos == NRF905_PAYLOAD_LEN) {
		retval = set_nrf905_op_mode(dev, NRF905_TX);
		dev->active_sender = NULL;

		/* Start a new msg*/
		*f_pos = 0;
	}

	goto out;

err:
	dev->active_sender = NULL;
out:
	mutex_unlock(&dev->dev_mutex);
	return retval;
}

static int nrf905_open(struct inode *inode, struct file *filp)
{
	int retval = 0;
	struct nrf905_dev_data *dev =
		container_of(inode->i_cdev, struct nrf905_dev_data, cdev);
	char devno[20];
	int fp_open;

	if (mutex_lock_interruptible(&dev->cdev_mutex))
		return -EINTR;

	filp->private_data = dev;

	nonseekable_open(inode, filp);

	fp_open = atomic_inc_return(&dev->fp_open);

	dev_info(dev->char_dev, "open: proc: %s, PID: %d, devno: %s, fp_open: %d\n",
		current->comm, current->pid, format_dev_t(devno, dev->cdev.dev),
		fp_open);

	mutex_unlock(&dev->cdev_mutex);

	return retval;
}

static int nrf905_release(struct inode *inode, struct file *filp)
{
	char devno[20];
	struct nrf905_dev_data *dev = filp->private_data;
	int fp_open;

	if (mutex_lock_interruptible(&dev->cdev_mutex))
		return -EINTR;

	fp_open = atomic_dec_return(&dev->fp_open);

	dev_info(dev->char_dev, "release: proc: %s, PID: %d, devno: %s, fp_open: %d\n",
		current->comm, current->pid, format_dev_t(devno, dev->cdev.dev),
		fp_open);

	if (!fp_open) {
		/* Wait transmission to finish.  */
		if (dev->chip.mode == NRF905_TX)
			usleep_range(20000UL, 22000UL);

		set_nrf905_op_mode(dev, NRF905_POWER_DOWN);

		dev->active_sender = NULL;
		dev->active_listener = NULL;
	}

	mutex_unlock(&dev->cdev_mutex);

	return 0;
}

int set_nrf905_op_mode(struct nrf905_dev_data *dev, enum nrf905_op_mode mode)
{
	int retval = 0;
	unsigned int mode_switch_delay = 0;

	if (mode < 0 || mode > NRF905_TX) {
		dev_err(dev->char_dev, "Invalid Nrf905 mode: %#x\n", mode);
		retval = -EINVAL;
		goto out;
	}

	if (dev->chip.mode == mode)
		goto out;

	/* Delay for nRF905 mode changes */
	if (dev->chip.mode == NRF905_POWER_DOWN)
		mode_switch_delay = 3000; /* 3ms */
	else
		mode_switch_delay = 700; /* 700us */

	if (mode == NRF905_TX) {
		memcpy(dev->chip.tx_payload, dev->dev_send_buf,
			NRF905_PAYLOAD_LEN);

		set_nrf905_op_mode(dev, NRF905_SPI_READY);

		nrf905_set_tx_addr(dev);

		nrf905_write_tx_payload(dev);

		/* check the channel is free before starting tx */
		retval = nrf905_wait_free_channel(dev);
		if (retval < 0) {
			set_nrf905_op_mode(dev, NRF905_POWER_DOWN);
			goto out;
		}
	}

	dev->chip.mode = mode;

	nrf905_set_gpio(dev, mode);

	/* wait chip to change the mode */
	usleep_range(mode_switch_delay, mode_switch_delay + 100);

out:
	return retval;
}

static const struct file_operations nrf905_fops = {
	.owner =	THIS_MODULE,
	.llseek =	no_llseek,
	.read =		nrf905_read,
	.write =	nrf905_write,
	.open =		nrf905_open,
	.release =	nrf905_release,
};

static int nrf905_probe(struct spi_device *spi)
{
	int retval = 0;
	struct nrf905_platform_data *p_data;

	dev_info(&spi->dev, "probe\n");

	retval = spi_setup(spi);
	if (retval < 0) {
		dev_err(&spi->dev, "Can not setup spi device\n");
		retval = -ENODEV;
		goto err;
	}

	spi_set_drvdata(spi, nrf905_dev);

	nrf905_dev->spi_dev = spi;

	p_data = dev_get_platdata(&spi->dev);

	if (!p_data) {
		dev_err(&spi->dev, "No platform data\n");
		retval = -EINVAL;
		goto err;
	}

	retval = nrf905_gpio_config(nrf905_dev, p_data);
	if (retval < 0)
		goto err;

	retval = devm_request_irq(
			nrf905_dev->char_dev,
			nrf905_dev->chip.dataready_irq,
			nrf905_dr_irq,
			IRQF_TRIGGER_RISING,
			"nrf905",
			nrf905_dev);
	if (retval < 0) {
		dev_err(&spi->dev, "Irq request failed\n");
		goto err_irq;
	}

	retval = nrf905_init_chip(nrf905_dev);
	if (retval < 0)
		goto err_init_chip;

	return retval;

err_init_chip:
	devm_free_irq(nrf905_dev->char_dev,
		nrf905_dev->chip.dataready_irq,
		nrf905_dev);
err_irq:
	nrf905_gpio_release(nrf905_dev, dev_get_platdata(&spi->dev));
err:
	return retval;
}

static int nrf905_remove(struct spi_device *spi)
{
	dev_info(&spi->dev, "remove\n");

	devm_free_irq(nrf905_dev->char_dev,
		nrf905_dev->chip.dataready_irq,
		nrf905_dev);

	nrf905_gpio_release(nrf905_dev, dev_get_platdata(&spi->dev));

	return 0;
}

static const struct spi_device_id nrf905_id[] = {
	{ "nrf905", 0 },
	{}
};
MODULE_DEVICE_TABLE(spi, nrf905_id);

static struct spi_driver nrf905_driver = {
	.driver = {
		.name = "nrf905",
		.owner = THIS_MODULE,
	},
	.probe = nrf905_probe,
	.remove = nrf905_remove,
	.id_table = nrf905_id,
};

static int __init nrf905_init(void)
{
	int retval = -ENODEV;
	dev_t devno = 0;
	char cdevno[20];
	struct nrf905_dev_data *dev;

	if (nrf905_major) {
		devno = MKDEV(nrf905_major, nrf905_minor);
		retval = register_chrdev_region(devno, NRF905_NR_DEVS,
			"nrf905");
	} else {
		retval = alloc_chrdev_region(&devno, nrf905_minor, NRF905_NR_DEVS,
			"nrf905");
		nrf905_major = MAJOR(devno);
	}
	if (retval < 0) {
		pr_err("nrf905: can't get major %d\n", nrf905_major);
		goto err_major;
	}

	pr_info("nrf905: %s\n", format_dev_t(cdevno, devno));

	dev = kzalloc(sizeof(*dev), GFP_KERNEL);
	if (IS_ERR_OR_NULL(dev)) {
		pr_err("nrf905: can't allocate memory for nrf905_dev\n");
		retval = -ENOMEM;
		goto err_nrf905_dev;
	}
	nrf905_dev = dev;

	init_waitqueue_head(&dev->recv_waitq);

	mutex_init(&dev->cdev_mutex);
	mutex_init(&dev->dev_mutex);

	spin_lock_init(&dev->chip.irq_lock);

	atomic_set(&nrf905_dev->fp_open, 0);

	dev->nrf905_class = class_create(THIS_MODULE, "nrf905");
	if (IS_ERR(dev->nrf905_class)) {
		pr_err("nrf905: class create fail\n");
		retval = PTR_ERR(dev->nrf905_class);
		goto err_class;
	}

	dev->nrf905_class->dev_groups = chip_config_groups;

	dev->char_dev =
		device_create(dev->nrf905_class, NULL,
			MKDEV(nrf905_major, nrf905_minor), NULL, "nrf905");

	if (IS_ERR(dev->char_dev)) {
		pr_err("nrf905: char device create failed\n");
		retval = PTR_ERR(dev->char_dev);
		goto err_char_dev;
	}

	dev_set_drvdata(dev->char_dev, dev);

	nrf905_dev->work_q = alloc_workqueue("nrf905", 0, 0);
	if (IS_ERR_OR_NULL(nrf905_dev->work_q)) {
		retval = -ENOMEM;
		goto err_workq;
	}

	INIT_WORK(&nrf905_dev->work, nrf905_data_ready);

	pr_info("nrf905: register spi driver\n");
	retval = spi_register_driver(&nrf905_driver);
	if (retval < 0) {
		pr_err("nrf905: spi driver register fail\n");
		goto err_spi_register;
	}

	cdev_init(&dev->cdev, &nrf905_fops);
	dev->cdev.owner = THIS_MODULE;

	retval = cdev_add(&dev->cdev, devno, 1);
	if (retval < 0) {
		pr_err("nrf905: cdev reg. failed, major %d, minor %d: %d\n",
			nrf905_major, nrf905_minor, retval);
		goto err_cdev;
	}

	pr_info("nrf905 ready: %d\n", retval);
	return retval;

err_cdev:
	spi_unregister_driver(&nrf905_driver);

err_spi_register:
	remove_proc_entry("nrf905", NULL);
	device_destroy(dev->nrf905_class,
		MKDEV(nrf905_major, nrf905_minor));

err_workq:
	destroy_workqueue(nrf905_dev->work_q);

err_char_dev:
	class_destroy(dev->nrf905_class);

err_class:
	kfree(dev);

err_nrf905_dev:
	unregister_chrdev_region(devno, NRF905_NR_DEVS);

err_major:
	return retval;
}
module_init(nrf905_init);

static void __exit nrf905_exit(void)
{
	dev_t devno = MKDEV(nrf905_major, nrf905_minor);

	cdev_del(&nrf905_dev->cdev);

	spi_unregister_driver(&nrf905_driver);

	flush_workqueue(nrf905_dev->work_q);
	destroy_workqueue(nrf905_dev->work_q);

	device_destroy(nrf905_dev->nrf905_class,
		MKDEV(nrf905_major, nrf905_minor));

	class_destroy(nrf905_dev->nrf905_class);

	kfree(nrf905_dev);

	unregister_chrdev_region(devno, NRF905_NR_DEVS);

	pr_info("nrf905 exit\n");
}
module_exit(nrf905_exit);

MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("NRF905 driver");
MODULE_AUTHOR("Tero Salminen");
MODULE_VERSION("0.1");
