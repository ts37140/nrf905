#ifndef NRF905_H_INCLUDED
#define NRF905_H_INCLUDED

#include <linux/init.h>
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/cdev.h>
#include <linux/uaccess.h>
#include <linux/proc_fs.h>
#include <mach/platform.h>
#include <linux/gpio/consumer.h>
#include <linux/gpio.h>
#include <linux/delay.h>
#include <linux/spi/spi.h>
#include <linux/interrupt.h>
#include <linux/of_platform.h>
#include <linux/of_gpio.h>

#define NRF905_MAJOR		0
#define NRF905_MINOR		0
#define NRF905_NR_DEVS		1

#define NRF905_GPIOS		6

/* commands defined in nRF905 datasheet */
#define SPI_CMD_NOP		0x0
#define SPI_CMD_W_CONFIG	0x0
#define SPI_CMD_R_CONFIG	0x10
#define SPI_CMD_W_TX_PAYLOAD	0x20
#define SPI_CMD_R_TX_PAYLOAD	0x21
#define SPI_CMD_W_TX_ADDR	0x22
#define SPI_CMD_R_TX_ADDR	0x23
#define SPI_CMD_R_RX_PAYLOAD	0x24

#define NRF905_ADDR_LEN		4
#define NRF905_CONFIG_REG_LEN	10
#define NRF905_PAYLOAD_LEN	32

struct nrf905_platform_data {
	uint8_t gpio_csn; /* SPI chip select */
	uint8_t gpio_dr; /* data ready */
	uint8_t gpio_cd; /* Collision detection */
	uint8_t gpio_pwr; /* Power */
	uint8_t gpio_trxce; /* Radio on/off */
	uint8_t gpio_txen; /* Tx enable/ disable i.e Rx enable */
};

/* nRF905 operation modes*/
enum nrf905_op_mode {
	NRF905_POWER_DOWN,
	NRF905_SPI_READY,
	NRF905_RX,
	NRF905_TX,
};

/* NRF905 configuration */
struct nrf905_chip_conf {
	uint8_t high_freq; /* high freq Pll for 800Mhz & 900Mhz bands */
	uint16_t channel; /* channel no, 9 bits */
	uint8_t pa_pwr; /* output power */
	uint8_t reduce_rx; /* reduce rx sensitivity & current */
	uint8_t auto_retran; /* retransmit tx register */
	uint8_t rx_addrw; /* rx-address width */
	uint8_t tx_addrw; /* tx-address width */
	uint8_t rx_payloadw; /* rx-payload width */
	uint8_t tx_payloadw; /* tx-payload width */
	uint8_t up_clock_freq; /* output clock frequency */
	uint8_t up_clock; /* output clock enable */
	uint8_t xof; /* crystal oscillator frequency */
	uint8_t crc; /* CRC enable */
	uint8_t crc_mode; /* CRC mode */
	uint8_t rx_addr[NRF905_ADDR_LEN]; /* chip address */
	uint8_t tx_addr[NRF905_ADDR_LEN]; /* tx address */
};

struct nrf905_gpios {
	struct gpio_desc *data_ready;
	struct gpio_desc *chipselect;
	struct gpio_desc *trxce;
	struct gpio_desc *chip_pwr;
	struct gpio_desc *txen;
	struct gpio_desc *carrier_detect;
};

struct nrf905_chip_data {
	struct nrf905_chip_conf conf;
	struct gpio_desc *control_gpio[NRF905_GPIOS];
	struct nrf905_gpios gpios;
	uint8_t config_reg[NRF905_CONFIG_REG_LEN];
	uint8_t rx_payload[NRF905_PAYLOAD_LEN];
	uint8_t tx_payload[NRF905_PAYLOAD_LEN];
	spinlock_t irq_lock;
	unsigned int dataready_irq;
	enum nrf905_op_mode mode;
	int cd_active;
};

struct nrf905_dev_data {
	struct nrf905_chip_data		chip;

	struct class			*nrf905_class;

	struct device			*char_dev;
	struct cdev			cdev;
	wait_queue_head_t		recv_waitq;
	struct file			*active_sender;
	struct file			*active_listener;
	uint8_t				dev_recv_buf[NRF905_PAYLOAD_LEN];
	uint8_t				dev_send_buf[NRF905_PAYLOAD_LEN];
	struct mutex			cdev_mutex;
	atomic_t			fp_open;

	struct spi_device		*spi_dev;
	struct workqueue_struct		*work_q;
	struct work_struct		work;
	struct mutex			dev_mutex;

	unsigned long			latest_msg;
};

/* Function prototypes: */
int set_nrf905_op_mode(struct nrf905_dev_data *dev, enum nrf905_op_mode mode);
int nrf905_init_chip(struct nrf905_dev_data *nrf905_dev);
int nrf905_gpio_config(struct nrf905_dev_data *dev);
void nrf905_gpio_release(struct nrf905_dev_data *nrf905_dev);
void nrf905_set_gpio(struct nrf905_dev_data *dev,
		     enum nrf905_op_mode mode);
void nrf905_data_ready(struct work_struct *work);
irqreturn_t nrf905_dr_irq(int irq, void *devptr);
int nrf905_read_chip_rx_buf(struct nrf905_dev_data *dev);
int nrf905_set_tx_addr(struct nrf905_dev_data *dev);
int nrf905_write_tx_payload(struct nrf905_dev_data *dev);
int nrf905_wait_free_channel(struct nrf905_dev_data *dev);

#endif /* NRF905_H_INCLUDED */
