# nrf905
Driver for Nordic Semiconductor nRF905 transceiver.

This is out-of-tree kernel module. It uses SPI bus and 6 GPIOs for communicating with nRF905 chip. 

For receiving and sending messages it creates character device /dev/nrf905.

Runtime configuration can be set via sysfs files:

/sys/class/nrf905/nrf905/rx_addr: nRF905 listen messages send to this address.

/sys/class/nrf905/nrf905/tx_addr: messages are sent to this address.

/sys/class/nrf905/nrf905/status: print debugging status information.

I didn't get SPI communication working if I run chipselect in SPI mode (alt0). SPI communication began to work when CS pin is defined for output and controlled separately in the code.

I've tested the driver with Raspbian Debian Wheezy (Release date: 2015-05-05) with kernel versions 3.18 and 4.1.6.

## Platformdata and Device tree
Driver reads GPIO configuration from platform data (nrf905_platform_data) or from device tree entry (e.g. nrf905-overlay.dts).
