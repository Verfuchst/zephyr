/*
 * SPDX-License-Identifier: Apache-2.0
 */

/*
 * Bus-specific functionality for BNO055s accessed via I2C.
 */

#include "bno055.h"

#if BNO055_BUS_I2C
static int bno055_bus_check_i2c(const struct device *bus,
				const union bno055_bus_config *bus_config)
{
	return device_is_ready(bus) ? 0 : -ENODEV;
}

static int bno055_reg_read_i2c(const struct device *bus,
			       const union bno055_bus_config *bus_config,
			       uint8_t start, uint8_t *buf, int size)
{
	return i2c_burst_read(bus, bus_config->i2c_addr,
			      start, buf, size);
}

static int bno055_reg_write_i2c(const struct device *bus,
				const union bno055_bus_config *bus_config,
				uint8_t reg, uint8_t val)
{
	return i2c_reg_write_byte(bus, bus_config->i2c_addr,
				  reg, val);
}

const struct bno055_bus_io bno055_bus_io_i2c = {
	.check = bno055_bus_check_i2c,
	.read = bno055_reg_read_i2c,
	.write = bno055_reg_write_i2c,
};
#endif	/* BNO055_BUS_I2C */
