/* motor.c - Driver for MOTOR vibration */

#include <kernel.h>
#include <init.h>
#include <drivers/gpio.h>
#include <drivers/motor.h>
#include <drivers/spi.h>
#include <device.h>
#include <zephyr/types.h>
#include <devicetree.h>
#include "motor_h.h"

#include <logging/log.h>

#define DT_DRV_COMPAT thm_motor

LOG_MODULE_REGISTER(MOTOR, CONFIG_MOTOR_LOG_LEVEL);

#if DT_NUM_INST_STATUS_OKAY(DT_DRV_COMPAT) == 0
#warning "MOTOR driver enabled without any devices"
#endif

/* SPI BUS CONFIGS */
#define MOTOR_SPI_OPERATION (SPI_WORD_SET(8) | SPI_TRANSFER_MSB)

static inline struct motor_data *to_data(const struct device *dev)
{
	return dev->data;
}

static inline const struct motor_config *to_config(const struct device *dev)
{
	return dev->config;
}

static inline const struct device *to_bus(const struct device *dev)
{
	return to_config(dev)->bus;
}

static inline const struct spi_config *to_bus_config(const struct device *dev)
{
	return &to_config(dev)->spi_cfg;
}

#ifdef MOTOR_ARCH_SPECIFIC

static inline const struct timer_config *to_tim_cfg(const struct device *dev)
{
	return &to_config(dev)->timer;
}

static inline int motor_tim_init(const struct device *dev)
{
	return to_config(dev)->ic_mode->init(dev, to_tim_cfg(dev));
}

#endif

static int motor_write_spi(const struct device *bus, uint8_t *cmd)
{
	/* Init buffer with 0 */
	memset(cmd, 0, FRAMES);

	struct spi_buf tx_buf = {
		.buf = cmd,
		.len = FRAMES
	};

	const struct spi_buf_set tx = {
		.buffers = &tx_buf,
		.count = 1
	};

	uint8_t ret = 0;
	ret = spi_write(to_bus(bus), to_bus_config(bus), &tx);
	if (ret) {
		LOG_DBG("spi_write FAIL %d\n", ret);
		return ret;
	}

	return 0;
}

static int motor_set_sens(const struct device *dev,
			  const uint16_t motor,
			  const uint8_t sensitivity)
{
	struct motor_data *data = to_data(dev);
	uint8_t *cmd = data->cmd;

	for (int i = 0; i < sensitivity; i++) {
		cmd[i] = motor;
	}
	return 0;
}


static const struct motor_driver_api motor_api_funcs = {
	.set_sensitivity = motor_set_sens,
};

static int motor_bus_check_spi(const struct device *bus, const struct spi_config *spi_cfg)
{
	const struct spi_cs_control *cs;

	if (!device_is_ready(bus)) {
		LOG_DBG("SPI bus %s not ready", bus->name);
		return -ENODEV;
	}

	cs = spi_cfg->cs;
	if (cs && !device_is_ready(cs->gpio_dev)) {
		LOG_DBG("SPI CS GPIO controller %s not ready",
			cs->gpio_dev->name);
		return -ENODEV;
	}

	return 0;

}

static int motor_init(const struct device *dev)
{
	struct motor_data *data = to_data(dev);
	int err = 0;

	LOG_DBG("initializing \"%s\" on bus \"%s\"",
		dev->name, to_bus(dev)->name);

	err = motor_bus_check_spi(to_bus(dev), to_bus_config(dev));
	if (err != 0) {
		LOG_DBG("bus check failed: %d", err);
		return err;
	}

#ifdef MOTOR_ARCH_SPECIFIC
	err = motor_tim_init(dev);
	if (err != 0) {
		LOG_DBG("timer init failed: %d", err);
		return err;
	}
#endif

	/* Setups circular mode */
	motor_write_spi(dev, data->cmd);
	if (err != 0) {
		LOG_DBG("DMA failed: %d", err);
		return err;
	}
	LOG_DBG("\"%s\" OK", dev->name);
	return 0;
}


#define DT_INST_CLK(inst)						 \
	{								 \
		.bus = DT_CLOCKS_CELL(DT_INST_PHANDLE(inst, tim), bus),	 \
		.enr = DT_CLOCKS_CELL(DT_INST_PHANDLE(inst, tim), bits), \
	}

/* Initializes a struct motor_config for an instance on a SPI bus. */
#define MOTOR_CONFIG_SPI(inst)					   \
	{							   \
		.bus = DEVICE_DT_GET(DT_INST_BUS(inst)),	   \
		.spi_cfg = SPI_CONFIG_DT_INST(inst,		   \
					      MOTOR_SPI_OPERATION, \
					      0),		   \
	}


#define MOTOR_CONFIG_SPI_TIMER_STM32(inst)			    \
	{							    \
		.bus = DEVICE_DT_GET(DT_INST_BUS(inst)),	    \
		.spi_cfg = SPI_CONFIG_DT_INST(inst,		    \
					      MOTOR_SPI_OPERATION,  \
					      0),		    \
		.timer = {					    \
			.timer = (TIM_TypeDef *)DT_REG_ADDR(	    \
				DT_INST_PHANDLE(inst, tim)),	    \
			.pclken = DT_INST_CLK(inst),		    \
			.pinctrl = pwm_pins_##inst,		    \
			.pinctrl_len = ARRAY_SIZE(pwm_pins_##inst), \
		},						    \
		.ic_mode = &motor_input_capture_mode_stm32,	    \
		.chain_length = DT_INST_PROP(inst, chain_length),   \
	}							    \

/* Concat strings e.g. MOTOR_CONFIG_SPI_TIMER_##STM32 */
#define CC_BOARD(MACRO, BOARD) \
	MACRO ## BOARD

/* Expands board macro e.g. board -> stm32 */
#define EXPAND_BOARD(MACRO, BOARD) \
	CC_BOARD(MACRO, BOARD)


#define MOTOR_CONFIG_SPI_TIMER_BOARD(board) \
	EXPAND_BOARD(MOTOR_CONFIG_SPI_TIMER_, board)

/*
 *  Main instantiation macro.
 *  TODO: Replace ST_STM32_DT_INST_PINCTR Macro with abstract one for each board
 */
#define MOTOR_DEFINE(inst)						      \
	static struct motor_data motor_data_##inst = { .cmd = { 0 } };	      \
	COND_CODE_1(MOTOR_ARCH_SPECIFIC,				      \
		    (static const struct soc_gpio_pinctrl pwm_pins_##inst[] = \
			     ST_STM32_DT_INST_PINCTRL(inst, 0)),	      \
		    ());						      \
	static const struct motor_config motor_config_##inst =		      \
		COND_CODE_1(MOTOR_ARCH_SPECIFIC,			      \
			    (MOTOR_CONFIG_SPI_TIMER_BOARD(BOARD)(inst)),      \
			    (MOTOR_CONFIG_SPI(inst)));			      \
	DEVICE_DT_INST_DEFINE(inst,					      \
			      motor_init,				      \
			      device_pm_control_nop,			      \
			      &motor_data_##inst,			      \
			      &motor_config_##inst,			      \
			      POST_KERNEL,				      \
			      CONFIG_MOTOR_INIT_PRIORITY,		      \
			      &motor_api_funcs);

DT_INST_FOREACH_STATUS_OKAY(MOTOR_DEFINE)
