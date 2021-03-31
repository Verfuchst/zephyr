#ifndef MOTOR_DRIVERS_MOTOR_MOTOR_H_H_
#define MOTOR_DRIVERS_MOTOR_MOTOR_H_H_

#include <device.h>
#include <drivers/spi.h>


#if defined CONFIG_MOTOR_STM32_INPUT_CAPTURE_MODE
#define MOTOR_ARCH_SPECIFIC
#define MOTOR_STM32
#endif /* CONFIG_MOTOR_STM32_INPUT_CAPTURE_MODE */

typedef int (*motor_input_capture_mode_init)(const struct device *tim);

struct motor_input_capture_mode {
        motor_input_capture_mode_init init;
};

#if defined MOTOR_ARCH_SPECIFIC && defined MOTOR_STM32 

#include <soc.h>
#include <stm32_ll_rcc.h>
#include <stm32_ll_tim.h>
#include <kernel.h>

#include <drivers/clock_control/stm32_clock_control.h>
#include <pinmux/stm32/pinmux_stm32.h>


struct timer_config {
        /** Timer instance. */
        TIM_TypeDef *timer;
        /** Clock configuration. */
        struct stm32_pclken pclken;
        /** pinctrl configurations. */
        const struct soc_gpio_pinctrl *pinctrl;
        size_t pinctrl_len;
};

extern const struct motor_input_capture_mode motor_input_capture_mode_stm32;

#endif /* MOTOR_ARCH_SPECIFIC && MOTOR_STM32 */

struct motor_config {
        const struct device *bus;
        const struct spi_config spi_cfg;
        /* number of connected motors */
        const int8_t chain_length;
#ifdef MOTOR_ARCH_SPECIFIC 
        struct timer_config timer;
        const struct motor_input_capture_mode *ic_mode;
#endif
};

struct motor_data {
        struct k_timer timer;
#ifdef MOTOR_ARCH_SPECIFIC
        /** Timer clock (Hz). */
        uint32_t tim_clk;
#endif
};



#endif /* MOTOR_DRIVERS_MOTOR_MOTOR_H_H_ */
