#ifndef MOTOR_DRIVERS_MOTOR_MOTOR_H_H_
#define MOTOR_DRIVERS_MOTOR_MOTOR_H_H_

#include <device.h>
#include <drivers/spi.h>

#define MOTOR_BOARD

#if defined CONFIG_MOTOR_STM32_INPUT_CAPTURE_MODE
#define MOTOR_ARCH_SPECIFIC
#define MOTOR_STM32
#undef MOTOR_BOARD
#define MOTOR_BOARD STM32
#endif /* CONFIG_MOTOR_STM32_INPUT_CAPTURE_MODE */

struct motor_config {
        const struct device *bus;
        const struct spi_config spi_cfg;
#ifdef MOTOR_STM32
        TIM_TypeDef *timer;
#endif
};

struct motor_data {
        struct k_timer timer;
#ifdef MOTOR_ARCH_SPECIFIC
        uint32_t tim_clk;
#endif
};

typedef int (*motor_input_capture_mode_init)(const struct device *tim);

struct motor_input_capture_mode {
        motor_input_capture_mode_init init;
};

#if defined MOTOR_ARCH_SPECIFIC && defined MOTOR_STM32 

#include <soc.h>
#include <stm32_ll_rcc.h>
#include <stm32_ll_tim.h>

#include <drivers/clock_control/stm32_clock_control.h>
#include <pinmux/stm32/pinmux_stm32.h>

struct timer_config {
        /** Timer instance. */
        TIM_TypeDef *timer;
        /** Prescaler */
        uint32_t prescaler;
        /** Clock configuration. */
        struct stm32_pclken pclken;
        /** pinctrl configurations. */
        const struct soc_gpio_pinctrl *pinctrl;
        /** Number of pinctrl configurations. */
        size_t pinctrl_len;
};

extern const struct motor_input_capture_mode motor_input_capture_mode_stm32;

#endif /* MOTOR_ARCH_SPECIFIC && MOTOR_STM32 */

#endif /* MOTOR_DRIVERS_MOTOR_MOTOR_H_H_ */
