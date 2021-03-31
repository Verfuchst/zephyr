#include "stm32l4xx_ll_tim.h"
#include <logging/log.h>
#include "motor_h.h"


#ifdef MOTOR_STM32

#if defined(CONFIG_SOC_SERIES_STM32F3X) ||                                     \
        defined(CONFIG_SOC_SERIES_STM32F7X) ||                                 \
        defined(CONFIG_SOC_SERIES_STM32G0X) ||                                 \
        defined(CONFIG_SOC_SERIES_STM32G4X) ||                                 \
        defined(CONFIG_SOC_SERIES_STM32H7X) ||                                 \
        defined(CONFIG_SOC_SERIES_STM32L4X) ||                                 \
        defined(CONFIG_SOC_SERIES_STM32MP1X) ||                                \
        defined(CONFIG_SOC_SERIES_STM32WBX)
#define TIMER_HAS_6CH 1
#else
#define TIMER_HAS_6CH 0
#endif

#if TIMER_HAS_6CH
#define TIMER_MAX_CH 6u
#else                                                    
#define TIMER_MAX_CH 4u
#endif   

/** Channel to LL mapping. */
static const uint32_t ch2ll[TIMER_MAX_CH] = {
        LL_TIM_CHANNEL_CH1, LL_TIM_CHANNEL_CH2,
        LL_TIM_CHANNEL_CH3, LL_TIM_CHANNEL_CH4,
#if TIMER_HAS_6CH
        LL_TIM_CHANNEL_CH5, LL_TIM_CHANNEL_CH6
#endif
};

/** Channel to compare set function mapping. */
static void (*const set_timer_compare[TIMER_MAX_CH])(TIM_TypeDef *,
                uint32_t) = {
        LL_TIM_OC_SetCompareCH1, LL_TIM_OC_SetCompareCH2,
        LL_TIM_OC_SetCompareCH3, LL_TIM_OC_SetCompareCH4,
#if TIMER_HAS_6CH
        LL_TIM_OC_SetCompareCH5, LL_TIM_OC_SetCompareCH6
#endif
};


LOG_MODULE_DECLARE(MOTOR, CONFIG_MOTOR_LOG_LEVEL);

static inline const struct timer_config *to_tim_cfg(const struct device *dev)
{
        return &((const struct motor_config*)dev->config)->timer;
}

static int get_tim_clk(const struct stm32_pclken *pclken, uint32_t *tim_clk)
{
        int r;
        const struct device *clk;
        uint32_t bus_clk, apb_psc;

        clk = DEVICE_DT_GET(STM32_CLOCK_CONTROL_NODE);

        r = clock_control_get_rate(clk, (clock_control_subsys_t *)pclken,
                        &bus_clk);
        if (r < 0) {
                return r;
        }

#if defined(CONFIG_SOC_SERIES_STM32H7X)
        if (pclken->bus == STM32_CLOCK_BUS_APB1) {
                apb_psc = CONFIG_CLOCK_STM32_D2PPRE1;
        } else {
                apb_psc = CONFIG_CLOCK_STM32_D2PPRE2;
        }
#else
        if (pclken->bus == STM32_CLOCK_BUS_APB1) {
                apb_psc = CONFIG_CLOCK_STM32_APB1_PRESCALER;
        }
#if !defined(CONFIG_SOC_SERIES_STM32F0X) && !defined(CONFIG_SOC_SERIES_STM32G0X)
        else {
                apb_psc = CONFIG_CLOCK_STM32_APB2_PRESCALER;
        }
#endif
#endif

#if defined(RCC_DCKCFGR_TIMPRE) || defined(RCC_DCKCFGR1_TIMPRE) || \
        defined(RCC_CFGR_TIMPRE)
        /*
         * There are certain series (some F4, F7 and H7) that have the TIMPRE
         * bit to control the clock frequency of all the timers connected to
         * APB1 and APB2 domains.
         *
         * Up to a certain threshold value of APB{1,2} prescaler, timer clock
         * equals to HCLK. This threshold value depends on TIMPRE setting
         * (2 if TIMPRE=0, 4 if TIMPRE=1). Above threshold, timer clock is set
         * to a multiple of the APB domain clock PCLK{1,2} (2 if TIMPRE=0, 4 if
         * TIMPRE=1).
         */

        if (LL_RCC_GetTIMPrescaler() == LL_RCC_TIM_PRESCALER_TWICE) {
                /* TIMPRE = 0 */
                if (apb_psc <= 2u) {
                        LL_RCC_ClocksTypeDef clocks;

                        LL_RCC_GetSystemClocksFreq(&clocks);
                        *tim_clk = clocks.HCLK_Frequency;
                } else {
                        *tim_clk = bus_clk * 2u;
                }
        } else {
                /* TIMPRE = 1 */
                if (apb_psc <= 4u) {
                        LL_RCC_ClocksTypeDef clocks;

                        LL_RCC_GetSystemClocksFreq(&clocks);
                        *tim_clk = clocks.HCLK_Frequency;
                } else {
                        *tim_clk = bus_clk * 4u;
                }
        }
#else
        /*
         * If the APB prescaler equals 1, the timer clock frequencies
         * are set to the same frequency as that of the APB domain.
         * Otherwise, they are set to twice (×2) the frequency of the
         * APB domain.
         */
        if (apb_psc == 1u) {
                *tim_clk = bus_clk;
        } else {
                *tim_clk = bus_clk * 2u;
        }
#endif

        return 0;
}


static int motor_stm32_init(const struct device *dev)
{
        const struct timer_config *tim_cfg = to_tim_cfg(dev);
        struct motor_data *data = (struct motor_data *)dev->data;
        const struct motor_config *cfg = (const struct motor_config *)dev->config;
        int err = 0;
        const struct device *clk;
        LL_TIM_InitTypeDef init;

        clk = DEVICE_DT_GET(STM32_CLOCK_CONTROL_NODE);

        err = clock_control_on(clk, (clock_control_subsys_t *)&tim_cfg->pclken);
        if (err < 0) {
                LOG_ERR("Could not initialize clock (%d)", err);
                return err;
        }

        err = get_tim_clk(&tim_cfg->pclken, &data->tim_clk);
        if (err < 0) {
                LOG_ERR("Could not obtain timer clock (%d)", err);
                return err;
        }

        err = stm32_dt_pinctrl_configure(tim_cfg->pinctrl,
                        tim_cfg->pinctrl_len,
                        (uint32_t)tim_cfg->timer);
        if (err < 0) {
                LOG_ERR("PWM pinctrl setup failed (%d)", err);
                return err;
        }
        
        LL_TIM_StructInit(&init);

        init.CounterMode = LL_TIM_COUNTERMODE_UP;
        init.Autoreload = 7;
        init.ClockDivision = LL_TIM_CLOCKDIVISION_DIV1;
        init.Prescaler = 1;

        if (LL_TIM_Init(tim_cfg->timer, &init) != SUCCESS) {
                LOG_ERR("Could not initialize timer");
                return -EIO;
        }

        /* IC Counting clk ch1*/
        LL_TIM_ENCODER_InitTypeDef encoder_init;
        encoder_init.IC1Polarity = LL_TIM_IC_POLARITY_RISING; 
        encoder_init.IC1Filter = LL_TIM_IC_FILTER_FDIV1; 
        encoder_init.EncoderMode = LL_TIM_ENCODERMODE_X2_TI1;
        encoder_init.IC1ActiveInput = LL_TIM_ACTIVEINPUT_DIRECTTI;

        LL_TIM_ENCODER_Init(tim_cfg->timer, &encoder_init);
        if (err < 0) {
                LOG_ERR("PWM pinctrl setup failed (%d)", err);
                return err;
        }

        /* OC Configuration save ch4*/
        LL_TIM_OC_InitTypeDef oc_init;
        LL_TIM_OC_StructInit(&oc_init); 
        oc_init.OCMode = LL_TIM_OCMODE_PWM1;
        oc_init.OCState = LL_TIM_OCSTATE_ENABLE; /* Nachlesen */
        oc_init.CompareValue = 7;

        LL_TIM_OC_Init(tim_cfg->timer, ch2ll[3], &oc_init);
        if (err < 0) {
                LOG_ERR("PWM pinctrl setup failed (%d)", err);
                return err;
        }
        
        LL_TIM_EnableARRPreload(tim_cfg->timer);
        LL_TIM_OC_EnablePreload(tim_cfg->timer, ch2ll[3]);
        LL_TIM_SetAutoReload(tim_cfg->timer, 7);
        LL_TIM_GenerateEvent_UPDATE(tim_cfg->timer);


        LL_TIM_EnableCounter(tim_cfg->timer);

        return 0;
}

const struct motor_input_capture_mode motor_input_capture_mode_stm32 = {
        .init = motor_stm32_init,
};

#endif /* MOTOR_STM32 */
