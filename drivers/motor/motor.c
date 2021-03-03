/* motor.c - Driver for MOTOR vibration */

#include <cstdint>
#include <kernel.h>
#include <init.h>
#include <drivers/gpio.h>
#include <drivers/motor.h>
#include <drivers/spi.h>
#include <device.h>
#include <zephyr/types.h>
#include <devicetree.h>

#include <logging/log.h>

#define DT_DRV_COMPAT thm_motor

LOG_MODULE_REGISTER(MOTOR, CONFIG_MOTOR_LOG_LEVEL);

#if DT_NUM_INST_STATUS_OKAY(DT_DRV_COMPAT) == 0
#warning "MOTOR driver enabled without any devices"
#endif


#define MOTOR_SPI_OPERATION (SPI_WORD_SET(8) | SPI_TRANSFER_MSB)

/* Number of sendet frames to get a 1-100 precentage for sensitivity*/
#define FRAMES 100

/* To many devices */
#define EMANYDEVS 5

struct motor_data {
        struct k_timer timer;
};

struct motor_config {
        const struct device *bus;
        const struct spi_config spi_cfg;
};

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

static int motor_write_spi(const struct device *bus,
                const uint16_t motor,
                const uint8_t val)
{   
        uint16_t cmd[FRAMES] = { 0 }; 
        
        /* Init the array with 1 depends on the val for the sensitivity */
        for(int i = 0; i < val; i++) {
                cmd[i] = motor;
        }

        struct spi_buf tx_buf = {
                .buf = cmd,
                .len = 1
        };

        const struct spi_buf_set tx = {
                .buffers = &tx_buf,
                .count = 1
        };

        uint8_t ret = 0;
        for(int i = 1; i <= FRAMES; i++) {
                ret = spi_write(to_bus(bus), to_bus_config(bus), &tx);
                if (ret) {
                        LOG_DBG("spi_write FAIL %d\n", ret);
                        return ret;
                }
                tx_buf.buf = cmd+i;
        }

        return 0;
}

static const struct motor_driver_api motor_api_funcs = {
        .write_sensitivity = motor_write_spi,
};


struct device *devices[5];
static int8_t number_of_devices;


void motor_worker_handler(struct k_work* work) 
{
        uint16_t motor = MOTOR_1 | MOTOR_2;
        for(uint8_t i = 0; i < number_of_devices; i++) {
                motor_write_spi(devices[i], motor, 20);
        }
}	

K_WORK_DEFINE(work, motor_worker_handler);

void motor_timer_handler(struct k_timer *timer) 
{
        k_work_submit(&work);
}

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


        if(number_of_devices > 5) {
                err = -EMANYDEVS;
                return err;
        }

        err = motor_bus_check_spi(to_bus(dev), to_bus_config(dev));
        if(err != 0) {
                LOG_DBG("bus check failed: %d", err);
                return err;
        }

        devices[number_of_devices++] = dev;

        k_timer_init(&data->timer, motor_timer_handler, NULL);
        k_timer_start(&data->timer,  K_NO_WAIT, K_MSEC(1));
        
        LOG_DBG("\"%s\" OK", dev->name);
        return 0;
}

/* Initializes a struct motor_config for an instance on a SPI bus. */
#define MOTOR_CONFIG_SPI(inst)                                          \
        {                                                               \
                .bus = DEVICE_DT_GET(DT_INST_BUS(inst)),                \
                .spi_cfg = SPI_CONFIG_DT_INST(inst,                     \
                                              MOTOR_SPI_OPERATION,      \
                                              0),                       \
        }

/*
 *  Main instantiation macro.
 */

#define MOTOR_DEFINE(inst)                                              \
        static struct motor_data motor_data_##inst;                     \
        static const struct motor_config motor_config_##inst =          \
                            MOTOR_CONFIG_SPI(inst);                     \
        DEVICE_DT_INST_DEFINE(inst,                                     \
                        motor_init,                                     \
                        device_pm_control_nop,                          \
                        &motor_data_##inst,                             \
                        &motor_config_##inst,                           \
                        POST_KERNEL,                                    \
                        CONFIG_MOTOR_INIT_PRIORITY,                     \
                        &motor_api_funcs);

DT_INST_FOREACH_STATUS_OKAY(MOTOR_DEFINE)
