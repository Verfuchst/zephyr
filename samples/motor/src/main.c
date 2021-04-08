#include <zephyr.h>
#include <device.h>
#include <devicetree.h>
#include <drivers/motor.h>

static const struct device *get_motor_device(void)
{
        const struct device *dev = DEVICE_DT_GET_ANY(thm_motor);

        if (dev == NULL) {
                /* No such node, or the node does not have status "okay". */
                printk("\nError: no device found.\n");
                return NULL;
        }

        if (!device_is_ready(dev)) {
                printk("\nError: Device \"%s\" is not ready; "
                                "check the driver initialization logs for errors.\n",
                                dev->name);
                return NULL;
        }

        printk("Found device \"%s\", ready to write \n", dev->name);
        return dev;
}


void main(void)
{
        const struct device *dev = get_motor_device();

        if (dev == NULL) {
                return;
        }
        while (1) {
                printk("sleep 1 sec \n");
                motor_set_sensitivity(dev, MOTOR_1 | MOTOR_2, 10);
                k_sleep(K_SECONDS(1));
        }
}
