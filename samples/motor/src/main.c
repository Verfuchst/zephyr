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

	int sensitivity = 0;
	while (1) {

		motor_set_sensitivity(dev, 0xFF, sensitivity);

#if defined CONFIG_MOTOR_FRAMES_10
		sensitivity += 1;
		if (sensitivity > 10) {
			sensitivity = 0;
		}
#elif defined CONFIG_MOTOR_FRAMES_100
		sensitivity += 10;
		if (sensitivity > 100) {
			sensitivity = 0;
		}
#endif
		k_sleep(K_SECONDS(5));
	}
}
