#ifndef ZEPHYR_INCLUDE_DRIVERS_MOTOR_H_
#define ZEPHYR_INCLUDE_DRIVERS_MOTOR_H_

#include <zephyr/types.h>
#include <device.h>

/**
 * The following defines, define the bits for the motor
 * depends on how they are connected e.g. MOTOR_1 is the
 * motor connected to the board, MOTOR_2 is connected
 * to MOTOR_1.
 */
#define MOTOR_1     (1 << 0)
#define MOTOR_2     (1 << 1)
#define MOTOR_3     (1 << 2)
#define MOTOR_4     (1 << 3)
#define MOTOR_5     (1 << 4)
#define MOTOR_6     (1 << 5)
#define MOTOR_7     (1 << 6)
#define MOTOR_8     (1 << 7)
#define MOTOR_9     (1 << 8)
#define MOTOR_10    (1 << 9)
#define MOTOR_11    (1 << 10)
#define MOTOR_12    (1 << 11)


/**
 * @typedef motor_api_set_sensitivity
 * @brief Callback API for writing the sensitivity to a single motor device 
 *                     or multiple devices 
 *
 * See motor_set_sensitivity() for argument description 
 */
typedef int (*motor_api_set_sensitivity)(const struct device *dev,
                                           const uint16_t motor,
                                           const uint8_t sensitivity); 

/**
 * @typedef motor_api_set_sensitivity_n
 * @brief Callback API for writing different sensitivity to multiple motor devices
 *
 * See motor_set_sensitivity_n() for argument description
 */
typedef int (*motor_api_set_sensitivity_n)(const struct device *dev,
                                            const uint16_t motors,
                                            const uint8_t *sensitivity); 

struct motor_driver_api {
    motor_api_set_sensitivity     set_sensitivity;
    motor_api_set_sensitivity_n   set_sensitivity_n;
};


/**
 * @brief Callback API for writing the sensitivity to a single motor device 
 *                     or multiple devices 
 * 
 * @param dev           Pointer to the motor device 
 * @param motor         contains the bit for the motor (e.g. MOTOR_1 or (MOTOR_1 | MOTOR_2))
 * @param sensitivity   sensitivity for the motor in percentage 1-100
 *
 * @return 0 if sucessful, negative errno code if failure.
 */
static inline int motor_set_sensitivity(const struct device *dev,
                                          const uint16_t motor,
                                          const uint8_t sensitivity)
{
    const struct motor_driver_api *api = 
        (const struct motor_driver_api *)dev->api;

    return api->set_sensitivity(dev, motor, sensitivity);
}

/**
 * @brief Callback API for writing different sensitivity to multiple motor devices
 * 
 * @param dev           Pointer to the motor device 
 * @param motor         contains the bits for the motor e.g. (MOTOR_1 | MOTOR_2)
 * @param sensitivity   points on a array where each field describes the sensitivity
 *                      for a motor from 1-100
 *                      
 *                      example:
 *                      array[0] = 20 // Sends a sensitivity of 20 to motor 1
 *                      array[5] = 50 // Sends a sensitivity of 50 to motor 6
 *
 * @return 0 if sucessful, negative errno code if failure.
 */
static inline int motor_set_sensitivity_n(const struct device *dev,
                                            const uint16_t motor,
                                            const uint8_t *sensitivity)
{
    const struct motor_driver_api *api = 
        (const struct motor_driver_api *)dev->api;

    return api->set_sensitivity_n(dev, motor, sensitivity);
}

#endif /*ZEPHYR_INCLUDE_DRIVERS_MOTOR_H_*/




