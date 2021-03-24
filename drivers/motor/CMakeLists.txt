zephyr_library()

zephyr_library_sources_ifdef(CONFIG_MOTOR motor.c motor_stm32.c)

