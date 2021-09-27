.. _bno055:

BNO055 accelerometer, gyroscope, magnetometer and orientation sensor*/
###################################

Overview
********

This sample shows how to use the Zephyr :ref:`sensor_api` API driver for the
`Bosch BNO055`_ environmental sensor.

.. _Bosch BNO055:
   https://www.bosch-sensortec.com/media/boschsensortec/downloads/datasheets/bst-bno055-ds000.pdf`

The sample periodically reads euler angles data from the
first available BME280 device discovered in the system. 

Building and Running
********************

The sample can be configured to support BNO055 sensors connected via either I2C
or UART. Configuration is done via :ref:`devicetree <dt-guide>`. The devicetree
must have an enabled node with ``compatible = "bosch,bno055";``. See below for
examples and common configurations.

.. _BNO055 datasheet:
   https://www.bosch-sensortec.com/media/boschsensortec/downloads/datasheets/bst-bno055-ds000.pdf 

Board-specific overlays
=======================

You can create a board-specific devicetree overlay for any board in the
:file:`boards` directory. This sample must provide

The build system uses these overlays by default when targeting those boards, so
no ``DTC_OVERLAY_FILE`` setting is needed when building and running.

For example, to build for the :ref:`disco_l475_iot1.overlay` using the
:zephyr_file:`/home/tobias/zephyrproject/zephyr/samples/sensor/bno055/boards/disco_l475_iot1.overlay`
overlay provided with this sample:

.. zephyr-app-commands::
   :zephyr-app: samples/sensor/bno055
   :goals: build flash
   :board: disco_l475_iot1

Sample Output
=============
