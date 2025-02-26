# The following lines of boilerplate have to be in your project's
# CMakeLists in this exact order for cmake to work correctly
cmake_minimum_required(VERSION 3.5)

include "freertos/FreeRTOS.h"
include "freertos/task.h"
include "driver/gpio.h"
include "driver/uart.h"
include "driver/i2c.h"
include "driver/pwm.h"

include($ENV{IDF_PATH}/tools/cmake/project.cmake)
project(app-template)
