# For more information about build system see
# https://docs.espressif.com/projects/esp-idf/en/latest/api-guides/build-system.html
# The following five lines of boilerplate have to be in your project's
# CMakeLists in this exact order for cmake to work correctly
cmake_minimum_required(VERSION 3.16)

set(EXTRA_COMPONENT_DIRS "common/calc" "common/gps" "common/qmc5883l" "common/mpu9250" "common/i2c" "common/kalman_filter" "common/stepper_motor" )
include($ENV{IDF_PATH}/tools/cmake/project.cmake)

project(sat)

target_add_binary_data(sat.elf "main/mqtt_eclipseprojects_io.pem" TEXT)