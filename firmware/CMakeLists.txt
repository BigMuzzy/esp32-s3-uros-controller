cmake_minimum_required(VERSION 3.16)

include($ENV{IDF_PATH}/tools/cmake/project.cmake)
project(esp32_s3_uros_driver)

# micro-ROS libmicroros.a duplicates 64-bit atomic symbols from newlib.
# Inject linker flag via the ELF target created by project().
idf_build_set_property(LINK_OPTIONS "-Wl,--allow-multiple-definition" APPEND)
target_link_options(${project_elf} PRIVATE "-Wl,--allow-multiple-definition")
