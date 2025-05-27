# MAX17055 Library
# https://github.com/AwotG/Arduino-MAX17055_Driver
# MIT License

cmake_minimum_required(VERSION 3.5)

idf_component_register(SRCS "MAX17055.cpp" 
                       INCLUDE_DIRS "."
                       REQUIRES arduino)

project(MAX17055_Lib)
