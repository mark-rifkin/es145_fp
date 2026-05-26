cmake_minimum_required(VERSION 3.22)

set(CMAKE_C_STANDARD 11)
set(CMAKE_C_STANDARD_REQUIRED ON)
set(CMAKE_C_EXTENSIONS ON)

if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE "Debug")
endif()

set(CMAKE_PROJECT_NAME full_bridge)
set(CMAKE_EXPORT_COMPILE_COMMANDS TRUE)

project(${CMAKE_PROJECT_NAME})
message("Build type: " ${CMAKE_BUILD_TYPE})

enable_language(C ASM)

add_executable(${CMAKE_PROJECT_NAME})

add_subdirectory(cmake/stm32cubemx)

# Cortex-M4 / STM32G474 CPU flags
set(STM32_CPU_FLAGS
    -mcpu=cortex-m4
    -mthumb
    -mfpu=fpv4-sp-d16
    -mfloat-abi=hard
)

target_compile_options(${CMAKE_PROJECT_NAME} PRIVATE ${STM32_CPU_FLAGS})
target_link_options(${CMAKE_PROJECT_NAME} PRIVATE ${STM32_CPU_FLAGS})
target_compile_options(STM32_Drivers PRIVATE ${STM32_CPU_FLAGS})

set(CMAKE_ASM_FLAGS "${CMAKE_ASM_FLAGS} -mcpu=cortex-m4 -mthumb -mfpu=fpv4-sp-d16 -mfloat-abi=hard")

target_link_directories(${CMAKE_PROJECT_NAME} PRIVATE
)

target_sources(${CMAKE_PROJECT_NAME} PRIVATE
    Src/full_bridge_pwm.c
    Src/grid_track.c
)

target_include_directories(${CMAKE_PROJECT_NAME} PRIVATE
)

target_compile_definitions(${CMAKE_PROJECT_NAME} PRIVATE
    USE_HAL_DRIVER
    STM32G474xx
    $<$<CONFIG:Debug>:DEBUG>
)

list(REMOVE_ITEM CMAKE_C_IMPLICIT_LINK_LIBRARIES ob)

target_link_libraries(${CMAKE_PROJECT_NAME}
    stm32cubemx
)