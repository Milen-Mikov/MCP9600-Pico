add_library(mcp9600_pico STATIC

    src/driver_mcp9600.c                    # generic driver

    interface/driver_mcp9600_interface.c    # pico HAL interface

    drivers/driver_mcp9600_basic.c          # continous reading 
    drivers/driver_mcp9600_interrupt.c      # interrupt read
    drivers/driver_mcp9600_shot.c           # single read 
)
target_link_libraries(mcp9600_pico
    pico_stdlib 
    hardware_i2c
)
target_include_directories(mcp9600_pico PUBLIC
    src
    interface
    drivers
)
