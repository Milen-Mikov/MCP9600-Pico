# MCP9600-Pico
MCP9600 thermocouple EMF to temperature converter library for the Raspberry Pi Pico C/C++ SDK. 

Based on the [LibDriver generic MCP9600 driver](https://github.com/libdriver/mcp9600).

## Requirements
This library must be used with the [Raspberry Pi Pico C/C++ SDK](https://www.raspberrypi.com/documentation/microcontrollers/c_sdk.html).

## Installation
To use this library, first copy or clone this repository into your project folder. 

Then add the following to your project's CMakeLists.txt folder:
```cmake
add_subdirectory(
	mcp9600-pico
)
target_link_libraries(PROJECT_NAME_HERE
        mcp9600_pico
)
```

## Examples
Here are some examples demonstrating each of the three provided drivers:
### Basic (continuous read)
```C
#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/i2c.h"

#include "driver_mcp9600_basic.h"

// pin definitions
#define pin_i2c_sda 2
#define pin_i2c_scl 3
#define mcp_i2c_inst i2c1
#define i2c_speed 100*1000 // 100kHz bus speed

// mcp constants
#define mcp_thermocouple_type MCP9600_THERMOCOUPLE_TYPE_K
#define mcp_i2c_address MCP9600_ADDRESS_8 // default for adafruit breakout

uint8_t status;
int16_t hot_raw;
float hot_s;
int16_t delta_raw;
float delta_s;
int16_t cold_raw;
float cold_s;

int main()
{
    stdio_init_all();

    //sleep_ms(5000); // delay for serial monitor

    // init i2c bus
    i2c_init(mcp_i2c_inst, i2c_speed);
    gpio_set_function(pin_i2c_sda, GPIO_FUNC_I2C);
    gpio_set_function(pin_i2c_scl, GPIO_FUNC_I2C);
    gpio_pull_up(pin_i2c_sda);
    gpio_pull_up(pin_i2c_scl);

    // create handle for an mcp9600
    mcp9600_handle_t mcp9600;

    // mcp9600 init
    status = mcp9600_basic_init(&mcp9600, mcp_i2c_inst, mcp_i2c_address, mcp_thermocouple_type);
    if (status != 0)
    {
        printf("Initialization failure.\n");

        while(true){
            sleep_ms(1000);
        }
    }

    while (true) {
        
        status = mcp9600_basic_read(&mcp9600, &hot_raw, &hot_s, &delta_raw, &delta_s, &cold_raw, &cold_s);

        if(status != 0) {
            printf("mcp9600 basic read failed.\n");
            mcp9600_basic_deinit(&mcp9600);
            break;
        }
        
        else {
            printf("mcp9600: hot %0.2f, delta %0.2f, cold %0.2f.\n", hot_s, delta_s, cold_s);
        }

        sleep_ms(1000);
    }
}

```

### Shot (single read)
```C
#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/i2c.h"

#include "driver_mcp9600_shot.h"

// pin definitions
#define pin_i2c_sda 2
#define pin_i2c_scl 3
#define mcp_i2c_inst i2c1
#define i2c_speed 100*1000 // 100kHz bus speed

// mcp constants
#define mcp_thermocouple_type MCP9600_THERMOCOUPLE_TYPE_K
#define mcp_i2c_address MCP9600_ADDRESS_8 // default for adafruit breakout

uint8_t status;
int16_t hot_raw;
float hot_s;
int16_t delta_raw;
float delta_s;
int16_t cold_raw;
float cold_s;

int main()
{
    stdio_init_all();

    //sleep_ms(5000); // delay for serial monitor

    // init i2c bus
    i2c_init(mcp_i2c_inst, i2c_speed);
    gpio_set_function(pin_i2c_sda, GPIO_FUNC_I2C);
    gpio_set_function(pin_i2c_scl, GPIO_FUNC_I2C);
    gpio_pull_up(pin_i2c_sda);
    gpio_pull_up(pin_i2c_scl);

    // create handle for an mcp9600
    mcp9600_handle_t mcp9600;

    // mcp9600 init
    status = mcp9600_shot_init(&mcp9600, mcp_i2c_inst, mcp_i2c_address, mcp_thermocouple_type);
    if (status != 0)
    {
        printf("Initialization failure.\n");

        while(true){
            sleep_ms(1000);
        }
    }

    while (true) {
        
        status = mcp9600_shot_read(&mcp9600, &hot_raw, &hot_s, &delta_raw, &delta_s, &cold_raw, &cold_s);

        if(status != 0) {
            printf("mcp9600 shot read failed.\n");
            mcp9600_shot_deinit(&mcp9600);
            break;
        }
        
        else {
            printf("mcp9600: hot %0.2f, delta %0.2f, cold %0.2f.\n", hot_s, delta_s, cold_s);
        }

        sleep_ms(1000);
    }
}
```

### Interrupt (continous read with alert pins)
```C
#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/i2c.h"
#include "hardware/gpio.h"

#include "driver_mcp9600_interrupt.h"

// pin definitions
#define pin_i2c_sda 2
#define pin_i2c_scl 3
#define mcp_i2c_inst i2c1
#define i2c_speed 100*1000 // 100kHz bus speed
#define pin_alert_1 4 // GPIO connected to alert 1

// mcp config
#define mcp_thermocouple_type MCP9600_THERMOCOUPLE_TYPE_K
#define mcp_i2c_address MCP9600_ADDRESS_8 // default for adafruit breakout

uint8_t status;
int16_t hot_raw;
float hot_s;
int16_t delta_raw;
float delta_s;
int16_t cold_raw;
float cold_s; 

uint8_t alert1_flag;

void alert1_callback() {
    alert1_flag = 1;
}

int main()
{
    stdio_init_all();

    //sleep_ms(5000); // delay for serial monitor

    // init i2c bus
    i2c_init(mcp_i2c_inst, i2c_speed);
    gpio_set_function(pin_i2c_sda, GPIO_FUNC_I2C);
    gpio_set_function(pin_i2c_scl, GPIO_FUNC_I2C);
    gpio_pull_up(pin_i2c_sda);
    gpio_pull_up(pin_i2c_scl);

    // initialize the interrupt pin
    gpio_init(pin_alert_1);
    gpio_set_dir(pin_alert_1, GPIO_IN);
    gpio_pull_up(pin_alert_1);
    gpio_set_irq_enabled_with_callback(pin_alert_1, GPIO_IRQ_EDGE_FALL, 1, &alert1_callback);

    // create handle for an mcp9600
    mcp9600_handle_t mcp9600;

    // mcp9600 init
    status = mcp9600_interrupt_init(&mcp9600, mcp_i2c_inst, mcp_i2c_address, mcp_thermocouple_type);
    if (status != 0)
    {
        printf("Initialization failure.\n");

        while(true){
            sleep_ms(1000);
        }
    }

    // clear alert 1
    status = mcp9600_interrupt_clear(&mcp9600, MCP9600_ALERT_1);
    if (status != 0)
    {
        printf("Alert 1 clear failure.\n");

        while(true){
            sleep_ms(1000);
        }
    }

    while (true) {
        
        status = mcp9600_interrupt_read(&mcp9600, &hot_raw, &hot_s, &delta_raw, &delta_s, &cold_raw, &cold_s);

        if(status != 0) {
            printf("mcp9600 shot read failed.\n");
            mcp9600_interrupt_deinit(&mcp9600);
            break;
        }
        
        else {
            printf("mcp9600: hot %0.2f, delta %0.2f, cold %0.2f.\n", hot_s, delta_s, cold_s);
        }

        // if the alert was detected
        if (alert1_flag == 1) {
            
            // print
            printf("Alert 1 Detected\n");

            // clear alert 1
            alert1_flag = 0;
            status = mcp9600_interrupt_clear(&mcp9600, MCP9600_ALERT_1);
            if (status != 0) {
                printf("Alert 1 clear failure.\n");
                mcp9600_interrupt_deinit(&mcp9600);
                break;
            }
        }

        sleep_ms(1000);
    }
}
```
