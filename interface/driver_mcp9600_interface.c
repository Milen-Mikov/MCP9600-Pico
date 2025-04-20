/**
 * Copyright (c) 2015 - present LibDriver All rights reserved
 * 
 * The MIT License (MIT)
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE. 
 *
 */

#include "driver_mcp9600_interface.h"
#include "pico/stdlib.h"
#include "hardware/i2c.h"


/**
 * @brief     interface iic bus write command
 * @param[in] i2c_inst pico i2c instance for device
 * @param[in] addr iic device write address
 * @param[in] *buf pointer to a data buffer
 * @param[in] len length of data buffer
 * @return    status code
 *            - 0 success
 *            - 1 write failed
 * @note      none
 */
uint8_t mcp9600_interface_iic_write_cmd(i2c_inst_t * i2c_inst, uint8_t addr, uint8_t *buf, uint16_t len)
{
    // attempt to write to the i2c bus, with a 100ms timeout
    int status = i2c_write_timeout_us(i2c_inst, addr, buf, len, false, 100000); 

    // if error, return 1, otherwise return 0
    if (status == PICO_ERROR_GENERIC) {
        return 1;
    }
    else {
        return 0;
    }
}

/**
 * @brief      interface iic bus read command
 * @param[in] i2c_inst pico i2c instance for device
 * @param[in]  addr iic device write address
 * @param[out] *buf pointer to a data buffer
 * @param[in]  len length of data buffer
 * @return     status code
 *             - 0 success
 *             - 1 read failed
 * @note       none
 */
uint8_t mcp9600_interface_iic_read_cmd(i2c_inst_t * i2c_inst, uint8_t addr, uint8_t *buf, uint16_t len)
{
    // attempt to read from the i2c bus, with a 100ms timeout
    int status = i2c_read_timeout_us(i2c_inst, addr, buf, len, false, 100000); 

    // if error, return 1, otherwise return 0
    if (status == PICO_ERROR_GENERIC) {
        return 1;
    }
    else {
        return 0;
    }
}

/**
 * @brief     interface delay ms
 * @param[in] ms time
 * @note      none
 */
void mcp9600_interface_delay_ms(uint32_t ms)
{
    sleep_ms(ms);
}

/**
 * @brief     interface print format data
 * @param[in] fmt format data
 * @note      none
 */
void mcp9600_interface_debug_print(const char *const fmt, ...)
{
    printf(fmt);
}
