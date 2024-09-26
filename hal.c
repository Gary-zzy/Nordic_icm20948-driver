/*
 * Copyright(c) 2021 - Jim Newman
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy of this
 * software and associated documentation files (the "Software"), to deal in the Software
 * without restriction, including without limitation the rights to use, copy, modify,
 * merge, publish, distribute, sublicense, and/or sell copies of the Software, and to
 * permit persons to whom the Software is furnished to do so, subject to the following
 * conditions:
 *
 * The above copyright notice and this permission notice shall be included in all copies
 * or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED,
 * PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT
 * HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF
 * CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE
 * OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
 */

/***********************************************************************/
/*                                                                     */
/* HAL -- These are the callback functions required by the InvenSense  */
/*        IMC-20948 driver.                                            */
/*                                                                     */
/***********************************************************************/

#include "nrf_delay.h"
#include "nrf_drv_rtc.h"

#include "imu.h"
#include "ens210.h"
#include "hal.h"
#include "i2c.h"
#include <nrfx.h>
#include <time.h>


//static bool verbose = false;	// For debugging I2C issues.

//const nrf_drv_rtc_t rtc = NRF_DRV_RTC_INSTANCE(0); /**< Declaring an instance of nrf_drv_rtc for RTC0. */

//uint32_t inv_icm20948_get_time_us(void)
//{
//    return nrf_drv_rtc_counter_get(&rtc);
//}



/*int inv_icm20948_i2c_read_reg(uint8_t reg, uint8_t *value)
{
    if (verbose)
        printf("Executing %s(0x%02x)\r\n", __func__, reg);

    *value = twi_read_register(IMU_ADDR, reg);

    if (verbose)
    {
        printf("0x%02x\r\n", *value);
    }

    return 0;
}

int inv_icm20948_i2c_read_reg_block(uint8_t reg, uint8_t * rbuffer, uint32_t rlen)
{
    int i;

    if (verbose)
        printf("Executing %s(0x%02x, %ld)\r\n", __func__, reg, rlen);

    twi_read_register_block(IMU_ADDR, reg, rbuffer, rlen);

    if (verbose)
    {
        for (i = 0; i < rlen; i++)
            printf(" val[%d]:0x%02x%s", i, rbuffer[i], (i%4==3?"\r\n":", "));
        printf("\r\n");
    }

    return 0;
}

int inv_icm20948_i2c_write_reg(uint8_t reg, uint8_t value)
{
    if (verbose)
    {
        printf("Executing %s(0x%02x, 0x%02x)\r\n", __func__, reg, value);
    }

    twi_write_register(IMU_ADDR, reg, value);

    return 0;
}

int inv_icm20948_i2c_write_reg_block(uint8_t reg, uint8_t *wbuffer, uint32_t wlen)
{
    int i;

    if (verbose)
    {
        printf("Executing %s(0x%02x, 0x%02x, %ld)", __func__, reg, wbuffer[0], wlen);
        if (wlen > 1)
        {
            printf("\r\n");
            for (i = 0; i < wlen; i++)
                printf(" val[%d]:0x%02x%s", i, wbuffer[i], (i%4==3?"\r\n":((i==(wlen-1)?"\r\n":","))));
        }
    }

    twi_write_register_block(IMU_ADDR, reg, wbuffer, wlen);

    return 0;
}*/




static bool verbose = false;	// For debugging I2C issues.

const nrf_drv_rtc_t rtc = NRF_DRV_RTC_INSTANCE(0); /**< Declaring an instance of nrf_drv_rtc for RTC0. */

uint32_t inv_icm20948_get_time_us(void)
{
    return nrf_drv_rtc_counter_get(&rtc);
}

void inv_icm20948_sleep_us(uint32_t us)
{
    nrf_delay_us(us);
}

int inv_icm20948_i2c_init(void)
{
    return 0;
}
/*下面修改的比较关键 涉及整个读写操作*/
int inv_icm20948_i2c_read_reg(uint8_t reg, uint8_t *value)
{
    int8_t result;

    // 读取单个寄存器
    result = i2c_read_single_register(IMU_ADDR, reg, value, 1);
    
    return (result == 0) ? 0 : -1;
}

int inv_icm20948_i2c_read_reg_block(uint8_t reg, uint8_t *rbuffer, uint32_t rlen)
{
    int8_t result;

    // 读取多个寄存器
    result = i2c_read_registers(IMU_ADDR, reg, rbuffer, rlen);
    
    return (result == 0) ? 0 : -1;
}

int inv_icm20948_i2c_write_reg(uint8_t reg, uint8_t value)
{
    int8_t result;

    // 写入单个寄存器
    result = i2c_write_single_register(IMU_ADDR, reg, &value, 1);
    
    return (result == 0) ? 0 : -1;
}

int inv_icm20948_i2c_write_reg_block(uint8_t reg, uint8_t *wbuffer, uint32_t wlen)
{
    int8_t result;

    // 写入多个寄存器
    result = i2c_write_registers(IMU_ADDR, wbuffer, wlen);
    
    return (result == 0) ? 0 : -1;
}



// 获取当前时间，单位为微秒（假设系统支持）
uint32_t ens210_get_time_us(void) {
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);  // 获取系统单调时间
    return (uint32_t)((ts.tv_sec * 1000000) + (ts.tv_nsec / 1000));  // 将时间转换为微秒
}

// 微秒级别的延迟
void ens210_sleep_us(uint32_t us) {
   nrf_delay_us(us);
}

// 初始化I2C接口
int ens210_i2c_init(void) {

    return 0; 
}

// 从 ENS210 寄存器读取一个字节
int ens210_i2c_read_reg(uint8_t reg, uint8_t *value) {
    // 发起I2C读取操作，读取一个寄存器
    return i2c_read_single_register(ENS210_ADDR, reg, value, 1);  // ENS210_ADDR 是 ENS210 的I2C地址
}

// 从 ENS210 连续读取多个字节
int ens210_i2c_read_reg_block(uint8_t reg, uint8_t *rbuffer, uint32_t rlen) {
    // 发起I2C读取操作，读取多个寄存器
    return i2c_read_registers(ENS210_ADDR, reg, rbuffer, rlen);  // `i2c_read_registers` 是底层读取多个字节的函数
}

// 向 ENS210 寄存器写入一个字节
int ens210_i2c_write_reg(uint8_t reg, uint8_t value) {
    // 发起I2C写操作，写入一个寄存器
    return  i2c_write_single_register(ENS210_ADDR, reg, &value,1);  // `i2c_write_register` 是底层写入单个字节的函数
}

// 向 ENS210 连续写入多个字节
int ens210_i2c_write_reg_block(uint8_t reg, uint8_t *wbuffer, uint32_t wlen) {
    // 发起I2C写操作，写入多个寄存器
    return i2c_write_registers(ENS210_ADDR,  wbuffer, wlen);  // `i2c_write_registers` 是底层写入多个字节的函数
    }


