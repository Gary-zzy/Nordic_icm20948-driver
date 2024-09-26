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

#include "nrf_log.h"
#include "nrf_log_ctrl.h"
#include "nrf_log_default_backends.h"

#include "imu.h"
#include "i2c.h"
#include "hal.h"



char *INV_ICM20948_ACCEL_FSR_ASCII [] = {
	"INV_ICM20948_ACCEL_FSR_02G",
	"INV_ICM20948_ACCEL_FSR_04G",
	"INV_ICM20948_ACCEL_FSR_08G",
	"INV_ICM20948_ACCEL_FSR_16G"
};


char *INV_ICM20948_GYRO_FSR_ASCII [] = {
	"INV_ICM20948_GYRO_FSR_250DPS",
	"INV_ICM20948_GYRO_FSR_500DPS",
	"INV_ICM20948_GYRO_FSR_1000DPS",
	"INV_ICM20948_GYRO_FSR_2000DPS"
};


inv_icm20948_chip_config chip_config_20948 = {
	.accl_fsr = INV_ICM20948_ACCEL_FSR_04G,
	.gyro_fsr = INV_ICM20948_GYRO_FSR_2000DPS,
	.magn_fsr = INV_ICM20948_MAGN_FSR_4900UT,
	.accl_fifo_enable = false,
	.gyro_fifo_enable = false,
	.magn_fifo_enable = false,
	.temp_fifo_enable = false,
	.enable = false,
	.bytes_per_datum = 0,
	.sample_rate = INV_ICM20948_INIT_SAMPLE_RATE,
	.accel_dlpf = INV_ICM20948_ACCEL_FILTER_246HZ,	// normal default
	.gyro_dlpf = INV_ICM20948_GYRO_FILTER_197HZ     // normal default
};

inv_icm20948_state st = {
        .chip_type   = INV_ICM20948,
        .chip_config = &chip_config_20948
};

int16_t inv_icm20948_set_power(inv_icm20948_state *st, bool power_on)
{
    int result;
    if (st->chip_config->enable != power_on) {
        result = inv_icm20948_set_sleep_mode(power_on == true ? false : true);
        if (result)
            return result;
        st->chip_config->enable = power_on;
        //if (power_on)
        //    msleep(INV_ICM20948_POWER_UP_TIME);
    }
    return 0;
}

int16_t inv_check_and_setup_chip(inv_icm20948_state *st)
{
    int16_t result;

    //twim_init();



    result = inv_icm20948_init(st);
    if (result)
        return -1;

    inv_icm20948_init_magnetometer();

    result = inv_icm20948_set_power(st, true);
    if (result)
        return -1;


    if (   st->chip_config->accl_fifo_enable == true
        || st->chip_config->gyro_fifo_enable == true
        || st->chip_config->magn_fifo_enable == true
        || st->chip_config->temp_fifo_enable == true )
    {
        st->chip_config->bytes_per_datum = 0;
        if (st->chip_config->accl_fifo_enable == true)
            st->chip_config->bytes_per_datum += 6;
        if (st->chip_config->gyro_fifo_enable == true)
            st->chip_config->bytes_per_datum += 6;
        if (st->chip_config->magn_fifo_enable == true)
            st->chip_config->bytes_per_datum += 6;
        if (st->chip_config->temp_fifo_enable == true)
            st->chip_config->bytes_per_datum += 2;
        inv_icm20948_reset_fifo(st);
    }
    else
    {
        inv_icm20948_write_register(IMU_INT_ENABLE_1, IMU_BIT_RAW_DATA_0_RDY_EN);
    }

    return 0;
}

int16_t inv_icm20948_init(inv_icm20948_state *st)
{
     uint8_t result, counter, temp, imu_device_id;

         imu_device_id = inv_icm20948_get_device_id();
    if (imu_device_id != IMU_EXPECTED_WHOAMI)
        return -1;//找寄存器



    counter = 0;
    inv_icm20948_write_register(IMU_PWR_MGMT_1, IMU_BIT_DEVICE_RESET);
    do {
        inv_icm20948_sleep_us(10); // 10uS delay
        temp = inv_icm20948_read_register(IMU_PWR_MGMT_1);
    } while ((temp & IMU_BIT_DEVICE_RESET) && (counter++ < 1000));//device reset

    // device requires 100mS delay after power-up/reset
    inv_icm20948_sleep_us(100000);    // 100mS delay

        // clear the sleep enable bit
    result = inv_icm20948_set_sleep_mode(false);
    if (result)
        return result; //醒过来


      // set the clock source
    temp = inv_icm20948_read_register(IMU_PWR_MGMT_1);
    temp &= 0xf8;
    temp |= 0x01;
    inv_icm20948_write_register(IMU_PWR_MGMT_1, temp);//设置时间

    inv_icm20948_write_register(IMU_ODR_ALIGN_EN, 0x01);//ODR 打开


    uint8_t new_val = read_single_ak09916_reg(IMU_USER_CTRL);
	new_val |= 0x10;

	write_single_ak09916_reg(IMU_USER_CTRL, new_val);
      

    // setup low pass filters 
    result = inv_icm20948_set_gyro_dlpf(st->chip_config->gyro_dlpf);
    result = inv_icm20948_set_accel_dlpf(st->chip_config->accel_dlpf);

    // setup sample rate
    result = inv_icm20948_set_sample_frequency(st->chip_config->sample_rate);
    if (result)
        return result;


    // set the gyro full scale range
    inv_icm20948_config_gyro(st->chip_config->gyro_fsr);

    // set the accelerometer full scale range
    inv_icm20948_config_accel(st->chip_config->accl_fsr);

    // set the sleep enable bit
    //inv_icm20948_set_sleep_mode(true);

    return 0;
}

int16_t inv_icm20948_set_sleep_mode(bool sleep_mode)
{
    uint8_t temp, temp2;

    // get the sleep enable bit
    temp = inv_icm20948_read_register(IMU_PWR_MGMT_1);
    if (sleep_mode == false)
        temp &= ~(IMU_BIT_SLEEP);    // clear the sleep bit
    else
        temp |= IMU_BIT_SLEEP;       // set the sleep bit
    inv_icm20948_write_register(IMU_PWR_MGMT_1, temp);
    temp2 = inv_icm20948_read_register(IMU_PWR_MGMT_1);
    if (temp != temp2)
    {
        return -1;
    }

    return 0;
}

int16_t inv_icm20948_set_sample_frequency(uint16_t rate)
{
    uint8_t divider;
    divider = (uint8_t)(1100 / rate) - 1;    // from ICM-20948 data sheet
    inv_icm20948_write_register(IMU_GYRO_SMPLRT_DIV, divider);
    return 0;
}

int16_t inv_icm20948_set_gyro_dlpf(inv_icm20948_gyro_filter_e rate)
{
    uint8_t temp;
    temp = inv_icm20948_read_register(IMU_GYRO_CONFIG_1);  // get current reg value
    temp &= 0xc6;                                          // clear all dlpf bit
    if (rate != INV_ICM20948_GYRO_FILTER_12106HZ_NOLPF) {
        temp |= 0x01;                                      // set FCHOICE bit
        temp |= (rate & 0x07) << 3;                        // set DLPFCFG bits
    }
    inv_icm20948_write_register(IMU_GYRO_CONFIG_1, temp);           // set new value
    return 0;
}

int16_t inv_icm20948_set_accel_dlpf(inv_icm20948_accel_filter_e rate)
{
    uint8_t temp;
    temp = inv_icm20948_read_register(IMU_ACCEL_CONFIG);            // get current reg value
    temp &= 0xc6;                                          // clear all dlpf bits
    if (rate != INV_ICM20948_ACCEL_FILTER_1209HZ_NOLPF) {
        temp |= 0x01;                                      // set FCHOICE bit
        temp |= (rate & 0x07) << 3;                        // set DLPFCFG bits
    }
    inv_icm20948_write_register(IMU_ACCEL_CONFIG, temp);            // set new value
    return 0;
}

void inv_icm20948_config_gyro(uint8_t full_scale_select)
{
    uint8_t temp;
    // set the gyro full scale range
    temp = inv_icm20948_read_register(IMU_GYRO_CONFIG_1);   // get current value of register
    temp &= 0xf9;                                  // clear bit4 and bit3
    temp |= ((full_scale_select & 0x03) << 1);     // set desired bits to set range
    inv_icm20948_write_register(IMU_GYRO_CONFIG_1, temp);
    //NRF_LOG_INFO("Gyroscope FSR: %s", INV_ICM20948_GYRO_FSR_ASCII[(full_scale_select & 0x03)]);
}

void inv_icm20948_config_accel(uint8_t full_scale_select)
{
    uint8_t temp;
    // set the accelerometer full scale range
    temp = inv_icm20948_read_register(IMU_ACCEL_CONFIG);    // get current value of register
    temp &= 0xf9;                                  // clear bit2 and bit1
    temp |= ((full_scale_select & 0x03) << 1);     // set desired bits to set range
    inv_icm20948_write_register(IMU_ACCEL_CONFIG, temp);
    //NRF_LOG_INFO("Accelerometer FSR: %s", INV_ICM20948_ACCEL_FSR_ASCII[(full_scale_select & 0x03)]);
}



uint8_t inv_icm20948_get_device_id(void)
{
    return (inv_icm20948_read_register(IMU_WHO_AM_I));
}

void inv_icm20948_read_accel_xyz(int16_t *ax, int16_t *ay, int16_t *az)
{
  uint8_t  data_blk[6];

    inv_icm20948_read_register_block(IMU_ACCEL_XOUT_H, data_blk, 6);
    *ax = (data_blk[0] << 8) + data_blk[1];
    *ay = (data_blk[2] << 8) + data_blk[3];
    *az = (data_blk[4] << 8) + data_blk[5];
}

void inv_icm20948_read_gyro_xyz(int16_t *gx, int16_t *gy, int16_t *gz)
{
    uint8_t  data_blk[6];

    inv_icm20948_read_register_block(IMU_GYRO_XOUT_H, data_blk, 6);
    *gx = (data_blk[0] << 8) + data_blk[1];
    *gy = (data_blk[2] << 8) + data_blk[3];
    *gz = (data_blk[4] << 8) + data_blk[5];
}


void inv_icm20948_temperature(int16_t *temperature)
{
    uint8_t  data_blk[2];
    int16_t raw_temperature;

    // 读取两个字节的温度数据
    inv_icm20948_read_register_block(IMU_TEMP_OUT_H, data_blk, 2);
    raw_temperature = (data_blk[0] << 8) + data_blk[1]; // 合并高位和低位

    // 转换为摄氏度
    *temperature = raw_temperature / 333.87 + 21.0;
}




void inv_icm20948_read_imu(IMU_DATA *imu_data)
{
    // This function reads the axis data directly from the registers.
    uint8_t data_blk[20];

    // burst read starts at register ACCEL_XOUT_H for 14 8 bit registers
    inv_icm20948_read_register_block(IMU_ACCEL_XOUT_H, data_blk, 20);
    imu_data->time_stamp = inv_icm20948_get_time_us();
    //imu_data->deviceid = (uint32_t)inv_icm20948_get_device_id();

    imu_data->ax = (data_blk[0] << 8) + data_blk[1];
    imu_data->ay = (data_blk[2] << 8) + data_blk[3];
    imu_data->az = (data_blk[4] << 8) + data_blk[5];

    imu_data->gx = (data_blk[6]  << 8) + data_blk[7];
    imu_data->gy = (data_blk[8]  << 8) + data_blk[9];
    imu_data->gz = (data_blk[10] << 8) + data_blk[11];

    imu_data->temperature = (data_blk[12] << 8) + data_blk[13];

    imu_data->mx = 1; //(data_blk[14] << 8) + data_blk[15];
    imu_data->my = 2; //(data_blk[16] << 8) + data_blk[17];
    imu_data->mz = 3; //(data_blk[18] << 8) + data_blk[19];

    //printk("ax %d ay %d az %d\n", imu_data->ax, imu_data->ay, imu_data->az);
    //printk("gx %d gy %d gz %d\n", imu_data->gx, imu_data->gy, imu_data->gz);
    //printk("mx %d my %d mz %d\n", imu_data->mx, imu_data->my, imu_data->mz);
}

int16_t inv_icm20948_get_fifo_counter(void)
{
    uint8_t data_blk[2];
    inv_icm20948_read_register_block(IMU_FIFO_COUNTH, data_blk, 2);
    return ((data_blk[0] << 8) | data_blk[1]);
}

void inv_icm20948_read_imu_fifo(inv_icm20948_state *st, IMU_DATA *imu_data)
{
    uint8_t data_blk[32];
    uint16_t i, fifo_count, bytes_per_datum;

    fifo_count = inv_icm20948_get_fifo_counter();

    bytes_per_datum = st->chip_config->bytes_per_datum;
    if (fifo_count >= bytes_per_datum) {
        inv_icm20948_read_register_block(IMU_FIFO_R_W, data_blk, bytes_per_datum);
        imu_data->time_stamp = inv_icm20948_get_time_us();
        fifo_count -= bytes_per_datum;
    }
    if (fifo_count) {    // I only want the first set of data
        // reset FIFO
        inv_icm20948_write_register(IMU_FIFO_RST, 0x1F);
        inv_icm20948_write_register(IMU_FIFO_RST, 0x00);
    }

    i = 0;
    if (st->chip_config->accl_fifo_enable) {
        imu_data->ax = (data_blk[i+0] << 8) + data_blk[i+1];
        imu_data->ay = (data_blk[i+2] << 8) + data_blk[i+3];
        imu_data->az = (data_blk[i+4] << 8) + data_blk[i+5];
        i += 6;
    }
    if (st->chip_config->gyro_fifo_enable) {
        imu_data->gx = (data_blk[i+0] << 8) + data_blk[i+1];
        imu_data->gy = (data_blk[i+2] << 8) + data_blk[i+3];
        imu_data->gz = (data_blk[i+4] << 8) + data_blk[i+5];
        i += 6;
    }
    if (st->chip_config->temp_fifo_enable) {
        imu_data->temperature = (data_blk[i+0] << 8) + data_blk[i+1];
        i += 2;
    }
    if (st->chip_config->magn_fifo_enable) {
        imu_data->mx = 1; //(data_blk[i+0] << 8) + data_blk[i+1];
        imu_data->my = 2; //(data_blk[i+2] << 8) + data_blk[i+3];
        imu_data->mz = 3; //(data_blk[i+4] << 8) + data_blk[i+5];
        i += 6;
    }

    //printk("ax %d ay %d az %d\n", imu_data->ax, imu_data->ay, imu_data->az);
    //printk("gx %d gy %d gz %d\n", imu_data->gx, imu_data->gy, imu_data->gz);
    //printk("mx %d my %d mz %d\n", imu_data->mx, imu_data->my, imu_data->mz);
}

int16_t inv_icm20948_reset_fifo(inv_icm20948_state *st)
{
    uint8_t temp;

    // disable interrupts
    inv_icm20948_write_register(IMU_INT_ENABLE, 0x00);
    inv_icm20948_write_register(IMU_INT_ENABLE_1, 0x00);
    inv_icm20948_write_register(IMU_INT_ENABLE_2, 0x00);
    inv_icm20948_write_register(IMU_INT_ENABLE_3, 0x00);


    // disable the sensor output to FIFO
    inv_icm20948_write_register(IMU_FIFO_EN_1, 0x00);
    inv_icm20948_write_register(IMU_FIFO_EN_2, 0x00);


    // disable fifo reading
    inv_icm20948_write_register(IMU_USER_CTRL, 0x00);


    // reset FIFO
    inv_icm20948_write_register(IMU_FIFO_RST, 0x1F);
    inv_icm20948_write_register(IMU_FIFO_RST, 0x00);

    // enable interrupt
    if (   st->chip_config->accl_fifo_enable
        || st->chip_config->gyro_fifo_enable
        || st->chip_config->magn_fifo_enable
        || st->chip_config->temp_fifo_enable) {
        inv_icm20948_write_register(IMU_INT_ENABLE_1, IMU_BIT_RAW_DATA_0_RDY_EN);
    }
    //inv_icm20948_write_register(IMU_INT_ENABLE_2, 0x01);    // enable for testing

    // enable FIFO reading and I2C master interface
    inv_icm20948_write_register(IMU_USER_CTRL, IMU_BIT_FIFO_EN);

    // enable sensor output to FIFO
    temp = 0;
    if (st->chip_config->gyro_fifo_enable)
        temp |= IMU_BIT_GYRO_FIFO_EN;
    if (st->chip_config->accl_fifo_enable)
        temp |= IMU_BIT_ACCEL_FIFO_EN;
    if (st->chip_config->temp_fifo_enable)
        temp |= IMU_BIT_TEMP_FIFO_EN;
    inv_icm20948_write_register(IMU_FIFO_EN_2, temp);

    // need to add support for temperature and magnetometer

    return 0;
}





/***********************************************************************/
/*                                                                     */
/* Mag                                              */
/*                                                                     */
/***********************************************************************/

//uint16_t inv_icm20948_read_register_16(uint16_t reg)
//{
//    uint8_t data[2];
//    inv_icm20948_read_register_block(reg, data, 2);
//    return ((uint16_t)data[0] << 8) | data[1];
//}

//void inv_icm20948_enable_mag_data_read(uint8_t reg, uint8_t bytes)
//{
//    inv_icm20948_write_register(IMU_I2C_SLV0_ADDR, AK09916_I2C_ADDR | AK09916_READ); // Read from AK09916
//    inv_icm20948_write_register(IMU_I2C_SLV0_REG, reg);                              // AK09916 register to read
//    inv_icm20948_write_register(IMU_I2C_SLV0_CTRL, 0x80 | bytes);  // Enable read, number of bytes
//    inv_icm20948_sleep_us(10000); // Delay 10ms
//}

//uint16_t inv_icm20948_read_ak09916_register_16(uint8_t reg)
//{
//    uint16_t regValue = 0;
//    inv_icm20948_enable_mag_data_read(reg, 0x02);
//    //regValue = inv_icm20948_read_register_16(IMU_EXT_SLV_SENS_DATA_00);
//    inv_icm20948_enable_mag_data_read(AK09916_HXL, 0x08);
//    //regValue = inv_icm20948_read_register_16(IMU_EXT_SLV_SENS_DATA_00);
//    return regValue;
//}


//uint8_t inv_icm20948_read_ak09916_register_8(uint8_t reg)
//{
//    inv_icm20948_enable_mag_data_read(reg, 0x01);
//    inv_icm20948_enable_mag_data_read(AK09916_HXL, 0x08);
//    uint8_t regVal = inv_icm20948_read_register(IMU_EXT_SLV_SENS_DATA_00);
//    return regVal;
//}


//void inv_icm20948_write_ak09916_register_8(uint8_t reg, uint8_t val)
//{
//    inv_icm20948_write_register(IMU_I2C_SLV0_ADDR, AK09916_I2C_ADDR); // Write to AK09916
//    inv_icm20948_write_register(IMU_I2C_SLV0_REG, reg);               // AK09916 register to write
//    inv_icm20948_write_register(IMU_I2C_SLV0_DO, val);                // Data to write
//    inv_icm20948_write_register(IMU_I2C_SLV0_CTRL, IMU_BIT_SLV0_EN | 0x01); //根据Hal库更改
//    //inv_icm20948_sleep_us(10000); // Delay 10ms
//}

//void inv_icm20948_read_ak09916_register_8(uint8_t reg, uint8_t val)
//{
//    inv_icm20948_write_register(IMU_I2C_SLV0_ADDR, AK09916_I2C_ADDR); // Write to AK09916
//    inv_icm20948_write_register(IMU_I2C_SLV0_REG, reg);               // AK09916 register to write
//    inv_icm20948_write_register(IMU_I2C_SLV0_DO, val);                // Data to write
//    inv_icm20948_write_register(IMU_I2C_SLV0_CTRL, IMU_BIT_SLV0_EN | 0x01); //根据Hal库更改
//    //inv_icm20948_sleep_us(10000); // Delay 10ms
//}


//uint16_t inv_icm20948_who_am_i_mag(void)
//{
//    // Read 16-bit WHO_AM_I from the magnetometer
//    return inv_icm20948_read_ak09916_register_16(AK09916_WIA_1);
//}


bool ak09916_who_am_i(void){

uint8_t ak09916_id = read_single_ak09916_reg(0x01);

	if(ak09916_id == 0x09)
		return true;
	else
		return false;

}




void inv_icm20948_set_mag_op_mode(uint8_t opMode)
{
    write_single_ak09916_reg(AK09916_CNTL2, opMode);
    inv_icm20948_sleep_us(10000); // Delay 10ms
    //if (opMode != AK09916_PWR_DOWN) {
        
    //}
}

void inv_icm20948_reset_mag(void)
{
    write_single_ak09916_reg(AK09916_CNTL3, 0x01);
    inv_icm20948_sleep_us(100000); // Delay 100ms
}



void inv_icm20948_enable_i2c_master(void)
{
    //// Enable I2C master
    //inv_icm20948_write_register(IMU_USER_CTRL, IMU_BIT_I2C_MST_EN);
    //// Set I2C master clock to 345.60 kHz
    //inv_icm20948_write_register(IMU_I2C_MST_CTRL, 0x07);
      uint8_t new_val = inv_icm20948_read_register(IMU_USER_CTRL);
	new_val |= 0x20;

	inv_icm20948_write_register(IMU_USER_CTRL, new_val);
	inv_icm20948_sleep_us(100000);

 //       uint8_t new_val1 = inv_icm20948_read_register(IMU_I2C_MST_CTRL);
	//new_val1 |= 0x07;

	//inv_icm20948_write_register(IMU_I2C_MST_CTRL, new_val1);	
 //       inv_icm20948_sleep_us(10000);
}


void inv_icm20948_i2c_clk_frq(void)
{

        uint8_t new_val1 = inv_icm20948_read_register(IMU_I2C_MST_CTRL);
	new_val1 |= 0x07;

	inv_icm20948_write_register(IMU_I2C_MST_CTRL, new_val1);	

}

void inv_icm20948_i2c_master_reset(void)
{
    uint8_t regVal = inv_icm20948_read_register(IMU_USER_CTRL);
    regVal |= IMU_BIT_I2C_MST_RST;
    inv_icm20948_write_register(IMU_USER_CTRL, regVal);
    //inv_icm20948_sleep_us(10000); // Delay 10ms
    
}

//void inv_icm20948_reset(void)
//{
//    inv_icm20948_write_register(IMU_PWR_MGMT_1, IMU_BIT_DEVICE_RESET);
//    inv_icm20948_sleep_us(10000); // Wait for registers to reset
//}

//void inv_icm20948_wake(bool sleep_mode)
//{
//    uint8_t regVal = inv_icm20948_read_register(IMU_PWR_MGMT_1);
//    if (sleep_mode) {
//        regVal |= IMU_BIT_SLEEP;
//    } else {
//        regVal &= ~IMU_BIT_SLEEP;
//    }
//    inv_icm20948_write_register(IMU_PWR_MGMT_1, regVal);
//}


void inv_icm20948_init_magnetometer(void){  //Hal 库里的
    
    inv_icm20948_i2c_master_reset();
    inv_icm20948_enable_i2c_master();
    inv_icm20948_i2c_clk_frq();
    while(!ak09916_who_am_i());
    inv_icm20948_reset_mag();
    NRF_LOG_INFO("shi 0x%02x",read_single_ak09916_reg(0x01));
    inv_icm20948_set_mag_op_mode(AK09916_CONTINUOUS_MODE_100HZ);


}




//int16_t inv_icm20948_init_magnetometer(void)
//{
//    uint8_t tries = 0;
//    bool initSuccess = false;
//    uint16_t whoAmI;

//    // Enable I2C Master Interface


//    inv_icm20948_enable_i2c_master();

//    // Reset Magnetometer
//    inv_icm20948_reset_mag();

//    // Wake up the device
//    //inv_icm20948_wake(false);

//    // Align ODR
//    inv_icm20948_write_register(IMU_ODR_ALIGN_EN, 0x01);

//    while (!initSuccess && (tries < 10)) {
//        inv_icm20948_sleep_us(10000); // Delay 10ms
//        inv_icm20948_enable_i2c_master();
//        inv_icm20948_sleep_us(10000); // Delay 10ms

//        whoAmI = inv_icm20948_who_am_i_mag();
//        if (!((whoAmI == AK09916_WHO_AM_I_1) || (whoAmI == AK09916_WHO_AM_I_2))) {
//            initSuccess = false;
//            inv_icm20948_i2c_master_reset();
//            tries++;
//        } else {
//            initSuccess = true;
//            NRF_LOG_INFO("AK09916 initSuccess!") 
//        }
//    }

//    if (initSuccess) {
//        inv_icm20948_set_mag_op_mode(AK09916_CONTINUOUS_MODE_100HZ);
       
//    }

//    return initSuccess ? 0 : -1;
//}


//void inv_icm20948_read_magn_xyz(int16_t *mx, int16_t *my, int16_t *mz)
//{
//    uint8_t data_blk[6];


//    // 读取磁力计数据块 (6字节: X轴, Y轴, Z轴各占2字节)
//    inv_icm20948_read_register_block(IMU_EXT_SLV_SENS_DATA_00  , data_blk, 6); // 从IMU_EXT_SLV_SENS_DATA_00地址 0x3B 开始读取

//    // 合并低字节和高字节，存储到 mx, my, mz
//    *mx = (int16_t)((data_blk[1] << 8) | data_blk[0]); // X轴数据（低字节在前，高字节在后）
//    *my = (int16_t)((data_blk[3] << 8) | data_blk[2]); // Y轴数据
//    *mz = (int16_t)((data_blk[5] << 8) | data_blk[4]); // Z轴数据


//}

void inv_icm20948_read_magn_xyz(int16_t *mx, int16_t *my, int16_t *mz){

	uint8_t data_blk[6];
	

        inv_icm20948_write_register(IMU_I2C_SLV0_ADDR, AK09916_I2C_ADDR | AK09916_READ);
        inv_icm20948_write_register(IMU_I2C_SLV0_REG, AK09916_HXL);
        inv_icm20948_write_register(IMU_I2C_SLV0_CTRL, 0x80|0x08); // 启用，读取len字节

        inv_icm20948_sleep_us(1000); // 等待数据被读取

        // 从扩展寄存器块读取数据
        inv_icm20948_read_register_block(IMU_EXT_SLV_SENS_DATA_00, data_blk, 6);


        *mx = (int16_t)((data_blk[1] << 8) | data_blk[0]); // X轴数据（低字节在前，高字节在后）
        *my = (int16_t)((data_blk[3] << 8) | data_blk[2]); // Y轴数据
        *mz = (int16_t)((data_blk[5] << 8) | data_blk[4]); // Z轴数据

	
}





/***********************************************************************/
/*                                                                     */
/* Support functions                                                   */
/*                                                                     */
/***********************************************************************/

static uint8_t current_bank = 0xff;

static void inv_icm20948_set_bank(uint16_t reg)
{
    uint8_t bank = (reg & 0xff00) >> 4;
    if (bank != current_bank) {
        inv_icm20948_i2c_write_reg(IMU_REG_BANK_SEL, bank);
        current_bank = bank;
    }
}

void inv_icm20948_write_register(uint16_t reg, uint8_t value)
{
    inv_icm20948_set_bank(reg);
    inv_icm20948_i2c_write_reg((uint8_t)(reg&0xff), value);
}

void inv_icm20948_write_register_block(uint16_t reg, uint8_t *block, uint8_t count)
{
    inv_icm20948_set_bank(reg);
    inv_icm20948_i2c_write_reg_block((uint8_t)(reg&0xff), block, count);
}

uint8_t inv_icm20948_read_register(uint16_t reg)
{
    uint8_t value;
    inv_icm20948_set_bank(reg);
    inv_icm20948_i2c_read_reg((uint8_t)(reg&0xff), &value);
    return value;
}

void inv_icm20948_read_register_block(uint16_t reg, uint8_t *block, uint8_t count)
{
    inv_icm20948_set_bank(reg);
    inv_icm20948_i2c_read_reg_block((uint8_t)(reg&0xff), block, count);
}
//新加


uint8_t read_single_ak09916_reg(uint8_t reg)
{
    // 设置ICM-20948的I2C主机以从AK09916读取数据
    inv_icm20948_write_register(IMU_I2C_SLV0_ADDR, AK09916_I2C_ADDR | AK09916_READ);
    inv_icm20948_write_register(IMU_I2C_SLV0_REG, reg);
    //inv_icm20948_write_register(IMU_I2C_SLV0_DO, 0xFF);
    inv_icm20948_write_register(IMU_I2C_SLV0_CTRL, 0x81); // 启用，读取1字节

    inv_icm20948_sleep_us(1000); // 等待数据被读取

    // 从扩展寄存器读取数据
    return inv_icm20948_read_register(IMU_EXT_SLV_SENS_DATA_00);
}

void write_single_ak09916_reg(uint8_t reg, uint8_t val)
{
    // 设置ICM-20948的I2C主机以向AK09916写入数据
    inv_icm20948_write_register(IMU_I2C_SLV0_ADDR,  AK09916_I2C_ADDR | AK09916_WRITE);
    inv_icm20948_write_register(IMU_I2C_SLV0_REG, reg);
    inv_icm20948_write_register(IMU_I2C_SLV0_DO, val);
    //inv_icm20948_write_register(IMU_I2C_SLV0_CTRL, 0x81); // 启用，写入1字节


    //inv_icm20948_sleep_us(1000); // 等待写入完成
}

//uint8_t* read_multiple_ak09916_reg(uint8_t reg, uint8_t len)
//{
//    // 设置ICM-20948的I2C主机以从AK09916读取多个数据

//    inv_icm20948_write_register(IMU_I2C_SLV0_ADDR, AK09916_I2C_ADDR | AK09916_READ);
//    inv_icm20948_write_register(IMU_I2C_SLV0_REG, reg);
//    inv_icm20948_write_register(IMU_I2C_SLV0_CTRL, IMU_BIT_SLV0_EN | len); // 启用，读取len字节

//    inv_icm20948_sleep_us(1000); // 等待数据被读取

//    // 从扩展寄存器块读取数据
//      inv_icm20948_read_register_block(IMU_EXT_SLV_SENS_DATA_00, multiple_ak09916, len);

//     return multiple_ak09916;
//}