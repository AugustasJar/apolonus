/*
 * imu_f1.c
 *
 *  Created on: Mar 18, 2023
 *      Author: augustas
 */
#include "imu_f1.h"
#include "math.h"

#define imu_ADDR 0xD0
#define WHO_AM_I_REG 0x75
#define PWR_MGMT_1_REG 0x6B
#define SMPLRT_DIV_REG 0x19
#define LOWPASS_CONFIG_REG 0x1A
#define ACCEL_CONFIG_REG 0x1C
#define ACCEL_XOUT_H_REG 0x3B
#define TEMP_OUT_H_REG 0x41
#define GYRO_CONFIG_REG 0x1B
#define GYRO_XOUT_H_REG 0x43
#define PI 3.14159265359
#define rad2deg  57.29567015
const uint8_t TIMEOUT = 100;
// initialization function, returns 0 if successful, otherwise 1.
uint8_t imu_init(I2C_HandleTypeDef *I2Cx){
    uint8_t check;
    uint8_t data;

    HAL_I2C_Mem_Read(I2Cx, imu_ADDR, WHO_AM_I_REG, 1, &check, 1, TIMEOUT);

    if (check == 104) // waiting for init flag.
    {
        // reset internal registers, set to use internal 8MHz clock.
        data = 0b00000000;
        HAL_I2C_Mem_Write(I2Cx, imu_ADDR, PWR_MGMT_1_REG, 1, &data, 1, TIMEOUT);

        // set the sample rate divider to 1kHz
        data = 0x07;
        HAL_I2C_Mem_Write(I2Cx, imu_ADDR, SMPLRT_DIV_REG, 1, &data, 1, TIMEOUT);

        // Set accelerometer configuration in ACCEL_CONFIG Register
        // XA_ST=0,YA_ST=0,ZA_ST=0, FS_SEL=0 -> � 2g
        data = 0x00;
        HAL_I2C_Mem_Write(I2Cx, imu_ADDR, ACCEL_CONFIG_REG, 1, &data, 1, TIMEOUT);

        // low pass filter configuration.
        // set to 260Hz bandwith for minimum delay.
        data = 0b00001000;
        HAL_I2C_Mem_Write(I2Cx, imu_ADDR, GYRO_CONFIG_REG, 1, &data, 1, TIMEOUT);

        //set the lowpass filter to 100Hz;
        data = 0b0000100;
		HAL_I2C_Mem_Write(I2Cx, imu_ADDR, LOWPASS_CONFIG_REG, 1, &data, 1, TIMEOUT);
        return 0;
    }
    return 1;
}
// calculates gyroscope bias
void imu_tune(I2C_HandleTypeDef *I2Cx, imu_t *dataStore) {

	uint8_t tempData[6];
	int16_t i= 1,i_max = 1000;
	int16_t x,y,z;

	dataStore->K_gyr = 0.94;
	dataStore->K_acc = 1-dataStore->K_gyr;
	dataStore->th_prev = 0;
	dataStore->gyro_x_raw_bias = 0;
	dataStore->gyro_y_raw_bias = 0;
	dataStore->gyro_z_raw_bias = 0;

	while (i <= i_max) {
		HAL_I2C_Mem_Read(I2Cx, imu_ADDR, GYRO_XOUT_H_REG, 1, &tempData, 6, TIMEOUT);

		x = (int16_t)(tempData[0] << 8 | tempData[1]);
		y = (int16_t)(tempData[2] << 8 | tempData[3]);
		z = (int16_t)(tempData[4] << 8 | tempData[5]);

		//iterative mean update.
		dataStore->gyro_x_raw_bias = dataStore->gyro_x_raw_bias + (x - dataStore->gyro_x_raw_bias)/(i+1);
		dataStore->gyro_y_raw_bias = dataStore->gyro_y_raw_bias + (y - dataStore->gyro_y_raw_bias)/(i+1);
		dataStore->gyro_z_raw_bias = dataStore->gyro_z_raw_bias + (z - dataStore->gyro_z_raw_bias)/(i+1);
		i++;
		HAL_Delay(5);
	}

}
void imu_read(I2C_HandleTypeDef *I2Cx, imu_t *dataStore) {

	uint8_t tempData[14];
	const double AccLsbSensitivity = 16384; // conversion factor to g, based on config.
	const double gyroLsbSensitivity = 131; // conversion factor to rad/s

	dataStore->x_prev = dataStore->gx;
	// read 14 data registers in a row.
	HAL_I2C_Mem_Read(I2Cx, imu_ADDR, ACCEL_XOUT_H_REG, 1, &tempData, 14, TIMEOUT);

	/*
		one measurement is 16bits, but stored in two 8bit registers,
		therefore, they need to be concatenated.
	*/
	dataStore->acc_x_raw = (int16_t)(tempData[0] << 8 | tempData[1]);
	dataStore->acc_y_raw = (int16_t)(tempData[2] << 8 | tempData[3]);
	dataStore->acc_z_raw = (int16_t)(tempData[4] << 8 | tempData[5]);

	dataStore->temp_raw = (int16_t)(tempData[6] << 8 | tempData[7]);

	dataStore->gyro_x_raw = (int16_t)(tempData[8] << 8 | tempData[9]) - dataStore->gyro_x_raw_bias;
	dataStore->gyro_y_raw = (int16_t)(tempData[10] << 8 | tempData[11]) - dataStore->gyro_y_raw_bias;
	dataStore->gyro_z_raw = (int16_t)(tempData[12] << 8 | tempData[13]) - dataStore->gyro_z_raw_bias;

	//convert to g
	dataStore->ax = ((double)dataStore->acc_x_raw)/AccLsbSensitivity;
	dataStore->ay = ((double)dataStore->acc_y_raw/AccLsbSensitivity);
	dataStore->az = ((double)dataStore->acc_z_raw/AccLsbSensitivity);

	// convert to Celsius
	dataStore->temp = (double)(dataStore->temp_raw/340 + 36.53);

	//convert to rad/s
	dataStore->gx = (dataStore->gyro_x_raw/gyroLsbSensitivity);
	dataStore->gy = (double)(dataStore->gyro_y_raw/gyroLsbSensitivity);
	dataStore->gz = (dataStore->gyro_z_raw/gyroLsbSensitivity);
}
void imu_complimentary(imu_t *dataStore, uint32_t dt) {

	dataStore->th = dataStore->K_acc*atan2(dataStore->ax,sqrt(pow(dataStore->ay,2) + pow(dataStore->az,2))) + dataStore->K_gyr*(dataStore->th_prev + dataStore->gy*dt);
	dataStore->th_prev = dataStore->th;
}
void imu_kalman(imu_t *dataStore) {

	// calculate state extrapolation


}
