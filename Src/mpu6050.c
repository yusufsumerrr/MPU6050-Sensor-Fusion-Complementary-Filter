#include "main.h"
#include "mpu6050.h"
#include "math.h"
#include <stdio.h>
#include <string.h>

extern I2C_HandleTypeDef hi2c1;
extern UART_HandleTypeDef huart2;

int16_t x_acc, y_acc, z_acc;
int16_t x_gyr, y_gyr, z_gyr;
float gx_bias = 0.0f, gy_bias = 0.0f, gz_bias = 0.0f;
float phiGyro =0.0f, thetaGyro = 0.0f;
float phiComp =0.0f, thetaComp = 0.0f;

void mpu6050_init()
{
	// HAL_I2C_IsDeviceReady
	HAL_StatusTypeDef ret = HAL_I2C_IsDeviceReady(&hi2c1, DEVICE_ADDRESS << 1, 1, 100);
	if (ret == HAL_OK) {
		printf(" Hello from MPU6050...\n");
	} else {
		printf("E R R O R");
	}

	// Register 27 – Gyroscope Configuration
	uint8_t temp_data = FS_GYRO_500;
	ret = HAL_I2C_Mem_Write(&hi2c1, DEVICE_ADDRESS << 1, REF_CONFIG_GYRO, 1, (uint8_t*) &temp_data, 1, 100);
	if (ret == HAL_OK) {
		printf(" Successfully written to register 27 \n");
	} else {
		printf("E R R O R\n");
	}

	// Register 28 – Accelerometer Configuration
	temp_data = FS_ACC_4G;
	ret = HAL_I2C_Mem_Write(&hi2c1, DEVICE_ADDRESS << 1, REF_CONFIG_ACC, 1, (uint8_t*) &temp_data, 1, 100);
	if (ret == HAL_OK) {
		printf(" Successfully written to register 28 \n");
	} else {
		printf(" E R R O R\n");
	}

	// Register 107 – Power Management 1
	temp_data = 0;
	ret = HAL_I2C_Mem_Write(&hi2c1, DEVICE_ADDRESS << 1, REF_CONFIG_CTRL, 1, (uint8_t*) &temp_data, 1, 100);
	if (ret == HAL_OK) {
		printf(" Power Mode\n");
	} else {
		printf("Sleep Mode\n");
	}
}



void gyro_calibrate()
{
	HAL_Delay(100);
	// While holding the gyro steady, collect incoming measurements to calculate the average bias (offset).
	for (int i = 0; i < N; i++)
	{
		mpu6050_read();
		gx_bias += x_gyr;
		gy_bias += y_gyr;
		gz_bias += z_gyr;
		HAL_Delay(10);   			// Every Sample 10ms, 100 sample in 1 second, Sample rate = 100 Hz
	}

	gx_bias /= N; 					// average bias (offset) for X-axis
	gy_bias /= N; 					// average bias (offset) for Y-axis
	gz_bias /= N; 					// average bias (offset) for Z-axis
}


void mpu6050_read()
{
	/*------------------- A C C E L E R O M E T E R --------------------------*/
	uint8_t datax[2];
	HAL_I2C_Mem_Read(&hi2c1, (DEVICE_ADDRESS << 1) + 1, REF_DATA, 1, datax, 2, 100);
	x_acc = ((uint8_t) datax[0] << 8) + datax[1];

	uint8_t datay[2];
	HAL_I2C_Mem_Read(&hi2c1, (DEVICE_ADDRESS << 1) + 1, REF_DATA + 2, 1, datay, 2, 100);
	y_acc = ((uint8_t) datay[0] << 8) + datay[1];

	uint8_t dataz[2];
	HAL_I2C_Mem_Read(&hi2c1, (DEVICE_ADDRESS << 1) + 1, REF_DATA + 4, 1, dataz, 2, 100);
	z_acc = ((uint8_t) dataz[0] << 8) + dataz[1];


	/*----------------------- G Y R O S C O P E ------------------------------*/
	HAL_I2C_Mem_Read(&hi2c1, (DEVICE_ADDRESS << 1) + 1, REF_DATA_GYRO, 1, datax, 2, 100);
	x_gyr = ((uint8_t) datax[0] << 8) + datax[1];

	HAL_I2C_Mem_Read(&hi2c1, (DEVICE_ADDRESS << 1) + 1, REF_DATA_GYRO + 2, 1, datay, 2, 100);
	y_gyr = ((uint8_t) datay[0] << 8) + datay[1];

	HAL_I2C_Mem_Read(&hi2c1, (DEVICE_ADDRESS << 1) + 1, REF_DATA_GYRO + 4, 1, dataz, 2, 100);
	z_gyr = ((uint8_t) dataz[0] << 8) + dataz[1];

}


void euler_angle(float dt_val)
{
	char uartBuffer[150];

	// --- ACC NORMALIZE --- LSB / (LSB/g) = g  --- ACC_SENS = 8192.0f for ±4g ---
	float ax = (float) x_acc / ACC_SENS;
	float ay = (float) y_acc / ACC_SENS;
	float az = (float) z_acc / ACC_SENS;

	// --- GYRO NORMALIZE --- LSB / (LSB/(°/s)) = °/s  --- GYRO_SENS = 65.5f for ±500 °/s ---
	float gx = ((float) x_gyr - gx_bias) / GYRO_SENS * DEG_TO_RAD;
	float gy = ((float) y_gyr - gy_bias) / GYRO_SENS * DEG_TO_RAD;
	float gz = ((float) z_gyr - gz_bias) / GYRO_SENS * DEG_TO_RAD;

	// --- Roll & Pitch from ACC ---
    float phiAcc = atan2f(ay, sqrtf(ax * ax + az * az));
    float thetaAcc = atan2f(-ax, sqrtf(ay * ay + az * az));

    // --- Roll & Pitch Rates from GYRO ---
    float phiHatRaw = gx + tanf(thetaGyro) * (sinf(phiGyro) * gy + cosf(phiGyro) * gz);
    float thetaHatRaw = cosf(phiGyro) * gy - sinf(phiGyro) * gz;
    phiGyro += phiHatRaw * dt_val;
    thetaGyro += thetaHatRaw * dt_val;

    // --- Roll & Pitch Rates from COMPLEMENTARY FILTER ---
    float phiHatFiltered = gx + tanf(thetaComp) * (sinf(phiComp) * gy + cosf(phiComp) * gz);
    float thetaHatFiltered = cosf(phiComp) * gy - sinf(phiComp) * gz;

    float alpha = 0.98f;
    phiComp = alpha * (phiComp + phiHatFiltered * dt_val) + (1.0f - alpha) * phiAcc;
    thetaComp = alpha * (thetaComp + thetaHatFiltered * dt_val) + (1.0f - alpha) * thetaAcc;

    // --- UART data transmission ---
    snprintf(uartBuffer, sizeof(uartBuffer),
             "%.2f %.2f %.2f %.2f %.2f %.2f\r\n",
             phiAcc * RAD_TO_DEG,   -thetaAcc * RAD_TO_DEG,    // ACC
			 phiGyro * RAD_TO_DEG,  -thetaGyro * RAD_TO_DEG,   // GYRO
             phiComp * RAD_TO_DEG,  -thetaComp * RAD_TO_DEG    // FILTER
             );
    HAL_UART_Transmit(&huart2, (uint8_t*) uartBuffer, strlen(uartBuffer), HAL_MAX_DELAY);

}




