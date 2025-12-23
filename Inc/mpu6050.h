
#ifndef INC_MPU6050_H_
#define INC_MPU6050_H_

#define DEVICE_ADDRESS 0x68 //1101000 (Slave address)

#define FS_GYRO_250 0
#define FS_GYRO_500 8
#define FS_GYRO_1000 9
#define FS_GYRO_2000 10

#define FS_ACC_2G 0
#define FS_ACC_4G 8
#define FS_ACC_8G 9
#define FS_ACC_16G 10

#define REF_CONFIG_GYRO 27
#define REF_CONFIG_ACC 28
#define REF_CONFIG_CTRL 107
#define REF_DATA 59
#define REF_DATA_GYRO 67

#define ACC_SENS   8192.0f    // ±4g
#define GYRO_SENS  65.5f      // ±500 degree/second

#define DEG_TO_RAD 0.01745329251f
#define RAD_TO_DEG 57.2957795f
#define N 100


void mpu6050_init();
void gyro_calibrate();
void mpu6050_read();
void euler_angle(float dt_val);
#endif /* INC_MPU6050_H_ */
