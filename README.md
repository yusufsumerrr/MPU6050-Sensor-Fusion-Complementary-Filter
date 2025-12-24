# MPU6050-Sensor-Fusion-Complementary-Filter
This project performs real-time attitude estimation using data from the MPU6050 Inertial Measurement Unit (IMU). The process begins with the calculation of the fundamental Euler angles (`Roll` and `Pitch`) using the gravity components obtained from the accelerometer and the angular velocity data from the gyroscope. These raw angle estimates are then combined using a `Complementary Filter` algorithm, which optimizes the high-frequency noise of the accelerometer and the time-accumulated drift error of the gyroscope. As a result of sensor fusion, the stabilized orientation data are transmitted via serial communication and visualized in real time as a 3D representation using a Python/PyGame-based interface to evaluate the system’s dynamic performance.

![WhatsApp Video 2025-12-21 at 01 58 11 (1)](https://github.com/user-attachments/assets/199afc61-f386-4e9e-9ac5-62977e6f4f45)

---

### 1. What is MPU6050?
MPU6050 is a 6-axis (6-DOF) motion tracking sensor that integrates a 3-axis accelerometer and a 3-axis gyroscope in a single package.

1. `3-Axis Accelerometer` ($m/s^2$)
    - Measures linear acceleration (change in linear velocity) along the X, Y, and Z axes.
    - Enables estimation of the device’s tilt angle (static orientation) relative to the ground by tracking the gravity vector.
    - Stable in the long term and does not suffer from drift.
    - Highly sensitive to vibrations and sudden movements (noise).
    - Adjustable acceleration range between ±2g, ±4g, ±8g, and ±16g.

2. ``3-Axis Gyroscope`` ($rad/s$)
    - Measures the angular velocity of the device around its own axes.
    - Captures rapid motions and orientation changes with high temporal accuracy.
    - Resistant to vibrations and provides smooth data.
    - Suffers from drift due to the accumulation of small errors over time; even when the device is stationary, the estimated angle may slowly change.
    - Programmable range of ±250°/s, ±500°/s, ±1000°/s, ±2000°/s.

---

### 2. MPU6050 Pins

<img width="197" height="256" alt="images" src="https://github.com/user-attachments/assets/6290fc2a-ce72-4699-a37f-b5923b8fe8d7" />

- ``VCC:``Power supply input. It is typically powered with 3.3 V or 5 V.
- ``GND:``Ground connection.
- ``SCL (Serial Clock Line):``Serial clock line. It carries the clock signal that synchronizes the timing and speed of data transmission.
- ``SDA (Serial Data Line):``Bidirectional serial data line. Accelerometer and gyroscope data from the sensor are transmitted to the microcontroller through this line.
- ``XDA & XCL (Auxiliary Data/Clock):``Auxiliary I2C lines. They are used to connect a secondary sensor, such as a magnetometer (compass), to the MPU6050 and allow the sensor to process this data directly. In standard applications, these pins are usually left unconnected.
- ``AD0 (Address Select):``Used to select the I2C address of the sensor. This pin allows two MPU6050 devices to be connected on the same I2C bus.
    - **AD0 = 0:** Device address is ``0x68``.
    - **AD0 = 1:** Device address is ``0x69``.
- ``INT (Interrupt):``Interrupt output pin. It sends a signal to the microcontroller when data is ready or when motion is detected. This eliminates the need for continuous polling, improving processing efficiency and reducing power consumption.

---

### 3. MPU6050 - STM32CubeIDE Configuration
We enable the I2C mode to activate the SDA and SCL pins.

<img width="1213" height="613" alt="Pasted image 20251219180721" src="https://github.com/user-attachments/assets/c5005bac-1e09-4927-bfe6-6f4e292c2bbb" />

- MPU6050_SCL -> PA15 -> I2C1_SCL  
- MPU6050_SDA ->  PB7 -> I2C1_SDA 
- MPU6050_VCC -> BOARD_+5V
- MPU6050_GND -> BOARD_GND

---

### 🛠️ 4. How It Works?

```C
/* USER CODE BEGIN 2 */
mpu6050_init();
gyro_calibrate();
/* USER CODE END 2 */
```
At system startup, we initialize the functions that we have defined. Now, let’s go through these functions. First, we start with the `mpu6050_init()` function.

```c
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

void mpu6050_init() 
{
	// HAL_I2C_IsDeviceReady
	HAL_StatusTypeDef ret = HAL_I2C_IsDeviceReady(&hi2c1, DEVICE_ADDRESS << 1, 1, 100);
	if (ret == HAL_OK) 
	{
		printf(" Hello from MPU6050...\n");
	} 
	else 
	{
		printf("E R R O R");
	}
	
	// Register 27 – Gyroscope Configuration
	uint8_t temp_data = FS_GYRO_500; // ±500°/s
	ret = HAL_I2C_Mem_Write(&hi2c1, DEVICE_ADDRESS << 1, REF_CONFIG_GYRO, 1, (uint8_t*) &temp_data, 1, 100);
	if (ret == HAL_OK)
	{
		printf(" Successfully written to register 27. \n");
	} 
	else 
	{
		printf("E R R O R\n");
	}
	
	// Register 28 – Accelerometer Configuration
	temp_data = FS_ACC_4G;
	ret = HAL_I2C_Mem_Write(&hi2c1, DEVICE_ADDRESS << 1, REF_CONFIG_ACC, 1, (uint8_t*) &temp_data, 1, 100);
	if (ret == HAL_OK) 
	{
		printf(" Successfully written to register 28.\n");
	} 
	else 
	{
		printf(" E R R O R\n");
	}
	
	// Register 107 – Power Management 1
	temp_data = 0;
	ret = HAL_I2C_Mem_Write(&hi2c1, DEVICE_ADDRESS << 1, REF_CONFIG_CTRL, 1, (uint8_t*) &temp_data, 1, 100);
	if (ret == HAL_OK) 
	{
		printf(" Power Mode\n");
	} 
	else 
	{
		printf(" Sleep Mode\n");
	}
}
```
1.The `HAL_I2C_IsDeviceReady` function is a check mechanism that verifies whether the target device (MPU6050) on the STM32 I2C bus is physically present and ready for communication.

<img width="785" height="47" alt="Pasted image 20251202151341" src="https://github.com/user-attachments/assets/0127cdf1-d8c0-453b-8f8f-7bdc337f9665" />

2.``Gyroscope configuration``is the process of defining the sensitivity range and resolution at which the sensor measures angular velocity (rate of rotation per unit time). In this project, the gyroscope sensitivity is set to `±500°/s`.

<img width="1290" height="167" alt="Pasted image 20251221181714" src="https://github.com/user-attachments/assets/510ec7c7-67f8-4696-bb9e-4e0babafede8" />

<img width="398" height="183" alt="Pasted image 20251221181513" src="https://github.com/user-attachments/assets/53f7a73b-d921-4754-85d4-38d619dd4493" />

3.``Accelerometer configuration``is the process of defining the dynamic range over which the sensor measures linear acceleration (including gravity and motion-induced acceleration) and the resolution of this data. This configuration limits the maximum G-force (gravitational acceleration unit) that the sensor can measure. In this project, the accelerometer sensitivity is set to `±4g`.

<img width="1290" height="169" alt="Pasted image 20251221182033" src="https://github.com/user-attachments/assets/9d74634c-c8f9-405d-ae67-89b37b0fa256" />

<img width="377" height="192" alt="Pasted image 20251221182111" src="https://github.com/user-attachments/assets/5fa6a683-748f-49ca-b03f-27589c50b392" />

4.The MPU6050 comes from the factory with Sleep Mode enabled to save power. To activate the sensor’s internal oscillator and measurement units, the device is woken up by writing `0` to the `Power Management 1` register.

---

Now let’s talk about the `gyro_calibrate()` function.

```c
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
```
- Gyroscope sensors inherently exhibit a systematic error known as bias, which causes the output to deviate from zero even when the sensor is stationary. The `gyro_calibrate()` function implemented in this project is executed at system startup, during a period when the sensor is assumed to be completely stable, in order to eliminate this error. The function collects 100 samples from all three axes ($g_x, g_y, g_z$), computes their arithmetic mean, and determines the sensor’s static error profile (offset). These bias values are then subtracted from every raw measurement during runtime to normalize the sensor’s zero point. This ``bias compensation``minimizes drift when the sensor is stationary and enables angular velocity measurements with higher accuracy and reduced noise.

---

```c
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
```
- This function reads the raw ``accelerometer``and``gyroscope``data from the MPU6050 sensor via``I2C``and stores them in the corresponding variables. In the first part, the 16-bit raw data of the accelerometer’s X, Y, and Z axes are read from the MPU6050’s consecutive register addresses using the`HAL_I2C_Mem_Read`function, and the high and low bytes are combined and assigned to the`x_acc`,`y_acc`,and`z_acc`variables. In the second part, the same procedure is performed for the gyroscope, and the angular velocity data for the X, Y, and Z axes are written to the`x_gyr`,`y_gyr`, and`z_gyr`variables. This function only acquires raw sensor data; scaling, filtering, or angle computation is not performed at this stage.

---

The raw accelerometer and gyroscope data have been read from the MPU6050 sensor. The next objective is to use these raw measurements to obtain the ``Euler angles (Roll, Pitch, Yaw),``which describe the system’s orientation in space. These three angles represent the rotation of a rigid body about its three principal axes around its center of mass.

![Airplane_control_Roll_Pitch_Yaw (1)](https://github.com/user-attachments/assets/47011d33-7899-4b49-b2c7-b3fe929cafe9)

- $\phi$ ``(Phi) – Roll:`` Rotation about the longitudinal (X) axis.
- $\theta$ ``(Theta) – Pitch:`` Rotation about the lateral (Y) axis.
- $\psi$ ``(Psi) – Yaw:`` Rotation about the vertical (Z) axis.


```c
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
```
1.This function calculates the Roll and Pitch Euler angles using the raw accelerometer and gyroscope data read from the MPU6050, and produces the results of different methods in a comparative manner. In the first step, the accelerometer data are normalized according to the selected measurement range and expressed in units of `g`, while the gyroscope data are converted to `rad/s` after subtracting their offsets.

2.Next, the ``Roll (φ)``and``Pitch (θ)``angles are directly calculated from the `accelerometer` data based on the gravity vector.

``Roll Angle from the accelerometer`` 

<img width="504" height="177" alt="Pasted image 20251223232524" src="https://github.com/user-attachments/assets/9225465b-52ec-4a9a-868b-5f0441aade9c" />

$$
s^2=Acc_Z^2 + Acc_X^2
$$

$$
tan(\phi_{\text{Roll}})=\frac{Acc_Y}{s}
$$

$$
\phi_{\text{Roll}} = arctan(\frac{Acc_Y}{\sqrt{Acc_X^2+Acc_Z^2}})
$$

``Pitch Angle from the accelerometer``

<img width="518" height="172" alt="Pasted image 20251223232559" src="https://github.com/user-attachments/assets/fbfa4ddd-9cb7-4dbb-b9ad-a6aba3577553" />

$$
s^2=Acc_Z^2 + Acc_Y^2
$$

$$
tan(\theta_{\text{Pitch}})=\frac{-Acc_X}{s}
$$

$$
\theta_{\text{Pitch}} = arctan(\frac{-Acc_X}{\sqrt{Acc_Y^2+Acc_Z^2}})
$$

> [!warning]
> When the accelerometer is stationary, it measures only the ``gravitational acceleration (g)``, and the gravity vector always points toward the center of the Earth. On axes perpendicular to gravity (X and Y), the measured acceleration is close to zero, while on the axis parallel to gravity, the measured acceleration is approximately``1g``. Depending on the sensor’s orientation, this value is distributed among the corresponding axes. When the sensor is in motion, the measured acceleration also includes dynamic acceleration components in addition to gravity. Therefore, the magnitude of the acceleration vector is approximately ``1g`` in the stationary state, and deviates from this value during motion.
> 
> <img width="319" height="292" alt="Pasted image 20251207194725" src="https://github.com/user-attachments/assets/3bbd3f81-2868-42e8-b75f-512e417a945c" />

3.Gyroscope data produces raw (drift-prone) estimates of the ``Roll (φ)``and ``Pitch (θ)``angles by ``integrating``the angular rate equations over time.

$$
\begin{bmatrix}
\dot{\phi} \\
\dot{\theta} \\
\dot{\psi}
\end{bmatrix}
=
\begin{bmatrix}
1 & \sin\phi \tan\theta & \cos\phi \tan\theta \\
0 & \cos\phi & -\sin\phi \\
0 & \sin\phi / \cos\theta & \cos\phi / \cos\theta
\end{bmatrix}
\begin{bmatrix}
p \\
q \\
r
\end{bmatrix}
$$

**$\dot{\phi}$** → Angular rate of the Roll angle  
**$\dot{\theta}$** → Angular rate of the Pitch angle




