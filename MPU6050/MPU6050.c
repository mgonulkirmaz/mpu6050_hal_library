/*
 * MPU6050.c
 *
 *  Created on: Aug 16, 2021
 *      Author: mgk
 */

#include "MPU6050.h"
#include <math.h>

float accel_sens, gyro_sens;

uint8_t MPU6050_Init(I2C_HandleTypeDef *I2Cx, uint8_t fs_sel, uint8_t afs_sel) {
	uint8_t check;
	uint8_t data;

	HAL_I2C_Mem_Read(I2Cx, MPU6050_ADDR, WHO_AM_I, 1, &check, 1, 0xFFFF);

	if (check == 0x68) {

		data = DEVICE_RESET_0 | SLEEP_0 | CYCLE_0 | TEMP_DIS_0 | CLKSEL_1;
		HAL_I2C_Mem_Write(I2Cx, MPU6050_ADDR, PWR_MGMT_1, 1, &data, 1, 0xFFFF);

		data = EXT_SYNC_SET_0 | DLPF_CFG_7;
		HAL_I2C_Mem_Write(I2Cx, MPU6050_ADDR, CONFIG, 1, &data, 1, 0xFFFF);

		data = SMPLRT_DIV;
		HAL_I2C_Mem_Write(I2Cx, MPU6050_ADDR, SMPRT_DIV, 1, &data, 1, 0xFFFF);

		data = fs_sel;
		HAL_I2C_Mem_Write(I2Cx, MPU6050_ADDR, GYRO_CONFIG, 1, &data, 1, 0xFFFF);

		data = afs_sel;
		HAL_I2C_Mem_Write(I2Cx, MPU6050_ADDR, ACCEL_CONFIG, 1, &data, 1,
				0xFFFF);

		data = FIFO_EN_0 | I2C_MST_EN_0 | I2C_IF_DIS_0 | FIFO_RESET_0
				| I2C_MST_RESET_1 | SIG_COND_RESET_0;
		HAL_I2C_Mem_Write(I2Cx, MPU6050_ADDR, USER_CTRL, 1, &data, 1, 0xFFFF);

		data = INT_LEVEL_0 | INT_OPEN_0 | LATCH_INT_EN_0 | INT_RD_CLEAR_1
				| FSYNC_INT_LEVEL_0 | FSYNC_INT_EN_0 | I2C_BYPASS_EN_1;
		HAL_I2C_Mem_Write(I2Cx, MPU6050_ADDR, INT_PIN_CFG, 1, &data, 1, 0xFFFF);

		data = FIFO_OFLOW_EN_0 | I2C_MST_INT_EN_0 | DATA_RDY_EN_1;
		HAL_I2C_Mem_Write(I2Cx, MPU6050_ADDR, INT_ENABLE, 1, &data, 1, 0xFFFF);

		switch (fs_sel) {
		case FS_SEL_0:
			gyro_sens = 131.0;
			break;
		case FS_SEL_1:
			gyro_sens = 65.5;
			break;
		case FS_SEL_2:
			gyro_sens = 32.8;
			break;
		case FS_SEL_3:
			gyro_sens = 16.4;
			break;
		default:
			return 2;
		}

		switch (afs_sel) {
		case AFS_SEL_0:
			accel_sens = 16384.0;
			break;
		case AFS_SEL_1:
			accel_sens = 8192.0;
			break;
		case AFS_SEL_2:
			accel_sens = 4096.0;
			break;
		case AFS_SEL_3:
			accel_sens = 2048.0;
			break;
		default:
			return 2;
		}

		return 0;
	}

	return 1;
}
void MPU6050_Calibrate(I2C_HandleTypeDef *I2Cx, MPU6050_t *mpu) {
	float sum[6] = { 0, 0, 0, 0, 0, 0 };

	for (int i = 1; i < MPU6050_CALIBRATION_CNT; i++) {
		MPU6050_Read(I2Cx, mpu);

		if (i > MPU6050_CALIBRATION_CNT - (MPU6050_CALIBRATION_CNT - 100)) {
			sum[0] += (float) mpu->Accel_X_RAW / accel_sens;
			sum[1] += (float) mpu->Accel_Y_RAW / accel_sens;
			sum[2] += (float) mpu->Accel_Z_RAW / accel_sens;
			sum[3] += (float) mpu->Gyro_X_RAW / gyro_sens;
			sum[4] += (float) mpu->Gyro_Y_RAW / gyro_sens;
			sum[5] += (float) mpu->Gyro_Z_RAW / gyro_sens;
		}
		HAL_Delay(2);
	}

	mpu->Accel_X_Offset = sum[0] / (MPU6050_CALIBRATION_CNT - 100);
	mpu->Accel_Y_Offset = sum[1] / (MPU6050_CALIBRATION_CNT - 100);
	mpu->Accel_Z_Offset = sum[2] / (MPU6050_CALIBRATION_CNT - 100);
	mpu->Gyro_X_Offset = sum[3] / (MPU6050_CALIBRATION_CNT - 100);
	mpu->Gyro_Y_Offset = sum[4] / (MPU6050_CALIBRATION_CNT - 100);
	mpu->Gyro_Z_Offset = sum[5] / (MPU6050_CALIBRATION_CNT - 100);

}
void MPU6050_ReadAccel(I2C_HandleTypeDef *I2Cx, MPU6050_t *mpu) {
	uint8_t buf[6];

	HAL_I2C_Mem_Read(I2Cx, MPU6050_ADDR, ACCEL_XOUT_H, 1, buf, 6, 0xFFFF);

	mpu->Accel_X_RAW = (int16_t) (buf[0] << 8 | buf[1]);
	mpu->Accel_Y_RAW = (int16_t) (buf[2] << 8 | buf[3]);
	mpu->Accel_Z_RAW = (int16_t) (buf[4] << 8 | buf[5]);

}
void MPU6050_ReadGyro(I2C_HandleTypeDef *I2Cx, MPU6050_t *mpu) {
	uint8_t buf[6];

	HAL_I2C_Mem_Read(I2Cx, MPU6050_ADDR, GYRO_XOUT_H, 1, buf, 6, 0xFFFF);

	mpu->Gyro_X_RAW = (int16_t) (buf[0] << 8 | buf[1]);
	mpu->Gyro_Y_RAW = (int16_t) (buf[2] << 8 | buf[3]);
	mpu->Gyro_Z_RAW = (int16_t) (buf[4] << 8 | buf[5]);
}
void MPU6050_ReadTemperature(I2C_HandleTypeDef *I2Cx, MPU6050_t *mpu) {
	uint8_t buf[2];

	HAL_I2C_Mem_Read(I2Cx, MPU6050_ADDR, TEMP_OUT_H, 1, buf, 2, 0xFFFF);

	mpu->Temperature_RAW = (int16_t) (buf[0] << 8 | buf[1]);
}
void MPU6050_Read(I2C_HandleTypeDef *I2Cx, MPU6050_t *mpu) {
	uint8_t buf[14];

	HAL_I2C_Mem_Read(I2Cx, MPU6050_ADDR, ACCEL_XOUT_H, 1, buf, 14, 0xFFFF);

	mpu->Accel_X_RAW = (int16_t) (buf[0] << 8 | buf[1]);
	mpu->Accel_Y_RAW = (int16_t) (buf[2] << 8 | buf[3]);
	mpu->Accel_Z_RAW = (int16_t) (buf[4] << 8 | buf[5]);
	mpu->Temperature_RAW = (int16_t) (buf[6] << 8 | buf[7]);
	mpu->Gyro_X_RAW = (int16_t) (buf[8] << 8 | buf[9]);
	mpu->Gyro_Y_RAW = (int16_t) (buf[10] << 8 | buf[11]);
	mpu->Gyro_Z_RAW = (int16_t) (buf[12] << 8 | buf[13]);

}
void MPU6050_ConvertRawValues(MPU6050_t *mpu) {

	mpu->Accel_X = ((float) mpu->Accel_X_RAW - mpu->Accel_X_Offset)
			/ accel_sens;
	mpu->Accel_Y = (float) ((float) mpu->Accel_Y_RAW - mpu->Accel_Y_Offset)
			/ accel_sens;
	mpu->Accel_Z = ((float) mpu->Accel_Z_RAW - mpu->Accel_Z_Offset)
			/ accel_sens;
	mpu->Temperature = ((float) (mpu->Temperature_RAW) / 340.0) + 36.53;
	mpu->Gyro_X = ((float) mpu->Gyro_X_RAW - mpu->Gyro_X_Offset) / gyro_sens;
	mpu->Gyro_Y = ((float) mpu->Gyro_Y_RAW - mpu->Gyro_Y_Offset) / gyro_sens;
	mpu->Gyro_Z = ((float) mpu->Gyro_Z_RAW - mpu->Gyro_Z_Offset) / gyro_sens;

}
void MPU6050_ComplementaryFilter(MPU6050_t *mpu, uint32_t delta_t_us) {
	float acc_vector = sqrt(
			mpu->Accel_X * mpu->Accel_X + mpu->Accel_Y * mpu->Accel_Y
					+ mpu->Accel_Z * mpu->Accel_Z);

	mpu->PitchAcc = asin(mpu->Accel_Y / acc_vector) * RAD_TO_DEG - (-0.75);
	mpu->RollAcc = asin(mpu->Accel_X / acc_vector) * -RAD_TO_DEG - 2.25;

	mpu->Pitch = (mpu->Pitch + mpu->Gyro_Y * (delta_t_us / 1000000))
			* mpu->P_Comp_Coeff + mpu->PitchAcc * (1.00 - mpu->P_Comp_Coeff);
	mpu->Roll = (mpu->Roll + mpu->Gyro_X * (delta_t_us / 1000000))
			* mpu->R_Comp_Coeff + mpu->RollAcc * (1.00 - mpu->R_Comp_Coeff);

}
void MPU6050_ZeroAll(MPU6050_t *mpu) {
	mpu->Accel_X = 0;
	mpu->Accel_X_Offset = 0;
	mpu->Accel_X_RAW = 0;
	mpu->Accel_Y = 0;
	mpu->Accel_Y_Offset = 0;
	mpu->Accel_Y_RAW = 0;
	mpu->Accel_Z = 0;
	mpu->Accel_Z_Offset = 0;
	mpu->Accel_Z_RAW = 0;
	mpu->Gyro_X = 0;
	mpu->Gyro_X_Offset = 0;
	mpu->Gyro_X_RAW = 0;
	mpu->Gyro_Y = 0;
	mpu->Gyro_Y_Offset = 0;
	mpu->Gyro_Y_RAW = 0;
	mpu->Gyro_Z = 0;
	mpu->Gyro_Z_Offset = 0;
	mpu->Gyro_Z_RAW = 0;
	mpu->Temperature = 0;
	mpu->Temperature_RAW = 0;
	mpu->Pitch = 0;
	mpu->PitchAcc = 0;
	mpu->Roll = 0;
	mpu->RollAcc = 0;
	mpu->Yaw = 0;
	mpu->YawAcc = 0;
	mpu->R_Comp_Coeff = 0.9625;
	mpu->P_Comp_Coeff = 0.9625;
	mpu->Y_Comp_Coeff = 0.9625;
}
