/*
 * mm_gyro.c
 *
 *  Created on: 12.01.2017
 *      Author: marc
 */

#include "lre_gyro.h"

void lre_gyro_init(void) {
	// initiate L3GD20 by educated guessing and/or the datasheet (or copy paste...)
	L3GD20_InitTypeDef gyroInitstruct;
	gyroInitstruct.Axes_Enable = L3GD20_AXES_ENABLE;
	gyroInitstruct.Band_Width = L3GD20_BANDWIDTH_1;
	gyroInitstruct.BlockData_Update = L3GD20_BlockDataUpdate_Continous;
	gyroInitstruct.Endianness = L3GD20_BLE_LSB;
	gyroInitstruct.Full_Scale = L3GD20_FULLSCALE_500;
	gyroInitstruct.Output_DataRate = L3GD20_OUTPUT_DATARATE_1;
	gyroInitstruct.Power_Mode = L3GD20_MODE_ACTIVE;

	L3GD20_Init(&gyroInitstruct);

	NVIC_EnableIRQ(SPI2_IRQn);
	lre_gyro_enableInt2();
}

void lre_gyro_enableInt2(void) {
	// config the pin connected to int2 output (DataReady) of the gyro as input
	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOC, ENABLE);

	GPIO_InitTypeDef gpioInitStruct;

	gpioInitStruct.GPIO_Mode = GPIO_Mode_IN;
	gpioInitStruct.GPIO_OType = GPIO_OType_PP;
	gpioInitStruct.GPIO_Pin = GPIO_Pin_2;
	gpioInitStruct.GPIO_PuPd = GPIO_PuPd_NOPULL;
	gpioInitStruct.GPIO_Speed = GPIO_Speed_Level_1;

	GPIO_Init(GPIOC, &gpioInitStruct);

	L3GD20_INT2InterruptCmd(L3GD20_INT2INTERRUPT_ENABLE);
}

/**
 * @brief  read z rate
 * @retval z rate in degree per second
 */
float lre_gyro_getZRate(void) {
	// prepare memory to store the two bytes
	uint8_t rateBuffer[2];
	// read the two bytes Z_low and Z_high from the sensor
	L3GD20_Read(rateBuffer, L3GD20_OUT_Z_L_ADDR, 2);

	// shift high byte 8 times left and add low byte to get 16-bit turn rate
	return (float) (((int16_t) (rateBuffer[1] << 8) + rateBuffer[0])
			- lre_gyro.offset.z) / 1000.0 * 17.5;
}

/**
 * @brief  requests rates from the gyro. Communication is done with SPI Interrupts
 * @retval none
 */
void lre_gyro_requestRate(void) {
	lre_gyro_resetStatus();
	L3GD20_CS_LOW();
	// clear receiver not empty flag (just in case)
	SPI_I2S_ClearFlag(SPI2, SPI_I2S_FLAG_RXNE);
	// enable the TXE interrupt
	SPI_I2S_ITConfig(SPI2, SPI_I2S_IT_TXE, ENABLE);
	// rest will be done in the interrupt handler
}

/**
 * @brief  resets the status variable
 * @retval none
 */
void lre_gyro_resetStatus(void) {
	lre_gyro.status = lre_gyro_status_standby;
}

void lre_gyro_getTemperature(void) {
	L3GD20_Read(&lre_gyro.temperature, L3GD20_OUT_TEMP_ADDR, 1);
	// cast variable to avoid warning. l3gd20_read function only supports unsigned variables..
	// mm_gyro.temperature = (int8_t)mm_gyro.temperature;
}

void lre_gyro_calibrate(void) {
	uint16_t counter = 0;
	uint8_t rateBuffer[6];
	int32_t bufferX = 0;
	int32_t bufferY = 0;
	int32_t bufferZ = 0;

	// read old sensor data and ignore them
	while (!GPIO_ReadInputDataBit(GPIOC, GPIO_Pin_2)) {
	}
	L3GD20_Read(rateBuffer, L3GD20_OUT_X_L_ADDR, 6);

	for (counter = 0; counter <= _lre_gyro_calibration_samples; counter++) {
		while (!GPIO_ReadInputDataBit(GPIOC, GPIO_Pin_2)) {
		}
		// prepare memory to store the two bytes
		// read the two bytes Z_low and Z_high from the sensor
		L3GD20_Read(rateBuffer, L3GD20_OUT_X_L_ADDR, 6);

		// shift high byte 8 times left and add low byte to get 16-bit turn rate
		bufferX += ((int16_t) (rateBuffer[1] << 8) + rateBuffer[0]);
		bufferY += ((int16_t) (rateBuffer[3] << 8) + rateBuffer[2]);
		bufferZ += ((int16_t) (rateBuffer[5] << 8) + rateBuffer[4]);
	}
	lre_gyro.offset.x = (int16_t) (bufferX / _lre_gyro_calibration_samples);
	lre_gyro.offset.y = (int16_t) (bufferY / _lre_gyro_calibration_samples);
	lre_gyro.offset.z = (int16_t) (bufferZ / _lre_gyro_calibration_samples);
}


// the following function is not used in this program
// Because STMicro has declared it in its driver library it has to be defined thought
uint32_t L3GD20_TIMEOUT_UserCallback(void) {
// do nothing
	return 0;
}

void SPI2_IRQHandler(void) {
	switch (lre_gyro.status) {
	// start of the SPI communication
	// send the register address of the first data register
	case 0:
		SPI_I2S_ClearFlag(SPI2, SPI_I2S_FLAG_TXE);
		SPI_I2S_ITConfig(SPI2, SPI_I2S_IT_TXE, DISABLE);
		// or the register address with 0xC0 to singal a multiple byte reading operation
		SPI_SendData8(SPI2, L3GD20_OUT_X_L_ADDR | 0b11000000);
		lre_gyro.status++;
		SPI_I2S_ITConfig(SPI2, SPI_I2S_IT_RXNE, ENABLE);
		break;
		// register address is completely sent. Start sending dummy bytes (0x00) to read the data from the data registers
	case 1:
		SPI_I2S_ClearFlag(SPI2, SPI_I2S_FLAG_RXNE);
		// read buffer. Ignore the data though
		SPI_ReceiveData8(SPI2);
		// send 8 dummy bits to receive the first rate byte
		SPI_SendData8(SPI2, 0x00);
		lre_gyro.status++;
		break;
		// receive the data and send next dummy byte. Do this 5 times to receive the other 5 data bytes
	case 2:
		SPI_I2S_ClearFlag(SPI2, SPI_I2S_FLAG_RXNE);
		// save received data into the buffer
		lre_gyro.buffer[lre_gyro.bCounter] = SPI_ReceiveData8(SPI2);
		// send 8 dummy bits to receive the next rate byte
		SPI_SendData8(SPI2, 0x00);
		lre_gyro.bCounter++;
		// do this 5 times. 6th byte will be received in the next case
		if (lre_gyro.bCounter >= 5) {
			lre_gyro.status++;
		}
		break;
		// receive the last byte, sort the 6 bytes to get the 3 signed, 16-bit rate values
	case 3:
		SPI_I2S_ClearFlag(SPI2, SPI_I2S_FLAG_RXNE);
		SPI_I2S_ITConfig(SPI2, SPI_I2S_IT_RXNE, DISABLE);
		// read last received byte
		lre_gyro.buffer[lre_gyro.bCounter] = SPI_ReceiveData8(SPI2);
		L3GD20_CS_HIGH();

		// sort bytes
		lre_gyro.rateI16.x = ((int16_t) (lre_gyro.buffer[1] << 8)
				+ lre_gyro.buffer[0]);
		lre_gyro.rateI16.y = ((int16_t) (lre_gyro.buffer[3] << 8)
				+ lre_gyro.buffer[2]);
		lre_gyro.rateI16.z = ((int16_t) (lre_gyro.buffer[5] << 8)
				+ lre_gyro.buffer[4]);

		// use calibration values
		lre_gyro.rateI16.x = (lre_gyro.rateI16.x - lre_gyro.offset.x)
				* _lre_gyro_scale_error_x;
		lre_gyro.rateI16.y = (lre_gyro.rateI16.y - lre_gyro.offset.y)
				* _lre_gyro_scale_error_y;
		lre_gyro.rateI16.z = (lre_gyro.rateI16.z - lre_gyro.offset.z)
				* _lre_gyro_scale_error_z;

		// multiply with 17.5 to get rates in mdeg/s
		lre_gyro.rateI16.x = lre_gyro.rateI16.x * 17.5;
		lre_gyro.rateI16.y = lre_gyro.rateI16.y * 17.5;
		lre_gyro.rateI16.z = lre_gyro.rateI16.z * 17.5;

		// calculate the rates in deg/s
		lre_gyro.rate.x = lre_gyro.rateI16.x / 1000.0;
		lre_gyro.rate.y = lre_gyro.rateI16.y / 1000.0;
		lre_gyro.rate.z = lre_gyro.rateI16.z / 1000.0;

		lre_gyro.status++;
		lre_gyro.bCounter = 0;
		break;
	default:
		break;
	}
}

