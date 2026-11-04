#include "SGP40.h"
#include "stm32l4xx_hal.h"
#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"
#include <stdio.h>

extern I2C_HandleTypeDef hi2c1;
extern SemaphoreHandle_t xI2CMutex;        // créé dans main, partagé avec l'OLED

#define SGP40_DELAY_MS(ms)   vTaskDelay(pdMS_TO_TICKS(ms))
#define SGP40_I2C_TIMEOUT    20            // ms

uint8_t  CRC_TABLE[] = {
	0, 49, 98, 83, 196, 245, 166, 151, 185, 136, 219, 234, 125, 76, 31, 46,
	67, 114, 33, 16, 135, 182, 229, 212, 250, 203, 152, 169, 62, 15, 92, 109,
	134, 183, 228, 213, 66, 115, 32, 17, 63, 14, 93, 108, 251, 202, 153, 168,
	197, 244, 167, 150, 1, 48, 99, 82, 124, 77, 30, 47, 184, 137, 218, 235,
	61, 12, 95, 110, 249, 200, 155, 170, 132, 181, 230, 215, 64, 113, 34, 19,
	126, 79, 28, 45, 186, 139, 216, 233, 199, 246, 165, 148, 3, 50, 97, 80,
	187, 138, 217, 232, 127, 78, 29, 44, 2, 51, 96, 81, 198, 247, 164, 149,
	248, 201, 154, 171, 60, 13, 94, 111, 65, 112, 35, 18, 133, 180, 231, 214,
	122, 75, 24, 41, 190, 143, 220, 237, 195, 242, 161, 144, 7, 54, 101, 84,
	57, 8, 91, 106, 253, 204, 159, 174, 128, 177, 226, 211, 68, 117, 38, 23,
	252, 205, 158, 175, 56, 9, 90, 107, 69, 116, 39, 22, 129, 176, 227, 210,
	191, 142, 221, 236, 123, 74, 25, 40, 6, 55, 100, 85, 194, 243, 160, 145,
	71, 118, 37, 20, 131, 178, 225, 208, 254, 207, 156, 173, 58, 11, 88, 105,
	4, 53, 102, 87, 192, 241, 162, 147, 189, 140, 223, 238, 121, 72, 27, 42,
	193, 240, 163, 146, 5, 52, 103, 86, 120, 73, 26, 43, 188, 141, 222, 239,
	130, 179, 224, 209, 70, 119, 36, 21, 59, 10, 89, 104, 255, 206, 157, 172
};

//Without_humidity_compensation
//sgp40_measure_raw + 2*humi + CRC + 2*temp + CRC
uint8_t  WITHOUT_HUM_COMP[8] = {0X26, 0X0F, 0X80, 0X00, 0XA2, 0X66, 0X66, 0X93}; // default Temperature=25 Humidity=50
uint8_t WITH_HUM_COMP[8] = {0x26, 0x0f, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00}; //Manual input
VocAlgorithmParams voc_algorithm_params;

/******************************************************************************
  function:	Send one byte of data to  I2C dev
  parameter:
            Addr: Register address
            Value: Write to the value of the register
  Info:
******************************************************************************/
//static void SGP40_Write(uint8_t *Value, uint8_t num)
//{
//	I2C_Write_Nbyte(Value, num);
//}
//
///******************************************************************************
//  function:	 read one byte of data to  I2C dev
//  parameter:
//            Addr: Register address
//  Info:
//******************************************************************************/
//static uint16_t SGP40_Read()
//{
//	//(address, 0, 3)#last byte is CRC8
//	uint16_t rdata;
//	//rdata = I2C_Read_Nbyte(0, 3);
//
//	return rdata;
//}

static uint8_t CRC8_MAXIM(uint8_t msb,uint8_t lsb)
{
	uint8_t crc = 0xff;
	crc ^= msb;
	crc = CRC_TABLE[crc];
	crc ^= lsb;
	crc = CRC_TABLE[crc];
	return crc;
}

/******************************************************************************
  function:	TSL2591 Initialization
  parameter:
  Info:
******************************************************************************/
//uint8_t SGP40_Init(void)
//{
//	printf("SGP40 VOC Sensor Init\r\n");
//
//	//feature set 0x3220
//	uint8_t wbuf[2];
//	uint16_t Rdata;
//	wbuf[0] = 0x20;
//	wbuf[1] = 0x2f;
//	//SGP40_Write(wbuf, 2);
//	HAL_I2C_Master_Transmit(&hi2c1, (0x59<<1),(uint8_t *)wbuf, sizeof(wbuf),HAL_MAX_DELAY);
//	HAL_Delay(30);
//	//uint16_t Rdata = SGP40_Read();
//	HAL_I2C_Master_Receive(&hi2c1,(0x59<<1), (uint8_t *)&Rdata, sizeof(Rdata),HAL_MAX_DELAY);
//	if(Rdata != 0x4032) { // 0x3220
//		printf("1.feature set should = 0x3220, but val = 0X%d\r\n", Rdata);
//		return 1;
//	}
//
//	// Self Test 0x4B00 is failed, 0xD400 pass
//	wbuf[0] = 0X28;
//	wbuf[1] = 0X0E;
//	//SGP40_Write(wbuf, 2); //feature set 0xD400
//	HAL_I2C_Master_Transmit(&hi2c1, (0x59<<1),(uint8_t *)wbuf, sizeof(wbuf),HAL_MAX_DELAY);
//	HAL_Delay(400); // must
//	//uint16_t Rdata = SGP40_Read();
//	HAL_I2C_Master_Receive(&hi2c1,(0x59<<1), (uint8_t *)&Rdata, sizeof(Rdata),HAL_MAX_DELAY);
//	if(Rdata != 0xD4) {
//		printf("2.Self Test should = 0xD400, but val = 0X%d\r\n", Rdata);
//		return 1;
//	}
//
//	// reset
//	HAL_Delay(30);
//	wbuf[0] = 0X00;
//	wbuf[1] = 0X06;
//	//SGP40_Write(wbuf, 2); //feature set 0xD400
//	HAL_I2C_Master_Transmit(&hi2c1, (0x59<<1),(uint8_t *)wbuf, sizeof(wbuf),HAL_MAX_DELAY);
//
//	VocAlgorithm_init(&voc_algorithm_params);
//
//	return 0;
//}

static HAL_StatusTypeDef sgp40_tx(uint8_t *buf, uint16_t n)
{
    if (xSemaphoreTake(xI2CMutex, pdMS_TO_TICKS(100)) != pdTRUE) return HAL_BUSY;
    HAL_StatusTypeDef st = HAL_I2C_Master_Transmit(&hi2c1, SGP40_ADDR << 1, buf, n, SGP40_I2C_TIMEOUT);
    xSemaphoreGive(xI2CMutex);
    return st;
}

static HAL_StatusTypeDef sgp40_rx(uint8_t *buf, uint16_t n)
{
    if (xSemaphoreTake(xI2CMutex, pdMS_TO_TICKS(100)) != pdTRUE) return HAL_BUSY;
    HAL_StatusTypeDef st = HAL_I2C_Master_Receive(&hi2c1, SGP40_ADDR << 1, buf, n, SGP40_I2C_TIMEOUT);
    xSemaphoreGive(xI2CMutex);
    return st;
}

uint8_t SGP40_Init(void)       // à appeler depuis une tâche (utilise vTaskDelay)
{
    uint8_t feat[2] = {0x20, 0x2F};
    uint8_t test[2] = {0x28, 0x0E};
    uint8_t rst     = 0x06;
    uint8_t rx[3];

    // Feature set : octets 0x32 0x40 (équivalent du test 0x4032 d'origine)
    if (sgp40_tx(feat, 2) != HAL_OK) return 1;
    SGP40_DELAY_MS(30);
    if (sgp40_rx(rx, 3) != HAL_OK || rx[0] != 0x32 || rx[1] != 0x40) return 1;

    // Self-test : 0xD4 = OK, 0x4B = NOK (320 ms)
    if (sgp40_tx(test, 2) != HAL_OK) return 1;
    SGP40_DELAY_MS(400);
    if (sgp40_rx(rx, 3) != HAL_OK || rx[0] != 0xD4) return 1;

    // Soft reset : general call, adresse 0x00
    xSemaphoreTake(xI2CMutex, pdMS_TO_TICKS(100));
    HAL_I2C_Master_Transmit(&hi2c1, 0x00, &rst, 1, SGP40_I2C_TIMEOUT);
    xSemaphoreGive(xI2CMutex);
    SGP40_DELAY_MS(30);

    VocAlgorithm_init(&voc_algorithm_params);
    return 0;
}

uint16_t SGP40_MeasureRaw(float temp, float humi)
{
	//printf("measureRaw\r\n");

	// 2*humi + CRC
	uint16_t paramh;
	paramh = (uint16_t)((humi * 65535) / 100 + 0.5); // for DATASHEET page13
	WITH_HUM_COMP[2] = paramh >> 8;
	WITH_HUM_COMP[3] = paramh & 0xFF;
	WITH_HUM_COMP[4] = CRC8_MAXIM(WITH_HUM_COMP[2], WITH_HUM_COMP[3]);
	
	// 2*temp + CRC
	uint16_t paramt;
	paramt = (uint16_t)(((temp + 45) * 65535) / 175);
	WITH_HUM_COMP[5] = paramt >> 8;
	WITH_HUM_COMP[6] = paramt & 0xFF;
	WITH_HUM_COMP[7] = CRC8_MAXIM(WITH_HUM_COMP[5], WITH_HUM_COMP[6]);

	//SGP40_Write(WITH_HUM_COMP, 8);
	HAL_I2C_Master_Transmit(&hi2c1, (0x59<<1),(uint8_t *)WITH_HUM_COMP, sizeof(WITH_HUM_COMP),HAL_MAX_DELAY);

	HAL_Delay(30);
	uint16_t Rdata;
	HAL_I2C_Master_Receive(&hi2c1,(0x59<<1), (uint8_t *)&Rdata, sizeof(Rdata),HAL_MAX_DELAY);
	return Rdata;
}

uint16_t SGP40_MeasureVOC(float temp, float humi)
{	
	int32_t voc_index;
	uint16_t sraw = SGP40_MeasureRaw(temp, humi);

	VocAlgorithm_process(&voc_algorithm_params, sraw, &voc_index);
	//printfln(voc_index);
	return voc_index;
}


