#include "SGP40.h"
#include "stm32l4xx_hal.h"
#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"

extern I2C_HandleTypeDef hi2c1;
extern SemaphoreHandle_t xI2CMutex;      // créé dans main avant le scheduler

#define SGP40_I2C_TIMEOUT   20           // ms
#define SGP40_MUTEX_WAIT    pdMS_TO_TICKS(100)
#define SGP40_DELAY_MS(ms)  vTaskDelay(pdMS_TO_TICKS(ms))

/* Trame de mesure : cmd (2) + humidité (2) + CRC + température (2) + CRC */
static uint8_t MEAS_CMD[8] = {0x26, 0x0F, 0x80, 0x00, 0xA2, 0x66, 0x66, 0x93};

VocAlgorithmParams voc_algorithm_params;

/* CRC8 Sensirion : polynôme 0x31, valeur initiale 0xFF (identique à l'ancienne table) */
static uint8_t crc8_2bytes(uint8_t msb, uint8_t lsb)
{
    uint8_t d[2] = { msb, lsb };
    uint8_t crc = 0xFF;
    for (int i = 0; i < 2; i++) {
        crc ^= d[i];
        for (int b = 0; b < 8; b++)
            crc = (crc & 0x80) ? (uint8_t)((crc << 1) ^ 0x31) : (uint8_t)(crc << 1);
    }
    return crc;
}

/* Transactions I2C protégées par le mutex partagé avec l'OLED */
static HAL_StatusTypeDef sgp40_tx(uint8_t *buf, uint16_t n)
{
    if (xSemaphoreTake(xI2CMutex, SGP40_MUTEX_WAIT) != pdTRUE) return HAL_BUSY;
    HAL_StatusTypeDef st = HAL_I2C_Master_Transmit(&hi2c1, SGP40_ADDR << 1, buf, n, SGP40_I2C_TIMEOUT);
    xSemaphoreGive(xI2CMutex);
    return st;
}

static HAL_StatusTypeDef sgp40_rx(uint8_t *buf, uint16_t n)
{
    if (xSemaphoreTake(xI2CMutex, SGP40_MUTEX_WAIT) != pdTRUE) return HAL_BUSY;
    HAL_StatusTypeDef st = HAL_I2C_Master_Receive(&hi2c1, SGP40_ADDR << 1, buf, n, SGP40_I2C_TIMEOUT);
    xSemaphoreGive(xI2CMutex);
    return st;
}

/* À appeler depuis une tâche (utilise vTaskDelay). Renvoie 0 si OK, 1 sinon. */
uint8_t SGP40_Init(void)
{
    uint8_t feat[2] = {0x20, 0x2F};      // get feature set
    uint8_t test[2] = {0x28, 0x0E};      // self test
    uint8_t rst     = 0x06;              // soft reset (general call)
    uint8_t rx[3];

    /* 1. Feature set : on vérifie seulement que le capteur répond avec un CRC valide */
    if (sgp40_tx(feat, 2) != HAL_OK) return 1;
    SGP40_DELAY_MS(30);
    if (sgp40_rx(rx, 3) != HAL_OK) return 1;
    if (crc8_2bytes(rx[0], rx[1]) != rx[2]) return 1;

    /* 2. Self-test : 0xD4 = OK, 0x4B = NOK (durée 320 ms) */
    if (sgp40_tx(test, 2) != HAL_OK) return 1;
    SGP40_DELAY_MS(400);
    if (sgp40_rx(rx, 3) != HAL_OK) return 1;
    if (crc8_2bytes(rx[0], rx[1]) != rx[2]) return 1;
    if (rx[0] != 0xD4) return 1;

    /* 3. Soft reset : general call, adresse 0x00 */
    if (xSemaphoreTake(xI2CMutex, SGP40_MUTEX_WAIT) == pdTRUE) {
        HAL_I2C_Master_Transmit(&hi2c1, 0x00, &rst, 1, SGP40_I2C_TIMEOUT);
        xSemaphoreGive(xI2CMutex);
    }
    SGP40_DELAY_MS(30);

    /* 4. Initialisation de l'algorithme VOC */
    VocAlgorithm_init(&voc_algorithm_params);
    return 0;
}

/* Renvoie le SRAW_VOC (ticks), ou 0 en cas d'erreur */
uint16_t SGP40_MeasureRaw(float temp, float humi)
{
    uint8_t rx[3];

    /* Bornes admises par le capteur */
    if (humi < 0.0f)    humi = 0.0f;
    if (humi > 100.0f)  humi = 100.0f;
    if (temp < -45.0f)  temp = -45.0f;
    if (temp > 130.0f)  temp = 130.0f;

    /* Humidité : ticks = RH * 65535 / 100, puis CRC */
    uint16_t paramh = (uint16_t)((humi * 65535.0f) / 100.0f + 0.5f);
    MEAS_CMD[2] = paramh >> 8;
    MEAS_CMD[3] = paramh & 0xFF;
    MEAS_CMD[4] = crc8_2bytes(MEAS_CMD[2], MEAS_CMD[3]);

    /* Température : ticks = (T + 45) * 65535 / 175, puis CRC */
    uint16_t paramt = (uint16_t)(((temp + 45.0f) * 65535.0f) / 175.0f + 0.5f);
    MEAS_CMD[5] = paramt >> 8;
    MEAS_CMD[6] = paramt & 0xFF;
    MEAS_CMD[7] = crc8_2bytes(MEAS_CMD[5], MEAS_CMD[6]);

    if (sgp40_tx(MEAS_CMD, 8) != HAL_OK) return 0;
    SGP40_DELAY_MS(30);                              // mutex relâché pendant l'attente
    if (sgp40_rx(rx, 3) != HAL_OK) return 0;
    if (crc8_2bytes(rx[0], rx[1]) != rx[2]) return 0;

    return ((uint16_t)rx[0] << 8) | rx[1];           // MSB en premier
}

/* Renvoie l'indice VOC 0..500 (0 pendant les 45 premières secondes).
   À appeler exactement une fois par seconde. */
uint16_t SGP40_MeasureVOC(float temp, float humi)
{
    int32_t voc_index = 0;
    uint16_t sraw = SGP40_MeasureRaw(temp, humi);    // 0 si erreur de lecture
    VocAlgorithm_process(&voc_algorithm_params, sraw, &voc_index);
    return (uint16_t)voc_index;
}
