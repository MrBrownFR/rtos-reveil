#ifndef _SGP40_H_
#define _SGP40_H_


#include "sensirion_arch_config.h"
#include "sensirion_voc_algorithm.h"



#define SGP40_ADDR       (0x59)

/***********************************************************************************/
uint8_t SGP40_Init(void);
uint16_t SGP40_MeasureRaw(float temp, float humi);
uint16_t SGP40_MeasureVOC(float temp, float humi);

#endif
