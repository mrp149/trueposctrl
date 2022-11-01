/*
 * truepos.h
 *
 *  Created on: Sep 1, 2017
 *      Author: Nathan
 *
 *  Updated on: Oct 31, 2022
 *   for linux: mrp
 */
#ifndef TRUEPOS_H_
#define TRUEPOS_H_
enum {
	SF_GPSDO_CONNECTED = 1,
	SF_BAD_ANTENNA = 2,
	SF_STARTUP = 4,
	SF_SURVEY = 8,
	SF_BAD_10M = 16,
	SF_BAD_PPS = 32
};

#define LastMsg_LEN (60)


#define __IO

typedef struct {
	uint8_t statusFlags;
	uint8_t status;
	uint8_t NumSats;
	float Temp;
	uint32_t Clock;
	uint8_t UTCOffset;
	float Vset_uV;
	float DOP;
	int32_t PPSPhase;
	int32_t PPSOffset;
	int32_t PPSStatus;
	int32_t TRAIM;
	uint32_t SurveyEndClock;
	uint32_t LockStartClock;
	char LastMsg[LastMsg_LEN];
} dispState_struct;


typedef struct
{
  uint32_t                 ErrorCode;        /*!< UART Error code                    */
}UART_HandleTypeDef;


extern dispState_struct dispState;
// extern TaskHandle_t  displayTaskHandle;

void StartDisplayTask(void const * argument);
void displayRequestRefresh();

void TruePosReadBuffer();
void TruePosInit(UART_HandleTypeDef *uartPtr, uint16_t id);
#endif
