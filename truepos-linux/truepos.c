/*
 * truepos.c
 *
 *  Created on: Sep 1, 2017
 *      Author: Nathan
 *
 *  Updated on: Oct 31, 2022
 *   for linux: mrp
 */

#include <string.h>
#include <stdint.h>
#include <stdlib.h>
#include <stdio.h>
#include <time.h>

#include "uart.h"
#include "truepos.h"

#define RSP_GETPOS "$GETPOS"
#define RSP_GETVER "$GETVER"
#define RSP_CLOCK "$CLOCK"
#define RSP_PPSDBG "$PPSDBG"
#define RSP_STATUS "$STATUS"
#define RSP_EXTSTATUS "$EXTSTATUS"
#define RSP_SURVEY "$SURVEY"
#define RSP_SETPOS "$SETPOS"
#define RSP_KALDBG "$KALDBG"
#define RSP_SAT "$SAT"
#define RSP_WSAT "$WSAT"

#define INFO_PROCEED "$TX PROCEED\r\n"
#define INFO_PPSDBG1 "$TX PPSDBG 1\r\n"


#ifdef UART
#define UART_Init()   uart_init()
#define UART_Tx(x)    printf("\nWRITE: %s", x);uart_tx(x, strlen(x))
#define UART_Rx()     uart_rx()
#else
#define UART_Init()
#define UART_Tx(x)    printf("\nWRITE: %s", x)
#define UART_Rx()     getchar()
#endif 

#define CMDBUF_LEN 128
char cmdBuf[CMDBUF_LEN];
static uint8_t ppsdbgReceived = 0;
static int cmdBufLen = 0;
static UART_HandleTypeDef *uart;
static uint16_t uart_id;
static uint16_t clockNoPPSDBG = 0;
static int LOCSent = 0;
static void HandleCommand();
static void HandlePPSDbgMsg();
static void HandleStatusMsg();
static void HandleStatusCode(uint8_t code);
static void HandleExtStatusMsg();
static void HandleClockMsg();
static void HandleSurveyMsg();
static void noteTx(char *msg);

void TruePosInit(UART_HandleTypeDef *uartPtr, uint16_t id) {
	uart = uartPtr;
	uart_id = id;
	UART_Init();
}



static int lastStatusFlags = 0;
static int lastStatus = 0;
static float lastvoltage = 0.0f;
static int  lastpps = 0;

static char * LockOn = " CLK:";

static int statusChanged() {

	// Criteria to move to the next line:
	// Any of statusFlags or status or PPSstatus has been changed
	// We do not track voltage or phase changes in assumption that this is up to the GPSDO
	if (lastStatusFlags ^ dispState.statusFlags || lastStatus ^ dispState.status || lastpps != dispState.PPSStatus) {
		lastStatusFlags = dispState.statusFlags;
		lastStatus = dispState.status;
		lastpps = dispState.PPSStatus;
	    return 1;
	}
	return 0;
}

static char * convert_gps_time() {

    time_t gpstime = dispState.Clock;
    struct tm  ts;
    static char       buf[10];

    // Format time, "hh:mm:ss" FURUNO does not back up the date after Sept. 2022
    ts = *gmtime(&gpstime);
    strftime(buf, sizeof(buf), "%H:%M:%S", &ts);

    return buf;
}

void displayRequestRefresh() {
    dispState_struct *d;
    const char * str;    // Status 1
    const char * str2;   // Status 2
    char  nl;            // Line feed
    char voltage[10];    // The voltage on the oscillator
    
    const char* const statusLabels[] = {
		"Locked",NULL,          // 0
		"Recovery",NULL,        // 1
		"Startup","5/5",        // 2 (Initialization?)
		"Holdover","(SET1PPS)", // 3 (From $SET1PPS?)
		"Forced","Holdover",    // 4 (Forced Holdover?)
		"Soft","Holdover",      // 5 (Soft Holdover?)
		"Unknown","Location",   // 6
		"OCXO","Training",      // 7 (OCXO Training?)
		"Holdover","Recovery",  // 8
		"Startup","0/5",        // 9
		"Startup","1/5",        // 10
		"Startup","2/5",        // 11
		"Startup","3/5",        // 12
		"Startup","4/5",        // 13
		"Wait A","0/4",         // 14
		"Wait A","1/4",         // 15
		"Wait A","2/4",         // 16
		"Wait A","3/4",         // 17
		"Wait A","4/4",         // 18
		"Wait B","0/3",         // 19
		"Wait B","1/3",         // 20
		"Wait B","2/3",         // 21
		"Wait B","3/3"          // 22
};

    
    d = &dispState;

	/* State */
	str = NULL;
	str2 = NULL;
	nl = 0;

	if(d->statusFlags & SF_STARTUP){
		str = "Boot";
	} else if(d->status >= 0 && d->status <= 22) {
    	str = statusLabels[2*d->status];
    }

	if(str != NULL) {
		str2 = statusLabels[2*d->status + 1];
	} else {
    	str = "Null";
    }


	if(statusChanged())
		putchar('\n');

	if (d->Vset_uV == 0.0f) {
		sprintf(voltage, "    0.00");
	} else {
	    sprintf(voltage, "%5.2f", d->Vset_uV);
	}
	// The optut: 
	//     GPS time, # Sat, Voltage, dV, Phase, Offset, PPS Status, PDOP, Locked time, Status
	printf("%s %8s GPS, NS %2d, Vo %s dV %5.2f, Ph %4d of %3d, PPS %1d, DOP %5.2f, Lck %4d, St %2d: %s %s\r",
	    LockOn,
    	convert_gps_time(),
    	d->NumSats,
    	voltage,
    	(lastvoltage?d->Vset_uV - lastvoltage:0),
    	d->PPSPhase,
    	d->PPSOffset,
    	d->PPSStatus,
    	d->DOP,
    	(d->LockStartClock?d->Clock - d->LockStartClock:0),
    	d->status,
    	str,
    	str2
	);
	    
		lastvoltage = d->Vset_uV;
        fflush(stdout);
}


void TruePosReadBuffer() {
	int stop=0;
	char x;
	do {
		// During boot, messages may be delayed by about 8 seconds.
               if((x = UART_Rx()) != EOF) {
			if(cmdBufLen == 0 && x != '$') {
			} else if (x == '\r' || x == '\n') {
				cmdBuf[cmdBufLen] = '\r';
				cmdBuf[cmdBufLen+1] = '\n';
				cmdBuf[cmdBufLen+2] = '\0';
				noteTx(cmdBuf);
				HandleCommand();
				cmdBufLen = 0;

				dispState.statusFlags |= SF_GPSDO_CONNECTED;
			} else if (cmdBufLen < (CMDBUF_LEN-4)) {
				cmdBuf[cmdBufLen] = x;
				cmdBufLen++;
			} else {
				// Too long, something bad happened
				cmdBufLen = 0;
			}
		} else {
			stop=1;
			dispState.statusFlags &= ~SF_GPSDO_CONNECTED;
			ppsdbgReceived=0;;
			displayRequestRefresh();
			putchar('\n');
		}
	} while(!stop);
}

static void noteTx(char *msg) {
//  Log implementation 
//	printf("%s", msg);
}

static void HandleCommand() {
	if(!strncmp(cmdBuf, RSP_CLOCK, sizeof(RSP_CLOCK)-1)) {
		dispState.statusFlags &= ~SF_STARTUP;
		clockNoPPSDBG++;
		if(clockNoPPSDBG >= 3) {
			clockNoPPSDBG = 0;
			ppsdbgReceived = 0;
			noteTx(INFO_PPSDBG1);
			UART_Tx("$PPSDBG 1\r\n");
		}
		HandleClockMsg();
		if(!ppsdbgReceived)
			displayRequestRefresh();
	} else if(!strncmp(cmdBuf, RSP_PPSDBG, sizeof(RSP_PPSDBG)-1)) {
		HandlePPSDbgMsg();
		ppsdbgReceived=1;
		displayRequestRefresh();
	}  else if(!strncmp(cmdBuf, RSP_STATUS, sizeof(RSP_STATUS)-1)) {
		HandleStatusMsg();
		if(!ppsdbgReceived)
			displayRequestRefresh();
	} else if(!strncmp(cmdBuf, RSP_EXTSTATUS, sizeof(RSP_EXTSTATUS)-1)) {
		HandleExtStatusMsg();
	} else if(!strncmp(cmdBuf, RSP_SURVEY, sizeof(RSP_SURVEY)-1)) {
		HandleSurveyMsg();
	}else if(!strncmp(cmdBuf, RSP_SETPOS, sizeof(RSP_SETPOS)-1)) {
		if(dispState.statusFlags & SF_STARTUP) {
			ppsdbgReceived = 0;
			noteTx(INFO_PPSDBG1);
			UART_Tx("$PPSDBG 1\r\n");
		}
	} else if(!strncmp(cmdBuf, RSP_GETVER, sizeof(RSP_GETVER)-1)) {
		if(strstr(cmdBuf, "BOOT") != NULL) {
			noteTx(INFO_PROCEED);
			UART_Tx("$PROCEED\r\n");
			dispState.statusFlags |= SF_STARTUP;
			dispState.status = 5;  // Startup
			dispState.Clock = 0;
			dispState.NumSats = 0;
			dispState.SurveyEndClock = 0;
			dispState.Temp = 0.0f;
			dispState.UTCOffset = 0;
			dispState.Vset_uV = 0.0f;
			dispState.PPSPhase = 0;
			dispState.PPSOffset = 0;
			dispState.TRAIM = 0;
			dispState.DOP = 0.0f;
			dispState.statusFlags &= ~(SF_BAD_10M | SF_BAD_ANTENNA | SF_BAD_PPS | SF_SURVEY);
			displayRequestRefresh();
		}
	} else if(!strncmp(cmdBuf, RSP_KALDBG, sizeof(RSP_KALDBG)-1)) {
	} else if(!strncmp(cmdBuf, RSP_SAT, sizeof(RSP_SAT)-1)) {
	} else if(!strncmp(cmdBuf, RSP_WSAT, sizeof(RSP_WSAT)-1)) {
	} else {
		// Need to ALWAYS let a \0 be at the end, due to multithreading
		strncpy(dispState.LastMsg, cmdBuf, LastMsg_LEN-1);
		dispState.LastMsg[LastMsg_LEN-1] = '\0';
		dispState.LastMsg[LastMsg_LEN-2] = '\0';
		int16_t i = strlen(dispState.LastMsg)-1;
		while(i>=0 && (dispState.LastMsg[i] == '\r' || dispState.LastMsg[i] == '\n')) {
			dispState.LastMsg[i] = '\0';
			i--;
		}
	}
}


/*

 $PPSDBG 1187153266 3 25.28081e3 -253 -6 2 2 0.0
 $PPSDBG 2 0.0 [Fewer parmaters when in holdover?]
 1: same as clock (GPS Time), FURUNO does not back up the date after Sept. 2022
 2: Same as $STATUS status, but updates much more often (and seems to skip
    states less often)

 3: Floating point number. DAC set value. Tends towards 29e3 on my board.
         Proportional to the DAC voltage On my RevC CTS board, 
         Vbias ~= 6.25e-5*PPS3. This may make sense for a 4.096 V reference:
         4.096/2^16=6.25e-5 During startup, it is not put in the result string
         (this field is blank, so two sequential space characters are in the
         string)
 4: Measured phase offset? Units seem something like 6.5*ns
 5: PPS offset from $PFEC,GPrrm message - range from -15 to +14
 6: PPS status from $PFEC,GPrrm message
 7: TRAIM status from $PFEC,GPrrm message 
 8: Always 0.0 - temperature on 12.1.1 firmware

*/

static void HandlePPSDbgMsg() {
	clockNoPPSDBG = 0;
	char *t;
	char *saveptr;
	float VsetF;
	int PPSPhase;
	int PPSOffset;
	int TRAIM;
	int i=0;
	//static long cl;
	if(!LOCSent) {
		/*
		static char* setpos = "$SETPOS X Y Z\r\n";
		UART_Tx((uint8_t*)setpos,strlen(setpos),100);
		*/
		LOCSent = 1;
	}
	t = strtok_r(cmdBuf, " \r\n",&saveptr);
	while(t != NULL) {
		switch(i) {
		case 1: // clock
			//cl = strtol(t,NULL,10);
			break;
		case 2: // status code
			HandleStatusCode(atoi(t));
			break;
		case 3: // Vset
			VsetF = strtof(t,NULL);
			dispState.Vset_uV = VsetF /* * 6.25e1f */; // uV as is reported
			break;
		case 4: // PPS Phase
			dispState.PPSPhase = strtol(t, NULL, 10);
			break;
		case 5: // PPS Offset
			dispState.PPSOffset = strtol(t, NULL, 10);
			break;
		case 6: // PPS Status
			dispState.PPSStatus = strtol(t, NULL, 10);
			break;
		case 7: // TRAIM
			dispState.TRAIM = strtol(t, NULL, 10);
			break;
		}
		i++;
		t = strtok_r(NULL, " \r\n",&saveptr);
	}
}

/*

 $EXTSTATUS
 1: SurveyStatus [0=normal, 1=surveying]
 2: Number of sats used for positioning, copied from $GPGGA message 
 3: HDOP if 2Dfix, PDOP if 3D fix, copied from $GPGGA message
 4: Temperature (close to FPGA? close to oven?) (my board reads about 45C)
 5: gps discard counter - error related ?

*/

static void HandleExtStatusMsg() {
	char *t;
	char *saveptr;
	int i=0;
	t = strtok_r(cmdBuf, " \r\n",&saveptr);
	while(t != NULL) {
		switch(i) {
		case 1: // survey?
			if(t[0] != '0')
				dispState.statusFlags |= SF_SURVEY;
			else
				dispState.statusFlags &= ~SF_SURVEY;
			break;
		case 2: // NSats
			dispState.NumSats = strtoul(t,NULL,10);
			break;
		case 3: // DOP
			dispState.DOP = strtof(t,NULL);
			break;
		case 4: // Temperature
			dispState.Temp = strtof(t,NULL);
			break;
		}
		i++;
		t = strtok_r(NULL, " \r\n",&saveptr);
	}
}

/*

 $CLOCK 1187156731 18 3
 1: GPS timestamp (secs since 1970), FURUNO does not back up the date after Sept. 2022 
 2: Count of leap-seconds
 3: Time figure-of-merit (1=good, 7=bad)

*/

static void HandleClockMsg() {
	char *t;
	char *saveptr;
	int i=0;
	t = strtok_r(cmdBuf, " \r\n",&saveptr);
	while(t != NULL) {
		switch(i) {
		case 1: // GPS time, FURUNO does not back up the date after Sept. 2022
			dispState.Clock = strtoul(t,NULL,10);
			break;
		case 2: // leap-seconds
			dispState.UTCOffset = strtoul(t,NULL,10);
			break;
		}
		i++;
		t = strtok_r(NULL, " \r\n",&saveptr);
	}
}


/*

 $SURVEY 40448488 -86915296 225 -34 7129
 [sent during a survey]
 1: Latitude
 2: Longitude
 3: Elevation_MSL
 4: Correction to MSL to get WGS elevation (add this value to MSL to get WGS ellipsoid)
 5: Number of seconds remaining

*/
static void HandleSurveyMsg() {
	char *t;
	char *saveptr;
	int i=0;
	t = strtok_r(cmdBuf, " \r\n",&saveptr);
	while(t != NULL) {
		switch(i) {
		case 5: // Remaining time
			dispState.SurveyEndClock = dispState.Clock + strtoul(t,NULL,10);
			break;
		}
		i++;
		t = strtok_r(NULL, " \r\n",&saveptr);
	}
}


/*

 $STATUS 
 1: (Maybe 10 MHz bad, based on packrat docs)
 2: (Maybe PPS bad, based on packrat docs)
 3: Antenna is bad? 0=good
 4: Holdover duration (secs)
 5: Number of sats tracked (different than, but within 2 of $EXTSTATUS, perhaps only counts channels 0-7???, range is 0-8)
 Status [Locked = 0, Recovery = 1, (Forced holdover?)=3, Train OXCO=7, Holdover = 8,
        [Startup A/B/C/D = 10/11/2/19 ]
        [ (transition from 1 to 0) = (14,15,16,17,18) ] Wait states when transitioning
        [ (transition from 0 to 1) = (20,21,22) ]  Wait states when transitioning
        [  (6 = locked, but unknown location????)

		"Locked",NULL,          // 0
		"Recovery",NULL,        // 1
		"Startup","5/5",        // 2 (Initialization?)
		"Holdover","(SET1PPS)", // 3 (From $SET1PPS?)
		"Forced","Holdover",    // 4 (Forced Holdover?)
		"Soft","Holdover",      // 5 (Soft Holdover?)
		"Unknown","Location",   // 6
		"OCXO","Training",      // 7 (OCXO Training?)
		"Holdover","Recovery",  // 8
		"Startup","0/5",        // 9
		"Startup","1/5",        // 10
		"Startup","2/5",        // 11
		"Startup","3/5",        // 12
		"Startup","4/5",        // 13
		"Wait A","0/4",         // 14
		"Wait A","1/4",         // 15
		"Wait A","2/4",         // 16
		"Wait A","3/4",         // 17
		"Wait A","4/4",         // 18
		"Wait B","0/3",         // 19
		"Wait B","1/3",         // 20
		"Wait B","2/3",         // 21
		"Wait B","3/3"          // 22

*/

static void HandleStatusMsg() {
	clockNoPPSDBG = 0;
	char *t;
	char *saveptr;
	uint8_t x;
	int i=0;
	t = strtok_r(cmdBuf, " \r\n",&saveptr);
	while(t != NULL) {
		switch(i) {
		case 1: // bad 10MHZ
			if(t[0] != '0')
				dispState.statusFlags |= SF_BAD_10M;
			else
				dispState.statusFlags &= ~SF_BAD_10M;
			break;
		case 2: // bad antenna
			if(t[0] != '0')
				dispState.statusFlags |= SF_BAD_PPS;
			else
				dispState.statusFlags &= ~SF_BAD_PPS;
			break;
		case 3: // bad antenna
			if(t[0] != '0')
				dispState.statusFlags |= SF_BAD_ANTENNA;
			else
				dispState.statusFlags &= ~SF_BAD_ANTENNA;
			break;
		case 6:
			x = atoi(t);
			HandleStatusCode(x);
			break;
		}
		i++;
		t = strtok_r(NULL, " \r\n",&saveptr);
	}
}
static void HandleStatusCode(uint8_t code) {
	dispState.status = code;
	if(code==0) {
		LockOn = "LOCK:";
		if(dispState.LockStartClock == 0)
			dispState.LockStartClock = dispState.Clock;
	}else {
		LockOn = " CLK:";
		dispState.LockStartClock = 0;
	}
}

