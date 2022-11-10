/* USER CODE BEGIN Includes */
#include <stdio.h>
#include <stddef.h>
#include <stdint.h>
#include <unistd.h>

#include "truepos.h"
/* USER CODE END Includes */

dispState_struct dispState;
UART_HandleTypeDef TP_UART;

#define TP_UART_ID 1

/* USER CODE END 0 */

int main(void)
{

  /* Infinite loop */
    for(;;)
    {
        TruePosInit(&TP_UART, TP_UART_ID);
        TruePosReadBuffer();
        TruePosStop();
        sleep(1);		// Just make sure UART is settled
    }
}
