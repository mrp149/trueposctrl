/* USER CODE BEGIN Includes */
#include <stdio.h>
#include <stddef.h>
#include <stdint.h>

#include "truepos.h"
/* USER CODE END Includes */

dispState_struct dispState;
UART_HandleTypeDef TP_UART;

#define TP_UART_ID 1

/* USER CODE END 0 */

int main(void)
{

  /* USER CODE BEGIN RTOS_QUEUES */
    TruePosInit(&TP_UART, TP_UART_ID);
  /* USER CODE END RTOS_QUEUES */
 
  /* Infinite loop */
  /* USER CODE BEGIN WHILE */

    for(;;)
    {
        TruePosReadBuffer();
        return 0;
    }

}
