#if defined (__USE_LPCOPEN)
#if defined(NO_BOARD_LIB)
#include "chip.h"
#else
#include "board.h"
#endif
#endif

#define DEBUG

#include <cr_section_macros.h>

#include "user_vcom.h"
#include "Laser.h"
#include "Servo.h"
#include "StepperMotor.h"
#include "Command.h"

#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"

void setupHardware();

/* Task Declaration */
void vReceiveTask(void *vParameters);
void vExecuteTask(void *vParameters);
void vCalibrateTask(void *vParameters);

/* Motor */
StepperMotor *xmotor;
StepperMotor *ymotor;

QueueHandle_t qCommand;

int main(void) {

    setupHardware();

    qCommand = xQueueCreate(10, sizeof(Command));

    xTaskCreate(vReceiveTask, "Receive Task", configMINIMAL_STACK_SIZE,
                NULL, (tskIDLE_PRIORITY + 1UL), (TaskHandle_t *) NULL);

    xTaskCreate(vExecuteTask, "Execute Task", configMINIMAL_STACK_SIZE,
                NULL, (tskIDLE_PRIORITY + 1UL), (TaskHandle_t *) NULL);

    xTaskCreate(vCalibrateTask, "Calibrate Task", configMINIMAL_STACK_SIZE,
    			NULL, (tskIDLE_PRIORITY + 2UL), (TaskHandle_t *) NULL);

    xTaskCreate(cdc_task, "CDC Task", configMINIMAL_STACK_SIZE * 2, NULL,
                (tskIDLE_PRIORITY + 1UL), (TaskHandle_t *) NULL);


    vTaskStartScheduler();

    while (1)
        ;

    return 0;
}

void setupHardware() {
    SystemCoreClockUpdate();
    Board_Init();

    Board_LED_Set(0, false);
}

/***** Task Definition *****/

void vCalibrateTask(void *vParameters) {

}

void vReceiveTask(void *vParameters) {

    while (1) {
    	/* get GCode from mDraw */

    	/* parse GCode */

    	/* send commands into queue */

    	/* send OK back */

    }
}

void vExecuteTask(void *vParameters) {

    while (1) {
    	/* wait for commands from queue */


    }
}

/* the following is required if runtime statistics are to be collected */
extern "C" {

/* X axis motor */
void SCT2_IRQHandler(void) {
	portYIELD_FROM_ISR(xmotor->irqHandler());
}

/* Y axis motor */
void SCT3_IRQHandler(void) {
	portYIELD_FROM_ISR(ymotor->irqHandler());
}

void vConfigureTimerForRunTimeStats(void) {
    Chip_SCT_Init(LPC_SCTSMALL1);
    LPC_SCTSMALL1->CONFIG = SCT_CONFIG_32BIT_COUNTER;
    LPC_SCTSMALL1->CTRL_U = SCT_CTRL_PRE_L(255) | SCT_CTRL_CLRCTR_L; // set prescaler to 256 (255 + 1), and start timer
}

}
/* end runtime statistics collection */
