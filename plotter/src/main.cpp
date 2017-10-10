#if defined (__USE_LPCOPEN)
#if defined(NO_BOARD_LIB)
#include "chip.h"
#else
#include "board.h"
#endif
#endif

#include <cr_section_macros.h>

#include "string.h"
#include "math.h"

#include "FreeRTOS.h"
#include "user_vcom.h"
#include "task.h"
#include "queue.h"

#include "GParser.h"
#include "Command.h"
#include "Config.h"
#include "XYMotor.h"
#include "Laser.h"
#include "Servo.h"


void setupHardware();

/* Tasks Declaration */
void vReceiveTask(void *vParameters);
void vExecuteTask(void *vParameters);
void vCalibrateTask(void *vParameters);

DigitalIoPin* dirXPin;
DigitalIoPin* stepXPin;
DigitalIoPin* dirYPin;
DigitalIoPin* stepYPin;
DigitalIoPin* lmXMin;
DigitalIoPin* lmXMax;
DigitalIoPin* lmYMin;
DigitalIoPin* lmYMax;
XYMotor *xymotor;
Servo *pen;
Laser *laser;

XYSetup xyconfig;
QueueHandle_t qCommand;

int main(void) {

    setupHardware();

    qCommand = xQueueCreate(10, sizeof(Command));

    xTaskCreate(vReceiveTask, "Receive Task", configMINIMAL_STACK_SIZE * 3, NULL,
            (tskIDLE_PRIORITY + 1UL), (TaskHandle_t *) NULL);

    xTaskCreate(vExecuteTask, "Execute Task", configMINIMAL_STACK_SIZE * 3, NULL,
            (tskIDLE_PRIORITY + 1UL), (TaskHandle_t *) NULL);

    xTaskCreate(vCalibrateTask, "Calibrate Task", configMINIMAL_STACK_SIZE,
            NULL, (tskIDLE_PRIORITY + 2UL), (TaskHandle_t *) NULL);

    xTaskCreate(cdc_task, "CDC Task", configMINIMAL_STACK_SIZE * 2, NULL,
            (tskIDLE_PRIORITY + 1UL), (TaskHandle_t *) NULL);

    vQueueAddToRegistry(qCommand, "Queue Command");

    vTaskStartScheduler();

    while (1);

    return 0;
}

void setupHardware() {
    SystemCoreClockUpdate();
    Board_Init();

    ITM_init();

    Board_LED_Set(0, false);

    dirXPin = new DigitalIoPin(motorXDirPort, motorXDirPin, DigitalIoPin::output, false);
    stepXPin = new DigitalIoPin(motorXPort, motorXPin, DigitalIoPin::output, false);
    dirYPin = new DigitalIoPin(motorYDirPort, motorYDirPin, DigitalIoPin::output, false);
    stepYPin = new DigitalIoPin(motorYPort, motorYPin, DigitalIoPin::output, false);
    lmXMin = new DigitalIoPin(limitXMinPort, limitXMinPin , DigitalIoPin::pullup, true);
    lmXMax = new DigitalIoPin(limitXMaxPort, limitXMaxPin , DigitalIoPin::pullup, true);
    lmYMin = new DigitalIoPin(limitYMinPort, limitYMinPin , DigitalIoPin::pullup, true);
    lmYMax = new DigitalIoPin(limitYMaxPort, limitYMaxPin , DigitalIoPin::pullup, true);

    xymotor = new XYMotor(dirXPin, stepXPin, dirYPin, stepYPin, lmXMin, lmXMax, lmYMin, lmYMax);
    pen = new Servo(penPort, penPin);
//    laser = new Laser();
//    laser->changeLaserPower(0); // drive laser low
}

/***** Task Definition *****/

void vCalibrateTask(void *vParameters) {
    /* Calibrating */
    int xstep = calibrateMotor(stepXPin, dirXPin, lmXMin, lmXMax);
    int ystep = calibrateMotor(stepYPin, dirYPin, lmYMin, lmYMax);;

    xymotor->setBaseX(xyconfig.length_x);
    xymotor->setBaseY(xyconfig.length_y);
    xymotor->setTotalStepX(xstep);
    xymotor->setTotalStepY(ystep);
    xyconfig.last_x_pos = 0;
    xyconfig.last_y_pos = 0;

    /* Calibration done */
    vTaskDelete(NULL);
}

void vReceiveTask(void *vParameters) {

    GParser parser;
    char message[] = "OK\n";

    while (1) {
        /* get GCode from mDraw */
        char buffer[RCV_BUFSIZE];
        int idx = 0;
        while (1) {
            int len = USB_receive((uint8_t *) (buffer + idx), RCV_BUFSIZE);

            char *pos = strstr((buffer + idx), "\n");// find '\n' <=> end of command

            if (pos != NULL)
                break;

            idx += len;

            vTaskDelay(1); // polling every 1 ms
        }

        ITM_write(buffer);

        /* parse GCode */
        Command gcode = parser.parse(buffer, strlen(buffer));

        /* send commands into queue */
        if (gcode.type != Command::invalid) {
            xQueueSendToBack(qCommand, &gcode, portMAX_DELAY);

            /* send `OK` message to mDraw */
            if (gcode.type != Command::connected)
                USB_send((uint8_t *) message, strlen(message));
        }
    }
}

void vExecuteTask(void *vParameters) {

    Command recv;

    while (1) {
        /* wait for commands from queue */
        if (xQueueReceive(qCommand, &recv, configTICK_RATE_HZ) == pdTRUE) {
            /* do something useful*/
            switch (recv.type) {
            case Command::connected:
                {
//                    char buffer[48];
//                    snprintf(buffer, 48, "M10 XY %d %d 0.00 0.00 A0 B0 H0 S%d U%d D%d \n",
//                             XYSetup.length_x, XYSetup.length_y, XYSetup.speed, XYSetup.pen_up, XYSetup.pen_down);

                    char buffer[] = "M10 XY 380 310 0.00 0.00 A0 B0 H0 S50 U130 D90 \n";
                    USB_send((uint8_t *) buffer, strlen(buffer));
                }
                break;
            case Command::laser:
//                laser->changeLaserPower(recv.params[0]);
                break;
            case Command::move:
                // draw at a constant speed
                xymotor->move(xyconfig.last_x_pos, xyconfig.last_y_pos,
                              recv.params[0], recv.params[1], motorPps);

                xyconfig.last_x_pos = recv.params[0];
                xyconfig.last_y_pos = recv.params[1];

                break;
            case Command::pen_position:
                pen->moveServo(recv.params[0]);

                break;
            case Command::pen_setting:
                xyconfig.pen_up = recv.params[0];
                xyconfig.pen_down = recv.params[1];

                break;
            case Command::plotter_setting:
                break;
            case Command::to_origin:
                pen->moveServo(xyconfig.pen_up);
                // laser->changeLaserPower(0);
                xymotor->move(xyconfig.last_x_pos, xyconfig.last_y_pos, 0, 0, motorPps);

                xyconfig.last_x_pos = 0;
                xyconfig.last_y_pos = 0;

                break;
            case Command::done:
            case Command::invalid:
            default:
                break;
            }
        }

    }
}

/* the following is required if runtime statistics are to be collected */
extern "C" {

void RIT_IRQHandler(void) {
    portEND_SWITCHING_ISR(xymotor->irqHandler());
}

void vConfigureTimerForRunTimeStats(void) {
    Chip_SCT_Init(LPC_SCTSMALL1);
    LPC_SCTSMALL1->CONFIG = SCT_CONFIG_32BIT_COUNTER;
    LPC_SCTSMALL1->CTRL_U = SCT_CTRL_PRE_L(255) | SCT_CTRL_CLRCTR_L; // set prescaler to 256 (255 + 1), and start timer
}

}
/* end runtime statistics collection */
