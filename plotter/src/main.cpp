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

    qCommand = xQueueCreate(1, sizeof(Command));

    xTaskCreate(vReceiveTask, "Receive Task", configMINIMAL_STACK_SIZE * 3, NULL,
            (tskIDLE_PRIORITY + 1UL), (TaskHandle_t *) NULL);

    xTaskCreate(vExecuteTask, "Execute Task", configMINIMAL_STACK_SIZE * 5, NULL,
            (tskIDLE_PRIORITY + 2UL), (TaskHandle_t *) NULL);

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
    Chip_RIT_Init(LPC_RITIMER);

    // set the priority level of the interrupt
    NVIC_SetPriority(RITIMER_IRQn, configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY + 1);

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

void vReceiveTask(void *vParameters) {
    vTaskDelay(configTICK_RATE_HZ);

    GParser parser;
    char message[] = "OK\n";

    while (1) {
        /* get GCode from mDraw */
        char buffer[RCV_BUFSIZE] = {'0'};
        int idx = 0;
        while (1) {
            int len = USB_receive((uint8_t *) (buffer + idx), RCV_BUFSIZE);

            char *pos = strstr((buffer + idx), "\n");// find '\n' <=> end of command

            if (pos != NULL) {
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

                break;
            }
            idx += len;
        }
    }
}

void vExecuteTask(void *vParameters) {
    vTaskDelay(configTICK_RATE_HZ);

    Command recv;

    while (1) {
        if (xQueueReceive(qCommand, &recv, portMAX_DELAY) == pdTRUE) {
            switch (recv.type) {
            case Command::connected:
                {
                    char buffer[48];

                    xymotor->isCalibrating = true;
                    pen->moveServo(xyconfig.pen_up);
//                    laser->changeLaserPower(0);
                    xymotor->calibrate();

                    xyconfig.length_x = xyconfig.length_y * xymotor->getTotalStepX() / xymotor->getTotalStepY();
                    xymotor->SetXStepInMM(xyconfig.length_x);
                    xymotor->SetYStepInMM(xyconfig.length_y);
                    xyconfig.last_x_pos = 0;
                    xyconfig.last_y_pos = 0;

                    // M10 XY 380 310 0.00 0.00 A0 B0 H0 S50 U130 D90
                    snprintf(buffer, 48, "M10 XY %d %d %.2f %.2f A0 B0 H0 S%d U%d D%d \n",
                            xyconfig.length_x, xyconfig.length_y, xyconfig.last_x_pos, xyconfig.last_y_pos,
                            xyconfig.speed, xyconfig.pen_up, xyconfig.pen_down);

                    USB_send((uint8_t *) buffer, strlen(buffer));
                }
                break;
            case Command::laser:
//                laser->changeLaserPower(recv.params[0]);
                break;
            case Command::move:
                xymotor->move(recv.params[0], recv.params[1]);

                xyconfig.last_x_pos = recv.params[0];
                xyconfig.last_y_pos = recv.params[1];

                vTaskDelay(recv.params[3]);
                break;
            case Command::pen_position:
                pen->moveServo(recv.params[0]);

                vTaskDelay(configTICK_RATE_HZ / 2);

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
                xymotor->move(0, 0);

                xyconfig.last_x_pos = 0;
                xyconfig.last_y_pos = 0;

                break;
            case Command::done:
            default:
                break;
            }
        }

    }
}

/* the following is required if runtime statistics are to be collected */
extern "C" {

void RIT_IRQHandler(void) {
    if (xymotor->isCalibrating) {
        portEND_SWITCHING_ISR(xymotor->irqHandlerCalibration());
    } else {
        portEND_SWITCHING_ISR(xymotor->irqHandler());
    }

}

void vConfigureTimerForRunTimeStats(void) {
    Chip_SCT_Init(LPC_SCTSMALL1);
    LPC_SCTSMALL1->CONFIG = SCT_CONFIG_32BIT_COUNTER;
    LPC_SCTSMALL1->CTRL_U = SCT_CTRL_PRE_L(255) | SCT_CTRL_CLRCTR_L; // set prescaler to 256 (255 + 1), and start timer
}

}
/* end runtime statistics collection */
